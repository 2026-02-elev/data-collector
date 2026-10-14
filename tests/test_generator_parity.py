# Synthetic vs measured pipeline parity: the generator's virtual UART stream through the real collector
# Author: Yongho Ahn <nanhosoft@gmail.com>
"""For every run in the data-generator's demo set, ``nano_uart.log`` is replayed
through this collector and the result is compared with the generator's own
``observations.jsonl`` / ``events.jsonl``. A model trained on synthetic records
must find the same fields, with the same meaning, in measured records.

Skipped when the generator checkout is not available
(CELLGUARD_GENERATOR_DIR or ../data-generator).
"""
import collections
import datetime as dt
import json
import os
from pathlib import Path

import pytest

from cg_collector.fusion import FusionContext
from cg_collector.replay import read_generator_log, replay

ROOT = Path(__file__).resolve().parents[1]
GEN = Path(os.environ.get("CELLGUARD_GENERATOR_DIR", ROOT.parent / "data-generator")) / "demo_outputs" / "runs"
RUNS = sorted(p for p in GEN.iterdir() if p.is_dir()) if GEN.exists() else []

EXACT = ["record_status", "telemetry_seq", "nano_millis", "nano_boot_epoch", "cell_voltage_v",
         "cumulative_tap_voltage_v", "cell_temperature_c", "temperature_quality", "nano_adc_tap_raw",
         "nano_adc_ntc_raw", "charge_commanded", "discharge_commanded", "pack_relay_commanded",
         "pack_relay_commanded_known", "command_state_source", "esr_command_mask", "leak_command_mask",
         "heater_pwm_command", "trip_reason", "trip_code", "controller_state", "controller_locks", "fault_commanded",
         "voltage_quality", "voltage_quality_bits", "temperature_quality_mask", "fault_physical_observed",
         "physical_fault_verified", "protocol_version", "schema_version"]
# Request tracking: the generator knows the true Nano event time, the collector estimates it from the
# Nano clock (a few ms off), so a request change that falls within ~50 ms of a slot may land on either side.
REQUEST = ["fault_requested", "power_requested"]
BOUNDARY_S = 0.05
SYNTHETIC_ONLY = {"generator_version", "seed", "config_hash", "hardware_operable", "fault_modeled_applied"}
MEASURED_ONLY = {"firmware_version", "hardware_revision", "raw_log_ref"}
MODEL_EVENTS = {"APPLIED", "RELEASED", "OBSERVABLE"}


def _ts(text):
    return None if text is None else dt.datetime.fromisoformat(text.replace("Z", "+00:00"))


def _same_ended(a, b, tol_s=0.05):
    if a is None or b is None:
        return a == b
    keys = ("type", "cell_id", "reason")
    if any(a[k] != b[k] for k in keys):
        return False
    return all(abs((_ts(a[k]) - _ts(b[k])).total_seconds()) <= tol_s for k in ("requested_since_utc", "ended_utc"))


def _replay(run):
    man = json.loads((run / "manifest.json").read_text())
    epoch = dt.datetime.fromisoformat(man["fixed_epoch_utc"].replace("Z", "+00:00"))
    obs, events = [], []
    col = replay(read_generator_log(run / "nano_uart.log", epoch), FusionContext(run_id=run.name), obs.append,
                 events.append, run_start=epoch, end_utc=epoch + dt.timedelta(seconds=float(man["duration_s"])))
    return col, obs, events


@pytest.mark.skipif(not RUNS, reason="data-generator demo outputs not available")
@pytest.mark.parametrize("run", RUNS, ids=[r.name for r in RUNS])
def test_records_match_generator(run):
    gen = [json.loads(l) for l in (run / "observations.jsonl").read_text().splitlines()]
    _, obs, _ = _replay(run)
    assert len(obs) == len(gen)
    cum = {"g": collections.Counter(), "c": collections.Counter()}
    for g, c in zip(gen, obs):
        assert abs(g["t_sim_s"] - c["t_sim_s"]) <= 0.02, (g["t_sim_s"], c["t_sim_s"])
        for k in EXACT:
            assert g.get(k) == c.get(k), (run.name, g["t_sim_s"], k, g.get(k), c.get(k))
        near_change = _near_change(g, c)
        for k in REQUEST:
            assert g[k] == c[k] or near_change, (run.name, g["t_sim_s"], k, g[k], c[k])
        assert _same_ended(g["fault_request_last_ended"], c["fault_request_last_ended"]) or near_change, g["t_sim_s"]
        # transport errors: each counted once; one may be attributed to the neighbouring record
        cum["g"].update(g["transport"])
        cum["c"].update(c["transport"])
        assert all(abs(cum["g"][k] - cum["c"][k]) <= 1 for k in cum["g"]), (g["t_sim_s"], cum)
        # same field set apart from provenance that only one side can have
        assert set(g) - SYNTHETIC_ONLY - MEASURED_ONLY - {"scenario_domain"} <= set(c), set(g) - set(c)
    assert cum["g"] == cum["c"]


def _near_change(g, c):
    """A request change (start / end) of either side within BOUNDARY_S of this slot."""
    t = _ts(g["timestamp_utc"])
    for rec in (g, c):
        ended = rec["fault_request_last_ended"] or {}
        stamps = [ended.get("requested_since_utc"), ended.get("ended_utc")]
        stamps += [f.get("since_utc") for f in ((rec["fault_requested"] or {}).get("faults") or [])]
        if any(x and abs((_ts(x) - t).total_seconds()) <= BOUNDARY_S for x in stamps):
            return True
    return False


@pytest.mark.skipif(not RUNS, reason="data-generator demo outputs not available")
@pytest.mark.parametrize("run", RUNS, ids=[r.name for r in RUNS])
def test_events_match_generator(run):
    def key(e):
        cge = e["evidence"].get("cge") or {}
        return (e["event_type"], e["fault_type"], e["cell_id"], e["requested_value"], e["commanded_value"],
                cge.get("cge_code"), cge.get("reason"))

    gen = [json.loads(l) for l in (run / "events.jsonl").read_text().splitlines()]
    # host-observable events only: model-only events and scenario injections (SENSOR/UART/I2C/AREF,
    # MCU_RESET, PARAM) are not something a Pi can observe
    gen = [e for e in gen if e["event_type"] not in MODEL_EVENTS and e["fault_type"] in ("NONE", "ESR", "LEAK", "HEAT")
           and e["requested_value"] != "MCU_RESET" and not isinstance(e["requested_value"], dict)]
    _, _, events = _replay(run)
    g, c = collections.Counter(map(key, gen)), collections.Counter(map(key, events))
    assert not (c - g), f"collector-only events: {c - g}"
    lost = sum((e["evidence"].get("cge") or {}).get("events_lost_before", 0) for e in events)
    # the generator logs the virtual Nano's own events; the Pi misses those whose CGE was lost
    assert sum((g - c).values()) <= lost, (g - c, lost)
