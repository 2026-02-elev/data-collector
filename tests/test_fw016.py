# Firmware 0.1.6-LOAD-VERIFY compatibility: capabilities, RAMP/DRAIN cycle, CGY/CGX, aborts, trips, link loss
# Author: Yongho Ahn <nanhosoft@gmail.com>
"""Runs against tests/fw016/sim016: the UNMODIFIED rig firmware controller (CellGuardNano_LoadVerify
0.1.6, SHA256SUMS) compiled for the PC behind a plant model. SIMULATED DEVICE - no hardware involved."""
import importlib.util
import json
from pathlib import Path

import jsonschema
import pytest

from cg_collector import cli
from cg_collector import firmware as fw
from cg_collector.cycle import CycleAborted, checkpoints, verify_voltage
from cg_collector.fusion import FusionContext
from cg_collector.preflight import UNSUPPORTED, WARN, Check, Report
from cg_collector.protocol import LoadTapSuspect, TripSnapshot, Version, crc16_ccitt_false, parse_message
from cg_collector.replay import replay
from cg_collector.session import NanoSession, UnsupportedCommand
from cg_collector.transport import SimTransport

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = ROOT / "tests" / "fw016" / "CellGuardNano_LoadVerify" / "cellguard_cycle.py"
# bench capture (2026-10-25, 400 mV charge then 300 mV discharge): on-load taps that looked inverted
BENCH_CGY = "CGY,215547,2,42,83,73,114,352,696,612,955,273"


def fr(payload):
    return f"${payload}*{crc16_ccitt_false(payload.encode()):04X}"


def rows(data_dir, kind):
    return [json.loads(l) for f in sorted((Path(data_dir) / "outbox").glob(f"{kind}-*.jsonl"))
            for l in f.read_text().splitlines()]


def sent_commands(data_dir):
    out = []
    for r in rows(data_dir, "raw"):
        if r.get("dir") == "P>N":
            f = r["frame"][1:r["frame"].index("*")].split(",")
            out.append(",".join(f[2:]))
    return out


def run_cli(monkeypatch, sim, args, controls=(), transport=SimTransport):
    def make(_a):
        t = transport(sim)
        for c in controls:
            t.control(c)
        return t

    monkeypatch.setattr(cli, "_transport", make)
    return cli.main(["--sim", sim, *args])


# ---------------------------------------------------------------- versions and capabilities

def test_capabilities_per_firmware():
    lv = fw.capabilities(Version("0.1.6-LOAD-VERIFY", "1.0", "PCB2-20260918-R2", 0))
    assert lv.family == "load_verify_0_1_6" and lv.staged_power and not lv.self_test and lv.diagnostic_frames
    assert lv.discharge_events == ("DRAIN_START", "DRAIN_STOP") and lv.stage_disarms
    v2 = fw.capabilities(Version("0.2.0", "1.0", "PCB2-20260918-R2", 0))
    assert v2.family == "v0_2" and v2.self_test and not v2.staged_power and "SELFTEST" in v2.commands
    v1 = fw.capabilities(Version("0.1.0", "1.0", "PCB2-20260918-R2", 0))
    assert v1.family == "v0_1" and not v1.self_test
    unk = fw.capabilities(Version("9.9-X", "1.0", "x", 0))
    assert not unk.known and unk.commands == fw.READ_ONLY
    for name, args in (("CHARGE", [1]), ("DISCHARGE", [1]), ("SELFTEST", []), ("ESR", [1, 1]), ("HEAT", [1, 40]),
                       ("RAMP", [1001]), ("RAMP", [149]), ("DRAIN", [99])):
        with pytest.raises(fw.UnsupportedCommand):
            lv.check(name, args)
    lv.check("CHARGE", [0])   # stopping is always allowed
    lv.check("RAMP", [400])
    with pytest.raises(fw.UnsupportedCommand):
        v2.check("RAMP", [400])
    with pytest.raises(fw.UnsupportedCommand):
        fw.capabilities(None).check("ARM")   # before CGV: read-only only


def test_session_detects_0_1_6_and_never_sends_unsupported(sim016_path):
    t = SimTransport(sim016_path)
    s = NanoSession(t)
    sent = []
    s.on_transmit(lambda _u, f: sent.append(f))
    try:
        with pytest.raises(UnsupportedCommand):
            s.command("ARM")                       # no CGV seen yet: firmware not identified
        assert sent == []
        assert s.command("SYNC").ok
        assert s.version.firmware == "0.1.6-LOAD-VERIFY" and s.capabilities.staged_power
        before = list(sent)
        for name, args in (("CHARGE", (1,)), ("DISCHARGE", (1,)), ("SELFTEST", ()), ("ESR", (1, 1))):
            with pytest.raises(UnsupportedCommand):
                s.command(name, *args)
        assert sent == before                      # nothing reached the wire
        s.pump(1.0, keepalive_s=None)              # leave the boot SELF_TEST
        assert s.command("ARM").ok
        ack = s.command("RAMP", 200)
        assert ack.ok and ack.text == "PENDING"
        assert s.command("DRAIN", 120).text == "CONFLICT"   # one stage at a time (firmware rule)
        assert s.command("OFF").ok
    finally:
        t.close()


# ---------------------------------------------------------------- the cycle

def test_cycle_charge_and_discharge_to_target(sim016_path, tmp_path, monkeypatch, capsys, normalized_schema,
                                              diagnostic_schema):
    rc = run_cli(monkeypatch, sim016_path, ["--data-dir", str(tmp_path), "--run-id", "c1", "cycle", "--charge-to",
                                            "300", "--step", "100", "--discharge-to", "150", "--i-am-supervising"])
    out = capsys.readouterr()
    assert rc == 0, out.err
    assert "Charge plan: 200 -> 300 mV per cell" in out.out and "PASS: charge + discharge cycle complete" in out.out
    cmds = sent_commands(tmp_path)
    stages = [c for c in cmds if c.startswith(("RAMP", "DRAIN"))]
    assert stages == ["RAMP,200", "RAMP,300", "DRAIN,150"]
    for st in stages:  # a new ARM right before every stage
        assert cmds[cmds.index(st) - 1] == "ARM"
    assert "SELFTEST" not in cmds and "CHARGE,1" not in cmds and "RESET" not in cmds
    # 1 s heartbeat while a stage runs
    raw = [r for r in rows(tmp_path, "raw") if r.get("dir") == "P>N"]
    pings = [r["pi_sent_utc"] for r in raw if ",PING*" in r["frame"]]
    assert len(pings) >= 10
    diags = rows(tmp_path, "diagnostics")
    for d in diags:
        jsonschema.validate(d, diagnostic_schema)
    result = [d for d in diags if d["kind"] == "CYCLE_RESULT"][-1]["detail"]
    assert result["outcome"] == "PASS" and [s["outcome"] for s in result["stages"]] == ["PASS"] * 3
    stops = [d["detail"] for d in diags if d["kind"] == "STAGE_STOP"]
    assert [(s["stage"], s["stop_class"]) for s in stops] == [("RAMP", "TARGET_REACHED")] * 2 + [("DRAIN", "TARGET_REACHED")]
    assert stops[-1]["target_mv_per_cell"] == 150
    events = rows(tmp_path, "events")
    req = [e for e in events if e["event_type"] == "REQUESTED" and e["evidence"].get("command") in ("RAMP", "DRAIN")]
    assert [(e["evidence"]["command"], e["requested_value"]) for e in req] == [("RAMP", 200), ("RAMP", 300),
                                                                                ("DRAIN", 150)]
    drain_stop = [e for e in events if e["evidence"].get("cge", {}).get("cge_code") == "DRAIN_STOP"][0]
    assert drain_stop["event_type"] == "COMMANDED" and drain_stop["evidence"]["stage_stop"]["nano_target_reached"]
    samples = rows(tmp_path, "samples")
    for r in samples:
        jsonschema.validate(r, normalized_schema)
        assert r["pack_current_a"] is None and r["current_quality"] == "INA219_UNAVAILABLE"
    active = [r for r in samples if r["power_stage_requested"]]
    assert {r["power_stage_requested"]["command"] for r in active} == {"RAMP", "DRAIN"}
    assert any(r["power_path_reported"]["discharge"] and r["power_requested"]["discharge"] for r in active)


def test_cycle_plan_and_acceptance_match_cellguard_cycle_py():
    spec = importlib.util.spec_from_file_location("cellguard_cycle_ref", REFERENCE)
    ref = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ref)
    for cur in range(-20, 1001, 7):
        for target in (150, 200, 250, 400, 1000):
            for step in (50, 100, 200):
                assert checkpoints(cur, target, step) == ref.checkpoints(cur, target, step)
    assert checkpoints(142, 400, 100) == [200, 300, 400]
    cases = [("RAMP", 200, (126, 134, 134, 142), (192, 200, 200, 208)),
             ("DRAIN", 120, (192, 200, 200, 208), (112, 120, 128, 120)),
             ("RAMP", 300, (200, 200, 200, 200), (301, 301, 301, 200)),
             ("DRAIN", 120, (200, 200, 200, 200), (100, 100, 100, 200)),
             ("DRAIN", 120, (200, 200, 200, 200), (192, 200, 200, 200)),
             ("RAMP", 400, (300,) * 4, (360, 400, 400, 445)),
             ("RAMP", 400, (300,) * 4, (380, 380, 380, 461))]
    for kind, target, before, after in cases:
        ours = theirs = None
        try:
            verify_voltage(kind, target, before, after)
        except CycleAborted as exc:
            ours = str(exc)
        try:
            ref.verify_voltage(kind, target, before, after)
        except ref.Aborted as exc:
            theirs = str(exc)
        assert (ours is None) == (theirs is None), (kind, after, ours, theirs)


def test_load_tap_unstable_aborts_sends_off_and_is_not_a_cell_voltage(sim016_path, tmp_path, monkeypatch, capsys,
                                                                       diagnostic_schema):
    rc = run_cli(monkeypatch, sim016_path, ["--data-dir", str(tmp_path), "cycle", "--charge-to", "200",
                                            "--discharge-to", "120", "--i-am-supervising"],
                 controls=["INJECT LOADED_INVERSION 1"])
    out = capsys.readouterr()
    assert rc == 1
    assert "DRAIN stopped before goal: LOAD_TAP_UNSTABLE" in out.err and "Fail-safe: OFF sent" in out.out
    assert "ON-LOAD SUSPECT: taps" in out.out and "LOAD_TAP_VERIFY: REST_OK" in out.out
    cmds = sent_commands(tmp_path)
    i = cmds.index("DRAIN,120")
    assert "OFF" in cmds[i:]
    assert not [c for c in cmds[i + 1:] if c.startswith(("ARM", "RAMP", "DRAIN", "RESET"))]  # no automatic retry
    diags = rows(tmp_path, "diagnostics")
    for d in diags:
        jsonschema.validate(d, diagnostic_schema)
    cgy = [d for d in diags if d["kind"] == "LOAD_TAP_SUSPECT_SNAPSHOT"]
    assert cgy and cgy[0]["detail"]["rested_cell_voltage"] is False and cgy[0]["detail"]["direction"] == "DISCHARGE"
    assert cgy[0]["detail"]["tap_mv_under_load"][2] == 612 and cgy[0]["frame"].startswith("$CGY,")
    verify = [d["detail"] for d in diags if d["kind"] == "REST_VERIFY"]
    assert verify == [dict(verify[0], reason="REST_OK", rested_voltage_ok=True, stage_successful=False)]
    stop = [d["detail"] for d in diags if d["kind"] == "STAGE_STOP"][-1]
    assert stop["stop_class"] == "ABORTED" and stop["abort_reason"] == "LOAD_TAP_UNSTABLE"
    result = [d["detail"] for d in diags if d["kind"] == "CYCLE_RESULT"][-1]
    assert result["outcome"] == "ABORTED" and "LOAD_TAP_UNSTABLE" in result["reason"]
    ev = [e for e in rows(tmp_path, "events") if e["evidence"].get("cge", {}).get("cge_code") == "DRAIN_STOP"][0]
    assert ev["evidence"]["stage_stop"]["stop_class"] == "ABORTED"
    assert not [e for e in rows(tmp_path, "events") if e["event_type"] == "TRIPPED"]  # no hardware trip
    # the on-load inverted taps (tap 3 = 612 mV below tap 2) never show up as measured voltages
    for r in rows(tmp_path, "samples"):
        taps = r["cumulative_tap_voltage_v"]
        if None not in taps:
            assert taps == sorted(taps) and min(r["cell_voltage_v"]) >= 0
        assert r["transport"]["format_errors_since_last"] == 0 and r["transport"]["crc_errors_since_last"] == 0


def test_real_trip_aborts_without_restart(sim016_path, tmp_path, monkeypatch, capsys, diagnostic_schema):
    # the 2026-10-25 bench sequence: 400 mV charge, then the discharge trips on a tap inversion
    rc = run_cli(monkeypatch, sim016_path, ["--data-dir", str(tmp_path), "cycle", "--charge-to", "400",
                                            "--discharge-to", "300", "--i-am-supervising"],
                 controls=["INJECT LOADED_INVERSION 1", "INJECT REST_INVERSION 1"])
    out = capsys.readouterr()
    assert rc == 1 and "Nano hardware trip: TAP_INVERSION" in out.err, out.err
    cmds = sent_commands(tmp_path)
    i = cmds.index("DRAIN,300")
    assert set(cmds[i + 1:]) <= {"PING", "RAW", "OFF"} and "OFF" in cmds[i + 1:]  # never RESET / ARM after a trip
    tripped = [e for e in rows(tmp_path, "events") if e["event_type"] == "TRIPPED"]
    assert tripped and tripped[0]["evidence"]["trip"] == "TAP_INVERSION"
    snap = tripped[0]["evidence"]["trip_snapshot"]           # CGX: the measurement behind the trip
    assert snap["trip_reason"] == "TAP_INVERSION" and snap["tap_mv"][2] < snap["tap_mv"][1]
    diags = rows(tmp_path, "diagnostics")
    for d in diags:
        jsonschema.validate(d, diagnostic_schema)
    assert [d["detail"]["reason"] for d in diags if d["kind"] == "REST_VERIFY"] == ["REST_INVERTED"]
    assert [d["detail"]["hardware_trip"] for d in diags if d["kind"] == "TRIP_SNAPSHOT"] == [True]


class LinkLostDuringStage(SimTransport):
    """Pulls the cable 2 s (simulated) after the first RAMP was sent."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.drop_at = None

    def write(self, data):
        if b",RAMP," in data and self.drop_at is None:
            self.drop_at = self.sim_ms + 2000
        super().write(data)

    def read_lines(self, timeout_s):
        if self.drop_at is not None and self.sim_ms >= self.drop_at:
            self.drop_at = -1
            self.control("LINK DOWN")
        return super().read_lines(timeout_s)


def test_uart_loss_during_a_stage_aborts(sim016_path, tmp_path, monkeypatch, capsys):
    rc = run_cli(monkeypatch, sim016_path, ["--data-dir", str(tmp_path), "cycle", "--charge-to", "300",
                                            "--i-am-supervising"], transport=LinkLostDuringStage)
    out = capsys.readouterr()
    assert rc == 1 and "No ACK for PING" in out.err and "Could not confirm OFF" in out.out
    result = [d["detail"] for d in rows(tmp_path, "diagnostics") if d["kind"] == "CYCLE_RESULT"][-1]
    assert result["outcome"] == "ABORTED"
    assert any(r["record_status"] == "MISSING_TELEMETRY" for r in rows(tmp_path, "samples"))


def test_no_link_at_all_refuses_the_cycle(sim016_path, tmp_path, monkeypatch, capsys):
    rc = run_cli(monkeypatch, sim016_path, ["--data-dir", str(tmp_path), "cycle", "--charge-to", "300",
                                            "--i-am-supervising"], controls=["LINK DOWN"])
    assert rc == 1 and "cycle refused - nothing energised" in capsys.readouterr().err
    assert not list((tmp_path / "outbox").glob("raw-*.jsonl"))  # never started


def test_cycle_needs_supervision_and_valid_targets(sim016_path, tmp_path, monkeypatch):
    assert run_cli(monkeypatch, sim016_path, ["--data-dir", str(tmp_path), "cycle", "--charge-to", "300"]) == 2
    with pytest.raises(SystemExit):  # same limits as cellguard_cycle.py
        run_cli(monkeypatch, sim016_path, ["--data-dir", str(tmp_path), "cycle", "--charge-to", "1200",
                                           "--i-am-supervising"])


def test_cycle_on_0_2_0_firmware_sends_no_ramp(sim_path, tmp_path, capsys):
    rc = cli.main(["--sim", sim_path, "--sim-gates", "unlocked", "--data-dir", str(tmp_path), "cycle",
                   "--charge-to", "300", "--i-am-supervising"])
    assert rc == 1 and "has no RAMP/DRAIN" in capsys.readouterr().err
    cmds = sent_commands(tmp_path)
    assert not [c for c in cmds if c.startswith(("RAMP", "DRAIN", "ARM"))]


# ---------------------------------------------------------------- boot check and monitor

def test_boot_check_on_0_1_6_marks_selftest_unsupported(sim016_path, tmp_path, monkeypatch, capsys):
    assert run_cli(monkeypatch, sim016_path, ["--data-dir", str(tmp_path), "selftest", "--json"]) == 0
    cap = capsys.readouterr()
    rep = json.loads(cap.out)
    st = {c["name"]: c["status"] for c in rep["checks"]}
    assert st["nano_self_test"] == UNSUPPORTED and rep["result"] == WARN   # never PASS
    assert rep["actuation"]["allowed"] is True and rep["capabilities"]["family"] == "load_verify_0_1_6"
    assert not [x for x in rep["transcript"] if x["dir"] == "P>N" and "SELFTEST" in x["frame"]]
    assert "[UNSUPPORTED] nano_self_test" in cap.err


def test_actuation_verdict_is_conservative():
    def report(**status):
        names = ("link", "firmware", "nano_self_test", "nano_status", "telemetry", "measurement", "adc_path")
        checks = [Check(n, status.get(n, "PASS"), "") for n in names]
        import datetime as dt
        now = dt.datetime(2026, 10, 25)
        return Report(checks, now, now, {"capabilities": {"family": "load_verify_0_1_6"}}, [])

    assert report(nano_self_test=UNSUPPORTED).actuation["allowed"]
    assert not report(nano_self_test=UNSUPPORTED, adc_path="SKIP").actuation["allowed"]
    assert not report(nano_self_test=UNSUPPORTED, measurement="FAIL").actuation["allowed"]
    r = report()
    r.meta["capabilities"]["family"] = "unknown"
    assert not r.actuation["allowed"]


def test_monitor_on_0_1_6_writes_diagnostics_stream(sim016_path, tmp_path, monkeypatch):
    assert run_cli(monkeypatch, sim016_path, ["--data-dir", str(tmp_path), "monitor", "--seconds", "3"]) == 0
    samples = rows(tmp_path, "samples")
    assert samples and all(r["firmware_version"] == "0.1.6-LOAD-VERIFY" for r in samples)
    assert all(r["cell_temperature_c"][0] is not None for r in samples if r["record_status"] == "OK")  # REF3040
    oks = [r for r in samples if r["record_status"] == "OK"]
    assert oks[-1]["nano_adc_tap_raw"][0] is not None  # RAW works on 0.1.6


# ---------------------------------------------------------------- frames

def test_cgy_cgx_frames_parse_and_are_not_transport_errors():
    y = parse_message(BENCH_CGY)
    assert isinstance(y, LoadTapSuspect) and y.direction_name == "DISCHARGE" and y.tap_mv == [352, 696, 612, 955]
    x = parse_message("CGX,1000,6,42,83,73,114,352,696,612,955,273,2")
    assert isinstance(x, TripSnapshot) and x.voltage_quality_flags == ["TAP_INVERSION"]
    import datetime as dt
    t0 = dt.datetime(2026, 10, 25, 4, 0, tzinfo=dt.timezone.utc)
    at = lambda ms: t0 + dt.timedelta(milliseconds=ms)  # noqa: E731
    cgt = lambda seq, ms: fr(f"CGT,{seq},{ms},318,319,310,335,2500,2500,2500,2500,0,0,0,0,0,0,0,0,0,0,0,0")  # noqa: E731
    lines = [(at(1008), "N>P", cgt(0, 1000)), (at(1500), "N>P", fr(BENCH_CGY)),
             (at(1600), "N>P", fr("CGQ,1,2")),                         # valid CRC, unknown type
             (at(2008), "N>P", cgt(1, 2000)),
             (at(2500), "N>P", fr(BENCH_CGY)[:-4] + "0000"),           # corrupted CRC
             (at(3008), "N>P", cgt(2, 3000))]
    out, diags = [], []
    col = replay(lines, FusionContext(run_id="r"), out.append, run_start=t0, diag_sink=diags.append)
    tr = [r["transport"] for r in out if r["record_status"] == "OK"]
    assert sum(t["crc_errors_since_last"] for t in tr) == 1 and sum(t["format_errors_since_last"] for t in tr) == 0
    assert col.counters["unknown_frames"] == 1
    assert [d["kind"] for d in diags] == ["LOAD_TAP_SUSPECT_SNAPSHOT"]
    assert all(r["cell_voltage_v"] == [0.318, 0.319, 0.31, 0.335] for r in out if r["record_status"] == "OK")


def test_replay_of_a_0_1_6_cycle_reproduces_every_record(sim016_path, tmp_path, monkeypatch):
    run_cli(monkeypatch, sim016_path, ["--data-dir", str(tmp_path / "live"), "--run-id", "r16", "cycle",
                                       "--charge-to", "200", "--discharge-to", "120", "--i-am-supervising"],
            controls=["INJECT LOADED_INVERSION 1"])
    raws = sorted((tmp_path / "live" / "outbox").glob("raw-*.jsonl"))
    assert cli.main(["replay", "--from-raw", *map(str, raws), "--out", str(tmp_path / "rp")]) == 0
    for kind in ("samples", "diagnostics"):
        live = rows(tmp_path / "live", kind)
        again = [json.loads(l) for l in (tmp_path / "rp" / f"{kind}.jsonl").read_text().splitlines()]
        if kind == "diagnostics":  # CYCLE_* verdicts come from the live runner, not from the UART stream
            live = [d for d in live if not d["kind"].startswith("CYCLE_")]
            for d in live + again:
                d.pop("diag_id")
        assert [json.dumps(r, sort_keys=True) for r in again] == [json.dumps(r, sort_keys=True) for r in live]


def test_run_plan_and_shell_refuse_charge_on_0_1_6(sim016_path, tmp_path, monkeypatch, capsys):
    import io
    plan = tmp_path / "p.json"
    plan.write_text(json.dumps({"steps": ["SYNC", "ARM", {"cmd": "CHARGE", "args": [1]}, {"wait": 2}]}))
    rc = run_cli(monkeypatch, sim016_path, ["--data-dir", str(tmp_path / "a"), "run", str(plan), "--i-am-supervising"])
    assert rc == 1 and "use RAMP" in capsys.readouterr().err
    assert "CHARGE,1" not in sent_commands(tmp_path / "a") and sent_commands(tmp_path / "a")[-1] == "OFF"
    monkeypatch.setattr("sys.stdin", io.StringIO("charge 1\nselftest\nping\nquit\n"))
    assert run_cli(monkeypatch, sim016_path, ["--data-dir", str(tmp_path / "b"), "shell", "--i-am-supervising"]) == 0
    out = capsys.readouterr().out.splitlines()
    assert out[2].startswith("refused: CHARGE,1 is refused") and out[3].startswith("refused: SELFTEST")
    assert out[4] == "OK NONE"
