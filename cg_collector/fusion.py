# CellGuard data collector - Nano telemetry + RAW + INA219 -> normalized_sample_v1
# Author: Yongho Ahn <nanhosoft@gmail.com>
"""Measured counterpart of the data-generator's virtual Pi (``cg_sim/normalize.py``).

Both produce the same record layout so a model trained on synthetic data gets
the same inputs from the real rig:

* one record per 1 Hz telemetry slot; slots without a valid CGT become
  ``record_status = MISSING_TELEMETRY`` with null measurements (never zero-filled)
* ``nano_adc_tap_raw`` / ``nano_adc_ntc_raw`` from a CGD requested right after
  each CGT and joined only if it arrives within ``raw_join_s`` of the CGT
* ``t_sim_s`` = experiment elapsed time: seconds since the run started, at the
  estimated Nano sample instant (Nano millis mapped onto the Pi clock with a
  running-minimum offset). ``nano_millis`` keeps the raw MCU time.
* INA219 current: newest sample taken at least ``time_guard_s`` before the
  Nano sample instant, not older than the sampler's ``max_age_s``.
"""
from __future__ import annotations

import collections
import dataclasses
import datetime as dt
from typing import Callable, Deque, Dict, List, Optional

from .current import CurrentReading, CurrentSampler
from .protocol import (PROTOCOL_VERSION, TRIP_NAMES, CalibrationInfo, Event, RawAdc, Status, Telemetry,
                       Version)
from .session import CommandTimeout, FaultTracker, NanoSession, Received

Record = Dict[str, object]
UTC = dt.timezone.utc


def rfc3339(t: dt.datetime) -> str:
    t = t.astimezone(UTC)
    return t.strftime("%Y-%m-%dT%H:%M:%S.") + f"{t.microsecond // 1000:03d}Z"


def rfc3339_us(t: dt.datetime) -> str:
    """Microsecond timestamps for raw evidence logs (replay must reproduce the live timing)."""
    return t.astimezone(UTC).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


@dataclasses.dataclass
class FusionContext:
    device_id: str = "cellguard-001"
    pack_id: str = "pack-01"
    run_id: str = "run-unset"
    scenario_id: str = "manual"
    hardware_revision: str = "PCB1-PCB2-2026-09-18"
    raw_log_ref: str = "raw-log-unset"
    baud: int = 115200
    period_s: float = 1.0          # telemetry slot
    raw_join_s: float = 0.19       # CGD must arrive this soon after its CGT (same 5 Hz sample)
    flush_after_s: float = 0.6     # emit a CGT even if its CGS/CGD never arrives
    # None (default, same as the generator): MISSING slots are written when telemetry resumes or
    # at close. A number writes them live once telemetry has been silent that long (the
    # LAST_KNOWN command fields and transport counters then reflect an earlier moment).
    live_gap_s: Optional[float] = None
    time_guard_s: float = 0.005    # INA219 sample must be at least this older than the Nano sample


def _command_fields(cgt: Optional[Telemetry], cgs: Optional[Status], src: str) -> Record:
    if cgt is None:
        return {"charge_commanded": False, "discharge_commanded": False, "pack_relay_commanded": False,
                "pack_relay_commanded_known": False, "command_state_source": "UNKNOWN", "esr_command_mask": 0,
                "leak_command_mask": 0, "heater_pwm_command": [0, 0, 0, 0], "trip_reason": "NONE", "trip_code": 0,
                "controller_state": "UNKNOWN", "controller_locks": None, "fault_commanded": None}
    return {"charge_commanded": cgt.charge, "discharge_commanded": cgt.discharge,
            "pack_relay_commanded": bool(cgs.relay) if cgs else False,
            "pack_relay_commanded_known": cgs is not None,
            "controller_locks": cgs.locks if cgs else None,
            "fault_commanded": {"esr_mask": cgt.esr_mask, "leak_mask": cgt.leak_mask, "heat": list(cgt.heat)},
            "command_state_source": src, "esr_command_mask": cgt.esr_mask, "leak_command_mask": cgt.leak_mask,
            "heater_pwm_command": list(cgt.heat), "trip_reason": TRIP_NAMES.get(cgt.trip, str(cgt.trip)),
            "trip_code": cgt.trip, "controller_state": cgs.state if cgs else "UNKNOWN"}


class Collector:
    """Turns the session's frame stream into normalized records (written to ``sink`` at once).

    Memory stays bounded (one pending slot + a small ``recent`` window).
    """

    def __init__(self, session: NanoSession, ctx: FusionContext, sink: Callable[[Record], None],
                 current: Optional[CurrentSampler] = None, request_raw: bool = True,
                 run_start_utc: Optional[dt.datetime] = None, recent_size: int = 32):
        self.s = session
        self.ctx = ctx
        self.sink = sink
        self.current = current
        self.request_raw = request_raw
        self.run_start = run_start_utc or session.t.now_utc()
        self.count = 0
        self.recent: Deque[Record] = collections.deque(maxlen=recent_size)
        self.boot_epoch = 0
        self.epoch_offset: Optional[float] = None
        self.last_millis: Optional[int] = None
        self.last_seq: Optional[int] = None
        self.pending: Optional[dict] = None
        self.last_t: Optional[float] = None
        self.last_cgt: Optional[Telemetry] = None
        self.last_cgs: Optional[Status] = None
        self.counters = {"cgt_valid": 0, "cgt_duplicate": 0, "cgt_seq_gap": 0, "records_ok": 0,
                         "records_missing": 0, "cgd_joined": 0, "raw_requests": 0, "boots_seen": 0}
        self._since = {"crc": 0, "dup": 0, "fmt": 0}
        session.subscribe(self.on_frame)
        session.add_idle_hook(self.service)
        session.faults.event_clock = self._event_utc

    def _event_utc(self, ev: Event, rx_utc: dt.datetime) -> dt.datetime:
        t = self.nano_t(ev.millis)
        return rx_utc if t is None else self.utc_of(t)

    # ---------------------------------------------------------------- time base
    def t_of(self, utc: dt.datetime) -> float:
        return (utc - self.run_start).total_seconds()

    def utc_of(self, t: float) -> dt.datetime:
        return self.run_start + dt.timedelta(microseconds=int(round(t * 1e6)))

    def nano_t(self, millis: int) -> Optional[float]:
        """Run time of a Nano millis value (None until the first CGT of this boot)."""
        return None if self.epoch_offset is None else millis / 1000.0 + self.epoch_offset

    # ---------------------------------------------------------------- input
    def on_frame(self, rec: Received) -> None:
        t_rx = self.t_of(rec.utc)
        msg = rec.message
        if msg is None:
            self._since["crc" if rec.error and "CRC" in rec.error else "fmt"] += 1
            return
        if isinstance(msg, Event) and msg.code == "BOOT":
            self._new_boot_epoch()
        elif isinstance(msg, Telemetry):
            self._on_cgt(t_rx, msg, len(rec.raw) + 1)
        elif isinstance(msg, Status):
            p = self.pending
            if p is not None and p["cgt"].seq == msg.seq and "cgs" not in p:
                p["cgs"] = msg
            self.last_cgs = msg
        elif isinstance(msg, RawAdc):
            p = self.pending
            if p is not None and "cgd" not in p and t_rx - p["t_rx"] < self.ctx.raw_join_s:
                p["cgd"] = msg
                self.counters["cgd_joined"] += 1

    def _new_boot_epoch(self) -> None:
        self.flush()
        self.boot_epoch += 1
        self.epoch_offset = None
        self.last_millis = None
        self.last_seq = None
        self.counters["boots_seen"] += 1

    def _on_cgt(self, t_rx: float, f: Telemetry, nbytes: int) -> None:
        if self.last_millis is not None and f.millis < self.last_millis:
            self._new_boot_epoch()  # reboot although the BOOT event was lost
        if self.last_seq is not None and f.seq == self.last_seq and f.millis == self.last_millis:
            self.counters["cgt_duplicate"] += 1
            self._since["dup"] += 1
            return
        if self.last_seq is not None and (f.seq - self.last_seq) % 65536 != 1:
            self.counters["cgt_seq_gap"] += 1
        off = t_rx - nbytes * 10.0 / self.ctx.baud - f.millis / 1000.0
        self.epoch_offset = off if self.epoch_offset is None else min(self.epoch_offset, off)
        stale_repeat = self.last_millis is not None and f.millis == self.last_millis
        self.flush()
        self.last_seq, self.last_millis = f.seq, f.millis
        self.counters["cgt_valid"] += 1
        self.pending = {"cgt": f, "t_rx": t_rx, "epoch": self.boot_epoch, "raw_sent": not self.request_raw}
        if stale_repeat and self.last_t is not None:
            self.pending["t_hint"] = self.last_t + self.ctx.period_s  # no new 5 Hz sample behind this CGT

    def service(self, session: NanoSession) -> None:
        """Idle hook: request RAW for the pending slot, flush stale slots, emit live MISSING slots."""
        p = self.pending
        now = self.t_of(session.t.now_utc())
        if p is not None and not p["raw_sent"]:
            p["raw_sent"] = True
            if now - p["t_rx"] < self.ctx.raw_join_s:
                self.counters["raw_requests"] += 1
                try:
                    session.command("RAW")  # its CGD arrives through on_frame
                except CommandTimeout:
                    self.counters["raw_timeouts"] = self.counters.get("raw_timeouts", 0) + 1
        self.tick(self.t_of(session.t.now_utc()))

    def tick(self, now: float) -> None:
        """A slot is written when the next CGT arrives or ``flush_after_s`` after its CGT, never
        earlier: the next CGT refines the Nano clock offset used for ``t_sim_s`` (generator rule)."""
        if self.pending is not None and now >= self.pending["t_rx"] + self.ctx.flush_after_s:
            self.flush()
        if self.ctx.live_gap_s is not None and self.pending is None and self.last_t is not None:
            while now - self.last_t >= self.ctx.live_gap_s:
                self._emit_missing(round(self.last_t + self.ctx.period_s, 3))

    # ---------------------------------------------------------------- output
    def flush(self) -> None:
        if self.pending is None:
            return
        p, self.pending = self.pending, None
        p["t_est"] = self._t_est(p)
        self._fill_gap(p["t_est"])
        self._emit(p)

    def close(self, t_end: Optional[float] = None) -> None:
        self.flush()
        if t_end is None:
            t_end = self.t_of(self.s.t.now_utc())
        self._fill_gap(t_end + 1e-6, trailing=True)

    def _t_est(self, p: dict) -> float:
        if "t_hint" in p:
            return round(p["t_hint"], 3)
        t = max(0.0, round(p["cgt"].millis / 1000.0 + self.epoch_offset, 3))
        if self.last_t is not None and t <= self.last_t:
            t = round(self.last_t + 0.001, 3)
        return t

    def _fill_gap(self, t_next: float, trailing: bool = False) -> None:
        per = self.ctx.period_s
        if self.last_t is None:
            if trailing:
                return
            n = int(round(t_next / per))  # telemetry missing from the start of the run
            for k in range(n):
                t = round(t_next - n * per + k * per, 3)
                if t >= 0:  # never a slot before the run started
                    self._emit_missing(t)
            return
        gap = t_next - self.last_t
        n = int(gap // per + 1e-9) if trailing else int(round(gap / per)) - 1
        base = self.last_t
        for k in range(1, n + 1):
            self._emit_missing(round(base + k * per, 3))

    def _current(self, t_sample: float, t_avail: float) -> Record:
        if self.current is None:
            reading = CurrentReading(None, "INA219_UNAVAILABLE", None)
        else:
            reading = self.current.reading_at(self.utc_of(t_sample - self.ctx.time_guard_s),
                                              available_by=self.utc_of(t_avail), age_ref=self.utc_of(t_sample))
        age = None
        if reading.sampled_utc is not None:
            age = round(t_sample - self.t_of(reading.sampled_utc), 3)
        valid = reading.quality == "VALID"
        return {"pack_current_a": round(reading.current_a, 6) if valid and reading.current_a is not None else None,
                "current_quality": reading.quality,
                "ina219_sampled_utc": rfc3339(reading.sampled_utc) if reading.sampled_utc else None,
                "current_age_s": age}

    def _base(self, t: float, t_avail: float) -> Record:
        version: Optional[Version] = self.s.version
        sim_build = version is not None and version.flag("HOST_SIM_BUILD")
        rec: Record = {"schema_version": "normalized_sample_v1"}
        rec.update(device_id=self.ctx.device_id, pack_id=self.ctx.pack_id, run_id=self.ctx.run_id,
                   scenario_id=self.ctx.scenario_id, timestamp_utc=rfc3339(self.utc_of(t)), t_sim_s=t)
        faults: FaultTracker = self.s.faults
        req, power, ended = faults.state_at(self.utc_of(t))  # as of the slot time
        rec["fault_requested"] = req
        rec["power_requested"] = dict(power)
        rec["fault_request_last_ended"] = ended
        rec["fault_modeled_applied"] = None    # no model for measured data
        rec["fault_physical_observed"] = None  # no physical feedback exists; never inferred
        rec.update(self._current(t, t_avail))
        rec["_sim_build"] = sim_build
        return rec

    def _tail(self, rec: Record) -> Record:
        version = self.s.version
        cal: Optional[CalibrationInfo] = self.s.calibration
        sim_build = rec.pop("_sim_build")
        rec.update({"physical_fault_verified": False,
                    "physical_harness_verified": bool(version and version.flag("MAP_VERIFIED") and not sim_build),
                    "calibration_id": cal.cal_id if cal else None,
                    "firmware_version": version.firmware if version else "unknown",
                    "protocol_version": PROTOCOL_VERSION,
                    "transport": {"crc_errors_since_last": self._since["crc"],
                                  "duplicates_since_last": self._since["dup"],
                                  "format_errors_since_last": self._since["fmt"]}})
        self._since = {"crc": 0, "dup": 0, "fmt": 0}
        rec["raw_log_ref"] = self.ctx.raw_log_ref
        fault_active = bool(rec.get("esr_command_mask") or rec.get("leak_command_mask")
                            or any(rec.get("heater_pwm_command") or []))
        if sim_build:  # firmware host simulator output is never measured data
            rec.update(source="synthetic", generator_version=f"nano_host_sim/{rec['firmware_version']}", seed=0,
                       config_hash="nano-host-sim", scenario_domain="simulation_only")
        else:
            rec.update(source="measured_fault_injection" if fault_active else "measured_normal",
                       hardware_revision=self.ctx.hardware_revision, scenario_domain="measured_hardware")
        return rec

    def _write(self, rec: Record) -> None:
        self.count += 1
        self.recent.append(rec)
        self.sink(rec)

    def _emit(self, p: dict) -> None:
        f: Telemetry = p["cgt"]
        t = p["t_est"]
        rec = self._base(t, p["t_rx"])  # current: only readings the Pi had when the CGT arrived
        rec.update({"record_status": "OK", "nano_boot_epoch": p["epoch"], "nano_millis": f.millis,
                    "telemetry_seq": f.seq, "pi_received_utc": rfc3339(self.utc_of(p["t_rx"]))})
        cells = [None if mv is None else mv / 1000.0 for mv in f.cell_mv]
        taps: List[Optional[float]] = []
        acc: Optional[float] = 0.0
        for c in cells:
            acc = None if (c is None or acc is None) else round(acc + c, 3)
            taps.append(acc)
        rec["cell_voltage_v"] = cells
        rec["cumulative_tap_voltage_v"] = taps
        rec["cell_temperature_c"] = [None if cc is None else cc / 100.0 for cc in f.temp_cc]
        rec["temperature_quality"] = f.temp_quality
        cgd: Optional[RawAdc] = p.get("cgd")
        rec["nano_adc_tap_raw"] = list(cgd.raw[:4]) if cgd else [None] * 4
        rec["nano_adc_ntc_raw"] = list(cgd.raw[4:]) if cgd else [None] * 4
        cgs = p.get("cgs")
        src = "CGT+CGS" if cgs else "CGT+LAST_KNOWN_CGS"
        rec.update(_command_fields(f, cgs or self.last_cgs, src))
        rec["voltage_quality"] = f.voltage_quality_enum
        rec["voltage_quality_bits"] = f.voltage_quality
        rec["temperature_quality_mask"] = f.temp_quality_mask
        self._write(self._tail(rec))
        self.last_t = t
        self.last_cgt = f
        self.last_cgs = cgs or self.last_cgs
        self.counters["records_ok"] += 1

    def _emit_missing(self, t: float) -> None:
        rec = self._base(t, t + 0.05)
        rec.update({"record_status": "MISSING_TELEMETRY", "nano_boot_epoch": self.boot_epoch, "nano_millis": None,
                    "telemetry_seq": None, "pi_received_utc": None,
                    "cell_voltage_v": [None] * 4, "cumulative_tap_voltage_v": [None] * 4,
                    "cell_temperature_c": [None] * 4, "temperature_quality": ["MISSING"] * 4,
                    "nano_adc_tap_raw": [None] * 4, "nano_adc_ntc_raw": [None] * 4})
        rec.update(_command_fields(self.last_cgt, self.last_cgs, "LAST_KNOWN"))
        rec["voltage_quality"] = "MISSING"
        rec["voltage_quality_bits"] = None
        rec["temperature_quality_mask"] = None
        self._write(self._tail(rec))
        self.last_t = t
        self.counters["records_missing"] += 1
