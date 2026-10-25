# CellGuard data collector - Nano diagnostics stream (nano_diagnostic_v1)
# Author: Yongho Ahn <nanhosoft@gmail.com>
"""Diagnostic frames and events that are neither measurements nor fault events.

``diagnostics-*.jsonl`` keeps them apart from ``samples`` (normalized_sample_v1)
and ``events`` (fault_event_v1), so none of them can be mistaken for a measured
cell voltage or a successful step:

=============================  =================================================================
kind                           source and meaning
=============================  =================================================================
``LOAD_TAP_SUSPECT_SNAPSHOT``  CGY: taps measured WITH the charge/drain MOSFET ON that looked
                               inverted. ``rested_cell_voltage = false`` - never a cell voltage.
``TRIP_SNAPSHOT``              CGX: the measurement that caused a latched hardware trip.
``REST_VERIFY``                CGE LOAD_TAP_VERIFY: the rested re-check after a CGY.
                               ``REST_OK`` = rested voltages are back to normal; the stage is
                               still ABORTED (``stage_successful = false``). ``REST_INVERTED`` =
                               the inversion persists at rest -> TAP_INVERSION trip.
``NANO_EVENT``                 other 0.1.6 CGE codes: LOAD_TAP_SUSPECT, LIVE_GUARD, CHECKPOINT,
                               PULSE_TOTAL, RAMP_TARGET, DRAIN_TARGET_SET
``STAGE_STOP``                 CGE CHARGE_STOP / DRAIN_STOP of a RAMP / DRAIN stage with its
                               classification (TARGET_REACHED / ABORTED / LIMIT / TRIP / HOST)
``CYCLE_STAGE``                cg-collect cycle: the Pi's verdict on one stage (PASS / ABORTED)
=============================  =================================================================
"""
from __future__ import annotations

import datetime as dt
from typing import Callable, Dict, Optional

from .fusion import Collector, rfc3339
from .protocol import Event, LoadTapSuspect, TripSnapshot, TRIP_NAMES
from .session import NanoSession, Received

DIAG_CODES = ("LOAD_TAP_SUSPECT", "LOAD_TAP_VERIFY", "LIVE_GUARD", "CHECKPOINT", "PULSE_TOTAL", "RAMP_TARGET",
              "DRAIN_TARGET_SET")
STOP_CODES = {"CHARGE_STOP": "RAMP", "DRAIN_STOP": "DRAIN"}
TARGET_REASONS = {"TARGET", "DRAIN_TARGET"}
ABORT_REASONS = {"LOAD_TAP_UNSTABLE", "TIMEOUT", "BUDGET"}
LIMIT_REASONS = {"PACK_LIMIT", "CELL_LIMIT", "PACK_FLOOR", "CELL_FLOOR"}


def classify_stop(reason: str) -> Dict[str, object]:
    """How a CHARGE_STOP / DRAIN_STOP ends a staged step. Only a target stop can be a success,
    and even then the Pi must still verify the rested voltages (cg-collect cycle)."""
    if reason in TARGET_REASONS:
        cls = "TARGET_REACHED"
    elif reason in ABORT_REASONS:
        cls = "ABORTED"
    elif reason in LIMIT_REASONS:
        cls = "LIMIT"
    elif reason == "TRIP":
        cls = "TRIP"
    else:  # HOST / OFF / SYNC
        cls = "HOST_STOP"
    return {"stop_class": cls, "nano_target_reached": cls == "TARGET_REACHED", "abort_reason":
            reason if cls == "ABORTED" else None}


class DiagnosticRecorder:
    def __init__(self, session: NanoSession, collector: Collector, sink: Callable[[Dict[str, object]], None]):
        self.s = session
        self.col = collector
        self.sink = sink
        self.n = 0
        self.stage_command: Optional[str] = None   # RAMP / DRAIN of the running stage
        self.stage_target: Optional[int] = None
        self.last_trip_snapshot: Optional[Dict[str, object]] = None
        session.subscribe(self.on_frame)
        session.on_command(self.on_command)

    def _base(self, kind: str, rec: Optional[Received], millis: Optional[int]) -> Dict[str, object]:
        ctx = self.col.ctx
        v = self.s.version
        sim = v is not None and v.flag("HOST_SIM_BUILD")
        t = self.col.nano_t(millis) if millis is not None else None
        utc = rec.utc if rec is not None else self.s.t.now_utc()
        if t is None:
            t = self.col.t_of(utc)
        self.n += 1
        return {"schema_version": "nano_diagnostic_v1",
                "source": "synthetic" if sim else "measured_normal",
                "scenario_domain": "simulation_only" if sim else "measured_hardware",
                "device_id": ctx.device_id, "pack_id": ctx.pack_id, "run_id": ctx.run_id,
                "scenario_id": ctx.scenario_id, "diag_id": f"{ctx.run_id}-d{self.n:05d}", "kind": kind,
                "timestamp_utc": rfc3339(self.col.utc_of(t)), "t_sim_s": round(max(t, 0.0), 6),
                "pi_received_utc": rfc3339(utc) if rec is not None else None, "nano_millis": millis,
                "frame": rec.raw if rec is not None else None, "raw_log_ref": ctx.raw_log_ref,
                "firmware_version": v.firmware if v else "unknown"}

    def write(self, kind: str, detail: Dict[str, object], rec: Optional[Received] = None,
              millis: Optional[int] = None) -> Dict[str, object]:
        out = self._base(kind, rec, millis)
        out["detail"] = detail
        self.sink(out)
        return out

    def on_command(self, name, args, ack, sent_utc: dt.datetime) -> None:
        if name in ("RAMP", "DRAIN") and ack.ok:
            self.stage_command, self.stage_target = name, int(args[0])

    def on_frame(self, rec: Received) -> None:
        m = rec.message
        if isinstance(m, LoadTapSuspect):
            self.write("LOAD_TAP_SUSPECT_SNAPSHOT", {
                "direction": m.direction_name, "measured_under_load": True, "rested_cell_voltage": False,
                "adc_tap_raw": m.raw, "tap_mv_under_load": m.tap_mv, "bandgap_raw": m.bandgap_raw,
                "note": "on-load tap reading that looked inverted; MOSFET switched off, rested re-check follows. "
                        "Not a cell voltage."}, rec, m.millis)
        elif isinstance(m, TripSnapshot):
            detail = {"trip_code": m.trip, "trip_reason": TRIP_NAMES.get(m.trip, str(m.trip)), "adc_tap_raw": m.raw,
                      "tap_mv": m.tap_mv, "bandgap_raw": m.bandgap_raw, "voltage_quality_bits": m.voltage_quality,
                      "voltage_quality": m.voltage_quality_flags, "hardware_trip": True}
            self.last_trip_snapshot = detail
            self.write("TRIP_SNAPSHOT", detail, rec, m.millis)
        elif isinstance(m, Event):
            self._on_event(rec, m)

    def _on_event(self, rec: Received, e: Event) -> None:
        if e.code == "LOAD_TAP_VERIFY":
            ok = e.reason == "REST_OK"
            self.write("REST_VERIFY", {
                "code": e.code, "reason": e.reason, "rested_voltage_ok": ok, "stage_successful": False,
                "note": "rested voltages are normal again, the stage is aborted (not a success)" if ok else
                        "inversion persists at rest: hardware trip"}, rec, e.millis)
        elif e.code in DIAG_CODES:
            meaning = {"LOAD_TAP_SUSPECT": "on-load tap inversion suspected, MOSFET off",
                       "LIVE_GUARD": "on-load emergency limit exceeded -> trip",
                       "CHECKPOINT": "all rested cells reached the milestone (requested = milestone mV, "
                                     "commanded = lowest cell mV)",
                       "PULSE_TOTAL": "MOSFET pulses of the stage (requested = count, commanded = total on ms)",
                       "RAMP_TARGET": "RAMP target accepted (requested = mV per cell)",
                       "DRAIN_TARGET_SET": "DRAIN target accepted (requested = mV per cell)"}[e.code]
            self.write("NANO_EVENT", {"code": e.code, "reason": e.reason, "requested": e.requested,
                                      "commanded": e.commanded, "meaning": meaning}, rec, e.millis)
        elif e.code in STOP_CODES and self.s.capabilities.staged_power:
            detail = {"code": e.code, "reason": e.reason, "stage": STOP_CODES[e.code],
                      "target_mv_per_cell": self.stage_target if self.stage_command == STOP_CODES[e.code] else None}
            detail.update(classify_stop(e.reason))
            self.write("STAGE_STOP", detail, rec, e.millis)
            self.stage_command = self.stage_target = None
        elif e.code == "BOOT":
            self.stage_command = self.stage_target = None
            self.last_trip_snapshot = None


__all__ = ["DiagnosticRecorder", "classify_stop"]
