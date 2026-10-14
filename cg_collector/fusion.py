# CellGuard data collector - Nano telemetry + INA219 -> normalized_sample_v1
# Author: Yongho Ahn <nanhosoft@gmail.com>
from __future__ import annotations

import collections
import dataclasses
import datetime as dt
from typing import Callable, Deque, Dict, List, Optional, Tuple

from .current import CurrentReading, CurrentSampler
from .protocol import Status, Telemetry, TRIP_NAMES, Version
from .session import FaultTracker, NanoSession, Received

Record = Dict[str, object]


def rfc3339(t: dt.datetime) -> str:
    return t.astimezone(dt.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


@dataclasses.dataclass
class FusionContext:
    device_id: str = "cellguard-001"
    pack_id: str = "pack-01"
    run_id: str = "run-unset"
    scenario_id: str = "manual"
    hardware_revision: str = "PCB1-PCB2-2026-09-18"
    raw_log_ref: str = "raw-log-unset"


def fuse_sample(tel: Telemetry, received_utc: dt.datetime, status: Optional[Status], version: Optional[Version],
                current: CurrentReading, faults: FaultTracker, ctx: FusionContext,
                calibration_id: Optional[str] = None) -> Record:
    """One normalized_sample_v1 record from a CRC-valid CGT (+ the CGS of the same seq).

    ``current`` must already be time-aligned (sampled at or before ``received_utc``).
    """
    cells_v = [None if mv is None else mv / 1000.0 for mv in tel.cell_mv]
    taps: List[Optional[float]] = []
    acc: Optional[float] = 0.0
    for v in cells_v:
        acc = None if (acc is None or v is None) else acc + v
        taps.append(None if acc is None else round(acc, 4))
    temps = [None if cc is None else cc / 100.0 for cc in tel.temp_cc]

    sim_build = version is not None and version.flag("HOST_SIM_BUILD")
    fault_active = bool(tel.esr_mask or tel.leak_mask or any(tel.heat))
    rec: Record = {
        "schema_version": "normalized_sample_v1",
        "device_id": ctx.device_id,
        "pack_id": ctx.pack_id,
        "run_id": ctx.run_id,
        "scenario_id": ctx.scenario_id,
        "timestamp_utc": rfc3339(received_utc),
        "pi_received_utc": rfc3339(received_utc),
        "nano_millis": tel.millis,
        "nano_telemetry_seq": tel.seq,
        "t_sim_s": tel.millis / 1000.0,  # relative time = Nano millis (not a simulation time for measured data)
        "cell_voltage_v": cells_v,
        "cumulative_tap_voltage_v": taps,
        "cell_temperature_c": temps,
        "temperature_quality": tel.temp_quality,
        "pack_current_a": current.current_a if current.quality == "VALID" else None,
        "current_quality": current.quality,
        "ina219_sampled_utc": rfc3339(current.sampled_utc) if current.sampled_utc and current.quality == "VALID" else None,
        "charge_commanded": tel.charge,
        "discharge_commanded": tel.discharge,
        "pack_relay_commanded": bool(status.relay) if status is not None else False,
        "pack_relay_commanded_known": status is not None,
        "esr_command_mask": tel.esr_mask,
        "leak_command_mask": tel.leak_mask,
        "heater_pwm_command": list(tel.heat),
        "trip_reason": TRIP_NAMES.get(tel.trip, str(tel.trip)),
        "controller_state": status.state if status is not None else ("ARMED" if tel.armed else "UNKNOWN"),
        "controller_locks": status.locks if status is not None else None,
        "voltage_quality": tel.voltage_quality_enum,
        "voltage_quality_flags": tel.voltage_quality_flags,
        # requested (host intent) / commanded (Nano outputs) / physical (unverified) kept apart
        "fault_requested": faults.requested_struct(),
        "power_requested": faults.power_requested(),
        "fault_request_last_ended": faults.last_ended,
        "fault_commanded": {"esr_mask": tel.esr_mask, "leak_mask": tel.leak_mask, "heat": list(tel.heat)},
        "fault_modeled_applied": None,       # no model for measured data
        "fault_physical_observed": None,     # no contact / current feedback per fault
        "physical_fault_verified": False,
        "physical_harness_verified": bool(version and version.flag("MAP_VERIFIED") and not sim_build),
        "calibration_id": calibration_id,
        "firmware_version": version.firmware if version else "unknown",
    }
    if sim_build:
        # Firmware host simulator output is never measured data.
        rec.update(source="synthetic", generator_version=f"nano_host_sim/{rec['firmware_version']}", seed=0,
                   config_hash="nano-host-sim")
    else:
        rec.update(source="measured_fault_injection" if fault_active else "measured_normal",
                   hardware_revision=ctx.hardware_revision, raw_log_ref=ctx.raw_log_ref)
    return rec


class Collector:
    """Subscribes to a session and emits one record per CGT.

    * Every frame reaches the collector (also those received while a command
      waits for its ACK) because it subscribes to the session dispatch.
    * The firmware sends CGS right after the CGT of the same seq, so at most
      one CGT waits for its CGS; a CGT whose CGS was lost is emitted when the
      next CGT arrives (relay/state fields then marked unknown).
    * Records go to ``sink`` immediately; only a bounded ``recent`` window is
      kept in memory, so week-long runs do not grow.
    """

    def __init__(self, session: NanoSession, ctx: FusionContext, sink: Callable[[Record], None],
                 current: Optional[CurrentSampler] = None, recent_size: int = 32):
        self.s = session
        self.ctx = ctx
        self.sink = sink
        self.current = current
        self.count = 0
        self.recent: Deque[Record] = collections.deque(maxlen=recent_size)
        self._pending: Optional[Tuple[dt.datetime, Telemetry]] = None
        session.subscribe(self.on_frame)

    def on_frame(self, rec: Received) -> None:
        msg = rec.message
        if isinstance(msg, Telemetry):
            self.close()
            self._pending = (rec.utc, msg)
        elif isinstance(msg, Status) and self._pending is not None and self._pending[1].seq == msg.seq:
            utc, tel = self._pending
            self._pending = None
            self._emit(utc, tel, msg)

    def close(self) -> None:
        if self._pending is not None:
            utc, tel = self._pending
            self._pending = None
            self._emit(utc, tel, None)

    def _emit(self, utc: dt.datetime, tel: Telemetry, st: Optional[Status]) -> None:
        reading = (CurrentReading(None, "INA219_UNAVAILABLE", None) if self.current is None
                   else self.current.reading_at(utc))
        cal = self.s.calibration.cal_id if self.s.calibration else None
        rec = fuse_sample(tel, utc, st, self.s.version, reading, self.s.faults, self.ctx, cal)
        self.count += 1
        self.recent.append(rec)
        self.sink(rec)
