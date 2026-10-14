# CellGuard data collector - fault_event_v1 records from commands and CGE events
# Author: Yongho Ahn <nanhosoft@gmail.com>
"""Same event vocabulary as the data-generator (``events.jsonl``):

* ``REQUESTED``      - this host sent a command (fault type + cell for ESR/LEAK/HEAT)
* ``FAULT_REJECTED`` - the Nano refused an actuation command (CGA ERR)
* ``COMMANDED``      - the Nano changed an output (CGE *_START/_STOP/_SET, RELAY_*, ARM/DISARM)
* ``TRIPPED`` / ``RESET`` / ``BOOT`` - from the matching CGE

A CGE lost on the link cannot be recorded; the gap is counted from ``eventSeq``
and reported as ``evidence.cge.events_lost_before`` on the next received event.
The generator's ``events.jsonl`` is a label stream built from the virtual Nano's
own events, so under frame loss it contains events this file cannot.

``APPLIED`` / ``RELEASED`` / ``OBSERVABLE`` exist only for the generator's model:
the rig has no per-fault physical feedback, so measured runs never contain them
and ``physical_fault_verified`` is always false.
"""
from __future__ import annotations

import datetime as dt
from typing import Callable, Dict, Optional, Sequence

from .fusion import Collector, rfc3339
from .protocol import Ack, Event
from .session import NanoSession, Received

CELL_CODES = ("ESR_START", "LEAK_START", "HEAT_START", "HEAT_SET", "ESR_STOP", "LEAK_STOP", "HEAT_STOP")
SYSTEM_CODES = ("CHARGE_START", "CHARGE_STOP", "DISCHARGE_START", "DISCHARGE_STOP", "RELAY_ON", "RELAY_OFF",
                "ARM", "DISARM")
REJECT_TRACKED = ("ESR", "LEAK", "HEAT", "CHARGE", "DISCHARGE", "ARM")


class EventRecorder:
    def __init__(self, session: NanoSession, collector: Collector, sink: Callable[[Dict[str, object]], None]):
        self.s = session
        self.col = collector
        self.sink = sink
        self.n = 0
        self.last_event_seq: Optional[int] = None
        self.lost_since_last = 0      # CGE eventSeq gaps: Nano events the Pi never received
        self.lost_total = 0
        self.duplicates = 0
        session.subscribe(self.on_frame)
        session.on_command(self.on_command)

    def _source(self, fault_type: str) -> Dict[str, str]:
        v = self.s.version
        if v is not None and v.flag("HOST_SIM_BUILD"):
            return {"source": "synthetic", "scenario_domain": "simulation_only"}
        injected = fault_type in ("ESR", "LEAK", "HEAT")
        return {"source": "measured_fault_injection" if injected else "measured_normal",
                "scenario_domain": "measured_hardware"}

    def _write(self, t: float, etype: str, ftype: str, cell: Optional[int], req=None, cmd=None, dur=None,
               **evidence) -> None:
        self.n += 1
        ctx = self.col.ctx
        v = self.s.version
        rec: Dict[str, object] = {
            "schema_version": "fault_event_v1", "run_id": ctx.run_id, "scenario_id": ctx.scenario_id,
            "event_id": f"{ctx.run_id}-e{self.n:05d}", "event_type": etype, "fault_type": ftype, "cell_id": cell,
            "timestamp_utc": rfc3339(self.col.utc_of(t)), "t_sim_s": round(max(t, 0.0), 6),
            "requested_value": req, "commanded_value": cmd, "applied_value": None, "duration_s": dur,
            "physical_fault_verified": False,
            "evidence": dict(collector="cg_collector", physical_harness_verified=False,
                             firmware_version=v.firmware if v else "unknown", raw_log_ref=ctx.raw_log_ref,
                             **{k: x for k, x in evidence.items() if x is not None}),
        }
        rec.update(self._source(ftype))
        self.sink(rec)

    def on_frame(self, rec: Received) -> None:
        e = rec.message
        if not isinstance(e, Event):
            return
        if e.code == "BOOT":
            self.last_event_seq = None
        if self.last_event_seq is not None:
            d = (e.event_seq - self.last_event_seq) % 65536
            if d == 0 or d >= 32768:  # duplicated / re-ordered frame: already recorded
                self.duplicates += 1
                return
            if d > 1:
                self.lost_since_last += d - 1
                self.lost_total += d - 1
        self.last_event_seq = e.event_seq
        t = self.col.nano_t(e.millis)
        if t is None:  # before the first CGT of this boot: use the receive time
            t = self.col.t_of(rec.utc)
        cge = {"cge_event_seq": e.event_seq, "nano_millis": e.millis, "cge_code": e.code, "reason": e.reason}
        recorded = e.code in CELL_CODES or e.code in SYSTEM_CODES or e.code in ("TRIP", "RESET", "BOOT")
        if recorded and self.lost_since_last:
            # Nano events since the previous recorded event were lost on the link
            cge["events_lost_before"] = self.lost_since_last
            self.lost_since_last = 0
        if e.code in CELL_CODES:
            self._write(t, "COMMANDED", e.code.split("_")[0], e.cell_id, req=e.requested, cmd=e.commanded, cge=cge)
        elif e.code in SYSTEM_CODES:
            self._write(t, "COMMANDED", "NONE", None, req=e.requested, cmd=e.commanded, cge=cge)
        elif e.code == "TRIP":
            self._write(t, "TRIPPED", "NONE", None, cge=cge, trip=e.reason)
        elif e.code == "RESET":
            self._write(t, "RESET", "NONE", None, req=1, cmd=1, cge=cge)
        elif e.code == "BOOT":
            self._write(t, "BOOT", "NONE", None, cge=cge)

    def on_command(self, name: str, args: Sequence[int], ack: Ack, sent_utc: dt.datetime) -> None:
        t = self.col.t_of(sent_utc)
        cga = f"CGA,{ack.seq},{'OK' if ack.ok else 'ERR'},{ack.text}"
        if name in ("ARM", "OFF", "TRIP", "RESET", "SYNC"):
            self._write(t, "REQUESTED", "NONE", None, req=name, command=name)
        elif name in ("CHARGE", "DISCHARGE"):
            self._write(t, "REQUESTED", "NONE", None, req=int(args[0]), command=name)
        elif name in ("ESR", "LEAK", "HEAT"):
            self._write(t, "REQUESTED", name, int(args[0]), req=int(args[1]))
        else:
            return  # PING / STATUS / RAW / CAL are not experiment events
        if not ack.ok and name in REJECT_TRACKED:
            ftype = name if name in ("ESR", "LEAK", "HEAT") else "NONE"
            cell = int(args[0]) if ftype != "NONE" else None
            self._write(t, "FAULT_REJECTED", ftype, cell, req=int(args[-1]) if args else None, cga=cga,
                        command=name)
