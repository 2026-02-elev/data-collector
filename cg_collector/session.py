# CellGuard data collector - command session, receive dispatch, fault-state tracking
# Author: Yongho Ahn <nanhosoft@gmail.com>
from __future__ import annotations

import collections
import dataclasses
import datetime as dt
import threading
from typing import Callable, Deque, Dict, List, Optional, Sequence, Tuple

from .protocol import (Ack, CalibrationInfo, Event, FrameError, Message, Telemetry, Version, TRIP_NAMES,
                       decode_line, encode_frame, parse_message)
from .transport import Transport

ACTUATION_COMMANDS = {"ARM", "RESET", "CHARGE", "DISCHARGE", "ESR", "LEAK", "HEAT", "BENCH"}
READ_ONLY_COMMANDS = {"SYNC", "PING", "STATUS", "RAW", "CAL", "OFF", "TRIP"}


@dataclasses.dataclass
class Received:
    utc: dt.datetime
    raw: str
    message: Optional[Message]
    error: Optional[str] = None


class CommandTimeout(RuntimeError):
    pass


class FaultTracker:
    """Keeps three facts apart for every actuator:

    * requested - what this host asked for (set by an accepted command, ended by
      a host stop *or* by any Nano-side stop event: TIMEOUT, TRIP, limits ...)
    * commanded - what the Nano reports it is driving (CGT masks / CGE events)
    * physical  - never known here: there is no contact/current feedback per
      fault, so it is always reported as unverified (null).
    """

    STOP_EVENTS = {"CHARGE_STOP": "CHARGE", "DISCHARGE_STOP": "DISCHARGE", "ESR_STOP": "ESR",
                   "LEAK_STOP": "LEAK", "HEAT_STOP": "HEAT"}

    def __init__(self) -> None:
        self.requested: Dict[Tuple[str, int], Dict[str, object]] = {}
        self.last_ended: Optional[Dict[str, object]] = None

    def _end(self, key: Tuple[str, int], reason: str, utc: dt.datetime) -> None:
        req = self.requested.pop(key, None)
        if req is not None:
            self.last_ended = {"type": key[0], "cell_id": key[1] or None, "reason": reason,
                               "requested_since_utc": req["since_utc"], "ended_utc": _iso(utc)}

    def _end_all(self, reason: str, utc: dt.datetime) -> None:
        for key in list(self.requested):
            self._end(key, reason, utc)

    def on_ack(self, name: str, args: Sequence[int], ack: Ack, utc: dt.datetime) -> None:
        if not ack.ok:
            return
        if name in ("SYNC", "OFF", "TRIP"):
            self._end_all("HOST_" + name, utc)
            return
        if name in ("CHARGE", "DISCHARGE"):
            key, value = (name, 0), int(args[0])
        elif name in ("ESR", "LEAK", "HEAT"):
            key, value = (name, int(args[0])), int(args[1])
        else:
            return
        if value:
            self.requested.setdefault(key, {"value": value, "since_utc": _iso(utc)})["value"] = value
        else:
            self._end(key, "HOST", utc)

    def on_event(self, ev: Event, utc: dt.datetime) -> None:
        kind = self.STOP_EVENTS.get(ev.code)
        if kind is not None:
            self._end((kind, ev.cell_id if kind in ("ESR", "LEAK", "HEAT") else 0), ev.reason, utc)
        elif ev.code == "TRIP":
            self._end_all("TRIP_" + ev.reason, utc)

    def requested_struct(self) -> Optional[Dict[str, object]]:
        faults = [{"type": k[0], "cell_id": k[1], "active": True, "value": v["value"], "since_utc": v["since_utc"]}
                  for k, v in sorted(self.requested.items()) if k[0] in ("ESR", "LEAK", "HEAT")]
        if not faults:
            return None
        first = dict(faults[0])
        if len(faults) > 1:
            first["all"] = faults
        return first

    def power_requested(self) -> Dict[str, bool]:
        return {"charge": ("CHARGE", 0) in self.requested, "discharge": ("DISCHARGE", 0) in self.requested}


def _iso(t: dt.datetime) -> str:
    return t.astimezone(dt.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


class NanoSession:
    """Sends CGC commands (seq management, same-seq retransmission) and
    dispatches EVERY received frame to the subscribers - including frames that
    arrive while a command is waiting for its ACK, so telemetry is never lost.

    Memory is bounded: only the last ``recent_size`` frames are kept.
    Thread-safe: a pump thread and a command caller may share one session.
    """

    def __init__(self, transport: Transport, ack_timeout_s: float = 0.3, retries: int = 3, recent_size: int = 256):
        self.t = transport
        self.ack_timeout_s = ack_timeout_s
        self.retries = retries
        self.seq = 0
        self.recent: Deque[Received] = collections.deque(maxlen=recent_size)
        self.version: Optional[Version] = None
        self.calibration: Optional[CalibrationInfo] = None
        self.frame_errors = 0
        self.faults = FaultTracker()
        self.last_ok_utc: Optional[dt.datetime] = None
        self._subscribers: List[Callable[[Received], None]] = []
        self._lock = threading.RLock()

    def subscribe(self, fn: Callable[[Received], None]) -> None:
        self._subscribers.append(fn)

    # -- receive side --
    def _ingest(self, lines: Sequence[str]) -> List[Received]:
        out = []
        for line in lines:
            if not line.startswith("$"):
                continue  # simulator markers / boot noise
            utc = self.t.now_utc()
            try:
                rec = Received(utc, line, parse_message(decode_line(line)))
            except FrameError as exc:
                self.frame_errors += 1
                rec = Received(utc, line, None, str(exc))
            msg = rec.message
            if isinstance(msg, Version):
                self.version = msg
            elif isinstance(msg, CalibrationInfo):
                self.calibration = msg
            elif isinstance(msg, Event):
                self.faults.on_event(msg, utc)
            self.recent.append(rec)
            for fn in self._subscribers:
                fn(rec)
            out.append(rec)
        return out

    def poll(self, timeout_s: float = 0.1) -> List[Received]:
        with self._lock:
            return self._ingest(self.t.read_lines(timeout_s))

    # -- command side --
    def command(self, name: str, *args: int) -> Ack:
        """Sends one command; on timeout re-sends the SAME seq (the Nano de-duplicates)."""
        with self._lock:
            self.seq = (self.seq + 1) & 0xFFFF
            seq = self.seq
            payload = ",".join(["CGC", str(seq), name] + [str(int(a)) for a in args])
            frame = encode_frame(payload)
            for _attempt in range(self.retries + 1):
                self.t.write(frame)
                deadline = self.t.now_utc() + dt.timedelta(seconds=self.ack_timeout_s)
                while True:
                    for rec in self._ingest(self.t.read_lines(0.05)):
                        if isinstance(rec.message, Ack) and rec.message.seq == seq:
                            if rec.message.ok:
                                self.last_ok_utc = rec.utc
                            self.faults.on_ack(name, args, rec.message, rec.utc)
                            return rec.message
                    if self.t.now_utc() >= deadline:
                        break
            raise CommandTimeout(f"no ACK for {payload}")

    def keepalive(self, interval_s: float = 1.0) -> Optional[Ack]:
        """Sends PING when no command was accepted for ``interval_s`` (Nano trips after 5 s)."""
        with self._lock:
            now = self.t.now_utc()
            if self.last_ok_utc is None or (now - self.last_ok_utc).total_seconds() >= interval_s:
                return self.command("PING")
            return None

    def pump(self, seconds: float, keepalive_s: Optional[float] = 1.0, step_s: float = 0.1) -> None:
        """Receives (and keeps the heartbeat alive) for ``seconds`` of transport time."""
        end = self.t.now_utc() + dt.timedelta(seconds=seconds)
        while self.t.now_utc() < end:
            if keepalive_s is not None:
                self.keepalive(keepalive_s)
            self.poll(step_s)


def trip_name(code: int) -> str:
    return TRIP_NAMES.get(code, str(code))


__all__ = ["ACTUATION_COMMANDS", "READ_ONLY_COMMANDS", "CommandTimeout", "FaultTracker", "NanoSession",
           "Received", "Telemetry", "trip_name"]
