# CellGuard data collector - command session, receive dispatch, fault-state tracking
# Author: Yongho Ahn <nanhosoft@gmail.com>
from __future__ import annotations

import bisect
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


FAULT_ORDER = {"ESR": 0, "LEAK": 1, "HEAT": 2}


class FaultTracker:
    """Keeps three facts apart for every actuator (same semantics as the
    data-generator's virtual Pi, so synthetic and measured records match):

    * requested - what this host asked for. Set by an accepted command; ended
      by a host stop (reason ``HOST``), by OFF/TRIP/SYNC (reason = command), by a
      Nano-side stop event (its reason: TIMEOUT, TRIP, PACK_LIMIT ...) or by a
      refusal (``REJECTED_<reason>``).
    * commanded - what the Nano reports it is driving (CGT masks / CGE events)
    * physical  - never known here (no contact/current feedback per fault),
      always reported as unverified (null).
    """

    STOP_EVENTS = {"CHARGE_STOP": "CHARGE", "DISCHARGE_STOP": "DISCHARGE", "ESR_STOP": "ESR",
                   "LEAK_STOP": "LEAK", "HEAT_STOP": "HEAT"}

    def __init__(self) -> None:
        self.requested: Dict[Tuple[str, int], Dict[str, object]] = {}
        self.last_ended: Optional[Dict[str, object]] = None
        # Maps a CGE to the time the Nano acted (set by the Collector once it knows the
        # Nano clock offset); defaults to the Pi receive time.
        self.event_clock: Callable[[Event, dt.datetime], dt.datetime] = lambda ev, rx_utc: rx_utc
        # (utc, snapshot) after every change, so records for past slots (gap fill) see the
        # state as of the slot time, not as of when they are written. Bounded.
        self._hist_t: Deque[dt.datetime] = collections.deque(maxlen=4096)
        self._hist_v: Deque[tuple] = collections.deque(maxlen=4096)

    def _snap(self, utc: dt.datetime) -> None:
        if self._hist_t and utc < self._hist_t[-1]:
            utc = self._hist_t[-1]  # keep the history monotonic
        self._hist_t.append(utc)
        self._hist_v.append((self.requested_struct(), self.power_requested(), self.last_ended))

    def state_at(self, utc: dt.datetime) -> tuple:
        """(fault_requested, power_requested, fault_request_last_ended) as of ``utc``."""
        i = bisect.bisect_right(list(self._hist_t), utc)
        if i == 0:
            return None, {"charge": False, "discharge": False}, None
        return self._hist_v[i - 1]

    def _end(self, key: Tuple[str, int], reason: str, utc: dt.datetime, since: Optional[str] = None) -> None:
        req = self.requested.pop(key, None)
        if req is not None or since is not None:
            self.last_ended = {"type": key[0], "cell_id": key[1] or None, "reason": reason,
                               "requested_since_utc": req["since_utc"] if req else since, "ended_utc": _iso(utc)}

    def _end_all(self, reason: str, utc: dt.datetime) -> None:
        for key in sorted(self.requested, key=lambda k: (FAULT_ORDER.get(k[0], 9), k[1])):
            self._end(key, reason, utc)

    @staticmethod
    def _key_value(name: str, args: Sequence[int]) -> Optional[Tuple[Tuple[str, int], int]]:
        if name in ("CHARGE", "DISCHARGE"):
            return (name, 0), int(args[0])
        if name in ("ESR", "LEAK", "HEAT"):
            return (name, int(args[0])), int(args[1])
        return None

    def on_ack(self, name: str, args: Sequence[int], ack: Ack, utc: dt.datetime,
               ack_utc: Optional[dt.datetime] = None) -> None:
        """``utc`` = when the command was sent (request time), ``ack_utc`` = when the reply arrived."""
        kv = self._key_value(name, args)
        if not ack.ok:
            if kv is not None and kv[1]:  # requested from sending until the refusal arrived
                key, value = kv
                prev = self.requested.get(key)
                self.requested[key] = {"value": value, "since_utc": prev["since_utc"] if prev else _iso(utc)}
                self._snap(utc)
                self._end(key, "REJECTED_" + ack.text, ack_utc or utc)
                self._snap(ack_utc or utc)
            return
        if name in ("SYNC", "OFF", "TRIP"):
            self._end_all(name, utc)
        elif kv is None:
            return
        else:
            key, value = kv
            if value:
                self.requested.setdefault(key, {"value": value, "since_utc": _iso(utc)})["value"] = value
            else:
                self._end(key, "HOST", utc)
        self._snap(utc)

    def on_event(self, ev: Event, utc: dt.datetime) -> None:
        if ev.code == "BOOT":  # a Nano reset switched every output off
            self._end_all("NANO_RESET", utc)
            self._snap(utc)
            return
        kind = self.STOP_EVENTS.get(ev.code)
        if kind is not None and ev.reason != "HOST":
            at = self.event_clock(ev, utc)
            self._end((kind, ev.cell_id if kind in FAULT_ORDER else 0), ev.reason, at)
            self._snap(at)

    def requested_struct(self) -> Optional[Dict[str, object]]:
        faults = [{"type": k[0], "cell_id": k[1], "active": True, "value": v["value"]}
                  for k, v in sorted(self.requested.items(), key=lambda kv: (FAULT_ORDER.get(kv[0][0], 9), kv[0][1]))
                  if k[0] in FAULT_ORDER]
        if not faults:
            return None
        return {"type": faults[0]["type"], "cell_id": faults[0]["cell_id"], "active": True, "count": len(faults),
                "faults": faults}

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
        self._tx_listeners: List[Callable[[dt.datetime, str], None]] = []
        self._cmd_listeners: List[Callable[[str, Sequence[int], Ack, dt.datetime], None]] = []
        self._idle_hooks: List[Callable[["NanoSession"], None]] = []
        self._lock = threading.RLock()

    def subscribe(self, fn: Callable[[Received], None]) -> None:
        """Every received frame (also those arriving while a command waits for its ACK)."""
        self._subscribers.append(fn)

    def on_transmit(self, fn: Callable[[dt.datetime, str], None]) -> None:
        """Every frame sent to the Nano (for the raw log; retransmissions included)."""
        self._tx_listeners.append(fn)

    def on_command(self, fn: Callable[[str, Sequence[int], Ack, dt.datetime], None]) -> None:
        """Every command that got an ACK: (name, args, ack, sent_utc)."""
        self._cmd_listeners.append(fn)

    def add_idle_hook(self, fn: Callable[["NanoSession"], None]) -> None:
        """Called between receive polls (pump / shell pump), outside any command wait."""
        self._idle_hooks.append(fn)

    def run_idle_hooks(self) -> None:
        with self._lock:
            for fn in self._idle_hooks:
                fn(self)

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
            sent_utc = self.t.now_utc()
            for _attempt in range(self.retries + 1):
                now = self.t.now_utc()
                for fn in self._tx_listeners:
                    fn(now, frame.decode("ascii").rstrip("\n"))
                self.t.write(frame)
                deadline = self.t.now_utc() + dt.timedelta(seconds=self.ack_timeout_s)
                while True:
                    for rec in self._ingest(self.t.read_lines(0.05)):
                        if isinstance(rec.message, Ack) and rec.message.seq == seq:
                            if rec.message.ok:
                                self.last_ok_utc = rec.utc
                            self.faults.on_ack(name, args, rec.message, sent_utc, rec.utc)
                            for fn in self._cmd_listeners:
                                fn(name, args, rec.message, sent_utc)
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
            self.run_idle_hooks()


def trip_name(code: int) -> str:
    return TRIP_NAMES.get(code, str(code))


__all__ = ["ACTUATION_COMMANDS", "READ_ONLY_COMMANDS", "CommandTimeout", "FaultTracker", "NanoSession",
           "Received", "Telemetry", "trip_name"]
