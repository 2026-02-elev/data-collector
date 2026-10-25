# CellGuard data collector - command session, receive dispatch, fault-state tracking
# Author: Yongho Ahn <nanhosoft@gmail.com>
from __future__ import annotations

import bisect
import collections
import dataclasses
import datetime as dt
import threading
from typing import Callable, Deque, Dict, List, Optional, Sequence, Tuple

from .firmware import Capabilities, UnsupportedCommand, capabilities
from .protocol import (Ack, CalibrationInfo, Event, FrameError, Message, Telemetry, UnknownFrameType, Version,
                       TRIP_NAMES, decode_line, encode_frame, parse_message)
from .transport import Transport

ACTUATION_COMMANDS = {"ARM", "RESET", "CHARGE", "DISCHARGE", "ESR", "LEAK", "HEAT", "BENCH", "RAMP", "DRAIN"}
READ_ONLY_COMMANDS = {"SYNC", "PING", "STATUS", "RAW", "CAL", "OFF", "TRIP", "SELFTEST"}


@dataclasses.dataclass
class Received:
    utc: dt.datetime
    raw: str
    message: Optional[Message]
    error: Optional[str] = None
    unknown_type: Optional[str] = None   # CRC-valid frame of a type the parser does not know (not an error)


class CommandTimeout(RuntimeError):
    pass


# reply frame type of a command (CGK / CGD carry the command's seq; CGV / CGS do not)
REPLY_TAG = {"CalibrationInfo": "CGK", "RawAdc": "CGD", "Version": "CGV", "Status": "CGS"}


@dataclasses.dataclass
class Reply:
    """Result of NanoSession.request(): the ACK, and the reply frame that belongs to it."""
    ack: Ack
    message: Optional[Message]
    malformed: List[str]          # CRC-valid or not, frames of the reply's type that did not parse
    waited_s: float

    @property
    def ok(self) -> bool:
        return self.ack.ok and self.message is not None


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

    STOP_EVENTS = {"CHARGE_STOP": "CHARGE", "DISCHARGE_STOP": "DISCHARGE", "DRAIN_STOP": "DISCHARGE",
                   "ESR_STOP": "ESR", "LEAK_STOP": "LEAK", "HEAT_STOP": "HEAT"}
    # power path the Nano REPORTS active (CGE start .. stop), independent of the momentary MOS state in CGT
    PATH_START = {"CHARGE_START": "charge", "DISCHARGE_START": "discharge", "DRAIN_START": "discharge"}
    PATH_STOP = {"CHARGE_STOP": "charge", "DISCHARGE_STOP": "discharge", "DRAIN_STOP": "discharge"}

    def __init__(self) -> None:
        self.requested: Dict[Tuple[str, int], Dict[str, object]] = {}
        self.last_ended: Optional[Dict[str, object]] = None
        self.path_reported = {"charge": False, "discharge": False}
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
        self._hist_v.append((self.requested_struct(), self.power_requested(), self.last_ended,
                             self.power_stage_requested(), dict(self.path_reported)))

    def _at(self, utc: dt.datetime) -> tuple:
        i = bisect.bisect_right(list(self._hist_t), utc)
        if i == 0:
            return None, {"charge": False, "discharge": False}, None, None, {"charge": False, "discharge": False}
        return self._hist_v[i - 1]

    def state_at(self, utc: dt.datetime) -> tuple:
        """(fault_requested, power_requested, fault_request_last_ended) as of ``utc``."""
        return self._at(utc)[:3]

    def stage_at(self, utc: dt.datetime) -> tuple:
        """(power_stage_requested, power_path_reported) as of ``utc``."""
        return self._at(utc)[3:]

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
        if name in ("RAMP", "DRAIN"):  # staged charge / discharge to a rested per-cell target (0.1.6)
            return ("CHARGE" if name == "RAMP" else "DISCHARGE", 0), int(args[0])
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
                self.requested[key] = {"value": value, "since_utc": prev["since_utc"] if prev else _iso(utc),
                                       "command": name}
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
                entry = self.requested.setdefault(key, {"value": value, "since_utc": _iso(utc)})
                entry.update(value=value, command=name)
            else:
                self._end(key, "HOST", utc)
        self._snap(utc)

    def on_event(self, ev: Event, utc: dt.datetime) -> None:
        if ev.code == "BOOT":  # a Nano reset switched every output off
            self._end_all("NANO_RESET", utc)
            self.path_reported = {"charge": False, "discharge": False}
            self._snap(utc)
            return
        path = self.PATH_START.get(ev.code) or self.PATH_STOP.get(ev.code)
        if path is not None:
            self.path_reported[path] = ev.code in self.PATH_START
            self._snap(self.event_clock(ev, utc))
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

    def power_stage_requested(self) -> Optional[Dict[str, object]]:
        """The open power request with its command (RAMP / DRAIN carry a rested per-cell target)."""
        for key in (("CHARGE", 0), ("DISCHARGE", 0)):
            req = self.requested.get(key)
            if req is not None:
                cmd = req.get("command", key[0])
                staged = cmd in ("RAMP", "DRAIN")
                return {"command": cmd, "direction": key[0],
                        "target_mv_per_cell": req["value"] if staged else None, "since_utc": req["since_utc"]}
        return None


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
        self.unknown_frames = 0

    @property
    def capabilities(self) -> Capabilities:
        """What the connected firmware supports (from its CGV; read-only until one was seen)."""
        return capabilities(self.version)

    def subscribe(self, fn: Callable[[Received], None]) -> None:
        """Every received frame (also those arriving while a command waits for its ACK)."""
        self._subscribers.append(fn)

    def unsubscribe(self, fn: Callable[[Received], None]) -> None:
        if fn in self._subscribers:
            self._subscribers.remove(fn)

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
            except UnknownFrameType as exc:
                self.unknown_frames += 1  # CRC was valid: kept as raw evidence, not counted as an error
                rec = Received(utc, line, None, None, unknown_type=exc.kind)
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
        """Sends one command; on timeout re-sends the SAME seq (the Nano de-duplicates).

        Raises UnsupportedCommand (and sends nothing) when the detected firmware does not
        support the command - e.g. SELFTEST or CHARGE,1 on 0.1.6-LOAD-VERIFY.
        """
        self.capabilities.check(name, args)
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

    def request(self, name: str, *args: int, reply: type, timeout_s: float = 1.0, match_seq: bool = True) -> Reply:
        """Sends a command and waits for the frame it causes (CAL -> CGK, RAW -> CGD, SYNC -> CGV, STATUS -> CGS).

        The Nano sends the ACK first and the reply right after it; on a real UART they often arrive in
        different reads, so the ACK alone says nothing about the reply. Only a reply received AFTER the
        command was sent counts (an older CGK / CGD is never reused); with ``match_seq`` it must also carry
        this command's seq. Raises CommandTimeout only for a missing ACK.
        """
        tag = REPLY_TAG.get(reply.__name__, "")
        got: List[Message] = []
        bad: List[str] = []

        def grab(rec: Received) -> None:
            if isinstance(rec.message, reply):
                got.append(rec.message)
            elif rec.message is None and tag and rec.raw.startswith("$" + tag + ","):
                bad.append(f"{rec.raw} ({rec.error or rec.unknown_type})")

        with self._lock:
            self.subscribe(grab)  # before sending: the reply may come in the same read as the ACK
            try:
                ack = self.command(name, *args)
                t0 = self.t.monotonic()
                if not ack.ok:
                    return Reply(ack, None, bad, 0.0)

                def match() -> Optional[Message]:
                    for m in got:
                        if not match_seq or getattr(m, "seq", None) == ack.seq:
                            return m
                    return None

                while match() is None and self.t.monotonic() - t0 < timeout_s:
                    self._ingest(self.t.read_lines(0.05))
                return Reply(ack, match(), bad, round(self.t.monotonic() - t0, 3))
            finally:
                self.unsubscribe(grab)

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
           "Received", "Reply", "Telemetry", "UnsupportedCommand", "trip_name"]
