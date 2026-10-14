# CellGuard data collector - offline re-processing of UART logs
# Author: Yongho Ahn <nanhosoft@gmail.com>
"""Runs a recorded UART conversation through the same Collector/EventRecorder
code as a live run. Accepted inputs:

* this collector's raw log (JSONL: ``{"pi_received_utc", "frame"}`` for Nano->Pi,
  ``{"pi_sent_utc", "frame", "dir": "P>N"}`` for Pi->Nano), and
* the data-generator's ``nano_uart.log`` (``<t_s> N>P|P>N <frame>``; t relative
  to the run's ``fixed_epoch_utc``).

Used to regenerate normalized samples after a parser fix (raw logs are the
evidence; derived files can be rebuilt) and to verify that synthetic UART
streams produce the same records through the real collector code.
"""
from __future__ import annotations

import datetime as dt
import json
from pathlib import Path
from typing import Callable, Dict, Iterable, Iterator, List, Optional, Tuple

from .fusion import Collector, FusionContext
from .events import EventRecorder
from .protocol import Ack, FrameError, decode_line, parse_message
from .session import NanoSession, Received
from .transport import Transport

UTC = dt.timezone.utc
Line = Tuple[dt.datetime, str, str]  # (utc, "N>P" | "P>N", frame text)


def read_generator_log(path: Path, epoch: dt.datetime) -> Iterator[Line]:
    for raw in Path(path).read_text(encoding="ascii", errors="replace").splitlines():
        parts = raw.split(" ", 2)
        if len(parts) != 3 or parts[1] not in ("N>P", "P>N"):
            continue
        yield epoch + dt.timedelta(microseconds=int(round(float(parts[0]) * 1e6))), parts[1], parts[2]


def read_collector_log(paths: Iterable[Path]) -> Iterator[Line]:
    for path in paths:
        for raw in Path(path).read_text(encoding="utf-8").splitlines():
            if not raw.strip():
                continue
            r = json.loads(raw)
            if r.get("dir") == "P>N":
                yield _parse_utc(r["pi_sent_utc"]), "P>N", r["frame"]
            else:
                yield _parse_utc(r["pi_received_utc"]), "N>P", r["frame"]


def _parse_utc(text: str) -> dt.datetime:
    return dt.datetime.fromisoformat(text.replace("Z", "+00:00")).astimezone(UTC)


class _ClockTransport(Transport):
    def __init__(self, start: dt.datetime):
        self.now = start

    def now_utc(self) -> dt.datetime:
        return self.now

    def write(self, data: bytes) -> None:  # replay never transmits
        raise RuntimeError("replay is read-only")

    def read_lines(self, timeout_s: float) -> List[str]:
        return []


def replay(lines: Iterable[Line], ctx: FusionContext, sample_sink: Callable[[dict], None],
           event_sink: Optional[Callable[[dict], None]] = None, run_start: Optional[dt.datetime] = None,
           end_utc: Optional[dt.datetime] = None) -> Collector:
    it = iter(lines)
    first = next(it, None)
    if first is None:
        raise ValueError("empty log")
    start = run_start or first[0]
    t = _ClockTransport(start)
    s = NanoSession(t)
    col = Collector(s, ctx, sample_sink, request_raw=True, run_start_utc=start)
    if event_sink is not None:
        EventRecorder(s, col, event_sink)
    sent: Dict[int, Tuple[str, List[int], dt.datetime]] = {}

    def match_ack(rec: Received) -> None:
        msg = rec.message
        if isinstance(msg, Ack) and msg.seq in sent:
            name, args, sent_utc = sent.pop(msg.seq)
            if msg.ok:
                s.last_ok_utc = rec.utc
            s.faults.on_ack(name, args, msg, sent_utc, rec.utc)
            for fn in s._cmd_listeners:
                fn(name, args, msg, sent_utc)

    s.subscribe(match_ack)
    for utc, direction, frame in [first, *it]:
        t.now = utc
        if direction == "P>N":
            try:
                f = decode_line(frame).split(",")
            except FrameError:
                continue
            if len(f) >= 3 and f[0] == "CGC" and f[1].isdigit():
                seq = int(f[1])
                if seq not in sent:  # retransmissions keep the first send time
                    sent[seq] = (f[2], [int(a) for a in f[3:] if a.lstrip("-").isdigit()], utc)
        else:
            s._ingest([frame])
        col.tick(col.t_of(utc))  # same order as a live run: receive, then the idle hook
    if end_utc is not None:
        t.now = end_utc
    col.close(col.t_of(t.now))
    return col


__all__ = ["read_collector_log", "read_generator_log", "replay", "parse_message"]
