# CellGuard data collector - offline re-processing of raw logs (streaming)
# Author: Yongho Ahn <nanhosoft@gmail.com>
"""Runs recorded raw evidence through the same Collector/EventRecorder code as a
live run. Accepted inputs:

* this collector's raw logs: ``raw-*.jsonl`` (UART, both directions) and
  ``ina219-*.jsonl`` (INA219 reads, ``ina219_raw_v1``), and
* the data-generator's ``nano_uart.log`` (``<t_s> N>P|P>N <frame>``, t relative
  to the run's ``fixed_epoch_utc``) and ``ina219_raw.jsonl``.

Everything is streamed: files are read line by line and the UART and INA219
streams are merged by time (``heapq.merge``), so a week of logs is processed
with bounded memory.

INA219 readings enter the current sampler at their ``available_utc`` (when the
Pi had them), so the joined current is the one the live run would have used.
"""
from __future__ import annotations

import datetime as dt
import heapq
import json
from pathlib import Path
from typing import Callable, Dict, Iterable, Iterator, List, Optional, Sequence, Tuple, Union

from .current import CurrentReading, CurrentSampler
from .events import EventRecorder
from .fusion import Collector, FusionContext
from .protocol import Ack, FrameError, decode_line
from .session import NanoSession, Received
from .transport import Transport

UTC = dt.timezone.utc
# (utc, kind, payload): kind "N>P" / "P>N" (payload = frame text) or "INA" (payload = CurrentReading)
Item = Tuple[dt.datetime, str, Union[str, CurrentReading]]
Line = Tuple[dt.datetime, str, str]


def _parse_utc(text: str) -> dt.datetime:
    return dt.datetime.fromisoformat(text.replace("Z", "+00:00")).astimezone(UTC)


def _lines(path: Path, encoding: str = "utf-8") -> Iterator[str]:
    with open(path, encoding=encoding, errors="replace") as f:
        for line in f:  # one line at a time, never the whole file
            yield line.rstrip("\n")


def read_generator_log(path: Path, epoch: dt.datetime) -> Iterator[Line]:
    for raw in _lines(Path(path), "ascii"):
        parts = raw.split(" ", 2)
        if len(parts) != 3 or parts[1] not in ("N>P", "P>N"):
            continue
        yield epoch + dt.timedelta(microseconds=int(round(float(parts[0]) * 1e6))), parts[1], parts[2]


def read_collector_log(paths: Iterable[Path]) -> Iterator[Line]:
    for path in paths:
        for raw in _lines(Path(path)):
            if not raw.strip():
                continue
            r = json.loads(raw)
            if r.get("dir") == "RUN":  # run header (time origin), not a frame
                continue
            if r.get("dir") == "P>N":
                yield _parse_utc(r["pi_sent_utc"]), "P>N", r["frame"]
            else:
                yield _parse_utc(r["pi_received_utc"]), "N>P", r["frame"]


def read_ina219_log(paths: Iterable[Path]) -> Iterator[Tuple[dt.datetime, str, CurrentReading]]:
    """ina219_raw_v1 lines, keyed by the time the reading became available to the Pi."""
    for path in paths:
        for raw in _lines(Path(path)):
            if raw.strip():
                r = CurrentReading.from_raw_record(json.loads(raw))
                yield r.available_utc or r.sampled_utc, "INA", r  # type: ignore[misc]


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
           end_utc: Optional[dt.datetime] = None,
           ina219: Optional[Iterable[Tuple[dt.datetime, str, CurrentReading]]] = None) -> Collector:
    """Streams ``lines`` (time ordered) and optional INA219 readings through a Collector.

    Without ``ina219`` the records say INA219_UNAVAILABLE (as a live run without
    ``--ina219``); with it, the current fields are rebuilt exactly.
    """
    streams: List[Iterable[Item]] = [lines]  # type: ignore[list-item]
    if ina219 is not None:
        streams.append(ina219)  # type: ignore[arg-type]
    merged = heapq.merge(*streams, key=lambda item: item[0]) if len(streams) > 1 else iter(streams[0])
    first = next(merged, None)
    if first is None:
        raise ValueError("empty log")
    start = run_start or first[0]
    t = _ClockTransport(start)
    s = NanoSession(t)
    sampler = CurrentSampler(None, clock=t.now_utc) if ina219 is not None else None
    col = Collector(s, ctx, sample_sink, current=sampler, request_raw=True, run_start_utc=start)
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
    item: Optional[Item] = first
    while item is not None:
        utc, kind, payload = item
        t.now = utc
        if kind == "INA":
            sampler.feed(payload)  # type: ignore[union-attr,arg-type]
        elif kind == "P>N":
            _note_command(sent, payload, utc)  # type: ignore[arg-type]
        else:
            s._ingest([payload])  # type: ignore[list-item]
            col.tick(col.t_of(utc))  # same order as a live run: receive, then the idle hook
        item = next(merged, None)
    if end_utc is not None:
        t.now = end_utc
    col.close(col.t_of(t.now))
    return col


def _note_command(sent: Dict[int, Tuple[str, List[int], dt.datetime]], frame: str, utc: dt.datetime) -> None:
    try:
        f = decode_line(frame).split(",")
    except FrameError:
        return
    if len(f) >= 3 and f[0] == "CGC" and f[1].isdigit():
        seq = int(f[1])
        if seq not in sent:  # retransmissions keep the first send time
            sent[seq] = (f[2], [int(a) for a in f[3:] if a.lstrip("-").isdigit()], utc)
        if len(sent) > 1024:  # commands that never got an ACK must not accumulate
            sent.pop(next(iter(sent)))


def replay_generator_run(run_dir: Path, ctx: FusionContext, sample_sink: Callable[[dict], None],
                         event_sink: Optional[Callable[[dict], None]] = None, with_current: bool = True) -> Collector:
    run_dir = Path(run_dir)
    man = json.loads((run_dir / "manifest.json").read_text())
    epoch = _parse_utc(man["fixed_epoch_utc"])
    ina_path = run_dir / "ina219_raw.jsonl"
    # an absent sensor leaves the log empty: same as a live run without --ina219 (INA219_UNAVAILABLE)
    has_ina = with_current and ina_path.exists() and ina_path.stat().st_size > 0
    ina = read_ina219_log([ina_path]) if has_ina else None
    return replay(read_generator_log(run_dir / "nano_uart.log", epoch), ctx, sample_sink, event_sink,
                  run_start=epoch, end_utc=epoch + dt.timedelta(seconds=float(man["duration_s"])), ina219=ina)


__all__ = ["read_collector_log", "read_generator_log", "read_ina219_log", "replay", "replay_generator_run"]
