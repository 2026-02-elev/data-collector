# CellGuard data collector - import cellguard_cycle.py serial traces
# Author: Yongho Ahn <nanhosoft@gmail.com>
"""Converts ``cellguard_cycle_YYYYMMDD_HHMMSS.log`` (written by the rig's
``cellguard_cycle.py``, CellGuardNano_LoadVerify 0.1.6) into the collector's
standard data:

    2026-10-25 13:28:40.140 RX $CGY,...*23D2
    2026-10-25 13:28:40.146 TX $CGC,12,PING*....

* every line is kept as raw evidence (``raw.jsonl``) with its direction
  (RX = Nano->Pi ``N>P``, TX = Pi->Nano ``P>N``), the original local timestamp
  text, the source file and line number, and its CRC / format verdict;
* the frames are replayed through the same Collector / EventRecorder /
  DiagnosticRecorder as a live run -> ``samples.jsonl`` (normalized_sample_v1),
  ``events.jsonl`` (fault_event_v1), ``diagnostics.jsonl`` (nano_diagnostic_v1);
* nothing is invented: there is no INA219 in these traces, so the current is
  null with ``INA219_UNAVAILABLE``; CGY (on-load suspect taps) goes to the
  diagnostics only, never into a cell voltage; missing telemetry stays missing;
* every record's ``raw_log_ref`` is ``<log file>:<line>`` of the frame it came from.

The trace has no time zone: pass the Pi's zone (``--log-tz Asia/Seoul``); the
default is this computer's local zone, and the zone used is recorded.
"""
from __future__ import annotations

import datetime as dt
import hashlib
import json
import re
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Tuple

from .fusion import FusionContext, rfc3339_us
from .protocol import FrameError, UnknownFrameType, decode_line, parse_message
from .replay import replay

LINE_RE = re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3}) (RX|TX) (.*)$")
UTC = dt.timezone.utc


def _zone(name: Optional[str]) -> dt.tzinfo:
    if not name or name == "local":
        return dt.datetime.now().astimezone().tzinfo  # type: ignore[return-value]
    if name.upper() == "UTC":
        return UTC
    from zoneinfo import ZoneInfo

    return ZoneInfo(name)


def read_cycle_log(path: Path, tz: dt.tzinfo) -> Iterator[Tuple[int, Optional[Tuple[dt.datetime, str, str, str]], str]]:
    """(line number, (utc, dir, frame, local stamp) or None when the line is not a trace line, text)."""
    with open(path, encoding="ascii", errors="replace") as f:
        for n, line in enumerate(f, 1):
            text = line.rstrip("\n").rstrip("\r")
            m = LINE_RE.match(text)
            if not m:
                yield n, None, text
                continue
            local = dt.datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S.%f").replace(tzinfo=tz)
            yield n, (local.astimezone(UTC), "N>P" if m.group(2) == "RX" else "P>N", m.group(3), m.group(1)), text


def _verdict(frame: str) -> Tuple[Optional[str], Optional[str], Optional[str]]:
    """(error, frame type, unknown type) of one frame."""
    try:
        payload = decode_line(frame)
    except FrameError as exc:
        return str(exc), None, None
    kind = payload.split(",", 1)[0]
    if kind == "CGC":  # Pi command (TX)
        return None, kind, None
    try:
        parse_message(payload)
    except UnknownFrameType as exc:
        return None, kind, exc.kind
    except FrameError as exc:
        return str(exc), kind, None
    return None, kind, None


def import_cycle_log(paths: List[Path], out: Path, ctx: FusionContext, tz_name: Optional[str] = None) -> Dict:
    out.mkdir(parents=True, exist_ok=True)
    tz = _zone(tz_name)
    report: Dict[str, object] = {"schema_version": "cycle_log_import_v1", "sources": [], "log_timezone": str(tz),
                                 "lines": 0, "rx": 0, "tx": 0, "crc_or_format_errors": 0, "unknown_types": 0,
                                 "non_trace_lines": [], "frame_types": {}, "firmware_versions": [],
                                 "time_reversals": [],
                                 "ina219": "not in trace (current null, INA219_UNAVAILABLE)"}
    items: List[Tuple[dt.datetime, str, str, str]] = []
    raw_rows: List[dict] = []
    types: Dict[str, int] = {}
    for path in paths:
        path = Path(path)
        report["sources"].append({"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
        for n, parsed, text in read_cycle_log(path, tz):
            report["lines"] = int(report["lines"]) + 1  # type: ignore[arg-type]
            ref = f"{path.name}:{n}"
            if parsed is None:
                if text.strip() and len(report["non_trace_lines"]) < 50:  # type: ignore[arg-type]
                    report["non_trace_lines"].append({"ref": ref, "text": text[:200]})  # type: ignore[union-attr]
                continue
            utc, direction, frame, local = parsed
            error, kind, unknown = _verdict(frame)
            report["rx" if direction == "N>P" else "tx"] = int(report["rx" if direction == "N>P" else "tx"]) + 1
            if error:
                report["crc_or_format_errors"] = int(report["crc_or_format_errors"]) + 1
            if unknown:
                report["unknown_types"] = int(report["unknown_types"]) + 1
            if kind:
                types[kind] = types.get(kind, 0) + 1
            if kind == "CGV" and not error:
                fw = decode_line(frame).split(",")[1]
                if fw not in report["firmware_versions"]:  # type: ignore[operator]
                    report["firmware_versions"].append(fw)  # type: ignore[union-attr]
            key = "pi_received_utc" if direction == "N>P" else "pi_sent_utc"
            raw_rows.append({key: rfc3339_us(utc), "dir": direction, "frame": frame, "error": error,
                             "unknown_type": unknown, "log_local_time": local, "source_log": path.name,
                             "source_line": n})
            items.append((utc, direction, frame, ref))
    report["frame_types"] = types
    if not items:
        raise ValueError("no RX/TX trace lines found")
    # The file order is the order things happened. A clock step backwards (NTP correction on the Pi) is
    # reported and the replay clock held, never re-sorted; raw.jsonl keeps the original stamps.
    held = []
    last = items[0][0]
    for utc, direction, frame, ref in items:
        if utc < last:
            if len(report["time_reversals"]) < 50:  # type: ignore[arg-type]
                report["time_reversals"].append({"ref": ref, "back_by_s": round((last - utc).total_seconds(), 3)})
            utc = last
        last = utc
        held.append((utc, direction, frame, ref))
    items = held
    start = items[0][0]
    ctx.raw_log_ref = items[0][3]
    with open(out / "raw.jsonl", "w", encoding="utf-8") as f:
        f.write(json.dumps({"dir": "RUN", "run_start_utc": rfc3339_us(start), "run_id": ctx.run_id,
                            "device_id": ctx.device_id, "pack_id": ctx.pack_id, "scenario_id": ctx.scenario_id,
                            "source": "cellguard_cycle_log", "log_timezone": str(tz),
                            "source_logs": report["sources"]}, separators=(",", ":")) + "\n")
        for row in raw_rows:
            f.write(json.dumps(row, separators=(",", ":")) + "\n")
    files = {k: open(out / f"{k}.jsonl", "w", encoding="utf-8") for k in ("samples", "events", "diagnostics")}
    sinks = {k: (lambda fh: (lambda r: fh.write(json.dumps(r, separators=(",", ":")) + "\n")))(fh)
             for k, fh in files.items()}
    try:
        col = replay(iter(items), ctx, sinks["samples"], sinks["events"], run_start=start, end_utc=items[-1][0],
                     diag_sink=sinks["diagnostics"])
    finally:
        for fh in files.values():
            fh.close()
    report["collector_counters"] = col.counters
    report["records"] = {k: sum(1 for _ in open(out / f"{k}.jsonl", encoding="utf-8"))
                         for k in ("samples", "events", "diagnostics")}
    (out / "import_report.json").write_text(json.dumps(report, indent=1, ensure_ascii=False), encoding="utf-8")
    return report


__all__ = ["import_cycle_log", "read_cycle_log"]
