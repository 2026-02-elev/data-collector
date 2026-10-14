# CellGuard data collector - rotating local storage and upload outbox
# Author: Yongho Ahn <nanhosoft@gmail.com>
"""Local persistence designed for week-long runs and later server upload.

Layout under ``data_dir``::

    spool/<kind>-<start>-<n>.jsonl.part   file being written (flushed every record)
    outbox/<kind>-<start>-<n>.jsonl       closed files, ready to be sent to the server
    sent/<kind>-<start>-<n>.jsonl         files an uploader confirmed
    quarantine/                           crash recovery reports and rejected partial lines

Files rotate by size and age, so memory and single-file size stay bounded.
Raw UART frames and normalized samples are stored separately: the raw log is
the measurement evidence (``raw_log_ref``), normalized samples are derived.
"""
from __future__ import annotations

import datetime as dt
import json
import os
from pathlib import Path
from typing import Callable, List, Optional

UTC = dt.timezone.utc


def _stamp(t: dt.datetime) -> str:
    return t.astimezone(UTC).strftime("%Y%m%dT%H%M%S%fZ")


class RotatingWriter:
    def __init__(self, data_dir: Path, kind: str, max_bytes: int = 50_000_000, max_age_s: float = 3600.0,
                 clock: Callable[[], dt.datetime] = lambda: dt.datetime.now(UTC)):
        self.dir = Path(data_dir)
        self.kind = kind
        self.max_bytes = max_bytes
        self.max_age_s = max_age_s
        self.clock = clock
        for sub in ("spool", "outbox", "sent", "quarantine"):
            (self.dir / sub).mkdir(parents=True, exist_ok=True)
        # A previous run that crashed leaves .part files: recover the valid records.
        self.recovered = [recover_part(stale, self.dir) for stale in
                          sorted((self.dir / "spool").glob(f"{kind}-*.jsonl.part"))]
        self._f = None
        self._path: Optional[Path] = None
        self._opened: Optional[dt.datetime] = None
        self._bytes = 0
        self._seq = 0
        self.closed_files: List[Path] = []

    @property
    def current_name(self) -> Optional[str]:
        return self._path.name[:-len(".part")] if self._path else None

    def _open(self) -> None:
        self._opened = self.clock()
        while True:  # sequence number: never reuse a name, even with an identical timestamp
            self._seq += 1
            name = f"{self.kind}-{_stamp(self._opened)}-{self._seq:05d}.jsonl"
            if not any((self.dir / d / n).exists() for d in ("spool", "outbox", "sent")
                       for n in (name, name + ".part")):
                break
        self._path = self.dir / "spool" / (name + ".part")
        self._f = open(self._path, "a", encoding="utf-8")
        self._bytes = 0

    def write_line(self, line: str) -> None:
        now = self.clock()
        if self._f is not None and (self._bytes >= self.max_bytes or
                                    (now - self._opened).total_seconds() >= self.max_age_s):  # type: ignore[operator]
            self.rotate()
        if self._f is None:
            self._open()
        data = line.rstrip("\n") + "\n"
        self._f.write(data)  # type: ignore[union-attr]
        self._f.flush()  # type: ignore[union-attr]
        self._bytes += len(data.encode("utf-8"))

    def write_record(self, rec: dict) -> None:
        self.write_line(json.dumps(rec, separators=(",", ":"), ensure_ascii=False))

    def rotate(self) -> Optional[Path]:
        if self._f is None or self._path is None:
            return None
        self._f.flush()
        os.fsync(self._f.fileno())
        self._f.close()
        final = self.dir / "outbox" / self._path.name[:-len(".part")]
        os.replace(self._path, final)
        self.closed_files.append(final)
        self._f, self._path = None, None
        return final

    def close(self) -> Optional[Path]:
        return self.rotate()


def recover_part(part: Path, data_dir: Path) -> dict:
    """Moves an unfinished spool file to the outbox keeping only complete, valid JSON lines.

    A crash usually leaves a truncated last line; any line that does not parse
    as a JSON object is moved byte-for-byte to ``quarantine/`` (never silently
    dropped) and a ``<name>.recovery.json`` report is written next to it.
    """
    data_dir = Path(data_dir)
    name = part.name[:-len(".part")]
    raw = part.read_bytes()
    good: List[bytes] = []
    bad: List[bytes] = []
    lines = raw.split(b"\n")
    for i, line in enumerate(lines):
        if not line.strip():
            continue
        complete = i < len(lines) - 1  # the last element has no terminating newline
        try:
            ok = complete and isinstance(json.loads(line.decode("utf-8")), dict)
        except (UnicodeDecodeError, ValueError):
            ok = False
        (good if ok else bad).append(line)
    final = data_dir / "outbox" / name
    tmp = final.with_suffix(".jsonl.tmp")
    with open(tmp, "wb") as f:
        for line in good:
            f.write(line + b"\n")
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, final)
    report = {"file": name, "recovered_records": len(good), "quarantined_lines": len(bad),
              "original_bytes": len(raw)}
    if bad:
        with open(data_dir / "quarantine" / (name + ".rejected"), "wb") as f:
            for line in bad:
                f.write(line + b"\n")
    (data_dir / "quarantine" / (name + ".recovery.json")).write_text(json.dumps(report), encoding="utf-8")
    part.unlink()
    return report


class Uploader:
    """Server transfer interface (HTTP / MQTT backend to be implemented later)."""

    def upload(self, path: Path) -> bool:
        raise NotImplementedError


class Outbox:
    def __init__(self, data_dir: Path):
        self.dir = Path(data_dir)

    def ready(self) -> List[Path]:
        return sorted((self.dir / "outbox").glob("*.jsonl"))

    def flush(self, uploader: Uploader) -> int:
        """Uploads every closed file; a file moves to sent/ only after a confirmed upload."""
        sent = 0
        for path in self.ready():
            if uploader.upload(path):
                os.replace(path, self.dir / "sent" / path.name)
                sent += 1
        return sent
