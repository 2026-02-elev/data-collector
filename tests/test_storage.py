# Author: Yongho Ahn <nanhosoft@gmail.com>
import datetime as dt
import json

from cg_collector.storage import Outbox, RotatingWriter, Uploader

T0 = dt.datetime(2026, 10, 14, 0, 0, 0, tzinfo=dt.timezone.utc)


def test_rotation_by_size_and_age(tmp_path):
    now = [T0]
    w = RotatingWriter(tmp_path, "samples", max_bytes=200, max_age_s=60, clock=lambda: now[0])
    for i in range(20):
        w.write_record({"i": i, "pad": "x" * 20})
    assert len(w.closed_files) >= 3  # size-based rotation
    now[0] += dt.timedelta(seconds=61)
    before = len(w.closed_files)
    w.write_record({"i": 99})
    assert len(w.closed_files) == before + 1  # age-based rotation
    w.close()
    rows = [json.loads(l) for f in sorted((tmp_path / "outbox").glob("*.jsonl")) for l in f.read_text().splitlines()]
    assert [r["i"] for r in rows] == list(range(20)) + [99]
    assert not list((tmp_path / "spool").glob("*.part"))


def test_outbox_moves_only_confirmed_uploads(tmp_path):
    w = RotatingWriter(tmp_path, "raw", clock=lambda: T0)
    w.write_line("a")
    w.close()

    class Flaky(Uploader):
        calls = 0

        def upload(self, path):
            self.calls += 1
            return self.calls > 1

    box, up = Outbox(tmp_path), Flaky()
    assert box.flush(up) == 0 and len(box.ready()) == 1
    assert box.flush(up) == 1 and box.ready() == []
    assert len(list((tmp_path / "sent").glob("*.jsonl"))) == 1
