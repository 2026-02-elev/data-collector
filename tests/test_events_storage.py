# fault_event_v1 records from commands/CGE, crash recovery of spool files
# Author: Yongho Ahn <nanhosoft@gmail.com>
import json

import jsonschema
import pytest

from cg_collector.storage import RotatingWriter, recover_part
from test_fusion_current import T0, at, fr, run, telemetry_lines


@pytest.fixture(scope="module")
def event_schema():
    from pathlib import Path

    return json.loads((Path(__file__).resolve().parents[1] / "schemas" / "fault_event_v1.schema.json").read_text())


def test_events_from_commands_and_cge(event_schema):
    lines = telemetry_lines(range(4))
    lines += [
        (at(1500), "P>N", fr("CGC,7,ESR,2,1")),
        (at(1512), "N>P", fr("CGA,7,OK,NONE")),
        (at(1513), "N>P", fr("CGE,5,1503,ESR_START,2,1,1,HOST")),
        (at(1514), "N>P", fr("CGE,5,1503,ESR_START,2,1,1,HOST")),  # duplicate frame
        (at(2500), "P>N", fr("CGC,8,HEAT,1,40")),
        (at(2512), "N>P", fr("CGA,8,ERR,HARDWARE_LOCKED")),
        (at(3600), "N>P", fr("CGE,9,3590,TRIP,0,0,0,HEARTBEAT_TIMEOUT")),  # eventSeq 6..8 lost
        (at(3601), "N>P", fr("CGE,10,3590,ESR_STOP,2,1,0,TRIP")),
    ]
    lines.sort(key=lambda x: x[0])
    out, events = run(lines)
    kinds = [(e["event_type"], e["fault_type"], e["cell_id"]) for e in events]
    assert kinds == [("REQUESTED", "ESR", 2), ("COMMANDED", "ESR", 2), ("REQUESTED", "HEAT", 1),
                     ("FAULT_REJECTED", "HEAT", 1), ("TRIPPED", "NONE", None), ("COMMANDED", "ESR", 2)]
    assert events[1]["evidence"]["cge"]["cge_code"] == "ESR_START"
    assert events[4]["evidence"]["cge"]["events_lost_before"] == 3
    assert events[1]["source"] == "measured_fault_injection" and events[4]["source"] == "measured_normal"
    for e in events:
        jsonschema.validate(e, event_schema)
        assert e["physical_fault_verified"] is False and e["scenario_domain"] == "measured_hardware"
    # The request ended with the Nano-side stop (seen as of the slot time).
    after = [r for r in out if r["record_status"] == "OK" and r["t_sim_s"] >= 4.0]
    assert after and after[0]["fault_request_last_ended"]["reason"] == "TRIP"
    during = [r for r in out if r["t_sim_s"] == 2.0][0]
    assert during["fault_requested"]["cell_id"] == 2


def test_partial_spool_file_recovery(tmp_path):
    spool = tmp_path / "spool"
    spool.mkdir()
    good = [json.dumps({"i": i}) for i in range(3)]
    (spool / "samples-x-00001.jsonl.part").write_bytes(("\n".join(good) + "\n{\"i\": 3, \"trunc").encode())
    (spool / "samples-y-00001.jsonl.part").write_bytes(b'{"i": 0}\nnot json\n{"i": 2}\n')
    w = RotatingWriter(tmp_path, "samples", clock=lambda: T0)
    reports = {r["file"]: r for r in w.recovered}
    assert reports["samples-x-00001.jsonl"]["recovered_records"] == 3
    assert reports["samples-x-00001.jsonl"]["quarantined_lines"] == 1
    assert reports["samples-y-00001.jsonl"]["recovered_records"] == 2
    rows = [json.loads(l) for l in (tmp_path / "outbox" / "samples-x-00001.jsonl").read_text().splitlines()]
    assert rows == [{"i": 0}, {"i": 1}, {"i": 2}]
    assert (tmp_path / "quarantine" / "samples-x-00001.jsonl.rejected").read_bytes() == b'{"i": 3, "trunc\n'
    assert not list(spool.glob("*.part"))


def test_recover_part_empty_file(tmp_path):
    (tmp_path / "outbox").mkdir()
    (tmp_path / "quarantine").mkdir()
    part = tmp_path / "raw-z.jsonl.part"
    part.write_bytes(b"")
    rep = recover_part(part, tmp_path)
    assert rep["recovered_records"] == 0 and (tmp_path / "outbox" / "raw-z.jsonl").exists()
