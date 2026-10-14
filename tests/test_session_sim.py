# Integration against the firmware host simulator (SIMULATED DEVICE)
# Author: Yongho Ahn <nanhosoft@gmail.com>
import jsonschema
import pytest

from cg_collector import protocol as p
from cg_collector.fusion import Collector, FusionContext
from cg_collector.runner import PlanError, RunAborted, parse_plan, run_steps
from cg_collector.session import NanoSession
from cg_collector.transport import SimTransport, Transport


class LossyTransport(Transport):
    def __init__(self, inner, drop=0, duplicate=False):
        self.inner, self.drop, self.duplicate, self.writes = inner, drop, duplicate, 0

    def write(self, data):
        self.writes += 1
        if self.drop > 0:
            self.drop -= 1
            return
        self.inner.write(data)
        if self.duplicate:
            self.inner.write(data)

    def read_lines(self, timeout_s):
        return self.inner.read_lines(timeout_s)

    def now_utc(self):
        return self.inner.now_utc()

    def close(self):
        self.inner.close()


@pytest.fixture
def sim(sim_path):
    created = []

    def make(gates="locked"):
        t = SimTransport(sim_path, gates=gates)
        created.append(t)
        return t

    yield make
    for t in created:
        t.close()


def test_retransmits_same_seq(sim):
    t = LossyTransport(sim(), drop=1)
    s = NanoSession(t)
    ack = s.command("SYNC")
    assert ack.ok and ack.seq == 1 and t.writes == 2


def test_duplicate_delivery_executes_once(sim):
    t = LossyTransport(sim("unlocked"), duplicate=True)
    s = NanoSession(t)
    t.inner.control("ADV 600")
    assert s.command("SYNC").ok and s.command("ARM").ok
    s.poll(0.05)
    events = [r.message for r in s.recent if isinstance(r.message, p.Event) and r.message.code == "ARM"]
    acks = [r.message for r in s.recent if isinstance(r.message, p.Ack) and r.message.seq == 2]
    assert len(events) == 1 and len(acks) == 2


def test_no_telemetry_lost_while_commands_wait(sim, normalized_schema):
    t = sim()
    s = NanoSession(t, recent_size=8)
    out = []
    col = Collector(s, FusionContext(run_id="sim"), out.append)
    cgt = []
    s.subscribe(lambda r: cgt.append(r) if isinstance(r.message, p.Telemetry) else None)
    s.command("SYNC")
    for _ in range(40):  # mostly time spent inside command() waiting for ACKs
        s.command("STATUS")
        s.command("PING")
        s.poll(0.25)
    col.close()
    assert len(cgt) >= 10
    oks = [r for r in out if r["record_status"] == "OK"]
    assert len(oks) == len(cgt)  # every CGT became a record, none lost
    assert len(s.recent) <= 8 and len(col.recent) <= 32  # bounded memory
    seqs = [r["telemetry_seq"] for r in oks]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)
    for rec in out:
        jsonschema.validate(rec, normalized_schema)
        assert rec["source"] == "synthetic" and rec["pack_current_a"] is None


def test_sequence_runner_keeps_one_session(sim):
    t = sim("unlocked")
    t.control("CELLS 800 800 800 800")
    s = NanoSession(t)
    out = []
    Collector(s, FusionContext(run_id="plan"), out.append)
    steps = parse_plan({"steps": ["SYNC", {"wait": 1}, "ARM", {"cmd": "CHARGE", "args": [1]}, {"wait": 8},
                                  {"cmd": "ESR", "args": [2, 1]}, {"wait": 2}, {"cmd": "ESR", "args": [2, 0]},
                                  {"cmd": "CHARGE", "args": [0]}, {"wait": 1}]})
    results = run_steps(s, steps, supervised=True)
    assert [r.ok for r in results] == [True] * len(results)
    assert results[-1].op == "OFF"
    charging = [r for r in out if r["charge_commanded"]]
    assert len(charging) >= 7  # heartbeat kept the session alive through the 8 s wait
    assert any(r["esr_command_mask"] == 2 and r["fault_requested"]["cell_id"] == 2 for r in out)
    assert all(r["trip_reason"] == "NONE" for r in out)


def test_runner_requires_supervision_and_aborts_on_refusal(sim):
    steps = parse_plan({"steps": ["SYNC", {"wait": 1}, "ARM", {"cmd": "CHARGE", "args": [1]}]})
    s = NanoSession(sim("locked"))
    with pytest.raises(PlanError):
        run_steps(s, steps, supervised=False)
    with pytest.raises(RunAborted):  # default firmware build: ARM -> HARDWARE_LOCKED
        run_steps(s, steps, supervised=True)
    assert s.faults.requested == {}


def test_request_cleared_when_nano_trips(sim):
    t = sim("unlocked")
    s = NanoSession(t)
    t.control("ADV 600")
    s.command("SYNC")
    s.command("ARM")
    assert s.command("LEAK", 1, 1).ok
    assert ("LEAK", 1) in s.faults.requested
    s.pump(6, keepalive_s=None)  # no heartbeat -> HEARTBEAT_TIMEOUT trip on the Nano
    assert s.faults.requested == {}
    assert s.faults.last_ended["reason"] in ("TRIP", "TRIP_HEARTBEAT_TIMEOUT")


def test_plan_validation():
    for bad in ({}, {"steps": []}, {"steps": ["ARM"]}, {"steps": ["SYNC", {"cmd": "ESR", "args": [1]}]},
                {"steps": ["SYNC", {"cmd": "FLY"}]}, {"steps": ["SYNC", {"wait": -1}]}):
        with pytest.raises(PlanError):
            parse_plan(bad)


class DropTelemetry(Transport):
    """Loses CGT frames between two simulated times (receive side)."""

    def __init__(self, inner, start_s, end_s):
        self.inner, self.start_s, self.end_s = inner, start_s, end_s

    def write(self, data):
        self.inner.write(data)

    def read_lines(self, timeout_s):
        lines = self.inner.read_lines(timeout_s)
        t = self.inner.sim_ms / 1000.0
        return [l for l in lines if not (l.startswith("$CGT") and self.start_s <= t < self.end_s)]

    def now_utc(self):
        return self.inner.now_utc()

    def close(self):
        self.inner.close()


def test_live_missing_telemetry_records(sim):
    t = DropTelemetry(sim(), 3.5, 6.5)
    s = NanoSession(t)
    out = []
    col = Collector(s, FusionContext(run_id="gap"), out.append)
    s.command("SYNC")
    s.pump(10)
    col.close()
    status = [r["record_status"] for r in out]
    assert status.count("MISSING_TELEMETRY") >= 3
    miss = [r for r in out if r["record_status"] == "MISSING_TELEMETRY" and r["t_sim_s"] > 2]
    assert miss and all(r["cell_voltage_v"] == [None] * 4 for r in miss)
    ts = [r["t_sim_s"] for r in out]
    assert ts == sorted(ts) and all(0.9 <= b - a <= 1.1 for a, b in zip(ts, ts[1:]))
