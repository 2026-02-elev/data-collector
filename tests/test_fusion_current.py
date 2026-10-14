# Fusion, INA219 time alignment, fault-state tracking
# Author: Yongho Ahn <nanhosoft@gmail.com>
import datetime as dt

import jsonschema

from cg_collector import protocol as p
from cg_collector.current import CurrentReading, CurrentSampler, CurrentSource, NoCurrentSensor
from cg_collector.fusion import FusionContext, fuse_sample
from cg_collector.session import FaultTracker

UTC = dt.timezone.utc
T0 = dt.datetime(2026, 10, 14, 12, 0, 0, tzinfo=UTC)
CGT = "CGT,7,7000,1201,1199,-32768,1203,-32768,4012,-32768,-32768,0,1,1,2,0,0,0,0,0,0,3,81"
CGS = "CGS,7,7001,ACTIVE,0,1,0,300,12,20,0,0,0,1,0,850,451"


class FakeClock:
    def __init__(self, t):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, ms):
        self.t += dt.timedelta(milliseconds=ms)


class FakeIna(CurrentSource):
    """Behaves like the real driver: stamps the reading with *its own* read time."""

    def __init__(self, clock, values):
        self.clock, self.values = clock, list(values)

    def read(self):
        return CurrentReading(self.values.pop(0), "VALID", self.clock())


def fuse(reading, version="CGV,0.1.0,1.0,PCB2-20260918-R2,0", tracker=None):
    return fuse_sample(p.parse_message(CGT), T0, p.parse_message(CGS), p.parse_message(version), reading,
                       tracker or FaultTracker(), FusionContext(run_id="r1"))


def test_missing_current_is_null_not_zero(normalized_schema):
    rec = fuse(CurrentReading(None, "INA219_UNAVAILABLE", None))
    assert rec["pack_current_a"] is None and rec["current_quality"] == "INA219_UNAVAILABLE"
    jsonschema.validate(rec, normalized_schema)


def test_sampler_uses_latest_sample_before_receipt():
    clock = FakeClock(T0)
    s = CurrentSampler(FakeIna(clock, [-0.40, -0.41, -0.42, -0.43]), clock=clock, max_age_s=0.5)
    s.sample_once()            # T0
    clock.advance(100)
    s.sample_once()            # T0+100
    clock.advance(100)
    s.sample_once()            # T0+200
    # Telemetry received at T0+150: the T0+200 sample is in the future -> not used,
    # the T0+100 sample is used (previously this whole reading would be dropped).
    r = s.reading_at(T0 + dt.timedelta(milliseconds=150))
    assert r.current_a == -0.41 and r.quality == "VALID"
    # Before the first sample: nothing to use.
    assert s.reading_at(T0 - dt.timedelta(milliseconds=1)).quality == "MISSING"
    # Too old: sampler stopped.
    assert s.reading_at(T0 + dt.timedelta(seconds=3)).quality == "MISSING"


def test_sampler_with_real_driver_timing_never_drops_good_data():
    """Sensor read right AFTER each telemetry frame (the reported bug) still pairs."""
    clock = FakeClock(T0)
    s = CurrentSampler(FakeIna(clock, [0.2] * 50), clock=clock)
    paired = 0
    for i in range(20):
        clock.advance(100)
        s.sample_once()
        if i % 10 == 5:
            received = clock()
            clock.advance(3)  # the next read happens just after the frame
            s.sample_once()
            if s.reading_at(received).quality == "VALID":
                paired += 1
    assert paired == 2


def test_unavailable_source_through_sampler():
    clock = FakeClock(T0)
    s = CurrentSampler(NoCurrentSensor(), clock=clock)
    s.sample_once()
    assert s.reading_at(T0).quality == "INA219_UNAVAILABLE"


def test_measured_and_simulated_labelling(normalized_schema):
    rec = fuse(CurrentReading(-0.42, "VALID", T0 - dt.timedelta(milliseconds=50)))
    assert rec["source"] == "measured_fault_injection"
    assert rec["pack_current_a"] == -0.42
    assert rec["physical_fault_verified"] is False and rec["fault_physical_observed"] is None
    assert rec["cumulative_tap_voltage_v"] == [1.201, 2.4, None, None]
    jsonschema.validate(rec, normalized_schema)
    sim = fuse(CurrentReading(None, "INA219_UNAVAILABLE", None), version="CGV,0.1.0,1.0,PCB2-20260918-R2,513")
    assert sim["source"] == "synthetic" and sim["physical_harness_verified"] is False
    jsonschema.validate(sim, normalized_schema)


def test_fault_tracker_follows_nano_side_stops():
    ft = FaultTracker()
    ok = p.Ack(5, True, "NONE")
    ft.on_ack("ESR", [2, 1], ok, T0)
    ft.on_ack("LEAK", [1, 1], ok, T0)
    ft.on_ack("CHARGE", [1], p.Ack(6, True, "PENDING"), T0)
    req = ft.requested_struct()
    assert {f["type"] for f in req["all"]} == {"ESR", "LEAK"}
    assert ft.power_requested() == {"charge": True, "discharge": False}
    # Nano ends the ESR fault on its own (max duration): the request is no longer in effect.
    ft.on_event(p.Event(1, 600000, "ESR_STOP", 2, 1, 0, "TIMEOUT"), T0)
    assert ("ESR", 2) not in ft.requested
    assert ft.last_ended["reason"] == "TIMEOUT" and ft.last_ended["cell_id"] == 2
    # Charge ended by the pack limit.
    ft.on_event(p.Event(2, 600100, "CHARGE_STOP", 0, 1, 0, "PACK_LIMIT"), T0)
    assert ft.power_requested()["charge"] is False
    # A refused request is never recorded.
    ft.on_ack("HEAT", [1, 40], p.Ack(7, False, "HARDWARE_LOCKED"), T0)
    assert ("HEAT", 1) not in ft.requested
    # A trip ends everything.
    ft.on_event(p.Event(3, 600200, "TRIP", 0, 0, 0, "HEARTBEAT_TIMEOUT"), T0)
    assert ft.requested == {} and ft.last_ended["reason"] == "TRIP_HEARTBEAT_TIMEOUT"
    assert ft.requested_struct() is None
