# Normalized records, raw join, missing slots, time base, current alignment, fault tracking
# Author: Yongho Ahn <nanhosoft@gmail.com>
import datetime as dt

import jsonschema

from cg_collector import protocol as p
from cg_collector.current import CurrentReading, CurrentSampler, CurrentSource, NoCurrentSensor
from cg_collector.fusion import FusionContext
from cg_collector.replay import replay
from cg_collector.session import FaultTracker

UTC = dt.timezone.utc
T0 = dt.datetime(2026, 10, 14, 12, 0, 0, tzinfo=UTC)


def fr(payload: str) -> str:
    return "$" + payload + "*" + f"{p.crc16_ccitt_false(payload.encode()):04X}"


def at(ms: float) -> dt.datetime:
    return T0 + dt.timedelta(milliseconds=ms)


def cgt(seq, millis, cells=(1200, 1201, 1199, 1200), esr=0, tq=85, temps=("-32768",) * 4, charge=0):
    t = ",".join(temps)
    return fr(f"CGT,{seq},{millis},{','.join(map(str, cells))},{t},{charge},0,1,{esr},0,0,0,0,0,0,0,{tq}")


def cgs(seq, millis, state="ARMED", relay=0):
    return fr(f"CGS,{seq},{millis},{state},0,{relay},0,300,5,10,0,0,0,1,0,0,450")


def cgd(seq):
    return fr(f"CGD,{seq},235,469,704,938,1023,1023,1023,1023")


def telemetry_lines(seqs, start_ms=1000, raw_delay_ms=15, with_raw=True):
    """Nano telemetry every second; receive time = Nano time + 8 ms."""
    lines = []
    for k in seqs:
        t = start_ms + 1000 * k
        lines.append((at(t + 8), "N>P", cgt(k, t)))
        lines.append((at(t + 12), "N>P", cgs(k, t)))
        if with_raw:
            lines.append((at(t + 12 + raw_delay_ms), "N>P", cgd(100 + k)))
    return lines


def run(lines, end_ms=None, **ctx):
    out, events = [], []
    replay(lines, FusionContext(run_id="r", **ctx), out.append, events.append, run_start=T0,
           end_utc=at(end_ms) if end_ms else None)
    return out, events


def test_one_record_per_slot_with_raw_and_elapsed_time(normalized_schema):
    out, _ = run(telemetry_lines(range(5)))
    assert [r["record_status"] for r in out] == ["MISSING_TELEMETRY"] + ["OK"] * 5  # slot 0 s had no telemetry
    ok = out[1:]
    # t_sim_s = run elapsed time at the Nano sample instant (not Nano uptime)
    assert [r["t_sim_s"] for r in ok] == [1.0, 2.0, 3.0, 4.0, 5.0]
    assert [r["nano_millis"] for r in ok] == [1000, 2000, 3000, 4000, 5000]
    assert ok[0]["nano_adc_tap_raw"] == [235, 469, 704, 938] and ok[0]["nano_adc_ntc_raw"] == [1023] * 4
    assert ok[0]["cumulative_tap_voltage_v"] == [1.2, 2.401, 3.6, 4.8]
    assert ok[0]["command_state_source"] == "CGT+CGS" and ok[0]["controller_state"] == "ARMED"
    assert ok[0]["source"] == "measured_normal" and ok[0]["scenario_domain"] == "measured_hardware"
    for r in out:
        jsonschema.validate(r, normalized_schema)


def test_lost_telemetry_becomes_missing_records(normalized_schema):
    lines = [l for l in telemetry_lines(range(7)) if not ("CGT,2," in l[2] or "CGT,3," in l[2])]
    out, _ = run(lines)
    status = [(r["t_sim_s"], r["record_status"]) for r in out[1:]]
    assert status == [(1.0, "OK"), (2.0, "OK"), (3.0, "MISSING_TELEMETRY"), (4.0, "MISSING_TELEMETRY"),
                      (5.0, "OK"), (6.0, "OK"), (7.0, "OK")]
    miss = out[3]
    assert miss["cell_voltage_v"] == [None] * 4 and miss["temperature_quality"] == ["MISSING"] * 4
    assert miss["voltage_quality"] == "MISSING" and miss["command_state_source"] == "LAST_KNOWN"
    assert miss["nano_adc_tap_raw"] == [None] * 4 and miss["pack_current_a"] is None
    for r in out:
        jsonschema.validate(r, normalized_schema)


def test_trailing_missing_slots_at_close():
    out, _ = run(telemetry_lines(range(3)), end_ms=6500)
    assert [r["record_status"] for r in out[-3:]] == ["MISSING_TELEMETRY"] * 3
    assert out[-1]["t_sim_s"] == 6.0


def test_late_raw_is_not_joined():
    out, _ = run(telemetry_lines(range(2), raw_delay_ms=300))  # CGD 312 ms after its CGT: other sample
    assert all(r["nano_adc_tap_raw"] == [None] * 4 for r in out)


def test_duplicate_and_corrupt_frames_are_counted():
    lines = telemetry_lines(range(3))
    lines.insert(1, (at(1009), "N>P", cgt(0, 1000)))           # duplicated CGT
    lines.insert(2, (at(1010), "N>P", "$CGT,garbage*0000"))    # CRC error
    out, _ = run(lines)
    # counted once, on the first record written after the errors (here the leading MISSING slot,
    # same attribution rule as the generator)
    assert sum(r["transport"]["duplicates_since_last"] for r in out) == 1
    assert sum(r["transport"]["crc_errors_since_last"] for r in out) == 1
    assert [r["record_status"] for r in out[1:]] == ["OK"] * 3


def test_reboot_starts_new_epoch():
    lines = telemetry_lines(range(3))
    lines.append((at(3900), "N>P", fr("CGE,0,0,BOOT,0,8,0,WATCHDOG")))
    lines += [(t + dt.timedelta(seconds=3), d, f) for t, d, f in telemetry_lines(range(2))]
    out, events = run(lines)
    oks = [r for r in out if r["record_status"] == "OK"]
    assert [r["nano_boot_epoch"] for r in oks] == [0, 0, 0, 1, 1]
    assert oks[3]["t_sim_s"] > oks[2]["t_sim_s"]
    assert any(e["event_type"] == "BOOT" for e in events)


class FakeClock:
    def __init__(self, t):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, ms):
        self.t += dt.timedelta(milliseconds=ms)


class FakeIna(CurrentSource):
    """Like the real driver: stamps each reading with its own read time."""

    def __init__(self, clock, values):
        self.clock, self.values = clock, list(values)

    def read(self):
        return CurrentReading(self.values.pop(0), "VALID", self.clock())


def test_sampler_uses_latest_sample_before_time():
    clock = FakeClock(T0)
    s = CurrentSampler(FakeIna(clock, [-0.40, -0.41, -0.42]), clock=clock, max_age_s=0.5)
    s.sample_once()
    clock.advance(100)
    s.sample_once()
    clock.advance(100)
    s.sample_once()
    r = s.reading_at(T0 + dt.timedelta(milliseconds=150))  # the T0+200 sample is in the future
    assert r.current_a == -0.41 and r.quality == "VALID"
    assert s.reading_at(T0 - dt.timedelta(milliseconds=1)).quality == "MISSING"
    assert s.reading_at(T0 + dt.timedelta(seconds=3)).quality == "MISSING"


def test_unavailable_source_through_sampler():
    clock = FakeClock(T0)
    s = CurrentSampler(NoCurrentSensor(), clock=clock)
    s.sample_once()
    assert s.reading_at(T0).quality == "INA219_UNAVAILABLE"


def test_fault_tracker_semantics():
    ft = FaultTracker()
    ok = p.Ack(5, True, "NONE")
    ft.on_ack("ESR", [2, 1], ok, T0, at(10))
    ft.on_ack("LEAK", [1, 1], ok, T0, at(10))
    ft.on_ack("CHARGE", [1], p.Ack(6, True, "PENDING"), at(100), at(110))
    req = ft.requested_struct()
    assert req["type"] == "ESR" and req["count"] == 2
    assert [f["type"] for f in req["faults"]] == ["ESR", "LEAK"]
    assert ft.power_requested() == {"charge": True, "discharge": False}
    ft.on_event(p.Event(1, 600000, "ESR_STOP", 2, 1, 0, "TIMEOUT"), at(1000))
    assert ("ESR", 2) not in ft.requested and ft.last_ended["reason"] == "TIMEOUT"
    ft.on_event(p.Event(2, 600100, "CHARGE_STOP", 0, 1, 0, "PACK_LIMIT"), at(2000))
    assert ft.power_requested()["charge"] is False
    # Refused: requested from sending until the refusal arrived.
    ft.on_ack("HEAT", [1, 40], p.Ack(7, False, "HARDWARE_LOCKED"), at(3000), at(3015))
    assert ("HEAT", 1) not in ft.requested
    assert ft.last_ended["reason"] == "REJECTED_HARDWARE_LOCKED"
    assert any(f["type"] == "HEAT" for f in ft.state_at(at(3005))[0]["faults"])
    # The state is queried as of a past time (gap fill).
    assert ft.state_at(at(50))[1] == {"charge": False, "discharge": False}
    assert ft.state_at(at(500))[1] == {"charge": True, "discharge": False}
    # A Nano reset ends everything.
    ft.on_event(p.Event(0, 0, "BOOT", 0, 8, 0, "WATCHDOG"), at(4000))
    assert ft.requested == {} and ft.last_ended["reason"] == "NANO_RESET"
