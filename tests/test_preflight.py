# Boot check ("test mode"): Pi-side checks before collection, against the firmware host simulator
# Author: Yongho Ahn <nanhosoft@gmail.com>
import datetime as dt
import json
from pathlib import Path

from cg_collector import cli
from cg_collector.current import CurrentReading, CurrentSource
from cg_collector.preflight import (FAIL, PASS, SKIP, WARN, PreflightOptions, run_preflight, tap_mv_from_raw,
                                    write_report)
from cg_collector.protocol import CalibrationInfo
from cg_collector.session import NanoSession
from cg_collector.transport import SimTransport

UTC = dt.timezone.utc
NOW = dt.datetime(2026, 10, 19, 9, 0, tzinfo=UTC)


def check(sim_path, tmp_path, gates="unlocked", reset="por", controls=(), **kw):
    t = SimTransport(sim_path, gates=gates, reset=reset)
    for line in controls:  # before the first sample: present at power-on
        t.control(line)
    try:
        opts = PreflightOptions(simulated=True, **kw.pop("opts", {}))
        return run_preflight(NanoSession(t), tmp_path, opts, system_now=lambda: NOW, ntp=lambda: True,
                             sleep=lambda _s: None, **kw)
    finally:
        t.close()


def test_healthy_simulator_passes_with_known_warnings(sim_path, tmp_path):
    r = check(sim_path, tmp_path)
    st = {c.name: c.status for c in r.checks}
    assert st["link"] == st["firmware"] == st["nano_status"] == st["telemetry"] == PASS
    assert st["clock"] == st["storage"] == PASS and st["ina219"] == SKIP
    # what the Nano can honestly not confirm: saturated NTC (cold or open) and nominal calibration
    nst = r.check("nano_self_test")
    assert nst.status == WARN and nst.data["fail"] == [] and nst.data["warn"] == ["NTC_SATURATED", "CAL_NOMINAL"]
    adc = r.check("adc_path")
    assert adc.data["max_diff_mv"] == 0 and adc.data["taps_from_raw_mv"] == adc.data["taps_firmware_mv"]
    assert r.result == WARN
    rec = r.to_record()
    assert rec["schema_version"] == "selftest_report_v1" and rec["firmware"]["version"] == "0.2.0"
    assert any(x["dir"] == "P>N" and ",SELFTEST*" in x["frame"] for x in rec["transcript"])


def test_power_on_sensor_fault_fails_and_locks(sim_path, tmp_path):
    r = check(sim_path, tmp_path, controls=["NTCRAW 2 0"])  # NTC 2 shorted at power-on
    nst = r.check("nano_self_test")
    assert nst.status == FAIL and nst.data["fail"] == ["NTC_FAULT"]
    assert r.check("measurement").status == FAIL
    assert "SELF_TEST_FAILED" in r.check("nano_status").data["locks"]
    assert r.result == FAIL


def test_tripped_nano_after_watchdog_reset(sim_path, tmp_path):
    r = check(sim_path, tmp_path, reset="wdt")
    assert r.check("nano_status").status == FAIL and r.check("nano_status").data["trip"] == "WATCHDOG_RESET"
    assert r.check("nano_self_test").data["fail"] == ["RESET_CAUSE"]  # reported by the boot self test
    assert r.result == FAIL


def test_wrong_harness_map_or_real_port_expectation_fails(sim_path, tmp_path):
    r = check(sim_path, tmp_path, opts={"expect_map": "PCB2-OTHER"})
    assert r.check("firmware").status == FAIL and "harness map" in r.check("firmware").detail
    t = SimTransport(sim_path)
    try:  # simulator firmware where a real Nano is expected
        r2 = run_preflight(NanoSession(t), tmp_path, PreflightOptions(simulated=False), ntp=lambda: True)
    finally:
        t.close()
    assert "host-simulator firmware" in r2.check("firmware").detail


class FakeIna(CurrentSource):
    def __init__(self, values, quality="VALID", config=0x399F):
        self.values, self.quality, self.config = list(values), quality, config

    def probe(self):
        return {"address": "0x40", "config_register": self.config, "expected": 0x399F}

    def read(self):
        v = self.values.pop(0) if self.values else 0.0
        return CurrentReading(v if self.quality == "VALID" else None, self.quality, NOW)


def test_ina219_checks(sim_path, tmp_path):
    ok = check(sim_path, tmp_path, current_source=FakeIna([0.001, -0.001] * 5)).check("ina219")
    assert ok.status == PASS and ok.data["reads"] == 10
    off = check(sim_path, tmp_path, current_source=FakeIna([0.05] * 10)).check("ina219")
    assert off.status == WARN and "shunt" in off.detail      # current with every output off
    bad = check(sim_path, tmp_path, current_source=FakeIna([], quality="I2C_ERROR")).check("ina219")
    assert bad.status == FAIL
    wrong = check(sim_path, tmp_path, current_source=FakeIna([0.0], config=0x0000)).check("ina219")
    assert wrong.status == FAIL and "config register" in wrong.detail
    missing = check(sim_path, tmp_path, current_error=OSError("[Errno 2] /dev/i2c-1")).check("ina219")
    assert missing.status == FAIL and "i2cdetect" in missing.detail


def test_clock_and_storage(sim_path, tmp_path):
    t = SimTransport(sim_path)
    try:
        blocker = tmp_path / "file"
        blocker.write_text("x")
        r = run_preflight(NanoSession(t), blocker / "data", PreflightOptions(simulated=True),
                          system_now=lambda: dt.datetime(1970, 1, 1, tzinfo=UTC), ntp=lambda: False)
    finally:
        t.close()
    assert r.check("clock").status == FAIL and "1970" in r.check("clock").detail
    assert r.check("storage").status == FAIL
    spool = tmp_path / "d" / "spool"
    spool.mkdir(parents=True)
    (spool / "raw-x.jsonl.part").write_text("{}\n")
    r2 = check(sim_path, tmp_path / "d")
    assert r2.check("storage").status == WARN and "unfinished" in r2.check("storage").detail


def test_report_goes_to_the_outbox(sim_path, tmp_path):
    r = check(sim_path, tmp_path)
    path = write_report(r, tmp_path)
    assert path.parent.name == "outbox" and path.name.startswith("selftest-") and path.suffix == ".jsonl"
    (rec,) = [json.loads(l) for l in path.read_text().splitlines()]
    assert rec["result"] == "WARN" and len(rec["checks"]) == 10


def test_tap_conversion_matches_firmware():
    cal = CalibrationInfo(0, "NOMINAL", "x", 2500, 5000, [1000000] * 4, [0] * 4, 3950, 10000)
    assert [tap_mv_from_raw(r, cal, i) for i, r in enumerate((235, 469, 704, 938))] == [1202, 2399, 3601, 4798]
    assert tap_mv_from_raw(1023, cal, 0) == 5233  # saturation ~5.23 V


# ---- CLI: test mode and boot mode -------------------------------------------------------------

def test_selftest_command(sim_path, tmp_path, capsys):
    assert cli.main(["--sim", sim_path, "--sim-gates", "unlocked", "--data-dir", str(tmp_path), "selftest",
                     "--json"]) == 0
    cap = capsys.readouterr()
    assert json.loads(cap.out)["result"] == "WARN"
    assert "BOOT CHECK: WARN" in cap.err and "[WARN] nano_self_test" in cap.err
    assert len(list((tmp_path / "outbox").glob("selftest-*.jsonl"))) == 1
    assert cli.main(["--sim", sim_path, "--data-dir", str(tmp_path), "selftest", "--fail-on-warn"]) == 1


def test_boot_check_blocks_start_on_fail(sim_path, tmp_path, capsys):
    args = ["--sim", sim_path, "--data-dir", str(tmp_path), "--expect-map", "PCB2-OTHER"]
    assert cli.main(args + ["monitor", "--seconds", "2"]) == 1
    assert "not starting" in capsys.readouterr().err
    outbox = sorted(p.name.split("-")[0] for p in (tmp_path / "outbox").glob("*.jsonl"))
    assert outbox == ["selftest"]  # nothing recorded, only the report
    assert cli.main(args + ["--boot-check", "warn", "monitor", "--seconds", "2"]) == 0
    assert any((tmp_path / "outbox").glob("samples-*.jsonl"))
    before = len(list((tmp_path / "outbox").glob("selftest-*.jsonl")))
    assert cli.main(args + ["--boot-check", "off", "monitor", "--seconds", "2"]) == 0
    assert len(list((tmp_path / "outbox").glob("selftest-*.jsonl"))) == before
