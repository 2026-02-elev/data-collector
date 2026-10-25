# Replies that come after their ACK in a separate UART read (rig bug: "adc_path: no CGK calibration frame")
# Author: Yongho Ahn <nanhosoft@gmail.com>
"""On the Pi the Nano's ACK and the CGK / CGD / CGV that follows it often arrive in different
serial reads. The host simulators deliver them in one batch, which hid the bug; these transports
reproduce the real timing: one line per read, late replies, lost replies, malformed replies."""
import json

import pytest

from cg_collector import cli
from cg_collector.preflight import FAIL, PASS, WARN, PreflightOptions, run_preflight
from cg_collector.protocol import CalibrationInfo, RawAdc, crc16_ccitt_false
from cg_collector.session import NanoSession
from cg_collector.transport import SimTransport


class SplitReads(SimTransport):
    """Every read_lines() returns at most one line: the ACK and its CGK are never in the same read."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.held = []
        self.reads = []

    def read_lines(self, timeout_s):
        if not self.held:
            self.held = super().read_lines(timeout_s)
        out = [self.held.pop(0)] if self.held else []
        self.reads.append(out)
        return out


class LateReplies(SimTransport):
    """Frames starting with ``prefixes`` arrive ``delay_ms`` (simulated) later; None = never."""

    def __init__(self, sim, prefixes=("$CGK,",), delay_ms=300, rewrite=None, **k):
        super().__init__(sim, **k)
        self.prefixes, self.delay_ms, self.rewrite = tuple(prefixes), delay_ms, rewrite
        self.late = []

    def read_lines(self, timeout_s):
        out = []
        for line in super().read_lines(timeout_s):
            if line.startswith(self.prefixes):
                if self.rewrite:
                    out.append(self.rewrite(line))
                elif self.delay_ms is not None:
                    self.late.append((self.sim_ms + self.delay_ms, line))
            else:
                out.append(line)
        due = [l for t, l in self.late if t <= self.sim_ms]
        self.late = [(t, l) for t, l in self.late if t > self.sim_ms]
        return out + due


def frame(payload):
    return f"${payload}*{crc16_ccitt_false(payload.encode()):04X}"


def drop_last_field(line):  # CRC-valid CGK with 15 instead of 16 fields
    return frame(",".join(line[1:line.index("*")].split(",")[:-1]))


def boot_check(transport, tmp_path, **opts):
    try:
        return run_preflight(NanoSession(transport), tmp_path, PreflightOptions(simulated=True, **opts),
                             ntp=lambda: True, sleep=lambda _s: None)
    finally:
        transport.close()


@pytest.fixture(params=["0.1.6", "0.2.0"])
def any_sim(request, sim016_path, sim_path):
    return sim016_path if request.param == "0.1.6" else sim_path


def test_split_reads_reproduce_the_rig_and_the_boot_check_passes(any_sim, tmp_path):
    t = SplitReads(any_sim)
    r = boot_check(t, tmp_path)
    adc = r.check("adc_path")   # before the fix: FAIL "no CGK calibration frame"
    assert adc.status in (PASS, WARN) and adc.data["max_diff_mv"] == 0, adc.detail
    # the ACK and its CGK really were delivered by different read_lines() calls
    flat = [l for read in t.reads for l in read]
    i = next(k for k, l in enumerate(flat) if l.startswith("$CGK,"))
    assert flat[i - 1].startswith("$CGA,") and all(len(read) <= 1 for read in t.reads)
    assert r.check("firmware").status in (PASS, WARN)       # CGV after the SYNC ACK
    assert r.check("nano_status").status in (PASS, WARN)    # CGS after the STATUS ACK
    assert r.result != FAIL and r.actuation["allowed"]


def test_late_cgk_and_cgd_are_waited_for(sim016_path, tmp_path):
    r = boot_check(LateReplies(sim016_path, prefixes=("$CGK,", "$CGD,"), delay_ms=300), tmp_path)
    assert r.check("adc_path").status in (PASS, WARN), r.check("adc_path").detail


def test_missing_cgk_fails_and_an_old_one_is_not_reused(sim016_path, tmp_path):
    t = LateReplies(sim016_path, delay_ms=None)  # CGK never arrives
    s = NanoSession(t)
    s.calibration = CalibrationInfo(1, "MEASURED", "OLD_BOARD", 2500, 5000, [1000000] * 4, [0] * 4, 3950, 10000)
    try:
        r = run_preflight(s, tmp_path, PreflightOptions(simulated=True), ntp=lambda: True)
    finally:
        t.close()
    adc = r.check("adc_path")
    assert adc.status == FAIL and adc.detail.startswith("no CGK frame with seq") and "after the CAL ACK" in adc.detail
    assert r.result == FAIL and not r.actuation["allowed"]   # still a real failure (no bypass)


def test_malformed_cgk_fails(sim016_path, tmp_path):
    r = boot_check(LateReplies(sim016_path, rewrite=drop_last_field), tmp_path)
    adc = r.check("adc_path")
    assert adc.status == FAIL and adc.detail.startswith("malformed CGK reply to CAL")


def test_missing_cgd_fails(sim016_path, tmp_path):
    r = boot_check(LateReplies(sim016_path, prefixes=("$CGD,",), delay_ms=None), tmp_path)
    assert r.check("adc_path").status == FAIL and "no CGD frame with seq" in r.check("adc_path").detail


def test_request_matches_the_reply_seq_and_ignores_older_replies(sim016_path):
    t = LateReplies(sim016_path, delay_ms=400)
    s = NanoSession(t)
    try:
        assert s.command("SYNC").ok
        first = s.request("CAL", reply=CalibrationInfo, timeout_s=0.1)
        assert first.ack.ok and first.message is None                 # its CGK is still on the way
        second = s.request("CAL", reply=CalibrationInfo, timeout_s=2.0)
        assert second.message is not None and second.message.seq == second.ack.seq != first.ack.seq
        raw = s.request("RAW", reply=RawAdc)
        assert raw.message.seq == raw.ack.seq
    finally:
        t.close()


def test_pack_near_zero_warning_is_kept(sim016_path, tmp_path):
    t = SplitReads(sim016_path)
    t.control("CELLS 0 0 0 0")  # discharged pack / harness not connected (the rig output had this warning)
    r = boot_check(t, tmp_path)
    m = r.check("measurement")
    assert m.status == WARN and "pack ~0 V" in m.detail


def test_monitor_and_query_with_split_reads(sim016_path, tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(cli, "_transport", lambda _a: SplitReads(sim016_path))
    assert cli.main(["--sim", sim016_path, "--data-dir", str(tmp_path), "monitor", "--seconds", "4"]) == 0
    samples = [json.loads(l) for f in (tmp_path / "outbox").glob("samples-*.jsonl") for l in f.read_text().splitlines()]
    oks = [r for r in samples if r["record_status"] == "OK"]
    assert oks and all(r["calibration_id"] == "NOMINAL_UNCALIBRATED" for r in oks)
    assert sum(r["nano_adc_tap_raw"][0] is not None for r in oks) >= len(oks) - 1   # CGD joined to its CGT
    capsys.readouterr()
    assert cli.main(["--sim", sim016_path, "query", "cal"]) == 0
    out = capsys.readouterr().out.splitlines()
    assert out[0] == "OK NONE" and out[1].startswith("$CGK,") and ",4096," in out[1]
