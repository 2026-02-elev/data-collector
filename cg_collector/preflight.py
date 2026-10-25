# CellGuard data collector - boot check ("test mode") before collection starts
# Author: Yongho Ahn <nanhosoft@gmail.com>
"""Checks everything the Pi can check before it starts recording or driving the rig:

==================  ===========================================================
``link``            UART answers (SYNC, PING round trip)
``firmware``        CGV: protocol, harness map version, simulator / bench build
``nano_self_test``  the Nano's own power-on self test (``SELFTEST`` -> CGE SELF_TEST)
``nano_status``     CGS: state, trip latch, reset cause, locks
``telemetry``       1 Hz CGT/CGS arrive, sequence continuous, no frame errors
``measurement``     voltage / temperature quality over the window, readings at rest stable
``adc_path``        RAW + CAL: the Pi recomputes the taps from the raw ADC counts
``ina219``          I2C device answers, reads VALID, ~0 A with every output off
``clock``           system time plausible and NTP synchronised (timestamps)
``storage``         data directory writable, enough free space, crash leftovers
==================  ===========================================================

Each check is PASS / WARN / FAIL / SKIP / UNSUPPORTED; the result is the worst of them
(UNSUPPORTED - the connected firmware has no such check, e.g. SELFTEST on
0.1.6-LOAD-VERIFY - counts like WARN: never reported as PASS). The
report (``selftest_report_v1``) is written to the outbox like any data file, so
the server sees why a device did or did not start.

Nothing here switches an output on. The Nano cannot read its outputs back, so
relays / MOSFETs / heaters are NOT verified (firmware docs/self_test.md).
"""
from __future__ import annotations

import dataclasses
import datetime as dt
import json
import math
import os
import shutil
import statistics
import subprocess
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional

from . import __version__
from .current import CurrentSource
from .protocol import (CalibrationInfo, Event, RawAdc, SELF_TEST_FAIL_BITS, SELF_TEST_WARN_BITS, Status, Telemetry,
                       TRIP_NAMES, Version, bit_names)
from .session import CommandTimeout, NanoSession, Received

PASS, WARN, FAIL, SKIP, UNSUPPORTED = "PASS", "WARN", "FAIL", "SKIP", "UNSUPPORTED"
_RANK = {PASS: 0, SKIP: 0, WARN: 1, UNSUPPORTED: 1, FAIL: 2}
# Pi-side checks that must hold when the Nano itself has no self test (0.1.x)
COMPENSATING = ("nano_status", "telemetry", "measurement", "adc_path")
UTC = dt.timezone.utc
TAP_DIVIDER = (47000 + 43000) / 43000
VQ_FAIL = ("AREF_INVALID", "ADC_SATURATED", "TAP_INVERSION", "STALE")


@dataclasses.dataclass
class Check:
    name: str
    status: str
    detail: str
    data: Dict[str, object] = dataclasses.field(default_factory=dict)


@dataclasses.dataclass
class PreflightOptions:
    simulated: bool = False               # --sim: a HOST_SIM_BUILD firmware is expected
    expect_protocol: str = "1.0"
    expect_map: str = "PCB2-20260918-R2"
    boot_wait_s: float = 3.0              # a USB-serial Nano resets on open (bootloader ~2 s)
    watch_s: float = 3.0                  # telemetry window
    self_test_timeout_s: float = 3.0
    reply_timeout_s: float = 1.0          # CGK / CGD / CGS after their command's ACK (separate UART reads)
    version_timeout_s: float = 2.0        # CGV after the SYNC ACK
    ping_count: int = 5
    max_rtt_ms: float = 100.0
    rest_spread_mv: int = 50              # cell reading spread over the window with every output off
    adc_path_tol_mv: int = 10
    ina219_samples: int = 10
    ina219_period_s: float = 0.1
    ina219_zero_tol_a: float = 0.010
    min_free_mb: int = 100
    warn_free_mb: int = 1000
    min_year: int = 2025


@dataclasses.dataclass
class Report:
    checks: List[Check]
    started_utc: dt.datetime
    finished_utc: dt.datetime
    meta: Dict[str, object]
    transcript: List[Dict[str, str]]

    @property
    def result(self) -> str:
        worst = max((_RANK[c.status] for c in self.checks), default=0)
        return {0: PASS, 1: WARN, 2: FAIL}[worst]

    def check(self, name: str) -> Check:
        return next(c for c in self.checks if c.name == name)

    @property
    def actuation(self) -> Dict[str, object]:
        """Conservative go / no-go for energising outputs (cycle / run). Read-only monitoring
        only needs ``result != FAIL``."""
        reasons = []
        if self.result == FAIL:
            reasons.append("boot check FAIL")
        fam = (self.meta.get("capabilities") or {}).get("family", "unknown")
        if fam == "unknown":
            reasons.append("unknown firmware: no actuation")
        names = {c.name: c for c in self.checks}
        st = names.get("nano_self_test")
        if st is not None and st.status == UNSUPPORTED:
            bad = [n for n in COMPENSATING if n not in names or names[n].status not in (PASS, WARN)]
            if bad:
                reasons.append("Nano has no self test and these Pi checks did not pass: " + ", ".join(bad))
        elif st is None or st.status not in (PASS, WARN):
            reasons.append("Nano self test did not pass")
        return {"allowed": not reasons, "reasons": reasons}

    def to_record(self) -> Dict[str, object]:
        return {"schema_version": "selftest_report_v1", "result": self.result, "actuation": self.actuation,
                "started_utc": _iso(self.started_utc), "finished_utc": _iso(self.finished_utc),
                "collector_version": __version__, **self.meta,
                "checks": [dataclasses.asdict(c) for c in self.checks], "transcript": self.transcript}


def _iso(t: dt.datetime) -> str:
    return t.astimezone(UTC).isoformat(timespec="milliseconds").replace("+00:00", "Z")


class _Watch:
    """Collects what arrives during the boot check (and a bounded raw transcript)."""

    def __init__(self, session: NanoSession, limit: int = 600):
        self.s = session
        self.frames: List[Received] = []
        self.transcript: List[Dict[str, str]] = []
        self.limit = limit
        self.active = True
        self.cgv_after_sync: Optional[Version] = None   # the CGV sent in reply to OUR SYNC (not a boot CGV)
        self.cgv_malformed: List[str] = []
        session.subscribe(self.on_rx)
        session.on_transmit(self.on_tx)

    def on_rx(self, rec: Received) -> None:
        self.frames.append(rec)
        self._log({"utc": _iso(rec.utc), "dir": "N>P", "frame": rec.raw, **({"error": rec.error} if rec.error else {})})

    def on_tx(self, utc: dt.datetime, frame: str) -> None:
        if self.active:
            self._log({"utc": _iso(utc), "dir": "P>N", "frame": frame})

    def _log(self, entry: Dict[str, str]) -> None:
        if self.active and len(self.transcript) < self.limit:
            self.transcript.append(entry)

    def since(self, i: int, kind: type) -> list:
        return [r for r in self.frames[i:] if isinstance(r.message, kind)]

    def close(self) -> None:
        self.active = False
        self.s.unsubscribe(self.on_rx)


def _wait(session: NanoSession, done: Callable[[], bool], timeout_s: float) -> bool:
    end = session.t.now_utc() + dt.timedelta(seconds=timeout_s)
    while not done():
        if session.t.now_utc() >= end:
            return False
        session.poll(0.05)
    return True


def tap_mv_from_raw(raw: int, cal: CalibrationInfo, ch: int) -> int:
    """Same conversion as the firmware's tapMvFromRaw() (cg_measure.cpp)."""
    mv = raw * cal.aref_mv / 1023.0 * TAP_DIVIDER
    mv = mv * (cal.gain_ppm[ch] / 1e6) + cal.offset_mv[ch]
    return int(math.floor(mv + 0.5)) if mv >= 0 else -int(math.floor(-mv + 0.5))


# ---------------------------------------------------------------------------------------- checks

def _check_link(session: NanoSession, w: _Watch, o: PreflightOptions) -> Check:
    _wait(session, lambda: bool(w.frames), o.boot_wait_s)  # boot frames / telemetry, if any
    heard = len(w.frames)
    try:
        rep = session.request("SYNC", reply=Version, match_seq=False, timeout_s=o.version_timeout_s)
        ack = rep.ack
        w.cgv_after_sync, w.cgv_malformed = rep.message, rep.malformed
    except CommandTimeout:
        return Check("link", FAIL, "no reply to SYNC: check Nano power, TX/RX crossing, 115200 baud and that the "
                                   "serial login console is disabled", {"frames_before_sync": heard,
                                                                         "frame_errors": session.frame_errors})
    if not ack.ok:
        return Check("link", FAIL, f"SYNC refused: {ack.text}", {"frames_before_sync": heard})
    rtts = []
    for _ in range(o.ping_count):
        t0 = session.t.now_utc()
        try:
            session.command("PING")
        except CommandTimeout:
            return Check("link", FAIL, "PING lost after SYNC: unstable link", {"rtt_ms": rtts})
        rtts.append((session.t.now_utc() - t0).total_seconds() * 1000.0)
    data = {"frames_before_sync": heard, "rtt_ms_median": round(statistics.median(rtts), 1),
            "rtt_ms_max": round(max(rtts), 1), "frame_errors": session.frame_errors}
    if data["rtt_ms_median"] > o.max_rtt_ms:
        return Check("link", WARN, f"slow replies (median {data['rtt_ms_median']} ms)", data)
    return Check("link", PASS, f"SYNC OK, PING {data['rtt_ms_median']} ms", data)


def _check_firmware(session: NanoSession, w: _Watch, o: PreflightOptions) -> Check:
    v = w.cgv_after_sync  # fresh CGV in reply to our SYNC; a boot CGV seen earlier does not count
    if v is None:
        if w.cgv_malformed:
            return Check("firmware", FAIL, "malformed CGV after SYNC", {"frames": w.cgv_malformed[:3]})
        return Check("firmware", FAIL, f"no CGV version frame within {o.version_timeout_s} s after the SYNC ACK")
    caps = session.capabilities
    from .protocol import CGV_FLAG_BITS

    flags = [n for n in CGV_FLAG_BITS if v.flag(n)]
    data = {"firmware": v.firmware, "protocol": v.protocol, "map_version": v.map_version, "flags": flags,
            "family": caps.family, "self_test": caps.self_test, "staged_power": caps.staged_power}
    problems, notes = [], []
    if v.protocol != o.expect_protocol:
        problems.append(f"protocol {v.protocol} (expected {o.expect_protocol})")
    if v.map_version != o.expect_map:
        problems.append(f"harness map {v.map_version} (expected {o.expect_map})")
    if v.flag("HOST_SIM_BUILD") and not o.simulated:
        problems.append("host-simulator firmware on a real port")
    if o.simulated and not v.flag("HOST_SIM_BUILD"):
        problems.append("--sim given but the firmware is not a simulator build")
    if problems:
        return Check("firmware", FAIL, "; ".join(problems), data)
    if v.flag("BENCH_BUILD"):
        notes.append("bench build (BENCH single-output command enabled)")
    if not caps.known:
        notes.append(f"unknown firmware {v.firmware!r}: read-only commands only, no actuation")
    elif caps.staged_power:
        notes.append("commissioning build: RAMP/DRAIN only (CHARGE,1/DISCHARGE,1 refused), no SELFTEST")
    unlocked = [n for n in ("POWER_UNLOCKED", "ESR_UNLOCKED", "LEAK_UNLOCKED", "HEAT_UNLOCKED") if v.flag(n)]
    data["unlocked"] = unlocked
    text = f"{v.firmware} / protocol {v.protocol} / {v.map_version}" + (" (SIMULATED DEVICE)" if o.simulated else "")
    return Check("firmware", WARN if notes else PASS, "; ".join([text] + notes), data)


def _check_nano_self_test(session: NanoSession, w: _Watch, o: PreflightOptions) -> Check:
    caps = session.capabilities
    if not caps.self_test:
        # not sent: this firmware has no SELFTEST (sending it would only be refused); the Pi-side
        # checks below (status, telemetry, measurement, ADC path) have to stand in for it
        return Check("nano_self_test", UNSUPPORTED,
                     f"firmware {caps.firmware or 'unknown'} has no power-on self test / SELFTEST "
                     "(Pi-side checks only)", {"family": caps.family})
    mark = len(w.frames)
    try:
        ack = session.command("SELFTEST")
    except CommandTimeout:
        return Check("nano_self_test", FAIL, "no reply to SELFTEST")
    if not ack.ok:
        why = {"UNKNOWN_CMD": "firmware has no SELFTEST (update to >= 0.2.0)",
               "TRIPPED": "Nano is TRIPPED: find the cause, then RESET (see nano_status)"}.get(ack.text, ack.text)
        return Check("nano_self_test", FAIL, f"SELFTEST refused: {why}", {"ack": ack.text})

    def result() -> Optional[Event]:
        for r in w.since(mark, Event):
            if r.message.code == "SELF_TEST":
                return r.message
        return None

    if not _wait(session, lambda: result() is not None, o.self_test_timeout_s):
        return Check("nano_self_test", FAIL, f"no SELF_TEST result within {o.self_test_timeout_s} s")
    ev = result()
    assert ev is not None
    fails = bit_names(ev.requested, SELF_TEST_FAIL_BITS)
    warns = bit_names(ev.commanded, SELF_TEST_WARN_BITS)
    if o.simulated:
        warns = [x for x in warns if x != "SIM_OR_BENCH"]  # expected for the host simulator
    data = {"fail_mask": ev.requested, "warn_mask": ev.commanded, "fail": fails, "warn": warns,
            "nano_result": ev.reason}
    if fails:
        return Check("nano_self_test", FAIL, "Nano self test FAIL: " + ", ".join(fails), data)
    if warns:
        return Check("nano_self_test", WARN, "warnings: " + ", ".join(warns), data)
    return Check("nano_self_test", PASS, "Nano self test PASS", data)


def _check_nano_status(session: NanoSession, w: _Watch, o: PreflightOptions) -> Check:
    try:  # CGS carries the telemetry seq, not the command seq: any CGS received after STATUS is current
        rep = session.request("STATUS", reply=Status, match_seq=False, timeout_s=o.reply_timeout_s)
    except CommandTimeout:
        return Check("nano_status", FAIL, "no reply to STATUS")
    if rep.message is None:
        why = f"malformed CGS: {rep.malformed[0]}" if rep.malformed else \
            f"no CGS status frame within {o.reply_timeout_s} s after the STATUS ACK"
        return Check("nano_status", FAIL, why)
    st: Status = rep.message  # type: ignore[assignment]
    flags = [n for n, b in (("POWER_ON", 1), ("EXTERNAL", 2), ("BROWNOUT", 4), ("WATCHDOG", 8)) if st.reset_flags & b]
    data = {"state": st.state, "trip": TRIP_NAMES.get(st.trip, str(st.trip)), "locks": st.locks,
            "reset_flags": flags, "free_ram": st.free_ram, "rx_crc_err": st.rx_crc_err,
            "rx_format_err": st.rx_format_err, "loop_overruns": st.loop_overruns}
    if st.state == "TRIPPED":
        return Check("nano_status", FAIL, f"TRIPPED ({data['trip']}): fix the cause, then RESET", data)
    notes = []
    if "WATCHDOG" in flags or "BROWNOUT" in flags:
        notes.append("last reset: " + "/".join(f for f in flags if f in ("WATCHDOG", "BROWNOUT")))
    if 0 < st.free_ram < 150:
        notes.append(f"low free RAM ({st.free_ram} B)")
    if st.loop_overruns:
        notes.append(f"{st.loop_overruns} loop overruns")
    text = f"{st.state}, locks {','.join(st.locks) or '-'}"
    return Check("nano_status", WARN if notes else PASS, "; ".join([text] + notes), data)


def _check_telemetry_and_measurement(session: NanoSession, w: _Watch, o: PreflightOptions) -> List[Check]:
    mark, err0 = len(w.frames), session.frame_errors
    session.pump(o.watch_s, keepalive_s=None)
    tel = [r.message for r in w.since(mark, Telemetry)]
    n_frames = len(w.frames) - mark
    errors = session.frame_errors - err0
    expected = int(o.watch_s)
    seqs = [t.seq for t in tel]
    gaps = sum(1 for a, b in zip(seqs, seqs[1:]) if (b - a) % 65536 != 1)
    data = {"window_s": o.watch_s, "cgt": len(tel), "frames": n_frames, "frame_errors": errors, "seq_gaps": gaps}
    out = []
    if not tel:
        out.append(Check("telemetry", FAIL, f"no telemetry in {o.watch_s} s (Nano loop stalled?)", data))
        out.append(Check("measurement", SKIP, "no telemetry"))
        return out
    if errors > max(1, n_frames // 5):
        out.append(Check("telemetry", FAIL, f"{errors} bad frames out of {n_frames + errors}: noisy link", data))
    elif errors or gaps or len(tel) < expected - 1:
        out.append(Check("telemetry", WARN, f"{len(tel)} CGT in {o.watch_s} s, {errors} bad frames, {gaps} gaps", data))
    else:
        out.append(Check("telemetry", PASS, f"{len(tel)} CGT in {o.watch_s} s", data))

    last = tel[-1]
    vq = sorted({f for t in tel for f in t.voltage_quality_flags})
    tq = [sorted({t.temp_quality[i] for t in tel}) for i in range(4)]
    spread = []
    for i in range(4):
        vals = [t.cell_mv[i] for t in tel if t.cell_mv[i] is not None]
        spread.append(max(vals) - min(vals) if vals else None)
    mdata = {"cell_mv": last.cell_mv, "pack_mv": None if None in last.cell_mv else sum(last.cell_mv),
             "voltage_quality": vq, "temperature_quality": tq, "temp_cc": last.temp_cc, "rest_spread_mv": spread}
    fails = [f for f in vq if f in VQ_FAIL]
    fails += [f"NTC{i + 1} {q}" for i in range(4) for q in tq[i] if q in ("SHORT_OR_LOW", "ADC_INVALID")]
    warns = []
    if "PACK_NEAR_ZERO" in vq:
        warns.append("pack ~0 V: empty pack or harness not connected (check with a meter)")
    sat = [str(i + 1) for i in range(4) if "SATURATED_OR_OPEN" in tq[i]]
    if sat:
        warns.append(f"NTC {','.join(sat)} saturated: <= ~25 degC or open (heaters stay blocked)")
    unstable = [str(i + 1) for i in range(4) if spread[i] is not None and spread[i] > o.rest_spread_mv]
    if unstable and not (last.charge or last.discharge):
        warns.append(f"cell {','.join(unstable)} reading moves > {o.rest_spread_mv} mV at rest")
    if fails:
        out.append(Check("measurement", FAIL, "invalid measurement: " + ", ".join(fails), mdata))
    elif warns:
        out.append(Check("measurement", WARN, "; ".join(warns), mdata))
    else:
        out.append(Check("measurement", PASS, f"cells {last.cell_mv} mV, temperatures valid", mdata))
    return out


def _missing(rep, tag: str, cmd: str, o: PreflightOptions) -> str:
    if not rep.ack.ok:
        return f"{cmd} refused: {rep.ack.text}"
    if rep.malformed:
        return f"malformed {tag} reply to {cmd}: {rep.malformed[0]}"
    return f"no {tag} frame with seq {rep.ack.seq} within {o.reply_timeout_s} s after the {cmd} ACK"


def _check_adc_path(session: NanoSession, w: _Watch, o: PreflightOptions) -> Check:
    try:  # the CGK of THIS CAL (same seq), even when it arrives in a later read than the ACK
        rep = session.request("CAL", reply=CalibrationInfo, timeout_s=o.reply_timeout_s)
    except CommandTimeout:
        return Check("adc_path", FAIL, "no reply to CAL")
    if rep.message is None:
        return Check("adc_path", FAIL, _missing(rep, "CGK", "CAL", o), {"cal_seq": rep.ack.seq})
    cal: CalibrationInfo = rep.message  # type: ignore[assignment]
    mark = len(w.frames)
    if not _wait(session, lambda: bool(w.since(mark, Telemetry)), 1.5):
        return Check("adc_path", FAIL, "no telemetry to compare with")
    tel: Telemetry = w.since(mark, Telemetry)[-1].message
    try:  # right after the CGT: the same 5 Hz sample (next sample is 200 ms later); the CGD of THIS RAW
        raw_rep = session.request("RAW", reply=RawAdc, timeout_s=o.reply_timeout_s)
    except CommandTimeout:
        return Check("adc_path", FAIL, "no reply to RAW")
    if raw_rep.message is None:
        return Check("adc_path", FAIL, _missing(raw_rep, "CGD", "RAW", o), {"raw_seq": raw_rep.ack.seq})
    raw = raw_rep.message.raw  # type: ignore[union-attr]
    data: Dict[str, object] = {"raw": raw, "calibration": f"{cal.source}/{cal.cal_id}", "aref_mv": cal.aref_mv}
    if None in tel.cell_mv:
        return Check("adc_path", SKIP, "cell voltages invalid in telemetry (see measurement)", data)
    taps_fw = [sum(tel.cell_mv[:i + 1]) for i in range(4)]
    taps_pi = [tap_mv_from_raw(raw[i], cal, i) for i in range(4)]
    diff = max(abs(a - b) for a, b in zip(taps_fw, taps_pi))
    data.update({"taps_firmware_mv": taps_fw, "taps_from_raw_mv": taps_pi, "max_diff_mv": diff})
    notes = [] if cal.source == "MEASURED" else ["NOMINAL (unmeasured) calibration"]
    if diff > 3 * o.adc_path_tol_mv:
        return Check("adc_path", FAIL, f"taps from raw differ by {diff} mV: conversion / calibration mismatch", data)
    if diff > o.adc_path_tol_mv:
        return Check("adc_path", WARN, f"taps from raw differ by {diff} mV", data)
    return Check("adc_path", WARN if notes else PASS, "; ".join([f"raw -> taps agree ({diff} mV)"] + notes), data)


def _check_ina219(source: Optional[CurrentSource], error: Optional[BaseException], o: PreflightOptions,
                  sleep: Callable[[float], None]) -> Check:
    if error is not None:
        return Check("ina219", FAIL, f"INA219 not reachable: {error} (check H9_1 wiring, `i2cdetect -y 1`)")
    if source is None:
        return Check("ina219", SKIP, "INA219 not enabled (--ina219): pack current will be null / INA219_UNAVAILABLE")
    data: Dict[str, object] = {}
    probe = getattr(source, "probe", None)
    if probe is not None:
        try:
            info = probe()
        except OSError as exc:
            return Check("ina219", FAIL, f"I2C read failed: {exc}")
        data.update(info)
        if info.get("config_register") != info.get("expected"):
            return Check("ina219", FAIL, f"config register {info.get('config_register')} != {info.get('expected')}: "
                                         "wrong device at this address?", data)
    reads = []
    for _ in range(o.ina219_samples):
        reads.append(source.read())
        sleep(o.ina219_period_s)
    qualities = sorted({r.quality for r in reads})
    vals = [r.current_a for r in reads if r.quality == "VALID" and r.current_a is not None]
    data.update({"reads": len(reads), "quality": qualities})
    if len(vals) < len(reads):
        return Check("ina219", FAIL, f"INA219 reads not valid: {','.join(qualities)}", data)
    mean = statistics.fmean(vals)
    data.update({"mean_a": round(mean, 6), "std_a": round(statistics.pstdev(vals), 6)})
    if abs(mean) > o.ina219_zero_tol_a:
        return Check("ina219", WARN, f"{mean * 1000:.1f} mA with every output off: offset, leakage or shunt wiring "
                                     "(H4_1) / sign - check before trusting the current", data)
    return Check("ina219", PASS, f"{len(vals)} reads VALID, {mean * 1000:.1f} mA at rest", data)


def ntp_synchronized() -> Optional[bool]:
    """True / False from systemd-timedated, None when unknown (no timedatectl)."""
    try:
        out = subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"], capture_output=True,
                             text=True, timeout=3)
    except (OSError, subprocess.SubprocessError):
        return None
    value = out.stdout.strip().lower()
    return {"yes": True, "no": False}.get(value)


def _check_clock(o: PreflightOptions, now: dt.datetime, ntp: Optional[bool]) -> Check:
    data = {"system_utc": _iso(now), "ntp_synchronized": ntp}
    if now.year < o.min_year:
        return Check("clock", FAIL, f"system clock says {now.year}: not set yet (Pi without RTC before NTP) - "
                                    "timestamps would be wrong", data)
    if ntp is False:
        return Check("clock", WARN, "NTP not synchronised: timestamps may drift", data)
    if ntp is None:
        return Check("clock", SKIP, "NTP state unknown (no timedatectl)", data)
    return Check("clock", PASS, "system clock NTP-synchronised", data)


def _check_storage(data_dir: Path, o: PreflightOptions) -> Check:
    data_dir = Path(data_dir)
    try:
        data_dir.mkdir(parents=True, exist_ok=True)
        probe = data_dir / ".selftest-write-probe"
        with open(probe, "wb") as f:
            f.write(b"ok\n")
            f.flush()
            os.fsync(f.fileno())
        probe.unlink()
    except OSError as exc:
        return Check("storage", FAIL, f"data directory not writable: {exc}", {"data_dir": str(data_dir)})
    free_mb = shutil.disk_usage(data_dir).free // 1_000_000
    parts = sorted(p.name for p in (data_dir / "spool").glob("*.part")) if (data_dir / "spool").exists() else []
    data = {"data_dir": str(data_dir), "free_mb": free_mb, "unfinished_files": parts[:20]}
    if free_mb < o.min_free_mb:
        return Check("storage", FAIL, f"only {free_mb} MB free", data)
    notes = []
    if free_mb < o.warn_free_mb:
        notes.append(f"low disk space ({free_mb} MB)")
    if parts:
        notes.append(f"{len(parts)} unfinished file(s) from an abnormal stop (recovered when collection starts)")
    return Check("storage", WARN if notes else PASS, "; ".join([f"{free_mb} MB free"] + notes), data)


# --------------------------------------------------------------------------------------- driver

def run_preflight(session: NanoSession, data_dir: Path, opts: Optional[PreflightOptions] = None,
                  current_source: Optional[CurrentSource] = None, current_error: Optional[BaseException] = None,
                  meta: Optional[Dict[str, object]] = None, system_now: Callable[[], dt.datetime] = None,
                  ntp: Callable[[], Optional[bool]] = ntp_synchronized,
                  sleep: Callable[[float], None] = time.sleep) -> Report:
    o = opts or PreflightOptions()
    now = system_now or (lambda: dt.datetime.now(UTC))
    started = session.t.now_utc()
    w = _Watch(session)
    checks: List[Check] = []
    try:
        link = _check_link(session, w, o)
        checks.append(link)
        if link.status == FAIL:
            for name in ("firmware", "nano_self_test", "nano_status", "telemetry", "measurement", "adc_path"):
                checks.append(Check(name, SKIP, "no link"))
        else:
            checks.append(_check_firmware(session, w, o))
            checks.append(_check_nano_self_test(session, w, o))
            checks.append(_check_nano_status(session, w, o))
            checks.extend(_check_telemetry_and_measurement(session, w, o))
            checks.append(_check_adc_path(session, w, o))
        checks.append(_check_ina219(current_source, current_error, o, sleep))
        checks.append(_check_clock(o, now(), ntp()))
        checks.append(_check_storage(data_dir, o))
    finally:
        w.close()
    v = session.version
    info = dict(meta or {})
    caps = session.capabilities
    info.update({"simulated": o.simulated,
                 "capabilities": {"family": caps.family, "self_test": caps.self_test,
                                  "staged_power": caps.staged_power, "diagnostic_frames": caps.diagnostic_frames},
                 "firmware": None if v is None else {"version": v.firmware, "protocol": v.protocol,
                                                      "map_version": v.map_version, "flags": v.flags}})
    return Report(checks, started, session.t.now_utc(), info, w.transcript)


def write_report(report: Report, data_dir: Path) -> Path:
    """One-record JSONL in the outbox (uploaded with the data files)."""
    out = Path(data_dir) / "outbox"
    out.mkdir(parents=True, exist_ok=True)
    stamp = report.started_utc.astimezone(UTC).strftime("%Y%m%dT%H%M%S%fZ")
    n = 1
    while (out / f"selftest-{stamp}-{n:05d}.jsonl").exists():
        n += 1
    final = out / f"selftest-{stamp}-{n:05d}.jsonl"
    tmp = final.with_suffix(".tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(json.dumps(report.to_record(), separators=(",", ":"), ensure_ascii=False) + "\n")
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, final)
    return final


def format_report(report: Report) -> str:
    lines = [f"[{c.status}] {c.name:<15} {c.detail}" for c in report.checks]
    lines.append(f"BOOT CHECK: {report.result}")
    act = report.actuation
    lines.append("actuation: " + ("allowed" if act["allowed"] else "NOT allowed - " + "; ".join(act["reasons"])))
    return "\n".join(lines)


__all__ = ["Check", "FAIL", "PASS", "PreflightOptions", "Report", "SKIP", "UNSUPPORTED", "WARN", "format_report",
           "ntp_synchronized", "run_preflight", "tap_mv_from_raw", "write_report"]
