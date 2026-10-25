# CellGuard data collector - supervised staged charge / discharge cycle (RAMP / DRAIN)
# Author: Yongho Ahn <nanhosoft@gmail.com>
"""The rig-proven sequence of ``cellguard_cycle.py`` (CellGuardNano_LoadVerify
0.1.6) running inside the collector's single UART session, so the cycle is
controlled and recorded through ONE open port (no second program on
/dev/serial0).

Kept exactly from cellguard_cycle.py (tests compare against it):

* 1 s heartbeat (PING) while a stage runs; every PING must be ACKed
* checkpoints: stepped per-cell targets from the highest measured cell
* before every stage: Nano READY with the relay open, then a NEW ``ARM``
  (a finished stage disarms the Nano)
* a stage ends only with CHARGE_STOP,TARGET / DRAIN_STOP,DRAIN_TARGET followed
  by RELAY_OFF and relay = 0 in CGS; any other stop reason (LOAD_TAP_UNSTABLE,
  TIMEOUT, BUDGET, limits) aborts the cycle
* at least 2.3 s rest and two fresh CGT readings with the relay open, then the
  rested cell voltages are checked: spread <= 80 mV, target +-40 mV, every cell
  moved in the right direction (discharge: at least one cell fell > 16 mV)
* stage timeouts 32 s (RAMP) / 37 s (DRAIN) on top of the Nano's own 25 / 30 s caps
* TRIP or TRIPPED state -> abort; never RESET or re-ARM after a trip
* on any abort, error, timeout or Ctrl-C: OFF is sent; no automatic retry

Voltages are RESTED millivolts PER CELL (not pack voltage). Run only with a
person watching the rig.
"""
from __future__ import annotations

import dataclasses
from typing import Callable, List, Optional, Sequence, Tuple

from .diagnostics import DiagnosticRecorder
from .protocol import INVALID_I16, Event, LoadTapSuspect, Status, Telemetry
from .session import CommandTimeout, NanoSession, Received, UnsupportedCommand

HEARTBEAT_SECONDS = 1.0
CHARGE_TIMEOUT_SECONDS = 32.0  # Nano has a separate hard cap of 25 s
DRAIN_TIMEOUT_SECONDS = 37.0   # Nano has a separate hard cap of 30 s
SETTLE_SECONDS = 2.3
CHARGE_MARGIN_MV = 40
DRAIN_MARGIN_MV = 40
MAX_SPREAD_MV = 80
IDLE_TIMEOUT_SECONDS = 5.0
MAX_BAD_FRAMES = 3
PRINTED_EVENTS = ("TRIP", "CHARGE_STOP", "DRAIN_STOP", "CHECKPOINT", "LOAD_TAP_SUSPECT", "LOAD_TAP_VERIFY")


class CycleAborted(RuntimeError):
    pass


def checkpoints(current_max: int, target: int, step: int) -> List[int]:
    """Future per-cell setpoints without repeating earlier levels (same as cellguard_cycle.py)."""
    if current_max >= target:
        return []
    first = max(150, ((current_max // step) + 1) * step)
    if first - current_max < 16:
        first += step
    levels = list(range(first, target + 1, step))
    if not levels or levels[-1] != target:
        levels.append(target)
    return levels


def verify_voltage(kind: str, target: int, before: Sequence[int], after: Sequence[int]) -> None:
    """Rested-voltage acceptance of one stage (same rules as cellguard_cycle.py)."""
    lo, hi = min(after), max(after)
    spread = hi - lo
    if spread > MAX_SPREAD_MV:
        raise CycleAborted(f"Cell spread {spread}mV exceeds {MAX_SPREAD_MV}mV")
    if kind == "RAMP":
        if lo < target - CHARGE_MARGIN_MV or hi > target + CHARGE_MARGIN_MV:
            raise CycleAborted(f"Charging checkpoint {target}mV was not achieved: {list(after)}")
        if not all(a >= b for a, b in zip(after, before)):
            raise CycleAborted(f"Some cell fell rather than charged: {list(before)} => {list(after)}")
    else:
        if lo < target - DRAIN_MARGIN_MV or hi > target + DRAIN_MARGIN_MV:
            raise CycleAborted(f"Discharge checkpoint {target}mV was not achieved: {list(after)}")
        if not all(a <= b for a, b in zip(after, before)):
            raise CycleAborted(f"Some cell rose rather than discharged: {list(before)} => {list(after)}")
        if not any(a < b - 16 for a, b in zip(after, before)):
            raise CycleAborted("Discharge did not measurably reduce the stored voltage")


def validate_targets(charge_to: Optional[int], discharge_to: Optional[int], step: int) -> None:
    """Same argument limits as cellguard_cycle.py (inside the firmware's RAMP / DRAIN ranges)."""
    if charge_to is None or not 150 <= charge_to <= 1000:
        raise ValueError("--charge-to must be in 150..1000 mV per cell")
    if discharge_to is not None and not 100 <= discharge_to <= 950:
        raise ValueError("--discharge-to must be in 100..950 mV per cell")
    if not 50 <= step <= 200:
        raise ValueError("--step must be 50..200 mV")
    if discharge_to is not None and discharge_to >= charge_to:
        raise ValueError("--discharge-to must be lower than --charge-to")


@dataclasses.dataclass
class StageResult:
    kind: str
    target_mv: int
    before_mv: List[int]
    after_mv: Optional[List[int]]
    outcome: str          # PASS / ABORTED
    reason: Optional[str] = None


class CycleRunner:
    """Drives RAMP / DRAIN stages over an existing NanoSession (the collector keeps recording)."""

    def __init__(self, session: NanoSession, diagnostics: Optional[DiagnosticRecorder] = None,
                 log: Callable[[str], None] = print, step_s: float = 0.15):
        self.s = session
        self.diag = diagnostics
        self.log = log
        self.step_s = step_s
        self.state: Optional[str] = None
        self.relay: Optional[int] = None
        self.trip = 0
        self.last_cgt: Optional[Tuple[int, ...]] = None
        self.last_cgt_at = -1.0
        self.events: List[Tuple[str, str, float]] = []
        self.results: List[StageResult] = []
        self.bad_frames0 = session.frame_errors
        self.last_ping_at = session.t.monotonic()
        session.subscribe(self._on_frame)

    # ------------------------------------------------------------------ link state
    def _on_frame(self, rec: Received) -> None:
        m = rec.message
        now = self.s.t.monotonic()
        if isinstance(m, Status):
            self.state, self.trip, self.relay = m.state, m.trip, int(m.relay)
        elif isinstance(m, Telemetry):
            # rested CGT cells only (CGY is a different frame and never used here). An invalid cell keeps the
            # firmware sentinel -32768, so the voltage checks fail on it - as in cellguard_cycle.py
            self.last_cgt = tuple(INVALID_I16 if v is None else int(v) for v in m.cell_mv)
            self.last_cgt_at = now
        elif isinstance(m, Event):
            self.events.append((m.code, m.reason, now))
            if len(self.events) > 500:
                del self.events[:100]
            if m.code in PRINTED_EVENTS:
                self.log(f"    {m.code}: {m.reason}")
        elif isinstance(m, LoadTapSuspect):
            self.log(f"    ON-LOAD SUSPECT: taps {m.tap_mv}mV; MOS OFF, verifying at rest")

    def _now(self) -> float:
        return self.s.t.monotonic()

    def _poll(self, seconds: Optional[float] = None) -> None:
        self.s.poll(self.step_s if seconds is None else seconds)
        self.s.run_idle_hooks()  # the collector writes slots / requests RAW in the same session
        if self.s.frame_errors - self.bad_frames0 > MAX_BAD_FRAMES:
            raise CycleAborted("Repeated malformed/CRC-invalid Nano telemetry")

    def _tripped(self) -> bool:
        return self.state == "TRIPPED" or bool(self.trip)

    def command(self, text: str, *args: int) -> str:
        try:
            ack = self.s.command(text, *args)
        except CommandTimeout as exc:
            raise CycleAborted(f"No ACK for {text}: UART link lost?") from exc
        except UnsupportedCommand as exc:
            raise CycleAborted(str(exc)) from exc
        if not ack.ok:
            raise CycleAborted(f"Nano rejected {text}{''.join(',' + str(a) for a in args)}: {ack.text}")
        if self._tripped():
            raise CycleAborted(f"Nano latched trip code {self.trip}")
        return ack.text

    def keepalive(self) -> None:
        if self._now() - self.last_ping_at >= HEARTBEAT_SECONDS:
            self.last_ping_at = self._now()
            self.command("PING")

    def wait_idle(self, timeout: float = IDLE_TIMEOUT_SECONDS) -> None:
        end = self._now() + timeout
        while self._now() < end:
            self._poll()
            if self._tripped():
                raise CycleAborted(f"Nano trip {self.trip}, re-arm prohibited")
            if self.state == "READY" and self.relay == 0:
                return
        raise CycleAborted(f"Nano not idle/READY (state={self.state}, relay={self.relay})")

    def settle_readings(self, since: float) -> Tuple[int, ...]:
        """Fresh measurements after relay OFF and after 2.3 s rest (at least two CGT)."""
        end = self._now() + max(SETTLE_SECONDS + 1.2, 4.0)
        last = None
        samples: List[Tuple[int, ...]] = []
        while self._now() < end:
            self._poll()
            if self._tripped():
                raise CycleAborted(f"Nano trip {self.trip} while settling")
            if self.relay == 0 and self.last_cgt_at > since and self.last_cgt_at != last:
                last = self.last_cgt_at
                samples.append(self.last_cgt)  # type: ignore[arg-type]
            if self._now() - since >= SETTLE_SECONDS and len(samples) >= 2:
                return samples[-1]
        raise CycleAborted("No fresh, repeatable rested CGT voltage after relay OFF")

    # ------------------------------------------------------------------ stages
    def run_stage(self, kind: str, target: int, before: Sequence[int]) -> Tuple[int, ...]:
        self.wait_idle()
        self.log(f"[{kind}] target {target}mV/cell | start {list(before)}")
        result = StageResult(kind, target, list(before), None, "ABORTED")
        self.results.append(result)
        try:
            self.command("ARM")  # every stage needs a new ARM (a finished stage disarms)
            self.command(kind, target)
            marker = len(self.events)
            stop_name = "CHARGE_STOP" if kind == "RAMP" else "DRAIN_STOP"
            success_reason = "TARGET" if kind == "RAMP" else "DRAIN_TARGET"
            deadline = self._now() + (CHARGE_TIMEOUT_SECONDS if kind == "RAMP" else DRAIN_TIMEOUT_SECONDS)
            stop_seen = False
            relay_off_at: Optional[float] = None
            while self._now() < deadline:
                self.keepalive()
                self._poll()
                if self._tripped():
                    raise CycleAborted(f"Nano TRIPPED code={self.trip}")
                new, marker = self.events[marker:], len(self.events)
                for code, reason, at in new:
                    if code == "TRIP":
                        raise CycleAborted("Nano hardware trip: " + reason)
                    if code == stop_name:
                        stop_seen = True
                        if reason != success_reason:  # LOAD_TAP_UNSTABLE, TIMEOUT, BUDGET, limits ...
                            raise CycleAborted(f"{kind} stopped before goal: {reason}")
                    if code == "RELAY_OFF" and stop_seen:
                        relay_off_at = self._now()
                if stop_seen and self.relay == 0 and relay_off_at is not None:
                    after = self.settle_readings(relay_off_at)
                    self.wait_idle()
                    verify_voltage(kind, target, before, after)
                    self.log(f"    PASS {list(after)}mV | spread {max(after) - min(after)}mV")
                    result.after_mv, result.outcome = list(after), "PASS"
                    return after
            raise CycleAborted(f"{kind} {target}mV timed out without a clean stop")
        except CycleAborted as exc:
            result.reason = str(exc)
            if self.last_cgt is not None:
                result.after_mv = list(self.last_cgt)
            raise
        finally:
            if self.diag is not None:
                self.diag.write("CYCLE_STAGE", dataclasses.asdict(result))

    def run(self, charge_to: int, discharge_to: Optional[int], step: int) -> Tuple[int, ...]:
        """Charge in steps to ``charge_to`` then (optionally) drain to ``discharge_to``. OFF on any failure."""
        validate_targets(charge_to, discharge_to, step)
        try:
            caps = self.s.capabilities
            if not caps.staged_power:
                raise CycleAborted(f"firmware {caps.firmware or 'unknown'} has no RAMP/DRAIN "
                                   "(cycle needs 0.1.6-LOAD-VERIFY); nothing was energised")
            start = self._now()
            while self.last_cgt is None and self._now() - start < 3.0:
                self._poll(0.2)
            if self.last_cgt is None:
                raise CycleAborted("No CGT telemetry from Nano. Check device/baud, not wiring.")
            self.wait_idle()
            initial = self.last_cgt
            if min(initial) < -20 or max(initial) - min(initial) > MAX_SPREAD_MV:
                raise CycleAborted(f"Initial cell voltages unsafe for staging: {list(initial)}")
            levels = checkpoints(max(initial), charge_to, step)
            if not levels:
                raise CycleAborted("Target already at/below highest cell; nothing to charge")
            self.log(f"Charge plan: {' -> '.join(map(str, levels))} mV per cell")
            volts = initial
            for target in levels:
                volts = self.run_stage("RAMP", target, volts)
            if discharge_to is not None:
                if min(volts) <= discharge_to:
                    raise CycleAborted("Discharge target must be BELOW all measured cells")
                volts = self.run_stage("DRAIN", discharge_to, volts)
            self.log(f"PASS: charge + discharge cycle complete. Final cells {list(volts)}mV")
            self._result("PASS", None, volts)
            return volts
        except BaseException as exc:
            self._result("ABORTED", f"{type(exc).__name__}: {exc}", self.last_cgt)
            self.fail_safe_off()
            raise

    def fail_safe_off(self) -> None:
        try:
            self.s.command("OFF")
            self._poll(0.3)
            self.log("Fail-safe: OFF sent to Nano")
        except Exception as err:  # link gone: the Nano's 5 s heartbeat timeout switches off by itself
            self.log(f"Could not confirm OFF ({err}); Nano heartbeat timer will trip.")

    def _result(self, outcome: str, reason: Optional[str], cells) -> None:
        if self.diag is not None:
            self.diag.write("CYCLE_RESULT", {"outcome": outcome, "reason": reason,
                                             "final_cells_mv": list(cells) if cells else None,
                                             "stages": [dataclasses.asdict(r) for r in self.results]})


__all__ = ["CycleAborted", "CycleRunner", "StageResult", "checkpoints", "validate_targets", "verify_voltage"]
