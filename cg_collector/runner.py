# CellGuard data collector - experiment sequence runner and interactive control shell
# Author: Yongho Ahn <nanhosoft@gmail.com>
"""One persistent session per run: SYNC -> (RESET) -> ARM -> commands, with a 1 s
heartbeat and continuous data collection between steps.

Safety rules enforced here (the firmware enforces its own independently):
* actuation needs ``supervised=True`` (a person is watching the rig);
* any error, timeout, refusal or Ctrl-C ends the run with ``OFF``;
* a refused actuation command stops the sequence (no blind continuation).
"""
from __future__ import annotations

import dataclasses
import json
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from .session import ACTUATION_COMMANDS, CommandTimeout, NanoSession

STEP_COMMANDS = {"SYNC", "PING", "STATUS", "RAW", "CAL", "ARM", "OFF", "TRIP", "RESET", "CHARGE", "DISCHARGE",
                 "ESR", "LEAK", "HEAT", "BENCH"}
ARG_COUNTS = {"CHARGE": 1, "DISCHARGE": 1, "ESR": 2, "LEAK": 2, "HEAT": 2, "BENCH": 2}


class PlanError(ValueError):
    pass


@dataclasses.dataclass
class Step:
    op: str               # command name, or WAIT
    args: List[int]
    seconds: float = 0.0
    allow_error: bool = False


def load_plan(path: Path) -> Dict[str, Any]:
    text = Path(path).read_text(encoding="utf-8")
    if path.suffix in (".yaml", ".yml"):
        import yaml  # optional dependency

        return yaml.safe_load(text)
    return json.loads(text)


def parse_plan(plan: Dict[str, Any]) -> List[Step]:
    if not isinstance(plan, dict) or not isinstance(plan.get("steps"), list):
        raise PlanError("plan must be an object with a 'steps' list")
    steps = []
    for i, raw in enumerate(plan["steps"], 1):
        if isinstance(raw, str):
            raw = {"cmd": raw}
        if "wait" in raw:
            secs = float(raw["wait"])
            if secs < 0:
                raise PlanError(f"step {i}: negative wait")
            steps.append(Step("WAIT", [], seconds=secs))
            continue
        name = str(raw.get("cmd", "")).upper()
        if name not in STEP_COMMANDS:
            raise PlanError(f"step {i}: unknown command {name!r}")
        args = [int(a) for a in raw.get("args", [])]
        if len(args) != ARG_COUNTS.get(name, 0):
            raise PlanError(f"step {i}: {name} expects {ARG_COUNTS.get(name, 0)} args")
        steps.append(Step(name, args, allow_error=bool(raw.get("allow_error", False))))
    if not steps or steps[0].op != "SYNC":
        raise PlanError("the first step must be SYNC")
    return steps


@dataclasses.dataclass
class StepResult:
    op: str
    args: List[int]
    ok: bool
    text: str


class RunAborted(RuntimeError):
    pass


def run_steps(session: NanoSession, steps: Sequence[Step], supervised: bool, heartbeat_s: float = 1.0
              ) -> List[StepResult]:
    if not supervised and any(s.op in ACTUATION_COMMANDS for s in steps):
        raise PlanError("plan contains actuation commands: run only with a person supervising (--i-am-supervising)")
    results: List[StepResult] = []
    try:
        for step in steps:
            if step.op == "WAIT":
                session.pump(step.seconds, keepalive_s=heartbeat_s)
                results.append(StepResult("WAIT", [], True, f"{step.seconds}s"))
                continue
            ack = session.command(step.op, *step.args)
            results.append(StepResult(step.op, list(step.args), ack.ok, ack.text))
            if not ack.ok and not step.allow_error:
                raise RunAborted(f"{step.op} {step.args} refused: {ack.text}")
    finally:
        try:
            ack = session.command("OFF")  # always leave the rig de-energised
            results.append(StepResult("OFF", [], ack.ok, ack.text))
        except CommandTimeout:
            results.append(StepResult("OFF", [], False, "NO_ACK"))
    return results


class Pump(threading.Thread):
    """Background receive + heartbeat for the interactive shell (real time only)."""

    def __init__(self, session: NanoSession, heartbeat_s: float = 1.0):
        super().__init__(name="cg-pump", daemon=True)
        self.session = session
        self.heartbeat_s = heartbeat_s
        self.stop_event = threading.Event()
        self.error: Optional[BaseException] = None

    def run(self) -> None:
        try:
            while not self.stop_event.is_set():
                self.session.keepalive(self.heartbeat_s)
                self.session.poll(0.05)
                self.session.run_idle_hooks()
        except BaseException as exc:  # surfaced by the shell
            self.error = exc

    def stop(self) -> None:
        self.stop_event.set()
        self.join(timeout=3)


def shell_command(session: NanoSession, line: str, supervised: bool) -> str:
    """Executes one shell line (e.g. ``charge 1``, ``esr 2 1``, ``status``); returns the reply text."""
    parts = line.split()
    if not parts:
        return ""
    name = parts[0].upper()
    if name not in STEP_COMMANDS:
        return f"unknown command {name}"
    try:
        args = [int(a) for a in parts[1:]]
    except ValueError:
        return "arguments must be integers"
    if len(args) != ARG_COUNTS.get(name, 0):
        return f"{name} expects {ARG_COUNTS.get(name, 0)} args"
    if name in ACTUATION_COMMANDS and not supervised:
        return "refused: start the shell with --i-am-supervising for actuation"
    ack = session.command(name, *args)
    return f"{'OK' if ack.ok else 'ERR'} {ack.text}"
