# CellGuard data collector - firmware families and what each one supports
# Author: Yongho Ahn <nanhosoft@gmail.com>
"""The collector talks to more than one Nano firmware. The CGV version string
decides which commands may be sent and which checks exist; nothing is assumed
for an unknown version (no actuation at all).

=====================  =====================  =====================================
family                 CGV firmware            notes
=====================  =====================  =====================================
``load_verify_0_1_6``  ``0.1.6-LOAD-VERIFY``   rig commissioning build: RAMP/DRAIN with a
                                               per-cell target, CHARGE,1 / DISCHARGE,1
                                               refused (USE_RAMP / USE_DRAIN), no SELFTEST,
                                               CGY / CGX diagnostics, REF3040 (4.096 V),
                                               a finished stage disarms (new ARM per stage)
``v0_2``               ``0.2.x``               power-on self test + SELFTEST, CHARGE/DISCHARGE
``v0_1``               ``0.1.0``               no self test, CHARGE/DISCHARGE
``unknown``            anything else           read-only commands only
=====================  =====================  =====================================

Sources: ``CellGuardNano_LoadVerify`` (0.1.6, see tests/fw016/SHA256SUMS) and
``2026-02-elev/arudino-firmware`` (0.1.0 / 0.2.0).
"""
from __future__ import annotations

import dataclasses
import re
from typing import FrozenSet, Optional, Sequence, Tuple

from .protocol import Version

READ_ONLY = frozenset({"SYNC", "PING", "STATUS", "RAW", "CAL", "OFF", "TRIP"})


class UnsupportedCommand(RuntimeError):
    """The detected firmware does not support this command; nothing was sent."""


@dataclasses.dataclass(frozen=True)
class Capabilities:
    family: str
    firmware: Optional[str]
    commands: FrozenSet[str]                 # command names that may be sent
    power_on_commands: FrozenSet[str]        # CHARGE,1 / DISCHARGE,1 accepted
    self_test: bool                          # SELFTEST + CGE SELF_TEST (power-on self test)
    staged_power: bool                       # RAMP,<mV> / DRAIN,<mV>
    diagnostic_frames: bool                  # CGY / CGX
    stage_disarms: bool                      # a finished power stage disarms (ARM again)
    discharge_events: Tuple[str, str]        # (start, stop) CGE codes of the discharge path
    ramp_range_mv: Optional[Tuple[int, int]] = None
    drain_range_mv: Optional[Tuple[int, int]] = None
    known: bool = True

    def check(self, name: str, args: Sequence[int] = ()) -> None:
        """Raises UnsupportedCommand when ``name args`` must not be sent to this firmware."""
        if name not in self.commands:
            raise UnsupportedCommand(f"{name} is not supported by firmware {self.firmware or 'unknown'} "
                                     f"({self.family})" + (": run SYNC first" if not self.known else ""))
        if name in ("CHARGE", "DISCHARGE") and args and int(args[0]) and name not in self.power_on_commands:
            alt = "RAMP" if name == "CHARGE" else "DRAIN"
            raise UnsupportedCommand(f"{name},1 is refused by firmware {self.firmware}: use {alt},<mV per cell> "
                                     "(cg-collect cycle)")
        rng = {"RAMP": self.ramp_range_mv, "DRAIN": self.drain_range_mv}.get(name)
        if rng is not None and (not args or not rng[0] <= int(args[0]) <= rng[1]):
            raise UnsupportedCommand(f"{name} target must be {rng[0]}..{rng[1]} mV per cell")


_ACT_0X = frozenset({"ARM", "RESET", "CHARGE", "DISCHARGE", "ESR", "LEAK", "HEAT", "BENCH"})

LOAD_VERIFY_0_1_6 = Capabilities(
    family="load_verify_0_1_6", firmware="0.1.6-LOAD-VERIFY",
    # ESR / LEAK / HEAT stay hardware-locked in this build (never sent: not verified on the rig)
    commands=READ_ONLY | {"ARM", "RESET", "CHARGE", "DISCHARGE", "RAMP", "DRAIN"},
    power_on_commands=frozenset(), self_test=False, staged_power=True, diagnostic_frames=True,
    stage_disarms=True, discharge_events=("DRAIN_START", "DRAIN_STOP"),
    ramp_range_mv=(150, 1000), drain_range_mv=(100, 950))


def _v0x(version: str, family: str, self_test: bool) -> Capabilities:
    cmds = READ_ONLY | _ACT_0X | ({"SELFTEST"} if self_test else set())
    return Capabilities(family=family, firmware=version, commands=frozenset(cmds),
                        power_on_commands=frozenset({"CHARGE", "DISCHARGE"}), self_test=self_test,
                        staged_power=False, diagnostic_frames=False, stage_disarms=False,
                        discharge_events=("DISCHARGE_START", "DISCHARGE_STOP"))


def unknown(version: Optional[str] = None) -> Capabilities:
    return Capabilities(family="unknown", firmware=version, commands=READ_ONLY, power_on_commands=frozenset(),
                        self_test=False, staged_power=False, diagnostic_frames=False, stage_disarms=False,
                        discharge_events=("DISCHARGE_START", "DISCHARGE_STOP"), known=False)


def capabilities(version: Optional[Version]) -> Capabilities:
    """Capabilities for a CGV frame (None = not seen yet: read-only commands only)."""
    if version is None:
        return unknown()
    fw = version.firmware
    if fw == "0.1.6-LOAD-VERIFY":
        return LOAD_VERIFY_0_1_6
    m = re.fullmatch(r"(\d+)\.(\d+)\.(\d+)", fw)
    if m and (int(m.group(1)), int(m.group(2))) == (0, 2):
        return _v0x(fw, "v0_2", self_test=True)
    if fw == "0.1.0":
        return _v0x(fw, "v0_1", self_test=False)
    return unknown(fw)


__all__ = ["Capabilities", "LOAD_VERIFY_0_1_6", "UnsupportedCommand", "capabilities", "unknown"]
