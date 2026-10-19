# CellGuard data collector - Nano UART protocol v1.0 (framing + message parsing)
# Author: Yongho Ahn <nanhosoft@gmail.com>
"""Frames are ``$<payload>*<CRC16>\\n`` (CRC-16/CCITT-FALSE over the payload).

The authoritative definition is ``docs/uart_protocol.md`` in the
``arudino-firmware`` repository. ACK ``OK`` means "accepted"; every output
field reported by the Nano is a *commanded* state, never a measured one.
"""
from __future__ import annotations

import dataclasses
from typing import List, Optional, Union

PROTOCOL_VERSION = "1.0"
MAX_FRAME_BYTES = 192
INVALID_I16 = -32768

STATE_NAMES = ("BOOT_SAFE", "SELF_TEST", "LOCKED", "READY", "ARMED", "ACTIVE", "TRIPPED")
TRIP_NAMES = {
    0: "NONE", 1: "PACK_OV", 2: "CELL_OV", 3: "OVER_TEMP", 4: "HEARTBEAT_TIMEOUT",
    5: "ADC_SATURATED", 6: "TAP_INVERSION", 7: "AREF_INVALID", 8: "HOST_TRIP",
    9: "WATCHDOG_RESET", 10: "BROWNOUT_RESET", 11: "STALE_SAMPLE", 12: "INTERNAL",
    13: "CHARGE_NO_RESPONSE", 14: "TAP_STEP", 15: "TEMP_SENSOR_FAULT",
}
TEMP_QUALITY = ("VALID", "SATURATED_OR_OPEN", "SHORT_OR_LOW", "ADC_INVALID")
VQ_BITS = {"ADC_SATURATED": 1, "TAP_INVERSION": 2, "AREF_INVALID": 4, "STALE": 8, "PACK_NEAR_ZERO": 16}
LOCK_BITS = {"POWER_LOCKED": 0, "ESR_LOCKED": 1, "LEAK_LOCKED": 2, "HEAT_LOCKED": 3, "MEAS_INVALID": 4,
             "NOSYNC": 5, "TEMP_SENSOR_FAULT": 6, "SELF_TEST_FAILED": 7}
# Nano power-on self test (firmware >= 0.2.0): CGE SELF_TEST, requested = fail bits, commanded = warn bits
SELF_TEST_FAIL_BITS = {"AREF": 0, "ADC_SATURATED": 1, "TAP_INVERSION": 2, "ADC_UNSTABLE": 3, "NTC_FAULT": 4,
                       "CALIBRATION": 5, "RESET_CAUSE": 6}
SELF_TEST_WARN_BITS = {"PACK_NEAR_ZERO": 0, "NTC_SATURATED": 1, "CAL_NOMINAL": 2, "HW_LOCKED": 3, "SIM_OR_BENCH": 4}


def bit_names(mask: int, table: dict) -> List[str]:
    return [name for name, bit in table.items() if mask >> bit & 1]
CGV_FLAG_BITS = {
    "MAP_VERIFIED": 0, "THERMAL_PROTECTION_VERIFIED": 1, "CURRENT_SENSOR_DECLARED": 2,
    "PACK_RELAY_VERIFIED": 3, "ESR_RELAY_VERIFIED": 4, "POWER_MOS_VERIFIED": 5, "R17_VERIFIED": 6,
    "NTC_HW_REVISED": 7, "BENCH_BUILD": 8, "HOST_SIM_BUILD": 9, "POWER_UNLOCKED": 10,
    "ESR_UNLOCKED": 11, "LEAK_UNLOCKED": 12, "HEAT_UNLOCKED": 13,
}


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode_frame(payload: str) -> bytes:
    raw = payload.encode("ascii")
    if b"$" in raw or b"*" in raw or b"\n" in raw:
        raise ValueError("payload must not contain '$', '*' or newline")
    frame = b"$" + raw + b"*" + f"{crc16_ccitt_false(raw):04X}".encode() + b"\n"
    if len(frame) > MAX_FRAME_BYTES:
        raise ValueError(f"frame is {len(frame)} bytes, limit {MAX_FRAME_BYTES}")
    return frame


class FrameError(ValueError):
    pass


def decode_line(line: Union[str, bytes]) -> str:
    """Validates one received line and returns its payload. Raises FrameError."""
    if isinstance(line, bytes):
        try:
            line = line.decode("ascii")
        except UnicodeDecodeError as exc:
            raise FrameError("non-ASCII bytes") from exc
    line = line.rstrip("\n")
    if line.endswith("\r"):
        line = line[:-1]
    if len(line) + 1 > MAX_FRAME_BYTES:
        raise FrameError("frame too long")
    if not line.startswith("$") or len(line) < 7 or line[-5] != "*":
        raise FrameError("bad frame structure")
    payload, crc_text = line[1:-5], line[-4:]
    if "*" in payload or "$" in payload:
        raise FrameError("delimiter inside payload")
    if any(c not in "0123456789ABCDEF" for c in crc_text):
        raise FrameError("CRC must be 4 uppercase hex digits")
    if any(not (0x20 <= ord(c) <= 0x7E) for c in payload):
        raise FrameError("non-printable payload")
    if crc16_ccitt_false(payload.encode("ascii")) != int(crc_text, 16):
        raise FrameError("CRC mismatch")
    return payload


@dataclasses.dataclass
class Ack:
    seq: int
    ok: bool
    text: str  # detail for OK, reason for ERR


@dataclasses.dataclass
class Telemetry:
    seq: int
    millis: int
    cell_mv: List[Optional[int]]
    temp_cc: List[Optional[int]]
    charge: bool
    discharge: bool
    armed: bool
    esr_mask: int
    leak_mask: int
    heat: List[int]
    trip: int
    voltage_quality: int
    temp_quality_mask: int

    @property
    def temp_quality(self) -> List[str]:
        return [TEMP_QUALITY[(self.temp_quality_mask >> (2 * i)) & 3] for i in range(4)]

    @property
    def voltage_quality_flags(self) -> List[str]:
        return [name for name, bit in VQ_BITS.items() if self.voltage_quality & bit]

    @property
    def voltage_quality_enum(self) -> str:
        vq = self.voltage_quality
        for name, enum in (("AREF_INVALID", "AREF_INVALID"), ("ADC_SATURATED", "ADC_SATURATED"),
                           ("TAP_INVERSION", "TAP_INVERSION"), ("STALE", "MISSING")):
            if vq & VQ_BITS[name]:
                return enum
        return "VALID"


@dataclasses.dataclass
class Status:
    seq: int
    millis: int
    state: str
    trip: int
    relay: bool
    lock_mask: int
    hb_age_ms: int
    last_cmd_seq: int
    rx_ok: int
    rx_crc_err: int
    rx_format_err: int
    rx_overflow: int
    reset_flags: int
    loop_overruns: int
    free_ram: int
    bandgap_raw: int

    @property
    def locks(self) -> List[str]:
        return [name for name, bit in LOCK_BITS.items() if self.lock_mask >> bit & 1]


@dataclasses.dataclass
class RawAdc:
    seq: int
    raw: List[int]  # A0..A7


@dataclasses.dataclass
class Event:
    event_seq: int
    millis: int
    code: str
    cell_id: int
    requested: int
    commanded: int
    reason: str


@dataclasses.dataclass
class Version:
    firmware: str
    protocol: str
    map_version: str
    flags: int

    def flag(self, name: str) -> bool:
        return bool(self.flags >> CGV_FLAG_BITS[name] & 1)


@dataclasses.dataclass
class CalibrationInfo:
    seq: int
    source: str
    cal_id: str
    aref_mv: int
    v5_mv: int
    gain_ppm: List[int]
    offset_mv: List[int]
    ntc_b: int
    ntc_r25: int


Message = Union[Ack, Telemetry, Status, RawAdc, Event, Version, CalibrationInfo]
FIELD_COUNTS = {"CGA": 4, "CGT": 23, "CGS": 17, "CGD": 10, "CGE": 8, "CGV": 5, "CGK": 16}


def _int(text: str) -> int:
    if not text or not (text.isdigit() or (text[0] == "-" and text[1:].isdigit())):
        raise FrameError(f"not an integer: {text!r}")
    return int(text)


def _opt(value: int) -> Optional[int]:
    return None if value == INVALID_I16 else value


def parse_message(payload: str) -> Message:
    f = payload.split(",")
    kind = f[0]
    if kind not in FIELD_COUNTS:
        raise FrameError(f"unknown frame type {kind!r}")
    if len(f) != FIELD_COUNTS[kind]:
        raise FrameError(f"{kind} expects {FIELD_COUNTS[kind]} fields, got {len(f)}")
    if kind == "CGA":
        if f[2] not in ("OK", "ERR"):
            raise FrameError("CGA status must be OK or ERR")
        return Ack(_int(f[1]), f[2] == "OK", f[3])
    if kind == "CGT":
        n = [_int(x) for x in f[1:]]
        tq = n[21]
        temps = [_opt(n[6 + i]) if ((tq >> (2 * i)) & 3) == 0 else None for i in range(4)]
        return Telemetry(seq=n[0], millis=n[1], cell_mv=[_opt(v) for v in n[2:6]], temp_cc=temps,
                         charge=bool(n[10]), discharge=bool(n[11]), armed=bool(n[12]), esr_mask=n[13],
                         leak_mask=n[14], heat=n[15:19], trip=n[19], voltage_quality=n[20], temp_quality_mask=tq)
    if kind == "CGS":
        if f[3] not in STATE_NAMES:
            raise FrameError(f"unknown state {f[3]!r}")
        n = [_int(x) for x in f[4:]]
        return Status(_int(f[1]), _int(f[2]), f[3], n[0], bool(n[1]), n[2], n[3], n[4], n[5], n[6], n[7], n[8],
                      n[9], n[10], n[11], n[12])
    if kind == "CGD":
        return RawAdc(_int(f[1]), [_int(x) for x in f[2:]])
    if kind == "CGE":
        return Event(_int(f[1]), _int(f[2]), f[3], _int(f[4]), _int(f[5]), _int(f[6]), f[7])
    if kind == "CGV":
        return Version(f[1], f[2], f[3], _int(f[4]))
    return CalibrationInfo(_int(f[1]), f[2], f[3], _int(f[4]), _int(f[5]), [_int(x) for x in f[6:10]],
                           [_int(x) for x in f[10:14]], _int(f[14]), _int(f[15]))
