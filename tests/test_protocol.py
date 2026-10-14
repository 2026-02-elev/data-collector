# Author: Yongho Ahn <nanhosoft@gmail.com>
import pytest

from cg_collector import protocol as p


def test_crc_known_vector():
    assert p.crc16_ccitt_false(b"123456789") == 0x29B1


def test_encode_decode_roundtrip():
    frame = p.encode_frame("CGC,1,PING")
    assert frame.startswith(b"$CGC,1,PING*") and frame.endswith(b"\n")
    assert p.decode_line(frame) == "CGC,1,PING"
    assert p.decode_line(frame[:-1] + b"\r\n") == "CGC,1,PING"


@pytest.mark.parametrize("bad", ["$CGC,1,PING*0000", "CGC,1,PING*1234", "$CGC,1,PING", "$CGC,1*,PING*1234"])
def test_decode_rejects(bad):
    with pytest.raises(p.FrameError):
        p.decode_line(bad)


def test_length_limits():
    long_payload = "CGT," + "9" * 190
    crc = p.crc16_ccitt_false(long_payload.encode())
    with pytest.raises(p.FrameError):
        p.decode_line(f"${long_payload}*{crc:04X}")
    with pytest.raises(ValueError):
        p.encode_frame("CGC,1," + "X" * 190)


def test_strict_field_counts():
    for bad in ("CGA,1,OK", "CGT,1,2,3", "CGX,1", "CGA,1,MAYBE,NONE"):
        with pytest.raises(p.FrameError):
            p.parse_message(bad)


def test_telemetry_decoding():
    t = p.parse_message("CGT,7,7000,1201,1199,-32768,1203,-32768,4012,-32768,-32768,0,1,1,2,0,0,0,0,0,0,3,81")
    assert t.cell_mv == [1201, 1199, None, 1203]
    assert t.temp_cc == [None, 4012, None, None]
    assert t.temp_quality == ["SATURATED_OR_OPEN", "VALID", "SATURATED_OR_OPEN", "SATURATED_OR_OPEN"]
    assert t.voltage_quality_flags == ["ADC_SATURATED", "TAP_INVERSION"]
    assert t.voltage_quality_enum == "ADC_SATURATED"


def test_status_and_new_trip_codes():
    s = p.parse_message("CGS,3,3000,LOCKED,15,0,79,999999,-1,0,0,0,0,1,0,0,450")
    assert s.locks == ["POWER_LOCKED", "ESR_LOCKED", "LEAK_LOCKED", "HEAT_LOCKED", "TEMP_SENSOR_FAULT"]
    assert p.TRIP_NAMES[13] == "CHARGE_NO_RESPONSE" and p.TRIP_NAMES[15] == "TEMP_SENSOR_FAULT"
    v = p.parse_message("CGV,0.1.0,1.0,PCB2-20260918-R2,512")
    assert v.flag("HOST_SIM_BUILD") and not v.flag("MAP_VERIFIED")
