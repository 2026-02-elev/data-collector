# CellGuard data collector - pack current (INA219 on the Pi, never the Nano)
# Author: Yongho Ahn <nanhosoft@gmail.com>
"""Pack current is read by the Raspberry Pi over I2C (PCB2 H9_1), independently
of the Nano. Hardware preconditions (see docs/README):

* The INA219 module is not in the purchase list - it must be sourced.
* I2C alone is not enough: the shunt (VIN+/VIN-) goes in series through
  PCB2 H4_1 (pin 1 = pack/relay side, pin 2 = charge/discharge resistor side).
* Shunt value, install direction and sign must be measured before trusting it.

Sign contract: + = charging the pack, - = discharging.

Time alignment: a background :class:`CurrentSampler` reads the sensor at a
fixed rate and keeps a short history. Fusion picks the newest sample taken
*at or before* the moment a telemetry frame was received (never a later
sample) and rejects samples older than ``max_age_s``. Unmeasured current is
``None`` with a quality code - never 0 A.
"""
from __future__ import annotations

import bisect
import collections
import dataclasses
import datetime as dt
import threading
from typing import Callable, Deque, List, Optional

UTC = dt.timezone.utc


@dataclasses.dataclass
class CurrentReading:
    current_a: Optional[float]   # sign-corrected (+ = charge); None unless VALID
    quality: str  # VALID | INA219_UNAVAILABLE | I2C_ERROR | MISSING | OUT_OF_RANGE
    sampled_utc: Optional[dt.datetime]
    available_utc: Optional[dt.datetime] = None  # when the value was known to the Pi (end of the I2C read)
    raw: Optional[dict] = None                   # driver-level evidence (register value, shunt, sign)

    def to_raw_record(self) -> dict:
        """ina219_raw_v1 line (same format as the data-generator's ina219_raw.jsonl)."""
        def us(t: Optional[dt.datetime]) -> Optional[str]:
            return None if t is None else t.astimezone(UTC).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
        return {"schema_version": "ina219_raw_v1", "sampled_utc": us(self.sampled_utc),
                "available_utc": us(self.available_utc or self.sampled_utc),
                "current_a": self.current_a if self.quality == "VALID" else None, "quality": self.quality,
                "raw": self.raw}

    @staticmethod
    def from_raw_record(r: dict) -> "CurrentReading":
        def ts(x: Optional[str]) -> Optional[dt.datetime]:
            return None if x is None else dt.datetime.fromisoformat(x.replace("Z", "+00:00")).astimezone(UTC)
        return CurrentReading(r.get("current_a"), r["quality"], ts(r.get("sampled_utc")),
                              ts(r.get("available_utc")), r.get("raw"))


class CurrentSource:
    def read(self) -> CurrentReading:
        raise NotImplementedError


class NoCurrentSensor(CurrentSource):
    """No INA219 installed (current default hardware state)."""

    def read(self) -> CurrentReading:
        return CurrentReading(None, "INA219_UNAVAILABLE", None)


class Ina219(CurrentSource):
    REG_CONFIG = 0x00
    REG_SHUNT_VOLTAGE = 0x01

    def __init__(self, bus: int = 1, address: int = 0x40, shunt_ohm: float = 0.1, sign: int = 1,
                 max_abs_a: float = 1.0, clock: Callable[[], dt.datetime] = lambda: dt.datetime.now(UTC)):
        from smbus2 import SMBus  # lazy import: only on the Pi

        self._bus = SMBus(bus)
        self.address = address
        self.shunt_ohm = shunt_ohm  # MEASURE the module's shunt; 0.1 R is only a common default
        self.sign = 1 if sign >= 0 else -1
        self.max_abs_a = max_abs_a
        self.clock = clock
        # 32 V range, PGA /8 (+-320 mV), 12-bit, continuous shunt + bus.
        self._bus.write_i2c_block_data(self.address, self.REG_CONFIG, [0x39, 0x9F])

    def read(self) -> CurrentReading:
        t0 = self.clock()
        try:
            hi, lo = self._bus.read_i2c_block_data(self.address, self.REG_SHUNT_VOLTAGE, 2)
        except OSError as exc:
            return CurrentReading(None, "I2C_ERROR", t0, self.clock(), {"error": str(exc)})
        t1 = self.clock()
        sampled = t0 + (t1 - t0) / 2  # middle of the I2C transaction
        reg = (hi << 8) | lo
        signed = reg - 0x10000 if reg & 0x8000 else reg
        raw = {"shunt_register": reg, "shunt_lsb_v": 10e-6, "shunt_ohm": self.shunt_ohm, "sign_correction": self.sign}
        current = round(self.sign * signed * 10e-6 / self.shunt_ohm, 6)
        if abs(current) > self.max_abs_a:
            return CurrentReading(None, "OUT_OF_RANGE", sampled, t1, raw)
        return CurrentReading(current, "VALID", sampled, t1, raw)


class CurrentSampler:
    """Polls a source at ``period_s`` and answers "what was the current at time t"."""

    def __init__(self, source: Optional[CurrentSource], clock: Callable[[], dt.datetime] = lambda: dt.datetime.now(UTC),
                 period_s: float = 0.1, history: int = 1500, max_age_s: float = 0.5):
        self.source = source
        self.clock = clock
        self.period_s = period_s
        self.max_age_s = max_age_s
        self._hist: Deque[CurrentReading] = collections.deque(maxlen=history)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._listeners: List[Callable[[CurrentReading], None]] = []

    def on_sample(self, fn: Callable[[CurrentReading], None]) -> None:
        """Every reading (the raw INA219 log); called from the sampler thread."""
        self._listeners.append(fn)

    def sample_once(self) -> CurrentReading:
        assert self.source is not None
        r = self.source.read()
        if r.sampled_utc is None:
            now = self.clock()
            r = dataclasses.replace(r, sampled_utc=now, available_utc=r.available_utc or now)
        self.feed(r)
        for fn in self._listeners:
            fn(r)
        return r

    def feed(self, r: CurrentReading) -> None:
        """Adds a reading in time order (live sampling, or a raw log during replay)."""
        if r.available_utc is None:
            r = dataclasses.replace(r, available_utc=r.sampled_utc)
        with self._lock:
            if self._hist and r.sampled_utc < self._hist[-1].sampled_utc:  # type: ignore[operator]
                self._hist.clear()  # clock went backwards: drop history rather than mis-order
            self._hist.append(r)

    def start(self) -> None:
        def run() -> None:
            while not self._stop.wait(self.period_s):
                self.sample_once()

        self.sample_once()
        self._thread = threading.Thread(target=run, name="ina219-sampler", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)

    def reading_at(self, utc: dt.datetime, available_by: Optional[dt.datetime] = None,
                   age_ref: Optional[dt.datetime] = None) -> CurrentReading:
        """Newest reading sampled at or before ``utc`` (and already available at ``available_by``).

        A VALID reading older than ``max_age_s`` (measured from ``age_ref``, default ``utc``)
        becomes MISSING. Same selection rule as the data-generator's virtual Pi.
        """
        with self._lock:
            hist = list(self._hist)
        ref = age_ref or utc
        times = [r.sampled_utc for r in hist]
        i = bisect.bisect_right(times, utc)  # samples at or before utc only
        while i > 0 and available_by is not None and hist[i - 1].available_utc > available_by:  # type: ignore[operator]
            i -= 1
        if i == 0:
            return CurrentReading(None, "MISSING", None)
        r = hist[i - 1]
        if r.quality == "VALID" and (ref - r.sampled_utc).total_seconds() > self.max_age_s:  # type: ignore[operator]
            return CurrentReading(None, "MISSING", r.sampled_utc, r.available_utc)
        return r
