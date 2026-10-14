# CellGuard data collector - byte transports (real UART, firmware host simulator)
# Author: Yongho Ahn <nanhosoft@gmail.com>
from __future__ import annotations

import datetime as dt
import subprocess
import time
from typing import List

UTC = dt.timezone.utc


class Transport:
    """Byte transport plus the clock used to timestamp received frames."""

    def write(self, data: bytes) -> None:
        raise NotImplementedError

    def read_lines(self, timeout_s: float) -> List[str]:
        """Returns complete lines received within ``timeout_s`` (may return early)."""
        raise NotImplementedError

    def now_utc(self) -> dt.datetime:
        return dt.datetime.now(UTC)

    def close(self) -> None:
        pass


class SerialTransport(Transport):
    """Real UART (/dev/serial0, /dev/ttyUSB0 ...) via pyserial, 115200 8N1."""

    def __init__(self, port: str, baud: int = 115200):
        import serial  # lazy: tests never open a real port

        self._ser = serial.Serial(port, baud, bytesize=8, parity="N", stopbits=1, timeout=0.02)
        self._buf = b""

    def write(self, data: bytes) -> None:
        self._ser.write(data)
        self._ser.flush()

    def read_lines(self, timeout_s: float) -> List[str]:
        deadline = time.monotonic() + timeout_s
        lines: List[str] = []
        while True:
            chunk = self._ser.read(max(1, self._ser.in_waiting))
            self._buf += chunk
            while b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                lines.append(line.decode("ascii", "replace"))
            if len(self._buf) > 4096:  # garbage without newline: drop, the parser counts errors
                self._buf = b""
            if lines or time.monotonic() >= deadline:
                return lines

    def close(self) -> None:
        self._ser.close()


class SimTransport(Transport):
    """Talks to the firmware host simulator (``arudino-firmware/tests/host/build/nano_host_sim``).

    SIMULATED DEVICE with a simulated clock: waiting ``timeout_s`` with nothing
    pending advances the simulator by that amount. Its CGV flags carry
    HOST_SIM_BUILD, so its data is never labelled as measured.
    """

    EPOCH = dt.datetime(2026, 10, 14, 0, 0, 0, tzinfo=UTC)

    def __init__(self, sim_path: str, gates: str = "locked", reset: str = "por"):
        self._proc = subprocess.Popen([sim_path, "--gates", gates, "--reset", reset], stdin=subprocess.PIPE,
                                      stdout=subprocess.PIPE, text=True, bufsize=1)
        self.sim_ms = 0
        self._pending = self._until_done()

    def _until_done(self) -> List[str]:
        out = []
        assert self._proc.stdout is not None
        for line in self._proc.stdout:
            line = line.rstrip("\n")
            if line == "#DONE":
                return out
            if line.startswith("#ERROR"):
                raise RuntimeError(line)
            out.append(line)
        raise RuntimeError("simulator exited")

    def control(self, line: str) -> List[str]:
        """Sends a simulator control line (ADV/CELLS/NTC/...); frames are queued for read_lines()."""
        assert self._proc.stdin is not None
        self._proc.stdin.write(line + "\n")
        self._proc.stdin.flush()
        if line.startswith("ADV "):
            self.sim_ms += int(line.split()[1])
        out = self._until_done()
        self._pending.extend(out)
        return out

    def write(self, data: bytes) -> None:
        for line in data.decode("ascii").split("\n"):
            if line:
                self.control("TXRAW " + line)

    def read_lines(self, timeout_s: float) -> List[str]:
        if not self._pending:
            self.control(f"ADV {max(10, int(round(timeout_s * 1000)))}")
        lines, self._pending = self._pending, []
        return lines

    def now_utc(self) -> dt.datetime:
        return self.EPOCH + dt.timedelta(milliseconds=self.sim_ms)

    def close(self) -> None:
        if self._proc.poll() is None:
            assert self._proc.stdin is not None
            self._proc.stdin.write("QUIT\n")
            self._proc.stdin.flush()
            self._proc.wait(timeout=5)
