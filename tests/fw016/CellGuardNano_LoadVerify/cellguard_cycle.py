#!/usr/bin/env python3
"""Supervised CellGuard commissioning: stepped charge + rested discharge test.

No pip packages required; Linux/Raspberry Pi only. Open /dev/serial0 at
115200, keep a 1-second heartbeat, require signed-off voltage at every stage,
and always send OFF on failure/interrupt. Do NOT run unattended.

  sudo python3 cellguard_cycle.py --charge-to 200 --discharge-to 120
  sudo python3 cellguard_cycle.py --charge-to 400 --step 100 --discharge-to 200

All voltages in the command are RESTED millivolts PER CELL, not pack voltage.
"""

import argparse
import binascii
import datetime as dt
import os
import select
import sys
import termios
import time
import tty

VERSION = "0.1.6-LOAD-VERIFY"
HEARTBEAT_SECONDS = 1.0
CHARGE_TIMEOUT_SECONDS = 32.0  # Nano has a separate hard cap of 25 s
DRAIN_TIMEOUT_SECONDS = 37.0   # Nano has a separate hard cap of 30 s
SETTLE_SECONDS = 2.3
CHARGE_MARGIN_MV = 40
DRAIN_MARGIN_MV = 40
MAX_SPREAD_MV = 80


class Aborted(RuntimeError):
    pass


def frame(command, seq):
    payload = f"CGC,{seq},{command}".encode("ascii")
    checksum = binascii.crc_hqx(payload, 0xFFFF)
    return b"$" + payload + b"*" + f"{checksum:04X}".encode() + b"\n"


def parse_line(line):
    """Returns verified payload fields or None (invalid CRC / malformed)."""
    if not line.startswith(b"$") or len(line) > 192:
        return None
    body, sep, crc = line[1:].rpartition(b"*")
    if not sep or len(crc) != 4:
        return None
    try:
        expected = int(crc, 16)
        if crc != f"{expected:04X}".encode():
            return None
        if binascii.crc_hqx(body, 0xFFFF) != expected:
            return None
        return body.decode("ascii").split(",")
    except (ValueError, UnicodeDecodeError):
        return None


def checkpoints(current_max, target, step):
    """Choose future per-cell setpoints without repeating earlier levels."""
    if current_max >= target:
        return []
    first = max(150, ((current_max // step) + 1) * step)
    if first - current_max < 16:
        first += step
    levels = list(range(first, target + 1, step))
    if not levels or levels[-1] != target:
        levels.append(target)
    return levels


class Link:
    def __init__(self, port, log_path):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.original_termios = termios.tcgetattr(self.fd)
        tty.setraw(self.fd, termios.TCSANOW)
        settings = termios.tcgetattr(self.fd)
        settings[4] = termios.B115200
        settings[5] = termios.B115200
        settings[2] |= termios.CREAD | termios.CLOCAL
        termios.tcsetattr(self.fd, termios.TCSANOW, settings)
        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.log = open(log_path, "a", encoding="utf-8", buffering=1)
        self.buffer = b""
        self.seq = 0
        self.acks = {}
        self.events = []
        self.last_cgt = None
        self.last_cgt_at = 0
        self.state = None
        self.relay = None
        self.trip = 0
        self.version = None
        self.bad_frames = 0
        self.last_ping_at = time.monotonic()

    def close(self):
        try:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.original_termios)
        finally:
            os.close(self.fd)
            self.log.close()

    def note(self, line):
        stamp = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self.log.write(f"{stamp} {line}\n")

    def send(self, command):
        self.seq += 1
        if self.seq > 65000:
            raise Aborted("Sequence budget exceeded. End session safely.")
        data = frame(command, self.seq)
        os.write(self.fd, data)
        self.note("TX " + data.decode("ascii").strip())
        return self.seq

    def poll(self, seconds=0.2):
        readable, _, _ = select.select([self.fd], [], [], seconds)
        if not readable:
            return
        try:
            data = os.read(self.fd, 4096)
        except BlockingIOError:
            return
        self.buffer += data
        if len(self.buffer) > 4096 and b"\n" not in self.buffer:
            self.buffer = b""
            raise Aborted("Serial framing lost (oversized incomplete stream)")
        while b"\n" in self.buffer:
            raw, self.buffer = self.buffer.split(b"\n", 1)
            raw = raw.rstrip(b"\r")
            self.note("RX " + raw.decode("ascii", errors="replace"))
            fields = parse_line(raw)
            if fields is None:
                self.bad_frames += 1
                if self.bad_frames > 3:
                    raise Aborted("Repeated malformed/CRC-invalid Nano telemetry")
                continue
            name = fields[0]
            try:
                if name == "CGA" and len(fields) >= 4:
                    self.acks[int(fields[1])] = (fields[2], fields[3])
                elif name == "CGV" and len(fields) >= 2:
                    self.version = fields[1]
                elif name == "CGY" and len(fields) >= 12:
                    # Live suspect snapshot is diagnostic, NOT a resting CGT.
                    loaded_taps = [int(x) for x in fields[7:11]]
                    print(f"    ON-LOAD SUSPECT: taps {loaded_taps}mV; MOS OFF, verifying at rest", flush=True)
                elif name == "CGT" and len(fields) >= 7:
                    self.last_cgt = tuple(int(x) for x in fields[3:7])
                    self.last_cgt_at = time.monotonic()
                elif name == "CGS" and len(fields) >= 6:
                    self.state = fields[3]
                    self.trip = int(fields[4])
                    self.relay = int(fields[5])
                elif name == "CGE" and len(fields) >= 8:
                    self.events.append((fields[3], fields[7], fields))
                    if len(self.events) > 500:
                        self.events.pop(0)
                    if fields[3] in ("TRIP", "CHARGE_STOP", "DRAIN_STOP", "CHECKPOINT",
                                     "LOAD_TAP_SUSPECT", "LOAD_TAP_VERIFY"):
                        print(f"    {fields[3]}: {fields[7]}", flush=True)
            except (ValueError, IndexError) as err:
                raise Aborted("Malformed Nano data: " + repr(fields)) from err

    def command(self, name, timeout=3.0):
        seq = self.send(name)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.poll(0.1)
            if seq in self.acks:
                status, detail = self.acks.pop(seq)
                if status != "OK":
                    raise Aborted(f"Nano rejected {name}: {detail}")
                return detail
            if self.state == "TRIPPED" or self.trip:
                raise Aborted(f"Nano latched trip code {self.trip}")
        raise Aborted(f"No ACK for {name} after {timeout:g}s")

    def keepalive(self):
        if time.monotonic() - self.last_ping_at >= HEARTBEAT_SECONDS:
            self.send("PING")
            self.last_ping_at = time.monotonic()

    def wait_idle(self, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.poll(0.15)
            if self.state == "TRIPPED" or self.trip:
                raise Aborted(f"Nano trip {self.trip}, re-arm prohibited")
            if self.state == "READY" and self.relay == 0:
                return
        raise Aborted(f"Nano not idle/READY (state={self.state}, relay={self.relay})")

    def settle_readings(self, since):
        """Require fresh measurements after relay OFF and after 2.3s rest."""
        end = time.monotonic() + max(SETTLE_SECONDS + 1.2, 4.0)
        last = None
        samples = []
        while time.monotonic() < end:
            self.poll(0.15)
            if self.state == "TRIPPED" or self.trip:
                raise Aborted(f"Nano trip {self.trip} while settling")
            if self.relay == 0 and self.last_cgt_at > since and self.last_cgt_at != last:
                last = self.last_cgt_at
                samples.append(self.last_cgt)
            if time.monotonic() - since >= SETTLE_SECONDS and len(samples) >= 2:
                # Last reported rested set is the one used for the next stage.
                return samples[-1]
        raise Aborted("No fresh, repeatable rested CGT voltage after relay OFF")


def verify_voltage(kind, target, before, after):
    lo, hi = min(after), max(after)
    spread = hi - lo
    if spread > MAX_SPREAD_MV:
        raise Aborted(f"Cell spread {spread}mV exceeds {MAX_SPREAD_MV}mV")
    if kind == "RAMP":
        if lo < target - CHARGE_MARGIN_MV or hi > target + CHARGE_MARGIN_MV:
            raise Aborted(f"Charging checkpoint {target}mV was not achieved: {after}")
        if not all(a >= b for a, b in zip(after, before)):
            raise Aborted(f"Some cell fell rather than charged: {before} => {after}")
    else:
        if lo < target - DRAIN_MARGIN_MV or hi > target + DRAIN_MARGIN_MV:
            raise Aborted(f"Discharge checkpoint {target}mV was not achieved: {after}")
        if not all(a <= b for a, b in zip(after, before)):
            raise Aborted(f"Some cell rose rather than discharged: {before} => {after}")
        if not any(a < b - 16 for a, b in zip(after, before)):
            raise Aborted("Discharge did not measurably reduce the stored voltage")


def run_stage(link, kind, target, before):
    link.wait_idle()
    print(f"[{kind}] target {target}mV/cell | start {list(before)}", flush=True)
    link.command("ARM")
    link.command(f"{kind},{target}")
    marker = len(link.events)
    stop_name = "CHARGE_STOP" if kind == "RAMP" else "DRAIN_STOP"
    success_reason = "TARGET" if kind == "RAMP" else "DRAIN_TARGET"
    timeout = CHARGE_TIMEOUT_SECONDS if kind == "RAMP" else DRAIN_TIMEOUT_SECONDS
    deadline = time.monotonic() + timeout
    stop_seen = False
    relay_off_at = None
    while time.monotonic() < deadline:
        link.keepalive()
        link.poll(0.15)
        if link.state == "TRIPPED" or link.trip:
            raise Aborted(f"Nano TRIPPED code={link.trip}")
        new = link.events[marker:]
        marker = len(link.events)
        for event_name, reason, fields in new:
            if event_name == "TRIP":
                raise Aborted("Nano hardware trip: " + reason)
            if event_name == stop_name:
                stop_seen = True
                if reason != success_reason:
                    raise Aborted(f"{kind} stopped before goal: {reason}")
            if event_name == "RELAY_OFF" and stop_seen:
                relay_off_at = time.monotonic()
        if stop_seen and link.relay == 0 and relay_off_at is not None:
            after = link.settle_readings(relay_off_at)
            link.wait_idle()
            verify_voltage(kind, target, before, after)
            print(f"    PASS {list(after)}mV | spread {max(after)-min(after)}mV", flush=True)
            return after
    raise Aborted(f"{kind} {target}mV timed out without a clean stop")


def run(args):
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = args.log or f"cellguard_cycle_{stamp}.log"
    link = Link(args.port, log_path)
    print(f"Serial trace: {os.path.abspath(log_path)}", flush=True)
    try:
        # Wait for at least one live telemetry sample, then actively synchronise.
        start = time.monotonic()
        while link.last_cgt is None and time.monotonic() - start < 3.0:
            link.poll(0.2)
        if link.last_cgt is None:
            raise Aborted("No CGT telemetry from Nano. Check device/baud, not wiring.")
        link.command("SYNC")
        # The SYNC ACK can arrive before the following CGV frame has finished
        # transmitting. poll() returns as soon as *any* bytes arrive, so a
        # single poll(0.2) is not a guaranteed 200 ms version wait.
        version_deadline = time.monotonic() + 2.0
        while link.version is None and time.monotonic() < version_deadline:
            link.poll(0.1)
        if link.version != VERSION:
            raise Aborted(f"Wrong firmware: {link.version!r} (expected {VERSION})")
        if args.probe_only:
            # Non-energizing check after a firmware upload. SYNC itself
            # commands the Nano safe; no ARM/RAMP/DRAIN is issued.
            status_deadline = time.monotonic() + 2.0
            while link.relay is None and time.monotonic() < status_deadline:
                link.poll(0.1)
            if link.relay != 0:
                raise Aborted(f"Probe saw relay={link.relay}; outputs must be OFF")
            print(f"PROBE: firmware={link.version}, state={link.state}, trip={link.trip}, "
                  f"relay={link.relay}, cells={list(link.last_cgt)}mV", flush=True)
            return
        link.wait_idle()
        initial = link.last_cgt
        if min(initial) < -20 or max(initial) - min(initial) > MAX_SPREAD_MV:
            raise Aborted(f"Initial cell voltages unsafe for staging: {initial}")
        levels = checkpoints(max(initial), args.charge_to, args.step)
        if not levels:
            raise Aborted("Target already at/below highest cell; nothing to charge")
        print(f"Charge plan: {' -> '.join(map(str, levels))} mV per cell", flush=True)
        volts = initial
        for target in levels:
            volts = run_stage(link, "RAMP", target, volts)
        if args.discharge_to is not None:
            if min(volts) <= args.discharge_to:
                raise Aborted("Discharge target must be BELOW all measured cells")
            volts = run_stage(link, "DRAIN", args.discharge_to, volts)
        print(f"PASS: charge + discharge cycle complete. Final cells {list(volts)}mV")
        print(f"Full trace: {os.path.abspath(log_path)}")
    except BaseException:
        try:
            link.send("OFF")
            link.poll(0.3)
            print("Fail-safe: OFF sent to Nano", flush=True)
        except Exception as err:
            print(f"Could not confirm OFF ({err}); Nano heartbeat timer will trip.", file=sys.stderr)
        raise
    finally:
        link.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--charge-to", type=int, metavar="MV_PER_CELL")
    parser.add_argument("--discharge-to", type=int, metavar="MV_PER_CELL")
    parser.add_argument("--step", type=int, default=100, metavar="MV")
    parser.add_argument("--probe-only", action="store_true", help="verify Nano version and read voltages; no ARM or MOS operation")
    parser.add_argument("--log", help="path for timestamped raw UART trace")
    args = parser.parse_args()
    if args.probe_only:
        if args.charge_to is not None or args.discharge_to is not None:
            parser.error("--probe-only does not accept charge or discharge targets")
    else:
        if args.charge_to is None or not 150 <= args.charge_to <= 1000:
            parser.error("--charge-to must be in 150..1000 mV per cell")
    if args.discharge_to is not None and not 100 <= args.discharge_to <= 950:
        parser.error("--discharge-to must be in 100..950 mV per cell")
    if not 50 <= args.step <= 200:
        parser.error("--step must be 50..200 mV")
    if args.discharge_to is not None and args.discharge_to >= args.charge_to:
        parser.error("--discharge-to must be lower than --charge-to")
    try:
        run(args)
    except Aborted as exc:
        print("ABORTED:", exc, file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Interrupted. Nano has been sent OFF.", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
