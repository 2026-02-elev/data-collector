#!/usr/bin/env python3
"""CellGuard edge (Raspberry Pi <-> Nano 0.2.0-RUNTIME / 0.1.9, MQTT over TLS, on-device AI with OTA).

Modes (control.json "mode", CLI `mode`, or MQTT cellguard/v1/{device}/command)
- collect : systematic cycle (charge -> stimulus -> balance -> drain), stimuli rotate over cells
- fault   : random episodes - none / single / compound faults (2-3 kinds). Nano >= 0.2.0
            runs them at the same time (HEAT + LEAK on, ESR load pulses during the overlap);
            0.1.9 allows one actuator at a time, so there they run back to back
- demo    : presentation (시연). The pack runs like normal operation (charge, then hold and watch);
            after a random 30..90 s ONE random fault is injected without announcement. The first
            confirmed AI detection sends TRIP: the Nano cuts everything at once (HOST_TRIP), the server
            gets diag AI_CUTOFF and a TRIPPED fault event naming the fault, cell and model. After
            demo_recover_s the trip is reset and the next show starts.
- passive : telemetry only, outputs off
In every mode the AI models (MLOps residual / classifier / load_esr) judge each sample on the Pi.
A confirmed anomaly is reported to the server (diag AI_DETECTION + fault_event OBSERVABLE) with
the ground truth the Pi itself commanded, so the server can score the detector.
Outside demo mode the AI is diagnostic only. In demo mode it may only make things safer: TRIP turns
every output off; it never switches anything on or relaxes a firmware limit.
Models are pulled from MLOps (OTA): production version changed -> download, checksum, test vectors
-> install, keep the previous one, roll back if the model cannot be loaded.

Design rules
- The Nano firmware is the safety authority. The Pi never weakens or bypasses it.
- The Pi stays far from the firmware trip limits (spread 80 mV, no-response checks)
  and judges spread on a median of several samples (ADC step ~8.4 mV).
- Nothing recoverable stops collection. Noise, slow balancing, a CRC error, a lost
  ACK or a firmware trip only skips or retries a step; telemetry keeps flowing.
- Repeated trips -> outputs off, passive telemetry only, cool down, then retry.
- A charge path that gives no voltage rise (open path: H4, RLY2, FH1, cable) is not retried
  every minute: after 2 such trips experiments pause (30 min, doubling to 4 h)
  with a CHARGE_PATH_NO_RESPONSE diagnostic. `resume` retries immediately.

Usage
  cellguard_edge.py run [--passive]           # service entry (systemd --user)
  cellguard_edge.py status                    # current status (status.json)
  cellguard_edge.py mode collect|fault|demo|passive|hold|discharge
  cellguard_edge.py pause | resume            # passive / back to the mode before pause
  cellguard_edge.py ota                       # check MLOps for new models now
  cellguard_edge.py ota-install NAME ZIP      # install a model package file (offline)
"""
from __future__ import annotations

import argparse
import binascii
import collections
import datetime as dt
import fcntl
import gzip
import json
import os
import queue
import random
import select
import shutil
import signal
import socket
import sqlite3
import statistics
import sys
import termios
import threading
import time
import traceback
from pathlib import Path

VERSION = "3.0.0"
HOME = Path(__file__).resolve().parent
sys.path.insert(0, str(HOME))
MODES = ("collect", "fault", "demo", "passive", "hold", "discharge")
DEFAULT_CONFIG = {
    "device_id": "cellguard-pi-01",
    "pack_id": "pack-01",
    "hardware_revision": "PCB2-20260918-R2",
    "serial_port": "/dev/serial0",
    "data_dir": str(HOME / "data"),
    "mqtt_host": "mqtt.misakachan.moe",
    "mqtt_port": 8883,
    "mqtt_ca": str(HOME / "ca.crt"),
    "mqtt_secrets": str(HOME / "secrets.env"),
    "telemetry_every_n": 1,           # publish every Nth CGT (1 = 1 Hz)
    "outbox_max_rows": 1_000_000,     # ~11 days offline at 1 Hz
    "raw_log_days": 30,
    "run_rotate_hours": 24,
    # experiment plan
    "charge_targets_mv": [600, 700, 800],
    "rest_mv": 350,
    "stimuli": ["control", "esr", "leak", "heat"],
    "baseline_s": 20, "hold_s": 20, "post_s": 30, "rest_s": 30,
    # Pi-side margins (firmware: spread trip 80 mV)
    "start_spread_max_mv": 48,        # filtered spread allowed to start charge/drain/stimulus
    "guard_raw_spread_mv": 70,        # one raw sample at/over this -> OFF
    "guard_filtered_spread_mv": 60,   # median spread at/over this -> OFF
    "balance_start_mv": 40, "balance_target_mv": 28, "balance_max_s": 360,
    "drain_min_cell_mv": 430,         # never DRAIN from below this (firmware no-response check)
    "heat_start_max_c": 30.0, "heat_guard_c": 38.0, "heat_cooldown_max_s": 300,
    # trip policy
    "trip_wait_s": 30, "trips_per_hour_pause": 3, "trip_pause_s": 3600,
    "charger_fail_pause_after": 2, "charger_retry_s": 1800, "charger_retry_max_s": 14400,
    # load pulse (ESR is only visible under load: ESR on -> settle -> DRAIN 1 s -> OFF, gives CGP samples)
    "pulse_s": 1.0, "pulse_settle_s": 0.35, "pulse_min_cell_mv": 400, "esr_pulses": 3, "control_pulses": 2,
    # fault mode (random episodes)
    "fault_weights": {"none": 0.15, "single": 0.45, "compound": 0.40},
    "fault_kinds": ["esr", "leak", "heat"],
    "fault_triple_p": 0.25,           # compound: chance of 3 kinds instead of 2
    "fault_same_cell_p": 0.5,         # compound: all on one cell vs. random cells
    "fault_overlap": True,            # compound on Nano >= 0.2.0: faults at the same time (False: back to back)
    "fault_partial_min": 0.5,         # leak/heat stopped after a random 50..100 % of the firmware limit
    "episode_baseline_s": 30, "episode_hold_s": 20, "episode_post_s": 45, "episode_rest_s": 30,
    # demo mode (시연): normal-looking run, one hidden random fault, AI detection -> TRIP
    "demo_fault_kinds": ["heat", "leak", "esr"],
    "demo_fault_weights": {"heat": 1.0, "leak": 1.0, "esr": 1.0},
    "demo_normal_min_s": 30, "demo_normal_max_s": 90,   # normal operation before the injection
    "demo_target_mv": 700,
    "demo_esr_pulses": 5,             # ESR only shows under load: up to this many load pulses
    "demo_watch_s": 30,               # keep watching after the fault ends (heat keeps rising)
    "demo_recover_s": 60,             # stay TRIPPED this long (dashboard shows it), then RESET
    "demo_rest_s": 20,                # pause between shows when nothing was cut
    "demo_cutoff_on": "any",          # any confirmed AI detection cuts | "injected": only the injected fault
    # operator modes/commands from the dashboard
    "hold_target_mv": 700, "hold_top_up_mv": 30, "hold_recheck_s": 15,
    "discharge_target_mv": 400,
    "cmd_min_target_mv": 150, "cmd_max_target_mv": 950,
    # AI + OTA
    "ai_enabled": True,
    "models_dir": str(HOME / "models"),
    "mlops_url": "https://cellguard.misakachan.moe/edge-api",   # token: EDGE_TOKEN= in secrets.env
    "ota_interval_s": 3600, "ota_first_check_s": 30,
    "ai_report_dedup_s": 120,
    "ai_match_grace_s": {"HEAT": 180, "LEAK": 60, "ESR": 30},
}

STATES = ("BOOT_SAFE", "SELF_TEST", "LOCKED", "READY", "ARMED", "ACTIVE", "TRIPPED")
TRIPS = {0: "NONE", 1: "PACK_OV", 2: "CELL_OV", 3: "OVER_TEMP", 4: "HEARTBEAT_TIMEOUT",
         5: "ADC_SATURATED", 6: "TAP_INVERSION", 7: "AREF_INVALID", 8: "HOST_TRIP",
         9: "WATCHDOG_RESET", 10: "BROWNOUT_RESET", 11: "STALE_SAMPLE", 12: "INTERNAL",
         13: "CHARGE_NO_RESPONSE", 14: "TAP_STEP", 15: "TEMP_SENSOR_FAULT",
         16: "CELL_IMBALANCE", 17: "DISCHARGE_NO_RESPONSE"}
TEMP_Q = ("VALID", "SATURATED_OR_OPEN", "SHORT_OR_LOW", "ADC_INVALID")
INVALID = -32768
FEATURE = {"esr": ("ESR", 1), "leak": ("LEAK", 2), "heat": ("HEAT", 3)}


def utcnow() -> dt.datetime:
    return dt.datetime.now(dt.timezone.utc)


def iso(t: dt.datetime) -> str:
    return t.isoformat(timespec="milliseconds").replace("+00:00", "Z")


def log(msg: str) -> None:
    print(f"{iso(utcnow())} {msg}", flush=True)


def crc(payload: bytes) -> str:
    return "%04X" % binascii.crc_hqx(payload, 0xFFFF)


class LinkError(Exception):
    pass


class NoSync(LinkError):
    pass


class Tripped(Exception):
    pass


class Paused(Exception):
    pass


class Skip(Exception):
    """A step cannot run right now; not an error."""


# --------------------------------------------------------------------------- raw log
class RawLog:
    def __init__(self, directory: Path, keep_days: int):
        self.dir, self.keep = directory, keep_days
        self.dir.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        self.day, self.fh = None, None

    def name(self) -> str:
        return f"uart-{self.day}.log"

    def write(self, direction: str, line: str) -> None:
        now = utcnow()
        day = now.strftime("%Y%m%d")
        with self.lock:
            if day != self.day:
                self._rotate(day)
            self.fh.write(f"{iso(now)} {direction} {line}\n")

    def flush(self) -> None:
        with self.lock:
            if self.fh:
                self.fh.flush()

    def _rotate(self, day: str) -> None:
        if self.fh:
            self.fh.close()
        self.day = day
        self.fh = open(self.dir / self.name(), "a", encoding="ascii", errors="replace", buffering=1 << 16)
        threading.Thread(target=self._housekeep, daemon=True).start()

    def _housekeep(self) -> None:
        cutoff = time.time() - self.keep * 86400
        for p in sorted(self.dir.glob("uart-*.log")):
            if p.name == self.name():
                continue
            try:
                with open(p, "rb") as src, gzip.open(str(p) + ".gz", "wb") as dst:
                    shutil.copyfileobj(src, dst)
                p.unlink()
            except OSError as e:
                log(f"rawlog gzip {p.name}: {e}")
        for p in self.dir.glob("uart-*.log.gz"):
            try:
                if p.stat().st_mtime < cutoff:
                    p.unlink()
            except OSError:
                pass


# --------------------------------------------------------------------------- outbox + MQTT
class Outbox:
    def __init__(self, path: Path, max_rows: int):
        self.max_rows = max_rows
        self.lock = threading.Lock()
        self.db = sqlite3.connect(str(path), check_same_thread=False, isolation_level=None)
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA synchronous=NORMAL")
        # AUTOINCREMENT: ids never go back to 1 when the table empties (the uploader keeps a cursor)
        cols = "(id INTEGER PRIMARY KEY AUTOINCREMENT, topic TEXT NOT NULL, payload TEXT NOT NULL, created REAL NOT NULL)"
        old = self.db.execute("SELECT sql FROM sqlite_master WHERE name='msgs'").fetchone()
        if old is None:
            self.db.execute("CREATE TABLE msgs " + cols)
        elif "AUTOINCREMENT" not in old[0].upper():
            self.db.execute("BEGIN")
            self.db.execute("CREATE TABLE msgs_new " + cols)
            self.db.execute("INSERT INTO msgs_new SELECT id, topic, payload, created FROM msgs")
            self.db.execute("DROP TABLE msgs")
            self.db.execute("ALTER TABLE msgs_new RENAME TO msgs")
            self.db.execute("COMMIT")
        self.pending: list = []
        self.last_trim = 0.0

    def put(self, topic: str, payload: dict) -> None:
        with self.lock:
            self.pending.append((topic, json.dumps(payload, separators=(",", ":")), time.time()))

    def flush(self) -> None:
        with self.lock:
            rows, self.pending = self.pending, []
            if rows:
                self.db.execute("BEGIN")
                self.db.executemany("INSERT INTO msgs (topic, payload, created) VALUES (?,?,?)", rows)
                self.db.execute("COMMIT")
            if time.time() - self.last_trim > 600:
                self.last_trim = time.time()
                lo, hi = self._id_range()
                n = hi - lo + 1 if hi else 0
                if n > self.max_rows:
                    self.db.execute("DELETE FROM msgs WHERE id <= ?", (hi - self.max_rows,))
                    log(f"outbox over cap: dropped rows up to id {hi - self.max_rows}")

    def fetch(self, after: int, n: int) -> list:
        with self.lock:
            return self.db.execute("SELECT id, topic, payload FROM msgs WHERE id > ? ORDER BY id LIMIT ?",
                                   (after, n)).fetchall()

    def delete(self, ids: list) -> None:
        if not ids:
            return
        with self.lock:
            self.db.execute("BEGIN")
            self.db.executemany("DELETE FROM msgs WHERE id = ?", [(i,) for i in ids])
            self.db.execute("COMMIT")

    def _id_range(self) -> tuple:
        # MIN/MAX(rowid) is a b-tree lookup; COUNT(*) scans the whole table (500 MB on the SD card
        # after a long offline spell) while holding the lock, which stalls the main loop and heartbeat
        # (two subqueries: a single "SELECT MIN(id), MAX(id)" defeats SQLite's min/max optimisation)
        lo, hi = self.db.execute("SELECT (SELECT MIN(id) FROM msgs), (SELECT MAX(id) FROM msgs)").fetchone()
        return (lo or 0, hi or 0)

    def backlog(self) -> int:
        """Upper bound (ids are AUTOINCREMENT; acked rows may leave small gaps)."""
        with self.lock:
            lo, hi = self._id_range()
            return (hi - lo + 1 if hi else 0) + len(self.pending)


def read_secrets(path: str) -> dict:
    out = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        v = v.strip()
        if len(v) >= 2 and v[0] == v[-1] and v[0] in "'\"":
            v = v[1:-1]
        out[k.strip()] = v
    return out


class Publisher(threading.Thread):
    MAX_INFLIGHT = 32

    def __init__(self, cfg: dict, outbox: Outbox):
        super().__init__(daemon=True, name="mqtt")
        self.cfg, self.outbox = cfg, outbox
        self.connected = False
        # paho calls on_publish while holding its own _out_message_mutex, and publish() takes
        # that mutex too: callbacks must never wait on a lock the publishing thread holds.
        # They only enqueue; this thread owns inflight/cursor.
        self.events: queue.SimpleQueue = queue.SimpleQueue()
        self.inflight: dict = {}
        self.cursor = 0
        self.sent = 0
        self.last_error = None
        self.enabled = True
        self.commands: queue.SimpleQueue = queue.SimpleQueue()   # remote commands for the Edge loop
        self.command_topic = f"cellguard/v1/{cfg['device_id']}/command"

    def run(self) -> None:
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            self.last_error = "python3-paho-mqtt not installed"
            self.enabled = False
            log("MQTT disabled: " + self.last_error)
            return
        try:
            sec = read_secrets(self.cfg["mqtt_secrets"])
            user, password = sec["MQTT_USERNAME"], sec["MQTT_PASSWORD"]
        except (OSError, KeyError) as e:
            self.last_error = f"secrets unavailable: {e!r}"
            self.enabled = False
            log("MQTT disabled: " + self.last_error)
            return
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                             client_id=f"{self.cfg['device_id']}-edge", clean_session=True)
        client.username_pw_set(user, password)
        client.tls_set(ca_certs=self.cfg["mqtt_ca"])
        client.max_inflight_messages_set(self.MAX_INFLIGHT)
        client.reconnect_delay_set(2, 60)

        def on_connect(c, u, flags, rc, props=None):
            ok = not rc.is_failure if hasattr(rc, "is_failure") else rc == 0
            self.events.put(("conn", ok))
            log(f"MQTT connect: {rc}")
            if not ok:
                self.last_error = f"connect {rc}"
            else:
                c.subscribe(self.command_topic, qos=1)

        def on_message(c, u, msg):
            try:
                body = json.loads(msg.payload.decode("utf-8"))
                if isinstance(body, dict):
                    self.commands.put(body)
            except (UnicodeDecodeError, ValueError):
                log(f"ignored command payload on {msg.topic}")

        def on_disconnect(c, u, flags, rc, props=None):
            self.events.put(("conn", False))
            log(f"MQTT disconnected: {rc}")

        def on_publish(c, u, mid, rc=None, props=None):
            self.events.put(("ack", mid))

        client.on_connect, client.on_disconnect, client.on_publish = on_connect, on_disconnect, on_publish
        client.on_message = on_message
        client.connect_async(self.cfg["mqtt_host"], int(self.cfg["mqtt_port"]), keepalive=30)
        client.loop_start()
        while True:
            try:
                done = []
                while True:
                    try:
                        kind, val = self.events.get_nowait()
                    except queue.Empty:
                        break
                    if kind == "ack":
                        rid = self.inflight.pop(val, None)
                        if rid is not None:
                            done.append(rid)
                    else:                       # (re)connect or disconnect: resend everything unacked
                        self.connected = val
                        self.inflight.clear()
                        self.cursor = 0
                self.outbox.delete(done)
                self.sent += len(done)
                if not self.inflight:
                    self.cursor = 0            # everything sent so far is acked: rescan from the start
                room = self.MAX_INFLIGHT - len(self.inflight) if self.connected else 0
                if room > 0:
                    for rid, topic, payload in self.outbox.fetch(self.cursor, room):
                        info = client.publish(topic, payload, qos=1)
                        if info.rc != 0:
                            break
                        self.inflight[info.mid] = rid
                        self.cursor = rid
                time.sleep(0.2)
            except Exception as e:  # never let the uploader die
                self.last_error = repr(e)
                log(f"MQTT loop error: {e!r}")
                time.sleep(5)


# --------------------------------------------------------------------------- Nano link
class Nano:
    """UART link: framing, CRC, ACK matching, retries, latest state."""

    def __init__(self, port: str, raw: RawLog, on_frame):
        self.port, self.raw, self.on_frame = port, raw, on_frame
        self.fd = None
        self.seq = 0
        self.cmd_lock = threading.Lock()
        self.ack_cv = threading.Condition()
        self.acks: dict = {}
        self.stop = False
        self.reader = None
        self.last_rx = 0.0
        self.counters = collections.Counter()

    # -- port
    def open(self) -> None:
        self.close()
        fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.ioctl(fd, termios.TIOCEXCL)
            a = termios.tcgetattr(fd)
            a[0] = 0
            a[1] = 0
            a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            a[3] = 0
            a[4] = a[5] = termios.B115200
            a[6][termios.VMIN] = 0
            a[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, a)
            termios.tcflush(fd, termios.TCIOFLUSH)
        except Exception:
            os.close(fd)
            raise
        self.fd = fd
        self.stop = False
        self.last_rx = time.monotonic()
        self.reader = threading.Thread(target=self._read_loop, daemon=True, name="uart")
        self.reader.start()
        log(f"serial open {self.port}")

    def close(self) -> None:
        self.stop = True
        if self.reader and self.reader is not threading.current_thread():
            self.reader.join(2)
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None

    def _read_loop(self) -> None:
        buf = bytearray()
        fd = self.fd
        while not self.stop:
            try:
                r, _, _ = select.select([fd], [], [], 0.5)
                if not r:
                    continue
                data = os.read(fd, 4096)
            except OSError as e:
                self.counters["read_error"] += 1
                log(f"serial read error: {e}")
                time.sleep(1)
                continue
            if not data:
                continue
            buf += data
            while True:
                i = buf.find(b"\n")
                if i < 0:
                    break
                line = bytes(buf[:i]).strip(b"\r")
                del buf[:i + 1]
                if line:
                    self._line(line)
            if len(buf) > 1024:
                self.counters["overflow"] += 1
                buf.clear()

    def _line(self, line: bytes) -> None:
        j = line.rfind(b"$")
        star = line.rfind(b"*")
        text = line.decode("ascii", "replace")
        if j < 0 or star < j or len(line) - star != 5:
            self.counters["bad_frame"] += 1
            self.raw.write("RX?", text)
            return
        payload = line[j + 1:star]
        if crc(payload) != line[star + 1:].decode("ascii", "replace").upper() or \
                any(b < 0x20 or b > 0x7E for b in payload):
            self.counters["crc_error"] += 1
            self.raw.write("RX!", text)
            return
        self.raw.write("RX", line[j:].decode("ascii"))
        self.last_rx = time.monotonic()
        self.counters["rx"] += 1
        fields = payload.decode("ascii").split(",")
        if fields[0] == "CGA" and len(fields) >= 4:
            try:
                seq = int(fields[1])
            except ValueError:
                return
            with self.ack_cv:
                self.acks[seq] = (fields[2] == "OK", fields[3])
                self.ack_cv.notify_all()
            return
        try:
            self.on_frame(fields)
        except Exception as e:
            self.counters["frame_handler_error"] += 1
            log(f"frame handler error {fields[0]}: {e!r}")

    # -- commands
    def _write(self, payload: str) -> None:
        frame = f"${payload}*{crc(payload.encode('ascii'))}\n".encode("ascii")
        if self.fd is None:
            raise LinkError("port closed")
        self.raw.write("TX", frame.decode().strip())
        os.write(self.fd, frame)

    def command(self, name: str, *args, timeout: float = 0.8, tries: int = 3):
        """Return (ok, text). Retries reuse the seq: the Nano replays the ACK of a duplicate."""
        with self.cmd_lock:
            self.seq = (self.seq + 1) & 0xFFFF
            seq = self.seq
            payload = ",".join(["CGC", str(seq), name] + [str(a) for a in args])
            with self.ack_cv:
                self.acks.pop(seq, None)
            for attempt in range(tries):
                try:
                    self._write(payload)
                except OSError as e:
                    raise LinkError(f"write: {e}")
                deadline = time.monotonic() + timeout
                with self.ack_cv:
                    while seq not in self.acks:
                        left = deadline - time.monotonic()
                        if left <= 0:
                            break
                        self.ack_cv.wait(left)
                    res = self.acks.pop(seq, None)
                if res is not None:
                    if attempt:
                        self.counters["ack_retry_ok"] += 1
                    if not res[0] and res[1] == "NOSYNC":
                        raise NoSync(name)
                    return res
                self.counters["ack_timeout"] += 1
            raise LinkError(f"no ACK for {name} (seq {seq})")

    def sync(self) -> None:
        with self.cmd_lock:
            self.seq = random.randint(1, 30000)
        ok, text = self.command("SYNC", timeout=1.0, tries=4)
        if not ok:
            raise LinkError(f"SYNC refused: {text}")


# --------------------------------------------------------------------------- telemetry state
class Telemetry:
    def __init__(self):
        self.cells = None          # latest raw mV (list of 4) or None if invalid
        self.temps = [None] * 4    # latest valid °C or None
        self.cgt = None            # latest parsed CGT dict
        self.cgt_at = 0.0
        self.history = collections.deque(maxlen=5)
        self.nano_state = "UNKNOWN"
        self.trip = 0
        self.last_trip = None      # reason name from the latest CGX / CGE TRIP
        self.relay = False
        self.runtime = None        # parsed CGR
        self.calibration_id = None
        self.firmware = "unknown"
        self.events = collections.deque(maxlen=200)   # (monotonic, name, cell, req, cmd, reason)

    def filtered(self):
        if len(self.history) < 3:
            return None
        recent = list(self.history)[-3:]
        return [statistics.median(s[i] for s in recent) for i in range(4)]

    def spread(self):
        f = self.filtered()
        return None if f is None else max(f) - min(f)

    def raw_spread(self):
        return None if self.cells is None else max(self.cells) - min(self.cells)

    def event_since(self, t0: float, *names):
        for e in reversed(list(self.events)):
            if e[0] < t0:
                break
            if e[1] in names:
                return e
        return None


# --------------------------------------------------------------------------- edge service
class Edge:
    def __init__(self, cfg: dict, passive: bool):
        self.cfg = cfg
        self.force_passive = passive
        self.data = Path(cfg["data_dir"])
        self.data.mkdir(parents=True, exist_ok=True)
        self.raw = RawLog(self.data / "raw", int(cfg["raw_log_days"]))
        self.outbox = Outbox(self.data / "outbox.sqlite3", int(cfg["outbox_max_rows"]))
        self.pub = Publisher(cfg, self.outbox)
        self.tel = Telemetry()
        self.nano = Nano(cfg["serial_port"], self.raw, self._frame)
        self.state_file = self.data / "state.json"
        self.persist = self._load_state()
        self.msg_n = 0
        self.cgt_n = 0
        self.phase = "starting"
        self.cycle_id = None
        self.fault_req = None
        self.last_off = 0.0
        self.trip_times: collections.deque = collections.deque(maxlen=20)
        self.pause_until = 0.0
        self.pause_reason = None
        self.charge_noresp = 0         # consecutive CHARGE_NO_RESPONSE trips
        self.charger_pauses = 0
        self._control_at = self._control().get("at")   # resume = a newer stamp than this
        self.selftest_started = None
        self.last_error = None
        self.notes = collections.Counter()
        self.hb_stop = threading.Event()
        self.early: list = []          # non-telemetry held until the first sample registered the device
        self.first_sample_at = None
        self.msg_lock = threading.Lock()   # message ids are made by the main, uart, ai and ota threads
        self.t0 = time.monotonic()
        # ground truth: what the Pi itself commanded, from the CGT/CGP actuator masks
        self.gt_lock = threading.Lock()
        self.gt: dict = {}             # (TYPE, cell) -> {"last": monotonic, "first": monotonic, "purpose": str}
        self.gt_prev: set = set()
        self.episode = None            # fault mode: current episode bookkeeping
        self.reported: dict = {}       # (TYPE, cell) -> monotonic of the last OBSERVABLE event
        self.ai_events = collections.deque(maxlen=20)
        self.demo = None               # demo mode: {"ep", "hit", "cut"} while the AI cutoff is armed
        self.ai_cutoff = None          # the TRIP the Pi sent on an AI detection (until the trip is reset)
        self.running_mode = None
        ctl = self._control()
        self._ctl_seen = {"ota_at": ctl.get("ota_at"), "reload_at": ctl.get("reload_at")}
        self.ota_wake = threading.Event()
        self.ai = self.ota = None
        self.ai_disabled = None
        self._setup_ai()
        self._new_run()

    # -- AI + OTA
    def _setup_ai(self) -> None:
        if not self.cfg.get("ai_enabled", True):
            self.ai_disabled = "disabled in config"
            return
        try:
            import cg_ai
        except ImportError as e:          # numpy missing: collect without AI rather than not at all
            self.ai_disabled = f"cg_ai unavailable: {e}"
            log("AI disabled: " + self.ai_disabled)
            return
        self.cg_ai = cg_ai
        http = None
        url = (self.cfg.get("mlops_url") or "").strip()
        if url:
            token = None
            try:
                token = read_secrets(self.cfg["mqtt_secrets"]).get("EDGE_TOKEN")
            except OSError:
                pass
            http = cg_ai.Http(url, token)
        self.ota = cg_ai.Ota(Path(self.cfg["models_dir"]), self.cfg["pack_id"], http, self._ota_report)
        self.ai = cg_ai.AiEngine(self.ota, self.cfg, self._on_ai_event, log=log)

    def _ota_report(self, model, version, status, detail) -> None:
        self.diag("OTA_RESULT", f"{model} {version}: {status}", model=model, version=version,
                  status=status, detail=str(detail)[:500], versions=self.ota.versions())

    def _ota_loop(self) -> None:
        first = True
        while True:
            self.ota_wake.wait(self.cfg["ota_first_check_s"] if first else self.cfg["ota_interval_s"])
            requested = self.ota_wake.is_set()
            self.ota_wake.clear()
            first = False
            try:
                res = self.ota.check_all()
            except Exception as e:         # never let the OTA thread die
                res = {"_": {"action": "error", "detail": repr(e)[:300]}}
            if any(r.get("action") == "applied" for r in res.values()):
                self.ai.reload()
            errors = {k: r.get("detail") for k, r in res.items() if r.get("action") == "error"}
            if requested or errors:
                self.diag("OTA_CHECK", "requested" if requested else "check failed",
                          versions=self.ota.versions(), result={k: r.get("action") for k, r in res.items()},
                          errors=errors or None)

    def _record_truth(self, kind: str, mask_bits, now: float) -> None:
        """Note which (type, cell) the actuators are driving right now (reader thread)."""
        purpose = (self.fault_req or {}).get("purpose", "injection")
        with self.gt_lock:
            for cell in mask_bits:
                e = self.gt.get((kind, cell))
                if e is None or now - e["last"] > 5:
                    self.gt[(kind, cell)] = {"first": now, "last": now, "purpose": purpose}
                else:
                    e["last"] = now

    def _truth_near(self, now: float) -> list:
        grace = self.cfg["ai_match_grace_s"]
        with self.gt_lock:
            return [{"type": k[0], "cell": k[1], "purpose": e["purpose"], "active": now - e["last"] < 2,
                     "age_s": round(now - e["last"], 1), "since_s": round(now - e["first"], 1)}
                    for k, e in self.gt.items() if now - e["last"] <= grace.get(k[0], 60)]

    def _on_ai_event(self, ev: dict) -> None:
        """AI thread: attach ground truth, report to the server."""
        now = time.monotonic()
        truth = self._truth_near(now)
        on_cell = [g for g in truth if g["cell"] == ev["cell"]]
        ftype = ev.get("fault_type")
        hits = [g for g in on_cell if ftype == "ANOMALY" or g["type"] == ftype]
        if any(g["purpose"] == "injection" for g in hits):
            match = "injected"
        elif hits:
            match = "balance"              # the Pi was bleeding this cell to balance the pack
        else:
            match = "none"
        ev = dict(ev, match=match, ground_truth=truth, mode=self.running_mode,
                  episode_id=(self.episode or {}).get("id"))
        self.ai_events.append({"at": iso(utcnow()), **{k: ev.get(k) for k in
                               ("model", "event", "cell", "fault_type", "match", "version")}})
        if ev["event"] == "detected":
            self.diag("AI_DETECTION", f"{ev['model']} {ftype} cell {ev['cell']} ({match})", **ev)
            ep = self.episode
            if ep is not None:
                ep["detections"].append({"t": now, "model": ev["model"], "type": ftype, "cell": ev["cell"],
                                         "match": match})
            d = self.demo
            if d is not None and d.get("cutoff", True) and d["hit"] is None \
                and ftype in ("ESR", "LEAK", "HEAT", "ANOMALY") \
                    and match != "balance" and (self.cfg["demo_cutoff_on"] != "injected" or match == "injected"):
                d["hit"] = dict(ev, t=now)     # the main thread sends TRIP on its next tick
            if ftype in ("ESR", "LEAK", "HEAT"):
                last = self.reported.get((ftype, ev["cell"]), -1e9)
                if now - last >= self.cfg["ai_report_dedup_s"]:
                    self.reported[(ftype, ev["cell"])] = now
                    self.fault_event("OBSERVABLE", ftype, ev["cell"],
                                     {"source": "edge_ai", "model": ev["model"], "version": ev.get("version"),
                                      "match": match, "ground_truth": truth,
                                      "scores": {k: ev[k] for k in ("score", "score_v", "score_t", "prob", "z")
                                                 if k in ev}})
        else:
            self.diag("AI_CLEARED", f"{ev['model']} cell {ev['cell']}", **ev)

    # -- persistence / ids
    def _load_state(self) -> dict:
        try:
            return json.loads(self.state_file.read_text())
        except (OSError, ValueError):
            return {"cycle_index": 0}

    def _save_state(self) -> None:
        tmp = self.state_file.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.persist))
        os.replace(tmp, self.state_file)

    def _new_run(self) -> None:
        now = utcnow()
        with self.msg_lock:
            self.run_id = f"edge-{now.strftime('%Y%m%dT%H%M%SZ')}-{os.urandom(2).hex()}"
            self.msg_n = 0
        self.run_start = time.monotonic()
        self.run_start_utc = now
        log(f"run_id {self.run_id}")

    def _msg_id(self) -> str:
        with self.msg_lock:
            self.msg_n += 1
            return f"{self.run_id}:{self.msg_n:09d}"

    def _base(self, schema: str) -> dict:
        mid = self._msg_id()
        return {"schema_version": schema, "device_id": self.cfg["device_id"], "run_id": mid.rsplit(":", 1)[0],
                "message_id": mid, "edge_version": VERSION}

    def _topic(self, kind: str) -> str:
        return f"cellguard/v1/{self.cfg['device_id']}/{kind}"

    def put(self, kind: str, payload: dict) -> None:
        # The gateway's diagnostics/selftest tables reference devices(device_id), which the
        # first telemetry sample creates. Hold other kinds briefly until that has happened.
        if kind == "telemetry":
            self.outbox.put(self._topic(kind), payload)
            if self.first_sample_at is None:
                self.first_sample_at = time.monotonic()
            return
        with self.msg_lock:
            if self.early is not None:
                self.early.append((kind, payload))
                return
        self.outbox.put(self._topic(kind), payload)

    def release_early(self) -> None:
        if self.early is not None and self.first_sample_at and time.monotonic() - self.first_sample_at > 5:
            with self.msg_lock:
                held, self.early = self.early, None
            for kind, payload in held:
                self.outbox.put(self._topic(kind), payload)

    def diag(self, kind: str, message: str = "", **details) -> None:
        if kind not in ("NANO_EVENT", "LOADED_SAMPLE"):
            log(f"[{kind}] {message} {details if details else ''}".rstrip())
        p = self._base("diagnostic_record_v1")
        p.update(occurred_at=iso(utcnow()), kind=kind, message=message,
                 details={"phase": self.phase, "cycle_id": self.cycle_id, **details})
        self.put("diagnostics", p)

    def fault_event(self, event_type: str, fault_type: str, cell, details: dict) -> None:
        p = self._base("fault_event_v1")
        p.update(event_id=p["message_id"], occurred_at=iso(utcnow()), event_type=event_type,
                 fault_type=fault_type, cell_id=cell if cell in (1, 2, 3, 4) else None,
                 details={"phase": self.phase, "cycle_id": self.cycle_id, **details},
                 firmware_version=self.tel.firmware, hardware_revision=self.cfg["hardware_revision"])
        self.put("events", p)

    # -- frames from the Nano (reader thread)
    def _frame(self, f: list) -> None:
        kind = f[0]
        tel = self.tel
        if kind == "CGT" and len(f) == 23:
            v = [int(x) for x in f[1:]]
            d = {"seq": v[0], "ms": v[1], "cells": v[2:6], "temps": v[6:10], "charge": v[10],
                 "drain": v[11], "armed": v[12], "esr": v[13], "leak": v[14], "heat": v[15:19],
                 "trip": v[19], "vq": v[20], "tq": v[21]}
            vq = d["vq"]
            vqual = ("AREF_INVALID" if vq & 4 else "ADC_SATURATED" if vq & 1 else
                     "TAP_INVERSION" if vq & 2 else "MISSING" if vq & 8 else "VALID")
            tq = [TEMP_Q[(d["tq"] >> (2 * i)) & 3] for i in range(4)]
            temps = [None if (t == INVALID or tq[i] != "VALID") else t / 100.0 for i, t in enumerate(d["temps"])]
            for i in range(4):
                if temps[i] is None and tq[i] == "VALID":
                    tq[i] = "ADC_INVALID"
            cells = d["cells"] if vqual == "VALID" and INVALID not in d["cells"] else None
            if vqual == "VALID" and cells is None:
                vqual = "OTHER_ERROR"
            tel.cgt, tel.cgt_at, tel.cells, tel.temps, tel.trip = d, time.monotonic(), cells, temps, d["trip"]
            if cells:
                tel.history.append(cells)
            self.cgt_n += 1
            now = time.monotonic()
            if d["esr"]:
                self._record_truth("ESR", [k + 1 for k in range(4) if d["esr"] >> k & 1], now)
            if d["leak"]:
                self._record_truth("LEAK", [k + 1 for k in range(4) if d["leak"] >> k & 1], now)
            if any(d["heat"]):
                self._record_truth("HEAT", [k + 1 for k in range(4) if d["heat"][k] > 0], now)
            if self.ai is not None:
                self.ai.feed_sample(self.cg_ai.step_from_cgt(now - self.t0, d["cells"], temps, tq, vqual == "VALID",
                                                             bool(d["charge"]), bool(d["drain"])), d["ms"])
            if self.cgt_n % max(1, int(self.cfg["telemetry_every_n"])) == 0:
                self._publish_sample(d, cells, temps, tq, vqual)
        elif kind == "CGS" and len(f) >= 6:
            tel.nano_state, tel.trip, tel.relay = f[3], int(f[4]), f[5] == "1"
        elif kind == "CGE" and len(f) == 8:
            name, cell, req, cmd, reason = f[3], int(f[4]), int(f[5]), int(f[6]), f[7]
            tel.events.append((time.monotonic(), name, cell, req, cmd, reason))
            if name == "STATE" and reason in STATES:
                tel.nano_state = reason
            elif name == "TRIP":
                tel.last_trip = reason
            self._nano_event(name, cell, req, cmd, reason, f)
        elif kind == "CGR" and len(f) == 14:
            v = [int(x) for x in f[3:]]
            tel.runtime = dict(zip(("enabled", "passed", "declared", "profile", "generation", "stored",
                                    "session", "heat_pwm", "heat_s", "esr_s", "leak_s"), v))
        elif kind == "CGV" and len(f) >= 4:
            tel.firmware = f[1]
        elif kind == "CGK" and len(f) >= 4:
            tel.calibration_id = f[3]
        elif kind == "CGX":
            if len(f) > 2 and f[2].isdigit():
                tel.last_trip = TRIPS.get(int(f[2]), f[2])
            self.diag("NANO_TRIP_SNAPSHOT", "trip snapshot frame", fields=f[1:])
        elif kind == "CGP" and len(f) == 9:
            direction = {1: "CHARGE", 2: "DISCHARGE"}.get(int(f[3]), f[3])
            esr_mask, cells_mv = int(f[4]), [int(x) for x in f[5:9]]
            if esr_mask:
                self._record_truth("ESR", [k + 1 for k in range(4) if esr_mask >> k & 1], time.monotonic())
            self.loaded_n = getattr(self, "loaded_n", 0) + 1
            self.diag("LOADED_SAMPLE", "", nano_send_ms=int(f[1]), nano_ms=int(f[2]),
                      direction=direction, esr_mask=esr_mask, cells_mv=cells_mv)
            if self.ai is not None and INVALID not in cells_mv:
                self.ai.feed_loaded(self.cg_ai.np.array(cells_mv, float) / 1000.0, int(f[2]), direction, esr_mask)

    def _nano_event(self, name, cell, req, cmd, reason, f) -> None:
        detail = {"nano_event": name, "cell": cell, "requested": req, "commanded": cmd,
                  "reason": reason, "nano_event_seq": int(f[1]), "nano_ms": int(f[2])}
        if name == "SELF_TEST":
            p = self._base("selftest_report_v1")
            started = self.selftest_started or utcnow()
            p.update(started_at=iso(started), finished_at=iso(utcnow()),
                     result=reason if reason in ("PASS", "WARN", "FAIL") else "FAIL",
                     checks={"fail_mask": req, "warn_mask": cmd, "reason": reason, "nano_event_seq": int(f[1])},
                     firmware_version=self.tel.firmware)
            self.put("selftest", p)
        elif name == "TRIP":
            cut = self.ai_cutoff
            if cut is not None and reason == "HOST_TRIP":
                ftype = cut["fault_type"] if cut["fault_type"] in ("ESR", "LEAK", "HEAT") else "NONE"
                self.fault_event("TRIPPED", ftype, cut["cell"], {**detail, "cause": "AI_CUTOFF", **cut["report"]})
            else:
                self.fault_event("TRIPPED", "NONE", None, detail)
        elif name in ("RESET", "BOOT"):
            self.fault_event(name, "NONE", None, detail)
        elif name.split("_")[0] in ("ESR", "LEAK", "HEAT") and name.endswith(("_START", "_STOP")):
            self.fault_event("COMMANDED" if name.endswith("_START") else "RELEASED",
                             name.split("_")[0], cell, detail)
        elif name not in ("HEALTH_STAGE", "HEALTH_LOG"):
            self.diag("NANO_EVENT", name, **detail)
        if name == "BOOT":
            self.notes["nano_boot"] += 1

    def _publish_sample(self, d, cells, temps, tq, vqual) -> None:
        cell_v = [None if cells is None else c / 1000.0 for c in (cells or [None] * 4)]
        taps, acc = [], 0.0
        for v in cell_v:
            acc = None if v is None or acc is None else round(acc + v, 3)
            taps.append(acc)
        injected = bool(d["esr"] or d["leak"] or any(d["heat"]))
        p = self._base("normalized_sample_v1")
        p.update(
            source="measured_fault_injection" if injected else "measured_normal",
            pack_id=self.cfg["pack_id"], scenario_id=f"edge_{self.running_mode or 'passive'}_v3",
            timestamp_utc=iso(utcnow()), t_sim_s=round(time.monotonic() - self.run_start, 3),
            cell_voltage_v=cell_v, cumulative_tap_voltage_v=taps, cell_temperature_c=temps,
            temperature_quality=tq, pack_current_a=None, current_quality="INA219_UNAVAILABLE",
            charge_commanded=bool(d["charge"]), discharge_commanded=bool(d["drain"]),
            pack_relay_commanded=bool(self.tel.relay), armed_commanded=bool(d["armed"]),
            esr_command_mask=d["esr"], leak_command_mask=d["leak"], heater_pwm_command=d["heat"],
            trip_reason=TRIPS.get(d["trip"], str(d["trip"])), trip_code=d["trip"],
            controller_state=self.tel.nano_state, voltage_quality=vqual, voltage_quality_bits=d["vq"],
            fault_requested=self.fault_req, fault_modeled_applied=None,
            physical_fault_verified=False, physical_harness_verified=False,
            calibration_id=self.tel.calibration_id, firmware_version=self.tel.firmware,
            hardware_revision=self.cfg["hardware_revision"], protocol_version="1.0",
            raw_log_ref=f"edge:raw/uart-{utcnow().strftime('%Y%m%d')}.log#CGT{d['seq']}",
            telemetry_seq=d["seq"], nano_millis=d["ms"], phase=self.phase, cycle_id=self.cycle_id,
            spread_mv=None if cells is None else max(cells) - min(cells),
        )
        if cells is None:
            p["untrusted_cell_voltage_mv"] = d["cells"]
        self.put("telemetry", p)

    # -- heartbeat (separate thread: PING every second, refreshes the Nano heartbeat)
    def _heartbeat(self) -> None:
        while not self.hb_stop.wait(1.0):
            try:
                if self.nano.fd is not None:
                    self.nano.command("PING", timeout=0.6, tries=2)
            except NoSync:
                self.notes["ping_nosync"] += 1
            except LinkError:
                self.notes["ping_fail"] += 1

    # -- housekeeping called from every wait loop
    def tick(self) -> None:
        self.release_early()
        self.outbox.flush()
        self.raw.flush()
        now = time.monotonic()
        if now - getattr(self, "_status_at", 0) > 2:
            self._status_at = now
            self._write_status()
            sd_notify("WATCHDOG=1")
        if now - self.nano.last_rx > 15:
            raise LinkError("no frames from Nano for 15 s")
        d = self.demo
        if d is not None and d["hit"] is not None and d["cut"] is None:
            self._ai_cutoff(d)
        if self.tel.trip or self.tel.nano_state == "TRIPPED":
            raise Tripped(self.trip_reason())
        if self.mode() != self.running_mode and self.phase not in ("passive", "trip_recovery"):
            raise Paused()             # mode switched: stop this cycle/episode, the run loop starts the new mode
        once = self._control().get("once")
        if isinstance(once, dict) and once.get("id") != self.persist.get("once_done")                 and self.phase not in ("passive", "trip_recovery"):
            raise Paused()             # one-shot command (charge/discharge/inject): the run loop runs it, then resumes the mode

    def _control(self) -> dict:
        return read_control(self.data)

    def _poll_commands(self) -> None:
        """Remote commands from MQTT cellguard/v1/{device}/command (main thread)."""
        while True:
            try:
                c = self.pub.commands.get_nowait()
            except queue.Empty:
                return
            name = c.get("command")
            ok, note = True, ""
            if name == "mode" and c.get("mode") in MODES:
                set_mode(self.data, c["mode"])
                params = {"target_mv": self._clamp(c["target_mv"], 0)} if c.get("target_mv") is not None else {}
                write_control(self.data, params=params)
                note = f"mode -> {c['mode']}" + (f" ({params['target_mv']} mV)" if params else "")
            elif name in ("pause", "resume"):
                set_mode(self.data, name)
                note = name
            elif name in ("charge", "discharge"):
                default = self.cfg["hold_target_mv"] if name == "charge" else self.cfg["discharge_target_mv"]
                target = self._clamp(c.get("target_mv"), default)
                write_control(self.data, once={"id": time.time(), "action": name, "target_mv": target})
                note = f"{name} to {target} mV"
            elif name == "inject":
                kind, cell = c.get("kind"), c.get("cell")
                if kind in ("heat", "leak", "esr") and cell in (1, 2, 3, 4):
                    once = {"id": time.time(), "action": "inject", "kind": kind, "cell": cell,
                            "ai_cutoff": bool(c.get("ai_cutoff", True))}
                    write_control(self.data, once=once)
                    note = f"inject {kind} on cell {cell}"
                else:
                    ok, note = False, "inject needs kind heat|leak|esr and cell 1..4"
            elif name == "reset_history":
                note = self._reset_history()
            elif name == "ota_check":
                self.ota_wake.set()
                note = "OTA check requested" if self.ota else f"AI off: {self.ai_disabled}"
                ok = self.ota is not None
            elif name == "status":
                note = "status"
            else:
                ok, note = False, "unknown command"
            self.diag("COMMAND_ACK", note, command=c, ok=ok, status=self._status_dict(brief=True))

    def _clamp(self, value, default: int) -> int:
        """A target in mV from a command, kept inside the range the rig can do."""
        c = self.cfg
        try:
            v = int(value)
        except (TypeError, ValueError):
            v = int(default)
        return max(c["cmd_min_target_mv"], min(c["cmd_max_target_mv"], v)) if v else int(default)

    def _param(self, key: str, default: int) -> int:
        p = self._control().get("params") or {}
        return self._clamp(p.get(key), default)

    def _reset_history(self) -> str:
        """Operator 'reset history': forget trips and pauses so experiments/commands start fresh,
        and clear a latched firmware trip."""
        n = len(self.trip_times)
        self.trip_times.clear()
        self.charge_noresp = 0
        self.charger_pauses = 0
        self.pause_until = 0.0
        self.pause_reason = None
        self.ai_cutoff = None
        for key in [k for k in self.notes if k.startswith("trip_")]:
            del self.notes[key]
        note = f"history cleared ({n} trips forgotten)"
        if self.tel.trip or self.tel.nano_state == "TRIPPED":
            try:
                ok, text = self.nano.command("RESET", timeout=0.6, tries=3)
                note += "; Nano RESET " + ("ok" if ok or text == "NOT_TRIPPED" else f"refused: {text}")
                if ok:
                    self.tel.last_trip = None
            except (LinkError, NoSync) as e:
                note += f"; Nano RESET failed: {e}"
        return note

    def _pending_once(self):
        """A one-shot command (charge / discharge / inject) written by _poll_commands, taken at most once."""
        once = self._control().get("once")
        if not isinstance(once, dict) or once.get("id") == self.persist.get("once_done"):
            return None
        self.persist["once_done"] = once.get("id")
        self._save_state()
        return once

    def run_once(self, once: dict) -> None:
        action = once.get("action")
        self.diag("ONCE_START", f"{action}", once=once, cells=self.tel.cells)
        try:
            if action == "charge":
                self.start_session()
                self.charge_to(self._clamp(once.get("target_mv"), self.cfg["hold_target_mv"]))
            elif action == "discharge":
                self.start_session()
                self.drain_to(self._clamp(once.get("target_mv"), self.cfg["discharge_target_mv"]))
            elif action == "inject":
                self.demo_show(forced=once)
        finally:
            try:
                if self.phase != "trip_recovery":
                    self.off()
            except (LinkError, NoSync):
                pass
        self.diag("ONCE_DONE", f"{action} done", once=once, cells=self.tel.cells, temps=self.tel.temps)

    def hold_mode(self) -> None:
        """Charge to a target, then keep the cells there (top up when the highest cell sags)."""
        c = self.cfg
        self.cycle_id = f"{self.run_id}:hold"
        self.start_session()
        self.charge_to(self._param("target_mv", c["hold_target_mv"]))
        while True:
            self.phase = "hold"
            self.wait(c["hold_recheck_s"])
            target = self._param("target_mv", c["hold_target_mv"])
            if max(self.need_cells()) < target - c["hold_top_up_mv"]:
                self.charge_to(target)

    def discharge_mode(self) -> None:
        """Drain to a target (never below the firmware floor), then idle with outputs off."""
        c = self.cfg
        self.cycle_id = f"{self.run_id}:discharge"
        self.start_session()
        self.drain_to(self._param("target_mv", c["discharge_target_mv"]))
        self.off()
        while True:
            self.phase = "discharge_done"
            self.wait(c["hold_recheck_s"])
            target = self._param("target_mv", c["discharge_target_mv"])
            if min(self.need_cells()) > target + 40:
                self.drain_to(target)
                self.off()

    def trip_reason(self) -> str:
        if self.tel.trip:
            return TRIPS.get(self.tel.trip, str(self.tel.trip))
        return self.tel.last_trip or "UNKNOWN"

    def mode(self) -> str:
        self._poll_commands()
        ctl = self._control()
        for key in ("ota_at", "reload_at"):
            if ctl.get(key) != self._ctl_seen[key]:
                self._ctl_seen[key] = ctl.get(key)
                if key == "ota_at":
                    self.ota_wake.set()
                elif self.ai is not None:
                    self.ai.reload()
                    self.diag("AI_RELOAD", "models reloaded after a local install", versions=self.ota.versions())
        if self.force_passive:
            return "passive"
        mode = normalize_mode(ctl.get("mode"))
        at = ctl.get("at")
        if at != self._control_at:
            self._control_at = at
            self.diag("MODE_REQUEST", f"mode -> {mode}", mode=mode)
            if mode != "passive" and self.pause_until > time.monotonic():
                self.pause_until = 0.0
                self.charge_noresp = 0
                self.charger_pauses = 0
                self.trip_times.clear()
                self.diag("PAUSE_CLEARED", "resume requested: experiments restart now")
        return mode

    def _write_status(self) -> None:
        st = self._status_dict()
        tmp = self.data / "status.tmp"
        tmp.write_text(json.dumps(st, indent=1))
        os.replace(tmp, self.data / "status.json")

    def _status_dict(self, brief: bool = False) -> dict:
        tel = self.tel
        ai = {"disabled": self.ai_disabled} if self.ai is None else {
            **self.ai.status(), "recent_events": list(self.ai_events)[-(3 if brief else 20):]}
        ota = None if self.ota is None else {
            "versions": self.ota.versions(), "server": None if self.ota.http is None else self.ota.http.base,
            "last_check": iso(dt.datetime.fromtimestamp(self.ota.last_check, dt.timezone.utc))
            if self.ota.last_check else None,
            "last_result": {k: {"action": r.get("action"), "detail": str(r.get("detail"))[:160]}
                            for k, r in self.ota.last_result.items()}}
        ep = self.episode
        if brief:
            return {"version": VERSION, "mode": self.mode_cached(), "phase": self.phase, "cells_mv": tel.cells,
                    "temps_c": tel.temps, "episode": None if ep is None else ep["id"],
                    "ai_models": None if self.ai is None else self.ai.versions, "ota": ota}
        return {
            "updated": iso(utcnow()), "version": VERSION, "mode": self.mode_cached(), "phase": self.phase,
            "running_mode": self.running_mode,
            "episode": None if ep is None else {k: ep[k] for k in ("id", "plan", "applied")},
            "ai": ai, "ota": ota,
            "cycle_id": self.cycle_id, "run_id": self.run_id, "nano_state": tel.nano_state,
            "trip": TRIPS.get(tel.trip, tel.trip), "cells_mv": tel.cells, "spread_mv_filtered": tel.spread(),
            "temps_c": tel.temps, "firmware": tel.firmware, "runtime": tel.runtime,
            "fault_requested": self.fault_req,
            "paused_until": iso(utcnow() + dt.timedelta(seconds=self.pause_until - time.monotonic()))
            if self.pause_until > time.monotonic() else None,
            "pause_reason": self.pause_reason if self.pause_until > time.monotonic() else None,
            "mqtt": {"enabled": self.pub.enabled, "connected": self.pub.connected, "sent": self.pub.sent,
                     "backlog": self.outbox.backlog(), "last_error": self.pub.last_error},
            "uart": dict(self.nano.counters), "notes": dict(self.notes),
            "charge_no_response_streak": self.charge_noresp,
            "trips_last_hour": sum(1 for t in self.trip_times if time.monotonic() - t < 3600),
            "loaded_samples": getattr(self, "loaded_n", 0),
            "last_error": self.last_error,
        }

    def mode_cached(self) -> str:
        return "passive" if self.force_passive else normalize_mode(self._control().get("mode"))

    def wait(self, seconds: float, guard=None, until=None) -> bool:
        """Sleep while keeping everything alive. Returns True if `until()` became true."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.tick()
            if guard:
                guard()
            if until and until():
                return True
            time.sleep(0.2)
        return False

    # -- low-level actions
    def off(self, why: str = "") -> None:
        self.fault_req = None
        for _ in range(3):
            try:
                self.nano.command("OFF", timeout=1.0, tries=3)
                break
            except NoSync:
                self.nano.sync()
            except LinkError:
                continue
        self.last_off = time.monotonic()

    def cmd(self, name: str, *args):
        try:
            return self.nano.command(name, *args)
        except NoSync:
            self.notes["resync"] += 1
            self.nano.sync()
            raise Skip(f"{name}: Nano lost sync (re-synced)")

    def arm(self) -> None:
        for _ in range(10):
            ok, text = self.cmd("ARM")
            if ok:
                return
            if text == "WAIT_RELAY":
                self.wait(0.5)
                continue
            if text == "NO_SESSION":
                self.start_session()
                continue
            raise Skip(f"ARM refused: {text}")
        raise Skip("ARM: relay did not settle")

    def start_session(self) -> None:
        """SELFTEST then SESSION (SESSION needs selftest <10 s old and >=5 s idle)."""
        self.phase = "session"
        if not self.temps_ok(5.5, 34.5):
            raise Skip(f"temperatures outside 5..35 C or invalid: {self.tel.temps}")
        for attempt in range(4):
            idle_left = 5.5 - (time.monotonic() - self.last_off)
            if idle_left > 0:
                self.wait(idle_left)
            t0 = time.monotonic()
            self.selftest_started = utcnow()
            ok, text = self.cmd("SELFTEST")
            if not ok:
                self.wait(3)
                continue
            self.wait(3, until=lambda: self.tel.event_since(t0, "SELF_TEST") is not None)
            ev = self.tel.event_since(t0, "SELF_TEST")
            if ev is None or ev[5] not in ("PASS", "WARN"):
                self.diag("SELFTEST_NOT_PASSED", "selftest failed or missing",
                          result=None if ev is None else ev[5], fail_mask=None if ev is None else ev[3])
                self.wait(10)
                continue
            ok, text = self.cmd("SESSION")
            if ok:
                self.cmd("FLAGS")
                return
            if text in ("COOLDOWN", "SELFTEST_REQUIRED", "NOT_IDLE"):
                self.off("session retry")
                continue
            raise Skip(f"SESSION refused: {text}")
        raise Skip("SESSION could not be opened")

    def temps_ok(self, lo: float, hi: float) -> bool:
        t = self.tel.temps
        return all(x is not None and lo <= x <= hi for x in t)

    def need_cells(self):
        f = self.tel.filtered()
        if f is None:
            self.wait(4)
            f = self.tel.filtered()
        if f is None:
            raise Skip("no valid cell voltages")
        return f

    # -- guards (run every 0.2 s while an output is on)
    def spread_guard(self) -> None:
        raw, filt = self.tel.raw_spread(), self.tel.spread()
        if raw is not None and raw >= self.cfg["guard_raw_spread_mv"] or \
                filt is not None and filt >= self.cfg["guard_filtered_spread_mv"]:
            self.off("spread guard")
            self.notes["spread_guard_stop"] += 1
            raise Skip(f"spread guard stop (raw {raw}, filtered {filt})")

    def idle_after(self, t0: float, stop_names) -> bool:
        return self.tel.event_since(t0, *stop_names) is not None or \
            (time.monotonic() - t0 > 1.5 and self.tel.nano_state in ("READY", "LOCKED"))

    # -- experiment steps
    def observe(self, phase: str, seconds: float) -> None:
        self.phase = phase
        self.wait(seconds)

    def balance(self, label: str) -> None:
        c = self.cfg
        s = self.tel.spread()
        if s is None or s <= c["balance_start_mv"]:
            return
        self.phase = label
        end = time.monotonic() + c["balance_max_s"]
        start_spread = s
        while time.monotonic() < end:
            f = self.need_cells()
            s = max(f) - min(f)
            if s <= c["balance_target_mv"]:
                break
            if min(f) < 320:
                self.diag("BALANCE_SKIPPED", "cells too low to bleed (LEAK needs >=300 mV)", cells=f)
                return
            hi = max(range(4), key=lambda i: f[i]) + 1
            self.arm()
            ok, text = self.cmd("LEAK", hi, 1)
            if not ok:
                self.off()
                self.diag("BALANCE_STOPPED", f"LEAK refused: {text}", cell=hi)
                return
            self.fault_req = {"type": "LEAK", "cell_id": hi, "active": True, "purpose": "balance"}
            t0 = time.monotonic()

            def done():
                g = self.tel.filtered()
                return g is not None and g[hi - 1] <= min(g) + 8

            def guard():
                g = self.tel.filtered()
                if g is not None and min(g) < 300:
                    self.off()
                    raise Skip("balance: lowest cell under 300 mV")
                self.spread_guard()

            self.wait(70, guard=guard, until=lambda: done() or self.idle_after(t0, ("LEAK_STOP",)))
            self.off()
            self.wait(3)
        self.diag("BALANCE_RESULT", label, start_spread_mv=start_spread, end_spread_mv=self.tel.spread())

    def ready_for_power(self) -> None:
        s = self.tel.spread()
        if s is not None and s > self.cfg["start_spread_max_mv"]:
            self.balance("balance_pre")
            s = self.tel.spread()
        if s is None or s > self.cfg["start_spread_max_mv"] + 8:
            raise Skip(f"spread {s} mV too high to move the pack")

    def charge_to(self, target: int) -> None:
        self.phase = "charge"
        for _ in range(8):
            f = self.need_cells()
            if max(f) >= target - 15:
                return
            self.ready_for_power()
            raw = self.tel.cells or f
            step = min(target, int((max(raw) + 150) // 50 * 50))
            if step <= max(raw) + 10:
                return
            self.arm()
            t0 = time.monotonic()
            ok, text = self.cmd("RAMP", step)
            if not ok:
                self.off()
                self.diag("RAMP_REFUSED", text, target=step, cells=f)
                return
            self.wait(32, guard=self.spread_guard,
                      until=lambda: self.idle_after(t0, ("CHARGE_STOP",)))
            ev = self.tel.event_since(t0, "CHARGE_STOP")
            self.off()
            if ev and ev[5] in ("TARGET", "HOST", "CELL_LIMIT", "PACK_LIMIT", "TIMEOUT"):
                self.charge_noresp = 0
                self.charger_pauses = 0
            self.wait(2)
            if ev and ev[5] not in ("TARGET", "HOST"):
                self.diag("CHARGE_STOPPED", ev[5], target=step, cells=self.tel.cells)
                if ev[5] in ("CELL_LIMIT", "PACK_LIMIT"):
                    return

    def drain_to(self, target: int) -> None:
        self.phase = "drain"
        for _ in range(4):
            f = self.need_cells()
            if min(f) < self.cfg["drain_min_cell_mv"] or min(f) <= target + 40:
                return
            self.ready_for_power()
            self.arm()
            t0 = time.monotonic()
            ok, text = self.cmd("DRAIN", target)
            if not ok:
                self.off()
                self.diag("DRAIN_REFUSED", text, target=target, cells=f)
                return
            self.wait(36, guard=self.spread_guard,
                      until=lambda: self.idle_after(t0, ("DRAIN_STOP",)))
            ev = self.tel.event_since(t0, "DRAIN_STOP")
            self.off()
            self.wait(2)
            if ev and ev[5] in ("DRAIN_TARGET", "CELL_FLOOR", "PACK_FLOOR"):
                return

    def load_pulse(self, cell=None) -> int:
        """Short DRAIN load (pulse_s), optionally with ESR on `cell`: the Nano sends loaded samples (CGP)
        only while power flows, and ESR only shows as an extra voltage drop under load.
        No-ESR pulses are the reference. Returns the number of CGP received."""
        c = self.cfg
        f = self.need_cells()
        if min(f) < c["pulse_min_cell_mv"]:
            raise Skip(f"cells {f} too low for a load pulse (>= {c['pulse_min_cell_mv']} mV)")
        self.ready_for_power()
        target = max(100, min(950, int(min(self.tel.cells or f)) - 100))
        n0 = getattr(self, "loaded_n", 0)
        self.arm()
        if cell is not None:              # the firmware ends an ESR probe after esr_s (4 s): keep this short
            t_esr = time.monotonic()
            ok, text = self.cmd("ESR", cell, 1)
            if not ok:
                self.off()
                self.fault_event("FAULT_REJECTED", "ESR", cell, {"reason": text})
                raise Skip(f"ESR refused: {text}")
            self.fault_req = {"type": "ESR", "cell_id": cell, "active": True, "value": 1,
                              "purpose": "injection", "episode_id": (self.episode or {}).get("id")}
            self.wait(1.0, until=lambda: self.tel.event_since(t_esr, "ESR_START") is not None)
            self.wait(c["pulse_settle_s"])     # relay contact settles before the load
        else:
            self.fault_req = {"type": "NONE", "cell_id": None, "active": False, "purpose": "control_load"}
        t0 = time.monotonic()
        try:
            # the Nano sends CGP only with the health log on and stage 3 (load); SYNC clears both
            self.cmd("HLOG", 1)
            self.cmd("HMARK", 3)
            ok, text = self.cmd("DRAIN", target)
            if not ok:
                raise Skip(f"load pulse DRAIN refused: {text}")
            self.wait(1.5, guard=self.spread_guard, until=lambda: self.tel.event_since(t0, "DRAIN_START") is not None)
            self.wait(c["pulse_s"], guard=self.spread_guard)
        finally:
            self.off()
            for name, arg in (("HMARK", 0), ("HLOG", 0)):
                try:
                    self.nano.command(name, arg)
                except LinkError:
                    pass
        self.wait(1.5, until=lambda: self.tel.event_since(t0, "DRAIN_STOP") is not None)
        got = getattr(self, "loaded_n", 0) - n0
        if got < 2:
            self.notes["pulse_few_cgp"] += 1
        return got

    def stimulus(self, kind: str, cell: int, seconds=None, pulses=None, reference=True) -> None:
        """One actuator probe. `seconds` (leak/heat) stops it early; None = full firmware limit.
        ESR = a no-ESR reference pulse (unless reference=False), then `pulses` load pulses with ESR on `cell`."""
        self.phase = f"stim_{kind}"
        rt = self.tel.runtime or {}
        if kind == "control":
            for _ in range(int(self.cfg["control_pulses"])):
                self.phase = "control_load"
                self.load_pulse(None)
                self.wait(4)
            self.phase = "stim_control"
            self.fault_req = {"type": "NONE", "cell_id": None, "active": False}
            self.wait(20)
            self.fault_req = None
            return
        name, bit = FEATURE[kind]
        if kind == "esr":
            if rt.get("profile") != 1 or not (rt.get("enabled", 0) >> bit) & 1:
                raise Skip(f"{name} disabled by Nano runtime config {rt}")
            if reference:
                self.load_pulse(None)          # reference right before
                self.wait(3)
            self._note_applied("esr", cell, pulses=0)
            rec = self.episode["applied"][-1] if self.episode else {}
            for k in range(int(pulses or self.cfg["esr_pulses"])):
                self.phase = "esr_injection"
                self.load_pulse(cell)
                rec["pulses"] = k + 1
                self.phase = "stim_esr"
                self.wait(4)
            return
        name, bit = FEATURE[kind]
        if rt.get("profile") != 1 or not (rt.get("enabled", 0) >> bit) & 1:
            raise Skip(f"{name} disabled by Nano runtime config {rt}")
        f = self.need_cells()
        if min(f) < 320 or max(f) > 980:
            raise Skip(f"cells {f} outside 320..980 mV for {name}")
        s = self.tel.spread()
        if s is None or s > self.cfg["start_spread_max_mv"]:
            self.balance("balance_pre")
            s = self.tel.spread()
            if s is None or s > self.cfg["start_spread_max_mv"]:
                raise Skip(f"spread {s} mV too high for {name}")
        value, limit_s = 1, rt.get(f"{kind}_s", 60)
        seconds = limit_s if seconds is None else min(limit_s, seconds)
        start_temp = self.tel.temps[cell - 1]
        if kind == "heat":
            temps = self.tel.temps
            if any(t is None for t in temps) or max(temps) >= self.cfg["heat_start_max_c"]:
                raise Skip(f"heat needs valid temperatures under {self.cfg['heat_start_max_c']} C: {temps}")
            value = min(32, int(rt.get("heat_pwm", 32)))

        def guard():
            if kind == "heat":
                t = self.tel.temps[cell - 1]
                if t is None or t >= self.cfg["heat_guard_c"]:
                    self.off()
                    raise Skip(f"heat guard: cell {cell} at {t} C")
            if kind == "leak":
                g = self.tel.filtered()
                if g is not None and min(g) < 300:
                    self.off()
                    raise Skip("leak: lowest cell under 300 mV")
            self.spread_guard()

        self.arm()
        t0 = time.monotonic()
        ok, text = self.cmd(name, cell, value)
        if not ok:
            self.off()
            self.fault_event("FAULT_REJECTED", name, cell, {"reason": text})
            raise Skip(f"{name} refused: {text}")
        self.fault_req = {"type": name, "cell_id": cell, "active": True, "value": value,
                          "purpose": "injection", "episode_id": (self.episode or {}).get("id")}
        self._note_applied(kind, cell, seconds=seconds)
        try:
            self.wait(seconds + (4 if seconds >= limit_s else 0), guard=guard,
                      until=lambda: self.idle_after(t0, (f"{name}_STOP",)))
        finally:
            self.off()
        if self.episode is not None:
            return                             # fault mode: keep the effect, next fault overlaps it
        if kind == "heat" and start_temp is not None:
            self.phase = "heat_cooldown"
            self.wait(self.cfg["heat_cooldown_max_s"],
                      until=lambda: (self.tel.temps[cell - 1] or 99) <= start_temp + 1.0)

    def _note_applied(self, kind: str, cell: int, **extra) -> None:
        ep = self.episode
        if ep is not None:
            ep["applied"].append({"kind": kind, "cell": cell, "t": time.monotonic(), **extra})

    # -- fault mode
    def plan_episode(self, rng: random.Random) -> dict:
        c = self.cfg
        w = c["fault_weights"]
        shapes = [s for s in ("none", "single", "compound") if w.get(s, 0) > 0]
        shape = rng.choices(shapes, weights=[w[s] for s in shapes])[0]
        kinds = list(c["fault_kinds"])
        if shape == "none" or not kinds:
            picks = []
        elif shape == "single" or len(kinds) < 2:
            picks = [rng.choice(kinds)]
        else:
            n = 3 if rng.random() < c["fault_triple_p"] and len(kinds) >= 3 else 2
            picks = rng.sample(kinds, n)
        same = rng.random() < c["fault_same_cell_p"]
        cell0 = rng.randint(1, 4)
        picks.sort(key=lambda k: k != "heat")      # heat first: its temperature rise outlasts the probe
        faults = []
        for k in picks:
            f = {"kind": k, "cell": cell0 if same else rng.randint(1, 4)}
            if k == "esr":
                f["pulses"] = rng.randint(2, 3)
            else:
                f["fraction"] = round(rng.uniform(c["fault_partial_min"], 1.0), 2)
            faults.append(f)
        return {"shape": shape, "faults": faults, "target_mv": rng.choice(c["charge_targets_mv"])}

    def fault_episode(self) -> None:
        c = self.cfg
        if time.monotonic() - self.run_start > c["run_rotate_hours"] * 3600:
            self._new_run()
        n = int(self.persist.get("episode_index", 0))
        plan = self.plan_episode(random.Random())
        ep = {"id": f"{self.run_id}:e{n:05d}", "plan": plan, "applied": [], "detections": [], "skipped": [],
              "t0": time.monotonic(), "start_temps": list(self.tel.temps)}
        self.cycle_id = ep["id"]
        self.episode = ep
        log(f"episode {n}: {plan['shape']} {plan['faults']}")
        self.diag("EPISODE_START", f"episode {n}: {plan['shape']}", episode_id=ep["id"], **plan)
        outcome = "aborted"
        try:
            self.off()
            self.start_session()
            self.observe("baseline", c["episode_baseline_s"])
            ep["start_temps"] = list(self.tel.temps)
            for step, fn in (("charge", lambda: self.charge_to(plan["target_mv"])),
                             ("hold", lambda: self.observe("hold", c["episode_hold_s"]))):
                try:
                    fn()
                except Skip as e:
                    self.off()
                    ep["skipped"].append({"step": step, "why": str(e)})
            rt = self.tel.runtime or {}
            ep["overlap"] = len(plan["faults"]) > 1 and c["fault_overlap"] and self.fw_concurrent()
            if ep["overlap"]:
                try:
                    self.compound_overlap(plan["faults"])
                except Skip as e:
                    self.off()
                    ep["skipped"].append({"step": "compound", "why": str(e)})
            for f in [] if ep["overlap"] else plan["faults"]:
                try:
                    if f["kind"] == "esr":
                        self.stimulus("esr", f["cell"], pulses=f["pulses"])
                    else:
                        self.stimulus(f["kind"], f["cell"],
                                      seconds=max(5, int(rt.get(f"{f['kind']}_s", 60) * f["fraction"])))
                except Skip as e:
                    self.off()
                    ep["skipped"].append({"step": f"{f['kind']}@{f['cell']}", "why": str(e)})
            self.observe("post", c["episode_post_s"] + (60 if not plan["faults"] else 0))
            heated = [a["cell"] for a in ep["applied"] if a["kind"] == "heat"]
            if heated:
                self.phase = "heat_cooldown"
                base = ep["start_temps"]
                self.wait(c["heat_cooldown_max_s"], until=lambda: all(
                    (self.tel.temps[k - 1] or 99) <= (base[k - 1] if base[k - 1] is not None else 30) + 1.0
                    for k in heated))
            for step, fn in (("balance", lambda: self.balance("balance_top")),
                             ("drain", lambda: self.drain_to(c["rest_mv"]))):
                try:
                    fn()
                except Skip as e:
                    self.off()
                    ep["skipped"].append({"step": step, "why": str(e)})
            self.off()
            self.observe("rest", c["episode_rest_s"])
            outcome = "completed"
        finally:
            self.episode = None
            self.persist["episode_index"] = n + 1
            self._save_state()
            self.diag("EPISODE_RESULT", f"episode {n} {outcome}", **self.score_episode(ep, outcome))

    # -- compound faults at the same time (Nano >= 0.2.0)
    def fw_concurrent(self) -> bool:
        """Nano 0.2.0+ runs ESR/LEAK/HEAT together (0.1.9: one actuator probe at a time)."""
        try:
            major, minor = (int(x) for x in self.tel.firmware.split("-")[0].split(".")[:2])
        except ValueError:
            return False
        return (major, minor) >= (0, 2)

    def compound_overlap(self, faults: list) -> None:
        """All faults of the episode at once. LEAK/HEAT start together and stay on while the ESR
        load pulses run; a pulse ends with DISCHARGE 0 + ESR n 0, never OFF (OFF ends every fault).
        Each LEAK/HEAT stops at its own planned time; the firmware times each one out by itself too."""
        c, rt, ep = self.cfg, self.tel.runtime or {}, self.episode
        self.phase = "compound"
        if rt.get("profile") != 1:
            raise Skip(f"Nano runtime profile is not OPERATE: {rt}")
        todo = []
        for f in faults:
            name, bit = FEATURE[f["kind"]]
            if (rt.get("enabled", 0) >> bit) & 1:
                todo.append(f)
            else:
                ep["skipped"].append({"step": f"{f['kind']}@{f['cell']}",
                                      "why": f"{name} disabled by Nano runtime config"})
        if not todo:
            return
        cells = self.need_cells()
        if min(cells) < 320 or max(cells) > 980:
            raise Skip(f"cells {cells} outside 320..980 mV for compound faults")
        heated = [f["cell"] for f in todo if f["kind"] == "heat"]
        if heated:
            temps = self.tel.temps
            if any(t is None for t in temps) or max(temps) >= c["heat_start_max_c"]:
                raise Skip(f"heat needs valid temperatures under {c['heat_start_max_c']} C: {temps}")
        esr = next((f for f in todo if f["kind"] == "esr"), None)
        if esr is not None:
            self.load_pulse(None)              # no-ESR reference right before (balances if needed)
            self.wait(3)
        else:
            self.ready_for_power()             # same spread rule as a single LEAK/HEAT
        leaking = any(f["kind"] == "leak" for f in todo)

        def guard():
            for k in heated:
                t = self.tel.temps[k - 1]
                if t is None or t >= c["heat_guard_c"]:
                    self.off()
                    raise Skip(f"heat guard: cell {k} at {t} C")
            if leaking:
                g = self.tel.filtered()
                if g is not None and min(g) < 300:
                    self.off()
                    raise Skip("leak: lowest cell under 300 mV")
            self.spread_guard()

        self.fault_req = {"type": "COMPOUND", "active": True, "purpose": "injection", "episode_id": ep["id"],
                          "faults": [{"type": FEATURE[f["kind"]][0], "cell_id": f["cell"]} for f in todo]}
        ends = []                              # [t_end, NAME, cell] of each LEAK/HEAT still on

        def stop_due():
            now = time.monotonic()
            for e in [e for e in ends if e[0] <= now]:
                ends.remove(e)
                self.cmd(e[1], e[2], 0)        # ALREADY if the firmware timed it out first

        try:
            self.arm()
            for f in todo:
                if f["kind"] == "esr":
                    continue
                kind, cell = f["kind"], f["cell"]
                name = FEATURE[kind][0]
                limit = int(rt.get(f"{kind}_s", 60))
                secs = min(limit, max(5, int(limit * f.get("fraction", 1.0))))
                value = min(32, int(rt.get("heat_pwm", 32))) if kind == "heat" else 1
                ok, text = self.cmd(name, cell, value)
                if not ok:
                    self.fault_event("FAULT_REJECTED", name, cell, {"reason": text, "overlap": True})
                    ep["skipped"].append({"step": f"{kind}@{cell}", "why": f"{name} refused: {text}"})
                    continue
                self._note_applied(kind, cell, seconds=secs, overlap=True)
                ends.append([time.monotonic() + secs, name, cell])
            if esr is not None:
                self._note_applied("esr", esr["cell"], pulses=0, overlap=True)
                rec = ep["applied"][-1]
                for k in range(int(esr.get("pulses") or c["esr_pulses"])):
                    stop_due()
                    self.phase = "esr_injection"
                    self.overlap_pulse(esr["cell"], guard)
                    rec["pulses"] = k + 1
                    self.phase = "compound"
                    self.wait(4, guard=guard)
            while ends:
                self.wait(max(0.0, min(e[0] for e in ends) - time.monotonic()), guard=guard)
                stop_due()
        finally:
            self.off()

    def overlap_pulse(self, cell: int, guard) -> int:
        """One ESR load pulse while other faults stay on (see compound_overlap). Every finished
        DRAIN disarms the Nano, so each pulse arms again. Returns the number of CGP received."""
        c = self.cfg
        f = self.need_cells()
        if min(f) < c["pulse_min_cell_mv"]:
            raise Skip(f"cells {f} too low for a load pulse (>= {c['pulse_min_cell_mv']} mV)")
        target = max(100, min(950, int(min(self.tel.cells or f)) - 100))
        n0 = getattr(self, "loaded_n", 0)
        self.arm()
        t_esr = time.monotonic()
        ok, text = self.cmd("ESR", cell, 1)
        if not ok:
            self.fault_event("FAULT_REJECTED", "ESR", cell, {"reason": text, "overlap": True})
            raise Skip(f"ESR refused: {text}")
        t0 = time.monotonic()
        try:
            self.wait(1.0, until=lambda: self.tel.event_since(t_esr, "ESR_START") is not None)
            self.wait(c["pulse_settle_s"], guard=guard)
            self.cmd("HLOG", 1)
            self.cmd("HMARK", 3)
            t0 = time.monotonic()
            ok, text = self.cmd("DRAIN", target)
            if not ok:
                raise Skip(f"load pulse DRAIN refused: {text}")
            self.wait(1.5, guard=guard, until=lambda: self.tel.event_since(t0, "DRAIN_START") is not None)
            self.wait(c["pulse_s"], guard=guard)
        finally:
            for name, args in (("DISCHARGE", (0,)), ("ESR", (cell, 0)), ("HMARK", (0,)), ("HLOG", (0,))):
                try:
                    self.nano.command(name, *args)
                except LinkError:
                    pass
        self.wait(1.5, until=lambda: self.tel.event_since(t0, "DRAIN_STOP") is not None)
        got = getattr(self, "loaded_n", 0) - n0
        if got < 2:
            self.notes["pulse_few_cgp"] += 1
        return got

    @staticmethod
    def score_episode(ep: dict, outcome: str) -> dict:
        """Which applied faults the AI caught (and how fast), and what it flagged with nothing applied."""
        t0 = ep["t0"]
        dets = list(ep["detections"])
        faults = []
        for a in ep["applied"]:
            if a.get("pulses") == 0:                 # ESR refused before the first pulse
                continue
            ftype = FEATURE[a["kind"]][0]
            hit = next((d for d in dets if d["cell"] == a["cell"] and d["t"] >= a["t"] - 1
                        and d["type"] in (ftype, "ANOMALY")), None)
            faults.append({"kind": a["kind"], "type": ftype, "cell": a["cell"], "detected": hit is not None,
                           "model": hit and hit["model"],
                           "latency_s": None if hit is None else round(hit["t"] - a["t"], 1),
                           **{k: a[k] for k in ("seconds", "pulses") if k in a}})
        return {"episode_id": ep["id"], "outcome": outcome, "shape": ep["plan"]["shape"], "plan": ep["plan"],
                "overlap": bool(ep.get("overlap")),
                "skipped": ep["skipped"], "faults": faults,
                "detected": sum(f["detected"] for f in faults), "applied": len(faults),
                "false_alarms": [{"model": d["model"], "type": d["type"], "cell": d["cell"],
                                  "t_s": round(d["t"] - t0, 1)} for d in dets if d["match"] == "none"],
                "detections": [{"model": d["model"], "type": d["type"], "cell": d["cell"], "match": d["match"],
                                "t_s": round(d["t"] - t0, 1)} for d in dets],
                "duration_s": round(time.monotonic() - t0, 1)}

    # -- demo mode (시연)
    def plan_demo(self, rng: random.Random) -> dict:
        c = self.cfg
        kinds = list(c["demo_fault_kinds"])
        w = c.get("demo_fault_weights") or {}
        kind = rng.choices(kinds, weights=[float(w.get(k, 1.0)) for k in kinds])[0]
        return {"shape": "single", "kind": kind, "cell": rng.randint(1, 4), "target_mv": int(c["demo_target_mv"]),
                "normal_s": round(rng.uniform(c["demo_normal_min_s"], c["demo_normal_max_s"]), 1)}

    def _ai_cutoff(self, d: dict) -> None:
        """Main thread, from tick(): the AI confirmed a fault while the demo cutoff was armed -> TRIP."""
        hit = d["hit"]
        d["cut"] = now = time.monotonic()
        inj = next((a for a in d["ep"]["applied"]), None)
        report = {"model": hit.get("model"), "version": hit.get("version"), "match": hit.get("match"),
                  "episode_id": d["ep"]["id"],
                  "injected": None if inj is None else {"type": FEATURE[inj["kind"]][0], "cell": inj["cell"]},
                  "detect_latency_s": None if inj is None else round(hit["t"] - inj["t"], 1),
                  "cutoff_latency_s": round(now - hit["t"], 2),
                  "scores": {k: hit[k] for k in ("score", "score_v", "score_t", "prob", "z") if k in hit}}
        self.ai_cutoff = {"at": now, "fault_type": hit.get("fault_type"), "cell": hit.get("cell"), "report": report}
        self.diag("AI_CUTOFF", f"{hit.get('model')} {hit.get('fault_type')} cell {hit.get('cell')}: TRIP",
                  fault_type=hit.get("fault_type"), cell=hit.get("cell"), **report)
        try:
            ok, text = self.nano.command("TRIP", timeout=0.6, tries=3)
        except LinkError as e:
            ok, text = False, repr(e)
        if not ok:                             # never leave the fault running: at least everything off
            self.off("ai cutoff")
            self.ai_cutoff = None
            self.diag("AI_CUTOFF_FAILED", f"TRIP not acknowledged: {text}; outputs switched off instead")
            raise Skip("AI cutoff: TRIP not acknowledged")
        raise Tripped("HOST_TRIP")

    def demo_show(self, forced: dict | None = None) -> None:
        """One show: prepare (cool down, session, charge) -> normal operation for a random time ->
        one random fault -> watch. The AI cutoff is armed from normal operation to the end of the watch."""
        c = self.cfg
        if time.monotonic() - self.run_start > c["run_rotate_hours"] * 3600:
            self._new_run()
        n = int(self.persist.get("demo_index", 0))
        plan = self.plan_demo(random.Random())
        if forced is not None:             # operator asked for this exact fault: short normal run, then inject
            plan = {**plan, "kind": forced["kind"], "cell": int(forced["cell"]), "normal_s": 8.0}
        ep = {"id": f"{self.run_id}:d{n:05d}", "plan": plan, "applied": [], "detections": [], "skipped": [],
              "t0": time.monotonic(), "start_temps": list(self.tel.temps)}
        self.cycle_id = ep["id"]
        log(f"demo {n}: {plan}")
        # the plan (which fault, where, when) stays on the Pi until DEMO_RESULT: the server learns of
        # the fault the way an operator would, from the AI cutoff (telemetry still carries the masks)
        self.diag("DEMO_START", f"demo {n}", episode_id=ep["id"], target_mv=plan["target_mv"],
                  **({"forced": True, "kind": plan["kind"], "cell": plan["cell"],
                      "ai_cutoff": bool(forced.get("ai_cutoff", True))} if forced else {}))
        outcome = "aborted"
        try:
            self.off()
            hot = c["heat_start_max_c"] - 1.0
            if not self.temps_ok(-50, hot):
                self.phase = "demo_cooldown"
                self.wait(c["heat_cooldown_max_s"], until=lambda: self.temps_ok(-50, hot))
            self.phase = "demo_prep"
            self.start_session()
            try:
                self.charge_to(plan["target_mv"])
                s = self.tel.spread()
                if s is not None and s > c["start_spread_max_mv"]:
                    self.balance("balance_pre")
            except Skip as e:
                self.off()
                ep["skipped"].append({"step": "prep", "why": str(e)})
            self.off()
            self.wait(3)
            ep["start_temps"] = list(self.tel.temps)
            self.episode = ep
            self.demo = {"ep": ep, "hit": None, "cut": None,
                         "cutoff": True if forced is None else bool(forced.get("ai_cutoff", True))}
            self.diag("DEMO_ARMED", "AI cutoff armed", episode_id=ep["id"])
            self.phase = "demo_operating"
            self.wait(plan["normal_s"])
            kind, cell = plan["kind"], plan["cell"]
            try:
                if kind == "esr":
                    # no reference pulse: it is a dataset label aid, not "normal operation", and load_esr v2
                    # flags cell 4 on it (z 4-5 with no ESR active), which would cut off before the fault
                    self.stimulus("esr", cell, pulses=c["demo_esr_pulses"], reference=False)
                else:
                    self.stimulus(kind, cell)          # up to the firmware limit; the cutoff ends it early
            except Skip as e:
                self.off()
                ep["skipped"].append({"step": f"{kind}@{cell}", "why": str(e)})
            self.phase = "demo_watch"
            self.wait(c["demo_watch_s"])
            outcome = "missed" if ep["applied"] else "not_injected"
        except Tripped:
            d = self.demo
            outcome = "cutoff" if d is not None and d["cut"] is not None else "firmware_trip"
            raise
        finally:
            d, self.demo, self.episode = self.demo, None, None
            self.off()
            self.persist["demo_index"] = n + 1
            self._save_state()
            hit = d and d["hit"]
            res = self.score_episode(ep, outcome)
            res["cutoff"] = None if not (d and d["cut"]) else {
                k: hit.get(k) for k in ("model", "version", "fault_type", "cell", "match")}
            if res["cutoff"] is not None and ep["applied"]:
                res["cutoff"]["detect_latency_s"] = round(hit["t"] - ep["applied"][0]["t"], 1)
            elif res["cutoff"] is not None:
                res["cutoff"]["before_injection"] = True
            self.diag("DEMO_RESULT", f"demo {n} {outcome}", **res)
        self.phase = "demo_rest"
        self.wait(c["demo_rest_s"])

    def cycle(self) -> None:
        c = self.cfg
        if time.monotonic() - self.run_start > c["run_rotate_hours"] * 3600:
            self._new_run()
        i = int(self.persist.get("cycle_index", 0))
        stimuli, targets = c["stimuli"], c["charge_targets_mv"]
        kind = stimuli[i % len(stimuli)]
        cell = (i // len(stimuli)) % 4 + 1
        target = targets[i % len(targets)]
        self.cycle_id = f"{self.run_id}:c{i:05d}"
        log(f"cycle {i}: target {target} mV, stimulus {kind} cell {cell}")
        summary = {"index": i, "target_mv": target, "stimulus": kind, "cell": cell, "skipped": []}
        self.off()
        self.start_session()
        self.observe("baseline", c["baseline_s"])
        for step, fn in (("charge", lambda: self.charge_to(target)),
                         ("hold", lambda: self.observe("hold", c["hold_s"])),
                         ("stimulus", lambda: self.stimulus(kind, cell)),
                         ("post", lambda: self.observe("post", c["post_s"])),
                         ("balance", lambda: self.balance("balance_top")),
                         ("drain", lambda: self.drain_to(c["rest_mv"]))):
            try:
                fn()
            except Skip as e:
                self.off()
                summary["skipped"].append({"step": step, "why": str(e)})
                log(f"skip {step}: {e}")
        self.off()
        self.observe("rest", c["rest_s"])
        summary["end_cells_mv"] = self.tel.cells
        summary["end_spread_mv"] = self.tel.spread()
        self.diag("CYCLE_SUMMARY", f"cycle {i}", **summary)
        self.persist["cycle_index"] = i + 1
        self._save_state()

    # -- recovery
    def handle_trip(self, reason: str) -> None:
        self.phase = "trip_recovery"
        cut = self.ai_cutoff
        demo_cut = reason == "HOST_TRIP" and cut is not None and time.monotonic() - cut["at"] < 60
        if not demo_cut:                   # a demo cutoff is wanted: it must not pause the experiments
            self.trip_times.append(time.monotonic())
        self.off("trip")
        if reason in ("NONE", "UNKNOWN"):   # CGS can report TRIPPED a moment before CGX/CGE arrive
            self._passive_sleep(1)
            reason = self.trip_reason()
        self.notes[f"trip_{reason}"] += 1
        c = self.cfg
        if demo_cut:
            self.diag("TRIP_RECOVERY", f"AI cutoff (HOST_TRIP): outputs off, reset in {c['demo_recover_s']} s",
                      trip=reason, cause="AI_CUTOFF", cells=self.tel.cells, temps=self.tel.temps)
        else:
            self.diag("TRIP_RECOVERY", f"firmware trip {reason}: outputs off, waiting",
                      trip=reason, cells=self.tel.cells, temps=self.tel.temps)
        try:
            self._trip_wait(reason, demo_cut)
        finally:
            self.ai_cutoff = None

    def _trip_wait(self, reason: str, demo_cut: bool) -> None:
        c = self.cfg
        now = time.monotonic()
        if demo_cut:
            self._reset_after(now + c["demo_recover_s"])
            return
        recent = sum(1 for t in self.trip_times if now - t < 3600)
        self.charge_noresp = self.charge_noresp + 1 if reason == "CHARGE_NO_RESPONSE" else 0
        # once the charge path was found open, one more no-response is enough to pause again (longer)
        need = 1 if self.charger_pauses else c["charger_fail_pause_after"]
        if self.charge_noresp >= need:
            pause_s = min(c["charger_retry_max_s"], c["charger_retry_s"] * 2 ** self.charger_pauses)
            self.charger_pauses += 1
            self.charge_noresp = 0
            self.pause_reason = "CHARGE_PATH_NO_RESPONSE"
            self.pause_until = now + pause_s
            self.diag("CHARGE_PATH_NO_RESPONSE",
                      f"charge gave no voltage rise {need} time(s) in a row; "
                      f"check the charge path (+5V_Cell -> Q3 -> R17 -> H4 -> RLY2 -> FH1 -> stack top). Experiments paused {pause_s} s (telemetry continues); "
                      "'cellguard-edge resume' retries now",
                      cells=self.tel.cells, pause_s=pause_s, pause_count=self.charger_pauses)
            log(f"charge path not responding: experiments paused {pause_s} s")
        elif reason != "CHARGE_NO_RESPONSE" and recent >= c["trips_per_hour_pause"]:
            self.pause_reason = f"{recent} trips in the last hour"
            self.pause_until = now + c["trip_pause_s"]
            self.diag("TRIP_PAUSE", f"{recent} trips in 1 h: experiments paused {c['trip_pause_s']} s "
                      "(telemetry continues)")
        self._reset_after(time.monotonic() + c["trip_wait_s"])

    def _reset_after(self, end: float) -> None:
        """Telemetry only until `end`, then RESET the trip (retried with back-off while the Nano refuses)."""
        backoff = 0
        while True:
            left = end - time.monotonic()
            if left > 0:
                self._passive_sleep(min(left, 30))
                continue
            try:
                ok, text = self.nano.command("RESET")
            except NoSync:
                self.nano.sync()
                continue
            if ok or text == "NOT_TRIPPED":
                self.tel.last_trip = None
                self.diag("TRIP_RESET", "trip cleared; next cycle starts with SELFTEST/SESSION")
                settle = time.monotonic() + 6
                while time.monotonic() < settle:
                    self._passive_sleep(1)
                    cgs_ok = self.tel.nano_state not in ("TRIPPED", "UNKNOWN")
                    if cgs_ok and not self.tel.trip:
                        break
                self.last_off = time.monotonic()
                return
            backoff = min(600, (backoff or 30) * 2)
            self.diag("TRIP_RESET_BLOCKED", text, retry_s=backoff, cells=self.tel.cells, temps=self.tel.temps)
            end = time.monotonic() + backoff

    def _passive_sleep(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.release_early()
            self.outbox.flush()
            self.raw.flush()
            self.mode()                    # remote commands, OTA / reload requests
            if time.monotonic() - getattr(self, "_status_at", 0) > 2:
                self._status_at = time.monotonic()
                self._write_status()
                sd_notify("WATCHDOG=1")
            if time.monotonic() - self.nano.last_rx > 15:
                raise LinkError("no frames from Nano for 15 s")
            time.sleep(0.5)

    def connect(self) -> None:
        delay = 2
        while True:
            try:
                self.nano.open()
                self.nano.sync()
                for name, args in (("OFF", ()), ("HLOG", (0,)), ("FLAGS", ()), ("CAL", ()), ("STATUS", ())):
                    self.nano.command(name, *args)
                self.last_off = time.monotonic()
                self.wait_first_sample()
                self.diag("LINK_UP", "Nano synced", firmware=self.tel.firmware, runtime=self.tel.runtime,
                          calibration_id=self.tel.calibration_id)
                return
            except (LinkError, OSError) as e:
                self.last_error = f"link: {e}"
                log(f"link setup failed: {e}; retry in {delay} s")
                self.nano.close()
                self._sleep_plain(delay)
                delay = min(60, delay * 2)

    def wait_first_sample(self) -> None:
        end = time.monotonic() + 5
        while time.monotonic() < end and self.tel.cgt_at < time.monotonic() - 3:
            time.sleep(0.2)

    def _sleep_plain(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.release_early()
            self.outbox.flush()
            sd_notify("WATCHDOG=1")
            time.sleep(0.5)

    def run(self) -> None:
        self.pub.start()
        if self.ai is not None:
            self.ai.start()
            threading.Thread(target=self._ota_loop, daemon=True, name="ota").start()
        self.connect()
        threading.Thread(target=self._heartbeat, daemon=True, name="heartbeat").start()
        sd_notify("READY=1")
        self.diag("SERVICE_START", f"cellguard-edge {VERSION}", mode=self.mode(),
                  cycle_index=self.persist.get("cycle_index", 0),
                  episode_index=self.persist.get("episode_index", 0),
                  ai_models=None if self.ota is None else self.ota.versions(), ai_disabled=self.ai_disabled,
                  mlops=None if self.ota is None or self.ota.http is None else self.ota.http.base)
        link_failures = 0
        while True:
            try:
                if self.tel.trip or self.tel.nano_state == "TRIPPED":
                    raise Tripped(self.trip_reason())
                mode = self.mode()
                once = None if time.monotonic() < self.pause_until else self._pending_once()
                if once is not None:
                    self.run_once(once)
                    continue
                if mode == "passive" or time.monotonic() < self.pause_until:
                    if self.phase != "passive":
                        self.off("passive")
                    self.phase = "passive"
                    self.running_mode = "passive"
                    self.cycle_id = None
                    self._passive_sleep(5)
                    continue
                if mode != self.running_mode:
                    self.diag("MODE_START", f"{mode} mode", mode=mode, previous=self.running_mode)
                self.running_mode = mode
                if mode == "fault":
                    self.fault_episode()
                elif mode == "demo":
                    self.demo_show()
                elif mode == "hold":
                    self.hold_mode()
                elif mode == "discharge":
                    self.discharge_mode()
                else:
                    self.cycle()
                link_failures = 0
            except Tripped as e:
                try:
                    self.handle_trip(str(e))
                except LinkError as le:
                    self._link_recover(le, link_failures)
                    link_failures += 1
            except Skip as e:
                self.notes["cycle_skip"] += 1
                self.diag("CYCLE_SKIPPED", str(e), cells=self.tel.cells, temps=self.tel.temps,
                          spread_mv=self.tel.spread())
                try:
                    self.off()
                    self._passive_sleep(60)
                except LinkError as le:
                    self._link_recover(le, link_failures)
                    link_failures += 1
            except Paused:
                self.off("pause")
                self.phase = "passive"
            except LinkError as e:
                self._link_recover(e, link_failures)
                link_failures += 1
            except Exception as e:
                self.last_error = repr(e)
                self.diag("HOST_ERROR", repr(e), traceback=traceback.format_exc()[-2000:])
                try:
                    self.off("error")
                except Exception:
                    pass
                self._sleep_plain(10)

    def _link_recover(self, e: Exception, failures: int) -> None:
        self.last_error = f"link: {e}"
        self.notes["link_recover"] += 1
        log(f"link problem: {e}")
        try:
            if failures >= 2:
                raise LinkError("reopen")
            self.nano.sync()
            self.nano.command("OFF")
            self.last_off = time.monotonic()
            self._sleep_plain(2)
            if time.monotonic() - self.nano.last_rx > 5:
                raise LinkError("still silent")
            self.diag("LINK_RESYNC", str(e))
        except LinkError:
            self.nano.close()
            self._sleep_plain(min(60, 5 * (failures + 1)))
            self.connect()


# --------------------------------------------------------------------------- systemd notify
def sd_notify(state: str) -> None:
    addr = os.environ.get("NOTIFY_SOCKET")
    if not addr:
        return
    if addr.startswith("@"):
        addr = "\0" + addr[1:]
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as s:
            s.connect(addr)
            s.sendall(state.encode())
    except OSError:
        pass


def normalize_mode(mode) -> str:
    if mode == "auto" or mode not in MODES:     # "auto" = v2 experiments = collect
        return "collect"
    return mode


def read_control(data: Path) -> dict:
    try:
        ctl = json.loads((data / "control.json").read_text())
        return ctl if isinstance(ctl, dict) else {}
    except (OSError, ValueError):
        return {}


def write_control(data: Path, **updates) -> dict:
    data.mkdir(parents=True, exist_ok=True)
    ctl = read_control(data)
    ctl.update(updates)
    tmp = data / "control.tmp"
    tmp.write_text(json.dumps(ctl))
    os.replace(tmp, data / "control.json")
    return ctl


def set_mode(data: Path, request: str) -> str:
    """collect | fault | passive | pause (= passive, remember the mode) | resume (= back to it)."""
    cur = normalize_mode(read_control(data).get("mode"))
    if request == "pause":
        extra = {"resume_mode": cur} if cur != "passive" else {}
        mode = "passive"
    elif request == "resume":
        mode = normalize_mode(read_control(data).get("resume_mode", "collect"))
        mode = "collect" if mode == "passive" else mode
        extra = {}
    else:
        mode = normalize_mode(request) if request != "passive" else "passive"
        extra = {"resume_mode": mode} if mode != "passive" else {}
    write_control(data, mode=mode, at=time.time(), **extra)
    return mode


def load_config(path: Path) -> dict:
    cfg = dict(DEFAULT_CONFIG)
    if path.exists():
        cfg.update(json.loads(path.read_text(encoding="utf-8")))
    return cfg


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["run", "status", "mode", "pause", "resume", "ota", "ota-install"])
    ap.add_argument("args", nargs="*")
    ap.add_argument("--config", default=str(HOME / "config.json"))
    ap.add_argument("--passive", action="store_true", help="telemetry only, never actuate")
    a = ap.parse_args()
    cfg = load_config(Path(a.config))
    data = Path(cfg["data_dir"])
    if a.command == "status":
        try:
            print((data / "status.json").read_text())
        except OSError:
            print("no status yet (service not running?)")
        return 0
    if a.command in ("mode", "pause", "resume"):
        req = a.command if a.command != "mode" else (a.args[0] if a.args else "")
        if req not in MODES + ("pause", "resume"):
            print(f"mode must be one of {', '.join(MODES)}")
            return 2
        mode = set_mode(data, req)
        print("mode ->", {"collect": "collect (systematic data collection cycle)",
                          "fault": "fault (random single/compound fault episodes, AI judged)",
                          "demo": "demo (normal-looking run, one hidden random fault, AI detection -> TRIP)",
                          "passive": "passive (telemetry only, outputs off)"}[mode])
        return 0
    if a.command == "ota":
        write_control(data, ota_at=time.time())
        print("OTA check requested (result: status -> ota, or diag OTA_CHECK / OTA_RESULT)")
        return 0
    if a.command == "ota-install":
        if len(a.args) != 2:
            print("usage: ota-install NAME PACKAGE.zip")
            return 2
        import cg_ai
        name, path = a.args
        if name not in cg_ai.MODEL_NAMES:
            print(f"model must be one of {', '.join(cg_ai.MODEL_NAMES)}")
            return 2
        res = cg_ai.Ota(Path(cfg["models_dir"]), cfg["pack_id"], None, None).install_file(name, Path(path))
        print(json.dumps(res, ensure_ascii=False))
        if res["action"] == "applied":
            write_control(data, reload_at=time.time())
        return 0 if res["action"] == "applied" else 1
    edge = Edge(cfg, a.passive)

    def stop(signum, frame):
        log(f"signal {signum}: outputs off, exiting")
        for _ in range(2):  # OFF is executed even as a stale seq; do not take the command lock here
            try:
                edge.nano._write(f"CGC,{(edge.nano.seq + 7) & 0xFFFF},OFF")
            except Exception:
                pass
            time.sleep(0.05)
        edge.outbox.flush()
        edge.raw.flush()
        os._exit(0)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    edge.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
