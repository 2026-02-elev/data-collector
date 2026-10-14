# CellGuard data collector - command line interface (cg-collect)
# Author: Yongho Ahn <nanhosoft@gmail.com>
from __future__ import annotations

import argparse
import contextlib
import datetime as dt
import json
import sys
from pathlib import Path
from typing import Iterator, Optional, Sequence

from .current import CurrentSampler, Ina219
from .events import EventRecorder
from .fusion import Collector, FusionContext, rfc3339
from .replay import read_collector_log, read_generator_log, replay
from .runner import PlanError, Pump, RunAborted, load_plan, parse_plan, run_steps, shell_command
from .session import CommandTimeout, NanoSession, Received
from .storage import Outbox, RotatingWriter
from .transport import SerialTransport, SimTransport, Transport

QUERY_COMMANDS = ("PING", "STATUS", "RAW", "CAL")


def _transport(a: argparse.Namespace) -> Transport:
    if a.sim:
        return SimTransport(a.sim, gates=a.sim_gates)
    return SerialTransport(a.port, a.baud)


@contextlib.contextmanager
def collection(a: argparse.Namespace, session: NanoSession) -> Iterator[Collector]:
    """Raw UART log (both directions) + normalized samples + fault events (+ INA219) for one run."""
    data_dir = Path(a.data_dir)
    clock = session.t.now_utc
    kw = dict(max_bytes=a.rotate_mb * 1_000_000, max_age_s=a.rotate_s, clock=clock)
    raw = RotatingWriter(data_dir, "raw", **kw)
    samples = RotatingWriter(data_dir, "samples", **kw)
    events = RotatingWriter(data_dir, "events", **kw)

    def log_rx(rec: Received) -> None:
        raw.write_record({"pi_received_utc": rfc3339(rec.utc), "dir": "N>P", "frame": rec.raw, "error": rec.error})

    def log_tx(utc: dt.datetime, frame: str) -> None:
        raw.write_record({"pi_sent_utc": rfc3339(utc), "dir": "P>N", "frame": frame})

    session.subscribe(log_rx)
    session.on_transmit(log_tx)
    sampler: Optional[CurrentSampler] = None
    if a.ina219:
        source = Ina219(bus=a.i2c_bus, address=int(a.i2c_address, 0), shunt_ohm=a.shunt_ohm, sign=a.current_sign)
        sampler = CurrentSampler(source, period_s=0.1)
        sampler.start()

    ctx = FusionContext(device_id=a.device_id, pack_id=a.pack_id, run_id=a.run_id, scenario_id=a.scenario_id,
                        live_gap_s=a.live_missing_s)

    def write_sample(rec: dict) -> None:
        samples.write_record(rec)
        if a.echo:
            print(f"{rec['timestamp_utc']} {rec['record_status']} {rec['controller_state']} "
                  f"cells={rec['cell_voltage_v']} I={rec['pack_current_a']} ({rec['current_quality']})")

    def write_event(rec: dict) -> None:
        events.write_record(rec)

    collector = Collector(session, ctx, write_sample, current=sampler, request_raw=not a.no_raw)
    EventRecorder(session, collector, write_event)
    # raw_log_ref always names the raw file being written at that moment
    session.subscribe(lambda _r: setattr(ctx, "raw_log_ref", raw.current_name or "raw-log-unset"))
    try:
        yield collector
    finally:
        collector.close()
        if sampler:
            sampler.stop()
        for w in (raw, samples, events):
            w.close()


def _replay(a: argparse.Namespace) -> int:
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    ctx = FusionContext(device_id=a.device_id, pack_id=a.pack_id, run_id=a.run_id or "replay",
                        scenario_id=a.scenario_id)
    end = None
    if a.from_generator:
        run = Path(a.from_generator)
        man = json.loads((run / "manifest.json").read_text())
        epoch = dt.datetime.fromisoformat(man["fixed_epoch_utc"].replace("Z", "+00:00"))
        ctx.run_id, ctx.scenario_id = a.run_id or man["run_id"], man["scenario_id"]
        lines = read_generator_log(run / "nano_uart.log", epoch)
        start, end = epoch, epoch + dt.timedelta(seconds=float(man["duration_s"]))
        ctx.raw_log_ref = str(run / "nano_uart.log")
    else:
        lines = read_collector_log([Path(x) for x in a.from_raw])
        start = None
        ctx.raw_log_ref = ",".join(Path(x).name for x in a.from_raw)
    with open(out / "samples.jsonl", "w", encoding="utf-8") as fs, open(out / "events.jsonl", "w",
                                                                          encoding="utf-8") as fe:
        col = replay(lines, ctx, lambda r: fs.write(json.dumps(r, separators=(",", ":")) + "\n"),
                     lambda r: fe.write(json.dumps(r, separators=(",", ":")) + "\n"), run_start=start, end_utc=end)
    print(json.dumps(col.counters), file=sys.stderr)
    return 0


def _last_state(session: NanoSession) -> str:
    from .protocol import Status

    for rec in reversed(session.recent):
        if isinstance(rec.message, Status):
            return f"{rec.message.state} trip={rec.message.trip} locks={','.join(rec.message.locks) or '-'}"
    return "unknown"


def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(prog="cg-collect", description="CellGuard Pi data collector (Nano UART protocol v1.0)")
    p.add_argument("--port", default="/dev/serial0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--sim", help="firmware host simulator binary (SIMULATED DEVICE)")
    p.add_argument("--sim-gates", default="locked", choices=["locked", "unlocked", "bench"])
    p.add_argument("--data-dir", default="data")
    p.add_argument("--device-id", default="cellguard-001")
    p.add_argument("--pack-id", default="pack-01")
    p.add_argument("--run-id", default=None)
    p.add_argument("--scenario-id", default="manual")
    p.add_argument("--rotate-mb", type=int, default=50)
    p.add_argument("--rotate-s", type=float, default=3600.0)
    p.add_argument("--echo", action="store_true", help="print a line per sample")
    p.add_argument("--no-raw", action="store_true", help="do not request CGD after each CGT (raw ADC fields null)")
    p.add_argument("--live-missing-s", type=float, default=None,
                   help="write MISSING_TELEMETRY slots live after this much silence (default: when telemetry resumes)")
    p.add_argument("--ina219", action="store_true", help="read pack current from the INA219 (I2C on the Pi)")
    p.add_argument("--i2c-bus", type=int, default=1)
    p.add_argument("--i2c-address", default="0x40")
    p.add_argument("--shunt-ohm", type=float, default=0.1, help="MEASURED shunt resistance of the module")
    p.add_argument("--current-sign", type=int, default=1, choices=[1, -1], help="-1 if installed reversed")
    sub = p.add_subparsers(dest="cmd", required=True)
    mon = sub.add_parser("monitor", help="read-only collection: SYNC, CAL, heartbeat, store samples")
    mon.add_argument("--seconds", type=float, default=0, help="0 = until Ctrl-C")
    run = sub.add_parser("run", help="execute an experiment plan (JSON/YAML) in one session while collecting")
    run.add_argument("plan")
    run.add_argument("--i-am-supervising", action="store_true")
    sh = sub.add_parser("shell", help="interactive control in one persistent session (heartbeat in background)")
    sh.add_argument("--i-am-supervising", action="store_true")
    q = sub.add_parser("query", help="one read-only command (PING/STATUS/RAW/CAL)")
    q.add_argument("name", choices=[c.lower() for c in QUERY_COMMANDS] + list(QUERY_COMMANDS))
    sub.add_parser("outbox", help="list closed files waiting for server upload")
    rp = sub.add_parser("replay", help="rebuild samples/events offline from a UART log")
    src = rp.add_mutually_exclusive_group(required=True)
    src.add_argument("--from-raw", nargs="+", help="this collector's raw-*.jsonl files (in order)")
    src.add_argument("--from-generator", help="a data-generator run directory (nano_uart.log + manifest.json)")
    rp.add_argument("--out", required=True, help="output directory (samples.jsonl, events.jsonl)")
    a = p.parse_args(argv)

    if a.cmd == "replay":
        return _replay(a)

    if a.cmd == "outbox":
        for path in Outbox(Path(a.data_dir)).ready():
            print(path)
        return 0

    t = _transport(a)
    session = NanoSession(t)
    a.run_id = a.run_id or f"run-{t.now_utc().strftime('%Y%m%dT%H%M%SZ')}"
    try:
        if a.cmd == "query":
            ack = session.command(a.name.upper())
            print(f"{'OK' if ack.ok else 'ERR'} {ack.text}")
            session.poll(0.1)
            for rec in list(session.recent)[-3:]:
                print(rec.raw)
            return 0 if ack.ok else 1

        if a.cmd == "monitor":
            with collection(a, session) as col:
                session.command("SYNC")
                session.command("CAL")
                try:
                    if a.seconds > 0:
                        session.pump(a.seconds)
                    else:
                        while True:
                            session.pump(60)
                except KeyboardInterrupt:
                    pass
                finally:
                    session.command("OFF")
                print(f"{col.count} samples -> {a.data_dir}", file=sys.stderr)
            return 0

        if a.cmd == "run":
            steps = parse_plan(load_plan(Path(a.plan)))
            with collection(a, session) as col:
                try:
                    results = run_steps(session, steps, supervised=a.i_am_supervising)
                    rc = 0
                except (RunAborted, CommandTimeout, KeyboardInterrupt) as exc:
                    print(f"run aborted: {exc}", file=sys.stderr)
                    results, rc = [], 1
                for r in results:
                    print(f"{r.op} {r.args} -> {'OK' if r.ok else 'ERR'} {r.text}")
                print(f"{col.count} samples -> {a.data_dir}", file=sys.stderr)
            return rc

        # shell
        with collection(a, session) as col:
            ack = session.command("SYNC")
            print(f"SYNC {'OK' if ack.ok else 'ERR'} {ack.text}")
            session.pump(1.0)  # let the first CGS arrive (the Nano may still be in SELF_TEST)
            print(f"state {_last_state(session)}", flush=True)
            pump = Pump(session)
            pump.start()
            try:
                for line in sys.stdin:
                    line = line.strip()
                    if line in ("quit", "exit"):
                        break
                    if pump.error:
                        print(f"pump stopped: {pump.error}")
                        break
                    print(shell_command(session, line, a.i_am_supervising), flush=True)
            except KeyboardInterrupt:
                pass
            finally:
                pump.stop()
                session.command("OFF")
            print(f"{col.count} samples -> {a.data_dir}", file=sys.stderr)
        return 0
    except PlanError as exc:
        print(f"plan error: {exc}", file=sys.stderr)
        return 2
    finally:
        t.close()


if __name__ == "__main__":
    sys.exit(main())
