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

from .current import CurrentSampler, CurrentSource, Ina219
from .cycle import CycleAborted, CycleRunner, validate_targets
from .cyclelog import import_cycle_log
from .diagnostics import DiagnosticRecorder
from .events import EventRecorder
from .fusion import Collector, FusionContext, rfc3339_us
from .preflight import FAIL, PASS, PreflightOptions, format_report, run_preflight, write_report
from .replay import read_collector_log, read_ina219_log, replay, replay_generator_run
from .runner import PlanError, Pump, RunAborted, load_plan, parse_plan, run_steps, shell_command
from .session import CommandTimeout, NanoSession, Received, UnsupportedCommand
from .storage import Outbox, RotatingWriter
from .transport import SerialTransport, SimTransport, Transport

QUERY_COMMANDS = ("PING", "STATUS", "RAW", "CAL")


def _transport(a: argparse.Namespace) -> Transport:
    if a.sim:
        return SimTransport(a.sim, gates=a.sim_gates)
    return SerialTransport(a.port, a.baud)


def _current_source(a: argparse.Namespace):
    """(source, error): the INA219 driver, or why it could not be opened."""
    if not a.ina219:
        return None, None
    try:
        return Ina219(bus=a.i2c_bus, address=int(a.i2c_address, 0), shunt_ohm=a.shunt_ohm, sign=a.current_sign), None
    except Exception as exc:  # no smbus2, no /dev/i2c-1, no device at the address ...
        return None, exc


def boot_check(a: argparse.Namespace, session: NanoSession, source: Optional[CurrentSource],
               error: Optional[BaseException]):
    """Runs the boot check, stores its report in the outbox and prints a summary."""
    opts = PreflightOptions(simulated=bool(a.sim), expect_map=a.expect_map, watch_s=a.boot_watch_s)
    meta = {"device_id": a.device_id, "pack_id": a.pack_id, "run_id": a.run_id,
            "port": "simulator" if a.sim else a.port, "command": a.cmd}
    report = run_preflight(session, Path(a.data_dir), opts, current_source=source, current_error=error, meta=meta)
    path = write_report(report, Path(a.data_dir))
    print(format_report(report), file=sys.stderr)
    print(f"report -> {path}", file=sys.stderr, flush=True)
    return report


@contextlib.contextmanager
def collection(a: argparse.Namespace, session: NanoSession, source: Optional[CurrentSource] = None
               ) -> Iterator[Collector]:
    """Raw UART log (both directions) + normalized samples + fault events (+ INA219) for one run."""
    data_dir = Path(a.data_dir)
    clock = session.t.now_utc
    kw = dict(max_bytes=a.rotate_mb * 1_000_000, max_age_s=a.rotate_s, clock=clock)
    raw = RotatingWriter(data_dir, "raw", **kw)
    samples = RotatingWriter(data_dir, "samples", **kw)
    events = RotatingWriter(data_dir, "events", **kw)
    diagnostics = RotatingWriter(data_dir, "diagnostics", **kw)
    run_start = clock()
    # first raw record: the run's time origin (t_sim_s = 0), so a replay reproduces t_sim_s
    raw.write_record({"dir": "RUN", "run_start_utc": rfc3339_us(run_start), "run_id": a.run_id,
                      "device_id": a.device_id, "pack_id": a.pack_id, "scenario_id": a.scenario_id})

    def log_rx(rec: Received) -> None:
        raw.write_record({"pi_received_utc": rfc3339_us(rec.utc), "dir": "N>P", "frame": rec.raw, "error": rec.error})

    def log_tx(utc: dt.datetime, frame: str) -> None:
        raw.write_record({"pi_sent_utc": rfc3339_us(utc), "dir": "P>N", "frame": frame})

    session.subscribe(log_rx)
    session.on_transmit(log_tx)
    sampler: Optional[CurrentSampler] = None
    ina_log: Optional[RotatingWriter] = None
    if source is not None:
        sampler = CurrentSampler(source, period_s=0.1)
        # every INA219 read (value, sample / availability time, quality, register) as raw evidence;
        # written only from the sampler thread
        ina_log = RotatingWriter(data_dir, "ina219", **kw)
        sampler.on_sample(lambda r: ina_log.write_record(r.to_raw_record()))
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

    # raw_log_ref always names the raw file being written at that moment (subscribed before the
    # collector, so a CGT's record names the file holding that CGT)
    session.subscribe(lambda _r: setattr(ctx, "raw_log_ref", raw.current_name or "raw-log-unset"))
    collector = Collector(session, ctx, write_sample, current=sampler, request_raw=not a.no_raw,
                          run_start_utc=run_start)
    EventRecorder(session, collector, write_event)
    # CGY / CGX and 0.1.6 diagnostic events: never samples, never fault events
    collector.diagnostics = DiagnosticRecorder(session, collector, diagnostics.write_record)  # type: ignore[attr-defined]
    try:
        yield collector
    finally:
        end = clock()
        collector.close(collector.t_of(end))
        # last raw record: where the run ended, so a replay writes the same trailing (missing) slots
        raw.write_record({"dir": "END", "run_end_utc": rfc3339_us(end)})
        if sampler:
            sampler.stop()
        for w in (raw, samples, events, diagnostics, ina_log):
            if w is not None:
                w.close()


def _replay(a: argparse.Namespace) -> int:
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    ctx = FusionContext(device_id=a.device_id, pack_id=a.pack_id, run_id=a.run_id or "replay",
                        scenario_id=a.scenario_id)
    fs = open(out / "samples.jsonl", "w", encoding="utf-8")
    fe = open(out / "events.jsonl", "w", encoding="utf-8")
    fd = open(out / "diagnostics.jsonl", "w", encoding="utf-8")
    sample_sink = lambda r: fs.write(json.dumps(r, separators=(",", ":")) + "\n")  # noqa: E731
    event_sink = lambda r: fe.write(json.dumps(r, separators=(",", ":")) + "\n")  # noqa: E731
    diag_sink = lambda r: fd.write(json.dumps(r, separators=(",", ":")) + "\n")  # noqa: E731
    try:
        if a.from_generator:
            run = Path(a.from_generator)
            man = json.loads((run / "manifest.json").read_text())
            ctx.run_id, ctx.scenario_id = a.run_id or man["run_id"], man["scenario_id"]
            ctx.raw_log_ref = str(run / "nano_uart.log")
            col = replay_generator_run(run, ctx, sample_sink, event_sink)
        else:
            if a.from_dir:
                d = Path(a.from_dir)
                raws = sorted(d.glob("raw-*.jsonl"))
                inas = sorted(d.glob("ina219-*.jsonl"))
            else:
                raws, inas = [Path(x) for x in a.from_raw], [Path(x) for x in (a.ina219_log or [])]
            if not raws:
                print("no raw-*.jsonl files", file=sys.stderr)
                return 2
            start, header = _run_header(raws[0])
            if header:
                ctx.run_id = a.run_id or header.get("run_id") or ctx.run_id
                ctx.device_id = header.get("device_id", ctx.device_id)
                ctx.pack_id = header.get("pack_id", ctx.pack_id)
                ctx.scenario_id = header.get("scenario_id", ctx.scenario_id)
            ctx.raw_log_ref = ",".join(p.name for p in raws)
            col = replay(read_collector_log(raws), ctx, sample_sink, event_sink, run_start=start,
                         end_utc=_run_footer(raws[-1]),
                         ina219=read_ina219_log(inas) if inas else None, diag_sink=diag_sink)
    finally:
        fs.close()
        fe.close()
        fd.close()
    print(json.dumps(col.counters), file=sys.stderr)
    return 0


def _run_footer(path: Path) -> Optional[dt.datetime]:
    """End of the run from the raw log's END record (None for logs written before it existed)."""
    last = None
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.strip():
                last = line
    try:
        r = json.loads(last) if last else {}
    except ValueError:
        return None
    if r.get("dir") != "END":
        return None
    return dt.datetime.fromisoformat(r["run_end_utc"].replace("Z", "+00:00"))


def _run_header(path: Path):
    with open(path, encoding="utf-8") as f:
        first = f.readline()
    try:
        r = json.loads(first)
    except ValueError:
        return None, None
    if r.get("dir") != "RUN":
        return None, None
    return dt.datetime.fromisoformat(r["run_start_utc"].replace("Z", "+00:00")), r


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
    p.add_argument("--boot-check", default="strict", choices=["strict", "warn", "off"],
                   help="boot check before monitor/run/shell: strict = do not start on FAIL (default), "
                        "warn = report and start anyway, off = skip")
    p.add_argument("--boot-watch-s", type=float, default=3.0, help="telemetry window of the boot check")
    p.add_argument("--expect-map", default="PCB2-20260918-R2", help="harness map version the firmware must report")
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
    st = sub.add_parser("selftest", help="test mode: check link, Nano self test, sensors, INA219, clock and storage")
    st.add_argument("--fail-on-warn", action="store_true", help="exit 1 on WARN as well")
    st.add_argument("--json", action="store_true", help="print the full report as JSON on stdout")
    cy = sub.add_parser("cycle", help="supervised staged charge (RAMP) / discharge (DRAIN) cycle while collecting "
                                      "(firmware 0.1.6-LOAD-VERIFY); targets are RESTED mV PER CELL")
    cy.add_argument("--charge-to", type=int, required=True, metavar="MV_PER_CELL")
    cy.add_argument("--discharge-to", type=int, metavar="MV_PER_CELL")
    cy.add_argument("--step", type=int, default=100, metavar="MV")
    cy.add_argument("--i-am-supervising", action="store_true")
    im = sub.add_parser("import-cycle-log", help="convert cellguard_cycle_*.log serial traces to samples/events/"
                                                 "diagnostics (no device needed)")
    im.add_argument("logs", nargs="+")
    im.add_argument("--out", required=True)
    im.add_argument("--log-tz", default="local", help="time zone of the trace's timestamps, e.g. Asia/Seoul")
    sub.add_parser("outbox", help="list closed files waiting for server upload")
    rp = sub.add_parser("replay", help="rebuild samples/events offline from a UART log")
    src = rp.add_mutually_exclusive_group(required=True)
    src.add_argument("--from-dir", help="a collector data directory's outbox (raw-*.jsonl + ina219-*.jsonl)")
    src.add_argument("--from-raw", nargs="+", help="this collector's raw-*.jsonl files (in order)")
    src.add_argument("--from-generator", help="a data-generator run directory (nano_uart.log + manifest.json)")
    rp.add_argument("--ina219-log", nargs="+", help="ina219-*.jsonl files to merge (with --from-raw)")
    rp.add_argument("--out", required=True, help="output directory (samples.jsonl, events.jsonl)")
    a = p.parse_args(argv)

    if a.cmd == "replay":
        return _replay(a)

    if a.cmd == "import-cycle-log":
        ctx = FusionContext(device_id=a.device_id, pack_id=a.pack_id,
                            run_id=a.run_id or "import-" + Path(a.logs[0]).stem, scenario_id=a.scenario_id)
        try:
            rep = import_cycle_log([Path(x) for x in a.logs], Path(a.out), ctx, a.log_tz)
        except ValueError as exc:
            print(f"import failed: {exc}", file=sys.stderr)
            return 2
        print(json.dumps({k: rep[k] for k in ("lines", "rx", "tx", "crc_or_format_errors", "unknown_types",
                                               "firmware_versions", "records")}, ensure_ascii=False))
        return 0

    if a.cmd == "cycle":
        try:
            validate_targets(a.charge_to, a.discharge_to, a.step)
        except ValueError as exc:
            p.error(str(exc))
        if not a.i_am_supervising:
            print("refused: cycle energises the rig - run it only with a person watching (--i-am-supervising)",
                  file=sys.stderr)
            return 2

    if a.cmd == "outbox":
        for path in Outbox(Path(a.data_dir)).ready():
            print(path)
        return 0

    t = _transport(a)
    session = NanoSession(t)
    a.run_id = a.run_id or f"run-{t.now_utc().strftime('%Y%m%dT%H%M%SZ')}"
    source, source_error = _current_source(a) if a.cmd != "query" else (None, None)
    try:
        if a.cmd == "selftest":
            report = boot_check(a, session, source, source_error)
            if a.json:
                print(json.dumps(report.to_record(), ensure_ascii=False, indent=1))
            return 0 if report.result == PASS or (report.result != FAIL and not a.fail_on_warn) else 1

        if a.cmd == "cycle":
            # always checked before energising, whatever --boot-check says; conservative go / no-go
            report = boot_check(a, session, source, source_error)
            if not report.actuation["allowed"]:
                print("cycle refused - nothing energised: " + "; ".join(report.actuation["reasons"]),
                      file=sys.stderr)
                return 1
            with collection(a, session, source) as col:
                runner = CycleRunner(session, col.diagnostics, log=lambda m: print(m, flush=True))
                try:
                    session.command("SYNC")  # every output off, fresh sequence
                    session.command("CAL")
                    runner.run(a.charge_to, a.discharge_to, a.step)
                    rc = 0
                except (CycleAborted, CommandTimeout, UnsupportedCommand) as exc:
                    print(f"ABORTED: {exc}", file=sys.stderr)
                    rc = 1
                except KeyboardInterrupt:
                    print("Interrupted. Nano has been sent OFF.", file=sys.stderr)
                    rc = 130
                print(f"{col.count} samples -> {a.data_dir}", file=sys.stderr)
            return rc

        if a.cmd in ("monitor", "run", "shell"):
            if a.boot_check != "off":
                report = boot_check(a, session, source, source_error)
                if report.result == FAIL and a.boot_check == "strict":
                    print("boot check FAILED - not starting (fix the FAIL items, or --boot-check warn)",
                          file=sys.stderr)
                    return 1
            if source_error is not None:
                print(f"INA219 unavailable ({source_error}): pack current will be null", file=sys.stderr)

        if a.cmd == "query":
            ack = session.command(a.name.upper())
            print(f"{'OK' if ack.ok else 'ERR'} {ack.text}")
            session.poll(0.1)
            for rec in list(session.recent)[-3:]:
                print(rec.raw)
            return 0 if ack.ok else 1

        if a.cmd == "monitor":
            with collection(a, session, source) as col:
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
            with collection(a, session, source) as col:
                try:
                    results = run_steps(session, steps, supervised=a.i_am_supervising)
                    rc = 0
                except (RunAborted, CommandTimeout, UnsupportedCommand, KeyboardInterrupt) as exc:
                    print(f"run aborted: {exc}", file=sys.stderr)
                    results, rc = [], 1
                for r in results:
                    print(f"{r.op} {r.args} -> {'OK' if r.ok else 'ERR'} {r.text}")
                print(f"{col.count} samples -> {a.data_dir}", file=sys.stderr)
            return rc

        # shell
        with collection(a, session, source) as col:
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
