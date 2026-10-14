# Author: Yongho Ahn <nanhosoft@gmail.com>
import io
import json
from pathlib import Path

import jsonschema

from cg_collector import cli

EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "plan_charge_esr.json"


def rows(data_dir, kind):
    return [json.loads(l) for f in sorted((data_dir / "outbox").glob(f"{kind}-*.jsonl")) for l in f.read_text().splitlines()]


def test_monitor_writes_raw_and_samples(sim_path, tmp_path, normalized_schema):
    assert cli.main(["--sim", sim_path, "--data-dir", str(tmp_path), "--run-id", "m1", "monitor", "--seconds", "5"]) == 0
    samples, raw = rows(tmp_path, "samples"), rows(tmp_path, "raw")
    assert len(samples) >= 4 and raw
    for r in samples:
        jsonschema.validate(r, normalized_schema)
        assert r["run_id"] == "m1" and r["raw_log_ref"].startswith("raw-")
    assert all(r["frame"].startswith("$") for r in raw)
    assert {r["dir"] for r in raw} == {"N>P", "P>N"}  # both directions logged (replayable)
    oks = [r for r in samples if r["record_status"] == "OK"]
    # RAW is requested after every CGT and joined to the same slot
    assert sum(r["nano_adc_tap_raw"][0] is not None for r in oks) >= len(oks) - 1
    assert all(0 <= r["t_sim_s"] <= 6 for r in samples)  # experiment elapsed time
    events = rows(tmp_path, "events")
    assert any(e["event_type"] == "REQUESTED" and e["requested_value"] == "SYNC" for e in events)
    assert any(json.loads(json.dumps(r))["frame"].startswith("$CGA") for r in raw)


def test_run_plan_with_unlocked_simulator(sim_path, tmp_path, capsys):
    rc = cli.main(["--sim", sim_path, "--sim-gates", "unlocked", "--data-dir", str(tmp_path), "run", str(EXAMPLE),
                   "--i-am-supervising"])
    out = capsys.readouterr().out
    assert rc == 0, out
    assert "ESR [2, 1] -> OK" in out and out.strip().splitlines()[-1].startswith("OFF")
    samples = rows(tmp_path, "samples")
    assert any(r["esr_command_mask"] == 2 for r in samples)


def test_run_refuses_actuation_without_supervision(sim_path, tmp_path):
    assert cli.main(["--sim", sim_path, "--data-dir", str(tmp_path), "run", str(EXAMPLE)]) == 2


def test_run_aborts_on_locked_firmware(sim_path, tmp_path, capsys):
    rc = cli.main(["--sim", sim_path, "--data-dir", str(tmp_path), "run", str(EXAMPLE), "--i-am-supervising"])
    assert rc == 1
    assert "HARDWARE_LOCKED" in capsys.readouterr().err


def test_shell_keeps_session(sim_path, tmp_path, monkeypatch, capsys):
    monkeypatch.setattr("sys.stdin", io.StringIO("status\narm\nleak 1 1\nesr 1 1\nleak 1 0\nfly\nquit\n"))
    assert cli.main(["--sim", sim_path, "--sim-gates", "unlocked", "--data-dir", str(tmp_path), "shell",
                     "--i-am-supervising"]) == 0
    out = capsys.readouterr().out.splitlines()
    assert out[0] == "SYNC OK NONE" and out[1].startswith("state READY")
    assert out[2:] == ["OK NONE", "OK NONE", "OK NONE", "OK NONE", "OK NONE", "unknown command FLY"]


def test_shell_without_supervision_refuses_actuation(sim_path, tmp_path, monkeypatch, capsys):
    monkeypatch.setattr("sys.stdin", io.StringIO("charge 1\nping\nquit\n"))
    assert cli.main(["--sim", sim_path, "--data-dir", str(tmp_path), "shell"]) == 0
    out = capsys.readouterr().out.splitlines()
    assert out[1].startswith("state LOCKED")
    assert out[2].startswith("refused") and out[3] == "OK NONE"


def test_query_and_outbox(sim_path, tmp_path, capsys):
    assert cli.main(["--sim", sim_path, "query", "status"]) == 0
    assert "OK NONE" in capsys.readouterr().out
    cli.main(["--sim", sim_path, "--data-dir", str(tmp_path), "monitor", "--seconds", "2"])
    capsys.readouterr()
    assert cli.main(["--data-dir", str(tmp_path), "outbox"]) == 0
    assert len(capsys.readouterr().out.splitlines()) == 3  # raw + samples + events


def test_replay_of_own_raw_log_reproduces_samples(sim_path, tmp_path):
    assert cli.main(["--sim", sim_path, "--sim-gates", "unlocked", "--data-dir", str(tmp_path / "live"), "--run-id",
                     "r1", "run", str(EXAMPLE), "--i-am-supervising"]) == 0
    live = rows(tmp_path / "live", "samples")
    raws = sorted((tmp_path / "live" / "outbox").glob("raw-*.jsonl"))
    assert cli.main(["--run-id", "r1", "replay", "--from-raw", *map(str, raws), "--out", str(tmp_path / "rp")]) == 0
    again = [json.loads(l) for l in (tmp_path / "rp" / "samples.jsonl").read_text().splitlines()]
    keys = ["record_status", "cell_voltage_v", "nano_adc_tap_raw", "esr_command_mask", "controller_state"]
    strip = lambda rs: [tuple(json.dumps(r[k]) for k in keys) for r in rs if r["record_status"] == "OK"]
    assert strip(again) == strip(live)
