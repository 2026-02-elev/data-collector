"""엣지 본체 시험 (하드웨어 없이): 모드 전환, 랜덤 고장 계획, AI 판정 → ground truth 매칭·보고, 에피소드 채점.

Windows에서도 돌도록 fcntl/termios는 빈 모듈로 대신한다 (직렬포트는 열지 않는다).
"""
import json
import random
import sys
import time
import types
from pathlib import Path

import pytest

EDGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EDGE))
for _m in ("fcntl", "termios"):
    sys.modules.setdefault(_m, types.ModuleType(_m))
import cellguard_edge as ce  # noqa: E402
import cg_ai  # noqa: E402

FIX = Path(__file__).resolve().parent / "fixtures"


def make_edge(tmp_path, **cfg):
    c = dict(ce.DEFAULT_CONFIG, data_dir=str(tmp_path / "data"), models_dir=str(tmp_path / "models"),
             mlops_url="", mqtt_secrets=str(tmp_path / "none.env"), **cfg)
    e = ce.Edge(c, passive=False)
    e.early = None                       # device already registered: publish straight away
    return e


def sent(edge, topic_kind):
    edge.outbox.flush()
    rows = edge.outbox.fetch(0, 10_000)
    return [json.loads(p) for _, t, p in rows if t.endswith("/" + topic_kind)]


def test_set_mode_pause_resume(tmp_path):
    d = tmp_path
    assert ce.normalize_mode(None) == "collect" and ce.normalize_mode("auto") == "collect"
    assert ce.set_mode(d, "fault") == "fault"
    assert ce.set_mode(d, "pause") == "passive"
    assert ce.read_control(d)["resume_mode"] == "fault"
    assert ce.set_mode(d, "resume") == "fault"
    ce.write_control(d, ota_at=1.0)
    assert ce.read_control(d)["mode"] == "fault"          # other keys survive
    assert ce.set_mode(d, "passive") == "passive"
    assert ce.set_mode(d, "resume") == "fault"


def test_plan_episode_shapes(tmp_path):
    e = make_edge(tmp_path)
    rng = random.Random(7)
    shapes = {"none": 0, "single": 0, "compound": 0}
    for _ in range(400):
        p = e.plan_episode(rng)
        shapes[p["shape"]] += 1
        kinds = [f["kind"] for f in p["faults"]]
        assert len(kinds) == len(set(kinds))
        assert len(kinds) == {"none": 0, "single": 1}.get(p["shape"], len(kinds))
        if p["shape"] == "compound":
            assert len(kinds) in (2, 3)
        if "heat" in kinds:
            assert kinds[0] == "heat"
        for f in p["faults"]:
            assert 1 <= f["cell"] <= 4
            assert ("pulses" in f) == (f["kind"] == "esr")
        assert p["target_mv"] in e.cfg["charge_targets_mv"]
    assert all(v > 20 for v in shapes.values()), shapes


def test_ai_event_matched_to_injection_and_deduplicated(tmp_path):
    e = make_edge(tmp_path)
    now = time.monotonic()
    e.fault_req = {"type": "LEAK", "cell_id": 3, "purpose": "injection"}
    e._record_truth("LEAK", [3], now)
    e._on_ai_event({"model": "classifier", "event": "detected", "cell": 3, "fault_type": "LEAK", "prob": 0.9,
                    "version": "v1"})
    e._on_ai_event({"model": "classifier", "event": "detected", "cell": 3, "fault_type": "LEAK", "prob": 0.9,
                    "version": "v1"})                                  # same (type, cell) again: no 2nd event
    e._on_ai_event({"model": "residual", "event": "detected", "cell": 1, "fault_type": "HEAT",
                    "score": 9.0, "score_v": 1.0, "score_t": 8.0, "version": "v1"})
    diags = [d for d in sent(e, "diagnostics") if d["kind"] == "AI_DETECTION"]
    assert [d["details"]["match"] for d in diags] == ["injected", "injected", "none"]
    ev = sent(e, "events")
    assert [(x["event_type"], x["fault_type"], x["cell_id"]) for x in ev] == \
        [("OBSERVABLE", "LEAK", 3), ("OBSERVABLE", "HEAT", 1)]
    assert ev[0]["details"]["match"] == "injected" and ev[1]["details"]["match"] == "none"
    assert len({x["message_id"] for x in ev}) == 2


def test_balance_bleed_is_not_an_injection(tmp_path):
    e = make_edge(tmp_path)
    e.fault_req = {"type": "LEAK", "cell_id": 2, "purpose": "balance"}
    e._record_truth("LEAK", [2], time.monotonic())
    e._on_ai_event({"model": "classifier", "event": "detected", "cell": 2, "fault_type": "LEAK", "prob": 0.8})
    d = [d for d in sent(e, "diagnostics") if d["kind"] == "AI_DETECTION"][0]
    assert d["details"]["match"] == "balance"


def test_anomaly_is_diagnostic_only(tmp_path):
    """residual ANOMALY has no fault_type in the server enum: diag yes, fault event no."""
    e = make_edge(tmp_path)
    e._on_ai_event({"model": "residual", "event": "detected", "cell": 4, "fault_type": "ANOMALY",
                    "score": 8.0, "score_v": 8.0, "score_t": 0.0})
    assert [d["kind"] for d in sent(e, "diagnostics")] == ["AI_DETECTION"]
    assert sent(e, "events") == []


def test_score_episode():
    t0 = 1000.0
    ep = {"id": "r:e00001", "t0": t0, "plan": {"shape": "compound"}, "skipped": [],
          "applied": [{"kind": "heat", "cell": 2, "t": t0 + 60, "seconds": 40},
                      {"kind": "leak", "cell": 2, "t": t0 + 110, "seconds": 20},
                      {"kind": "esr", "cell": 2, "t": t0 + 140, "pulses": 0}],     # refused: not counted
          "detections": [{"t": t0 + 20, "model": "residual", "type": "ANOMALY", "cell": 1, "match": "none"},
                         {"t": t0 + 79, "model": "residual", "type": "HEAT", "cell": 2, "match": "injected"}]}
    r = ce.Edge.score_episode(ep, "completed")
    assert r["applied"] == 2 and r["detected"] == 1
    heat, leak = r["faults"]
    assert heat["detected"] and heat["latency_s"] == 19.0 and heat["model"] == "residual"
    assert not leak["detected"]
    assert [f["cell"] for f in r["false_alarms"]] == [1]


def test_frames_feed_ai_and_truth(tmp_path):
    """CGT/CGP lines go to the AI queue; actuator masks become ground truth."""
    e = make_edge(tmp_path)
    for n in cg_ai.MODEL_NAMES:
        assert e.ota.install_file(n, FIX / f"{n}-v1.zip")["action"] == "applied"
    # CGT: seq ms c1..c4 t1..t4 charge drain armed esr leak h1..h4 trip vq tq
    cgt = ["CGT", "5", "12000", "600", "610", "605", "598", "2500", "2510", "2490", "2505",
           "0", "0", "1", "0", "4", "0", "0", "0", "0", "0", "0", "0"]
    e._frame(cgt)
    e._frame(["CGP", "12400", "12350", "2", "2", "560", "520", "565", "558"])
    kinds = []
    while not e.ai.q.empty():
        kinds.append(e.ai.q.get_nowait()[0])
    assert kinds == ["cgt", "cgp"]
    truth = {(g["type"], g["cell"]) for g in e._truth_near(time.monotonic())}
    assert truth == {("LEAK", 3), ("ESR", 2)}
    tel = sent(e, "telemetry")
    assert tel[0]["leak_command_mask"] == 4 and tel[0]["scenario_id"] == "edge_passive_v3"


def test_status_dict_is_json(tmp_path):
    e = make_edge(tmp_path)
    json.dumps(e._status_dict())
    json.dumps(e._status_dict(brief=True))


@pytest.mark.parametrize("cmd,mode", [({"command": "mode", "mode": "fault"}, "fault"),
                                      ({"command": "pause"}, "passive")])
def test_remote_command(tmp_path, cmd, mode):
    e = make_edge(tmp_path)
    e.pub.commands.put(cmd)
    e.mode()
    assert ce.read_control(e.data)["mode"] == mode
    acks = [d for d in sent(e, "diagnostics") if d["kind"] == "COMMAND_ACK"]
    assert acks and acks[0]["details"]["ok"]
