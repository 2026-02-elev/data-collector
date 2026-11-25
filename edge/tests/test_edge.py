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


def test_fw_concurrent(tmp_path):
    e = make_edge(tmp_path)
    for fw, ok in (("0.2.0-RUNTIME", True), ("0.3.1", True), ("0.1.9-RUNTIME", False), ("unknown", False)):
        e.tel.firmware = fw
        assert e.fw_concurrent() is ok, fw


class FakeNano:
    def __init__(self):
        self.cmds = []

    def command(self, name, *args, timeout=0.8, tries=3):
        self.cmds.append((name, *args))
        return True, "NONE"

    def sync(self):
        pass


def test_compound_overlap_runs_faults_together(tmp_path, monkeypatch):
    """Nano 0.2.0: HEAT + LEAK start together, ESR load pulses run while they are on and end with
    DISCHARGE 0 / ESR n 0 (not OFF), each fault stops at its own time, a single OFF at the very end."""
    e = make_edge(tmp_path)
    clock = [1000.0]
    monkeypatch.setattr(ce.time, "monotonic", lambda: clock[0])

    def wait(seconds, guard=None, until=None):
        if guard:
            guard()
        clock[0] += seconds
        return False

    e.wait = wait
    e.nano = FakeNano()
    e.tel.firmware = "0.2.0-RUNTIME"
    e.tel.runtime = {"profile": 1, "enabled": 15, "heat_pwm": 32, "heat_s": 20, "esr_s": 4, "leak_s": 60}
    e.tel.cells, e.tel.temps = [600, 605, 598, 602], [25.0] * 4
    e.need_cells = lambda: [600, 605, 598, 602]
    e.tel.filtered = lambda: [600, 605, 598, 602]
    e.spread_guard = lambda: None
    refs = []
    e.load_pulse = lambda cell=None: refs.append(cell) or 5
    e.episode = {"id": "r:e00001", "applied": [], "skipped": [], "detections": []}
    faults = [{"kind": "heat", "cell": 2, "fraction": 1.0}, {"kind": "leak", "cell": 3, "fraction": 0.5},
              {"kind": "esr", "cell": 2, "pulses": 2}]
    e.compound_overlap(faults)

    cmds = e.nano.cmds
    assert refs == [None]                                        # no-ESR reference pulse first
    assert cmds[:3] == [("ARM",), ("HEAT", 2, 32), ("LEAK", 3, 1)]
    assert cmds.count(("ESR", 2, 1)) == 2 and cmds.count(("DISCHARGE", 0)) == 2 and cmds.count(("ESR", 2, 0)) == 2
    assert [c for c in cmds if c[0] == "OFF"] == [("OFF",)] and cmds[-1] == ("OFF",)
    i_heat, i_leak = cmds.index(("HEAT", 2, 0)), cmds.index(("LEAK", 3, 0))
    assert i_heat < i_leak                                       # 20 s heat ends before the 30 s leak
    assert cmds.index(("DRAIN", 498)) < i_heat                  # ESR pulses inside the overlap
    applied = {(a["kind"], a["cell"]): a for a in e.episode["applied"]}
    assert applied[("heat", 2)]["seconds"] == 20 and applied[("leak", 3)]["seconds"] == 30
    assert applied[("esr", 2)]["pulses"] == 2 and all(a["overlap"] for a in applied.values())
    assert e.fault_req is None                                   # OFF clears the request


def test_compound_overlap_skips_disabled_kind(tmp_path):
    e = make_edge(tmp_path)
    e.tel.runtime = {"profile": 1, "enabled": 1 | 4, "leak_s": 60}   # POWER + LEAK only
    e.episode = {"id": "r:e00002", "applied": [], "skipped": [], "detections": []}
    e.need_cells = lambda: (_ for _ in ()).throw(AssertionError("must not touch the rig"))
    e.compound_overlap([{"kind": "heat", "cell": 1, "fraction": 1.0}, {"kind": "esr", "cell": 1, "pulses": 2}])
    assert [s["step"] for s in e.episode["skipped"]] == ["heat@1", "esr@1"]


# ---- demo mode (시연): normal-looking run, one hidden random fault, AI detection -> TRIP
def test_plan_demo(tmp_path):
    e = make_edge(tmp_path)
    rng = random.Random(3)
    seen = set()
    for _ in range(300):
        p = e.plan_demo(rng)
        seen.add(p["kind"])
        assert p["shape"] == "single" and 1 <= p["cell"] <= 4
        assert e.cfg["demo_normal_min_s"] <= p["normal_s"] <= e.cfg["demo_normal_max_s"]
    assert seen == {"heat", "leak", "esr"}
    assert "demo" in ce.MODES and ce.set_mode(tmp_path, "demo") == "demo"


def armed_demo(e, kind="heat", cell=2):
    ep = {"id": "r:d00001", "t0": time.monotonic(), "plan": {"shape": "single", "kind": kind, "cell": cell},
          "applied": [], "skipped": [], "detections": []}
    e.episode = ep
    e.demo = {"ep": ep, "hit": None, "cut": None}
    e.nano = FakeNano()
    return ep


def test_demo_ai_detection_trips_and_reports(tmp_path):
    e = make_edge(tmp_path)
    ep = armed_demo(e, "heat", 2)
    e.fault_req = {"type": "HEAT", "cell_id": 2, "purpose": "injection"}
    e._note_applied("heat", 2, seconds=20)
    ep["applied"][0]["t"] -= 5.0
    e._record_truth("HEAT", [2], time.monotonic())
    e._on_ai_event({"model": "classifier", "event": "detected", "cell": 2, "fault_type": "HEAT", "prob": 0.97,
                    "version": "v3"})
    assert e.demo["hit"]["match"] == "injected"
    with pytest.raises(ce.Tripped, match="HOST_TRIP"):
        e._ai_cutoff(e.demo)
    assert ("TRIP",) in e.nano.cmds and e.demo["cut"] is not None
    cut = [d for d in sent(e, "diagnostics") if d["kind"] == "AI_CUTOFF"][0]["details"]
    assert cut["fault_type"] == "HEAT" and cut["cell"] == 2 and cut["injected"] == {"type": "HEAT", "cell": 2}
    assert cut["detect_latency_s"] >= 5.0
    e._frame(["CGE", "40", "99000", "TRIP", "0", "0", "0", "HOST_TRIP"])       # the Nano confirms
    tripped = [x for x in sent(e, "events") if x["event_type"] == "TRIPPED"][-1]
    assert (tripped["fault_type"], tripped["cell_id"]) == ("HEAT", 2)
    assert tripped["details"]["cause"] == "AI_CUTOFF" and tripped["details"]["model"] == "classifier"


def test_demo_cutoff_policy(tmp_path):
    e = make_edge(tmp_path)
    armed_demo(e)
    e.fault_req = {"type": "LEAK", "cell_id": 1, "purpose": "balance"}       # bleeding to balance: no cut
    e._record_truth("LEAK", [1], time.monotonic())
    e._on_ai_event({"model": "classifier", "event": "detected", "cell": 1, "fault_type": "LEAK", "prob": 0.9})
    assert e.demo["hit"] is None
    e2 = make_edge(tmp_path / "b", demo_cutoff_on="injected")                # false alarm: no cut
    armed_demo(e2)
    e2._on_ai_event({"model": "residual", "event": "detected", "cell": 3, "fault_type": "HEAT", "score": 9.0})
    assert e2.demo["hit"] is None
    e3 = make_edge(tmp_path / "c")                                           # default "any": it cuts
    armed_demo(e3)
    e3._on_ai_event({"model": "residual", "event": "detected", "cell": 3, "fault_type": "HEAT", "score": 9.0})
    assert e3.demo["hit"]["match"] == "none"
    e4 = make_edge(tmp_path / "d")                                           # not armed: diagnostic only
    e4._on_ai_event({"model": "residual", "event": "detected", "cell": 3, "fault_type": "HEAT", "score": 9.0})
    assert e4.demo is None


def test_demo_trip_not_acknowledged_switches_off(tmp_path):
    e = make_edge(tmp_path)
    armed_demo(e)
    e.demo["hit"] = {"model": "classifier", "fault_type": "HEAT", "cell": 2, "t": time.monotonic()}
    e.nano.command = lambda name, *a, **k: e.nano.cmds.append((name, *a)) or (name != "TRIP", "BUSY")
    with pytest.raises(ce.Skip):
        e._ai_cutoff(e.demo)
    assert e.nano.cmds[-1] == ("OFF",) and e.ai_cutoff is None


def test_demo_show_cut_during_fault(tmp_path):
    """Whole show with the rig stubbed: the fault starts, the AI fires, the next tick sends TRIP;
    DEMO_RESULT names the cut, the demo index advances and the cutoff is not a 'trip to pause on'."""
    e = make_edge(tmp_path, demo_normal_min_s=1, demo_normal_max_s=1)
    e.nano = FakeNano()
    e.tel.temps = [25.0] * 4
    e.tel.cells = [700, 702, 699, 701]
    e.start_session = lambda: None
    e.charge_to = lambda target: None
    e.wait = lambda seconds, guard=None, until=None: e.tick()
    e.tick = lambda: (e._ai_cutoff(e.demo) if e.demo and e.demo["hit"] and e.demo["cut"] is None else None)
    e.plan_demo = lambda rng: {"shape": "single", "kind": "leak", "cell": 4, "target_mv": 700, "normal_s": 1}

    def stimulus(kind, cell, seconds=None, pulses=None, reference=True):
        e.fault_req = {"type": "LEAK", "cell_id": cell, "purpose": "injection"}
        e._note_applied(kind, cell, seconds=60)
        e._record_truth("LEAK", [cell], time.monotonic())
        e._on_ai_event({"model": "classifier", "event": "detected", "cell": cell, "fault_type": "LEAK",
                        "prob": 0.95, "version": "v3"})
        e.wait(1)

    e.stimulus = stimulus
    with pytest.raises(ce.Tripped):
        e.demo_show()
    assert e.demo is None and e.episode is None and e.persist["demo_index"] == 1
    res = [d for d in sent(e, "diagnostics") if d["kind"] == "DEMO_RESULT"][0]["details"]
    assert res["outcome"] == "cutoff" and res["detected"] == 1
    assert res["cutoff"]["fault_type"] == "LEAK" and res["cutoff"]["cell"] == 4
    start = [d for d in sent(e, "diagnostics") if d["kind"] == "DEMO_START"][0]["details"]
    assert "kind" not in start and "cell" not in start                      # the plan stays hidden
    resets = []
    e._reset_after = lambda end: resets.append(round(end - time.monotonic()))
    e._passive_sleep = lambda s: None
    e.handle_trip("HOST_TRIP")
    assert resets == [e.cfg["demo_recover_s"]] and len(e.trip_times) == 0 and e.ai_cutoff is None


# ---- operator commands from the dashboard: modes with a target, one-shots, reset history
def command(e, **cmd):
    e.pub.commands.put(cmd)
    e._poll_commands()
    ack = [d for d in sent(e, "diagnostics") if d["kind"] == "COMMAND_ACK"][-1]
    return {**ack["details"], "note": ack["message"]}


def test_command_modes_hold_and_discharge(tmp_path):
    e = make_edge(tmp_path)
    assert {"hold", "discharge"} <= set(ce.MODES)
    ack = command(e, command="mode", mode="hold", target_mv=650)
    assert ack["ok"] and ce.read_control(e.data)["mode"] == "hold"
    assert e._param("target_mv", 700) == 650
    command(e, command="mode", mode="discharge", target_mv=99999)        # clamped to what the rig can do
    assert e._param("target_mv", 400) == e.cfg["cmd_max_target_mv"]
    command(e, command="mode", mode="demo")                              # no target: params cleared
    assert e._param("target_mv", 700) == 700


def test_command_one_shots_run_once(tmp_path):
    e = make_edge(tmp_path)
    assert e._pending_once() is None
    assert command(e, command="charge", target_mv=600)["ok"]
    once = e._pending_once()
    assert once["action"] == "charge" and once["target_mv"] == 600
    assert e._pending_once() is None                                     # the same command never runs twice
    assert command(e, command="inject", kind="heat", cell=3, ai_cutoff=False)["ok"]
    once = e._pending_once()
    assert (once["action"], once["kind"], once["cell"], once["ai_cutoff"]) == ("inject", "heat", 3, False)
    assert not command(e, command="inject", kind="smoke", cell=9)["ok"]  # unknown fault / cell refused
    assert e._pending_once() is None


def test_command_reset_history(tmp_path):
    e = make_edge(tmp_path)
    e.nano = FakeNano()
    e.trip_times.extend([1.0, 2.0, 3.0])
    e.pause_until = time.monotonic() + 3600
    e.pause_reason = "3 trips in the last hour"
    e.charge_noresp, e.charger_pauses = 2, 1
    e.notes["trip_OVER_TEMPERATURE"] += 2
    ack = command(e, command="reset_history")
    assert ack["ok"] and "3 trips" in ack["note"]
    assert not e.trip_times and e.pause_until == 0.0 and e.pause_reason is None
    assert (e.charge_noresp, e.charger_pauses) == (0, 0) and "trip_OVER_TEMPERATURE" not in e.notes
    e.tel.nano_state = "TRIPPED"                                         # a latched firmware trip gets a RESET
    assert "RESET ok" in command(e, command="reset_history")["note"]
    assert ("RESET",) in e.nano.cmds


def test_one_shot_interrupts_a_running_mode(tmp_path):
    """hold/discharge never return, so a pending one-shot must stop them (Paused) for the run loop to run it."""
    e = make_edge(tmp_path)
    e.nano = FakeNano()
    e.nano.last_rx = time.monotonic()
    e.nano.counters = {}
    e.phase = "hold"
    e.running_mode = "hold"
    ce.set_mode(e.data, "hold")
    e.tick()                                              # nothing pending: carries on
    assert command(e, command="charge", target_mv=500)["ok"]
    with pytest.raises(ce.Paused):
        e.tick()
    assert e._pending_once()["action"] == "charge"        # the run loop takes it ...
    e.tick()                                              # ... and the mode carries on afterwards
