"""엣지 AI 시험: OTA 검증·설치·되돌리기, MLOps 테스트 벡터 일치(학습-서빙), 이상 판정 로직.

fixtures/*.zip 은 MLOps가 실측 번들(2026-11-07~10, 15세션)로 학습해 만든 OTA 패키지 v1 그대로다.
    python -m pytest edge/tests -q
"""
import hashlib
import json
import sys
import time
import zipfile
import io
from pathlib import Path

import numpy as np
import pytest

EDGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EDGE))
import cg_ai  # noqa: E402

FIX = Path(__file__).resolve().parent / "fixtures"


def pkg(name):
    data = (FIX / f"{name}-v1.zip").read_bytes()
    return data, hashlib.sha256(data).hexdigest()


class FakeHttp:
    def __init__(self, packages):
        self.packages = packages          # name -> (version, data, sha)
        self.reports = []

    def get(self, path):
        parts = path.strip("/").split("/")
        if parts[0] == "models" and parts[-1] == "production":
            if parts[1] not in self.packages:
                return 404, {}, b"{}"
            return 200, {}, json.dumps({"version": self.packages[parts[1]][0]}).encode()
        if parts[0] == "models" and parts[-1] == "package":
            v, data, sha = self.packages[parts[1]]
            return 200, {"X-Package-SHA256": sha}, data
        return 404, {}, b""

    def post_json(self, path, body):
        self.reports.append((path, body))
        return 201


@pytest.mark.parametrize("name", cg_ai.MODEL_NAMES)
def test_package_passes_test_vectors(tmp_path, name):
    data, sha = pkg(name)
    slot = cg_ai.ModelSlot(tmp_path, name)
    ok, detail, files, manifest = slot.verify(data, sha)
    assert ok, detail
    assert manifest["name"] == name and "test_vectors.json" in files


def test_ota_pull_apply_then_up_to_date(tmp_path):
    http = FakeHttp({n: ("v1", *pkg(n)) for n in cg_ai.MODEL_NAMES})
    seen = []
    ota = cg_ai.Ota(tmp_path, "pack-01", http, lambda *a: seen.append(a))
    res = ota.check_all()
    assert all(r["action"] == "applied" for r in res.values()), res
    assert ota.versions() == {n: "v1" for n in cg_ai.MODEL_NAMES}
    assert [b["status"] for p, b in http.reports if p == "/edge/versions"] == ["applied"] * 3
    assert len(seen) == 3
    res = ota.check_all()
    assert all(r["action"] == "none" for r in res.values())


def test_corrupt_package_is_rejected_and_current_kept(tmp_path):
    data, sha = pkg("residual")
    ota = cg_ai.Ota(tmp_path, "pack-01", None, None)
    assert ota.apply("residual", data, sha)["action"] == "applied"
    bad = data[:-1] + bytes([data[-1] ^ 0xFF])
    res = ota.apply("residual", bad, sha)
    assert res["action"] == "failed" and "checksum" in res["detail"]
    assert ota.slots["residual"].current_version == "v1"


def test_test_vector_mismatch_is_rejected(tmp_path):
    """모델 계수가 바뀌었는데 테스트 벡터는 그대로면 (학습-서빙 불일치) 설치하지 않는다."""
    data, _ = pkg("load_esr")
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        files = {n: z.read(n) for n in z.namelist()}
    model = json.loads(files["model.json"])
    model["threshold"] = float(model["threshold"]) + 50
    files["model.json"] = json.dumps(model).encode()
    manifest = json.loads(files["manifest.json"])
    manifest["files"]["model.json"] = hashlib.sha256(files["model.json"]).hexdigest()
    files["manifest.json"] = json.dumps(manifest).encode()
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        for n, d in files.items():
            z.writestr(n, d)
    out = buf.getvalue()
    res = cg_ai.Ota(tmp_path, "p", None, None).apply("load_esr", out, hashlib.sha256(out).hexdigest())
    assert res["action"] == "failed" and "test vector" in res["detail"]


def test_rollback(tmp_path):
    ota = cg_ai.Ota(tmp_path, "p", None, None)
    ota.apply("load_esr", *pkg("load_esr"))
    slot = ota.slots["load_esr"]
    assert slot.rollback() is None           # nothing older yet
    ota.apply("load_esr", *pkg("load_esr"))  # same version again -> previous exists
    assert slot.rollback() == "v1"


def installed_engine(tmp_path, events):
    ota = cg_ai.Ota(tmp_path, "p", None, None)
    for n in cg_ai.MODEL_NAMES:
        assert ota.apply(n, *pkg(n))["action"] == "applied"
    eng = cg_ai.AiEngine(ota, {}, events.append, log=lambda m: None)
    eng._load()
    return eng


def rest_step(t, v_mv, temps):
    return cg_ai.step_from_cgt(t, v_mv, temps, ["VALID"] * 4, True, False, False)


def test_engine_flags_heated_cell_only(tmp_path):
    events = []
    eng = installed_engine(tmp_path, events)
    rng = np.random.default_rng(1)
    v = [600, 610, 605, 598]
    for k in range(120):                     # 2 min quiet
        temps = [25.0 + rng.normal(0, 0.03) for _ in range(4)]
        eng._sample(rest_step(k, v, temps), 1000 * k)
    assert [e for e in events if e["event"] == "detected"] == []
    for k in range(120, 160):                # cell 2 heats ~0.6 C over 30 s (like HEAT pwm 32)
        temps = [25.0 + rng.normal(0, 0.03) for _ in range(4)]
        temps[1] += min(0.6, 0.02 * (k - 120))
        eng._sample(rest_step(k, v, temps), 1000 * k)
    det = [e for e in events if e["event"] == "detected"]
    assert det, "no detection"
    assert {e["cell"] for e in det} == {2}
    assert {e["fault_type"] for e in det} == {"HEAT"}
    assert all(e["version"] == "v1" for e in det)


def test_engine_load_esr(tmp_path):
    events = []
    eng = installed_engine(tmp_path, events)
    eng._sample(rest_step(0, [700, 700, 700, 700], [25.0] * 4), 10_000)
    eng._loaded(np.array([0.69, 0.69, 0.69, 0.69]), 10_500, "DISCHARGE", 0)      # equal drops: normal
    assert not [e for e in events if e["model"] == "load_esr"]
    eng._loaded(np.array([0.69, 0.69, 0.60, 0.69]), 10_600, "DISCHARGE", 4)      # cell 3 drops 90 mV more
    det = [e for e in events if e["model"] == "load_esr"]
    assert len(det) == 1 and det[0]["cell"] == 3 and det[0]["fault_type"] == "ESR"
    eng._loaded(np.array([0.69, 0.69, 0.60, 0.69]), 10_700, "DISCHARGE", 4)      # held: no repeat
    assert len([e for e in events if e["model"] == "load_esr"]) == 1
    eng._loaded(np.array([0.69, 0.69, 0.60, 0.69]), 20_000, "DISCHARGE", 4)      # rest too old: ignored
    assert eng.last_out["load_esr"]["rest_gap_ms"] == 700   # the stale one did not update


def test_detector_classifier_confirm_and_clear():
    d = cg_ai.Detector({"ai_classifier_confirm_windows": 3, "ai_clear_steps": 2})
    leak = {"probs": np.array([[0.1, 0.0, 0.9, 0.0]] + [[1, 0, 0, 0]] * 3), "label": ["leak", "normal", "normal", "normal"]}
    esr = {"probs": np.array([[0.1, 0.9, 0.0, 0.0]] * 4), "label": ["esr"] * 4}
    normal = {"probs": np.array([[1.0, 0, 0, 0]] * 4), "label": ["normal"] * 4}
    assert d.classifier(leak) == [] and d.classifier(leak) == []
    ev = d.classifier(leak)
    assert len(ev) == 1 and ev[0]["cell"] == 1 and ev[0]["fault_type"] == "LEAK"
    assert d.classifier(leak) == []                       # already active
    out = [e for _ in range(5) for e in d.classifier(esr)]   # esr class not reported by default (load_esr owns ESR)
    assert [e["event"] for e in out] == ["cleared"]          # leak on cell 1 clears, nothing new detected
    assert d.classifier(normal) == []
    ev = d.classifier(normal)
    assert ev == [] or ev[0]["event"] == "cleared"


# MLOps가 실측 복합 고장 데이터셋(2026-11-18~20, 772 에피소드)으로 학습한 패키지. ③ v3는 멀티레이블
NEW = {"residual": "residual-v5.zip", "load_esr": "load_esr-v2.zip", "classifier": "classifier-v3.zip"}


def _zip_json(fname, member):
    return json.loads(zipfile.ZipFile(FIX / fname).read(member))


@pytest.mark.parametrize("name", cg_ai.MODEL_NAMES)
def test_new_packages_pass_test_vectors(tmp_path, name):
    data = (FIX / NEW[name]).read_bytes()
    ok, detail, files, manifest = cg_ai.ModelSlot(tmp_path, name).verify(data, hashlib.sha256(data).hexdigest())
    assert ok, detail


def test_multilabel_classifier_outputs_fault_lists():
    model = _zip_json(NEW["classifier"], "model.json")
    vec = _zip_json(NEW["classifier"], "test_vectors.json")
    clf = cg_ai.FaultClassifier(model)
    outs = [o for o in (clf.update(cg_ai.as_step(s["input"])) for s in vec["steps"]) if o is not None]
    assert outs and all(o["classifier_type"] == "multilabel" and o["probs"].shape == (4, 3) for o in outs)
    thr = np.array([model["thresholds"][c] for c in outs[0]["classes"]])
    for o in outs:
        for k in range(4):
            assert o["labels"][k] == [c for c, p, t in zip(o["classes"], o["probs"][k], thr) if p >= t]
            if o["labels"][k]:
                assert o["label"][k] in o["labels"][k]
            else:
                assert o["label"][k] == "normal"


def test_multilabel_model_needs_sigmoid_and_thresholds():
    model = _zip_json(NEW["classifier"], "model.json")
    for bad in ({**model, "thresholds": {"leak": 0.9}}, {**model, "output_activation": "softmax"},
                {**model, "classifier_type": "something"}):
        with pytest.raises(ValueError):
            cg_ai.FaultClassifier(bad)
    single = cg_ai.FaultClassifier(_zip_json("classifier-v1.zip", "model.json"))   # 기존 모델은 그대로
    assert not single.multi


def test_detector_multilabel_confirms_each_fault_on_one_cell():
    d = cg_ai.Detector({"ai_classifier_confirm_windows": 3, "ai_clear_steps": 2})

    def out(labels):
        return {"classifier_type": "multilabel", "classes": ["esr", "leak", "heat"],
                "probs": np.full((4, 3), 0.95), "labels": labels}

    both = out([["leak", "heat"], [], [], []])
    assert d.classifier(both) == [] and d.classifier(both) == []
    ev = d.classifier(both)
    assert sorted(e["fault_type"] for e in ev) == ["HEAT", "LEAK"] and {e["cell"] for e in ev} == {1}
    assert d.classifier(both) == [] and d.active_classifier() == {"1": "heat+leak"}
    ev = [e for _ in range(2) for e in d.classifier(out([["heat"], [], [], []]))]
    assert [(e["event"], e["class"]) for e in ev] == [("cleared", "leak")]      # 누설만 해제, 과열은 유지
    ev = [e for _ in range(3) for e in d.classifier(out([["esr"], [], [], []]))]
    assert [(e["event"], e["class"]) for e in ev] == [("cleared", "heat")]      # esr은 보고 안 함 (load_esr 몫)


def test_step_from_cgt_quality_rules():
    s = cg_ai.step_from_cgt(1.0, [500, 510, 520, 530], [25.0, None, 26.0, 27.0],
                            ["VALID", "VALID", "SHORT_OR_LOW", "VALID"], True, True, False)
    assert np.allclose(s["v"], [0.5, 0.51, 0.52, 0.53])
    assert np.isnan(s["temp"][1]) and np.isnan(s["temp"][2]) and s["temp"][3] == 27.0
    s = cg_ai.step_from_cgt(1.0, [500, 510, 520, 530], [25.0] * 4, ["VALID"] * 4, False, False, True)
    assert np.isnan(s["v"]).all() and s["discharge"]
