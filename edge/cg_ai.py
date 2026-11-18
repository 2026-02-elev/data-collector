"""CellGuard 엣지 AI: MLOps 모델 추론, OTA 갱신, 이상 판정.

numpy와 표준 라이브러리만 쓴다 (Pi Zero 2 W에 추가 설치 없이).

추론 코드는 MLOps `app/edge_ref` (residual.py, classifier.py, load_check.py)를 그대로 옮긴 것이다.
OTA 패키지의 test_vectors.json으로 이 코드가 서버와 같은 값을 내는지 설치 전에 확인한다 (학습-서빙 불일치 방지).

모델 (MLOps 이름 → 엣지가 하는 일)
    residual    ② 잔차 + EWMA. 1초 측정마다 셀별 점수. 실측 모델은 온도 상승 점수라 과열을 맡는다
    classifier  ③ 고장 분류기. 최근 20초 창으로 셀별 정상/내부저항/누설/과열. 실측에서 누설·과열을 맡는다
    load_esr    부하 측정(CGP) 내부저항 판정. 부하 전압과 직전 휴지 전압 차이로 내부저항 증가 셀을 찾는다

ADR-006: 이 판정은 진단 정보다. 펌웨어 보호(①)를 대신하거나 우회하지 않는다. Pi는 판정 결과로 출력을 켜지 않는다.
"""
from __future__ import annotations

import hashlib
import io
import json
import queue
import shutil
import threading
import time
import urllib.error
import urllib.request
import zipfile
from collections import deque
from pathlib import Path

import numpy as np

# =========================================================================== ② residual (edge_ref/residual.py)
REGIMES = ["rest", "charge", "discharge", "transition"]
Z_CLIP = 50.0


def temp_deviation(temp: np.ndarray) -> np.ndarray:
    temp = np.asarray(temp, dtype=float)
    ok = np.isfinite(temp)
    out = np.full(len(temp), np.nan)
    if ok.sum() < 3:
        return out
    for k in np.flatnonzero(ok):
        out[k] = temp[k] - temp[ok & (np.arange(len(temp)) != k)].mean()
    return out


def features(dv, v_prev, i, di, use_current):
    n = len(dv)
    others = (dv.sum() - dv) / (n - 1)
    cols = [others, v_prev, np.ones(n)]
    if use_current:
        cols += [np.full(n, i), np.full(n, di)]
    return np.stack(cols, axis=-1)


def regime_index(charge: bool, discharge: bool) -> int:
    return 1 if charge else 2 if discharge else 0


class ResidualScorer:
    def __init__(self, model: dict, calibration: dict | None = None):
        self.coef = np.asarray(model["coef"], dtype=float)
        self.pack_normalized = bool(model.get("pack_normalized", False))
        self.gain = None
        if self.pack_normalized:
            g = (calibration or {}).get("cell_gain")
            self.gain = np.asarray(g if g is not None else [1.0] * self.coef.shape[1], dtype=float)
        self.std = np.asarray(model["residual_std_mv"], dtype=float) / 1000.0
        self.use_current = bool(model.get("use_current", False))
        self.temp_std = None if model.get("temp_std_c") is None else np.asarray(model["temp_std_c"], dtype=float)
        self.temp_offset = np.asarray(model.get("temp_offset_c") or [0.0] * self.coef.shape[1], dtype=float)
        self.temp_mode = model.get("temp_mode", "level")
        self.temp_lam_base = float(model.get("temp_base_lambda", 1 / 300))
        self.temp_lam_fast = float(model.get("temp_fast_lambda", 0.2))
        self.temp_base = np.full(self.coef.shape[1], np.nan)
        self.lam = float(model["ewma_lambda"])
        self.lam_slow = None if model.get("ewma_lambda_slow") is None else float(model["ewma_lambda_slow"])
        self.threshold = float(model["threshold"])
        self.max_gap = float(model.get("max_gap_s", 2.5))
        self.norm = np.sqrt(self.lam / (2 - self.lam))
        self.norm_slow = None if self.lam_slow is None else np.sqrt(self.lam_slow / (2 - self.lam_slow))
        n = self.coef.shape[1]
        self.ewma_v = np.zeros(n)
        self.ewma_slow = np.zeros(n)
        self.ewma_t = np.zeros(n)
        self._prev = None

    def _out(self, n, valid, dv_hat=None, z=None, reg=-1):
        sv = np.abs(self.ewma_v) / self.norm
        if self.lam_slow is not None:
            sv = np.maximum(sv, np.abs(self.ewma_slow) / self.norm_slow)
        st = self.ewma_t.copy() if self.temp_mode == "rise" else np.abs(self.ewma_t) / self.norm
        score = np.maximum(sv, st)
        return {"valid": valid, "regime": reg,
                "dv_hat": np.zeros(n) if dv_hat is None else dv_hat,
                "z": np.zeros(n) if z is None else z,
                "score_v": sv, "score_t": st, "score": score, "alarm": score > self.threshold}

    def _temp_update(self, temp):
        if self.temp_std is None:
            return
        ok = ~np.isnan(temp)
        if ok.sum() < 3:
            return
        if self.temp_mode == "rise":
            dev = temp_deviation(temp)
            okd = np.isfinite(dev)
            self.temp_base = np.where(okd & np.isnan(self.temp_base), dev, self.temp_base)
            zt = np.where(okd, np.clip((dev - self.temp_base) / self.temp_std, 0.0, Z_CLIP), 0.0)
            self.temp_base = np.where(okd, (1 - self.temp_lam_base) * self.temp_base + self.temp_lam_base * dev,
                                      self.temp_base)
            self.ewma_t = np.where(okd, (1 - self.temp_lam_fast) * self.ewma_t + self.temp_lam_fast * zt, self.ewma_t)
            return
        zt = np.zeros(len(temp))
        for k in np.flatnonzero(ok):
            others = temp[ok & (np.arange(len(temp)) != k)]
            zt[k] = (temp[k] - others.mean() - self.temp_offset[k]) / self.temp_std[k]
        zt = np.clip(zt, -Z_CLIP, Z_CLIP)
        self.ewma_t = np.where(ok, (1 - self.lam) * self.ewma_t + self.lam * zt, self.ewma_t)

    def update(self, step: dict) -> dict:
        v = np.asarray(step["v"], dtype=float)
        n = len(v)
        temp = np.asarray(step.get("temp", [np.nan] * n), dtype=float)
        i = float(step.get("i", np.nan))
        reg_now = regime_index(step["charge"], step["discharge"])
        self._temp_update(temp)
        bad = np.isnan(v).any() or (self.use_current and np.isnan(i))
        if bad:
            self._prev = None
            return self._out(n, False)
        prev = self._prev
        self._prev = (float(step["t"]), v, i, reg_now)
        if prev is None or step["t"] - prev[0] > self.max_gap:
            return self._out(n, False)
        t_prev, v_prev, i_prev, reg_prev = prev
        r = 3 if reg_now != reg_prev else reg_now
        dv = v - v_prev
        if self.gain is not None:
            dv = dv / self.gain
        x = features(dv, v_prev, i, i - i_prev if self.use_current else 0.0, self.use_current)
        dv_hat = (x * self.coef[r]).sum(axis=-1)
        z = np.clip((dv - dv_hat) / self.std[r], -Z_CLIP, Z_CLIP)
        self.ewma_v = (1 - self.lam) * self.ewma_v + self.lam * z
        if self.lam_slow is not None:
            self.ewma_slow = (1 - self.lam_slow) * self.ewma_slow + self.lam_slow * z
        return self._out(n, True, dv_hat, z, r)


# =========================================================================== ③ classifier (edge_ref/classifier.py)
FEATURES = ["z_mean", "z_std", "z_min", "z_max", "rest_slope_mv_s", "has_rest", "load_offset_mv", "has_load",
            "max_jump_mv", "temp_dev_c", "temp_valid", "temp_rise_c", "frac_charge", "frac_discharge", "v_mean",
            "abs_current_a", "current_valid"]


def _relative(x: np.ndarray) -> np.ndarray:
    out = np.full_like(x, np.nan)
    for k in range(x.shape[1]):
        others = np.delete(x, k, axis=1)
        with np.errstate(all="ignore"):
            m = np.nanmean(others, axis=1) if np.isfinite(others).any() else np.full(len(x), np.nan)
        out[:, k] = x[:, k] - m
    return out


def _slope(t, y):
    ok = np.isfinite(y)
    if ok.sum() < 10:
        return None
    tt, yy = t[ok] - t[ok].mean(), y[ok]
    den = (tt ** 2).sum()
    return None if den == 0 else float((tt * (yy - yy.mean())).sum() / den)


def window_features(t, v, z, reg, temp, i) -> np.ndarray:
    n = v.shape[1]
    d = _relative(v) * 1000.0
    tdev = _relative(temp)
    rest = reg == 0
    load = reg != 0
    feats = np.zeros((n, len(FEATURES)))
    cur_ok = np.isfinite(i)
    for k in range(n):
        zk = z[:, k]
        dk = d[:, k]
        f = feats[k]
        f[0:4] = [zk.mean(), zk.std(), zk.min(), zk.max()]
        s = _slope(t[rest], dk[rest]) if rest.sum() >= 10 else None
        f[4], f[5] = (s, 1.0) if s is not None else (0.0, 0.0)
        a, b = dk[load & np.isfinite(dk)], dk[rest & np.isfinite(dk)]
        f[6], f[7] = (a.mean() - b.mean(), 1.0) if len(a) >= 3 and len(b) >= 3 else (0.0, 0.0)
        jumps = np.abs(np.diff(dk))
        f[8] = float(np.nanmax(jumps)) if np.isfinite(jumps).any() else 0.0
        tk = tdev[:, k]
        ok = np.isfinite(tk)
        f[9] = float(tk[ok].mean()) if ok.any() else 0.0
        f[10] = float(ok.mean())
        f[11] = float(tk[ok][-1] - tk[ok][0]) if ok.sum() >= 2 else 0.0
        f[12], f[13] = float((reg == 1).mean()), float((reg == 2).mean())
        vk = v[:, k][np.isfinite(v[:, k])]
        f[14] = float(vk.mean()) if len(vk) else 0.0
        f[15] = float(np.abs(i[cur_ok]).mean()) if cur_ok.any() else 0.0
        f[16] = float(cur_ok.mean())
    return feats


def mlp_forward(model: dict, x: np.ndarray) -> np.ndarray:
    h = (x - np.asarray(model["norm_mean"])) / np.asarray(model["norm_std"])
    layers = model["layers"]
    for k, layer in enumerate(layers):
        h = h @ np.asarray(layer["W"]) + np.asarray(layer["b"])
        if k < len(layers) - 1:
            h = np.maximum(h, 0.0)
    h = h - h.max(axis=1, keepdims=True)
    e = np.exp(h)
    return e / e.sum(axis=1, keepdims=True)


class FaultClassifier:
    def __init__(self, model: dict):
        self.model = model
        self.window = int(model["window_s"])
        self.min_valid = float(model.get("min_valid_frac", 0.5))
        self.classes = model["classes"]
        self.use_current = bool(model.get("use_current", False))
        self.residual = ResidualScorer(model["residual"])
        self.buf = deque(maxlen=self.window)

    def update(self, step: dict) -> dict | None:
        r = self.residual.update(step)
        reg = 1 if step["charge"] else 2 if step["discharge"] else 0
        i = float(step.get("i", np.nan)) if self.use_current else np.nan
        self.buf.append((float(step["t"]), np.asarray(step["v"], float), r["z"] if r["valid"] else np.full(4, np.nan),
                         reg, np.asarray(step.get("temp", [np.nan] * 4), float), i))
        if len(self.buf) < self.window:
            return None
        t, v, z, reg_a, temp, i = (np.array([b[j] for b in self.buf]) for j in range(6))
        valid = np.isfinite(z).all(axis=1)
        if valid.mean() < self.min_valid:
            return None
        zz = np.where(np.isfinite(z), z, 0.0)
        probs = mlp_forward(self.model, window_features(t, v, zz, reg_a, temp, i))
        return {"probs": probs, "label": [self.classes[j] for j in probs.argmax(axis=1)]}


# =========================================================================== load ESR (edge_ref/load_check.py)
def relative_drop_mv(v_rest, v_load) -> np.ndarray:
    drop = (np.asarray(v_rest, float) - np.asarray(v_load, float)) * 1000.0
    n = len(drop)
    return np.array([drop[k] - np.delete(drop, k).mean() for k in range(n)])


class LoadEsrCheck:
    def __init__(self, model: dict):
        self.mean = np.asarray(model["rel_drop_mean_mv"], float)
        self.std = np.asarray(model["rel_drop_std_mv"], float)
        self.threshold = float(model["threshold"])

    def check(self, v_rest, v_load) -> dict:
        vr, vl = np.asarray(v_rest, float), np.asarray(v_load, float)
        if not (np.isfinite(vr).all() and np.isfinite(vl).all()):
            return {"z": np.full(4, np.nan), "alarm": np.zeros(4, bool), "cell": None, "valid": False}
        z = (relative_drop_mv(vr, vl) - self.mean) / self.std
        alarm = z > self.threshold
        return {"z": z, "alarm": alarm, "cell": int(np.argmax(z)) + 1 if alarm.any() else None, "valid": True}


RUNNERS = {"residual": ResidualScorer, "classifier": FaultClassifier, "load_esr": LoadEsrCheck}
MODEL_NAMES = tuple(RUNNERS)


# =========================================================================== OTA (edge_ref/ota_agent.py)
def as_step(inp: dict) -> dict:
    def arr(x):
        return np.array([np.nan if v is None else float(v) for v in x], dtype=float)
    return {"t": float(inp["t"]), "v": arr(inp["v"]), "i": np.nan if inp.get("i") is None else float(inp["i"]),
            "temp": arr(inp.get("temp") or [None] * len(inp["v"])),
            "charge": bool(inp["charge"]), "discharge": bool(inp["discharge"])}


class Http:
    """MLOps 엣지 경로 (OCI nginx /edge-api/ → MLOps). X-Edge-Token 헤더로 인증한다."""

    def __init__(self, base_url: str, token: str | None, timeout: float = 20):
        self.base, self.token, self.timeout = base_url.rstrip("/"), token, timeout

    def _headers(self, extra=None) -> dict:
        h = {"User-Agent": "cellguard-edge"}
        if self.token:
            h["X-Edge-Token"] = self.token
        h.update(extra or {})
        return h

    def get(self, path: str):
        req = urllib.request.Request(self.base + path, headers=self._headers())
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return r.status, dict(r.headers), r.read()
        except urllib.error.HTTPError as e:
            return e.code, dict(e.headers), e.read()

    def post_json(self, path: str, body: dict) -> int:
        req = urllib.request.Request(self.base + path, data=json.dumps(body).encode("utf-8"), method="POST",
                                     headers=self._headers({"Content-Type": "application/json"}))
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return r.status
        except urllib.error.HTTPError as e:
            return e.code


def _header(headers: dict, key: str):
    return next((v for k, v in headers.items() if k.lower() == key.lower()), None)


class ModelSlot:
    """모델 하나의 설치 폴더: <root>/<name>/{current,previous,incoming,broken}."""

    def __init__(self, root: Path, name: str):
        self.name = name
        self.root = Path(root) / name
        self.factory = RUNNERS[name]

    @property
    def current_version(self):
        p = self.root / "current" / "manifest.json"
        try:
            return json.loads(p.read_text(encoding="utf-8"))["version"]
        except (OSError, ValueError, KeyError):
            return None

    def model(self) -> dict | None:
        p = self.root / "current" / "model.json"
        try:
            return json.loads(p.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None

    def runner(self, calibration: dict | None = None):
        m = self.model()
        if m is None:
            return None
        if self.name == "residual" and m.get("pack_normalized"):
            return self.factory(m, calibration)
        return self.factory(m)

    def verify(self, data: bytes, expected_sha: str | None):
        """(ok, 설명, 파일들, manifest). 체크섬 2단계 + 테스트 벡터."""
        if expected_sha is None or hashlib.sha256(data).hexdigest() != expected_sha:
            return False, "package checksum mismatch", {}, None
        try:
            with zipfile.ZipFile(io.BytesIO(data)) as z:
                files = {n: z.read(n) for n in z.namelist()}
            manifest = json.loads(files["manifest.json"])
        except (zipfile.BadZipFile, KeyError, ValueError) as e:
            return False, f"bad package: {e!r}", {}, None
        if manifest.get("name") not in (None, self.name):
            return False, f"package is for model {manifest.get('name')}, not {self.name}", {}, None
        for fname, sha in (manifest.get("files") or {}).items():
            if hashlib.sha256(files.get(fname, b"")).hexdigest() != sha:
                return False, f"{fname} checksum mismatch", {}, None
        if "model.json" not in files:
            return False, "model.json missing", {}, None
        if "test_vectors.json" in files:
            ok, detail = self.run_test_vectors(json.loads(files["model.json"]), json.loads(files["test_vectors.json"]))
            if not ok:
                return False, detail, {}, None
        return True, "checksums and test vectors passed", files, manifest

    def run_test_vectors(self, model: dict, vectors: dict):
        try:
            cal = vectors.get("calibration")
            runner = self.factory(model, cal) if cal is not None else self.factory(model)
            tol = vectors["tolerance"]
            for i, step in enumerate(vectors["steps"]):
                inp = step["input"]
                out = runner.check(inp["v_rest"], inp["v_load"]) if hasattr(runner, "check") else runner.update(as_step(inp))
                expected = step["expected"]
                if expected is None:
                    if out is not None:
                        return False, f"test vector {i}: output mismatch (train/serve skew)"
                    continue
                for key, want in expected.items():
                    got = None if out is None else out.get(key)
                    if got is None or not np.allclose(np.asarray(got, float), np.asarray(want, float), atol=tol):
                        return False, f"test vector {i}: '{key}' mismatch (train/serve skew)"
        except Exception as e:  # a broken package must never take the service down
            return False, f"test vectors raised {e!r}"
        return True, ""

    def install(self, files: dict) -> None:
        """incoming에 다 쓴 뒤 폴더 이름만 바꾼다. 중간에 꺼져도 반쯤 적용된 모델이 남지 않는다."""
        current, previous, incoming = self.root / "current", self.root / "previous", self.root / "incoming"
        self.root.mkdir(parents=True, exist_ok=True)
        if incoming.exists():
            shutil.rmtree(incoming)
        incoming.mkdir()
        for name, data in files.items():
            (incoming / name).write_bytes(data)
        if previous.exists():
            shutil.rmtree(previous)
        if current.exists():
            current.rename(previous)
        incoming.rename(current)

    def rollback(self) -> str | None:
        current, previous, broken = self.root / "current", self.root / "previous", self.root / "broken"
        if not previous.exists():
            return None
        if broken.exists():
            shutil.rmtree(broken)
        if current.exists():
            current.rename(broken)
        previous.rename(current)
        return self.current_version


class Ota:
    """pull 방식 OTA. 운영(production) 버전이 바뀌면 받아서 검증하고 바꾼다. 결과는 MLOps와 MQTT 둘 다로 보고한다."""

    def __init__(self, root: Path, pack_id: str, http: Http | None, report):
        self.root = Path(root)
        self.pack_id = pack_id
        self.http = http
        self.report = report          # report(model, version, status, detail) -> MQTT diag
        self.slots = {n: ModelSlot(self.root, n) for n in MODEL_NAMES}
        self.last_check = None
        self.last_result: dict = {}

    def versions(self) -> dict:
        return {n: s.current_version for n, s in self.slots.items()}

    def calibration(self):
        try:
            return json.loads((self.root / "calibration.json").read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None

    def check_all(self) -> dict:
        """모든 모델 확인. {이름: 결과}. 하나라도 새로 적용되면 result["changed"]=True."""
        self.last_check = time.time()
        out = {}
        for name in MODEL_NAMES:
            try:
                out[name] = self.check(name)
            except (OSError, ValueError) as e:      # network down, DNS, TLS, bad JSON: keep the current model
                out[name] = {"action": "error", "detail": repr(e)[:200]}
        if self.http is not None:
            try:
                st, _, body = self.http.get(f"/packs/{self.pack_id}/calibration")
                if st == 200:
                    self.root.mkdir(parents=True, exist_ok=True)
                    (self.root / "calibration.json").write_bytes(body)
            except OSError:
                pass
        self.last_result = out
        return out

    def check(self, name: str) -> dict:
        if self.http is None:
            return {"action": "none", "detail": "mlops_url not configured"}
        slot = self.slots[name]
        st, _, body = self.http.get(f"/models/{name}/production")
        if st == 404:
            return {"action": "none", "detail": "no production version on the server", "current": slot.current_version}
        if st != 200:
            return {"action": "error", "detail": f"HTTP {st} on production lookup"}
        try:
            latest = json.loads(body)["version"]
        except (ValueError, KeyError, TypeError):
            return {"action": "error", "detail": "production lookup did not return model JSON "
                                                 "(is /edge-api/ exposed on the server?)"}
        if latest == slot.current_version:
            return {"action": "none", "detail": "up to date", "current": latest}
        st, headers, data = self.http.get(f"/models/{name}/{latest}/package")
        if st != 200:
            return self._result(name, latest, "failed", f"package download HTTP {st}")
        return self.apply(name, data, _header(headers, "X-Package-SHA256"), latest)

    def apply(self, name: str, data: bytes, sha: str | None, version: str | None = None) -> dict:
        slot = self.slots[name]
        ok, detail, files, manifest = slot.verify(data, sha)
        version = version or (manifest or {}).get("version") or "?"
        if not ok:
            return self._result(name, version, "failed", detail)
        slot.install(files)
        try:                                   # the model must also load with the live settings
            slot.runner(self.calibration())
        except Exception as e:
            back = slot.rollback()
            return self._result(name, version, "rolled_back", f"runner failed {e!r}; back to {back}")
        return self._result(name, version, "applied", detail)

    def install_file(self, name: str, path: Path) -> dict:
        """서버 없이 패키지 zip 파일로 설치 (현장 설치, 시험용). 체크섬은 zip 자신으로 계산하고 파일별 체크섬과 테스트 벡터는 그대로 본다."""
        data = Path(path).read_bytes()
        return self.apply(name, data, hashlib.sha256(data).hexdigest())

    def _result(self, name, version, status, detail) -> dict:
        body = {"pack_id": self.pack_id, "model": name, "version": version, "status": status,
                "ts": time.time(), "detail": detail}
        if self.http is not None:
            try:
                self.http.post_json("/edge/versions", body)
            except OSError:
                pass
        if self.report:
            self.report(name, version, status, detail)
        return {"action": status, "version": version, "detail": detail}


# =========================================================================== detection
CLASS_TO_FAULT = {"esr": "ESR", "leak": "LEAK", "heat": "HEAT"}


class Detector:
    """모델 출력 → 확정된 이상 판정. 순수 로직이라 단독 시험이 된다.

    - ② residual: 셀 경보가 confirm_steps번 이어지면 확정. 점수 중 온도 몫이 크면 HEAT, 아니면 ANOMALY
    - ③ classifier: 같은 고장 클래스가 confirm_windows번 이어지고 확률 >= min_prob 이면 확정 (report_classes만)
    - load_esr: 부하 측정 한 번의 경보로 바로 확정 (측정이 드물다)
    해제: 경보가 clear_steps번 연속으로 없을 때. 같은 (모델, 셀)은 해제 전까지 다시 보고하지 않는다.
    """

    def __init__(self, cfg: dict):
        self.res_confirm = int(cfg.get("ai_residual_confirm_steps", 3))
        self.cls_confirm = int(cfg.get("ai_classifier_confirm_windows", 3))
        self.min_prob = float(cfg.get("ai_classifier_min_prob", 0.6))
        self.report_classes = set(cfg.get("ai_classifier_report", ["leak", "heat"]))
        self.clear_steps = int(cfg.get("ai_clear_steps", 10))
        self.esr_hold_s = float(cfg.get("ai_load_esr_hold_s", 60))
        self.reset()

    def reset(self):
        self.res_run = np.zeros(4, int)
        self.res_quiet = np.zeros(4, int)
        self.res_active = [False] * 4
        self.cls_run = [0] * 4
        self.cls_last = [None] * 4
        self.cls_quiet = [0] * 4
        self.cls_active = [None] * 4
        self.esr_active_until = [0.0] * 4

    def residual(self, out: dict) -> list[dict]:
        events = []
        alarm = np.asarray(out["alarm"], bool)
        for k in range(4):
            if alarm[k]:
                self.res_run[k] += 1
                self.res_quiet[k] = 0
                if not self.res_active[k] and self.res_run[k] >= self.res_confirm:
                    self.res_active[k] = True
                    heat = float(out["score_t"][k]) >= float(out["score_v"][k])
                    events.append({"model": "residual", "event": "detected", "cell": k + 1,
                                   "fault_type": "HEAT" if heat else "ANOMALY",
                                   "score": round(float(out["score"][k]), 3),
                                   "score_v": round(float(out["score_v"][k]), 3),
                                   "score_t": round(float(out["score_t"][k]), 3)})
            else:
                self.res_run[k] = 0
                self.res_quiet[k] += 1
                if self.res_active[k] and self.res_quiet[k] >= self.clear_steps:
                    self.res_active[k] = False
                    events.append({"model": "residual", "event": "cleared", "cell": k + 1})
        return events

    def classifier(self, out: dict | None) -> list[dict]:
        events = []
        if out is None:
            return events
        probs = np.asarray(out["probs"], float)
        for k, label in enumerate(out["label"]):
            p = float(probs[k].max())
            fault = label if label in self.report_classes and p >= self.min_prob else None
            if fault is not None and fault == self.cls_last[k]:
                self.cls_run[k] += 1
            else:
                self.cls_run[k] = 1 if fault else 0
            self.cls_last[k] = fault
            if fault:
                self.cls_quiet[k] = 0
                if self.cls_active[k] != fault and self.cls_run[k] >= self.cls_confirm:
                    self.cls_active[k] = fault
                    events.append({"model": "classifier", "event": "detected", "cell": k + 1,
                                   "fault_type": CLASS_TO_FAULT[fault], "class": fault, "prob": round(p, 3),
                                   "probs": [round(float(x), 3) for x in probs[k]]})
            else:
                self.cls_quiet[k] += 1
                if self.cls_active[k] and self.cls_quiet[k] >= self.clear_steps:
                    events.append({"model": "classifier", "event": "cleared", "cell": k + 1,
                                   "class": self.cls_active[k]})
                    self.cls_active[k] = None
        return events

    def load_esr(self, out: dict, now: float) -> list[dict]:
        events = []
        if not out.get("valid"):
            return events
        z = np.asarray(out["z"], float)
        for k in np.flatnonzero(np.asarray(out["alarm"], bool)):
            if now >= self.esr_active_until[k]:
                events.append({"model": "load_esr", "event": "detected", "cell": int(k) + 1, "fault_type": "ESR",
                               "z": round(float(z[k]), 3), "z_all": [round(float(x), 3) for x in z]})
            self.esr_active_until[k] = now + self.esr_hold_s
        return events


class AiEngine(threading.Thread):
    """측정을 큐로 받아 별도 스레드에서 추론한다 (UART 수신 스레드를 막지 않는다).

    feed_sample(): CGT 1초 측정마다,  feed_loaded(): CGP 부하 측정마다,  reload(): OTA 적용 뒤.
    판정은 on_event(event_dict)로 넘긴다 (Edge가 ground truth를 붙여 MQTT로 보고).
    """

    REST_MAX_GAP_MS = 3000

    def __init__(self, ota: Ota, cfg: dict, on_event, log=print):
        super().__init__(daemon=True, name="ai")
        self.ota, self.cfg, self.on_event, self.log = ota, cfg, on_event, log
        self.q: queue.Queue = queue.Queue(maxsize=600)
        self.detector = Detector(cfg)
        self.runners: dict = {}
        self.versions: dict = {}
        self.need_reload = True
        self.last_rest = None          # (nano_ms, v[4]) of the latest rest CGT
        self.stats = {"samples": 0, "loaded": 0, "dropped": 0, "errors": 0, "detections": 0, "busy_ms": 0.0}
        self.last_error = None
        self.last_out: dict = {}

    # -- producer side (any thread)
    def feed_sample(self, step: dict, nano_ms: int | None):
        self._put(("cgt", step, nano_ms))

    def feed_loaded(self, v_load, nano_ms: int, direction: str, esr_mask: int):
        self._put(("cgp", v_load, nano_ms, direction, esr_mask))

    def reload(self):
        self._put(("reload",))

    def _put(self, item):
        try:
            self.q.put_nowait(item)
        except queue.Full:
            self.stats["dropped"] += 1

    # -- worker
    def _load(self):
        self.need_reload = False
        cal = self.ota.calibration()
        runners, versions = {}, {}
        for name, slot in self.ota.slots.items():
            try:
                r = slot.runner(cal)
            except Exception as e:
                self.last_error = f"{name}: {e!r}"
                r = None
            if r is not None:
                runners[name] = r
                versions[name] = slot.current_version
        self.runners, self.versions = runners, versions
        self.detector.reset()
        self.log(f"AI models loaded: {versions or 'none'}")

    def run(self):
        while True:
            item = self.q.get()
            try:
                if item[0] == "reload" or self.need_reload:
                    self._load()
                    if item[0] == "reload":
                        continue
                t0 = time.perf_counter()
                if item[0] == "cgt":
                    self._sample(item[1], item[2])
                elif item[0] == "cgp":
                    self._loaded(*item[1:])
                self.stats["busy_ms"] = round(0.9 * self.stats["busy_ms"] + 0.1 * (time.perf_counter() - t0) * 1000, 2)
            except Exception as e:      # one bad sample must not kill inference
                self.stats["errors"] += 1
                self.last_error = repr(e)[:300]

    def _emit(self, events):
        for ev in events:
            ev["version"] = self.versions.get(ev["model"])
            if ev["event"] == "detected":
                self.stats["detections"] += 1
            self.on_event(ev)

    def _sample(self, step: dict, nano_ms):
        self.stats["samples"] += 1
        v = step["v"]
        if nano_ms is not None and not step["charge"] and not step["discharge"] and np.isfinite(v).all():
            self.last_rest = (int(nano_ms), np.array(v, float))
        r = self.runners.get("residual")
        if r is not None:
            out = r.update(step)
            self.last_out["residual"] = {"score": [round(float(x), 2) for x in out["score"]],
                                         "threshold": r.threshold}
            self._emit(self.detector.residual(out))
        c = self.runners.get("classifier")
        if c is not None:
            out = c.update(step)
            if out is not None:
                self.last_out["classifier"] = {"label": out["label"]}
            self._emit(self.detector.classifier(out))

    def _loaded(self, v_load, nano_ms, direction, esr_mask):
        self.stats["loaded"] += 1
        chk = self.runners.get("load_esr")
        if chk is None or self.last_rest is None:
            return
        rest_ms, v_rest = self.last_rest
        gap = int(nano_ms) - rest_ms
        if gap < 0 or gap > self.REST_MAX_GAP_MS:
            return
        out = chk.check(v_rest, v_load)
        self.last_out["load_esr"] = {"z": [round(float(x), 2) for x in out["z"]], "rest_gap_ms": gap}
        events = self.detector.load_esr(out, time.monotonic())
        for ev in events:
            ev.update(direction=direction, esr_mask=esr_mask, rest_gap_ms=gap)
        self._emit(events)

    def status(self) -> dict:
        return {"models": self.versions, "stats": dict(self.stats), "last_error": self.last_error,
                "queue": self.q.qsize(), "last": self.last_out,
                "active": {"residual": [k + 1 for k in range(4) if self.detector.res_active[k]],
                           "classifier": {str(k + 1): c for k, c in enumerate(self.detector.cls_active) if c}}}


def step_from_cgt(t: float, cells_mv, temps_c, temp_quality, voltage_valid: bool, charge: bool, drain: bool) -> dict:
    """CGT 한 줄 → 추론 입력. MLOps step_from_record(normalized_sample_v1)와 같은 규칙:
    전압 품질이 VALID가 아니면 전압 전부 NaN, 온도는 셀별 품질이 VALID일 때만, 전류 없음(NaN)."""
    v = np.array([np.nan if (c is None or not voltage_valid) else c / 1000.0 for c in (cells_mv or [None] * 4)])
    temp = np.array([t_ if (t_ is not None and q == "VALID") else np.nan for t_, q in zip(temps_c, temp_quality)],
                    dtype=float)
    return {"t": float(t), "v": v, "i": np.nan, "temp": temp, "charge": bool(charge), "discharge": bool(drain)}
