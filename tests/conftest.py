# CellGuard data collector - test fixtures
# Author: Yongho Ahn <nanhosoft@gmail.com>
import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="session")
def normalized_schema():
    return json.loads((ROOT / "schemas" / "normalized_sample_v1.schema.json").read_text())


@pytest.fixture(scope="session")
def diagnostic_schema():
    return json.loads((ROOT / "schemas" / "nano_diagnostic_v1.schema.json").read_text())


@pytest.fixture(scope="session")
def sim_path() -> str:
    """Builds the firmware host simulator from an arudino-firmware checkout.

    Set CELLGUARD_FIRMWARE_DIR, or keep the firmware repo next to this one.
    Integration tests are skipped when neither is available.
    """
    fw = Path(os.environ.get("CELLGUARD_FIRMWARE_DIR", ROOT.parent / "arudino-firmware"))
    host = fw / "tests" / "host"
    if not (host / "Makefile").exists() or shutil.which(os.environ.get("CXX", "g++")) is None:
        pytest.skip("firmware checkout / C++ compiler not available for the host simulator")
    subprocess.run(["make", "-s", "-C", str(host), "sim"], check=True)
    return str(host / "build" / "nano_host_sim")


FW016 = ROOT / "tests" / "fw016"


@pytest.fixture(scope="session")
def sim016_path() -> str:
    """Builds the 0.1.6-LOAD-VERIFY host simulator: the unmodified rig firmware (tests/fw016) + a plant model."""
    cxx = os.environ.get("CXX", "g++")
    if shutil.which(cxx) is None:
        pytest.skip("C++ compiler not available for the 0.1.6 host simulator")
    src = FW016 / "CellGuardNano_LoadVerify"
    out = FW016 / "build" / "sim016"
    out.parent.mkdir(exist_ok=True)
    sources = [FW016 / "sim016.cpp"] + [src / f for f in ("cg_controller.cpp", "cg_measure.cpp", "cg_outputs.cpp",
                                                          "cg_protocol.cpp", "cg_crc16.cpp")]
    if not out.exists() or out.stat().st_mtime < max(f.stat().st_mtime for f in sources):
        subprocess.run([cxx, "-std=c++17", "-O1", "-DCG_HOST_SIM_BUILD=1", f"-I{src}", "-o", str(out),
                        *map(str, sources)], check=True)
    return str(out)
