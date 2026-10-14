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
