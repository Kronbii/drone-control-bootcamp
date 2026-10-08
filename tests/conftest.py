import os
import shutil
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def easypid_src() -> Path | None:
    """Where the easyPID C++ sources are: $EASYPID_SRC, a sibling checkout, or the
    Arduino sketchbook (``arduino-cli lib install easyPID@1.1.0``)."""
    cands = []
    if os.environ.get("EASYPID_SRC"):
        cands.append(Path(os.environ["EASYPID_SRC"]))
    cands += [
        ROOT.parent / "easyPID" / "src",
        Path.home() / "Arduino" / "libraries" / "easyPID" / "src",
        Path.home()
        / "snap"
        / "arduino-cli"
        / "current"
        / "Arduino"
        / "libraries"
        / "easyPID"
        / "src",
    ]
    for c in cands:
        if (c / "easyPID.cpp").exists() and (c / "PIDTuner.cpp").exists():
            return c
    return None


@pytest.fixture(scope="session")
def root() -> Path:
    return ROOT


@pytest.fixture(scope="session")
def easypid_harness(tmp_path_factory):
    src = easypid_src()
    cxx = shutil.which("g++") or shutil.which("c++")
    if src is None or cxx is None:
        pytest.skip("easyPID C++ sources or a C++ compiler not available")
    out = tmp_path_factory.mktemp("cpp") / "harness"
    import subprocess

    subprocess.run(
        [
            cxx,
            "-std=c++11",
            "-O1",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(ROOT / "tests" / "cpp"),
            "-I",
            str(src),
            str(ROOT / "tests" / "cpp" / "harness.cpp"),
            str(src / "easyPID.cpp"),
            str(src / "PIDTuner.cpp"),
            "-o",
            str(out),
        ],
        check=True,
    )
    return out
