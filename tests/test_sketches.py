"""The Arduino sketches: shared headers in sync, only the real easyPID API is used,
constants mirrored from the Python side, and (when arduino-cli is available) they compile."""

from __future__ import annotations

import importlib.util
import re
import shutil
import subprocess

import pytest

from bootcamp import capstone, rig
from conftest import ROOT, easypid_src

ARDUINO = ROOT / "arduino"
SKETCHES = sorted(p for p in ARDUINO.iterdir() if p.is_dir() and (p / f"{p.name}.ino").exists())

# Public API of easyPID 1.1.0 (easyPID's AGENTS.md, section 3).
PID_METHODS = {
    "begin",
    "update",
    "setSetpoint",
    "setMeasurement",
    "compute",
    "setTunings",
    "setOutputLimits",
    "setIntegralLimits",
    "setAntiWindup",
    "setDerivativeFilter",
    "setSampleTime",
    "setDirection",
    "reset",
    "getError",
    "getPterm",
    "getIterm",
    "getDterm",
    "getOutput",
}
TUNER_METHODS = {
    "start",
    "update",
    "isComplete",
    "getTunings",
    "applyTunings",
    "getUltimateGain",
    "getUltimatePeriod",
    "cancel",
    "getState",
    "getProgress",
}
CONSTANTS = {
    "ANTIWINDUP_NONE",
    "ANTIWINDUP_CLAMP",
    "ANTIWINDUP_BACKCALC",
    "FILTER_NONE",
    "FILTER_EMA",
    "DIRECT",
    "REVERSE",
    "TUNING_ZIEGLER_NICHOLS",
    "TUNING_TYREUS_LUYBEN",
    "TUNING_PESSEN",
    "TUNING_NO_OVERSHOOT",
    "TUNER_IDLE",
    "TUNER_RELAY_STEP",
    "TUNER_ANALYZING",
    "TUNER_COMPLETE",
    "TUNER_FAILED",
}


def _load_tool(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "tools" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_there_are_sketches_for_the_hardware_sessions():
    names = [s.name for s in SKETCHES]
    for prefix in ("s01", "s02", "s03", "s04", "s07"):
        assert any(n.startswith(prefix) for n in names)


def test_shared_header_copies_are_in_sync():
    assert _load_tool("sync_sketch_headers").main(["--check"]) == 0


def _instances(code: str, cls: str) -> set[str]:
    return set(re.findall(rf"\b{cls}\s+(\w+)\s*\(", code))


@pytest.mark.parametrize("sketch", SKETCHES, ids=[s.name for s in SKETCHES])
def test_sketches_use_only_the_easypid_api(sketch):
    code = (sketch / f"{sketch.name}.ino").read_text()
    for cls, allowed in (("PIDController", PID_METHODS), ("PIDTuner", TUNER_METHODS)):
        for obj in _instances(code, cls):
            used = set(re.findall(rf"\b{obj}\.(\w+)\s*\(", code))
            assert used <= allowed, f"{sketch.name}: {obj} uses {sorted(used - allowed)}"
    for const in set(re.findall(r"\b(ANTIWINDUP_\w+|FILTER_\w+|TUNING_\w+|TUNER_\w+)\b", code)):
        assert const in CONSTANTS, f"{sketch.name}: unknown easyPID constant {const}"


def test_api_list_matches_the_easypid_header_when_available():
    src = easypid_src()
    if src is None:
        pytest.skip("easyPID sources not found")
    header = (src / "easyPID.h").read_text() + (src / "PIDTuner.h").read_text()
    for m in PID_METHODS | TUNER_METHODS:
        assert re.search(rf"\b{m}\s*\(", header), m
    for c in CONSTANTS:
        assert c in header, c


def _const(code: str, name: str) -> str:
    m = re.search(rf"\b{name}\s*(?:\[\d+\])?\s*=\s*([^;]+);", code)
    assert m, name
    return m.group(1)


def _num(s: str) -> float:
    return float(s.strip().rstrip("fFuUlL"))


def test_safety_constants_match_the_python_reference():
    code = (ARDUINO / "common" / "rig_safety.h").read_text()
    for name in (
        "POT_MIN_VALID",
        "POT_MAX_VALID",
        "ARM_MAX_ANGLE_DEG",
        "ARM_MIN_BOOT_MS",
        "TRIP_ANGLE_DEG",
        "TRIP_TIME_MS",
        "US_IDLE",
        "US_CAP",
    ):
        assert _num(_const(code, name)) == pytest.approx(getattr(rig, name)), name


def test_capstone_schedule_matches_the_python_reference():
    code = (ARDUINO / "s07_capstone" / "s07_capstone.ino").read_text()
    t = [_num(x) for x in _const(code, "TEST_T").strip("{} ").split(",")]
    sp = [_num(x) for x in _const(code, "TEST_SP").strip("{} ").split(",")]
    assert list(zip(t, sp)) == [tuple(map(float, x)) for x in capstone.RIG_TEST]
    pairs = {
        "TEST_END_S": capstone.RIG_TEST_END,
        "DIST_TIME_S": capstone.DIST_TIME,
        "DIST_US": capstone.DIST_US,
        "DIST_MS": capstone.DIST_MS,
        "FAULT_TIME_S": capstone.FAULT_TIME,
        "FAULT_ETA": capstone.FAULT_ETA,
        "FAULT_STEP_TIME_S": capstone.FAULT_STEP_TIME,
        "FAULT_STEP_DEG": capstone.FAULT_STEP_DEG,
    }
    for name, value in pairs.items():
        assert _num(_const(code, name)) == pytest.approx(value), name


@pytest.mark.arduino
@pytest.mark.parametrize("sketch", SKETCHES, ids=[s.name for s in SKETCHES])
def test_sketch_compiles_for_uno(sketch, tmp_path):
    cli = shutil.which("arduino-cli")
    if cli is None:
        pytest.skip("arduino-cli not installed")
    cmd = [
        cli,
        "compile",
        "-b",
        "arduino:avr:uno",
        "--warnings",
        "all",
        "--output-dir",
        str(tmp_path),
    ]
    src = easypid_src()
    if src is not None:
        cmd += ["--library", str(src.parent)]
    res = subprocess.run(cmd + [str(sketch)], capture_output=True, text=True)
    if res.returncode != 0 and "Platform 'arduino:avr' not found" in res.stdout + res.stderr:
        pytest.skip("arduino:avr core not installed")
    assert res.returncode == 0, res.stdout + res.stderr
    assert "warning:" not in res.stderr, res.stderr
