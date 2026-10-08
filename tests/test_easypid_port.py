"""The Python port of easyPID behaves like the C++ library.

Two kinds of tests: (1) the documented behaviours of easyPID 1.1.0, checked on the
port alone; (2) a cross-check against the real C++ sources compiled on the host
(skipped when they are not available), driven by random command scripts.
"""

from __future__ import annotations

import math
import subprocess

import numpy as np
import pytest

from bootcamp import easypid as ep
from bootcamp.plants import rc_ladder


def make(kp=2.0, ki=0.5, kd=0.1, lo=0.0, hi=255.0):
    clk = ep.VirtualClock()
    pid = ep.PIDController(kp, ki, kd, lo, hi, clock=clk)
    pid.begin()
    return pid, clk


# ------------------------------------------------------------- documented behaviour
def test_no_derivative_kick_on_first_update():
    pid, _ = make(kp=0.0, ki=0.0, kd=1.0, lo=-1e6, hi=1e6)
    assert pid.update(100.0, 0.0, 10) == 0.0
    assert pid.getDterm() == 0.0
    pid.reset()
    assert pid.update(50.0, 0.0, 10) == 0.0


def test_terms_reconstruct_output_when_unsaturated():
    pid, _ = make(lo=-1e6, hi=1e6)
    for k in range(20):
        out = pid.update(10.0, 0.4 * k, 20)
        assert math.isclose(out, pid.getPterm() + pid.getIterm() + pid.getDterm(), abs_tol=1e-9)


def test_dt_is_in_milliseconds():
    pid, _ = make(kp=0.0, ki=1.0, kd=0.0, lo=-1e6, hi=1e6)
    pid.update(1.0, 0.0, 1000)  # one second of error 1 -> integral 1
    assert math.isclose(pid.getIterm(), 1.0)


def test_non_positive_dt_returns_previous_output():
    pid, _ = make()
    out = pid.update(10.0, 0.0, 10)
    assert pid.update(99.0, 0.0, 0) == out
    assert pid.update(99.0, 0.0, -5) == out


def test_automatic_timing_ignores_calls_within_the_same_millisecond():
    pid, clk = make(kp=0.0, ki=1.0, kd=0.0, lo=-1e6, hi=1e6)
    clk.advance(10)
    first = pid.update(1.0, 0.0)
    for _ in range(100):  # a fast loop() calls update() many times per millisecond
        assert pid.update(1.0, 0.0) == first
    assert math.isclose(pid.getIterm(), 0.010)


def test_clamp_anti_windup_lets_the_integrator_unwind():
    pid, _ = make(kp=1.0, ki=5.0, kd=0.0, lo=0.0, hi=10.0)
    for _ in range(200):  # unreachable setpoint: output pinned at the top rail
        pid.update(100.0, 0.0, 10)
    assert pid.getOutput() == 10.0
    i_sat = pid.getIterm()
    assert i_sat <= 10.0 + 1e-9  # integration inhibited while pushing into saturation
    for _ in range(50):  # error reverses: the output must leave the rail quickly
        pid.update(0.0, 5.0, 10)
    assert pid.getOutput() < 10.0


def test_no_anti_windup_winds_up():
    pid, _ = make(kp=1.0, ki=5.0, kd=0.0, lo=0.0, hi=10.0)
    pid.setAntiWindup(ep.ANTIWINDUP_NONE)
    for _ in range(200):
        pid.update(100.0, 0.0, 10)
    assert pid.getIterm() > 900.0


def test_backcalc_with_zero_ki_stays_finite():
    pid, _ = make(kp=3.0, ki=0.0, kd=0.2, lo=-1.0, hi=1.0)
    pid.setAntiWindup(ep.ANTIWINDUP_BACKCALC)
    for k in range(100):
        out = pid.update(50.0, k * 0.1, 10)
        assert math.isfinite(out)


def test_reverse_keeps_error_sign_and_negates_terms():
    pid, _ = make(kp=2.0, ki=0.0, kd=0.0, lo=-100, hi=100)
    pid.setDirection(ep.REVERSE)
    out = pid.update(10.0, 4.0, 10)
    assert pid.getError() == 6.0
    assert out == -12.0 == pid.getPterm()


def test_filter_alpha_is_clamped():
    pid, _ = make()
    pid.setDerivativeFilter(ep.FILTER_EMA, 1.0)
    assert pid.filterAlpha_ == 0.999
    pid.setDerivativeFilter(ep.FILTER_EMA, -3)
    assert pid.filterAlpha_ == 0.0


def test_invalid_limits_are_ignored():
    pid, _ = make(lo=0, hi=255)
    pid.setOutputLimits(10, 10)
    pid.setIntegralLimits(5, -5)
    assert (pid.outMin_, pid.outMax_) == (0, 255)
    assert not pid.integralLimitsSet_


def test_tuning_rules_match_the_library_table():
    Ku, Pu = 10.0, 2.0
    assert ep.tuning_rule_gains(Ku, Pu, ep.TUNING_ZIEGLER_NICHOLS) == pytest.approx((6.0, 6.0, 1.5))
    kp = Ku / 2.2
    assert ep.tuning_rule_gains(Ku, Pu, ep.TUNING_TYREUS_LUYBEN) == pytest.approx(
        (kp, kp / 4.4, kp * 2 / 6.3)
    )


def _relay_run(tuner, plant, clk, y, nb=0.0):
    assert tuner.start(2.5, 1.0, nb, 2.5)
    for _ in range(20000):
        u = tuner.update(y)
        if tuner.getState() != ep.TUNER_RELAY_STEP:
            break
        clk.advance(10)
        y = plant.step(u, 0.01)
    return y


def test_relay_tuner_finds_the_rc_ladder_ultimate_point():
    """A 3-stage RC ladder has 1/(1 + 6 sT + 5 s^2T^2 + s^3T^3): Ku = 29 at w = sqrt(6)/RC.

    easyPID averages the cycles from the first relay edge on, while the limit cycle of
    this plant is still growing, so a first run from rest is biased (Ku high, Pu low).
    Starting the tuner a second time, with the plant already oscillating, measures the
    settled limit cycle. Lab 2 uses exactly this two-pass procedure.
    """
    plant = rc_ladder(10e3, 100e-6, 3)  # RC = 1 s
    clk = ep.VirtualClock()
    pid = ep.PIDController(1.0, 0.0, 0.0, 0.0, 5.0, clock=clk)
    tuner = ep.PIDTuner(pid)
    pid.begin()
    y = 0.0
    for _ in range(3000):  # bring the plant to the operating point first
        y = plant.step(2.5, 0.01)
    y = _relay_run(tuner, plant, clk, y)
    assert tuner.isComplete()
    first = (tuner.getUltimateGain(), tuner.getUltimatePeriod())
    _relay_run(tuner, plant, clk, y)
    assert tuner.isComplete()
    second = (tuner.getUltimateGain(), tuner.getUltimatePeriod())
    pu_theory = 2 * math.pi / math.sqrt(6)
    assert first[0] > 1.1 * 29 and first[1] < 0.95 * pu_theory  # the documented bias
    assert second[0] == pytest.approx(29.0, rel=0.08)
    assert second[1] == pytest.approx(pu_theory, rel=0.05)


# ------------------------------------------------------------- C++ cross-check
def _random_script(rng: np.random.Generator) -> list[str]:
    lo, hi = (0.0, 255.0) if rng.random() < 0.5 else (-50.0, 50.0)
    kp, ki, kd = rng.uniform(0, 4), rng.uniform(0, 3), rng.uniform(0, 0.4)
    cmds = [f"new {kp:.3f} {ki:.3f} {kd:.3f} {lo} {hi}", "begin"]
    cmds.append(f"aw {rng.integers(0, 3)}")
    if rng.random() < 0.6:
        cmds.append(f"filt 1 {rng.uniform(0, 0.95):.3f}")
    if rng.random() < 0.3:
        cmds.append("dir 1")
    if rng.random() < 0.3:
        cmds.append(f"ilim {-rng.uniform(1, 30):.2f} {rng.uniform(1, 30):.2f}")
    y = 0.0
    sp = float(rng.uniform(-40, 120))
    for k in range(300):
        y += float(rng.normal(0, 3))
        if k % 60 == 0:
            sp = float(rng.uniform(-40, 120))
        roll = rng.random()
        if roll < 0.75:
            dt = 0 if rng.random() < 0.03 else int(rng.integers(1, 50))
            cmds.append(f"upd {sp:.2f} {y:.2f} {dt}")
        elif roll < 0.9:
            cmds.append(f"adv {int(rng.integers(0, 30))}")
            cmds.append(f"auto {sp:.2f} {y:.2f}")
        elif roll < 0.94:
            cmds.append(
                f"tun {rng.uniform(0, 4):.3f} {rng.uniform(0, 3):.3f} {rng.uniform(0, 0.4):.3f}"
            )
        elif roll < 0.97:
            cmds.append(f"dir {rng.integers(0, 2)}")
        elif roll < 0.99:
            cmds.append(f"olim {lo + rng.uniform(0, 10):.2f} {hi - rng.uniform(0, 10):.2f}")
        else:
            cmds.append("reset")
    return cmds


def _run_port(cmds: list[str]) -> list[list[float]]:
    clk = ep.VirtualClock()
    pid = None
    out = []
    for line in cmds:
        c, *a = line.split()
        v = [float(x) for x in a]
        if c == "new":
            pid = ep.PIDController(*v, clock=clk)
        elif c == "begin":
            pid.begin()
        elif c == "reset":
            pid.reset()
        elif c == "aw":
            pid.setAntiWindup(int(v[0]))
        elif c == "filt":
            pid.setDerivativeFilter(int(v[0]), v[1])
        elif c == "dir":
            pid.setDirection(int(v[0]))
        elif c == "olim":
            pid.setOutputLimits(*v)
        elif c == "ilim":
            pid.setIntegralLimits(*v)
        elif c == "tun":
            pid.setTunings(*v)
        elif c == "adv":
            clk.advance(int(v[0]))
        elif c in ("upd", "auto"):
            o = pid.update(*v)
            out.append([o, pid.getPterm(), pid.getIterm(), pid.getDterm(), pid.getError()])
    return out


def _run_cpp(harness, cmds: list[str]) -> list[list[float]]:
    res = subprocess.run(
        [str(harness)], input="\n".join(cmds) + "\n", capture_output=True, text=True, check=True
    )
    return [[float(x) for x in line.split()] for line in res.stdout.splitlines()]


@pytest.mark.cpp
@pytest.mark.parametrize("seed", range(12))
def test_port_matches_cpp_library(easypid_harness, seed):
    cmds = _random_script(np.random.default_rng(seed))
    py = np.array(_run_port(cmds))
    cc = np.array(_run_cpp(easypid_harness, cmds))
    assert py.shape == cc.shape
    scale = np.maximum(1.0, np.abs(cc))
    worst = np.max(np.abs(py - cc) / scale)
    assert worst < 2e-3, f"port and C++ differ by {worst:.2e} (relative)"


@pytest.mark.cpp
def test_tuner_port_matches_cpp_library(easypid_harness):
    """Closed relay loop on a 3-stage RC ladder: the C++ tuner, fed the measurements the
    port produced, must output the same relay sequence and the same Ku, Pu and gains."""
    plant = rc_ladder(10e3, 100e-6, 3)
    clk = ep.VirtualClock()
    pid = ep.PIDController(1.0, 0.0, 0.0, 0.0, 5.0, clock=clk)
    tuner = ep.PIDTuner(pid)
    pid.begin()
    y = 0.0
    for _ in range(3000):
        y = plant.step(2.5, 0.01)
    cmds = ["new 1 0 0 0 5", "begin", "tstart 2.5 1 0.02 2.5"]
    tuner.start(2.5, 1.0, 0.02, 2.5)
    outs = []
    for _ in range(3000):
        ym = round(y, 4)
        cmds.append(f"tupd {ym}")
        u = tuner.update(ym)
        outs.append([u, int(tuner.getState()), tuner.getProgress()])
        if tuner.getState() != ep.TUNER_RELAY_STEP:
            break
        cmds.append("adv 10")
        clk.advance(10)
        y = plant.step(u, 0.01)
    rules = [f"tres {r}" for r in range(4)]
    res = subprocess.run(
        [str(easypid_harness)],
        input="\n".join(cmds + rules) + "\n",
        capture_output=True,
        text=True,
        check=True,
    )
    lines = res.stdout.splitlines()
    assert lines[0] == "1"
    cc = np.array([[float(x) for x in ln.split()] for ln in lines[1 : 1 + len(outs)]])
    assert np.allclose(cc, np.array(outs), rtol=1e-4, atol=1e-5)
    assert tuner.isComplete()
    for r, ln in enumerate(lines[1 + len(outs) :]):
        ok, kp, ki, kd, Ku, Pu = (float(x) for x in ln.split())
        okp, pkp, pki, pkd = tuner.getTunings(r)
        assert ok == 1 and okp
        assert (kp, ki, kd, Ku, Pu) == pytest.approx(
            (pkp, pki, pkd, tuner.getUltimateGain(), tuner.getUltimatePeriod()), rel=1e-4
        )
