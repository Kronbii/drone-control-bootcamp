"""Capstone acceptance tests (session 7): the same test on the rig and in simulation.

Track R (rig): ``arduino/s07_capstone`` runs :data:`RIG_TEST` when it receives ``test``;
:func:`simulate_rig_test` runs the identical schedule on the rig model, so a team can
check a tuning before touching the hardware. :func:`evaluate_rig_test` turns either log
into the numbers of the spec sheet, and :data:`RIG_SPEC` holds the pass limits.

Track S (simulation): a waypoint course for the quadrotor with a 50 % loss of
effectiveness on rotor 1 half-way through, scored by :func:`evaluate_quad_course`.

The timing constants below are mirrored in ``s07_capstone.ino``; the test-suite checks
that both stay identical.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np

from .metrics import step_info

# ----------------------------------------------------------------- rig test schedule
# (start time s, setpoint deg); the last entry holds until the end
RIG_TEST = (
    (0.0, 0.0),
    (3.0, 15.0),
    (8.0, -10.0),
    (13.0, 0.0),
)
RIG_TEST_END = 33.0
DIST_TIME = 20.0  # s: software disturbance pulse on the differential command
DIST_US = 40.0  # us
DIST_MS = 100  # ms
FAULT_TIME = 25.0  # s: motor-2 throttle scaled to FAULT_ETA (stand-in for a damaged prop)
FAULT_ETA = 0.75
FAULT_STEP_TIME = 29.0  # s: +10 deg step while the fault is active
FAULT_STEP_DEG = 10.0


def rig_test_setpoint(t: float) -> float:
    sp = 0.0
    for t0, v in RIG_TEST:
        if t >= t0:
            sp = v
    if t >= FAULT_STEP_TIME:
        sp = FAULT_STEP_DEG
    return sp


def rig_test_disturbance_us(t: float) -> float:
    return DIST_US if DIST_TIME <= t < DIST_TIME + DIST_MS / 1000.0 else 0.0


@dataclass(frozen=True)
class RigSpec:
    step_overshoot_pct: float = 20.0  # +15 deg step
    step_settling_s: float = 2.0  # into a +-1 deg band
    hold_error_deg: float = 1.0  # mean |error| over the last 1 s of each hold
    dist_peak_deg: float = 8.0
    dist_recovery_s: float = 1.5  # back inside +-1 deg
    fault_error_deg: float = 1.5  # mean |error| 2-4 s after the fault step
    u_rms_us: float = 25.0  # command activity while holding 0 deg (chatter)


RIG_SPEC = RigSpec()


def _band_settling(t, y, target, band) -> float:
    out = np.nonzero(np.abs(y - target) > band)[0]
    if len(out) == 0:
        return 0.0
    if out[-1] == len(y) - 1:
        return math.nan
    return float(t[out[-1] + 1] - t[0])


def evaluate_rig_test(t, sp, angle, u) -> dict:
    """Spec-sheet numbers from a test log (``t`` in s, angles in deg, ``u`` in us)."""
    t, sp, angle, u = (np.asarray(a, dtype=float) for a in (t, sp, angle, u))
    res: dict = {}
    m = (t >= 3.0) & (t < 8.0)
    info = step_info(t[m], angle[m], 15.0, y0=0.0)
    res["step_overshoot_pct"] = info.overshoot_pct
    res["step_settling_s"] = _band_settling(t[m], angle[m], 15.0, 1.0)
    holds = [(2.0, 3.0, 0.0), (7.0, 8.0, 15.0), (12.0, 13.0, -10.0), (19.0, 20.0, 0.0)]
    res["hold_error_deg"] = max(
        float(np.mean(np.abs(angle[(t >= a) & (t < b)] - v))) for a, b, v in holds
    )
    md = (t >= DIST_TIME) & (t < FAULT_TIME)
    dev = np.abs(angle[md])
    res["dist_peak_deg"] = float(dev.max())
    res["dist_recovery_s"] = _band_settling(t[md], angle[md], 0.0, 1.0)
    mf = (t >= FAULT_STEP_TIME + 2.0) & (t < FAULT_STEP_TIME + 4.0)
    res["fault_error_deg"] = float(np.mean(np.abs(angle[mf] - FAULT_STEP_DEG)))
    mh = (t >= 14.0) & (t < DIST_TIME)
    res["u_rms_us"] = float(np.sqrt(np.mean((u[mh] - u[mh].mean()) ** 2)))
    return res


def spec_check(results: dict, spec: RigSpec = RIG_SPEC) -> dict:
    """``{metric: (value, limit, passed)}``; a nan value (did not settle) fails."""
    out = {}
    for k, limit in asdict(spec).items():
        v = results[k]
        out[k] = (v, limit, bool(math.isfinite(v) and v <= limit))
    return out


def simulate_rig_test(
    controller,
    params=None,
    sensors=None,
    seed: int = 0,
    dt: float = 0.005,
    base_us: float = 1350.0,
):
    """Run the acceptance test on the rig model.

    ``controller(t, sp, measurements, dt) -> u_us`` as for :func:`bootcamp.rig.simulate_rig`.
    The fault is applied like the sketch does it: motor 2's throttle above idle is
    scaled by :data:`FAULT_ETA`.
    """
    from .rig import mix, simulate_rig

    def ctrl(t, sp, m, dt_):
        return controller(t, sp, m, dt_) + rig_test_disturbance_us(t)

    def mixer(base, u, lo, hi):
        m1, m2 = mix(base, u, lo, hi)
        if mixer.t >= FAULT_TIME:
            m2 = lo + FAULT_ETA * (m2 - lo)
        return m1, m2

    mixer.t = 0.0

    def ctrl_with_clock(t, sp, m, dt_):
        mixer.t = t
        return ctrl(t, sp, m, dt_)

    return simulate_rig(
        ctrl_with_clock,
        RIG_TEST_END,
        setpoint=rig_test_setpoint,
        dt=dt,
        base_us=base_us,
        params=params,
        sensors=sensors,
        seed=seed,
        mixer=mixer,
    )


# ----------------------------------------------------------------- quadrotor course
COURSE = ((0.0, 0.0, 1.0), (2.0, 0.0, 1.5), (2.0, 2.0, 1.5), (0.0, 2.0, 1.0), (0.0, 0.0, 1.0))
COURSE_DWELL = 4.0  # s per waypoint
COURSE_FAULT_TIME = 9.0  # s: 50 % loss of effectiveness on rotor 1
COURSE_FAULT_ETA = 0.5


@dataclass(frozen=True)
class QuadSpec:
    max_error_after_fault_m: float = 0.6  # distance to the current waypoint, >= 2 s into each leg
    final_error_m: float = 0.10  # mean over the last 1 s
    crashed: bool = False


def course_eta(t: float) -> np.ndarray:
    return np.array([COURSE_FAULT_ETA if t >= COURSE_FAULT_TIME else 1.0, 1.0, 1.0, 1.0])


def evaluate_quad_course(log) -> dict:
    t, p, ref = log.t, log.p, log.p_ref
    err = np.linalg.norm(p - ref, axis=1)
    since_switch = np.mod(t, COURSE_DWELL)
    m = (t >= COURSE_FAULT_TIME) & (since_switch >= 2.0)  # once each leg has had 2 s
    return {
        "crashed": bool(log.crashed),
        "max_error_after_fault_m": float(err[m].max()) if m.any() else math.nan,
        "final_error_m": float(err[t >= t[-1] - 1.0].mean()),
    }
