"""Fault detection and fault-tolerant control helpers (session 6).

The small functions here are the reference versions of the session-6 exercises
(feasibility of hover after a loss of effectiveness, CUSUM, isolation by fault
signature, re-allocation). The full pipeline -- estimator, CUSUM bank, supervisor and
yaw-relaxed controller -- comes from the ``rotorfault`` package (Rami Kronbi's
rotor-fault simulation study), wrapped by :func:`run_case`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

ROTORFAULT_HELP = (
    "The rotorfault package is not installed. Install it from its repository, e.g.\n"
    "  pip install 'rotorfault @ git+https://github.com/Kronbii/rotor-fault-recovery'\n"
    "or, offline, from the wheel your instructor hands out:\n"
    "  pip install rotorfault-0.1.0-py3-none-any.whl\n"
    "(see docs/setup.md, 'rotorfault')."
)


def require_rotorfault():
    try:
        import rotorfault  # noqa: F401
    except ImportError as e:  # pragma: no cover - depends on the environment
        raise ImportError(ROTORFAULT_HELP) from e
    return rotorfault


# ----------------------------------------------------------------- feasibility
def eta_min_yaw_hover(thrust_to_weight: float) -> float:
    """Smallest effectiveness of one rotor that still allows hover *with yaw held*.

    With zero net torque the X-quad allocation is unique: every rotor must produce a
    quarter of the weight, so ``eta f_max >= W / 4`` and ``eta >= 1 / (T/W)``.
    """
    return 1.0 / thrust_to_weight


# ----------------------------------------------------------------- detection
def cusum(r, mu1: float, sigma: float, h: float):
    """One-sided CUSUM (Page 1954) for a mean shift ``0 -> mu1`` in N(0, sigma^2) noise.

    ``g_k = max(0, g_{k-1} + (mu1 / sigma^2) (r_k - mu1 / 2))``; alarm at the first
    ``g_k > h``. Returns ``(alarm_index or -1, g)``.
    """
    r = np.asarray(r, dtype=float)
    g = np.zeros(len(r))
    acc = 0.0
    k_alarm = -1
    for k, rk in enumerate(r):
        acc = max(0.0, acc + (mu1 / sigma**2) * (rk - 0.5 * mu1))
        g[k] = acc
        if k_alarm < 0 and acc > h:
            k_alarm = k
    return k_alarm, g


def isolate(r, D) -> int:
    """Index of the fault signature (column of ``D``) best aligned with residual ``r``.

    Uses the normalised projection ``t_j = d_j^T r / |d_j|`` (the statistic the rotorfault
    FDI feeds to its CUSUM bank).
    """
    D = np.asarray(D, dtype=float)
    t = D.T @ np.asarray(r, dtype=float) / np.linalg.norm(D, axis=0)
    return int(np.argmax(t))


# ----------------------------------------------------------------- reconfiguration
def allocate_with_eta(
    B, eta, T: float, tau, f_min: float | None = None, f_max: float | None = None
):
    """Rotor commands that produce ``[T, tau]`` on a vehicle with effectiveness ``eta``.

    The vehicle produces ``B diag(eta) f``, so ``f = (B diag(eta))^-1 [T, tau]``;
    optional limits clip the result.
    """
    Be = np.asarray(B, dtype=float) * np.asarray(eta, dtype=float)[None, :]
    f = np.linalg.solve(Be, np.concatenate([[T], np.asarray(tau, dtype=float)]))
    if f_min is not None or f_max is not None:
        f = np.clip(f, -np.inf if f_min is None else f_min, np.inf if f_max is None else f_max)
    return f


# ----------------------------------------------------------------- rotorfault wrapper
@dataclass
class CaseSummary:
    baseline: str
    lost: bool
    lost_reason: str
    detection_delay_ms: float
    isolated_rotor: int  # 1-based, 0 if none
    max_deviation_cm: float
    switch_to_relaxed_ms: float
    final_spin_rad_s: float


def run_case(
    loe: float,
    rotor: int = 1,
    baseline: str = "estimated",
    traj: str = "hover",
    duration: float = 5.0,
    onset: float = 2.0,
    seed: int = 0,
):
    """One closed-loop rotorfault simulation; ``rotor`` is 1-based (1 = front right).

    Returns ``(SimResult, CaseSummary)``. Uses rotorfault's default (calibrated) FDI
    configuration.
    """
    require_rotorfault()
    from rotorfault import metrics as M
    from rotorfault.fdi import FDIConfig
    from rotorfault.scenarios import make_scenario
    from rotorfault.sim import simulate

    sc = make_scenario(traj, duration, rotor=rotor - 1, magnitude=loe, onset=onset)
    r = simulate(sc, baseline, seed, FDIConfig())
    fault = loe > 0
    delay = (r.alarm_time - onset) * 1000 if fault and math.isfinite(r.alarm_time) else math.nan
    sw = (r.switch_relaxed - onset) * 1000 if math.isfinite(r.switch_relaxed) else math.nan
    summary = CaseSummary(
        baseline=baseline,
        lost=bool(r.crashed),
        lost_reason=r.crash_reason,
        detection_delay_ms=delay,
        isolated_rotor=r.isolated + 1 if r.isolated >= 0 else 0,
        max_deviation_cm=100 * M.max_deviation(r, onset if fault else 0.0),
        switch_to_relaxed_ms=sw,
        final_spin_rad_s=M.spin_rate(r) if not r.crashed else math.nan,
    )
    return r, summary
