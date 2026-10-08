"""Step-response metrics and simple system identification.

Definitions (used in every notebook, lab sheet and the capstone rubric):

* **rise time**: time to go from 10 % to 90 % of the change ``y0 -> r``;
* **overshoot**: ``100 (peak - r) / (r - y0)`` in percent, 0 if the response never
  passes ``r`` (for a downward step the signs flip accordingly);
* **settling time**: first time after which ``|y - r|`` stays within ``band`` times the
  step size (2 % by default); ``nan`` if it has not settled by the end of the record;
* **steady-state error**: ``r`` minus the mean of ``y`` over the last 10 % of samples;
* **IAE**: integral of ``|r - y|`` (trapezoidal).
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np


@dataclass
class StepInfo:
    rise_time: float
    overshoot_pct: float
    settling_time: float
    steady_state_error: float
    peak: float
    peak_time: float
    iae: float

    def as_dict(self) -> dict:
        return asdict(self)

    def __str__(self) -> str:
        return (
            f"rise {self.rise_time:.3g} s | overshoot {self.overshoot_pct:.1f} % | "
            f"settling {self.settling_time:.3g} s | ss error {self.steady_state_error:.3g} | "
            f"IAE {self.iae:.3g}"
        )


def _first_crossing(t: np.ndarray, s: np.ndarray, level: float) -> float:
    """Linearly interpolated first time ``s`` reaches ``level`` (s increasing)."""
    idx = np.nonzero(s >= level)[0]
    if len(idx) == 0:
        return math.nan
    k = int(idx[0])
    if k == 0:
        return float(t[0])
    s0, s1 = s[k - 1], s[k]
    frac = 0.0 if s1 == s0 else (level - s0) / (s1 - s0)
    return float(t[k - 1] + frac * (t[k] - t[k - 1]))


def step_info(t, y, r: float, y0: float | None = None, band: float = 0.02) -> StepInfo:
    """Metrics of a step response ``y(t)`` towards the setpoint ``r`` (see module doc)."""
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    y0 = float(y[0]) if y0 is None else float(y0)
    step = r - y0
    if step == 0:
        raise ValueError("setpoint equals the initial value: no step to analyse")
    # normalised response: 0 at the start, 1 at the setpoint, whatever the direction
    s = (y - y0) / step
    t10 = _first_crossing(t, s, 0.1)
    t90 = _first_crossing(t, s, 0.9)
    rise = t90 - t10 if math.isfinite(t10) and math.isfinite(t90) else math.nan
    k_peak = int(np.argmax(s))
    overshoot = max(0.0, float(s[k_peak]) - 1.0) * 100.0
    outside = np.nonzero(np.abs(s - 1.0) > band)[0]
    if len(outside) == 0:
        settling = 0.0
    elif outside[-1] == len(s) - 1:
        settling = math.nan
    else:
        settling = float(t[outside[-1] + 1] - t[0])
    n_tail = max(1, len(y) // 10)
    sse = float(r - y[-n_tail:].mean())
    iae = integrate(np.abs(r - y), t)
    return StepInfo(
        rise_time=rise,
        overshoot_pct=overshoot,
        settling_time=settling,
        steady_state_error=sse,
        peak=float(y[k_peak]),
        peak_time=float(t[k_peak] - t[0]),
        iae=iae,
    )


@dataclass
class FOPDT:
    """First order plus dead time: ``K e^{-L s} / (tau s + 1)``."""

    K: float
    tau: float
    L: float


def fit_fopdt_two_point(t, y, du: float, y0: float | None = None) -> FOPDT:
    """Identify a FOPDT model from an open-loop step of size ``du`` applied at ``t[0]``.

    Two-point method (Smith): with ``t28`` and ``t63`` the times at which the response
    reaches 28.3 % and 63.2 % of its final change, ``tau = 1.5 (t63 - t28)`` and
    ``L = t63 - tau``. The final value is the mean of the last 10 % of samples.
    """
    t = np.asarray(t, dtype=float) - float(t[0])
    y = np.asarray(y, dtype=float)
    y0 = float(y[0]) if y0 is None else float(y0)
    n_tail = max(1, len(y) // 10)
    dy = float(y[-n_tail:].mean()) - y0
    s = (y - y0) / dy
    t28 = _first_crossing(t, s, 0.283)
    t63 = _first_crossing(t, s, 0.632)
    tau = 1.5 * (t63 - t28)
    return FOPDT(K=dy / du, tau=tau, L=max(0.0, t63 - tau))


def ziegler_nichols_open_loop(model: FOPDT) -> tuple[float, float, float]:
    """Reaction-curve PID rule as written in easyPID's ``docs/tuning_guide.md``:
    ``Kp = 1.2 T/(K L)``, ``Ki = Kp/(2 L)``, ``Kd = 0.5 Kp L``."""
    if model.L <= 0:
        raise ValueError("the open-loop Ziegler-Nichols rule needs a dead time L > 0")
    kp = 1.2 * model.tau / (model.K * model.L)
    return kp, kp / (2.0 * model.L), 0.5 * kp * model.L


def integrate(f, t) -> float:
    """Trapezoidal integral of samples ``f`` over times ``t``."""
    f = np.asarray(f, dtype=float)
    t = np.asarray(t, dtype=float)
    return float(np.sum(0.5 * (f[1:] + f[:-1]) * np.diff(t)))


def rms(x) -> float:
    x = np.asarray(x, dtype=float)
    return float(np.sqrt(np.mean(x * x)))
