"""Closed-loop simulation of a plant with an easyPID controller (or any callable).

The loop runs exactly like a sketch does: at every sample the controller reads the
(possibly noisy, quantised, delayed) measurement, computes an output, the output is
held for one sample period, and the plant evolves.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from .easypid import PIDController
from .metrics import StepInfo, step_info
from .plants import DeadTime, LinearPlant


@dataclass
class LoopResult:
    t: np.ndarray
    r: np.ndarray
    y: np.ndarray  # true plant output at the sample instants
    y_meas: np.ndarray  # what the controller saw
    u: np.ndarray  # controller output (before the actuator delay)
    p: np.ndarray
    i: np.ndarray
    d: np.ndarray

    def step_info(self, t0: float = 0.0, r: float | None = None, band: float = 0.02) -> StepInfo:
        """Metrics of the response from ``t0`` (a setpoint change) to the end."""
        m = self.t >= t0 - 1e-12
        target = float(self.r[m][-1]) if r is None else r
        return step_info(self.t[m], self.y[m], target, band=band)


def _as_fn(x) -> Callable[[float], float]:
    if callable(x):
        return x
    v = float(x)
    return lambda t: v


def simulate_pid(
    plant: LinearPlant,
    controller: PIDController | Callable[[float, float, float], float],
    setpoint,
    t_end: float,
    dt: float,
    *,
    noise_std: float = 0.0,
    measure: Callable[[float], float] | None = None,
    disturbance=0.0,
    delay_samples: int = 0,
    u_limits: tuple[float, float] | None = None,
    seed: int = 0,
    reset: bool = True,
) -> LoopResult:
    """Simulate ``t_end`` seconds with sample period ``dt`` (seconds).

    ``controller`` is an easyPID :class:`PIDController` (called as
    ``update(r, y, dt*1000)``) or a function ``f(r, y, dt) -> u``.
    ``setpoint`` and ``disturbance`` (added to the plant input) may be constants or
    functions of time. ``measure`` maps the noisy plant output to what the controller
    sees (e.g. ADC quantisation). ``delay_samples`` delays the actuator.
    """
    rng = np.random.default_rng(seed)
    r_fn, d_fn = _as_fn(setpoint), _as_fn(disturbance)
    if reset:
        plant.reset()
        if isinstance(controller, PIDController):
            controller.begin()
    delay = DeadTime(delay_samples)
    n = int(round(t_end / dt)) + 1
    out = {k: np.zeros(n) for k in ("t", "r", "y", "y_meas", "u", "p", "i", "d")}
    y = plant.output()
    for k in range(n):
        t = k * dt
        r = r_fn(t)
        ym = y + (noise_std * rng.standard_normal() if noise_std > 0 else 0.0)
        if measure is not None:
            ym = measure(ym)
        if isinstance(controller, PIDController):
            u = controller.update(r, ym, dt * 1000.0)
            p, i, d = controller.getPterm(), controller.getIterm(), controller.getDterm()
        else:
            u = float(controller(r, ym, dt))
            p = i = d = np.nan
        for key, val in (("t", t), ("r", r), ("y", y), ("y_meas", ym), ("u", u)):
            out[key][k] = val
        out["p"][k], out["i"][k], out["d"][k] = p, i, d
        ua = delay(u)
        if u_limits is not None:
            ua = min(max(ua, u_limits[0]), u_limits[1])
        y = plant.step(ua + d_fn(t), dt)
    return LoopResult(**out)


def simulate_open_loop(plant: LinearPlant, u, t_end: float, dt: float) -> tuple:
    """Open-loop response to the input ``u`` (constant or function of time)."""
    u_fn = _as_fn(u)
    plant.reset()
    n = int(round(t_end / dt)) + 1
    t = np.arange(n) * dt
    y = np.zeros(n)
    uu = np.zeros(n)
    y[0] = plant.output()
    for k in range(n - 1):
        uu[k] = u_fn(t[k])
        y[k + 1] = plant.step(uu[k], dt)
    uu[-1] = u_fn(t[-1])
    return t, y, uu
