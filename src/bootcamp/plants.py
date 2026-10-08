"""Plants used in sessions 1-2: first/second-order systems, dead time and RC ladders.

Every linear plant is a state-space model ``x' = A x + B u, y = C x + D u`` that is
stepped with the *exact* zero-order-hold discretisation for the chosen time step, so
simulations do not depend on an integration method (students write their own Euler
step in session 1 and compare it against this).

The RC ladder is the physical plant of labs 1-2: an Arduino PWM pin drives ``n``
identical R-C stages and ``analogRead`` measures the voltage on the last capacitor.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy.linalg import expm


class LinearPlant:
    """Continuous-time LTI plant stepped with an exact ZOH discretisation."""

    def __init__(self, A, B, C, D=None, x0=None, name: str = ""):
        self.A = np.atleast_2d(np.asarray(A, dtype=float))
        n = self.A.shape[0]
        self.B = np.asarray(B, dtype=float).reshape(n)
        self.C = np.asarray(C, dtype=float).reshape(n)
        self.D = 0.0 if D is None else float(D)
        self.name = name
        self._x0 = np.zeros(n) if x0 is None else np.asarray(x0, dtype=float).copy()
        self.x = self._x0.copy()
        self._cache: dict[float, tuple[np.ndarray, np.ndarray]] = {}

    @property
    def order(self) -> int:
        return self.A.shape[0]

    def reset(self, x0=None) -> None:
        self.x = (self._x0 if x0 is None else np.asarray(x0, dtype=float)).copy()

    def discretise(self, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """``(Ad, Bd)`` with ``x[k+1] = Ad x[k] + Bd u[k]`` for a held input."""
        key = round(float(dt), 12)
        if key not in self._cache:
            n = self.order
            M = np.zeros((n + 1, n + 1))
            M[:n, :n] = self.A
            M[:n, n] = self.B
            E = expm(M * dt)
            self._cache[key] = (E[:n, :n], E[:n, n])
        return self._cache[key]

    def output(self, u: float = 0.0) -> float:
        return float(self.C @ self.x) + self.D * u

    def step(self, u: float, dt: float) -> float:
        """Apply ``u`` for ``dt`` seconds; returns the output at the end of the step."""
        Ad, Bd = self.discretise(dt)
        self.x = Ad @ self.x + Bd * float(u)
        return self.output()

    def dc_gain(self) -> float:
        return float(-self.C @ np.linalg.solve(self.A, self.B)) + self.D

    def poles(self) -> np.ndarray:
        return np.linalg.eigvals(self.A)


def first_order(K: float, tau: float, y0: float = 0.0) -> LinearPlant:
    """``tau y' + y = K u``."""
    return LinearPlant([[-1.0 / tau]], [K / tau], [1.0], x0=[y0], name=f"K={K}, tau={tau}")


def second_order(K: float, wn: float, zeta: float) -> LinearPlant:
    """``y'' + 2 zeta wn y' + wn^2 y = K wn^2 u`` (states: y, y')."""
    A = [[0.0, 1.0], [-(wn**2), -2.0 * zeta * wn]]
    return LinearPlant(A, [0.0, K * wn**2], [1.0, 0.0], name=f"wn={wn}, zeta={zeta}")


def double_integrator(gain: float = 1.0, damping: float = 0.0) -> LinearPlant:
    """``y'' = gain u - damping y'`` -- a drone axis, to first approximation."""
    return LinearPlant([[0.0, 1.0], [0.0, -damping]], [0.0, gain], [1.0, 0.0])


def rc_ladder(R: float, C: float, stages: int = 1) -> LinearPlant:
    """``stages`` identical R-C sections in cascade, input volts -> last-node volts.

    Node ``k`` (capacitor voltage ``v_k``) obeys
    ``C dv_k/dt = (v_{k-1} - v_k)/R - (v_k - v_{k+1})/R`` with ``v_0 = u`` and no
    current leaving the last node (the ADC input is high impedance). Every stage
    loads the one before it, so the poles are *not* simply ``-1/RC`` repeated.
    """
    n = int(stages)
    if n < 1:
        raise ValueError("stages must be >= 1")
    a = 1.0 / (R * C)
    A = np.zeros((n, n))
    for k in range(n):
        A[k, k] = -2.0 * a if k < n - 1 else -a
        if k > 0:
            A[k, k - 1] = a
        if k < n - 1:
            A[k, k + 1] = a
    B = np.zeros(n)
    B[0] = a
    Cm = np.zeros(n)
    Cm[-1] = 1.0
    return LinearPlant(A, B, Cm, name=f"RC ladder x{n} (R={R:g}, C={C:g})")


class DeadTime:
    """Pure delay of ``n`` samples (a FIFO), e.g. sensor latency or ESC frames."""

    def __init__(self, n: int, initial: float = 0.0):
        self.n = int(n)
        self._buf: deque[float] = deque([float(initial)] * self.n, maxlen=self.n or None)

    def __call__(self, u: float) -> float:
        if self.n == 0:
            return float(u)
        out = self._buf[0]
        self._buf.append(float(u))
        return out


@dataclass
class ArduinoIO:
    """Unit conversions of an Uno/Nano: 8-bit PWM out, 10-bit ADC in, 5 V reference."""

    vref: float = 5.0
    pwm_max: int = 255
    adc_max: int = 1023

    def pwm_to_volts(self, pwm: float) -> float:
        return float(np.clip(pwm, 0, self.pwm_max)) / self.pwm_max * self.vref

    def volts_to_adc(self, v: float) -> int:
        return int(np.clip(np.round(v / self.vref * self.adc_max), 0, self.adc_max))

    @property
    def counts_per_pwm(self) -> float:
        """Static gain of a resistive path from ``analogWrite`` to ``analogRead``."""
        return self.adc_max / self.pwm_max


ARDUINO = ArduinoIO()
