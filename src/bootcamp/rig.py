"""Model of the session-3 rig: a two-motor seesaw (one axis of a quadrotor on a bench).

::

        motor 1 (+)                         motor 2 (-)
          | T1          pivot (pot)           | T2
       ===O==================^==================O===
                          stand

* ``theta`` is the beam angle, positive when the motor-1 end is up (degrees in the
  public API, like the sketches).
* ``J theta'' = l (T1 - T2) - m g h sin(theta) - b theta'``, with ``h`` the distance of
  the centre of mass *below* the pivot (``h > 0``: pendulum-stable, ``h = 0``: neutral,
  like a drone in hover, ``h < 0``: unstable). Mechanical stops at +-``angle_limit``.
* Each motor: ESC pulse 1000-2000 us -> throttle 0-1 -> static thrust
  ``T_max (a1 thr + a2 thr^2)``, followed by a first-order lag ``tau_motor``.
* Commands are in ESC microseconds and angles in degrees, exactly as in
  ``arduino/s03_rig_angle_pid``, so gains found here transfer to the sketch.

**Every default below is an assumption** for the build in ``hardware/rig`` (A2212-class
1000 KV motors, 10x4.5 props, 12 V supply, 60 cm beam). Lab 3 measures the thrust
curve and the free-swing period of *your* rig; put your numbers into
:class:`RigParams` before trusting a simulated tuning.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field, replace

import numpy as np

G = 9.81


@dataclass(frozen=True)
class RigParams:
    arm: float = 0.25  # m, pivot to motor axis
    mass: float = 0.29  # kg, everything that rotates
    inertia: float = 0.0126  # kg m^2 about the pivot
    cg_below_pivot: float = 0.01  # m (h)
    damping: float = 0.010  # N m s/rad (bearings + air)
    thrust_max: float = 8.0  # N per motor at full throttle
    thrust_a1: float = 0.2  # linear share of the thrust curve
    thrust_a2: float = 0.8  # quadratic share (a1 + a2 = 1)
    tau_motor: float = 0.08  # s, thrust lag of motor + 10-inch prop
    angle_limit_deg: float = 30.0  # rubber stops
    us_min: int = 1000
    us_max: int = 2000

    def thrust(self, us) -> np.ndarray:
        """Static thrust (N) for an ESC pulse width (us)."""
        thr = np.clip(
            (np.asarray(us, dtype=float) - self.us_min) / (self.us_max - self.us_min), 0, 1
        )
        return self.thrust_max * (self.thrust_a1 * thr + self.thrust_a2 * thr**2)

    def thrust_slope(self, us: float) -> float:
        """dT/d(us) at a pulse width (N per microsecond)."""
        thr = (us - self.us_min) / (self.us_max - self.us_min)
        return (
            self.thrust_max
            * (self.thrust_a1 + 2 * self.thrust_a2 * thr)
            / (self.us_max - self.us_min)
        )

    @property
    def gravity_stiffness(self) -> float:
        """``m g h`` (N m/rad): restoring torque per radian for small angles."""
        return self.mass * G * self.cg_below_pivot


@dataclass(frozen=True)
class SensorModel:
    """Potentiometer at the pivot and an MPU-6050 on the beam."""

    pot_deg_per_count: float = 270.0 / 1023.0  # 270-degree pot on a 10-bit ADC
    pot_noise_counts: float = 0.5
    gyro_bias_dps: float = 1.5  # deg/s, constant per run (drawn +-)
    gyro_noise_dps: float = 0.3  # deg/s white noise per sample, motors off
    accel_noise_g: float = 0.01  # g per sample, motors off
    vib_gyro_dps: float = 4.0  # extra gyro noise at full throttle (vibration)
    vib_accel_g: float = 0.35  # extra accelerometer noise at full throttle
    imu_offset: float = 0.0  # m from the pivot along the beam (0 = on the axis)


@dataclass
class RigLog:
    t: np.ndarray
    theta: np.ndarray  # true angle (deg)
    rate: np.ndarray  # true rate (deg/s)
    pot: np.ndarray  # pot angle (deg, quantised)
    gyro: np.ndarray  # gyro rate (deg/s)
    accel_x: np.ndarray  # accelerometer along the beam (g)
    accel_z: np.ndarray  # accelerometer normal to the beam (g)
    us1: np.ndarray
    us2: np.ndarray
    setpoint: np.ndarray
    u: np.ndarray  # differential command (us)
    extra: dict = field(default_factory=dict)

    @property
    def accel_angle(self) -> np.ndarray:
        """Tilt from the accelerometer alone, ``atan2(a_x, a_z)`` (deg)."""
        return np.degrees(np.arctan2(self.accel_x, self.accel_z))


def mix(base_us: float, u_us: float, lo: float, hi: float) -> tuple[float, float]:
    """Reference mixer: ``base +- u``, clipped, keeping the differential if possible.

    When one side would exceed a limit the *base* is shifted (like Betaflight's
    air mode), so the commanded torque survives; only if ``2|u| > hi - lo`` is the
    differential itself reduced.
    """
    u = max(-(hi - lo) / 2, min((hi - lo) / 2, u_us))
    m1, m2 = base_us + u, base_us - u
    shift = 0.0
    top, bot = max(m1, m2), min(m1, m2)
    if top > hi:
        shift = hi - top
    elif bot < lo:
        shift = lo - bot
    return m1 + shift, m2 + shift


class Rig:
    """Rigid-beam simulation, RK4 at 1 kHz, stepped by the controller period."""

    def __init__(
        self,
        params: RigParams | None = None,
        sensors: SensorModel | None = None,
        seed: int = 0,
        theta0_deg: float = 0.0,
        eta: tuple[float, float] = (1.0, 1.0),
    ):
        self.p = params or RigParams()
        self.s = sensors or SensorModel()
        self.rng = np.random.default_rng(seed)
        self.eta = np.array(eta, dtype=float)  # loss-of-effectiveness factors per motor
        self.gyro_bias = self.rng.uniform(-1, 1) * self.s.gyro_bias_dps
        self.x = np.array([math.radians(theta0_deg), 0.0, 0.0, 0.0])  # theta, w, T1, T2
        self._alpha = 0.0  # last angular acceleration (for the accelerometer)

    def _deriv(self, x, Tc):
        p = self.p
        th, w, T1, T2 = x
        torque = p.arm * (self.eta[0] * T1 - self.eta[1] * T2)
        torque -= p.gravity_stiffness * math.sin(th) + p.damping * w
        a = torque / p.inertia
        return np.array([w, a, (Tc[0] - T1) / p.tau_motor, (Tc[1] - T2) / p.tau_motor])

    def settle_motors(self, us1: float, us2: float) -> None:
        """Start with the motors already spun up to these commands."""
        self.x[2:4] = self.p.thrust([us1, us2])

    def step(self, us1: float, us2: float, dt: float, sub: int = 5) -> None:
        Tc = self.p.thrust([us1, us2])
        h = dt / sub
        lim = math.radians(self.p.angle_limit_deg)
        for _ in range(sub):
            x = self.x
            k1 = self._deriv(x, Tc)
            k2 = self._deriv(x + 0.5 * h * k1, Tc)
            k3 = self._deriv(x + 0.5 * h * k2, Tc)
            k4 = self._deriv(x + h * k3, Tc)
            self.x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            self._alpha = k1[1]
            if abs(self.x[0]) > lim:  # inelastic stop
                self.x[0] = math.copysign(lim, self.x[0])
                if self.x[1] * self.x[0] > 0:
                    self.x[1] = 0.0

    def measure(self, throttle_avg: float) -> dict:
        s, rng = self.s, self.rng
        th, w = self.x[0], self.x[1]
        counts = round(
            math.degrees(th) / s.pot_deg_per_count + s.pot_noise_counts * rng.standard_normal()
        )
        vib = max(0.0, min(1.0, throttle_avg))
        g_sd = math.hypot(s.gyro_noise_dps, s.vib_gyro_dps * vib)
        a_sd = math.hypot(s.accel_noise_g, s.vib_accel_g * vib)
        gyro = math.degrees(w) + self.gyro_bias + g_sd * rng.standard_normal()
        # Specific force in beam axes (x along the beam towards motor 1, z normal to it,
        # "up" when level), in g. At rest: (sin theta, cos theta). An IMU mounted r metres
        # from the pivot also feels the centripetal (-r w^2) and tangential (r alpha) parts.
        r = s.imu_offset
        ax = math.sin(th) - r * w * w / G + a_sd * rng.standard_normal()
        az = math.cos(th) + r * self._alpha / G + a_sd * rng.standard_normal()
        return {
            "pot": counts * s.pot_deg_per_count,
            "gyro": gyro,
            "accel_x": ax,
            "accel_z": az,
        }

    @property
    def theta_deg(self) -> float:
        return math.degrees(self.x[0])

    @property
    def rate_dps(self) -> float:
        return math.degrees(self.x[1])


Controller = Callable[[float, float, dict, float], float]


def simulate_rig(
    controller: Controller,
    t_end: float,
    setpoint=0.0,
    dt: float = 0.005,
    base_us: float = 1350.0,
    us_lo: float = 1100.0,
    us_hi: float = 1600.0,
    params: RigParams | None = None,
    sensors: SensorModel | None = None,
    seed: int = 0,
    theta0_deg: float = 0.0,
    eta: tuple[float, float] = (1.0, 1.0),
    fault_time: float | None = None,
    mixer: Callable[[float, float, float, float], tuple[float, float]] = mix,
    disturbance: Callable[[float], float] | None = None,
) -> RigLog:
    """Run ``controller(t, setpoint_deg, measurements, dt) -> u_us`` on the rig.

    ``measurements`` holds ``pot`` (deg), ``gyro`` (deg/s), ``accel_x``/``accel_z``
    (g). The differential command ``u`` goes through ``mixer(base, u, lo, hi)``.
    ``eta`` scales each motor's thrust from ``fault_time`` on (a loss of effectiveness).
    ``disturbance(t)`` is an extra torque (N m), e.g. a tap on the beam.
    """
    rig = Rig(params, sensors, seed, theta0_deg)
    rig.settle_motors(base_us, base_us)
    sp_fn = setpoint if callable(setpoint) else (lambda t, v=float(setpoint): v)
    n = int(round(t_end / dt)) + 1
    cols = (
        "t",
        "theta",
        "rate",
        "pot",
        "gyro",
        "accel_x",
        "accel_z",
        "us1",
        "us2",
        "setpoint",
        "u",
    )
    log = {c: np.zeros(n) for c in cols}
    us1 = us2 = base_us
    span = rig.p.us_max - rig.p.us_min
    for k in range(n):
        t = k * dt
        if fault_time is not None and t >= fault_time:
            rig.eta = np.array(eta, dtype=float)
        thr = 0.5 * ((us1 + us2) - 2 * rig.p.us_min) / span
        m = rig.measure(thr)
        sp = sp_fn(t)
        u = float(controller(t, sp, m, dt))
        us1, us2 = mixer(base_us, u, us_lo, us_hi)
        row = (
            t,
            rig.theta_deg,
            rig.rate_dps,
            m["pot"],
            m["gyro"],
            m["accel_x"],
            m["accel_z"],
            us1,
            us2,
            sp,
            u,
        )
        for c, v in zip(cols, row):
            log[c][k] = v
        if disturbance is not None:
            tau_d = disturbance(t)
            if tau_d:
                rig.x[1] += tau_d / rig.p.inertia * dt
        rig.step(us1, us2, dt)
    return RigLog(**log)


def linear_model(params: RigParams | None = None, base_us: float = 1350.0):
    """Linearised rig about ``theta = 0`` and the base command.

    States ``[theta (rad), omega (rad/s), torque (N m)]``, input differential
    command ``u`` (us). The torque lags the command with the motor time constant.
    Returns ``(A, B)``.
    """
    p = params or RigParams()
    k_u = 2 * p.arm * p.thrust_slope(base_us)  # N m per us of differential
    A = np.array(
        [
            [0.0, 1.0, 0.0],
            [-p.gravity_stiffness / p.inertia, -p.damping / p.inertia, 1.0 / p.inertia],
            [0.0, 0.0, -1.0 / p.tau_motor],
        ]
    )
    B = np.array([0.0, 0.0, k_u / p.tau_motor])
    return A, B


def with_changes(params: RigParams | None = None, **changes) -> RigParams:
    return replace(params or RigParams(), **changes)
