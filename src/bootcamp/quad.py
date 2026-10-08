"""A minimal quadrotor simulator and cascaded PID controller (session 5).

Small on purpose -- about 300 lines a student can read in one sitting -- but with the
physics that matter for control:

* 6-DOF rigid body: ``m dv/dt = R (0, 0, T) - R D R^T v - m g e3`` and
  ``J dw/dt = tau - w x J w - K_w w``, quaternion attitude, RK4 integration;
* four rotors with thrust ``f_i = eta_i k_f Omega_i^2``, yaw reaction torque
  ``s_i kappa f_i`` and a first-order lag on rotor speed;
* an X-configuration allocation matrix ``B``: ``[T, tau_x, tau_y, tau_z] = B f``.

Left out (and present in the rotorfault study model): rotor gyroscopic torque and the
reaction torque of accelerating rotors, quadratic drag, sensor noise and latency,
turbulence, parameter uncertainty. The vehicle parameters are the nominal 5-inch quad
of the rotorfault study, and the test-suite checks that both models agree in hover.

Conventions: world z up; body FLU; rotors numbered from above
1 front-right, 2 front-left, 3 rear-left, 4 rear-right; rotors 1 and 3 spin so that
their reaction torque on the body is +z (``s = (+1, -1, +1, -1)``).

The controller is the cascade used by most open-source flight stacks::

    position --P--> velocity --PID--> acceleration --> tilt + thrust
             attitude --P--> body rate --PID--> angular acceleration --> torque --> B^-1

and every PID in it is an easyPID ``PIDController`` (the Python port), i.e. the same
controller that runs on the Arduino in the rig labs.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from .attitude import R_to_euler_zyx, quat_to_R
from .easypid import ANTIWINDUP_CLAMP, FILTER_EMA, PIDController

G = 9.81


@dataclass
class QuadParams:
    mass: float = 0.638  # kg
    inertia: tuple = (1.3636e-3, 2.1226e-3, 3.1269e-3)  # kg m^2 (diagonal)
    arm: float = 0.11  # m, motor to centre
    kappa: float = 0.015  # m, yaw torque per newton of thrust
    k_f: float = 8.96e-7  # N s^2, f = k_f Omega^2
    omega_min: float = 250.0  # rad/s (idle)
    omega_max: float = 2500.0  # rad/s
    tau_motor: float = 0.025  # s
    drag: tuple = (0.16, 0.16, 0.20)  # N s/m, body axes
    rot_damping: tuple = (2.5e-3, 2.5e-3, 2.0e-3)  # N m s/rad
    g: float = G

    @property
    def J(self) -> np.ndarray:
        return np.diag(self.inertia)

    @property
    def f_max(self) -> float:
        return self.k_f * self.omega_max**2

    @property
    def f_min(self) -> float:
        return self.k_f * self.omega_min**2

    @property
    def weight(self) -> float:
        return self.mass * self.g

    @property
    def hover_thrust(self) -> float:
        return self.weight / 4

    @property
    def thrust_to_weight(self) -> float:
        return 4 * self.f_max / self.weight

    @property
    def spin_dir(self) -> np.ndarray:
        return np.array([1.0, -1.0, 1.0, -1.0])

    @property
    def rotor_positions(self) -> np.ndarray:
        th = np.radians([-45.0, 45.0, 135.0, 225.0])
        return np.stack([self.arm * np.cos(th), self.arm * np.sin(th), np.zeros(4)], axis=1)

    def mixer(self) -> np.ndarray:
        """``B`` (4x4) with ``[T, tau_x, tau_y, tau_z] = B f``."""
        r = self.rotor_positions
        return np.vstack([np.ones(4), r[:, 1], -r[:, 0], self.kappa * self.spin_dir])


# ====================================================================== dynamics
class Quadrotor:
    """State ``x`` (17): position, velocity, quaternion (w,x,y,z), body rate, rotor speeds."""

    def __init__(self, params: QuadParams | None = None, position=(0.0, 0.0, 1.0)):
        self.P = params or QuadParams()
        self.eta = np.ones(4)  # rotor effectiveness (1 healthy, 0 failed)
        self.x = np.zeros(17)
        self.x[0:3] = position
        self.x[6] = 1.0
        self.x[13:17] = math.sqrt(self.P.hover_thrust / self.P.k_f)
        P = self.P
        self._r = P.rotor_positions
        self._J = np.array(P.inertia)
        self._D = np.array(P.drag)
        self._Kw = np.array(P.rot_damping)

    # -- convenience views
    @property
    def p(self):
        return self.x[0:3]

    @property
    def v(self):
        return self.x[3:6]

    @property
    def q(self):
        return self.x[6:10]

    @property
    def w(self):
        return self.x[10:13]

    @property
    def R(self):
        return quat_to_R(self.x[6:10])

    def rotor_thrusts(self, x=None) -> np.ndarray:
        x = self.x if x is None else x
        return self.eta * self.P.k_f * x[13:17] ** 2

    def deriv(self, x: np.ndarray, omega_cmd: np.ndarray) -> np.ndarray:
        P = self.P
        f = self.eta * P.k_f * x[13:17] ** 2
        T = f.sum()
        tau = np.array([self._r[:, 1] @ f, -(self._r[:, 0] @ f), P.kappa * (P.spin_dir @ f)])
        q = x[6:10]
        R = quat_to_R(q)
        v = x[3:6]
        w = x[10:13]
        drag_body = -self._D * (R.T @ v)
        acc = (R @ (np.array([0.0, 0.0, T]) + drag_body)) / P.mass
        acc[2] -= P.g
        qw, qx, qy, qz = q
        dq = 0.5 * np.array(
            [
                -qx * w[0] - qy * w[1] - qz * w[2],
                qw * w[0] + qy * w[2] - qz * w[1],
                qw * w[1] + qz * w[0] - qx * w[2],
                qw * w[2] + qx * w[1] - qy * w[0],
            ]
        )
        Jw = self._J * w
        dw = (tau - np.cross(w, Jw) - self._Kw * w) / self._J
        dOmega = (omega_cmd - x[13:17]) / P.tau_motor
        return np.concatenate([v, acc, dq, dw, dOmega])

    def thrust_to_speed(self, f_cmd) -> np.ndarray:
        """ESC map: commanded thrust (N) -> rotor-speed command, within limits."""
        P = self.P
        f = np.clip(np.asarray(f_cmd, dtype=float), P.f_min, P.f_max)
        return np.sqrt(f / P.k_f)

    def step(self, f_cmd, dt: float) -> None:
        """Advance ``dt`` seconds with the rotor thrust commands held (RK4)."""
        wc = self.thrust_to_speed(f_cmd)
        x = self.x
        k1 = self.deriv(x, wc)
        k2 = self.deriv(x + 0.5 * dt * k1, wc)
        k3 = self.deriv(x + 0.5 * dt * k2, wc)
        k4 = self.deriv(x + dt * k3, wc)
        x = x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        x[6:10] /= np.linalg.norm(x[6:10])
        self.x = x


# ====================================================================== allocation
def allocate(B: np.ndarray, T: float, tau, f_min: float, f_max: float) -> np.ndarray:
    """Rotor thrusts for ``[T, tau]`` with saturation priorities.

    1. roll/pitch torque first: scaled down only if it cannot fit at all;
    2. collective thrust next: shifted by the smallest amount that fits;
    3. yaw torque last: the largest fraction ``alpha in [0, 1]`` that still fits.
    """
    Binv = np.linalg.inv(B)
    base = Binv[:, 0] * T
    h = Binv[:, 1] * tau[0] + Binv[:, 2] * tau[1]
    d = Binv[:, 3] * tau[2]
    span = f_max - f_min
    spread = h.max() - h.min()
    if spread > span:
        h = h * (span / spread)
    f = base + h
    if f.max() > f_max:
        f = f - (f.max() - f_max)
    if f.min() < f_min:
        f = f + (f_min - f.min())
    alpha = 1.0
    for fi, di in zip(f, d):
        if di > 1e-12:
            alpha = min(alpha, (f_max - fi) / di)
        elif di < -1e-12:
            alpha = min(alpha, (f_min - fi) / di)
    alpha = max(alpha, 0.0)
    return np.clip(f + alpha * d, f_min, f_max)


def accel_to_tilt(ax: float, ay: float, yaw: float, g: float = G) -> tuple[float, float]:
    """Small-angle map from a desired horizontal acceleration to (roll, pitch) in rad.

    For ``R = Rz(yaw) Ry(pitch) Rx(roll)`` the thrust direction is approximately
    ``(pitch cos(yaw) + roll sin(yaw), pitch sin(yaw) - roll cos(yaw), 1)``, so
    ``pitch = (ax cos(yaw) + ay sin(yaw)) / g`` and
    ``roll = (ax sin(yaw) - ay cos(yaw)) / g``.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    return (ax * s - ay * c) / g, (ax * c + ay * s) / g


# ====================================================================== control
@dataclass
class Reference:
    p: np.ndarray
    v: np.ndarray = field(default_factory=lambda: np.zeros(3))
    yaw: float = 0.0


@dataclass
class CascadeGains:
    pos_kp: tuple = (1.5, 1.5, 2.0)  # 1/s -> velocity setpoint
    v_max: float = 3.0  # m/s
    vel_kp: tuple = (3.0, 3.0, 4.0)  # (m/s^2)/(m/s)
    vel_ki: tuple = (1.0, 1.0, 2.0)
    vel_kd: tuple = (0.0, 0.0, 0.0)
    acc_max: tuple = (6.0, 6.0, 8.0)  # m/s^2
    att_kp: tuple = (8.0, 8.0, 4.0)  # 1/s -> body-rate setpoint
    rate_max: float = 6.0  # rad/s
    rate_kp: tuple = (25.0, 25.0, 10.0)  # (rad/s^2)/(rad/s)
    rate_ki: tuple = (10.0, 10.0, 5.0)
    rate_kd: tuple = (0.25, 0.25, 0.0)
    rate_d_alpha: float = 0.6  # easyPID EMA derivative filter
    ang_acc_max: tuple = (400.0, 400.0, 60.0)  # rad/s^2
    max_tilt_deg: float = 35.0


def _pid(kp, ki, kd, lim, alpha=None) -> PIDController:
    pid = PIDController(kp, ki, kd, -lim, lim)
    pid.setAntiWindup(ANTIWINDUP_CLAMP)
    if alpha is not None and kd > 0:
        pid.setDerivativeFilter(FILTER_EMA, alpha)
    pid.begin()
    return pid


class CascadedController:
    """Position -> velocity -> attitude -> rate cascade built from easyPID controllers."""

    def __init__(
        self,
        params: QuadParams | None = None,
        gains: CascadeGains | None = None,
        tilt_map: Callable = accel_to_tilt,
        allocator: Callable = allocate,
    ):
        self.P = params or QuadParams()
        self.g = gains or CascadeGains()
        g = self.g
        self.vel = [_pid(g.vel_kp[i], g.vel_ki[i], g.vel_kd[i], g.acc_max[i]) for i in range(3)]
        self.rate = [
            _pid(g.rate_kp[i], g.rate_ki[i], g.rate_kd[i], g.ang_acc_max[i], g.rate_d_alpha)
            for i in range(3)
        ]
        self.tilt_map = tilt_map
        self.allocator = allocator
        self.B = self.P.mixer()
        self.last = {}

    def __call__(self, quad: Quadrotor, ref: Reference, dt: float) -> np.ndarray:
        P, g = self.P, self.g
        dt_ms = dt * 1000.0
        p, v, w = quad.p, quad.v, quad.w
        roll, pitch, yaw = R_to_euler_zyx(quad.R)

        # outer loops: position (P) -> velocity (PID) -> acceleration command
        v_sp = np.clip(np.array(g.pos_kp) * (ref.p - p), -g.v_max, g.v_max) + ref.v
        a = np.array([self.vel[i].update(v_sp[i], v[i], dt_ms) for i in range(3)])

        # acceleration -> attitude and thrust
        roll_d, pitch_d = self.tilt_map(a[0], a[1], yaw, P.g)
        tmax = math.radians(g.max_tilt_deg)
        roll_d, pitch_d = float(np.clip(roll_d, -tmax, tmax)), float(np.clip(pitch_d, -tmax, tmax))
        tilt_comp = max(math.cos(roll) * math.cos(pitch), 0.5)
        T = P.mass * (P.g + a[2]) / tilt_comp

        # attitude (P) -> body-rate setpoint
        e_yaw = math.atan2(math.sin(ref.yaw - yaw), math.cos(ref.yaw - yaw))
        e_att = np.array([roll_d - roll, pitch_d - pitch, e_yaw])
        rate_sp = np.clip(np.array(g.att_kp) * e_att, -g.rate_max, g.rate_max)

        # rate (PID) -> angular acceleration -> torque
        alpha = np.array([self.rate[i].update(rate_sp[i], w[i], dt_ms) for i in range(3)])
        J = np.array(P.inertia)
        tau = J * alpha + np.cross(w, J * w)

        f = self.allocator(self.B, T, tau, P.f_min, P.f_max)
        self.last = dict(rate_sp=rate_sp, att_sp=np.array([roll_d, pitch_d, ref.yaw]), T=T, tau=tau)
        return f


# ====================================================================== simulation
@dataclass
class QuadLog:
    t: np.ndarray
    p: np.ndarray
    v: np.ndarray
    euler: np.ndarray  # roll, pitch, yaw (rad)
    w: np.ndarray
    f_cmd: np.ndarray
    thrust: np.ndarray
    p_ref: np.ndarray
    rate_sp: np.ndarray
    crashed: bool
    crash_time: float


def hover_at(point) -> Callable[[float], Reference]:
    pt = np.asarray(point, dtype=float)
    return lambda t: Reference(p=pt.copy())


def waypoints(points, dwell: float) -> Callable[[float], Reference]:
    """Step through ``points``, ``dwell`` seconds each (then hold the last)."""
    pts = [np.asarray(p, dtype=float) for p in points]

    def ref(t: float) -> Reference:
        k = min(int(t // dwell), len(pts) - 1)
        return Reference(p=pts[k].copy())

    return ref


def simulate_quad(
    controller: CascadedController | Callable,
    reference: Callable[[float], Reference],
    t_end: float,
    dt: float = 0.002,
    substeps: int = 2,
    params: QuadParams | None = None,
    start=(0.0, 0.0, 1.0),
    eta_schedule: Callable[[float], np.ndarray] | None = None,
    motor_scale: float = 1.0,
    gyro_noise: float = 0.0,
    seed: int = 0,
) -> QuadLog:
    """Closed-loop run; the controller sees the true state (plus optional gyro noise).

    ``motor_scale`` multiplies the *true* motor time constant (robustness studies);
    ``eta_schedule(t)`` returns the rotor effectiveness vector (faults).
    """
    P = params or QuadParams()
    true_P = (
        P
        if motor_scale == 1.0
        else QuadParams(**{**P.__dict__, "tau_motor": P.tau_motor * motor_scale})
    )
    quad = Quadrotor(true_P, start)
    rng = np.random.default_rng(seed)
    n = int(round(t_end / dt))
    log = {k: [] for k in ("t", "p", "v", "euler", "w", "f_cmd", "thrust", "p_ref", "rate_sp")}
    crashed, t_crash = False, math.nan
    for k in range(n):
        t = k * dt
        if eta_schedule is not None:
            quad.eta = np.asarray(eta_schedule(t), dtype=float)
        ref = reference(t)
        if gyro_noise > 0:
            x_true = quad.x.copy()
            quad.x[10:13] = x_true[10:13] + gyro_noise * rng.standard_normal(3)
            f = controller(quad, ref, dt)
            quad.x = x_true
        else:
            f = controller(quad, ref, dt)
        log["t"].append(t)
        log["p"].append(quad.p.copy())
        log["v"].append(quad.v.copy())
        log["euler"].append(np.array(R_to_euler_zyx(quad.R)))
        log["w"].append(quad.w.copy())
        log["f_cmd"].append(np.asarray(f, dtype=float).copy())
        log["thrust"].append(quad.rotor_thrusts())
        log["p_ref"].append(ref.p.copy())
        last = getattr(controller, "last", {})
        log["rate_sp"].append(np.asarray(last.get("rate_sp", np.full(3, np.nan))))
        for _ in range(substeps):
            quad.step(f, dt / substeps)
        tilt = math.acos(max(-1.0, min(1.0, quad.R[2, 2])))
        if quad.p[2] <= 0.0 or tilt > math.pi / 2 or not np.all(np.isfinite(quad.x)):
            crashed, t_crash = True, t + dt
            break
    arr = {k: np.array(v) for k, v in log.items()}
    return QuadLog(**arr, crashed=crashed, crash_time=t_crash)
