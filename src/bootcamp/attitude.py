"""Rotations, quaternions and attitude filters (session 4).

Conventions (the same as the session-5 simulator and the rotorfault study):

* world frame z up; body frame x forward, y left, z up (FLU);
* ``R`` maps body coordinates to world coordinates: ``v_world = R v_body``;
* Euler angles are Z-Y-X (yaw ``psi``, then pitch ``theta``, then roll ``phi``):
  ``R = Rz(psi) Ry(theta) Rx(phi)``;
* quaternions are scalar-first unit quaternions ``q = (w, x, y, z)``.
"""

from __future__ import annotations

import math

import numpy as np

G = 9.81


# ---------------------------------------------------------------- rotation matrices
def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def euler_zyx_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def R_to_euler_zyx(R: np.ndarray) -> tuple[float, float, float]:
    """``(roll, pitch, yaw)``; pitch is in [-90, 90] deg (gimbal lock at +-90)."""
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def hat(v) -> np.ndarray:
    """Skew-symmetric matrix with ``hat(a) @ b == cross(a, b)``."""
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


# ---------------------------------------------------------------- quaternions
def quat_mul(a, b) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def quat_conj(q) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    return np.concatenate([[math.cos(angle / 2)], math.sin(angle / 2) * axis])


def quat_to_R(q) -> np.ndarray:
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def quat_rotate(q, v) -> np.ndarray:
    """Rotate the 3-vector ``v`` by ``q``: ``q (0, v) q*``."""
    p = quat_mul(quat_mul(q, np.concatenate([[0.0], v])), quat_conj(q))
    return p[1:]


def quat_integrate(q, omega_body, dt: float) -> np.ndarray:
    """Propagate ``q`` by a constant body rate over ``dt`` (exact for constant omega)."""
    w = np.asarray(omega_body, dtype=float)
    th = float(np.linalg.norm(w)) * dt
    if th < 1e-12:
        dq = np.array([1.0, *(0.5 * w * dt)])
    else:
        dq = quat_from_axis_angle(w, th)
    q2 = quat_mul(q, dq)
    return q2 / np.linalg.norm(q2)


# ---------------------------------------------------------------- IMU and filters
def accel_tilt(ax: float, ay: float, az: float) -> tuple[float, float]:
    """Roll and pitch (rad) from a specific-force measurement at rest.

    At rest an accelerometer measures ``R^T (0, 0, g)``, which for ZYX Euler angles is
    ``g (-sin(theta), sin(phi) cos(theta), cos(phi) cos(theta))``.
    """
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.hypot(ay, az))
    return roll, pitch


def alpha_from_time_constant(tau: float, dt: float) -> float:
    """Complementary-filter weight for a crossover time constant ``tau``."""
    return tau / (tau + dt)


class ComplementaryFilter:
    """``angle = alpha (angle + rate dt) + (1 - alpha) accel_angle`` (one axis)."""

    def __init__(self, alpha: float = 0.98, angle0: float = 0.0):
        self.alpha = float(alpha)
        self.angle = float(angle0)

    def update(self, rate: float, accel_angle: float, dt: float) -> float:
        a = self.alpha
        self.angle = a * (self.angle + rate * dt) + (1.0 - a) * accel_angle
        return self.angle


class AngleKalman:
    """Two-state Kalman filter: ``x = [angle, gyro_bias]`` (one axis).

    Predict with the gyro (``angle += (rate - bias) dt``), correct with the
    accelerometer angle. ``q_angle``/``q_bias`` are process-noise intensities
    (per second), ``r_meas`` the accelerometer-angle variance.
    """

    def __init__(self, q_angle=0.001, q_bias=0.003, r_meas=0.03, angle0=0.0):
        self.x = np.array([float(angle0), 0.0])
        self.P = np.diag([1.0, 1.0])
        self.q_angle, self.q_bias, self.r = q_angle, q_bias, r_meas

    def predict(self, rate: float, dt: float) -> None:
        F = np.array([[1.0, -dt], [0.0, 1.0]])
        self.x = np.array([self.x[0] + (rate - self.x[1]) * dt, self.x[1]])
        Q = np.diag([self.q_angle * dt, self.q_bias * dt])
        self.P = F @ self.P @ F.T + Q

    def correct(self, accel_angle: float) -> None:
        H = np.array([1.0, 0.0])
        S = float(H @ self.P @ H) + self.r
        K = self.P @ H / S
        self.x = self.x + K * (accel_angle - self.x[0])
        self.P = (np.eye(2) - np.outer(K, H)) @ self.P

    def update(self, rate: float, accel_angle: float, dt: float) -> float:
        self.predict(rate, dt)
        self.correct(accel_angle)
        return float(self.x[0])

    @property
    def angle(self) -> float:
        return float(self.x[0])

    @property
    def bias(self) -> float:
        return float(self.x[1])
