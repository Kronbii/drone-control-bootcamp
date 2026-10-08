"""Sanity checks of the teaching models: plants, metrics, rig, attitude, quadrotor."""

from __future__ import annotations

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from bootcamp import attitude as att
from bootcamp import easypid as ep
from bootcamp.loop import simulate_open_loop, simulate_pid
from bootcamp.metrics import fit_fopdt_two_point, step_info
from bootcamp.plants import DeadTime, first_order, rc_ladder, second_order
from bootcamp.rig import RigParams, linear_model, mix, simulate_rig


# ------------------------------------------------------------------ plants
def test_first_order_matches_the_analytic_step():
    t, y, _ = simulate_open_loop(first_order(2.0, 0.5), 1.0, 3.0, 0.01)
    assert np.allclose(y, 2.0 * (1 - np.exp(-t / 0.5)), atol=1e-12)


def test_rc_ladder_poles_and_gain():
    p = rc_ladder(10e3, 100e-6, 3)
    assert p.dc_gain() == pytest.approx(1.0)
    # 1 / (1 + 6 s + 5 s^2 + s^3) for RC = 1 s
    assert np.sort(p.poles().real) == pytest.approx(np.sort(np.roots([1, 5, 6, 1]).real))


def test_second_order_overshoot_matches_formula():
    zeta = 0.3
    t, y, _ = simulate_open_loop(second_order(1.0, 4.0, zeta), 1.0, 8.0, 0.001)
    info = step_info(t, y, 1.0)
    assert info.overshoot_pct == pytest.approx(
        100 * math.exp(-math.pi * zeta / math.sqrt(1 - zeta**2)), rel=1e-3
    )


def test_dead_time_delays_by_n_samples():
    d = DeadTime(3)
    assert [d(x) for x in [1, 2, 3, 4, 5]] == [0, 0, 0, 1, 2]


def test_fopdt_fit_recovers_parameters():
    K, tau, L, dt = 2.0, 1.5, 0.4, 0.01
    t = np.arange(0, 12, dt)
    y = np.where(t < L, 0.0, K * (1 - np.exp(-(t - L) / tau)))
    m = fit_fopdt_two_point(t, y, 1.0)
    assert (m.K, m.tau, m.L) == pytest.approx((K, tau, L), rel=0.03)


def test_pi_removes_the_offset_of_p_control():
    plant = first_order(1.0, 1.0)
    p_only = simulate_pid(plant, ep.PIDController(4.0, 0.0, 0.0, -100, 100), 1.0, 8.0, 0.01)
    pi = simulate_pid(plant, ep.PIDController(4.0, 4.0, 0.0, -100, 100), 1.0, 8.0, 0.01)
    assert p_only.y[-1] == pytest.approx(4 / 5, rel=1e-3)  # K Kp / (1 + K Kp)
    assert pi.y[-1] == pytest.approx(1.0, abs=1e-3)


# ------------------------------------------------------------------ rig
def test_rig_free_swing_period_matches_the_pendulum_formula():
    p = RigParams(damping=0.0)
    log = simulate_rig(lambda t, sp, m, dt: 0.0, 12.0, theta0_deg=5.0, params=p, dt=0.002)
    up = np.nonzero((log.theta[:-1] < 0) & (log.theta[1:] >= 0))[0]
    period = np.diff(log.t[up]).mean()
    expected = 2 * math.pi * math.sqrt(p.inertia / p.gravity_stiffness)
    assert period == pytest.approx(expected, rel=0.01)


def test_rig_pid_holds_a_setpoint():
    pid = ep.PIDController(2.0, 1.0, 0.5, -200, 200)
    pid.begin()

    def ctrl(t, sp, m, dt):
        return pid.update(sp, m["pot"], dt * 1000)

    log = simulate_rig(ctrl, 8.0, setpoint=10.0, seed=1)
    assert abs(log.theta[-200:].mean() - 10.0) < 0.5
    assert np.all(np.abs(log.theta) < 30.0)


def test_mixer_keeps_differential_by_shifting_the_base():
    m1, m2 = mix(1550, 100, 1100, 1600)
    assert m1 - m2 == pytest.approx(200)
    assert max(m1, m2) <= 1600
    m1, m2 = mix(1350, 400, 1100, 1600)  # cannot fit: differential limited to the span
    assert m1 - m2 == pytest.approx(500)


def test_rig_linear_model_is_stable_open_loop_when_cg_is_below_the_pivot():
    A, _ = linear_model(RigParams(cg_below_pivot=0.01))
    assert np.all(np.linalg.eigvals(A).real < 0)
    A, _ = linear_model(RigParams(cg_below_pivot=-0.01))
    assert np.any(np.linalg.eigvals(A).real > 0)


def test_accelerometer_angle_agrees_with_the_beam_at_rest():
    log = simulate_rig(
        lambda t, sp, m, dt: 0.0, 0.5, theta0_deg=12.0, params=RigParams(cg_below_pivot=0.0)
    )
    assert np.median(log.accel_angle) == pytest.approx(12.0, abs=1.0)


# ------------------------------------------------------------------ attitude
@pytest.mark.parametrize("angles", [(0.1, -0.2, 0.3), (1.0, 0.4, -2.0), (-0.5, 1.2, 3.0)])
def test_euler_and_quaternions_agree_with_scipy(angles):
    roll, pitch, yaw = angles
    R = att.euler_zyx_to_R(roll, pitch, yaw)
    ref = Rotation.from_euler("ZYX", [yaw, pitch, roll]).as_matrix()
    assert np.allclose(R, ref)
    assert att.R_to_euler_zyx(R) == pytest.approx(angles)
    q = Rotation.from_matrix(R).as_quat()  # scipy is scalar-last
    q = np.array([q[3], q[0], q[1], q[2]])
    assert np.allclose(att.quat_to_R(q), R)
    v = np.array([0.3, -1.0, 2.0])
    assert np.allclose(att.quat_rotate(q, v), R @ v)


def test_accel_tilt_inverts_gravity():
    roll, pitch = 0.3, -0.4
    f = att.euler_zyx_to_R(roll, pitch, 1.0).T @ np.array([0, 0, 9.81])
    assert att.accel_tilt(*f) == pytest.approx((roll, pitch))


def test_quaternion_integration_is_exact_for_constant_rate():
    q = np.array([1.0, 0, 0, 0])
    for _ in range(1000):
        q = att.quat_integrate(q, [0.0, 0.0, math.pi], 0.001)
    assert np.allclose(att.quat_to_R(q), att.rot_z(math.pi), atol=1e-9)


def test_complementary_filter_rejects_gyro_bias():
    rng = np.random.default_rng(0)
    cf = att.ComplementaryFilter(att.alpha_from_time_constant(0.5, 0.005))
    for _ in range(4000):  # stationary at 10 deg, gyro bias 2 deg/s, noisy accelerometer
        cf.update(2.0 + rng.normal(0, 0.3), 10.0 + rng.normal(0, 2.0), 0.005)
    assert cf.angle == pytest.approx(10.0 + 2.0 * 0.5, abs=0.6)  # bias leaks as bias * tau


def test_kalman_estimates_the_gyro_bias():
    rng = np.random.default_rng(1)
    kf = att.AngleKalman(r_meas=4.0)
    for _ in range(6000):
        kf.update(2.0 + rng.normal(0, 0.3), 10.0 + rng.normal(0, 2.0), 0.005)
    assert kf.bias == pytest.approx(2.0, abs=0.3)
    assert kf.angle == pytest.approx(10.0, abs=0.5)
