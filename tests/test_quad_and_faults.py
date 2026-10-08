"""Quadrotor simulator sanity, agreement with the rotorfault study model, FDI helpers."""

from __future__ import annotations

import math

import numpy as np
import pytest

from bootcamp import fault
from bootcamp.logs import parse_lines
from bootcamp.quad import (
    CascadedController,
    QuadParams,
    Quadrotor,
    accel_to_tilt,
    allocate,
    hover_at,
    simulate_quad,
    waypoints,
)

rotorfault = pytest.importorskip("rotorfault", reason="rotorfault not installed")


def test_hover_is_an_equilibrium():
    q = Quadrotor()
    f = np.full(4, q.P.hover_thrust)
    for _ in range(1000):
        q.step(f, 0.001)
    assert np.abs(q.p - [0, 0, 1]).max() < 1e-9
    assert np.abs(q.w).max() < 1e-9


def test_parameters_and_mixer_match_the_rotorfault_study():
    from rotorfault.params import nominal_params

    P, Q = nominal_params(), QuadParams()
    assert Q.mass == pytest.approx(P.mass)
    assert np.allclose(Q.inertia, np.diag(P.inertia), rtol=1e-4)
    assert np.allclose(Q.mixer(), P.mixer())
    assert Q.thrust_to_weight == pytest.approx(P.thrust_to_weight)


def test_roll_response_matches_the_rotorfault_dynamics():
    from rotorfault.dynamics import QuadDynamics, hover_state, thrust_to_speed
    from rotorfault.params import nominal_params

    P = nominal_params()
    dyn = QuadDynamics(P)
    x = hover_state(P, (0, 0, 1.0))
    q = Quadrotor(QuadParams(), (0, 0, 1.0))
    f = np.array([1.55, 1.58, 1.58, 1.55])  # left rotors stronger: pure roll
    for _ in range(300):
        x = dyn.step(x, thrust_to_speed(f, P), np.ones(4), 0.001)
        q.step(f, 0.001)
    assert np.abs(x[:13] - q.x[:13]).max() < 1e-4


def test_allocation_is_exact_when_feasible_and_respects_limits():
    P = QuadParams()
    B = P.mixer()
    w = np.array([P.weight, 0.02, -0.01, 0.005])
    f = allocate(B, w[0], w[1:], P.f_min, P.f_max)
    assert np.allclose(B @ f, w)
    f = allocate(B, P.weight, np.array([0.3, 0.0, 0.2]), P.f_min, P.f_max)
    assert f.min() >= P.f_min - 1e-12 and f.max() <= P.f_max + 1e-12


def test_tilt_map_inverts_the_small_angle_model():
    for yaw in (0.0, 0.7, -2.0):
        roll, pitch = accel_to_tilt(1.0, -0.5, yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        ax = 9.81 * (pitch * c + roll * s)
        ay = 9.81 * (pitch * s - roll * c)
        assert (ax, ay) == pytest.approx((1.0, -0.5))


def test_cascade_flies_a_waypoint_step():
    log = simulate_quad(CascadedController(), waypoints([(0, 0, 1), (1, 0, 1)], 2.0), 6.0)
    assert not log.crashed
    assert np.abs(log.p[-1] - [1, 0, 1]).max() < 0.03
    assert log.p[:, 0].max() < 1.15  # moderate overshoot


def test_passive_cascade_loses_the_vehicle_after_a_complete_rotor_loss():
    log = simulate_quad(
        CascadedController(),
        hover_at((0, 0, 1)),
        4.0,
        eta_schedule=lambda t: np.array([0.0 if t > 1.0 else 1.0, 1, 1, 1]),
    )
    assert log.crashed


# ------------------------------------------------------------------ FDI helpers
def test_eta_min_matches_the_thrust_share_argument():
    P = QuadParams()
    eta = fault.eta_min_yaw_hover(P.thrust_to_weight)
    assert eta * P.f_max == pytest.approx(P.hover_thrust)


def test_cusum_detects_a_shift_and_not_noise():
    rng = np.random.default_rng(3)
    r = rng.normal(0, 1, 2000)
    k, _ = fault.cusum(r, 1.0, 1.0, 12.0)
    assert k == -1
    r[1000:] += 1.0
    k, _ = fault.cusum(r, 1.0, 1.0, 12.0)
    assert 1000 < k < 1060


def test_isolation_and_reallocation():
    P = QuadParams()
    B = P.mixer()
    f = np.full(4, P.hover_thrust)
    D = -(B * f[None, :])  # signature of a loss on each rotor
    for j in range(4):
        assert fault.isolate(0.3 * D[:, j], D) == j
    eta = np.array([1.0, 0.6, 1.0, 1.0])
    w = np.array([P.weight, 0.01, 0.0, 0.0])
    f = fault.allocate_with_eta(B, eta, w[0], w[1:])
    assert np.allclose((B * eta[None, :]) @ f, w)


def test_rotorfault_pipeline_recovers_where_passive_fails():
    _, passive = fault.run_case(1.0, rotor=1, baseline="passive", duration=3.0, onset=1.0)
    _, est = fault.run_case(1.0, rotor=1, baseline="estimated", duration=3.0, onset=1.0)
    assert passive.lost
    assert not est.lost
    assert est.isolated_rotor == 1
    assert 0 < est.detection_delay_ms < 100


def test_log_parser_skips_comments_and_cut_lines():
    lines = ["# banner", "t_ms,sp,y", "0,1,0", "10,1,0.", "20,1", "garbage", "30,1,0.5"]
    d = parse_lines(lines)
    assert list(d) == ["t_ms", "sp", "y"]
    assert d["t_ms"].tolist() == [0, 10, 30]
