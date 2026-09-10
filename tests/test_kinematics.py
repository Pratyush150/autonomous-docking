"""Differential-drive kinematics: limits, curvature preservation, exact arcs."""

from __future__ import annotations

import math

import pytest

from autodock.geometry import Pose2D
from autodock.kinematics import (
    DiffDriveLimits,
    body_to_wheels,
    clamp_twist,
    integrate,
    rate_limit_wheels,
    wheels_to_body,
)

LIMITS = DiffDriveLimits()


@pytest.mark.parametrize("v,omega", [(0.0, 0.0), (0.3, 0.0), (0.0, 1.4), (-0.2, 0.9), (0.12, -2.3)])
def test_wheel_conversion_round_trips(v, omega):
    left, right = body_to_wheels(v, omega, LIMITS)
    v_back, omega_back = wheels_to_body(left, right, LIMITS)
    assert v_back == pytest.approx(v)
    assert omega_back == pytest.approx(omega)


def test_spin_in_place_drives_the_wheels_in_opposite_directions():
    left, right = body_to_wheels(0.0, 1.0, LIMITS)
    assert left == pytest.approx(-right)
    assert right > 0.0


def test_declared_limits_agree_with_the_wheel_limit():
    left, right = body_to_wheels(LIMITS.max_linear, 0.0, LIMITS)
    assert max(abs(left), abs(right)) == pytest.approx(LIMITS.max_wheel_speed)
    left, right = body_to_wheels(0.0, LIMITS.max_angular, LIMITS)
    assert max(abs(left), abs(right)) == pytest.approx(LIMITS.max_wheel_speed)


def test_clamp_preserves_curvature_rather_than_speed():
    v, omega = 3.0, 6.0
    v_c, omega_c = clamp_twist(v, omega, LIMITS)
    assert v_c < v
    # The whole point: the robot follows the same geometric path, slower.
    assert omega_c / v_c == pytest.approx(omega / v)
    left, right = body_to_wheels(v_c, omega_c, LIMITS)
    assert max(abs(left), abs(right)) == pytest.approx(LIMITS.max_wheel_speed)


def test_clamp_is_a_no_op_inside_the_envelope():
    v, omega = 0.1, 0.4
    assert clamp_twist(v, omega, LIMITS) == (v, omega)


def test_clamp_handles_a_pure_spin_without_dividing_by_zero():
    v_c, omega_c = clamp_twist(0.0, 50.0, LIMITS)
    assert v_c == pytest.approx(0.0)
    assert omega_c == pytest.approx(LIMITS.max_angular)


def test_rate_limit_bounds_each_wheel_independently():
    dt = 0.04
    step = LIMITS.max_wheel_accel * dt
    out = rate_limit_wheels((0.0, 0.0), (100.0, -100.0), dt, LIMITS)
    assert out[0] == pytest.approx(step)
    assert out[1] == pytest.approx(-step)
    # A command inside the slew limit passes through untouched.
    assert rate_limit_wheels((1.0, 1.0), (1.0 + 0.5 * step, 1.0), dt, LIMITS)[0] == pytest.approx(1.0 + 0.5 * step)


def test_constant_twist_traces_a_circle_of_the_right_radius():
    v, omega = 0.25, 0.5
    radius = v / omega
    period = 2.0 * math.pi / omega
    steps = 10_000
    dt = period / steps
    pose = Pose2D(0.0, 0.0, 0.0)
    for _ in range(steps):
        pose = integrate(pose, v, omega, dt)
    # A full revolution returns to the start.
    assert math.hypot(pose.x, pose.y) < 1e-9
    # Halfway round it is one diameter to the left.
    half = Pose2D(0.0, 0.0, 0.0)
    for _ in range(steps // 2):
        half = integrate(half, v, omega, dt)
    assert half.x == pytest.approx(0.0, abs=1e-9)
    assert half.y == pytest.approx(2.0 * radius, abs=1e-9)


def test_arc_integration_beats_euler_on_a_single_large_step():
    v, omega, dt = 0.5, 1.5, 0.2
    exact = integrate(Pose2D(0.0, 0.0, 0.0), v, omega, dt)
    euler_x = v * dt * math.cos(0.0)
    # Euler keeps the robot on the tangent; the true arc has already curved away.
    assert exact.x < euler_x
    assert exact.y > 0.0
    radius = v / omega
    assert math.hypot(exact.x, exact.y - radius) == pytest.approx(radius, abs=1e-12)


def test_the_drive_is_non_holonomic():
    """No twist produces sideways body-frame motion in the limit of small dt."""
    pose = Pose2D(0.0, 0.0, 0.3)
    for v, omega in [(0.2, 0.0), (0.2, 1.0), (-0.15, -2.0), (0.0, 2.0)]:
        for dt in (1e-3, 1e-4):
            moved = integrate(pose, v, omega, dt)
            dx, dy = moved.x - pose.x, moved.y - pose.y
            lateral = -math.sin(pose.theta) * dx + math.cos(pose.theta) * dy
            forward = math.cos(pose.theta) * dx + math.sin(pose.theta) * dy
            assert forward == pytest.approx(v * dt, abs=1e-9)
            # Sideways displacement is second order in dt, never first order.
            assert abs(lateral) <= 1.0 * abs(v * omega) * dt * dt + 1e-15


def test_zero_angular_rate_takes_the_straight_line_branch_exactly():
    pose = integrate(Pose2D(1.0, 2.0, math.pi / 3), 0.4, 0.0, 0.25)
    assert pose.theta == pytest.approx(math.pi / 3)
    assert pose.x == pytest.approx(1.0 + 0.1 * math.cos(math.pi / 3))
    assert pose.y == pytest.approx(2.0 + 0.1 * math.sin(math.pi / 3))
