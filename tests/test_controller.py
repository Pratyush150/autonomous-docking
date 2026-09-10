"""Control laws: pure pursuit onto the axis, in-place alignment, the blind arc plan."""

from __future__ import annotations

import math

import pytest

from autodock.controller import ControllerGains, DockingController
from autodock.geometry import DockError, DockGeometry
from autodock.kinematics import DiffDriveLimits, body_to_wheels
from autodock.states import DockingPolicy, DockState

CTRL = DockingController()
LIMITS = DiffDriveLimits()
DOCK = DockGeometry()
POLICY = DockingPolicy()


def test_pure_pursuit_steers_towards_the_axis_from_either_side():
    left = CTRL.pure_pursuit(DockError(s=2.0, lateral=0.30, yaw=0.0))
    right = CTRL.pure_pursuit(DockError(s=2.0, lateral=-0.30, yaw=0.0))
    # Offset to the left of the axis, pointing along it: turn right.
    assert left[1] < 0.0
    assert right[1] > 0.0
    assert left[0] > 0.0 and right[0] > 0.0


def test_pure_pursuit_goes_straight_when_already_on_the_axis():
    v, omega = CTRL.pure_pursuit(DockError(s=2.0, lateral=0.0, yaw=0.0))
    assert omega == pytest.approx(0.0, abs=1e-12)
    assert v > 0.0


def test_pure_pursuit_turns_in_place_when_the_target_is_behind():
    v, omega = CTRL.pure_pursuit(DockError(s=2.0, lateral=0.0, yaw=math.pi))
    assert v == pytest.approx(0.0)
    assert abs(omega) > 0.0


def test_pure_pursuit_slows_down_as_the_standoff_approaches():
    far = CTRL.pure_pursuit(DockError(s=3.0, lateral=0.0, yaw=0.0))[0]
    near = CTRL.pure_pursuit(DockError(s=POLICY.standoff + 0.05, lateral=0.0, yaw=0.0))[0]
    assert near < far


def test_a_retry_approaches_more_slowly_and_tracks_more_tightly():
    first = CTRL.pure_pursuit(DockError(s=2.0, lateral=0.3, yaw=0.0), repositions=0)
    retry = CTRL.pure_pursuit(DockError(s=2.0, lateral=0.3, yaw=0.0), repositions=2)
    assert retry[0] < first[0]
    assert abs(retry[1] / retry[0]) > abs(first[1] / first[0])  # tighter curvature


def test_turn_in_place_does_not_translate():
    v, omega = CTRL.turn_in_place(DockError(s=0.75, lateral=0.05, yaw=math.radians(10.0)))
    assert v == 0.0
    assert omega < 0.0
    left, right = body_to_wheels(v, omega, LIMITS)
    assert left == pytest.approx(-right)


def test_line_follow_is_zero_on_the_axis_and_corrects_off_it():
    assert CTRL.line_follow(DockError(0.5, 0.0, 0.0), steps_since_fix=0)[1] == pytest.approx(0.0)
    off = CTRL.line_follow(DockError(0.5, 0.02, 0.0), steps_since_fix=0)
    assert off[1] < 0.0
    yawed = CTRL.line_follow(DockError(0.5, 0.0, math.radians(3.0)), steps_since_fix=0)
    assert yawed[1] < 0.0


def test_line_follow_freezes_when_the_fix_goes_stale():
    v, omega = CTRL.line_follow(DockError(0.5, 0.02, 0.0), steps_since_fix=100)
    assert omega == 0.0
    assert v > 0.0


def test_line_follow_gains_are_critically_damped_at_the_declared_length():
    g = ControllerGains()
    lam = 0.5 * g.line_k_yaw
    assert g.line_k_y == pytest.approx(lam * lam)


def test_blind_plan_is_straight_when_the_estimate_is_perfect():
    plan = CTRL.plan_blind_arc(DockError(0.35, 0.0, 0.0))
    assert plan.curvature == pytest.approx(0.0, abs=1e-9)
    assert plan.predicted_swept == pytest.approx(0.0, abs=1e-9)


def test_blind_plan_beats_driving_straight_when_off_the_axis():
    est = DockError(0.35, 0.010, 0.0)
    plan = CTRL.plan_blind_arc(est)
    straight_swept = abs(est.lateral + est.yaw * est.s)
    assert plan.curvature < 0.0
    assert plan.predicted_swept < straight_swept
    assert plan.predicted_swept < 0.75 * straight_swept


def test_blind_plan_trades_lateral_against_heading_using_the_mechanical_criterion():
    """Nulling lateral offset completely is not optimal once heading is charged for."""
    est = DockError(0.35, 0.020, 0.0)
    plan = CTRL.plan_blind_arc(est)
    null_lateral = -2.0 * est.lateral / (est.s ** 2)
    assert abs(plan.curvature) < abs(null_lateral)
    assert plan.predicted_lateral * est.lateral > 0.0  # some offset is kept on purpose


def test_blind_plan_respects_the_curvature_limit():
    hopeless = DockError(0.35, 0.5, 0.0)
    plan = CTRL.plan_blind_arc(hopeless)
    assert abs(plan.curvature) <= ControllerGains().blind_max_curvature + 1e-12
    assert plan.predicted_swept > DOCK.lateral_tolerance  # and the gate will refuse it


def test_commands_are_always_inside_the_wheel_envelope():
    cases = [
        (DockState.SEARCH, DockError(2.0, 0.0, 0.0)),
        (DockState.APPROACH, DockError(3.0, 2.0, 3.0)),
        (DockState.ALIGN, DockError(0.75, 0.0, 2.0)),
        (DockState.FINAL_APPROACH, DockError(0.4, 0.4, 1.0)),
        (DockState.BLIND_APPROACH, DockError(0.3, 0.0, 0.0)),
        (DockState.RETREAT, DockError(0.5, 0.0, 0.0)),
    ]
    for state, est in cases:
        v, omega = CTRL.command(state, est, blind_curvature=5.0)
        left, right = body_to_wheels(v, omega, LIMITS)
        assert max(abs(left), abs(right)) <= LIMITS.max_wheel_speed + 1e-9, state


def test_the_states_that_must_not_move_do_not_move():
    for state in (DockState.ACQUIRE, DockState.COMMIT, DockState.VERIFY, DockState.DOCKED, DockState.ABORTED):
        assert CTRL.command(state, DockError(0.4, 0.05, 0.1)) == (0.0, 0.0), state


def test_retreat_and_reposition_reverse():
    assert CTRL.command(DockState.RETREAT, DockError(0.4, 0.0, 0.0))[0] < 0.0
    assert CTRL.command(DockState.REPOSITION, DockError(0.75, 0.0, 0.0))[0] < 0.0
