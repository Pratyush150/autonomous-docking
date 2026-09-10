"""Frames, the dock-frame error triple, and the mechanical engagement criterion."""

from __future__ import annotations

import math

import pytest

from autodock.geometry import DockError, DockGeometry, Pose2D, wrap_angle


@pytest.mark.parametrize(
    "angle,expected",
    [(0.0, 0.0), (math.pi, math.pi), (-math.pi, math.pi), (3 * math.pi, math.pi), (2.5 * math.pi, 0.5 * math.pi)],
)
def test_wrap_angle_lands_in_half_open_interval(angle, expected):
    assert wrap_angle(angle) == pytest.approx(expected, abs=1e-12)


def test_pose_inverse_is_a_true_inverse():
    pose = Pose2D(1.3, -0.7, 2.1)
    identity = pose.compose(pose.inverse())
    assert identity.x == pytest.approx(0.0, abs=1e-12)
    assert identity.y == pytest.approx(0.0, abs=1e-12)
    assert identity.theta == pytest.approx(0.0, abs=1e-12)


def test_relative_to_matches_hand_computation():
    frame = Pose2D(1.0, 2.0, math.pi / 2)
    point = Pose2D(1.0, 5.0, math.pi / 2)
    rel = point.relative_to(frame)
    assert rel.x == pytest.approx(3.0)
    assert rel.y == pytest.approx(0.0, abs=1e-12)
    assert rel.theta == pytest.approx(0.0, abs=1e-12)


def test_standoff_lies_on_the_dock_axis_at_the_stated_distance():
    dock = DockGeometry(x=2.0, y=-1.0, yaw=math.radians(37.0), standoff=0.75)
    err = dock.error(dock.standoff_pose())
    assert err.s == pytest.approx(0.75)
    assert err.lateral == pytest.approx(0.0, abs=1e-12)
    assert err.yaw == pytest.approx(0.0, abs=1e-12)


def test_error_and_world_pose_round_trip():
    dock = DockGeometry(x=-3.0, y=4.5, yaw=math.radians(200.0))
    err = DockError(1.234, -0.056, math.radians(4.0))
    back = dock.error(dock.world_pose(err))
    assert back.s == pytest.approx(err.s)
    assert back.lateral == pytest.approx(err.lateral)
    assert back.yaw == pytest.approx(err.yaw)


def test_range_bearing_and_incidence_are_consistent_with_the_geometry():
    err = DockError(s=2.0, lateral=0.0, yaw=0.0)
    assert err.range_to_marker == pytest.approx(2.0)
    assert err.bearing_to_marker == pytest.approx(0.0, abs=1e-12)
    assert err.incidence == pytest.approx(0.0, abs=1e-12)

    # Sitting 1 m to the left of the axis and 1 m out: the marker is 45 degrees
    # to the right of a robot that is pointing along the axis.
    off = DockError(s=1.0, lateral=1.0, yaw=0.0)
    assert off.bearing_to_marker == pytest.approx(-math.pi / 4)
    assert off.incidence == pytest.approx(math.pi / 4)


def test_yaw_eats_lateral_budget_through_the_swept_half_width():
    dock = DockGeometry()
    straight = DockError(0.0, 0.020, 0.0)
    yawed = DockError(0.0, 0.020, math.radians(4.0))
    assert dock.swept_half_width(straight) == pytest.approx(0.020)
    extra = dock.shoe_length * math.sin(math.radians(4.0))
    assert dock.swept_half_width(yawed) == pytest.approx(0.020 + extra)
    # The coupling is what makes alignment matter: 20 mm of offset is fine on
    # its own, and the same 20 mm with 4 degrees of yaw is not.
    assert dock.engagement_ok(straight)
    assert not dock.engagement_ok(yawed)


def test_engagement_respects_both_the_swept_width_and_the_hard_yaw_limit():
    dock = DockGeometry()
    # Perfectly centred but past the jam angle: the shoe catches the funnel wall.
    assert not dock.engagement_ok(DockError(0.0, 0.0, dock.yaw_tolerance * 1.01))
    assert dock.engagement_ok(DockError(0.0, 0.0, dock.yaw_tolerance * 0.99))
    # Exactly at the lateral limit with no yaw: accepted.
    assert dock.engagement_ok(DockError(0.0, dock.lateral_tolerance, 0.0))
    assert not dock.engagement_ok(DockError(0.0, dock.lateral_tolerance * 1.01, 0.0))


def test_hitting_the_dock_body_is_a_wider_test_than_mating():
    dock = DockGeometry()
    grazing = DockError(0.0, 0.5 * (dock.lateral_tolerance + dock.funnel_outer_half_width), 0.0)
    assert dock.hits_dock_body(grazing)
    assert not dock.engagement_ok(grazing)
    assert not dock.hits_dock_body(DockError(0.0, dock.funnel_outer_half_width * 1.2, 0.0))
