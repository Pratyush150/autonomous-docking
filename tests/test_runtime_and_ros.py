"""The online runtime, and the ROS wrapper's behaviour with no ROS installed."""

from __future__ import annotations

import math

import pytest

from autodock import ros_node
from autodock.geometry import DockError
from autodock.perception import MarkerObservation
from autodock.runtime import DockingRuntime
from autodock.states import DockState


def obs(s: float, lateral: float, yaw: float) -> MarkerObservation:
    return MarkerObservation(s, lateral, yaw, 0.004, 0.004, math.radians(1.0), 0)


def test_runtime_starts_searching_and_spins():
    rt = DockingRuntime()
    cmd = rt.step(now=0.0, dt=0.04)
    assert cmd.state is DockState.SEARCH
    assert cmd.v == 0.0
    assert abs(cmd.omega) > 0.0


def test_runtime_advances_through_acquisition_on_clean_fixes():
    rt = DockingRuntime()
    for i in range(30):
        rt.step(now=i * 0.04, dt=0.04, observation=obs(2.0, 0.0, 0.0))
        rt.feed_odometry(0.0, 0.0, 0.04)
    assert rt.state is DockState.APPROACH
    assert rt.estimate.s == pytest.approx(2.0, abs=0.05)


def test_runtime_odometry_moves_the_estimate_and_the_odometer():
    rt = DockingRuntime()
    rt.step(now=0.0, dt=0.04, observation=obs(2.0, 0.0, 0.0))
    rt.feed_odometry(0.5, 0.0, 0.1)
    assert rt.odom_distance == pytest.approx(0.05)
    assert rt.estimate.s == pytest.approx(1.95, abs=0.02)


def test_runtime_stops_dead_in_a_terminal_state():
    rt = DockingRuntime()
    rt.machine.state = DockState.DOCKED
    cmd = rt.step(now=1.0, dt=0.04)
    assert (cmd.v, cmd.omega) == (0.0, 0.0)


def test_ros_module_imports_without_ros_and_says_so():
    assert ros_node.RCLPY_AVAILABLE is False


def test_constructing_the_node_without_ros_fails_loudly():
    with pytest.raises((RuntimeError, TypeError)):
        ros_node.DockingNode()


def test_marker_pose_conversion_is_the_inverse_of_the_geometry():
    """A marker 2 m ahead and square on means s=2, no lateral, no heading error."""
    o = ros_node.observation_from_marker_pose(
        2.0, 0.0, 0.0, sigma_s=0.01, sigma_lateral=0.01, sigma_yaw=0.01
    )
    assert o.s == pytest.approx(2.0)
    assert o.lateral == pytest.approx(0.0)
    assert o.yaw == pytest.approx(0.0)


def test_marker_pose_conversion_matches_the_dock_frame_definition():
    truth = DockError(s=1.5, lateral=0.08, yaw=math.radians(4.0))
    # Where that robot would see the marker, in its own camera frame.
    c, s = math.cos(truth.yaw), math.sin(truth.yaw)
    x = truth.s * c - truth.lateral * s
    y = -truth.s * s - truth.lateral * c
    back = ros_node.observation_from_marker_pose(
        x, y, -truth.yaw, sigma_s=0.01, sigma_lateral=0.01, sigma_yaw=0.01
    )
    assert back.s == pytest.approx(truth.s)
    assert back.lateral == pytest.approx(truth.lateral)
    assert back.yaw == pytest.approx(truth.yaw)
