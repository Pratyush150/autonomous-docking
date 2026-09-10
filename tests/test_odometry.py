"""Drive-train asymmetry and the odometry that cannot see it."""

from __future__ import annotations

import numpy as np
import pytest

from autodock.geometry import Pose2D
from autodock.kinematics import DiffDriveLimits, body_to_wheels
from autodock.odometry import DriveTrain, DriveTrainSpec, WheelOdometry

LIMITS = DiffDriveLimits()


def perfect_drive() -> DriveTrain:
    spec = DriveTrainSpec(asymmetry_std=0.0, scale_error_std=0.0, gyro_bias_std=0.0, slip_noise_std=0.0)
    return DriveTrain(spec, LIMITS, np.random.default_rng(0))


def test_a_perfect_drive_train_matches_its_own_odometry_exactly():
    drive = perfect_drive()
    odom = WheelOdometry(LIMITS)
    pose = Pose2D(0.0, 0.0, 0.0)
    wheels = body_to_wheels(0.3, 0.2, LIMITS)
    for _ in range(200):
        pose, _, _ = drive.step(pose, wheels, 0.02)
        odom.step(wheels, 0.02)
    assert pose.x == pytest.approx(odom.pose.x, abs=1e-12)
    assert pose.y == pytest.approx(odom.pose.y, abs=1e-12)
    assert pose.theta == pytest.approx(odom.pose.theta, abs=1e-12)


def test_asymmetry_curves_a_straight_command_at_the_predicted_rate():
    drive = perfect_drive()
    drive.gain_left, drive.gain_right = 1.0, 1.02
    expected = (1.02 - 1.0) / 1.01 / LIMITS.track_width
    assert drive.straight_line_curvature == pytest.approx(expected)

    pose = Pose2D(0.0, 0.0, 0.0)
    wheels = body_to_wheels(0.2, 0.0, LIMITS)
    distance = 0.0
    for _ in range(500):
        pose, v, _ = drive.step(pose, wheels, 0.01)
        distance += v * 0.01
    # Commanded dead straight, arrived on an arc curving left.
    assert pose.y > 0.0
    assert pose.y == pytest.approx(0.5 * expected * distance * distance, rel=0.02)


def test_lateral_drift_over_a_blind_segment_grows_quadratically():
    """Doubling the blind distance quadruples the lateral error. This is the
    single most important scaling law in the whole package."""
    drive = perfect_drive()
    drive.gain_left, drive.gain_right = 1.0, 1.02
    wheels = body_to_wheels(0.2, 0.0, LIMITS)

    def travel(steps: int) -> float:
        pose = Pose2D(0.0, 0.0, 0.0)
        for _ in range(steps):
            pose, _, _ = drive.step(pose, wheels, 0.01)
        return pose.y

    short = travel(250)
    long = travel(500)
    assert long / short == pytest.approx(4.0, rel=0.05)


def test_the_odometer_measures_path_length_not_displacement():
    odom = WheelOdometry(LIMITS)
    wheels = body_to_wheels(0.0, 1.0, LIMITS)   # spin in place
    for _ in range(100):
        odom.step(wheels, 0.01)
    assert odom.distance == pytest.approx(0.0)
    odom.reset()
    wheels = body_to_wheels(-0.2, 0.0, LIMITS)  # reversing still adds distance
    for _ in range(100):
        odom.step(wheels, 0.01)
    assert odom.distance == pytest.approx(0.2)


def test_drift_scale_widens_the_spread_of_drawn_drive_trains():
    def spread(scale: float) -> float:
        rng = np.random.default_rng(4)
        drive = DriveTrain(DriveTrainSpec(drift_scale=scale), LIMITS, rng)
        values = []
        for _ in range(400):
            drive.sample()
            values.append(drive.straight_line_curvature)
        return float(np.std(values))

    assert spread(3.0) == pytest.approx(3.0 * spread(1.0), rel=0.1)
    assert spread(0.0) == pytest.approx(0.0, abs=1e-12)


def test_gyro_bias_rotates_the_robot_with_the_wheels_stopped():
    drive = perfect_drive()
    drive.gyro_bias = 0.05
    pose = Pose2D(0.0, 0.0, 0.0)
    for _ in range(100):
        pose, _, _ = drive.step(pose, (0.0, 0.0), 0.01)
    assert pose.theta == pytest.approx(0.05, rel=1e-6)


def test_same_seed_gives_the_same_drive_train():
    a = DriveTrain(DriveTrainSpec(), LIMITS, np.random.default_rng(9))
    b = DriveTrain(DriveTrainSpec(), LIMITS, np.random.default_rng(9))
    assert a.gain_left == b.gain_left
    assert a.gain_right == b.gain_right
    assert a.gyro_bias == b.gyro_bias
