"""The dock-frame estimator: latency compensation, gating, floors, divergence recovery."""

from __future__ import annotations

import math

import pytest

from autodock.estimator import DockEstimator, EstimatorNoise
from autodock.geometry import Pose2D
from autodock.perception import MarkerObservation

R = (0.004 ** 2, 0.004 ** 2, math.radians(1.0) ** 2)


def obs(s: float, lateral: float, yaw: float, age: int = 0) -> MarkerObservation:
    return MarkerObservation(s, lateral, yaw, 0.004, 0.004, math.radians(1.0), age)


def test_prediction_moves_the_estimate_and_ages_the_fix():
    est = DockEstimator(pose=Pose2D(-2.0, 0.0, 0.0))
    est.steps_since_fix = 0
    est.predict(0.5, 0.0, 0.1)
    assert est.error.s == pytest.approx(1.95)
    assert est.steps_since_fix == 1


def test_prediction_inflates_the_covariance():
    est = DockEstimator()
    est.var = (1e-6, 1e-6, 1e-6)
    before = est.var
    est.predict(0.5, 0.0, 0.1)
    assert all(a >= b for a, b in zip(est.var, before))


def test_a_fix_pulls_the_estimate_towards_the_measurement():
    est = DockEstimator(pose=Pose2D(-2.0, 0.10, 0.0))
    est.var = (0.04, 0.04, math.radians(20.0) ** 2)
    assert est.update(obs(2.0, 0.0, 0.0))
    assert abs(est.error.lateral) < 0.10
    assert est.total_fixes == 1
    assert est.steps_since_fix == 0


def test_latency_is_compensated_by_replaying_the_odometry():
    """A late fix must not drag the estimate backwards by v * latency."""
    noise = EstimatorNoise()
    est = DockEstimator(noise, pose=Pose2D(-2.0, 0.0, 0.0))
    est.var = (0.04, 0.04, math.radians(20.0) ** 2)
    for _ in range(3):
        est.predict(0.5, 0.0, 0.1)          # now at s = 1.85
    assert est.error.s == pytest.approx(1.85)
    # A perfect observation of where the robot was 3 ticks ago.
    est.update(obs(2.0, 0.0, 0.0, age=3))
    assert est.error.s == pytest.approx(1.85, abs=1e-9)

    # The same fix applied without compensation would land at 2.0 - drop it in
    # with age 0 and watch the estimate jump backwards.
    naive = DockEstimator(noise, pose=Pose2D(-1.85, 0.0, 0.0))
    naive.var = (0.04, 0.04, math.radians(20.0) ** 2)
    naive.update(obs(2.0, 0.0, 0.0, age=0))
    assert naive.error.s > 1.9


def test_covariance_never_falls_below_the_floor():
    noise = EstimatorNoise()
    est = DockEstimator(noise, pose=Pose2D(-1.0, 0.0, 0.0))
    for _ in range(500):
        est.update(obs(1.0, 0.0, 0.0))
    assert est.lateral_sigma == pytest.approx(noise.pos_floor)
    assert est.yaw_sigma == pytest.approx(noise.yaw_floor)


def test_the_floor_keeps_the_filter_listening():
    """Without a floor the gain collapses; with one it still tracks a step."""
    est = DockEstimator(pose=Pose2D(-1.0, 0.0, 0.0))
    for _ in range(500):
        est.update(obs(1.0, 0.0, 0.0))
    for _ in range(30):
        est.update(obs(1.0, 0.010, 0.0))
    assert est.error.lateral > 0.005


def test_an_outlier_is_gated_out():
    est = DockEstimator(pose=Pose2D(-1.0, 0.0, 0.0))
    for _ in range(20):
        est.update(obs(1.0, 0.0, 0.0))
    before = est.error.lateral
    assert not est.update(obs(1.0, 0.0, math.radians(60.0)))
    assert est.rejected_fixes == 1
    assert est.error.lateral == pytest.approx(before)
    assert est.consecutive_fixes == 0


def test_sustained_rejection_triggers_a_reset_instead_of_wedging_shut():
    """The gate must not be able to lock the filter out of its own data."""
    noise = EstimatorNoise(reset_after_rejections=5)
    est = DockEstimator(noise, pose=Pose2D(-1.0, 0.0, 0.0))
    for _ in range(20):
        est.update(obs(1.0, 0.0, 0.0))
    # The truth is now somewhere else entirely; every fix looks like an outlier.
    accepted = [est.update(obs(1.0, 0.40, 0.0)) for _ in range(5)]
    assert accepted[:4] == [False, False, False, False]
    assert accepted[4] is True
    assert est.resets == 1
    assert est.error.lateral == pytest.approx(0.40)
    assert est.consecutive_rejections == 0


def test_a_miss_breaks_the_consecutive_fix_run():
    est = DockEstimator()
    est.update(obs(1.0, 0.0, 0.0))
    assert est.consecutive_fixes == 1
    assert not est.update(None)
    assert est.consecutive_fixes == 0


def test_reset_forgets_the_state_and_the_history():
    est = DockEstimator()
    est.update(obs(1.0, 0.0, 0.0))
    est.predict(0.2, 0.0, 0.1)
    est.reset()
    assert est.steps_since_fix > 100
    assert est.consecutive_fixes == 0
    assert est.lateral_sigma > 1.0
