"""The fiducial model: visibility limits, noise growth, latency, dropouts, ambiguity."""

from __future__ import annotations

import math

import numpy as np
import pytest

from autodock.geometry import DockError
from autodock.perception import FiducialSensor, MarkerModel

MODEL = MarkerModel()


def sensor(seed: int = 0, **kwargs) -> FiducialSensor:
    model = MarkerModel(**kwargs) if kwargs else MODEL
    return FiducialSensor(model, np.random.default_rng(seed))


def test_marker_is_invisible_below_its_minimum_range():
    assert not MODEL.visible(DockError(MODEL.min_range - 0.01, 0.0, 0.0))
    assert MODEL.visible(DockError(MODEL.min_range + 0.01, 0.0, 0.0))


def test_marker_is_invisible_beyond_the_maximum_range():
    assert not MODEL.visible(DockError(MODEL.max_range + 0.01, 0.0, 0.0))
    assert MODEL.visible(DockError(MODEL.max_range - 0.01, 0.0, 0.0))


def test_field_of_view_is_enforced_on_the_bearing():
    # Robot 2 m out, pointed 50 degrees away from the dock: outside a 35 degree half-FOV.
    assert not MODEL.visible(DockError(2.0, 0.0, math.radians(50.0)))
    assert MODEL.visible(DockError(2.0, 0.0, math.radians(20.0)))


def test_steep_incidence_hides_the_marker():
    steep = DockError(1.0, 2.0, math.radians(60.0))
    assert abs(steep.incidence) > MODEL.max_incidence
    assert not MODEL.visible(steep)


def test_noise_grows_with_range():
    near = MODEL.sigmas(0.4)
    far = MODEL.sigmas(3.5)
    assert all(f > n for f, n in zip(far, near))
    # The floor is the sub-pixel corner limit and is non-zero even at zero range.
    assert all(s > 0.0 for s in MODEL.sigmas(0.0))


def test_dropout_probability_rises_at_both_range_limits():
    mid = MODEL.dropout_probability(DockError(2.0, 0.0, 0.0))
    near = MODEL.dropout_probability(DockError(MODEL.min_range + 0.005, 0.0, 0.0))
    far = MODEL.dropout_probability(DockError(MODEL.max_range - 0.005, 0.0, 0.0))
    assert mid == pytest.approx(MODEL.dropout_prob)
    assert near > mid
    assert far > mid


def test_observations_are_delayed_by_exactly_the_stated_number_of_steps():
    s = sensor(seed=1, latency_steps=3, dropout_prob=0.0, ambiguity_prob=0.0,
               range_sigma=(0.0, 0.0), lateral_sigma=(0.0, 0.0), yaw_sigma=(0.0, 0.0))
    truths = [DockError(2.0 - 0.1 * i, 0.0, 0.0) for i in range(8)]
    got = [s.observe(t) for t in truths]
    assert got[0] is None and got[1] is None and got[2] is None
    for i in range(3, 8):
        assert got[i] is not None
        assert got[i].s == pytest.approx(truths[i - 3].s)
        assert got[i].age_steps == 3


def test_noise_is_zero_mean_and_matches_the_declared_sigma():
    s = sensor(seed=7, dropout_prob=0.0, ambiguity_prob=0.0, latency_steps=0)
    truth = DockError(1.5, 0.0, 0.0)
    lat = [s.observe(truth).lateral for _ in range(4000)]
    expected = MODEL.sigmas(1.5)[1]
    assert abs(float(np.mean(lat))) < 0.1 * expected
    assert float(np.std(lat)) == pytest.approx(expected, rel=0.1)


def test_pose_ambiguity_mirrors_the_heading_about_the_viewing_ray():
    s = sensor(seed=3, dropout_prob=0.0, ambiguity_prob=1.0, latency_steps=0,
               range_sigma=(0.0, 0.0), lateral_sigma=(0.0, 0.0), yaw_sigma=(0.0, 0.0))
    truth = DockError(2.0, 0.5, math.radians(3.0))
    obs = s.observe(truth)
    assert obs is not None
    assert obs.yaw == pytest.approx(2.0 * truth.incidence - truth.yaw)
    # The flip is a large structured error, not extra noise.
    assert abs(obs.yaw - truth.yaw) > 4.0 * MODEL.sigmas(2.0)[2]
    assert s.frames_flipped == 1


def test_ambiguity_is_rarer_close_in_than_far_out():
    assert MODEL.ambiguity_probability(DockError(0.4, 0.0, 0.0)) < MODEL.ambiguity_probability(
        DockError(3.5, 0.0, 0.0)
    )


def test_dropouts_actually_drop_frames_and_are_counted():
    s = sensor(seed=11, dropout_prob=0.5, ambiguity_prob=0.0, latency_steps=0)
    truth = DockError(1.5, 0.0, 0.0)
    got = [s.observe(truth) for _ in range(600)]
    misses = sum(1 for g in got if g is None)
    assert 0.4 * 600 < misses < 0.6 * 600
    assert s.frames_dropped == misses


def test_same_seed_gives_the_same_stream():
    truth = DockError(1.2, 0.05, 0.02)
    a = [FiducialSensor(MODEL, np.random.default_rng(5)).observe(truth) for _ in range(1)]
    b = [FiducialSensor(MODEL, np.random.default_rng(5)).observe(truth) for _ in range(1)]
    assert (a[0] is None) == (b[0] is None)
