"""A fiducial pose sensor that gets worse as the robot closes in.

The optimistic model of marker-based docking is "the camera gives me the dock
pose, I drive to it". The reason docking is hard is that the sensor stops
working exactly where the accuracy requirement gets tight:

* **Minimum range.** A marker large enough to be detected at 3 m fills or
  overflows the frame at 0.3 m, and on most robots the camera is mounted above
  and behind the docking shoe, so the marker leaves the field of view before
  contact. Below ``min_range`` there are no observations at all. The last
  stretch is dead reckoning, and this model makes that explicit rather than
  letting a controller quietly keep reading a pose that would not exist.
* **Range-dependent noise.** Fiducial translation error grows roughly with
  range; the out-of-plane rotation is worse, because it is estimated from the
  small perspective difference between the near and far edges of a nearly
  fronto-parallel square. Yaw noise is modelled with a larger range slope than
  translation noise for that reason.
* **Incidence.** Seen from far off the axis the marker is foreshortened and its
  pose is ill-conditioned. Past ``max_incidence`` the detector is treated as
  not reporting.
* **Dropouts.** Motion blur, exposure, a partially occluded tag. Independent
  per frame, with the probability rising near the range limits.
* **Planar pose ambiguity.** The one that surprises people. A square marker
  viewed nearly head-on has two pose solutions that project to almost the same
  four corners, and the solver picks the wrong one whenever noise on the corner
  positions is comparable to the difference between them. The wrong solution is
  the true pose mirrored about the viewing ray, so the reported heading is out
  by roughly twice the incidence angle -- a large, structured, *confident*
  error rather than a bit of extra noise. It gets more likely as the marker
  gets smaller in the image, which means at long range, which is exactly where
  a docking approach starts. Averaging does not remove it. Rejecting it does,
  which is why :class:`autodock.estimator.DockEstimator` gates on innovation.
* **Latency.** Capture, transfer, detect and solve take longer than one control
  tick. An observation describes where the robot *was*, and the estimator has
  to account for that (see :mod:`autodock.estimator`).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Optional

import numpy as np

from .geometry import DockError

__all__ = ["MarkerModel", "MarkerObservation", "FiducialSensor"]


@dataclass(frozen=True)
class MarkerModel:
    """Parameters of the fiducial pose observation.

    Noise terms are ``sigma = a + b * range``; ``a`` is the floor set by corner
    localisation at the detector's sub-pixel limit, ``b`` the growth with range.
    """

    max_range: float = 4.0
    min_range: float = 0.30
    half_fov: float = math.radians(35.0)
    max_incidence: float = math.radians(60.0)
    range_sigma: tuple[float, float] = (0.003, 0.010)
    lateral_sigma: tuple[float, float] = (0.002, 0.008)
    yaw_sigma: tuple[float, float] = (math.radians(0.6), math.radians(2.0))
    dropout_prob: float = 0.06
    ambiguity_prob: float = 0.10
    near_band: float = 0.12
    far_band: float = 0.60
    latency_steps: int = 2

    def sigmas(self, rng_to_marker: float) -> tuple[float, float, float]:
        """Return ``(sigma_s, sigma_lateral, sigma_yaw)`` at a given range."""
        r = max(rng_to_marker, 0.0)
        return (
            self.range_sigma[0] + self.range_sigma[1] * r,
            self.lateral_sigma[0] + self.lateral_sigma[1] * r,
            self.yaw_sigma[0] + self.yaw_sigma[1] * r,
        )

    def visible(self, error: DockError) -> bool:
        """Geometric visibility, before dropouts are applied."""
        r = error.range_to_marker
        if r < self.min_range or r > self.max_range:
            return False
        if abs(error.bearing_to_marker) > self.half_fov:
            return False
        if abs(error.incidence) > self.max_incidence:
            return False
        return True

    def ambiguity_probability(self, error: DockError) -> float:
        """Chance the solver returns the mirrored pose, rising with range.

        Scaled with ``(range / max_range)^2`` because the two solutions
        separate in the image in proportion to the marker's apparent size, and
        apparent size falls off with range.
        """
        ratio = min(error.range_to_marker / max(self.max_range, 1e-6), 1.0)
        return self.ambiguity_prob * ratio * ratio

    def dropout_probability(self, error: DockError) -> float:
        """Per-frame miss probability, rising as the geometry degrades.

        Detections do not stop cleanly at ``min_range``; they get flaky first,
        as the tag starts to clip the frame edge and the corner refinement runs
        out of margin. ``near_band`` and ``far_band`` set how wide those flaky
        zones are.
        """
        r = error.range_to_marker
        near_edge = max(0.0, 1.0 - (r - self.min_range) / max(self.near_band, 1e-6))
        far_edge = max(0.0, 1.0 - (self.max_range - r) / max(self.far_band, 1e-6))
        edge = max(near_edge, far_edge)
        return min(0.85, self.dropout_prob + 0.35 * edge * edge)


@dataclass(frozen=True)
class MarkerObservation:
    """One fiducial pose fix, already resolved into dock-frame error terms.

    ``age_steps`` is the number of control ticks between the exposure and this
    observation being handed to the estimator.
    """

    s: float
    lateral: float
    yaw: float
    sigma_s: float
    sigma_lateral: float
    sigma_yaw: float
    age_steps: int

    def as_error(self) -> DockError:
        """The measured dock-frame error."""
        return DockError(self.s, self.lateral, self.yaw)


@dataclass
class FiducialSensor:
    """Stateful sensor: owns the latency queue and its own random stream."""

    model: MarkerModel = field(default_factory=MarkerModel)
    rng: np.random.Generator = field(default_factory=lambda: np.random.default_rng(0))
    _queue: Deque[DockError] = field(default_factory=deque, init=False, repr=False)
    frames_seen: int = field(default=0, init=False)
    frames_dropped: int = field(default=0, init=False)
    frames_flipped: int = field(default=0, init=False)

    def reset(self) -> None:
        """Clear the latency queue and the counters."""
        self._queue.clear()
        self.frames_seen = 0
        self.frames_dropped = 0
        self.frames_flipped = 0

    def observe(self, true_error: DockError) -> Optional[MarkerObservation]:
        """Push the current true pose in, pop a delayed and corrupted one out.

        Returns ``None`` when the marker is not observable this frame, which is
        the normal case below ``min_range``.
        """
        self._queue.append(true_error)
        if len(self._queue) <= self.model.latency_steps:
            return None
        delayed = self._queue.popleft()

        if not self.model.visible(delayed):
            return None
        if self.rng.random() < self.model.dropout_probability(delayed):
            self.frames_dropped += 1
            return None

        sig_s, sig_lat, sig_yaw = self.model.sigmas(delayed.range_to_marker)
        self.frames_seen += 1
        yaw = delayed.yaw
        if self.rng.random() < self.model.ambiguity_probability(delayed):
            yaw = 2.0 * delayed.incidence - delayed.yaw
            self.frames_flipped += 1
        return MarkerObservation(
            s=delayed.s + float(self.rng.normal(0.0, sig_s)),
            lateral=delayed.lateral + float(self.rng.normal(0.0, sig_lat)),
            yaw=yaw + float(self.rng.normal(0.0, sig_yaw)),
            sigma_s=sig_s,
            sigma_lateral=sig_lat,
            sigma_yaw=sig_yaw,
            age_steps=self.model.latency_steps,
        )
