"""Dock-relative pose estimate: odometry prediction plus delayed fiducial fixes.

The estimator holds the robot's pose *in the dock frame*, because that is the
only frame in which the docking problem is stated. It has two jobs that a
naive "just use the latest detection" implementation gets wrong.

**Latency.** A fiducial pose describes the robot's position two or three
control ticks ago. Dropping it straight into the state pulls the estimate
backwards by ``v * latency`` every time a detection arrives, which at 0.3 m/s
and 130 ms is 40 mm of periodic step exactly where the controller is trying to
settle. Instead the fix is applied at its own timestamp and then re-propagated
forward through the odometry deltas that have accumulated since.

**Outliers.** A fiducial pose that has flipped to the mirrored solution is not
noisy, it is wrong, and it is wrong by more than the noise model allows. Each
component of the innovation is compared against ``gate_sigmas`` times its own
predicted standard deviation and the whole fix is dropped if any component
fails. The gate is a trade, not a free win: too tight and the estimator starves
itself of the fixes it needs, which shows up as the ``fix_stale`` guard firing.
``rejected_fixes`` is exposed so the trade can be measured rather than guessed.

**Divergence.** A gate and a covariance floor together can wedge a filter shut.
If one bad fix is accepted while the covariance is still wide -- a flipped
fiducial pose during acquisition is the usual culprit -- the state is now
wrong, the covariance shrinks to its floor anyway, and every subsequent
*correct* detection is more than ``gate_sigmas`` away and gets thrown out. The
filter sits there quietly rejecting the only data that could fix it, and
because it is not receiving fixes the behaviour above it flips between
searching and acquiring forever. The cure is a watchdog:
``reset_after_rejections`` consecutive rejections are taken as evidence that
the state, not the data, is wrong, and the filter is re-initialised on the
observation. Any filter with a gate needs one of these.

**Blind operation.** Below the marker's minimum range there are no fixes at
all. The estimator keeps predicting, its variance keeps growing, and
``steps_since_fix`` keeps counting. Guards elsewhere in the package use that
counter rather than assuming a fix is always available.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Optional

from .geometry import DockError, Pose2D, wrap_angle
from .kinematics import integrate
from .perception import MarkerObservation

__all__ = ["DockEstimator", "EstimatorNoise"]


@dataclass(frozen=True)
class EstimatorNoise:
    """Process noise, expressed per metre driven and per radian turned.

    ``yaw_per_metre`` is not a tuning knob. Wheel asymmetry of standard
    deviation ``sigma_g`` produces a heading error growing at
    ``sigma_g / track_width`` radians per metre driven, which for the drive
    train modelled in :mod:`autodock.odometry` is about 3 degrees per metre.
    Setting it lower makes the filter over-confident: its covariance collapses,
    it stops believing new detections, and it carries an early error straight
    into the dock while reporting a small sigma. That failure is silent, and it
    is the reason the commit gate checks sigma as well as the offsets.

    ``pos_floor`` and ``yaw_floor`` are a covariance floor, and they exist for
    a specific reason. The dominant odometry error is a *bias*: this robot's
    left wheel is 1.2 % larger than its right wheel for the whole run, and its
    rate gyro has a fixed offset. A random-walk process model cannot represent
    a bias, so a filter given only white process noise will keep shrinking its
    covariance below the error it actually has, stop weighting detections, and
    lag the bias. Flooring the covariance turns the filter into a
    fading-memory one: it never becomes more certain than the level at which
    the model itself is wrong. Without the floor the estimator in this package
    reported 0.15 degrees of heading sigma while carrying 1.8 degrees of
    heading error, and the commit gate happily believed it.
    """

    pos_per_metre: float = 0.030
    yaw_per_metre: float = math.radians(3.0)
    yaw_per_radian: float = math.radians(3.0)
    pos_floor: float = 0.003
    yaw_floor: float = math.radians(0.6)
    gate_sigmas: float = 3.5
    reset_after_rejections: int = 5


@dataclass
class DockEstimator:
    """Kalman-lite estimate of the robot pose in the dock frame.

    The covariance is kept diagonal. The three axes are not independent in
    reality -- a yaw error turns into a lateral error as soon as the robot
    moves -- but the coupling is handled by re-propagating the mean through the
    exact arc model, and a diagonal covariance is enough for its actual job
    here, which is deciding how much to trust a fix and telling the guards when
    the estimate has gone stale.
    """

    noise: EstimatorNoise = field(default_factory=EstimatorNoise)
    pose: Pose2D = field(default_factory=lambda: Pose2D(-3.0, 0.0, 0.0))
    var: tuple[float, float, float] = (4.0, 4.0, math.radians(90.0) ** 2)
    steps_since_fix: int = 10_000
    consecutive_fixes: int = 0
    total_fixes: int = 0
    rejected_fixes: int = 0
    resets: int = 0
    consecutive_rejections: int = 0
    _history: Deque[tuple[float, float, float]] = field(default_factory=lambda: deque(maxlen=16), init=False, repr=False)

    def reset(self, pose: Pose2D | None = None) -> None:
        """Forget everything; used on retreat before a fresh acquisition."""
        if pose is not None:
            self.pose = pose
        self.var = (4.0, 4.0, math.radians(90.0) ** 2)
        self.steps_since_fix = 10_000
        self.consecutive_fixes = 0
        self.consecutive_rejections = 0
        self._history.clear()

    @property
    def error(self) -> DockError:
        """Current estimated dock-frame error."""
        return DockError.from_pose(self.pose)

    @property
    def has_fix(self) -> bool:
        """True once at least one fiducial fix has been folded in."""
        return self.total_fixes > 0

    @property
    def lateral_sigma(self) -> float:
        """One-sigma lateral uncertainty, metres."""
        return math.sqrt(self.var[1])

    @property
    def yaw_sigma(self) -> float:
        """One-sigma heading uncertainty, radians."""
        return math.sqrt(self.var[2])

    def _floored(self, var: tuple[float, float, float]) -> tuple[float, float, float]:
        """Apply the covariance floor. See :class:`EstimatorNoise`."""
        return (
            max(var[0], self.noise.pos_floor ** 2),
            max(var[1], self.noise.pos_floor ** 2),
            max(var[2], self.noise.yaw_floor ** 2),
        )

    def predict(self, v: float, omega: float, dt: float) -> None:
        """Propagate with an odometry twist and grow the covariance."""
        self.pose = integrate(self.pose, v, omega, dt)
        self._history.append((v, omega, dt))
        self.steps_since_fix += 1
        ds = abs(v) * dt
        dth = abs(omega) * dt
        pos_q = (self.noise.pos_per_metre * ds) ** 2
        yaw_q = (self.noise.yaw_per_metre * ds) ** 2 + (self.noise.yaw_per_radian * dth) ** 2
        self.var = self._floored((self.var[0] + pos_q, self.var[1] + pos_q, self.var[2] + yaw_q))

    def _reinitialise(self, measured: Pose2D, r: tuple[float, float, float]) -> None:
        """Throw the state away and restart on this observation."""
        self.pose = measured
        self.var = self._floored((4.0 * r[0], 4.0 * r[1], 4.0 * r[2]))
        self.steps_since_fix = 0
        self.consecutive_fixes = 1
        self.consecutive_rejections = 0
        self.total_fixes += 1
        self.resets += 1

    def update(self, obs: Optional[MarkerObservation]) -> bool:
        """Fold in a fiducial fix, compensating for its latency.

        Returns True if the fix was used.
        """
        if obs is None:
            self.consecutive_fixes = 0
            return False

        measured = Pose2D(-obs.s, obs.lateral, obs.yaw)
        replay = list(self._history)[-obs.age_steps :] if obs.age_steps > 0 else []
        for v, omega, dt in replay:
            measured = integrate(measured, v, omega, dt)

        r = (obs.sigma_s ** 2, obs.sigma_lateral ** 2, obs.sigma_yaw ** 2)
        innovation = (
            measured.x - self.pose.x,
            measured.y - self.pose.y,
            wrap_angle(measured.theta - self.pose.theta),
        )
        gate = self.noise.gate_sigmas
        if any(abs(innovation[i]) > gate * math.sqrt(self.var[i] + r[i]) for i in range(3)):
            self.rejected_fixes += 1
            self.consecutive_fixes = 0
            self.consecutive_rejections += 1
            if self.consecutive_rejections < self.noise.reset_after_rejections:
                return False
            self._reinitialise(measured, r)
            return True
        self.consecutive_rejections = 0
        gains = tuple(self.var[i] / (self.var[i] + r[i]) for i in range(3))
        self.pose = Pose2D(
            self.pose.x + gains[0] * innovation[0],
            self.pose.y + gains[1] * innovation[1],
            wrap_angle(self.pose.theta + gains[2] * innovation[2]),
        )
        self.var = self._floored(tuple((1.0 - gains[i]) * self.var[i] for i in range(3)))
        self.steps_since_fix = 0
        self.consecutive_fixes += 1
        self.total_fixes += 1
        return True
