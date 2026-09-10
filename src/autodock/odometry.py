"""Drive-train asymmetry and the odometry that does not know about it.

The blind segment of a docking approach is dead reckoned, so its accuracy is
decided by how far the robot's *actual* motion departs from the motion its
encoders report. The dominant cause on a small indoor base is not encoder
resolution; it is that the two wheels are not identical. Tyre wear, tyre
pressure, an uneven load, one wheel slipping slightly more on the same carpet:
all of it shows up as a small multiplicative difference between the two
effective wheel radii.

That difference has a specific and unpleasant signature. A robot commanded to
drive dead straight instead travels a circular arc of curvature

``kappa = (g_r - g_l) / ((g_r + g_l) / 2) / track_width``

so its lateral departure after ``d`` metres of blind travel grows as
``kappa * d^2 / 2`` -- quadratic in the blind distance, which is why the length
of the blind segment matters far more than it looks like it should.

:class:`DriveTrain` applies the asymmetry to produce true motion.
:class:`WheelOdometry` integrates the same commands through the *nominal*
geometry, which is all a real robot can do. The gap between them is the drift.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .geometry import Pose2D
from .kinematics import DiffDriveLimits, integrate, wheels_to_body

__all__ = ["DriveTrainSpec", "DriveTrain", "WheelOdometry"]


@dataclass(frozen=True)
class DriveTrainSpec:
    """Per-episode drive-train imperfections, drawn once and then fixed.

    ``asymmetry_std`` is the standard deviation of the fractional difference
    between the two effective wheel radii. ``drift_scale`` multiplies it (and
    the gyro bias) so a whole family of robots, from well-calibrated to badly
    worn, can be swept with one knob.
    """

    asymmetry_std: float = 0.015
    scale_error_std: float = 0.010
    gyro_bias_std: float = 0.004
    slip_noise_std: float = 0.004
    drift_scale: float = 1.0


@dataclass
class DriveTrain:
    """Turns wheel-speed commands into the motion the robot actually makes."""

    spec: DriveTrainSpec = field(default_factory=DriveTrainSpec)
    limits: DiffDriveLimits = field(default_factory=DiffDriveLimits)
    rng: np.random.Generator = field(default_factory=lambda: np.random.default_rng(0))
    gain_left: float = field(default=1.0, init=False)
    gain_right: float = field(default=1.0, init=False)
    gyro_bias: float = field(default=0.0, init=False)

    def __post_init__(self) -> None:
        self.sample()

    def sample(self) -> None:
        """Draw a fresh set of drive-train imperfections for a new episode."""
        k = self.spec.drift_scale
        common = 1.0 + float(self.rng.normal(0.0, self.spec.scale_error_std * k))
        diff = float(self.rng.normal(0.0, self.spec.asymmetry_std * k))
        self.gain_left = common * (1.0 - 0.5 * diff)
        self.gain_right = common * (1.0 + 0.5 * diff)
        self.gyro_bias = float(self.rng.normal(0.0, self.spec.gyro_bias_std * k))

    @property
    def straight_line_curvature(self) -> float:
        """Curvature the robot follows when it is commanded to go straight, 1/m."""
        mean = 0.5 * (self.gain_left + self.gain_right)
        return (self.gain_right - self.gain_left) / mean / self.limits.track_width

    def step(self, pose: Pose2D, wheels: tuple[float, float], dt: float) -> tuple[Pose2D, float, float]:
        """Advance the true pose. Returns ``(pose, v_true, omega_true)``."""
        noise = self.spec.slip_noise_std * self.spec.drift_scale
        left = wheels[0] * self.gain_left * (1.0 + float(self.rng.normal(0.0, noise)))
        right = wheels[1] * self.gain_right * (1.0 + float(self.rng.normal(0.0, noise)))
        v, omega = wheels_to_body(left, right, self.limits)
        omega += self.gyro_bias
        return integrate(pose, v, omega, dt), v, omega


@dataclass
class WheelOdometry:
    """Dead reckoning from wheel commands through the nominal geometry.

    This is deliberately naive. It is what a robot without an external
    reference has, and its error is the thing the docking design has to be
    robust to.
    """

    limits: DiffDriveLimits = field(default_factory=DiffDriveLimits)
    pose: Pose2D = field(default_factory=lambda: Pose2D(0.0, 0.0, 0.0))
    distance: float = 0.0

    def reset(self, pose: Pose2D | None = None) -> None:
        """Reset the integrated pose and the odometer."""
        self.pose = pose if pose is not None else Pose2D(0.0, 0.0, 0.0)
        self.distance = 0.0

    def step(self, wheels: tuple[float, float], dt: float) -> tuple[float, float]:
        """Integrate one tick. Returns the believed ``(v, omega)``."""
        v, omega = wheels_to_body(wheels[0], wheels[1], self.limits)
        self.pose = integrate(self.pose, v, omega, dt)
        self.distance += abs(v) * dt
        return v, omega
