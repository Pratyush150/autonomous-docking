"""Differential-drive kinematics, wheel limits, and exact arc integration.

Two facts about a differential drive decide the whole design of a docking
controller, and both live in this module.

1.  **It is non-holonomic.** The wheel contact points cannot slide sideways, so
    the body-frame velocity has no ``y`` component at any instant. There is no
    command that moves the robot 10 mm to its left. Lateral error can only be
    traded against heading and distance travelled, which means it has to be
    removed *before* the last stretch, not during it.
2.  **The limits are on the wheels, not on the body.** ``(v, omega)`` is a
    convenient abstraction, but the actuators saturate in wheel speed and wheel
    acceleration. Clamping ``v`` and ``omega`` independently changes the
    curvature of the path -- exactly the quantity a docking approach cares
    about -- so :func:`clamp_twist` scales both together instead.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .geometry import Pose2D, wrap_angle

__all__ = ["DiffDriveLimits", "body_to_wheels", "wheels_to_body", "clamp_twist", "rate_limit_wheels", "integrate"]


@dataclass(frozen=True)
class DiffDriveLimits:
    """Actuator envelope of a small indoor differential-drive base."""

    wheel_radius: float = 0.050
    track_width: float = 0.300
    max_wheel_speed: float = 12.0
    max_wheel_accel: float = 30.0

    @property
    def max_linear(self) -> float:
        """Fastest straight-line speed, in m/s."""
        return self.wheel_radius * self.max_wheel_speed

    @property
    def max_angular(self) -> float:
        """Fastest spin-in-place rate, in rad/s."""
        return 2.0 * self.max_linear / self.track_width

    @property
    def max_linear_accel(self) -> float:
        """Straight-line acceleration limit implied by the wheel limit, m/s^2."""
        return self.wheel_radius * self.max_wheel_accel


def body_to_wheels(v: float, omega: float, limits: DiffDriveLimits) -> tuple[float, float]:
    """Convert a body twist to left/right wheel angular speeds in rad/s."""
    half_track = 0.5 * limits.track_width
    left = (v - omega * half_track) / limits.wheel_radius
    right = (v + omega * half_track) / limits.wheel_radius
    return left, right


def wheels_to_body(left: float, right: float, limits: DiffDriveLimits) -> tuple[float, float]:
    """Convert left/right wheel angular speeds in rad/s to a body twist."""
    v = 0.5 * limits.wheel_radius * (left + right)
    omega = limits.wheel_radius * (right - left) / limits.track_width
    return v, omega


def clamp_twist(v: float, omega: float, limits: DiffDriveLimits) -> tuple[float, float]:
    """Scale ``(v, omega)`` uniformly until both wheels are inside their limit.

    Uniform scaling keeps ``omega / v`` -- the path curvature -- unchanged. The
    robot then follows the same geometric path more slowly, instead of
    following a different, straighter path at the requested speed. During a
    docking approach the path is the thing you are trying to control, so this
    is the correct trade.

    A pure spin (``v == 0``) is clamped on ``omega`` alone.
    """
    left, right = body_to_wheels(v, omega, limits)
    peak = max(abs(left), abs(right))
    if peak <= limits.max_wheel_speed or peak == 0.0:
        return v, omega
    scale = limits.max_wheel_speed / peak
    return v * scale, omega * scale


def rate_limit_wheels(
    previous: tuple[float, float],
    command: tuple[float, float],
    dt: float,
    limits: DiffDriveLimits,
) -> tuple[float, float]:
    """Apply the wheel acceleration limit to a wheel-speed command.

    Returned per wheel, so a slew limit on one wheel does not silently rotate
    the robot: callers convert back with :func:`wheels_to_body` and use the
    twist that the base can actually produce this tick.
    """
    step = limits.max_wheel_accel * dt
    out = []
    for prev, cmd in zip(previous, command):
        delta = max(-step, min(step, cmd - prev))
        out.append(prev + delta)
    return out[0], out[1]


def integrate(pose: Pose2D, v: float, omega: float, dt: float) -> Pose2D:
    """Advance a pose by a constant twist held over ``dt``, exactly.

    Constant ``(v, omega)`` traces a circular arc of radius ``v / omega``.
    Euler integration replaces that arc with its tangent and accumulates a
    bias that always points to the outside of the turn -- small per step,
    systematic over a whole approach. The closed form costs nothing.
    """
    if abs(omega) < 1e-9:
        return Pose2D(pose.x + v * dt * math.cos(pose.theta), pose.y + v * dt * math.sin(pose.theta), pose.theta)
    radius = v / omega
    theta_next = pose.theta + omega * dt
    return Pose2D(
        pose.x + radius * (math.sin(theta_next) - math.sin(pose.theta)),
        pose.y - radius * (math.cos(theta_next) - math.cos(pose.theta)),
        wrap_angle(theta_next),
    )
