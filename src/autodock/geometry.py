"""SE(2) geometry, the dock frame, and the mechanical engagement criterion.

Everything in this package works in one of three frames:

``world``
    An arbitrary fixed frame. Only the simulator and the plots use it.
``dock``
    Origin at the centre of the dock's contact plane, ``+x`` pointing along the
    direction the robot must travel to enter the dock, ``+y`` to the left of
    that direction. The robot is at negative ``x`` while it is still outside.
    Every guard and every controller in this package is written in this frame,
    because every quantity that matters mechanically is a dock-frame quantity.
``robot``
    Body frame, ``+x`` forward.

The dock-frame error triple used throughout is ``(s, lateral, yaw)``:

``s``
    Remaining distance along the dock axis, ``s = -x_dock``. Positive while the
    robot is still outside the dock, zero at the contact plane.
``lateral``
    Signed offset from the dock axis, ``lateral = y_dock``.
``yaw``
    Heading relative to the dock axis, zero when the robot points straight in.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = [
    "Pose2D",
    "DockError",
    "DockGeometry",
    "wrap_angle",
]


def wrap_angle(angle: float) -> float:
    """Wrap an angle to ``(-pi, pi]``."""
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


@dataclass(frozen=True)
class Pose2D:
    """A planar pose. Angles in radians, distances in metres."""

    x: float
    y: float
    theta: float

    def compose(self, other: "Pose2D") -> "Pose2D":
        """Return ``self * other``: ``other`` expressed in ``self``'s parent frame."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        return Pose2D(
            self.x + c * other.x - s * other.y,
            self.y + s * other.x + c * other.y,
            wrap_angle(self.theta + other.theta),
        )

    def inverse(self) -> "Pose2D":
        """Return the pose that undoes this one."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        return Pose2D(-(c * self.x + s * self.y), -(-s * self.x + c * self.y), wrap_angle(-self.theta))

    def relative_to(self, frame: "Pose2D") -> "Pose2D":
        """Express this pose in ``frame``."""
        return frame.inverse().compose(self)


@dataclass(frozen=True)
class DockError(object):
    """Robot pose relative to the dock, in the coordinates that matter."""

    s: float
    lateral: float
    yaw: float

    @property
    def range_to_marker(self) -> float:
        """Straight-line distance from the robot to the marker on the dock face."""
        return math.hypot(self.s, self.lateral)

    @property
    def bearing_to_marker(self) -> float:
        """Bearing of the dock marker in the robot body frame."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx = self.s * c - self.lateral * s
        dy = -self.s * s - self.lateral * c
        return math.atan2(dy, dx)

    @property
    def incidence(self) -> float:
        """Angle between the marker's outward normal and the line of sight.

        A fiducial seen at a steep incidence gives a badly conditioned pose:
        the out-of-plane rotation is the least observable degree of freedom.
        """
        return math.atan2(self.lateral, max(self.s, 1e-9))

    def to_pose(self) -> Pose2D:
        """Return the robot pose in the dock frame."""
        return Pose2D(-self.s, self.lateral, self.yaw)

    @staticmethod
    def from_pose(pose: Pose2D) -> "DockError":
        """Build an error triple from a robot pose already expressed in the dock frame."""
        return DockError(-pose.x, pose.y, wrap_angle(pose.theta))


@dataclass(frozen=True)
class DockGeometry:
    """Where the dock is and what it will physically accept.

    The tolerances are not free parameters: they describe a specific piece of
    metal. See ``docs/FAILURE_MODES.md`` for the derivation.

    Attributes
    ----------
    x, y, yaw:
        Pose of the dock in the world frame. ``yaw`` is the heading the robot
        must hold at contact.
    standoff:
        Distance along the axis at which the approach stops and alignment is
        checked. The final approach starts here.
    funnel_outer_half_width:
        Half-width of the dock's outer face. Beyond this the robot misses the
        dock body entirely and no bump is registered.
    lateral_tolerance:
        Half-width of the funnel throat minus half the docking shoe. A shoe
        entering inside this is guided onto the contacts; outside it, the shoe
        strikes the funnel lip.
    yaw_tolerance:
        Hard limit past which the shoe jams on the funnel wall instead of
        sliding along it.
    shoe_length:
        Length of the docking shoe that has to pass the funnel throat. A yawed
        shoe sweeps a wider corridor than a straight one, which is why lateral
        and heading tolerance trade against each other rather than being
        independent budgets.
    """

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    standoff: float = 0.60
    funnel_outer_half_width: float = 0.090
    lateral_tolerance: float = 0.025
    yaw_tolerance: float = math.radians(6.0)
    shoe_length: float = 0.120

    @property
    def pose(self) -> Pose2D:
        """Dock contact-plane pose in the world frame."""
        return Pose2D(self.x, self.y, self.yaw)

    def axis_unit(self) -> tuple[float, float]:
        """Unit vector along the approach direction, in world coordinates."""
        return math.cos(self.yaw), math.sin(self.yaw)

    def standoff_pose(self) -> Pose2D:
        """World pose of the standoff: on the dock axis, ``standoff`` metres out.

        The standoff is derived from the dock pose rather than chosen in the
        world frame, so the final approach is a straight line along the dock's
        own axis by construction.
        """
        ux, uy = self.axis_unit()
        return Pose2D(self.x - self.standoff * ux, self.y - self.standoff * uy, self.yaw)

    def error(self, pose: Pose2D) -> DockError:
        """Dock-frame error of a robot pose given in the world frame."""
        return DockError.from_pose(pose.relative_to(self.pose))

    def world_pose(self, error: DockError) -> Pose2D:
        """Inverse of :meth:`error`."""
        return self.pose.compose(error.to_pose())

    def swept_half_width(self, error: DockError) -> float:
        """Half-width of the corridor the shoe sweeps as it enters the throat.

        A shoe of length ``L`` held at heading ``yaw`` and offset ``lateral``
        needs ``|lateral| + L*|sin(yaw)|`` of clearance, not ``|lateral|``.
        """
        return abs(error.lateral) + self.shoe_length * abs(math.sin(error.yaw))

    def engagement_ok(self, error: DockError) -> bool:
        """True if a shoe arriving at this pose actually mates with the contacts."""
        return (
            abs(error.yaw) <= self.yaw_tolerance
            and self.swept_half_width(error) <= self.lateral_tolerance
        )

    def hits_dock_body(self, error: DockError) -> bool:
        """True if the robot touches the dock at all (bump switch, not charge)."""
        return abs(error.lateral) <= self.funnel_outer_half_width
