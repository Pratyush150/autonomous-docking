"""ROS 2 wrapper around :class:`autodock.runtime.DockingRuntime`.

**This node is written but has not been run.** ROS 2 is not installed in the
environment this repository was developed and measured in, so every number in
the README comes from the pure-Python simulator and none of it comes from here.
The import of ``rclpy`` is guarded so that the rest of the package, and the
whole test suite, work without ROS present; what is tested here is that the
module imports, that the guard reports honestly, and that the node cannot be
constructed without ROS.

The node is thin on purpose. It converts messages into the three inputs the
runtime takes -- a fiducial observation, a bump reading, a charge reading --
and converts the returned twist into ``geometry_msgs/Twist``. No docking logic
lives in this file, because logic that lives in a ROS callback can only be
tested by bringing up ROS.

Expected wiring::

    /dock/marker_pose   geometry_msgs/PoseStamped   fiducial pose of the dock
                                                    marker in the camera frame
    /dock/bump          std_msgs/Bool               contact switch
    /dock/charging      std_msgs/Bool               charger present
    /cmd_vel            geometry_msgs/Twist         out
    /dock/state         std_msgs/String             out, current behaviour state
    /odom               nav_msgs/Odometry           wheel odometry twist

The marker pose is expected in the camera frame with the marker's own frame
convention; :func:`observation_from_marker_pose` converts it into the dock-frame
error triple the runtime works in, and is pure Python so it can be unit tested.
"""

from __future__ import annotations

import math
from typing import Optional

from .geometry import wrap_angle
from .perception import MarkerObservation
from .runtime import DockingRuntime

try:  # pragma: no cover - exercised only where ROS 2 is installed
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import Bool, String

    RCLPY_AVAILABLE = True
except ImportError:  # pragma: no cover - the path taken everywhere in this repo
    rclpy = None
    Node = object
    RCLPY_AVAILABLE = False

__all__ = ["RCLPY_AVAILABLE", "observation_from_marker_pose", "DockingNode", "main"]


def observation_from_marker_pose(
    x: float,
    y: float,
    yaw: float,
    *,
    sigma_s: float,
    sigma_lateral: float,
    sigma_yaw: float,
    age_steps: int = 0,
) -> MarkerObservation:
    """Convert a marker pose in the robot's camera frame into a dock-frame error.

    ``(x, y, yaw)`` is the marker's pose as the detector reports it: ``x``
    forward from the camera, ``y`` to the left, ``yaw`` the rotation of the
    marker plane about the vertical. The robot's pose relative to the dock is
    the inverse of that, expressed in the dock's frame:

    * remaining distance along the dock axis ``s = x*cos(yaw) + y*sin(yaw)``
    * lateral offset ``lateral = -(-x*sin(yaw) + y*cos(yaw))``
    * heading error ``yaw_error = -yaw``

    Pure arithmetic, no ROS types, so it can be tested without a ROS install.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    along = x * c + y * s
    across = -x * s + y * c
    return MarkerObservation(
        s=along,
        lateral=-across,
        yaw=wrap_angle(-yaw),
        sigma_s=sigma_s,
        sigma_lateral=sigma_lateral,
        sigma_yaw=sigma_yaw,
        age_steps=age_steps,
    )


class DockingNode(Node):  # pragma: no cover - requires a ROS 2 install
    """Runs the docking runtime at a fixed rate off ROS topics."""

    def __init__(self, rate_hz: float = 25.0) -> None:
        if not RCLPY_AVAILABLE:
            raise RuntimeError(
                "rclpy is not available; autodock.ros_node.DockingNode needs a ROS 2 install. "
                "The docking logic itself is in autodock.runtime and needs nothing."
            )
        super().__init__("autodock")
        self.runtime = DockingRuntime()
        self.dt = 1.0 / rate_hz
        self._observation: Optional[MarkerObservation] = None
        self._bump = False
        self._charging = False
        self._odom = (0.0, 0.0)

        self.declare_parameter("marker_sigma_lateral", 0.006)
        self.declare_parameter("marker_sigma_range", 0.006)
        self.declare_parameter("marker_sigma_yaw", math.radians(1.5))

        self.create_subscription(PoseStamped, "dock/marker_pose", self._on_marker, 10)
        self.create_subscription(Bool, "dock/bump", self._on_bump, 10)
        self.create_subscription(Bool, "dock/charging", self._on_charging, 10)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._state_pub = self.create_publisher(String, "dock/state", 10)
        self.create_timer(self.dt, self._on_tick)

    def _on_marker(self, msg) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._observation = observation_from_marker_pose(
            msg.pose.position.x,
            msg.pose.position.y,
            yaw,
            sigma_s=self.get_parameter("marker_sigma_range").value,
            sigma_lateral=self.get_parameter("marker_sigma_lateral").value,
            sigma_yaw=self.get_parameter("marker_sigma_yaw").value,
        )

    def _on_bump(self, msg) -> None:
        self._bump = bool(msg.data)

    def _on_charging(self, msg) -> None:
        self._charging = bool(msg.data)

    def _on_odom(self, msg) -> None:
        self._odom = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _on_tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        command = self.runtime.step(
            now=now,
            dt=self.dt,
            observation=self._observation,
            bump=self._bump,
            charging=self._charging,
        )
        self._observation = None  # one fix is used once

        twist = Twist()
        twist.linear.x = command.v
        twist.angular.z = command.omega
        self._cmd_pub.publish(twist)

        state = String()
        state.data = command.state.value
        self._state_pub.publish(state)

        if command.change is not None:
            self.get_logger().info(
                f"{command.change.source.value} -[{command.change.guard}]-> {command.change.target.value}"
            )

        self.runtime.feed_odometry(self._odom[0], self._odom[1], self.dt)


def main(args=None) -> None:  # pragma: no cover - requires a ROS 2 install
    """Entry point for ``ros2 run``."""
    if not RCLPY_AVAILABLE:
        raise RuntimeError("rclpy is not available")
    rclpy.init(args=args)
    node = DockingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
