"""Motion commands for each state: pure pursuit onto the dock axis, then a line law.

The controller is a function of the *state* and the *estimate*. It holds no
mode logic of its own -- if you find yourself wanting an ``if`` here that
decides what the robot is trying to do, it belongs in the transition table.

Three laws do all the work.

**Pure pursuit onto the dock axis** (``APPROACH``). The path is the dock's own
axis, extended backwards, and the goal is the standoff point on it. Chasing a
lookahead point on that line pulls the robot onto the axis *and* onto the
axis heading, which is the only way a differential drive can get rid of lateral
error: by trading it for path length. The lookahead shrinks as the goal nears,
so the last part of the approach tracks tightly rather than cutting the corner.

**Turn in place** (``ALIGN``). Free for a differential drive and exact. It
removes heading error and, importantly, removes nothing else -- which is why
``ALIGN`` cannot rescue a lateral offset and the state machine sends that case
to ``REPOSITION`` instead.

**Line following** (``FINAL_APPROACH``). Curvature ``-(k_y*y + k_yaw*theta)``
drives lateral offset and heading to zero *together*, with the gains chosen to
be critically damped over a chosen convergence length. A law that only nulls
lateral offset arrives at the dock pointing across it.

``COMMIT`` and ``BLIND_APPROACH`` need almost no controller at all, which is
the point. ``COMMIT`` holds the wheels still while the estimator averages; a
stationary robot is the only configuration in which a fiducial pose is free of
motion blur and latency error. ``BLIND_APPROACH`` commands a constant forward
speed and exactly zero angular rate, because below the marker's minimum range
there is nothing left to close the loop on. Whatever error exists when the
robot leaves ``COMMIT`` is the error it arrives with, plus drift.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .geometry import DockError, DockGeometry, wrap_angle
from .kinematics import DiffDriveLimits, clamp_twist
from .states import DockState, DockingPolicy

__all__ = ["ControllerGains", "DockingController", "BlindPlan"]


@dataclass(frozen=True)
class BlindPlan:
    """The blind segment, decided at the commit gate and then not revisited."""

    curvature: float
    distance: float
    predicted_lateral: float
    predicted_yaw: float
    predicted_swept: float


@dataclass(frozen=True)
class ControllerGains:
    """Speeds, lookaheads and gains.

    ``line_k_y`` and ``line_k_yaw`` are not tuned by hand. For the small-angle
    line model ``dy/ds = theta``, ``dtheta/ds = kappa``, choosing
    ``kappa = -(k_y*y + k_yaw*theta)`` gives ``lambda^2 + k_yaw*lambda + k_y``,
    so a critically damped response with a convergence length of ``1/lam``
    metres needs ``k_yaw = 2*lam`` and ``k_y = lam^2``. The defaults are
    ``lam = 8`` per metre, i.e. lateral error falls by ``e`` every 125 mm of
    travel, which is as fast as the wheel limits support at the final speed.
    """

    search_omega: float = 0.70
    approach_speed: float = 0.30
    approach_lookahead: float = 0.70
    min_lookahead: float = 0.22
    min_speed: float = 0.06
    spin_threshold: float = 1.20
    align_kp: float = 1.8
    align_max_omega: float = 0.8
    final_speed: float = 0.09
    blind_speed: float = 0.07
    line_k_y: float = 64.0
    line_k_yaw: float = 16.0
    fresh_fix_ticks: int = 4
    engage_speed: float = 0.030
    retreat_speed: float = 0.20
    reposition_speed: float = 0.16
    retry_speed_factor: float = 0.60
    retry_lookahead_factor: float = 0.75
    blind_max_curvature: float = 1.5
    blind_plan_samples: int = 161


@dataclass
class DockingController:
    """Stateless control laws plus the actuator clamp."""

    gains: ControllerGains = field(default_factory=ControllerGains)
    policy: DockingPolicy = field(default_factory=DockingPolicy)
    limits: DiffDriveLimits = field(default_factory=DiffDriveLimits)
    dock: DockGeometry = field(default_factory=DockGeometry)

    def command(
        self,
        state: DockState,
        est: DockError,
        *,
        steps_since_fix: int = 0,
        repositions: int = 0,
        search_sign: float = 1.0,
        blind_curvature: float = 0.0,
    ) -> tuple[float, float]:
        """Return the clamped body twist ``(v, omega)`` for this state."""
        if state is DockState.SEARCH:
            v, omega = 0.0, math.copysign(self.gains.search_omega, search_sign or 1.0)
        elif state is DockState.ACQUIRE:
            v, omega = 0.0, 0.0
        elif state is DockState.APPROACH:
            v, omega = self.pure_pursuit(est, repositions=repositions)
        elif state is DockState.ALIGN:
            v, omega = self.turn_in_place(est)
        elif state is DockState.REPOSITION:
            v, omega = -self.gains.reposition_speed, 0.0
        elif state is DockState.FINAL_APPROACH:
            v, omega = self.line_follow(est, steps_since_fix=steps_since_fix)
        elif state is DockState.COMMIT:
            v, omega = 0.0, 0.0
        elif state is DockState.BLIND_APPROACH:
            v = self.gains.blind_speed
            omega = v * blind_curvature
        elif state is DockState.ENGAGE:
            v, omega = self.gains.engage_speed, 0.0
        elif state is DockState.RETREAT:
            v, omega = -self.gains.retreat_speed, 0.0
        else:
            v, omega = 0.0, 0.0
        return clamp_twist(v, omega, self.limits)

    def pure_pursuit(self, est: DockError, *, repositions: int = 0) -> tuple[float, float]:
        """Chase a lookahead point on the dock axis, aiming at the standoff."""
        g = self.gains
        x, y, theta = -est.s, est.lateral, est.yaw
        goal_x = -self.policy.standoff
        to_goal = max(goal_x - x, 0.0)

        lookahead = g.approach_lookahead * (g.retry_lookahead_factor ** repositions)
        ld = min(max(0.6 * to_goal + 0.15, g.min_lookahead), lookahead)
        x_ahead = x + math.sqrt(max(ld * ld - y * y, 0.0)) if abs(y) < ld else x
        x_ahead = min(x_ahead, goal_x)

        dx, dy = x_ahead - x, -y
        alpha = wrap_angle(math.atan2(dy, dx) - theta)
        if abs(alpha) > g.spin_threshold:
            return 0.0, math.copysign(g.align_max_omega, alpha)

        reach = max(math.hypot(dx, dy), 1e-3)
        curvature = 2.0 * math.sin(alpha) / reach
        speed = g.approach_speed * (g.retry_speed_factor ** repositions)
        speed *= max(0.25, 1.0 - abs(alpha) / g.spin_threshold)
        speed = min(speed, g.min_speed + 0.9 * to_goal)
        return speed, speed * curvature

    def turn_in_place(self, est: DockError) -> tuple[float, float]:
        """Null heading error against the dock axis without translating."""
        omega = -self.gains.align_kp * est.yaw
        omega = max(-self.gains.align_max_omega, min(self.gains.align_max_omega, omega))
        return 0.0, omega

    def plan_blind_arc(self, est: DockError) -> BlindPlan:
        """Pick the single constant curvature for the blind segment.

        The blind segment has exactly one free parameter, and "drive straight"
        is only the right value for it when the robot is already perfectly on
        the axis. At the commit gate the robot knows -- as well as it will ever
        know -- that it is, say, 5 mm left of the axis and pointing 0.4 degrees
        right. A straight run carries both of those into the funnel. A gentle
        arc can null the lateral offset, at the price of arriving with more
        heading error, and the mechanical criterion says exactly how that trade
        should be made: minimise the swept half-width
        ``|lateral| + L*|sin(yaw)|`` at the contact plane.

        The prediction uses the small-angle model ``y(d) = y0 + yaw0*d +
        kappa*d^2/2``, ``yaw(d) = yaw0 + kappa*d``, which is accurate to better
        than a per cent for the couple of degrees the gate allows through. The
        search is a scan over the one parameter, which is cheap, runs once, and
        is easy to reason about -- and unlike a closed-form solution it stays
        correct when the curvature limit binds.

        The plan is also what the commit gate is checked against. The question
        that decides go/no-go is not "is the robot close to the axis" but "does
        the best blind segment available from here still land inside the
        mechanical tolerance, with enough margin left for the drift that will
        happen afterwards". Those are different questions, and the first one
        rejects plenty of states the second one accepts.

        This is still open loop. It corrects for what the robot knew at the
        gate, not for what happens afterwards, which is what the drift sweep in
        ``examples/sweep.py`` is about.
        """
        d = max(est.s, 1e-3)
        limit = self.gains.blind_max_curvature
        n = max(self.gains.blind_plan_samples, 3)
        best = BlindPlan(0.0, d, est.lateral, est.yaw, float("inf"))
        for i in range(n):
            kappa = -limit + 2.0 * limit * i / (n - 1)
            lateral = est.lateral + est.yaw * d + 0.5 * kappa * d * d
            yaw = est.yaw + kappa * d
            swept = abs(lateral) + self.dock.shoe_length * abs(math.sin(yaw))
            if swept < best.predicted_swept:
                best = BlindPlan(kappa, d, lateral, yaw, swept)
        return best

    def line_follow(self, est: DockError, *, steps_since_fix: int) -> tuple[float, float]:
        """Track the dock axis while the marker is fresh; go straight when it is not."""
        v = self.gains.final_speed
        if steps_since_fix > self.gains.fresh_fix_ticks:
            return v, 0.0
        curvature = -(self.gains.line_k_y * est.lateral + self.gains.line_k_yaw * est.yaw)
        return v, v * curvature
