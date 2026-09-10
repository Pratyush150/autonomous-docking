"""The online docking core: one tick in, one wheel command out.

Everything above this module is either a simulator or a ROS node. This is the
part that would run on the robot, and it is deliberately free of both: it takes
a fiducial observation (or ``None``), a bump reading and a charge reading, and
returns a body twist. It never touches a clock, a topic or a sensor driver.

Keeping it separate is not tidiness. It is what makes the state machine
testable at all: the simulator in :mod:`autodock.sim` and a ROS node in
:mod:`autodock.ros_node` drive exactly the same object, so a behaviour proved
in simulation is the behaviour that ships, and a bug found on the robot can be
reproduced by replaying the same observation sequence offline.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from .controller import BlindPlan, ControllerGains, DockingController
from .estimator import DockEstimator, EstimatorNoise
from .geometry import DockError, DockGeometry, Pose2D
from .kinematics import DiffDriveLimits
from .machine import DockingStateMachine, StateChange
from .perception import MarkerObservation
from .states import DockingPolicy, DockState, GuardContext

__all__ = ["DockingRuntime", "RuntimeCommand"]


@dataclass(frozen=True)
class RuntimeCommand:
    """What the runtime wants the base to do this tick, and why."""

    v: float
    omega: float
    state: DockState
    change: Optional[StateChange]
    context: GuardContext


@dataclass
class DockingRuntime:
    """State machine, estimator and controller wired together for online use."""

    policy: DockingPolicy = field(default_factory=DockingPolicy)
    gains: ControllerGains = field(default_factory=ControllerGains)
    limits: DiffDriveLimits = field(default_factory=DiffDriveLimits)
    dock: DockGeometry = field(default_factory=DockGeometry)
    estimator_noise: EstimatorNoise = field(default_factory=EstimatorNoise)
    estimator: DockEstimator = field(init=False)
    machine: DockingStateMachine = field(init=False)
    controller: DockingController = field(init=False)
    odom_distance: float = field(default=0.0, init=False)
    fixes_in_state: int = field(default=0, init=False)
    travel_budget: float = field(default=0.0, init=False)
    search_sign: float = field(default=1.0, init=False)
    plan: BlindPlan = field(init=False)

    def __post_init__(self) -> None:
        self.estimator = DockEstimator(self.estimator_noise, pose=Pose2D(-3.0, 0.0, 0.0))
        self.machine = DockingStateMachine(self.policy)
        self.controller = DockingController(self.gains, self.policy, self.limits, self.dock)
        self.plan = self.controller.plan_blind_arc(DockError(self.policy.commit_range, 0.0, 0.0))

    @property
    def state(self) -> DockState:
        """Current behaviour state."""
        return self.machine.state

    @property
    def estimate(self) -> DockError:
        """Current dock-frame estimate."""
        return self.estimator.error

    def step(
        self,
        *,
        now: float,
        dt: float,
        observation: Optional[MarkerObservation] = None,
        bump: bool = False,
        charging: bool = False,
    ) -> RuntimeCommand:
        """Fold in this tick's sensing, evaluate the table, return a twist."""
        if self.estimator.update(observation):
            self.fixes_in_state += 1
            accepted = True
        else:
            accepted = False

        est = self.estimator.error
        if self.machine.state in (DockState.FINAL_APPROACH, DockState.COMMIT):
            self.plan = self.controller.plan_blind_arc(est)

        ctx = GuardContext(
            est_s=est.s,
            est_lateral=est.lateral,
            est_yaw=est.yaw,
            lateral_sigma=self.estimator.lateral_sigma,
            yaw_sigma=self.estimator.yaw_sigma,
            steps_since_fix=self.estimator.steps_since_fix,
            consecutive_fixes=self.estimator.consecutive_fixes,
            fixes_in_state=self.fixes_in_state,
            has_fix=self.estimator.has_fix,
            marker_detected=accepted,
            dt=dt,
            total_time=now,
            odom_distance=self.odom_distance,
            travel_budget=self.travel_budget,
            planned_swept=self.plan.predicted_swept,
            planned_yaw=self.plan.predicted_yaw,
            bump=bump,
            charge_present=charging,
        )

        change = self.machine.step(ctx)
        if change is not None:
            self.fixes_in_state = 0
            if change.target is DockState.BLIND_APPROACH:
                self.travel_budget = max(est.s, 0.0) + self.policy.overtravel
            if change.target is DockState.SEARCH:
                self.search_sign = (
                    math.copysign(1.0, est.bearing_to_marker) if self.estimator.has_fix else 1.0
                )

        if self.machine.state.terminal:
            return RuntimeCommand(0.0, 0.0, self.machine.state, change, ctx)

        v, omega = self.controller.command(
            self.machine.state,
            est,
            steps_since_fix=self.estimator.steps_since_fix,
            repositions=self.machine.repositions,
            search_sign=self.search_sign,
            blind_curvature=self.plan.curvature,
        )
        return RuntimeCommand(v, omega, self.machine.state, change, ctx)

    def feed_odometry(self, v: float, omega: float, dt: float) -> None:
        """Report the twist the base actually executed, from its own encoders."""
        self.estimator.predict(v, omega, dt)
        self.odom_distance += abs(v) * dt
