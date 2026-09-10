"""Closed-loop docking episode: truth, sensor, estimator, machine, controller.

One tick, in order:

1. compute the true dock-frame error from the true pose (the robot never sees this);
2. take a fiducial observation, which may be absent, late and noisy;
3. fold it into the estimator, re-propagated to now;
4. build the guard context and evaluate the transition table;
5. ask the controller for a twist, clamp it to the wheel envelope, rate limit
   the wheels, and convert back to the twist the base can actually produce;
6. push that through the imperfect drive train to get the new true pose;
7. integrate the *commanded* wheels through nominal geometry to get odometry,
   and predict the estimator forward with it;
8. resolve contact against the dock.

Steps 6 and 7 are the whole story: the robot acts on 7 and is graded on 6.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import numpy as np

from .controller import ControllerGains
from .estimator import EstimatorNoise
from .geometry import DockError, DockGeometry, Pose2D, wrap_angle
from .kinematics import DiffDriveLimits, body_to_wheels, rate_limit_wheels
from .odometry import DriveTrain, DriveTrainSpec, WheelOdometry
from .perception import FiducialSensor, MarkerModel
from .runtime import DockingRuntime
from .states import DockingPolicy, DockState

__all__ = ["StartDistribution", "EpisodeConfig", "EpisodeResult", "run_episode"]

_STATE_INDEX = {state: i for i, state in enumerate(DockState)}


@dataclass(frozen=True)
class StartDistribution:
    """Where an episode starts, relative to the dock.

    ``misalignment`` scales the lateral spread, and is the sweep axis for "how
    badly placed was the robot when docking was commanded" -- in practice set
    by whatever navigation stack drops the robot near the dock. Heading is not
    scaled because it is already uniform over the full circle: a robot handed a
    "go dock" command has whatever heading its last task left it with,
    frequently facing away from the dock, which is what ``SEARCH`` is for.
    """

    s_range: tuple[float, float] = (1.6, 3.0)
    lateral_span: float = 1.10
    yaw_span: float = math.radians(180.0)
    misalignment: float = 1.0

    def sample(self, rng: np.random.Generator) -> DockError:
        """Draw a starting dock-frame error."""
        s = float(rng.uniform(*self.s_range))
        lateral = float(rng.uniform(-1.0, 1.0)) * self.lateral_span * self.misalignment
        yaw = float(rng.uniform(-1.0, 1.0)) * self.yaw_span
        return DockError(s, lateral, wrap_angle(yaw))


@dataclass(frozen=True)
class EpisodeConfig:
    """Everything needed to run one docking episode."""

    dock: DockGeometry = field(default_factory=DockGeometry)
    limits: DiffDriveLimits = field(default_factory=DiffDriveLimits)
    marker: MarkerModel = field(default_factory=MarkerModel)
    drive: DriveTrainSpec = field(default_factory=DriveTrainSpec)
    policy: DockingPolicy = field(default_factory=DockingPolicy)
    gains: ControllerGains = field(default_factory=ControllerGains)
    start: StartDistribution = field(default_factory=StartDistribution)
    estimator_noise: EstimatorNoise = field(default_factory=EstimatorNoise)
    dt: float = 0.04
    record: bool = False


@dataclass
class EpisodeResult:
    """Outcome of one episode. Every field is measured, none is assumed."""

    seed: int
    success: bool
    outcome: str
    duration: float
    attempts: int
    retries: int
    repositions: int
    start_error: DockError
    final_error: DockError
    contact_error: Optional[DockError]
    longitudinal_offset: float
    dead_reckon_error: float
    blind_distance: float
    straight_line_curvature: float
    frames_seen: int
    frames_dropped: int
    state_history: List[tuple]
    trajectory: Optional[np.ndarray] = None

    @property
    def lateral_offset(self) -> float:
        """Signed lateral offset where the final approach ended, metres."""
        err = self.contact_error if self.contact_error is not None else self.final_error
        return err.lateral

    @property
    def yaw_offset(self) -> float:
        """Signed heading error where the final approach ended, radians."""
        err = self.contact_error if self.contact_error is not None else self.final_error
        return err.yaw


def _lerp_pose(a: Pose2D, b: Pose2D, t: float) -> Pose2D:
    return Pose2D(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), wrap_angle(a.theta + t * wrap_angle(b.theta - a.theta)))


def run_episode(config: EpisodeConfig, seed: int, observer: Optional[Callable] = None) -> EpisodeResult:
    """Run one docking episode to a terminal state and report what happened.

    ``observer(change, ctx)`` is called for every transition that fires, with
    the guard context that fired it. It exists so that tests can assert
    properties *at the moment a transition happens* -- for instance that the
    robot never enters ``BLIND_APPROACH`` from a state its own commit gate
    should have rejected -- rather than inferring them from the final result.
    """
    streams = np.random.SeedSequence(seed).spawn(3)
    rng_start = np.random.default_rng(streams[0])
    sensor = FiducialSensor(config.marker, np.random.default_rng(streams[1]))
    drive = DriveTrain(config.drive, config.limits, np.random.default_rng(streams[2]))

    start_error = config.start.sample(rng_start)
    pose = config.dock.world_pose(start_error)

    odom = WheelOdometry(config.limits)
    runtime = DockingRuntime(
        policy=config.policy,
        gains=config.gains,
        limits=config.limits,
        dock=config.dock,
        estimator_noise=config.estimator_noise,
    )

    wheels_prev = (0.0, 0.0)
    bumped = False
    live_contact: Optional[DockError] = None
    contact_error: Optional[DockError] = None
    distance_at_last_fix = 0.0
    blind_distance = 0.0
    dead_reckon_error = 0.0
    longitudinal_offset = 0.0
    trajectory: List[tuple] = []

    t = 0.0
    max_steps = int(config.policy.episode_timeout / config.dt) + 4
    for _ in range(max_steps):
        true_error = config.dock.error(pose)
        obs = sensor.observe(true_error)
        charging = bumped and live_contact is not None and config.dock.engagement_ok(live_contact)

        previous_state = runtime.state
        command = runtime.step(now=t, dt=config.dt, observation=obs, bump=bumped, charging=charging)
        est = runtime.estimate
        if command.context.marker_detected:
            distance_at_last_fix = odom.distance

        change = command.change
        if change is not None:
            if observer is not None:
                observer(change, command.context)
            if change.target is DockState.BLIND_APPROACH:
                distance_at_last_fix = odom.distance
            if previous_state is DockState.BLIND_APPROACH:
                blind_distance = odom.distance - distance_at_last_fix
                longitudinal_offset = true_error.s
                dead_reckon_error = est.s - true_error.s
            if change.target in (DockState.RETREAT, DockState.REPOSITION):
                bumped = False
                live_contact = None

        if config.record:
            trajectory.append(
                (
                    t,
                    pose.x,
                    pose.y,
                    pose.theta,
                    _STATE_INDEX[runtime.state],
                    true_error.s,
                    true_error.lateral,
                    true_error.yaw,
                    est.lateral,
                    est.yaw,
                    1.0 if obs is not None else 0.0,
                )
            )

        if runtime.state.terminal:
            break

        v_cmd, omega_cmd = command.v, command.omega
        if bumped:
            v_cmd = min(v_cmd, 0.0)

        wheels_cmd = body_to_wheels(v_cmd, omega_cmd, config.limits)
        wheels = rate_limit_wheels(wheels_prev, wheels_cmd, config.dt, config.limits)
        wheels_prev = wheels

        pose_prev = pose
        pose, _, _ = drive.step(pose, wheels, config.dt)
        v_odo, w_odo = odom.step(wheels, config.dt)
        runtime.feed_odometry(v_odo, w_odo, config.dt)

        if not bumped:
            s_prev = config.dock.error(pose_prev).s
            s_now = config.dock.error(pose).s
            if s_now <= 0.0 < s_prev:
                frac = s_prev / max(s_prev - s_now, 1e-12)
                touch_pose = _lerp_pose(pose_prev, pose, frac)
                touch_error = config.dock.error(touch_pose)
                if config.dock.hits_dock_body(touch_error):
                    bumped = True
                    live_contact = touch_error
                    contact_error = touch_error
                    pose = touch_pose

        t += config.dt

    machine = runtime.machine
    final_error = config.dock.error(pose)
    if runtime.state is DockState.BLIND_APPROACH:
        longitudinal_offset = final_error.s
        dead_reckon_error = runtime.estimate.s - final_error.s
    success = runtime.state is DockState.DOCKED
    if success:
        outcome = "docked"
    elif machine.abort_guard is not None:
        outcome = f"abort:{machine.abort_guard}"
    else:
        outcome = "unterminated"

    return EpisodeResult(
        seed=seed,
        success=success,
        outcome=outcome,
        duration=t,
        attempts=min(machine.attempt, config.policy.max_attempts),
        retries=machine.retries_used,
        repositions=sum(1 for h in machine.history if h.target is DockState.REPOSITION),
        start_error=start_error,
        final_error=final_error,
        contact_error=contact_error,
        longitudinal_offset=longitudinal_offset,
        dead_reckon_error=dead_reckon_error,
        blind_distance=blind_distance,
        straight_line_curvature=drive.straight_line_curvature,
        frames_seen=sensor.frames_seen,
        frames_dropped=sensor.frames_dropped,
        state_history=[(h.time, h.source.value, h.guard, h.target.value) for h in machine.history],
        trajectory=np.asarray(trajectory) if config.record else None,
    )
