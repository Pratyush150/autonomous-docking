"""States, the guard context, and every transition guard as a pure function.

A docking behaviour written as nested ``if`` statements is untestable, because
the condition that fires a transition is tangled with the code that acts on it.
Here the two are separated: a guard is a pure function of
:class:`GuardContext` and :class:`DockingPolicy` returning ``bool``, and
nothing else. Every guard in :data:`GUARDS` has a unit test, and the test suite
asserts that the transition table only references guards that exist and that
every guard is covered.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict

__all__ = ["DockState", "DockingPolicy", "GuardContext", "GUARDS", "guard"]


class DockState(Enum):
    """The docking behaviour, as an explicit set of states."""

    SEARCH = "search"
    ACQUIRE = "acquire"
    APPROACH = "approach"
    ALIGN = "align"
    REPOSITION = "reposition"
    FINAL_APPROACH = "final_approach"
    COMMIT = "commit"
    BLIND_APPROACH = "blind_approach"
    ENGAGE = "engage"
    VERIFY = "verify"
    RETREAT = "retreat"
    DOCKED = "docked"
    ABORTED = "aborted"

    @property
    def terminal(self) -> bool:
        """True for the two states with no outgoing transitions."""
        return self in (DockState.DOCKED, DockState.ABORTED)


@dataclass(frozen=True)
class DockingPolicy:
    """Every threshold the behaviour uses, in one place.

    There are two alignment windows, and the difference between them is the
    design.

    ``align_lateral_tol`` / ``align_yaw_tol`` are the *coarse* window checked at
    the standoff. Failing it means the approach was bad enough that another run
    at it is worth the time.

    ``commit_swept_budget`` / ``commit_yaw_budget`` are the *fine* gate, checked
    at ``commit_range`` with the robot stopped and averaging fixes, immediately
    before it goes blind. They are not applied to where the robot is; they are
    applied to where the planned blind segment says it will *end up*, and they
    are the mechanical tolerance minus the drift that accumulates after the
    gate. ``commit_lateral_tol`` / ``commit_yaw_tol`` sit outside them as a
    sanity bound on the estimate itself.

    This gate is a go/no-go. A differential drive parked at the commit range
    can bend its blind segment a little, but it cannot step sideways, so the
    only two options are commit, or back out and try again. Every number here
    is derived in ``docs/FAILURE_MODES.md``.
    """

    standoff: float = 0.75
    align_lateral_tol: float = 0.020
    align_yaw_tol: float = math.radians(3.0)
    standoff_reach_tol: float = 0.02
    commit_margin: float = 0.05
    commit_range: float = 0.35
    commit_lateral_tol: float = 0.035
    commit_yaw_tol: float = math.radians(4.0)
    commit_swept_budget: float = 0.013
    commit_yaw_budget: float = math.radians(3.5)
    commit_fixes_required: int = 10
    commit_max_lateral_sigma: float = 0.0035
    commit_timeout: float = 3.0
    acquire_fixes_required: int = 4
    acquire_max_lateral_sigma: float = 0.05
    search_timeout: float = 20.0
    fix_timeout: float = 4.0
    acquire_fix_timeout: float = 2.5
    max_attempts: int = 3
    max_repositions: int = 2
    episode_timeout: float = 150.0
    overtravel: float = 0.040
    reposition_backoff: float = 0.70
    retreat_distance: float = 1.30
    engage_dwell: float = 0.30
    verify_dwell: float = 0.40


@dataclass
class GuardContext:
    """Everything a guard is allowed to look at.

    Estimated quantities carry the ``est_`` prefix. Guards never see the true
    pose: the whole point of the exercise is that the robot decides on its
    estimate and is graded on the truth.
    """

    est_s: float = 0.0
    est_lateral: float = 0.0
    est_yaw: float = 0.0
    lateral_sigma: float = 10.0
    yaw_sigma: float = 10.0
    steps_since_fix: int = 10_000
    consecutive_fixes: int = 0
    fixes_in_state: int = 0
    has_fix: bool = False
    marker_detected: bool = False
    dt: float = 0.04
    total_time: float = 0.0
    odom_distance: float = 0.0
    time_in_state: float = 0.0
    distance_in_state: float = 0.0
    travel_budget: float = 0.0
    planned_swept: float = 1e9
    planned_yaw: float = 1e9
    bump: bool = False
    charge_present: bool = False
    attempt: int = 1
    repositions: int = 0
    max_attempts: int = 3

    @property
    def seconds_since_fix(self) -> float:
        """Wall-clock age of the last accepted fiducial fix."""
        return self.steps_since_fix * self.dt


GuardFn = Callable[[GuardContext, DockingPolicy], bool]
GUARDS: Dict[str, GuardFn] = {}


def guard(name: str) -> Callable[[GuardFn], GuardFn]:
    """Register a transition guard under ``name``."""

    def decorate(fn: GuardFn) -> GuardFn:
        if name in GUARDS:
            raise ValueError(f"duplicate guard name: {name}")
        GUARDS[name] = fn
        return fn

    return decorate


@guard("episode_timeout")
def episode_timeout(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """The whole docking sequence has taken too long. Hard abort from any state."""
    return ctx.total_time >= policy.episode_timeout


@guard("marker_detected")
def marker_detected(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """A fiducial fix was *accepted* by the estimator this tick.

    Deliberately not "the detector reported something". A detection the
    estimator throws out as an outlier has told the behaviour nothing, and
    leaving the search on it produces a state machine that flips between
    searching and acquiring at the control rate while getting no closer to a
    usable pose.
    """
    return ctx.marker_detected


@guard("search_exhausted")
def search_exhausted(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """A full search sweep finished without ever seeing the marker."""
    return ctx.time_in_state >= policy.search_timeout


@guard("pose_locked")
def pose_locked(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """Enough consecutive fixes, and the estimate is tight enough to drive on.

    A single detection is not an acquisition. One bad corner ordering or one
    reflection off a floor tile produces a pose that is confidently wrong, and
    acting on it sends the robot away from the dock.
    """
    return (
        ctx.consecutive_fixes >= policy.acquire_fixes_required
        and ctx.lateral_sigma <= policy.acquire_max_lateral_sigma
    )


@guard("acquisition_lost")
def acquisition_lost(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """The marker went away again before the lock was established."""
    return ctx.seconds_since_fix > policy.acquire_fix_timeout


@guard("fix_stale")
def fix_stale(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """No usable fix for too long while a fix was expected to be available."""
    return ctx.seconds_since_fix > policy.fix_timeout


@guard("standoff_reached")
def standoff_reached(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """The robot has arrived at the standoff pose on the dock axis."""
    return ctx.est_s <= policy.standoff + policy.standoff_reach_tol


@guard("aligned")
def aligned(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """Inside the alignment window: cleared to start the final approach.

    Both terms are required. Heading alone is not enough, because a
    differential drive that is perfectly parallel to the dock axis but 40 mm
    to one side of it cannot remove that 40 mm without first turning away from
    the axis -- and there is no room left to do that.
    """
    return (
        abs(ctx.est_lateral) <= policy.align_lateral_tol
        and abs(ctx.est_yaw) <= policy.align_yaw_tol
        and ctx.seconds_since_fix <= policy.fix_timeout
    )


@guard("lateral_out_of_tolerance")
def lateral_out_of_tolerance(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """Off the dock axis at the standoff, with repositioning budget left.

    Turning on the spot cannot fix this, so the only remedy is to back off and
    fly the approach again with more path length to converge over.
    """
    return abs(ctx.est_lateral) > policy.align_lateral_tol and ctx.repositions < policy.max_repositions


@guard("realign_exhausted")
def realign_exhausted(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """Off the axis at the standoff and out of repositioning budget."""
    return abs(ctx.est_lateral) > policy.align_lateral_tol and ctx.repositions >= policy.max_repositions


@guard("reposition_complete")
def reposition_complete(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """Backed far enough out along the axis to run the approach again."""
    return ctx.est_s >= policy.standoff + policy.reposition_backoff


@guard("commit_range_reached")
def commit_range_reached(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """Closed to the last range at which the marker is still observable."""
    return ctx.est_s <= policy.commit_range


@guard("commit_locked")
def commit_locked(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """Stopped, averaged, and inside the fine window: cleared to go blind.

    The sigma term matters as much as the offsets. Standing still removes
    motion blur and latency error, so a handful of stationary frames shrink the
    estimate's uncertainty in a way that no amount of driving does. Committing
    on a single frame means committing on that frame's noise.

    ``planned_swept`` and ``planned_yaw`` come from
    :meth:`autodock.controller.DockingController.plan_blind_arc`: they are the
    predicted contact pose under the best blind segment available from here,
    not the pose the robot is in now.
    """
    return (
        ctx.fixes_in_state >= policy.commit_fixes_required
        and ctx.lateral_sigma <= policy.commit_max_lateral_sigma
        and abs(ctx.est_lateral) <= policy.commit_lateral_tol
        and abs(ctx.est_yaw) <= policy.commit_yaw_tol
        and ctx.planned_swept <= policy.commit_swept_budget
        and abs(ctx.planned_yaw) <= policy.commit_yaw_budget
    )


@guard("commit_rejected")
def commit_rejected(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """The go/no-go window closed without a lock, and a re-approach is still allowed.

    Nothing has been hit, so this is the cheap failure: back out along the axis
    and fly the approach again rather than spending one of the retry attempts.
    """
    return ctx.time_in_state >= policy.commit_timeout and ctx.repositions < policy.max_repositions


@guard("commit_abandoned")
def commit_abandoned(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """Out of re-approaches at the commit gate. Give the attempt up.

    Refusing to enter the dock is the correct outcome here. Committing on an
    estimate that failed its own quality check is how a robot ends up wedged
    against the funnel lip with the drive stalled.
    """
    return ctx.time_in_state >= policy.commit_timeout


@guard("bump_detected")
def bump_detected(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """The contact switch closed: the robot is touching something."""
    return ctx.bump


@guard("travel_budget_exceeded")
def travel_budget_exceeded(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """Drove the whole dead-reckoned distance plus the overtravel, no contact.

    Either the robot stopped short because odometry over-reported distance, or
    it went straight past the dock because it was too far off the axis to touch
    it. Both are failed attempts, and both must stop the drive rather than let
    it push indefinitely.
    """
    return ctx.distance_in_state >= ctx.travel_budget


@guard("engage_settled")
def engage_settled(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """Held against the dock long enough for the spring contacts to seat."""
    return ctx.time_in_state >= policy.engage_dwell


@guard("charge_confirmed")
def charge_confirmed(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """Charging voltage present after the settling window: really docked.

    The bump switch says the robot hit something. Only this says it mated.
    Skipping it is how a robot ends up parked against its dock with a flat
    battery.
    """
    return ctx.charge_present and ctx.time_in_state >= policy.verify_dwell


@guard("charge_absent")
def charge_absent(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """Contact made, no charge. Mechanically touching, electrically not docked."""
    return (not ctx.charge_present) and ctx.time_in_state >= policy.verify_dwell


@guard("attempts_exhausted")
def attempts_exhausted(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """The retry budget is spent. This is the stated abort condition."""
    return ctx.attempt > policy.max_attempts


@guard("retreat_complete")
def retreat_complete(ctx: GuardContext, policy: DockingPolicy) -> bool:
    """Backed out far enough to have the marker in view and try again."""
    return ctx.distance_in_state >= policy.retreat_distance
