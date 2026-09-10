"""The docking state machine: an explicit, ordered, fully guarded transition table.

Reading the table is the fastest way to understand the behaviour::

    SEARCH ---marker_detected--------> ACQUIRE
    ACQUIRE --pose_locked-----------> APPROACH
    APPROACH -standoff_reached------> ALIGN
    ALIGN ----aligned---------------> FINAL_APPROACH
    ALIGN ----lateral_out_of_tol----> REPOSITION -> APPROACH
    FINAL_APPROACH --commit_range---> COMMIT
    COMMIT ---commit_locked---------> BLIND_APPROACH
    COMMIT ---commit_rejected-------> REPOSITION -> APPROACH
    COMMIT ---commit_abandoned------> RETREAT
    BLIND_APPROACH --bump_detected--> ENGAGE -> VERIFY
    VERIFY ---charge_confirmed------> DOCKED
    VERIFY ---charge_absent---------> RETREAT -> SEARCH
    <any> ----episode_timeout-------> ABORTED
    RETREAT --attempts_exhausted----> ABORTED

Transitions for a state are evaluated in order and the first satisfied guard
wins, so the ordering is part of the specification. Aborts are listed first in
every state, which is what makes "the abort condition always wins" a property
you can test rather than a convention you hope holds.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from .states import GUARDS, DockingPolicy, DockState, GuardContext

__all__ = ["Transition", "TRANSITIONS", "DockingStateMachine", "StateChange"]


@dataclass(frozen=True)
class Transition:
    """One row of the table."""

    source: DockState
    guard: str
    target: DockState


def _t(source: DockState, guard: str, target: DockState) -> Transition:
    if guard not in GUARDS:
        raise KeyError(f"unknown guard {guard!r}")
    return Transition(source, guard, target)


S = DockState

TRANSITIONS: Tuple[Transition, ...] = (
    _t(S.SEARCH, "episode_timeout", S.ABORTED),
    _t(S.SEARCH, "marker_detected", S.ACQUIRE),
    _t(S.SEARCH, "search_exhausted", S.ABORTED),

    _t(S.ACQUIRE, "episode_timeout", S.ABORTED),
    _t(S.ACQUIRE, "pose_locked", S.APPROACH),
    _t(S.ACQUIRE, "acquisition_lost", S.SEARCH),

    _t(S.APPROACH, "episode_timeout", S.ABORTED),
    _t(S.APPROACH, "standoff_reached", S.ALIGN),
    _t(S.APPROACH, "fix_stale", S.SEARCH),

    _t(S.ALIGN, "episode_timeout", S.ABORTED),
    _t(S.ALIGN, "fix_stale", S.SEARCH),
    _t(S.ALIGN, "aligned", S.FINAL_APPROACH),
    _t(S.ALIGN, "lateral_out_of_tolerance", S.REPOSITION),
    _t(S.ALIGN, "realign_exhausted", S.RETREAT),

    _t(S.REPOSITION, "episode_timeout", S.ABORTED),
    _t(S.REPOSITION, "fix_stale", S.SEARCH),
    _t(S.REPOSITION, "reposition_complete", S.APPROACH),

    _t(S.FINAL_APPROACH, "episode_timeout", S.ABORTED),
    _t(S.FINAL_APPROACH, "bump_detected", S.ENGAGE),
    _t(S.FINAL_APPROACH, "commit_range_reached", S.COMMIT),
    _t(S.FINAL_APPROACH, "fix_stale", S.RETREAT),

    _t(S.COMMIT, "episode_timeout", S.ABORTED),
    _t(S.COMMIT, "commit_locked", S.BLIND_APPROACH),
    _t(S.COMMIT, "commit_rejected", S.REPOSITION),
    _t(S.COMMIT, "commit_abandoned", S.RETREAT),

    _t(S.BLIND_APPROACH, "episode_timeout", S.ABORTED),
    _t(S.BLIND_APPROACH, "bump_detected", S.ENGAGE),
    _t(S.BLIND_APPROACH, "travel_budget_exceeded", S.RETREAT),

    _t(S.ENGAGE, "episode_timeout", S.ABORTED),
    _t(S.ENGAGE, "engage_settled", S.VERIFY),

    _t(S.VERIFY, "episode_timeout", S.ABORTED),
    _t(S.VERIFY, "charge_confirmed", S.DOCKED),
    _t(S.VERIFY, "charge_absent", S.RETREAT),

    _t(S.RETREAT, "attempts_exhausted", S.ABORTED),
    _t(S.RETREAT, "episode_timeout", S.ABORTED),
    _t(S.RETREAT, "retreat_complete", S.SEARCH),
)


@dataclass(frozen=True)
class StateChange:
    """A transition that actually fired, with the clock reading."""

    time: float
    source: DockState
    guard: str
    target: DockState


@dataclass
class DockingStateMachine:
    """Holds the current state and the retry bookkeeping. No control law here.

    The machine is deliberately ignorant of geometry, controllers and sensors.
    It takes a :class:`GuardContext`, fills in the two fields it owns
    (``time_in_state`` and ``distance_in_state``), evaluates the table, and
    returns the transition that fired.
    """

    policy: DockingPolicy = field(default_factory=DockingPolicy)
    state: DockState = DockState.SEARCH
    attempt: int = 1
    repositions: int = 0
    entry_time: float = 0.0
    entry_distance: float = 0.0
    history: List[StateChange] = field(default_factory=list)
    abort_guard: Optional[str] = None
    _by_state: Dict[DockState, Tuple[Transition, ...]] = field(default_factory=dict, init=False, repr=False)

    def __post_init__(self) -> None:
        table: Dict[DockState, List[Transition]] = {}
        for tr in TRANSITIONS:
            table.setdefault(tr.source, []).append(tr)
        self._by_state = {k: tuple(v) for k, v in table.items()}

    @property
    def retries_used(self) -> int:
        """Number of retries consumed, i.e. attempts after the first."""
        return max(0, min(self.attempt, self.policy.max_attempts) - 1)

    def outgoing(self, state: DockState) -> Tuple[Transition, ...]:
        """Transitions leaving ``state``, in evaluation order."""
        return self._by_state.get(state, ())

    def step(self, ctx: GuardContext) -> Optional[StateChange]:
        """Evaluate the table once. Returns the transition taken, if any."""
        if self.state.terminal:
            return None
        ctx.time_in_state = ctx.total_time - self.entry_time
        ctx.distance_in_state = ctx.odom_distance - self.entry_distance
        ctx.attempt = self.attempt
        ctx.repositions = self.repositions
        ctx.max_attempts = self.policy.max_attempts

        for tr in self.outgoing(self.state):
            if GUARDS[tr.guard](ctx, self.policy):
                return self._enter(tr, ctx)
        return None

    def _enter(self, tr: Transition, ctx: GuardContext) -> StateChange:
        change = StateChange(ctx.total_time, self.state, tr.guard, tr.target)
        self.history.append(change)
        self.state = tr.target
        self.entry_time = ctx.total_time
        self.entry_distance = ctx.odom_distance
        if tr.target is DockState.RETREAT:
            self.attempt += 1
            self.repositions = 0
        if tr.target is DockState.REPOSITION:
            self.repositions += 1
        if tr.target is DockState.ABORTED:
            self.abort_guard = tr.guard
        return change
