"""The transition table itself: structure, ordering, and the retry bookkeeping."""

from __future__ import annotations

from autodock.machine import TRANSITIONS, DockingStateMachine
from autodock.states import GUARDS, DockingPolicy, DockState, GuardContext

POLICY = DockingPolicy()


def blank(**kwargs) -> GuardContext:
    kwargs.setdefault("total_time", 1.0)
    return GuardContext(**kwargs)


def test_every_transition_names_a_registered_guard():
    for tr in TRANSITIONS:
        assert tr.guard in GUARDS


def test_terminal_states_have_no_outgoing_transitions():
    for tr in TRANSITIONS:
        assert not tr.source.terminal, tr


def test_every_non_terminal_state_can_be_left():
    sources = {tr.source for tr in TRANSITIONS}
    for state in DockState:
        if not state.terminal:
            assert state in sources, state


def test_every_state_is_reachable_from_search():
    reached = {DockState.SEARCH}
    changed = True
    while changed:
        changed = False
        for tr in TRANSITIONS:
            if tr.source in reached and tr.target not in reached:
                reached.add(tr.target)
                changed = True
    assert reached == set(DockState)


def test_abort_is_evaluated_before_anything_else_in_every_state():
    """A hard abort must win over a guard that would otherwise keep the run going."""
    machine = DockingStateMachine(POLICY)
    for state in DockState:
        if state.terminal:
            continue
        first = machine.outgoing(state)[0]
        assert first.target is DockState.ABORTED, state


def test_the_first_satisfied_guard_wins():
    machine = DockingStateMachine(POLICY, state=DockState.SEARCH)
    # Both episode_timeout and marker_detected hold; the abort is listed first.
    change = machine.step(blank(total_time=POLICY.episode_timeout, marker_detected=True))
    assert change is not None
    assert change.target is DockState.ABORTED
    assert change.guard == "episode_timeout"


def test_no_transition_returns_none_and_leaves_the_state_alone():
    machine = DockingStateMachine(POLICY, state=DockState.SEARCH)
    assert machine.step(blank()) is None
    assert machine.state is DockState.SEARCH


def test_terminal_states_ignore_further_input():
    machine = DockingStateMachine(POLICY, state=DockState.DOCKED)
    assert machine.step(blank(total_time=1e9)) is None
    assert machine.state is DockState.DOCKED


def test_entering_retreat_consumes_an_attempt_and_resets_repositions():
    machine = DockingStateMachine(POLICY, state=DockState.ALIGN)
    machine.repositions = POLICY.max_repositions
    change = machine.step(blank(est_lateral=1.0, steps_since_fix=0))
    assert change.target is DockState.RETREAT
    assert machine.attempt == 2
    assert machine.repositions == 0


def test_repositions_are_counted_and_then_refused():
    machine = DockingStateMachine(POLICY, state=DockState.ALIGN)
    for expected in (1, 2):
        machine.state = DockState.ALIGN
        change = machine.step(blank(est_lateral=1.0, steps_since_fix=0))
        assert change.target is DockState.REPOSITION
        assert machine.repositions == expected
    machine.state = DockState.ALIGN
    change = machine.step(blank(est_lateral=1.0, steps_since_fix=0))
    assert change.target is DockState.RETREAT


def test_retries_are_bounded_and_the_abort_condition_fires():
    """Fail verification forever; the machine must stop, not loop."""
    machine = DockingStateMachine(POLICY, state=DockState.VERIFY)
    clock, odometer = 0.0, 0.0
    for _ in range(10 * POLICY.max_attempts):
        if machine.state is DockState.ABORTED:
            break
        clock += 1.0
        odometer += 2.0
        change = machine.step(blank(total_time=clock, odom_distance=odometer))
        if change is not None and change.target is DockState.SEARCH:
            machine.state = DockState.VERIFY  # pretend the next attempt got back here
    assert machine.state is DockState.ABORTED
    assert machine.abort_guard == "attempts_exhausted"
    assert machine.attempt == POLICY.max_attempts + 1
    assert machine.retries_used == POLICY.max_attempts - 1


def test_time_in_state_is_measured_from_the_last_transition():
    machine = DockingStateMachine(POLICY, state=DockState.SEARCH)
    machine.step(blank(total_time=5.0, marker_detected=True))
    assert machine.entry_time == 5.0
    assert machine.state is DockState.ACQUIRE
    # 10 s later, only 5 s have been spent in ACQUIRE. Nothing here is measured
    # against the absolute clock, so no guard may fire.
    assert machine.step(blank(total_time=10.0, steps_since_fix=0)) is None


def test_distance_in_state_is_measured_from_the_last_transition():
    machine = DockingStateMachine(POLICY, state=DockState.RETREAT)
    machine.entry_distance = 4.0
    ctx = blank(odom_distance=4.0 + POLICY.retreat_distance - 0.01)
    assert machine.step(ctx) is None
    ctx = blank(odom_distance=4.0 + POLICY.retreat_distance + 1e-9)
    assert machine.step(ctx).target is DockState.SEARCH


def test_history_records_what_fired_and_when():
    machine = DockingStateMachine(POLICY, state=DockState.SEARCH)
    machine.step(blank(total_time=2.0, marker_detected=True))
    assert len(machine.history) == 1
    entry = machine.history[0]
    assert (entry.time, entry.source, entry.guard, entry.target) == (
        2.0,
        DockState.SEARCH,
        "marker_detected",
        DockState.ACQUIRE,
    )
