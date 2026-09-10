"""Every transition guard, asserted true in the case it exists for and false next to it.

Guards are the whole specification of the docking behaviour, so they get tested
one at a time and by name. The final test in this file fails if a guard is ever
added without a test, which is the point of keeping them in a registry.
"""

from __future__ import annotations

from autodock.machine import TRANSITIONS
from autodock.states import GUARDS, DockingPolicy, GuardContext

POLICY = DockingPolicy()


def ctx(**kwargs) -> GuardContext:
    """A context that satisfies no guard, with the named fields overridden."""
    base = dict(
        est_s=2.0,
        est_lateral=0.0,
        est_yaw=0.0,
        lateral_sigma=10.0,
        yaw_sigma=10.0,
        steps_since_fix=0,
        consecutive_fixes=0,
        fixes_in_state=0,
        has_fix=False,
        marker_detected=False,
        dt=0.04,
        total_time=1.0,
        odom_distance=0.0,
        time_in_state=0.0,
        distance_in_state=0.0,
        travel_budget=1.0,
        planned_swept=1.0,
        planned_yaw=1.0,
        bump=False,
        charge_present=False,
        attempt=1,
        repositions=0,
    )
    base.update(kwargs)
    return GuardContext(**base)


def fires(name: str, **kwargs) -> bool:
    return GUARDS[name](ctx(**kwargs), POLICY)


def test_episode_timeout():
    assert fires("episode_timeout", total_time=POLICY.episode_timeout)
    assert not fires("episode_timeout", total_time=POLICY.episode_timeout - 0.01)


def test_marker_detected():
    assert fires("marker_detected", marker_detected=True)
    assert not fires("marker_detected", marker_detected=False)


def test_search_exhausted():
    assert fires("search_exhausted", time_in_state=POLICY.search_timeout)
    assert not fires("search_exhausted", time_in_state=POLICY.search_timeout - 0.01)


def test_pose_locked_needs_both_a_run_of_fixes_and_a_tight_estimate():
    good = dict(consecutive_fixes=POLICY.acquire_fixes_required, lateral_sigma=POLICY.acquire_max_lateral_sigma)
    assert fires("pose_locked", **good)
    assert not fires("pose_locked", **{**good, "consecutive_fixes": POLICY.acquire_fixes_required - 1})
    assert not fires("pose_locked", **{**good, "lateral_sigma": POLICY.acquire_max_lateral_sigma * 1.01})


def test_acquisition_lost():
    stale = int(POLICY.acquire_fix_timeout / 0.04) + 1
    assert fires("acquisition_lost", steps_since_fix=stale)
    assert not fires("acquisition_lost", steps_since_fix=1)


def test_fix_stale():
    stale = int(POLICY.fix_timeout / 0.04) + 1
    assert fires("fix_stale", steps_since_fix=stale)
    assert not fires("fix_stale", steps_since_fix=1)


def test_standoff_reached():
    assert fires("standoff_reached", est_s=POLICY.standoff + POLICY.standoff_reach_tol)
    assert not fires("standoff_reached", est_s=POLICY.standoff + POLICY.standoff_reach_tol + 0.01)


def test_aligned_requires_lateral_heading_and_a_fresh_fix():
    good = dict(est_lateral=0.0, est_yaw=0.0, steps_since_fix=0)
    assert fires("aligned", **good)
    assert not fires("aligned", **{**good, "est_lateral": POLICY.align_lateral_tol * 1.01})
    assert not fires("aligned", **{**good, "est_yaw": POLICY.align_yaw_tol * 1.01})
    assert not fires("aligned", **{**good, "steps_since_fix": int(POLICY.fix_timeout / 0.04) + 1})


def test_aligned_is_a_position_test_not_only_a_heading_test():
    """Turning on the spot cannot fix lateral error, so heading alone is not enough."""
    assert not fires("aligned", est_lateral=0.05, est_yaw=0.0)


def test_lateral_out_of_tolerance_only_while_repositions_remain():
    assert fires("lateral_out_of_tolerance", est_lateral=POLICY.align_lateral_tol * 1.5, repositions=0)
    assert not fires("lateral_out_of_tolerance", est_lateral=0.0, repositions=0)
    assert not fires(
        "lateral_out_of_tolerance",
        est_lateral=POLICY.align_lateral_tol * 1.5,
        repositions=POLICY.max_repositions,
    )


def test_realign_exhausted_is_the_complement():
    assert fires("realign_exhausted", est_lateral=POLICY.align_lateral_tol * 1.5, repositions=POLICY.max_repositions)
    assert not fires("realign_exhausted", est_lateral=0.0, repositions=POLICY.max_repositions)


def test_reposition_complete():
    assert fires("reposition_complete", est_s=POLICY.standoff + POLICY.reposition_backoff)
    assert not fires("reposition_complete", est_s=POLICY.standoff + POLICY.reposition_backoff - 0.01)


def test_commit_range_reached():
    assert fires("commit_range_reached", est_s=POLICY.commit_range)
    assert not fires("commit_range_reached", est_s=POLICY.commit_range + 0.01)


def test_commit_locked_needs_every_condition():
    good = dict(
        fixes_in_state=POLICY.commit_fixes_required,
        lateral_sigma=POLICY.commit_max_lateral_sigma,
        est_lateral=0.0,
        est_yaw=0.0,
        planned_swept=POLICY.commit_swept_budget,
        planned_yaw=POLICY.commit_yaw_budget,
    )
    assert fires("commit_locked", **good)
    for field, bad in [
        ("fixes_in_state", POLICY.commit_fixes_required - 1),
        ("lateral_sigma", POLICY.commit_max_lateral_sigma * 1.01),
        ("est_lateral", POLICY.commit_lateral_tol * 1.01),
        ("est_yaw", POLICY.commit_yaw_tol * 1.01),
        ("planned_swept", POLICY.commit_swept_budget * 1.01),
        ("planned_yaw", POLICY.commit_yaw_budget * 1.01),
    ]:
        assert not fires("commit_locked", **{**good, field: bad}), field


def test_commit_gate_is_about_where_the_blind_segment_ends_not_where_the_robot_is():
    """Sitting off the axis is fine if the planned arc still lands in tolerance."""
    good = dict(
        fixes_in_state=POLICY.commit_fixes_required,
        lateral_sigma=POLICY.commit_max_lateral_sigma,
        est_lateral=0.9 * POLICY.commit_lateral_tol,
        est_yaw=0.0,
        planned_swept=0.5 * POLICY.commit_swept_budget,
        planned_yaw=0.0,
    )
    assert fires("commit_locked", **good)
    assert not fires("commit_locked", **{**good, "planned_swept": POLICY.commit_swept_budget * 2.0})


def test_commit_rejected_and_abandoned_split_on_the_reposition_budget():
    late = POLICY.commit_timeout
    assert fires("commit_rejected", time_in_state=late, repositions=0)
    assert not fires("commit_rejected", time_in_state=late, repositions=POLICY.max_repositions)
    assert fires("commit_abandoned", time_in_state=late, repositions=POLICY.max_repositions)
    assert not fires("commit_abandoned", time_in_state=late - 0.01, repositions=POLICY.max_repositions)


def test_bump_detected():
    assert fires("bump_detected", bump=True)
    assert not fires("bump_detected", bump=False)


def test_travel_budget_exceeded():
    assert fires("travel_budget_exceeded", distance_in_state=1.0, travel_budget=1.0)
    assert not fires("travel_budget_exceeded", distance_in_state=0.99, travel_budget=1.0)


def test_engage_settled():
    assert fires("engage_settled", time_in_state=POLICY.engage_dwell)
    assert not fires("engage_settled", time_in_state=POLICY.engage_dwell - 0.01)


def test_charge_confirmed_needs_voltage_and_the_settling_window():
    assert fires("charge_confirmed", charge_present=True, time_in_state=POLICY.verify_dwell)
    assert not fires("charge_confirmed", charge_present=True, time_in_state=POLICY.verify_dwell - 0.01)
    assert not fires("charge_confirmed", charge_present=False, time_in_state=POLICY.verify_dwell)


def test_charge_absent_is_the_complement_after_the_same_window():
    assert fires("charge_absent", charge_present=False, time_in_state=POLICY.verify_dwell)
    assert not fires("charge_absent", charge_present=True, time_in_state=POLICY.verify_dwell)
    assert not fires("charge_absent", charge_present=False, time_in_state=POLICY.verify_dwell - 0.01)


def test_attempts_exhausted_is_the_stated_abort_condition():
    assert fires("attempts_exhausted", attempt=POLICY.max_attempts + 1)
    assert not fires("attempts_exhausted", attempt=POLICY.max_attempts)


def test_retreat_complete():
    assert fires("retreat_complete", distance_in_state=POLICY.retreat_distance)
    assert not fires("retreat_complete", distance_in_state=POLICY.retreat_distance - 0.01)


def test_every_registered_guard_has_a_test_in_this_file():
    source = open(__file__, encoding="utf-8").read()
    missing = [name for name in GUARDS if f'"{name}"' not in source]
    assert missing == []


def test_every_registered_guard_is_used_by_the_transition_table():
    used = {t.guard for t in TRANSITIONS}
    assert set(GUARDS) == used
