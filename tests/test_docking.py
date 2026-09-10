"""Closed-loop properties of whole episodes.

These are the tests that would catch a regression in the behaviour rather than
in a formula: the robot never commits to the blind segment from a state its own
gate should have refused, retries are bounded, and the abort condition fires
when docking is impossible instead of the robot trying forever.
"""

from __future__ import annotations

import math
from dataclasses import replace

from autodock.geometry import DockGeometry
from autodock.metrics import summarise
from autodock.montecarlo import run_campaign, with_blind_radius, with_drift_scale
from autodock.perception import MarkerModel
from autodock.sim import EpisodeConfig, run_episode
from autodock.states import DockState

BASE = EpisodeConfig()


def entries(config: EpisodeConfig, seeds, target: DockState):
    """Collect the guard contexts at every transition into ``target``."""
    found = []

    def observer(change, ctx):
        if change.target is target:
            found.append(ctx)

    for seed in seeds:
        run_episode(config, seed, observer=observer)
    return found


def test_the_baseline_design_docks():
    results = run_campaign(BASE, episodes=60, seed0=0)
    summary = summarise(results, BASE.dock)
    assert summary.success_rate >= 0.90
    assert all(r.outcome != "unterminated" for r in results)


def test_every_episode_reaches_a_terminal_state():
    for config in (BASE, with_drift_scale(BASE, 5.0), with_blind_radius(BASE, 0.9)):
        for seed in range(6):
            result = run_episode(config, seed)
            assert result.outcome != "unterminated"
            assert result.outcome == "docked" or result.outcome.startswith("abort:")


def test_the_blind_segment_is_never_entered_outside_the_commit_gate():
    """The property the whole design exists to guarantee."""
    policy = BASE.policy
    config = with_drift_scale(BASE, 4.0)
    contexts = entries(config, range(25), DockState.BLIND_APPROACH)
    assert contexts, "no episode reached the blind approach"
    for ctx in contexts:
        assert ctx.fixes_in_state >= policy.commit_fixes_required
        assert ctx.lateral_sigma <= policy.commit_max_lateral_sigma
        assert abs(ctx.est_lateral) <= policy.commit_lateral_tol
        assert abs(ctx.est_yaw) <= policy.commit_yaw_tol
        assert ctx.planned_swept <= policy.commit_swept_budget
        assert abs(ctx.planned_yaw) <= policy.commit_yaw_budget


def test_the_final_approach_is_never_started_outside_the_alignment_window():
    policy = BASE.policy
    contexts = entries(with_drift_scale(BASE, 3.0), range(25), DockState.FINAL_APPROACH)
    assert contexts
    for ctx in contexts:
        assert abs(ctx.est_lateral) <= policy.align_lateral_tol
        assert abs(ctx.est_yaw) <= policy.align_yaw_tol
        assert ctx.seconds_since_fix <= policy.fix_timeout


def test_retries_are_bounded_even_when_nothing_works():
    """A dock that cannot be mated must consume exactly the retry budget."""
    impossible = replace(BASE, dock=DockGeometry(lateral_tolerance=0.0002, yaw_tolerance=math.radians(0.05)))
    results = [run_episode(impossible, seed) for seed in range(8)]
    for result in results:
        assert not result.success
        assert result.outcome == "abort:attempts_exhausted"
        assert result.attempts == BASE.policy.max_attempts
        assert result.retries == BASE.policy.max_attempts - 1
        engagements = sum(1 for h in result.state_history if h[3] == "engage")
        assert engagements <= BASE.policy.max_attempts


def test_repositions_are_bounded_within_an_attempt():
    tight = replace(BASE, policy=replace(BASE.policy, align_lateral_tol=1e-6))
    for seed in range(6):
        result = run_episode(tight, seed)
        per_attempt = 0
        for _, _, guard, target in result.state_history:
            if target == "reposition":
                per_attempt += 1
                assert per_attempt <= BASE.policy.max_repositions
            if target == "retreat":
                per_attempt = 0


def test_abort_fires_when_the_marker_can_never_be_seen():
    blind = replace(BASE, marker=MarkerModel(max_range=0.05))
    result = run_episode(blind, 0)
    assert result.outcome == "abort:search_exhausted"
    assert result.duration <= BASE.policy.search_timeout + 1.0


def test_abort_fires_on_the_episode_timeout():
    slow = replace(BASE, policy=replace(BASE.policy, episode_timeout=3.0))
    result = run_episode(slow, 0)
    assert result.outcome == "abort:episode_timeout"
    assert result.duration <= 3.0 + BASE.dt


def test_docking_only_succeeds_when_the_true_pose_is_inside_the_mechanical_tolerance():
    """Success is graded on truth, never on the robot's own opinion."""
    for seed in range(40):
        result = run_episode(BASE, seed)
        if result.success:
            assert result.contact_error is not None
            assert BASE.dock.engagement_ok(result.contact_error)


def test_a_contact_outside_tolerance_is_caught_by_verification_not_by_the_bump():
    """Touching the dock is not docking; the charge check is what decides."""
    results = [run_episode(with_drift_scale(BASE, 6.0), seed) for seed in range(20)]
    touched_but_failed = [r for r in results if r.contact_error is not None and not r.success]
    assert touched_but_failed, "expected some failed matings at this drift level"
    for result in touched_but_failed:
        assert any(guard == "charge_absent" for _, _, guard, _ in result.state_history)


def test_episodes_are_reproducible_from_their_seed():
    a = run_episode(BASE, 123)
    b = run_episode(BASE, 123)
    assert a.outcome == b.outcome
    assert a.duration == b.duration
    assert a.lateral_offset == b.lateral_offset
    c = run_episode(BASE, 124)
    assert (c.start_error.lateral, c.start_error.yaw) != (a.start_error.lateral, a.start_error.yaw)


def test_more_drift_and_a_longer_blind_segment_both_make_it_worse():
    def rate(config) -> float:
        return summarise(run_campaign(config, episodes=25, seed0=500), config.dock).success_rate

    assert rate(BASE) > rate(with_drift_scale(BASE, 5.0))
    assert rate(BASE) > rate(with_blind_radius(BASE, 0.9))


def test_the_state_machine_visits_the_states_in_the_designed_order():
    result = run_episode(BASE, 1)
    assert result.success
    visited = [target for _, _, _, target in result.state_history]
    expected = ["acquire", "approach", "align", "final_approach", "commit", "blind_approach", "engage", "verify", "docked"]
    assert [v for v in visited if v in expected] == expected
