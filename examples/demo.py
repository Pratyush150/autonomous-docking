#!/usr/bin/env python3
"""One docking run, narrated, then a small Monte Carlo campaign.

    python3 examples/demo.py --demo

No arguments needed, no display, no network. Runs in a few seconds.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from autodock import EpisodeConfig, run_campaign, run_episode, summarise  # noqa: E402
from autodock.states import DockState  # noqa: E402

BAR = "-" * 78


def narrate(config: EpisodeConfig, seed: int) -> None:
    """Run one episode and print its state timeline and contact pose."""
    result = run_episode(config, seed)
    start = result.start_error
    print(BAR)
    print(f"Single episode, seed {seed}")
    print(BAR)
    print(
        f"start: {start.s:.2f} m out along the dock axis, "
        f"{start.lateral * 1000:+.0f} mm off it, heading {math.degrees(start.yaw):+.0f} deg"
    )
    print(f"drive train: commanded straight, actually curves at {result.straight_line_curvature:+.4f} 1/m")
    print()
    print(f"{'t [s]':>7}  {'from':<15} {'guard':<24} {'to':<15}")
    for t, source, guard, target in result.state_history:
        print(f"{t:7.2f}  {source:<15} {guard:<24} {target:<15}")
    print()
    if result.contact_error is not None:
        err = result.contact_error
        dock = config.dock
        print(
            f"contact pose:  lateral {err.lateral * 1000:+.1f} mm, "
            f"heading {math.degrees(err.yaw):+.2f} deg, "
            f"swept half-width {dock.swept_half_width(err) * 1000:.1f} mm "
            f"(tolerance {dock.lateral_tolerance * 1000:.0f} mm)"
        )
    print(f"blind segment: {result.blind_distance * 1000:.0f} mm driven with no marker in view")
    print(f"dead reckoning was off by {result.dead_reckon_error * 1000:+.1f} mm in range at contact")
    print(f"detections: {result.frames_seen} used, {result.frames_dropped} dropped")
    print(f"outcome: {result.outcome} after {result.duration:.1f} s, {result.retries} retries")
    print()


def campaign(config: EpisodeConfig, episodes: int, workers: int) -> None:
    """Run a campaign and print the distribution table."""
    print(BAR)
    print(f"Monte Carlo, {episodes} randomised starts and disturbance draws")
    print(BAR)
    results = run_campaign(config, episodes=episodes, seed0=0, workers=workers)
    summary = summarise(results, config.dock)
    print(summary.headline())
    print()
    print(summary.table())
    print()
    print("outcomes:", summary.outcomes)
    print("retries: ", summary.retry_histogram)
    print()
    reached = {}
    for result in results:
        for _, _, _, target in result.state_history:
            reached[target] = reached.get(target, 0) + 1
    ordered = [s.value for s in DockState if s.value in reached]
    print("state entries across the campaign:")
    for name in ordered:
        print(f"  {name:<16} {reached[name]}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--demo", action="store_true", help="run the default demonstration (this is the default)")
    parser.add_argument("--seed", type=int, default=1, help="seed for the narrated episode")
    parser.add_argument("--episodes", type=int, default=80, help="episodes in the campaign")
    parser.add_argument("--workers", type=int, default=1, help="processes for the campaign")
    args = parser.parse_args()

    config = EpisodeConfig()
    narrate(config, args.seed)
    campaign(config, args.episodes, args.workers)


if __name__ == "__main__":
    main()
