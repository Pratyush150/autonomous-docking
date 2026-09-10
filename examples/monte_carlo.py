#!/usr/bin/env python3
"""Monte Carlo campaigns: the nominal robot, and a worn one.

    python3 examples/monte_carlo.py --episodes 400 --workers 8

The second campaign is the same design on a drive train three times less well
matched -- the fleet a year after commissioning. It is there because the
nominal numbers on their own say nothing about how the design ages.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from autodock import EpisodeConfig, run_campaign, summarise  # noqa: E402
from autodock.montecarlo import with_drift_scale  # noqa: E402

BAR = "=" * 78


def report(name: str, config: EpisodeConfig, episodes: int, seed0: int, workers: int) -> None:
    """Run one campaign and print its summary."""
    results = run_campaign(config, episodes=episodes, seed0=seed0, workers=workers)
    summary = summarise(results, config.dock)
    print(BAR)
    print(name)
    print(BAR)
    print(summary.headline())
    print()
    print(summary.table())
    print()
    print("outcomes:         ", summary.outcomes)
    print("retries used:     ", summary.retry_histogram)
    print(
        "inside tolerance:  "
        f"{100.0 * summary.success_rate:.1f}% of episodes ended charging "
        f"(mechanical tolerance {config.dock.lateral_tolerance * 1000:.0f} mm swept, "
        f"{config.dock.yaw_tolerance * 57.29578:.0f} deg heading)"
    )
    print()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--episodes", type=int, default=400)
    parser.add_argument("--seed0", type=int, default=0)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--worn-drift", type=float, default=3.0, help="drift scale for the second campaign")
    args = parser.parse_args()

    base = EpisodeConfig()
    report("Nominal drive train (drift scale 1.0)", base, args.episodes, args.seed0, args.workers)
    report(
        f"Worn drive train (drift scale {args.worn_drift})",
        with_drift_scale(base, args.worn_drift),
        args.episodes,
        args.seed0,
        args.workers,
    )


if __name__ == "__main__":
    main()
