#!/usr/bin/env python3
"""Where the design stops working: one-dimensional sweeps and a 2-D boundary.

    python3 examples/sweep.py --episodes 150 --workers 8

Writes a CSV of every grid point next to the figures so the numbers in the
README can be checked without re-running anything.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from autodock import EpisodeConfig, run_campaign, run_sweep, summarise  # noqa: E402
from autodock.montecarlo import with_blind_radius, with_drift_scale  # noqa: E402

DRIFT = [0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
BLIND = [0.15, 0.25, 0.35, 0.50, 0.65, 0.80, 0.95]
MISALIGNMENT = [0.25, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]

GRID_DRIFT = [1.0, 2.0, 3.0, 4.0, 5.0]
GRID_BLIND = [0.20, 0.35, 0.50, 0.65, 0.80]


def one_d(config: EpisodeConfig, episodes: int, workers: int, rows: list) -> None:
    """Run the three single-axis sweeps and print them as tables."""
    axes = [
        ("drift", DRIFT, "odometry drift scale", ""),
        ("blind", BLIND, "blind segment", " m"),
        ("misalignment", MISALIGNMENT, "initial lateral spread scale", ""),
    ]
    for axis, values, label, unit in axes:
        print(f"\n{label}")
        print(f"| {label} | success | median swept (mm) | 95th swept (mm) | mean retries | median time (s) |")
        print("|---|---|---|---|---|---|")
        points = run_sweep(config, axis, values, episodes=episodes, workers=workers)
        for point in points:
            s = point.summary
            print(
                f"| {point.value:g}{unit} | {100.0 * s.success_rate:.1f}% | {s.swept_mm.p50:.1f} | "
                f"{s.swept_mm.p95:.1f} | {s.retries.mean:.2f} | {s.duration_s.p50:.1f} |"
            )
            rows.append(
                {
                    "sweep": axis,
                    "value": point.value,
                    "value2": "",
                    "episodes": s.episodes,
                    "success_rate": round(s.success_rate, 4),
                    "contact_rate": round(s.contact_rate, 4),
                    "swept_p50_mm": round(s.swept_mm.p50, 2),
                    "swept_p95_mm": round(s.swept_mm.p95, 2),
                    "lateral_p95_mm": round(s.lateral_mm.p95, 2),
                    "yaw_p95_deg": round(s.yaw_deg.p95, 2),
                    "mean_retries": round(s.retries.mean, 3),
                    "median_time_s": round(s.duration_s.p50, 2),
                }
            )


def two_d(config: EpisodeConfig, episodes: int, workers: int, rows: list) -> None:
    """Run the drift x blind-segment grid: the boundary that matters."""
    print("\nsuccess rate, odometry drift scale (rows) against blind segment in m (columns)")
    header = "| drift | " + " | ".join(f"{b:.2f} m" for b in GRID_BLIND) + " |"
    print(header)
    print("|---" * (len(GRID_BLIND) + 1) + "|")
    for drift in GRID_DRIFT:
        cells = []
        for blind in GRID_BLIND:
            cfg = with_blind_radius(with_drift_scale(config, drift), blind)
            results = run_campaign(cfg, episodes=episodes, seed0=20_000, workers=workers)
            summary = summarise(results, cfg.dock)
            cells.append(f"{100.0 * summary.success_rate:.0f}%")
            rows.append(
                {
                    "sweep": "drift_x_blind",
                    "value": drift,
                    "value2": blind,
                    "episodes": summary.episodes,
                    "success_rate": round(summary.success_rate, 4),
                    "contact_rate": round(summary.contact_rate, 4),
                    "swept_p50_mm": round(summary.swept_mm.p50, 2),
                    "swept_p95_mm": round(summary.swept_mm.p95, 2),
                    "lateral_p95_mm": round(summary.lateral_mm.p95, 2),
                    "yaw_p95_deg": round(summary.yaw_deg.p95, 2),
                    "mean_retries": round(summary.retries.mean, 3),
                    "median_time_s": round(summary.duration_s.p50, 2),
                }
            )
        print(f"| {drift:g} | " + " | ".join(cells) + " |")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--episodes", type=int, default=150, help="episodes per grid point")
    parser.add_argument("--grid-episodes", type=int, default=100, help="episodes per 2-D grid cell")
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--out", type=Path, default=Path(__file__).resolve().parents[1] / "docs" / "sweep.csv")
    parser.add_argument("--skip-2d", action="store_true")
    args = parser.parse_args()

    config = EpisodeConfig()
    rows: list = []
    started = time.time()
    one_d(config, args.episodes, args.workers, rows)
    if not args.skip_2d:
        two_d(config, args.grid_episodes, args.workers, rows)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nwrote {args.out} ({len(rows)} grid points) in {time.time() - started:.0f} s")


if __name__ == "__main__":
    main()
