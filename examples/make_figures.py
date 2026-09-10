#!/usr/bin/env python3
"""Regenerate every figure in docs/figures.

    python3 examples/sweep.py --workers 8          # writes docs/sweep.csv
    python3 examples/make_figures.py --workers 8

The sweep figures read docs/sweep.csv so that the plots and the numbers quoted
in the README come from the same run. Everything else is computed here.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from autodock import EpisodeConfig, run_episode  # noqa: E402
from autodock.montecarlo import run_campaign, with_drift_scale  # noqa: E402
from autodock.sim import _STATE_INDEX  # noqa: E402
from autodock.states import DockState  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
FIGURES = ROOT / "docs" / "figures"

INK = "#1b1f24"
MUTED = "#6b7480"
GRID = "#dde2e8"
BLUE = "#2b5d8a"
GREEN = "#2f6f4e"
RED = "#b23a48"
AMBER = "#c98a1b"
PANEL = "#f7f8fa"

plt.rcParams.update(
    {
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
        "axes.edgecolor": MUTED,
        "axes.labelcolor": INK,
        "axes.titlesize": 11,
        "axes.titleweight": "bold",
        "axes.labelsize": 9.5,
        "text.color": INK,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5,
        "legend.fontsize": 8.5,
        "legend.frameon": False,
        "grid.color": GRID,
        "grid.linewidth": 0.8,
        "font.size": 9.5,
        "figure.dpi": 150,
    }
)


def tidy(ax, grid: bool = True) -> None:
    """Strip the top and right spines and add a light grid."""
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    if grid:
        ax.grid(True, linestyle="-", alpha=0.7)
        ax.set_axisbelow(True)


def draw_dock(ax, config: EpisodeConfig, scale: float = 1.0) -> None:
    """Draw the dock body, funnel and contact plane in dock-frame coordinates."""
    dock = config.dock
    outer = dock.funnel_outer_half_width * scale
    throat = dock.lateral_tolerance * scale
    depth = 0.10 * scale
    mouth = 0.07 * scale
    ax.add_patch(Rectangle((0.0, -outer * 2.0), depth, outer * 4.0, facecolor="#e3e8ee", edgecolor=MUTED, lw=1.0, zorder=1))
    for sign in (1.0, -1.0):
        ax.add_patch(
            Polygon(
                [(-mouth, sign * outer * 2.0), (-mouth, sign * outer), (0.0, sign * throat), (0.0, sign * outer * 2.0)],
                closed=True,
                facecolor="#e3e8ee",
                edgecolor=MUTED,
                lw=1.0,
                zorder=1,
            )
        )
    ax.axvline(0.0, color=INK, lw=1.2, zorder=2)
    ax.axhline(0.0, color=BLUE, lw=1.0, linestyle=(0, (6, 4)), zorder=2)


def approach_geometry(config: EpisodeConfig, seed: int) -> None:
    """Figure 1: the geometry every guard is written in."""
    result = run_episode(EpisodeConfig(record=True), seed)
    traj = result.trajectory
    dock = config.dock
    policy = config.policy

    fig, (ax, axz) = plt.subplots(1, 2, figsize=(11.2, 4.4), gridspec_kw={"width_ratios": [1.35, 1.0]})

    for a in (ax, axz):
        draw_dock(a, config)

    x = -traj[:, 5]
    y = traj[:, 6]
    ax.plot(x, y, color=BLUE, lw=1.6, label="robot path", zorder=4)

    ax.axvline(-policy.standoff, color=AMBER, lw=1.1, linestyle=(0, (4, 3)))
    ax.axvline(-policy.commit_range, color=RED, lw=1.1, linestyle=(0, (4, 3)))
    ax.axvspan(-policy.commit_range, 0.0, color=RED, alpha=0.07, zorder=0)
    ax.annotate(
        "standoff\n(on the dock axis,\nderived from the dock pose)",
        xy=(-policy.standoff, 0.42),
        ha="center",
        fontsize=8,
        color=AMBER,
    )
    ax.annotate("commit gate", xy=(-policy.commit_range - 0.03, -0.52), ha="right", fontsize=8, color=RED)
    ax.annotate(
        f"blind: {policy.commit_range * 1000:.0f} mm\nno marker below "
        f"{config.marker.min_range * 1000:.0f} mm",
        xy=(-policy.commit_range / 2, -0.72),
        ha="center",
        fontsize=8,
        color=RED,
    )
    ax.plot([-policy.standoff], [0.0], marker="o", ms=6, color=AMBER, zorder=5)
    ax.annotate(
        "dock axis",
        xy=(-2.2, 0.03),
        fontsize=8,
        color=BLUE,
    )
    ax.plot([x[0]], [y[0]], marker="o", ms=6, mfc="white", mec=BLUE, mew=1.5, zorder=5)
    ax.annotate("start", xy=(x[0], y[0]), xytext=(8, 10), textcoords="offset points", fontsize=8, color=BLUE)
    ax.set_xlim(-3.0, 0.3)
    ax.set_ylim(-0.95, 0.95)
    ax.set_aspect("equal")
    ax.set_xlabel("along the dock axis [m]   (0 = contact plane)")
    ax.set_ylabel("lateral offset from the axis [m]")
    ax.set_title("Approach geometry, dock frame")
    tidy(ax)
    ax.legend(loc="upper left")

    # Close-up of the tolerance the robot has to land inside.
    mask = traj[:, 5] < 0.45
    axz.plot(-traj[mask, 5] * 1000, traj[mask, 6] * 1000, color=BLUE, lw=1.8, zorder=4)
    axz.axhspan(
        -dock.lateral_tolerance * 1000,
        dock.lateral_tolerance * 1000,
        color=GREEN,
        alpha=0.12,
        zorder=0,
    )
    axz.axhline(dock.lateral_tolerance * 1000, color=GREEN, lw=1.0)
    axz.axhline(-dock.lateral_tolerance * 1000, color=GREEN, lw=1.0)
    axz.axvline(-policy.commit_range * 1000, color=RED, lw=1.1, linestyle=(0, (4, 3)))
    if result.contact_error is not None:
        axz.plot([0.0], [result.contact_error.lateral * 1000], marker="o", ms=7, color=RED, zorder=6)
        axz.annotate(
            f"contact\n{result.contact_error.lateral * 1000:+.1f} mm, "
            f"{math.degrees(result.contact_error.yaw):+.2f} deg",
            xy=(0.0, result.contact_error.lateral * 1000),
            xytext=(-160, 34),
            fontsize=8,
            color=RED,
            arrowprops=dict(arrowstyle="-", color=RED, lw=0.8),
        )
    axz.annotate(
        f"mechanical tolerance +/-{dock.lateral_tolerance * 1000:.0f} mm swept",
        xy=(-430, dock.lateral_tolerance * 1000 + 3),
        fontsize=8,
        color=GREEN,
    )
    axz.set_xlim(-460, 40)
    axz.set_ylim(-60, 60)
    axz.set_xlabel("along the dock axis [mm]")
    axz.set_ylabel("lateral offset [mm]")
    axz.set_title("The last 450 mm")
    tidy(axz)

    fig.suptitle(
        f"Docking approach, seed {seed}: pure pursuit onto the dock axis, commit gate, then "
        f"{policy.commit_range * 1000:.0f} mm blind",
        fontsize=11,
        y=1.0,
    )
    fig.tight_layout()
    fig.savefig(FIGURES / "approach-geometry.png", bbox_inches="tight")
    plt.close(fig)


def trajectories(config: EpisodeConfig, episodes: int) -> None:
    """Figure 2: many randomised starts, nominal and worn, in the dock frame."""
    nominal = [run_episode(EpisodeConfig(record=True), seed) for seed in range(episodes)]
    worn_config = with_drift_scale(EpisodeConfig(record=True), 3.0)
    worn = [run_episode(worn_config, seed) for seed in range(episodes)]

    fig, (ax, axz, axw) = plt.subplots(1, 3, figsize=(14.0, 4.3), gridspec_kw={"width_ratios": [1.3, 1.0, 1.0]})
    for a in (ax, axz, axw):
        draw_dock(a, config)

    docked = sum(r.success for r in nominal)
    for result in nominal:
        traj = result.trajectory
        colour = GREEN if result.success else RED
        ax.plot(-traj[:, 5], traj[:, 6], color=colour, lw=0.8, alpha=0.55)
        ax.plot([-traj[0, 5]], [traj[0, 6]], marker=".", ms=4, color=colour, alpha=0.8)

    ax.set_xlim(-3.3, 0.3)
    ax.set_ylim(-1.5, 1.5)
    ax.set_aspect("equal")
    ax.set_xlabel("along the dock axis [m]")
    ax.set_ylabel("lateral offset [m]")
    ax.set_title(f"{episodes} randomised starts, nominal\n({docked} docked, {episodes - docked} not)")
    tidy(ax)

    for axis, results, title in [
        (axz, nominal, "Nominal drive train"),
        (axw, worn, "Drift scale 3"),
    ]:
        for result in results:
            traj = result.trajectory
            colour = GREEN if result.success else RED
            mask = traj[:, 5] < 0.45
            axis.plot(-traj[mask, 5] * 1000, traj[mask, 6] * 1000, color=colour, lw=0.8, alpha=0.5)
        axis.axhspan(
            -config.dock.lateral_tolerance * 1000,
            config.dock.lateral_tolerance * 1000,
            color=GREEN,
            alpha=0.12,
            zorder=0,
        )
        axis.axvline(-config.policy.commit_range * 1000, color=RED, lw=1.1, linestyle=(0, (4, 3)))
        axis.annotate("commit gate", xy=(-config.policy.commit_range * 1000 + 8, 62), fontsize=8, color=RED)
        failed = sum(1 for r in results if not r.success)
        axis.set_xlim(-460, 40)
        axis.set_ylim(-75, 75)
        axis.set_xlabel("along the dock axis [mm]")
        axis.set_ylabel("lateral offset [mm]")
        axis.set_title(f"The last 450 mm: {title}\n({failed} of {len(results)} outside tolerance)")
        tidy(axis)

    fig.suptitle(
        "Trajectories from randomised start poses, heading uniform over the full circle. "
        "Green: the episode ended docked. Red: it did not, and every attempt it made is drawn.",
        fontsize=11,
        y=1.02,
    )
    fig.tight_layout()
    fig.savefig(FIGURES / "trajectories.png", bbox_inches="tight")
    plt.close(fig)


def contact_error(config: EpisodeConfig, episodes: int, workers: int) -> None:
    """Figure 3: contact error against how badly the robot started."""
    results = run_campaign(config, episodes=episodes, seed0=0, workers=workers)
    worn = run_campaign(with_drift_scale(config, 3.0), episodes=episodes, seed0=0, workers=workers)
    dock = config.dock
    tol = dock.lateral_tolerance * 1000

    def points(rs):
        rs = [r for r in rs if r.contact_error is not None]
        lat0 = np.array([abs(r.start_error.lateral) for r in rs])
        yaw0 = np.array([abs(math.degrees(r.start_error.yaw)) for r in rs])
        swept = np.array([dock.swept_half_width(r.contact_error) * 1000 for r in rs])
        ok = np.array([r.success for r in rs])
        return lat0, yaw0, swept, ok

    fig, axes = plt.subplots(1, 3, figsize=(12.4, 3.9))
    for ax, (rs, label) in zip(axes[:2], [(worn, "lateral"), (worn, "heading")]):
        lat0, yaw0, swept, ok = points(rs)
        xs = lat0 if label == "lateral" else yaw0
        ax.scatter(xs[ok], swept[ok], s=11, color=GREEN, alpha=0.6, label="docked")
        if (~ok).any():
            ax.scatter(xs[~ok], swept[~ok], s=13, color=RED, alpha=0.75, label="no mate")
        ax.axhline(tol, color=INK, lw=1.1, linestyle=(0, (5, 3)))
        ax.annotate("mechanical tolerance", xy=(xs.max() * 0.02, tol + 1.2), fontsize=8, color=INK)
        ax.set_xlabel("initial lateral offset [m]" if label == "lateral" else "initial heading error [deg]")
        ax.set_ylabel("swept half-width at contact [mm]")
        ax.set_title(f"Contact error vs initial {label} (drift scale 3)")
        ax.set_ylim(0, max(tol * 1.6, float(swept.max()) * 1.05))
        tidy(ax)
    axes[0].legend(loc="upper right")

    ax = axes[2]
    _, _, swept_n, _ = points(results)
    _, _, swept_w, _ = points(worn)
    bins = np.linspace(0, max(60.0, float(np.percentile(swept_w, 99))), 34)
    ax.hist(swept_n, bins=bins, color=BLUE, alpha=0.65, label="nominal drive train")
    ax.hist(swept_w, bins=bins, color=AMBER, alpha=0.6, label="drift scale 3")
    ax.axvline(tol, color=INK, lw=1.1, linestyle=(0, (5, 3)))
    ax.annotate("tolerance", xy=(tol + 1.5, ax.get_ylim()[1] * 0.55), fontsize=8, color=INK)
    ax.set_xlabel("swept half-width at contact [mm]")
    ax.set_ylabel("episodes")
    ax.set_title("It is the tail that fails, not the median")
    ax.legend(loc="upper right")
    tidy(ax)

    fig.suptitle(
        f"Contact error over {episodes} episodes: where the robot started barely matters, "
        "how well its wheels are matched does",
        fontsize=11,
        y=1.02,
    )
    fig.tight_layout()
    fig.savefig(FIGURES / "contact-error.png", bbox_inches="tight")
    plt.close(fig)


def load_sweep(path: Path) -> dict:
    """Read docs/sweep.csv into {sweep: [rows]}."""
    if not path.exists():
        raise SystemExit(f"{path} not found. Run:  python3 examples/sweep.py --workers 8")
    grouped = defaultdict(list)
    with path.open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            grouped[row["sweep"]].append(row)
    return grouped


def success_sweep(grouped: dict) -> None:
    """Figure 4: success rate along each single axis."""
    panels = [
        ("drift", "odometry drift scale (x nominal wheel mismatch)", "Odometry drift"),
        ("blind", "blind segment [m]", "Blind-segment length"),
        ("misalignment", "initial lateral spread (x nominal)", "Initial misalignment"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(12.4, 3.8), sharey=True)
    for ax, (key, xlabel, title) in zip(axes, panels):
        rows = sorted(grouped[key], key=lambda r: float(r["value"]))
        xs = [float(r["value"]) for r in rows]
        ys = [100.0 * float(r["success_rate"]) for r in rows]
        swept = [float(r["swept_p95_mm"]) for r in rows]
        ax.plot(xs, ys, marker="o", ms=5, color=BLUE, lw=1.8)
        ax.axhline(95.0, color=MUTED, lw=0.9, linestyle=(0, (4, 3)))
        ax.set_xlabel(xlabel)
        ax.set_title(title)
        ax.set_ylim(0, 104)
        tidy(ax)
        twin = ax.twinx()
        twin.plot(xs, swept, marker="s", ms=4, color=AMBER, lw=1.2, alpha=0.85)
        twin.set_ylim(0, max(100.0, max(swept) * 1.1))
        twin.spines["top"].set_visible(False)
        twin.tick_params(axis="y", colors=AMBER, labelsize=8)
        if ax is axes[-1]:
            twin.set_ylabel("95th pct swept error [mm]", color=AMBER, fontsize=9)
    axes[0].set_ylabel("docked and charging [%]")
    axes[0].annotate("95%", xy=(min(float(r["value"]) for r in grouped["drift"]), 96.5), fontsize=8, color=MUTED)
    n = grouped["drift"][0]["episodes"]
    fig.suptitle(
        f"Success rate along each axis ({n} episodes per point). Blue: success rate. "
        "Amber: 95th percentile contact error.",
        fontsize=11,
        y=1.03,
    )
    fig.tight_layout()
    fig.savefig(FIGURES / "success-sweep.png", bbox_inches="tight")
    plt.close(fig)


def success_boundary(grouped: dict) -> None:
    """Figure 5: the two axes that matter, together."""
    rows = grouped["drift_x_blind"]
    drifts = sorted({float(r["value"]) for r in rows})
    blinds = sorted({float(r["value2"]) for r in rows})
    grid = np.full((len(drifts), len(blinds)), np.nan)
    for row in rows:
        i = drifts.index(float(row["value"]))
        j = blinds.index(float(row["value2"]))
        grid[i, j] = 100.0 * float(row["success_rate"])

    fig, ax = plt.subplots(figsize=(7.4, 4.6))
    im = ax.imshow(grid, origin="lower", cmap="RdYlGn", vmin=0, vmax=100, aspect="auto")
    ax.set_xticks(range(len(blinds)), [f"{b:.2f}" for b in blinds])
    ax.set_yticks(range(len(drifts)), [f"{d:g}" for d in drifts])
    ax.set_xlabel("blind segment [m]")
    ax.set_ylabel("odometry drift scale")
    for i in range(len(drifts)):
        for j in range(len(blinds)):
            value = grid[i, j]
            ax.text(
                j,
                i,
                f"{value:.0f}%",
                ha="center",
                va="center",
                fontsize=9,
                color=INK if 25 < value < 92 else "white",
                fontweight="bold",
            )
    ax.contour(grid, levels=[95.0], colors=[INK], linewidths=1.6)
    cbar = fig.colorbar(im, ax=ax, pad=0.02)
    cbar.set_label("docked and charging [%]", fontsize=9)
    cbar.outline.set_visible(False)
    n = rows[0]["episodes"]
    ax.set_title(
        f"Success boundary ({n} episodes per cell); the black contour is 95%",
        fontsize=11,
    )
    fig.tight_layout()
    fig.savefig(FIGURES / "success-boundary.png", bbox_inches="tight")
    plt.close(fig)


def _detection_gaps(visible: np.ndarray, min_len: int) -> np.ndarray:
    """Mark runs of at least ``min_len`` consecutive frames with no detection."""
    out = np.zeros_like(visible, dtype=bool)
    start = None
    for i, seen in enumerate(visible):
        if not seen and start is None:
            start = i
        elif seen and start is not None:
            if i - start >= min_len:
                out[start:i] = True
            start = None
    if start is not None and len(visible) - start >= min_len:
        out[start:] = True
    return out


def state_timeline(config: EpisodeConfig, seed: int) -> None:
    """Figure 6: what the state machine actually did on a run that had to retry."""
    recorded = with_drift_scale(EpisodeConfig(record=True), 4.0)  # a badly matched drive train
    result = run_episode(recorded, seed)
    traj = result.trajectory
    order = [s for s in DockState]
    fig, (ax, axr) = plt.subplots(2, 1, figsize=(11.0, 5.4), sharex=True, gridspec_kw={"height_ratios": [1.5, 1.0]})

    codes = traj[:, 4].astype(int)
    ax.step(traj[:, 0], codes, where="post", color=BLUE, lw=1.6)
    ax.set_yticks(range(len(order)), [s.value for s in order])
    ax.set_ylabel("state")
    ax.set_title(
        f"State machine timeline, seed {seed}, drift scale 4 -- "
        f"{result.retries} retry, outcome: {result.outcome}"
    )
    tidy(ax, grid=False)
    ax.grid(True, axis="y", linestyle="-", alpha=0.5)
    for t, _, guard, target in result.state_history:
        ax.annotate(
            guard,
            xy=(t, _STATE_INDEX[DockState(target)]),
            xytext=(2, 5),
            textcoords="offset points",
            fontsize=6.5,
            color=MUTED,
            rotation=25,
        )

    axr.plot(traj[:, 0], traj[:, 5], color=INK, lw=1.4, label="range to contact plane [m]")
    axr.plot(traj[:, 0], traj[:, 6], color=RED, lw=1.2, label="lateral offset [m]")
    gap = _detection_gaps(traj[:, 10] > 0.5, min_len=8)
    axr.fill_between(
        traj[:, 0], -1.4, 3.4, where=gap, color=AMBER, alpha=0.16, step="mid",
        label="no detection for >0.3 s",
    )
    axr.set_ylim(-1.4, 3.4)
    axr.set_xlabel("time [s]")
    axr.set_ylabel("dock-frame error")
    axr.legend(loc="upper right", ncol=3)
    tidy(axr)

    fig.tight_layout()
    fig.savefig(FIGURES / "state-timeline.png", bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--episodes", type=int, default=300, help="episodes for the contact-error figure")
    parser.add_argument("--trajectories", type=int, default=60)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--retry-seed", type=int, default=13)
    parser.add_argument("--sweep-csv", type=Path, default=ROOT / "docs" / "sweep.csv")
    args = parser.parse_args()

    FIGURES.mkdir(parents=True, exist_ok=True)
    config = EpisodeConfig()

    print("approach-geometry.png")
    approach_geometry(config, args.seed)
    print("trajectories.png")
    trajectories(config, args.trajectories)
    print("contact-error.png")
    contact_error(config, args.episodes, args.workers)
    print("state-timeline.png")
    state_timeline(config, args.retry_seed)

    grouped = load_sweep(args.sweep_csv)
    print("success-sweep.png")
    success_sweep(grouped)
    print("success-boundary.png")
    success_boundary(grouped)
    print(f"figures written to {FIGURES}")
    print("regenerate with:")
    print("  python3 examples/sweep.py --workers 8")
    print(f"  python3 examples/make_figures.py --workers {args.workers}")


if __name__ == "__main__":
    main()
