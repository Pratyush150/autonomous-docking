"""Monte Carlo campaigns and the parameter sweeps that find the failure boundary.

A campaign fixes the design and randomises the world: start pose, drive-train
asymmetry, detection dropouts, pose noise. A sweep runs one campaign per grid
point along a parameter axis. The sweep is the point of the exercise -- a
single success rate is a number, but the curve of success rate against
odometry drift and blind-segment length is a design rule you can act on.

Every episode is keyed by an integer seed, so any run is reproducible and any
interesting failure can be replayed alone with ``run_episode(config, seed)``.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Callable, Dict, List, Optional, Sequence

from .metrics import CampaignSummary, summarise
from .sim import EpisodeConfig, EpisodeResult, run_episode

__all__ = [
    "run_campaign",
    "run_sweep",
    "SweepPoint",
    "with_drift_scale",
    "with_blind_radius",
    "with_misalignment",
]


def run_campaign(
    config: EpisodeConfig,
    episodes: int = 400,
    seed0: int = 0,
    workers: int = 1,
    progress: Optional[Callable[[int, int], None]] = None,
) -> List[EpisodeResult]:
    """Run ``episodes`` episodes with seeds ``seed0 .. seed0+episodes-1``.

    ``workers > 1`` spreads the episodes over processes. Results are identical
    either way: the seed, not the scheduling, decides the episode.
    """
    seeds = list(range(seed0, seed0 + episodes))
    if workers > 1:
        from concurrent.futures import ProcessPoolExecutor

        with ProcessPoolExecutor(max_workers=workers) as pool:
            out = list(pool.map(_run_one, [(config, s) for s in seeds], chunksize=8))
        if progress is not None:
            progress(episodes, episodes)
        return out

    out = []
    for i, seed in enumerate(seeds, start=1):
        out.append(run_episode(config, seed))
        if progress is not None and (i % 25 == 0 or i == episodes):
            progress(i, episodes)
    return out


def _run_one(args) -> EpisodeResult:
    config, seed = args
    return run_episode(config, seed)


def with_drift_scale(config: EpisodeConfig, scale: float) -> EpisodeConfig:
    """Same design, worse (or better) calibrated drive train."""
    return replace(config, drive=replace(config.drive, drift_scale=scale))


def with_blind_radius(config: EpisodeConfig, min_range: float) -> EpisodeConfig:
    """Same design, marker lost at a different range.

    This is the camera mounting decision: a marker that stays visible to 0.15 m
    needs a low, forward camera; one lost at 0.9 m is what you get when the
    camera sits on top of a mast. The commit range moves with it, because the
    robot has to make its go/no-go call at the last range where it can still
    see anything, and the standoff is pushed out if it would otherwise fall
    inside the commit range.
    """
    commit = min_range + config.policy.commit_margin
    standoff = max(config.policy.standoff, commit + 0.30)
    return replace(
        config,
        marker=replace(config.marker, min_range=min_range),
        policy=replace(config.policy, commit_range=commit, standoff=standoff),
    )


def with_misalignment(config: EpisodeConfig, misalignment: float) -> EpisodeConfig:
    """Same design, robot dropped off worse-placed by the navigation stack."""
    return replace(config, start=replace(config.start, misalignment=misalignment))


@dataclass(frozen=True)
class SweepPoint:
    """One grid point: the axis value and what the campaign there measured."""

    axis: str
    value: float
    summary: CampaignSummary

    @property
    def success_rate(self) -> float:
        """Fraction of episodes that ended docked and charging."""
        return self.summary.success_rate


def run_sweep(
    config: EpisodeConfig,
    axis: str,
    values: Sequence[float],
    episodes: int = 150,
    seed0: int = 10_000,
    workers: int = 1,
    progress: Optional[Callable[[str, float, CampaignSummary], None]] = None,
) -> List[SweepPoint]:
    """Run one campaign per value along ``axis``.

    ``axis`` is one of ``"drift"``, ``"blind"`` or ``"misalignment"``. The same
    seed block is reused at every grid point so that the difference between two
    points is the parameter, not the draw.
    """
    makers: Dict[str, Callable[[EpisodeConfig, float], EpisodeConfig]] = {
        "drift": with_drift_scale,
        "blind": with_blind_radius,
        "misalignment": with_misalignment,
    }
    if axis not in makers:
        raise KeyError(f"unknown sweep axis {axis!r}; expected one of {sorted(makers)}")

    points: List[SweepPoint] = []
    for value in values:
        cfg = makers[axis](config, float(value))
        results = run_campaign(cfg, episodes=episodes, seed0=seed0, workers=workers)
        summary = summarise(results, cfg.dock)
        if progress is not None:
            progress(axis, float(value), summary)
        points.append(SweepPoint(axis, float(value), summary))
    return points
