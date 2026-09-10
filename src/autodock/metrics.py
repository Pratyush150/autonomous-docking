"""Turning a pile of episodes into the numbers that decide whether it works.

A single docking run tells you nothing. The distribution over randomised start
poses and disturbance realisations tells you everything, and the tail matters
more than the median: a dock that works at the median and fails at the 95th
percentile fails several times a day.

All distance figures are reported in millimetres and all angles in degrees,
because that is the scale at which the mechanical tolerance is written.
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence

import numpy as np

__all__ = ["Spread", "CampaignSummary", "summarise"]


@dataclass(frozen=True)
class Spread:
    """Median with a 5th/95th percentile band."""

    p5: float
    p50: float
    p95: float
    mean: float

    def __str__(self) -> str:
        return f"{self.p50:.2f} [{self.p5:.2f}, {self.p95:.2f}]"


def _spread(values: Sequence[float]) -> Spread:
    arr = np.asarray(list(values), dtype=float)
    if arr.size == 0:
        nan = float("nan")
        return Spread(nan, nan, nan, nan)
    p5, p50, p95 = np.percentile(arr, [5.0, 50.0, 95.0])
    return Spread(float(p5), float(p50), float(p95), float(arr.mean()))


@dataclass(frozen=True)
class CampaignSummary:
    """Aggregate of a Monte Carlo campaign."""

    episodes: int
    successes: int
    contacts: int
    success_rate: float
    contact_rate: float
    mate_rate_given_contact: float
    lateral_mm: Spread
    yaw_deg: Spread
    swept_mm: Spread
    longitudinal_mm: Spread
    dead_reckon_mm: Spread
    blind_distance_mm: Spread
    duration_s: Spread
    retries: Spread
    retry_histogram: Dict[int, int]
    outcomes: Dict[str, int]

    def table(self) -> str:
        """Markdown table of the headline distributions."""
        rows = [
            ("|lateral| at contact (mm)", self.lateral_mm),
            ("|heading| at contact (deg)", self.yaw_deg),
            ("swept half-width at contact (mm)", self.swept_mm),
            ("longitudinal offset at stop (mm)", self.longitudinal_mm),
            ("dead-reckoning range error (mm)", self.dead_reckon_mm),
            ("blind segment driven (mm)", self.blind_distance_mm),
            ("time to dock (s)", self.duration_s),
            ("retries used", self.retries),
        ]
        out = ["| metric | 5th | median | 95th |", "|---|---|---|---|"]
        for name, spread in rows:
            out.append(f"| {name} | {spread.p5:.2f} | {spread.p50:.2f} | {spread.p95:.2f} |")
        return "\n".join(out)

    def headline(self) -> str:
        """One-line summary."""
        return (
            f"{self.episodes} episodes, success {100.0 * self.success_rate:.1f}%, "
            f"contact {100.0 * self.contact_rate:.1f}%, "
            f"mate|contact {100.0 * self.mate_rate_given_contact:.1f}%, "
            f"median {self.duration_s.p50:.1f} s"
        )


def summarise(results: Iterable, dock=None) -> CampaignSummary:
    """Aggregate episodes into a :class:`CampaignSummary`.

    Contact-pose statistics are taken over the episodes that actually touched
    the dock on their last attempt. Episodes that drove past it have no contact
    pose, and averaging them in as zero error would flatter the result.
    """
    results = list(results)
    if not results:
        raise ValueError("no episodes to summarise")
    if dock is None:
        from .geometry import DockGeometry

        dock = DockGeometry()

    contacts = [r for r in results if r.contact_error is not None]
    successes = [r for r in results if r.success]
    lateral = [abs(r.contact_error.lateral) * 1000.0 for r in contacts]
    yaw = [abs(math.degrees(r.contact_error.yaw)) for r in contacts]
    swept = [dock.swept_half_width(r.contact_error) * 1000.0 for r in contacts]
    longitudinal = [r.longitudinal_offset * 1000.0 for r in results]
    dr = [r.dead_reckon_error * 1000.0 for r in results]
    blind = [r.blind_distance * 1000.0 for r in results if r.blind_distance > 0.0]
    duration = [r.duration for r in successes] if successes else [r.duration for r in results]
    retries: List[float] = [float(r.retries) for r in results]

    return CampaignSummary(
        episodes=len(results),
        successes=len(successes),
        contacts=len(contacts),
        success_rate=len(successes) / len(results),
        contact_rate=len(contacts) / len(results),
        mate_rate_given_contact=(len(successes) / len(contacts)) if contacts else float("nan"),
        lateral_mm=_spread(lateral),
        yaw_deg=_spread(yaw),
        swept_mm=_spread(swept),
        longitudinal_mm=_spread(longitudinal),
        dead_reckon_mm=_spread(dr),
        blind_distance_mm=_spread(blind),
        duration_s=_spread(duration),
        retries=_spread(retries),
        retry_histogram=dict(sorted(Counter(int(r.retries) for r in results).items())),
        outcomes=dict(sorted(Counter(r.outcome for r in results).items())),
    )
