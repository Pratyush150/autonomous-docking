"""Aggregation: percentiles, conditional rates, and what gets counted."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import pytest

from autodock.geometry import DockError, DockGeometry
from autodock.metrics import _spread, summarise

DOCK = DockGeometry()


@dataclass
class FakeResult:
    success: bool
    outcome: str
    duration: float
    retries: int
    contact_error: Optional[DockError]
    longitudinal_offset: float = 0.0
    dead_reckon_error: float = 0.0
    blind_distance: float = 0.30


def make(n: int, success: bool, lateral: float, contact: bool = True) -> list:
    return [
        FakeResult(
            success=success,
            outcome="docked" if success else "abort:attempts_exhausted",
            duration=20.0 + i,
            retries=0 if success else 2,
            contact_error=DockError(0.0, lateral, 0.0) if contact else None,
        )
        for i in range(n)
    ]


def test_spread_reports_the_requested_percentiles():
    s = _spread(list(range(101)))
    assert s.p5 == pytest.approx(5.0)
    assert s.p50 == pytest.approx(50.0)
    assert s.p95 == pytest.approx(95.0)
    assert s.mean == pytest.approx(50.0)


def test_spread_of_nothing_is_nan_rather_than_a_crash():
    s = _spread([])
    assert math.isnan(s.p50)


def test_rates_are_computed_over_the_right_denominators():
    results = make(8, True, 0.001) + make(2, False, 0.05) + make(2, False, 0.0, contact=False)
    summary = summarise(results, DOCK)
    assert summary.episodes == 12
    assert summary.successes == 8
    assert summary.contacts == 10
    assert summary.success_rate == pytest.approx(8 / 12)
    assert summary.contact_rate == pytest.approx(10 / 12)
    # Mating rate is conditioned on having touched the dock at all.
    assert summary.mate_rate_given_contact == pytest.approx(8 / 10)


def test_contact_statistics_exclude_episodes_that_never_touched_the_dock():
    """Counting a clean miss as zero error would flatter the numbers."""
    with_misses = make(4, True, 0.004) + make(4, False, 0.0, contact=False)
    summary = summarise(with_misses, DOCK)
    assert summary.lateral_mm.p50 == pytest.approx(4.0)


def test_units_are_millimetres_and_degrees():
    results = [FakeResult(True, "docked", 20.0, 0, DockError(0.0, 0.012, math.radians(2.0)))]
    summary = summarise(results, DOCK)
    assert summary.lateral_mm.p50 == pytest.approx(12.0)
    assert summary.yaw_deg.p50 == pytest.approx(2.0)
    expected = DOCK.swept_half_width(DockError(0.0, 0.012, math.radians(2.0))) * 1000.0
    assert summary.swept_mm.p50 == pytest.approx(expected)


def test_duration_is_reported_over_successful_episodes_when_there_are_any():
    results = make(3, True, 0.001)
    for r in make(3, False, 0.05):
        r.duration = 150.0
        results.append(r)
    summary = summarise(results, DOCK)
    assert summary.duration_s.p95 < 100.0


def test_outcome_and_retry_histograms_are_reported():
    results = make(3, True, 0.001) + make(2, False, 0.05)
    summary = summarise(results, DOCK)
    assert summary.outcomes == {"abort:attempts_exhausted": 2, "docked": 3}
    assert summary.retry_histogram == {0: 3, 2: 2}


def test_summarising_nothing_is_an_error():
    with pytest.raises(ValueError):
        summarise([])


def test_table_and_headline_render():
    summary = summarise(make(5, True, 0.003), DOCK)
    table = summary.table()
    assert table.count("\n") == 9
    assert "median" in table
    assert "success 100.0%" in summary.headline()
