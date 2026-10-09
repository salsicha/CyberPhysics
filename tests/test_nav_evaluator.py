import math
from collections import deque

import pytest

import synthetic_world as sw
from conftest import install_ros_stubs

install_ros_stubs()
from wildnav_pkg.nav_evaluator import NavEvaluator, SourceStats  # noqa: E402

ORIGIN = (37.9234, -122.5967)


def test_source_stats_summary():
    stats = SourceStats()
    for error in (1.0, 2.0, 3.0, 4.0, 10.0):
        stats.add(error)
    summary = stats.summary()
    assert summary['fixes'] == 5
    assert summary['max_m'] == 10.0
    assert summary['mean_m'] == 4.0
    assert summary['rmse_m'] == pytest.approx(
        math.sqrt((1 + 4 + 9 + 16 + 100) / 5), abs=0.01)
    assert summary['p95_m'] == 10.0


def test_source_stats_empty():
    assert SourceStats().summary() == {'fixes': 0}


def make_evaluator():
    evaluator = NavEvaluator.__new__(NavEvaluator)
    evaluator.truth = deque(maxlen=100)
    evaluator.tolerance_ns = int(0.5e9)
    return evaluator


def test_error_vs_truth_matches_nearest_sample():
    evaluator = make_evaluator()
    evaluator.truth.append((1_000_000_000, ORIGIN[0], ORIGIN[1]))
    evaluator.truth.append((2_000_000_000, ORIGIN[0] + 0.001, ORIGIN[1]))
    # Estimate 100 m east of the first truth sample, near its timestamp.
    lat, lon = sw.local_to_latlon(100.0, 0.0, *ORIGIN)
    error = evaluator._error_vs_truth(1_100_000_000, lat, lon)
    assert error == pytest.approx(100.0, abs=0.01)


def test_error_vs_truth_rejects_stale_timestamps():
    evaluator = make_evaluator()
    evaluator.truth.append((1_000_000_000, ORIGIN[0], ORIGIN[1]))
    assert evaluator._error_vs_truth(5_000_000_000, *ORIGIN) is None


def test_error_vs_truth_without_truth():
    assert make_evaluator()._error_vs_truth(0, *ORIGIN) is None


def test_error_vs_truth_ignores_non_finite_estimates():
    evaluator = make_evaluator()
    evaluator.truth.append((1_000_000_000, ORIGIN[0], ORIGIN[1]))
    assert evaluator._error_vs_truth(1_000_000_000, float('nan'), ORIGIN[1]) is None


def test_fused_estimate_is_scored_from_its_fix_not_a_local_frame():
    """The fused fix carries latitude/longitude, so scoring needs no origin
    (fused odometry is about the EKF origin, not INITIAL_LAT/LON)."""
    from types import SimpleNamespace as NS
    evaluator = make_evaluator()
    evaluator.stats = {'fused': SourceStats()}
    evaluator.last_fused_ns = 0
    evaluator.get_parameter = lambda name: NS(value=1.0)
    evaluator.truth.append((5_000_000_000, ORIGIN[0], ORIGIN[1]))
    lat, lon = sw.local_to_latlon(30.0, 40.0, *ORIGIN)
    evaluator._on_fused(NS(header=NS(stamp=NS(sec=5, nanosec=0)), latitude=lat, longitude=lon))
    assert evaluator.stats['fused'].summary()['max_m'] == pytest.approx(50.0, abs=0.01)
