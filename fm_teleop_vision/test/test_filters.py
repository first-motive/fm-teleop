"""One-Euro filter — pure math, no ROS, no camera, no model."""

import math

import pytest

from fm_teleop_vision.filters import OneEuroFilter, Vec3OneEuro


def _stddev(values):
    mean = sum(values) / len(values)
    return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))


def _noisy_constant(n, amplitude):
    """A constant signal (0.0) plus a deterministic, mean-near-zero wobble."""
    return [amplitude * math.sin(i) * math.cos(i * 0.5) for i in range(n)]


def test_filter_reduces_jitter_on_a_noisy_constant():
    # The whole point of the filter: on a held-still signal it cuts the spread.
    raw = _noisy_constant(200, amplitude=0.05)
    f = OneEuroFilter(min_cutoff=0.5, beta=0.0)
    filtered = [f(x, dt=1.0 / 30.0) for x in raw]
    # Drop the seed transient before comparing the steady-state spread.
    assert _stddev(filtered[20:]) < _stddev(raw[20:])


def test_filter_tracks_a_step():
    # A sustained move must still arrive — smoothing adds lag, not a permanent offset.
    f = OneEuroFilter(min_cutoff=1.0, beta=0.02)
    out = 0.0
    for _ in range(200):
        out = f(1.0, dt=1.0 / 30.0)
    assert out == pytest.approx(1.0, abs=1e-2)


def test_vec3_filters_each_axis_independently():
    f = Vec3OneEuro(min_cutoff=1.0, beta=0.02)
    out = None
    for _ in range(200):
        out = f([1.0, -2.0, 0.5], dt=1.0 / 30.0)
    assert out[0] == pytest.approx(1.0, abs=1e-2)
    assert out[1] == pytest.approx(-2.0, abs=1e-2)
    assert out[2] == pytest.approx(0.5, abs=1e-2)


def test_reset_clears_history():
    f = OneEuroFilter()
    for _ in range(10):
        f(5.0, dt=1.0 / 30.0)
    f.reset()
    # After reset the next sample seeds afresh, returning itself.
    assert f(2.0, dt=1.0 / 30.0) == 2.0
