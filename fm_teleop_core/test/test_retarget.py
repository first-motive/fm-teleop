"""Retarget math — pure, no ROS graph."""

import pytest

from fm_teleop_core import retarget


def test_deadzone_zeros_small_readings():
    assert retarget.deadzone(0.05, 0.1) == 0.0
    assert retarget.deadzone(-0.05, 0.1) == 0.0


def test_deadzone_passes_through_large_readings():
    assert retarget.deadzone(0.5, 0.1) == 0.5
    assert retarget.deadzone(-0.5, 0.1) == -0.5


def test_clamp_bounds_scalar():
    assert retarget.clamp(5.0, 0.0, 1.0) == 1.0
    assert retarget.clamp(-5.0, 0.0, 1.0) == 0.0
    assert retarget.clamp(0.5, 0.0, 1.0) == 0.5


def test_clamp_vector_bounds_each_value():
    out = retarget.clamp_vector([2.0, -2.0], [(0.0, 1.0), (-1.0, 0.0)])
    assert out == [1.0, -1.0]


def test_clamp_vector_rejects_length_mismatch():
    with pytest.raises(ValueError):
        retarget.clamp_vector([1.0, 2.0], [(0.0, 1.0)])


def test_scale_multiplies():
    assert retarget.scale(0.5, 2.0) == 1.0
