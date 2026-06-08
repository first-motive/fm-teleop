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


def test_displacement_to_twist_zero_at_neutral():
    # At the neutral pose the command is exactly zero on every axis.
    out = retarget.displacement_to_twist(
        [1.0, 2.0, 3.0], [1.0, 2.0, 3.0], factor=4.0, threshold=0.03, limit=1.0
    )
    assert out == [0.0, 0.0, 0.0]


def test_displacement_to_twist_deadzone_kills_rest_jitter():
    # A small wobble within the deadzone of neutral produces no command.
    out = retarget.displacement_to_twist(
        [0.01, -0.02, 0.0], [0.0, 0.0, 0.0], factor=10.0, threshold=0.03, limit=1.0
    )
    assert out == [0.0, 0.0, 0.0]


def test_displacement_to_twist_scales_past_deadzone():
    # A 0.1 m displacement past the deadzone scales linearly (no clamp hit here).
    out = retarget.displacement_to_twist(
        [0.1, 0.0, 0.0], [0.0, 0.0, 0.0], factor=5.0, threshold=0.03, limit=10.0
    )
    assert out[0] == pytest.approx(0.5)
    assert out[1] == 0.0 and out[2] == 0.0


def test_displacement_to_twist_clamps_each_axis():
    # A large displacement is bounded to +/- limit symmetrically.
    out = retarget.displacement_to_twist(
        [1.0, -1.0, 1.0], [0.0, 0.0, 0.0], factor=100.0, threshold=0.03, limit=0.8
    )
    assert out == [0.8, -0.8, 0.8]


def test_displacement_to_twist_rejects_non_3_vector():
    with pytest.raises(ValueError):
        retarget.displacement_to_twist(
            [0.0, 0.0], [0.0, 0.0], factor=1.0, threshold=0.0, limit=1.0
        )
