"""Leader-to-follower retarget math: joint selection and the per-period ramp bound."""

import math

import pytest

from fm_teleop_leader.follow import limit_step, select_positions
from fm_teleop_leader.sts3215 import CENTRE_STEP, steps_to_radians, vector_to_radians


def test_select_positions_reorders_to_follower_order():
    names = ["elbow_flex", "shoulder_pan", "wrist_roll"]
    positions = [0.3, 0.1, 0.5]
    assert select_positions(names, positions, ["shoulder_pan", "elbow_flex"]) == [0.1, 0.3]


def test_select_positions_drops_a_sample_missing_a_joint():
    # A partial vector would command the absent joint to a stale value, so None means drop.
    assert select_positions(["shoulder_pan"], [0.1], ["shoulder_pan", "elbow_flex"]) is None


def test_limit_step_passes_the_first_sample_through():
    assert limit_step(None, [1.0, -2.0], 0.05) == [1.0, -2.0]


def test_limit_step_bounds_travel_per_period():
    assert limit_step([0.0, 0.0], [1.0, -1.0], 0.05) == [0.05, -0.05]


def test_limit_step_moves_the_whole_way_inside_the_bound():
    assert limit_step([0.0], [0.02], 0.05) == [0.02]


def test_limit_step_disabled_by_a_non_positive_bound():
    assert limit_step([0.0], [9.0], 0.0) == [9.0]


def test_limit_step_rejects_a_length_mismatch():
    with pytest.raises(ValueError):
        limit_step([0.0], [1.0, 2.0], 0.05)


def test_steps_to_radians_centres_on_the_servo_midpoint():
    assert steps_to_radians(CENTRE_STEP) == 0.0
    assert steps_to_radians(CENTRE_STEP + 1024) == pytest.approx(math.pi / 2)


def test_steps_to_radians_applies_offset_and_sign():
    assert steps_to_radians(CENTRE_STEP + 1024, offset_steps=1024) == 0.0
    assert steps_to_radians(CENTRE_STEP + 1024, sign=-1) == pytest.approx(-math.pi / 2)


def test_vector_to_radians_rejects_a_length_mismatch():
    with pytest.raises(ValueError):
        vector_to_radians([CENTRE_STEP, CENTRE_STEP], offsets=[0])
