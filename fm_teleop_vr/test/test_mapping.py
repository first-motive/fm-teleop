"""VR mapping math: stick deadzone/scaling and the hysteretic grip preset."""

import pytest

from fm_teleop_vr.mapping import CLOSE, OPEN, grip_preset, stick_to_base_twist


def test_stick_drift_inside_the_deadzone_does_not_move_the_base():
    assert stick_to_base_twist(0.1, -0.1, 0.15, 0.3, 0.8) == (0.0, 0.0)


def test_stick_scales_to_the_speed_limits():
    linear, angular = stick_to_base_twist(1.0, -1.0, 0.15, 0.3, 0.8)
    assert linear == pytest.approx(0.3)
    assert angular == pytest.approx(-0.8)


def test_stick_is_clamped_to_the_speed_beyond_full_deflection():
    linear, _ = stick_to_base_twist(2.0, 0.0, 0.15, 0.3, 0.8)
    assert linear == pytest.approx(0.3)


def test_grip_closes_above_and_opens_below_the_thresholds():
    assert grip_preset(0.9, None, 0.35, 0.65) == CLOSE
    assert grip_preset(0.1, CLOSE, 0.35, 0.65) == OPEN


def test_grip_holds_between_the_thresholds():
    # A single threshold would chatter here; the current state is the tiebreak.
    assert grip_preset(0.5, CLOSE, 0.35, 0.65) == CLOSE
    assert grip_preset(0.5, OPEN, 0.35, 0.65) == OPEN


def test_grip_rejects_inverted_thresholds():
    with pytest.raises(ValueError):
        grip_preset(0.5, None, 0.65, 0.35)
