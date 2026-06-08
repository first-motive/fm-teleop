"""Dex3 hand preset + slider mapping contract."""

import pytest

from fm_teleop_device import hand_presets


def test_joints_seven_per_hand_in_dex3_order():
    for side in hand_presets.SIDES:
        names = hand_presets.joints(side)
        assert len(names) == 7
        assert names[0].endswith("thumb_0_joint")
        assert names[2].endswith("thumb_2_joint")
        assert names[6].endswith("index_1_joint")
    # The two hands name distinct joints.
    assert hand_presets.joints("left") != hand_presets.joints("right")


@pytest.mark.parametrize("side", hand_presets.SIDES)
@pytest.mark.parametrize("name", ["open", "close", "pinch"])
def test_every_preset_value_within_urdf_limits(side, name):
    # Guards against a sign or copy-paste error in a mirrored preset.
    targets = hand_presets.preset_targets(side, name)
    limits = hand_presets.LIMITS[side]
    assert len(targets) == 7
    for value, (lo, hi) in zip(targets, limits):
        assert lo <= value <= hi


@pytest.mark.parametrize("side", hand_presets.SIDES)
def test_open_is_neutral(side):
    assert hand_presets.preset_targets(side, "open") == [0.0] * 7


def test_close_is_mirrored_between_hands():
    # The middle/index joints flex to opposite signs on the two hands.
    left = hand_presets.preset_targets("left", "close")
    right = hand_presets.preset_targets("right", "close")
    # middle_0 (index 3): left flexes negative, right positive.
    assert left[3] < 0 and right[3] > 0
    assert left[4] < 0 and right[4] > 0


def test_pinch_leaves_middle_open():
    for side in hand_presets.SIDES:
        targets = hand_presets.preset_targets(side, "pinch")
        assert targets[3] == 0.0  # middle_0
        assert targets[4] == 0.0  # middle_1
        assert targets[2] != 0.0  # thumb_2 curled


def test_clamp_passthrough_within_limits():
    side = "left"
    values = [0.0, 0.5, 1.0, -1.0, -1.0, -1.0, -1.0]
    assert hand_presets.clamp(side, values) == values


def test_clamp_bounds_out_of_range_values():
    # left thumb_2 upper is ~1.745; over-range clamps down.
    out = hand_presets.clamp("left", [0, 0, 99.0, 0, 0, 0, 0])
    assert out[2] == pytest.approx(1.74532925)


def test_clamp_rejects_wrong_length():
    with pytest.raises(ValueError):
        hand_presets.clamp("left", [0.0, 0.0, 0.0])


def test_unknown_preset_raises():
    with pytest.raises(ValueError):
        hand_presets.preset_targets("left", "fist")


def test_unknown_side_raises():
    with pytest.raises(ValueError):
        hand_presets.joints("third")
