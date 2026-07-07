"""Pure unit tests for gripper_teleop.preset_positions — no ROS graph."""

import pytest

from fm_teleop_device.gripper_teleop import preset_positions

OPEN = [0.0]
CLOSE = [-0.7854]


def test_open_variants_map_to_open():
    for p in ("open", "OPEN", " Open ", "release"):
        assert preset_positions(p, OPEN, CLOSE) == [0.0]


def test_close_variants_map_to_close():
    for p in ("close", "closed", "grasp", "pinch"):
        assert preset_positions(p, OPEN, CLOSE) == [-0.7854]


def test_returns_a_copy_not_the_input_list():
    out = preset_positions("open", OPEN, CLOSE)
    out.append(9.9)
    assert OPEN == [0.0]           # source list untouched


def test_unknown_preset_raises():
    with pytest.raises(ValueError):
        preset_positions("wiggle", OPEN, CLOSE)
    with pytest.raises(ValueError):
        preset_positions("", OPEN, CLOSE)
