"""Dex3 hand preset + slider mapping (pure logic, no ROS).

The G1-D hands are two Dex3 three-finger hands, each 7 actuated finger joints. This
module turns a named preset (open / close / pinch) or a raw 7-value slider vector into
the joint targets the hand JointTrajectoryController expects. The node
(``g1_hand_teleop.py``) owns the subscriptions + publishers; everything here is plain
data + functions so it can be unit-tested without a ROS graph.

Joint order matches the Dex3 motor index order used by the controllers + the hardware
bridge: thumb 0/1/2, middle 0/1, index 0/1. The two hands are mirrored, so a curled
finger swings to opposite-signed limits on left vs right — the presets below reflect
that. Position bounds are the vendored g1_d URDF limits; sliders are clamped to them.
"""

JOINTS = {
    "left": [
        "left_hand_thumb_0_joint",
        "left_hand_thumb_1_joint",
        "left_hand_thumb_2_joint",
        "left_hand_middle_0_joint",
        "left_hand_middle_1_joint",
        "left_hand_index_0_joint",
        "left_hand_index_1_joint",
    ],
    "right": [
        "right_hand_thumb_0_joint",
        "right_hand_thumb_1_joint",
        "right_hand_thumb_2_joint",
        "right_hand_middle_0_joint",
        "right_hand_middle_1_joint",
        "right_hand_index_0_joint",
        "right_hand_index_1_joint",
    ],
}

# (lower, upper) per joint, in JOINTS order, from the vendored g1_d URDF.
LIMITS = {
    "left": [
        (-1.04719755, 1.04719755),  # thumb_0
        (-0.72431163, 1.04719755),  # thumb_1
        (0.0, 1.74532925),          # thumb_2
        (-1.57079632, 0.0),         # middle_0
        (-1.74532925, 0.0),         # middle_1
        (-1.57079632, 0.0),         # index_0
        (-1.74532925, 0.0),         # index_1
    ],
    "right": [
        (-1.04719755, 1.04719755),  # thumb_0
        (-1.04719755, 0.72431163),  # thumb_1
        (-1.74532925, 0.0),         # thumb_2
        (0.0, 1.57079632),          # middle_0
        (0.0, 1.74532925),          # middle_1
        (0.0, 1.57079632),          # index_0
        (0.0, 1.74532925),          # index_1
    ],
}

# Named poses, in JOINTS order. open = fingers extended (all neutral); close = all
# fingers curled to their flexed limit; pinch = thumb + index curled to meet, middle
# left open. Curl direction is mirrored between hands, matching LIMITS.
PRESETS = {
    "left": {
        "open": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "close": [0.0, 1.04719755, 1.74532925, -1.57079632, -1.74532925, -1.57079632, -1.74532925],
        "pinch": [0.0, 1.04719755, 1.74532925, 0.0, 0.0, -1.57079632, -1.74532925],
    },
    "right": {
        "open": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "close": [0.0, -1.04719755, -1.74532925, 1.57079632, 1.74532925, 1.57079632, 1.74532925],
        "pinch": [0.0, -1.04719755, -1.74532925, 0.0, 0.0, 1.57079632, 1.74532925],
    },
}

SIDES = ("left", "right")


def joints(side):
    """Joint names for one hand, in controller order."""
    return JOINTS[_side(side)]


def preset_targets(side, name):
    """Return the 7 joint targets for a named preset, clamped to the URDF limits.

    Raises ValueError for an unknown side or preset name.
    """
    side = _side(side)
    try:
        values = PRESETS[side][name]
    except KeyError:
        raise ValueError(
            f"Unknown hand preset '{name}'. One of: {', '.join(sorted(PRESETS[side]))}."
        )
    return clamp(side, values)


def clamp(side, values):
    """Clamp a 7-value vector to the hand's per-joint limits (slider passthrough).

    Raises ValueError if the vector is not exactly 7 values.
    """
    side = _side(side)
    limits = LIMITS[side]
    if len(values) != len(limits):
        raise ValueError(
            f"Expected {len(limits)} values for the {side} hand, got {len(values)}."
        )
    return [max(lo, min(hi, float(v))) for v, (lo, hi) in zip(values, limits)]


def _side(side):
    if side not in JOINTS:
        raise ValueError(f"Unknown hand side '{side}'. One of: {', '.join(SIDES)}.")
    return side
