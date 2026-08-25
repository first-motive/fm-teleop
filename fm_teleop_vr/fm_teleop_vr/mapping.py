"""VR control mapping — pure functions, no ROS.

Two small mappings sit between a controller's raw inputs and the contract's channels,
and both have edge behaviour worth testing without a headset attached:

    stick_to_base_twist   thumbstick axes -> forward / turn magnitudes
    grip_preset           analogue grip -> a latched open|close hand preset

``grip_preset`` is hysteretic on purpose. A single threshold makes the hand chatter
open/close while the operator holds the grip near it; two thresholds with the current
state as the tiebreak mean the hand changes only on a deliberate squeeze or release.
"""

from fm_teleop_core.retarget import clamp, deadzone

OPEN = "open"
CLOSE = "close"


def stick_to_base_twist(forward_axis, turn_axis, threshold, speed, turn_speed):
    """Map two stick axes to ``(linear_x, angular_z)`` base magnitudes.

    Each axis is deadzoned (stick drift never creeps the base), scaled by its speed, and
    clamped to that speed so a stick at full deflection is the bound.
    """
    linear = clamp(deadzone(forward_axis, threshold) * speed, -speed, speed)
    angular = clamp(deadzone(turn_axis, threshold) * turn_speed, -turn_speed, turn_speed)
    return linear, angular


def grip_preset(value, current, open_below, close_above):
    """Return the hand preset for an analogue grip reading, with hysteresis.

    ``current`` is the last preset (or None before the first reading). Between the two
    thresholds the preset holds. Raises ValueError when ``open_below`` is not below
    ``close_above`` — an inverted pair would toggle on every reading.
    """
    if open_below >= close_above:
        raise ValueError(
            f"open_below ({open_below}) must be below close_above ({close_above})."
        )
    if value >= close_above:
        return CLOSE
    if value <= open_below:
        return OPEN
    return current
