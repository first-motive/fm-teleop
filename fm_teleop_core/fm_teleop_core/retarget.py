"""Retarget helpers — pure device-to-command math shared by every source (no ROS).

A source's job is to map raw device input (stick axes, 6-DOF deltas, tracked joint
angles) onto the contract's command magnitudes. The recurring pieces of that mapping
live here as plain functions so they are unit-tested without a ROS graph and reused
rather than re-derived in each source: deadzone, clamp, and vector clamp.
"""


def deadzone(value, threshold):
    """Zero a reading whose magnitude is below ``threshold`` (stick drift cutoff)."""
    return 0.0 if abs(value) < threshold else float(value)


def clamp(value, lower, upper):
    """Clamp a scalar to ``[lower, upper]``."""
    return max(lower, min(upper, float(value)))


def clamp_vector(values, limits):
    """Clamp each value to its ``(lower, upper)`` pair.

    Raises ValueError if ``values`` and ``limits`` differ in length.
    """
    values = list(values)
    limits = list(limits)
    if len(values) != len(limits):
        raise ValueError(
            f"Expected {len(limits)} values, got {len(values)}."
        )
    return [clamp(v, lo, hi) for v, (lo, hi) in zip(values, limits)]


def scale(value, factor):
    """Multiply a reading by a speed scalar, returning a float."""
    return float(value) * float(factor)


def displacement_to_twist(position, neutral, factor, threshold, limit):
    """Map a tracked point's displacement from a neutral origin to a linear velocity.

    The position-control retarget used by the vision source: jog speed is proportional
    to how far the tracked point (a wrist) is held from the neutral pose captured at
    engage. Per axis, the displacement ``position - neutral`` is deadzoned (kills
    rest jitter near neutral), scaled (metres of displacement -> command velocity),
    and clamped to ``[-limit, limit]`` (bounds the jog). Returns a 3-vector
    ``[vx, vy, vz]``; the caller fills a Twist's linear part and leaves angular zero.

    ``position`` and ``neutral`` are 3-vectors in the same frame. Raises ValueError
    if either is not length 3.
    """
    position = list(position)
    neutral = list(neutral)
    if len(position) != 3 or len(neutral) != 3:
        raise ValueError(
            f"position and neutral must be 3-vectors, got {len(position)} and {len(neutral)}."
        )
    return [
        clamp(scale(deadzone(p - n, threshold), factor), -limit, limit)
        for p, n in zip(position, neutral)
    ]
