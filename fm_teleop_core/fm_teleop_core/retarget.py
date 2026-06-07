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
