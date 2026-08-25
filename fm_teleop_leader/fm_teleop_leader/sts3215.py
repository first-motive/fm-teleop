"""STS3215 step-to-radian conversion — pure math, no ROS and no serial bus.

The SO-101 leader arm's Feetech STS3215 servos report position as a 12-bit step count
over a 360° travel: 0…4095, centre 2048. A follower joint wants radians about its own
zero. The conversion is one line of arithmetic, but it carries the two per-joint knobs
every leader arm needs — a home offset (the operator's neutral is not the servo's
centre) and a direction sign (a mirrored linkage runs the other way) — so it lives here
where it is unit-tested without a motor attached.
"""

import math

STEPS_PER_TURN = 4096
CENTRE_STEP = 2048


def steps_to_radians(steps, offset_steps=0, sign=1):
    """Convert one servo's step count to radians about the joint's zero.

    ``offset_steps`` shifts the zero (the arm's home reading); ``sign`` is 1 or -1 for a
    joint whose positive direction is inverted relative to the follower's.
    """
    return sign * (steps - CENTRE_STEP - offset_steps) * (2.0 * math.pi / STEPS_PER_TURN)


def vector_to_radians(steps, offsets=None, signs=None):
    """Convert a per-joint step vector, applying per-joint offsets and signs.

    ``offsets`` and ``signs`` default to zeros and ones. Raises ValueError when either
    is given at a different length than ``steps``.
    """
    steps = list(steps)
    offsets = list(offsets) if offsets else [0] * len(steps)
    signs = list(signs) if signs else [1] * len(steps)
    if len(offsets) != len(steps) or len(signs) != len(steps):
        raise ValueError(
            f"Expected {len(steps)} offsets and signs, got {len(offsets)} and {len(signs)}."
        )
    return [
        steps_to_radians(step, offset, sign)
        for step, offset, sign in zip(steps, offsets, signs)
    ]
