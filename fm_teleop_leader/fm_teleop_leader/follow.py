"""Leader-to-follower retarget math — pure functions, no ROS.

A leader arm publishes a ``sensor_msgs/JointState`` whose ``name``/``position`` pairing
is arbitrary in order and may carry joints the follower does not have. The follower's
controller wants one ordered position vector. The two steps between them are here, so
they unit-test without a ROS graph:

    select_positions   name-keyed leader sample -> follower-ordered vector
    limit_step         bound how far that vector may move in one control period

``limit_step`` is the safety piece. The leader-bypass path skips MoveIt Servo, so no
downstream node bounds the follower's velocity: a leader teleport (a dropped sample, a
motor re-index, an operator picking the arm up) would otherwise reach the controller as
a full-speed jump. Bounding per-period travel turns that into a ramp.
"""


def select_positions(names, positions, wanted):
    """Return the positions for ``wanted``, in that order, or None if any is missing.

    ``names`` and ``positions`` are the parallel arrays of a JointState sample; ``wanted``
    is the leader joint names the follower needs. Returning None (rather than a partial
    vector) keeps a half-populated trajectory off the controller — a sample missing a
    joint is a sample to drop, not to guess at.
    """
    lookup = dict(zip(names, positions))
    try:
        return [float(lookup[name]) for name in wanted]
    except KeyError:
        return None


def limit_step(previous, target, max_step):
    """Move ``previous`` toward ``target`` by at most ``max_step`` per joint.

    ``previous`` is the last commanded vector, or None on the first sample (nothing to
    ramp from — the target passes through). ``max_step`` of 0 or less disables the limit.
    Raises ValueError when the two vectors differ in length.
    """
    if previous is not None and len(previous) != len(target):
        raise ValueError(
            f"Expected {len(previous)} positions, got {len(target)}."
        )
    if previous is None or max_step is None or max_step <= 0:
        return [float(value) for value in target]
    return [
        float(prev) + max(-max_step, min(max_step, float(goal) - float(prev)))
        for prev, goal in zip(previous, target)
    ]
