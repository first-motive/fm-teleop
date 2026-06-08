"""Leader-arm teleop source (skeleton).

A physical leader arm whose joint states drive the follower directly. This is the
contract's leader-bypass path: the source publishes the ``arm_trajectory`` channel
(``trajectory_msgs/JointTrajectory``) straight to the follower's arm controller,
skipping MoveIt Servo — the leader's joints already are a valid pose stream.

Planned mapping:

    leader /joint_states  ->  arm_trajectory  ->  follower arm controller

Implement by subscribing to the leader's ``sensor_msgs/JointState`` and republishing
each sample as a single-point JointTrajectory via ``self.contract_publisher(
"arm_trajectory", topic=<follower controller>)``. See ../README.md for the add-a-source
guide.
"""

import rclpy

from fm_teleop_core.source import TeleopSource


class LeaderSource(TeleopSource):
    """Leader-arm follow source — not yet implemented."""

    def __init__(self):
        raise NotImplementedError(
            "fm_teleop_leader is a skeleton. Implement leader /joint_states -> "
            "arm_trajectory (leader bypass to the follower controller). See README.md."
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        LeaderSource()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
