"""Leader-arm teleop source — a physical leader arm drives the follower directly.

The leader's joints already form a valid pose stream, so this source republishes them
straight to the follower's arm controller, skipping MoveIt Servo (which exists to turn
Cartesian/joint *deltas* into safe motion). That is the contract's leader-bypass path:

    leader /joint_states  ->  arm_trajectory (JointTrajectory)  ->  follower controller

    leader sample ---> select_positions ---> limit_step ---> single-point trajectory
      (name-keyed)      (follower order)     (ramp bound)      (at rate_hz)

Engagement is a deadman on ``~/enable`` (std_msgs/Bool, from the panel or the TUI),
matching the vision source. The rising edge seeds the ramp from the follower's *current*
joint state, so engaging with the two arms far apart drives a bounded ramp instead of a
jump. Releasing stops commanding immediately; the controller holds its last point.

Safety lives here because nothing downstream provides it. Servo is bypassed, so this node
owns the two bounds that keep a bypass survivable: ``max_joint_step`` caps travel per
control period, and ``sample_timeout`` stops commanding when the leader stream goes quiet.
The zenoh bridge denies ``*_controller/joint_trajectory`` across machines, so this source
runs on the rig with its leader, never remotely.

``joints`` and ``command_topic`` have no useful defaults — they are the follower's
controller identity. ``teleop.launch.py`` reads both from the ``fm_bringup`` registry, so
an operator never types them.
"""

import rclpy
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from fm_teleop_core.source import TeleopSource
from fm_teleop_leader.follow import limit_step, select_positions


class LeaderSource(TeleopSource):
    """Republish a leader arm's joint state as follower trajectory points."""

    def __init__(self, **kwargs):
        super().__init__("leader_source", **kwargs)

        # Follower controller identity — supplied by the launch from the robot registry.
        self.declare_parameter("command_topic", "")
        self.declare_parameter("joints", [""])
        # Leader joint names, when they differ from the follower's. Empty -> same names.
        self.declare_parameter("leader_joints", [""])

        self.declare_parameter("leader_joint_states_topic", "/leader/joint_states")
        self.declare_parameter("follower_joint_states_topic", "/joint_states")
        self.declare_parameter("enable_topic", "~/enable")
        self.declare_parameter("rate_hz", 50.0)
        # Seconds the controller is given to reach each point. Below one control period
        # the controller extrapolates and the arm judders; a small multiple is smooth.
        self.declare_parameter("time_from_start", 0.1)
        # Radians of travel allowed per published point — the bypass's velocity bound.
        self.declare_parameter("max_joint_step", 0.05)
        # Seconds without a leader sample before commanding stops.
        self.declare_parameter("sample_timeout", 0.5)

        # A list parameter is declared with a one-element default because rclpy infers a
        # parameter's type from its default and cannot infer from an empty list. The
        # placeholder element is the "unset" value, and a blank entry anywhere else is a
        # malformed list: dropping it would publish a short trajectory the controller
        # rejects, while reporting success here.
        joints = list(self.get_parameter("joints").value)
        if not joints or joints == [""]:
            raise ValueError("leader_source: 'joints' is required (follower joint order).")
        if any(not name for name in joints):
            raise ValueError(f"leader_source: 'joints' has a blank entry: {joints}.")
        self._joints = joints
        leader_joints = list(self.get_parameter("leader_joints").value)
        if leader_joints == [""]:
            leader_joints = []
        self._leader_joints = leader_joints or self._joints
        if len(self._leader_joints) != len(self._joints):
            raise ValueError(
                f"leader_source: 'leader_joints' has {len(self._leader_joints)} names, "
                f"'joints' has {len(self._joints)} — they map one to one."
            )

        self._rate_hz = float(self.get_parameter("rate_hz").value)
        self._time_from_start = float(self.get_parameter("time_from_start").value)
        self._max_step = float(self.get_parameter("max_joint_step").value)
        self._sample_timeout = float(self.get_parameter("sample_timeout").value)

        self._pub = self.contract_publisher(
            "arm_trajectory", topic=self.get_parameter("command_topic").value
        )

        # Single-threaded executor: the subscriptions and the timer never interleave, so
        # this multi-field state needs no lock. A move to a multi-threaded executor would.
        self._enabled = False
        self._commanded = None  # last commanded vector — the ramp's starting point
        self._leader = None  # latest leader vector, in follower order
        self._leader_stamp = None  # node clock seconds of that sample
        self._follower = None  # latest follower vector, for seeding the ramp

        self.create_subscription(
            JointState,
            self.get_parameter("leader_joint_states_topic").value,
            self._on_leader,
            10,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("follower_joint_states_topic").value,
            self._on_follower,
            10,
        )
        self.create_subscription(
            Bool, self.get_parameter("enable_topic").value, self._on_enable, 10
        )
        self.create_timer(1.0 / self._rate_hz, self._tick)

        self.get_logger().info(
            f"leader_source up: {len(self._joints)} joints -> "
            f"{self.get_parameter('command_topic').value} at {self._rate_hz} Hz "
            f"(deadman {self.get_parameter('enable_topic').value})"
        )

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_leader(self, msg):
        positions = select_positions(msg.name, msg.position, self._leader_joints)
        if positions is None:
            # A sample missing a joint is dropped, not patched — a partial vector would
            # command the absent joints to whatever they last held.
            return
        self._leader = positions
        self._leader_stamp = self._now()

    def _on_follower(self, msg):
        self._follower = select_positions(msg.name, msg.position, self._joints)

    def _on_enable(self, msg):
        if msg.data and not self._enabled:
            # Rising edge: ramp from where the follower actually is, not from the last
            # commanded point of a previous session.
            self._commanded = self._follower
            if self._commanded is None:
                self.get_logger().warn(
                    "leader_source: engaged before the follower's joint state arrived; "
                    "holding until it does."
                )
        elif not msg.data and self._enabled:
            self._commanded = None
        self._enabled = bool(msg.data)

    def _tick(self):
        if not self._enabled or self._leader is None:
            return
        if self._now() - self._leader_stamp > self._sample_timeout:
            # Leader stream went quiet — stop commanding and re-seed on its return, so a
            # reconnect after the operator moved the leader ramps instead of jumping.
            self._commanded = None
            return
        if self._commanded is None:
            self._commanded = self._follower
            if self._commanded is None:
                return
        self._commanded = limit_step(self._commanded, self._leader, self._max_step)
        self._pub.publish(self._trajectory(self._commanded))

    def _trajectory(self, positions):
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(seconds=self._time_from_start).to_msg()
        message = JointTrajectory()
        message.header = self.stamped_header("")
        message.joint_names = list(self._joints)
        message.points = [point]
        return message


def main(args=None):
    rclpy.init(args=args)
    node = LeaderSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
