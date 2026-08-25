"""SO-101 leader-arm driver — Feetech serial bus in, ``sensor_msgs/JointState`` out.

``leader_source`` consumes a leader joint stream; on real hardware this node produces it.
The leader arm is a passive SO-101: five STS3215 arm servos plus a gripper, read over one
serial bus, torque never enabled. This node polls their positions and publishes them as a
JointState under the follower's joint names, which is all ``leader_source`` needs.

    /dev/tty.usbmodem*  ->  FeetechMotorsBus.sync_read  ->  steps_to_radians
                        ->  sensor_msgs/JointState on ``leader_joint_states_topic``

The bus library is LeRobot's ``FeetechMotorsBus`` — the same stack the provisioning
scripts under ``scripts/`` drive, so motor IDs assigned there are the IDs read here. It
is imported lazily inside ``connect()``: LeRobot lives in its own venv rather than the
ROS environment, so the module must import (and the tests must run) without it.

Hardware only. The sim leader-follower path needs no driver — any publisher on the leader
topic stands in for the arm, which is how the mode is proven on the mujoco backend first.
"""

import rclpy
from sensor_msgs.msg import JointState

from fm_teleop_core.source import TeleopSource
from fm_teleop_leader.sts3215 import vector_to_radians

# SO-101 leader joints in bus-ID order, matching the IDs the provisioning script assigns
# (scripts/configure-so101-leader-motors.py) and the follower's controller joint names.
DEFAULT_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def _default_bus(port, baudrate, joints):
    """Open a Feetech bus over ``port`` with one STS3215 per joint, IDs 1..n."""
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorNormMode

    # RAW, not a normalised mode: sync_read normalises on read when a norm mode is set,
    # and this node's conversion is defined on the servo's 0..4095 step count.
    motors = {
        name: Motor(index + 1, "sts3215", MotorNormMode.RAW)
        for index, name in enumerate(joints)
    }
    bus = FeetechMotorsBus(port=port, motors=motors)
    bus.connect()
    bus.set_baudrate(baudrate)
    return bus


class LeaderDriver(TeleopSource):
    """Publish an SO-101 leader arm's servo positions as a JointState stream."""

    def __init__(self, *, bus_factory=None, **kwargs):
        super().__init__("leader_driver", **kwargs)

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 1_000_000)
        self.declare_parameter("joints", DEFAULT_JOINTS)
        self.declare_parameter("topic", "/leader/joint_states")
        self.declare_parameter("rate_hz", 50.0)
        # Per-joint home offset in servo steps and direction sign (1 or -1), in `joints`
        # order. The one-element default is the "unset" value (rclpy infers a list
        # parameter's type from its default and cannot infer from an empty list): it
        # keeps the servo centre as zero and every direction positive.
        self.declare_parameter("offset_steps", [0])
        self.declare_parameter("signs", [1])

        self._joints = list(self.get_parameter("joints").value)
        self._offsets = self._per_joint("offset_steps", 0)
        self._signs = self._per_joint("signs", 1)

        self._pub = self.create_publisher(
            JointState, self.get_parameter("topic").value, 10
        )
        self._bus_factory = bus_factory or _default_bus
        self._bus = None
        self._read_failed = False  # an unplugged bus fails every tick; log the edge only
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)

    def _per_joint(self, name, unset):
        """Read a per-joint list parameter, or None when it is left at its default.

        A list of the wrong length is a malformed launch argument, not a partial config:
        silently ignoring it would leave half the arm reading in the wrong direction.
        """
        values = list(self.get_parameter(name).value)
        if values == [unset]:
            return None
        if len(values) != len(self._joints):
            raise ValueError(
                f"leader_driver: '{name}' has {len(values)} values for "
                f"{len(self._joints)} joints."
            )
        return values

    def connect(self):
        """Open the serial bus. Separate from ``__init__`` so the node constructs headless."""
        self._bus = self._bus_factory(
            self.get_parameter("port").value,
            int(self.get_parameter("baudrate").value),
            self._joints,
        )
        self.get_logger().info(
            f"leader_driver up: {len(self._joints)} motors on "
            f"{self.get_parameter('port').value}"
        )

    def _tick(self):
        if self._bus is None:
            return
        try:
            reading = self._bus.sync_read("Present_Position", self._joints)
        except Exception as exc:  # a bus hiccup must not kill the node mid-session
            if not self._read_failed:
                self._read_failed = True
                self.get_logger().warn(
                    f"leader_driver: bus read failed ({exc}); skipping samples until it returns."
                )
            return
        if self._read_failed:
            self._read_failed = False
            self.get_logger().info("leader_driver: bus read recovered.")
        message = JointState()
        message.header = self.stamped_header("")
        message.name = list(self._joints)
        message.position = vector_to_radians(
            [reading[name] for name in self._joints], self._offsets, self._signs
        )
        self._pub.publish(message)

    def destroy_node(self):
        if self._bus is not None:
            self._bus.disconnect()
            self._bus = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeaderDriver()
    try:
        node.connect()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
