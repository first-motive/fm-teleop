"""gripper_teleop — map a named hand preset (open|close) to a gripper controller command.

The vision pipeline detects hand openness (hand_tracker finger curl) and mirror_source turns
it into a std_msgs/String preset ("open" | "close") on a per-side topic. This adapter is the
missing link that actually MOVES a gripper: it subscribes that preset and publishes a
trajectory_msgs/JointTrajectory to a JointTrajectoryController gripper.

Generic + params-driven so it serves OpenArm's pinch gripper and, later, SO-101:

    preset_topic   std_msgs/String                  "open" | "close"   (in; from mirror_source)
    command_topic  trajectory_msgs/JointTrajectory                     (out; to the gripper JTC)

Joint names + open/close positions are params, single-sourced from fm_bringup.registry's
``gripper`` spec. Mirrors g1_hand_teleop.py's preset -> trajectory pattern. The pure
open/close -> positions mapping is ``preset_positions`` so it unit-tests without a ROS graph.
"""

from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

_OPEN = {"open", "release", "0"}
_CLOSE = {"close", "closed", "grasp", "pinch", "1"}


def preset_positions(preset, open_positions, close_positions):
    """Map a preset name to gripper joint positions; raise ValueError on an unknown preset."""
    key = (preset or "").strip().lower()
    if key in _OPEN:
        return list(open_positions)
    if key in _CLOSE:
        return list(close_positions)
    raise ValueError("unknown gripper preset %r (expected open|close)" % preset)


class GripperTeleop(Node):
    """Drive a JointTrajectoryController gripper from a hand-preset String."""

    def __init__(self):
        super().__init__("gripper_teleop")
        self.declare_parameter("preset_topic", "/gripper_teleop/right/preset")
        self.declare_parameter(
            "command_topic", "/openarm_right_gripper_controller/joint_trajectory"
        )
        self.declare_parameter("joints", ["openarm_right_finger_joint1"])
        self.declare_parameter("open_positions", [0.0])
        self.declare_parameter("close_positions", [-0.7854])
        # Controller travel time to the target; small for responsive teleop.
        self.declare_parameter("move_time_s", 0.3)

        gp = self.get_parameter
        self._joints = list(gp("joints").value)
        self._open = [float(x) for x in gp("open_positions").value]
        self._close = [float(x) for x in gp("close_positions").value]
        self._move_time = float(gp("move_time_s").value)

        self._pub = self.create_publisher(JointTrajectory, gp("command_topic").value, 10)
        self.create_subscription(String, gp("preset_topic").value, self._on_preset, 10)
        self.get_logger().info(
            "gripper_teleop: %s -> %s (joints=%s open=%s close=%s)"
            % (gp("preset_topic").value, gp("command_topic").value,
               self._joints, self._open, self._close)
        )

    def _on_preset(self, msg):
        try:
            positions = preset_positions(msg.data, self._open, self._close)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return
        traj = JointTrajectory()
        traj.joint_names = list(self._joints)
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(self._move_time), nanosec=int((self._move_time % 1.0) * 1e9)
        )
        traj.points = [point]
        self._pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = GripperTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
