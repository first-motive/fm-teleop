"""Dex3 hand teleop adapter — presets + sliders -> hand JointTrajectory.

Turns the panel's high-level hand commands into the full 7-joint trajectories the Dex3
hand controllers expect (the JTC rejects partial trajectories, so every command names
all 7 finger joints). Two inputs per hand:

    ~/<side>/preset   std_msgs/String           open | close | pinch
    ~/<side>/sliders  std_msgs/Float64MultiArray 7 raw joint targets (passthrough)

and one output per hand:

    /g1_<side>_hand_controller/joint_trajectory  trajectory_msgs/JointTrajectory

Presets resolve to clamped joint targets; sliders are clamped and forwarded as-is. The
mapping (joint order, limits, preset poses) lives in hand_presets.py so it is unit-tested
without a ROS graph. In sim this drives the hand JTCs; on real hardware the same
trajectories feed the Dex3 bridge (g1_hand_sdk_bridge).
"""

from builtin_interfaces.msg import Duration
import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from fm_teleop_core import contract
from fm_teleop_core.source import TeleopSource
from fm_teleop_device import hand_presets


class G1HandTeleop(TeleopSource):
    """Map hand presets + sliders onto the two Dex3 hand controllers.

    Consumes the contract's hand_preset (String) and hand_sliders
    (Float64MultiArray) channels and emits the JointTrajectory the controllers want.
    """

    def __init__(self):
        super().__init__("g1_hand_teleop")

        self.declare_parameter(
            "left_traj_topic", "/g1_left_hand_controller/joint_trajectory"
        )
        self.declare_parameter(
            "right_traj_topic", "/g1_right_hand_controller/joint_trajectory"
        )
        # Time the controller takes to reach the target; small for responsive teleop.
        self.declare_parameter("point_time", 0.5)

        self._pub = {
            "left": self.create_publisher(
                JointTrajectory, self.get_parameter("left_traj_topic").value, 10
            ),
            "right": self.create_publisher(
                JointTrajectory, self.get_parameter("right_traj_topic").value, 10
            ),
        }

        preset_type = contract.HAND_PRESET.msg_type
        sliders_type = contract.HAND_SLIDERS.msg_type
        for side in hand_presets.SIDES:
            self.create_subscription(
                preset_type, f"~/{side}/preset",
                lambda msg, s=side: self._on_preset(s, msg), 10
            )
            self.create_subscription(
                sliders_type, f"~/{side}/sliders",
                lambda msg, s=side: self._on_sliders(s, msg), 10
            )

    def _on_preset(self, side, msg):
        try:
            targets = hand_presets.preset_targets(side, msg.data)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return
        self._publish(side, targets)

    def _on_sliders(self, side, msg):
        try:
            targets = hand_presets.clamp(side, list(msg.data))
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return
        self._publish(side, targets)

    def _publish(self, side, targets):
        traj = JointTrajectory()
        traj.joint_names = hand_presets.joints(side)
        point = JointTrajectoryPoint()
        point.positions = list(targets)
        seconds = float(self.get_parameter("point_time").value)
        point.time_from_start = Duration(
            sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9)
        )
        traj.points = [point]
        self._pub[side].publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = G1HandTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
