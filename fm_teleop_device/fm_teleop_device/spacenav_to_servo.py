"""SpaceMouse -> MoveIt Servo adapter.

spacenav_node publishes the 6-DOF device as geometry_msgs/Twist on /spacenav/twist.
Servo wants a TwistStamped in the command frame, so this stamps and relays it. The
SpaceMouse gives the best Cartesian ergonomics but is USB, so Linux-only — no
OrbStack/Docker passthrough on the Mac.
"""

from geometry_msgs.msg import Twist, TwistStamped
import rclpy

from fm_teleop_core.source import TeleopSource


class SpacenavToServo(TeleopSource):
    """Stamp /spacenav/twist into Servo's delta_twist_cmds."""

    def __init__(self):
        super().__init__("spacenav_to_servo")
        self.declare_parameter("command_frame", "openarm_right_base_link")
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("spacenav_topic", "/spacenav/twist")

        self._frame = self.get_parameter("command_frame").value
        self._pub = self.contract_publisher(
            "arm_twist", topic=self.get_parameter("twist_topic").value
        )
        self.create_subscription(
            Twist, self.get_parameter("spacenav_topic").value, self._on_twist, 10
        )

    def _on_twist(self, msg):
        stamped = TwistStamped()
        stamped.header = self.stamped_header(self._frame)
        stamped.twist = msg
        self._pub.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = SpacenavToServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
