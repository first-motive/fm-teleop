"""Gamepad -> MoveIt Servo adapter.

Maps a sensor_msgs/Joy stream to the two command streams Servo consumes:

    left stick            linear X / Y
    triggers (LT/RT)      linear Z down/up
    right stick           angular X / Y
    bumpers (LB/RB)       angular Z

Commands are unitless ([-1, 1]) TwistStamped; Servo scales them (see servo.yaml).
Per-joint jogging is the Foxglove panel's job. Axis and button indices follow the
common Xbox-style layout and are overridable via params.

On Linux the gamepad is read by the joy package's joy_node from /dev/input. On the
Mac the container has no USB passthrough (OrbStack limitation) — run a host-side
HID->Joy bridge that republishes /joy over the OrbStack network instead.
"""

from geometry_msgs.msg import TwistStamped
import rclpy
from sensor_msgs.msg import Joy

from fm_teleop_core import retarget
from fm_teleop_core.source import TeleopSource


class JoyToServo(TeleopSource):
    """Translate Joy messages into Servo Cartesian twist commands."""

    def __init__(self):
        super().__init__("joy_to_servo")

        self.declare_parameter("command_frame", "openarm_right_base_link")
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        # Xbox-style axis/button indices (override per controller).
        self.declare_parameter("axis_linear_x", 1)
        self.declare_parameter("axis_linear_y", 0)
        self.declare_parameter("axis_trigger_down", 2)  # LT
        self.declare_parameter("axis_trigger_up", 5)    # RT
        self.declare_parameter("axis_angular_x", 4)
        self.declare_parameter("axis_angular_y", 3)
        self.declare_parameter("button_angular_neg", 4)  # LB
        self.declare_parameter("button_angular_pos", 5)  # RB
        self.declare_parameter("deadzone", 0.1)

        self._frame = self.get_parameter("command_frame").value
        self._pub = self.contract_publisher(
            "arm_twist", topic=self.get_parameter("twist_topic").value
        )
        self.create_subscription(Joy, "/joy", self._on_joy, 10)

    def _axis(self, joy, index):
        """Read an axis with deadzone, guarding against short Joy arrays."""
        if index < 0 or index >= len(joy.axes):
            return 0.0
        return retarget.deadzone(joy.axes[index], self.get_parameter("deadzone").value)

    def _button(self, joy, index):
        return joy.buttons[index] if 0 <= index < len(joy.buttons) else 0

    def _on_joy(self, joy):
        params = self.get_parameter
        twist = TwistStamped()
        twist.header = self.stamped_header(self._frame)
        twist.twist.linear.x = self._axis(joy, params("axis_linear_x").value)
        twist.twist.linear.y = self._axis(joy, params("axis_linear_y").value)
        # Triggers rest at +1 and fall to -1 when pressed; map to [0, 1] up/down.
        up = (1.0 - self._axis(joy, params("axis_trigger_up").value)) / 2.0
        down = (1.0 - self._axis(joy, params("axis_trigger_down").value)) / 2.0
        twist.twist.linear.z = up - down
        twist.twist.angular.x = self._axis(joy, params("axis_angular_x").value)
        twist.twist.angular.y = self._axis(joy, params("axis_angular_y").value)
        twist.twist.angular.z = float(
            self._button(joy, params("button_angular_pos").value)
            - self._button(joy, params("button_angular_neg").value)
        )
        self._pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
