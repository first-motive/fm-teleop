"""VR-controller teleop source — a tracked controller jogs the arm, base, and hand.

A headset-tracked 6-DOF controller is the richest single input the contract takes: its
pose drives the arm, its thumbstick the base, and its grip the hand. Each maps onto an
existing channel, so nothing downstream changes:

    controller pose - engage pose  ->  arm_twist (TwistStamped)  ->  MoveIt Servo
    thumbstick                     ->  base_twist (Twist)         ->  /cmd_vel
    analogue grip                  ->  hand_preset (String)       ->  hand teleop

Engagement is a deadman button on the controller's ``sensor_msgs/Joy`` stream, matching
the vision source's deadman: the rising edge latches the controller's current pose as
neutral, and the arm jogs proportional to displacement from it while the button is held.
Releasing publishes one zero twist (and one zero base twist) so Servo and the base stop
immediately rather than coasting on the last command.

This node consumes a VR *bridge* — whatever republishes a headset runtime's controller
state as ``PoseStamped`` + ``Joy``. No OpenXR or vendor SDK is imported here, so the node
runs on any machine and is proven in sim by publishing those two topics by hand.

Orientation is deferred: the arm jogs linearly, angular stays zero. A pose *difference*
is the natural next step (quaternion delta to an angular magnitude) but it needs Servo's
angular scaling tuned per robot, so the MVP keeps the axis count that is already proven.
"""

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from fm_teleop_core.retarget import displacement_to_twist
from fm_teleop_core.source import TeleopSource
from fm_teleop_vr.mapping import grip_preset, stick_to_base_twist


class VrSource(TeleopSource):
    """Jog the arm, base, and hand from one tracked VR controller."""

    def __init__(self, **kwargs):
        super().__init__("vr_source", **kwargs)

        # Input topics from the VR bridge.
        self.declare_parameter("pose_topic", "/vr/controller/right/pose")
        self.declare_parameter("joy_topic", "/vr/controller/right/joy")

        # Output channels. An empty hand topic disables the hand mapping, which is the
        # default: most robots in the fleet have a gripper adapter, not a preset hand.
        self.declare_parameter("command_frame", "openarm_right_base_link")
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("base_twist_topic", "/cmd_vel")
        self.declare_parameter("hand_preset_topic", "")

        self.declare_parameter("rate_hz", 50.0)
        # Displacement (m) from the engage pose -> Servo's unitless twist, the same
        # position-control retarget the vision source uses.
        self.declare_parameter("scale", 4.0)
        self.declare_parameter("deadzone", 0.02)
        self.declare_parameter("clamp", 1.0)

        # Controller layout. Indices follow the common OpenXR-to-Joy convention and are
        # overridable per bridge.
        self.declare_parameter("button_deadman", 0)  # trigger
        self.declare_parameter("axis_stick_forward", 1)
        self.declare_parameter("axis_stick_turn", 0)
        self.declare_parameter("axis_grip", 2)
        self.declare_parameter("stick_deadzone", 0.15)
        self.declare_parameter("base_speed", 0.3)  # m/s at full deflection
        self.declare_parameter("base_turn_speed", 0.8)  # rad/s at full deflection
        self.declare_parameter("grip_open_below", 0.35)
        self.declare_parameter("grip_close_above", 0.65)

        self._frame = self.get_parameter("command_frame").value
        self._pub = self.contract_publisher(
            "arm_twist", topic=self.get_parameter("twist_topic").value
        )
        self._base_pub = self.contract_publisher(
            "base_twist", topic=self.get_parameter("base_twist_topic").value
        )
        hand_topic = self.get_parameter("hand_preset_topic").value
        self._hand_pub = (
            self.contract_publisher("hand_preset", topic=hand_topic) if hand_topic else None
        )

        # Single-threaded executor: the subscriptions and the timer never interleave, so
        # this state needs no lock. A move to a multi-threaded executor would.
        self._pose = None  # latest controller position, [x, y, z] in the bridge's frame
        self._neutral = None  # pose latched at the deadman's rising edge
        self._capture_neutral = False  # latch pending until a pose arrives
        self._enabled = False
        self._stick = (0.0, 0.0)
        self._grip = None

        self.create_subscription(
            PoseStamped, self.get_parameter("pose_topic").value, self._on_pose, 10
        )
        self.create_subscription(
            Joy, self.get_parameter("joy_topic").value, self._on_joy, 10
        )
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)

        self.get_logger().info(
            f"vr_source up: {self.get_parameter('pose_topic').value} + "
            f"{self.get_parameter('joy_topic').value} -> arm_twist, base_twist"
            + (f", hand_preset on {hand_topic}" if self._hand_pub else "")
        )

    def _on_pose(self, msg):
        self._pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def _axis(self, joy, index):
        return joy.axes[index] if 0 <= index < len(joy.axes) else 0.0

    def _on_joy(self, joy):
        params = self.get_parameter
        index = params("button_deadman").value
        held = bool(joy.buttons[index]) if 0 <= index < len(joy.buttons) else False

        if held and not self._enabled:
            # Rising edge: latch the current pose as neutral, so the jog starts at zero
            # wherever the operator happens to be holding the controller. The latch is
            # deferred to the tick — engaging before the bridge's first pose arrives would
            # otherwise latch nothing and leave the arm silent for the whole hold.
            self._capture_neutral = True
        elif not held and self._enabled:
            self._neutral = None
            self._capture_neutral = False
            self._publish_stop()
        self._enabled = held

        self._stick = stick_to_base_twist(
            self._axis(joy, params("axis_stick_forward").value),
            self._axis(joy, params("axis_stick_turn").value),
            params("stick_deadzone").value,
            params("base_speed").value,
            params("base_turn_speed").value,
        )
        self._update_grip(self._axis(joy, params("axis_grip").value))

    def _update_grip(self, value):
        if self._hand_pub is None:
            return
        preset = grip_preset(
            value,
            self._grip,
            self.get_parameter("grip_open_below").value,
            self.get_parameter("grip_close_above").value,
        )
        if preset is not None and preset != self._grip:
            self._grip = preset
            self._hand_pub.publish(String(data=preset))

    def _tick(self):
        # The deadman gates the base as well as the arm: a source that streamed /cmd_vel
        # whenever it was running would fight every other writer on that topic.
        if not self._enabled:
            return
        linear, angular = self._stick
        base = Twist()
        base.linear.x = linear
        base.angular.z = angular
        self._base_pub.publish(base)

        if self._pose is None:
            return
        if self._capture_neutral:
            self._neutral = self._pose
            self._capture_neutral = False
        if self._neutral is None:
            return
        params = self.get_parameter
        twist = TwistStamped()
        twist.header = self.stamped_header(self._frame)
        vx, vy, vz = displacement_to_twist(
            self._pose,
            self._neutral,
            params("scale").value,
            params("deadzone").value,
            params("clamp").value,
        )
        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = vz
        self._pub.publish(twist)

    def _publish_stop(self):
        """Zero both motion channels so Servo and the base stop the instant of release."""
        twist = TwistStamped()
        twist.header = self.stamped_header(self._frame)
        self._pub.publish(twist)
        self._base_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = VrSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
