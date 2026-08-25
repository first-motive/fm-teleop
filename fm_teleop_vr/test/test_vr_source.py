"""VrSource node smoke test — no headset, no bridge, no Servo.

The node is driven through its pose/joy callbacks and its publishers are replaced with
recorders, so the whole engage → jog → release path runs on a bare rclpy context.
"""

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy

from fm_teleop_vr.vr_source import VrSource


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node(**overrides):
    params = {"hand_preset_topic": "/hand/right/preset", "deadzone": 0.0, **overrides}
    node = VrSource(
        parameter_overrides=[Parameter(name, value=value) for name, value in params.items()]
    )
    node._pub = RecordingPublisher()
    node._base_pub = RecordingPublisher()
    node._hand_pub = RecordingPublisher()
    return node


def _pose(x, y, z):
    message = PoseStamped()
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.position.z = z
    return message


def _joy(deadman=False, forward=0.0, turn=0.0, grip=0.0):
    return Joy(axes=[turn, forward, grip], buttons=[1 if deadman else 0])


def test_silent_until_the_deadman_is_held(ros):
    node = _make_node()
    try:
        node._on_pose(_pose(0.1, 0.0, 0.0))
        node._on_joy(_joy(forward=1.0))
        node._tick()
        assert node._pub.messages == []
        assert node._base_pub.messages == []
    finally:
        node.destroy_node()


def test_jog_is_proportional_to_displacement_from_the_engage_pose(ros):
    node = _make_node(scale=2.0, clamp=10.0)
    try:
        node._on_pose(_pose(1.0, 1.0, 1.0))
        node._on_joy(_joy(deadman=True))
        node._tick()  # the first tick after the rising edge latches neutral
        node._on_pose(_pose(1.1, 1.0, 1.0))
        node._tick()

        twist = node._pub.messages[-1].twist
        assert twist.linear.x == pytest.approx(0.2)
        assert twist.linear.y == pytest.approx(0.0)
        # Orientation is deferred — the MVP jogs linearly only.
        assert twist.angular.z == 0.0
    finally:
        node.destroy_node()


def test_engaging_before_the_first_pose_latches_when_it_arrives(ros):
    # Latching at the rising edge would latch None here and leave the arm silent for the
    # whole hold; the latch waits for the bridge's first pose instead.
    node = _make_node(scale=2.0, clamp=10.0)
    try:
        node._on_joy(_joy(deadman=True))
        node._tick()
        assert node._pub.messages == []

        node._on_pose(_pose(1.0, 1.0, 1.0))
        node._tick()  # latches neutral here, so this tick is a zero jog
        node._on_pose(_pose(1.1, 1.0, 1.0))
        node._tick()
        assert node._pub.messages[-1].twist.linear.x == pytest.approx(0.2)
    finally:
        node.destroy_node()


def test_the_stick_drives_the_base_while_engaged(ros):
    node = _make_node()
    try:
        node._on_pose(_pose(0.0, 0.0, 0.0))
        node._on_joy(_joy(deadman=True, forward=1.0, turn=-1.0))
        node._tick()

        base = node._base_pub.messages[-1]
        assert base.linear.x == pytest.approx(0.3)
        assert base.angular.z == pytest.approx(-0.8)
    finally:
        node.destroy_node()


def test_release_zeroes_both_motion_channels(ros):
    node = _make_node()
    try:
        node._on_pose(_pose(0.0, 0.0, 0.0))
        node._on_joy(_joy(deadman=True, forward=1.0))
        node._tick()
        node._on_joy(_joy(deadman=False, forward=1.0))

        assert node._pub.messages[-1].twist.linear.x == 0.0
        assert node._base_pub.messages[-1].linear.x == 0.0

        # ... and stays quiet afterwards.
        published = len(node._base_pub.messages)
        node._tick()
        assert len(node._base_pub.messages) == published
    finally:
        node.destroy_node()


def test_grip_publishes_a_preset_only_when_it_changes(ros):
    node = _make_node()
    try:
        node._on_joy(_joy(grip=0.9))
        node._on_joy(_joy(grip=0.9))
        node._on_joy(_joy(grip=0.5))  # inside the hysteresis band — no change
        assert [m.data for m in node._hand_pub.messages] == ["close"]

        node._on_joy(_joy(grip=0.1))
        assert [m.data for m in node._hand_pub.messages] == ["close", "open"]
    finally:
        node.destroy_node()


def test_the_hand_channel_is_off_without_a_topic(ros):
    node = VrSource(parameter_overrides=[Parameter("hand_preset_topic", value="")])
    try:
        assert node._hand_pub is None
        node._on_joy(_joy(grip=0.9))  # no publisher, no crash
    finally:
        node.destroy_node()


def test_a_short_joy_message_does_not_crash(ros):
    node = _make_node()
    try:
        node._on_joy(Joy(axes=[], buttons=[]))
        assert node._stick == (0.0, 0.0)
    finally:
        node.destroy_node()
