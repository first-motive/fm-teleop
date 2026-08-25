"""LeaderDriver smoke test — a fake bus stands in for the Feetech serial stack.

The bus is injected, so the read → convert → publish path runs with no serial port and
without importing LeRobot (which lives in its own venv, not the ROS environment).
"""

import pytest
import rclpy
from rclpy.parameter import Parameter

from fm_teleop_leader.leader_driver import LeaderDriver
from fm_teleop_leader.sts3215 import CENTRE_STEP

JOINTS = ["shoulder_pan", "elbow_flex"]


class FakeBus:
    def __init__(self, reading):
        self.reading = reading
        self.disconnected = False

    def sync_read(self, register, joints):
        if self.reading is None:
            raise RuntimeError("bus timeout")
        return dict(zip(joints, self.reading))

    def disconnect(self):
        self.disconnected = True


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


def _make_node(reading):
    bus = FakeBus(reading)
    node = LeaderDriver(
        bus_factory=lambda port, baudrate, joints: bus,
        parameter_overrides=[Parameter("joints", value=JOINTS)],
    )
    node._pub = RecordingPublisher()
    return node, bus


def test_no_publish_before_connect(ros):
    node, _ = _make_node([CENTRE_STEP, CENTRE_STEP])
    try:
        node._tick()
        assert node._pub.messages == []
    finally:
        node.destroy_node()


def test_publishes_joint_state_in_radians(ros):
    node, _ = _make_node([CENTRE_STEP, CENTRE_STEP + 1024])
    try:
        node.connect()
        node._tick()
        message = node._pub.messages[0]
        assert message.name == JOINTS
        assert message.position[0] == pytest.approx(0.0)
        assert message.position[1] == pytest.approx(1.5707963, abs=1e-6)
    finally:
        node.destroy_node()


def test_a_wrong_length_offset_list_is_rejected(ros):
    with pytest.raises(ValueError):
        LeaderDriver(
            bus_factory=lambda port, baudrate, joints: FakeBus([]),
            parameter_overrides=[
                Parameter("joints", value=JOINTS),
                Parameter("offset_steps", value=[0, 0, 0]),
            ],
        )


def test_a_bus_read_failure_skips_the_sample(ros):
    node, _ = _make_node(None)
    try:
        node.connect()
        node._tick()
        assert node._pub.messages == []
    finally:
        node.destroy_node()


def test_destroy_disconnects_the_bus(ros):
    node, bus = _make_node([CENTRE_STEP, CENTRE_STEP])
    node.connect()
    node.destroy_node()
    assert bus.disconnected
