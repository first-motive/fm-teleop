"""LeaderSource node smoke test — no leader arm, no controller, no serial bus.

The node is driven through its callbacks directly (leader sample, follower sample,
deadman) and its publisher is replaced with a recorder, so the whole engage → ramp →
publish path runs on a bare rclpy context. This is the deterministic half of the test
plan; the SO-101 leader run stays manual and hardware-gated.
"""

import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from fm_teleop_leader.leader_source import LeaderSource

JOINTS = ["shoulder_pan", "elbow_flex"]


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
    params = {
        "joints": JOINTS,
        "command_topic": "/so101_arm_controller/joint_trajectory",
        "max_joint_step": 0.1,
        **overrides,
    }
    node = LeaderSource(
        parameter_overrides=[Parameter(name, value=value) for name, value in params.items()]
    )
    node._pub = RecordingPublisher()
    return node


def _joint_state(names, positions):
    message = JointState()
    message.name = list(names)
    message.position = list(positions)
    return message


def test_missing_joints_parameter_is_rejected(ros):
    with pytest.raises(ValueError):
        LeaderSource(
            parameter_overrides=[
                Parameter("command_topic", value="/arm_controller/joint_trajectory")
            ]
        )


def test_a_blank_joint_entry_is_rejected(ros):
    # Dropping it would publish a short trajectory the controller rejects, after this
    # node logged success — the malformed launch argument must fail here instead.
    with pytest.raises(ValueError):
        _make_node(joints=["shoulder_pan", "", "elbow_flex"])


def test_mismatched_leader_joints_are_rejected(ros):
    with pytest.raises(ValueError):
        _make_node(leader_joints=["shoulder_pan"])


def test_silent_until_the_deadman_is_held(ros):
    node = _make_node()
    try:
        node._on_follower(_joint_state(JOINTS, [0.0, 0.0]))
        node._on_leader(_joint_state(JOINTS, [1.0, 1.0]))
        node._tick()
        assert node._pub.messages == []
    finally:
        node.destroy_node()


def test_engaged_tick_ramps_from_the_follower_pose(ros):
    node = _make_node()
    try:
        node._on_follower(_joint_state(JOINTS, [0.0, 0.0]))
        node._on_leader(_joint_state(JOINTS, [1.0, -1.0]))
        node._on_enable(Bool(data=True))
        node._tick()
        node._tick()

        assert len(node._pub.messages) == 2
        first, second = node._pub.messages
        assert first.joint_names == JOINTS
        # The controller reads time_from_start, not the header frame; assert the shape.
        assert first.points[0].time_from_start.sec == 0
        assert first.points[0].time_from_start.nanosec == 100_000_000
        # Bounded by max_joint_step per publish, not jumped to the leader's pose.
        assert first.points[0].positions == pytest.approx([0.1, -0.1])
        assert second.points[0].positions == pytest.approx([0.2, -0.2])
    finally:
        node.destroy_node()


def test_leader_sample_missing_a_joint_is_dropped(ros):
    node = _make_node()
    try:
        node._on_leader(_joint_state(["shoulder_pan"], [1.0]))
        assert node._leader is None
    finally:
        node.destroy_node()


def test_release_stops_commanding(ros):
    node = _make_node()
    try:
        node._on_follower(_joint_state(JOINTS, [0.0, 0.0]))
        node._on_leader(_joint_state(JOINTS, [1.0, 1.0]))
        node._on_enable(Bool(data=True))
        node._tick()
        node._on_enable(Bool(data=False))
        node._tick()
        assert len(node._pub.messages) == 1
    finally:
        node.destroy_node()


def test_a_stale_leader_stream_stops_commanding(ros):
    node = _make_node(sample_timeout=0.05)
    try:
        node._on_follower(_joint_state(JOINTS, [0.0, 0.0]))
        node._on_leader(_joint_state(JOINTS, [1.0, 1.0]))
        node._on_enable(Bool(data=True))
        node._tick()
        node._leader_stamp -= 1.0  # the leader stream goes quiet
        node._tick()
        assert len(node._pub.messages) == 1
    finally:
        node.destroy_node()


def test_commanding_resumes_when_the_leader_stream_returns(ros):
    node = _make_node(sample_timeout=0.05)
    try:
        node._on_follower(_joint_state(JOINTS, [0.0, 0.0]))
        node._on_leader(_joint_state(JOINTS, [1.0, 1.0]))
        node._on_enable(Bool(data=True))
        node._tick()
        node._leader_stamp -= 1.0
        node._tick()

        # The ramp re-seeds from the follower's state, so the resumed command steps by
        # one bound from where the follower actually is — never a jump to the leader.
        node._on_follower(_joint_state(JOINTS, [0.1, 0.1]))
        node._on_leader(_joint_state(JOINTS, [1.0, 1.0]))
        node._tick()
        assert len(node._pub.messages) == 2
        assert node._pub.messages[1].points[0].positions == pytest.approx([0.2, 0.2])
    finally:
        node.destroy_node()


def test_engaging_before_the_follower_state_arrives_holds(ros):
    node = _make_node()
    try:
        node._on_leader(_joint_state(JOINTS, [1.0, 1.0]))
        node._on_enable(Bool(data=True))
        node._tick()
        assert node._pub.messages == []

        # ... and commands as soon as the follower's state lands.
        node._on_follower(_joint_state(JOINTS, [0.0, 0.0]))
        node._tick()
        assert len(node._pub.messages) == 1
    finally:
        node.destroy_node()
