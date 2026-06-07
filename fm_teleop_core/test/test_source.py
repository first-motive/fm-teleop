"""TeleopSource base node: contract-driven publisher creation."""

import pytest
import rclpy

from fm_teleop_core.source import TeleopSource


@pytest.fixture
def node():
    rclpy.init()
    source = TeleopSource("test_source")
    yield source
    source.destroy_node()
    rclpy.shutdown()


def test_contract_publisher_uses_channel_default_topic(node):
    pub = node.contract_publisher("arm_twist")
    assert pub.topic_name == "/servo_node/delta_twist_cmds"


def test_contract_publisher_topic_override(node):
    pub = node.contract_publisher("hand_preset", topic="/g1/left/preset")
    assert pub.topic_name == "/g1/left/preset"


def test_contract_publisher_requires_topic_for_empty_channel(node):
    # Per-instance channels (hand_preset) have no default topic.
    with pytest.raises(ValueError):
        node.contract_publisher("hand_preset")


def test_contract_publisher_rejects_unknown_channel(node):
    with pytest.raises(ValueError):
        node.contract_publisher("warp_drive")


def test_stamped_header_carries_frame(node):
    header = node.stamped_header("base_link")
    assert header.frame_id == "base_link"
