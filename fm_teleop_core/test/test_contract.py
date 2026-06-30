"""The command contract carries every channel from the convergence map."""

from control_msgs.msg import JointJog
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
import pytest
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory

from fm_teleop_core import contract

# (key, message type, default topic) for every channel the convergence map names.
# Empty topic = per-instance channel (resolved at construction).
EXPECTED = [
    ("arm_twist", TwistStamped, "/servo_node/delta_twist_cmds"),
    ("arm_joint", JointJog, "/servo_node/delta_joint_cmds"),
    ("arm_pose_target", PoseStamped, "/target_pose"),
    ("base_twist", Twist, "/cmd_vel"),
    ("hand_preset", String, ""),
    ("hand_sliders", Float64MultiArray, ""),
    ("arm_trajectory", JointTrajectory, ""),
]


def test_contract_has_exactly_the_mapped_channels():
    assert set(contract.CHANNELS) == {key for key, _, _ in EXPECTED}


@pytest.mark.parametrize("key, msg_type, topic", EXPECTED)
def test_each_channel_type_and_topic(key, msg_type, topic):
    ch = contract.channel(key)
    assert ch.msg_type is msg_type
    assert ch.topic == topic


def test_unknown_channel_raises():
    with pytest.raises(ValueError):
        contract.channel("teleportation")
