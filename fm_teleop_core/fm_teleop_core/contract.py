"""The teleop command contract — the standard messages every source collapses to.

Teleop is many input modalities (panel, gamepad, SpaceMouse, hand, and the planned
leader / VR / vision) onto one robot fleet. They converge here: whatever the device,
a source publishes one of a small fixed set of standard-interface channels, and the
sinks in ``fm_control`` (Servo, the controllers, the hardware bridges) consume them.

    SOURCES                        CONTRACT (standard msgs)             SINKS (fm_control)
    panel · gamepad · spacemouse   TwistStamped / JointJog -> Servo     controllers
    hand · [leader·vr·vision]      Twist -> /cmd_vel                    + bridges
                                   JointTrajectory (leader bypass)
                                   String / Float64MultiArray -> hands

This module is the single source of truth for that set: each channel's message type
and default topic. ``TeleopSource`` builds its publishers from it, so adding or moving
a channel is a one-line edit here, not a hunt across every source. No custom interface
package (``fm_teleop_msgs``) exists by design — the contract is standard messages only.
"""

from dataclasses import dataclass

from control_msgs.msg import JointJog
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory


@dataclass(frozen=True)
class Channel:
    """One contract channel: a message type on a default topic.

    ``topic`` may be empty for per-instance channels whose topic is only known at
    construction (the hand channels are namespaced per side, the leader-bypass
    trajectory is named per arm controller). A source passes the concrete topic to
    ``TeleopSource.contract_publisher`` in that case.
    """

    key: str
    msg_type: type
    topic: str
    summary: str


# Cartesian + per-joint arm jog go to MoveIt Servo's delta command topics.
ARM_TWIST = Channel(
    key="arm_twist",
    msg_type=TwistStamped,
    topic="/servo_node/delta_twist_cmds",
    summary="Cartesian arm jog -> MoveIt Servo",
)
ARM_JOINT = Channel(
    key="arm_joint",
    msg_type=JointJog,  # control_msgs/JointJog
    topic="/servo_node/delta_joint_cmds",
    summary="Per-joint arm jog -> MoveIt Servo",
)
# Absolute end-effector pose target -> MoveIt Servo PoseTracking. Unlike ARM_TWIST (a
# velocity jog that drifts on a steady bias), this is an absolute pose the EE servos to
# and HOLDS — the vision mirror source uses it for indexed 1:1 hand mirroring. One
# controller has one writer, so a source emits ARM_TWIST or ARM_POSE_TARGET, never both.
ARM_POSE_TARGET = Channel(
    key="arm_pose_target",
    msg_type=PoseStamped,
    topic="/target_pose",
    summary="Absolute EE pose target -> MoveIt Servo PoseTracking",
)
# Mobile-base velocity; diff_drive / holonomic controllers remap onto this.
BASE_TWIST = Channel(
    key="base_twist",
    msg_type=Twist,
    topic="/cmd_vel",
    summary="Base velocity -> drive controller",
)
# Hand control: a named pose or a raw joint vector, namespaced per hand side.
HAND_PRESET = Channel(
    key="hand_preset",
    msg_type=String,
    topic="",
    summary="Named hand pose (open|close|pinch) -> hand teleop",
)
HAND_SLIDERS = Channel(
    key="hand_sliders",
    msg_type=Float64MultiArray,
    topic="",
    summary="Raw hand joint targets -> hand teleop",
)
# Leader-arm bypass: a full trajectory straight to an arm controller (skips Servo).
ARM_TRAJECTORY = Channel(
    key="arm_trajectory",
    msg_type=JointTrajectory,
    topic="",
    summary="Full arm trajectory -> controller (leader bypass)",
)

CHANNELS = {
    channel.key: channel
    for channel in (
        ARM_TWIST,
        ARM_JOINT,
        ARM_POSE_TARGET,
        BASE_TWIST,
        HAND_PRESET,
        HAND_SLIDERS,
        ARM_TRAJECTORY,
    )
}


def channel(key):
    """Return the channel for a key, raising ValueError for an unknown one."""
    try:
        return CHANNELS[key]
    except KeyError:
        raise ValueError(
            f"Unknown contract channel '{key}'. One of: {', '.join(sorted(CHANNELS))}."
        )
