"""Start the policy server for one arm of one robot profile.

    ros2 launch fm_teleop_policy policy.launch.py robot_profile:=fm_openarm_v2 arm:=right

The server is the only node started here: it publishes onto the teleop contract's
``arm_joint`` channel, so it needs a running MoveIt Servo and robot (a sim run or the
real arm) that this launch does not bring up. It starts **disengaged**; publishing
``true`` on ``/policy_server/enable`` hands it the arm:

    ros2 topic pub /policy_server/enable std_msgs/msg/Bool '{data: true}'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = (
    ("robot_profile", "loader_openarm_v2",
     "Robot profile naming the joints: loader_openarm_v2 | fm_openarm_v2."),
    ("arm", "right", "Which arm of the robot profile the policy drives."),
    ("policy_type", "zero", "zero (no checkpoint, proves the loop) | lerobot."),
    ("checkpoint", "", "Checkpoint directory for a lerobot policy."),
)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(name, default_value=default, description=description)
            for name, default, description in ARGUMENTS
        ]
        + [
            Node(
                package="fm_teleop_policy",
                executable="policy_server",
                name="policy_server",
                output="screen",
                parameters=[
                    {name: LaunchConfiguration(name) for name, _, _ in ARGUMENTS}
                ],
            )
        ]
    )
