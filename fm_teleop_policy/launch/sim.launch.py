"""Start the policy server inside an fm-sim run, as the ``policy_serve`` controller.

fm-sim's ``run.launch.py`` includes this file for the ``policy_serve`` catalog entry
(``catalog/controllers/policy_serve.yaml``) and passes exactly three arguments:

    use_sim        'true' | 'false' (default 'false'); sets the server's use_sim_time
    robot_profile  required; the robot entry's profile, e.g. fm_openarm_v2
    scenario       the scenario name; validated, otherwise unused by the server

The server drives the profile's right arm with the zero policy (the run contract
carries no caller parameters yet). The server is always started
with ``sim_only``. With ``use_sim:=true`` it also gets ``engage_on_start``, so it
engages itself on its first valid observation: a sim run has no operator to hold
the gate. That touches this server only; nothing is published on any ``enable``
topic. With ``use_sim:=false`` it stays disengaged, as
``policy.launch.py`` leaves it.

Like ``policy.launch.py``, this starts no MoveIt Servo: the server's JointJog output
reaches the arm only through a Servo the run brings up elsewhere.
"""

import re

from fm_teleop_policy.profile import load_profile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Same rule fm-sim applies to every name that becomes a path, topic or frame.
NAME = re.compile(r"[A-Za-z][A-Za-z0-9_]*")
USE_SIM_VALUES = ("true", "false")


def _checked_name(argument, value):
    # The raw value is not echoed: it is caller input and may be anything.
    if not NAME.fullmatch(value):
        raise ValueError(f"{argument} must be a name matching {NAME.pattern}")
    return value


def _launch_setup(context):
    use_sim = LaunchConfiguration("use_sim").perform(context)
    if use_sim not in USE_SIM_VALUES:
        raise ValueError(f"use_sim must be one of: {', '.join(USE_SIM_VALUES)}")
    robot_profile = _checked_name(
        "robot_profile", LaunchConfiguration("robot_profile").perform(context)
    )
    _checked_name("scenario", LaunchConfiguration("scenario").perform(context))
    # Fails here, naming the shipped profiles, rather than in the node after launch.
    load_profile(robot_profile)

    in_sim = use_sim == "true"
    return [
        Node(
            package="fm_teleop_policy",
            executable="policy_server",
            name="policy_server",
            output="screen",
            parameters=[
                {
                    "robot_profile": robot_profile,
                    "use_sim_time": in_sim,
                    "engage_on_start": in_sim,
                    # The server's second engage guard; set here by convention, not enforced.
                    "sim_only": True,
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim", default_value="false",
                description="'true' or 'false'; sets use_sim_time and engage_on_start.",
            ),
            DeclareLaunchArgument(
                "robot_profile",
                description="Robot profile naming the joints the policy acts in.",
            ),
            DeclareLaunchArgument(
                "scenario", default_value="empty",
                description="Scenario of the run; validated, not used by the server.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
