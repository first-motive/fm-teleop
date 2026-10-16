"""Robot profiles — the joint, controller, and frame names one robot description uses.

The same OpenArm runs under two naming schemes: anvil-loader's ``follower_*`` names on
the real robot today, and the canonical ``openarm_*`` names fm-robot's description and
the simulators use. A profile yaml names one scheme, so a consumer picks a robot by
name instead of carrying either set of names in its code:

    name: loader_openarm_v2
    base_frame: follower_body_link0
    arms:
      right: {joints, gripper_joint, arm_controller, trajectory_controller,
              ee_frame, hand_frame}
      left:  {...}
    cameras: {...}        # optional

fm-skills and anvil-embodied-ai read the same schema, so its keys are shared and must
not drift. Profiles ship in this package's ``share/fm_teleop_policy/robot_profiles``.
"""

import os
import re

import yaml

PROFILE_NAME = re.compile(r"[A-Za-z0-9_]+")
PROFILE_KEYS = ("name", "base_frame", "arms")
ARM_KEYS = (
    "joints",
    "gripper_joint",
    "arm_controller",
    "trajectory_controller",
    "ee_frame",
    "hand_frame",
)


def profile_directory():
    """Return the installed directory the shipped profiles live in."""
    # Imported here so the loader and its tests run without a sourced ROS install.
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(get_package_share_directory("fm_teleop_policy"), "robot_profiles")


def load_profile(name, directory=None):
    """Load and validate the profile ``<directory>/<name>.yaml``.

    ``directory`` defaults to the installed profiles. Raises ValueError for an unknown
    name, naming the profiles that exist, and for a file that breaks the schema.
    """
    directory = directory or profile_directory()
    # A name is a bare file stem: a separator or ".." would read a yaml from outside
    # the profile directory.
    valid_name = PROFILE_NAME.fullmatch(name or "")
    path = os.path.join(directory, f"{name}.yaml")
    if not valid_name or not os.path.isfile(path):
        known = (
            sorted(
                entry[: -len(".yaml")]
                for entry in os.listdir(directory)
                if entry.endswith(".yaml")
            )
            if os.path.isdir(directory)
            else []
        )
        raise ValueError(
            f"Unknown robot profile '{name}' in {directory}. "
            f"One of: {', '.join(known) or 'none'}."
        )
    with open(path, encoding="utf-8") as handle:
        profile = yaml.safe_load(handle)
    validate_profile(profile, source=path)
    return profile


def validate_profile(profile, source="profile"):
    """Raise ValueError when ``profile`` is missing a key the shared schema requires."""
    if not isinstance(profile, dict):
        raise ValueError(f"{source}: a robot profile is a mapping.")
    missing = [key for key in PROFILE_KEYS if key not in profile]
    if missing:
        raise ValueError(f"{source}: missing {', '.join(missing)}.")
    if not isinstance(profile["arms"], dict) or not profile["arms"]:
        raise ValueError(f"{source}: 'arms' must map at least one arm name to its names.")
    for arm, names in profile["arms"].items():
        if not isinstance(names, dict):
            raise ValueError(f"{source}: arm '{arm}' must map each name key to a value.")
        missing = [key for key in ARM_KEYS if key not in names]
        if missing:
            raise ValueError(f"{source}: arm '{arm}' is missing {', '.join(missing)}.")
        joints = names["joints"]
        if not isinstance(joints, list) or not joints or not all(joints):
            raise ValueError(f"{source}: arm '{arm}' needs a list of non-blank joints.")


def arm_joints(profile, arm):
    """Return ``arm``'s joints in controller order, raising ValueError for an unknown arm."""
    arms = profile["arms"]
    if arm not in arms:
        raise ValueError(
            f"Robot profile '{profile['name']}' has no arm '{arm}'. "
            f"One of: {', '.join(sorted(arms))}."
        )
    return list(arms[arm]["joints"])
