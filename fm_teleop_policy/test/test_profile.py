"""Robot profiles: the shipped files meet the shared schema, and the loader's errors."""

import os

import pytest
import yaml

from fm_teleop_policy.profile import arm_joints, load_profile, validate_profile

SHIPPED = os.path.join(os.path.dirname(__file__), "..", "robot_profiles")
NAMES = ("loader_openarm_v2", "fm_openarm_v2")


@pytest.mark.parametrize("name", NAMES)
def test_shipped_profile_meets_the_schema_with_both_arms(name):
    profile = load_profile(name, SHIPPED)
    assert profile["name"] == name
    assert sorted(profile["arms"]) == ["left", "right"]
    for arm in ("left", "right"):
        assert len(arm_joints(profile, arm)) == 7


@pytest.mark.parametrize(
    ("name", "first_right_joint"),
    [("loader_openarm_v2", "follower_r_joint1"), ("fm_openarm_v2", "openarm_right_joint1")],
)
def test_each_profile_names_its_own_scheme(name, first_right_joint):
    assert arm_joints(load_profile(name, SHIPPED), "right")[0] == first_right_joint


def test_joints_come_back_in_controller_order():
    joints = arm_joints(load_profile("loader_openarm_v2", SHIPPED), "right")
    assert joints == [f"follower_r_joint{index}" for index in range(1, 8)]


def test_an_unknown_profile_names_the_ones_that_exist():
    with pytest.raises(ValueError) as excinfo:
        load_profile("nope", SHIPPED)
    assert "fm_openarm_v2" in str(excinfo.value)
    assert "loader_openarm_v2" in str(excinfo.value)


def test_an_unknown_arm_names_the_ones_that_exist():
    with pytest.raises(ValueError) as excinfo:
        arm_joints(load_profile("fm_openarm_v2", SHIPPED), "middle")
    assert "left, right" in str(excinfo.value)


def _profile():
    with open(os.path.join(SHIPPED, "loader_openarm_v2.yaml"), encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def test_a_missing_top_level_key_is_rejected():
    profile = _profile()
    del profile["base_frame"]
    with pytest.raises(ValueError, match="base_frame"):
        validate_profile(profile)


def test_a_missing_arm_key_is_rejected():
    profile = _profile()
    del profile["arms"]["left"]["ee_frame"]
    with pytest.raises(ValueError, match="arm 'left' is missing ee_frame"):
        validate_profile(profile)


@pytest.mark.parametrize("joints", [[], ["follower_r_joint1", ""], "follower_r_joint1"])
def test_blank_or_malformed_joints_are_rejected(joints):
    profile = _profile()
    profile["arms"]["right"]["joints"] = joints
    with pytest.raises(ValueError, match="non-blank joints"):
        validate_profile(profile)


def test_cameras_are_optional():
    profile = _profile()
    profile.pop("cameras", None)
    validate_profile(profile)


@pytest.mark.parametrize("name", ["../robot_profiles/fm_openarm_v2", "sub/fm_openarm_v2", ""])
def test_a_name_that_is_not_a_bare_stem_is_rejected(name):
    # A path in the name would read a yaml from outside the profile directory.
    with pytest.raises(ValueError, match="Unknown robot profile"):
        load_profile(name, SHIPPED)


def test_a_missing_profile_directory_is_the_documented_error(tmp_path):
    with pytest.raises(ValueError, match="One of: none"):
        load_profile("fm_openarm_v2", str(tmp_path / "absent"))


@pytest.mark.parametrize("entry", [None, ["joints"], "follower_r_joint1"])
def test_an_arm_entry_that_is_not_a_mapping_is_rejected(entry):
    profile = _profile()
    profile["arms"]["right"] = entry
    with pytest.raises(ValueError, match="arm 'right' must map"):
        validate_profile(profile)
