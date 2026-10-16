"""Policy backends: the loader's contract, the zero backend's shape, action checks."""

import math

import pytest

from fm_teleop_policy.policies import check_action, LeRobotPolicy, ZeroPolicy, load_policy

JOINTS = ["shoulder_pan", "elbow_flex"]


def test_zero_policy_returns_a_one_action_chunk_of_joint_width():
    chunk = load_policy("zero", "", JOINTS).act({"observation.state": {}})
    assert chunk == [[0.0, 0.0]]


def test_zero_policy_needs_action_names():
    with pytest.raises(ValueError):
        load_policy("zero", "", [])


def test_unknown_policy_type_names_the_ones_that_exist(tmp_path):
    with pytest.raises(ValueError) as excinfo:
        load_policy("groot", str(tmp_path), JOINTS)
    assert "zero" in str(excinfo.value)


def test_lerobot_policy_needs_a_checkpoint():
    with pytest.raises(ValueError):
        load_policy("lerobot", "", JOINTS)


def test_lerobot_policy_reports_a_missing_checkpoint_directory(tmp_path):
    # A checkpoint is a directory (config + weights), so a path that is not one is an
    # error worth naming before any import of torch is attempted.
    missing = tmp_path / "nope"
    with pytest.raises(FileNotFoundError):
        LeRobotPolicy(str(missing)).load()


def test_lerobot_policy_refuses_to_act_before_load(tmp_path):
    with pytest.raises(RuntimeError):
        LeRobotPolicy(str(tmp_path)).act({})


def test_lerobot_policy_needs_action_names(tmp_path):
    # The state vector and the action vector are both in the follower's joint order, so
    # the backend cannot build either without it.
    with pytest.raises(ValueError):
        LeRobotPolicy(str(tmp_path)).load()


def test_zero_policy_carries_its_action_names():
    assert ZeroPolicy(JOINTS).action_names == JOINTS


def test_a_well_formed_action_passes():
    assert check_action([0.1, -2], 2) is None
    assert check_action((0.0, 0.0), 2) is None


@pytest.mark.parametrize(
    ("action", "reason"),
    [
        ([0.1, 0.2, 0.3], "3 values for 2 joints"),
        (0.1, "not a list"),
        ("ab", "not a list"),
        ([0.1, "0.2"], "not a real number"),
        ([0.1, True], "not a real number"),
        ([0.1, None], "not a real number"),
        ([0.1, math.nan], "non-finite"),
        ([math.inf, 0.1], "non-finite"),
    ],
)
def test_a_malformed_action_says_why(action, reason):
    assert reason in check_action(action, 2)
