"""The sim include launch, loaded from the installed share and set up as fm-sim would.

fm-sim's run.launch.py includes it with ``use_sim``, ``robot_profile``,
``scenario`` and ``params_file``; these tests drive its setup function with those
arguments.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch_ros.actions import Node
import pytest
import rclpy
import yaml

from fm_teleop_policy.server import PolicyServer


@pytest.fixture(scope="module")
def sim_launch():
    path = os.path.join(
        get_package_share_directory("fm_teleop_policy"), "launch", "sim.launch.py"
    )
    spec = importlib.util.spec_from_file_location("fm_teleop_policy_sim_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def host_local(monkeypatch):
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    monkeypatch.delenv("ROS_LOCALHOST_ONLY", raising=False)


@pytest.fixture
def not_host_local(monkeypatch):
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET")
    monkeypatch.delenv("ROS_LOCALHOST_ONLY", raising=False)


def _declared(sim_launch):
    return {
        action.name: action
        for action in sim_launch.generate_launch_description().entities
        if hasattr(action, "default_value")
    }


def _setup(sim_launch, **arguments):
    """Run the setup function with the launch file's defaults plus ``arguments``."""
    context = LaunchContext()
    for name, declared in _declared(sim_launch).items():
        if declared.default_value is not None:
            context.launch_configurations[name] = declared.default_value[0].text
    context.launch_configurations.update(arguments)
    return context, sim_launch._launch_setup(context)


def _expanded_files(node, context):
    """Return the parameter files launch_ros passes the node, in order."""
    # launch_ros has no public getter for a node's parameters. It writes dict
    # parameters to a yaml file and records it in a private field, read here.
    node._perform_substitutions(context)
    expanded = getattr(node, "_Node__expanded_parameter_arguments", None)
    assert expanded is not None, (
        "launch_ros no longer has Node.__expanded_parameter_arguments; "
        "update this helper for the installed launch_ros"
    )
    assert all(is_file for _, is_file in expanded)
    return [path for path, _ in expanded]


def _parameters(node, context):
    parameters = {}
    for path in _expanded_files(node, context):
        with open(path) as handle:
            parameters.update(yaml.safe_load(handle)["/**"]["ros__parameters"])
    return parameters


def test_declares_the_run_arguments(sim_launch):
    assert set(_declared(sim_launch)) == {
        "use_sim", "robot_profile", "scenario", "params_file"
    }


def test_params_file_defaults_to_empty(sim_launch):
    assert _declared(sim_launch)["params_file"].default_value[0].text == ""


def test_use_sim_defaults_to_false_and_robot_profile_is_required(sim_launch):
    declared = _declared(sim_launch)
    assert declared["use_sim"].default_value[0].text == "false"
    assert declared["robot_profile"].default_value is None


def test_by_default_the_server_runs_on_wall_time_and_stays_disengaged(
    sim_launch, not_host_local
):
    context, (node,) = _setup(sim_launch, robot_profile="fm_openarm_v2")
    assert isinstance(node, Node)
    assert node.node_package == "fm_teleop_policy"
    assert node.node_executable == "policy_server"
    assert _parameters(node, context) == {
        "robot_profile": "fm_openarm_v2",
        "use_sim_time": False,
        "engage_on_start": False,
        "sim_only": True,
    }


def test_a_sim_run_engages_the_server_through_its_own_parameter(sim_launch, host_local):
    context, (node,) = _setup(sim_launch, use_sim="true", robot_profile="fm_openarm_v2")
    assert _parameters(node, context) == {
        "robot_profile": "fm_openarm_v2",
        "use_sim_time": True,
        "engage_on_start": True,
        "sim_only": True,
    }


@pytest.mark.parametrize("use_sim", ["true", "false"])
def test_nothing_but_the_server_is_started(sim_launch, host_local, use_sim):
    # Engaging by publishing on an enable topic could reach another server on a
    # shared graph; the only action is this server's own node.
    _, actions = _setup(sim_launch, use_sim=use_sim, robot_profile="fm_openarm_v2")
    # Exact type: a plain ExecuteProcess (say a `ros2 topic pub`) would not match Node.
    assert [type(action) for action in actions] == [Node]


@pytest.mark.parametrize("range_", ["SUBNET", "OFF", "SYSTEM_DEFAULT", None])
def test_a_sim_run_refuses_to_engage_unless_discovery_is_host_local(
    sim_launch, monkeypatch, range_
):
    monkeypatch.delenv("ROS_LOCALHOST_ONLY", raising=False)
    if range_ is None:
        monkeypatch.delenv("ROS_AUTOMATIC_DISCOVERY_RANGE", raising=False)
    else:
        monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", range_)
    with pytest.raises(RuntimeError, match="refusing to engage on start"):
        _setup(sim_launch, use_sim="true", robot_profile="fm_openarm_v2")


def test_a_sim_run_accepts_ros_localhost_only(sim_launch, not_host_local, monkeypatch):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    context, (node,) = _setup(sim_launch, use_sim="true", robot_profile="fm_openarm_v2")
    assert _parameters(node, context)["engage_on_start"] is True


@pytest.mark.parametrize("value", ["", "True", "yes", "1"])
def test_refuses_a_use_sim_other_than_true_or_false(sim_launch, value):
    with pytest.raises(ValueError, match="use_sim must be one of: true, false$"):
        _setup(sim_launch, use_sim=value, robot_profile="fm_openarm_v2")


@pytest.mark.parametrize("argument", ["robot_profile", "scenario"])
@pytest.mark.parametrize("value", ["", "../fm_openarm_v2", "a/b", "9lives", "x y"])
def test_refuses_a_malformed_name_without_echoing_it(sim_launch, argument, value):
    arguments = {"robot_profile": "fm_openarm_v2", argument: value}
    with pytest.raises(ValueError, match=f"^{argument} must be a name") as raised:
        _setup(sim_launch, **arguments)
    assert repr(value) not in str(raised.value)


def test_refuses_an_unknown_robot_profile(sim_launch):
    with pytest.raises(ValueError, match="Unknown robot profile 'no_such_robot'"):
        _setup(sim_launch, robot_profile="no_such_robot")



def _server_from_files(sim_launch, tmp_path, overrides):
    """Return the real server, built from the files the launch passes, in order."""
    params_file = tmp_path / "params.yaml"
    # Keyed by the node's name, as the fm-sim agent writes the run's file.
    params_file.write_text(yaml.safe_dump(
        {"/policy_server": {"ros__parameters": overrides}}
    ))
    context, (node,) = _setup(
        sim_launch, robot_profile="fm_openarm_v2", params_file=str(params_file)
    )
    files = _expanded_files(node, context)
    assert str(params_file) in files
    # rcl, not this test, decides precedence: the server reads the files in the
    # order launch_ros passes them.
    arguments = ["--ros-args"]
    for path in files:
        arguments += ["--params-file", path]
    return PolicyServer(cli_args=arguments)


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_a_params_file_override_beats_the_servers_own_value(sim_launch, tmp_path, ros):
    server = _server_from_files(sim_launch, tmp_path, {"rate_hz": 25.0})
    assert server.get_parameter("rate_hz").value == 25.0
    server.destroy_node()


def test_a_params_file_cannot_change_a_launch_owned_parameter(
    sim_launch, tmp_path, ros, not_host_local
):
    # use_sim is false, so this file would let a wall-time server self-engage.
    server = _server_from_files(sim_launch, tmp_path, {
        "use_sim_time": True,
        "engage_on_start": True,
        "sim_only": False,
        "robot_profile": "loader_openarm_v2",
        "arm": "left",
        "policy_type": "lerobot",
        "checkpoint": "/elsewhere",
        "observation_topic": "/other_states",
        "command_topic": "/other_cmds",
        "enable_topic": "/other_enable",
        "status_topic": "/other_status",
        "rate_hz": 25.0,
    })
    assert {
        name: server.get_parameter(name).value
        for name in (
            "use_sim_time", "engage_on_start", "sim_only", "robot_profile",
            *sim_launch.PINNED,
        )
    } == {
        "use_sim_time": False,
        "engage_on_start": False,
        "sim_only": True,
        "robot_profile": "fm_openarm_v2",
        **sim_launch.PINNED,
    }
    assert server.get_parameter("rate_hz").value == 25.0
    server.destroy_node()
