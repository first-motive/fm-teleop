"""The fm-sim catalog entries this package installs, read back through the ament index.

``catalog.schema.json`` is a verbatim copy of fm-sim's
``fm_sim_bringup/schema/catalog.schema.json`` at commit
4721f383019afbaeea1e087ddb224918f4f76d88 (first-motive/fm-sim#106, run params).
It is vendored because this repo's CI does not build fm-sim. Only its
``controller`` definition (and the ``share_file``, ``entry``, ``param`` and
``param_target`` definitions it references) is used here, so
later fm-sim changes to other definitions, such as robot ``launch_roles``, do not
affect these tests; refresh the copy when the controller definition changes.
Each check below is one fm-sim's ``catalog.load`` makes on every entry it reads.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.resources import get_resource, get_resources
import jsonschema
import pytest
import rclpy
import yaml

from fm_teleop_policy.server import PolicyServer

PACKAGE = "fm_teleop_policy"
RESOURCE_TYPE = "fm_sim.controllers"
SCHEMA_PATH = os.path.join(os.path.dirname(__file__), "catalog.schema.json")
# Entry name -> whether run.launch.py may start it.
EXPECTED = {"policy_serve": True, "fastwam": False}


def _ours():
    return sorted(
        name
        for name in get_resources(RESOURCE_TYPE)
        if get_resource(RESOURCE_TYPE, name)[0].strip().startswith(PACKAGE + "/")
    )


def _load(name):
    marker, prefix = get_resource(RESOURCE_TYPE, name)
    package, path_in_package = marker.strip().split("/", 1)
    with open(os.path.join(prefix, "share", package, path_in_package)) as handle:
        entry = yaml.safe_load(handle)
    entry["package"] = package
    return entry


@pytest.fixture(scope="module")
def controller_schema():
    with open(SCHEMA_PATH) as handle:
        schema = json.load(handle)
    return {**schema, "$ref": "#/definitions/controller"}


def test_installs_exactly_the_expected_markers():
    assert _ours() == sorted(EXPECTED)


@pytest.mark.parametrize("name", sorted(EXPECTED))
def test_marker_points_at_this_packages_entry_yaml(name):
    marker, _ = get_resource(RESOURCE_TYPE, name)
    assert marker.strip() == f"{PACKAGE}/catalog/controllers/{name}.yaml"


@pytest.mark.parametrize("name", sorted(EXPECTED))
def test_entry_validates_and_names_itself(name, controller_schema):
    entry = _load(name)
    jsonschema.validate(entry, controller_schema)
    assert entry["name"] == name
    assert entry["package"] == PACKAGE
    assert entry["available"] is EXPECTED[name]


def test_policy_serve_launch_is_installed():
    launch = _load("policy_serve")["launch"]
    assert launch["package"] == PACKAGE
    assert os.path.isfile(
        os.path.join(get_package_share_directory(PACKAGE), launch["path"])
    )


def test_policy_serve_needs_servo_and_the_trajectory_role():
    # The server's JointJog reaches the arm only through MoveIt Servo (the robot's
    # servo role), which streams the profile's trajectory_controller (fm_openarm_v2:
    # openarm_*_arm_controller), filed by the openarm robot entry under arm_trajectory.
    assert _load("policy_serve")["activate"] == ["arm_trajectory", "servo"]


def test_fastwam_is_a_placeholder():
    entry = _load("fastwam")
    assert entry["launch"] is None
    assert entry["activate"] == []


def test_policy_serve_tunable_defaults_lie_within_their_bounds():
    for name, param in _load("policy_serve")["params"].items():
        assert param["min"] <= param["default"] <= param["max"], name


def test_policy_serve_tunables_target_the_servers_own_parameters():
    # Each target must name a parameter the server declares, with the same default,
    # or a run's override would be silently ignored or move the unedited baseline.
    rclpy.init()
    try:
        server = PolicyServer()
        for name, param in _load("policy_serve")["params"].items():
            target = param["target"]
            assert target["kind"] == "ros_param", name
            assert target["node"] == server.get_fully_qualified_name(), name
            assert server.get_parameter(target["param"]).value == param["default"], name
        server.destroy_node()
    finally:
        rclpy.shutdown()


def _without(param, *keys):
    return {key: value for key, value in param.items() if key not in keys}


RATE = {
    "label": "Rate", "type": "float", "default": 1.0, "min": 0.5, "max": 2.0,
    "target": {"kind": "ros_param", "node": "/policy_server", "param": "rate_hz"},
}
MODE = {**_without(RATE, "min", "max"), "type": "enum", "default": "a", "enum": ["a"]}
GAINS = {**RATE, "type": "array", "items": "float", "length": 2, "default": [1.0, 1.0]}


def test_the_schema_fixtures_are_valid(controller_schema):
    entry = _load("policy_serve")
    entry["params"] = {"rate": RATE, "mode": MODE, "gains": GAINS}
    jsonschema.validate(entry, controller_schema)


@pytest.mark.parametrize("param", [
    pytest.param({**RATE, "target": {"kind": "topic", "name": "/x"}}, id="unknown-target-kind"),
    pytest.param(_without(RATE, "min"), id="float-without-min"),
    pytest.param(_without(MODE, "enum"), id="enum-without-enum"),
    pytest.param(_without(GAINS, "length"), id="array-without-length"),
    pytest.param({**RATE, "step": 0}, id="zero-step"),
    pytest.param(
        {**RATE, "target": {**RATE["target"], "node": "policy_server"}},
        id="node-not-fully-qualified",
    ),
])
def test_the_schema_refuses_a_malformed_tunable(controller_schema, param):
    entry = _load("policy_serve")
    entry["params"] = {"rate": param}
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(entry, controller_schema)
