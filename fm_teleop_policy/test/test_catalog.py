"""The fm-sim catalog entries this package installs, read back through the ament index.

``catalog.schema.json`` is a verbatim copy of fm-sim's
``fm_sim_bringup/schema/catalog.schema.json`` at commit
262e6290a424637da6b0f3262f403ea13acefa37 (first-motive/fm-sim#41). It is vendored
because this repo's CI does not build fm-sim; refresh it when that schema changes.
Each check below is one fm-sim's ``catalog.load`` makes on every entry it reads.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.resources import get_resource, get_resources
import jsonschema
import pytest
import yaml

PACKAGE = "fm_teleop_policy"
RESOURCE_TYPE = "fm_sim.controllers"
SCHEMA_PATH = os.path.join(os.path.dirname(__file__), "catalog.schema.json")
# Entry name -> whether run.launch.py may start it.
EXPECTED = {"policy_serve": False, "fastwam": False}


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


def test_policy_serve_needs_the_trajectory_role():
    # Servo streams the profile's trajectory_controller (fm_openarm_v2:
    # openarm_*_arm_controller), which the openarm robot entry files under this role.
    assert _load("policy_serve")["activate"] == ["arm_trajectory"]


def test_fastwam_is_a_placeholder():
    entry = _load("fastwam")
    assert entry["launch"] is None
    assert entry["activate"] == []
