"""Smoke test: every module in the package imports without hardware or a bus."""

import importlib

import pytest


@pytest.mark.parametrize(
    "module",
    ["fm_teleop_leader.follow", "fm_teleop_leader.sts3215",
     "fm_teleop_leader.leader_source", "fm_teleop_leader.leader_driver"],
)
def test_import_module(module):
    importlib.import_module(module)
