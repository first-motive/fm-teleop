"""Smoke test: every module imports without torch, lerobot, or a checkpoint."""

import importlib

import pytest


@pytest.mark.parametrize(
    "module",
    ["fm_teleop_policy.policies", "fm_teleop_policy.profile", "fm_teleop_policy.server"],
)
def test_import_module(module):
    importlib.import_module(module)
