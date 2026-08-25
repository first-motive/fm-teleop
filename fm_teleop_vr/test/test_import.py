"""Smoke test: every module in the package imports without a headset or a bridge."""

import importlib

import pytest


@pytest.mark.parametrize("module", ["fm_teleop_vr.mapping", "fm_teleop_vr.vr_source"])
def test_import_module(module):
    importlib.import_module(module)
