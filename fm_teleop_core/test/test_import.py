"""Smoke test: package modules import cleanly."""

import importlib


def test_import_modules():
    importlib.import_module("fm_teleop_core.contract")
    importlib.import_module("fm_teleop_core.retarget")
    importlib.import_module("fm_teleop_core.source")
