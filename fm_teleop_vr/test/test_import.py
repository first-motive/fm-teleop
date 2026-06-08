"""Smoke test: skeleton imports, and instantiating raises a clear NotImplementedError."""

import importlib

import pytest


def test_import_module():
    importlib.import_module("fm_teleop_vr.vr_source")


def test_instantiation_raises_not_implemented():
    from fm_teleop_vr.vr_source import VrSource

    with pytest.raises(NotImplementedError):
        VrSource()
