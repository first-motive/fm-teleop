"""Smoke test: skeleton imports, and instantiating raises a clear NotImplementedError."""

import importlib

import pytest


def test_import_module():
    importlib.import_module("fm_teleop_leader.leader_source")


def test_instantiation_raises_not_implemented():
    from fm_teleop_leader.leader_source import LeaderSource

    with pytest.raises(NotImplementedError):
        LeaderSource()
