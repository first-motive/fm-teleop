"""Smoke test: package modules import cleanly."""

import importlib


def test_import_modules():
    importlib.import_module("fm_teleop_device.joy_to_servo")
    importlib.import_module("fm_teleop_device.spacenav_to_servo")
    importlib.import_module("fm_teleop_device.g1_hand_teleop")
    importlib.import_module("fm_teleop_device.hand_presets")
