from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "classify-so101-leader-motor-variant.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("classify_so101_leader_motor_variant", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_classify_variant_buckets_and_ambiguity():
    module = load_module()
    thresholds = module.load_thresholds(str(Path(__file__).resolve().parents[1] / "config" / "sts3215_variant_thresholds.json"))

    code, status, confidence = module.classify_variant(0.34, thresholds, travel_counts=1536)
    assert (code, status) == ("C046", "classified")
    assert confidence > 0.5

    code, status, confidence = module.classify_variant(0.42, thresholds, travel_counts=1536)
    assert (code, status) == ("C044", "classified")
    assert confidence > 0.5

    code, status, confidence = module.classify_variant(0.56, thresholds, travel_counts=1536)
    assert (code, status) == ("C001", "classified")
    assert confidence > 0.5

    code, status, confidence = module.classify_variant(0.39, thresholds, travel_counts=1536)
    assert code is None
    assert status == "ambiguous"
    assert confidence == 0.0


def test_select_targets_guards():
    module = load_module()
    with pytest.raises(module.ProbeError, match="Expected exactly one connected motor"):
        module.select_targets({1: 777, 2: 777}, requested_ids=None, all_motors=False)

    targets = module.select_targets({3: 777}, requested_ids=None, all_motors=False)
    assert [(target.motor_id, target.model_number) for target in targets] == [(3, 777)]


def test_determine_probe_window_shrinks_to_safe_rom():
    module = load_module()
    thresholds = module.load_thresholds(str(Path(__file__).resolve().parents[1] / "config" / "sts3215_variant_thresholds.json"))

    class FakeBus:
        def __init__(self):
            self.values = {
                "Present_Position": 760,
                "Min_Position_Limit": 500,
                "Max_Position_Limit": 1080,
            }

        def read(self, data_name, *_args, **_kwargs):
            return self.values[data_name]

    probe_window = module.determine_probe_window(FakeBus(), thresholds)
    assert probe_window.low_goal >= 500
    assert probe_window.high_goal <= 1080
    assert probe_window.travel_counts == (probe_window.high_goal - probe_window.low_goal)
    assert module.MIN_TRAVEL_COUNTS <= probe_window.travel_counts < thresholds["travel_counts"]


def test_run_variant_probe_times_out_when_motor_never_settles():
    module = load_module()
    thresholds = module.load_thresholds(str(Path(__file__).resolve().parents[1] / "config" / "sts3215_variant_thresholds.json"))

    class FakeBus:
        def write(self, *_args, **_kwargs):
            return None

        def read(self, *_args, **_kwargs):
            return 0

        def enable_torque(self):
            return None

        def disable_torque(self):
            return None

    with pytest.raises(module.ProbeError, match="did not settle"):
        module.move_and_time(
            FakeBus(),
            123,
            settle_timeout_s=0.05,
            position_tolerance=2,
            consecutive_samples=2,
            sample_interval_s=0.0,
        )


def test_payload_shape_for_electronic_only():
    module = load_module()
    electronic = module.ElectronicProbe(
        port="/dev/tty.usbmodem-test",
        baudrate=1_000_000,
        motor_id=1,
        model_number=777,
        firmware_version="3.10",
        family="sts3215",
        signature={"Model_Number": 777},
    )
    payload = module.to_payload(electronic, None, requested_classify=False)
    assert payload["status"] == "electronic_only"
    assert payload["model_number"] == 777
    assert payload["trial_times"] == []
    assert payload["classification"] is None
    assert "https://libraries.io/pypi/sts-suite" in payload["manual_fallback"]
    assert "/dev/tty.usbmodem-test" in payload["manual_fallback"]


def test_sts_suite_fallback_mentions_install_and_command():
    module = load_module()
    message = module.sts_suite_fallback("/dev/tty.usbmodem-demo")
    assert "https://libraries.io/pypi/sts-suite" in message
    assert "uv tool install sts-suite" in message
    assert "python3 -m pip install sts-suite" in message
    assert "sts" in message
    assert "/dev/tty.usbmodem-demo" in message
