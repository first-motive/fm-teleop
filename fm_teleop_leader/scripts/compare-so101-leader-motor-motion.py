#!/usr/bin/env python3
"""Run a repeatable motion pattern for manual SO-101 leader motor comparison.

Use this when automatic classification is not trustworthy enough and you want to
compare motors by eye or by the printed timing summary. The script:

1. confirms a single STS3215-family motor is connected
2. picks a safe motion window around the motor's current position
3. runs the same left -> center -> right -> center pattern every time
4. returns the motor to center and disables torque

Interpretation for your current leader-arm sort:
- faster side sweeps usually indicate C046
- slower side sweeps usually indicate C044
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import statistics
import sys
import time
from pathlib import Path
from types import ModuleType
from typing import Any

DEFAULT_BAUDRATE = 1_000_000
DEFAULT_TIMEOUT_S = 1.0
DEFAULT_RETRIES = 3
DEFAULT_SETTLE_TIMEOUT_S = 1.5
DEFAULT_CYCLES = 4
DEFAULT_TRAVEL_COUNTS = 1536
DEFAULT_PAUSE_S = 0.15


def load_classifier_module() -> ModuleType:
    script_path = Path(__file__).with_name("classify-so101-leader-motor-variant.py")
    spec = importlib.util.spec_from_file_location("classify_so101_leader_motor_variant", script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load helper module from {script_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a repeatable motion pattern for manual SO-101 leader motor comparison."
    )
    parser.add_argument("--port", help="Serial port of the Feetech bus. Defaults to auto-detect.")
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help="Bus baudrate. Default: 1000000.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help="Serial read timeout in seconds. Default: 1.0.",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=DEFAULT_RETRIES,
        help="Retries per ping/write/read. Default: 3.",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=DEFAULT_CYCLES,
        help="How many left/center/right/center cycles to run. Default: 4.",
    )
    parser.add_argument(
        "--travel-counts",
        type=int,
        default=DEFAULT_TRAVEL_COUNTS,
        help="Requested total sweep span in raw counts before adaptive shrinking. Default: 1536.",
    )
    parser.add_argument(
        "--settle-timeout",
        type=float,
        default=DEFAULT_SETTLE_TIMEOUT_S,
        help="Max seconds to wait for each move to settle. Default: 1.5.",
    )
    parser.add_argument(
        "--pause",
        type=float,
        default=DEFAULT_PAUSE_S,
        help="Pause in seconds between settled moves. Default: 0.15.",
    )
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON only.")
    return parser.parse_args()


def move_and_pause(
    helper: ModuleType,
    bus: Any,
    goal: int,
    *,
    settle_timeout_s: float,
    position_tolerance: int,
    consecutive_samples: int,
    pause_s: float,
) -> float:
    elapsed = helper.move_and_time(
        bus,
        goal,
        settle_timeout_s=settle_timeout_s,
        position_tolerance=position_tolerance,
        consecutive_samples=consecutive_samples,
    )
    if pause_s > 0:
        time.sleep(pause_s)
    return elapsed


def run_motion_pattern(
    helper: ModuleType,
    bus: Any,
    probe_window: Any,
    *,
    cycles: int,
    settle_timeout_s: float,
    pause_s: float,
) -> dict[str, Any]:
    position_tolerance = 12
    consecutive_samples = 3
    left_times: list[float] = []
    right_times: list[float] = []
    center_return_times: list[float] = []

    bus.write("Acceleration", "motor", 254, normalize=False)
    bus.write("Goal_Time", "motor", 0, normalize=False)
    bus.enable_torque()
    try:
        move_and_pause(
            helper,
            bus,
            probe_window.present_position,
            settle_timeout_s=settle_timeout_s,
            position_tolerance=position_tolerance,
            consecutive_samples=consecutive_samples,
            pause_s=pause_s,
        )
        for _ in range(cycles):
            left_times.append(
                move_and_pause(
                    helper,
                    bus,
                    probe_window.low_goal,
                    settle_timeout_s=settle_timeout_s,
                    position_tolerance=position_tolerance,
                    consecutive_samples=consecutive_samples,
                    pause_s=pause_s,
                )
            )
            center_return_times.append(
                move_and_pause(
                    helper,
                    bus,
                    probe_window.present_position,
                    settle_timeout_s=settle_timeout_s,
                    position_tolerance=position_tolerance,
                    consecutive_samples=consecutive_samples,
                    pause_s=pause_s,
                )
            )
            right_times.append(
                move_and_pause(
                    helper,
                    bus,
                    probe_window.high_goal,
                    settle_timeout_s=settle_timeout_s,
                    position_tolerance=position_tolerance,
                    consecutive_samples=consecutive_samples,
                    pause_s=pause_s,
                )
            )
            center_return_times.append(
                move_and_pause(
                    helper,
                    bus,
                    probe_window.present_position,
                    settle_timeout_s=settle_timeout_s,
                    position_tolerance=position_tolerance,
                    consecutive_samples=consecutive_samples,
                    pause_s=pause_s,
                )
            )
    finally:
        try:
            helper.move_and_time(
                bus,
                probe_window.present_position,
                settle_timeout_s=settle_timeout_s,
                position_tolerance=position_tolerance,
                consecutive_samples=consecutive_samples,
            )
        except Exception:
            pass
        bus.disable_torque()

    side_times = left_times + right_times
    return {
        "left_times": [round(value, 4) for value in left_times],
        "right_times": [round(value, 4) for value in right_times],
        "center_return_times": [round(value, 4) for value in center_return_times],
        "median_left_time": round(statistics.median(left_times), 4),
        "median_right_time": round(statistics.median(right_times), 4),
        "median_side_time": round(statistics.median(side_times), 4),
        "comparison_hint": (
            "Compare this motor against the majority group using the same command. "
            "Lower median side time usually means faster C046; higher median side time usually means slower C044."
        ),
    }


def main() -> int:
    args = parse_args()
    helper = load_classifier_module()

    try:
        port = helper.resolve_port(args.port)
        bus = helper.open_bus(port, args.baudrate, args.timeout)
        try:
            ids_to_models = helper.discover_motors(bus, args.retries)
            targets = helper.select_targets(ids_to_models, requested_ids=None, all_motors=False)
            target = targets[0]
            electronic = helper.run_electronic_probe(
                bus,
                port=port,
                baudrate=args.baudrate,
                target=target,
            )
            if electronic.model_number != helper.EXPECTED_MODEL_NUMBER:
                raise helper.ProbeError(
                    f"Expected STS3215 model number {helper.EXPECTED_MODEL_NUMBER}, got {electronic.model_number}."
                )

            helper.point_bus_at_motor(bus, target.motor_id)
            requested_travel_counts = args.travel_counts
            probe_window = None
            motion = None
            last_error = None
            while requested_travel_counts >= helper.MIN_TRAVEL_COUNTS:
                probe_window = helper.determine_probe_window_for_travel_counts(
                    bus,
                    requested_travel_counts=requested_travel_counts,
                )
                try:
                    motion = run_motion_pattern(
                        helper,
                        bus,
                        probe_window,
                        cycles=args.cycles,
                        settle_timeout_s=args.settle_timeout,
                        pause_s=args.pause,
                    )
                    break
                except helper.ProbeError as exc:
                    last_error = exc
                    requested_travel_counts //= 2
            if probe_window is None or motion is None:
                if last_error is not None:
                    raise last_error
                raise helper.ProbeError("Could not find a safe motion window for comparison.")
        finally:
            helper.close_bus(bus)
    except Exception as exc:
        payload = {
            "port": args.port,
            "baudrate": args.baudrate,
            "status": "failed",
            "next_step": str(exc),
        }
        if args.json:
            print(json.dumps(payload, indent=2))
        else:
            print(f"Status: failed\nNext Step: {payload['next_step']}")
        return 1

    payload = {
        "port": electronic.port,
        "baudrate": electronic.baudrate,
        "motor_id": electronic.motor_id,
        "model_number": electronic.model_number,
        "firmware_version": electronic.firmware_version,
        "family": electronic.family,
        "probe_window": {
            "low_goal": probe_window.low_goal,
            "center_goal": probe_window.present_position,
            "high_goal": probe_window.high_goal,
            "travel_counts": probe_window.travel_counts,
        },
        **motion,
        "status": "completed",
        "next_step": "Swap the next motor into the same centered position and rerun this exact command.",
        "manual_fallback": helper.sts_suite_fallback(electronic.port),
    }

    if args.json:
        print(json.dumps(payload, indent=2))
    else:
        print("SO-101 Leader Motor Manual Compare")
        print(f"Port: {payload['port']}")
        print(f"Motor ID: {payload['motor_id']}")
        print(f"Model Number: {payload['model_number']}")
        print(f"Firmware: {payload['firmware_version'] or 'unknown'}")
        print(f"Window: {payload['probe_window']}")
        print(f"Median left time: {payload['median_left_time']}s")
        print(f"Median right time: {payload['median_right_time']}s")
        print(f"Median side time: {payload['median_side_time']}s")
        print(payload["comparison_hint"])
        print(payload["next_step"])
        print(payload["manual_fallback"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
