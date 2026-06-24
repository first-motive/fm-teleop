#!/usr/bin/env python3
"""Classify SO-101 leader STS3215 motors as C001, C044, or C046.

This is a standalone host-native helper built on the same vendored LeRobot +
Feetech stack used elsewhere in this repo. It does two things:

1. confirm the motor is electronically an STS3215 family servo
2. classify the physical gearbox variant by timing a short position move

The classifier intentionally avoids permanent EEPROM writes. It only uses
short-lived position commands needed for the probe.
"""

from __future__ import annotations

import argparse
import json
import shutil
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorNormMode
except ImportError as exc:  # pragma: no cover - runtime dependency check
    FeetechMotorsBus = Any  # type: ignore[assignment]
    Motor = Any  # type: ignore[assignment]
    MotorNormMode = Any  # type: ignore[assignment]
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None

DEFAULT_BAUDRATE = 1_000_000
DEFAULT_TIMEOUT_S = 1.0
DEFAULT_RETRIES = 3
DEFAULT_SETTLE_TIMEOUT_S = 1.5
DEFAULT_CYCLES = 3
DEFAULT_SAMPLE_INTERVAL_S = 0.02
EXPECTED_MODEL_NUMBER = 777
EXPECTED_FAMILY = "sts3215"
RAW_CENTER = 2048
MIN_TRAVEL_COUNTS = 80
LIMIT_MARGIN_COUNTS = 32
DEFAULT_THRESHOLD_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "sts3215_variant_thresholds.json"
)
STS_SUITE_URL = "https://libraries.io/pypi/sts-suite"
STS_SUITE_INSTALL = "uv tool install sts-suite"
STS_SUITE_ALT_INSTALL = "python3 -m pip install sts-suite"


class ProbeError(RuntimeError):
    """Raised for workflow or protocol failures."""


@dataclass(frozen=True)
class ElectronicProbe:
    port: str
    baudrate: int
    motor_id: int
    model_number: int
    firmware_version: str | None
    family: str
    signature: dict[str, int | str | None]


@dataclass(frozen=True)
class ProbeTarget:
    motor_id: int
    model_number: int


@dataclass(frozen=True)
class ProbeWindow:
    low_goal: int
    high_goal: int
    travel_counts: int
    present_position: int
    min_limit: int
    max_limit: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Classify SO-101 leader STS3215 motors as C001, C044, or C046."
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
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON only.")
    parser.add_argument(
        "--electronic-only",
        action="store_true",
        help="Only confirm the electronic STS3215 family identity.",
    )
    parser.add_argument(
        "--classify",
        action="store_true",
        help="Run the active motion probe to classify C001/C044/C046.",
    )
    parser.add_argument(
        "--all-motors",
        action="store_true",
        help="Probe every responding motor on the daisy chain instead of requiring exactly one.",
    )
    parser.add_argument(
        "--ids",
        help="Comma-separated motor IDs to probe on the daisy chain. Example: --ids 1,2,3",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=DEFAULT_CYCLES,
        help="Forward/back probe cycles. Default: 3.",
    )
    parser.add_argument(
        "--settle-timeout",
        type=float,
        default=DEFAULT_SETTLE_TIMEOUT_S,
        help="Max seconds to wait for each move to settle. Default: 1.5.",
    )
    parser.add_argument(
        "--variant-thresholds",
        default=str(DEFAULT_THRESHOLD_PATH),
        help="Path to the STS3215 timing threshold JSON file.",
    )
    parser.add_argument(
        "--sts-suite-help",
        action="store_true",
        help="Print the optional sts-suite TUI fallback instructions and exit.",
    )
    return parser.parse_args()


def list_candidate_ports() -> list[str]:
    ports = sorted(Path("/dev").glob("tty*"))
    candidates = []
    for path in ports:
        name = path.name
        if "Bluetooth" in name or "debug-console" in name:
            continue
        if any(token in name for token in ("usbmodem", "ttyACM", "ttyUSB", "cu.usbmodem", "cu.usbserial")):
            candidates.append(str(path))
    return candidates


def resolve_port(explicit_port: str | None) -> str:
    if explicit_port:
        return explicit_port

    candidates = list_candidate_ports()
    if len(candidates) == 1:
        return candidates[0]

    raise ProbeError(
        "Could not auto-select a serial port. Pass --port explicitly or run lerobot-find-port."
    )


def open_bus(port: str, baudrate: int, timeout_s: float) -> FeetechMotorsBus:
    if IMPORT_ERROR is not None:
        raise ProbeError(
            "LeRobot + Feetech dependencies are required. Activate ~/.venvs/lerobot first."
        ) from IMPORT_ERROR
    bus = FeetechMotorsBus(
        port=port,
        motors={"motor": Motor(1, EXPECTED_FAMILY, MotorNormMode.DEGREES)},
    )
    bus._connect(handshake=False)
    bus.set_baudrate(baudrate)
    bus.set_timeout(int(timeout_s * 1000))
    return bus


def close_bus(bus: FeetechMotorsBus) -> None:
    try:
        bus.port_handler.closePort()
    except Exception:
        pass


def discover_motors(bus: FeetechMotorsBus, retries: int) -> dict[int, int]:
    ids_to_models = bus.broadcast_ping(num_retry=max(0, retries - 1), raise_on_error=False) or {}
    if not ids_to_models:
        raise ProbeError(
            "No motor responded at this baudrate. Check power, cabling, and external servo power."
        )
    return ids_to_models


def parse_requested_ids(raw_ids: str | None) -> list[int] | None:
    if raw_ids is None:
        return None

    requested = []
    for item in raw_ids.split(","):
        token = item.strip()
        if not token:
            continue
        try:
            requested.append(int(token))
        except ValueError as exc:
            raise ProbeError(f"Invalid motor ID in --ids: {token!r}") from exc

    if not requested:
        raise ProbeError("--ids was provided but no valid motor IDs were parsed.")

    deduped = sorted(set(requested))
    return deduped


def select_targets(
    ids_to_models: dict[int, int],
    *,
    requested_ids: list[int] | None,
    all_motors: bool,
) -> list[ProbeTarget]:
    discovered_ids = sorted(ids_to_models)
    if requested_ids is not None:
        missing = [motor_id for motor_id in requested_ids if motor_id not in ids_to_models]
        if missing:
            raise ProbeError(
                f"Requested motor IDs not found on the bus: {missing}. Discovered IDs: {discovered_ids}."
            )
        return [ProbeTarget(motor_id, ids_to_models[motor_id]) for motor_id in requested_ids]

    if all_motors:
        return [ProbeTarget(motor_id, ids_to_models[motor_id]) for motor_id in discovered_ids]

    if len(ids_to_models) != 1:
        raise ProbeError(
            "Expected exactly one connected motor for single-motor mode, "
            f"but found {len(ids_to_models)}: {ids_to_models}. Use --all-motors or --ids."
        )

    motor_id = discovered_ids[0]
    return [ProbeTarget(motor_id, ids_to_models[motor_id])]


def point_bus_at_motor(bus: FeetechMotorsBus, motor_id: int) -> None:
    bus.motors["motor"].id = motor_id


def safe_read(bus: FeetechMotorsBus, data_name: str) -> int | None:
    try:
        return int(bus.read(data_name, "motor", normalize=False))
    except Exception:
        return None


def run_electronic_probe(
    bus: FeetechMotorsBus,
    *,
    port: str,
    baudrate: int,
    target: ProbeTarget,
) -> ElectronicProbe:
    point_bus_at_motor(bus, target.motor_id)
    firmware_versions = bus._read_firmware_version([target.motor_id], raise_on_error=False)
    firmware = firmware_versions.get(target.motor_id)
    signature = {
        "ID": target.motor_id,
        "Model_Number": target.model_number,
        "Firmware_Version": firmware,
        "Baud_Rate": safe_read(bus, "Baud_Rate"),
        "Min_Position_Limit": safe_read(bus, "Min_Position_Limit"),
        "Max_Position_Limit": safe_read(bus, "Max_Position_Limit"),
        "Phase": safe_read(bus, "Phase"),
        "Operating_Mode": safe_read(bus, "Operating_Mode"),
        "Acceleration": safe_read(bus, "Acceleration"),
        "Max_Torque_Limit": safe_read(bus, "Max_Torque_Limit"),
    }
    return ElectronicProbe(
        port=port,
        baudrate=baudrate,
        motor_id=target.motor_id,
        model_number=target.model_number,
        firmware_version=firmware,
        family=EXPECTED_FAMILY if target.model_number == EXPECTED_MODEL_NUMBER else "unknown",
        signature=signature,
    )


def load_thresholds(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    required = {"travel_counts", "boundary_margin_s", "variants"}
    missing = required - set(data)
    if missing:
        raise ProbeError(f"Threshold file missing required keys: {sorted(missing)}")
    return data


def sts_suite_fallback(port: str | None) -> str:
    command = shutil.which("sts") or "sts"
    port_hint = (
        f" select port {port} at 1000000 baud,"
        if port
        else " select the active servo port at 1000000 baud,"
    )
    return (
        f"Optional manual fallback: sts-suite ({STS_SUITE_URL}). "
        f"Install with '{STS_SUITE_INSTALL}' or '{STS_SUITE_ALT_INSTALL}', run '{command}',"
        f"{port_hint} then use 'x' for the movement test or 'o' for the oscilloscope."
    )


def move_and_time(
    bus: FeetechMotorsBus,
    goal_position: int,
    *,
    settle_timeout_s: float,
    position_tolerance: int,
    consecutive_samples: int,
    sample_interval_s: float = DEFAULT_SAMPLE_INTERVAL_S,
    clock=time.perf_counter,
    sleep_fn=time.sleep,
) -> float:
    bus.write("Goal_Position", "motor", goal_position, normalize=False)
    start = clock()
    settled = 0
    while True:
        elapsed = clock() - start
        if elapsed > settle_timeout_s:
            raise ProbeError(
                f"Motor did not settle at goal {goal_position} within {settle_timeout_s:.2f}s."
            )
        present = int(bus.read("Present_Position", "motor", normalize=False))
        if abs(present - goal_position) <= position_tolerance:
            settled += 1
            if settled >= consecutive_samples:
                return elapsed
        else:
            settled = 0
        sleep_fn(sample_interval_s)


def expected_time_for_counts(variant_cfg: dict[str, Any], travel_counts: int) -> float:
    seconds_per_60_deg = float(variant_cfg["seconds_per_60_deg"])
    degrees = travel_counts * (360.0 / 4096.0)
    return seconds_per_60_deg * (degrees / 60.0)


def classify_variant(
    median_time_s: float, thresholds: dict[str, Any], *, travel_counts: int
) -> tuple[str | None, str, float]:
    ordered = ordered_variant_timings(thresholds, travel_counts=travel_counts)
    if len(ordered) != 3:
        raise ProbeError("Expected exactly three variants in the threshold file.")

    boundary_margin_s = float(thresholds["boundary_margin_s"])
    boundaries = []
    for (_, left), (_, right) in zip(ordered, ordered[1:]):
        boundaries.append((left + right) / 2.0)

    nearest_boundary_distance = min(abs(median_time_s - boundary) for boundary in boundaries)
    if nearest_boundary_distance <= boundary_margin_s:
        return None, "ambiguous", 0.0

    if median_time_s < boundaries[0]:
        code = ordered[0][0]
    elif median_time_s < boundaries[1]:
        code = ordered[1][0]
    else:
        code = ordered[2][0]

    confidence = min(0.99, max(0.5, nearest_boundary_distance / (boundary_margin_s * 2.0)))
    return code, "classified", round(confidence, 3)


def ordered_variant_timings(
    thresholds: dict[str, Any], *, travel_counts: int
) -> list[tuple[str, float]]:
    variants = thresholds["variants"]
    return sorted(
        (
            (code, expected_time_for_counts(cfg, travel_counts))
            for code, cfg in variants.items()
        ),
        key=lambda item: item[1],
    )


def determine_probe_window(bus: FeetechMotorsBus, thresholds: dict[str, Any]) -> ProbeWindow:
    return determine_probe_window_for_travel_counts(
        bus, requested_travel_counts=int(thresholds["travel_counts"])
    )


def determine_probe_window_for_travel_counts(
    bus: FeetechMotorsBus, *, requested_travel_counts: int
) -> ProbeWindow:
    present_position = int(bus.read("Present_Position", "motor", normalize=False))
    min_limit = safe_read(bus, "Min_Position_Limit")
    max_limit = safe_read(bus, "Max_Position_Limit")

    if min_limit is None:
        min_limit = 0
    if max_limit is None:
        max_limit = 4095
    if max_limit <= min_limit:
        raise ProbeError(f"Invalid motor limits: min={min_limit}, max={max_limit}.")

    requested_half_span = requested_travel_counts // 2
    low_room = present_position - (min_limit + LIMIT_MARGIN_COUNTS)
    high_room = (max_limit - LIMIT_MARGIN_COUNTS) - present_position
    half_span = min(requested_half_span, low_room, high_room)
    if half_span * 2 < MIN_TRAVEL_COUNTS:
        raise ProbeError(
            "Motor does not expose enough safe motion around its current position for classification. "
            f"present={present_position}, min={min_limit}, max={max_limit}."
        )

    low_goal = present_position - half_span
    high_goal = present_position + half_span
    return ProbeWindow(
        low_goal=low_goal,
        high_goal=high_goal,
        travel_counts=high_goal - low_goal,
        present_position=present_position,
        min_limit=min_limit,
        max_limit=max_limit,
    )


def attempt_variant_probe_cycles(
    bus: FeetechMotorsBus,
    probe_window: ProbeWindow,
    *,
    cycles: int,
    settle_timeout_s: float,
    position_tolerance: int,
    consecutive_samples: int,
) -> list[float]:
    trial_times: list[float] = []
    move_and_time(
        bus,
        probe_window.low_goal,
        settle_timeout_s=settle_timeout_s,
        position_tolerance=position_tolerance,
        consecutive_samples=consecutive_samples,
    )
    for _ in range(cycles):
        forward_time = move_and_time(
            bus,
            probe_window.high_goal,
            settle_timeout_s=settle_timeout_s,
            position_tolerance=position_tolerance,
            consecutive_samples=consecutive_samples,
        )
        backward_time = move_and_time(
            bus,
            probe_window.low_goal,
            settle_timeout_s=settle_timeout_s,
            position_tolerance=position_tolerance,
            consecutive_samples=consecutive_samples,
        )
        trial_times.append((forward_time + backward_time) / 2.0)
    move_and_time(
        bus,
        probe_window.present_position,
        settle_timeout_s=settle_timeout_s,
        position_tolerance=position_tolerance,
        consecutive_samples=consecutive_samples,
    )
    return trial_times


def run_variant_probe(
    bus: FeetechMotorsBus,
    thresholds: dict[str, Any],
    *,
    cycles: int,
    settle_timeout_s: float,
) -> dict[str, Any]:
    position_tolerance = int(thresholds.get("position_tolerance_counts", 12))
    consecutive_samples = int(thresholds.get("consecutive_settle_samples", 3))
    acceleration = int(thresholds.get("probe_acceleration", 50))
    requested_travel_counts = int(thresholds["travel_counts"])
    probe_window: ProbeWindow | None = None
    trial_times: list[float] | None = None
    last_error: ProbeError | None = None

    bus.write("Acceleration", "motor", acceleration, normalize=False)
    bus.write("Goal_Time", "motor", 0, normalize=False)
    bus.enable_torque()
    try:
        while requested_travel_counts >= MIN_TRAVEL_COUNTS:
            probe_window = determine_probe_window_for_travel_counts(
                bus, requested_travel_counts=requested_travel_counts
            )
            try:
                trial_times = attempt_variant_probe_cycles(
                    bus,
                    probe_window,
                    cycles=cycles,
                    settle_timeout_s=settle_timeout_s,
                    position_tolerance=position_tolerance,
                    consecutive_samples=consecutive_samples,
                )
                break
            except ProbeError as exc:
                last_error = exc
                requested_travel_counts //= 2
        if probe_window is None or trial_times is None:
            if last_error is not None:
                raise last_error
            raise ProbeError("Could not find a safe probe window for classification.")
    finally:
        bus.disable_torque()

    assert trial_times is not None
    assert probe_window is not None
    median_time_s = statistics.median(trial_times)
    classification, status, confidence = classify_variant(
        median_time_s, thresholds, travel_counts=probe_window.travel_counts
    )
    ordered = ordered_variant_timings(thresholds, travel_counts=probe_window.travel_counts)
    lowest_expected = ordered[0][1]
    highest_expected = ordered[-1][1]
    nominal_envelope_margin = float(thresholds["boundary_margin_s"])
    outside_nominal = (
        median_time_s < (lowest_expected - nominal_envelope_margin)
        or median_time_s > (highest_expected + nominal_envelope_margin)
    )
    if status == "ambiguous":
        next_step = (
            "Timing sits near a variant boundary. Use the sts-suite fallback below to cross-check "
            "the movement test and oscilloscope against the measured trial times."
        )
    else:
        next_step = "Classification completed."
        if outside_nominal:
            confidence = min(confidence, 0.6)
            next_step = (
                "Classification completed, but the measured timing sits outside the nominal timing bands. "
                "Use the sts-suite fallback below if you want a manual cross-check."
            )

    return {
        "trial_times": [round(value, 4) for value in trial_times],
        "median_time": round(median_time_s, 4),
        "classification": classification,
        "confidence": confidence,
        "status": status,
        "next_step": next_step,
        "manual_fallback": None,
        "probe_window": {
            "low_goal": probe_window.low_goal,
            "high_goal": probe_window.high_goal,
            "travel_counts": probe_window.travel_counts,
            "present_position": probe_window.present_position,
            "min_limit": probe_window.min_limit,
            "max_limit": probe_window.max_limit,
        },
    }


def to_payload(
    electronic: ElectronicProbe,
    motion: dict[str, Any] | None,
    *,
    requested_classify: bool,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "port": electronic.port,
        "baudrate": electronic.baudrate,
        "motor_id": electronic.motor_id,
        "model_number": electronic.model_number,
        "firmware_version": electronic.firmware_version,
        "family": electronic.family,
        "signature": electronic.signature,
        "trial_times": [],
        "median_time": None,
        "classification": None,
        "confidence": 0.0,
        "status": "electronic_only",
        "next_step": (
            "Electronic probe confirms only the STS3215 family. Run with --classify to distinguish "
            "C001, C044, and C046."
        ),
        "manual_fallback": sts_suite_fallback(electronic.port),
    }
    if electronic.model_number != EXPECTED_MODEL_NUMBER:
        payload["status"] = "failed"
        payload["next_step"] = (
            f"Expected STS3215 model number {EXPECTED_MODEL_NUMBER}, got {electronic.model_number}."
        )
        return payload

    if requested_classify and motion is not None:
        payload.update(motion)
        payload["manual_fallback"] = sts_suite_fallback(electronic.port)
    return payload


def print_human(payload: dict[str, Any]) -> None:
    print("SO-101 Leader Motor Variant Probe")
    print(f"Port: {payload['port']}")
    print(f"Baudrate: {payload['baudrate']}")
    print(f"Motor ID: {payload['motor_id']}")
    print(f"Model Number: {payload['model_number']}")
    print(f"Firmware: {payload['firmware_version'] or 'unknown'}")
    print(f"Family: {payload['family']}")
    if payload["trial_times"]:
        print(f"Trial Times (s): {payload['trial_times']}")
        print(f"Median Time (s): {payload['median_time']}")
        print(f"Classification: {payload['classification'] or 'ambiguous'}")
        print(f"Confidence: {payload['confidence']}")
    else:
        print("Classification: not run")
    print(f"Status: {payload['status']}")
    print(f"Next Step: {payload['next_step']}")
    if payload.get("manual_fallback"):
        print(f"Manual Fallback: {payload['manual_fallback']}")


def print_human_many(payloads: list[dict[str, Any]]) -> None:
    print(f"SO-101 Leader Motor Variant Probe ({len(payloads)} motors)")
    for index, payload in enumerate(payloads, start=1):
        print()
        print(f"[{index}/{len(payloads)}] Motor ID {payload['motor_id']}")
        print_human(payload)


def main() -> int:
    args = parse_args()
    if args.sts_suite_help:
        port = args.port
        if port is None:
            candidates = list_candidate_ports()
            if len(candidates) == 1:
                port = candidates[0]
        print(sts_suite_fallback(port))
        return 0
    requested_classify = args.classify or not args.electronic_only

    try:
        port = resolve_port(args.port)
        thresholds = load_thresholds(args.variant_thresholds)
        bus = open_bus(port, args.baudrate, args.timeout)
        try:
            ids_to_models = discover_motors(bus, args.retries)
            targets = select_targets(
                ids_to_models,
                requested_ids=parse_requested_ids(args.ids),
                all_motors=args.all_motors,
            )
            payloads = []
            for target in targets:
                electronic = run_electronic_probe(
                    bus,
                    port=port,
                    baudrate=args.baudrate,
                    target=target,
                )
                motion = None
                if requested_classify and electronic.model_number == EXPECTED_MODEL_NUMBER:
                    motion = run_variant_probe(
                        bus,
                        thresholds,
                        cycles=args.cycles,
                        settle_timeout_s=args.settle_timeout,
                    )
                payloads.append(to_payload(electronic, motion, requested_classify=requested_classify))
        finally:
            close_bus(bus)
    except Exception as exc:
        payload = {
            "port": args.port,
            "baudrate": args.baudrate,
            "motor_id": None,
            "model_number": None,
            "firmware_version": None,
            "family": "unknown",
            "signature": {},
            "trial_times": [],
            "median_time": None,
            "classification": None,
            "confidence": 0.0,
            "status": "failed",
            "next_step": str(exc),
            "manual_fallback": sts_suite_fallback(args.port),
        }
        if args.json:
            print(json.dumps(payload, indent=2))
        else:
            print_human(payload)
        return 1

    if args.json:
        if len(payloads) == 1:
            print(json.dumps(payloads[0], indent=2))
        else:
            print(json.dumps({"motors": payloads}, indent=2))
    else:
        if len(payloads) == 1:
            print_human(payloads[0])
        else:
            print_human_many(payloads)

    allowed_statuses = {"classified", "ambiguous", "electronic_only"}
    return 0 if all(payload["status"] in allowed_statuses for payload in payloads) else 1


if __name__ == "__main__":
    raise SystemExit(main())
