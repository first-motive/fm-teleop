#!/usr/bin/env python3
"""Interactively identify and configure SO-101 leader-arm STS3215 motors.

This script is intentionally standalone in repo terms: one file, interactive,
and callable directly from the terminal. For reliable bus communication it uses
the vendored LeRobot + Feetech stack already installed by ``setup-lerobot.sh``.

Key limitation:
- all STS3215 variants report the same model number, so exact joint identity
  cannot be derived electronically.
- this script therefore depends on the operator sorting motors by tactile
  resistance / gear-ratio bucket before assigning IDs.

Typical usage:

1. Safe probe of what is currently on the bus:
   ``~/.venvs/lerobot/bin/python scripts/configure-so101-leader-motors.py --probe``

2. Configure the full SO-101 leader-arm set from joint 6 down to 1:
   ``~/.venvs/lerobot/bin/python scripts/configure-so101-leader-motors.py``

3. Configure exactly one known joint:
   ``~/.venvs/lerobot/bin/python scripts/configure-so101-leader-motors.py --joint 2``
"""

from __future__ import annotations

import argparse
import sys
import textwrap
import time
from dataclasses import dataclass

try:
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorNormMode
except ImportError as exc:  # pragma: no cover - runtime dependency check
    raise SystemExit(
        "LeRobot + Feetech dependencies are required. Activate ~/.venvs/lerobot first."
    ) from exc

DEFAULT_PORT = "/dev/tty.usbmodem5B3D0471021"
DEFAULT_BAUDRATE = 1_000_000
DEFAULT_TIMEOUT_S = 1.0
DEFAULT_RETRIES = 3


@dataclass(frozen=True)
class JointTarget:
    step_index: int
    joint_number: int
    joint_name: str
    target_id: int
    gear_ratio: str
    pile_label: str


TARGETS = [
    JointTarget(
        step_index=1,
        joint_number=6,
        joint_name="Gripper",
        target_id=6,
        gear_ratio="1:147",
        pile_label="Loose / Fluid",
    ),
    JointTarget(
        step_index=2,
        joint_number=5,
        joint_name="Wrist Roll",
        target_id=5,
        gear_ratio="1:147",
        pile_label="Loose / Fluid",
    ),
    JointTarget(
        step_index=3,
        joint_number=4,
        joint_name="Wrist Flex",
        target_id=4,
        gear_ratio="1:147",
        pile_label="Loose / Fluid",
    ),
    JointTarget(
        step_index=4,
        joint_number=3,
        joint_name="Elbow Flex",
        target_id=3,
        gear_ratio="1:191",
        pile_label="Medium Stiff",
    ),
    JointTarget(
        step_index=5,
        joint_number=2,
        joint_name="Shoulder Lift",
        target_id=2,
        gear_ratio="1:345",
        pile_label="Extremely Stiff",
    ),
    JointTarget(
        step_index=6,
        joint_number=1,
        joint_name="Base / Pan",
        target_id=1,
        gear_ratio="1:191",
        pile_label="Medium Stiff",
    ),
]

TARGET_BY_JOINT = {target.joint_number: target for target in TARGETS}


class ProtocolError(RuntimeError):
    """Raised for communication or workflow mismatches."""


def clear_screen(enabled: bool) -> None:
    if enabled:
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()


def banner(title: str) -> str:
    line = "=" * len(title)
    return f"{line}\n{title}\n{line}"


class FeetechSerialBus:
    def __init__(
        self,
        *,
        port: str,
        baudrate: int,
        timeout_s: float,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self._bus = FeetechMotorsBus(
            port=port,
            motors={"motor": Motor(1, "sts3215", MotorNormMode.DEGREES)},
        )
        self._bus._connect(handshake=False)
        self._bus.set_baudrate(baudrate)

    def close(self) -> None:
        try:
            self._bus.port_handler.closePort()
        except Exception:
            pass

    def ping(self, motor_id: int, *, retries: int = DEFAULT_RETRIES) -> bool:
        model = self._bus.ping(
            motor_id,
            num_retry=max(0, retries - 1),
            raise_on_error=False,
        )
        return model is not None

    def configure_id(self, *, current_id: int, target_id: int) -> None:
        setup_bus = FeetechMotorsBus(
            port=self.port,
            motors={"motor": Motor(target_id, "sts3215", MotorNormMode.DEGREES)},
        )
        setup_bus._connect(handshake=False)
        try:
            setup_bus.set_baudrate(self.baudrate)
            setup_bus.setup_motor(
                "motor",
                initial_baudrate=self.baudrate,
                initial_id=current_id,
            )
        finally:
            setup_bus.port_handler.closePort()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactively identify and configure SO-101 leader-arm STS3215 motors."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port of the Feetech bus.")
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help="Bus baudrate. Default: 1000000.",
    )
    parser.add_argument(
        "--joint",
        type=int,
        choices=sorted(TARGET_BY_JOINT),
        help="Configure only one specific joint instead of the full 6->1 sequence.",
    )
    parser.add_argument(
        "--probe",
        action="store_true",
        help="Read-only check: ping IDs 1..6 and report which respond at the current baudrate.",
    )
    parser.add_argument(
        "--no-clear",
        action="store_true",
        help="Do not clear the terminal between interactive steps.",
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
        help="Retries per ping/write operation. Default: 3.",
    )
    return parser.parse_args()


def targets_for_run(args: argparse.Namespace) -> list[JointTarget]:
    if args.joint is not None:
        return [TARGET_BY_JOINT[args.joint]]
    return TARGETS


def print_intro(args: argparse.Namespace, selected_targets: list[JointTarget]) -> None:
    print(banner("SO-101 Leader Motor Configurator"))
    print(
        textwrap.dedent(
            f"""
            Port: {args.port}
            Baudrate: {args.baudrate}
            Important:
            - all STS3215 variants look identical electronically on the bus.
            - exact joint identity must come from your tactile gear-ratio sorting.
            - if you only have one loose 1:147 motor in hand, this script cannot tell whether
              it is joint 4, 5, or 6 without your assembly workflow context.

            This run will cover:
            """
        ).strip()
    )
    for target in selected_targets:
        print(
            f"- Joint {target.joint_number}: {target.joint_name} | "
            f"ID {target.target_id} | Gear {target.gear_ratio} | Pile {target.pile_label}"
        )
    print()


def probe_ids(bus: FeetechSerialBus, retries: int) -> int:
    print(banner("Bus Probe"))
    responding = []
    for motor_id in range(1, 7):
        responds = bus.ping(motor_id, retries=retries)
        status = "RESPONDS" if responds else "no response"
        print(f"ID {motor_id}: {status}")
        if responds:
            responding.append(motor_id)

    print()
    if not responding:
        print("No IDs 1..6 responded at this baudrate. Check power, wiring, and baudrate.")
        return 1

    if responding == [1]:
        print(
            "Exactly one motor is responding on factory/default ID 1. "
            "That means the motor is reachable, but not which physical joint it is."
        )
    else:
        print(
            "At least one configured motor is already on the bus. "
            "Make sure only one motor is connected during configuration."
        )

    return 0


def wait_for_enter(message: str) -> None:
    input(f"{message}\n")


def discover_connected_id(
    bus: FeetechSerialBus,
    target: JointTarget,
    retries: int,
) -> int | None:
    if bus.ping(1, retries=retries):
        return 1

    if target.target_id != 1:
        if bus.ping(target.target_id, retries=retries):
            return target.target_id

    return None


def configure_target(
    bus: FeetechSerialBus,
    target: JointTarget,
    retries: int,
) -> None:
    current_id = discover_connected_id(bus, target, retries)
    if current_id is None:
        raise RuntimeError(
            "Motor not found! Check power/cables and ensure exactly one motor is connected."
        )

    if current_id == target.target_id:
        print(
            f"Motor already responds as ID {target.target_id} "
            f"for Joint {target.joint_number} ({target.joint_name})."
        )
        return

    bus.configure_id(current_id=current_id, target_id=target.target_id)
    time.sleep(0.5)

    if not bus.ping(target.target_id, retries=retries):
        raise RuntimeError(
            f"Failed to verify new ID {target.target_id}. The motor did not answer after re-addressing."
        )


def run_interactive(args: argparse.Namespace, selected_targets: list[JointTarget]) -> int:
    print_intro(args, selected_targets)
    wait_for_enter("Press Enter to begin.")

    try:
        bus = FeetechSerialBus(
            port=args.port,
            baudrate=args.baudrate,
            timeout_s=args.timeout,
        )
    except Exception as exc:
        print(f"Failed to open serial port {args.port}: {exc}", file=sys.stderr)
        return 1

    try:
        for index, target in enumerate(selected_targets, start=1):
            clear_screen(not args.no_clear)
            print(banner(f"STEP {index}/{len(selected_targets)}"))
            print(
                textwrap.dedent(
                    f"""
                    Joint {target.joint_number}: {target.joint_name}
                    Target ID: {target.target_id}
                    Gear Ratio: {target.gear_ratio}
                    Physical Pile: {target.pile_label}

                    Please plug in exactly ONE motor from your [{target.pile_label}] pile
                    into the driver board. Ensure NO OTHER MOTORS are connected.
                    """
                ).strip()
            )
            wait_for_enter("Press Enter when ready...")

            configure_target(bus, target, args.retries)
            print()
            print(
                f"SUCCESS: Motor configured as ID {target.target_id} "
                f"(Joint {target.joint_number}: {target.joint_name})!"
            )
            wait_for_enter("Unplug it, label it, then press Enter to continue...")
    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)
        return 130
    except (ProtocolError, RuntimeError) as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        bus.close()

    clear_screen(not args.no_clear)
    print(banner("All Done"))
    print("All requested joints were processed.")
    return 0


def main() -> int:
    args = parse_args()
    selected_targets = targets_for_run(args)

    try:
        bus = FeetechSerialBus(
            port=args.port,
            baudrate=args.baudrate,
            timeout_s=args.timeout,
        )
    except Exception as exc:
        print(f"Failed to open serial port {args.port}: {exc}", file=sys.stderr)
        return 1

    if args.probe:
        try:
            return probe_ids(bus, args.retries)
        except ProtocolError as exc:
            print(f"Probe failed: {exc}", file=sys.stderr)
            return 1
        finally:
            bus.close()

    bus.close()
    return run_interactive(args, selected_targets)


if __name__ == "__main__":
    raise SystemExit(main())
