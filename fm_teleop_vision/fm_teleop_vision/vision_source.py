"""Vision hand-tracking teleop source (skeleton).

A camera tracks the operator's hand (e.g. MediaPipe / a depth model); wrist pose jogs
the arm and finger curl drives the gripper — no worn hardware, the most scalable physical
source after the panel.

Planned mapping:

    wrist pose delta  ->  arm_twist (TwistStamped)   ->  MoveIt Servo
    finger curl       ->  hand_sliders / hand_preset ->  hand teleop

Implement by subscribing to the tracker's landmark stream, deriving a wrist twist and a
finger-curl vector (deadzone + clamp via ``fm_teleop_core.retarget``), and publishing
through ``self.contract_publisher(...)``. See ../README.md for the add-a-source guide.
"""

import rclpy

from fm_teleop_core.source import TeleopSource


class VisionSource(TeleopSource):
    """Vision hand-tracking source — not yet implemented."""

    def __init__(self):
        raise NotImplementedError(
            "fm_teleop_vision is a skeleton. Implement tracked hand pose -> arm_twist + "
            "hand channels. See README.md."
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        VisionSource()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
