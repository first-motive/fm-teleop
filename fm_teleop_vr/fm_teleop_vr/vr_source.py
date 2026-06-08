"""VR-controller teleop source (skeleton).

A 6-DOF VR controller (headset-tracked) jogs the arm in Cartesian space and drives the
mobile base, mapping naturally onto the contract's Servo + base-velocity channels.

Planned mapping:

    controller pose delta  ->  arm_twist (TwistStamped)  ->  MoveIt Servo
    thumbstick             ->  base_twist (Twist)         ->  /cmd_vel
    grip / trigger         ->  hand_preset / hand_sliders ->  hand teleop

Implement by subscribing to the VR bridge's pose/button stream, differencing pose to a
twist (deadzone + speed scalar via ``fm_teleop_core.retarget``), and publishing through
``self.contract_publisher(...)``. See ../README.md for the add-a-source guide.
"""

import rclpy

from fm_teleop_core.source import TeleopSource


class VrSource(TeleopSource):
    """VR-controller source — not yet implemented."""

    def __init__(self):
        raise NotImplementedError(
            "fm_teleop_vr is a skeleton. Implement VR controller pose -> arm_twist + "
            "base_twist (and grip -> hand channels). See README.md."
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        VrSource()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
