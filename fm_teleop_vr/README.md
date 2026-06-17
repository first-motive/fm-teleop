# fm_teleop_vr

Teleop source skeleton: **VR controllers**. A headset-tracked 6-DOF controller jogs the
arm in Cartesian space and drives the base. Buildable and importable today; the node body
is one session of work away.

## Planned Mapping

```
controller pose delta  ->  arm_twist (TwistStamped)   ->  MoveIt Servo
thumbstick             ->  base_twist (Twist)          ->  /cmd_vel
grip / trigger         ->  hand_preset / hand_sliders  ->  hand teleop
```

VR maps cleanly onto the contract: a 6-DOF pose is exactly what Cartesian Servo wants,
and the spare inputs (stick, grip) cover the base and hands.

## Status

Skeleton. `VrSource.__init__` raises `NotImplementedError`; running
`ros2 run fm_teleop_vr vr_source` fails with a clear message. To implement: subscribe to
the VR bridge's pose/button stream, difference pose into a twist (deadzone + speed scalar
via `fm_teleop_core.retarget`), and publish via `self.contract_publisher(...)`.

## Build Type

`ament_python`.
