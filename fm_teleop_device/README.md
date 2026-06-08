# fm_teleop_device

Teleop sources for physical input devices. Each maps a device onto the shared command
contract (`fm_teleop_core`); the launch layer (`fm_bringup/teleop.launch.py`) starts the
matching driver node alongside.

## Sources

```
joy_to_servo       gamepad Joy        -> arm_twist (TwistStamped)  -> MoveIt Servo
spacenav_to_servo  SpaceMouse Twist   -> arm_twist (TwistStamped)  -> MoveIt Servo
g1_hand_teleop     hand_preset/sliders-> JointTrajectory           -> Dex3 hand controllers
```

`joy_to_servo` and `spacenav_to_servo` subclass `TeleopSource` and publish the `arm_twist`
channel. `g1_hand_teleop` consumes the contract's `hand_preset` (String) and `hand_sliders`
(Float64MultiArray) channels and emits the 7-joint trajectories the Dex3 controllers expect;
the preset/limit math lives in `hand_presets.py` (pure, unit-tested without a ROS graph).

## Run

```bash
ros2 run fm_teleop_device joy_to_servo
ros2 run fm_teleop_device spacenav_to_servo
ros2 run fm_teleop_device g1_hand_teleop
```

In practice these launch through `fm_bringup/teleop.launch.py` (`scripts/teleop.sh
--input joy|spacenav`); the G1-D hand node is registry-driven per robot.

## Device availability

The gamepad reads `/joy` (Linux `/dev/input`, or a Mac host-side HID->Joy bridge); the
SpaceMouse is USB, so Linux-only — no OrbStack/Docker passthrough on the Mac.

## Build type

`ament_python`.
