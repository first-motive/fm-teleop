# fm_teleop_vision

Teleop source skeleton: **vision hand-tracking**. A camera tracks the operator's hand;
wrist pose jogs the arm and finger curl drives the gripper — no worn hardware. Buildable
and importable today; the node body is one session of work away.

## Planned mapping

```
wrist pose delta  ->  arm_twist (TwistStamped)    ->  MoveIt Servo
finger curl       ->  hand_sliders / hand_preset  ->  hand teleop
```

After the browser panel, this is the most scalable physical source: a webcam is all the
operator needs.

## Status

Skeleton. `VisionSource.__init__` raises `NotImplementedError`; running
`ros2 run fm_teleop_vision vision_source` fails with a clear message. To implement:
subscribe to the tracker's landmark stream, derive a wrist twist and finger-curl vector
(deadzone + clamp via `fm_teleop_core.retarget`), and publish via
`self.contract_publisher(...)`.

## Build type

`ament_python`.
