# fm_teleop_vr

Teleop source: **VR controllers**. A headset-tracked 6-DOF controller jogs the arm in
Cartesian space, drives the base, and opens or closes the hand.

## Mapping

```
controller pose - engage pose  ->  arm_twist (TwistStamped)   ->  MoveIt Servo
thumbstick                     ->  base_twist (Twist)          ->  /cmd_vel
analogue grip                  ->  hand_preset (String)        ->  hand teleop
```

VR maps cleanly onto the contract: a 6-DOF pose is exactly what Cartesian Servo wants,
and the spare inputs (stick, grip) cover the base and hands.

## The Bridge

`vr_source` consumes a VR *bridge* — whatever republishes a headset runtime's controller
state as `geometry_msgs/PoseStamped` + `sensor_msgs/Joy`:

```
headset runtime ---> VR bridge ---> /vr/controller/right/{pose,joy} ---> vr_source
```

No OpenXR or vendor SDK is imported here, so the node runs on any machine and is proven
in sim by publishing those two topics by hand.

## Engagement

A deadman button on the `Joy` stream gates everything, matching the vision source. Its
rising edge latches the controller's current pose as neutral, so the jog starts at zero
wherever the operator is holding the controller; releasing publishes one zero arm twist
and one zero base twist so Servo and the base stop rather than coast.

The base is gated by the same deadman: a source that streamed `/cmd_vel` whenever it was
running would fight every other writer on that topic.

## Deferred

Orientation. The arm jogs linearly and angular stays zero. A quaternion delta to an
angular magnitude is the natural next step, but it needs Servo's angular scaling tuned
per robot, so the MVP keeps the axis count already proven by the vision source.

## Running It

```bash
ros2 launch fm_bringup teleop.launch.py robot:=openarm input:=vr
```

## Build Type

`ament_python`.
