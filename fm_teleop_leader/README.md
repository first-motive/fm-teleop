# fm_teleop_leader

Teleop source skeleton: **leader-arm follow**. A physical leader arm whose joint states
drive the follower directly. Buildable and importable today; the node body is one session
of work away.

## Planned Mapping

```
leader /joint_states  ->  arm_trajectory (JointTrajectory)  ->  follower arm controller
```

This is the contract's **leader-bypass** path: the leader's joints already form a valid
pose stream, so the source republishes them straight to the follower's arm controller,
skipping MoveIt Servo (which exists to turn Cartesian/joint *deltas* into safe motion).

## Status

Skeleton. `LeaderSource.__init__` raises `NotImplementedError`; running
`ros2 run fm_teleop_leader leader_source` fails with a clear message. To implement:
subscribe to the leader's `sensor_msgs/JointState`, map each sample to a single-point
`JointTrajectory`, and publish via `self.contract_publisher("arm_trajectory",
topic=<follower controller>)`.

## Build Type

`ament_python`.
