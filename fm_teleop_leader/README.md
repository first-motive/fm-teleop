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

## Utility Scripts

This repo also carries standalone SO101 leader-arm bringup helpers under
`fm_teleop_leader/scripts/`:

- `classify-so101-leader-motor-variant.py` probes one STS3215 motor and estimates
  whether it is a `C001`, `C044`, or `C046` variant from timed motion.
- `compare-so101-leader-motor-motion.py` runs a repeatable left-center-right-center
  motion so an operator can compare motors by eye when classification is ambiguous.
- `configure-so101-leader-motors.py` walks an operator through assigning IDs to a
  sorted pile of leader-arm motors.

The timing thresholds used by the classifier live in
`fm_teleop_leader/config/sts3215_variant_thresholds.json`, and the package test suite
includes a focused regression test for the threshold bucketing logic.

## Build Type

`ament_python`.
