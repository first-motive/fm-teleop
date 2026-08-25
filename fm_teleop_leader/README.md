# fm_teleop_leader

Teleop source: **leader-arm follow**. A physical leader arm whose joint states drive the
follower directly.

## Mapping

```
leader /joint_states  ->  arm_trajectory (JointTrajectory)  ->  follower arm controller
```

This is the contract's **leader-bypass** path: the leader's joints already form a valid
pose stream, so the source republishes them straight to the follower's arm controller,
skipping MoveIt Servo (which exists to turn Cartesian/joint *deltas* into safe motion).

## Nodes

| Node | Role |
|------|------|
| `leader_source` | leader `JointState` -> single-point `JointTrajectory` on the follower's controller |
| `leader_driver` | SO-101 leader arm's Feetech bus -> leader `JointState` (hardware only) |

```
leader_driver ---> /leader/joint_states ---> leader_source ---> <arm>_controller/joint_trajectory
   (hardware)         (or any publisher, in sim)                    (follower)
```

`leader_source` needs the follower's controller identity — `joints` in controller order
and `command_topic`. `teleop.launch.py` reads both from the `fm_bringup` robot registry,
so an operator selecting Leader-Follower types neither.

### Safety

Servo is bypassed, so this source owns the bounds nothing downstream provides:

- **Deadman** on `~/enable` (`std_msgs/Bool`). The rising edge seeds the ramp from the
  follower's *current* joint state, so engaging with the arms far apart ramps rather
  than jumps. Releasing stops commanding; the controller holds its last point.
- **`max_joint_step`** caps radians of travel per published point.
- **`sample_timeout`** stops commanding when the leader stream goes quiet, and re-seeds
  on its return.
- The zenoh bridge denies `*_controller/joint_trajectory` between machines, so the
  source runs on the rig with its leader — never remotely.

### Running It

Sim first — any publisher on `/leader/joint_states` stands in for the arm:

```bash
ros2 launch fm_bringup teleop.launch.py robot:=so101 input:=leader
ros2 topic pub /leader_source/enable std_msgs/msg/Bool '{data: true}'
```

On hardware, `leader_driver` supplies the stream once the motors are provisioned with
the scripts below:

```bash
ros2 run fm_teleop_leader leader_driver --ros-args -p port:=/dev/ttyACM0
```

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
