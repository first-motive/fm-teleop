# fm_teleop_policy

Policy serving as a teleop source: a trained checkpoint drives the same arm the
operator drives, through the same command contract.

## Role

```
/joint_states -> observation -> policy.act -> action chunk
                                           -> arm_joint (JointJog) -> Servo -> controller
```

This package succeeds fm-policy's `fm_policy_serve`, which is frozen at `v0.1.2`
and is not edited. Its behaviour is unchanged except for one thing: the action
vector's joint order comes from a robot profile, chosen by `robot_profile` and
`arm`, rather than from the `fm_bringup` registry. One node therefore serves the
loader's `follower_*` OpenArm and the canonical `openarm_*` one.

The node owns ROS plumbing only: observations, rates, the chunk buffer, the
engage gate, and the status rail. Models live behind `policies.load_policy`, so a
new model family is a new class in `policies.py` and no change to `server.py`.

| Backend | Checkpoint | Use |
|---------|-----------|-----|
| `zero` | none | All-zero actions; proves the whole loop in sim with no trained model |
| `lerobot` | a checkpoint directory | A LeRobot policy, with torch and lerobot imported lazily |

The `lerobot` backend fills `observation.state` only. A policy trained on camera
features rejects that batch rather than acting on half its input.

## Robot Profiles

A profile names one robot description's joints, controllers, and frames. fm-skills
and anvil-embodied-ai read the same schema, so its keys must stay identical across
the three repos.

```yaml
name: loader_openarm_v2
base_frame: follower_body_link0
arms:
  right:
    joints: [follower_r_joint1, ..., follower_r_joint7]   # controller order
    gripper_joint: follower_r_finger_joint1
    arm_controller: follower_r_forward_position_controller
    trajectory_controller: follower_r_jtc
    ee_frame: follower_r_hand_tcp
    hand_frame: follower_r_hand
  left: { same keys, follower_l_* }
cameras: { ... }   # optional
```

| Profile | Names | Source |
|---------|-------|--------|
| `loader_openarm_v2` (default) | `follower_*` | anvil-loader, on the real OpenArm today |
| `fm_openarm_v2` | `openarm_*` | fm-robot's description over openarm_description's `default_bimanual` preset |

`fm_openarm_v2`'s `ee_frame` (`openarm_*_grasp_frame`) exists only when the
description is built with `emit_grasp_frame:=true`. The node itself reads only
`joints`.

## The Engage Gate

The server starts **disengaged** and publishes nothing until `~/enable`
(`std_msgs/Bool`) goes true. Autonomy and teleop write the same contract channel,
so a controller keeps one writer and one motion path; engaging is the operator's
act. The deferred `fm_fsm` arbiter will own that gate without a change here.

Inference runs on its own thread, in its own callback group under a two-thread
executor, so a slow model never stalls the control tick. A chunk whose inference
straddled a disengage is discarded, and an action with a NaN or inf value drops
its chunk instead of reaching Servo. A chunk that is malformed in any other way
(wrong width, a non-number) is refused when inference returns it.

Servo enforces the joint-velocity, singularity, and collision limits downstream.
The server adds two bounds of its own: `chunk_timeout` stops publishing when
inference has gone quiet, and `observation_timeout` stops inference when the
robot's state stream goes quiet, and also stops a chunk that lands after the
stream went quiet during inference. Both are measured on the monotonic clock, so
`use_sim_time` before `/clock` arrives cannot hold them open.

## Run

The launch starts the server alone. MoveIt Servo and a robot, simulated or real,
must already be running.

```bash
ros2 launch fm_teleop_policy policy.launch.py robot_profile:=fm_openarm_v2 arm:=right
ros2 topic pub /policy_server/enable std_msgs/msg/Bool '{data: true}'
ros2 topic echo /policy_server/status
```

With a trained checkpoint:

```bash
ros2 launch fm_teleop_policy policy.launch.py \
  policy_type:=lerobot checkpoint:=/path/to/checkpoint
```

## Parameters

Every parameter is read-only after startup.

| Parameter | Default | Role |
|-----------|---------|------|
| `robot_profile` | `loader_openarm_v2` | Profile whose joint names the policy acts in (launch arg) |
| `arm` | `right` | Which arm of the profile the policy drives (launch arg) |
| `policy_type` | `zero` | Which backend to load (launch arg) |
| `checkpoint` | `""` | Checkpoint directory for `lerobot` (launch arg) |
| `observation_topic` | `/joint_states` | Observation source |
| `command_topic` | `/servo_node/delta_joint_cmds` | The contract's `arm_joint` channel |
| `enable_topic` | `~/enable` | The engage gate |
| `status_topic` | `~/status` | JSON status for a desktop autonomy rail; carries the checkpoint's basename and a short error summary, with the full error in the node log |
| `rate_hz` | `50.0` | Control cadence; drains the chunk |
| `inference_rate_hz` | `10.0` | Model cadence; fills the chunk |
| `status_rate_hz` | `1.0` | Status line cadence |
| `chunk_timeout` | `1.0` | Seconds an unrefilled chunk stays publishable |
| `observation_timeout` | `0.5` | Seconds without an observation before inference stops |

## Test

The profile and backend tests need no ROS:

```bash
cd fm_teleop_policy
PYTHONPATH=. uv run --no-project --with pytest --with pyyaml pytest -q test/test_profile.py test/test_policies.py
```

The node and import tests (`test/test_server.py`, `test/test_import.py`) need a
sourced ROS 2 workspace:

```bash
colcon build --packages-up-to fm_teleop_policy
colcon test --packages-select fm_teleop_policy && colcon test-result --verbose
```

## Build Type

`ament_python`. Builds on Humble and Jazzy; Jazzy is the target. CI builds and
tests it in both lanes.
