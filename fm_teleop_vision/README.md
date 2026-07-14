# fm_teleop_vision

Camera-based teleop for First Motive's arm: an operator's hand drives the robot, no worn
hardware — just a webcam or a phone streaming over the network. Part of the
[`fm-teleop`](../README.md) layer, run as part of the assembled
[`fm-ros2`](https://github.com/first-motive/fm-ros2) workspace.

Two independent inputs live here; pick one with `input:=` on the launch:

| `input:=` | Node(s) | Control model | Channel |
|-----------|---------|---------------|---------|
| **`mirror`** *(current focus)* | `hand_tracker` + `mirror_source` | **1:1 absolute hand-pose mirroring** ("mouse pickup"): the EE holds an absolute target that tracks your hand metre-for-metre | `arm_pose_target` → `/target_pose` |
| `vision` | `vision_source` | Wrist **velocity jog**: holding the wrist off a neutral pose jogs the arm | `arm_twist` → Servo twist |

This README is written to get a new person **running the `mirror` path end-to-end**; the
`vision` path is documented lower down.

---

## Quickstart — `mirror` on macOS / OrbStack

All commands run from the **fm-ros2 workspace root** (`fm-ros2/`), where the Docker overlays
and `scripts/` live. The container service is `fm` (`docker-fm-1`), workspace mounted at `/ws`.

### 0. One-time setup (order matters)
```bash
C="docker compose -f docker/compose.yaml -f docker/compose.macos.yaml"
$C run --rm fm bash scripts/import-externals.sh                                   # externals + OpenArm gravity patch
$C run --rm fm bash src/fm-teleop/fm_teleop_vision/scripts/download_model.sh      # MediaPipe models -> models/
$C run --rm fm colcon build --symlink-install
$C exec fm pip install mediapipe==0.10.14                                         # until baked into the image
```

### 1. Camera (see [Camera on macOS](#camera-on-macos) — the container can't reach your LAN)
```bash
# On the HOST: relay the phone stream through localhost (container reaches it via host.docker.internal).
socat TCP-LISTEN:8090,reuseaddr,fork TCP:<phone-ip>:8081     # leave running
```

### 2. Sim — terminal 1
```bash
./scripts/sim.sh --robot openarm --backend mujoco
```

### 3. Teleop — terminal 2, then enable the debug channel
```bash
./scripts/teleop.sh --robot openarm --input mirror \
  camera_source:=http://host.docker.internal:8090/video rotate_deg:=90 publish_debug_image:=true

# once nodes are up, turn on the internal-state debug publisher (see Debugging):
docker exec docker-fm-1 bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; \
  export ROS_DOMAIN_ID=0; ros2 param set /mirror_source publish_debug true'
```
The bare launch (any platform) is `ros2 launch fm_bringup teleop.launch.py robot:=openarm
input:=mirror camera_source:=... rotate_deg:=90 publish_debug_image:=true`; `sim.sh`/`teleop.sh`
are the fm-ros2 macOS wrappers around it.

### 4. Engage — terminal 3 (get into a comfortable pose first)
```bash
docker compose -f docker/compose.yaml -f docker/compose.macos.yaml exec -T fm \
  /ros_entrypoint.sh ros2 topic pub -r 15 /vision/engage std_msgs/msg/Bool "{data: true}"
```
Raise your hand in front of the camera → the arm mirrors it. `pose_tracking_node` logging
`"target pose is achieved"` = following; `"target pose was not updated. Aborting."` = idle
(no hand / disengaged).

### 5. Foxglove
Connect Foxglove Studio to `ws://localhost:8765` and import the ready-made dashboard
[`foxglove/mirror_teleop.json`](foxglove/mirror_teleop.json) (3D arm + `/vision/image` +
record buttons + debug plots).

---

## How it works

### `mirror` pipeline
```
camera frame
   │  cv2.VideoCapture (webcam index or MJPEG/RTSP URL)
   ▼
hand_tracker            MediaPipe Hands (21 landmarks) → a metric-ish hand pose
   │  /vision/hand_pose (PoseStamped, normalized image-width units, camera frame)
   │  /vision/grip (Float64 curl 0..1)   /vision/tracking_active (Bool)   /vision/image (debug)
   │  /vision/<left|right>/skeleton (fm_teleop_msgs/HandSkeleton — full 21-landmark 3D+2D
   │       hand + per-finger joint angles) and /quality (HandQuality) — the recorded
   │       "second data stream"; set num_hands:=2 (capture_hands:=both) to publish both hands
   ▼
mirror_source           on ENGAGE latch: hand_ref, EE pose ee_ref (from tf2), metric scale W_m
   │  every tick:  target = clamp_box( ee_ref + remap( mirror_gain · W_m · (hand_now − hand_ref) ) )
   ▼
   /target_pose (PoseStamped, absolute EE target in command/base frame)
   ▼
pose_tracking_node      MoveIt Servo *PoseTracking* (lives in fm_control / fm-robot) → joint commands → arm
```
Key ideas: it holds an **absolute** target (a steady hand → a steady arm, no integrated drift
to the singularity); orientation is held at the engage pose (`enable_angular=false`, landmark
orientation is unreliable); the target is **clamped to a workspace box** (clamped motion is the
`overflow` you can log). The pure retarget math is [`mapping.py`](fm_teleop_vision/mapping.py)
(ROS-free, unit-tested).

Engage UX (deadman): `/vision/engage` true → latch reference + mirror; disengage / tracking
lost / stale hand → stop publishing, PoseTracking times out and holds. A brief tracking flicker
is debounced (`tracking_grace`) so it doesn't re-latch and jump.

### `vision` pipeline (wrist velocity jog)
```
WristTracker (MediaPipe Pose) → One-Euro filter → displacement from engage-neutral
   → displacement_to_twist (scale · deadzone · clamp) → TwistStamped → MoveIt Servo
```
Linear only (angular stays zero). Engage on `/vision_teleop/enable`; re-centre by releasing
and pressing again. Run with `input:=vision camera_source:=<index|url>`.

---

## Camera on macOS

OrbStack on macOS **cannot pass a USB webcam into the container**, and the container sits on a
NAT bridge with **no route to your LAN** — so a phone-webcam URL that works from the Mac fails
from the container. Always relay the stream through the host:

```bash
# 0. confirm the phone stream is up FROM THE MAC (200 = good; the endless MJPEG stream means
#    curl's --max-time cutoff / exit 28 is still success):
curl -s -o /dev/null -w "%{http_code}\n" --max-time 5 http://<phone-ip>:8081/video
# 1. relay host:8090 -> phone:8081 (raw TCP: lossless, preserves the /video path so rotate stays 90):
socat TCP-LISTEN:8090,reuseaddr,fork TCP:<phone-ip>:8081
# 2. point the launch at the relay:
#    camera_source:=http://host.docker.internal:8090/video rotate_deg:=90
```
`<phone-ip>` is from an IP-webcam app (e.g. IP Camera Lite, iOS — portrait ⇒ `rotate_deg:=90`).
Set the phone to Auto-Lock=Never and keep the app foregrounded; its DHCP IP changes (re-check
and update the socat target). On Linux with a real USB webcam, skip the relay and pass
`camera_source:=0`.

**Gotcha:** if `hand_tracker` starts while the stream has a hiccup, its capture stalls (no
`tracking_active`). Fix: restart terminal 2. Confirm it's healthy with
`docker exec docker-fm-1 ps -eo pcpu,args | grep hand_tracker` → ~70% CPU means it's decoding
frames.

---

## Debugging & data capture

Tooling under [`scripts/`](scripts/) captures the whole `hand → target → actual-EE` chain so
you can see exactly how a hand motion became a robot motion.

**Instrumented internals.** Set `publish_debug:=true` on `mirror_source` (step 3 above) and it
publishes `/vision/mirror_debug` (`Float64MultiArray`, layout `mirror_debug_v1`) with the values
the two poses can't show: the latched `hand_ref`/`ee_ref`, `W_m`, per-axis scale, the pre-clamp
target, and the workspace `overflow`.

**Record a session** (rosbag of every pipeline topic **+** a synchronized CSV/JSONL):
```bash
# Persistent Foxglove-button mode — leave running; ● REC / ■ STOP publish std_msgs/Bool on
# /capture/record; status echoes on /capture/status:
docker exec docker-fm-1 bash /ws/src/fm-teleop/fm_teleop_vision/scripts/capture_mirror.sh

# ...or a one-shot CLI capture (Ctrl-C to finalize):
docker exec docker-fm-1 bash /ws/src/fm-teleop/fm_teleop_vision/scripts/capture_mirror.sh \
  --auto-start --session-name mytest
```
Each session lands in `data/teleop_captures/<timestamp>_session/`: `mirror_log.csv` (one
time-aligned row per tick — hand, commanded target, actual EE from tf, per-axis error, joints,
and the `dbg_*` internals), `mirror_log.jsonl`, a `bag/`, and `meta.json`.

**The second data stream.** The same session also captures `hands.jsonl` — both hands' full
skeleton per tick (21 landmarks in 3D metres + 2D pixels, per-finger joint angles, palm
orientation, grip) — the high-value human-hand ground truth that sits on top of the raw video.
`meta.json` carries `hand_qa`: per-hand tracking %, mean confidence, in-frame/occlusion %,
jitter p50/p95, `both_hands_pct`, and a composite quality score + A–D grade, so a dataset
consumer can filter to only clean episodes. For a clean raw head-camera feed (no burned-in
overlay), launch with `camera_input:=topic` — the `fm_sensors` camera node then publishes
`/head_cam/image_raw` and the tracker consumes exactly the frames that get recorded.

**Analyze** (per-axis ranges, the empirically-measured hand→target axis mapping, servo tracking
error, workspace clamping; writes plots):
```bash
docker exec docker-fm-1 bash -lc 'source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; \
  python3 /ws/src/fm-teleop/fm_teleop_vision/scripts/analyze_mirror.py /ws/data/teleop_captures/<SESSION>'
```

---

## Live tuning (`mirror_source` params, no relaunch)

```bash
R(){ docker exec docker-fm-1 bash -lc "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; export ROS_DOMAIN_ID=0; $1"; }
R 'ros2 param set /mirror_source mirror_gain 1.2'                      # amplitude (1.0 = 1:1)
R "ros2 param set /mirror_source axis_map_linear \"['z','-x','-y']\""  # flip a wrong-direction axis
R 'ros2 param set /mirror_source axis_gain "[1.0,1.0,1.0]"'           # enable the depth axis (off by default)
```
Persist winners in `fm_bringup/config/<robot>/vision.yaml` (fm-app).

### `mirror_source` parameters

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `mirror_gain` | `1.0` | Unitless amplitude; `1.0` = 1:1 physical mirroring. |
| `axis_gain` | `[1.0, 1.0, 0.0]` | Per **input** axis (image-x, image-y, depth). Depth defaults **off** — the mono depth proxy is weak; opt in once x/y feel right. |
| `axis_map_linear` | `[z, x, -y]` | Remap operator/image axes → command-frame axes (tune signs by eye). |
| `hand_span_m` | `0.09` | Real wrist→middle-knuckle span; sets the metres-per-image-unit scale `W_m`. |
| `fallback_scale` | `0.9` | m/unit used when the hand size at engage is degenerate. |
| `workspace_min` / `workspace_max` | `[-0.1,-0.6,-0.55]` / `[0.55,0.3,0.25]` | Command-frame clamp box (keeps the arm out of the straight-arm singularity). Must contain the engage EE pose. |
| `command_frame` / `ee_frame` | `openarm_right_base_link` / `openarm_right_link7` | Servo planning frame and the tf link read for the EE reference. |
| `publish_rate` | `50.0` | Target stream rate (Hz) into PoseTracking. |
| `command_timeout` | `0.2` | Hand pose older than this ⇒ stop commanding. |
| `tracking_grace` | `0.3` | Debounce window for a `tracking_active` flicker (avoids a re-latch jump). |
| `enable_angular` | `false` | Orientation mirroring (reserved; landmark orientation is unreliable). |
| `enable_grip` | `true` | Publish gripper open/close presets from finger curl. |
| `grip_open_below` / `grip_close_above` | `0.35` / `0.65` | Curl hysteresis thresholds. |
| `publish_debug` | `false` | Publish `/vision/mirror_debug` internal state (see Debugging). |
| `hand_pose_topic` / `grip_topic` / `tracking_topic` / `engage_topic` | `vision/*` | Input topics from `hand_tracker`. |
| `target_pose_topic` | `/target_pose` | `arm_pose_target` output. |

### `vision_source` parameters
Wrist-jog params (`scale`, `deadzone`, `clamp`, `use_x/y/z`, One-Euro `filter_*`, `wrist_side`,
`min_visibility`, `twist_topic`, `enable_topic`, `debug_image_topic`, `camera_source`,
`model_path`) — unchanged from the wrist-jog design; drop `use_z:=false` if forward/back feels
unstable (MediaPipe depth is the noisy axis).

---

## Frame / axis mapping

MediaPipe axes are image-style (+x right, +y down, +z toward camera); the command frame is
REP-103 (+x forward, +y left, +z up). With the operator facing the camera, the default
`axis_map_linear=[z, x, -y]` maps:

```
command +x (forward) ← input z (depth)       # ← but axis_gain depth = 0 by default, so forward is held
command +y (left)    ← input x (image-x)      # hand left/right
command +z (up)      ← input −y (image-y)      # hand up/down
```
These signs are the first thing to flip if the arm moves the wrong way — expect to tune once
against the live sim, then persist in `vision.yaml`.

---

## Testing

```bash
colcon test --packages-select fm_teleop_vision
colcon test-result --verbose
```
The math (`mapping.py`) and filters are pure Python and tested without a camera or a ROS graph;
the node smoke tests inject fakes.

## Build & deps

`ament_python`. Runtime needs `mediapipe` + `opencv-python` (pip-only, no rosdep key — installed
into the image / container). Console scripts: `hand_tracker`, `mirror_source`, `vision_source`
(see `setup.py`). MediaPipe models (`hand_landmarker.task`, `pose_landmarker_heavy.task`) are
fetched by `scripts/download_model.sh` into `models/` (gitignored, ~30 MB).
