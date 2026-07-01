# fm_teleop_vision

Teleop source: **vision wrist-tracking**. A camera tracks the operator's wrist with
MediaPipe Pose; holding the wrist away from a neutral pose jogs the arm through MoveIt
Servo. After the browser panel, this is the most scalable physical source — a webcam is
all the operator needs, no worn hardware.

This phase closes the smallest loop: one wrist, linear motion only, the `arm_twist`
channel. Gripper control (finger curl), wrist orientation, and full-body tracking are
deferred.

## Mapping

```
camera frame
   │  cv2.VideoCapture (webcam index or MJPEG/RTSP URL)
   ▼
WristTracker (MediaPipe Pose, world position in metres)
   ▼
Vec3 One-Euro filter (kills rest-jitter without adding reach lag)
   ▼
displacement from the engage-time neutral  →  displacement_to_twist (scale · deadzone · clamp)
   ▼
TwistStamped on arm_twist  →  /servo_node/delta_twist_cmds  →  MoveIt Servo
```

Only the linear part of the twist is driven; angular stays zero (MediaPipe wrist
orientation is unreliable). The retarget math is `fm_teleop_core.retarget.displacement_to_twist`.

## Engage UX (deadman)

```
panel "Vision (hold)" button → std_msgs/Bool on /vision_teleop/enable
   press (rising edge) → capture the current wrist as neutral, start jogging
   hold                → arm jogs proportional to displacement from neutral
   release             → publish one zero twist, hold the arm
```

The neutral pose is captured on every fresh engage, so the operator can re-centre by
releasing and pressing again. Losing the wrist mid-engage (occlusion, low visibility)
holds the arm and resets the filter, so a re-acquire does not jump.

## Run

```bash
# 1. One-time: fetch the MediaPipe model (~30 MB, gitignored under models/).
bash scripts/download_model.sh

# 2. Install the pip-only runtime deps into the container image.
uv pip install mediapipe opencv-python

# 3. Start the sim + Servo in one terminal (see fm_bringup), then the vision input:
ros2 launch fm_bringup teleop.launch.py input:=vision camera_source:=0
```

`camera_source` is a webcam index (`0`, the host webcam) or a stream URL.

### Camera: USB vs stream

OrbStack on macOS cannot pass a USB webcam into the container, so the webcam index
(`camera_source:=0`) works **only when the node runs on the host**. The portable path is
an MJPEG/RTSP stream from a phone IP-webcam app on the same network:

```
camera_source:=http://<phone-ip>:8080/video
```

Watch the annotated `/vision_teleop/debug/compressed` stream in Foxglove to confirm the
camera has the wrist (green marker) and whether the deadman is held (`ENGAGED` / `held`).

## Z-axis caveat

MediaPipe's depth (Z) estimate is markedly noisier than X/Y — it is the suspect axis. The
One-Euro filter tames it, but if the forward/back jog feels unstable, drop it without
touching the other axes by running the node directly with the param set:

```bash
ros2 run fm_teleop_vision vision_source --ros-args -p use_z:=false
```

`use_x`, `use_y`, `use_z` all default true.

## Parameters

| Parameter            | Default                              | Meaning                                            |
| -------------------- | ------------------------------------ | -------------------------------------------------- |
| `camera_source`      | `0`                                  | Webcam index or stream URL.                        |
| `model_path`         | `models/pose_landmarker_heavy.task`  | MediaPipe Pose model (from `download_model.sh`).   |
| `wrist_side`         | `right`                              | Which wrist to track (`left` / `right`).           |
| `command_frame`      | `openarm_right_base_link`            | Frame the twist is stamped in (Servo command frame).|
| `scale`              | `4.0`                                | Displacement (m) → command velocity.               |
| `deadzone`           | `0.03`                               | Per-axis displacement (m) below which the jog is zero. |
| `clamp`              | `1.0`                                | Per-axis twist bound (Servo's unitless input).     |
| `use_x` / `use_y` / `use_z` | `true`                        | Per-axis enable; drop `use_z` if depth is unstable.|
| `min_visibility`     | `0.5`                                | Below this, the wrist is treated as lost.          |
| `rate_hz`            | `30.0`                               | Capture + command rate.                            |
| `filter_min_cutoff` / `filter_beta` / `filter_d_cutoff` | `1.0` / `0.02` / `1.0` | One-Euro filter tuning.            |
| `twist_topic`        | `/servo_node/delta_twist_cmds`       | `arm_twist` output topic.                          |
| `enable_topic`       | `/vision_teleop/enable`              | Deadman input.                                     |
| `debug_image_topic`  | `/vision_teleop/debug/compressed`    | Annotated JPEG; empty disables it.                 |

## Frame mapping

MediaPipe world axes are image-style (+x right, +y down, +z toward the camera); the
command frame is REP-103 (+x forward, +y left, +z up). The mapping lives in one small,
commented function (`_to_command_frame`). With the operator facing the camera:

```
forward (+x) ← +z   (push the hand toward the camera → arm forward)
left    (+y) ← −x   (hand to the operator's right → arm right)
up      (+z) ← −y   (hand up → arm up)
```

These three signs are the first thing to flip if the arm jogs the wrong way during
bringup — expect to tune them once against the live sim.

## Build type

`ament_python`. MediaPipe and OpenCV are pip-only (no rosdep key), so the node needs
`mediapipe` + `opencv-python` in the runtime image; the tests do not (the retarget and
filter tests are pure Python, and the node smoke test injects fakes).
