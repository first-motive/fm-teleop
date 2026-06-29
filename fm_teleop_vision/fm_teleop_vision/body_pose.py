# Vendored from vision-based-joint-tracking (pose_pipeline/pose.py), unchanged.
# Used by the future tracking_mode="full_body" (whole-body / humanoid) path; the v1
# arm/hand teleop uses hand.py instead. Keep in sync with the upstream product.
"""MediaPipe Pose Landmarker (Tasks API) wrapper.

Uses RunningMode.VIDEO (synchronous) with a strictly-increasing timestamp. We deliberately
avoid LIVE_STREAM: its GPU path crashes on Apple Silicon after a few minutes.

`process()` returns a `PoseFrame` carrying, per joint:
  - image-space pixels (px, py)  -> for the 2D overlay
  - world-space metres (wx, wy, wz, hip origin) -> for the 3D skeleton + metrics
  - visibility / presence
"""

from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass

import cv2
import mediapipe as mp

log = logging.getLogger(__name__)

BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
RunningMode = mp.tasks.vision.RunningMode

NUM_LANDMARKS = 33

# Index -> human-readable name (MediaPipe Pose 33-landmark topology).
LANDMARK_NAMES = [
    "nose", "left_eye_inner", "left_eye", "left_eye_outer", "right_eye_inner",
    "right_eye", "right_eye_outer", "left_ear", "right_ear", "mouth_left",
    "mouth_right", "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_pinky", "right_pinky", "left_index",
    "right_index", "left_thumb", "right_thumb", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle", "left_heel",
    "right_heel", "left_foot_index", "right_foot_index",
]

# Bone connections (index pairs) — the canonical MediaPipe Pose 33-landmark topology.
# The legacy `mediapipe.solutions.pose` module is NOT bundled in the Tasks-API wheels
# (mediapipe >= 0.10.x), so we define the connection set directly instead of importing it.
POSE_CONNECTIONS = [
    # face
    (0, 1), (1, 2), (2, 3), (3, 7), (0, 4), (4, 5), (5, 6), (6, 8), (9, 10),
    # left arm + hand
    (11, 13), (13, 15), (15, 17), (15, 19), (15, 21), (17, 19),
    # right arm + hand
    (12, 14), (14, 16), (16, 18), (16, 20), (16, 22), (18, 20),
    # shoulders + torso
    (11, 12), (11, 23), (12, 24), (23, 24),
    # left leg + foot
    (23, 25), (25, 27), (27, 29), (27, 31), (29, 31),
    # right leg + foot
    (24, 26), (26, 28), (28, 30), (28, 32), (30, 32),
]


def _attr(landmark, name: str) -> float:
    val = getattr(landmark, name, None)
    return float(val) if val is not None else 0.0


@dataclass
class Joint:
    idx: int
    px: float  # image pixel x
    py: float  # image pixel y
    wx: float  # world metres x (hip origin)
    wy: float
    wz: float
    visibility: float
    presence: float


@dataclass
class PoseFrame:
    detected: bool
    joints: list  # list[Joint]; empty if not detected
    image_w: int
    image_h: int
    timestamp_ms: int
    inference_ms: float


class PoseEstimator:
    def __init__(
        self,
        model_path: str,
        *,
        num_poses: int = 1,
        min_pose_detection_confidence: float = 0.5,
        min_pose_presence_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
        output_segmentation_masks: bool = False,
        timestamp_strategy: str = "counter",
    ):
        if not os.path.exists(model_path):
            raise FileNotFoundError(
                f"Pose model not found at {model_path!r}. "
                "Run `bash scripts/download_model.sh` to fetch pose_landmarker_heavy.task."
            )
        options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=RunningMode.VIDEO,
            num_poses=num_poses,
            min_pose_detection_confidence=min_pose_detection_confidence,
            min_pose_presence_confidence=min_pose_presence_confidence,
            min_tracking_confidence=min_tracking_confidence,
            output_segmentation_masks=output_segmentation_masks,
        )
        self._landmarker = PoseLandmarker.create_from_options(options)
        self._strategy = timestamp_strategy
        self._counter = 0
        self._t0 = None
        self._last_ts_ms = -1
        log.info("PoseEstimator ready (VIDEO mode, model=%s)", model_path)

    def _next_timestamp_ms(self, wall_ts: float) -> int:
        if self._strategy == "wallclock":
            if self._t0 is None:
                self._t0 = wall_ts
            ts = int((wall_ts - self._t0) * 1000.0)
            ts = max(ts, self._last_ts_ms + 1)  # enforce strict monotonicity
        else:  # counter (recommended): immune to wall-clock stalls / collisions
            self._counter += 1
            ts = self._counter * 33
        self._last_ts_ms = ts
        return ts

    def process(self, frame_bgr, wall_ts: float) -> PoseFrame:
        h, w = frame_bgr.shape[:2]
        ts_ms = self._next_timestamp_ms(wall_ts)
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        t0 = time.perf_counter()
        result = self._landmarker.detect_for_video(mp_image, ts_ms)
        inference_ms = (time.perf_counter() - t0) * 1000.0

        if not result.pose_landmarks or not result.pose_world_landmarks:
            return PoseFrame(False, [], w, h, ts_ms, inference_ms)

        image_lms = result.pose_landmarks[0]
        world_lms = result.pose_world_landmarks[0]
        joints = []
        for i in range(min(len(image_lms), len(world_lms))):
            il = image_lms[i]
            wl = world_lms[i]
            joints.append(
                Joint(
                    idx=i,
                    px=il.x * w,
                    py=il.y * h,
                    wx=wl.x,
                    wy=wl.y,
                    wz=wl.z,
                    visibility=_attr(il, "visibility"),
                    presence=_attr(il, "presence"),
                )
            )
        return PoseFrame(True, joints, w, h, ts_ms, inference_ms)

    def close(self) -> None:
        self._landmarker.close()


def draw_overlay(
    frame_bgr,
    pose_frame: PoseFrame,
    *,
    point_color=(0, 255, 0),
    line_color=(255, 255, 0),
    vis_threshold: float = 0.5,
):
    """Return a BGR copy with the 2D landmarks + bones drawn on it."""
    out = frame_bgr.copy()
    if not pose_frame.detected:
        cv2.putText(out, "no pose", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return out

    pts = {j.idx: (int(round(j.px)), int(round(j.py))) for j in pose_frame.joints}
    for a, b in POSE_CONNECTIONS:
        if a in pts and b in pts:
            cv2.line(out, pts[a], pts[b], line_color, 2)
    for j in pose_frame.joints:
        if j.visibility >= vis_threshold:
            cv2.circle(out, pts[j.idx], 4, point_color, -1)
        else:
            cv2.circle(out, pts[j.idx], 3, (0, 0, 255), 1)  # low-visibility joints in red
    return out
