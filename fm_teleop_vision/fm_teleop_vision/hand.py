"""MediaPipe Hand Landmarker (Tasks API) wrapper.

New for fm_teleop_vision (the upstream product tracks the whole body; here we track one
hand to drive a robot arm). Modeled on the vendored pose.py: RunningMode.VIDEO with a
strictly-increasing timestamp (the LIVE_STREAM GPU path crashes on Apple Silicon).

`process()` returns a `HandFrame` carrying, per landmark:
  - image-space pixels (px, py)            -> for the optional 2D debug overlay
  - world-space metres (wx, wy, wz, wrist origin) -> for the wrist pose + orientation + curl

Geometry (palm orientation, finger curl) is intentionally NOT computed here — it lives in
the ROS-free ``fm_teleop_vision.mapping`` module so it is unit-tested without mediapipe.
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
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
RunningMode = mp.tasks.vision.RunningMode

NUM_LANDMARKS = 21

# Index -> name (MediaPipe Hands 21-landmark topology).
LANDMARK_NAMES = [
    "wrist",
    "thumb_cmc", "thumb_mcp", "thumb_ip", "thumb_tip",
    "index_mcp", "index_pip", "index_dip", "index_tip",
    "middle_mcp", "middle_pip", "middle_dip", "middle_tip",
    "ring_mcp", "ring_pip", "ring_dip", "ring_tip",
    "pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip",
]

# Named indices used by the mapping (palm frame + finger curl).
WRIST = 0
THUMB_TIP = 4
INDEX_MCP = 5
INDEX_TIP = 8
MIDDLE_MCP = 9
MIDDLE_TIP = 12
RING_MCP = 13
RING_TIP = 16
PINKY_MCP = 17
PINKY_TIP = 20
FINGERTIPS = (INDEX_TIP, MIDDLE_TIP, RING_TIP, PINKY_TIP)
MCPS = (INDEX_MCP, MIDDLE_MCP, RING_MCP, PINKY_MCP)

# Bone connections for the debug overlay (palm + 5 fingers).
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),          # thumb
    (0, 5), (5, 6), (6, 7), (7, 8),          # index
    (5, 9), (9, 10), (10, 11), (11, 12),     # middle
    (9, 13), (13, 14), (14, 15), (15, 16),   # ring
    (13, 17), (17, 18), (18, 19), (19, 20),  # pinky
    (0, 17),                                 # palm base
]


@dataclass
class HandLandmark:
    idx: int
    px: float  # image pixel x
    py: float  # image pixel y
    wx: float  # world metres x (wrist origin)
    wy: float
    wz: float


@dataclass
class HandFrame:
    detected: bool
    landmarks: list  # list[HandLandmark]; empty if not detected
    handedness: str  # "Left" | "Right" | "" as reported by the model (see mirror note)
    image_w: int
    image_h: int
    timestamp_ms: int
    inference_ms: float
    score: float = 0.0  # model handedness/presence confidence in [0, 1]


class HandEstimator:
    """Wrap MediaPipe HandLandmarker; return one selected hand per frame.

    ``preferred_handedness`` ("Left"/"Right"/None): with ``num_hands`` >= 2, prefer the
    hand whose model-reported handedness matches; otherwise return the first hand. NOTE:
    MediaPipe reports handedness as if the image were mirrored (selfie view); for a raw
    (un-mirrored) webcam the labels are swapped. The tracker node accounts for this.
    """

    def __init__(
        self,
        model_path: str,
        *,
        num_hands: int = 1,
        min_hand_detection_confidence: float = 0.5,
        min_hand_presence_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
        preferred_handedness=None,
        timestamp_strategy: str = "counter",
    ):
        if not os.path.exists(model_path):
            raise FileNotFoundError(
                f"Hand model not found at {model_path!r}. "
                "Run `bash scripts/download_model.sh` to fetch hand_landmarker.task."
            )
        options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=RunningMode.VIDEO,
            num_hands=num_hands,
            min_hand_detection_confidence=min_hand_detection_confidence,
            min_hand_presence_confidence=min_hand_presence_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self._landmarker = HandLandmarker.create_from_options(options)
        self._preferred = preferred_handedness
        self._strategy = timestamp_strategy
        self._counter = 0
        self._t0 = None
        self._last_ts_ms = -1
        log.info("HandEstimator ready (VIDEO mode, model=%s)", model_path)

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

    def _select(self, result):
        """Return the index of the hand to use, or None."""
        if not result.hand_landmarks:
            return None
        if self._preferred and result.handedness:
            for i, cats in enumerate(result.handedness):
                if cats and cats[0].category_name == self._preferred:
                    return i
        return 0

    def process(self, frame_bgr, wall_ts: float) -> HandFrame:
        h, w = frame_bgr.shape[:2]
        ts_ms = self._next_timestamp_ms(wall_ts)
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        t0 = time.perf_counter()
        result = self._landmarker.detect_for_video(mp_image, ts_ms)
        inference_ms = (time.perf_counter() - t0) * 1000.0

        idx = self._select(result)
        if idx is None or not result.hand_world_landmarks:
            return HandFrame(False, [], "", w, h, ts_ms, inference_ms)

        image_lms = result.hand_landmarks[idx]
        world_lms = result.hand_world_landmarks[idx]
        handed = ""
        if result.handedness and idx < len(result.handedness) and result.handedness[idx]:
            handed = result.handedness[idx][0].category_name

        landmarks = []
        for i in range(min(len(image_lms), len(world_lms))):
            il = image_lms[i]
            wl = world_lms[i]
            landmarks.append(
                HandLandmark(idx=i, px=il.x * w, py=il.y * h, wx=wl.x, wy=wl.y, wz=wl.z)
            )
        return HandFrame(True, landmarks, handed, w, h, ts_ms, inference_ms)

    def process_hands(self, frame_bgr, wall_ts: float):
        """Return a HandFrame for EVERY detected hand (up to ``num_hands``).

        Like ``process`` but does not collapse to a single selected hand — used to feed
        the bimanual capture stream. Runs inference ONCE. Handedness labels are the raw
        model labels (selfie-mirror convention); the caller anatomically corrects them.
        Returns ``[]`` when nothing is detected.
        """
        h, w = frame_bgr.shape[:2]
        ts_ms = self._next_timestamp_ms(wall_ts)
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        t0 = time.perf_counter()
        result = self._landmarker.detect_for_video(mp_image, ts_ms)
        inference_ms = (time.perf_counter() - t0) * 1000.0

        frames = []
        if not result.hand_world_landmarks:
            return frames
        n = min(len(result.hand_landmarks), len(result.hand_world_landmarks))
        for idx in range(n):
            image_lms = result.hand_landmarks[idx]
            world_lms = result.hand_world_landmarks[idx]
            handed, score = "", 0.0
            if result.handedness and idx < len(result.handedness) and result.handedness[idx]:
                cat = result.handedness[idx][0]
                handed, score = cat.category_name, float(cat.score)
            landmarks = []
            for i in range(min(len(image_lms), len(world_lms))):
                il = image_lms[i]
                wl = world_lms[i]
                landmarks.append(
                    HandLandmark(idx=i, px=il.x * w, py=il.y * h, wx=wl.x, wy=wl.y, wz=wl.z)
                )
            frames.append(HandFrame(True, landmarks, handed, w, h, ts_ms, inference_ms, score))
        return frames

    def close(self) -> None:
        self._landmarker.close()


def _draw_hand(out, hand_frame: HandFrame, point_color, line_color):
    """Draw one hand's 21 landmarks + bones onto ``out`` in place."""
    pts = {lm.idx: (int(round(lm.px)), int(round(lm.py))) for lm in hand_frame.landmarks}
    for a, b in HAND_CONNECTIONS:
        if a in pts and b in pts:
            cv2.line(out, pts[a], pts[b], line_color, 2)
    for lm in hand_frame.landmarks:
        cv2.circle(out, pts[lm.idx], 4, point_color, -1)


def draw_overlay(frame_bgr, hand_frame: HandFrame, *, point_color=(0, 255, 0), line_color=(255, 255, 0)):
    """Return a BGR copy with the 2D hand landmarks + bones drawn on it (single hand)."""
    out = frame_bgr.copy()
    if not hand_frame.detected:
        cv2.putText(out, "no hand", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return out
    _draw_hand(out, hand_frame, point_color, line_color)
    return out


# Per-handedness colours so left/right read differently in the bimanual overlay. MediaPipe's
# raw label is selfie-mirrored, so which physical hand gets which colour may swap — cosmetic.
_HAND_COLORS = {
    "Left": ((0, 255, 0), (255, 255, 0)),     # green points, cyan bones
    "Right": ((0, 200, 255), (0, 128, 255)),  # amber points/bones
}
_HAND_COLORS_DEFAULT = ((0, 255, 0), (255, 255, 0))


def draw_hands_overlay(frame_bgr, hand_frames):
    """Return a BGR copy with EVERY detected hand drawn (the bimanual/both-hands overlay).

    ``hand_frames`` is the full list of tracked hands; it may be empty or all-undetected on a
    no-detection frame, which renders the same "no hand" label as the single-hand path."""
    out = frame_bgr.copy()
    drawn = 0
    for hf in hand_frames:
        if not hf.detected:
            continue
        point_color, line_color = _HAND_COLORS.get(hf.handedness, _HAND_COLORS_DEFAULT)
        _draw_hand(out, hf, point_color, line_color)
        drawn += 1
    if drawn == 0:
        cv2.putText(out, "no hand", (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    return out
