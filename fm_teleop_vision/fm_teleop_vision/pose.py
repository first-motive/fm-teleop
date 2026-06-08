"""MediaPipe Pose Landmarker (Tasks API) wrapper, scoped to a single wrist.

The teleop loop needs one thing from the camera: where the operator's wrist is in space.
This wraps the full-body Pose Landmarker and returns just the chosen wrist's world
position (metres, hip origin) plus its image pixel and visibility, so the node maps a
wrist displacement to an arm twist and the debug overlay can mark the tracked point.

Design notes carried from the perception proof:
  - Use the Tasks API (``mediapipe.tasks.python.vision.PoseLandmarker``), not the
    deprecated ``mediapipe.solutions.pose``.
  - ``RunningMode.VIDEO`` with a strictly-increasing counter timestamp — the LIVE_STREAM
    GPU path crashes on Apple Silicon after a few minutes.
  - ``pose_world_landmarks`` is metres with the hip midpoint as origin; that is what we
    feed to the displacement retarget.

OpenCV and MediaPipe are imported lazily inside ``WristTracker`` (not at module top), so
``WristSample`` and ``WRIST_INDEX`` import without the heavy deps. The node smoke test
reuses ``WristSample`` through a fake tracker and never loads OpenCV or MediaPipe.
"""

from dataclasses import dataclass
import os

# MediaPipe Pose 33-landmark indices for the two wrists.
WRIST_INDEX = {"left": 15, "right": 16}


@dataclass
class WristSample:
    """One frame's wrist estimate. ``detected`` is False when no pose was found."""

    detected: bool
    wx: float  # world metres, hip origin
    wy: float
    wz: float
    px: float  # image pixel x (for the debug overlay)
    py: float
    visibility: float
    image_w: int
    image_h: int


_NO_DETECT = WristSample(False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0)


def _attr(landmark, name):
    """Read a float landmark attribute, defaulting to 0.0 when absent.

    MediaPipe omits ``visibility`` / ``presence`` on some model variants; treating a
    missing field as 0.0 lets the caller apply its visibility threshold uniformly
    rather than special-casing the attribute's existence.
    """
    value = getattr(landmark, name, None)
    return float(value) if value is not None else 0.0


class WristTracker:
    """Detect a pose per frame and return the selected wrist's world position."""

    def __init__(
        self,
        model_path,
        side="right",
        *,
        min_pose_detection_confidence=0.5,
        min_pose_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    ):
        # Validate before importing MediaPipe so the error paths are unit-testable
        # without the heavy dep installed.
        if side not in WRIST_INDEX:
            raise ValueError(f"side must be 'left' or 'right', got {side!r}.")
        if not os.path.exists(model_path):
            raise FileNotFoundError(
                f"Pose model not found at {model_path!r}. Run "
                "scripts/download_model.sh to fetch pose_landmarker_heavy.task."
            )
        self._index = WRIST_INDEX[side]

        import mediapipe as mp

        options = mp.tasks.vision.PoseLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=model_path),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=min_pose_detection_confidence,
            min_pose_presence_confidence=min_pose_presence_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self._landmarker = mp.tasks.vision.PoseLandmarker.create_from_options(options)
        self._counter = 0

    def process(self, frame_bgr):
        """Run detection on a BGR frame; return the wrist sample (or a no-detect)."""
        import cv2
        import mediapipe as mp

        h, w = frame_bgr.shape[:2]
        # Strictly-increasing counter timestamp (ms): immune to wall-clock stalls.
        self._counter += 1
        ts_ms = self._counter * 33
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self._landmarker.detect_for_video(mp_image, ts_ms)

        if not result.pose_landmarks or not result.pose_world_landmarks:
            return WristSample(False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, w, h)

        image_lm = result.pose_landmarks[0][self._index]
        world_lm = result.pose_world_landmarks[0][self._index]
        return WristSample(
            detected=True,
            wx=world_lm.x,
            wy=world_lm.y,
            wz=world_lm.z,
            px=image_lm.x * w,
            py=image_lm.y * h,
            visibility=_attr(image_lm, "visibility"),
            image_w=w,
            image_h=h,
        )

    def close(self):
        self._landmarker.close()
