"""WristTracker validation — the cheap error paths, no MediaPipe and no model.

The tracker validates its arguments before importing MediaPipe, so these run on a base
image without the heavy dep. Live landmark extraction needs the real model and a camera,
so it stays in the manual test plan.
"""

import pytest

from fm_teleop_vision.pose import WRIST_INDEX, WristTracker


def test_wrist_index_covers_both_sides():
    assert WRIST_INDEX["left"] == 15
    assert WRIST_INDEX["right"] == 16


def test_rejects_unknown_side():
    with pytest.raises(ValueError):
        WristTracker("models/pose_landmarker_heavy.task", side="middle")


def test_missing_model_raises_filenotfound(tmp_path):
    missing = tmp_path / "absent.task"
    with pytest.raises(FileNotFoundError):
        WristTracker(str(missing), side="right")
