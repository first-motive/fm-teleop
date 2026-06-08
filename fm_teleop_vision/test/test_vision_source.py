"""VisionSource node smoke test — mocked capture + tracker, no camera and no model.

The node injects its capture and tracker through factories, so these tests drive the
full tick loop (enable -> capture neutral -> displace -> publish twist) without OpenCV,
MediaPipe, or a webcam. The debug image is disabled so the headless path never imports
cv2. This is the deterministic half of the test plan; the live camera run stays manual.
"""

import sys
import types

from geometry_msgs.msg import Vector3
import pytest
import rclpy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from fm_teleop_vision.pose import WristSample
from fm_teleop_vision.vision_source import VisionSource


def _zero_linear():
    return Vector3(x=0.0, y=0.0, z=0.0)


class FakeCapture:
    """Always returns a frame; the fake tracker ignores its contents."""

    def __init__(self):
        self.released = False

    def read(self):
        return True, object()

    def release(self):
        self.released = True


class FakeTracker:
    """Returns a wrist at a position the test mutates between ticks."""

    def __init__(self):
        self.pos = (0.0, 0.0, 0.0)
        self.visibility = 1.0
        self.detected = True
        self.closed = False

    def process(self, frame):
        x, y, z = self.pos
        return WristSample(self.detected, x, y, z, 0.0, 0.0, self.visibility, 64, 48)

    def close(self):
        self.closed = True


class RecordingPublisher:
    """Stands in for the arm_twist publisher to capture what the node sends."""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node(tracker):
    node = VisionSource(
        capture_factory=lambda source: FakeCapture(),
        tracker_factory=lambda: tracker,
    )
    # Headless: drop the debug image so no tick imports cv2, and record twists.
    node._debug_pub = None
    node._pub = RecordingPublisher()
    return node


def test_constructs_without_camera_or_model(ros):
    node = _make_node(FakeTracker())
    try:
        assert isinstance(node, VisionSource)
    finally:
        node.destroy_node()


def test_engage_captures_neutral_then_jogs(ros):
    tracker = FakeTracker()
    node = _make_node(tracker)
    try:
        # Rising edge: the next good frame becomes neutral, and that tick publishes zero.
        node._on_enable(Bool(data=True))
        node._on_tick()
        assert node._neutral is not None
        assert node._pub.messages[-1].twist.linear == _zero_linear()

        # Hold the wrist away from neutral; after the filter settles the arm jogs.
        tracker.pos = (0.25, 0.0, 0.0)
        for _ in range(60):
            node._on_tick()
        last = node._pub.messages[-1].twist.linear
        # MediaPipe +x maps to command -y, so a +x hold jogs -y (see _to_command_frame).
        assert last.y < 0.0
    finally:
        node.destroy_node()


def test_release_holds_with_zero_twist(ros):
    tracker = FakeTracker()
    node = _make_node(tracker)
    try:
        node._on_enable(Bool(data=True))
        node._on_tick()
        tracker.pos = (0.25, 0.0, 0.0)
        for _ in range(30):
            node._on_tick()
        # Falling edge: deadman publishes a single zero twist immediately.
        node._on_enable(Bool(data=False))
        assert node._pub.messages[-1].twist.linear == _zero_linear()
        assert node._neutral is None
    finally:
        node.destroy_node()


def test_lost_track_holds_while_engaged(ros):
    tracker = FakeTracker()
    node = _make_node(tracker)
    try:
        node._on_enable(Bool(data=True))
        node._on_tick()
        tracker.detected = False
        node._on_tick()
        assert node._pub.messages[-1].twist.linear == _zero_linear()
    finally:
        node.destroy_node()


class _FakeFrame:
    """A stand-in BGR frame: copy() returns itself; shape gives the label anchor."""

    shape = (48, 64, 3)

    def copy(self):
        return self


class _FakeBuf:
    def tobytes(self):
        return b"jpeg-bytes"


def test_debug_publishes_compressed_image(ros, monkeypatch):
    # Mock cv2 so the annotate/encode path runs without OpenCV installed.
    fake_cv2 = types.SimpleNamespace(
        FONT_HERSHEY_SIMPLEX=0,
        circle=lambda *a, **k: None,
        putText=lambda *a, **k: None,
        imencode=lambda ext, img: (True, _FakeBuf()),
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)

    tracker = FakeTracker()
    node = VisionSource(
        capture_factory=lambda source: FakeCapture(),
        tracker_factory=lambda: tracker,
    )
    try:
        recorder = RecordingPublisher()
        node._debug_pub = recorder  # enable debug with a recording sink
        sample = tracker.process(object())
        node._publish_debug(_FakeFrame(), sample)
        assert len(recorder.messages) == 1
        msg = recorder.messages[0]
        assert isinstance(msg, CompressedImage)
        assert msg.format == "jpeg"
        assert bytes(msg.data) == b"jpeg-bytes"
    finally:
        node.destroy_node()


def test_debug_disabled_is_noop(ros):
    node = _make_node(FakeTracker())  # _make_node sets _debug_pub = None
    try:
        # A disabled debug must return before importing cv2 — guards the headless path.
        sys.modules.pop("cv2", None)
        node._publish_debug(_FakeFrame(), FakeTracker().process(object()))
        assert "cv2" not in sys.modules
    finally:
        node.destroy_node()
