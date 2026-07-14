"""hand_tracker — perception node: a camera + MediaPipe Hands -> a hand pose stream.

This node owns the heavy, blocking work (camera read + model inference) on a BACKGROUND
THREAD (modeled on the product's Pose Studio PipelineRunner), so it never stalls the ROS
executor. The thread writes the latest reduced hand state under a lock; a ROS timer reads
it and publishes — publishers are only touched on the executor thread.

It does NOT know the robot or the teleop contract. It publishes a generic stream that
``vision_source`` turns into arm commands:

    vision/hand_pose       geometry_msgs/PoseStamped   wrist position + palm orientation
    vision/grip            std_msgs/Float64            finger curl 0 (open) .. 1 (closed)
    vision/tracking_active std_msgs/Bool               True while a hand is tracked
    vision/image           sensor_msgs/CompressedImage optional 2D debug overlay

It ALSO publishes the full hand skeleton as a recorded annotation stream (the dataset's
"second data stream"), per hand, decoupled from control — set num_hands:=2 to capture both
hands even though only one drives the robot:

    vision/<left|right>/skeleton  fm_teleop_msgs/HandSkeleton  21 landmarks (3D+2D) + angles
    vision/<left|right>/quality   fm_teleop_msgs/HandQuality   per-frame tracking quality

Position is in normalized-image-WIDTH units: x = wrist/width in [0,1], y = wrist/width in
[0, h/w] (ALL axes share one unit so a single metres-per-unit factor applies — see
mapping.control_position), and z = an apparent-size depth proxy (hand bigger = closer).
Single-camera depth is coarse — that is exactly the limitation the upstream perception
proof measured — so the depth axis is gain-gated separately (see vision_source).
Orientation + curl come from the metric ``hand_world_landmarks`` and are reliable.

Camera: two input modes. input_mode="device" (default) opens ``camera_source`` directly —
a device index like "0" on Linux (/dev passthrough), or an http/rtsp URL on Mac/OrbStack
(a phone IP-webcam app). input_mode="topic" instead subscribes to a ROS image topic (the
fm_sensors head camera), so the tracked frames are exactly the frames the recorder saves
and the same tracker runs on head + wrist cameras.
"""

import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, Quaternion
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float64

from fm_teleop_msgs.msg import HandQuality, HandSkeleton
from fm_teleop_vision import body_pose as pose_mod
from fm_teleop_vision import hand as hand_mod
from fm_teleop_vision import mapping
from fm_teleop_vision.capture import CameraSource
from fm_teleop_vision.filters import OneEuroFilter, SkeletonFilter

_MODEL_FILES = {"hand": "hand_landmarker.task", "full_body": "pose_landmarker_heavy.task"}


def _image_msg_to_bgr(msg):
    """Convert a sensor_msgs/Image to a BGR numpy frame (topic input mode)."""
    import numpy as np
    h, w = msg.height, msg.width
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    enc = (msg.encoding or "bgr8").lower()
    if enc in ("bgr8", "rgb8"):
        arr = arr.reshape(h, w, 3)
        return arr[:, :, ::-1].copy() if enc == "rgb8" else arr.copy()
    if enc in ("bgra8", "rgba8"):
        arr = arr.reshape(h, w, 4)[:, :, :3]
        return arr[:, :, ::-1].copy() if enc == "rgba8" else arr.copy()
    if enc == "mono8":
        return np.repeat(arr.reshape(h, w, 1), 3, axis=2).copy()
    return arr.reshape(h, w, -1)[:, :, :3].copy()   # fallback: assume bgr-like


class _TopicFrameSource:
    """A frames() source fed by a ROS image subscription — the topic-input twin of
    capture.CameraSource. Yields (frame_bgr, wall_ts), always the latest pushed frame, so
    the worker loop is identical whether frames come from a device or a camera topic."""

    def __init__(self, timeout=1.0):
        self._cond = threading.Condition()
        self._latest = None
        self._seq = 0
        self._stopped = False
        self._timeout = timeout

    def push(self, frame_bgr, wall_ts):
        with self._cond:
            self._latest = (frame_bgr, wall_ts)
            self._seq += 1
            self._cond.notify_all()

    def frames(self):
        last = 0
        while not self._stopped:
            with self._cond:
                self._cond.wait_for(lambda: self._seq != last or self._stopped,
                                    timeout=self._timeout)
                if self._stopped:
                    return
                if self._seq == last:
                    continue
                frame, ts = self._latest
                last = self._seq
            yield frame, ts

    def release(self):
        with self._cond:
            self._stopped = True
            self._cond.notify_all()


class _Sample:
    __slots__ = ("detected", "pos", "quat", "curl", "wall_ts", "jpeg")

    def __init__(self):
        self.detected = False
        self.pos = (0.0, 0.0, 0.0)
        self.quat = mapping.IDENTITY_QUAT
        self.curl = 0.0
        self.wall_ts = 0.0
        self.jpeg = None


class _HandSample:
    """Full per-hand annotation for one frame — the recorded "second data stream".

    Distinct from ``_Sample`` (which is the reduced CONTROL state for one selected
    hand): this keeps everything MediaPipe produced so the tracker can publish a
    HandSkeleton + HandQuality per hand without disturbing the control path.
    """

    __slots__ = ("detected", "hand", "world_lms", "image_lms", "joint_angles",
                 "quat", "curl", "pos", "confidence", "in_frame", "jitter", "wall_ts")

    def __init__(self):
        self.detected = False
        self.hand = ""                     # anatomically-corrected "left" | "right"
        self.world_lms = None              # list[(x, y, z)] length 21 (metres, filtered)
        self.image_lms = None              # list[(px, py)] length 21 (pixels, raw)
        self.joint_angles = None           # list[float] length N_JOINT_ANGLES
        self.quat = mapping.IDENTITY_QUAT
        self.curl = 0.0
        self.pos = (0.0, 0.0, 0.0)         # control position (norm image-width units), filtered
        self.confidence = 0.0
        self.in_frame = False
        self.jitter = 0.0
        self.wall_ts = 0.0


class HandTracker(Node):
    def __init__(self):
        super().__init__("hand_tracker")

        # --- parameters ---
        self.declare_parameter("camera_source", "0")  # int index (Linux) or URL (Mac)
        self.declare_parameter("backend", "auto")      # auto | v4l2 | ffmpeg | avfoundation
        # Frame source: "device" opens camera_source directly (today); "topic" subscribes to
        # a ROS image topic (e.g. the fm_sensors head-cam), so the tracked frames are exactly
        # the frames recorded, and the same tracker will run on head + wrist cameras.
        self.declare_parameter("input_mode", "device")            # device | topic
        self.declare_parameter("input_image_topic", "head_cam/image_raw")
        # hand: MediaPipe Hand Landmarker (21 landmarks, one hand; grip from finger curl).
        # full_body: MediaPipe Pose Landmarker (33 body joints); the operator's body_side
        # wrist drives control and the elbow->wrist FOREARM is the apparent-size depth
        # proxy (set vision_source hand_span_m ~0.26 — teleop.launch.py does this for
        # you). No grip signal (no finger landmarks). Useful to A/B tracking quality.
        self.declare_parameter("tracking_mode", "hand")  # hand | full_body
        self.declare_parameter("body_side", "right")     # full_body: which arm to track
        # full_body: wrist+elbow must be at least this visible to count as tracked —
        # below it the model extrapolates joints far off-body (red dots in the overlay).
        self.declare_parameter("min_joint_visibility", 0.5)
        self.declare_parameter("model_path", "")       # empty -> resolve from package share
        self.declare_parameter("num_hands", 1)
        self.declare_parameter("hand_label", "")       # "Left"|"Right"|"" (model label; mirror-dependent)
        self.declare_parameter("min_detection_confidence", 0.5)
        self.declare_parameter("min_presence_confidence", 0.5)
        self.declare_parameter("min_tracking_confidence", 0.5)
        self.declare_parameter("filter_enabled", True)
        self.declare_parameter("filter_min_cutoff", 1.0)
        self.declare_parameter("filter_beta", 0.02)
        self.declare_parameter("filter_d_cutoff", 1.0)
        self.declare_parameter("publish_rate", 30.0)   # Hz of the publish timer
        self.declare_parameter("stale_timeout", 0.5)   # s; older than this -> not active
        self.declare_parameter("vision_frame", "camera")
        self.declare_parameter("publish_debug_image", False)
        # Phone IP-webcam apps often stream sideways/upside-down depending on how the phone is
        # held. Rotate the frame to upright (0|90|180|270, CLOCKWISE) so image x/y — and thus the
        # teleop control axes — match the operator's real left-right / up-down.
        self.declare_parameter("rotate_deg", 0)
        self.declare_parameter("hand_pose_topic", "vision/hand_pose")
        self.declare_parameter("grip_topic", "vision/grip")
        self.declare_parameter("tracking_topic", "vision/tracking_active")
        self.declare_parameter("image_topic", "vision/image")
        # --- hand-skeleton annotation stream (the recorded "second data stream") ---
        # Publish the full 21-landmark skeleton + per-finger joint angles + a compact
        # quality signal, per hand, on <ns>/<left|right>/{skeleton,quality}. Set
        # num_hands:=2 to capture BOTH hands (control still uses one). Independent of the
        # control publishers above.
        self.declare_parameter("publish_skeleton", True)
        self.declare_parameter("skeleton_topic_ns", "vision")
        # MediaPipe reports handedness for a mirrored (selfie) view. On a raw (un-mirrored)
        # feed the labels are swapped, so we flip them to anatomical left/right. Set true
        # only if the feed is already mirrored.
        self.declare_parameter("selfie_view", False)

        gp = self.get_parameter
        self._mode = gp("tracking_mode").value
        if self._mode not in ("hand", "full_body"):
            self.get_logger().warning(
                f"tracking_mode='{self._mode}' unknown; falling back to 'hand'."
            )
            self._mode = "hand"
        side = str(gp("body_side").value).lower()
        # MediaPipe Pose indices: anatomical left/right (un-mirrored feed -> labels true).
        self._wrist_idx, self._elbow_idx = (16, 14) if side == "right" else (15, 13)
        self._min_vis = float(gp("min_joint_visibility").value)
        self._frame = gp("vision_frame").value
        self._stale = float(gp("stale_timeout").value)
        self._debug = bool(gp("publish_debug_image").value)
        self._rotate = self._rotate_flag(int(gp("rotate_deg").value))
        self._hand_label = gp("hand_label").value          # raw model label to drive control
        self._selfie_view = bool(gp("selfie_view").value)
        self._publish_skeleton = bool(gp("publish_skeleton").value)
        self._skeleton_ns = str(gp("skeleton_topic_ns").value).rstrip("/")
        self._input_mode = str(gp("input_mode").value).lower()
        self._in_stamp = None                              # latest camera-frame stamp (topic mode)

        # --- publishers ---
        self._pose_pub = self.create_publisher(PoseStamped, gp("hand_pose_topic").value, qos_profile_sensor_data)
        self._grip_pub = self.create_publisher(Float64, gp("grip_topic").value, qos_profile_sensor_data)
        self._active_pub = self.create_publisher(Bool, gp("tracking_topic").value, 10)
        self._image_pub = (
            self.create_publisher(CompressedImage, gp("image_topic").value, qos_profile_sensor_data)
            if self._debug else None
        )
        # Per-hand annotation publishers (skeleton + quality), one pair each for "left"
        # and "right". The control publishers above are untouched; these are the recorded
        # second stream. Only in hand mode (full_body has no finger landmarks).
        # Per-hand streams: skeleton + quality (annotation), plus a control pose + grip so a
        # per-arm mirror_source can drive BOTH arms (bimanual control). The single-hand
        # /vision/hand_pose above is unchanged for the one-arm path.
        self._skel_pubs = None
        self._qual_pubs = None
        self._hpose_pubs = None
        self._hgrip_pubs = None
        self._hactive_pubs = None
        if self._publish_skeleton and self._mode == "hand":
            self._skel_pubs, self._qual_pubs = {}, {}
            self._hpose_pubs, self._hgrip_pubs, self._hactive_pubs = {}, {}, {}
            for _label in ("left", "right"):
                self._hactive_pubs[_label] = self.create_publisher(
                    Bool, "%s/%s/tracking_active" % (self._skeleton_ns, _label), 10)
                self._skel_pubs[_label] = self.create_publisher(
                    HandSkeleton, "%s/%s/skeleton" % (self._skeleton_ns, _label),
                    qos_profile_sensor_data)
                self._qual_pubs[_label] = self.create_publisher(
                    HandQuality, "%s/%s/quality" % (self._skeleton_ns, _label),
                    qos_profile_sensor_data)
                self._hpose_pubs[_label] = self.create_publisher(
                    PoseStamped, "%s/%s/hand_pose" % (self._skeleton_ns, _label),
                    qos_profile_sensor_data)
                self._hgrip_pubs[_label] = self.create_publisher(
                    Float64, "%s/%s/grip" % (self._skeleton_ns, _label),
                    qos_profile_sensor_data)

        # --- shared state written by the worker, read by the timer ---
        self._lock = threading.Lock()
        self._sample = _Sample()
        self._hand_samples = {}            # anat label -> _HandSample (both hands)
        self._stop = threading.Event()

        # Topic input: an image subscription feeds a _TopicFrameSource the worker consumes.
        self._topic_source = None
        if self._input_mode == "topic":
            self._topic_source = _TopicFrameSource()
            self.create_subscription(
                Image, gp("input_image_topic").value, self._on_image, qos_profile_sensor_data)

        self._worker = threading.Thread(target=self._run, name="hand_tracker_worker", daemon=True)
        self._worker.start()

        period = 1.0 / max(float(gp("publish_rate").value), 1.0)
        self._timer = self.create_timer(period, self._publish)
        self.get_logger().info("hand_tracker up (mode=%s, frame=%s)" % (self._mode, self._frame))

    # --- model path resolution ---
    def _resolve_model_path(self):
        path = self.get_parameter("model_path").value
        if path:
            return path
        model_file = _MODEL_FILES.get(self._mode, _MODEL_FILES["hand"])
        return os.path.join(get_package_share_directory("fm_teleop_vision"), "models", model_file)

    # --- worker thread: blocking capture + inference ---
    def _run(self):
        gp = self.get_parameter
        try:
            if self._input_mode == "topic":
                source = self._topic_source          # fed by the image subscription
            else:
                source = CameraSource(gp("camera_source").value, backend=gp("backend").value)
            if self._mode == "full_body":
                est = pose_mod.PoseEstimator(
                    self._resolve_model_path(),
                    num_poses=1,
                    min_pose_detection_confidence=float(gp("min_detection_confidence").value),
                    min_pose_presence_confidence=float(gp("min_presence_confidence").value),
                    min_tracking_confidence=float(gp("min_tracking_confidence").value),
                )
            else:
                est = hand_mod.HandEstimator(
                    self._resolve_model_path(),
                    num_hands=int(gp("num_hands").value),
                    min_hand_detection_confidence=float(gp("min_detection_confidence").value),
                    min_hand_presence_confidence=float(gp("min_presence_confidence").value),
                    min_tracking_confidence=float(gp("min_tracking_confidence").value),
                    preferred_handedness=(gp("hand_label").value or None),
                )
        except Exception as exc:  # bad model path, camera open failure, etc.
            self.get_logger().error("hand_tracker init failed: %r" % exc)
            return

        mc = float(gp("filter_min_cutoff").value)
        beta = float(gp("filter_beta").value)
        dc = float(gp("filter_d_cutoff").value)
        use_filter = bool(gp("filter_enabled").value)
        self._flt_params = (mc, beta, dc)
        # SkeletonFilter smooths the WORLD landmarks (orientation + curl). Hand mode keeps
        # ONE filter per hand (landmark indices collide across hands), created lazily in
        # _store_hands. full_body uses none. A separate One-Euro trio smooths the CONTROL
        # POSITION, derived from IMAGE coordinates and so otherwise unfiltered.
        skel_filters = {}
        pos_filters = {}          # per-hand control-position One-Euro trio (bimanual)
        pos_filt = [OneEuroFilter(mc, beta, dc) for _ in range(3)] if use_filter else None

        prev_ts = None
        try:
            for frame_bgr, wall_ts in source.frames():
                if self._stop.is_set():
                    break
                if self._rotate is not None:
                    # De-rotate a sideways phone stream to upright, so image x/y (and thus the
                    # control axes) match the operator's real left-right / up-down. Done before
                    # detection so landmarks + the debug overlay are both in the upright frame.
                    import cv2
                    frame_bgr = cv2.rotate(frame_bgr, self._rotate)
                dt = (wall_ts - prev_ts) if prev_ts else (1.0 / 30.0)
                prev_ts = wall_ts
                if self._mode == "full_body":
                    self._store_pose(est.process(frame_bgr, wall_ts), frame_bgr, dt, pos_filt)
                else:
                    hands = est.process_hands(frame_bgr, wall_ts)
                    self._store_hands(hands, frame_bgr, dt, pos_filt,
                                      skel_filters, pos_filters, use_filter)
        except Exception as exc:
            self.get_logger().error("hand_tracker worker stopped: %r" % exc)
        finally:
            source.release()
            try:
                est.close()
            except Exception:
                pass

    def _on_image(self, msg):
        """Topic input: convert an incoming camera frame and hand it to the worker. Runs on
        the executor thread; heavy inference stays on the worker (via the frame source)."""
        try:
            bgr = _image_msg_to_bgr(msg)
        except Exception as e:  # noqa: BLE001 — surface a bad encoding, keep the node alive
            self.get_logger().warn("image convert failed: %r" % e, throttle_duration_sec=5.0)
            return
        st = msg.header.stamp
        wall_ts = st.sec + st.nanosec * 1e-9
        if wall_ts <= 0.0:
            wall_ts = time.time()
        self._in_stamp = msg.header.stamp    # share the camera stamp with the skeleton stream
        if self._topic_source is not None:
            self._topic_source.push(bgr, wall_ts)

    def _store(self, hf, frame_bgr, dt, pos_filt, overlay_frames=None):
        s = _Sample()
        s.wall_ts = time.time()
        if hf.detected:
            lms = {lm.idx: (lm.wx, lm.wy, lm.wz) for lm in hf.landmarks}
            img = {lm.idx: (lm.px, lm.py) for lm in hf.landmarks}
            wrist_w = lms[hand_mod.WRIST]
            s.quat = mapping.palm_orientation(
                wrist_w, lms[hand_mod.INDEX_MCP], lms[hand_mod.MIDDLE_MCP], lms[hand_mod.PINKY_MCP]
            )
            s.curl = mapping.finger_curl(
                wrist_w,
                lms[hand_mod.MIDDLE_MCP],
                [lms[i] for i in hand_mod.MCPS],
                [lms[i] for i in hand_mod.FINGERTIPS],
            )
            pos = mapping.control_position(
                img[hand_mod.WRIST], img[hand_mod.MIDDLE_MCP], hf.image_w
            )
            if pos_filt is not None:
                pos = tuple(pos_filt[i](pos[i], dt) for i in range(3))
            s.pos = pos
            s.detected = True
        if self._debug:
            # In hand mode _store_hands passes every tracked hand so the overlay shows BOTH
            # (bimanual) — not just the control hand. overlay_frames=None keeps the single-hand
            # encode for the full_body path and any caller that does not supply the list.
            s.jpeg = (self._encode_hands(frame_bgr, overlay_frames)
                      if overlay_frames is not None
                      else self._encode(frame_bgr, hf))
        with self._lock:
            self._sample = s

    def _store_hands(self, hands, frame_bgr, dt, pos_filt, skel_filters, pos_filters, use_filter):
        """Reduce EVERY detected hand to a _HandSample (the per-hand annotation + control
        streams) and the selected hand to the control _Sample. Control reuses _store
        unchanged, so the one-arm mirror path behaves exactly as before. Each hand's world
        smoothing filter is applied once per frame and reused for both sample and control."""
        samples = {}
        filtered_list = []           # filtered frames in detection order
        control_filtered = None
        for hf in hands:
            label = self._anat_label(hf.handedness)
            key = label or "unknown"
            if use_filter:
                flt = skel_filters.get(key)
                if flt is None:
                    flt = SkeletonFilter(*self._flt_params)
                    skel_filters[key] = flt
                filtered = flt.apply(hf, dt)
                pf = pos_filters.get(key)
                if pf is None:
                    pf = [OneEuroFilter(*self._flt_params) for _ in range(3)]
                    pos_filters[key] = pf
            else:
                filtered = hf
                pf = None
            samples[key] = self._build_hand_sample(hf, filtered, label, dt, pf)
            filtered_list.append(filtered)
            if control_filtered is None and self._is_control_hand(hf.handedness):
                control_filtered = filtered
        if control_filtered is None and filtered_list:
            control_filtered = filtered_list[0]   # first detected hand (old _select fallback)
        if control_filtered is None:
            control_filtered = hand_mod.HandFrame(False, [], "", 0, 0, 0, 0.0)
        # Draw ALL tracked hands on the debug overlay (both hands in bimanual capture), while
        # control still follows the single selected hand.
        self._store(control_filtered, frame_bgr, dt, pos_filt, overlay_frames=filtered_list)
        with self._lock:
            self._hand_samples = samples

    def _anat_label(self, raw):
        """Anatomically-correct a raw MediaPipe handedness label. MediaPipe labels a
        mirrored (selfie) view, so on a raw feed the labels are swapped — flip them unless
        selfie_view. Returns "" for unknown."""
        if not raw:
            return ""
        lab = raw.strip().lower()
        if lab not in ("left", "right"):
            return lab
        if self._selfie_view:
            return lab
        return "left" if lab == "right" else "right"

    def _is_control_hand(self, raw):
        """Control keeps the old preference: match hand_label (raw model label) when set;
        otherwise no preference (the caller falls back to the first detected hand)."""
        pref = self._hand_label
        if not pref:
            return False
        return bool(raw) and raw.strip().lower() == str(pref).strip().lower()

    def _build_hand_sample(self, raw_hf, filtered_hf, label, dt, pos_filt):
        hs = _HandSample()
        hs.wall_ts = time.time()
        hs.hand = label
        hs.detected = True
        hs.confidence = float(getattr(filtered_hf, "score", 0.0) or 0.0)
        n = hand_mod.NUM_LANDMARKS
        wl = {lm.idx: (lm.wx, lm.wy, lm.wz) for lm in filtered_hf.landmarks}
        il = {lm.idx: (lm.px, lm.py) for lm in raw_hf.landmarks}   # image coords: raw (unsmoothed)
        hs.world_lms = [wl.get(i, (0.0, 0.0, 0.0)) for i in range(n)]
        hs.image_lms = [il.get(i, (0.0, 0.0)) for i in range(n)]
        hs.joint_angles = mapping.finger_joint_angles(hs.world_lms)
        hs.quat = mapping.palm_orientation(
            hs.world_lms[hand_mod.WRIST], hs.world_lms[hand_mod.INDEX_MCP],
            hs.world_lms[hand_mod.MIDDLE_MCP], hs.world_lms[hand_mod.PINKY_MCP])
        hs.curl = mapping.finger_curl(
            hs.world_lms[hand_mod.WRIST], hs.world_lms[hand_mod.MIDDLE_MCP],
            [hs.world_lms[i] for i in hand_mod.MCPS],
            [hs.world_lms[i] for i in hand_mod.FINGERTIPS])
        # Control position (wrist -> x, y, apparent-size depth) from raw IMAGE landmarks,
        # smoothed by this hand's One-Euro trio — same reduction as the single-hand control.
        pos = mapping.control_position(
            il.get(hand_mod.WRIST, (0.0, 0.0)), il.get(hand_mod.MIDDLE_MCP, (0.0, 0.0)),
            raw_hf.image_w)
        if pos_filt is not None:
            pos = tuple(pos_filt[i](pos[i], dt) for i in range(3))
        hs.pos = pos
        hs.in_frame = self._all_in_frame(raw_hf)
        hs.jitter = self._lm_jitter(raw_hf, filtered_hf)
        return hs

    @staticmethod
    def _all_in_frame(hf):
        w, h = hf.image_w, hf.image_h
        if w <= 0 or h <= 0:
            return False
        return all(0.0 <= lm.px <= w and 0.0 <= lm.py <= h for lm in hf.landmarks)

    @staticmethod
    def _lm_jitter(raw, filtered):
        """Mean per-landmark displacement (metres) between the raw and smoothed world
        landmarks — a temporal-noise proxy: higher = jerkier tracking this frame."""
        r = {lm.idx: (lm.wx, lm.wy, lm.wz) for lm in raw.landmarks}
        f = {lm.idx: (lm.wx, lm.wy, lm.wz) for lm in filtered.landmarks}
        keys = set(r) & set(f)
        if not keys:
            return 0.0
        total = 0.0
        for k in keys:
            rx, ry, rz = r[k]
            fx, fy, fz = f[k]
            total += ((rx - fx) ** 2 + (ry - fy) ** 2 + (rz - fz) ** 2) ** 0.5
        return total / len(keys)

    def _store_pose(self, pf, frame_bgr, dt, pos_filt):
        """full_body reduction: the body-model WRIST is the control point, the FOREARM
        (elbow->wrist) is the apparent-size depth proxy (so vision_source's hand_span_m
        must be the operator's forearm length, ~0.26 m). No grip (no finger landmarks);
        orientation identity (angular is off). Low-visibility wrist/elbow counts as NOT
        tracked — vision_source then holds rather than chase extrapolated joints."""
        s = _Sample()
        s.wall_ts = time.time()
        if pf.detected:
            js = {j.idx: j for j in pf.joints}
            wrist = js.get(self._wrist_idx)
            elbow = js.get(self._elbow_idx)
            if (wrist is not None and elbow is not None
                    and min(wrist.visibility, elbow.visibility) >= self._min_vis):
                pos = mapping.control_position(
                    (wrist.px, wrist.py), (elbow.px, elbow.py), pf.image_w
                )
                if pos_filt is not None:
                    pos = tuple(pos_filt[i](pos[i], dt) for i in range(3))
                s.pos = pos
                s.detected = True
        if self._debug:
            s.jpeg = self._encode(frame_bgr, pf)
        with self._lock:
            self._sample = s

    @staticmethod
    def _rotate_flag(deg):
        """Map a CLOCKWISE rotation in degrees (0|90|180|270) to a cv2.rotate flag, None if 0."""
        deg = int(deg) % 360
        if deg == 0:
            return None
        import cv2
        return {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
                270: cv2.ROTATE_90_COUNTERCLOCKWISE}.get(deg)

    def _encode(self, frame_bgr, hf):
        try:
            import cv2  # provided by the base image (mediapipe pulls opencv)
            draw = pose_mod.draw_overlay if self._mode == "full_body" else hand_mod.draw_overlay
            annotated = draw(frame_bgr, hf)
            ok, buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if not ok:
                self.get_logger().warn(
                    "debug image: imencode failed (shape=%r dtype=%r)"
                    % (getattr(annotated, "shape", None), getattr(annotated, "dtype", None)),
                    throttle_duration_sec=2.0,
                )
            return bytes(buf) if ok else None
        except Exception as e:  # noqa: BLE001 — surface why the overlay isn't publishing
            self.get_logger().warn("debug image encode failed: %r" % e, throttle_duration_sec=2.0)
            return None

    def _encode_hands(self, frame_bgr, hand_frames):
        """Encode the debug overlay with EVERY tracked hand drawn (hand mode). Mirrors
        _encode but draws the whole list so bimanual capture shows both hands."""
        try:
            import cv2
            annotated = hand_mod.draw_hands_overlay(frame_bgr, hand_frames)
            ok, buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if not ok:
                self.get_logger().warn(
                    "debug image: imencode failed (shape=%r dtype=%r)"
                    % (getattr(annotated, "shape", None), getattr(annotated, "dtype", None)),
                    throttle_duration_sec=2.0,
                )
            return bytes(buf) if ok else None
        except Exception as e:  # noqa: BLE001 — surface why the overlay isn't publishing
            self.get_logger().warn("debug image encode failed: %r" % e, throttle_duration_sec=2.0)
            return None

    # --- timer: publish the latest sample at a steady rate ---
    def _publish(self):
        with self._lock:
            s = self._sample
            hand_samples = self._hand_samples
        # Annotation streams first, independent of the control hand: e.g. the left hand
        # may be captured while the right (control) hand is absent.
        if self._skel_pubs is not None:
            self._publish_skeletons(hand_samples)
        active = bool(s.detected and (time.time() - s.wall_ts) < self._stale)
        self._active_pub.publish(Bool(data=active))
        if not active:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = s.pos
        qw, qx, qy, qz = s.quat
        msg.pose.orientation.w = qw
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        self._pose_pub.publish(msg)
        self._grip_pub.publish(Float64(data=float(s.curl)))
        if self._image_pub is not None and s.jpeg is not None:
            img = CompressedImage()
            img.header = msg.header
            img.format = "jpeg"
            img.data = s.jpeg
            self._image_pub.publish(img)

    def _publish_skeletons(self, hand_samples):
        """Publish the fresh per-hand HandSkeleton + HandQuality. Runs on the executor
        thread (publishers are not touched from the worker). A hand with no fresh sample
        just goes quiet — consumers/recorder infer absence from staleness."""
        # In topic mode stamp with the camera frame time so the skeleton and the recorded
        # frame share one clock; otherwise stamp at publish.
        now = (self._in_stamp if (self._input_mode == "topic" and self._in_stamp is not None)
               else self.get_clock().now().to_msg())
        for label in ("left", "right"):
            hs = hand_samples.get(label)
            fresh = hs is not None and hs.detected and (time.time() - hs.wall_ts) < self._stale
            # tracking_active is published EVERY tick (True/False) so a per-arm mirror_source
            # sees the hand drop and stops; the data streams below publish only when fresh.
            self._hactive_pubs[label].publish(Bool(data=bool(fresh)))
            if not fresh:
                continue
            skel = HandSkeleton()
            skel.header.stamp = now
            skel.header.frame_id = self._frame
            skel.hand = label
            skel.detected = True
            skel.confidence = float(hs.confidence)
            skel.world_landmarks = [Point(x=float(x), y=float(y), z=float(z))
                                    for (x, y, z) in hs.world_lms]
            skel.image_landmarks = [Point(x=float(px), y=float(py), z=0.0)
                                    for (px, py) in hs.image_lms]
            qw, qx, qy, qz = hs.quat
            skel.palm_orientation = Quaternion(w=float(qw), x=float(qx), y=float(qy), z=float(qz))
            skel.joint_angles = [float(a) for a in hs.joint_angles]
            skel.grip = float(hs.curl)
            self._skel_pubs[label].publish(skel)

            q = HandQuality()
            q.header.stamp = now
            q.header.frame_id = self._frame
            q.hand = label
            q.detected = True
            q.confidence = float(hs.confidence)
            q.in_frame = bool(hs.in_frame)
            q.jitter = float(hs.jitter)
            self._qual_pubs[label].publish(q)

            # Per-hand control pose + grip — the input a per-arm mirror_source subscribes to
            # for bimanual control (left hand -> left arm, right hand -> right arm).
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = self._frame
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = hs.pos
            pose.pose.orientation.w = float(qw)
            pose.pose.orientation.x = float(qx)
            pose.pose.orientation.y = float(qy)
            pose.pose.orientation.z = float(qz)
            self._hpose_pubs[label].publish(pose)
            self._hgrip_pubs[label].publish(Float64(data=float(hs.curl)))

    def destroy_node(self):
        self._stop.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
