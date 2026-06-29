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

Position is in normalized-image-WIDTH units: x = wrist/width in [0,1], y = wrist/width in
[0, h/w] (ALL axes share one unit so a single metres-per-unit factor applies — see
mapping.control_position), and z = an apparent-size depth proxy (hand bigger = closer).
Single-camera depth is coarse — that is exactly the limitation the upstream perception
proof measured — so the depth axis is gain-gated separately (see vision_source).
Orientation + curl come from the metric ``hand_world_landmarks`` and are reliable.

Camera: on Linux (compose.linux.yaml, /dev passthrough) ``camera_source`` may be a device
index like "0"; on Mac/OrbStack there is no camera passthrough, so use an http/rtsp URL
(a phone IP-webcam app on the OrbStack network).
"""

import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float64

from fm_teleop_vision import body_pose as pose_mod
from fm_teleop_vision import hand as hand_mod
from fm_teleop_vision import mapping
from fm_teleop_vision.capture import CameraSource
from fm_teleop_vision.filters import OneEuroFilter, SkeletonFilter

_MODEL_FILES = {"hand": "hand_landmarker.task", "full_body": "pose_landmarker_heavy.task"}


class _Sample:
    __slots__ = ("detected", "pos", "quat", "curl", "wall_ts", "jpeg")

    def __init__(self):
        self.detected = False
        self.pos = (0.0, 0.0, 0.0)
        self.quat = mapping.IDENTITY_QUAT
        self.curl = 0.0
        self.wall_ts = 0.0
        self.jpeg = None


class HandTracker(Node):
    def __init__(self):
        super().__init__("hand_tracker")

        # --- parameters ---
        self.declare_parameter("camera_source", "0")  # int index (Linux) or URL (Mac)
        self.declare_parameter("backend", "auto")      # auto | v4l2 | ffmpeg | avfoundation
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

        # --- publishers ---
        self._pose_pub = self.create_publisher(PoseStamped, gp("hand_pose_topic").value, qos_profile_sensor_data)
        self._grip_pub = self.create_publisher(Float64, gp("grip_topic").value, qos_profile_sensor_data)
        self._active_pub = self.create_publisher(Bool, gp("tracking_topic").value, 10)
        self._image_pub = (
            self.create_publisher(CompressedImage, gp("image_topic").value, qos_profile_sensor_data)
            if self._debug else None
        )

        # --- shared state written by the worker, read by the timer ---
        self._lock = threading.Lock()
        self._sample = _Sample()
        self._stop = threading.Event()

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
            cam = CameraSource(gp("camera_source").value, backend=gp("backend").value)
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
        # SkeletonFilter smooths the WORLD landmarks (orientation + curl — hand mode
        # only; full_body uses neither). A separate One-Euro trio smooths the CONTROL
        # POSITION, which is derived from IMAGE coordinates and so would otherwise be
        # unfiltered.
        flt = SkeletonFilter(mc, beta, dc) if (use_filter and self._mode == "hand") else None
        pos_filt = [OneEuroFilter(mc, beta, dc) for _ in range(3)] if use_filter else None

        prev_ts = None
        try:
            for frame_bgr, wall_ts in cam.frames():
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
                hf = est.process(frame_bgr, wall_ts)
                if flt is not None:
                    hf = flt.apply(hf, dt)
                if self._mode == "full_body":
                    self._store_pose(hf, frame_bgr, dt, pos_filt)
                else:
                    self._store(hf, frame_bgr, dt, pos_filt)
        except Exception as exc:
            self.get_logger().error("hand_tracker worker stopped: %r" % exc)
        finally:
            cam.release()
            try:
                est.close()
            except Exception:
                pass

    def _store(self, hf, frame_bgr, dt, pos_filt):
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
            s.jpeg = self._encode(frame_bgr, hf)
        with self._lock:
            self._sample = s

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

    # --- timer: publish the latest sample at a steady rate ---
    def _publish(self):
        with self._lock:
            s = self._sample
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
