"""Vision wrist teleop source — move a hand in front of a camera, the arm jogs.

A camera tracks the operator's wrist (MediaPipe Pose, world position in metres). Holding
the wrist away from a neutral pose jogs the arm in that direction through MoveIt Servo;
returning to neutral stops it. This is the smallest vision loop behind the teleop
contract: one wrist, linear twist only, ``arm_twist`` channel — gripper and orientation
are deferred.

    camera frame ->  WristTracker (MediaPipe)  ->  One-Euro filter  ->  displacement
                     from the engage-time neutral  ->  TwistStamped on arm_twist -> Servo

Engagement is a deadman on ``/vision_teleop/enable`` (std_msgs/Bool, from the panel): the
rising edge captures the current wrist as neutral, and the arm jogs only while it stays
true. Releasing it publishes one zero twist so Servo holds immediately.

Trust + safety: ``camera_source``, ``model_path``, ``scale``, and ``clamp`` are operator
launch params, read once at construction (runtime ``ros2 param set`` cannot mutate them
after). ``scale`` and ``clamp`` are the teleop speed knobs; the published twist is the
*requested* jog, and MoveIt Servo enforces the final joint-velocity, singularity, and
collision limits downstream — it is the safety backstop, not this node.

OpenCV and MediaPipe are imported lazily by the default capture/tracker factories, so the
module imports — and the node constructs with injected fakes — without them. That keeps
the node smoke test hermetic (no camera, no model) and colcon test green on a base image.
"""

from geometry_msgs.msg import TwistStamped
import rclpy
from std_msgs.msg import Bool

from fm_teleop_core.retarget import displacement_to_twist
from fm_teleop_core.source import TeleopSource
from fm_teleop_vision.filters import Vec3OneEuro


def _default_capture(source):
    """Open an OpenCV capture from a webcam index (``"0"``) or a stream URL."""
    import cv2

    spec = int(source) if str(source).isdigit() else source
    return cv2.VideoCapture(spec)


class VisionSource(TeleopSource):
    """Track a wrist and jog the arm via Servo while the deadman is held."""

    def __init__(self, *, capture_factory=None, tracker_factory=None):
        super().__init__("vision_source")

        # Camera + model.
        self.declare_parameter("camera_source", "0")
        self.declare_parameter("model_path", "models/pose_landmarker_heavy.task")
        self.declare_parameter("wrist_side", "right")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("min_visibility", 0.5)
        # Retarget: displacement (m) from neutral -> Servo's unitless twist command.
        self.declare_parameter("command_frame", "openarm_right_base_link")
        self.declare_parameter("scale", 4.0)
        self.declare_parameter("deadzone", 0.03)
        self.declare_parameter("clamp", 1.0)
        # Per-axis enable; Z is the suspect axis (MediaPipe depth is noisiest), so it
        # can be dropped without touching X/Y.
        self.declare_parameter("use_x", True)
        self.declare_parameter("use_y", True)
        self.declare_parameter("use_z", True)
        # One-Euro filter on the wrist world position.
        self.declare_parameter("filter_min_cutoff", 1.0)
        self.declare_parameter("filter_beta", 0.02)
        self.declare_parameter("filter_d_cutoff", 1.0)
        # Topics.
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("enable_topic", "/vision_teleop/enable")

        self._frame = self.get_parameter("command_frame").value
        self._scale = self.get_parameter("scale").value
        self._deadzone = self.get_parameter("deadzone").value
        self._clamp = self.get_parameter("clamp").value
        self._use_axes = [
            self.get_parameter("use_x").value,
            self.get_parameter("use_y").value,
            self.get_parameter("use_z").value,
        ]
        self._min_visibility = self.get_parameter("min_visibility").value

        self._filter = Vec3OneEuro(
            self.get_parameter("filter_min_cutoff").value,
            self.get_parameter("filter_beta").value,
            self.get_parameter("filter_d_cutoff").value,
        )

        # Engagement state.
        self._enabled = False
        self._capture_neutral = False  # set on the rising edge, consumed next good frame
        self._neutral = None
        self._last_tick = None

        # Injectable factories keep the heavy deps (OpenCV, MediaPipe) out of the test
        # path; the defaults build the real capture + tracker lazily.
        self._capture_factory = capture_factory or _default_capture
        self._tracker_factory = tracker_factory or self._build_tracker
        self._capture = self._capture_factory(self.get_parameter("camera_source").value)
        self._tracker = self._tracker_factory()

        self._pub = self.contract_publisher(
            "arm_twist", topic=self.get_parameter("twist_topic").value
        )
        self.create_subscription(
            Bool, self.get_parameter("enable_topic").value, self._on_enable, 10
        )
        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.create_timer(period, self._on_tick)
        self.get_logger().info(
            f"vision_source up: wrist={self.get_parameter('wrist_side').value}, "
            f"frame={self._frame}, deadman={self.get_parameter('enable_topic').value}"
        )

    def _build_tracker(self):
        """Default tracker factory — lazily imports MediaPipe (real camera path)."""
        from fm_teleop_vision.pose import WristTracker

        return WristTracker(
            self.get_parameter("model_path").value,
            side=self.get_parameter("wrist_side").value,
        )

    def _on_enable(self, msg):
        was = self._enabled
        self._enabled = bool(msg.data)
        if self._enabled and not was:
            # Rising edge: capture the next good wrist sample as the neutral origin.
            self._capture_neutral = True
            self.get_logger().info("vision teleop engaged — capturing neutral")
        elif was and not self._enabled:
            # Falling edge: hold the arm immediately, forget the neutral + filter state.
            self._publish_zero()
            self._neutral = None
            self._capture_neutral = False
            self._filter.reset()
            self.get_logger().info("vision teleop released — holding")

    def _on_tick(self):
        ok, frame = self._capture.read()
        if not ok or frame is None:
            return

        sample = self._tracker.process(frame)
        # Lost or low-confidence track: hold while engaged, and drop filter history so a
        # re-acquire does not jump.
        if not sample.detected or sample.visibility < self._min_visibility:
            if self._enabled:
                self._publish_zero()
            self._filter.reset()
            return

        dt = self._dt()
        filtered = self._filter([sample.wx, sample.wy, sample.wz], dt)
        position = self._to_command_frame(filtered)

        if self._capture_neutral:
            self._neutral = position
            self._capture_neutral = False
            self._publish_zero()  # at neutral, the command is zero by definition
            return

        if not self._enabled or self._neutral is None:
            return

        twist = displacement_to_twist(
            position, self._neutral, self._scale, self._deadzone, self._clamp
        )
        twist = [v if use else 0.0 for v, use in zip(twist, self._use_axes)]
        self._publish_twist(twist)

    def _dt(self):
        """Seconds since the last tick, for the filter. Zero on the first call.

        The first sample only seeds the filter (which ignores dt on its seed path), so
        returning 0.0 keeps that explicit rather than inventing a nominal interval.
        """
        now = self.get_clock().now().nanoseconds * 1e-9  # ROS nanoseconds -> seconds
        if self._last_tick is None:
            self._last_tick = now
            return 0.0
        dt = now - self._last_tick
        self._last_tick = now
        return dt

    def _to_command_frame(self, xyz):
        """Map a MediaPipe world point to the arm command frame (REP-103).

        MediaPipe world axes are image-style: +x right, +y down, +z toward the camera.
        The command frame is REP-103: +x forward, +y left, +z up. With the operator
        facing the camera, the mapping is:

            forward (+x) <- +z   (push the hand toward the camera -> arm forward)
            left    (+y) <- -x   (hand to the operator's right -> arm right, -y)
            up      (+z) <- -y   (hand up -> arm up)

        These three signs are the first thing to flip if the arm jogs the wrong way
        during bringup — expect to tune them once against the live sim.
        """
        x, y, z = xyz
        return [z, -x, -y]

    def _publish_twist(self, linear):
        msg = TwistStamped()
        msg.header = self.stamped_header(self._frame)
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = linear
        self._pub.publish(msg)

    def _publish_zero(self):
        self._publish_twist([0.0, 0.0, 0.0])

    def destroy_node(self):
        # Release the camera + model handles the factories opened.
        if getattr(self, "_capture", None) is not None:
            release = getattr(self._capture, "release", None)
            if release:
                release()
        if getattr(self, "_tracker", None) is not None:
            close = getattr(self._tracker, "close", None)
            if close:
                close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
