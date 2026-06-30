"""mirror_source — teleop contract source: hand pose stream -> absolute EE pose target.

The 1:1 hand-mirroring counterpart to ``vision_source`` (which is wrist velocity jogging):
this source HOLDS an absolute pose instead of jogging, and runs alongside it (a distinct
node + teleop input), so neither replaces the other.

Subscribes to the generic stream from ``hand_tracker`` and turns it into the teleop
contract's ``arm_pose_target`` (a ``geometry_msgs/PoseStamped``) consumed by the MoveIt
Servo *PoseTracking* node (``pose_tracking_node`` in ``fm_control``), plus an optional
gripper ``hand_preset``. It holds no camera or model code — only the mapping from a tracked
hand to a robot pose target — so it depends just on rclpy + fm_teleop_core + tf2 + standard
messages (the perception deps stay in ``hand_tracker``).

Control model — INDEXED POSITION MIRRORING (a "mouse pickup"), NOT velocity jogging. On
engage we latch a reference: the hand pose, the current EE pose (read from tf2,
``command_frame`` -> ``ee_frame``) AND the metric conversion W_m = the image width in
metres at the hand's distance (pinhole, from the apparent hand size — see
mapping.image_width_m). Each tick we publish an ABSOLUTE target:

    ee_target = ee_ref + mirror_gain * W_m * remap(hand_now - hand_ref)   (clamped to a box)

so with mirror_gain=1 the EE travels the same METRES the operator's hand really moved
(~1:1 mirroring), independent of how far the operator stands from the camera.

PoseTracking servos the EE to that target and HOLDS. A steady hand -> a steady target -> the
arm holds: no glide and, crucially, no *integrated* drift to a singularity. (This replaces the
earlier clutch-velocity model, which published a Twist and drifted toward the straight-arm
singularity on any steady bias in the noisy hand signal.) Orientation is held constant at the
EE orientation latched on engage (the mono landmark orientation is unreliable — angular off).

    vision/engage = true  -> latch hand_ref + ee_ref (from tf2), start mirroring
    move hand            -> EE tracks the scaled, mirrored hand offset and holds when it stops
    disengage / tracking lost / stale -> STOP publishing; PoseTracking times out and holds

Publishes:
    arm_pose_target (PoseStamped) -> /target_pose   [contract]  (PoseTracking node consumes it)
    hand_preset (String)          -> gripper_teleop  [contract, optional]
"""

from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, Float64, String
import tf2_ros

from fm_teleop_core import retarget  # noqa: F401  (kept on the path; mapping reuses it)
from fm_teleop_core.source import TeleopSource
from fm_teleop_vision import mapping


class MirrorSource(TeleopSource):
    """Map a tracked hand pose onto an absolute EE pose target (+ optional gripper)."""

    def __init__(self):
        super().__init__("mirror_source")

        # --- arm command ---
        # command_frame is BOTH the Servo planning frame and the tf target we read the EE
        # pose in, so the latched ee_ref and the published target are already in the frame
        # PoseTracking expects (no extra transform). ee_frame is the tf source link (the EE).
        self.declare_parameter("command_frame", "openarm_right_base_link")
        self.declare_parameter("ee_frame", "openarm_right_link7")
        self.declare_parameter("target_pose_topic", "/target_pose")
        self.declare_parameter("publish_rate", 50.0)        # Hz; steady stream feeds PoseTracking
        self.declare_parameter("command_timeout", 0.2)      # s; hand pose older than this -> stop

        # --- mirroring (per input axis: image-x, image-y, depth) ---
        # Hand position arrives in normalized-image-WIDTH units; the metres-per-unit
        # conversion W_m (image width in metres at the hand) is derived from the apparent
        # hand size and LATCHED AT ENGAGE (mapping.image_width_m), so mirror_gain=1.0 is
        # ~1:1 physical mirroring at any camera distance. axis_gain trims/enables each
        # INPUT axis (image-x, image-y, depth): depth is the weakest (mono apparent-size
        # proxy), so it defaults to 0 — camera-plane-only mirroring first; opt in once x,y
        # feels right. fallback_scale (m/unit) is used only when the hand size at engage is
        # degenerate. axis_map remaps the operator image axes into command-frame axes; tune
        # the signs by eye. All live-tunable (see _on_params).
        self.declare_parameter("mirror_gain", 1.0)                   # unitless; 1.0 = 1:1
        self.declare_parameter("axis_gain", [1.0, 1.0, 0.0])         # per input axis
        self.declare_parameter("hand_span_m", 0.09)                  # wrist -> middle MCP
        self.declare_parameter("fallback_scale", 0.9)                # m/unit if size degenerate
        self.declare_parameter("axis_map_linear", ["z", "x", "-y"])
        # Workspace box (command-frame metres): the target is clamped into it so the arm can
        # never be driven into the straight-arm singularity / joint limits. MUST contain the
        # engage EE pose or engaging would jump the arm to the nearest face — the OpenArm
        # right-arm bent spawn EE sits near (0.19, -0.26, -0.28) in openarm_right_base_link, so
        # these bounds bracket that dexterous region and cap forward reach short of full
        # extension. Tune on hardware (live-tunable).
        self.declare_parameter("workspace_min", [-0.10, -0.60, -0.55])
        self.declare_parameter("workspace_max", [0.55, 0.30, 0.25])

        # tracking_active can flicker false for 1-2 frames near the frame edge / on fast motion;
        # treat tracking as still-good within this grace window so a blip does NOT re-index
        # (re-latching would jump the arm). Only a sustained loss / a disengage drops commanding.
        self.declare_parameter("tracking_grace", 0.30)     # s

        # Reserved: drive EE rotation from hand orientation. Off — the landmark orientation
        # estimate drifts; pose mirroring holds the engage orientation. Declared so the launch
        # arg keeps working; wiring an orientation source is future work.
        self.declare_parameter("enable_angular", False)

        # --- gripper (optional; needs a consumer e.g. gripper_teleop on SO-101) ---
        self.declare_parameter("enable_grip", True)
        self.declare_parameter("gripper_preset_topic", "/gripper_teleop/right/preset")
        self.declare_parameter("grip_open_below", 0.35)
        self.declare_parameter("grip_close_above", 0.65)

        # --- input topics ---
        self.declare_parameter("hand_pose_topic", "vision/hand_pose")
        self.declare_parameter("grip_topic", "vision/grip")
        self.declare_parameter("tracking_topic", "vision/tracking_active")
        self.declare_parameter("engage_topic", "vision/engage")

        gp = self.get_parameter
        self._frame = gp("command_frame").value
        self._ee_frame = gp("ee_frame").value
        self._timeout = float(gp("command_timeout").value)
        self._mirror_gain = float(gp("mirror_gain").value)
        self._axis_gain = list(gp("axis_gain").value)
        self._hand_span_m = float(gp("hand_span_m").value)
        self._fallback_scale = float(gp("fallback_scale").value)
        self._lin_map = mapping.parse_axis_map(gp("axis_map_linear").value)
        self._ws_min = list(gp("workspace_min").value)
        self._ws_max = list(gp("workspace_max").value)
        self._ws_box = (tuple(self._ws_min), tuple(self._ws_max))
        self._grace = float(gp("tracking_grace").value)
        self._enable_grip = bool(gp("enable_grip").value)
        self._grip_open_below = float(gp("grip_open_below").value)
        self._grip_close_above = float(gp("grip_close_above").value)

        # --- publishers (from the contract; never hard-coded types/topics) ---
        # PoseTracking subscribes target_pose with SystemDefaultsQoS (RELIABLE), so the default
        # contract qos (RELIABLE, depth 10) is compatible.
        self._target_pub = self.contract_publisher(
            "arm_pose_target", topic=gp("target_pose_topic").value
        )
        self._grip_pub = (
            self.contract_publisher("hand_preset", topic=gp("gripper_preset_topic").value)
            if self._enable_grip else None
        )

        # --- tf2: read the live EE pose (command_frame -> ee_frame) to anchor the mirror ---
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # --- live state ---
        self._engaged = False
        self._tracking = False
        self._pose = None           # (pos, quat) latest received hand pose
        self._pose_t = None         # rclpy Time of the latest pose
        self._curl = 0.0
        self._commanding = False    # currently mirroring (reference latched)?
        self._hand_ref = None       # hand position latched at engage
        self._w_m = None            # image width (m) at the hand, latched at engage
        self._ee_ref_pos = None     # EE position (command frame) latched at engage
        self._ee_ref_quat = None    # EE orientation (command frame) latched at engage; held
        self._last_tracking_t = None
        self._grip_state = "open"
        self._warned_tf = False

        # hand_tracker publishes the pose/grip/tracking stream BEST_EFFORT
        # (qos_profile_sensor_data); subscribe with the same so the QoS is compatible.
        self.create_subscription(PoseStamped, gp("hand_pose_topic").value, self._on_pose, qos_profile_sensor_data)
        self.create_subscription(Float64, gp("grip_topic").value, self._on_grip, qos_profile_sensor_data)
        self.create_subscription(Bool, gp("tracking_topic").value, self._on_tracking, qos_profile_sensor_data)
        # engage is a UI toggle — keep it RELIABLE so a press is never dropped.
        self.create_subscription(Bool, gp("engage_topic").value, self._on_engage, 10)

        period = 1.0 / max(float(gp("publish_rate").value), 1.0)
        self._timer = self.create_timer(period, self._tick)
        # Live tuning: `ros2 param set /vision_source mirror_gain 1.2`,
        # `... axis_gain "[1.0,1.0,1.0]"` (enable depth), `... axis_map_linear
        # "['z','-x','-y']"`, `... workspace_max "[0.6,...]"` — no relaunch. Persist
        # winners in fm_bringup/config/<robot>/vision.yaml.
        self.add_on_set_parameters_callback(self._on_params)
        self.get_logger().info(
            "vision_source up (pose mirroring; command_frame=%s ee_frame=%s)"
            % (self._frame, self._ee_frame)
        )

    def _on_params(self, params):
        for p in params:
            n = p.name
            if n == "mirror_gain":
                self._mirror_gain = float(p.value)
            elif n == "axis_gain":
                self._axis_gain = list(p.value)
            elif n == "hand_span_m":
                self._hand_span_m = float(p.value)
            elif n == "fallback_scale":
                self._fallback_scale = float(p.value)
            elif n == "axis_map_linear":
                self._lin_map = mapping.parse_axis_map(list(p.value))
            elif n == "workspace_min":
                self._ws_min = list(p.value)
                self._ws_box = (tuple(self._ws_min), tuple(self._ws_max))
            elif n == "workspace_max":
                self._ws_max = list(p.value)
                self._ws_box = (tuple(self._ws_min), tuple(self._ws_max))
            elif n == "tracking_grace":
                self._grace = float(p.value)
            elif n == "command_timeout":
                self._timeout = float(p.value)
        return SetParametersResult(successful=True)

    # --- subscriptions ---
    def _on_pose(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        self._pose = ((p.x, p.y, p.z), (o.w, o.x, o.y, o.z))
        self._pose_t = self.get_clock().now()

    def _on_grip(self, msg):
        self._curl = float(msg.data)

    def _on_tracking(self, msg):
        self._tracking = bool(msg.data)

    def _on_engage(self, msg):
        self._engaged = bool(msg.data)

    # --- helpers ---
    def _fresh(self):
        if self._pose is None or self._pose_t is None:
            return False
        age = (self.get_clock().now() - self._pose_t).nanoseconds * 1e-9
        return age <= self._timeout

    def _lookup_ee(self):
        """Latest EE pose ((x,y,z),(w,x,y,z)) in the command frame, or None if tf not ready.

        Non-blocking (Time() = latest available, no timeout) so it never stalls the timer.
        """
        try:
            tf = self._tf_buffer.lookup_transform(self._frame, self._ee_frame, Time())
        except tf2_ros.TransformException as exc:
            if not self._warned_tf:
                self.get_logger().warn(
                    "EE transform %s->%s not available yet (%s); waiting to engage."
                    % (self._frame, self._ee_frame, exc.__class__.__name__)
                )
                self._warned_tf = True
            return None
        self._warned_tf = False
        t = tf.transform.translation
        r = tf.transform.rotation
        return (t.x, t.y, t.z), (r.w, r.x, r.y, r.z)

    def _publish_target(self, pos):
        msg = PoseStamped()
        msg.header = self.stamped_header(self._frame)
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos
        qw, qx, qy, qz = self._ee_ref_quat
        msg.pose.orientation.w = qw
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        self._target_pub.publish(msg)

    # --- fixed-rate command loop ---
    def _tick(self):
        now = self.get_clock().now()
        if self._tracking:
            self._last_tracking_t = now
        # Debounce a tracking flicker: still "ok" for a grace window after the last true sample.
        tracking_ok = (
            self._last_tracking_t is not None
            and (now - self._last_tracking_t).nanoseconds * 1e-9 <= self._grace
        )
        can_command = self._engaged and tracking_ok and self._fresh()

        if not can_command:
            # Disengaged / tracking lost / stale -> STOP publishing targets. PoseTracking's
            # target_pose_timeout elapses and the node re-seeds a hold at the current EE, so
            # the arm holds in place (the "mouse pen-up" — no command, no drift).
            self._commanding = False
            return

        hand_now = self._pose[0]

        if not self._commanding:
            # Rising edge into mirroring: latch the references (hand + EE) AND the metric
            # conversion W_m (image width in metres at the hand, from the apparent hand
            # size) so the mapping is indexed from here with zero offset (no startup jump)
            # and a stable amplitude for the whole engagement. Skip if tf isn't ready.
            ee = self._lookup_ee()
            if ee is None:
                return
            self._ee_ref_pos, self._ee_ref_quat = ee
            self._hand_ref = hand_now
            self._w_m = mapping.image_width_m(hand_now[2], hand_span_m=self._hand_span_m)
            self._commanding = True
            if self._w_m is None:
                self.get_logger().warn(
                    "engaged; ee_ref=(%.3f, %.3f, %.3f) but hand size degenerate (z=%.4f)"
                    " -> fallback_scale=%.2f m per image width"
                    % (self._ee_ref_pos + (hand_now[2], self._fallback_scale))
                )
            else:
                self.get_logger().info(
                    "engaged; ee_ref=(%.3f, %.3f, %.3f); image width ~= %.2f m at hand"
                    " (z=%.3f) -> %.2f m EE per image width"
                    % (self._ee_ref_pos + (self._w_m, hand_now[2], self._mirror_gain * self._w_m))
                )
            self._publish_target(self._ee_ref_pos)   # zero-delta target -> hold, no jump
            return

        scale = mapping.metric_scale(
            self._w_m, mirror_gain=self._mirror_gain,
            axis_gain=self._axis_gain, fallback_scale=self._fallback_scale,
        )
        target, overflow = mapping.metric_mirror_target(
            self._ee_ref_pos, self._hand_ref, hand_now, self._lin_map,
            scale=scale, hand_span_m=self._hand_span_m, workspace_box=self._ws_box,
        )
        if any(abs(o) > 1e-9 for o in overflow):
            # The box silently eating motion reads as "the arm barely moves" — say so.
            self.get_logger().warn(
                "workspace box clamping target: " + ", ".join(
                    "%s %+.3f m past %s"
                    % ("xyz"[i], overflow[i], "max" if overflow[i] > 0 else "min")
                    for i in range(3) if abs(overflow[i]) > 1e-9
                ),
                throttle_duration_sec=1.0,
            )
        self._publish_target(target)

        if self._grip_pub is not None:
            state = mapping.grip_preset(
                self._curl, self._grip_state,
                open_below=self._grip_open_below, close_above=self._grip_close_above,
            )
            if state != self._grip_state:
                self._grip_state = state
                self._grip_pub.publish(String(data=state))


def main(args=None):
    rclpy.init(args=args)
    node = MirrorSource()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
