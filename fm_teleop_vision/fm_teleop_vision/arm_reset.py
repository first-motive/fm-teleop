#!/usr/bin/env python3
"""arm_reset — disengage the mirror teleop and drive the arm back to its home pose.

The RESET button publishes ``std_msgs/Bool(data=true)`` on ``/vision/reset``. On the rising
edge this node performs a two-step "start over":

  1. Publish ``Bool(false)`` on ``/vision/engage`` so ``mirror_source`` disengages and stops
     streaming ``/target_pose``.
  2. After a short ``grace_s``, stream ``/target_pose`` = the arm's HOME end-effector pose
     for ``duration_s`` so MoveIt Servo drives the EE home; then stop (Servo times out and
     holds home). Re-engaging afterwards re-latches the mirror reference at the home pose.

WHY drive home through ``/target_pose`` and not a joint_trajectory: MoveIt Servo owns the
arm controller and publishes a hold command continuously (~30 Hz) whenever it is running,
even while disengaged. A one-shot (or repeated) joint_trajectory to the controller is
therefore drowned by Servo's stream and never executes — verified live. The reliable way to
move the arm is to feed Servo its own input: a fresh, correctly-STAMPED ``/target_pose``.
Servo servos the EE there, reproducing the home joint config (validated to <0.01 rad).

``home_ee_*`` is the forward-kinematics of the URDF spawn "ready" joints
([0, 0.5, 0, 1.2, 0, 0.3, 0]) in ``command_frame`` — a bent, dexterous pose off the
straight-arm singularity. Defaults target the OpenArm right arm; override per robot if the
spawn pose changes. Do NOT click ENGAGE during the ~``duration_s`` home move — a second
``/target_pose`` writer would fight the home stream.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

# RELIABLE + KEEP_LAST(10): matches mirror_source's /vision/engage subscription and
# PoseTracking's /target_pose subscription so a button click and the home stream both land.
_RELIABLE = QoSProfile(
    depth=10, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE
)


class ArmReset(Node):
    """Turn a /vision/reset pulse into a disengage + a servo-driven return to the home pose."""

    def __init__(self):
        super().__init__("arm_reset")
        self.declare_parameter("reset_topic", "/vision/reset")
        self.declare_parameter("engage_topic", "/vision/engage")
        self.declare_parameter("target_pose_topic", "/target_pose")
        # command_frame must equal the frame mirror_source stamps /target_pose in (the Servo
        # planning frame); vision_session.launch.py sources it from servo.yaml so they agree.
        self.declare_parameter("command_frame", "openarm_right_base_link")
        # HOME EE pose in command_frame = FK of the URDF spawn "ready" joints, measured from tf.
        self.declare_parameter("home_ee_position", [0.201, -0.265, -0.262])       # x, y, z (m)
        self.declare_parameter("home_ee_orientation", [-0.223, -0.421, 0.108, 0.872])  # x,y,z,w
        self.declare_parameter("publish_rate", 50.0)   # Hz to stream the home target
        # grace: let mirror_source process the disengage (stop its /target_pose) before we drive.
        self.declare_parameter("grace_s", 0.15)
        # how long to stream the home target — long enough for Servo to converge from any pose.
        self.declare_parameter("duration_s", 4.5)

        gp = self.get_parameter
        self._engage_topic = gp("engage_topic").value
        self._frame = gp("command_frame").value
        self._home_pos = [float(x) for x in gp("home_ee_position").value]
        self._home_quat = [float(x) for x in gp("home_ee_orientation").value]
        self._rate = float(gp("publish_rate").value)
        self._grace_s = float(gp("grace_s").value)
        self._duration_s = float(gp("duration_s").value)

        if len(self._home_pos) != 3 or len(self._home_quat) != 4:
            raise RuntimeError(
                "arm_reset: home_ee_position must be [x,y,z] and home_ee_orientation [x,y,z,w]"
            )

        self._engage_pub = self.create_publisher(Bool, self._engage_topic, _RELIABLE)
        self._target_pub = self.create_publisher(
            PoseStamped, gp("target_pose_topic").value, _RELIABLE
        )
        self.create_subscription(
            Bool, gp("reset_topic").value, self._on_reset, _RELIABLE
        )

        # Timers, live only during a reset: grace (one-shot) then drive (repeating, self-stopping).
        self._grace_timer = None
        self._drive_timer = None
        self._drive_end = None

        self.get_logger().info(
            "arm_reset ready: %s -> disengage(%s) + servo /target_pose to home %s (frame %s) over %.1fs"
            % (
                gp("reset_topic").value, self._engage_topic, self._home_pos,
                self._frame, self._duration_s,
            )
        )

    def _on_reset(self, msg):
        if not msg.data:
            return
        if self._grace_timer is not None or self._drive_timer is not None:
            self.get_logger().warn("reset already in flight; ignoring")
            return
        self.get_logger().info(
            "reset requested -> disengaging, homing after %.2fs" % self._grace_s
        )
        self._engage_pub.publish(Bool(data=False))
        self._grace_timer = self.create_timer(self._grace_s, self._start_drive)

    def _start_drive(self):
        # Grace elapsed: tear the one-shot grace timer down and start streaming the home target.
        if self._grace_timer is not None:
            self._grace_timer.cancel()
            self.destroy_timer(self._grace_timer)
            self._grace_timer = None
        self._drive_end = self.get_clock().now() + Duration(seconds=self._duration_s)
        self._drive_timer = self.create_timer(1.0 / max(self._rate, 1.0), self._drive)
        self.get_logger().info(
            "homing: streaming /target_pose = %s for %.1fs" % (self._home_pos, self._duration_s)
        )

    def _drive(self):
        now = self.get_clock().now()
        if now >= self._drive_end:
            self._drive_timer.cancel()
            self.destroy_timer(self._drive_timer)
            self._drive_timer = None
            self.get_logger().info("home reached; releasing /target_pose (Servo holds).")
            return
        msg = PoseStamped()
        msg.header.frame_id = self._frame
        msg.header.stamp = now.to_msg()  # MUST be fresh — PoseTracking drops stale targets
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self._home_pos
        qx, qy, qz, qw = self._home_quat
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._target_pub.publish(msg)


def main():
    rclpy.init()
    node = ArmReset()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
