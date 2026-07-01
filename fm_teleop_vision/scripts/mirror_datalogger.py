#!/usr/bin/env python3
"""mirror_datalogger — session-controlled synchronized capture of the mirror teleop.

Records, at a fixed rate, one time-aligned row joining the three points of the
hand-mirroring chain so you can debug *how a hand motion became a robot motion*:

    HAND      /vision/hand_pose        (PoseStamped, camera frame, image-width units)
      |  mirror_source: target = clamp( ee_ref + remap(scale*(hand_now-hand_ref)) )
    COMMAND   /target_pose             (PoseStamped, command/base frame)
      |  pose_tracking_node (MoveIt Servo) drives the joints
    ACTUAL    tf base_link -> link7    (the EE the robot actually reached)

plus engage/tracking/grip/servo-status/joint-state and, when mirror_source is
instrumented (publish_debug:=true), the internal latched refs + W_m + pre-clamp
target + workspace overflow on /vision/mirror_debug (Float64MultiArray).

TWO WAYS TO DRIVE IT:
  * Foxglove button: leave it running; it starts/stops a session when you publish
    std_msgs/Bool on --control-topic (default /capture/record). Status is echoed on
    /capture/status (String, latched) for an Indicator/Raw-message panel.
  * CLI one-shot: pass --auto-start; it records one session until Ctrl-C/SIGTERM.

Each session -> a timestamped dir under --base-dir containing:
    mirror_log.csv    one flat row per tick (FIXED columns; safe for pandas)
    mirror_log.jsonl  one nested JSON object per tick (full fidelity)
    bag/              a ros2 bag of every pipeline topic (unless --no-bag)
    meta.json         params snapshot, topic health counts, frames, rate, duration

Subscribes to the DATA topics BEST_EFFORT (a best-effort subscriber is QoS-
compatible with both best-effort and reliable publishers, so we never silently get
nothing); the CONTROL topic is RELIABLE so a button press is never dropped.

/vision/mirror_debug Float64MultiArray layout (label "mirror_debug_v1") — index i
maps to MIRROR_DEBUG_FIELDS[i] below:
    0 commanding   5 ee_ref_x   10 scale_z    15 preclamp_y  20 overflow_x
    1 w_m          6 ee_ref_y   11 moved_x    16 preclamp_z  21 overflow_y
    2 hand_ref_x   7 ee_ref_z   12 moved_y    17 target_x    22 overflow_z
    3 hand_ref_y   8 scale_x    13 moved_z    18 target_y
    4 hand_ref_z   9 scale_y    14 preclamp_x 19 target_z
"""

from __future__ import annotations

import argparse
import csv
import datetime
import json
import math
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, Int8, String
import tf2_ros

# Field order of /vision/mirror_debug — kept in lockstep with mirror_source.py's
# _publish_debug_msg. Index i in the array == this name (23 fields).
MIRROR_DEBUG_FIELDS = [
    "commanding", "w_m",
    "hand_ref_x", "hand_ref_y", "hand_ref_z",
    "ee_ref_x", "ee_ref_y", "ee_ref_z",
    "scale_x", "scale_y", "scale_z",
    "moved_x", "moved_y", "moved_z",
    "preclamp_x", "preclamp_y", "preclamp_z",
    "target_x", "target_y", "target_z",
    "overflow_x", "overflow_y", "overflow_z",
]

# FIXED csv schema — written in full at session start so the header always covers
# every column (dbg_* and joints included even before their first sample arrives),
# avoiding ragged rows when instrumentation / joints start mid-session.
BASE_FIELDS = [
    "t_wall", "t_ros", "engage", "tracking", "commanding_est",
    "hand_x", "hand_y", "hand_z", "hand_qw", "hand_qx", "hand_qy", "hand_qz", "hand_age",
    "hand_ref_x", "hand_ref_y", "hand_ref_z", "hand_dx", "hand_dy", "hand_dz",
    "tgt_x", "tgt_y", "tgt_z", "tgt_qw", "tgt_qx", "tgt_qy", "tgt_qz", "tgt_age",
    "ee_x", "ee_y", "ee_z", "ee_qw", "ee_qx", "ee_qy", "ee_qz",
    "ee_ref_x", "ee_ref_y", "ee_ref_z",
    "moved_x", "moved_y", "moved_z",
    "err_x", "err_y", "err_z", "err_dist",
    "grip", "servo_status",
]
N_ARM_JOINTS = 7
JOINT_FIELDS = ["j%d" % i for i in range(1, N_ARM_JOINTS + 1)]
DBG_FIELDS = ["dbg_" + f for f in MIRROR_DEBUG_FIELDS]
ALL_FIELDS = BASE_FIELDS + JOINT_FIELDS + DBG_FIELDS

# Pipeline topics to put in the ros2 bag (intersected with what exists at start).
BAG_TOPICS = [
    "/vision/hand_pose", "/vision/tracking_active", "/vision/grip", "/vision/engage",
    "/vision/image", "/vision/mirror_debug", "/target_pose",
    "/tf", "/tf_static", "/joint_states", "/dynamic_joint_states",
    "/openarm_right_arm_controller/joint_trajectory",
    "/openarm_right_arm_controller/controller_state",
    "/openarm_right_forward_position_controller/commands",
    "/pose_tracking_node/status", "/pose_tracking_node/delta_twist_cmds",
    "/pose_tracking_node/delta_joint_cmds", "/gripper_teleop/right/preset", "/clock",
]


def _best_effort_qos(depth: int = 50) -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST,
                      durability=DurabilityPolicy.VOLATILE, depth=depth)


def _latched_qos() -> QoSProfile:
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)


class MirrorDataLogger(Node):
    def __init__(self, base_dir, rate, command_frame, ee_frame, hand_span_m,
                 arm_joint_prefix, control_topic, record_bag, session_name):
        super().__init__("mirror_datalogger")
        self._base_dir = base_dir
        self._command_frame = command_frame
        self._ee_frame = ee_frame
        self._hand_span_m = hand_span_m
        self._arm_joint_prefix = arm_joint_prefix
        self._record_bag = record_bag
        self._session_name = session_name

        # --- latest cached values (updated by callbacks; sampled by the timer) ---
        self._hand = None
        self._hand_t = None
        self._target = None
        self._target_t = None
        self._engage = False
        self._tracking = False
        self._last_tracking_t = None
        self._grip = 0.0
        self._servo_status = None
        self._joints = {}
        self._dbg = None

        # --- session state ---
        self._active = False
        self._sess_dir = None
        self._csv_f = None
        self._jsonl_f = None
        self._writer = None
        self._bag_proc = None
        self._rows = 0
        self._t_start = None
        self._counts = {}
        self._tf_fail = 0

        # --- engage-latched reference estimate (external proxy) ---
        self._commanding_est = False
        self._hand_ref = None
        self._ee_ref = None

        self._grace = 0.30
        self._hand_timeout = 0.20

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        qos = _best_effort_qos()
        self.create_subscription(PoseStamped, "/vision/hand_pose", self._on_hand, qos)
        self.create_subscription(PoseStamped, "/target_pose", self._on_target, qos)
        self.create_subscription(Bool, "/vision/engage", self._on_engage, qos)
        self.create_subscription(Bool, "/vision/tracking_active", self._on_tracking, qos)
        self.create_subscription(Float64, "/vision/grip", self._on_grip, qos)
        self.create_subscription(Int8, "/pose_tracking_node/status", self._on_status, qos)
        self.create_subscription(JointState, "/joint_states", self._on_joints, qos)
        self.create_subscription(Float64MultiArray, "/vision/mirror_debug", self._on_dbg, qos)

        # Status publisher (latched so a late-joining Foxglove panel sees it).
        self._status_pub = self.create_publisher(String, "/capture/status", _latched_qos())

        # Control topic: RELIABLE so a button press is never dropped.
        if control_topic:
            self.create_subscription(
                Bool, control_topic, self._on_control,
                QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST, depth=10))
            self.get_logger().info("listening for record toggles on %s (std_msgs/Bool)"
                                   % control_topic)

        self._rate = rate
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self._set_status("idle")
        self.get_logger().info(
            "mirror_datalogger ready: %.0f Hz, base=%s, bag=%s (frames %s->%s)"
            % (rate, base_dir, record_bag, command_frame, ee_frame))

    # --- callbacks -------------------------------------------------------------
    def _on_hand(self, m):
        p, o = m.pose.position, m.pose.orientation
        self._hand = (p.x, p.y, p.z, o.w, o.x, o.y, o.z)
        self._hand_t = self.get_clock().now()
        self._bump("hand_pose")

    def _on_target(self, m):
        p, o = m.pose.position, m.pose.orientation
        self._target = (p.x, p.y, p.z, o.w, o.x, o.y, o.z)
        self._target_t = self.get_clock().now()
        self._bump("target_pose")

    def _on_engage(self, m):
        self._engage = bool(m.data)
        self._bump("engage")

    def _on_tracking(self, m):
        self._tracking = bool(m.data)
        if m.data:
            self._last_tracking_t = self.get_clock().now()
        self._bump("tracking")

    def _on_grip(self, m):
        self._grip = float(m.data)
        self._bump("grip")

    def _on_status(self, m):
        self._servo_status = int(m.data)
        self._bump("status")

    def _on_joints(self, m):
        for name, pos in zip(m.name, m.position):
            self._joints[name] = pos
        self._bump("joint_states")

    def _on_dbg(self, m):
        vals = list(m.data)
        if len(vals) >= len(MIRROR_DEBUG_FIELDS):
            self._dbg = {MIRROR_DEBUG_FIELDS[i]: vals[i]
                         for i in range(len(MIRROR_DEBUG_FIELDS))}
            self._bump("mirror_debug")

    def _on_control(self, m):
        if m.data and not self._active:
            self.start_session()
        elif not m.data and self._active:
            self.stop_session()

    def _bump(self, key):
        if self._active:
            self._counts[key] = self._counts.get(key, 0) + 1

    # --- helpers ---------------------------------------------------------------
    def _age(self, t):
        return None if t is None else (self.get_clock().now() - t).nanoseconds * 1e-9

    def _lookup_ee(self):
        try:
            tf = self._tf_buffer.lookup_transform(self._command_frame, self._ee_frame, Time())
        except tf2_ros.TransformException:
            if self._active:
                self._tf_fail += 1
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, t.z, r.w, r.x, r.y, r.z)

    def _arm_joint_names(self):
        return sorted(n for n in self._joints if n.startswith(self._arm_joint_prefix))

    def _set_status(self, text):
        self.get_logger().info("[capture] %s" % text)
        self._status_pub.publish(String(data=text))

    def _existing_bag_topics(self):
        have = {n for n, _ in self.get_topic_names_and_types()}
        return [t for t in BAG_TOPICS if t in have]

    # --- session lifecycle -----------------------------------------------------
    def start_session(self, name=None):
        if self._active:
            return
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._sess_dir = os.path.join(self._base_dir, "%s_%s" % (ts, name or self._session_name))
        os.makedirs(self._sess_dir, exist_ok=True)
        # reset per-session state
        self._rows = 0
        self._tf_fail = 0
        self._counts = {}
        self._commanding_est = False
        self._hand_ref = None
        self._ee_ref = None
        self._t_start = time.time()
        # open outputs with the FULL fixed schema
        self._csv_f = open(os.path.join(self._sess_dir, "mirror_log.csv"), "w", newline="")
        self._writer = csv.DictWriter(self._csv_f, fieldnames=ALL_FIELDS, extrasaction="ignore")
        self._writer.writeheader()
        self._jsonl_f = open(os.path.join(self._sess_dir, "mirror_log.jsonl"), "w")
        # best-effort params snapshot
        try:
            with open(os.path.join(self._sess_dir, "params_mirror_source.yaml"), "w") as f:
                subprocess.run(["ros2", "param", "dump", "/mirror_source"],
                               stdout=f, stderr=subprocess.DEVNULL, timeout=8)
        except Exception:
            pass
        # rosbag
        if self._record_bag:
            topics = self._existing_bag_topics()
            try:
                self._bag_proc = subprocess.Popen(
                    ["ros2", "bag", "record", "-o", os.path.join(self._sess_dir, "bag")] + topics,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    preexec_fn=os.setsid)
            except Exception as exc:
                self._bag_proc = None
                self.get_logger().warn("could not start ros2 bag: %s" % exc)
        self._active = True
        self._set_status("recording -> %s" % os.path.basename(self._sess_dir))

    def stop_session(self):
        if not self._active:
            return
        self._active = False
        dur = time.time() - self._t_start
        # stop bag first so it flushes while we finalize files
        if self._bag_proc is not None:
            try:
                os.killpg(os.getpgid(self._bag_proc.pid), signal.SIGINT)
                self._bag_proc.wait(timeout=15)
            except Exception:
                try:
                    self._bag_proc.kill()
                except Exception:
                    pass
            self._bag_proc = None
        meta = {
            "session": os.path.basename(self._sess_dir),
            "rate_hz": self._rate, "duration_s": round(dur, 2), "rows": self._rows,
            "command_frame": self._command_frame, "ee_frame": self._ee_frame,
            "hand_span_m": self._hand_span_m, "tf_lookup_failures": self._tf_fail,
            "instrumented": self._counts.get("mirror_debug", 0) > 0,
            "recorded_bag": self._record_bag,
            "msg_counts": self._counts,
            "msg_rates_hz": {k: round(v / dur, 1) for k, v in self._counts.items()} if dur > 0 else {},
        }
        with open(os.path.join(self._sess_dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
        for fh in (self._csv_f, self._jsonl_f):
            try:
                fh.flush()
                fh.close()
            except Exception:
                pass
        self._csv_f = self._jsonl_f = self._writer = None
        self._set_status("idle (last: %s, %d rows, %.1fs%s)"
                         % (os.path.basename(self._sess_dir), self._rows, dur,
                            "" if meta["instrumented"] else ", NOT instrumented"))

    # --- fixed-rate sampling ---------------------------------------------------
    def _tick(self):
        if not self._active:
            return
        now = self.get_clock().now()
        hand_age = self._age(self._hand_t)
        tgt_age = self._age(self._target_t)
        track_age = self._age(self._last_tracking_t)
        tracking_ok = track_age is not None and track_age <= self._grace
        hand_fresh = hand_age is not None and hand_age <= self._hand_timeout
        commanding = self._engage and tracking_ok and hand_fresh

        ee = self._lookup_ee()
        if commanding:
            # Rising-edge latch, RETRIED each tick until BOTH refs are captured: tf
            # may not be ready on the first commanding tick, so don't mark ourselves
            # latched (and stop retrying) until ee_ref actually exists — mirrors how
            # mirror_source waits for tf before latching ee_ref.
            if self._hand_ref is None and self._hand is not None:
                self._hand_ref = self._hand[:3]
            if self._ee_ref is None and ee is not None:
                self._ee_ref = ee[:3]
        else:
            # Disengaged: drop the references so the next engagement re-latches, and
            # clear the last debug frame so idle rows don't carry stale dbg_* values.
            self._hand_ref = None
            self._ee_ref = None
            self._dbg = None
        self._commanding_est = commanding

        hand, tgt = self._hand, self._target
        hand_d = (tuple(hand[i] - self._hand_ref[i] for i in range(3))
                  if hand and self._hand_ref else (None, None, None))
        moved = (tuple(tgt[i] - self._ee_ref[i] for i in range(3))
                 if tgt and self._ee_ref else (None, None, None))
        err = ((tgt[0] - ee[0], tgt[1] - ee[1], tgt[2] - ee[2])
               if tgt and ee else (None, None, None))
        err_dist = (math.sqrt(sum(e * e for e in err))
                    if all(e is not None for e in err) else None)

        arm_names = self._arm_joint_names()
        joints = {n: self._joints[n] for n in arm_names}

        row = {
            "t_wall": round(time.time(), 4), "t_ros": round(now.nanoseconds * 1e-9, 4),
            "engage": int(self._engage), "tracking": int(self._tracking),
            "commanding_est": int(commanding),
            "hand_x": _g(hand, 0), "hand_y": _g(hand, 1), "hand_z": _g(hand, 2),
            "hand_qw": _g(hand, 3), "hand_qx": _g(hand, 4), "hand_qy": _g(hand, 5), "hand_qz": _g(hand, 6),
            "hand_age": _r(hand_age),
            "hand_ref_x": _g(self._hand_ref, 0), "hand_ref_y": _g(self._hand_ref, 1), "hand_ref_z": _g(self._hand_ref, 2),
            "hand_dx": _r(hand_d[0]), "hand_dy": _r(hand_d[1]), "hand_dz": _r(hand_d[2]),
            "tgt_x": _g(tgt, 0), "tgt_y": _g(tgt, 1), "tgt_z": _g(tgt, 2),
            "tgt_qw": _g(tgt, 3), "tgt_qx": _g(tgt, 4), "tgt_qy": _g(tgt, 5), "tgt_qz": _g(tgt, 6),
            "tgt_age": _r(tgt_age),
            "ee_x": _g(ee, 0), "ee_y": _g(ee, 1), "ee_z": _g(ee, 2),
            "ee_qw": _g(ee, 3), "ee_qx": _g(ee, 4), "ee_qy": _g(ee, 5), "ee_qz": _g(ee, 6),
            "ee_ref_x": _g(self._ee_ref, 0), "ee_ref_y": _g(self._ee_ref, 1), "ee_ref_z": _g(self._ee_ref, 2),
            "moved_x": _r(moved[0]), "moved_y": _r(moved[1]), "moved_z": _r(moved[2]),
            "err_x": _r(err[0]), "err_y": _r(err[1]), "err_z": _r(err[2]), "err_dist": _r(err_dist),
            "grip": round(self._grip, 4),
            "servo_status": self._servo_status if self._servo_status is not None else "",
        }
        for i, n in enumerate(arm_names[:N_ARM_JOINTS], start=1):
            row["j%d" % i] = round(joints[n], 5)
        if self._dbg is not None:
            for k in MIRROR_DEBUG_FIELDS:
                row["dbg_" + k] = round(self._dbg[k], 5)

        self._writer.writerow(row)
        jrow = dict(row)
        jrow["joints"] = joints
        self._jsonl_f.write(json.dumps(jrow) + "\n")
        self._rows += 1
        if self._rows % 100 == 0:
            self._csv_f.flush()
            self._jsonl_f.flush()

    def shutdown(self):
        if self._active:
            self.stop_session()


def _g(tpl, i):
    return round(tpl[i], 5) if tpl is not None and tpl[i] is not None else ""


def _r(v):
    return round(v, 5) if v is not None else ""


def main():
    ap = argparse.ArgumentParser(description="Session-controlled vision-mirror teleop logger.")
    ap.add_argument("--base-dir", default="/ws/data/teleop_captures")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--control-topic", default="/capture/record",
                    help='std_msgs/Bool toggle (Foxglove button); "" to disable')
    ap.add_argument("--auto-start", action="store_true",
                    help="record one session immediately until Ctrl-C (CLI mode)")
    ap.add_argument("--no-bag", action="store_true", help="do not record a ros2 bag")
    ap.add_argument("--session-name", default="session")
    ap.add_argument("--command-frame", default="openarm_right_base_link")
    ap.add_argument("--ee-frame", default="openarm_right_link7")
    ap.add_argument("--hand-span-m", type=float, default=0.09)
    ap.add_argument("--arm-joint-prefix", default="openarm_right_joint")
    args = ap.parse_args()

    rclpy.init()
    node = MirrorDataLogger(
        args.base_dir, args.rate, args.command_frame, args.ee_frame,
        args.hand_span_m, args.arm_joint_prefix,
        None if args.control_topic == "" else args.control_topic,
        not args.no_bag, args.session_name)
    if args.auto_start:
        node.start_session()

    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))
    try:
        while rclpy.ok() and not stop["flag"]:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
