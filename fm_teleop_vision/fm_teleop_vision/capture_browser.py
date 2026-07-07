#!/usr/bin/env python3
"""capture_browser — serve recorded teleop sessions to the web GUI over the WS bridge.

The GUI is a browser: it cannot read the container filesystem, only the Foxglove bridge
topics. This node bridges that gap for the recordings viewer (mirror_datalogger stays the
sole recorder; this only reads):

    scan --base-dir                       -> /capture/index  (String JSON, latched)  list + light stats
    GUI publishes a session id            -> /capture/select (String)                request one session
    load that session's mirror_log.csv    -> /capture/detail (String JSON, latched)  full stats + downsampled series

Stats come from the pure ``capture_stats`` module (unit-tested without ROS). The index is
rebuilt at startup and whenever ``/capture/status`` reports a finished recording, so a new
capture shows up without reopening the GUI. Per-session light stats are cached to
``summary.json`` in the session dir so rescans stay cheap.
"""

import json
import os

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from std_msgs.msg import String

from fm_teleop_vision import capture_stats


def _latched():
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)


def _reliable():
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST, depth=10)


class CaptureBrowser(Node):
    def __init__(self):
        super().__init__("capture_browser")
        self.declare_parameter("base_dir", "/ws/data/teleop_captures")
        self.declare_parameter("index_topic", "/capture/index")
        self.declare_parameter("select_topic", "/capture/select")
        self.declare_parameter("detail_topic", "/capture/detail")
        self.declare_parameter("status_topic", "/capture/status")
        gp = self.get_parameter
        self._base = gp("base_dir").value

        self._index_pub = self.create_publisher(String, gp("index_topic").value, _latched())
        self._detail_pub = self.create_publisher(String, gp("detail_topic").value, _latched())
        self.create_subscription(String, gp("select_topic").value, self._on_select, _reliable())
        # status is latched by the recorder; refresh the index when a recording finishes.
        self._last_status = None
        self.create_subscription(String, gp("status_topic").value, self._on_status, _latched())

        self._publish_index()
        self.get_logger().info("capture_browser ready: serving %s" % self._base)

    # --- index ---------------------------------------------------------------
    def _session_dirs(self):
        if not os.path.isdir(self._base):
            return []
        out = []
        for name in os.listdir(self._base):
            d = os.path.join(self._base, name)
            if os.path.isdir(d) and os.path.exists(os.path.join(d, "meta.json")):
                out.append(d)
        return sorted(out, key=os.path.basename, reverse=True)  # newest first

    def _read_meta(self, d):
        try:
            with open(os.path.join(d, "meta.json")) as f:
                return json.load(f)
        except Exception:
            return {}

    def _light(self, d):
        """Light summary for the index list, cached to summary.json."""
        cache = os.path.join(d, "summary.json")
        csv_path = os.path.join(d, "mirror_log.csv")
        try:
            if os.path.exists(cache) and os.path.getmtime(cache) >= os.path.getmtime(csv_path):
                with open(cache) as f:
                    return json.load(f)
        except Exception:
            pass
        meta = self._read_meta(d)
        try:
            rows = capture_stats.read_csv(csv_path)
        except Exception:
            rows = []
        light = capture_stats.light_summary(rows, meta)
        light["session"] = os.path.basename(d)
        try:
            with open(cache, "w") as f:
                json.dump(light, f)
        except Exception:
            pass
        return light

    def _publish_index(self):
        sessions = []
        for d in self._session_dirs():
            try:
                sessions.append(self._light(d))
            except Exception as exc:
                self.get_logger().warn("index: skipping %s (%s)" % (d, exc))
        self._index_pub.publish(String(data=json.dumps({"sessions": sessions})))
        self.get_logger().info("published /capture/index: %d sessions" % len(sessions))

    def _on_status(self, msg):
        text = msg.data or ""
        # rebuild when a recording just completed (status went to "idle ...").
        if text.startswith("idle") and text != self._last_status:
            self._publish_index()
        self._last_status = text

    # --- detail --------------------------------------------------------------
    def _ws_box(self, d):
        """Read the workspace clamp box from the session's params snapshot, else defaults."""
        try:
            with open(os.path.join(d, "params_mirror_source.yaml")) as f:
                p = yaml.safe_load(f)["/mirror_source"]["ros__parameters"]
            return list(p["workspace_min"]), list(p["workspace_max"])
        except Exception:
            return capture_stats.DEFAULT_WS_MIN, capture_stats.DEFAULT_WS_MAX

    def _on_select(self, msg):
        sid = (msg.data or "").strip()
        d = os.path.join(self._base, sid)
        if not sid or not os.path.isdir(d):
            self.get_logger().warn("select: unknown session %r" % sid)
            self._detail_pub.publish(String(data=json.dumps({"session": sid, "error": "not found"})))
            return
        try:
            rows = capture_stats.read_csv(os.path.join(d, "mirror_log.csv"))
        except Exception as exc:
            self._detail_pub.publish(String(data=json.dumps({"session": sid, "error": str(exc)})))
            return
        meta = self._read_meta(d)
        ws_min, ws_max = self._ws_box(d)
        detail = {
            "session": sid,
            "summary": capture_stats.summarize(rows, meta, ws_min, ws_max),
            "series": capture_stats.series(rows),
        }
        self._detail_pub.publish(String(data=json.dumps(detail)))
        self.get_logger().info("published /capture/detail for %s (%d rows)" % (sid, len(rows)))


def main():
    rclpy.init()
    node = CaptureBrowser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
