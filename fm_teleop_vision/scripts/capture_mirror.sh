#!/usr/bin/env bash
# capture_mirror.sh — launch the mirror-teleop capture logger inside the fm container.
#
# The logger (mirror_datalogger.py) records BOTH a synchronized hand->target->actual
# CSV/JSONL and a raw ros2 bag of every pipeline topic, per session, into a timestamped
# dir under /ws/data/teleop_captures/.
#
#   Default: PERSISTENT Foxglove-button mode — leave it running; start/stop a session by
#   publishing std_msgs/Bool on /capture/record (a Foxglove Publish/Button panel). Status
#   is echoed on /capture/status (String, latched).
#
#   One-shot CLI: pass --auto-start to record a single session until Ctrl-C.
#
# Any extra args pass straight through to mirror_datalogger.py (e.g. --no-bag, --rate 100,
# --session-name reachtest, --control-topic /my/topic).
#
#   docker compose ... exec fm bash /ws/src/fm-teleop/fm_teleop_vision/scripts/capture_mirror.sh [args]
set -euo pipefail

# ROS setup scripts reference unbound vars; source them with nounset off.
set +u
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

exec python3 /ws/src/fm-teleop/fm_teleop_vision/scripts/mirror_datalogger.py "$@"
