#!/usr/bin/env bash
# Standalone front door for fm-teleop. Builds the workspace and launches one
# teleop input node on its own — the gamepad -> MoveIt Servo adapter (joy_to_servo).
#
# This launches a SINGLE teleop input node standalone: it reads /joy and publishes
# Servo twist commands. It does NOT bring up the full Servo teleop pipeline — that
# needs MoveIt Servo plus a sim/robot target, which run through the fm-app launcher
# (`ros2 launch fm_bringup teleop.launch.py`), not from this repo. fm-teleop carries
# no launch files of its own, so standalone scope ends at the input node.
#
# The host OS picks the path (override with --native / --container):
#   Linux  -> native:    build + launch directly on the host (ROS2 Humble installed)
#   Darwin -> container: build the fm-teleop image, bring it up via the fm-docker
#                        compose overlays, build + launch inside it (OrbStack)
#
#   ./run.sh                       # auto-detect, run joy_to_servo
#   ./run.sh --native              # force the host path (Linux)
#   ./run.sh --container           # force the container path (macOS / OrbStack)
#   ./run.sh --ros-args -p deadzone:=0.2   # extra args pass through to ros2 run
set -euo pipefail

cd "$(dirname "$0")"

# --- Per-repo config (downstream repos retune these two) ----------------------
IMAGE=fm-teleop:humble                                 # local image tag for the container path
LAUNCH=(ros2 run fm_teleop_device joy_to_servo)        # what `run.sh` launches
# -----------------------------------------------------------------------------

MODE=""                  # "" = auto-detect; else native | container
PASSTHROUGH=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --native)    MODE=native; shift ;;
    --container) MODE=container; shift ;;
    *)           PASSTHROUGH+=("$1"); shift ;;
  esac
done

# Auto-detect the path from the host OS when not forced by a flag.
if [[ -z "$MODE" ]]; then
  case "$(uname -s)" in
    Linux)  MODE=native ;;
    Darwin) MODE=container ;;
    *) echo "error: unsupported host '$(uname -s)' — pass --native or --container" >&2; exit 1 ;;
  esac
fi

LAUNCH+=(${PASSTHROUGH[@]+"${PASSTHROUGH[@]}"})

if [[ "$MODE" == native ]]; then
  # Host path: build in place, launch on the host. No git externals to import —
  # Python/npm deps resolve via rosdep + npm (fm-teleop.repos pulls only docker/).
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  rosdep install --from-paths . --ignore-src -y -r 2>/dev/null || true
  colcon build --symlink-install
  source install/setup.bash
  echo ">> launching joy_to_servo on the host — publishes Servo twist from /joy"
  exec "${LAUNCH[@]}"
fi

# Container path: build the local image, bring it up, build + launch inside it.
# The fm-docker compose overlays live in docker/, imported via fm-teleop.repos —
# pull them on first run so a fresh clone works with no manual setup.
if [[ ! -d docker ]]; then
  vcs import < fm-teleop.repos
fi
COMPOSE=(docker compose -f docker/compose.yaml -f docker/compose.macos.yaml)
export FM_IMAGE="$IMAGE"
export FM_WS="$PWD"

echo ">> building $IMAGE (FROM the fm-robot layer)"
docker build -t "$IMAGE" .
echo ">> bringing the container up (idempotent)"
"${COMPOSE[@]}" up -d
echo ">> building the workspace inside the container"
"${COMPOSE[@]}" exec fm /ros_entrypoint.sh colcon build --symlink-install
echo ">> launching joy_to_servo — publishes Servo twist from /joy"
echo ">> tear down with: ${COMPOSE[*]} down"
# `exec` skips the image ENTRYPOINT, so route through it to source ROS + overlay.
exec "${COMPOSE[@]}" exec fm /ros_entrypoint.sh "${LAUNCH[@]}"
