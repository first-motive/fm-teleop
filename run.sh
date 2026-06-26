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
# Curl-able (no clone needed) on macOS:
#   curl -fsSL https://raw.githubusercontent.com/first-motive/fm-teleop/main/run.sh | bash
#
# From a clone:
#   ./run.sh [--native|--container] [-h|--help] [--ros-args -p deadzone:=0.2 ...]
#
# The host OS picks the path (override with --native / --container):
#   linux  -> native:    build + launch directly on the host (ROS2 Humble installed)
#   macos  -> container: run the fm-teleop image via the fm-docker compose overlays,
#                        build + launch inside it (OrbStack)
#
# Piped via curl, the shared host checks (fm-tools lib.sh) and the compose
# overlays are fetched from their pinned tags and cached under ~/.cache/fm-teleop,
# so later runs work offline.
#
#   ./run.sh                              # auto-detect path
#   ./run.sh --native                     # force the host path (Linux)
#   ./run.sh --container                  # force the container path (macOS / OrbStack)
#   ./run.sh --ros-args -p deadzone:=0.2  # extra args pass through to ros2 run
#
# The body is wrapped in main() and called on the last line, so a truncated
# curl|bash never half-runs.
set -euo pipefail

# --- Per-repo config (downstream repos retune these) --------------------------
LOCAL_IMAGE=fm-teleop:humble                           # locally-built tag for the clone dev loop
BAKED_IMAGE=ghcr.io/first-motive/fm-teleop:humble      # published image for the no-clone baked path
LAUNCH=(ros2 run fm_teleop_device joy_to_servo)        # what `run.sh` launches
FM_TELEOP_RAW="https://raw.githubusercontent.com/first-motive/fm-teleop/main"
# lib.sh is owned by fm-tools, fetched from a pinned release tag (the single
# reuse home). The container runtime is delegated to fm-docker via install.sh.
FM_TOOLS_RAW="https://raw.githubusercontent.com/first-motive/fm-tools/v0.2.0"
CACHE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/fm-teleop"
# -----------------------------------------------------------------------------

# Keep the caller's directory: it is the workspace for the native build and the
# mount (FM_WS) for the container.
INVOKE_DIR="$PWD"

# Resolve the script's own dir; empty when piped via curl|bash. A clone has the
# repo files next to the script (REPO_DIR set); a piped run does not (REPO_DIR
# empty), so deps are fetched from the raw URLs instead.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-}")" 2>/dev/null && pwd)" || SCRIPT_DIR=""
if [ -n "$SCRIPT_DIR" ] && [ -f "$SCRIPT_DIR/fm-teleop.repos" ]; then
  REPO_DIR="$SCRIPT_DIR"
else
  REPO_DIR=""
fi

# Load the shared bootstrap library (fm-tools lib.sh) for fm_detect_os /
# fm_has_docker. Reuse a cached fetch, else fetch from the pinned fm-tools tag
# and cache it. run.sh is itself curl|bash-able, so the library may not be on
# disk. The checks must run in this shell, so source rather than execute.
load_lib() {
  local cached="$CACHE_DIR/lib.sh"
  if [ ! -f "$cached" ]; then
    mkdir -p "$CACHE_DIR"
    chmod 700 "$CACHE_DIR"  # lib.sh is sourced from here; keep the cache user-only
    # Fetch to a temp file and rename only on success: an interrupted download
    # must never leave a partial file later runs treat as cached.
    local tmp="$cached.tmp.$$"
    curl -fsSL --proto '=https' --proto-redir '=https' "$FM_TOOLS_RAW/lib.sh" -o "$tmp" \
      || { rm -f "$tmp"; echo "error: failed to fetch lib.sh from fm-tools" >&2; exit 1; }
    [ -s "$tmp" ] || { rm -f "$tmp"; echo "error: empty lib.sh download" >&2; exit 1; }
    mv "$tmp" "$cached"
  fi
  # shellcheck source=/dev/null
  source "$cached"
}

usage() {
  cat <<'EOF'
run.sh — build the workspace and launch the fm-teleop joy_to_servo node

Usage: ./run.sh [--native|--container] [-h|--help] [ros2 run args…]

  --native         force the host path (Linux)
  --container      force the container path (macOS / OrbStack)
  -h, --help       show this help

Extra args (e.g. --ros-args -p deadzone:=0.2) pass through to ros2 run.
Env: FM_SELFTEST=1  load deps + resolve OS/mode, then stop before any work.
EOF
}

main() {
  local mode=""
  local -a passthrough=()
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help)   usage; return 0 ;;
      --native)    mode=native; shift ;;
      --container) mode=container; shift ;;
      *)           passthrough+=("$1"); shift ;;
    esac
  done

  load_lib

  # Auto-detect the path from the host OS when not forced by a flag. fm_detect_os
  # (from lib.sh) echoes macos|linux.
  if [[ -z "$mode" ]]; then
    case "$(fm_detect_os)" in
      linux)  mode=native ;;
      macos)  mode=container ;;
      *) echo "error: could not resolve host path — pass --native or --container" >&2; return 1 ;;
    esac
  fi

  # CI self-test hook: deps loaded, OS + mode resolved — stop before any runtime
  # work. Lets the curl-path test exercise the piped fetch without OrbStack.
  if [ -n "${FM_SELFTEST:-}" ]; then
    echo "selftest ok: lib loaded, mode=$mode"
    return 0
  fi

  # Passthrough args (e.g. --ros-args -p deadzone:=0.2) reach the node.
  LAUNCH+=(${passthrough[@]+"${passthrough[@]}"})

  if [[ "$mode" == native ]]; then
    run_native
  else
    run_container
  fi
}

# Host path: build in place, launch on the host. No git externals to import —
# Python/npm deps resolve via rosdep (fm-teleop.repos pulls only docker/).
run_native() {
  set +u  # ROS setup scripts reference unbound vars; nounset would abort the source
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  set -u
  cd "$INVOKE_DIR"
  rosdep install --from-paths . --ignore-src -y -r 2>/dev/null || true
  colcon build --symlink-install
  set +u
  # shellcheck source=/dev/null
  source install/setup.bash
  set -u
  echo ">> launching joy_to_servo on the host — publishes Servo twist from /joy"
  exec "${LAUNCH[@]}"
}

# Container path (macOS / OrbStack). Bring up a runtime if none is present, then
# dispatch on clone vs pipe:
#   pipe (no source on disk) -> pull the baked image and run it with no mount, so
#                               the entrypoint sources the workspace baked in.
#   clone (source on disk)   -> mount source at /ws, rebuild inside, launch, so
#                               edits override the baked build (the dev loop).
run_container() {
  cd "$INVOKE_DIR"

  # Bring up a container runtime if missing — install + start OrbStack via install.sh.
  if ! fm_has_docker; then
    echo ">> no container runtime — setting up OrbStack via install.sh"
    if [ -n "$REPO_DIR" ]; then
      bash "$REPO_DIR/install.sh" --no-pull
    else
      curl -fsSL --proto '=https' --proto-redir '=https' "$FM_TELEOP_RAW/install.sh" | bash -s -- --no-pull
    fi
    fm_has_docker || { echo "error: container runtime still unavailable after setup." >&2; return 1; }
  fi

  if [ -z "$REPO_DIR" ]; then
    # Baked path: curl-to-launch, no clone, no mount. The image carries a built
    # /ws/install overlay (see Dockerfile), so route through the entrypoint to
    # source ROS + that overlay, then launch. --pull missing fetches on first run;
    # arm64 matches the macOS overlay's platform pin. joy_to_servo is a single
    # non-interactive node spinning on /joy, so no port mapping is needed.
    echo ">> running the baked image $BAKED_IMAGE (no clone, no mount)"
    echo ">> launching joy_to_servo — publishes Servo twist from /joy"
    exec docker run --rm --pull missing --platform linux/arm64 \
      "$BAKED_IMAGE" /ros_entrypoint.sh "${LAUNCH[@]}"
  fi

  # Mounted path: build the local image, bring it up, build + launch inside it.
  # The fm-docker compose overlays live in docker/, imported via fm-teleop.repos —
  # pull them on first run so a fresh clone works with no manual setup.
  if [[ ! -d docker ]]; then
    vcs import < fm-teleop.repos
  fi
  local -a compose=(docker compose -f docker/compose.yaml -f docker/compose.macos.yaml)
  export FM_IMAGE="$LOCAL_IMAGE"
  export FM_WS="$INVOKE_DIR"

  echo ">> building $LOCAL_IMAGE (FROM the fm-robot layer)"
  docker build -t "$LOCAL_IMAGE" .
  echo ">> bringing the container up (idempotent)"
  "${compose[@]}" up -d
  echo ">> building the workspace inside the container"
  "${compose[@]}" exec fm /ros_entrypoint.sh colcon build --symlink-install
  echo ">> launching joy_to_servo — publishes Servo twist from /joy"
  echo ">> tear down with: ${compose[*]} down"
  # `exec` skips the image ENTRYPOINT, so route through it to source ROS + overlay.
  exec "${compose[@]}" exec fm /ros_entrypoint.sh "${LAUNCH[@]}"
}

main "$@"
