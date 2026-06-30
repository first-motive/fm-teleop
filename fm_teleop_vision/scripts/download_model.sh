#!/usr/bin/env bash
# Fetch the MediaPipe models fm_teleop_vision needs into the package's models/ dir.
# Idempotent: skips any model already present.
#
#   bash scripts/download_model.sh
#
#   hand_landmarker.task        — hand_tracker / mirror_source (1:1 hand mirroring)
#   pose_landmarker_heavy.task  — vision_source (wrist jog) + hand_tracker full_body mode
#
# The nodes' `model_path` parameters default to this location (installed to the package
# share dir at build). bash 3.2-compatible (the macOS default) — no associative arrays.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="$SCRIPT_DIR/../models"
mkdir -p "$MODEL_DIR"

fetch() {
  local name="$1" url="$2" path="$MODEL_DIR/$1"
  if [ -f "$path" ]; then
    echo "Model already present: $path"
    return
  fi
  echo "Downloading $name ..."
  curl -fL --retry 3 --retry-delay 2 -o "$path" "$url"
  echo "Saved to $path"
}

fetch hand_landmarker.task \
  "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"

fetch pose_landmarker_heavy.task \
  "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/latest/pose_landmarker_heavy.task"
