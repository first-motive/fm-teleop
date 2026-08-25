#!/usr/bin/env bash
# Fetch the MediaPipe pose model vision_source needs into the package's models/ dir.
# Idempotent: skips the model if it is already present.
#
#   bash scripts/download_model.sh
#
#   pose_landmarker_heavy.task  — vision_source (wrist jog)
#
# The hand tracker's own models live with it in fm_data_perception, which ships the same
# script for the hand model. The node's `model_path` parameter defaults to this location
# (installed to the package share dir at build). bash 3.2-compatible (the macOS default).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="$SCRIPT_DIR/../models"
mkdir -p "$MODEL_DIR"

MODEL="$MODEL_DIR/pose_landmarker_heavy.task"
if [ -f "$MODEL" ]; then
  echo "Model already present: $MODEL"
  exit 0
fi

echo "Downloading pose_landmarker_heavy.task ..."
curl -fL --retry 3 --retry-delay 2 -o "$MODEL" \
  "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/latest/pose_landmarker_heavy.task"
echo "Saved to $MODEL"
