#!/usr/bin/env bash
# Fetch the MediaPipe Pose Landmarker (heavy) model into the package's models/ dir.
# Idempotent: skips the download if the file is already present.
#
#   bash scripts/download_model.sh
#
# The node's `model_path` parameter defaults to this location.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="$SCRIPT_DIR/../models"
MODEL_PATH="$MODEL_DIR/pose_landmarker_heavy.task"
URL="https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_heavy/float16/latest/pose_landmarker_heavy.task"

mkdir -p "$MODEL_DIR"

if [ -f "$MODEL_PATH" ]; then
    echo "Model already present: $MODEL_PATH"
    exit 0
fi

echo "Downloading pose_landmarker_heavy.task ..."
curl -fL --retry 3 --retry-delay 2 -o "$MODEL_PATH" "$URL"
echo "Saved to $MODEL_PATH"
