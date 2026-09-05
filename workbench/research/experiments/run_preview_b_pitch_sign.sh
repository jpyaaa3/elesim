#!/usr/bin/env bash
# First preview experiment: b_pitch sign sweep (+0.05 vs -0.05).
# Prerequisite: gaze_preview_enable=true in the source config; host reachable for batch.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$ROOT"

SOURCE_CONFIG="${SOURCE_CONFIG:-payload/config/pilot/config.yaml}"
LOG_DIR="${LOG_DIR:-logs/walking_baseline}"
DURATION="${DURATION:-60}"
TRIALS="${TRIALS:-10}"
MOTION="${MOTION:-forward}"
PRESET="${PRESET:-neutral}"
TMP_CONFIG="${TMP_CONFIG:-/tmp/elesim_preview_sign_config.yaml}"

_make_config() {
  local b_pitch="$1"
  python workbench/research/experiments/config_overlay.py \
    --base "$SOURCE_CONFIG" \
    --output "$TMP_CONFIG" \
    --set behaviors.gaze.preview.enable=true \
    --set "behaviors.gaze.preview.b_pitch=${b_pitch}"
}

_run_batch() {
  local prefix="$1"
  local b_pitch="$2"
  echo "=== b_pitch=${b_pitch} (${prefix}) ==="
  _make_config "$b_pitch"
  PYTHONPATH=. python workbench/research/experiments/run_walking_baseline_batch.py \
    --config "$TMP_CONFIG" \
    --gaze pitch_preview \
    --run-prefix "$prefix" \
    --preset "$PRESET" \
    --motion "$MOTION" \
    --duration "$DURATION" \
    --trials "$TRIALS" \
    --log-dir "$LOG_DIR" \
    --notes "preview b_pitch sign sweep b=${b_pitch}"
}

if [[ "${DRY_RUN:-0}" == "1" ]]; then
  echo "DRY_RUN=1 — planned runs:"
  echo "  1) exp_gaze_preview_bp05  gaze_preview_b_pitch=+0.05"
  echo "  2) exp_gaze_preview_bn05  gaze_preview_b_pitch=-0.05"
  echo "  analyze: PYTHONPATH=. python workbench/research/analysis/analyze_preview_b_pitch_sign.py --log-dir $LOG_DIR"
  exit 0
fi

_run_batch exp_gaze_preview_bp05 "+0.05"
_run_batch exp_gaze_preview_bn05 "-0.05"

PYTHONPATH=. python workbench/research/analysis/analyze_preview_b_pitch_sign.py --log-dir "$LOG_DIR"
