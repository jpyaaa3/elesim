#!/usr/bin/env bash
# First preview experiment: b_pitch sign sweep (+0.05 vs -0.05).
# Prerequisite: gaze_preview_enable=true in the source config; host reachable for batch.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

SOURCE_CONFIG="${SOURCE_CONFIG:-config.ini}"
LOG_DIR="${LOG_DIR:-logs/walking_baseline}"
DURATION="${DURATION:-60}"
TRIALS="${TRIALS:-10}"
MOTION="${MOTION:-forward}"
PRESET="${PRESET:-neutral}"
TMP_CONFIG="${TMP_CONFIG:-/tmp/elesim_preview_sign_config.ini}"

if [[ "${DRY_RUN:-0}" != "1" ]] && ! grep -qE '^gaze_preview_enable\s*=\s*true' "$SOURCE_CONFIG"; then
  echo "ERROR: set gaze_preview_enable=true in $SOURCE_CONFIG before sign sweep" >&2
  exit 1
fi

_make_config() {
  local b_pitch="$1"
  cp "$SOURCE_CONFIG" "$TMP_CONFIG"
  python - "$TMP_CONFIG" "$b_pitch" <<'PY'
import re, sys
path, b_pitch = sys.argv[1], sys.argv[2]
text = open(path, encoding="utf-8").read()
text = re.sub(r"^gaze_preview_enable\s*=\s*\w+", "gaze_preview_enable = true", text, count=1, flags=re.M)
text = re.sub(r"^gaze_preview_b_pitch\s*=\s*[-+0-9.]+", f"gaze_preview_b_pitch = {b_pitch}", text, count=1, flags=re.M)
open(path, "w", encoding="utf-8").write(text)
PY
}

_run_batch() {
  local prefix="$1"
  local b_pitch="$2"
  echo "=== b_pitch=${b_pitch} (${prefix}) ==="
  _make_config "$b_pitch"
  PYTHONPATH=. python tools/run_walking_baseline_batch.py \
    --config "$TMP_CONFIG" \
    --gaze preview \
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
  echo "  analyze: PYTHONPATH=. python tools/analyze_preview_b_pitch_sign.py --log-dir $LOG_DIR"
  exit 0
fi

_run_batch exp_gaze_preview_bp05 "+0.05"
_run_batch exp_gaze_preview_bn05 "-0.05"

PYTHONPATH=. python tools/analyze_preview_b_pitch_sign.py --log-dir "$LOG_DIR"
