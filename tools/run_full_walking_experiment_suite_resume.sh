#!/usr/bin/env bash
# Resume suite from UV+FF + Body Pitch preview + analyze.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate elesim
export PYTHONPATH=.

LOG_DIR="${LOG_DIR:-logs/walking_baseline}"
BATCH_LOG="${BATCH_LOG:-$LOG_DIR/_full_rerun_v2}"
SUITE_LOG="${SUITE_LOG:-$BATCH_LOG/suite_resume.log}"
mkdir -p "$BATCH_LOG"

COMMON=(
  --config config.ini
  --motion forward
  --preset neutral
  --max-duration 30
  --stop-at-standoff 0.85
  --pose-settle-s 2.0
  --video-preroll-s 2.0
  --sim-warmup-s 180
  --perception-warmup-s 30
  --no-restart-host
  --log-dir "$LOG_DIR"
  --batch-log-dir "$BATCH_LOG"
)

_run() {
  echo ""
  echo "========== $* =========="
  python tools/run_walking_baseline_batch.py "${COMMON[@]}" "$@"
}

{
  echo "[resume] started $(date -Iseconds)"

  echo "=== Phase 1/2: UV+FF (10 trials) ==="
  _run --gaze uv_ff --run-prefix exp_gaze_uv_ff --trials 10 \
    --notes "full rerun v2 video-preroll uv_ff"

  echo "=== Phase 2/2: Body Pitch preview (3 trials) ==="
  _run --gaze pitch_preview --trials 3 \
    --run-id-stem neutral_forward_preview_pos \
    --notes "full rerun v2 video-preroll pitch-preview"

  echo "=== Analyze all runs ==="
  RUNS=()
  for n in 001 002 003 004 005 006 007 008 009 010; do
    RUNS+=(exp_gaze_off_neutral_forward_${n})
    RUNS+=(exp_baseline_neutral_forward_${n})
    RUNS+=(exp_gaze_uv_ff_neutral_forward_${n})
  done
  for n in 001 002 003; do
    RUNS+=(neutral_forward_preview_pos_${n})
  done
  for rid in "${RUNS[@]}"; do
    python tools/analyze_walking_metrics.py "$rid" --log-dir "$LOG_DIR" --merged >/dev/null || true
  done

  echo "=== 4-way compare ==="
  python tools/analyze_walking_metrics.py --log-dir "$LOG_DIR" --compare \
    exp_gaze_off_neutral_forward_010 \
    exp_baseline_neutral_forward_010 \
    exp_gaze_uv_ff_neutral_forward_010 \
    neutral_forward_preview_pos_003 \
    | tee "$LOG_DIR/compare_summary.json"

  echo "=== Final figures ==="
  python tools/make_gait_preview_final_figures.py \
    --log-dir "$LOG_DIR" \
    --compare "$LOG_DIR/compare_summary.json" \
    --out results/pitch_preview_final

  echo "[resume] finished $(date -Iseconds)"
} 2>&1 | tee -a "$SUITE_LOG"
