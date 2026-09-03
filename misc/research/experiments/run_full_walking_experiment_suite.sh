#!/usr/bin/env bash
# Re-run the walking/gaze experiment suite (4 conditions × 5 trials, full-trial analysis).
# Prerequisite: conda env elesim, GPU sim works on this machine (run from a real terminal).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$ROOT"
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate elesim
export PYTHONPATH=.

LOG_DIR="${LOG_DIR:-logs/walking_baseline}"
BATCH_LOG="${BATCH_LOG:-$LOG_DIR/_full_rerun_v8}"
SUITE_LOG="${SUITE_LOG:-$BATCH_LOG/suite_runner.log}"
NOTES_TAG="${NOTES_TAG:-v8}"
# 0 = full trial (entire eye-in-hand video duration)
ANALYSIS_WINDOW_S="${ANALYSIS_WINDOW_S:-0}"
mkdir -p "$BATCH_LOG"

COMMON=(
  --config payload/config/pilot/config.yaml
  --motion forward
  --preset neutral
  --max-duration 30
  --stop-at-standoff 0.85
  --pose-settle-s 2.0
  --video-preroll-s 2.0
  --sim-warmup-s 120
  --perception-warmup-s 30
  --restart-host
  --log-dir "$LOG_DIR"
  --batch-log-dir "$BATCH_LOG"
)

_run() {
  echo ""
  echo "========== $* =========="
  python misc/research/experiments/run_walking_baseline_batch.py "${COMMON[@]}" "$@"
}

ANALYZE_ARGS=()
if [[ "${ANALYSIS_WINDOW_S}" != "0" ]]; then
  ANALYZE_ARGS+=(--analysis-window-s "$ANALYSIS_WINDOW_S")
fi

FIG_ARGS=(--analysis-window-s 0)
if [[ "${ANALYSIS_WINDOW_S}" != "0" ]]; then
  FIG_ARGS=(--analysis-window-s "$ANALYSIS_WINDOW_S")
fi

{
  echo "[suite] started $(date -Iseconds)"
  echo "[suite] log_dir=$LOG_DIR batch_log=$BATCH_LOG notes_tag=$NOTES_TAG trials=5"

  echo "=== Phase 1/4: UV feedback (5 trials) ==="
  _run --gaze uv --run-prefix exp_baseline --trials 5 \
    --notes "full rerun ${NOTES_TAG} standoff0.85 dur30 uv sim-vis"

  echo "=== Phase 2/4: Gaze off (5 trials) ==="
  _run --gaze off --run-prefix exp_gaze_off --trials 5 \
    --notes "full rerun ${NOTES_TAG} standoff0.85 dur30 off sim-vis"

  echo "=== Phase 3/4: UV+FF (5 trials) ==="
  _run --gaze uv_ff --run-prefix exp_gaze_uv_ff --trials 5 \
    --notes "full rerun ${NOTES_TAG} standoff0.85 dur30 uv_ff sim-vis"

  echo "=== Phase 4/4: Body Pitch preview (5 trials) ==="
  _run --gaze pitch_preview --trials 5 \
    --run-id-stem neutral_forward_preview_pos \
    --notes "full rerun ${NOTES_TAG} standoff0.85 dur30 pitch-preview sim-vis"

  RUNS=()
  for n in 001 002 003 004 005; do
    RUNS+=(exp_gaze_off_neutral_forward_${n})
    RUNS+=(exp_baseline_neutral_forward_${n})
    RUNS+=(exp_gaze_uv_ff_neutral_forward_${n})
    RUNS+=(neutral_forward_preview_pos_${n})
  done

  echo "=== Analyze all ${NOTES_TAG} runs (full trial) ==="
  for rid in "${RUNS[@]}"; do
    python misc/research/analysis/analyze_walking_metrics.py "$rid" --log-dir "$LOG_DIR" --merged "${ANALYZE_ARGS[@]}" >/dev/null || true
  done

  echo "=== 4-way compare (trial 005 representative; full trial) ==="
  python misc/research/analysis/analyze_walking_metrics.py --log-dir "$LOG_DIR" "${ANALYZE_ARGS[@]}" --compare \
    exp_gaze_off_neutral_forward_005 \
    exp_baseline_neutral_forward_005 \
    exp_gaze_uv_ff_neutral_forward_005 \
    neutral_forward_preview_pos_005 \
    | tee "$LOG_DIR/compare_summary.json"

  echo "=== Final figures (4-way mean ± SD, n=5) ==="
  python misc/research/analysis/make_gait_preview_final_figures.py \
    --log-dir "$LOG_DIR" \
    --compare "$LOG_DIR/compare_summary.json" \
    --notes-filter "$NOTES_TAG" \
    "${FIG_ARGS[@]}" \
    --out misc/research/results/pitch_preview_final

  echo "[suite] finished $(date -Iseconds)"
} 2>&1 | tee -a "$SUITE_LOG"
