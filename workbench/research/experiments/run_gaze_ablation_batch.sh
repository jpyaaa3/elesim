#!/usr/bin/env bash
# Gaze ablation: off + uv_ff (uv already done as exp_baseline_*), then 3-way analyze.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$ROOT"
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate elesim
export PYTHONPATH=.
mkdir -p logs/walking_baseline/_batch

COMMON=(--config payload/config/pilot/config.yaml --motion forward --preset neutral
        --max-duration 30 --stop-at-standoff 0.85 --trials 10
        --sim-warmup-s 120 --perception-warmup-s 30 --no-restart-host)

echo "=== gaze ablation: off (10 trials) ==="
python workbench/research/experiments/run_walking_baseline_batch.py "${COMMON[@]}" \
  --gaze off --run-prefix exp_gaze_off --notes "gaze ablation off" \
  2>&1 | tee -a logs/walking_baseline/_batch/ablation_runner.log

echo "=== gaze ablation: uv_ff (10 trials) ==="
python workbench/research/experiments/run_walking_baseline_batch.py "${COMMON[@]}" \
  --gaze uv_ff --run-prefix exp_gaze_uv_ff --notes "gaze ablation uv_ff" \
  2>&1 | tee -a logs/walking_baseline/_batch/ablation_runner.log

echo "=== analyze per condition ==="
RUNS=()
for n in 001 002 003 004 005 006 007 008 009 010; do
  RUNS+=(exp_gaze_off_neutral_forward_${n})
  RUNS+=(exp_baseline_neutral_forward_${n})
  RUNS+=(exp_gaze_uv_ff_neutral_forward_${n})
done

for rid in "${RUNS[@]}"; do
  python workbench/research/analysis/analyze_walking_metrics.py "$rid" --log-dir logs/walking_baseline --merged >/dev/null
done

echo "=== 3-way compare (trial 001) ==="
python workbench/research/analysis/analyze_walking_metrics.py --log-dir logs/walking_baseline --compare \
  exp_gaze_off_neutral_forward_001 exp_baseline_neutral_forward_001 exp_gaze_uv_ff_neutral_forward_001 \
  | tee logs/walking_baseline/gaze_ablation_compare_001.json

echo "=== 3-way compare (all 30 runs) ==="
python workbench/research/analysis/analyze_walking_metrics.py --log-dir logs/walking_baseline --compare "${RUNS[@]}" \
  | tee logs/walking_baseline/gaze_ablation_compare_all.json

echo "=== done ==="
