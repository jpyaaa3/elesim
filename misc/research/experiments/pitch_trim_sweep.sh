#!/usr/bin/env bash
# Pitch-trim evaluation sweep checklist (sim restart required per case).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$ROOT"

echo "=== Pitch trim sweep (v1: sim restart per case) ==="
echo "Primary case: bent_upward + backward"
echo ""
echo "Cases (edit the sim config mpc_pitch_trim_* then restart elesim-sim each time):"
echo "  1. baseline: kx_forward=0 kx_backward=0 kz=0"
echo "  2. forward-only: kx_forward>0"
echo "  3. backward-only: kx_backward>0"
echo "  4. z-comp: kz>0 with z_ref"
echo ""
echo "Per case:"
echo "  export ELESIM_WALKING_METRICS=1"
echo "  export ELESIM_RUN_ID=exp_trim_<case>_001"
echo "  elesim-sim --config sim/config/config.yaml"
echo "  python misc/research/experiments/walking_baseline.py --run-id \$ELESIM_RUN_ID \\"
echo "    --preset bent_upward --motion backward --duration 15 --vx -0.35 --gaze off"
echo ""
echo "Compare:"
echo "  python misc/research/analysis/analyze_walking_metrics.py --evaluate-trim exp_trim_baseline_001 exp_trim_<case>_001"
