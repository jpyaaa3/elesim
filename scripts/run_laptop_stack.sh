#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
CONFIG="${1:-deployments/controller/config/config.pc.yaml}"

pids=()
cleanup() {
  for pid in "${pids[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

PYTHONPATH="${ROOT}/packages/protocol/src:${ROOT}/deployments/router/src${PYTHONPATH:+:${PYTHONPATH}}" \
  python3 -m elesim_router.main --bind tcp://0.0.0.0:5558 & pids+=("$!")
PYTHONPATH="${ROOT}/packages/protocol/src:${ROOT}/deployments/controller/src${PYTHONPATH:+:${PYTHONPATH}}" \
  python3 -m elesim_controller.main \
    --config "$CONFIG" \
    --runtime-config deployments/controller/config/runtime.yaml & pids+=("$!")
PYTHONPATH="${ROOT}/packages/protocol/src:${ROOT}/deployments/ui/src${PYTHONPATH:+:${PYTHONPATH}}" \
  python3 -m elesim_ui.main \
    --config deployments/ui/config/default.yaml & pids+=("$!")
wait -n "${pids[@]}"
