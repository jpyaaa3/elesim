#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
CONFIG="${1:-pilot/config/config.pc.yaml}"

pids=()
cleanup() {
  for pid in "${pids[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

PYTHONPATH="${ROOT}/packages/protocol/src:${ROOT}/pilot/src${PYTHONPATH:+:${PYTHONPATH}}" \
  python3 -m elesim_pilot.main \
    --config "$CONFIG" \
    --runtime-config pilot/config/runtime.yaml & pids+=("$!")
PYTHONPATH="${ROOT}/packages/protocol/src:${ROOT}/ui/src${PYTHONPATH:+:${PYTHONPATH}}" \
  python3 -m elesim_ui.main \
    --config ui/config/default.yaml & pids+=("$!")
wait -n "${pids[@]}"
