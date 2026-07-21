#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
PYTHONPATH="${ROOT}/packages/protocol/src:${ROOT}/deployments/simulator/src${PYTHONPATH:+:${PYTHONPATH}}" \
  python3 -m elesim_simulator.main \
    --config "${1:-deployments/simulator/config/config.pc.yaml}" \
    --runtime-config deployments/simulator/config/runtime.yaml \
    "${@:2}"
