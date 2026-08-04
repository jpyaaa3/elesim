#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
PYTHONPATH="${ROOT}/packages/protocol/src:${ROOT}/sim/src${PYTHONPATH:+:${PYTHONPATH}}" \
  python3 -m elesim_sim.main \
    --config "${1:-sim/config/config.pc.yaml}" \
    --runtime-config sim/config/runtime.yaml \
    "${@:2}"
