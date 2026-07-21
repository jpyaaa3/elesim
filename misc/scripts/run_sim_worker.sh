#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
PYTHONPATH="${ROOT}/packages/protocol/src:${ROOT}/simulator/src${PYTHONPATH:+:${PYTHONPATH}}" \
  python3 -m elesim_simulator.main \
    --config "${1:-simulator/config/config.pc.yaml}" \
    --runtime-config simulator/config/runtime.yaml \
    "${@:2}"
