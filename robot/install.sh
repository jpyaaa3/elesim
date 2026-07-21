#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 -m venv "${root}/venv"
"${root}/venv/bin/python" -m pip install --upgrade pip
"${root}/venv/bin/python" -m pip install -r "${root}/requirements.lock"
"${root}/venv/bin/python" -m pip install --no-deps "${root}"/wheels/elesim_protocol-*.whl "${root}"/wheels/elesim_robot-*.whl
printf 'Installed Elesim robot runtime in %s/venv\n' "${root}"
