#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash

workspace="${ELESIM_WORKSPACE:-$PWD}"
venv="${ELESIM_DEV_VENV:-$HOME/.venv}"
projects=(
  "$workspace/packages/protocol"
  "$workspace/router"
  "$workspace/controller"
  "$workspace/ui"
  "$workspace/simulator"
  "$workspace/robot"
  "$workspace/misc/tooling/setup"
  "$workspace/misc/tooling/model_builder"
)

for project in "${projects[@]}"; do
  if [[ ! -f "$project/pyproject.toml" ]]; then
    printf 'missing Elesim development project: %s\n' "$project" >&2
    exit 2
  fi
done

if [[ ! -x "$venv/bin/python" ]]; then
  "${PYTHON:-python3}" -m venv --system-site-packages "$venv"
fi
"$venv/bin/python" -m pip install --disable-pip-version-check --no-deps --editable \
  "${projects[@]}" >/tmp/elesim-editable-install.log
export PATH="$venv/bin:$PATH"

exec "$@"
