#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash

workspace="${ELESIM_WORKSPACE:-$PWD}"
venv="${ELESIM_DEV_VENV:-$HOME/.venv}"
projects=(
  "$workspace/packages/protocol"
  "$workspace/controller"
  "$workspace/ui"
  "$workspace/simulator"
  "$workspace/robot"
  "$workspace/misc/tooling/setup"
  "$workspace/misc/tooling/model_builder"
)
interfaces="$workspace/packages/elesim_interfaces"
ros_overlay="${ELESIM_DEV_ROS_OVERLAY:-$HOME/.elesim/ros_overlay}"

for project in "${projects[@]}"; do
  if [[ ! -f "$project/pyproject.toml" ]]; then
    printf 'missing Elesim development project: %s\n' "$project" >&2
    exit 2
  fi
done

if [[ ! -f "$interfaces/package.xml" || ! -f "$interfaces/CMakeLists.txt" ]]; then
  printf 'missing Elesim ROS interface package: %s\n' "$interfaces" >&2
  exit 2
fi

mkdir -p "$ros_overlay"
colcon --log-base "$ros_overlay/log" build \
  --base-paths "$interfaces" \
  --build-base "$ros_overlay/build" \
  --install-base "$ros_overlay/install" \
  --symlink-install >/tmp/elesim-colcon-build.log
set +u
source "$ros_overlay/install/setup.bash"
set -u

if [[ ! -x "$venv/bin/python" ]]; then
  "${PYTHON:-python3}" -m venv --system-site-packages "$venv"
fi
"$venv/bin/python" -m pip install --disable-pip-version-check --no-deps --editable \
  "${projects[@]}" >/tmp/elesim-editable-install.log
export PATH="$venv/bin:$PATH"

exec "$@"
