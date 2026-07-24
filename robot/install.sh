#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf 'ROS 2 Humble is required at /opt/ros/humble\n' >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
set -u
colcon --log-base "${root}/ros/log" build \
  --base-paths "${root}/interfaces/elesim_interfaces" \
  --build-base "${root}/ros/build" \
  --install-base "${root}/ros/install"
python3 -m venv --system-site-packages "${root}/venv"
"${root}/venv/bin/python" -m pip install --upgrade pip
"${root}/venv/bin/python" -m pip install -r "${root}/requirements.lock"
"${root}/venv/bin/python" -m pip install --no-deps "${root}"/wheels/elesim_protocol-*.whl "${root}"/wheels/elesim_robot-*.whl
printf 'Installed Elesim robot runtime and ROSIDL overlay under %s\n' "${root}"
