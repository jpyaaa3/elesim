#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ROS_DISTRO="${ROS_DISTRO:-humble}"
UNITREE_WS="${UNITREE_ROS2_WS:-$HOME/unitree_ros2}"

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
  echo "[run_host_jetson] missing /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

if [[ -f "${UNITREE_WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${UNITREE_WS}/install/setup.bash"
else
  echo "[run_host_jetson] missing ${UNITREE_WS}/install/setup.bash" >&2
  echo "[run_host_jetson] build unitree_ros2 first, or set UNITREE_ROS2_WS" >&2
  exit 1
fi

export UNITREE_ROS2_WS="${UNITREE_WS}"
export PYTHONPATH="${ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

python3 host.py --config config.jetson.ini "$@"
