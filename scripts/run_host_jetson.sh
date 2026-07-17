#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ROS_DISTRO="${ROS_DISTRO:-humble}"
if [[ -n "${UNITREE_ROS2_WS:-}" ]]; then
  UNITREE_WS="${UNITREE_ROS2_WS}"
elif [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  UNITREE_WS="$HOME/ros2_ws"
else
  UNITREE_WS="$HOME/unitree_ros2"
fi

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1091
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
else
  echo "[run_host_jetson] missing /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

if [[ -f "${UNITREE_WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  set +u
  source "${UNITREE_WS}/install/setup.bash"
  set -u
else
  echo "[run_host_jetson] missing ${UNITREE_WS}/install/setup.bash" >&2
  echo "[run_host_jetson] build the ROS2 workspace first, or set UNITREE_ROS2_WS" >&2
  exit 1
fi

export UNITREE_ROS2_WS="${UNITREE_WS}"
export PYTHONPATH="${ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

python3 host.py --config configs/config.jetson.yaml "$@"
