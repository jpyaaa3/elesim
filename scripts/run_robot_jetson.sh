#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

ROS_DISTRO="${ROS_DISTRO:-humble}"
UNITREE_WS="${UNITREE_ROS2_WS:-$HOME/ros2_ws}"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${UNITREE_WS}/install/setup.bash"
set -u

export PYTHONPATH="${ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
elesim-robot --config deployments/robot/config/default.yaml "$@"
