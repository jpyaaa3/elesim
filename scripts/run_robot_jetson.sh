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

PYTHONPATH="${ROOT}/packages/protocol/src:${ROOT}/deployments/robot/src${PYTHONPATH:+:${PYTHONPATH}}" \
  python3 -m elesim_robot.main \
    --config deployments/robot/config/default.yaml \
    "$@"
