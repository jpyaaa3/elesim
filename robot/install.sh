#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf 'ROS 2 Humble is required at /opt/ros/humble\n' >&2
  exit 2
fi
for artifact in \
  "${root}/systemd/elesim-unitree-bridge.service" \
  "${root}/systemd/elesim-robot.service"; do
  if [[ ! -f "${artifact}" ]]; then
    printf 'Required Robot service artifact is missing: %s\n' "${artifact}" >&2
    exit 2
  fi
done
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
printf 'Installed Elesim Robot runtime and ROSIDL overlay under %s\n' "${root}"
printf 'Installed executables:\n'
printf '  %s/venv/bin/elesim-unitree-bridge\n' "${root}"
printf '  %s/venv/bin/elesim-robot\n' "${root}"
printf 'Standalone service artifacts (not registered or started):\n'
printf '  %s/systemd/elesim-unitree-bridge.service\n' "${root}"
printf '  %s/systemd/elesim-robot.service\n' "${root}"
printf '%s\n' \
  'No account, group, /etc configuration, or systemd state was changed.' \
  'Before starting this standalone release, an administrator must:' \
  '  1. Provide the elesim Robot account and the dedicated elesim-unitree account/group.' \
  '  2. Add the elesim Robot account to the elesim-unitree supplementary group.' \
  '  3. Install and edit /etc/elesim/robot.yaml; go2.ros_workspace/install/setup.bash must exist.' \
  '  4. Keep the Unitree DDS interface/domain private and distinct from the Elesim DDS graph.' \
  '  5. Install both service files, reload systemd, and then enable/start elesim-robot.service.' \
  'The Robot service is bound to the bridge; stopping or losing the bridge stops Robot.'
