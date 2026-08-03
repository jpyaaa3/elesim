#!/usr/bin/env bash
set -euo pipefail

prepare=0
if [[ "${1:-}" == "--prepare" ]]; then
  prepare=1
  shift
fi
if [[ $# -eq 0 ]]; then
  set -- bash
fi

workspace="${ELESIM_WORKSPACE:-$PWD}"
venv="${ELESIM_DEV_VENV:-$HOME/.elesim/venv}"
interfaces="$workspace/packages/elesim_interfaces"
ros_overlay="${ELESIM_DEV_ROS_OVERLAY:-$HOME/.elesim/ros_overlay}"
state_root="${ELESIM_DEV_STATE_ROOT:-$HOME/.elesim}"
ready_file="$state_root/dev-env.ready"
fingerprint_file="$state_root/dev-env.fingerprint"
lock_file="$state_root/dev-env.lock"
projects=(
  "$workspace/packages/protocol"
  "$workspace/controller"
  "$workspace/ui"
  "$workspace/simulator"
  "$workspace/robot"
  "$workspace/misc/tooling/setup"
  "$workspace/misc/tooling/model_builder"
)
editable_args=()
for project in "${projects[@]}"; do
  editable_args+=(--editable "$project")
done

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

mkdir -p "$state_root" "$ros_overlay"
exec 9>"$lock_file"
flock 9

fingerprint_inputs=(
  "$interfaces/package.xml"
  "$interfaces/CMakeLists.txt"
)
while IFS= read -r -d '' interface_file; do
  fingerprint_inputs+=("$interface_file")
done < <(
  find "$interfaces/msg" "$interfaces/srv" "$interfaces/action" \
    -type f -print0 2>/dev/null | sort -z
)
for project in "${projects[@]}"; do
  fingerprint_inputs+=("$project/pyproject.toml")
done
input_fingerprint="$({
  printf 'dev-env-script\0'
  sha256sum /usr/local/bin/elesim-dev-env
  for input in "${fingerprint_inputs[@]}"; do
    printf '%s\0' "${input#"$workspace"/}"
    sha256sum "$input"
  done
} | sha256sum | awk '{print $1}')"
stored_fingerprint=""
if [[ -f "$fingerprint_file" ]]; then
  stored_fingerprint="$(<"$fingerprint_file")"
fi

if (( prepare )) || [[ ! -f "$ready_file" ]] || [[ "$input_fingerprint" != "$stored_fingerprint" ]]; then
  rm -f "$ready_file"
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  colcon --log-base "$ros_overlay/log" build \
    --base-paths "$interfaces" \
    --build-base "$ros_overlay/build" \
    --install-base "$ros_overlay/install" \
    --symlink-install >/tmp/elesim-colcon-build.log
  if [[ ! -x "$venv/bin/python" ]]; then
    "${PYTHON:-python3}" -m venv --system-site-packages --without-pip "$venv"
  fi
  "$venv/bin/python" -m pip install --disable-pip-version-check \
    --no-build-isolation --no-deps \
    "${editable_args[@]}" >/tmp/elesim-editable-install.log
  fingerprint_tmp="$fingerprint_file.tmp.$$"
  printf '%s\n' "$input_fingerprint" >"$fingerprint_tmp"
  mv -f "$fingerprint_tmp" "$fingerprint_file"
  touch "$ready_file"
fi
flock -u 9

set +u
source /opt/ros/humble/setup.bash
source "$ros_overlay/install/setup.bash"
set -u
export PATH="$venv/bin:$PATH"

exec "$@"
