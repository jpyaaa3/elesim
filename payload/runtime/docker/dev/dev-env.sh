#!/usr/bin/env bash
set -euo pipefail

prepare=0
if [[ "${1:-}" == "--prepare" ]]; then
  prepare=1
  shift
fi
runtime_parity=0
if [[ "${1:-}" == "--runtime-parity" ]]; then
  runtime_parity=1
  shift
fi
if (( runtime_parity )) && [[ $# -ne 0 ]]; then
  printf 'runtime-parity does not accept arguments\n' >&2
  exit 64
fi
if [[ $# -eq 0 ]]; then
  set -- bash
fi

workspace="${ELESIM_WORKSPACE:-$PWD}"
venv="${ELESIM_DEV_VENV:-$HOME/.elesim/venv}"
interfaces="$workspace/payload/runtime/common/elesim_interfaces"
ros_overlay="${ELESIM_DEV_ROS_OVERLAY:-$HOME/.elesim/ros_overlay}"
state_root="${ELESIM_DEV_STATE_ROOT:-$HOME/.elesim}"
ready_file="$state_root/dev-env.ready"
fingerprint_file="$state_root/dev-env.fingerprint"
lock_file="$state_root/dev-env.lock"
projects=(
  "$workspace/payload/runtime/common/protocol"
  "$workspace/payload/runtime/docker/pilot/app"
  "$workspace/payload/runtime/docker/ui/app"
  "$workspace/payload/runtime/docker/sim/app"
  "$workspace/payload/runtime/native/robot/app"
  "$workspace/payload/runtime/docker/setup/app"
  "$workspace/model"
)
editable_args=()
for project in "${projects[@]}"; do
  editable_args+=(--editable "$project")
done

for project in "${projects[@]}"; do
  if [[ ! -f "$project/pyproject.toml" ]]; then
    printf 'missing EleSim development project: %s\n' "$project" >&2
    exit 2
  fi
done
if [[ ! -f "$interfaces/package.xml" || ! -f "$interfaces/CMakeLists.txt" ]]; then
  printf 'missing EleSim ROS interface package: %s\n' "$interfaces" >&2
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
# The development environment is intentionally broader than any one runtime
# role, but its persistent preparation must still notice deployable contract
# changes. Otherwise a source checkout can update a role lock/entrypoint while
# the long-lived dev container keeps validating an older runtime contract.
runtime_contract_inputs=(
  "$workspace/payload/runtime/docker/shared/Dockerfile.app"
  "$workspace/payload/runtime/docker/pilot/requirements.lock"
  "$workspace/payload/runtime/docker/pilot/entrypoint"
  "$workspace/payload/runtime/docker/pilot/Dockerfile.release"
  "$workspace/payload/runtime/docker/sim/requirements.lock"
  "$workspace/payload/runtime/docker/sim/entrypoint"
  "$workspace/payload/runtime/docker/sim/Dockerfile.release"
  "$workspace/payload/runtime/docker/ui/requirements.lock"
  "$workspace/payload/runtime/docker/ui/entrypoint"
  "$workspace/payload/runtime/docker/ui/Dockerfile.release"
  "$workspace/payload/runtime/native/robot/requirements.lock"
)
for input in "${runtime_contract_inputs[@]}"; do
  if [[ -f "$input" ]]; then
    fingerprint_inputs+=("$input")
  fi
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

if (( runtime_parity )); then
  parity_tool="$workspace/workbench/tools/release/runtime_parity.py"
  build_tool="$workspace/workbench/tools/release/build.py"
  verify_tool="$workspace/workbench/tools/release/verify.py"
  if [[ ! -f "$parity_tool" || ! -f "$build_tool" || ! -f "$verify_tool" ]]; then
    printf 'runtime parity tools are missing from workspace: %s\n' "$workspace" >&2
    exit 2
  fi
  (
    cd "$workspace"
    python3 "$parity_tool"
    python3 "$build_tool"
    python3 "$verify_tool" "$workspace/dist/releases"
  )
  exit 0
fi

exec "$@"
