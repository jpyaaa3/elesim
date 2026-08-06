#!/usr/bin/env bash
# Start the Elesim installer in a disposable Python container.
set -euo pipefail

repository="${ELESIM_REPOSITORY:-jpyaaa3/elesim}"
ref="${ELESIM_REF:-main}"
invocation_dir="${ELESIM_INVOCATION_DIR:-$PWD}"
gui_port="${ELESIM_GUI_PORT:-8765}"
raw_url="${ELESIM_BOOTSTRAP_URL:-https://raw.githubusercontent.com/${repository}/${ref}/installer/bootstrap/bootstrap.py}"
cache_dir="${ELESIM_CACHE_DIR:-$HOME/.cache/elesim/setup}"
bootstrap_file="$cache_dir/bootstrap.py"
bootstrap_tmp=""
archive_env_file=""
browser_pid=""

fail() {
  printf 'Elesim bootstrap error: %s\n' "$*" >&2
  exit 2
}

cleanup() {
  if [[ -n "$bootstrap_tmp" ]]; then
    rm -f -- "$bootstrap_tmp" >/dev/null 2>&1 || true
  fi
  if [[ -n "$archive_env_file" ]]; then
    rm -f -- "$archive_env_file" >/dev/null 2>&1 || true
  fi
  if [[ -n "$browser_pid" ]]; then
    kill "$browser_pid" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

command -v curl >/dev/null 2>&1 || fail "curl is required"
[[ "$gui_port" =~ ^[0-9]+$ ]] && ((gui_port >= 1 && gui_port <= 65535)) || \
  fail "ELESIM_GUI_PORT must be in 1..65535"
[[ -d "$invocation_dir" ]] || fail "invocation directory does not exist: $invocation_dir"

docker_cmd=(docker)
if ! command -v docker >/dev/null 2>&1; then
  if [[ ! -r /etc/os-release ]] || ! grep -q '^ID=ubuntu$' /etc/os-release; then
    fail "Docker is missing. Install Docker Engine and the Compose plugin first."
  fi
  answer="n"
  if [[ -r /dev/tty ]]; then
    read -r -p "Docker가 없습니다. Ubuntu 패키지를 설치합니까? [y/N]: " answer </dev/tty
  fi
  if [[ "$answer" =~ ^[Yy]$ ]]; then
    sudo apt-get update
    sudo apt-get install -y docker.io
    if ! sudo apt-get install -y docker-compose-v2; then
      sudo apt-get install -y docker-compose-plugin
    fi
    sudo systemctl enable --now docker
  else
    fail "Docker Engine and Docker Compose plugin are required"
  fi
fi

if ! docker info >/dev/null 2>&1; then
  if sudo docker info >/dev/null 2>&1; then
    docker_cmd=(sudo docker)
    printf '%s\n' "[bootstrap] 현재 shell은 Docker 권한이 없어 이번 실행만 sudo를 사용합니다."
    printf '%s\n' "[bootstrap] sudo usermod -aG docker \"$USER\" 후 다시 로그인하면 생성된 명령을 sudo 없이 쓸 수 있습니다."
  else
    fail "Docker daemon is not running or is not reachable"
  fi
fi

"${docker_cmd[@]}" compose version >/dev/null 2>&1 || \
  fail "Docker Compose v2 plugin ('docker compose') is required"

mkdir -p "$cache_dir" "$HOME/.local/share/elesim" "$HOME/.local/bin"
bootstrap_tmp="$(mktemp "$cache_dir/.bootstrap.py.XXXXXX")"
curl -fsSL "$raw_url" -o "$bootstrap_tmp"
mv -f -- "$bootstrap_tmp" "$bootstrap_file"
bootstrap_tmp=""
if [[ -n "${ELESIM_ARCHIVE_URL:-}" ]]; then
  case "$ELESIM_ARCHIVE_URL" in
    *$'\n'*|*$'\r'*) fail "ELESIM_ARCHIVE_URL must not contain newlines" ;;
  esac
  archive_env_file="$(mktemp "$cache_dir/.archive-env.XXXXXX")"
  printf 'ELESIM_ARCHIVE_URL=%s\n' "$ELESIM_ARCHIVE_URL" >"$archive_env_file"
fi

host_arch="$(uname -m)"
host_os_id=""
host_os_version=""
if [[ -r /etc/os-release ]]; then
  host_os_id="$(sed -n 's/^ID=//p' /etc/os-release | head -n 1 | tr -d '"')"
  host_os_version="$(sed -n 's/^VERSION_ID=//p' /etc/os-release | head -n 1 | tr -d '"')"
fi
host_jetson=0
if [[ -r /etc/nv_tegra_release ]] || \
   { [[ -r /proc/device-tree/model ]] && grep -qi jetson /proc/device-tree/model; }; then
  host_jetson=1
fi
host_wsl=0
if [[ -n "${WSL_DISTRO_NAME:-}" ]] || \
   { [[ -r /proc/sys/kernel/osrelease ]] && grep -qi microsoft /proc/sys/kernel/osrelease; }; then
  host_wsl=1
fi
host_wslg=0
if [[ -d /mnt/wslg ]]; then
  host_wslg=1
fi
host_display=0
if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
  host_display=1
fi
host_gpu_list=""
if command -v nvidia-smi >/dev/null 2>&1; then
  host_gpu_list="$(nvidia-smi -L 2>/dev/null || true)"
fi
gui_token="$(od -An -N24 -tx1 /dev/urandom | tr -d ' \n')"
gui_mode=1
for argument in "$@"; do
  case "$argument" in
    wizard|install|update|status)
      gui_mode=0
      break
      ;;
    gui)
      gui_mode=1
      break
      ;;
  esac
done

port_is_in_use() {
  (exec 3<>"/dev/tcp/127.0.0.1/$1") >/dev/null 2>&1
}

if ((gui_mode)); then
  requested_gui_port="$gui_port"
  attempts=0
  while port_is_in_use "$gui_port"; do
    ((attempts += 1))
    if ((attempts >= 100 || gui_port >= 65535)); then
      fail "could not find an available GUI port near ${requested_gui_port}"
    fi
    ((gui_port += 1))
  done
  if [[ "$gui_port" != "$requested_gui_port" ]]; then
    printf '%s\n' \
      "[bootstrap] GUI port ${requested_gui_port} is in use; selected another available port: ${gui_port}"
  fi
fi
no_open="${ELESIM_NO_OPEN:-0}"
unitree_ros2_ws="${ELESIM_UNITREE_ROS2_WS:-${UNITREE_ROS2_WS:-$HOME/ros2_ws}}"
for argument in "$@"; do
  if [[ "$argument" == "--no-open" ]]; then
    no_open=1
  fi
done

docker_args=(
  run --rm -i
  --user "$(id -u):$(id -g)"
  --workdir "$invocation_dir"
  --env "HOME=$HOME"
  --env "ELESIM_OPERATOR_HOME=$HOME"
  --env "ELESIM_UNITREE_ROS2_WS=$unitree_ros2_ws"
  --env "ELESIM_UNITREE_INTERFACE=${ELESIM_UNITREE_INTERFACE:-eth0}"
  --env "ELESIM_UNITREE_DOMAIN_ID=${ELESIM_UNITREE_DOMAIN_ID:-1}"
  --env "ELESIM_REPOSITORY=$repository"
  --env "ELESIM_REF=$ref"
  --env "ELESIM_CACHE_DIR=$cache_dir"
  --env "ELESIM_INVOCATION_DIR=$invocation_dir"
  --env "ELESIM_GUI_HOST=0.0.0.0"
  --env "ELESIM_GUI_PORT=$gui_port"
  --env "ELESIM_GUI_TOKEN=$gui_token"
  --env "ELESIM_VERIFY_BOOTSTRAP_SOURCE=1"
  --env "ELESIM_HOST_ARCH=$host_arch"
  --env "ELESIM_HOST_OS_ID=$host_os_id"
  --env "ELESIM_HOST_OS_VERSION=$host_os_version"
  --env "ELESIM_HOST_JETSON=$host_jetson"
  --env "ELESIM_HOST_WSL=$host_wsl"
  --env "ELESIM_HOST_WSLG=$host_wslg"
  --env "ELESIM_HOST_DISPLAY=$host_display"
  --env "ELESIM_HOST_USER=${USER:-dev}"
  --env "ELESIM_HOST_GPU_LIST=$host_gpu_list"
  --volume "$HOME:$HOME"
  --volume "$bootstrap_file:/tmp/elesim-bootstrap.py:ro"
)
if [[ -n "$archive_env_file" ]]; then
  docker_args+=(--env-file "$archive_env_file")
fi
if ((gui_mode)); then
  docker_args+=(--publish "127.0.0.1:${gui_port}:${gui_port}")
elif [[ -r /dev/tty ]]; then
  docker_args+=(--tty)
fi
case "$invocation_dir/" in
  "$HOME/"*) ;;
  *) docker_args+=(--volume "$invocation_dir:$invocation_dir") ;;
esac
case "$unitree_ros2_ws/" in
  "$HOME/"*|"$invocation_dir/"*) ;;
  *)
    if [[ "$unitree_ros2_ws" == /* && -d "$unitree_ros2_ws" ]]; then
      docker_args+=(--volume "$unitree_ros2_ws:$unitree_ros2_ws:ro")
    fi
    ;;
esac
if [[ -n "${SSH_AUTH_SOCK:-}" && -S "${SSH_AUTH_SOCK}" ]]; then
  docker_args+=(
    --env "SSH_AUTH_SOCK=$SSH_AUTH_SOCK"
    --volume "$SSH_AUTH_SOCK:$SSH_AUTH_SOCK"
  )
fi

gui_url="http://127.0.0.1:${gui_port}/?token=${gui_token}"
if ((gui_mode)); then
  printf '%s\n' "[bootstrap] 호스트 Python/CUDA/ROS 환경을 건드리지 않고 GUI 설치기를 시작합니다."
  printf '%s\n' "[bootstrap] ${gui_url}"
  printf '%s\n' "[remote] ssh -L ${gui_port}:127.0.0.1:${gui_port} -p <ssh-port> <user>@<server>"
else
  printf '%s\n' "[bootstrap] 호스트 Python 환경을 건드리지 않고 Elesim setup을 시작합니다."
fi
if ((gui_mode)) && [[ "$no_open" != "1" ]] && \
   command -v xdg-open >/dev/null 2>&1 && \
   [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
  (
    for _attempt in $(seq 1 180); do
      if curl -fsS "http://127.0.0.1:${gui_port}/" >/dev/null 2>&1; then
        xdg-open "$gui_url" >/dev/null 2>&1 || true
        exit 0
      fi
      sleep 1
    done
  ) &
  browser_pid="$!"
fi

if ((gui_mode)); then
  gui_arguments=(gui)
  for argument in "$@"; do
    if [[ "$argument" != "gui" ]]; then
      gui_arguments+=("$argument")
    fi
  done
  "${docker_cmd[@]}" "${docker_args[@]}" python:3.10-slim \
    python /tmp/elesim-bootstrap.py \
      "${gui_arguments[@]}" \
      --host 0.0.0.0 \
      --port "$gui_port" \
      --token "$gui_token" \
      --invocation-dir "$invocation_dir" \
      --repository "$repository" \
      --ref "$ref"
elif [[ -r /dev/tty ]]; then
  "${docker_cmd[@]}" "${docker_args[@]}" python:3.10-slim \
    python /tmp/elesim-bootstrap.py "$@" </dev/tty
else
  "${docker_cmd[@]}" "${docker_args[@]}" python:3.10-slim \
    python /tmp/elesim-bootstrap.py "$@"
fi
