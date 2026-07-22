#!/usr/bin/env bash
# Start the Elesim installer in a disposable Python container.
set -euo pipefail

repository="${ELESIM_REPOSITORY:-jpyaaa3/elesim}"
ref="${ELESIM_REF:-main}"
raw_url="${ELESIM_BOOTSTRAP_URL:-https://raw.githubusercontent.com/${repository}/${ref}/misc/setup/bootstrap.py}"
cache_dir="${ELESIM_CACHE_DIR:-$HOME/.cache/elesim/setup}"
bootstrap_file="$cache_dir/bootstrap.py"

fail() {
  printf 'Elesim bootstrap error: %s\n' "$*" >&2
  exit 2
}

command -v curl >/dev/null 2>&1 || fail "curl is required"

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
curl -fsSL "$raw_url" -o "$bootstrap_file"

docker_args=(
  run --rm -i
  --user "$(id -u):$(id -g)"
  --env "HOME=$HOME"
  --env "ELESIM_REPOSITORY=$repository"
  --env "ELESIM_REF=$ref"
  --env "ELESIM_CACHE_DIR=$cache_dir"
  --volume "$HOME:$HOME"
  --volume "$bootstrap_file:/tmp/elesim-bootstrap.py:ro"
)
if [[ -r /dev/tty ]]; then
  docker_args+=(--tty)
fi

printf '%s\n' "[bootstrap] 호스트 Python 환경을 건드리지 않고 설치 마법사를 시작합니다."
if [[ -r /dev/tty ]]; then
  "${docker_cmd[@]}" "${docker_args[@]}" python:3.10-slim \
    python /tmp/elesim-bootstrap.py "$@" </dev/tty
else
  "${docker_cmd[@]}" "${docker_args[@]}" python:3.10-slim \
    python /tmp/elesim-bootstrap.py "$@"
fi
