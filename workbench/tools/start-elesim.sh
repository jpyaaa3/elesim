#!/usr/bin/env bash
# Start this checkout's Ubuntu Sim/UI and restart the remote Jetson services.
set -Eeuo pipefail

SCRIPT="$(readlink -f -- "${BASH_SOURCE[0]}")"
ROOT="$(dirname -- "$(dirname -- "$(dirname -- "$SCRIPT")")")"
STATE="$ROOT/local/one-click"
CONFIG="$STATE/connection.conf"
mkdir -p "$STATE"
chmod 700 "$STATE"

usage() {
  cat <<'EOF'
Usage:
  bash workbench/tools/start-elesim.sh             configure once, restart Jetson services, launch Sim/UI
  bash workbench/tools/start-elesim.sh --configure edit the saved Jetson SSH destination
  bash workbench/tools/start-elesim.sh --check     check local launch files without connecting
EOF
}

configure() {
  local host="192.168.50.11" port="22" user="idim" remote_root="/home/idim/elesim"
  if [[ -f "$CONFIG" ]]; then
    IFS= read -r host < "$CONFIG" || true
    IFS= read -r port < <(sed -n '2p' "$CONFIG") || true
    IFS= read -r user < <(sed -n '3p' "$CONFIG") || true
    IFS= read -r remote_root < <(sed -n '4p' "$CONFIG") || true
  fi
  read -r -p "Jetson SSH host [$host]: " v; host="${v:-$host}"
  read -r -p "SSH port [$port]: " v; port="${v:-$port}"
  read -r -p "Jetson user [$user]: " v; user="${v:-$user}"
  read -r -p "Jetson EleSim folder [$remote_root]: " v; remote_root="${v:-$remote_root}"
  [[ "$host" =~ ^[A-Za-z0-9._:-]+$ && "$port" =~ ^[0-9]{1,5}$ && "$user" =~ ^[A-Za-z_][A-Za-z0-9_-]*$ && "$remote_root" == /* && "$remote_root" != *$'\n'* ]] || { echo "입력값을 확인하세요." >&2; return 2; }
  ((10#$port > 0 && 10#$port <= 65535)) || { echo "SSH 포트 범위를 확인하세요." >&2; return 2; }
  umask 077
  printf '%s\n%s\n%s\n%s\n' "$host" "$port" "$user" "$remote_root" > "$CONFIG"
  chmod 600 "$CONFIG"
  echo "Jetson 접속 정보 저장: $CONFIG"
}

local_check() {
  local missing=0
  for path in "$HOME/miniforge3/etc/profile.d/conda.sh" \
    "$HOME/miniforge3/envs/elesim-sim/bin/python" \
    "$HOME/miniforge3/envs/elesim-ui/bin/python" \
    "$ROOT/local/ubuntu-sim/ros/install/setup.bash" \
    "$ROOT/local/ubuntu-ui/ros/install/setup.bash" \
    "$ROOT/local/ubuntu-sim/sim-config.yaml" \
    "$ROOT/local/ubuntu-sim/sim-runtime.yaml" \
    "$ROOT/local/ubuntu-ui/ui-runtime.yaml"; do
    if [[ ! -e "$path" ]]; then echo "없음: $path"; missing=1; fi
  done
  command -v ssh >/dev/null || { echo "ssh 명령이 없습니다."; missing=1; }
  command -v gnome-terminal >/dev/null || { echo "gnome-terminal 명령이 없습니다."; missing=1; }
  [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || { echo "그래픽 로그인 세션이 없습니다."; missing=1; }
  (( missing == 0 )) && echo "이 PC의 Sim/UI 실행 파일 확인 완료. Jetson 접속은 아직 확인하지 않았습니다."
  return "$missing"
}

stop_existing() {
  local sim_config="$ROOT/local/ubuntu-sim/sim-config.yaml"
  local ui_config="$ROOT/local/ubuntu-ui/ui-runtime.yaml"
  python3 - "$ROOT" "$sim_config" "$ui_config" <<'PY'
import os, signal, sys, time
root, sim_config, ui_config = map(os.path.realpath, sys.argv[1:])
targets = {sim_config, ui_config}
mine = os.getuid()
found = []
for entry in os.scandir('/proc'):
    if not entry.name.isdigit():
        continue
    pid = int(entry.name)
    try:
        if os.stat(f'/proc/{pid}').st_uid != mine:
            continue
        raw = open(f'/proc/{pid}/cmdline', 'rb').read().split(b'\0')
        args = [os.fsdecode(x) for x in raw if x]
        cwd = os.path.realpath(os.readlink(f'/proc/{pid}/cwd'))
    except (OSError, ProcessLookupError):
        continue
    role = None
    if '-m' in args:
        i = args.index('-m')
        if i + 1 < len(args) and args[i + 1] in ('elesim_sim.main', 'elesim_ui.main'):
            role = args[i + 1]
    # Sim is unique per system: stop every same-user Sim checkout on this PC.
    # UI remains scoped to this checkout so other operator sessions are untouched.
    is_target = role == 'elesim_sim.main' or (
        cwd == root and role == 'elesim_ui.main' and
        any(args[i] == '--config' and i + 1 < len(args) and os.path.realpath(args[i+1]) in targets for i in range(len(args)))
    )
    if is_target:
        if pid != os.getpid():
            found.append(pid)
if found:
    print('기존 Sim/UI 종료 요청 (SIGINT): ' + ', '.join(map(str, found)), flush=True)
for pid in found:
    try: os.kill(pid, signal.SIGINT)
    except ProcessLookupError: pass
deadline = time.monotonic() + 20
while found and time.monotonic() < deadline:
    found = [p for p in found if os.path.exists(f'/proc/{p}')]
    time.sleep(.25)
if found:
    print('정상 종료되지 않은 Sim/UI에 종료 신호를 보냅니다: ' + ', '.join(map(str, found)), flush=True)
    for pid in found:
        try: os.kill(pid, signal.SIGTERM)
        except ProcessLookupError: pass
    deadline = time.monotonic() + 5
    while found and time.monotonic() < deadline:
        found = [p for p in found if os.path.exists(f'/proc/{p}')]
        time.sleep(.2)
if found:
    print('종료되지 않은 Sim/UI 프로세스가 있어 새 Sim 실행을 중단합니다: ' + ', '.join(map(str, found)), file=sys.stderr)
    sys.exit(1)
print('이 PC에서 실행 중이던 Sim과 현재 checkout의 UI를 종료했습니다.')
PY
}

role_worker() {
  local role="$1" run="$2"
  [[ "$run" =~ ^[0-9]{8}T[0-9]{6}$ && ( "$role" == sim || "$role" == ui ) ]] || exit 2
  local run_dir="$STATE/runs/$run"
  mkdir -p "$run_dir"
  local log="$run_dir/$role.log" status_file="$run_dir/$role.exit"
  set +u
  source "$HOME/miniforge3/etc/profile.d/conda.sh"
  if [[ "$role" == sim ]]; then
    conda activate elesim-sim
    cd "$ROOT"
    source "$ROOT/local/ubuntu-sim/ros/install/setup.bash"
    python -u -m elesim_sim.main --viewer \
      --config "$ROOT/local/ubuntu-sim/sim-config.yaml" \
      --runtime-config "$ROOT/local/ubuntu-sim/sim-runtime.yaml"
  else
    conda activate elesim-ui
    cd "$ROOT"
    source "$ROOT/local/ubuntu-ui/ros/install/setup.bash"
    python -u -m elesim_ui.main --config "$ROOT/local/ubuntu-ui/ui-runtime.yaml"
  fi
}

terminal_role() {
  set +e
  role_worker "$2" "$3" 2>&1 | tee -a "$STATE/runs/$3/$2.log"
  local rc=${PIPESTATUS[0]}
  echo
  echo "프로그램이 종료되었습니다 (exit=$rc). 로그: $STATE/runs/$3/$2.log"
  read -r -p "이 창을 닫으려면 Enter를 누르세요. " _ || true
  return "$rc"
}

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then usage; exit 0; fi
if [[ "${1:-}" == --configure ]]; then configure; exit; fi
if [[ "${1:-}" == --check ]]; then local_check; exit; fi
if [[ "${1:-}" == --worker ]]; then role_worker "$2" "$3"; exit; fi
if [[ "${1:-}" == --terminal ]]; then terminal_role "$@"; exit; fi

[[ $# == 0 ]] || { usage >&2; exit 2; }
[[ -f "$CONFIG" ]] || configure
local_check

mapfile -t cfg < "$CONFIG"
[[ ${#cfg[@]} == 4 ]] || { echo "Jetson 설정 파일이 올바르지 않습니다. --configure로 다시 입력하세요." >&2; exit 2; }
host="${cfg[0]}" port="${cfg[1]}" user="${cfg[2]}" remote_root="${cfg[3]}"
[[ "$host" =~ ^[A-Za-z0-9._:-]+$ && "$port" =~ ^[0-9]{1,5}$ && "$user" =~ ^[A-Za-z_][A-Za-z0-9_-]*$ && "$remote_root" == /* ]] || { echo "Jetson 설정 파일을 확인하세요." >&2; exit 2; }

# Remote actions are deliberately limited to the staged Pilot unit and the three EleSim services.
read -r -d '' REMOTE_SCRIPT <<'REMOTE' || true
set -Eeuo pipefail
mode="$1"; root="$2"
[[ "$root" == /* && -x "$root/bin/elesim-robot" && -x "$root/bin/elesim-unitree-bridge" ]] || { echo "Jetson EleSim 설치 경로가 맞지 않습니다: $root" >&2; exit 2; }
for f in "$root/roles/robot/config/installed.yaml" "$root/roles/pilot/config/installed.yaml" "$root/roles/pilot/config/runtime.installed.yaml" "$root/local/systemd/elesim-pilot.service" "$root/local/systemd/wait-network.py"; do
  [[ -f "$f" ]] || { echo "Jetson 파일이 없습니다: $f" >&2; exit 2; }
done
command -v systemctl >/dev/null
[[ "$(uname -m)" == aarch64 ]] || { echo "접속한 장비가 Jetson ARM64가 아닙니다." >&2; exit 2; }
if [[ "$mode" == preflight ]]; then
  systemctl list-unit-files elesim-robot.service elesim-unitree-bridge.service >/dev/null
  echo "Jetson 파일 및 서비스 사전 확인 완료."
  exit 0
fi
sudo -v
if ! systemctl cat elesim-pilot.service >/dev/null 2>&1; then
  sudo install -m 0644 "$root/local/systemd/elesim-pilot.service" /etc/systemd/system/elesim-pilot.service
  sudo systemctl daemon-reload
fi
sudo systemctl stop elesim-pilot.service || true
"$root/roles/pilot/venv/bin/python" "$root/local/systemd/wait-network.py" \
  "$root/roles/pilot/config/runtime.installed.yaml" --timeout 120
sudo systemctl restart elesim-robot.service
sudo systemctl is-active --quiet elesim-unitree-bridge.service
sudo systemctl is-active --quiet elesim-robot.service
sudo systemctl start elesim-pilot.service
sudo systemctl is-active --quiet elesim-pilot.service
echo "Jetson 서비스 상태:"
systemctl is-active elesim-unitree-bridge.service elesim-robot.service elesim-pilot.service
sudo journalctl -b -u elesim-unitree-bridge.service -u elesim-robot.service -u elesim-pilot.service -n 12 --no-pager
REMOTE

remote_call() {
  local mode="$1" remote_cmd
  printf -v remote_cmd 'bash -c %q -- %q %q' "$REMOTE_SCRIPT" "$mode" "$remote_root"
  ssh -t -p "$port" -o ConnectTimeout=8 -o ServerAliveInterval=15 "$user@$host" "$remote_cmd"
}

exec 9>"$STATE/launcher.lock"
flock -n 9 || { echo "이미 이 스크립트가 실행 중입니다." >&2; exit 2; }
echo "시작 전 이 PC의 기존 Sim 프로세스를 정리합니다."
stop_existing
echo "Jetson 접속과 서비스 사전 확인: $user@$host:$port"
remote_call preflight
echo "Jetson Pilot 중지 → 네트워크 대기 → Robot/bridge 재시작 → Pilot 시작"
remote_call restart

run="$(date +%Y%m%dT%H%M%S)"
run_dir="$STATE/runs/$run"
mkdir -p "$run_dir"
chmod 700 "$STATE/runs" "$run_dir"
gnome-terminal --wait --title="EleSim Sim" -- bash "$SCRIPT" --terminal sim "$run" 9>&- >"$run_dir/sim-launch.log" 2>&1 &
sim_pid=$!
gnome-terminal --wait --title="EleSim UI" -- bash "$SCRIPT" --terminal ui "$run" 9>&- >"$run_dir/ui-launch.log" 2>&1 &
ui_pid=$!
exec 9>&-
sleep 2
kill -0 "$sim_pid" 2>/dev/null || { cat "$run_dir/sim-launch.log" >&2; echo "Sim 터미널을 열지 못했습니다." >&2; exit 1; }
kill -0 "$ui_pid" 2>/dev/null || { cat "$run_dir/ui-launch.log" >&2; echo "UI 터미널을 열지 못했습니다." >&2; exit 1; }
echo "Sim/UI 터미널을 열었습니다. 실행 로그: $run_dir"
echo "UI에서 target robot-go2, Safety fault, 최신 telemetry를 확인하세요. 이 스크립트는 모터 Torque를 켜거나 fault를 지우지 않습니다."
