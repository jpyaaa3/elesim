# Elesim 사용자 설명서

Elesim은 Unitree GO2에 장착된 4-DOF 분절 로봇팔을 제어하고 Genesis에서
시뮬레이션하는 분산 프로그램이다. 실행 프로그램은 서로의 Python 구현을 직접
import하지 않고 protocol-v4 메시지와 광고된 media stream으로 통신한다.

```text
조작 노트북                         Router/연산 서버
+------------------------+          +-------------------------+
| UI                     |<--ZMQ--->| Router                  |
| Controller             |          | Simulator (Genesis)     |
| - Vision / IK          |<--RGBD---| - observer WebRTC       |
| - Look/Aim/Grasp       |<--WebRTC-| - hand-eye WebRTC       |
+------------------------+          +-------------------------+
                                            |
                                            | 선택적 TURN relay
                                            v
                                          Coturn
```

다섯 프로그램의 책임은 다음과 같다.

| 프로그램 | 책임 |
| --- | --- |
| Router | endpoint 등록, 탐색, lease, 메시지 routing, WebRTC signaling |
| Controller | 인식, IK, Look-Aim-Grasp, Gaze, 목표 관절값 계산 |
| UI | 조작 화면, 명령, observer/hand-eye 영상, 원격 시뮬레이션 조작 |
| Simulator | Genesis 물리, 가상 센서, RGBD와 WebRTC 렌더링 |
| Robot | 실제 모터와 GO2 I/O, RGBD 송신, deadman과 로컬 안전 |

## 먼저 결정할 것

어느 컴퓨터에 어떤 역할을 둘지 먼저 정한다.

| 사용 형태 | 조작 노트북 | 고성능 서버 | Robot Jetson |
| --- | --- | --- | --- |
| 한 PC 시뮬레이션 | Router, Simulator, Controller, UI | 없음 | 없음 |
| 분산 시뮬레이션 | Controller, UI | Router, Simulator, 선택적 Coturn | 없음 |
| 실제 로봇 | Controller, UI, Router 또는 별도 Router | 선택 사항 | Robot |

원격 시뮬레이션의 권장 배치는 서버에 Router와 Simulator를, 노트북에 Controller와
UI를 두는 것이다. Simulator는 서버에서 계산하고, 노트북 UI는 전체 장면용
`observer`와 손끝 카메라인 `hand-eye` 영상을 각각 WebRTC로 받는다.

## 설치 전 주의사항

- 컨테이너 설치는 호스트 Python, CUDA SDK, ROS SDK를 변경하지 않는다.
- Docker가 없다면 설치 마법사가 Ubuntu 패키지 설치 여부를 묻는다.
- GPU Simulator에는 호스트 NVIDIA driver와 NVIDIA Container Toolkit이 필요하다.
- 자동 생성 Simulator 컨테이너는 현재 Ubuntu 22.04 `linux/amd64` 대상이다.
- Robot Jetson은 generic Docker 설치 대상이 아니다. JetPack/L4T 환경에서 native
  설치를 사용한다.
- 여러 컴퓨터에서 `127.0.0.1`은 항상 그 명령을 실행한 컴퓨터 자신이다.
- 공유 서버에서는 허가받은 GPU 번호를 확인한 뒤 설치한다.

## 설치 마법사

깨끗한 Ubuntu에는 Docker Compose 격리 설치를 권장한다. `git clone` 없이 설치
마법사를 시작할 수 있다.

정식 `main` 브랜치:

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/misc/setup/bootstrap.sh | bash
```

현재 `refactoring` 브랜치를 시험할 때:

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/refactoring/misc/setup/bootstrap.sh | ELESIM_REF=refactoring bash
```

원격 코드를 바로 실행하고 싶지 않다면 `bootstrap.sh`를 먼저 내려받아 읽은 뒤
실행한다. 설치 마법사는 다음 프로필을 제공한다.

| 프로필 | 역할 |
| --- | --- |
| 한 PC 시뮬레이션 | Router, Simulator, Controller, UI |
| 조작 노트북 | Controller, UI |
| 시뮬레이션 서버 | Router, headless Simulator |
| Robot Jetson | Robot, native 전용 |
| 사용자 지정 | 선택한 역할 |

### 설치 질문의 의미

`설치 위치`는 역할별 설정, 모델, Compose 파일과 cache가 들어갈 prefix다.

```text
/path/to/directory
```

`터미널 명령을 둘 위치`는 `elesim-up`, `elesim-down` 같은 래퍼를 둘 곳이다.

```text
/path/to/directory/bin
```

`GPU 사용 정책`은 다음 중 하나다.

- `inherit`: 실행 시점의 `CUDA_VISIBLE_DEVICES`를 따른다.
- `specific`: 하나의 GPU index 또는 UUID를 설치 설정에 고정한다.
- `cpu`: GPU를 노출하지 않고 Genesis CPU backend를 사용한다.

공유 서버에서 항상 GPU 0을 쓰도록 허가받았다면 `specific`과 `0`을 선택한다.
작업 스케줄러가 GPU를 배정한다면 `inherit`을 선택하고 실행할 때 지정한다.

```bash
CUDA_VISIBLE_DEVICES=<NUMBER> elesim-up
```

`.bashrc`에 `CUDA_VISIBLE_DEVICES`를 전역 설정하면 다른 연구 프로그램과 스케줄러
작업까지 영향을 받으므로 권장하지 않는다.

`Router hostname/IP`에는 다른 기기에서 도달 가능한 Router 주소를 넣는다.
SSH 접속 주소와 같을 수 있지만 SSH 포트는 Elesim 포트와 관계없다.

`RGBD stream 광고 hostname/IP`에는 Controller가 접속할 수 있는 Simulator 또는
Robot 컴퓨터의 주소를 넣는다.

`credential root`는 CurveZMQ 인증서 묶음의 위치다. Router를 처음 설치하는
서버에서는 기본값을 선택하고 새 credential을 생성한다. 노트북과 Robot에는
서버에서 생성한 역할별 키만 전달한다.

`TURN realm`은 인증 영역 이름이지 접속 주소가 아니다. 시험 환경에서는
`elesim.local`, 정식 환경에서는 관리하는 DNS 이름을 사용한다.

`Coturn 사용`은 서로 다른 NAT 사이에서 WebRTC 직접 연결이 실패할 수 있을 때
선택한다. 같은 LAN에서는 끌 수 있다.

## 설치 후 공통 명령

설치한 `bin`이 PATH에 없다면 한 번 등록한다.

```bash
echo 'export PATH="/설치/prefix/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

`~/.bashrc`가 아니라 `/.bashrc`로 입력하면 루트 디렉터리에 쓰려고 하므로
`Permission denied`가 발생한다.

```bash
elesim-up                 # 이미지 빌드 후 선택한 역할을 백그라운드로 시작
elesim-logs               # 통합 로그를 따라감
elesim-net doctor         # 주소, Router, 광고 stream 진단
elesim-net doctor --active # 실제 RGBD/WebRTC frame까지 진단
elesim-down               # 생성된 역할 종료
```

`elesim-logs`에서 `Ctrl+C`를 눌러도 로그 보기를 중단할 뿐 컨테이너는 계속
실행된다. `doctor --active`는 짧은 simulation session을 점유하므로 UI를 먼저
종료하고 실행한다.

설치 상태는 기본적으로 다음에 저장된다.

```text
~/.local/share/elesim/install-state.json
```

사용자 지정 상태 경로가 출력됐다면 그 경로가 기준이다.

## 한 컴퓨터에서 시뮬레이션

가장 간단한 방법은 설치 마법사에서 `한 PC 시뮬레이션`과 Docker Compose를
선택하는 것이다.

```bash
elesim-up
elesim-logs
```

Router가 먼저 시작되고 Simulator가 등록된 뒤 Controller와 UI가 연결된다. 최초
Simulator image build는 Genesis, Torch와 Pinocchio 때문에 오래 걸릴 수 있다.

소스 개발 환경에서는 네 프로세스를 별도 터미널에서 순서대로 실행한다.

```bash
# 터미널 1
elesim-router --config router/config/default.yaml
```

```bash
# 터미널 2
elesim-simulator --config simulator/config/config.pc.yaml --runtime-config simulator/config/runtime.yaml --model-bundle model/bundles/default --server tcp://127.0.0.1:5558
```

```bash
# 터미널 3
elesim-controller --config controller/config/config.pc.yaml --runtime-config controller/config/runtime.yaml --server tcp://127.0.0.1:5558 --target sim-default
```

```bash
# 터미널 4
elesim-ui --config ui/config/default.yaml --server tcp://127.0.0.1:5558 --controller-id controller-main --sim-id sim-default
```

`misc/scripts/run_laptop_stack.sh`는 개발 편의용이며 로컬 Router도 함께 실행한다.
원격 서버에 연결할 때는 사용하지 않는다.

## 원격 Simulator 전체 절차

이 절차에서는 명령을 실행할 컴퓨터를 `[서버]`와 `[노트북]`으로 구분한다.
예시 값은 다음과 같다.

```text
서버 IP             127.0.0.1
서버 SSH            username@127.0.0.1:8080
서버 설치 prefix    /path/to/directory
Simulator ID        sim-default
Controller ID       controller-main
```

실제 환경에 맞게 주소, 사용자, SSH 포트와 prefix를 바꾼다.

### 1. 서버 설치와 시작

`[서버]` 설치 마법사에서 다음을 선택한다.

```text
프로필             시뮬레이션 서버
설치 방식          Docker Compose
GPU 정책           specific 또는 scheduler에 맞는 inherit
Router 주소        서버의 도달 가능한 IP/DNS
RGBD 광고 주소     같은 서버 IP/DNS
보안               CURVE
credential root    <prefix>/secrets
credential 생성    yes
Coturn              NAT를 넘을 때 yes
```

설치 완료 후:

```bash
# [서버]
elesim-up
docker compose -f /path/to/directory/containers/compose.yaml ps
elesim-logs
```

`router`와 `simulator`가 모두 `Up`이어야 한다. `elesim-logs`에서 빠져나올 때는
`Ctrl+C`를 사용해도 서비스가 종료되지 않는다.

서버 방화벽 정책에 맞춰 노트북에서 필요한 TCP `5558`과 `5568` 접근을 허용한다.
기관 방화벽이나 포트 포워딩은 설치 마법사가 변경하지 않는다.

### 2. Coturn 시작

현재 설치 마법사는 TURN secret과 환경 파일을 만들지만 Coturn 컨테이너를
`elesim-up`에 포함하지 않는다. Coturn을 선택했다면 `[서버]`에서 별도로 시작한다.

```bash
# [서버]
SOURCE_ROOT="$(python3 -c 'import json; print(json.load(open("/path/to/directory/.local/share/elesim/install-state.json"))["source_root"])')"
docker compose --env-file /path/to/directory/infra/coturn.env -f "$SOURCE_ROOT/misc/infra/coturn/compose.yaml" up -d
```

필요한 방화벽 경로는 TCP/UDP `3478`과 UDP `49160-49200`이다. 직접 ICE가
성공하는 같은 LAN에서는 Coturn을 실행하지 않아도 된다.

### 3. 노트북으로 credential 전달

Private key를 Git, 메신저 또는 공개 파일 서버로 전달하지 않는다. 기존 SSH
포트를 사용해 `scp`한다. SSH가 2222번이라면 대문자 `-P 2222`를 사용한다.

```bash
# [노트북] Elesim 저장소 루트
cd ~/ws/elesim
mkdir -p misc/infra/generated/remote-server/curve/{clients,router}
```

```bash
# [노트북]
scp -P 2222 username@127.0.0.1:8080:/path/to/directory/secrets/curve/clients/controller-main.key_secret misc/infra/generated/remote-server/curve/clients/
scp -P 2222 username@127.0.0.1:8080:/path/to/directory/secrets/curve/clients/ui-main.key_secret misc/infra/generated/remote-server/curve/clients/
scp -P 2222 username@127.0.0.1:8080:/path/to/directory/secrets/curve/clients/doctor-main.key_secret misc/infra/generated/remote-server/curve/clients/
scp -P 2222 username@127.0.0.1:8080:/path/to/directory/secrets/curve/router/router.key misc/infra/generated/remote-server/curve/router/
chmod 600 misc/infra/generated/remote-server/curve/clients/*.key_secret
```

다음 세 파일을 확인한다.

```bash
# [노트북]
find misc/infra/generated/remote-server -maxdepth 5 -type f
```

```text
curve/clients/controller-main.key_secret
curve/clients/ui-main.key_secret
curve/clients/doctor-main.key_secret
curve/router/router.key
```

`misc/infra/generated/`는 `.gitignore` 대상이다. 서버의 credential 전체를 노트북에
복사하지 않는다. `scp`의 2222번은 파일 전송에만 사용되며 Elesim 제어와 영상은
다른 포트를 사용한다.

### 4. 기존 개발 노트북에서 원격 설정 생성

이미 저장소와 개발 컨테이너가 있는 노트북은 새 설치가 필요 없다. `[노트북]`
저장소 루트에서 ignored 설정 파일을 만든다.

```bash
sed -e 's#tcp://sim.example.com:5558#tcp://127.0.0.1:5558#' -e 's#/etc/elesim/secrets/#remote-server/#g' controller/config/runtime.public.example.yaml > misc/infra/generated/controller.remote.yaml
```

```bash
sed -e 's#tcp://sim.example.com:5558#tcp://127.0.0.1:5558#' -e 's#/etc/elesim/secrets/#remote-server/#g' ui/config/public.example.yaml > misc/infra/generated/ui.remote.yaml
```

생성 결과를 확인한다.

```bash
grep -R 'server_endpoint\|key_secret\|router.key' misc/infra/generated/*.remote.yaml
```

### 5. 노트북 Controller와 UI 실행

의존성이 준비된 기존 개발 환경 또는 `uropj` 컨테이너를 사용한다. 저장소의
로컬 `j` helper가 있다면 각 터미널에서 `./j`로 컨테이너에 들어갈 수 있다.

Controller 터미널:

```bash
# [노트북 개발 환경]
cd ~/ws/elesim
PYTHONPATH="$PWD/packages/protocol/src:$PWD/controller/src" python3 -m elesim_controller.main --config controller/config/config.pc.yaml --runtime-config misc/infra/generated/controller.remote.yaml
```

UI 터미널:

```bash
# [노트북 개발 환경]
cd ~/ws/elesim
PYTHONPATH="$PWD/packages/protocol/src:$PWD/ui/src" python3 -m elesim_ui.main --config misc/infra/generated/ui.remote.yaml
```

UI에서 `sim-default`가 보이면 선택한다. UI는 다음 기능을 제공한다.

- `observer`: 전체 시뮬레이션 장면 영상
- `hand-eye`: 로봇 손끝 카메라 영상
- observer 좌클릭 드래그: orbit
- observer 우클릭 드래그: pan
- wheel: zoom
- pause/resume, single-step, reset, speed, reset-view, debug marker 제어

원격으로 전달되는 것은 Genesis 운영체제 창의 화면 캡처가 아니라 Simulator가
별도로 렌더링한 두 WebRTC stream이다.

### 6. 새 노트북에 설치 마법사를 사용할 때

새 노트북에서는 위의 서버 키 네 개를 먼저 안전한 로컬 credential root에 복사한 뒤
설치 마법사에서 `조작 노트북`, 서버 Router 주소, CURVE와 해당 credential root를
선택한다. 설치 후에는 다음만 실행한다.

```bash
# [노트북]
elesim-up
elesim-net doctor --active
```

## 서버에서도 Genesis Viewer 보기

시뮬레이션 서버 프로필은 기본적으로 headless다. observer와 hand-eye 렌더링은
계속하지만 서버 바탕화면에는 Genesis Viewer를 열지 않는다. NoMachine 세션에서
네이티브 Viewer도 확인하려면 X11을 명시적으로 연결한다.

먼저 `[서버의 NoMachine 데스크톱 터미널]`에서 Viewer 설정을 활성화한다.

```bash
sed -i 's/extends: config.remote.yaml/extends: config.pc.yaml/' /path/to/directory/roles/simulator/config/app.installed.yaml
```

Compose override를 만든다.

```bash
printf '%s\n' 'services:' '  simulator:' '    environment:' '      DISPLAY: "${DISPLAY}"' '    volumes:' '      - /tmp/.X11-unix:/tmp/.X11-unix:rw' > /path/to/directory/containers/viewer.override.yaml
```

검증하고 실행한다.

```bash
xhost +si:localuser:root
docker compose -f /path/to/directory/containers/compose.yaml -f /path/to/directory/containers/viewer.override.yaml config --quiet
docker compose -f /path/to/directory/containers/compose.yaml -f /path/to/directory/containers/viewer.override.yaml up -d --force-recreate simulator
```

NoMachine의 X display가 종료되면 Viewer와 Simulator가 영향을 받을 수 있다. 장시간
서비스는 headless가 더 안정적이다. Viewer 사용을 마치면 Simulator를 먼저 내리고
임시 X 권한을 회수한다.

```bash
xhost -si:localuser:root
```

다음 실행부터 다시 headless로 쓰려면 설정을 원복한 뒤 일반 stack을 시작한다.

```bash
sed -i 's/extends: config.pc.yaml/extends: config.remote.yaml/' /path/to/directory/roles/simulator/config/app.installed.yaml
elesim-up
```

`config.pc.yaml`을 유지하는 동안에는 plain `elesim-up`이 아니라 X11 override를
포함한 Compose 명령으로 Simulator를 시작해야 한다.

## 네트워크와 보안

| 용도 | 기본 경로 |
| --- | --- |
| SSH/`scp` | 서버에서 이미 운영하는 SSH 포트, 예: TCP 2222 |
| Router 제어와 signaling | TCP 5558 |
| RGBD 직접 stream | TCP 5568 |
| WebRTC media | ICE가 선택한 UDP 경로 또는 TURN relay |
| Coturn | TCP/UDP 3478, UDP 49160-49200 |
| NoMachine | 기존 서버 설정에 따름 |

SSH 포트는 credential 파일 전달용일 뿐 Elesim runtime transport가 아니다. 서버가
SSH 2222를 사용한다면 22번을 새로 열 필요가 없다.

원격 Router는 CurveZMQ로 endpoint를 인증한다. WebRTC는 DTLS/SRTP를 사용하고,
Coturn은 Router가 발급하는 짧은 수명의 REST credential을 사용한다. 다음 자료는
절대 Git에 올리지 않는다.

- `*.key_secret`
- `turn.secret`
- `coturn.env`
- 생성된 원격 설정과 전체 credential root

## 안전하게 종료하기

노트북에서는 Controller와 UI 터미널에서 각각 `Ctrl+C`를 누른다.

Viewer override를 사용한 서버에서는:

```bash
# [서버]
docker compose -f /path/to/directory/containers/compose.yaml -f /path/to/directory/containers/viewer.override.yaml down
docker stop elesim-coturn 2>/dev/null || true
xhost -si:localuser:root
```

Headless 서버에서는:

```bash
# [서버]
elesim-down
docker stop elesim-coturn 2>/dev/null || true
```

확인:

```bash
docker ps
sudo ss -lntup | grep -E ':(5558|5568|3478)\b' || true
```

기존 SSH 2222와 NoMachine은 Elesim이 만든 서비스가 아니므로 종료하지 않는다.

## 제거와 재설치

먼저 삭제할 prefix가 정확한지 확인한다. 다음 예시는
`/path/to/directory`만 제거한다.

```bash
docker compose -f /path/to/directory/containers/compose.yaml down --remove-orphans --volumes --rmi all 2>/dev/null || true
docker rm -f elesim-coturn 2>/dev/null || true
xhost -si:localuser:root 2>/dev/null || true
sudo rm -rf /path/to/directory/
rm -f /path/to/directory/.local/share/elesim/install-state.json
rm -rf /path/to/directory/.cache/elesim/setup
```

삭제한 디렉터리 안에 현재 shell이 있었다면 다음 오류가 날 수 있다.

```text
shell-init: error retrieving current directory: getcwd: ... No such file or directory
```

이때는 `cd ~` 또는 새 터미널로 이동한다. 전역 `docker builder prune`은 다른
연구자의 build cache까지 지울 수 있으므로 사용하지 않는다.

PATH 등록을 제거하려면 `~/.bashrc`에서 해당 `export PATH=...` 한 줄만 삭제하고
새 터미널을 연다.

## 문제 해결

### `Address already in use ... 5558`

같은 호스트에서 Router가 이미 실행 중이다.

```bash
sudo ss -lntp 'sport = :5558'
```

기존 Router를 확인한 뒤 하나만 남긴다.

### `simulator is unavailable`

Router가 `sim-default` Simulator를 현재 등록된 endpoint로 찾지 못했다는 뜻이다.
UI는 0.5초마다 다시 시도하므로 먼저 서버 상태를 확인한다.

```bash
docker compose -f /설치/prefix/containers/compose.yaml ps
docker compose -f /설치/prefix/containers/compose.yaml logs --tail=200 simulator
```

`Restarting` 또는 `Exited`면 마지막 Simulator traceback을 해결해야 한다.

### `No module named elesim_simulator.config`

Python `config` package를 누락한 오래된 설치 컨텍스트다. 수정된 브랜치를 push한
뒤 setup download cache를 지우고 다시 설치한다.

```bash
rm -rf ~/.cache/elesim/setup
```

### SSH 22번이 거부되지만 `ssh -p 2222`는 동작함

서버 SSH가 2222를 쓰는 것이다. 새 22번을 열지 말고 다음처럼 사용한다.

```bash
scp -P 2222 USER@SERVER:/원본/파일 /목적지/
```

실수로 UFW 22번 규칙을 추가했다면 기존 2222 규칙은 건드리지 않고 제거한다.

```bash
sudo ufw delete allow 22/tcp
```

### `Viewer closed`

Genesis 운영체제 Viewer를 닫으면 Viewer-enabled Simulator가 예외로 종료될 수 있다.
장시간 원격 실행에는 `config.remote.yaml` headless profile을 사용한다.

### `command not found: elesim-up`

설치 시 지정한 `bin`을 PATH에 넣거나 절대경로로 실행한다.

```bash
/설치/prefix/bin/elesim-up
```

## 개발 환경

소스 개발용 Python 환경은 다음처럼 구성한다. 실제 GPU/Genesis 환경에서는 기존
`urop` 또는 `uropj` 개발 컨테이너를 사용할 수 있다.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip 'setuptools>=68' wheel
python -m pip install -r router/requirements.lock -r controller/requirements.lock -r ui/requirements.lock -r simulator/requirements.lock
python -m pip install 'git+https://github.com/elijah-waichong-chan/go2-convex-mpc.git'
python -m pip install --no-deps -e packages/protocol -e router -e controller -e ui -e simulator
python -m pip check
```

역할별 릴리스를 생성하고 격리 설치를 검증하려면:

```bash
python3 misc/tooling/release/build.py
python3 misc/tooling/release/verify.py dist/releases
```

결과는 `dist/releases/{router,controller,ui,robot,simulator}`에 생성된다. 각 역할은
자기 application wheel과 같은 버전의 protocol wheel만 가진다.

## 모델 수정

Simulator는 실행 중 URDF를 다시 만들지 않고 `model/bundles/default`를 읽는다.
geometry나 blueprint를 바꿨을 때만 개발 환경에서 재생성한다.

```bash
elesim-build-sim-bundle --assets misc/model/source/assets --output model/bundles/default
elesim-build-arm-model --config controller/config/config.pc.yaml --assets misc/model/source/assets --output controller/config/arm_model.json
```

## 테스트

```bash
python3 misc/tooling/quality/check.py --group required
python3 misc/tooling/quality/check.py --group extended
```

GUI 테스트 러너:

```bash
PYTHONPATH=packages/protocol/src:controller/src:ui/src:misc/tooling/model_builder/src python3 misc/tooling/quality/test_gui.py
```

자동 테스트는 실제 Genesis GPU 렌더링, 실제 NAT의 TURN relay 선택, 부하 상태의
WebRTC 지연, RealSense, Dynamixel과 GO2 동작을 완전히 보증하지 않는다.

## 저장소 구조

```text
router/                     Router 배포 프로젝트
controller/                 Controller 배포 프로젝트
ui/                         UI 배포 프로젝트
robot/                      Robot 배포 프로젝트
simulator/                  Simulator 배포 프로젝트
packages/protocol/          protocol-v4 계약과 transport
model/bundles/default/      Simulator 완성 모델
misc/model/source/          원본 geometry와 blueprint
misc/tooling/model_builder/ 오프라인 모델 생성
misc/tooling/release/       역할별 릴리스 생성과 검증
misc/tooling/setup/         설치 마법사와 네트워크 진단
misc/tooling/quality/       자동 테스트와 테스트 GUI
misc/tooling/debug/         수동 진단 도구
misc/tooling/experiments/   반복 실험 실행기
misc/integration/           멀티프로세스 통합 테스트
misc/infra/                 Curve credential과 Coturn 구성
misc/setup/                 git clone 없는 bootstrap
misc/scripts/               소스 개발 실행 helper
misc/docs/                  아키텍처와 배포 문서
```

세부 문서:

- [아키텍처](misc/docs/architecture.md)
- [설정 체계](misc/docs/configuration.md)
- [설치 마법사와 네트워크 진단](misc/docs/setup.md)
- [릴리스와 멀티호스트 배포](misc/docs/deployment.md)
- [미해결 문제](misc/docs/OPEN_ISSUES_KR.md)
