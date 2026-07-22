# Elesim 사용자 설명서

```text
+------------+               +------------+               +------------+
|            | <--- ZMQ ---> |            |               |            |
|            |               |   Router   |               |   Robot    |
|     UI     | <-- WebRTC -> |            | <--- ZMQ ---> |            |
|            |               |  Simulator |               | Controller |
|            | <--- RGBD --> |            |               |            |
+------------+               +------------+               +------------+
```

| 프로그램 | 책임 |
| --- | --- |
| Router | 서버 |
| Controller | 계산 |
| UI | 인터페이스 |
| Simulator | Genesis 엔진 및 기타 렌더링 |
| Robot | 실제 모터 및 Go2와 소통 + 최소한의 안전 로직 |

## 설치 마법사

설치파일의 브랜치가 `main`일 경우 아래와 같이 설치 마법사를 실행함.

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/misc/setup/bootstrap.sh | bash
```

그러나 현재 `refactoring` 브랜치를 사용 중이므로, 대신 아래 명령어를 이용함.

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/refactoring/misc/setup/bootstrap.sh | ELESIM_REF=refactoring bash
```

| 설치 방식 | 설치 범위 |
| --- | --- |
| 한 PC 시뮬레이션 | 전체 설치 |
| 조작 노트북 | Controller, UI |
| 시뮬레이션 서버 | Router, headless Simulator |
| Robot Jetson | Robot, native 전용 |
| 사용자 지정 | 설치할 옵션을 수동으로 조정 |

### 설치 가이드

설치 위치는 미지정 시 기본값 `~/.local/share/elesim`을 사용함.

먼저, `설치 위치`는 다음과 같이 입력함.

```text
/path/to/directory
```

`터미널 명령을 둘 위치`는 다음과 같이 입력함.

```text
/path/to/directory/bin
```

`GPU 사용 정책`은 다음 중 하나를 지정함.

- `inherit`: 실행 시점의 `CUDA_VISIBLE_DEVICES`를 따름.
- `specific`: 하나의 GPU index 또는 UUID를 고정하여 설치함.
- `cpu`: CPU-only 모드.

다음으로 인증서 옵션을 선택함. 입력하지 않고 Enter로 넘길 시 기본값으로 설정됨.

- `credential root`: CurveZMQ 인증서 묶음의 위치임. 
- `TURN realm`: 인증 영역 이름으로, 접속 주소는 아님. 시험 환경에서는 `elesim.local`, 정식 환경에서는 도메인 네임을 사용함.
- `Coturn 사용`: 서로 다른 NAT 사이에서 WebRTC 직접 연결이 실패할 수 있을 때 선택함. 같은 LAN에서는 끌 수 있다.

## 설치 후 공통 명령

설치한 `bin`이 PATH에 없을 시 아래 명령어를 이용해 등록함.

```bash
echo 'export PATH="/path/to/directory/bin:$PATH"' >> ~/.bashrc
```

```bash
source ~/.bashrc
```

PATH에 bin을 등록했다면 Elesim을 실행할 수 있음.

```bash
elesim-up                    # 이미지 빌드 및 백그라운드 실행
elesim-logs                  # 로그 표출
elesim-net doctor            # 주소, Router, 광고 stream 진단
elesim-net doctor --active   # 실제 RGBD/WebRTC frame까지 진단
elesim-down                  # Elesim 종료
```

설치 상태는 기본적으로 아래 경로에 저장되나, 사용자 지정 상태 경로가 출력됐다면 그 경로가 기준이 됨.

```text
~/.local/share/elesim/install-state.json
```

## 단일 컴퓨터로 시뮬레이션할 때

먼저, Elesim을 실행함.

```bash
elesim-up
```

`inherit` 옵션으로 설치하여 GPU 사용을 제한하여야 할 때에는 다음과 같이 실행함.

```bash
CUDA_VISIBLE_DEVICES=<NUMBER> elesim-up
```

Elesim을 올린 후에 정상 작동을 확인함.

```bash
elesim-logs
```

### Router

```bash
# 터미널 1
elesim-router --config router/config/default.yaml
```

### Simulator

```bash
# 터미널 2
elesim-simulator --config simulator/config/config.pc.yaml --runtime-config simulator/config/runtime.yaml --model-bundle model/bundles/default --server tcp://127.0.0.1:5558
```

### Controller

```bash
# 터미널 3
elesim-controller --config controller/config/config.pc.yaml --runtime-config controller/config/runtime.yaml --server tcp://127.0.0.1:5558 --target sim-default
```

### UI

```bash
# 터미널 4
elesim-ui --config ui/config/default.yaml --server tcp://127.0.0.1:5558 --controller-id controller-main --sim-id sim-default
```

## 별도 서버나 컴퓨터를 사용할 때

### 1. 정상 설치 여부 확인

```bash
# [서버]
elesim-up
```

```bash
# [서버]
docker compose -f /path/to/directory/containers/compose.yaml ps
```

```bash
# [서버]
elesim-logs
```

서버 방화벽 정책에 맞춰 노트북에서 필요한 TCP `5558`과 `5568` 접근을 허용하여야 함.
이는 설치 마법사가 변경해주지 않음.

### 2. Coturn 시작

현재 설치 마법사는 TURN secret과 환경 파일을 만들지만 Coturn 컨테이너를
`elesim-up`에 포함하지 않는다. Coturn을 선택했다면 `[서버]`에서 별도로 시작한다.

```bash
# [서버]
SOURCE_ROOT="$(python3 -c 'import json; print(json.load(open("~/.local/share/elesim/install-state.json"))["source_root"])')"
```

```bash
# [서버]
docker compose --env-file /path/to/directory/infra/coturn.env -f "$SOURCE_ROOT/misc/infra/coturn/compose.yaml" up -d
```

필요한 방화벽 경로는 TCP/UDP `3478`과 UDP `49160-49200`이다. 

서버와 클라이언트가 같은 LAN에 연결되어 있다면 직접 ICE가 성공하므로 Coturn을 실행하지 않아도 된다.

### 3. 노트북으로 credential 전달

```bash
# [클라이언트]
mkdir -p misc/infra/generated/remote-server/curve/{clients,router}
```

`2222`는 예시로써, 실제로 사용하는 서버의 포트는 상이할 수 있음.

```bash
# [클라이언트]
scp -P 2222 username@0.0.0.0:/path/to/directory/secrets/curve/clients/controller-main.key_secret misc/infra/generated/remote-server/curve/clients/
```

```bash
# [클라이언트]
scp -P 2222 username@0.0.0.0:/path/to/directory/secrets/curve/clients/ui-main.key_secret misc/infra/generated/remote-server/curve/clients/
```

```bash
# [클라이언트]
scp -P 2222 username@0.0.0.0:/path/to/directory/secrets/curve/clients/doctor-main.key_secret misc/infra/generated/remote-server/curve/clients/
```

```bash
# [클라이언트]
scp -P 2222 username@0.0.0.0:/path/to/directory/secrets/curve/router/router.key misc/infra/generated/remote-server/curve/router/
```

```bash
# [클라이언트]
chmod 600 misc/infra/generated/remote-server/curve/clients/*.key_secret
```

위 입력이 끝났으면 인증서가 잘 생성됐는지 확인한다.

```bash
# [클라이언트]
find misc/infra/generated/remote-server -maxdepth 5 -type f
```

```text
# 정상적으로 생성된 경우
curve/clients/controller-main.key_secret
curve/clients/ui-main.key_secret
curve/clients/doctor-main.key_secret
curve/router/router.key
```

### 4. 기존 개발 노트북에서 원격 설정 생성

1회성 작업으로써, 기존에 실행한 바가 있으면 스킵할 수 있음.

```bash
sed -e 's#tcp://sim.example.com:5558#tcp://127.0.0.1:5558#' -e 's#/etc/elesim/secrets/#remote-server/#g' controller/config/runtime.public.example.yaml > misc/infra/generated/controller.remote.yaml
```

```bash
sed -e 's#tcp://sim.example.com:5558#tcp://127.0.0.1:5558#' -e 's#/etc/elesim/secrets/#remote-server/#g' ui/config/public.example.yaml > misc/infra/generated/ui.remote.yaml
```

생성 결과를 확인하려면 아래 명령어를 입력함.

```bash
grep -R 'server_endpoint\|key_secret\|router.key' misc/infra/generated/*.remote.yaml
```

### 5. 노트북 Controller와 UI 실행

```bash
# [클라이언트]
PYTHONPATH="$PWD/packages/protocol/src:$PWD/controller/src" python3 -m elesim_controller.main --config controller/config/config.pc.yaml --runtime-config misc/infra/generated/controller.remote.yaml
```

```bash
# [노트북 개발 환경]
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

## 서버에서도 Genesis Viewer 보기

```bash
sed -i 's/extends: config.remote.yaml/extends: config.pc.yaml/' /path/to/directory/roles/simulator/config/app.installed.yaml
```

```bash
printf '%s\n' 'services:' '  simulator:' '    environment:' '      DISPLAY: "${DISPLAY}"' '    volumes:' '      - /tmp/.X11-unix:/tmp/.X11-unix:rw' > /path/to/directory/containers/viewer.override.yaml
```

```bash
xhost +si:localuser:root
```

```bash
docker compose -f /path/to/directory/containers/compose.yaml -f /path/to/directory/containers/viewer.override.yaml config --quiet
```

```bash
docker compose -f /path/to/directory/containers/compose.yaml -f /path/to/directory/containers/viewer.override.yaml up -d --force-recreate simulator
```

서버에서 Viewer 사용을 마쳤다면 아래 명령어를 입력해 X11 사용을 종료함.

```bash
xhost -si:localuser:root
```

Headless로 복구하려면 아래 명령어를 사용함.

```bash
sed -i 's/extends: config.pc.yaml/extends: config.remote.yaml/' /path/to/directory/roles/simulator/config/app.installed.yaml
elesim-up
```

## 네트워크와 보안

다음 자료는 절대 Git에 올리면 안 됨.

- `*.key_secret`
- `turn.secret`
- `coturn.env`
- 생성된 원격 설정과 전체 credential root

## 종료 여부 확인

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
