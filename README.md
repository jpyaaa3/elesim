# Elesim

Elesim은 Unitree GO2에 장착된 4-DOF 분절 로봇팔을 제어하고 시뮬레이션하는
분산 프로그램이다. 다섯 개 실행 프로그램은 서로의 Python 구현을 import하지
않고 protocol-v3 메시지로만 통신한다.

## 설치 구성 선택

먼저 어느 컴퓨터에 어떤 프로그램을 설치할지 정한다.

| 사용 형태 | 노트북 | 고성능 PC | Robot Jetson |
| --- | --- | --- | --- |
| 단일 PC 시뮬레이션 | Router, Controller, UI, Simulator | 없음 | 없음 |
| 분산 시뮬레이션 | Router, Controller, UI | Simulator | 없음 |
| 실제 로봇 | Router, Controller, UI | 선택 사항 | Robot |

각 프로그램의 책임은 다음과 같다.

| 프로그램 | 책임 |
| --- | --- |
| `elesim-router` | endpoint 등록, 탐색, lease 발급, 메시지 전달 |
| `elesim-controller` | 인식, IK, Look-Aim-Grasp, Gaze, 목표 관절값 계산 |
| `elesim-ui` | ImGui 조작 화면, 상태 표시, Simulator 영상 수신 |
| `elesim-simulator` | Genesis 물리 연산, 가상 센서, 영상 렌더링 |
| `elesim-robot` | Dynamixel/GO2 제어, RGBD 송신, 로컬 안전 처리 |

일반 개발에서는 저장소 소스를 editable install해서 사용한다. 실제 장비에
배포할 때는 `dist/releases/<role>`만 전달한다.

## 사전 요구 사항

- Linux와 Python 3.10 이상
- `git`, `python3-venv`, `pip`, `setuptools>=68`, `wheel`
- Simulator용 GPU 드라이버와 Genesis 실행 환경
- UI용 OpenGL, GLFW와 데스크톱 디스플레이 환경
- Robot Jetson용 Dynamixel, RealSense, ROS2 Humble 및 `unitree_ros2`
- 여러 컴퓨터를 사용할 경우 상호 접근 가능한 LAN과 고정 또는 확인 가능한 IP

과학 계산, Genesis, GUI 및 GPU 의존성이 이미 준비된 개발 컨테이너에서 설치하는
것이 가장 단순하다. Jaeger와 OpenTelemetry는 진단용 선택 사항이며 정상 실행에
필수적이지 않다.

## 개발용 소스 설치

다음 명령은 한 컴퓨터에서 시뮬레이션 전체 스택을 실행할 수 있는 개발 환경을
만든다. 저장소 루트에서 실행한다.

```bash
python3 -m venv .venv
source .venv/bin/activate

python -m pip install --upgrade pip "setuptools>=68" wheel
python -m pip install \
  -r router/requirements.lock \
  -r controller/requirements.lock \
  -r ui/requirements.lock \
  -r simulator/requirements.lock

# 기본 GO2 Simulator의 convex_mpc locomotion에 필요하다.
python -m pip install \
  "git+https://github.com/elijah-waichong-chan/go2-convex-mpc.git"

python -m pip install --no-deps \
  -e packages/protocol \
  -e router \
  -e controller \
  -e ui \
  -e simulator
```

테스트와 모델 재생성 도구까지 사용할 개발자는 이어서 설치한다.

```bash
python -m pip install "pytest>=8,<9"
python -m pip install --no-deps -e misc/tooling/model_builder
python -m pip check
```

모든 역할을 한 환경에 설치할 필요는 없다. 분산 시뮬레이션용 고성능 PC에는
`protocol + simulator`, 실제 Robot Jetson에는 `protocol + robot`만 있으면
된다. Jetson은 아래의 릴리스 설치 방식을 권장한다.

설치 후 다음 명령이 보여야 한다.

```bash
elesim-router --help
elesim-controller --help
elesim-ui --help
elesim-simulator --help
```

`command not found`가 나오면 `.venv`가 활성화됐는지와 editable install이
완료됐는지 확인한다.

## 릴리스 생성

개발 환경에서 역할별 독립 설치 묶음을 생성한다.

```bash
python3 misc/tooling/release/build.py
```

결과는 다음 위치에 생긴다.

```text
dist/releases/
├── router/
├── controller/
├── ui/
├── robot/
└── simulator/
```

각 릴리스에는 해당 프로그램 wheel, 같은 버전의 protocol wheel,
`requirements.lock`, 설정과 배포 파일만 포함된다. Wheel은 설치 가능한 Python
패키지 파일이며, 각 역할에는 application wheel과 protocol wheel 두 개가 들어간다.
Simulator 릴리스에는 검증된 `model/bundles/default`도 포함된다.

빌더는 기본적으로 다음 항목을 함께 검증한다.

- 다른 역할의 Python 패키지가 wheel에 섞이지 않았는지
- 릴리스 설정을 읽을 수 있는지
- Controller arm model과 Simulator model bundle이 완전한지
- 격리 설치한 console entrypoint가 실행되는지

검증만 다시 실행하려면 다음 명령을 사용한다.

```bash
python3 misc/tooling/release/verify.py dist/releases
```

## 릴리스 직접 설치

Router, Controller, UI, Simulator는 같은 방식으로 독립 가상환경에 설치할 수
있다. 배포할 컴퓨터에 필요한 역할의 디렉터리만 복사한 뒤 실행한다.

```bash
sudo install -d -o "$USER" -g "$USER" /opt/elesim/controller
cp -a dist/releases/controller/. /opt/elesim/controller/
cd /opt/elesim/controller
python3 -m venv venv
venv/bin/python -m pip install --upgrade pip
venv/bin/python -m pip install -r requirements.lock
venv/bin/python -m pip install --no-deps wheels/*.whl
venv/bin/python -m pip check
```

위 예시의 `controller`를 `router`, `ui`, `simulator`로 바꾸면 각 역할을
같은 방식으로 설치할 수 있다. 역할별 가상환경을 분리해야 릴리스의 독립성이
유지된다.

Simulator의 기본 설정은 `convex_mpc` locomotion을 사용한다. 이 외부 Git
패키지는 현재 `simulator/requirements.lock`에 포함되지 않으므로 Simulator
가상환경에는 별도로 설치한다.

```bash
cd /opt/elesim/simulator
venv/bin/python -m pip install \
  "git+https://github.com/elijah-waichong-chan/go2-convex-mpc.git"
```

설치된 실행 파일은 각 릴리스의 `venv/bin/` 아래에 있다.

```text
/opt/elesim/router/venv/bin/elesim-router
/opt/elesim/controller/venv/bin/elesim-controller
/opt/elesim/ui/venv/bin/elesim-ui
/opt/elesim/simulator/venv/bin/elesim-simulator
```

## Docker 이미지 빌드

Router, Controller, UI, Simulator 릴리스에는 독립 Docker build context가
포함된다. 예를 들어 Controller 이미지는 다음과 같이 만든다.

```bash
cd dist/releases/controller
set -a
. ./WHEELS.env
set +a

docker build \
  --build-arg PROTOCOL_WHEEL="$PROTOCOL_WHEEL" \
  --build-arg APP_WHEEL="$APP_WHEEL" \
  -t elesim-controller .
```

다른 역할도 해당 릴리스 디렉터리에서 이미지 태그만 바꿔 같은 방식으로 빌드한다.
제공된 Dockerfile은 역할별 wheel과 설정의 격리를 검증하는 최소 템플릿이다.
UI 컨테이너에는 호스트 디스플레이와 OpenGL 시스템 라이브러리가 필요하고,
Simulator 컨테이너에는 Genesis용 GPU 환경과 별도 `go2-convex-mpc` 설치가
필요하다. 따라서 UI와 Simulator의 실제 GPU 이미지는 준비된 개발 이미지에
릴리스 wheel을 설치하거나, 역할 Dockerfile에 해당 시스템 의존성을 추가해서
만든다.

## Robot Jetson 설치

빌드 컴퓨터에서 `dist/releases/robot/`만 Jetson으로 전달한다. Jetson에는
`elesim` 서비스 계정과 `python3-venv`가 준비돼 있어야 하며, 해당 계정에
시리얼 장치와 카메라 접근 권한을 부여해야 한다.

Jetson에서 다음 순서로 설치한다.

```bash
sudo install -d -o elesim -g elesim /opt/elesim-robot
sudo cp -a /tmp/elesim-robot/. /opt/elesim-robot/
sudo chown -R elesim:elesim /opt/elesim-robot
sudo -u elesim bash /opt/elesim-robot/install.sh

sudo install -d /etc/elesim
sudo cp /opt/elesim-robot/config/default.yaml /etc/elesim/robot.yaml
sudo editor /etc/elesim/robot.yaml
```

`/etc/elesim/robot.yaml`에서 최소한 다음 값을 실제 환경에 맞춘다.

- `runtime.server_endpoint`: Router 컴퓨터의 LAN IP와 포트
- `runtime.device`: Dynamixel 장치 경로
- `camera.advertise`: Controller가 접속할 수 있는 Jetson LAN IP
- `go2.ros_workspace`: 실제 `unitree_ros2` workspace

예시는 다음과 같다.

```yaml
runtime:
  server_endpoint: tcp://192.168.0.10:5558
  device: /dev/ttyUSB0
camera:
  bind: tcp://0.0.0.0:5568
  advertise: tcp://192.168.0.30:5568
```

설정을 마친 뒤 systemd 서비스를 설치한다.

```bash
sudo cp /opt/elesim-robot/systemd/elesim-robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now elesim-robot
sudo journalctl -u elesim-robot -f
```

GO2 backend을 사용할 때는 systemd의 `elesim` 계정에서도 ROS2와
`unitree_ros2` Python 패키지를 import할 수 있어야 한다. 서비스 등록 전에
같은 계정으로 수동 실행해 장치 권한과 ROS 환경을 먼저 확인한다.

## 네트워크 설정

모든 제어 메시지는 Router를 기준으로 연결된다. 여러 컴퓨터를 사용하는 경우
`127.0.0.1`은 자기 자신만 뜻하므로 원격 연결 주소나 advertise 주소로 쓰면
안 된다.

| 설정 파일 | 반드시 확인할 값 |
| --- | --- |
| `controller/config/runtime.yaml` | `server_endpoint`, `active_target` |
| `ui/config/default.yaml` | `server_endpoint`, Controller와 Simulator ID |
| `simulator/config/runtime.yaml` | `server_endpoint`, `streams.rgbd_advertise` |
| `robot/config/default.yaml` | `server_endpoint`, device, camera advertise |

기본 통신 포트는 다음과 같다.

| 용도 | 기본값 |
| --- | --- |
| Router ZMQ | TCP 5558 |
| RGBD 직접 스트림 | TCP 5568 |
| WebRTC 영상 | signaling은 Router, media는 직접 연결 |

Router 호스트의 방화벽은 5558을 허용해야 한다. RGBD 수신이 필요한 컴퓨터에서는
송신 호스트의 5568에 접근할 수 있어야 한다. WebRTC를 사용할 때는 호스트
방화벽과 NAT가 media 연결을 막지 않는지도 확인한다.

한 호스트의 `0.0.0.0:5558`에는 Router를 하나만 실행할 수 있다.
`Address already in use`가 나오면 기존 Router 프로세스나 컨테이너를 종료한다.

## 단일 노트북 시뮬레이션 실행

### 소스 워크스페이스

가상환경을 활성화한 뒤 터미널 두 개를 사용한다.

터미널 1에서 Router, Controller, UI를 실행한다.

```bash
source .venv/bin/activate
./misc/scripts/run_laptop_stack.sh controller/config/config.pc.yaml
```

터미널 2에서 Simulator를 실행한다.

```bash
source .venv/bin/activate
./misc/scripts/run_sim_worker.sh simulator/config/config.pc.yaml \
  --model-bundle model/bundles/default \
  --server tcp://127.0.0.1:5558
```

종료할 때는 각 터미널에서 `Ctrl+C`를 누른다. UI에서 workflow 중단 버튼을
누르는 것은 Pick 동작만 중단하며 프로세스 전체 종료 명령이 아니다.

### 설치된 릴리스

각 명령은 별도 터미널에서 Router → Simulator → Controller → UI 순서로 실행한다.

```bash
cd /opt/elesim/router
./venv/bin/elesim-router --bind tcp://0.0.0.0:5558
```

```bash
cd /opt/elesim/simulator
./venv/bin/elesim-simulator \
  --config config/default.yaml \
  --runtime-config config/runtime.yaml \
  --model-bundle model/bundles/default \
  --server tcp://127.0.0.1:5558
```

```bash
cd /opt/elesim/controller
./venv/bin/elesim-controller \
  --config config/default.yaml \
  --runtime-config config/runtime.yaml \
  --server tcp://127.0.0.1:5558 \
  --target sim-default
```

```bash
cd /opt/elesim/ui
./venv/bin/elesim-ui \
  --config config/default.yaml \
  --server tcp://127.0.0.1:5558
```

WebRTC 의존성이나 영상 경로를 의도적으로 사용하지 않을 때만 UI에
`--no-webrtc`를 추가한다.

## 원격 Simulator 실행

노트북에서 Router, Controller, UI를 실행하고 고성능 PC에서 Simulator만
실행한다. Simulator의 `server_endpoint`와 CLI `--server`에는 Router
노트북의 LAN IP를 사용한다.

```bash
cd /opt/elesim/simulator
./venv/bin/elesim-simulator \
  --config config/default.yaml \
  --runtime-config config/runtime.yaml \
  --model-bundle model/bundles/default \
  --server tcp://192.168.0.10:5558
```

이때 `config/runtime.yaml`의 `rgbd_advertise`도 고성능 PC의 실제 LAN IP로
바꿔야 한다. UI의 orbit, pan, zoom 입력은 protocol 메시지로 Simulator에
전달되므로 영상 생성 컴퓨터가 달라도 노트북에서 시점을 조작할 수 있다.

## 실제 로봇 실행

노트북에서 Router, Controller, UI를 실행한 뒤 Jetson의 Robot 서비스를
시작한다. Controller의 `active_target` 또는 `--target`은 Robot endpoint ID인
`robot-go2`로 지정한다.

```bash
cd /opt/elesim/controller
./venv/bin/elesim-controller \
  --config config/default.yaml \
  --runtime-config config/runtime.yaml \
  --server tcp://192.168.0.10:5558 \
  --target robot-go2
```

Robot은 Controller가 계산한 canonical 4-DOF `q`만 수신한다. IK나 Pick
workflow를 Jetson에서 다시 계산하지 않으며, lease 검사, stale sequence 차단,
deadman, 전류 제한과 estop만 로컬에서 처리한다.

## 모델 수정

Simulator는 실행 중 URDF를 다시 만들지 않고 사전 생성된
`model/bundles/default`를 읽는다. geometry나 blueprint를 변경했을 때만 개발
환경에서 모델을 다시 만든다.

```bash
elesim-build-sim-bundle \
  --assets misc/model/source/assets \
  --output model/bundles/default

elesim-build-arm-model \
  --config controller/config/config.pc.yaml \
  --assets misc/model/source/assets \
  --output controller/config/arm_model.json
```

모델을 바꾼 뒤에는 릴리스를 다시 생성해야 한다. Simulator runtime 재빌드는
개발 전용이며 `ELESIM_SIM_DEV_REBUILD=1`을 명시한 경우에만 허용된다.

## 테스트

표준 소프트웨어 검증 명령은 다음과 같다.

```bash
python3 misc/tooling/quality/check.py --group required
python3 misc/tooling/quality/check.py --group extended
```

`required`는 protocol, 다섯 릴리스 프로젝트, model/release 도구와 5-process
topology를 검사한다. `extended`는 분석, 디버그, 실험 도구, 코드 크기 예산과
핵심 안전 조건 mutation 검사를 실행한다.

GUI 테스트 러너는 다음과 같이 실행한다.

```bash
PYTHONPATH=packages/protocol/src:controller/src:ui/src:misc/tooling/model_builder/src \
python3 misc/tooling/quality/test_gui.py
```

실제 Genesis 렌더링, WebRTC 지연, RealSense, Dynamixel과 GO2 동작은 자동
테스트만으로 보증되지 않으므로 장비별 수동 검증이 별도로 필요하다.

## 저장소 구조

```text
router/                     Router 릴리스 프로젝트
controller/                 Controller 릴리스 프로젝트
ui/                         UI 릴리스 프로젝트
robot/                      Robot 릴리스 프로젝트
simulator/                  Simulator 릴리스 프로젝트
packages/protocol/          모든 역할이 공유하는 protocol-v3 계약
model/bundles/default/      Simulator가 읽는 완성 모델
misc/                       런타임에 설치하지 않는 개발지원 자산
misc/model/source/          원본 geometry와 blueprint
misc/tooling/model_builder/ 오프라인 모델 생성 도구
misc/tooling/release/       역할별 릴리스 생성기
misc/tooling/quality/       테스트 GUI와 품질 점검 도구
misc/tooling/debug/         수동 진단 도구
misc/tooling/experiments/   반복 가능한 실험 실행기
misc/integration/           멀티프로세스 topology 테스트
misc/scripts/               소스 워크스페이스 실행 스크립트
misc/docs/                  아키텍처, 설정과 배포 문서
misc/results/               보존된 실험 결과
```

세부 문서:

- [아키텍처](misc/docs/architecture.md)
- [설정 체계](misc/docs/configuration.md)
- [릴리스 배포](misc/docs/deployment.md)
- [미해결 문제](misc/docs/OPEN_ISSUES_KR.md)
