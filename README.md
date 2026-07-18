# Elesim

Elesim은 Unitree GO2에 장착된 4-DOF 분절 로봇팔을 제어하고 시뮬레이션하는
프로젝트다. 현재 저장소는 하나의 거대한 실행 프로그램이 아니라, 서로 독립적으로
설치하고 배포할 수 있는 다섯 개 프로그램의 소스 워크스페이스다.

기존의 `ctrl.py`, `host.py`, `sim.py` 실행 방식은 제거됐다. 각 프로그램은
공통 protocol-v3 메시지만 알고 있으며, 다른 프로그램의 Python 코드를 직접
import하지 않는다.

## 실행 구성

| 프로그램 | 일반적인 실행 위치 | 책임 |
| --- | --- | --- |
| `elesim-ui` | 노트북 | ImGui 조작 화면, 상태 표시, 시뮬레이터 영상 수신 |
| `elesim-controller` | 노트북 | 인식, IK, Look-Aim-Grasp, Gaze, 목표 관절값 계산 |
| `elesim-router` | 노트북 또는 고성능 PC | endpoint 등록, 탐색, lease 발급, 메시지 전달 |
| `elesim-robot` | 로봇의 Jetson | Dynamixel/GO2 제어, RGBD 송신, 로컬 안전 처리 |
| `elesim-simulator` | 고성능 PC 또는 노트북 | Genesis 물리 연산, 가상 센서, 영상 렌더링 |

명령, telemetry, lease와 WebRTC signaling은 ZMQ를 통해 전달된다. RGBD와
렌더링 영상처럼 큰 데이터는 Router를 거치지 않고 송신자와 수신자가 직접
통신한다.

```text
UI -- operator_intent --> Controller -- motion_command --> Robot 또는 Simulator
UI <-- operator_result -- Controller <-- telemetry/ack --- Robot 또는 Simulator
                              |
                              +---- Router에서 endpoint 탐색 및 lease 획득
```

## 폴더 구조

```text
packages/protocol/          모든 배포물이 공유하는 protocol-v3 계약
deployments/ui/             UI 소스, 설정, Docker 배포 정의
deployments/controller/     제어 알고리즘과 노트북 측 runtime
deployments/router/         ZMQ Router와 lease authority
deployments/robot/          Jetson용 드라이버, 설정, systemd 정의
deployments/simulator/      Genesis runtime과 사전 빌드 모델 로더
model/source/               원본 geometry와 blueprint
model/bundles/default/      Simulator가 기본으로 읽는 완성 모델
tooling/model_builder/      모델을 오프라인에서 생성하는 도구
tooling/release/            역할별 독립 release context 생성기
tooling/quality/            테스트 GUI와 품질 점검 도구
tooling/debug/              수동 진단 도구
tooling/experiments/        반복 가능한 실험 실행기
integration/                여러 프로세스를 함께 검증하는 테스트
docs/                       아키텍처, 설정, 배포 문서
```

각 배포물은 자신의 `config/`, `requirements.lock`, `pyproject.toml`을 소유한다.
전역 `engine/`, `configs/`, `assets/`에 의존하는 코드는 더 이상 없다.

## 로컬 시뮬레이션 실행

저장소 루트 `/home/user/ws/elesim`에서 실행한다. 개발 컨테이너처럼 의존성이
이미 설치된 환경에서는 아래 명령을 각각 별도 터미널에서 순서대로 실행하면 된다.

### 1. Router

```bash
PYTHONPATH=packages/protocol/src:deployments/router/src \
python3 -m elesim_router.main --bind tcp://0.0.0.0:5558
```

### 2. Simulator

```bash
PYTHONPATH=packages/protocol/src:deployments/simulator/src \
python3 -m elesim_simulator.main \
  --config deployments/simulator/config/config.pc.yaml \
  --runtime-config deployments/simulator/config/runtime.yaml \
  --model-bundle model/bundles/default \
  --server tcp://127.0.0.1:5558
```

### 3. Controller

```bash
PYTHONPATH=packages/protocol/src:deployments/controller/src \
python3 -m elesim_controller.main \
  --config deployments/controller/config/config.pc.yaml \
  --runtime-config deployments/controller/config/runtime.yaml \
  --server tcp://127.0.0.1:5558 \
  --target sim-default
```

### 4. UI

```bash
PYTHONPATH=packages/protocol/src:deployments/ui/src \
python3 -m elesim_ui.main \
  --config deployments/ui/config/default.yaml \
  --server tcp://127.0.0.1:5558
```

WebRTC 의존성이 없는 환경에서는 UI에 `--no-webrtc`를 추가할 수 있다. 이 경우
제어 UI는 실행되지만 시뮬레이터 렌더링 영상은 표시되지 않는다.

`scripts/run_laptop_stack.sh`는 wheel을 설치해 `elesim-*` console command를
사용할 수 있는 환경에서 Router, Controller, UI를 한꺼번에 실행한다. Simulator는
연산용 PC에서 따로 실행하거나 노트북에서 추가로 실행해야 한다.

## 실제 로봇 실행

Router와 Controller는 노트북에서 실행하고, `elesim-robot`만 로봇에 연결된
Jetson에서 실행한다. Jetson에는 전체 저장소가 필요하지 않다.

개발 소스에서 직접 실행하는 예시는 다음과 같다.

```bash
PYTHONPATH=packages/protocol/src:deployments/robot/src \
python3 -m elesim_robot.main \
  --config deployments/robot/config/default.yaml \
  --server tcp://LAPTOP_IP:5558 \
  --device /dev/ttyUSB0 \
  --camera \
  --rgbd-bind tcp://0.0.0.0:5568 \
  --rgbd-advertise tcp://JETSON_IP:5568
```

`--rgbd-bind`는 Jetson이 실제로 bind할 주소이고, `--rgbd-advertise`는
Controller가 접속할 수 있는 Jetson의 LAN 주소다. `0.0.0.0`을 advertise하면
안 된다. GO2 ROS2를 사용할 때는 Jetson 셸에서 ROS2와 `unitree_ros2` workspace를
먼저 source해야 한다.

Robot은 Controller가 계산한 4개 관절의 canonical `q`만 수신한다. IK나 Pick
workflow를 다시 계산하지 않으며, lease 검사, stale sequence 차단, deadman,
전류 제한과 estop만 로컬에서 처리한다.

## 다른 PC에서 Simulator 실행

고성능 PC에서 Simulator를 실행할 경우 `--server`에는 Router가 실행 중인
노트북 또는 서버의 IP를 지정한다.

```bash
elesim-simulator \
  --config /opt/elesim/config/default.yaml \
  --runtime-config /opt/elesim/config/runtime.yaml \
  --model-bundle /opt/elesim/model/bundles/default \
  --server tcp://LAPTOP_IP:5558
```

UI의 orbit, pan, zoom 입력은 UI → Controller → Router → Simulator 순서로
전달된다. 따라서 영상이 원격 PC에서 생성되더라도 노트북에서 시점을 조작할 수 있다.

## Release 생성과 설치

역할별 독립 배포 디렉터리는 다음 명령으로 만든다.

```bash
python3 tooling/release/build.py
```

생성 결과는 `dist/releases/<role>/`에 저장된다. 각 디렉터리에는 해당 프로그램
wheel, 동일 버전의 protocol wheel, 설정, dependency lock과 Dockerfile 또는
systemd 파일만 들어간다. Simulator release에는 `model/bundles/default`도
포함된다.

Router Docker image 빌드 예시:

```bash
cd dist/releases/router
set -a
. ./WHEELS.env
set +a
docker build \
  --build-arg PROTOCOL_WHEEL="$PROTOCOL_WHEEL" \
  --build-arg APP_WHEEL="$APP_WHEEL" \
  -t elesim-router .
```

Controller, UI, Simulator도 각각의 release 디렉터리에서 같은 방식으로 빌드한다.

Jetson에는 `dist/releases/robot`만 복사한 뒤 설치한다.

```bash
cd /opt/elesim-robot
bash install.sh
sudo mkdir -p /etc/elesim
sudo cp config/default.yaml /etc/elesim/robot.yaml
sudo cp systemd/elesim-robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now elesim-robot
```

자세한 배포 설명은 [docs/deployment.md](docs/deployment.md)를 참고한다.

## 모델 수정

Simulator는 정상 실행 중에 URDF를 다시 만들지 않고
`model/bundles/default`를 읽는다. geometry나 blueprint를 변경했을 때만 다음
명령으로 모델을 다시 생성한다.

```bash
PYTHONPATH=packages/protocol/src:deployments/controller/src:tooling/model_builder/src \
python3 -m elesim_model_builder.cli \
  --assets model/source/assets \
  --output model/bundles/default
```

runtime에서 모델을 다시 빌드하는 동작은 개발용이며
`ELESIM_SIM_DEV_REBUILD=1`을 명시한 경우에만 허용된다.

## 테스트

전체 테스트는 역할별로 분리돼 있다. 과학 계산 의존성이 설치된 개발
컨테이너에서 실행하는 것을 권장한다.

```bash
PYTHONPATH=packages/protocol/src python3 -m pytest packages/protocol/tests
PYTHONPATH=packages/protocol/src:deployments/router/src python3 -m pytest deployments/router/tests
PYTHONPATH=packages/protocol/src:deployments/robot/src python3 -m pytest deployments/robot/tests
PYTHONPATH=packages/protocol/src:deployments/controller/src python3 -m pytest deployments/controller/tests
PYTHONPATH=packages/protocol/src:deployments/simulator/src python3 -m pytest deployments/simulator/tests
PYTHONPATH=packages/protocol/src:deployments/ui/src python3 -m pytest deployments/ui/tests
```

5개 프로세스의 등록, lease, 명령 전달과 UI 요청 왕복은 다음 스모크 테스트로
확인한다.

```bash
PYTHONPATH=packages/protocol/src:deployments/router/src \
python3 integration/smoke_topology.py
```

GUI 테스트 러너:

```bash
PYTHONPATH=packages/protocol/src:deployments/controller/src:deployments/ui/src:tooling/model_builder/src \
python3 tooling/quality/test_gui.py
```

코드 책임과 통신 규칙은 [docs/architecture.md](docs/architecture.md), 설정 파일
규칙은 [docs/configuration.md](docs/configuration.md)에서 확인할 수 있다.
