# Elesim

`elesim_wip`는 세그먼트형 로봇 팔과 그리퍼를 대상으로 한 **시뮬레이션 / 제어 / 하드웨어 연동 실험용 워크스페이스**입니다.

이 저장소는 다음을 한 프로젝트 안에 묶고 있습니다.

- **Genesis 기반 시뮬레이터**
- **ImGui + GLFW 기반 제어 UI**
- **UI / 시뮬레이터 / 실제 모터 사이를 중계하는 host 프로세스**
- **위치 IK와 방향 정렬을 다루는 소형 IK 모듈**

현재 구조와 기능별 수정 시작점은 [Architecture Guide](./docs/architecture.md)를
기준으로 한다. 루트 실행 파일은 호환용 launcher이며, 새 런타임 wiring은
`apps/`, 재사용 가능한 동작은 `engine/`에 둔다.

목표는 완성형 제품 코드보다는,  
로봇 기구학, 처짐 모델, 그리퍼 동작, 하드웨어 디버깅 로직을 빠르게 실험하고 조정할 수 있는 개발 환경을 제공하는 것입니다.

## 프로젝트 구성

### 핵심 실행 파일

- [sim.py](./sim.py)  
  Genesis 씬을 띄우고, 생성된 URDF/자산으로 로봇을 스폰하며, 명령을 받아 시뮬레이션을 진행합니다.

- [ctrl.py](./ctrl.py)  
  운영자용 데스크톱 UI입니다. 목표 좌표, 방향 벡터, IK 실행, 하드웨어 제어, 그리퍼 제어 등이 여기서 이뤄집니다.

- [host.py](./host.py)  
  `ctrl.py`, `sim.py`, 실제 하드웨어 사이의 중계 프로세스입니다. 장치 연결, 상태 브로드캐스트, 명령 포워딩을 담당합니다.

### IK 모듈

- [engine/robot/arm/ik.py](./engine/robot/arm/ik.py)  
  UI가 직접 호출하는 공개 IK 진입점입니다.

- [engine/robot/arm/iklib](./engine/robot/arm/iklib)  
  IK 내부 구현 패키지입니다.
  - [kinematics.py](./engine/robot/arm/iklib/kinematics.py): FK, grasp pose, Jacobian, 공통 수학
  - [solver.py](./engine/robot/arm/iklib/solver.py): 위치 중심 IK
  - [aligner.py](./engine/robot/arm/iklib/aligner.py): 위치가 정해진 뒤 방향을 맞추는 정렬 로직
  - [tweaker.py](./engine/robot/arm/iklib/tweaker.py): 미세 조정용 로직

### 부가 도구

별도 실험 스크립트는 가능하면 **[engine/robot/arm/ik.py](./engine/robot/arm/ik.py)** 만 공개 API로 사용하고,
`engine/robot/arm/iklib/*` 는 내부 구현으로 취급하는 편이 좋습니다.

## 이 시스템으로 할 수 있는 일

이 프로젝트는 대략 다음과 같은 작업을 지원합니다.

- 시뮬레이터에서 로봇을 조작
- 실제 하드웨어와 연결하여 명령 전달
- 목표 좌표에 대한 위치 IK 계산
- 목표 방향 벡터와 실제 끝단 방향 비교
- 처짐 모델(`sag_model`)을 반영한 체인 거동 확인
- IK / 정렬 / 미세조정 알고리즘 실험

현재 로봇 모델은 다음 자유도를 가집니다.

- linear 1축
- roll 1축
- bend 2축
- gripper

## 실행 구조

기본적으로 아래 3개 프로세스를 함께 띄우는 구조입니다.

1. `host.py`
2. `sim.py`
3. `ctrl.py`

프로세스 간 통신은 [configs/config.yaml](./configs/config.yaml)에 정의된 **로컬 ZeroMQ endpoint**를 통해 이뤄집니다.

흐름은 다음과 같습니다.

- `ctrl.py`가 명령을 생성
- `host.py`가 이를 중계
- `sim.py`가 상태와 명령을 받아 시뮬레이션 수행
- 시뮬레이션 피드백과 하드웨어 상태가 다시 `host.py`를 통해 UI로 전달

즉 `host.py`가 전체 시스템의 허브 역할을 합니다.

외부 companion app을 붙일 때도 같은 구조를 유지합니다.

- 현재 기본 UI에는 `Visual Servoing` 패널이 있고, 여기서 카메라 인식, Look, Aim, Ready Pose, Tweak을 시작할 수 있습니다.
- 인식 경로는 rough 3D 좌표를 절대 정답으로 믿지 않고, 먼저 Look으로 현재 tip에서 물체를 바라보는 기준 시선을 만든 뒤, Aim으로 UV 중심 정렬을 수행하고 그 차이를 임시 등각 처짐으로 해석하는 방식입니다.
- 외부 앱이 별도로 좌표를 보낼 때는 `tcp://127.0.0.1:5555`로 `source="perception"` 메시지를 보내고, `host.py`가 world 좌표계 디버그 마커로 바꿔 `sim.py`에 중계합니다.
- e2 계열의 view-aware / 3D visual-servo helper는 [engine/vision/visual_servoing/pick_visual_servo.py](./engine/vision/visual_servoing/pick_visual_servo.py), [engine/vision/visual_servoing/pick_view_pregrasp.py](./engine/vision/visual_servoing/pick_view_pregrasp.py)에 실험 모듈로 보관되어 있습니다.

## 통합본 기준

현재 트리의 공식 제어 경로는 다음입니다.

- `ctrl.py`의 `Visual Servoing` 패널
- [engine/vision/perception/capture.py](./engine/vision/perception/capture.py)
- [engine/behaviors/pick/actions.py](./engine/behaviors/pick/actions.py)
- `host.py`를 통한 perception relay / marker relay

즉, 현재 기본 pick 흐름은 **rough 3D Look + e1 계열 aiming + equal-sag corrected Ready Pose 방식**입니다.
카메라가 잡은 물체를 완전한 3D 목표점으로 한 번에 풀기보다,
대략적인 3D 좌표로 먼저 물체를 바라보고, 카메라 중심 정렬 이후의 차이를 보정량으로 삼아 corrected ready pose를 만드는 경로를 공식 경로로 채택했습니다.

반면 다음은 공식 런타임 경로가 아닙니다.

- `pick_fsm` 계열의 host 중심 대형 상태기계
- `pick_fsm` 전용 UI / config 축
- 완전한 3D pregrasp 기반 pick runtime

이 저장소에는 `pick_fsm`에서 재사용 가치가 있는 수학 helper 일부만
[engine/vision/visual_servoing](./engine/vision/visual_servoing) 아래에 남겨 두었습니다.
즉, 이 저장소는 **모든 과거 브랜치의 런타임을 그대로 공존시키는 형태가 아니라, 사용 중인 경로만 공식화한 통합본**입니다.

## 설치

필수 패키지는 [requirements.txt](./requirements.txt)에 정리되어 있습니다.

예시 환경 구성:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

추가로 주의할 점:

- `sim.py`를 실행하려면 활성 환경에 `genesis-world`가 설치되어 있어야 합니다.
- 프로세스 간 통신에는 `pyzmq`가 필요합니다.
- UI 실행에는 `glfw`, `imgui[glfw]` 계열이 필요합니다.
- 실제 하드웨어를 쓰려면 `dynamixel-sdk`, `pyserial`이 필요합니다.

## 실행 방법

### 1. host 실행

```bash
python3 host.py
```

### 2. simulator 실행

```bash
python3 sim.py
```

### 3. control panel 실행

```bash
python3 ctrl.py
```

시뮬레이터만 쓸 경우에는 [configs/config.yaml](./configs/config.yaml)에서:

```yaml
simulation:
  runtime:
    use_hardware: false
```

로 두면 됩니다.

실제 하드웨어를 함께 쓸 경우에는:

```yaml
simulation:
  runtime:
    use_hardware: true
```

로 바꾸고, 대상 시리얼 장치가 연결되어 있어야 합니다.

## 설정 파일

프로젝트 전체 설정은 [configs/config.yaml](./configs/config.yaml)에 있습니다.
PC와 Jetson 전용 차이는 각각 [configs/config.pc.yaml](./configs/config.pc.yaml),
[configs/config.jetson.yaml](./configs/config.jetson.yaml)에 있으며 기본 설정을 `extends`합니다.
구조, 상속 규칙, 검증 방식은 [설정 가이드](./docs/configuration.md)를 참고하십시오.

주요 설정 예시는 다음과 같습니다.

- 하드웨어 사용 여부
- ZeroMQ endpoint
- 모터 방향 convention
- joint limit
- spawn 위치
- 디버그 마커 표시 여부

로봇 조립 결과는 [crafts](./crafts) 아래에 생성되며,
원본 자산과 정의는 다음 위치에 있습니다.

- [assets](./assets)
- [builders](./builders)

## 일반적인 사용 흐름

시뮬레이션 기준으로 보면, 보통 다음 순서로 작업합니다.

1. `ctrl.py`에서 목표 위치를 정한다.
2. 목표 방향 벡터를 입력한다.
3. `Solve IK`를 실행한다.
4. 목표 마커와 실제 끝단 마커를 비교한다.
5. 필요하면 정렬 또는 미세조정 로직을 실험한다.

하드웨어 모드에서는 여기에 더해:

- 포트 검색 / 적용
- 토크 on/off
- 그리퍼 열기 / 닫기
- 카메라 인식 시작 / 정지
- Look 실행
- Aim 시작 / 정지
- Ready Pose 실행
- Tweak 실행

같은 동작도 UI에서 수행할 수 있습니다.

Visual Servoing / Aim 관련해서는 다음처럼 이해하는 편이 정확합니다.

1. detector로 물체를 찾는다.
2. tracker로 연속 추적한다.
3. Look은 현재 tip 위치에서 감지된 물체를 바라보도록 시선을 맞추고 nominal ready baseline을 기록한다.
4. Aim은 world 좌표 절대정답을 믿기보다, 현재 카메라 시야에서 목표 UV로 중심 정렬을 수행한다.
5. Look baseline과 Aim 이후 ready pose의 차이로 임시 등각 처짐을 추정한다.
6. Ready Pose는 보정값이 있으면 corrected ready pose로, 없으면 nominal ready pose로 IK를 실행한다.

## 개발자가 어디부터 보면 좋은가

제어 흐름을 보고 싶다면:

- [ctrl.py](./ctrl.py)
- [engine/robot/arm/ik.py](./engine/robot/arm/ik.py)

위치 IK 로직을 보고 싶다면:

- [engine/robot/arm/iklib/solver.py](./engine/robot/arm/iklib/solver.py)

방향 정렬 로직을 보고 싶다면:

- [engine/robot/arm/iklib/aligner.py](./engine/robot/arm/iklib/aligner.py)

공통 기구학 계산을 보고 싶다면:

- [engine/robot/arm/iklib/kinematics.py](./engine/robot/arm/iklib/kinematics.py)

시뮬레이터 측 마커, 링크, 체인 적용을 보고 싶다면:

- [sim.py](./sim.py)

현재 visual-servo / pick helper를 보고 싶다면:

- [engine/vision/visual_servoing](./engine/vision/visual_servoing)
- [engine/behaviors/pick/actions.py](./engine/behaviors/pick/actions.py)

## 현재 성격

이 저장소는 **활발히 실험 중인 WIP 저장소**입니다.

따라서 다음을 예상하는 편이 맞습니다.

- 제어 로직이 계속 바뀜
- IK / 정렬 / 미세조정이 튜닝 중임
- 시뮬레이션과 하드웨어 경로가 동시에 발전 중임
- 일부 기능은 실험용 인터페이스를 유지한 채 개선되고 있음

즉, 안정된 제품 코드보다는  
로봇 제어 아이디어를 빠르게 검증하는 연구/개발용 코드베이스로 이해하는 것이 가장 정확합니다.
