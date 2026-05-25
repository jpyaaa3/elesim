# Elesim WIP

`elesim_wip`는 세그먼트형 로봇 팔과 그리퍼를 대상으로 한 **시뮬레이션 / 제어 / 하드웨어 연동 실험용 워크스페이스**입니다.

이 저장소는 다음을 한 프로젝트 안에 묶고 있습니다.

- **Genesis 기반 시뮬레이터**
- **ImGui + GLFW 기반 제어 UI**
- **UI / 시뮬레이터 / 실제 모터 사이를 중계하는 host 프로세스**
- **위치 IK와 방향 정렬을 다루는 소형 IK 모듈**

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

- [engine/ik.py](./engine/ik.py)  
  UI가 직접 호출하는 공개 IK 진입점입니다.

- [engine/iklib](./engine/iklib)  
  IK 내부 구현 패키지입니다.
  - [kinematics.py](./engine/iklib/kinematics.py): FK, grasp pose, Jacobian, 공통 수학
  - [solver.py](./engine/iklib/solver.py): 위치 중심 IK
  - [aligner.py](./engine/iklib/aligner.py): 위치가 정해진 뒤 방향을 맞추는 정렬 로직
  - [tweaker.py](./engine/iklib/tweaker.py): 미세 조정용 로직

### 부가 도구

- [addons](./addons)  
  본 실행계와 분리된 실험/분석 도구 모음입니다.

대표 예시:

- [addons/ik_solution_space_probe.py](./addons/ik_solution_space_probe.py)  
  특정 목표점에 대해 IK 해공간을 샘플링하고 시각화하는 도구

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

프로세스 간 통신은 [config.ini](./config.ini)에 정의된 **로컬 ZeroMQ endpoint**를 통해 이뤄집니다.

흐름은 다음과 같습니다.

- `ctrl.py`가 명령을 생성
- `host.py`가 이를 중계
- `sim.py`가 상태와 명령을 받아 시뮬레이션 수행
- 시뮬레이션 피드백과 하드웨어 상태가 다시 `host.py`를 통해 UI로 전달

즉 `host.py`가 전체 시스템의 허브 역할을 합니다.

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

시뮬레이터만 쓸 경우에는 [config.ini](./config.ini)에서:

```ini
use_hardware = false
```

로 두면 됩니다.

실제 하드웨어를 함께 쓸 경우에는:

```ini
use_hardware = true
```

로 바꾸고, 대상 시리얼 장치가 연결되어 있어야 합니다.

## 설정 파일

프로젝트 전체 설정은 [config.ini](./config.ini)에 있습니다.

주요 설정 예시는 다음과 같습니다.

- 하드웨어 사용 여부
- ZeroMQ endpoint
- 모터 방향 convention
- joint limit
- spawn 위치
- 디버그 마커 표시 여부

로봇 조립 결과는 [craft](./craft) 아래에 생성되며,
원본 자산과 정의는 다음 위치에 있습니다.

- [assets](./assets)
- [builder](./builder)

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

같은 동작도 UI에서 수행할 수 있습니다.

## 개발자가 어디부터 보면 좋은가

제어 흐름을 보고 싶다면:

- [ctrl.py](./ctrl.py)
- [engine/ik.py](./engine/ik.py)

위치 IK 로직을 보고 싶다면:

- [engine/iklib/solver.py](./engine/iklib/solver.py)

방향 정렬 로직을 보고 싶다면:

- [engine/iklib/aligner.py](./engine/iklib/aligner.py)

공통 기구학 계산을 보고 싶다면:

- [engine/iklib/kinematics.py](./engine/iklib/kinematics.py)

시뮬레이터 측 마커, 링크, 체인 적용을 보고 싶다면:

- [sim.py](./sim.py)

## 현재 성격

이 저장소는 **활발히 실험 중인 WIP 저장소**입니다.

따라서 다음을 예상하는 편이 맞습니다.

- 제어 로직이 계속 바뀜
- IK / 정렬 / 미세조정이 튜닝 중임
- 시뮬레이션과 하드웨어 경로가 동시에 발전 중임
- 일부 기능은 실험용 인터페이스를 유지한 채 개선되고 있음

즉, 안정된 제품 코드보다는  
로봇 제어 아이디어를 빠르게 검증하는 연구/개발용 코드베이스로 이해하는 것이 가장 정확합니다.
