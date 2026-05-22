# Elesim

`elesim`은 텐던/세그먼트 형태의 로봇 팔과 그리퍼를 대상으로 한 개인용 시뮬레이션/제어 프로젝트입니다.

이 저장소는 다음 요소를 함께 포함합니다.

- Genesis 기반 시뮬레이션 런타임
- ImGui + GLFW 기반 데스크톱 제어 UI
- UI, 시뮬레이터, 실제 하드웨어 사이를 연결하는 호스트 브리지
- 위치 해석과 자세 보정을 위한 소규모 IK 패키지

빠른 실험, 운동학 검증, 제어 아이디어 시험, 그리퍼 동작 확인, 하드웨어 연동 디버깅을 염두에 두고 구성된 작업용 코드베이스입니다.

## 주요 구성

- [sim.py](./sim.py)  
  Genesis 씬을 실행하고, 생성된 자산/URDF로부터 로봇을 스폰하며, 제어 명령을 받아 시뮬레이션 상태를 발행합니다.

- [ctrl.py](./ctrl.py)  
  작업자용 UI를 실행합니다. 목표 위치, 목표 방향, 하드웨어 제어, IK 실행 등의 기능이 이쪽에 모여 있습니다.

- [host.py](./host.py)  
  `ctrl.py`, `sim.py`, 선택적 Dynamixel 하드웨어 사이를 중계하는 허브 역할을 합니다. 장치 연결, 상태 브로드캐스트, 명령 전달을 담당합니다.

- [engine/ik](./engine/ik)  
  내부 IK 패키지입니다.
  - [kinematics.py](./engine/ik/kinematics.py): 순기구학, grasp pose, Jacobian, 공통 수학 처리
  - [solver.py](./engine/ik/solver.py): 위치 중심 IK 해석
  - [tweaker.py](./engine/ik/tweaker.py): 미세 자세 보정
  - [pipeline.py](./engine/ik/pipeline.py): UI에서 호출하는 IK 진입점

- [addons](./addons)  
  실험 및 분석용 보조 도구 모음입니다.  
  예: [addons/ik_solution_space_probe.py](./addons/ik_solution_space_probe.py)는 특정 목표점에 대한 IK 해 공간을 탐색합니다.

## 시스템 개요

이 시스템으로 다음 작업을 할 수 있습니다.

- 시뮬레이터에서 로봇 제어
- 필요 시 실제 하드웨어 연결
- 목표점으로부터 도달 가능한 자세 계산
- 목표 위치와 목표 방향 시각화
- 실제 grasp 위치와 방향 비교
- 메인 런타임에 로직을 고정하지 않고 IK 및 보정 전략 반복 실험

현재 로봇 모델은 다음 축을 포함합니다.

- 선형 축 1개
- 롤 축 1개
- 벤딩 제어 2개
- 그리퍼

## 실행 구조

주요 프로세스는 보통 다음 3개를 함께 실행합니다.

1. `host.py`
2. `sim.py`
3. `ctrl.py`

프로세스 간 통신은 [config.ini](./config.ini)에 설정된 로컬 ZeroMQ 엔드포인트를 사용합니다.

- 제어 채널
- 시뮬레이션 발행 채널
- 시뮬레이션 피드백 채널

`host.py`가 중심 허브입니다.  
`ctrl.py`는 `host.py`로 명령을 보내고, `sim.py`는 명령/상태를 구독하며, 시뮬레이션 피드백은 다시 호스트를 통해 전달됩니다.

## 설치

Python 의존성은 [requirements.txt](./requirements.txt)에 정리되어 있습니다.

일반적인 설정 예시는 다음과 같습니다.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

참고:

- `sim.py`를 실행하려면 활성 환경에 `genesis-world`가 설치되어 있어야 합니다.
- 프로세스 간 통신에는 `pyzmq`가 필요합니다.
- 제어 UI에는 `glfw`, `imgui[glfw]`가 필요합니다.
- 하드웨어 사용 시 `dynamixel-sdk`, `pyserial`이 필요합니다.

## 실행 방법

호스트 실행:

```bash
python3 host.py
```

시뮬레이터 실행:

```bash
python3 sim.py
```

제어 UI 실행:

```bash
python3 ctrl.py
```

시뮬레이션만 사용할 경우 [config.ini](./config.ini)에서 `use_hardware = false`로 두면 됩니다.  
하드웨어까지 함께 사용할 경우 `use_hardware = true`로 설정하고 대상 시리얼 장치가 연결되어 있어야 합니다.

## 설정

프로젝트 전반의 런타임 설정은 [config.ini](./config.ini)에서 관리합니다. 예를 들면:

- GPU / 하드웨어 사용 여부
- ZeroMQ 엔드포인트
- 모터 방향 규약
- 조인트 제한 및 모델 설정
- 스폰 위치와 디버그 마커 표시 여부

로봇 조립 결과물은 [craft](./craft)에 생성되며, 원본 데이터는 [assets](./assets)와 [builder](./builder)에 있습니다.

## 운영 흐름

UI 기준 일반적인 시뮬레이션 작업 흐름은 다음과 같습니다.

1. 목표 위치를 정합니다.
2. 목표 방향 벡터를 정합니다.
3. `Solve IK`를 실행합니다.
4. 목표 마커와 실제 grasp 마커를 비교합니다.
5. 결과가 만족스럽지 않으면 보정 로직을 다시 조정합니다.

하드웨어 모드에서는 장치 선택, 토크 제어, 그리퍼 명령 같은 기능도 함께 사용할 수 있습니다.

## 개발 메모

이 저장소는 실험을 염두에 두고 다음과 같이 분리되어 있습니다.

- 로봇 형상과 조립은 데이터 기반
- IK 로직은 런타임 제어 코드와 분리
- 분석 도구는 `addons/`에 분리
- 시뮬레이션과 하드웨어를 독립적으로 시험 가능

제어 로직을 확장할 때는 보통 다음 파일들이 주요 진입점입니다.

- [engine/ik/pipeline.py](./engine/ik/pipeline.py): UI 연계 IK 흐름
- [engine/ik/solver.py](./engine/ik/solver.py): 위치 IK 해석
- [engine/ik/tweaker.py](./engine/ik/tweaker.py): 미세 보정
- [engine/ik/kinematics.py](./engine/ik/kinematics.py): 공통 운동학 계산

## 상태

이 저장소는 현재도 계속 수정 중인 작업용 프로젝트입니다.  
실험적 동작, 구조 변경, 제어 로직 조정이 계속 포함될 수 있습니다.
