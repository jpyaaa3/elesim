# Jetson 혼합 역할 배치 확장 작업 기록

상태: 구현 완료, 실행 환경 게이트 대기

기준일: 2026-08-09

기준 브랜치: `refactoring`

이 문서는 Jetson 한 대에 native Robot과 하나 이상의 container 역할을 함께
배치하기 위한 설계·구현·검증 세션의 작업 기록이다. 코드 수정 세션이 끝날
때마다 결정, 변경 파일, 검사 결과와 다음 작업을 이 문서에 누적한다. 이
문서의 목적은 새 마일스톤을 계속 늘리는 것이 아니라, 하나의 확장 작업을
중단 후에도 같은 흐름으로 재개할 수 있게 하는 것이다.

## 1. 이번 작업의 목표

Jetson을 Robot 전용 컴퓨터로 고정하지 않는다. 다만 플랫폼 경계를 없애는
것이 아니라, 한 Host 안에서도 각 Role과 각 설치 backend가 독립성을 유지하게
한다.

최초 지원 목표는 다음과 같다.

- Jetson의 native/systemd `robot`은 계속 독립적인 안전 단위로 유지한다.
- 같은 Jetson에 container/Compose `pilot`과 `ui`를 선택적으로 배치한다.
- `sim`은 ARM64 이미지와 Genesis/Pinocchio/MPC 의존성을 검증하기 전까지
  Jetson 혼합 배치에서 허용하지 않는다.
- 기존 Robot-only 설치와 기존 simulation-only topology의 의미를 깨지 않는다.

## 2. 착수 당시 상태와 발견된 원인

착수 당시 제한은 Jetson의 본질적 능력보다 자료구조가 호스트 단위로 backend를
고정하기 때문에 생긴다.

- `InstallState`는 한 설치에 하나의 `install_mode`만 둔다.
- `ManagedHost`는 한 Host에 하나의 `install_mode`, `lifecycle`, `install_root`,
  `bin_dir`만 둔다.
- `robot`이 포함된 설치는 `robot` 단독 native/systemd여야 한다.
- 연결관리자 GUI도 Robot 블록을 고정하고 다른 역할과 섞지 않았다.
- `environment/containers/Dockerfile.app`의 `sim` 경로는 현재 amd64만
  명시적으로 허용한다.
- Pilot/UI는 ARM64에서의 실제 이미지 build와 런타임 의존성 검증이 아직 없다.

따라서 해결책은 Robot 예외를 한두 군데에서 제거하는 것이 아니다. Host,
Role, 설치·lifecycle 단위를 분리해야 한다.

## 3. 설계 원칙

### 3.1 Host와 DeploymentUnit을 분리한다

하나의 Host는 여러 DeploymentUnit을 가질 수 있다.

```text
Host: jetson-lab
├── NativeUnit: robot
│   ├── backend: native
│   ├── lifecycle: systemd
│   └── own prefix, manifest, security view
└── ComposeUnit: jetson-runtime
    ├── roles: pilot, ui          # 검증된 역할만
    ├── backend: container
    ├── lifecycle: compose
    └── own prefix, project, manifest, security views
```

- **Host**는 컴퓨터의 주소, SSH, 아키텍처, Jetson 여부, Docker, GPU,
  display 같은 capability만 소유한다.
- **Role**은 Pilot/Sim/UI/Robot이라는 논리 endpoint 책임만 표현한다.
- **DeploymentUnit**은 설치 경로, backend, lifecycle, ownership와 그 안의
  역할 집합을 소유한다.
- 동일 Host에 있어도 Role 간 Python 객체 공유나 메서드 호출은 추가하지 않는다.
  통신은 계속 DDS/WebRTC 계약으로만 한다.

### 3.2 설치 경로와 ownership은 Unit별로 분리한다

기존 Robot prefix 안에 Compose 산출물을 억지로 넣지 않는다.

```text
<robot-prefix>/
  roles/robot/
  security/
  bin/elesim-robot

<runtime-prefix>/
  containers/compose.yaml
  roles/pilot/
  roles/ui/
  security/
  bin/elesim-up
```

이렇게 해야 `elesim-update`, `elesim-down`, uninstall, 로그 보존과
ownership manifest가 서로의 파일을 소유하거나 삭제하지 않는다.

### 3.3 Robot 안전 경계를 우선한다

- Robot은 계속 native/systemd이며 container로 바꾸지 않는다.
- Pilot/UI/Sim Compose에는 Unitree ROS 2 환경, Unitree private NIC용 설정,
  bridge socket, 장치 노드를 전달하지 않는다.
- Robot safety/deadman은 Pilot/UI/Sim lifecycle과 독립적으로 동작해야 한다.
- SROS2는 같은 Host라는 이유로 enclave를 합치지 않고 Role별로 유지한다.

## 4. 데이터 모델

기존 topology v1-v3은 읽을 수 있어야 하며, 현재 구현은 schema v4의
호스트 객체에 `units`를 저장한다. v1-v3 입력은 단일 unit으로 정규화하고,
homogeneous legacy mirror는 읽기 호환 경계에만 남긴다.

```json
{
  "schema_version": 4,
  "hosts": [
    {
      "id": "jetson-lab",
      "local": false,
      "jetson": true,
      "dds": {"address": "100.x.y.z", "interface": "tailscale0"},
      "ssh": {"auth_mode": "tailscale", "port": 22},
      "units": [
        {
          "id": "robot-native",
          "install_mode": "native",
          "lifecycle": "systemd",
          "install_root": "/opt/elesim-robot",
          "bin_dir": "/opt/elesim-robot/bin",
          "assignments": [{"role": "robot", "endpoint_id": "robot-go2"}]
        },
        {
          "id": "jetson-runtime",
          "install_mode": "container",
          "lifecycle": "compose",
          "install_root": "/opt/elesim-runtime",
          "bin_dir": "/opt/elesim-runtime/bin",
          "assignments": [
            {"role": "pilot", "endpoint_id": "pilot-main"},
            {"role": "ui", "endpoint_id": "ui-main"}
          ]
        }
      ]
    }
  ]
}
```

현재 불변식은 다음과 같다.

- 각 endpoint Role은 topology 전체에서 정확히 한 번만 배치한다.
- 하나의 Unit 안에서는 하나의 backend/lifecycle만 사용한다.
- `robot`은 native/systemd Unit에만 배치한다.
- `jetson: true`인 Host는 필수 native/systemd Robot Unit을 함께 가져야 한다.
  Pilot/UI를 추가할 때에는 별도 container/Compose Unit으로 둔다.
- Compose Unit에는 capability 검사를 통과한 container 역할만 넣는다.
- Unit마다 절대 경로, ownership manifest, security view를 독립적으로 가진다.
- 고정 Compose 프로젝트명(`elesim-runtime`)을 보존하므로 한 Host에는
  container/Compose Unit을 최대 하나만 둔다. Robot native Unit은 그 옆에
  추가할 수 있다.
- Host의 DDS/SSH 정보는 공유할 수 있지만, endpoint ID와 Role 권한은 공유하지
  않는다.

## 5. 역할별 capability 정책

초기에는 기능을 열어 놓고 나중에 실패시키지 않고, 이미지 build·의존성·장치
검사를 통과한 역할만 GUI에서 배치 가능하게 한다.

| 역할 | Jetson 혼합 배치 초안 | 선행 검증 |
| --- | --- | --- |
| Robot | 항상 native/systemd | 기존 JetPack/Unitree/안전 조건 |
| Pilot | 허용 후보 | ARM64 이미지, PyTorch와 런타임 smoke |
| UI | 허용 후보 | ARM64 이미지, local display 전달과 창 동작 |
| Sim | 당장 금지 | ARM64 Genesis/Pinocchio/MPC 이미지와 GPU 검증 |

모니터 연결은 UI/Viewer capability다. 모니터가 있다고 해서 Sim의 ARM64
의존성 문제가 해결되는 것은 아니다. display 전달은 `DISPLAY`/Wayland/X11
소켓과 권한을 명시적으로 설정하며, 광범위한 `xhost` 허용을 기본값으로
삼지 않는다.

## 6. lifecycle 초안

Unit별 lifecycle을 유지하되 Host 조정자가 순서를 관리한다.

### 시작

1. Compose Unit의 선택된 이미지를 build한다.
2. Pilot/UI를 `--no-build`로 시작한다.
3. Unitree bridge와 Robot native service를 시작하고 native health/safety 상태를 확인한다.
4. DDS descriptor, heartbeat, 필요한 session readiness를 확인한다. 런타임이 먼저
   올라가므로 Robot이 아직 준비되지 않은 동안의 명령은 startup fencing으로
   보류되며, Robot safety는 여전히 local deadman을 따른다.

### 정지

1. Robot native service를 먼저 safe-hold/deadman 경계로 정지한다.
2. Pilot/UI/Sim Compose Unit을 정지한다.
3. Robot service에 묶인 Unitree bridge를 함께 정리한다.

정확한 stop 순서와 timeout은 실제 Robot 안전 검증 전에는 확정하지 않는다.
어떤 순서에서도 Robot local safety가 DDS discovery나 UI 종료에 의존해서는
안 된다.

### 보안 세대 교체

모든 Unit에 대해 다음을 하나의 Host transaction으로 처리한다.

1. Unit별 role-scoped bundle을 stage한다.
2. 현재 실행 중인 Unit/Role을 캡처한다.
3. 모든 관련 Unit을 정지한다.
4. 각 Unit의 `security/current`를 원자적으로 교체한다.
5. 캡처한 Unit/Role만 재시작하고 세대를 검증한다.
6. 어느 Unit이든 실패하면 모든 Unit을 이전 상태로 rollback한다.

CA private key는 계속 operator Authority에만 남긴다.

## 7. 구현 순서 — 이번 작업의 점화 플랜

아래 순서로 진행한다. 각 단계가 끝날 때마다 이 문서의 진행 로그와 검사
결과를 갱신한다.

### A. 경계와 schema

- `DeploymentUnit`/host capability 자료구조를 설계한다.
- topology v3 단일-unit 파일을 새 구조로 읽는 migration을 먼저 만든다.
- 기존 Robot-only와 simulation-only 입력의 의미를 보존한다.
- mixed Jetson의 허용 조합과 거부 이유를 정적 validator로 고정한다.

### B. 설치·ownership

- Robot native prefix와 Compose prefix를 독립적으로 생성한다.
- Unit별 wrapper, manifest, update/down/log 경로를 만든다.
- 기존 uninstall이 다른 Unit을 입양하거나 삭제하지 않도록 ownership 검사를
  확장한다.
- 같은 Jetson에서 두 prefix가 충돌하지 않는지 검사한다.

### C. lifecycle·security

- Compose lifecycle과 Robot systemd lifecycle을 Host 조정자 아래에 둔다.
- build/start/stop/rollback이 Unit별 역할 집합을 정확히 전달하게 한다.
- managed SROS2 stage/switch/verify/rollback을 mixed Host에 맞춘다.
- Unitree bridge와 Robot safety가 Container Unit의 lifecycle과 섞이지 않는지
  회귀 테스트를 추가한다.

### D. capability·이미지

- 호스트 아키텍처, Docker, GPU, display와 역할별 이미지 지원 여부를
  preflight에 노출한다.
- Pilot ARM64 build/runtime 검증을 먼저 한다.
- UI ARM64와 display 전달을 검증한다.
- Sim은 별도 ARM64 검증이 끝날 때까지 명시적으로 거부한다.

### E. 연결관리자 UI

- Jetson 카드 안에 `Robot native`와 `Container runtime` lane을 표시한다.
- 역할을 lane에 배치할 때 capability와 backend를 함께 검증한다.
- install root/bin dir를 Host 단일 값이 아니라 Unit별 입력으로 바꾼다.
- 저장/배포/전체 시작 로그가 Host와 Unit을 구분해 표시되게 한다.

### F. 검증

- topology/state migration과 invalid combination 테스트
- mixed Host의 두 prefix ownership 테스트
- Unit별 build/start/stop/rollback fake lifecycle 테스트
- role-scoped SROS2 bundle 격리 테스트
- Pilot/UI ARM64 이미지 build 및 최소 runtime smoke
- 실제 Jetson에서 Robot 안전, DDS, UI display를 수동 확인

## 8. 이번 작업의 비목표

- Router/ZMQ 재도입
- Robot을 Docker로 이전
- Unitree DDS를 Elesim DDS graph에 노출
- 기존 DDS contract를 typed service/action으로 전환
- Jetson을 자동으로 발견해 역할을 임의 배치
- ARM64 검증 전 Sim을 허용
- 실제 Jetson 수동 검증을 자동 테스트 통과로 대체

## 9. 진행 로그

| 날짜 | 단계 | 변경/결정 | 검사 | 다음 작업 |
| --- | --- | --- | --- | --- |
| 2026-08-09 | 초안 | Host와 DeploymentUnit 분리, Robot+Pilot/UI를 1차 목표로 결정 | 코드 수정 없음 | A: schema와 불변식 설계 |
| 2026-08-09 | A/C 착수 | `DeploymentUnit`을 도입하고 기존 topology v1-v3를 단일 unit으로 읽도록 유지했다. Host는 여러 unit을 가지며 Robot은 Jetson/native/systemd, 나머지는 container/Compose로 검증한다. mixed Host의 stage/activate/rollback/status/build/start 경계를 unit별로 분리하고 SROS2 bundle을 unit 역할별로 필터링했다. | Python compile 및 mixed topology 수동 round-trip 통과; 전체 gate는 Docker daemon 부재로 아직 미실행 | UI를 일반 COM Host 카드 4개와 Robot drag/drop으로 전환하고 unit 입력/회귀 테스트를 추가 |
| 2026-08-09 | E 완료 | 연결관리자의 모든 COM 카드를 동등한 Host 카드로 만들고, 각 카드에 `Container runtime unit`/`Robot native unit` lane을 추가했다. Robot은 Jetson native lane에서만, 현재 amd64 전용 Sim은 Jetson에 놓지 못하게 drag/drop·서버 validator 양쪽에서 막는다. runtime/Robot prefix와 endpoint ID는 계속 unit별로 저장한다. | `node --check`, Python compileall, `git diff --check`, mixed model·role-scoped bundle 수동 probe 통과; Docker 기반 pytest gate는 Docker daemon 권한 부재로 대기 | lifecycle·security fake 회귀와 generated install/release gate 실행; 이후 실제 Jetson 수동 인수시험 |
| 2026-08-09 | B/C/D/F 마감 | Unit별 Compose/native 명령, prefix·bin dir·security root·role-scoped bundle·세대 rollback map을 분리했다. 한 Jetson의 필수 Robot native와 Pilot/UI Compose를 같은 Host 카드에서 독립적으로 저장·검사·시작·정지·복구한다. Jetson 표시는 mandatory Robot unit을 요구하며, 빈 COM 카드와 로컬 Authority의 Robot 배치를 GUI에서 즉시 거부한다. 기존 homogeneous schema-v3 mirror와 legacy Robot card 읽기도 유지했다. | mixed topology/lifecycle/security probe PASS; i18n 110-key parity, JavaScript syntax, Python compileall, `git diff --check` PASS. `python3 misc/tools/quality/check.py --group required`는 host에 pytest가 없고 Docker daemon 권한도 없어 실행환경 게이트에서 중단됨. | 실제 Jetson에서 두 prefix/ownership, ARM64 Pilot/UI 이미지, native safety, DDS/SROS2/WebRTC/display를 수동 인수시험으로 확인 |
| 2026-08-09 | 최종 감사 | Jetson의 Robot 필수 조건을 자료구조와 저장 직전에 이중 검증하고, mixed Unit의 `install_root`/`bin_dir`가 같거나 서로 포함되지 않도록 고정했다. 레거시 `public/` trust material은 role view에 유지하되 role별 enclave는 계속 격리한다. | mandatory-Robot·non-overlap·mixed round-trip/security/lifecycle probe PASS; i18n 110-key parity, Python compileall, JavaScript syntax, `git diff --check` PASS. Required gate는 host `pytest`/Docker daemon 부재로 동일하게 차단됨. | 자동 게이트가 가능한 Elesim 개발 컨테이너에서 required/extended 및 실제 Jetson 수동 인수시험 실행 |

## 10. goal 지정용 요약

다음 goal은 아래 문장으로 시작할 수 있다.

> Jetson 한 대에 native Robot과 검증된 container Pilot/UI를 함께 배치할 수
> 있도록 Host/Role/DeploymentUnit 경계를 분리하고, 기존 Robot-only·simulation-
> only 동작을 보존하면서 topology migration, Unit별 ownership/lifecycle/
> SROS2 배포, 연결관리자 UI와 회귀 테스트를 단계적으로 구현하라. Sim의
> ARM64 지원은 별도 capability 검증 전까지 거부하라.

이 goal의 완료는 소프트웨어 검증 완료를 뜻한다. 실제 Jetson 안전·display·
DDS·성능 검증은 별도의 수동 인수시험으로 기록한다.
