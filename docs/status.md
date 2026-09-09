# 구현 상태와 수용시험

갱신일: 2026-09-07. 이 문서만 마일스톤, 현재 완료 범위, 미해결 항목, 수동
acceptance gate를 소유한다. 구현 불변식은 `architecture.md`, wire 계약은
`dds_contracts.md`, 운영 절차는 `setup.md`와 `deployment.md`를 따른다.

## 현재 목표: 기존 기능의 운영 경로 완결

### 다중 system 실행 설계 초안 (계획, 미구현)

목표는 **한 host 설치와 고정 `elesim-runtime` Compose project를 공유하면서**
서로 다른 `system_id`의 EleSim graph를 동시에 실행하는 것이다. 새 edition이나
두 번째 전체 설치, system별 Compose project는 만들지 않는다. `system_id`를 별도
instance ID 없이 실행 instance의 정본 key로 사용한다. 이 절은 후속 구현의 초안이며
현재 container 이름, 상태 schema 또는 명령 동작이 이미 바뀌었다는 뜻이 아니다.

상태와 자원의 소유 범위는 다음 세 층으로 나눈다.

| 범위 | 소유할 값과 자원 |
| --- | --- |
| host 설치 | prefix/bin, source revision, Docker context/Engine ID, 설치된 role capability, 공통 image/build cache, `elesim-dev`, Tailscale sidecar, 전체 ownership manifest |
| system instance | `system_id`, 이 host의 `assigned_roles`, DDS/compute/TURN 설정, role별 생성 config와 SROS2 view, 로그, 실행 상태 |
| graph topology | host와 role 배치, DDS/SSH endpoint, endpoint ID, SROS2 Authority generation과 배포 transaction |

제안된 설치 경계는 다음과 같다. 정확한 파일명과 schema는 B1에서 focused test와
함께 확정한다.

```text
<prefix>/
  install-state.json
  containers/compose.yaml
  instances/<system_id>/
    state.json
    apps/<role>/config/
    security/
    secrets/
    cache/
    logs/
  connections/<system_id>/topology.json
  authority/<system_id>/
```

Compose project 이름은 계속 `elesim-runtime`이다. 공통 `dev`, tools와 Tailscale
service는 한 번만 생성하고, application service key는
`instance-<system_id>-<role>`처럼 system을 포함한다. application container의
전역 `container_name`은 제거하고 Compose가 이름을 만들게 하며,
`io.elesim.install_uuid`, `io.elesim.system_id`, `io.elesim.role` exact label로
소유권을 검증한다. image는 설치 단위에서 공유하되 instance 삭제가 공통 image를
삭제하지 못하게 한다.

운영 명령은 기존 wrapper에 `--system <system_id>`를 추가하는 방향으로 유지한다.
instance가 하나뿐이면 인자를 생략할 수 있지만, 둘 이상이면 생략을 거부한다.
생략을 `--all`로 해석하지 않는다. instance stop/remove는 선택한 service만
대상으로 하며 project 전체 `docker compose down`은 명시적인 전체 제거 또는
host uninstall에만 허용한다. instance lifecycle에서는 `--remove-orphans`도
사용하지 않는다. aggregate Compose 파일은 등록된 모든 instance service를 항상
포함하고, host helper는 선택한 system에서 파생된 exact service key만 허용한다.
서로 다른 system의 연결관리자는 동시에 실행할 수 있지만 같은 system의 저장·보안
배포는 system별 lock으로 직렬화한다.

DDS application topic과 SROS2 policy는 이미 `system_id` namespace를 사용하므로
이 기능만을 위한 wire protocol version 변경은 계획하지 않는다. 각 graph의
`system_id`는 설치 내에서 유일해야 한다. 초기 구현은 같은 Docker Engine에서
동시에 활성인 system의 DDS domain도 서로 다르게 요구해 discovery 간섭을 줄인다.
이는 운용 격리 규칙이지 보안 경계가 아니며, 보안 경계는 계속 SROS2 enforce다.

Tailscale sidecar와 개발 attachment는 host 공용으로 유지한다. managed Coturn은
host/Tailscale network namespace의 listen/relay port가 충돌하므로 instance별
고정 port와 겹치지 않는 relay 범위를 설치 상태에서 할당하는 방향을 우선 검토한다.
외부 TURN은 instance별 credential 경로를 가진다. GPU 선택과 writable runtime
경로도 instance 설정으로 내려 같은 host의 두 Sim이 설정 파일을 공유하지 않게 한다.
공통 mutable `:local` image를 동시에 build하지 않도록 host 설치 단위 build lock과
context fingerprint를 유지한다. instance 제거는 image를 삭제하지 않고 전체 host
uninstall만 공통 image 제거를 소유한다.

물리 Robot은 host당 하나의 native 안전 경계와 고정 systemd lifecycle을 유지한다.
한 Robot host에서 두 system이 Robot을 동시에 활성화하는 것은 거부한다. 초기
완료 범위는 여러 Robot 없는 graph의 동시 실행과, Robot을 포함한 graph
하나가 별도 Robot 없는 graph와 공존하는 경우까지다. templated systemd나
하나의 물리 Robot을 여러 graph가 공유하는 기능은 요구가 생기기 전에는 만들지 않는다.

#### 다중 system 마일스톤

| ID / 상태 | 결과 | 완료 증거 |
| --- | --- | --- |
| B0 계약 / 초안 | 현재 단일 instance 동작과 목표 경계를 구분한다 | 상태/명령/소유권/DDS/Robot/Coturn 결정과 충돌 회귀 목록 |
| B1 상태 분리 / 미착수 | host 설치와 system instance가 독립적으로 저장된다 | schema migration; 기존 설치가 기존 `system_id`의 한 instance로 손실 없이 이관 |
| B2 Compose namespace / 대기(B1) | 한 project에 같은 role의 여러 service가 존재한다 | service/config/label 충돌 검사와 생성 Compose isolation test |
| B3 lifecycle / 대기(B2) | 한 system의 up/down/status/logs가 다른 system을 건드리지 않는다 | 두 Robot 없는 instance 동시 실행, 한쪽 stop/remove/update 후 다른 쪽 생존 |
| B4 연결·보안 / 대기(B3) | topology와 SROS2 transaction이 system별로 독립적이다 | 두 manager 동시 실행, generation/rollback/실패 journal 교차 오염 없음 |
| B5 공용 인프라 / 대기(B3) | dev/Tailscale/image를 공유하며 TURN/GPU 자원 충돌을 거부하거나 할당한다 | sidecar 유지, image ownership, Coturn port/range, GPU/writable path 회귀 |
| B6 통합 수용 / 대기(B4,B5) | 여러 graph의 제어·RGBD·WebRTC·종료가 격리된다 | 같은 host와 multi-host software smoke, SROS2 교차 publish/subscribe 거부, Robot 독점 검사 |

B0에서 먼저 고정할 위험 회귀는 세 가지다. `down`/`--remove-orphans`가 다른
system을 제거하지 않을 것, 연결관리자의 전역 `elesim-manager` 이름을 없애고
system별 transaction lock을 사용할 것, `<prefix>/apps/<role>`와 공통 Sim cache를
instance 경로로 옮길 것이다. 현재 topology의 "host당 Compose runtime unit 하나"
제약은 공통 aggregate project/설치 unit과 모순되지 않으므로 이 이유만으로 풀지 않는다.

첫 vertical slice는 한 host에 `lab_a`와 `lab_b`라는 두 Robot 없는
instance를 만들고 Pilot/Sim/UI를 모두 동시에 띄운 뒤, `lab_a`만 내렸을 때
`lab_b`의 process와 설정·DDS 상태가 그대로 남는 것이다. 이 slice가 통과하기
전에는 multi-host, managed Coturn 자동 할당이나 Robot 전환 UI를 확장하지 않는다.

### 연결관리자 COM 편집 화면 개편 (2026-09-08)

- 고정된 세로형 COM 카드와 `미사용` 토글을 제거했다. 실제 등록된 COM만 세로로
  나열하고, 각 COM 내부는 네트워크(IP/interface) / 역할 drag-and-drop / 필요한
  경우의 SSH 설정을 가로 3열로 표시한다. 좁은 화면에서는 두 열과 한 열로 접힌다.
- 설치 경로는 좌측에서 상시 표시한다. `이 컴퓨터가 연결 관리자를 실행함`은
  `이 컴퓨터는 운영용임`으로 바꿔 우측 SSH 제목 옆에 배치했고, 운영용 COM에서
  SSH 입력 전체를 숨겨 의미 없는 안내 공간도 없앴다. SSH 사용자명은 비어 있는
  값으로 시작한다. 역할 열을 이전 시안보다 줄이고 그 폭을 SSH/private-key 열에
  넘겨 긴 인증 경로를 더 여유 있게 표시한다.
- `+ COM`은 `컴퓨터 추가`로 바꿨고 dialog에서 일반 컴퓨터와 Robot 컴퓨터
  (Jetson)를 구분해 추가한다. 전역 역할 추가 버튼 대신 각 COM 역할 영역 우상단의
  `[+]`가 그 COM에 역할을 바로 추가한다. 편집 보드의 역할 카드 개수에는 제한이
  없고, 번호는 모든 COM을 합쳐 역할별로 독립 증가한다(`UI 1`, `UI 2`, `Sim 1`).
  한 줄에는 카드 세 개를 놓으며 drop zone은 기본 세 줄과 하단 drop 여백을 항상
  보여준다. 카드 drag 중 viewport 상·하단에서는 연속 자동 스크롤하고, 삽입
  위치에는 보이지 않는 grid placeholder를 넣고, 마우스 커서가 들어간 카드의
  칸을 선택한다. 기존 카드는 FLIP 이동 애니메이션으로 다음
  열/행으로 물러난다.
- 각 컴퓨터 헤더의 `COM1`, `COM2` 이름은 편집할 수 있으며 별도 host ID 입력란은
  제거했다. 검증된 컴퓨터 이름이 topology의 host ID가 된다. 현재 편집 버튼에는
  연필 자산이 준비될 때까지 기존 코끼리 그림을 사용한다. 헤더의 `▲`/`▼`는 화면과
  저장되는 host 배열의 순서를 함께 바꾼다.
- 실행 모드 선택을 제거하고 topology schema를 v5로 올렸다. 1–4개 COM과 실제
  역할 카드가 graph를 직접 정의하며 Pilot/Sim/UI/Robot의 고정 집합을 강요하지
  않는다. v1–v4는 v5로 이관하고 기존 `topology_mode`는 검증 후 폐기한다. 복수
  역할 카드는 endpoint ID와 함께 저장할 수 있지만, instance별 Compose/service
  namespace가 생기기 전까지 실행 작업은 명시적으로 거부한다(B2).
- 새 역할 카드의 endpoint ID는 역할별 `pilot-1`, `sim-1`, `ui-1`, `robot-1`
  형식으로 번호를 붙인다. Robot COM의 역할 영역은 세로로 두 zone을 쌓지 않고,
  같은 높이 안에서 일반 역할 2/3와 고정 Robot 1/3을 좌우로 배치한다. 일반 역할
  카드는 좌측 두 열, Robot은 우측 한 열을 사용한다. 역할 추가 안내는 실제로
  카드를 추가하는 좌측 일반 역할 zone에만 표시한다.
- 상단의 `네트워크: 로컬/인터넷` 선택은 제거했다. 연결관리자는 명시된 COM 주소를
  이미 알고 있으므로 저장되는 topology에는 항상 static discovery를 사용한다.
  COM별 DDS 주소와 Interface 입력은 그대로 유지한다.
- setup 전체 suite **613 passed**. canonical `elesim-dev`는 Docker socket
  권한 거부 및 wrapper 부재로 사용할 수 없어 승인된 host localhost/socket 실행을
  사용했다. 정식 전체 gate와 실제 브라우저·다중 host 수용시험은 별개다.

### 설치 역할과 topology 배정 분리 (2026-09-07)

- 설치 상태 schema v11에서 설치 capability인 `roles`와 현재 graph 배정인
  `assigned_roles`를 분리했다. 기존 v1-v10 상태는 `assigned_roles=null`로
  이관되어 설치 역할 전체를 쓰는 동작을 보존한다.
- 연결 관리자는 `assigned_roles`가 설치 역할의 부분집합이면 허용한다. DDS/XML,
  Compose 환경, endpoint identity, SROS2 app view와 start/stop/build는 현재 배정
  역할만 대상으로 한다. 비활성 역할의 설치 파일과 image는 보존한다.
- 재배정 시 실행 중인 비활성 역할이 있으면 설정 변경 전에 거부한다. 기본
  `elesim-up`은 상태 파일의 최신 배정을 매번 읽고, 명시적으로 비배정 역할을
  요청해도 거부한다. Pilot만 시작할 때 별도로 실행 중인 Sim/Coturn을 끄지 않는다.
- 동일 host에서 여러 graph를 동시에 실행하는 instance namespace 분리는 이번
  변경에 포함하지 않았다. Compose project/container 이름은 계속 고정이다.
- `elesim-dev`는 Docker daemon에 존재하지 않아 canonical container gate를
  실행하지 못했다. 호스트 setup suite를 소켓 허용 구간과 일반 구간으로 나눠
  **612 passed**로 확인했고 bootstrap **73 passed**, 관련 상태/배포/실행 회귀
  **83 passed**를 재확인했다. required gate의 Protocol 131, UI 67과 DDS RGBD 2는
  통과했지만 host 환경의 Unitree socket 권한, Genesis, ROS 2, aiortc/av와 wheel
  build backend 부재로 Robot 일부, Sim, topology, WebRTC 및 isolated release는
  canonical 성공 판정하지 않는다.

### UI·Robot 호출 경로 정리 (2026-09-07)

- UI 영상 재연결 성공/실패에 중복된 backoff 계산을 통합했다. 재시도
  횟수 16, 간격 5초 상한, 다른 영상·DDS session 보존과 응답 후 초기화를
  반복 실패/성공 회귀로 확인했다. 사용하지 않는 panel 상태 하나도 제거했다.
- Robot 실제 entrypoint/device 호출에 없는 tick 변환, Sim 속도 추정,
  midpoint 이동과 IPC 빈 로그 메서드를 제거했다. 카메라의 미사용 물체
  위치 추정기와 전달 모듈을 없애고 RealSense intrinsics는 기존 공통
  `RgbdIntrinsics`를 직접 사용한다. 실제 bridge 로그와 모드별 안전 정지는 유지한다.
- UI의 기존 sag 테스트는 저장소에 없는 preset 폴더를 가정해 실패했다.
  테스트 전용 임시 config에서 폴더 유무와 상대 경로 fallback을 검증하도록
  수정했으며, 이를 위해 불필요한 source 폴더를 만들지 않았다.
- dev container/래퍼 부재를 재확인했다. 호스트 UI 전체 **67 passed**,
  Robot 전체 **102 passed + 2 subtests**. Robot socket/peer-credential 검사는
  sandbox 권한 거부 후 승인된 임시 UDS/가짜 장치 실행으로 통과했다.
  실제 하드웨어나 정식 컨테이너 검증을 대신하지 않는다.
- 이번 runtime **143줄 순감**, 이전 정리 포함 **1,466줄 순감**(model 이동 제외).
  삭제된 source는 Git HEAD에서 복구 가능하며 외부 데이터/설치물은 그대로다.
  Pick/Sim scientific 회귀, isolated release와 실제 운영 경로 검증은 여전히 남는다.

### 설치·연결 군살 제거 후속 기록 (2026-09-07)

- 실제 frontend 호출과 host 실행 경로를 Luna 두 작업과 주 에이전트가
  교차 검수했다. 설치 마법사의 미사용 SSH fingerprint API와 파일 선택
  분기를 제거했다. 설치 경로 선택·root 제한 및 연결관리자의 SSH 기능은 유지한다.
- CLI preset은 사용되지 않는 이름/설명/remote metadata 클래스 대신 role
  tuple 표로 축소했다. 기존 CLI profile 이름·역할·custom 검증은 유지한다.
  내부 Python `Profile` export와 미사용 updater `pull_services` 인자는 제거했다.
  일반 update는 여전히 build만 하고 전용 Tailscale update만 pull/recreate한다.
- 상태 poll의 `compose config --services`는 container unit당 최대 세 번에서
  한 번으로 줄였다. 다음 poll에는 다시 읽는다. configured/created/running
  구분과 조회 오류, Coturn/Tailscale readiness 판정을 보존했다.
- 호출자가 없는 `_switch_only`와 테스트만 거치던 forwarding helper 둘을
  제거했다. generation activate/rollback, legacy ownership cleanup, host
  helper와 runtime-tools namespace 경계는 실제 사용하므로 남겼다.
- 이번 setup runtime 순감 **124줄**, 앞선 runtime 정리와 합쳐 **1,323줄**
  순감(model 이동 제외). 설치물·로그·외부 데이터 변경 없이 소스만 정리했다.
- dev container/래퍼 부재를 재확인한 뒤 호스트 부분 검증:
  `PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=payload/runtime/docker/setup/app:payload/runtime/common/protocol python3 -m pytest -p no:cacheprovider workbench/tests/setup -q --tb=short`
  → **600 passed**. 임시 HTTP/Unix socket 테스트는 sandbox 권한 거부 후
  승인된 실행으로 재검증했다. JS `node --check`와 `git diff --check`도 통과.
- 정식 required/extended, isolated release, real-RMW/GPU/실물 gate는 아직
  미실행이다. 다음은 개발 환경 확보 후 앞서 막힌 Pick/Sim 회귀와 실제
  UI→Pilot→Sim 조작·실패·취소 경로를 검수한다. 전체 감사/R 완료를 주장하지 않는다.

### 2026-09-07 로직 정리 중간 기록

- Luna 세 작업으로 Pilot workflow, perception, tracing 호출 경로를 나누어
  조사했다. 사용량 제한 이후 남은 UI 조건문과 연결부는 주 에이전트가 검수했다.
- 항상 false인 Pick/Gaze 원격 위임과 예외만 발생시키던 client 명령을
  제거했다. Perception도 Pilot 실행으로 일원화했다. 실제 RGBD 수신과
  observation 전달, hardware LJI 차단 및 로컬 stop 경계는 유지한다.
  이전 `provider=host`/`run_local=false` 설정은 명시적으로 거부한다.
- UI의 snapshot 캐시로 처리하는 읽기 명령을 원격 operator 허용 목록에서
  제거했다. wire/view 버전 유지 및 구형 custom client 거부 결정은
  `dds_contracts.md`에 기록했다.
- Pilot/Sim tracing 중복 및 Robot 전달 facade를 기존 공통 tracing으로
  통합했다. 로컬 JSONL은 Pilot/Sim의 명시적 tracing 활성화에만 생성하며
  파일당 10 MiB, 활성 파일 포함 네 개로 회전한다. Sim의 주기적 성능
  출력 기본값은 껐으며 명시적 설정으로 다시 켤 수 있다.
- 기존 model 이동을 제외한 runtime diff는 약 1,200줄 순감이다. 제거한
  소스는 Git HEAD로 복구할 수 있으며 설치물·로그·장비는 삭제하지 않았다.
- 재확인: `docker ps -a --filter name=elesim-dev` 결과가 비어 있고,
  `elesim-dev`/`elesim-up` 래퍼도 PATH에 없다. 기존 다른 checkout 소유
  runtime을 변경하거나 host 의존성을 설치하지 않았다.
- **호스트 부분 검증만 수행:** protocol 전체, Pilot operator, UI
  operator proxy/session, Pilot perception YAML ownership, 공통 tracing
  회귀 묶음은 159 passed + 5 subtests. 회전 한도, 기록 경로 실패,
  consumer trace carrier, JSON-only sampling도 포함한다.
- `test_control_client.py`, `test_perception_config_update.py`,
  `scenarios/pick/test_75_mobile_pipeline.py`, Sim `test_config.py`는
  `ModuleNotFoundError: scipy`로 수집 실패했다. required/extended 전체,
  isolated release, real-RMW topology 및 GPU/실물 경로는 이번 revision에서
  검증하지 못했다. 기존 성공 기록으로 대신하지 않는다.
- 다음 검수: 정식 개발 attachment에서 위 네 회귀와 전체 gate 실행;
  설치·연결관리자의 GUI→작업 실행→실패/rollback 흐름을 계속 추적한다.
  legacy state/ownership migration, 미연결 typed ROSIDL은 사용자가 없다는
  추정만으로 삭제하지 않았다. 전체 감사와 R 마일스톤은 아직 미완료다.

기능을 추가하기 전에 설치 → 연결 → 표출 → 조작 → 작업 수행 경로를
검증한다. 아래 `R0`–`R5`는 운영 수용 마일스톤이다. 과거 구현 단계의
`M1`, `M2-A`, `M2-B` 완료 기록과 별개이며, 기존 테스트 통과 기록으로
자동 완료 처리하지 않는다. 현재 완료된 R 마일스톤은 없다.

프론트는 설치마법사, 연결관리자, UI, Sim 표출이다. 백은 설치·연결 실행부,
Pilot, Robot과 Sim의 가상 장치·물리 실행부다. 이는 감사할 책임의 구분이며
패키지 이동이나 새로운 서비스 분리를 지시하지 않는다.

### 범위와 대표 환경

- 첫 목표는 **R1 설치 + R2 연결·표출**이다. R0는 이를 위한 검증 준비다.
- 기본 수용 경로는 Robot 없는 Pilot/Sim/UI 각 하나다. 우선 한 host의
  native Docker Engine과 실제 GPU/display에서 확인한다. 이는 제안된 기준
  환경이며 현재 장비가 확보됐다는 뜻이 아니다. 실제 첫 실행 전에 host,
  GPU, OS, Docker backend, interface, 보안 profile을 기록해 고정한다.
- 이후 R2에서 UI host와 Pilot/Sim host를 분리한 실제 두-host 경로를 확인한다.
  한-host만 통과하면 R2는 부분 완료다. DDS와 SSH 주소를 각각 기록한다.
- 소유 LAN/제한된 routed VPN은 `trusted-network`를 사용할 수 있다.
  공유망을 사용하면 해당 실행 전 SROS2 enforce 검증이 선행 조건이다.
- Docker Desktop/WSL, TURN relay, CPU-only, 다른 카메라 profile 등은 별도
  환경 gate다. 대표 환경 통과를 이 환경들의 지원 증거로 확대하지 않는다.
  실제 사용 장비가 해당 환경이면 그 gate를 현재 마일스톤의 선행 조건으로 올린다.
- Pick·Gaze·Wrap 중 작업 하나를 선택하는 일은 R4 착수 조건이다. 지금 모두를
  완성 대상으로 삼거나 사용 여부를 추측해서 삭제하지 않는다.

### 마일스톤과 완료 조건

| ID / 상태 | 사용자에게 보장할 결과 | 완료에 필요한 증거 |
| --- | --- | --- |
| R0 검증 준비 / 미착수 | 동일 revision에서 실패를 재현할 수 있다 | 정식 개발 attachment, gate 기준선, 미해결 실패 목록, 대표 환경 기록 |
| R1 설치 / 미착수 | 마법사에서 설치를 끝내고 설치 상태를 다시 확인한다 | 설치·재진입·입력 실패·취소·동일 조건 재시도; 생성물과 표시 상태 일치 |
| R2 연결·표출 / 미착수 | 연결관리자로 시작해서 UI의 두 영상과 상태를 본다 | 한-host 및 두-host 기동, 실제 frame 갱신, 종료·재시작, 한 peer/영상 중단 표시 |
| R3 기본 조작 / 대기(R2) | 조작이 실제 상태에 반영되고 중단하면 멈춘다 | UI→Pilot→Sim 명령과 telemetry 왕복, lease 상실·reset·재접속 회귀 |
| R4 작업 하나 / 대기(R3, 작업 선택) | 선택한 작업을 성공·실패·취소 후 다시 실행한다 | 고정 입력의 반복 실행, 작업별 성공 기준, 실패 사유와 재시도 결과 |
| R5 실물 / 대기(R3, 장비) | 기본 조작과 로컬 안전이 Robot에서 성립한다 | Jetson/GO2/arm 실측, bridge/통신 상실 시 정지와 cleanup, 장치 피드백 |

R5의 기본 장치 검증은 R4 알고리즘 완성을 기다릴 필요가 없다. 다만 R4에서
선택한 작업의 **실물 성공**을 주장하려면 R4와 R5 양쪽 증거가 필요하다.

**R0 — 검증 준비**

1. 설치 UUID, prefix, Docker context/Engine과 Compose 소유권을 확인하고
   setup-generated `elesim-dev`를 사용할 수 있게 한다. 실행 중인 다른
   prefix의 `elesim-runtime`을 임의로 교체하지 않는다.
2. 아래 required/extended gate와 release build/verify를 정확한 revision에서
   실행한다. 실패를 제품 결함, 테스트 결함, 환경 제약으로 분류한다.
   원인을 모르는 실패는 그대로 미해결로 남긴다.
3. 후속 milestone의 테스트를 막는 실패를 해소하고, 나머지 실패는 재현 명령,
   영향 범위, 처리할 R 단계와 함께 기록한다. 부분 통과는 전체 gate 통과가 아니다.

**R1 — 설치**

- 같은 revision의 bootstrap → 마법사 → 검증 → 설치 완료 → 새 shell의
  상태 명령까지 실행한다. source config 수동 수정 없이 prefix가 완성돼야 한다.
- 설치 완료는 파일/context 생성 완료를 뜻한다. image build와 runtime 기동은
  R2에서 판단한다. 화면도 이 차이를 표시해야 한다.
- 잘못된 입력은 수정 가능한 사유를 내고, 취소·실패 후 같은 조건으로 다시
  시도할 수 있어야 한다. 상태 파일과 실제 생성물을 함께 확인한다.
- 재설치/update/uninstall은 전용 검증 prefix에서 확인한다. update의 실행 중
  역할 보존과 manifest 기반 제거 경계를 검증하고 외부 checkout은 보존한다.

**R2 — 연결·표출**

- 연결관리자에서 topology 저장 → 설정 적용 → 전체 시작을 수행한다.
  build, process 실행, DDS discovery, Sim scene/session, 영상 준비의 상태와
  실패 사유를 구별해 볼 수 있어야 한다.
- observer/hand-eye 각각 실제 디코드 frame 갱신을 확인한다. ICE 연결이나
  검은 fallback frame만으로 통과하지 않는다. 창 닫기·재열기와 시점 조작도 확인한다.
- 대표 환경별 시작→표출→종료를 3회 반복하고, 한 번은 10분간 두 영상을 관찰한다.
  이는 초기 회귀 수용 기준이며 장시간 안정성 보증이 아니다. 기동 시간,
  frame age, FPS, stall/recovery를 기록한다. 성능 합격 수치는 첫 측정 후
  용도에 맞게 정하고 별도 실행에서 검증한다.
- 한 peer 종료와 한 영상 중단을 주입한다. UI가 stale/실패를 표시하고, 재시작
  후 회복하거나 필요한 수동 조치를 구체적으로 알려야 한다.

**R3 — 기본 조작**

- 자동 Pick/Gaze/Wrap을 켜지 않고 관절·그리퍼·GO2 기본 명령의 실제 적용과
  telemetry를 확인한다. ACK 수신만으로 이동 완료를 판정하지 않는다.
- 중단, Sim pause/reset, Pilot 종료, target 변경 때 이전 작업과 명령이
  되살아나지 않는지 확인한다. pause 중에도 상태·세션 관리는 살아 있어야 한다.
- 정상 조작과 장애 주입을 각각 3회 반복한다. 명령/피드백 오차와 정지 시간은
  해당 설정의 limit/deadman/lease 기준과 대조하고 revision·설정값을 함께 남긴다.

**R4 — 작업 하나**

- 실제 필요한 작업 하나와 입력·장면·성공 조건을 먼저 고정한다. 예를 들어
  mock attach, 물리 파지, 물체 들어 올리기는 서로 다른 성공 조건이다.
- 선택한 작업만 UI 진입점부터 Pilot 계산, Sim 결과까지 추적한다. 성공 사례뿐
  아니라 입력 없음/도달 불가/중단을 포함해 상태와 재실행 가능성을 확인한다.
- 시험 횟수·성공률·허용 오차·시간 제한을 작업 착수 때 정하고 성공 사례만
  선별하지 않는다. 다른 자동 작업은 후속 후보로 남긴다.

**R5 — 실물**

- 장비 담당자가 확보된 환경에서 두 systemd unit, 전용 계정/UDS credential,
  private Unitree NIC/domain, 장치 방향·limit와 실제 피드백을 확인한다.
- Pilot/bridge 상실, 잘못된 packet, deadman 만료 시 실제 정지 시간을 측정한다.
  GO2 경로 실패 중에도 arm safe-hold·torque-off·cleanup을 확인한다.
- Sim 성공이나 mock test는 물리 안전 증거가 아니다. 장비/현장 검증이 없으면
  R5는 대기 상태를 유지한다.

### 코드와 기존 테스트 진입점

아래 경로는 저장소 루트 기준이다. 실행 suite와 허용 import 경로는
`workbench/tools/quality/check.py`를 정본으로 사용한다.

| 단계 | 코드 진입점 | 기존 focused test / 증거의 한계 |
| --- | --- | --- |
| R1 | `payload/runtime/docker/setup/app/elesim_setup/service.py`, `gui.py`, `container_installer.py` | `workbench/tests/setup/test_gui.py`, `test_container_installer.py`, `test_uninstall.py`; 생성물·mock 검증은 실제 설치 증거가 아님 |
| R2 | `payload/runtime/docker/setup/app/elesim_setup/connections.py`; `payload/runtime/docker/ui/app/elesim_ui/sim_session.py` | `workbench/tests/setup/test_connections.py`, `workbench/tests/apps/ui/test_sim_session.py`; fake peer/receiver 통과는 실제 두-host 영상 증거가 아님 |
| R3 | `payload/runtime/docker/pilot/app/elesim_pilot/operator.py`; `payload/runtime/docker/sim/app/elesim_sim/endpoint.py` | `workbench/tests/apps/sim/test_endpoint.py`, `workbench/tests/protocol/test_peer_authority.py`; 명령·권한 검증은 실제 이동/정지 측정과 별개 |
| R4 | 선택한 Pilot 작업의 UI 호출과 `payload/runtime/docker/pilot/app/elesim_pilot/pick/` 또는 `gaze/` | Pick 선택 시 `workbench/tests/apps/pilot/headless/test_pick_workflow.py`, `workbench/tests/apps/pilot/test_pick_stop_lifecycle.py`; stub phase 검증은 perception·IK·파지 성공 증거가 아님 |
| R5 | `payload/runtime/native/robot/app/elesim_robot/main.py`, `runtime.py`, `go2/unitree_bridge_daemon.py` | `workbench/tests/apps/robot/test_main_lifecycle.py`, `test_unitree_ipc.py`; local UDS/fake backend는 GO2 장치 검증과 별개 |

표의 짧은 파일명은 같은 셀에서 앞서 명시한 디렉터리 기준이다.
`workbench/tests/system/smoke_topology.py`는 네 role의 실제 RMW process probe다.
`test_dds_rgbd.py`, `test_webrtc_media.py`는 같은 system 디렉터리에 있으며
실제 생성된 영상, GPU, 원격망과 TURN relay를 포괄하는 수용시험은 아니다.

### 현재 알려진 검증 공백과 첫 작업

2026-09-05/06 세션 기록에 근거한 시작점이며, 현재 revision의 재시험 결과가 아니다.

- 정식 `elesim-dev`가 없었고 `elesim-up`/`elesim-dev`가 PATH에서 발견되지 않았다.
  당시 실행 중인 프로젝트는 `/home/user/ws/newsim/containers/compose.yaml` 소유였다.
  R0에서 실제 설치 상태를 다시 조사하고 소유권이 확인된 경로로 준비한다.
- 호스트 SciPy 미설치로 Pilot/Robot/model 테스트가 수집되지 않았다.
  release wheel 검증은 setuptools 59.6.0의 `UNKNOWN/0.0.0` 생성으로 실패했다.
  호스트에 의존성을 추가하지 말고 정식 환경에서 재시험한다.
- setup 호스트 실행은 585 passed / 13 failed였다. 소켓 permission 실패와
  stream timeout이 있었으며, timeout까지 동일 원인으로 단정하지 않는다.
- UI는 64 passed / 1 failed: SAG 탐색 기본값 테스트가 존재하지 않는 `sag/`
  디렉터리를 기대했다. 구현의 config-root fallback과 기대 계약을 R0에서 대조한다.
- `model/` 경로 평탄화는 미커밋 변경이다. 검증 증거에 HEAD만 쓰지 말고
  working-tree diff 식별도 포함한다. 마일스톤 문서 작성은 이 변경의 재검증이 아니다.

다음 담당자는 R0의 환경/실패 기준선부터 시작한다. 이후 R1과 R2를 순서대로
수행하며, 각 단계의 실패를 고치는 데 필요한 변경만 진행한다.

### 작업 선택과 증거 기록 규칙

- 프론트 진입점 → 요청 → 백 처리 → 실제 결과 → 화면 상태를 한 기능으로
  추적한다. 테스트 개수나 코드 줄 수는 사용률/완료율이 아니다.
- 코드는 현재 경로 필수 / 다른 운영 조건에 필요 / 후속 작업용 / 용도 불명으로
  분류한다. 삭제는 대체 경로·실제 사용 여부·안전/복구 영향 확인 후 결정한다.
- 현재 gate를 막지 않는 대규모 재구조화, UI 전면 개편, typed service/action
  전환, 새 자동 작업, 임의 성능 최적화는 착수하지 않는다.
- 실패한 시나리오는 최소 재현을 남기고 기존 focused test를 보강한다. 기존
  suite와 같은 확인만 하는 새 테스트 체계나 별도 마일스톤 도구는 만들지 않는다.
- 상태는 미착수 / 진행 / 환경대기 / 부분완료 / 완료로 갱신한다. 완료에는
  모든 해당 수용 조건의 증거가 필요하다. 환경이 없다는 이유로 통과 처리하지 않는다.
- 원본 로그는 `workbench/evidence/generated/readiness/<run-id>/`에 둔다.
  검토할 요약만 `workbench/evidence/curated/readiness/`로 승격한다. 실제 실행 전
  빈 폴더를 만들지 않으며 token, private key, TURN secret은 보관하지 않는다.
- 각 결과는 `R-ID / 날짜 / commit+diff / host·topology·backend·보안 / 설정 /
  실행 명령·조작 / 기대 결과 / 관측 결과·측정값 / pass·fail·blocked /
  증거 경로 / 다음 조치`를 포함한다. 이 문서는 요약과 증거 링크만 소유한다.

## 기존 software 구현 범위

- Pilot/Sim/UI/Robot의 Router-free ROS 2/DDS 직접 통신과 protocol v6
  `PeerEnvelope`가 구현됐다.
- descriptor/heartbeat, boot/sequence fence, bounded startup queue, Robot/Sim
  motion lease, Sim UI session이 구현됐다.
- encoded latest-only RGB-D broker와 observer/hand-eye WebRTC 분리가 구현됐다.
- Robot–Unitree bridge UDS 경계, peer credential 검증, replay fence와 deadman
  stop이 구현됐다.
- installer state v10, mode-free topology schema v5, 독립
  DDS/SSH endpoint, ownership-based uninstall이 구현됐다.
- fixed `elesim-runtime` Compose와 선택적 `elesim-dev` attachment, managed
  Coturn, Docker Desktop Tailscale sidecar가 구현됐다.
- role-scoped managed SROS2 generation의 stage/activate/verify/rollback/recover와
  external keystore 경계가 구현됐다.
- four-role + infra release build/verify, four-process DDS smoke, RGB-D 및 두
  WebRTC track software gate가 존재한다.

과거 software 검증 기록이며, 현재 revision의 통과나 R 마일스톤 완료를 뜻하지
않는다. 아래 조건별 수동 gate도 별도로 남는다.

## 조건별 gate: production readiness 전 필수

1. 실제 2–4 host에서 descriptor/heartbeat, addressed control, lease/session
   expiry, stale boot 거부와 latest-only RGB-D를 검증한다.
2. SROS2 enforce가 role별 publish/subscribe를 실제로 허용·거부하고, rotation이
   교체된 generation을 revoke하는지 검증한다.
3. Jetson/GO2에서 Unitree bridge stop deadline, arm safe-hold/cleanup, deadman과
   malformed/disconnect 처리 시간을 측정한다.

## 조건별 gate: 추가 운영 환경

- L2 LAN, routed LAN/VPN, global IPv6에서 DDS 경로를 확인하고 ordinary IPv4
  NAT/CGNAT/symmetric NAT은 actionable failure를 내는지 확인한다.
- Docker Desktop Tailscale sidecar 두 node의 enrollment, namespace/address,
  route, static discovery와 bidirectional DDS를 검증한다.
- managed generation rotate, one-host failure rollback, interrupted recovery,
  SSH fingerprint pinning, non-default OpenSSH port와 Tailscale SSH re-auth를
  실제 host에서 검증한다.
- observer/hand-eye WebRTC의 direct 및 Coturn relay ICE, SDP bounds,
  DTLS/SRTP, renegotiation과 peer loss를 검증한다.
- GPU/CPU policy, NVIDIA reservation/CUDA visibility, NVENC/libx264,
  X11/WSLg owner와 Genesis Viewer를 실제 host에서 검증한다.
- source-to-consumer frame age, bandwidth, loss, p95 latency, Genesis render,
  conversion/transfer, media encode와 CPU MPC timing을 분리 측정한다.
- 실제 Look–Aim–Grasp convergence와 physical stop deadline을 검증한다.

## 보류된 후속 작업

- 생성된 typed ROS service/action의 runtime wiring. 현재 control/signaling
  surface는 protocol v6 `PeerEnvelope`다.
- perception, camera timing, MPC와 physical adapter의 property/stress coverage.
- 측정 근거가 생긴 뒤 Genesis inertia, neutral-qpos, self-collision warning
  재평가.
- bounded queue와 authority/security 경계를 보존하는 operator diagnostics.

## 변경하지 말아야 할 경계

- Coturn은 DDS를 운반하지 않고 SSH forwarding은 DDS locator가 아니다.
- `ROS_DOMAIN_ID`는 인증 또는 tenant isolation이 아니다.
- running container, exact boot heartbeat, authority/session grant, media
  readiness는 서로 다른 상태다.
- `elesim-update`는 실행 중 container를 교체하지 않는다.
- Router/ZMQ compatibility, unbounded queue, transient-local control QoS를
  복구책으로 추가하지 않는다.

## 검증 명령

```bash
elesim-dev python3 workbench/tools/quality/check.py --group required
elesim-dev python3 workbench/tools/quality/check.py --group extended
elesim-dev python3 workbench/tools/release/build.py
elesim-dev python3 workbench/tools/release/verify.py dist/releases
```

실제 gate 결과는 날짜, topology, host, interface, security profile, source
revision과 실패 범위를 함께 기록한다.
