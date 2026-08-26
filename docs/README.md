# EleSim 문서 백과사전

EleSim은 네 개의 독립 배포 애플리케이션(`pilot`, `sim`, `ui`, `robot`)으로
구성된 Router 없는 ROS 2/DDS 시스템이다. 이 문서는 현재 구현을 설명하는
정본과, 과거 감사·실험 기록을 구분한다.

> 문서 기준일: 2026-08-17. 설치·배포·운영 명령은 생성된 설치본의
> `elesim-setup status`와 이 문서의 정본을 함께 확인한다. 과거 문서의
> 날짜·마일스톤·로그는 현재 동작의 근거가 아니다.

## 빠른 읽기

1. [`architecture.md`](architecture.md) — 프로세스, 권한, DDS/WebRTC, 안전 경계
2. [`setup.md`](setup.md) — 설치 마법사, GPU/Viewer, 업데이트와 로컬 수명주기
3. [`deployment.md`](deployment.md) — 릴리스, 단일/다중 호스트, Robot, 보안 배포
4. [`configuration.md`](configuration.md) — 생성 설정과 필드 의미
5. [`dds_contracts.md`](dds_contracts.md) — protocol v6 계약과 QoS
6. [`design/connection_manager.md`](design/connection_manager.md) — 토폴로지 GUI와 원자적 배포

운영 중 확인이 필요하면 다음 순서로 읽는다.

```text
elesim-status → elesim-logs → setup.md의 문제 해결표
             → connection_manager.md의 호스트별 check/preflight
```

## 현재 시스템의 한 문장 정의

| 구분 | 현재 계약 |
| --- | --- |
| 제어·발견 | 모든 역할이 직접 CycloneDDS/ROS 2 UDP peer로 통신한다. 중앙 Router, ZMQ, CURVE는 없다. |
| 영상 | Sim이 observer/hand-eye WebRTC track을 만든다. DDS는 offer/answer 신호만 운반하고, 픽셀은 DTLS/SRTP로 흐른다. |
| RGB-D | Robot/Sim source의 raw frame을 Pilot이 edge broker에서 한 번 encode하고, inter-host에는 Pilot 소유의 latest-only `encoded_rgbd_v1` DDS stream만 보낸다. |
| 보안 | 소유 LAN/VPN의 `trusted-network` 또는 공유/비신뢰 망의 enforce-mode `sros2` 중 하나다. |
| 일반 설치 | `elesim-runtime` Compose, `elesim-pilot`, `elesim-ui`, `elesim-sim`; Robot은 Jetson native-only다. |
| 개발자 설치 | `elesim-runtime-dev`의 영속 `elesim-dev`; Jaeger는 선택적 `elesim-jaeger`다. |
| 설치와 실행 | 설치는 파일/컨텍스트만 생성한다. `elesim-update`는 재생성·증분 빌드, `elesim-up`은 적용·시작이다. |
| 연결 관리자 | 비밀 없는 토폴로지·보안 generation·호스트 lifecycle을 초기화한다. 재시작 버튼은 없다. 런타임 재시작은 각 호스트의 `elesim-down` 후 `elesim-up`으로 한다. |

## 백과사전식 목차

### 시스템

- [`architecture.md`](architecture.md): 역할과 소유권, authority/lease, Sim media
  boundary, 네트워크·보안, 성능 경계, 검증 범위.
- [`code_map.md`](code_map.md): 현재 worktree에서 생성되는 live 정적/관측 코드 그래프.
- [`configuration.md`](configuration.md): 설치 설정의 정규 필드와 생성 파일.
- [`dds_contracts.md`](dds_contracts.md): 모든 PeerEnvelope/typed RGB-D 계약.

### 설치와 배포

- [`setup.md`](setup.md): bootstrap, General/Developer, GPU, X11, update/up/down,
  logs/status, uninstaller, 기본 문제 해결.
- [`deployment.md`](deployment.md): release context, Compose/Robot, 다중 호스트,
  SROS2/TURN, Docker Desktop sidecar, 검증 명령.
- [`design/connection_manager.md`](design/connection_manager.md): schema v4의
  `full`/`simulation-only`, 독립 DDS·SSH endpoint, preflight와 security rollout.
- [`design/rgbd_edge_broker.md`](design/rgbd_edge_broker.md): Pilot 소유 RGB-D
  broker, source-local handoff와 `simulation-only` Pilot+Sim 배치.

### 연구·실험

- [`experiment_framework.md`](experiment_framework.md): 실험 프레임워크의
  연구용 경계.
- [`design/preview_gaze.md`](design/preview_gaze.md),
  [`design/gait_phase_preview.md`](design/gait_phase_preview.md): 제안·실험 설계.

### 상태·역사

- [`MILESTONES.md`](MILESTONES.md): 현재 완료 범위와 남은 수동 acceptance gate.
- [`OPEN_ISSUES.md`](OPEN_ISSUES.md), [`OPEN_ISSUES_KR.md`](OPEN_ISSUES_KR.md):
  현재 미해결 사항만 기록한다.
- [`jetson_mixed_role_rollout.md`](jetson_mixed_role_rollout.md): 2026-08-09
  mixed-role 작업 기록. 현재 실행 절차는 `deployment.md`를 따른다.

## 용어와 경계

- **role**: `pilot`, `sim`, `ui`, `robot` 중 하나의 배포 애플리케이션.
- **host**: DDS 주소/인터페이스와 SSH 관리 주소를 소유하는 물리·가상 컴퓨터.
- **deployment unit**: 한 prefix, ownership manifest, Compose 또는 systemd 수명주기를
  함께 갖는 독립 설치. 한 host에 여러 unit이 있을 수 있다.
- **endpoint**: DDS에서 광고하는 논리 ID와 현재 boot ID의 조합. discovery는
  authority를 주지 않는다.
- **authority**: Robot/Sim의 motion lease 또는 Sim의 UI simulation session.
- **manager**: `elesim-connections`를 실행하는 operator laptop의 초기화·배포 도구.
- **runtime**: 실제 역할 컨테이너/Robot 서비스와 그 역할이 소유한 Coturn.

SSH forwarding 포트는 설치 GUI 접근용일 뿐 DDS/WebRTC 포트가 아니다. static
peer와 TURN은 DDS NAT traversal을 제공하지 않는다. 일반 IPv4 NAT/CGNAT/
symmetric NAT은 지원 범위 밖이며, routed LAN/VPN 또는 검증된 global IPv6가
필요하다.

## 검증의 범위

자동 테스트는 계약, 패키지 경계, 생성 산출물, 별도 프로세스의 DDS/RGB-D,
실제 aiortc 인코딩을 검증한다. 다음은 수동 gate다.

- 실제 2–4 호스트의 DDS discovery/control/RGB-D와 SROS2 enforce 권한
- Docker Desktop Tailscale sidecar의 양방향 DDS와 TURN relay 후보
- NAT/방화벽/VPN 조건, GPU/NVIDIA Container Toolkit, X11/WSLg Viewer
- Genesis 성능, observer/hand-eye 실제 화면, Jetson/Unitree stop deadline
- 물리 로봇의 safe-hold, torque-off 및 Look–Aim–Grasp 폐루프

과거 audit은 이 경계를 축소하거나 자동 통과로 바꾸지 않는다.
