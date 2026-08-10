# Elesim 마일스톤 및 남은 인수시험

기준일: 2026-08-04
기준 브랜치: `refactoring`

이 문서는 구현 작업과 실제 장비·네트워크에서의 인수시험을 분리해 기록한다.
자동 테스트가 통과했다고 해서 다중 호스트, SROS2 enforce, GPU, NAT, Jetson의
동작까지 증명된 것으로 간주하지 않는다.

## 현재 결론

Router/ZMQ 제거와 DDS 전환, M1 소프트웨어 작업, M2-A/M2-B의 연결관리자
소프트웨어 구현은 완료됐다. 따라서 당장 남은 것은 기능을 계속 쪼개는 새
마일스톤이 아니라 아래 M3~M6의 고정된 인수시험이다.

현재 제어·signaling은 bounded `PeerEnvelope` DDS carrier를 사용한다. ROSIDL
typed service/action 정의는 생성되어 있지만 runtime에는 아직 연결되지 않았다.
이를 엄격한 typed surface로 바꾸는 일은 별도 후속 개발이며, 아래 인수시험을
통과하기 위한 선행 조건은 아니다.

## 완료된 마일스톤

### 기반 전환 — Router 없는 ROS 2/DDS

- Pilot, UI, Sim, Robot이 중앙 Router 없이 직접 DDS로 통신한다.
- RGBD와 WebRTC signaling은 DDS, observer/hand-eye 영상 pixel은 WebRTC
  DTLS/SRTP로 유지한다.
- ZMQ, CurveZMQ, CURVE, ZAP 의존성과 Router 릴리스를 제거했다.
- 실제 CycloneDDS를 사용하는 same-host 4프로세스 topology smoke를 통과했다.

### M1 — Software & Operator Readiness

- Docker/native runtime의 bounded text-log snapshot과 보존 정책을 추가했다.
- ownership manifest, exact-prefix 확인, preserve-by-default 정책을 가진
  host-only CLI 언인스톨을 추가했다. GUI는 계획과 명령 안내만 한다.
- Unitree ROS 2 참여자를 `elesim-unitree-bridge` daemon으로 분리했다.
  Robot은 bounded `SOCK_SEQPACKET` UDS IPC만 사용하며 peer credential,
  boot/sequence fencing, deadman stop을 적용한다.
- native Robot 릴리스에 Robot/bridge 두 systemd unit과 계정·ACL 전제조건을
  반영했다.

### M2-A — Two-host preflight

- Jetson 없이 두 COM 호스트의 DDS/SSH endpoint를 검증하는 ephemeral preflight를
  구현했다.
- preflight는 topology 저장, 키 발급, 원격 배포를 수행하지 않는다.

### M2-B — Simulation-only connection manager

- `full`과 `simulation-only` topology mode를 고정했다.
- simulation-only에서는 Robot/Jetson 없이 Pilot, Sim, UI만
  1~3개 호스트에 배치할 수 있다.
- GUI, deployment, lifecycle rollback, SROS2 policy가 활성 role 집합만
  사용하도록 했다.
- topology schema v1-v3 입력을 v4로 정규화하며 managed SROS2
  Authority/host bundle 생성 로직을 반영했다.

### 릴리스·문서·자동 검증

- 네 application release tree와 shared ROSIDL/protocol wheel을 생성하고
  격리 검증한다.
- setup, connection manager, security, release 문서를 현재 구조에 맞췄다.
- required/extended gate와 ownership, logging, Unitree 회귀 테스트를 통과했다.

### 소프트웨어 후속 — 계약·운영·조작성

플랫폼 인수시험과 혼동하지 않는 범위에서 다음 소프트웨어 경계를 고정했다.

- `DDS_CONTRACTS`와 [`dds_contracts.md`](dds_contracts.md)가 모든
  `PeerEnvelope` message type의 송수신 역할, QoS, authority와 payload 정책을
  단일 목록으로 제공한다. 빈 lease 갱신/해제와 ack/error 계열은 오타 field를
  거부한다.
- 연결 관리자는 선택된 runtime namespace의 Tailscale `tailscale0`를 감지해
  prefill하고, topology schema v4에서 SSH와 DDS endpoint를 독립적으로
  저장한다. 설치기는 `direct-host` 또는 `tailscale-sidecar` backend를 자동
  결정해 고정한다. `check`, `start`, `stop`, `restart`는 기존 pinned local/SSH
  lifecycle을 재사용하며 DDS discovery나 WebRTC media의 생존을 SSH 성공으로
  추정하지 않는다.
- 정적 peer는 tools runtime namespace에서 `iproute2` route 검사를 거친다.
  keyless Tailscale SSH topology에는 같은 namespace의 port-22 도달 실패를
  조기에 보고하는 negative-only probe가 추가되며, 상태·CycloneDDS XML·Compose
  DDS 값이 어긋난 구형 산출물은 실행 전에 거부한다. 어느 probe도 DDS/UDP
  discovery의 실기 수용을 대신하지 않는다.
- Observer camera 조작은 pinned Genesis 1.2.0 Trackball 의미(왼쪽 orbit,
  가운데 pan, 오른쪽/휠 zoom, pole clamp)를 사용하고 Roll 표시 방향은
  canonical roll 증가와 일치한다. UI의 두 영상은 별도 native window 한 곳에서
  렌더되며 창 닫기는 숨김, 주 창의 Show 버튼으로 재개된다.

이 항목들은 코드·단위/구조 검증 대상이며, 실제 다중 호스트·NAT·GPU·Jetson
수용시험을 통과했다는 뜻이 아니다.

## 남은 마일스톤

아래 네 개를 남은 인수 단계로 고정한다. 각 단계의 exit condition을 만족하기
전에는 다음 단계로 완료 선언하지 않는다. 새 마일스톤을 중간에 추가하지 않는다.

### M3 — Simulation-only 실제 2호스트 수용시험

목적: Jetson 없이 노트북과 연구실 서버 사이에서 실제 서로 다른 OS process가
직접 DDS/WebRTC로 통신하는지 확인한다.

대상 구성:

- Tailscale routed VPN을 우선 사용한다. 주소는 문서나 소스에 하드코딩하지
  않고 각 호스트의 현재 설정에서 입력한다.
- Robot은 배치하지 않는다.
- Pilot, Sim, UI를 두 호스트에 나누어 배치한다.
- DDS interface, domain, RMW, discovery mode, security profile을 양쪽에서
  일치시킨다. Tailscale은 DDS relay가 아니므로 실제 양방향 UDP 경로를
  확인한다.

Exit condition:

- 두 호스트에서 connection manager preflight와 topology validation이 통과한다.
- Pilot 명령, Sim authority/session, UI simulation control이
  실제 호스트 사이에서 왕복한다.
- RGBD가 최신 샘플 의미를 유지하며 subscriber backlog 없이 전달된다.
- observer와 hand-eye 두 WebRTC stream이 실제 ICE negotiation 후 동작한다.
- Pilot/UI/Sim를 각각 종료·재시작해 lease/session 만료, stale
  sequence 거부, 재발견과 재연결을 확인한다.
- loss/reorder 상황에서 stop/deadman과 오류 로그가 확인된다.
- M3는 일반 IPv4 NAT/CGNAT/symmetric NAT 지원 증명이 아니다. 해당 경로는
  명시적으로 실패하고 actionable diagnostic을 내야 한다. Docker Desktop/WSL은
  WSL `tailscale0` 상속이 아니라 Docker VM 내부 kernel-mode sidecar를 사용하고,
  sidecar 등록·static peer·양방향 DDS를 실제 두 호스트에서 별도로 증명해야 한다.
  Generated Compose와 namespace guard의 자동검사는 이 실기 exit condition을
  대신하지 않는다.

### M4 — 보안·원격 배포·호스트 수용시험

목적: 실제 네트워크와 설치 운영에서 DDS 보안 경계와 connection manager의
배포 원자성을 확인한다.

Exit condition:

- L2 LAN, routed LAN/static peer, routed VPN, 가능하면 global IPv6에서 고정
  RMW의 discovery/control/RGBD 상호운용성을 각각 기록한다.
- controlled LAN/VPN의 `trusted-network` 경계와 shared network의 SROS2
  enforce를 구분해 검증한다.
- 무권한 publish/subscribe가 거부되고, `ROS_DOMAIN_ID`만 바꿔 보안을 우회할
  수 없음을 확인한다.
- managed Authority의 host별 role bundle, 전체 generation rotation,
  한 호스트 실패 시 rollback, 교체 generation revocation을 확인한다.
- SSH agent/선택 key 또는 Tailscale SSH(키 없는 port 22), host fingerprint
  pinning, 비표준 OpenSSH port를 실제 원격 host에서 확인한다. SSH forwarding은
  설치 GUI 접근에만 사용한다.
- GUI loopback 접근을 로컬 및 비표준 SSH forward에서 확인한다.
- 외부/managed Coturn의 lifecycle과 실제 WebRTC relay candidate를 ICE stats와
  Coturn 로그로 확인한다. TURN은 DDS를 전달하지 않는다.

### M5 — Jetson/Unitree 하드웨어 수용시험

목적: 실제 Jetson/GO2에서 native Robot과 분리된 Unitree bridge의 안전 경계를
확인한다.

Exit condition:

- JetPack/ROS2 Unitree workspace, dedicated account/group/ACL, 두 systemd
  unit의 `BindsTo`/`PartOf` lifecycle이 실제 설치된다.
- Unitree DDS가 private NIC/domain에만 묶이고 Elesim inter-host DDS와
  섞이지 않는다.
- UDS peer credential, malformed packet, sequence/boot replay, bridge
  disconnect가 모두 거부·정지 동작으로 이어진다.
- deadman/lease 만료가 요구 시간 안에 모터를 정지시킨다.
- IPC 오류가 arm safe-hold, torque-off, hardware close를 건너뛰지 않는다.
- 실제 카메라/모터/GO2 safety와 ROS2 context/domain 공존을 기록한다.

### M6 — 조작·인지 end-to-end 수용시험

목적: 통신·배포를 넘어 실제 시뮬레이션과 하드웨어 조작이 완료되는지 확인한다.

Exit condition:

- Genesis에서 Look-Aim-Grasp 한 사이클을 target visibility, 제한된 camera
  motion, 감소하는 `remain`, blind handoff, contact/gripper close까지 포함해
  완주한다.
- 실제 하드웨어에서도 안전한 grasp/stop 사이클을 최소 한 번 완주한다.
- RealSense/YOLO/Genesis frame continuity, depth validity, tracker identity,
  frame drop과 timestamp/p95 latency를 기록한다.
- 사용자 stop, reconnect, reacquire, process loss에서 camera/perception과
  Robot safety가 일관되게 종료·복구된다.

## 마일스톤 외 후속 backlog

다음은 플랫폼 인수시험을 막지는 않지만 별도 개발 대상으로 남아 있다.

- `PeerEnvelope` 기반 control/signaling을 typed ROS service/action runtime
  surface로 단계적으로 연결하고 schema/version 계약을 갱신한다.
- broad exception과 silent fallback을 구조화된 endpoint/UI health와 expected
  error로 줄인다.
- RealSense, Dynamixel, Unitree bridge, Genesis camera의 integration-rig
  coverage를 높인다.
- Genesis/Pinocchio upstream warning과 dynamics/contact 가정을 실제 접촉
  데이터로 정리한다.

## 현재 실행 순서

1. M3를 Tailscale 기반 simulation-only 두 호스트에서 수행한다.
2. M3 결과를 바탕으로 M4의 보안·원격 배포·TURN 시험을 수행한다.
3. Jetson이 준비되면 M5를 수행한다.
4. M5와 무관하게 가능한 Genesis 검증을 먼저 하고, 장비가 준비되면 M6의
   하드웨어 수용시험을 마무리한다.

상세한 known issue와 과거 관찰 근거는
[`OPEN_ISSUES_KR.md`](OPEN_ISSUES_KR.md)를 참조한다.
