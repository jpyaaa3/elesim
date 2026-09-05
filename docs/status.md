# 구현 상태와 수용시험

갱신일: 2026-09-05. 이 문서만 현재 완료 범위, 미해결 항목, 수동
acceptance gate를 소유한다. 구현 불변식은 `architecture.md`, wire 계약은
`dds_contracts.md`, 운영 절차는 `setup.md`와 `deployment.md`를 따른다.

## 완료된 software 범위

- Pilot/Sim/UI/Robot의 Router-free ROS 2/DDS 직접 통신과 protocol v6
  `PeerEnvelope`가 구현됐다.
- descriptor/heartbeat, boot/sequence fence, bounded startup queue, Robot/Sim
  motion lease, Sim UI session이 구현됐다.
- encoded latest-only RGB-D broker와 observer/hand-eye WebRTC 분리가 구현됐다.
- Robot–Unitree bridge UDS 경계, peer credential 검증, replay fence와 deadman
  stop이 구현됐다.
- installer state v10, topology schema v4 `full`/`simulation-only`, 독립
  DDS/SSH endpoint, ownership-based uninstall이 구현됐다.
- fixed `elesim-runtime` Compose와 선택적 `elesim-dev` attachment, managed
  Coturn, Docker Desktop Tailscale sidecar가 구현됐다.
- role-scoped managed SROS2 generation의 stage/activate/verify/rollback/recover와
  external keystore 경계가 구현됐다.
- four-role + infra release build/verify, four-process DDS smoke, RGB-D 및 두
  WebRTC track software gate가 존재한다.

완료는 software 검증을 뜻한다. 아래 수동 gate를 대신하지 않는다.

## P0: production readiness 전 필수

1. 실제 2–4 host에서 descriptor/heartbeat, addressed control, lease/session
   expiry, stale boot 거부와 latest-only RGB-D를 검증한다.
2. SROS2 enforce가 role별 publish/subscribe를 실제로 허용·거부하고, rotation이
   교체된 generation을 revoke하는지 검증한다.
3. Jetson/GO2에서 Unitree bridge stop deadline, arm safe-hold/cleanup, deadman과
   malformed/disconnect 처리 시간을 측정한다.

## P1: 운영 수용시험

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

## P2: 후속 작업

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
