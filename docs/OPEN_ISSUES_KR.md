# 현재 미해결 이슈

기준일: 2026-08-17. 과거 backlog가 아니라 현재 남은 수용시험과 후속 작업만
기록한다. 현재 구조는 [`architecture.md`](architecture.md), 설치/배포는
[`setup.md`](setup.md)와 [`deployment.md`](deployment.md)를 정본으로 한다.

## P0 — 운영 준비 완료 전에 반드시 증명할 것

1. **실제 다중 호스트 DDS**: 2–4대에서 descriptor/heartbeat 수렴, addressed
   control, lease/session 만료, latest-only RGB-D, stale boot 거부를 확인한다.
2. **SROS2 enforce**: role별 허용/거부 publish·subscribe와 managed generation
   rotate/rollback/revocation을 실제 host에서 확인한다.
3. **물리 안전**: Jetson/GO2에서 Unitree bridge stop deadline, arm cleanup,
   deadman과 hardware failure 처리를 확인한다.

## P1 — 운영 수용시험

- Docker Desktop Tailscale sidecar 두 node의 enrollment, namespace/address/
  route, static discovery와 양방향 DDS를 확인한다. SSH 성공은 DDS 증거가 아니다.
- routed LAN/VPN/global IPv6와 지원하지 않는 IPv4 NAT/CGNAT/symmetric NAT의
  명시적 진단을 확인한다.
- observer/hand-eye WebRTC direct 및 Coturn relay ICE, SDP 제한, DTLS/SRTP,
  renegotiation과 peer loss를 확인한다.
- GPU/CPU policy, NVIDIA reservation/CUDA visibility, NVENC/libx264, X11/WSLg
  display owner와 Genesis Viewer를 실제 host에서 확인한다.
- frame age/bandwidth/loss/p95 latency와 Genesis render, camera conversion,
  transfer, media encode, CPU MPC timing을 분리 측정한다.
- SSH fingerprint pinning, non-default OpenSSH port, Tailscale SSH check
  re-auth를 실제 host에서 확인한다.

## P2 — 후속 작업

- generated typed ROS service/action의 runtime wiring. 현재 control/signaling은
  protocol v6 `PeerEnvelope`다.
- perception/camera/MPC/physical adapter의 property·stress coverage 확대.
- Genesis inertia, neutral-qpos, self-collision warning을 모델/physics 측정
  이후 재평가한다.
- bounded queue, 명시적 실패, authority/security 경계를 보존하는 operator
  diagnostics 개선.

## 이슈가 아닌 것

- Coturn은 DDS를 운반하지 않으며 SSH forwarding은 DDS locator가 아니다.
- `ROS_DOMAIN_ID`는 인증/tenant isolation이 아니다.
- running Sim과 UI simulation session grant는 다르다. scene/media readiness와
  exact boot heartbeat가 별도 gate다.
- `elesim-update`는 running container를 교체하지 않는다. 적용은 `elesim-up`,
  전체 재시작은 `elesim-down` 후 `elesim-up`이다.
- Router/ZMQ, pre-v4 topology, dated audit의 과거 기록은 현재 결함이 아니다.
