# EleSim 마일스톤과 수용시험

이 문서는 2026-08-17 기준의 현재 범위표다. “구현됨”은 software gate를
통과했다는 뜻이고, 실제 네트워크·GPU·하드웨어 검증을 대신하지 않는다.

## 완료된 software 범위

### Router 없는 ROS 2/DDS

- Pilot/Sim/UI/Robot이 직접 CycloneDDS peer로 통신한다.
- protocol major 6 `PeerEnvelope`, typed latest-only RGB-D, descriptor/heartbeat,
  boot/sequence/lease/session fence가 구현되어 있다.
- Router/ZMQ/CURVE surface와 legacy runtime alias가 제거됐다.

### 역할과 lifecycle

- Robot/Sim motion lease와 Sim UI simulation session이 분리됐다.
- Robot safety/deadman은 DDS discovery와 독립적으로 동작한다.
- Sim media는 bounded latest-only frame slot과 별도 WebRTC worker를 사용한다.
- observer/hand-eye는 WebRTC DTLS/SRTP, DDS는 signaling만 운반한다.

### 설치·배포

- General `elesim-runtime`, Developer `elesim-runtime-dev`/`elesim-dev`가 고정됐다.
- installer state v9, ownership-based uninstall, source freshness, GPU policy,
  direct-host/tailscale-sidecar backend가 구현됐다.
- topology schema v4의 `full`/`simulation-only`, independent DDS/SSH endpoint,
  role-aware deployment가 구현됐다.
- managed SROS2 generation의 stage/activate/verify/rollback과 external keystore
  경계가 구현됐다.
- release context four-role + infra build/verify가 구현됐다.

### 자동 검증

필수/extended quality gate, 역할별 테스트, ROSIDL build, release isolation,
four-process DDS smoke, RGB-D roundtrip, encoded two-stream WebRTC 검증이
구성되어 있다. canonical 환경은 setup-generated `elesim-dev`다.

## 남은 수동 acceptance gate

### A. 실제 다중 호스트

- 2–4 host에서 discovery/control/lease/session/RGB-D를 확인한다.
- L2 LAN, routed LAN/VPN, global IPv6에서 직접 UDP 경로와 실패 진단을 확인한다.
- ordinary IPv4 NAT, CGNAT, symmetric NAT은 지원되지 않음을 확인한다.
- Docker Desktop Tailscale sidecar 양쪽 enrollment, namespace, static peer,
  bidirectional DDS를 확인한다.

### B. 보안과 배포

- SROS2 enforce permission이 role별 publish/subscribe를 실제로 거부/허용하는지
  확인한다.
- managed generation rotate, 한 host failure rollback, replaced generation
  revocation을 실제 host에서 확인한다.
- SSH fingerprint pinning, non-default OpenSSH port, Tailscale SSH re-auth를
  확인한다.

### C. 영상과 성능

- observer/hand-eye 두 track의 실제 화면, SDP renegotiation, DTLS/SRTP,
  direct/relay ICE candidate를 확인한다.
- NVENC/libx264 선택, camera conversion/transfer, Genesis render, MPC solve의
  timing과 frame age/bandwidth/loss를 측정한다.
- GPU reservation, CPU-only, WSLg/X11 및 실제 Viewer display owner를 확인한다.

### D. Robot과 end-to-end

- Jetson JetPack/ROS2, Unitree private NIC/domain, UDS peer credentials,
  bridge loss/malformed packet deadman, arm cleanup을 확인한다.
- 실제 Look–Aim–Grasp, camera timing, physical stop deadline과 convergence를
  확인한다.

## 현재 실행 순서

```bash
# 개발/소프트웨어
elesim-dev python3 misc/tools/quality/check.py --group required
elesim-dev python3 misc/tools/quality/check.py --group extended
elesim-dev python3 misc/tools/release/build.py
elesim-dev python3 misc/tools/release/verify.py dist/releases

# 설치된 host
elesim-status
elesim-net namespace-check --dds-interface <interface>
elesim-connections                 # operator laptop, topology/security
```

수동 gate를 자동 성공으로 문서화하지 않는다. 상세 미해결 항목은
[`OPEN_ISSUES.md`](OPEN_ISSUES.md)와 [`OPEN_ISSUES_KR.md`](OPEN_ISSUES_KR.md),
과거 결과는 [`audit/`](audit/)에 있다.
