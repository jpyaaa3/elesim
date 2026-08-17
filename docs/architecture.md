# EleSim Architecture

이 문서는 현재 Router 없는 EleSim runtime의 정본이다. 설치 절차는
[`setup.md`](setup.md), 다중 호스트 배포는 [`deployment.md`](deployment.md),
wire 계약은 [`dds_contracts.md`](dds_contracts.md)를 따른다.

## 1. 배포 모델

EleSim은 네 개의 독립 애플리케이션으로 나뉜다. monorepo의 공통 소스는 개발
편의를 위한 것이며, 릴리스 wheel이나 일반 사용자 이미지가 다른 배포
애플리케이션을 import하는 것은 허용하지 않는다.

```text
full topology (2–4 hosts)

  pilot ──────── DDS ──────── sim ──────── private Unitree DDS/NIC
    │             │            │                    ▲
    │             │            └─ WebRTC media       │
    └──── DDS ─── ui ──────────────── DTLS/SRTP      │
                                  robot ── UDS ──────┘

simulation-only topology (1–3 hosts)

                 pilot ─── DDS ─── sim
                   ▲                │
                   └──── ui ────────┘
```

`simulation-only`은 Pilot/Sim/UI만 갖고 Robot 또는 Jetson placeholder를
저장하지 않는다. `full`은 Pilot/Sim/UI/Robot을 각각 정확히 한 번
배치하고 Robot은 native Jetson unit이어야 한다. 두 모드와 schema migration은
[`design/connection_manager.md`](design/connection_manager.md)에 정의되어 있다.

중앙 Router, ZMQ, CurveZMQ, CURVE, ZAP은 현재 구조에 없다. 각 DDS participant는
필요한 peer와 직접 IP-routable해야 하며, DDS discovery는 애플리케이션
registry나 권한 부여기가 아니다.

## 2. 역할과 소유권

| 역할 | 소유 | 소유하지 않는 것 |
| --- | --- | --- |
| `pilot` | Vision, Arm model, Look/Aim/Grasp, Gaze, target 생성, 한 target lease | 물리 I/O, Genesis, UI 구현 |
| `sim` | Genesis, prebuilt model, virtual telemetry/RGB-D, motion lease, UI simulation session, observer/hand-eye 렌더와 WebRTC signaling/media | operator workflow, 물리 하드웨어 |
| `ui` | operator intent, simulation control, 상태 표시, 두 WebRTC 수신 화면 | IK/workflow, hardware driver, Genesis |
| `robot` | Dynamixel/GO2 I/O, RGB-D, motion lease, deadman, limit, local safety | model builder, IK, UI, Sim |

Robot과 Sim은 자기 motion lease의 유일한 authority다. Sim은 UI
simulation session의 유일한 authority다. discovery, `ROS_DOMAIN_ID`, static
peer는 이 권한을 부여하지 않는다.

### Unitree 경계

`elesim-unitree-bridge`는 Jetson 내부의 전용 하드웨어 adapter다. Unitree
ROS 2와 CycloneDDS는 private Jetson–GO2 NIC/domain에서 이 daemon만 사용한다.
Inter-host EleSim graph에는 Robot만 참여한다. Robot과 bridge는
`AF_UNIX SOCK_SEQPACKET` bounded JSON packet을 주고받고, `SO_PEERCRED`, boot
ID, monotonic sequence, command/parameter allowlist, keepalive deadman을
검증한다.

bridge는 다섯 번째 애플리케이션도 Router도 아니다. disconnect, malformed
packet, replay, keepalive expiry는 GO2 stop을 유발하지만, Robot은 arm
safe-hold·torque-off·hardware cleanup을 계속 수행한다.

## 3. 프로세스와 의존성

```text
{pilot, sim, ui, robot}
       │
       ├── elesim_interfaces (ROSIDL wire types)
       └── protocol (PeerEnvelope, discovery, authority, RGB-D helpers)

model/builder ── model/source → model/bundles/default
installer/package ── state/config/Compose/security/lifecycle artifacts
misc/tools/release ── isolated release contexts
misc/system_tests ── cross-process acceptance probes
```

각 배포 tree는 sibling 구현을 import하지 않는다. 공유 가능한 것은
`packages/elesim_interfaces`의 ROSIDL type과 `packages/protocol`의 transport
primitive뿐이다. typed ROS service/action 정의는 생성되지만 현재 runtime에
연결되어 있지 않다. 현재 control/signaling carrier는 protocol major 6의
bounded `PeerEnvelope`다.

General 설치는 고정 `elesim-runtime` Compose project와 선택된
`elesim-pilot`, `elesim-ui`, `elesim-sim` container를 사용한다. Robot은
native-only다. Developer 설치는 고정 `elesim-runtime-dev` project의 영속
`elesim-dev` 한 개와 선택적 `elesim-jaeger`만 만든다.

## 4. 통신 경계

### DDS control와 discovery

각 participant는 고유 endpoint ID와 process마다 새 boot ID를 광고한다.
`EndpointDescriptor`와 `EndpointHeartbeat`가 정확한 endpoint/boot 쌍을
확정한 뒤에야 주소 지정 envelope을 처리한다. startup 동안 수신한 envelope은
한 heartbeat timeout 동안 최대 512개만 보관하며, 정확한 source descriptor가
나타나면 해제하고 아니면 버린다. 무제한 queue나 transient-local control QoS로
이 경계를 대체하지 않는다.

Pilot은 discovery interval마다 `select_target`을 반복하고 Sim/Robot의
`target_selected`를 확인한다. stale boot, sequence, lease/session token은
거부한다. 이전 process의 envelope이 새 process의 권한을 되살릴 수 없다.

### Authority와 lifecycle

- Pilot은 한 번에 한 Robot 또는 Sim target만 lease한다.
- Robot/Sim이 lease를 serialize하고 Pilot boot/target identity를 기록한다.
- Sim은 독립적으로 한 UI simulation session만 grant한다.
- switch, explicit release, TTL expiry, process restart는 이전 권한을 revoke한다.
- lease/session renewal은 owner가 확인한 live peer와 token에 한해서만 허용한다.
- Estop은 일반 command path를 우회할 수 있지만 role/authority 검사는 유지한다.
- Robot의 transport loss는 안전 상태(lease revoke, deadman 유지, 재탐색)로 처리한다.

### RGB-D

RGB-D는 `RgbdFrame` typed DDS sample 하나로 전달하는 latest-only coherent
stream이다. subscriber backlog를 만들지 않으며, 오래된 frame은 새 frame으로
덮어쓴다. Robot과 Sim의 RGB-D topic은 `dds_contracts.md`의 QoS·권한 표를
따른다.

## 5. Sim 내부 구조와 영상

Sim은 하나의 deployable application, DDS participant, SROS2 enclave다. 다만
physics/authority와 media worker를 내부적으로 분리한다.

```text
Genesis scene thread
  ├─ physics, leases, session gate
  └─ latest-only frame slots (observer, hand-eye)
                         │
                         ▼
                    media worker
                   aiortc / FFmpeg / ICE
```

Genesis scene/camera object는 scene owner thread에서만 접근한다. frame slot은
고정 크기이며 producer/consumer backlog가 없다. media worker의 encode,
signaling, ICE 지연이 physics나 DDS receive loop를 block하지 않는다. worker
실패는 해당 media operation을 실패시킬 뿐 Sim heartbeat·lease·session
authority를 죽이지 않는다.

DDS endpoint는 boot identity를 일찍 광고하지만, Sim scene과 media worker의
bounded startup handshake가 끝나기 전에는 UI session을 grant하지 않는다.
따라서 UI의 초기 request는 잠시 거부될 수 있고, 새 descriptor 이후 재시도된다.

UI는 observer와 hand-eye를 별도 WebRTC track으로 받는다. WebRTC offer/answer
signaling은 Sim 소유의 reliable DDS request/reply이고, 픽셀은 DTLS/SRTP다.
Coturn은 필요할 때 ICE media candidate만 relay하며 DDS discovery/control/
RGB-D/signaling을 relay하지 않는다.

H.264 encoder는 NVIDIA/FFmpeg NVENC가 노출되면 `h264_nvenc`를 시도하고,
권한·드라이버 실패 시 `libx264`로 되돌아간다. `ELESIM_WEBRTC_ENCODER=cpu`
또는 `nvenc`는 의도적인 A/B·요청 모드이며 계약과 latest-only semantics를
바꾸지 않는다.

Genesis GPU backend가 CPU 부담 전체를 없애지는 않는다. camera render,
RGB/depth conversion·resize·host transfer, Genesis–Pinocchio copy, CasADi
QP/MPC solve, torque assembly, metrics는 각각 별도 timing field로 본다.
현재 QP/MPC solver와 DDS serialization은 CPU domain이며, GPU offload는
별도 측정·검증 없이는 가정하지 않는다.

## 6. 모델과 설정 lifecycle

`model/source`는 builder input이고 `model/bundles/default`는 immutable runtime
input이다. Sim은 기본적으로 bundle을 읽고, `ELESIM_SIM_DEV_REBUILD=1`일
때만 개발 중 rebuild한다. Pilot은 `config/arm_model.json`을 읽으며 runtime에
assembly를 만들지 않는다.

```bash
elesim-build-sim-bundle --assets model/source/assets --output model/bundles/default
elesim-build-arm-model --config pilot/config/config.pc.yaml \
  --assets model/source/assets --output pilot/config/arm_model.json
```

설치된 설정은 source default와 분리된 prefix 아래 생성된다. 역할 컨테이너는
role-specific YAML, read-only config/model mount, role-scoped security view만
받는다. 설정의 정규 필드와 ownership은 [`configuration.md`](configuration.md)에
있다.

## 7. 네트워크와 보안

### DDS profile

- `trusted-network`: DDS encryption 없음. 소유 LAN 또는 routed VPN, 명시적
  interface/firewall boundary에서만 사용한다.
- `sros2`: 공유·관찰 가능한 망의 enforce-mode authentication, access control,
  encryption. 각 role에 전용 enclave와 least-privilege permission을 준다.

두 profile 모두 동일한 `system_id`, `domain_id`, RMW, discovery mode, bound
interface, compatible QoS가 필요하다. `ROS_DOMAIN_ID`는 보안 경계가 아니다.
일반 IPv4 NAT/CGNAT/symmetric NAT은 지원하지 않는다. static peer는 discovery
seed일 뿐 direct UDP sample의 relay가 아니다.

### SROS2 ownership

`external`은 operator가 준비한 local keystore/enclave를 사용한다. `managed`는
operator laptop의 connection manager가 Authority generation을 만들고,
공통 public material과 host에 배정된 role enclave만 각 runtime host에
배포한다. CA private key와 다른 host의 role key는 절대 runtime host에 복사하지
않는다.

rotation은 모든 host를 stage → stop → atomic activate → restart/verify하는
transaction이다. 실패하면 이미 건드린 host를 이전 generation으로 rollback한다.
Authority는 administrative asset이며 runtime peer·broker·fifth service가
아니다.

### Docker backend

설치 시 `direct-host` 또는 `tailscale-sidecar`를 결정해 state에 고정한다.
Docker Desktop은 WSL의 `tailscale0`를 상속하지 않으므로, sidecar backend는
고정 `elesim-tailscale` kernel-mode node와 실제 sidecar namespace를 만든다.
role, runtime-network doctor, Sim-owned Coturn만 그 namespace를 공유한다.
sidecar는 host network infrastructure이지 DDS relay, SSH endpoint, Router가
아니다. `elesim-tailscale login/status`로 한 번 enrollment하고, SSH 관리
주소와 DDS sidecar 주소를 별도 기록한다.

## 8. 성능과 안전의 불변식

- physics loop에는 network peer, WebRTC encoder, DDS subscriber backlog를
  동기적으로 기다리는 코드가 없어야 한다.
- 모든 입력 queue와 media frame slot은 유한하다. full이면 입력을 거부하거나
  오래된 frame을 버리고 진단한다.
- pause는 physics만 멈추며 heartbeat/status/media session은 유지한다.
- reset은 simulation epoch을 증가시키고 stale workflow를 중단한다.
- Robot의 deadman/safety monitor는 DDS discovery callback에 의존하지 않는다.
- cleanup·hardware stop 실패는 숨기지 않고 fatal diagnostic으로 남긴다.
- fallback은 black frame, CPU encoder 등 bounded recovery일 뿐 성공을 가장하지
  않는다.

## 9. 검증 경계

```bash
python3 misc/tools/quality/check.py --group required
python3 misc/tools/quality/check.py --group extended
python3 misc/tools/release/build.py
python3 misc/tools/release/verify.py dist/releases
```

자동 gate는 ROSIDL, 역할 경계, release isolation, 별도 프로세스 DDS/RGB-D,
encoded WebRTC track과 contract/lease/safety test를 검증한다. `elesim-dev`
컨테이너가 canonical scientific/ROS test environment다.

자동화가 증명하지 않는 것은 실제 multi-host route/discovery, SROS2 enforce
authorization, NAT/TURN relay, GPU/X11/WSLg, Genesis viewer·observer 화면,
Jetson/Unitree physical safety와 Look–Aim–Grasp convergence다. 이들은
[`MILESTONES.md`](MILESTONES.md)의 수동 acceptance gate다.
