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

             [pilot + sim] ─── DDS ─── ui
                   │                  ▲
                   └──── local RGB-D ─┘
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
| `pilot` | Vision, Arm model, Look/Aim/Grasp, Gaze, target 생성, 한 target lease, RGB-D edge broker | 물리 I/O, Genesis, UI 구현 |
| `sim` | Genesis, prebuilt model, virtual telemetry/RGB-D source, motion lease, UI simulation session, observer/hand-eye 렌더와 WebRTC signaling/media | operator workflow, 물리 하드웨어, inter-host RGB-D broker |
| `ui` | operator intent, simulation control, 상태 표시, 두 WebRTC 수신 화면 | IK/workflow, hardware driver, Genesis |
| `robot` | Dynamixel/GO2 I/O, RGB-D source, motion lease, deadman, limit, local safety | model builder, IK, UI, Sim, inter-host RGB-D broker |

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

model/builder ── model/bundles/default/assets → model/bundles/default
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

RGB-D의 source는 Robot 또는 Sim이지만 inter-host broker는 Pilot 하나다. source
edge가 raw frame을 encoded latest-only sample로 바꾸고 Pilot이 이를 검증·relay
한다. legacy raw source만 Pilot이 한 번 encode한다. Pilot은 encoded stream을
`/rgbd/frame`으로 publish하고 UI와 필요 시 Sim은 broker stream을 decode한다.
source가 둘인 구성에서 source topic을 inter-host consumer가 직접 구독하는 것은
금지한다.

기존 `RgbdFrame` typed DDS sample은 migration/diagnostic 호환 경계로 남을 수
있지만, 새 배포의 외부 stream descriptor는 `encoded_rgbd_v1`과
`stream.rgbd.broker.v1` capability를 명시해야 한다. codec, calibration ID,
depth scale, sequence와 payload bound는 encoded frame metadata가 소유하며
`StreamDescriptor` 구조나 protocol major를 불필요하게 확장하지 않는다.
자세한 배치와 실패 경계는 [`design/rgbd_edge_broker.md`](design/rgbd_edge_broker.md)를
따른다.

## 5. Sim 내부 구조와 영상

Sim은 하나의 deployable application, DDS participant, SROS2 enclave다. 다만
physics/authority와 media worker를 내부적으로 분리한다.

```text
Physics/authority process                 visual-only Genesis process
  ├─ physics, leases, session gate       ├─ camera scene + camera.render()
  ├─ latest state snapshots ────────────►├─ latest-only shared RGB-D slots
  └─ DDS receive/telemetry                └─ no collision/authority updates
                                      │
                                      ▼
                                  media worker
                                 aiortc / FFmpeg / ICE
```

기본 `camera_execution: async_process` 모드에서는 Genesis physics scene과
카메라 scene을 서로 다른 프로세스에 둔다. physics 프로세스는 카메라 객체나
`camera.render()`를 호출하지 않고, bounded `CameraStateSnapshot`만 latest-only
queue에 넣는다. 카메라 프로세스는 그 snapshot을 자체 visual replica에 적용해
렌더하고, 고정 크기 공유 메일박스에 최신 RGB-D 한 장만 쓴다. 따라서 느린 EGL
첫 렌더, GPU 동기화, host frame 복사가 physics step·DDS receive loop를
직접 block하지 않는다. 메일박스와 command/result queue에는 backlog가 없으며,
오래된 snapshot/frame은 새 데이터로 덮어쓴다. `sync_legacy`는 비교 측정과
장치 진단을 위한 명시적 fallback일 뿐이다.

각 Genesis scene/camera object는 자신이 생성된 프로세스에서만 접근한다. media
worker의 encode, signaling, ICE 지연이 physics나 DDS receive loop를 block하지
않는다. 카메라 프로세스가 런타임 중 실패해도 해당 media operation만 실패하고
Sim의 heartbeat·lease·session authority는 살아 있어야 한다. 초기 scene/worker
생성 실패는 잘못된 배포를 조기에 드러내기 위해 명시적으로 startup failure로
처리한다.

DDS endpoint는 boot identity를 일찍 광고하지만, Sim scene과 media worker의
bounded startup handshake가 끝나기 전에는 UI session을 grant하지 않는다.
따라서 UI의 초기 request는 잠시 거부될 수 있고, 새 descriptor 이후 재시도된다.

UI는 observer와 hand-eye를 별도 WebRTC track으로 받는다. Observer는 Genesis
1.2.x `ViewerOptions`의 기본 위치·look-at·world-Z up·FOV에서 시작한다. 기본
좌클릭 드래그는 카메라 위치와 world-Z up을 고정한 pan/tilt만 수행하고,
휠클릭 드래그는 시점 평행이동, 휠 회전은 확대/축소를 수행한다. Hand-eye의
operator view는 under-slung 180도 roll mount를
화면에서 보정하며, DDS RGB-D와 calibration frame은 원본 방향을 유지한다.
두 카메라의 capture cadence는 wall clock과 simulation time을 모두 만족해야
한다. 느린 physics step이 wall-clock 주기를 항상 초과하더라도 매 step마다 두
render를 강제하여 real-time factor를 더 악화시키지 않는다.
GPU 모드의 Genesis는 `performance_mode`를 사용한다. Headless Sim은 EGL을
사용하며 숫자형 `CUDA_VISIBLE_DEVICES`가 하나면 동일한 `EGL_DEVICE_ID`를
선택해 렌더링과 계산이 서로 다른 GPU에 걸리지 않게 한다. Genesis의 정규화된
RGB는 resize, channel reorder, uint8 변환까지 CUDA에서 처리하고 DDS/PyAV가
요구하는 최종 host frame만 한 번 전송한다. Native Viewer는 창 시스템의 OpenGL
선택을 유지한다.
WebRTC offer/answer
signaling은 Sim 소유의 reliable DDS request/reply이고, 픽셀은 DTLS/SRTP다.
Coturn은 필요할 때 ICE media candidate만 relay하며 DDS discovery/control/
RGB-D/signaling을 relay하지 않는다.

H.264 encoder는 NVIDIA/FFmpeg NVENC가 노출되면 `h264_nvenc`를 시도하고,
권한·드라이버 실패 시 `libx264`로 되돌아간다. `ELESIM_WEBRTC_ENCODER=cpu`
또는 `nvenc`는 의도적인 A/B·요청 모드이며 계약과 latest-only semantics를
바꾸지 않는다. NVENC 경로는 B-frame을 끄고 스트림 속도에 맞춘 1초 GOP와
forced-IDR을 사용해 RTP burst loss 뒤 복구 지연을 제한한다. H.264 RTP
payload budget은 기본 1000바이트(환경변수 `ELESIM_WEBRTC_RTP_PAYLOAD_MAX`로
900--1200 범위에서 조정 가능)로 제한해 TURN/Tailscale IPv6 캡슐화 뒤의
조각화 패킷을 피한다. UI는 ICE 연결 상태만으로 LIVE를 표시하지 않고,
디코드 프레임 정지 시 해당 track만 재협상한다.

Genesis GPU backend가 CPU 부담 전체를 없애지는 않는다. camera render,
RGB/depth conversion·resize·host transfer, Genesis–Pinocchio copy, CasADi
QP/MPC solve, torque assembly, metrics는 각각 별도 timing field로 본다.
현재 QP/MPC solver와 DDS serialization은 CPU domain이며, GPU offload는
별도 측정·검증 없이는 가정하지 않는다.

### Mock object hug vertical slice

Sim의 `config/mock_objects/`는 시작 시 검증되는 bounded OBJ catalog다. Genesis
scene build 뒤 동적 mesh 추가를 가정하지 않기 위해 catalog entity는 scene
build 전에 숨김 위치에 준비하고, UI session 명령은 기존 entity의 pose만
바꾼다. OBJ 파일이나 host path는 DDS에 싣지 않는다.

Pilot은 Sim status의 content hash, lifecycle revision, world-oriented XZ convex hull을
입력으로 target-first 두 section 감싸안기 자세와 개활지 국소 보간 경로를
계산한다. 모든 motion waypoint에 동일 identity와 최종 자세를 첨부하며 Sim은
적용 전에 이를 다시 검증한다. 최종 자세가 연속 sample에서 정착한 경우에만 mock object를 arm tip에
논리적으로 attach한다. 이 결과는 물리 파지 성공이나 collision-free RRT의
증명이 아니다. 향후 scan/reconstruction producer는 같은 bounded artifact
descriptor를 만들고, environment-aware planner와 motor-load verifier는 Pilot의
local planner 및 Sim mock verifier 경계에서 교체한다.

이 기능은 protocol major 6의 capability-gated additive extension이다. Sim은
`simulation.mock_hug.v1` capability가 없는 v6 peer에는 기존 status shape만
보내며, Pilot은 capability를 광고한 정확한 Sim boot와 motion lease를 실행
전체에서 고정한다. 따라서 target/boot/lease 변경은 다음 waypoint 전에 실행을
취소한다.

## 6. 모델과 설정 lifecycle

`model/bundles/default/assets`가 ZED Mini의 canonical builder input이며,
`model/bundles/default`는 ZED Mini 기본 프로파일의 assets와 생성된
blueprint/URDF를 함께 담는 self-contained runtime bundle이다.
`model/bundles/d435`는 기존 D435 프로파일의 독립적인 전체 bundle이다.
빌더는 임시 디렉터리에서 각 bundle을 만든 뒤 같은 경로에 원자적으로
publish하므로 별도의 중복 source tree나 runtime의 builder import가 필요하지
않다. Pilot/Sim의 `camera_profile`이 driver, hand-eye calibration, model
bundle을 함께 선택하며, SDK/device가 없는 경우 다른 프로파일로 자동 전환하지
않고 시작을 거부한다. Sim은 선택된 bundle을 읽고,
`ELESIM_SIM_DEV_REBUILD=1`일 때만 개발 중 rebuild한다. Pilot은
`config/arm_model.json`을 읽으며 runtime에 assembly를 만들지 않는다.

```bash
elesim-build-sim-bundle --assets model/bundles/default/assets --output model/bundles/default
elesim-build-sim-bundle --assets model/bundles/d435/assets --output model/bundles/d435
elesim-build-arm-model --config pilot/config/config.yaml \
  --assets model/bundles/default/assets --output pilot/config/arm_model.json
```

설치된 설정은 source default와 분리된 prefix 아래 생성된다. 역할 컨테이너는
role-specific YAML, read-only config/model mount, role-scoped security view만
받는다. Pilot과 Sim의 application 설정은 각각 `config/config.yaml` 한 파일에
공통값과 `profiles.pc`, `profiles.remote`, `profiles.jetson` 구획을 함께 둔다.
파일의 `mode` 또는 CLI의 `--mode`가 선택한 구획만 공통값에 깊이 병합되며,
설치 overlay도 같은 파일을 `extends`해 선택된 profile만 덮어쓴다. DDS와
설치/보안 runtime 설정(`runtime.yaml`)은 이 application 설정과 별개다.
설정의 정규 필드와 ownership은 [`configuration.md`](configuration.md)에 있다.

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
아니다. `elesim-tailscale login/status`로 한 번 enrollment하고 필요할 때
`elesim-tailscale update`로 설치된 고정 image를 적용한다. SSH 관리 주소와
DDS sidecar 주소는 별도 기록한다.

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
