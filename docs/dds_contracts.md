# EleSim DDS 계약

현재 wire major는 **protocol 6**이다. 이 문서는
`packages/protocol/src/elesim_protocol/contracts.py`의 `DDS_CONTRACTS`와
`packages/elesim_interfaces`의 typed messages를 사람이 검토하기 위한
registry다. 새 메시지·field·QoS는 구현보다 먼저 protocol/schema 결정을
기록하고 registry, validator, contract test, process smoke를 함께 바꾼다.

## 1. Carrier와 identity

control/signaling은 bounded `PeerEnvelope` DDS carrier를 사용한다.
각 envelope은 message type, source/target endpoint, source boot ID,
monotonic sequence, lease/session fence와 bounded payload를 갖는다. 수신자는
구현 메서드를 호출하지 않고 이 값들을 검증한 뒤 자기 domain state만 바꾼다.

각 participant는 별도의 ROS discovery descriptor/heartbeat topic에
`PeerRef`, role, capability, stream descriptor, exact boot-specific resource
prefix를 광고한다. descriptor와 같은 endpoint/boot의 heartbeat가 확인되기
전에는 peer를 live authority로 취급하지 않는다. startup queue는 heartbeat
timeout 동안 최대 512 envelope만 보관한다.

RGB-D는 별도 typed `elesim_interfaces/msg/RgbdFrame` topic이다. observer와
hand-eye 픽셀은 DDS payload가 아니며 WebRTC DTLS/SRTP track이다.
`webrtc_signal`만 PeerEnvelope에 들어간다.

## 2. Control registry

| message | sender → receiver | authority/용도 | QoS | payload 규칙 |
| --- | --- | --- | --- | --- |
| `discover` | Pilot/UI → all | peer 조회 | reliable control | `role`, `capability` |
| `endpoint_list` | all → Pilot/UI | descriptor 목록 | reliable control | bounded endpoint array |
| `operator_intent` | UI → Pilot | Pilot workflow | reliable control | validated request DTO |
| `operator_result` | Pilot → UI | workflow result | reliable control | `request_id`, `ok`, bounded result/error |
| `select_target` | Pilot → Robot/Sim | target owner lease 요청 | reliable control | `target_id` only |
| `target_selected` | Robot/Sim → Pilot | lease 응답 | reliable control | target + `lease_id` |
| `renew_target` | Pilot → Robot/Sim | lease 갱신 | reliable control | empty object |
| `release_target` | Pilot → Robot/Sim | lease 반납 | reliable control | empty object |
| `target_released` | Robot/Sim → Pilot | 명시적 종료 | reliable control | target + optional reason |
| `target_lost` | Robot/Sim → Pilot | peer/TTL loss | reliable control | target + reason |
| `lease_granted` | Robot/Sim → relevant peers | authority 관찰 | reliable control | Pilot identity |
| `lease_revoked` | Robot/Sim → relevant peers | authority 종료 | reliable control | reason |
| `motion_command` | Pilot → Robot/Sim | lease-bound motion | best-effort, depth 1 | validated `MotionCommandRequest` |
| `telemetry` | Robot/Sim → Pilot/UI | bounded snapshot | reliable control | additive bounded state map |
| `ack` | all → all | legacy-compatible ack | reliable control | `reply_to`, `ok`, optional reason |
| `error` | all → request source | bounded failure | reliable control | `reply_to`, reason |

`motion_command`는 latest command 성격의 depth-1 best effort다. Estop과 local
deadman은 이 carrier나 DDS discovery callback에 의존하지 않는다.

Mock hug의 마지막 `motion_command`는 `mock_hug` 메타데이터
(`solution_id`, object lifecycle `revision`, OBJ `sha256`, `final_q`)를 함께
보낸다. 실행 시 캡처한 exact Sim endpoint/boot/lease도 routing fence로
포함하며 Pilot 송신 직전과 Sim 수신 직후에 다시 확인한다. Sim은 각
waypoint를 적용하기 전에 현재 spawn identity와 고정된
`final_q`를 다시 검증하고, 마지막 자세가 정착한 뒤에만 attach한다. 이
메타데이터는 motion lease를 대체하거나 새 authority를 만들지 않는다.
Robot은 `mock_hug`가 붙은 target을 하드웨어에 쓰기 전에 명시적으로 거부한다.

## 3. Simulation session과 media

| message | sender → receiver | authority/용도 | QoS | payload 규칙 |
| --- | --- | --- | --- | --- |
| `open_simulation_session` | UI → Sim | UI session 요청 | reliable control | `OpenSimulationSessionRequest` |
| `simulation_session_opened` | Sim → UI | session grant | reliable control | typed session DTO |
| `simulation_session_granted` | Sim → Sim | local session event | reliable control | typed session DTO |
| `simulation_session_revoked` | Sim → UI/Sim | session 종료 | reliable control | reason/epoch |
| `renew_simulation_session` | UI → Sim | session 갱신 | reliable control | empty object |
| `close_simulation_session` | UI → Sim | session 반납 | reliable control | `CloseSimulationSessionRequest` |
| `simulation_command` | UI → Sim | pause/reset/camera/speed | reliable control | `SimulationCommandRequest` |
| `simulation_result` | Sim → UI | command result | reliable control | typed result DTO |
| `simulation_status` | Sim → UI/Pilot | scene/session state | reliable control | typed bounded status |
| `webrtc_signal` | UI ↔ Sim | offer/answer/ICE signaling | reliable control | bounded SDP/ICE DTO |

Sim이 session authority를 소유한다. motion lease와 UI session은 서로 다른
권한이다. `simulation_session_opened`가 오기 전 UI가 media를 요청하면
bounded retry/diagnostic만 발생하고 권한이 생기지 않는다.

`simulation_command`의 mock-object surface는 catalog `asset_id`와 pose만
받는다. OBJ bytes/path는 `PeerEnvelope`에 넣지 않는다. Sim은 시작 전에
local catalog를 검증하고 scene build 전에 bounded entity pool을 준비한다.
`simulation_status.mock_object`는 Pilot 계산용 world-oriented XZ convex
hull(최대 64점), lifecycle revision과 SHA-256만 전달한다. 이 additive
필드는 `simulation.mock_hug.v1` capability를 광고한 v6 peer에만 전달하여
기존 strict v6 status parser를 깨지 않는다. `spawn_mock_object`,
`remove_mock_object`, `detach_mock_object`는 UI session에 묶이며, 실제 arm
motion은 계속 Pilot motion lease에만 묶인다. pose는 ±10 m, Euler는 ±360°로
제한한다.

## 4. RGB-D QoS

`RgbdFrame`은 timestamp, frame identity, dimensions, intrinsics와 RGB/depth
payload를 함께 갖는 coherent sample이다. publisher와 subscriber는 latest-only
depth-1 semantics를 사용하며, 오래된 sample을 backlog로 쌓지 않는다. Robot과
Sim의 topic prefix는 endpoint boot/resource descriptor와 함께 광고된다.

RGB-D는 WebRTC를 대체하지 않는다. Pilot/Sim이 제어·perception에 쓰는 정합
sample과 UI가 보는 observer/hand-eye 영상은 서로 다른 경로다.

## 5. 검증과 fencing

수신자는 다음 순서로 envelope을 검사한다.

1. registered message type과 sender/receiver role
2. exact source endpoint ID와 live boot descriptor/heartbeat
3. monotonic sequence와 bounded payload/strict fields
4. target lease 또는 Sim session token/epoch
5. domain-specific finite/range/allowlist validation

boot 변경, restart, release, TTL expiry는 이전 lease/session을 revoke한다.
새 peer descriptor가 오기 전 queue는 유한하고, descriptor가 timeout 안에
오지 않으면 버린다. stale boot이나 replayed sequence를 “최근 값”으로
처리하지 않는다.

## 6. 보안과 네트워크 계약

`trusted-network`는 controlled LAN/routed VPN에서만 허용하는 plaintext DDS
profile이다. 공유·관찰 가능한 네트워크는 role-scoped SROS2 enforce를
사용한다. `ROS_DOMAIN_ID`, namespace, static peer 주소는 인증 경계가 아니다.

static peers는 DDS discovery seed이며 DDS/NAT relay가 아니다. Coturn은
WebRTC DTLS/SRTP media만 relay한다. SSH local-forward, Tailscale SSH, HTTP
test server는 DDS endpoint가 아니다. 일반 IPv4 NAT/CGNAT/symmetric NAT은
지원하지 않는다.

## 7. 변경 금지선

- Router/ZMQ/CURVE 호환층을 추가하지 않는다.
- control QoS를 transient-local로 바꾸거나 startup queue를 무제한으로 만들지
  않는다.
- typed ROS service/action이 생성되어 있다는 이유로 runtime surface라고
  문서화하지 않는다. 현재 surface는 이 registry와 `RgbdFrame`이다.
- payload를 application object, Python pickle, sibling import로 전달하지 않는다.
- 새 field는 additive/ bounded인지, protocol major를 올려야 하는지 먼저
  결정한다.

정합성 검사:

```bash
elesim-dev python3 misc/tools/quality/check.py --group required
elesim-dev python3 misc/system_tests/smoke_topology.py
```
