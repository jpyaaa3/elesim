# EleSim DDS PeerEnvelope 계약 목록

기준 protocol major는 6이다. 아래 목록은 `packages/protocol/src/elesim_protocol/contracts.py`의
`DDS_CONTRACTS`가 실행 시 검사하는 단일 source of truth이다. `PeerEnvelope`는
메시지의 목적지와 boot/sequence fence를 운반하는 bounded DDS carrier이고,
RGBD는 별도의 typed `elesim_interfaces/msg/RgbdFrame` topic이다. observer와
hand-eye pixel은 WebRTC DTLS/SRTP이며 `webrtc_signal`만 이 표를 따른다.

| 메시지 | 송신 → 수신 | 소유권/용도 | QoS | payload 계약 |
|---|---|---|---|---|
| `discover` | Pilot/UI → 전체 | discovery 조회 | reliable control | `role`, `capability` |
| `endpoint_list` | Robot/Sim/Pilot → Pilot/UI | discovery 응답 | reliable control | endpoint descriptor 배열 |
| `operator_intent` | UI → Pilot | workflow 의도 | reliable control | `OperatorIntentRequest` |
| `operator_result` | Pilot → UI | workflow 결과 | reliable control | `request_id`, `ok`, optional `result`/`error` |
| `select_target` | Pilot → Robot/Sim | motion lease 획득 | reliable control | `target_id` |
| `target_selected` | Robot/Sim → Pilot | lease 응답 | reliable control | `target_id`, `lease_id` |
| `renew_target` | Pilot → Robot/Sim | lease 갱신 | reliable control | 빈 object |
| `release_target` | Pilot → Robot/Sim | lease 반납 | reliable control | 빈 object |
| `target_released`/`target_lost` | Robot/Sim → Pilot | lease 종료/peer loss | reliable control | `target_id`, optional `reason` |
| `lease_granted`/`lease_revoked` | Robot/Sim → 관련 peer | authority 관찰 알림 | reliable control | pilot 또는 reason |
| `motion_command` | Pilot → Robot/Sim | motion command | best-effort, depth 1 | `MotionCommandRequest` |
| `telemetry` | Robot/Sim → Pilot/UI | bounded state snapshot | reliable control | additive bounded state map |
| `ack` | 전체 → 전체 | legacy-compatible 결과 ack | reliable control | optional `reply_to`, `ok`, `reason` |
| `open_simulation_session` | UI → Sim | UI session lease | reliable control | `OpenSimulationSessionRequest` |
| `simulation_session_opened`/`granted`/`revoked` | Sim → UI/Sim | session lifecycle | reliable control | typed simulation session DTO |
| `renew_simulation_session` | UI → Sim | session 갱신 | reliable control | 빈 object |
| `close_simulation_session` | UI → Sim | session 종료 | reliable control | `CloseSimulationSessionRequest` |
| `simulation_command` | UI → Sim | pause/reset/camera command | reliable control | `SimulationCommandRequest` |
| `simulation_result`/`simulation_status` | Sim → UI/Pilot | command result/status | reliable control | typed simulation DTO |
| `webrtc_signal` | UI ↔ Sim | offer/answer signaling | reliable control | bounded SDP DTO; media는 WebRTC |
| `error` | 전체 → 요청자 | 실패 사유 | reliable control | `reply_to`, `reason` |

## 경계와 변경 규칙

- 프로그램은 서로의 구현 메서드를 호출하지 않는다. 수신자는 `message_type`,
  source endpoint/boot descriptor, lease/session fence, sequence를 확인한 뒤
  자기 domain state만 변경한다.
- `ROS_DOMAIN_ID`, namespace, static peer 주소는 인증 경계가 아니다.
  소유 LAN/routed VPN에서만 `trusted-network`를 쓰고, 공유망은 role-scoped
  SROS2 enforce를 사용한다.
- 새로운 메시지나 payload field를 추가할 때는 protocol major/schema 결정을
  먼저 기록하고 registry, validator, contract test, four-process smoke를 함께
  갱신한다. typed ROS service/action 정의는 현재 생성만 되어 있고 이 표의
  runtime surface가 아니다.
- static peers는 DDS discovery seed일 뿐 NAT/TURN relay가 아니다. Tailscale은
  양방향 routed UDP 경로를 제공할 수 있지만 설치·로그인·ACL 변경은 운영자가
  수행한다.
