# EleSim DDS 계약

현재 wire major는 **protocol 6**이다. 이 문서는
`payload/runtime/common/protocol/elesim_protocol/contracts.py`의 `DDS_CONTRACTS`와
`payload/runtime/common/elesim_interfaces`의 typed messages를 사람이 검토하기 위한
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

동일 endpoint에 두 live boot가 겹치면 어느 쪽도 authority 대상으로 선택하지
않는다. Exact-identity 조회는 unavailable을 반환하고 발행은 재시도 가능한
transport 오류로 거부한다. Lease/session의 로컬 해제와 DDS 루프는 계속
작동하며, 이전 boot의 heartbeat timeout 후 남은 정확한 boot만 다시 발견한다.
이 복구는 protocol 6의 기존 identity·wire·QoS를 유지한다.

RGB-D는 source와 Pilot의 local handoff 이후 Pilot이 소유하는 별도 DDS stream이다.
observer와 hand-eye 픽셀은 DDS payload가 아니며 WebRTC DTLS/SRTP track이다.
`webrtc_signal`만 PeerEnvelope에 들어간다. 기존 typed
`elesim_interfaces/msg/RgbdFrame`은 migration/diagnostic raw contract로 남을 수
있지만 새 inter-host deployment는 encoded broker contract를 사용한다.

## 2. Control registry

Motion renewal recovery retains protocol major 6 and the existing wire fields.
Pilot correlates `error.reply_to` with a bounded history of renewal requests,
including the target endpoint/boot and lease token. A rejection for the current
lease clears that lease and emits local `target_lost`, allowing normal target
selection to acquire a new lease. Unrelated errors and delayed errors or release
notifications for an older lease cannot revoke the current lease.

2026-09-07 operator surface 정리: wire major 6과 view schema 1을 유지한다.
IK visualization adds `simulation_command.set_ik_target` under the existing UI
simulation-session lease (protocol v6 / simulation schema v1, additive command).
Arguments are finite world-frame `position` (three metres values, bounded ±10 m)
and normalized `direction` (three values bounded ±1). This changes display
markers only, never Robot motion, Sim joint targets or collision geometry.
Older Sims explicitly reject the unknown command. UI resends the latest target
after a new session. Observer rendering receives the marker in its bounded
camera snapshot; hand-eye RGB-D excludes these debug markers.

Arm trajectory execution is an additive protocol-v6 operator extension:
`plan_arm_to_target()` computes a bounded cuRobo path to the current UI XYZ without
moving; `execute_arm_trajectory(plan_id)` consumes the prepared plan once;
`cancel_arm_trajectory()` cancels it. `view_snapshot.service.arm_trajectory`
reports progress. Older Pilots reject these allowlisted calls. No new Robot
wire payload or protocol-major change is introduced: Pilot streams existing
lease-bound canonical `target.q` messages. Preparation pins exact target boot
and lease, model/artifact digest, fresh measured start and open-claw state;
execution rechecks these and rejects concurrent operator motion. Route loss,
stale feedback, tracking error or timing overrun cancels remaining samples.
Planning uses the Pilot's explicit static world file; it does not synchronize
Genesis obstacles or certify physical clearance.
For XYZ target planning, `target_xyz` denotes the 20 mm object's center and
`planned_xyz` the intended final open-claw grasp center at that same coordinate.
The prepared path's final FK must be within 1 mm of the requested object center;
an outside standoff is not accepted as a completed plan. These existing snapshot
fields and operator/wire shapes are unchanged (protocol v6). Target collision
checking remains active for both claws and every other modeled arm link.

현재 UI가 사용하는 `view_snapshot`, `service_call`, `state_call`, `state_set`만
지원한다. UI가 snapshot cache에서 읽는 조회 메서드, `snapshot`과
`service_get` 우회 조회는 제거한다. Envelope/응답 구조와 현재 UI 동작은
변하지 않으며, 제거된 호출을 직접 보내는 구형 custom client는 명시적으로
거부된다. 이 제한은 모든 과거 custom client와의 호환성을 주장하지 않는다.

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
| `telemetry` | Robot/Sim → Pilot/UI; Robot → opt-in Sim | bounded snapshot, no motion authority | reliable control | additive bounded state map |
| `ack` | all → all | legacy-compatible ack | reliable control | `reply_to`, `ok`, optional reason |
| `error` | all → request source | bounded failure | reliable control | `reply_to`, reason |

`motion_command`는 latest command 성격의 depth-1 best effort다. Estop과 local
deadman은 이 carrier나 DDS discovery callback에 의존하지 않는다.

Sim telemetry의 선택적 `go2_controller_fault`는 최대 512자 문자열이다. GO2
제어기가 복구 불가능한 오류로 정지하면 사유를 유지하고, 명시적 제어기 reset
후 빈 문자열로 해제한다. Pilot operator snapshot과 UI GO2 panel에 전달되며
새 motion authority나 Robot의 arm `safety_fault` 해제 권한을 만들지 않는다.
Protocol major 6의 additive telemetry 확장이며 기존 발신자는 생략할 수 있다.

Depth-plane calibration 역시 protocol major 6의 additive surface다. Robot의
선택적 `measurement_time_s`는 arm sample의 source wall-clock 시각이며 기존
`measurement_monotonic_s`는 freshness 확인 의미를 유지한다. Pilot은 이를
같은 source의 RGB-D capture time과 대응시키며 다른 host의 monotonic clock을
서로 비교하지 않는다. 기존 encoded RGB-D의 source/boot/calibration ID와
capture time은 decode/adapter 뒤에도 보존한다. ROSIDL 메시지와 carrier는
바뀌지 않는다.

Operator view schema 1의 선택적 service 값 `floor_calibration_status`는 평면
후보·기준면·측정 오차·동기화/freshness 상태를 전달한다. Allowlisted
`confirm_floor_calibration(candidate_id)`, `reset_floor_calibration()`과
`export_floor_calibration()`은 Pilot-local 관측 상태와 bounded 기록만 다룬다.
실제 arm motion이나 Robot correction-model 설치 권한을 만들지 않는다.
카메라 좌표 보정 선택은 기존 `update_perception_config`의 검증된 설정
경계를 사용한다. 구형 UI는 새 service 값을 무시하며 구형 Pilot에는 새
service 값이 없어 이 제어를 표시하지 않는다.

시각 기울기 보정은 같은 schema 1 service object의 additive
`correction_mode="orientation"`, `observable=["tilt_2d"]`와
`unobservable=["translation_3d", "rotation_about_normal"]`로 범위를 표시한다.
`confirm_floor_calibration`은 선택 면이 수평이라는 operator 확인이며
`reference.kind="confirmed_horizontal"`, `absolute=false`를 반환한다. 높이가
다른 평행면은 같은 방향 근거로 사용할 수 있다. Camera origin·관절값·Robot
명령은 변경하지 않으며 `distance_error_m`는 legacy 진단 값일 뿐 적용량이 아니다.
새 carrier/message/operation은 없고 protocol major 6은 유지한다.

Robot presentation mirror restores the former hardware-state display path.
Protocol major 6 is unchanged; the mirror capability adds no message type or
payload field. The open telemetry state map also carries the additive
`claw_load_permille` hardware reading described below.
A Sim configured with `runtime.mirror_robot_id` advertises
`presentation.robot-state.v1`. Robot sends its existing measured telemetry to
these live Sim peers even without a Pilot lease. Each send is independent of
motion authority and a disappearing viewer cannot revoke the Robot's lease.
Sim accepts only its configured Robot endpoint's exact live boot, increasing
envelope sequence, and finite measured `q`. It stores only the latest pose and
freezes it on telemetry loss. A Sim motion lease takes precedence over mirroring.
This path never emits commands to Robot, enables torque, or grants a lease.
It also carries available GO2 base/leg state for `mirror_from_host` rendering.
Old Sim peers do not advertise the capability and receive no new traffic.

Gripper `target.claw_closed` retains its boolean wire meaning. Robot alone maps
it to calibrated motor degrees (`arm.claw_open_deg=340`, `arm.claw_closed_deg=230`)
instead of sending logical 0/180 into a physical 230..340 clamp. The additive
telemetry `claw_closed` reports the nearest calibrated endpoint from measured
motor 5 position. Pilot/UI keep this feedback separate from requested state;
Sim mirrors it. Protocol major 6 and existing command/envelope fields remain
unchanged. Physical speed limits and native controller ABI 5 are host-local
implementation/configuration changes, not new DDS authority.

IMU 해석 모델 선택은 기존 v6 `motion_command`의 새 명령 값
`set_imu_model`이다. 새 message type이나 기존 registry field set을 추가하지
않으므로 wire major는 6을 유지하고, 명령 안의 `model.schema_version`을 1로
고정한다. UI `view_snapshot`의 service 항목도 기존 schema 1에
`imu_models`와 `imu_model_status`를 추가하는 방식으로 확장한다. Payload는
`selection_id`와 `model` (`schema_version`, `id`, `version`, `program`)만
허용한다. `program`은 최대 64개의 선행 노드 참조 계산 그래프와 네 개의 출력
노드 번호다. 지원 연산은 `q`, `imu`, `const`, `add`, `sub`, `mul`, `div`,
`neg`, `sin`, `cos`이다. `imu`의 index 0..2는 RPY 입력이다. Pilot의
`config/imu_models/*.json`이 계산식과 표시 이름을 소유하고, Robot은 motion
lease와 sequence를 확인한 후 계산 그래프를 C++ 실행 계획으로 검증·설치한다.
계산 중 비정상 수치나 stale IMU는 Robot의 로컬 fault 경계를 따른다. 제공하는
모델은 현재 `identity/v1` JSON 하나이며 IMU 데이터를 사용하지 않는다.
Robot telemetry의 `imu_model`, `imu_model_selection_id`,
`imu_feedback_connected`는 추가 필드다. Pilot은 같은 lease에서 같은 선택 ID를
돌려받아야 활성으로 표시한다. Best-effort 선택 명령이 유실되면 재전송한다.
UI의 기존 sag JSON 경로 선택과 Pilot/Sim의 q 기반 sag 계산은 제거한다.
Pilot의 IMU 모델 catalog가 보정 계산식을 소유하고 Robot의 C++ 루프가 실행한다.

Teensy reader diagnostics are additive under protocol 6 telemetry `imu_reader`
and operator-view schema 1 `imu_model_status.reader`. Reader schema 1 contains
`connected`, `error`, `invalid_lines`, and at most two `sensors` (`IMU1`, `IMU2`).
Each sensor has `quaternion_wxyz`, `rpy_rad`, `device_time_ms`, `sequence`
(null for legacy firmware), `received_age_s`, and `fresh`. Freshness requires
a new v2 sequence within 0.2 s; legacy ten-field lines never prove freshness.
These are uncalibrated diagnostics, not the native correction input;
`imu_feedback_connected` retains its existing meaning. Pilot suppresses reader
values after one second without selected-target telemetry. No motion authority,
model-input expansion or protocol-major change is introduced.

The user-supplied Full IMU CSV is also supported: 52 columns (tx_ms, seq,
two 25-column sensor blocks). Reader schema 1 adds `source_format`,
`freshness_verified`, `reports_recent`, and optional `full_imu` metadata
(tx_time_ms, bno_time_us, accuracy, gravity, gyro, linear_accel, accel, mag).
Full IMU `newAny` only proves some report changed; it cannot verify quaternion
freshness. `fresh` remains false for this format. Quaternion/RPY may be null
before the first valid rotation report. Blocks with newAny=0 do not refresh
sensor age, and the global transmit sequence does not refresh the other sensor.

`reboot_faulted_motor`는 기존 v6 `motion_command`의 command 문자열만 추가하므로
message shape와 protocol major 6은 유지한다. Robot은 활성 lease, torque OFF,
특정 DYNAMIXEL Hardware Error를 확인하고 fault ID 하나만 재부팅한다. UI Hardware
패널은 팔 지지 확인을 받은 뒤 이 요청을 보낸다. Robot은 재부팅 후 모든 모터의
torque OFF와 정상 telemetry를 검증해야 latch를 해제하며, torque-on이나 이동 명령은
발행하지 않는다.

`clear_fault`도 기존 v6 `motion_command` 문자열을 사용한다. UI의
`clear_safety_fault`는 Robot에 torque OFF, fresh telemetry, fault 재발 여부를
검증하게 한다. 검증 실패 시 latch를 유지하고 사유를 반환한다. 성공해도 torque는
OFF로 남으며, 이후 torque-on은 별도 operator 동작이다.

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

### Edge broker contract

| 단계 | sender → receiver | carrier | payload |
| --- | --- | --- | --- |
| source edge handoff | Robot 또는 Sim → Pilot | bounded DDS source topic | encoded coherent RGB-D (legacy raw 입력 허용) |
| encoded broker stream | Pilot → UI/optional Sim | DDS/SROS2 typed latest-only | `encoded_rgbd_v1` |

Pilot은 `stream.rgbd.broker.v1` capability와 함께 `StreamDescriptor`를
광고한다. descriptor의 기존 필드를 다음처럼 해석한다.

```yaml
name: rgbd
transport: dds
media_kind: rgbd
endpoint: /elesim/pilot_main/rgbd/frame
message_type: elesim_interfaces/msg/EncodedRgbdFrame
qos_profile: sensor_data_latest_only
```

Robot may additionally advertise `rgbd_right` for the ZED Mini right-eye color
image. Pilot relays it to the configured `rgbd.wire.right_topic` broker topic;
it uses the same encoded frame type with `has_depth=false`. The left-eye
`rgbd` stream remains the perception input and carries registered depth. This
named optional stream and separate topic do not change the ROS message or
protocol major version.

Pilot also publishes the left-eye color-only preview to the configured
`rgbd.wire.left_preview_topic` (normally
`/elesim/pilot_main/rgbd/left_preview`). UI subscribes to this topic for the
left ZED preview so remote preview traffic does not carry the larger depth
payload. The full-depth `/rgbd/frame` broker remains available to Pilot
perception and other consumers that need depth.

codec, dimensions, intrinsics/calibration ID, depth scale, sequence, source boot
identity와 payload bound는 frame metadata에 있다. 새 descriptor field나
protocol major bump는 이 migration에 필요하지 않다. `stream.rgbd.broker.v1`
capability가 없는 peer에 encoded를 raw로 오인시켜 보내지 않는다.

source 단계의 기존 `RgbdFrame`은 timestamp, frame identity, dimensions,
intrinsics와 RGB/depth payload를 함께 갖는 coherent sample이다. 모든 publisher와
subscriber는 latest-only depth-1 semantics를 사용하며 오래된 sample을 backlog로
쌓지 않는다. 새 inter-host consumer는 Robot/Sim source topic을 직접 구독하지
않고 Pilot broker endpoint만 사용한다.

Python publisher는 raw Image와 EncodedRgbdFrame의 uint8 payload에 소유권이
분리된 `array('B')`를 넣어 생성 ROSIDL의 native buffer 경로를 사용한다.
기존 payload 크기·값 검증과 wire 필드는 유지한다. 이는 protocol-v6의
로컬 표현 최적화이며 새 wire schema가 아니다.

RGB-D는 WebRTC를 대체하지 않는다. Pilot/Sim이 제어·perception에 쓰는 정합
sample과 UI가 보는 observer/hand-eye 영상은 서로 다른 경로다.

### RGB-D failure와 측정

- handoff가 늦으면 Pilot은 오래된 frame을 버리고 perception age를 보고한다.
- encode 실패 시 frame을 발행하지 않는다. sequence/boot identity를 유지해
  stale frame을 새 frame으로 오인하지 않게 한다.
- consumer decode 실패는 해당 consumer stream만 끊으며 Pilot perception이나
  Robot safety를 중지시키지 않는다.
- broker restart는 새 boot ID와 sequence를 사용한다.

Acceptance evidence는 encoded bytes/frame, bitrate, source-to-Pilot age,
encode p50/p95, DDS publish-to-receive p50/p95, decode p50/p95와
overwrite/drop/gap count를 포함한다. 이 측정 없이 render, GPU transfer,
encoder와 DDS를 원인으로 단정하지 않는다.

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
elesim-dev python3 workbench/tools/quality/check.py --group required
elesim-dev python3 workbench/tests/system/smoke_topology.py
```


Ghost preview adds `simulation_command.set_arm_ghost` to protocol v6 / simulation
schema v1 (additive, existing UI simulation-session authority). `preview:null`
clears it; otherwise the strict payload contains a plan ID (1..64 chars), 2..96
finite q4 samples, matching increasing `times_s` from zero through at most 120 s,
and a finite proper rigid 4x4 world/base transform. No torque, gripper or motion
command fields are accepted. Pilot's operator view provides this bounded preview
only for a ready, unconsumed, unexpired plan. UI forwards it to Sim independently
of the selected Robot motion target and clears it on other phases/disconnection.
Sim interpolates visual samples in wall time, caps playback at 120 seconds,
and draws translucent URDF debug meshes. These meshes are excluded from hand-eye
RGB-D and have no physics or collision entity. Older Sims reject the unknown
command; all participating software must be updated together.
