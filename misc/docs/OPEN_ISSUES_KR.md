# 미해결 이슈

> 경로 이관 안내(2026-07-18): 아래의 `engine/`, 루트 실행 파일,
> `configs/`, `assets/`, `crafts/` 표기는 protocol-v3 이전 구조의 기록이다.
> 현재 위치는 최상위 릴리스 프로젝트별 `src/`, 프로젝트별 `config/`,
> `misc/model/source/assets`, `model/bundles/default`이다. 과거 증거를 보존하기
> 위해 기존 관찰 내용은 삭제하지 않았다.

이 파일은 우선순위가 더 높은 작업을 진행하는 동안 알려진 미해결/보류 이슈가 묻히지 않도록 추적한다.

## 현재 상태 (2026-08-03)

이 절이 deployment 기반 현 구조의 기준이다. 아래의 긴 절들은 refactor 이전
근거를 보존한 기록이며, 여기에서 다시 언급하지 않은 경로와 테스트 수치는 과거
정보로 취급한다.

### P0. Router 없는 ROS 2/DDS 전환은 live 증명과 typed surface 후속이 필요함

- 상태: 실제 network 검증과 typed service/action binding에 대해 open이다.
  Router/ZMQ 제거, direct DDS carrier, software-only contract와 4-process
  CycloneDDS smoke는 완료했다.
- 최종 topology에는 Router, ZMQ, CurveZMQ, CURVE key, ZAP policy가 없다.
  Robot/Simulator가 자기 motion lease를 소유하고 Simulator가 별도 UI session을
  소유한다. RGBD와 WebRTC signaling request/reply는 DDS이며 영상 pixel은
  DTLS/SRTP WebRTC로 유지한다.
- control/signaling은 현재 bounded `PeerEnvelope` DDS message를 사용한다.
  typed service/action 정의는 생성되지만 runtime에 연결되지 않았으므로 활성
  interface인 것처럼 안내하면 안 된다.
- 완료한 software 근거에는 ROSIDL 산출물, Router 없는 네 release tree,
  protocol/setup 및 role별 suite, endpoint 중복 시 fail-closed, restart identity,
  target-owned lease/session 만료, stale sequence 거부, coherent RGBD,
  두 stream negotiation, 실제 RMW를 사용한 same-host 4-process smoke가 있다.
- 필요한 live 근거: 한 host, L2 multicast, routed static peer, routed VPN,
  global IPv6, 실제 SROS2 enforce, loss/reorder, process kill, Wi-Fi/VPN
  reconnect, NAT-only layout의 명시적 거부다.

### P0. 실제 Look-Aim-Grasp 수렴은 아직 증명되지 않음

- 상태: open, 최우선.
- UV/LJI/equal-sag/ready/IK deterministic property, headless phase workflow,
  실패/정상 grasp 로그 replay까지 자동화했다.
- 기존 실패 로그는 object-world jump, measured-motion stall, 약 98 mm에서의
  blind handoff, 약 20도 look error, 최종 abort로 재현·진단된다.
- 남은 근거: target visibility, 제한된 camera motion, 감소하는 `remain`, 안전한
  blind handoff, 그럴듯한 접촉과 gripper close를 증명하는 Genesis 1회 및 실제
  하드웨어 1회의 완전한 실행이다.

### P1. Camera/perception timing은 live 검증이 필요함

- 상태: open.
- Pick stop이 camera shutdown을 소유하지 않는다는 것과 worker lifecycle/state
  transition은 unit test로 고정했다.
- 그러나 hand-eye camera가 움직이는 동안 RealSense/YOLO/Genesis frame 연속성,
  depth validity, tracker identity는 아직 증명하지 못했다.
- 필요한 근거: Look, Aim, LJI, blind handoff, 사용자 stop, reconnect, reacquire를
  관통하는 timestamp/frame-drop metric이다.

### P1. Multi-host 및 hardware 배포는 여전히 수동 gate임

- 상태: open.
- same-host test는 DDS multicast/static-peer discovery, direct user-data
  locator, vendor port mapping, loss 환경 QoS와 SROS2 permission을 증명하지
  못한다.
- 실제 LAN/routed-VPN/global-IPv6, Jetson USB/serial, ROS2/Unitree
  domain/context 공존, clock skew, packet loss와 process restart timing은 live
  gate에서 검증하지 못했다.
- 일반 IPv4 NAT, CGNAT, symmetric NAT는 의도적으로 지원하지 않는다. TURN은
  WebRTC media만 relay하며 DDS 우회 수단으로 안내하면 안 된다.

### P1. 원격 Genesis 영상과 조작은 live gate가 필요함

- 상태: open.
- 기존 테스트는 observer/hand-eye media 독립성, Simulator main-thread
  mailbox, DDS session/signaling contract와 same-host 4-process DDS smoke를
  증명한다.
- Genesis GPU offscreen capture, aiortc encode/decode latency, 실제 ICE 선택,
  Coturn relay, 두 컴퓨터 사이 orbit/pan/zoom 반응성은 아직 증명하지 못했다.
- 필요한 근거: direct LAN 1회와 TURN relay 1회에서 두 영상, pause/step/reset,
  제한된 command backlog, 양쪽 process restart 후 reconnect를 확인하는 것이다.

### P1. DDS 보안과 원격 설치는 live 검증이 필요함

- 상태: open, 사용자 설명서에 기록한 운영상 한계다.
- State schema v7, 비밀값 없는 connection topology, 분리된 DDS/SSH endpoint,
  SROS2 Authority generation, host별 role bundle, pinned SSH host key,
  all-host activation/rollback transaction은 구현하고 software test했다.
  `external` keystore는 `managed` generation과 계속 구분된다.
- `elesim-connections`는 전체 Authority를 조작 노트북에만 보관하고 각 host에는
  공개 자료와 배정 역할 enclave만 전달해야 한다. 부분 배포는 시작 전에
  fail-closed하거나 이전 generation으로 rollback해야 하며 SSH hostname/port에서
  DDS locator를 추론하면 안 된다.
- `trusted-network`는 DDS 암호화가 없고 명시적인 LAN/VPN interface 및 firewall
  경계 안에서만 허용된다. `ROS_DOMAIN_ID`는 보안이 아니다.
- Managed Coturn은 REST HMAC secret을 Coturn과 같은 host의 Simulator에만
  mount하고, Simulator가 session-bound 단기 credential을 발급한다. UI에는
  secret을 절대 전달하지 않는다. External TURN은 별도 provisioned credential을
  사용할 수 있다.
- 이전 in-process `UnitreeRos2Bridge`의 security/context 충돌은 software에서
  해결했다. 전용 `elesim-unitree-bridge` daemon이 분리된 private NIC/domain의
  stock local/plaintext Unitree DDS를 소유하고, Robot은 credential을 확인하는
  bounded Unix IPC만 사용하면서 유일한 inter-host SROS2 participant로 남는다.
  Unitree topic은 Elesim policy에 추가하지 않았다. 남은 근거는 실제
  Jetson/GO2에서 NIC 격리, 계정/ACL 설정과 bridge 단절·잘못된 packet 시 stop
  deadline을 검증하는 것이다.
- 필요한 live 근거: 실제 host browser flow, 무권한 publish/subscribe 거부를
  포함한 production RMW/SROS2 enforce, 기본값이 아닌 SSH 관리 port와 pinned-key
  실패, 전체 generation rotation/rollback, managed/external Coturn lifecycle,
  정확한 cleanup 명령이다.

### P1. 고정 self-hosted container의 host/GPU 검증이 필요함

- 상태: live Docker 검증에 대해 open이다. generator와 ownership guard는
  구현했다.
- 일반판은 고정 `elesim-runtime` role container를 쓰고 개발자판은 상시
  `elesim-dev`와 선택적 `elesim-jaeger`만 사용한다. 외부 개인 Compose 환경은
  공식 test path가 아니다.
- 필요한 근거: Ubuntu/WSL clean install/build/start/down, 반복 `elesim-dev`
  shell에서 임시 container가 늘지 않음, 고정 이름 충돌 진단, NVIDIA/CPU variant,
  GUI forwarding과 Jaeger다.

### P2. 광범위한 runtime fallback은 계속 감사해야 함

- 상태: open.
- 현재 deployments와 misc/tooling 전체에 broad exception handler가 약 267개 있다.
  optional driver/UI fallback도 포함하지만 모두 관측 가능하고 안전하다고 가정할
  수 있는 수량은 아니다.
- 특히 camera, Genesis, ROS2, telemetry에서 silent fallback을 typed expected
  error와 구조화된 endpoint/UI health로 계속 바꿔야 한다.

### P2. Physical adapter의 headless coverage가 낮음

- 상태: 의도적으로 open이며 숨기지 않는다.
- 순수 제어 코드는 UV 94%, LJI 91%, workflow 87%, replay 93%, robot runtime
  79%로 강하게 실행된다. UI panel, RealSense/YOLO, Dynamixel transport,
  Unitree bridge, Genesis camera operation은 낮다.
- `misc/docs/audit/2026-07-20/coverage.md` 참고. 이 영역은 더 깊은 mock이 아니라
  integration rig가 필요하다.

### P2. Genesis 및 upstream dynamics warning이 남음

- 상태: open, 현재 non-fatal.
- GO2 neutral qpos와 neutral self-collision filtering은 live contact/dynamics로
  판단해야 한다. inertia frame 해석도 미결이다.
- `hppfcl` -> `coal` warning은 Pinocchio/convex-MPC dependency chain에서 오며,
  Elesim은 `hppfcl`을 직접 import하지 않는다.

### 2026-07-20 refactor에서 닫힌 항목

- model bundle은 self-contained, hash 검증, runtime immutable 상태다.
- Robot은 physical I/O, measured canonical `q`, deadman/current/read-failure safety,
  stale sequence와 lease enforcement를 소유한다.
- Controller/Simulator는 direct protocol-v4 endpoint를 사용한다. sibling role의
  복사 구현을 제거했고 import boundary를 테스트한다.
- payload/lifecycle/trace/reconnect/partial command/async UI/Pick stop/camera
  lifecycle/WebRTC signaling에 contract test가 있다.
- 핵심 알고리즘에 deterministic property/headless/replay test가 생겼고, 일곱
  critical mutant를 모두 테스트가 잡는다.
- deployment class는 1000줄, function은 900줄을 넘을 수 없다. 큰 workflow
  파일은 method마다 무한 분할하지 않고 책임 section으로 나눴다.
- 생성된 모든 release를 sibling deployment와 source-tree import가 없는 임시
  위치에 설치해 probe한다.

### 역사적 기록: 2026-07-21 remote-simulator refactor에서 닫힌 항목

아래는 교체 전 ZMQ/Router 구현의 기록이다. 해당 transport와 credential 선택은
최종 ROS 2/DDS 아키텍처에 포함되지 않는다.

- non-loopback Router와 RGBD transport는 기본적으로 CurveZMQ를 요구하며,
  Router는 public key, endpoint ID, role의 정확한 조합을 인증한다.
- UI는 Controller를 camera-input relay로 쓰지 않고 독립 Simulator session을
  소유한다.
- Simulator는 observer/hand-eye WebRTC view를 분리해 보내고 orbit, pan, zoom,
  pause/resume, step, reset, speed, marker 명령을 Genesis main thread에서 적용한다.
- Coturn REST credential은 Router가 짧은 수명으로 발급하며 static secret과
  private key 생성물은 Git에서 제외된다.
- TURN 갱신은 기존 simulation session 안에서 UI의 두 peer를 교체하며, 로컬
  교체 실패 시 작동 중인 receiver를 보존한다.

---

## Refactor 이전 역사적 backlog

이 아래는 현 상태에 도달한 과정을 보존한 기록이다. 위의 기준 절에도 등장하는
항목만 현재 이슈로 취급한다.

## 전반적/잠재적 코드베이스 리스크 (2026-07-01)

아래 항목들은 현재 코드 구조와 최근 테스트 결과를 보며 확인한 넓은 범위의 리스크다. 반드시 현재 실패라는 뜻은 아니지만, "모든 테스트 통과" 뒤에도 실제 통합 문제가 남을 수 있는 지점들이다.

### 1. Headless 테스트만으로 물리 폐루프 성공을 증명하지 못함

- 상태: open.
- 맥락: Docker pytest는 현재 통과한다 (`299 passed, 1 warning`). 다만 테스트 대부분은 contract, synthetic scenario, mock service 기반이다.
- 리스크: Look/Aim/LJI/Grasp가 수학/오케스트레이션 테스트는 통과해도 Genesis 물리, 실제 카메라 타이밍, depth jitter, 하드웨어 latency에서 실패할 수 있다.
- 추가할 것:
  - `host.py`, `sim.py`, `ctrl.py` 또는 pick service equivalent를 시작하고 완전한 pick log를 검증하는 반복 가능한 sim smoke run.
  - `[Visual]`, `[Perception]`, `[Grasp-Ctrl]`, `[Grasp] blind` 시퀀스를 대상으로 한 recorded-log replay 테스트.
  - helper output뿐 아니라 물리적 수렴에 대한 pass/fail 기준.

### 2. Entry point와 behavior module이 아직 너무 큼

- 상태: open.
- 근거:
  - `engine/behaviors/pick/actions.py`: 약 9341줄.
  - `sim.py`: 약 3055줄.
  - `host.py`: 약 2335줄.
  - `engine/vision/perception/capture.py`: 약 1831줄.
  - `engine/core/config_loader.py`: 약 1465줄.
- 리스크: state ownership과 failure path를 감사하기 어렵다. 작은 수정이 Look, Aim, Grasp, perception, UI 동작을 의도치 않게 결합할 수 있다.
- 제안 방향:
  - `ControlService`는 coordinator로 두되 pick action을 phase별로 분리한다 (`look`, `aim`, `equal_sag`, `grasp_lji`, `blind_finish`).
  - host transport, command arbitration, trajectory scheduling, hardware IO를 별도 모듈로 옮긴다.
  - phase-level state object를 추가해 로그와 테스트가 transition을 명시적으로 검증하게 한다.

### 3. 광범위한 exception handling이 실제 fault를 숨길 수 있음

- 상태: open.
- 근거: repository scan 기준 `host.py`, `sim.py`, `engine/` 아래에 `except Exception` 패턴이 대략 185개 있다.
- 리스크: detector, socket, IK solve, camera update, GO2 bridge 실패가 조용히 fallback으로 내려가 정상 동작처럼 보일 수 있다.
- 감사할 것:
  - 예상 가능한 optional dependency 실패는 typed exception으로 대체한다.
  - runtime path의 silent `pass` 블록을 structured debug/status message로 바꾼다.
  - recovery/fallback mode를 host state에 노출해 UI/test GUI에서 보이게 한다.

### 4. 환경 parity가 약함

- 상태: open.
- 근거:
  - 현재 shell에서는 local `python3 -m pytest`가 불가능하지만 Docker pytest는 통과한다.
  - `scipy` 같은 dependency가 없으면 local direct import가 실패할 수 있다.
  - GO2 MPC import는 upstream `hppfcl` -> `coal` deprecation warning을 낸다.
- 리스크: laptop, Docker, Jetson이 미묘하게 다른 dependency stack을 실행할 수 있다. 한 환경의 테스트 결과가 다른 환경을 대표하지 않을 수 있다.
- 제안 방향:
  - 공식 test runner와 공식 runtime runner를 하나씩 문서화한다.
  - 필요 시 `pytest`, `numpy`, `scipy`, `genesis`, `pinocchio`, `convex_mpc`, camera dependency, ROS2/Unitree dependency를 확인하는 environment smoke command를 추가한다.
  - `hppfcl` warning은 pick pipeline defect가 아니라 dependency hygiene으로 추적한다.

### 5. Config/profile drift와 네트워크 topology의 ownership이 더 강해야 함

- 상태: open.
- 맥락: `configs/config.yaml`는 localhost endpoint를 쓰고, `configs/config.pc.yaml`와 `configs/config.jetson.yaml`는 명시적인 PC/Jetson TCP endpoint를 가진다.
- 리스크: 올바른 process를 잘못된 profile로 실행하면 stale host에 연결하거나, stale preview endpoint를 쓰거나, traffic이 local loopback인지 Wi-Fi LAN인지 ROS2/Unitree transport인지 불명확해질 수 있다.
- 제안 방향:
  - `host.py`, `sim.py`, `ctrl.py` 시작 시 active config file, bind/connect endpoint, mode를 보여주는 startup profile banner를 출력한다.
  - laptop과 Jetson에서 어떤 process가 도는지 profile matrix로 문서화한다.
  - PC profile이 Jetson-only endpoint를 bind하는 식의 불가능한 조합을 startup handshake에서 거부한다.

### 6. Generated runtime artifact에 provenance check가 필요함

- 상태: open.
- 맥락: runtime 동작은 combined URDF 같은 `crafts/` 아래 생성 파일에 의존한다. 최근 linear-zero 작업도 regenerated URDF semantics에 의존한다.
- 리스크: source가 바뀌어도 stale generated URDF가 남아 code/config와 맞지 않는 runtime을 만들 수 있다.
- 제안 방향:
  - 생성 URDF/manifest에 source config hash 또는 build timestamp를 찍는다.
  - `crafts/robot.urdf`가 관련 asset/config보다 오래되면 startup check가 경고하게 한다.
  - linear-zero check를 static unit test뿐 아니라 runtime smoke test에도 포함한다.

### 7. Sim/real command semantics에 boundary test가 필요함

- 상태: open.
- 맥락: 이전 작업에서 simulation-only motion behavior, host trajectory scheduling, real/feedback state path를 분리했지만 경계가 여전히 미묘하다.
- 리스크: sim은 commanded q를 따라 안정적으로 보일 수 있지만, Real2Sim이나 실제 하드웨어는 measured q, lag, source ownership에 의존한다.
- 확인할 것:
  - 각 source (`slider`, `ik`, `lji_step`, `perception`, `sim`)에 대해 host가 trajectory smoothing을 적용하는지, direct target을 쓰는지, 무시하는지 검증한다.
  - Grasp/LJI feedback path에서 `sim_q`와 command `q` 선택을 확인한다.
  - measured q가 commanded q보다 lagging하는 replay test를 추가하고 LJI가 과적분하지 않는지 확인한다.

### 8. Perception/camera lifecycle coupling이 여전히 위험함

- 상태: open.
- 맥락: Grasp/LJI는 지속적인 center/depth update에 의존한다. 이전 manual run에서는 grasp/stop transition 주변에서 camera/perception이 멈추거나 튀는 현상이 있었다.
- 리스크: UI stop, pick stop, blind handoff, recovery path가 operator가 "motion만 stop"할 의도였는데 perception까지 멈출 수 있다.
- 제안 방향:
  - detector, tracker, preview stream, sim-camera relay, pick phase에 명시적인 lifecycle state를 정의한다.
  - Look, Aim, LJI, blind finish, user stop 전반에 대해 "motion은 멈추지만 camera는 유지" 테스트를 추가한다.
  - camera/perception state를 side effect가 아니라 로그와 UI status에 노출한다.

### 9. 핵심 알고리즘에 property/stress test가 부족함

- 상태: open.
- 맥락: 현재 테스트는 알려진 scenario와 invariant를 고정하지만, 대부분의 알고리즘이 넓은 random/adversarial input set에서 테스트되지는 않는다.
- 리스크: IK, LJI, equal-sag, feasible-ready, UV control이 수작업 fixture에 없는 edge geometry에서 실패할 수 있다.
- 제안 방향:
  - reachable/unreachable IK target, noisy depth, singular Jacobian, joint-limit case에 대해 deterministic random seed를 추가한다.
  - "error 감소", "ill-conditioned input reject", "command cap 준수", "reason 보고" 같은 contract를 검증한다.
  - 수학적으로 답이 하나인 경우에만 exact numeric expected output을 쓴다.

### 10. Dependency deprecation: `hppfcl` -> `coal`

- 상태: open, 낮은 우선순위.
- 맥락: 전체 테스트는 통과하지만 GO2 MPC import test에서 `Please update your 'hppfcl' imports to 'coal'` warning이 나온다.
- 현재 해석: repo가 `hppfcl`을 직접 import하지는 않는다. warning은 `convex_mpc`/Pinocchio dependency chain에서 나오는 것으로 보인다.
- 제안 방향:
  - 이것을 pick 또는 engine behavior failure로 취급하지 않는다.
  - Docker/Jetson dependency stack을 업데이트할 때, 또는 warning이 import failure로 바뀌면 다시 본다.

## Look-Aim-Grasp 검증 backlog (2026-06-29)

아래 항목들은 반드시 코드 결함이라는 뜻은 아니다. pick pipeline을 안정적이라고 보기 전에 real/sim loop에서 검증해야 하는 다음 behavior들이다.

### 0. Grasp end-to-end가 아직 검증되지 않음

- 상태: open, 최우선.
- 맥락: Look과 Aim은 여러 번 조정됐지만, 최신 변경 이후 전체 Look -> Aim -> Grasp 시퀀스는 아직 검증되지 않았다.
- 확인할 것:
  - LJI approach가 target을 충분히 오래 시야에 유지하는가.
  - `remain`이 blind handoff를 정당화할 정도로 단조롭게 감소하는가.
  - look axis가 크게 틀어진 상태에서 blind finish가 시작되지 않는가.
  - gripper가 물리적으로 그럴듯한 pre-contact target에 도달한 뒤에만 닫히는가.
- 필요한 근거: `[Pick] look`, `[Aim]`, `[Pick] equal_sag`, `[Grasp-Ctrl]`, `[Grasp] blind` section을 포함한 complete run log 하나.

### 1. Look preferred-vector selection은 live validation 필요

- 상태: open.
- 맥락: UI가 이제 `Ball dir`을 받고, Look은 fallback rough pre-aim 전에 해당 vector를 먼저 시도한다. 코드는 테스트됐지만 behavior는 테스트되지 않았다.
- 확인할 것:
  - `Ball dir` convention이 실제 사용에서 직관적인가: `ready/camera -> object`.
  - 정상 target placement에서 preferred vector가 불필요한 pre-aim 없이 성공하는가.
  - 나쁜 preferred vector가 끔찍한 view pose를 만들지 않고 pre-aim으로 graceful fallback하는가.
- 필요한 근거: preferred-vector success와 fallback case를 모두 보여주는 로그.

### 2. Rough pre-aim fallback은 tuning이 필요할 수 있음

- 상태: open.
- 맥락: fallback pre-aim은 의도적으로 `u=+0.10, v=0.00`을 넓은 tolerance로 대충 aim한다. 목표는 terrible Look seed를 피하되 over-optimization하지 않는 것이다.
- 확인할 것:
  - Look 전에 너무 많은 step을 쓰지 않는가.
  - 너무 잘 center해서 equal-sag signal을 없애지 않는가.
  - 실패한 preferred-vector Look보다 더 크게 arm을 휘두르지 않는가.
- 주요 config: `look_pre_aim_max_steps`, `look_pre_aim_target_uv_u`, `look_pre_aim_tol`, `look_pre_aim_step_scale`.

### 3. Aim motion taper는 real camera validation 필요

- 상태: open.
- 맥락: Aim cap은 이제 남은 UV error에 따라 taper되므로 target 근처에서 motion이 줄어야 한다. Unit test는 cap math만 다룬다.
- 확인할 것:
  - 큰 initial error에서도 충분히 빠르게 수렴하는가.
  - near-target motion이 더 이상 overshoot하거나 눈에 띄게 swing하지 않는가.
  - divergence/stuck recovery가 여전히 필요한 때 trigger되는가.
- 필요한 근거: `delta`, requested roll/seg, equal-sag accept 여부를 보여주는 `[Visual] aim step` 로그.

### 4. Equal-sag acceptance는 여전히 critical gate

- 상태: open.
- 맥락: Aim이 center에 도달해도 `aim centered but equal sag rejected`로 실패할 수 있다.
- 확인할 것:
  - rejection reason이 지나치게 엄격한 threshold가 아니라 실제 geometry failure에 의해 주로 발생하는가.
  - 최신 Look/pre-aim behavior가 sag 추정을 위한 충분한 ready-to-centered drift를 만드는가.
  - accepted equal-sag correction이 Grasp target을 object에서 lateral하게 멀어지게 하지 않는가.
- 주요 config: `sag_drift_max_dir_error_deg`, `sag_drift_max_lateral_m`, `sag_drift_axial_only`.

### 5. LJI grasp controller stability audit 필요

- 상태: open.
- 맥락: LJI path에는 damping/settling 관련 최근 변경이 많다. 테스트는 sample quality와 helper logic을 다루지만 full closed-loop behavior는 다루지 않는다.
- 확인할 것:
  - `dq_cmd`와 `dq_meas`가 camera를 흔들 정도로 sign alternation하지 않는가.
  - `remain`이 blind handoff threshold 근처에서 stall하지 않는가.
  - approach 중 depth validity가 안정적인가.
  - reacquire logic이 main LJI step과 싸우지 않는가.
- 필요한 근거: `u_err`, `v_err`, `z_err`, `dq_cmd`, `dq_meas`, `remain`, transition message를 포함한 `[Grasp-Ctrl]` series.

### 6. Blind handoff threshold가 아직 너무 이른 수 있음

- 상태: open.
- 맥락: 이전 로그에서 `remain ~= 98mm` 부근 blind handoff 후 IK failure와 tolerance 초과 look error가 나타났다.
- 확인할 것:
  - `blind_micro_start_m` / LJI handoff 설정이 lateral/look error가 큰 상태에서 blind mode로 들어가지 않는가.
  - Blind extend/bisect path가 반복 IK failure 없이 close tolerance에 도달할 수 있는가.
  - Post-blind `look_err`가 close 전에 tolerance 아래인가.
- 주요 config: `blind_micro_start_m`, `lij_uv_handoff_m`, `grasp_waypoint_max_dir_error_deg`, `grasp_guided_handoff_m`.

### 7. Linear zero definition은 generated-URDF confirmation 필요

- 상태: mostly fixed, runtime confirmation 하나 필요.
- 맥락: user-facing offset shim 없이 `u_linear=0`이 URDF/prismatic `q=0`으로 mapping되도록 linear q definition을 바꿨다.
- 확인할 것:
  - 재빌드된 `crafts/arm.urdf` / `crafts/robot.urdf`가 `j_plate_housing` upper limit `0.0`을 쓰는가.
  - runtime이 의도한 zero pose에서 linear joint를 시작하는가.
  - IK와 host clamp가 절대 q=0을 넘겨 command하지 않는가.
- 필요한 근거: rebuild 후 startup log, `u=0`과 `u=max` quick sanity check.

### 8. Perception tracking/depth robustness는 별도 리스크

- 상태: open.
- 맥락: pick pipeline은 이제 Aim/LJI 중 안정적인 center UV와 depth에 크게 의존한다.
- 확인할 것:
  - tracker가 근처 detection이나 stale box 사이에서 jump하지 않는가.
  - arm/camera motion 중 depth가 valid하게 유지되는가.
  - mock/sim perception과 real perception이 compatible world-frame convention을 쓰는가.
- 필요한 근거: sudden object-world jump 또는 `depth_valid=false` 주변 perception log.

## Genesis의 inertia interpretation

- 상태: unresolved, basic sim startup에는 non-blocking.
- 맥락: URDF export는 기대한 field (`ixx`, `iyy`, `izz`)에 inertia 값을 쓴다. Genesis는 model을 load하지만 `link.inertial_i`에서 principal moment처럼 diagonal 값 순서를 바꿔 노출한다.
- 관찰 예: `plate_physics.json`과 `crafts/arm.urdf`에는 `ixx=0.0750208333`, `iyy=0.300020833`, `izz=0.375`가 있지만 Genesis debug output은 `diag=[0.375, 0.3000208437, 0.0750208348]`를 보인다.
- 현재 mitigation: `engine/robot/go2/mpc/payload_model.py`는 여러 `link.inertial_i` shape를 처리하고, `ELISIM_DEBUG_INERTIA=1`로 raw inertial attribute를 출력할 수 있다.
- 다음 확인: `ELISIM_DEBUG_INERTIA=1 python3 sim.py`로 실행해 `inertial_R`, `inertial_rot`, `inertial_quat` 같은 inertial frame rotation이 `inertial_attrs`에 있는지 확인한다.
- Genesis가 inertial frame rotation을 노출하지 않으면, payload compensation은 `link.inertial_i` 대신 URDF/physics JSON에서 arm inertia를 직접 읽는다.

## GO2 neutral qpos warning

- 상태: unresolved, 현재 non-fatal.
- 맥락: Genesis는 GO2를 neutral `qpos0=0`에서 build하지만 GO2 calf joint는 negative-only limit를 가진다.
- 관찰 warning: `Neutral robot position (qpos0) exceeds joint limits.`
- 현재 mitigation: `sim.py`는 `scene.build()` 직후 GO2 leg joint를 ready pose로 설정한다.
- 남은 문제: warning 자체는 ready pose 적용 전 build 중에 발생한다.

## GO2 neutral self-collision filter warning

- 상태: unresolved, neutral `qpos0`와 관련된 것으로 보임.
- 맥락: Genesis가 neutral configuration에서 self-collision을 일으키는 일부 geometry pair를 filter한다.
- 관찰 warning: `Filtered out geometry pairs causing self-collision for the neutral configuration (qpos0)`.
- 현재 해석: post-build ready pose가 아니라 invalid neutral GO2 pose 때문일 가능성이 높다.
- 다음 확인: 실제 ready-pose simulation에서 contact가 불안정하거나 기대한 collision이 빠지는 경우에만 다시 본다.

## Update repo selective integration

- 상태: 2026-06-22 기준 mostly integrated.
- 맥락: `/home/user/dev/ws/humble_ws/update/elesim`에는 더 새로운 upstream 작업이 있었다: PC/Jetson split configs, GO2 hardware bridge, remote perception worker, GO2 mirror mode, lazy GO2 MPC imports.
- 제약: 현재 repo에는 local GO2 assets (`assets/go2/`)와 `builders/go2_arm_merger.py`가 추가되어 있으므로 current repo를 무작정 덮어쓰면 안 된다.
- 통합된 것:
  - GO2 local asset layout은 `assets/go2/go2.urdf`와 local DAE mesh를 유지한다.
  - `engine/robot/go2/hardware`는 `/sportmodestate`, `/lowstate` default를 쓰는 `pose_source`, `SportModeState`, `LowState`, 12-DOF leg sync support를 가진다.
  - `engine.core.protocol.pack_state`, `host.py`, `sim.py`가 `go2_leg_q`를 relay한다.
  - `sim.py`는 `go2_locomotion.mirror_from_host`를 지원한다. mirror mode는 MPC를 실행하지 않고 host base pose와 leg q를 따른다.
  - `configs/config.pc.yaml`, `configs/config.jetson.yaml`, `perception_worker.py`, update의 GO2 hardware tests를 추가했다.
- 검증:
  - `python3 -m py_compile sim.py host.py engine/core/config_loader.py engine/core/protocol.py engine/robot/go2/locomotion/config.py engine/robot/go2/hardware/*.py perception_worker.py`
  - `env PYTHONPATH=. python3 -m unittest discover -s tests/scenarios/go2 -p 'test_10_lowstate.py'`
  - `env PYTHONPATH=. python3 -m unittest discover -s tests/scenarios/go2 -p 'test_11_bridge.py'`
- 남은 확인: full PC/Jetson live run은 실제 ROS2 Unitree topic과 host/sim pair execution이 필요하다.
