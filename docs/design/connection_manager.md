# GUI 연결관리자 설계 메모

> 상태: software 구현 완료. 실제 다중 host, SROS2 enforce, VPN/LAN 및 Jetson
> 검증은 수동 gate다.
>
> 갱신일: 2026-08-04

## 1. 배경과 목표

Elesim은 ZMQ Router를 제거하고 ROS 2/DDS 기반의 직접 peer-to-peer 통신으로
전환했다. 따라서 배포 프로그램은 다른 프로그램의 Python 메서드를 직접
호출하지 않는다. 각 프로그램은 versioned wire contract에 따라 다음처럼
주소가 지정된 메시지를 주고받는다.

```text
어느 논리 endpoint에게
어떤 명령/이벤트를
어떤 schema의 parameter와 함께 보낸다.
```

여기서 명령과 parameter schema는 protocol/ROSIDL의 책임이고, 해당 endpoint가
어느 컴퓨터에서 실행되는지는 배치와 네트워크 설정의 책임이다. GUI
연결관리자의 목적은 후자를 사람이 이해 가능한 형태로 배치하고 검증하는 것이다.

연결관리자는 다음을 해야 한다.

- UI, Pilot, Sim, Robot의 실행 host 배치를 보여 준다.
- 각 host의 DDS 네트워크 profile을 일관되게 생성한다.
- 실제 DDS discovery/control/RGBD 연결성을 검사해 준다.
- SSH 관리 경로, DDS 경로, WebRTC/TURN 경로를 혼동하지 않게 한다.

연결관리자는 다음을 하면 안 된다.

- Router나 중앙 application broker로 변하면 안 된다.
- DDS endpoint의 영구 registry/authority가 되면 안 된다.
- endpoint ID를 IP/port에 영구적으로 묶으면 안 된다.
- SROS2 private key, TURN static secret, Tailscale OAuth/auth key를 일반 GUI
  상태 파일에 저장하면 안 된다.

## 2. 현재 runtime의 논리 endpoint 수

현재 배포 프로그램 endpoint 역할은 정확히 네 종류다.

| 역할 | 기본 endpoint ID | 주 책임 |
| --- | --- | --- |
| UI | `ui-main` | 사용자 조작, Pilot 요청, Sim 영상 수신 |
| Pilot | `pilot-main` | 인지, IK, Pick/Gaze, 대상 선택 |
| Sim | `sim-default` | Genesis, 가상 RGBD, UI session, WebRTC 송신 |
| Robot | `robot-go2` | 실제 I/O, 실제 RGBD, deadman/safety |

이는 **논리 endpoint 수**이지 컴퓨터 수가 아니다.

설치기와 연결관리자는 이 네 논리 ID를 기본값으로 사용한다. 컴퓨터 배치는
고정된 컴퓨터 유형 프리셋이 아니라 host 카드와 role assignment를 직접 선택하는
방식이다. 따라서 한 host에 여러 role을 둘 수 있고, endpoint ID를 바꾸는
경우에도 설치 구성과 연결 topology 양쪽에 같은 값을 명시적으로 반영해야 한다.

- 시뮬레이션 전용 구성은 UI+Pilot+Sim를 한 host에 두면 1대도 가능하다.
- 실물 Robot을 포함하면 최소 구성은 Robot Jetson 1대와 나머지 역할을 실행하는
  host 1대, 즉 2대다.
- 현재 제품 범위에서 UI/Pilot/Sim/Robot을 각각 다른 host에 두면
  4대다.
- 미래에는 Robot이나 Sim를 복수로 둘 수 있으므로 protocol 내부의 최대
  endpoint 수를 4로 고정하면 안 된다. 다만 첫 GUI의 host slot을 4개로 제한하는
  것은 현재 범위에서 합리적이다.

### 2.1 명시적 topology mode

연결 상태 파일 schema v3에는 `topology_mode`가 들어간다. schema v1/v2 파일은
하위 호환을 위해 `full` 또는 저장된 mode로 해석한 뒤 v3으로 정규화한다.

| mode | 허용 role | host 수 | 설치 제약 |
| --- | --- | --- | --- |
| `full` | Pilot, Sim, UI, Robot 각 1회 | 2~4 | Robot host만 native/Jetson/systemd |
| `simulation-only` | Pilot, Sim, UI 각 1회 | 1~3 | 모든 host container/Compose, Robot/Jetson 금지 |

`simulation-only`는 물리 Robot이 없다는 사실을 모델에 명시하는 실행 경로다.
Robot을 빈 endpoint로 남겨 두거나 가짜 Jetson host를 만들어 full topology처럼
저장하지 않는다. 따라서 설치기와 배포기는 해당 host의 세 역할만 생성·시작하며,
SROS2 정책도 활성 role 집합만 권한에 포함한다. `full`의 Robot safety와 native
systemd 규칙은 이 mode에서도 완전히 그대로 유지된다.

Sim 내부의 렌더러, Robot/Sim의 RGBD publisher, UI 내부의 session
thread 등은 독립 배포 client가 아니다. 예를 들어 UI의 operator 채널과 sim
채널은 하나의 UI DDS peer/boot identity를 공유한다.

## 3. 반드시 구분할 용어

| 용어 | 의미 | 연결관리자가 저장/표시하는 방식 |
| --- | --- | --- |
| Host | 실제 컴퓨터 또는 VM | 사람이 붙인 이름, 설치/관리 정보 |
| Role | UI/Pilot/Sim/Robot 책임 | Host에 배치하는 블록 |
| endpoint ID | `pilot-main` 같은 논리 주소 | role assignment에 저장 |
| boot ID | 매 실행마다 바뀌는 process incarnation | runtime discovery에서 읽기 전용 표시 |
| DDS locator | 실제 IP, UDP port, transport 정보 | DDS가 discovery로 교환; 고정 설정값이 아님 |
| SSH 주소/포트 | 설치 GUI forwarding, 원격 설치/로그 접근 | 관리용 정보; DDS/WebRTC runtime 주소가 아님 |
| TURN URL/credential | WebRTC media relay 설정 | Sim/active UI session용; DDS용이 아님 |

특히 `endpoint_id`가 사용자가 말한 “X번 자리”에 해당한다. IP와 UDP port는
그 endpoint가 현재 어느 host에서 실행되는지에 따라 바뀔 수 있으며, boot ID는
재시작 때마다 바뀐다.

## 4. 현재는 어떻게 통신하는가

### 4.1 이전 Router/ZMQ 구조

이전에는 각 호스트(노트북, compute host, Jetson)가 중앙 Router에 outbound ZMQ 연결을
유지했다.

```text
Laptop ── outbound connection ──> Router <── outbound connection ── Compute host/Jetson
```

따라서 NAT 뒤 노트북은 Router로 나가는 연결만 만들 수 있으면 되었고, Router는
그 기존 연결을 통해 노트북으로 메시지를 중계할 수 있었다. 이는 polling이 아니라
지속적인 양방향 연결을 Router가 중계한 것이다.

### 4.2 현재 DDS P2P 구조

현재 Router는 없다. 각 role은 DDS participant이며, discovery가 `EndpointDescriptor`
와 heartbeat를 통해 상대를 찾는다. 발견 뒤 peer는 상대가 광고한 boot-specific
DDS locator로 직접 UDP를 보낸다.

```text
UI          <── DDS ──> Pilot
UI          <── DDS ──> Sim
Pilot  <── DDS ──> Sim/Robot
Sim   <── DDS ──> Robot (필요한 구성에서)
```

RGBD도 `RgbdFrame` DDS topic으로 직접 전달한다. Observer/hand-eye 영상 pixel만
WebRTC DTLS/SRTP이며, WebRTC offer/answer signaling은 DDS control carrier로
직접 전달된다.

같은 L2 LAN에서는 multicast discovery가 locator를 자동 교환한다. multicast가
라우팅되지 않는 환경에서는 CycloneDDS static peer에 도달 가능한 hostname/IP를
discovery seed로 넣는다. static peer는 relay가 아니며, 실제 제어/RGBD traffic은
여전히 peer 사이에서 직접 오간다.

## 5. NAT와 포트포워딩의 한계

DDS P2P에서는 필요한 모든 peer 쌍이 양방향 UDP로 서로 도달 가능해야 한다.
노트북이 NAT 뒤라서 아래처럼 outbound만 가능한 경우는 지원되는 topology가 아니다.

```text
Operator host -> Compute host/Jetson : 가능
Compute host/Jetson -> Operator host : 불가능
```

노트북이 UDP packet 하나를 outbound로 보낸다고 해도 그것만으로는 충분하지 않다.
NAT mapping은 시간 제한이 있을 수 있고, 상대는 공인 mapping address/port를 알아야
하며, DDS가 사설 locator를 광고할 수 있고, symmetric NAT는 destination마다
mapping을 달리할 수 있다. 현재 Elesim DDS에는 STUN/ICE/hole-punching/DDS relay가
없다.

다음은 DDS NAT traversal 해결책이 아니다.

- 한 호스트만 포트포워딩하기
- SSH `-L` 또는 `-R` tunnel 하나 만들기
- TURN만 켜기
- static peer만 입력하기

SSH `2222`는 installer GUI와 설치/관리 transfer 용도다. TURN은 WebRTC media만
relay하며 DDS discovery, control, RGBD, signaling을 relay하지 않는다.

## 6. 권장 네트워크: Tailscale mesh VPN

현재 조건에서는 조작 호스트, Sim host, Robot Jetson을 같은 Tailscale tailnet에
참여시키는 것이 가장 단순한 권장 경로다.

```text
Laptop (UI + Pilot) ─┐
Sim host             ├─ Tailscale mesh VPN
Jetson (Robot)            ┘
```

각 host에서 Tailscale client/daemon은 필요하지만, 노트북에 inbound port forwarding을
열 필요는 없다. DDS에는 다음을 사용한다.

- DDS interface: `tailscale0`
- discovery: 보수적으로 `static`
- static peers: 각 host의 Tailscale IP/hostname
- graph settings: 같은 `system_id`, `domain_id`, RMW, discovery mode,
  security profile

Tailscale은 transport reachability를 제공할 뿐 DDS role authorization을 대체하지
않는다. 본인만 쓰는 통제된 tailnet이라면 firewall/ACL을 전제로
`trusted-network`를 검토할 수 있지만, shared/observable tailnet 또는 shared compute
network에서는 role-scoped SROS2 enforce mode가 필요하다.

Tailscale을 모든 DDS host에 설치할 수 없다면 한 host를 subnet/VPN gateway로 두는
고급 경로가 있다. 그러나 Jetson의 반환 route, source NAT, DDS locator advertisement,
static discovery를 모두 실제로 검증해야 한다. 이것은 현재 Elesim이 자동 생성하거나
지원한다고 주장하면 안 되는 수동 네트워크 과제다.

## 7. GUI 연결관리자

### 7.1 Canvas

초기 GUI는 Scratch 같은 host/role canvas로 구성한다.

```text
[ Host A / Laptop ]       [ Host B / Compute ]      [ Host C ]
  [UI] [Pilot]         [Sim]               unused

[ Robot / Jetson ]
  [Robot]  # 고정
```

- 첫 화면은 요청한 `COM1`, `COM2`, `COM3`, `Robot` slot을 사용하며 각 host의
  표시 이름과 안정적인 host ID는 별도로 변경할 수 있다.
- Pilot, Sim, UI 블록은 일반 Host 카드에 drag & drop 한다.
- `full` mode에서는 Robot 블록을 Robot/Jetson 카드에 고정한다. 현재 Robot은
  Jetson/JetPack native 단독 설치만 허용하므로 일반 container role과 섞지 않는다.
- `simulation-only` mode에서는 Robot 카드를 숨기고 COM1~COM3만 사용한다. COM
  카드는 1~3개까지 활성화할 수 있고, Pilot/Sim/UI 세 블록은 각 한
  번씩만 배치한다.
- 각 Host에는 `unused` 토글이 있으며, unused host는 어둡게 표시하고 생성/검사
  대상에서 제외한다.
- 각 role block에는 endpoint ID를 표시하고 필요하면 수정한다.
- 한 일반 Host에 UI+Pilot+Sim를 함께 배치하는 것은 허용한다.

### 7.2 Host 설정

일반적인 “IP와 forwarded application port” 입력란 대신 다음을 분리한다.

| 설정 | 예시 | 의미 |
| --- | --- | --- |
| Host label | `Laptop`, `Compute` | 화면 표시용 |
| role assignment | UI, Pilot | 해당 host에 설치/실행할 역할 |
| DDS interface | `eth0`, `wlan0`, `wg0`, `tailscale0` | DDS가 사용할 NIC |
| DDS 광고 주소 | `100.x.y.z` (포트 없음) | static mode에서 다른 host의 discovery seed로 자동 파생 |
| SSH management address/port | `server.example:2222` | installer/로그/관리용 |
| WebRTC/TURN | managed/external TURN | Sim media 구성용 |
| security profile | trusted-network/SROS2 | graph 보안 profile |

DDS UDP port forwarding은 일반 입력란으로 노출하지 않는다. 실제 UDP locator/port는
RMW/DDS가 선택·광고하며 고정된 app port 모델이 아니다. 연결관리자는 runtime에서
발견된 locator를 진단 정보로 보여 줄 수는 있다.

### 7.3 Jetson 없는 두 호스트 사전 점검 (M2-A)

Jetson을 실제로 켤 수 없는 동안에는 `COM` 카드에서 활성 상태인 **정확히 두
호스트**만 골라 `두 호스트 점검`을 실행할 수 있다. 이 점검은 저장·배포하지 않는
role-neutral 문서로 다음만 확인한다.

- 각 호스트의 DDS 광고 주소가 hostname/IP이고 포트를 붙이지 않았는가
- 각 호스트의 DDS 인터페이스가 `tailscale0`처럼 한 개의 NIC 이름인가
- local 호스트는 SSH endpoint가 없고, remote 호스트는 명시적인 SSH host/user/port를
  갖는가
- 요청한 경우 remote SSH host-key probe가 그 host와 port에 도달하는가

사전 점검의 SSH 포트는 Tailscale의 별도 “DDS 포트”가 아니다. 현재 Elesim은
일반 OpenSSH/Paramiko 경로를 사용하므로 실제 `sshd` 포트를 입력한다. 기본 sshd가
22이면 22를 쓰고, 다른 포트를 쓰면 그 값을 그대로 입력한다. SSH fingerprint는
정식 저장/배포 topology에서 다시 고정해야 하며, 사전 점검 결과는 저장하지 않는다.

`python3 -m http.server 8080` 같은 임시 HTTP 확인은 사람이 tailnet 경로를 확인하는
데 사용할 수 있지만 연결관리자 topology에는 입력하지 않는다. 8080은 DDS, SSH,
WebRTC의 runtime 포트가 아니다.

이 점검은 DDS participant discovery, control/RGBD sample, SROS2 권한, WebRTC media,
NAT traversal을 증명하지 않는다. Robot이 필요하면 `full` mode의 Robot role을
포함한 2~4 host topology를 사용하고, 물리 장비가 없으면 `simulation-only` mode로
1~3 host topology를 저장·배포한다.

### 7.4 Global graph 설정

모든 참여 host가 호환되어야 하는 값은 canvas 밖의 공통 graph profile로 둔다.

```text
system_id
ROS_DOMAIN_ID
RMW implementation
discovery mode
DDS security profile
```

연결관리자는 각 role별 endpoint ID와 host assignment를 저장하되, boot ID나 실제
UDP locator를 설정값으로 저장하지 않는다.

## 8. 연결 검사 UX와 현재 범위

단순 ICMP ping은 보조 정보일 뿐 DDS 성공을 보장하지 않는다. GUI의 “연결 점검”은
다음 계층으로 보여 주는 것이 좋다.

1. 선택한 interface, DNS/IP, VPN/Tailscale 상태 확인
2. 선택 사항으로 SSH 관리 연결 확인
3. DDS `EndpointDescriptor`/heartbeat 상호 발견 확인
4. directed control round-trip, motion lease, simulation session 확인
5. active RGBD frame 하나 수신 확인
6. UI의 실제 WebRTC offer/answer/decoded-frame 검증은 별도 live test로 표시

현재 배포 transaction은 각 host에서 `elesim-net doctor --json`을 실행해 설정,
graph, topic 표면을 검증한다. `--active`의 RGBD sample 검사는 별도 운영 명령이다.
아래 edge matrix와 directed control/motion round-trip은 아직 실제 다중 host 수동
검증 항목이며 GUI가 성공했다고 추정해서는 안 된다.

연결관리자는 로컬 `tailscale0`의 현재 IPv4를 읽기 전용으로 제안할 수 있지만
Tailscale 설치·로그인·ACL을 변경하지 않는다. 저장된 topology에는 DDS 주소와 SSH
관리 주소/포트를 별도로 남긴다. 저장 이후에는 `check`, `start`, `stop`, `restart`
작업으로 각 host의 Compose/systemd 관리 상태를 확인·조작할 수 있으며, SSH 성공을
DDS discovery나 WebRTC media 성공으로 해석하지 않는다.

```text
Operator host <-> Sim host: DDS discovery / control / RGBD / WebRTC signaling
Laptop <-> Jetson: DDS discovery / control / RGBD
Sim host <-> Jetson: 필요한 역할 조합에 따른 DDS reachability
Operator host <-> Sim host: WebRTC media는 별도 상태
```

실패 원인은 최소한 다음으로 구분한다.

- interface/VPN route 불일치
- `system_id`/domain/RMW/security profile 불일치
- static peer 누락 또는 multicast 미도달
- firewall 또는 양방향 UDP 미도달
- NAT-only topology
- duplicate endpoint ID 또는 stale boot
- SROS2 permission denial

## 9. 운영과 보안 경계

- `elesim-up`, `elesim-status`, `elesim-logs`, `elesim-down`은 **현재 host의
  설치 prefix/Compose project**만 관리한다. 원격 host 로그는 그 host에서 별도로
  실행하거나 SSH로 접근해야 한다.
- `elesim-logs`는 DDS endpoint나 중앙 logger가 아니라 `docker compose logs -f`
  래퍼다. native Robot은 systemd/journal 로그가 대상이다.
- 중앙 logger/observability collector는 유용하지만 이후 과제로 둔다. 추가하더라도
  DDS broker나 connection manager authority가 아니라, OTLP 같은 단방향 관측 sink여야
  한다. Collector 장애가 control/RGBD/WebRTC를 멈추게 하면 안 된다.
- Tailscale OAuth/auth key, Authority private key, TURN static secret은 GUI
  topology/소스 저장소에 넣지 않는다. Authority는 운영 컴퓨터의 mode-0700
  경로에만 남고, 각 host에는 자기 역할 enclave가 든 제한 bundle만 전달한다.

## 10. 구현된 범위와 남은 gate

`elesim-connections`는 loopback/token GUI, role placement, mode-0600 topology,
SSH host-key pinning, trusted-network 전체 배포, managed SROS2 발급·전체 generation
전환·rollback, read-only Tailscale hint와 bounded lifecycle status/actions를 구현한다.
DDS static peer는 활성 host의 광고 주소에서 파생한다.

다음은 구현 성공으로 간주하지 않는다.

1. 실제 LAN/VPN에서의 양방향 DDS discovery/control/RGBD
2. SROS2 enforce에서 허가·거부가 의도대로 작동한다는 실 RMW 증명
3. 작업 중 process/host 장애를 포함한 다중 host rollback 실증
4. Jetson native Robot와 물리 안전 deadline
5. Tailscale 설치·로그인·OAuth provisioning 자동화

이 gate의 구체적인 절차와 판정 기준은 `docs/setup.md`,
`docs/deployment.md`, `docs/OPEN_ISSUES_KR.md`가 관리한다.
