# 배포와 릴리스

현재 배포는 General container 역할, Developer all-project container, native
Jetson Robot의 세 경계를 갖는다. 설치·로컬 lifecycle은
[`setup.md`](setup.md), 프로세스 계약은 [`architecture.md`](architecture.md)를
참조한다.

## 1. 릴리스 산출물

저장소 루트에서 release context를 만든다.

```bash
python3 misc/tools/release/build.py
python3 misc/tools/release/verify.py dist/releases
```

`dist/releases/`에는 다음 네 애플리케이션 tree와 infrastructure tree가
생긴다.

```text
dist/releases/
├── pilot/       # Pilot wheel + protocol + ROSIDL + config
├── sim/         # Sim wheel + protocol + ROSIDL + prebuilt model bundle
├── ui/          # UI wheel + protocol + ROSIDL + config
├── robot/       # Robot + Unitree bridge + exactly two systemd units
└── infra/       # setup/bootstrap/connection-manager/container inputs
```

`infra`는 Router도 다섯 번째 runtime application도 아니다. `environment/coturn`
은 외부 TURN operator input이고 release tree가 아니다. Docker Desktop의
`tailscale` service도 host network infrastructure다.

Release builder는 각 context에 application wheel, transport-neutral support
wheel, ROSIDL source, config, dependency pins와 deployment metadata를 넣고,
Sim에는 ZED Mini `default`와 D435 `d435` immutable model bundle을 모두 넣는다.
기본 검증은 clean temporary target에
각 wheel을 설치하고 sibling visibility, config/model, role entrypoint
`--help`를 확인한다. Robot은 bridge/IPC module, 두 console script와 정확히
`elesim-robot.service`, `elesim-unitree-bridge.service`를 추가로 확인한다.

일반 사용자는 release context를 수동으로 조합하기보다 설치기가 생성한
prefix/build context를 사용한다. 수동 build는 진단·릴리스 개발용이다.

## 2. General Compose

General은 고정 `elesim-runtime` project와 role별 image/container 이름을
사용한다.

| 역할 | image | container | 실행 경계 |
| --- | --- | --- | --- |
| Pilot | `elesim/pilot:local` | `elesim-pilot` | Docker |
| Sim | `elesim/sim:local` | `elesim-sim` | Docker, amd64 GPU/CPU profile |
| UI | `elesim/ui:local` | `elesim-ui` | Docker |
| Coturn | upstream pinned image | `elesim-coturn` | Sim host의 선택적 WebRTC media service |
| Robot | 별도 native release | systemd units | Jetson only |

설치기는 source config를 prefix에 복사하고 role-specific read-only mount,
Sim model bundle mount, DDS/security 환경을 생성한다. Compose bind path는
local absolute path이므로 remote Docker daemon을 local install처럼 사용하지
않는다. backend는 설치 때 `direct-host` 또는 `tailscale-sidecar`로 고정된다.

### 네트워크 backend

- `direct-host`: native Docker Engine host network namespace에 role/tools가
  참여한다. 선택한 LAN/VPN interface가 그 namespace에 있어야 한다.
- `tailscale-sidecar`: Docker Desktop Linux VM의 고정 `elesim-tailscale`이
  kernel-mode `tailscale0`를 제공하고 role, runtime-network doctor, active
  Sim-owned Coturn이 `network_mode: service:tailscale`로 같은 namespace를
  쓴다. 일반 tools는 enrollment 전에도 동작한다.

sidecar는 Router, DDS relay, SSH endpoint, authorization service가 아니다.
Tailscale browser/device login은 한 번만 필요하며, sidecar IP는 DDS address,
WSL/host IP는 SSH management address가 될 수 있다.

sidecar를 최신 stable 버전으로 올리려면 `elesim-tailscale update`를 사용한다.
이 명령은 Tailscale이 컨테이너 배포에 권장하는 공식 `stable` image를
pull하고, sidecar와 namespace를 공유하며 당시 실행 중이던 role/Coturn만
안전하게 재생성·재연결한다. 일반 `elesim-up`은 암묵적으로 새 버전을
가져오지 않으므로 운영자가 요청한 시점에만 버전이 바뀐다.

## 3. 설치·업데이트·활성화 순서

```bash
# 각 role host, 해당 prefix에서
elesim-update       # source 검증, artifact 재생성, incremental image build
elesim-down         # 기존 runtime을 중지할 때만
elesim-up --no-build
```

설치 마법사는 build/start를 하지 않는다. `elesim-update`는 실행 중인
container를 교체하지 않으며 topology/security/log/cache를 보존한다.
rebuilt image를 적용하는 단계가 `elesim-up`이다. image/Dockerfile 결함은
down 또는 `--purge`만으로 고쳐지지 않고 update/build가 필요하다.
성공한 update는 설치 UUID가 일치하는 이전 local image 중 태그와 container
참조가 모두 없는 것만 정확한 image ID로 정리한다. 실행 중 container가 아직
참조하는 이전 image와 foreign/upstream image는 보존하며 전역 prune은 하지 않는다.
`elesim-uninstall`은 설치가 만든 runtime/container와 `elesim/*:local` image를
정확히 제거하지만, BuildKit/download cache와 foreign/upstream image는 건드리지
않는다.

전체 multi-host 시작은 connection manager가 모든 host build를 먼저 완료한
뒤 `--no-build` launch를 수행한다. BuildKit plain progress는 host 라벨과
함께 GUI/manager terminal에 전달된다. `docker logs`나 synthetic heartbeat가
아니다.

## 4. DDS 배포 프로파일

한 graph의 모든 participant는 다음을 일치시켜야 한다.

- `system_id`, `domain_id`, pinned `rmw_cyclonedds_cpp`
- discovery mode와 static peer seed
- DDS bound interface와 직접 도달 가능한 advertised address
- protocol version, QoS, security profile

L2 LAN에서는 multicast discovery, routed LAN/VPN에서는 직접 도달 가능한
static peers를 쓴다. static peers는 discovery seed이고 sample relay가 아니다.
일반 IPv4 NAT/CGNAT/symmetric NAT은 지원하지 않으며 SSH port-forward도 DDS
locator를 제공하지 않는다.

### 보안 profile

`trusted-network`는 소유 LAN/routed VPN과 firewall boundary에서만 사용한다.
공유·관찰 가능한 네트워크는 `sros2` enforce를 사용한다.

```bash
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_KEYSTORE=/etc/elesim/keystore
exec <role-command> --ros-args --enclave <role-enclave>
```

`external` provisioning은 operator-supplied keystore를 유지한다.
`managed` provisioning은 operator laptop의 Authority에서 generation을 만들고
common public material과 host-assigned role enclave만 배포한다. CA private key,
aggregate Authority, unrelated role key는 runtime host에 두지 않는다.

managed generation은 `stage → digest/manifest validate → stop → atomic
activate → restart/verify` 순서의 transaction이다. 어느 host에서든 실패하면
이미 적용된 host는 이전 generation으로 rollback한다. pending marker가 있는
host는 generation이 runnable해질 때까지 runtime start를 거부한다.

## 5. 연결 관리자와 토폴로지

`elesim-connections`는 operator laptop에서 non-secret topology를 저장한다.
schema v4는 DDS endpoint와 SSH endpoint를 별도로 보관한다.

| mode | hosts/roles | Robot |
| --- | --- | --- |
| `full` | 2–4 host, Pilot/Sim/UI/Robot 각 1회 | native Jetson unit 필수 |
| `simulation-only` | 1–3 host, Pilot/Sim/UI 각 1회 | 저장하지 않음 |

`simulation-only`의 RGB-D 지연을 줄이려면 두 host 배치를 권장한다. 한
deployment unit에 `[pilot, sim]`, 다른 unit/host에 `[ui]`를 배치한다. Sim은
Genesis RGB-D source이고 source edge에서 encoded sample을 만든다. Pilot은
source DDS topic을 받아 perception과 broker relay를 담당한다(legacy raw source만
Pilot에서 encode). UI는 Pilot broker의 encoded stream만 구독한다.
세 role을 한 host에 두는 기존 local-sim 배치도 유효하지만, source raw RGB-D를
별도 host로 먼저 보내는 배치는 새 배포에서 사용하지 않는다.

schema v1–v3 입력은 읽을 때 v4로 normalize한다(v1은 `full`). 한 host에 여러
role 또는 독립 deployment unit이 있을 수 있다. Robot은 native `robot-native`
unit, Pilot/UI 같은 container role은 별도 `runtime` unit으로 관리할 수 있다.

manager 단계는 다음처럼 구분한다.

```text
check/preflight → topology 저장 → security provision/deploy/rotate
                 → host별 build → start/stop/status
```

`check`는 읽기 전용이고 `start`/`stop`은 Compose/systemd management state를
조작한다. GUI에는 “restart”를 runtime 재구성처럼 제공하지 않는다. 정확한
재시작은 host의 `elesim-down` 후 `elesim-up`이고, 보안/토폴로지 변경은
transaction으로 수행한다.

two-host preflight는 Jetson 없이 정확히 두 COM endpoint를 검사하는 ephemeral
contract다. topology 저장, key issuance, DDS/WebRTC/NAT 증명을 하지 않는다.
HTTP test server, SSH forwarding port, DDS address를 서로 바꾸어 입력하지
않는다.

상세 GUI 흐름·failure state·recovery는
[`design/connection_manager.md`](design/connection_manager.md)에 있다.

## 6. Developer 배포

Developer는 `<workspace>/.elesim/development` 아래 하나의 persistent
`elesim-dev` container를 만들고 `elesim-runtime-dev` project로 관리한다.
개발 source와 venv/home는 persistent하다. `elesim-dev`는 Compose `exec`를
사용해야 하며 random `run --rm` container를 만들지 않는다.

```bash
elesim-up
elesim-dev python3 misc/system_tests/smoke_topology.py
elesim-dev python3 misc/tools/quality/check.py --group required
```

선택적 Jaeger는 `elesim-up --jaeger`로만 추가된다. General role image나
다중 호스트 production artifact로 Developer image를 사용하지 않는다.

## 7. Native Robot Jetson

Robot 설치는 검출된 Jetson에서만 진행한다. Setup은 다음 두 unit과 exact
config를 생성하고, account/group/ACL/systemd 등록 명령을 출력하지만 `sudo`를
대신 실행하지 않는다.

```text
elesim-robot.service          # EleSim DDS/SROS2, hardware, local safety
elesim-unitree-bridge.service # private Unitree DDS/NIC/domain only
```

Bridge는 `elesim-unitree` account로 private NIC/domain(기본 `eth0`, domain
`1`)에 bind한다. Robot은 inter-host DDS/VPN interface에 bind한다. 두 경계는
`/run/elesim-unitree/bridge.sock`의 bounded `SOCK_SEQPACKET`이며, Unitree
topics를 Tailscale/LAN으로 노출하지 않는다.

`dist/releases/robot`은 standalone/manual artifact다. Managed connection
topology에는 native setup으로 생성한 prefix와 두 unit을 사용하고, standalone
release layout을 managed host처럼 등록하지 않는다.

Jetson에서 Robot과 Pilot/UI Compose를 함께 운영하려면 서로 다른 prefix와
deployment unit을 사용한다. Robot unit이 mandatory인 Jetson에서 Sim을
실행하려면 ARM64 이미지/runtime gate를 별도로 통과해야 한다.

## 8. TURN와 WebRTC

Coturn은 Sim이 소유하는 선택적 media infrastructure다. DTLS/SRTP WebRTC
packets와 ICE candidates만 relay하며 DDS discovery/control/RGB-D/signaling은
relay하지 않는다.

managed TURN secret는 Coturn과 co-located Sim에만 mount하고, Sim이 active UI
session에 short-lived credential을 DDS로 전달한다. UI에는 static HMAC secret을
주지 않는다. external TURN credential JSON도 Sim에만 read-only mount한다.
Pilot/UI-only host는 TURN private file을 받지 않는다.

## 9. 로그·상태·소유권

```bash
elesim-status
elesim-logs
elesim-logs --save
elesim-down
```

`elesim-status`는 current host의 container state, IP, GPU reservation/CVD,
DDS interface/security, Sim backend/encoder/display/stream을 요약한다.
다른 host의 상태는 각 host에서 실행하거나 manager topology를 사용한다.

General log는 bounded Docker log와 최대 다섯 개 private snapshot을 유지한다.
`logs/`와 `log/`는 source/repository runtime output과 구분하며, generated
prefix의 log archive는 ownership manifest가 관리한다.

```bash
elesim-uninstall
```

Uninstall은 UUID, exact wrapper/systemd hash, Compose/image labels와 sidecar
ownership을 검증한 뒤 owned resource만 즉시 제거한다. `--keep-logs`와
`--keep-authority`로 보존할 수 있다. Docker prune, broad recursive delete,
foreign image 제거는 배포 절차가 아니다.

## 10. 검증 명령과 수동 수용시험

필수 자동 gate:

```bash
python3 misc/tools/quality/check.py --group required
python3 misc/tools/quality/check.py --group extended
python3 misc/tools/release/build.py
python3 misc/tools/release/verify.py dist/releases
```

`elesim-dev` 환경에서 실행하는 것이 canonical이다. 다음은 여전히 실제 host
수용시험이다.

- 2–4 host의 discovery/control/RGB-D와 lease/session expiry
- SROS2 enforce permission과 generation rollback
- Docker Desktop sidecar의 bidirectional DDS 및 TURN relay candidate
- LAN/routed VPN/global IPv6와 지원되지 않는 NAT의 명시적 실패
- GPU/CPU policy, NVENC/libx264, X11/WSLg Viewer와 observer/hand-eye 화면
- Jetson Unitree bridge deadman, arm cleanup, 물리 Look–Aim–Grasp
