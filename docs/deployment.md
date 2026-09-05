# 배포와 릴리스

현재 배포는 container runtime 역할, 선택적 all-project 개발 attachment, native
Jetson Robot의 세 경계를 갖는다. 설치·로컬 lifecycle은
[`setup.md`](setup.md), 프로세스 계약은 [`architecture.md`](architecture.md)를
참조한다.

## 1. 릴리스 산출물

저장소 루트에서 release context를 만든다.

```bash
elesim-dev python3 workbench/tools/release/build.py
elesim-dev python3 workbench/tools/release/verify.py dist/releases
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

`infra`는 Router도 다섯 번째 runtime application도 아니다. Managed Coturn은
Sim 설치가 생성하고, external TURN 설정은 operator가 별도로 제공한다. Docker
Desktop의 `tailscale` service도 host network infrastructure다.

Release builder는 각 context에 application wheel, transport-neutral support
wheel, ROSIDL source, config, dependency pins와 deployment metadata를 넣고,
Sim에는 ZED Mini `zed-mini`와 D435 `d435` immutable model bundle을 모두 넣는다.
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
container를 교체하지 않으며 topology/security/log/cache를 보존한다. 현재
설치의 build fingerprint와 이미지 label을 갱신하고 이전 dangling image는
소유권 조건 아래 정리한다. rebuilt image를 적용하는 단계가 `elesim-up`이다.
`elesim-up`은 fingerprint가 일치하면 `--no-build`로 시작하고, 이미지가 없거나
fingerprint가 다를 때만 build한다. image/Dockerfile 결함은
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

`elesim-connections`는 operator laptop에서 non-secret topology, host lifecycle,
managed SROS2 rollout을 소유한다. runtime application이나 Router가 아니며 GUI
container에 Docker socket, tailscaled local API 또는 Authority private key를
주지 않는다.

| mode | hosts/roles | Robot |
| --- | --- | --- |
| `full` | 2–4 host, Pilot/Sim/UI/Robot 각 1회 | native Jetson unit 필수 |
| `simulation-only` | 1–3 host, Pilot/Sim/UI 각 1회 | 저장하지 않음 |

schema v1–v3 입력은 읽을 때 v4로 normalize한다(v1은 `full`). 한 host에 여러
role 또는 독립 deployment unit이 있을 수 있다. Robot은 native `robot-native`
unit, container role은 별도 `runtime` unit으로 관리할 수 있다. DDS
address/interface와 SSH address/port/user/fingerprint는 독립 필드이며 어느
한쪽에서 다른 쪽을 추론하지 않는다. static peer는 active DDS address에서만
만든다.

`simulation-only` 권장 배치는 `[pilot, sim]` Compose unit과 별도 `[ui]`
unit이다. Sim source가 encoded sample을 Pilot에 넘기고 Pilot이 broker stream을
UI로 relay한다. 세 role을 한 host에 두는 것도 유효하다. 새 배포에서 raw source
RGB-D를 inter-host consumer가 직접 구독하지 않는다.

### Check와 preflight

`check`는 SSH 및 namespace interface/address/route를 읽기 전용으로 확인한다.
two-host preflight는 Jetson 없이 정확히 두 COM endpoint를 검사하며 topology,
key, generation 또는 role deployment를 저장하지 않는다. 성공해도 DDS
descriptor/heartbeat, SROS2 permission, RGB-D, WebRTC와 NAT를 증명하지 않는다.
HTTP/SSH forwarding/TURN port를 DDS address로 입력하지 않는다.

### Managed security transaction

```text
validate topology/roles
  → generate on operator laptop
  → stage common public + assigned enclaves
  → verify every digest/manifest
  → stop affected roles
  → atomic activate
  → start --no-build and verify
  → commit journal
```

첫 generation은 `provision`한다. active generation이 있으면 반복 provision이나
deploy를 거부하고 `rotate`를 사용한다. 일부 host가 실패하면 이전 generation과
pending marker를 복원한다. 중단된 transaction은 `recover`로 정리한다.

### Lifecycle와 readiness

manager 단계는 다음처럼 구분한다.

```text
check/preflight → topology 저장 → security provision/deploy/rotate
                 → host별 build → start/stop/status
```

`start`는 모든 host build가 성공한 뒤 `--no-build`로 launch한다. host helper와
pinned SSH channel은 allowlisted EleSim command만 실행한다. 실패 시 이번 job이
시작한 role만 rollback한다. `start`/`stop`은 management state이며 runtime
재구성용 restart action은 없다. 명시적 재시작은 정확한 host prefix에서
`elesim-down` 후 `elesim-up --no-build`로 수행한다.

Readiness는 다음 상태를 합치지 않는다.

1. Compose/systemd started
2. namespace interface/address/route valid
3. exact endpoint descriptor + matching boot heartbeat
4. Sim scene/media startup complete
5. authority/session grant와 WebRTC response

Manager의 DDS gate는 3까지만 bounded wait한다. Viewer는 topology SSH user가
소유한 X11 socket/Xauthority만 사용하며 모호하거나 유효하지 않으면 start 전에
실패한다. pending generation, missing heartbeat, remote command failure와 viewer
failure는 서로 다른 recovery 원인을 유지한다.

## 6. 개발 attachment

개발 attachment는 일반 설치의 `elesim-runtime` project에 profile-scoped
`elesim-dev` container 하나를 추가한다. 외부 Git checkout과 설치 prefix의
전용 home/cache는 persistent하다. `elesim-dev`는 Compose `exec`를
사용해야 하며 random `run --rm` container를 만들지 않는다.

```bash
elesim-up
elesim-dev python3 workbench/tests/system/smoke_topology.py
elesim-dev python3 workbench/tools/quality/check.py --group required
```

별도 observability 컨테이너는 배포하지 않는다. runtime role image나
다중 호스트 production artifact로 개발 image를 사용하지 않는다. 개발 셸은
DDS participant가 아니므로 role keystore/enclave를 자동 mount하지 않는다.

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
elesim-dev python3 workbench/tools/quality/check.py --group required
elesim-dev python3 workbench/tools/quality/check.py --group extended
elesim-dev python3 workbench/tools/release/build.py
elesim-dev python3 workbench/tools/release/verify.py dist/releases
```

`elesim-dev` 환경에서 실행하는 것이 canonical이다. 다음은 여전히 실제 host
수용시험이다.

- 2–4 host의 discovery/control/RGB-D와 lease/session expiry
- SROS2 enforce permission과 generation rollback
- Docker Desktop sidecar의 bidirectional DDS 및 TURN relay candidate
- LAN/routed VPN/global IPv6와 지원되지 않는 NAT의 명시적 실패
- GPU/CPU policy, NVENC/libx264, X11/WSLg Viewer와 observer/hand-eye 화면
- Jetson Unitree bridge deadman, arm cleanup, 물리 Look–Aim–Grasp
