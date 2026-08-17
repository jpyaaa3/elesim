# 구성 참조

설치된 구성은 source default와 분리된 prefix 아래 생성된다. 이 문서는 필드의
의미와 ownership을 설명하며, 실제 값은 `install-state.json`, 생성 YAML,
`elesim-status`를 확인한다.

## 1. 구성 계층

```text
source defaults
  ├─ role/config/*.yaml
  ├─ model/bundles/default
  └─ packages/elesim_interfaces
        │ installer generates
        ▼
installed prefix
  ├─ install-state.json / install-ownership.json
  ├─ containers/compose.yaml and build contexts
  ├─ roles/<role>/config, model, security view
  ├─ security/current or external keystore view
  ├─ connections/topology.json (operator laptop only)
  └─ secrets/ and logs/runs/
```

Source config는 설치 중 복사되며 installed config를 수정해도 source default로
되돌아가지 않는다. generated files를 직접 편집한 뒤 `elesim-update`하면
owned artifact가 다시 생성될 수 있다. 변경은 setup GUI, `elesim-net`,
`elesim-connections` 또는 source/config의 정식 경계에서 한다.

## 2. 공통 DDS 필드

모든 역할의 runtime DDS profile은 다음 값을 갖는다.

| 필드 | 의미 | 불변식 |
| --- | --- | --- |
| `system_id` | 논리 시스템 식별자 | 한 graph에서 compatible해야 함 |
| `domain_id` | ROS 2/DDS domain | 보안 경계가 아님 |
| `rmw_implementation` | 현재 `rmw_cyclonedds_cpp` | 모든 peer가 compatible해야 함 |
| `discovery_mode` | multicast 또는 static peers | static peer는 seed일 뿐 relay가 아님 |
| `static_peers` | 직접 도달 가능한 DDS 주소 | SSH/WebRTC port를 넣지 않음 |
| `interface` | DDS bind interface | advertised address가 이 interface에 실제 할당되어야 함 |
| `security_profile` | `trusted-network` 또는 `sros2` | network trust boundary와 일치해야 함 |
| `endpoint_id` | 논리 participant 이름 | role별 unique; boot마다 새 boot ID |
| `enclave` | role-scoped SROS2 enclave | aggregate Authority를 mount하지 않음 |

`trusted-network`는 owned LAN/routed VPN에서만 허용하고, `sros2`는 enforce
authentication/access-control/encryption을 사용한다. `ROS_DOMAIN_ID`만으로
다른 사용자의 participant를 막을 수 없다.

## 3. Topology 필드

connection topology schema v4는 다음을 분리한다.

| 필드 | 의미 |
| --- | --- |
| `topology_mode` | `full` 또는 `simulation-only` |
| `hosts[]` | stable host ID, display name, local flag |
| `dds.address` | DDS advertised IP/hostname (port 없음) |
| `dds.interface` | DDS bind interface, 예: `tailscale0` |
| `ssh.mode` | `openssh` 또는 keyless `tailscale` |
| `ssh.address/port/user` | 독립적인 setup/deployment management destination |
| `ssh.fingerprint` | operator-confirmed host key fingerprint |
| `units[]` | host 안의 독립 prefix/Compose 또는 native unit |
| `roles[]` | 해당 unit의 `pilot`, `sim`, `ui`, `robot` assignment |

`full`은 두–네 host와 네 role을 요구하고 `simulation-only`는 한–세 host와
Pilot/Sim/UI만 허용한다. schema v1–v3 입력은 load 시 v4로 normalize하며
v1은 `full`로 해석한다. DDS 주소와 SSH 주소가 같아도 한 필드에서 다른 필드를
추론하지 않는다.

## 4. GPU와 Viewer

Role별 GPU policy는 `inherit`, `specific`, `cpu`다.

```yaml
gpu:
  policy: specific       # inherit | specific | cpu
  device: "GPU-..."     # specific일 때 index 또는 UUID
viewer:
  enabled: false         # native Genesis Viewer는 명시적 실행만
  display: null
  user: null
```

- `inherit`: daemon/scheduler가 노출한 GPU와 host `CUDA_VISIBLE_DEVICES`를
  따른다.
- `specific`: Compose `device_ids` 한 개만 reservation한다. container 안에서
  host index를 다시 CVD로 재적용하지 않는다.
- `cpu`: GPU request를 만들지 않고 Sim Genesis backend/encoder를 CPU로 한다.

Pilot과 Sim policy는 독립적이다. Viewer는 기본 headless이며 `--view`와 검증된
X11 display/user가 있어야 한다. 원격 SSH username과 X11 session owner가 다르면
manager가 시작을 거부한다.

## 5. TURN/WebRTC

TURN은 Sim-owned optional media infrastructure다.

```yaml
turn:
  mode: none       # none | managed | external
  url: null
  credentials_file: null  # Sim에만 read-only mount
```

`managed` secret는 Coturn과 co-located Sim에만 존재하고 Sim이 active UI
session에 짧은 credential을 전달한다. `external` credential file도 Sim에만
mount한다. UI/Pilot-only host와 DDS transport에는 static TURN secret을
전달하지 않는다.

WebRTC signaling은 DDS PeerEnvelope request/reply이고 픽셀은 DTLS/SRTP다.
TURN URL을 DDS static peer나 SSH endpoint로 입력하지 않는다.

## 6. SROS2 ownership

state schema v9는 `external`과 `managed`를 구분한다.

- `external`: operator가 관리하는 keystore/enclave path. EleSim은 외부
  private material을 삭제하거나 rotate하지 않는다.
- `managed`: connection manager가 만든 generation ID, host bundle, active/
  pending state. operator laptop에 complete Authority가 있고 host에는
  common public material과 배정된 role enclave만 있다.

역할 컨테이너는 `<prefix>/security/roles/<role>`처럼 안정적인 role view만
mount한다. aggregate generation tree와 CA private key는 애플리케이션 mount가
아니다. generation rotation은 모든 host의 staged digest/manifest와 rollback
journal을 남긴다.

## 7. ownership와 비밀

`install-ownership.json`은 UUID와 exact path, wrapper hash, Compose labels,
systemd unit hash, sidecar metadata를 묶는다. Uninstaller는 이 manifest를
검증하기 전에는 mutation하지 않는다.

EleSim이 소유하는 것은 generated runtime config, managed role bundle, optional
TURN/Tailscale state와 bounded logs다. 다음은 소유하지 않는다.

- source checkout, 사용자의 기존 Docker image/layer
- external keystore, external TURN credentials
- Tailscale control-plane device/ACL/account
- host Python/CUDA/ROS/apt 설정

외부 credential, SROS2 private key, generated keystore는 source Git에 커밋하지
않는다. `environment/generated/`는 scratch area다.

## 8. 런타임 점검

```bash
elesim-status
elesim-net namespace-check --dds-interface tailscale0
docker inspect elesim-sim
```

`namespace-check`는 interface/address/route의 structural gate다. 실제 DDS
descriptor/heartbeat, SROS2 permission, RGB-D/WebRTC media는 별도 live gate다.
`elesim-status`의 `gpu.cuda_visible_devices`, device request, `sim.video.*`,
`dds.*`를 함께 읽어 host/container 설정을 혼동하지 않는다.
