# Connection Manager

`elesim-connections`는 operator laptop에서 실행하는 non-secret topology와
호스트 lifecycle/security rollout 도구다. runtime application이나 Router가
아니며, Docker socket·tailscaled local API·SROS2 CA private key를 GUI
container에 주지 않는다.

## 1. 책임과 비책임

### 책임

- topology schema v4의 `full`/`simulation-only` 저장·정규화
- 독립적인 DDS address/interface와 SSH address/port/user/fingerprint 관리
- read-only host check와 ephemeral two-host preflight
- managed SROS2 generation의 provision/deploy/rotate/recover transaction
- host별 build stream과 allowlisted Compose/systemd `start`/`stop`/`check`
- role별 GPU policy, Sim Viewer user/display, Coturn ownership을 deployment에 전달

### 책임 아님

- DDS discovery, authority grant, WebRTC media, NAT traversal을 대신하지 않는다.
- SSH 연결 성공을 DDS readiness나 SROS2 permission의 증거로 표시하지 않는다.
- source checkout, external keystore/TURN credential, 다른 Docker project를
  수정·삭제하지 않는다.
- runtime 중간 설정을 위한 “restart” 버튼을 제공하지 않는다. 전체 재시작은
  각 host prefix에서 `elesim-down` 후 `elesim-up`으로 한다.

## 2. Topology schema v4

```yaml
schema: 4
topology_mode: simulation-only   # full | simulation-only
hosts:
  - id: com1
    local: true
    dds: {address: 100.64.0.10, interface: tailscale0}
    ssh: {mode: openssh, address: 192.168.1.10, port: 22, user: operator}
    units:
      - id: runtime
        roles: [pilot, ui]
```

실제 schema에는 install prefix, fingerprint, unit/backend/security fields가
추가된다. 핵심 불변식은 다음과 같다.

- `full`: 2–4 host, Pilot/Sim/UI/Robot 각 정확히 한 번, Robot native Jetson.
- `simulation-only`: 1–3 host, Pilot/Sim/UI 각 정확히 한 번, Robot/Jetson
  placeholder 없음.
- 한 host는 여러 role 또는 독립 deployment unit을 가질 수 있다.
- DDS address에는 port를 쓰지 않는다. SSH destination은 별도 필드다.
- schema v1–v3은 load 시 v4로 normalize하고 v1은 `full`로 해석한다.
- static peers는 active host의 DDS address에서만 만들며, manager SSH 주소에서
  자동 추론하지 않는다.

GUI는 카드에서 role을 배치하고, Pilot/Sim GPU policy를 독립 지정한다.
`inherit`/`specific`/`cpu`와 고정 UUID 또는 null 정책은 installation state의
host capability와 충돌하지 않도록 비활성화될 수 있다. Viewer는 별도
`--view` opt-in이며, 관리 username에 속한 X11 socket/Xauthority만 검사한다.

## 3. Endpoint check와 preflight

`check`는 per-host read-only management query다. runtime role이 실제로 DDS
heartbeat를 냈다고 주장하지 않는다.

ephemeral two-host preflight는 Jetson이 없을 때 정확히 두 active COM card의
DDS/SSH endpoint와 namespace route를 점검한다.

```text
입력: DDS address/interface + 독립 SSH address/port/user/auth/fingerprint
검사: SSH handshake, namespace interface/address/peer route, saved config shape
출력: readiness evidence와 다음 조치
저장하지 않음: topology, authority, security generation, role deployment
```

HTTP `8080`, SSH forwarding `2222`, TURN port를 DDS endpoint로 입력하지 않는다.
preflight가 성공해도 실제 descriptor/heartbeat, SROS2, RGB-D, WebRTC, NAT는
별도 acceptance gate다.

## 4. Security transaction

managed SROS2의 첫 `provision`은 generation을 만들고 모든 active host를
검증해 atomic activation한다. generation이 이미 active면 반복 provision/deploy는
거부하고 의도적 교체에는 `rotate`를 사용한다.

```text
validate topology/roles
  → generate Authority generation on operator laptop
  → stage common public + host-assigned role bundles
  → verify digest/manifest on every host
  → stop affected roles
  → activate generation atomically
  → restart with --no-build and verify role/heartbeat
  → commit journal
```

실패 시 이미 stage/stop/activate된 host를 이전 generation과 pending marker로
돌린다. interrupted state는 GUI의 `recover` action으로 같은 transaction
boundary를 정리한다. 한 host에 aggregate CA private key나 unrelated enclave를
복사하지 않는다.

`trusted-network`는 새 generation 없이도 명시적 interface/firewall trust를
검증해야 하고, external SROS2는 operator-supplied keystore를 그대로 유지한다.

### RGB-D 권장 배치

`simulation-only`에서 connection manager는 다음 배치를 유효한 일반 topology로
취급한다.

```yaml
topology_mode: simulation-only
hosts:
  - id: compute
    units:
      - id: runtime
        assignments:
          - {role: pilot, endpoint_id: pilot-main}
          - {role: sim, endpoint_id: sim-default}
  - id: operator
    units:
      - id: runtime
        assignments:
          - {role: ui, endpoint_id: ui-main}
```

이 배치는 새 role이나 Router를 만들지 않는다. source인 Sim과 RGB-D edge broker인
Pilot을 같은 Compose unit에 두어 raw handoff를 host 밖으로 내보내지 않고, UI는
Pilot이 소유하는 encoded broker stream만 받는다. `full`에서는 같은 원칙으로
Robot source와 Pilot broker 사이의 local handoff를 우선한다.

## 5. Host lifecycle

`start`는 Compose/systemd management state만 다룬다. full start는 모든 선택
host의 `docker compose --progress plain build`를 먼저 완료한 후 role을
`elesim-up --no-build`로 올린다. BuildKit stdout/stderr는 host label과 함께
GUI job log와 manager-launch terminal에 전달된다.

```text
check → (optional) provision/deploy/rotate → build all hosts → start all hosts
stop  → optional recover/rotate
```

manager는 Docker daemon socket이 아닌 짧은 host helper와 pinned SSH channel을
통해 고정된 EleSim command만 실행한다. remote command timeout과 build timeout은
분리되어 있다. start 실패 시 이번 job에서 시작한 role만 rollback하며, 기존
foreign project나 source를 건드리지 않는다.

재시작을 요청하려면 manager가 아니라 각 host의 prefix에서 다음을 실행한다.

```bash
elesim-down
elesim-up --no-build
```

topology/security 변경과 runtime restart를 한 버튼에서 합치지 않는 이유는
partial host failure와 generation rollback을 구분하기 위해서다.

## 6. DDS readiness

manager readiness는 다음을 서로 다른 상태로 기록한다.

1. Compose/systemd container started
2. namespace interface/address/route structurally valid
3. exact endpoint descriptor + matching boot heartbeat discovered
4. Sim scene/media startup handshake complete
5. authority/session grant 및 WebRTC signaling response

“container running”은 3–5를 의미하지 않는다. `sim-default`가 늦게 나타나면
Sim log의 scene build/descriptor/heartbeat를 기다린다. `__enter__` 같은
container Python/RMW 예외는 stale image 또는 runtime dependency 문제로
diagnose하고, source/host Python을 우회하지 않는다.

## 7. Viewer와 remote user

연결 manager가 `--view`를 전달할 때 topology의 SSH user를 `--viewer-user`로
전달한다. Sim host wrapper는 그 사용자의 X11 socket과 GDM/user Xauthority를
검증하고, 여러 세션이면 `DP-*`/`HDMI-*` 물리 output을 우선한다. 상응하는
display가 없으면 시작 전에 실패한다.

Viewer는 native Genesis window일 뿐 UI WebRTC 화면이 아니다. UI의 observer와
hand-eye stream은 Sim의 media worker에서 별도로 받아야 한다.

## 8. Failure states와 recovery

| 상태/오류 | 해석 | 조치 |
| --- | --- | --- |
| `topology is not saved` | manager가 host/role invariant를 아직 commit하지 않음 | 저장 후 check/preflight부터 재실행 |
| `pending state is inconsistent` | 이전 managed rollout이 중단됨 | GUI `recover`, 같은 prefix의 로그/journal 확인 |
| `descriptor/heartbeat missing` | process는 떴지만 exact boot가 live가 아님 | role log, interface/route, security view 확인 |
| `sim-default not discovered` | Sim scene 또는 DDS path가 아직 준비되지 않음 | Sim scene build와 descriptor/heartbeat를 기다리거나 실패 원인 확인 |
| `__enter__`/RCL error | tools/runtime Python/RMW가 image와 맞지 않음 | `elesim-update`로 image 재생성·build 후 `elesim-up` |
| `remote command failed` | SSH command/remote prefix/permission 문제 | remote host owner, exact prefix, fingerprint를 독립 점검 |
| `viewer display ambiguous` | 같은 user의 여러 X session이 동률 | `--viewer-user`가 실제 display owner인지 확인하고 명시적 `DISPLAY` 선택 |

manager recovery는 무조건 `docker prune`이나 broad deletion을 수행하지 않는다.
ownership manifest와 generation journal이 검증되지 않으면 fail closed한다.

## 9. 운영 명령

```bash
elesim-connections                 # operator laptop에서 GUI
elesim-status                      # 각 host current runtime summary
elesim-logs                        # 각 role host 로그
elesim-net namespace-check ...     # role namespace 구조 점검
elesim-down && elesim-up --no-build # host-owned deliberate restart
```

GUI는 loopback/token-only이며 원격 사용은 SSH local-forward다. `elesim-up`은
이미지 적용·runtime activation, `elesim-connections`는 fleet topology/security
초기화다. 두 명령의 ownership과 failure semantics를 혼용하지 않는다.
