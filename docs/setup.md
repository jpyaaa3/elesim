# 설치와 로컬 운영

이 문서는 설치 prefix 하나의 생성·업데이트·실행·제거를 설명한다. 다중
호스트 토폴로지, managed SROS2 generation과 릴리스 산출물은
[`deployment.md`](deployment.md)를 정본으로 삼는다.

## 1. 설치가 보장하는 것

설치 마법사는 호스트 Python, CUDA, ROS, APT 상태를 container mode에서
변경하지 않고 다음을 생성한다.

- `install-state.json`, `install-ownership.json`
- role별 Compose/build context와 immutable runtime configuration
- `elesim-up`, `elesim-down`, `elesim-update`, `elesim-logs`, `elesim-status`
  및 `elesim-setup`, `elesim-net`, `elesim-connections` wrapper
- 선택한 security/TURN/config state와 bounded log archive
- 선택 시 같은 Compose project에 붙는 persistent `elesim-dev` 개발 attachment

설치 자체는 image build나 runtime start를 하지 않는다. 첫 build와 실행은
설치 후의 `elesim-up` 또는 `elesim-update`가 담당한다.

## 2. Bootstrap

권장 경로는 raw `install.sh`를 호출하는 것이다.

```bash
curl -fsSL \
  https://raw.githubusercontent.com/jpyaaa3/elesim/refactoring/installer/bootstrap/install.sh \
  | ELESIM_REF=refactoring bash
```

재현 가능한 설치는 branch 대신 40자리 commit SHA를 raw URL과
`ELESIM_REF` 양쪽에 사용한다.

```bash
commit=0123456789abcdef0123456789abcdef01234567
curl -fsSL \
  "https://raw.githubusercontent.com/jpyaaa3/elesim/${commit}/installer/bootstrap/install.sh" \
  | ELESIM_REF="$commit" bash
```

Bootstrap은 Docker Engine/Compose v2, OS/architecture, Jetson/WSL/WSLg,
display, NVIDIA GPU, invocation directory, SSH agent, Docker context/Engine
ID와 host `tailscale*` hint를 조사한다. Docker Desktop과 native Docker를
구분해 `direct-host` 또는 `tailscale-sidecar`를 선택하고 그 값을 설치
state에 고정한다.

GUI는 host loopback에만 열리고 URL token으로 보호된다. 기본 포트는 `8765`이며
점유 중이면 제한된 범위에서 다음 포트를 찾는다. 원격 접근은 GUI port를
SSH local-forward한다.

```bash
ssh -L 8765:127.0.0.1:8765 -p <ssh-port> <user>@<server>
```

이 SSH 포트는 설치 GUI 접근용일 뿐 DDS, RGB-D, WebRTC 또는 TURN endpoint가
아니다. GUI를 public interface에 bind하거나 token을 로그/스크립트에 남기지
않는다. `ELESIM_NO_OPEN=1`은 browser auto-open을 끄고,
`ELESIM_GUI_PORT=<port>`는 첫 후보를 고정한다.

`ELESIM_REPOSITORY`, `ELESIM_REF`, `ELESIM_ARCHIVE_URL`, `ELESIM_CACHE_DIR`와
`--refresh`는 source retrieval만 제어한다. archive extractor는 absolute path,
parent traversal, link, device entry를 거부하며 stale cache를 자동 실행하지
않는다.

## 3. Runtime 설치와 개발 attachment

### Runtime 설치

설치기는 선택한 `pilot`, `sim`, `ui`를 Docker role image로 만든다. Robot은
감지된 Jetson에서만 native-only로 선택 가능하며, `elesim-robot.service`와
`elesim-unitree-bridge.service` 두 systemd unit을 생성한다. Generic amd64
container backend는 Robot을 받지 않는다.

Jetson Robot 설치는 host의 ROS 2 Humble과 `colcon`을 사용해
`payload/runtime/common/elesim_interfaces` overlay를 빌드해야 한다. Bootstrap은
`/opt/ros/humble/setup.bash`와 `colcon`이 감지된 Jetson에서만 EleSim 전용
host venv로 setup을 실행한다. 이 venv는 `~/.cache/elesim/setup` 아래에
생성되며 host Python 패키지나 ROS/Apt 상태를 수정하지 않는다. ROS 2가 없는
Jetson은 Robot 설치 전에 host ROS 2/Unitree workspace를 준비해야 한다.

고정된 runtime 이름은 다음과 같다.

```text
project:     elesim-runtime
images:      elesim/pilot:local  elesim/sim:local  elesim/ui:local
containers:  elesim-pilot       elesim-sim       elesim-ui
optional:    elesim-coturn (Sim host), elesim-tailscale (Docker Desktop)
```

한 host의 같은 prefix에는 EleSim installation을 두 개 만들지 않는다.
필요하면 서로 다른 prefix와 독립 deployment unit을 사용한다.

### 선택적 개발 attachment

개발 attachment는 기존의 완전한 Git checkout을 같은 `elesim-runtime`
project에 profile-scoped 영속 privileged `elesim-dev`로 연결한다.
ROS/scientific stack, 모든 role, model tooling과 tests가 들어가지만 설치기는
checkout을 생성·갱신·소유하지 않는다. `pilot`/`sim`/`ui` 컨테이너와 역할은
그대로 분리되며, 개발 셸에는 런타임 DDS/SROS2 identity를 자동 지급하지 않는다.

```bash
elesim-up             # 선택한 runtime 역할 시작
elesim-dev            # 필요할 때 developer profile 시작 후 Compose exec
```

반복해서 `docker compose run --rm` 개발 컨테이너를 만들지 않는다. attachment는
source checkout을 ownership deletion boundary로 삼지 않는다.

## 4. 생성된 prefix와 PATH

설치 prefix의 대표 구조는 다음과 같다.

```text
<prefix>/
├── install-state.json
├── install-ownership.json
├── maintenance/                  # stdlib-only uninstaller
├── containers/compose.yaml
├── apps/<role>/                  # config/model/security view
├── security/                      # managed generation/current
├── connections/                   # non-secret topology
├── secrets/                       # owned TURN/Tailscale state only
└── logs/runs/                     # optional bounded snapshots
```

설치기는 `<prefix>/bin`에 idempotent PATH block을 `.bashrc`에 추가한다. 현재
shell은 설치 직후 한 번만 다음을 실행한다.

```bash
source ~/.bashrc
```

Uninstaller는 설치 manifest로 확인된 이 설치의 prefix/bin/runtime 산출물,
managed security view, logs, operator Authority와 EleSim 로컬 image를 제거하는
host-local factory reset 경계다. BuildKit/download cache는 재사용 가능한
cache이므로 삭제하지 않는다. 다른 프로젝트의 home, source checkout,
Tailscale admin record, 외부 keystore/TURN credential은 EleSim이 생성했다는
소유 증거가 없으므로 보존한다. 외부 보안 상태의 폐기와 재발급은
`elesim-connections`의 managed generation transaction에서 수행한다.

## 5. GPU 정책

설치 GUI의 role별 GPU policy는 다음 세 가지다.

| policy | Compose access | runtime 의미 |
| --- | --- | --- |
| `inherit` | 선택 daemon이 노출한 GPU | host/scheduler가 정한 `CUDA_VISIBLE_DEVICES`를 전달 |
| `specific` | `device_ids` 한 개 reservation | 한 index/UUID만 노출하며 container 안에서 host index를 다시 적용하지 않음 |
| `cpu` | GPU reservation 없음 | Sim Genesis backend와 encoder를 CPU로 고정 |

Pilot과 Sim은 독립적으로 policy를 가질 수 있다. GUI에서 GPU가 고정되었거나
CPU-only로 감지된 role은 상속 checkbox를 미리 선택/해제한 상태로 보여주며
사용자가 설치 뒤 임의로 바꾸지 못한다. `specific`은 UUID/index를 예약할
뿐이며, `CUDA_VISIBLE_DEVICES`만 바꿔서 container runtime이 차단한 장치를
되살릴 수 없다.

설치 후 실제 값을 확인한다.

```bash
docker exec elesim-sim sh -lc \
  'printf "CVD=%s\\n" "${CUDA_VISIBLE_DEVICES:-unset}"; \
   command -v nvidia-smi >/dev/null && nvidia-smi --query-gpu=index,uuid,name --format=csv,noheader || true'
nvidia-smi
```

`inherit`는 host 환경의 값에 의존하므로, 서로 다른 host에서 숫자가 같다고
같은 물리 GPU라는 뜻은 아니다. `elesim-status`의 device request와 container
내 `torch.cuda.device_count()`를 함께 본다.

## 6. Display와 Sim Viewer

원격 Sim은 기본 headless다. observer와 hand-eye WebRTC track은 유지하지만
native Genesis Viewer는 자동으로 켜지지 않는다. 실제 X11 세션에서 이번
실행만 Viewer를 열려면 명시적으로 `--view`를 사용한다.

```bash
DISPLAY=:0 CUDA_VISIBLE_DEVICES=0 elesim-up --view
```

연결 관리자가 SSH로 Sim을 시작할 때는 topology의 SSH 관리 username을
`--viewer-user`로 전달하고, 그 사용자가 소유한 X11 socket과 Xauthority
후보만 검사한다. 여러 세션이면 물리 `DP-*`/`HDMI-*` 출력을 NX/VNC보다
우선하며, 검증된 조합이 없으면 Compose 전에 실패한다. root의 임의 X11
권한이나 다른 사용자의 세션을 사용하지 않는다.

Viewer를 끄면 Sim은 여전히 camera render와 WebRTC media worker를 실행할 수
있다. native 창이 노트북/원격 host 화면으로 전송되는 기능은 없다.

## 7. 일상 수명주기

### Update

```bash
elesim-update
```

설치 state가 기록한 repository/ref를 다시 가져와 ownership manifest를
검증하고, owned artifact를 재생성한 뒤 선택 image를 증분 build한다. topology,
security generation, credentials, model cache와 logs는 보존하며 실행 중인
container는 교체하지 않는다. 업데이트 후 새 image를 적용하려면 별도의
`elesim-up`이 필요하다.

`elesim-update`는 source/Dockerfile 결함을 고치는 재빌드 경계이지 자동
restart가 아니다. 성공한 update는 현재 설치의 fingerprint가 붙은 이전
dangling image만 ownership 조건 아래 정리한다. `--purge`나 down은 image layer를
지우거나 foreign resource를 prune하지 않는다.

### Up

```bash
elesim-up                 # fingerprint가 다를 때만 build
elesim-up pilot           # Pilot만 시작
elesim-up sim             # Sim만 시작
elesim-up ui              # UI만 시작
elesim-up --no-build      # 이미 준비된 image로 적용
elesim-up --view          # 명시적 Sim native Viewer
```

`elesim-up`은 선택한 role image의 generated build fingerprint를 local image
label과 비교한다. 이미지가 없거나 fingerprint가 다를 때만 build하고, 일치하면
`--no-build`로 Compose를 시작한다. tools image도 같은 검사를 거쳐 readiness
검사에서 변경 없이 다시 build하지 않는다. multi-host 전체 build/launch와
security rollout은 `elesim-connections`가 host별로 조정한다.
동일한 active generation과 역할을 임의로 덮어쓰지 않는다.

General 설치는 role별 host wrapper를 만들지 않는다. `elesim-pilot`,
`elesim-sim`, `elesim-ui`는 Docker 내부의 고정 container/entrypoint 이름이고,
호스트에서는 항상 `elesim-up <role>`을 사용한다.

### Down, logs, status

```bash
elesim-down                 # runtime role/Coturn 중지·로그 snapshot
elesim-down --purge         # exact owned runtime resource도 정리
elesim-logs                 # 현재 role 로그 follow (Ctrl+C는 follow만 중지)
elesim-logs --save          # bounded private snapshot 저장
elesim-status               # host/IP/container/GPU/DDS/media 요약
```

`elesim-down`은 managed Coturn 등 해당 Compose 프로젝트가 소유한 서비스도
설치가 소유하면 함께 중지한다. Docker Desktop의 `elesim-tailscale` sidecar는
일반 `elesim-down`에서 유지되며 `elesim-down --purge`에서만 내려간다. 로그
archive 실패가 있어도 shutdown은 시도하며 최종 exit status에는 archive 실패를
반영한다.

연결 관리자의 “restart”는 런타임 설정을 원자적으로 재적용하는 동작이 아니므로
사용하지 않는다. 전체 재시작은 각 호스트의 정확한 prefix에서 `elesim-down`
후 `elesim-up`으로 수행하고, multi-host 재구성은 manager의 stop/start 또는
security transaction으로 처리한다.

## 8. Tailscale sidecar와 네트워크 점검

Docker Desktop은 WSL distribution의 host `tailscale0`를 container namespace에
상속하지 않는다. sidecar 설치에서는 다음 명령으로 한 번 enrollment하고
상태를 확인한다.

```bash
elesim-tailscale login
elesim-tailscale status
elesim-tailscale update
elesim-net namespace-check --dds-interface tailscale0
```

`login`은 browser/device flow이고 stale Running node를 재인증할 수 있다.
EleSim은 auth/OAuth key를 저장하지 않고 sidecar node state만 prefix의
mode-0700 secrets 아래 보관한다. `status`의 sidecar DDS IP와 host/WSL SSH
주소는 서로 다른 값일 수 있다.

`update`는 설치된 Compose의 고정 Tailscale image를 pull한 뒤 sidecar를
재생성한다. sidecar namespace를 공유하는 role과 managed Coturn 중 당시
실행 중이던 서비스만 잠시 중지하고 같은 목록을 다시 연결하므로, 로그인
상태 volume과 중지되어 있던 서비스는 건드리지 않는다. 새 Tailscale
버전/다이제스트로 이동하려면 먼저 `elesim-update`로 설치 산출물을 갱신한
다음 `elesim-tailscale update`를 실행한다. 새 sidecar가 준비되지 않으면
명령은 실패하고 역할을 자동으로 시작하지 않는다.

`namespace-check`는 role과 같은 namespace에서 interface 존재, advertised
address 할당, static peer route를 읽기 전용으로 검사한다. SSH 연결 성공은
DDS UDP discovery 증거가 아니며, SSH/Tailscale nc는 DDS traffic을 relay하지
않는다.

## 9. 보안과 연결 관리자 경계

`trusted-network`는 소유 LAN/routed VPN에서만 허용하고, 공유망은 `sros2`
enforce를 사용한다. managed mode의 Authority private key는 operator laptop에
남고 각 host에는 common public material과 배정 role enclave만 전달된다.
`elesim-connections`가 generation을 provision/rotate/deploy하며 partial failure
시 rollback한다. 외부 keystore는 EleSim이 소유하지 않는다.

관리자 GUI는 loopback/token-only이며 Docker socket과 tailscaled local API를
받지 않는다. host helper는 allowlisted EleSim command와 선택적 Tailscale SSH
stream만 수행한다. DDS endpoint/interface/address와 SSH management
destination/port/user/fingerprint는 별도 필드다.

자세한 topology mode, preflight, host lifecycle, security journal은
[`deployment.md`](deployment.md)를 참조한다.

## 10. 제거

```bash
elesim-uninstall
```

uninstaller는 install UUID, wrapper/systemd hash, Docker label/metadata와
managed sidecar ownership을 검증한 뒤에만 즉시 mutation한다. 기본적으로 owned
runtime/log/operator Authority를 제거하고, `--keep-logs` 또는
`--keep-authority`로 보존할 수 있다. 외부 source, credentials, keystore,
Docker upstream image, Tailscale control-plane node는 삭제하지 않는다.

legacy generated path가 manifest 없이 남아 있으면 자동 adopt하지 않고
실패한다. `docker system prune`, broad `rm -rf`, home 전체 삭제로 설치를
정리하지 않는다.

## 11. 문제 해결표

| 증상 | 먼저 확인할 것 | 의미/조치 |
| --- | --- | --- |
| `DDS readiness` 실패 | `elesim-status`, `elesim-net namespace-check`, role log | manager는 최대 5분 동안 exact descriptor/boot heartbeat만 기다린다. interface/address/route와 실제 peer heartbeat를 분리해서 확인한다. Sim scene/media session은 별도이며 UI가 재시도한다. SSH/HTTP 성공만으로 해결되지 않는다. |
| `__enter__` 또는 `rclpy` 예외 | `elesim-update` 후 새 tools/runtime image인지, container Python/RMW 버전 | host Python을 고치지 말고 generated image를 재생성·재빌드한 뒤 `elesim-up`한다. |
| `sim-default` 미발견 | Sim scene/media startup, Pilot/UI descriptor/heartbeat, security bundle | Sim container가 running이어도 session grant 전일 수 있다. scene handshake와 exact boot를 기다린다. |
| Viewer가 다른 사용자 화면에 뜸 | `--viewer-user`, 해당 사용자의 X socket/Xauthority, `DISPLAY` | 연결 topology의 SSH username과 실제 display owner를 일치시킨다. 다른 사용자의 X를 허용하지 않는다. |
| `simulation session is not connected` | UI/Sim boot, session grant/renewal, WebRTC signaling log | DDS session과 WebRTC media를 별도로 진단한다. Coturn은 DDS를 고치지 않는다. |
| observer가 깨짐/렉 | `elesim-status`의 encoder/backend/streams와 Sim perf fields | NVENC/libx264 fallback, scene render, camera conversion, MPC solve를 각각 측정한다. QoS를 무작정 낮추지 않는다. |
| Robot 설치에서 `/opt/ros/humble/setup.bash` 없음 | Jetson host의 ROS 2 Humble, `colcon`, `~/ros2_ws/install/setup.bash` | 해당 prerequisites가 있는 Jetson은 bootstrap이 host venv 경로를 선택한다. 파일이 없으면 ROS 2/Unitree workspace를 먼저 준비하고, 컨테이너 로그에서 이 오류가 나면 bootstrap source를 갱신한다. |
| `canonicalize_version(... strip_trailing_zero ...)`로 `elesim_interfaces` 빌드 실패 | host Python의 `setuptools`/`packaging` 혼합 | bootstrap이 캐시 venv에 호환되는 metadata 패키지를 설치하고 ROSIDL 빌드에만 우선 사용한다. host 전역 `pip`를 업그레이드하지 않는다. |
| `managed SROS2 pending` | manager에서 generation `provision`/`rotate`/`recover` | generation transaction을 끝내기 전 role을 임의로 up하지 않는다. |
| `elesim-update` 후 옛 동작 | update는 container를 교체하지 않음 | 정확한 prefix에서 `elesim-down` 후 `elesim-up`한다. |
| `No module named pip` bootstrap | host venv/cache를 직접 고치지 않음 | `install.sh`를 새 source ref로 다시 실행해 setup cache snapshot을 재생성한다. |

curl bootstrap은 runtime에 필요하지 않은 `payload/runtime/docker/sim/app/elesim_sim/rl` 연구/학습
스택을 source snapshot에서 제외한다. 이 디렉터리는 저장소에는 남아 있으므로
연구 코드를 별도로 실행할 때는 checkout을 사용한다.

## 12. 자동 검증과 수동 gate

Canonical 테스트는 setup-generated `elesim-dev`에서 실행한다.

```bash
elesim-dev python3 workbench/tools/quality/check.py --group required
elesim-dev python3 workbench/tools/quality/check.py --group extended
elesim-dev python3 workbench/tools/release/build.py
elesim-dev python3 workbench/tools/release/verify.py dist/releases
```

이는 실제 두 host의 NAT, SROS2 enforce, GPU/X11, TURN relay, Genesis viewer,
Jetson safety를 증명하지 않는다. 그 항목은 [`status.md`](status.md)의
수동 gate로 남긴다.
