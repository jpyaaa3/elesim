# 설치와 로컬 운영

이 문서는 설치 prefix 하나의 생성·업데이트·실행·제거를 설명한다. 다중
호스트 토폴로지, managed SROS2 generation과 릴리스 산출물은
[`deployment.md`](deployment.md)를 정본으로 삼는다.

## 1. 설치가 보장하는 것

설치 마법사는 호스트 Python, CUDA, ROS, APT 상태를 container mode에서
변경하지 않고 다음을 생성한다.

- `install-state.json`, `install-ownership.json`
- role별 Compose/build context와 immutable runtime configuration
- scoped instance lifecycle용 `elesim-instance`, 모든 설치의 release 갱신용
  `elesim-update`, legacy fixed project용 `elesim-up`, `elesim-down`,
  `elesim-logs`, `elesim-status`, 그리고 `elesim-setup`, `elesim-net`,
  `elesim-connections` wrapper
- 선택한 security/TURN/config state와 bounded log archive
- 선택 시 같은 Compose project에 붙는 persistent `elesim-dev` 개발 attachment

설치 자체는 image build나 runtime start를 하지 않는다. 첫 release build/publish는
`elesim-update`, 등록된 system의 실행은 `elesim-instance <system> up`이 담당한다.

Scoped 설치의 정확한 owner identity는 다음 read-only 명령으로 확인할 수 있다.

```bash
elesim-net identity
```

Native Robot installations also support this command. Their response contains
`install_mode: native`, the ownership UUID, `prefix` and `bin_dir`, without a
Compose project. The connection manager lists that installation with no Docker
release choices. Both the native setup package and the connection manager must
be updated to support this response.

On Jetson, the wizard can select native Robot together with container Pilot/UI.
The selected prefix/bin belongs to the container installation; Robot receives
the sibling `<prefix>-robot` directory and its own `bin`, state and ownership
UUID. The preview shows the Robot path. Only the container bin is registered
in PATH. Each installation has its own update/uninstall lifecycle. Installation
is sequential: if the second installation fails, the first remains installed.
Sim currently requires amd64 and is unavailable on ARM Jetson. Robot-only
connection cards show native installation lookup without Docker release input.

Image build reports list tools, Sim, Pilot, UI and Robot in that order (only
images actually built are listed). Native Robot does not emit a Docker image.
New installation names use one of 71 bundled adjectives; image aliases use one
of 123 animals, sampled using Python `secrets`: `quick-lion`. Collisions add a
number (`quick2`, `lion2`). Reservations remain locked and persist independently
of image cleanup. Existing two-word names remain readable and keep their bindings.
Names are labels, not security identifiers; UUIDs and hashes remain internal.

The SSH fingerprint button requires a username before showing confirmation.
In OpenSSH mode an empty private-key path selects the forwarded SSH agent;
implicit key-file discovery is disabled. Tailscale SSH is explicitly keyless
and ignores neither a missing username nor the required host-key confirmation.

응답에는 enrollment 검증에 필요한 `install_uuid`와 `project`가 들어 있으며,
최근 설치는 화면 표시용 `install_name`도 함께 반환한다. `install_name`은
소유권 증명이 아니며, 구형 `schema_version: 1` 3필드 응답도 계속 읽을 수
있다. 원격 scoped topology에 이 JSON의 `install_uuid`를 deployment unit별로
등록해야 한다(`project`를 함께 기록할 수도 있다). 연결 관리자는 pinned SSH host key로 인증한 뒤 같은
명령을 다시 읽어 비교하며, 등록되지 않은 원격 unit에는 lifecycle/security
작업을 보내지 않는다. 이 enrollment는 설치 UUID를 자동 추정하거나 mutating
작업 중에 생성하지 않는다.

## 2. Bootstrap

권장 경로는 raw `install.sh`를 호출하는 것이다.

```bash
curl -fsSL \
  https://raw.githubusercontent.com/jpyaaa3/elesim/main/installer/install.sh \
  | ELESIM_REF=main bash
```

재현 가능한 설치는 branch 대신 40자리 commit SHA를 raw URL과
`ELESIM_REF` 양쪽에 사용한다.

```bash
commit=0123456789abcdef0123456789abcdef01234567
curl -fsSL \
  "https://raw.githubusercontent.com/jpyaaa3/elesim/${commit}/installer/install.sh" \
  | ELESIM_REF="$commit" bash
```

Bootstrap은 Docker Engine/Compose v2, OS/architecture, Jetson/WSL/WSLg,
display, NVIDIA GPU, invocation directory, SSH agent, Docker context/Engine
ID와 host `tailscale*` hint를 조사한다. Docker Desktop과 native Docker를
구분해 `direct-host` 또는 `tailscale-sidecar`를 선택하고 그 값을 설치
state에 고정한다.

설치 진입점은 `installer/install.sh`, Python bootstrap은 `installer/bootstrap.py`다.
이전 `installer/bootstrap/` 경로를 사용하는 기존 설치의 update wrapper는
새 경로로 한 번 갱신한다. 아래 `prefix`에 기존 설치 경로를 지정한다.

```bash
prefix="$HOME/ws/newsim"
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/installer/install.sh \
  | ELESIM_REF=main ELESIM_SCOPED_UPDATE=1 ELESIM_INVOCATION_DIR="$prefix" \
    bash -s -- --state "$prefix/install-state.json" update
"$prefix/bin/elesim-update"
```

첫 명령은 설치 산출물과 wrapper를 갱신하고, 두 번째는 새 wrapper로 release를
빌드·발행한다. 과거 commit에 고정한 설치는 해당 commit의 기존 경로를 사용한다.

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

신규 container 설치의 runtime namespace는 `elesim-<install name>`으로
고정된다. 한 설치의 여러 system instance가 이 project를 공유하지만, 다른
설치나 legacy 고정 project를 자동으로 인수하지 않는다.

```text
project:     elesim-quiet_otter
images:      elesim/sim:quiet_otter-golden_snail
containers:  install/system/endpoint-scoped service names
optional:    install-scoped Coturn (Sim host), Tailscale (Docker Desktop), dev service
```

동일 host에서도 서로 다른 prefix와 install UUID는 독립 namespace를 갖는다.
기존 `elesim-runtime`을 사용하는 legacy 설치는 별도 prefix/bin과 ownership
증거를 유지하며 신규 설치가 자동 변경하지 않는다. 새 prefix/bin을 기존
설치기는 상위 표준 ownership manifest의 실제 소유 대상과 충돌하는 새
prefix/bin을 mutation 전에 거부한다. prefix/bin 자체는 하위 전체의 독점
경계가 아니다. 예를 들어 기존 prefix가 홈이어도 소유 대상 밖의
`~/ws/newsim`과 그 전용 `bin`은 허용한다. 재귀 삭제되는 managed/log/authority
root 및 소유 파일·디렉터리·wrapper·manifest와의 겹침은 계속 거부한다.
inventory 디렉터리는 업데이트 시 하위를 다시 소유 목록에 넣을 수 있으므로
보호한다. 단순 created 디렉터리는 비었을 때만 삭제되므로 그 사실만으로
하위 독립 설치를 금지하지 않는다. 기존 manifest를 삭제·수정할 필요는 없다.

한 container 설치에는 Pilot/Sim/UI를 모두 준비해 둘 수 있다. 연결 관리자는
설치된 `roles`를 capability inventory로 취급하고, 현재 topology에 선택된
`assigned_roles`만 설정하고 실행한다. 예를 들어 두 host 모두 세 역할을 설치한
상태에서 한 host에는 Pilot만, 다른 host에는 Sim/UI만 배정할 수 있다. 배정을
바꾸기 전에 이전 topology에서 제외될 실행 중 역할은 중지해야 한다.

### 선택적 개발 attachment

개발 attachment는 기존의 완전한 Git checkout을 같은 install-scoped Compose
project에 profile-scoped 영속 privileged `elesim-dev`로 연결한다.
ROS/scientific stack, 모든 role, model tooling과 tests가 들어가지만 설치기는
checkout을 생성·갱신·소유하지 않는다. `pilot`/`sim`/`ui` 컨테이너와 역할은
그대로 분리되며, 개발 셸에는 런타임 DDS/SROS2 identity를 자동 지급하지 않는다.

```bash
elesim-instance <system> up
elesim-dev            # developer profile 시작 후 Compose exec
```

반복해서 `docker compose run --rm` 개발 컨테이너를 만들지 않는다. attachment는
source checkout을 ownership deletion boundary로 삼지 않는다.

## 4. 생성된 prefix와 PATH

설치 prefix의 대표 구조는 다음과 같다. release는 build fingerprint/image ID로
고정되고, 각 instance의 config·security view·writable cache·logs는 해당
system/endpoint 아래에만 기록된다.

```text
<prefix>/
├── install-state.json
├── install-ownership.json
├── maintenance/                  # stdlib-only host lifecycle/release/uninstall modules
├── containers/compose.yaml
├── apps/<role>/                  # config/model/security view
├── releases/<release-key>/       # immutable manifest + runtime data
├── instances/<system>/           # state/config/cache/log/security/secrets
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

설치 후 실제 값은 install/system/endpoint-scoped service를 대상으로
`elesim-instance <system> status`에서 확인한다. Docker container 이름을 전역 고정값
`elesim-sim`으로 가정하지 않는다.

`inherit`는 host 환경의 값에 의존하므로, 서로 다른 host에서 숫자가 같다고
같은 물리 GPU라는 뜻은 아니다. `elesim-instance <system> status`의 device request와 container
내 `torch.cuda.device_count()`를 함께 본다.

## 6. Display와 Sim Viewer

원격 Sim은 기본 headless다. observer와 hand-eye WebRTC track은 유지하지만
native Genesis Viewer는 자동으로 켜지지 않는다. 실제 X11 세션에서 이번
실행만 Viewer를 열려면 명시적으로 `--view`를 사용한다.

```bash
DISPLAY=:0 CUDA_VISIBLE_DEVICES=0 elesim-instance <system> up --view
```

연결 관리자가 SSH로 Sim을 시작할 때는 topology의 SSH 관리 username을
`--viewer-user`로 전달하고, 그 사용자가 소유한 X11 socket과 Xauthority
후보만 검사한다. 여러 세션이면 물리 `DP-*`/`HDMI-*` 출력을 NX/VNC보다
우선하며, 검증된 조합이 없으면 Compose 전에 실패한다. root의 임의 X11
권한이나 다른 사용자의 세션을 사용하지 않는다.

Viewer를 끄면 Sim은 여전히 camera render와 WebRTC media worker를 실행할 수
있다. native 창이 노트북/원격 host 화면으로 전송되는 기능은 없다.

## 7. 일상 수명주기

사용자 명령의 공통 진입점은 설치된 `bin/elesim`이다. PATH 등록 전에는
해당 bin 디렉터리에서 `./elesim`으로 실행한다.

```bash
elesim connections
elesim up <system>
elesim down <system>
elesim logs <system>
elesim info <system>
elesim remove <system>
elesim tailscale login
elesim tailscale status
elesim update
elesim uninstall
```

각 명령은 기존 설치 wrapper로 전달된다. `info`는 instance `status`에 연결된다.
`remove`는 기존 instance 제거 규칙을 적용하며 설치 전체 제거는 `uninstall`이다.
Native Robot과 legacy 설치는 scoped instance가 없으므로 `up/down/logs/info`를
ID 없이 사용하고 `remove`는 지원하지 않는다. Native Robot 설치에는 연결 관리자와
Tailscale sidecar wrapper가 없으므로 해당 명령은 사용할 수 없다.

`elesim update`는 sidecar 설치에서 `elesim-tailscale update`를 먼저 실행하고,
성공하면 `elesim-update`를 실행한다. sidecar 갱신은 네트워크 컨테이너를 재생성할
수 있다. 실패하면 EleSim 업데이트를 진행하지 않는다. sidecar 없는 설치에서는
EleSim만 갱신하며 host Tailscale은 변경하지 않는다. 기존 개별 wrapper도 유지한다.

### Update

Scoped updates hand off to the freshly generated `elesim-release` command after
bootstrap completes, preserving the installation lock. This keeps publication
rules aligned with the new installation artifacts. An updater predating this
handoff may fail with `invalid scoped image for role sim` after refreshing to a
new tag format. In that case, run `./elesim-release` from the installation's
`bin` directory to finish building/publishing with the refreshed command; it
does not fetch source again and reuses the Docker build cache.

```bash
elesim-update
```

설치 state가 기록한 repository/ref를 다시 가져와 ownership manifest를
검증하고, 새 immutable release를 build/publish한다. topology, 등록된 instance의
release pin, security generation, credentials, model cache와 logs는 보존하며
실행 중인 container를 교체하거나 instance를 새 release로 repin하지 않는다.
새 release를 사용하려면 해당 system을 명시적으로 register/replace해야 한다.

`elesim-update`는 source/Dockerfile 결함을 고치는 재빌드 경계이지 자동
restart가 아니다. 성공한 scoped update는 현재 설치의 이전 이미지 중 어느
instance나 container도 참조하지 않는 이미지만 ownership 조건 아래 정리한다.
Legacy update는 기존 dangling-image 정리만 유지한다. `--purge`나 down은 image layer를
지우거나 foreign resource를 prune하지 않는다.

### Build cache

Sim and development images pin Genesis World 1.4.1 and NumPy 1.26.4.
Sim uses `opencv-python==4.11.0.86`, matching Genesis's distribution dependency;
OpenCV 4.12 requires NumPy 2 on Python 3.10. Development retains the same-version
contrib distribution for Pilot and also pins Genesis's OpenCV dependency.
The shared Sim and development builds check imports and the NumPy/Torch bridge
after dependency installation. Genesis's Madrona 0.0.10 dependency installs CUDA
12 NVRTC/nvJitLink packages on Linux amd64 even with CPU compute selected; their
presence does not select GPU execution. Full GPU/rendering acceptance remains
a separate runtime check.

Bootstrap은 `~/.cache/elesim/setup/environments-v2/`에 Python 의존성과
EleSim 패키지 환경을 분리하여 보관한다. Python 버전/플랫폼/실행 경로,
packaging 도구 조건과 setup `requirements.lock`이 같으면 의존성 설치와
pip 검사를 다시 실행하지 않는다. 소스 경로나 커밋만 바뀌어도 같은 의존성을
재사용한다. EleSim 패키지는 protocol/setup 파일 내용별 환경에 설치하며,
완성된 의존성 환경을 `.pth`로 참조한다. 실행 중인 이전 패키지 환경은 수정하지
않는다. 새 패키지 빌드는 검증된 소스 복사본에서 수행한다.

캐시별 잠금으로 동시 생성을 직렬화하고 pip 검증 성공 후에만 완료 표시를
기록한다. 실패/중단된 환경은 재사용하지 않으며 재시도는 새 디렉터리에서
수행한다. 기존 `venv-*` 캐시는 자동 삭제하지 않는다. 이 변경 후 첫 실행은
새 캐시를 준비하며, 이후 코드만 바뀌는 업데이트는 라이브러리를 재설치하지 않는다.

`elesim-update`와 첫 릴리스의 `elesim-release`는 이미지 빌드의 stdout/stderr를
`<prefix>/logs/build/<UTC timestamp>-<random>.log`에 보관한다. 디렉터리는 0700,
파일은 0600이며 symlink 조상 경로를 거부한다. 로그 파일 생성·쓰기 실패는
빌드를 실패 처리하며, 자식 명령의 실패 코드는 그대로 전달한다.
curl bootstrap의 venv 생성·packaging 도구·의존성·EleSim 패키지 설치·검증도
같은 단계별 transcript 형식을 사용한다. 이 준비 로그는
`<ELESIM_CACHE_DIR>/logs/setup/` (기본 `~/.cache/elesim/setup/logs/setup/`)에 남는다.
TTY에서는 현재 출력과 경과 시간을 한 줄로 갱신한다. 성공하면 마지막 3줄,
생략 표시(`...`)와 완료 시간을 남기고, 실패하면 마지막 12줄과 실패 코드를 남긴다.
curl과 새 update/release wrapper는 non-TTY에서도 compact 요약을 기본으로 쓴다.
`ELESIM_VERBOSE=1`은 원문 출력, `ELESIM_BUILD_PROGRESS=plain`은 애니메이션 없는
요약을 선택한다. `ELESIM_BUILD_PROGRESS=auto`는 non-TTY 원문 전달을 선택한다.
구형 update가 설치 파일 갱신 뒤 호출하는 `elesim-compose build`도 새 helper를
거친다. 직접 Compose 호출은 기본 auto이므로 연결관리자/SSH의 non-TTY 원문
스트림을 유지한다. 중첩된 helper는 빌드를 이중 기록하지 않는다.
Ctrl+C/SIGTERM은 빌드 process group에 전달하고 3초 후에도 남으면 종료한다.
빌드 transcript는 runtime snapshot 보존 옵션과 별개로 저장하며 자동 순환
삭제하지 않는다. 설치 제거의 기본 로그 삭제 및 `--keep-logs` 대상이다.
Bootstrap의 GUI URL, sudo·로그인 안내와 설치 폼은 캡처하지 않고 그대로
표출한다. 비대화형 install/update의 설치 단계·완료·다음 명령·PATH 안내는
요약 안에서도 표출한다. GUI 서버 전체를 감싸거나 URL token을 transcript에
수집하지 않는다. GUI의 설치 job 로그는 기존 callback 경로를 유지한다.
bootstrap cache 로그는 설치 prefix 밖에 있으므로 제거 시에도 보존한다.

### 같은 이미지 이름 아래 여러 태그

Scoped 이미지 이름은 `elesim/<role>:<install name>-<release name>`이다.
UUID와 전체 fingerprint는 소유권 및 빌드 메타데이터에 보존한다. 한 번의
scoped update/release가 만드는 각 Pilot·Sim·UI 이미지에는 역할별로 다른 짧은
release name을 붙인다. 기존 UUID project는 업데이트해도 유지하고, 새로
생성하는 이미지 태그에 짧은 이름을 쓴다. 변경된 역할 입력에는 새 release
name을 예약하며, 정확히 같은 입력을 재시도할 때는 기존 name을 재사용한다. 성공한
scoped update/release 및 `elesim-instance <system> up` 뒤에는 미참조 구버전
이미지를 자동 정리한다.
현재 Compose의 최신 이미지, 등록된 instance가 고정한 릴리스 이미지,
실행·정지 container가 참조하는 이미지와 외부 별칭/registry digest는 보존한다.
설치 lock 아래 Engine ID·install UUID·project·fingerprint와 소유 목록을 확인하고
exact image ID만 `docker image rm` (force 없이)으로 제거한다. 미완료 registry나
transaction lease, 소유권 불일치가 있으면 삭제하지 않고 실패를 보고한다.
BuildKit cache·다른 설치·upstream image는 정리하지 않는다.
이는 다른 설치를 만드는 것이 아니라, 기존 instance의 릴리스 pin을 유지하는
버전 보관이다. release name이 바뀌어도 동일한 image ID와 공통 Docker 레이어를
재사용할 수 있다. Repository 이름만 같은 항목을 중복 설치로 판단하거나
일괄 삭제하지 않는다. 실행 중 container 및 등록된 release의 참조를 먼저
확인해야 한다. `<none>` 이미지와 서로 다른 버전 태그는 별도로 구분한다.
Release manifest/data는 이력으로 남지만, 미등록 구버전의 이미지 보관을
보장하지 않는다. 정리된 버전으로 되돌리려면 해당 소스를 다시 빌드/발행해야
한다. 업데이트는 instance를 자동 repin/restart하지 않으며, 구버전에 고정된
system이 있다면 그 이미지는 계속 남는다. 정리 실패는 완료된 build/start를
되돌리지 않으며 재시도로 복구한다.

Dockerfiles use BuildKit cache mounts for pip downloads/wheels; these caches
are not included in the runtime image. Cache misses must still be buildable
from the declared dependencies. Do not purge Docker caches during an ordinary
update. Runtime APT/CasADi/Torch and pinned MPC installation precede app source
copies; tools ABI repair precedes protocol/app copies. Release dependency
installation precedes application wheels and runtime config/data. Developer
UID/GID arguments are declared after dependency installation so a different
developer account does not invalidate those expensive layers.

Sim robotpkg/CasADi shell blocks fail immediately on installation, pinned-commit,
build or plugin-check failure; cleanup must not turn these failures into success.
All release application images run `pip check` after installing their wheels.
The developer image no longer silently attempts `rosdep init`: dependencies
are installed explicitly with APT/pip, and no repository workflow uses rosdep.
Build-command test doubles remain in versioned `workbench/tests`, outside curl
snapshots and application packages.

The first build after this layer reordering can rebuild dependencies. Subsequent
source-only builds should reuse dependency layers. Actual cache-hit/timing
acceptance still requires a live Docker daemon: build twice unchanged, then
change only app source, requirements, or developer UID/GID separately and inspect
BuildKit `CACHED` output. Static ordering tests are not proof of cache hits.

### Scoped instance lifecycle

```bash
elesim-instance <system> up [--no-build]
elesim-instance <system> down
elesim-instance <system> logs
elesim-instance <system> status
elesim-instance <system> remove
```

`elesim-instance`는 등록된 system의 exact service와 해당 release만 대상으로
lifecycle한다. `remove`도 선택한 instance 자원만 정리하고 공용 release/image나
legacy fixed project를 건드리지 않는다. register는 기존 system을 덮어쓰지
않으며, 의도적인 release 교체는 명시적인 replace transaction으로 수행한다.

Scoped 설치의 generic `elesim-up`, `elesim-down`, `elesim-logs`, `elesim-status`
wrapper는 fail-closed로 거부된다. 이 명령들은 legacy fixed `elesim-runtime`
설치에서만 유지된다.

### Down, logs, status

```bash
elesim-instance <system> down
elesim-instance <system> logs
elesim-instance <system> status
elesim-instance <system> remove
```

`elesim-instance <system> down`은 managed Coturn 등 해당 instance가 소유한
서비스만 중지한다. Docker Desktop의 install-scoped Tailscale sidecar는 instance
lifecycle에서 유지된다. 로그
archive 실패가 있어도 shutdown은 시도하며 최종 exit status에는 archive 실패를
반영한다.

연결 관리자의 “restart”는 런타임 설정을 원자적으로 재적용하는 동작이 아니므로
사용하지 않는다. scoped 전체 재시작은 각 호스트의 정확한 prefix에서 해당
system을 `elesim-instance <system> down` 후 `up`으로 수행하고, multi-host
재구성은 manager의 stop/start 또는
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

설치 마법사와 연결 관리자는 같은 `app/` 프로젝트와 wheel에 함께 패키징되지만
서로 다른 Python 패키지다. 설치·호스트 lifecycle과 설치 자산은
`elesim_setup/`이 소유하고, 다중 호스트 토폴로지 편집·SROS2 authority/policy와
연결 관리자 웹 자산은 `elesim_connections/`가 소유한다. 사용자 진입점은 기존과
같이 `elesim-connections`로 유지되며, 두 패키지 중 어느 쪽도 다른 runtime
deployment 구현을 import하지 않는다.

`elesim-connections`는 인자 없이 편집 화면을 연다. 시스템 ID는 화면에서
정하며 저장하면 `connections/<system-id>/topology.json`에 기록된다.
기존 시스템을 바로 열려면 `elesim-connections --system <id>`를 사용한다.
명시적으로 선택한 workspace는 다른 시스템 ID로 저장할 수 없다.
화면을 열거나 저장하는 것만으로 runtime을 시작하지 않는다.

`trusted-network`는 소유 LAN/routed VPN에서만 허용하고, 공유망은 `sros2`
enforce를 사용한다. managed mode의 Authority private key는 operator laptop에
남고 각 host에는 common public material과 배정 role enclave만 전달된다.
`elesim-connections`가 generation을 provision/rotate/deploy하며 partial failure
시 rollback한다. 외부 keystore는 EleSim이 소유하지 않는다.

관리자 GUI는 loopback/token-only이며 Docker socket과 tailscaled local API를
받지 않는다. host helper는 allowlisted EleSim command와 선택적 Tailscale SSH
stream만 수행한다. DDS endpoint/interface/address와 SSH management
destination/port/user/fingerprint는 별도 필드다.

자세한 role-derived topology, preflight, host lifecycle, security journal은
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
| `DDS readiness` 실패 | `elesim-instance <system> status`, `elesim-net namespace-check`, role log | manager는 최대 5분 동안 exact descriptor/boot heartbeat만 기다린다. interface/address/route와 실제 peer heartbeat를 분리해서 확인한다. Sim scene/media session은 별도이며 UI가 재시도한다. SSH/HTTP 성공만으로 해결되지 않는다. |
| `__enter__` 또는 `rclpy` 예외 | `elesim-update` 후 새 immutable release인지, container Python/RMW 버전 | host Python을 고치지 말고 release를 갱신한 뒤 해당 system을 명시적으로 replace하고 `elesim-instance <system> up`한다. |
| Sim endpoint 미발견 | Sim scene/media startup, Pilot/UI descriptor/heartbeat, security bundle | Sim container가 running이어도 session grant 전일 수 있다. 등록된 graph role ID(예: `sim-1`)와 scene handshake, exact boot를 확인한다. |
| Viewer가 다른 사용자 화면에 뜸 | `--viewer-user`, 해당 사용자의 X socket/Xauthority, `DISPLAY` | 연결 topology의 SSH username과 실제 display owner를 일치시킨다. 다른 사용자의 X를 허용하지 않는다. |
| `simulation session is not connected` | UI/Sim boot, session grant/renewal, WebRTC signaling log | DDS session과 WebRTC media를 별도로 진단한다. Coturn은 DDS를 고치지 않는다. |
| observer가 깨짐/렉 | `elesim-instance <system> status`의 encoder/backend/streams와 Sim perf fields | NVENC/libx264 fallback, scene render, camera conversion, MPC solve를 각각 측정한다. QoS를 무작정 낮추지 않는다. |
| Robot 설치에서 `/opt/ros/humble/setup.bash` 없음 | Jetson host의 ROS 2 Humble, `colcon`, `~/ros2_ws/install/setup.bash` | 해당 prerequisites가 있는 Jetson은 bootstrap이 host venv 경로를 선택한다. 파일이 없으면 ROS 2/Unitree workspace를 먼저 준비하고, 컨테이너 로그에서 이 오류가 나면 bootstrap source를 갱신한다. |
| `canonicalize_version(... strip_trailing_zero ...)`로 `elesim_interfaces` 빌드 실패 | host Python의 `setuptools`/`packaging` 혼합 | bootstrap이 캐시 venv에 호환되는 metadata 패키지를 설치하고 ROSIDL 빌드에만 우선 사용한다. host 전역 `pip`를 업그레이드하지 않는다. |
| `managed SROS2 pending` | manager에서 generation `provision`/`rotate`/`recover` | generation transaction을 끝내기 전 role을 임의로 up하지 않는다. |
| `elesim-update` 후 옛 동작 | update는 container나 instance pin을 교체하지 않음 | 새 release를 명시적으로 register/replace한 뒤 `elesim-instance <system> up`한다. |
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
