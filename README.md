# Elesim 사용자 설명서

Elesim은 한 프로그램이 아니라 독립 배포 가능한 네 프로그램으로 구성된다.

| 프로그램 | 책임 |
| --- | --- |
| Controller | Vision, IK, Look/Aim/Grasp, Gaze, 목표값 계산 |
| UI | 사용자 입력, 상태 표시, observer/hand-eye 영상, Simulator 조작 |
| Simulator | Genesis, 가상 telemetry, observer/hand-eye 렌더링 |
| Robot | 실제 모터·카메라 I/O, feedback, deadman, 로컬 안전 제한 |

프로그램끼리는 ROS 2/DDS로 직접 통신한다. 현재 discovery와 RGBD는 typed ROS
message이고, 제어·WebRTC signaling은 bounded protocol-v5 DDS message를
사용한다. observer/hand-eye 영상만 WebRTC를 사용한다. 중앙 Router와
ZMQ/CURVE transport는 없다. 서로 다른 컴퓨터에 설치해도 되지만 DDS
participant 사이에 양방향 UDP 경로가 있어야 한다.

## 빠른 설치

Ubuntu 또는 WSL 터미널에서 설치할 디렉터리로 이동한 뒤 실행한다.

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/misc/setup/bootstrap.sh | bash
```

`main`이 아닌 브랜치를 시험할 때는 URL과 `ELESIM_REF`를 같은 브랜치로 맞춘다.

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/refactoring/misc/setup/bootstrap.sh \
  | ELESIM_REF=refactoring bash
```

브랜치와 태그처럼 이동할 수 있는 ref는 실행할 때마다 조건부로 최신성을 확인한다.
서버가 HTTP `304 Not Modified`를 반환하면 검증된 snapshot을 재사용하고, 변경됐으면
새 revision을 별도로 내려받는다. 반면 full 40자리 commit SHA는 immutable
snapshot으로 재사용한다. 터미널에는 실제 revision 또는 archive digest와 함께
새 다운로드, HTTP `304` 검증, immutable cache 재사용 중 해당 상태가 표시된다.

이전 설치기의 URL 기반 cache가 남아 있어도 새 cache는 이를 자동으로 우회한다.
네트워크, archive 검증, bootstrap 세대 일치 확인이 실패하면 이전 snapshot은
보존하지만 오래된 설치기를 대신 실행하지 않고 중단한다. 조건부 cache를 무시하고
전체 archive를 다시 확인하려면 다음처럼 `--refresh`를 `bash` 인자로 전달한다.

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/refactoring/misc/setup/bootstrap.sh \
  | ELESIM_REF=refactoring bash -s -- --refresh
```

이 명령은 다음 순서로 동작한다.

1. Docker와 Compose v2 사용 가능 여부를 확인한다.
2. 일회성 `python:3.10-slim` 컨테이너에서 설치 GUI를 실행한다.
3. 브라우저로 `http://127.0.0.1:8765`를 연다. 사용 중인 포트면 다음 빈
   포트를 자동 선택하고 정확한 URL을 터미널에 출력한다.
4. 선택한 설치 파일과 이미지 build context만 생성한다.

설치 중에는 호스트 Python, CUDA, ROS, APT를 변경하지 않는다. Docker가 없는
Ubuntu에서만 설치 여부를 터미널로 한 번 묻고, 사용자가 승인한 경우에만 Docker
패키지를 설치한다. 설치 GUI 자체는 Docker socket을 받지 않으므로 이미지를
빌드하거나 서비스를 시작하지 않는다.

브라우저가 자동으로 열리지 않으면 터미널에 출력된 token 포함 URL을 직접 연다.
GUI는 호스트 loopback에만 공개된다.

### 원격 컴퓨터에서 GUI 열기

서버에 SSH로 접속한 터미널에서 설치기를 실행한다.

```bash
# [서버]
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/misc/setup/bootstrap.sh \
  | ELESIM_NO_OPEN=1 bash
```

출력된 GUI 포트가 `8765`라면 노트북에서 SSH tunnel을 연다. `2222`는 예시이며
실제 SSH 포트를 사용한다.

```bash
# [노트북]
ssh -L 8765:127.0.0.1:8765 -p 2222 USER@SERVER
```

그 다음 노트북 브라우저에서 서버 터미널에 출력된 token 포함 URL을 연다. 설치
GUI 포트는 외부 방화벽에 공개할 필요가 없다.

## 설치 종류

### 일반 사용자용

네 프로그램 중 이 컴퓨터에 필요한 역할을 체크한다.

| 기본 선택 | 역할 |
| --- | --- |
| 한 PC 시뮬레이션 | Simulator, Controller, UI |
| 조작 노트북 | Controller, UI |
| 시뮬레이션 서버 | Simulator |
| 사용자 지정 | 필요한 역할 직접 선택 |
| Robot Jetson | Robot 단독 |

Simulator, Controller, UI는 역할별 Docker 이미지와 하나의 Compose project로
구성된다. Robot은 Jetson/JetPack이 감지된 호스트에서만 선택할 수
있으며 현재 native 단독 설치만 지원한다.

### 개발자용

전체 저장소와 하나의 privileged 개발 컨테이너를 만든다. 이 이미지는 ROS2,
Genesis, Torch, Pinocchio, RealSense, Dynamixel, WebRTC, OpenTelemetry,
모델 builder와 테스트 도구를 포함한다.

- 설치 위치가 비어 있으면 선택한 GitHub ref를 clone한다.
- 완전한 Elesim Git checkout이면 pull/reset 없이 그대로 사용한다.
- 관계없는 파일이 있는 디렉터리는 덮어쓰지 않고 거부한다.
- Jaeger는 선택 사항이며 별도 profile로 생성된다.
- Ubuntu/WSL `amd64`에서만 지원한다.
- `/dev`, host network, host IPC와 GUI socket을 사용하는 privileged
  컨테이너임을 확인해야 설치할 수 있다.

## GUI 입력 항목

### 설치 경로

기본 설치 위치는 `curl` 명령을 실행한 현재 디렉터리이고, 기본 명령 위치는 그
아래 `bin/`이다. 서버에서 동작하는 `찾아보기` 버튼으로 변경할 수 있다.

`PATH에 등록`을 선택하면 설치기는 `~/.bashrc`의 Elesim 관리 블록만 원자적으로
추가하거나 갱신한다. 최초 변경 시 `~/.bashrc.elesim.bak`도 남긴다. 부모
터미널의 환경은 바꿀 수 없으므로 설치 후 한 번 실행한다.

```bash
source ~/.bashrc
```

### GPU 정책

- `inherit`: 실행 시점의 `CUDA_VISIBLE_DEVICES`와 scheduler 할당을 따른다.
- `specific`: `nvidia-smi -L`에 나온 index 또는 UUID 하나만 컨테이너에
  노출한다. 컨테이너 안에서는 보통 논리 `cuda:0`으로 보인다.
- `cpu`: 컨테이너 GPU 요청을 제거하고 Genesis GPU backend도 끈다.

공용 연구 서버에서는 `inherit`가 기본이다. 한 번만 GPU 0을 사용하려면:

```bash
CUDA_VISIBLE_DEVICES=0 elesim-up
```

### DDS 네트워크와 보안

- `system ID`: 한 Elesim graph의 ROS namespace이다. 참여할 모든 호스트에서
  같아야 한다.
- `ROS domain ID`: 같은 DDS domain의 번호이다. 참여할 모든 호스트에서 같아야
  하지만 보안 수단은 아니다.
- `DDS interface`: DDS가 사용할 로컬 interface 이름(예: `eth0`, `wg0`)이다.
  다른 peer가 실제로 도달할 수 있는 LAN 또는 VPN interface를 선택한다.
- `discovery`: 같은 L2에서는 multicast를, multicast가 전달되지 않는 routed
  network에서는 reachable static peer 주소를 사용한다. static peer는 relay가
  아니며 양방향 UDP 경로를 만들지 않는다.

보안 profile은 둘 중 하나다.

- `trusted-network`: DDS 암호화 없음. 소유한 LAN 또는 routed VPN에서만 쓰고,
  선택한 interface와 방화벽으로 참여 가능 호스트를 제한한다.
- `sros2`: 공유 compute나 신뢰하지 않는 network에서 사용한다. 역할마다 별도
  keystore enclave를 배치하고 DDS Security 인증·권한·암호화를 enforce한다.

`ROS_DOMAIN_ID`는 우연한 graph 충돌을 줄일 뿐 인증, 접근 통제, 암호화 또는
tenant 격리를 제공하지 않는다. 최종 구조에는 ZMQ, CurveZMQ, CURVE key와 ZAP
allowlist가 없다.

설치 GUI는 계속 loopback에만 열린다. 원격 GUI를 위한 SSH `-p 2222` 같은 값은
SSH server의 포트일 뿐 DDS 설정에 들어가지 않는다.

### TURN/Coturn

- `미사용`: 같은 LAN 또는 직접 ICE가 가능한 환경.
- `이 Simulator와 Coturn 실행`: Simulator 호스트의 생성 Compose에 Coturn을
  넣는다.
- `기존 relay 사용`: 별도로 운영 중인 TURN URL을 사용한다.

Managed Coturn은 public hostname/IP, realm과 credential 정책이 필요하다.
REST HMAC을 쓰면 static secret은 Coturn과 같은 호스트의 Simulator만 갖고,
Simulator가 활성 session에 묶인 단기 ICE credential을 UI에 발급한다. UI에는
static secret을 전달하지 않는다. 선택하면 `elesim-up`,
`elesim-down`, `elesim-logs`가 Coturn까지 함께 관리한다. 필요한 방화벽 경로는
TCP/UDP `3478`과 UDP `49160-49200`이다. TURN은 WebRTC media relay이며 DDS
topic이나 signaling을 연결해 주지 않는다. Managed TURN credential과 signaling은
DDS로 전달되므로 managed mode는 `sros2` profile을 요구한다.

외부 TURN을 Simulator 호스트에 설치할 때에는 relay가 발급한 자격증명 JSON도
선택한다. 파일 형식은
`{"username":"...","credential":"...","expires_at":4102444800}`이며
`expires_at`은 장기 credential이면 생략할 수 있다. 이 파일은 Simulator
컨테이너에만 read-only로 mount되고, Controller/UI 전용 노트북에는 복사되지
않는다. Simulator가 활성 UI session에 필요한 값을 DDS로 전달하므로 공유망에서는
SROS2를 사용한다.

## 설치 후 명령

설치 완료 화면의 절대경로 명령을 먼저 사용하면 PATH 등록 여부와 관계없이
실행할 수 있다. 설치기는 파일만 생성했으므로 첫 `elesim-up`에서 이미지를
빌드한다.

### 일반 사용자용

```bash
elesim-build                 # 선택한 이미지 build
elesim-up                    # build 후 detached 실행
elesim-logs                  # 로그 follow; Ctrl+C는 서비스가 아니라 follow만 종료
elesim-net doctor            # DDS graph/QoS/TURN/WebRTC 광고 진단
elesim-net doctor --active   # 실제 RGBD sample까지 진단
elesim-down                  # 생성 Compose project 종료
elesim-setup status          # 설치 상태 확인
```

`elesim-<role>`은 선택한 역할 하나를 foreground로 실행한다. 설치 상태는
`<설치 위치>/install-state.json`, Compose 파일은
`<설치 위치>/containers/compose.yaml`에 있다.

### 개발자용

```bash
elesim-build                 # 통짜 개발 이미지 build
elesim-up                    # 개발 컨테이너 detached 실행
elesim-logs                  # 개발 컨테이너와 선택한 Jaeger 로그
elesim-dev                   # 개발 shell
elesim-down                  # 개발 환경 종료
```

Jaeger를 선택했다면:

```bash
elesim-jaeger-up
elesim-jaeger-down
```

Jaeger UI 기본 주소는 `http://127.0.0.1:16686`이다. 개발 컨테이너 시작 시
`$HOME/.venv`에 저장소 패키지를 editable로 연결하므로 소스 수정은 즉시 반영된다.

## 단일 컴퓨터 시뮬레이션

일반 사용자용에서 `한 PC 시뮬레이션`을 선택하고 loopback 보안을 유지한다.

```bash
elesim-up
elesim-logs
```

UI에서 endpoint ID `sim-default`를 선택하면 다음 영상을 받는다.

- `observer`: 전체 Genesis 장면
- `hand-eye`: 로봇 손끝 카메라

Observer에서 orbit, pan, zoom을 조작할 수 있고 pause/resume, single-step,
reset, speed, reset-view와 debug marker 명령도 Simulator로 보낸다. 전달되는
것은 Genesis 운영체제 Viewer의 화면 캡처가 아니라 Simulator가 별도로 렌더링한
WebRTC stream이다.

## 원격 시뮬레이션 서버

### 서버 컴퓨터

1. 일반 사용자용 `시뮬레이션 서버`를 선택한다.
2. 노트북과 서버를 같은 LAN 또는 routed VPN에 놓는다.
3. 두 호스트에 같은 system/domain ID를 넣고, DDS가 사용할 LAN/VPN interface를
   선택한다.
4. L2 multicast가 불가능하면 노트북의 reachable 주소를 static peer로 넣는다.
5. 신뢰 network면 `trusted-network`, 공유 network면 `sros2`를 선택한다.
6. WebRTC direct ICE가 불가능하면 Coturn을 선택한다. Managed mode를 쓰려면
   `sros2` profile을 선택한다.
7. 설치 후 `elesim-up`을 실행한다.

```bash
# [서버]
elesim-up
elesim-logs
```

원격 Simulator profile은 native Genesis Viewer를 끄지만 observer와 hand-eye
렌더링은 유지한다.

### 조작 노트북

1. 일반 사용자용 `조작 노트북`을 선택한다.
2. 서버와 같은 system/domain ID를 넣고 LAN/VPN interface를 선택한다.
3. routed network이면 서버의 reachable 주소를 static peer로 넣는다.
4. 서버와 같은 security profile을 고른다. `sros2`이면 노트북 역할의 enclave만
   설치한다.
5. 설치 후 `elesim-up`을 실행한다.

필수 네트워크 경로:

| 용도 | 기본 경로 |
| --- | --- |
| DDS discovery와 user data | 선택한 interface의 양방향 UDP; RMW/domain 설정에 따라 결정 |
| Managed Coturn | TCP/UDP `3478`, UDP `49160-49200` |
| 직접 WebRTC ICE | 환경이 선택한 UDP candidate |

방화벽 변경은 연구실 정책과 관리자 권한이 관련되므로 설치기가 자동으로 하지
않는다. 공인 서버 `123.123.123.123`과 NAT 뒤 노트북처럼 서로 직접 라우팅되지
않는 구성은 서버 port forwarding만으로 지원되지 않는다. 이 경우 routed VPN을
사용한다. TURN이나 SSH `2222` port forwarding은 DDS 경로를 대신하지 않는다.

## 실제 Robot Jetson

Jetson에서 일반 사용자용 `Robot`만 선택한다. Robot은 다른 역할과 분리되며
JetPack/L4T, ROS2 Humble, `unitree_ros2`, 장치 권한과 로컬 안전 설정이 준비된
호스트를 전제로 native 설치한다. 실제 모터를 연결하기 전에 deadman, 제한값과
feedback 방향을 별도 검증한다.

## 네트워크 진단

```bash
elesim-net doctor
```

기본 진단은 DDS participant discovery, endpoint descriptor/heartbeat,
control/RGBD topic, TURN 연결과 Simulator signaling carrier를 확인한다.

```bash
elesim-net doctor --active --timeout 8
```

Active 진단은 실제 DDS `RgbdFrame`을 받는다. 두 WebRTC 영상과 실제 relay
candidate 선택은 일반 UI를 사용한 live 검증 항목이다.

## 종료와 보안

```bash
elesim-down
```

`elesim-logs`에서 `Ctrl+C`를 누르는 것은 로그 follow만 멈춘다. 서비스 종료에는
반드시 `elesim-down`을 사용한다.

Git에 올리면 안 되는 파일:

- SROS2 keystore의 private key
- `turn.secret`
- 외부 TURN `turn.credentials.json`
- 생성된 SROS2 keystore와 credential root
- 설치된 원격 host 설정
- `misc/infra/generated/`

서버에서 임시 X11 권한을 열었다면 종료 후 반드시 회수한다.

```bash
xhost -si:localuser:root
```

## 제거와 재설치

먼저 제거할 설치 위치를 정확히 확인한다. 아래의
`/exact/elesim/prefix`를 실제 한 설치만 가리키는 절대경로로 바꾼다.

```bash
/exact/elesim/prefix/bin/elesim-down
docker compose -f /exact/elesim/prefix/containers/compose.yaml down \
  --remove-orphans --volumes --rmi local
rm -rf /exact/elesim/prefix
```

공용 서버에서 `docker system prune`, `docker builder prune`, 전역 CUDA 변경을
사용하지 않는다. 다른 프로젝트와 사용자의 자원까지 제거할 수 있다.

PATH 등록을 제거하려면 `~/.bashrc`에서 아래 두 marker 사이만 삭제한다.

```text
# >>> Elesim managed PATH >>>
# <<< Elesim managed PATH <<<
```

삭제한 디렉터리 안에 현재 shell이 있었다면 `getcwd: cannot access parent
directories`가 나타날 수 있다. `cd ~` 또는 새 터미널로 이동한다.

## 문제 해결

### 다른 호스트가 discovery되지 않음

두 호스트의 system/domain ID, RMW implementation, 선택한 interface와 방화벽을
확인한다. routed network에서는 양쪽이 서로 도달 가능한 static peer를 사용해야
한다. static peer 설정은 NAT를 우회하지 않는다.

### `simulator is unavailable`

Simulator heartbeat가 만료됐거나 process가 재시작 중이다. 같은 endpoint ID를
주장하는 복수 boot가 탐지돼도 안전을 위해 선택이 거부된다.

```bash
docker compose -f /설치/위치/containers/compose.yaml ps
docker compose -f /설치/위치/containers/compose.yaml logs --tail=200 simulator
```

### `command not found: elesim-up`

설치 완료 화면에 나온 절대경로를 사용하거나 PATH 등록 후 새 shell을 연다.

```bash
/설치/위치/bin/elesim-up
```

### SSH 22번은 거부되고 `ssh -p 2222`는 동작함

원격 설치 GUI tunnel의 `ssh -p 2222`에 사용한다. SSH `2222`, DDS가 사용하는
UDP, TURN `3478`은 서로 다른 용도이다. 작동 중인 SSH 포트 대신 22번을 새로
열 필요가 없다.

### `Viewer closed`

Native Genesis Viewer를 닫으면 Viewer-enabled Simulator가 종료될 수 있다.
장시간 원격 실행은 installer가 생성한 headless remote profile과 UI observer
stream을 사용한다.

## 개발과 검증

개발자용 설치를 사용하거나 준비된 Python 환경에서 canonical gate를 실행한다.

```bash
python3 misc/tooling/quality/check.py --group required
python3 misc/tooling/quality/check.py --group extended
python3 misc/tooling/release/build.py
python3 misc/tooling/release/verify.py dist/releases
```

자동 테스트는 실제 DDS multicast/static-peer discovery, SROS2 enforce,
packet loss와 Wi-Fi/VPN reconnect, Genesis GPU 렌더링, 실제 NAT의 TURN relay
선택, 부하 상태의 RGBD/WebRTC 지연, RealSense, Dynamixel과 GO2의 물리 동작을
보증하지 않는다.

## 저장소 구조

```text
controller/                 Controller 배포 프로젝트
ui/                         UI 배포 프로젝트
robot/                      Robot 배포 프로젝트
simulator/                  Simulator 배포 프로젝트
packages/elesim_interfaces/ ROS 2 msg/srv/action 계약
model/bundles/default/      Simulator 완성 모델
misc/model/source/          원본 geometry와 blueprint
misc/tooling/model_builder/ 오프라인 모델 생성
misc/tooling/release/       역할별 릴리스 생성과 검증
misc/tooling/setup/         GUI 설치기와 네트워크 진단
misc/tooling/quality/       자동 테스트와 테스트 GUI
misc/integration/           멀티프로세스 통합 테스트
misc/infra/                 보안, 일반/개발 컨테이너 입력
misc/setup/                 git clone 없는 bootstrap
misc/docs/                  아키텍처와 배포 문서
```

세부 문서:

- [아키텍처](misc/docs/architecture.md)
- [설정 체계](misc/docs/configuration.md)
- [설치기 내부와 네트워크 진단](misc/docs/setup.md)
- [릴리스와 멀티호스트 배포](misc/docs/deployment.md)
- [미해결 문제](misc/docs/OPEN_ISSUES_KR.md)
