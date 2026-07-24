# Elesim 사용자 설명서

Elesim은 한 프로그램이 아니라 독립 배포 가능한 다섯 프로그램으로 구성된다.

| 프로그램 | 책임 |
| --- | --- |
| Router | endpoint 등록, lease, 메시지 라우팅, WebRTC signaling, TURN credential |
| Controller | Vision, IK, Look/Aim/Grasp, Gaze, 목표값 계산 |
| UI | 사용자 입력, 상태 표시, observer/hand-eye 영상, Simulator 조작 |
| Simulator | Genesis, 가상 telemetry, observer/hand-eye 렌더링 |
| Robot | 실제 모터·카메라 I/O, feedback, deadman, 로컬 안전 제한 |

프로그램끼리는 ZMQ protocol 또는 protocol에 광고된 RGBD/WebRTC stream으로만
통신한다. 서로 다른 컴퓨터에 설치해도 되고, 한 컴퓨터에 함께 설치해도 된다.

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

다섯 프로그램 중 이 컴퓨터에 필요한 역할을 체크한다.

| 기본 선택 | 역할 |
| --- | --- |
| 한 PC 시뮬레이션 | Router, Simulator, Controller, UI |
| 조작 노트북 | Controller, UI |
| 시뮬레이션 서버 | Router, Simulator |
| 사용자 지정 | 필요한 역할 직접 선택 |
| Robot Jetson | Robot 단독 |

Router, Simulator, Controller, UI는 역할별 Docker 이미지와 하나의 Compose
project로 구성된다. Robot은 Jetson/JetPack이 감지된 호스트에서만 선택할 수
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

### 주소와 보안

- `Router hostname/IP`: 이 컴퓨터를 포함한 모든 endpoint가 Router에 접속할
  때 사용하는 주소이다.
- `advertise hostname/IP`: 다른 컴퓨터가 이 호스트의 직접 RGBD stream에
  접속할 때 사용하는 주소이다.
- `127.0.0.1`은 같은 컴퓨터만 뜻한다. 다른 컴퓨터 주소로 사용할 수 없다.
- 원격 ZMQ에는 CurveZMQ를 사용한다. `신뢰 LAN 평문`은 암호화가 없는 명시적
  개발 예외이다.

Curve credential 선택:

- `로컬 묶음 사용`: 이미 이 호스트에 필요한 역할별 key가 있을 때 사용한다.
- `이 Router에서 생성`: Router 설치 호스트에서 새 묶음을 만든다.
- `Router 호스트에서 받기`: SSH agent 또는 선택한 SSH 개인키로 역할에 필요한
  파일만 가져온다.

SSH 수신은 hostname, SSH port, 사용자, 서버의 credential root를 입력하고
`호스트 확인`을 누른다. 표시된 host fingerprint를 사용자가 승인해야 전송한다.
비밀번호 인증은 GUI에 저장하지 않으며 지원하지 않는다. SSH/`scp`는 설치 시 key
전달 수단일 뿐, Elesim 제어·영상 통신에는 사용되지 않는다.

### TURN/Coturn

- `미사용`: 같은 LAN 또는 직접 ICE가 가능한 환경.
- `이 Router와 Coturn 실행`: Router 호스트의 생성 Compose에 Coturn을 넣는다.
- `기존 relay 사용`: 별도로 운영 중인 TURN URL을 사용한다.

Managed Coturn은 Curve credential, Router 역할, public hostname/IP와 realm이
필요하다. 선택하면 `elesim-up`, `elesim-down`, `elesim-logs`가 Coturn까지 함께
관리한다. 필요한 방화벽 경로는 TCP/UDP `3478`과 UDP `49160-49200`이다.

## 설치 후 명령

설치 완료 화면의 절대경로 명령을 먼저 사용하면 PATH 등록 여부와 관계없이
실행할 수 있다. 설치기는 파일만 생성했으므로 첫 `elesim-up`에서 이미지를
빌드한다.

### 일반 사용자용

```bash
elesim-build                 # 선택한 이미지 build
elesim-up                    # build 후 detached 실행
elesim-logs                  # 로그 follow; Ctrl+C는 서비스가 아니라 follow만 종료
elesim-net doctor            # DNS/TCP/ZMQ/TURN/WebRTC 광고 진단
elesim-net doctor --active   # 실제 RGBD와 두 WebRTC frame까지 진단
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

UI에서 `sim-default`를 선택하면 다음 영상을 받는다.

- `observer`: 전체 Genesis 장면
- `hand-eye`: 로봇 손끝 카메라

Observer에서 orbit, pan, zoom을 조작할 수 있고 pause/resume, single-step,
reset, speed, reset-view와 debug marker 명령도 Simulator로 보낸다. 전달되는
것은 Genesis 운영체제 Viewer의 화면 캡처가 아니라 Simulator가 별도로 렌더링한
WebRTC stream이다.

## 원격 시뮬레이션 서버

### 서버 컴퓨터

1. 일반 사용자용 `시뮬레이션 서버`를 선택한다.
2. Router와 advertise 주소에 노트북에서 접근 가능한 서버 IP/DNS를 입력한다.
3. CurveZMQ와 `이 Router에서 생성`을 선택한다.
4. NAT relay가 필요하면 managed Coturn을 선택한다.
5. 설치 후 `elesim-up`을 실행한다.

```bash
# [서버]
elesim-up
elesim-logs
```

원격 Simulator profile은 native Genesis Viewer를 끄지만 observer와 hand-eye
렌더링은 유지한다.

### 조작 노트북

1. 일반 사용자용 `조작 노트북`을 선택한다.
2. Router 주소에 서버 IP/DNS를 입력한다.
3. CurveZMQ와 `Router 호스트에서 받기`를 선택한다.
4. 서버 SSH 주소와 실제 포트, 사용자, 서버 credential root를 입력한다.
5. fingerprint를 확인하고 설치한다.

GUI는 Controller/UI/doctor와 Router public key 등 노트북 역할에 필요한 파일만
복사한다. 서버의 전체 credential root나 Router private key는 노트북으로
전달하지 않는다.

필수 네트워크 경로:

| 용도 | 기본 경로 |
| --- | --- |
| Router | TCP `5558` |
| 직접 CurveZMQ RGBD | TCP `5568` |
| Managed Coturn | TCP/UDP `3478`, UDP `49160-49200` |
| 직접 WebRTC ICE | 환경이 선택한 UDP candidate |

방화벽 변경은 연구실 정책과 관리자 권한이 관련되므로 설치기가 자동으로 하지
않는다.

## 실제 Robot Jetson

Jetson에서 일반 사용자용 `Robot`만 선택한다. Robot은 다른 역할과 분리되며
JetPack/L4T, ROS2 Humble, `unitree_ros2`, 장치 권한과 로컬 안전 설정이 준비된
호스트를 전제로 native 설치한다. 실제 모터를 연결하기 전에 deadman, 제한값과
feedback 방향을 별도 검증한다.

## 네트워크 진단

```bash
elesim-net doctor
```

기본 진단은 Router DNS/TCP, protocol endpoint 등록, advertised RGBD endpoint,
TURN 연결과 Simulator의 두 WebRTC 광고를 확인한다.

```bash
elesim-net doctor --active --timeout 8
```

Active 진단은 실제 RGBD multipart와 `observer`, `hand_eye_preview` frame을
받는다. Simulator의 UI session 하나를 잠시 점유하므로 일반 UI를 먼저 종료한다.
ICE 연결 성공만으로 TURN relay candidate가 실제 선택됐다고 단정할 수는 없다.

## 종료와 보안

```bash
elesim-down
```

`elesim-logs`에서 `Ctrl+C`를 누르는 것은 로그 follow만 멈춘다. 서비스 종료에는
반드시 `elesim-down`을 사용한다.

Git에 올리면 안 되는 파일:

- `*.key_secret`
- `turn.secret`
- 생성된 credential root
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

### `Address already in use ... 5558`

같은 호스트에 Router가 이미 실행 중이다.

```bash
sudo ss -lntp 'sport = :5558'
```

소유한 Compose project를 확인한 뒤 Router 하나만 남긴다.

### `simulator is unavailable`

Router에 `sim-default` Simulator가 등록되지 않았거나 재시작 중이다.

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

설치 GUI의 SSH port에 `2222`를 입력한다. Router TCP `5558`, SSH `2222`,
TURN `3478`은 서로 다른 용도이다. 작동 중인 SSH 포트 대신 22번을 새로 열
필요가 없다.

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

자동 테스트는 실제 Genesis GPU 렌더링, 실제 NAT의 TURN relay 선택, 부하 상태의
WebRTC 지연, RealSense, Dynamixel과 GO2의 물리 동작을 보증하지 않는다.

## 저장소 구조

```text
router/                     Router 배포 프로젝트
controller/                 Controller 배포 프로젝트
ui/                         UI 배포 프로젝트
robot/                      Robot 배포 프로젝트
simulator/                  Simulator 배포 프로젝트
packages/protocol/          protocol-v4 계약과 transport
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
