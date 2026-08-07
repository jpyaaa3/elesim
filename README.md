# Elesim 사용자 설명서

> **운영자용 한 줄 요약**<br>
> 설치기는 설정 파일과 실행 환경을 만들고, 실제 실행은 `elesim-up`이 담당한다.
> 여러 컴퓨터를 연결할 때는 `elesim-connections`를 별도로 실행한다.

Elesim은 하나의 거대한 프로그램이 아니라, 서로 독립적으로 배포할 수 있는 네
가지 애플리케이션으로 구성된다.

| 프로그램 | 책임 |
| --- | --- |
| Pilot | Vision, IK, Look/Aim/Grasp, Gaze, 목표값 계산 |
| UI | 사용자 입력, 상태 표시, observer/hand-eye 영상, Sim 조작 |
| Sim | Genesis, 가상 telemetry, observer/hand-eye 렌더링 |
| Robot | 실제 모터·카메라 I/O, feedback, deadman, 로컬 안전 제한 |

프로그램끼리는 ROS 2/DDS로 직접 통신한다. 현재 discovery와 RGBD는 typed ROS
message이고, 제어·WebRTC signaling은 bounded protocol-v6 DDS message를
사용한다. observer/hand-eye 영상만 WebRTC를 사용한다. 중앙 Router와
ZMQ/CURVE transport는 없다. 서로 다른 컴퓨터에 설치해도 되지만 DDS
participant 사이에 양방향 UDP 경로가 있어야 한다.

## 이 문서에서 먼저 볼 것

| 목적 | 먼저 읽을 섹션 | 핵심 명령 |
| --- | --- | --- |
| 처음 설치 | [빠른 설치](#빠른-설치) · [설치 종류](#설치-종류) | `curl ... \| bash` |
| 한 컴퓨터에서 시뮬레이션 | [단일 컴퓨터 시뮬레이션](#단일-컴퓨터-시뮬레이션) | `elesim-up` |
| 여러 컴퓨터 연결 | [연결 관리자](#연결-관리자) · [원격 시뮬레이션 호스트](#원격-시뮬레이션-호스트) | `elesim-connections` |
| 설치 전 준비·빈칸 작성 | [Zero2Omega 사용자 안내서](docs/zero2omega.md) | 설치 전제지식·입력 예시 |
| 로그·진단 | [설치 후 명령](#설치-후-명령) · [네트워크 진단](#네트워크-진단) | `elesim-logs`, `elesim-net doctor` |
| 제거·재설치 | [제거와 재설치](#제거와-재설치) | `elesim-uninstall --plan` |

### 목차

- [빠른 설치](#빠른-설치)
- [설치 종류](#설치-종류)
- [GUI 입력 항목](#gui-입력-항목)
- [설치 후 명령](#설치-후-명령)
- [단일 컴퓨터 시뮬레이션](#단일-컴퓨터-시뮬레이션)
- [원격 시뮬레이션 호스트](#원격-시뮬레이션-호스트)
- [Zero2Omega 사용자 안내서](docs/zero2omega.md)
- [실제 Robot Jetson](#실제-robot-jetson)
- [네트워크 진단](#네트워크-진단)
- [종료와 보안](#종료와-보안)
- [제거와 재설치](#제거와-재설치)
- [문제 해결](#문제-해결)
- [개발과 검증](#개발과-검증)
- [저장소 구조](#저장소-구조)

> [!WARNING]
> DDS의 `trusted-network` 프로파일은 DDS 자체 인증·암호화를 사용하지 않는다.
> 소유한 LAN 또는 접근이 제한된 VPN에서만 사용하고, 공유망에서는 `sros2`를
> 선택한다. Tailscale은 네트워크 터널이지 DDS 참가자 인증을 대신하지 않는다.

## 빠른 설치

Ubuntu 또는 WSL 터미널에서 설치할 디렉터리로 이동한 뒤 실행한다.

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/installer/bootstrap/bootstrap.sh | bash
```

`main`이 아닌 브랜치를 시험할 때는 URL과 `ELESIM_REF`를 같은 브랜치로 맞춘다.

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/refactoring/installer/bootstrap/bootstrap.sh \
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
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/refactoring/installer/bootstrap/bootstrap.sh \
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
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/installer/bootstrap/bootstrap.sh \
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

설치 GUI의 역할 화면에서 이 컴퓨터에 둘 역할을 필요한 만큼 체크한다. 한
컴퓨터에 여러 역할을 함께 둘 수 있으며, 별도의 "컴퓨터 종류" 프리셋은 없다.

| 체크할 역할 | 책임 |
| --- | --- |
| Sim | Genesis 시뮬레이션, 가상 RGBD와 WebRTC 송신 |
| Pilot | 인식, IK, Pick/Gaze와 목표 생성 |
| UI | 운영자 화면과 원격 조작 |
| Robot | Jetson의 실제 장치와 로컬 안전 제어 (단독) |

Sim, Pilot, UI는 역할별 Docker 이미지와 하나의 Compose project로
구성된다. Robot은 Jetson/JetPack이 감지된 호스트에서만 선택할 수
있으며 현재 native 단독 설치만 지원한다.

터미널 자동화에서는 `install --role sim --role pilot`처럼 역할을
반복해서 지정한다. 예전 `--profile` 인자는 기존 스크립트 호환을 위해 숨겨져
있지만 새 설치 흐름의 사용자 선택지는 역할 목록이다.

일반 Compose project 이름은 `elesim-runtime`으로 고정된다. 선택한 역할에 따라
`elesim-pilot`, `elesim-ui`, `elesim-sim`이 생기고, managed TURN을
선택한 Sim 호스트에만 `elesim-coturn`이 추가된다. Robot은
`elesim-robot` 컨테이너가 아니라 Jetson의 native/systemd 서비스다. 같은
호스트에 두 번째 일반 설치를 만들면 임의 이름을 붙이지 않고 충돌을 알려준다.
소스 디렉터리, DDS/설치 역할 키, Python 패키지(`elesim_pilot`·`elesim_sim`),
실행 명령(`elesim-pilot`·`elesim-sim`), 이미지 태그가 모두 같은 이름 체계를
사용한다. `controller`/`simulator`는 런타임 이름이 아니며, 구버전 상태를
읽을 때만 마이그레이션 입력으로 인정한다.

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

개발자판은 일반 역할 컨테이너를 함께 만들지 않는다. 고정된
`elesim-runtime-dev` project의 `elesim-dev` 하나에 네 프로그램과 개발/테스트
의존성을 모두 넣고, tracing을 선택했을 때만 별도 `elesim-jaeger`를 추가한다.
여러 터미널에서 `elesim-dev`를 실행해도 같은 상시 컨테이너에 `exec`하며 새
랜덤 이름 컨테이너를 만들지 않는다.
연결 GUI를 열 때만 `elesim-manager` one-shot 도구가 생겼다가 종료된다.
Docker 및 선택적 Tailscale 접근은 호스트의 단기 헬퍼가 허용된 Elesim
명령만 중계하며, manager에는 Docker/tailscaled socket을 넘기지 않는다.
이는 상시 애플리케이션이나 다섯 번째 역할이 아니다.

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
- `cpu`: 컨테이너 GPU 요청을 제거하고 Genesis GPU backend도 끈다. 개발
  이미지도 CUDA wheel 대신 CPU PyTorch wheel을 선택한다.

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

SROS2 key를 관리하는 방식은 두 가지다.

- `external`: 사용자가 기존 keystore와 base enclave를 직접 공급하고 교체한다.
- `managed`: 조작 노트북의 `elesim-connections`가 전체 Authority를 보관하고,
  각 host에는 공통 공개자료와 그 host에 배정된 역할 enclave만 전달한다.

일반 설치 GUI에서 `managed`를 고를 때에는 아직 keystore 경로를 입력하지 않는다.
설치기는 실행 파일과 Compose를 만들되 `<설치 위치>/security/provisioning-required`
표시를 남긴다. 조작 노트북의 `elesim-connections`가 모든 host에 한 generation을
성공적으로 적용하기 전까지 `elesim-up`과 역할 실행 명령은 안전하게 거부된다.
기존 keystore를 직접 운영할 때에만 `external`을 선택한다.

Managed mode에서 모든 컴퓨터가 서로의 공개키 목록과 CA 개인키를 통째로 들고
있을 필요는 없다. 모두 같은 공개 CA를 신뢰하고 각자 발급받은 역할 인증서와
개인키만 가진다. CA 개인키와 다른 host의 역할 key는 조작 노트북을 떠나지
않는다. key 교체는 새 generation을 모든 host에 먼저 staging한 뒤 한꺼번에
활성화하며, 중간 실패 시 이전 generation으로 되돌린다.

`ROS_DOMAIN_ID`는 우연한 graph 충돌을 줄일 뿐 인증, 접근 통제, 암호화 또는
tenant 격리를 제공하지 않는다. 최종 구조에는 ZMQ, CurveZMQ, CURVE key와 ZAP
allowlist가 없다.

설치 GUI는 계속 loopback에만 열린다. 원격 GUI를 위한 SSH `-p 2222` 같은 값은
SSH server의 포트일 뿐 DDS 설정에 들어가지 않는다.

### 연결 관리자

역할 설치가 끝난 뒤 조작 노트북에서 실행한다.

```bash
elesim-connections
```

일반 설치기의 `통신과 보안` 단계는 이제 기본값과 TURN 선택만 보여준다.
DDS 주소·인터페이스·SSH endpoint·SROS2 generation은 실행 시점에 바뀌는
토폴로지이므로 연결 관리자에서 입력한다. 연결 관리자가 SROS2 자료를 생성·배포하므로
사용자가 AES 값이나 개인키 본문을 설치기에 입력할 필요가 없다.

연결 관리자는 두 가지 명시적 topology mode를 제공한다.

- `full`: Pilot, UI, Sim, Robot을 각각 한 번씩 2~4개 host에 배치한다.
  Robot은 Jetson native/systemd 역할로 고정된다.
- `simulation-only`: 물리 Robot/Jetson 없이 Pilot, UI, Sim를 각각
  한 번씩 1~3개 host에 배치한다. 세 역할을 한 컴퓨터에 함께 둘 수도 있다.

두 mode 모두 한 컴퓨터가 여러 역할을 맡을 수 있으며 local host는 정확히 하나다.

각 host에는 서로 독립적인 두 주소를 입력한다.

- `DDS 주소/interface`: 프로그램이 UDP P2P로 실제 통신하고 static peer를 만드는
  runtime 경로
- `SSH host/port/user`: fingerprint 확인, 설치 상태 확인, SROS2 bundle 전송,
  재시작 같은 관리 경로

같은 IP를 쓸 수는 있지만 자동으로 서로 변환하지 않는다. SSH `2222`를 입력해도
DDS port가 `2222`가 되는 것이 아니다. 연결 설정에는 SSH password, 개인키 본문,
SROS2 private key, TURN secret을 저장하지 않는다.

Jetson을 당장 사용할 수 없으면 GUI에서 `simulation-only` mode로 저장·배포할 수
있다. 연결 관리자에는 일상 흐름을 벗어난 고급 탭을 두지 않고 `호스트 점검`,
`전체 정지`를 별도 유지보수 동작으로 제공한다. 호스트 점검은 저장된 topology의
모든 host에 대해 런타임 namespace, 설치/SSH 관리 경로, Compose 또는 Robot
systemd 상태를 한 번에 읽기 전용으로 검사한다. 이 점검은 DDS 양방향 통신,
RGBD, WebRTC, SROS2 권한, NAT traversal을 증명하지 않는다. `Tailscale SSH` 모드를
선택하면 개인키 없이 Tailscale 인증을 사용하고 포트는 22로 고정된다. ACL이
`action: check`라면 먼저 대화형 Tailscale SSH 재인증을 승인한다. 일반 OpenSSH over
Tailscale를 선택한 경우에만 해당 호스트의 실제 `sshd` 포트를 입력한다. `python3 -m http.server
8080`은 경로 확인용 임시 HTTP일 뿐 DDS/SSH 설정이 아니다. `full` mode의
정식 저장·배포에만 Robot을 포함한 네 역할 topology가 필요하다.

정상 작업은 `검증 후 저장` → `보안 자료 생성·재발급 및 실행 준비` →
`전체 시작` 순서다. 이 버튼은 첫 실행이면 보안 자료를 생성·검증하고, 이미 활성
세대가 있으면 새 세대로 재발급·검증한다. `Abort`는 실행 중인 현재 작업을 안전한
경계에서 취소한다. `호스트 점검`은 보안 세대를 만들거나
런타임을 시작하지 않는다. `전체 정지`는 활성 역할을 중단하지만 이미 발급된
보안 세대를 되돌리지는 않는다. 로컬에 `tailscale0`가 있으면 현재 주소를 읽기 전용으로 자동 제안하지만
Tailscale 설치·로그인·ACL 변경은 하지 않는다. 재연결 뒤 주소가 바뀌었는지 확인한
뒤 저장한다. `전체 시작`은 모든 호스트의 이미지를 먼저 완성한 뒤 역할을
시작한다. 시작 전에는 설정한 DDS interface가 실제 역할 컨테이너와 같은 network
namespace에 보이는지도 가볍게 검사한다. `tailscale0`은 직접 bind 대상으로
정상 보존되며, 런타임 namespace에 실제로 존재할 때 통과한다. 특히 Docker
Desktop의 별도 Linux VM에서는 WSL의 `tailscale0`가 보이지 않을 수 있다. 이 경우
SROS2 키 발급 자체가 실패하는 것은 아니지만, 런타임 시작은 직접 bind 검사에서
실패한다. 직접 bind에는 같은 WSL/Linux namespace에서 동작하는 Docker Engine이
필요하며, 컨테이너 안에서 route되는 다른 interface를 선택하는 것은 별도의
라우팅/NAT 모드다. SSH용 Tailscale helper는 DDS를 relay하지 않는다. 이때 각
호스트의 실제 `docker compose --progress plain build` 출력이
연결 관리자 작업 기록과 `elesim-connections`를 실행한 터미널에 COM 이름과 함께
실시간 표시된다. 이미지 빌드는 아직 컨테이너가 실행되기 전의 작업이므로
`docker logs`나 `docker events`를 따로 볼 필요가 없다. 보안 초기 배포와 회전은
원래 실행 중이던 역할만 다시 시작하며,
처음부터 꺼져 있던 역할을 임의로 켜지 않는다. 보안 배포 중단으로 세대가
불일치한 경우의 복구 API는 내부 안전장치로 유지되며, 일반 작업자는 먼저
`Abort`하고 `호스트 점검` 결과를 확인한 뒤 재시도한다.

### TURN/Coturn

- `미사용`: 같은 LAN 또는 직접 ICE가 가능한 환경.
- `이 Sim와 Coturn 실행`: Sim 호스트의 생성 Compose에 Coturn을
  넣는다.
- `기존 relay 사용`: 별도로 운영 중인 TURN URL을 사용한다.

Managed Coturn은 public hostname/IP, realm과 credential 정책이 필요하다.
REST HMAC을 쓰면 static secret은 Coturn과 같은 호스트의 Sim만 갖고,
Sim가 활성 session에 묶인 단기 ICE credential을 UI에 발급한다. UI에는
static secret을 전달하지 않는다. 선택하면 `elesim-up`,
`elesim-down`, `elesim-logs`가 Coturn까지 함께 관리한다. 필요한 방화벽 경로는
TCP/UDP `3478`과 UDP `49160-49200`이다. TURN은 WebRTC media relay이며 DDS
topic이나 signaling을 연결해 주지 않는다. Managed TURN credential과 signaling은
DDS로 전달되므로 managed mode는 `sros2` profile을 요구한다.

외부 TURN을 Sim 호스트에 설치할 때에는 relay가 발급한 자격증명 JSON도
선택한다. 파일 형식은
`{"username":"...","credential":"...","expires_at":4102444800}`이며
`expires_at`은 장기 credential이면 생략할 수 있다. 이 파일은 Sim
컨테이너에만 read-only로 mount되고, Pilot/UI 전용 노트북에는 복사되지
않는다. Sim가 활성 UI session에 필요한 값을 DDS로 전달하므로 공유망에서는
SROS2를 사용한다.

## 설치 후 명령

설치 완료 화면의 절대경로 명령을 먼저 사용하면 PATH 등록 여부와 관계없이
실행할 수 있다. Container 역할은 설치기가 build context만 생성하므로 첫
`elesim-up`에서 이미지를 빌드한다. Native Robot은 설치 중 전용 venv를 만들고
`elesim-up`은 등록된 systemd unit을 시작한다. 단, managed SROS2 설치는 먼저
조작 노트북에서 `elesim-connections`로 전체 host generation을 적용해야 한다.

> **실행 순서**<br>
> 설치 완료 → `source ~/.bashrc`(PATH 등록을 선택한 경우) →
> `elesim-up` → 필요할 때 `elesim-connections`.

### 일반 사용자용

```bash
elesim-build                 # Container 역할에서 선택한 이미지 build
elesim-update                # 같은 branch의 새 소스와 이미지를 증분 갱신
elesim-up                    # Container는 build/detach, Robot은 systemd start
elesim-logs                  # 로그 follow; Ctrl+C는 서비스가 아니라 follow만 종료
elesim-logs --save           # 선택한 runtime의 bounded text snapshot 저장
elesim-net doctor            # DDS graph/QoS/TURN/WebRTC 광고 진단
elesim-net doctor --active   # 실제 RGBD sample까지 진단
elesim-connections           # 조작 호스트의 multi-host 배치·managed SROS2 관리
elesim-down                  # 선택한 runtime 종료
elesim-setup status          # 설치 상태 확인
```

아직 역할 컨테이너가 하나도 생성되지 않았다면 `elesim-logs`와
`elesim-logs --save`는 조용히 끝나지 않고 `elesim-up`이 필요하다고 알린다.
`elesim-down`은 이미 정지한 runtime에 대해 Compose 경고를 내지 않는 안전한
no-op이다.

여러 호스트를 시작하면 연결 관리자가 컨테이너 생성과 DDS 애플리케이션 피어
발견을 구분해 `DDS readiness`를 표시한다. Sim의 Genesis 장면을 만드는 동안에는
`대기 중`일 수 있으며, 이 진단은 느린 Sim을 자동으로 내리지 않는다. 특정 피어를
엄격하게 확인할 때는 IP/SSH 이름이 아니라 endpoint ID를 사용한다.

```bash
elesim-net doctor --json --strict-peers \
  --expect-peer sim-default --timeout 8
```

이 검사는 DDS endpoint descriptor까지만 확인한다. RGBD 프레임, WebRTC
ICE/DTLS-SRTP, SROS2 권한과 실제 하드웨어 동작은 별도의 검증이다.

실제 그래픽 세션이 있는 호스트에서 native Genesis Viewer를 일시적으로
열려면 Sim 역할이 설치된 상태에서 다음처럼 실행한다. 기본 `elesim-up`은
계속 headless로 실행된다.

```bash
DISPLAY=:0 CUDA_VISIBLE_DEVICES=0 elesim-up --view
```

`--view`는 실행 직전에 현재 DISPLAY의 `xhost +si:localuser:root` 권한을
필요할 때만 임시로 추가하고, `elesim-down`에서 Elesim이 추가한 권한만
`xhost -si:localuser:root`로 회수한다. 이미 있던 권한은 회수하지 않는다.
SSH X11 forwarding, X11 인증 또는 WSLg가 준비되어 있어야 한다. Viewer를
닫으면 Sim가 종료될 수 있으므로, 장시간 원격 운용에는 UI의 WebRTC
observer stream을 사용한다.

`elesim-<role>`은 선택한 역할 하나를 foreground로 실행한다. 설치 상태는
`<설치 위치>/install-state.json`에 있다. Container 설치의 Compose 파일은
`<설치 위치>/containers/compose.yaml`이다. Native Robot은 설치 과정에서 venv를
직접 만들므로 `elesim-build`가 없고, 연결 관리는 조작 노트북에서 수행하므로
Jetson에 `elesim-connections`를 만들지 않는다. 대신 아래에서 설명하는 두
systemd unit을 `elesim-up`/`elesim-down`이 제어한다.

`elesim-update`는 설치 때 기록한 repository/ref에서 최신 bootstrap과 source를
받고 ownership manifest를 다시 검증한 뒤, 같은 prefix의 설치기 소유 파일만
갱신한다. Container 설치에서는 선택한 역할과 tools 이미지를 Docker cache로
증분 build한다. 설정, connection topology, managed SROS2 generation,
Authority, secret, cache와 log는 보존한다. 실행 중 컨테이너도 건드리지 않으므로
성공 후 `elesim-up`을 한 번 실행해야 새 이미지가 적용된다.

일반 설치의 `runtime text log archive`는 기본으로 켜져 있다. Docker 자체
`json-file` 로그는 서비스마다 `10 MiB × 4`로 제한되고, `--save` 또는
`elesim-down`은 `<설치 위치>/logs/runs/<UTC timestamp>/`에 서비스별 text
snapshot을 남긴다. 최근 5회만 보존하며 디렉터리는 `0700`, 파일은 `0600`이다.
필요 없으면 설치 화면에서 끌 수 있다. `elesim-down`은 snapshot 실패 여부와
상관없이 종료를 시도하고, 저장 실패는 non-zero status로 알린다. 역할
컨테이너가 이미 정지한 경우에는 archive를 건너뛰고 성공적으로 종료한다.

### 개발자용

개발자용 `elesim-up`은 애플리케이션 역할이 아니라 상시 개발 컨테이너
`elesim-dev`를 시작한다. 연결 관리자는 자동으로 실행되지 않으며, 필요할 때
다음 명령을 별도로 실행한다.

```bash
elesim-build                 # 통짜 개발 이미지 build
elesim-update                # clean checkout을 fast-forward하고 개발 이미지 증분 build
elesim-up                    # 개발 컨테이너 detached 실행
elesim-logs                  # 개발 컨테이너와 선택한 Jaeger 로그
elesim-dev                   # 개발 shell
elesim-down                  # 개발 환경 종료
elesim-connections           # 멀티호스트 topology·보안 연결 관리자
```

Jaeger를 선택했다면:

```bash
elesim-jaeger-up
elesim-jaeger-down
```

Jaeger UI 기본 주소는 `http://127.0.0.1:16686`이다. 개발 컨테이너 시작 시
`$HOME/.venv`에 저장소 패키지를 editable로 연결하므로 소스 수정은 즉시 반영된다.
개발자용 update는 tracked 변경이 staged/unstaged 상태로 남아 있으면 거부하고,
설치 때 선택한 ref의 fast-forward만 허용한다.
호스트에 pytest, ROS2 또는 과학 패키지를 별도 설치하지 말고 다음처럼 실행한다.

```bash
elesim-dev python3 misc/tools/quality/check.py --group required
```

## 단일 컴퓨터 시뮬레이션

일반 사용자용에서 Sim, Pilot, UI 역할을 체크하고 loopback
interface로 제한한 `trusted-network` profile을 사용한다.

```bash
elesim-up
elesim-logs
```

UI에서 endpoint ID `sim-default`를 선택하면 다음 영상을 받는다.

- `observer`: 전체 Genesis 장면
- `hand-eye`: 로봇 손끝 카메라

Observer에서 orbit, pan, zoom을 조작할 수 있고 pause/resume, single-step,
reset, speed, reset-view와 debug marker 명령도 Sim로 보낸다. 전달되는
것은 Genesis 운영체제 Viewer의 화면 캡처가 아니라 Sim가 별도로 렌더링한
WebRTC stream이다.

## 원격 시뮬레이션 호스트

### Sim를 실행하는 호스트

1. 일반 사용자용에서 Sim 역할을 체크한다.
2. 조작 호스트와 이 호스트를 같은 LAN 또는 routed VPN에 놓는다.
3. 두 호스트에 같은 system/domain ID를 넣고, DDS가 사용할 LAN/VPN interface를
   선택한다.
4. L2 multicast가 불가능하면 조작 호스트의 reachable 주소를 static peer로 넣는다.
5. 신뢰 network면 `trusted-network`, 공유 network면 `sros2`를 선택한다.
6. WebRTC direct ICE가 불가능하면 Coturn을 선택한다. Managed mode를 쓰려면
   `sros2` profile을 선택한다.
7. Managed SROS2이면 먼저 조작 호스트의 `elesim-connections`에서 모든 host에
   generation을 적용한 뒤 `elesim-up`을 실행한다.

```bash
# [Sim 호스트]
elesim-up
elesim-logs
```

Sim 호스트에 실제 X11/WSLg 디스플레이가 있고 native Genesis Viewer가
필요한 경우에만 다음처럼 일회성으로 Viewer를 켤 수 있다.

```bash
DISPLAY=:0 CUDA_VISIBLE_DEVICES=0 elesim-up --view
```

`--view`는 설치된 headless 설정이나 SROS2 자료를 변경하지 않는다. 단지
이번 실행의 Sim 컨테이너에 Viewer 플래그를 전달하고 X11 권한을 임시 관리한다.
`elesim-down`을 실행하면 이 실행에서 추가한 권한을 회수한다. 디스플레이가
없는 원격 compute host에서는 기본 `elesim-up`과 UI WebRTC observer를 사용한다.

원격 Sim profile은 native Genesis Viewer를 끄지만 observer와 hand-eye
렌더링은 유지한다.

### 조작 호스트 (노트북 등)

1. 일반 사용자용에서 Pilot와 UI 역할을 체크한다.
2. Sim 호스트와 같은 system/domain ID를 넣고 LAN/VPN interface를 선택한다.
3. routed network이면 Sim 호스트의 reachable 주소를 static peer로 넣는다.
4. Sim 호스트와 같은 security profile을 고른다. 외부 keystore를 쓸 때에는 조작 호스트
   역할 enclave만 설치한다.
5. `elesim-connections`에서 조작 호스트를 local host로 두고 Sim 호스트/Jetson의 DDS 주소와
   별도 SSH 관리 주소를 입력한다. Managed SROS2를 선택했다면 여기서 모든 host의
   role bundle을 같은 generation으로 provision한다.
6. Provision이 성공한 뒤 각 host에서 `elesim-up`을 실행한다.

필수 네트워크 경로:

| 용도 | 기본 경로 |
| --- | --- |
| DDS discovery와 user data | 선택한 interface의 양방향 UDP; RMW/domain 설정에 따라 결정 |
| Managed Coturn | TCP/UDP `3478`, UDP `49160-49200` |
| 직접 WebRTC ICE | 환경이 선택한 UDP candidate |

방화벽 변경은 연구실 정책과 관리자 권한이 관련되므로 설치기가 자동으로 하지
않는다. 공인 주소 `123.123.123.123`과 NAT 뒤 노트북처럼 서로 직접 라우팅되지
않는 구성은 한 호스트의 port forwarding만으로 지원되지 않는다. 이 경우 routed VPN을
사용한다. TURN이나 SSH `2222` port forwarding은 DDS 경로를 대신하지 않는다.

## 실제 Robot Jetson

Jetson에서 일반 사용자용 `Robot`만 선택한다. Robot은 다른 역할과 분리되며
JetPack/L4T, ROS2 Humble, `unitree_ros2`, 장치 권한과 로컬 안전 설정이 준비된
호스트를 전제로 native 설치한다. 실제 모터를 연결하기 전에 deadman, 제한값과
feedback 방향을 별도 검증한다.

설치기는 현재 계정·설치 경로를 반영한 두 unit을 만든다.

- `elesim-robot.service`: Elesim DDS/SROS2, 팔·카메라와 로컬 안전을 소유한다.
- `elesim-unitree-bridge.service`: 전용 `elesim-unitree` 계정으로 GO2의
  local/plaintext DDS만 소유한다.

두 process는 `/run/elesim-unitree/bridge.sock`의 bounded
`SOCK_SEQPACKET` IPC로만 통신한다. Robot 사용자는 `elesim-unitree` 그룹을
통해 socket에 접근하고 양쪽은 `SO_PEERCRED`로 상대 UID를 확인한다. GO2 DDS
interface/domain은 Elesim DDS interface/domain과 달라야 하며, Unitree 전용
물리 NIC에만 묶어야 한다. Tailscale/LAN interface에 Unitree topic을 노출하면
안 된다. Unitree 기본값은 `eth0`/domain `1`이며 필요하면 bootstrap 전에
`ELESIM_UNITREE_INTERFACE`와 `ELESIM_UNITREE_DOMAIN_ID`를 지정한다.
`UNITREE_ROS2_WS` 또는 `ELESIM_UNITREE_ROS2_WS`로 실제 `unitree_ros2`
workspace를 지정할 수 있으며 기본은 `$HOME/ros2_ws`다.

설치기는 전용 계정·그룹, ACL과 두 unit을 등록하는 정확한 명령을 출력할 뿐
`sudo`나 서비스 시작은 자동 실행하지 않는다. Managed SROS2라면 먼저 unit을
등록·enable만 하고 `elesim-connections`가 전체 호스트에 키를 배포할 때까지
시작하지 않는다. Robot의 `elesim-logs`는 두 unit의 journald를 함께 follow하고
일반 설치와 같은 bounded snapshot을 지원한다. `dist/releases/robot`은 독립
수동 배포용이며 현재 연결관리자 호스트로 쓸 수 없다.

## 네트워크 진단

```bash
elesim-net doctor
```

기본 진단은 DDS participant discovery, endpoint descriptor/heartbeat,
control/RGBD topic, TURN 연결과 Sim signaling carrier를 확인한다.

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

- 조작 노트북의 SROS2 Authority와 모든 CA/역할 private key
- host별 managed SROS2 bundle과 generation 활성화 상태
- `turn.secret`
- 외부 TURN `turn.credentials.json`
- 생성된 SROS2 keystore와 credential root
- 설치된 원격 host 설정과 연결 관리자 topology
- `environment/generated/`

서버에서 임시 X11 권한을 열었다면 종료 후 반드시 회수한다.

```bash
xhost -si:localuser:root
```

## 제거와 재설치

설치 GUI의 `기존 설치 클린 제거`는 ownership manifest를 확인하고 host
터미널에서 실행할 정확한 명령만 출력한다. GUI 자체는 Docker나 파일을 삭제하지
않는다. 설치가 만든 host 전용 `elesim-uninstall`로 plan을 확인하거나 곧바로
제거할 수 있다.

```bash
elesim-uninstall --plan
elesim-uninstall
```

제거기는 설치 UUID, exact wrapper hash, Compose project/config path와 Docker
label을 모두 다시 확인한 뒤 이 설치가 소유한 고정 container와
`elesim/*:local` image만 제거한다. 외부 image, 외부 TURN credential, 외부
SROS2 keystore와 source checkout은 제거하지 않는다. runtime 설정·키·secret,
text log와 이 설치가 소유한 조작 노트북 SROS2 Authority는 기본 삭제한다. 로그나
Authority를 남겨야 할 때만 각각 `--keep-logs`, `--keep-authority`를 붙인다.
완료 기록은 삭제되는 prefix 밖의
`${XDG_STATE_HOME:-~/.local/state}/elesim/uninstall/`에 남는다.

native Robot의 두 systemd unit이 설치되어 있거나 실행 중이면 제거기는 아무
것도 바꾸기 전에 중단하고, hash가 일치하는 unit에 한해 정확한
`disable --now`/unit 삭제 명령을 안내한다. 그 명령을 실행한 뒤 plan부터 다시
실행한다.

공용 서버에서 `docker system prune`, `docker builder prune`, wildcard 삭제,
전역 CUDA 변경을 사용하지 않는다. PATH 관리 block도 manifest와 정확히
일치할 때 제거기가 원자적으로 정리한다.

삭제한 디렉터리 안에 현재 shell이 있었다면 `getcwd: cannot access parent
directories`가 나타날 수 있다. `cd ~` 또는 새 터미널로 이동한다.

## 문제 해결

### 다른 호스트가 discovery되지 않음

두 호스트의 system/domain ID, RMW implementation, 선택한 interface와 방화벽을
확인한다. routed network에서는 양쪽이 서로 도달 가능한 static peer를 사용해야
한다. static peer 설정은 NAT를 우회하지 않는다.

### `sim is unavailable`

Sim heartbeat가 만료됐거나 process가 재시작 중이다. 같은 endpoint ID를
주장하는 복수 boot가 탐지돼도 안전을 위해 선택이 거부된다.

```bash
docker compose -f /설치/위치/containers/compose.yaml ps
docker compose -f /설치/위치/containers/compose.yaml logs --tail=200 sim
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

Native Genesis Viewer를 닫으면 Viewer-enabled Sim가 종료될 수 있다.
장시간 원격 실행은 installer가 생성한 headless remote profile과 UI observer
stream을 사용한다.

## 개발과 검증

개발자용 설치를 사용하거나 준비된 Python 환경에서 canonical gate를 실행한다.

```bash
elesim-dev python3 misc/tools/quality/check.py --group required
elesim-dev python3 misc/tools/quality/check.py --group extended
elesim-dev python3 misc/tools/release/build.py
elesim-dev python3 misc/tools/release/verify.py dist/releases
```

자동 테스트는 실제 DDS multicast/static-peer discovery, SROS2 enforce,
packet loss와 Wi-Fi/VPN reconnect, Genesis GPU 렌더링, 실제 NAT의 TURN relay
선택, 부하 상태의 RGBD/WebRTC 지연, RealSense, Dynamixel과 GO2의 물리 동작을
보증하지 않는다.

## 저장소 구조

```text
pilot/                         Pilot 배포 프로젝트
ui/                            UI 배포 프로젝트
robot/                         Robot 배포 프로젝트
sim/                           Sim 배포 프로젝트
packages/elesim_interfaces/    ROS 2 msg/srv/action 계약
packages/protocol/             DDS 계약·공용 전송 기반
model/source/                  원본 geometry와 blueprint
model/builder/                 오프라인 모델 생성기
model/bundles/default/         Sim 완성 모델
environment/                  Docker·Compose·TURN·개발환경 입력
installer/bootstrap/           git clone 없는 bootstrap
installer/package/             GUI 설치기·연결/SROS2 관리자
misc/system_tests/            멀티프로세스 시스템 검증
misc/research/                분석·실험·디버그·결과
misc/tools/                   품질·릴리스·개발 도구
docs/                         아키텍처·설치·배포 문서
```

세부 문서:

- [아키텍처](docs/architecture.md)
- [설정 체계](docs/configuration.md)
- [설치기 내부와 네트워크 진단](docs/setup.md)
- [릴리스와 멀티호스트 배포](docs/deployment.md)
- [Zero2Omega 사용자 안내서](docs/zero2omega.md)
- [마일스톤과 남은 인수시험](docs/MILESTONES.md)
- [미해결 문제](docs/OPEN_ISSUES_KR.md)
