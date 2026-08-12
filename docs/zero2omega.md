# Zero2Omega 사용자 안내서

> 설치 화면에서 무엇을 알고 있어야 하는지, 연결관리자의 빈칸을 어떻게
> 채우는지, 무엇을 비워 두어도 되는지를 처음부터 끝까지 설명하는 문서다.

이 문서에서 **Zero2Omega**는 현재 EleSim의 설치·연결·실행 흐름을 가리키는
사용자용 이름으로 사용한다. 현재 구조는 중앙 Router나 ZMQ를 사용하지 않고,
Pilot·Sim·UI·Robot이 ROS 2/DDS로 직접 통신한다. 카메라 영상의 픽셀만
WebRTC(DTLS/SRTP)로 전달되고, WebRTC의 연결 협상도 DDS를 통해 전달된다.

먼저 기억할 한 문장은 이것이다.

> **DDS IP와 SSH IP는 서로 다른 경로다. Native host network에서는 같을 수
> 있지만, Docker Desktop sidecar에서는 DDS는 sidecar IP를, SSH는 WSL/호스트
> IP를 사용한다.**

---

## 1. 시작 전에 결정할 것

다음 여섯 가지를 정하면 설치 화면에서 거의 막히지 않는다.

| 결정할 것 | 선택 기준 |
| --- | --- |
| 물리 Robot 사용 여부 | Jetson을 지금 사용할 수 없으면 `simulation-only`를 선택한다. |
| 역할 배치 | Pilot, Sim, UI를 어느 컴퓨터에서 실행할지 정한다. Robot은 Jetson 한 대에만 둔다. |
| 네트워크 | 같은 L2 LAN이면 multicast, Tailscale 같은 routed VPN이면 static을 선택한다. |
| DDS 주소 | 각 활성 runtime namespace에서 다른 컴퓨터가 도달할 수 있는 IP 또는 hostname을 확인한다. |
| 관리 경로 | 원격 컴퓨터에는 Tailscale SSH 또는 일반 OpenSSH 중 하나를 정한다. |
| DDS 보안 | 기본 권장은 `sros2`; 정말 소유한 LAN/VPN에서만 `trusted-network`를 사용한다. |

### 1.1 사용자가 미리 알아야 하는 정보

- 설치할 각 컴퓨터의 로그인 사용자 이름.
- 각 설치가 **Native host network**인지 **Docker Desktop Tailscale sidecar**인지.
  설치기가 Docker backend를 보고 자동 선택한 뒤 그 결과를 고정한다.
- Native host network에서는 host의 현재 Tailscale IP/hostname과 `tailscale0`.
  Sidecar에서는 `elesim-tailscale status`가 보여 주는 별도 sidecar IP와
  sidecar namespace의 `tailscale0`.
- 각 컴퓨터에서 실제로 설치된 EleSim prefix와 `bin/` 경로.
  예를 들어 `/home/user/ws/five`와 `/home/user/ws/five/bin`처럼 입력한다.
- 원격 컴퓨터를 관리할 SSH 사용자와 포트. Tailscale SSH는 포트 `22`를
  사용하고, 일반 OpenSSH는 실제 sshd 포트를 사용한다.
- 모든 DDS runtime node가 같은 tailnet에 등록되어 있는지. Docker Desktop
  sidecar는 WSL에 로그인된 기존 Tailscale을 물려받지 않으므로 처음 한 번
  `elesim-tailscale login`으로 별도 등록한다.
- 원격 SSH에 접근할 수 있는지. ACL에 `action: check`가 있으면 처음 한 번은
  사람이 직접 Tailscale SSH 재인증을 승인해야 할 수 있다.

### 1.2 사용자가 알 필요 없는 것

다음은 연결관리자가 생성하거나 DDS/WebRTC가 runtime에 정하는 값이다.

- DDS 애플리케이션 포트 번호.
- DDS 포트포워딩 목록.
- 개인키·공개키의 내용, SROS2 CA 개인키, `turn.secret`의 내용.
- `python3 -m http.server 8080`으로 열어 둔 임시 HTTP 포트.
- 노트북에 별도의 “수신용 IP”를 만드는 작업.

`8080`, SSH `2222`, TURN `3478`, DDS UDP locator는 서로 다른 계층의 값이다.
특히 SSH 포트포워딩 `ssh -L ... -p 2222`는 설치 GUI에 접근하기 위한 통로일
뿐, DDS나 WebRTC 포트가 아니다.

---

## 2. 추천 구성

### 2.1 Jetson이 아직 없을 때

가장 안전한 첫 성공 경로는 다음과 같다.

```text
노트북  ── Pilot + UI
서버컴  ── Sim
          (둘 다 Tailscale, simulation-only)
```

세 역할을 한 컴퓨터에 모두 설치해도 된다. 이 경우 호스트 한 대만 활성화한
`simulation-only` topology를 저장한다.

### 2.2 Jetson을 사용할 때

`full` topology에서만 Robot을 추가한다.

```text
조작 컴퓨터 ── Pilot + UI
연산 컴퓨터 ── Sim
Jetson      ── Robot (native/systemd) + 선택적 Pilot/UI (별도 Compose unit)
```

`full`은 Robot을 포함해 활성 호스트 2~4대, `simulation-only`는 Robot 없이
활성 호스트 1~3대를 허용한다. 두 경우 모두 각 역할은 정확히 한 번만 배치한다.

---

## 3. 설치 마법사에서 입력하는 법

### 3.1 설치 시작

설치할 컴퓨터에서 원하는 디렉터리로 이동해 bootstrap을 실행한다.

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/refactoring/installer/bootstrap/install.sh \
  | ELESIM_REF=refactoring bash
```

안정 릴리스로 설치할 때는 `refactoring`을 `main` 또는 선택한 40자리 commit
SHA로 바꾼다. 설치 GUI는 호스트 Python·CUDA·ROS·APT를 바꾸지 않고 파일과
Compose context만 만든다. 이미지는 첫 `elesim-up` 또는 `elesim-build`에서
생성된다.

브라우저를 닫기 전에 설치 완료 화면의 절대경로를 확인한다. PATH 등록을
선택했다면 현재 터미널에는 즉시 반영되지 않으므로 한 번 실행한다.

```bash
source ~/.bashrc
```

### 3.2 설치 종류

#### 일반 사용자용

필요한 역할만 체크한다. “노트북”, “서버” 같은 컴퓨터 종류를 고르는 방식이
아니다.

- Sim: Genesis와 가상 RGBD/WebRTC 송신.
- Pilot: 인식·IK·Pick/Gaze·목표 생성.
- UI: 조작 화면·상태 표시·영상 수신.
- Robot: Jetson에서 실제 장치와 안전 제어. Jetson 감지 시에만 선택한다.

한 컴퓨터에 여러 역할을 체크해도 된다. 일반 설치의 컨테이너 이름은 선택한
역할에 따라 `elesim-pilot`, `elesim-sim`, `elesim-ui`가 되고, Robot은
컨테이너가 아니라 native systemd 서비스다.

#### 개발자용

전체 저장소와 하나의 `elesim-dev` 컨테이너를 만든다. `elesim-connections`도
이 환경에서 실행한다. 개발자용이라고 해서 일반 역할 컨테이너 네 개를 따로
설치할 필요는 없다. Jaeger는 선택 사항이다.

### 3.3 설치 화면의 빈칸

| 항목 | 입력 | 생략/기본값 |
| --- | --- | --- |
| 설치 prefix | 이 컴퓨터에서 실제로 사용할 절대경로 | 현재 `curl`을 실행한 디렉터리 기본값을 그대로 써도 된다. |
| command/bin directory | 보통 `<prefix>/bin` | 기본값을 유지한다. |
| 역할 체크박스 | 이 컴퓨터에 둘 역할만 체크 | 필요 없는 역할은 체크하지 않는다. |
| GPU 정책 | GPU 사용이면 `inherit`/`specific`, CPU면 `cpu` | 잘 모르면 `inherit`; CPU-only면 `cpu`. |
| SROS2/통신 값 | 일반 설치에서는 연결관리자에서 설정 | AES 값, 개인키 내용, DDS 포트는 입력하지 않는다. |
| TURN/ICE | Sim이 direct ICE를 먼저 시도하고, SROS2에서만 managed Coturn fallback을 사용 | trusted-network는 TURN 없이 동작한다. Coturn은 Sim runtime의 일부이며 별도 호스트 role/card가 아니다. |

설치기는 Docker backend를 자동 판별해 `direct-host` 또는
`tailscale-sidecar` 결과만 저장한다. 사용자가 매번 Docker context를 바꾸는
기능은 아니다. Docker Desktop sidecar가 생성된 컴퓨터에서는 연결관리자를
열기 전에 다음을 한 번 실행한다.

```bash
elesim-tailscale login
elesim-tailscale status
```

브라우저/device 로그인을 완료하고 `status`의 tailnet IP를 적어 둔다. EleSim은
Tailscale auth/OAuth key나 브라우저 credential을 저장하지 않는다. 일반적인
down/up/update는 sidecar node state를 보존하므로 매번 로그인할 필요가 없다.

설치가 끝난 뒤에는 각 컴퓨터에서 역할을 바로 시작하기보다, managed SROS2를
선택했다면 먼저 조작 컴퓨터에서 연결관리자의 보안 세대를 적용한다.

### 3.4 `통신과 보안` 페이지의 숨은 항목과 TURN

현재 설치 GUI의 DDS 주소·static peer·설치용 SSH 입력란은 연결관리자와
겹치므로 숨겨져 있다. 이 항목을 억지로 채우거나 예전 설명서의 값을 복사하지
말고, 설치 후 `elesim-connections`에서 각 host의 현재 값을 입력한다.

TURN은 Sim의 WebRTC media relay일 뿐 DDS를 중계하지 않는다. `trusted-network`는
direct ICE만 사용하고 Coturn을 만들지 않는다. SROS2를 선택한 Sim은 설치된
Coturn endpoint를 사용하며, static secret은 Sim과 Coturn에만 남는다. 연결관리자는
Sim 카드 하단에 relay 입력란을 만들지 않고 Sim 호스트의 `elesim-net show`에서
endpoint를 읽어 검증한 뒤 적용한다. 새 설치 관리자에서는 외부 relay URL/credential
파일을 입력할 수 없고, 이전 상태의 external 호환 경로는 하위 런타임에서만
읽을 수 있다.

`PATH에 등록`은 선택 사항이다. 선택하지 않아도 설치 완료 화면에 나온
`<prefix>/bin/elesim-up` 같은 절대경로로 실행할 수 있다. `로컬 평문 로그 보관`
은 `elesim-logs --save`와 종료 시 최대 다섯 개의 제한된 로그 snapshot을 남기는
기능이며, 중앙 서버로 전송하지 않는다.

SROS2 provisioning은 `managed`가 기본이다. 이 경우 keystore 경로와 base
enclave를 비워 두고 연결관리자가 generation을 만든다. 이미 EleSim 밖에서
관리하는 keystore를 쓰는 고급 `external` 경로를 의도적으로 선택한 경우에만
기존 keystore 디렉터리와 base enclave(예: `/elesim`)를 지정한다. 새 키를 이
설치기에 붙여 넣거나 managed/external을 실행 중에 임의로 섞지 않는다.

---

## 4. 연결관리자 열기

연결관리자는 **조작 컴퓨터 한 대**에서만 연다. 보통 UI/Pilot을 쓰는 노트북을
로컬 host로 선택한다.

```bash
elesim-connections
```

브라우저 URL은 터미널에 출력된다. 연결관리자 페이지가 이미 떠 있으면 같은
포트를 두 번 열지 않는다. `Address already in use`가 나오면 이전에 실행한
연결관리자/설치 GUI를 먼저 닫고, 실제 작업이 없을 때만 정확한
`elesim-manager` transient container를 정리한다. 최신 wrapper는 중지 상태의
stale manager를 다음 실행 때 자동 정리한다. 현재 실행이 소유한 manager는
종료 신호를 받아도 고정 이름이 남지 않도록 정리하며, 다른 실행이 소유한
실행 중 manager는 건드리지 않는다. 전역 `docker prune`은 사용하지 않는다.
필요하면 새 포트를 지정한다.

```bash
elesim-connections --port 8771
```

페이지가 `127.0.0.1`에만 열리는 것은 정상이다. 다른 컴퓨터의 브라우저에서
보려면 연결관리자를 실행한 컴퓨터에 SSH tunnel을 만들고, 그 SSH 포트만
`-p`로 지정한다. tunnel의 포트는 DDS 설정에 넣지 않는다.

로컬 카드에서 `이 컴퓨터가 연결 관리자를 실행함`을 선택하면 그 카드의
`설치용 SSH` 영역은 숨겨진다. 로컬 host에 SSH를 입력하면 검증 오류가 난다.

---

## 5. 연결관리자 입력란별 설명

### 5.1 DDS 공통 설정

이 값들은 활성 host 모두가 **같게** 가져야 한다.

| 입력란 | 무엇을 쓰나 | 언제 바꾸나 |
| --- | --- | --- |
| 토폴로지 모드 | `full` 또는 `simulation-only` | Jetson이 없으면 `simulation-only`. |
| 시스템 ID | 기본 `elesim` 같은 소문자 ID | 다른 DDS graph와 충돌할 때만 바꾼다. 예: `omega_lab`. |
| ROS 도메인 ID | `0`~`232`의 정수 | 같은 연구실의 다른 graph와 분리할 때 바꾼다. 보안 수단은 아니다. |
| RMW 구현 | `rmw_cyclonedds_cpp` | 특별한 호환성 요구가 없으면 기본값 유지. |
| 탐색 방식 | `multicast` 또는 `static` | 같은 L2 LAN은 multicast, Tailscale/routed VPN은 static 권장. |
| 보안 프로필 | `sros2` 또는 `trusted-network` | 공유망이면 `sros2`; 소유한 폐쇄 LAN/VPN에서만 trusted. |

`trusted-network`는 DDS 암호화·인증이 없는 평문 모드다. “Tailscale 안에
있다”는 사실만으로 모든 사용자가 신뢰된다는 뜻은 아니므로, 공유 tailnet이나
공용 컴퓨터에서는 `sros2`를 쓴다.

### 5.2 COM 카드와 deployment lane

#### 사용 안 함

COM 카드의 `사용 안 함`을 체크하면 카드가 어두워지고 topology에서 제외된다.
COM3이 처음 비활성화되어 있어도 정상이다. 필요한 경우 체크를 해제해 활성화한다.

- `simulation-only`: COM을 1~3개 활성화하고 Robot 블록은 숨긴다.
- `full`: COM을 2~4개 활성화하고 네 역할을 카드 사이에 배치한다.
- 활성 카드에는 `Container runtime unit`과 `Robot native unit` lane이 있다.
  Robot은 Jetson으로 표시한 카드의 native lane에만 놓을 수 있으며, Jetson
  카드에는 Robot unit이 필수다. 같은 카드의 Pilot/UI는 별도 Compose unit으로
  유지된다.
- 역할이 없는 카드는 활성화하지 않는다. 활성 카드마다 적어도 하나의 역할이
  있어야 저장할 수 있다.

#### 안정적인 호스트 ID

`com1`, `compute_a`처럼 **소문자로 시작하는 고정 식별자**를 쓴다. 화면에
보이는 IP나 컴퓨터 이름 대신, topology 파일이 같은 컴퓨터를 다시 알아보는
용도다. 공백·대문자·슬래시는 쓰지 않는다. 기본값 `com1`, `com2`, `com3`,
`com4`를 그대로 사용해도 된다.

#### 표시 이름

사람이 알아보기 위한 이름이다. `노트북`, `서버컴`, `Jetson 지하실`처럼 자유롭게
지어도 되지만 활성 host끼리는 겹치지 않게 한다. 통신에는 쓰이지 않는다.

#### 이 컴퓨터가 연결 관리자를 실행함

정확히 한 카드만 선택한다. 연결관리자를 실행한 컴퓨터를 뜻한다. 이 선택은
“원격 컴퓨터에서 SSH를 열어 주는 서버 역할”을 부여하는 것이 아니다. 단지
관리 작업을 시작할 기준점을 정하는 것이다.

#### DDS 광고 주소

다른 host가 실제로 도달할 수 있는 **IP 또는 hostname만** 입력한다.

```text
좋음: 100.109.151.37
좋음: compute.example.internal
나쁨: 100.109.151.37:8080
나쁨: http://100.109.151.37
나쁨: 127.0.0.1
```

Native host network에서 Tailscale을 쓰면 host의 현재 Tailscale IP 또는
tailnet hostname을 사용한다. Docker Desktop sidecar에서는 WSL의 주소가 아니라
`elesim-tailscale status`의 sidecar IP를 사용한다. 연결관리자가 runtime
namespace의 `tailscale0` 주소를 자동 제안할 수 있지만, 이는 읽기 전용
힌트이므로 저장 전에 현재 값을 확인한다. 주소가 바뀌면 topology에서 다시
저장해야 한다.

이 입력란은 DDS discovery와 static peer 생성에 사용된다. 애플리케이션 포트를
붙이지 않는다.

#### DDS 바인드 인터페이스

DDS가 실제로 사용할 NIC 이름 하나를 쓴다.

```text
Tailscale (native/sidecar runtime): tailscale0
일반 LAN:  eth0, eno1, wlan0 등 실제 이름
VPN:      wg0 등 실제 이름
```

`/home/...`, `http://...`, `tailscale0:22`처럼 경로나 포트를 넣지 않는다.
모든 host의 인터페이스 이름이 같을 필요는 없지만, 각 host에서 선택한 인터페이스가
다른 peer의 광고 주소로 양방향 도달 가능해야 한다.

#### 설치 루트

그 host에서 EleSim이 실제로 설치된 **절대경로**를 입력한다.

```text
/home/user/ws/five
/home/hckang/continuum_app/six
```

로컬 카드에서는 연결관리자를 실행한 설치 prefix와 정확히 같아야 한다. 예를
들어 `/home/user/.local/share/elesim`에 설치된 연결관리자로
`/home/user/ws/five`를 로컬 root라고 쓰면 거부된다. 원격 카드에서는 원격
컴퓨터의 경로를 입력하며, 노트북 경로를 복사하지 않는다.

#### 명령 디렉터리

대부분 `<설치 루트>/bin`이다.

```text
/home/user/ws/five/bin
/home/hckang/continuum_app/six/bin
```

`~/...`, 상대경로, `..`, `bin_dir: /usr/local/bin`처럼 설치 root와 무관한
값을 피한다. 반드시 POSIX 절대경로로 쓴다. 관리자가 원격에서 실행할
`elesim-up`, `elesim-net` 등의 위치다.

#### 역할 블록과 Endpoint ID

Pilot·Sim·UI 블록을 활성 COM 카드로 드래그한다. `full`에서 Robot 블록도
활성 카드 사이에서 드래그할 수 있지만, 대상 카드는 Jetson으로 표시되어야
하고 Robot은 native/systemd lane에만 놓을 수 있다. 같은 카드의 Pilot/UI는
별도 Compose unit으로 저장된다. 블록 안의 Endpoint ID는 논리 주소이며 IP가
아니다.
보통 다음 기본값을 그대로 둔다.

| 역할 | 기본 Endpoint ID |
| --- | --- |
| Pilot | `pilot-main` |
| Sim | `sim-default` |
| UI | `ui-main` |
| Robot | `robot-go2` |

Endpoint ID를 바꾸려면 소문자·숫자·`-`·`_`만 사용하고, 같은 topology에서
중복하지 않는다. 프로그램이 어느 Python 메서드에 직접 접근하는 것이 아니라
이 논리 endpoint에 명령을 보내므로, IP가 바뀌어도 ID는 안정적으로 유지한다.

### 5.3 설치용 SSH

`설치용 SSH`는 원격 파일 전송, host 상태 확인, SROS2 bundle 배포와 lifecycle
명령을 위한 관리 경로다. DDS나 WebRTC 런타임 연결을 만들지 않는다.

로컬 카드에서는 이 영역을 **통째로 비워 둔다**. 원격 카드에서는 다음을 입력한다.

| 입력란 | 필수 여부 | 입력 방법 |
| --- | --- | --- |
| SSH 호스트 | 원격 host에서 필수 | 관리 대상 WSL/host의 IP/hostname만. `:22`를 붙이지 않는다. DDS sidecar IP와 달라도 된다. |
| SSH 포트 | 원격 host에서 필수 | Tailscale SSH면 자동 `22`; 일반 OpenSSH면 실제 sshd 포트. |
| SSH 사용자 | 원격 host에서 필수 | 원격 Linux 계정. 예: `hckang`, `user`. |
| Tailscale SSH 사용 | 선택 | Tailscale SSH를 쓸 때 체크. 개인키 칸은 비운다. |
| 개인키 파일 경로 | OpenSSH에서 선택 | 키 **내용이 아닌 경로**. 비우면 SSH agent 사용. |
| 고정된 호스트키 SHA-256 | 저장/배포 원격 host에서 필수 | `SSH 호스트키 확인`으로 얻고, 독립적으로 확인한 뒤 고정. |

#### Tailscale SSH를 사용하는 경우

1. 원격 컴퓨터와 조작 컴퓨터 모두 Tailscale 로그인 상태인지 확인한다.
2. `Tailscale SSH 사용`을 체크한다.
3. 포트는 22로 고정되므로 입력하지 않는다.
4. 개인키 경로는 비워 둔다. `~/.ssh/id_ed25519`를 넣으면 오히려 검증 오류다.
5. SSH 사용자는 원격 Linux 계정을 입력한다. Tailscale 계정 이메일을 쓰는
   칸이 아니다.
6. ACL에 `action: check`가 있으면 조작 컴퓨터의 터미널에서 한 번 대화형
   접속을 승인한다.

```bash
tailscale ssh <원격리눅스사용자>@<원격tailscale호스트> true
```

재인증/승인 화면을 끝낸 뒤 연결관리자로 돌아와 host key를 확인한다. Tailscale
SSH도 호스트키 fingerprint pinning은 필요하다.

`elesim-tailscale login`은 DDS runtime sidecar를 tailnet에 등록하는 명령이고,
Tailscale SSH는 연결관리자가 WSL/호스트에서 관리 명령을 실행하는 경로다. 둘은
같은 기능이 아니다. Sidecar 구성에서는 DDS IP에는 sidecar 주소를, SSH
호스트에는 WSL/호스트 주소를 입력한다.

#### 일반 OpenSSH를 사용하는 경우

- 실제 SSH 포트를 입력한다. 서버가 `2222`에서 sshd를 듣고 있다면 `2222`를
  입력한다. Tailscale의 기본값이라고 생각해 임의로 22를 넣지 않는다.
- SSH agent를 사용한다면 개인키 칸을 비운다.
- agent를 사용하지 않으면 조작 컴퓨터에서 읽을 수 있는 개인키 파일 경로를
  입력한다. `~/.ssh/id_ed25519` 또는 `/home/user/.ssh/id_ed25519`처럼
  **경로만** 입력한다.
- 파일 내용을 붙여 넣거나 `turn.secret`이 들어 있는 디렉터리를 입력하지
  않는다. 키 파일은 일반 파일이어야 하며 권한은 보통 `0600`이어야 한다.

#### SSH 호스트키 fingerprint

`SSH 호스트키 확인`을 누르면 관리자가 실제 SSH 호스트키의 SHA-256 값을
읽어온다. 화면의 fingerprint가 맞는지 별도 신뢰 경로(관리자가 알려 준 값,
직접 확인한 콘솔 등)로 확인한 뒤 저장한다. 이 값은 원격 host를 인증하는
값이지 DDS 암호화 키나 SROS2 개인키가 아니다.

### 5.4 Sim의 ICE와 Coturn

Sim은 direct ICE를 먼저 시도한다. `trusted-network`에서는 TURN URL과
credential source를 비우고 Coturn을 시작하지 않는다. `sros2`에서는 설치된
Sim Compose에 Coturn relay fallback이 함께 들어가며, Sim만 static HMAC
secret을 읽어 짧은 session credential을 발급한다. WebRTC media는 두 경우
모두 DTLS/SRTP이고, Coturn은 DDS discovery·topic·signaling을 중계하지 않는다.

Coturn은 Sim이 소유하는 내부 서비스라 연결관리자 COM 카드의 별도 영역이나
저장 topology field가 아니다. 연결관리자는 Sim 호스트의 `elesim-net show`에서
TURN URL, realm, public host, secret path 같은 비밀이 아닌 값만 읽고 SROS2
transaction에 반영한다. 사용자는 TURN secret 내용을 입력하거나 복사하지 않는다.

---

## 6. 저장·점검·배포 버튼 순서

### 6.1 Jetson 없이 처음 구성하기

1. `simulation-only`를 선택한다.
2. 필요한 COM만 활성화한다. 두 컴퓨터면 COM1·COM2만 남기고 COM3은
   `사용 안 함`으로 둔다.
3. Pilot·UI·Sim을 COM 카드에 정확히 한 번씩 배치한다.
4. 공통 DDS 값과 각 카드의 주소/interface/root/bin을 입력한다.
5. 로컬 카드 하나를 선택하고, 원격 카드의 SSH를 채운다.
6. 각 원격 카드에서 `SSH 호스트키 확인`을 눌러 fingerprint를 채운다.
7. `검증 후 저장`을 누른다. fingerprint를 새로 채웠다면 다시 저장한다.
8. Docker Desktop sidecar를 사용하는 경우 설치 마법사가 마지막에 보여 준
   `elesim-tailscale login`을 해당 host에서 먼저 실행하고 브라우저/device 승인을
   완료한다.
9. 보안 profile이 `sros2`이면 `보안 및 실행 준비`를 누른다.
   첫 실행은 키 생성·검증으로, 이미 세대가 있으면 새 세대 재발급·검증으로
   자동 분기한다.
10. `전체 시작`을 누른다. 또는 각 host에서 `elesim-up`을 실행한다.

정상 흐름은 저장 후 필요한 sidecar 로그인을 완료하고 보안 자료 생성·재발급 및 실행 준비를 수행한 뒤 시작하는 흐름이다. 오류가 나면 `Abort`로 현재 작업을 안전한 경계에서
취소하고, 유지보수 영역의 `호스트 점검`으로 저장된 모든 host의 namespace,
설치/SSH 관리 경로와 Compose/systemd 상태를 한 번에 확인한다. 기존의
`두 호스트 점검`은 자동화/API 호환용으로만 남아 있으며 일반 GUI 흐름에는
노출하지 않는다.

초기 보안 배포는 키와 설정만 맞추며, 원래 꺼져 있던 역할을 켜지 않는다.
`전체 시작`은 모든 host의 image build를 먼저 끝낸 뒤 역할을 시작한다. 중단된
managed SROS2 작업의 복구는 내부 안전장치로 유지되며, 일반 작업자는 먼저
`Abort`와 `호스트 점검`으로 상태를 확인한 뒤 재시도한다.

### 6.2 `trusted-network`를 선택한 경우

`trusted-network`에서는 SROS2 generation을 만들지 않는다.

1. topology를 저장한다.
2. `초기 배포 (보안 포함)`을 눌러 공통 DDS 설정을 각 host에 적용한다.
3. `전체 시작`을 누른다.

이 모드는 평문 DDS이므로 소유한 LAN 또는 방화벽으로 제한한 routed VPN에서만
사용한다.

### 6.3 이미 SROS2 generation이 있는 경우

`보안 자료 생성·재발급 및 실행 준비`를 다시 누르면 기존 세대를 자동으로 교체한다.
이 작업은 모든 host를 사전 점검하고 새 세대를 원자적으로 활성화한다. 한 host가
실패하면 이전 세대로 되돌리는 것을 전제로 한다.

### 6.4 유지보수·실행 버튼의 의미

| 버튼 | 하는 일 | 하지 않는 일 |
| --- | --- | --- |
| 보안 자료 생성·재발급 및 실행 준비 | 첫 실행은 생성, 이후에는 모든 host의 managed SROS2 generation 원자 교체 | 런타임 시작 또는 키 본문 입력 |
| 호스트 점검 | namespace, 설치/SSH 관리 경로, Compose/systemd 상태를 읽기 전용 확인 | DDS discovery/WebRTC 성공 판정 |
| 전체 정지 | 모든 host에서 현재 활성인 역할을 중단 | 이미지 삭제, 보안 세대 롤백, 재시작 |
| 전체 시작 | 모든 host image 준비 후 각 lifecycle 시작 | NAT를 뚫거나 Tailscale 설치 |
| Abort | 실행 중인 작업을 안전한 경계에서 취소 | 이미 commit된 세대 되돌리기 |

---

## 7. 실제 입력 예시

아래는 “Docker Desktop 노트북에서 연결관리자를 실행하고, native Docker
서버컴에서 Sim을 실행하는 simulation-only” 예시다. IP는 서로 다른 node의
예시일 뿐이며 실제 `status` 결과로 바꿔야 한다.

### 공통

```text
토폴로지 모드: simulation-only
시스템 ID: elesim
ROS 도메인 ID: 0
RMW: rmw_cyclonedds_cpp
탐색: static
보안: sros2
```

### COM1 — 노트북, 로컬 Pilot/UI

```text
안정적인 호스트 ID: com1
표시 이름: 노트북
로컬: 선택
DDS 광고 주소: 100.90.10.11         # 노트북 sidecar의 예시 IP
DDS 인터페이스: tailscale0
설치 루트: /home/user/ws/five
명령 디렉터리: /home/user/ws/five/bin
역할: Pilot (pilot-main), UI (ui-main)
설치용 SSH: 전부 생략
```

### COM2 — 서버컴, 원격 Sim

```text
안정적인 호스트 ID: com2
표시 이름: 서버컴
로컬: 해제
DDS 광고 주소: 100.74.222.24        # 서버 native Tailscale 예시 IP
DDS 인터페이스: tailscale0
설치 루트: /home/hckang/continuum_app/six
명령 디렉터리: /home/hckang/continuum_app/six/bin
역할: Sim (sim-default)
SSH 호스트: 100.74.222.24            # 이 native 예시에서는 DDS와 같음
SSH 사용자: hckang
Tailscale SSH: 선택
SSH 포트: Tailscale SSH면 22, 일반 OpenSSH면 실제 포트
개인키: Tailscale SSH면 빈 칸; OpenSSH agent면 빈 칸; 파일 사용 시 경로
호스트키: 확인 버튼으로 채운 SHA256 fingerprint
```

서버도 Docker Desktop sidecar라면 DDS 주소에는 그 sidecar IP를 쓰고 SSH
호스트에는 서버의 WSL/host IP를 별도로 쓴다. `100.74.222.24:8080`처럼 DDS
주소에 포트를 붙이지 않는다. 8080 HTTP 서버를
열어 둔 것은 사람이 경로를 시험하는 데만 쓸 수 있고, 연결관리자 입력값이 아니다.

---

## 8. Tailscale과 네트워크 점검

Tailscale은 각 runtime node를 같은 routed VPN에 넣어 주는 수단이다.
Native host network는 host의 Tailscale node를 사용한다. Docker Desktop은
WSL의 `tailscale0`를 상속하지 않으므로 생성된 kernel-mode sidecar가 별도 node가
된다. Sidecar는 EleSim role이나 Router가 아니라 그 컴퓨터의 container network
인프라다.

Native host network에서는 먼저 확인한다.

```bash
tailscale status
tailscale ip -4
ip link show tailscale0
```

Docker Desktop sidecar에서는 다음을 사용한다.

```bash
elesim-tailscale login       # 브라우저/device 등록 또는 stale node 재인증
elesim-tailscale status
```

로그인은 브라우저/device 승인 방식이며 auth/OAuth key를 EleSim 설정에 넣지
않는다. Sidecar state는 일반 종료와 업데이트에서 보존된다. DDS role,
runtime-network doctor와 활성 Sim-owned Coturn은 sidecar namespace를
공유하지만 SSH 명령은 별도 WSL/host 주소로 간다.

두 host가 서로 ping된다는 것은 좋은 첫 단계지만 DDS 성공을 보장하지 않는다.
DDS는 선택한 interface로 양방향 UDP가 통과해야 하고, 모든 participant가
system/domain/RMW/discovery/security 설정을 호환해야 한다. `SSH 호스트키 확인`
성공도 DDS를 증명하지 않는다.

Tailscale에서 다음은 서로 다른 기능이다.

| 기능 | 용도 |
| --- | --- |
| Native/sidecar `tailscale0` 주소 | DDS runtime P2P 주소 |
| `elesim-tailscale login/status` | Docker Desktop sidecar node 등록/확인 |
| Tailscale SSH | 연결관리자의 원격 관리/배포 경로 |
| SSH `-L` 포워딩 | loopback 설치 GUI를 브라우저로 여는 경로 |
| TURN/Coturn | WebRTC 영상 media relay |

일반 IPv4 NAT·CGNAT·symmetric NAT를 DDS가 자동으로 통과한다고 가정하지
않는다. Tailscale graph는 `static` discovery만 사용하고, 각 static peer는
discovery seed일 뿐 DDS relay가 아니다. 서로 직접 라우팅되는 VPN이나 검증된
LAN/IPv6가 필요하다.

---

## 9. 자주 보는 오류

### `managed SROS2 role bundle이 아직 provision되지 않았습니다`

정상적인 안전 차단이다. 조작 컴퓨터에서 `elesim-connections`를 열고 topology를
저장한 뒤 `보안 자료 생성·재발급 및 실행 준비`를 성공시킨다. 그 다음에 `elesim-up`을
실행한다.

### `topology is not saved`

`검증 후 저장`을 먼저 누른다. 저장된 topology가 있어야 배포·시작 작업이
동일한 입력을 사용한다.

### `install_root가 ... prefix와 다릅니다`

로컬 카드의 설치 루트를 연결관리자 자신이 설치된 prefix와 똑같이 맞춘다.
연결관리자를 `/home/user/ws/five`에서 실행했다면 local root도 그 값이어야
한다. 다른 설치 위치의 `elesim-connections`를 섞어 실행하지 않는다.

### `bin_dir must be a contained absolute POSIX path`

`/home/user/ws/five/bin`처럼 절대경로를 입력한다. `bin`, `~/...`, `../bin`,
`/`는 쓰지 않는다.

### `Authentication failed`, `EOF`, `timed out`, `connection refused`

1. 원격 host의 Tailscale online 상태와 현재 IP를 확인한다.
2. Tailscale SSH라면 원격 Linux 사용자와 ACL을 확인하고, `action: check`이면
   `tailscale ssh <user>@<host> true`를 한 번 승인한다.
3. 일반 OpenSSH라면 실제 sshd 포트와 agent/개인키를 확인한다.
4. `SSH 호스트키 확인`은 SSH 입력란의 관리 주소를 사용한다. Sidecar DDS IP를
   WSL/host SSH 주소 대신 넣지 않는다.
5. SSH가 성공해도 DDS가 자동으로 성공하는 것은 아니므로 interface와 양방향
   UDP 경로를 별도로 확인한다.

### `tailscale0`이 runtime namespace에 없거나 DDS 주소가 할당되지 않았음

- Native host network이면 현재 Docker Engine의 host namespace에 선택한
  인터페이스가 실제로 있는지 확인한다.
- Docker Desktop이면 설치가 `tailscale-sidecar`로 고정되었는지 확인하고
  `elesim-tailscale login`, `elesim-tailscale status`를 실행한다.
- Sidecar `status`의 IP를 DDS 주소에, WSL/host 주소를 SSH 입력란에 저장한다.
- routed Tailscale에서 탐색은 `static`이어야 한다.

이 검사를 억지로 우회하거나 SSH helper를 DDS relay처럼 사용하지 않는다.
namespace/interface/address/route 검사가 성공해도 실제 두 호스트 DDS 성공은
아니므로 아래 수동 gate를 계속 수행한다.

### `Address already in use` 또는 `elesim-manager` 이름 충돌

이전 연결관리자 또는 설치 GUI가 남아 있는 경우다. 기존 브라우저와 실행 중인
연결관리자를 닫고, 활성 작업이 없는지 확인한 뒤 다시 실행한다. 정확한 transient
container만 확인하며 전역 Docker 리소스를 지우지 않는다. 계속 필요하면 새
loopback 포트를 사용한다.

```bash
elesim-connections --port 8771
```

예전에 출력된 URL/token은 프로세스가 끝난 뒤 사용할 수 없다. 새 실행에서
터미널에 출력된 URL을 사용한다.

설치 완료 화면에서 stale manager 정리 명령을 복사할 수도 있다. 이 명령은
중지된 manager만 지우고 다른 실행 중 연결관리자는 건드리지 않는다. PATH 등록을
선택했다면 같은 화면에서 `source ~/.bashrc`도 복사해 현재 shell에 반영한다.

### `기존 unpinned EleSim 설치가 현재 Docker daemon에 속한다는 증거가 없습니다`

v1-v8 설치를 처음 업데이트할 때 현재 daemon에 같은 install UUID와 Compose
label을 가진 기존 container 또는 local image가 하나 이상 있어야 한다. 원래
설치에 사용한 Docker context를 선택해 다시 실행한다. 한 번도 build하지 않아
증거가 없는 legacy 설치라면 해당 설치의 검증된 clean uninstall 후 새로
설치한다. 빈 daemon을 임의로 기존 설치 소유자로 채택하지 않는다.

### `설치 시 고정한 Docker Engine과 현재 daemon이 다릅니다`

다른 Docker context를 선택했거나 Docker Desktop/Engine reset으로 Engine ID가
바뀐 상태다. EleSim은 자동으로 다른 daemon에 소유권을 옮기지 않는다. 원래
daemon/context를 복원해 정상 uninstall하거나, 기존 prefix를 건드리지 않고 새
빈 prefix에 재설치한 뒤 별도 audited cleanup을 진행한다. state/ownership JSON을
손으로 고치거나 Docker prune으로 우회하지 않는다.

### `연결관리자에서 호스트 상태는 되지만 DDS가 안 보임`

호스트 상태는 SSH/Compose/systemd 관리 상태일 뿐이다. 다음을 확인한다.

- 모든 host의 system ID, ROS domain ID, RMW가 같은가.
- DDS interface가 실제 Tailscale/LAN interface인가.
- routed VPN에서 discovery를 `static`으로 두고 주소를 정확히 입력했는가.
- `trusted-network`를 선택했다면 방화벽이 선택 interface의 UDP를 허용하는가.
- `sros2`라면 모든 역할의 같은 generation bundle이 활성화됐는가.

---

## 10. 평소 실행과 종료

설치와 연결 배포가 끝난 뒤에는 역할을 소유한 host에서 다음을 사용한다.

```bash
elesim-up
elesim-logs
elesim-net doctor
elesim-net doctor --active
elesim-down
```

`elesim-logs`에서 `Ctrl+C`는 로그 follow만 멈추며 서비스를 정지하지 않는다.
서비스를 끌 때는 `elesim-down`을 사용한다. 연결관리자 컨테이너까지 명시적으로
종료하려면 `elesim-down --purge`를 사용한다. 연결관리자의 `전체 시작/정지`는
여러 host를 한 번에 조작하는 편의 기능이고, 개별 host에서 직접 명령을 실행해도
된다.

### 완전 제거

설치한 **각 host의 정확한 prefix**에서 plan을 먼저 확인한다.

```bash
elesim-uninstall --plan
elesim-uninstall
```

로그와 조작 컴퓨터의 이 설치 소유 SROS2 Authority도 기본 삭제된다. 남길 때만
`--keep-logs`, `--keep-authority`를 추가한다. 외부 source/credential/keystore는
항상 보존한다. `docker system prune`이나 wildcard 삭제는 사용하지 않는다.
Docker Desktop sidecar 설치에서는 일반 down/update가
`<prefix>/secrets/tailscale` login state를 보존하지만, 검증된 완전 제거는 이
설치가 소유한 exact state directory도 제거한다. 공용 upstream Tailscale
image나 다른 프로젝트의 data는 제거하지 않는다. 로컬 state 삭제는 tailnet
control plane의 device record를 revoke하지 않으므로 폐기할 node는 Tailscale
관리자 콘솔에서도 별도로 제거한다.

---

## 11. 마지막 확인표

처음 실행하기 전에 아래만 체크하면 된다.

- [ ] Jetson이 없으면 `simulation-only`를 선택했다.
- [ ] 각 역할이 정확히 한 번씩 배치됐다.
- [ ] 로컬 host는 정확히 하나이고, 로컬 SSH 칸은 비어 있다.
- [ ] DDS 주소에는 IP/hostname만 있고 `:포트`가 없다.
- [ ] Tailscale이면 DDS interface가 `tailscale0`이다.
- [ ] Docker Desktop sidecar이면 `elesim-tailscale status`가 `Running`과 IPv4를 보여 주고 그 IP를 DDS 주소로 썼다.
- [ ] 모든 host의 system ID, domain, RMW, discovery, security가 호환된다.
- [ ] Tailscale/routed VPN이면 discovery가 `static`이고 모든 활성 DDS 주소가 peer로 파생된다.
- [ ] 각 host의 install root/bin이 그 host에 실제 존재하는 절대경로다.
- [ ] 원격 SSH 주소/user/port가 실제 관리 경로와 일치한다. Sidecar이면 SSH IP와 DDS IP를 구분했다.
- [ ] 원격 host의 fingerprint를 확인하고 저장했다.
- [ ] Tailscale SSH를 쓰면 개인키 칸을 비웠고 포트 22를 사용한다.
- [ ] OpenSSH를 쓰면 agent 또는 개인키 파일 경로를 준비했다.
- [ ] `sros2`면 보안 자료 생성·재발급 및 실행 준비를 `elesim-up`보다 먼저 실행했다.
- [ ] host 상태 확인 성공을 DDS/WebRTC 성공으로 오해하지 않았다.

---

## 12. 현재 문서의 범위

이 안내서는 설치기와 연결관리자의 입력·순서를 설명한다. 다음은 실제 장비와
네트워크에서 별도로 확인해야 한다.

- 다중 host의 실제 DDS discovery/control/RGBD 왕복.
- Docker Desktop sidecar와 두 번째 실제 host 사이의 static-peer discovery,
  양방향 DDS, reconnect.
- SROS2 enforce 권한이 허가·거부하는지.
- NAT/CGNAT 환경의 실패 진단과 실제 Coturn relay candidate.
- 실제 WebRTC 두 스트림의 영상 수신.
- NVIDIA/WSLg 렌더링과 Jetson/Unitree 물리 안전 동작.

따라서 `SSH 호스트키 확인`, ping, `python3 -m http.server 8080` 중 하나가
성공했다고 해서 EleSim 전체가 연결된 것으로 간주하지 않는다.
