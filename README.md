<p align="center"><img src="./assets/branding/icon.png" width="180" alt="EleSim logo"></p>

<h1 align="center">EleSim</h1>

<p align="center">https://doi.org/10.1109/LRA.2026.3663818</p>

<br>

배포 가능한 애플리케이션 소스는 [`payload/`](payload/README.md)에 있습니다.
개발 editable install, 설치 context 생성, release build가 모두 같은 tree를
사용하며 `dist/`는 생성된 release output 전용입니다.

## 1. 설치 방법

### 🧙 설치 마법사 실행

아래 명령어를 실행하십시오.

   ```bash
   curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/installer/bootstrap/install.sh | bash
   ```

### 🐳 Docker Desktop을 사용 중일 경우

Docker Desktop 사용 중 발급된 Tailscale IP는 DDS 통신을 위한 인터페이스 사용이 제한됩니다.

EleSim의 설치가 끝나면 아래 명령어로 native Docker 기반 Tailscale IP를 생성하십시오.

   ```bash
   elesim-tailscale login
   ```

<br>

## 2. 어플리케이션

| 역할 | 하는 일 |
| --- | --- |
| Pilot | 로봇의 동작 계산 |
| UI | 조작 패널 |
| Sim | Genesis 기반 시뮬레이션 |
| Robot | Jetson에서 로봇을 실제 조작 |

영상은 WebRTC로, 그 외 정보는 DDS로 공유됩니다.

### 💻 주요 명령어

```bash
elesim-up                  # EleSim을 시작

elesim-down                # EleSim을 종료
elesim-down --purge        # EleSim을 종료 및 연결 관리자(elesim-manager)도 제거

elesim-update              # EleSim을 업데이트

elesim-connections         # 각 컴퓨터 간 연결 관리 및 EleSim을 시작

elesim-logs                # EleSim의 로그 확인
elesim-status              # 이 컴퓨터의 EleSim 실행 정보 확인

elesim-tailscale login     # native Docker 기반으로 Tailscale에 로그인
elesim-tailscale update    # Tailscale을 업데이트

elesim-uninstall           # EleSim을 삭제
```

<br>

## 3. 문서

[설치](docs/setup.md) ·
[배포와 운영](docs/deployment.md) ·
[구성](docs/configuration.md) ·
[아키텍처](docs/architecture.md) ·
[DDS 계약](docs/dds_contracts.md) ·
[구현 상태](docs/status.md) ·
[연구와 진단](docs/research.md)
