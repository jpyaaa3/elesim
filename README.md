<p align="center"><img src="./assets/branding/icon.png" width="180" alt="EleSim logo"></p>

<h1 align="center">EleSim</h1>

<p align="center">https://doi.org/10.1109/LRA.2026.3663818</p>

<br>

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

[처음부터 끝까지](docs/zero2omega.md) ·
[설치](docs/setup.md) ·
[배포와 운영](docs/deployment.md) ·
[구성](docs/configuration.md) ·
[아키텍처](docs/architecture.md) ·
[코드 지도](docs/code_map.md) ·
[분산 런타임](docs/distributed_runtime.md) ·
[DDS 계약](docs/dds_contracts.md) ·
[실험 프레임워크](docs/experiment_framework.md) ·
[Jetson 혼합 역할](docs/jetson_mixed_role_rollout.md) ·
[연결 관리자 설계](docs/design/connection_manager.md) ·
[Preview/Gaze 설계](docs/design/preview_gaze.md) ·
[Gait phase preview](docs/design/gait_phase_preview.md) ·
[설치·네트워크·보안](docs/setup.md#dds-network-and-security) ·
[마일스톤](docs/MILESTONES.md) ·
[미해결 항목](docs/OPEN_ISSUES_KR.md) ·
[Open issues](docs/OPEN_ISSUES.md)
