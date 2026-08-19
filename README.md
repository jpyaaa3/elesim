<p align="center"><img src="./assets/branding/icon.png" width="180" alt="EleSim logo"></p>

<h1 align="center">EleSim</h1>

<p align="center">https://doi.org/10.1109/LRA.2026.3663818</p>

<br>

## 1. 설치 방법

### 🚀 설치 마법사 실행

아래 명령어를 실행합니다. (아직 merge 안 했으므로 아래 명령어 쓰십쇼)

   ```bash
   curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/installer/bootstrap/install.sh | bash
   ```
   ```bash
   curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/refactoring/installer/bootstrap/install.sh \
    | ELESIM_REF=refactoring bash
   ```

### 🐳 Docker Desktop을 사용 중일 경우

Docker Desktop 사용 중 발급된 Tailscale IP는 DDS 통신을 위한 인터페이스 사용이 제한됩니다.

EleSim의 설치가 끝나면 아래 명령어로 native Docker 기반 Tailscale IP를 생성하십시오.

SSH는 Docker Desktop 사용 여부와 무관합니다.

   ```bash
   elesim-tailscale login
   ```

<br>

## 2. 어플리케이션

| 역할 | 하는 일 |
| --- | --- |
| Pilot | 카메라를 보고 목표와 로봇 동작을 계산합니다. |
| UI | 사용자가 화면을 보고 명령을 내립니다. |
| Sim | Genesis에서 로봇과 카메라를 시뮬레이션합니다. |
| Robot | 실제 로봇을 움직이고 안전을 관리합니다. |

역할들은 중앙 Router 없이 ROS 2/DDS로 직접 통신합니다. 영상은 WebRTC를 사용합니다.

### 💻 자주 쓰는 명령

```bash
elesim-update       # 설치 파일과 이미지를 업데이트
elesim-up           # 이 컴퓨터의 역할을 시작
elesim-down         # 이 컴퓨터의 역할을 종료
elesim-down --purge  # 역할 종료 후 연결관리자(elesim-manager)도 제거
elesim-logs         # 실행 로그 확인
elesim-status       # 이 컴퓨터의 역할·IP·GPU·미디어 스펙 확인
elesim-connections  # 여러 컴퓨터의 연결·보안·실행 관리
elesim-uninstall --plan  # 삭제될 항목만 미리 확인
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
[Open issues](docs/OPEN_ISSUES.md) ·
[감사 기록](docs/audit/2026-07-20/README.md) ·
[감사 커버리지](docs/audit/2026-07-20/coverage.md)
