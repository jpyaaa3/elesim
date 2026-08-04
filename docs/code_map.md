# 코드 지도와 자료 분류

이 저장소는 네 개의 독립 배포 프로그램과, 이를 개발·검증하기 위한 자료를
같이 둔다. 다음 경계를 기준으로 파일의 성격을 판단한다.

| 위치 | 성격 | 배포물 포함 여부 |
| --- | --- | --- |
| `pilot/`, `ui/`, `sim/`, `robot/`의 `src/`, `config/` | 네 런타임 프로그램 | 해당 역할만 포함 |
| `packages/protocol/`, `packages/elesim_interfaces/` | DDS 계약·공용 전송 기반 | 역할별 릴리스에 필요한 범위 포함 |
| `installer/package/`, `environment/`, `tools/release/` | 설치·배포 도구 | 설치/릴리스 산출물에 필요한 범위만 포함 |
| `tools/quality/`, `system_tests/` | 자동 검증 | 포함하지 않음 |
| `research/analysis/`, `research/experiments/`, `research/debug/` | 오프라인 분석, 재현 실험, 수동 디버그 | 포함하지 않음 |
| `research/results/` | 실험 결과·재현 근거 | 포함하지 않음 |
| `dist/`, `build/`, `*.egg-info/`, `__pycache__/`, `.pytest_cache/` | 생성 산출물·캐시 | Git 추적 대상 아님 |

## 배포 공통물: 고정 목록

"공통"은 저장소에서 같이 보인다는 뜻이 아니라, **네 역할의 독립 릴리스에
동일하게 복사해도 되는 런타임 계약물**이라는 뜻으로만 쓴다. 이 목록은 다음
둘로 고정한다.

| 공통 배포물 | 이유 | 소유자 |
| --- | --- | --- |
| `wheels/elesim_protocol-*.whl` | DDS 주소화, 세션·권한, payload 검증의 공통 구현 | `packages/protocol` |
| `interfaces/elesim_interfaces/` | ROSIDL 메시지·서비스·액션의 공통 wire contract | `packages/elesim_interfaces` |

각 역할 릴리스의 최상위 허용 목록은 아래와 같고, 릴리스 검증이 이 목록 밖의
파일을 거부한다.

| 역할 | 공통 항목 외 허용 항목 |
| --- | --- |
| Pilot | `Dockerfile`, Pilot wheel, Pilot `config/` |
| UI | `Dockerfile`, UI wheel, UI `config/` |
| Robot | Robot wheel, Robot `config/`, `install.sh`, `systemd/` |
| Sim | `Dockerfile`, Sim wheel, Sim `config/`, `model/bundles/default/` |

`requirements.lock`, `WHEELS.env`, `wheels/`, `config/`은 모든 릴리스에
존재하지만, 같은 파일이라는 뜻은 아니다. 각 역할이 자기 의존성과 설정을
소유한다. 특히 모델은 Sim 전용이며, 테스트·연구 자료·다른 역할의
wheel·소스·설정은 절대 공통 배포물이 아니다.

`dist/releases/infra/`는 네 역할 중 하나가 아니라 설치용 별도 인프라
릴리스다. 여기의 setup/컨테이너 자료를 애플리케이션 릴리스에 섞지 않는다.

`src/elesim_*` 형식은 Python의 표준 src-layout이다. 역할 이름을 Python
패키지 이름과 분리해, wheel을 설치했을 때 저장소 경로가 import 경로에
우연히 섞이지 않게 한다. 따라서 이 접두사는 제거 대상이 아니다.

`tests/regression/`은 과거의 단순 보관함이 아니라, 이미 발견된 장애와
안전 계약을 계속 검증하는 회귀 테스트다. 런타임에서 쓰이지 않는 재생·투영
보조 코드는 이 테스트 패키지 안에 둔다. 반대로 런타임 경로에서 실제로
호출되는 `pilot/experiment/walking_trial.py` 같은 코드는 이름과 달리
런타임 코드이므로 이 단계에서 옮기지 않는다.

YOLO/Torch와 RealSense는 현재 선택 가능한 perception 기능이다. 기본
개발 환경의 의존성 정책은 바꾸지 않았으며, 이를 별도 설치 프로필로 나누는
작업은 호환성·배포 검증을 포함하는 다음 단계의 독립 과제다.
