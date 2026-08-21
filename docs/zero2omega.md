# Zero2Omega 사용자 안내서 (요약)

이 파일은 예전 장문 설치 메모를 대체하는 사용자용 quick reference다. 현재
값·예외·보안 경계의 정본은 [`README.md`](README.md), 설치는
[`setup.md`](setup.md), 다중 호스트는 [`design/connection_manager.md`](design/connection_manager.md)다.

## 1. 먼저 결정할 것

1. General 또는 Developer인가?
2. General이면 Pilot/Sim/UI 중 어느 role을 이 prefix가 소유하는가?
3. Sim/Pilot GPU policy가 `inherit`, `specific`, `cpu` 중 무엇인가?
4. 소유 LAN/VPN의 `trusted-network`인가, 공유망의 `sros2`인가?
5. 여러 host면 `full`인가 `simulation-only`인가?
6. Sim은 headless인가, 이번 실행에 명시적 native Viewer가 필요한가?

Robot은 감지된 Jetson에서만 native-only다. Jetson이 없으면
`simulation-only`로 Pilot/Sim/UI 세 역할만 저장한다.

## 2. 설치

```bash
curl -fsSL \
  https://raw.githubusercontent.com/jpyaaa3/elesim/refactoring/installer/bootstrap/install.sh \
  | ELESIM_REF=refactoring bash
source ~/.bashrc
```

설치는 build/start를 하지 않는다. GUI가 끝난 뒤 prefix를 소유하는 host에서
`elesim-update` 또는 `elesim-up`을 실행한다. remote GUI는 SSH local-forward로
접근하고, forwarding port는 DDS/WebRTC endpoint가 아니다.

## 3. 권장 운영 순서

```bash
elesim-update       # source/owned artifacts 재생성 + incremental build
elesim-up           # image 적용 + runtime 시작
elesim-status       # IP, state, GPU, DDS, Sim media 요약
elesim-logs         # 로그 follow
elesim-down         # 중지 및 bounded archive
```

update는 running container를 교체하지 않는다. 새 image를 적용하려면
`elesim-up`; 의도적인 전체 restart는 `elesim-down` 후 `elesim-up`이다.

Developer는 다음처럼 persistent container를 쓴다.

```bash
elesim-up --jaeger
elesim-dev
```

## 4. 다중 호스트

operator laptop에서 `elesim-connections`를 실행한다. 각 카드에 DDS
address/interface와 별도 SSH address/port/user/fingerprint를 입력한다.

- `full`: 2–4 host, Pilot/Sim/UI/Robot, Robot native Jetson
- `simulation-only`: 1–3 host, Pilot/Sim/UI만, Robot 없음

순서는 `check/preflight → security provision/deploy/rotate → build → start`다.
GUI는 runtime 중간 재시작을 하지 않으며, host-owned deliberate restart는
각 prefix에서 `elesim-down && elesim-up --no-build`로 수행한다.

## 5. GPU와 Viewer

`specific`은 Compose `device_ids` 하나를 예약하고, `cpu`는 GPU reservation과
Genesis GPU backend를 끈다. 실제 container 값은 다음으로 확인한다.

```bash
docker exec elesim-sim sh -lc \
  'printf "CVD=%s\\n" "${CUDA_VISIBLE_DEVICES:-unset}"; \
   command -v nvidia-smi >/dev/null && nvidia-smi --query-gpu=index,uuid,name --format=csv,noheader || true'
```

Sim native Viewer는 기본 off/headless다. 실제 display owner를 명시해 한 번만
켠다.

```bash
DISPLAY=:0 CUDA_VISIBLE_DEVICES=0 elesim-up --view
```

연결 관리자는 topology SSH username 소유의 X11 session만 선택한다. observer와
hand-eye는 native window가 아니라 WebRTC track이므로 Viewer가 없어도 전송될
수 있다.

## 6. 흔한 오해

- `running` container ≠ DDS descriptor/heartbeat ≠ Sim session grant.
- SSH/HTTP 연결 성공 ≠ DDS UDP readiness.
- Coturn ≠ DDS relay. TURN은 WebRTC DTLS/SRTP media만 운반한다.
- `ROS_DOMAIN_ID` ≠ 인증.
- `elesim-update` ≠ restart.
- `--purge` ≠ source/image defect repair.
- observer/hand-eye 영상과 typed RGB-D는 서로 다른 경로다.

## 7. 제거

정확한 설치 prefix에서만 실행한다.

```bash
elesim-uninstall
```

manifest 검증 실패 시 uninstaller는 fail closed한다. Docker 전체 prune, home
전체 삭제, 다른 host prefix 삭제는 사용하지 않는다.
