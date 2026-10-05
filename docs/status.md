# 구현 상태와 수용시험

갱신일: 2026-10-05. 이 문서만 마일스톤, 현재 완료 범위, 미해결 항목, 수동
acceptance gate를 소유한다. 구현 불변식은 `architecture.md`, wire 계약은
`dds_contracts.md`, 운영 절차는 `setup.md`와 `deployment.md`를 따른다.

## 현재 목표: 기존 기능의 운영 경로 완결

### Sim startup 캐시 비교 (2026-10-05, 진행)

- `add8e6d` 기반 상대 RTX A6000 GPU 0의 기존 Sim 이미지에서 별도 DDS 없는
  진단 프로세스로 같은 GO2+arm/Plane, dt=0.02, Newton 50 iterations를 비교했다.
  실행 중 Sim도 같은 GPU를 사용했으므로 전용 GPU 성능 수치는 아니다.
- profiler를 켠 static/dynamic build는 153.21/140.05초. static에서는
  `kernel.materialize` 누적 99.53초, 첫 physics step 120.39초가 관측됐다.
  누적 시간은 중첩되므로 합산하지 않으며 profiler 오버헤드도 포함한다.
- profiler 없는 별도 프로세스 재실행은 **static 62.26초, dynamic 5.08초**.
  step 100개(첫 10개 제외) 중앙값/p95는 각각 1.44/4.30ms, 1.89/5.04ms.
  양쪽 최종 qpos는 finite. 무제어 물리 probe이며 보행·MPC 안정성 증거가 아니다.
- 원인은 GPU `performance_mode=True` 강제로 선택된 static arrays의 반복
  kernel specialization 비용이다. 캐시 디렉터리 자체는 지속되고 있었다.
  physics 기본값을 dynamic으로 바꾸고 `simulation.runtime.genesis_performance_mode`
  명시적 GPU opt-in을 제공한다. 정밀도/충돌/solver 설정은 유지한다.
- 카메라 worker는 이미 dynamic arrays였다. 각 worker의 init/model/build 시간과
  별도 GO2 controller 초기화 시간을 추가해 전체 startup에서 구분한다.
- 재현: `workbench/research/debug/profile_sim_startup.py <robot.urdf>`를 설치된
  Sim/dev 환경에서 실행한다. `--performance-mode`는 static, `--profile`은
  비용 분석용이다. 캐시를 유지한 채 별도 프로세스로 반복한다.
- 원본: `workbench/evidence/generated/readiness/20261005-startup/`.
  이 축소 scene 결과를 실제 전체 Sim의 94초와 직접 비교하지 않는다.
  전체 application 배포 후 build/영상/step 검증은 아직 진행 중이다.
- 기존 생성 dev 환경의 Sim 전체 + quality: **496 passed / 3 skipped**.
  기본 dynamic 설정, static 명시 선택, 잘못된 설정 거부와 CPU 경계를 포함한다.
  fresh dev build나 MPC 보행 수용시험 결과는 아니다.

### 자율 점검 후속 (2026-10-05, 진행)

첫 변경 묶음 `e646290`과 후속 `8321765`, 설치 취소 보완 `3900305`는
원격 main에 반영돼 있다.
`8321765`에서 required/extended 전체와 네 역할 격리 release build/verify가
통과했다. 이후 설치 취소 경계를 보완했고 설치 전체 **1,138 passed / 3 skipped**,
model/release 도구 **82 passed**를 다시 확인했다. 새 설치 변경을 포함한
`release-install-progress`의 네 역할 격리 release 검사도 통과했다.
`3900305` 커밋 후 같은 산출물에 별도 `verify.py`를 실행해 네 역할 모두
다시 통과했으며 실행 중인 역할 컨테이너를 교체하지 않았다.

- release 검증의 별도 setup 모듈 허용 목록에 `install_transaction`이 누락돼
  실제 release build가 실패했다. 목록을 수정하고 실제 소스 목록과 대조하는
  회귀를 추가했다. 수정 전 전체 gate도 같은 문제로 model/release가 실패했다.
- Mock Hug의 identity/lifecycle 검사 두 개가 제한 시간 내 기하 해를 못 찾아
  전체 gate에서 실패했다. 이 세 수명주기 검사는 고정된 contact solver
  출력을 사용하도록 범위를 분리했다. 실제 기하 solver 검사는 유지한다.
  제품 solver의 시간 제한과 알고리즘은 변경하지 않았다.
- RL의 지연 관측이 reset 후 이전 episode 값을 전달하는 결함을 두 검사로
  재현했다. reset된 env 행만 다음 첫 관측으로 이력을 채우고 다른 env의
  지연 이력은 유지한다. 관측 차원은 동일하지만 새 학습의 reset 의미는
  수정되므로 기존 학습과 완전히 동일한 재현이라고 주장하지 않는다.
- RL 전용 scene에도 설정된 GO2 leg pose를 build 전에 검증·지정한다.
  runtime 기본 stand로 덮어쓰지 않으며 잘못된 shape/dtype/limit는 어떤
  joint도 수정하기 전에 거부한다. 후속 실제 CPU build에서 qpos0 관절 한계
  경고가 사라졌고 3회 step/reset을 재확인했다. 캐시가 있는 후속 build는
  33.0초였다. 초기 실행과 조건이 달라 코드 수정의 속도 개선율로 비교하지 않는다.
- SSH 경로 분리 진단: EleSim helper/ProxyCommand를 제거한 단순 전달에서도
  host TCP와 host `tailscale nc` 모두 client 1,288바이트 KEX 뒤 멈췄다.
  host route는 `tailscale0`, interface MTU 1,280이었다. TCP 소켓 하나만
  `TCP_MAXSEG=1024`(협상 후 1,012)로 만든 진단은 기본 알고리즘 그대로
  **0.104초**에 같은 공개 key를 반환했다. host VPN 경로의 패킷 크기 문제가
  유력하나 어느 계층이 packet을 잃는지는 확정하지 않는다. 전역 MTU/route/
  ACL과 제품 암호 설정은 변경하지 않았다. 삭제된 최초 오류와의 동일성도
  확정하지 않는다.
- 전용 happy prefix의 실제 wizard API 재설치도 완료했다. 새 shell의 generic
  `elesim-status`는 scoped 설치에 global runtime이 없다는 올바른 안내와
  exit 64를 반환했다. 등록된 system의 정상 상태 검증은 아직 아니다.
  dev에 Node가 없어 skip된 frontend 3개는 host Node에서 별도 **3 passed**다.

원본 로그와 전용 설치는
`workbench/evidence/generated/readiness/20261005-autonomous/`에 있다.
기존 소유 dev 서비스에서 실행했으며 새 dev image build, 실제 두 host의
로그인/영상, 물리 장비 검증으로 해석하지 않는다.

- **R1:** 실제 loopback WizardServer HTTP API에서 잘못된 입력 거부, 검증,
  파일 설치 완료를 확인했다. 처음 설치 중 취소하면 manifest 없는 생성물이
  남아 같은 입력으로 재시도할 수 없는 결함을 재현했다. 정확한 생성 경로가
  처음에 없었을 때만 rollback하고 기존/외부 파일은 보존하도록 수정했다.
  같은 출력 root의 동시 설치는 private advisory lock으로 직렬화한다.
  lock은 같은 UID·임시 디렉터리 namespace 안에서 유효하며 다른 컨테이너의
  별도 `/tmp`까지 직렬화한다고 주장하지 않는다.
- 실제 API 재실행에서 **취소 → 같은 입력 재시도 → 완료**를 확인했다.
  생성 host uninstaller로 해당 전용 prefix를 제거했고 tombstone도 evidence
  아래 남겼다. 삭제할 Docker container/image는 없었다. 사용 중인 설치,
  역할 프로세스와 외부 checkout은 변경하지 않았다.
- 기존 설치의 역할 변경 중 취소하면 Compose만 새 내용으로 남는 결함도
  재현했다. 실패 시 기존 wrapper·state·Compose·기존 role config를 복구하고,
  ownership manifest가 다른 writer에 의해 바뀌면 복구를 거부한다. 신규 생성
  build context와 role 파일까지 전체 snapshot으로 되돌리는 기능은 아니며
  같은 입력 재시도에서 다시 생성한다. cache/log/credential/release pin은
  복구 snapshot에 넣지 않는다. 실제 API에서 제어 파일 복구, 같은 입력
  재시도 완료, install UUID 유지까지 확인했다.
- 설치 commit callback을 명시해 manifest 발행 뒤의 늦은 취소가 완료를
  취소로 바꾸지 않게 했다. 실제 API에서도 commit 직후 취소 요청을 주입해
  완료 상태와 manifest가 일치함을 확인했다. 복합 설치는 다음 설치를 시작할
  때 다시 취소를 검사한다. native commit callback은 software 검사 범위다.
- 새 모듈의 bootstrap/release 배포 목록을 갱신했다. 설치 계획의 오래된
  generic `elesim-up` 안내는 연결관리자에서 host/role을 구성하는 안내로
  바꿨다. 브라우저 화면, bootstrap 전체, 실제 update 이미지 발행은 미실행이다.
- **R2/R3:** UI 명령 queue가 제출 당시 session ID를 보존한다. 검증 중 session
  변경, 전송 중 session 폐기, 전송 중 추가 입력을 회귀로 고정했다. 전송 중인
  명령도 queue 한도에 포함하고 폐기된 session의 tracking을 되살리지 않는다.
- 영상 decoder clock의 예외·NaN·무한대·음수·잘못된 문자열은 LIVE로 표시하지
  않고 해당 stream만 재시도한다. 정상 상대 stream과 DDS session은 유지한다.
- **R3/R4:** Pilot 제어 소유권 timeout은 monotonic clock을 사용하며 만료 뒤
  도착한 heartbeat가 소유권을 부활시키지 않는다. Pick 단계 wait가 끝난
  직후의 실패/취소도 재확인하고 wait 예외를 단계 실패로 보고한다. GO2 정지
  전송 또는 Gaze 정지가 실패해도 나머지 Pick 정지 절차는 모두 시도한다.
  이는 소프트웨어 회귀이며 실제 정지 시간 측정은 아니다.
- **구조 정리:** UI 명령 queue/영상 health와 RL 관측/episode 통계를 소유
  모듈로 분리했다. 기존 가독성 제한을 유지한 채 `UiSimSession` 978줄,
  `WrapGraspEnv` 992줄로 검사된다. RL public VecEnv·통계 계약은 유지한다.
  실제 CPU Genesis 1 env에서 3회 step/reset, 유한 관측/보상, policy 12 및
  privileged 53채널, 통계/metadata 생성을 확인했다. build는 약 208.5초였으며
  convex decomposition과 초기 kernel 준비를 포함한다. 학습 성공률이나
  GPU 성능 결과가 아니다. 후속 초기 자세 수정 결과는 위에 따로 기록했다.

| 현재 집중 검증 | 결과 |
| --- | --- |
| 설치 전체 (refresh/commit 경계 수정 후) | 1,138 passed, 3 skipped |
| UI session/명령/영상 health | 77 passed |
| Pilot heartbeat/중단/단계 흐름 | 406 passed, 21 skipped |
| Sim 관측/통계/초기 자세/reset | 469 passed, 3 skipped |
| quality 도구와 실제 runtime 가독성 | 22 passed |
| 실제 wizard API | 입력 오류, 신규/기존 설치 취소 후 재시도, commit 후 늦은 취소 표시 |
| 전용 prefix 생성 uninstaller | 완료; Docker 삭제 대상 없음 |
| 실제 CPU RL env | 3회 step/reset 완료; 학습/정책 성능은 미검증 |

R4의 후속 감사 대상은 기존 **Pick**으로 정했다. 지각·IK·Sim 결과를 포함하는
고정 장면 end-to-end 성공 기준과 실제 반복 수용은 아직 실행하지 않았다.
R5는 장비·현장 운영자가 없어 대기한다. 현재 R 단계의 수용 완료를 과거
M/B milestone이나 단위 검사 통과에서 추론하지 않는다. 두 호스트 진행에는
상대 SSH 사용자명·설치 경로와 host fingerprint를 사용자에게 확인받았다.
확인한 key를 고정한 실제 Tailscale SSH 인증과 아래 읽기 전용 점검이 완료됐다.

### 두 호스트 읽기 전용 점검 (2026-10-05)

후속 실행은 사용자가 명시적으로 승인했다. 양쪽 실제 updater가 비대화형
환경에서 `/dev/tty`의 존재만 보고 Docker TTY를 요청해 실패했다. 기존
`has_terminal` 검사로 통일한 `b191332`를 발행했으며 bootstrap 검사
**95 passed**다. host Python은 setup package가 없어 수집하지 못했고,
기존 설치 소유 dev에서 검사했다. 수정 후 양쪽 bootstrap이 통과했다.

로컬 UI release `ccfa92b7a95312d45c2ea66ab77ebc8a69c642b3ba123af95db0bdc6eb1cb92f`
발행이 완료됐다(source `b191332`, build 367.8초). 원격 Pilot/Sim 발행은
진행 중이다. sidecar는 기존 인증이 없어 NeedsLogin이었고 사용자가 브라우저
인증을 완료했다. 현재 DDS IPv4는 **100.127.177.101**로 이전 주소를 대체한다.
sidecar에서 상대 노드 Tailscale ping이 성공했다. 생성된 실제 연결 관리자
`readiness`의 두 호스트 HTTP preflight도 통과했고 SSH 지문이 확인된 값과
일치했다. 이는 DDS/영상 runtime 수용 완료가 아니다.

원격 GPU 1은 다른 작업이 사용 중이다. 후속 Robot-free 실행은 로컬 UI,
상대 Pilot/Sim, Sim GPU 0, Viewer 비활성으로 준비한다. 관리자는 loopback
18766에 열었으며 인증 token/브라우저 로그인 URL은 이 문서에 보관하지 않는다.

후속 원격 release
`62b13b17a41c481dcf9b52814e01cffd5218cab852edf1f7e5205bdcd6106643`도 발행됐다.
실제 관리자에서 `readiness` system을 저장하고 `prepare` → `start`를 실행해
양쪽 등록, 네트워크 검사, 세 역할 기동, 양쪽 DDS descriptor/heartbeat 검사가
완료됐다. domain 42, CycloneDDS static discovery, trusted-network,
양쪽 `tailscale0` 바인딩이며 Robot은 없다. 별도 image rebuild 없이 각 host의
동일 source revision release를 사용했다.

Sim은 GPU backend로 model load 6.08초, physics scene build 111.70초였다.
카메라 worker 준비 뒤 readiness gate가 열렸고 실제 UI에서 observer와
hand-eye 각각 640×480 frame 디코딩을 08:11:21 UTC에 확인했다. X11의 해당
두 EleSim 창만 캡처해 `video 2/2`, `OBSERVER LIVE`, `HAND-EYE LIVE`와
telemetry/Sim 시간 갱신도 확인했다. 실제 증거는
`workbench/evidence/generated/readiness/20261005-live/`다.

초기 kernel 준비 중 일시적인 peer lost/lease 재선택이 있었으므로 startup
전체가 무중단이었다고 주장하지 않는다. `qpos0 exceeds joint limits`는 해당
실행에서 보이지 않았다. 기존 다섯 neutral collision 쌍은 유지됐으며
watertight 경고는 physics scene 완료 후 카메라 모델 준비 때 다수 재현됐다.
X11 합성 버튼 입력은 동작 변화가 확인되지 않아 일시정지/step/reset 수용
증거로 사용하지 않는다. 08:11:21–08:21:55 UTC의 10분 이상 관찰에서는
08:20:11에 observer가 5초 decoder 정체를 감지했고 08:20:13에 자동 재협상과
frame 디코딩을 복구했다. hand-eye와 DDS session은 유지됐다. 두 LIVE 화면은
재확인했으나 무중단 통과는 아니다. 종료·재접속 반복은 진행 중이다.

복구된 observer의 `negotiation retry sent` 문구가 빨간 오류로 남는 실제
표시 결함을 수정했다. 해당 receiver에서 실제 fresh frame이 확인될 때만
그 stream의 정확한 재시도 안내를 지우고, SDP answer만 받은 상태·다른 stream·
명령 오류는 유지한다. focused session **31 passed**, UI+quality **101 passed**다.
이 수정은 `a964a48`로 발행했고 로컬 UI release
`d037de74f8c024876def6231630d761799e7f2cb26d5162df357e4a6c083e39e`에 포함됐다.
세 번째 회차에서 이 UI release를 명시적으로 등록했다. 원격 Pilot/Sim은
`b191332` release를 유지한다.

실제 두-host 시작과 두 영상 디코딩·LIVE 표시를 세 번 확인했다. 첫 두 회차는
정상 종료까지 완료했고, 세 번째 UI는 단일 Sim 장애 표시 검증을 위해 유지했다.
후속 Sim physics build는 94.13초, 94.50초여서 시작 지연이 해소된 것은 아니다.
Sim 하나를 정지하면 UI가 endpoint 상실과 `video 0/2`, 두 stream WAIT로
바뀌고 오래된 pixels를 LIVE로 유지하지 않음을 화면으로 확인했다.
관리자 전체 start는 실행 중 역할이 있으면 명시적으로 거부하는 설계다.
단일 peer 복구는 상대의 기존 `elesim-instance readiness up`으로 실행했고
Pilot Running/Sim Started 및 exit 0을 확인했다. 영상 재연결은 이어서 확인한다.

08:42:37–38 UTC에 열린 UI의 두 스트림이 다시 connected/frame decoded가
됐고, Sim 단일 중단 후 영상 복구를 확인했다. 사용자의 실제 카메라 창 버튼
입력으로 pause를 확인한 다음 protocol-v6 boot별 control topic을 읽기 전용
구독해 step/resume/reset을 관측했다. 관측 노드는 application endpoint나
권한을 발행하지 않았고 session/lease token을 기록하지 않았다.

| 실제 UI 입력 | 관측한 Sim 응답·상태 |
| --- | --- |
| pause | paused=true, sim_time_s=170.48; 두 영상 LIVE 유지 |
| step | ok=true; paused=true 유지, 170.48 → 170.50초 |
| resume | ok=true; paused=false, 시간 증가 재개 |
| reset | ok=true; epoch 0 → 1, sim_time_s=0.02부터 다시 증가; 두 영상 LIVE |

관측 노드에서 본 요청→결과 간격은 step 24.2ms, resume 52.5ms, reset 64.3ms다.
각 1회 표본이며 실제 버튼 입력 지연이나 정지 deadline 측정으로 해석하지
않는다. 원본은 `20261005-live/sim-controls-v6.jsonl`, 화면은 같은 디렉터리의
`camera-user-paused.png`, `camera-user-reset.png`다. 이는 UI→Sim session 명령
경로이며 Pilot motion/IK/Pick end-to-end 통과와 구분한다. 현재 readiness는
reset 후 running, UI와 원격 Pilot/Sim이 실행 중이다.
후속 관리자 `check`도 로컬 `running [ui]`, 상대 `running [pilot, sim]`으로
완료됐다(`20261005-live/final-check.json`). 전체 check는 약 149초였으며
runtime 시작 시간과 별도인 관리 점검 지연으로 남긴다. 임시 읽기 전용 DDS
관측 프로세스는 모두 종료했고 실제 readiness runtime과 sidecar는 유지했다.

자동 버튼 입력이 먹지 않은 원인은 WSLg가 자동 focus를 부여하지 않아 X11
focus=0으로 남은 것이었다. 사용자가 Sim Camera 창을 클릭한 후 해당 창의
실제 X11 focus를 확인했다. 이 과정은 제품 버튼 동작 수용 통과와 구분한다.

- `hckang@100.74.222.24:22`의 확인된 ED25519 지문은
  `SHA256:2oAaevxeuYi0yc3umwg15I5c/plBF8GW2x9TvfN7paY`다. 제품의
  `ParamikoConnector`와 명시적 `tailscale` 인증으로 연결했다. 전역 SSH trust
  파일은 변경하지 않았다.
- 상대 prefix `/home/hckang/ws/newsim`은 Pilot/Sim 설치, `direct-host`,
  `default` Docker context, install UUID `3fc3487e-0788-4a76-a152-e485144b0d0b`,
  project `elesim-merry`다. 고정된 Engine 확인과 생성 Compose 상태 조회가
  성공했다. Tailscale은 Running/online이며 IPv4가 `100.74.222.24`다.
- 상대 개발 checkout `/home/hckang/ws/newsim_dev/elesim`은 `d1044b8`이며
  working tree가 깨끗하다. checkout·설치·컨테이너를 업데이트하지 않았다.
- 로컬은 UI 설치, `tailscale-sidecar`, project `elesim-warm3`다. 양쪽
  Compose 조회에서 dev만 실행 중이고, 등록된 instance JSON과 저장된
  connection JSON은 없다. 로컬 sidecar도 실행 중이지 않다.
- 양쪽 발행 release의 source revision은 `bbdf390`으로, 이번 수정이 들어간
  checkout과 다르다. 따라서 현재 발행 image로 최신 수정의 수용을 주장하지
  않는다. SSH 관리 경로 성공은 DDS/영상 성공을 뜻하지 않는다.
- 다음 실제 실행은 양쪽 최신 release 발행, 로컬 sidecar 기동·주소 확인,
  로컬 UI + 상대 Pilot/Sim의 Robot-free system 등록·기동, DDS와 두 영상
  검증 순서다. 아직 image build/update, runtime 배포·시작은 하지 않았다.

전체 gate 원본은 `all-checks-followup.log`이며 protocol 141, Robot 104,
Pilot 406(+21 skip), Sim 469(+3 skip), UI 77, model/release 82, DDS RGBD 2,
encoded WebRTC 2, quality 22, analysis 10, debug 4, experiment 10이 통과했다.
실제 4프로세스 DDS smoke와 등록된 critical mutation 7개도 통과했다.
해당 실행의 setup은 1,129(+3 skip)이었고 이후 설치 수정의 1,138 결과는
`setup-progress-final.log`다. Node가 없는 dev의 frontend skip은 host의
3개 별도 통과로 보충하며 전체가 동일 환경에서 skip 없이 통과했다고 하지 않는다.

Genesis 1.4.1 / Quadrants 1.3.0 실제 초기화에서 geometry cache와 kernel
cache 모두 XDG root 아래 선택됨을 확인했다. 생성 Sim Compose의 persistent
cache mount와 일치한다. 쓰기 권한 문제로 private `/tmp` fallback을 택하면
container 재생성 뒤 캐시가 보존된다고 보장할 수 없다. 임의 캐시 삭제나
solver/collision 설정 완화는 적용하지 않았다.

### Genesis scene 경고 조사 (2026-10-05)

사용자가 제공한 Sim 로그의 네 경고를 현재 Genesis 1.4.1과 ZED Mini
bundle로 조사했다. 원본 진단은
`workbench/evidence/generated/readiness/20261005-scene/`에 있다.
설치된 runtime 이미지나 물리 장비는 변경하지 않았다.

| 로그 | 확인된 원인 / 처리 |
| --- | --- |
| neutral self-collision `(2, 28)` | 바닥을 먼저 추가한 현행 비병합 모델에서 `Head_upper` 원통과 `plate` 메시. 둘 다 base에 고정된 가지다. |
| `(8, 9)`, `(14, 15)`, `(20, 21)`, `(26, 27)` | FL/FR/RL/RR 각각 `calflower1` 원통과 `foot` 구체. 각각 같은 calf에 고정된 가지다. 움직이는 관절 충돌의 관측값이 아니라 qpos0에서 자동 제외된 geometry 쌍이다. |
| `qpos0 exceeds joint limits` | URDF의 초기 calf 값 0이 네 관절의 범위 `[-2.7227, -0.83776]` 밖이다. 기존 stand 설정은 scene.build 이후였다. 빌드 전에 12개 GO2 관절의 stand 값을 검증해 Genesis joint description에 지정하도록 수정했다. 실제 CPU build에서 해당 경고가 사라졌다. |
| `i32 <- i64` | Genesis `RigidSolver._init_tree_fields`의 `argmax(axis=1)` 결과인 `dofs_mass_envelope_start`를 i32 배열로 복사하는 경고. 진단에서 shape `(32,)`, 최솟값/최댓값 모두 0이었다. 실수 물리 상태의 정밀도 전환이 아니며 이 배열에서 값 손실은 없다. 외부 라이브러리 dtype 처리는 변경하지 않았다. |
| `Mesh is not watertight` | 현행 물리씬 build에서는 재현되지 않았다. 별도 mesh 검사에서 정점 seam을 합친 뒤에도 GO2 시각 DAE 7개와 plate OBJ가 watertight가 아니었다. plate는 열린 edge가 아니라 `(0,0,-0.015)`–`(0,0,-0.021)` m edge에 면 네 개가 연결된 non-manifold 형상이다. 어느 메시가 사용자 로그를 냈는지는 아직 확정하지 않는다. |

Genesis의 기본 필터는 일부 고정 가지 사이도 neutral overlap으로 처리한다.
이 다섯 쌍은 유효한 stand qpos0로 바꾼 뒤에도 동일하게 보고됐다. 충돌 전체를
끄거나 `enable_neutral_collision=True`로 중첩 형상 사이 접촉력을 강제하지
않았다. 실제 plate와 head의 물리적 간섭 여부는 단순화한 충돌 형상의 중첩과
구분해서 확인해야 한다. 메시를 convex hull로 영구 교체하거나 관성을 재추정하지
않았다.

첫 CPU build 프로파일은 모델 load 약 5.0초, build 171.1초였다. 그중
Quadrants kernel materialization 누적 약 90.7초로, Python 프로파일 비용이
포함된다. 기존 캐시를 사용하는 후속 비프로파일 build는 약 10.7초였다.
서로 조건이 달라 초기 자세 수정의 속도 개선 수치로 비교하지 않는다.
GPU와 사용자의 해당 설치에서 반복 시작 시간을 측정한 결과도 아니다.
runtime에 Genesis 초기화 / robot model load / scene build 각각의 시간을
flush해서 남기도록 추가했다. 카메라 replica에도 빌드 전 GO2 stand 준비를
적용했으며, 전체 renderer/GPU 경로는 아직 실행하지 않았다. 자동 테스트 suite는
이번 조사에서 실행하지 않았고, 실제 CPU 모델 로딩과 scene build로 진단했다.

### R0 초기 기준선과 SSH 경로 감사 이력 (2026-10-05)

기준 revision은 `bbdf390d56bb240c85a6ac8c8944f25d4086d6f6`이며 시작 시
working tree는 깨끗했다. 아래 과거 기록의 미커밋 변경·개발 wrapper 부재를
현재 상태로 사용하지 않는다. 새 마일스톤 체계를 추가하지 않고 R0 → R1 → R2를
진행한다. M1/M2와 B1–B5의 과거 software 완료는 R 수용시험을 대체하지 않는다.

#### 환경과 원래 장애의 증거

- 사용자는 상대 주소의 22번 포트 조회 실패를 보고했지만 오류 원문은
  삭제됐다. 이후 호스트 `100.109.151.37`, sidecar `100.82.19.33`, 상대
  `100.74.222.24`를 확인했다. 당시 GUI 인증 모드는 확정되지 않았다.
- 알려진 로컬 설치 `/home/user/ws/newsim`의 UUID는
  `cd824ccf-e543-4e4c-8cf4-8203623a9cef`, Compose project는 `elesim-warm3`,
  backend는 `tailscale-sidecar`다. 설치 source snapshot은 위 HEAD와 같다.
  `elesim-dev`, `elesim-connections`, `elesim-tailscale`의 파일 hash가 각각
  ownership manifest와 일치한다. 전체 설치 소유권 검증을 의미하지 않는다.
- 개발 attachment는 활성화돼 있으며 workspace는 이 checkout이다. 권한 제한
  밖에서 확인한 Docker context `default`의 Engine ID는 설치에 pin된
  `10a51a74-ee24-480d-be71-16ee52bdc55b`와 일치한다. 호스트 Tailscale은
  `Running`, self online이다. peer 6개 존재는 특정 peer의 SSH 접근 증거가 아니다.
- Sandbox 안에서는 Docker 접근과 Tailscale Unix socket 연결이 거부됐다.
  `tailscale status`의 "doesn't appear to be running" 출력만으로 daemon 중단을
  판단하면 안 된다. 직접 socket 접속의 `EPERM`과 권한 밖 상태를 대조했다.
- 상대는 현재 online이며 호스트 직접 TCP와 `tailscale nc` 양쪽에서
  `SSH-2.0-Tailscale` 배너를 받았다. 호스트 `ssh-keyscan`도 공개 host key를
  조회했다. sidecar node는 offline이고 알려진 설치 project에는 dev만 실행
  중이다. 이 결과는 사용자 인증이나 runtime DDS 연결 성공을 의미하지 않는다.
- 실제 SSH 사용자 인증, sidecar namespace, DDS, 두 영상은 아직 검증하지
  않았다. 실제 운영 role 배포나 물리 동작은 수행하지 않았다.

#### 연결 경로와 수정

```text
GUI /api/ssh/fingerprint
  → credentials.probe_ssh_fingerprint
  → open_ssh_connection: 직접 TCP 우선, 연결 실패 시 open_tailscale_proxy
  → host_proxy → private host_helper → 호스트 tailscale nc → 상대 SSH

설치 조회 / 배포 / lifecycle
  → ParamikoConnector → 동일한 proxy 선택
  → OpenSSH agent/명시적 key 또는 명시적 Tailscale SSH 인증

런타임 DDS
  → 별도 sidecar namespace/interface/address/route → 실제 descriptor/heartbeat
```

- 기존 일반 OpenSSH 접속은 지문 조회와 달리 host proxy를 사용하지 않았다.
  proxy가 필요한 Docker/WSL 환경에서는 지문 확인 뒤 설치 조회/접속이 실패할 수
  있었다. agent/명시적 key와 성공/실패 조합 4개가 수정 전 모두 실패하는 것으로
  재현했다. TCP/proxy 선택을 `credentials.open_ssh_connection`으로 통합했다.
  포트, 인증 방식, 지문 pinning과 실패 시 proxy cleanup을 유지한다.
- 일반 SSH proxy 실패도 helper 오류를 보존해 보고한다. transport 경로 선택은
  인증 방식 변경이 아니다. DDS 주소와 SSH 관리 주소가 같다는 오래된 모듈
  설명도 수정했다.
- 실제 알려진 dev 컨테이너의 Paramiko 3.5.1로 상대 지문을 조회하면 직접
  TCP는 성공하지만 기존 proxy 우선 경로는 8초 후 `No existing session`으로
  실패했다. 20초로 늘린 진단에서도 키 교환이 끝나지 않았다. helper의 배너와
  초기 협상은 왕복하며, client 1,288바이트 협상과 48바이트 다음 메시지가
  helper를 통과한 뒤 응답이 멈추는 것을 바이트 수로 기록했다.
- 같은 경로에서 작은 협상 목록을 사용한 일시적 진단은 성공했다. 별도의
  `tailscale nc` + `ssh-keyscan`도 성공했다. 메시지 크기에 민감한 문제라는
  증거이며 MTU, Tailscale 또는 다른 계층의 결함으로 아직 확정하지 않는다.
  호스트 Tailscale 버전은 1.102.3이다. 제품 암호 알고리즘 목록은 변경하지 않았다.
- 문서상 fallback인 proxy가 작동하는 직접 경로를 덮어쓰지 않도록 고쳤다.
  OpenSSH/Tailscale SSH/지문 조회 모두 직접 TCP 실패 시에만 proxy를 시도한다.
  지문 불일치와 인증 거부 뒤에는 경로 fallback을 하지 않는다. 수정 후 실제
  지문 조회 3회가 **0.091 / 0.046 / 0.035초**에 같은 ED25519 지문을 반환했다.
  직접 경로가 없는 환경의 기본 Paramiko proxy 협상은 별도 미해결 gate다.
  삭제된 원문과의 동일성, 설치된 manager GUI에 대한 수정 적용은 주장하지 않는다.
- SSH 조회의 stale response 회귀는 native 설치 label의 오래된 구두점 기대값에서
  먼저 실패했다. 현재 화면 문자열에 맞춰 실제 stale response 검증이 실행되게 했다.
- 가독성 검사가 삭제된 `payload/apps`를 순회해 **소스 0개로 통과**하고 있었다.
  현재 네 runtime package 경로로 고치고 경로 누락·빈 입력을 실패시킨다.
  실제 검사에서 `WrapGraspEnv` 1,347줄, `UiSimSession` 1,008줄이 기존 class
  제한 1,000줄을 넘는다. 제한을 늘리거나 빈 검사 통과를 복원하지 않는다.
- Sim RL 모듈은 `tensordict`를 직접 import하지만 Sim 선택 의존성과 dev lock에
  선언이 없어 기존 dev에서 전체 Sim 검사가 수집 실패했다. Sim `rl` extra와
  dev lock에 TensorDict 0.14.2, RSL-RL 5.4.0, TensorBoard 2.21.0을 명시하고,
  `test` extra에도 직접 사용하는 TensorDict를 포함했다. 별도 학습 환경 안내도
  같은 extra를 사용한다. 알려진 dev venv에 해당 의존성을 보완했으며 Torch
  2.12.1 / NumPy 1.26.4는 유지했다. 실제 runner import는 통과했지만 학습,
  GPU rollout, 기존 checkpoint 호환성을 증명하지 않는다.
- Sim GO2 MPC 테스트 세 곳의 저장소 상대 경로가 한 단계 위를 가리켜
  두 검사가 실패했다. 현재 테스트 위치에 맞게 고쳤다.
- 격리 릴리스에서 Pilot이 동봉된 arm 모델을 찾지 못했다. 역할의 `config/`와
  나란한 `data/models/arm/default.json`을 먼저 찾도록 하고 독립 디렉터리
  회귀를 추가했다. 명시적 `ELESIM_ARM_MODEL` 우선권과 소스 실행 경로는 유지한다.
- UI가 더 이상 모델 데이터를 배포하지 않는데 release Dockerfile과 manifest에
  `data/` 요구가 남아 있었다. 두 요구를 제거해 실제 UI 의존성과 일치시켰다.

기존 가독성 gate는 네 runtime role만 검사한다. setup/connection의 다음
집중 지점은 별도 감사 대상이며, 아래 크기만으로 기능 결함을 단정하지 않는다.

| 위치 | 현재 책임이 모인 class / 크기 | 먼저 추적할 경계 |
| --- | --- | --- |
| `elesim_connections/connections.py` | `ConnectionDeploymentRunner` / 2,960줄 | 요청 분기, scoped/native transaction, journal recovery, runtime readiness |
| `elesim_setup/container_installer.py` | `ContainerInstaller` / 2,401줄 | 설치 정책 결정, Compose/wrapper 생성, ownership refresh |
| `elesim_connections/secure_deployment.py` | `InstalledElesimLifecycle` / 2,133줄 | host 조회와 변경, 상태 캐시, 배포·시작·검증 실패 전달 |
| `elesim_setup/instance_runtime.py` | `InstanceRuntime` / 1,477줄 | instance 선택, namespace 준비, 정확한 service lifecycle과 cleanup |

#### 현재 검증과 다음 순서

원본 로그는 `workbench/evidence/generated/readiness/20261005-r0/`에 있다.
서로 겹치는 focused test 수를 합산하지 않는다.

- 수정 전 host setup: **1,097 passed / 15 failed**. 소켓 권한 실패 13개,
  stdin stream timeout 1개, frontend label 기대값 불일치 1개였다.
- 해당 socket/GUI/installer 묶음은 권한 밖에서 label 수정 후 **172 passed**.
  stream timeout도 재검증에서 통과했다. 이 결과는 원격 SSH나 DDS 성공이 아니다.
- SSH/credentials 수정 후 집중 검증: **134 passed**. 초기 130개에 더해
  proxy 경유 중 인증 거부·지문 불일치를 원래 오류로 보존하고 자원을 닫는
  agent/key 조합 4개를 확인했다.
- 직접 TCP 우선 수정 전 host setup 전체는 **1,116 passed**. 이후 직접 우선
  선택·fallback·인증·GUI 집중 검증은 **181 passed**. 전체와 집중 결과는
  서로 다른 수정 시점의 증거이며 합산하지 않는다.
- host protocol + 수정 전 quality 검사: **163 passed + 5 subtests**. 이때 quality의
  가독성 통과는 무효다. 검사 경로 수정 후 quality는 **21 passed / 1 failed**로,
  위 두 class 크기 초과를 정확히 보고한다. host UI는 **69 passed**.
- 생성 `elesim-dev`의 정식 실행은 `up -d --build dev`에서 image 재빌드를
  시작했으나 사용자 중단으로 종료됐다. 해당 실행에서는 required가 시작되지
  않았다. 기존 dev 서비스와 사용자 셸은 유지했다.
- 아래 결과는 같은 설치의 생성 `elesim-compose -f <prefix>/containers/compose.yaml
  exec -T dev /usr/local/bin/elesim-dev-env ...`로 기존 서비스를 사용한 증거다.
  새 dev image 재빌드 완료나 wrapper 전체 경로 통과를 의미하지 않는다.
  처음 required 실행은 setup 도중 중단됐으며, 수정한 묶음은 개별 재실행했다.

| 기존 dev 검증 묶음 | 결과 |
| --- | --- |
| protocol / Robot | 141 / 104 passed |
| Pilot 재실행 | 400 passed, 21 skipped |
| Sim 재실행 | 460 passed, 3 skipped |
| UI | 69 passed |
| model/release 도구 재실행 | 81 passed |
| 실제 4프로세스 DDS topology | passed, revocation 누락 후 lease 재획득 포함 |
| DDS RGBD / encoded WebRTC | 2 / 2 passed |
| setup 재실행 | 1,121 passed, 3 skipped |
| extended quality | 21 passed, 1 failed: 위 두 class 크기 초과 |
| critical mutations | 현재 등록된 7개 모두 검출 |
| analysis / debug / experiment | 10 / 4 / 10 passed |
| release build와 내장 격리 검사, 별도 verify 재실행 | Pilot/UI/Robot/Sim 네 역할 passed |

릴리스는 이번 evidence 아래 `release-check/releases`에만 생성했다. 설치된
manager/runtime release 교체나 sidecar 시작은 하지 않았다. 기본 proxy 전용
경로, 새 dev image 재빌드, 실제 두 host 로그인·DDS·영상 수용시험은 남아 있다.

| 순서 | 남은 작업 | 마일스톤과 완료 기준 |
| --- | --- | --- |
| 1 | 정식 gate 기준선 및 직접 경로가 없는 경우의 proxy 키 교환 원인 | R0: 재현 명령·revision·환경과 실패 원인 분류 |
| 2 | GUI→지문→설치 조회→설정 적용의 주소/인증/오류 전달 감사 | R1/R2: 같은 입력이 같은 관리 경로를 사용하며 실패 후 재시도 가능 |
| 3 | 설치 state·topology·release·instance의 쓰기 주체, 취소/rollback 감사 | R1: 격리된 검증 설치에서 생성물·표시 상태·복구 일치 |
| 4 | 단일 host 후 실제 두 host의 시작→두 영상→종료/재접속 | R2: 기존 3회 반복·10분 관찰과 중단 표시 조건 충족 |
| 5 | UiSimSession의 session authority와 stream 재시도 책임 정리 | R2: 현재 크기 초과 해소와 영상/세션 복구 회귀 |
| 6 | 기본 조작과 선택한 작업의 내부 책임 감사; WrapGraspEnv 분해 검토 | R3/R4: 기존 조작/작업 기준, RL 사용 시 관측·reset·성공 계약 보존 |

크기 초과는 구조 부채이며 단순 줄 이동으로 해결하지 않는다. 전체 refactor,
R1/R2 수용 완료, 원래 Tailscale 장애 해결은 아직 주장하지 않는다.

### Motion lease renewal recovery (2026-09-28)

- Reproduced a lost revocation: the target expires its lease while Pilot is
  temporarily undiscovered, then rejects Pilot's continuing renewals with
  `no_active_lease`. Pilot previously kept renewing that rejected lease.
- PeerClient now correlates renewal errors against bounded request history,
  target endpoint/boot and lease identity, emits local `target_lost`, and lets
  Pilot's existing discovery/selection loop reacquire authority. Old errors,
  old release notifications and queued local loss events cannot clear a newer
  grant. Protocol major 6 and all wire shapes remain unchanged.
- Host protocol/Pilot connection regression suite: **153 passed, 5 subtests
  passed**. The four-process DDS smoke now injects a missed revocation and
  requires reacquisition plus accepted motion under the new lease.
- Canonical development-image verification is pending; this does not establish
  why the original deployed lease expired or prove live multi-host timing.

### Mixed native Robot / scoped container preparation (2026-09-16)

- Connection-manager preparation now includes the native Robot unit in the
  scoped transaction: preflight, configuration/security application, readback,
  rollback and interrupted recovery. Robot retains its native systemd lifecycle.
- Jetson cards expose independent native Robot installation lookup and paths;
  overlapping native/container prefix or wrapper paths are rejected. Container
  configuration receives the Robot endpoint from the saved topology.
- Recovery journals accept native-unit progress markers, including a recovery
  interrupted while applying the native target. Regression coverage exercises
  managed/trusted preparation, failure rollback and forward recovery.
- Final host fallback setup suite: **993 passed** (83.75 seconds), including
  local native lookup, noninteractive mixed installation and GPU policy-arrival
  regressions. JavaScript/Python syntax and diff whitespace checks passed.
- Final image reports use tools/Sim/Pilot/UI/Robot order. Alias audit found
  8,733 internal adjective/animal combinations with secure random selection
  and full-name collision checks. Language buttons display 한국어/English;
  SSH confirmation requires a username. An arriving inherit policy restores
  its checkbox default without overwriting choices on subsequent status polls.
- The `elesim-dev` wrapper is unavailable and Docker socket access returns
  permission denied. Host socket tests passed with sandbox escalation; canonical
  development/release gates, live multi-host SROS2 and Jetson systemd/hardware
  acceptance remain unrun. No physical deployment or motion was performed.

### GO2 MPC replacement selection (2026-09-16)

- Selected Quadruped-PyMPC's nominal acados CPU backend for a prototype;
  see [decision and migration gates](go2-mpc-replacement.md).
- Selection/documentation only: the current go2-convex-mpc dependency is still
  enabled. No Genesis, physical Robot or operational milestone is marked passed.
- Reviewed the current uncommitted diff (English runtime messages, read-only
  instance planning and SSH lookup gating). Fixed stale SSH probe results being
  applied to changed endpoints and aligned the lookup button's ARIA disabled
  state with its actual busy/blocked state; added an executable Node regression.
- Host fallback: 44 focused installer/scoped-registration/RL-progress/frontend
  checks and 172 GUI/connection/deployment checks passed. JavaScript syntax and
  diff whitespace checks passed. The installed `elesim-dev` wrapper is absent
  and Docker has no development service container; canonical container gates,
  live multi-host EROFS reproduction and Genesis/MPC behavior remain unverified.
- The separate RL deployment test could not collect because host Python lacks
  `torch`; it is not counted among the passing tests. No host packages were added.

### wrap-grasp-rl 로컬 통합 (2026-09-14)

- 로컬 `integrate/wrap-grasp-rl`에서 main `4628bdb`와 PR #3의
  `437c05c`를 통합한다. 협업 브랜치와 학교 서버는 수정하지 않았다.
- 현재 main의 Pilot 전용 정책 추론기를 유지하며 관측 채널 계약을 이식한다.
  구형 정책의 -0.23 m 범위를 현재 -0.166666667 m 범위로 자동 변환하지 않는다.
  재학습/재평가 없이 manifest만 고쳐 호환성을 주장해서는 안 된다.
- 공통 시작 자세는 Robot에도 적용되므로 main 값을 보존한다. RL Home은
  명시적인 학습/동작 설정이다. CAD 부착 위치와 1.5도/mm 보정은 유지한다.
- 호스트 부분 검증: 프로토콜 137개 및 subtest 5개 통과; 모델/릴리스 도구
  80개 통과(모델 변환의 기존 gimbal-lock 경고 5개). 중복 집계하지 않는다.
- Pilot 집중 검증 25개 통과, Torch/실제 export/Shapely가 필요한 23개 skip;
  Sim 소유권 AST 검사 2개 통과. Python 소스 컴파일과 diff 공백 검사 통과.
- 임시 경로에 네 역할과 protocol wheel을 생성했다. 실제 infrastructure
  검증에서 발견한 setup 모듈 목록 누락 13개를 보완하고 산출물 회귀 검사를
  추가했다. 이후 격리 실행 probe는 호스트의 `python -S`에서 `yaml`을
  찾지 못해 실패했다. 생성 성공을 실행 검증 성공으로 해석하지 않는다.
- 학습 출력은 기본적으로 신규 run만 허용한다. supervisor 재시도는 명시적
  `--continue-run`으로 같은 디렉터리의 checkpoint를 사용하고 기존 metadata를
  보존한다. export 덮어쓰기는 명시적 `--overwrite`가 필요하다.
- [RL 전환 절차](rl-integration.md)에 별도 checkout/외부 run 사용과 Pilot
  artifact 전달 절차를 기록했다. 원격 PR 변경, main 최종 병합·push는 보류한다.
- `/home/user/ws/newsim/install-state.json`에서 developer addon이 비활성이고
  `elesim-dev` wrapper가 없음을 확인했다. Docker 조회에도 개발 서비스는 없다.
  호스트 Sim runtime 테스트는 `genesis` 누락으로 수집 실패했다.
  정식 required/extended, 생성 릴리스 실행 격리, 실제 TorchScript 추론,
  Genesis GPU 학습/재개 및 학교 checkpoint 검증은 아직 완료하지 않았다.

### One-EleSim policy 해체 (2026-09-10, 구현 완료 범위)

현재 goal은 구형 RL 설치를 자동 수정·인수하지 않고 신형 설치와 여러 system의
공존을 구현하는 것이다. 아래 과거 B0 초안의 **전역 고정 Compose project 및
공통 mutable `:local` image** 결정은 다음 계약으로 대체됐다. 구현된 격리·상태
경계와 실제 Docker/RL 통합 수용시험은 별개다.

- 신규 설치 project는 `elesim-<install name>`이며 한 설치의 여러
  system은 이 project를 공유한다. 구형 설치의 `elesim-runtime`은 자동 인수하지 않는다.
- connection topology는 schema v1–v5를 읽어 schema v6으로 normalize하고,
  저장 시에는 항상 v6을 쓴다. 컨테이너 graph role ID는 global registry와
  instance schema v3에 저장하며 schema v2 입력은 읽을 때 이관한다. native
  Robot graph role ID는 topology와 혼합 배포 transaction journal에만 남고
  native Robot용 scoped instance는 만들지 않는다.
- 호스트 설치 / 불변 release / system instance 상태를 분리한다. release는 빌드
  입력 fingerprint와 이미지 ID를 기록하고 system별 참조를 고정한다.
- 서비스·설정·보안 view·쓰기 가능한 cache·로그는 system/endpoint 단위다.
  서로 다른 ID의 이름 변환 충돌을 허용하지 않는다. 같은 system의 중복 role
  실행 거부는 control/media 검증 전까지 유지한다.
- fresh container install의 project는 `elesim-<install name>`다.
  release는 immutable이며 `elesim-update`는 새 release를 build/publish하지만
  등록된 instance를 새 release로 repin하지 않는다. instance는
  `elesim-instance <system> up|down|logs|status|remove`로 exact service만
  대상으로 lifecycle한다. scoped install에서 generic `elesim-up`, `elesim-down`,
  `elesim-logs`, `elesim-status`는 거부되고 legacy fixed project에서만 유지된다.
- 인스턴스 lifecycle은 `--remove-orphans`와 project-wide `down`을 사용하지
  않는다. 설치 잠금과 system별 transaction journal/lock을 구분한다.
- 기존 실행 설치를 새 schema로 강제 이관하지 않는다. 공존 설치는 별도 prefix/bin과
  PATH 비등록을 사용한다. 실제 RL 서버 배포·종료·이관은 이번 goal에서 수행하지 않는다.
- 현재 확인은 host 테스트뿐이며 canonical/live Docker proof는 주장하지 않는다.
  실제 RL server는 건드리지 않았다. B6 live integration 수용시험은 수동으로
  남아 있다.

Luna 작업 분담: namespace 식별자 및 검증, instance state/registry 및 검증,
기존 lifecycle 호출 경로 조사. 주 에이전트가 계약·기존 코드 통합·수용시험을 소유한다.

#### 상태와 운영 경계

`system_id`가 instance의 정본 key다. 한 신규 host 설치의 여러 system은
설치 UUID로 구분된 project를 공유하되, 구형 RL 설치와 공존할 때는 별도
prefix/bin 및 project를 사용한다. 신규 container 설치는 이 scoped 경로를
기본으로 사용하고 legacy manifest만 기존 고정 project를 유지한다.

상태와 자원의 소유 범위는 다음 세 층으로 나눈다.

| 범위 | 소유할 값과 자원 |
| --- | --- |
| host 설치 | prefix/bin, Docker context/Engine ID, 설치된 role capability, 공통 build cache, scoped dev/Tailscale, 전체 ownership manifest |
| 불변 release | source revision, platform, role별 build fingerprint/image ID, runtime data snapshot 및 digest |
| system instance | schema v3의 컨테이너 graph role ID, `system_id`, release key, endpoint 배정, DDS/compute/TURN 설정, endpoint별 config와 SROS2 view, 캐시·로그·실행 상태 |
| graph topology | host와 role 배치, DDS/SSH endpoint, endpoint ID, SROS2 Authority generation과 배포 transaction |

구현은 registry와 생성 설정을 원자적으로 통합하고, 소유권 manifest와
instance별 config/cache/log/security/TURN/GPU 경계를 기록한다. legacy 상태와
고정 project는 보존한다.

```text
<prefix>/
  install-state.json
  containers/compose.yaml
  releases/<release_key>/
    manifest.json
    data/
  instances/<system_id>/
    state.json
    endpoints/<endpoint_id>/
      config/
      cache/
    security/
    secrets/
    cache/
    logs/
  connections/<system_id>/topology.json
  authority/<system_id>/
```

공통 dev/tools/Tailscale은 설치당 한 번 생성한다. fresh runtime container는
`elesim-<install name>-<system_id>-<role>` alias를, 설치 공용 container는
`elesim-<install name>-<component>` alias를 사용한다. application service key,
설치 UUID, endpoint/role exact label을 함께 검증하고 실행 image는 불변 image ID로
pin한다. 기존 manifest의 hash형 naming은 호환을 위해 유지하며 instance 삭제는
공통 image를 삭제하지 않는다.

운영 명령은 `elesim-instance <system> <up|down|logs|status|remove>`를 사용한다.
scoped generic wrapper는 fail-closed하고 legacy fixed project wrapper는 기존
동작을 유지한다. instance stop/remove는 선택한 service만 대상으로 하며
project 전체 `docker compose down`과 `--remove-orphans`는 사용하지 않는다.
서로 다른 system의 연결관리자는 동시에 실행할 수 있지만 같은 system의 저장·보안
배포는 system별 lock으로 직렬화한다.

DDS application topic과 SROS2 policy는 이미 `system_id` namespace를 사용하므로
이 기능만을 위한 wire protocol version 변경은 계획하지 않는다. 각 graph의
`system_id`는 설치 내에서 유일해야 한다. 서로 다른 system은 같은 DDS domain을
공유할 수 있으며, 각 graph의 모든 participant는 해당 system의 `system_id`와
`domain_id`를 함께 사용한다. DDS domain은 discovery 범위이며 보안 경계가 아니다.
보안 경계는 계속 SROS2 enforce다.

Tailscale sidecar와 개발 attachment는 host 공용으로 유지한다. managed Coturn은
host/Tailscale network namespace의 listen/relay port가 충돌하므로 instance별
고정 listen port와 relay block을 설치 UUID/system ID에서 결정적으로 할당하고,
같은 prefix 안의 충돌을 등록 전에 거부한다. 다른 prefix의 실제 host bind 충돌은
Docker가 최종 거부하며 live 공존 gate에서 확인해야 한다.
외부 TURN은 instance별 credential 경로를 가진다. GPU 선택과 writable runtime
경로도 instance 설정으로 내려 같은 host의 두 Sim이 설정 파일을 공유하지 않게 한다.
신규 image tag는 설치 이름과 역할별 readable release alias를 사용하며, 설치 UUID와
build fingerprint는 소유권·검증 metadata로 보존한다. host 설치 단위 build lock을
둔다. instance 제거는 image를 삭제하지 않고 전체 host uninstall만 공통 image
제거를 소유한다.

물리 Robot은 host당 하나의 native 안전 경계와 고정 systemd lifecycle을 유지한다.
한 Robot host에서 두 system이 Robot을 동시에 활성화하는 것은 거부한다. native
Robot과 같은 Jetson의 컨테이너 역할은 하나의 scoped deployment transaction에서
함께 준비·적용·검증·복구하며, native Robot 자체에는 별도 system instance를 만들지
않는다. 초기 완료 범위는 여러 Robot 없는 graph의 동시 실행과, Robot을 포함한
graph 하나가 별도 Robot 없는 graph와 공존하는 경우까지다. templated systemd나
하나의 물리 Robot을 여러 graph가 공유하는 기능은 요구가 생기기 전에는 만들지 않는다.

#### 다중 system 마일스톤

| ID / 상태 | 결과 | 완료 증거 |
| --- | --- | --- |
| B0 계약 / 확정 | legacy 설치 보존 및 install/release/system 경계를 구분한다 | 위 범위와 명시적인 미지원 경계 |
| B1 상태 분리 / 완료(소프트웨어) | host 설치·immutable release·system instance가 독립적으로 저장된다 | instance schema v3, global role ID, schema v2 read migration, registry/release publication 회귀 |
| B2 Compose namespace / 완료(소프트웨어) | 한 install project 안에서 system/endpoint별 service와 자원이 격리된다 | exact service rendering, per-instance config/cache/log/TURN/GPU/security, legacy preservation 회귀 |
| B3 lifecycle / 완료(소프트웨어) | 한 system의 up/down/status/logs/remove가 다른 system을 건드리지 않는다 | `elesim-instance` wrapper, generic scoped refusal, transaction journal/lock 회귀 |
| B4 연결·보안 / 완료(소프트웨어) | topology와 SROS2 transaction이 system별로 독립적이다 | schema v6 read migration, exact registration/replace, staged publication·rollback·recovery 회귀 |
| B5 공용 인프라 / 완료(소프트웨어) | immutable release와 공용 dev/Tailscale을 공유하며 instance 자원 충돌을 거부·할당한다 | release build/publish, sidecar/image ownership, Coturn/GPU/cache/log 경계 회귀 |
| B6 통합 수용 / 수동 | 여러 graph의 제어·RGBD·WebRTC·종료가 격리된다 | live Docker/RL, multi-host media, SROS2 교차 publish/subscribe 거부, Robot 독점 검증은 미실행 |

B0에서 고정한 위험 회귀는 세 가지다. `down`/`--remove-orphans`가 다른
system을 제거하지 않는 것, 연결관리자의 전역 lock 대신 system별 transaction
journal/lock을 사용하는 것, role config와 Sim cache를 instance 경로로 격리하는
것이다. 이 경계는 구현·회귀 검증됐다.

첫 vertical slice의 host-side isolation checks는 구현됐다. 다만 실제 Docker
동시 실행과 DDS/WebRTC/RL 통합은 B6 수동 수용시험으로 남아 있다.

2026-09-10 bounded final host fallback에서는 one-EleSim 관련 집중 묶음
**279 passed**, setup 전체 **837 passed / 14 failed**였다. 전체 실패는 이
sandbox에서 금지된 loopback/Unix/X11 socket 생성 13건과 그에 따른 stream
timeout 1건이며 제품 성공으로 바꾸어 세지 않는다. 이후 추가한 release UUID
조회, graph role ID canonical 충돌, 중첩 prefix 거부 경계까지 포함한 최종
관련 묶음은 **123 passed**, downstream connection/deployment 묶음은
**151 passed**다. `py_compile`, connection-manager JavaScript syntax,
`git diff --check`도 통과했다. 정식 `elesim-dev`, Docker daemon, live graph는
사용할 수 없어 B6와 canonical required/extended/release gate는 미실행이다.

### 연결관리자 COM 편집 화면 개편 (2026-09-08)

- 고정된 세로형 COM 카드와 `미사용` 토글을 제거했다. 실제 등록된 COM만 세로로
  나열하고, 각 COM 내부는 네트워크(IP/interface) / 역할 drag-and-drop / 필요한
  경우의 SSH 설정을 가로 3열로 표시한다. 좁은 화면에서는 두 열과 한 열로 접힌다.
- 설치 경로는 좌측에서 상시 표시한다. `이 컴퓨터가 연결 관리자를 실행함`은
  `이 컴퓨터는 운영용임`으로 바꿔 우측 SSH 제목 옆에 배치했고, 운영용 COM에서
  SSH 입력 전체를 숨겨 의미 없는 안내 공간도 없앴다. SSH 사용자명은 비어 있는
  값으로 시작한다. 역할 열을 이전 시안보다 줄이고 그 폭을 SSH/private-key 열에
  넘겨 긴 인증 경로를 더 여유 있게 표시한다.
- `+ COM`은 `컴퓨터 추가`로 바꿨고 dialog에서 일반 컴퓨터와 Robot 컴퓨터
  (Jetson)를 구분해 추가한다. 전역 역할 추가 버튼 대신 각 COM 역할 영역 우상단의
  `[+]`가 그 COM에 역할을 바로 추가한다. 편집 보드의 역할 카드 개수에는 제한이
  없고, 번호는 모든 COM을 합쳐 역할별로 독립 증가한다(`UI 1`, `UI 2`, `Sim 1`).
  한 줄에는 카드 세 개를 놓으며 drop zone은 기본 세 줄과 하단 drop 여백을 항상
  보여준다. 카드 drag 중 viewport 상·하단에서는 연속 자동 스크롤하고, 삽입
  위치에는 보이지 않는 grid placeholder를 넣고, 마우스 커서가 들어간 카드의
  칸을 선택한다. 기존 카드는 FLIP 이동 애니메이션으로 다음
  열/행으로 물러난다.
- 각 컴퓨터 헤더의 `COM1`, `COM2` 이름은 편집할 수 있으며 별도 host ID 입력란은
  제거했다. 검증된 컴퓨터 이름이 topology의 host ID가 된다. 현재 편집 버튼에는
  연필 자산이 준비될 때까지 기존 코끼리 그림을 사용한다. 헤더의 `▲`/`▼`는 화면과
  저장되는 host 배열의 순서를 함께 바꾼다.
- 실행 모드 선택을 제거하고 topology schema를 v6으로 올렸다. 1–4개 COM과 실제
  역할 카드가 graph를 직접 정의하며 Pilot/Sim/UI/Robot의 고정 집합을 강요하지
  않는다. v1–v5는 v6으로 이관하고 기존 `topology_mode`는 검증 후 폐기한다. 복수
  역할 카드는 endpoint ID와 함께 저장하며 instance별 Compose/service namespace로
  exact lifecycle을 수행한다.
- 새 역할 카드의 endpoint ID는 역할별 `pilot-1`, `sim-1`, `ui-1`, `robot-1`
  형식으로 번호를 붙인다. Robot COM의 역할 영역은 세로로 두 zone을 쌓지 않고,
  같은 높이 안에서 일반 역할 2/3와 고정 Robot 1/3을 좌우로 배치한다. 일반 역할
  카드는 좌측 두 열, Robot은 우측 한 열을 사용한다. 역할 추가 안내는 실제로
  카드를 추가하는 좌측 일반 역할 zone에만 표시한다.
- 상단의 `네트워크: 로컬/인터넷` 선택은 제거했다. 연결관리자는 명시된 COM 주소를
  이미 알고 있으므로 저장되는 topology에는 항상 static discovery를 사용한다.
  COM별 DDS 주소와 Interface 입력은 그대로 유지한다.
- setup 전체 suite **613 passed**. canonical `elesim-dev`는 Docker socket
  권한 거부 및 wrapper 부재로 사용할 수 없어 승인된 host localhost/socket 실행을
  사용했다. 정식 전체 gate와 실제 브라우저·다중 host 수용시험은 별개다.

### 설치 역할과 topology 배정 분리 (2026-09-07)

- 설치 상태 schema v11에서 설치 capability인 `roles`와 현재 graph 배정인
  `assigned_roles`를 분리했다. 기존 v1-v10 상태는 `assigned_roles=null`로
  이관되어 설치 역할 전체를 쓰는 동작을 보존한다.
- 연결 관리자는 `assigned_roles`가 설치 역할의 부분집합이면 허용한다. DDS/XML,
  Compose 환경, endpoint identity, SROS2 app view와 start/stop/build는 현재 배정
  역할만 대상으로 한다. 비활성 역할의 설치 파일과 image는 보존한다.
- 재배정 시 실행 중인 비활성 역할이 있으면 설정 변경 전에 거부한다. 이 문단의
  과거 install-wide `elesim-up` 동작은 legacy fixed project에만 남고, scoped
  설치는 `elesim-instance <system> up`으로 exact assignment만 시작한다. Pilot만
  시작할 때 다른 system의 Sim/Coturn을 끄지 않는다.
- 동일 host에서 여러 graph를 동시에 실행하는 instance namespace 분리와
  per-instance lifecycle은 구현됐다. Compose project는 install alias로 고정하고
  container alias는 system/role, service/resource는 system/endpoint로 구분한다.
- `elesim-dev`는 Docker daemon에 존재하지 않아 canonical container gate를
  실행하지 못했다. 호스트 setup suite를 소켓 허용 구간과 일반 구간으로 나눠
  **612 passed**로 확인했고 bootstrap **73 passed**, 관련 상태/배포/실행 회귀
  **83 passed**를 재확인했다. required gate의 Protocol 131, UI 67과 DDS RGBD 2는
  통과했지만 host 환경의 Unitree socket 권한, Genesis, ROS 2, aiortc/av와 wheel
  build backend 부재로 Robot 일부, Sim, topology, WebRTC 및 isolated release는
  canonical 성공 판정하지 않는다.

### UI·Robot 호출 경로 정리 (2026-09-07)

- UI 영상 재연결 성공/실패에 중복된 backoff 계산을 통합했다. 재시도
  횟수 16, 간격 5초 상한, 다른 영상·DDS session 보존과 응답 후 초기화를
  반복 실패/성공 회귀로 확인했다. 사용하지 않는 panel 상태 하나도 제거했다.
- Robot 실제 entrypoint/device 호출에 없는 tick 변환, Sim 속도 추정,
  midpoint 이동과 IPC 빈 로그 메서드를 제거했다. 카메라의 미사용 물체
  위치 추정기와 전달 모듈을 없애고 RealSense intrinsics는 기존 공통
  `RgbdIntrinsics`를 직접 사용한다. 실제 bridge 로그와 모드별 안전 정지는 유지한다.
- UI의 기존 sag 테스트는 저장소에 없는 preset 폴더를 가정해 실패했다.
  테스트 전용 임시 config에서 폴더 유무와 상대 경로 fallback을 검증하도록
  수정했으며, 이를 위해 불필요한 source 폴더를 만들지 않았다.
- dev container/래퍼 부재를 재확인했다. 호스트 UI 전체 **67 passed**,
  Robot 전체 **102 passed + 2 subtests**. Robot socket/peer-credential 검사는
  sandbox 권한 거부 후 승인된 임시 UDS/가짜 장치 실행으로 통과했다.
  실제 하드웨어나 정식 컨테이너 검증을 대신하지 않는다.
- 이번 runtime **143줄 순감**, 이전 정리 포함 **1,466줄 순감**(model 이동 제외).
  삭제된 source는 Git HEAD에서 복구 가능하며 외부 데이터/설치물은 그대로다.
  Pick/Sim scientific 회귀, isolated release와 실제 운영 경로 검증은 여전히 남는다.

### 설치·연결 군살 제거 후속 기록 (2026-09-07)

- 실제 frontend 호출과 host 실행 경로를 Luna 두 작업과 주 에이전트가
  교차 검수했다. 설치 마법사의 미사용 SSH fingerprint API와 파일 선택
  분기를 제거했다. 설치 경로 선택·root 제한 및 연결관리자의 SSH 기능은 유지한다.
- CLI preset은 사용되지 않는 이름/설명/remote metadata 클래스 대신 role
  tuple 표로 축소했다. 기존 CLI profile 이름·역할·custom 검증은 유지한다.
  내부 Python `Profile` export와 미사용 updater `pull_services` 인자는 제거했다.
  일반 update는 여전히 build만 하고 전용 Tailscale update만 pull/recreate한다.
- 상태 poll의 `compose config --services`는 container unit당 최대 세 번에서
  한 번으로 줄였다. 다음 poll에는 다시 읽는다. configured/created/running
  구분과 조회 오류, Coturn/Tailscale readiness 판정을 보존했다.
- 호출자가 없는 `_switch_only`와 테스트만 거치던 forwarding helper 둘을
  제거했다. generation activate/rollback, legacy ownership cleanup, host
  helper와 runtime-tools namespace 경계는 실제 사용하므로 남겼다.
- 이번 setup runtime 순감 **124줄**, 앞선 runtime 정리와 합쳐 **1,323줄**
  순감(model 이동 제외). 설치물·로그·외부 데이터 변경 없이 소스만 정리했다.
- dev container/래퍼 부재를 재확인한 뒤 호스트 부분 검증:
  `PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=payload/runtime/docker/setup/app:payload/runtime/common/protocol python3 -m pytest -p no:cacheprovider workbench/tests/setup -q --tb=short`
  → **600 passed**. 임시 HTTP/Unix socket 테스트는 sandbox 권한 거부 후
  승인된 실행으로 재검증했다. JS `node --check`와 `git diff --check`도 통과.
- 정식 required/extended, isolated release, real-RMW/GPU/실물 gate는 아직
  미실행이다. 다음은 개발 환경 확보 후 앞서 막힌 Pick/Sim 회귀와 실제
  UI→Pilot→Sim 조작·실패·취소 경로를 검수한다. 전체 감사/R 완료를 주장하지 않는다.

### 2026-09-07 로직 정리 중간 기록

- Luna 세 작업으로 Pilot workflow, perception, tracing 호출 경로를 나누어
  조사했다. 사용량 제한 이후 남은 UI 조건문과 연결부는 주 에이전트가 검수했다.
- 항상 false인 Pick/Gaze 원격 위임과 예외만 발생시키던 client 명령을
  제거했다. Perception도 Pilot 실행으로 일원화했다. 실제 RGBD 수신과
  observation 전달, hardware LJI 차단 및 로컬 stop 경계는 유지한다.
  이전 `provider=host`/`run_local=false` 설정은 명시적으로 거부한다.
- UI의 snapshot 캐시로 처리하는 읽기 명령을 원격 operator 허용 목록에서
  제거했다. wire/view 버전 유지 및 구형 custom client 거부 결정은
  `dds_contracts.md`에 기록했다.
- Pilot/Sim tracing 중복 및 Robot 전달 facade를 기존 공통 tracing으로
  통합했다. 로컬 JSONL은 Pilot/Sim의 명시적 tracing 활성화에만 생성하며
  파일당 10 MiB, 활성 파일 포함 네 개로 회전한다. Sim의 주기적 성능
  출력 기본값은 껐으며 명시적 설정으로 다시 켤 수 있다.
- 기존 model 이동을 제외한 runtime diff는 약 1,200줄 순감이다. 제거한
  소스는 Git HEAD로 복구할 수 있으며 설치물·로그·장비는 삭제하지 않았다.
- 재확인: canonical/live Docker proof는 수행하지 않았고 실제 RL server도
  건드리지 않았다. scoped generic lifecycle은 fail-closed이며 등록된 system은
  `elesim-instance`로만 조작한다.
- **호스트 부분 검증만 수행:** protocol 전체, Pilot operator, UI
  operator proxy/session, Pilot perception YAML ownership, 공통 tracing
  회귀 묶음은 159 passed + 5 subtests. 회전 한도, 기록 경로 실패,
  consumer trace carrier, JSON-only sampling도 포함한다.
- `test_control_client.py`, `test_perception_config_update.py`,
  `scenarios/pick/test_75_mobile_pipeline.py`, Sim `test_config.py`는
  `ModuleNotFoundError: scipy`로 수집 실패했다. required/extended 전체,
  isolated release, real-RMW topology 및 GPU/실물 경로는 이번 revision에서
  검증하지 못했다. 기존 성공 기록으로 대신하지 않는다.
- 다음 검수: 정식 개발 attachment에서 위 네 회귀와 전체 gate 실행;
  설치·연결관리자의 GUI→작업 실행→실패/rollback 흐름을 계속 추적한다.
  legacy state/ownership migration, 미연결 typed ROSIDL은 사용자가 없다는
  추정만으로 삭제하지 않았다. 전체 감사와 R 마일스톤은 아직 미완료다.

기능을 추가하기 전에 설치 → 연결 → 표출 → 조작 → 작업 수행 경로를
검증한다. 아래 `R0`–`R5`는 운영 수용 마일스톤이다. 과거 구현 단계의
`M1`, `M2-A`, `M2-B` 완료 기록과 별개이며, 기존 테스트 통과 기록으로
자동 완료 처리하지 않는다. 현재 완료된 R 마일스톤은 없다.

프론트는 설치마법사, 연결관리자, UI, Sim 표출이다. 백은 설치·연결 실행부,
Pilot, Robot과 Sim의 가상 장치·물리 실행부다. 이는 감사할 책임의 구분이며
패키지 이동이나 새로운 서비스 분리를 지시하지 않는다.

### 범위와 대표 환경

- 첫 목표는 **R1 설치 + R2 연결·표출**이다. R0는 이를 위한 검증 준비다.
- 기본 수용 경로는 Robot 없는 Pilot/Sim/UI 각 하나다. 우선 한 host의
  native Docker Engine과 실제 GPU/display에서 확인한다. 이는 제안된 기준
  환경이며 현재 장비가 확보됐다는 뜻이 아니다. 실제 첫 실행 전에 host,
  GPU, OS, Docker backend, interface, 보안 profile을 기록해 고정한다.
- 이후 R2에서 UI host와 Pilot/Sim host를 분리한 실제 두-host 경로를 확인한다.
  한-host만 통과하면 R2는 부분 완료다. DDS와 SSH 주소를 각각 기록한다.
- 소유 LAN/제한된 routed VPN은 `trusted-network`를 사용할 수 있다.
  공유망을 사용하면 해당 실행 전 SROS2 enforce 검증이 선행 조건이다.
- Docker Desktop/WSL, TURN relay, CPU-only, 다른 카메라 profile 등은 별도
  환경 gate다. 대표 환경 통과를 이 환경들의 지원 증거로 확대하지 않는다.
  실제 사용 장비가 해당 환경이면 그 gate를 현재 마일스톤의 선행 조건으로 올린다.
- Pick·Gaze·Wrap 중 작업 하나를 선택하는 일은 R4 착수 조건이다. 지금 모두를
  완성 대상으로 삼거나 사용 여부를 추측해서 삭제하지 않는다.

### 마일스톤과 완료 조건

| ID / 상태 | 사용자에게 보장할 결과 | 완료에 필요한 증거 |
| --- | --- | --- |
| R0 검증 준비 / 부분완료 | 동일 revision에서 실패를 재현할 수 있다 | 기존 dev 전체 gate·격리 release 통과; 새 dev image build와 원래 삭제된 오류 동일성은 미확인 |
| R1 설치 / 부분완료 | 마법사에서 설치를 끝내고 설치 상태를 다시 확인한다 | 설치·재진입·입력 실패·취소·동일 조건 재시도; 생성물과 표시 상태 일치 |
| R2 연결·표출 / 부분완료 | 실제 두-host DDS와 두 영상 디코딩·창 표시 확인; 유지·반복 검증 진행 | 한-host 및 두-host 기동, 실제 frame 갱신, 종료·재시작, 한 peer/영상 중단 표시 |
| R3 기본 조작 / 부분완료 | 실제 UI→Sim pause/step/resume/reset과 결과·epoch 확인 | UI→Pilot→Sim motion/telemetry 왕복, lease 상실·반복 조작과 정지 시간은 남음 |
| R4 Pick / 대기(R3, 실제 장면) | 선택한 작업을 성공·실패·취소 후 다시 실행한다 | 고정 입력의 반복 실행, 작업별 성공 기준, 실패 사유와 재시도 결과 |
| R5 실물 / 대기(R3, 장비) | 기본 조작과 로컬 안전이 Robot에서 성립한다 | Jetson/GO2/arm 실측, bridge/통신 상실 시 정지와 cleanup, 장치 피드백 |

R5의 기본 장치 검증은 R4 알고리즘 완성을 기다릴 필요가 없다. 다만 R4에서
선택한 작업의 **실물 성공**을 주장하려면 R4와 R5 양쪽 증거가 필요하다.

**R0 — 검증 준비**

1. 설치 UUID, prefix, Docker context/Engine과 Compose 소유권을 확인하고
   setup-generated `elesim-dev`를 사용할 수 있게 한다. 실행 중인 다른
   prefix의 legacy fixed `elesim-runtime` 또는 다른 install-scoped project를 임의로
   교체하지 않는다.
2. 아래 required/extended gate와 release build/verify를 정확한 revision에서
   실행한다. 실패를 제품 결함, 테스트 결함, 환경 제약으로 분류한다.
   원인을 모르는 실패는 그대로 미해결로 남긴다.
3. 후속 milestone의 테스트를 막는 실패를 해소하고, 나머지 실패는 재현 명령,
   영향 범위, 처리할 R 단계와 함께 기록한다. 부분 통과는 전체 gate 통과가 아니다.

**R1 — 설치**

- 같은 revision의 bootstrap → 마법사 → 검증 → 설치 완료 → 새 shell의
  상태 명령까지 실행한다. source config 수동 수정 없이 prefix가 완성돼야 한다.
- 설치 완료는 파일/context 생성 완료를 뜻한다. image build와 runtime 기동은
  R2에서 판단한다. 화면도 이 차이를 표시해야 한다.
- 잘못된 입력은 수정 가능한 사유를 내고, 취소·실패 후 같은 조건으로 다시
  시도할 수 있어야 한다. 상태 파일과 실제 생성물을 함께 확인한다.
- 재설치/update/uninstall은 전용 검증 prefix에서 확인한다. update의 실행 중
  역할 보존과 manifest 기반 제거 경계를 검증하고 외부 checkout은 보존한다.

**R2 — 연결·표출**

- 연결관리자에서 topology 저장 → 설정 적용 → 전체 시작을 수행한다.
  build, process 실행, DDS discovery, Sim scene/session, 영상 준비의 상태와
  실패 사유를 구별해 볼 수 있어야 한다.
- observer/hand-eye 각각 실제 디코드 frame 갱신을 확인한다. ICE 연결이나
  검은 fallback frame만으로 통과하지 않는다. 창 닫기·재열기와 시점 조작도 확인한다.
- 대표 환경별 시작→표출→종료를 3회 반복하고, 한 번은 10분간 두 영상을 관찰한다.
  이는 초기 회귀 수용 기준이며 장시간 안정성 보증이 아니다. 기동 시간,
  frame age, FPS, stall/recovery를 기록한다. 성능 합격 수치는 첫 측정 후
  용도에 맞게 정하고 별도 실행에서 검증한다.
- 한 peer 종료와 한 영상 중단을 주입한다. UI가 stale/실패를 표시하고, 재시작
  후 회복하거나 필요한 수동 조치를 구체적으로 알려야 한다.

**R3 — 기본 조작**

- 자동 Pick/Gaze/Wrap을 켜지 않고 관절·그리퍼·GO2 기본 명령의 실제 적용과
  telemetry를 확인한다. ACK 수신만으로 이동 완료를 판정하지 않는다.
- 중단, Sim pause/reset, Pilot 종료, target 변경 때 이전 작업과 명령이
  되살아나지 않는지 확인한다. pause 중에도 상태·세션 관리는 살아 있어야 한다.
- 정상 조작과 장애 주입을 각각 3회 반복한다. 명령/피드백 오차와 정지 시간은
  해당 설정의 limit/deadman/lease 기준과 대조하고 revision·설정값을 함께 남긴다.

**R4 — 작업 하나**

- 실제 필요한 작업 하나와 입력·장면·성공 조건을 먼저 고정한다. 예를 들어
  mock attach, 물리 파지, 물체 들어 올리기는 서로 다른 성공 조건이다.
- 선택한 작업만 UI 진입점부터 Pilot 계산, Sim 결과까지 추적한다. 성공 사례뿐
  아니라 입력 없음/도달 불가/중단을 포함해 상태와 재실행 가능성을 확인한다.
- 시험 횟수·성공률·허용 오차·시간 제한을 작업 착수 때 정하고 성공 사례만
  선별하지 않는다. 다른 자동 작업은 후속 후보로 남긴다.

**R5 — 실물**

- 장비 담당자가 확보된 환경에서 두 systemd unit, 전용 계정/UDS credential,
  private Unitree NIC/domain, 장치 방향·limit와 실제 피드백을 확인한다.
- Pilot/bridge 상실, 잘못된 packet, deadman 만료 시 실제 정지 시간을 측정한다.
  GO2 경로 실패 중에도 arm safe-hold·torque-off·cleanup을 확인한다.
- Sim 성공이나 mock test는 물리 안전 증거가 아니다. 장비/현장 검증이 없으면
  R5는 대기 상태를 유지한다.

### 코드와 기존 테스트 진입점

아래 경로는 저장소 루트 기준이다. 실행 suite와 허용 import 경로는
`workbench/tools/quality/check.py`를 정본으로 사용한다.

| 단계 | 코드 진입점 | 기존 focused test / 증거의 한계 |
| --- | --- | --- |
| R1 | `payload/runtime/docker/setup/app/elesim_setup/service.py`, `gui.py`, `container_installer.py` | `workbench/tests/setup/test_gui.py`, `test_container_installer.py`, `test_uninstall.py`; 생성물·mock 검증은 실제 설치 증거가 아님 |
| R2 | `payload/runtime/docker/setup/app/elesim_connections/connections.py`; `payload/runtime/docker/ui/app/elesim_ui/sim_session.py` | `workbench/tests/setup/test_connections.py`, `workbench/tests/apps/ui/test_sim_session.py`; fake peer/receiver 통과는 실제 두-host 영상 증거가 아님 |
| R3 | `payload/runtime/docker/pilot/app/elesim_pilot/operator.py`; `payload/runtime/docker/sim/app/elesim_sim/endpoint.py` | `workbench/tests/apps/sim/test_endpoint.py`, `workbench/tests/protocol/test_peer_authority.py`; 명령·권한 검증은 실제 이동/정지 측정과 별개 |
| R4 | 선택한 Pilot 작업의 UI 호출과 `payload/runtime/docker/pilot/app/elesim_pilot/pick/` 또는 `gaze/` | Pick 선택 시 `workbench/tests/apps/pilot/headless/test_pick_workflow.py`, `workbench/tests/apps/pilot/test_pick_stop_lifecycle.py`; stub phase 검증은 perception·IK·파지 성공 증거가 아님 |
| R5 | `payload/runtime/native/robot/app/elesim_robot/main.py`, `runtime.py`, `go2/unitree_bridge_daemon.py` | `workbench/tests/apps/robot/test_main_lifecycle.py`, `test_unitree_ipc.py`; local UDS/fake backend는 GO2 장치 검증과 별개 |

표의 짧은 파일명은 같은 셀에서 앞서 명시한 디렉터리 기준이다.
`workbench/tests/system/smoke_topology.py`는 네 role의 실제 RMW process probe다.
`test_dds_rgbd.py`, `test_webrtc_media.py`는 같은 system 디렉터리에 있으며
실제 생성된 영상, GPU, 원격망과 TURN relay를 포괄하는 수용시험은 아니다.

### 현재 알려진 검증 공백과 첫 작업

2026-09-05/06 세션 기록에 근거한 시작점이며, 현재 revision의 재시험 결과가 아니다.

- 정식 `elesim-dev`가 없었고 `elesim-up`/`elesim-dev`가 PATH에서 발견되지 않았다.
  당시 실행 중인 프로젝트는 `/home/user/ws/newsim/containers/compose.yaml` 소유였다.
  R0에서 실제 설치 상태를 다시 조사하고 소유권이 확인된 경로로 준비한다.
- 호스트 SciPy 미설치로 Pilot/Robot/model 테스트가 수집되지 않았다.
  release wheel 검증은 setuptools 59.6.0의 `UNKNOWN/0.0.0` 생성으로 실패했다.
  호스트에 의존성을 추가하지 말고 정식 환경에서 재시험한다.
- setup 호스트 실행은 585 passed / 13 failed였다. 소켓 permission 실패와
  stream timeout이 있었으며, timeout까지 동일 원인으로 단정하지 않는다.
- UI는 64 passed / 1 failed: SAG 탐색 기본값 테스트가 존재하지 않는 `sag/`
  디렉터리를 기대했다. 구현의 config-root fallback과 기대 계약을 R0에서 대조한다.
- `model/` 경로 평탄화는 미커밋 변경이다. 검증 증거에 HEAD만 쓰지 말고
  working-tree diff 식별도 포함한다. 마일스톤 문서 작성은 이 변경의 재검증이 아니다.

다음 담당자는 R0의 환경/실패 기준선부터 시작한다. 이후 R1과 R2를 순서대로
수행하며, 각 단계의 실패를 고치는 데 필요한 변경만 진행한다.

### 작업 선택과 증거 기록 규칙

- 프론트 진입점 → 요청 → 백 처리 → 실제 결과 → 화면 상태를 한 기능으로
  추적한다. 테스트 개수나 코드 줄 수는 사용률/완료율이 아니다.
- 코드는 현재 경로 필수 / 다른 운영 조건에 필요 / 후속 작업용 / 용도 불명으로
  분류한다. 삭제는 대체 경로·실제 사용 여부·안전/복구 영향 확인 후 결정한다.
- 현재 gate를 막지 않는 대규모 재구조화, UI 전면 개편, typed service/action
  전환, 새 자동 작업, 임의 성능 최적화는 착수하지 않는다.
- 실패한 시나리오는 최소 재현을 남기고 기존 focused test를 보강한다. 기존
  suite와 같은 확인만 하는 새 테스트 체계나 별도 마일스톤 도구는 만들지 않는다.
- 상태는 미착수 / 진행 / 환경대기 / 부분완료 / 완료로 갱신한다. 완료에는
  모든 해당 수용 조건의 증거가 필요하다. 환경이 없다는 이유로 통과 처리하지 않는다.
- 원본 로그는 `workbench/evidence/generated/readiness/<run-id>/`에 둔다.
  검토할 요약만 `workbench/evidence/curated/readiness/`로 승격한다. 실제 실행 전
  빈 폴더를 만들지 않으며 token, private key, TURN secret은 보관하지 않는다.
- 각 결과는 `R-ID / 날짜 / commit+diff / host·topology·backend·보안 / 설정 /
  실행 명령·조작 / 기대 결과 / 관측 결과·측정값 / pass·fail·blocked /
  증거 경로 / 다음 조치`를 포함한다. 이 문서는 요약과 증거 링크만 소유한다.

## 기존 software 구현 범위

- Pilot/Sim/UI/Robot의 Router-free ROS 2/DDS 직접 통신과 protocol v6
  `PeerEnvelope`가 구현됐다.
- descriptor/heartbeat, boot/sequence fence, bounded startup queue, Robot/Sim
  motion lease, Sim UI session이 구현됐다.
- encoded latest-only RGB-D broker와 observer/hand-eye WebRTC 분리가 구현됐다.
- Robot–Unitree bridge UDS 경계, peer credential 검증, replay fence와 deadman
  stop이 구현됐다.
- installer state v11, mode-free topology schema v6 (v1–v5 read/normalize), 독립
  DDS/SSH endpoint, ownership-based uninstall이 구현됐다.
- install-scoped Compose namespace와 선택적 `elesim-dev` attachment, managed
  Coturn, Docker Desktop Tailscale sidecar가 구현됐다. Legacy fixed
  `elesim-runtime` 자원은 자동 인수하지 않으며, 실제 공존 운영은 아직 live gate가
  아니다.
- role-scoped managed SROS2 generation의 stage/activate/verify/rollback/recover와
  external keystore 경계가 구현됐다.
- four-role + infra release build/verify, four-process DDS smoke, RGB-D 및 두
  WebRTC track software gate가 존재한다.

과거 software 검증 기록이며, 현재 revision의 통과나 R 마일스톤 완료를 뜻하지
않는다. 아래 조건별 수동 gate도 별도로 남는다.

## 조건별 gate: production readiness 전 필수

1. 실제 2–4 host에서 descriptor/heartbeat, addressed control, lease/session
   expiry, stale boot 거부와 latest-only RGB-D를 검증한다.
2. SROS2 enforce가 role별 publish/subscribe를 실제로 허용·거부하고, rotation이
   교체된 generation을 revoke하는지 검증한다.
3. Jetson/GO2에서 Unitree bridge stop deadline, arm safe-hold/cleanup, deadman과
   malformed/disconnect 처리 시간을 측정한다.

## 조건별 gate: 추가 운영 환경

- L2 LAN, routed LAN/VPN, global IPv6에서 DDS 경로를 확인하고 ordinary IPv4
  NAT/CGNAT/symmetric NAT은 actionable failure를 내는지 확인한다.
- Docker Desktop Tailscale sidecar 두 node의 enrollment, namespace/address,
  route, static discovery와 bidirectional DDS를 검증한다.
- managed generation rotate, one-host failure rollback, interrupted recovery,
  SSH fingerprint pinning, non-default OpenSSH port와 Tailscale SSH re-auth를
  실제 host에서 검증한다.
- observer/hand-eye WebRTC의 direct 및 Coturn relay ICE, SDP bounds,
  DTLS/SRTP, renegotiation과 peer loss를 검증한다.
- GPU/CPU policy, NVIDIA reservation/CUDA visibility, NVENC/libx264,
  X11/WSLg owner와 Genesis Viewer를 실제 host에서 검증한다.
- source-to-consumer frame age, bandwidth, loss, p95 latency, Genesis render,
  conversion/transfer, media encode와 CPU MPC timing을 분리 측정한다.
- 실제 Look–Aim–Grasp convergence와 physical stop deadline을 검증한다.

## 보류된 후속 작업

- 생성된 typed ROS service/action의 runtime wiring. 현재 control/signaling
  surface는 protocol v6 `PeerEnvelope`다.
- perception, camera timing, MPC와 physical adapter의 property/stress coverage.
- 측정 근거가 생긴 뒤 Genesis inertia, neutral-qpos, self-collision warning
  재평가.
- bounded queue와 authority/security 경계를 보존하는 operator diagnostics.

## 변경하지 말아야 할 경계

- Coturn은 DDS를 운반하지 않고 SSH forwarding은 DDS locator가 아니다.
- `ROS_DOMAIN_ID`는 인증 또는 tenant isolation이 아니다.
- running container, exact boot heartbeat, authority/session grant, media
  readiness는 서로 다른 상태다.
- `elesim-update`는 실행 중 container를 교체하지 않는다.
- Router/ZMQ compatibility, unbounded queue, transient-local control QoS를
  복구책으로 추가하지 않는다.

## 검증 명령

```bash
elesim-dev python3 workbench/tools/quality/check.py --group required
elesim-dev python3 workbench/tools/quality/check.py --group extended
elesim-dev python3 workbench/tools/release/build.py
elesim-dev python3 workbench/tools/release/verify.py dist/releases
```

실제 gate 결과는 날짜, topology, host, interface, security profile, source
revision과 실패 범위를 함께 기록한다.
