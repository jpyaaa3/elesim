# 구현 상태와 수용시험

갱신일: 2026-11-13. 이 문서만 마일스톤, 현재 완료 범위, 미해결 항목, 수동
acceptance gate를 소유한다. 구현 불변식은 `architecture.md`, wire 계약은
`dds_contracts.md`, 운영 절차는 `setup.md`와 `deployment.md`를 따른다.

## 현재 목표: 기존 기능의 운영 경로 완결

### cuRobo 우선·GA 보충 교보재 연결 (2026-11-13)

- 새 study의 `--reverse-planner auto`는 고정된 solver 파지에서 전 구간 native
  cuRobo를 먼저 시도하고, 검색/물리 평가 실패·시간 초과 시 GA로 보충한다.
  명시한 기존 GA 자료가 있으면 원본 검증 후 사용하며, 없으면 기존 GA 검색을
  실행한다. 실행 오류나 손상된 증거는 보충으로 숨기지 않는다. 현재 기본값은
  `ga`이며 기존 study의 provenance를 수정해 실행 모드를 바꾸지 않는다.
- cuRobo 원본 후보·출처를 보존하는 교보재 어댑터와 schema-v1
  `hug-reverse-teacher` manifest를 추가했다. 원본 역경로와 독립 재생 3회의
  상태를 같은 물리 기준으로 검사하고, GA generation으로 바꿔 표시하지 않는다.
- GPU2의 독립 연구 소스 사본에서 동일한 `task` profile, 2 proposals, 300 s
  상한으로 기존 D100 세 anchor를 고정해 실제 파이프라인을 실행했다.
  **2번은 새 cuRobo 결과**, **1·3번은 명시한 기존 GA 자료**를 사용했다.
  세 케이스 모두 PPO 입력과 물리 reset bank 재구성을 통과했다. 각 검사는
  2 environments, 32 reverse bins, nominal Home, 부분 복원과 위치/속도 보존을
  검사했으며 저장 경로 대비 재생 오차는 모든 항목에서 0이었다.
- `--stop-after teachers`는 세 anchor의 위 검사까지 실행하고 학습 전에 멈춘다.
  이번 실행은 총 **620 s (10분 20초)**였고, 2번 역경로 생성·재검증은 196.4 s,
  해당 reset bank 검사는 32.9 s였다. 전체 시간에 새 GA 검색·모델 fitting·PPO
  학습은 포함되지 않는다. 신선한 GA 검색 대비 속도 향상으로 해석하지 않는다.
- focused host **57 passed**, server **67 passed**. Canonical required/extended/
  release/DDS gates는 usable generated dev attachment가 없어 실행하지 않았다.
  512-lane 학습·Home/lift/hold 학습 성공·보간 publication은 이번 증거에 포함되지
  않는다. [조건·소스 SHA·선택된 생성기·물리 검사](../workbench/evidence/curated/planning/curobo_teacher_pipeline_20261113.json),
  [실행 방법](../workbench/research/experiments/hug_pipeline/README.md#use-curobo-before-ga).

### Mapper 생성 속도·q 삼각형의 XY 투영 표시 (2026-11-10)

- 실제 `/home/user/ws/hugmap/mapper.html`에 반영했다. Construct band의 Slow/Normal/
  Fast 작업 구간을 4/9/14 ms에서 **12/24/48 ms**로 늘렸다. 기본 Fast와 0.1° 탐색
  간격은 유지하며 Stop 이벤트에 제어를 돌려준다. 화면과 상태 설명에서 A/B/C 꼭짓점
  라벨을 제거했다.
- XY는 기준점 세 개를 잇지 않고, q3-q4 삼각형 내부를 최대 0.5° 격자로 나눠 실제
  작도 함수에 넣은 영역을 채우고 경계를 그린다. 면의 꼭짓점과 중심이 μ/CAD/전개
  검사를 통과해야 하며 접촉 feature/root가 다른 면은 잇지 않는다. 별도 worker와
  선택별 표시 cache를 사용한다. 재생성 도구도 기존 embedded study를 보존한다.
- 현재 D100 RL 삼각형은 anchor root가 **+1,+1,−1**이다. 동일 q3/q4
  `(-64.5,−166.5)°`에서도 중심 `(-126.48,217.34)`와 `(-145.11,232.43)` mm가
  모두 유효하다(μ≈0.04412). 두 XY 영역은 이렇게 여러 접촉 분기가 섞인 결과다.
  q 삼각형이 하나라고 XY가 한 연속 영역이라는 뜻은 아니다. 연속 단일 영역을
  요구하려면 같은 접촉 분기·연속성 조건으로 삼각형을 선정하는 추가 작업이 필요하며,
  이번 표시 수정은 선정 알고리즘이나 학습 입력을 변경하지 않았다.
- 작도/충돌 kernel과 기존 study·기준점은 보존했다. 관련 host 회귀 **25 passed**,
  실제 headless Edge에서 q/XY 전환·worker 완료·그림과 라벨을 확인했다. 표시는 표본
  mesh이며 연속 도달/물리 성공 증명이 아니다. Canonical gates·학습/실기기 동작은
  이번 작업에서 실행하지 않았다.
  [소스 SHA·조건·분기 대조](../workbench/evidence/curated/planning/mapper_ui_20261110.json).

### Connections 버튼 갱신과 WSLg UI GPU 시작 수정 (2026-11-06)

- 주기 조회의 `checking` placeholder가 매번 `runtimeReady=false`로 바꿔 시작 버튼과
  GPU 옵션을 잠그던 것을 수정했다. 이미 검증한 같은 topology에서는 조회 중 상태를
  유지하고, 실제 host/정책 실패·topology 변경·작업 전환에서는 다시 잠근다. 최초 조회는
  여전히 검증 완료 전까지 잠근다. 같은 GPU 목록의 DOM을 재생성하지 않으며 실패한
  start의 GPU/Viewer 선택은 복원 후 편집해 재시도할 수 있다.
- 실제 manager의 최종 오류는 operator UI의 `NVIDIA UI rendering requested, found
  Mesa: llvmpipe`였다. 일반 Linux용 NVIDIA GLX 설정만 생성하고 WSLg의 `/dev/dxg`와
  WSL 라이브러리를 노출하지 않던 문제다. WSLg에서는 Mesa D3D12와 선택 index/UUID에
  대응하는 adapter 이름을 생성한다. Tools에는 외부 host의 WSLg·GPU 목록만 넘기고
  자신의 GPU 권한을 요구하지 않는다. 선택 GPU의 이름이 모호하면 등록을 거부한다.
- 수정된 생성기가 만든 Compose 설정과 수정된 graphics 소스를 기존 고정 UI 이미지에
  일회성으로 적용해 실제 `D3D12 (NVIDIA GeForce RTX 3050 Laptop GPU)` 렌더링을
  확인했다. llvmpipe 강제 대조군은 의도대로 거부했다. DDS/물리 동작은 시작하지 않았다.
- 최종 host setup 전체 **1,386 passed**, 별도 UI graphics **7 passed**. Setup이 실행하는
  Node 동작 검사는 **12 passed**로 위 setup 수에 포함된 하위 검사다. 한 차례 setup
  전체 검사에서 bootstrap 상대 import manifest 규칙 위반을 찾아 고쳤고 최종 전체를
  다시 통과했다. Host UI 전체는 `av` 부재로 collection 오류이며 의존성을 설치하지 않았다.
- 알려진 설치의 generated dev wrapper로 required gate를 시도했으나 개발 이미지 빌드가
  외부 APT 다운로드 단계에서 완료되지 않아 테스트가 시작되지 않았다. 마지막 빌드 로그만
  남았고 최종 exit 증거는 확보하지 못했다. Canonical required/extended/release build/
  isolated verify는 미완료·미실행이며 통과로 취급하지 않는다.
- **소스와 검증만 완료했으며 현재 설치·실행 중 manager에는 아직 배포하지 않았다.**
  갱신된 setup/tools 배포와 UI 이미지 갱신, instance 재등록이 활성화 경로다. 기존 image
  pin은 update만으로 바뀌지 않는다. 실제 두 host 전체 시작 성공은 추가 확인이 필요하다.
  [소스 SHA·검증 범위](../workbench/evidence/curated/planning/connections_wslg_gpu_20261106.json).

### C형 상벌점 v3 재설정과 검증 (2026-11-06)

- 사용자의 상벌점 수정 승인에 따라 `hug_c`를 보상 계약 v3로 변경했다.
  함수 목표 보상/actor 목표 관측에는 **q3/q4만** 남기고 linear/roll 목표를
  제거했다. 네 축 action, 실제 Home, C형 파지·들어올린 뒤 5초 유지 조건은
  유지한다. Actor는 20관측→4제어이며 critic에는 접촉/최고기록 등 8개 추가
  관측을 준다. 이전 v2 정책을 새 학습으로 resume하지 않도록 experiment/
  policy schema와 control mode를 v3로 바꿨다. 이전 자료는 보존한다.
- 상벌점은 정상 두 접촉의 C 파지 0.2초 확인 후 +3(한 번), 새로운 최저점
  여유 20 mm까지 +5, 새로운 연속 유지 5초까지 +10, 최종 성공 +30이다.
  들어올림/유지 최고기록은 낙하 후에도 보존하여 같은 진도를 재지급하지 않는다.
  공중 파지 후 놓침 −3/회, 제드 면 접촉 −1/초, 시간 −0.1/초, 적용 action
  제곱합 ×−0.001, 금지 충돌/한도 fault −30 후 종료, 시간 초과 −5다.
  제드 면 접촉이 섞인 **정책 스텝 전체의 양의 보상과 기록 갱신을 차단**한다.
  물체의 통상적인 XY 이동 자체에는 벌점을 추가하지 않았다.
- q 힌트 잠재값은 0~1이며 `gamma*Phi(next)-Phi(previous)`로 지급하고 종료
  잠재값은 0으로 정산한다. PPO와 shaping에 같은 **gamma=0.995**를 적용해
  물체 없이 목표 q만 맞추어도 에피소드 수익이 증가하던 허점을 막았다.
  이전 Wrap 실험의 설정은 변경하지 않았다. 각 보상 항을 독립 로그와
  수집/재생 결과에 기록한다. [상벌점 정의](../workbench/research/experiments/hug_c/README.md).
- 접촉 위치·법선을 각 링크 좌표로 변환한다. 실제 모델의 제드 쪽 축은
  node-local −Z이고 그리퍼/클로는 장착 회전을 거쳐 −X다. 양의 면 반대인
  cap/tip 접촉도 반쪽 면과 법선 일관성으로 판정한다. 실제 GPU의 box/plane
  검사가 Genesis 1.4.5 법선의 **B→A 방향**을 확인했고, 이 과정에서 초안의
  반대 부호를 발견해 바로잡았다. Roll ±90°, A/B 순서, global/local ID와
  padding의 영향을 회귀로 검사했다.
- 관련 호스트 회귀는 **81 passed / 1 skipped**(host Torch 부재)다. 서버에는
  pytest가 없어 설치하지 않았고, 실제 Torch CPU/GPU의 6환경×200스텝 보상,
  500 tick 유지 및 접촉 parser를 NumPy와 직접 비교해 통과했다. 실제 Genesis
  100 mm의 세 Home/목표 자세와 runner 생성·zero-action 보상 스텝은
  `ready=true`, `physics_tested=true`, `reward_step_tested=true`다. 이는 초기
  소프트웨어·물리 검사이며 학습 수렴/들어올리기 성공 증거가 아니다.
  원자료는 `workbench/evidence/generated/c-hug-reward-v3/`에 보관한다.
  Canonical 개발 attachment가 준비되지 않아 required/extended/release gates는
  이번 수정에서 실행하지 않았다. Host에 과학 스택은 추가 설치하지 않았다.
- 기존 실패 명령을 새 코드에서 실제 재생했을 때 500회 경로는 **−10.27점**,
  2000회 경로는 **−8.37점**이며 제드 면 접촉은 각각 259/22 physics tick
  (2.59/0.22초) 감지됐다. 파지·들어올림·유지·성공 보상은 두 경로 모두 0이다.
  선언한 synthetic 시나리오의 할인 합계는 빠른 성공 +28.85, 느린 성공 +23.20,
  목표 q만 맞춘 실패 −3.38, 제드 면 밀기 −8.08로 의도한 순서를 확인했다.
  Synthetic 순서와 기존 경로 재채점은 새 정책의 학습 성공을 뜻하지 않는다.
  [상벌점·소스 해시·검증 보고](../workbench/evidence/curated/planning/c_hug_reward_v3_20261106.json).
- 이전 v2 run은 첫 100 mm 기준점에서 **2,000회 모두 실패**하고
  `finished-anchor-failures`로 자동 종료됐다. 500/1000/1500/2000회 영상과
  policy/trajectory/verified NPZ, 재생 보고는 모두 로컬 수집됐다. 다른 직경은
  시작되지 않았다. 이 시점에는 v3 소스·검사만 적용했고 RL을 시작하지 않았다.
  Newton-Gym과 다른 작업에는 종료/재시작/설정 변경을 하지 않았다.
- 이후 사용자 `이제 서버컴에서 RL 시작하세요` 지시로 GPU2에 새 run
  `workbench/evidence/generated/c-hug-reward-v3-20261106T135818Z`를 시작했다.
  v3의 제드 면/몸체 기준으로 5개 직경 전체를 다시 정적 검사하고 통과 후보로
  입력을 재생성했다. 통과 수는 841/863, 1559/1577, 1572/1592, 1109/1140,
  670/703이다. 100 mm 세 기준점의 독립 physics preflight·보상 스텝도 통과했고,
  **512환경 실제 PPO optimizer 진행**을 확인했다. q 힌트만 주는 새 20관측/
  4action·gamma 0.995이며 이전 정책을 resume하지 않고 새로 시작했다.
  Collection PID/PGID `2313130`, 첫 study `2313139`, 첫 train `2313237`이다.
  100→125→150→175→200 mm 순차, 각 3기준점·최대2000회·500회마다 평가 영상/
  NPZ·원본 재생, 실패 시 중단하는 기존 규칙을 유지한다. Luna
  `luna_reward_v3_guard`가 60초 간격 읽기 전용 감시와 완성된 영상/NPZ 수집을
  인계받았다. 초반 측정은 약 3.8초/iteration으로, 전체 순수 학습은 각 anchor가
  500회 성공이면 약8시간, 모두2000회까지가면 약32시간이며 검증 시간이 추가된다.
  초기 로그는 성공 증거가 아니고 최종 들어올리기/보간 성공은 아직 미확인이다.

### 첫 C형 기준점의 Home 접근 검사 (2026-11-07)

- 사용자 지적으로 첫 100 mm 기준점 `c-train-000-d100`의 **Home→목표 경로**를
  별도 검사했다. 기존 `physics preflight`는 Home과 목표 각각의 정적 충돌 검사이며,
  두 자세 사이의 접근 경로를 검증하지 않았다. 통과 결과를 접근 가능성으로 해석하면 안 된다.
- 실제 Genesis에서 속도 한도를 지킨 단순 보간 경로는 0.44초에 제드 면으로 접촉하고,
  0.46초부터 원기둥을 1 mm 이상 움직인다. 5.1초 재생의 최대 XY 이동은 **333.86 mm**이고
  파지·들어올리기·유지는 실패했다. 이는 한 경로의 실패이며 모든 동작의 불가능 증명은 아니다.
- 실제 mesh의 보수적 XY convex projection으로 원기둥을 고정한 네 축 RRT를 60초간
  검사했으나 우회 경로를 찾지 못했다. seg1만 검사한 34,523셀 및 247,860셀 격자 모두
  Home과 목표가 서로 다른 두 자유공간 성분에 속했다. q4는 seg1의 장애물을 해소할 수 없다.
  유한 격자·탐색과 보수적 형상 근사이므로 연속 공간의 불가능성을 증명한 결과는 아니다.
- **현재 첫 anchor는 Home 접근이 검증되지 않은 대상**이다. 이 결과를 RL 성능 문제만으로
  해석하지 않으며, 다른 기준점·직경으로 일반화하지 않는다. 다음 선정에는 허용 면 접촉을
  지키는 실제 Home 접근 경로 확인이 필요하다. 동적 접촉으로 원기둥을 옮기는 전략의
  도달 가능성도 이번 고정 물체 기하 검사로 판정하지 않았다.
- 독립 검사 모듈은 `research/analysis/`에 추가하여 실행 중 `hug_c` 소스 해시를 보존했다.
  학습/경비와 Newton-Gym을 변경하지 않았다. 원자료와 MP4는 기존 v3 run 아래
  `home-direct-approach/`에 수집했다.
  [검사 범위·수치·증거 해시](../workbench/evidence/curated/planning/c_hug_home_approach_20261107.json).
- 이후 사용자 지시로 같은 배치에서 실제 UI `Extend Arm` 목표만 재생했다. 프리셋은
  linear −10 mm, roll 0°, q3/q4 0°이며, RL 제어 속도 한도로 2.2초 이동 후 1초
  유지했다. 0.48초에 **gripper_base가 4.39 N으로 최초 접촉**하며 제드 면 접촉도
  감지됐고, 원기둥 최대 XY 이동은 **105.55 mm**다. 따라서 이 기준점은
  Home→Extend Arm 무접촉 준비 절차에서 제외해야 한다. 모든 접근 전략의 불가능성을
  증명한 결과는 아니며, 학습 소스/프로세스는 변경하지 않았다. 독립 MP4·명령 NPZ와
  보고는 기존 run의 `home-extend-arm/`에 수집했다.
  [Extend Arm 측정 결과·재현 명령](../workbench/evidence/curated/planning/c_hug_extend_arm_20261107.json).

### Standalone mapper 전개 충돌 기준 재계산 (2026-11-07)

- 사용자 지시에 따라 `/home/user/ws/hugmap/mapper.html`에 실제 Home→Extend Arm
  전개 충돌 envelope를 추가했다. 35 mm는 눕힌 노드의 반폭이며, roll 0°에서
  전개하는 노드의 실제 반폭은 **44 mm**다. 최종 파지의 평면 충돌만으로 전개
  가능성을 판단하던 누락을 보완한다. 기존 최종 자세의 1 mm 접촉 근사 허용과
  보호 부품 간격은 `finalObjectCollision`에 유지한다.
- 실제 URDF의 44개 충돌 형상(카메라 visual을 RL처럼 collision으로 포함)과
  앱 Home/Extend Arm 매핑, 고정 Go2 다리 자세로 340구간을 계산했다. 부품별
  전개 XY convex hull에 물체 반지름과 **1 mm + 0.04268 mm**를 적용한다.
  0.04268 mm는 실제 체인 길이·총 관절 변화에서 얻은 표본 사이 곡률 보정이다.
  외접 primitive 및 부품별 convex hull은 보수적이며 모든 접근의 불가능 판정이 아니다.
- XY와 q 화면, 역함수 픽셀 보완, RL 함수 export가 같은 합성 판정을 사용한다.
  회색 설명은 최종 자세와 Home/Extend Arm envelope를 구분하고 부품·signed gap을
  표시한다. 제외 영역 체크박스는 표시만 바꾸며 판정을 끄지 않는다. 기존 DATA는
  보존했고 출력은 계속 외부 라이브러리 없는 HTML 한 파일이다. 원본을 generated에
  백업한 뒤 검증된 파일 SHA를 확인하여 지정 경로에 적용했다.
- 관련 host 회귀 **19 passed**: 기존 최종 기하/허용 오차 유지, 실패 기준점 제외,
  정상 대조군·지름 효과, 독립 FK의 표본 사이 포함, UI 프리셋 일치, XY/q/픽셀 및
  RL forward/inverse 일치와 재생성 멱등성을 검사했다. 150 mm 두 밴드의 각
  6,481,800각도 전수 scan·픽셀 보완도 invalid/lost=0이다. 이 scan은 Node Canvas
  stub이며 실제 브라우저 rasterization 검사는 아니다. 브라우저가 설치되지 않아
  실제 브라우저 검사는 실행하지 않았다. Canonical 개발 attachment/gates도 미실행이다.
- 독립 GPU 물리 대조군 `(x,y)=(-113.313,177.586) mm, D=100 mm`은 3.2초
  Home→Extend Arm 재생에서 물체 접촉/제드 면 접촉/fault가 없었다. 기존 실패
  기준점은 새 기준에서 제외되며 실제로는 105.55 mm 밀림을 기록했다. 둘 다 이후
  roll·감싸기·들어올리기를 검사한 것은 아니다. 기존 RL 자료를 새 기준으로 재선정하거나
  새 학습을 시작하지 않았다. 실행 중이던 v3는 이후 자체 2000회 실패 게이트로
  `finished-anchor-failures` 종료했고 다른 직경은 시작되지 않았다. Newton-Gym은 보존했다.
- [재생성·가정·기준](research.md#mapper의-homeextend-arm-충돌-기준),
  [형상·소스 SHA·검증 보고](../workbench/evidence/curated/planning/mapper_collision_20261107.json).

### RL 삼각형을 전개 가능 영역에서 재선정 (2026-11-07)

- 사용자 지시로 C형 RL 기준점 선정을 `preparation-clear-area-v2` 계약으로 변경했다.
  목적은 계속 **새 유효 영역과 삼각형의 교집합 면적 최대화**다. 유효 영역은 최종
  자세 CAD와 실제 mapper의 Home→Extend Arm envelope를 모두 통과해야 한다.
  면적 동률이면 최소 전개 여유, 최악 μ, 삼각형 조건 순으로 선택한다. 삼각형 안의
  제외 영역은 허용하지만 train/validation/test와 정확 XY 역함수에서는 매번 제거한다.
- 전개 envelope가 없는 옛 mapper, 다른 모델/프리셋, 옛 삼각형 collection은 거부한다.
  모든 후보/기준점/held-out 점에 전개 통과 여부와 여유를 저장한다. 기존 512환경,
  Home 출발 네 축 자유 제어, q3/q4 힌트와 보상 v3, 5초 들어올림 유지·NPZ 경로
  보간 계약은 유지한다. **정책에 Extend Arm이나 회전 시간표를 강제하지 않는다.**
- 새 5직경 입력과 그림은
  `workbench/evidence/generated/c-hug-ready-triangles-20261107/index.html`이다.
  먼저 전개 envelope를 통과한 4,204개 후보 전체를 실제 GPU에서 Home/목표 검사하고,
  100/125/150/175/200 mm에서 584/1211/1196/764/399개 통과 q 셀 중
  57.99/72.83/74.50/67.81/64.65%를 덮는 세 기준점을 각각 선정했다. μ≤0.4,
  총각 1° grid이며 연속 전역 최적성·성공률을 뜻하지 않는다. 총 15 train,
  150 validation, 500 test를 생성했고 예시 pilot도 새 D150 삼각형으로 갱신했다.
- 학습 전 실제 physics check는 이제 각 기준점에서 Home 유지, Home→Extend Arm
  전개와 1초 settle, 별도로 직접 놓은 목표 자세를 검사한다. 전개 중 한 번이라도
  로봇/물체 접촉, fault, XY 1 mm 초과 이동이 있으면 실패다. 과거 정적 ready 보고만으로
  `train`을 시작할 수 없다. `collection check`로 다섯 직경을 학습 없이 검사할 수 있다.
  이후 roll·감싸기·들어올리기 전체 경로는 이번 준비 검사로 보증하지 않는다.
- 전개 envelope만으로 고른 첫 삼각형은 150/175/200 mm 각 한 점에서 실제
  Home/목표 충돌 또는 물체 이동으로 실패했다. 실패 원자료를 보존하고, 후보 전체
  물리 검사를 **삼각형 선정 전에** 적용하도록 `collection screen`과
  `prepare --catalog-preflights` 경로를 연결했다. 단순히 실패 꼭짓점만 삭제하지 않는다.
- 관련 host 회귀 **67 passed**이며 실제 GPU2의 재선정한 **15개 기준점 전부**
  Home·전개·직접 배치 목표 검사를 통과했다. 전개 중 물체 접촉 tick은 모두 0이며,
  최대 XY 이동은 0.552 mm(기준 ≤1 mm), Home/목표 fault·제드 면 위반도 없다. 소스 hash는
  `318dd14f73fef42e9e46837a4efc0282f397c7fff57b1572a1b7ec94c7002d69`다.
  상세 결과와 model/input SHA는 아래 선별 보고에 기록한다. 새 학습·optimizer
  update는 실행하지 않았다. Newton-Gym은 보존했고 이전 실패 자료는 변경하지 않았다.
  Canonical 개발 attachment/gates는 준비된 환경 부재로 미실행이다.
  [선정·검증 보고](../workbench/evidence/curated/planning/c_hug_prepared_triangles_20261107.json),
  [사용법](../workbench/research/experiments/hug_c/README.md).
- 이후 사용자 시작 지시로 같은 검증 소스의 새 collection을 GPU2에서 실행했다.
  첫 D100 study가 자체 전개 사전 검사도 통과했고 **512환경 PPO 32회 이상** 및
  `model_0.pt` 생성을 확인했다. Collection PID/PGID `2329235`, study `2329239`,
  첫 train `2329369`다. 새 삼각형의 첫 기준점은 q3/q4 `(-89.5,-162.5)°`이며,
  이전 실패 policy를 resume하지 않았다. 100→125→150→175→200 mm 순차,
  각 3기준점·최대2000회·500회마다 MP4/NPZ·3회 원본 재생과 실패 중단 규칙을
  유지한다. 출력은 `workbench/evidence/generated/c-hug-ready-triangles-20261107`다.
  Luna `luna_prepared_triangle_guard`에 읽기 전용 감시와 완성 영상/NPZ 수집을
  인계했다. 초기 optimizer 진행은 파지·들어올림 성공 증거가 아니다.
  [실행·PID·검증 기록](../workbench/evidence/curated/planning/c_hug_prepared_launch_20261107.json).
- 이 실행은 첫 새 D100 기준점에서 **2,000회 평가도 실패**하여
  `2026-11-07 04:07 KST`에 `finished-anchor-failures`로 자동 종료됐다. 시작부터
  약 2시간 9분이며 나머지 기준점·직경은 학습하지 않았다. 500/1000/1500/2000회
  평가 모두 파지·들어올림·유지 보상 0이며 1000회에는 fault도 발생했다. 마지막은
  fault 없이 linear −173.33 mm, roll +90.10°, q3/q4 약 −180/+180°의 접은
  자세로 시간 초과했다. 영상과 명령값상 물체를 잡는 동작을 얻지 못했으며, 이를
  기하적 불가능성으로 해석하지 않는다. 네 평가의 MP4·policy/trajectory/verified
  NPZ·report/replay 24개를 Luna가 로컬에 수집하고 원격/로컬 SHA를 확인했다.
  현재 owned collection/study/train 프로세스는 없고 새 재학습은 실행하지 않았다.
  [최종 평가 기록](../workbench/evidence/curated/planning/c_hug_prepared_result_20261107.json).
- 사용자 영상 원인 분석 요청으로 기존 policy를 별도 1환경에서 다시 재생했다.
  2000회 정책은 **9.1초부터 네 명령 한계에 포화**되고 마지막 raw action 평균은
  `[-0.871,+1.030,-0.766,+0.702]`로 모두 한계 바깥을 향한다. 적용 증분은 네 축
  모두 0이며 이 재생에서 proximal/distal 물체 접촉 tick은 0이다. 직접적인 정지는
  물체에 걸린 것보다 정책의 한계 방향 요청이 clip되는 현상으로 확인됐다.
  네 저장 평가 모두 q4가 양수여서 목표 C형 조건에 진입하지 않았다.
- 학습 로그 2000개 출력에서 파지·들기·유지 보상은 모두 0, 표시 action std는
  0.5→0.08이다. 단, 이 출력은 반올림된 batch 요약이다. 종료 potential을 0으로
  정산하는 q shaping의 할인 합계는 실제 네 평가 모두 약 −0.3280404로 같았다.
  이 shaping은 단기 진행 신호를 줄 수 있으나 정답 모방을 구현한 것은 아니다.
  행동 벌점은 한계 clip 뒤 적용 증분에 붙으므로 한계 바깥 요청을 계속해도 그 항은
  0이다. 희박한 양성 경험·동등한 실패 q 수익·값싼 포화 행동의 조합이 안전한 실패
  정책으로 굳은 원인으로 보이며, 각 요인의 인과 기여를 분리한 재학습은 하지 않았다.
  훈련 소스/설정과 Newton-Gym은 변경하지 않았다.
  [정책 출력·접촉·보상 항등식 측정](../workbench/evidence/curated/planning/c_hug_policy_stall_20261107.json).

### 함수 기반 C형 Home→들어올리기·q 보간 실험 — 이전 v2 이력 (2026-11-06)

- `workbench/research/experiments/hug_c`에 새 외부 실험을 추가했다. 기존
  `hug_interpolation` 두-band 실험과 그 실제 결과를 보존한다. payload에는
  원기둥 전용 기능이나 새 역할을 추가하지 않는다. 실행법은
  [`hug_c/README.md`](../workbench/research/experiments/hug_c/README.md)다.
- `/home/user/ws/hugmap/mapper.html`의 작도 함수와 CAD 검사 함수를 추출해
  실행한다. HTML의 `DATA` 점군이나 map.json은 케이스 선정에 사용하지 않는다.
  사용자 선택에 따라 **100/125/150/175/200 mm를 직경별 독립 수집**한다.
  μ_min≤0.4, 1° 총 굽힘각 raster에서 유효 셀은 각각 863/1577/1592/1140/703개,
  선택한 q 삼각형의 영역 커버율은 약 72.87/86.18/88.49/79.72/77.29%다.
  직경마다 3 train / 30 validation / 100 held-out test를 실제 함수로 생성했다.
  합계 15개 맨땅 RL 기준점이며 직경을 섞어 학습하는 구성이 아니다. 전역 최적
  증명이나 동작 성공률이 아니며, 함수 영역과 삼각형의 교집합 면적을 최적화한다.
  준비 자료는 `workbench/evidence/generated/c-hug-four-dof/diameters/collection.json`
  및 각 직경 폴더의 `triangle.html`, `pilot.json`, `catalog.json`이다.
- 모든 episode는 실제 Home preset에서 시작한다. 현재 UI `(180,180,10,10)`을
  실제 매핑으로 변환한 값은 linear −120 mm, roll 0°, q3 −170°, q4 +170°다.
  q3/q4는 세그먼트 총 굽힘각이며 Sim/NPZ 경계에서만 노드당 rad로 변환한다.
  **Go2만 고정하고 linear·roll·q3·q4를 정책이 첫 action부터 조절**한다.
  전개·회전·들어올림 시점을 강제하는 시간표를 제거했다. 함수 목표 q는 관측과
  목표 진행 보상에 남기며, 실제 두 역할의 C 접촉 이후 목표 유도 보상을 해제해
  들어올리기에 자세를 바꿀 수 있게 했다. Home의 양수 q4는 유효하다.
- 성공은 **5초 연속** 원기둥 최저점의 지면 여유 ≥20 mm, 지면 접촉 없음,
  그리퍼 상대 미끄러짐 ≤10 mm, 두 접촉 역할 유지율 ≥80%, C형 유지와
  금지 충돌/한도 fault 없음이다. 그리퍼 포즈에는 local link ID를 사용하고
  접촉 쌍에는 global ID를 사용한다. 낙하·과도한 미끄러짐·C형 이탈은 유지 시간을
  초기화한다. 20초 episode 내 시작 시점과 최종 linear/roll은 자유이며, 들어올림은
  필수다. 5초는 본 실험의 기본 설정이다. 금지 충돌 이후 양의 shaping도 차단한다.
- 일치하는 실제 physics preflight가 있어야 train이 가능하다. **512병렬**/최대
  2,000회/500회 평가가 기본이며, 각 원본의 정책 수집과 명령 경로 3회 재생이
  통과해야 다음 anchor와 q 보간으로 간다. MP4와 policy/trajectory NPZ를 저장한다.
  30점 검증 후 그중 5점에 ±5/±10 mm 배치 오차 80회를 추가 평가하고, 통과하면
  100점 최종 시험을 실행한다. 실패 점에 자동으로 추가 RL을 돌리지 않는다.
  q 가중치로 네 채널 전체 경로를 보간하며, 짧은 원본을 가장 긴 원본 시간으로
  늘린다. 기준점별 수집/재생 성공이 없으면 보간·정책 선택으로 넘어가지 않는다.
- `collection`은 `(x,y,D)`에서 정확히 일치하는 직경을 찾고 mapper 역함수와 CAD/
  μ/삼각형 조건을 검사하여 3개 policy/trajectory NPZ와 q 가중치를 선택한다.
  직경별 완료된 검증·배치 오차·시험과 원본 3회 재생을 확인한다. 없는 D를 가까운
  직경으로 대체하지 않으며 직경 간 보간은 아직 없다. 110 mm 같은 새 직경은
  입력과 study만 추가하므로 기존 5개 직경의 NPZ·프로필을 변경하지 않는다.
- 호스트 관련 신·구 실험 회귀 **64 passed / 1 skipped**(Torch 부재)다.
  C형 전용 두 파일의 **32 passed**는 Home/네 축 제어·총각 변환·5초 유지와
  초기화·면적 목적·함수 전용 분할·경로/NPZ·preflight·orchestration·직경 추가
  보존·정책 선택 게이트를 검사한다. 실제 다섯 직경의 함수점에서 XY 역산도
  확인했으며 동적 성공 증거는 아니다. 호스트에는 Torch/Genesis/tensordict/rsl_rl이
  없어 `check --physics`는 종료 2, `physics_tested=false`를 기록했다.
  코드 준비 단계에서는 수정된 방식의 GPU 실험을 시작하지 않았고, 이후 아래의
  명시적 시작 지시로 실행했다. 들어올리기 성공과 경로 보간 성공은 아직 미확인이다.
  Newton-Gym은 건드리지 않았다. Canonical required/extended/release gate도 이번 작업에서
  실행하지 않았다. 준비된 canonical 개발 환경이 없으며 현재 attachment 빌드
  미완료 상황은 상단의 설치 검증 기록을 따른다.
  [현재 코드·모델 해시와 직경별 준비 요약](../workbench/evidence/curated/planning/c_hug_four_dof_20261106.json).
- 이후 사용자 `학습 시작. Luna한테 맡기고 자러가세요` 지시로 새 collection
  `workbench/evidence/generated/c-hug-four-dof-20261106T080153Z`를 GPU2에서 실행했다.
  다섯 직경 전체의 Home/목표 정적 검사에서 통과한 후보는 각각
  **841/863, 1559/1577, 1572/1592, 1109/1140, 670/703**개다. 이 후보로 q 삼각형과
  분할을 다시 생성했으며 이전 함수 전용 입력과 이전 run은 보존했다.
  첫 100 mm의 세 기준점 physics preflight도 ready=true이며 실제 PPO update와
  actor 22관측→4제어, critic 24관측을 확인했다. Collection PID/PGID는 `2284822`,
  첫 study PID는 `2284831`이다. 512환경, 기준점당 최대 2000회, 500회마다 자동
  MP4/NPZ·원본 재생 평가를 수행한다. 100→125→150→175→200 mm 순서이고 기존
  실패 중단 게이트를 유지한다. `.git/c-hug-active-run.json`은 새 collection을
  가리킨다. Luna `luna_four_dof_guard`가 읽기 전용 감시와 완성된 보고 영상/NPZ
  수집을 인계받았다. 서버 Torch의 네 축 증분·500 tick 유지 계산도 통과했다.
- 이전에는 사용자 승인으로 서버 `hckang@100.74.222.24:~/elesim`의 GPU2에서 실험을
  실행했다. 최초 run `c-hug-rl-20261106T060616Z`는 세 번째 꼭짓점의 Home/목표
  충돌로 사전 점검에서 중단했고 학습하지 않았다. 실패 결과와 당시 소스를 보존했다.
  `hug_c.catalog_preflight`로 함수 후보 1,592개를 64병렬/자세당 10 tick씩 검사해
  Home·목표 모두 fault 없는 1,572개를 확인했다. 검사 모델·물리 설정과 source
  kernel digest가 일치해야 재선정에 쓸 수 있다. q 삼각형을 이 후보들로 다시
  선정한 뒤 자체 3점 physics preflight도 ready=true를 확인했다.
- 중단된 이전 실행은 `workbench/evidence/generated/c-hug-rl-20261106T061027Z`다.
  GPU2 UUID `GPU-a40782b2-881c-e285-2a8e-66cae9162a01`, study PID `2077627`,
  첫 256병렬 PPO PID `2078254`이며 25회 이상 실제 optimizer 진행과 `model_0.pt`
  생성을 확인했다. 재선정 꼭짓점은 `(-104.5,-129.5)`, `(-0.5,-136.5)`,
  `(-1.5,-163.5)`도, 정적 통과 영역의 면적 커버율은 약 88.826%다. 이 수치는
  동작 성공률이 아니다. 포인터는 서버 `.git/c-hug-active-run.json`이다.
- 초기 비교용 `baseline/baseline.mp4`와 경로 NPZ를 저장했다. 학습하지 않은
  기준 동작은 fault=false지만 지면 접촉/유지 실패로 success=false다. 원기둥을
  실제 들어올리는 성공은 아직 확인하지 않았다. 500회마다 정책 평가 MP4/NPZ와
  원본 3회 재생을 자동 저장했고, 당시 사용자 지시로 Luna가 읽기 전용 감시했다.
  중간 상태·초기 사전 점검·영상은 로컬의 같은 generated 경로에도 보관한다.
- 이전 256환경 학습 중 GPU2를 1초 간격으로 30회 측정했다. 사용률 평균 65.63%,
  중앙값 69%, 범위 19~73%, 최대 메모리 3,175/49,140 MiB, 평균 전력 130.65 W다.
  최근 20 iteration 평균은 2.7765초였다. 사용자 지시에 따라 **다음 새 GPU2
  study는 512환경**으로 시작하는 프리셋을 `hug_c/gpu2-next-study.json` 및 서버
  `.git/c-hug-next-run.json`에 기록했다. 이후 이전 실행은 중단했고 새 시작은 미지시다.
  512/1024환경 처리량과 수렴 속도는 아직 측정하지 않았다.
- 이후 사용자 지적으로 이 run의 고정 전개 계약을 재검토했다. Home 출발에서
  linear/roll을 2초 시간표로 강제하면서 정책에는 q3/q4 보정만 허용하여,
  필요한 전개·회전 순서를 선택할 수 없었다. 이를 학습 부족으로 간주하고
  계속 돌리는 것은 부적절하여 해당 소유 process group `2077627`만 SIGTERM했다.
  상태는 `stopped-invalid-approach-contract`이며 체크포인트·500/1000회 평가 영상과
  원자료는 보존했다. 계약을 수정한 뒤에도 새 시작 지시 전에는 재시작하지 않는다.
  이 결과는 C형 끌어안기나 들어올리기 자체의 불가능성 증거가 아니다.

### 두 밴드 Hug RL의 첫 GPU 실행 준비와 보고 영상 (2026-11-06)

- 사용자 승인으로 `hckang@100.74.222.24`의 `/home/hckang/elesim`에서 물리 GPU
  2(A6000, UUID `GPU-a40782b2-881c-e285-2a8e-66cae9162a01`) 전용 실행을 시작했다.
  기존 Conda 환경을 보존하고 `.venv/hug-rl-20261106`에 Genesis 1.4.5,
  Quadrants 1.3.3을 격리했다. Torch 2.12.1, rsl-rl-lib 5.4.0,
  tensordict 0.14.2를 사용하며 `pip check`가 통과했다.
- 실제 GPU 초기 물리 검사에서 대표 6개 모두 초기 fault가 없었고 FK와
  PPO/environment interface 검사가 통과했다. 비파지 대조군의 외란 뒤 이동은
  50.828 mm로, 10 mm hold 판정과 구별됐다. 이는 초기 구성의 검사이며 학습
  수렴이나 파지 성공의 증거가 아니다.
- `study`는 별도 32환경/20회 smoke 후 대표 6개를 각각 256환경/2,000회로
  순차 실행한다. 500회마다 NPZ 정책·경로와 25 fps 탑뷰 MP4, 원본 재생 수치를
  저장한다. 성공한 원본 6개가 필요하며, validation 실패 시 test를 열지 않는다.
  `collect`/`replay --video`는 한 평가 환경에만 카메라를 만들고 physics tick에
  프레임을 채집한다. 실패한 평가도 보존한다.
- 실제 Torch 내보내기 probe에서 공유 ELU 인스턴스를 `modules()`가 중복 제거하는
  문제를 찾아 Sequential의 위치 순서를 직접 검사하도록 수정했다. 실제 RSL
  MLP의 Torch/NumPy 최대 오차는 `7.45e-8`이다. RSL 5.4의 resume은 마지막 완료
  iteration 다음부터 이어가도록 보정했다.
- 호스트 관련 회귀는 **51 passed / 1 skipped**(호스트 Torch 부재)다. 실제 Torch
  내보내기는 위 서버 CPU probe로 별도 확인했다. canonical required/extended와
  release gates는 이번 실험에서 실행하지 않았다. 학습/영상/보간 검증 결과는
  실행 산출물로 확인해야 하며 이 준비 결과를 성공률로 취급하지 않는다.
- 원자료 루트는 서버
  `workbench/evidence/generated/hug-rl-20261106-005955/attempt-02/`다.
  `status.json`, `study.log`, `logs/`, `reports/`에 단계·명령·중간 결과를 남긴다.
  이전 준비 시도와 교체 이유도 상위 디렉터리에 보존한다. 기존 설치·다른 GPU의
  작업·물리 Robot은 조작하지 않았다.
- 32환경/20회 smoke의 PPO 실행·정책 NPZ 내보내기·수집·원본 재생과 MP4 저장이
  실제 완료됐다. smoke는 fault=false지만 목표 각도 오차 0.06532 rad,
  hold 이동 20.847 mm로 성공 기준을 충족하지 않았다. 수집과 원본 경로 재생의
  판정·측정값은 일치했다. 영상은 H.264 960×720, 25 fps, 101 frame/4.04초로
  파일과 실제 장면을 확인했다. 이후 첫 대표 256환경 학습 PID `1948093`에서
  PPO iteration 1~14가 진행되는 것을 확인했다. 사용자 요청에 따라 Luna
  subagent가 프로세스·로그·체크포인트·영상 저장을 읽기 전용으로 감시한다.
- 최종 실행은 2026-11-06 09:25:52 KST에 `finished-anchor-failures`로 종료했다.
  실제 study 경과는 29,724.701초(약 8시간 15분 25초), 대표 6개 각각 2,000회다.
  밴드 1의 대표 3개는 각각 성공한 수집·원본 재생 체크포인트를 확보했고 선택된
  학습 횟수는 `b1-x-100-y320-d150:2000`, `b1-x-100-y340-d150:500`,
  `b1-x-80-y320-d150:1500`이다. 밴드 2의 대표 3개는 500/1000/1500/2000
  네 체크포인트 모두 실패했다. 이는 그 실험 조건의 학습·평가 결과이며 기하학적
  불가능의 증명이 아니다. 6개 성공 원본 조건이 충족되지 않아 보간/validation/test는
  실행하지 않았다. 들어올리기는 이번 MDP/판정에 포함되지 않는다.
- MP4는 smoke 1개와 대표별 4개씩 총 **25개**다. 최종 상태·24개 checkpoint
  평가·영상 25개·성공 원본 3개 NPZ와 선택된 정책 NPZ는 로컬의 같은 generated
  상대 경로에도 수신했다. 학습 프로세스와 GPU2 실험 프로세스가 종료됐고 Luna
  감시도 완료됐다. 실행 오류·NaN·영상 저장 누락은 발견하지 않았다.

newhug 세션 인계: [의도·확정 조건·현재 탐색 방식·다음 질문](newhug-handoff.md).
현재 마지막 쟁점은 원형 수집의 적은 해가 탐색 누락인지 실제 제약인지 구분하는 것이다.
적응형 국소 탐색/해 가지 추적은 아직 구현하지 않았다.

### hugmap 단일 JSON 지름 테이블·재개·원형 theta 표시 (2026-11-03)

0.4.0 GUI는 기본 지름 50~250 mm/5 mm의 41개 원형 맵을 schema v2
`hugmap-table` JSON 한 파일에 저장한다. 기본 위치 20 mm이면 총 41,246개
격자다. 크기별 상태와 결과를 `maps`에 넣고 공통 설정/형상은 metadata에 둔다.
지름 범위/간격과 x/y/theta 해상도·μ를 입력한다. theta 항목은 원형에서도 열어 두며,
값을 저장하되 원형 솔브는 theta=0 한 번만 실행한다.

`New table`/`Resume table`, CLI `--table`/`--resume`을 추가했다. 하나의 worker pool에
반지름을 명시적으로 전달한다. 완료 좌표는 건너뛰고 일부 저장된 고정 작업 묶음의
앞선 해를 warm start로 연결한다. 10초/크기 완료/중지 시 원자적으로 체크포인트를
교체한다. 동시 쓰기는 OS 잠금용 `.lock` 보조 파일로 막고, 프로세스 종료 시 잠금은
해제된다. 모델·제약·solver revision이 다른 파일은 재개하지 않는다. 테이블 결과는
메모리에 모아 저장하므로 고해상도에서는 메모리 사용도 커진다.

Reader는 지름 선택과 파일 전체 공통 색 범위를 지원하며 기존 schema v1도 읽는다.
원형 theta=0 데이터는 reader에서 저장된 theta 간격의 `[-180,180)` 층으로 복제한다.
기본 5도는 72층이다. 기존 100 mm 맵의 22개 해는 1,584개 표시 점으로 확장되지만
저장된 결과와 발견 개수는 유지한다. 비원형에는 대칭 복제를 하지 않는다. 원본
계산과 별도의 reader 기능이며, 1,000,000개를 넘는 표시 확장은 오류로 안내한다.

호스트 기존 18개 회귀 통과 후 테이블/reader 집중 검사 **17 passed**:
41개 맵 단일 파일, 크기별 worker 반지름, 취소·재개와 설정 보존, 부분 작업 묶음
재개/연속 실행 일치, 완료 좌표 건너뛰기, writer 잠금과 원자 저장 실패 시 원본 보존,
불일치 metadata 거부, 단일 파일 외부 실행 CLI 및 원형 대칭 확장을 확인했다.
테스트의 41개 맵 생성은 거친 격자를 사용했으며 실제 기본 41,246개 전체 테이블은
이번 코드 수정에서 생성하지 않았다. 이전 (20,360,0) 탐색 누락은 수정하지 않았다.
실제 Tk GUI 조작은 DISPLAY 제한으로 미검증이며 canonical wrapper도 없어
required/extended/release gates는 미실행이다.

### hugmap 가속 전후 전체 지도 비교: 해 1개 누락 확인 (2026-11-03)

사용자가 제공한 `/home/user/ws/hugmap-20261103-190421.json`(0.1.0, complete)을
가속 후 `workbench/evidence/generated/hugmap-speed-20261103/map-20mm-v03.json`과
좌표로 대조했다. `/home/user/ws`의 나머지 JSON은 160 mm newhug 결과이므로
같은 실험이 아니며 비교에 사용하지 않았다. config/object/coordinates/constraints/
geometry와 1,006개 좌표 집합은 동일하다.

전 23개, 후 22개 발견. 공통 발견 22개는 접촉 부위가 전부 같고 최대 차이는
seg1 1.67e-7도, seg2 2.15e-7도, mu_min 2.83e-9 미만이다. 제외 102개와 공통
미발견 881개도 일치한다. 단 `(20 mm,360 mm,0 deg)`는 전 found / 후 not_found다.
전 해는 seg1=37.3854153518461도, seg2=-172.63589484977487도,
mu_min=0.13024283882871812이며 현재 최종 판정기도 통과한다. 현재 cold solve도
not_found를 재현했다. 유효 해의 탐색 누락으로 확인했으며 이번 비교에서 수정하지 않았다.
따라서 이전 표본 비교를 전체 결과의 동등성으로 확대하면 안 된다.

파일에 기록된 실행 시간은 1019.48초/95.16초(약 10.71배)이며 실행 당시 부하는
동일하게 통제한 실험이 아니다. 상세 대조는 같은 generated 디렉터리의
`full-before-after-comparison.json`에 기록했다.

### hugmap 3D reader 수정 (2026-11-03)

0.3.1에서 사용자 요구를 바로잡아 theta 단면 선택을 제거했다. X/Y/Theta를
3개 공간축으로 두고 전체 theta의 격자 점을 함께 표시하며 seg1/seg2/mu_min을
색으로 선택한다. 원형 데이터는 theta=0 평면에 놓인다. Tk/NumPy 직교 3D 투영,
깊이 버퍼와 picking, 드래그 회전·휠 확대·시점 초기화를 추가했다. X/Y는 공통
mm 축척, theta는 독립 표시 축척이다. 기본은 발견 해, 선택적으로 미발견/제외도
표시한다. 추가 의존성은 없다. 계산 솔버는 변경하지 않았다.

reader 집중 검사 **8 passed**: 기존 데이터/색/입력 검사에 더해 3D 투영의
3축 보존, 카메라 회전·확대, 여러 theta 동시 raster, 깊이 버퍼 picking·clip을
검사했다. 실제 GUI 창 조작은 기존 DISPLAY :0 연결 제한으로 미검증이다.

### hugmap CPU 배치 솔버 가속 (2026-11-03)

0.3.0에서 배치별 SciPy 다중 솔브/수치 미분을 NumPy 일괄 계산으로 교체했다.
64개 초기값의 polygon 거리·FK·해석적 Jacobian을 함께 계산하며, 초기값별로
독립된 감쇠/수락 상태와 관절 bounds/step cap을 갖는 damped Gauss-Newton을 쓴다.
최종 유효성 판정은 기존 `inspect_pose`의 접촉·충돌·μ 컷오프를 그대로 사용한다.
유효 후보는 기존 seed 순서로 최대 16개 보존한다. 일괄 실행의 `attempts`는
실제 평가한 전체 초기값 수(64)를 표시한다. SciPy 의존성을 제거했고 NumPy와
GUI용 Tkinter만 필요하다. GPU 경로와 비원형 접촉 솔버는 구현하지 않았다.

초기 12개 표본: 기존 121.86초, 시제품 4.35초. 다른 정답군을 포함한 추가 12개
표본(일부 중복): 기존 92.42초, 신규 4.72초(19.57배); process CPU 합계는
81.97초/4.22초(19.42배). 추가 표본의 발견 여부는 12/12 일치, 발견 해의 최소 μ
최대 차이는 8.09e-10 미만이다. 이것은 전체 해공간의 동등성/완전성 증명이 아니다.

기본 20 mm / 원 100 mm / μ 0.5 / 7프로세스의 전체 지도는 **95.16초**에 완료됐다.
1,006개 고유 좌표, 보호 형상 제외 102개, 수치 풀이 904개 중 발견 22개/미발견
882개다. 미발견은 불가능 판정이 아니다. 발견 22개는 기존 판정기로 재검사했고
reader도 전체 JSON을 정상적으로 읽었다. 이전 솔버의 전체 지도는 재실행하지
않았으므로 전체 지도 가속 배율을 주장하지 않는다.

호스트 기존 14개 검사와 추가 배치 기하/해석 미분 검사 1개, 총 **15개 통과**.
거리·최근접점은 원래 scalar 기하와, Jacobian은 중앙차분과 비교했다. 단일 파일
분리 실행, 순차/병렬 결과 일치, 취소/JSON, reader 검사도 포함된다. 실제 GUI
표시는 DISPLAY 문제로 여전히 미검증이며 canonical wrapper도 없으므로 전체
required/extended/release gates는 미실행이다.
근거: `workbench/evidence/generated/hugmap-speed-20261103/`의 `comparison.json`,
`initial-raw.json`, `extended-raw.json`, `reference-source.txt`, `map-20mm-v03.json`.

### hugmap JSON heatmap reader (2026-11-03)

단일 파일 0.2.0에 `Open JSON reader`와 `--read map.json`을 추가했다.
물체 `(x,y,theta)`의 theta 단면별 XY 히트맵에서 seg1/seg2/mu_min을 선택한다.
파일 전체 공통 색 범위, signed seg 각도, hover 수치, 미발견/제외/미저장 구분을
구현했다. 보간은 하지 않는다. Tk raster를 사용하고 추가 plotting 의존성은 없다.
영어/ASCII UI를 유지한다. schema·중복 좌표·NaN·격자 해상도를 검증한다.

호스트 focused reader 검사 **5 passed**: theta별 조회와 공통 색 범위, 빈/상수 지도,
세 가지 무효/미저장 상태, 잘못된 입력 거부, ASCII 및 CLI 옵션을 확인했다.
기존 solver의 전체 탐색을 반복하지 않았다. Tk DISPLAY :0 연결 불가로 실제
화면 렌더·마우스 조작은 미검증이다. canonical dev wrapper 부재에 따른 기존
required/extended/release gate 미실행 상태는 유지한다.

### hugmap 고정 몸체 배치 지도 (2026-11-03)

`workbench/research/support/hugmap.py` 0.1.0을 별도 단일 파일 계산기로 추가했다.
몸체 중심을 고정하고 지름 100 mm 원의 중심을 전방 반경 500 mm 반원에 배치한다.
JSON은 물체의 `(x_mm,y_mm,theta_deg)`를 유지하며 circle만 theta=0으로 생략한다.
비원형 접촉 솔버는 미구현이다. 기본 20×20 mm 격자는 1,006개이고 고정 보호 형상에
걸리는 102개를 제외한 904개를 계산한다. 두 굽힘은 배치당 최대 64회 초기값 탐색,
유효 후보 16개 상한이며 최저 발견 μ를 대표로 저장한다. 전역 최적/불가능 증명은 아니다.

기하·충돌·마찰 검사는 newhug에서 독립 복사했다. NumPy/SciPy와 GUI용 Tkinter만
필요하고 sibling 모듈·CAD 파일은 읽지 않는다. CPU process pool, 고정 4배치 작업,
작업 묶음 내 warm start, 자식 BLAS 단일 스레드, 접촉쌍 점수 벡터화를 적용했다.
현재 호스트 자동 선택은 7프로세스이며 CLI `--workers`로 조절할 수 있다.
GUI 중지 시 완료 결과를 JSON으로 마무리한다. 강제 종료 partial의 자동 이어하기와
reader는 미구현이다. 실행과 좌표/각도 단위는 `docs/research.md`에 기록했다.

호스트 `.venv/bin/python -m unittest workbench.tests.research.analysis.test_hugmap -v`:
**9 passed**. 기존 정답의 μ 재현, ±1 mm 이웃 해, 엄격 μ 컷오프, 격자 누락/중복,
후보·시도 상한, 활성 worker 취소와 JSON 완결, 출력 보존, 단일 파일을 별도 임시
폴더로 복사한 `python -I` 실행 및 1/2프로세스 결과 일치를 확인했다.
작은 250 mm 격자에서 1프로세스 16.72초, 2프로세스 12.85초였다. 이 측정은 전체
20 mm 지도나 코어 수에 비례한 가속을 보증하지 않는다. 기본 전체 지도는 미실행이다.
Tk는 DISPLAY :0에 연결할 수 없어 실제 GUI 화면 조작은 미검증이다. generated dev
wrapper가 없어 canonical required/extended/release gates는 미실행이며 실기기 검증도 아니다.

### newhug 기록 μ 하한 제거 (2026-11-03)

Standalone 0.6.1에서 기록 μ 하한 설정·UI·CLI·JSON 필드를 제거했다. 이제
**최소 필요 μ < 허용 μ**인 유효 해를 전부 기록한다. 낮은 양수·0인 최소 필요 μ도
허용 μ가 그보다 크면 제외하지 않는다. 기존 전역 상한 컷오프, 몸체·카메라 보호,
접촉 부위 제한과 band 정렬은 유지한다. 바늘 맵 schema는 3이다.

100/160 mm 원·256회·허용 μ 0.5에서 팔 후보 4/5개가 원본 바늘 맵에도 각각
4/5개 전부 기록됐다. 기존 기록 하한 0.2에 걸렸던 최소 μ 0.0787/0.1405 등의
해도 남는다. 관련 host 회귀 **72 passed**, standalone 자체 **37 passed**로
총 **109 passed**이며 `/home/user/ws/newhug.py`에도 동일하게 반영했다.
[조건·소스 SHA·수치](../workbench/evidence/curated/planning/newhug_no_lower_mu_20261103.json).
이전 버전의 하한 필터 설명·수치는 아래 이력에 보존한다. generated dev wrapper
부재로 canonical required/extended/release gates는 미실행이며 실제 브라우저
레이아웃·Genesis/실기기 검증은 아니다.

### newhug 전역 μ 컷오프·각도 오차 기능 제거 (2026-11-03)

Standalone 0.6.0에서 `표시 마찰계수`를 **허용 μ 컷오프**로 변경했다.
일반 솔버는 후보 순위·저장·콜백 전에 `최소 필요 μ < 허용 μ`를 적용하며,
원본/정렬 바늘·band·JSON/SVG에도 같은 조건을 적용한다. 같은 값은 force closure
경계라 제외한다. 기존 기록 하한은 유지해 바늘은 `하한 ≤ 최소 필요 μ < 허용 μ`만
남긴다. 법선 각도 오차 입력·기본 2° 가산·오차 포함 μ 점수와 CLI/JSON 필드를
제거했고, 비교 runner도 최소 μ 하나를 사용한다. 독립 결과 schema는 5,
바늘 맵은 2다. EleSim DDS/runtime 계약은 변경하지 않았다.

계산 후 μ를 내리면 현재 팔 자세·원본 바늘·정렬 바늘을 즉시 거르고 band도 다시
계산한다. 저장은 해당 필터 결과를 사용한다. 현재 작업에서 찾은 해만 메모리에
보존해 μ를 되돌렸을 때 복원하며, 탐색 당시보다 높은 μ에서 새 해를 찾으려면
재계산이 필요하다고 표시한다. 새 작업·옛 cutoff revision의 늦은 응답은 거부한다.
계산 중 μ 변경은 비활성화한다. 현재 작업을 넘어가는 결과 캐시는 추가하지 않았다.

100/160 mm 원·256회·기록 하한 0.2·허용 μ 0.5를 실제 수집해, 팔 후보는 4/5개,
바늘은 1/4개였다. 컷오프를 넘은 후보 252/251개는 결과에 들어가지 않았다.
이는 정해진 횟수의 부분 탐색이며 결과 개수는 보장하지 않는다.
[조건·소스 SHA·수치](../workbench/evidence/curated/planning/newhug_friction_cutoff_20261103.json).

Host cutoff **13**, 바늘 맵 **33**, 몸체 보호 **17**, 실제 localhost API·JavaScript
**4**, Wrap 비교 **7**의 합 **74 passed**, standalone 자체 **37 passed**로
관련 검사 **111 passed**다. UI 스크립트는 DOM 대역을 사용하며 실제 브라우저
레이아웃 검증은 아니다. generated dev wrapper가 없어 canonical
required/extended/release gates는 미실행이다. 실기기·Genesis 동작 검증은 아니며
아래 0.5.x 수치는 이전 컷오프·오차 조건의 이력이다.

### newhug 몸체 접근 금지·접촉 부위 제한 (2026-11-03)

Standalone 0.5.1은 접촉 허용을 **seg1·seg2·gripper_base·gripper_claw**로 명시하고,
카메라·하우징·웨지를 보호 부위로 지정한다. 기존에도 하우징·웨지가 접촉쌍에는
없었지만 관통 검사만 있어 부수적으로 닿는 해가 통과할 수 있었다. 이제 몸체는
ZED와 같은 별도 보호 조건으로 **기본 1 mm보다 커야** 하며, 보호 간격 0에서도
접촉·미세 관통을 거부한다. 일반 접촉 구성·수치 보정·정확한 원 수집·최종 검증에
적용한다. UI/CLI에서 몸체 보호 간격을 조정하고 보호 외곽을 표시하며 JSON에는
하우징·웨지별 실제 간격과 허용 접촉 그룹을 기록한다.

100/160 mm 원에서 각 256회·명목 μ 하한 0.2로 다시 수집한 **254개씩 전부**
보호 간격을 통과했다. 기록된 최소 몸체 간격은 각각 1.796/1.143 mm다.
신규 보호 부위 회귀 **17**, 기존 바늘 맵 **34**, 실제 loopback API·JavaScript
**3**, Wrap 비교 **7**의 합 **61 passed**, standalone 자체 검사 **37 passed**로
관련 host 검사 총 **98 passed**다. 원본 `hug.py`와 runtime은 유지하며
`/home/user/ws/newhug.py`에 같은 파일을 동기화했다. 기존 서버는 재실행해야 반영된다.
Canonical dev wrapper 부재로 required/extended/release gates는 실행하지 않았고,
실기기·Genesis 접근/포획 검증은 아니다. 아래 0.5.0 바늘 그림·수치는 이전 조건의
이력으로 보존한다.

### standalone newhug 바늘 맵·원형 band 정렬 (2026-11-03)

`workbench/research/support/newhug.py`를 독립 계산기 정본으로 두고 동일한 0.5.0
파일을 `/home/user/ws/newhug.py`에 동기화했다. 원본 `hug.py`는 유지했다.
UI·CLI에서 iteration(기본 256), seed, 선택적 시간 상한과 **필요 최소 μ ≥ 기록 하한**을
설정하고 상위 10개 미리보기 제한 전에 전체 해를 기록한다. 명목/오차 포함 μ를
선택하며 취소 시 부분 결과를 저장한다. 리니어는 기존 1.5°/mm·260°를 유지한다.
바늘은 GO2 기준점의 주차 선분이며 현재 장착의 node0 중립 전방 오프셋 343 mm를 쓴다.

원의 수집은 정확한 원/로봇 면 접선·간격을 사용한다. 끌어모으기는 단일 반경에
강제 정렬하지 않고, 최대 겹침 수의 기본 80%가 지지하는 연속 반경 band를 선택한다.
안쪽·바깥쪽 부분을 자르고 각 남은 구간을 동일 둘레각으로 회전한다. 서로 다른
반경을 보존하며 원본 선분·잘린 구간·linear 부분 범위를 함께 저장한다. UI에서
자동/수동 band, 합의 강도·둘레각 조정과 바늘 클릭 시 팔 자세 표시, JSON/SVG 저장을
제공한다. 바늘 사이를 보간하지 않는다.

동일 seed·256회·명목 μ 하한 0.2에서 지름 100 mm는 **254개 기록, 179개 잔존,
band 316.51~386.54 mm**, 160 mm는 **254개 기록, 174개 잔존,
band 442.30~558.96 mm**였다. 남은 바늘들의 기준 반경도 각각 서로 다르다.
[원본/정렬 비교](../workbench/evidence/curated/planning/newhug_needle_band_20261103.svg),
[조건·소스 SHA·수치](../workbench/evidence/curated/planning/newhug_needle_band_20261103.json),
[사용·재현 방법](research.md#newhug-바늘-맵과-원형-band-정렬).

Host standalone 자체 검사 **37 passed**와 신규 수학·리니어·band **34 passed**,
실제 loopback API·페이지 JavaScript **3 passed**, 기존 Wrap 비교 **7 passed**로
총 **81 passed**다. JavaScript 검사는 Node의 DOM 대역을 사용하며 실제 브라우저
레이아웃 검증을 뜻하지 않는다. 알려진 generated dev wrapper가 없어 canonical
required/extended/release gates는 실행하지 않았다. Genesis/RL rollout·실기기
파지, GO2 본체·다리·플레이트 충돌과 접근 경로는 검증하지 않았으며 runtime은
변경하지 않았다. R0–R5 수용 완료로 승격하지 않는다.

### standalone newhug의 Wrap 유사 과제 (2026-11-02)

`analysis.newhug_wrap_comparison`으로 지름 100 mm, 중심 (0.368, 0.160, 0.550) m의
기존 Wrap 원기둥에 대해 base 고정·현재 리니어 [-173.33, 0] mm·마디 ±36°·
manifest의 결합 한계·카메라 1 mm 비접촉 조건을 걸었다. newhug의 FK/프로파일/마찰
점수를 쓰며 정확한 원의 접선 방정식과 표본화한 관절 탐색으로 **정적 끝 자세만**
비교했다. 기본 +90°와 runner 안에서 반사한 -90°를 검사하며 standalone UI는
바꾸지 않았다. -90°에서 리니어 -75.63 mm, 마디 -12.53°/-32.90°의 C자와
node4/node9 접촉을 찾았고 이상적인 평면 마찰계수 하한 0.1361/법선 2° 오차 포함
0.1718, 카메라 간격 67.12 mm다. 같은 C자의 +90° 배치는 카메라 관통 9.40 mm로
거부된다. 기본 +90°의 선두 유효 해는 리니어 -137.32 mm, +25.83°/-36°의 S자다.

독립 URDF 3D FK의 hinge 오차는 2e-16 m 이하, 실제 camera OBJ의 360방향 support
오차는 3e-10 m 이하다. Host standalone 자체 검사 **37 passed**, 비교 수학·현재
장착·리니어 회귀 **7 passed**. 새 RL policy rollout/Genesis 접근·들어올리기와
canonical required/extended gate는 수행하지 않았다. 원격 SSH는 22 연결 거절/
2222 인증 실패였고 host에는 Torch/Genesis가 없다. 물리 파지·부하·압축 성공이나
전역 최적성의 증거로 승격하지 않는다.
[그림](../workbench/evidence/curated/planning/newhug_wrap_20261102.png),
[조건·수치·소스 SHA](../workbench/evidence/curated/planning/newhug_wrap_20261102.json),
[재현 방법](research.md#newhug와-고정-배치-wrap-과제-비교).

### 그리퍼를 고정한 Hug 판정 진단 (2026-11-02)

꼬다리는 기존 그리퍼다. 길이 조합을 설계 변수로 둔 초기 실험은 사용자 범위
정정 뒤 중단하고 원자료를 보존했다. 완료한 본 진단은 **현재 reference 그리퍼
근사 `n=0.155`와 두 구간 각각 250 mm**를 고정한 8개 형상이다. 실제 그리퍼
CAD/jaw 접촉이나 ID5 집기를 재현하지 않으며, runtime의 판정·제어·UI는 바꾸지
않았다. `/home/user/ws/hug.py`도 변경하지 않았다. 현재 vendored reference와
공통 계산 함수/클래스 52개의 AST가 모두 같다.

원본은 구간 길이를 물체에 맞춰 바꿔 찾으므로 그 결과를 별도 집계했다. 고정 길이
진단은 연속적인 첫 접촉 위치, 두 감김 방향, 측면/그리퍼 끝점 접촉을 탐색하고
몸통 간섭·관통 허용 오차·가상 닫힘선의 최소 면적 중심 여유를 검사한다. 형상당
탐색 상한은 20초/512 solves이며 전수 탐색이 아니다. 구판 exact 판정은 각
진단 후보에 그대로 다시 적용했다.

| 형상 | 진단 기하 후보 | 같은 후보 중 구판 exact 통과 | 선두 후보 최소 굽힘 여유 | 몸통 간격 |
| --- | ---: | ---: | ---: | ---: |
| 지름 160 mm 원형 | 17 | 0 | 9.05° | 5.99 mm |
| 지름 200 mm 원형 | 1 | 0 | 30.35° | 2.44 mm |

18개 모두 `P2 is behind P1`에 걸렸고 10개는 구판의 무조건적인 polygon
`crosses`에도 걸렸다. 허용 오차 기반 기하 진단과 구판 exact는 같지 않다.
나머지 6개 형상은 이번 예산에서 미발견이며 기구적으로 불가능하다는 뜻이 아니다.
후보 수는 작업 공간 크기나 파지 성공률도 아니다. 선두 두 후보 모두 고정 base와
같은 재료 접촉 위치에서 두 굽힘만 ±5° 보정하는 제한된 ±1 mm/±1° 국소 탐색은
0/6이었다. 접근·회수 경로 또는 가능한 모든 자세 보정의 실패를 증명하지 않는다.

선두 후보는 가정한 마찰계수 0.3/0.6에서 이상적인 평면 두 점 force-closure를
만족하고 마찰 0에서는 만족하지 않는다. 독립 접촉 정상력·면적 중심·검증되지 않은
+Y task 축의 계산일 뿐 실제 부하 여유가 아니다. **ID3/ID4 전류-접촉력 보정,
구간 결합, 압축·미끄럼·유지와 실물 파지는 미검증**이며 관련 결과는 계속 `null`이다.
재구성 메쉬에서 실행 가능한 Hug나 RL 대체 실행기가 완성됐다는 뜻이 아니다.

기존 서버 Pilot image를 network-none/GPU 없음/과학 소스 read-only의 CPU 진단으로
사용했다. 원격 **23 passed / 1 skipped**(24 tests, 선택적 Node probe 부재), 최종
host 보고서 **5 passed**와 실제 HTML의 8개 행/18개 후보 선택 probe를 통과했다.
Host에는 Shapely를 설치하지 않았고 전체 기하는 Pilot image에서 검사했다. 현재
canonical required/extended 및 Genesis/Robot gate를 반복한 결과가 아니다.
원래 **243개 container ID·image·상태와 그 image 52개**를 보존했고, 새 진단 8개도
삭제하지 않고 정지했다. 중단한 스윕의 종료 137은 명시적인 stop 결과다. 기존 cache를
mount/삭제하지 않았으며 정상 dev/Pilot/Sim은 기동하지 않았다.
[오프라인 후보 viewer](../workbench/evidence/curated/planning/hug_fixed_gripper_20261102.html),
[실행 조건·수치·소스/원자료 SHA·보존 감사](../workbench/evidence/curated/planning/hug_fixed_gripper_20261102.json).

### Visual Servoing 통합 UI와 원격 GPU2 회귀 (2026-11-01)

작업 탭을 **Visual Servoing / Manual / Status**로 정리했다. 기존 Scan·Legacy 탭과
Manual의 Mock Object/Wrap 행을 제거하고 **Scene → Scan → Reconstruction → Action**
네 단계에 배치·관측·재구성·동작 시험을 모았다. 이전 panel 4개와 교체 전 회귀
원문은 archive ZIP과 SHA inventory로 보존했다. 후속 요청으로 Sim 카메라는 observer
메인·우측 상단 hand-eye 보조 화면으로 고정하고 레이아웃 선택·교환을 제거했다.
기존 선택적 pop-out은 유지한다. 이 후속 변경의 camera/workspace/layout 회귀는
host **112 passed**다. 아래 303개 UI 전체 검사는 이 레이아웃 단순화 이전 실행이다.

UI는 명시적 intent와 ACK 표시를 소유한다. Pilot 요청은 motion endpoint/boot,
scene 명령은 UI Sim/boot/session에 묶어 조작 대상과 카메라 선택의 독립성을 유지한다.
Reference solution은 양쪽 context와 object revision/digest/epoch가 바뀌면 버린다.
스캔 중 장면 변경·경쟁 동작과 중복 제출을 막고, 재구성 적용은 동일 producer Sim
boot와 published revision을 검증해 matching ACK가 와야 완료로 표시한다.
Stop workflow, 항상 보이는 Torque OFF, paused 상태의 명시적 Resume Sim을 제공한다.
Wrap의 존재하지 않던 `pick_cancelled` 참조도 기존 Pick stop event로 바꿔 ramp/roll
중지를 연결했다. 기존 IK solver의 계산 취소는 미지원이며 해당 Stop은 torque-off를
요청한다. 실기기 stop deadline을 검증한 결과가 아니다.

스캔은 **현재 Sim 전용**이다. 메쉬 기반 gripper/embrace 실행과 유지 검증은 미연결로
표시한다. Sim ground-truth reference Hug와 설정된 원통 geometry의 Wrap 정책을
재구성 메쉬 기반 실행으로 표시하지 않는다. 기존 detector·UV/LJI 계산과 wire API는
공유 backend/호환 경로로 유지하며 DDS v6/operator-view v1은 변경하지 않았다.
[화면·조작·모델 연결 범위](visual_servoing.md).

`hckang@100.74.222.24`의 현재 설치 dev service에서 GPU2 UUID를 CUDA 장치 하나로
선택하고 실제 CUDA 계산 및 변환 검사를 수행했다. 기존 dev image는 Genesis 1.4.1이고
현재 generated wrapper의 선택 alias가 없어, wrapper를 통한 build/recreate 대신
**기존 컨테이너 ID를 보존한 Compose exec + 별도 venv/ROSIDL overlay**를 사용했다.
검증 venv는 Genesis 1.4.5, Quadrants 1.3.3, Torch 2.12.1, Open3D CPU 0.20.0 등
저장소 dev lock **32개**와 일치한다. 최종 실행 파일 **1,335개**의 SHA를 실행 전후 및
로컬 소스와 대조했다. 스냅샷의 코드 identity는 local base `e88ca97` + 파일 manifest다.
Git-aware 설치 검사용으로 복사한 서버 checkout metadata의 HEAD `70757e6`를 실제
실행 코드 revision으로 사용하지 않는다.

최종 required는 **3,982 passed / 6 skipped**, 종료 코드 0, **281.41초**다.
Protocol 194, Robot 166, Pilot **728**, Sim **1,117**, UI **298**, model/release 98,
setup **1,369**, 기타 pytest 시스템 검사 12개가 통과했다. 실제 CycloneDDS 4-process
topology, 2-process Sim motor 및 UI scene-command smoke 3개도 통과했다. Encoded
WebRTC 두 스트림과 active scan/재구성 artifact 경로가 포함된다. Skip은 qpOASES
plugin 부재 1개, opt-in 실제 Genesis camera-worker 2개, Node.js가 없는 설치 frontend
behavior 3개다. 실제 Robot·NAT/TURN·multi-host SROS2·GPU renderer 인수시험의 대체가 아니다.

위 required 실행 뒤 **실제 Genesis 1.4.5**를 GPU2에서 별도로 기동했다. Opt-in
camera-worker 검사 **2 passed / 0 skipped**, 110.26초로 observer RGBD와 자세 변경 후
observer·hand-eye 양쪽 영상 갱신을 확인했다. CUDA UUID와 EGL 장치 UUID를 대조했고
두 renderer 모두 NVIDIA A6000, `software=false`였다. 기존 dev의 EGL vendor JSON
누락은 검증 전용 파일로 보완했으며 원본 image는 수정하지 않았다.

기존 Sim image의 maintained `sim-step`도 `cameras=true`, base command `[0,0,0]`으로
실행해 통과했다. 실제 GO2+arm 물리 씬에서 6 sim초 동안 arm hold와 1280×720 두 영상을
함께 구동했고 각각 28.36 fps, fall/fault 없음이다. 정지 시험이므로 active MPPI solve나
보행 검증은 아니다. 별도 cache의 startup은 261.66초였고 원본 runtime cache를 지우지
않았으므로 cold-start 비교 자료로 사용하지 않는다. 기존 202개 container ID를 유지하고
dev·Pilot·Sim을 원래 정지 상태로 복원했다. 원래 required 3,982/6 집계와 이 추가
실행을 구분한다.
[추가 실행 조건·결과·원본 로그 SHA](../workbench/evidence/curated/ui/visual_servoing_genesis_20261101.json).

이어서 격리 설치의 jar `cycle-24`에서 **새 Visual Servoing의 실제 ImGui widget과
controller**를 실제 Genesis/DDS/WebRTC에 연결해 한 바퀴 통과했다. Mouse down/up으로
Place/Remove, Scan, Apply, Resume와 단계 이동 **8회**를 실행했다. UI 코드 29개 파일은
현재 로컬 소스와 SHA가 일치하며 다른 역할은 import하지 않는다. 고정 암 3-view에서
관측점 **3,733개**, view별 바닥 보정과 실제 AdaPoinTr **8,192개** 출력을 확인했다.
Scan은 **51.82초**, 메쉬 적용은 **2.87초**이며 matching ACK 뒤 `applied` 표시,
중복 Apply 차단, rebuild 후 두 영상 갱신, Pause 후 Resume와 미연결 grasp 차단을
확인했다. 원본 decoder 영상은 각 644 frame / 64.40초, 10 fps로 보존했다.

이 cycle은 **headless ImGui**이며 OS native 창·UI OpenGL renderer 검증은 아니다.
기존 발행 backend revision `af52eda`와 **20 mm voxel surface** 프로필을 사용했다.
현재 기본 Poisson/4 mm remesh의 이번 UI cycle 검증으로 해석하지 않는다. 독립 평가의
observed/predicted/mesh vertex p95는 각각 **0.96/10.08/32.45 mm**로 해당 fixture
기준을 통과했으나 완전 형상·일반 instance segmentation·집기 성공을 주장하지 않는다.
검증 전 202개 container의 ID·image·상태가 모두 같고 추가 진단 5개도 삭제 없이
정지 상태다. 기존 dev와 실제 `test` Pilot/Sim은 down, restart 0이다.

옛 study-wide inventory에서 이번 기동 전에 이미 없던 2개 ID 때문에 driver가 최초
aggregate를 `complete=false`로 표시했다. 최초 결과·옛 missing 목록은 그대로 보존하고
이번 실행 전 inventory를 기준으로 재집계했다. 현재 실행의 누락·변경은 계속 실패로
판정하며 prelaunch snapshot이 없는 legacy 실행의 기존 gate도 유지한다. 회귀검사
**6 passed**다. [실제 UI cycle 조건·영상·평가·감사 결과 SHA](../workbench/evidence/curated/ui/visual_servoing_live_cycle_20261101.json).

이후 `cycle-30`은 **production UI 창 전체**를 private Xvfb에서 열고 실제 XTest
입력으로 Manual 토크, Scan 예산 3, 시작, Reconstruction Apply와 Action 확인을
수행했다. 실제 Genesis 1.4.5/DDS RGBD/WebRTC 두 영상과 **현재 기본 Poisson/4 mm
remesh**를 사용했다. 초기 Sim fixture 배치는 probe가 맡고 원본 빨간 jar는 적용 뒤에도
숨기지 않는다. UI 코드 29개 SHA가 현재 로컬과 일치하고 sibling app import는 없다.

현재 Pilot image에서 Poisson child를 처음 실행하자 Open3D의 EGL 의존성이 누락돼
실패했다. `libegl1` 추가 뒤 build import gate가 USB 누락도 잡았다. Generated app과
standalone Pilot Dockerfile에 `libegl1`/`libusb-1.0-0` 및 실제 Open3D import gate를
넣었고, 새 generated Pilot image의 Open3D **0.20.0** import/build가 통과했다.
Backend 발행 revision은 `61da794`다. 실제 desktop의 공유 peer channel에는 직접
peer의 `.node`가 없어 scene-rebuild capability가 전달되지 않던 문제도 수정했다.
읽기 전용 discovery snapshot을 제공하고, I/O 대기 중 UI session lock을 잡지 않으며
대상·boot·session 변경 중 얻은 결과는 버린다. 최종 원격 **전체 UI 303 passed**,
host focused UI **93 passed**, profile/network/preservation/geometry **23 passed**다.
위 required 3,982/6 집계는 이전 실행이며 이 후속 변경 뒤 전체 required를 반복하지 않았다.

Scan은 실제 Start 위젯 입력부터 완료 표시까지 **58.75초**, AdaPoinTr 추론은
**2.95초**였다. 관측 **3,733개**와 생성 **8,192개**에서 Poisson의 **23,832 vertex /
47,676 triangle** 메쉬를 만들었다. 최종 성분 1개, 열린 경계·비다양체 edge·winding
불일치·자기 교차 0이며 vertex-manifold다. 독립 평가 observed/predicted/mesh vertex
p95는 **0.96/10.08/11.83 mm**로 기준을 통과했다. 38.5% 방향 coverage와 숨은 면 추정의
한계는 유지하며 완전 형상·관측 free-space·집기 성공을 주장하지 않는다.

Apply backend는 **8.08초**, 실제 위젯 입력에서 matching ACK까지 **9.05초**였다.
Entity 상태·camera PID가 보존되고 두 영상이 다시 갱신됐다. 재빌드 중 잠시 Control
OFFLINE을 표시한 뒤 마지막 영상에서는 ONLINE으로 복구됐다. 무편집·1배속 **1920×1080,
30 fps, 2,923 frame / 97.434초**의 전체 창 원본을 디코딩 검증했다. 이번 시작 전
**207개 container, 45개 image, 기존 cache 13,300개 항목**의 ID/image/상태 또는
inode/size/mtime/type가 모두 보존됐다. 추가 진단도 삭제하지 않고 모두 정지했으며
기존 dev·실제 Pilot/Sim은 down, restart 0이다. 실패한 cycle-25..29와 빌드 로그도 보존했다.
[전체 UI 영상](../workbench/evidence/curated/ui/visual_servoing_poisson_demo_20261101.mp4),
[실행 조건·독립 평가·원본 SHA·보존 감사](../workbench/evidence/curated/ui/visual_servoing_poisson_20261101.json).

Extended 실행은 **184 passed / 1 failed / 1 skipped**이며 critical mutation **7개**를
모두 잡았다. 남은 실패는 기존 readability 검사 하나로, 해당 실행 당시 수정하지 않은
`PyMpcGenesisController` 1,079줄과 `UiSimSession` 1,057줄이 1,000줄 한도를 넘는다.
당시 HEAD와 같은 바이트임을 확인했으며 한도를 낮추거나 검사를 건너뛰지 않았다.
최종 네 역할 release는 별도 출력 경로에서 생성·격리 검증을 통과했다.

넓은 검사에서 발견한 test-double API 누락, Git에 없던 replay `.log` fixture,
editable import가 가짜 legacy image 검사에 새어 들어가던 문제를 assertion을 유지하며
보완했다. 첫 실패 실행과 후속 성공 실행을 모두 보존했다. 서버 기존 **202개 container
ID**와 dev image를 유지했고 dev는 원래 정지 상태로 복원했다. 실제 `test` Pilot/Sim은
같은 ID·image, restart 0, down 상태이며 로컬 실제 UI도 기동하지 않았다. 원본 Genesis
cache나 image를 정리하지 않았다. [조건·집계·원본 로그 SHA](../workbench/evidence/curated/ui/visual_servoing_20261101.json).
R0–R5 운영 인수 완료를 주장하지 않는다.

### Genesis 1.4.5 GO2 접촉력 읽기 순서 수정 (2026-11-01)

실제 UI에서 보행 시 `touchdown timeout after 0.30s: FL,FR,RL,RR`가 발생했다.
Sim process는 살아 있고 restart는 0회였다. 설치된 Genesis 소스를 확인하니
`set_dofs_position`은 arm DOF만 지정해도 `collider.clear`를 호출해 모든 접촉력까지
지운다. 기존 loop는 arm 쓰기 뒤 그 live getter를 읽어 실제 지지를 잃은 것으로
판단했다. GPU2 원본 재현에서는 첫 walk의 실제 접촉력이 각 발 약 13 N인데
제어기 관측은 모두 0 N이었다. 2.71 sim초에 주저앉아 전진·정지 시험에 실패했다.

완료된 physics step 직후 소유한 contact tensor를 갱신하고 ready/walk 양쪽에서
그 값을 사용한다. 자세·속도는 기존 current getter로 읽으며 접촉력도 같은 한 번의
host transfer에 포함한다. 다음 물리 계산의 zero force는 그대로 반영하고 reset은
보관한 표본을 폐기한다. 접촉 임계값·timeout·허용 토크는 변경하지 않았다.

설치된 Sim 이미지의 별도 source 복사본으로 control-state, MPC adapter, contact
gate, scene ordering, direct arm write, runtime readback **177 passed**다. 기존 앱
파일의 전후 hash도 일치했다. canonical dev gate의 대체 증거가 아니다.
수정 `3e46240`을 push하고 양쪽 정식 updater로 발행했다. GPU2 비교에서 동일
config chain bytes와 모델 데이터로 **0.15 m/s 명령 8초 + 정지**를 통과했다.
실제 X 전진은 **0.570 m**, 정지 명령 뒤 stand 복귀는 **0.8초**다. UI 기본인
**0.8 m/s 명령 8초 + 정지**도 같은 seed/reset의 **3회 모두 통과**했다.
각 회 X 전진 **4.735 m**, stand 복귀 **1.4초**이며 최대 |roll|/|pitch|는
**0.078/0.127 rad**다. 세 회는 같은 결정적 조건의 반복이며 무작위 환경의
통계 검증이 아니다. DDS·카메라 없는 production physics/MPPI/arm-hold 시험이다.
장시간 600초, 횡이동/회전과 camera/DDS 부하 동시 보행은 여전히 별도 gate다.
[조건·결과·원시 증거 SHA](../workbench/evidence/curated/sim/go2_contact_20261101.json).
양쪽 정식 등록/SROS2 교체와 실제 `test` 활성화를 완료했다. 설치된 Connections
runner의 전체 start는 **209.47초**였으며 등록 재사용 검사 72.98초, compute 시작
56.31초, 양쪽 DDS 준비 검사 54.97초가 포함된다. UI/Pilot/Sim의 실제 image ID와
수정 파일 hash가 manifest/source와 일치하고 Pilot/Sim은 물리 GPU2, restart 0이다.
실제 UI의 ONLINE·observer/hand-eye LIVE를 확인했다. 기존 원격 container **200개**와
로컬 sidecar/dev/readiness UI **3개**의 ID도 보존했다. 별도 benchmark는 위에 적은
범위이며, 실제 앱에서도 정상 touchdown 탐색이 기록되고 새 fault
latch가 없는 것도 관측했다. Host-only 집중 검사는 **36 passed**이며
tensor 2개는 Torch 부재로 skip했다. 넓은 host 검사에서는 445 passed와 함께
Pinocchio/Torch/convex_mpc/CasADi 부재로 43개가 실패했다.

격리 bench가 image dependency PYTHONPATH와 mock catalog 경로를 덮어써 초기화에
실패하던 문제도 수정했다. Image 자체 경로와 선언된 read-only data만 사용한다.
해당 Compose 병합·격리·회수 회귀는 **8 passed**다. 원시 실패/재시도 결과도 보존했다.

이 검증에서 파일만 등록했던 `go2_contact_check`는 제거했다. Host remove CLI가
현재 Docker의 `No such container` 응답을 부재로 인식하지 못하는 별도 결함을
발견했다. 고정 Engine을 확인하고 기존 Compose service verifier의 실제 부재 조회를
통과한 registry API로 해당 진단 등록만 제거했으며 Docker 자원 삭제는 하지 않았다.
CLI 응답 문구 호환 수정은 후속 과제로 남는다.

### Instance 교체 시 endpoint runtime cache 보존 (2026-11-01)

Release/security 교체가 endpoint 디렉터리 전체를 바꾸면서 등록된 `test`의
Genesis cache까지 지우는 결함을 확인했다. 이전 배포에서 기존 실험 container와
image는 보존했으나, 활성 `test`의 warm cache는 재생성됐다. Container/image
자동 회수 생략 옵션이 endpoint cache까지 보존한다는 설명은 잘못됐다.

공유 endpoint는 생성된 `config`만 transaction으로 교체하고 기존 `cache`의
디렉터리·파일 inode를 유지한다. 새 endpoint의 cache는 생성하고 퇴역 endpoint는
검증된 registry ID로만 제거한다. 성공, instance/Compose 교체 직후 실패 rollback,
endpoint 추가·제거와 기존 security/remove/teardown 경로를 포함한 host fallback은
**72 passed**다. 양쪽 정식 updater와 실제 SROS2/release 등록 교체 뒤 기존 cache
디렉터리 inode를 유지했고, 서버의 기존 **190개 파일**은 inode·크기·mtime가 모두
일치했다. 새 준비 cache가 추가되어 총 **274개**가 됐다.

### 수정 소스의 실제 배포와 Connections 기동 보완 (2026-11-01)

`eb49daa`와 bootstrap 설정 manifest 누락을 고친 `a98d52b`를 push했다.
양쪽 정식 updater로 같은 source revision의 새 release를 발행했으며 실제 UI
모듈 hash도 source와 일치한다. 정상 Connections의 SROS2 rotate 및 start를
실행해 양쪽 DDS endpoint gate는 통과했다. 로컬 host의 `xhost` 부재는 Ubuntu
`x11-xserver-utils` 한 package로 해결했고 실제 새 UI의 OpenGL preflight도 통과했다.

실제 화면 검증에서 Robot 없는 graph의 camera preview가 허용되지 않은 Robot
topic 생성을 반복하는 결함을 확인했다. Preview를 UI 자체의 live Robot discovery로
지연하고, Robot이 뒤늦게 발견되면 구독을 시작하도록 수정했다. Pilot/control target
선택에는 의존하지 않는다. UI **194 passed**이며 `7a2a94e`를 양쪽에 정식 update하고
SROS2 generation과 실제 instance pin도 교체했다. Connections 전체 start는
**207.05초**에 완료했으며 새 UI의 ONLINE과 observer/hand-eye LIVE를 실제 화면 및
사용자 확인으로 검증했다. GO2 보행은 별도 실패가 있으며 이 기동 결과로 통과를 주장하지 않는다.
Bootstrap **113 passed**, 실제 배포 source tree **560 files** 검증도 통과했다.

현재 설치를 확인해 로컬 UI와 서버 Pilot/Sim이 이전 immutable image를 실행하고
있음을 확인했다. 소스 변경, release 발행, instance pin 교체와 실제 실행 확인을
구분하며 새 UI는 push/update만으로 적용됐다고 판단하지 않는다.

SSH 지문 조회와 실제 인증의 공통 helper에서 Tailscale socket의 TCP MSS를
connect 전에 1100 byte로 제한했다. host MTU/SSH 알고리즘/지문/인증은 유지한다.
생산 ParamikoConnector로 저장된 서버 지문과 keyless 인증을 통과하고 실제 원격
명령까지 **2.67초**에 완료했다. 이 증거는 DDS/영상 경로의 준비를 뜻하지 않는다.
Host에는 Paramiko가 없어 `/tmp`의 격리 venv에만 SSH 라이브러리를 준비했다.

Connections network preparation은 새 `elesim-tailscale start`를 호출한다.
기존 sidecar는 recreate/pull 없이 start하며, 저장된 host state와 달리 container가
빈 state만 보는 Desktop 복원 오류는 live namespace 사용자가 없을 때 한 번만
restart한다. 실제 인증 누락은 여전히 명시적인 login을 요구한다. 런타임 namespace
preflight는 read-only로 유지한다. `ELESIM_KEEP_RUNTIME_ARTIFACTS=1`은 자동
container/image/alias 회수를 생략하고 읽기 전용 Docker 조회로 release 선택
목록만 갱신한다. 서버의 대상 Pilot/Sim 외 기존 **200개 container ID**와 로컬의
기존 sidecar/dev/readiness UI ID는 보존했다. 활성 test의 endpoint cache는 위 결함으로
재생성됐으며 다른 실험 cache는 건드리지 않았다.

UI **192 passed**, 연결/설치의 5개 집중 suite **378 passed**, Connections runner와
updater **50 passed**다. 집중 suite의 첫 sandbox 실행은 Unix/X11 bind 거부로
실패했으며 정상 권한에서 위 수치를 확인했다. 그 뒤 보존 옵션의 release 목록
갱신과 기존 sidecar start를 별도 회귀 검사한다. 현재 설치의 generated dev wrapper는
발견했으나 invocation이 새 ROS base 빌드와 dev container 재생성으로 이어져
보존 지시를 지키기 위해 빌드 중 중단했다. canonical required/extended, isolated
release 및 real-RMW 4-role smoke는 이 revision에서 미실행이다.

### 직접 실행하는 Mock UI (2026-11-01)

저장소 루트의 `python3 -m workbench.tools.ui_preview --interactive`로 production
ControlPanel과 로컬 fixture를 사용하는 `EleSim - Mock UI` 창을 연다. 탭·영상
배치·배율·창 분리/복귀를 조작할 수 있으며 DDS participant나 실제 장치 실행기는
생성하지 않는다. Async 요청은 로컬에서 미연결 응답을 반환하고 요청 기록은
256개로 제한한다. PNG 출력은 기존 `--output` 모드로 유지한다.

WSLg의 `WAYLAND_DISPLAY` 때문에 PyOpenGL이 EGL을 고르는 반면 host GLFW는
X11/GLX를 사용해 renderer 초기화가 실패하던 문제를 확인했다. Interactive 모드의
Linux/X11 프로세스에 GLX를 명시해 해결했다. 샌드박스의 X11 소켓 접근은 EPERM으로
거부되어 표시 권한으로 실행했으며, 실제 X11 계층에서 `EleSim - Mock UI` 창 생성을
확인했다. Host UI **192 passed**이며 PyAV 부재의 `test_zed_preview.py` 제외와
canonical 미실행 범위는 아래와 같다. 실제 드래그·분리/복귀의 수동 검증은 남아 있다.

### 카메라와 작업 조작을 한 창으로 정리 (2026-11-01)

기존 다열 조작부와 시작 시 자동으로 열리던 카메라 창을 기본 한 창의 작업
공간으로 바꿨다. `Scan / Manual / Legacy / Status` 탭으로 나누고 HSV·YOLO·UV/LJI
조작은 `Legacy`에 둔다. 탭 선택줄은 내용 스크롤과 분리해 고정했다. 오른쪽에는
기존 영상 session을 표시하며 observer/hand-eye 확대·나란히 보기와 Robot 좌우
함께/한쪽 확대를 지원한다. 제어 대상과 영상 출처 선택은 여전히 독립이다.

메인 창은 자유롭게 크기를 조절하고 divider로 조작부 폭을 바꾼다. 좁은 창은
위아래 배치로 전환하며 `Controls`를 끄면 카메라 영역을 넓힌다. UI 배율은
글자·위젯만 바꾸며 창 크기를 고정하지 않는다. 별도 카메라 창은 `Pop out` 요청에
한 번만 생성하고 `Dock` 또는 창 닫기로 내장 영상에 복귀한다. 같은 SimView와
공유 texture를 쓰며 기본 구성에서는 두 번째 창/ImGui renderer를 만들지 않는다.
수동 탭/조작부가 숨겨질 때 활성 GO2 속도를 해제하고 held 입력을 놓기 전에는
다시 움직이지 않는다. 활성 tracking/pick의 정지와 Torque OFF는 상단에 남긴다.

Host UI **190 passed**다. 실제 ImGui context로 640×480–1920×1200의 탭 선택·
영역 경계, 긴 설정 스크롤 중 탭 고정, 영상 focus/fallback, PIP 위 observer 입력
차단, 분리 context 초기화/실패 복귀와 숨겨진 수동 입력 해제를 검사했다.
`test_zed_preview.py`는 host의 PyAV(`av`) 부재로 제외했다. EGL/Mesa llvmpipe로
production UI와 합성 영상 fixture를 렌더했으며 최종 보존한 세 배치 모두 렌더 중
operator 요청 0개다. 이 결과는 DDS/실제 영상/모터 실행의 증거가 아니다.

[`기본 화면`](../workbench/evidence/curated/ui/workspace-20261101/workspace.png),
[`수동 조작·핸드아이 확대`](../workbench/evidence/curated/ui/workspace-20261101/manual.png),
[`재현 명령·source/PNG 해시`](../workbench/evidence/curated/ui/workspace-20261101/summary.json)를
보존했다. 추가 narrow/Robot stereo/camera-only 그림과 JUnit 결과는
`workbench/evidence/generated/ui-workspace-20261101/`에 있다.
`python3 -m workbench.tools.ui_preview --help`로 같은 렌더 도구를 실행한다.
초기 X11 `:0` 접근 실패로 실제 GLFW 창에서 드래그·분리·복귀는 미실행이며, generated
development attachment 부재로 canonical required/extended, release build/isolated
verify 및 real-RMW topology gate도 미실행이다. 실행 중 이미지/instance는 갱신하지 않았다.

### 점군 → Poisson → 거리장 → 삼각형 표면 메쉬 (2026-10-31)

통합 `scan.integrated.yaml`의 source 기본 mesher를 `PoissonSurface`로 바꿨다.
관측/지지 생성점의 법선을 추정·정향하고 Screened Poisson을 푼 뒤, 그 표면의
거리장을 **4 mm** 간격으로 샘플링해 zero surface를 Marching Tetrahedra로
추출한다. 원본 native mesh의 비정상 연결을 scene에 직접 넘기지 않는다.
최종 vertex/edge manifold, 정향, 교차, grid/triangle/vertex 예산을 검사한다.
실제 Open3D import/계산은 Pilot 소유 CPU spawn 자식에만 두며 exit 139,
취소와 deadline에 그 자식만 회수한다. Model API 1과 scene schema 1은 유지한다.

실행 23의 같은 **관측 4,325점 / 원본 생성 8,192점**을 사용했다. 지지 조건은
기존 이웃 8개/반경 10 mm를 유지해 생성점 8,023개를 mesh에 쓴다. Raw 배열은
한 점도 바꾸지 않았다. 비교의 두 voxel OBJ hash도 이전 기록과 동일하다.

| 메셔 | triangles | 실제 변 연결 성분 | 비정상 변 | vertex 표면 p95 | 면적 sampling F-score@10 mm | CPU 중앙값 |
| --- | --- | --- | --- | --- | --- | --- |
| Voxel 10 mm | 4,220 | 1 | 12 | 29.73 mm | 0.7676 | 0.054 s |
| Voxel 5 mm | 18,560 | 4 | 504 | 19.73 mm | 0.8792 | 0.112 s |
| **Poisson + 거리장 4 mm** | **48,424** | **1** | **0** | **18.68 mm** | **0.8966** | **8.974 s** |

최종 출력은 열린 경계/정향 오류/확인된 자기 교차 0개다. 면적 가중 30,000점을
사용한 mesh→원형 p95는 17.61 mm, 원형→sampled mesh p95는 11.52 mm다.
역방향은 정확한 triangle 거리 대신 sampling 근사이며 완전성·접촉 정확도의
증거가 아니다. 시간은 WSL CPU warmup 뒤 5회 교차 실행의 중앙값이다.
Poisson은 매번 만드는 CPU 자식의 spawn/import/검증까지 포함하며 새 촬영,
completion 추론 또는 Sim 삽입 시간을 포함하지 않는다.

원본 Poisson에는 열린 경계 45개와 비정상 변 15개가 있었다. 거리장의 parity vote와
양의 외부 padding은 유한한 닫힌 충돌용 proxy를 추론하는 가정이며 원래 입구와
내벽을 확인한 것이 아니다. 25 mm² 미만 성분만 명시적으로 제거하며 이번에는
0.164 mm²짜리 성분 1개를 제거했다. 가장 큰 성분만 고르지 않는다. Raw 관측과
생성 배열, 제거량과 부호 보정 수를 provenance에 보존한다. 점 주위에 6 mm 두께를
주었던 별도 후보는 오차가 더 커 채택하지 않았으며 실패 원자료는 보존한다.

[`비교 그림`](../workbench/evidence/curated/sim/jar-poisson-mesh-20261031/comparison.png),
[`재현 가능한 scene bundle`](../workbench/evidence/curated/sim/jar-poisson-mesh-20261031/scene.zip),
[`지표/해시/검증 범위`](../workbench/evidence/curated/sim/jar-poisson-mesh-20261031/summary.json)을
남겼다. Bundle을 다시 풀어 `SceneArtifact.load`로 검사했고 원본 NPZ SHA-256도
정확히 재현했다. Bundle/그림/환경 pin을 합쳐 약 1.3 MB다. Raw OBJ/viewer 및
실패 진단은 `workbench/evidence/generated/jar-poisson-comparison-20261031/`에 있다.

사용자 명시 승인으로 project 전용 `.venv`에 Open3D CPU 0.20.0과 고정된
NumPy/SciPy를 설치해 실제 계산을 수행했다. 재구성/scan/analysis **97 passed**,
AdaPoinTr/floor **9 passed**, setup **18 passed**, release 설정 **17 passed**로
**총 141 passed / skipped 0**이다. 실제 sphere/torus 복원과 구멍 보존, native
오류/취소/timeout 회수, 분리축 기반 교차 재확인, SceneArtifact export가 포함된다.
기존 immutable 역할 image/instance/영상은 변경하지 않았다. 새 메쉬의 live Sim
삽입과 물리 파지는 미실행이다. Generated development attachment 부재로 canonical
required/extended, release build/isolated verify 및 real-RMW topology gate도 미실행이다.

### 같은 단지 점군의 20/10/5 mm 메쉬 비교 (2026-10-31)

실행 23의 관측 **4,325점 / 원본 예측 8,192점**을 그대로 사용해 CPU에서만
재메쉬했다. 생성점 필터 반경은 해상도와 분리해 **10 mm / 이웃 8개**로 고정했다.
세 결과 모두 같은 생성점 8,023개를 사용하고 169개를 제외한다. 20 mm OBJ의
SHA-256이 기존 GPU 실행 23의 mesh와 정확히 같아 기준선 재현도 확인했다.

| 해상도 | 점유 복셀 | 삼각형 | 면 연결 성분 | mesh vertex 표면 p95 | CPU 생성 중앙값 |
| --- | --- | --- | --- | --- | --- |
| 20 mm | 286 | 1,076 | 1 | 34.44 mm | 35.9 ms |
| **10 mm** | **1,191** | **4,220** | **1** | **29.73 mm** | **53.3 ms** |
| 5 mm | 4,755 | 18,560 | 66 | 19.73 mm | 125.7 ms |

연결 성분은 면을 공유하는 점유 voxel의 기준이며 일반 mesh component 수와
구분한다. 5 mm는 vertex 오차가 작아져도 입력 밀도 대비 빈틈/분리가 늘고 기존
4,000 voxel 예산도 넘는다. 통합 scan 기본값은 **10 mm**로 낮췄다. 관측 통합의
4 mm와는 독립이며, 생성점 필터 반경은 계속 10 mm다. 기존 설정에서 반경을
생략하면 예전의 voxel/2 동작을 유지한다. 비교에서만 명시한 voxel 예산 8,000은
runtime 기본 4,000을 바꾸지 않으며 최대 허용치다. 최종 scene의 triangle/vertex/
byte 계약 검사는 유지한다.

[`비교 그림`](../workbench/evidence/curated/sim/jar-voxel-comparison-20261031/comparison.png),
[`같이 회전하는 3D 비교`](../workbench/evidence/curated/sim/jar-voxel-comparison-20261031/comparison.html),
[`수치·해시 및 OBJ 기록`](../workbench/evidence/curated/sim/jar-voxel-comparison-20261031/summary.json)을
추가했다. 원본과 세 메쉬는 같은 축척/시점으로 표시한다. 더 작은 voxel에서도
목 위 돌기와 빈틈은 남아 있어 해상도 변경만으로 원형이 복구됐다고 주장하지 않는다.
Vertex-to-surface p95 역시 전체 표면의 관측률이나 파지 정확도를 뜻하지 않는다.

시간은 WSL host CPU에서 해상도별 warmup 뒤 5회 교차 실행한 중앙값이며
file I/O, 촬영, completion 재추론, Sim 삽입 시간은 포함하지 않는다. 이번에는
서버 원본 NPZ 읽기 외 live Docker/GPU/Robot 변경을 하지 않았다. 아래 98.1초
영상과 그 당시 20 mm 실행 기록은 보존한다. 새 기본값은 source 설정 변경이며
기존 immutable 역할 image/instance를 갱신하거나 재시작하지 않았다.

관련 host fallback **74 passed**다. 검증 중 기존 freshness test가 worker에서
mock 시간을 1초 뛰어넘겨 control deadman과 경쟁하던 문제를 확인했다. 실제
control tick으로 시간을 진행하도록 test만 고쳤으며 stale/future frame 거절과
500 ms runtime guard는 유지한다. HTML은 JavaScript 문법 검사와 PNG 확인을
마쳤으며 browser WebGL 실기동은 미검증이다. Generated development attachment
부재로 canonical required/extended, release build/isolated verify, 4-role real-RMW
topology gate는 미실행이고, 새 해상도의 live Sim 삽입·물리 파지는 검증하지 않았다.

### 굴곡 있는 단지 1-cycle 데모 (2026-10-31)

최대 몸통 지름 **120 mm / 높이 220 mm**, 목 지름 52.8 mm, 열린 입구와
내벽·돌출 lip·잘록한 하단이 있는 jar를 직접 생성했다. ShapeNet에서 가져온
개별 sample이 아니다. 원본 OBJ는 **2,402 vertices / 4,800 faces**이며 물리 씬과
두 camera replica가 같은 content hash의 메쉬를 convexify/decimate 없이 쓴다.
[`형상 미리보기`](../workbench/evidence/curated/sim/jar_scan_target_20261031.png)와
[`원본 OBJ`](../workbench/evidence/curated/sim/jar_scan_target_20261031.obj)를 보존한다.

서버 GPU 2/A6000의 정상 역할 이미지로 바닥 보정 → 자동 시점 수집 → 실제
AdaPoinTr AP-PS55 → voxel mesh → UI Apply → 새 두 WebRTC frame까지 실행했다.
Runtime `af52eda4722e930a43ca9326fef240d336d10652`, 실행 23에서 예산 12개 중
**6개 시점 / 관측 4,325점 / 원본 예측 8,192점**을 얻었다. 각 시점의 바닥 보정이
3개 안정 frame으로 적용됐고 보정용 capture 12개는 채택 view와 구분해 남겼다.
`information_saturated` 종료는 완전 관측 판정이 아니며 기존 completeness=false와
needs_more_observation=true를 유지한다. Scan 응답 뒤 완료까지 **86.07초**,
첫 model forward **2.56초**, 실행 중 mesh 적용 함수 전체 **2.69초**였다.
기존 kernel cache의 복사본 및 첫 시도 이후의 jar 전처리 cache를 사용한 단일
경로 검사이며 cold startup/steady-state 추론/지연 통계로 해석하지 않는다.

복잡한 원본 형상에서 두 품질 결함을 수정했다. 실행 21의 세 번째 촬영은 색
마스크의 pixel 하나가 1.654 m 뒤 배경 깊이를 읽어 메쉬를 크게 확장했다.
25 mm 이내의 인접 depth로 성분을 만들고 기존 detector 최소 면적 20 pixel
미만인 성분만 제외해 넓은 별도 내벽은 유지했다. 실행 22에서는 관측 오염은
없어졌지만, 드문드문 이어진 생성점이 voxel 메쉬에 돌기를 만들었다. 통합 mesher는
10 mm 반경에 이웃점 8개가 있는 생성점만 사용하도록 했다. **모든 관측점과 원본
예측 8,192점은 보존**하며 최종 mesh에는 생성점 8,023개를 사용하고 169개를 제외했다.
필터는 물체 정답·형상별 bounding box를 쓰지 않으며, 두 옵션 모두 외부 adapter에
자동 적용하지 않는다. 일반 adapter의 기본값 0은 계속 비활성이다.

| 실행 | 관측 표면 p95 | 원본 생성점 표면 p95 | mesh vertex 표면 p95 | 기존 형상 기준 |
| --- | --- | --- | --- | --- |
| 21, 입력 오염 | 0.87 mm | 40.43 mm | 866.42 mm | 거부 |
| 22, 입력 수정 | 0.90 mm | 10.88 mm | 249.74 mm | 거부 |
| 23, 생성점 지지 조건 추가 | 0.90 mm | 10.88 mm | 34.44 mm | 통과 |

관측 최대 오차는 최종 2.23 mm다. 위 판정 한계는 기존 observed p95 15 mm,
generated/mesh vertex p95 45 mm로 유지했다. 최종 mesh는 **536 vertices / 1,076 faces**다.
원본 생성점에는 최대 약 398 mm의 outlier가 남아 있고, 이를 포함한 원본 배열의
수치와 mesh의 수치를 분리한다. 거친 20 mm voxel surface는 입구·목의 세부 형상을
단순화하며, vertex 표면 거리는 전체 표면의 완전성이나 접촉·파지 성능을 증명하지 않는다.

실패한 두 실행도 보존했다. 세 실행의 아홉 역할 process 모두 exit 0/OOM false,
Pilot fatal log 0 bytes이며 마지막 Python stack 수집은 1,701 sample/error 없음이다.
관련 host fallback은 Sim/analysis **34 passed**, Pilot **68 passed**다. 검증 설치에
generated development wrapper가 없고 host에는 Genesis가 없어 Runtime CLI/
async-camera 전체 수집과 canonical required/extended, release build/isolated verify,
4-role real-RMW topology gate는 미실행이다. 정상 역할 GPU 검증을 이 전체 gate의
대체로 취급하지 않는다. 물리 Robot/GO2, 실제 장식용 트리의 변형·파지·들기,
native UI 창 및 다중 호스트/NAT 검증도 포함하지 않는다.

최종 실제 화면의 **98.1초 / 1배속**
[`단지 통합 데모`](../workbench/evidence/curated/sim/scan_cycle_jar_20261031.mp4)와
[`실행·해시 보고서`](../workbench/evidence/curated/sim/scan_cycle_jar_20261031.json)를 추가했다.
원격 `cycle-21/`, `cycle-22/`, `cycle-23/`, `jar-demo-20261031/` 원자료 및 기존 영상은
보존한다. 작업 전 180개 container의 ID/name/image/status는 최종 202개 inventory에서
누락·변경 0개이고, 기존 cache와 원본 source/checkpoint SHA-256, 외부 checkout
`70757e6`의 clean 상태도 확인했다. 기존 container/image/volume/cache는 삭제하지 않았다.

### 바닥 보정 → 자동 스캔 → 실제 AdaPoinTr → Sim 적용 (2026-10-31)

Pilot의 기본 scan 설정을 `scan.integrated.yaml`로 연결했다. 실행 전에 hash가
고정된 source/config/AP-PS55 checkpoint를 strict-load하고, 각 시점의 동일한
RGB-D와 FK pose로 바닥 tilt를 보정한다. 안정된 평면을 확인한 capture만 시야
정합·관측 점군에 넣는다. 자동 시점 탐색 뒤 실제 AdaPoinTr가 예측한 점을 원래
metre/world 좌표로 복원하고, 관측/예측을 구분한 채 20 mm voxel mesh와 immutable
scene bundle을 만든다. 기존 UI Apply 경로로 실행 중인 Sim과 두 camera replica에
적용한다. 모델 교체 경계는 model API 1이며 별도 연구 스크립트 호출을 요구하지 않는다.

원본 PoinTr graph와 32,470,033개 parameter를 사용하며 missing/unexpected key는
모두 0개다. 모델은 Pilot이 소유한 별도 spawn CUDA process에서 실행한다.
준비·추론 취소/deadline은 그 자식만 종료하며 worker가 끝나기 전 Pilot 자원을
해제하지 않는다. source의 private 추출본만 정리하고 원본 asset과 Triton/Genesis
cache는 보존한다. 외부 source/weights의 명시적 read-only 제공은 필요하며,
자동 다운로드나 `ObservedOnly` fallback은 없다. 설치/실행 경로는
[`model_integration.md`](model_integration.md#실행과-sim-전달)에 기록했다.

서버 GPU 2/A6000의 정상 설치 Pilot/Sim/UI 이미지로 다음 실제 경로를 통과했다.
아래 p95는 GT를 Pilot에 보내지 않는 별도 사후 평가의 **표면까지 거리**다.
메쉬 수치는 vertex 기준이며 미관측 면의 정확도·완전성을 뜻하지 않는다.

| 실행 | 설정 / 채택 시점 | 관측점 / 예측점 | 관측 p95 | 예측 p95 | mesh vertex p95 | Sim 적용 |
| --- | --- | --- | --- | --- | --- | --- |
| 18 | 원통, 예산 3 / 3 | 2,195 / 8,192 | 1.63 mm | 7.78 mm | 32.00 mm | 2.52 s |
| 19 | 원통, hand-eye 5° 오차, 예산 12 / 5 | 2,065 / 8,192 | 8.35 mm | 13.71 mm | 35.87 mm | 2.75 s |
| 20 | 최종 freshness 검사, 원통, 예산 3 / 3 | 2,195 / 8,192 | 1.63 mm | 7.78 mm | 32.00 mm | 2.52 s |

실행 18/19의 runtime은 `67e5b8370580ac72d85c50717eae1049a676179c`다.
실행 19는 Pilot의 private hand-eye calibration에만 오차를 넣었으며 Sim truth나
제어는 바꾸지 않았다. 각 촬영의 보정 전 tilt는 1.26–4.88°였고 모두 3개의
안정 frame으로 보정을 적용했다. 12개를 모두 촬영한 것이 아니라 새 관측의
이득이 줄어 5개에서 `information_saturated`로 끝났다. 모든 결과의
`geometry_complete_claimed=false`, `needs_more_observation=true`는 유지한다.
바닥은 명시적 수평 기준으로 tilt만 보정하며 camera origin, 절대 높이/translation,
법선 주위 yaw를 알아냈다고 주장하지 않는다. HSV 단일 대상 segmentation과
거친 voxel mesher도 범위에 명시한다.

첫 GPU 통합 실행 17은 같은 자세 재촬영마다 이동/정착을 반복해 capture 간격이
0.53–0.60초로 늘고 기존 0.5초 평면 안정성 창을 계속 초기화했다. fresh feedback과
관절 오차를 재검사하면서 이미 정착한 동일 목표를 재사용하도록 수정했다.
안정성 창이나 settle 허용 오차를 완화하지 않았다. 최종 runtime `4a98102`는
동일 source clock의 최신 telemetry에 비해 500 ms 넘게 오래되거나 200 ms 넘게
미래인 RGB-D도 거부한다. 실패 원본과 성공 원본을 모두 보존한다.

실행 20은 최종 runtime `4a9810297b22def238888d464c1604acf8e3f8c2`를 정상
이미지로 다시 발행해 검증했다. Start 응답 뒤 scan 완료까지 56.43초였고,
6개의 보정용 capture와 3개의 채택 view를 구분해 보존했다.

세 실행 모두 원본 원통을 숨긴 후 새 mesh를 적용하고, 기존 entity 상태와 camera
PID 보존 및 응답 이후의 새 두 WebRTC frame을 확인했다. 아홉 역할 process가
모두 exit 0 / OOM false였고 Pilot fatal log는 0 bytes다. 안전한 Python stack
수집기는 각각 1,370 / 6,114 / 1,154개 sample을 남겼다. 첫 model forward는 각각 약
2.67 / 2.71 / 2.51초로, 해당 실행의 최초 Triton kernel 준비와 동시 Sim 부하를 포함한다.
별도 steady-state 추론 benchmark나 cold Genesis startup 시간으로 취급하지 않는다.

이번 수정의 host fallback은 Pilot **117 passed**, setup/release 경로 **35 passed**,
UI **16 passed**, protocol registry **7 passed**, FK/별도 UI·Pilot process/stack
sampler **5 passed**다. 사용할 수 있는 generated development attachment가 없어
canonical required/extended, release build/isolated verify, 4-process real-RMW
topology gate는 실행하지 않았다. Host scientific/ROS 의존성을 추가 설치하지 않았다.
실제 검증은 고정 암/Robot-free/한 호스트의 격리 bridge와 headless UI backend이며,
물리 Robot/GO2/native UI 창/다중 호스트 및 NAT 수용시험을 대신하지 않는다.
R0–R5 전체의 완료로 판정하지 않는다.

실행 18의 실제 decoded WebRTC 화면을 1배속으로 구성한 **73.7초**
[`통합 데모`](../workbench/evidence/curated/sim/scan_cycle_integrated_20261031.mp4)와
[`수치·해시 보고서`](../workbench/evidence/curated/sim/scan_cycle_integrated_20261031.json)를
추가했다. 이전 기준선 영상은 그대로 보존한다. 아래 139 수정과 이전 원통 영상은
각 당시의 baseline 결과이며, 새 바닥 보정/AdaPoinTr 실행 증거와 구분한다.
원격 `cycle-17/`부터 `cycle-20/`까지 실패/성공 원자료와 새 진단 container를
모두 보존했다. 작업 전 153개 container의 ID/name/image/status는 최종 180개
inventory에서도 누락·변경 0개다. 기존 kernel cache와 원본 모델 asset을 보존했고,
source/weights SHA-256과 기존 외부 checkout `70757e6`의 clean 상태도 재확인했다.

### Pilot exit 139 원인 분리와 수정 (2026-10-31)

진단 probe에 넣었던 `faulthandler.dump_traceback_later`가 설치된 CPython
3.10.12에서 139를 유발함을 분리 재현했다. 같은 정상 Pilot 이미지의 실제
상태 직렬화를 4개 thread에서 실행하되 DDS/GPU는 초기화하지 않았다.
Native 주기 덤프는 0.1 ms, 1 ms, 원래의 **1초 주기 모두 exit 139**였다.
덤프 없는 기준 실행은 60초 / 19,905회, Python 수집기로 교체한 두 실행은
각각 약 120초 / **40,639회와 42,956회**를 통과했다. 수집기는 합계 1,059개
stack sample을 남겼고 fatal/error log는 비어 있다. 요청 주기는 실제 sample
주파수가 아니며 인공 부하의 처리량을 앱 성능으로 해석하지 않는다.

한 native 실패는 GDB로 SIGSEGV와 405 MB core를 확보했다. C stack은
Python 평가/vectorcall과 동시에 실행 중인 trace writer를 보여준다. Interpreter가
stripped binary이므로 정확히 손상된 C object까지 특정했다고 주장하지 않는다.
CPython의 frame/code 수명 경쟁은 source와 현상에서 얻은 추론이며, 다른 버전의
[upstream 보고](https://github.com/python/cpython/issues/158200)를 이 환경의
증명으로 대신하지 않는다. 과거 실행 7/9의 원본 전체를 그대로 replay한 결과와도
구분한다. 확인한 트리거는 probe에서 제거했고 fatal-signal handler는 유지했다.

Runtime `e7b81b18e069a70d3797f5c8e417e4f828d767c6`에는 코드 검토로 확인한
종료 결함도 수정했다. RGB-D receive/publish thread가 끝나기 전에 DDS endpoint를
파괴하지 않으며 join timeout이면 참조를 보존하고 오류를 보고해 재시도한다.
Pilot connection callback이 끝난 뒤 facade relay를 닫고, SIGTERM은 workflow,
media와 trace 정리를 거치게 했다. 이 결함들을 이번 139의 입증된 원인으로
혼동하지 않는다. 서버 GPU 2의 정상 설치/발행 이미지에서 원통 실행 15와 구형
실행 16 모두 10 Hz 수집을 켠 채 scan → mesh 적용 → 두 WebRTC 화면 복구를
통과했다. 원통은 3-view / 2,219점 / 1,152 sample, 구는 3-view / 1,173점 /
1,208 sample이다. **두 fatal log 모두 0 bytes**, 여섯 역할 process 모두
**exit 0**으로 종료했다. 기존 kernel cache 복사본을 쓴 안정성 검증이며 cold
startup이나 지연 통계의 측정이 아니다.

관련 host fallback은 **113 passed**다. 현재 검증 설치에 사용할 수 있는 generated
development attachment가 없어 canonical required/extended, isolated release와
4-process topology gate는 이번 수정에서 실행하지 않았다. Host Python에
부족한 scientific/ROS 의존성을 설치하지 않았다. 실제 검사는 고정 암/Robot-free
GPU 경로이며 물리 Robot/native UI window/다중 호스트 수용시험은 포함하지 않는다.
비교표, core SHA256, GDB와 실행별 해시는
[`pilot_native_139_20261031.json`](../workbench/evidence/curated/sim/pilot_native_139_20261031.json)에
기록하며 원격 `native-139-20261031/`, `cycle-15/`, `cycle-16/` 원자료와 기존
container/cache는 보존한다. 분리 진단 뒤 확보한 기존 136개 checkpoint와 각
실행 직전 inventory의 container ID/image/status 대조에서 누락·변경은 0개다.
최종 153개 container를 모두 보존했고 기존 외부 checkout `70757e6`도 변경하지 않았다.

### 원통 1-cycle 데모 (2026-10-31)

같은 서버 GPU 2/고정 암 조건에서 반경 45 mm, 높이 140 mm의 빨간 원통으로
다시 촬영했다. 물리 씬은 이미 원통을 지원했지만 async camera replica가 항상
구를 생성하던 누락을 수정했다. Runtime `2949c1a0e228bf523b8ffe54cd92f1526536f1e2`는
immutable render recipe에 shape/height를 전달하고 두 worker가 같은 형상을 만든다.
구·원통 회귀는 수정 전 실패했고 수정 후 Sim focused **51 passed, 2 skipped**다.

정상 역할 이미지의 격리 실행 14에서 실제 DDS RGB-D로 **3개 시점 / 2,219점**을
수집하고 **240 vertices / 476 faces**의 20 mm voxel mesh를 생성했다. 원본 원통을
숨긴 뒤 UI Apply backend로 새 mesh를 적용하고 두 WebRTC 화면의 새 frame과
기존 entity 상태/camera PID 보존을 확인했다. 별도 평가기의 유한 원통 옆면·끝면
거리 회귀 **2 passed**이며, 점군 표면 거리 p95 **1.60 mm**, mesh vertex 표면 거리
p95 **23.19 mm**다. 미관측 표면 완성을 주장하지 않는 기존 기준선을 유지한다.

Scan 시작 응답부터 완료 관측까지 **46.68초**, 적용 함수 전체 **2.49초**
(물리 2.15초 / 두 카메라 잔여 대기 0.26초)였다. 기존 kernel cache의 복사본에서
처음 생성한 원통 scan mesh를 적용한 단일 실행이다. 같은 mesh 반복이나 cold
startup 지연의 통계로 해석하지 않는다. 초기 Sim 준비 이후 두 실제 WebRTC 화면을
기록한 1배속 영상은 **57.8초**다. GDB/주기적 stack 수집 없이 완료했고 이번
Pilot fatal log는 비어 있다. 이 실행 당시 미확정이었던 139는 위의 별도
원인 분리/수정 결과를 따른다.

이전 구형 영상과 실패 기록을 보존한다. 새 시도는 시작 직전 전체 container
inventory를 추가 대조하고 probe를 실행 폴더에 고정 복사한다. 이번 작업 전
130개 container를 보존했으며 정상 image build/publish 뒤의 prelaunch inventory도
검사한다. Canonical/Robot/native UI window 및 다중 호스트 gate 범위는 아래와 같다.
영상은 [`scan_cycle_cylinder_20261031.mp4`](../workbench/evidence/curated/sim/scan_cycle_cylinder_20261031.mp4),
수치/해시는 [`scan_cycle_cylinder_20261031.json`](../workbench/evidence/curated/sim/scan_cycle_cylinder_20261031.json)이다.
원자료는 서버 `/home/hckang/ws/elesim-scan-cycle-20261030/cycle-14/`에 보존한다.

### 실제 카메라 수집 → 다중 시야 → 메쉬 → 실행 중 Sim 적용 (2026-10-31)

서버 GPU 2의 정상 설치 이미지 Pilot/Sim/UI를 각각 분리해, 고정 암의 실제
Genesis RGB-D → DDS → 3-view scan → baseline mesh export → UI Apply backend →
실행 중 Sim 씬 적용을 확인했다. Native UI 창의 자동 조작은 포함하지 않으며,
UI가 실제로 디코딩한 두 WebRTC 화면을 녹화했다. 원본 구를 숨긴 후 두 화면의
빨간 픽셀 0개, 적용 응답 이후의 새 비어 있지 않은 두 화면, 원본의 실제 이동,
기존 entity q/velocity/root pose와 camera PID 보존을 함께 확인했다.

운영 경로 연결은 `b4f3819`, 실제 실행에서 수정한 runtime은 `b96398f`부터
`5d7c123`까지다. Scan bundle의 상대 경로·digest·Sim ID/boot를 publication에
넣고 UI가 같은 Sim에만 적용하게 했다. Sim이 실제 torque/claw 상태를 telemetry에
내보내고 fixed entity의 root pose도 rebuild 시 복원한다. 시야 후보 12,000개를
만드는 동안 16개마다 1 ms를 양보해 DDS 제어/취소를 처리한다. 500 ms guard는
완화하지 않았다. 실제 hand-eye와 intrinsics의 calibration identity를 encoded
RGB-D에 넣었고, 완료 status의 중복 key, 고정 암의 world spawn 적용, RGB mask
경계에서 섞이는 바닥 depth를 수정했다. 경계 제외는 예제의 `mask_erode_px=2`이며
모델 API 1/기존 wire major 6을 유지한다. 추가 정상 실행 12에서 scan 자체는
3-view/1,182점으로 끝났지만, 새 lease를 발급받은 뒤에도 이전
`motion lease renewal rejected: no_active_lease` 오류가 남아 제어권 확인이
30초에 실패했다. `5d7c123`은 새 grant를 받은 경우에만 이 obsolete 오류를
지우며 다른 명령/안전 오류와 telemetry만으로 오류를 지우지 않는 규칙을 유지한다.
해당 회귀는 수정 전 실패, 수정 후 통과했다.

실행 10/11 이미지는 runtime commit `2905b8e608705bba7c6280bdd054a8e334b81692`,
최종 실행 13은 `5d7c123a94a7d0fe4425e3a6efc95ebf4f99c284`에서
normal generated Dockerfile로 만들고 정상 release publication/instance 등록을
거쳤다. 각 역할에서 sibling package가 없음을 확인했고 별도 internal bridge의
eth0/static DDS peers를 사용했다. 공유 artifact root는 Pilot RW/Sim RO이고 UI는
bundle/정답 파일을 받지 않는다. GPU는 A6000 UUID 하나만 예약했다. 초기 build는
기존 cache의 독립 복사본을 쓰며 cold-start 측정은 아니다.

현재 엄격한 수용 조건을 통과한 실행 10/11/13은 모두 3개 시점, 관측점
**1,180개**, 추정점 **0개**, mesh **146 vertices / 288 faces**다. 관측점의 실제
구 표면 거리 p95는 **0.933 mm**이고, 20 mm voxel mesh vertex의 표면 거리 p95는
**28.04 mm**다. 이 두 정확도를 같은 지표로 해석하지 않는다. 시야 방향 proxy는
0.427이며 실제 전체 표면 수집률이 아니다. `geometry_complete_claimed=false`,
`needs_more_observation=true`로 종료한다. HSV/FK/ObservedOnly 기준선이며
AdaPoinTr나 학습된 자세/잔차 모델의 정확도를 입증한 결과는 아니다.

| 실행 | Scan 시작 응답→완료 관측 | 적용 함수 전체 | 물리 rebuild | 두 카메라 잔여 대기 | 녹화 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 10, allocator debug + GDB | 54.11초 | 2.47초 | 2.03초 | 0.33초 | 65.9초 |
| 11, 기본 allocator + GDB | 44.09초 | 2.88초 | 2.44초 | 0.32초 | 57.2초 |
| 13, 기본 allocator / GDB·주기적 stack 수집 없음 | 47.75초 | 2.51초 | 2.09초 | 0.34초 | 58.9초 |

10/11/13의 동일 fixture는 앞선 실행 cache를 복사해 같은 mesh 전처리를 재사용했다.
앞선 첫 생성 mesh의 적용 함수 전체는 6.97초였지만 당시 원본 제거/최종 UI
frame 검사가 부족해 현재의 전체 수용시험 통과로 세지 않는다. 이 차이를 알고리즘
개선이나 임의 새 OBJ의 지연 상한으로 주장하지 않는다. 초기 Python/컨테이너
시작과 Sim build는 녹화/scan 시간 밖이다.

실패 기록도 보존했다. 시도 7/9에서 Pilot **exit 139**가 발생했고 9의 Python
fatal stack은 operator 응답 직렬화 시점을 가리킨다. 당시에는 네이티브 원인을
확정하지 못했고, 10/11의 GDB 실행에서 재현되지 않았다는 이유로 수정됐다고
표시하지 않았다. 당시 probe에는 주기적 faulthandler stack 수집이 있었고 13은
주기적 수집과 GDB 없이 정상 완료했다. 이후 별도 비교 재현으로 해당 native
수집 API를 트리거로 확인해 위와 같이 교체했다. 첫 volatile motion intent가 실제
피드백에 반영되지 않는 경우도 남는다. Probe는 같은 Sim boot의 명시적
torque-on을 제한 횟수 재시도하고 실제 피드백을 요구했으며, runtime의 전달
보장이 해결됐다는 뜻은 아니다.

Host focused 회귀는 scan/model Pilot **71 passed**, lease/state/connection/scan/trajectory
Pilot **105 passed**(앞 묶음과 중복 포함), Sim **49 passed, 2 skipped**다. 그 밖에
이 작업의 protocol registry/PeerClient **24 passed**, optional cuRobo contract **10 passed,
4 skipped**를 확인했다. Host에 Genesis/ROS/AV 또는 과학 의존성을 추가하지
않았다. 이번 검증 설치에는 dev attachment가 없고 기존 wrapper는 기존 개발
컨테이너를 재생성할 수 있어 사용하지 않았다. Canonical required/extended,
release build/isolated verify, 네 역할 `smoke_topology.py`는 미실행이다. 정상
역할 이미지 세 개의 실제 DDS/GPU/WebRTC 검사가 그 gate나 Robot 검사를 대신하지
않는다. 다중 호스트 artifact 전달, native UI 창, GO2 이동/집기와 물리 Robot은
이번 범위 밖이다. R0–R5 전체 완료를 선언하지 않는다.

기존 설치/release pin/source checkout은 교체하지 않고 별도 정상 설치를 만들었다.
원래 inventory 60개 컨테이너와 원본 cache를 보존하며 새 진단 컨테이너도 삭제하지
않는다. 재현/증거 파일의 소유 경계는
[`scan_cycle.md`](../workbench/tests/system/scan_cycle.md)를 따른다.
실행 13의 실제 영상은 [`scan_cycle_demo_20261031.mp4`](../workbench/evidence/curated/sim/scan_cycle_demo_20261031.mp4),
수치/원자료 digest는 [`scan_cycle_20261031.json`](../workbench/evidence/curated/sim/scan_cycle_20261031.json)이다.
원자료는 서버 `/home/hckang/ws/elesim-scan-cycle-20261030/cycle-*/`에 보존한다.

### 씬 교체 지연 추가 단축: 병렬 준비와 고정 모델 재사용 (2026-10-30)

`f624c65`의 순차 교체 경로를 줄였다. 두 visual worker에 scene 준비를 먼저
요청하고 그동안 main thread에서 물리 씬을 재빌드한다. Worker는 초기 로봇/
지면/고정 asset의 resolved description을 재사용해 같은 URDF를 다시 읽지 않는다.
이 사전은 고정 recipe에 한정하며 새로 가져온 OBJ를 누적 보관하지 않는다.
물리 상태 복원과 같은 revision의 두 새 프레임을 확인하는 성공 조건은 유지한다.
Rollback 전에 진행 중 camera 준비를 회수하고, 늦게 도착한 완료 응답도 거부한다.

같은 서버 GPU 2, Genesis 1.4.5 이미지, 1280×720 두 영상, 독립적인 커널 cache
복사본에서 세 번 실행했다. 각 실행은 서로 다른 새 OBJ 형상 두 개를 넣었으며
첫 번째 형상 쌍은 아래 이전 측정과 동일하다. 입력 검증과 두 첫 프레임까지의
전체 시간은 다음과 같다. 임의 메시/다수 물체의 지연 상한은 아니다.

| 동작 | 이전 순차 경로 | 병렬 준비: 형상 1 / 2 / 3 |
| --- | --- | --- |
| 새 OBJ 288면 삽입 | 3.91초 | **2.02 / 2.07 / 2.17초** |
| 새 OBJ 20,400면으로 교체 | 7.55초 | **4.69 / 4.65 / 4.85초** |
| 가져온 물체 제거 | 4.18초 | **1.89 / 1.92 / 2.01초** |

물리 재빌드/상태 복원은 작은 OBJ 1.73–1.86초, 큰 OBJ 4.15–4.34초였다.
카메라 scene 준비는 0.61–1.22초로 줄었고 물리 작업에 가려진다. 물리 완료 후
남은 대기와 두 첫 프레임은 0.23–0.26초였다. 따라서 카메라 준비 전체 시간을
물리 시간에 합산하지 않는다. 일시정지는 남으며 다음 병목은 새 메시 처리와
Genesis의 전체 물리 씬 재구성이다. 별도 cProfile은 이 호출 경로를 확인하기
위한 계측 실행이고, 계측 오버헤드가 있어 위 일반 측정값과 직접 비교하지 않는다.
초기 cold compilation이나 collision 형상/정밀도/제약조건을 줄인 변경은 아니다.

세 실행 모두 qpos/velocity, Scene identity, MPC/controller와 카메라 PID,
비볼록 경로를 유지했다. 카메라 실패, 물리 재빌드 실패, 준비 중 UI session
해제를 주입해 이전 큰 메시/물리 상태/두 새 프레임을 복구하고 paused를 유지함을
확인했다. 잘못된 revision도 거부했으며 각 실행 이후 stationary 100 step을
통과했다. 병렬 경로의 Sim non-RL 회귀는 **924 passed, 3 skipped**, 이후 추가한
late-ack timeout guard를 포함한 최종 camera/scene/operator 집중 회귀는
**54 passed, 2 skipped**다. GPU가 실행한 최종 Sim/protocol/probe 해시는 현재
로컬 코드와 일치한다. 기존 readability 세 건과 준비되지 않은 dev attachment로
인한 전체 canonical gate 미실행 상태는 아래 기록과 같으며 실물 Robot/실제
WebRTC viewer 검증은 포함하지 않는다.

운영 release pin과 컨테이너를 교체하지 않았다. 작업 전 55개 중 별도 `/emulator`
한 개에서 `die`/`destroy` 이벤트가 기록됐고 실행자는 Docker events로 확인되지
않았다. 이 작업은 해당 컨테이너의 변경/삭제 명령을 실행하지 않았다. 나머지
54개는 ID/name/image/status가 보존됐으며 새 진단 컨테이너 6개도 유지한다.
원본 cache는 보존하고 각 trial은 자기 복사본에 썼으며 외부 checkout은 clean
`70757e6` 그대로다. 최종 집중 회귀와 GPU 실행 직전의 58개도 모두 보존됐다.
요약은 [`genesis_scene_overlap_20261030.json`](../workbench/evidence/curated/sim/genesis_scene_overlap_20261030.json),
원자료는 서버 `/home/hckang/ws/elesim-genesis145-20261029/live-scene-speed-20261030/`다.

### 실행 중 새 OBJ 교체와 카메라 프로세스 재사용 (2026-10-30)

Dynamic/async Sim에 비볼록 충돌 경로 예열과 `InteractiveScene.rebuild()`를
연결했다. 초기 build는 보이지 않는 작은 비볼록 고정 메시를 포함하며, 새 OBJ를
로드하거나 모두 지울 때도 해당 충돌 경로를 유지한다. 예열은 최초 컴파일 비용을
startup으로 옮긴다. 빈 캐시의 첫 시작 지연을 없앴다는 의미는 아니다.

현재 UI session의 `load_scene`/`clear_scene`은 main thread에서 motion을 차단하고
qpos/velocity/DOF 설정을 저장한 뒤 물리 씬을 재빌드한다. 기존 entity handle을
arm/GO2/controller/payload에 다시 결합하고 MPC model/solver는 유지한다.
카메라 프로세스 둘도 유지한 채 내부 visual scene만 재구성한다. Epoch별 두
교체 응답과 같은 revision의 두 새 프레임을 받은 뒤에만 성공으로 처리한다.
실패하면 이전 geometry/state/camera revision으로 rollback하고 paused 상태로
남는다. Rollback까지 실패하면 Sim을 종료한다.

서버 GPU 2의 같은 Genesis 1.4.5 이미지와 비볼록 커널 seed cache의 독립
복사본에서 측정했다. 각 새 OBJ는 해당 실행 캐시에 처음 들어오는 합성 메시다.
해상도는 hand-eye/observer 모두 1280×720이며 아래 전체 시간에는 두 카메라
첫 프레임을 포함한다. 최초 버전은 매번 camera process를 재시작했고, 최종
버전은 PID를 유지했다.

| 동작 | 카메라 재시작 전체 | 최종 물리/상태 복원 | 최종 카메라/첫 프레임 | 최종 전체 |
| --- | --- | --- | --- | --- |
| 새 OBJ 288면 삽입 | 16.59초 | 1.88초 | 1.98초 | **3.91초** |
| 새 OBJ 20,400면으로 교체 | 20.04초 | 4.67초 | 2.61초 | **7.55초** |
| 가져온 물체 제거 | 16.52초 | 1.96초 | 2.21초 | **4.18초** |

물리와 카메라 재구성은 순차이며 전체에는 입력 검증/기존 dispatcher 정리도
포함한다. 최종 초기 startup은 28.07초로, 이 표는 startup을 제외한 실행 중
교체 시간이다. 카메라 실패 주입 시 6.00초에 이전 큰 OBJ와 새 카메라 프레임을
복구하고 paused를 유지했다. 잘못된 revision은 씬 변경 없이 거부했다.
모든 교체·rollback·clear에서 Scene identity, qpos/velocity, controller/solver,
카메라 PID 및 비볼록 경로 보존을 확인했고 이후 stationary 100 step도 통과했다.
단일 실행이며 임의 메시/다수 물체의 지연 상한은 아니다.

UI의 `Sim Scene`은 `simulation.scene-rebuild.v1`을 광고하는 Sim에만 명령을
보낸다. Bundle 전체는 Sim의 `simulation.runtime.scene_import_root` 아래에
이미 있어야 하며 UI에는 `new-scene/scene.json` 같은 상대 경로를 넣는다.
기본 root는 Sim의 `$XDG_CACHE_HOME/scene-imports`다. 자동 다중 호스트 전송,
동적 attach/집기는 이번 범위에 없다. Wire major 6/schema 1은 유지한다.

검증은 원격 role-isolated Sim **922 passed, 3 skipped**, host protocol
**188 passed, 5 skipped + 5 subtests**, UI **153 passed**(host에 `av`가 없어
`test_zed_preview.py` 제외), quality matrix/layout **12 passed**다.
새 명령의 capability/session/결과 응답은 실제 두 DDS OS process에서도 통과했고
필수 matrix의 `dds-scene-commands`에 등록했다. Host readability gate는 기준
HEAD부터 있던 1000-line class 초과 세 건으로 실패한다. 기준 길이는 PyMPC
1068, ControlPanel 1131, UiSimSession 1015줄이며 현재 뒤 둘은 1134/1036줄이다.
기준을 완화하지 않았다. 전체 canonical required/extended/release gate는 사용
가능한 현재 dev attachment가 없어 실행하지 않았다. 물리 Robot과 실제 WebRTC
viewer 검증도 포함하지 않는다.

최종 GPU worker가 읽은 Sim/protocol 전체 source hash를 현재 로컬 코드와
대조했다. 운영 release pin/컨테이너는 교체하지 않고 기존 이미지에 읽기 전용
소스 스냅샷을 붙여 실행했다. 작업 전 기존 컨테이너 **45개**의 ID/name/image/status,
외부 checkout의 clean `70757e6` 상태를 보존했다. 추가 진단 컨테이너 10개도
삭제하지 않았다. 원본 cache는 복사만 하고 각 trial은 자기 cache에 썼다.
요약은 [`genesis_live_scene_20261030.json`](../workbench/evidence/curated/sim/genesis_live_scene_20261030.json),
원자료는 서버 `/home/hckang/ws/elesim-genesis145-20261029/live-scene-20261030/`다.

### 캐시된 씬에 새 OBJ 추가: GPU 2 실측 (2026-10-30)

사용자가 지정한 서버 GPU 2(RTX A6000,
`GPU-a40782b2-881c-e285-2a8e-66cae9162a01`)에서 위 1.4.5 정식 Sim 이미지를
사용했다. 기존 씬의 완료된 캐시를 복사하고 GPU 2에서 기준 실행을 마친 뒤,
그 복사본을 OBJ별로 다시 복사했다. 각 OBJ의 첫 실행에는 그 OBJ의 전처리나
추가 충돌 경로가 미리 준비되지 않는다. 반복 실행은 해당 OBJ의 캐시를 재사용한다.

메시는 합성한 닫힌 비구면 OBJ 두 개이며 같은 production scene-artifact importer로
추가했다. World/metres, fixed/collision=true, convexify/decimate=false다.
물리와 1280×720 카메라 복제 씬 둘을 새 프로세스에서 구성한 결과는 다음과 같다.

| 실행 | 물리 OBJ 처리 | 물리 build | 두 카메라 첫 프레임까지 Startup |
| --- | --- | --- | --- |
| 기존 씬 캐시 기준 | 없음 | 3.88초 | 26.30초 |
| 새 OBJ 320면, 첫 추가 | 0.04초 | 134.94초 | 157.29초 |
| 같은 OBJ 반복 | 0.02초 | 4.23초 | 20.73초 |
| 새 OBJ 20,480면, 첫 추가 | 2.27초 | 130.18초 | 155.41초 |
| 같은 OBJ 반복 | 0.51초 | 4.60초 | 22.07초 |

따라서 기존 씬의 warm 수치만으로 새 충돌 OBJ의 첫 삽입을 수초로 예측할 수 없다.
이번 두 첫 실행의 지연은 메시 파일 처리보다 물리 build가 지배한다. Camera build는
1.42–1.63초이고 병렬 실행하므로 합산하지 않는다. Startup에는 assets/controller와
첫 프레임을 포함하지만 Python import/컨테이너 시작은 제외한다.

별도 캐시 복사본에서 같은 물리 `Scene` 객체에 `InteractiveScene.rebuild`를
적용했다. 기존 엔티티를 이름으로 다시 찾고 qpos/velocity를 복원·대조했으며
CUDA 완료까지 기다렸다. 이때 카메라와 앱 controller 재결합은 제외했다.

| 같은 프로세스의 물리 재빌드 | Build만 | 메시 처리·재구성·상태 복원 전체 |
| --- | --- | --- |
| 비볼록 충돌 경로가 없는 기준 씬 → 새 320면 OBJ | 134.15초 | 134.48초 |
| 같은 OBJ 재빌드 | 1.23초 | 1.57초 |
| 준비된 경로에서 처음 보는 20,480면 OBJ로 교체 | 2.02초 | 4.55초 |
| 같은 큰 OBJ 재빌드 | 1.19초 | 1.68초 |

전후 기록한 rigid compiler config는 동일했고 collider config는
`has_nonconvex_nonterrain` 하나만 false→true로 바뀌었다. 설치된 Genesis의
`ColliderStaticConfig` 생성부도 이 상수들의 변경이 재컴파일을 유발한다고 명시한다.
따라서 이번 첫 지연은 비볼록 충돌 경로의 최초 준비와 연결된다. 그 뒤 다른 새
메시로 교체할 때는 이 경로를 재사용했다. 이는 객체 수를 계속 늘리거나 다른 solver
기능을 켜도 같은 시간이 나온다는 증거는 아니다.

다섯 전체 GPU trial은 모두 정상 종료했고 두 카메라 후속 프레임, stationary hold와
scene revision을 확인했다. 네 in-process 재빌드도 정상 종료하고 기존 상태 및
Scene identity 보존을 확인했다. Probe의 `--scene-manifest`와 물리 OBJ 처리 타이머를
추가했으며 host 측정기 회귀 **6 passed**다. 이번 시작 전 기존 컨테이너 39개의
ID/image/name/status와 원본 캐시의 파일 digest가 보존됐다. 진단용 컨테이너도
삭제하지 않았다. GPU 2에는 약 2 GiB를 사용하는 다른 process가 있었고, 각 표
항목은 단일 실행이므로 통계적 범위나 임의 스캔 메시의 시간 상한을 의미하지
않는다. 이 단계에서는 운영 앱의 hot-swap, controller/media 재결합을 제외했다.
이후 앱 구현과 전체 시간은 위 실행 중 교체 항목을 따른다.

SSH는 TCP 연결/배너 뒤 KEX 또는 긴 명령에서 멈췄다. 기존 10-05 기록과 같이
진단 연결 소켓 하나에 `TCP_MAXSEG=1024`를 적용하자 기본 SSH 알고리즘과 기존
host-key 검증으로 전송을 완료했다. 시스템 MTU/route/전역 SSH 설정은 변경하지
않았다. 외부 IP의 2222 포트는 응답했지만 현재 키 인증은 거부됐으며 지문/인증
검증을 우회하지 않았다. 재현 명령은 `benchmarks.md`에 기록했다.

요약은 [`genesis_cached_obj_20261030.json`](../workbench/evidence/curated/sim/genesis_cached_obj_20261030.json),
원자료는 서버 `/home/hckang/ws/elesim-genesis145-20261029/obj-gpu2-20261030/`에 있다.

### Genesis 1.4.5 업그레이드와 첫 시작 실측 (2026-10-29)

Sim package, Sim/development dependency lock과 이미지의 실제 import/version 검사를
Genesis 1.4.5 / Quadrants 1.3.3으로 맞췄다. NumPy 1.26.4, 물리 solver/충돌 설정과
로봇 geometry는 유지한다. 현재 문서의 정적 배열 설명을 실제 기본값인 dynamic으로
바로잡았다. 과거 1.4.1 측정 기록은 비교 근거로 그대로 보존한다.

업그레이드 소스 `92eb0cd`를 푸시하고 서버의 기존 `newsim` 설치를 갱신해 release
`b0df94d9510e8ceaeb703cab3b19c7114e18e1e4e6c5b9963c165ea9a44aa4f9`를 발행했다.
Sim 이미지는 `elesim/sim:merry-cat` / `sha256:fe30fd0a76fd721f36ded1e5d26457cf132cf87f41913591bd27166832e910e6`다.
별도 측정 컨테이너에서 실제 Genesis/Quadrants 버전과 캐시 경로를 확인했다.
기존 runtime의 release pin과 컨테이너를 교체하지 않고 새 이미지를 검사했다.

서버의 단일 RTX A6000에서 같은 GO2+arm/고정 대상, dt 0.01/2 substeps,
1280×720 카메라 둘로 측정했다. 매 cold 실행마다 Genesis/Quadrants/Numba/CUDA
캐시를 별도의 빈 경로로 지정했다. Python import/컨테이너 시작은 startup 타이머
밖이며, startup은 assets부터 두 카메라 첫 프레임 확인까지다.

| 실행 | 물리 build | Hand-eye / observer build | Startup |
| --- | --- | --- | --- |
| 1.4.5 cold 1 | 150.07초 | 34.09 / 32.61초 | 257.54초 |
| cold 2 | 146.52초 | 31.02 / 30.36초 | 249.94초 |
| cold 3 | 144.16초 | 31.41 / 30.31초 | 248.42초 |
| 1.4.5 cache 재사용 | 4.97초 | 1.66 / 1.74초 | 24.81초 |
| 1.4.1 cold 기준 | 130.59초 | 28.48 / 28.47초 | 240.66초 |

카메라는 병렬 build이므로 합산하지 않는다. 1.4.5 cold 물리 build 중앙값은
146.52초로, 이 환경에서 버전업만으로 첫 시작 지연이 개선됐다는 근거는 없다.
구버전 기준도 같은 현재 앱 소스·설정을 사용했으며 protocol 해시는 일치한다.
Sim의 해시 차이는 후보 runtime에서 제외된 미사용 `rl/` source뿐이었다.

별도의 빈 캐시에서 물리 `Scene.build()`에만 cProfile을 적용했다. 카메라 worker는
제외했고 물리 설정은 유지했다. 프로파일 오버헤드가 포함된 build는 **185.62초**다.
아래 값은 이 별도 실행의 호출 경로별 시간이며 일반 실측 150.07초의 분할값이 아니다.

| 호출 경로 | 시간 |
| --- | --- |
| 최초 step → 충돌 판정 `Collider.detection` | 80.44초 |
| 최초 step → 제약조건 풀이 `func_solve_decomposed` | 39.10초 |
| 최초 step → 나머지 동역학/적분 커널 | 26.29초 |
| Simulator/rigid solver 구성 `Simulator.build` | 26.23초 |
| Visualizer 구성 `Visualizer.build` | 10.18초 |
| 상태 저장/복구 `Scene._reset` | 3.37초 |

최초 step 전체는 145.83초다. 설치된 Genesis 소스도 `Scene.build`가
`Compiling simulation kernels` 단계에서 `_sim.step()`을 실행함을 확인했다.
다른 분류로 보면 전체 build 중 Quadrants kernel 경로가 160.96초이며, 그 안에서
`materialize`가 73.11초(AST 변환 72.16초 포함), `launch_kernel`이 87.80초다.
이 수치는 위 호출 경로와 중첩되므로 합산하지 않는다. Python cProfile만으로는
`launch_kernel`의 native 컴파일/실행 내부를 더 나눌 수 없다. 따라서 라이브러리
import가 아니라 충돌/제약 커널의 최초 컴파일 경로가 다음 최적화 대상이다.
GO2 controller 초기화 16.21초(MPC JIT 9.00초 포함)와 모델 로딩 7.22초는
일반 cold 1 실행의 별도 단계이며 위 물리 build 시간 밖이다.

새 이미지 Sim runtime **909 passed / 3 skipped**. 기본 이미지에 선택적 학습
의존성이 없어 RL suite는 제외했다. 기존 버전에서도 실패한 cross-role FK 비교는
두 앱 source root가 명시된 system gate로 옮겼고, DDS fake client의 discovery
fixture와 진단 사용자 환경을 보완했다. 해당 system/matrix **9 passed**다.
짧은 trial이 첫 EGL frame 전에 끝나던 측정기를 수정하고 대기/실패/timeout 및
step-cost 회귀 **6 passed**를 확인했다. 최초 실패 원자료는 서버에 보존한다.
Host release **71 passed**, scene artifact **5 passed**이며 host의 Genesis 미설치로
runtime import를 요구하는 다섯 모듈은 collection에서 중단됐었다.

초기 image build는 여유 공간 2 GiB guard에서 중단했다. 사용자가 builder prune으로
공간을 확보한 뒤 같은 정식 발행 경로를 재개해 완료했다. 자동 image/container
collector와 진단 container 자동 삭제를 제외했다. 시작 전 기존 23개 컨테이너의
ID/image/name/status가 모두 보존됐다. 일부 camera 종료에서 Quadrants cache-lock
warning이 있었으나 cache 재사용 실행도 완료됐다. 실제 Robot, 다중 호스트 DDS와
연결된 WebRTC viewer 검증으로 확대하지 않는다. 요약은
[`genesis_startup_20261029.json`](../workbench/evidence/curated/sim/genesis_startup_20261029.json)에
있으며 원자료는 서버 `/home/hckang/ws/elesim-genesis145-20261029/`에 보존한다.

### 코드 리뷰 후속 (2026-10-29)

자동 scan과 모델 framework, 제어/표시 경계 및 설치/release 변경을 이 세션에서
검토하고 재현 가능한 결함 두 개를 수정했다. 최초 외부 CLI 실행은 전송 승인
부족으로 거부됐으나 사용자의 명시적 승인 후 `codex review --uncommitted`를
읽기 전용으로 실행했다. 검토 중 대상 170개 파일의 해시가 유지됨을 확인했다.

- **P2, scan 취소 경쟁 조건:** tick의 초기 취소 검사 뒤 다른 thread가 취소하면
  해당 tick에서 q 명령이 한 번 더 전송될 수 있었다. 전송 직전에 취소 상태와
  session 활성 여부를 condition lock 안에서 확인하고 취소 phase로 종료한다.
- **P2, Sim 표시 대상의 명시적 고정값 덮어쓰기:** `presentation_target`이
  `runtime.mirror_robot_id`를 다른 Robot으로 바꿀 수 있었다. 설정된 ID는 유지하고
  discovery 기반 선택만 Pilot이 바꿀 수 있게 했다. 같은 고정 ID의 새 live boot는
  기존 identity/sequence 검사를 거쳐 갱신할 수 있다. Wire major/field는 유지한다.

두 문제 모두 수정 전 실패를 기록하고 회귀를 추가했다. 수정 후 Pilot **619 passed,
27 skipped, 3 subtests**(Shapely 세 모듈 제외, 기존 warning 1개), Sim mirror/endpoint/
distributed config와 계약 **48 passed**, 앱 배포 경계/layout/matrix **20 passed**다.
변경된 Python **123개**의 AST 구문 검사와 `git diff --check`도 통과했다. 실제 RMW,
GPU/Genesis와 Robot 검증 범위는 아래 scan 단계의 미실행 gate와 같다.

외부 리뷰의 추가 지적 세 개도 수정 전 실패를 재현하고 수정했다.

- **P1, 기존 UI release pin의 기동 불가:** 새 점검 flag를 모르는 구버전 UI에
  이를 강제로 전달했다. Python entrypoint에서 이미지의 graphics helper 유무를
  확인한다. 구버전은 점검 미지원 로그와 함께 원래 기동을 유지하며, 지원되는
  helper가 실패하면 기동을 차단하고 임시 X11 grant를 정리한다. Image pin은 유지한다.
- **P2, 상태 조회 중 GPU 선택 초기화:** `checking` placeholder를 새 정책으로
  처리해 10초마다 선택값이 기본값으로 돌아갔다. 점검 중에는 마지막 정책·기기
  목록·입력 초안을 유지하고 launch/edit는 `runtimeReady`로 막는다. 실제 unreachable
  응답은 기존처럼 stale 정책을 폐기한다.
- **P2, 역할 이동 뒤 GPU/Viewer 설정 잔존:** 호스트에서 빠진 역할의 정책을
  새 instance에 복사해 validation이 실패했다. 명시적 launch option 유무와 관계없이
  현재 역할의 GPU 정책만 남기고 Sim이 없는 호스트에서는 Viewer도 해제한다.

추가 회귀를 포함한 관련 Python 설치/배포 검사 **277 passed**, JavaScript **8 passed**,
변경된 Python **124개**의 구문 검사와 `git diff --check`가 통과했다. 실제 Docker 실행과 UI/OpenGL 구동은
이 host fake-process 결과로 입증하지 않는다. 세 수정과 해당 회귀만 대상으로 한
외부 재리뷰에서 추가로 조치할 회귀는 발견되지 않았다. 재리뷰는 읽기 전용 소스
검토이며 테스트를 재실행하지 않았고, 위 277/8개는 이 작업 환경에서 실행한 결과다.

재현·수정 후 로그와 구조화된 결과는
`workbench/evidence/generated/auto-review/`에 보관한다.

### Sim 자동 다중 시야 scan (2026-10-29)

기존 연구용 시점 계획기와 점군 통합을 Pilot 실행 경로에 연결했다. UI의
`Object Scan (Sim)` 시작/취소 → Sim의 leased 4축 이동 → 새 피드백 세 번으로
정착 확인 → 새 coherent DDS RGB-D → instance mask/다중 시야 통합 → 모델 API 1의
completion/mesh/선택적 계획 → immutable scene bundle과 `scan.json` 저장까지
구현했다. 원본 NPZ와 시점별 silhouette PNG/NPZ도 남긴다. 설정은 Pilot의
`scan.example.yaml` 또는 `--scan-models`이며 설치/release에 포함한다.

실제 운용 관절 범위와 고정 base를 지키고 arm self-collision 및 관측된 주변
표면을 이동 구간 전체에서 검사한다. 관절 추종 오차도 separation 여유에 포함한다.
모델/탐색 worker와 DDS motion tick을 분리하고 boot/lease/Sim epoch 변경,
500 ms 피드백 지연·제어 지연, 추종 실패·취소 시 후속 명령을 차단한다.
다른 팔 작업과 동시 실행하지 않으며 Robot 대상은 거부한다.

`view_budget`/`no_safe_distinct_view`/`information_saturated`를 구분하고 UI에
관측 수, 점 수, 시야 방향 proxy와 미관측 방향을 표시한다. 숨겨진 표면의
완성을 주장하지 않는다. 처음 대상이 보이도록 조준해야 하며 물체/base는 고정이다.
물체 뒤집기·집기·base 이동, 학습 completion의 정확도, 동적 mesh 교체·자동
multi-host 전달·계획 실행은 이 작업에 포함되지 않는다. 실사용 순서와 artifact는
[`model_integration.md`](model_integration.md#sim-자동-다중-시야-수집)를 따른다.

Host 검증은 실제 FK/시점 계획기/충돌 모델과 analytic RGB-D 센서 fixture,
fake DDS/카메라의 live session, UI→Pilot 별도 프로세스 계약을 사용한다.
실제 RMW/GPU/Genesis 동작을 입증하지 않는다. 현재 설치의 generated dev
wrapper/container가 없고 host에는 ROS 2/Genesis/Torch가 없어 canonical
required/extended, release build/isolated verify 및 real-RMW topology smoke를
실행하지 못했다. Robot은 사용할 수 없다는 합의에 따라 시험하지 않는다.
원시 로그는 `workbench/evidence/generated/active-scan/`에 보관한다.

현재 변경의 host 결과:

- Pilot **618 passed, 27 skipped, 3 subtests**. Shapely가 필요한 세 모듈은 제외했고,
  기존 nonfinite preview 시험의 NumPy warning 1개가 있다. Scan의 취소·정착·freshness,
  운용 상한과 여러 물체의 일관된 재정렬 회귀를 포함한다.
- UI **152 passed**. AV가 필요한 `test_zed_preview.py`는 제외했다.
- Protocol **177 passed, 5 skipped, 5 subtests**.
- Sim telemetry/artifact·별도 프로세스·실제 FK scan·quality matrix **19 passed**,
  layout **4 passed**. Analytic sphere fixture의 0.4 m 배치에서 3개 시점/3개 채택,
  관측점의 구 표면 오차 p95 **0.333 mm**였다. 0.3 m 배치는 추종 오차 envelope과
  겹쳐 다음 이동을 거부하고 1개 시점만 보존했다. 실제 렌더/mesh 품질 수치는 아니다.
- Configuration/bootstrap **129 passed**, release tool self-tests **71 passed**.
  실제 image/release build 결과는 아니다.
- Shipped scan model factory/hash preflight, compileall과 `git diff --check` 통과.

### 외부 모델 연결 framework (2026-10-28)

다른 작업자가 개발하는 모델을 core workflow 수정 없이 연결하기 위한 첫 실행
경계를 구현했다. Pilot model API 1의 pose, ordered pose correction, instance
segmentation, completion, mesher, optional planner를 YAML factory/asset SHA-256으로
선택한다. 기록 관측 iterable에서 물체별 점군·mesh와 미실행 계획안을 생성하는
Python API 및 `elesim-reconstruct` CLI가 있다. 물체 ID/frame/단위, 관측과 추정,
source boot/calibration, plan 시작 q/limits/contact 역할을 검사한다.

Scene artifact schema 1(JSON/OBJ/NPZ)은 immutable 파일 bundle이다. Sim은 선택한
bundle을 build 전에 고정 geometry로 읽고 물리 scene과 두 camera replica의
revision을 일치시킨다. 설치/release에 adapter runner와 경로를 보정한 baseline
예제를 포함한다. 외부 모델 제공 형식과 실행 방법은
[`model_integration.md`](model_integration.md)를 따른다.

이 범위는 trained IMU residual/AdaPoinTr/일반 instance tracker/접촉 planner의
완성 선언이 아니다. Shipped reconstruction baseline은 HSV/FK, 명시적
ObservedOnly, voxel surface다. 실시간 DDS 수집 연결, 자동 scan과 UI 실행 경로는
위 10-29 후속에서 구현했다. Multi-host bundle 전송, 실행 중 mesh 교체·동적
attachment·계획 실행은 남는다.
Robot raw IMU 보정/native 입력 연결도 별도 미완료다. Robot IMU 슬롯은 기존
JSON/C++ graph 계약을 유지한다. 저장 공간/고급 cache 정책은 보류한 그대로다.

Host 검증은 외부 모델 fixture와 fake Genesis를 사용한다. 별도 Pilot CLI →
Pilot을 import할 수 없는 Sim 소비 프로세스까지 검증하되 실제 GPU scene build,
렌더·충돌 접촉, 모델 정확도, Robot 동작과 canonical isolated release gate는
입증하지 않는다. 현재 설치의 생성된 개발 wrapper/container가 없고 host에는
Genesis/Torch/ROS 2가 없어 canonical required/extended, release build/isolated verify,
실제 RMW topology smoke는 실행하지 않았다. 기존 readability baseline 초과도
이 작업으로 해결했다고 선언하지 않는다.

10-28 framework 단계의 host 결과(최신 scan 검증과 구분):

- Pilot **590 passed, 27 skipped, 3 subtests**. Shapely가 필요한 세 모듈은 제외했다.
  기존 nonfinite preview 입력 시험의 NumPy warning 1개가 있다.
- Protocol **177 passed, 5 skipped, 5 subtests**. 새 scene artifact 계약 시험 포함.
- 재구성 Sim helper/config + 독립 소비 프로세스 **6 passed**; contract/matrix **26 passed**.
- Configuration/bootstrap/container installer **218개 통과**. 처음 sandbox의 Unix
  socket bind로 막힌 5개는 로컬 socket 허용 후 별도로 통과했다.
- Release tool self-tests **70 passed**; example model preflight, compileall과
  `git diff --check` 통과. 실제 release image build/격리 설치 결과는 아니다.

원시 로그는 `workbench/evidence/generated/model-framework/`에 보관한다.

### plzread 현장 수정의 저장소 적용 (2026-10-28)

현장에서 원격 설치 후 실제 작동한 EleSim과 `plzread.md`의 수정 근거를 정식
소스·설정·설치 생성물·release 입력에 반영했다. 현장 디렉터리의 최종 코드와
후속 미완료 요구사항을 적용하되, 아래 사용자 확정값이 과거 매핑/상한보다
우선한다. 이 기록은 R0–R5의 실기기·실네트워크 수용 완료 선언이 아니다.

**완료 범위 재대조: 전체 목표 달성은 아직 아님**

핵심 현장 diff의 소스 이식과 host 회귀 통과를 모든 현장 문제의 해결 완료로
확대해서는 안 된다. Robot을 사용할 수 없다는 합의된 제약 외에도 다음이 남는다.

- `_ensure_venv()`는 새 venv를 base interpreter로 생성하지만 이미 존재하는
  venv의 private-cache 경유 링크와 `pyvenv.cfg`를 자동 수리하지 않는다.
  현장에서 수행한 기존 venv 수리까지 일반 update가 재현하는 것은 아니다.
- 용량 검사는 prefix의 payload 파일 복사 공간을 추정한다. Docker daemon의
  CUDA 이미지 빌드 공간·빌드 캐시 예산 확인/안내까지 완성한 것은 아니다.
- 기존 Robot의 명시적 calibration/domain을 보존하므로 새 소스 기본값만으로
  기존 설치가 1.5°/mm·260° 및 EleSim 1 / Unitree 0으로 일괄 전환되지 않는다.
  설정과 모델을 함께 적용하는 실제 upgrade 절차는 아직 검증하지 않았다.
- 새 Jetson 이미지 build, 격리 release 설치 및 실제 DDS/두 영상 경로는
  미검증이다. Wi-Fi/SSH 간헐 장애의 근본 원인을 해결했다는 근거도 없다.

아래 표의 “정식 반영”은 각 코드 변경의 적용 위치와 동작을 뜻하며 위 미완료
범위의 완료 증거는 아니다. 전체 완료 판단은 이 항목과 검증 경계를 함께 따른다.

**확정한 리니어 기준**

- **1.5 motor degrees = 1 mm, max = 260 degrees.**
  `q=-u/1500` m, q 범위 `[-0.17333333333333334, 0]` m다.
- Pilot·Robot·Sim 기본값, 공통 변환, ZED Mini/D435 blueprint·URDF·bundle,
  IK 모델과 planning artifact를 맞췄다. 기존 259°=230 mm 및 그 위의 199°
  상한은 이번 기준으로 대체한다. 별도 사용자 상한은 같은 환산비로 q를 제한한다.
- Planning의 충돌 구·속도·다른 관절은 유지했다. 재사용 경로는 기존 artifact와
  source/mesh hash, 동일 geometry/FK/속도 및 linear 구간 축소를 검사한다.
  변형·확대·손상은 거부하며 provenance를 metadata에 기록한다.
- oct27 policy와 manifest는 원본 그대로 복구했다. Pilot은 학습 좌표를
  재환산하지 않고 출력 범위를 설치 한계와 교집합으로 제한한다. Home이 그
  밖이면 시작을 거부한다. 이 제한에서 실제 정책 성능은 장비 확인 대상이다.

**반영 위치와 동작**

| 현장 항목 | 정식 반영 |
| --- | --- |
| private bootstrap Python으로 인한 native venv 접근 실패 | native installer가 실제 base interpreter 사용 |
| YAML 교체 후 서비스 ACL 소실 | 원자적 설정 교체와 network/refresh rollback의 mode·POSIX ACL 보존; ACL 복사 실패 시 기존 파일 유지 |
| net configure 후 Robot 호스트 설정 유실 | state v12 `robot_host`, installed YAML 이관, 사용자/NIC/workspace/domain 및 arm/mapping/safety·장치 보정 보존 |
| tools profile 이미지 삭제·재빌드 | cleanup이 모든 Compose profile의 현재 이미지 보호; dev/foreign 소유권 경계 유지 |
| Jetson 디스크 부족 | 설치/release 파일시스템의 payload 복사 여유 공간 선검사와 위치·필요량 보고; 자동 prune 없음 |
| cuRobo/모델·정책 설치 누락 | bootstrap·context·설치·release에 planning/world/Wrap 입력 포함; 고정 commit cuRobo와 hash 확인 패치를 Orin 이미지에서 빌드 |
| Torch/cuSolver graph fallback 실패 | cuSolver 11.7.1.2 DSO를 hash 확인해 private loader 경로에 배치; planner 초기 CUDA SVD 확인 |
| sphere world/경로 최적화 | 초기·IK·경로 world 변환, soft self-collision sphere당 2 mm 여유, hard/독립 보간 검사 유지; 12회/120 s/graph attempt 2, 외부 180 s watchdog과 결과 로그 |
| 측정 처짐이 다음 명령 목표를 덮어씀 | 측정 HostState와 마지막 명령 q 분리; target/lease/boot·토크·거절 시 캐시 폐기 |
| Robot 상실 후 Sim 자동 제어 | Robot 우선 및 단절 중 기억; Sim-only graph는 유지. Robot discovery 뒤 Sim motion 거부 |
| Robot 실측을 Sim에 표시 | capability 협상 `presentation_target`으로 선택 Robot ID/boot 전달, 정확한 source boot/sequence 검사, GO2 kinematic 표시와 마지막 실측 유지 |
| GO2 domain/다리 측정 누락 | 신규 EleSim domain 1 / Unitree domain 0; 호스트 NIC는 별도 저장; leg dq/추정 토크 전달, 미수신 값 생성 안 함 |
| 두 UI 창 입력·CPU 사용 | 창별 ImGui callback context와 복원; 카메라 30 Hz, 제어 60/20 Hz와 event wait, opt-in 통계 |
| 제어 대상과 영상 선택 결합 | Control Target / Camera View 분리, system/Robot별 좌우 원본 RGB-D 직접 구독 및 기존 SROS2 read 권한 확인 |
| UI X11/renderer | host UID/GID, 설치 시 cookie 고정 대신 session ACL, DDS 전 GL/vendor 사전 검사, 소유한 ACL만 복구/회수 |
| UI GPU 선택·보존 | Connections·CLI·instance `role_compute.ui`에 software/NVIDIA 및 장치 선택; 기존 ui-rendering.json 읽기 이관 |
| Hardware 전류 표 | 이름/숫자 ID 1–5 대응; 실제 Gripper mA가 있을 때만 표시, load 별도 유지 |
| Wrap·보정·기본 목표 | Roll motor +1, Wrap sign +1, Pilot offset -9, Extend Arm offset 적용, lift sleep 연결; 공 XYZ=(.2976,.1682,.2901), Wrap 중심=(.368,.16,.55), radius=.05, height=1.1 |
| Connections 준비 뒤 지연·잘못된 롤백 | 준비 inventory 30 s 재사용, 상태 SSH 20 s 예산, 호스트별 병렬 후속 표시, transient probe 최대 3회 재접속, 음성 결과는 롤백 전 최신 5 s 검사 |
| Wi-Fi 운영 | 절전 off 뒤 재발한 현장 근거 보존; NIC/profile별 진단·명시적 변경·원복 안내. 설치가 네트워크 설정을 자동 변경하지 않음 |
| release 동물 이름/catalog/활성 버전 | 기존 immutable 발행·등록 및 ownership 경로 유지; 현장 release/state/Compose를 다른 설치로 복사하지 않음 |

Protocol major 6을 유지한다. 새 메시지는 `presentation.target.v1`을 광고한
Sim에만 보내 구형 v6 parser에 전달되지 않는다. Registry/payload/문서와
four-process smoke에 같은 계약을 추가했다. 선택 표시 경로는 motion authority를
만들지 않는다. Robot local limit/deadman·bridge 상실 시 arm cleanup은 유지한다.

정책 파일은 commit `1a8913825c45112fe9c50a9bca63da8e7cdb2655`에서 복구했다.
`policy.pt` 198996 bytes / SHA256
`9712131dbe7fa474af7dd36c2fd01dc1830b0b14ce5dac91dfbf70d5bf109683`,
`interface.json` 5606 bytes / SHA256
`55e1807b9af0d36c7a4058b32589921f30594980d8500edfe8755ef482e10442`이며
`payload/data/policies/wrap-grasp/provenance.json`에 원본 URL을 기록했다.

**검증과 적용 경계**

- Host setup 전체 **1,349 passed**. 이후 필수 payload 입력 검증의
  bootstrap/container 집중 재검사 **202 passed**, 상태 timeout 수정의
  secure-deployment 집중 재검사 **124 passed**. 이 집중 검사는 전체 결과와 중복된다.
- Host Pilot **570 passed, 27 skipped, 3 subtests passed**; Robot
  **166 passed, 10 subtests passed**; UI **149 passed**; Sim endpoint/미러/
  telemetry/operator/rate 집중 **44 passed**. Pilot은 Shapely가 필요한
  hug_geometry/mock_hug/stereo_rgbd 3개 파일, UI는 AV가 필요한 zed_preview
  파일을 수집에서 제외했다. Pilot의 NaN 입력 회귀에서 기존 RuntimeWarning 1건이 있다.
- Protocol **159 passed, 5 skipped, 5 subtests passed**; 모델/FK/범위/계약 집중
  **26 passed, 1 skipped**(Protocol과 일부 중복); release 도구 **69 passed**.
  Node host별 응답/선택 보존 행동 검사와 Python·JS·shell 문법,
  `git diff --check` 통과. 원본 출력은
  `workbench/evidence/generated/field-20261028/`에 둔다.
- 추가 정적 검사 **9 passed, 1 failed**. 실패는 클래스 길이 예산 검사다.
  기존 HEAD도 ControlPanel 1040, UiSimSession 1015, PyMpcGenesisController
  1068행으로 1000행 제한을 초과했다. 이번 ControlPanel은 1128행이며 다른
  두 클래스는 그대로다. 이 가독성 정리는 남겨 두며 extended gate 통과로
  기록하지 않는다. Layout 경계·고정 cuRobo 패치·smoke 누락 의존성 검사는 통과했다.
- 사용할 수 있는 현재 설치의 generated dev wrapper/고정 Docker daemon이 없다.
  canonical required/extended, release build/isolated verify, real-RMW DDS
  topology·RGB-D·encoded WebRTC 및 Genesis/GPU/Jetson 이미지 build는 미실행이다.
  Host Torch/Pinocchio/Shapely/AV/ROS 부재로 해당 owning suite의 일부를
  제외/skip했다. host Python에는 의존성을 설치하지 않았다.
- Robot을 사용할 수 없다는 사용자 조건을 반영한다. native 제어기는 장치 없는
  SDK double로 확인하며 실제 모터/하중/정지 시간·GO2 수신·두 영상·무선 안정성은
  새 검증 결과로 주장하지 않는다. 현장 원격 설치·작동 기록은 그대로 근거로 남긴다.
- 이번 작업은 소스 적용이며 원격 접속·배포·서비스 재시작·모터 명령은 하지 않았다.
  기존 설치의 명시적 보정/domain은 보존한다. 실제 적용에는 동일한 260° 기준의
  Robot/Pilot/Sim 설정과 모델을 새 release/정상 등록 경로에서 함께 선택해야 한다.
  저장소 수정만으로 실행 중 설치의 보정값이 자동 전환되지는 않는다.

### Robot native 설치 조회 버튼 갱신 수정 (2026-10-27)

- 연결 관리자에서 SSH 호스트키를 확인해도 일반 설치 조회 버튼만 갱신되어
  Robot native 버튼은 비활성 상태로 남는 문제를 재현했다. 확인 완료와
  주소·포트·SSH 모드 변경 시 두 설치 버튼을 함께 갱신하도록 수정했다.
- 기존 SSH 응답의 stale 검사와 각 조회의 개별 busy 상태를 유지한다.
  Node 행동 회귀로 native 버튼 활성화·재잠금과 지연 응답 거부를 확인했으며,
  호스트 연결 GUI/행동 집중 검사는 **46 passed**다. HTTP 검사의 loopback
  서버는 샌드박스 밖에서 실행했다. 기존 개발 이미지 준비 미완료로 canonical
  gate는 미실행이며, 실제 설치본 갱신·원격 SSH 접속·서비스 재시작은 하지 않았다.

### oct27 정책 기준 4DOF Linear 범위 적용 (2026-10-27, 아래 범위는 10-28 결정으로 대체됨)

- 사용자 지정으로 공통 Linear 좌표를 `[-0.230, 0]` m로 변경했다. 모터·UI
  표시각은 0–259도를 유지하고 q=0..-230 mm에 선형 대응시킨다. 이전
  1.5도/mm 환산과 -172.666667 mm 끝점은 이번 기본값으로 대체된다.
- Pilot·Sim의 매핑과 RL mechanism은 공통 값을 읽으며 Robot의 명시적 YAML도
  같은 범위로 맞췄다. ZED Mini·D435의 blueprint/URDF/bundle hash, Pilot IK
  모델과 cuRobo artifact를 재생성했다. cuRobo는 기존 원격 CPU 환경에서
  동일 CAD 자산의 hash를 확인하고 전체 builder를 실행했으며 1005 spheres와
  22개 기하 분리 조건을 유지했다. 호스트에 의존성을 추가하지 않았다.
- `oct27@1a89138`의 실제 정책 manifest가 세 역할의 새 설정으로 범위 검사를
  통과하고 259도 ↔ -230 mm가 왕복됨을 확인했다. 정책 파일·기존 설치·release
  pin·실행 프로세스는 변경하지 않았다. DDS wire shape/version도 그대로다.
- 호스트 Protocol·model-builder·Pilot arm/config·Robot/UI config 검사
  **262 passed, 10 skipped, 8 subtests passed**. 별도 범위·wrap·native 제어
  집중 검사는 **86 passed, 22 skipped, 5 subtests passed**이며 일부 중복된다.
  `git diff --check` 통과. 호스트 Torch/Genesis 부재로 RL arm-limit 및 Sim
  runtime config 시험은 수집할 수 없어 위 검사에서 제외했다. 기존 개발 이미지
  준비 미완료로 canonical required/extended·real-RMW·isolated-release gate와
  실제 GPU 렌더링·물리 동작은 미실행이다.

### Orin Pilot CUDA 12.6 이미지 분리 (2026-10-27, 이미지 실검증 진행 중)

- AGX Orin / L4T 36.5 / JetPack 6.2.2의 첫 Pilot release가 마지막 `pip check`에서
  `nvidia-cusparselt-cu13 0.8.1 is not supported on this platform`로 실패했다.
  기존 Dockerfile이 JetPack 6에도 PyPI Torch 2.12.1/CUDA13을 설치한 것이 원인이다.
- L4T 36.4/36.5의 GPU Pilot context는 Jetson AI Lab의 CUDA12.6 Torch 2.11.0 /
  torchvision 0.26.0 wheel을 URL·SHA256으로 고정한다. 두 원본을 직접 받아 hash와
  metadata를 검증했고 대상 호스트의 기존 wheel과도 동일하다. CUDA12.6·cuDNN은
  NVIDIA의 signed Jetson repo, cuDSS 0.8.0은 hash 고정 aarch64/CUDA12 archive를 쓴다.
  CPU와 일반 host lock은 유지하며 알려지지 않은 Jetson GPU release는 먼저 거부한다.
- Host setup **1,331 passed**다. source 보존·context fingerprint·outer-host release 전달·
  다운로드/서명키 hash 실패 시 중단을 확인한다. 대상 호스트의 기존 Torch 조합은
  실제 Orin SM8.7에서 GPU 행렬 연산과 CUDA NMS를 통과했다. 이는 새 이미지의
  성공 증거와 구분하며 정식 update/build 및 이미지 GPU 검증은 진행 중이다.
- 정식 update의 host bootstrap에서 operation-lock helper의 maintenance
  `PYTHONPATH`가 새 setup을 가려 `elesim_setup.cli` import를 막는 문제도 재현했다.
  host bootstrap과 새 setup subprocess에서 상속 Python 경로를 분리하고
  사전검사 오류에 실제 stderr를 남긴다. bootstrap/lock/update/Jetson 집중 회귀
  **142 passed**이며 lock descriptor와 보유 검사는 변경하지 않았다.
- JetPack CSV hook은 `--gpus all`만 전달하면 컨테이너 시작을 거부하는 것도
  실제 호스트에서 재현했다. 같은 이미지의 `/bin/true`가 `--runtime=nvidia`를
  함께 지정하면 통과했다. GPU Compose와 이후 tools 내부 scoped 등록에도
  같은 runtime을 사용하며 CPU·device_ids·CUDA index 규약은 유지한다.
- 원격 Robot 두 service와 Docker container가 모두 정지/없는 상태임을 기록했다.
  모터나 DDS 제어 명령은 보내지 않는다. canonical dev 환경의 기존 공백은 유지한다.

### Jetson Robot 설치 source 누락 수정 (2026-10-27)

- Git에 존재하는 `native_arm/`을 bootstrap archive 추출 목록에서 누락해
  Robot 설치가 `Installation source is incomplete`로 중단되는 문제를 재현했다.
  C++ controller와 bundled Dynamixel SDK tree를 보존하고, Linux 빌드에 필요한
  소스·헤더·라이선스 **32개**를 snapshot 완료 검사의 필수 입력에 넣었다.
- 기존의 같은 Git revision 캐시에서도 이 tree가 빠지면 재사용하지 않고
  재다운로드한다. HTTP 304 뒤 무조건 재요청·캐시 복구와 native tree 내부의
  symlink 거부를 회귀로 확인했다. 설치 경로의 SDK 추가 다운로드는 없다.
- Bootstrap **110 passed**이며 추출한 소스만으로 host C++ shared library를
  실제 빌드했다. 보고된 `0ac1626`의 실제 Git archive도 새 추출기로 처리하면
  native **39파일**이 보존되고 Robot installer source 사전검사를 통과한다.
- Host setup 전체 **1,313개 통과**다. 샌드박스 실행의 1,299 passed와 소켓
  권한/입출력 실패 14개를 권한 허용 후 재실행한 14 passed를 합친 결과다.
  실제 Jetson ARM64/ROSIDL 빌드·systemd 등록·기기 동작은 이번에 실행하지 않았다.
  기존 dev 준비 미완료에 따른 canonical gate 공백은 아래 기록과 같다.

### Workbench 미사용 도구 보관 (2026-10-27)

- 구형 실행기·결과 전용 그림·height/mount 실험 CLI **24개**를 로컬 ZIP으로
  보관했다. 앞선 통합에서 제거한 다섯 파일도 기록된 HEAD에서 원문을 회수했다.
  모든 원문/ZIP의 SHA256을 확인하고 원래 연구 경로에서 제거했다. ZIP은
  `.gitignore` 대상이며 [목록·이유·hash](../workbench/archive/index.json)만 Git에 남긴다.
- 현재 연구 source/시나리오는 **35개**, 공통 recipe는 **9개**다. 품질·release와
  유지하는 수동 진단까지 [Workbench 목록](../workbench/README.md)에 정리했다.
  문서에서 archived CLI를 현재 실행 명령으로 안내하던 부분도 수정했다.
- 회귀에 필요한 옛 계산부 여섯 개는 `tests/research/support/`에 CLI 없이 유지한다.
  함수/클래스 AST가 원문과 같고 기존 assertion은 그대로다. source/runtime/
  테스트 어디에서도 archive를 import하지 않는다. raw evidence는 옮기지 않았다.
- ZIP을 임시로 없앤 상태의 연구·품질 검사에서 **174 passed, 1 skipped,
  기존 readability 1 failed**다. 같은 원자료의 scan 비교도 paired 품질 **5/6**으로
  같은 실패를 유지한다. 검사 후 ZIP을 동일 hash로 복구했다. GPU/실기동 및
  canonical 환경의 기존 검증 공백은 아래 개편 기록과 같다.

### Workbench 벤치 실행·판정 개편 (2026-10-27)

- 소프트웨어 검사는 기존 quality 매트릭스를 유지하고 GUI도 같은 목록/명령을
  사용한다. 연구 analysis/experiment의 빠진 Sim source root를 보완했다.
  protocol 안의 cross-role/model/RL assertion 세 개는 내용 그대로 시스템
  `arm-operating-range` 게이트로 옮겨 protocol의 sibling import를 제거했다.
- 무거운 실험은 보행/gaze·Sim 성능·시각 보정·다중 시점·형상 완성의 다섯 분야와
  `python3 -m workbench.tools.bench` 실행기로 묶었다. JSON plan, 실행 환경과
  source/image/config/model/checkpoint hash, 각 stage/episode 시간, raw artifact
  checksum, 검증·보고·archive 수집을 기록한다. YAML extends도 추적하고 실행 중
  입력 변경을 거부한다. `complete`와 `accepted`를 분리해 실패를 exit 0으로
  숨기지 않는다. 중단/timeout/고아 child와 자기 작업만 정리하는 회귀가 있다.
- 깨진 shell/preview entrypoint 다섯 개를 scenario JSON과 하나의 suite worker로
  통합했다. 실제 동작이 없던 host-restart/standoff/video/preroll 옵션은 제거했다.
  보행 판정은 contact CSV 존재만으로 통과하지 않고 이동·넘어짐·기록 구간을
  확인한다. Sim/Pilot 원본 metadata와 기존 안전 회귀를 보존했다. 정상 서비스에
  접속하는 대신 연구용 gaze는 별도 loopback DDS identity/topic을 사용한다.
- Host 검사는 **336 passed, 6 skipped, 1 failed**다. 남은 실패는 HEAD에도 동일한
  `PyMpcGenesisController` 1,068줄, `ControlPanel` 1,040줄, `UiSimSession` 1,015줄이
  기존 class 1,000줄 기준을 넘는 readability 검사다. production 세 파일과 검사
  기준의 HEAD/current hash가 같음을 기록했다. 기준을 바꾸거나 xfail하지 않았다.
- CPU 해석 시각 보정 전체 pipeline은 통과했고 원자료 재평가 JSON은 정확히 같다.
  새 동시 producer/consumer scan도 **39회 획득·39구간 충돌 감사**까지 완료했으며
  형상 기준 미달은 실패로 남겼다. 이전 GPU 결과의 비교도 **112→39회, paired 품질
  5/6**으로 같은 실패를 반환한다. 이 실패 archive의 pack/수집/hash 검증을 통과했다.
  CPU analytic scan을 기존 Genesis GPU의 형상 수치로 대체하지 않는다.
- 과거 generated code **1,988파일/731개 내용**을 삭제 없이 색인했다. 원자료 경로와
  기존 증거 링크를 유지한다. 새 executor의 원격 GPU2/Genesis·실제 DDS gaze·
  AdaPoinTr GPU·manager SSH 수집·GUI 표시 자체는 live 미검증이다. 기존 생성 dev
  준비 미완료로 canonical required/extended·real-RMW·isolated release 전체
  게이트도 미실행이며, 보행/시각 모델 개선이나 Sim 속도 개선을 주장하지 않는다.
  [명령·범위·이전 경로 정리](benchmarks.md),
  [검사·실패 보존·hash 근거](../workbench/evidence/curated/experiments/workbench_cleanup_20261027.json).

### 다중 시야 scan 추가 제약 (2026-10-27, 충돌 감사 통과 / 시야 변경 69% 감소)

- `e98b07b` push와 양쪽 정식 update 후 원격 GPU2 A6000에서 비교했다.
  같은 1,005 sphere·2 mm 여유·Linear ±4 mm/Roll ±0.06 rad/Bend 각각
  ±0.018 rad 오차 구간을 두 정책에 적용했다. 후보 끝점과 이동 구간 전체를
  joint-motion Lipschitz bound로 확인하고, 전송 직전에도 재검사한다.
  Digest가 다른 모델, 위험한 시작점, 안전한 끝점 사이의 충돌, 검증 예산
  초과를 모두 거부하는 회귀가 있다. 실제 URDF의 독립 FK와 계획 FK도 일치한다.
- 최대 20회 각도 대조군은 **18/20/18/18/20/18회, 합계 112회**,
  정보량·조기 종료 정책은 **4/5/4/7/12/7회, 합계 39회** 촬영했다.
  최초 관측 후 시야 변경은 **106→33회(68.9% 감소)**다. 첫 자세의 재촬영은
  기본 정책에서 생략한다. 모든 조건의 촬영 수가 줄었으며, 정답 형상은
  시야 선택·통합 소비자에게 주지 않았다.
- 실제 joint offset을 적용한 **151구간**의 연속 sphere 분리 감사는 두 실행
  모두 통과했다. 정상 test Sim/Pilot의 ID·이미지·정지 상태를 보존했고 이번
  진단 컨테이너는 모두 제거했다. 보수적 arm/받침대/열린 claw 모델의 결과이며
  모델 밖 GO2/케이블/환경이나 동역학 추종 오차까지 인증한 것은 아니다.
- 표면 p95는 새 정책이 여섯 조건 모두 작고 수집률 감소는 최대 **2.54%p**다.
  그러나 사전 paired 품질 기준은 **5/6 통과**다. 일반 box의 높이 오차가
  **9.98→14.75 mm(+4.76 mm)**로 증가해 축별 증가 ≤3 mm 기준을 넘었다.
  기준을 완화해 통과로 바꾸지 않았다. 전체 형상 기준은 두 정책 모두
  **3/6 통과**이며, 관절 오차가 있는 얇은 box에서는 새 관측의 연결성 부족으로
  통합 fit을 거부하고 첫 유효 점군을 보존했다. 형상 확보 완료를 주장하지 않는다.
- 관련 host 회귀 **94 passed, 4 skipped**다. 선택적 GPU adapter 의존성 검사와
  기존 dev image 준비 미완료로 실행하지 못한 canonical required/extended,
  real-RMW·isolated release gate를 구분한다. 비교 원자료와 사진 **731파일**을
  내려받아 SHA256을 검증했다. 실행용 스냅샷에서 빠진 planning 자료 세 개는
  같은 Git 해시로 별도 stage하고 artifact/source digest를 확인했다. 두 초기
  진단 실행의 입력 누락·실행기 변수 오류는 촬영 분모에 넣지 않는다.
  [절차·품질 예외](multiview_scan.md#추가-조건-self-collision과-촬영-횟수),
  [비교·충돌·SHA 증거](../workbench/evidence/curated/experiments/arm_multiview_safe_20261027.json),
  [횟수·수집률·표면 오차](../workbench/evidence/curated/experiments/arm_multiview_safe_comparison_20261027.png),
  [새 시야의 실제 RGB](../workbench/evidence/curated/experiments/arm_multiview_safe_views_20261027.png).

### 4DOF 다중 시야 점군 scan (2026-10-27, 획득·통합 검증 / 전체 형상 4/6 통과)

- 관측 중심·범위와 명목 FK로 합법적인 다음 시야를 고른다. 첫 optical frame을
  기준으로 하고 원본 RGB-D는 보존한다. 독립 ICP만으로 면이 두꺼워지는 실패를
  재현해, 최종 통합은 여러 관측의 **공통 bend 영점 두 축**을 추정하고 상대 FK를
  재계산한 뒤 3 mm voxel로 묶는다. 같은 자세만 반복한 관측성 부족도 거부한다.
- `d6e33b6` push와 양쪽 정식 update 후 원격 GPU2 A6000에서 actual arm URDF와
  장착 camera로 **6 episode·120 RGB-D**를 촬영했다. Box/얇은 box/cylinder의
  정상·관절 오차 조건이며 depth 잡음과 dropout도 포함한다. 실제 GPU UUID
  하나만 보였고 OpenGL도 A6000/software=false였다. Consumer는 센서/encoder와
  명목 model만 읽고 scene/catalog/true pose는 마운트하지 않았다.
- 전체 정답 표면의 수집률은 첫 시야 **34.6~46.8% → 통합 53.1~79.0%**다.
  관절 오차 세 조건의 표면 p95는 명목 FK 누적 **15.50/19.39/17.47 mm →
  3.80/4.00/3.95 mm**이며 precision@5mm는 약 99%다. 최대 치수 오차는
  **1.37/1.67/2.46/1.92/1.37/7.08 mm**였다. 주입한 Bend +0.8/−0.6도는
  약 +0.78~0.81/−0.60~−0.61도로 추정했다.
- 사전 기준은 **4/6 통과**다. 두 실패는 수집률 53.1/54.4%로 60%에 미달한
  경우이며 기준을 완화하지 않았다. 추가 시야 선택과 미관측 면 확보가 남는다.
  명목 world anchor는 관절 오차 조건에서 약 19.5 mm/3.16도 틀리므로 절대
  위치 4 mm 정확도라는 뜻이 아니다. 정적 물체·고정 base·일정한 관절 오차의
  kinematic 시험이며 일반 segmentation·동역학·연속 충돌 안전·runtime
  자동 스캔 UI·motion/DDS 연결은 검증 범위 밖이다.
- 관련 host 회귀 **71 passed**다. Canonical dev 준비 미완료인 기존 사유로
  required/extended·real-RMW·isolated release gate는 실행하지 않았다. 원자료
  **550파일**을 노트북으로 가져와 SHA256을 모두 검증했다. 기존 test Sim/Pilot의
  ID·이미지·정지 상태는 유지했고 진단 컨테이너는 제거했다.
  [로직·재현·평가 한계](multiview_scan.md),
  [수치·격리·SHA 증거](../workbench/evidence/curated/experiments/arm_multiview_scan_20261027.json),
  [실제 팔 자세](../workbench/evidence/curated/experiments/arm_multiview_arm_20261027.png),
  [장착 카메라 시야](../workbench/evidence/curated/experiments/arm_multiview_views_20261027.png),
  [처음·명목 FK·보정 누적·정답](../workbench/evidence/curated/experiments/arm_multiview_clouds_20261027.png).

### AdaPoinTr 체크포인트·입력 규약 재시험 (2026-10-27, 기존 box replay 중앙값 기준 통과)

- 공식 cropped ShapeNet-55 URL과 저자가 이슈 #88/#132에서 안내한 URL은 같은
  만료된 링크다. 현재 원본 체크포인트를 확보한 것으로 취급하지 않는다.
  공식 Projected-ShapeNet55 URL도 만료됐으나 공개 SGA-GSN의 AP-PS55
  inference-only 재배포본(130,024,020 bytes, SHA256 `66e22112…781b7`)을
  확보해 별도의 탐색 비교에 사용한다. 원본 저자 배포와 동일한 파일이거나
  논문 최고 성능 가중치임을 증명한 것은 아니다.
- PCN 16,384점, cropped ShapeNet55 8,192점, Projected55 8,192점의 config와
  입력 affine 규칙을 명시적으로 구분했다. 새 host 회귀 7개와 기존 관련
  회귀를 합쳐 **23 passed**다. `4664db2` push 및 양쪽 정식 update 후,
  고정 RGB-D 48 capture/20 object 입력·독립 evaluator·GPU2에서 비교했다.
- AP-PS55는 Projected55 config로 **32,470,033 parameter strict load**에
  missing/unexpected=0이며 8,192점을 출력한다. Center-only/.5/1.0 × 두 축과
  기존 PCN baseline의 7조건, **140 observation-variant**를 실행했다.
  사전 지정한 center-only world Z-up 후보는 관측 중심만 빼고 meter 크기를
  유지한다. 공개 SGA-GSN 사용 사례에 근거한 가설이지 학습 규약의 완전한
  재현을 증명한 것은 아니다.
- Seed 0의 물체 관측별 중앙값은 PCN 대비 표면 p95 **27.10 → 13.27 mm**,
  관측점→출력 p95 **11.84 → 4.62 mm**, F1@10mm **.432 → .865**,
  미관측 표면 recall **.065 → .695**다. 관측만의 F1은 **.595**다.
  같은 반경 .5에서도 AP-PS55는 F1 **.802**로 개선됐으므로 효과를 전처리
  제거에만 귀속하지 않는다. 반경 1.0은 두 축 모두 FAIL이었다.
- 사전 후보의 sampling seed 1·2, 추가 **40 observation**도 기존 중앙값
  수용 기준을 통과했다. 세 seed의 표면 p95 중앙값은 **12.71~13.27 mm**,
  F1 **.865~.866**, 미관측 recall **.693~.701**이다. 이것은 같은 기록을
  다시 샘플한 결과로 새 물체 검증은 아니다.
- Seed 0 개별 관측은 **20/20** F1 개선이지만 다섯 형상 기준을 모두 만족한
  것은 **12/20**이다. 최악 표면 p95 **24.17 mm**, 특정 box의 미관측 recall은
  약 .05로 실패가 남는다. 기존 8개 box의 중앙값 통과와 일반 형상·실기
  정확도를 구분한다. Hidden은 관측점에서 10 mm 넘게 떨어진 truth 표면의
  대용 지표이며 ray visibility로 분류한 것은 아니다.
- PCN baseline의 관측·출력 array **20/20**은 이전 기록과 완전히 같았다.
  독립 평가를 포함한 모든 stage가 정상 종료했고 원본 관측과 일반 test
  Sim/Pilot의 ID·이미지·stopped 상태는 유지했으며 진단 컨테이너는 제거했다.
  원자료 **299파일**을 내려받아 SHA256을 검증했다. 약 **13.6 ms** 단독
  forward는 acquisition·Sim/MPPI 동시 실행 지연이 아니다. Canonical gate는
  기존 dev 준비 미완료 사유로 실행하지 않았다. Production 의존성·설정·
  collision/grasp 경로는 바꾸지 않았다. 새 형상·크기·잡음의 독립 검증과
  runtime 통합은 남는다.
  [해석·재현](shape_completion.md#ap-ps55-checkpointframe-replay-2026-10-27),
  [수치·격리 증거](../workbench/evidence/curated/experiments/adapointr_profiles_20261027.json),
  [7조건 비교](../workbench/evidence/curated/experiments/adapointr_profiles_metrics_20261027.png),
  [관측·기준선·후보·정답](../workbench/evidence/curated/experiments/adapointr_profiles_clouds_20261027.png).

### AdaPoinTr 전처리 원인 분리 (2026-10-27, GPU 대조 실험 완료)

- `56a717b` push 및 원격 정식 updater 완료 후 GPU2 A6000에서 공개
  airplane/sofa의 native, 중심 이동만, 크기 변경만, 중심+반경 .5,
  중심+반경 1.0을 5개 sampling seed로 비교했다(**50 inference**).
  같은 seed의 모든 조건은 같은 물리 점 인덱스·순서를 사용하고 결과를 원본
  좌표로 역변환한다. 원래 PCN Y-up 축을 유지해 축 규약을 혼입시키지 않았다.
- 관측점→출력 거리 p95 / 원본 관측 AABB 대각선의 중앙값은 비행기 native
  **0.448% → 현재 기본 정규화 6.785%**이며 paired ratio는 **15.23배**다.
  소파는 **0.665% → 0.737%**로 기본 조합의 영향은 작았으나 중심 이동만
  하면 **5.020% / 7.51배**였다. 크기 변경만은 비행기에서 **19.93배**였다.
  큰 효과는 5/5 seed에서 나타났다. 변환 효과는 물체별로 달라 단순 합산이나
  정규화 제거를 검증된 해결책으로 취급하지 않는다.
- 공개 샘플에는 완전한 정답이 없으므로 관측 표면 보존과 native 출력과의
  차이를 평가했다. 우리 partial 중심·크기 전처리 자체가 관측 보존을 악화시킬
  수 있음은 확인됐으나 전체 형상 정확도나 기존 box 실패의 단일 원인은 아니다.
  이전 결과는 가중치+전처리의 결합 결과다. World meter를 그대로 넣는 것도
  원본 demo canonical frame과 다르므로 대체 입력 규약과 box 재검증이 남는다.
- 동일 점 인덱스·역변환·원본 좌표 단위 거리의 host TDD는 red 이후 새 7개,
  관련 시험 포함 **16 passed**다. GPU inference는 정상 종료했으며 기존 test
  Sim/Pilot ID·이미지·stopped 상태를 보존했고 진단 컨테이너는 제거했다.
  Runtime 설정과 production 전처리는 바꾸지 않았다. Canonical gate 미실행
  사유는 아래 도입 시험과 같다.
  [해석·재현](shape_completion.md#same-sample-preprocessing-control-2026-10-27),
  [수치·격리 증거](../workbench/evidence/curated/experiments/adapointr_preprocessing_20261027.json),
  [수치 비교](../workbench/evidence/curated/experiments/adapointr_preprocessing_metrics_20261027.png),
  [출력 비교](../workbench/evidence/curated/experiments/adapointr_preprocessing_clouds_20261027.png).

### AdaPoinTr 도입 시험 (2026-10-27, 실행 통과·형상 품질 미달)

- 기존 설치 설정·일반 세션·MPPI 제어값을 유지한 채 원격 GPU2 A6000의 설치
  소유 Compose 진단 작업에서 시험했다. 공식 PoinTr source `4603257`과 공식
  PCN checkpoint SHA256 `f58a5650…664fa1`을 고정했다. ShapeNet-55 공식 링크는
  HTML을 반환해 확보하지 못했으며 이번 결과는 PCN용 가중치에 한정된다.
- 원본 forward와 32,494,657개 parameter를 strict load했다(missing/unexpected=0).
  기존 Torch 2.12.1+cu130에 nvcc/PointNet++ 확장이 없어 FPS는 기존 Triton,
  gather는 Torch로 실행한다. FPS 8사례는 초기점·near-zero 제외·tie를 포함해
  CPU 참조와 인덱스가 같았고 gather도 일치했다. 실제 upstream CUDA 바이너리와
  직접 대조한 결과는 아니다. Loss/training은 비활성이며 namespace 격리를 전제로
  한 연구 worker다. Runtime Pilot package의 의존성·설정은 바꾸지 않았다.
- 공개 airplane/sofa 샘플의 출력은 모두 16,384점이고 반복 차이는 0이었다.
  배치1 FP32 단독 추론 중앙값은 **14.30 / 13.92 ms**, PyTorch peak allocated는
  **171 MiB**였다. 첫 forward의 kernel 준비는 **1.27초**였다. 이 수치는
  공유 GPU에서 Sim/MPPI와 동시 실행한 step/RTF나 전체 acquisition 지연이 아니다.
- 센서만 읽는 전처리와 독립 evaluator를 분리했다. 정답은 evaluator에만
  마운트한다. 24자세×2 seed의 **48 capture 중 10 capture에서 8개 box의 부분
  점군 20건**을 얻었다. 나머지도 capture 분모에 남긴다. 320×240 Genesis의
  정적 RGB-D replay, red connected-component segmentation 기준선이다.
- TDD에서 world Z-up과 공식 PCN Y-up 축 순서 차이를 수정했다. 같은 입력의
  축 대조군, 반경 .5와 1.0 정규화 모두 사전에 고정한 형상 기준에 미달했다.
  PCN 축·반경 .5에서 물체별 표면 p95의 중앙값은 **27.10 mm**, 관측점의
  대응 수치는 **1.06 mm**였다. Median F1@10mm는 **관측만 .595 → 모델 .432**,
  hidden-surface recall 중앙값은 **.065**였다. 원본 관측점은 별도 array에
  완전히 보존한다. ICP·정답 중심·정답 크기 정규화는 사용하지 않았다.
- 기록 점군의 배치1 단독 forward 중앙값은 약 **13.04 ms**로 초기 속도 기준을
  통과했지만 geometry acceptance는 FAIL이다. 관측을 대체하거나 Pick/Grasp의
  collision shape로 자동 적용하지 않는다. 정확도 원인을 가중치의 학습 분포나
  partial-frame 중심·크기 가정으로 확정한 실험은 아니며 후속 비교가 필요하다.
- 연구 TDD **9 passed**, 관련 재생 회귀를 포함해 host **13 passed**다. 초기
  EasyDict 누락과 축 규약 수정 전 결과도 원자료에 남겼다. Canonical dev 최초
  이미지 준비가 미완료인 기존 사유로 required/extended·real-RMW·isolated release
  gate는 실행하지 않았으며 host/실제 GPU 시험과 구분한다. 진단 컨테이너는
  모두 제거됐고 기존 test Sim/Pilot ID·이미지·stopped 상태는 보존했다.
  [실험 절차·한계](shape_completion.md),
  [수치·버전·격리 증거](../workbench/evidence/curated/experiments/adapointr_20261027.json),
  [관측/추정/정답](../workbench/evidence/curated/experiments/adapointr_objects_20261027.png),
  [수치 비교](../workbench/evidence/curated/experiments/adapointr_metrics_20261027.png).

### 불완전한 관절로 생긴 시각 기울기 보정 (2026-10-26, 소프트웨어·정적 Genesis 검증)

- 목표는 실제 바닥의 semantic identity나 Z=0 높이를 찾는 것이 아니라 팔의
  관절 오차 때문에 기울어진 관측 좌표계를 바로잡는 것이다. 실사용 보정은
  최소 tilt 회전만 적용하고 현재 FK camera origin을 그대로 유지한다.
  Camera mount는 고정하며 전체 관절·위치·yaw 식별을 주장하지 않는다.
- 서로 다른 높이의 수평면을 같은 방향 근거로 사용한다. 3도 이상 충돌하는
  지지 평면 방향은 거부하고, UI의 확인은 선택 면이 수평이라는 operator
  선언이다. 동기화·source epoch·freshness·active workflow 경계는 유지한다.
- 실제 q와 reported q를 분리한 24자세×6조건×2 seed 합성 RGB-D 회귀를 만들었다.
  Actual q에는 Linear ±5 mm, Roll ±5도, Bend별 ±1.5도의 오차와 pose-dependent
  deflection을 주고 camera mount는 바꾸지 않는다. Primary metric은 잔여
  tilt와 cross-view object-normal 일관성이다. CPU analytic 288경우·864 frame은
  통과했고 camera-origin 강제 이동은 전부 0이었다.
- `ff640b5`를 push하고 양쪽 정식 updater를 완료한 뒤 실제 GPU2 A6000의
  Genesis RGB-D 864 frame을 격리된 consumer로 처리했다. 관절 오차 조건의
  tilt p95는 **6.56 → 0.10도**, pose-dependent deflection은 **4.40 → 0.10도**,
  높이가 다른 수평면·복수 평행면·noise 조건도 약 **0.10도**였다. Camera
  origin 강제 이동은 0이며 3도 이상 잔여 tilt를 수용한 경우는 없었다.
- 조건마다 **32/48경우, 16/24자세**에서 기준면을 지원했다. 나머지는 보정을
  거부했고 평가 분모에 남겼다. Joint-offset 조건의 독립 물체 상면 normal
  p95는 **5.97 → 0.09도**였으나 cross-view는 조건당 4쌍에 불과하다.
  높은 판이 물체를 가린 조건에는 object-normal 정확도를 산출하지 않았다.
  Clean control은 보정 후 약 0.10도 오차를 얻었으며 원인은 단정하지 않는다.
- Consumer에는 RGB-D/reported q와 nominal arm·hand-eye만 제공했고 actual q,
  truth·generator·network·GPU 접근은 없었다. Normal test Sim/Pilot의 ID·image·
  stopped 상태는 유지했고 진단 컨테이너는 모두 제거했다. 실기 하중·운동·
  노출 동기화·전체 관절/위치/yaw 정확도는 미검증이다.
  [수치·격리·SHA 증거](../workbench/evidence/curated/experiments/visual_tilt_20261026.json),
  [전후 결과 그림](../workbench/evidence/curated/experiments/visual_tilt_20261026.png).
- 관련 host 회귀 58개와 새 focused batch 29개는 통과했다(중복 포함). 넓은
  Protocol/Pilot/UI host batch는 830 passed, 32 skipped, 8 subtests passed이며
  host Shapely/PyAV 부재로 관련 4개 파일을 제외했다. 역할 기본값을 함께 읽는
  Protocol linear 시험은 Sim/Robot source root를 추가해 확인했다. 기존 설치의
  dev 이미지 준비가 미완료여서 canonical을 재시도하지 않았으며 real RMW,
  isolated-release gate 통과로 해석하지 않는다.
- 이전 높이·장착오차 실험은 역사적 결과로 보존하되 현재 수용 조건으로 쓰지
  않는다. 재현은 명시적인 `--legacy-height` 옵션으로만 실행한다.

### 실제 팔 4DOF × 독립 오차 검증 (2026-10-26)

- Linear·Roll·두 Bend의 허용 범위에 Latin hypercube 자세를 뽑고 독립 오차
  seed와 교차하는 RGB-D 실험을 추가했다. visibility로 자세를 버리지 않으며
  미검출·지원 평면 부재·오보정도 평가 분모에 남긴다. 0.8 m 고정 bench base,
  static pose, 이상적인 동기 encoder 조건이며 물리 구동 검증은 아니다.
- `4ffe351`을 push하고 양쪽 정식 updater를 실행한 뒤 GPU2 A6000에서 **24자세
  × 8조건 × 오차 seed 2개 = 384경우, 1,152 frame**을 처리했다. Roll은
  **−88.10..84.83°**, 두 Bend는 **−34.52..35.94° / −34.67..34.03°**다.
  Pilot FK와 독립 URDF FK는 무작위 80자세에서 일치했고, 실제 Genesis node9
  transform도 각 샘플에서 대조했다. Consumer에는 RGB-D/q와 nominal arm·hand-eye
  파일만 제공했으며 truth/generator/network/GPU 접근은 없다.
- 물체가 관측되고 보정도 가능한 경우의 위치 p95는 높이·기울기 조건
  **113.29 → 2.10 mm**, 잡음·누락 조건 **100.53 → 4.09 mm**였다. 회전하는
  local SE(3) 장착 오차는 **115.39 → 56.87 mm**로 남았다. 단일 frame 평면
  제약만으로 모든 장착 오차를 복구할 수 없다.
- 정상 조건 **48경우 중 32경우**가 바닥을 지원했고 나머지 16경우는 보정을
  주지 않았다. 물체도 관측된 경우는 정상 10/48, tilt/height 12/48이므로
  위 정확도를 전체 자세에 보장한다고 해석하지 않는다. 낮은 판 조건에서는
  **32/48경우가 모두 잘못된 판을 수용**, 위치 p95가 **1.13 → 106.96 mm**로
  악화했다. Overall acceptance는 **FAIL**이다. configured height의 허용 오차는
  lone plane의 물리적 identity를 보장하지 않는다.
- 높은 판 조건의 유효 선택 1건은 판 아래에서 보인 실제 바닥이었다. 평가기는
  scene label만으로 이 경우를 오탐 처리하던 검사를 수정해 독립 정답 평면
  geometry로 판정한다. 낮은 판 오인식 32건과 전체 FAIL은 변하지 않는다.
- 진단 one-off는 모두 제거됐고 기존 test Sim/Pilot의 ID·이미지·실행 상태는
  보존했다. 관련 host 회귀 **59 passed**. Canonical required는 알려진 dev
  wrapper를 실행했으나 최초 개발 이미지의 CUDA 의존성 준비가 끝나지 않아
  **2,616.6초 뒤 이번에 시작한 build client만 SIGINT로 중단**했다(exit 130).
  테스트 본문은 미실행이며 canonical 통과로 해석하지 않는다. 다른 설치
  update와 정상 앱은 중단하지 않았다. 생성된 dev 외 별도 개발 이미지는 없다.
  [수치·SHA·격리 증거](../workbench/evidence/curated/experiments/vision_arm_matrix_20261026.json),
  [자세별 결과 그림](../workbench/evidence/curated/experiments/vision_arm_matrix_20261026.png).
- 후속 offline 탐색에서는 condition/truth 없이 consumer의 plane/q 기록만
  사용해 source boot별 고정 mount를 fit했다. **물체 점이 있는 자세는 전부
  피팅에서 제외**했다. 고정 local-mount 조건의 held-out 물체 관측 11건은
  기존 per-frame **56.87 mm → fixed-mount 2.66 mm p95**였다. 높이·기울기를
  world에서 바꾼 조건과 잡음 조건에는 같은 고정 mount 모델이 맞지 않았다.
  낮은 판 조건의 training plane-distance RMS는 **87.9 / 102.3 mm**로 컸다.
  이는 model consistency 진단의 단서이며 자동 reject/모델 선택을 구현한
  결과가 아니다. Candidate는 설치·적용하지 않았다. 추가 host 회귀 1개 통과.

### Sensor-only 부분 점군 기준선 (2026-10-26)

- Pilot에 RGB-D·intrinsics·encoder만 허용하는 schema-1 로컬 replay packet과
  visible surface deprojection, bounded voxel fusion을 추가했다. object/camera
  truth와 asset/mesh 필드를 거부하며 ICP로 pose 오차를 지우지 않는다.
- 별도 producer/consumer/evaluator 실험은 무작위 물체와 숨긴 camera mount
  오차를 사용하고, floor fitting에 쓰지 않은 물체 점의 world 오차를 평가한다.
  XY/yaw negative control을 포함한다. CPU analytic 3-seed 회귀와 실제 Genesis
  1.4.1 GPU2의 3개 상자·48 RGB-D frame 시험을 통과했다. 안정화 뒤 36 frame의
  물체 점 world error p95는 tilt/height 조건 **86.46 → 0.99 mm**, object surface
  distance p95 **0.97 mm**였다. XY/yaw 조건은 **70.60 → 70.59 mm**로 남았다.
  별도 컨테이너의 실제 mount를 검사했으며 consumer는 truth·scene generator를
  받지 않고 network도 없다. 당시 test Sim/Pilot은 같은 ID·이미지·실행 상태를
  유지했고 진단 컨테이너는 남지 않았다. 양쪽 정식 updater로 `cc030b1`을
  게시한 뒤 그 새 이미지의 runtime module SHA를 확인했다.
- 알려진 Z=0 바닥, 이상적인 가상 Cartesian encoder, 인공 depth noise sigma
  0.5 mm 조건이다. 단순 HSV segmentation과 보이는 부분의 4 mm voxel fusion
  기준선이며 PCN·실제 팔 FK·카메라 정확도·grasp의 완료 증거로 해석하지 않는다.
  절차는 `docs/visual_calibration.md`,
  [수치·격리·SHA 증거](../workbench/evidence/curated/experiments/vision_partial_cloud_20261026.json)를 따른다.
- Genesis vertical FOV와 raster pixel centre에 맞게 Sim RGB-D intrinsics를
  수정했다. width로 focal length를 만들던 부분을 height로 바꾸고 integer
  pixel 좌표의 principal point를 `(width-1)/2, (height-1)/2`로 맞췄다.
- 새 sensor/geometry/analytic 실험 시험 **13 passed**, 기존 floor/hand-eye 및
  host 가능 Sim camera 회귀 **59 passed, 2 skipped**, Pilot 경계 **3 passed**.
  Host의 Genesis 부재로
  `test_async_camera.py`는 수집할 수 없었다. canonical gate는 미실행이다.

### 뎁스 평면 기반 시각 캘리브레이션 소프트웨어 (2026-10-26)

- Pilot에 metric depth deprojection, bounded multi-plane RANSAC/weighted fit,
  지지 면적·두 축 폭·잔차 검사, 고정 기준면 선택과 추적을 구현했다. 큰
  책상도 자동으로 바닥으로 승격하지 않으며 초기 관측 기준은 상대 기준임을
  표시한다. 보정은 camera tilt 2축과 plane-normal distance만 다룬다.
- 기존 encoded RGB-D의 source/boot/calibration ID와 capture time을 adapter
  뒤에도 보존한다. Robot의 additive `measurement_time_s`로 source wall-clock
  관절 측정 시각을 제공하고 Pilot에서 bracketing pose를 보간한다. 실제
  촬영/측정 timing 검증은 별도이며 telemetry 기반 보정은 기본 차단한다.
- 기존 perception 입력을 받는 6 Hz latest-only worker, 선택적 world-coordinate
  보정, 원본과 분리된 preview plane outline, UI Visual Calibration의 기준면
  선택/reset/기록 export를 연결했다. 기본값은 관측 ON, 좌표 보정 OFF다.
  Active IK/Pick/Gaze 중 기준·설정 변경은 거부한다. Observer 자체는 motor
  명령을 보내지 않으나 좌표 보정 적용 뒤 기존 workflow의 계획 결과는 바뀔
  수 있다. 구체적 절차와 한계는 `docs/visual_calibration.md`를 따른다.
- 최대 2,000개의 plane·예측 pose·raw q/body/IMU model 진단 기록을 export하고,
  별도 offline fixed-mount fitter는 rank 부족과 source/reference 혼합을 거부한다.
  매 다섯 번째 관측은 fitting에서 제외해 plane consistency를 평가한다.
  결과는 별도 candidate 파일이며 설치된 hand-eye 값을 자동 변경하지 않는다.
- 합성 1280×720 입력, 4,096점, 두 평면의 host CPU fitting 20회는 median
  **11.02 ms**, p95 **13.63 ms**였다. 초기 fixed-128-trial RANSAC은 median
  35.40 ms였고 confidence 기반 조기 종료를 넣었다. 물리 카메라 정확도,
  live latency나 Sim step 성능의 증거는 아니다. 원자료는
  `workbench/evidence/generated/floor-calibration-20261026/`에 있다.
- Host에서 Protocol/Pilot/UI wheel을 build하고 새 모듈 6개 포함 및 source
  checkout 밖 zip-wheel import를 확인했다. 최종 전체 host 회귀는 **969 passed,
  32 skipped, 18 subtests passed**다. Host의 Shapely/PyAV 부재로 관련 4개
  파일은 제외했다. Canonical gate 통과로 해석하지 않는다.
- 알려진 installed `elesim-dev` wrapper는 존재하지만 pinned Docker socket에
  접근이 거부됐다. Canonical required/extended, release build/verify 및 real-RMW
  `smoke_topology.py`는 미실행이며 host에도 `rclpy`가 없다. 실기 ZED/D435
  timing·기준면 정답·독립 높이/자세 오차와 IMU/plane/combined 비교는 남아 있다.

### 물리 적분까지 전체 50 Hz로 맞춘 실제 실행 (2026-10-23)

- 사용자 요청에 따라 `7e84048` 정식 설치 이미지의 진단 실행에서 dt **.02**,
  substeps **1**로 물리 내부 적분도 **50 Hz**로 바꿨다. 팔 명령·다리 토크
  갱신·MPPI 모두 50 Hz이며 실제 MPPI stride는 **1**이다. GPU2·4096×2,
  기존 팔·DDS 모터 경로·RGB-D와 두 1280×720 WebRTC를 유지했다.
  토크 필터 alpha는 100→50 Hz에서 같은 시간상수를 유지하도록 **.5→.75**로
  바꿨고, 이미 MPPI 50 Hz에서 적용되던 force alpha **.7**은 유지했다.
- 정지 측정 구간은 clock RTF **1.000**이었다. Walk 첫 1초를 제외한 **153 tick**의
  전체 loop 평균은 **23.07 ms**, p95 **33.13 ms**, 실제 clock RTF **.860**이다.
  전체 정상 walk 203 tick의 평균은 **23.57 ms / RTF .842**였고 **72.4%**가
  20 ms 예산을 넘었다. 기존 100 Hz는 MPPI가 두 tick마다 실행되지만 이
  구성은 매 tick 실행되므로 한 step 비용이 같다고 가정하지 않는다.
- 전진 명령 후 **4.90 sim초**, 실제 walk 진입 후 **4.06 sim초**에 네 발의
  `touchdown timeout after 0.30s: FL,FR,RL,RR`로 실패했다. 마지막 UI pitch는
  약 **−.391 rad**였고 요청한 20초 전진을 완료하지 못했다. 현재 revision의
  100 Hz 대조도 fault가 있었으므로, 이 단일 실행으로 주기 하향만을 실패
  원인으로 확정하지 않는다. 실시간 성능과 정상 보행의 동시 통과는 미달이다.
- 종료 후 기존 **100 Hz** Sim/Pilot의 정확한 컨테이너·이미지·GPU2 실행 상태를
  복원했다. 운영 설정·소스 기본값은 변경하지 않았다.
  [조건·결과·원시 파일 SHA](../workbench/evidence/curated/sim/all50_20261023.json).

### CPU 물리 + GPU MPPI와 전체 GPU 배치의 첫 비교 (2026-10-23)

- 양쪽 정식 updater와 기존 키·정책을 보존하는 등록 API로 `7e84048`을 적용했다.
  원격 test의 실제 Sim 이미지는 `cfc4bc915652`, Pilot은 `ac1f3b62f32b`이며
  두 역할의 GPU reservation은 `['2']`다. 같은 이미지·설정에서 parent Genesis
  초기화만 GPU/CPU로 바꿨다. 전역 `use_gpu=True`, MPPI 4096×2·명목 50 Hz,
  dt .01/substeps 2, 팔 쓰기, SROS2 DDS, RGB-D와 두 1280×720 WebRTC를 유지했다.
  CPU 물리 실행에서도 MPPI `cuda:0`, NVIDIA GL renderer와 NVENC를 확인했다.
- 두 실행의 설정·제어 코드 hash는 같았지만 실제 보행 궤적은 달라졌다.
  walk 진입 후 첫 100 tick을 제외한 다음 **200 tick**의 전체 루프 평균은
  GPU 물리 **18.87 ms**, CPU 물리 **17.35 ms**였다. 주 스레드 CPU 시간은
  **13.47 / 11.20 ms**, 나머지 경과 시간은 **5.39 / 6.14 ms**다.
  나머지 시간에는 scheduling·lock·device 대기가 섞이므로 순수 GPU 계산
  시간으로 해석하지 않는다. 단일 실행의 약 8% 차이를 반복 검증한 개선으로
  확정하지 않는다.
- GPU는 walk 시작 약 **3.32 sim초** 뒤 finite force trajectory fault,
  CPU는 약 **6.52 sim초** 뒤 touchdown timeout으로 실패했다. CPU 실행은
  전진도 거의 이루지 못했다. 둘 다 요청한 20초 전진을 완료하지 못했으므로
  CPU 배치 채택이나 정상 보행의 속도 우위를 입증한 결과가 아니다.
  두 실행 모두 종료 후 원래 Sim/Pilot 실행 상태를 복원했다.
- GPU 재측정과 CUDA/CPU 세부 추적은 자동 승인 검토의 workspace 지출 한도로
  진행하지 못했다. 운영 물리 backend는 GPU를 유지한다. 진단 코드는 generated
  evidence에만 있으며 production 로직 변경은 없다.
  [조건·결과·원시 파일 SHA](../workbench/evidence/curated/sim/backend_placement_20261023.json).

### 명목 네 발 지지 시간과 실제 착지 대기의 중복 (2026-10-23)

- `900f841`을 양쪽 정식 update/등록 후 전체 앱·원격 GPU2로 검증했다. 일반 착지
  대기의 회전 목표는 의도대로 동작했지만 약 **5.8 sim초** 보행 후 pitch 증가와
  finite trajectory fault로 실패했다. FR/RL 지지 calf는 fault 전에 물리 관절
  상한에 도달했다. 자세 회복 대기는 이미 pitch .293 rad일 때 시작했다.
  정상 Sim/Pilot을 복원했으며 600초 gate는 통과하지 않았다.
- GPU gait가 명목 네 발 지지 구간 시작에서 착지 확인·load ramp를 끝까지 기다린
  다음 남은 명목 dwell까지 추가로 소비했다. 처음 10번 전환에서는 남을 두 발이
  기존 접촉·하중·현재 힘·자세 검사를 통과한 시점이 실제 liftoff보다 **30–50 ms**
  빨랐다. 마지막 악화된 전환의 차이는 20 ms여서 이 수정만으로 회복된다고
  단정하지 않는다. 긴 실제 stance가 반복되어 reach를 소진하는 원인 후보이다.
- 새 후보는 기존에 완전히 loaded인 diagonal이 있을 때만 명목 네 발 지지 구간을
  진행시켜 실제 착지 확인과 겹친다. 다음 liftoff 경계에서는 기존의 모든 현재
  접촉·하중·힘·자세 검사를 다시 요구한다. blocked의 실제 support horizon,
  진행 중 swing, 확인/하중 시간, timeout, 힘/토크 제한 및 CPU 정책은 유지한다.
  새 회귀 **3 failed / 4 passed**로 중복 대기를 재현했다. 리뷰가 발견한 아직 진행 중인 swing의 조기 support 약속도 **1 failed /
  7 passed → 수정**했다. 전체 Sim **1068 passed / 5 skipped**이며, 실제 GPU
  효과는 검증 중이다.

### 착지 대기의 회전 제동과 디딤점 예측 불일치 (2026-10-23)

- `51292d6`을 양쪽 정식 update/등록 후 원격 GPU2에서 검증했다. 두 WebRTC·
  RGB-D DDS·팔 유지·4096×2·dt .01/substeps 2를 유지했지만 약 **5.4 sim초**
  보행 뒤 finite trajectory fault로 실패했다. 451개 완료 torque tick에서 자세
  회복 대기는 한 번도 작동하지 않았다. 종료 후 정상 Sim/Pilot을 복원했다.
- 100 Hz 실제 기록의 독립 분석에서 일반 착지 대기가 cruise tick의 **30.75%**였다.
  이때 전진 진행은 보존하지만 회전을 제동했다. 개별 대기에서 Euler yaw 속도가
  **.818→.131**, **.700→−.013 rad/s**로 줄었다. 마지막 FL 디딤 예측의 실제
  stance 끝 yaw 오차는 **−.190 rad**, 횡방향 위치 오차는 **−115 mm**였고,
  이어 지지 FL calf가 실제 관절 상한을 넘었다. 상관관계이며 단독 인과 증명은 아니다.
  Torque bias와 zero-GRF는 feasible했고 non-foot 접촉 증거는 없었다.
- 일반 GPU 착지 대기에서만 이미 달성한 같은 방향의 정확한 Euler yaw 속도를
  유효 shaped 명령 크기까지 유지하는 후보를 회귀 **8 failed / 8 passed**로 재현했다.
  회전 reference는 world-Z에서 body frame으로 변환하고 두 디딤 예측에도 같은
  yaw 속도를 준다. 대기 중 yaw 각도는 현재값으로 유지해 seed의 추가 가속 요청을
  만들지 않는다. 키 해제·반대 방향·자세 회복·CPU 경로의 제동은 유지한다.
  수정 후 GO2 **538 passed / 1 skipped**, 전체 Sim **1057 passed / 5 skipped**다.
  독립 코드 검토를 완료했다. 실제 GPU 실패와 다음 후보는 위 항목에 기록했다.
  600초 gate와 운영 보행의 실시간 속도는 여전히 미해결이다.

### 큰 swing 전이와 몸통 회복 대기 후보 (2026-10-22)

- `9106d60`의 전체 앱 전진+회전은 약 **12.7 sim초** 후 finite force trajectory
  fault로 실패했다. 정확한 liftoff 이벤트 **34개 모두** 남은 두 발의 현재 힘이
  5 N보다 컸다. 현재 하중 검사만으로 안정성을 해결하지 못했다. 보행 성능
  12개 창 중앙값은 **17.73 ms / RTF .564**이며, 단일 실행 차이를 속도 개선으로
  확정하지 않는다. 종료 후 정상 Sim/Pilot을 복원했다.
- 6.1 Sol 독립 검토에서 마지막 큰 기울기 이전의 FR 목표 전이가 약 **.625 m /
  .182 s**임을 확인했다. 현재 cubic 수평 swing의 목표 최고속도는 약 **5.16 m/s**,
  시작 가속도는 약 **113 m/s²**다. 이어 실제 지지력이 사라지고 roll 속도가
  커졌다. Swing 반작용·추종 포화·지지발 미끄러짐을 분리할 필요가 있으며,
  목표 수치를 실제 발 속도/가속도로 해석하지 않는다.
- 다음 GPU 후보는 새 liftoff 직전 roll/pitch **.25 rad** 또는 정확한 Euler
  roll/pitch rate **1 rad/s** 초과 시 현재 지지로 회복을 기다린다. 이미 시작한
  swing은 계속한다. 자세 대기 중에는 기존 착지 대기의 속도 유지와 달리 제동을
  요청하며, stance damping 상쇄도 하지 않는다. 실제 접촉·힘/토크 제한과
  CPU gait 정책은 보존한다. 새 정책 **15 failed → 수정**, 리뷰의 제동 회귀는
  fixture 수정 뒤 **3 failed / 15 passed → 수정**, GO2 **522 passed /
  1 skipped**, 전체 Sim **1041 passed / 5 skipped**다. 실제 장시간 GPU 효과는
  검증 전이다.
- 계측은 현재 tick의 토크 생성 뒤 힘 명령·최종/포화 전 토크, 관절 속도,
  swing 시작/종료점과 centroidal 내부 각운동량을 함께 기록하도록 보완했다.
  새 계측의 GPU 실행 및 원인 판정은 아직 완료하지 않았다.

### 지지 전환의 현재 하중 검사와 팔 상태 배열 중복 제거 (2026-10-22)

- 복원한 `c7e7f90`의 전체 앱 기준 보행은 약 **52 sim초** 후 finite force
  trajectory fault로 실패했다. 두 WebRTC·RGB-D DDS·팔 유지·4096×2·GPU2를
  유지한 보행 49개 성능 창의 loop 중앙값은 **18.47 ms / RTF .541**였다.
- 매-step 팔 쓰기에서 constraint acceleration warmstart만 유지한 대조는
  약 **15 sim초** 후 FL 착지 timeout으로 실패했다. Equality geometry와
  collision cache 초기화는 유지했으며 loop 중앙값은 **18.13 ms / RTF .551**
  (14개 창)이었다. 안정성·속도 이득을 입증하지 못해 채택하지 않는다.
  두 진단은 정상 Sim/Pilot을 완전히 중단한 단독 장면이며 종료 후 복원했다.
- 접촉 이력과 load ramp가 남아 있으면 현재 바닥 힘이 없는 발을 남긴 채 반대
  diagonal을 들 수 있음을 회귀 **6 failed / 17 passed**로 재현했다. 새 liftoff를
  넘기 직전에 남을 두 발의 현재 normal force도 각각 5 N보다 큰지 확인한다.
  stance 중 짧은 impulse 누락 허용, 기존 swing 진행, 접촉 확인·load ramp·
  탐색 timeout·힘/토크 제한은 유지한다. GO2 focused **504 passed / 1 skipped**,
  전체 Sim **1023 passed / 5 skipped**다. `9106d60`을 push하고 양쪽 정식
  update/등록을 완료했다. 기존 역할 키·정책의 SHA가 같고, 원격 실제 두 역할의
  GPU reservation은 `['2']`다. 후속 실제 시험의 실패는 위 항목에 기록했다.
- 팔 payload의 네 batch 배열을 link별 row로 나눴다가 다시 stack하던 작업을
  제거했다. 질량·중심·속도·관성은 정확히 같고 노드별 물리량도 유지한다.
  새 회귀 **2 failed → 통과**, focused **25 passed**, 전체 Sim **1016 passed /
  5 skipped**다. 로컬 CPU microbenchmark 중앙값은 **.313→.218 ms**이며,
  이를 전체 원격 step 절감으로 대신하지 않는다.
  [조건·결과·원시 파일 SHA](../workbench/evidence/curated/sim/support_handover_20261022.json).

### 실측 보행 주기 예측의 실제 실패와 되돌림 (2026-10-22)

- `14519db`를 양쪽 정식 updater로 설치·등록했고 실제 이미지 및 GPU2 reservation을
  확인했다. 초기 reset 명령이 시계에 반영되지 않아, 아직 한 번도 걷지 않은 새
  중립 장면에서 reset 없이 시작했다. 두 WebRTC·RGB-D DDS·팔 유지·4096×2·
  dt .01/substeps 2를 모두 유지했지만 전진+회전 약 **2.8 sim초** 후 finite force
  trajectory fault로 실패했다. 600초 gate는 통과하지 못했다.
- 최근 gait cycle로 stance 예측을 늘린 변경은 실제 개선이 없어 되돌린다.
  두 실패 실험(preload 제한, 실측 stance 외삽)을 모두 제거하고 `eec3e2c`의
  기준 보행 로직을 복원한다. RGB-D buffer와 재시작 boot 중복 복구는 유지한다.
  복원한 접촉·디딤점·토크 focused **144 passed**다.
- 별도 실제 DDS 프로세스에서 기존/신규 Pilot boot의 중복을 만들었다. Sim이 양쪽
  exact peer를 unavailable로 거부하고 살아남으며, 이전 boot 만료 후 새 Pilot이
  lease를 다시 얻는 smoke를 통과했다. 보행 회복의 증거로 대신하지 않는다.

### Pilot 재시작 중 boot 중복이 Sim 통신 스레드를 종료시키는 오류 (2026-10-22)

- `ac78062`를 양쪽 정식 updater로 설치·등록했고 실제 원격 Sim/Pilot 이미지와
  GPU2 reservation을 확인했다. Cold scene build는 약 110초, 카메라 build는
  약 23초였다. 이 준비 대기 중 UI operator snapshot 응답이 끊겨 보행은 시작하지
  못했다. 이 구간을 새 보행 성능이나 안정성 결과로 인정하지 않는다.
- Pilot만 재시작하자 기존/신규 boot ID가 heartbeat timeout 동안 중복되었고,
  `DdsPeerNode.describe()`의 `PeerAmbiguityError`가 Sim endpoint `_run`과
  close의 lease-revoke 알림까지 전파해 DDS thread를 종료했다. Container는
  running으로 남았다. 중복 boot에서 어느 쪽도 선택하지 않는 정책은 유지한다.
- Exact 조회는 중복 상태를 unavailable로, publish는 기존 retry 대상인
  `DdsTransportError`로 처리한다. 새 wire field·QoS·authority 완화 없이 protocol
  major 6을 유지하며 `dds_contracts.md`에 기록했다. 회귀 **2 failed → 수정**,
  전체 protocol+Sim endpoint **183 passed**다. 원래 boot가 stale해진 뒤
  남은 exact boot만 다시 조회할 수 있는 것까지 검증했다. 실제 네 독립 프로세스의
  DDS topology smoke도 통과했다.
- 원래 보행 주기 예측 수정의 GPU 보행 효과와 600초 연속 보행 gate는 미해결이다.

### 실제 운영 보행의 착지 탐색 누적과 RTF 재검증 (2026-10-22)

- `eec3e2c`를 양쪽 정식 updater로 설치·등록했다. 원격 Sim/Pilot의 실제
  GPU reservation은 모두 `['2']`였다. RGB-D DDS와 두 1280×720 WebRTC 영상을
  켠 정상 설치 앱에서 정지 15 sim초는 **16.26 wall초**(구간 비율 **.923**),
  전진+좌회전은 약 **25.85 sim초** 후 finite force trajectory fault로 실패했다.
  보행 구간 비율은 약 **.486**이며 600초 gate는 통과하지 못했다.
- 정상 역할을 완전히 중단한 동일 설치 이미지의 단독 계측에서 안정 정지 10개 창의
  loop/RTF 중앙값은 **9.99 ms / 1.001**, 보행 31개 창은 **18.24 ms / .548**였다.
  보행 physics 구간(제어기 포함)은 **13.30 ms**, 별도 solve 호출은 **6.91 ms**다.
  solve 호출 평균을 매 tick 비용으로 더하지 않는다. 이 계측 보행도 약 34초 후 실패했다.
- 실패 전 12초 접촉 이력에서 실제 gait cycle 중앙값은 **.67–.68초**였다
  (설치 설정 2.2 Hz의 명목 .455초). 착지 확인 중에도 실제 바닥 힘을 받는 발의 탐색 목표가 실측 발보다
  최대 **20.2 mm** 낮아졌다. 명목 주기와 지지 전환 지연의 차이도 추가 조사 대상이다.
- 실제 바닥 접촉이 시작되면 확인 전 탐색의 계속 하강을 멈추고 기존 swing의
  8 mm preload 이하로 제한하는 회귀를 추가했다(**1 failed / 3 passed → 수정**).
  공중 힘, 5 N 이하 힘에는 적용하지 않으며, 40 ms 접촉 확인·60 ms load ramp·
  미확인 발의 GRF=0·힘/토크 제한은 유지했다. focused **127 passed**, 전체 Sim
  **1018 passed / 5 skipped**였지만 `e4ce7ce`의 실제 운영 보행은 약 **4.2초**에
  네 발 착지 timeout으로 실패했다. 이 preload 제한은 채택하지 않고 되돌린다.
- 다음 회귀는 접촉 확인/load ramp가 늘리는 실제 liftoff 간격을 stance 예측에
  반영한다(**2 failed → 수정**, focused **125 passed**). 명목 주기로 시작하고
  최근 8개 실측 cycle의 중앙값에서 기존 swing 시간을 뺀다. 정지 중 이력을 비우며
  실제 접촉 gate·swing 시간·기존 reach margin과 CPU acados 분기는 유지한다.
  전체 Sim **1018 passed / 5 skipped**다. 설치 GPU 보행 효과는 아직 검증 전이다.

### RGB-D DDS 메시지의 바이트별 Python 검증 병목 (2026-10-22)

- `bacb1a7`을 양쪽 정식 updater로 설치·등록하고 GPU2 선택을 시작 전후에 확인했다.
  정상 Sim/Pilot을 완전히 중단한 뒤 같은 설치 이미지·설정·SROS2 역할의 단독 진단으로
  두 1280×720 WebRTC 스트림, 팔 유지, dt .01/substeps 2, MPPI 4096×2를 실행했다.
- RGB-D DDS 발행만 10초씩 교대로 멈춘 **비용 분리 시험**에서 초기 다섯 창을 제외한
  보행 시작 전 loop 중앙값은 발행 켬 **17.28 ms / RTF .578**(7개 창), 끔
  **9.04 ms / RTF 1.106**(10개 창)이었다. 보행 중에는 각각 **33.86 / 14.64 ms**
  (각 3개 창)였다. 발행을 끈 진단을 운영 성능이나 보행 수용시험으로 인정하지 않는다.
- 60개 RGB-D 발행의 cProfile에서 생성 ROSIDL `color_data`/`depth_data` setter의
  누적 시간은 약 **1.303 / .479초**였다. 원래 `list(payload)`와 생성 setter의 바이트별
  타입·범위 검사가 수백만 Python 호출을 만들었다. 이 계측 시간에는 profiler 비용이
  포함된다. WebRTC 인코더 하나의 비용으로 해석하지 않는다.
- Encoded RGB-D와 raw Image의 payload는 소유권이 분리된 `array('B')`로 전달해
  생성 ROSIDL의 native buffer 경로를 사용한다. RGB·uint16/float32 깊이·깊이 생략의
  실제 ROS 직렬화/역직렬화에서 모든 필드와 payload 바이트가 동일하다. CDR 여백은
  비결정적이므로 큰 메시지의 전체 직렬화 byte hash가 항상 같다고 주장하지 않는다.
  회귀 **5 failed → 32 passed**, 설치 fixture를 포함한 후속 focused **93 passed**다.
- Required 검사에서 protocol **165**, Pilot **492 / 21 skipped**, Sim **1014 /
  5 skipped**, UI **114**, model/release tests **90**, DDS RGB-D **2**, 두 WebRTC
  **2**, 실제 4-process topology 및 Sim 모터·보행·gripper·stop smoke가 통과했다.
  초기 전체 실행에서 과거 Call sign/삭제된 up wrapper/domain 격리를 기대하지 않던
  설치 fixture 15개와, 자동 heartbeat를 켠 채 누락을 가정한 Robot 테스트 1개가
  실패했다. 현재 사양에 맞춰 fixture만 고쳤고 IPC **22 passed**다. Robot 실행 코드는
  바꾸지 않았다. 최종 Robot **161**, protocol **165**, setup **1272 / 3 skipped**
  재검증이 통과했다. 수정 후 재실행을 포함해 required의 모든 항목이 통과했다
  (**3312 passed / 29 skipped**, topology와 Sim motor script smoke 별도).
- 작은 GPU 커널 쓰기와 1 ms Python switch interval 대조는 전체 개선을 입증하지
  못해 적용하지 않았다. 지연된 미래 착지 추가와 상태 feedback force 후보만으로도
  저장한 실패 입력의 finite 해는 복구되지 않아 보행 로직에 적용하지 않았다.
  최신 실제 UI 계측 보행은 약 **49초 후 finite trajectory fault**였고 600초 통과가 아니다.
  바이트 버퍼 수정본을 양쪽에 갱신한 실제 RTF·보행 검증은 아직 진행 중이다.

### GO2 4096×2 장시간 실패 재현과 계측 보완 (2026-10-22)

- 정상 UI→Pilot→Sim 및 두 WebRTC 수신을 포함한 `dc5b423` 시험에서 정지 15초의
  RTF 중앙값은 약 **.517**이었다. 전진+회전 약 **4.2초 이후 FL 착지 timeout**으로
  실패했다. 단독 Genesis 진단의 104.3초 결과를 정상 앱의 안정성으로 대신하지 않는다.
- 새 stereo preview 토픽을 포함한 managed SROS2 rotate를 정식 경로로 완료했다.
  이 과정에서 rotate가 기존 역할별 GPU 선택을 지우는 오류를 발견했다. GPU 제한을
  확인하기 전에 재시작한 것은 작업 절차 오류였다. 보행 명령 전에 두 역할을 즉시
  중단하고 정상 instance replace로 GPU2 선택을 복구했다. 다음 시작 **전** Compose와
  시작 후 실제 두 컨테이너의 `DeviceRequests=['2']`를 확인했다. Rotate에서도
  compute·role_compute·viewer를 보존하는 회귀를 추가했다(**3 failed → 수정**).
- 디딤점 예측은 새 swing 시작과 실제 force solve가 있는 tick에서만 수행한다.
  생략한 계산을 강제로 수행한 대조와 토크 생성에 쓰는 sample 입력이 정확히 같다.
  접촉 변화에 따른 즉시 solve는 유지한다. 새 focused **125 passed**, 전체 Sim
  **1014 passed / 5 skipped**. GPU 설치본의 새 성능 측정은 아직 진행 중이다.
- `09c88d8`를 push하고 양쪽 `elesim-update`를 완료했다. 기존 SROS2 역할 키와
  정책 파일의 SHA가 모두 같은 상태로 새 release binding을 정식 staging 및
  instance replace transaction으로 등록했다. 원격 Sim/Pilot의 GPU 2번을 유지했다.
- 컨테이너 내부에서 PID 1에 보낸 SIGSTOP은 적용되지 않았다. 기존 앱과 진단을
  함께 돌린 초기 측정의 성능 수치는 폐기한다. 후속 진단은 Docker의 STOP 신호와
  `/proc/1/status`의 정지를 확인한 뒤 실행했고, 종료 후 원래 앱을 재개했다.
- 측정 하네스에 빠졌던 매-step 팔 자세 유지를 추가하고 command/arm/physics와
  camera dispatch를 모두 step 시간에 포함했다. DDS와 WebRTC 전송은 포함하지
  않는다. 두 1280×720 카메라, 실제 target collision, dt .01/substeps 2,
  MPPI 4096×2 조건의 전진+회전은 **20.1초에 fault**, 직진은 **67.8초에 fault**로
  실패했다. 제자리 회전은 **60초 보행 및 정지 복귀**를 통과했다. 600초 통과가 아니다.
- 보행 step 중앙값/p95는 전진+회전 **13.28/16.67 ms**, 직진 **13.69/16.64 ms**,
  제자리 회전 **13.86/16.45 ms**였다. 전진+회전에서 카메라를 끈 값은
  **11.32/13.05 ms**다. 전체 실제 앱의 RTF 회복으로 해석하지 않는다.
- 실제 실패 입력의 4096×2 CPU 재생에서 두 iteration 모두 finite 후보가 **0개**였다.
  유효 후보를 가중 평균에서 잃는 일반적인 optimizer 오류도 고쳤지만, 그 수정만으로
  이 실패를 해결하지 못한다. 원래 tilt·motor 제약과 최종 finite 검사를 유지한다.
- 동일 몸통 속도에서 세계 축 방향만 바꿔도 착지 시점 몸통 예측이 **55.9 mm** 달라지는
  좌표계 오류와, 지지 대기 중 유효 yaw=0을 예측에 반영하지 않는 오류를 재현하고
  수정했다. 새 회귀 **3 failed → 통과**. 팔 쓰기는 연속 인덱스와 CPU tensor 경로로
  정리하며 매-step 팔·claw 쓰기와 최종 FK, 몸통·다리 속도를 보존한다. GPU 동등성 및
  새 설치본 보행·속도 재검증은 진행 중이다.
- 오류 이유를 Sim→DDS→Pilot snapshot→UI로 전하고 explicit reset 시 비운다.
  protocol-v6의 bounded additive telemetry 필드다. 4개 독립 프로세스의 실제 DDS
  topology smoke에서 오류/해제 전달을 통과했다. 통합 software 검사 **1315 passed /
  5 skipped**, 후속 팔·solver focused **90 passed**, 최종 전체 Sim **1013 passed /
  5 skipped**다. [단독 실행 조건과 원시 artifact SHA](../workbench/evidence/curated/sim/gait_rtf_20261022.json).
- `20ad73e`를 양쪽 update/등록한 GPU2 검증에서 8개 팔 자세, 46 links/32 DOFs의
  위치·회전·선속도·각속도와 관절 상태는 기존 방식과 **오차 0**이었다. 팔 밖의
  속도도 보존했다. 동기화한 팔 쓰기 평균 **1.560→1.244 ms**. 팔 유지·양 카메라를
  포함한 보행 step 중앙값/p95 **12.68/16.45 ms**, 전진+회전은 **104.3초에 finite
  trajectory fault**였다. 이전 20.1초보다 오래 갔지만 600초 안정성/1.0 RTF 통과가 아니다.
- 실제 UI→DDS→Pilot 운영 검증에서 기존 SROS2 정책에 새 stereo left preview 토픽
  권한이 없음을 확인했다. Publisher의 RCLError가 `pilot-dds` 스레드를 종료시켰고
  container는 running으로 남았다. Media 초기화 callback의 middleware/OS 예외를
  보고하고 heartbeat·telemetry·motion을 유지하도록 회귀 수정했다(**2 failed →
  Pilot connection 21 passed**). 현재 정책의 정식 보안 갱신과 운영 재검증은 진행 중이다.

### GO2 지지발 손실 복구·회전 디딤점 예측 회귀 (2026-10-22)

- 기존 `1203cb4` 기반 실제 UI 4096×2 시험은 전진+좌회전 **54.0초의
  시뮬레이션 보행** 뒤 착지 타임아웃과 낙상으로 실패했다. 실시간 약
  178.7초였으며 10분 안정성 통과가 아니다. 낙상 전후 명령 수신은 이어졌다.
  당시 로그만으로 최초 불안정 원인이나 인터넷 단절의 기여를 확정하지 않는다.
- 기존 지지발이 공중으로 튀었을 때 공중 높이에서 4 cm 아래만 탐색하던
  복구 목표를 평지 높이에 맞췄다. 이미 낮은 발을 위로 당기지 않으며,
  기존 탐색 힘·토크·하강 깊이 제한은 유지한다. 진단 로그에 복구 분기와
  목표 높이를 추가했다.
- 몸통 기준 속도의 회전 궤적과 가감속을 한 번에 예측하고, 착지·지지 종료
  도달성 검사가 각 미래 몸통 방향도 사용하도록 수정했다. 일정 속도는
  원호 수식으로 계산한다. 직선·후진·좌우 기본 명령과 4096×2는 유지한다.
- 새 회귀의 수정 전 결과는 **15 failed / 1 passed**, 수정 후 소규모 host
  검사는 **43 passed / 1 deselected**였다. 현재 설치의 기존 공식 개발
  attachment를 재사용한 전체 Sim 검사는 **995 passed / 5 skipped**다.
  로컬 GPU·Genesis 장면을 초기화하지 않고 JAX CPU를 명시했다.
- GPU2 기준 실측과 수정본 장시간 보행 검증은 진행 중이다. 이 단위 검사
  결과를 보행 안정성이나 실시간 속도 회복의 증거로 해석하지 않는다.

### 실제 세션의 0.6 RTF·갱신 지연 계측과 중복 FK 제거 (2026-10-21)

- 로컬 변경을 stash에 보관하고 원격 `f0049dc`를 fast-forward한 뒤 복원했다.
  원격 `hckang@100.74.222.24`의 실제 등록된 `test` Sim은 `1203cb4` 기반
  `merry-tuna`였다. 해당 컨테이너와 GPU2 RTX A6000만 사용했으며 이미지와
  instance pin은 바꾸지 않았다. dt=.01/substeps=2, MPPI **4096×2**, 두
  **1280×720/최대 30 Hz** 카메라를 유지한 중립 상태의 운영 세션을 계측했다.
  이번 수치를 새 보행 안정성 시험으로 해석하지 않는다.
- 카메라 켬/끔 대조의 안정 구간 평균 loop는 **15.781 / 9.987 ms**였다.
  켬 상태의 command **7.380 ms**, physics **6.418 ms**가 10 ms 예산을
  초과한다. 부모 camera submit 자체는 **.228 ms**였다. 카메라 복제 상태
  갱신·렌더링이 함께 켜질 때 제어·물리 구간이 늘어나는 것은 확인했지만,
  이를 WebRTC 인코더나 메모리 복사 하나의 비용으로 확정하지 않는다.
  command와 physics에는 GPU 완료 대기가 포함되며 physics는 제어기도 포함한다.
- 원본으로 복원한 뒤 DDS를 읽기 전용으로 다시 관찰했다. 실제 **17.685초에
  Sim 11.400초**, **64.46 step/s, RTF .645**였다. 같은 호스트 Sim→Pilot
  메시지 age 중앙값/p95는 **2.05/2.53 ms**, 상태 갱신 간격은 **75.7/110.1 ms**다.
  20 Hz telemetry가 시뮬 시간에 맞춰 나가므로 느린 step은 실제 피드백도
  늦춘다. UI까지의 WAN/영상 종단 지연은 이 값으로 측정한 것이 아니다.
- 팔·그리퍼 속도 쓰기의 FK를 미루고 뒤의 위치 쓰기에서 한 번 갱신한다.
  몸통·다리 속도와 DDS 명령/피드백을 유지한다. 카메라 복제 씬은 root와
  관절을 한 번에 갱신하고, 같은 값의 mock/target/observer 쓰기를 생략한다.
  epoch 초기화와 실패 후 재시도, 부분 snapshot 의미를 유지한다. GPU2에서
  팔 8개 상태와 두 카메라 모델 각각 11개 상태의 모든 링크 자세·속도는
  **finite, 오차 0**이었다. 팔 밖의 DOF 속도도 정확히 같았다.
- 카메라의 소수 마감 시간이 반올림 오차로 갱신되지 않는 경우도 수정했다.
  고정 60 step/s 시험에서 10초 동안 **186→180회**로 중복 슬롯을 제거했다.
  100 step/s의 엄격한 wall 간격 검사에서는 **250회**다. 이는 스케줄러 재현
  수치이며 실제 수신 FPS 측정이 아니다. 누락되던 PyMpc solve 이벤트도 기존
  `go2_mpc_solve` 계측 열로 연결하고 호출 횟수 기준 평균을 검증했다.
- 카메라 패치만의 순차 대조는 평균 **18.514→17.331 ms**, 팔 패치만은
  **18.894→16.585 ms**였다. 각각의 실행에서 초기 두 창을 제외하고 step 수로
  가중했다. 별도 실행·호스트 부하 변동이 있어 이 절감량을 합산하지 않는다.
  **두 카메라를 켠 1.0 RTF 회복은 아직 입증하지 못했다.** GPU GL 렌더링을
  유지한 CPU 자세 계산 대조는 이득이 작아 채택하지 않았다. Genesis static
  mode도 별도 계측만 했으며 기존 기본값은 유지했다.
- 기존 개발 컨테이너에서 로컬 GPU를 비활성화하고 JAX CPU를 명시한 Sim 전체
  pytest는 **979 passed / 5 skipped**다. 첫 실행의 JAX CUDA 초기화 오류는
  환경 지정으로 해소했다. 진단 하네스 fault injection **7 passed**다.
  최종 패치를 모두 합친 새 설치본, 보행 재검증, 전체 required/extended 및
  release gates는 아직 실행하지 않았다. 임시 원격 패치는 원본 bytes·소유권·
  설정을 복원하고 정상 Sim을 다시 켰다.
  [실행 조건·수치 동등성·원시 파일 SHA 기록](../workbench/evidence/curated/sim/runtime_rtf_20261021.json).

### main 통합 전 macOS 호스트 검증 (2026-10-21)

- 원격 `origin/main`을 fetch하고 `pull --ff-only`로 확인했다. 변경 전 HEAD와
  원격은 모두 `1203cb4`여서 가져올 추가 커밋은 없었다. `local/`은 ignore를
  유지하며, 별도 중첩 Git 저장소 `src/convex-mpc-unitree-go2/`는 게시하지 않는다.
- 기존 Conda `elesim-ros`에서 protocol **149 + subtest 5**, Robot **147**,
  UI **113**, 변경 Pilot **148**, 변경 Sim **35**, 변경 setup **37**, 모델
  **6**이 통과했다(서로 다른 테스트 총 **635 passed**, subtest 별도).
  Sit 버튼 테스트의 ImGui 대역에 `text_disabled`를 보완했고 UI 전체가 통과했다.
  Python 변경 파일 79개 AST/JSON 3개 구문 검사와 diff 공백 검사가 통과했다.
- Linux 대상 검사 16개는 macOS에서 통과하지 못했다: Unitree `SO_PEERCRED`/
  Unix `SOCK_SEQPACKET` 9개 실패, 실제 native SDK의 macOS 링크 오류에 따른
  fixture 오류 5개, GNU `realpath -e`를 쓰는 journald archive 2개 실패다.
  SDK 대역 native 검사는 통과했다. 호스트 패키지를 추가하지 않았고 Linux 전용
  요구를 macOS에 맞춰 변경하지 않았다. Canonical Docker required/extended,
  DDS 다중 프로세스와 실제 hardware gates는 generated dev wrapper 부재로 미실행이다.
  원시 로그는 `workbench/evidence/generated/20261021-main-publish/`에 보관한다.

### Jetson Roll 모터 방향 변경 시도 및 되돌림 (2026-10-21)

- **최신 상태:** 사용자가 -1 적용 후 실물과 회색 Sim 팔의 Roll 방향이
  오히려 반대임을 확인하여 Jetson `motor_direction`을 원래 `[1, 1, 1, -1]`로
  되돌렸다. Wrap roll_sign은 +1로 유지했다. 아래 기본값 차이와 소프트웨어
  검증만으로 이 팔의 물리 부호가 잘못됐다고 단정할 수 없다. 롤백 설정의
  loader 검증·Robot PID 유지 확인을 완료했다. 백업과 기록은
  `~/elesim/updates/20261021-roll-motor-revert-192321/`에 있다. 서비스 재시작·
  물리 명령은 보내지 않았으며, 사용자가 Robot을 재시작해야 롤백이 반영된다.

- Wrap +1 적용 후 회색 Sim 팔과 실물 Roll 방향이 반대라는 사용자 관찰을
  조사했다. Robot telemetry의 q는 Sim 관절로 부호 변경 없이 전달된다.
  레포 `HardwareConfig`·Robot 기본 YAML과 Jetson 설치 wheel의 기본값은 모두
  `motor_direction: [1, -1, 1, -1]`인데, Jetson `installed.yaml`만
  `[1, 1, 1, -1]`로 덮어쓰고 있었다. 앞선 manifest +1만으로 실물 모터
  방향까지 일치한다고 판단한 설명은 근거가 부족했다.
- Jetson 설치 설정의 Roll motor_direction만 -1로 복구했다. Wrap roll_sign은
  +1로 유지한다. 이 설정은 native 모터 목표와 Python 측정값 변환 양쪽에
  적용되므로 Sim 표시만 반전하는 보정은 추가하지 않았다. 백업·배포 기록은
  `~/elesim/updates/20261021-roll-motor-polarity/`에 있다. Robot 프로세스를
  재시작하거나 물리 명령을 전송하지 않았으며, 사용자 재시작 전에는 기존
  설정이 계속 실행된다. 이전 계획은 폐기하고 새 측정값으로 다시 계획한다.
- SDK 대역의 양·음 Roll 명령이 raw tick을 반대 방향으로 바꾸고, 실제 Robot
  telemetry 변환이 canonical Roll을 복원하는 회귀를 추가했다. Native/config/
  wrap host 검사 **81 passed**. 실제 실물·Sim 방향 확인은 사용자 재시작 후
  남아 있으며, canonical Docker gates는 generated dev wrapper 부재로 미실행이다.

### Wrap grasp Roll 방향 일치 및 Jetson 배포 (2026-10-21)

- 현재 정책 manifest의 `roll_axis_sign: 1.0`과 학습/Sim 좌표계에 맞춰 Pilot
  `WrapGraspConfig.roll_sign` 기본값을 +1로 변경했다. 이전 -1 보정은 wrap의
  관측·waypoint·lift만 반전시켰다. 일반 IK/슬라이더와 Robot motor mapping은
  변경하지 않았다. 양·음 Roll에서 세 adapter callback을 실제 호출하는 회귀
  검증을 포함한 host wrap suite **50 passed**.
- 설치본과 비교해 `pick/wrap.py`만 변경된 Pilot wheel을 Jetson에 배포하고,
  설치 `wrap_grasp.yaml`에도 `roll_sign: 1.0`을 명시했다. 별도 Python 프로세스로
  기본값·유효 설정과 wheel 전체 파일 일치를 확인했다. 백업과 배포 기록은
  `~/elesim/updates/20261021-wrap-roll/`에 있다. 실행 중 Pilot은 유지했으며,
  적용하려면 사용자 Pilot 재시작이 필요하다. Robot 재시작·물리 명령은 없었다.
- Canonical Docker gates는 generated dev wrapper 부재로 실행하지 않았다.
  실제 wrap 동작의 방향 확인은 사용자 실행 후 남아 있다.

### Use current XYZ와 Sim claw 중심의 표시 오차 (2026-10-21)

- Target 구가 claw 중심보다 약 2 cm 높아 보인다는 사용자 관찰을, 읽기 전용으로
  받은 현재 Robot 자세를 이용한 실제 Genesis CPU 물리 장면에서 재현했다.
  기존 경로는 GO2 몸통만 step 후 복원하고 팔은 물리 계산으로 변위된 채 표시했다.
  두 CAD claw tip connector의 Genesis 월드 좌표 중점을 독립적으로 비교한 결과,
  표시 claw 중심이 Pilot FK보다 **18.183 mm 아래**였다. 직전 배포 전후의
  `Use current XYZ`·grasp FK/오프셋 코드는 동일하며, 임의 Z 보정은 추가하지 않았다.
- Robot mirror 팔도 물리 step 뒤 측정 q로 복원한 다음 렌더링하도록 수정했다.
  Claw 보간은 추가 진행하지 않고 GO2/다리 속도도 arm 쓰기로 덮지 않는다.
  일반 Sim 제어 전환과 reset은 복원 상태를 해제한다. Arm-only/GO2와 매-step/
  주기 제한 viewer 경로를 검증했다. 재현 장면의 수정 후 중점 오차는 0.001 mm 미만이다.
- Host focused **76 passed, 2 skipped**; 실제 Genesis fixture의 before/after PNG와
  raw 결과는 `workbench/evidence/generated/current_xyz_alignment/`, 요약은
  `workbench/evidence/curated/planning/current_xyz_display_20261021.json`이다.
  Mac Sim 소스 변경이며 사용자가 해당 Sim만 재시작하면 적용한다. Pilot/Robot
  배포·재시작과 물리 명령 전송은 하지 않았다. Canonical Docker gates는 기존과
  같이 generated dev wrapper 부재로 실행하지 않았다.

### Target 물체 중심까지의 claw 진입 경로 (2026-10-21)

- Target XYZ를 지름 20 mm 물체의 중심으로 두고, 최종 열린 claw 사이 grasp
  중심이 그 좌표에 1 mm 이내로 도달하도록 수정했다. 물체 앞 접근점에서 끝나는
  경로는 성공으로 받지 않는다. Target 구를 IK·경로 계획과 실행용 resampling
  검사에 모두 포함하며, claw를 포함한 어느 링크도 target 충돌 검사에서 제외하지 않는다.
  자동 claw 닫힘은 추가하지 않았다.
- Claw의 30 mm voxel circumsphere가 실제 빈 공간까지 막던 근사를 15 mm로
  세분화했다. 몸체 형상은 유지하며 휴대 가능한 계획 artifact를 **1005 spheres**로
  재생성했다. CAD claw 표면을 계속 감싸고 지름 20 mm 구가 열린 사이 공간에
  들어가는지 검사한다. UI는 최종 claw 중심을 표시하며 offline CLI 결과는
  `goal_xyz`를 쓰는 schema v2다. DDS protocol-v6 payload 구조는 바뀌지 않는다.
- Host focused **81 passed**; 마지막 Pilot/UI subset **61 passed**. 실제 Jetson
  CUDA 5 mm 이동 계획은 31점, 실행용 61점 모두 충돌 검사를 통과했다. 최종 위치
  오차 **9.54e-7 m**, 최소 target 구 간격 **3.95 mm**였고, 구 중심을 claw 내부에
  둔 충돌도 거부했다. CUDA 초기화·IK·계획 전체 178.2초의 offline probe이며
  물리 이동 검증은 아니다. 증거: `workbench/evidence/curated/planning/curobo_target_grasp_20261021.json`.
- Jetson 역할 venv에 Pilot wheel과 설치된 planning model을 갱신했다. Wheel
  SHA256은 `f6cdb7a85b51b060ee9c353fbaca17a2593096cc1c3a6ea933358f0affb6a1b7`;
  기존 package/model backup과 배포 기록은
  `/home/idim/elesim/updates/20261021-target-claw-center/`다. Staged/installed CLI import,
  module/model hash를 확인했다. 실행 중 Pilot은 재시작하지 않았으며 새 동작은
  사용자가 Pilot·Mac UI를 재시작한 뒤 새 계획부터 적용한다. Robot은 변경하지 않았다.
- Mac 3-process DDS smoke는 현재 test shell의 `elesim_interfaces` overlay가 없어
  시작하지 못했다. Generated dev wrapper도 없어 canonical Docker gates는 실행하지 않았다.

### GO2 mirror와 Ghost 베이스 표시 순서 수정 (2026-10-21)

- 기존 native viewer는 물리 step 직후 본체를 그리고, 그 뒤 GO2를 실측 자세로
  복원했다. 따라서 다음 loop의 plate/mesh 좌표 로그는 정상이면서 화면에만
  접촉으로 일시 변위된 본체가 남을 수 있었다. `SimScene.step`에서 GO2 복원을
  viewer 갱신 앞으로 옮겼다. 매 step 갱신과 주기 제한 갱신에 모두 적용한다.
- Mac `elesim-ros` host focused **57 passed, 2 skipped**. 물리 계산으로 본체가
  20 mm 올라가는 대역에서 두 viewer 갱신 경로 모두 실측 높이를 그리는지 검증했다.
  실제 Genesis headless rasterizer에서도 제어된 20 mm 변위를 사용해, 복원된
  readback과 렌더러 사이 20 mm 차이가 갱신 후 0이 됨을 재현했다. 동일 q/base의
  Ghost와 본체는 plate/housing을 포함한 모든 팔 링크의 렌더 메시 경계가
  0.001 mm 이내로 일치했다. 증거는 `workbench/evidence/generated/ghost_alignment/`.
- 물리 I/O·Jetson 배포·실행 중 프로세스 재시작은 하지 않았다. 사용자가 Sim
  재시작 후 실제 mirror session에서 Ghost 베이스 표시 문제 해결을 확인했다.
  Generated dev wrapper가 없는
  현재 환경에서 canonical Docker gates는 실행하지 않았다.

### Teensy dual-IMU reader (2026-10-20)

- 실제 사용 중인 Full IMU sketch의 **52열 CSV**도 추가 지원했다. 센서별
  `newAny=0`은 수신 나이를 갱신하지 않으며, 전역 tx sequence를 quaternion의
  측정 순번으로 취급하지 않는다. quaternion에서 RPY를 계산하고 gravity/gyro/
  linear_accel/accel/mag, accuracy, BNO timestamp를 보존한다. all-zero 초기
  quaternion은 null 자세다. Full IMU는 기존 펌웨어 그대로 raw 진단이 가능하다.
  line bound는 전체 데이터를 담도록 2048 bytes다. Full IMU의 최근 report 수신과
  quaternion freshness 미검증 상태를 UI에서 분리한다.
- 최신 host focused **130 passed + 10 subtests**, UI 대역 **2 passed**다.
  52열 필드 대응, quaternion→RPY, 초기 미측정, 두 센서의 독립적인 newAny,
  잘린/비정상 frame, 실제 PTY full-CSV 수신과 telemetry 직렬화를 검증했다.
  아래의 실제 GUI/DDS/Jetson 및 canonical 미검증 범위는 유지된다.
- Robot-local POSIX reader를 추가했다. 선택한 USB serial 하나만 열고 IMU1/IMU2의
  최신 quaternion/RPY(rad)를 별도 보관하며, bounded parser, 재연결과 종료를
  지원한다. `runtime.imu_device`는 기본 비활성화다. 독립 진단 명령은
  `elesim-imu-read --device <실제 Teensy 포트>`다.
- telemetry `imu_reader`를 Pilot을 거쳐 UI IMU panel에 전달한다. 모델 활성화와
  실제 raw sample 수신 상태를 분리했다. native 보정 입력에는 아직 연결하지 않는다.
- 펌웨어에 실제 sensor report 수신 시각과 순번을 추가했다. 반복 출력은 sample
  age를 갱신하지 않는다. 기존 10-field 펌웨어는 읽되 freshness unverified다.
- host Conda Python focused Robot/Pilot/protocol **126 passed + 5 subtests**,
  UI 표시 **1 passed** (imgui import stub 및 렌더 함수 대역)다. 실제 PTY 수신,
  disconnect/stop, 잘못된 프레임, 순번 wrap/replay, wire roundtrip을 포함한다.
  실제 GUI rendering/DDS process smoke와 Jetson USB/권한/재연결/Teensy flashing은
  미검증이다. 알려진 설치본의 generated dev wrapper가 없어 canonical gates는
  실행하지 않았다. Arduino CLI의 Teensy core는 있으나 SparkFun BNO080 library가
  없어 펌웨어 compile도 실행하지 않았다. 현재 설치본 wheel은 갱신하지 않았다.

### cuRobo 취소와 실행 tick 지연 처리 (2026-10-20)

- CUDA 호출을 Pilot DDS thread의 취소 불가능한 Python worker에서 분리해,
  성공한 planning worker는 context를 유지하고 Cancel/timeout은 그 프로세스를
  종료하도록 했다. model/solver initialization은 각각 900초, IK solve/path
  planning/validation은 각각 180초로 제한하며 UI stage를 분리해 표시한다.
- 기존 `trajectory scheduling overrun`은 Pilot의 제어 tick이 다음 25ms
  waypoint 시각보다 100ms 이상 늦으면 경로 전체를 취소했다. 새 동작은 100ms
  초과 지연에서 경로 시계만 재설정하고 waypoint를 건너뛰지 않는다. 250ms를
  넘는 정지는 실제 지연 시간과 함께 경로를 취소한다.
- Pilot wheel만 Jetson 역할 venv에 설치했다. wheel SHA256은
  `d9e962380f5d6d8a9da23f96c0115ca6df76d1f1e1256baf852c3d2fa587d010`, 설치 전
  package backup은 `/home/idim/elesim/updates/20261020-curobo-worker/before-install.tar.gz`다.
  설치 모듈 import/signature를 확인했다. 기존 Pilot PID 2499909는 계속 실행 중이므로
  새 코드는 아직 해당 process에 적용되지 않았다. Pilot과 Robot은 재시작하지 않았고
  물리 명령도 보내지 않았다.
- Host focused Pilot/CuRobo tests **66 passed, 1 skipped**; Python compile와 diff
  검사를 통과했다. Jetson 실 GPU planning과 실물 경로 실행은 하지 않았다.

### 전류 한계 safety latch UI 복구 경로 배포 (2026-10-20)

- Hardware 패널의 software fault 복구 버튼을 UI→Pilot→Robot에 연결했다.
  Robot은 torque OFF, fresh telemetry와 local fault 재발 여부를 검사한 뒤에만
  latch를 지운다. 검사가 실패하면 latch를 유지하고 이유를 반환한다. 성공해도
  torque는 OFF이며, torque-on은 별도 조작이다. 모터 Hardware Error는 기존처럼
  별도의 확인된 motor reboot 경로를 쓴다.
- Jetson Pilot/protocol wheel을 갱신했다. 원본은
  `/home/idim/elesim/updates/20261020-safety-clear/pilot-before-safety-clear.tar.gz`,
  wheel은 같은 디렉터리의 `wheels/`에 보관했다. 설치 전후 Robot과 Unitree
  bridge는 active였으며 PID 2496380/2499294를 유지했다. 두 서비스를 재시작하거나
  모터 명령을 보내지 않았다. Robot runtime의 `clear_fault` 경로는 이미 설치되어
  있어 이번 복구 연결에 Robot 패키지 변경은 필요하지 않다.
- Mac UI는 checkout source를 사용하므로 UI를 다시 시작해야 새 버튼이 표시된다.
  Jetson Pilot도 새 service allowlist와 handler를 로드하려면 사용자가 재시작해야
  한다. 현재 실행 중인 두 프로세스는 설치된 새 파일을 아직 메모리에 읽지 않았다.
- UI/Pilot/Robot/protocol 집중 검사 **83 passed, 1 skipped**다. Skip은 Robot
  runtime test 안의 기존 선택 조건이며, 실제 recovery/Torque ON은 실행하지 않았다.

### Jetson 모터 Hardware Error 재부팅 복구 배포 (2026-10-20)

- Hardware 패널에 fault 복구 버튼을 추가했다. 팔 지지 확인과 Torque OFF를
  요구하고, 활성 cuRobo 경로는 취소하되 Robot lease를 유지한다. Robot은
  단일 DYNAMIXEL Hardware Error만 허용하며 재부팅 전후 실제 Torque Enable
  레지스터를 검사한다. 현재 위치를 목표로 기록·확인하고, 재부팅 후 모든 축의
  Torque OFF와 정상 telemetry를 검증해야 latch를 해제한다. DDS major 6은 유지한다.
- Robot/Pilot/protocol wheel과 Jetson ARM64 ABI 5 native library를 설치했다.
  Robot/Pilot import, recovery command allowlist 및 ABI 5를 읽기 전용으로
  확인했다. Mac `elesim-ros` UI/protocol은 이 checkout의 source를 사용한다.
- 설치 전 Robot/Pilot/protocol package, console scripts와 native library를
  `/home/idim/elesim/updates/20261020-motor-reboot/before-install.tar.gz`에
  백업했다. Robot은 배포 전후 PID 2496380으로 active였고 두 서비스 모두 배포 후
  active다. restart나 모터 명령은 보내지 않았다. 현재 Robot 프로세스 메모리에는
  이전 ABI 4가 남아 있어 새 복구 기능은 사용자가 Robot을 재시작한 뒤 적용된다.
  Pilot은 배포 확인 시 실행 중이지 않았다.
- Mac Robot/Pilot/protocol 집중 검사 **120 passed, 1 skipped**, UI 복구 검사
  **4 passed**다. Jetson에서는 native controller를 ARM64로 다시 빌드하고 ABI 5,
  wheel import와 protocol allowlist를 확인했다. 실제 모터 복구 동작은 실행하지 않았다.

### Jetson ZED Mini 드라이버와 카메라 SDK 격리 (2026-10-20)

- UI→Pilot 읽기 전용 snapshot에서 DDS/Pilot online과 응답을 확인했다.
  Robot의 연결 종료는 잘못 선택된 RealSense SDK의 장치 초기화가 GIL을
  15초 동안 점유해 Unitree keepalive를 막는 것으로 재현했다.
- `camera.backend: zed`와 ZED Mini의 왼쪽 영상/정렬된 mm depth/SDK intrinsics
  경로를 추가했다. 신규 source 기본값은 ZED, 필드가 없는 기존 설정은
  RealSense다. 카메라 SDK와 RGBD publisher를 spawn 자식 프로세스로 격리하고
  상태는 bounded latest-only로 전달한다. 종료 요청도 child lock을 기다리지
  않는 shared flag를 사용한다. 기존 deadman 기준과 DDS 계약은 유지한다.
- Mac 카메라/설정/lifecycle 검사 **42 passed**, Jetson 카메라/lifecycle/IPC
  검사 **32 passed**다. SDK가 GIL을 잡는 15초 대기 double에서도 부모 상태
  조회와 제한 시간 내 종료를 검증했다. 실제 ZED SDK 5.4.1 probe의 부모
  최대 tick 간격은 약 21 ms였으며 네트워크 publish와 물리 이동은 없었다.
- Robot wheel과 설치된 `camera.backend: zed` 설정을 갱신했다. 실제 장치는
  아직 `ZED-M / serial 0 / NOT AVAILABLE`이므로 영상/depth 수신은 미검증이다.
  사용자가 USB 3 케이블을 다시 연결하기로 했다. Robot/bridge는 현재 active다.
  백업은 Jetson
  `updates/20261020-go2/zed/before-zed.tar.gz`에 보관했다.

### Jetson GO2 관절 상태 미러 누락 수정 (2026-10-20)

- Bridge의 LowState parser는 12개 관절각을 읽고 IPC `OdomSample.leg_q`까지
  전달했지만, `RobotRuntime.state()`가 DDS telemetry의 `go2` object에 관절각을
  복사하지 않았다. Sim의 기존 수신/Genesis mirror 경로는 `go2.leg_q`를
  기대하므로 base pose와 arm은 동기화돼도 GO2 다리는 갱신되지 않았다.
- Robot telemetry에 측정된 `sample.leg_q`를 `go2.leg_q`로 포함하도록 수정하고
  Jetson Robot wheel을 갱신했다. 설정의 GO2 활성화와 ZED backend를 유지했다.
  적용은 Robot 프로세스 재시작 후부터 유효하다.
- 읽기 전용 systemd/journal 확인에서 Robot은 DDS discovery 후 GO2 IPC peer
  disconnect로 재시작된 기록도 보였다. 이 연결 끊김은 관절값 필드 누락과
  별개의 문제이며, 재시작 뒤에도 반복되면 같은 journal의 직전 traceback과
  bridge 로그를 따라 추가 조사해야 한다.

### Jetson Torque ON 중 GO2 IPC heartbeat 유지 (2026-10-20)

- Robot main loop가 DDS motion command를 동기 처리하면서 같은 loop에서만
  Unitree IPC heartbeat를 보냈다. Dynamixel Torque ON의 mode/profile/goal
  preload가 deadman 0.5초를 넘기면 bridge가 IPC를 끊고, Robot cleanup이 arm
  device를 닫아 UI Apply Port가 잠깐 OFF로 보일 수 있다. native torque-on은
  현재 goal을 preload하기 위해 시작할 때 잠시 motor torque register를
  비활성화하므로, 이는 UI apply-port 설정과는 별개다.
- `UnitreeIpcClient`에 순서 번호와 socket write lock을 공유하는 독립 heartbeat
  thread를 넣었다. hardware 명령이 main loop를 점유해도 GO2 deadman heartbeat는
  계속 나간다. bridge의 0.5초 deadman과 실제 Torque 동작은 변경하지 않았다.
- Robot wheel을 Jetson에 설치했다. 현재 실행 중 프로세스에는 아직 반영되지
  않으므로 사용자가 Robot 서비스를 재시작해야 한다. 이 변경으로 실제 Torque
  ON은 시도하지 않았다. Python 문법 검사와 wheel 빌드는 완료했고, 테스트는
  실행하지 않았다.

### Jetson Unitree bridge의 시작 준비 상태 보장 (2026-10-20)

- 실제 활성화에서 Robot이 Unitree socket 생성 전에 시작하고
  `FileNotFoundError`로 실패했다. Robot의 재시작은 `PartOf` bridge까지
  재시작해 같은 경합을 반복했다. 실패 루프를 중지했으며 두 실제 unit은
  수정 적용 후에도 inactive 상태다.
- Standalone unit과 installer 생성 unit을 `Type=notify`,
  `NotifyAccess=main`, `TimeoutStartSec=30`으로 맞췄다. Unitree backend 초기화와
  IPC listen 이후에만 daemon이 `READY=1`을 보낸다. GO2 비활성 모드는
  Unitree context 없이 준비 완료를 알린다. 기존 Robot의 `After`/`BindsTo`,
  peer credential 검증과 deadman은 유지한다.
- Jetson IPC 회귀 **22 passed**, Mac 설정/생성 unit 회귀 **31 passed**와
  notify/팔 전용/overlay focused 검사를 통과했다. Jetson 실제 user-systemd에서
  2초 지연 fake backend와 `After`/`Requires` consumer를 실행해 socket 준비 후
  종속 서비스가 성공함을 확인했다. 이 검증은 ROS/물리 장치를 사용하지 않는다.
- Robot wheel과 설치된 bridge unit을 갱신하고 `daemon-reload`했다.
  `systemctl show`로 Type=notify, NotifyAccess=main, 시작 timeout 30초와
  inactive를 확인했다. 실제 Robot/bridge 재시작은 사용자 실행으로 남겼다.
  백업과 검사 자료는 Jetson `updates/20261020-go2/readiness/`에 보관했다.

### Jetson GO2 재배포 준비와 실제 DDS domain 확인 (2026-10-20)

- `idim@147.46.175.23:2227`의 Robot은 정지 상태이고 기존 Unitree bridge만
  실행 중이었다. 과거 journal에서 잘못된 `eth0`와 bridge socket 부재에 따른
  시작 실패를 확인했다. 현재 물리 링크는 `eno1`, 1 Gbps이며
  `192.168.123.99/24`에서 GO2 `192.168.123.161` ping이 성공했다.
- GO2의 실제 DDS graph는 **domain 0**이었다. 기존 domain 1은 10초간
  Sport/LowState 모두 0개, domain 0은 8초간 Sport 2400개와 LowState 약
  4000개를 수신했다. 실제 publisher의 QoS는 reliable/volatile이며 reliable와
  best-effort 구독 모두 수신됐다. 읽기 전용 구독 probe이고 이동 명령은 없다.
  설치 Robot의 Unitree 설정을 `eno1 / 0`으로 갱신했으며 inter-host
  EleSim `tailscale0 / 68`과 분리했다.
- 새 wrapper가 Unitree workspace를 읽을 때 `COLCON_TRACE: unbound variable`로
  실패하는 것도 실제 Jetson에서 재현했다. daemon의 overlay source에서
  nounset을 끄고 errexit/pipefail은 유지했다. unset optional variable과 source
  실패 전파 회귀를 추가했으며 Mac focused 4 passed, Jetson 전체 IPC 17 passed다.
  실제 ROS workspace source와 native loader 경로 재실행은 유지한다.
- 기존 설치의 Python package·설정·실행 파일·native library를 백업하고 현재
  Robot/Pilot/protocol wheel 및 Jetson에서 빌드한 native ABI 4를 설치했다.
  Robot의 schema 5 그리퍼 load threshold 250 permille와 GO2 활성화를 적용했다.
  Robot/Pilot 설정, native ABI, 현재 생성 arm model 로딩을 확인했다. 모델
  bounds는 Linear [-0.172666667, 0] m로 유지하며 Pilot도 use_go2=true다.
  Jetson C++ SDK 대역 기반 native motion/contact 회귀도 **5 passed**다.
- Mac 실물 연결용 별도 설정/전체 실행 명령은 ignored
  `local/jetson-go2/README.md`에 있다. UI는 hardware/GO2를 활성화하고 Sim은
  Robot GO2/팔 측정 상태를 mirror한다. 기존 `local/mac-sim`과 Jetson의
  미커밋 Git checkout은 보존했다. 자동 stand/gait는 none이다.
- 실제 모터 포트를 열거나 서비스를 재시작하지 않았다. 현재 실행 중인 이전
  bridge는 재시작 전까지 domain 1을 유지하며 새 Robot/Pilot의 실제 연결·
  command/telemetry round trip은 재시작 후 확인해야 한다. 학습 서버 GPU 2의
  학습은 계속 진행 중이다. 서버 백업/검사 결과는
  `/home/idim/elesim/updates/20261020-go2/`, 로컬 수신 증거는
  `workbench/evidence/generated/20261020-jetson-go2/`다.

### 현재 팔 모델의 Linux CUDA 재학습 준비 (2026-10-20)

- 사용자 지정 학습 서버 `hckang@147.47.191.80:2222`에 키 인증으로 접속했다.
  사용자 지시에 따라 학습·검증은 물리 GPU 2(RTX A6000, 48 GiB)만 사용한다.
  `CUDA_VISIBLE_DEVICES=2`를 적용하고 실제 compute PID의 GPU UUID를 대조했다.
- 빈 `~/elesim`에 현재 HEAD `1203cb4`를 checkout하고 Mac의 미커밋 patch를
  적용했다. 학습 관련 소스·설정·모델 170개 파일의 SHA-256이 일치한다.
  서버의 기존 `elesim-rl` 환경을 Genesis 1.4.1 / Quadrants 1.3.0 /
  Torch 2.12.1 / NumPy 1.26.4 / rsl-rl-lib 5.4.0 / tensordict 0.14.2로
  맞췄으며 `pip check`를 통과했다. 변경 전후 package 목록을 보관했다.
- Linear [-0.172666667, 0] m, Roll ±90°, Bend ±36°/node와
  command direction `[1, 1, 1, -1]`을 canonical Sim 설정에서 읽는다.
  학습 시 GO2 base와 다리는 고정하며 Raibert/JAX 보행을 사용하지 않는다.
  새 run은 기존 checkpoint를 resume하지 않는다.
- CUDA 4-env PPO 2-iteration smoke가 정상 종료됐다. 총 9개 종료 episode의
  lift 성공 4개, collision 3개, retention 2개이며 timeout은 별도 중복 집계다.
  이는 CUDA 실행·체크포인트 저장과 초기 task의 작동 확인이며 학습 수렴이나
  최종 Home 시작 성공률의 검증은 아니다.
- GPU 2 EGL 렌더에서 near-wrap 정적 자세의 coverage 272°, 물체 접촉 8개와
  arm/floor·arm/GO2·arm/self 접촉 없음이 확인됐다. 팔을 먼저 배치하고
  물체를 옮기는 렌더이므로 실제 접근 경로 전체를 검증한 것은 아니다.
- Stage 1 본 학습을 GPU 2, **256 env / 1500 iterations**, 새 가중치로
  시작했다(PID `1422752`). 초기 PPO 반복이 정상 진행 중이며 단일 반복은
  약 17초다. 초기 checkpoint의 부동소수 tensor 74개가 모두 유한하며
  학습 중에도 170개 source/model hash가 일치한다. 저장 경로는
  `var/rl/sim/wrap_grasp_current_arm/stage1_gpu2-20261020/`,
  로그는 같은 상위 디렉터리의 `gpu2-20261020.log`다. 학습 완료와 최종
  Home 시작 평가·Stage 2/3 일반화·실물 이식은 아직 검증하지 않았다.
- ignored overlay/서버 실행 안내: `local/rl-retrain/`. 서버 evidence:
  `workbench/evidence/generated/20261020-rl-gpu2/`, smoke run:
  `var/rl/sim/wrap_grasp_current_arm/stage1_gpu2-smoke-20261020/`.
  실제 Robot과 사용자 Sim/UI/Pilot은 재시작하지 않았다.

### macOS Raibert 시작 직후 전도 수정 (2026-10-20)

- 하드웨어 없는 Mac 설정의 `raibert_trot`에서 명령 없이도 전도했다.
  Genesis 1.4.1 실제 GO2 모델의 같은 자세 IK에서 앞다리 목표
  `[0.1, 0.8, -1.5]`가 `[0.0, 0.1, 0.8]`로 추출되는 것을 재현했다.
  Free root는 qpos 7개/DOF 6개인데 IK 결과를 DOF 인덱스로 읽고 있었다.
  각 관절의 `qs_idx_local`로 추출하며 잘못된 크기/비유한 결과는 기존
  fallback 자세를 유지한다.
- 인덱스만 수정한 GO2+팔 CPU 실험도 0.61초에 몸통 높이가 0.147 m로
  내려앉았다. 정지 중 매 tick 현재 접지 위치를 IK로 다시 푸는 제어가
  관절 처짐을 목표값에 반영해 지지력을 잃었다. 초기 Raibert의 고정 stand
  관절 목표를 복원하고 다음 보행의 발 기준점은 실제 발 위치로 갱신한다.
- JAX/MPPI 도입 직전 `67d8b1e^`와 현재 HEAD의 Raibert 제어 파일은
  동일했다. qpos 인덱스 코드는 `f2c463a`, 정지 IK 경로는 `8e0b43c`부터
  존재한다. 사용자가 정상 동작을 확인했던 정확한 커밋/환경은 아직 특정하지
  않았으며 JAX 도입이나 macOS 자체가 직접 원인이라고 단정하지 않는다.
- Conda `elesim-ros` focused 회귀 **38 passed**. 실제 Mac CPU와 Metal의
  같은 GO2+팔/평면/기본 arm pose에서 각각 500 step(5초)을 명령 없이
  수행했다. 두 실행 모두 마지막 측정 높이 약 0.3025 m, 측정 최대
  roll/pitch 약 0.009°/0.577°로 전도하지 않았다. 카메라/DDS 없는 독립
  물리 실험이며 보행·자세 전환·다른 payload/지형의 안정성 검증은 아니다.
  실행 중인 사용자 Sim/UI/Pilot과 실제 Robot은 재시작하지 않았다.
- evidence: `workbench/evidence/generated/20261020-raibert-qpos/`.
  기존 개발 attachment가 이 Mac에 제공되지 않아 canonical
  required/extended/release gate는 실행하지 않았다.

### 원격 고아 청소의 Mounts 순서 오판 수정 (2026-10-19)

- 원격 `hckang@100.74.222.24`의 설치 manifest와 pinned Docker Engine을
  확인한 뒤 종료된 설치 소유 컨테이너 68개를 8회 read-only inspect했다.
  66개에서 `Mounts` 배열의 순서만 달라지는 것을 확인했다. 배열 원소를
  JSON으로 정규화해 정렬하면 내용은 같았으며, 기존 전체 JSON 비교는
  이를 metadata 경합으로 오판해 청소를 중단했다.
- 삭제 전 재검사에서 `Mounts` 원소의 순서만 정규화한다. 원소 중복과
  모든 필드 내용, 나머지 inspect metadata의 비교는 유지한다. 실제
  변경 시 대상 컨테이너 이름/ID와 변경된 최상위 항목 이름을 표시한다.
- 이번 수정의 테스트는 추가·실행하지 않았다. 실제 청소·설치본 갱신·
  푸쉬도 실행하지 않았다. 아래 자동 청소 회귀 수치는 이 수정 이전 결과다.

### 자동 청소에 종료된 고아 역할 컨테이너 회수 추가 (2026-10-19)

- 성공한 update/release와 공개 instance `up`의 기존 collector에서 종료된
  고아 역할 컨테이너를 먼저 제거한 뒤 이미지·태그·별칭을 회수한다. 설치
  UUID, pinned Engine, Compose project/설정 경로, 역할별 release image ID를
  대조한다. 저장된 실행 스크립트의 과거 `tddwalk` custom-name 일회성
  Compose 실행은 이 조건에 해당한다. 이름 접두사나 orphan 경고만으로
  삭제하지 않으며 정확한 container ID에 비강제 `rm`만 사용한다.
- 현재 등록된 정확한 역할 컨테이너는 정지 상태여도 보존한다. 실행 중,
  paused/restarting/created, dev/tools/Tailscale/Coturn, 다른 설치와 출처
  불명 컨테이너도 보존한다. 남은 모든 컨테이너는 이미지와 생성 시 참조한
  태그를 계속 보호한다. 보존된 logs/security/secrets만 남은 제거 인스턴스는
  등록으로 취급하지 않고, 불완전 registry/transaction은 청소를 중단한다.
- 삭제 전 image/container metadata와 참조를 재검사하고, 고아 제거 후 다시
  모든 container 참조를 확인한다. registry/provenance 오류, 상태 경합,
  Docker의 삭제 거부를 강제 옵션으로 우회하지 않는다. 기존 자동 실행
  시점만 사용하며 background/down/remove hook은 추가하지 않았다.
- host collector **113 passed**, 이미지 이름·instance remove·release·updater
  관련 **73 passed**, container installer **84 passed / 5 failed**다. 실패한
  5개는 기존 X11 fixture의 Unix socket bind가 sandbox에서 `EPERM`으로
  거부됐다. 실제 Docker 삭제와 canonical dev container 검사는 실행하지
  않았다. 설치본을 갱신하지 않았으며 사용자 지시에 따라 푸쉬하지 않았다.

### Bootstrap 조회 모듈 manifest 누락 수정 (2026-10-19)

- `afb1e14`의 `instance_listing.py`가 Bootstrap과 release verifier의 명시적
  Python 모듈 목록에서 누락돼 source snapshot 검증이 실패했다. 기존 검사
  3개에서 같은 누락을 재현한 뒤 두 목록에 모듈을 등록했다.
- 소스/Bootstrap 목록 일치를 exact equality로 확인하고, 조회 모듈 누락 거부와
  release package/wheel 포함 검사를 보강했다. host Bootstrap 관련 **28 passed**,
  release verifier/실제 infrastructure 복사·wheel 생성 검사 **52 passed**다.
  실제 checkout에서 허용된 파일 462개를 추린 source snapshot도 검증을 통과했다.
- 실제 설치/update와 canonical dev container 검사는 실행하지 않았다.
  사용자 지시에 따라 푸쉬하지 않았다.

### 인스턴스별 콜사인 조회 (2026-10-19)

- `elesim list`로 해당 설치의 등록된 instance ID와 그 아래 역할별 콜사인을
  표시한다. `--json`도 지원한다. stopped instance를 포함하며, 각 pin의
  release manifest에서 배정된 역할만 읽는다. 새 release의 별칭이나 Docker
  가용 이미지 목록으로 기존 instance의 표시를 바꾸지 않는다.
- public wrapper는 기존 `elesim-instance --list`를 거쳐 stdlib-only host
  maintenance bundle을 실행한다. Docker 실행·등록/lock 파일 생성이 없고,
  schema-v2/v3·과거 별칭·제거 후 보존한 로그 디렉터리를 처리한다.
- 실제 로컬 registry에서 `readiness → ui: donkey`, `test → ui: cow`를
  read-only 조회했다. 설치본 wrapper 갱신과 실제 multi-host 조회는 미실행이다.
- host focused **43 passed**이며, 설치·native wrapper·ownership·uninstall까지
  포함한 관련 batch는 **225 passed / 5 failed**다. 5개는 기존 X11 fixture의
  Unix socket `bind`가 sandbox에서 `EPERM`으로 막혔다. 새 listing·isolated
  stdlib bundle·다른 checkout에서의 wrapper 실행 검사는 통과했다. 공식 dev
  container 검사는 이번에 실행하지 않았으며 앞선 Docker 권한 검토가 모델
  용량 오류로 막힌 상태다. 실제 Docker/원격/GPU 동작을 입증하는 결과는 아니다.

### MPPI 기본값 4096×2 복귀 (2026-10-19)

- 사용자 요청으로 YAML, locomotion/MPC config dataclass와 force adapter의
  GPU 반복 기본값을 **2회**로 되돌렸다. 현재 배포 기본값은 **4096×2**이며
  아래 4096×1 유지/반영 기록은 각 비교 당시의 결정이다.
- dt=.01/substeps=2, 관절 제어·접촉 관측 100 Hz와 MPPI 갱신 50 Hz를 유지한다.
  설정 기본값 회귀 기대값과 architecture 문서를 2회에 맞췄다.
- 호스트의 실제 config loader와 두 config dataclass·force adapter 기본값에서
  4096×2 및 dt=.01/substeps=2를 확인했다. 공식 개발 wrapper의 설정 pytest는
  자동 권한 검토의 `Selected model is at capacity` 오류로 실행되지 않았다.
- 별도 4096×1 영상 촬영은 t=18.24초에 controller fault로 중단됐다. 구체적
  원인은 아직 확인하지 않았으며, 이번 기본값 복귀로 해결됐다고 판단하지 않는다.
  이 복귀 후 실제 Genesis 보행·영상 재촬영은 아직 실행하지 않았다.

### MPPI 2048×1 비교·4096×1 유지 (2026-10-19)

- 공식 Sim `0debb43` 설치 이미지에서 후보 수만 2048→4096→4096→2048로
  비교했다. GPU2, dt=.01/substeps=2, 관측·제어 100 Hz, 기본 optimizer 50 Hz,
  iteration=1·seed42, 작은 팔 동작과 두 1280×720 H264/NVENC loopback을 유지했다.
  네 방향 8초/20초·전진/후진 반복 출발/정지와 네 방향 짧은 재시험에서
  **2048×1과 4096×1 모두 14/14 통과**했다. 1024 비교에서 실패했던 긴 후진·
  반복 후진·긴 좌측 이동 후 stand 복귀도 2048에서는 통과했다.
- 같은 **14개 시험, 각 15,600 정상 보행 step**에서 평균 loop는
  **4096: 9.4958 ms / 2048: 9.4974 ms**로 전체 처리 시간 개선은 보이지 않았다.
  p95도 **12.756 / 12.745 ms**였다. MPPI 호출 평균만 **4.089→3.840 ms
  (-6.08%)**로 감소했다. 몸통 roll/pitch RMS는 **2.59/3.61→2.64/4.02도**다.
  평균 loop가 10 ms 미만이어도 매 step의 마감을 입증한 결과는 아니다.
- 이번에 다시 측정한 4096 대조군은 앞선 1024 비교의 10.255 ms와 다르다.
  실행 간 처리 시간 변동의 원인은 분리하지 않았다. 서로 다른 비교의 후보
  시간만 이어 붙여 샘플 수에 따른 처리 시간 곡선으로 해석하지 않는다.
- timer는 제어·물리·camera submit이며 팔 명령 쓰기와 10 Hz 실제 pose 검사는
  timer 밖이다. 초기 build/JIT와 실제 UI/DDS/WAN은 지연 비교 범위 밖이다.
  한 seed의 full/short 평지 비교이며 일반적인 안정성 확률을 입증하지 않는다.
  공중 지지력과 영상 decode 오류는 모두 0회다.
- 사용자 요청대로 **배포 기본값은 4096×1로 유지**한다. 두 후보 비교는 공식
  설치 이미지의 config override였으며 runtime source/default는 그대로다.
  시험 container 종료·GPU2 유휴·기존 instance pin 보존을 확인했다.
  [전체 비교·측정 조건·raw SHA](../workbench/evidence/curated/sim/mppi_samples_2048_20261019.json)를 보존했다.

### MPPI 1024×1 보행·처리 시간 비교 (2026-10-19)

- 양쪽에 공식 업데이트한 `0debb43`의 원격 Sim 설치본 `e95e78f0…`
  (`merry-goose`)에서 후보 수만 4096→1024→1024→4096으로 비교했다.
  GPU2, dt=.01/substeps=2, 관측·제어 100 Hz, 기본 optimizer 50 Hz,
  iteration=1·seed42·기존 gain·제한·접촉/정지 로직을 유지했다.
  작은 팔 bend 동작과 두 1280×720 H264/NVENC loopback 송수신을 함께 실행했다.
- 첫 쌍은 네 방향 8초/20초와 전진·후진 3회 출발/정지 10개씩, 반복 쌍은
  네 방향 8초 4개씩이다. **4096×1은 14/14, 1024×1은 11/14 통과**했다.
  1024의 짧은 보행 8개는 모두 통과했지만 다음 세 시험은 실패했다.

  - 긴 후진: 준비 2초를 포함한 t=14.45초에
    `GPU MPC produced no finite force trajectory`로 safe fault에 들어갔다.
  - 긴 좌측 이동: 20초 이동 뒤 zero command를 4초 줘도 stage가 `walk`였다.
    controller fault는 없지만 stand 복귀 조건을 만족하지 못했다.
  - 반복 후진: 두 번째 후진의 t=19.30초에 RL 착지 timeout 0.30초로 fault가 났다.
- 양쪽이 완주한 **11개 시험, 각 10,200 정상 보행 step**에서 평균 loop는
  **10.255→8.966 ms (-12.57%, 1.290 ms 절감)**, p95는 **13.840→12.249 ms**다.
  MPPI 호출 평균은 **4.182→3.645 ms (-12.85%)**였다. 같은 완주 구간의 몸통
  roll/pitch RMS도 **2.73/3.26→3.18/3.73도**로 증가했다. 실패 시험은 모두
  보존하며 정상 완주 latency 비교에서만 제외했다.
- timer는 제어·물리·camera submit이다. 팔 명령 쓰기와 10 Hz 실제 pose 검사는
  timer 밖이고 trial sim/wall에는 포함된다. 초기 build/JIT·실제 UI/DDS/WAN은
  지연 비교 범위 밖이다. **1024의 평균이 10 ms 미만이어도 p95는 예산을 넘는다.**
  한 seed의 full/short 비교이며 장시간 실패 재현율이나 일반적인 안정성 확률을
  입증한 결과는 아니다. 관측된 공중 지지력과 영상 decode 오류는 모두 0회다.
- **배포 기본값은 4096×1로 유지**한다. 이번 변경은 결과·문서이며, 시험은
  공식 동일 설치 이미지에서 config override로 실행했다. 네 실행 모두 controller/
  JAX/payload/bridge/snapshot SHA와 실제 GPU 후보 수·iteration을 확인했다.
  시험 container 종료·GPU2 유휴·기존 instance pin 보존을 확인했다.
  [전체 시험·실패·측정 조건·raw SHA](../workbench/evidence/curated/sim/mppi_samples_1024_20261019.json)를 보존했다.

### 100 Hz 유지·MPPI 4096×1 비교와 기본값 반영 (2026-10-19)

- 같은 공식 Sim `331e754`에서 MPPI 반복 횟수만 2→1→1→2로 비교했다.
  GPU2, dt=.01/substeps=2, 관측·토크 100 Hz, 기본 optimizer 50 Hz,
  4096 samples·seed42·gain·필터를 유지했다. 작은 팔 bend 동작과 두
  1280×720 H264/NVENC loopback 송수신을 함께 실행했다.
- 첫 쌍은 8초/20초 전후좌우와 전진·후진 3회 출발/정지 10개씩, 반복 쌍은
  8초 전후좌우 4개씩이다. **4096×1은 14/14 통과, 4096×2는 11/14 통과**다.
  2회 설정의 장시간 후진·후진 반복·반복 쌍의 짧은 후진에서 fault가 발생했다.
  앞선 cadence 비교의 2회 설정은 14/14 통과였으므로 이번 결과로 1회가
  보편적으로 더 안정적이라고 주장하지 않는다. 실행 간 변동 원인은 미확정이다.
- 두 설정이 모두 완주한 **11개 시험, 각 11,400 정상 보행 step**을 같은
  구간으로 비교했다. 평균 loop는 **10.482→10.060 ms (-4.03%, .422 ms 절감)**,
  p95는 **14.788→13.482 ms**였다. MPPI 호출 평균은 **5.614→4.194 ms
  (-25.29%)**, p95는 **6.425→4.799 ms**였다. optimizer 개선을 전체 loop 개선과
  구분한다. **평균과 p95 모두 10 ms 마감을 달성했다는 결과는 아니다.**
- timer는 제어·물리·camera submit이다. 팔 명령 쓰기·10 Hz 실제 pose 검사는
  timer 밖이고, 전체 trial sim/wall에는 그 비용이 포함된다. 초기 build/JIT,
  per-tick JSON/PNG·실제 UI/DDS/WAN은 지연 비교에서 제외했다. 실패한 시험을
  숨기지 않고 latency 비교에서는 양쪽 완주 시험만 같은 쌍으로 골랐다.
- YAML·두 config dataclass·force adapter의 GPU 반복 기본값을 1로 맞췄다.
  2회 명시 선택은 유지한다. Python 함수 body는 모두 그대로이며 feedback·
  접촉·payload·제한·DDS 모터 쓰기·정지/reset을 바꾸지 않았다. 세 기본값
  회귀가 red→green이고 전체 Sim은 **926 passed / 3 skipped / 1 failed**다.
  실패는 기존 RL 시작 좌표 검사다. 실제 DDS 모터 mailbox·4-process topology,
  protocol **145개**가 통과했다. full required/extended·isolated-release와
  WAN/SROS2/hardware gate는 미검증이다.
- `0debb43` push 후 양쪽 공식 `elesim-update`를 완료했다. 운영 pin은
  유지하고 원격 `tddwalk`만 release `e95e78f0…`, Sim `merry-goose`로 교체했다.
  새 이미지의 8개 module SHA, 기본 dt=.01/substeps=2·4096×1, GPU2 JIT 초기화를
  확인했다. 이 설치 확인은 설정/SHA/JIT 범위이며 설치 후 Genesis 보행을
  다시 실행한 검사는 아니다. 위 보행 비교와 controller/JAX/payload/bridge/
  snapshot 코드 SHA가 같고, adapter/config 변경은 기본값뿐이다.
  시험 container 종료·GPU2 유휴·기존 pin 보존·로컬 dev 종료를 확인했다.
  [전체 비교·오류·raw SHA](../workbench/evidence/curated/sim/mppi_iterations_20261019.json)를 보존했다.

### 현재 제어기의 100/50 Hz 비교 (2026-10-19)

- `331e754`를 양쪽 공식 `elesim-update`로 반영하고 기존 instance pin을
  보존했다. 원격 `tddwalk`만 최신 release `adb39bfa…`, Sim `merry-cricket`으로
  replace한 뒤 설치 모듈 SHA와 GPU2 UUID를 확인했다. 운영 인스턴스는 유지했다.
- GPU2에서 100→50→50→100 Hz 순서로 비교했다. dt/substeps는 .01/2와 .02/4,
  물리 적분 간격 5 ms·기본 MPPI 50 Hz·4096×2·gain·필터는 같게 뒀다.
  두 1280×720 H264/NVENC loopback 영상과 작은 팔 bend 동작도 함께 실행했다.
  첫 쌍은 8초/20초 전후좌우와 전진·후진 3회 출발/정지 10개씩, 반복 쌍은
  8초 전후좌우 4개씩이다. 매 stop 뒤 stand 복귀를 요구했다.
- **100 Hz는 14/14 통과, 50 Hz는 0/14 통과**다. 50 Hz는 2초 준비를 포함한
  t=3.86–8.18초, 이동 명령 뒤 1.86–6.18초에 fault로 종료했다. 착지 timeout은
  4회, `GPU MPC produced no finite force trajectory`는 10회였다. 50 Hz 짧은 네 방향의
  실패 시점과 물리 자세가 재실행에서도 같았고, 마지막 100 Hz 대조는 다시
  4/4 통과했다. 관측된 공중 지지력은 모든 실행에서 0회이고 영상 decode
  오류도 없었다. 현재 제어기를 무조정으로 50 Hz로 낮추는 변경은 채택하지 않는다.
- 100 Hz의 정상 보행 구간 가중 평균은 **10.38 ms/step**이었다. timer는 제어·
  물리·camera submit이고 팔 명령 쓰기와 10 Hz 실제 pose 검사는 timer 밖이다.
  실패가 일찍 발생한 50 Hz에는 안정 보행 처리율 개선을 주장하지 않는다.
  이전 실험과 다른 측정이므로 11.24→10.38 ms를 특정 패치의 효과로 보지 않는다.
- 접촉 피드백 지연과 tick당 필터 응답 변화의 영향은 아직 분리하지 않았다.
  별도로 재조정한 50 Hz 제어기의 가능성을 배제하는 결과는 아니다. 평지·
  loopback 조건이며 실제 UI/DDS/WAN·SROS2·hardware gate는 포함하지 않는다.
  시험 container 종료·GPU2 유휴·기존 pin 보존 및 설치 기본 .01/2 유지를 확인했다.
  [전체 결과·오류·raw archive SHA](../workbench/evidence/curated/sim/cadence_50hz_20261019.json)를 보존했다.

### 감쇠 입력·몸 좌표변환 묶기 (2026-10-19)

- 지지 다리의 입력 검증·우변 준비·속도 제한을 묶고, 각 다리의
  `np.linalg.lstsq(rcond=1e-3)`는 유지했다. 별도 batched SVD는 특이값 경계
  355개 중 3개에서 답이 달라져 적용하지 않았다. 몸 bridge의 선형·각속도와
  pose 차분 2/4개 벡터는 현재 역회전으로 한 번에 변환한다. controller의
  matrix/Euler 변환도 방금 읽은 Rotation을 공유한다. 매 tick 새 상태를 읽고
  reset·속도 차분 fallback·착지 확인·감쇠 gain·토크 제한·DDS 모터 쓰기를 유지한다.
- 패치 전 원격 GPU2 보행에서 기록한 입력 **843개**, 별도 특이/경계 입력
  **355개**에 실제 패치를 재생해 기존 토크와 byte 단위 일치를 확인했다.
  bridge·matrix/Euler 변환의 별도 **256개** 입력도 정확히 일치했다.
  개발 attachment에서 전체 Sim은 **923 passed / 3 skipped / 1 failed**다.
  실패는 기존 RL 시작 좌표 -.166 m가 하한 -.142 m를 벗어나는 검사다.
  새 회귀의 full-suite 모듈 재로딩 간섭은 검사 코드를 보정해 해소했다.
- 두 프로세스 CycloneDDS 모터 명령 수신, 네 프로세스 topology와 protocol
  **145개**가 통과했다. 이번 DDS 검사는 Sim 명령 mailbox까지이며, 실제
  Genesis 모터 적용의 이전 GPU 검증과 구분한다. 전체 required/extended와
  isolated-release gate, 새 설치본 GPU 보행·UI/WAN/SROS2/hardware는 미검증이다.
- 앞선 원격 CPU replay의 감쇠 **.221865→.196073 ms**, 좌표변환
  **.176030→.129618 ms**를 근거로 적용했다. 별도 계산 절감의 합은 약
  **.0722 ms**이며 새 전체 step 감소량을 측정한 수치가 아니다. 이번 작업은
  source 적용·push 범위이고 설치 release/pin은 교체하지 않는다.
  [동일 출력·측정 조건·검사와 한계](../workbench/evidence/curated/sim/batch_control_math_20261019.json)를 보존했다.

### 제어 쓰기 비용 축소·DDS 모터 경로 회귀 (2026-10-19)

- `4a3be80`은 연속된 다리 DOF에 공개 Genesis control API의 `range` 선택을
  사용한다. 임의 순서·음수·중복 index는 기존 목록을 유지한다. 현재 CPU
  float64 토크는 복사 없는 Tensor view로 넘겨 setter의 NumPy broadcast
  변환을 피한다. 제어 모드·토크는 매 tick 계속 쓰며 위치 제어·fault hold·
  감쇠·한계·정지·reset·metrics를 유지했다. MPPI·보행·하중·bridge 계산은
  변경하지 않았고 기존 함수 AST 변경은 `_step`의 control 호출뿐이다.
- GPU2의 고정 상태에서 기존 목록/NumPy 쓰기는 **.372 ms**, 범위/NumPy는
  **.269 ms**, 범위/원본 float64 Tensor는 **.085 ms/call**이었다. 각 방식은
  반대 순서 두 block에서 warmup 후 총 200회 측정했다. 별도 실제 보행의
  상세 timer에서는 force 쓰기가 **.791→.277 ms/step**, 해당 thread CPU가
  **.751→.262 ms/call**이었다. 고정 상태·상세 timer를 clean loop와 섞지 않는다.
- 팔을 움직이는 1,200 tick에서 기존/새 공개 control 호출의 전체 GPU
  mode·force·position·velocity 배열을 같은 순간 대조했다. 위치 쓰기 336회와
  토크 쓰기 864회가 **byte 단위로 정확히 일치**했다. 매 tick 명령 전달과
  관련 없는 관절의 보존도 함수 회귀에 포함했다. 뒤의 새 전진 실행은 기존
  이미지 및 같은 이미지의 기존 경로 두 실행과 120개 cached 위치·자세·속도·
  fault·stage·MPPI cost가 정확히 같았다. 첫 새 실행은 다른 유효 궤적이었으며
  이전 코드의 카메라 끔 실행과는 정확히 같았다. 실행 간 차이의 원인은
  확정하지 않았고 모든 실행의 궤적이 같다고 주장하지 않는다.
- 두 1280×720 H264/NVENC loopback 영상 조건에서 직전 이미지 2회 평균은
  **11.34 ms**, 새 이미지 2회는 **11.24 ms (-.84%)**였다. 같은 새 이미지에서
  기존 쓰기를 복원한 대조군 2회는 **11.65 ms**, 최적화 2회는 **11.24 ms
  (-3.48%, .406 ms 절감)**였다. 각 기존/새 방식을 번갈아 실행했으며 p95
  범위는 **15.44–16.04 / 15.49–15.91 ms**로 겹친다. 전체 loop 개선은 작고
  p95 개선은 뚜렷하지 않다. **카메라 사용 시 10 ms 예산은 아직 미달**이다.
  카메라 끔은 각 1회 **9.27→8.36 ms**였다.
- 조건은 dt=.01/substeps=2, MPPI 4096×2, 2초 준비·8초 보행·2초 정지다.
  t=4..10초의 600 step을 보고한다. 첫 새 이미지의 219.6초 프로세스 시간에는
  초기 물리/카메라 커널 준비가 포함되며 step timer에서는 제외했다. clean
  loop는 controller·physics·camera submit이고 실제 UI/DDS/WAN, 매 tick
  PNG/JSON/진단 snapshot 및 팔 명령 쓰기는 포함하지 않는다. 별도 상세·팔
  동작/전수 대조 실행은 clean 지연 비교에서 제외한다.
- 실제 두 프로세스 CycloneDDS로 lease를 얻어 서로 다른 팔 4축·Go2 속도·
  그리퍼 명령과 torque-off를 보냈다. 실제 `SimEndpoint`와 `SimRuntime`의
  명령 소비 loop를 거쳐 Genesis의 14개 팔/그리퍼 DOF 적용을 확인했다.
  같은 순간 비교를 위해 physics 직전에만 검사 loop를 중단했다. 팔 값의
  최대 오차는 **1.47e-8**, 다리 위치와 관련 없는 속도는 정확히 같았다.
  첫 진단의 base Euler 좌표 bit-exact 단언은 FK 정규화의 **2.50e-7 rad**
  차이로 실패했다. 재검사는 base RPY만 1e-6 rad를 허용하고 다리/속도 정확
  비교를 유지해 통과했다. 이를 위해 runtime 코드를 바꾸지는 않았다.
- 새 `dds-sim-motor`를 required 검사 목록에 추가했다. 기존 네 프로세스
  DDS topology의 Robot→Sim 측정 자세 동기화·lease 복구와 protocol **145개**도
  통과했다. 직접 관련 검사는 **456 passed / 1 skipped**, Sim 전체+quality는
  **884 passed / 3 skipped / 1 failed**다. 실패는 기존 RL 시작 좌표
  -.166 m가 하한 -.142 m를 벗어나는 `rl/test_start_path.py`이며 수정하지 않았다.
  full required/extended/isolated-release gate와 SROS2/WAN/hardware는 미검증이다.
- push 후 양쪽 공식 `elesim-update`를 완료했다. 기존 instance pin을 유지하고
  원격 `tddwalk`만 release `43b94809…`, Sim image `sha256:22b2a47f…`
  (`merry-zebra`)로 replace했다. 설치 모듈 SHA와 GPU2 UUID를 확인했다.
  전진·후진 .8 m/s, 좌우 .35 m/s, 팔 동작·정지가 통과했고 영상 decode 오류는
  없었다. 원격 진단 container 종료·GPU2 유휴·다른 instance pin 보존과 로컬
  개발 service 종료를 확인했다. [설치·실측·전수 대조·DDS 검사 및 한계](../workbench/evidence/curated/sim/control_write_20261019.json)에
  원시 archive SHA와 첫 진단 실패도 함께 보존했다.

### 몸·팔 상태 조회 통합 (2026-10-19)

- `f050f2d`는 몸·팔·발 접촉의 개별 상태 읽기 11개를 공개 Genesis getter의
  연속 entity 배열 7개와 한 CPU packet 전송으로 묶었다. 전체 link/DOF 범위를
  읽어 매 field의 GPU index 생성·선택을 피하고 필요한 행은 CPU에서 고른다.
  실제 robot은 46 link/32 DOF이며 packet은 3,200 bytes다. 소유한 읽기 전용
  snapshot을 매 제어 tick 새로 만들고 bridge·전체 17-link 하중·접촉 판단이
  공유한다. authored frame·float64 CPU 변환·속도 차분 fallback·reset을 유지한다.
  최소 getter entity와 별도 arm entity는 기존 읽기 경로를 사용한다.
- 원격 GPU2의 고정 상태에서 기존 읽기는 **1.479 ms**, 선택 link를 합쳐 한
  번 전송하는 방식은 **1.094 ms**, 연속 전체 entity 범위를 합쳐 읽는 방식은
  **.547 ms (-63.0%)**였다. 5개 방법의 public getter 값은 모두 정확히 같다.
  각 방법은 반대 순서 두 block, block당 20 warmup 후 80회다. 이 수치는
  physics·solver가 없는 고정 상태 재생이며 전체 step 시간으로 해석하지 않는다.
- source push와 양쪽 공식 `elesim-update`를 완료했다. 기존 pin 보존 후
  원격 `tddwalk`만 release `ccb004d0…`, Sim image `sha256:000cb10a…`
  (`merry-turkey`)로 replace했다. GPU2 UUID와 설치된 controller/bridge/
  payload/snapshot/JAX 파일 SHA를 검사했다. MPPI 4096×2, dt=.01/substeps=2,
  전체 관성·접촉·보행·토크·optimizer 계산은 유지했다.
- 두 1280×720 H264/NVENC loopback 영상 송수신 조건의 clean 전진 평균은
  직전 설치본 2회 **13.93–14.12 ms**, 패치 설치본 2회 **11.86–12.29 ms**다.
  두 평균의 평균은 **14.02→12.08 ms (-13.87%)**, p95 범위는
  **18.44–18.78→15.95–17.54 ms**였다. 준비·정지를 뺀 t=4..10초의 600 step이며
  PNG·JSON 기록과 매 tick 진단 snapshot 직렬화는 timed loop에서 제외했다.
- 동일 패치 이미지에서 snapshot만 끄고 기존 getter 경로를 사용한 대조군은
  **14.53 ms**, 새 경로 반복 평균은 **12.08 ms (-16.86%)**다. 이 대조군과
  첫 새 경로 실행의 120개 10 Hz 위치·자세·속도·fault·stage·MPPI cost는
  **정확히 일치**했다. 다른 실행 전체의 궤적까지 같았다는 뜻은 아니다.
- clean 제어 sample 준비는 평균 **6.115→3.436 ms/step (-43.8%)**였다.
  별도 상세 timer에서 공통 상태 조회 **1.564 ms**, bridge 변환 **.286 ms**,
  팔 하중 계산 **.502 ms/step**으로 나뉜다. 새 snapshot의 직접 Tensor.cpu
  호출 1회는 함수 회귀에서 확인했다. 상세 global Tensor.cpu hook의 1,440회는
  tensor subclass 중첩 dispatch·다른 main-thread 호출을 포함하므로 실제
  GPU copy 수가 아니다. 중첩 구간의 시간도 겹쳐 합산하지 않는다.
- 팔을 작은 두 bend sinusoid로 움직이는 동안 같은 물리 상태에서 raw state,
  bridge q/dq, 전체 하중을 기존 public getter 경로와 각각 **864회 대조해
  모든 값이 정확히 일치**했다. 이 실행은 고의로 읽기를 추가한 shadow 검사라
  clean 지연 비교에서 제외했다. 새 9개 조건(두 clean 전진, 기존 경로 대조,
  카메라 끔, 상세 전진, 팔 동작 대조, 후진, 좌·우)과 baseline 3개가 모두
  8초 보행·2초 정지를 완주했다. 영상 10개 실행은 모두 NVENC·decode 오류 0이다.
- 카메라 끔 평균/p95는 **10.36/13.20→8.54/11.52 ms**였으며 각 1회다.
  카메라를 사용하는 **10 ms 예산은 아직 달성하지 못했다**. 새 clean 영상의
  물리 host 호출 평균은 **3.67 ms/step**, MPPI는 **5.26 ms/solve**, cadence를
  반영하면 **2.83 ms/step**이다. 공유 자원 경쟁과 실행 변동을 포함한 wall
  time이며, 이 항목들을 독립 GPU 연산 시간이나 다음 최적화의 보장 이득으로
  해석하지 않는다. 실제 UI/DDS/SROS2/WAN과 장시간·hardware 보행은 이번 검사가 아니다.
- 새 함수 회귀 red 7개 후 직접 관련 canonical Go2/시작/readback은
  **403 passed / 1 skipped**다. Sim 전체는 **850 passed / 3 skipped / 1 failed**다.
  실패는 기존 RL start waypoint -.166 m가 선형 한계 -.142 m를 벗어나는
  `rl/test_start_path.py::test_the_shipped_path_ends_where_the_wrap_is`다. 변경된
  모듈을 import하지 않는 독립 실행에서도 재현됐고 이 RL 설정은 수정하지 않았다.
  전체 required/extended/release gate는 다시 실행하지 않았다.
- 진단 container 종료·GPU2 유휴·양쪽 나머지 instance pin 보존·로컬 개발
  service 종료를 확인했다. [공개 getter 비교·설치·회귀·보행 및 archive SHA](../workbench/evidence/curated/sim/body_arm_state_20261019.json)에
  원시 증거와 한계를 기록했다.

### MPPI 입력 단일 버퍼 전달 (2026-10-19)

- `573f95d`는 CPU 입력 17개를 6,972-byte float32 배열 하나로 묶어 전달하고
  JIT 안에서 기존 shape·bool로 복원한다. 이전 해와 uint32 난수 상태는 GPU에
  유지하고, reset의 초기 zero mean도 GPU에 준비한다. 매 solve가 소유하는 새
  배열을 사용한다. 보행·비용 함수·샘플·반복 수는 유지했으며 기존 optimizer의
  AST도 정확히 같다. World 원점 보정은 기존 float64 경로를 유지한다.
- 기존 코드의 업로드 횟수 회귀는 **19회/기대 1회**로 실패했다. 패치 후 입력
  복원·메모리 소유·warm shift/fresh/reset·계산 동등성을 포함한 canonical
  Go2/시작/runtime readback 검사는 **391 passed / 1 skipped**다. 설치된 개발
  attachment의 CPU JAX를 사용했고, 동일 persistent service의 기존 이미지를
  재사용했다. 전체 required/extended/release software gate는 다시 실행하지 않았다.
- push 후 양쪽 설치에서 공식 `elesim-update`를 완료하고 기존 instance pin
  보존을 확인했다. 원격의 격리 `tddwalk`만 release `addaf4a8…`, Sim image
  `sha256:4b72a172…` (`merry-monkey`)로 replace했다. 설정·선택 GPU UUID와
  설치된 JAX 모듈 SHA를 확인하고 physical GPU2에서 4096×2를 실행했다.
- 실제 보행 입력을 사용한 연속 12개와 자세·속도·질량·관성·초기화 플래그·
  search mask 등을 바꾼 200개에서 새/기존 GPU kernel의 force trajectory,
  cost, 난수 상태가 **정확히 일치**했다. 고정 입력 solve 재생은 CPU packing과
  결과 대기를 포함해 평균 **6.89→4.83 ms/call**이었다. 이 재생은 Genesis와
  controller 상태 준비를 포함하지 않는다.
- 전체 loop는 직전 설치본으로 두 번, 새 설치본으로 두 번 같은 조건에서
  측정했다. dt=.01/substeps=2, 고정 팔, 2초 준비·8초 .8 m/s 전진·2초 정지,
  1280×720 두 영상의 실제 loopback H264/NVENC 송수신이며 t=4..10초의
  600 step을 보고한다. PNG·JSON 저장과 매 tick 진단 snapshot은 timed loop에
  없다. 평균은 **15.90–16.10→14.09–14.96 ms**, 두 평균의 평균은
  **16.00→14.52 ms (-9.23%)**다. p95는 **21.85–21.89→18.43–19.32 ms**다.
  실제 보행의 MPPI solve는 **7.35–7.49→5.24–5.25 ms/call**이었다.
- 상세 계측은 322 solve에 upload 322회로 **1회/solve**를 확인했다. CPU
  packet 생성 **.091 ms**와 `device_put` **.368 ms/call**이 들었다. 몸 상태
  준비 **2.565 ms/step**, 팔 하중 준비 **2.202 ms/step**은 남아 있으며 팔의
  네 상태 배열 읽기는 그 중 **1.840 ms**다. clean run의 전체 sample 준비는
  **6.18–6.31 ms/step**, 물리 host 호출은 **3.03–3.51 ms/step**이다. 각 값은
  inclusive이고 서로 겹치는 구간을 합산하지 않는다. 다음 대상은 Genesis
  상태 snapshot 공유와 GPU index 준비 재사용이다. **10 ms 예산은 아직 넘는다.**
- 새 설치본의 카메라 끔, 두 clean 영상 반복, 상세 전진, 후진, 좌·우 등
  **7개 조건**이 모두 8초 보행과 정지를 완주했다. 영상 6개 조건과 baseline
  2개 모두 NVENC를 사용했고 decode 오류는 없었다. 새 카메라 끔 평균/p95는
  **11.24/14.40 ms**다. 첫 새 이미지 실행은 준비 포함 225초, 영상 재실행은
  약 55초였으며 이 초기화 시간은 steady-step 통계에서 제외했다. Genesis의
  전체 궤적은 전후 달랐고 단기 자세·fault·정지 gate를 통과했다. 장시간 보행이나
  실제 UI/DDS/SROS2/WAN 검증은 아니며 loopback 수신도 같은 host CPU를 쓴다.
- 진단 container 종료·GPU2 유휴·나머지 instance pin 보존과 로컬 개발 service
  종료를 확인했다. [계측·회귀·설치 및 archive SHA 기록](../workbench/evidence/curated/sim/mppi_input_buffer_20261019.json)에
  원시 증거와 실행 경계를 남겼다.

### 버퍼 패치 전 제어 루프 병목 상세 계측 (2026-10-19)

- 팔 최적화가 설치된 `24007a4`의 controller/JAX/payload 파일 SHA를 매 실행
  확인하고 원격 GPU2에서 관찰만 수행했다. 운영 제어 로직은 수정하지 않았다.
  dt=.01/substeps=2, MPPI 4096×2, 팔 고정, target sphere 제외, 2초 준비·
  8초 .8 m/s 전진·2초 정지를 사용했다. 평균/p95는 정상 보행 t=4..10초의
  600 step이다. 실행 중 PNG·JSON 저장과 매 tick 진단 상태 복사를 제거했다.
- 기준 실행 평균/p95는 카메라 끔 **11.46/15.47 ms**, 두 카메라만
  **13.67/19.09 ms**, 두 영상 H264/NVENC 송수신 **15.59–15.70 /
  21.31–21.63 ms**다. 송수신 기준은 2회 반복했다. 실제 1280×720 두 peer의
  decode 오류는 없었다. 수신 process도 같은 호스트를 쓰는 loopback이며
  DDS/SROS2·실제 UI·WAN은 포함하지 않는다.
- 상세 계측에서 몸 상태 읽기·변환은 **2.52–2.73 ms/step**, 팔 하중 측정은
  **2.18–2.23 ms/step**였다. 그 중 팔의 네 상태 배열 읽기만 약 **1.84 ms**다.
  몸 6회·팔 4회·접촉 1회의 GPU→NumPy 상태 읽기가 매 step 반복된다.
  Pinocchio의 기구학·COM·관성 계산 합계는 약 **.08 ms/step**로 작았다.
- MPPI 1회 약 **7.95–8.25 ms** 안에는 **19회 `device_put`의 2.63–2.76 ms**와
  결과 대기·읽기 **2.51–2.57 ms**가 포함된다. 19개 중 2개는 이미 GPU에 있다.
  실제 host 입력은 **6,969 bytes**였다. 입력 전달과 GPU 연산 시간을 같은
  것으로 해석하지 않는다. 세부 timer의 절대 loop 값은 기준보다 약 6–9% 크다.
- 실제 입력을 고정한 사후 전달 실험에서 평균은 개별 **2.171 ms**, PyTree
  **1.703 ms**, CPU에서 한 배열로 합친 전달 **.344 ms**였다. CPU 합치기와
  GPU 완료 대기를 포함했고, GPU에 있던 mean/key는 유지했다. 복원한 입력
  값은 모두 일치했다. 이 단계에서는 입력 전달 재생까지만 검증했다.
- CUDA trace 20 step의 H2D/DtoH 복사 엔진 동작 합계는 약 **.92 ms**였다.
  작은 복사 자체보다 API 호출·index/tensor 준비·dispatch·동기화 비용이 컸다.
  Python profile에서는 Quadrants kernel 진입이 200 step에 **6,002회**였다.
  GPU trace의 kernel 수와는 다른 값이며, inclusive/비동기 시간은 합하지 않는다.
  카메라·미디어 worker는 CUDA trace 대상 밖이고 추적 overhead도 커서 전체
  GPU 사용률이나 이론적인 최대 가속으로 환산하지 않는다.
- 기준 실행의 Genesis 물리 host 호출도 **3.16–3.64 ms/step**이다. 다음
  구현 후보는 MPPI 입력 패킹과 Genesis 상태 snapshot/index buffer 공유다.
  접촉·착지·토크 제한이나 모델 정밀도를 줄이는 방향은 이번 계측의 우선 대상이 아니다.
- 정상·상세·CPU/CUDA profile **10개 실행**이 완주·정지했다. 첫 CUDA profiler
  준비는 container UID의 passwd 이름 부재로 실패해, 진단에만 USER/private
  cache 경로를 지정한 재실행에서 trace를 얻었다. 제어기 fault는 없었다.
  기록을 줄인 실행들의 loop 안 GC는 최대 **.793 ms**, generation-2는 0회였다.
  이전 진단의 .3초 GC를 일반 운영의 고정 주기 현상으로 일반화하지 않는다.
- 모든 진단 container 종료·GPU2 유휴·기존 세 instance pin 보존을 확인했다.
  [구간별 비용·전달 재생·CUDA 추적 요약](../workbench/evidence/curated/sim/control_cost_20261019.json)에
  원시 archive SHA와 실제 설치 image/revision을 기록했다. 코드 변경이 없어
  전체 software/release gate는 다시 실행하지 않았다.

### 팔 하중 준비 비용 평가·전체 link 배열 합산 (2026-10-18–19)

- 원격 GPU2의 설치된 `10ca3a5` Sim에서 4096×2, dt=.01/substeps=2와 두
  카메라를 유지해 팔 고정/동작을 측정했다. 팔 하중은 평균 **3.58/3.71 ms**,
  이 중 네 배열의 상태 읽기가 **1.68/1.75 ms**였다. 남은 약 1.9 ms에는
  link별 CPU 계산과 계측 overhead가 포함된다. PNG 기록은 제외했다.
- 실제 17개 유질량 link의 자세를 CPU에서 재생하면 원형 scalar 합산은
  **1.28–1.30 ms**, node2/node7에 다섯 node씩 묶은 9-link 근사는
  **.73–.75 ms**였다. 초기 자세의 그룹 관성까지 보존한 근사도 이후 COM
  최대 약 **2.08 mm**, 팔 관성 Frobenius 상대 오차 최대 **2.07%**가 남았다.
  단순 질량 배수만 적용하면 COM 최대 약 **3.07 mm**였다.
- 같은 17개 link를 배열로 합산하고 고정 관성 정보를 재사용한 CPU 후보는
  **.11–.12 ms**이며, 원형과 COM 차이 <3e-12 mm, 관성 상대 차이 <6e-16이었다.
  이는 GPU 읽기를 뺀 CPU 재생 결과다. MPPI 자체에는 이미 총질량·COM·관성만
  전달되므로 열 node가 열 진자로 rollout되는 구조는 아니다. 불필요하게 매번
  반복하던 link별 상수 준비와 scalar 계산을 줄이는 방식을 채택했다.
- 상수 재사용 red는 실제 설치 모듈의 함수 회귀 **1 failed / 10 passed**로
  확인했다. 구현은 live pose·velocity, stale 속도의 COM 차분 fallback,
  mass override, massless/선택 link와 reset 시 상수 갱신을 유지한다.
  후보 소스 하나를 임시 주입한 동일 함수 회귀와 공식 update 후 설치 이미지의
  동일 회귀는 각각 **11 passed**다. 갱신된 canonical 개발 attachment의 Go2·
  시작 검증·runtime readback 회귀는 **382 passed / 1 skipped**다. MPPI 산술
  회귀는 CPU에서 수행하고 실제 GPU/Genesis 시험은 원격 GPU2로 분리했다.
- `24007a4`를 push하고 원격 공식 `elesim-update`를 완료했다. 기존 instance
  pin 세 개가 모두 유지됨을 확인하고 격리 `tddwalk`만 새 release로 replace했다.
  설치 파일 SHA와 GPU2 UUID를 검사한 뒤 팔 고정/동작 전진, 후진, 좌·우
  **5개 조건**과 node2/node7 근사 **2개 조건**이 각각 8초 보행·2초 정지를
  완주했다. 진단 container는 종료됐고 GPU2 compute process는 남지 않았다.
- 계측 wrapper 내부의 per-link timer를 뺀 동일 baseline/candidate 비교에서
  팔 하중 평균은 고정 **3.28→1.94 ms (-40.9%)**, 동작 **3.32→2.07 ms
  (-37.7%)**였다. 명령된 보행 구간 전체 loop 평균은 각각 **15.45→14.47 ms
  (-6.3%)**, **14.68→14.53 ms (-1.0%)**, p95는 **21.49→20.01 ms**와
  **20.87→20.14 ms**다. 각 조건 1회 비교이고, 동작 조건에서는 MPPI·토크
  구간의 변동이 절감 일부를 상쇄했다. 평균/상위 지연 모두 10 ms 예산을 넘는다.
- 같은 두 baseline/candidate의 각 600개 sample에서 몸 위치·자세, 발,
  접촉 gate, base twist와 적용 토크가 정확히 같았다. 두 묶음 근사의 하중
  평균은 고정 **1.93 ms**, 동작 **1.89 ms**로, 전체 정밀 배열 합산보다
  각각 약 **.009/.180 ms**만 짧았다. 전체 loop는 일관되게 빨라지지 않아
  production에는 근사를 넣지 않았다. 실제 Genesis 모델도 그대로다.
- 기존 후보 screen은 7개 모두 통과했지만 오른쪽 실제 속도는 .35 m/s 명령에
  .213 m/s였다. 이 screen이나 짧은 시험을 전체 보행 수용 완료로 해석하지 않는다.
  7개 모두 `late_descent` 공중 정지 비율은 0이며 원형과 비교한 두 조건의
  스윙 오차도 같다. Apex 직후를 포함하는 기존 전체 하강 지표는 8.4–20.9%로
  별도 보존하며, 전체 헛발질 지표가 0이라고 보고하지 않는다.
  실제 두 카메라 1280×720/최대 30 Hz, PNG와 DDS/WebRTC peer 없는 조건이다.
  GC·나머지 상태 읽기·제어 비용 및 실제 peer 미디어 성능은 이번에 변경하지 않았다.
- 로컬 공식 `elesim-update`도 `24007a4`까지 완료했고 기존 UI instance 두
  개의 pin을 보존했다. 원격의 사용자 `test`/`readiness` pin도 유지했다.
  양쪽의 정규 runtime을 이번 평가를 위해 새로 시작하지 않았다. 전체 required/
  extended, isolated release 및 실제 DDS/WebRTC 네트워크 gate는 이 범위에서 미실행이다.
- [비용 구간과 재생 후보 기록](../workbench/evidence/curated/sim/payload_cost_20261018.json)을
  보존한다. 이 비용 평가는 DDS/WebRTC 실제 네트워크 수용시험을 대체하지 않는다.

### 같은 이미지의 미참조 태그 회수 (2026-10-18)

- `up`/update 뒤 image ID가 보호되면 그 ID의 모든 동물 별칭까지 남던 경로를
  수정했다. 이미지 보존과 태그 보존을 분리하고 현재 Compose, 등록 instance의
  exact release tag, 실행·정지 container의 생성 참조만 이름 보호에 사용한다.
  ID로 생성한 container는 태그 하나를 남기며 참조 metadata가 없으면 전부 보존한다.
- 설치 UUID·Engine·project·fingerprint·소유 목록, 외부 별칭/digest와 dev 제외를
  유지한다. 계획 후 container 참조가 달라지거나 삭제 직전 image metadata가
  바뀌면 중단한다. 같은 ID의 미사용 태그를 제거한 뒤 이름 예약도 반환하고,
  exact tag가 사라진 과거 release는 실행 가능한 선택 목록에서 제외한다.
- 누적 재현 **5 failed / 1 passed** → 전체 청소 회귀 통과. canonical dev의
  청소·이름 예약·release·updater·instance runtime/CLI 집중 검사는 **127 passed**다.
  두 실제 Docker 설치를 삭제/파일 쓰기 없이 대조한 예행에서는 로컬 20개,
  원격 31개 태그가 회수 후보였다(원격의 미참조 tools 이미지 1개 포함).
  실행·정지 container 자체의 삭제는 이 자동 이미지/태그 회수 경로에 넣지 않는다.
  전체 required/extended 및 isolated release build/verify는 이 범위에서 미실행이다.

### 보행 전진성·지지 감쇠·착지 예측 TDD (2026-10-18, 설치 기본값 21개 통과)

- 사용자의 before/after 영상 피드백대로 스윙 오차 감소만으로 전체 보행을
  개선으로 평가하지 않는다. Before는 실제 .260 m/s지만 뒤쪽 힙이 앞보다
  평균 6.2 cm 높았고, after는 .125 m/s에 앞쪽이 2.6 cm 높았다.
- 계측·분리 실험을 `edcb904`, `0b14f97`, `f6d09e6`, `bf973c2`로 순차 push하고
  매번 양쪽 공식 update를 마쳤다. 원격 GPU2 격리 `tddwalk`의 설치 revision,
  controller hash와 GPU UUID를 검사한 뒤 실제 Genesis 후보 **30개**를 비교했다.
  dt=.01/substeps=2, MPPI 4096×2/seed42와 물리 모델은 유지했다.
- 뒤쪽 목표만 2 cm 낮추면 기울기는 줄지만 속도가 더 줄었다. 감쇠 전체를
  낮추는 후보는 .5 m/s 이상으로 빨라져도 발 추종과 일부 완주가 악화됐다.
  수직 스윙 보정만 남기는 후보도 채택하지 않았다.
- 지지 중 실제 진행속도의 명령 방향 성분만 0..현재 shaped 목표속도로
  제한하고, 지면 고정발의 관절 기준속도로 변환해 기존 감쇠의 기준 토크를
  더한다. 수직·측면·회전·역방향·과속·정지 오차와 null space는 계속 감쇠한다.
  확인된 지지발에만 허용하고 최종 토크 제한·필터를 통과한다. GRF에는 기존
  착지 하중을 곱하지만, 일정한 motor damping의 기준 토크는 하중으로 줄이지 않는다.
- 착지 대기는 새 liftoff를 계속 막되, 이미 달성한 명령 방향 속도를 매번
  0으로 제동하지 않는다. 횡방향 drift나 아직 달성하지 않은 가속은 요청하지
  않는다. 진행 방향쪽 목표 높이를 유지하는 뒤쪽 1 cm body-reference trim도 적용한다.
  지면 높이·접촉 증거·load ramp·timeout·힘/토크 상한·CPU acados는 유지한다.
- 선택한 prototype은 .8 m/s 명령에서 실제 **.655 m/s**, roll/pitch RMS
  **1.17/1.67도**, 스윙 위치 오차 p95 **12.97 mm**로 완주·정지했다.
  trim 없는 같은 보상도 20초를 완주했다. 이는 연구용 명시적 개입 결과이며
  새 production 기본값의 다방향 수용시험 완료를 뜻하지 않는다.
- 기존 전체 하강 정지 비율은 그대로 보존한다. 빠른 후보의 약 12% 표시는
  모두 apex 직후 progress=.55의 단일 20 ms sample이고, before는 최대
  3개 연속 및 progress=.825까지 남았다. progress>=.70의 `late_descent`
  group을 별도로 추가했으며, 선택 후보의 하강 후반 공중 정지는 0이다.
  기존 지표를 삭제하거나 0으로 보고하지 않는다.
- Production 이식은 missing helper import red부터 진행했다. 첫 전체 회귀는
  **751 passed / 3 failed**로, 세 실패는 이전의 '착지 대기이면 속도 0' 기대값이다.
  bounded 진행·CPU 유지·역방향/과속·접촉 mask를 명시적으로 검증하도록 확장한
  canonical Sim/관련 보행 회귀는 **763 passed / 4 skipped / 1 deselected**다.
  스윙 진단이 지지 참조까지 바꾸는 red도 분리한 같은 전체 회귀는 **764 passed**다.
  반복 보행의 뒤쪽 cycle도 포함하고 정지 구간을 제외하는 분석 red를 수정한
  분석/자세 focused 회귀는 **43 passed**다. 제외 1건은 기존 RL 선형축
  경유점 불일치이며 해당 경로는 변경하지 않았다.
- `029ed33`를 양쪽 공식 update하고 실제 설치 controller 해시를 확인한 기본값
  시험에서 전진/좌/우 6초는 완주했지만 후진은 t=3.52 s에 finite-force
  fault로 정지했다. 따라서 긴 보행/반복 정지는 진행되지 않았고 완료로 보고하지 않는다.
- 후진 FL 착지의 기록된 Jacobian·관절 토크를 분해하면 downward task -19 N이
  몸통 이동에 따른 원시 관절 감쇠의 upward 약 +19 N과 상쇄됐다. 발은 바닥
  높이에 있어도 하중 증거가 부족해 clock이 기다렸고, 기존 지지발이 과신전됐다.
  search에 ground-relative zero-velocity 감쇠 기준을 추가해 base 기여만 뺀다.
  실제 발 움직임·기존 passive 1.1/active .5 감쇠, 60 N task bound와 탐색 속도,
  접촉 증거·timeout·CPU 경로는 유지한다. Gate를 완화해서 넘기지 않는다.
- `test_59`의 **5 failed / 2 passed**로 이 상쇄를 재현했다. 수정 후 탐색 힘
  제한·정지 중 착지·오차 감쇠·CPU·진단 분리를 포함한 canonical 전체는
  **774 passed / 4 skipped / 1 deselected**다. 수정한 설치본의 후진/전체
  실제 보행은 다음 push/양쪽 공식 update 뒤 다시 검증한다.
- `4deb398`도 양쪽 공식 update/정확한 해시/GPU2로 재검증했다. 전진·좌·우는
  6초와 정지를 완주했지만 후진은 t=3.84 s에 다시 fault였다. 다음 지지 확인
  구간에서 GRF만 load ramp를 받는데 일정한 motor damping의 기준 토크까지
  load로 줄여 DC drag를 남기는 불일치를 추가 red 4개로 재현했다.
  Reference에는 load를 곱하지 않고, 실제 확인된 접촉 mask로만 허용한다.
  GRF의 기존 load ramp와 접촉·최종 토크 제한은 유지한다.
- 이 수정 후 동일 canonical 전체는 **777 passed / 4 skipped / 1 deselected**다.
- `734a36d`도 양쪽 공식 update/설치 해시/GPU2 확인 후 실제 10개 시험을 했다.
  전후좌우 6초·정지와 전진/좌/우 20초는 완주했으며, 20초 전진의 실제
  평균 속도는 .798 m/s, roll/pitch RMS는 1.93/1.87도였다. 그러나 후진 20초는
  t=10.02 s, 전진/후진 반복은 각각 정지/재출발 구간에서 실패했다. 7/10 완주이며
  안정성 완료로 보고하지 않는다.
- 후진 FL의 t=9.62 s 기록에서 요청한 적용 GRF 10 N에 비해 감쇠 환산 upward
  약 25.5 N, passive를 포함한 적용 토크의 환산 downward -15.3 N이었다.
  실제 normal은 0 N이고, 무하중이 이어져 확인된 접촉을 다시 잃었다. 이 환산은
  quasi-static actuator 추정이며 새 물리 측정값이 아니다.
- 보정 전 `test_60`은 **3 failed / 4 passed**로, .1 m/s 수직 base 운동에서
  지지 10 N이 감쇠 후 upward 6.15 N이 되는 현상을 재현했다. 확인된 지지발의
  최종 active 토크와 passive damping을 환산해 기존 solved normal 하한에 부족한
  성분만 최대 60 N 범위의 motor bias로 보상한다. 접선 감쇠, 충분한 지지의 감쇠,
  MPC force/load 요청과 contact gate, 최종 토크 상한·필터 및 CPU는 유지한다.
  특이 Jacobian에서는 보상을 만들지 않는다. 관련 focused 회귀는 **51 passed**다.
  동일 canonical 전체는 **790 passed / 4 skipped / 1 deselected**다.
- `b2b21f5`를 push/양쪽 update/해시·GPU2 확인 후 시험했지만, 사후 normal
  보정은 짧은 후진부터 t=3.78 s에 실패했다. 전진·좌·우는 완주했으나 long
  gate는 열리지 않았다. 이 후보의 runtime과 구현 전용 `test_60`은 제거하고
  실패 증거와 해당 commit의 회귀 이력을 보존했다.
- 추가로 `734a36d` 후진의 clock=5.01, `b2b21f5`의 clock=.72에서 다음 지지
  쌍은 이미 fully loaded인데 곧 뗄 이전 쌍의 무하중 때문에 clock 전체가
  정지하는 교착을 확인했다. GPU는 네 발 overlap에서 실제 full-load 대각
  한 쌍이 있으면 경계까지 진행하되, 새 liftoff 경계를 넘을 때는 남을 모든
  발의 실제 확인·완전한 load ramp·새 airborne 증거를 검사한다. 접촉을
  조작하거나 없는 발에 GRF를 주지 않는다. CPU strict gate와 settling은 유지한다.
- `test_61`은 옵션 연결 후 **6 failed / 7 passed**로 교착을 재현했다.
  Search→새 swing에서 예전 start/착지 목표를 재사용하는 추가 red도 재현해,
  해당 전환만 현재 발에서 새 궤적을 latch하고 새 foothold를 계산하도록 했다.
  실제 접촉 이력과 진행 중 swing의 고정 목표는 유지한다.
- 이 handover/새 swing 수정의 canonical 전체는 **793 passed / 4 skipped /
  1 deselected**다. `3615957`도 push/양쪽 공식 update/해시·GPU2 확인 후 실제
  10개 시험을 했다. 전후좌우 6초와 후진/좌/우 20초는 완주했지만, 전진
  20초는 t=5.04 s에 touchdown timeout, 전후진 반복도 후반 cycle에서 실패했다.
  후진 20초 평균 속도는 .687 m/s지만 pitch RMS 8.04도로 자세도 미해결이다.
- 네 발 overlap에서 이전 쌍만 fully loaded여도 clock을 진행시킨 추가 변경은
  원복한다. 착지 ramp 동안 기존 pause를 유지하며, 새 지지 쌍이 fully loaded인
  뒤 떠날 이전 발 때문에 생기는 교착만 해제한다. 다음 liftoff의 실제 지지
  확인과 새 swing 재계획은 유지한다. 이 timing 보존 red 1개를 수정한 전체
  canonical도 **793 passed / 4 skipped / 1 deselected**다. 실제 재검증은 다음
  push/양쪽 공식 update 뒤 진행한다.
- `c0bcb46`도 같은 push/양쪽 update/정확한 해시/GPU2로 실제 10개를 비교했다.
  전후좌우 6초와 20초 **8개는 모두 완주·정지**했다. 전진 20초 실제 .814 m/s,
  roll/pitch RMS 3.10/3.40도, 후진 .739 m/s와 2.32/5.01도다. 하지만 전진
  반복 t=16.24 s, 후진 반복 t=23.22 s에 fault가 남았다. 정지 전부터 기울기가
  컸으므로 '정지 제동만의 문제'로 확정하지 않는다.
- 출발 준비 자세 중 command shaper가 미리 목표속도까지 올라가 실제 걷기
  시작에서 가속 제한을 건너뛰는 현상을 `test_62` **5 failed / 1 passed**로
  재현했다. GPU의 ready→walk에서 held target을 유지하고 현재 속도만 0으로
  되돌린다. 실제 보행부터 단일 가속 제한기를 쓰며 추가 quintic 배율은 CPU에만 유지한다.
- 실제 actuator torque는 처음 .35에서 시작하지만 rollout은 1로 계산하던
  차이를 `test_63` **13 failed / 1 passed**로 재현했다. 내부 `PyMpcInput`의
  별도 `actuation_scales` horizon을 contact-load preview와 곱해 실제 force gain을
  예측한다. 실행에서 다시 곱하지 않는다. gain 상승이 완료될 때까지 네 발 지지를
  유지하며, 같은 gain을 반영해 stance/search 감쇠 참조와 계측도 일치시킨다.
  실제 passive damping과 힘·토크 상한은 유지한다.
- 첫 force/torque 분배에 유효하지 않은 zero history를 섞는 추가 red
  **1 failed / 1 passed**도 재현했다. 첫 GPU 분배만 새 분배로 시작하고 이후
  기존 필터를 유지한다. Search의 startup passive 보정 추가 red도 통과했다.
  이 수정 후 canonical 전체는 **819 passed / 4 skipped / 1 deselected**다.
  새 설치본의 반복 출발·정지 우선 실제 GPU2 검증은 다음 push/양쪽 update 뒤 진행한다.
- `d0eee6c`도 push/양쪽 공식 update와 설치 controller/solver 해시, GPU2를
  확인한 뒤 실제 10개를 돌렸다. 전후진 각 3회 출발·정지, 전후좌우 6초와
  전진/좌/우 20초는 모두 완주·정지했다. **9/10**이며 후진 20초만
  t=15.62 s에 finite-force fault가 남았다. 전진 20초 실제 .810 m/s,
  roll/pitch RMS 2.20/2.71도다. 실패한 후진을 제외하고 완료로 판정하지 않는다.
- 후진 FL 착지 t=14.86 s에서 기존 감쇠의 upward 환산은 약 28.2 N,
  요청된 적용 지지력은 10 N, 실제 접촉 normal은 0 N이었다. 적용 motor
  torque와 passive를 합친 downward 환산은 -17.9 N이다. 이는 기록한
  Jacobian·토크의 quasi-static 분해이며 추가 물리 측정값은 아니다.
- Search는 지면 기준인데 stance로 바뀌면 명령 방향 평행이동만 보상해
  몸통 상하·회전 운동의 감쇠가 새 발을 들어 올리는 차이를 확인했다.
  GPU의 확인된 stance에도 측정한 world foot velocity와 joint velocity로
  동일한 지면 고정발 기준을 적용한다. 실제 발 오차·null space 감쇠와
  접촉 증거·load ramp·최종 토크 상한·필터는 유지하며, 몸통 제동은 MPC가
  계속 요청한다. CPU 경로는 그대로다. 구현 전 물리 토크 red 7개를 재현했다.
  stationary-body fixture 두 곳의 measured foot velocity를 J*qdot과 일치시켜
  기존 기대 토크를 그대로 검증한다. Zero-velocity 요청의 실제 JAX optimizer
  제동도 CPU에서 확인했다. Canonical 전체는 **828 passed / 4 skipped /
  1 deselected**이며 새 설치본 GPU2 재검증은 다음 push/양쪽 update 뒤 진행한다.
- `ca6f1e7`도 push/양쪽 공식 update/해시/GPU2를 확인했지만 실제 결과는
  **3/10**으로 악화됐다. 짧은 전진·좌·우만 완주·정지했고, 후진은 7.22 s,
  전진 반복은 18.24 s, 긴 좌/우도 각각 21.72/10.10 s에 fault였다.
  긴 전진도 23.98 s에 아직 to_stand여서 완료로 세지 않는다. 모든 지지발에
  적용한 지면 기준 보상은 채택하지 않는다.
- `test_65` **7 failed / 1 passed**로 기존 fully loaded 지지발의 자세 감쇠를
  보존해야 하는 조건을 재현했다. 확인된 새 발의 기존 60 ms load ramp
  동안에만 search와 같은 측정 기준을 쓰고, fully loaded 발은 이전 bounded
  progress 기준과 자세/정지 감쇠를 유지하도록 범위를 줄였다. Ramp 종료의
  복귀도 기존 torque filter를 통과한다. 새 시간상수·접촉 완화·force 보정은
  추가하지 않았다. Canonical 전체는 **839 passed / 4 skipped / 1 deselected**다.
  이 범위 축소 후보의 실제 보행 결과는 다음 설치본 검증에서 확인한다.
- `3d6a0ac`도 push/양쪽 update와 설치 해시/GPU2를 확인했지만 **6/10**이었다.
  짧은 전후좌우와 긴 좌/우는 완주·정지했으며 후진 20초는 4.60 s,
  전진 반복 19.88 s, 후진 반복 25.42 s, 전진 20초 23.74 s에 fault였다.
  착지 중에만 적용한 보상도 채택하지 않고 `d0eee6c`의 지지 감쇠로 돌아간다.
- 추가 분해에서 후진 FL t=14.86 s upward 환산 28.2 N의 주성분은
  lateral translation 16.7 N과 rotation 16.2 N이며 root vertical은 3.1 N이다.
  Foot-error 등의 반대 성분도 포함한 합이며, 단순 수직 보상으로 단정하지 않는다.
- 같은 기록에서 calf 명령이 31.995 Nm 상한에 잘리는 구간이 있었지만
  MPPI rollout에는 180 N GRF cap과 friction만 있고 실제 joint torque box가
  없었다. 실제 JAX 계산의 **7 failed / 1 passed**로 startup/load와 bias를
  포함한 raw/final 모터 상한 초과를 재현했다. Seed·모든 sample·weighted
  update를 기존 두 clipping 단계의 affine torque box에 넣고 그 힘을 그대로
  scoring한다. 불가능한 normal floor보다 실제 상한을 우선한다.
- 기존 force history가 새 자세의 상한을 다시 넘기는 추가 red 1개도 재현했다.
  이전 힘을 현재 가능한 범위로 줄인 뒤 기존 계수로 섞는다. 미래 Jacobian과
  auxiliary offset은 짧은 horizon 동안 고정하는 근사이며, 전체 leg dynamics나
  필터 지연을 모두 모델링했다는 뜻은 아니다. 새 모터 상한·접촉 완화는 없다.
  어떤 지지력으로도 motor bias를 만족할 수 없는 추가 red **1 failed /
  1 passed**도 재현해 실제 지지 후보 scoring에서 배제했다. Swing의 offset은
  이 지지 조건에 넣지 않는다. Canonical 전체는 **851 passed / 4 skipped /
  1 deselected**다. 관측에는
  제약 활성 여부와 실제 solver 반환 힘의 raw/final 토크 비율을 기록한다.
  다음 push/양쪽 공식 update 뒤 controller·solver·JAX 해시를 모두 검사하고
  실제 GPU2에서 긴 후진부터 같은 수용시험을 다시 진행한다.
- `2f60449`를 push/양쪽 공식 update한 뒤 controller·solver·JAX의 세 해시와
  GPU2를 검사했다. **전후좌우 6초/20초와 후진 3회 반복은 완주·정지**했다.
  전진 반복은 fault 없이 25.98 s에 아직 to_stand여서 **9/10 완료**다.
  전진/후진 20초는 실제 .782/.785 m/s, roll/pitch RMS 2.47/2.34도와
  2.72/2.36도다. 왼쪽 짧은 보행은 roll RMS 4.12도로 4도 screen을 초과했다.
  Motion screen은 정지 미완료도 포함해 8/10이며 기준 초과를 숨기지 않는다.
  모든 trial의 late_descent 공중 정지는 0이고 기존 전체 하강 지표는 보존한다.
- 모든 trial에서 실제 solver에 모터 제약이 켜졌고 계획 토크 비율은 1 이하다.
  50 Hz solve 사이 100 Hz 제어에서는 후진/전진 반복에 각각 7/3개의 확인된
  stance raw 상한 초과 관측이 남아 최종 보호 clipping은 유지한다. 이는
  aux/gain/filter 전 원시값이며 최종 clipping의 직접 판정은 아니다. 모든
  substep에서 완전히 같은 힘을 예측했다는 뜻은 아니다. 실제 설치 filter는
  force .7 / torque .5이며 dataclass 기본 .35/.25와 구분한다.
- 전진 마지막 stop t=24.72 s에는 실제 vxy .042 m/s인데 새 swing이 시작됐다.
  Settling 진입은 25.54 s, to_stand는 25.76 s였다. 벽시계 support window와
  접촉 대기로 늦어진 실제 gait phase를 동시에 기다리는 정지 race다.
  `test_67` **6 failed / 5 passed**로 재현했다. GPU는 감속/dwell 뒤 새 liftoff를
  먼저 동결하며 기존 swing은 끝내고 실제 네 발 접촉·load·몸통 정지를 확인한
  뒤 위치 제어로 복귀한다. CPU nominal-window 규칙은 유지한다.
- 최소 `__new__` fixture 네 곳에 실제 controller의 backend 필드를 추가해
  기존 기대값을 그대로 검증한 canonical 전체는 **862 passed / 4 skipped /
  1 deselected**다.
- `2cda621`도 양쪽 공식 update와 네 소스 해시/GPU2를 확인하고 10개 실제
  trial을 마쳤다. 전진 반복의 앞 두 정지는 1.42/1.40 s에 stand로 복귀했지만
  세 번째 재출발 t=19.66 s에 finite-force fault였다. 나머지 9개는 완주·정지했다.
  왼쪽 짧은 보행 roll RMS 4.115도도 남아 motion screen은 여전히 8/10이다.
  모든 trial의 pre-step 공중 지지력 tick은 0이다. 해결 완료로 보고하지 않는다.
- 실패 직전 뒤쪽 RL 발은 몸통 뒤 0.48 m에 남았다. 몸통이 .26→.30 m로
  상승하고 명령이 1.2 m/s²로 가속되는데 착지 계획은 현재 낮은 속도·높이를
  외삽했다. GPU liftoff와 미래 재착지에 같은 command-shaper 가속 적분과
  높은 쪽 body-height 도달 검사를 적용하는 후보를 추가했다. Half-support
  보폭에도 기존 60 ms 하중 인계를 포함한다. Swing 중 고정 착지점, 접촉 증거,
  하중 인계, timeout, 토크 상한 및 CPU 경로는 유지한다. `test_68`은 missing
  helper import red부터 시작했다. 기존 전체와 새 수학 회귀는 **881 passed /
  4 skipped / 1 deselected**이며, 실제 Pinocchio sample의 두 예측 경로와
  swing 착지점 고정까지 확장한 focused 회귀는 **20 passed**다. 실제 원격
  개선은 다음 설치 시험에서 확인한다.
- `bf2b36a`를 push/양쪽 update/네 해시/GPU2 확인 후 실제 시험 중, 전진 반복
  3회는 모두 완주·정지했다(실제 .712 m/s, roll/pitch RMS 2.33/5.12도).
  그러나 후진 반복 3회차 t=22.10 s에 실패했다. 정상속도 보폭까지 60 ms분
  늘린 후보는 후진에서 14도 이상 기울어지는 부작용이 있어 그대로 채택하지 않는다.
  도달 검사에만 기존 하중 인계 시간을 남기고 등속 보폭은 원래 값으로 복원했다.
  가속 적분·높이 예측·두 재착지 경로는 유지한다. 추가 red는 **2 failed /
  18 passed**, 수정 뒤 같은 canonical 전체는 **882 passed / 4 skipped /
  1 deselected**다. 이 보폭 복원 후보도 별도로 push/update 후 실제 검증한다.
- `10ca3a5`를 push/양쪽 공식 update한 뒤, GPU2와 네 소스 해시를 다시 검사했다.
  전후좌우 짧은/20초 보행과 전후 각 3회 재출발 **10개 모두 완주·정지**했고
  같은 motion screen도 **10/10**이다. 전진/후진 20초의 실제 평균은
  **.808/.772 m/s**, roll/pitch RMS는 **3.25/3.04도와 2.86/4.33도**다.
  앞 후보에서 남았던 왼쪽 짧은 보행의 roll도 2.42도로 screen을 통과했다.
- 두 실제 카메라를 켠 .2/.4/.8 m/s 보행 **3/3**과 카메라 20초·팔 동작
  20초·카메라+팔 3회 재출발 **3/3**도 완주·정지 및 motion screen을 통과했다.
  영상용 .8 m/s 보행의 실제 평균은 **.814 m/s**, roll/pitch RMS 2.28/3.49도다.
  이 16개 전체에서 pre-step 공중 지지력 tick과 `late_descent` 공중 정지는
  모두 0이다. 기존 전체 하강 지표는 그대로 보존하며 apex 부근 저속 sample까지
  0으로 보고하지 않는다.
- 전진/후진 각 **10회** 추가 재출발도 모두 stand로 복귀했다. 정지 뒤 복귀
  최댓값은 각각 1.66/1.48 s다. 60초 전진과 좌우 40초도 완주·정지와 motion
  screen을 통과했다. 추가 장시간 시험은 **5/5**, 전체 실제 시험은 **21/21**이다.
  60초 전진의 실제 평균은 .804 m/s, roll/pitch RMS는 2.14/2.22도다.
  전체 21개에서 `late_descent` 공중 정지와 pre-step 공중 지지력 tick은 0이다.
  기록된 모든 solver의 motor 제약은 활성이고 계획 토크 비율은 float 오차
  5.5e-8 이내에서 1 이하, 실제 적용 명령도 기록된 URDF 상한 이내다.
  20 ms 진단 sample이 모든 제어/물리 substep의 동일성을 증명하지는 않는다.
  실제 observer 프레임 358개로 정속/슬로모션 영상을 만들고 SHA·ffprobe와
  연속 발 동작 프레임을 확인했다. 재생은 simulation time의 1×/0.5×이며
  프레임 보간이나 자세 합성을 하지 않는다. 두 영상 전체 디코딩도 성공했다.
- [후보별 수치와 원시 bundle 해시](../workbench/evidence/curated/sim/walking_stance_reference_20261017.json)를
  보존한다. 검증한 runtime revision은 `10ca3a5`이며, 이후 기록/문서만 바꾼
  commit은 동일한 네 runtime source hash로 구분한다. 검증 범위는 해당 설치
  기본값의 평지 GO2+팔 시뮬레이션이다.
  DDS/WebRTC 네트워크·실물 Robot 수용시험과 R0–R5 완료를 뜻하지 않는다.

### 스윙 기준 속도 감쇠 TDD (2026-10-17, 추종 개선 확인·속도 미해결)

- 원격 Robot 상태 동기화·선형축 보정·cuRobo 변경 `a7b3962` 위에 보관한
  `test_57`을 복원하고 `a24e487`로 push했다. 기존 보행 runtime은 유지했다.
- `test_57`의 미구현 import 실패부터 재개했다. GPU free swing에서만 motor
  passive 감쇠 중 `leg_kv_damping`을 active 오차 감쇠로 옮기고, 몸통 운동을
  뺀 world foot 기준 속도를 Jacobian으로 변환해 기존 총 감쇠의 기준 토크를
  더한다. 실제 추종 오차와 Jacobian null space는 계속 감쇠한다. URDF 속도
  상한은 공통 비율로 적용해 특이 자세에서 새 null-space 구동을 만들지 않는다.
- Stance/search, CPU acados, 접촉 확인, 공중 지지력 금지, 토크/힘 상한과
  필터는 유지한다. damping setter는 발의 support mode가 바뀔 때만 호출한다.
  원시 진단에 실제 latched swing start와 touchdown depth를 기록한다.
- 첫 canonical Sim/보행 검사: **685 passed / 3 skipped / 1 failed**.
  실패는 원격 선형축 하한 -0.142 m와 기존 RL 경유점 -0.166 m의 불일치
  (`rl/test_start_path.py`)다. 이 보행 후보와 무관하며 경유점을 임의로 바꾸지
  않았다. 특이 자세에서 개별 관절 clipping이 null-space 토크를 만드는
  추가 red를 재현하고 공통 비율 제한으로 수정했다. 해당 RL 1건을 명시적으로
  제외한 canonical Sim/고정발/보행 및 분석기 회귀는 **692 passed / 4 skipped /
  1 deselected**다. skip은 qpOASES, opt-in 실제 camera worker 2개와 미지정
  fixed-foot trace다. 분석기는 정확한 추종·공중 정지·search 제외·미기록 start
  거부 및 CLI 재현을 검사하며, baseline/후보의 실제 원시 기록에서도 이전
  분석 결과를 동일하게 재현했다.
- `9231adf`를 push하고 양쪽 공식 update를 완료했다. 원격 GPU2 격리
  `tddwalk`에서 설치 revision과 controller hash, 단일 GPU2 UUID, dt=.01 /
  substeps=2 / MPPI 4096×2를 확인하고 기존 `a24e487`과 같은 두 카메라
  포함 .2/.4/.8 m/s 시험을 했다. 양쪽 모두 세 시험을 완주·정지했다.
- .8 m/s의 free-swing 위치 오차 p95는 **49.44→5.99 mm**, 하강 중 공중
  정지 sample 비율은 **52.08→0%**(192/188 관측), 착지 지연 중앙값은
  **100→40 ms**다. .2/.4 m/s도 오차 p95 35.29/44.24→7.00/6.76 mm,
  공중 정지 37.0/48.47→0/0%다. 이는 원하는 Vz<-.1 m/s, 실제 Z>.03 m인
  free-swing에서 실제 Vz>=-.03 m/s를 멈칫으로 센 수치다.
- 추가로 카메라 없는 전후좌우 6초·20초, 전진/후진 각 3회 출발·정지까지
  10개를 모두 완주했고 모든 stop이 stand에 도달했다. 후보 실제 Genesis
  13개 시험에서 fault/넘어짐과 공중 지지력 sample은 0이다.
- 전진 속도 추종은 해결되지 않았다. .8 m/s 평균 실제 속도는 .260→.125 m/s,
  12초 전진량은 2.786→1.438 m로 줄었다. pitch RMS는 9.68→5.47도로
  줄었으나 스윙 개선을 전체 보행/속도 개선으로 간주하지 않는다. 다음 분리
  대상은 지지 중 원시 관절 속도 감쇠와 실제 전진 토크다. 확인한 스윙
  추종을 유지하고 지지발의 제동 원인을 계측한다.
- [비교·회귀 수치](../workbench/evidence/curated/sim/walking_swing_tracking_20261017.json)에
  release/hash/GPU와 원시 경로를 보존했다. 계측·분석은 production Genesis와
  카메라 worker를 사용하되 DDS 참여나 실제 Robot 명령을 만들지 않는다.
  PNG 저장 부하가 있어 이 timing을 실시간 성능 gate로 사용하지 않는다.

### 그리퍼 물체 접촉 시 과도한 닫힘 방지 (2026-10-19)

- 이전 동작은 grasp 완료 후 ID 5에 고정 230° Goal Position을 쓰고,
  모터 전류는 표시 및 2.5 A arm-wide fault 감시에만 사용했다. 물체에 닿아도
  위치 목표를 측정 위치로 옮기는 접촉 처리가 없어, 계속 닫힘을 밀다가 모터
  Hardware Error를 유발할 수 있었다.
- Robot native controller에 gripper 전용 stall/contact hold를 추가했다. 닫힘 중
  전류가 800 mA 이상이고 위치가 3 encoder tick 이내로 120 ms 정체되면 Goal
  Position을 그 순간의 측정 tick으로 옮긴다. hard current threshold에 이르면
  즉시 같은 처리를 한다. 반복되는 동일 닫힘 명령은 열림 명령 전까지 hold점을
  덮지 않는다. Position mode는 유지하며 arm ID 1–4의 current guard는 유지한다.
  설정은 `arm.claw_contact_current_ma`, `claw_contact_stall_s`,
  `claw_stall_position_ticks`이고 기본값은 800 mA / 0.12 s / 3 ticks다.
- Robot native/Python ABI를 2→3으로 함께 올려 구형 shared library와의 혼용을
  시작 단계에서 거부한다. Register-backed SDK double로 접촉 시 목표 rebase,
  반복 close 무시, 명시적 open 해제 및 ID 1–4 Torque 유지를 검증했다.
  Robot runtime/config/native motion focused **54 passed**. 실제 Robot native
  controller도 Jetson 임시 경로에서 빌드해 ABI 3을 확인했다.
- Jetson의 현재 실행은 ID 5 Hardware Error를 이미 보고한 latch 상태다. 수정본은
  저장소와 `/tmp/elesim-claw-contact-20261019`의 검증용 build에만 적용했다.
  활성 Robot/library 교체, 재시작, fault clear, Torque ON 또는 실물 명령은
  보내지 않았다. 실제 물체/전류 threshold 보정과 활성 Robot 적용은 남아 있다.

### XL430 그리퍼 부하 감지 단위 수정 (2026-10-19)

- Hardware Error/Shutdown은 모터 내부에 래치되며 사용자가 설명한 대로 장치
  재부팅으로 해제된다. 앞선 read-only 확인 시 ID 5 Hardware Error Status는 0,
  Torque는 OFF였다. Robot 재시작·Torque ON·실물 이동은 수행하지 않았다.
- ID 5는 XL430-W250 (model 1060)이며 주소 126은 Present Current가 아니라
  추정 Present Load다. 기존 native 코드가 이 값을 mA로 환산해 접촉 threshold와
  전류 제한에 사용하던 오류를 바로잡는다. 부하는 0.1% 단위의 추정치이므로
  `claw_contact_load_permille` 기본값 250은 초기 소프트웨어 기준이지 힘 단위의
  교정값은 아니다. 닫힘 중 25%를 50 ms 유지하면 측정 tick에 목표를 고정하고,
  90% 이상은 즉시 고정한다. ID 5는 mA telemetry/전류 제한에서 빠지고 별도
  `claw_load_permille`로 표시된다. ID 1–4 전류 제한은 유지한다.
- Robot config schema 5는 기존 schema 4를 읽고 과거
  `claw_contact_current_ma` 값을 버린 뒤 새 부하 기준 기본값을 적용한다. Native
  ABI는 4로 올려 새 Python 구조와 구형 library의 혼용을 막는다. 접촉 경계·
  telemetry 표시 regression 및 Jetson ARM64 빌드는 아직 검증해야 하며, 현재는
  소스 변경만 준비 중이다.

### 실행 추종 지연 처리 (2026-10-19)

- 사용자 목표 약 (0.1284,-0.0123,0.5009)에서 Plan 성공 후 Execute가
  `measured tracking error exceeds 4 mm / 0.06 rad`로 중단됐다. 당시 로그에는
  축별 measured/commanded 값이 없어 실패 축/모터 원인을 단정할 수 없다.
  기존 실행기는 측정 추종 여부와 무관하게 nominal 시간마다 목표를 전진시켰다.
- 다음 목표와 실측 간격이 hard bound의 75%를 넘으면 waypoint 진행을 보류하고
  마지막 목표를 정상 주기로 재전송한다. 실물에서 4 mm/0.06 rad hard bound 안으로
  추종했으나 전체 경로 timeout이 먼저 발동했다. nominal 3.75초 계획에서 76개 중
  71개 waypoint까지 약 14.5초가 걸렸으므로 timeout을
  `max(20초, 4×duration+8초)`로 늘렸다. 이전 전체 제한은 14.25초였다.
- 실제 Jetson 설정 속도 상한은 Linear 0.20 m/s, Roll 3.5 rad/s, 굽힘 1.0 rad/s인데
  cuRobo 모델은 각각 0.02/0.35/0.10으로 계획해 10배 낮았다. 공통 Robot 기본값을
  실제 설정과 맞추고 계획 모델도 Robot 상한의 100%를 사용한다. 25 ms 이하 간격으로
  경로를 보간하고 삽입된 모든 점에서 관절 한계와 충돌을 다시 검사한다. 실행 추종
  제한은 8 mm/0.12 rad, waypoint 진행 대기 기준은 hard limit의 90%, 대기 timeout은 3초다.
  관절 범위, 충돌 검사, Robot의 로컬 slew/deadman과 최종 정착 1 mm/0.015 rad는
  그대로 유지한다.
- 첫 실제 Torque ON 실행은 계획 성공 후 waypoint 71/76에서 전체 timeout으로
  실패했다. 측정 q는 목표와 Linear 약 1.2 mm, Roll 약 0.001 rad, Bend1 약
  0.003 rad, Bend2 약 0.004 rad 차이였다. 실패 원인은 추종 hard limit가 아니라
  14.25초 전체 timeout이었다.
- 이후 100% 속도 모델과 25 ms 재샘플링, 확장 timeout을 Pilot에 적용하고 같은 XYZ를
  다시 계획·실행했다. 61개 waypoint / nominal 1.5초 경로가 Torque ON 실물에서
  `Measured goal reached`로 완료됐다. 최종 commanded-vs-measured q 오차는
  Linear 0.50 mm, Roll 0.00038 rad, Bend1 0.00436 rad, Bend2 0.00471 rad였다.
  실행 중 최대 관측 오차는 Linear 약 1.21 mm, Roll 약 0.0011 rad, Bend1 약
  0.0071 rad, Bend2 약 0.0057 rad로 새 8 mm/0.12 rad hard limit 안이었다.
  Torque ON 경로 기록은
  `~/.cache/elesim/mac-arm/feedback-paced-test-full-speed-retry.log`에 있다.
- Host trajectory/UI 36 passed와 별도 3-process real-DDS fixture 통과는 수정 전
  검증이다(모의 Robot/Planner). canonical required/extended/release gates는 현재
  generated development wrapper가 없어 실행하지 않았다. 실물 속도는 Robot 설정
  상한 100%로 계획·송신했지만, 독립적인 축별 실제 속도 계측은 아직 하지 않았다.

### Roll 원래 Position 모드로 복구 (2026-10-19, 최신)

- 사용자가 원래 Position(3) 서보잉을 사용했다고 확인했다. 앞선 누적 raw만으로
  Extended(4)가 필요하다고 판단한 변경을 철회했다. 기존 원점 불일치의 원인은
  아직 확정되지 않았으며 아래 다회전 보정 기록은 변경 이력일 뿐 현재 설정이 아니다.
- Native controller/Python binding/runtime의 다회전 전용 변경을 함께 제거하고
  ABI 2로 복구했다. `roll_zero_raw` 설정과 validator/전용 테스트도 제거했다.
  Linear 259° 및 관련 모델/Ghost/속도·그리퍼/계획 수정은 그대로 유지했다.
- 사용자 Torque OFF 확인 후 ID2 mode 4→3을 썼다. 실제 raw 6139→2043,
  Homing Offset 0→0, Torque 0→0을 읽어 확인했다. offset/goal/torque-on은
  쓰지 않았다. 설치 Robot 설정에서 `roll_zero_raw:6140`을 제거했다.
- Host runtime/config/native-double 50 passed; Jetson Linux native safety/motion
  9 passed 및 ABI 2 shared library build 통과. Robot Python wheel/library를 함께
  적용했다. Backup과 mode readback은 `~/.cache/elesim/position-restore-20261019`.
  Robot PID 2364536 (`robot-position-restored.log`), fresh connected/Torque OFF/
  safety fault 없음 확인. 실제 torque-on/motion 또는 물리 원점 정합 수용 시험은 하지 않았다.

### Ghost 경로 미리보기 및 Linear 상한 확대 (2026-10-19)

- Plan ready snapshot에 최대 96개의 timed q4 visual samples와 계획 당시 base
  transform을 제공한다. UI는 Sim session의 additive v6/schema1 `set_arm_ghost`
  command로 전달하며 phase 변경/실행/취소/연결 해제 시 지운다. 모델 메시를
  alpha 1/3 초록 debug geometry로 렌더링하므로 physics/collision/Robot 명령이 없다.
  Observer/native viewer에만 표시하고 hand-eye RGB-D에서 제외한다. wall time
  15 Hz로 paused 상태에서도 반복 재생하며 최대 120초 뒤 숨긴다.
- Ghost focused/protocol/operator **132 passed**, 별도 3-process trajectory DDS
  fixture 및 Ghost command를 포함한 4-process real-DDS topology smoke 통과.
  Real Genesis render: visible 1971 / moved 3124 green pixels, hand-eye 0 / hidden 0;
  실제 alpha BLEND 확인. 모든 arm link FK를 Pilot과 비교했다.
- 작업 중 사용자 현재 위치를 새 Linear 상한으로 지정했다. 실측 raw 2946 /
  258.989°에서 259°로 설정했다. 공통 scale은 1.5°/mm를 유지하며 calibration 및
  operating endpoint를 259° / q=-0.172666667 m로 함께 확장했다. Home/start는
  213°를 유지한다. 모든 역할 기본값, ZED/D435 URDF/bundle, IK/cuRobo metadata,
  RL effective bounds를 재생성했다. 모델/보정 focused **93 passed**;
  최종 Ghost/trajectory/UI regression **72 passed**.
- macOS /var→/private/var alias 때문에 임시 IK 모델 빌드의 asset 상대경로가
  어긋나는 문제도 canonical temporary root로 수정하고 재생성/테스트했다.
- 사용자 Torque OFF 확인 후 Jetson protocol/Robot wheels와 설정/IK/cuRobo 모델,
  Pilot preview module을 적용했다. Robot PID 2358844 (`robot-linear259.log`),
  Pilot PID 2358883 (`pilot-ghost-linear259.log`). Mac Sim PID 82490 / UI PID 82491
  (`sim-ghost-linear259.log`, `ui-ghost-linear259.log`). 목표 XYZ/방향 보존.
  Fresh readback: Linear 258.989°, q=-0.172659341 m, active max/limit 259°,
  Torque OFF, safety fault 없음, can_plan=true. 양쪽 WebRTC stream 1280x720 decode.
  새 set_arm_ghost clear/display command 수신 확인. 실제 Plan-ready Ghost 자동
  재생의 live operator 수용 확인은 남아 있으며 실제 이동/Torque ON은 보내지 않았다.
- Backup/staging: Jetson `~/.cache/elesim/linear259-20261019/before-apply`,
  `~/.cache/elesim/curobo-integration/arm_trajectory.before-ghost.py`.
  Canonical required/extended/release gates는 generated dev wrapper 부재로 미실행.
  변경된 endpoint의 새 CUDA full-plan/hardware trajectory 수용 시험은 미실행이다.

### Roll 실측 원점 보정 (2026-10-19)

- 사용자가 물리 원점 raw 6144를 확인했다. 기존 0..4095 clamp와 single-turn
  목표 생성이 이 기준을 보존하지 못했다. Robot의 optional `arm.roll_zero_raw`로
  읽기와 목표 생성을 함께 보정하며 signed multi-turn/4096 counts를 보존한다.
  기존 설치 기본값은 None; wire/model/기구 감속비는 바꾸지 않는다. Native ABI 3.
- Calibrated Roll은 ID 2의 mode 4를 요구하고 Torque ON 때 mode를 덮어쓰지 않는다.
  mode 3 또는 보정 범위 밖의 reset 위치에서는 Torque ON을 차단한다. 모터
  전원/재부팅 뒤에는 물리 원점을 다시 확인해야 하며 자동 회전수 복구는 없다.
- 사용자 지지·Torque OFF·물리 원점 확인 후 Jetson의 기존 stage를 백업했다.
  ID 2 mode 3→4 전환 전후 raw는 모두 6126, Homing Offset은 모두 0이었다.
  해당 확인 자세의 6126을 설치 설정에 저장하고 ABI 3 library/Robot wheel을
  적용했다. Robot PID 2282915; `robot-roll-calibration.log`. Live raw 6125에서
  canonical Roll 0.00076699 rad (약 0.044°), Torque OFF, safety fault 없음.
  Torque ON/Home/실제 이동은 보내지 않았다. Mac UI/Sim에는 기존 측정 q 경로로 전달된다.
- 이후 사용자가 다시 지정한 정확한 원점 자세에서 raw 6140이 연속 안정적으로
  관측되어 설치 보정을 6126→6140으로 갱신했다. 현재 Robot PID 2284470,
  `robot-roll-zero-6140.log`; 앞선 6126은 초기 측정 이력이다. Torque OFF로
  재시작했으며 이동/Torque ON 명령은 보내지 않았다.
- Host focused 60 passed (native SDK double/config/runtime). Jetson real-SDK
  build 및 native safety/motion suite 12 passed. Mac real Linux SDK build suite는
  플랫폼 불일치로 5 setup errors였고 Jetson에서 해당 suite가 통과했다.
  Canonical Docker required/extended/release gates는 generated dev wrapper 부재로
  실행하지 않았다. 실제 Torque ON/trajectory/재전원 후 보정은 미검증이다.

### Plan 시작 자세 경계 오차 (2026-10-19)

- Linear UI 213에서 실측 213.10~213.19°가 수신되어 q=-0.142066~-0.142125 m로
  모델 하한 -0.142 m를 약 0.066~0.125 mm 벗어났다. `solve_position`의 seed
  검사에서 일반적인 q operating-limit 오류로 실패했으며, 목표 XYZ의 도달성
  판정 결과가 아니었다.
- 실제 telemetry/실행 시작 검사를 보존하고 planning seed에만 작은 경계 투영을
  적용한다: Linear 0.20 mm, rotary 0.001 rad 이내. 목표/경로/충돌/모델 한계는
  그대로 엄격히 검사하며, 허용오차보다 큰 시작 오차는 축·값·한계·초과량으로
  preflight에서 거부한다. Host trajectory suite 26 passed.
- 사용자 Torque OFF 확인 후 수정본을 Jetson Pilot에 적용했다(PID 2295007,
  `pilot-boundary-fix.log`). 입력 XYZ/방향을 보존·복원했다. Pilot 재시작 중
  Robot이 old/new Pilot boot 중복 보호로 종료돼 Robot도 OFF로 재시작했다
  (PID 2295737, `robot-after-pilot-boundary.log`). 중복 보호는 변경하지 않았다.
  실측 경계 샘플의 seed 투영과 1 mm 초과 거부를 설치 모듈로 확인했다.
  별도 3-process real-DDS fixture도 통과했다(모의 Robot/Planner, 실제 이동 없음).
  Ghost 미리보기 작업은 이 문제 해결을 위해 보류됐으며 구현 변경은 아직 없다.
- 이후 사용자 Plan/Execute 로그에서 계획 생성과 실행 시작 후 새 lease 발급을
  확인했다. 이전 `motion command rejected: no_active_lease`가 reply 상태에 남아
  다음 Plan을 막았다. `send_current_target_meta`만 전송해 실제 Robot의 정상 ACK를
  확인한 뒤 can_plan=true, blocked_reason 비어 있음으로 복구했다(이동/torque 명령 없음).
  당시 사용자 Torque ON이므로 추가 재시작은 하지 않았다.
- Source `RemoteState.target_changed`는 새 target grant 때 같은 endpoint라도
  이전 no-active-target/lease 오류만 해제하도록 보완했다. 다른 오류와 telemetry의
  비해제 정책은 유지한다. RemoteState+trajectory 32 passed. 이 예방 수정은
  사용자 재시작 요청 후 실측 Torque OFF를 확인해 Pilot에 적용했다.
  이전 heartbeat 만료를 5초 기다린 뒤 시작했다(PID 2305093,
  `pilot-lease-recovery.log`). 목표 XYZ/방향을 보존·복원했으며 실제 이동은 보내지 않았다.


### Plan 상태 표시 보완 (2026-10-19)

- Robot 미연결로 Plan preflight가 거부돼도 idle로 남고 UI callback 오류가
  사라지던 표시를 보완했다. 실패 phase/reason을 snapshot에 보존하고
  can_plan/blocked_reason, CUDA/model 로딩·IK·경로 계획·검증 stage와
  planning elapsed time을 표시한다. 요청 전송/ack 대기/timeout·오류는
  UI 패널에 유지하며 실행/cancel에도 같은 callback 표시를 사용한다.
- Host focused **35 passed**. 기존 목표 XYZ를 보존한 채 Jetson Pilot과
  Mac UI를 재시작해 적용했다. Robot을 시작하거나 실제 이동을 보내지 않는다.
  별도 3-process real-DDS fixture에서도 단계 표시·계획·실행·completion을
  통과했다. Live snapshot에서 Robot 미연결 blocked_reason/can_plan=false와
  보존된 목표 (0.3, 0, 0.3) m를 확인했다.
  이후 사용자 요청으로 기존 Jetson arm-only Robot stage를 시작하고 연결과
  실측 telemetry, Torque OFF를 확인했다. Torque ON/이동은 보내지 않았다.
  시작 시 Linear motor 약 257°가 operating cap 213°를 초과했고,
  Roll raw position 6091도 관측됐다. 실제 위치/멀티턴 해석 확인 전에는
  현재 측정값에서의 경로 실행을 검증한 것으로 간주하지 않는다.

### IK 목표 표시 복구 (2026-10-19)

- UI Target XYZ/방향을 `set_ik_target` display-only simulation command로
  전달한다. 실제 motion target이 Robot이어도 Sim session으로 표시하며,
  재연결 시 최신 목표를 다시 전송한다. Additive v6 command / simulation
  schema v1 유지. q, collision 물체, 실제 Robot 제어는 변경하지 않는다.
- 기존 IK 마커 그리기 호출의 소실과 async camera snapshot 누락을 복구했다.
  초록 구/방향 화살표를 native viewer와 Observer 영상에 표시하며,
  hand-eye RGB-D에는 debug geometry를 포함하지 않는다. 표시 snapshot은
  pre-step camera publish 전에 갱신해 paused 상태의 목표 변경도 반영한다.
- Host focused partial **167 passed, 2 skipped**. 실제 네 프로세스 CycloneDDS
  smoke에서 새 command 전달과 기존 lease/미디어 signaling/mirror를 통과했다.
  실제 Genesis render probe에서 Observer green pixels 33, hand-eye 0,
  hidden 0을 확인했다. 재현은 `workbench/tests/system/smoke_ik_marker.py`.
  증거는 `workbench/evidence/curated/planning/ik_target_visualization_20261019.json`.
- Mac Sim/UI에 적용 후 새 command 수신과 두 1280×720 WebRTC stream decode를
  확인했다. 재개 시 Jetson Pilot/Robot은 꺼져 있어 Pilot만 복구했다.
  Robot은 시작하지 않았고 실제 이동 명령도 보내지 않았다.
- 전체 UI/Sim/protocol partial: 875 passed, 18 skipped, 1 failure는 기존
  RL start waypoint -0.166 m가 operating lower bound -0.142 m 밖인 회귀다
  (`rl/test_start_path.py`); 해당 RL 소스는 이 표시 수정에서 변경하지 않았다.
  Canonical required/extended/release Docker gate는 generated dev wrapper
  부재로 미실행이다.

### cuRobo XYZ operator 경로 실행 (2026-10-16)

- UI Target XYZ에서 명시적인 `Plan XYZ` / `Execute path` / `Cancel path`를
  추가했다. Pilot worker가 CUDA position IK와 joint planning을 소유하며,
  기존 lease-bound canonical target.q만 전송한다. Additive protocol-v6
  operator allowlist이며 Robot wire type / protocol major 변경은 없다.
- Robot-only, identity/v1 correction, open-claw, stationary-base 경로다.
  Exact boot/lease, advancing measured stamp, fresh start, Target XYZ,
  model/world digest, 120 s plan lifetime, single consumption을 검사한다.
  Feedback 350 ms / tracking 4 mm·0.06 rad / lateness 100 ms 초과 시
  취소하고 exact lease release로 Robot local safe hold를 요청한다.
  Torque-on/claw/GO2 명령은 생성하지 않는다. 실제 물리 clearance와
  stop timing은 아직 검증하지 않았다.
- Host partial: Pilot(Replay 제외)/UI/protocol **682 passed + 8 subtests**,
  추가 UI 버튼 분리/disable 검사 **5 passed**. 별도 3-process 실제 CycloneDDS
  fixture probe에서 5 mm / 11 point 전달·실측 completion, 동시 Home 거부,
  plan replay 거부가 통과했다. 실제 Orin CUDA worker에서도 5 mm XYZ 경로
  31 points / 1.5 s를 145.0 s에 계산하고 FK 오차 1.03e-6 m를 확인했다.
  이 probe는 fixture feedback이며 DDS / physical motion을 생성하지 않았다.
  증거는 `workbench/evidence/curated/planning/curobo_xyz_worker_20261016.json`.
- Jetson cache venv에 새 Pilot/protocol wheel과 arm-only source/config를
  준비했다. Mac Sim과 같은 원점이 되도록 세션 Pilot의 use_go2=false를
  명시한다. 사용자 Torque OFF 확인과 fresh telemetry 재확인 후 Mac Pilot을
  중지하고 Jetson cache Pilot을 시작했다. Robot은 재시작하지 않았다.
  Mac UI 재시작 후 DDS/operator status, fresh measured FK, Torque OFF와
  두 WebRTC stream decode를 확인했다. 실제 이동·Torque ON·Home은 보내지
  않았으며 physical path execution / clearance / stop deadline는 미검증이다.
- Canonical required/extended/release Docker gates는 generated dev wrapper가
  없어 실행하지 않았다. 동적 Genesis scene 전달은 별도 미구현이다.

### 선택적 Jetson cuRobo 연동 (2026-10-16)

- Pilot에 선택 가능한 position IK backend와 timed joint trajectory API /
  `elesim-plan` offline CLI를 추가했다. 기본 IK는 native이며 live process를
  전환하거나 실제 명령을 전송하지 않았다. DDS/Robot authority는 변경하지 않는다.
- 기존 arm collision mesh와 IK 모델에서 휴대 가능한 4축 mimic URDF,
  911개 collision sphere와 source/artifact hash를 생성해 Pilot release에
  포함한다. 원본 mesh의 전체 독립 관절 범위에 대한 +X interval bound가
  양수인 22개 링크 쌍만 추가 self-collision 제외한다. 연결·rigid subassembly
  기본 제외 정책은 별도이며 실제 물리 clearance를 보증하지 않는다.
- FK context 생성 시 fixed/prismatic neutral child origin을 connector anchor로
  잘못 대체하던 경로와 parent-frame 변환을 수정했다. 그리퍼·카메라 fixed
  branch의 최대 약 4 cm 차이를 없애고 IK 모델을 재생성했다. 기존 active
  grasp FK는 유지하며 모든 링크 위치·회전을 URDF와 비교한다.
- Orin / R36.5 / CUDA 12.6 / PyTorch 2.11 / NumPy 1.26.4의 별도 cache
  venv에 `cuRobo 0.7.8+elesim.1`을 빌드·설치했다. Source hash 검증을 거쳐
  pair-index capacity / CUDA loop counter / warp block 크기를 보정한다.
  원래 serial fallback의 stale collision 결과·gradient와 넓은 loop counter
  overflow를 피하면서 모든 collision pair를 유지한다. Runtime은 stock
  버전과 serial fallback을 거부한다. 기존 Robot stage/system 환경은 유지한다.
- 실제 GPU probe: FK/회전/911개 collision sphere를 33개 자세에서 비교했다.
  최대 위치 오차 6.92e-7 m, sphere 오차 6.66e-7 m, 회전 오차 2.49e-6 rad.
  26개 자세가 collision-free였고, 10 mm 이동 경로 93 points / 0.05 s,
  position IK, enclosing obstacle 거부 및 collision→free 복귀를 통과했다.
  증거는 `workbench/evidence/curated/planning/curobo_jetson_20261016.json`.
- 이 모델의 초기화 57.5 s, 첫/반복 경로 계획 30.8/10.6 s,
  첫/반복 IK 56.2/2.10 s다. 실시간 성능은 충족하지 않으며 기본값을 바꾸지
  않았다. Sphere fit / collision cost와 planner 설정 최적화는 후속 과제다.
- Conda host partial: model/Pilot(Replay 제외)/release 및 patch-tool 검사
  **521 passed + 3 subtests**. 기존 Replay fixture가 없어 해당 디렉터리는
  제외했다. canonical required/extended/release Docker gates는 generated
  development wrapper 부재로 실행하지 않았다.
- live Genesis scene 전송, Home/slider/workflow의 자동 trajectory 실행,
  접촉 허용, GO2 whole-body planning은 아직 연결하지 않았다. 현재 모델은
  plate를 포함한 arm/open-claw 정적 geometry다. 사용·검증 절차는 `curobo.md`.

### Linear 보정의 저장소·학습 공통 적용 (2026-10-16)

- 장비별 임시 설정에 있던 213° 운용 제한을 protocol/Pilot/Sim/Robot의
  schema와 배포 기본 YAML에 반영했다. 환산 기준 250° = 166.667 mm는
  유지하므로 213° 목표는 142 mm 이동이다. 시작 자세도 213°로 맞췄다.
- 두 카메라 assembly의 URDF/blueprint와 bundle digest, Pilot IK 모델을
  소스에서 재생성했다. IK 재생성은 현재 zed-mini asset의 camera transform
  metadata도 갱신한다. RL은 전체 환산 범위 대신 설정의 effective q bounds를
  읽으므로 기본 범위는 [-0.142, 0] m이고 사용자 제한도 따라간다.
- 모델의 base/housing 대향면 간격은 182 → 40 mm다. 실측 0 위치는
  183 mm였으며 새 끝점의 실측과 마찰·하중·구동 응답 검증은 남아 있다.
  기존 설치의 명시적 설정과 학습 artifact는 자동 변경되지 않는다.
  재배포 시 설정을 병합하고 모델/policy interface를 재생성해야 한다.
- 기존 Conda elesim-ros에서 calibration/model-bundle/Robot config 45 passed,
  protocol 및 Sim/Pilot 설정·mapping 183 passed + 8 subtests,
  RL limits/deploy/overrides 38 passed. canonical required/extended/release
  gates는 생성된 development wrapper가 없어 실행하지 않았다.
  이번 저장소 반영에서는 실물 명령·프로세스 재시작을 수행하지 않았다.

### Home 그리퍼 오동작 / 실제 팔 저속 제어 (2026-10-16)

- Home/슬라이더가 기존 `claw_closed` 요청을 부수적으로 재전송하던 경로를
  제거했다. Robot의 boolean 변환도 논리 0/180°에서 물리 열림 340° / 닫힘
  230°로 수정했다. 종전에는 두 값 모두 드라이버의 230° 하한으로 clamp되어
  UI가 열린 상태여도 실제 그리퍼가 닫혔다. Robot motor 5 측정 위치의
  `claw_closed`를 Pilot snapshot, UI 표시와 Sim mirror까지 전달한다.
- C++ `command_q`의 즉시 최종 목표 쓰기를 없애고 모든 목표/보정 결과를
  motor-space slew limiter로 보낸다. 기본 canonical q 상한은 직선
  0.02 m/s, Roll 0.35 rad/s, 굽힘 0.10 rad/s다. Firmware profile velocity도
  환산 상한 이하로 제한하며, 긴 loop stall은 20 ms 이상의 목표 점프로
  보상하지 않는다. Torque-on 전 fresh 측정값을 모든 모터의 hold goal에
  preload하고 torque-off/hold는 대기 목표를 버린다. Native ABI 2 mismatch는
  시작을 거부한다. Installer/release 필수 파일에 `motion_limits.h`를 추가했다.
- Sim은 측정 자세를 추가 rate limiting 없이 표시한다. 일반 Sim motion
  command는 기존 limiter를 유지한다.
- macOS focused 회귀 265 passed + 5 subtests, 추가 preset 회귀 포함 focused
  10 passed. 확장 protocol/Pilot/UI/Sim 및 관련 Robot 검사: **1382 passed,
  18 skipped, 2 failed + 8 subtests**. 실패 두 개는 기존 replay fixture
  `problematic_grasp.log`/`healthy_grasp.log`가 checkout에 없기 때문이다.
  Jetson에서는 실제 native library 빌드와 관련 Robot 테스트 **56 passed**;
  register-backed SDK double로 goal slew, stall cap, hold-before-torque,
  torque 재인가 시 stale target 제거를 검증했다. 실제 4프로세스 DDS smoke도 통과했다.
- 사용자가 토크 OFF와 팔 지지를 확인한 뒤 별도 Jetson stage와 Mac 세
  프로세스를 갱신·재시작했다. 실제 motor 5 약 230°가 `claw_closed=true`로
  전달되는 것을 확인했다. Agent는 Home/이동/torque-on을 실행하지 않았다.
  실제 부하에서의 속도·gripper 끝점·stop deadline 측정과 canonical
  required/extended/release gates는 미실행이다. R0–R5 완료를 뜻하지 않는다.
- 이후 사용자 요청으로 Jetson arm stage의 목표 변화율 상한만 10배
  (0.20 m/s, 3.5 rad/s, 1.0 rad/s)로 조정했다. 모터 profile 제한은 유지하여
  실제 속도 10배를 보장하지 않는다. 이 값은 저장소 밖의 해당 실행 설정이며
  위 소스 기본값은 보수적인 저속 설정을 유지한다.

### Robot → Sim 측정 자세 동기화 복구 (2026-10-16)

- 기존 host-state 표시 경로를 직접 DDS telemetry로 복구했다. Sim의
  `runtime.mirror_robot_id`는 기본 비활성이고, 명시된 Robot의 exact live
  boot·sequence·measured q만 받아 latest-value 자세를 갱신한다. Sim motion
  lease가 우선하며, telemetry 손실 시 마지막 자세를 유지한다.
- protocol major 6, ROSIDL 및 telemetry payload 형식은 유지한다. opt-in
  capability와 telemetry 수신 role을 확장하고 SROS2 Robot → Sim publish
  정책을 추가했다. 수신 boot는 기존 carrier에서 로컬 Envelope metadata로
  보존하며 payload가 identity를 주장할 수 없다. 표시 경로는 Robot 명령이나
  torque/lease를 만들지 않고, viewer 유실이 Robot motion lease를 해제하지 않는다.
- macOS `elesim-ros`의 focused protocol/Sim/Robot lifecycle/security policy
  회귀: **213 passed + 5 subtests**. 실제 CycloneDDS 별도 4프로세스 topology
  smoke는 Robot → Sim measured-pose 반영을 포함해 통과했다. canonical dev
  wrapper는 현재 설치에서 제공되지 않아 Docker required/extended 및
  isolated release gates는 실행하지 않았다.
- 확장 native host 검사: 908 passed, 18 skipped, 8 failed, 5 errors.
  실패는 macOS에 없는 Linux SOCK_SEQPACKET/SO_PEERCRED와 Linux 전용
  native arm builder 경로다. 이를 canonical 통과로 간주하지 않는다.
- 사용자 확인 후 토크 OFF에서 별도 Jetson arm staging 환경을 갱신했다.
  Mac Sim에서 실제 `robot-arm` telemetry의 measured-pose 수신·반영 로그와
  장면/두 WebRTC track 준비를 확인했다. 기존 설치/systemd는 변경하지 않았다.
  실제 팔 동작을 보내며 시각적 추종·지연·정지 시간을 측정하는 gate와
  새 SROS2 enforce 정책의 live 검증은 미실행이다. R0–R5 완료를 의미하지 않는다.

### 접촉력–관절 토크 역동역학 TDD (2026-10-16, 후보 미채택)

- 감쇠 제거 후보 `c90de7c`/`0623a58`는 .8 m/s 실보행 실패로 원복했다.
  원복 `d55683e`를 push하고 양쪽 공식 update를 완료했다. 사용자 `test`
  instance의 release pin이나 lifecycle은 변경하지 않았다.
- 접지발 가속도 및 unactuated base 방정식과 실제 swing 토크를 함께 푸는
  18 DOF 역동역학을 GPU stance 경로에 연결했다. 관절 중력·관성, motor
  armature=.01, passive 감쇠를 포함하며 접촉 가속도 목표는 고정 지면의 0이다.
  GRF/하중 램프/토크 제한·필터와 기존 접촉 안전 기준은 유지한다.
- 별도 Pinocchio model/data를 사용하며 팔의 네 배치 조회 snapshot을 그대로
  재사용한다. 현재 자세의 강체 payload 근사로, 내부 팔 가속도와 Coulomb
  friction의 정확한 보상은 포함하지 않는다. 시작 구간은 기존 ready
  assistance에서 점진적으로 전환한다.
- 수치 TDD는 synthetic forward contact solve와의 힘 왕복, 실제 GO2 RNEA,
  세계 좌표 foot 가속도의 유한차분, payload 좌표 변환/누적 방지, actuator
  제한과 swing 유지 등을 검사한다. 공식 개발 attachment에서 신규 21개를
  포함한 전체 Sim/관련 보행 회귀는 **674 passed / 4 skipped**다. skip은
  qpOASES, opt-in 실제 camera worker 2개, 미지정 fixed-foot trace다.
- MPPI 4096×2, 설치 물리 기본값 및 UI 목표속도 .8/.35/.8은 유지한다.
  후보를 push/양쪽 update한 뒤 격리 `tddwalk`/원격 GPU2에서 카메라 포함
  .2/.4/.8 m/s 동일 시험을 진행한다. 새 후보의 실제 보행 효과는 아직
  검증하지 않았으며 수치 회귀를 보행 개선으로 간주하지 않는다.
- `af22bc6`를 push/양쪽 update한 뒤 GPU2/두 카메라/설치 기본값으로
  동일 3속도 시험을 했다. .2/.4 m/s는 +1.367/+2.754 m 후 stand에
  도달했으나, .8 m/s는 t=4.36 s/+0.451 m, max roll=.772 rad에서
  finite-force fault로 실패했다. 공중 지지력 샘플은 세 경우 모두 0이다.
  .2 m/s의 평균 실제 속도는 .119 m/s, pitch RMS=1.57도이나 roll RMS는
  2.94도로 늘었다. 전체 개선으로 채택하지 않는다. 역동역학 추가 시간은
  세 경우 median .59~.61 ms/p95 .82~.85 ms다(PNG 부하 포함, 성능 gate 아님).
- 실패 후 실제 지면 가속도 0으로 독립 순동역학을 풀자, 새로 넣은
  `-20*v_foot` 접촉 가속도 servo가 요청 GRF와의 일치를 깨는 회귀를
  재현했다(2 failed). 이 servo를 제거한 같은 canonical 회귀는
  **675 passed / 4 skipped**다. 새 후보의 실제 보행은 아직 미검증이다.
  [수치와 실패 기록](../workbench/evidence/curated/sim/walking_force_mapping_20261016.json)에
  같은 선택 조건의 force 편향 및 원시 경로를 보존한다.
- 고정 접촉 가속도 후보 `4a368de`도 push/양쪽 update 후 GPU2·카메라·설치
  기본값으로 재검증했다. .2/.4 m/s는 +1.368/+3.103 m 후 stand에 도달했으나,
  .8 m/s는 t=5.80 s/+1.582 m에 RR touchdown timeout으로 실패했다.
  max roll/pitch=.284/.434 rad이며 이 후보도 안정한 보행으로 채택하지 않는다.
  같은 설치 소스에 관측만 추가한 camera-free 재현은 t=5.82 s/+1.268 m에
  finite-force fault로 끝났다. solver나 반환 토크는 수정하지 않았다.
- 위 재현의 pre-step q/dq/payload, 적용 토크, post-step 3D 접촉력을 수집했다.
  접촉 집합이 일치하고 완전히 하중을 받은 137 sample에서 요청 Fz의 실제
  대비 MAE=6.68 N, 토크 한계 및 history filter를 거친 모델의 MAE=1.88 N다.
  22 mm sphere 접촉 모델은 1.79 N으로 차이가 작아 기하 변경은 보류했다.
  clipping 전 원래 frame 접촉 모델의 힘 왕복 오차는 1e-13 N 이하다.
  실제 base 각가속도 대비 pitch RMS 오차는 기존 SRBD 10.37 rad/s²,
  필터 포함 전신 모델 4.91 rad/s²다. 지면 안정화·Coulomb friction·한 step
  측정 차이는 이 계산에 남으며, 접촉 전환 구간으로 확대하지 않는다.
- 다음 후보는 첫 horizon stage에 현재 전신 물성, 실제 swing 토크,
  force/torque filter 및 토크 한계를 넣는다. 초기 힘 분배와 첫 탐색
  wrench도 실제 행렬로 보정한다. 수치 회귀는 force→torque→limit/filter→
  independent forward dynamics 왕복, solver/controller 적용 토크의 일치,
  실제 첫-step bias에 대한 MPC 반응과 JIT 재사용을 포함한다.
  전체 canonical 회귀는 **695 passed / 4 skipped**다. 최종 physical friction
  cone 적용 후에도 같은 결과다. 실제 기록의 세 상태에서 4096×2 CPU 수치
  재생은 모두 유효한 후보를 반환했고 JIT cache entry는 1개였다. 이 재생은
  current support를 horizon에 고정하므로 실제 gait/Genesis 성공 증거가 아니다.
  새 공식 release의 Genesis 보행 gate는 아직 미검증이다.
- 위 첫-stage 후보 `e0f05af`도 push/양쪽 update 후 GPU2에서 실제 보행을
  마쳤다. .2 m/s는 t=10.42 s/+1.467 m에 RL/RR touchdown timeout으로
  실패했다. .4 m/s는 +3.398 m 후 stand에 도달했으나, .8 m/s는
  t=11.42 s/+4.777 m에 finite-force fault로 실패했다. 중단 요청 때는
  세 속도 기록이 이미 완료되어 있었다. 공중 지지력 sample은 모두 0이다.
  solve median은 8.96~9.20 ms, 별도 actuator 행렬 구성은 약 1 ms다.
- 저속 gate도 퇴보하여 전신 역동역학/첫-stage 예측 후보의 runtime과 해당
  테스트를 모두 원복했다. controller와 기존 GPU solver는 `d55683e`와
  byte-for-byte 일치한다. canonical Sim/관련 보행 회귀는 다시
  **653 passed / 4 skipped**다. 구현과 수치 회귀는 후보 commit에,
  실제 원시·검토용 evidence는 현재 작업 기록에 남긴다. 향후 이 실험을
  재생할 때는 해당 commit의 모듈을 사용해야 한다.
- 다음 범위는 안정했던 몸통 제어를 유지한 swing 궤적 추종이다. `4a368de`
  실제 기록을 liftoff부터 재구성한 swing 오차는 median 44.9 mm/p95 99.8 mm,
  가속 구간 속도 투영 추종은 .658이다. 요구 관절 속도 최대 23.29 rad/s,
  실제 최대 12.59 rad/s이며, raw/passive 총 감쇠 1.6을 그대로 이겨야 하는
  기준 감쇠 토크의 p95는 24.82 Nm다. 궤적/좌표를 재구성한 진단으로
  보상 후 성공을 뜻하지 않는다. swing의 감쇠와 기준 속도 보상을 별도
  TDD로 검증하고, 기존 stance 제동·토크/접촉/탐색 제한은 유지한다.

### 물성 수정 및 보행 TDD 재개 (2026-10-15)

- 아래 실보행 t는 초기 settle 2초를 포함한다.
- 사용자 지시로 TDD를 재개했다. 먼저 Genesis의 주축 관성 회전을 링크
  좌표계에 적용하고, bare GO2 Pinocchio 모델의 BODY frame별 물성을 실제
  Genesis descriptor로 교체한다. 고정 링크는 해당 부모 관절에 합산하며
  반복 적용은 질량을 누적하지 않는다. 팔은 기존 dynamic payload로만 더한다.
- 회귀는 4 failed/4 passed → 8 passed. 회전된 관성, 고정 링크 합산,
  누적 방지와 누락 링크 발견 시 부분 변경 방지를 검사한다.
- 실제 Genesis descriptor 독립 합산과 production GO2+payload를 세 자세에서
  대조했다. 총 질량은 15.881010170 kg로 일치하고, 질량/COM/속도/관성의
  최대 절대 오차는 2e-15 이하이다. 장면 build/step 없는 수치 검증이며
  원시는 `workbench/evidence/generated/arm-physics-20261015/verify_model_sync.json`.
- canonical 개발 attachment에서 `pytest workbench/tests/apps/sim
  workbench/tests/research/debug/test_fixed_foot_tracking.py
  workbench/tests/research/experiments/test_contact_walking_progress.py
  -k 'not supported_legs_near_extension_make_mppi_lower_the_predicted_body'`
  결과 591 passed/4 skipped/1 deselected. 제외는 qpOASES, opt-in 실제 camera
  worker 2개, 이번 실행에서 trace를 주지 않은 fixed-foot 물리 검사다.
  deselected 1개는 별도로 보존한 미완성 다리 신전 비용 TDD다.
- `e728f71`을 push/양쪽 update하고 GPU2 발행 Sim `merry-dolphin`에서
  같은 단기 전후좌우 시험을 했다. 전진 6.06 s/+1.045 m, 후진 3.20 s/-0.143 m에
  착지 실패했고 좌/우는 통과(+0.282/-0.452 m). 물성 일치는 보행 개선을
  보장하지 않았으며, 이전 `fbc599e`보다 전후 gate가 퇴보했다.
  실측값이 없으므로 팔 선언 질량 0.5779 kg는 변경하지 않았다.
  원시는 `workbench/evidence/generated/contact-gait/e728f71/`에 회수했다.
- 다음 신전 비용 TDD는 2 failed/3 passed → 5 passed. MPPI 상태/힘 비용에
  지지발의 hip→foot 도달 반경을 추가했다. 95.5 mm hip lateral offset을
  제거한 두 링크 반경에 대해 calf 안전 상한(URDF upper-.04 rad)보다
  20 mm 안쪽에서 연속 벌점을 준다. 예측 COM/자세 및 실제 COM-relative
  hip 위치를 사용하며 swing은 제외하고 landing load를 곱한다.
  신규 비용이 실제 optimizer 출력과 예측 하강을 바꾸는 회귀를 포함한다.
  geometry 없는 기존 solver 호출과 acados 계약은 유지한다.
- 위 신전 비용까지 포함한 같은 canonical Sim/보행 명령은 제외 없이
  593 passed/4 skipped. `2ad0aa4`를 push/양쪽 update하고 GPU2 발행 Sim
  `merry-oyster`로 검증했다. 후진/우측은 통과(-1.619/-0.457 m)했지만,
  전진 7.12 s/+1.515 m에 RR 착지 실패, 좌측 9.14 s/+0.655 m에
  finite rollout 실패다. 따라서 전체 보행 gate는 미통과이며 긴 시험은
  실행하지 않았다. 원시는 `contact-gait/2ad0aa4/`에 회수했다.
- 별도 확정 결함: 설치된 Genesis `abd/forward_dynamics.py`는 FORCE=2에서
  actuator KV를 사용하지 않는다. 기존 `_set_torque_actuation()`의
  `leg_kv_damping=3`은 실제로 적용되지 않고 passive joint damping=.1만
  substep마다 작동했다. FORCE 모드에서 passive damping을 .1+3으로
  적용하고 stand/ready에서는 .1로 복구한다. 회귀 1 failed → green,
  같은 canonical Sim/보행 명령 594 passed/4 skipped.
- 실제 descriptor/armature=.01을 포함한 전진 실패 자세의 고정 자세 선형
  근사에서 50 Hz 폐루프 spectral radius는 1.0515 → .9482였다.
  이 근사는 Genesis 보행 증거가 아니다. 원시 계산은
  `contact-gait/analyze_foot_pd.py`와 `.json`에 있다.
- `9eaf8ba`도 push/양쪽 update 후 GPU2 Sim `merry-ox`로 검증했다.
  force-mode passive damping readback은 12 DOF 모두 3.1이었다.
  3 cm 이상 공중/normal<=2 N/지지 명령>5 N 샘플은 네 방향 모두 0회.
  그러나 전진/좌측은 끝까지 서기로 복귀했어도 진행량 +0.449/+0.100 m로
  최소 gate에 미달했고, 후진은 8.00 s/-0.271 m에 RL 착지 실패했다.
  우측만 통과(-0.401 m). 착지 지연 중앙값은 앞/좌/우 140/170/160 ms로
  늘었다. 원시는 `contact-gait/9eaf8ba/`에 회수했다.
- 다음 수정은 22 mm foot sphere/2 mm local offset의 현재 평지 형상에
  맞춰 지지 높이 상한을 30 mm로 제한한다. 공중 발은 힘 값과 무관하게
  다음 liftoff 전에 지지에서 제외하고, 공중 충격/이전 접촉 이력으로
  새 착지를 확인하지 않는다. 재접촉은 기존 40 ms/80 ms 증거가 필요하다.
  강화 회귀 6 failed/2 passed → green; 같은 전체 명령 601 passed/4 skipped.
- 실제 적용된 감쇠 3의 진행량/착지 지연 퇴보 때문에 source override를
  PyMPC 기본값 1로 되돌린다(실제 passive 총합 3.1 → 1.1).
  앞 실패 자세의 선형 근사 radius=.8892이나 실제 보행 성공의 증거는
  아니다.
- `6ba6b4b`를 push/양쪽 update하고 GPU2 발행 Sim `merry-ibis`에서
  force-mode damping 1.1과 설치 소스 SHA를 확인했다. 단기 네 방향은
  낙상/timeout 없이 끝났고 공중 지지력 샘플은 모두 0회다.
  후진/좌/우는 stand까지 통과(-.991/+.486/-.341 m). 전진은 +1.437 m로
  이동했지만 정지 2초 뒤 pitch=.15998 rad가 handoff 기준 .15보다 커
  walk에 남았다. 따라서 전 방향 단기 gate 완료로 선언하지 않는다.
- 같은 `6ba6b4b`의 독립 긴 시험: 후진/좌/우 20초 및 후진 이동·정지
  3회 반복은 통과(-4.322/+1.298/-1.133/-3.450 m).
  전진은 15.70 s/+3.754 m에 RL touchdown timeout. 전진 3회 반복은
  25.98 s/+4.964 m까지 fault 없이 움직였지만 마지막 stand 복귀가 안 됐다.
  긴 시험도 공중 지지력 샘플은 전부 0회다. 원시는
  `workbench/evidence/generated/contact-gait/6ba6b4b/`에 회수한다.
- 전진 정지 9.5~9.98 s의 명령 Fz=134.37 N, 실제 Fz=154.87 N으로
  평균 20.50 N 차이가 났다. 같은 실제 물성 모델에서 누락된 다리 중력의
  등가 Fz=14.09 N, 중력+감쇠 근사는 18.63 N이다. 이는 quasi-static
  근사이며 나머지 관성/마찰/필터 영향까지 동일하다는 주장은 아니다.
  계산 원시는 `contact-gait/analyze_force_bias.py`와 `.json`.
- 토크 변환을 stance에서 `g_leg(q)-J.T@GRF`로 수정하고 swing/search에도
  같은 다리 중력 보상을 더한다. 현재 자세의 Pinocchio g를 매 sample에
  읽으며 기존 force/torque cap과 contact gate는 유지한다.
  세 모드 회귀 3 failed → green, canonical Sim/보행 604 passed/4 skipped.
  fixed-foot 진단도 production torque path와 같은 중력 입력을 사용한다.
- `ca35984`를 push/양쪽 update 후 GPU2 Sim `merry-peacock`에서 실제 중력
  벡터와 소스 SHA를 확인했다. 후진/좌/우는 통과(-.755/+.528/-.523 m),
  전진은 6.64 s/+1.201 m에 RL/RR 착지 실패해 퇴보했다. 공중 지지력은
  전 방향 0회지만 전진 보행 gate는 미통과다. 이 후보의 긴 시험은
  실행하지 않았고, 원시는 `contact-gait/ca35984/`에 회수했다.
- 다음 후보는 확인된 지지발의 GRF→토크 변환에만 g를 더한다.
  공중 swing/search에 g를 더하면 기존 착지 추종 하중도 바뀌므로, 이
  변경을 제거한 A/B다. 세 모드 회귀 2 failed/1 passed → green,
  canonical Sim/보행 604 passed/4 skipped.
- `1b3498f`를 push/양쪽 update 후 GPU2 Sim `merry-boar`에서 검증했다.
  후진/좌/우는 통과(-.815/+.392/-.463 m)했으나 전진은 3.70 s/+.352 m에
  RL/RR 착지 실패로 더 퇴보했다. 공중 지지력은 네 방향 모두 0회다.
  긴 시험은 실행하지 않았고 원시는 `contact-gait/1b3498f/`에 회수했다.
- 직전 두 후보의 전진 trace에서 착지 확인 직후 MPPI가 뒤 지지발의 Fz를
  거의 0으로 줄인 뒤 실제 접촉을 잃는 구간을 확인했다. 다음 TDD는 확인된
  지지발의 ramp 적용 후 명령 Fz 하한 10 N을 넣는다. 회귀 2 failed/4 passed로
  무시되는 입력을 재현했다. 하한은 seed/후보/평가/반환에 동일하게 적용하고,
  swing은 계속 0, 기존 상한과 접촉 확인·높이·timeout 기준은 유지한다.
  가중 후보가 0으로 퇴화할 때도 평가와 반환이 같도록 다시 project한다.
  위 canonical Sim/보행 명령은 610 passed/4 skipped다.
  이 하한이 실제 접촉 유지나 전진 성공을 보장하는지는 별도 GPU 시험 대상이다.
- `cb9e55d`를 push/양쪽 update하고 GPU2 Sim `merry-koi`에서 단기 네 방향을
  검증했다. 하중이 있는 발의 실제 토크 변환 전 Fz 하한은 trace에서도
  10 N으로 확인됐다. 후진/좌/우는 stand까지 통과(-1.167/+.551/-.423 m),
  전진은 7.38 s/+1.542 m에 RR touchdown timeout으로 실패했다.
  네 방향 모두 공중 지지력 샘플은 0회이며 원시는 `contact-gait/cb9e55d/`.
  MPPI solve 중앙값은 약 5 ms이고, 평지/카메라 없는 물리 step 중앙값은
  17~20 ms다. 전체 UI/영상 포함 실시간 성능을 입증하는 시험은 아니다.
- 전진 trace에서 대기 때마다 목표 vx=.8→0이 반복되고 nose-down pitch가
  증가했다. 다음 후보는 사용자가 계속 이동을 명령한 착지 대기 동안만
  실측 수평 속도의 75%를 reference로 사용한다. seed의 속도 feedback
  gain=8 기준 오차를 실측 속도의 25%로 줄이는 가설이며, 실제 감속 상한을
  보장하지 않는다. 사용자 정지 명령과 접촉/timeout 판정은 유지한다.
  실제 Pinocchio 모델을 거친 production `_sample` 회귀 3 failed/2 passed →
  green이며, canonical Sim/보행 명령은 615 passed/4 skipped다.
- `0783bd3`도 push/양쪽 update 후 GPU2 Sim `merry-chicken`에서 확인했다.
  후진/좌/우는 stand까지 통과(-1.490/+.520/-.511 m)했으나 전진은
  6.64 s/+1.394 m에 RR timeout, max pitch=.466 rad로 퇴보했다.
  공중 지지력은 전부 0회. 원시는 `contact-gait/0783bd3/`에 회수하며,
  착지 대기 속도 reference 변경은 취소하고 `cb9e55d`의 대기 동작으로 돌아간다.
- 별도 누락: 접촉을 잃어 search 중인 발은 지지력뿐 아니라 MPPI의 도달
  범위 비용에서도 제외되고 있었다. `cb9e55d` 마지막 자세/목표를 재구성하면
  RR의 지면 도달 반경은 약 .399 m로 calf 안전 반경 .386 m를 넘는다.
  이 재구성은 post-step pose와 진단 목표의 근사이며 시간 정렬된 전체 상태
  재현은 아니다. 이를 반올림한 별도 수치 회귀에서 search mask를 전달해도
  force가 완전히 같은 1 failed/4 passed로 누락을 재현했다.
- 현재 search mask는 지면 목표의 기하 비용에만 포함한다. 미확인 발의
  GRF와 contact/load는 여전히 0이며 접촉 확인 없이 지지를 약속하지 않는다.
  실제 optimizer 출력 및 production `_sample` 입력 경계를 포함한
  canonical Sim/보행 명령은 620 passed/4 skipped다. 원격 보행 gate는 미검증.
- `6800d01`을 push/양쪽 update 후 GPU2 Sim `merry-mule`에서 확인했다.
  후진/좌/우는 통과(-.961/+.530/-.494 m), 전진은 5.70 s/+.915 m에
  RL timeout으로 실패했다. 전진 max pitch는 .348 rad로 줄었지만 성공은
  더 짧아졌다. 공중 지지력은 네 방향 모두 0회. 원시는 `contact-gait/6800d01/`.
- 다음 A/B는 반복 퇴보한 다리 중력 feedforward를 토크에서 제거한다.
  실제 물성 동기화, 감쇠, 지지력 하한과 search 기하 비용은 유지한다.
  현재 Pinocchio g는 진단에 계속 기록하지만 동적 접촉력 추종이 검증되기
  전에는 actuation에 더하지 않는다. 이 변경의 회귀 1 failed/2 passed →
  green, canonical Sim/보행 620 passed/4 skipped. 원격 gate는 미검증이다.
- `1d0594f`를 push/양쪽 update 후 GPU2 Sim `merry-parrot`에서 확인했다.
  후진/좌/우는 통과(-1.606/+.480/-.512 m)했으나 전진은 5.78 s/+.893 m에
  RL timeout이었다. max pitch=.423 rad, 공중 지지력은 네 방향 모두 0회다.
  g 제거만으로는 search 기하 비용 후보의 퇴보를 회복하지 못했다.
  원시는 `contact-gait/1d0594f/`에 회수했다.
- 설치 Genesis `constraint/solver.py`의 `func_update_contact_force`는 매 solve에
  링크 force를 0으로 초기화하고 이번 contact force를 합산한다. rigid substep의
  constraint solve에서 호출되며 `get_links_net_contact_force()`는 이 배열의
  현재 snapshot이다. 현재 20 ms/4 substeps에서는 제어·접촉 관측 50 Hz와
  물리 적분 200 Hz가 다르다. 실제 영향은 별도 cadence A/B가 필요하다.
- 진단 도구에 `--substeps`를 추가하고 dt/substeps를 함께 기록한다. 설치
  기본값은 바꾸지 않고, 20 ms/4와 5 ms/1의 같은 5 ms 적분 간격을 비교한다.
  순수 회귀 5 failed/2 passed → green, canonical Sim/보행 625 passed/4 skipped.
  이 주기 비교의 원격 보행 및 성능 gate는 아직 미검증이다.
- `39c1bcc`까지 push/양쪽 update하고 동일 controller SHA의 GPU2 공식 Sim
  `merry-wren`에서 진단 설정 5 ms/1 substep을 실행했다. 5 ms 적분은 유지하고
  접촉 관측·다리 제어를 200 Hz로 높였다. 기본 YAML은 20 ms/4로 유지한다.
  단기 네 방향은 stand까지 통과(+.991/-1.210/+.389/-.387 m), 전진 max pitch는
  .118 rad였다. 20초 전진/후진/우측도 통과(+4.679/-3.787/-1.085 m)했고
  후진 이동·정지 3회는 -2.880 m로 통과했다.
  좌측 20초는 14.315 s에 finite trajectory 실패, 전진 3회는 25.995 s에 fault 없이
  끝났으나 walk에 남아 미통과다. 모든 공중 지지력 샘플은 0회다.
  원시는 `contact-gait/39c1bcc-hz200/`에 회수한다.
  진단 trace를 포함한 sim/wall은 .28~.42, step 중앙값은 14~17 ms라 200 Hz
  실시간을 충족하지 않는다. 기본 주기를 바꿀 근거로 사용하지 않는다.
- 다음 후보는 GPU MPPI force 갱신을 25 → 50 Hz로 예측 stage dt=.02에 맞춘다.
  현재는 stage-0 힘을 40 ms 동안 유지해 20 ms 예측과 실행이 달랐다.
  실제 토크 루프 호출 간격 및 CPU acados의 기존 25 Hz 유지 회귀를 포함해
  canonical Sim/보행 635 passed/4 skipped다. 원격 보행/성능 gate는 미검증이다.
- 별도 성능 수정은 팔의 질량/COM/관성을 계산할 때 링크별 pos/quat/vel/ang
  네 readback을 entity의 네 배치 조회로 바꾼다. 살아 있는 선택 링크만 읽고
  local index/순서를 유지한다. scalar-only entity는 기존 경로를 유지하되,
  bulk API가 잘못된 배열을 반환하면 숨기지 않고 거절한다. 링크의 질량·관성
  값, 주축 회전, 속도 유한차분과 reset 의미는 유지한다.
  세 회귀 3 failed → green. 새 force 주기까지 포함한 canonical Sim/보행 명령은
  638 passed/4 skipped다. GPU 실물 API 동등성과 실제 step 절감은 다음
  원격 후보에서 측정하며, 단위시험만으로 가속을 선언하지 않는다.
- `961c2eb`(force 50 Hz)와 `678bbb7`(payload bulk)을 push/양쪽 update 후 GPU2
  Sim `merry-wombat`에서 실물 API를 비교했다. 질량/COM/속도/관성 오차는 모두
  0이었으며 payload measure 중앙값은 scalar 10.617 → bulk 3.315 ms였다.
  기존 50 Hz 제어는 후진/좌/우 통과(-1.478/+.385/-.465 m), 전진은
  9.10 s/+1.651 m에 RR timeout. 카메라 없는 진단 처리율은 실시간의
  1.29~1.51배였다. 원시는 `contact-gait/678bbb7/`.
- 같은 공식 이미지에서 10 ms/2 substeps, 기록은 기존과 같은 20 ms로
  비교했다. 5 ms 물리 적분과 GPU MPPI 50 Hz는 유지하고 제어·관측만 100 Hz다.
  단기 네 방향은 모두 통과(+1.113/-1.291/+.363/-.435 m). 20초 네 방향도
  stand까지 통과(+4.001/-4.280/+1.406/-1.200 m), 후진 이동·정지 3회도
  -4.045 m로 통과했다. 전진 3회만 19.74 s/+3.243 m에 RL timeout이었다.
  전체 9/10 통과이며 공중 지지력 샘플은 전부 0회다. 진단 처리율은
  단기 1.08~1.18배, 긴 이동 .91~.98배였다. 원시는 `contact-gait/678bbb7-hz100/`.
- 전진 반복의 두 번째 정지 끝 t=17.98 s에는 속도가 거의 0이고 4발 full load였지만
  pitch=.157374 rad로 stand 기준 .15를 넘었다. walk에 남아 다음 출발을 한 뒤
  무너졌다. 다음 후보는 정지 중의 완전한 네 발 지지, COM speed<.08 m/s,
  angular speed<.15 rad/s, max joint speed<.5 rad/s에서만 GPU 다리 중력을 더한다.
  동적 보행, 미착지/부분 하중, CPU acados는 제외한다. stand 판정은 바꾸지 않는다.
- 위 후보와 100 Hz/2 substeps 기본 설정 회귀는 red→green. canonical Sim/보행
  명령은 647 passed/4 skipped다. 설치된 기본 config를 덮어쓰지 않는 원격 시험은
  다음 단계이며 아직 통과로 선언하지 않는다. full required/extended 및 isolated
  release build/verify는 이번 보행 범위에서 미실행이다.
- `ddc9a8f`를 push하고 두 호스트의 공식 `elesim-update`를 완료했다. 원격
  GPU2 전용 `tddwalk`의 공식 Sim `merry-tuna`에서 controller SHA와 설치 기본값
  dt=.01/substeps=2, MPPI 4096×2를 검사하고 dt override 없이 실행했다.
  단기 전후좌우, 20초 전후좌우, 전진/후진 이동·정지 3회 모두 통과했다(10/10).
  20초 이동량은 +3.949/-4.514/+1.114/-1.391 m이며 모든 시험이 stand로 끝났다.
  전진 반복은 +4.460 m로 끝났고 중간 3회 정지 모두 2초 안에 stand로 복귀했다.
  원시는 `workbench/evidence/generated/contact-gait/ddc9a8f-defaults/`에 회수했다.
- 위 10개 시험의 공중 지지력 샘플(관측 normal<=2 N, foot height>.03 m,
  적용 GRFz>5 N)은 모두 0회다. 착지 확인 지연 중앙값은 전진/후진/좌/우
  100/80/120/120 ms다. 이는 정해진 샘플 판정이며 모든 공중 발 떨림이나
  자연스러운 보행을 입증하는 지표는 아니다. 카메라 없는 진단 sim/wall은
  .88~1.11이고, 통신·영상·실물 검증을 포함하지 않는다.
- 속도 추종은 미해결이다. 20초 전진의 t=4~22 s에서 평균 실측 vx=.202 m/s,
  평균 reference vx=.479 m/s이고, 명령은 .8 m/s다. 착지 대기 비율은 약 40%다.
  같은 좌측 평균 vy=.054 m/s/명령 .35 m/s이고 yaw도 누적된다. 넘어짐 gate의
  통과를 목표속도나 직진 품질의 통과로 확대하지 않는다.
- 반복 진단이 마지막 정지만 검사하던 공백도 보강했다. 이제 각 이동·정지
  주기의 마지막 sample에서 stand/fault/fall/시간 커버리지를 검사하고 summary에
  남긴다. 회귀 6 failed/7 passed → 13 passed, canonical Sim/보행 명령은
  653 passed/4 skipped다. 기존 원시 10개도 강화된 판정으로 14/14 정지를 통과했다.
- 같은 `ddc9a8f` 설치 기본값의 추가 긴 시험도 5/5 통과했다. 전진/후진
  6초 이동·2초 정지 10회(총 82초)는 +15.536/-12.577 m, 전진 60초는
  +12.608 m, 좌/우 40초는 +2.525/-2.549 m였다. 모두 stand 복귀, fault/낙상과
  위 공중 지지력 샘플은 0회다. 진단 sim/wall은 .88~.98이다.
  전진 60초의 max pitch=.384 rad, 후진 반복의 옆 방향 누적 이동=-4.111 m라
  자세 품질과 방향 유지까지 해결됐다고 선언하지 않는다. 별도 카메라/팔 동작
  결합 검증을 이어간다. 원시는 `contact-gait/ddc9a8f-defaults-stress/`에 보존한다.
  강화된 판정으로 긴 시험의 정지 23/23도 통과했다. 설정·판정 범위·15개 수치는
  [검토용 요약](../workbench/evidence/curated/sim/contact_gait_20261015.json)에 보존한다.
- `f49ece6`도 push/양쪽 공식 update를 마쳤다. 원격 release
  `751cea28df77810f59f77cf1b1e1f8a5adcd652d892647f8a755e73d8ece7265`를
  `tddwalk`에 등록하고 같은 GPU2/controller SHA/설치 기본값으로 결합 검증했다.
  두 1280×720 카메라를 켠 전진 20초(+4.568 m), 작은 팔 굽힘 운동을 넣은
  전진 20초(+3.456 m), 두 부하를 함께 넣은 전진·정지 3회(+4.768 m) 모두
  통과했다. 추가 정지 5/5도 실제 stand에 도달했다. 현재 이 범위는 총 18/18,
  정지 42/42이며 timeout/낙상과 위 공중 지지력 샘플은 0회다.
  팔 시험은 theta1/2에 각각 .12/.08 rad, 8초 주기 사인 운동을 더한다.
- 카메라 첫 실행은 hand-eye/observer 4.77/4.60 FPS, 뒤의 카메라+팔 시험은
  19.90/20.16 FPS였다. 초기 준비의 영향은 별도 분리하지 않았다. 결합 시험의
  sim/wall=.681로 실시간에는 미달하며, camera-only=.823, arm-only=.827이다.
  이 시험은 실제 GPU 렌더/FrameHub까지 포함하지만 인코더, DDS, WebRTC,
  UI 화면이나 실물을 포함하지 않는다. 영상 수용시험의 완료로 확대하지 않는다.
  원시는 `workbench/evidence/generated/contact-gait/f49ece6-defaults-integration/`,
  세부 요약은 위 검토용 JSON의 `integration`이다.

### 보행 품질 재점검 (2026-10-15)

- 사용자의 낮은 자세·취권 보행 피드백을 받아 속도 추종과 자세를 다시
  측정한다. 기존 18/18 낙상 방지 gate는 자연스러운 보행이나 .8 m/s 추종을
  입증하지 않는다. 조회 시 실제 `test` Sim은 정지 상태여서 조작하지 않고,
  공식 `4144148` release를 격리된 `tddwalk`에 등록해 GPU2에서 재생했다.
- 설치 기본 dt=.01/substeps=2, MPPI 4096×2를 유지하고 두 1280×720 카메라를
  켠 채 .2/.4/.8 m/s 전진을 각각 12초 수행했다. 세 시험 모두 낙상/fault 없이
  정지 후 stand로 복귀했지만, t=4~14 s의 평균 실제 전진 속도는
  .061/.162/.225 m/s다. 평균 MPC 속도 참조는 .119/.237/.470 m/s이고
  착지 대기는 40.4/40.8/41.2%다. 목표속도를 낮춰도 추종 손실이 남는다.
- 준비 자세 전환은 몸통 높이를 .304 → .262 m로 낮춘다. 반면 정상 보행
  구간의 높이 중앙값은 .320/.322/.331 m로 nominal .3 m보다 높고,
  pitch RMS는 3.79/7.28/6.87도다. 낮은 준비 자세와 보행 중 기울기는
  별도로 다룬다. 관절 토크 포화나 6 rad/s 초과 샘플은 이 구간에 없었다.
- 완전 하중 지지발의 실제 Fz−명령 Fz 차는 .8 m/s에서 평균 22.20 N이다.
  같은 자세의 준정적 근사는 중력 7.66 N, 원시 관절속도 감쇠 13.76 N을
  예측한다. 감쇠의 등가 수평 성분은 -15.80 N이다. 관성·접촉 과도·한 tick
  시간 차를 생략한 근사이므로 원인 확정이나 정확한 힘 복원으로 취급하지
  않는다. 지지 중 몸통 이동을 관절 감쇠가 제동하는지 별도 TDD로 확인한다.
- 원시는 `workbench/evidence/generated/contact-gait/4144148-defaults-speed-sweep/`,
  [검토용 수치](../workbench/evidence/curated/sim/walking_quality_20261015.json)에
  설정·판정 범위·제약을 기록했다. PNG 저장 부하가 있으므로 처리율 benchmark로
  사용하지 않는다. 이 직접 scene probe는 idle camera cache를 갱신하지 않아
  준비 자세의 높이는 이미지 대신 수치 기록으로 판정했다.
- GPU stance 감쇠 후보는 몸통 이동을 상쇄해 발을 고정하는 관절 속도를
  최소제곱으로 구하고, 하중 계수로 blend한 속도 오차를 사용한다. 실제 slip과
  Jacobian null space는 계속 감쇠하고 작은 특이값의 역변환은 제한한다.
  passive 감쇠 1.1, swing/search, CPU acados, 접촉·토크 안전 기준은 유지한다.
- 공식 개발 이미지가 현재 설정 alias로 존재하지 않아 생성된 `elesim-dev`
  wrapper로 빌드 중이다. 기다리는 동안 production 토크 함수 본문을 그대로
  컴파일한 NumPy-only 제한 수치 진단은 3 failed/3 passed → 6 passed였다.
  runtime import나 물리를 포함하지 않아 canonical 회귀/실보행 gate로
  계산하지 않는다. 실제 개발 attachment 회귀와 공식 release A/B는 미검증이다.
- 초기 후보 `c90de7c`는 push/양쪽 공식 update 후 GPU2 전용 `tddwalk`에서
  같은 카메라 포함 3속도 시험을 마쳤다. .2/.4 m/s는 12초에
  +1.239/+2.523 m로 stand까지 통과했지만, .8 m/s는 t=8.14 s/+3.098 m에
  `GPU MPC produced no finite force trajectory`로 실패했다. 최대 pitch는
  .458 rad였고 공중 지지력 샘플은 세 경우 모두 0회다. 속도 증가가 보행
  안정성을 입증하지 않으므로 이 후보를 채택하지 않는다. 원시는
  `contact-gait/c90de7c-defaults-speed-sweep/`에 보존한다.
- 다음 후보는 수평 base 이동만 감쇠 기준으로 삼고 회전/수직 움직임의
  감쇠는 남긴다. 발을 고정하는 모든 관절 속도를 제외하면 자세가 무너질
  때의 제동도 사라지는 초기 후보의 문제를 분리한다. 같은 sample에서
  읽은 세계 base 속도를 캐시하며 추가 Genesis readback은 없다.
  제한 수치 진단은 새 회전/수직 제동 회귀에서 2 failed/6 passed → 8 passed다.
  canonical 회귀와 새 공식 release 보행은 아직 미검증이다.
- 위 수평 이동 후보 `0623a58`도 push/양쪽 공식 update 뒤 같은 GPU2
  3속도 시험을 마쳤다. .2/.4 m/s는 +.691/+2.337 m로 stand까지 통과했으나,
  .8 m/s는 t=6.10 s/+1.317 m에 RL touchdown timeout으로 실패했다.
  최대 pitch=.309 rad, 공중 지지력 샘플은 세 경우 모두 0회다. 제동 제거만으로
  안정성과 추종을 함께 개선하지 못해 초기 `4144148`의 controller 및 해당
  회귀 파일을 정확히 복구했다. 실패한 기능의 단위 테스트도 함께 되돌리고
  수치/실보행 기록과 두 후보 commit은 보존한다.
- 다음 조사 범위는 GRF→토크 변환의 관절 중력·관성·passive 감쇠와 실제
  접촉력의 일치다. MPPI 4096×2, UI 목표속도 .8/.35/.8 및 접촉 안전 기준은
  유지한다. 아직 새로운 동역학 보상 기능이나 보행 성공을 선언하지 않는다.

### 팔 물성 감사 (2026-10-15, 보행 TDD 일시 중지)

- 사용자 요청으로 보행 TDD를 멈추고 읽기/수치 감사만 수행했다. 아직 물성이나
  제어 동작을 수정하지 않았다. 이전 도달 범위 비용 TDD의 uncommitted 작업은 보존했다.
- 배포 모델 SHA-256은 로컬/원격 모두
  `bb949e2b508c23cb4979c41545cbf80373710e211aa6b29cbc4cdcd31866175c`다.
  팔 17개 링크 질량 합계는 0.5779 kg, bare GO2 URDF는 15.019 kg,
  MPPI가 사용하는 합계는 15.5969 kg다. 팔 질량의 이중 반영은 없었다.
- 팔 관성행렬은 모두 양수 및 관성 삼각조건을 만족한다. 원본 URDF의 전체
  Pinocchio 모델과 bare GO2 + production payload 합성은 세 자세(직선/접음/
  몸통 회전 및 관절 운동)에서 질량/COM/속도/관성이 약 1e-16 오차로 일치했다.
- **실제 Genesis 입력에서 관성 회전 누락을 확인했다.** Genesis는
  `desc.inertia`를 주축 좌표계로 저장하고 `desc.inertial_quat`로 링크 좌표계와
  연결한다. `ArmPayloadCompensator.measure()`는 이 회전을 적용하지 않고
  링크 회전만 곱한다. 실제 descriptor로 재계산하면 팔 관성 상대 오차가
  5.53/14.84/8.20%, 전체 관성 기준 0.53/0.61/0.56%였다. 누락 회전을
  수치 감사에만 포함하면 최대 절대 오차가 1e-9 kg m² 이하로 줄었다.
- 별도 불일치: 같은 `merge_fixed_links=False` 파싱에서 Genesis 전체 질량은
  **15.881010 kg**다. URDF에서 관성이 없는 `*_calflower`, `*_calflower1`
  8개 링크에 각각 약 37.06/33.96 g를 추정해 총 0.28411 kg(1.82%)를 더한다.
  MPPI의 bare Pinocchio 모델에는 이 추가 질량이 없다. 팔 질량은 Genesis에서도
  정확히 0.5779 kg다. 이 결과는 장면 build/step 없이 descriptor를 읽은 감사다.
- 0.5779 kg의 실물 타당성은 아직 미확인이다. 저장된 관성은 선언 질량과
  메시 기반 근사값이며 실물 모터/배선 등을 포함한 측정값과의 대조가 필요하다.
  메시 일부는 닫힌 체적이 아니므로 메시 균일밀도 재계산만으로 물성 오답을
  확정하지 않았다. 두 모델 불일치의 보행 영향도 아직 검증하지 않았다.
- 감사 코드와 수치 결과는 Git-ignored
  `workbench/evidence/generated/arm-physics-20261015/`의 `audit_arm.py`,
  `audit.json`, `genesis_metadata.py`, `genesis_metadata.json`,
  `mesh_recheck.json`에 있다. 이 감사 이후 사용자 지시로 위 TDD를 재개했다.

### 발행 릴리즈 기반 TDD 및 착지 도달 범위 (2026-10-14)

- `286d1c2`를 push/양쪽 update하고 원격 GPU2 Sim `merry-lion`에서
  전후좌우를 재시험했다. 미래 착지 위치 미리보기는 전진 7.68 s/+1.517 m,
  후진 9.46 s/-1.423 m까지 진행했지만 양쪽 모두 착지 실패. 좌측은
  6초 명령+2초 정지 통과(+0.489 m), 우측도 통과(-0.519 m).
- `d5f5b0c`는 MPPI가 착지 직후 하중 계수 0.33→0.67→1.0을 예측 동역학과
  초기 힘 배분에 반영한다. 로컬 JAX 실패 자세에서 실제 전달 수직 명령이
  약 133→156 N으로 증가했고 Sim+보행 회귀 598 passed/4 skipped였다.
  push/양쪽 update한 원격 GPU2 발행 이미지 `merry-cheetah`의 동일 시험은
  전진 5.74 s/+1.053 m, 후진 5.42 s/-0.977 m, 우측 5.16 s/-0.392 m에
  착지 timeout으로 실패했고 좌측만 통과(+0.407 m). 즉 하중 모델만으로는
  안정화되지 않았으며 전후 방향은 이전 이미지보다 더 일찍 실패했다.
  원시 JSONL은 `workbench/evidence/generated/contact-gait/d5f5b0c/`에 있다.
- `d5f5b0c` 실패 자세의 추가 TDD: 전진 RL과 후진 FL은 처음 접촉한 뒤 몸통이
  계속 이동해 calf 신전 한계 부근에서 지지를 잃었다. 발을 들어 올릴 때
  예상 착지와 지지 종료 양쪽에서 닿을 수 있는 지면 목표를 선택하고, 이미
  지지 중인 발의 세계 좌표는 유지한다. 두 실제 실패 자세의 목표 회귀는
  red→green, Sim+보행 회귀 600 passed/4 skipped. `ba38e6b`로 push/양쪽
  update 후 원격 GPU2 `merry-quail` 실보행은 전진 4.64 s/-0.033 m,
  후진 6.78 s/-0.629 m에 실패했으나 좌/우는 모두 통과(+0.551/-0.631 m).
  초반 실제 몸통 속도가 거의 0인데도 목표 속도 0.8 m/s를 미래 이동량으로
  적용해 전진 발을 과도하게 앞에 두는 역효과가 확인됐다. 원시 JSONL은
  `workbench/evidence/generated/contact-gait/ba38e6b/`에 있다.
- `d3b0762`는 저속 출발의 과도한 목표 이동을 재현해 red→green으로 고치고,
  전후 방향의 실제 진행 속도가 충분할 때만 해당 속도 예측을 키웠다.
  Sim+보행 회귀 601 passed/4 skipped. push/양쪽 update 후 원격 GPU2
  `merry-pelican` 실보행은 전진 8.28 s/+1.819 m, 후진 5.42 s/-0.977 m에
  착지 timeout으로 실패했고 좌/우는 모두 통과(+0.551/-0.631 m).
  전진은 6초 명령 구간 뒤 정지 무렵 오른쪽 뒷발 접촉을 잃었다. 전진 RR은
  지지력 약 70 N 중에도 접촉이 끊겼고 후진 FL은 종아리 신전 한계에 닿았다.
  원시 JSONL은 `workbench/evidence/generated/contact-gait/d3b0762/`에 있다.
- `ca52d30`은 보행 중 순간 저속에도 명령 속도를 예상 지지 종료에 반영했다.
  회귀 603 passed/4 skipped 후 push/양쪽 update, 원격 GPU2 `merry-eagle`의
  전진은 5.42 s/+0.948 m, 후진은 5.06 s/-0.782 m로 더 일찍 착지 실패했다.
  좌/우는 같은 +0.551/-0.631 m로 통과했다. 이 gait-age 예측 변경은 취소하고
  `d3b0762`의 발 배치로 돌아간다. 원시 결과는
  `workbench/evidence/generated/contact-gait/ca52d30/`에 있다.
- `b0b35ed` TDD는 예정 착지 때 발 중앙이 0.026~0.027 m, 실제 normal=0인데
  목표가 0.025 m에서 속도 0으로 끝나는 현상에 집중한다. 힘을 받는 실제
  발 중앙은 약 0.023 m였다. 스윙 actuator 끝점만 nominal 지면보다 8 mm
  아래로 내려 하중을 유도하고, 미착지 시 탐색이 끝점에서 이어지도록 한다.
  MPC 지면 목표는 유지하며 스윙 편향과 탐색을 합한 추가 깊이는 최대 4 cm다.
  두 회귀 red→green, adapter/contact/workspace 77 passed,
  Sim+보행 회귀 603 passed/4 skipped. push/양쪽 update 후 원격 GPU2
  `merry-walrus`는 전진 +1.215 m, 후진 -1.743 m로 각각 6초 명령+2초 정지
  및 stand 복귀를 통과했다. 두 방향의 예정 착지→실제 접촉 지연 중앙값은
  40→0 ms였다. 최대 pitch는 전진 .343→.165, 후진 .514→.239 rad였다.
  그러나 좌측은 6.88 s/+0.111 m, 우측은 3.82 s/반대 방향 +0.128 m에
  착지 실패해 **전체 보행 gate는 여전히 미통과**다. 원시 결과는
  `workbench/evidence/generated/contact-gait/b0b35ed/`에 있다.
- `fbc599e`는 8 mm actuator 하강을 명령의 앞뒤 성분 비율로 정하고 발을
  들어 올릴 때 고정한다. 순수 좌우는 이전 끝점으로 되돌리고, stop 도중
  이미 진행 중인 swing의 끝점은 바뀌지 않는다. 방향별 3개 회귀 red→green,
  Sim+보행 회귀 608 passed/4 skipped.
  push/양쪽 update 후 GPU2 발행 Sim `merry-toad`에서 네 방향 모두
  6초 명령+2초 정지 및 stand 복귀를 통과했다. 이동은 전진 +1.215 m,
  후진 -1.743 m, 좌 +0.551 m, 우 -0.631 m다. 전후 예정 착지→접촉 중앙값은
  0 ms, 좌우는 기존 40 ms이며 timeout은 없었다. 최대 pitch는 전진 .165,
  후진 .254 rad다. raw JSONL은
  `workbench/evidence/generated/contact-gait/fbc599e/`에 보관한다.
  확장 시험은 전진 16.26 s/+2.784 m, 후진 11.24 s/-2.829 m,
  우측 13.14 s/-1.198 m에 실패했다. 좌측 20초는 통과(+1.751 m).
  전후 이동·정지 3회 반복은 각각 14.90/13.46 s에 실패했다.
  긴 보행 gate는 미통과이며, 이 뒤 사용자가 팔 물성 감사만 요청해
  도달 범위 비용 TDD를 일시 중지했다.
- `fa85b44`를 push/양쪽 update하고 원격 GPU2 Sim `merry-elephant`로
  4096×2, 50 Hz, substeps=4, 전후좌우 6초 명령+2초 정지를 실행했다.
  좌우는 통과(+0.616/-0.511 m), 전진은 7.08 s/+1.111 m,
  후진은 6.16 s/-0.570 m에 넘어져 미통과. 전진의 최대 roll 0.775 rad였다.
  필터만 force/tau alpha 1로 바꾸는 A/B는 네 방향 모두 더 빨리 실패했다.
  현장 설정 .7/.5 필터의 지연은 약 37 ms이며 이 값은 유지한다.
- MPPI horizon TDD: 현재 지지발이 horizon 중 swing 후 재접촉할 때
  이전 위치를 계속 쓰던 모델을 수정했다. 예측 착지 이후만 새 지지 위치로
  표시하고, 이미 시작한 swing의 latch는 유지한다.
- `731c5ca`도 push 및 양쪽 `elesim-update` 완료. 원격 Sim `merry-orca`의
  기본 50 Hz 전후좌우는 모두 실패(3.88/3.18/5.24/5.88 s).
  같은 이미지에서 dt=0.005, MPPI solve 25 Hz로 비교하면 좌우는 6초 명령과
  2초 정지까지 통과(각 약 0.57 m, 최대 roll/pitch 약 0.12 rad)했다.
  전진은 4.52 s/+0.486 m, 후진은 7.24 s/-1.395 m에 실패했다.
  각 시간은 settle 2초를 포함한다. 이는 단일 trial이며 기본 dt 변경 근거로
  확정하지 않는다. dt=0.02/substeps=4도 좌우는 통과했으나 전후는 실패했다.
  물리 1 step에서 -5.5 cm였던 최소 foot Z는 4 substeps에서 +1.6 cm 이상이었다.
  200 Hz control의 step median 약 14 ms 대비 50 Hz/4 substeps는 약 21 ms였다.
- 전진 실패에서 base z=0.38 m 및 rear calf 신전 한계 포화를 확인했다.
  payload 높이 목표가 `max(nominal, current COM - .01)`이라 몸통 상승에 대한
  복원 오차가 1 cm로 제한되는 결함을 수정한다. 목표를 nominal body height +
  현재 COM-base 높이 차로 정해 팔의 COM 상승과 몸통 상승을 구분한다.
  `0efbe67`로 push/양쪽 update 후 Sim `merry-magpie`에서 회귀 green 및
  substeps=4 물리 보행을 확인했다. 전후 실패가 남고 좌측만 정지까지 통과해
  높이 기준 수정만으로 보행이 안정화됐다고 판단하지 않는다.
- 추가 계측에서 base가 목표보다 6 cm 높고 상승 중에도 MPPI 요청 Fz가
  207 N(중력 약 153 N)을 기록했다. 솔버 단독의 상승/warm-start 회귀도 red였다.
  수직 높이/속도 및 roll/pitch 비용을 강화하고 현재 상태 seed도 결정적
  incumbent 후보에 포함한다. 240 ms 계획의 추가 상승을 2 cm 이하로 제한하고
  끝에서 하강하도록 검사한다. canonical 개발 attachment에서 Sim+보행 진행
  회귀 **566 passed, 3 skipped**. 강한 벌점의 실제 보행 영향은 발행 이미지로
  별도 검증해야 하며 이 단계의 성공을 아직 선언하지 않는다.
- `a46fbaa8be4c2ab7e7790a408b897143a2a9c31e`를 main에 push하고 양쪽의 설치된
  `elesim-update`를 실행했다. 양쪽 release manifest의 source_revision을 확인했다.
  원격 Sim `merry-eel`에서 네 방향 실패가 다시 재현됐다. 임시 app 소스를
  덮어쓰지 않고 발행 이미지의 site-packages를 사용했다.
- 원격 `tddwalk` Sim 전용 인스턴스는 GPU 2 UUID
  `GPU-a40782b2-881c-e285-2a8e-66cae9162a01` 하나만 Docker에 예약한다.
  컨테이너 nvidia-smi/torch/JAX에서도 장치 하나를 확인했다. 테스트는 실제
  Genesis와 MPPI 4096×2를 사용하며 DDS, 카메라, Robot을 실행하지 않는다.
  기존 `test` 인스턴스의 pin/보안 설정은 유지했다.
- 새 TDD는 이동한 몸통에서 뒤에 남은 지면 목표의 관절 도달 범위를 검증한다.
  해석적 FK/IK 및 동일 지면 투영 회귀는 5 failed/6 passed에서 green으로 전환.
  배포 URDF/Pinocchio FK 교차 검사와 보정 속도 제한/속도 feedforward 회귀를
  추가했다. 관절 여유 0.04 rad, 탐색 중 수평 목표 변화 상한 1.5 m/s이며
  힘/토크 한계와 접촉 timeout은 유지한다. 해가 없으면 기존 timeout으로 실패한다.
- canonical 개발 attachment의 Sim 전체: **563 passed, 3 skipped**.
  제외는 qpOASES 1개와 opt-in 실제 camera worker smoke 2개다.
  도달 범위 수정의 원격 발행 이미지 보행 비교도 위와 같이 미통과다.
  단위 테스트 성공은 보행 안정화의 증거가 아니다.

### URDF 영점 및 착지 목표 TDD (2026-10-14)

- 고정 몸통/공중 발 추종 실험으로 MPC·보행 위상·바닥 충돌을 제외하고 실제
  Genesis 조립 모델, 관절 물성, production `_torques` 경로를 검증했다.
  `profile_fixed_foot.py --legacy-reference`가 이전 기준점 오류를 재현한다.
- **확정 원인:** 빌드 전 `desc.init_qpos`에 stand 각도를 쓰면서 `qpos0`가
  변경됐다. Genesis FK/DOF는 `qpos - qpos0`, 제한 검사는 raw `qpos`를 사용한다.
  실제 기록에서 calf DOF -1.8 rad가 raw -3.3 rad가 되어 하한 -2.7227 rad를
  위반했고, 거의 정지한 calf에 약 -9 Nm가 입력돼도 제한 반력이 상쇄했다.
  Pinocchio의 중력 토크는 약 -0.15 Nm였다. 이는 샘플 수나 게인 문제의 증거가
  아니라 좌표 기준 불일치의 재현이다.
- 초기 자세 helper는 검증만 수행하고 URDF 기준점/물리 한계를 수정하지 않는다.
  Runtime, camera worker, RL scene 호출부를 함께 변경했다. 실제 자세는 기존
  build 후 설정 경로가 담당한다. Zero-neutral calf 범위 경고는 예상되며,
  이를 없애려고 기준점/한계를 바꾼 과거 기록의 접근을 철회한다.
- 동일 RL 발, dt=0.02, 고정 몸통 1초 유지 → 목표를 3 cm 하강 → 유지:
  최대 발 오차 **73.638 mm → 4.657 mm**, 최대 속도 **1.305 → 0.157 m/s**.
  로컬 RTX3050와 원격 A6000에서 동일 결과. 이 실험은 다른 세 발의 목표도
  고정한 공중 토크 제어이며 부유 몸통/보행 안정성을 대신하지 않는다.
- TDD: 영점 불변식 2개 red→green, 실제 물리 추적의 정착 오차 10 mm 및
  유지 구간 변동 2 mm gate red→green. Raw qpos 제한 좌표 회귀도 추가했다.
- 영점 수정 후 보행 계측에서 별도 착지 전환 오류를 확인했다. 미착지 swing의
  기존 지면 목표를 현재 공중 발 위치로 덮어써 탐색 종료 목표가 z=0.0348 m에
  남았다. 기존 touchdown을 유지하고 그 아래로 탐색하도록 수정했으며, 최대
  탐색 깊이에서 잔류 하강 속도 명령도 제거했다. 두 회귀 red→green; 관련
  adapter/contact suite 57 passed. 힘 60 N, 추가 하강 4 cm, 300 ms timeout은 유지.
- 최종 canonical 개발 attachment: `pytest workbench/tests/apps/sim
  workbench/tests/research/debug/test_fixed_foot_tracking.py
  workbench/tests/research/experiments/test_contact_walking_progress.py -q --tb=short -rs`,
  `ELESIM_FIXED_FOOT_TRACE=workbench/evidence/generated/contact-gait/fixed-zero-reference.jsonl`:
  **552 passed, 3 skipped**. 제외는 qpOASES 미설치 1개와 opt-in 실제 camera worker
  smoke 2개. 전체 required/extended 및 release build/verify는 이번 범위에서 미실행.
- **보행 gate는 미통과.** 최종 4096×2, 50 Hz, 명령 6초+정지 2초 시험에서
  전진 3.56 s / 후진 3.18 s / 좌측 5.26 s / 우측 4.10 s에 모두 착지 timeout.
  시간에는 초기 settle 2초가 포함된다. 방향별 이동은 +0.244 / -0.150 m X,
  +0.243 / -0.087 m Y. 이는 보행 안정화 성공이나 전 방향 성능 향상의 증거가 아니다.
  MPPI solve p95는 약 4.5–4.9 ms였다.
- 남은 전진 실패에서는 RL calf=-0.765 rad(신전 상한 -0.83776 rad를 넘어선
  soft-limit 상태), 발 z=0.053 m, 탐색 목표 z=0.011 m를 기록했다. 몸통 이동
  동안 뒤쪽에 남은 목표의 도달 가능성과 부유 몸통/지지력 전환을 다음에 검증한다.
  고정 몸통 결과만으로 높은 게인 하나를 원인으로 확정하지 않는다.
- 원시 결과는 Git-ignored `workbench/evidence/generated/contact-gait/`의
  `fixed-legacy-qpos.jsonl`, `fixed-zero-reference.jsonl`,
  `zero-search-remote-evidence.tgz`, `zero-search-summary.json`에 보관한다.
  원격 archive에는 두 수정 단계의 보행 JSONL/log와 사용한 소스 사본이 포함된다.
- 실제 Sim/Robot 인스턴스나 선택 릴리즈는 변경하지 않았다. 원격 실험은 기존
  `elesim-merry-dev` 내부 `/tmp/elesim-contact-tdd-20261013` 사본에서 DDS 없이 수행.
  MPPI 4096×2 및 기본 dt=0.02는 그대로다. 200 Hz 비교는 수용시험을 통과하지
  못했으므로 기본값에 반영하지 않았다.

### 접촉 기반 보행 TDD 및 원격 재현 (2026-10-13, 미완료 보행 gate)

- `75b3fcf` 이후 수정본에서 실패 테스트를 먼저 재현했다: 착지 끝에서도
  내려가는 swing 속도, 접촉 상실 tick의 다음 liftoff 진행, gait 정지에 따른
  startup torque/ready 보정 동결, 반복 충격 접촉의 미확인, 기존 swing의 공중 정지.
  끝점 속도 0의 quartic swing, 접촉 판정 선행, startup 시간 분리,
  80 ms 창 내 40 ms 접촉 증거, 발별 swing 진행으로 수정했다.
- 새 접촉 상태를 사용하는 테스트 fixture와 오래된 iteration=4 검사를 갱신했다.
  현재 canonical 개발 attachment에서 `pytest workbench/tests/apps/sim
  workbench/tests/research/experiments/test_contact_walking_progress.py -q --tb=short -rs`:
  **548 passed, 3 skipped**. 제외는 qpOASES 플러그인 1개, opt-in 실제 카메라
  smoke 2개다. 보행 성공을 의미하지 않는다.
- 로컬 개발 JAX는 CPU-only라 실제 GPU MPPI 실행을 거부했다. 원격의 기존
  `elesim-merry-dev` venv에는 JAX가 없어 Dockerfile과 동일한 JAX/CUDA plugin
  0.4.38 및 NumPy 1.26.4를 설치했다. 호스트 Python은 변경하지 않았다.
  `/tmp/elesim-contact-tdd-20261013`에 수정 Sim 소스를 분리하여 사용했고,
  원격 checkout이나 내려간 운영 Sim은 교체/재시작하지 않았다.
- 원격 A6000 GPU 1에서 실제 Genesis GO2+arm 장면, MPPI 4096x2,
  dt=0.02, settle 2 s, command 6 s, stop 2 s로 전후 +/-0.8 m/s,
  좌우 +/-0.35 m/s를 실행했다. 카메라/네트워크/물리 Robot은 포함하지 않았다.
  여러 접촉 전환 수정 단계에서 네 방향 모두 보호 상태로 종료하여 **수용 실패**다.
  20 ms 샘플에서 공중 RL 발 높이가 tick마다 약 2 cm 진동하며 실제 접촉력이
  0인 구간을 확인했다. 접촉 gate만으로 발 추종 진동을 해결하지 못했다.
- 임시 복사본에서만 PD 250/5, 추가 하향 preload 15 N, dt=0.005를 각각
  비교했다. 모두 완주 실패라 이 gain/preload/dt 변경은 source에 채택하지 않았다.
  단순 타임아웃 연장이나 보호 기능 해제로 통과시켜서는 안 된다.
- 진단 runner에 접촉 판정 입력/출력, 발별 진행률, sample-period 옵션과
  명령 방향 최소 진행량 검사를 추가했다. 정지한 채 버티는 것은 보행 통과가 아니다.
  원본 및 단계별 결과는 `workbench/evidence/generated/contact-gait/remote-evidence.zip`,
  요약은 같은 디렉터리 `summary.json`에 회수했다(생성 artifact, Git 제외).
- 남은 일: 실제 swing/search 발 추종과 관절 토크를 함께 검증하고 안정화한 뒤
  동일 전후좌우/정지 시험을 다시 통과시킬 것. 이 수정본을 보행 해결 완료로
  배포하지 않았다. 전체 required/extended/release gate는 이번 좁은 TDD 범위에서
  실행하지 않았다.

### 설치 파일 소유권 감사 (2026-10-13)

- `/home/user/ws/newsim`의 실제 bin 17개를 manifest와 대조했다.
  `elesim-instance-register`만 wrapper 목록에서 누락됐고 나머지 16개는
  기록과 SHA-256이 일치했다. prefix의 파일을 정확한 inventory와 managed,
  log, authority, external 경계에 대조했을 때 경계 밖의 추가 누락은 없었다.
  관리 루트 안의 동적 파일은 디렉터리 소유권으로 포함되며 개별 해시 검증을
  수행했다는 의미는 아니다. compute 실물 설치는 이번 감사에 포함하지 않았다.
- 컨테이너/native 생성 코드와 wrapper 목록을 대조하고, 새 `elesim-register`의
  설치 전 경로 충돌 검사에도 명시적인 claim을 추가했다.
- 미기록된 이전 wrapper는 자동 인수·삭제하지 않고 경로를 안내하며 보존한다.
  이 경우 업데이트를 계속하지만, 기록된 wrapper의 해시 불일치는 계속 거부한다.
- 컨테이너 UI/Sim/전체 역할 및 native Robot, 개발 attachment에 생성된 bin
  파일 전체와 manifest wrapper 목록의 일치 검사를 추가했다. 실제 설치 파일과
  manifest를 직접 수정하지 않았다.

### SROS2 생성 도메인 전달 수정 (2026-10-06)

- `test`의 UI/Pilot/Sim 로그에서 `Could not find domain 42 in governance`
  를 확인했다. 기존 서명 governance/permissions는 domain 0을 포함했다.
- Humble SROS2가 읽는 `ROS_DOMAIN_ID`를 topology의 domain_id로 각 발급
  subprocess에 명시한다. keystore, enclave, permission 생성에 동일하게
  적용하며 manager의 전역 환경은 변경하지 않는다.
- 평문 instance를 최초 SROS2 provision으로 교체한 journal에는 이전
  security_generation이 없는 것이 정상이다. 그 경우를 허용하되 기존
  SROS2 instance 및 rotate의 이전 generation 검증은 유지한다.
- 자동 테스트는 실행하지 않았다. 기존 잘못된 서명 정책은 소스 수정만으로
  갱신되지 않으며 새 generation의 회전 배포가 필요하다.
- 사용자 승인으로 기존 manager 환경에서 수정된 소스를 사용해 `test`를
  회전 배포했다. generation `g-20261006t112106637873z-feddb33c4c36`의
  operator/UI 및 compute/Pilot·Sim 서명 governance/permissions에서 domain
  42를 읽어 확인했다. 등록 트랜잭션은 정상 완료됐다. 발급 전 ROS 환경 누락으로
  실패한 시도는 동일 발급 완료 후 정규 recover로 복구했다. runtime 재시작과
  실제 DDS discovery는 이번 작업에서 실행하지 않았다. manager 이미지 자체는
  교체하지 않았으므로 영구 적용에는 수정 소스로 manager 업데이트가 필요하다.

### PyMPC 접촉 전환의 지지력/토크 필터 수정 (2026-10-06)

- 전진 시 앞발, 후진 시 뒷발이 끌리며 몸통이 기우는 증상에 대한 코드 조사:
  현재 `pympc` 경로는 roll/pitch 목표가 0이며 legacy `pitch_trim`을 사용하지
  않는다. 스윙 목표 위치·속도와 발 Jacobian은 이미 world-aligned다.
- 접촉 집합이 바뀔 때 재계산한 힘을 이전 접촉 집합의 힘과 섞는 지연을
  제거했다. 해당 tick은 새 힘 분배와 제한된 토크로 필터 이력을 시작하며,
  동일 접촉 집합에서는 기존 필터를 유지한다. 마찰·힘·토크 제한 및 시작
  ramp는 유지한다. acados와 jax_mppi의 공통 토크 경로에 적용된다.
- 소스 수정만 완료. 테스트 및 실제 보행 검증은 실행하지 않았다. 증상의
  확정 원인이나 보행 안정성 개선으로 간주하지 않는다. 접촉 전환 시 토크
  응답이 빨라지므로 실제 적용 후 피치, 발 높이와 착지 충격 비교가 필요하다.

### GPU 카메라 렌더링 / 인코딩 경로 수정 (2026-10-06)

- 열린 `test` 세션(`elesim-merry-test-sim`, GPU 2)을 읽기 전용으로 점검했다.
  실제 카메라 프로세스에 `swrast_dri.so`가 로드되어 있었고, 같은 컨테이너의
  EGL probe는 `Mesa / llvmpipe (LLVM 15.0.7, 256 bits)`를 반환했다.
  NVIDIA graphics/video 라이브러리는 있었지만 EGL vendor JSON은 Mesa 것만
  있었다. 따라서 해상도 자체나 GPU MPC 경쟁을 이 세션의 원인으로 단정하지 않는다.
- Sim runtime/release 및 dev Dockerfile에 `10_nvidia.json`을 등록했다.
  driver 설치 없이 런타임이 제공하는 `libEGL_nvidia.so.0`을 GLVND가 찾게 하며,
  무거운 의존성 layer 뒤에 두어 재빌드 범위를 줄인다. CPU host의 Mesa는 유지한다.
- 빈 Compose encoder override는 실제 Sim GPU/CPU 기본값을 따른다. 자동
  NVENC 감지는 `/dev/nvidia0`뿐 아니라 `/dev/nvidia2` 등도 인식한다.
  headless async/sync 카메라는 준비 전에 실제 GL vendor/renderer를 기록하며,
  GPU 요청에 소프트웨어 렌더러가 선택되면 오류를 낸다. 두 카메라 720p는 유지한다.
- 수정 소스와 Dockerfile에서 추출한 NVIDIA manifest를 별도 짧은 진단
  프로세스에 적용했다. Mesa manifest를 함께 남겨도 RTX A6000 / software=false,
  자동 encoder `h264_nvenc`, 720p 12개 입력에서 8,498 bytes 출력을 확인했다.
  이는 실제 GPU context/encoding 확인이며 full scene FPS/step 개선 측정은 아니다.
- 기존 배포 YAML의 GPU MPC 기본값 변경에 뒤처진 config test 기대값을
  `jax_mppi`로 맞췄다. 이번 수정은 실행 중 `acados` MPC 설정을 바꾸지 않는다.
- 기존 설치의 Compose dev 실행 경로에서 Sim 전체 + Dockerfile cache/shell
  검사 **554 passed / 3 skipped**. GPU 2 자동 encoder 선택, 빈 Compose override,
  소프트웨어 GL 거부와 context 해제 회귀를 포함한다. `git diff --check` 통과.
- 실행 중 세션과 설치 pin은 변경하지 않았다. 반영에는 수정 이미지 발행 및
  Sim instance 교체가 필요하다. 기존 지시대로 컨테이너 교체/삭제는 자동 수행하지
  않았으며, canonical required/extended 전체와 release 재빌드도 이번 범위에서는
  실행하지 않았다. 증거: `workbench/evidence/generated/readiness/20261006-live-performance/`.

### 선택형 GPU MPC (2026-10-06, 진행)

- 후속 사용자 요청으로 배포 Sim YAML의 `solver_backend` 기본값을
  `jax_mppi`로 변경했다. 아래 CPU 기본값 기록은 이 변경 이전 검증 이력이다.
  CPU `acados` 구현과 키 생략 시 코드 기본값은 보존하며 자동 CPU fallback은
  없다. 기존 설치/실행 인스턴스는 변경하지 않았다. 이번 기본값 변경에서는
  테스트와 이미지 재빌드를 실행하지 않았다.
- 사용자 승인 범위: CPU acados를 기본값으로 보존하고 GPU 병렬 MPC를 선택
  옵션으로 추가한다. 기존 R0–R5와 별도 작업이며 Robot 물리 제어는 변경하지 않는다.
- G1 인터페이스/설정: `robot.go2.locomotion.mpc.solver_backend`는 `acados`
  기본값 또는 명시적 `jax_mppi`다. 동일 controller의 gait/swing/force/torque/
  stop/reset 경계를 사용한다. GPU 실패 시 CPU로 자동 전환하지 않는다.
- G2 GPU 계산: Sim 소유 JAX SRBD/MPPI kernel에 mass/inertia를 매 solve마다
  전달한다. 후보 rollout을 병렬 평가하고 bounded sample/iteration budget,
  실제 simulation-time warm-start 이동, 초기 JIT, deterministic reset을 구현했다.
  upstream의 global configuration/fixed-inertia closure를 그대로 사용하지 않는다.
- G3 의존성: Python 3.10/NumPy 1.26.4용 JAX 0.4.38과 CUDA12 plugin/pjrt
  0.4.38을 Sim/dev 이미지 정의에 추가했다. CPU 모드는 CUDA plugin을 설치하지
  않는다. ONNX와 호환되도록 ml_dtypes는 0.5.4로 맞췄다. 실제 Sim CUDA 이미지
  검증은 아래에 기록하며 dev CUDA 이미지 자체의 재빌드는 이번 범위에서 하지 않았다.
- 정식 dev에서 기존 CPU adapter 및 GPU 계약/CPU JAX 수학 검증 **55 passed**.
  Sim 전체 회귀는 **507 passed / 3 skipped**. 이는 실제 GPU kernel, 보행,
  Docker release 검증의 증거가 아니다.
- source `67d8b1e`에서 정식 required gate 통과: Protocol 141, Robot 104,
  Pilot 406/21 skipped, Sim 507/3 skipped, UI 79, model/release 82,
  실제 four-process DDS smoke, DDS RGBD 2, WebRTC 2, setup 1141/3 skipped.
  extended 22+10+4+10 통과, release build 및 isolated verify 통과.
  이후 terminal cost/설정 validation 보완은 별도 focused 32 passed다.
- A6000 GPU 0의 격리 venv에서 실제 CUDA 실행 확인. 합성 상태/접촉/payload
  100회(초기 20회 제외) solve p50/p95: CPU **15.51/26.38ms**, GPU
  **5.18/6.26ms**, GPU 초기 JIT **5.70초**. CPU/GPU 출력은 동등하지 않으며
  보행·미디어 성능 증거가 아니다. 기존 CPU Sim이 같은 GPU에서 실행 중이었다.
- 초기 GPU 보행(terminal cost 보완 전)은 전진 15초/정지 5초×2, 총 42초를
  전도/fault 없이 stand로 완료했다. 이동 6.75m로 CPU보다 추종이 부족하여
  명목 acados와 같은 terminal state cost를 추가하고 재비교 중이다.
  evidence: `workbench/evidence/generated/readiness/20261005-gpu-mpc/`.
- terminal-cost 후보 7회와 CPU 기준선 1회는 모두 전도/fault 없이 stand로
  완료했으나 GPU의 전진 속도·yaw 추종이 CPU보다 나빴다. 계산 시간 개선만으로
  보행 동등성을 주장하지 않는다. weighted mean/반대 부호 sampling 후보를
  비교 중이며, 다음 후보는 몸체 wrench를 탐색한 뒤 접촉 발에 분배한다.
  이 단계의 wrench 수학/경계 focused gate는 **14 passed**였다.
- 후속 wrench 후보의 방향별 7회와 camera 1회는 stand로 완료했지만 전진
  추종이 부족했다. terminal cost 제거/속도 가중치 증가/반복 수 4의 후보를
  비교했다. `running-high` 단일 trial은 8.34m 이동, steady body vx 0.284m/s
  (명령 0.35, CPU 기준 0.293), solve p50/p95 6.81/8.35ms였다.
  단일 결과로 동등성을 판정하지 않고 4방향 반복, 팔 동작, camera를 검증 중이다.
  GPU 기본 반복 수를 4로 조정하며 CPU backend 기본값은 유지한다.
- float32 rollout 전에 float64 world-origin rebase를 수행해 큰 좌표에서
  발 lever arm 정밀도를 보존한다. translation-invariance 회귀를 포함한
  GPU 수학/설정 focused gate **33 passed**. 이는 실제 GPU 보행 증거와 구분한다.
- 이 후보의 정식 required gate 통과: Protocol 141, Robot 104,
  Pilot 406/21 skipped, Sim 516/3 skipped, UI 79, model/release 82,
  four-process DDS smoke, DDS RGBD 2, WebRTC 2, setup 1141/3 skipped.
  로컬 `dist/gpu-mpc-20261006/releases` 생성 및 네 역할 isolated verify도
  통과했다. 이는 아래 실패한 원격 Docker 이미지 build와 별개의 검증이다.
- `00acf7d` 후보: 전진/횡이동/회전/후진 각 2회, 팔 동작 2회, camera 1회,
  **11/11**이 42초 trial을 전도/fault 없이 마지막 stand로 완료했다. 각 trial은
  settle 2초, 이동 15초/정지 5초×2이며 trial 사이 reset한다. 평면/고정 target
  제외/Robot·DDS 없는 기존 격리 venv 실험이다. 팔은 theta1 ±0.12rad,
  theta2 ±0.08rad/8초 사인 명령으로 동작하며 inertia trace 변화를 기록했다.
  전진 steady body vx는 0.286–0.288m/s(CPU 0.293), 횡이동 vy는
  0.087m/s(명령 0.25), 회전 wz는 0.429–0.433rad/s(명령 0.8)다.
  전진 yaw-rate RMSE는 0.368–0.394rad/s로 CPU 기준 0.221보다 높다.
  따라서 무전도/속도 개선을 모든 방향 추종의 동등성으로 해석하지 않는다.
- 두 camera 720p/target 30Hz 동시 실행: 42초 simulation의 wall time은
  CPU 74.11초 → GPU 57.06초, solve p50/p95는 25.70/34.32ms →
  8.10/11.92ms, 이동 step p95는 64.53ms → 46.17ms였다.
  hand-eye/observer 관측 fps는 CPU 13.17/12.29, GPU 13.81/11.58이며
  capture-to-observation p95는 약 61.6ms → 48.4/48.6ms다.
  기존 live CPU Sim과 GPU를 공유한 비교이며 WebRTC/network/UI 지연이나
  실제 30fps 달성 증거가 아니다. camera 원시 기록도 해시 검증하여 보존한다.
- extended gate는 22+10+4+10 및 실행된 7개 critical mutation 모두 통과했다.
- CPU 추가 기준선(횡/회전/후진/팔 각 1회)도 **4/4** stand로 완료했다.
  동일 steady 구간의 CPU/GPU body 속도는 횡 0.079/0.087m/s,
  후진 -0.180/(-0.179…-0.183)m/s, 팔 동작 전진 0.308/(0.289…0.292)m/s다.
  회전은 CPU 0.622/GPU 0.429–0.433rad/s로 GPU가 더 부족하여 yaw 가중치
  800/2000 후보를 추가 비교했다. CPU 기준선은 각 1회이므로 통계적 동등성
  또는 재현성 보장은 아니다.
- yaw 가중치 800/2000의 전진·회전 각 1회, **4/4** stand로 완료했다.
  2000 후보는 전진 vx 0.286m/s를 유지하며 회전 wz 0.521rad/s,
  전진 yaw-rate RMSE 0.337rad/s였다. 800 후보의 전진 vx 0.279,
  회전 wz 0.503보다 양호해 2000을 선택했다.
  yaw 수정 후 Sim 회귀 **516 passed / 3 skipped**, 새 로컬
  `dist/gpu-mpc-yaw-20261006/releases` 생성 및 네 역할 isolated verify 통과.
- 최종 yaw 설정의 추가 전진 2회/횡/회전/후진/팔/camera 각 1회,
  **7/7** 모두 42초 trial을 stand로 완료했다. 전진 vx 0.275–0.282m/s,
  yaw-rate RMSE 0.318–0.350rad/s, 회전 wz 0.508rad/s였다.
  전진 속도는 이전 200 가중치보다 조금 낮고 yaw 흔들림/회전 추종은 개선됐다.
  최종 횡 vy 0.080, 후진 vx -0.171, 팔 동작 전진 vx 0.277m/s로,
  특히 팔 동작 전진과 회전은 CPU보다 부족하다. CPU 기본값은 유지하며
  GPU를 보행 동등성이 입증된 대체품으로 표시하지 않는다.
  최종 camera trial: wall **54.09초**, solve p50/p95 **7.84/10.70ms**,
  이동 step p95 **43.86ms**, hand-eye/observer **14.72/11.09fps**,
  capture-to-observation p95 **47.30/45.67ms**. 앞의 공존 부하/비전송 계측
  한계는 동일하다. 최종 원시 기록을 포함한 20개 후보/CPU 파일의 복사와
  SHA256 검증을 완료했다. 근거는 `raw/candidate-manifest.json`,
  `tracking-analysis.json`, `final-source.json`에 있다.
- 원격 이미지 build는 source `446cd08`에서 ENOSPC로 실패했다. 현재 원격
  여유 공간은 최초 약 6GiB였고 이후 읽기 전용 조회에서 약 11GiB로 증가했다.
  증가 원인은 확인하지 않았으며 이번 작업에서 삭제하지 않았다. 검증 전용
  venv는 약 4.5GiB다. 기존 CPU runtime은
  유지되며 새 GPU 이미지는 아직 발행/적용하지 않았다. 사용자는 삭제를 직접
  수행하기로 했으므로 image/container/cache 삭제 및 prune을 실행하지 않는다.
  `elesim-update`/instance `up`도 자동 image collection이 있으므로 그대로
  재실행하지 않는다. 공간 확보와 삭제 없는 build 경로 확인이 선행되어야 한다.
- 검증 종료 뒤 read-only 점검: free **10.27GiB**, 진단 프로세스 **0**,
  `/tmp/elesim-gpu-mpc-20261005/venv` **4.47GiB**이며 symlink가 아니다.
  현재 CPU Sim은 이 venv를 사용하지 않는다. 사용자에게 상대 컴퓨터의
  `elesim-merry-readiness-sim` 안에서 이 정확한 venv만 직접 삭제하도록
  명령을 전달했다. 사용자가 직접 삭제한 뒤 경로 부재와 free **14.73GiB**를
  확인했다. 이미지/container/volume 삭제나 prune은 실행하지 않았다.
- source `6740aa6`의 원격 build를 재개했다. 설치된 update/release wrapper의
  사본에서 정확한 image collector 호출과 task container `--rm`만 제외하며,
  원래 wrapper/ownership/Engine guard/installation lock/publication 검증은
  유지한다. bootstrap script는 pinned commit의 파일 hash를 확인했다.
  task setup/publication container는 남겨 두며 직접 제거하지 않는다.
  free 2GiB 미만이면 이번 build process group만 종료하는 감시를 추가했다.
  현재 CPU readiness container와 instance release pin은 유지한다.
- 원격 build/발행 **exit 0**, build 506.9초. 소스 `6740aa6`, 릴리스
  `fb441a6e3442e20144999933661920e2b8d636308e0366fd33068b6111e9710e`,
  Sim `elesim/sim:merry-shark` / image `sha256:8d7aceb046b3…` / 15.08GB.
  Pilot은 `merry-shrimp`, tools는 기존 `merry-ferret` 재사용이다.
  종료 시 여유 공간은 약 19.06GiB였으며 guard 중단 없이 완료했다.
- 새 이미지에서 network=none/GPU 0 진단 **exit 0**: CPU acados 기본값과
  실제 solve, CPU 경로의 JAX 미초기화, Torch CUDA tensor, JAX CUDA12 plugin,
  최종 kernel SHA256 일치, deterministic reset, 동적 mass 및 접촉력 제한을
  확인했다. GPU 합성 100회 중 초기 20회 제외 p50/p95 **7.86/10.58ms**,
  초기 JIT **5.31초**다. Torch 2.12.1/CUDA13과 JAX 0.4.38/CUDA12 plugin이
  이 이미지에서 함께 실행됐다. 실제 장면/영상 성능 증거와는 구분한다.
  기존 readiness Sim/Pilot의 이미지 ID와 running 상태가 유지됨을 확인했다.
- 새 이미지 + 새 릴리스 모델을 사용한 network=none/GPU 0 장면 검증도
  **cold/warm 2/2** stand로 완료했다. 전진 15초/정지 5초×2 동안 팔을
  움직이고 두 1280×720/target 30Hz renderer를 함께 사용했다. 설정/모델은
  read-only mount이며 DDS publisher를 생성하지 않는다. 물리 scene build는
  cold **113.10초**, warm **5.35초**; warm camera build는 hand-eye **1.91초**,
  observer **2.95초**였다. GPU JIT는 각각 **4.90/4.79초**다.
  cold trial wall **75.54초** / 관측 fps **5.89/7.12**, warm trial wall
  **59.59초** / 관측 fps **13.91/7.20**. warm solve p50/p95 **7.61/10.62ms**,
  이동 step p95 **42.14ms**, capture-to-observation p95 **48.34/47.55ms**다.
  실제 30fps 또는 WebRTC/UI 지연 수용 완료를 의미하지 않으며, 기존 CPU
  camera 기준선은 팔 동작이 없었으므로 이 조합과 직접 동등 비교하지 않는다.
  cold 종료 때 한 camera worker의 Quadrants cache lock 경고가 있었지만
  warm 실행에서는 physics/camera cache 재사용과 정상 종료를 확인했다.
  raw cold/warm 기록을 각각 보존하고 SHA256 검증했다.
- 최종 free **19.01GiB**. setup/publish/image-probe/scene 네 task container는
  모두 exit 0으로 남겼으며 합계 writable layer 약 **52MB**다. 기존
  readiness Sim/Pilot은 계속 running이고 이미지 ID도 그대로다. 사용자
  지시대로 image/container/volume 삭제나 Docker prune은 실행하지 않았다.
  근거: `published-image.json`, `image-build-no-cleanup.log`, `image-probe.log`,
  `new-image-scene*.log`, `raw/new-image-scene*.jsonl`,
  `final-container-inventory.jsonl`.
- 진단 runner에 backend 선택, solve/step timing, 선택적 두 renderer 부하를
  추가했다. renderer 진단에서는 DDS publisher 생성만 차단하며 실제 visual
  workers/dispatch를 사용한다. WebRTC encoding/network 수용시험은 별개다.
- G4 남은 수용: 발행한 새 릴리스를 실제 readiness에 적용하고 선택적 GPU
  backend의 WebRTC/UI 경로를 검증한다. 기존 container 교체/삭제는 사용자가
  직접 수행한다는 현재 지시를 따른다. 추가 payload와 다른 지형/속도에
  일반화된 안정성은 미검증이다.
  현재 실행 중 readiness와 기본 CPU 선택은 바꾸지 않았다.

### 720p / 30fps 요청 (2026-10-05, 적용 완료)

- 사용자 요청에 따라 두 camera의 해상도 1280×720을 유지하고 remote 목표
  cadence를 10→30 Hz로 변경한다. observer의 공통/schema 기본값도 30 Hz다.
- 최신 프레임만 유지하는 큐와 wall/simulation cadence 제한은 유지한다.
  설정 30 Hz는 실제 새 영상 30 fps 보장이 아니다. 이전 아래 10 Hz 측정과 구분한다.
- source `86e3f92`, 원격 릴리스 `7ff371b6…`를 기존 readiness에 적용했다.
  실행 중 Sim의 merged config에서 두 영상 모두 1280×720 / target_fps=30을 읽었고,
  실제 UI decoder의 두 1280×720 영상과 LIVE 복귀를 확인했다. 실제 30fps 처리량은
  계측하지 않았다. evidence: `workbench/evidence/generated/readiness/20261005-camera-30fps/`.
- 기존 config 회귀의 기대값을 수정했다. 이번 설정 변경에서는 테스트를 실행하지 않았다.

### Respawn / 720p / MPC 후속 (2026-10-05)

- 범위는 기존 Robot-free readiness의 Sim/Pilot/UI다. 실제 Robot motion과
  전체 R0–R5 acceptance 완료를 의미하지 않는다. runtime 수정 source는
  `4023f0e`, 원격 릴리스 `cac8df61…`를 적용했고 최종 설치 검증을 완료했다.
  당시 검증한 Sim image ID는 `sha256:06a67c35…`이며 로컬 UI 기존 릴리스 핀은 유지했다.
- **Respawn**: MPC reset 이후 남던 camera root/leg mirror와 SimScene pose cache를
  초기화하고 실제 reset pose를 한 번 읽는다. MPC가 idle/paused여도 초기 장면을
  제출한다. 정상 정지 후에는 기존 telemetry의 root readback을 재사용하고 idle
  leg pose만 feedback cadence에서 읽는다. camera 제출 자체는 GPU readback이 없다.
- `705b2ce` 설치 이미지에서 UI 버튼과 동일한 OperatorSession 경로로 reset:
  **0.167초**에 epoch=1/t=0.02 상태, **0.442초**에 성공 응답을 받았고 초기
  자세 영상 복귀를 확인했다. 이 수치는 DDS 상태/응답 지연이며 실제 버튼 클릭부터
  화면 갱신까지의 계측값은 아니다.
- **720p**: observer/hand-eye 기본값을 모두 1280×720, remote cadence 10 Hz로
  설정했다. RTX A6000 GPU 0의 두 renderer 비교(18초 중 초기 6초 제외)는
  640×480→720p에서 observer **10.00→9.83 fps**, hand-eye **10.00→10.00 fps**였다.
  두 실제 WebRTC decoder의 1280×720과 두 영상 LIVE도 확인했다.
- Sim GPU Compose에 누락된 NVIDIA `video` capability를 추가했다. 실제 이미지의
  720p 이동 패턴 60-frame encode는 **h264_nvenc**, fallback=false,
  warm p50/p95 **2.36/4.26ms**였다. 이전 libx264 p95는 26–29ms였다.
  renderer 비교는 정지 장면의 별도 프로세스이며 전체 네트워크 성능 증거는 아니다.
- **MPC baseline**: 통신 없는 동일 GO2+arm/평면(dt=0.02)에서 vx=0.15는
  t=3.5에 pitch 약 27°, t=8.2에 roll 약 -58°로 전도했다.
- 채택한 수정:
  - 명목 착지 폭에 빠졌던 hip 회전의 수직 leg 투영(발당 약 26mm)을 보정하고
    bundled URDF의 Pinocchio FK와 비교한다.
  - MPC와 swing controller가 같은 liftoff touchdown을 유지한다. swing 위치와
    일치하는 속도 참조를 사용하고 stance 보조 감쇠/이전 지지 토크 필터를 섞지 않는다.
  - 접촉 위치 warm start를 켠다. 기존 force/torque/friction 한계는 유지한다.
  - 매 tick zero command가 `is_idle(0)`의 strict comparison 때문에 stop dwell을
    초기화하던 결함을 수정한다. 현재 swing 이후 네 발로 균형을 잡으며 실제 속도와
    기울기가 안정돼야 stand로 전환한다. 재출발 때 estimator/solver의 이전 상태를 지운다.
  - 평면 착지 후보는 yaw 기준으로 계산하고 COM 속도 오차의 0.1초분을 축당
    ±4cm로 제한해 보정한다.
- **최종 source GPU 반복**: settle 2초 후 이동 15초/정지 5초를 reset 없이 두 번
  반복한 42초 trial에서 **전진 vx=0.35 2/2, 횡이동 vy=0.25 4/4,
  회전 wz=0.8 3/3, 후진 vx=-0.35 1/1** 모두 전도/fault 없이 마지막 stand에 도달했다.
  고정 perception target을 뺀 평면, 카메라/DDS 없는 독립 프로세스 결과다.
  이전 후보의 횡이동/회전 실패 기록도 보존한다.
- **실제 설치 검증**: UI 버튼과 같은 operator 경로로 vx=±0.25의 전진 5초→
  정지 4초→후진 5초→정지 4초를 두 번 수행했다. 영상 두 개 LIVE/1280×720,
  정지 및 Respawn 후 초기 자세와 기본 target 복귀를 확인했다. 고정 target은
  검증 중 보행 경로 밖 (2,2,0.2)로 옮겼고 마지막에 (0.8,0,0.2)로 복구했다.
  첫 회 최대 roll/pitch는 6.3°/12.3°, 두 번째는 7.3°/3.3°였다.
  첫 진단은 모든 동작/복구 ACK 이후 종료 코드 143을 반환했다. 명령 없는
  최소 close probe와 전체 두 번째 검증은 정상 정리 로그 및 exit 0이었다.
- 최종 설치 이미지의 720p NVENC 재검증: fallback=false, encode p50/p95
  **2.48/4.45ms**. 새 이미지 첫 시작은 physics build **101.48초**,
  hand-eye/observer build **20.21/20.83초**였다. 이전 동일 이미지 warm-cache
  측정과 구분하며 소스 변경 첫 실행까지 30초라고 주장하지 않는다.
- **남은 한계**: 전진 displacement는 8.71–8.74m(30초 명령 적분 10.5m),
  횡이동은 2.15–2.49m(명령 적분 7.5m)로 속도 추종이 부족하다. 횡이동 최대
  roll은 trial별 8.3–23.1°, pitch 4.2–11.8°다. 다른 속도/반대 회전/지형/arm pose/
  payload에 일반화된 보행 안정성을 주장하지 않는다.
- 기존 prebuilt 3-iteration cap의 status=2(MAXITER)는 여전히 발생한다. 최대
  10회 재solve 후보는 p95 약 98ms이고 빠른 전진도 해결하지 못해 채택하지 않았다.
  채택한 warm-start 계열의 이전 solve p95는 약 28ms였다. 최종 반복의 42초
  simulation은 약 50–56초 wall time이었고 동시 runtime/빌드 부하도 있었다.
  완전 수렴이나 1× 실시간 보장을 의미하지 않는다.
- **소프트웨어 검증**: 최종 Sim **494 passed / 3 skipped**. Respawn/720p 단계의
  Sim+Compose 생성 검증은 **559 passed / 3 skipped**였다. 이 변경으로 전체
  required/extended/release/hardware gate 완료를 새로 주장하지 않는다.
- `workbench/research/debug/profile_sim_walking.py`는 production 모델/MPC/reset을
  사용하며 `--cycles`로 respawn 없는 재출발을 검사한다. pose stage/base twist/
  displacement/roll-pitch를 JSONL에 기록하고 전도/fault/미완료 trial은 exit 1이다.
  evidence: `workbench/evidence/generated/readiness/20261005-respawn-video-mpc/`.

### Sim startup 캐시 비교 (2026-10-05, 부분완료)

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
- 당시 재현에 사용한 축소 scene `profile_sim_startup.py`는 현재
  [workbench archive](../workbench/archive/README.md)에 보관했다.
  `--performance-mode`는 static, `--profile`은 비용 분석용이었다.
  현재 Sim probe의 startup 기록과 측정 범위는 `benchmarks.md`를 따른다.
- 원본: `workbench/evidence/generated/readiness/20261005-startup/`.
  이 축소 scene 결과를 실제 전체 Sim의 94초와 직접 비교하지 않는다.
  아래 최종 전체 application 결과를 별도로 기록한다.
- 기존 생성 dev 환경의 Sim 전체 + quality: **496 passed / 3 skipped**.
  기본 dynamic 설정, static 명시 선택, 잘못된 설정 거부와 CPU 경계를 포함한다.
  fresh dev build나 MPC 보행 수용시험 결과는 아니다.
- `368d044` 실제 원격 배포는 첫 build 100.51초, 같은 컨테이너의 프로세스
  재시작 74.90초였다. 첫 배포 후 두 영상 LIVE를 확인했지만 축소 실험의
  5초 수준은 재현되지 않았다. `elesim-instance down/up`은 컨테이너를 유지한
  stop/start였으며 container recreation 증거로 취급하지 않는다.
- 추가 원인: Genesis `destroy()`가 정상 interpreter exit에서 cache를 저장하나
  현재 Docker SIGTERM 경로와 multiprocessing `os._exit`는 이를 건너뛸 수 있다.
  실행 중 `dump_cache_data_to_disk()` 직접 호출은 GPU probe에서
  `free(): invalid pointer`로 실패해 폐기했다. 운영 앱에 이 경로를 배포하지 않았다.
  Docker SIGTERM이 Python finally를 실행하게 하고 physics/camera 종료에서
  지원되는 `gs.destroy()`를 호출하는 경로로 보완한다.
  중간 checkpoint안의
  unit test 498 passed는 폐기한 API의 GPU 안전성 증거가 아니다.
- 지원되는 종료 경로로 바꾼 최종 Sim + quality 검사도 **498 passed / 3 skipped**.
  SIGTERM의 finally 실행과 기존 handler 복원을 검증했다.
- **최종 live 결과 (`cf845da`)**: remote source를 정식 update/등록/start로
  적용했다. release `4b669db4c184b46c5e352cc6d0bfee2c5d194402dab7e86007b5cadda07a30b6`,
  Sim `elesim/sim:merry-ibis`, image
  `sha256:df4c7f1ad4eddfb0a3b03653c8207b37c9cb02c202a87cfb528edcd042d3acb4`.
  기존 UI release는 유지했다. 첫 build는 97.41초로 여전히 느리다.
- 정식 scoped `down` 뒤 Sim **exit=0 / OOM=false**를 확인했다. exact project와
  image를 검증한 후 정지된 해당 Sim 컨테이너만 제거하고 scoped `up`으로
  새 컨테이너를 만들었다. 이 실행에서 physics build **12.22초**, GO2 controller
  초기화 **1.52초**, hand-eye/observer build **2.43/2.61초**를 확인했다.
  비교되는 첫 실행은 각각 97.41초, 8.09초, 20.80/20.90초였다.
- Genesis backend 요청 로그부터 scene/media readiness까지 **180.02 → 31.66초**.
  container 시작 전 관리 작업, Python import와 UI 수신 지연은 이 구간 밖이다.
  이전 static full-scene 재실행 94초 안팎과 비교하면 physics build는 크게
  줄었지만, 장기 throughput/보행 정확도 동일성을 증명한 결과는 아니다.
- 실제 UI에서 observer/hand-eye **2/2 LIVE**, epoch=0 running t=59.80 →
  143.18 증가를 확인했다. 증거는 `final-first.log`, `final-warm.log`,
  `final-recreate.log`, `camera-final-warm.png`, `camera-final-running.png`,
  `shutdown-tests.log`다. 첫 실행 컴파일 비용, 강제
  종료/정리 timeout으로 cache를 못 쓴 경우, 다른 GPU/모델의 재컴파일은 남는다.
  MPC 보행 수용시험이나 다른 R 마일스톤의 완료로 확대하지 않는다.

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

### Hugmap standalone HTML reader (2026-11-03)

JSON reader UI moved from Tk to `workbench/research/support/hugmap_reader.html`. Offline browser file input supports v1 maps and v2 diameter tables, a stored-diameter slider, shared color ranges, 3D rotation/zoom/picking and display-only circle symmetry. Python launches the sibling HTML; `--read` prints the JSON path for manual browser selection. Solver/storage unchanged. Host reader/table tests: 17 passed. Node checks exercised real legacy JSON, symmetry expansion, table slider, empty maps, validation and canvas rendering with a stub DOM. Actual browser interaction was not verified; no browser tool available.

### CAD-confirmed housing centering (2026-11-03)

Corrected the housing mesh in ZED Mini and D435 by URDF Y=-0.0015 m,
restoring +/-65 mm about roll. ZED Mini COM Y, assembly/combined/planning
URDF inertial origins, bundle fingerprints, planning sphere centers and
source/artifact hashes are updated. Intrinsic inertia and roll height
155 mm are unchanged. Planning sphere translation preserves the original
mesh enclosure; all housing X-separation proofs remain unchanged because
neither X geometry nor joints changed. This is an exact translation of the
existing cover, not a fresh voxelization (host trimesh is unavailable).
The D435 COM was already Y=0; its unrelated historical inertia calibration
was not changed. Pilot IK context was regenerated to a temporary file and
matched the shipped context exactly. Both profiles' URDFs regenerated from
blueprints produce COM Y=0.

Updated repository newhug/hugmap plus /home/user/ws/newhug.py; hugmap 0.4.1
now has x=0 on roll. Old experiment JSON remains evidence for old geometry;
resume rejects incompatible geometry and HTML labels the legacy offset.
Regression coverage checks both meshes/roll axes, corrected COM, planning
hashes and sphere vertex coverage, embedded research geometry and rejection
of old table checkpoints. Host model suite: 28 passed, 1 skipped (trimesh).
Hugmap: 27 passed. Protected-body suite: 17 passed. HTML Node checks passed.
Canonical dev/ROS/GPU/physical gates were not run: no known generated dev
wrapper is available. No deployment or physical motion performed.

### Hugmap quadrant search (2026-11-03)

Hugmap 0.5.0 enumerates x<=0, y>=0 inside 500 mm. Default 20 mm grid:
516 placements per diameter, 21,156 across 41 diameters. JSON stores explicit
region and circle roll-reflection metadata (+90 computed, -90 reflected;
seg1/seg2/mu unchanged). GUI and HTML reader label the region. This chooses
roll per side and does not promise every old fixed-roll semicircle solution.
Old tables remain readable; resume validates region and symmetry metadata.
Theta handling and worker parallelism are unchanged. Host hugmap suite and
HTML Node validation used; no physical motion or full table generation.

### Hugmap reader removal (2026-11-03)

Removed the Python GUI reader button, --read CLI, browser launcher and obsolete Python visualization helpers. HTML remains independently usable. SavedMapData only validates saved checkpoints for resume.


### RL playground extraction (2026-11-05)

Implemented: generic `elesim_sim.rl` scene/runner/command timing/trajectory/replay
APIs; cylinder-specific Wrap code and its regression suite moved to
`workbench/research/experiments/wrap_grasp` and the matching research test tree.
The old required Wrap checks remain a required `wrap-grasp` quality gate.
Tracked originals were archived before removal; no runtime depends on that ZIP.
Bootstrap now includes the common RL library and still excludes all workbench
experiments. Existing Pilot policy export contracts were not changed.
Terminal waypoint/state capture now occurs before automatic episode reset.

The external Hug experiment can prepare a six-anchor local pilot from v3 maps
and generate fixed-D, fixed-roll barycentric residual trajectory candidates.
It refuses extrapolation, context/branch mismatches and un-replayed anchors.
Candidates explicitly do not claim physics validation. This completes library
extraction and offline experiment preparation, not the two-band RL experiment.
The dedicated two-band physical MDP, calibrated map-to-world placement, new
training/collection driver and hold/tug replay adapter are still outstanding.

Host evidence: 40 tests passed for command timing, terminal snapshots, replay,
trajectory roundtrip, local pilot splits, interpolation and migrated Wrap
configuration/progress; bootstrap suite 113 and quality routing/UI wrapper suite 13 passed. Python compilation and
`git diff --check` passed. The full migrated Torch-dependent task suite and
canonical required/extended/release/GPU gates were not run: host lacks Torch,
Genesis, tensordict and rsl_rl, and no usable generated development attachment
is known for this checkout. No dependencies were installed into host Python.
The requested current `kyung.json` is absent from `/home/user/ws`; the available
`gaga.json` is schema v2 with old fixed-roll semantics and was not substituted.
No training process, deployment, policy artifact or original map was modified.


### Two-band RL execution paths and home runbook (2026-11-05)

This supersedes the incomplete-physics-task boundary in the extraction entry
above. The user requested executable paths and instructions, explicitly not an
actual training run. The full runbook is [hug-rl-experiment.md](hug-rl-experiment.md).

Implemented the external Hug MDP and configure/check/train/collect/replay/
interpolate/validate commands. The task fixes roll/linear, controls two segment
angles, uses exact requested cylinder diameter and explicit mass/height/friction,
and evaluates two-role contact plus a timed kick/hold criterion. It creates a
local robot URDF with physical bend/roll limits, fixed zero-opening jaws and
camera collision geometry, leaving the source model unchanged. URDF/map frame
conversion, pre-training FK checks, solver-state resets, finite-state checks,
per-tick command recording and original-path replay are wired. A low-friction
ungrasped control must fail the retention criterion during physics preflight.
PPO/Genesis API usage was checked against the pinned upstream source, not
validated by executing those dependencies here.

`/home/user/ws/hugmap/map.json` was copied without modifying the original to
`workbench/evidence/generated/hug_interpolation/input/map.json`; SHA-256
`c28a6f9535e8859fa400300903d145ca6d9663ef805db83d04a2fe9f1782b1ee`.
Prepared 150 mm train/validation/test split is 6/22/2 across two local triangles.
Ready pilot/experiment inputs are in the sibling `home-ready/` directory. These
inputs/results are ignored by Git and must be transferred separately if needed.

Current evidence: 56 host tests passed, Python compilation and diff whitespace
checks passed. The real input passed offline configuration and digest checks.
Host dependency preflight exits 2 with Torch, Genesis, tensordict and rsl_rl
missing, as expected. No host dependencies were installed. No generated dev
wrapper is known for this checkout. GPU smoke/learning/collection/physical
replay and canonical required/extended/release gates remain unrun. No learning
success or physical interpolation success is claimed. No deployment, hardware
motion, commit or push was performed.


### Publishable RL example (2026-11-05)

The repository README now links to the Hug learning example. Its English README
contains setup, physics check, training, original replay and interpolation
commands. `example_pilot.json` includes the actual 30-case, 150 mm subset with
source digest and train/validation/test splits (6/22/2); no full map download is
required for that example. The 82 MB source map, generated experiment files,
checkpoints and training outputs remain untracked. The bundled-input regression
brings the focused host suite to 57 passing tests. GPU execution remains unrun.


### Hug NPZ policy and trajectory artifacts (2026-11-05)

Switched trajectory writes and directory-based interpolation/validation to
compressed NPZ v2. Numeric arrays retain float32 commands; metadata/diagnostics
are Unicode JSON strings, loaded with pickle disabled. Legacy JSON trajectory
reads remain available for explicit conversion. Reports/configuration remain JSON.
Added separate Hug policy NPZ export and `collect --policy`: the policy stores
ELU MLP weights, normalization, observation names and case/context metadata.
Export performs a NumPy/Torch parity check before publication. The seven-input,
two-output Hug contract is not the old Wrap hardware policy contract.
Example commands now export policy NPZ before collecting/replaying trajectory
NPZ. Host tests cover roundtrip/dtype/no-pickle, legacy conversion, distinct
artifact schemas, normalization/ELU inference and split routing. Actual Torch
export parity and Genesis policy execution remain unrun on this host.

NPZ-focused plus existing configuration/progress host regression suite: 64 passed.
The unexecuted example configuration was regenerated under `home-ready-npz/`;
the earlier generated inputs were preserved.

### C-hug evolutionary PPO population — 2026-11-07 (not launched)

- `hug_c/experiment train` defaults to an independent four-policy population;
  `--strategy single` keeps the baseline. Total 512 rollout allocation is four
  sequential 128-environment members sharing physics, not 512 concurrent robots.
  Home, free four-axis actions, q-only hint, rewards and lift/hold remain unchanged.
- `hug_c/population.py` adds stagnation-triggered single-member replacement,
  checkpoint rewind, exploration-floor perturbation, output-bounded actor mutation,
  niche-local strategy archives, physical admission checks, immutable generation
  checkpoints/NPZ, historical champion publication, and complete-generation resume.
  Novel safe approach strategies may survive weaker global fitness; PPO rollout
  buffers and optimizers remain independent. Evaluation costs are counted separately.
- Reviewed Bai et al. EvoRL survey (arXiv:2303.04150), original ERL, PBT,
  PGA-MAP-Elites, Safe Mutations and the survey's Supe-RL comparison. References,
  departures, descriptor bias, launch command and limitations are in
  `workbench/research/experiments/hug_c/README.md`. This is a task-specific hybrid,
  not a reproduced paper algorithm or evidence of escaping the observed stall.
- Local focused host suite passed 65 tests with 3 Torch-dependent skips. The server's existing pinned Torch/rsl_rl CPU environment
  passed all nine population checks, including real PPO on a synthetic environment,
  mutation acceptance/rejection, NPZ export, multi-generation branching and resume.
  No Genesis scene or GPU training was started; Newton-Gym was untouched. Canonical
  development-container gates and a real Genesis population smoke remain unrun;
  no generated development wrapper is available in this checkout and local Torch/
  Genesis are absent. A fresh experiment and physics preflight are required by the
  changed source digest before real training. Existing failed-run evidence is intact.

### C-hug evolutionary PPO pilot launched — 2026-11-07 14:24 KST

- User authorized execution. GPU2 UUID
  `GPU-a40782b2-881c-e285-2a8e-66cae9162a01`, process/PGID `2368570`,
  output `workbench/evidence/generated/c-hug-evolution-20261107T0518Z`.
  Only D100 `c-train-000-d100` is being trained, from fresh initialization.
  Four PPO members share 128 physics lanes sequentially, with 512 total rollout
  allocation, 100 updates/generation and at most 2000 updates/member. The raw
  train process stays resident throughout; no reporting-chunk runner reloads.
- New code hash `7ae2b8dc88a1471e40844cbecea994c655a441991b8747122f067dde77ddc231`;
  model remains `f17ffa8c5e4ba1b358a9962b7b6f7cdb6dc883ffb06b9c0568ff0ab3dec199bb`.
  Fresh real-GPU Home/Extend/goal preflight passed all three D100 anchors.
  A separate Genesis smoke completed one real PPO update, physical evaluation,
  checkpoint and policy NPZ export for each of four members on 3 lanes/member.
  This is pipeline evidence, not successful grasp/lift learning.
- Added single-anchor study selection and atomic population phase/progress files
  for operational monitoring. Focused host suite: 67 passed, 3 Torch skips.
  Remote source was backed up before applying only the Hug research modules;
  exact new source ZIP and manifests are saved with the run. No Git push was made.
- Luna guard `/root/luna_evolution_guard` monitors the run, captures first-100 and
  subsequent 500/1000/1500/2000-update policy videos, and collects artifacts.
  Final replay/hold success remains to be evaluated. Newton-Gym was not changed;
  its existing process `2292112` remains assigned to GPU1.
- Launch audit: `workbench/evidence/curated/planning/c_hug_evolution_launch_20261107.json`.
  Current pointer: `workbench/evidence/generated/c-hug-evolution/active-launch.json`.

### C-hug report camera moved to the arm side — 2026-11-07 19:58 KST

- The user reported cylinder occlusion in policy videos. The new report-only
  `analysis.record_hug_video` entrypoint uses position `(0.85,-1.15,1.60)` m,
  look-at `(0.35,0.05,0.40)` m and world Z up, viewing the arm from the opposite
  side of the C-target cylinder. `analysis.watch_hug_population` now uses this
  collector for future milestone videos. Camera metadata is stored in both the
  trajectory diagnostics and report JSON; original videos are preserved.
- The actual 500-update policy was recorded again on GPU2. Its 20-second video
  was visually checked: the arm silhouette and separation from the cylinder are
  visible. The 2,001 timed command samples, full collection measurements and
  policy metadata match the original recording exactly (maximum command
  difference 0). The recording changes presentation, not the failed grasp/lift
  result. Downloaded MP4/NPZ/report hashes were verified. Artifacts:
  `c-hug-concurrent-20261107T1025Z/server-reports/camera-review-00500/`.
- Eight focused host tests passed for watcher reporting and recording cadence.
  No host scientific dependencies were installed. Canonical container gates
  remain unrun because this checkout has no usable generated development wrapper.
- Only the idle report watcher was replaced: `2395904` → `2401099`, preserving
  milestone completion records and the original remaining watch deadline.
  Training PID `2395787` continued to 700 completed updates; Newton-Gym
  `2292112` stayed alive. Task source SHA remains `b58659c11ba05c42c3d00ad0fe9f6e59fc82188762a11c84244b3bbc36a0d66b`.
  No rewards, selection criteria, policies or experiment hashes were changed.
  Audit: `workbench/evidence/curated/planning/c_hug_report_camera_20261107.json`.

### C-hug reports independent of Codex login — 2026-11-07

- Added `workbench/research/analysis/watch_hug_population.py`, outside the frozen
  experiment digest, so video reporting survives the operator session. Reporting
  process PID/PGID/SID `2371391` runs detached on the server and never changes or
  restarts training. It polls every 60 seconds for 500/1000/1500/2000 milestones,
  saves policy/video/trajectory reports and a final replay under the run's
  `server-reports/`, and exits on training completion or an 8-hour reporting limit.
- Original training PID `2368570` is unchanged. Tests: five host regressions pass
  for PID identity, completed-generation selection, failed-grasp report handling,
  replay errors and read-only polling. Remote `--once` probe passed; both detached
  processes were confirmed alive from a separate SSH connection. Frozen Hug code
  and reward settings were not edited for this change.
- Luna completed and hash-verified the first 100-update video under
  `guard-reports/updates-00100/`. It now observes/collects only; subsequent rendering
  belongs to the server watcher to avoid duplicate GPU report jobs.
- Evidence: `workbench/evidence/generated/c-hug-evolution-20261107T0518Z/server-watch-confirmed.json`.
  Results remain on the server while the operator is logged out and can be
  collected after reconnecting. No notification service was added.

### C-hug next reward revision v4: time and compact footprint — 2026-11-07 (local only)

- User requested stronger delay and workspace costs after reviewing 100/800-update
  videos. Implemented next experiment schema v4, reward contract
  `bounded-milestones-compact-workspace-time-v4`. Time is -0.3/s (was -0.1),
  exempt only during a currently confirmed qualified loaded hold. Touching/rubbing
  or a historical grasp does not exempt waiting. The 20-second limit and 5-second
  physical verification are retained.
- New `hug_c/workspace.py` uses actual collision-box corners of ten nodes and
  three gripper links. Fixed convex XY envelope covers Home/goal plus initial
  object footprint, with 50 mm allowance; squared excess cost caps at -0.2/s at
  100 mm excess. Actual link poses are sampled every physics tick. Vertical lift
  is exempt from this footprint preference; necessary detours remain possible.
  It is not a new hard collision boundary or an approach-feasibility proof.
- Policy steps now charge requested action effort at the existing coefficient,
  closing the zero-cost blocked-command hole. Command-only replay explicitly
  labels its action-cost lower bound. One-time/record milestone credits prevent
  rubbing or repeated contacts from paying for delay.
- Population schema v3 fixes the observed generation-modulo/stagnation lock by
  choosing variation type with a persisted actual-attempt counter. Regression
  interventions at 3/6/9/12 yield exploration/weights/rewind/exploration. Equal
  task/approach progress also reports efficiency costs, consistently with the
  next reporting selector.
- Focused host tests: 73 passed, 5 Torch skips; reporting tests: 5 passed. Separate
  server CPU process loaded proposed modules in memory and passed 15 geometry,
  Torch effort, mutation and real-PPO synthetic/resume checks. No host dependency
  installation, server source update, GPU scene or new training was performed.
  Canonical development-container gates remain unavailable; the new physical
  workspace integration still requires a fresh configure/Genesis preflight.
- Kinematic audit of saved commanded poses estimates extra workspace costs
  -0.57 (100-update best) versus -3.94 (800-update policy), with time -6 instead
  of -2 for both failed 20-second rollouts. These are offline command estimates,
  not newly measured physical returns or evidence of improved RL convergence.
- Live GPU2 run `c-hug-evolution-20261107T0518Z` remains frozen at its original
  source/reward contract; Luna/server watcher continue monitoring it. The new
  code was not inserted mid-run. Audit:
  `workbench/evidence/curated/planning/c_hug_efficiency_v4_20261107.json`.

### C-hug v4 replacement launched — 2026-11-07 17:47 KST

- User explicitly requested stopping the current run and restarting v4. Verified
  exact process identity/group ownership before terminating old training `2368570`
  and watcher `2371391`; old run had completed 900 updates/9 generations. Its
  state, partial tenth generation, videos, checkpoints and 39-file exact training
  source ZIP remain under `c-hug-evolution-20261107T0518Z`. Newton-Gym process
  `2292112` stayed alive on GPU1 and was not changed.
- New root `workbench/evidence/generated/c-hug-evolution-v4-20261107T0840Z`.
  Code SHA `2a302ed3abd82dbdd4f33f3faaf50fa2f892358fcf05edeced4dda3ecd8a152a`;
  model SHA `f17ffa8c5e4ba1b358a9962b7b6f7cdb6dc883ffb06b9c0568ff0ab3dec199bb`.
  Fresh schema-v4 configuration passed actual GPU preflight for all three D100
  anchors, plus a new four-replica footprint diagnostic: Home/goal excess 0 in
  every lane; extended-pose excess 0.405066 m and cost fraction 1 in every lane.
  The latter is direct-placement numerical evidence, not an approach proof.
- Four-member real Genesis smoke completed one PPO update, physical evaluation,
  checkpoint and NPZ export per member using the new costs. Main process/PGID/SID
  `2384143`, started `2026-11-07T08:47:06.582152+00:00`, was confirmed detached
  (PPID 1) and actually assigned GPU2 UUID
  `GPU-a40782b2-881c-e285-2a8e-66cae9162a01` from a separate SSH connection.
  Scope remains only D100 `c-train-000-d100`, fresh weights, four sequential
  128-environment members, 20 generations/2000 updates per member.
- Detached report watcher `2384249` records 100/500/1000/1500/2000-update videos
  plus final replay for up to 12 hours, without Codex login. Luna guard was
  redirected to new status/JSON/MP4 collection and does not start duplicate
  renderer jobs. Both watcher and main process were confirmed alive. No new
  grasp/lift success is established by startup checks.
- Current pointer: `workbench/evidence/generated/c-hug-evolution/active-launch.json`.
  Audit: `workbench/evidence/curated/planning/c_hug_efficiency_v4_launch_20261107.json`.
  Previous canonical-container gates remain unrun; this launch adds real GPU
  task startup evidence rather than general release/hardware acceptance.

### C-hug permanent simultaneous population — 2026-11-07 19:29 KST

- User requested four policies running together, and explicitly required the
  same configuration for future launches. Population schema v4 now defaults to
  `simultaneous-shared-physics`: one 512-environment scene, four disjoint 128-lane
  PPO groups. `population_batch.py` combines four action blocks, advances physics
  once and sends observations/rewards/dones/timeouts only to the owning learner.
  Normalizers, buffers, optimizers and adaptive learning rates remain private.
  Evaluation also advances all policies together and records each representative
  path only through its own done. The training sample budget is unchanged.
- Regression evidence: 80 host tests passed, 7 Torch skips. Fourteen server CPU
  checks passed, including real PPO simultaneous rollout/resume and explicit
  compatible-task migration. Additional real-PPO regression exercised immediate
  rewind after transfer. Unit routing proves 3 updates x2 rollout steps call
  physics six times rather than 24 while each learner receives its correct data.
  No host dependency installation or canonical development-container gate was run.
- Previous sequential v4 processes `2384143` / watcher `2384249` were stopped after
  identity validation. Its 400 fully completed updates, four-policy continuation
  checkpoints, optimizer states, staleness, archive, floors, historical champion
  and one actual variation attempt were explicitly transferred. Partial later
  generation evidence and exact old source remain in the previous run. No old
  experiment hashes were rewritten. Fifteen transferred checkpoint files were
  verified byte-for-byte against their originals; transferred champion NPZ has a
  new context plus explicit original checkpoint hashes.
- New root: `workbench/evidence/generated/c-hug-concurrent-20261107T1025Z`.
  Source SHA `b58659c11ba05c42c3d00ad0fe9f6e59fc82188762a11c84244b3bbc36a0d66b`;
  model remains `f17ffa8c5e4ba1b358a9962b7b6f7cdb6dc883ffb06b9c0568ff0ab3dec199bb`.
  Actual GPU preflight passed; a 512-robot/four-policy real PPO smoke completed
  simultaneous update/evaluation/NPZ export before launch. This is execution
  evidence, not successful grasp/lift evidence.
- Main `2395787` started at `2026-11-07T10:29:25.443830+00:00`, detached PPID1
  and assigned the same GPU2 UUID. It inherits 400 updates and adds only 1600,
  retaining the 2000-update target. At 10:31 UTC, progress showed all four policies
  beyond 20 joint PPO updates with `physical_num_envs=512`. Each generation has
  one shared `training_wall_s`; do not sum duplicated per-member wall durations.
- Detached watcher `2395904` owns 500/1000/1500/2000 videos/final replay; Luna
  monitors and collects without duplicate renderer jobs. Existing Newton-Gym
  `2292112` remains alive on GPU1. Scope stays the first D100 anchor.
- CLI defaults, `gpu2-next-study.json` and the task README now describe actual
  simultaneous 512 robots. `--population-from` is explicit compatible sequential
  migration; strict `--resume` is for an already-concurrent run.
  Audit: `workbench/evidence/curated/planning/c_hug_concurrent_launch_20261107.json`.
  Current pointer: `workbench/evidence/generated/c-hug-evolution/active-launch.json`.

### Independent GA to BC to ordinary PPO setup — 2026-11-08

Research-only `hug_ga.handoff` adds explicit verify-demo/bc/ppo entrypoints.
Three successful Home action replays gate demonstration publication; matching
case/context and valid finite data gate BC. BC trains the pinned actor with its
normalization and exports actor-only Torch state plus standard policy NPZ.
PPO creates a fresh critic/optimizer, verifies actor transfer and three nominal
physical evaluations before updates, then uses one ordinary policy over 512
lanes. Existing reward-v4 is preserved; evolution is not called. Final PPO is
not automatically accepted as an improvement. No training run was launched.
Five host tests passed; pinned server CPU synthetic BC reduced MSE from .3773
to .1413, transferred deterministic actor outputs exactly, and exported NPZ
within 2.4e-7 of Torch. Actual GPU pipeline remains untested pending successful
GA data. Canonical development-container gates remain unrun (no usable generated
wrapper); no host scientific dependencies were installed. New prepared inputs:
`workbench/evidence/generated/hug-ga-d100-handoff-20261108/plan.json`.

### GA population parade capture — 2026-11-08

Independent GA now records every tenth evaluated generation as a caption-free
1920x1080 MP4, 25 fps under current physics dt. A whole-population camera renders
all lanes (default128) using actual visual environment offsets. Recording hooks
physics ticks during evaluation; it does not rerun winners or alter fitness.
The scoped scene factory preserves pinned hug_c physics source and only supplies
visualization/camera options. Eight focused host tests passed, including cadence,
all-grid framing and recorder cleanup. Pinned Genesis API was inspected remotely;
actual GPU video rendering/search was not launched and remains unverified.
Prepared source-matching plan: `workbench/evidence/generated/hug-ga-d100-parade-20261108`.
Canonical development-container gates remain unrun (no generated wrapper).

### First independent GA launched on GPU2 — 2026-11-08 19:39 KST

- User authorized GA execution and discussion after completion. Scope is the
  first100mm C-hug anchor `c-train-000-d100`, starting at the real Home,128 distinct
  candidate paths in one actual physics batch, six knots, at most100 generations
  (12,800 candidates). Stop after the first successful generation. BC/PPO do not
  start automatically. Existing task/reward source remains frozen at b58659c11.
- Actual128-lane GPU physics and raw MP4 smoke passed. The first visual layout
  left too much unused frame, so presentation now uses1.8m drawing spacing and
  eight world-Y slots, with exact frustum fitting for a landscape16x8 view. This
  changes no lane physics poses. Final launch preview was visually inspected.
  The launch has an exact GA source ZIP and file SHA inventory. Focused host
  tests remain8 passed; no new host dependencies or canonical wrapper gates.
- GA source SHA `74487b99882e36721ad244d4ca2e6565906abcec8730ec345d78013f34a1c528`.
  Root: `workbench/evidence/generated/hug-ga-d100-20261108T103534Z`.
  PID/PGID/SID2472536 started at10:39:40UTC (19:39:40KST), PPID1, using GPU2 UUID
  `GPU-a40782b2-881c-e285-2a8e-66cae9162a01`. Confirmed in actual nvidia-smi
  compute-process list and process environment. GPU0/1 jobs were not touched.
- First generation completed in34.65s after14.21s initialization. No successes:
  111 forbidden-contact/limit cases,11 wrong-face cases and6 safe no-grasp cases.
  Second generation was active. Single-generation extrapolation is about58min
  for100 generations before unknown parade overhead; not a measured final ETA.
- Every tenth evaluation records a caption-free parade MP4. Progress now writes
  generation/elapsed time, rejection counts and a recent-generation ETA; until
  the first parade its rendering overhead is uncertain. Detached completion
  records distinguish success-found, budget-exhausted and exceptions.
  Audit: `workbench/evidence/curated/planning/hug_ga_gpu2_launch_20261108.json`.
  Pointer: `workbench/evidence/generated/hug-ga-active.json`.

### Independent GA found its first successful candidate — 2026-11-08 19:48 KST

The GPU2 run ended normally with `success-found` at generation14, after1,792
candidate evaluations and520.50s including initialization. Zero-based candidate124
in `search/generation-0013` passed the existing task criterion at19.8s: grasp,
lift and hold fractions1; fault=false; wrong-face-any=false. This is a batched
search success, **not yet an independently replay-verified demonstration**.
No BC/PPO or follow-on search was launched. Generation10 parade MP4 plus the
successful generation's genomes, commands, rollouts and reports were copied
locally with SHA verification. Pointer `hug-ga-active.json` now reports finished.
Audit: `workbench/evidence/curated/planning/hug_ga_first_result_20261108.json`.
The earlier approximately one-hour estimate was a maximum-budget extrapolation;
early success stopped the actual run after about8min40s.

### Candidate124 single-scene video — 2026-11-08 20:23 KST

At the user's request, `analysis.record_ga_candidate` replayed exactly the198
saved actions of generation14 candidate124 in a separate one-lane GPU2 scene.
The raw arm-side video was downloaded, SHA-verified and visually inspected.
Single replay accepted=true: grasp confirmed, hold5.08s, minimum bottom clearance
48.93mm, max gripper-relative slip8.08mm, contact coverage1, fault=false and no
wrong-face exposure anywhere in the episode. The cylinder is visibly tilted;
this records existing criterion success, not an upright-object requirement.
One replay does not satisfy the three-repeat demonstration gate or establish
robustness. No further GA, BC or PPO ran. Frozen GA/task source digests preserved.
Video: `hug-ga-d100-20261108T103534Z/candidate-124-video-20261108T112228Z/candidate.mp4`.
Audit: `workbench/evidence/curated/planning/hug_ga_candidate124_video_20261108.json`.

### GA and RL now require the full90-degree raised pose — 2026-11-08

- User requested final90-degree lift in both GA and RL. Confirmed canonical
  convention in legacy Wrap lift config: wrap at roll-90°, lift back to roll0°.
  New defaults require actual measured roll0° ±5° for the entire five-second
  loaded hold, alongside clearance>=20mm, no floor contact, C posture, contact
  coverage and retention. Roll leaving tolerance resets the window. Home/motion
  actions remain free; no timeline or fixed intermediate deployment was added.
- Shared HoldTracker enforces the final pose; RewardLedger caps lift progress by
  min(clearance progress, roll progress), and gates hold/success/time exemption
  on raised roll. Existing reward coefficients are unchanged. GA uses these same
  progress/success metrics, reports roll error/posture status and seeds solver
  grasp then raised roll0 knots without forcing mutation to keep that schedule.
- Current experiment schema5, policy schema4, control mode
  `policy-four-dof-home-lift90-v4`, reward-v5. Old partial-lift experiments,
  policies and context-mismatched demonstrations are rejected for new runs.
  Original candidate124/video/results and exact40-file prior task source ZIP
  remain preserved locally and remotely. No old success record was relabeled.
- Focused host tests:91 passed,7 Torch skips. Added regressions for original
  winner's -55° roll rejection, both tolerance boundaries, timer reset/restart,
  height independence, full-reward/time exemption rejection at partial roll,
  GA seed construction and legacy artifact rejection. Actual pinned server CPU
  NumPy/Torch tests agree: old roll has0 hold ticks and no hold/success credit;
  roll0 reaches500 hold ticks and succeeds. This is numeric software evidence,
  not a new physical success or robustness validation.
- Source change and fresh inputs were applied to the server after confirming no
  Hug processes were active and matching original source hashes. Fresh prepared
  experiment/GA plan live under `workbench/evidence/generated/hug-ga-lift90-20261108`.
  Task SHA `5e62f15239711d6e883bc2d6cd449fe8c535a47d2952490ce22a4006cd889684`.
  No GA search, BC or PPO started. Canonical development-container gates remain
  unrun because there is no usable generated wrapper; no host deps installed.
  Audit: `workbench/evidence/curated/planning/hug_lift90_condition_20261108.json`.

### Full90-degree lift GA started — 2026-11-08 20:39 KST

User explicitly requested another GA run after the final-roll condition change.
Root `workbench/evidence/generated/hug-ga-lift90-20261108`, PID/PGID/SID2475925,
started11:39:19UTC. Confirmed PPID1, running state, actual GPU2 compute assignment
and CUDA UUID `GPU-a40782b2-881c-e285-2a8e-66cae9162a01`. Scope remains firstD100
anchor,128 simultaneous candidates, six knots,100-generation maximum and raw
parade videos every10 generations. Success-found stops search; BC/PPO do not
start. Criterion is loaded clearance plus roll0±5° for five seconds. Source
snapshots and hashes are preserved in the run root; other GPU jobs untouched.
At initial verification, generation0 evaluation was active without exceptions.
GA SHA `35f3065776fe006c7710cf3a6f09fa3878a1932f3fcde0ca1210e2120ec5d4ab`;
task SHA `5e62f15239711d6e883bc2d6cd449fe8c535a47d2952490ce22a4006cd889684`.
Audit: `workbench/evidence/curated/planning/hug_ga_lift90_launch_20261108.json`.

### Full90-degree GA found a successful candidate — 2026-11-08 21:31 KST

The second GA ended normally with success-found at generation84, after10,752
candidate evaluations and3,131.39s (52min11s) including initialization/video.
Zero-based candidate22 in `search/generation-0083` passed the full lift90 criterion
at18.5s: grasp/lift/hold fractions1, posture complete, final roll error.0659569rad
(3.78deg), fault=false, wrong-face-any=false. This is one batched search success;
independent single-scene replay and three-repeat demonstration verification have
not run. No BC/PPO followed. The winner's genomes/commands/rollouts/reports and
80th-generation parade were retrieved with matching SHA; eight parades10..80
exist on the server. Pointer `hug-ga-active.json` now records finished.
Audit: `workbench/evidence/curated/planning/hug_ga_lift90_result_20261108.json`.

### Lift90 candidate22 recorded alone — 2026-11-08 22:06 KST

User requested the winning candidate video. The existing single-candidate helper
replayed exactly185 saved actions in a separate GPU2 scene and recorded raw RGB.
Single replay accepted=true: full lift posture with roll error3.78deg, continuous
hold5.04s, minimum floor clearance474.88mm, hand-relative slip8.41mm, contact
coverage1, no fault and no wrong-face exposure. Video was downloaded, SHA-verified
and visually inspected: the arm encloses and rotates the cylinder into the
raised position. This is one nominal replay, not three-repeat approval or
robustness testing. No GA/BC/PPO started. Artifacts are under
`hug-ga-lift90-20261108/candidate-22-video-20261108T130513Z/`.
Audit: `workbench/evidence/curated/planning/hug_ga_lift90_candidate22_video_20261108.json`.

### GA-only 50mm horizontal object-motion constraint started — 2026-11-08 22:39 KST

- User selected a strict50mm test to investigate whether large object dragging
  is necessary. Added optional `prepare --max-object-xy-mm` in plan schema2.
  The only new selection condition is initial-relative world-XY centre excursion:
  >50mm at any10ms physics tick disqualifies permanently, even if the object
  returns. Height, object orientation and accumulated travel are excluded.
  Tracking freezes after lane termination; evaluation stops that lane at the
  next macro boundary. Reports record peak distance and first violating tick.
- GA success/fitness cannot rescue a violating candidate. Constraint metadata
  persists into rollouts, independent demonstration replay and BC provenance;
  BC refuses mixed constraints. Single-candidate video honours source constraints
  and permits a stricter counterfactual bound. PPO reward/settings were not changed.
  Same Home, D100 first anchor, seed42, six knots,128 candidates,100 generations,
  20s episode, full90-degree lift and5s hold; parades remain every10 generations.
-26 focused host tests passed. Actual pinned server NumPy/Torch checks confirm
  permanent transient rejection and vertical-motion exclusion. Prior successful
  candidate22 replayed with50mm cap is rejected at tick1061 (10.61s); next macro
  boundary peak58.30mm. The truncation reward is not a complete-episode PPO score.
  Counterfactual video/report were downloaded and SHA-verified.
- New root `workbench/evidence/generated/hug-ga-xy50-20261108T133605Z`.
  Main PID/PGID/SID2484281 began13:39:12UTC (22:39KST), PPID1, verified on GPU2
  UUID `GPU-a40782b2-881c-e285-2a8e-66cae9162a01`. At first check2 generations /
  256 candidates had completed; no admissible candidate yet (second generation:
  104 motion-limit,24 forbidden-contact/limit). Search continues; this is not an
  impossibility result. No automatic BC/PPO or reward changes. Other jobs untouched.
- Task code SHA stays5e62f15239711d6e883bc2d6cd449fe8c535a47d2952490ce22a4006cd889684;
  GA SHA a4eba37d43088d528ed2cddf98563d5922c9069e36455945fdcc05eababedf0d.
  Old GA sources archived before server update; exact new launch source saved.
  Audit: `workbench/evidence/curated/planning/hug_ga_xy50_launch_20261108.json`.
  Canonical dev-wrapper gates remain unrun (no usable generated wrapper); no host
  scientific dependencies installed. GPU startup evidence covers the new monitor.

### 50mm-constrained GA exhausted its budget — 2026-11-08 23:40 KST

The50mm experiment completed100 generations /12,800 candidate evaluations in
3,649.25s (60min49s), ending normally with budget-exhausted, returncode1 and0
successful candidates. Best admissible lineage appeared at generation94,
candidate31, carried into final generation candidate0: confirmed grasp, peak
object XY displacement47.894mm, lift progress.415068, final roll error.955501rad
(54.75deg), no qualified hold, no fault/wrong-face/movement violation. This is a
bounded search failure for one seed, not proof that50mm makes the task physically
impossible. All ten parades10..100 exist. Best lineage data and final parade were
copied locally with matching SHA. No BC/PPO or follow-up search launched.
Audit: `workbench/evidence/curated/planning/hug_ga_xy50_result_20261108.json`.
Pointer `hug-ga-active.json` now records finished.

### 50mm GA generation-progress audit — 2026-11-09

Read all100 generation reports (12,800 evaluations) at the user's request to
assess whether more generations were likely to help. First admissible grasp /
loaded-lift progress at generation24 (.223322), then .245251 at34, .377313 at52,
and .415039 at77. The generation94 record is .415068: only .000028789 additional
progress and final-roll error54.76246→54.74620deg (0.01626deg improvement).
Every admissible candidate's qualified hold progress remained0 through100.
Thus the last23 generations were effectively a plateau in task-level progress;
the trace provides weak support for a small unchanged-budget extension. This
is a single-seed diagnosis, not proof no later breakthrough can occur. No new
search, physics rollout or training launched. Raw history and progress audit
saved under the50mm run root; compact evidence:
`workbench/evidence/curated/planning/hug_ga_xy50_progress_20261109.json`.

### GA 50mm cutoff now exempts airborne motion — 2026-11-09

At the user's request, changed code only; no new GA/search, physics replay, BC
or PPO started. New `hug-ga-object-motion-v2` contracts check initial-relative
XY displacement at every physics tick only while grounded. Airborne means no
loaded floor contact and positive oriented-cylinder bottom clearance; the final
raised roll is not required for exemption. Landing reactivates the same initial
reference. A prior grounded violation remains permanent after lift or return.
Reports/rollouts distinguish overall peak XY displacement from its grounded
peak. Independent demonstration verification and single-candidate recording use
the same tracker. Historical v1 constraints retain whole-episode semantics,
including on stricter numeric replay overrides; old results are not relabeled.
PPO rewards, the shared lift90 task, full-lift/hold requirements and other fitness
rules are unchanged. Fresh source-pinned plans are required for a future run.

Focused host tests:31 passed, including airborne exemption, landing, permanent
ground violation, independent peak tracking, conflicting floor/clearance evidence
and legacy scope. Server CPU NumPy/Torch checks also passed, including the actual
physics-tick monitor callback chain and restoration on normal/exception exit;
Genesis was not imported and no physical rollout ran. Updated six scoped server
files after checking all46 prior launch source hashes and no active Hug process;
pre-edit sources were archived in the completed run root. The40 shared task files
remain identical. GA SHA
`ddcdecee025e3ff4e23cc074c80e9b7db6256a10b4c22b7ea4bba2ec8e2258d0`;
task SHA stays `5e62f15239711d6e883bc2d6cd449fe8c535a47d2952490ce22a4006cd889684`.
Canonical container gates remain unrun because no usable generated development
wrapper is available; no host dependencies installed. No fresh plan or active
run pointer was created. Audit:
`workbench/evidence/curated/planning/hug_ga_ground_only_20261109.json`.

### Ground-only50mm GA started on GPU2 — 2026-11-09 00:37 KST

User authorized execution after the airborne-exemption code report. Prepared a
fresh plan under `workbench/evidence/generated/hug-ga-ground50-20261108T153727Z`
from the same validated lift90 experiment, with v2 ground-contact-only50mm
constraint, D100 first C anchor, Home start,128 distinct simultaneous candidates,
six knots, seed42 and100-generation maximum. Full roll0±5deg loaded5s hold remains
required. Raw whole-population parades are recorded every10 generations.

PID/PGID/SID2490977 began15:37:27UTC, is detached with PPID1 and was confirmed
running on actual GPU2 UUID `GPU-a40782b2-881c-e285-2a8e-66cae9162a01` (1074MiB).
At15:38:21UTC,2 generations /256 evaluations had completed and generation3 was
active. No admissible success yet; these initial grounded evaluations match the
old seed's early behavior. Both source digests match the preceding code-change
audit. Exact46-file launch-source ZIP, hash inventory, plan and startup evidence
were copied locally. The previous whole-episode50mm run remains intact. Other
GPU jobs were inspected only. No BC/PPO follows automatically.

Luna monitoring was delegated per the user's standing request: read-only
progress/error/completion checks and collection of already-recorded artifacts,
without replay, job control or source changes. Active pointer:
`workbench/evidence/generated/hug-ga-active.json`. Launch audit:
`workbench/evidence/curated/planning/hug_ga_ground50_launch_20261109.json`.

### Ground-only50mm GA exhausted100 generations — 2026-11-09 01:40 KST

The run completed100 generations /12,800 evaluations in3,764.61s (62min45s),
ending normally with budget-exhausted, returncode1 and no successful candidate.
The GA process has exited. Best admissible lineage first appeared at generation94
candidate31 and survives as final candidate0: grasp1, lift progress.4915168,
qualified hold0, final lift-roll error.842443rad, no fault/wrong-face/motion
violation. Overall XY displacement86.145mm versus grounded peak45.569mm confirms
the airborne exemption admits this candidate. The previous whole-episode-cut
best was.415068; this is improved progress, not a completed lift or feasibility
proof. Progress was already.491363 by generation83, with only.000154 further
improvement through the budget limit. Across the search845 admissible candidate
evaluations had overall displacement>50mm while the grounded peak stayed<=50mm;
these counts include lineage repeats, not distinct behaviors.

All ten parades10..100 exist remotely. Luna copied terminal status/completion,
final report/genomes/rollouts and the final existing parade locally under
`hug-ga-ground50-20261108T153727Z/remote-terminal/`, with SHA checks. The active
pointer is finished. No independent replay, further GA, BC or PPO followed.
Other GPU jobs and the prior experiment remain untouched. Result audit:
`workbench/evidence/curated/planning/hug_ga_ground50_result_20261109.json`.


### XYD 입력 기반 역GA 프레임워크 (2026-11-09)

- `workbench/research/experiments/hug_reverse_ga` 추가. Mapper 함수의 정확한 XY
  역변환으로 C형 파지 q3/q4를 구하고, 그 자세에서 실제 Home까지 중간 자세와
  시간을 진화시킨다. 삼각형 파일이나 기존 점군은 필요 없다. 기존 mapper의
  정적 충돌/준비 동작 CAD 필터는 유지하며, 그 범위 밖을 탐색했다고 주장하지 않는다.
- 128개체·100세대 기본값, 매 physics tick 실격 판정, Home 연속1초 확인,
  성공/미완주/실격 순서의 선택, 10세대마다 무자막 영상, 성공 후 독립 준비/재생3회.
  초기 솔버 파지가 불성립하면 `initial_grasp_invalid`로 탐색 전에 종료한다.
- 로봇 전체 관절 q/qd와 물체 pose/velocity 및 접촉을 NPZ에 저장한다. 프로세스 내
  완전 상태 복원을 비교 검증하며, 이후 curriculum은 초기 준비+역경로 prefix 재생으로
  물체 상태까지 복원해야 한다. PPO 연결/검증된 reset pool은 별도 후속 작업이다.
- Host 규칙/모의 admission·restore·실패 재검증 테스트 **23 passed**. 실제 mapper
  `(x,y,D)=(-113.312708,177.585814,100)mm` 준비 실행이
  `q3=-89.5°, q4=-162.5°, mu_min≈0.17658`을 재현했다.
- 사용자 요청에 따라 **GPU 실행/원격 배포는 하지 않았다**. Host에는 Torch/Genesis가
  없고 사용 가능한 generated dev wrapper도 없어 canonical gates와 실제 물리 admission,
  Genesis snapshot restore, 영상 및 GPU 탐색은 검증하지 않았다. 성공 파지나 경로를
  얻었다는 증거가 아니다. [실행 안내](../workbench/research/experiments/hug_reverse_ga/README.md).


### 역GA GPU2 실행 (2026-11-09)

- 사용자가 실행을 승인하여 `hug-reverse-ga-d100-20261109T050108Z`를 GPU2
  UUID `GPU-a40782b2-881c-e285-2a8e-66cae9162a01`, PID2527763으로
  분리 실행했다. 100mm 기존 XY, 128개체·최대100세대, 10세대마다 무자막 영상.
- 초기 파지 **128/128 통과**, 마지막 유효 접촉49ticks, 최대 XY drift1.565mm.
  완전 물리 상태 복원 검사와 첫2세대 실행을 통과했다. 확인 당시 성공0, clean0;
  정답 경로/독립 재생 성공을 얻은 상태는 아니다. Luna가 읽기 전용으로 경비한다.
- 첫 시도는 admission 후 reset에서 Torch inference tensor의 in-place 변경 오류로
  종료됐다. 역GA tick의 `inference_mode`를 `no_grad`로 수정했다. 기존 조건·물리
  파라미터는 유지하고 새 디렉터리로 재시작했다. Host **23 passed, 1 skipped**
  (Torch 없음); 서버 실제 Torch CPU 회귀 검사 통과. 서버 pytest는 없어 설치하지 않았다.
- 프로세스의 parent1 및 정확한 GPU UUID를 확인했고 기존 다른 GPU 작업은 유지했다.
  초기 영상, 소스 ZIP/해시, 원격 시작과 진도 증거를 보존했다. BC/PPO는 실행하지 않았다.
  [실행 기록](../workbench/evidence/curated/planning/hug_reverse_ga_gpu2_launch_20261109.json).


### 역GA 첫 역경로 검증 완료 (2026-11-09)

- 위 GPU2 실행은 **3세대,170.24s** 후 `reverse_path_verified`로 종료했다.
  세번째 세대 candidate83의 솔버 파지→Home 경로는 **5.89s**이며 마지막
  연속1s Home 정지 검증을 포함한다. 물체 최대 XY 이동39.188mm, 기울기1.611deg.
- 별도 초기 준비 후 독립 재생3회 모두 각128/128 성공. 명목 조건에서 반복 확인한
  결과이며 랜덤 물성/초기 상태 강건성 검사는 아니다. 첫 성공 시 종료하는 계획에 따라
  100세대까지 돌지 않았다. 10세대 사열 영상 대신 초기 영상과 검증 영상3개가 남았다.
- 완료 후 PID2527763 및 GPU2 compute 점유가 사라진 것을 확인했다. 다른 GPU 작업은
  유지했다. Luna 경비도 완료 조건으로 종료했다. 역경로 NPZ는 서버에 보존되며 보고서와
  대표 검증 영상을 로컬에 수집했다. 정방향 끌어안기/리프트 및 PPO reset pool 검증은
  아직 수행하지 않았다. [결과](../workbench/evidence/curated/planning/hug_reverse_ga_result_20261109.json).


### 역GA→PPO 역커리큘럼 코드 연결 (2026-11-09)

- `workbench/research/experiments/hug_reverse_rl`에 `prepare/check/train/evaluate`
  명령을 구현했다. 검증된 역GA 결과의 완전 물리 상태를 prefix replay로 재구성하여
  snapshot bank를 만든다. 관절/물체의 속도를 보존하고, 부분 reset이 다른 lane을
  변경하지 않는지 확인한다. 각 에피소드 보상 잠재값은 복원된 실측 q로 초기화한다.
- 파지 상태에서 기존90도 리프트·5초 유지의 전체 성공을 먼저 학습하고, clean
  frontier 성공률로 Home 쪽 reset 범위를 넓힌다. 기본70% 진출/20% 후퇴,512회
  frontier 에피소드,80 control-step cooldown, 과거 구간20% 유지. 마지막에는 원래
  물체 배치의 Home reset50%를 섞는다. 과거 epoch/easy 구간 성공 및 한 번이라도
  ZED-side 접촉한 에피소드는 frontier 진출 근거로 사용하지 않는다.
- 기본512 병렬의 일반 PPO이며 역GA 명령을 정책 action으로 강제하지 않는다.
  기존 q3/q4 힌트·상벌점·자유4축 제어는 유지한다. 체크포인트마다 커리큘럼
  frontier/window/cooldown/RNG를 hash로 짝지어 저장·복구한다. actor NPZ 및 원래
  Home에서의 명시적 평가/영상 명령을 추가했다. 과거 C-hug/GA 소스는 수정하지 않았다.
- Host **48 passed,6 skipped**(Torch/tensordict 없음). 서버 CPU Torch 검사5개
  통과: 실측 reset·물체 속도, 종료 전 성공 집계, 과거 잘못된 접촉, 실제 모델 생성,
  실제 체크포인트 저장·재개. optimizer update는0회다. 실제 GA 입력의 portable
  계획 생성/로컬 재로딩을 확인했다.
- GPU2의 **2환경·21시작점**에서 prefix replay/full-state restore/1tick 접촉검사/
  nominal Home/부분 reset 검사를 통과했다. CPU replay는 quaternion 차이
  0.00030468>0.0001로 거부됐으며 기준은 완화하지 않았다. 원본과 같은 GPU
  환경에서 통과했다.512환경 bank 및 학습은 실행하지 않았으며, train 때 요청한
  병렬 수로 bank를 다시 구축·검증한다.
- **PPO 학습은 시작하지 않았다.** 최종 nominal Home 파지·리프트 성공은 여전히
  학습/평가 대상이다. generated dev wrapper와 host 과학 스택 부재로 canonical
  gates는 미실행. [실행 안내](../workbench/research/experiments/hug_reverse_rl/README.md),
  [검증 기록](../workbench/evidence/curated/planning/hug_reverse_ppo_codecheck_20261109.json).


### Go2 leg PD snapshot reset correction (2026-11-09)

PPO PID2532049 was stopped at user request. Genesis snapshot reset clears control
modes; reverse GA had restored only arm control, and the first PPO adapter
intentionally copied that faulty condition for path bins. This is corrected by
`restore_leg_hold`: reissue the configured standing position target after every
reverse GA and PPO snapshot restore, including selected environment IDs. No
joint teleport or velocity zeroing is added. Old path/plan hashes now mismatch
and require fresh reverse GA verification. Tests cover full/subset PD commands.
Live corrected replay and training have not been launched; existing logs and
checkpoints remain preserved.


### 다리 PD 복구 후 역GA 재실행·영상 완료 (2026-11-09)

- 사용자의 서버 재실행/직접 감독 지시로 GPU2에서
  `hug-reverse-ga-legpd-20261109T074421Z`를 실행했다. 같은100mm 좌표,
  128개체·최대100세대. parent가 직접 상태와 결과를 확인했으며 경비 위임은 없었다.
- **3세대·239.66s**에 `reverse_path_verified`. 성공 후보3개 중 winner83의
  명목 경로5.89s(마지막1s Home 확인 포함), 물체 최대XY36.804mm/기울기3.167deg.
  새 초기 준비 후 독립 재생3회 모두128/128 성공.
- 조기 성공해도 최종 탐색128개체 사열을 별도로 녹화하도록 추가했다.
  사열1920×1080·20s, winner 단독960×720·5.92s를 수집·화면 확인했다.
  winner 단독 재생도 성공했고, 다리 최대속도0.001327deg/s, 최대 목표각 오차
  0.3102deg로 자세 유지가 확인됐다. 다리 위치/속도를 강제로 덮어쓰지 않는다.
- 초기 자동 SSH 승인 검사1회가 시간 초과됐고 허용된 재시도로 복구했다.
  서버 계산은 그동안 독립적으로 정상 완료했다. 최종 GPU2 compute 작업 없음.
  PPO는 재시작하지 않았다. 이전 다리 제어 오류 조건의 경로·계획을 재사용하지
  않으며 다음 PPO는 이 새 역GA로 계획/bank를 다시 만들어야 한다.
- Host focused24 passed,1 skipped(Torch 없음), 실제 GPU 역GA·독립 재생·영상
  검증 완료. [실행/영상 증거](../workbench/evidence/curated/planning/hug_reverse_ga_legpd_result_20261109.json).


### 수정된 역GA 기반 PPO 재시작 (2026-11-09)

- 사용자의 학습 시작 지시로 GPU2에서
  `hug-reverse-ppo-legpd-20261109T144613Z`를 PID2554799로 분리 실행했다.
  기존100mm C형 좌표,512환경·2000업데이트의 일반 PPO다. 다리 PD를 복구한
  최신 역GA로 새 계획을 생성했으며 이전 오류 조건의 체크포인트는 재개하지 않았다.
- 512환경에서21개 시작점의 prefix replay/full-state restore/접촉 probe와
  original Home/부분 reset 검증을 통과했다. 실제 PPO 업데이트 진행을 확인했다.
  파지 근처에서 전체 리프트·5초 유지를 먼저 배우고 성공률로 Home까지 확장한다.
  q3/q4 힌트와 기존 reward-v5, 자유4축 제어를 유지한다.
- 별도 report-only 도구로 체크포인트를 검증·NPZ export하여 frontier와 원래
  Home 영상을 찍는다. 모델0의 실제 영상2개 생성으로 GPU 경로를 확인했다.
  이는 초기 모델 시연이며 학습 성공 증거가 아니다. 서버의 detached watcher
  PID2555447이100/500/1000/1500/final 영상 및 최종 original-Home 평가3회를
  수행한다. Luna는 읽기 전용 경비만 담당한다. Newton-Gym/다른 GPU 작업은 유지했다.
- Host PPO focused25 passed,5 skipped(Torch/tensordict 없음), 소스 hash 일치,
  실제512환경 상태 검증과 학습·초기 영상 검사를 완료했다. Canonical dev/release
  gates는 generated dev wrapper 부재로 미실행. 학습 완료 및 Home 성공은
  아직 미확인이며 [실행 기록](../workbench/evidence/curated/planning/hug_reverse_ppo_legpd_launch_20261109.json)을 따른다.


### 역커리큘럼 PPO 최종 결과: Home 연결 실패 (2026-11-10)

- 위 leg-PD 복구 실행은512환경·2000업데이트를 완료했다. 준비/검증 포함
  elapsed8943.305s(약2시간29분), 명목 학습 스텝16,384,000개. 오류 중단이
  아니라 학습 예산 완료이며 목표 성공을 뜻하지 않는다.
- 최대 frontier index16, 최종15(전체 index0~20). 대략300회 이후15~16
  경계에서 진출·후퇴를 반복했고 원래 Home reset 활성 단계에는 도달하지 못했다.
  모델500/1000/1500/final의 단일 frontier 시연은 모두 성공했지만 Home은 실패했다.
- 최종 모델의 frontier index15 시연은9.72s 영상으로 감싸기·리프트·5초 유지를
  통과했다. 원래 물체 배치의 Home 평가3회는 **0/3**, 모두 physical fault로
  종료했고 wrong-face 접촉은0회였다. 최종 original-Home 성공 목표는 미달성이다.
  동일 명목 조건의 결정적 재생3회이며 랜덤 강건성 평가로 해석하지 않는다.
- 보고 영상·NPZ 정책·최종 checkpoint/커리큘럼·평가 JSON을 로컬로 수집했고
  최종 영상의 대표 프레임도 확인했다. 자동 영상 명령5개는 모두0으로 종료,
  최종 평가 명령은 실패 결과를 나타내는1로 종료했다. 학습/watcher는 종료됐고
  GPU2 compute 작업 없음. 다른 GPU 작업은 유지했다. Luna 경비도 완료했다.
- reference index15는 물체 중심이 초기보다35.87mm 높고 vz+0.354m/s,
  index16은 floor contact 상태라는 차이를 확인했다. 병목 원인 후보이며 아직
  인과 검증은 없다. 동일 설정의 학습 연장보다 경계 커리큘럼 진단을 권한다.
  [최종 기록](../workbench/evidence/curated/planning/hug_reverse_ppo_legpd_result_20261110.json).


### 역커리큘럼 경계 진단·새 Home 연결 경로 검증 (2026-11-10)

- 사용자의 경계 시험 지시에 따라 GPU2에서 기존 정책을 고정한712개 에피소드를
  평가했다. 학습 업데이트0회이며 기존 PPO/task source hash는 그대로다.
  `probe_reverse_boundary`와 `probe_reverse_bridge`는 보고용 research 도구다.
- 500/1000회 정책은 reverse tick376까지 성공하고377부터 실패했다. 최종
  정책은375까지 성공,376부터 실패했다. 10ms 샘플링으로도 날카로운 성공 경계가
  확인됐다. 잡음 크기만 조절해377/400의 실패가 해결되지는 않았다.
- reverse 경로상 시작점9개에서는 짧은 명령 연결 후 기존 정책에 인계하면
  두 정책 모두 전체 과제를 성공했다. 원래 물체 배치의 Home에서는 같은 인계
  자세 도달은 안전했지만 이후 정책이 실패했다. GA Home의 물체 위치가 원래와
  약33mm 달라서 팔 자세만 같다고 같은 시작 상태로 볼 수 없었다.
- 원래 Home/물체 배치에서 인계 자세의 linear/roll35개 조합을 탐색해 각 정책
  14개 성공 후보를 찾았다. 선택한 명령은 linear−100mm,roll+0.1rad,
  q3 total−81.4583deg/q4 total+105.6442deg. 기존 action rate로1.1s 접근 후
  최종 PPO에 인계하면 총10.6s에 기존 감싸기/90도 리프트/5초 유지 과제를
  완료했다. 이력/에피소드 시계를 인계 시 초기화한 별도 재생3회도 모두 성공했다.
- 이력 초기화 전후 관절 q/qd와 물체 pose/vel6종 비교 오차는 모두0이다.
  준비 구간의 물체 최대XY이동4.46e−9m, fault/wrong-face0. 111개 전체 물리
  상태 프레임을 새 reset reference 후보로 보존했다. 새 인계점의 고정 정책
  잡음 평가: noise scale0은32/32,0.25는32/32,1은31/32 성공.
- 준비동작을 포함한 시연 성공이며 **Home부터 PPO만으로 학습된 정책은 아직
  아니다**. 다음 수정안은 검증된 forward Home→handoff prefix를 새 reset bank로
  재생하고, 기존 PPO를 명시적으로 warm-start하여 겹치는 출발 구간을 Home 쪽으로
  확장하는 것이다. 실제로 그 접근을 학습하는지는 짧은 pilot으로 확인해야 한다.
- 모든 GPU2 시험은 종료됐고 다른 GPU 작업은 유지했다. 원시 결과/영상/소스
  기록을 수집하고 화면을 확인했다. Host focused29 passed,5 skipped(Torch/
  tensordict 없음); GPU 물리 재생 검증 완료. generated dev wrapper 부재로
  canonical dev/release gates는 미실행. RL 핵심 코드는 아직 수정하지 않았다.
  [검증·수정안 문서](hug-reverse-ppo-boundary-study.md),
  [수치 기록](../workbench/evidence/curated/planning/hug_reverse_boundary_study_20261110.json).


### Home 접근 PPO·리프트 떨림 감점 코드와 검사 완료 (2026-11-10)

- 사용자의 두 수정 지시에 따라 외부 실험 `hug_home_rl`을 추가했다. 검증된
  nominal Home→handoff forward prefix를 재생해 전체 물리 상태 bank를 만들고,
  인계점→Home 순서의 겹치는 출발 구간을 확장한다. 기존 actor/critic/관측 정규화는
  엄격히 이어받고 최적화·커리큘럼은 새로 시작한다. 기존 frozen C-hug/역GA/
  역PPO 소스 hash와 payload API는 변경하지 않았다. q3/q4 힌트·자유4축·기존
  감싸기/90도 리프트/5초 유지 목표를 유지한다. 학습 에피소드에는 스크립트 접근이 없다.
- 리프트·유지의 거친 명령 변화에는 실제 적용된 증분의 변화량을 축별로 정규화해
  감점을 준다. 작은 보정 deadband, 처음 활성화 step 면제, 에피소드 최대1점
  예산을 적용한다. 물리 tick에서 확인된 파지/5mm 부상으로 활성화되고, 접촉이나
  roll 조건을 잠깐 벗어나도 종료까지 latch된다. 리프트 이전에는 적용하지 않는다.
- Host focused59 passed,9 skipped(Torch/tensordict 없음). 서버 실제 CPU Torch/
  RSL 검사4개 통과: NumPy 비용 일치·부분 reset·성공 집계·safe weights-only
  warm-start와 새 최적화 상태/paired resume. 현재 구현 source hash는
  `c7b12823ec137f40688ab52c4b5b7dc7cc62cfdca6fab4ba04df4dfa9228d8cf`.
- GPU2에서2/512환경·23개 시작점의 prefix 재생/상태·속도 복원/접촉 및 부분
  reset 후 미선택 lane의 다음 물리 tick 보존까지 통과했다. 초기 actor의 새 인계점
  재생3/3 성공, smooth 감점은 매회0.734705/1점, 총 return44.2412였다. 초기
  Home 재생은 예상대로 실패하고 smooth 감점0점으로 리프트 전 비적용을 확인했다.
- 실제16환경 PPO1회 업데이트와 새 체크포인트에서 재개1회까지 검사했고,
  커리큘럼 step16→32 재개 및 초기 actor/정규화 비교 오차0.0을 확인했다.
  체크포인트→실행 가능한 NPZ export와 팔 쪽 영상 재생도 통과했다.
- 검사용 PPO 업데이트는 총2회이며 **장시간 학습은 시작하지 않았다**. Home
  접근 학습 성공이나 떨림 감소 성과는 아직 검증 대상이다. 모든 GPU2 검사 작업은
  종료했고 다른 GPU 작업은 보존했다. 새 실험을 바로 실행할 계획/가중치는
  `workbench/evidence/generated/hug-home-rl-codecheck-20261110/revision-2/plan`에
  있으며 로컬/서버 `hug-home-rl-ready.json`이 준비 상태를 기록한다.
- generated dev wrapper 부재로 canonical dev/release gates는 미실행이다.
  [실행 안내](../workbench/research/experiments/hug_home_rl/README.md),
  [검사 기록](../workbench/evidence/curated/planning/hug_home_ppo_two_changes_codecheck_20261110.json).


### 임의 직경의 세 앵커 GA→RL→보간 전 과정 계획 (2026-11-10)

- 사용자 요청으로 기존 triangle/collection/study, 역GA, Home-prefix PPO와 보간
  코드를 조사하고 전 과정 구현 계획을 작성했다. 이번 단계는 계획만 수행했으며
  새로운 학습이나 GPU 작업을 시작하지 않았다.
- 핵심 누락은 선정한 q/root를 보존하는 GA 입력, 기존 PPO를 전제로 하지 않는
  정방향 reference 검증·보정, Wrap 끝점에서의 cold-start lift→Home 커리큘럼,
  새 정책에서 명령 경로를 수집·재생·phase 정렬하는 보간 연결부다. shared/case
  context를 분리해 다른 세 앵커의 hash를 억지로 같게 만들지 않는다.
- 계획은 새 repository-only coordinator에서 실패·예산·재개·캐시·GPU·영상까지
  연결하고, 빈 디렉터리에서 새 D로 전체 과정을 확인하는 것을 완료 조건으로 둔다.
  V1 보간은 기존 의도대로 q3/q4 가중치로 네 축 명령 경로를 섞는 것이며, 신경망
  가중치 보간이나 새 닫힌루프 정책 생성으로 부르지 않는다.
  [상세 계획](hug-ga-rl-pipeline-plan.md).

## Hug workflow retirement — 2026-11-10

Archived exact pre-edit bytes of 85 legacy Hug source/runbook files in local-only
`workbench/archive/retired-hug-pipelines-20261110.zip`; all member hashes verified
and recorded in the tracked archive inventory. Removed five obsolete standalone
files (old reverse-PPO watcher/recorder, forward-GA candidate recorder, one-off
Pilot diagnostic launcher, old GPU2 study preset). Marked five legacy workflow
runbooks historical and added `workbench/research/experiments/HUG_WORKFLOWS.md`.
Current entrypoints are reverse GA and Home PPO. Old Python implementations remain
where imports, regression tests or recursive provenance digests require them;
this is not a completed shared-library extraction. Raw evidence and regression
assertions were preserved. No server changes or training runs.

Verification: host tests for Home PPO, reverse GA, reverse PPO and bridge probes:
83 passed, 1 skipped. Initial collection without the Sim package path failed;
rerun with repository Sim/protocol PYTHONPATH passed. `git diff --check` passed.
Canonical development-container and GPU gates were not run: no usable generated
development wrapper is configured here; this source/document retirement does not
establish new physics or learning results. Archive ZIPs are ignored and must be
copied separately if historical uncommitted sources need off-machine backup.

## Solver / reverse-GA / Home-PPO pipeline implementation — 2026-11-10

User authorized implementation and GPU2 validation. Added repository-only
`workbench/research/experiments/hug_pipeline` with plan/run/resume/status/resolve,
immutable mapper inputs, exact solver-root anchor preservation, sequential worker
processes, output-hash checks and paired training resume. References require
policy-free physical forward validation/repair and three replays; cold PPO uses
actual forward state snapshots and learns every action after reset, including
Home-to-preparation. Policy, original command replay, retimed anchor and held-out
interpolation have separate success gates. No release/runtime interfaces changed.

Initial CPU regression: 102 passed, 1 skipped. Initial GPU2 v2 study exposed that
old short static screening admits vertices without sustained two-segment grasp;
those anchors were rejected by reverse admission (not treated as successful).
The owned coordinator was interrupted and screening was strengthened in the new
pipeline to require the same one-second settling/final 0.2-second proper Wrap
admission before triangle selection. Old code/source-pinned experiments remain
unchanged. New v3 study uses D100, 3-degree diagnostic sampling, 128 GA candidates,
20 search generations, 512 PPO environments, and stops after reset-bank checks.
This is a bounded engineering validation, not a published interpolation study.
Live outcome will be appended below. No successful new Home policy is claimed.

Pipeline follow-up: 152 host regression tests passed, 1 skipped; a separate
Torch-only test is skipped on host. Server scientific Python passed the real
cold-start optimizer/export/resume test (one initial and one resumed PPO update
on a fake CPU VecEnv, explicitly not physics evidence). The v3 physical study
screened 70 geometric samples down to 9 sustained-admission cases; its first
anchor reverse GA succeeded in generation 3 and all three release replays.
Forward replay reached the goal q but did not sustain both contacts. The owned
v3 run was interrupted for an explicit lexicographic forward-selection correction:
sub-tolerance q noise no longer outranks contact progress and object position.
The v4 study reuses that exact verified reverse evidence without rewriting it,
rechecks the physical catalog and performs new policy-free forward repair.
At generation 5 it achieved both contacts for 0.09 s with final XY displacement
about 1.287 mm, below the required 0.3 s continuous Wrap acceptance. This remains
an in-progress physical test, not a successful Home policy or full pipeline proof.


## Research publication and Git cleanup — 2026-11-10

Prepared the maintained Hug solver/GA/Home-PPO pipeline, mapper tools, source
packages, regression assertions and compact curated evidence for version control.
The current portable mapper template now includes the CAD/preparation filters
and total-angle q view used by the standalone page; triangle overlay sources and
the idempotent builder are versioned. Generated DATA remains an external input.
Raw NPZ/checkpoints/videos/logs and retired ZIPs stay directory-ignored; added
`workbench/evidence/local/` for uncurated local notes and copies. No source or
regression dependency folder is ignored merely because its CLI was superseded.

Focused host Hug/mapper regression: 425 passed, 28 skipped, 10 subtests passed.
The four HTTP tests initially could not bind a loopback socket in the sandbox;
they passed when rerun with that permission. Broad unrelated research collection
was unavailable with the limited host Pilot/Torch environment. Required/extended
canonical, ROS and isolated-release gates were not run: the generated development
attachment is unavailable and the host is not the scientific/ROS environment.
No dependencies were installed. The mapper-specific follow-up passed 12 tests.

The checkout's pipeline source hash still matches the running GPU2 v5 study:
`72ce95230abc9c0107368fbd2c41ec26ab19f579639001eb2f9cc7a672ffa512`.
First-anchor reverse and policy-free forward references passed three replays;
512-environment/32-bin restores passed. Interim updates1000 and1500 each passed
one unguided original-Home evaluation; final three-replay certification, the
other two anchors and interpolation remain pending. Intermediate clearance may
precede the delayed grasp-confirmation event in recorded traces; phase ordering
is an explicit remaining alignment acceptance issue. Any normalization must
produce separately replay-validated artifacts, preserving original provenance.
The running training source has not been edited for this Git cleanup.

Unrelated Connections/WSLg UI/setup implementation and matching test edits remain
in the working tree and are excluded from this research commit.


### Third-anchor GA reference audit — 2026-11-10

At user request, recorded caption-free single-robot reverse-GA and forward-GA
reference videos under GPU2 v5 third-anchor `reference-audit-20261110/`.
The independent forward replay matched the stored full joint/object positions
and velocities with maximum error zero for every checked field and passed the
original forward validator. This establishes faithful replay, not path quality.
The 17.81-second path has about 262.24 degrees of cumulative roll travel for a
90.03-degree net change; roll reaches +29.50 degrees and q3 reaches +51.30 degrees
before ending at the selected negative-bend grasp. At the stalled 8.04-second
reset: linear -94.50 mm, roll +6.63 degrees, q3 -59.18 degrees, q4 +4.85 degrees,
no proper segment contact and negligible object displacement. Two-segment
contacts first appear around 16.31 seconds. The forward search stops at its first
successful generation and does not optimize a successful reference for simplicity.
PPO consumes these as reset states, not action-imitation labels. A causal claim
that GA detours explain the curriculum stall requires further controlled tests.
No reward, reference or running PPO source was changed during the audit.


### First complete three-anchor attempt: two policies pass, third Home transfer fails — 2026-11-11 report

The GPU2 v5 D100 study has finished and stopped at the third policy gate. First
and second anchors each passed three unguided nominal-Home policy evaluations
and three independent command replays, with full lift and five-second retention.
Third anchor completed 2000 PPO updates in about 9400 seconds but failed all three
nominal-Home evaluations at 12.70 seconds with physical fault; these final runs
reported no camera-side contact. Its curriculum remained around reset bins16/17
(8.62/8.04 seconds along the verified forward reference), ending at bin16 with
nominal_ready=false. This is a failed Home-transfer result, not a reset validation
failure or proof of infeasibility. GA detours were observed in the separate audit;
their causal contribution remains untested. No command replay of the failed third
policy, phase alignment or interpolation validation was launched. No publication
was produced. All coordinator/training workers for this study have exited.

Collected all three actor NPZs, final evaluation/rollout evidence, successful
command-replay results for the first two and the third final failure video under
the original local generated study root. Failed actors remain labeled failed.
The compact final inventory is
`workbench/evidence/curated/planning/hug_pipeline_final_20261111.json`.
No further training or reward/reference changes were started for this report.

### 2026-11-11 — GA cumulative motor-command travel gates

- Added hard, per-candidate travel caps: linear 300 motor deg, roll 200 motor
  deg, seg1+seg2 500 motor deg. No overall sum cap, no net-motion subtraction,
  and no PPO reward or post-lift termination change.
- Reverse search, forward repair, fresh verification and reference ingestion
  enforce the same limits. Reports expose per-axis/group totals and rejection
  reasons; physically impossible endpoint budgets fail before scene creation.
- Source hashes change; prior plans/evidence are preserved, not silently migrated.
- Host focused motion/reverse-GA/reverse-curriculum/pipeline tests: 83 passed,
  1 skipped. `git diff --check` passed. GPU training/replay and canonical dev
  gates were not run for this change (no usable generated dev attachment).
  No remote update, push, or new training launched.

### 2026-11-11 — Revised GA motor-command travel limits

- Revised the three hard GA limits to linear 200, roll 200 and seg1+seg2 550
  motor degrees. The counter and both reverse and forward validation retain the
  existing semantics; there is no combined-total limit or PPO reward change.
- New boundary tests cover all three groups, the previous closest candidate at
  its former rejection tick and minimum endpoint travel for the three anchors.
  Focused host suite: 87 passed, 1 skipped; `git diff --check` passed.
- This local change invalidates old source-pinned plans; previous run evidence
  remains unchanged. No remote update, new GA, PPO or canonical dev gates were
  run for this revision; the generated development attachment is unavailable.

### 2026-11-11 — Reverse GA evaluates budget-disqualified paths to completion

- Travel excess at 200/200/550 motor degrees now permanently disqualifies a
  reverse-GA candidate without terminating evaluation. Continue to physical Home
  plus one continuous second of rest, the 20-second timeout, or an immediate-stop
  violation. Collisions, tilt/XY limits, wrong-face contact, command bounds/slew
  and the initial roll-direction rule remain immediate stops.
- Selection orders accepted success, budget-disqualified Home completion,
  unfinished paths without hard stops, then hard-stop failures. Among over-budget
  completions, minimize worst relative group excess and then total relative
  excess. No reward weights or new acceptance cap were introduced.
- Reports distinguish completion, eligibility and hard stops, preserve the first
  violation, accumulate later reasons and record full evaluated travel/excess.
  Progress and plans identify `reverse-completion-before-budget-ranking-v2`.
  Verified export still requires success without any rejection and three replays.
- Focused host reverse-GA/motion-budget/curriculum suite: 71 passed, 1 skipped
  (Torch-dependent check unavailable). Coverage includes continued motion after
  budget excess, final 660-degree travel instead of the initial 600-degree
  crossing, over-budget Home completion and breeding, later collision/tilt stops,
  and rejection of those completed failures at the curriculum boundary. Updated
  the curriculum fixture's stale violation-column count for the roll rule.
- Source pins require fresh plans. No GPU replay, new GA/PPO run or canonical
  development-container gate was executed for this revision; no usable generated
  development attachment is available in this checkout.


### 2026-11-12 — Three reverse paths verified under 200/220/600; D100 curriculum queued

The repeated D100 reverse-GA run used the same three exact mapper roots, seed42,
128 lanes and up to100 generations. With motor-command travel limits200/220/600,
anchors1/2/3 passed at generations18/24/61. All three independent nominal release
replays and all single-robot winner recordings passed for every anchor. Winner
linear/roll/bend-sum travel was187.50/180/591.64,180/180/594.39 and180/180/541.28
motor degrees. These results establish grasp-to-Home release paths, not forward
grasp/lift or interpolation success. Eighteen local MP4s passed ffprobe.
[Reverse-GA execution evidence](../workbench/evidence/curated/planning/hug_reverse_ga_200_220_600_20261112.json).

All three artifacts passed source/model/Home/root and portable reverse-curriculum
input validation without changing original evidence. A fresh
`hug-pipeline-d100-ga20261112` study pins those three anchors,512 environments and
2000 PPO updates per anchor, then30 validation/100 test interpolation samples plus
valid probes, with no additional held-out RL or anchor reselection. Source/model
pins and nominal-Home/forward-reference/reset-bank gates remain enforced.

At the recorded snapshot, catalog screening and first reverse reuse had passed;
the first forward reference remained at one proper contact despite matching q.
Its repair was running and PPO had not started. A detached supervisor waits for
all three forward/reset checks before learning; Luna monitors stages and videos.
No Home-to-Wrap policy, lift, aligned replay or interpolation success is claimed.
[Launch snapshot](../workbench/evidence/curated/planning/hug_d100_curriculum_launch_20261112.json).

### 2026-11-12 direct reverse-GA → PPO launch

The pipeline now consumes verified reverse full-state snapshots directly, with
no forward-GA prerequisite. Overlapping reset windows, nominal Home/original
object resets, free PPO actions and post-lift smoothing remain. Host regression:
88 passed, 2 skipped (Torch-dependent); canonical dev gates were not run because
no generated development attachment is available.

GPU2 study `workbench/evidence/generated/hug-direct-reverse-d100-fixed-v2-20261112`
pins the original three D100 anchors and 30/100 held-out cases using
`--reuse-selection`, with the three `ga-completion-20261112` releases reused.
Case 1 passed fresh 512-environment / 32-bin physical reset validation, nominal
Home and subset restoration checks; PPO reached update 4/2000 at handoff to
Luna. Updates are not a successful lift or interpolation certificate. Videos
are scheduled every 500 updates. Other GPU jobs were left running.
Two earlier launch attempts are preserved as interrupted: one reselected
anchors; the next invoked base Conda by resolving the venv Python symlink.
The v2 launch preserves the venv path and passed the exact package-version
check before starting; it does not reuse those attempts' physics checks.

### 2026-11-13 PPO linear budget and endpoint contract

The pipeline PPO/evaluation task now caps cumulative applied linear command
travel at 200 motor degrees, with Home at 180 degrees and final hold at 0±3
motor degrees. The complete five-second loaded hold requires both actual and
commanded linear position within tolerance. Budget breach latches the existing
terminal fault (-30) and blocks positive rewards. Other reward coefficients and
q3/q4 hints are retained. Curriculum starts debit `abs(u_reset - 180)` and expose
remaining budget as the 21st actor observation. Reset snapshots retain velocities;
reverse-teacher physics replay is not charged as a learned episode. New plan and
policy contracts reject legacy 20-input PPO checkpoints.

Host focused regression: 99 passed, 7 skipped for unavailable scientific packages.
Remote CPU verification loaded the edited repository sources in memory without
modifying the server checkout: 27 unittest checks passed, including Torch budget
tracking, subset restoration, 21-input actor construction, fake-environment
optimizer/export/resume and NumPy policy equivalence. This did not start Genesis
physics or a robot RL run. The three previous successful trajectories are all
rejected by the new travel cap. Reverse-GA digest remains
`f772220ad6d973a76afe972ff6d512b00c4bc34167181f3d83d21166208badba`.
Canonical dev gates were not run (no generated development attachment); new GPU
reset-bank acceptance and constrained learning remain to be run. Source changes
are local only; no server deployment or training restart was performed.

### 2026-11-14 reconstructed D100 hug adapter (local, uncommitted)

Added a 100 mm × 1.1 m cylinder OBJ and UI preset, a dynamic Sim cylinder without
logical attachment, Pilot-local cylinder fitting / exact mapper inversion / three
trajectory interpolation, and explicit prepare/execute intents. Existing executor
retains Home/telemetry/route/base guards; reconstructed plans additionally pin the
scan revision and Sim epoch. Runtime package includes anchors and checksummed
mapper kernel; Pilot Docker recipes include Node.js. Scope remains Sim-only.

Host fallback focused Pilot/UI/Sim-catalog/protocol checks: 232 passed, 1 skipped.
These are partial software evidence. Generated development attachment and pinned
Docker daemon are unavailable here; canonical required/extended, isolated release,
DDS multi-process and integrated Genesis camera/contact execution were not run.
No server deployment or physical movement was performed for this integration.

Push verification, 2026-11-14: combined host fallback setup/graphics/UI/Pilot/
interpolation/mapper checks: 210 passed; 3 ACL cases failed during fixture
`os.setxattr(system.posix_acl_access)` with EINVAL, before the code under test.
Connection runtime-status JavaScript regression command passed. Canonical Docker
and live deployment gates remain unrun; this push does not activate installations.
