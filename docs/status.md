# 구현 상태와 수용시험

갱신일: 2026-10-18. 이 문서만 마일스톤, 현재 완료 범위, 미해결 항목, 수동
acceptance gate를 소유한다. 구현 불변식은 `architecture.md`, wire 계약은
`dds_contracts.md`, 운영 절차는 `setup.md`와 `deployment.md`를 따른다.

## 현재 목표: 기존 기능의 운영 경로 완결

### 보행 전진성·지지 감쇠·착지 참조 TDD (2026-10-17, 기본값 실보행 재검증 중)

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
- [후보별 수치와 원시 bundle 해시](../workbench/evidence/curated/sim/walking_stance_reference_20261017.json)를
  보존한다. 다음은 새 source push/양쪽 공식 update 후 설치된 기본값 그대로
  GPU2의 전후좌우 긴 보행, 반복 정지, 두 카메라·팔 동작 및 비교 영상 확인이다.
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
- 재현: `workbench/research/debug/profile_sim_startup.py <robot.urdf>`를 설치된
  Sim/dev 환경에서 실행한다. `--performance-mode`는 static, `--profile`은
  비용 분석용이다. 캐시를 유지한 채 별도 프로세스로 반복한다.
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
