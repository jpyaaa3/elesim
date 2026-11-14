# 연구와 진단

이 문서는 repository-only 실험 규칙과 현재 유지하는 연구 기능을 소유한다.
연구 코드는 runtime role, Compose service, DDS broker 또는 release input이 아니다.
현재 유지하는 도구는 [Workbench 목록](../workbench/README.md)에서 확인한다.
옛 실행기·결과 전용 그림은 [로컬 archive](../workbench/archive/README.md)에 보관한다.

## 실행 경계

- runner/catalog: `workbench/tools/bench/` ([명령·판정·측정 범위](benchmarks.md))
- calculation worker: `workbench/research/experiments/`
- offline analysis: `workbench/research/analysis/`
- curated evidence: `workbench/evidence/curated/`
- generated evidence: `workbench/evidence/generated/` (Git 제외)
- manual debugger: `workbench/research/debug/`
- canonical environment: setup-generated persistent `elesim-dev`

```bash
elesim-dev python3 -m workbench.tools.bench list
```

각 결과는 source revision, effective config, role topology, endpoint/boot,
run ID, GPU policy, encoder, simulated/physical hardware와 통과하지 않은 수동
gate를 기록한다. Headless simulation은 물리 convergence, SROS2 enforce, NAT,
TURN relay 또는 display 동작의 증거가 아니다.

## 기존 그리퍼를 고정한 Hug 판정 진단

`hug_diagnostics`는 RL 학습이나 실기기 동작 없이 기존 `hug.py`의 기하 판정을
검토한다. **꼬다리는 기존 그리퍼**다. 길이·유무를 설계 변수로 탐색하지 않으며
두 구간 250 mm와 reference의 그리퍼 직선 근사 `n=0.155`를 고정한다.
이 수치는 실제 그리퍼 CAD의 접촉 형상이나 jaw 개폐 동작을 재현한다는 뜻이 아니다.

```bash
python3 -m workbench.research.analysis.hug_diagnostics \
  --output workbench/evidence/generated/hug-fixed-gripper --budget 20
python3 -m workbench.research.analysis.hug_diagnostic_report \
  --input workbench/evidence/generated/hug-fixed-gripper/report.json \
  --output workbench/evidence/generated/hug-fixed-gripper-view
```

첫 명령에는 기존 NumPy/SciPy/Shapely 환경이 필요하고 보고서는 stdlib만 사용한다.
`--reference /path/to/hug.py`로 읽을 신뢰 소스를 명시할 수 있다. 기본값은 Pilot에
vendoring된 reference다. GUI를 실행하거나 원본 solver를 수정하지 않는다.
출력 경로는 기존 것을 덮어쓰지 않는다. `--gripper-ratio`는 다른 실측 고정 입력을
명시하기 위한 값 하나이며 길이 조합 목록은 받지 않는다.

원본 solver는 물체에 맞춰 구간 길이도 바꾸므로 그 결과를 별도로 보관한다.
진단 탐색은 길이 고정 상태에서 첫 구간 접선 접촉, 두 번째 구간/그리퍼 측면
접촉과 그리퍼 끝점 접촉을 구분한다. 끝점에 측면 접선 일치를 강요하지 않는다.
관통 허용 오차는 물체 장축의 `1e-5`, 의도 접촉 간격은 그 두 배이며 몸통 간섭과
팔 self-intersection도 거부한다. 가상 포획선의 면적 중심 여유는 구판의 최소값을
유지한다. 각 후보에 원본 `evaluate_exact`와 P2 전방 조건의 결과도 그대로 기록한다.
구판의 무조건적인 polygon `crosses`와 허용 오차 기반 진단은 같지 않으며, 어느
진단 후보도 runtime의 성공 판정으로 승격하지 않는다.

원본 접촉쌍 선택 전에는 가상 닫힘선이 면적 중심에 너무 가까워 최소 여유를
만족할 수 없는 쌍만 제외한다. 양쪽 감김 방향과 연속적인 첫 접촉 위치를 탐색한다.
각 형상에 같은 시간·solve 횟수 상한을 적용하고, 미발견을 물리적 불가능으로
해석하지 않는다. 후보 수는 작업 공간의 크기나 물체별 성공률이 아니다.
선두 후보의 ±1 mm / ±1° 오차 여섯 개에 대해 두 굽힘을 ±5° 안에서 보정한다.
이것은 최종 정적 기하의 국소 탐색이며 연속 접근·회수 경로를 증명하지 않는다.

선택한 두 접촉 법선에서 평면 Coulomb force-closure와 정규화된 +Y 지지량을
계산한다. 마찰계수는 0/0.3/0.6의 가정이고, 각 접촉에 독립적인 정상력 1단위를
허용한다. +Y는 검증된 중력 방향이 아니며 면적 중심도 실측 질량 중심이 아니다.
실제 모터 결합, ID3/ID4 전류-접촉력 보정, 물체 강성·압축·파지 유지와 ID5 집기는
검증하지 않는다. 보고서의 `hold_verified`, `motor_load_margin`, `compression_margin`은
계속 `null`이다. 재구성 메쉬나 Robot 실행기에 이 진단을 연결하지 않는다.

## newhug 바늘 맵과 원형 band 정렬

독립 계산기의 정본은 `workbench/research/support/newhug.py`이며, 동일한 파일을
`/home/user/ws/newhug.py`에서도 실행할 수 있다. EleSim runtime에는 연결하지 않는다.
브라우저 UI는 다음 명령으로 연다.

```bash
python3 -m workbench.research.support.newhug
```

`바늘 맵 수집`은 기본 **256회 접촉 후보 평가**를 수행한다. 사용자가 iteration과
seed를 정하고, 선택적으로 수집 시간 상한을 설정할 수 있다. 0초는 시간 제한 없음이며
중단하면 부분 맵을 보존한다. 일반 `후보 계산`의 8초 시간 예산은 별도다. 지정 횟수에
도달하기 전에 탐색 후보가 소진되거나 중단되면 실제 수행 횟수를 기록한다.

0.6.0부터 **허용 마찰계수 μ**는 전역 컷오프다. 최소 필요 `mu_critical`이 설정값보다
작은 해만 팔 자세와 원본/정렬 바늘 맵, JSON/SVG에 남긴다. 경계와 같은 값은 force
closure가 아니므로 제외한다. 법선 각도 오차 입력·기본 2° 가산·오차 포함 μ와 관련
CLI/JSON 필드는 제거했다. 0.6.1부터 기록 μ 하한도 제거해,
**최소 필요 μ < 허용 μ**인 유효 해를 모두 기록한다. 작은 양수나 0인 최소 필요 μ도
허용 μ가 그보다 크면 남긴다. 무한대·누락 값은 기록하지 않는다.
상위 10개 미리보기로 잘라내기 전에 모든 기록 대상 해를 수집한다.
원형 수집은 정확한 원/로봇 면의 접선과
원/폴리곤 간격을 사용하므로 64각형의 이산 대칭에 임의 회전을 가정하지 않는다.
계산 후 μ를 변경하면 기존 후보·바늘·band를 함께 다시 거른다. 계산 중에는 μ를
고정하며, 늦은 이전 필터 응답은 버린다. 탐색 당시 μ 이하에서 이미 찾은 해는
컷오프를 낮췄다가 되돌려도 복원된다. 탐색 당시보다 높은 μ 범위의 새 해는 재계산이
필요하다. 현재 작업의 결과만 메모리에 보존하며 디스크 결과 캐시는 만들지 않는다.

0.5.1부터 접촉 허용 그룹을 `seg1`, `seg2`, `gripper_base`, `gripper_claw`로
명시한다. 카메라와 **하우징·웨지**는 물체 접촉이 금지되며 각각 기본 1 mm 보호
간격을 검사한다. 몸체는 기존의 관통 허용 오차와 별개로 검사하므로 보호 간격을
0으로 낮춰도 접촉은 통과하지 않는다. UI의 `몸체·웨지 보호 간격`이나
`--body-clearance-mm`으로 조정한다. 직접 접촉 구성, 역접촉 피팅·보정, 정확한 원의
수집과 최종 판정에 같은 조건을 적용하며 JSON에 보호 부위별 실제 간격을 기록한다.

각 바늘은 **GO2 body 기준점**의 주차 선분이다. 팔 첫 관절 위치와 혼동하지 않도록,
현재 URDF의 중립 전방 오프셋 343 mm와 리니어 범위 [-173.333, 0] mm를 명시한다.
`p_GO2 = p_node0 - (0.343 + linear) * forward`이며 1.5°/mm·최대 260° 결정을 유지한다.
팔 자세·접촉점·법선·μ·기하 검사와 각 선분 끝의 linear 값을 함께 저장한다.

원형 `끌어모으기`는 하나의 주차 반경에 모든 바늘을 맞추지 않는다. 각 선분이 차지하는
반경 구간을 집계하고, 최대 겹침 수의 설정 비율(기본 **80%**) 이상이 지지하는 연속
band를 선택한다. 여러 덩어리가 있으면 가장 높은 겹침 수, 누적 지지량 순으로 선택한다.
안쪽·바깥쪽의 희박한 부분을 잘라낸 뒤 각 남은 구간의 중점을 같은 둘레각으로 회전한다.
각 바늘의 반경은 그대로 보존되며, band를 두 번 통과하면 두 구간 모두 남긴다.
band 경계·합의 강도·정렬 둘레각은 UI에서 바꿀 수 있다. 원본 맵은 유지하고
`gathered`에 잘린 구간, 보존된 전체 선분, linear 부분 구간과 제외 개수를 기록한다.
빈틈을 보간하거나 band 전체가 도달 가능하다고 판정하지 않는다.

```bash
python3 -m workbench.research.support.newhug --solve --record-needles \
  --shape circle --width-mm 100 --iterations 256 --mu .5 \
  --output workbench/evidence/generated/newhug-map.json \
  --needle-svg workbench/evidence/generated/newhug-map.svg \
  --gathered-svg workbench/evidence/generated/newhug-band.svg
```

출력 파일은 새 경로여야 한다. `--mu .3`, `--seed 42`,
`--map-time-limit 30`, `--gather-support .6`, `--gather-band-mm 300 400`으로
기록·탐색·band 조건을 조정한다. UI의 JSON 저장에도 원본 맵과 정렬 결과가 모두 들어가며,
SVG 저장은 현재 선택한 팔/원본 바늘/정렬 바늘 화면을 저장한다. 바늘을 클릭하면
대응하는 팔 자세를 표시한다.

이는 현재 standalone 팔·하우징·카메라를 검사한 **정적 평면 해 지도**다. GO2 본체·다리·
플레이트 충돌, 결합 구동의 힘 한계, 접근 경로, RL 포획 성공과 실제 들어올리기는
검증하지 않는다. band 지지량은 수집된 해의 겹침 수이며 성공 확률이 아니다.

## hugmap 고정 몸체·물체 배치 지도

`workbench/research/support/hugmap.py`는 한 파일로 복사해 실행하는 독립 GUI/CLI다.
Python 3.10+, NumPy가 필요하며 GUI에는 Tkinter가 필요하다. 0.3.0부터 SciPy는 필요 없다.
EleSim/newhug 모듈이나 별도 모델 파일은 런타임에 읽지 않는다.

```bash
python3 workbench/research/support/hugmap.py
python3 workbench/research/support/hugmap.py --table \
  --diameter-min-mm 50 --diameter-max-mm 250 --diameter-step-mm 5 \
  --dx-mm 20 --dy-mm 20 --mu .5 --workers 4 \
  --output workbench/evidence/generated/hugmap-circle-table.json
python3 workbench/research/support/hugmap.py --resume \
  workbench/evidence/generated/hugmap-circle-table.json
```

몸체는 newhug의 130×233 mm 하우징 직사각형을 대용으로 사용한다. 실제 Go2
본체·다리 모델이 아니다. 몸체 중심을 원점으로 고정하고 물체 중심을 전방(+y)
반경 500 mm 왼쪽 전방 사분원(x<=0, y>=0)의 격자에 배치한다. 기본은 지름 50~250 mm 원을 5 mm 간격으로
계산하는 41개 크기의 테이블이다. 기존 단일 100 mm 맵 CLI는 `--run`으로 유지한다.
리니어는 중립 고정, 두 굽힘만 최적화한다. 좌표 형식은 물체의 `(x,y,theta)`를
유지하되 원형에서는 theta=0 한 번만 평가한다. 비원형 접촉 솔버는 미구현이다.
GUI의 theta 해상도 항목은 원형에서도 입력 가능하며 값은 저장한다. Circle에서는
계산에 사용하지 않고 theta=0 한 번만 푼다. JSON에는 theta=0 표본만 보존하고, reader는 원의 회전 대칭을 사용해 표시를 확장한다.

기본 위치 간격은 x/y 각 20 mm, 허용 μ는 0.5다. 지름당 516개, 전체 21,156개
격자에서 크기별 몸체·웨지 보호 간격을 적용한다. 100 mm일 때는 제외 102개,
수치 풀이 904개다. 배치당 초기값 64개를
배열로 한꺼번에 풀고 중복 제거한 유효 후보 최대 16개를 남긴다. 각 초기값은
최대 80회 접촉 잔차 평가를 사용한다. 초기값별 감쇠·수락 상태를 독립적으로
갖는 bounded damped Gauss-Newton과 해석적 거리 미분을 사용한다. `attempts`는
병렬 평가한 전체 초기값 수다. 최종 기하·충돌·마찰 검사는 기존 판정을 사용한다.
발견한 후보 중 최소 μ의 자세를 기록하며 전역 최적 또는 해 부재를 증명하지 않는다.
접촉 허용 부위, 카메라/몸체 보호와 자기 충돌 검사는 newhug에서 가져왔다.

CPU 프로세스 풀은 affinity/cgroup 할당을 고려해 자동 선택하며 최대 32개다.
`--workers`로 지정할 수도 있다. 고정 크기 작업 묶음과 묶음 내 이웃 해 초기값을
사용하고, 자식의 BLAS 스레드는 1개로 제한한다. 실행 완료 순서로 저장하므로
JSON 배열 순서가 아니라 좌표로 조회해야 한다. 41개 크기에 하나의 process pool을
재사용하고, 작업 대기열은 worker 수에 비례해 제한한다. 테이블 결과는 메모리에
모아 원자적 JSON 체크포인트로 저장하므로 고해상도에서는 메모리 사용도 증가한다.

JSON에는 설정/단위/형상, 각 배치의 `x_mm,y_mm,theta_deg`,
`seg1_deg,seg2_deg,mu_min`, 접촉 정보와 탐색 상태가 들어간다. seg 각도는
5노드 그룹의 총 굽힘이며 노드당 각도는 그 1/5다. 상태는 `found`, `not_found`,
`excluded`로 구분한다. `not_found`는 불가능 판정이 아니다.
테이블은 schema v2 `hugmap-table`이며 공통 `metadata`와 지름별
`maps: [{diameter_mm,status,results,summary}]`를 한 JSON에 저장한다.
GUI `New table`은 기존 파일을 덮어쓰지 않는다. `Resume table`/`--resume`은
파일의 설정을 읽고 완료된 좌표를 건너뛴다. 일부만 저장된 작업 묶음은 앞선 저장
해를 warm start로 이어 받아 연속 실행과 같은 계산 순서를 유지한다.
크기별 `pending/running/complete/cancelled/failed` 상태를 기록한다.

약 10초마다, 각 지름 완료 시, 정상 종료/중지 시에 임시 파일을 fsync하고 원자적으로
교체한다. 강제 프로세스 종료 뒤에는 마지막 완료 체크포인트에서 재개할 수 있다.
동시 writer를 차단하는 `.lock` 보조 파일은 데이터가 없는 OS 잠금용이며,
프로세스 종료 시 잠금은 해제된다. JSON 데이터는 한 파일이다.
재개는 모델/제약/솔버 revision이 같은 파일만 허용한다. 전체 재개 테스트는 작은
격자로 수행했으며 기본 21,156개 전체 테이블을 생성했다고 보증하지 않는다.
기존 `--run` 단일 맵(schema v1)은 `.partial` 스트리밍과 새 파일 발행 방식을 유지한다.

### hugmap JSON heatmap reader

별도 `workbench/research/support/hugmap_reader.html`을 브라우저로 열고 **Open JSON**으로 파일을 선택한다. 서버나 인터넷 연결은 필요 없다.
테이블을 열면 `Diameter` 슬라이더에서 저장된 지름을 선택한다. 기존 단일 맵도 그대로 읽는다.
색 범위는 테이블 전체 지름과 theta에 걸쳐 공유한다.
`seg1`, `seg2`, `mu_min`을 색으로 선택하고 **X/Y/Theta를 세 공간축**으로 표시한다.
Theta 단면을 고르는 UI는 없다. 모든 theta의 저장된 점을 한 3차원 공간에 함께
투영한다. 원형은 저장된 theta 간격으로 `[-180,180)` 층에 같은 값을 복제해 표시한다.
기본 5도는 72층이다. 원본 표본이나 저장된 발견 개수는 바꾸지 않는다.
비원형이나 이미 여러 theta를 가진 파일에는 이 복제를 적용하지 않는다.
드래그로 회전, 휠로 확대,
`Reset view`로 초기 시점 복귀가 가능하다. X/Y는 같은 mm 축척을 사용하고
Theta(deg)는 별도 표시 축척을 사용한다.

색 범위는 파일 전체에서 공유하고, seg 각도는 0을 중심으로 파랑에서 빨강,
μ는 연속 색으로 표시한다. 기본은 발견된 해를 표시한다. `Show not-found / excluded`를
켜면 미발견(연회색), 제외(진회색)도 추가한다. 미저장 점은 그리지 않으며 보간하지 않는다.
마우스를 점 위에 올리면 좌표·각도·μ·상태를 확인한다. 깊이순으로 그린 앞쪽 점을 선택한다.
독립 HTML의 Canvas와 JavaScript로 3D 투영을 구현하며 외부 라이브러리나 CDN 의존성은 없다. UI 문구는 영어다.

### mapper의 Home→Extend Arm 충돌 기준

공유용 `/home/user/ws/hugmap/mapper.html`은 최종 파지 자세의 투영 충돌과
Home→Extend Arm 전개의 충돌 envelope를 함께 적용한다. XY·q 화면의 작도,
픽셀 역산 보완, RL용 `hug_c.source` 함수 추출이 같은 `objectCollision`을 쓴다.
`finalObjectCollision`은 최종 자세만 따로 검사하며 기존 팔 접촉의 1 mm 형상
근사 허용, 카메라/하우징/웨지 1 mm 보호 간격, 자기 충돌 0.05 mm 수치 허용을
유지한다. 이 1 mm 접촉 근사를 전개에 적용하지 않는다.

전개는 실제 앱 매핑의 Home(linear −120 mm, roll 0°, q3 −170°, q4 +170°)에서
Extend Arm(linear −10 mm, roll 0°, q3/q4 0°)까지 네 명령을 선형 보간한다.
이는 명시한 준비 동작이며, 실제 Pilot의 버튼 명령에 대한 동적 제어 경로와 같다고
가정하지 않는다. q3/q4는 총각이고 계산 시 다섯 관절에 나눈다. 실제 URDF의
팔·카메라·하우징·웨지와 고정된 Go2 몸체/다리 충돌 형상을 사용한다. 각 부품이
전개 중 차지하는 XY의 convex hull을 구해 물체 원과 검사한다. 물체는 이 형상의
높이를 가로지르는 고정된 수직 원기둥으로 가정한다.

기준은 `distance(object centre, preparation envelope) > D/2 + 1 mm + ε`다.
현재 340구간에서 관절 변화와 실제 체인 길이로 계산한 표본 사이 보정 ε는
약 **0.04268 mm**다. 원통/구 충돌 primitive는 외접 다각형으로 보수적으로
근사한다. 부품별 convex envelope는 빈 공간을 포함할 수 있으므로 회색 판정은
이 준비 동작에 대한 보수적 제외다. 모든 가능한 접근의 불가능성을 뜻하지 않는다.

`Home → Extend Arm envelope` 체크박스는 XY의 제외 영역 표시만 켜고 끈다.
충돌 필터는 계속 적용된다. 회색 후보의 설명에는 최종 자세와 전개 중 어느 조건에
걸렸는지, 부품명과 signed gap을 표시한다. 이후 roll·감싸기·물체 이동·들어올리기는
검사하지 않으므로 전개 필터 통과를 전체 접근 성공으로 해석하면 안 된다.

모델이나 프리셋이 바뀌면 현재 HTML을 입력으로 재생성한다. 출력 HTML에는 필요한
형상이 모두 포함되며 공유받은 사용자는 Python이나 서버 없이 파일 하나로 실행한다.

```bash
PYTHONPATH=payload/runtime/common/protocol:payload/runtime/docker/sim/app:. \
python3 -m workbench.research.analysis.rebuild_mapper_collision \
  /home/user/ws/hugmap/mapper.html \
  --output workbench/evidence/generated/mapper-collision/mapper.html \
  --report workbench/evidence/generated/mapper-collision/build.json
```

이 명령은 검토할 출력 파일을 생성하며 입력 HTML을 바꾸지 않는다. NumPy/SciPy와
앱 설정 로더는 재생성 시에만 필요하다. 임의 상수로 114 mm 선을 넓히거나, 기존
JSON에서 실패 점을 골라 추가 제외하는 방식이 아니다.

## newhug와 고정 배치 Wrap 과제 비교

독립 실행형 `/home/user/ws/newhug.py`의 프로파일·FK·마찰 점수를 명시적으로
읽어 Wrap의 지름 100 mm 원기둥, 고정 로봇 베이스, 현재 260° 리니어 한계와
학습 manifest의 결합 굽힘 한계에 맞춘다. 자유로운 평면 base 배치는 금지한다.

```bash
python3 -m workbench.research.analysis.newhug_wrap_comparison \
  --solver /home/user/ws/newhug.py \
  --output workbench/evidence/generated/newhug-wrap-comparison
```

NumPy/SciPy/PyYAML이 있는 환경에서 실행한다. 원본 solver나 runtime을 수정하지
않으며 출력 디렉터리는 새 경로여야 한다. 원은 polygon 근사 대신 정확한 원으로
접촉시키고 두 로봇 면의 접선 방정식을 푼다. 한 굽힘을 표본화하고 나머지 굽힘의
횡방향 배치 방정식을 수치적으로 풀므로 전역 최적성·불가능성 증명은 아니다.
기본 +90°와 실험 runner에서 프로파일을 반사한 -90°를 각각 검사한다. -90°는
현재 standalone UI에 추가된 기능이 아니다. 별도 URDF 3D FK와 실제 camera OBJ의
360방향 support로 위치·롤 반사를 교차 검사한다.

현재 runner는 오차 가산 없이 최소 μ만 계산한다. 아래는 2026-11-02 당시 버전의
보존된 실험 결과다. -90° 해는 리니어 -75.63 mm, 마디당 -12.53°/-32.90°의 C자이며
node4/node9에서 접촉한다. 이상적인 평면 두 점 마찰계수 하한은 0.1361,
법선 오차 2°를 넣으면 0.1718이고 실제 계수는 이 하한보다 커야 한다.
카메라 간격은 67.12 mm다. 같은 C자를 +90°에 놓으면 카메라가 약 9.40 mm
관통하여 탈락하고, +90°의 선두 해는 S자에 가깝다. 설정 주석의 18°/36° 예시는
현재 기하에서 결합 한계·물체 관통에 걸리므로 성공한 RL 궤적의 정답으로 쓰지 않는다.

[비교 그림](../workbench/evidence/curated/planning/newhug_wrap_20261102.png)과
[조건·수치·소스 SHA](../workbench/evidence/curated/planning/newhug_wrap_20261102.json)는
정적 자세 실험의 증거다. 새 정책 rollout/Genesis 접근·들어올리기, 모터 부하·압축·
실물 유지 검증을 수행했다는 뜻이 아니다. 서버 SSH는 22 연결 거절/2222 인증 실패로
사용하지 못했고 host에는 Torch/Genesis가 없어 동적 비교는 하지 않았다.

## 저장 점군의 메쉬 해상도 비교

저장한 scan의 메쉬 해상도 비교는 관측/생성 점군 NPZ 하나를 그대로 사용한다.
`compare_voxel_mesh`는 10 mm 반경/이웃 8개 필터를 고정해 20/10/5 mm로 만들고,
5 mm를 비교하기 위해 점유 voxel 예산 8,000을 명시한다. Runtime의 기본 예산
4,000은 바꾸지 않는다. Truth는 메쉬를 만든 뒤 평가/원본 표시에서만 사용한다.
이 명령은 촬영·completion 추론·Sim 씬 교체를 실행하지 않는다.

```bash
python3 -m workbench.research.analysis.compare_voxel_mesh \
  --input source-target.npz --truth sim.json --reference jar-target.obj \
  --output workbench/evidence/generated/voxel-comparison
```

필요한 scientific dependency가 이미 있는 환경에서 실행하며 host에 새로 설치하지
않는다. 각 해상도의 유효 scene bundle, 원본 배열, OBJ/hash, CPU 재메쉬 시간,
표면까지의 vertex 거리, PNG 및 network dependency 없는 HTML 회전 viewer를 만든다.
명령은 기존 출력 디렉터리를 덮어쓰지 않는다. 거리 지표 하나를 표면 완전성으로
해석하지 않으며 별도 runtime/GPU/접촉 검증을 대체하지 않는다.

`compare_surface_mesh`는 동일 점군의 voxel 10/5 mm와 Poisson/거리장 메셔를
비교한다. Open3D `meshing` extra를 갖춘 명시적 환경이 필요하다. 사용자 승인으로
project 전용 venv에 Open3D 0.20.0과 고정된 NumPy/SciPy를 설치해 실제 계산을
검증했다. 이 환경은 setup-generated canonical development attachment의 대체가 아니다.

```bash
python3 -m workbench.research.analysis.compare_surface_mesh \
  --input source-target.npz \
  --reference-report workbench/evidence/curated/sim/scan_cycle_jar_20261031.json \
  --reference workbench/evidence/curated/sim/jar_scan_target_20261031.obj \
  --output workbench/evidence/generated/surface-comparison
```

연결 수는 점유 voxel 인접이 아닌 삼각형의 실제 변/꼭짓점 기준으로 분리해
기록한다. Mesh에서 면적 가중 30,000점을 뽑아 원형까지의 거리와 역방향 sampled
거리, 10 mm F-score도 계산한다. 역방향 지표는 정확한 point-to-triangle 거리가
아니며 샘플링 오차를 포함한다. Truth는 메셔 실행이 끝난 뒤 evaluator에서만 읽는다.
`--poisson-options options.json`으로 명시적 모델 옵션을 비교할 수 있다.
`--topology-probe`는 truth 없이 고정된 법선/깊이 후보의 품질 검사만 실행하고,
`--raw-diagnostics`는 거부된 native 출력의 데이터와 결함을 기록한다. 이 두 진단
모드는 truth/reference 입력을 받지 않으며 loadable SceneArtifact를 만들지 않는다.
원본 surface의 수치 진단에는 `remesh_resolution_m: 0`을 명시한다. 모든 실행은
새 출력 경로를 요구하고 실패 원자료를 덮어쓰지 않는다.

## Pitch-preview gaze

상태: 구현됨. runtime mode는 `pitch_preview`다. GO2 보행 중 eye-in-hand
camera를 안정화하며 Look–Aim–Grasp와 별도다.

```text
s = [u_err, v_err]
du = [delta_roll, delta_s1, delta_s2]
du = -(J^T Q J + R)^-1 J^T Q (s + preview_term)
preview_term = [0, b_pitch * pitch_rate_lead]
pitch_rate_lead = filtered_pitch_rate + tau * pitch_acc_est
```

- pitch rate 우선순위: timestamp가 유효한 body angular velocity, 이어서
  연속 body-pitch 차분.
- invalid configuration은 시작 전에 실패한다.
- 실행 중 일시적 입력/solve 실패는 해당 tick만 UV로 fallback하고 이유를
  기록한다.
- metrics는 requested/actual mode, use/fallback ratio, pitch rate/lead,
  preview term, joint delta와 solve time을 포함한다.
- 비교 run은 동일 gait, velocity, target, duration과 run ID를 사용한다.

구현은 `elesim_pilot/gaze/{preview_lite,preview_mpc,gaze_service}.py`와
`elesim_pilot/pick/gaze_actions.py`가 소유한다. 이 방식은 one-step
regularized solve이며 gait-phase model이나 multi-step horizon이 아니다.

## GO2 MPC contact 진단

현재 convex MPC를 교체하기 전에 controller, plant, contact 오차를 분리한다.

아래는 2026-09-29 contact 진단 당시의 baseline이다. 현재 실제 설정은
공통 bench의 configuration/manifest로 확인한다.

- Genesis 1.2.0 Newton rigid solver, 50 iterations
- surface friction 0.8, rectangular MPC constraint 0.55
- circular Coulomb projection 후 torque 생성
- normal GRF cap 180 N/foot
- 기본 50 Hz에서 effective MPC timestep 0.02 s
- armature/damping/friction/range startup readback 검증

```bash
elesim-dev env ELESIM_WALKING_METRICS=1 ELESIM_RUN_ID=mpc-baseline elesim-sim
```

산출물은 `<run>_walking.csv`, `<run>_contact.csv`, `<run>_meta.json`이다.
contact readback은 10 Hz sample cadence에서만 수행한다.

이 명령은 `elesim-dev` 안에서 별도 Sim 프로세스를 띄우는 standalone 실험용이다.
Connections가 관리하는 실제 Sim에서 측정하려면 Sim 호스트의
`containers/compose.instances.yaml`에서 대상 system의 `io.elesim.role: sim`
서비스 `environment`에 아래 값을 추가하고, 호스트의
`elesim-instance <system> up --no-build`로 해당 인스턴스를 갱신한다.
이 파일은 Connections 재등록/교체 때 다시 생성되므로 일회 측정에만 쓴다.

```yaml
ELESIM_WALKING_METRICS: "1"
ELESIM_RUN_ID: "mpc-slip-20260929-a"
```

Connections에서 Sim이 다시 ready가 된 뒤 일정한 속도로 10–20초간 한 방향만
걷게 하고, Sim 컨테이너의 `/opt/elesim/logs/walking_baseline/`에서 해당
`_contact.csv`를 개발 checkout의 `logs/walking_baseline/`으로 복사한다.
예를 들어 Sim 호스트에서 다음처럼 찾고 복사할 수 있다.

```bash
mkdir -p logs/walking_baseline
sim_container=$(docker ps --filter label=io.elesim.system_id=test --filter label=io.elesim.role=sim --format '{{.Names}}')
docker cp "${sim_container}:/opt/elesim/logs/walking_baseline/mpc-slip-20260929-a_contact.csv" logs/walking_baseline/
```

`test`와 run ID를 실제 system/run ID로 바꾸고, 복사한 뒤 개발 checkout에서
리포트를 생성한다.

```bash
elesim-dev python3 workbench/research/analysis/analyze_contact_metrics.py mpc-slip-20260929-a
```

이 명령은 Markdown과 JSON을 `logs/walking_baseline/`에 쓴다. 같은 CSV는
`python3 workbench/research/analysis/analyze_contact_metrics.py <run-id> --log-dir <csv-폴더>`로
어느 checkout에서도 분석할 수 있다. 개발 서비스에서 `elesim-sim`을 직접
실행하면 Connections Sim과 별도 프로세스이므로, 그 결과를 실제 Connections
세션의 증거로 혼용하지 않는다.

판정 순서:

1. raw `friction_ratio > 1`: optimizer/contact-cone mismatch.
2. normal force는 유사하지만 tangential GRF error가 큼: contact realization
   또는 collision manifold 문제.
3. desired/actual GRF가 유사하지만 stance slip이 큼: foot point, Jacobian,
   stance feedback/WBC 문제.
4. 조기 torque saturation 또는 GRF divergence: plant dynamics, payload,
   force limit 문제.

동일 command duration에서 velocity error, stance slip, GRF error, body
roll/pitch RMS, torque RMS, MPC solve time과 RTF를 비교한다. 이 검증 전에는
NMPC/WBC 교체를 개선으로 판정하지 않는다.

### Housing center correction (2026-11-03)

The `062222.stp` housing (`하우스_CPY_ll`) has side planes at +/-65 mm
about the roll axis, with the bottom 155 mm below that axis. The June 24
`mesh diet` asset introduced a +1.5 mm lateral offset. Both shipped camera
profiles now translate the housing mesh by -1.5 mm in URDF Y; roll height
remains 155 mm. ZED Mini inertial origins and planning collision spheres
follow the same correction. Research `newhug.py` and `hugmap.py` embed the
corrected +/-65 mm footprint and updated source hashes.

Hugmap 0.4.1 uses the housing center with x=0 on the roll axis (the forward
origin remains at the housing midpoint). Old tables retain their original
geometry and coordinates as historical evidence: they are readable, but
cannot be resumed with corrected geometry. The HTML reader labels the old
1.5 mm housing offset. Recompute tables for the corrected model; shifting
old coordinates alone does not revalidate housing collisions. Historical
curated/generated evidence is not rewritten as if calculated with this fix.

Hugmap 0.5.0 computes the left-forward quadrant with roll +90 degrees.
JSON `metadata.symmetry` specifies the circle-only reuse rule: reflect x,
keep y/seg1/seg2/mu, and use roll -90 degrees. Results store computed poses
only; x=0 is not duplicated. This selects a roll per side, rather than
preserving every solution of the earlier fixed-roll semicircle search.
The legacy radius metadata key is retained for reader compatibility; the
new `interest_region` and `lateral_half_plane` explicitly restrict the grid.
Old semicircle tables remain readable but cannot resume into a quadrant run.


## External RL tasks and two-band interpolation

The old cylinder Wrap task is now repository-only. See
[RL integration](rl-integration.md) for train/eval/export commands using
`python -m workbench.research.experiments.wrap_grasp.<command>`.
Its configuration, reward, curriculum and diagnostics live together outside
payload. The generic Sim API accepts caller-created environments and does not
know what object is being grasped. Existing checkpoints remain task artifacts;
Pilot still consumes the existing exported policy/interface pair.

The first offline interpolation tool runs with NumPy and the Sim package:

```bash
python -m workbench.research.experiments.hug_interpolation \
  --anchors anchor-a.npz anchor-b.npz anchor-c.npz \
  --target target.json --output candidate.npz
```

Anchors use `elesim-command-trajectory` NPZ schema v2 with numeric `times_s` and `commands`,
Unicode `channels`, and JSON-string `metadata_json` / `diagnostics_json` arrays. Channels are `linear_m`, `roll_rad`,
`theta1_rad`, `theta2_rad`; bend angles are **per node**, not total degrees.
Five nodes per segment are assumed by this experiment. Metadata must supply
`case_id`, `xy_mm`, `band`, `roll_deg`, `diameter_mm`, a local connected `branch`,
and matching `simulation_context` (model digest, coordinate convention, physics,
controller and initial-state conditions). `diagnostics.replay_success` must be
true for every anchor, based on physical replay of that original trajectory.

Target JSON supplies that same metadata plus `start_command`, `goal_command` and
`duration_s`. The goal comes from the geometric solution at the target point.
The tool blends time-normalized residuals around each path's start-to-goal line,
using barycentric weights inside the three XY anchors. Diameter and roll remain
fixed. It refuses extrapolation, incompatible contexts, positive seg2 commands
and bend-limit violations. This is a local candidate generator, not a global
band fit or an event-aligned/contact-feedback controller. Its output explicitly
has `physics_validated=false`; no arbitrary gap is certified as connected.

`elesim_sim.rl.replay.replay` accepts a task-owned reset/command/step/measure/
evaluate adapter and samples the trajectory at physics ticks. The task defines
success, including any hold/tug protocol. Replay never trains. An output path is
created exclusively so previous evidence is not overwritten.

`python -m workbench.research.experiments.hug_interpolation.prepare kyung.json
--output pilot.json` selects three training anchors per roll at D=150 mm and
separate validation/test points in a small fully sampled triangle. It requires
schema v3 and refuses legacy fixed-roll maps. The source digest and split are
saved before training. These six anchors cover only two local triangles.

The external two-band MDP and configure/check/train/collect/replay/interpolate/
validate CLI are now implemented. Follow [the executable experiment runbook](hug-rl-experiment.md)
for the complete command sequence, fixed physical assumptions, original-path
replay gate and held-out evaluation. The task uses its own goal/contact/hold
reward and criterion, not the old Wrap reward. Physical execution is still
unverified here: no GPU training or Genesis replay was run during implementation.
The supplied v3 map is copied to
`workbench/evidence/generated/hug_interpolation/input/map.json`; `gaga.json` was
not substituted. Generated inputs/results are ignored by Git.


`workbench.research.experiments.hug_reverse_rl.experiment` provides explicit
`prepare`, `check`, `train` and `evaluate` commands for a verified reverse-GA
reset curriculum. It learns lift/hold near the solver grasp and expands starts
toward Home using clean frontier episode success. Full-state prefix replay and
selective-restore checks gate training. Policy NPZ exports and videos use the
existing actor/renderer; evaluation includes Home with the original object pose.
[Commands and evidence boundaries](../workbench/research/experiments/hug_reverse_rl/README.md).

### Mapper C triangle overlay

`ws/hugmap/mapper.html` can embed the current study's exact three anchors and a
standalone worker for function-domain triangle selection. `Current RL anchors`
restores the physical-admission-screened study triangle; `Select C triangle`
samples the chosen diameter, mu cutoff and total-angle resolution through the
same mapper/CAD/preparation functions, then maximizes occupied q-area inside the
triangle. Clearance, worst anchor mu and conditioning are the tie-breaks.
Browser random starts use a portable JS stream; the small-catalog exhaustive
method and imported actual RL vertices match Python exactly. General seeded
searches share the objective/search procedure without a bitwise RNG promise.

The q plot shows the interpolation triangle without vertex letter labels. The XY
view maps a barycentric mesh of that same q triangle through the actual contact
model at no more than 0.5° spacing. Corners and centres must pass the same friction,
CAD and preparation filters; contact features/roots are kept separate, and missing
cells are not joined. It fills the mapped mesh and draws its boundary instead of
connecting three XY anchors with chords. This is a sampled visual projection,
not a continuous feasibility or physical-admission certificate for the interior.
Projection runs in a separate worker and is cached with the active selection.
Click an anchor to inspect its total q3/q4 angles. Parameter changes
invalidate the displayed selection and cancel stale worker replies. Construction
remains responsive during selection. The page stays one standalone HTML file.

To embed or refresh a completed physical catalog and selected study anchors:

```bash
PYTHONPATH=payload/runtime/common/protocol:payload/runtime/docker/sim/app:. \
python3 -m workbench.research.analysis.mapper_triangle /path/to/mapper.html \
  --study workbench/evidence/generated/STUDY --output /path/to/updated-mapper.html
```

With `--study`, the builder checks the study/kernel match and recomputes the exact
Python triangle. Without it, a presentation refresh preserves an already embedded
study, its vertices and physical-screen results. Geometry and collision kernels
remain unchanged. Construct band uses 12/24/48 ms work slices for Slow/Normal/Fast,
with Fast selected by default; scan resolution remains 0.1° and every slice yields
to the Stop/control events. The builder is idempotent. Browser-only
candidates have no Genesis admission test; the UI labels that distinction.
