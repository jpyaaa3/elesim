# 4DOF arm multi-view scan

2026-10-27. 이 실험은 팔을 움직여 **실제로 보이는 표면을 더 확보하고 하나의
점군으로 합치는** 단계다. AdaPoinTr의 추측으로 미관측 면을 채우는 단계는
포함하지 않는다. 현재 runtime UI의 자동 스캔 기능과 연결한 상태는 아니다.

## 획득과 통합

1. 장착 카메라의 RGB-D에서 물체 mask를 얻고 intrinsics로 역투영한다.
   이번 물체 분리는 red connected-component 기준선이다. 첫 유효 optical
   frame을 scan 좌표계로 고정하고 원본 센서는 그대로 보관한다.
2. 명목 4DOF FK와 관절 한도에서 Halton 후보 12,000개를 만든다. 관측 점군의
   중심·범위가 화각 안에 있고, 기존 시야와 방향이 다른 후보를 고른다.
   현재 최소 방향 차이는 4도, 인접 관측과의 최대 차이는 42도다. 실제 URDF
   관절을 보간해 움직인 뒤 장착 camera에서 다시 찍는다. 최대 20회이며 마지막에
   첫 자세를 재방문한다. 물체의 정답 중심·종류·크기는 후보 평가에 주지 않는다.
3. local point-to-plane ICP는 진행 중 미리보기와 겹침 확인에 사용한다.
   평면이나 원기둥에서 독립적인 촬영별 ICP 오차를 누적하면 면이 두꺼워질 수
   있으므로 최종 결과는 다른 방법으로 재계산한다. 여러 관측이 **같은 bend
   영점 오차 두 개**를 공유한다고 놓고, 관측끼리 겹치는 표면의 법선 방향
   거리를 robust least squares로 줄인다. 최적화할 때 정답 mesh는 읽지 않는다.
4. 추정한 공통 오차로 각 촬영의 상대 camera FK를 다시 구하고, 원본 점을
   한 번씩 재투영한 뒤 3 mm voxel에서 평균한다. 겹침 graph에 연결되지 않는
   관측과 관측성이 부족한 fit은 거부한다. Source/boot/calibration 변경,
   역전 sequence와 제한 초과도 섞지 않는다.

이 fit은 scan 동안 일정한 굽힘 영점 오차를 가정한다. Linear/roll의 공통
영점 이동은 관절 한도에 걸리지 않는 구간에서 이 상대 관측의 gauge이므로
복원했다고 주장하지 않는다. 한도에서 clipping되는 효과를 일반적인 영점
식별 근거로 이용하지도 않는다. 절대
world XYZ/yaw 보정이나 자세에 따라 바뀌는 처짐의 일반적 해결책도 아니다.
기존 기준면 기울기 보정과는 별개이며, 향후 world 표현에는 좌표 정렬을
명시적으로 연결해야 한다.

## TDD에서 고친 실패

- 동일한 시야를 반복했는데 ICP가 조금씩 이동하며 새 voxel을 만들던 문제:
  같은 명목 자세에서 이전 상대 변환을 유지한다.
- 새로 보인 면을 기존 map에 맞지 않는 점으로 간주하던 문제:
  표면 방향이 맞는 겹침을 사용하고 미관측 표면을 정렬 오차로 벌점화하지 않는다.
- 독립 ICP의 작은 잔차가 좋은 전체 형상을 뜻하지 않던 문제:
  여러 촬영의 공통 관절 오차를 맞춰 기하를 일관되게 재계산한다.
- 같은 자세만 반복해도 관절 영점을 식별했다고 판단하던 문제:
  Jacobian의 절대 크기와 singular-value 조건을 함께 확인한다.
- 긴 scan의 원시 점을 한 배열로 붙여 점 수 한도를 넘던 문제:
  각 관측의 한도를 유지하며 voxel map에 순서대로 더한다.

Host의 관련 unit·통합 회귀는 71개다. 필수/확장 전체 gate, real-RMW와 isolated
release gate는 기존 선택적 dev image 준비 미완료 사유로 실행하지 않았다.
Host에 과학/ROS 의존성을 새로 설치하거나 별도 dev 이미지를 만들지 않았다.

## 실제 Genesis 결과

시험한 실행 코드는 `d6e33b6c6dca526934d935424cc8f200f4ada8ff`다. 먼저 push하고
노트북과 원격의 정식 `elesim-update`를 실행했다. 원격 Tailscale host의
GPU **2**(RTX A6000, UUID `GPU-a40782b2-881c-e285-2a8e-66cae9162a01`)만
Genesis와 OpenGL에 보였다. GL renderer도 NVIDIA A6000/software=false였다.
점군 소비자는 CPU 작업이며 GPU 예약과 외부 네트워크가 없었다.

실제 arm URDF와 고정 hand-eye mount를 움직여 480×360 RGB-D **120 frame**을
촬영했다. 고정 base·정적 물체의 box/얇은 box/cylinder 각각에 정상 관절과
오차 관절 조건을 적용한 여섯 episode다. 정상은 depth 잡음 σ=0.6 mm,
오차 조건은 Linear +3 mm, Roll +3도, Bend +0.8/−0.6도에 depth σ=1.5 mm와
10% dropout이다. 힘 제어·관절 동역학·연속 충돌 안전은 검증하지 않았다.

| 조건 | 첫 시야 → 통합 표면 수집률 | 통합 표면 오차 p95 | 최대 치수 오차 | 전체 기준 |
| --- | ---: | ---: | ---: | --- |
| Box | 34.9 → 53.1% | 1.36 mm | 1.37 mm | 미달: 수집률 |
| 얇은 box | 46.8 → 79.0% | 1.55 mm | 1.67 mm | 통과 |
| Cylinder | 38.8 → 68.5% | 1.38 mm | 2.46 mm | 통과 |
| Box + 관절 오차 | 34.6 → 63.9% | 3.80 mm | 1.92 mm | 통과 |
| 얇은 box + 관절 오차 | 45.2 → 60.1% | 4.00 mm | 1.37 mm | 통과 |
| Cylinder + 관절 오차 | 37.3 → 54.4% | 3.95 mm | 7.08 mm | 미달: 수집률 |

수집률은 **전체 정답 표면**에서 점군까지 5 mm 이내인 비율이다. 통합 표면
오차는 반대 방향인 점군→실제 표면 거리다. 두 지표를 혼동하지 않는다.
관절 오차 세 조건의 단순 FK 누적 표면 p95는 15.50/19.39/17.47 mm였고, 공통
오차 보정 후 3.80/4.00/3.95 mm다. 정답 표면에서 5 mm 안에 있는 점의 비율은
세 조건 모두 약 99%다. 시야 방향 폭은 78~110도다.

사전에 정한 전체 기준은 수용 시야 ≥4, 수집률 ≥60%, 개선 ≥12%p,
표면 p95 ≤4 mm(정상)/7 mm(관절 오차), 점군 precision ≥90%, 최대 치수 오차
≤12 mm다. **4/6 통과**이며 기준을 낮춰 전체 성공으로 표시하지 않았다.
나머지 두 경우에도 관측 통합과 치수 범위는 개선됐지만 미관측 표면이 남는다.
현재 자세 후보·화각·관절 범위·view budget에서 부족한 시야를 더 확보하는 것이
후속 과제다. 가동 범위 밖 표면은 몸체/base 이동도 별도로 고려해야 한다.

소비자는 실제 camera pose, actual q, object mesh/catalog와 생성 script를
마운트하지 않았다. 독립 evaluator만 정답을 읽고, 정답을 첫 optical frame과
같은 좌표로 표현해 평가한다. ICP로 평가 결과를 정답에 맞추지 않는다.
명목 world anchor는 관절 오차 조건에서 약 19.5 mm/3.16도 틀리므로 이 결과를
절대 위치 오차 4 mm라는 뜻으로 해석하면 안 된다.

기존 test Sim/Pilot의 container ID·이미지·정지 상태를 보존했고 진단
컨테이너는 제거했다. 전체 원자료 550파일을 내려받아 SHA256을 검증한다.
상세 검증 상태와 digest는 [증거 JSON](../workbench/evidence/curated/experiments/arm_multiview_scan_20261027.json)에 있다.

## 재현 파일과 경계

- Pilot 순수 기하: `elesim_pilot/vision/multiview.py`.
- Private producer: `workbench.research.experiments.arm_scan_scene`.
- 센서만 읽는 consumer: `workbench.research.experiments.arm_scan`.
- 독립 evaluator: `workbench.research.experiments.arm_scan_evaluate`.
- 그림: `workbench.research.analysis.arm_scan_plot`.

Producer와 consumer를 동시에 실행한다. Producer에는 nominal data와 실제
scene, consumer에는 sensor 하위 폴더·명목 arm model·hand-eye만 마운트한다.
서로는 요청 폴더로 q4/sequence를 주고받는다. Producer의 truth 하위 폴더는
consumer에 주지 않는다. 연구 script는 runtime release에 포함되지 않으므로
기록한 SHA의 파일을 별도로 stage한다. 새 DDS 계약은 없다.

```bash
python3 -m workbench.research.experiments.arm_scan_scene \
  /requests /produced --data-root /nominal-data --backend genesis

python3 -m workbench.research.experiments.arm_scan \
  /requests /sensors /scans --arm-model /nominal/arm.json \
  --hand-eye /nominal/hand-eye.json --max-views 20

python3 -m workbench.research.experiments.arm_scan_evaluate \
  /truth /scans --output /evaluation/evaluation.json
```

실제 측정에는 기존 설치 소유 Compose one-off와 고정 Engine/설치 UUID를
검증한 driver를 썼다. Producer에 GPU UUID 하나만 예약하고 consumer와 평가기는
GPU를 예약하지 않는다. 실제 mount·이미지·DeviceRequests도 inspect로 검증한다.
이 명령들은 실물 Robot 동작을 지시하는 경로가 아니다.

각 episode의 `cloud.npz`에서 `fused`가 meter 단위의 통합 XYZ다. `first`는 첫
원시 부분 점군, `fk_only`는 같은 수용 관측의 명목 FK 대조군이고 `icp_control`은
진행 중의 독립 정렬 대조군이다. 각 view의 원본 RGB-D와 encoder는 별도 파일에
남는다. 미관측 면을 추측할 모델은 이 관측 점군을 보존한 채 후속으로 붙인다.
