# 외부 모델을 끼우는 틀

EleSim은 모델의 학습 코드와 실행 경계를 분리한다. 다른 작업자는 자신의 모델을
아래 model API 1에 맞춘 Python adapter로 제공한다. Pilot에 그 package와 필요한
의존성을 설치하고 YAML의 factory/asset을 선택하면 core workflow를 고치지 않고
교체할 수 있다. Checkpoint 파일만으로는 입출력 전처리·좌표 복원을 알 수 없으므로
adapter가 함께 필요하다. `--check`는 로딩과 버전/hash 검증이며 모델 정확도 시험은 아니다.

## 실행 흐름과 소유권

```text
관측 iterable: RGB-D + intrinsics + 실측 q + source identity/time [+ 보정된 IMU]
  → pose → pose_corrections (선언한 순서)
  → segmenter: 물체별 instance mask와 지속 ID
  → 물체별 deprojection / bounded voxel fusion
  → completion: 모델 생성 점
  → mesher: 관측 점 + 생성 점 → 삼각형 mesh
  → planner (선택): 현재 실측 q + 물체들 → 계획안
  → scene bundle → Sim 시작 시 물리 scene / observer / hand-eye
```

기본 Sim scan은 `scan.integrated.yaml`의 **FK → 각 촬영의 바닥/수평면 tilt 보정 →
자동 시점 수집·정합 → 실제 AdaPoinTr AP-PS55 → voxel mesh → UI Apply**를 사용한다.
`scan.example.yaml`은 학습 모델 없는 명시적 비교 기준선이다. 통합 설정은 모델
자산이 없다고 기준선으로 바뀌지 않는다. UI Apply의 실행 중 scene 교체 경로는 아래와 같다.

모델 실행과 결과 생성은 Pilot 소유다. Sim은 검증된 파일만 읽으며 Pilot
구현을 import하지 않는다. Robot은 기존 물리 I/O·limits·deadman을 소유한다.
기록 replay 실행기는 motion lease를 획득하거나 모터 명령을 보내지 않는다.
별도 Sim 자동 scan은 Pilot의 기존 Sim motion lease로 팔을 움직이고 새 DDS
RGB-D를 받아 같은 모델 슬롯에 연결한다. UI는 시작·취소·진행 표시를 담당한다.
촬영 중 물체와 base를 고정해야 한다. 가려진 표면의 완전한 관측은 보장하지 않는다.

## 모델 담당자에게 주는 계약

공개 타입은 `elesim_pilot.reconstruction`에서 import한다. 모든 factory는
`factory(*, options: dict, assets: dict[str, str])`로 호출되고, 반환한 객체는
`api_version = 1`, 아래 `stage`, 버전 문자열 `version`과 메서드를 제공한다.
`assets` 값은 hash를 확인한 파일의 절대 경로다. 모델은 시작할 때 한 번 로드한다.

| stage | 메서드 | 반환값 |
| --- | --- | --- |
| `pose` | `estimate(measurement, task)` | `Pose(frame_id, world_from_camera)` |
| `pose_correction` | `correct(measurement, pose, task)` | 같은 frame의 `Pose` |
| `segmenter` | `segment(measurement, task)` | `InstanceMask`의 tuple/list |
| `completion` | `complete(cloud, task)` | 모델 생성 점 `N×3` NumPy 배열 |
| `mesher` | `mesh(completed_object, task)` | `Mesh(object_id, frame_id, vertices, triangles)` |
| `planner` | `plan(context)` | `Plan(target_id, frame_id, times_s, q, contact_ids)` |

계약의 기준은 다음과 같다.

- 거리·좌표는 metre, 각도는 radian이다. `world_from_camera`는 원본 optical
  좌표(X right, Y down, Z forward)를 scene 좌표로 보내는 4×4 rigid transform이다.
  Sim 소비 frame은 `world`(Z up)다. Floor tilt만으로 절대 world 위치를 알아냈다고
  취급하면 안 된다. `Pose` 제공자가 base/world 및 hand-eye 정합을 책임진다.
- `Measurement.sensor`는 `vision.partial_cloud.SensorObservation`이다. Color는
  BGR `uint8`, depth는 `uint16`/`float32`와 `depth_scale`(metre/raw unit), intrinsics,
  source ID/boot/calibration, source time, sequence, 실측 encoder q 네 축을 갖는다.
  q 순서는 `[linear_m, roll_rad, seg1_rad, seg2_rad]`다. 현재 리니어 기본 범위는
  `[-260/1500, 0]` m, 환산은 **1.5°/mm**다.
- `Measurement.base_world_transform`은 선택적인 실측 base→world rigid transform이다.
  Sim 자동 scan은 고정 여부를 검사한 base pose를 제공하며 기본 `ArmPose`가
  hand-eye FK에 적용한다. RGB-D의 Sim camera truth는 이 추정에 사용하지 않는다.
- 한 실행은 같은 source/boot/calibration과 증가하는 sequence를 사용한다.
  시간 역전, frame 변경, nonfinite/초과 데이터는 거부한다. 입력과 모델 결과의
  배열은 복사 후 read-only로 전달한다. 모델 내부 버퍼를 다음 호출에서 재사용해도
  이전 관측/결과가 덮어써지지 않는다.
- IMU가 필요한 pose 모델/보정기는 `requires_imu = True`를 선언한다.
  `ImuSample(rpy_rad, source_boot_id, calibration_id, timestamp_s)`가 필요하며
  boot가 같고 관측보다 0..0.2초 이내여야 한다. Mount와 축·단위 보정, source
  clock 동기화는 수집기 책임이다. 이 검사만으로 물리적인 시간 정합을 증명하지 않는다.
- `InstanceMask(object_id, label, mask, confidence)`의 mask는 원본 depth와 같은
  크기의 boolean 영상이다. 사과와 나무, 같은 나무의 여러 사과도 각각 다른
  instance ID를 사용하고 여러 시점에서 같은 물체 ID를 유지한다. Semantic class
  이름 하나나 bbox만으로 서로 다른 물체를 합치면 안 된다. 같은 pixel의 여러
  instance 배정, 중복 ID, ID의 label 변경을 거부한다. ID association 정확도는
  segmenter 담당자의 검증 대상이다.
- 기본적으로 선택한 `task.target_id`만 `target`, 나머지는 `obstacle`이다.
  사용자가 허용한 물체만 `contact`로 지정한다. 예를 들어 사과가 target이고
  나무는 obstacle이면 planner가 나무를 접촉 대상으로 출력할 수 없다.
- `ObjectCloud.observed`와 `CompletedObject.predicted`는 별개다. completion은
  생성 점만 반환하며 관측을 덮어쓰지 않는다. 모델이 정규화/중심 이동한 좌표로
  계산했다면 adapter 안에서 원래 frame과 metre로 복원해야 한다. 모델 출력은
  추정치이며 측정한 자유 공간이나 접촉 가능한 영역의 증명이 아니다.
- `Mesh.vertices`는 같은 scene frame의 `N×3`, `triangles`는 0-based integer
  `M×3`다. 잘못된 index, 퇴화 삼각형, 물체 ID/frame 변경을 거부한다.
- `PlanningContext`는 task, scene objects, 마지막 실측 `start_q`, mapping,
  source identity/sequence/time을 제공한다. Planner의 로봇 geometry·solver·policy는
  자신의 assets에서 받는다. 출력 q는 `N×4`, 시간은 초다. 시작 q, 관절 한계,
  target/frame, 허용 contact ID를 검사한다. 충돌·토크·파지 성공은 별도 검증 대상이다.

Scene/view budget은 32 views, 16 objects다. 모델 생성 점과 mesh는 각각 최대
100,000 points/vertices/triangles, voxel 통합은 물체당 50,000 cells다. Mesh OBJ는
물체당 8 MiB, 좌표는 ±100 m 이내다. 계획은 최대 4,000 samples/120초다.
한 번의 `run()`은 동기 실행이며 취소는 모델 호출 전후에 확인한다. 모델의
GPU hang/추론 deadline은 adapter가 별도로 다뤄야 한다.

API 1의 선택적 lifecycle은 `prepare()`, `close()`, `diagnostics()`다.
`supports_cancellation = True`를 선언한 adapter의 실행 메서드에는 `cancelled`
callback을 추가로 전달한다. 기존 adapter의 위치 인자는 바뀌지 않는다.
`ObservationPending`은 같은 자세의 새 coherent capture를 요청하며 정상 관측으로
합치지 않는다. Live scan은 자세당 최대 8 capture, replay는 최대 256 raw capture와
32 accepted views를 허용한다. `diagnostics()`는 JSON dict 최대 16 KiB이며 실제
실행 결과를 manifest의 `model_execution`에 남긴다. `with Pipeline.from_config(...)`
또는 명시적인 `close()`로 외부 자원을 정리한다.

## 모델 package와 YAML

외부 completion adapter의 형태는 다음과 같다. `load_your_model`/`infer_metres`는
제공자가 구현할 모델 고유 코드이며 EleSim이 특정 학습 프레임워크를 강제하지 않는다.

```python
import numpy as np
from your_package.backend import load_your_model

class CompletionAdapter:
    api_version = 1
    stage = "completion"
    version = "adapointr-your-training-run-1"

    def __init__(self, *, options, assets):
        self.model = load_your_model(assets["weights"], **options)

    def complete(self, cloud, task):
        # 입력을 수정하지 않는다. 모델 고유 normalization/inverse는 이 안에서 한다.
        return np.asarray(self.model.infer_metres(cloud.observed), dtype=float)
```

`payload/config/pilot/reconstruction.example.yaml`을 복사하고 해당 stage를 바꾼다.
예를 들어 completion만 교체하면 다음과 같다.

```yaml
completion:
  factory: your_package.adapters:CompletionAdapter
  options:
    device: cuda:0
  assets:
    weights:
      path: /models/your-checkpoint.pt
      sha256: <64-character-sha256>
```

Factory는 설치된 신뢰하는 Python 코드다. EleSim이 network에서 package나
checkpoint를 자동 다운로드하지 않는다. SHA-256은 명시한 파일과의 일치를
확인하며 모델 품질이나 checkpoint의 안전성을 인증하지 않는다. 경로는 YAML
기준으로 해석한다. Checkpoint 외 config/tokenizer 등 입력도 `assets`에 등록한다.
Factory 로딩 실패, dependency 누락, API/stage 불일치, hash 불일치 시 오류를
보고하고 실행을 중단한다. 다른 모델로 조용히 fallback하지 않는다.

`pose_corrections`는 최대 네 개의 correction spec을 받는 순서 있는 목록이다.
IMU 잔차 보정 뒤 floor tilt 보정을 적용하려면 그 순서대로 adapter를 나열한다.
기본 `FloorTilt`는 관측 전용 floor service의 나중 snapshot 대신, 이번 촬영의
depth와 pose를 같은 FloorEstimator에 직접 넣는다. `reference_normal: [0, 0, 1]`은
수평 방향을 신뢰한다는 명시적 설정이다. 안정된 평면을 얻지 못하거나 지지 방향이
충돌하면 보정을 적용하지 않고 재촬영/실패한다. Camera origin은 유지하며 절대
높이·translation·법선 주위 yaw를 알아냈다고 취급하지 않는다.

비교용 `scan.example.yaml`은 기존 HSV detector + arm/hand-eye FK + `ObservedOnly` +
`VoxelSurface`다. 학습된 completion과 planner는 포함하지 않는다. `ObservedOnly`는 명시적으로 빈
생성 점을 반환한다. `VoxelSurface`는 관측/생성 점의 점유 voxel 표면을 삼각형으로
만드는 연결 기준선이다. 실제 매끈한 복원 mesh나 물리 접촉 품질을 보장하지 않으며
completion과 독립적으로 mesher도 교체할 수 있다.

기존 detector adapter는 단일 대상용이다. YOLO의 실제 instance mask는 사용할 수
있지만 bbox fallback과 여러 같은-label 후보는 거부한다. 일반적인 사과/나무 장면은
추적을 포함한 다중 instance segmenter adapter를 제공해야 한다.
통합 예제의 `depth_component_gap_m: 0.025`는 인접 pixel의 깊이 차이가 25 mm
이하인 성분 중 detector의 `min_area_px` 미만인 작은 성분을 제외한다. 색 마스크의
고립된 배경 깊이가 메쉬를 확장하는 것을 막으며, 충분한 pixel을 가진 별도 내벽은
유지한다. Object mesh/정답을 쓰지 않고, 실제 관측을 새 점으로 대체하지 않는다.
외부 adapter에는 자동 적용하지 않으며 이 옵션의 기본값 0은 비활성이다.
통합 `VoxelSurface`의 `min_generated_neighbors: 8`은 voxel 크기의 절반인 반경
안에서 이웃점 8개가 지지하는 생성점만 mesh에 사용한다. 관측점은 모두 유지하고
원본 생성 배열도 그대로 저장한다. 촬영으로 직접 확인되지 않은 표면의 정확성을
인증하는 검사가 아니라 드문드문 뻗은 생성 노이즈의 voxel화를 막는 선택적 기준이다.
Mesher diagnostics에 사용/제외한 생성점 수를 기록하며 기본값 0은 비활성이다.

## 실행과 Sim 전달

### Sim 자동 다중 시야 수집

UI의 `Object Scan (Sim)`에서 시작한다. 실행 중인 Sim을 선택하고 torque ON,
claw open 상태에서 대상이 hand-eye 화면에 보이도록 먼저 조준한다. GO2/base와
물체는 고정하고 Pick/Gaze/trajectory 등 다른 팔 작업은 정지한다. Instance ID는
segmenter가 반환하는 지속 ID와 같아야 한다. 기본 단일 대상 예제는 `target`이다.
최대 촬영 수는 2..24, 세션 상한은 600초다.

Pilot은 기본적으로 자신의 config 옆 `scan.integrated.yaml`을 읽는다. 사용자 모델은
Pilot 실행 인자 `--scan-models /path/to/scan.yaml`로 선택한다. 이 YAML은 replay와
같은 model API 1이다. 기본 통합 설정은 Sim의 빨간 대상용 HSV mask/FK,
FloorTilt와 실제 AdaPoinTrCompletion을 선택한다. 비교 기준선은
`--scan-models .../scan.example.yaml`로 선택한다.

통합 설정은 다음 외부 asset을 Pilot에 read-only로 제공해야 한다. 자동 다운로드나
checkpoint 자동 교체는 없다. 모델 source는 고정 PoinTr revision
`4603257ed3db9e7dad349b712e1b2fe0da207015`의 archive다.

- `/opt/elesim/models/adapointr/pointr-source.tar.gz`: SHA-256
  `ab52d68e84bf79e7d7969a826c93672c2ff4d997b714a571b722bcd5f4be668e`
- `/opt/elesim/models/adapointr/ap_ps55.pth`: SHA-256
  `66e22112aa43e74cd0716012201ea40bda0492e8163897e6e45102e86cb781b7`
- Projected55 model config는 `config/reconstruction/adapointr.projected55.yaml`에
  포함한다. AP-PS55는 기존 실험에 사용한 제3자 inference 재배포본이며 원저자의
  최고 성능 checkpoint와 동일하다는 주장은 하지 않는다.

정상 amd64 Pilot 이미지는 Torch, einops, easydict를 포함한다. 실행은 한 CUDA
device가 필요하며 모델을 먼저 strict-load해 준비를 확인한 뒤 scan 이동을 시작한다.
독립 `spawn` process는 원본 모델 forward와 Triton FPS/Torch gather를 사용한다.
누적 관측점의 중심만 빼고 world Z-up/metre 크기를 유지하며 2,048점을 샘플링하고,
8,192개 생성점을 원래 좌표에 복원한다. 관측은 변경하지 않는다. Source/config/
checkpoint hash, 실제 load·추론, 전처리 및 보정 적용 여부는 결과에 기록한다.
이 경로의 실행 완료는 미관측 형상이나 물리 접촉 품질의 보증이 아니다.

한 세션은 다음 순서로 진행한다.

1. 현재 Sim endpoint/boot/lease와 simulation epoch, 고정 base를 묶는다.
2. 현재까지 관측한 점군과 4축 FK로 다음 시점을 선택한다. 실제 운용 관절 범위,
   digest가 일치하는 arm sphere 모델, 관측된 주변 표면을 검사한다. 연속 구간의
   separation bound에 추종 오차 3 mm / 0.04 rad를 포함하며 검사 예산 초과도 거부한다.
3. DDS 제어 tick에서 20 mm/s, 0.15 rad/s 이하로 이동한다. 서로 다른 새 관절
   피드백 세 개가 1 mm / 0.015 rad 이내에 들어와야 정착으로 인정한다.
4. 정착 시각 이후이며 같은 boot/calibration·증가한 sequence·해당 q를 가진
   coherent RGB-D만 촬영한다. 마스크와 물체별 점군을 만들고 다음 시점으로 반복한다.
5. 겹치는 시야를 정렬하고 조건이 맞으면 공통 bend 영점 오차를 추정해 원본 점군을
   다시 통합한다. 사용자 pose correction이 nominal FK를 바꾸면 이 nominal joint fit은
   적용하지 않는다. 같은 최종 camera transform으로 모든 instance를 배치한다.
6. completion/mesher/선택적 planner를 호출하고 scene bundle과 보고서를 저장한다.

모델 준비와 추론의 취소/deadline은 AdaPoinTr 자식 process만 종료한다. 기본
deadline은 호출당 120초다. 모델 source의 private 추출본만 정상 종료 시 정리하고
원본 asset과 기존 Triton/Genesis cache는 보존한다.

모델·시점 탐색·파일 작업은 worker에서, leased motion 전송은 DDS 제어 tick에서
실행한다. 취소·피드백 지연·추종 실패·lease/boot/epoch/base 변경은 후속 명령을
막고 원래 lease를 해제한다. AdaPoinTr는 취소 시 자식을 종료한다. 취소 callback을
지원하지 않는 외부 adapter는 호출이 반환할 때까지 새 scan을 시작할 수 없다.
Robot 대상은 거부한다.

Pilot capture root의 `object_scans/<job-id>/`에 다음을 남긴다. Capture root는
기존 perception capture 경로 설정을 따른다.

```text
scan.json             # phase, 각 view의 q/seq/채택 여부/경로 검사, 종료 이유
calibration/*.npz     # 보정 안정화에 사용한 원본 SensorObservation
frames/000.npz        # 원본 SensorObservation; truth/mesh 필드 없음
silhouettes/000.png   # 선택한 대상의 시점별 이진 마스크
silhouettes/000.npz   # mask, 추정 camera/base transform, 관측 camera 점군
scene/scene.json      # 성공한 재구성 bundle; OBJ/NPZ와 함께 사용
```

`scan.json`은 촬영별 base/world transform, floor correction 및 completion 실행
기록도 남긴다. 원본 SensorObservation schema 1의 엄격한 field allowlist는 유지한다.
Replay의 `--frames`에 scan 디렉터리 전체를 주면 calibration/accepted capture를
source 시간 순서로 읽고 해당 sequence의 base pose metadata를 함께 적용한다.

`scan.json.phase=completed`가 세션 성공 기준이다. 실패·취소 시 원본은 남겨 진단할
수 있으며 export 중 취소하면 scene 파일도 남을 수 있다. UI는 자동으로 실행하지 않는다.
종료 이유는 `view_budget`, `no_safe_distinct_view`, `information_saturated`로 구분한다.
UI와 보고서는 96개 방향 probe의 시야 다양성과 남은 방향을 제공한다. 이 비율은
표면 완성도가 아니며 `geometry_complete_claimed=false`, `needs_more_observation=true`를
유지한다. 안 보인 공간·GO2·케이블·동역학은 충돌 검사의 보증 범위에 없다.
물체 뒤집기/집기나 base 이동, 미관측 면의 완성은 별도 작업이다.

### 기록 replay와 scene 소비

설치된 Pilot 환경에서는 console command를 사용한다. 저장소의 개발 attachment가
있다면 같은 명령 앞에 `elesim-dev`를 붙인다.

```bash
elesim-reconstruct --config /path/to/pipeline.yaml --check
elesim-reconstruct --config /path/to/pipeline.yaml \
  --app-config /path/to/pilot/config.yaml \
  --frames /path/to/recorded-frames --output /path/to/new-scene \
  --scene-id orchard --target apple-1 --task pick
```

`--app-config`는 실제 Pilot 운용 mapping을 계획 검증에 사용한다. 생략하면
현재 소스 기본값을 사용한다. Scene manifest provenance에는 사용한 mapping,
factory/version/asset hash, options hash, source identity와 마지막 관측 시각을
기록한다. NPZ 입력은 기존 SensorObservation schema 1 파일이며 파일명 순서로
읽는다. 기존 packet에는 IMU가 없으므로 IMU correction이 필요한 경우 아래
Python API에서 calibrated sample을 함께 넘겨야 한다.

```python
from elesim_pilot.reconstruction import Pipeline, Task, export_result

pipeline = Pipeline.from_config("/path/to/pipeline.yaml", mapping=actual_mapping)
result = pipeline.run(measurements, Task("orchard", "apple-1", operation="wrap"),
                      cancelled=cancel_requested, progress=report_stage)
manifest = export_result(result, "/path/to/new-scene")
```

`measurements`는 검증된 `Measurement`를 제공하는 iterable이다. 모델 오류나
취소 시 부분 scene을 정상 결과로 반환하지 않는다. Export는 새 디렉터리만
생성하고 기존 bundle을 덮어쓰지 않으며 `scene.json`을 마지막에 게시한다.

```text
new-scene/
  scene.json        # schema 1, m/world, object ID/label/role, mesh SHA-256, provenance
  apple-1.obj      # world 좌표의 triangular mesh
  apple-1.npz      # observed / predicted 별도 배열; allow_pickle=False
  tree-1.obj
  tree-1.npz
  plan.json        # planner 선택 시만; scene revision에 묶인 미실행 제안
```

Bundle 전체를 Sim host에 복사하고 container에서도 읽히게 명시적으로 배치한다.
자동 multi-host 전송/mount나 DDS 파일 업로드는 없다. 기존 Sim 실행 인자에
다음을 더하거나 YAML의 `simulation.runtime.scene_manifest`를 지정한다.

```bash
elesim-sim --scene-manifest /path/inside/sim/new-scene/scene.json
```

Sim은 schema/frame/units/path/OBJ/hash를 검증하고 build 전에 물체들을 고정
geometry로 추가한다. 물리 scene은 collision을 켜고 observer/hand-eye의 렌더
replica는 물리 scene과 같은 manifest revision의 OBJ를 collision 없이 읽는다.
Build 전후에도 hash를 검사한다. Mesh의 Z-up/metre 좌표를 유지하고 자동 align,
decimation, convexification, watertight wrapping을 끈다. 설정 근거는 고정 버전의
[Genesis 1.4.5 Mesh options](https://github.com/Genesis-Embodied-AI/genesis-world/blob/v1.4.5/genesis/options/morphs.py)다.
닫힌 표면·법선·self-intersection 등 실제 충돌에 필요한 mesh 품질은 mesher가
검증해야 하며 파일 validator가 물리 적합성을 보증하지 않는다.

Dynamic/async Sim에서는 재시작 없이 bundle을 교체할 수 있다. Bundle 전체를
Sim의 `simulation.runtime.scene_import_root` 아래 새 디렉터리에 게시하고 UI의
`Sim Scene`에 `new-scene/scene.json` 같은 상대 경로를 입력해 `Load scene`을 누른다.
기본 root는 Sim process의 `$XDG_CACHE_HOME/scene-imports`다. `Clear scene`은
가져온 물체만 제거하며 기본 로봇과 지면은 유지한다. 지원 capability가 있는
Sim의 유효한 UI session에서만 이 명령을 보낸다. 자동 multi-host 전송은 없다.
Pilot의 `--scan-output-root`와 Sim의 `scene_import_root`가 같은 공유 bundle
디렉터리를 가리키면 UI `Object Scan (Sim)`의 `Apply reconstructed scene`으로
완료 결과를 바로 적용한다. Pilot은 공유 root 기준 상대 manifest와 digest,
관측을 제공한 Sim ID/boot를 상태에 게시한다. UI는 같은 live Sim인지 확인하고
파일을 읽지 않은 채 적용 명령을 보낸다. Pilot의 기본 출력 root도
`$XDG_CACHE_HOME/scene-imports`이며 설치된 Python package 안에 쓰지 않는다.
공유 경로 설정은 배포 시 명시적으로 한다. 다른 호스트의 파일을 옮기지는 않는다.
Sim은 물리 상태를 저장하고 재빌드·controller 재결합·두 카메라 새 프레임 확인을
마친 뒤 응답한다. 실패하면 이전 씬으로 rollback하고 paused 상태로 남는다.
비볼록 충돌 커널은 기본적으로 초기 build에서 준비하지만 새 OBJ의 처리와
카메라 복제 씬 재생성 시간은 남는다. 파일은 사용 중 덮어쓰지 않는다.
물체의 동적 attach/들기, 계획 실행은 제공하지 않는다. 기존 mock-hug와 RL 실행기는 별도 경로다.
파일 검증은 protocol package의 `scene_artifact.py`가 담당하며 DDS wire major 6은
변경하지 않는다. 실행 중 교체 명령은 `simulation.scene-rebuild.v1` capability를
사용하는 major 6의 선택적 확장이다.

## 수집 좌표와 mask 경계

고정 암의 world 배치는 실행 설정 `robot.arm.spawn.xyz/euler_deg`를 사용한다.
생성된 IK geometry에 들어 있는 기본 spawn이 이 값을 덮어쓰지 않는다.
GO2 장착 시에는 기존 측정된 base pose 경로를 유지한다.

기준 `DetectorSegmenter`의 `mask_erode_px`는 0..8 정수이며 기본값은 0이다.
Sim `scan.example.yaml`은 2를 명시해 JPEG/안티앨리어싱 경계의 RGB mask가
뒤쪽 바닥 깊이를 섞는 픽셀을 제외한다. 경계 부근의 관측을 줄이는 옵션이며
미관측 면을 채우거나 완전한 실루엣을 보증하지 않는다. Model API는 1이다.

## Robot IMU 모델과 구분

Robot의 5 ms arm loop에는 임의 Python/Torch 모델을 넣지 않는다. 기존
`payload/config/pilot/imu_models/*.json`의 schema 1 expression graph가 이 슬롯이다.
모델 담당자는 q 4축/IMU 3축 입력과 corrected q 4축 출력을 최대 64개 허용 연산으로
export하고 Pilot catalog에 등록한다. 자세한 연산과 형식은
[`imu_models/README.md`](../payload/config/pilot/imu_models/README.md)를 따른다.
로컬 C++가 graph·stale IMU·한계를 검증한다. 현재 shipped 모델은 identity이며
raw Teensy IMU의 mount/단위 보정 및 native 입력 연결은 여전히 남아 있다.
이번 pose-correction adapter는 재구성 카메라 좌표 보정용이며 이 Robot 계약을
대체하거나 raw IMU 연결이 완료됐다고 선언하지 않는다.

## 검증 범위

Host tests는 외부 factory import, pin/hash/API 오류, source fencing, 다중 물체
분리, correction 순서/IMU freshness, 관측/추정 보존, 계획 limit/contact 검증,
immutable export와 Sim 소비를 다룬다. System test는 Pilot CLI와 Pilot이 보이지
않는 별도 Sim 소비 프로세스를 실행한다. Genesis 호출은 fake로 검증한다.
자동 scan의 별도 analytic sensor 시험은 실제 4축 FK·충돌 모델·시점 계획기를 사용해
다중 시야 수집, 접근이 막혔을 때의 중단과 관측 표면 오차를 검사한다. Live adapter는
fake DDS/카메라로 정착·freshness·취소·epoch/lease 변경을 검사하고 UI→Pilot의
operator JSON 호출을 별도 프로세스로 검증한다. 실제 RMW 전달, GPU 렌더·물리 접촉,
learned model 정확도, Jetson/Robot 동작 및 canonical isolated-release 검증은 이 host
결과에 포함되지 않는다. 현재 증거와 미실행 사유는 [`status.md`](status.md)를 따른다.
