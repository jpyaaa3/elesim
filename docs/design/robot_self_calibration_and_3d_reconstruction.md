# 로봇 자체 보정 및 3D 재구축 전략

상태: 연구/설계 초안  
갱신일: 2026-08-24
범위: 로봇 팔의 기구학 오차, 하중 의존 처짐, eye-in-hand RGB-D/IMU
불확실성, point-cloud 재구축, 그리고 이를 조사하기 위해 필요한
Pilot/Robot/Sim/UI/protocol 경계.

이 문서는 일반적인 아키텍처 메모보다 의도적으로 더 규범적이다. 코딩
에이전트가 작은 구현 작업으로 분할할 수 있어야 한다. 소프트웨어 전용
테스트가 물리적 정확도를 증명한다고 주장하지 않는다.

## 1. 결정 요약

EleSim은 FK, ZED tracking, D435 depth, 어느 IMU도 ground truth로 지정해서는
안 된다. 문제는 독립적인 제약을 단계적으로 추가하고, 파라미터가 관측되지
않는 추정치를 거부하는 방식으로 해결한다.

권장 순서는 다음과 같다.

1. 어떤 estimator도 바꾸기 전에 동기화되고 replay 가능한 증거를 기록한다.
2. camera intrinsic, camera/IMU extrinsic, time offset을 runtime 밖에서 보정한다.
3. 알려진 간격의 두 socket에 공을 넣는 방식처럼 반복 가능한 물리적 closure로
   무하중 정적 기구학을 보정한다.
4. 정적 로봇 파라미터를 고정한 상태에서 hand-eye extrinsic을 보정한다.
5. 반복 측정으로 저차원의 gravity/payload/direction 의존 compliance 모델을
   식별한다.
6. 임의 장면의 다중-view depth registration은 residual refinement와 검증에만
   사용한다.
7. 불확실성과 provenance를 포함해 point cloud와 mesh를 재구축한다. mesh를
   무조건적인 진실로 publish하지 않는다.

첫 구현은 offline 실험 pipeline이어야 한다. offline 방법이 held-out data에서
현재 baseline보다 좋아진 뒤에야 runtime scan 제어, 새 DDS 계약, mesh 전송,
UI workflow를 후속 단계로 진행한다.

## 2. 문제 정의

compliant arm에 카메라가 장착된 경우 world-frame depth point는 일반적으로
다음과 같이 계산된다.

```text
p_world = T_world_base
        * FK(q_encoder, theta_kinematic)
        * T_tip_camera
        * p_camera
```

각 항이 잘못될 수 있다.

- `q_encoder`는 zero offset, backlash, sampling latency, drivetrain compliance로
  인해 물리적 joint angle과 다르다.
- `theta_kinematic`에는 부정확한 joint axis, link length, frame transform이
  포함된다.
- `T_tip_camera`에는 장착 오차와 hand-eye 오차가 포함된다.
- `p_camera`에는 stereo/depth noise, invalid edge, multipath 또는 texture
  failure, camera calibration error가 포함된다.
- 중력, payload, 온도, 접근 방향에 따라 arm configuration이 변한다.
- camera, arm encoder, ZED IMU, BNO085의 timestamp는 본질적으로 동시에
  기록되지 않는다.

물체 형상까지 알려져 있지 않으면 optimizer는 동일한 residual을 로봇, 센서,
물체 중 하나를 변경하여 설명할 수 있다. 이것은 단순히 optimizer가 나쁜
문제가 아니라 gauge 및 observability 문제다.

### 2.1 분리해야 하는 오차 종류

| 종류 | 전형적인 상태 | 예상 수명 | URDF를 수정할 수 있는가? |
| --- | --- | --- | --- |
| Camera intrinsic/depth bias | focal length, distortion, depth scale | camera/profile | 아니오 |
| Sensor mounting | tip-to-camera, camera-to-IMU | remount event | source URDF가 아닌 model/calibration artifact |
| Static robot geometry | joint zero, axis, 일부 link transform | assembly/service event | 생성된 calibrated model만 |
| Compliance | gravity, payload mass/CoM, configuration | pose/load마다 | 아니오; runtime correction model |
| Hysteresis | approach direction, gear state | motion마다 | 아니오; stateful correction 또는 uncertainty |
| Temperature/drift | motor/link temperature, elapsed operation | 시간에 따라 변함 | 아니오 |
| Scene model | points, surfels, TSDF/mesh | scan session | 절대로 아님 |

이들을 익명 `sag_model.json` 하나로 합치면 낮은 residual의 의미를 해석할 수
없다. 각 artifact는 무엇을 추정하고 무엇을 고정하는지 선언해야 한다.

## 3. 현재 repository가 이미 제공하는 것

구현은 다음 경계를 확장해야 하며 우회해서는 안 된다.

- Robot은 물리 arm/camera acquisition과 local safety를 소유한다.
- Pilot은 perception, FK/IK, workflow, calibration optimization, mesh
  generation을 소유한다.
- Sim은 Genesis visualization과 mock scan generation을 소유한다.
- UI는 operator intent와 presentation을 소유한다.
- `packages/elesim_interfaces`는 ROSIDL wire type을 소유한다.
- `packages/protocol`은 bounded transport, validation, contract registry를
  소유한다.
- `model/builder`는 생성된 immutable robot/model artifact를 소유한다.

현재 활용할 수 있는 표면은 다음과 같다.

- `RgbdFrame`은 coherent latest-only RGB/depth/intrinsics sample이며 이미
  선택적인 `arm_q`와 camera pose 필드를 가진다.
- Sim은 이미 pose가 포함된 synthetic RGB-D sample을 publish한다.
- `origin/3d_fusion`에는 pure NumPy transform, crop, plane removal, voxel
  fusion, synthetic scan test, FK pose composition, cylinder-fit reporting이
  들어 있다.
- Pilot과 Sim에는 기능적으로 중복된 sag evaluator와 예시 coefficient 파일이
  이미 있다.
- ZED Mini와 D435는 서로 다른 driver, calibration, model bundle을 가진 명시적
  profile이다.

이 작업과 관련된 현재 gap은 다음과 같다.

- Robot의 `CameraPublisherThread`는 `arm_q` 또는 synchronized camera pose를
  채우지 않는다.
- 현재 RGB-D timestamp는 capture 뒤 wall time을 사용하며, 문서화된
  exposure/device timestamp가 아니다.
- 현재 ZED driver는 left RGB와 depth만 publish한다. positional tracking,
  spatial mapping, IMU sample, pose covariance를 활성화하지 않는다.
- Latest-only RGB-D는 live perception에는 적합하지만 scan archive나 bulk
  artifact 전송 수단으로는 신뢰할 수 없다.
- 기존 sag JSON에는 schema version, units, robot/bundle hash, training
  dataset identity, payload scope, temperature scope, validation report,
  covariance가 없다.
- Pilot과 Sim은 sag evaluator를 중복 보유한다. 이 설계는 sibling import를
  허가하지 않는다. 후속 작업에서 생성된 shared artifact evaluator를 선택할지,
  테스트된 복사본을 유지할지 결정해야 한다.
- `origin/3d_fusion`은 pre-refactor architecture에 속하므로 merge해서는 안
  된다. 재사용은 선택한 pure algorithm과 test를 현재 소유자에게 port하는
  것을 뜻한다.

## 4. Measurement 전략: chicken-and-egg loop 끊기

하나의 실험으로 모든 파라미터를 식별할 수 없다. 한 번에 작은 parameter
group만 free인 여러 실험을 사용한다.

### 4.1 Stage A: sensor calibration

목표: sensor timing/mounting error가 arm sag로 학습되지 않게 한다.

고정:

- robot kinematics;
- calibration target을 사용하는 경우의 object geometry.

추정:

- camera intrinsics 및 depth scale/bias;
- camera-to-ZED-IMU transform과 time offset;
- camera-to-BNO085 transform과 time offset;
- IMU noise와 bias parameter.

필수 excitation:

- 관측 가능한 모든 축을 중심으로 회전;
- translation 및 angular velocity 변화;
- single-axis wrist-roll-only sequence 금지.

수용하려면 유한한 covariance, 제한된 timestamp jitter, 다른 motion sequence를
사용한 validation, camera serial number와 firmware/SDK version을 명시한
report가 필요하다.

### 4.2 Stage B: static kinematic closure

목표: camera에 의존하지 않고 zero-load joint offset과 선택된 geometric
transform을 결정한다.

권장 fixture:

- end effector 끝의 spherical tool;
- 둘 이상의 반복 가능한 socket;
- 정확하게 측정한 socket separation;
- 가능하면 workspace의 여러 위치에 둔 socket.

socket `s`에 속한 sample에서 `x_i(theta)`를 FK가 예측한 ball center라고
하자. 최소 objective는 다음과 같다.

```text
min_theta  sum_s sum_i in s rho(||x_i(theta) - mean_s(x(theta))||^2)
         + lambda * rho((||mean_1 - mean_2|| - known_distance)^2)
```

처음에는 다음만 사용한다.

- 네 개의 joint zero offset;
- 독립적으로 근거가 있는 경우의 base transform;
- tool-center transform.

처음부터 모든 link transform을 free로 두지 않는다. Jacobian rank와 held-out
residual이 해당 parameter group이 관측 가능하고 유용하다는 것을 보여줄 때만
한 group씩 추가한다.

### 4.3 Stage C: hand-eye calibration

목표: arm geometry나 compliance가 오차를 흡수하지 못하게 하면서
`T_tip_camera`를 추정한다.

권장 순서:

1. asymmetric fiducial/AprilGrid 또는 알려진 sphere;
2. plane 및 depth residual validation;
3. arbitrary-object multi-view point-cloud refinement.

첫 hand-eye solve 동안 static kinematics는 고정한다. arbitrary-scene
refinement 동안에는 제한된 six-DOF hand-eye delta만 free로 둔다.

### 4.4 Stage D: elastostatic/compliance calibration

목표: static geometry를 고정한 뒤 gravity와 payload에 따라 달라지는 pose
error를 모델링한다.

처음에는 의도적으로 작은 model이 적합하다.

```text
delta_tip(q, load) = J(q) * C * wrench(q, payload_mass, payload_com)
```

현재 segmented-arm 표현이 이를 직접 지원할 수 없다면 저차원 basis를 사용한다.

```text
delta(q, load, direction) = B(q) * beta_load
                          + beta_direction[approach_sign]
```

첫 dataset은 각 validation pose를 다음 조건으로 반복해야 한다.

- 추가 payload 없음;
- 알고 있는 mass와 CoM을 가진 payload를 최소 두 종류;
- positive 및 negative approach direction;
- gravity leverage가 다른 여러 arm configuration;
- 기록된 temperature 또는 온도가 제어되지 않았다는 명시적 진술.

하나의 payload에 고차원 unconstrained polynomial을 fitting하지 않는다. payload와
approach-direction 반복이 없다면 model을 exploratory로 보고하고 IK에 활성화하지
않는다.

### 4.5 Stage E: arbitrary-scene consistency

목표: 정적 scene의 여러 depth view 사이의 residual inconsistency를 측정하고
refine한다.

view `i`의 point `p_i`와 view `j`의 대응 point/normal `(p_j, n_j)`에 대해
robust point-to-plane residual을 사용한다.

```text
r_ij = n_j^T (T_i(q_i, theta) p_i - p_j)
```

이 단계에는 다음이 필요하다.

- 충분한 overlap;
- 세 차원을 span하는 surface normal;
- correspondence를 만들 수 있을 만큼 충분히 가까운 initial estimate;
- depth-edge와 confidence rejection;
- robust loss와 correspondence trimming;
- 인접 frame을 많이 모으는 것이 아니라 joint-space diversity를 확보.

plane, sphere, cylinder는 각각 null direction을 가진다. scan planner는
퇴화한 shape에서 낮은 residual을 조용히 수용하지 말고 observability를
보고해야 한다.

## 5. Reconstruction architecture

### 5.1 Data ownership

```text
Robot
  synchronized sensor acquisition
  local device timestamps and arm snapshot
  local safety and scan motion execution only through existing lease
       |
       | typed latest-only RGB-D + bounded metadata
       v
Pilot
  keyframe selection, replay archive, calibration factors
  point-cloud registration, fusion, mesh generation, validation
       |
       | bounded status/preview first; artifact transfer only after design gate
       v
Sim
  point-cloud/mesh visualization
  deterministic synthetic fixtures and imported OBJ/PLY mock input
       |
       v
UI
  scan/calibration workflow, progress, diagnostics, result comparison
```

Robot은 reconstruction이나 Pilot code를 import해서는 안 된다. Pilot은
Robot/Sim implementation을 import해서는 안 된다. Sim은 calibration authority가
되어서는 안 된다.

### 5.2 두 pose hypothesis를 유지하고 하나로 덮어쓰지 않기

각 captured keyframe은 독립적인 pose evidence를 보존해야 한다.

```text
pose_fk            FK from encoder state and selected calibration artifact
pose_vio           optional ZED/VIO relative pose
pose_refined       optional optimizer output
pose_covariance_*  uncertainty for each estimate
```

FK를 ICP/VIO 결과로 덮어쓰지 않는다. 두 hypothesis의 차이는 진단 evidence이며
ablation test에 필요하다.

### 5.3 Raw evidence와 derived product

Raw evidence는 immutable이다.

- color/depth frame 또는 선택된 keyframe;
- 원본 intrinsic과 depth scale;
- device 및 receive timestamp;
- correction 전 raw joint state;
- IMU sample;
- camera profile/serial/SDK 정보;
- Robot/Pilot boot ID와 scan session ID.

Derived product는 재현 가능하고 폐기할 수 있다.

- filtered point cloud;
- normal과 correspondence;
- FK/VIO/refined pose;
- fused cloud, TSDF, mesh;
- calibration parameter;
- report와 preview.

알고리즘을 다시 실행하면 configuration과 code revision을 가진 새로운 derived
run을 생성해야 하며, raw capture를 변경해서는 안 된다.

## 6. Dataset schema v1

첫 구현은 새 network protocol이 아니라 directory artifact를 사용해야 한다.
권장 layout은 다음과 같다.

```text
scan-session/
  manifest.json
  frames/
    000000.meta.json
    000000.color.jpg
    000000.depth.npy
  imu/
    zed.jsonl
    bno085.jsonl
  arm/
    samples.jsonl
  derived/
    <run-id>/
      config.json
      keyframes.json
      poses.npz
      fused.ply
      mesh.obj
      report.json
```

`manifest.json` 최소 필드:

```json
{
  "schema": "elesim.scan-session.v1",
  "session_id": "uuid",
  "created_at_utc": "RFC3339",
  "system_id": "bounded-string",
  "source_role": "robot-or-sim",
  "source_endpoint_id": "robot-go2",
  "source_boot_id": "boot-id",
  "camera_profile": "zed_mini",
  "camera_serial": "vendor-serial",
  "camera_sdk": "version",
  "model_bundle": "default",
  "model_bundle_sha256": "hex",
  "hand_eye_sha256": "hex",
  "sag_artifact_sha256": null,
  "units": {"length": "m", "angle": "rad", "time": "s"},
  "clock_domains": ["camera-device", "host-monotonic", "host-wall"],
  "frame_count": 0,
  "complete": false
}
```

Frame metadata 최소 필드:

```json
{
  "sequence": 0,
  "capture_status": "accepted",
  "device_timestamp_ns": 0,
  "host_monotonic_before_ns": 0,
  "host_monotonic_after_ns": 0,
  "host_wall_timestamp_ns": 0,
  "arm_q_raw": [0.0, 0.0, 0.0, 0.0],
  "arm_sample_before_sequence": 0,
  "arm_sample_after_sequence": 1,
  "pose_straddle_rad": 0.0,
  "intrinsics": {"fx": 0.0, "fy": 0.0, "cx": 0.0, "cy": 0.0,
                 "width": 640, "height": 480},
  "depth_scale": 1.0,
  "depth_invalid_fraction": 0.0,
  "color_sha256": "hex",
  "depth_sha256": "hex"
}
```

규칙:

- 모든 숫자 값은 finite여야 한다.
- SI 단위는 필수다.
- 경로는 상대 경로이며 session root 아래에 포함되어야 한다.
- replay 전에 hash를 검증한다.
- atomic finalization 전에 capture가 중단되면 `complete=false`로 유지하며
  calibration dataset으로 수용하지 않는다.
- raw file은 Git에 commit하지 않는다.
- synthetic fixture는 의도적으로 작고 완전히 결정론적인 경우에만 commit할 수
  있다.

## 7. Calibration artifact schema v1

현재의 version 없는 coefficient dictionary를 확장하지 않는다. 처음에는 legacy
evaluator를 감쌀 수 있는 versioned artifact를 도입한다.

```json
{
  "schema": "elesim.arm-calibration.v1",
  "artifact_id": "uuid",
  "created_at_utc": "RFC3339",
  "robot_identity": {
    "model_bundle_sha256": "hex",
    "arm_serial": "string-or-explicit-unknown",
    "camera_profile": "zed_mini",
    "camera_serial": "string-or-explicit-unknown"
  },
  "parameter_groups": {
    "joint_zero_offsets_rad": [0.0, 0.0, 0.0, 0.0],
    "base_delta_se3": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "tool_camera_delta_se3": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "compliance": {"family": "none", "coefficients": []},
    "backlash": {"family": "none", "coefficients": []}
  },
  "fixed_groups": ["camera_intrinsics"],
  "dataset_sha256": "hex",
  "solver": {"name": "string", "version": "string", "config_sha256": "hex"},
  "observability": {
    "jacobian_rank": 0,
    "parameter_count": 0,
    "singular_values": [],
    "condition_number": 0.0
  },
  "validation": {
    "accepted": false,
    "train_rms_m": 0.0,
    "validation_rms_m": 0.0,
    "max_error_m": 0.0,
    "workspace_cells": [],
    "payloads_kg": [],
    "approach_directions": [],
    "temperature_range_c": null
  },
  "limitations": []
}
```

활성화 규칙:

- `validation.accepted`는 true여야 한다.
- robot/model/camera identity와 hash가 일치해야 한다.
- serial을 알 수 없는 경우 명시적인 operator confirmation이 필요하며,
  조용히 일치하는 것으로 처리하지 않는다.
- 지원하지 않는 schema나 parameter family는 fail closed한다.
- training sample과 validation sample은 서로 분리되어야 한다.
- 선언된 payload/workspace/temperature scope 밖의 artifact는 diagnostic-only이며
  IK를 변경해서는 안 된다.

## 8. Observability와 acceptance

모든 solver는 scalar RMS보다 많은 결과를 생성해야 한다.

필수 수치 진단:

- 파라미터를 비교 가능한 단위로 scaling한 뒤의 Jacobian rank;
- singular value와 condition number;
- 근사 parameter covariance 또는 계산할 수 없는 이유;
- residual histogram과 robust-loss inlier fraction;
- pose, workspace, payload, direction별 error;
- train과 held-out validation error;
- initial value에 대한 민감도;
- 해당하는 경우 FK-only, VIO-only, refined pose의 ablation 결과.

즉시 reject 조건:

- finite가 아닌 parameter/residual;
- gauge fixing 이후 선언된 parameter count보다 낮은 rank;
- 설정된 threshold를 넘는 condition number;
- calibration 전 baseline보다 나쁜 validation error;
- 구체적으로 승인된 설명 없이 parameter가 bound에 도달;
- 작은 train residual과 함께 validation residual이 실질적으로 악화;
- normal 방향 또는 joint-space coverage 부족;
- model/calibration/camera identity 불일치;
- 설정된 straddle bound를 넘어서는 미확인 time alignment.

이 설계는 보편적인 millimetre threshold를 지정하지 않는다. 초기 threshold는
반복 측정 noise와 task accuracy에서 도출한 뒤 experiment configuration에
기록해야 한다.

## 9. `origin/3d_fusion`에서 port할 것

기존 동작에 대한 test를 먼저 작성한 뒤 port한다.

- pure `transform_points` 동작과 float32 보존;
- conservative camera-depth gate와 world crop;
- coordinate-key aliasing이 없는 deterministic voxel downsampling;
- point별 camera provenance;
- 주입 가능한 operation으로서의 plane filtering;
- elapsed time이 아닌 측정된 angle에 따른 scan planning;
- pose-straddle rejection;
- camera travel, observation span, fused-vs-single-view, wall-thickness metric;
- synthetic cylinder/table fixture와 pose-error sensitivity test;
- 현재 motion lease와 Robot safety ownership 아래 구현한 guaranteed
  return-to-safe/home behavior.

다음은 port하지 않는다.

- ZMQ request path, 이전 `engine/` ownership, 이전 configuration file,
  이전 UI state propagation;
- environment path로 root-level `zed_cylinder_bench.py`를 발견하는 방식;
- background에 anchor하는 숨은 fallback;
- 일반 reconstruction primitive의 cylinder-only 이름;
- FK-metrology mode 밖에서의 blanket `NO ICP` rule.

두 모드를 명시적으로 유지한다.

```text
metrology mode:
  transform-and-merge only
  ICP/VIO refinement forbidden
  misregistration is the measurement

reconstruction mode:
  coarse pose from calibrated FK and/or VIO
  bounded robust registration allowed
  raw pose hypotheses retained for ablation
```

## 10. External source 도입 matrix

External source는 우선 reference다. license, dependency surface, maintenance
state, isolation test를 기록하기 전에는 코드를 복사하지 않는다.

| Source | License/status | 사용할 것 | 사용하지 않을 것 |
| --- | --- | --- | --- |
| [MUKCa](https://github.com/platonics-delft/kinematics_calibration) | MIT; research project | two-socket fixture, closure objective, held-out workspace 개념 | production dependency 또는 load sag도 모델링한다는 주장 |
| [robot_calibration](https://github.com/mikeferguson/robot_calibration) | Apache-2.0; mature ROS 2 package | sample/finder/model/error-block 개념, parameter mask, export/report pattern | fit/gap 분석 전 직접 runtime dependency로 사용 |
| [industrial_calibration](https://github.com/ros-industrial/industrial_calibration) | Apache-2.0; ROS-Industrial/Ceres | optimizer 및 accuracy-analysis pattern | Robot/Pilot runtime에 stack 전체를 import |
| [robot_cal_tools](https://github.com/Jmeyer1292/robot_cal_tools) | Apache-2.0; standalone Ceres-oriented | calibration problem 구조와 synthetic test | 검토하지 않은 parameter convention |
| [multirobot-calibration](https://github.com/ctu-vras/multirobot-calibration) | MATLAB research toolbox; code reuse 전 license 확인 필요 | factor taxonomy와 observability 실험 | MATLAB code 복사 또는 runtime dependency |
| [Kalibr](https://github.com/ethz-asl/kalibr) | BSD-family; established offline toolbox | offline camera/IMU spatial-temporal calibration procedure | runtime dependency 또는 arm ground truth |
| [OpenVINS](https://github.com/rpng/open_vins) | GPL-3.0 | algorithm/paper 비교와 VIO 진단 | 명시적인 GPL 결정 없이 EleSim에 복사/연결 |
| [RegHEC](https://github.com/Shiyu-Xing/RegHEC) | MIT; research code | arbitrary-object hand-eye objective, coarse-to-fine registration | 오래된 dependency tree vendor 또는 ICP를 truth로 신뢰 |
| [eye_hand_calib_pointcloud](https://github.com/payam-nourizadeh/eye_hand_calib_pointcloud) | MIT; small third-party adaptation | Linux/headless dataset과 output idea | 재현 전 production dependency 또는 performance 주장 |
| [roboreg](https://github.com/roboreg/roboreg) | Apache-2.0; API가 developmental로 표시됨 | robot-mesh-to-depth validation 선택지 | 핵심 calibration dependency 또는 ideal mesh와 compliant arm의 혼동 |
| [Peters et al.](https://arxiv.org/abs/2206.03430) | paper; verified official implementation 없음 | offline-SLAM formulation, point-to-plane factor, degeneracy check | source 복사 또는 arbitrary scene이 항상 충분하다는 주장 |
| [KUKA elastostatic example](https://github.com/Walid-khaled/Elastostatic-Calibration-of-7DOF-KUKA-Linear-Axis) | MIT; small MATLAB example | compliance 수학과 실험 용어 | 직접 code port 또는 KUKA coefficient의 일반화 |
| [Open3D reconstruction system](https://github.com/isl-org/Open3D/blob/main/docs/tutorial/t_reconstruction_system/integration.rst) | MIT; 유지보수되는 범용 3D library | known-pose RGB-D TSDF/voxel integration, mesh 추출, offline benchmark | TSDF output을 pose/calibration ground truth로 취급하거나 모든 runtime image에 강제 설치 |
| [Voxblox](https://github.com/ethz-asl/voxblox) | BSD-3-Clause; 성숙했지만 ROS/C++ 중심 | TSDF weighting, block storage, serialization, mesh 비교 기준 | 오래된 ROS stack과 C++ dependency 전체를 Pilot에 직접 결합 |
| [RTAB-Map](https://github.com/introlab/rtabmap) | BSD-family; 성숙한 graph-SLAM system | ZED/D435 sequence의 loop-closure 및 reconstruction 외부 baseline | EleSim 역할 경계를 대체하는 내장 SLAM framework 또는 Robot runtime dependency |
| [Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics) | BSD-2-Clause; semantic TSDF research system | 장래 semantic reconstruction의 factor와 output 구조 참고 | 현재 geometric calibration 범위에 semantic stack을 선제 도입 |

### 10.1 외부 code 도입 절차

dependency를 복사하거나 추가하기 전에 다음 source record를 만든다.

```text
repository URL
exact commit
license and NOTICE obligations
files/ideas being used
dependency and platform requirements
why reimplementation is insufficient
tests proving equivalent behavior
release-image impact
rollback/removal plan
```

우선순위는 다음과 같다.

1. paper의 experiment를 작은 자체 구현으로 재현한다.
2. mature tool을 offline에서 실행하고 output을 변환한다.
3. 유지보수되는 permissive library를 developer/calibration environment에서만
   사용한다.
4. 별도 review를 거친 최후의 수단으로만 code를 vendor한다.

OpenVINS는 GPL-3.0이므로 명시적인 licensing decision 전에는
paper/reference-only로 유지해야 한다. Kalibr은 external offline tool이어야
하며 모든 release image에 설치하지 않는다. 작은 research repository는
test vector와 아이디어를 제공해야지 operational dependency가 되어서는 안 된다.

### 10.2 Reconstruction library 선택 규칙

Phase 1의 FK metrology는 NumPy 기반 transform-and-merge로 시작한다. 이 단계에
ICP나 TSDF dependency를 넣으면 pose 오차가 fusion에 의해 가려질 수 있다.

Phase 6에서 mesh가 실제로 필요해지면 `Open3D`를 첫 prototype 후보로 평가한다.
채택 여부는 popularity가 아니라 동일 raw session에 대한 다음 비교로 결정한다.

```text
baseline: deterministic point/voxel fusion
candidate: optional Open3D TSDF adapter
reference: offline Voxblox 또는 RTAB-Map replay

compare:
  held-out surface error
  pose perturbation sensitivity
  peak resident memory
  processing time
  deterministic replay/hash stability
  arm64 wheel/build availability
  isolated release size and dependency closure
```

Open3D adapter는 Pilot의 offline reconstruction 경계 뒤에 있어야 하며 import
실패 시 다른 알고리즘으로 조용히 fallback하면 안 된다. 선택하지 않았거나
설치되지 않았다면 명시적으로 `backend_unavailable`을 반환한다. Voxblox와
RTAB-Map은 최초 구현의 dependency가 아니라 동일 dataset을 재생하는 외부
baseline이다. Kimera-Semantics는 geometric reconstruction과 uncertainty가
안정된 뒤 별도 요구가 생기기 전까지 범위 밖이다.

## 11. Protocol 계획

첫 offline phase에서는 protocol message를 추가하지 않는다.

### 11.1 1단계 재사용

- live RGB-D latest-only, best effort, depth one을 유지한다.
- timestamp interpolation/straddle semantics를 정의하고 test한 뒤에만 기존
  Robot `RgbdFrame.arm_q`를 채운다.
- raw arm state를 보존한다. sag-corrected 값만 전송하지 않는다.
- Sim의 RGB-D publisher를 사용해 결정론적인 replay fixture를 만든다.
- drop과 coverage를 보고한다. 무제한 queue를 추가하여 live stream을 reliable하게
  만들려 하지 않는다.

### 11.2 Protocol change가 정당화되는 시점

operator-controlled remote scan이 유용하다는 것을 offline experiment가
증명한 뒤에만 protocol change를 정당화한다. 다음 순서로 갱신한다.

1. protocol design decision과 major/additive compatibility decision;
2. `packages/elesim_interfaces` ROSIDL;
3. `packages/protocol/src/elesim_protocol/contracts.py`;
4. strict payload validator와 role/authority table;
5. SROS2 topic policy;
6. multi-process contract test;
7. release isolation test;
8. documentation.

후보 control message는 bounded metadata만 운반한다.

```text
start_scan       UI -> Pilot operator intent, then Pilot -> Robot lease-bound request
scan_status      Robot/Pilot -> Pilot/UI bounded progress and diagnostics
cancel_scan      UI -> Pilot, then Pilot -> Robot
scan_result      Pilot -> UI/Sim metadata and artifact identity
```

Large point cloud, RGB-D archive, mesh를 `PeerEnvelope`에 넣지 않는다. bulk
transport를 새로 만들기 전에 기존 live RGB-D와 Pilot-side keyframe capture로
충분한지 측정한다. 충분하지 않다면 bounds, backpressure, resumability, hash,
authorization, SROS2/network behavior를 다루는 별도 bulk-data design을 작성한다.

### 11.3 Authority

- UI는 workflow를 요청하며 scan motor position을 직접 command하지 않는다.
- Pilot은 scan planning과 calibration workflow를 소유한다.
- Robot은 active Pilot motion lease를 검증하고 실행/safety를 소유한다.
- Robot cancellation 또는 deadman expiry는 Pilot/UI를 기다리지 않고 arm을
  locally safe condition으로 돌려야 한다.
- Sim mock scan은 이미 정의된 Sim motion/session authority를 사용하며 Robot
  behavior를 약화하지 않는다.

## 12. Sim과 UI 계획

### 12.1 Sim

Sim은 network mesh transfer 전에 결정론적 source를 지원해야 한다.

- analytic plane/sphere/cylinder/asymmetric-object point cloud;
- configurable depth noise, missing pixel, outlier, timestamp skew, joint
  offset, backlash, compliance;
- immutable mock geometry로서의 imported OBJ/PLY;
- FK-only, VIO-like drift, known-ground-truth pose channel;
- 별도 색상으로 raw per-view point, FK fusion, refined fusion, residual vector
  rendering;
- runtime model-builder import 없음.

Genesis rendering이 필요할 때만 synthetic generator를 Sim 소유로 둔다.
여러 test suite에서 공유하는 pure geometry/noise fixture는 작은 test data 또는
protocol-independent test helper로 두며, 새 deployable application을 만들지
않는다.

### 12.2 UI

첫 UI는 one-click magic calibrator가 아니라 diagnostic UI다.

필수 화면:

- selected target/Robot과 active lease;
- camera profile과 serial;
- session phase, frame count, drop reason, angular/workspace coverage,
  pose straddle, depth-invalid fraction;
- raw/FK/refined cloud visibility toggle;
- train/validation residual, condition number, acceptance/rejection reason;
- artifact identity, scope, inactive/active 여부;
- validation 뒤의 명시적 activate action; 자동 activation 없음.

UI는 bounded summary와 preview를 받는다. calibration dataset을 parse하거나,
optimizer를 실행하거나, raw point-cloud state를 보유하지 않는다.

## 13. Micro-managed 구현 backlog

아래 각 task는 하나의 review 가능한 commit을 목표로 한다. 더 나누는 것은
허용하지만 phase gate를 가로질러 합치면 안 된다.

### Phase 0: evidence와 baseline 고정

#### T0.1 현재 sag 동작 inventory

검사할 파일:

- `pilot/src/elesim_pilot/robot/arm/sag_model.py`
- `sim/src/elesim_sim/robot/arm/sag_model.py`
- `pilot/config/sag/*.json`
- `ui/config/sag/*.json`
- `sag_model`을 주입하는 모든 IK/FK call site

Deliverable:

- units, parameter 의미, call direction, 중복 code에 대한 test-backed inventory;
- behavior 변경 없음.

계수의 의미를 복원할 수 없으면 중단한다. unknown으로 기록하며 변수명에서
물리적 의미를 추론하지 않는다.

#### T0.2 scan-session v1 parser 정의

Owner: Pilot offline tooling.  
Input: section 6에 맞는 directory.  
Output: immutable typed session/frame object.

Test:

- valid minimal session;
- non-finite number rejection;
- hash mismatch rejection;
- path traversal/symlink rejection;
- incomplete session rejection;
- unit/profile/model mismatch rejection;
- deterministic manifest digest.

Camera, ROS, DDS, hardware import는 허용하지 않는다.

#### T0.3 deterministic synthetic dataset writer

Owner: Sim test tooling 또는 boundary review 후 Pilot-test-only fixture.
알려진 pose와 deterministic seeded noise를 가진 plane, sphere, cylinder,
asymmetric multi-plane object sequence를 생성한다.

Test에는 다음 독립 주입이 포함되어야 한다.

- joint zero error;
- hand-eye error;
- Gaussian depth noise;
- depth edge outlier;
- frame timestamp skew;
- pose-dependent sag;
- forward/reverse backlash;
- missing frame.

Ground truth는 test-only 별도 file에 두며 under-test solver가 이를 읽지 못하게
한다.

#### Gate P0

- runtime/protocol 변경 없음;
- ROS/hardware 없이 schema/parser test 통과;
- synthetic dataset bit-reproducible;
- 현재 required quality gate 통과.

### Phase 1: old architecture가 아닌 FK metrology port

#### T1.1 pure fusion primitive port

Source reference: `origin/3d_fusion:engine/vision/scan/fusion.py`.  
Owner: Pilot reconstruction package.  
section 9에 열거된 pure geometry만 port한다.

Test:

- transform direction과 units;
- float32 preservation;
- empty array;
- voxel-key collision regression;
- crop conservativeness;
- deterministic output ordering 또는 명시적 unordered comparison;
- ICP/import side effect 없음.

#### T1.2 FK pose/provenance metric port

Input: raw `q4`, selected calibrated model, hand-eye artifact.  
Output: `T_world_camera`, camera origin/look/right, model/artifact hash.

대표 joint state에서 현재 Pilot FK와 비교하고 convention ambiguity에서 실패하는
test를 작성한다.

#### T1.3 FK metrology replay 구현

Input: synthetic 또는 captured session.  
Output: fused cloud와 report; parameter optimization 없음.

필수 report: single-view noise floor, fused wall thickness, observation span,
camera travel, frame rejection reason, pose-source=`fk`.

#### Gate P1

- 1-degree/5-mm pose error 주입 시 fusion residual이 측정 가능하게 증가;
- 이전 `engine`, ZMQ, root bench import 없음;
- cylinder는 package identity가 아닌 하나의 plugin/fixture;
- isolated Pilot release context에서 offline test 통과.

### Phase 2: coherent Robot evidence

#### T2.1 arm/camera time semantics 명세

code 작성 전에 다음 decision record를 작성한다.

- clock domain;
- camera profile마다 device timestamp가 가능한지;
- arm sample 사이의 interpolation 규칙;
- 최대 arm sample age와 pose straddle;
- clock reset/non-monotonic timestamp 동작;
- raw와 corrected `arm_q` 의미.

#### T2.2 Robot RGB-D arm state 채우기

Robot만 변경한다. camera worker에 arm-snapshot provider를 주입하며 Pilot FK를
import하지 않는다. camera acquisition 전후에 arm sample을 기록하고 time
contract가 허용할 때만 interpolation한다. 그렇지 않으면 diagnostic과 함께
`arm_q`를 생략한다.

Test:

- 정확한 stationary interpolation;
- bounded moving interpolation;
- stale/straddled state rejection;
- device capture failure;
- camera worker cleanup과 Robot safety 불변;
- 기존 latest-only DDS semantics 불변.

#### T2.3 Pilot-side bounded keyframe recorder

기존 latest-only RGB-D를 subscribe한다. 측정된 joint motion, view novelty,
depth validity, minimum interval로 keyframe을 선택한다. scan-session v1을
atomic하게 기록한다. image encoding/disk write로 DDS receive loop를 block하지
않고 bounded worker queue를 사용하며, queue가 차면 drop/reject와 counter를
기록한다.

#### Gate P2

- 새 message type 없음;
- queue bound와 disk-space limit test;
- session replay가 FK fusion을 재현;
- packet/frame loss가 hang이 아니라 coverage 감소와 명시적 counter가 됨;
- Robot deadman과 motion lease test 통과.

### Phase 3: static calibration

#### T3.1 Contact dataset schema와 fixture protocol

socket identity, known distance와 measurement uncertainty, raw joint state,
payload/tool identity, approach direction, acceptance/retry reason을 정의한다.
사용할 수 없는 force sensor를 가정하지 않고 contact를 감지하는 물리 절차를
명시한다.

#### T3.2 Parameter-mask optimizer

충분하면 우선 NumPy/SciPy로 구현한다. 측정된 필요성과 dependency review 뒤에만
Ceres를 도입한다. optimizer는 free/fixed parameter group을 명시적으로 요구하고
gauge가 고정되지 않은 configuration을 거부해야 한다.

#### T3.3 Static artifact와 held-out validation

fitting 전에 pose/workspace 기준으로 sample을 분리한다. artifact schema v1과
사람이 읽을 수 있는 report를 생성한다. 자동 activation은 하지 않는다.

#### Gate P3

- synthetic recovery가 noise-derived tolerance 안에 있음;
- 잘못된 socket distance와 잘못 label된 sample을 validation이 검출;
- held-out error가 nominal FK보다 개선;
- parameter rank/covariance 보고;
- activation mismatch fail closed.

### Phase 4: hand-eye와 time calibration integration

#### T4.1 Kalibr converter, Kalibr runtime 아님

검토된 Kalibr output을 versioned EleSim sensor-calibration artifact로 변환한다.
frame name, unit, camera serial/profile, timestamp, matrix orthonormality를
검증한다.

#### T4.2 Known-target hand-eye solver/replay

robot geometry를 고정한다. 알려진 extrinsic synthetic test와 real-data report
생성을 제공한다.

#### T4.3 Arbitrary-object bounded hand-eye refinement

trimmed point-to-plane correspondence와 제한된 SE(3) delta를 사용한다.
FK-only, known-target, refined 결과를 held-out view에서 비교한다.

#### Gate P4

- 첫 solve에서 time offset을 sag와 동시에 free로 두지 않음;
- arbitrary-scene refinement가 설정된 extrinsic bound 밖으로 움직이지 않음;
- refinement가 held-out cloud consistency 개선;
- degenerate plane/cylinder sequence reject.

### Phase 5: compliance

#### T5.1 payload/direction experiment 설계

각 pose에서 known payload mass/CoM, gravity vector, approach sign, settled
time, temperature, repeated measurement를 기록한다.

#### T5.2 저차원 compliance model 구현

linear stiffness/basis model부터 시작한다. legacy coefficient는 의미가 알려진
경우에만 명시적인 migration wrapper로 읽는다.

#### T5.3 Validation과 activation

보지 않은 pose와 최소 한 개 held-out payload를 요구한다. forward/reverse
hysteresis를 별도로 보고한다. scope 밖 activation은 diagnostic을 반환하고
uncorrected static model을 사용해야 하며, 조용히 extrapolate하지 않는다.

#### Gate P5

- zero-load static accuracy가 악화되지 않음;
- unseen-payload performance가 개선;
- 과도한 extrapolation reject;
- 동일 artifact에서 Pilot과 Sim이 독립 package test에서 동일 correction 생성.

### Phase 6: reconstruction product

#### T6.1 Keyframe registration

correspondence filter, robust point-to-plane refinement, pose hypothesis 보존을
구현한다. local minima, insufficient overlap, repeated geometry, bad
initialization을 test한다.

#### T6.2 Fusion과 mesh

point-cloud/voxel fusion부터 시작한다. TSDF/mesh는 memory, latency, accuracy를
측정한 뒤에 추가한다. 가능한 경우 각 vertex/voxel output에 observation count와
uncertainty를 보존하거나 요약한다.

별도 benchmark task에서 동일 session을 deterministic baseline과 optional
Open3D adapter로 재생한다. Voxblox/RTAB-Map 결과는 외부 reference artifact로만
수집한다. backend 선택, 정확한 version/commit, parameter, random seed는 run
manifest에 기록하며 backend 부재를 성공이나 fallback으로 처리하지 않는다.

#### T6.3 Sim import와 visualization

path containment, size/vertex limit, finite coordinate validation, deterministic
mock file을 사용해 local PLY/OBJ artifact를 load한다. visual mesh가 load됐다는
이유만으로 Genesis collision behavior를 변경하지 않는다.

실행 중 새로 생성되어 사전 catalog 등록이 불가능한 reconstruction mesh는
Genesis scene을 매번 rebuild하는 방식으로 시작하지 않는다. 별도 prototype에서
다음의 bounded visual/collision 이중 표현을 검증한다.

- scene build 전에 `enable_custom_vverts=True`인 visual-only triangle pool을
  하나 예약한다. 이것은 EleSim 설계상의 이름이며 Genesis의 별도 entity type이
  아니다.
- pool face topology는 독립된 triangle 연속열로 고정하고, reconstruction
  snapshot을 triangle soup으로 펼친 뒤 `set_vverts()`로 vertex 위치만
  갱신한다. 사용하지 않는 triangle은 퇴화시켜 숨긴다.
- 초기 용량은 4,096 triangles로 제한한다. live preview는 1,024 triangles,
  settled result는 4,096 triangles를 목표로 하며, 별도 profiling을 통과한
  경우에만 최대 8,192 triangles를 허용한다. 초과 입력은 Pilot의 명시적
  decimation 결과가 없으면 거부한다.
- visual snapshot 갱신은 reconstruction revision 변경 시 또는 최대 2--5 Hz로
  제한한다. Genesis/WebRTC frame loop와 DDS callback에서 mesh 변환을 수행하지
  않는다.
- visual vertex 변경은 collision geometry를 변경하지 않는다. planning/physics는
  별도로 미리 예약한 bounded box/capsule/convex proxy pool을 사용하고 두 표현은
  동일 artifact revision/hash로 묶는다.
- runtime normal, shading, camera sensor 반영, GPU memory, scene-build time,
  `set_vverts()` p50/p95 latency와 observer/hand-eye FPS를 Genesis 1.2.0에서
  측정한다. normal 갱신이 불충분하면 첫 prototype은 neutral/unlit 표현과
  depth/silhouette 검증으로 제한한다.
- 이 prototype이 실패하거나 품질 한계를 넘는 경우에만 cancellable scene
  replacement/rebuild를 명시적 fallback으로 설계한다. 조용한 rebuild나
  media-ready 상태의 장시간 정지는 허용하지 않는다.

#### Gate P6

- raw session과 run config만으로 output 재생성 가능;
- corrupted/oversized artifact fail closed;
- mesh가 `PeerEnvelope`를 통과하지 않음;
- Sim visualization은 optional이며 physics/DDS/media를 block하지 않음.

### Phase 7: operator workflow와 protocol

#### T7.1 Contract decision

기존 operator intent/result와 telemetry로 모든 bounded workflow metadata를
전달할 수 있는지 결정한다. 별도의 authority/QoS 의미가 있을 때만 message
type을 추가한다.

#### T7.2 Pilot/Robot scan lifecycle

명시적인 idle/preparing/capturing/processing/completed/failed/cancelled state,
idempotent cancellation, timeout, lease loss, boot-ID fencing을 구현한다.

#### T7.3 UI diagnostics와 명시적 activation

section 12.2의 UI를 구현한다. mismatch, poor observability, failed validation,
incomplete evidence에서는 activation을 disable한다.

#### Gate P7

- runtime wiring 전에 contract registry와 docs 갱신;
- startup queue는 512/heartbeat timeout bound를 유지;
- control QoS와 RGB-D latest-only semantics를 유지;
- multi-process smoke, SROS2 policy, release build, release verification 통과;
- manual multi-host loss와 cancellation test는 계속 manual gate로 기록.

## 14. Test matrix

### 14.1 Pure unit test

- SE(3) convention과 inverse/composition property;
- finite/shape/unit validation;
- depth-to-point projection;
- voxel, crop, normal, residual function;
- robust-loss와 parameter-bound 동작;
- schema parsing, hashing, path containment, atomic finalization;
- deterministic random seed.

### 14.2 Property와 mutation test

- scene과 pose에 common rigid transform을 적용해도 gauge fixing 뒤 relative
  residual은 변하지 않음;
- injected joint error를 키우면 FK-metrology residual이 증가;
- 필요한 excitation을 제거하면 rank가 감소하거나 reject;
- camera profile 또는 model hash 교체를 reject;
- stale frame, duplicate sequence, non-monotonic time, NaN, oversized image,
  queue-full path가 계속 visible failure;
- optimizer가 test-only ground truth를 읽지 못함.

### 14.3 Synthetic integration scenario

| Scenario | Expected outcome |
| --- | --- |
| perfect FK + noisy depth | fused residual near depth noise floor |
| joint-zero error only | static optimizer recovers offset within tolerance |
| hand-eye error only | hand-eye stage recovers; static stage does not absorb it |
| payload sag only | static artifact remains unchanged; compliance stage improves |
| timestamp skew while moving | capture rejects/flags frames; no fake sag fit |
| one plane only | observability rejection |
| long cylinder only | axial/rotational degeneracy reported |
| asymmetric multi-plane target | full pose becomes observable |
| low overlap | registration failure, not fabricated mesh |
| frame loss | reduced coverage and explicit counters; bounded memory |

### 14.4 Hardware/manual gate

- ZED Mini와 D435의 camera USB/SDK/device timestamp behavior;
- 실제 capture load에서 synchronized arm-state age와 p95 pose straddle;
- fixture repeatability와 socket-distance measurement uncertainty;
- payload mass/CoM과 approach-direction 반복;
- ZED/BNO085 time calibration과 temperature drift;
- multi-host RGB-D bandwidth, fragmentation, loss, p95 frame age;
- 추가 topic에 대한 SROS2 policy;
- Pilot/UI/DDS loss 상황의 Robot stop/return behavior;
- 독립적으로 측정한 object의 reconstruction accuracy.

이는 simulated pass로 대체할 수 없다.

## 15. Performance와 storage budget

image/cloud를 처리하는 모든 구현 task는 다음을 명시해야 한다.

- 최대 resolution과 frame당 byte;
- session당 최대 keyframe 수;
- bounded in-memory queue depth;
- 최대 session/artifact byte;
- disk-full behavior;
- CPU/GPU timing과 p50/p95 frame age;
- DDS callback, Robot safety loop, Pilot worker, offline process 중 어느 곳에서
  실행되는지.

초기 보수적 규칙:

- Robot safety/control loop에서 point-cloud operation 금지;
- DDS receive callback에서 disk encoding 금지;
- live RGB-D latest-only depth one 유지;
- 모든 dense cloud 전송보다 offline replay 우선;
- 명확한 이점이 profiling으로 입증되지 않는 한 keyframe은 Robot에서 full
  XYZ로 만들지 말고 depth/RGB에서 선택;
- mesh generation은 cancellable하며 vertex/memory limit을 가짐;
- partial result는 incomplete로 표시하고 절대 activate하지 않음.

## 16. 추가 조사 또는 실험이 필요한 질문

다음 질문은 추측으로 닫지 말고 open 상태로 유지한다.

1. physical arm은 device timestamp가 있는 encoder sample을 제공하는가, 아니면
   host read time만 제공하는가?
2. 배포된 SDK version에서 ZED Mini가 image, depth, IMU, pose timestamp를 하나의
   clock domain으로 제공할 수 있는가?
3. BNO085는 ZED와 같은 rigid terminal body에 부착됐는가, 아니면 distributed
   bending을 관측할 수 있는 다른 link에 부착됐는가?
4. 현재 sag coefficient를 만든 물리 측정은 무엇이며 단위는 무엇인가?
5. force/torque sensor 없이 이용 가능한 반복 contact signal은 무엇인가?
6. 실제 운용을 대표하는 payload mass/CoM과 temperature range는 무엇인가?
7. grasping과 scanning에 필요한 absolute object/endpoint accuracy는 얼마인가?
8. real routed VPN에서 Pilot-side latest-only RGB-D capture로 충분한가, 아니면
   별도의 bounded bulk-data path가 필요한가?
9. 이후 소비되는 reconstruction 표현은 visual mesh, collision mesh,
   dimensional measurement 중 무엇인가? 아니면 서로 다른 tolerance를 가진
   모두인가?
10. production에는 없더라도 calibration 중에 두 번째 독립 camera 또는 임시
    external marker를 사용할 수 있는가?

## 17. 권장 첫 세 coding assignment

runtime scan button이나 protocol extension 전에 다음을 수행한다.

1. **T0.1 현재 sag inventory** — 사실을 복구하고 unknown을 노출한다.
2. **T0.2/T0.3 dataset schema와 deterministic synthetic generator** —
   evidence/test boundary를 만든다.
3. **T1.1/T1.2/T1.3 FK-metrology replay** — `3d_fusion`의 과학적으로 유용한
   부분을 port하고 알려진 pose error가 측정 가능한지 증명한다.

이 commit을 마친 뒤 synthetic report를 검토하고 coherent Robot capture로
진행할지 수학 model을 수정할지 결정한다. 이 방식은 첫 구현을 되돌릴 수 있게
하며, 검증되지 않은 estimator를 중심으로 거대한 DDS/UI surface가 생기는 것을
막는다.

## 18. Luna 실행 계약

coding agent에게 한 번에 phase 전체를 주지 않는다. 정확히 하나의 `Tn.m`
task만 다음 template으로 전달한다. unknown이 contract를 바꾸면 agent는
범위를 넓히지 말고 중단해야 한다.

```markdown
# Objective

관찰 가능한 하나의 결과를 한 문장으로 설명한다.

# Read first

- AGENTS.md
- docs/architecture.md
- process boundary를 건드리면 docs/dds_contracts.md
- 이 설계의 관련 task와 gate

# Allowed files

- 수정할 수 있는 정확한 기존 경로
- 생성할 수 있는 정확한 새 경로

# Forbidden changes

- 수정할 수 없는 role과 directory
- 명시하지 않은 protocol/config/release 변경 금지
- host dependency 설치 금지
- 명시하지 않은 generated artifact commit 금지

# Input contract

- Schema version
- Shape와 SI unit
- Coordinate-transform direction
- Timestamp/clock semantics
- Identity와 hash requirement
- Bound와 invalid-input behavior

# Output contract

- Return type/artifact path
- Status와 rejection reason vocabulary
- Provenance field
- Atomicity와 partial-failure behavior

# Required tests written first

- Nominal case
- Boundary and empty case
- Corrupt/non-finite/mismatch case
- Relevant property or mutation
- Package-isolation case

# Acceptance criteria

- 이름이 지정된 test와 quality gate 통과
- fixture noise로 정당화한 numerical tolerance
- unrelated worktree 변경 없음
- `git diff --check` 통과

# Stop conditions

- 필요한 physical convention을 찾을 수 없음
- 기존 behavior가 이 task와 모순
- schema/protocol major decision 필요
- external license 불명
- test에 현재 없는 real hardware가 필요

# Handoff

- 변경한 파일
- test와 정확한 결과
- 만든 가정
- 여전히 필요한 manual gate
- 구현하지 않고 제안하는 다음 `Tn.m` task
```

추가 실행 규칙:

- agent에게 편집 전에 inspect하고 handoff에서 정확한 source path를 인용하게
  한다.
- numerical/schema 변경은 production code 전에 red test를 요구한다.
- task마다 parameter family 하나, transport surface 하나, UI workflow 하나만
  허용한다.
- numerical fixture에는 known ground truth, seed, unit, tolerance를 제공하며
  acceptance threshold를 agent가 만들도록 요구하지 않는다.
- phase gate를 넘어 task가 자동으로 계속되지 않게 한다. report를 검토하고
  다음 phase를 명시적으로 승인한다.
- 이전 artifact schema가 부족하다는 것을 발견하면 새 field를 소비하기 전에
  전용 task로 schema를 변경한다.
- hardware 부재로 skip한 test는 pass가 아니다. manual gate에 기록한다.

첫 assignment 예시는 다음과 같다.

```markdown
# Objective

runtime behavior를 변경하지 않고 기존 sag model을 inventory하며 현재
observable한 unit, shape validation, Pilot/Sim equivalence를 고정하는 test를
추가한다.

# Allowed files

- docs/design/robot_self_calibration_and_3d_reconstruction.md
- pilot/tests/test_sag_model_inventory.py
- sim/tests/test_sag_model_inventory.py
- 해당 package test directory 아래의 새 test fixture

# Forbidden changes

- production source 수정 금지
- coefficient 수정 금지
- protocol/config/release 수정 금지
- legacy sag file 삭제 또는 migration 금지

# Required evidence

- `segment_errors_from_model`의 모든 production consumer
- 각 boundary에서 input이 radian인지 degree인지
- output unit과 sign convention
- Pilot/Sim evaluator가 동일한지 또는 의도적으로 다른지
- commit된 coefficient의 provenance를 알 수 없는 경우 이를 명시

# Required tests

- 빈 model은 zero correction을 반환
- legacy/refined model shape validation
- non-finite coefficient rejection behavior를 기록
- 대표 Pilot/Sim output이 동등

# Acceptance criteria

- package test와 required gate 통과
- runtime behavior와 JSON byte가 변경되지 않음
- 알려지지 않은 physical meaning을 추측하지 않음
```

## 19. 전체 프로그램의 Definition of done

다음 조건을 모두 만족할 때에만 program이 완료된다.

- 모든 raw evidence를 replay할 수 있고 provenance가 완전하다.
- sensor, static kinematic, hand-eye, compliance, scene parameter가 분리된
  versioned artifact 또는 명시적인 fixed group이다.
- 모든 accepted calibration에 rank/covariance와 held-out validation이 있다.
- mismatch 및 scope 밖 artifact는 fail closed한다.
- FK-only, VIO-only, refined reconstruction을 비교할 수 있다.
- 알려진 degeneracy를 실행 가능한 reason과 함께 reject한다.
- Pilot/Robot/Sim/UI가 독립적으로 deploy 가능하다.
- DDS contract가 bounded이며 large artifact가 `PeerEnvelope`에 들어가지 않는다.
- Robot safety와 lease behavior가 reconstruction과 독립적이다.
- generated release에는 자신이 소유한 runtime dependency만 포함된다.
- required/extended gate, release build/verify, 새 synthetic test가 모두 통과한다.
- 실제 hardware와 multi-host manual gate를 선언된 operating scope에 대해
  별도로 기록하고 통과한다.

그보다 부족한 결과도 가치 있는 실험일 수 있다. 그러나 신뢰할 수 있는 로봇
calibration으로 활성화하지 말고 실험으로 명확히 표시해야 한다.
