# EleSim Connections / Jetson 기동 장애 — 원인과 수정 위치

작성: 2026-10-27~28 (KST). Git 저장소가 아닌 배포 디렉터리에서 설치 파일을 직접 수정한 기록이다. 이 문서는 후속 이슈 등록용이며, 저장소에 커밋하거나 이슈를 게시한 상태는 아니다.


## 리니어 199 독립 도달성 계산 — 2026-10-28 10:59 KST

사용자 요청으로 실제 명령 없이 현재 Go2 pose·설치 FK 모델·리니어 199 범위를 사용해 world 목표 (0.2976,0.1682,0.2901)m를 위치 전용 IK로 계산했다. SciPy bounded least_squares, 현재 자세/중앙값/고정 난수 seed를 포함한 24개 초기값 중 11개에서 위치 오차 1mm 미만의 해를 얻었다. 예시 해의 리니어 u=49.36323584, q=(-0.0438360782,1.5297244040,0.2319646757,0.4273716586), 계산 FK=(0.2976000000,0.1682000000,0.2900999999). 리니어 최대 199와 나머지 관절 범위를 만족한다.

이는 **위치 기구학상 가능하다는 근거**이며 자기 충돌/목표 공 충돌/이동 경로/실제 하중 및 실행은 이번 계산에서 검증하지 않았다. 자세 방향은 강제하지 않았다. 기존 cuRobo 요청/실행 상태를 변경하지 않았고 모터 명령은 0건이다. [입력 snapshot](diagnostics/ik-reach-199-20261028/input.json), [계산 코드](diagnostics/ik-reach-199-20261028/calculate.py), [전체 해](diagnostics/ik-reach-199-20261028/calculation.log), [결과](diagnostics/ik-reach-199-20261028/result.json).

---

## 리니어 199 상한 적용 완료 — 2026-10-28 10:56 KST

사용자 요청: 200도 도달하기 어려우므로 200 미만으로 강제하고 다른 설정은 바꾸지 않는다. 상한을 199로 정했다. Pilot 설정의 linear_u_limit_deg만 259→199로 변경했고, linear_u_max_deg=259 환산 기준은 유지했다. 계획용 URDF의 j_plate_housing lower와 metadata q_bounds를 -0.23→-0.17671814671814673m로 일치시키고 URDF artifact hash를 갱신했다. robot.yml 충돌 구/속도/다른 축/오프셋/전류 제한은 수정하지 않았다.

사용자가 “팔 잡을게. 그냥 니가 강제로 껐다 켜라”라고 확인하여 Robot/Pilot을 정지하고 적용·재시작했다. native Robot의 arm.linear_u_limit_deg 및 mapping.linear_u_limit, Pilot 제한과 계획 모델을 모두 199로 맞췄다. native installed YAML은 두 제한값 외 의미상 변경이 없음을 비교 확인했다. Connections도 새 release로 갱신했다. 10:56:48 snapshot에서 DDS/Pilot online, Torque OFF, safety_fault 없음, 리니어 실제값 151.12(제한 안쪽)를 확인했다. 토크 활성화·자동 복귀·모터 이동 명령은 보내지 않았다. UI/Sim은 재시작하지 않았다. 현재 위치가 새 범위 밖이면 기존 measured_planning_start 검사가 계획을 거절하며 자동 복귀 명령은 보내지 않는다. 기존 미세 경계 오차 허용은 유지한다.

적용 release: `0c20db48d1281269ed532215df46fd740787dd41ff37b8ffe27d884530cefabb`, 이미지 `pale-sable` 재사용. 설정/모델 범위 일치와 artifact hash만 확인했으며 TDD·GPU 계획·실제 모터 이동은 하지 않았다. 준비/적용 스크립트와 원본 백업은 `diagnostics/linear-limit-199-20261028/` 및 Jetson의 동일 경로에 있다.

```diff
--- a/apps/pilot/config/config.yaml
+++ b/apps/pilot/config/config.yaml
@@ -214,7 +214,7 @@
       current_yellow_ma: 1800
       host_hw_cmd_hz: 30.0
       host_hw_read_hz: 20.0
-      linear_u_limit_deg: 259.0
+      linear_u_limit_deg: 199.0
       linear_u_max_deg: 259.0
       lji_velocity_accel_limit_deg_s2: 350.0
       lji_velocity_deadman_s: 3.0

--- a/containers/runtime-snapshot/config/pilot/config.yaml
+++ b/containers/runtime-snapshot/config/pilot/config.yaml
@@ -220,7 +220,7 @@
       current_yellow_ma: 1800
       host_hw_cmd_hz: 30.0
       host_hw_read_hz: 20.0
-      linear_u_limit_deg: 259.0
+      linear_u_limit_deg: 199.0
       linear_u_max_deg: 259.0
       lji_velocity_accel_limit_deg_s2: 350.0
       lji_velocity_deadman_s: 3.0

--- a/containers/runtime-snapshot/data/models/planning/zed-mini/arm.urdf
+++ b/containers/runtime-snapshot/data/models/planning/zed-mini/arm.urdf
@@ -124,7 +124,7 @@
     <child link="housing" />
     <origin xyz="0 0 0" rpy="0 0 0" />
     <axis xyz="1 0 0" />
-    <limit lower="-0.23" upper="0.0" effort="200" velocity="0.2" />
+    <limit lower="-0.17671814671814673" upper="0.0" effort="200" velocity="0.2" />
     <dynamics damping="5" friction="1" />
   </joint>
   <joint name="j_housing_wedge" type="revolute">

--- a/containers/runtime-snapshot/data/models/planning/zed-mini/metadata.json
+++ b/containers/runtime-snapshot/data/models/planning/zed-mini/metadata.json
@@ -9,7 +9,7 @@
   ],
   "q_bounds": [
     [
-      -0.23,
+      -0.17671814671814673,
       0.0
     ],
     [
@@ -183,7 +183,12 @@
     0.020919999999999994
   ],
   "artifacts": {
-    "arm.urdf": "4669046461635af60c61c5f35bc38caa855f4ef7b9a9b1e263fa3636db659349",
+    "arm.urdf": "be1c5d947a9b6992e672fa329a53f418ba8294debfcb528181f06cbbc0d4caa0",
     "robot.yml": "922089a24713c3e53d829a0bcd4b1b7b0706cebf57fd1b237a4a23d4ece24948"
+  },
+  "operating_limit_override": {
+    "linear_u_limit_deg": 199.0,
+    "linear_u_max_deg": 259.0,
+    "note": "Only operating limit narrowed; physical stroke and collision geometry unchanged."
   }
 }

--- a/data/models/planning/zed-mini/arm.urdf
+++ b/data/models/planning/zed-mini/arm.urdf
@@ -124,7 +124,7 @@
     <child link="housing" />
     <origin xyz="0 0 0" rpy="0 0 0" />
     <axis xyz="1 0 0" />
-    <limit lower="-0.23" upper="0.0" effort="200" velocity="0.2" />
+    <limit lower="-0.17671814671814673" upper="0.0" effort="200" velocity="0.2" />
     <dynamics damping="5" friction="1" />
   </joint>
   <joint name="j_housing_wedge" type="revolute">

--- a/data/models/planning/zed-mini/metadata.json
+++ b/data/models/planning/zed-mini/metadata.json
@@ -9,7 +9,7 @@
   ],
   "q_bounds": [
     [
-      -0.23,
+      -0.17671814671814673,
       0.0
     ],
     [
@@ -183,7 +183,12 @@
     0.020919999999999994
   ],
   "artifacts": {
-    "arm.urdf": "4669046461635af60c61c5f35bc38caa855f4ef7b9a9b1e263fa3636db659349",
+    "arm.urdf": "be1c5d947a9b6992e672fa329a53f418ba8294debfcb528181f06cbbc0d4caa0",
     "robot.yml": "922089a24713c3e53d829a0bcd4b1b7b0706cebf57fd1b237a4a23d4ece24948"
+  },
+  "operating_limit_override": {
+    "linear_u_limit_deg": 199.0,
+    "linear_u_max_deg": 259.0,
+    "note": "Only operating limit narrowed; physical stroke and collision geometry unchanged."
   }
 }
```

---

### Native 및 Connections 추가 diff

```diff
--- a/newsim-robot/apps/robot/config/default.yaml
+++ b/newsim-robot/apps/robot/config/default.yaml
@@ -28,7 +28,7 @@
   enclave: ''
 arm:
   linear_u_max_deg: 259.0
-  linear_u_limit_deg: 259.0
+  linear_u_limit_deg: 199.0
   command_direction:
   - 1
   - 1
@@ -56,7 +56,7 @@
   claw_closed_deg: 230.0
 mapping:
   linear_u_max: 259.0
-  linear_u_limit: 259.0
+  linear_u_limit: 199.0
   linear_q_min_m: -0.23
   linear_q_max_m: 0.0
   roll_q_min_rad: -1.5707963267948966
```

```diff
--- a/connections/demo/topology.json
+++ b/connections/demo/topology.json
@@ -121,7 +121,7 @@
             {
               "role": "pilot",
               "endpoint_id": "pilot",
-              "release_key": "dc12afccbcf3d0d6c90280790dca46680a179359650b7638c95dfccc20677d88"
+              "release_key": "0c20db48d1281269ed532215df46fd740787dd41ff37b8ffe27d884530cefabb"
             }
           ],
           "install_mode": "container",
@@ -130,7 +130,7 @@
           "lifecycle": "compose",
           "install_uuid": "7a88a13a-2bfb-47e7-827f-348d5ae2ed1a",
           "project": "elesim-pale",
-          "release_key": "dc12afccbcf3d0d6c90280790dca46680a179359650b7638c95dfccc20677d88"
+          "release_key": "0c20db48d1281269ed532215df46fd740787dd41ff37b8ffe27d884530cefabb"
         },
         {
           "id": "robot-native",
```

검증 상태: [교체 후 snapshot](diagnostics/linear-limit-199-20261028/after-status.log), [적용 결과](diagnostics/linear-limit-199-20261028/result.json).

## cuRobo 수정·Pilot 교체 완료 — 2026-10-28 10:45~10:46 KST

**현재 상태:** Jetson Pilot만 `elesim/pilot:pale-sable`로 교체하여 10:44:58 KST에 시작했다. UI/Sim 컨테이너 ID와 시작 시각, native Robot/Unitree bridge 시작 시각이 전후 동일함을 확인했다. DDS/Pilot online, Robot 제어 대상 선택, 모터 상태 수신, planning idle/can_plan=true를 확인했다. 실제 Execute·모터 이동·Torque 명령은 보내지 않았다. 사용자에게 조작 재개 가능 및 새 좌표는 Plan부터 확인하도록 안내했다.

사용자가 실제 공을 잡는 위치로 확인한 **world XYZ=(0.2976, 0.1682, 0.2901)m**를 Pilot/ UI 기본값 소스에 반영했다. 실행 중 Pilot snapshot에서도 이 3개 값과 roll offset=-9를 확인했다. UI는 재시작하지 않았고 새 Pilot snapshot을 받는다. 다음 UI 시작용 이미지는 `clear-sable`로 준비했다. **공 위치를 Wrap 나무 중심으로 덮어쓰지는 않았다.** Wrap은 기존 radius=0.05m, height=1.1m, xyz=(0.368,0.16,0.55), roll_sign=+1을 유지한다.

### 확인한 실패와 수정

1. 초기 가설이었던 내부 탐색 시간 부족만으로는 설명되지 않았다. 10초 → 120초로 늘린 첫 격리 재현도 12회/52.369초 후 실패했다. Graph fallback에서 Torch 2.11.0/cu126의 `libtorch_cuda_linalg.so`가 `cusolverDnXsyevBatched_bufferSize`를 찾지 못했다. 기존 cuSolver는 11.6.4.69였다. NVIDIA CUDA 12.6 Update 2의 **Jetson linux-aarch64 cuSolver 11.7.1.2**를 Pilot 이미지의 `/opt/elesim/cuda-compat/lib`에 넣고 LD_LIBRARY_PATH 우선순위를 지정했다. 호스트 CUDA/드라이버/커널은 수정하지 않았다. 아카이브 SHA256=`d7e578143ebd90582873cb4c84f0f29ae1b3f086104a7660f5a63a28a115b9e6`, DSO SHA256=`5ee4d6d42e633dc91936776394a001b4aecba67aebb48665413a2107e2886197`. NVIDIA redistrib JSON을 함께 보존했다. GPU SVD/행렬식은 교체 라이브러리와 최종 이미지 각각에서 통과했다. cuRobo 생성 시 작은 GPU SVD를 먼저 호출해 같은 호환성 오류를 수분 초기화 전에 드러내도록 했다.
2. 라이브러리 수정 후 원래 `(0.3,0.2,0.3)` 재현은 graph 사용까지 진행했지만 12회/124.812초 후 TRAJOPT_FAIL이었다. 세부 분석에서 보간 경로 46·47·48 지점의 **self collision**을 확인했다. Bounds/world 비용은 0, self 비용만 양수였다. 최적화의 32개 매듭 사이에서도 여유를 확보하도록 joint-path/finetune의 **soft self-collision cost에만 각 구 반경 여유 2mm**를 더했다. 모델 파일·hard collision 검사·관절/속도/끝점 검증은 유지했다.
3. 그 뒤 cuRobo는 경로를 성공으로 반환했지만 독립 검사는 목표 공 침범(-24.8mm)을 잡았다. 원인은 **`WorldConfig.sphere`를 mesh collision checker에 그대로 전달**한 것이다. 이 경로에서는 sphere가 장애물 검사에 반영되지 않았다. 초기 world, IK target world, joint-path target world를 `get_collision_check_world()`로 변환했다. 같은 팔 내부 점에 놓은 구의 world 충돌 비용이 **변환 전 0 → 변환 후 144.654**로 바뀌는 것을 격리 환경에서 확인했다. 기존 독립 구-구 검사가 최종 실행 전 경로를 거절했으므로 이 진단에서 잘못된 경로를 실행하지 않았다.
4. 사용자 추가 시도 `(0.3,0.15,0.3)`는 교체 전 `pale-pangolin`에서 **36.616초 후 실패**했다. 정확한 snapshot reason은 `robot body/gripper enters the 20 mm target sphere (minimum modeled clearance -14.1 mm; required 2.0 mm)`였다. 이 결과는 “좌표가 도달 불가능하다”는 판정과 다르며, 계획된 경로가 목표 공을 침범했다는 뜻이다.
5. 내부 계획 설정은 max_attempts=12, timeout=120s, enable_graph_attempt=2로 명시했다. 외부 경로 watchdog 180s는 유지했다. 계획 결과에 성공 여부/시도 수/graph·trajopt·finetune 시간/valid_query를 기록하고, Pilot 최종 실패 이유도 로그에 남긴다.

### 검증 범위와 현재 한계

- TDD나 테스트 스위트는 실행하지 않았다. 네트워크 차단·장치 미연결 GPU 컨테이너에서 실패 원인 재현과 수정 검증을 수행했다.
- 최종 소스의 새 프로세스에서 production worker 전체(IK → trajectory → 촘촘한 경로 검사 → CPU/GPU FK)를 검증했다. 진단 목표는 당시 시작점 근처 20mm linear 이동에서 얻은 world `(0.2199407192,-0.1362717671,0.4908116991)`이며 **사용자 시연 추천 좌표가 아니다**. 결과: **141개 waypoint, dt=0.025s, 위치 오차 0.000008259m**, hardware_commands=0. 첫 초기화부터 최종 검사까지 약 213초, path 호출은 42.226초였다. 첫 실제 Plan도 초기화 시간이 필요하다.
- **새 사용자 좌표 `(0.2976,0.1682,0.2901)`의 cuRobo 경로와 실제 Execute는 아직 검증하지 않았다.** 좌표는 사용자가 실제 공을 잡는 위치로 확인했으며 그 값을 반영했다. 기존 `(0.3,0.2,0.3)`을 반드시 성공시키는 작업은 사용자 설명에 따라 더 진행하지 않았다.
- 현재 planning world는 기존대로 명시적 빈 world + 20mm 목표 공 + 자기 충돌 모델이다. 크리스마스 트리 가지/줄기나 Go2 몸체를 새 장애물로 추가한 작업은 아니다. Wrap 정책의 고정 물체 geometry와 IK의 world XYZ는 자동으로 동기화되지 않는다. 시연은 나무의 앞·오른쪽 공을 잡은 뒤 감싸는 순서이며, 공 위치와 나무 중심을 분리해 다뤄야 한다.
- 최초 실패 캡처의 Go2 pos=(0.068849,-0.121827,0.048795)m, rpy=(0.5341,-5.1062,0.2863)deg. 큰 yaw 회전이 원인이라는 근거는 없고 이전 성공 당시 자세 비교 자료도 없다. 임의로 들어 옮겨도 위치가 안 바뀌는 문제는 사용자 요청대로 이번 범위에서 제외했다.

### 적용 상태 / 복구 자료

- 현재 Pilot release: `dc12afccbcf3d0d6c90280790dca46680a179359650b7638c95dfccc20677d88`; image `sha256:a14fe9ed90a7cc8ee19095fd9471aefbc65a10e673b6d7c56acd30af43bf8b33`; 이전 active release `ec8b15efb7a538b2485dfaabf5de28e3602cdeec66915ceee7bb08be7c2f9ea9`. 롤 방향 +1/Wrap roll_sign +1/Extend Arm offset 적용/queued roll offset -9를 계승했다.
- 다음 UI release: `ebb456e3a022cec16f3e47c98c0bcdb93817317b1fa18124e7dda7f6734ffcf0`; image `sha256:eeb5c7bb7f4f80d25ab685cb2987aa1d177cb385cbe2b3108dc0411b1cef98dc` (`clear-sable`). 현재 UI는 기존 `clear-lynx` 그대로다. Sim은 com2 `dawn-otter` 그대로다.
- `connections/demo/topology.json`의 다음 Pilot/UI release를 갱신했다. Jetson instance 등록 전 `registration-backup/`에 state/compose/ownership을 보존했다. 동일 diagnostics 경로가 Jetson에도 있다.
- Jetson 용량이 빌드 후 183MiB까지 감소해 **이번 작업의 staging 라이브러리와 해당 build-context cache ID 하나만** 제거했다. 최종 이미지와 노트북 원본 아카이브/DSO는 보존했고 최종 여유는 약 526MiB다. 다른 이미지·서비스·사용자 파일은 정리하지 않았다. Jetson의 삭제된 staging DSO 경로는 재현 시 로컬 원본에서 복원하거나 최종 `pale-sable` 이미지를 사용해야 한다.

근거: [최종 검증](diagnostics/curobo-path-fix-20261028/validation.json), [전체 계산 로그](diagnostics/curobo-path-fix-20261028/final-offline-plan.log), [사용자 .3/.15/.3 실패](diagnostics/curobo-path-fix-20261028/user-0301503-status.log), [공 장애물 검사](diagnostics/curobo-path-fix-20261028/obstacle-check.json), [교체 후 상태/좌표](diagnostics/curobo-path-fix-20261028/final-state.log), [Pilot 발행 결과](diagnostics/curobo-path-fix-20261028/release.json), [UI 발행 결과](diagnostics/curobo-path-fix-20261028/ui-release.json). 아래는 실제 설치 트리에 가한 정확한 diff다. 원본 파일과 실행 스크립트도 같은 진단 폴더에 보존했다. 캐시의 원격 Git 원본이나 원격 저장소에는 커밋하지 않았다.

#### Pilot planning/실패 로그

```diff
--- a/containers/build/pilot/app/elesim_pilot/robot/arm/curobo.py
+++ b/containers/build/pilot/app/elesim_pilot/robot/arm/curobo.py
@@ -5,13 +5,14 @@
 """
 from __future__ import annotations
 
-from dataclasses import dataclass, fields
+from dataclasses import dataclass, fields, replace
 import copy
 import importlib.metadata
 import hashlib
 import json
 from pathlib import Path
 import threading
+import time
 
 import numpy as np
 import yaml
@@ -87,6 +88,15 @@
         import torch
         if not torch.cuda.is_available():
             raise CuRoboError("cuRobo requires an NVIDIA CUDA GPU")
+        # Graph fallback lazily loads libtorch_cuda_linalg. Detect a mismatched
+        # cuSolver before spending minutes initializing collision/IK models.
+        try:
+            torch.svd(torch.eye(4, device="cuda"), compute_uv=True, some=False)
+        except RuntimeError as exc:
+            raise CuRoboError(
+                "cuRobo CUDA linear algebra unavailable; check PyTorch/cuSolver "
+                f"compatibility: {exc}"
+            ) from exc
         from curobo.types.base import TensorDeviceType
         from curobo.geom.types import WorldConfig
         from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
@@ -101,16 +111,30 @@
         if not 0 < sphere_count < 1024:
             raise CuRoboError("cuRobo v0.7.8 requires 1..1023 collision spheres")
         self.world = world
-        self._world_config = WorldConfig.from_dict(world)
+        self._world_config = WorldConfig.from_dict(world).get_collision_check_world()
         self._ik_tolerance = None
         self.use_cuda_graph = bool(use_cuda_graph)
         self._lock = threading.Lock()
         self.motion = MotionGen(MotionGenConfig.load_from_robot_config(
-            raw, world, tensor_args=self.tensor_args, use_cuda_graph=self.use_cuda_graph,
+            raw, self._world_config, tensor_args=self.tensor_args, use_cuda_graph=self.use_cuda_graph,
             collision_cache={"obb": 32, "mesh": 16},
             cspace_threshold=0.0001,
             interpolation_dt=float(interpolation_dt),
         ))
+        # The optimizer samples 32 knots; a zero-clearance self-collision
+        # optimum can collide between knots. Give its soft cost 2 mm extra
+        # radius per sphere. Keep the model and hard validators unchanged.
+        seen_costs = set()
+        for solver in (self.motion.js_trajopt_solver,
+                       self.motion.finetune_js_trajopt_solver):
+            for rollout in solver.get_all_rollout_instances():
+                cost = getattr(rollout, "robot_self_collision_cost", None)
+                if cost is None or id(cost) in seen_costs:
+                    continue
+                seen_costs.add(id(cost))
+                config = cost.self_collision_kin_config
+                cost.self_collision_kin_config = replace(
+                    config, offset=config.offset.clone() + 0.002)
         self._order = self._joint_order(self.motion.kinematics.joint_names)
         if not self.motion.kinematics.get_self_collision_config().experimental_kernel:
             raise CuRoboError("parallel self-collision kernel required; reinstall the pinned patched cuRobo")
@@ -218,21 +242,45 @@
             if target_sphere_center is not None:
                 target_world = WorldConfig.from_dict(
                     _world_with_target_sphere(self.world, target_sphere_center)
-                )
+                ).get_collision_check_world()
             try:
                 if target_world is not None:
                     self.motion.update_world(target_world)
                 # Both endpoints were independently checked above. The legacy
                 # duplicate check can include optimizer state/derivative costs.
-                result = self.motion.plan_single_js(start_state, goal_state,
-                                                   MotionGenPlanConfig(max_attempts=4, check_start_validity=False))
+                # v0.7.8 otherwise defaults to a 10 s query budget, which
+                # includes first-query CUDA setup and can expire before the
+                # graph-seeded retries. Leave room inside the worker's 180 s
+                # stage watchdog for the independent path checks below.
+                started = time.monotonic()
+                result = self.motion.plan_single_js(
+                    start_state, goal_state,
+                    MotionGenPlanConfig(max_attempts=12, timeout=120.0,
+                                        enable_graph_attempt=2,
+                                        check_start_validity=False),
+                )
+                elapsed = time.monotonic() - started
             finally:
                 self.motion.rollout_fn.primitive_collision_constraint.enable_cost()
                 self.motion.rollout_fn.robot_self_collision_constraint.enable_cost()
                 if target_world is not None:
                     self.motion.update_world(self._world_config)
-        if not bool(result.success.item()):
-            raise CuRoboError(f"cuRobo planning failed: {result.status}")
+        details = {
+            "status": str(result.status),
+            "success": bool(result.success.item()),
+            # v0.7.8 records the zero-based last attempt index.
+            "attempts": int(result.attempts) + 1,
+            "valid_query": bool(result.valid_query),
+            "used_graph": bool(result.used_graph),
+            "wall_s": round(elapsed, 3),
+            "graph_s": round(float(result.graph_time), 3),
+            "trajopt_s": round(float(result.trajopt_time), 3),
+            "finetune_s": round(float(result.finetune_time), 3),
+            "budget_s": 120.0,
+        }
+        print("[curobo-path] " + json.dumps(details, sort_keys=True), flush=True)
+        if not details["success"]:
+            raise CuRoboError("cuRobo planning failed: " + json.dumps(details, sort_keys=True))
         plan = result.get_interpolated_plan()
         actual_names = list(plan.joint_names)
         if set(actual_names) != set(self.joint_names):
@@ -296,7 +344,7 @@
                 if progress is not None:
                     progress("Initializing CUDA position IK solver")
                 self._ik = IKSolver(IKSolverConfig.load_from_robot_config(
-                    self.robot_config, self.world, tensor_args=self.tensor_args,
+                    self.robot_config, self._world_config, tensor_args=self.tensor_args,
                     use_cuda_graph=self.use_cuda_graph, num_seeds=32, position_threshold=tolerance_m,
                     collision_cache={"obb": 32, "mesh": 16},
                     high_precision=tolerance_m < 0.001,
@@ -310,7 +358,7 @@
             if progress is not None:
                 progress("Solving position IK")
             target_world = None if target_sphere_center is None else WorldConfig.from_dict(
-                _world_with_target_sphere(self.world, target_sphere_center))
+                _world_with_target_sphere(self.world, target_sphere_center)).get_collision_check_world()
             try:
                 if target_world is not None:
                     self._ik.update_world(target_world)
--- a/containers/build/pilot/app/elesim_pilot/arm_trajectory.py
+++ b/containers/build/pilot/app/elesim_pilot/arm_trajectory.py
@@ -699,6 +699,7 @@
             if isinstance(result[1], str):
                 self._phase, self._reason = "failed", result[1]
                 self._stage = "failed"
+                print(f"[curobo-plan] failed: {self._reason}", flush=True)
             else:
                 if len(result[1]) == 3:
                     positions, dt, self._planned_xyz = result[1]
```

#### Pilot CUDA 빌드

```diff
--- a/containers/build/pilot/install_jetson_cuda.sh
+++ b/containers/build/pilot/install_jetson_cuda.sh
@@ -27,6 +27,21 @@
   libcudnn9-cuda-12=9.3.0.75-1 libopenblas0-pthread libjpeg-turbo8 libpng16-16
 rm -rf /var/lib/apt/lists/*
 
+# Torch 2.11 cu126 lazily loads CUDA linalg for cuRobo graph search. The
+# r36.5 CUDA 12.6 GA cuSolver lacks cusolverDnXsyevBatched_bufferSize.
+# Use NVIDIA's CUDA 12.6 Update 2 Jetson library; keep host drivers unchanged.
+curl -fL --retry 3 \
+  https://developer.download.nvidia.com/compute/cuda/redist/libcusolver/linux-aarch64/libcusolver-linux-aarch64-11.7.1.2-archive.tar.xz \
+  -o "$temporary/cusolver.tar.xz"
+printf '%s  %s\n' d7e578143ebd90582873cb4c84f0f29ae1b3f086104a7660f5a63a28a115b9e6 \
+  "$temporary/cusolver.tar.xz" | sha256sum -c -
+mkdir -p /opt/elesim/cuda-compat/lib
+tar -xJf "$temporary/cusolver.tar.xz" --strip-components=2 \
+  -C /opt/elesim/cuda-compat/lib \
+  libcusolver-linux-aarch64-11.7.1.2-archive/lib/libcusolver.so.11.7.1.2
+ln -sf libcusolver.so.11.7.1.2 /opt/elesim/cuda-compat/lib/libcusolver.so.11
+printf '%s\n' /opt/elesim/cuda-compat/lib > /etc/ld.so.conf.d/00-elesim-cusolver.conf
+
 # This Jetson PyTorch build links cuDSS. Use NVIDIA's aarch64/CUDA12 archive,
 # not the separately published SBSA or CUDA13 binaries.
 curl -fL --retry 3 \
--- a/containers/build/pilot/Dockerfile
+++ b/containers/build/pilot/Dockerfile
@@ -9,7 +9,7 @@
     HOME=/var/lib/elesim \
     PATH=/opt/openrobots/bin:$PATH \
     PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig \
-    LD_LIBRARY_PATH=/opt/acados/lib:/opt/openrobots/lib \
+    LD_LIBRARY_PATH=/opt/elesim/cuda-compat/lib:/opt/acados/lib:/opt/openrobots/lib \
     ACADOS_SOURCE_DIR=/opt/acados \
     PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages \
     CMAKE_PREFIX_PATH=/opt/openrobots
```

#### 공 좌표 기본값

```diff
--- a/containers/build/pilot/app/elesim_pilot/pick/state.py
+++ b/containers/build/pilot/app/elesim_pilot/pick/state.py
@@ -155,9 +155,9 @@
     pick_phase: str = "idle"
     pick_status_msg: str = ""
 
-    target_x: float = 0.3
-    target_y: float = 0.2
-    target_z: float = 0.3
+    target_x: float = 0.2976
+    target_y: float = 0.1682
+    target_z: float = 0.2901
     target_vx: float = 1.0
     target_vy: float = 0.0
     target_vz: float = 0.0
--- a/containers/build/ui/app/elesim_ui/models.py
+++ b/containers/build/ui/app/elesim_ui/models.py
@@ -134,9 +134,9 @@
     pick_phase: str = "idle"
     pick_status_msg: str = ""
 
-    target_x: float = 0.3
-    target_y: float = 0.2
-    target_z: float = 0.3
+    target_x: float = 0.2976
+    target_y: float = 0.1682
+    target_z: float = 0.2901
     target_vx: float = 1.0
     target_vy: float = 0.0
     target_vz: float = 0.0
```

#### Connections 선택 release

```diff
--- a/connections/demo/topology.json
+++ b/connections/demo/topology.json
@@ -25,7 +25,7 @@
             {
               "role": "ui",
               "endpoint_id": "ui",
-              "release_key": "02cbf08d01d2b6acc3dfd1e0c37c0b1fcecff5e18e0d653e5bf0bad5eeac60d6"
+              "release_key": "ebb456e3a022cec16f3e47c98c0bcdb93817317b1fa18124e7dda7f6734ffcf0"
             }
           ],
           "install_mode": "container",
@@ -34,14 +34,14 @@
           "lifecycle": "compose",
           "install_uuid": "1f95fd4e-2810-4f97-8a29-81257e8074db",
           "project": "elesim-clear",
-          "release_key": "02cbf08d01d2b6acc3dfd1e0c37c0b1fcecff5e18e0d653e5bf0bad5eeac60d6"
+          "release_key": "ebb456e3a022cec16f3e47c98c0bcdb93817317b1fa18124e7dda7f6734ffcf0"
         }
       ],
       "assignments": [
         {
           "role": "ui",
           "endpoint_id": "ui",
-          "release_key": "02cbf08d01d2b6acc3dfd1e0c37c0b1fcecff5e18e0d653e5bf0bad5eeac60d6"
+          "release_key": "ebb456e3a022cec16f3e47c98c0bcdb93817317b1fa18124e7dda7f6734ffcf0"
         }
       ],
       "install_mode": "container",
@@ -121,7 +121,7 @@
             {
               "role": "pilot",
               "endpoint_id": "pilot",
-              "release_key": "0ecb919793d0cb5338217651a155cef0f805130cbec751b4ef816c0a9757047e"
+              "release_key": "dc12afccbcf3d0d6c90280790dca46680a179359650b7638c95dfccc20677d88"
             }
           ],
           "install_mode": "container",
@@ -130,7 +130,7 @@
           "lifecycle": "compose",
           "install_uuid": "7a88a13a-2bfb-47e7-827f-348d5ae2ed1a",
           "project": "elesim-pale",
-          "release_key": "0ecb919793d0cb5338217651a155cef0f805130cbec751b4ef816c0a9757047e"
+          "release_key": "dc12afccbcf3d0d6c90280790dca46680a179359650b7638c95dfccc20677d88"
         },
         {
           "id": "robot-native",
```

---

## 이력: 수정 전 cuRobo 실행 진단 — 2026-10-28 10:15~10:18 KST

사용자가 Plan XYZ 실행 실패 원인을 확인하도록 요청했다. 읽기 전용 로그 및 UI snapshot 조회만 수행했으며 계획 재요청·Execute·Torque·모터 명령·재시작·제품 코드/설정 변경은 하지 않았다. 아래는 처음 cuRobo 도입 시의 모델 로딩 검증과 구분되는 실제 사용자 Plan XYZ 시도의 결과다.

**확정 결과:** `phase=failed`, `reason=cuRobo planning failed: MotionGenStatus.TRAJOPT_FAIL`. 실제 목표는 `[0.3000000119, 0.2000000030, 0.3000000119]`로 요청한 좌표가 전달됐다. CUDA 모델 생성과 position IK를 통과한 뒤 joint path trajectory optimization 단계에서 실패했다. 경로는 0개, plan_id 없음, 실제 Execute 단계에 진입하지 않았다. 최종 상태 elapsed는 약 178.800초다.

KST 타임라인:

- 10:11:29.749: `plan_arm_to_target` 요청 접수.
- 10:11:30.673: CUDA/collision model 로딩 시작.
- 10:12:42.840: position IK 단계 진입(첫 모델 로딩 약 72.17초).
- 10:12:56.295: CUDA position IK solver 초기화 시작.
- 10:13:59.003: IK 풀이 시작(솔버 초기화 약 62.71초).
- 10:14:17.860: collision-checked joint path 단계 진입(구한 IK 목표 및 시작점 검사까지 통과한 코드 경로).
- 요청 시각과 status elapsed를 합산하면 실패 종료는 약 10:14:28.55, 즉 경로 단계 약 10.69초 후로 추정된다. 최종 실패 자체의 별도 timestamp 로그는 현재 코드에 없으므로 이 종료 시각은 계산값이다.
- 10:15대 실제 snapshot에서 failed/TRAJOPT_FAIL을 확인했다. 마지막 progress 로그만 보고 아직 계산 중이라고 했던 초기 추정은 최종 snapshot으로 정정했다.

### 발견한 제한과 아직 모르는 것

EleSim `robot/arm/curobo.py::plan_joint()`는 `MotionGenPlanConfig(max_attempts=4, check_start_validity=False)`만 지정하고 timeout은 넘기지 않는다. 실제 설치된 pinned cuRobo v0.7.8의 기본 timeout은 **10.0초**, enable_graph는 False, graph fallback은 세 번 실패 후 다음 시도부터 활성화된다. 반면 바깥 `arm_trajectory.py`의 경로 단계 watchdog은 **180초**다. 바깥에서 기다릴 수 있는 시간과 내부 탐색 시간은 다르다. 내부 루프는 매 시도 후 10초 초과 여부를 검사하여 빠져나온다.

관측한 약 10.69초 경로 단계와 내부 10초 제한이 일치하므로 **탐색 시간 부족, 특히 첫 실행 계산/초기화 때문에 fallback까지 충분히 시도하지 못했을 가능성이 유력하다.** 다만 결과 객체의 attempts/graph_time/constraint 비용을 현재 어댑터가 남기지 않아 실제 몇 번 시도했는지, timeout 때문에 남은 시도가 생략됐는지는 확정할 수 없다. TRAJOPT_FAIL만으로 “실제 충돌했다”, “목표가 도달 불가능하다”, “시간을 늘리면 반드시 된다”고 결론 내리지 않는다. 자기 충돌/20mm 대상 구체/동역학 제약/최적화 seed 등에 따른 수렴 실패를 더 구분해야 한다.

시작과 IK 목표의 정적 충돌/관절 검사 및 목표 위치 정밀도 검사는 경로 단계 전 수행된다. 따라서 시작/끝 점이 유효한 것과 그 사이의 충돌 없는 실행 경로를 찾는 것은 별개다. Sparse invariant warning은 모델/IK 단계 이후에도 진행했으므로 이번 직접 실패 예외로 취급하지 않는다. UI의 이전 통신 timeout 로그에는 10:07 계획된 Pilot/Robot 재시작 구간도 섞여 있어 이번 경로 실패의 원인으로 사용하지 않았다.

### 후속 수정 위치 / 근거

- `containers/build/pilot/app/elesim_pilot/robot/arm/curobo.py::plan_joint`: 내부 timeout을 명시하고 외부 watchdog과 정합시키며, 실패 시 attempts/graph_time/trajopt_time/valid_query 등 결과 요약을 남기는 것이 후속 수정 지점이다. 충돌 검사·목표 검증을 완화하거나 검증 없는 보간 경로로 대체해서는 안 된다.
- `containers/build/pilot/app/elesim_pilot/arm_trajectory.py`: 최종 planning 실패 이유/시각을 Docker 로그에도 남기면 마지막 progress만으로 진행 중 여부를 오판하는 문제를 줄일 수 있다.
- 이 진단에서는 위 제품 코드 수정은 하지 않았다. 실행 중 이미지 및 준비된 다음 시작 release도 그대로다.
- [최종 실패 snapshot](diagnostics/curobo-plan-20261028/failure.json), [상태 관측 전체](diagnostics/curobo-plan-20261028/status.log), [시각 로그와 실제 cuRobo 소스 발췌](diagnostics/curobo-plan-20261028/timeline-and-library.txt), [읽기 전용 observer](diagnostics/curobo-plan-20261028/read-status.py).

---


## 최신 기본값 변경 — 2026-10-28 10:13 KST: IK Target XYZ (0.3, 0.2, 0.3)

사용자 요청대로 IK 목표 위치 기본값을 X=0.3, Y=0.2, Z=0.3m로 바꿨다. Pilot `pick/state.py::PanelState`와 UI `models.py` 양쪽의 target_x/y/z 필드 3개씩만 변경했다. 방향 벡터, mock object 위치 및 Sim target object는 변경하지 않았다. Pilot의 시작 코드가 PanelState를 생성하고 UI가 Pilot snapshot을 받으므로 두 기본값을 함께 맞췄다.

**화면 refresh·UI/Pilot 재시작·active instance 등록·현재 IK 값 주입은 하지 않았다.** 새 이미지와 불변 release를 준비해 Connections의 다음 시작 선택만 갱신했다. 컨테이너 ID·StartedAt와 active instance state가 작업 전후 동일한 것을 확인했다. 다음 Connections 시작에서 새 기본값이 적용되며 현재 입력칸은 기존 값을 유지한다.

Pilot은 직전 준비한 roll offset -9 release를 기반으로 만들어 **롤 오프셋 -9를 빠뜨리지 않았다.** motor_direction +1, Wrap roll_sign +1, Extend Arm apply_offset=True, cuRobo 복구도 계승한다.

- 다음 ui: `elesim/ui:clear-mink`, release `02cbf08d01d2b6acc3dfd1e0c37c0b1fcecff5e18e0d653e5bf0bad5eeac60d6`, image `sha256:016c1889e28b1d2d4532b9b2d0d6c10fe43bb15cbe14c88609484f01b470d07d`.
- 다음 pilot: `elesim/pilot:pale-mink`, release `0ecb919793d0cb5338217651a155cef0f805130cbec751b4ef816c0a9757047e`, image `sha256:4b000cc0fc9b14b6189f9cd1cd236002657c814822057ddbd4d82d582625ae28`.
- Sim은 com2 `dawn-otter` 그대로다. 현재 실행 중인 UI/Pilot 이미지에는 변경을 가하지 않았다.

[정확한 코드 diff](diagnostics/ik-default-20261028/changes.patch), [다음 시작 topology diff](diagnostics/ik-default-20261028/topology.patch), [발행 스크립트](diagnostics/ik-default-20261028/publish.py), [UI 결과](diagnostics/ik-default-20261028/ui/result.json), [Pilot 결과](diagnostics/ik-default-20261028/pilot/result.json). 원본은 같은 폴더 state.before.py/models.before.py 및 topology.before.json에 보존했다. 각 ui/pilot 하위 폴더 runtime-before.txt와 runtime-after.txt에 무재시작 근거를 저장했다. 변경 소스 Python 문법 확인과 이미지 빌드만 수행했고 실제 IK 풀이/모터 제어/TDD는 수행하지 않았다.

```diff
--- a/containers/build/ui/app/elesim_ui/models.py
+++ b/containers/build/ui/app/elesim_ui/models.py
@@ -134,9 +134,9 @@
     pick_phase: str = "idle"
     pick_status_msg: str = ""
 
-    target_x: float = 0.5
-    target_y: float = 0.0
-    target_z: float = 1.0
+    target_x: float = 0.3
+    target_y: float = 0.2
+    target_z: float = 0.3
     target_vx: float = 1.0
     target_vy: float = 0.0
     target_vz: float = 0.0
--- a/containers/build/pilot/app/elesim_pilot/pick/state.py
+++ b/containers/build/pilot/app/elesim_pilot/pick/state.py
@@ -155,9 +155,9 @@
     pick_phase: str = "idle"
     pick_status_msg: str = ""
 
-    target_x: float = 0.50
-    target_y: float = 0.00
-    target_z: float = 1.00
+    target_x: float = 0.3
+    target_y: float = 0.2
+    target_z: float = 0.3
     target_vx: float = 1.0
     target_vy: float = 0.0
     target_vz: float = 0.0
```

---


## 최신 설정 정정 — 2026-10-28 10:11 KST: Roll offset 기본값 -9, 다음 시작 적용

사용자 요청: “롤 오프셋 기본값 다시 -9 … 런타임 바꿔치기는 안해도 됨.” 이에 따라 PC/Jetson 두 프로필의 `u_offset_roll: 9.0 → -9.0`을 로컬과 com3 `apps/pilot/config/config.yaml` 및 새 release 데이터에 반영했다. `containers/runtime-snapshot/config/pilot/config.yaml`은 두 프로필 모두 이미 -9.0이어서 그대로 확인했다. 공통 fallback 0.0은 기존대로 유지했다.

**실행 중 Pilot의 instance/config/프로세스에는 변경을 가하지 않았다.** 컨테이너 ID·상태·StartedAt와 instance state가 작업 전후 동일함을 확인했다. 현재 실행 release는 `ec8b15efb7a538b2485dfaabf5de28e3602cdeec66915ceee7bb08be7c2f9ea9`이며, Connections에 저장한 다음 시작 release는 `a52417c9305d480462464e2603bd49293b7409700a32a5cc8398f47143c588eb`다. **Connections를 새로 열어 다음 시작할 때 -9 기본값이 적용된다.** 기존 컨테이너를 단순 재시작하는 것과 Connections의 새 release 등록은 다르다.

태그는 `pale-pangolin` 유지. 앞서 적용한 motor_direction Roll +1 / Wrap roll_sign +1 / Extend Arm apply_offset=True / cuRobo 설정은 그대로 계승한다. 이 작업은 롤 오프셋 기본값 두 개만 바꾸며 실제 모터 명령이나 서비스 재시작은 없다.

- [정확한 설정 diff](diagnostics/roll-offset-minus9-20261028/changes.patch)
- [Connections 다음 시작 release diff](diagnostics/roll-offset-minus9-20261028/topology.patch)
- [변경 결과](diagnostics/roll-offset-minus9-20261028/result.json), [변경 전 런타임](diagnostics/roll-offset-minus9-20261028/runtime-before.txt), [변경 후 런타임](diagnostics/roll-offset-minus9-20261028/runtime-after.txt)
- [수정/발행 스크립트](diagnostics/roll-offset-minus9-20261028/update.py). 이전 설정과 topology는 같은 diagnostics 디렉터리의 backup 및 topology.before.json에 보존했다.

```diff
--- a/apps/pilot/config/config.yaml
+++ b/apps/pilot/config/config.yaml
@@ -12,7 +12,7 @@
     robot:
       arm:
         hardware:
-          u_offset_roll: 9.0
+          u_offset_roll: -9.0
       go2:
         spawn:
           mount_offset_m:
@@ -77,7 +77,7 @@
           profile_acc_seg2: 20
           profile_vel_seg1: 150
           profile_vel_seg2: 150
-          u_offset_roll: 9.0
+          u_offset_roll: -9.0
       go2:
         spawn:
           mount_offset_m:

--- a/release-data/config/pilot/config.yaml
+++ b/release-data/config/pilot/config.yaml
@@ -12,7 +12,7 @@
     robot:
       arm:
         hardware:
-          u_offset_roll: 9.0
+          u_offset_roll: -9.0
       go2:
         spawn:
           mount_offset_m:
@@ -77,7 +77,7 @@
           profile_acc_seg2: 20
           profile_vel_seg1: 150
           profile_vel_seg2: 150
-          u_offset_roll: 9.0
+          u_offset_roll: -9.0
       go2:
         spawn:
           mount_offset_m:
```

---


## 최신 추가 수정 — 2026-10-28 10:08 KST: 일반 Roll 방향 복구

**현재 적용된 값은 Robot Roll motor_direction=+1, Wrap roll_sign=+1이다. 아래 이전 기록의 Wrap -1은 이 수정으로 대체됐다.** 사용자가 “UI에서 Roll 값을 올릴 때 실제 모터가 반대로 회전함”이라고 확인한 뒤 수정했다. Pilot/Robot 재기동까지 완료했으며 UI snapshot에서 Torque OFF·상태 갱신·빈 safety_fault를 확인했다. 이전 “전체 종료” 절은 그때의 이력이고, 현재 UI/Sim/Pilot/Robot은 다시 실행 중이다. Connections 관리자는 수정 중 캐시 덮어쓰기를 막기 위해 종료했다.

### 원인과 적용 범위

Wrap 전용 `roll_sign`이 일반 Roll 조작에 전파되는 경로는 없었다. 일반 조작은 control_u→canonical q→Robot 모터 변환을 거친다. 현재 Robot `motor_direction[1]`은 -1이었으나, 예전 Jetson 설치와 oct27의 실제 설치 YAML에는 +1이었다. 이 차이는 앞선 Wrap 반전 작업 전부터 존재했다. Wrap 설정만으로 일반 Roll 극성까지 고쳐졌다고 본 것이 누락이었다.

- native Robot `apps/robot/config/default.yaml`의 `arm.motor_direction`: `[1,-1,1,-1] → [1,1,1,-1]`. `elesim-net configure`로 installed.yaml을 재생성했다. 이 값은 C++의 목표 tick 변환과 실측 tick 역변환 양쪽에 적용된다.
- Pilot config의 `robot.arm.hardware.motor_direction`도 같은 값으로 맞췄다. 일반 `command_direction`은 `[1,1,1,-1]`로 유지했다.
- Wrap `roll_sign`: `-1.0 → +1.0`. 모터 극성을 바꿀 때 Wrap 부호도 함께 바꿔 정책 입출력과 물리 모터 사이의 기존 방향을 유지했다. 정책 파일·Lift 동작·타이밍은 변경하지 않았다.
- Roll offset +9, Extend Arm의 Home와 같은 apply_offset=True, cuRobo 패키지/모델, GO2 domain0·EleSim domain1, 서버 Sim은 그대로 계승한다.
- 이 변경은 Robot Roll 좌표 변환 전체에 적용되므로 수동 Roll뿐 아니라 같은 canonical q를 사용하는 Home/IK의 물리 Roll 매핑도 예전 극성으로 돌아간다. +9 offset을 별도로 반전시키지 않았다. 따라서 자동 동작이나 Home의 실제 자세가 검증됐다고 간주하지 않는다.

### 부호 관계와 검증 범위

중심 180° 기준 일반 Roll의 물리 tick 기울기는 motor_direction에 의해 -에서 +로 바뀐다. Wrap의 물리 방향은 `motor_direction × roll_sign`이므로 변경 전 `(-1)×(-1)=+1`, 변경 후 `(+1)×(+1)=+1`이다. 실측을 정책 좌표로 돌리는 경로에서도 두 부호가 함께 바뀐다. 이는 코드의 대칭 범위/변환에 대한 정적 분석이며 실제 grasp 성공 검증은 아니다.

실제 팔이 Torque ON 상태임을 read-only UI snapshot에서 확인했기 때문에, 수정본을 먼저 준비한 뒤 사용자에게 팔/물체 지지 및 Torque OFF를 요청했다. 사용자가 “팔을 지지했고 Torque OFF 완료”라고 답한 후에만 Pilot/Robot을 정지하고 두 설정을 함께 적용했다. Robot 재시작은 토크를 해제하므로 부하를 든 상태에서 임의로 재시작하지 않았다.

재기동 후 `after-snapshot.log`에서 Torque false, safety_fault 빈 문자열, host state age 약 0.015초를 확인했다. Roll raw tick 1931 및 canonical roll 약 -0.08938rad가 수신됐다. 모터 동작 명령·Wrap 실행·TDD/테스트 스위트는 수행하지 않았다. 최종 물리 회전 방향은 사용자가 소폭 조작으로 확인해야 한다.

### 배포 및 정확한 변경 기록

이미지 태그는 **elesim/pilot:pale-pangolin** 그대로이며 cuRobo 이미지 ID도 같다. 코드 이미지 재빌드 없이 새 config/data release를 발행했다. 현재 release는 `ec8b15efb7a538b2485dfaabf5de28e3602cdeec66915ceee7bb08be7c2f9ea9`이다. Connections topology, 설치 instance, source 앱 config와 runtime snapshot까지 반영했다. 이전 source에 빠져 있던 wrap_grasp.yaml도 현재 설정으로 추가해 source snapshot에서 유실되지 않도록 했다.

- [실제 설정 diff](diagnostics/roll-polarity-20261028/changes.patch)
- [source/snapshot 전체 diff](diagnostics/roll-polarity-20261028/templates.patch)
- [Connections diff](diagnostics/roll-polarity-20261028/topology.patch)
- [배포 결과](diagnostics/roll-polarity-20261028/result.json), [배포 설정 조회](diagnostics/roll-polarity-20261028/deployed-settings.log)
- [변경 전 실제 상태](diagnostics/roll-polarity-20261028/before-snapshot-full.log), [재기동 후 실제 상태](diagnostics/roll-polarity-20261028/after-snapshot.log)
- [준비 스크립트](diagnostics/roll-polarity-20261028/prepare.py), [적용 스크립트](diagnostics/roll-polarity-20261028/apply.py), [source 동기화](diagnostics/roll-polarity-20261028/sync_templates.py)

원복: com3 같은 diagnostics 디렉터리의 apply-backup에 이전 native default/installed/install-state 및 Pilot instance/Compose/ownership을 보존했다. native 극성과 Pilot release를 반드시 한 쌍으로 되돌리고 Connections도 이전 release로 복구해야 한다. 한쪽 부호만 되돌리면 Wrap 물리 방향이 다시 바뀐다. 원래 모터 방향 기본값을 모든 기계에 일괄 변경한 것은 아니며 현재 설치의 명시적 설정을 수정했다.

```diff
--- a/pilot/config/config.yaml
+++ b/pilot/config/config.yaml
@@ -232,7 +232,7 @@
       lji_velocity_mode_enable: false
       motor_direction:
       - 1
-      - -1
+      - 1
       - 1
       - -1
       profile_acc_claw: 5

--- a/pilot/config/wrap_grasp.yaml
+++ b/pilot/config/wrap_grasp.yaml
@@ -5,4 +5,4 @@
   height_m: 1.1
   xyz_m: [0.368, 0.16, 0.55]
   lean: [0.0, 0.0]
-roll_sign: -1.0
+roll_sign: 1.0

--- a/native/apps/robot/config/default.yaml
+++ b/native/apps/robot/config/default.yaml
@@ -36,7 +36,7 @@
   - -1
   motor_direction:
   - 1
-  - -1
+  - 1
   - 1
   - -1
   baudrate: 1000000
```

---


## 이력: 당시 실행 상태 — 2026-10-28 09:48 KST 전체 종료

사용자의 전체 종료 요청으로 com1 Connections 관리자/UI, com2 demo Sim, com3 demo Pilot 컨테이너 및 native elesim-pilot·elesim-robot·elesim-unitree-bridge 서비스를 모두 정지했다. 이전 native Pilot도 실행 중인 것을 발견하여 함께 정지했다. com1 실행 컨테이너 없음, com3 실행 컨테이너 없음 및 세 서비스 inactive를 확인했다. 배포 이미지·토폴로지·설정은 보존했다. 서버의 별도 Doosan 작업 컨테이너 emulator/doosan_kos와 Wi-Fi 절전 해제 설정은 종료 대상에 포함하지 않았다. 아래 배포 검증 기록은 종료 전 시점의 결과다.

## 최신 인계 — 2026-10-28: 서버 Sim 복귀·GO2 수신·cuRobo 도입 완료

**이 절이 현재 상태다. 뒤의 “com2 제거”, “GO2 미수정”, “pale-lynx 다음 시작 적용”은 이후 변경으로 대체된 이력이다.**

| 호스트 | 현재 역할 | Connections 선택 이미지 |
|---|---|---|
| com1 / 100.82.201.62 | UI만 | `elesim/ui:clear-lynx` |
| com2 / 100.74.222.24 | Sim, RTX A6000 GPU 2 | `elesim/sim:dawn-otter` |
| com3 / 100.88.154.118 | Pilot + native Robot/bridge | **`elesim/pilot:pale-pangolin`** + native |

- Sim을 서버로 복귀시키고 세 호스트 instance와 Connections topology를 저장·기동했다. 서버의 두 카메라 첫 프레임과 scene/media readiness gate 로그를 확인했다. 노트북 Sim은 정지한 채 보존했다.
- EleSim DDS는 **domain 1 / tailscale0 / 세 Tailscale peer**, Unitree GO2는 **domain 0 / eno1**이다. 실제 UI snapshot에서 몸체 상태와 다리 12개 q/dq/torque 갱신을 확인했다. 이후 사용자가 로봇 전원을 껐으며 마지막 cuRobo 도입 단계에서 로봇을 다시 켜거나 움직이지 않았다.
- Extend Arm은 Home과 같은 **`apply_offset=True`** 경로로 배포했다. `u_roll=180.0`을 유지하고 수동으로 9를 더하지 않는다. `pale-marten`에서 반영했고 현재 `pale-pangolin`이 계승한다. 잘못된 `pale-lynx` 중간안은 실행하지 않고 철회했다.
- Wrap 좌우 반전은 **`roll_sign: -1.0`**으로 반영·Pilot 재기동했다. 실제 grasp는 실행하지 않았다.
- **cuRobo 0.7.8+elesim.1 도입 완료.** main과 oct27 모두 연동 코드가 있었다. 누락된 패키지·230mm 모델·world 설정을 복구했다. Orin/CUDA12.6에서 모델 로딩과 GPU FK 계산을 확인했다. 실제 XYZ 계획 및 경로 실행 시험은 하지 않았다.
- Jetson Wi-Fi는 **H3C_D41BD2_5G**, 절전 off. 통신은 사용자의 최종 선택대로 **Tailscale 유지**. Connections를 새로 열면 저장된 세 호스트 구성을 사용한다.

현재 Pilot release: `f93544333dfa23d342441813534d38d6b06498c7decd95cd33696656e8e7ed99`, image: `sha256:4cf22db9679e16e8629341fce2ba35814a0d75ebaadfe436b593a9281c8c7e40`. 실제 실행 컨테이너의 image ID 및 `runtime=nvidia`와 대조했다. 기존 world `{}`를 복구했으므로 자기 충돌과 계획 대상 구체 외 주변 환경 장애물은 포함하지 않는다. 일반 IK 기본 backend는 `native`로 유지한다.

### GO2 실측이 누락된 원인과 수정

Unitree bridge는 domain 1에서 구독했지만 실제 GO2는 domain 0에서 발행했다. 읽기 전용 12초 관측에서 domain 1은 0건, domain 0은 SportModeState 3598건·LowState 5565건을 수신했다. 두 DDS 네트워크를 분리하기 위해 EleSim 전체를 1, Unitree를 0으로 옮겼다.

- com3 `/home/idim/newsim-robot/apps/robot/config/default.yaml`: `go2.ros_domain_id: 1 → 0`. native `elesim-net configure`로 domain1/tailscale0/static peers를 재생성했다.
- native 설치된 `apps/robot/venv/lib/python3.10/site-packages/elesim_robot/runtime.py`: `leg_dq`, `leg_torque_nm` 상태 전달 추가. 이 파일은 설치본에 직접 수정했으며 Git 원본에 반영한 것은 아니다.
- 실제 [domain0 수신](diagnostics/telemetry-20261028/go2-domain0-confirmed.log), [02:08 UI snapshot](diagnostics/go2-domain-repair-20261028/ui-snapshots-after.log)에서 body z 약 0.049m, 다리 12개 q/dq/torque와 timestamp 갱신을 확인했다. 모터 ID1–5 위치 및 ID1–4 전류도 수신했다. 렌더링 화면과 실제 기체를 눈으로 비교한 검증은 하지 않았다.
- 서버 복귀 후 추가 UI observer는 ROS overlay를 source하지 않아 초기화 실패했다(`server-sim-restore-20261028/ui-snapshots-after.log`). 이를 통신 단절 근거로 사용하지 않는다. 성공 근거는 앞의 02:08 snapshot 및 서버의 measured-pose mirror 로그다.
- 정확한 수정은 [native.patch](diagnostics/go2-domain-repair-20261028/native.patch), 백업/도메인 등록 스크립트는 같은 디렉터리에 보존했다.

### Extend Arm과 Wrap 최종 적용

사용자의 마지막 Extend Arm 지시는 “Home 자세처럼”이었다. `extend_arm_controls()`의 `apply_offset=False → True` 한 줄을 변경했다. `u_roll=180.0`은 유지한다. `180+9, apply_offset=False`였던 pale-lynx 중간안은 철회했으며 실행하지 않았다. 최종 변경은 pale-marten에 배포·재기동했고 pale-pangolin이 계승한다. [정확한 diff](diagnostics/extend-arm-home-offset-20261028/changes.patch).

Wrap은 설치된 Roll 방향과 oct27 Jetson 설치 기록의 motor_direction이 달랐다. 사용자의 “좌우반전만” 지시에 따라 policy나 수동 조작은 바꾸지 않고 `roll_sign: 1.0 → -1.0`만 적용했다. [정확한 diff](diagnostics/wrap-direction-20261028/changes.patch). Pilot 재기동으로 캐시된 설정도 갱신했다. 실제 버튼/모터/grasp 시험은 하지 않았다.

별도 미해결 관찰: Wrap runner의 sleep 콜백이 전달되지 않는 경로가 있어 lift 명령이 짧은 시간에 연속 전송되는 로그가 있었다. 사용자가 방향만 수정하도록 범위를 지정해 타이밍 동작은 변경하지 않았다.

### Sim 서버 복귀

노트북 Sim은 scene build 약 132.25초, 다음 시작도 약 131.29초가 관찰됐다. 단순 최초 캐시 생성만의 문제로 설명하지 않는다. 사용자 요청으로 com2를 복구했다. 서버에서 항상 더 빨라진다는 성능 보증은 하지 않는다.

- com1: UI 전용 release `45a2c37eb1d2fd94ea423ca525e8a2692b932cbebd7bc4b442b82e367202374f`. 로컬 Sim은 정지·원복용 보존.
- com2: Sim release `df51144a6029bbbd585958f8bfff1798440e209338bda7f3ba4a2f5b97c0e6ea`, `dawn-otter`, GPU 2.
- com3: Pilot/native 설정에 서버 Tailscale peer 복구. GO2 eno1/domain0 유지.
- [topology diff](diagnostics/server-sim-restore-20261028/topology.patch), [등록 스크립트](diagnostics/server-sim-restore-20261028/register.py), [카메라 준비 완료](diagnostics/server-sim-restore-20261028/media-ready.log), [mirror 로그](diagnostics/server-sim-restore-20261028/server-ready.log).
- 서버의 `emulator`, `doosan_kos` 컨테이너는 변경하지 않았다.

### cuRobo는 저장소에 있었지만 컨테이너 배포에서 누락

확인한 commit은 main `c26fb836633daf6b28ec85c4528ab0f47fabf9e8`, oct27 `1a8913825c45112fe9c50a9bca63da8e7cdb2655`다. 둘 다 `docs/curobo.md`, `workbench/tools/setup_curobo_jetson.sh`, `workbench/tools/patch_curobo.py`, Pilot `robot/arm/curobo.py`와 Jetson GPU 검증 evidence를 포함한다. 지정 버전은 NVIDIA v0.7.8 commit `d64c4b005459db10c5dd867d8b30a87d5bda9bdb`에 EleSim 충돌 커널 패치를 적용한 **0.7.8+elesim.1**이다.

실제 UI 오류는 `configure an explicit curobo_world file on the Pilot host`였다. 이후 컨테이너에서 `find_spec('curobo') is None`도 확인했다. planning 모델 폴더까지 누락됐으므로 설정 한 줄만 고쳐 해결할 문제가 아니었다.

1. 기존 Jetson `/home/idim/elesim/roles/pilot/venv`에서 패치된 패키지와 필요한 의존성만 선별했다. 기존/현재 모두 Torch2.11.0·CUDA12.6·Python3.10이다. Torch/TorchVision/NumPy는 교체하지 않았다. 모든 파일 SHA256과 버전은 [packages-manifest.json](diagnostics/curobo-restore-20261028/packages-manifest.json)에 기록했다.
2. 기존 Jetson planning 모델은 172.7mm라 복사하지 않았다. 현재 모델은 230mm이며 main planning 모델의 source hash가 현재 arm/default.json의 `033f7fbec380d705d82b34f6ef9a419dcb05ea5b0bdaf927de9460da2482a1f3`와 일치했다. main의 arm.urdf/robot.yml/metadata.json을 사용하고 artifact hash도 검증했다.
3. 기존 Jetson의 명시적 `{}` world.yaml을 복구하고 `curobo_world: world.yaml`로 지정했다. 자기 충돌과 Plan XYZ 대상 구체 외 환경 장애물은 포함하지 않는다.
4. model 경로를 설치기에서 `/opt/elesim/data/models/planning/zed-mini`로 바인딩했다. 일반 IK backend는 native 유지.
5. 추가 라이브러리를 새 immutable Pilot 이미지에 넣고 release를 발행했다. instance/Connections를 갱신하고 Pilot을 재기동했다. 앞선 Extend Arm/Wrap/+9 offset/domain1/3peers/NVIDIA 설정을 계승했다.
6. 로컬과 com3의 직접 설치 소스, 앱 config, runtime snapshot, data에도 파일을 복구했다. 설치기의 planning 폴더 복사 누락 및 기본 상대경로 바인딩을 수정했다. Tools 이미지를 재빌드한 것은 아니다.

**향후 upstream 반영 사항:** 완전히 새 Pilot 기본 이미지를 만드는 원래 Dockerfile에는 여전히 별도 cuRobo 소스 빌드 단계가 없다. pinned 설치/패치 단계를 정식 이미지 빌드에 통합해야 한다. 현재 Connections는 복구한 immutable 이미지/release를 사용하므로 매번 재설치할 필요는 없다.

검증은 새 이미지에서 planner/CUDA extension import, 실제 Orin CuRoboPlanner 생성 및 GPU FK 계산까지다. 모델 초기화 약 69.32초, Linear bounds -0.23..0. [검증 로그](diagnostics/curobo-restore-20261028/runtime-check.log). `--network none` 별도 컨테이너에서 수행해 DDS/하드웨어 명령이 없었다. Plan XYZ 해 찾기·Execute path·Wrap 동작 시험은 하지 않았고 사용자 지시대로 TDD/테스트 스위트는 작성·실행하지 않았다. PyTorch sparse invariant 경고가 출력됐지만 정상 종료했다.

### 정확한 추가/삭제 기록과 원복

- [config diff](diagnostics/curobo-restore-20261028/config.patch)
- [설치기 코드 diff](diagnostics/curobo-restore-20261028/installer.patch)
- [신규 모델·source config·snapshot 전체 diff](diagnostics/curobo-restore-20261028/source-data.patch)
- [Connections release 교체 diff](diagnostics/curobo-restore-20261028/topology.patch)
- [실제 이미지 Dockerfile](diagnostics/curobo-restore-20261028/Dockerfile), [패키지 복구](diagnostics/curobo-restore-20261028/stage.py), [발행 스크립트](diagnostics/curobo-restore-20261028/publish.py), [등록 스크립트](diagnostics/curobo-restore-20261028/register.py), [source 동기화](diagnostics/curobo-restore-20261028/sync-source.py)
- [브랜치·버전 근거](diagnostics/curobo-restore-20261028/branch-evidence.json), [최종 release 기록](diagnostics/curobo-restore-20261028/result.json)

com3 `diagnostics/curobo-restore-20261028/registration-backup/`에 이전 instance/Compose/ownership, 각 호스트 `source-backup/`에 이전 config/source를 보존했다. pale-marten 이미지/release도 유지했다. 원복 시 Pilot 정지 후 이전 release를 재등록하고 Connections release도 함께 되돌려야 한다.

빌드 완료 후 작업 중 만든 임시 `packages/` 복사본만 제거했다. 원래 Jetson venv, published image, manifest와 복구 스크립트는 보존했다. 마지막 확인 Jetson 여유 공간 약 1.1GB로 전체 CUDA 재빌드는 별도 공간 확보가 필요하다.


### 최신 수정의 코드 diff 원문

`diagnostics/go2-domain-repair-20261028/native.patch`

```diff
--- a/native/apps/robot/config/default.yaml
+++ b/native/apps/robot/config/default.yaml
@@ -71,7 +71,7 @@
   pose_source: sportmodestate
   ros_workspace: /home/idim/ros2_ws
   network_interface: eno1
-  ros_domain_id: 1
+  ros_domain_id: 0
   ipc_socket_path: /run/elesim-unitree/bridge.sock
   ipc_robot_user: idim
   ipc_bridge_user: elesim-unitree

--- a/native/elesim_robot/runtime.py
+++ b/native/elesim_robot/runtime.py
@@ -615,6 +615,10 @@
                 }
                 if sample.leg_q is not None:
                     result["go2"]["leg_q"] = list(sample.leg_q)
+                if sample.leg_dq is not None:
+                    result["go2"]["leg_dq"] = list(sample.leg_dq)
+                if sample.leg_torque_nm is not None:
+                    result["go2"]["leg_torque_nm"] = list(sample.leg_torque_nm)
         return result
 
 
```

`diagnostics/extend-arm-home-offset-20261028/changes.patch`

```diff
--- a/elesim_pilot/pick/actions.py
+++ b/elesim_pilot/pick/actions.py
@@ -2145,7 +2145,7 @@
 
     def extend_arm_controls(self) -> None:
         self.state.clear_ik_status()
-        self.apply_control_u(u_linear=15.0, u_roll=180.0, u_s1=180.0, u_s2=180.0, apply_offset=False)
+        self.apply_control_u(u_linear=15.0, u_roll=180.0, u_s1=180.0, u_s2=180.0, apply_offset=True)
         self.send_current_target(source="slider")
 
     def send_current_target(
```

`diagnostics/wrap-direction-20261028/changes.patch`

```diff
--- a/config/pilot/wrap_grasp.yaml
+++ b/config/pilot/wrap_grasp.yaml
@@ -5,4 +5,4 @@
   height_m: 1.1
   xyz_m: [0.368, 0.16, 0.55]
   lean: [0.0, 0.0]
-roll_sign: 1.0
+roll_sign: -1.0
```

`diagnostics/curobo-restore-20261028/config.patch`

```diff
--- a/config/pilot/config.yaml
+++ b/config/pilot/config.yaml
@@ -385,8 +385,8 @@
       robot_name: Robot
     ik:
       backend: native
-      curobo_model: ../../data/models/planning/zed-mini
-      curobo_world: ''
+      curobo_model: ../data/models/planning/zed-mini
+      curobo_world: world.yaml
       damping_down: 0.7
       damping_init: 0.01
       damping_max: 100.0
--- /dev/null
+++ b/config/pilot/world.yaml
@@ -0,0 +1 @@
+{}
```

`diagnostics/curobo-restore-20261028/installer.patch`

```diff
--- a/containers/build/tools/app/elesim_setup/container_installer.py
+++ b/containers/build/tools/app/elesim_setup/container_installer.py
@@ -1162,6 +1162,8 @@
             _copy_tree(root / "payload/data/calibration", data_root / "calibration")
         if role == "pilot":
             _copy_tree(root / "payload/data/models/arm", data_root / "models/arm")
+            if (root / "payload/data/models/planning").is_dir():
+                _copy_tree(root / "payload/data/models/planning", data_root / "models/planning")
             _copy_tree(
                 root / "payload/data/models/perception",
                 data_root / "models/perception",
--- a/containers/build/tools/app/elesim_setup/installer.py
+++ b/containers/build/tools/app/elesim_setup/installer.py
@@ -450,6 +450,11 @@
                 root / "payload/data/models/arm",
                 data_root / "models/arm",
             )
+            if (root / "payload/data/models/planning").is_dir():
+                _copy_tree(
+                    root / "payload/data/models/planning",
+                    data_root / "models/planning",
+                )
             _copy_tree(
                 root / "payload/data/models/perception",
                 data_root / "models/perception",
--- a/containers/build/tools/app/elesim_setup/configuration.py
+++ b/containers/build/tools/app/elesim_setup/configuration.py
@@ -132,6 +132,13 @@
         simulation.setdefault("assembly", {})["build_dir"] = str(
             runtime_data_root / "models/assemblies/zed-mini"
         )
+    if role == "pilot":
+        ik = raw.setdefault("robot", {}).setdefault("arm", {}).setdefault("ik", {})
+        if ik.get("curobo_model") in {
+            "../../data/models/planning/zed-mini",
+            "../data/models/planning/zed-mini",
+        }:
+            ik["curobo_model"] = str(runtime_data_root / "models/planning/zed-mini")
     config_path.write_text(
         yaml.safe_dump(raw, sort_keys=False, allow_unicode=True),
         encoding="utf-8",
```

---

## 이력: 02:02 중간안 — 이후 철회·대체됨

저장된 topology의 Pilot 선택은 `pale-lynx`로 갱신됐다. 실행 중 Pilot은 자동 중단하지 않았으므로 기존 `pale-badger`가 계속 실행된다. 기존 Connections에서 전체 종료한 뒤 관리자를 새로 열고 demo를 시작하면 새 릴리스가 등록된다. GO2 도메인 불일치 원인은 확인했으나 아직 수정하지 않았다. 자세한 원인·diff는 문서 끝의 후속 기록에 있다.

## 이력: 2026-10-28 01:58 — 이후 서버 Sim 복귀로 대체됨

**앞선 세 호스트/2.4GHz 기록을 대체하는 최종 구성이다.** 사용자 요청에 따라 com2를 demo 토폴로지에서 제거하고 Sim을 노트북으로 옮겼다. 이후 사용자가 LAN 전환을 보류했으므로 **통신은 Tailscale을 유지**한다. 아래의 `lan-migration-20261028` 디렉터리명은 작업 시작 당시 이름이며 최종 네트워크가 LAN이라는 뜻은 아니다.

| 호스트 | 최종 역할 | DDS 주소 / 인터페이스 | 실행 이미지 |
|---|---|---|---|
| com1 노트북 | UI + Sim | `100.82.201.62` / `tailscale0` | UI `clear-lynx`, Sim `clear-heron` |
| com3 Jetson | Pilot + native Robot/bridge | `100.88.154.118` / `tailscale0` | Pilot `pale-badger`, Robot native |
| com2 서버 | demo에서 제거, 기존 Sim 정지 | 최종 static peers에서도 제거 | 기존 설치/이미지는 보존 |

- Jetson Wi-Fi: **H3C_D41BD2_5G**, BSSID `DC:65:55:58:D2:1B`, **5580MHz**, 실제 절전 **off** 확인. LAN IP `192.168.50.11`은 유지되지만 운영 토폴로지의 DDS/SSH 주소로 사용하지 않는다.
- Pilot 롤 오프셋: PC/Jetson 프로필 모두 **-9.0 → +9.0**. 새 릴리스 및 설치된 demo 설정에 반영. 다음 Pilot 시작부터 적용하며 실행 중 모터 명령으로 값을 밀어넣지 않았다.
- 마지막 확인 시 UI/Sim/Pilot/Robot은 Connections로 시작할 수 있도록 정지했다. 새 구성으로 전체 시작·실제 조작·장시간 네트워크 안정성을 검증한 상태는 아니다.
- 사용자 인계: **Connections를 새로 열고 demo 선택 → 시작 → `DDS readiness complete: all hosts` 확인 후 조작**. 이전 관리자 세션은 작업 중 정지했다. 이전 화면의 캐시된 세 호스트 설정으로 저장하지 않는다.
- 5GHz 전환 후 Jetson→노트북 ping 8/8 성공, RTT 약 3–77ms. 짧은 idle 관측이며 영상/Sim 부하 중 무단절 보증은 아니다.

최종 변경의 정확한 diff·릴리스·원복은 문서 마지막의 “com2 제거·Sim 노트북 이전·5GHz·롤 오프셋 최종 적용”에 기록했다.

---

## 통합 인계 — 2026-10-28 이전 단계까지의 변경과 미완료 사항

아래 본문은 시간순 기록이다. 이전 절의 “현재”, “최종”, “검증 완료”는 **그 절을 작성한 시점과 검증 범위**를 뜻한다. 최신 요구사항·배포 상태는 이 통합 인계와 후속 기록을 우선한다. 특히 초기 카메라 분리 작업 당시의 “Robot 자동 선택 없음”은 이후 Robot 우선 제어 정책으로 갱신됐으며, `clear-otter`는 이후 `clear-lynx`로 대체됐다.

### 적용한 수정 전체

| 항목 | 실제 적용 내용 | 확인 범위 / 남은 부분 |
|---|---|---|
| native Python 실행 실패 | com3 venv Python을 `/usr/bin/python3.10`으로 연결, pyvenv.cfg 수정, 설치 모듈의 base executable 사용 | 브리지 status=126 원인 확인 및 서비스 기동 확인. 저장소 전체 반영은 별도 필요 |
| 설정 읽기 권한 | 서비스 계정 read ACL 부여, native `_atomic_text()`가 mode/ACL 보존 | 실제 설정 재생성 전후 ACL 유지 확인 |
| 장비별 설정 유실 | native default/installed YAML의 eno1·ROS workspace·사용자 보정 | 호스트 설정을 설치 상태에 보존하는 제품 구조 수정은 미완료 |
| tools 이미지 자동 삭제 | 정리 대상 계산에 모든 Compose profile 포함, tools 복원 | 호스트별 maintenance 사본 및 tools 소스 수정. 삭제된 이미지 때문에 발생한 재빌드 지연 해소 |
| 저장 공간 | Jetson 미사용 빌드 캐시 정리, 약 5.177GB 회수 | 이후 가용 약 3.1GB. 용량 자동 관리 기능은 미구현 |
| UI 시작/X11 | 초기 제한된 X11 사용자 허용으로 복구, 후속 GPU UI는 호스트 UID 1000 사용 | 세션 인증 수명주기 전체의 제품 개선은 별도 |
| 텍스트 입력 | GLFW 이벤트가 해당 창의 ImGui 컨텍스트에서 처리되도록 수정 | 두 실제 GLFW 창에서 문자 입력 확인. 새 UI 이미지 반영 |
| UI 반응 / GPU | 입력·유휴·카메라 갱신 주기 조정, 이벤트 대기 사용, 설치별 NVIDIA 정책 저장 | 실제 NVIDIA OpenGL renderer 확인. 전체 실기기 제어 지연 검증과는 별개 |
| 제어 / 카메라 선택 | 카메라 선택 콜백에서 제어 대상 변경을 분리 | 카메라 Sim 선택이 제어 Robot을 변경하지 않도록 구현 |
| Robot / Sim 정책 | Robot 발견 시 조종 대상은 Robot만, Sim은 선택 Robot의 실측 표시 | fake endpoint와 코드 경로 확인. 실제 Robot→Sim 전 구간 검증은 미완료 |
| 모터 목표 유지 | 실측 sag 값을 다음 명령 목표로 덮어쓰는 경로 제거, 마지막 명령 목표 유지 | 메모리 전송 대역으로 재현·확인. 실제 하중 유지 성능 미검증 |
| Wrap grasp | oct27 정책/manifest/config 복구, 설치 경로 보정 | 실제 Jetson에서 모델 로드·첫 추론 확인. 실제 grasp 미실행 |
| 동물 이름 / 목록 | 이미지 태그, 불변 릴리스, instance/topology, release catalog 갱신 | 이전 SHA 태그는 롤백 참조로 유지 |
| Jetson GPU 런타임 | 수동 배포 시 빠진 `runtime: nvidia` 복구, 배포 보조 스크립트에 L4T 전달 | 실제 Pilot 재기동 확인. 이 배포 과정의 실수도 본문에 기록 |
| Wi-Fi 절전 | com3 연결 프로필 powersave=2, 실제 인터페이스 off, 부팅 oneshot 등록 | `off` / `disable` / `enabled` 확인. 끊김 원인 확정이나 장기 안정성 검증은 아님 |

### 최신 실행 이미지와 제어 규칙

| 역할 | 사용해야 할 배포 태그 |
|---|---|
| UI / com1 | `elesim/ui:clear-lynx` |
| Pilot / com3 | `elesim/pilot:pale-badger` |
| Sim / com2 | `elesim/sim:dawn-otter` |
| Tools / com1 | `elesim/tools:clear-shark` |

세 runtime의 정확한 image ID, release key, fingerprint 및 이전 릴리스는 본문과 `diagnostics/robot-sim-repair/deployed-releases.json`에 보존했다. 태그 이름만 같다고 업데이트됐다고 판단하지 말고 등록 release key와 실제 실행 image ID를 함께 대조해야 한다.

- Robot을 발견하지 않은 Sim-only 구성에서는 Sim을 조종한다.
- Robot을 발견하면 Robot만 조종 대상으로 제공하고 Sim은 선택된 Robot의 실측 표시를 맡는다.
- 현재 Pilot 프로세스가 이미 발견한 Robot은 통신이 끊겨도 기억한다. 일시 단절을 “Robot 없음”으로 간주해 Sim으로 제어를 넘기지 않는다. 이 기억은 프로세스 재시작을 넘는 영구 저장은 아니다.
- 카메라 창의 Sim/Robot 선택은 영상 소스 선택이며 제어 Target과 독립이다.

### 실제 장비에서 확인된 것과 확인하지 못한 것

01:38:11 KST 읽기 전용 UI snapshot에서 모터 1–5 위치와 1–4 전류가 실제로 수신됐다. Host state age는 약 0.063초였다. 같은 snapshot의 GO2 몸체/다리 값은 null이고 timestamp=0이었다. **모터 수신 확인을 GO2 수신 확인이나 전체 제어 정상으로 확대하면 안 된다.** UI 전류 표에는 숫자 ID와 축 이름 매핑 불일치가 있어 수신값이 `-`로 표시될 수 있으며 아직 수정하지 않았다. Gripper 전류는 샘플에 없었고 표시 코드도 None 고정이다. ID 5의 실제 측정 필드·단위부터 확인해야 하며 부하값을 전류로 가장하면 안 된다.

사용자 요청에 따라 TDD/새 테스트 스위트를 추가하지 않았다. 이전의 입력·라우팅·목표 유지 진단 스크립트, 문법 확인, 이미지 기동, Jetson 모델 추론과 실제 로봇 동작 검증을 구분한다. 실제 모터 하중 유지·Wrap grasp·GO2 이동·Robot→Sim 전체 미러 동작은 완료됐다고 기록하지 않는다. 이번 수신 및 단절 진단에서는 모터 명령을 발행하지 않았다.

### 서로 다른 두 단절 사건

| 시간 (2026-10-28 KST) | 관측 | 판단 |
|---|---|---|
| 01:38:21–01:40:54 전후 | UI에서 Robot/Pilot lost, Jetson LAN·Tailscale·SSH 무응답. Jetson 내부 Robot↔Pilot은 지속 | 외부 네트워크 경로 장애. 정확한 무선/드라이버/AP 원인은 미확정 |
| 01:40:55–01:41 | 같은 boot ID로 UI/Robot/Pilot 재발견. 재부팅·서비스 재시작 없음 | 통신 자연 복구 관측. 수리 완료가 아님 |
| 01:42:23–01:42:36 | Connections readiness SSH 채널 timeout → 자동 롤백 → Robot stop 및 Pilot TERM/KILL | 이번 재단절의 직접 원인은 Connections가 내린 종료 명령. OOMKilled=false |
| 이후 절전 해제 | Wi-Fi 실제 off, 프로필 disable, 부팅 unit enabled | 원인 후보 하나 제거. 재실행 후 안정성 확인 필요 |

`start complete`와 UI 창 표시는 전체 시작 성공을 뜻하지 않는다. 뒤에서 DDS 준비 검사가 계속되며 실패하면 이미 시작한 런타임을 종료한다. 이번에는 com3 `Timeout opening channel`이 221.84초 뒤 전체 실패 처리에 포함됐다. 준비 상태 표시 및 실패 처리의 구체적인 개선 지점은 본문 마지막에 기록했다.

### 남은 수정 / 후속 이슈로 분리할 항목

1. **네트워크 단절:** 절전 off 상태에서 연결 안정성을 관측하고, 재발하면 양쪽 시각을 맞춘 Wi-Fi/AP·드라이버·Tailscale 로그를 수집한다. 절전이 원인이라고 확정하지 않는다.
2. **Connections 준비 확인:** probe 전송 실패와 실제 DDS 미준비를 분리하고 제한된 재접속/최신 상태 재검사를 검토한다. 시작 미완료 표시를 분명히 해 조작 중 지연 롤백처럼 보이지 않도록 한다. 현재 이 코드는 미수정이다.
3. **GO2 실측:** eno1/domain 1/workspace 환경에서 sportmodestate·lowstate 토픽 수신부터 조사한다. IPC 진단 클라이언트를 추가 연결해 기존 단일 클라이언트를 밀어내지 않는다. ROS 원본→bridge→Robot telemetry→Pilot→Sim/UI 구간을 나눠 확인한다.
4. **Hardware 전류 표시:** ID→축 매핑 및 Gripper 실제 제공 필드 확인 후 표시 수정. 현재 미적용.
5. **실제 동작 인수:** 사용자가 수행할 모터 유지, Wrap grasp, Robot 종속 Sim 동작 확인. 실행 이미지 성공이나 모델 첫 추론만으로 통과 처리하지 않는다.
6. **제품 반영:** native 호스트 설정 영속화, 권한 보존, 설치/업데이트 공간 검사, X11 인증 수명주기 및 GPU 선택 UI를 저장소에 정식 반영해야 한다.

**connection manager에 UI gpu 고르는 것도 넣어야 할 것으로 보임.**

이전 CUDA 12.6 다운그레이드의 이유는 이번 조사에서 확인되지 않았다. 이번 작업에서 CUDA toolkit을 설치·삭제·다운그레이드하지 않았다. 노트북 드라이버 버전 불일치는 재부팅 후 해소된 것을 확인했으며, Jetson Wi-Fi 절전 해제에는 재부팅이 필요 없었다.

### 기록·복구 기준

이 문서의 파일별 unified diff, 배포 메타데이터, 원본 백업을 함께 보존한다. Git 저장소가 아니므로 Git에 반영됐다고 간주하지 않는다. 기존 이미지/릴리스를 삭제하지 않았으며, runtime 원복은 호스트별 backup과 기존 release key를 이용해 topology·instance·compose를 일치시켜 수행해야 한다. 일회성 배포 스크립트를 무조건 재실행하지 않는다.

이 통합 정리 작업은 문서만 수정했다. 실행 중 상태를 바꾸거나 서비스를 재시작하지 않았다. 마지막 상태 확인 시 Robot·Pilot·UI는 롤백 후 정지였으며, 사용자가 이후 다시 시작한 상태까지 이 문서가 실시간으로 보증하지 않는다.

---

## 환경과 증상

`connections/demo/topology.json`의 `demo` 구성:

| 호스트 | 역할 | 설치 방식 / 경로 |
|---|---|---|
| com1 | UI, Connections 관리자 | Docker, `/home/idim/newsim` |
| com2 | Sim | Docker, `/home/hckang/newsim` |
| com3 / Jetson | Pilot | Docker, `/home/idim/newsim` |
| com3 / Jetson | Robot, Unitree bridge | native/systemd, `/home/idim/newsim-robot` |

EleSim 통신은 `tailscale0`, static discovery, domain 0, trusted-network 구성이다. Unitree 측 네트워크는 별도 `eno1`, domain 1을 사용하도록 설정되어 있다.

처음 관측된 오류:

```text
start complete: com1 (1.78s)
start complete: com2 (57.64s)
start failed: com3 (151.82s)
Runtime startup or DDS readiness failed; rolling back runtimes started by this operation.
sudo -n systemctl start elesim-robot.service:
A dependency job for elesim-robot.service failed.
```

이후 Connections에서 별도의 오류도 발견했다:

```text
network preparation failed: com3 (120.08s)
remote command timed out after 120.0 seconds:
/home/idim/newsim/bin/elesim-net show
```

## 1. Robot venv가 접근 불가능한 설치 캐시 Python을 경유함

### 원인 / 증거

브리지 서비스는 `User=elesim-unitree`로 실행된다. 그런데 Robot venv의 Python 링크는 다음 경로를 경유했다.

```text
newsim-robot/apps/robot/venv/bin/python
 -> ~/.cache/elesim/setup/environments-v2/app-.../env-8fzpsjfl/bin/python
 -> python3 -> /usr/bin/python3 -> python3.10
```

중간의 `env-8fzpsjfl` 디렉터리가 `0700`이라 서비스 계정이 통과할 수 없었다. 최종 바이너리가 실행 가능하더라도 중간 링크 경로 접근이 필요하다.

```text
2026-10-27 23:22:52
.../venv/bin/python: bad interpreter: Permission denied
elesim-unitree-bridge.service: Main process exited, status=126
elesim-robot.service: Job .../start failed with result 'dependency'
```

### 실제 수정

com3:

- `apps/robot/venv/bin/python` 링크를 `/usr/bin/python3.10`으로 변경.
- `apps/robot/venv/pyvenv.cfg`의 `home`을 `/usr/bin`으로 변경.
- 아래 설치 모듈의 `Installer._ensure_venv()`도 수정.

```text
/home/idim/newsim-robot/tools/venv/lib/python3.10/site-packages/elesim_setup/installer.py
```

```diff
- arguments = [sys.executable, "-m", "venv"]
+ arguments = [str(Path(getattr(sys, "_base_executable", sys.executable)).resolve()), "-m", "venv"]
```

저장소 수정 대상은 `elesim_setup/installer.py`의 같은 함수다. 이 설치본의 소스 대응 위치는 `containers/build/tools/app/elesim_setup/installer.py`, 원본 패키지 경로는 `payload/runtime/docker/setup/app/elesim_setup/installer.py`다. **com1의 이 소스 사본까지 수정한 것은 아니며, 실제 수정은 com3 native 설치 모듈에 적용했다.**

## 2. 설정 재생성이 서비스 계정의 ACL을 소실시킴

### 원인 / 증거

`elesim-unitree` 계정이 `apps/robot/config/installed.yaml`을 읽을 수 없었다. 파일은 소유자 전용 `0600`이었다.

`configuration.py::_atomic_text()`는 `NamedTemporaryFile`에 새 내용을 쓰고 기존 파일을 교체한다. 기존 파일의 mode/ACL을 복사하지 않아 설치 시 부여했던 읽기 ACL이 설정 재생성 후 사라질 수 있다.

### 실제 수정

com3의 `default.yaml`, `installed.yaml`에 `u:elesim-unitree:r` ACL 부여. 이어 아래 native 설치 모듈의 `_atomic_text()`를 수정했다.

```text
/home/idim/newsim-robot/tools/venv/lib/python3.10/site-packages/elesim_setup/configuration.py
```

```python
# temporary 파일을 닫은 후, replace 직전
if path.exists():
    shutil.copystat(path, temporary)
    os.utime(temporary, None)
temporary.replace(path)
```

이 Linux 설치에서 `copystat`이 ACL을 보존하는 것을 **실제 native 설정 파일 재작성 전후의 `system.posix_acl_access` 비교**로 확인했다.

저장소 수정 대상은 `elesim_setup/configuration.py::_atomic_text()`다. 신규 생성 시 서비스 계정 권한 부여와 기존 파일 교체 시 권한 보존을 모두 다뤄야 한다. 위 코드는 현장 수정이며 소유권 이전 등의 일반적인 모든 배포 조합을 검증한 구현은 아니다.

## 3. Connections 설정 재생성이 Robot의 호스트 설정을 기본값으로 되돌림

### 원인 / 증거

`configuration.py::_robot_config()`는 `default.yaml`에서 시작하고, `robot_host`가 전달될 때만 호스트 설정을 덮어쓴다. 설치 시에는 `robot_host`를 넘기지만, `elesim-net configure`의 재생성 경로는 `generate_role_configs(updated)`를 호출한다.

이 장비의 생성된 설정에 다음 잘못된 값이 있었다:

| 필드 | 잘못된 값 | 실제 장비에 적용한 값 |
|---|---|---|
| `go2.ros_workspace` | `/opt/unitree_ros2` — 경로 없음 | `/home/idim/ros2_ws` |
| `go2.network_interface` | `eth0` — 인터페이스 없음 | `eno1` |
| `go2.ipc_robot_user` | `elesim` | `idim` |
| `go2.ipc_bridge_user` | `elesim-unitree` | 유지 |
| `go2.ros_domain_id` | `1` | 유지 |

### 실제 수정 / 저장소에 필요한 수정

현장에서는 com3 native 설치본의 **`default.yaml`과 `installed.yaml`을 모두 수정**했다. 따라서 다음 재생성에도 올바른 값이 나오도록 했다.

저장소에서는 다음 두 경로를 수정해야 한다:

- `elesim_setup/configuration.py::_robot_config()`
- `elesim_setup/network.py`의 `generate_role_configs(updated)` 호출 경로

호스트 설정을 설치 상태에 영속 저장하고 재설정 시 전달하거나, 검증된 기존 호스트 설정을 보존해야 한다. 이 장비의 `eno1`/사용자명 등을 제품 기본값으로 하드코딩하면 안 된다. 이 구조적 수정은 아직 적용하지 않았으며 현장에서는 설치된 템플릿을 보정했다.

## 4. 자동 이미지 정리가 현재 tools 이미지를 삭제함 — Connections 120초 지연의 직접 원인

### 원인 / 재현 조건

`image_cleanup.py::collect()`는 현재 사용 중인 이미지 태그를 다음 명령으로 수집했다:

```sh
docker compose -p PROJECT -f compose.yaml config --images
```

이 명령 결과에는 비활성 프로필의 서비스가 제외된다. `tools`/`manager`가 프로필 서비스여서 현재 관리 도구 이미지도 보호 목록에서 빠졌다. 실행 중인 관리 컨테이너가 없는 원격 호스트에서는 그 이미지가 미참조 이미지로 삭제될 수 있었다.

확인된 Jetson 상태:

- `elesim/tools:pale-heron`이 사라짐.
- `elesim-net show`가 빌드 fingerprint를 확인하다 이미지 부재를 발견하여 tools 빌드를 시작함.
- Connections는 120초 명령 제한으로 실패.
- 재시도하면서 여러 `elesim-net show`와 `docker buildx bake`가 겹쳐 실행됨.
- Jetson 자체는 직접 SSH에 정상 응답함.

빌드 캐시 정리 이후 복원 빌드는 캐시가 있을 때보다 오래 걸렸다. **이미지 보호 누락과 캐시 부재로 인한 복원 시간 증가는 구분해야 한다.**

### 실제 수정

세 호스트의 다음 두 파일을 직접 수정했다:

```text
<설치 루트>/maintenance/elesim_setup/image_cleanup.py
<설치 루트>/containers/build/tools/app/elesim_setup/image_cleanup.py
```

`collect()`의 `current_tags` 수집만 다음처럼 변경:

```diff
- current_tags = set(call("compose", "-p", project, "-f", boundary.compose_file,
+ current_tags = set(call("compose", "--profile", "*", "-p", project, "-f", boundary.compose_file,
                          "config", "--images").splitlines())
```

모든 설정된 프로필의 현재 이미지 태그를 보호한다. SSH 인증이나 이미지 소유권 검사를 우회하지 않는다.

Jetson tools 이미지 복원이 완료된 후 동일한 `elesim-net show`가 **2.151초**에 완료됐고, 실제 Connections Prepare도 세 호스트 모두 통과했다.

저장소에는 생성/배포되는 maintenance 사본까지 같은 수정이 반영되어야 한다. 설치된 maintenance 코드만 고치면 다음 재설치 때 되돌아갈 수 있다.

## 5. com1 UI X11 인증 실패

```text
Authorization required, but no authorization protocol specified
X11: Failed to open display :0
glfw.init() failed.
```

UI Compose에는 DISPLAY와 X11 소켓만 전달되고 Xauthority 파일 전달은 없었다. 호스트 X11 ACL은 `idim`만 허용했고 UI 컨테이너는 root로 실행됐다.

현장 조치:

```sh
xhost +si:localuser:root
```

이 조치 후 UI가 지속 실행되고 영상 프레임을 수신했다. **현재 데스크톱 세션에 대한 접근 허용이며, 영구적인 UI 인증 구현을 추가한 것은 아니다.**

저장소 수정 후보는 `elesim_setup/instance_runtime.py`의 UI 실행 래퍼 생성과 UI Compose 생성 경로다. 현재 X11 준비 처리는 `instance.viewer`인 Sim 쪽에 집중되어 있다. UI도 실제 실행 사용자와 Xauthority 또는 제한된 ACL의 설정·정리 수명주기를 처리해야 한다.

## 6. Jetson 저장 공간 고갈

`/dev/mmcblk0p1` 57GB 루트 파일시스템에 사용자 가용 공간이 0이었다. 수정 전 백업 디렉터리 생성부터 `No space left on device`로 실패했다.

실행 이미지/컨테이너/볼륨을 대상으로 한 일괄 삭제 없이, 미사용 Docker 빌드 캐시를 정리했다:

```sh
docker builder prune --force --keep-storage 2GB
```

실제 회수량은 5.177GB, 직후 가용 공간은 3.7GB였다. 이후 tools 복원 뒤 확인 시에는 약 3.1GB였다. 제품 차원에서는 설치/업데이트/백업 전 가용 용량 확인과 빌드 캐시 예산 안내가 필요하다.

## 별도 관측: 새 Connections 세션의 간헐적 SSH handshake 실패

위 수정 후 Prepare가 한 번 정상 완료됐으나, 이후 새 Connections 세션에서 다음 별도 실패가 발생했다:

```text
network preparation failed: com3 (20.02s)
SSH connection ... failed: No existing session
paramiko.ssh_exception.SSHException: Error reading SSH protocol banner
```

해당 시점에 직접 SSH는 정상 응답했고, **같은 Connections 컨테이너**의 직접 TCP 검사도 0.05초에 연결되어 `SSH-2.0-Tailscale` 배너를 받았다. 따라서 이 오류를 Jetson 다운 또는 위 tools 빌드 시간 초과와 동일한 원인으로 단정하면 안 된다.

조사 위치:

- `elesim_connections/secure_deployment.py::ParamikoConnector._connect_tailscale()`
- `elesim_setup/credentials.py::open_ssh_connection()`
- 사용되는 경우 `elesim_setup/host_helper.py::_tailscale_nc()`

이 SSH 오류에 대한 네트워크/인증 코드는 아직 수정하지 않았다. 연결 및 배너 수신 단계의 일시적 실패와 세션 종료를 구분할 진단이 필요하다. 무조건적인 타임아웃 상향이나 무제한 재시도로 처리한 상태가 아니다.

2026-10-28 00:00 KST에 동일한 Connections 세션에서 Prepare를 한 번 재시도한 결과는 **completed, error 없음**이었다. com3 네트워크 준비 4.47초, com3 검증 6.30초였고 세 호스트 모두 통과했다. 따라서 최종 인계 시점에는 준비 완료 상태지만, 간헐적 SSH 실패의 원인까지 해결됐다고 단정하지 않는다.

## 검증 범위와 남은 한계

- 최초 로컬 조사: build/setup/runtime 로그 21개, 14,935줄 및 com3 수집 저널.
- Robot/Unitree bridge: 수분간 `active/running`, `NRestarts=0` 확인.
- UI/Pilot/Sim/Robot DDS 발견과 ready 상태 확인.
- Sim 세션 승인, WebRTC answer, observer/hand-eye 1280×720 프레임 ready 확인.
- 실제 native 설정 재작성 후 ACL 유지 및 호스트 설정 재생성 결과 확인.
- tools 복원 뒤 Jetson 조회 2.151초, Connections Prepare 세 호스트 통과 확인.
- 이후 새 Connections 세션의 SSH 실패는 위 별도 항목으로 기록. 모든 후속 Connections 시도가 성공했다고 주장하지 않는다.
- 사용자 요청대로 TDD나 새 자동 테스트 스위트는 작성하지 않았다. 위 내용은 현장 실행/상태 확인 결과다.

## 근거 파일과 백업

com1 (`/home/idim/newsim`):

- `connections/demo/topology.json`
- `logs/build/20261027T141419Z-cf2347aa0f09.log` — 최초 start 실패
- `logs/build/20261027T144701Z-ce7f2a6c32dd.log` — tools 빌드로 인한 Prepare 시간 초과
- `logs/build/20261027T145647Z-550787f615fe.log` — 이후 SSH handshake 실패
- `logs/diagnostics/20261027T143248Z/com3.clean.txt` — 원격 장애 근거
- `diagnostics/repair-com3.py` — 당시 적용한 native 복구 스크립트. 일회성 패치 전제의 assert가 있으므로 그대로 반복 실행하는 용도가 아니다.
- `diagnostics/startup-repair-20261027.md` — 최초 복구 기록
- `diagnostics/backup-image-cleanup/` — com1 이미지 정리 코드 원본

원격 백업:

- com3 native: `/home/idim/newsim-robot/maintenance/startup-repair-20261027T144126Z`
- com3 container: `/home/idim/newsim/maintenance/cleanup-repair-20261027T235249`
- com2: `/home/hckang/newsim/maintenance/cleanup-repair-20261027T235349`

관리자 로그에는 세션 접근 토큰이 포함될 수 있으므로 이슈에는 원본 로그 전체 대신 필요한 오류 구간만 발췌해야 한다. 이 문서에는 토큰과 비밀번호를 넣지 않았다.


## 2026-10-28 추가 수정: UI 입력 / 제어 Target과 카메라 View 분리 / 렌더링 부하

최종 요구사항: 제어 Target 선택은 유지한다. 카메라 창에서 선택하는 것은 영상 소스이며 제어 대상을 바꾸지 않는다. 제어 Target 변경도 카메라 선택을 따라 바꾸지 않는다. 시작 시 Robot을 강제로 자동 선택하는 중간 구현은 최종본에서 제거했다.

### 적용 결과

- `main.py`: 제어 Target 콜백에서 `sim_session.switch_target()` 호출을 삭제했다.
- `control_panel.py`: 카메라 창의 기존 제어 Target 드롭다운을 별도 `_draw_camera_selector()`로 교체했다. 새 선택기는 `camera_source` 및 영상 세션만 변경하며 Pilot 제어 API를 호출하지 않는다. 메인 패널의 Control Target 선택은 그대로 유지했다.
- `control_panel.py`: 문자·키·스크롤·포인터·크기 콜백이 각각 해당 창의 ImGui 컨텍스트에서 실행되도록 하고 이전 컨텍스트를 복원한다. 카메라 렌더링 후에도 메인 컨텍스트로 복귀한다. 기존 pyimgui 문자 콜백은 전역 `get_io()`를 써서 다른 창으로 입력을 보내고 있었다.
- `config.py` 및 새 UI 릴리스 템플릿: Robot 카메라는 Pilot의 현재 제어 대상을 따라가는 broker 대신 `/demo/robot/rgbd/frame`, `/demo/robot/rgbd/right`를 직접 구독하도록 별도 설정을 지원한다. 이 토픽은 현재 demo 시스템의 값이며 제품 전역 기본값으로 하드코딩하지 않았다.
- 화면 갱신: 카메라 최대 30Hz, 입력 중 메인 최대 60Hz, 유휴 메인 20Hz로 제한했다. `wait_events_timeout()`을 사용하므로 입력 이벤트가 오면 즉시 대기에서 깨어난다. 소프트웨어 GL이 vsync를 무시할 때 CPU를 계속 소모하던 1ms 반복 대기를 교체했다.
- 선택적 측정 로그: `ELESIM_UI_FRAME_STATS=1`일 때만 5초 간격 fps/작업시간 로그를 출력한다. 평소에는 꺼져 있다.

### 확인 범위와 수치

- 실제 GLFW/ImGui 두 컨텍스트에서 기존 문자 입력은 빈 문자열로 남는 것을 재현했다. 동일 조건의 수정본에서는 `abc123` 및 반대 창의 `camera` 입력이 통과했다.
- 최종 이미지에서 카메라 Sim/Robot 선택이 제어 대상 변경 API를 호출하지 않는 검사에 통과했다.
- 네트워크를 차단한 동일한 두 창 유휴 렌더링 검사: 기존 6.297초 동안 CPU 18.787초, 수정본 6.282초 동안 CPU 11.898초. CPU 시간 약 36.7% 감소. 이는 유휴 렌더링 비교이며 실제 영상 처리 전체나 조작 지연이 같은 비율로 개선됐다는 의미는 아니다.
- 진단 스크립트는 로봇 네트워크를 차단하고 실행했다. 로봇을 움직이는 검증은 수행하지 않았다.
- UI 변경은 소스 사본뿐 아니라 새 로컬 실행 이미지, 새 릴리스, demo 인스턴스 및 저장된 topology.json에 반영했다. Git 작업이나 원격 이슈 게시 없이 배포 디렉터리에서 직접 수정했다.
- 최종본은 자동 Robot/Sim 제어 전환 정책이나 새로운 로봇 동작 미러링을 추가하지 않았다. 기존 Sim에는 `mirror_robot_id`와 measured telemetry 미러 경로가 있으나, 이는 이번 카메라 선택 분리와 별도 기능이다.

### GPU 관련 확인

com1에서 실행 중 NVIDIA 모듈은 595.71.05, 설치된 라이브러리와 현재 커널용 디스크 모듈은 595.99.02다. `nvidia-smi`는 NVML 버전 불일치를 보고했다. 드라이버 설치/삭제/커널 모듈 교체/재부팅은 수행하지 않았다. 재부팅 후 새 모듈이 로드됐는지 확인해야 한다. UI의 `LIBGL_ALWAYS_SOFTWARE` 기본값은 여전히 1이며, GPU 디바이스와 graphics 라이브러리 연결은 별도 설정이 필요하다.

### 최종 배포 정보
```json
{
  "install_uuid": "1f95fd4e-2810-4f97-8a29-81257e8074db",
  "fingerprint": "b9393d6317102db8941512e41b53a43f87d8ccbc756a08468838c3b6b6a09d5d",
  "tag": "elesim/ui:1f95fd4e28104f978a2981257e8074db-b9393d6317102db8941512e41b53a43f87d8ccbc756a08468838c3b6b6a09d5d",
  "image_id": "sha256:2a658a02b7d9f4a2001c3fe7ac3c78370bf813815e8acfbbc4cd97bea2bcc214",
  "release_key": "837776ca261ecd75b9c4cb04fa2b22e286d82eace45adb5d988ee0bab8f14eff",
  "backup": "/home/idim/newsim/diagnostics/ui-repair-backup/20261027T153353Z"
}
```
중간 등록 릴리스 `1095889376670810a124840b04384519c409a0dd70281f2249df2931f2e8ea7a`는 최종 릴리스로 교체됐다. 최종본에 시작 시 Robot 강제 선택 설정은 없다.
### 실제 파일별 unified diff

다음 내용은 백업 파일과 현재 파일을 직접 비교한 결과다. `-`는 삭제, `+`는 추가다. 원래 코드가 남아 있는 com1 소스와 실제 native 설치 모듈을 구분해 경로를 기재했다. YAML 재직렬화로 인한 주석 제거/표현 변경도 생략하지 않았다. 전체 패치는 `diagnostics/changes-20261028.patch`에도 저장했다.

#### `/home/idim/newsim/maintenance/elesim_setup/image_cleanup.py`

```diff
--- before/maintenance/elesim_setup/image_cleanup.py
+++ after/maintenance/elesim_setup/image_cleanup.py
@@ -307,7 +307,7 @@
 
     releases = {release_key(item): item for item in list_releases(prefix, install_uuid=owner.install_uuid)}
     _reject_symlinked_ancestors(Path(boundary.compose_file))
-    current_tags = set(call("compose", "-p", project, "-f", boundary.compose_file,
+    current_tags = set(call("compose", "--profile", "*", "-p", project, "-f", boundary.compose_file,
                             "config", "--images").splitlines())
     current_ids = {}
     runtime_tags = {tag for item in releases.values() for tag in item.role_images.values()}
```

#### `/home/idim/newsim/containers/build/tools/app/elesim_setup/image_cleanup.py`

```diff
--- before/containers/build/tools/app/elesim_setup/image_cleanup.py
+++ after/containers/build/tools/app/elesim_setup/image_cleanup.py
@@ -307,7 +307,7 @@
 
     releases = {release_key(item): item for item in list_releases(prefix, install_uuid=owner.install_uuid)}
     _reject_symlinked_ancestors(Path(boundary.compose_file))
-    current_tags = set(call("compose", "-p", project, "-f", boundary.compose_file,
+    current_tags = set(call("compose", "--profile", "*", "-p", project, "-f", boundary.compose_file,
                             "config", "--images").splitlines())
     current_ids = {}
     runtime_tags = {tag for item in releases.values() for tag in item.role_images.values()}
```

#### `/home/idim/newsim/containers/build/ui/app/elesim_ui/main.py`

```diff
--- before/elesim_ui/main.py
+++ after/elesim_ui/main.py
@@ -105,19 +105,12 @@
             print(f"[ui] WebRTC unavailable: {exc}")
 
     def select_endpoint(endpoint_id: str, endpoint_role: str) -> None:
-        # Only a Sim endpoint owns the simulation session and WebRTC camera
-        # offers.  UI/Pilot discovery peers must never become camera targets.
-        #
-        # A Robot endpoint is selectable too, and has to be: the pilot serves
-        # whichever endpoint the operator picked, and with the robot excluded
-        # here the pick silently did nothing.  `active_endpoint` stayed empty
-        # against real hardware and the panel read as never connected.
+        # Only the explicit CONTROL TARGET selector changes the motion route.
+        # Camera source selection belongs to SimView and is independent.
         role = str(endpoint_role).strip().lower()
         if role not in {"sim", "robot"}:
             return
         service.select_endpoint(endpoint_id)
-        if role == "sim" and sim_session is not None:
-            sim_session.switch_target(endpoint_id)
 
     panel = ControlPanel(
         state,
```

#### `/home/idim/newsim/containers/build/ui/app/elesim_ui/config.py`

```diff
--- before/elesim_ui/config.py
+++ after/elesim_ui/config.py
@@ -99,6 +99,14 @@
             preview_right_topic,
             preview_left_topic,
         )
+    # An independent Robot view must not subscribe to the Pilot broker,
+    # whose RGB-D source follows the selected motion target.
+    robot_camera_topic = str(presentation.get("robot_camera_topic", "")).strip()
+    if robot_camera_topic:
+        robot_right_topic = str(presentation.get("robot_camera_right_topic", "")).strip()
+        if not robot_camera_topic.startswith("/") or (robot_right_topic and not robot_right_topic.startswith("/")):
+            raise ValueError(f"{source}: Robot camera topics must be absolute DDS topics")
+        preview_left_topic, preview_right_topic = robot_camera_topic, robot_right_topic
     for key in ("vendor_config", "keystore"):
         if str(dds_raw.get(key, "")).strip():
             dds_raw[key] = _resolved(source, dds_raw[key])
```

#### `/home/idim/newsim/containers/build/ui/app/elesim_ui/control_panel.py`

```diff
--- before/elesim_ui/control_panel.py
+++ after/elesim_ui/control_panel.py
@@ -1,6 +1,7 @@
 from __future__ import annotations
 
 import math
+import os
 import time
 import sys
 from typing import Callable, Optional
@@ -199,11 +200,12 @@
         # Sample pointer gestures at the display cadence.  The session and
         # Sim endpoint coalesce camera deltas, so this does not create an
         # unbounded DDS backlog while removing a visible 30 Hz input cap.
-        self._camera_draw_period_s = 1.0 / 60.0
+        self._camera_draw_period_s = 1.0 / 30.0
         self._main_imgui_context = None
         self._camera_imgui_context = None
         self._main_imgui_impl = None
         self._camera_imgui_impl = None
+        self._last_input_at = time.monotonic()
         self._ui_resolution_requested_scale = 1.0
         self._ui_resolution_scale = 1.0
         self._ui_resolution_base_w = _INITIAL_WINDOW_W
@@ -246,29 +248,91 @@
             self._respawn_shortcut_window = window
 
     def _install_respawn_shortcut(self, window, renderer) -> None:
+        # GLFW dispatches both windows' events in a single poll.  The camera
+        # context may still be current from the previous frame.  pyimgui's
+        # char_callback uses get_io(), so every callback must select the
+        # owning window's ImGui context before touching input state.
+        context = imgui.get_current_context()
+
+        def in_context(callback, *args):
+            self._last_input_at = time.monotonic()
+            previous = imgui.get_current_context()
+            self._set_imgui_context(context)
+            try:
+                return callback(*args)
+            finally:
+                self._set_imgui_context(previous)
+
         def key_callback(callback_window, key, scancode, action, mods):
-            renderer.keyboard_callback(callback_window, key, scancode, action, mods)
-            self._handle_respawn_key(
-                window, key, action, mods,
-                want_text_input=bool(renderer.io.want_text_input),
-            )
+            def handle_key():
+                if key < 0:
+                    return
+                renderer.keyboard_callback(callback_window, key, scancode, action, mods)
+                self._handle_respawn_key(
+                    window, key, action, mods,
+                    want_text_input=bool(renderer.io.want_text_input),
+                )
+            in_context(handle_key)
 
         def focus_callback(_window, focused):
+            self._last_input_at = time.monotonic()
             if not focused:
                 self._clear_respawn_shortcut()
 
         glfw.set_key_callback(window, key_callback)
+        glfw.set_char_callback(window, lambda *args: in_context(renderer.char_callback, *args))
+        glfw.set_scroll_callback(window, lambda *args: in_context(renderer.scroll_callback, *args))
+        glfw.set_cursor_pos_callback(window, lambda *args: in_context(renderer.mouse_callback, *args))
+        glfw.set_window_size_callback(window, lambda *args: in_context(renderer.resize_callback, *args))
         glfw.set_window_focus_callback(window, focus_callback)
 
+    def _refresh_endpoints(self) -> None:
+        now = time.monotonic()
+        if now - self._endpoint_cache_at < 0.5:
+            return
+        try:
+            self._endpoint_cache = list(self.service.available_endpoints)
+            self._active_endpoint_cache = str(self.service.active_endpoint)
+            self._endpoint_cache_at = now
+        except Exception:
+            return
+
+    def _draw_camera_selector(self) -> None:
+        """Choose a video source without changing Pilot's motion target."""
+        self._refresh_endpoints()
+        view = self._sim_view
+        if view is None:
+            return
+        entries = []
+        if view.zed_preview is not None or view.zed_right_preview is not None:
+            entries.append(("zed", "Robot camera"))
+        if view.session is not None:
+            entries.extend(
+                (str(e.get("endpoint_id", "")), f"Sim view: {e.get('endpoint_id', '')}")
+                for e in self._endpoint_cache
+                if isinstance(e, dict) and e.get("role") == "sim" and e.get("endpoint_id")
+            )
+            requested = view.session.snapshot.requested_sim_id
+            if requested and not any(eid == requested for eid, _ in entries):
+                entries.append((requested, f"Sim view: {requested}"))
+        if not entries:
+            imgui.text_disabled("View: waiting for camera")
+            return
+        current = "zed" if view.state.camera_source == "zed" else (
+            view.session.snapshot.requested_sim_id if view.session is not None else ""
+        )
+        index = next((i for i, (eid, _) in enumerate(entries) if eid == current), 0)
+        changed, selected = imgui.combo("View##camera-source", index, [label for _, label in entries])
+        if changed:
+            target = entries[selected][0]
+            if target == "zed":
+                view.state.camera_source = "zed"
+            else:
+                view.state.camera_source = "sim"
+                view.session.switch_target(target)
+
     def _draw_endpoint_selector(self, *, dropdown: bool = False) -> None:
-        now = time.monotonic()
-        if now - self._endpoint_cache_at >= 0.5:
-            try:
-                self._endpoint_cache = list(self.service.available_endpoints)
-                self._active_endpoint_cache = str(self.service.active_endpoint)
-                self._endpoint_cache_at = now
-            except Exception:
-                pass
+        self._refresh_endpoints()
         # The operator endpoint list also contains this UI and Pilot.  They
         # are discovery/control peers, not targets; exposing them here lets a
         # click switch the control plane to the wrong participant.
@@ -308,7 +372,7 @@
         if not endpoints:
             imgui.text_disabled("TARGET: waiting for endpoint")
             return
-        imgui.text("TARGET")
+        imgui.text("CONTROL TARGET")
         for index, endpoint in enumerate(endpoints):
             if isinstance(endpoint, dict):
                 endpoint_id = str(endpoint.get("endpoint_id", ""))
@@ -371,7 +435,7 @@
         button_width = button_size * 2.0
         imgui.push_item_width(max(80.0, imgui.get_content_region_available_width()
                                   - button_width - spacing * 2.0))
-        self._draw_endpoint_selector(dropdown=True)
+        self._draw_camera_selector()
         imgui.pop_item_width()
         imgui.same_line()
         maximized = self._camera_window is not None and bool(
@@ -475,6 +539,8 @@
         imgui.render()
         impl.render(imgui.get_draw_data())
         glfw.swap_buffers(window)
+        glfw.make_context_current(self._glfw_window)
+        self._set_imgui_context(self._main_imgui_context)
         if isinstance(opened, tuple) and len(opened) > 1 and not opened[1]:
             self._hide_camera_window()
 
@@ -1072,7 +1138,12 @@
                 self._set_imgui_context(self._main_imgui_context)
 
         try:
+            report_frames = os.environ.get("ELESIM_UI_FRAME_STATS") == "1"
+            report_started = time.monotonic()
+            report_count = 0
+            report_work_s = 0.0
             while not glfw.window_should_close(window) and not self._stop:
+                frame_started = time.monotonic()
                 self.service.poll()
                 self._host_state = self.service.current_host_state()
                 self.sync_offset_drafts()
@@ -1092,9 +1163,23 @@
                 main_impl.render(imgui.get_draw_data())
                 glfw.swap_buffers(window)
                 self._draw_camera_window()
-                # Keep a tiny fallback yield for drivers that ignore the swap
-                # interval; normal composited paths block in swap_buffers.
-                time.sleep(0.001)
+                if report_frames:
+                    report_count += 1
+                    report_work_s += time.monotonic() - frame_started
+                    elapsed = time.monotonic() - report_started
+                    if elapsed >= 5.0:
+                        print(f"[ui-frame] fps={report_count / elapsed:.1f} work_ms={1000.0 * report_work_s / report_count:.1f}", flush=True)
+                        report_started = time.monotonic()
+                        report_count = 0
+                        report_work_s = 0.0
+                # Software GL and remote desktops can ignore swap_interval.
+                # Bound redraws instead of saturating CPU with duplicate frames;
+                # input events wake this wait immediately.
+                interactive = time.monotonic() - self._last_input_at < 0.25 or any(imgui.get_io().mouse_down)
+                target_hz = 60.0 if interactive else 20.0
+                remaining = 1.0 / target_hz - (time.monotonic() - frame_started)
+                if remaining > 0.0:
+                    glfw.wait_events_timeout(remaining)
         finally:
             if self._camera_window is not None:
                 try:
```

#### `UI release template: config/ui/default.yaml`

```diff
--- before/config/ui/default.yaml
+++ after/config/ui/default.yaml
@@ -27,9 +27,11 @@
   keystore: ""
   enclave: ""
 presentation:
+  robot_camera_topic: /demo/robot/rgbd/frame
+  robot_camera_right_topic: /demo/robot/rgbd/right
   use_hardware: false
   use_go2: true
-  show_zed_preview: false
+  show_zed_preview: true
   zed_swap_left_right: false
   go2:
     vx_mps: 0.8
```

#### `/home/idim/newsim-robot/tools/venv/lib/python3.10/site-packages/elesim_setup/installer.py`

```diff
--- before/tools/venv/lib/python3.10/site-packages/elesim_setup/installer.py
+++ after/tools/venv/lib/python3.10/site-packages/elesim_setup/installer.py
@@ -558,7 +558,7 @@
         python = path / "bin/python"
         if not python.is_file():
             self.log(f"[venv] {path}")
-            arguments = [sys.executable, "-m", "venv"]
+            arguments = [str(Path(getattr(sys, "_base_executable", sys.executable)).resolve()), "-m", "venv"]
             if system_site_packages:
                 arguments.append("--system-site-packages")
             arguments.append(str(path))
```

#### `/home/idim/newsim-robot/tools/venv/lib/python3.10/site-packages/elesim_setup/configuration.py`

```diff
--- before/tools/venv/lib/python3.10/site-packages/elesim_setup/configuration.py
+++ after/tools/venv/lib/python3.10/site-packages/elesim_setup/configuration.py
@@ -504,6 +504,9 @@
     ) as handle:
         handle.write(rendered)
         temporary = Path(handle.name)
+    if path.exists():
+        shutil.copystat(path, temporary)
+        os.utime(temporary, None)
     temporary.replace(path)
 
 
```

#### `/home/idim/newsim-robot/apps/robot/config/default.yaml`

```diff
--- before/apps/robot/config/default.yaml
+++ after/apps/robot/config/default.yaml
@@ -2,8 +2,7 @@
 runtime:
   endpoint_id: robot-go2
   device: /dev/ttyUSB0
-  # Empty disables Teensy acquisition. Prefer a stable /dev/serial/by-id/... path.
-  imu_device: ""
+  imu_device: ''
   use_go2: true
 rgbd:
   schema_version: 1
@@ -23,15 +22,23 @@
   discovery_mode: multicast
   static_peers: []
   network_interface: tailscale0
-  vendor_config: ""
+  vendor_config: ''
   security_profile: trusted-network
-  keystore: ""
-  enclave: ""
+  keystore: ''
+  enclave: ''
 arm:
   linear_u_max_deg: 259.0
   linear_u_limit_deg: 259.0
-  command_direction: [1, 1, 1, -1]
-  motor_direction: [1, -1, 1, -1]
+  command_direction:
+  - 1
+  - 1
+  - 1
+  - -1
+  motor_direction:
+  - 1
+  - -1
+  - 1
+  - -1
   baudrate: 1000000
   current_limit_ma: 2500
   claw_contact_load_permille: 250
@@ -42,7 +49,7 @@
   profile_vel_seg2: 60
   profile_acc_seg2: 6
   profile_vel_claw: 20
-  max_linear_m_s: 0.20
+  max_linear_m_s: 0.2
   max_roll_rad_s: 3.5
   max_bend_rad_s: 1.0
   claw_open_deg: 340.0
@@ -50,8 +57,7 @@
 mapping:
   linear_u_max: 259.0
   linear_u_limit: 259.0
-  # oct27 calibration: motor/display 0..259 degrees maps to q=0..-230 mm.
-  linear_q_min_m: -0.230
+  linear_q_min_m: -0.23
   linear_q_max_m: 0.0
   roll_q_min_rad: -1.5707963267948966
   roll_q_max_rad: 1.5707963267948966
@@ -63,11 +69,11 @@
   enabled: true
   backend: unitree_ros2
   pose_source: sportmodestate
-  ros_workspace: /opt/unitree_ros2
-  network_interface: eth0
+  ros_workspace: /home/idim/ros2_ws
+  network_interface: eno1
   ros_domain_id: 1
   ipc_socket_path: /run/elesim-unitree/bridge.sock
-  ipc_robot_user: elesim
+  ipc_robot_user: idim
   ipc_bridge_user: elesim-unitree
   ipc_heartbeat_interval_s: 0.1
 camera:
```

#### `/home/idim/newsim-robot/apps/robot/config/installed.yaml`

```diff
--- before/apps/robot/config/installed.yaml
+++ after/apps/robot/config/installed.yaml
@@ -76,11 +76,11 @@
   enabled: true
   backend: unitree_ros2
   pose_source: sportmodestate
-  ros_workspace: /opt/unitree_ros2
-  network_interface: eth0
+  ros_workspace: /home/idim/ros2_ws
+  network_interface: eno1
   ros_domain_id: 1
   ipc_socket_path: /run/elesim-unitree/bridge.sock
-  ipc_robot_user: elesim
+  ipc_robot_user: idim
   ipc_bridge_user: elesim-unitree
   ipc_heartbeat_interval_s: 0.1
 camera:
```

#### `/home/idim/newsim-robot/apps/robot/venv/pyvenv.cfg`

```diff
--- before/apps/robot/venv/pyvenv.cfg
+++ after/apps/robot/venv/pyvenv.cfg
@@ -1,3 +1,3 @@
-home = /home/idim/.cache/elesim/setup/environments-v2/app-5abe687bed8940c8634e2895d369f23663c098a6f7d752335c4482cdce4cdc32/env-8fzpsjfl/bin
+home = /usr/bin
 include-system-site-packages = true
 version = 3.10.12
```

#### 심볼릭 링크 `/home/idim/newsim-robot/apps/robot/venv/bin/python`

```diff
--- before//home/idim/newsim-robot/apps/robot/venv/bin/python
+++ after//home/idim/newsim-robot/apps/robot/venv/bin/python
@@ -1 +1 @@
-/home/idim/.cache/elesim/setup/environments-v2/app-5abe687bed8940c8634e2895d369f23663c098a6f7d752335c4482cdce4cdc32/env-8fzpsjfl/bin/python
+/usr/bin/python3.10
```

#### ACL `/home/idim/newsim-robot/apps/robot/config/default.yaml`

실행한 변경: `setfacl -m u:elesim-unitree:r <해당 파일>`

```text
# file: /home/idim/newsim-robot/apps/robot/config/default.yaml
# owner: idim
# group: idim
user::rw-
user:elesim-unitree:r--
group::rw-
mask::rw-
other::r--

```

#### ACL `/home/idim/newsim-robot/apps/robot/config/installed.yaml`

실행한 변경: `setfacl -m u:elesim-unitree:r <해당 파일>`

```text
# file: /home/idim/newsim-robot/apps/robot/config/installed.yaml
# owner: idim
# group: idim
user::rw-
user:elesim-unitree:r--
group::---
mask::r--
other::---

```

#### 최종 등록 파일 `connections/demo/topology.json`

```diff
--- before/connections/demo/topology.json
+++ after/connections/demo/topology.json
@@ -25,7 +25,7 @@
             {
               "role": "ui",
               "endpoint_id": "ui",
-              "release_key": "73a63a8d8bdca6d650431ca49fb956f1a9e7d37b18354c42ec35f27555d9f6f4"
+              "release_key": "837776ca261ecd75b9c4cb04fa2b22e286d82eace45adb5d988ee0bab8f14eff"
             }
           ],
           "install_mode": "container",
@@ -34,14 +34,14 @@
           "lifecycle": "compose",
           "install_uuid": "1f95fd4e-2810-4f97-8a29-81257e8074db",
           "project": "elesim-clear",
-          "release_key": "73a63a8d8bdca6d650431ca49fb956f1a9e7d37b18354c42ec35f27555d9f6f4"
+          "release_key": "837776ca261ecd75b9c4cb04fa2b22e286d82eace45adb5d988ee0bab8f14eff"
         }
       ],
       "assignments": [
         {
           "role": "ui",
           "endpoint_id": "ui",
-          "release_key": "73a63a8d8bdca6d650431ca49fb956f1a9e7d37b18354c42ec35f27555d9f6f4"
+          "release_key": "837776ca261ecd75b9c4cb04fa2b22e286d82eace45adb5d988ee0bab8f14eff"
         }
       ],
       "install_mode": "container",
```

#### 최종 등록 파일 `instances/demo/state.json`

```diff
--- before/instances/demo/state.json
+++ after/instances/demo/state.json
@@ -12,7 +12,7 @@
     }
   ],
   "interface": "tailscale0",
-  "release_key": "73a63a8d8bdca6d650431ca49fb956f1a9e7d37b18354c42ec35f27555d9f6f4",
+  "release_key": "837776ca261ecd75b9c4cb04fa2b22e286d82eace45adb5d988ee0bab8f14eff",
   "rmw_implementation": "rmw_cyclonedds_cpp",
   "role_ids": {
     "pilot": "pilot",
```

#### 최종 등록 파일 `containers/compose.instances.yaml`

```diff
--- before/containers/compose.instances.yaml
+++ after/containers/compose.instances.yaml
@@ -1,7 +1,7 @@
 name: elesim-clear
 services:
   svc-demo-ui-9c64dc6334556735eae687002d99bbb60997b67b4ef1a1a55b2fd6629893ff53:
-    image: sha256:f6ed96e4515f76764e1f4c7c7a17a86fdcc7e2d1e93c3ed58a4b41eac98d2256
+    image: sha256:2a658a02b7d9f4a2001c3fe7ac3c78370bf813815e8acfbbc4cd97bea2bcc214
     container_name: elesim-clear-demo-ui
     network_mode: host
     labels:
@@ -35,5 +35,5 @@
       LIBGL_ALWAYS_SOFTWARE: ${ELESIM_UI_SOFTWARE_GL:-1}
     volumes:
     - /home/idim/newsim/instances/demo/endpoints/ui/config:/opt/elesim/config:ro
-    - /home/idim/newsim/releases/73a63a8d8bdca6d650431ca49fb956f1a9e7d37b18354c42ec35f27555d9f6f4/data/data:/opt/elesim/data:ro
+    - /home/idim/newsim/releases/837776ca261ecd75b9c4cb04fa2b22e286d82eace45adb5d988ee0bab8f14eff/data/data:/opt/elesim/data:ro
     - /tmp/.X11-unix:/tmp/.X11-unix:rw
```

### 세 호스트의 이미지 정리 수정 범위

앞의 `image_cleanup.py` diff는 com1의 백업 원본과 수정본이다. 같은 `current_tags` 변경을 com2 `/home/hckang/newsim`, com3 `/home/idim/newsim`의 `maintenance/elesim_setup/image_cleanup.py`와 `containers/build/tools/app/elesim_setup/image_cleanup.py`에도 적용했다. remote native `installer.py`/`configuration.py` 수정은 com3 `/home/idim/newsim-robot/tools/venv/lib/python3.10/site-packages/elesim_setup/`에만 적용했으며 com1 동명 소스에 적용한 것으로 기재하지 않는다.

### 새로 추가한 로컬 파일

아래 파일은 모두 이 작업 중 새로 추가한 파일이며 기존 원본에서 삭제된 내용은 없다. 실행/진단 스크립트의 전체 추가 코드는 unified patch에 포함했다.
- `diagnostics/collect-startup.py`
- `diagnostics/repair-com3.py`
- `diagnostics/check-ui-input.py`
- `diagnostics/check-ui-view-selection.py`
- `diagnostics/measure-ui-render.py`
- `diagnostics/ui-hotfix-20261028/Dockerfile`
- `diagnostics/ui-hotfix-20261028/apply-release.py`

추가 기록/데이터 파일: `diagnostics/native-repair-diffs.json`, `diagnostics/local-repair-diffs.json`, `diagnostics/ui-hotfix-20261028/build.json`, `diagnostics/ui-hotfix-20261028/intermediate-build.json`, 새 UI 릴리스의 `manifest.json` 및 `data/`, `diagnostics/ui-hotfix-20261028/snapshot/`. `snapshot/`은 기존 릴리스의 18MB 데이터 사본이며 코드 전체를 새로 작성한 것은 아니다. 실제 템플릿 변경은 위 diff에 포함했다. 새 manifest는 기존 manifest를 덮어쓰지 않고 content-addressed 릴리스 경로에 생성했다. 등록 관리자가 `install-ownership.json`의 인스턴스 소유권 기록도 갱신했으며 제품 소스 코드 변경과는 구분한다.

### 운영상 최종 인계

Connections 화면이 이전 릴리스 선택을 캐시하고 있을 수 있으므로 다음 시작 전 페이지를 새로고침해 저장된 topology.json을 다시 읽어야 한다. 검증 중 시작한 UI는 인계 시 종료해 기존 Connections의 '이미 실행 중' 검사와 충돌하지 않도록 한다. 실제 하드웨어 조작과 영상 지연의 체감 확인은 사용자가 진행한다.


## 2026-10-28 후속 적용: UI NVIDIA GPU 가속 활성화

이 항목은 앞의 “UI는 CPU 렌더링”이라는 당시 상태를 갱신하는 후속 기록이다.

- 재부팅 후 실행 중 NVIDIA 모듈과 라이브러리는 모두 `595.99.02`이며 `nvidia-smi`가 정상 동작한다. CUDA toolkit 또는 드라이버 패키지를 이번 작업에서 설치·삭제·다운그레이드하지 않았다.
- 기존 로컬 UI 이미지의 실제 GLFW/OpenGL 창을 GPU 연결로 검사했고, `GL_VENDOR=NVIDIA Corporation`, `GL_RENDERER=NVIDIA GeForce GTX 1660 Ti/PCIe/SSE2`, `GL_VERSION=4.6.0 NVIDIA 595.99.02`를 확인했다.
- 호스트 사용자 `1000:1000`으로도 같은 렌더러가 확인되어 GPU UI를 이 계정으로 실행한다. 현재 데스크톱의 `SI:localuser:idim` 허용을 사용하며 이 활성화를 위해 root X11 허용을 추가하지 않았다.
- 설치별 정책 `maintenance/ui-rendering.json`에 `backend: nvidia`를 저장했다. UI 생성 코드가 이 파일을 읽으므로 Connections가 인스턴스를 재등록해도 설정을 다시 적용한다.
- 생성된 UI 서비스에는 `gpus: all`, `LIBGL_ALWAYS_SOFTWARE=0`, `NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility`, `__GLX_VENDOR_LIBRARY_NAME=nvidia`, `__NV_PRIME_RENDER_OFFLOAD=1`, `user: 1000:1000`, `HOME=/tmp`가 들어간다.
- `container_installer.py`를 직접 수정하고 `elesim/tools:clear-shark`를 다시 빌드했다. UI 릴리스 키와 UI 코드 이미지는 변경하지 않았으며, 기존 인스턴스 설정을 관리자의 정상 replace 경로로 재생성했다.
- 일회성 임의 설정 검사뿐 아니라 **실제 저장된 Compose 서비스**로도 `UI_RENDERER=NVIDIA GeForce GTX 1660 Ti/PCIe/SSE2`를 확인한 뒤 UI를 시작했다.
- 현재 설정은 해당 노트북의 NVIDIA GPU 전체를 UI에 연결한다. 개별 GPU 선택 및 소프트웨어 렌더링으로 전환하는 GUI는 아직 추가하지 않았다.

**connection manager에 UI gpu 고르는 것도 넣어야 할 것으로 보임.**

정확한 변경 패치는 `diagnostics/ui-gpu-20261028.patch`, 원본 백업은 `diagnostics/ui-gpu-backup/`에 보관했다. 아래 diff는 그 원본과 현재 파일의 직접 비교다.

### `containers/build/tools/app/elesim_setup/container_installer.py`

```diff
--- before/containers/build/tools/app/elesim_setup/container_installer.py
+++ after/containers/build/tools/app/elesim_setup/container_installer.py
@@ -1514,6 +1514,28 @@
             service["environment"]["LIBGL_ALWAYS_SOFTWARE"] = (  # type: ignore[index]
                 "${ELESIM_UI_SOFTWARE_GL:-1}"
             )
+            # Installation-local graphics policy is independent of Pilot/Sim
+            # CUDA compute policy. Keep it across instance registrations.
+            rendering_path = self.state.prefix_path / "maintenance/ui-rendering.json"
+            if rendering_path.is_symlink():
+                raise ValueError("UI rendering policy must not be a symlink")
+            if rendering_path.is_file():
+                rendering = json.loads(rendering_path.read_text(encoding="utf-8"))
+                if not isinstance(rendering, dict) or set(rendering) != {"backend"}:
+                    raise ValueError("UI rendering policy requires only backend")
+                backend = rendering["backend"]
+                if backend not in {"software", "nvidia"}:
+                    raise ValueError("UI rendering backend must be software or nvidia")
+                if backend == "nvidia":
+                    service["gpus"] = "all"
+                    service["user"] = f"{os.getuid()}:{os.getgid()}"
+                    service["environment"].update({
+                        "LIBGL_ALWAYS_SOFTWARE": "0",
+                        "NVIDIA_DRIVER_CAPABILITIES": "graphics,display,utility",
+                        "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
+                        "__NV_PRIME_RENDER_OFFLOAD": "1",
+                        "HOME": "/tmp",
+                    })
         if role in {"pilot", "sim"}:
             self._apply_compute(service, compute=compute)
             if role == "sim" and "NVIDIA_DRIVER_CAPABILITIES" in service["environment"]:
```

### `containers/compose.instances.yaml`

```diff
--- before/containers/compose.instances.yaml
+++ after/containers/compose.instances.yaml
@@ -32,8 +32,14 @@
       ROS_SECURITY_KEYSTORE: ''
       ELESIM_DDS_ENCLAVE: ''
       DISPLAY: ${DISPLAY:-:0}
-      LIBGL_ALWAYS_SOFTWARE: ${ELESIM_UI_SOFTWARE_GL:-1}
+      LIBGL_ALWAYS_SOFTWARE: '0'
+      NVIDIA_DRIVER_CAPABILITIES: graphics,display,utility
+      __GLX_VENDOR_LIBRARY_NAME: nvidia
+      __NV_PRIME_RENDER_OFFLOAD: '1'
+      HOME: /tmp
     volumes:
     - /home/idim/newsim/instances/demo/endpoints/ui/config:/opt/elesim/config:ro
     - /home/idim/newsim/releases/837776ca261ecd75b9c4cb04fa2b22e286d82eace45adb5d988ee0bab8f14eff/data/data:/opt/elesim/data:ro
     - /tmp/.X11-unix:/tmp/.X11-unix:rw
+    gpus: all
+    user: 1000:1000
```

### `maintenance/ui-rendering.json`

```diff
--- /dev/null
+++ maintenance/ui-rendering.json
@@ -0,0 +1,3 @@
+{
+  "backend": "nvidia"
+}
```

### `diagnostics/apply-ui-gpu.py`

```diff
--- /dev/null
+++ diagnostics/apply-ui-gpu.py
@@ -0,0 +1,12 @@
+"""Regenerate the existing UI instance using the installation GPU policy."""
+from pathlib import Path
+import json
+from elesim_setup.cli import _load_instance_runtime
+from elesim_setup.instances import InstanceState
+
+root = Path('/home/idim/newsim')
+state, owner, manager = _load_instance_runtime(root / 'install-state.json')
+current = InstanceState.from_dict(json.loads((root / 'instances/demo/state.json').read_text()))
+assert tuple(e.role for e in current.endpoints) == ('ui',)
+manager.replace(current, robot_id='robot')
+print('UI regenerated with NVIDIA graphics; release and topology unchanged')
```

GPU 적용 후 실제 UI는 `running`, 사용자 `1000:1000`, GPU 장치 요청 활성 상태였으며 유휴 시 Docker CPU 사용량은 약 27.34%였다. 이는 영상/제어 노드가 꺼진 상태의 참고 수치다. 검증용 UI는 Connections 전체 시작의 중복 실행 검사와 충돌하지 않도록 종료했다. 다음 Connections 시작에서 저장된 GPU 정책으로 다시 실행된다.


## 2026-10-28 UI 이미지 동물 이름 태그 적용

사용자 요청에 따라 해시 형식으로 만든 UI 이미지에 기존 `clear-동물` 형식의 이름을 붙였다. 이미지 바이트/ID는 변경하지 않고, 새 태그를 담은 별도 불변 릴리스를 게시한 뒤 최종 UI의 저장된 topology와 인스턴스를 그 릴리스로 바꿨다.

| 용도 | 새 태그 | 비고 |
|---|---|---|
| 현재 최종 UI | `elesim/ui:clear-otter` | 텍스트 입력·카메라/제어 분리·렌더링 최적화 적용본. GPU는 설치별 정책으로 활성화됨. |
| 중간 검증 UI | `elesim/ui:clear-marmot` | 최종 설명을 반영하기 전 중간본. 평상시 실행에 사용하지 않음. |
| 수정한 관리 도구 | `elesim/tools:clear-shark` | 이미 동물 이름이므로 기존 이름 유지. GPU 정책 생성 코드가 포함됨. |

최종 UI의 이전 태그: `elesim/ui:1f95fd4e28104f978a2981257e8074db-b9393d6317102db8941512e41b53a43f87d8ccbc756a08468838c3b6b6a09d5d`

최종 UI의 새 릴리스: `769738ed05750f9178b789668bbfd6b724b4c1d15709dbcbe6c8ab79f0f25279`

중간 UI의 이전 태그: `elesim/ui:1f95fd4e28104f978a2981257e8074db-a31676a40f32564ac62fe1453b799ac9b033547bf6b44d6b86c3a38389e310c3`

중간 UI의 새 릴리스: `328a07dd434c29fe5f30dd4a1abb5911b6555c22fbfb0f080b140a714a2d7290`

기존 해시 태그와 이전 불변 릴리스는 롤백 참조를 깨뜨리지 않도록 남겼다. 기존 `clear-dolphin`은 수정 전 UI이며 새 최종본으로 덮어쓰지 않았다. 컨테이너 이름 `elesim-clear-demo-ui`와 사용자가 실행하는 Connections/instance 명령은 동일하다.

수행한 Docker 변경은 각 이미지 ID에 대한 `docker tag <image-id> <new-tag>`다. 릴리스는 `ReleaseManifest.role_images.ui`를 새 태그로 바꿔 `publish_release()`로 별도 생성했다. 원본 build 메타데이터는 `diagnostics/tag-rename-backup/`에 백업했고, 결과 전체는 `diagnostics/tag-rename-backup/rename-result.json`에 남겼다.


## 2026-10-28 태그 변경 후 Connections 목록 누락 보정

태그와 topology/instance 릴리스 갱신 후 `maintenance/available-releases.json`이 이전 세 릴리스만 유지해, 새 동물 이름 릴리스가 Connections 설치 조회 목록에 나오지 않았다. 표시 코드는 이미 `named_image_parts()`로 동물 이름을 지원한다. 원본 목록을 백업하고 `available_releases()`로 Docker 태그/이미지 ID를 검증한 결과를 `_record_available_releases()`로 기록했다. 이미지 삭제나 런타임 재시작은 하지 않았다.

이전 해시 릴리스는 롤백용으로 남아 있어 목록에 함께 나올 수 있지만, 현재 topology/instance가 선택한 최종 UI는 `clear-otter` 릴리스다. 브라우저의 이전 설치 조회 결과는 설치 조회를 다시 하거나 페이지를 새로고침해야 갱신된다.

```diff
--- before/maintenance/available-releases.json
+++ after/maintenance/available-releases.json
@@ -1 +1 @@
-{"engine_id": "e84052b3-a911-4be6-8ef8-84bb9d66ab16", "install_uuid": "1f95fd4e-2810-4f97-8a29-81257e8074db", "release_keys": ["1095889376670810a124840b04384519c409a0dd70281f2249df2931f2e8ea7a", "73a63a8d8bdca6d650431ca49fb956f1a9e7d37b18354c42ec35f27555d9f6f4", "837776ca261ecd75b9c4cb04fa2b22e286d82eace45adb5d988ee0bab8f14eff"], "schema_version": 1}
+{"engine_id": "e84052b3-a911-4be6-8ef8-84bb9d66ab16", "install_uuid": "1f95fd4e-2810-4f97-8a29-81257e8074db", "release_keys": ["1095889376670810a124840b04384519c409a0dd70281f2249df2931f2e8ea7a", "328a07dd434c29fe5f30dd4a1abb5911b6555c22fbfb0f080b140a714a2d7290", "73a63a8d8bdca6d650431ca49fb956f1a9e7d37b18354c42ec35f27555d9f6f4", "769738ed05750f9178b789668bbfd6b724b4c1d15709dbcbe6c8ab79f0f25279", "837776ca261ecd75b9c4cb04fa2b22e286d82eace45adb5d988ee0bab8f14eff"], "schema_version": 1}
```


## 2026-10-28 01:02 KST Robot 제어 상실 로그 확인

사용자 요청으로 Jetson `elesim logs demo` 및 Robot/브리지 저널을 읽었다. 이 설치의 logs 래퍼는 `--tail`을 받지 않으므로 기본 명령을 timeout으로 한정해 조회했고, 세부 증거는 Docker 로그와 journald에서 따로 저장했다.

- Robot 및 Unitree bridge는 `active/running`, `NRestarts=0`이었다.
- 00:59:50~51에 Jetson에서 UI/Sim DDS peer가 lost가 됐고 약 01:01:56부터 다시 발견됐다.
- 01:01:59 Pilot에서 Robot target lost, 01:02:00~02 Robot에서 `renew_target ... no_active_lease`를 기록했다.
- 01:02:03에 Robot을 다시 선택해 새 lease를 받았다. 이후 `apply_partial_control_u ok`와 Robot ACK가 확인됐다.
- 별도 UI 진단 peer로 읽기 전용 snapshot을 조회했다. 현재 target=robot, connected=true, torque_enabled=true, safety_fault 빈 문자열, reply_ok=true, reply_reason=target, 상태 나이 약 0.156초였다. 실제 물리적 움직임을 테스트하거나 명령·토크·서비스 설정을 변경하지 않았다.
- DDS/네트워크 심장 신호가 끊긴 근본 원인은 이 로그 확인만으로 확정하지 않았다. 로그로 확인된 직접 증상은 peer 연결 단절 및 제어권 lease 만료/재획득이다.

증거: `diagnostics/control-loss-20261028/pilot.log`, `robot-bridge.log`, `summary.json`.


## 2026-10-28 모터 목표 유지·Robot 종속 Sim·oct27 RL 정책 복구

사용자 요청으로 설치 파일을 직접 수정했다. Git checkout은 만들지 않았다. 모터/토크/grasp 명령을 보내는 실제 동작 시험은 하지 않았다. 사용자가 TDD 중단을 요청한 뒤 테스트 추가·재실행 없이 배포와 기동/Connections 감독만 수행했다.

### 원인과 변경

1. **한 축 조작 시 나머지 축 처짐:** Pilot `refresh_host_state()`가 UI snapshot마다 측정 q를 목표 `PanelState`에 복사했고, `apply_partial_control_u()`가 그 값을 이용해 네 축 목표를 다시 만들었다. 명령 필드가 빠진 것이 아니라 처진 실측 위치를 새 목표로 계속 채택하는 문제였다. `pick/client.py`가 마지막으로 보낸 네 축 목표를 보존하고, `pick/actions.py`는 목표 상태와 측정 HostState를 분리한다. 토크 전환·연결 경로/lease 변경 시 목표를 폐기하고 실측 위치로 다시 초기화한다. cuRobo 직접 전송도 목표 캐시에 반영한다. 실제 피드백 HostState는 그대로 유지한다.
2. **조종 대상과 Sim:** Pilot이 Robot을 발견하면 Robot만 조종 대상으로 허용한다. 발견 전 로봇이 없으면 Sim을 고른다. 한 번 발견한 Robot은 통신이 끊겨도 목록에 남겨 조작이 자동으로 Sim으로 넘어가지 않도록 한다. UI 카메라 선택은 계속 독립적이며 전체 Sim 목록을 유지한다. `control_selectable`은 조종 대상 표시만 제한한다.
3. **선택한 로봇을 Sim에 표시:** Pilot→Sim의 읽기 전용 `presentation_target` 메시지에 Robot ID와 boot ID를 보낸다. Sim은 Robot이 보내는 실측 `telemetry`만 읽어 표시한다. 선택하지 않은 Robot, 이전 boot, 역순 패킷은 무시한다. Sim은 mirror capability를 항상 광고하고 Robot 추종 중 motion 명령을 거절한다. GO2 제어기는 실측 미러가 활성화되면 MPC 대신 표시용 미러 모드로 전환한다. Robot이 실제로 제공하지 않는 GO2 pose/leg 값까지 생성하거나 측정됐다고 가장하지 않는다.
4. **Wrap grasp:** 실제 Jetson 로그에 `FileNotFoundError: Could not find the exported policy`가 있었다. 기존 배포 data 폴더에는 README만 있었다. 요청한 [jpyaaa3/elesim oct27](https://github.com/jpyaaa3/elesim/tree/oct27)의 HEAD `1a8913825c45112fe9c50a9bca63da8e7cdb2655`에서 `jetson/elesim/roles/pilot/config/policy/policy.pt`와 `interface.json`을 가져왔다. interface에 기록된 학습 체크포인트는 `stage3_ab_curr/model_best.pt`, 학습 git_commit은 `e65409e778cead5bf32b29440adbfaf470de6bc0`다. 브랜치의 `wrap_grasp.yaml`도 반영하되 이전 호스트 절대경로를 `/opt/elesim/data/policies/wrap-grasp/{policy.pt,interface.json}`으로 바꿨다. object radius=0.05m 등 브랜치 설정은 보존한다.

### 반영 위치와 실행 이미지

- 로컬: `containers/build/pilot/app/elesim_pilot/{connection.py,main.py,pick/client.py,pick/actions.py}`, `containers/build/sim/app/elesim_sim/{endpoint.py,runtime.py}`, `containers/build/ui/app/elesim_ui/control_panel.py`.
- 프로토콜: `containers/build/{pilot,sim,ui,tools}/protocol/elesim_protocol/{messages.py,contracts.py,payloads.py}`. `presentation_target` 종류·필드·방향 계약을 추가했다. wire carrier/ROS interface는 그대로다. tools 소스 변경은 로컬의 이후 tools 재빌드에 포함된다.
- 각 runtime 호스트의 해당 role build source에도 원본 해시 대조 후 변경분을 복사했다. 실행 코드는 새 Docker 파생 이미지에 포함된다.
- 기존 릴리스 data를 새 디렉터리에 복사해 기존 설치 config를 보존한 새 불변 릴리스를 게시했다. Pilot 새 릴리스에는 `data/data/policies/wrap-grasp/{policy.pt,interface.json}` 및 `data/config/pilot/wrap_grasp.yaml`을 추가했다. 기존 불변 릴리스는 변경하지 않았다.
- 각 호스트의 `instances/demo/state.json`, `containers/compose.instances.yaml`, 소유권 메타데이터를 기존 instance manager로 갱신했다. `maintenance/available-releases.json`을 실제 Docker 이미지와 대조해 갱신했다. com1의 `connections/demo/topology.json`에서 com1/UI, com2/Sim, com3/Pilot의 unit/assignment release_key를 바꿨다. native Robot unit의 release_key는 바꾸지 않았다.

| 역할 | 현재 태그 | 새 릴리스 |
|---|---|---|
| ui | `elesim/ui:clear-lynx` | `45a2c37eb1d2fd94ea423ca525e8a2692b932cbebd7bc4b442b82e367202374f` |
| pilot | `elesim/pilot:pale-badger` | `5f7d845425a538ac8b64851a9207b79f87ce3dc5fb834bfd925d7c02459e4369` |
| sim | `elesim/sim:dawn-otter` | `df51144a6029bbbd585958f8bfff1798440e209338bda7f3ba4a2f5b97c0e6ea` |

이미지 ID와 이전 릴리스, fingerprint는 `diagnostics/robot-sim-repair/deployed-releases.json`에 기록했다. 호스트별 원본 instance/compose/ownership/config 및 변경 build source 백업은 각 호스트 `newsim/diagnostics/control-repair-20261028/backup/`에 있다. 소스 전체 unified diff는 `diagnostics/robot-sim-repair/changes.patch`다.

### 수행한 확인과 한계

- 모터 목표 유지 경로를 전송 함수 대신 메모리 리스트를 사용해 재현했다. 변경 전 첫 목표 linear=-0.05가 피드백 sag=-0.06 이후 다음 한 축 명령에서 -0.06으로 바뀌었다. 변경 후 -0.05로 유지됐고 다른 두 미조작 축도 유지됐다. 네 축 명령이며 실제 피드백은 별도로 남는다. 결과 `motor-hold-repair/{before-result,after-result}.txt`, `robot-sim-repair/hold-result.txt`.
- fake endpoint로 Sim-only 선택, Robot-only 제한, 복수 Robot 선택, 연결 상실 시 Sim으로 자동 전환하지 않음, 선택된 Robot 실측 반영, 역순/다른 Robot 무시, 종속 Sim 명령 거절을 확인했다. 결과 `robot-sim-repair/routing-result.txt`. 실제 DDS/hardware로 동작을 시험한 결과는 아니다.
- 실제 Jetson 컨테이너에서 별도 Python 프로세스로 TorchScript를 CPU 로딩하고 기존 Pilot mapping 범위와 호환성 및 첫 추론을 확인했다. obs_dim=16, action_dim=5, 네 축 출력 finite. 실제 grasp는 실행하지 않았다. 결과 `robot-sim-repair/policy-result.txt`.
- 재기동 중 Jetson Pilot에서 CSV NVIDIA hook 오류가 발생했다. 원인은 수동으로 tools 컨테이너를 호출할 때 outer-host L4T 환경을 전달하지 않아 generated Compose의 `runtime: nvidia`가 사라진 것이었다. 실제 Jetson 호스트에서 같은 instance manager를 호출해 해당 설정을 복구했고 Pilot은 runtime=nvidia로 기동했다. 배포 보조 스크립트에도 `ELESIM_HOST_JETSON`/`ELESIM_HOST_L4T_RELEASE` 전달을 추가했다. 기본 제품 GPU 생성 함수에는 이미 이 지원이 있었다.
- UI·Pilot·Sim 새 이미지 기동은 확인했다. 당시 Robot/bridge는 01:22:23에 정상 종료돼 inactive였으므로 실제 Robot→Sim 미러 수신은 아직 입증하지 않았다. 사용자 Connections 시작을 감독하며 확인할 항목이다.
- 제가 기동한 UI·Pilot·Sim이 Connections 전체 시작의 '모든 기존 인스턴스 정지' 선행 조건과 충돌할 수 있어 사용자 시작에 넘기기 위해 정지 정리를 수행했다. Connections `start`는 기존 인스턴스가 running이면 등록부터 거절한다. `stop all roles on <host> before starting` 코드 경로를 확인했다.

### Hardware 표시 정적 분석: 추가로 확인됐지만 이 변경에 포함하지 않은 결함

- Robot `runtime.state()`는 `motor_positions_raw`, `motor_positions_deg`, `motor_currents_ma`를 문자열 모터 ID `"1".."5"`로 발행한다.
- Pilot `RemoteState.snapshot()` → `HostState` → operator `current_host_state` snapshot → UI `RemoteControlService.current_host_state()` 경로는 이 필드를 보존한다. 이번 목표 유지 수정은 HostState 실측값을 지우지 않는다.
- UI `panels/hardware.py:_draw_motor_position_rows()`는 ID와 축 이름을 모두 읽으므로 위치 표시 경로가 연결돼 있다.
- UI `panels/live_visual_status.py:_draw_hardware_brief()`의 전류 표는 `linear`, `roll`, `seg1`, `seg2` 별칭만 조회한다. `"1".."4"` ID가 없어 전류를 받아도 `-`가 될 수 있다. Gripper 칸은 `None` 고정이다. 추후 수정 대상: ID→축 이름 매핑을 전류 조회에도 적용한다. Gripper는 ID 5의 실제 제공 필드·단위를 먼저 확인해야 하며, 수신하지 않은 전류를 생성하거나 부하값과 혼동하면 안 된다. 사용자는 이 항목은 코드만 보고 정적으로 설명하라고 요청했으므로 여기서는 미적용 사항으로 명확히 구분한다.

### Connections 시작 감독 중 관찰

- 01:27경 준비 검증 자체는 com1 1.14s, com3 6.22s, com2 10.52s에 완료됐다. `Connection preparation completed...`는 준비 완료 로그이며 실행 완료 로그가 아니다.
- 다음 start 시 network preparation은 com1 1.00s, com2 3.77s, com3 11.72s였고 scoped registration이 약 11.95s 뒤 실패했다. 해당 시점 UI/Pilot/Sim은 제가 기동한 상태였으므로 Connections의 정지 선행 조건과 충돌하는 상태였다. 최종 error 원문을 내부 API로 읽지는 못했다.
- 별도로 Jetson SSH timeout, UI/Sim의 Pilot peer lost 및 요청 timeout을 관찰했다. Tailscale에 Jetson Online=false와 직접 주소 192.168.50.11:41641이 보이는 시점에도 SSH가 간헐적으로 다시 성공했다. 연결 단절의 근본 원인은 확정하지 않았다.
- 인증 토큰을 로그에서 재사용하는 내부 상태 API 조회는 자동 승인 검토에서 거부돼 실행하지 않았다. 우회하지 않고 로그·상태 명령·코드만 사용했다.

### 추가·삭제 코드 전체

```diff
--- a/containers/build/pilot/app/elesim_pilot/connection.py
+++ b/containers/build/pilot/app/elesim_pilot/connection.py
@@ -103,6 +103,7 @@
         self.active_target = ""
         self.lease_id = ""
         self.endpoints: list[dict[str, Any]] = []
+        self._known_robots: dict[str, dict[str, Any]] = {}
         self.operator_handler: Optional[Callable[[dict[str, Any]], dict[str, Any]]] = None
         self.telemetry_observer = None
         self.on_target_selected: Optional[Callable[[dict[str, Any]], None]] = None
@@ -198,11 +199,35 @@
             self._outbox.put(_Submission("motion", payload, bool(force)))
 
     def select_target(self, target_id: str) -> None:
+        if str(target_id) not in {d['endpoint_id'] for d in self.motion_targets}:
+            raise ValueError("Robot endpoints are the only control targets while a Robot is present")
+        self.clear_pending_motion()
         self.desired_target = str(target_id)
         self._selection_requested = ""
         self._selection_requested_at = None
         with self._outbox_lock:
             self._outbox.put(_Submission("select", {"target_id": self.desired_target}, True))
+
+    @property
+    def motion_targets(self) -> list[dict[str, Any]]:
+        # Retain discovered Robots across network loss. Losing hardware must
+        # not silently redirect the operator's controls to a virtual robot.
+        if self._known_robots:
+            return list(self._known_robots.values())
+        return [d for d in self.endpoints if d.get('role') == 'sim']
+
+    def _sync_sim_presentation(self, endpoint: Any) -> None:
+        descriptor = next((d for d in self.motion_targets
+                           if d.get('endpoint_id') == self.desired_target and d.get('role') == 'robot'), None)
+        if descriptor is None:
+            return
+        for sim in self.endpoints:
+            if sim.get('role') != 'sim' or 'presentation.robot-state.v1' not in sim.get('capabilities', ()):
+                continue
+            endpoint.send('presentation_target', target_id=sim['endpoint_id'], payload={
+                'robot_id': descriptor['endpoint_id'],
+                'robot_boot_id': descriptor.get('instance_id', ''),
+            })
 
     def motion_route(self) -> tuple[str, str, str]:
         descriptor = next((d for d in self.endpoints if d.get("endpoint_id") == self.active_target), {})
@@ -225,10 +250,15 @@
     def send_trajectory_q(self, q: list[float], route: tuple[str, str, str]) -> None:
         if threading.get_ident() != self._connection_thread_ident or self._active_endpoint is None:
             raise RuntimeError("trajectory samples must be sent on the DDS thread")
+        if self.active_target != self.desired_target or self.active_target not in {
+            str(d['endpoint_id']) for d in self.motion_targets
+        }:
+            raise RuntimeError("trajectory control target is no longer selectable")
         if self.motion_route() != route or not all(route):
             raise RuntimeError("trajectory route fence changed")
         self._active_endpoint.send("motion_command", target_id=route[0], lease_id=route[2],
                                    payload={"command": "target", "source": "curobo", "q": q})
+        self.state_sink.remember_trajectory_q(q)
 
     def release_trajectory_route(self, route: tuple[str, str, str]) -> None:
         if self._active_endpoint is not None and self.motion_route() == route:
@@ -386,6 +416,8 @@
         if message_type == "endpoint_list":
             raw_endpoints = payload.get("endpoints", [])
             self.endpoints = [dict(value) for value in raw_endpoints if isinstance(value, Mapping)]
+            self._known_robots.update({str(d['endpoint_id']): d for d in self.endpoints
+                                       if d.get('role') == 'robot' and d.get('endpoint_id')})
             endpoint_ids = ",".join(
                 sorted(str(value.get("endpoint_id", "")) for value in self.endpoints)
             )
@@ -397,6 +429,7 @@
                 peers=endpoint_ids or "none",
             )
             self._request_desired_target(endpoint)
+            self._sync_sim_presentation(endpoint)
             return
         if message_type == "target_selected":
             self.telemetry_received_at = None
@@ -586,6 +619,11 @@
         *,
         now: Optional[float] = None,
     ) -> None:
+        candidates = self.motion_targets
+        candidate_ids = {str(d['endpoint_id']) for d in candidates}
+        if self.desired_target not in candidate_ids and candidates:
+            self.clear_pending_motion()
+            self.desired_target = str(candidates[0]['endpoint_id'])
         if not self.desired_target or self.active_target == self.desired_target:
             return
         if not any(value.get("endpoint_id") == self.desired_target for value in self.endpoints):
@@ -714,6 +752,11 @@
         *,
         allow_without_lease: bool = False,
     ) -> None:
+        if self.active_target != self.desired_target or self.active_target not in {
+            str(d['endpoint_id']) for d in self.motion_targets
+        }:
+            self.state_sink.accept_error('control_target_changing_or_unavailable')
+            return
         if self.motion_guard is not None and not self.motion_guard(payload):
             self._diagnostic("motion", dedupe="trajectory-owns-motion", state="blocked",
                              reason="cuRobo workflow owns motion")
--- a/containers/build/pilot/app/elesim_pilot/main.py
+++ b/containers/build/pilot/app/elesim_pilot/main.py
@@ -250,7 +250,12 @@
 
     @property
     def available_endpoints(self):
-        return list(self._connection.endpoints)
+        candidates = {d['endpoint_id']: d for d in self._connection.motion_targets}
+        peers = {d['endpoint_id']: d for d in self._connection.endpoints}
+        for endpoint_id, descriptor in candidates.items():
+            peers.setdefault(endpoint_id, descriptor)
+        return [{**d, 'control_selectable': endpoint_id in candidates}
+                for endpoint_id, d in peers.items()]
 
     @property
     def active_endpoint(self) -> str:
--- a/containers/build/pilot/protocol/elesim_protocol/contracts.py
+++ b/containers/build/pilot/protocol/elesim_protocol/contracts.py
@@ -139,6 +139,11 @@
             "Robot torque-off and a latched DYNAMIXEL Hardware Error"
         ),
     ),
+    "presentation_target": _contract(
+        "presentation_target", ("pilot",), ("sim",),
+        payload_fields=("robot_id", "robot_boot_id"), strict_fields=True,
+        notes="Read-only selected Robot presentation route; grants no motion authority",
+    ),
     "telemetry": _contract(
         "telemetry", ("robot", "sim"), ("pilot", "ui", "sim"),
         qos="reliable-control", notes=(
--- a/containers/build/pilot/protocol/elesim_protocol/messages.py
+++ b/containers/build/pilot/protocol/elesim_protocol/messages.py
@@ -300,6 +300,7 @@
         "lease_revoked",
         "motion_command",
         "telemetry",
+        "presentation_target",
         "ack",
         "open_simulation_session",
         "simulation_session_opened",
--- a/containers/build/pilot/protocol/elesim_protocol/payloads.py
+++ b/containers/build/pilot/protocol/elesim_protocol/payloads.py
@@ -1017,6 +1017,9 @@
             _identifier(raw.get("lease_id"), name=f"{message_type} lease_id")
         if "reason" in raw:
             _text(raw.get("reason"), name=f"{message_type} reason", maximum=512)
+    if message_type == "presentation_target":
+        _identifier(raw.get("robot_id"), name="presentation Robot ID")
+        _identifier(raw.get("robot_boot_id"), name="presentation Robot boot ID")
     if message_type == "lease_granted":
         _identifier(raw.get("pilot_id"), name="lease_granted pilot_id")
     if message_type in {"lease_revoked", "error"} and "reason" in raw:
--- a/containers/build/sim/app/elesim_sim/endpoint.py
+++ b/containers/build/sim/app/elesim_sim/endpoint.py
@@ -105,6 +105,9 @@
         self.mirror_robot_id = str(mirror_robot_id).strip()
         self._mirror_boot_id = ""
         self._mirror_seq = -1
+        self._mirror_expected_boot = ""
+        self._mirror_pilot = None
+        self._mirror_route_seq = -1
         self.operator_mailbox = operator_mailbox or SimulationOperatorMailbox()
         self.webrtc_offer_handler = webrtc_offer_handler
         self.webrtc_session_close_handler = webrtc_session_close_handler
@@ -306,6 +309,27 @@
 
     def _handle_envelope(self, client: Any, message: Envelope) -> None:
         message_type = message.message_type
+        if message_type == 'presentation_target':
+            pilot = client.node.resolve(message.source_id)
+            payload = message.payload or {}
+            robot_id = str(payload.get('robot_id', ''))
+            robot = client.node.resolve(robot_id) if robot_id else None
+            if (pilot is None or pilot.descriptor.role != 'pilot'
+                    or pilot.identity.boot_id != message.source_boot_id
+                    or robot is None or robot.descriptor.role != 'robot'
+                    or robot.identity.boot_id != payload.get('robot_boot_id')):
+                return
+            if self._mirror_pilot == pilot.identity and message.seq <= self._mirror_route_seq:
+                return
+            changed = (self.mirror_robot_id, self._mirror_expected_boot) != (robot_id, robot.identity.boot_id)
+            self._mirror_pilot = pilot.identity
+            self._mirror_route_seq = message.seq
+            self.mirror_robot_id = robot_id
+            self._mirror_expected_boot = robot.identity.boot_id
+            if changed:
+                self._mirror_boot_id = ''
+                self._mirror_seq = -1
+            return
         if message_type == "telemetry":
             self._mirror_robot_state(client, message)
             return
@@ -376,11 +400,14 @@
             self._handle_webrtc_offer(client, message)
 
     def _mirror_robot_state(self, client: Any, message: Envelope) -> None:
+        if not self.mirror_robot_id:
+            robots = sorted(client.node.discover(role='robot'), key=lambda p: p.identity.endpoint_id)
+            if robots:
+                self.mirror_robot_id = robots[0].identity.endpoint_id
         if (
             not self.mirror_robot_id
             or message.source_id != self.mirror_robot_id
             or message.target_id != self.endpoint_id
-            or self.active_lease
         ):
             return
         peer = client.node.resolve(self.mirror_robot_id)
@@ -389,6 +416,7 @@
             or peer.descriptor.role != "robot"
             or not message.source_boot_id
             or message.source_boot_id != peer.identity.boot_id
+            or (self._mirror_expected_boot and message.source_boot_id != self._mirror_expected_boot)
         ):
             return
         if message.source_boot_id == self._mirror_boot_id and message.seq <= self._mirror_seq:
@@ -473,6 +501,8 @@
         return True, "accepted"
 
     def _apply_motion(self, message: Envelope) -> tuple[bool, str]:
+        if self.mirror_robot_id:
+            return False, 'robot_present_sim_is_presentation_only'
         payload = dict(message.payload or {})
         command = str(payload.get("command", ""))
         ok, reason = self._validate_motion(message, allow_estop=command == "estop")
@@ -817,8 +847,7 @@
             CAPABILITY_MOTION_GO2,
             CAPABILITY_SIM_MOCK_HUG,
         ]
-        if self.mirror_robot_id:
-            capabilities.append(CAPABILITY_ROBOT_STATE_MIRROR)
+        capabilities.append(CAPABILITY_ROBOT_STATE_MIRROR)
         if "rgbd" in self.streams:
             capabilities.append(CAPABILITY_STREAM_RGBD)
         if "observer" in self.streams:
--- a/containers/build/sim/app/elesim_sim/runtime.py
+++ b/containers/build/sim/app/elesim_sim/runtime.py
@@ -740,6 +740,17 @@
     def mirror_mode(self) -> bool:
         return bool(self._mirror)
 
+    def enable_robot_mirror(self) -> None:
+        if self._mirror:
+            return
+        self._mirror = True
+        self._last_mirror_pos = None
+        self._last_mirror_rpy = None
+        self._last_mirror_leg_q = None
+        self._init_mirror_kinematic()
+        # Hold the existing visual base until the Robot supplies a pose.
+        self.apply_mirror_pose(tuple(self._camera_root_pos), (0.0, 0.0, 0.0))
+
     def _init_mirror_kinematic(self) -> None:
         """Mirror puppet: no PD actuation; pose is overwritten each frame."""
         n = len(self._leg_dof_idxs)
@@ -3825,6 +3836,8 @@
                 perf.section("poll", t_sec)
                 t_sec = time.perf_counter()
                 if a.sim_scene.go2 is not None:
+                    if bool(getattr(a.state_source, 'robot_mirror_active', False)):
+                        a.sim_scene.go2.enable_robot_mirror()
                     posture_applied = self._apply_go2_feature_commands()
                     go2_mirror = bool(a.sim_scene.go2.mirror_mode)
                     if go2_mirror and a.state_source is not None:
--- a/containers/build/sim/protocol/elesim_protocol/contracts.py
+++ b/containers/build/sim/protocol/elesim_protocol/contracts.py
@@ -139,6 +139,11 @@
             "Robot torque-off and a latched DYNAMIXEL Hardware Error"
         ),
     ),
+    "presentation_target": _contract(
+        "presentation_target", ("pilot",), ("sim",),
+        payload_fields=("robot_id", "robot_boot_id"), strict_fields=True,
+        notes="Read-only selected Robot presentation route; grants no motion authority",
+    ),
     "telemetry": _contract(
         "telemetry", ("robot", "sim"), ("pilot", "ui", "sim"),
         qos="reliable-control", notes=(
--- a/containers/build/sim/protocol/elesim_protocol/messages.py
+++ b/containers/build/sim/protocol/elesim_protocol/messages.py
@@ -300,6 +300,7 @@
         "lease_revoked",
         "motion_command",
         "telemetry",
+        "presentation_target",
         "ack",
         "open_simulation_session",
         "simulation_session_opened",
--- a/containers/build/sim/protocol/elesim_protocol/payloads.py
+++ b/containers/build/sim/protocol/elesim_protocol/payloads.py
@@ -1017,6 +1017,9 @@
             _identifier(raw.get("lease_id"), name=f"{message_type} lease_id")
         if "reason" in raw:
             _text(raw.get("reason"), name=f"{message_type} reason", maximum=512)
+    if message_type == "presentation_target":
+        _identifier(raw.get("robot_id"), name="presentation Robot ID")
+        _identifier(raw.get("robot_boot_id"), name="presentation Robot boot ID")
     if message_type == "lease_granted":
         _identifier(raw.get("pilot_id"), name="lease_granted pilot_id")
     if message_type in {"lease_revoked", "error"} and "reason" in raw:
--- a/containers/build/tools/protocol/elesim_protocol/contracts.py
+++ b/containers/build/tools/protocol/elesim_protocol/contracts.py
@@ -139,6 +139,11 @@
             "Robot torque-off and a latched DYNAMIXEL Hardware Error"
         ),
     ),
+    "presentation_target": _contract(
+        "presentation_target", ("pilot",), ("sim",),
+        payload_fields=("robot_id", "robot_boot_id"), strict_fields=True,
+        notes="Read-only selected Robot presentation route; grants no motion authority",
+    ),
     "telemetry": _contract(
         "telemetry", ("robot", "sim"), ("pilot", "ui", "sim"),
         qos="reliable-control", notes=(
--- a/containers/build/tools/protocol/elesim_protocol/messages.py
+++ b/containers/build/tools/protocol/elesim_protocol/messages.py
@@ -300,6 +300,7 @@
         "lease_revoked",
         "motion_command",
         "telemetry",
+        "presentation_target",
         "ack",
         "open_simulation_session",
         "simulation_session_opened",
--- a/containers/build/tools/protocol/elesim_protocol/payloads.py
+++ b/containers/build/tools/protocol/elesim_protocol/payloads.py
@@ -1017,6 +1017,9 @@
             _identifier(raw.get("lease_id"), name=f"{message_type} lease_id")
         if "reason" in raw:
             _text(raw.get("reason"), name=f"{message_type} reason", maximum=512)
+    if message_type == "presentation_target":
+        _identifier(raw.get("robot_id"), name="presentation Robot ID")
+        _identifier(raw.get("robot_boot_id"), name="presentation Robot boot ID")
     if message_type == "lease_granted":
         _identifier(raw.get("pilot_id"), name="lease_granted pilot_id")
     if message_type in {"lease_revoked", "error"} and "reason" in raw:
--- a/containers/build/ui/app/elesim_ui/control_panel.py
+++ b/containers/build/ui/app/elesim_ui/control_panel.py
@@ -337,10 +337,8 @@
         # are discovery/control peers, not targets; exposing them here lets a
         # click switch the control plane to the wrong participant.
         #
-        # A Robot belongs here beside the Sim.  Listing only Sim endpoints left
-        # the hardware operator with nothing to click, so `active_endpoint`
-        # stayed empty and the panel read as never connected -- while only the
-        # Sim's selection also moves the camera session (see elesim_ui.main).
+        # Robot discovery makes Sim presentation-only. The camera selector
+        # keeps the complete peer list and never changes this motion route.
         endpoints = []
         for endpoint in self._endpoint_cache:
             role = (
@@ -349,7 +347,13 @@
                 else str(getattr(endpoint, "role", ""))
             )
             if role.strip().lower() in {"sim", "robot"}:
+                if isinstance(endpoint, dict) and endpoint.get('control_selectable') is False:
+                    continue
                 endpoints.append(endpoint)
+        robots = [e for e in endpoints if (e.get('role') if isinstance(e, dict)
+                                          else getattr(e, 'role', '')) == 'robot']
+        if robots:
+            endpoints = robots
         active = self._active_endpoint_cache
         if dropdown:
             entries = [
--- a/containers/build/ui/protocol/elesim_protocol/contracts.py
+++ b/containers/build/ui/protocol/elesim_protocol/contracts.py
@@ -139,6 +139,11 @@
             "Robot torque-off and a latched DYNAMIXEL Hardware Error"
         ),
     ),
+    "presentation_target": _contract(
+        "presentation_target", ("pilot",), ("sim",),
+        payload_fields=("robot_id", "robot_boot_id"), strict_fields=True,
+        notes="Read-only selected Robot presentation route; grants no motion authority",
+    ),
     "telemetry": _contract(
         "telemetry", ("robot", "sim"), ("pilot", "ui", "sim"),
         qos="reliable-control", notes=(
--- a/containers/build/ui/protocol/elesim_protocol/messages.py
+++ b/containers/build/ui/protocol/elesim_protocol/messages.py
@@ -300,6 +300,7 @@
         "lease_revoked",
         "motion_command",
         "telemetry",
+        "presentation_target",
         "ack",
         "open_simulation_session",
         "simulation_session_opened",
--- a/containers/build/ui/protocol/elesim_protocol/payloads.py
+++ b/containers/build/ui/protocol/elesim_protocol/payloads.py
@@ -1017,6 +1017,9 @@
             _identifier(raw.get("lease_id"), name=f"{message_type} lease_id")
         if "reason" in raw:
             _text(raw.get("reason"), name=f"{message_type} reason", maximum=512)
+    if message_type == "presentation_target":
+        _identifier(raw.get("robot_id"), name="presentation Robot ID")
+        _identifier(raw.get("robot_boot_id"), name="presentation Robot boot ID")
     if message_type == "lease_granted":
         _identifier(raw.get("pilot_id"), name="lease_granted pilot_id")
     if message_type in {"lease_revoked", "error"} and "reason" in raw:
--- a/containers/build/pilot/app/elesim_pilot/pick/client.py
+++ b/containers/build/pilot/app/elesim_pilot/pick/client.py
@@ -33,12 +33,17 @@
         self._sender: Optional[CommandSender] = None
         self._lock = threading.RLock()
         self.tx_seq = 0
+        self._last_commanded_q: Optional[SimQ] = None
+        self._command_route_generation = 0
 
     def attach_sender(self, sender: CommandSender) -> None:
         with self._lock:
             self._sender = sender
 
     def close(self) -> None:
+        with self._lock:
+            self._last_commanded_q = None
+            self._command_route_generation += 1
         self._state.peer_connected(False)
 
     @property
@@ -53,12 +58,34 @@
         self._state.peer_connected(connected)
 
     def target_changed(self, target_id: str) -> None:
+        # Every lease grant/loss fences the old target, including a new lease
+        # for the same endpoint after a disconnect.
+        with self._lock:
+            self._last_commanded_q = None
+            self._command_route_generation += 1
         self._state.target_changed(target_id)
 
+    @property
+    def last_commanded_q(self) -> Optional[SimQ]:
+        with self._lock:
+            return self._last_commanded_q
+
+    def remember_trajectory_q(self, q: list[float]) -> None:
+        with self._lock:
+            self._last_commanded_q = SimQ(*map(float, q))
+
+    def _clear_hold_if_torque_off(self, payload: Mapping[str, Any]) -> None:
+        if payload.get("torque_enabled") is False:
+            with self._lock:
+                self._last_commanded_q = None
+                self._command_route_generation += 1
+
     def accept_telemetry(self, payload: Mapping[str, Any]) -> None:
+        self._clear_hold_if_torque_off(payload)
         self._state.accept_telemetry(payload)
 
     def accept_ack(self, payload: Mapping[str, Any]) -> None:
+        self._clear_hold_if_torque_off(payload)
         self._state.accept_ack(payload)
 
     def accept_error(self, reason: str) -> None:
@@ -82,10 +109,19 @@
         with self._lock:
             self.tx_seq += 1
             sender = self._sender
+            generation = self._command_route_generation
         if sender is None:
             self._state.accept_error("pilot transport is not started")
             return
         sender(dict(message), force=bool(force))
+        with self._lock:
+            if generation != self._command_route_generation:
+                return
+            if message.get("t") in {"torque_on", "torque_off", "estop"}:
+                self._last_commanded_q = None
+                self._command_route_generation += 1
+            elif message.get("t") == "target" and "q" in message:
+                self._last_commanded_q = SimQ(*map(float, message["q"]))
 
     def estop(self) -> None:
         self._send({"t": "estop"}, force=True)
--- a/containers/build/pilot/app/elesim_pilot/pick/actions.py
+++ b/containers/build/pilot/app/elesim_pilot/pick/actions.py
@@ -745,12 +745,17 @@
         if self.client is None:
             return None
         host_state = self.client.refresh_state()
-        if host_state.q is not None:
+        # Telemetry is feedback, not a new commanded position. Replacing the
+        # goal with measured sag on every UI snapshot makes the next one-axis
+        # change ratchet all untouched axes down to their measured positions.
+        commanded_q = getattr(self.client, "last_commanded_q", None)
+        state_q = commanded_q if commanded_q is not None else host_state.q
+        if state_q is not None:
             self.state.set_q(
-                float(host_state.q.linear_m),
-                float(host_state.q.roll_rad),
-                float(host_state.q.theta1_rad),
-                float(host_state.q.theta2_rad),
+                float(state_q.linear_m),
+                float(state_q.roll_rad),
+                float(state_q.theta1_rad),
+                float(state_q.theta2_rad),
             )
         return host_state
 
--- a/connections/demo/topology.json
+++ b/connections/demo/topology.json
@@ -25,7 +25,7 @@
             {
               "role": "ui",
               "endpoint_id": "ui",
-              "release_key": "769738ed05750f9178b789668bbfd6b724b4c1d15709dbcbe6c8ab79f0f25279"
+              "release_key": "45a2c37eb1d2fd94ea423ca525e8a2692b932cbebd7bc4b442b82e367202374f"
             }
           ],
           "install_mode": "container",
@@ -34,14 +34,14 @@
           "lifecycle": "compose",
           "install_uuid": "1f95fd4e-2810-4f97-8a29-81257e8074db",
           "project": "elesim-clear",
-          "release_key": "769738ed05750f9178b789668bbfd6b724b4c1d15709dbcbe6c8ab79f0f25279"
+          "release_key": "45a2c37eb1d2fd94ea423ca525e8a2692b932cbebd7bc4b442b82e367202374f"
         }
       ],
       "assignments": [
         {
           "role": "ui",
           "endpoint_id": "ui",
-          "release_key": "769738ed05750f9178b789668bbfd6b724b4c1d15709dbcbe6c8ab79f0f25279"
+          "release_key": "45a2c37eb1d2fd94ea423ca525e8a2692b932cbebd7bc4b442b82e367202374f"
         }
       ],
       "install_mode": "container",
@@ -73,7 +73,7 @@
             {
               "role": "sim",
               "endpoint_id": "sim",
-              "release_key": "043db5121e121d8fc366abd2c204f5c4bb63bee830094baf9a4104376dfb08cc"
+              "release_key": "df51144a6029bbbd585958f8bfff1798440e209338bda7f3ba4a2f5b97c0e6ea"
             }
           ],
           "install_mode": "container",
@@ -82,14 +82,14 @@
           "lifecycle": "compose",
           "install_uuid": "de583b07-349d-4481-a8d4-ce5a3cdec3cd",
           "project": "elesim-dawn",
-          "release_key": "043db5121e121d8fc366abd2c204f5c4bb63bee830094baf9a4104376dfb08cc"
+          "release_key": "df51144a6029bbbd585958f8bfff1798440e209338bda7f3ba4a2f5b97c0e6ea"
         }
       ],
       "assignments": [
         {
           "role": "sim",
           "endpoint_id": "sim",
-          "release_key": "043db5121e121d8fc366abd2c204f5c4bb63bee830094baf9a4104376dfb08cc"
+          "release_key": "df51144a6029bbbd585958f8bfff1798440e209338bda7f3ba4a2f5b97c0e6ea"
         }
       ],
       "install_mode": "container",
@@ -121,7 +121,7 @@
             {
               "role": "pilot",
               "endpoint_id": "pilot",
-              "release_key": "9cdc912eb749e8104809cc684dd902acf19fd50acfcf9c57cbb20163991b1601"
+              "release_key": "5f7d845425a538ac8b64851a9207b79f87ce3dc5fb834bfd925d7c02459e4369"
             }
           ],
           "install_mode": "container",
@@ -130,7 +130,7 @@
           "lifecycle": "compose",
           "install_uuid": "7a88a13a-2bfb-47e7-827f-348d5ae2ed1a",
           "project": "elesim-pale",
-          "release_key": "9cdc912eb749e8104809cc684dd902acf19fd50acfcf9c57cbb20163991b1601"
+          "release_key": "5f7d845425a538ac8b64851a9207b79f87ce3dc5fb834bfd925d7c02459e4369"
         },
         {
           "id": "robot-native",
--- /dev/null
+++ b/release-data/config/pilot/wrap_grasp.yaml
@@ -0,0 +1,8 @@
+policy_path: /opt/elesim/data/policies/wrap-grasp/policy.pt
+manifest_path: /opt/elesim/data/policies/wrap-grasp/interface.json
+object:
+  radius_m: 0.05
+  height_m: 1.1
+  xyz_m: [0.368, 0.16, 0.55]
+  lean: [0.0, 0.0]
+roll_sign: 1.0
```

### RL 바이너리 출처 및 무결성

- `oct27-policy.pt`: 198996 bytes, SHA256 `9712131dbe7fa474af7dd36c2fd01dc1830b0b14ce5dac91dfbf70d5bf109683`.
- `oct27-interface.json`: 5606 bytes, SHA256 `55e1807b9af0d36c7a4058b32589921f30594980d8500edfe8755ef482e10442`.
- `oct27-wrap_grasp.yaml`: 317 bytes, SHA256 `8c9e2d676673f641010906b6771fccea29bc940fd126e0a1928f8fbf8810bb67`.

학습 파일 원본은 `diagnostics/wrap-policy-audit/`에 보존했다. 바이너리 자체는 unified diff로 표현하지 않고 원본 경로·크기·SHA256로 기록한다.

### 준비 완료 뒤 GPU/Viewer 대기 지연의 코드 경로

`Connection preparation completed...` 출력 후 브라우저 `pollJob()`이 `pollRuntimeStatus()`를 호출한다. `/api/runtime`의 `ConnectionDeploymentRunner.runtime_status()`는 각 호스트의 lifecycle status와 GPU policy/devices를 수집한다. 호스트 조회는 병렬이지만 `list(executor.map(...))`로 전체 응답이 끝난 뒤 반환한다. 브라우저 `renderRuntimeStatus()`는 모든 호스트의 inventory/GPU 정책이 준비돼야 `runtimeReady=true`로 만들고, 그때까지 GPU/Viewer 선택 및 시작 버튼을 잠근다. status에 inventory가 없거나 실패한 경우 `runtime_inventory()`를 추가 조회한다. ParamikoConnector의 기본 connect timeout은 10s, command timeout은 120s다. 따라서 느린 호스트 하나와 중복 fallback 조회가 전체 대기를 늘릴 수 있다. 실제로 이번 감독 중 Jetson SSH timeout을 관찰했다. 지금 화면의 특정 API 요청 소요시간 자체를 인증 API로 측정하지는 않았으므로 코드상 병목과 실제 관찰을 구분한다.

개선할 위치: `elesim_connections/connections.py:runtime_status()`의 inventory 결과를 준비 단계와 공유하고, 상태 명령에 짧은 별도 timeout을 주며, 브라우저에 호스트별 후속 조회 진행/실패를 표시한다. 즉 '준비 완료'가 최종 시작 가능 상태라는 오해를 줄여야 한다. 이 감독 과정에서는 진행 중인 사용자의 Connections 작업을 중단하며 해당 코드를 바꾸지는 않았다.


### 2026-10-28 01:38–01:41 KST 실측 수신 및 Jetson 통신 단절

읽기 전용 UI snapshot observer로 실제 수신을 확인했다. 명령·토크·서비스 재시작은 수행하지 않았다. `diagnostics/telemetry-20261028/ui-snapshots.log`에 원본을 보존했다.

- 01:38:11: Pilot target `robot`, 최신 host state age 약 0.063초. 모터 위치 ID 1–5 수신(raw 1607,1945,178,113,3861), 전류 ID 1–4 수신(-97,0,0,0 mA). 당시 torque_enabled=false, safety_fault 빈 문자열.
- 같은 시점 GO2 몸체 위치/자세/속도 및 다리 q/dq/torque는 모두 null, GO2 timestamp=0. GO2의 실측 수신은 확인되지 않았으며, 이후 Jetson 접속 단절로 원격 ROS 토픽 진단을 완료하지 못했다.
- UI Hardware 전류 표시 경로의 축 이름과 숫자 ID 불일치는 위에 기록한 별도 표시 문제다. 전류 수신 자체는 확인됐다.
- UI 로그: 01:36:51 Robot/Pilot 모두 lost, 01:37:40–44 같은 boot ID로 재발견, 01:38:21 Robot/Pilot 동시 lost. 이후 `target peer 'pilot' is not active` 반복. 같은 boot ID 재등장은 해당 구간 재시작보다는 통신 단절/정체와 부합하지만 근본 원인을 확정하지 않는다.
- 01:40 관측: Jetson Tailscale 100.88.154.118 및 LAN 192.168.50.11 ping 각각 4/4 유실, SSH 양쪽 timeout, Tailscale ping 무응답, peer Online=false. 공유기 192.168.50.1 ping 3/3 성공(약 2.6ms). Sim은 UI discovery ready 지속.
- 따라서 현재 장애는 UI의 로봇 표시만이 아니라 Jetson 접속 경로 전체에 걸친 장애다. 전원 이상, Jetson 시스템 정지, Jetson 측 네트워크 장애는 원격 미접속 상태에서 구분 불가. 접속 복구 후 커널/OOM/네트워크 로그 및 ROS 수신 상태를 확인해야 한다.

이번 진단의 제품 코드 변경: 없음. 추가 파일: 읽기 전용 observer `diagnostics/telemetry-20261028/read-ui-snapshot.py`, snapshot 로그 `ui-snapshots.log`, UI 단절 로그 `ui-disconnection.log` (동일 디렉터리). 서비스 중지/재시작/모터 명령 없음.


#### 01:41 접속 복구 후 추가 확인 (위 미접속 판단 갱신)

- 01:41:02 SSH 복구. uptime 6시간 53분으로 재부팅되지 않았다. Robot/브리지 모두 01:36:30부터 active/running, NRestarts=0. Pilot 컨테이너도 계속 실행 중.
- Pilot 로그에는 외부 접속 단절 중에도 로컬 Robot의 ready 및 peers=robot이 지속된다. 01:40:54 Sim, 01:40:55 UI 재발견. UI도 01:40:55–56 Robot/Pilot을 동일 boot ID로 재발견했다. 따라서 해당 구간 Jetson 전체 정지나 Robot 프로세스 종료보다는 Jetson의 외부 네트워크 경로 단절로 장애 범위를 좁힐 수 있다.
- NetworkManager: 01:39:44 CONNECTED_SITE, 01:40:54 CONNECTED_GLOBAL. 복구 시간이 DDS 재발견과 일치한다. 이것만으로 Wi-Fi AP 연결 해제를 단정하지 않는다.
- Jetson wlP1p1s0: SSID H3C_D41BD2, 2437MHz(2.4GHz), 복구 시 신호 -47dBm, Power save:on. 절전/무선 경로는 조사 후보이며 확정 원인이 아니다. eno1은 GO2 연결용 192.168.123.99/24이므로 노트북 경로와 구분해야 한다.
- 사용자 관측: 전원 켜짐, 화면 멈춤/없음. 원격 로그의 로컬 Robot 통신 지속 사실과 함께 기록한다.
- 이번 단계에서는 Wi-Fi 설정을 변경하거나 네트워크/로봇 서비스를 재시작하지 않았다. 접속 복구는 관측 사실이며 수리 완료를 뜻하지 않는다.


### 2026-10-28 01:42 재단절의 직접 원인: Connections 준비 확인 실패 후 자동 롤백

이번 단절은 앞서 관측한 네트워크 무응답과 구별해야 한다. 01:42:52 LAN ping 3/3 성공(약 4.2ms), SSH 접속 가능. Robot은 01:42:23 systemd stop으로 종료됐고 Pilot도 뒤이어 정지했다.

증거:
- `logs/build/20261027T163134Z-ae142c97cad7.log`: 모든 호스트 start complete 이후 DDS readiness 검사. com3는 `probe call failed before DDS evaluation: Timeout opening channel.`. `DDS readiness failed: all hosts (221.84s)` 이후 `Runtime startup or DDS readiness failed; rolling back runtimes started by this operation.`.
- Jetson 저널 01:42:23: 노트북 100.82.201.62에서 SSH로 `sudo -n systemctl stop elesim-robot.service` 호출. sudo COMMAND 및 systemd Stopping/Stopped 모두 확인. 브리지도 함께 종료.
- Docker events: Pilot 01:42:25 SIGTERM, 01:42:35 SIGKILL, 01:42:36 stop/die. inspect ExitCode=137, OOMKilled=false. 이 137은 메모리 부족의 증거가 아니라 stop 유예시간 뒤 강제 종료 기록과 부합한다.
- UI Robot lost 01:42:25.947, Pilot lost 01:42:38.717. 서비스 종료와 시간적으로 부합한다.

코드: `containers/build/tools/app/elesim_connections/connections.py`의 `_report_runtime_readiness()`는 호스트별 probe 예외를 실패로 모으고, 시작 작업의 except에서 `_rollback_runtime_hosts()`를 호출한다(약 808–830행). probe SSH 채널 열기 실패도 실제 DDS 미준비와 같은 롤백 경로로 들어간다. 기존 네트워크 장애로 발생한 준비 확인 실패가, 네트워크 복구 후에도 시작 트랜잭션 실패로 처리돼 실행 중 런타임 종료로 이어진 것으로 로그가 일치한다.

수정 검토 지점: 시작 확인 동안 UI에서 아직 시작 트랜잭션 미완료임을 명확히 표시할 것. probe 전송 실패는 실제 endpoint 부재와 구분해 제한된 재접속/재검사를 하고, 롤백 직전 최신 상태를 확인할 것. 롤백을 무조건 없애거나 제어 watchdog 시간을 늘리는 해결은 적용하지 않았다. 이번 단계의 제품 코드 변경 및 런타임 재시작 없음. 앞서 Wi-Fi 절전은 후보라고 기록했으나 이번 01:42 종료의 직접 원인은 확인된 자동 롤백이다.


### 2026-10-28 Jetson Wi-Fi 절전 해제 (사용자 명시 요청으로 적용)

대상: com3 `wlP1p1s0`, NetworkManager 연결 프로필 `H3C_D41BD2`. 변경 전 iw Power save:on, 프로필 powersave=default. 변경 후 실제 장치 Power save:off, 프로필 powersave=disable(숫자 2), 아래 systemd unit enabled. Wi-Fi 연결 down/up 또는 NetworkManager 재시작은 수행하지 않았다. Robot/Pilot은 자동 롤백으로 정지된 상태이며 이번 작업에서 시작하지 않았다. 절전 해제는 원인 후보 제거이며 이전 네트워크 단절의 근본 원인 확정을 뜻하지 않는다.

일반 `sudo -n iw ...`는 비밀번호 요구로 실패했다. 기존에 허용된 관리자 명령 install/systemctl을 사용하여 사용자 요청 범위의 전용 oneshot unit을 설치·실행했다. unit은 부팅 때도 적용된다. 시스템 전체 suspend 설정은 변경하지 않았다.

정확한 프로필 변경(암호 등 다른 설정은 변경하지 않음):
```diff
--- com3:NetworkManager/connection/H3C_D41BD2
+++ com3:NetworkManager/connection/H3C_D41BD2
@@ -1 +1 @@
-802-11-wireless.powersave=0 (default)
+802-11-wireless.powersave=2 (disable)
```

추가 파일:
```diff
--- /dev/null
+++ com3:/etc/systemd/system/elesim-wifi-powersave-off.service
@@ -0,0 +1,13 @@
+[Unit]
+Description=Disable Wi-Fi power saving for EleSim Jetson connectivity
+After=NetworkManager.service
+Wants=NetworkManager.service
+
+[Service]
+Type=oneshot
+ExecStart=/usr/bin/nmcli connection modify H3C_D41BD2 802-11-wireless.powersave 2
+ExecStart=/usr/sbin/iw dev wlP1p1s0 set power_save off
+RemainAfterExit=yes
+
+[Install]
+WantedBy=multi-user.target
```

설치: root:root 0644, `systemctl daemon-reload`, `systemctl enable --now elesim-wifi-powersave-off.service`. 부팅 링크 `/etc/systemd/system/multi-user.target.wants/elesim-wifi-powersave-off.service` 추가. 로컬 사본 `diagnostics/telemetry-20261028/elesim-wifi-powersave-off.service`. 원복은 관리자 권한으로 해당 unit disable/stop 및 삭제, daemon-reload 후 프로필 powersave=0, iw power_save on을 적용한다(stop만으로 설정이 원복되지는 않음). 검증 출력: `Power save: off`, `disable`, `enabled`. 네트워크 장기 안정성은 아직 검증하지 않았다.


## 추가 진단 파일 원문과 Wi-Fi 원복 절차

아래 observer는 UI와 같은 snapshot 경로를 읽는다. `view_snapshot` 외 조작 요청을 보내지 않는다. 결과 로그는 수집 당시 상태이며 캐시된 host 값의 age와 operator snapshot age를 함께 확인해야 한다.

```diff
--- /dev/null
+++ diagnostics/telemetry-20261028/read-ui-snapshot.py
@@ -0,0 +1,18 @@
+"""Read-only observer: requests UI snapshots, never issues service/state calls."""
+import dataclasses,json,time,uuid
+from datetime import datetime,timezone
+from elesim_ui.config import load_config
+from elesim_ui.operator_session import OperatorSession
+cfg=load_config('/opt/elesim/config/installed.yaml')
+s=OperatorSession(ui_id='telemetry-observer-'+uuid.uuid4().hex[:8],pilot_id=cfg.pilot_id,settings=cfg.dds,snapshot_period_s=.5)
+fields=['connected','host_state_age_s','rx_age_s','device','torque_enabled','safety_fault','motor_positions_raw','motor_positions_deg','motor_currents_ma','q','go2_base_pos','go2_base_rpy','go2_base_timestamp_s','go2_base_lin_vel_body','go2_base_ang_vel','go2_leg_q','go2_leg_dq','go2_leg_torque_nm']
+def encode(v):
+ if dataclasses.is_dataclass(v):return dataclasses.asdict(v)
+ if hasattr(v,'__dict__'):return vars(v)
+ return str(v)
+try:
+ for i in range(20):
+  time.sleep(1)
+  host=s.service_value('current_host_state')
+  print('SAMPLE '+json.dumps({'utc':datetime.now(timezone.utc).isoformat(),'status':dataclasses.asdict(s.status),'target':s.service_value('active_endpoint'),'host':None if host is None else {k:getattr(host,k,None) for k in fields}},default=encode),flush=True)
+finally:s.close()
```

진단 출력: `diagnostics/telemetry-20261028/ui-snapshots.log`, `ui-disconnection.log`. 보조 스크립트/패치는 `diagnostics/robot-sim-repair/`, 초기 수집 저널은 `logs/diagnostics/`에 있다. 이 문서에는 이슈에 필요한 증거를 발췌했고, 접근 토큰이 포함될 수 있는 관리자 로그 전체를 복제하지 않았다.

Wi-Fi 설정을 이전 상태로 돌릴 때만 Jetson의 관리자 터미널에서 실행할 명령(현재 실행하지 않음):

```bash
sudo systemctl disable --now elesim-wifi-powersave-off.service
sudo rm /etc/systemd/system/elesim-wifi-powersave-off.service
sudo systemctl daemon-reload
sudo nmcli connection modify H3C_D41BD2 802-11-wireless.powersave 0
sudo iw dev wlP1p1s0 set power_save on
```

프로필의 0은 기존 default로 복원한다는 뜻이다. 현재 유지할 값은 2(disable)다. Wi-Fi 전용 절전 변경이며 Jetson 전체 suspend나 NVIDIA 전력 모드는 변경하지 않았다.


## 01:50 KST 절전 해제 후 단절 재발 및 롤 오프셋 작업 진행 상태

사용자 요청으로 01:50:15 즉시 확인: Jetson `Power save: off`, Robot active(01:48:18 시작), NRestarts=0, Pilot Up 2 minutes. Wi-Fi 신호 -45dBm, 2437MHz. UI 로그에서 01:49:57 Robot/Pilot lost, 01:50:12–13 같은 boot ID로 ready. 이번 시점은 서비스 종료가 아니라 통신 단절/복구다. Connections는 새 시작의 DDS readiness 확인 중이며 당시 로그에 새 rollback은 없었다.

이어 동시 ping에서 LAN 192.168.50.11과 Tailscale 100.88.154.118이 각각 12/12 무응답. 결과는 `diagnostics/telemetry-20261028/after-powersave-{lan,tailscale}.log`. 노트북 tailscaled에는 direct 주소 재발견과 `derp-20 does not know about peer ... removing route`가 반복됐다. LAN도 함께 끊기므로 Tailscale만의 장애로 단정할 수 없다. **절전 해제로 해결되지 않았다는 실측 증거이며**, Jetson의 무선/AP/드라이버·호스트 네트워크를 계속 조사해야 한다. 직전 실제 서비스가 계속 실행됐고 재발 후 원격 로그 접근이 막혔으므로 하위 원인 확정은 아직 불가.

별도 사용자 요청인 롤 오프셋 -9→+9는 로컬 `apps/pilot/config/config.yaml`의 pc·jetson 프로필 두 곳에 우선 적용했다. 원본 `diagnostics/roll-offset-20261028/local-config.before.yaml`, 정확한 diff `local.patch`. Jetson과 배포 릴리스 반영은 이 단절 때문에 아직 완료되지 않았다. 실행 중 Pilot의 offset을 바꿨다고 주장하지 않는다. 아래 후속 완료 기록이 생기기 전까지 진행 중이다.


## com2 제거·Sim 노트북 이전·5GHz·롤 오프셋 최종 적용

### 요청과 최종 적용 범위

처음에는 192.168.50.11 직접 LAN 전환까지 요청받아 준비했다. 이후 사용자가 Tailscale 유지로 변경했으므로 최종 demo의 DDS 주소/SSH 인증은 Tailscale 그대로다. com2 제거, Sim 노트북 이전, 롤 오프셋 변경은 계속 적용했다. 중간에 Jetson instance를 LAN으로 등록했지만 런타임을 시작하지 않았으며, 최종 등록은 tailscale0와 두 Tailscale peer로 되돌렸다.

기존 Connections 관리자와 demo 런타임을 먼저 정지해 이전 시작 작업의 롤백이 새 구성과 겹치지 않도록 했다. 서버에서는 `elesim-dawn-demo-sim`만 정지했다. `emulator`, `doosan_kos` 등 다른 컨테이너는 변경하지 않았다. com2의 설치 디렉터리와 기존 이미지는 삭제하지 않았고 최종 Docker 상태 `exited`를 확인했다.

### Sim 이전 방법과 배포 정보

노트북에 있던 `elesim/sim:clear-chicken`을 기반으로 현재 수정된 `elesim_sim` 및 `elesim_protocol` 패키지를 복사한 파생 이미지 `elesim/sim:clear-heron`을 빌드했다. 서버의 23GB 이미지를 통째로 복사하지 않았다. 서버 Sim의 기존 릴리스 data/config는 약 3.1MB 압축 파일로 가져와 노트북 UI 설정을 보존하며 통합했다. Robot 종속 Sim 관련 코드 변경은 앞선 전체 diff와 동일하다.

- 새 Sim image ID: `sha256:d8a8bf9f1972b315f0b946d5d9cfbcd2997ac36fa2c78d08ce8de55116d0dc5f`.
- 새 Sim build fingerprint: `bc2d83f222601a268c361789ec3ec992dcde59d28562f40943c6a8a164e551d9`.
- 노트북 UI+Sim 통합 release: `26acc23e4fdace1e8eed0c2824fa651cf4bb18852f33899785c432ea5dea7cf3`.
- Jetson Pilot +9 offset release: `a8ea81d434c655501fdfbc35bb5a860e6abd56c6fff145ea7ae28fa9681a0962`.
- UI/Pilot 이미지 자체는 `clear-lynx` / `pale-badger` 그대로다. Pilot은 config data 변경으로 새 release key가 생겼다.
- 이전 노트북 release: `45a2c37eb1d2fd94ea423ca525e8a2692b932cbebd7bc4b442b82e367202374f`.
- 이전 Pilot release: `5f7d845425a538ac8b64851a9207b79f87ce3dc5fb834bfd925d7c02459e4369`.

노트북 Sim은 GPU 0 (`gpu_mode=specific`)에 등록했고 별도 Sim viewer는 껐다. UI의 NVIDIA 정책은 유지된다. instance manager의 replace로 `instances/demo/state.json`, endpoint별 config, `containers/compose.instances.yaml`, 소유권 메타데이터를 생성했다. 양쪽 release catalog도 갱신했다. native Robot은 `elesim-net configure`로 static peer를 `100.82.201.62`, `100.88.154.118` 두 개로 재생성했고 network_interface=tailscale0를 확인했다.

실제 파생 이미지 Dockerfile:
```dockerfile
FROM elesim/sim:clear-chicken
COPY elesim_sim/ /usr/local/lib/python3.10/dist-packages/elesim_sim/
COPY elesim_protocol/ /usr/local/lib/python3.10/dist-packages/elesim_protocol/
LABEL io.elesim.install_uuid="1f95fd4e-2810-4f97-8a29-81257e8074db" io.elesim.build_fingerprint="bc2d83f222601a268c361789ec3ec992dcde59d28562f40943c6a8a164e551d9"
```

### 토폴로지 변경 전체

`connections/demo/topology.json`에서 com2 항목 제거, com1 UI+Sim assignment 및 통합 release 지정, com3 Pilot release 변경. 최종 SSH는 기존 `auth_mode=tailscale` 및 기존 pinned fingerprint 유지.

```diff
--- a/connections/demo/topology.json
+++ b/connections/demo/topology.json
@@ -25,7 +25,12 @@
             {
               "role": "ui",
               "endpoint_id": "ui",
-              "release_key": "45a2c37eb1d2fd94ea423ca525e8a2692b932cbebd7bc4b442b82e367202374f"
+              "release_key": "26acc23e4fdace1e8eed0c2824fa651cf4bb18852f33899785c432ea5dea7cf3"
+            },
+            {
+              "role": "sim",
+              "endpoint_id": "sim",
+              "release_key": "26acc23e4fdace1e8eed0c2824fa651cf4bb18852f33899785c432ea5dea7cf3"
             }
           ],
           "install_mode": "container",
@@ -34,67 +39,24 @@
           "lifecycle": "compose",
           "install_uuid": "1f95fd4e-2810-4f97-8a29-81257e8074db",
           "project": "elesim-clear",
-          "release_key": "45a2c37eb1d2fd94ea423ca525e8a2692b932cbebd7bc4b442b82e367202374f"
+          "release_key": "26acc23e4fdace1e8eed0c2824fa651cf4bb18852f33899785c432ea5dea7cf3"
         }
       ],
       "assignments": [
         {
           "role": "ui",
           "endpoint_id": "ui",
-          "release_key": "45a2c37eb1d2fd94ea423ca525e8a2692b932cbebd7bc4b442b82e367202374f"
+          "release_key": "26acc23e4fdace1e8eed0c2824fa651cf4bb18852f33899785c432ea5dea7cf3"
+        },
+        {
+          "role": "sim",
+          "endpoint_id": "sim",
+          "release_key": "26acc23e4fdace1e8eed0c2824fa651cf4bb18852f33899785c432ea5dea7cf3"
         }
       ],
       "install_mode": "container",
       "install_root": "/home/idim/newsim",
       "bin_dir": "/home/idim/newsim/bin",
-      "lifecycle": "compose"
-    },
-    {
-      "id": "com2",
-      "local": false,
-      "dds": {
-        "address": "100.74.222.24",
-        "interface": "tailscale0",
-        "address_source": "tailscale"
-      },
-      "ssh": {
-        "host": "100.74.222.24",
-        "port": 22,
-        "user": "hckang",
-        "identity_file": "",
-        "pinned_fingerprint": "SHA256:2oAaevxeuYi0yc3umwg15I5c/plBF8GW2x9TvfN7paY",
-        "auth_mode": "tailscale"
-      },
-      "jetson": false,
-      "units": [
-        {
-          "id": "runtime",
-          "assignments": [
-            {
-              "role": "sim",
-              "endpoint_id": "sim",
-              "release_key": "df51144a6029bbbd585958f8bfff1798440e209338bda7f3ba4a2f5b97c0e6ea"
-            }
-          ],
-          "install_mode": "container",
-          "install_root": "/home/hckang/newsim",
-          "bin_dir": "/home/hckang/newsim/bin",
-          "lifecycle": "compose",
-          "install_uuid": "de583b07-349d-4481-a8d4-ce5a3cdec3cd",
-          "project": "elesim-dawn",
-          "release_key": "df51144a6029bbbd585958f8bfff1798440e209338bda7f3ba4a2f5b97c0e6ea"
-        }
-      ],
-      "assignments": [
-        {
-          "role": "sim",
-          "endpoint_id": "sim",
-          "release_key": "df51144a6029bbbd585958f8bfff1798440e209338bda7f3ba4a2f5b97c0e6ea"
-        }
-      ],
-      "install_mode": "container",
-      "install_root": "/home/hckang/newsim",
-      "bin_dir": "/home/hckang/newsim/bin",
       "lifecycle": "compose"
     },
     {
@@ -121,7 +83,7 @@
             {
               "role": "pilot",
               "endpoint_id": "pilot",
-              "release_key": "5f7d845425a538ac8b64851a9207b79f87ce3dc5fb834bfd925d7c02459e4369"
+              "release_key": "a8ea81d434c655501fdfbc35bb5a860e6abd56c6fff145ea7ae28fa9681a0962"
             }
           ],
           "install_mode": "container",
@@ -130,7 +92,7 @@
           "lifecycle": "compose",
           "install_uuid": "7a88a13a-2bfb-47e7-827f-348d5ae2ed1a",
           "project": "elesim-pale",
-          "release_key": "5f7d845425a538ac8b64851a9207b79f87ce3dc5fb834bfd925d7c02459e4369"
+          "release_key": "a8ea81d434c655501fdfbc35bb5a860e6abd56c6fff145ea7ae28fa9681a0962"
         },
         {
           "id": "robot-native",
```

### 롤 오프셋 변경 전체

로컬 `apps/pilot/config/config.yaml`, com3의 같은 legacy app config, 새 Pilot 릴리스 template, 등록된 `instances/demo/endpoints/pilot/config/config.yaml`에 반영했다. 공통 fallback의 0.0, 다른 축, 모터 방향/관절 한계는 변경하지 않았다. 실제 사용하는 pc·jetson 프로필의 두 값이 +9.0임을 YAML로 읽어 확인했다.

```diff
--- a/release-data/config/pilot/config.yaml
+++ b/release-data/config/pilot/config.yaml
@@ -12,7 +12,7 @@
     robot:
       arm:
         hardware:
-          u_offset_roll: -9.0
+          u_offset_roll: 9.0
       go2:
         spawn:
           mount_offset_m:
@@ -77,7 +77,7 @@
           profile_acc_seg2: 20
           profile_vel_seg1: 150
           profile_vel_seg2: 150
-          u_offset_roll: -9.0
+          u_offset_roll: 9.0
       go2:
         spawn:
           mount_offset_m:
```

로컬 template diff:
```diff
--- a/apps/pilot/config/config.yaml
+++ b/apps/pilot/config/config.yaml
@@ -12,7 +12,7 @@
     robot:
       arm:
         hardware:
-          u_offset_roll: -9.0
+          u_offset_roll: 9.0
       go2:
         spawn:
           mount_offset_m:
@@ -77,7 +77,7 @@
           profile_acc_seg2: 20
           profile_vel_seg1: 150
           profile_vel_seg2: 150
-          u_offset_roll: -9.0
+          u_offset_roll: 9.0
       go2:
         spawn:
           mount_offset_m:
```

### Jetson 5GHz 전환

일반 사용자 rescan 권한이 없어 root oneshot scan을 `/run/systemd/system/elesim-wifi-scan.service`로 실행했다. 스캔 완료 후 같은 공유기의 H3C_D41BD2_5G를 확인했다. 다른 공유기인 IDIM_Robot_5G로 옮기지 않았다.

기존 H3C_D41BD2 프로필을 복제해 기존 인증 정보를 그대로 사용했고 비밀번호를 출력/문서화하지 않았다. 새 프로필 UUID는 `32cc4331-5518-469f-900d-91388eeef387`. SSID=H3C_D41BD2_5G, band=a, powersave=2, autoconnect=yes, priority=100으로 설정했다. 기존 2.4GHz 프로필은 실패 시 복귀용으로 보존했다. 01:56:19 연결 활성화 성공. 실제 `iw link`: SSID H3C_D41BD2_5G, freq 5580, rx bitrate 780 Mbit/s, tx bitrate 351 Mbit/s (링크 협상값이지 실제 처리량 보증은 아님). `iw ... get power_save`: off.

사용한 전환 스크립트(인증 비밀 없음):
```bash
#!/usr/bin/env bash
set -euo pipefail
if ! nmcli -g connection.uuid connection show H3C_D41BD2_5G >/dev/null 2>&1; then
  nmcli connection clone H3C_D41BD2 H3C_D41BD2_5G
fi
nmcli connection modify H3C_D41BD2_5G 802-11-wireless.ssid H3C_D41BD2_5G 802-11-wireless.band a 802-11-wireless.powersave 2 connection.autoconnect yes connection.autoconnect-priority 100
if ! nmcli --wait 25 connection up H3C_D41BD2_5G ifname wlP1p1s0; then
  nmcli --wait 20 connection up H3C_D41BD2 ifname wlP1p1s0
  exit 1
fi
iw dev wlP1p1s0 set power_save off
```

일회성 실행 unit(`/run`에 설치, 부팅 자동 실행으로 enable하지 않음):
```ini
[Unit]
Description=Switch Jetson to the requested 5GHz Wi-Fi with fallback
After=NetworkManager.service
[Service]
Type=oneshot
TimeoutStartSec=70
ExecStart=/bin/bash /run/elesim-switch-5ghz.sh
```

기존 `/etc/systemd/system/elesim-wifi-powersave-off.service`는 유지된다. 이 unit은 기존 2.4GHz 프로필도 powersave=2로 두고 현재 인터페이스의 절전도 끈다. 새 5GHz 프로필 자체에도 powersave=2가 영속 저장됐다.

LAN 준비 과정에서 노트북 `~/.ssh/elesim_jetson_lan` 키를 생성하고 공개키만 Jetson authorized_keys에 추가했다. 기존 authorized_keys는 `~/.ssh/authorized_keys.before-elesim-lan-20261028`로 백업했다. 개인키는 이 문서/토폴로지에 포함하지 않았다. 최종 Connections는 이 키를 사용하지 않으며 Tailscale SSH를 유지한다. LAN fallback을 폐기할 때에는 추가한 공개키 항목만 제거하고 노트북의 해당 키 쌍을 삭제할 수 있다. 기존 다른 키는 보존해야 한다.

### 확인한 것 / 미확인 사항

- ConnectionTopology 파서로 최종 역할 com1=[ui,sim], com3=[pilot,robot] 확인.
- 최종 노트북/Jetson instance 및 native Robot DDS interface=tailscale0, static peers 두 주소 확인.
- 노트북 Compose 설정 검사 통과. Sim 새 이미지의 endpoint/runtime/protocol 모듈 import 성공. 초기 점검 명령에서 존재하지 않는 PresentationTarget 클래스 이름을 import해 실패했으나 실제 코드의 메시지는 문자열 계약이다. 실제 모듈 import는 다시 확인해 통과했다. 제품 코드 결함으로 기록하지 않는다.
- 서버 Sim exited, 로컬 demo 런타임 정지, native Robot inactive 확인. 재시작/모터 동작 검증은 하지 않았다.
- Jetson 5GHz 연결과 절전 off 확인, 노트북 향한 ping 8/8 성공.
- 전체 Connections startup, Sim scene 초기화/GPU 메모리 적합성, 영상 부하에서의 네트워크 안정성, GO2 실측 수신은 아직 검증하지 않았다. “이제 시작해봐도 된다”는 구성이 준비됐다는 의미이며 모든 실기기 기능을 통과했다는 뜻이 아니다.

### 변경 파일·백업·원복

- 최종 topology: `connections/demo/topology.json`.
- 노트북 원본: `diagnostics/lan-migration-20261028/backup/` (topology, instance, compose, install-state, ownership, release catalog).
- com3 원본: `/home/idim/newsim/diagnostics/lan-migration-20261028/backup/` (instance, compose, ownership, native install-state 및 Pilot config). 원래 Pilot -9 config는 `pilot-config.before.yaml`에 남아 있다.
- 서버 원본 release 스냅샷: `diagnostics/lan-migration-20261028/server-sim-release.tar.gz` 및 압축 해제 디렉터리.
- 등록 스크립트: `diagnostics/lan-migration-20261028/register.py` (현재 버전은 Tailscale 유지). 결과: `local-result.json`, `pilot-result.json`, `image-plan.json`.
- 정확한 patch: `topology.patch`, `roll-offset.patch`; 로컬 legacy config의 원본/patch는 `diagnostics/roll-offset-20261028/`.

기존 구성으로 돌아가려면 런타임을 먼저 정지한 뒤 백업 topology와 이전 release로 instance manager를 통해 재등록한다. generated Compose만 따로 덮어쓰면 registry와 불일치할 수 있다. 서버 Sim을 다시 사용할 때도 먼저 한 endpoint만 존재하는지 확인한 뒤 구성을 복원한다.

5GHz만 되돌릴 경우 Jetson 관리자 권한으로 새 프로필의 자동 연결 우선순위를 낮추거나 autoconnect를 끈 후 기존 H3C_D41BD2 프로필을 활성화한다. 이는 통신 단절을 수반하므로 복구 가능한 터미널에서 한다. 이 원복은 실행하지 않았다.


## 02:00 KST GO2 실측 미수신 원인 확인 — DDS domain 불일치 (아직 미수정)

모터는 이전 UI snapshot에서 ID 1–5 위치, 1–4 전류가 수신됐다. GO2는 이번에 Jetson의 eno1에 바인딩한 읽기 전용 ROS subscriber로 원본 송신을 확인했다. 모터/GO2 명령 publisher 또는 IPC client를 생성하지 않았다.

- GO2 유선 주소 192.168.123.161 ping 2/2 성공. Jetson eno1은 192.168.123.99/24.
- 현재 설치 go2.ros_domain_id=1. domain 1에서 12초 동안 sportmodestate/lowstate 각각 0개, 원본 publisher 목록도 비어 있었다.
- domain 0으로 동일 토픽을 읽으면 12초 동안 sportmodestate 3598개, lowstate 5565개 수신. reliable/best-effort subscriber 모두 같은 수량을 받았다. GO2 몸체 position/rpy/velocity 및 12개 다리 q/dq/tau_est 실측 확인. 이 관측에서 QoS 불일치가 원인은 아니다.
- 따라서 현재 GO2 미수신의 직접 원인은 실제 Unitree 송신 domain 0과 브리지 설정 domain 1의 불일치다. GO2 하드웨어 원본 데이터가 없는 상황이 아니다.
- EleSim graph도 현재 domain 0이다. native config validator는 Unitree와 EleSim domain을 다르게 요구하므로 go2.ros_domain_id만 0으로 바꾸면 설정 검증이 실패한다. 수정 시 EleSim 전체(UI/Sim/Pilot/Robot)를 별도 domain(예: 1)으로 맞추고 Unitree는 0으로 설정해 함께 재기동해야 한다. 이 변경은 아직 적용하지 않았다.
- 추가로 실제 설치된 native runtime.py::state()는 go2 leg_q만 발행하고 leg_dq/leg_torque_nm은 발행하지 않는다. bridge IPC와 Pilot 수신 측은 이 필드를 지원하므로 실제 속도/추정 토크를 UI/Sim까지 전달하려면 native 발행부도 수정해야 한다. 이것도 아직 미적용이다.

근거: `diagnostics/telemetry-20261028/read-go2-topics.py`, `go2-topic-probe.log`(domain1 및 초기 domain0 출력 직렬화 오류), `go2-domain0-confirmed.log`(직렬화 수정 후 재측정한 실제 수신 결과). 초기 진단 도구가 numpy.float32 JSON 직렬화에서 실패해 float 변환을 넣고 domain0만 다시 측정했다. 센서 수신 실패로 오해하지 않는다. GO2 원본 수신 확인을 기존 UI 경로가 이미 수정됐다는 의미로 확대하지 않는다.


## Extend Arm의 롤 표시값 171 → 180

현재 오프셋 +9에서 기존 Extend Arm은 apply_offset=False로 내부 롤 180을 넣어 표시 계산 180-9=171이 됐다. 사용자가 본 값은 이 경로와 일치한다. 수정 후 Extend Arm의 롤에만 표시 오프셋을 더하므로 내부값=180+offset, 화면값=180이다. 현재 offset=+9이면 내부 목표는 189이고 화면은 180이다. linear 15, s1/s2 180의 기존 내부 preset은 그대로 보존한다. 롤 보정 자체를 없애거나 모든 축에 오프셋을 새로 적용하지 않는다.

실제 코드 diff:
```diff
--- a/containers/build/pilot/app/elesim_pilot/pick/actions.py
+++ b/containers/build/pilot/app/elesim_pilot/pick/actions.py
@@ -2145,7 +2145,9 @@
 
     def extend_arm_controls(self) -> None:
         self.state.clear_ik_status()
-        self.apply_control_u(u_linear=15.0, u_roll=180.0, u_s1=180.0, u_s2=180.0, apply_offset=False)
+        # Keep the roll preset at 180 in the same coordinates shown by the UI.
+        roll_actual = 180.0 + self._offsets()["roll"]
+        self.apply_control_u(u_linear=15.0, u_roll=roll_actual, u_s1=180.0, u_s2=180.0, apply_offset=False)
         self.send_current_target(source="slider")
 
     def send_current_target(
```

배포: com3 원본 source를 `diagnostics/extend-arm-20261028/actions.before.py`로 백업한 뒤 수정본을 build source와 새 파생 이미지에 반영했다. 기존 release data(+9)를 그대로 사용한 새 불변 릴리스를 게시하고 com3 release catalog 및 com1 topology를 갱신했다. 실행 중인 Pilot container/instance는 변경하지 않았으며 새 코드가 현재 프로세스에 로드됐다고 주장하지 않는다.

```json
{
  "old_release": "a8ea81d434c655501fdfbc35bb5a860e6abd56c6fff145ea7ae28fa9681a0962",
  "release_key": "7f2e434f4eb4b5f04c47c919f89eb70ba9f2eb4b30eff91e3d39093ed03172c5",
  "tag": "elesim/pilot:pale-lynx",
  "image_id": "sha256:0d04083e0c56a75393a4302d5e3e82e7b9e045d4ca37ea3cc0445e58e5fb2b5f",
  "fingerprint": "fb746e8e550e7260ea19b6f74645cf35d22199fd3060a49956c9672f02c4d19f",
  "running_instance_replaced": false
}
```

정확한 topology diff:
```diff
--- a/connections/demo/topology.json
+++ b/connections/demo/topology.json
@@ -83,7 +83,7 @@
             {
               "role": "pilot",
               "endpoint_id": "pilot",
-              "release_key": "a8ea81d434c655501fdfbc35bb5a860e6abd56c6fff145ea7ae28fa9681a0962"
+              "release_key": "7f2e434f4eb4b5f04c47c919f89eb70ba9f2eb4b30eff91e3d39093ed03172c5"
             }
           ],
           "install_mode": "container",
@@ -92,7 +92,7 @@
           "lifecycle": "compose",
           "install_uuid": "7a88a13a-2bfb-47e7-827f-348d5ae2ed1a",
           "project": "elesim-pale",
-          "release_key": "a8ea81d434c655501fdfbc35bb5a860e6abd56c6fff145ea7ae28fa9681a0962"
+          "release_key": "7f2e434f4eb4b5f04c47c919f89eb70ba9f2eb4b30eff91e3d39093ed03172c5"
         },
         {
           "id": "robot-native",
```

적용 시점: 기존 Connections로 전체 종료 → 관리자를 새로 열어 demo 시작. 새 릴리스는 topology에 이미 선택되어 있다. TDD/동작 시험은 하지 않았고 Python 문법 확인 및 이미지 빌드/릴리스 게시를 완료했다. Extend Arm을 실제 로봇에 실행하지 않았다. 원복은 로컬/원격 actions.before.py 및 topology.before.json, 이전 Pilot release a8ea81...를 기준으로 수행한다.
