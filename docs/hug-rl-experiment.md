# 두 band의 경로 학습과 보간 실험 실행법

2026-11-05. **실행 경로는 구현했지만 이 checkout에서 GPU 학습·물리 재생은 실행하지 않았다.**
아래 순서에서 처음 실행할 것은 `check --physics`다. 이것이 실패하면 학습을 시작하지 않는다.
이 문서는 현재 코드로 실행할 명령, 생성 파일, 판정과 재시작 방법을 설명한다.

## 무엇을 실험하는가

D=150 mm에서 band별 작은 삼각형 하나를 잡는다. 각 삼각형의 꼭짓점 3개씩,
총 **6개 케이스만 PPO로 각각 학습**한다. 그 경로를 원래 위치에서 먼저 재생한 후,
다른 위치는 추가 RL 없이 보간 경로로 실행한다.

현재 입력의 분할은 train 6 / validation 22 / test 2다. 각 삼각형은 x/y 방향으로
20 mm 범위를 갖는다. 이는 두 띠 전체를 대표한다는 주장이 아니라 국소 보간의 첫 실험이다.
`test` 2개는 validation으로 설정을 조정한 뒤 마지막에 평가한다. 실패한 test를 보고
기준점을 고치면 그 점들은 더 이상 최종 독립 시험점이 아니다.

- band 1: roll −90°, band 2: roll +90°.
- 두 경우 모두 seg2 명령은 0 이하, 즉 ZED 반대 방향이다.
- linear=0, roll은 에피소드 내 고정이다. Go2도 고정한다.
- 정책은 seg1/seg2의 **노드당** 각도 증분 두 개를 출력한다.
- 맵의 총 굽힘각은 `radians(seg_deg)/5`로 바꾼다. C/S 부호 경계로 가족을 나누지 않는다.
- 초기에는 seg1을 물체 반대 방향으로 노드당 0.30 rad 벌리고 seg2=0으로 둔다.
  Home에서 roll을 바꾸고 이 초기 자세까지 가는 동작은 이번 실험 범위가 아니다.

코드 위치:

| 위치 | 소유하는 것 |
| --- | --- |
| `payload/runtime/docker/sim/app/elesim_sim/rl` | 공통 scene/runner/관절 명령/경로 파일/재생 API |
| `workbench/research/experiments/hug_interpolation` | 이번 물체·시작 상태·보상·성공 판정·분할·보간 |
| `workbench/research/experiments/wrap_grasp` | 이전 원기둥 Wrap 과제, 로봇 scene/index helper |

새 Hug 환경은 이전 Wrap 환경/보상/커리큘럼을 상속하지 않는다. 로봇 인덱싱과 gain
설정 helper, PPO 네트워크 설정만 재사용한다. Pilot·DDS·Robot에 연결하지 않는다.

## 1. 집에 가져갈 것과 환경

이 변경이 들어 있는 **전체 EleSim checkout**이 필요하다.
[공개 학습 예제](../workbench/research/experiments/hug_interpolation/README.md)는 포함된
`example_pilot.json`만으로 실행할 수 있으며, 대용량 맵은 새 기준점을 추출할 때만 필요하다. 새 파일까지 포함해야 한다.
코드·모델 digest를 비교하므로 다른 checkout의 checkpoint를 임의로 섞지 않는다.
집에서 다른 checkout을 쓴다면 이 예제와 공통 RL 분리가 포함된 revision으로 갱신한다.

맵 작업용 사본:

```text
workbench/evidence/generated/hug_interpolation/input/map.json
```

원본은 `/home/user/ws/hugmap/map.json`이며 수정하지 않았다. 사본은 85,369,250 bytes다.
SHA-256:

```text
c28a6f9535e8859fa400300903d145ca6d9663ef805db83d04a2fe9f1782b1ee
```

`workbench/evidence/generated/`는 Git에서 제외된다. **Git pull만으로 맵이나
생성된 pilot/experiment 파일이 옮겨지지 않는다.** 맵을 위 상대경로에 별도로 복사하거나
아래 `MAP`을 실제 사본 경로로 바꾼다. 포함된 예제만 실행할 때는 이 복사가 필요 없다. `gaga.json` 같은 schema v2 고정-roll 파일은 거부한다.

준비된 현재 설치의 개발 attachment나 별도의 학습용 Python 환경에서 실행한다.
이 checkout의 호스트 Python에 패키지를 설치해 두지는 않았다.
현재 고정 버전은 다음과 같고 `check` 및 실제 실행에서 확인한다.

| 패키지 | 버전 |
| --- | --- |
| torch | 2.12.1 (CUDA build suffix 허용) |
| genesis-world | 1.4.5 |
| tensordict | 0.14.2 |
| rsl-rl-lib | 5.4.0 |
| tensorboard | 2.21.0 |
| numpy | 1.26.4 권장, 프로젝트 범위는 1.26 이상 2 미만 |

Linux/WSL의 CUDA GPU를 기준으로 한다. `--backend cpu`는 작은 진단용이다.
현재 CLI의 `gpu` 경로는 CUDA를 요구한다. Metal/MPS 경로는 구현하지 않았다.
준비된 개발 attachment가 있으면 그 안에서 다음 명령을 실행한다. 별도 환경을 만들 경우
**호스트 전역이 아닌 전용 venv**에서 다음과 같이 설치한다. NVIDIA 드라이버와 선택한
Torch CUDA wheel의 호환성은 해당 머신에서 확인해야 한다.

```bash
python3 -m venv "$HOME/.venvs/elesim-hug"
source "$HOME/.venvs/elesim-hug/bin/activate"
python -m pip install 'numpy==1.26.4' 'torch==2.12.1' 'tensorboard==2.21.0'
python -m pip install -e payload/runtime/common/protocol -e 'payload/runtime/docker/sim/app[rl]'
```

프로젝트의 기존 학습 환경이 준비되어 있으면 다시 설치할 필요가 없다. 생성된
`elesim-dev` wrapper가 없는 checkout에서 임의의 개발 컨테이너를 찾을 필요도 없다.

이 작업에서 미리 생성한 입력은 다음 경로에도 있다.

```text
workbench/evidence/generated/hug_interpolation/home-ready/pilot.json
workbench/evidence/generated/hug_interpolation/home-ready/experiment.json
```

같은 코드·모델을 가져간 경우 그대로 사용할 수 있다. 아래 명령은 결과를 섞지 않도록
새 OUT에 다시 준비하는 방식이다. 집의 GPU에서 생성할 `physics-check.json`은 아직 없다.

## 2. 작업 디렉터리와 설정 생성

저장소 루트에서 Bash로 실행한다. 이하 모든 명령은 같은 셸을 사용한다.
GUI는 기본적으로 열지 않는다. 시각 확인을 원할 때만 `check`/`replay`에 `--viewer`를 붙인다.

```bash
cd /path/to/elesim
export PYTHONPATH="$PWD/payload/runtime/common/protocol:$PWD/payload/runtime/docker/sim/app:$PWD${PYTHONPATH:+:$PYTHONPATH}"
hug() { python3 -m workbench.research.experiments.hug_interpolation.experiment "$@"; }
MAP="$PWD/workbench/evidence/generated/hug_interpolation/input/map.json"
OUT="$PWD/workbench/evidence/generated/hug_interpolation/run-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"

python3 -m workbench.research.experiments.hug_interpolation.prepare "$MAP" \
  --diameter-mm 150 --output "$OUT/pilot.json"
hug configure --pilot "$OUT/pilot.json" --output "$OUT/experiment.json"
hug check --experiment "$OUT/experiment.json" --output "$OUT/dependencies.json"
```

`prepare`는 각 roll에서 해가 있는 작은 격자 삼각형을 선택하고 분할을 저장한다.
`configure`는 URDF 좌표 변환, 실험 설정, 모델/코드/케이스 digest를 확정한다.
기본 설정을 바꿀 때는 변경할 키만 담은 JSON을 만들고
`configure --settings settings.json`을 사용한다. 설정 키는 `configure.py`의 `DEFAULTS`에 있다.
예를 들어 `{"mass_kg": 0.4}`는 질량만 바꾼다. 이 경우 **새 OUT과 새 실험**을 쓴다.

현재 소스 기준 좌표 변환은 `node0_world=(0.343,0,0.562)` m,
맵의 `node0=(0,144.5)` mm를 사용한다. 따라서:

```text
object_world_x = 0.343 + (map_y_mm - 144.5)/1000
object_world_y = -map_x_mm/1000
object_world_z = cylinder_height/2 = 0.55 m
```

상수를 월드 원점에 임의로 붙이지 않고 URDF FK로 node0를 계산한다. 장착 방향이나
맵 mounting offset이 다르면 configure가 거부한다. 생성 파일은 덮어쓰지 않는다.

## 3. 물리 사전 점검 — 학습하지 않음

```bash
hug check --physics --experiment "$OUT/experiment.json" --output "$OUT/physics-check.json"
```

이 명령은 학습 기준점 6개의 초기 자세를 검사한다. Genesis link 위치와 URDF FK가
2 mm 이내인지, 짧은 유지 중 초기 충돌/과도한 관절 변형이 없는지 확인한다.
이어 열린 팔 상태에서 물체에 외란을 주어 바닥만으로 유지 시험을 통과하지 않는지 확인한다.
마지막으로 PPO runner와 2차원 정책 출력을 생성해 API를 점검한다. optimizer 학습은 하지 않는다.

`ready=true`, 종료 코드 0이어야 다음으로 간다. 초기 충돌이나 FK 불일치가 나오면
장시간 PPO로 해결하려 하지 말고 모델·초기 자세·관절 규약을 먼저 확인한다.
패키지 누락/CUDA 미사용은 코드 2, 물리 사전 점검 불통과는 코드 1이다.
API 예외가 나면 traceback을 보존한다. GPU에서 이 경로가 이미 통과했다고 주장하지 않는다.

실험용 `assets/robot.urdf`는 실행 디렉터리 아래 생성한다. 원본 bundle은 건드리지 않는다.
복사본에서 10개 bend joint를 ±36°, roll을 ±90°로 열고 카메라 visual mesh를 collision에도
넣는다. 두 jaw는 맵의 zero-opening 형상 그대로 고정하며 jaw 개폐를 학습하지 않는다.
기본 bundle의 0/0 joint limit과 카메라 collision 누락을 그대로 사용하지 않는다.
이 충돌 형상은 Genesis의 분해/근사 형상이며 정밀 CAD 접촉 인증은 아니다.

## 4. 첫 기준점에서 짧게 확인한 뒤 6개 학습

API/메모리 확인을 위해 먼저 한 케이스만 작은 병렬 수로 학습할 수 있다.
**이 단계부터 실제 RL이다.** 현재 작업 중에는 이 명령을 실행하지 않았다.

```bash
hug train --experiment "$OUT/experiment.json" \
  --case b1-x-100-y320-d150 --runs "$OUT/smoke-runs" \
  --num-envs 32 --iterations 20
```

위 case ID는 현재 맵/150 mm pilot에 있는 값이다. 다른 지름이나 맵이면 `pilot.json`의
`split=train`인 case ID를 사용한다. 짧은 20회 학습으로 파지 성공을 요구하지 않는다.
로그가 유한하고 메모리·속도가 감당되는지 확인하는 용도다.

이후 전체 기준점 6개를 실행한다.

```bash
hug train --experiment "$OUT/experiment.json" --all \
  --runs "$OUT/runs" --num-envs 256 --iterations 2000
```

256은 **한 기준점의 병렬 복제 수**이고 2000은 **기준점당 PPO iteration 수**다.
6개를 서로 다른 프로세스로 순서대로 학습하여 Genesis 초기화/상태가 섞이지 않게 한다.
GPU 6개를 동시에 쓰는 명령은 아니다. 실제 소요 시간은 첫 실행 로그로 판단한다.
첫 JIT/scene build 시간과 steady-state rollout 시간을 구분한다.

각 `$OUT/runs/<case-id>/`에는 `run.json`, `model_*.pt`, `model_final.pt`,
TensorBoard 로그, 완료한 학습 시도의 `training_attempts.jsonl`이 남는다.
후자는 wall-clock 초, 추가 iteration 수, resume checkpoint, 패키지 버전을 기록한다.

```bash
tensorboard --logdir "$OUT/runs" --bind_all=false
```

중단된 **한 케이스**를 같은 설정·병렬 수로 이어가려면:

```bash
hug train --experiment "$OUT/experiment.json" \
  --case b1-x-100-y320-d150 --runs "$OUT/runs" \
  --num-envs 256 --iterations 1000 --resume
```

`--iterations 1000`은 누적 목표가 아니라 **추가 1000회**다. `--resume`은 해당 run의
가장 최근 checkpoint를 읽는다. 일부 케이스만 생성된 상태에서 `--all --resume`을 쓰지 않는다.
이미 완료한 케이스는 그대로 두고 나머지 case ID를 개별 실행하면 된다.
기존 디렉터리에 `--resume` 없이 학습하면 거부한다. OOM이면 새 run 경로에서 128/64 등으로
줄인다. 이 첫 실험에서는 병렬 수를 바꾼 resume를 자동 허용하지 않는다.

## 5. 원본 경로 수집 → 같은 위치에서 재생

다음은 각 정책을 결정론적으로 한 번 실행하여 **물리 tick마다 실제 보낸 PD 목표**를 기록한다.
policy action만 저장하거나 종료 후 reset된 관절값을 마지막 경로로 기록하지 않는다.
그다음 같은 케이스에서 정책 없이 경로만 재생한다. 원본 재생을 통과한 파일만 보간에 넣는다.

```bash
(
  set -e
  for run in "$OUT"/runs/*; do
    [ -d "$run" ] || continue
    cid=$(basename "$run")
    hug collect --experiment "$OUT/experiment.json" --run "$run" \
      --output "$OUT/collected/$cid.json"
    hug replay --experiment "$OUT/experiment.json" \
      --trajectory "$OUT/collected/$cid.json" \
      --output "$OUT/original-replay/$cid.json" \
      --verified-output "$OUT/anchors/$cid.json"
  done
)
```

수집 또는 재생 성공 판정이 실패하면 파일은 남고 종료 코드 1을 반환한다. 위 subshell은
그 케이스에서 멈춘다. 실패한 원본을 억지로 `replay_success=true`로 바꾸지 않는다.
`collect --checkpoint "$run/model_1000.pt"`처럼 다른 checkpoint를 명시할 수 있다.
재시도는 새로운 출력 이름/디렉터리를 사용한다. 최종 anchors 폴더에는 각 기준점당
**성공한 원본 재생 경로 하나씩, 총 6개만** 둔다. 중복 case ID는 거부한다.

학습의 terminal 성공과 원본 경로 재생 성공이 다르면 보간 문제가 아니다.
시간축, 초기화, PD, 물체 초기 상태, 관측에 반응한 피드백 효과를 먼저 확인한다.
이 코드의 수집은 고정 초기 조건에서의 단일 결정론적 경로이며 반복 성공률을 증명하지 않는다.

## 6. Validation: 추가 RL 없이 보간하고 재생

```bash
hug interpolate --experiment "$OUT/experiment.json" --anchors "$OUT/anchors" \
  --split validation --output "$OUT/validation-trajectories"
hug validate --experiment "$OUT/experiment.json" \
  --trajectories "$OUT/validation-trajectories" --split validation \
  --output "$OUT/validation-report.json"
```

보간은 각 점의 기하 솔버 목표 q를 이용한다. 삼각형 barycentric weight로 원본 경로의
시작→끝 기준선에 대한 잔차를 섞는다. approach 3초와 hold 1초의 경계를 유지하고
hold 구간은 최종 명령으로 고정한다. 경로를 수정할 정도의 clip이나 외삽은 하지 않는다.
경계 위 점은 허용하지만 삼각형 밖, 다른 band/roll/D/branch/context는 거부한다.

`validate`는 정책/optimizer 없이 경로만 재생한다. 동일 D의 scene 하나를 재사용하되
각 케이스의 roll, 관절, 물체 위치·자세·속도와 판정 버퍼를 reset한다.
실패가 하나라도 있으면 코드 1이지만 전체 보고서는 기록한다. `results`에서
`goal_error_rad`, `hold_displacement_m`, `hold_contact_fraction`, `fault`를 비교한다.
보고서를 보존한 뒤 다음 실험의 기준점이나 보간을 조정한다. 실패 시 자동 RL은 하지 않는다.

최종 설정을 정한 뒤 test 2개를 한 번 평가한다.

```bash
hug interpolate --experiment "$OUT/experiment.json" --anchors "$OUT/anchors" \
  --split test --output "$OUT/test-trajectories"
hug validate --experiment "$OUT/experiment.json" \
  --trajectories "$OUT/test-trajectories" --split test --output "$OUT/test-report.json"
```

이 결과로 말할 수 있는 것은 **이 초기 상태·물체·물리 조건에서 이 두 국소 영역의 경로가
보간 가능한지**다. 전체 띠의 동역학적 연결성이나 실기기 성공을 증명하지 않는다.

## 판정과 모델 가정

기본 물체는 지름 150 mm, 높이 1.1 m, 질량 0.30 kg의 수직 자유 원기둥이다.
바닥 z=0에 서고 팔 높이는 약 0.562 m다. 높이/질량은 2D 맵에서 나온 값이 아니라
이번 3D 실험의 명시적 가정이다. 팔–물체 마찰계수는 0.5다.

Genesis는 접촉쌍에서 큰 쪽 마찰계수를 사용하므로 robot=0.5,
object=floor=0.01로 설정한다. 물체에 robot과 같은 마찰을 주고 floor만 낮추면
바닥 마찰을 낮춘 효과가 없어지는 점을 피했다. 이 API 의미는
[Genesis 1.4.5 RigidEntity 소스](https://github.com/Genesis-Embodied-AI/Genesis/blob/v1.4.5/genesis/engine/entities/rigid_entity/rigid_entity.py)에 따른다.

정책 actor 관측은 실제 seg 평균 2개, 현재 명령 2개, 목표 2개, 시간 1개다.
critic은 물체 상대 위치/속도 및 두 접촉군 정보를 추가로 본다. action은 0.1초당
노드각 최대 0.03 rad 증분이며 모든 물리 tick에서 선형 ramp로 보낸다.
보상은 목표 오차 감소, 목표 근접, 접촉, 작은 action 비용, fault 비용, 최종 성공 보너스다.
마지막 1초는 정책의 action을 무시하고 명령을 고정한다.

성공하려면 다음을 모두 만족해야 한다.

- 전체 4초 종료. 끝의 두 segment 실제 평균각 오차가 각각 노드당 0.06 rad 이하.
- hold 시작 때 world +Y 방향으로 0.10 m/s 속도 외란을 더한 뒤, 1초 동안 최종 이동이 10 mm 이하.
- hold tick의 80% 이상에서 seg1(node0–4)과 distal(node5–9 또는 gripper)이 동시에 물체에 접촉.
- 전체 과정에 금지 충돌, 팔/물체 관련 5 mm 초과 penetration, 80 mm 초과 물체 이동, 15° 초과 기울기가 없음.
- linear 실제 이탈 3 mm 이하, roll 이탈 0.03 rad 이하, segment 내 노드각 max−min이 0.05 rad 이하.
- seg2가 카메라 쪽으로 유의미하게 굽거나 물리 상태가 비유한 값이 되지 않음.

접촉은 양의 힘이 있는 buffer entry로 판정한다. 카메라·하우징·wedge·Go2와의 금지 접촉,
팔의 바닥 접촉과 팔 자체 충돌을 fault로 처리한다. 초기 충돌쌍을 임의로 허용 목록에 넣지 않는다.

이것은 **접촉 + 약한 외란 후 유지라는 조작적 성공 기준**이다. 6D force closure,
임의 외란 저항, 물체 들어 올리기, 하드웨어 torque/탄성 식별의 증명은 아니다.
각 노드에 같은 PD 목표를 주지만 실제 기계식 기어 결합을 구속식으로 모델링한 것은 아니며,
노드간 각도 편차 검사를 통해 과도한 독립 변형을 거부한다. 정책 관측/행동이 다르므로
예전 100 mm Wrap checkpoint를 이 환경에 넣을 수 없다.

## 문제가 생겼을 때

| 증상 | 먼저 할 일 |
| --- | --- |
| 코드/모델/context digest 불일치 | 수정한 소스로 새 OUT에서 configure. 옛 checkpoint의 의미를 자동 바꾸지 않는다. |
| FK mismatch 또는 처음부터 fault | `check --physics --viewer`로 장착/roll/초기 자세/카메라 형상 확인. RL 시간을 늘리지 않는다. |
| ungrasped control도 10 mm 이하로 멈춤 | 외란·바닥 마찰·질량·단위 확인. 바닥 지지를 파지로 판정하지 않는다. |
| OOM | 새 run에서 num-envs 감소. viewer 없이 실행. |
| 오래 학습해도 성공 0 | 먼저 goal error와 fault/contact를 구분. 목표에 못 가는지, 도달하지만 유지가 안 되는지 확인. |
| 원본 replay 실패 | 수집 trace, reset, 시간축과 PD 조건 확인. 아직 보간을 평가하지 않는다. |
| 원본은 성공, 보간은 실패 | 유의미한 실험 결과다. 실패 위치/접촉/목표 오차 기록 후 새 validation 실험 설계. |
| 파일 존재 오류 | 덮어쓰기 방지다. 새 OUT 또는 새 출력 파일명 사용. 학습 재개만 `--resume`. |
| API traceback | 설치 버전과 traceback 보존. 현재 환경에서 GPU gate를 실행하지 못했으므로 무시하지 않는다. |

공통 PPO 연결은 [rsl_rl 5.4.0 OnPolicyRunner](https://github.com/leggedrobotics/rsl_rl/blob/v5.4.0/rsl_rl/runners/on_policy_runner.py)
의 환경/관측 인터페이스를 따른다. 코드의 host-only 테스트 통과와 실제 Genesis/PPO 실행 통과는 구분한다.


## 이번 작업에서 확인한 범위

Host-only 회귀 56개 통과: URDF→맵 축/각도, 실험용 URDF의 한계·카메라·jaw 처리,
설정·케이스 digest, validation/test 분리, hold 구간 보간, 명령 기록/재생 및 기존 Wrap
설정/진행 판정 회귀. Python compilation과 `git diff --check`도 통과했다.
호스트 의존성 점검은 Torch/Genesis/tensordict/rsl_rl 누락을 보고하고 코드 2로 끝났다.
GPU smoke, 실제 PPO 학습, 수집, 원본/보간 경로 물리 재생은 실행하지 않았다.
