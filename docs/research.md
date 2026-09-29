# 연구와 진단

이 문서는 repository-only 실험 규칙과 현재 유지하는 연구 기능을 소유한다.
연구 코드는 runtime role, Compose service, DDS broker 또는 release input이 아니다.

## 실행 경계

- runner: `workbench/research/experiments/`
- offline analysis: `workbench/research/analysis/`
- curated evidence: `workbench/evidence/curated/`
- generated evidence: `workbench/evidence/generated/` (Git 제외)
- manual debugger: `workbench/research/debug/`
- canonical environment: setup-generated persistent `elesim-dev`

```bash
elesim-up
elesim-dev python3 workbench/research/experiments/<runner>.py
```

각 결과는 source revision, effective config, role topology, endpoint/boot,
run ID, GPU policy, encoder, simulated/physical hardware와 통과하지 않은 수동
gate를 기록한다. Headless simulation은 물리 convergence, SROS2 enforce, NAT,
TURN relay 또는 display 동작의 증거가 아니다.

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

Production baseline:

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
