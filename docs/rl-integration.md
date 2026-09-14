# wrap-grasp-rl 전환 절차

이 절차는 별도 checkout에서 검증하기 위한 것이다. 학교 서버의 실행 중인
학습 프로세스, 기존 checkout/환경, run 디렉터리는 변경하지 않는다.
`wrap-grasp-rl`에 통합 결과를 강제 push하지 않는다.

## 학습 자료와 Pilot의 경계

학습 자료(`model_*.pt`, `model_best.pt`, curriculum sidecar, metadata와
TensorBoard 로그)는 학습 서버의 run 디렉터리에 남긴다. Pilot은 checkpoint를
직접 읽지 않고 export된 `policy.pt`와 `interface.json`만 소비한다.
`policy.npz`는 export의 NumPy 검증 산출물이며 현재 Pilot 추론기는 TorchScript다.
Pilot에는 Sim/Genesis/RL 학습 패키지를 설치할 필요가 없다.

구형 run의 실제 위치는 서버에서 확인해야 한다. 과거 기본값은 `sim/rl_runs`,
현재 기본값은 `var/rl/sim`이다. 자동 이동이나 삭제를 하지 않는다.

## 별도 학습 환경에서 실행

아래 경로는 예시다. 기존 환경을 수정하지 말고 별도 checkout 및 검증된 학습
환경에서 실행한다. 새 run 이름과 export 디렉터리를 선택한다.

```bash
cd /path/to/integration-checkout
export PYTHONPATH="$PWD/payload/runtime/common/protocol:$PWD/payload/runtime/docker/sim/app"
python -m elesim_sim.rl.train \
  --resume /path/to/old-run/model_best.pt \
  --set train.log_dir=/path/to/new-runs --stamp integration-check
python -m elesim_sim.rl.eval --checkpoint /path/to/new-run/model_best.pt
python -m elesim_sim.rl.export \
  --checkpoint /path/to/new-run/model_best.pt --out-dir /path/to/new-export
```

학습 때 사용한 `--config`, `--overlay`, `--set`은 평가/export에도 동일하게
전달한다. 기존 checkpoint의 optimizer/네트워크/관측 계약과 현재 코드가
다르면 resume가 유효하다고 볼 수 없다. 파일을 읽을 수 있다는 것과 학습
의미가 유지된다는 것은 다르다.

## 기구·정책 호환성

현재 선형축은 1.5도/mm, 0–250도에 0–166.666667 mm 후퇴다. 기존
-0.23 m 모델로 학습한 정책을 현재 모델과 동일한 정책으로 취급하지 않는다.
CAD plate 부착 위치도 현재 main 값을 유지했다. 기존 checkpoint는 원래 환경에
보존하고, 새 환경에서 재평가 또는 재학습한다. manifest 범위만 편집해서
Pilot의 호환성 검사를 우회하지 않는다.

관측은 manifest에 선언된 12채널(load 없음) 또는 16채널(load 포함) 순서를
따른다. `trained_under.object_pose_source`의 `told`/`measured` 의미와 실제
입력원의 일치도 확인해야 한다. 물리 구동은 이번 통합의 검증 범위가 아니다.

## Pilot에 전달

검증한 export의 두 파일을 대상 설치의 `data/policies/wrap-grasp/`에 함께
배치한다. 외부 경로를 사용할 경우 Pilot의 `policy_path`와 `manifest_path`를
명시하고 컨테이너에서 읽을 수 있게 해야 한다. 새 immutable release를
publish한 뒤 대상 instance를 명시적으로 교체해야 기존 pin이 새 파일을 쓴다.
실행 중인 서버에 자동 배포하거나 기존 release의 파일을 덮어쓰지 않는다.

현재 통합 검증 결과와 미실행 gate는 [status.md](status.md)를 참조한다.
