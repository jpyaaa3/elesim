# 현재 사용하는 Workbench

자동 회귀는 `tools/quality/check.py`, 무거운 실험은 `tools/bench/`로 실행한다.
벤치의 입력·GPU·격리·판정 방식은 [benchmarks 문서](../docs/benchmarks.md)가 소유한다.
이 목록의 보조 도구는 개별 진단/원자료 준비/그림 생성에 사용한다.

## 평소 실행하는 명령

저장소 루트에서, 현재 설치의 준비된 `bin/elesim-dev` 안에서 실행한다.
CPU만 필요한 검사는 의존성이 있는 host에서도 부분 검증으로 실행할 수 있다.

| 목적 | 명령 |
| --- | --- |
| 필수 소프트웨어 회귀 | `python3 workbench/tools/quality/check.py --group required` |
| 도구·연구 회귀 | `python3 workbench/tools/quality/check.py --group extended` |
| 사용 가능한 벤치·입력 확인 | `python3 -m workbench.tools.bench list` |
| 실험 계획 / 실행 | `python3 -m workbench.tools.bench plan\|run <recipe> ...` |
| 결과 검증 / 보고 | `python3 -m workbench.tools.bench verify\|report <run-dir>` |
| 결과 보관 / 수집 / 색인 | `python3 -m workbench.tools.bench pack\|collect\|index ...` |
| 격리 release 생성 / 검증 | `python3 workbench/tools/release/build.py`, `python3 workbench/tools/release/verify.py dist/releases` |

`plan|run` 같은 표기는 각각 별도 명령이라는 뜻이다. 셸에 `|`를 입력하지 않는다.

## 유지하는 벤치

| 분야 | recipe | 하는 일 |
| --- | --- | --- |
| 보행 | `walking-core` | production 보행의 이동·넘어짐·반복 정지 확인 |
| 보행 중 시선 | `walking-gaze` | 자체 loopback DDS Sim/Pilot로 gaze 조건 비교 |
| Sim 성능 | `sim-step` | 전체 trial 처리 비용, 선택적 camera/arm 작업 |
| 시각 보정 | `visual-tilt`, `visual-tilt-replay` | 4DOF 관절 오차·기준면 방향 보정 / 원자료 재평가 |
| 다중 시점 | `multiview-scan`, `multiview-compare` | 안전한 시야 획득·통합 / 촬영 수와 형상 품질 비교 |
| 형상 완성 | `completion-adapointr`, `completion-replay` | AdaPoinTr 추론 / 형상·속도·원관측 보존 평가 |

이 목록은 유지하는 도구 목록이다. 각 기능의 실기동 검증 여부와 미통과 항목은
[`docs/status.md`](../docs/status.md)에 기록한다.

## 남겨 둔 보조 도구

모듈 이름 앞에 `python3 -m workbench.research.`를 붙여 실행한다.
모든 명령의 입력은 `--help`에서 확인한다. 무거운 GPU/ROS 명령은 해당 과학
환경에서 실행하고, 단순 분석을 위해 host에 의존성을 추가하지 않는다.

| 모듈 | 유지 이유 |
| --- | --- |
| `analysis.analyze_contact_metrics` | 발 접촉·미끄럼·힘 오차 CSV 분석 |
| `analysis.walking_motion` | 실제 이동 방향·속도 분석 |
| `analysis.walking_swing` | 발 궤적·스윙 추종 분석 |
| `analysis.arm_scan_plot` | 현재 multiview 원자료의 RGB/팔 자세/통합 점군 그림 |
| `analysis.visual_tilt_plot` | 현재 시각 보정 평가 그림 |
| `debug.profile_fixed_foot` | 몸통 고정 상태에서 발 추종 원인 분리, 접촉 회귀용 trace 획득 |
| `debug.sim_camera_preview` | 실제 DDS RGB-D 수신·깊이 미리보기 |
| `debug.probe_scene_rebuild` | 실행 중 OBJ 교체·controller/state 보존·카메라 복구·rollback 실측 |
| `analysis.render_scan_cycle_demo` | 실제 UI WebRTC 녹화와 측정된 단계 기록으로 1배속 MP4 구성 |
| `debug.scan_cycle_native` | 격리 scan Pilot의 private PID namespace에서 네이티브 크래시 수집 |
| `debug.pilot_native_repro` | 설치된 Pilot 이미지에서 직렬화와 native/safe 주기 스택 진단을 분리 비교 |
| `debug.trace_viewer` | runtime trace JSONL 조회 |
| `experiments.adapointr_inputs` | 보정된 센서 점군을 물체별 추론 입력으로 준비 |
| `experiments.adapointr_gpu_smoke` | 모델/checkpoint 및 GPU FPS·gather adapter 확인 |

## 벤치 내부에서 쓰는 파일

`research/`에는 아래 소스/시나리오 파일과 package `__init__.py`를 둔다.
위의 사용자 진입점에서 필요한 worker/helper이며, 각각을 새 실행 경로로
복제하지 않는다.

| 경로 | 파일 |
| --- | --- |
| `research/analysis/` | `analyze_contact_metrics.py`, `analyze_walking_metrics.py`, `arm_scan_compare.py`, `arm_scan_plot.py`, `evaluate_scan_cycle.py`, `profile_scan_models.py`, `render_scan_cycle_demo.py`, `visual_tilt_plot.py`, `walking_motion.py`, `walking_swing.py` |
| `research/debug/` | `profile_fixed_foot.py`, `profile_sim_walking.py`, `probe_scene_rebuild.py`, `pilot_native_repro.py`, `scan_cycle_native.py`, `sim_camera_preview.py`, `trace_viewer.py` |
| `research/experiments/` · 보행 | `batch_readiness.py`, `config_overlay.py`, `run_walking_baseline_batch.py`, `walking_baseline.py`, `walking_scenarios.py`, `walking_suite.py`, `walking_suites.json` |
| `research/experiments/` · 시각 | `vision_arm_scene.py`, `vision_reconstruct.py`, `vision_tilt_evaluate.py`, `arm_scan.py`, `arm_scan_scene.py`, `arm_scan_evaluate.py` |
| `research/experiments/` · 완성 | `adapointr.py`, `adapointr_inputs.py`, `adapointr_infer.py`, `adapointr_evaluate.py`, `adapointr_gpu_smoke.py` |
| `research/support/` | `adapointr_backend.py`, `adapointr_ops.py`, `adapointr_profiles.py`, `output.py`, `pilot_runtime.py`, `scan_cycle_install.py`, `scan_cycle_run.py`, `ssh_mss_proxy.py`, `thread_stacks.py`, `worker.py` |

정상 설치의 Pilot/Sim/UI 이미지 세 개로 실행하는 scan 운영 경로 검사는
[`tests/system/scan_cycle.md`](tests/system/scan_cycle.md)를 따른다. 역할별 probe와
녹화 helper는 `tests/system/`에 두며 sibling deployment를 import하지 않는다.
이는 고정 암의 실제 DDS/RGB-D/WebRTC 연결 검사이며 기존 품질 벤치나 전체
canonical gate를 대신하지 않는다.

`tools/quality/`, `tools/bench/`, `tools/release/`는 계속 유지한다. 이 밖의 유지
도구는 `tools/setup_preview.py`, `tools/connection_manager_preview.py`와 Jetson
cuRobo용 `tools/setup_curobo_jetson.sh`, `tools/patch_curobo.py`다. runtime 기능은
이 정리의 대상이 아니다.

## 보관 위치

옛 실행기·그림 생성기·중단한 실험 CLI **24개**는
[`archive/retired-tools-20261027.zip`](archive/retired-tools-20261027.zip)에 원본 그대로
압축 보관했다. ZIP은 Git에서 제외하고, [목록·SHA256](archive/index.json)과
[보관 규칙](archive/README.md)은 버전 관리한다. 24개 중 5개는 앞선 통합에서
이미 제거한 entrypoint를 HEAD에서 회수한 것이고, 19개는 이번에 옮겼다.

회귀검사에 필요한 옛 계산부 여섯 개는 `tests/research/support/`에 CLI 없이
보존했다. 계산 함수의 AST와 기존 assertion을 유지하며, 벤치/runtime은
테스트 helper나 archive를 import하지 않는다. 원자료는 계속
`evidence/generated/`, 선별된 보고서·사진은 `evidence/curated/`에 둔다.
