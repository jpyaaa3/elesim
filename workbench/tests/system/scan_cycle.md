# 고정 암 Sim 1-cycle 운영 경로 검사

정상 설치로 만든 Pilot/Sim/UI 이미지 세 개를 분리해 실행한다. 각 probe는
자기 역할만 import할 수 있어야 한다. 실제 Genesis, 관절 피드백, DDS RGB-D,
operator RPC, WebRTC 수신과 `load_scan_result` backend를 사용한다. Native UI
창을 자동 조작하지는 않는다. 현재 완료 범위와 수치는
[`docs/status.md`](../../../docs/status.md)만 소유한다.

`probe_scan_cycle_sim.py`는 고정 암을 배치하고 실제 카메라 앞 0.4 m에 반경
45 mm의 빨간 구 또는 높이 140 mm인 같은 반경의 원통을 둔다.
`launch --shape sphere|cylinder|jar`로 선택하며 기본값은 sphere다. `jar`는
높이 220 mm / 최대 지름 120 mm, 좁은 목·돌출 lip·속이 빈 몸통을 가진 직접
생성한 시험용 mesh다. ShapeNet 개별 sample을 가져온 것은 아니다. Physics와
두 camera scene은 같은 content-addressed OBJ를 쓰며 convexify/decimate하지 않는다.
정답은 Sim의 사후 평가 파일에만 남긴다. Pilot에는
정답 파일, 실제 camera pose 또는 물체 mesh를 마운트하지 않는다. Pilot은
측정 q/명목 FK와 실제 RGB-D로 시야를 선택한다. `use_hardware=false`,
`use_go2=false`이며 물리 Robot에는 연결하지 않는다.

UI probe는 실제 피드백으로 torque/claw 상태를 확인한 뒤 3-view scan을
시작한다. 시야 두 개 이상, 관측점, 동일한 Sim boot의 scene publication을
요구한다. Scan 제어권 반환 뒤 기존 operator 명령으로 원본을 숨기고 두
카메라의 빨간 픽셀이 사라져야 한다. 이어 같은 UI Apply backend로 bundle을
적용하고, 응답 이후 네 frame version 이상 지난 비어 있지 않은 두 화면을
확인한다. Sim은 원본의 실제 위치, 기존 entity q/velocity/root pose, camera
PID와 rebuild 이후 frame timestamp를 독립 기록한다.

현재 driver 기본값 `--pipeline-mode integrated`는 HSV + FK + 촬영별 FloorTilt +
실제 AdaPoinTr AP-PS55 + 20 mm voxel surface다. 기존 기준선은
`--pipeline-mode baseline`으로 명시한다. Source archive와 checkpoint는 이전
실험의 고정 파일을 read-only로 붙이고 hash를 검사한다. Host Python에는
패키지를 설치하지 않으며 설치된 tools 이미지에서 YAML을 읽는다.
`geometry_complete_claimed=false`, `needs_more_observation=true`를 유지한다.
관측점의 표면 오차와 거친 mesh vertex 오차는 별도로 보고한다. 원통 평가는
world-Z 축의 유한 원통 옆면·끝면·모서리를 모두 포함한다. 형상 완성,
일반 instance segmentation, GO2 이동, 동적 집기 및 다중 호스트 파일 전송의
수용시험은 아니다. 이 검사는 한 호스트의 사설 internal bridge와 공유 artifact
root를 사용하며 보안/NAT 시험을 대신하지 않는다.
통합 HSV adapter는 25 mm depth 연결성으로 detector `min_area_px` 미만의 작은
깊이 성분도 제외한다. 고립된 background pixel은 제거하고 넓은 jar 내벽은 유지한다.
Jar 평가는 Sim-private r,z profile의 회전 표면까지 거리를 독립 계산하며,
외벽·내벽·rim·bottom을 포함하고 rotation-axis의 면적 없는 선은 제외한다.

통합 평가는 각 accepted view의 floor correction이 적용됐고 actual model이
strict-load/추론하여 8,192개 predicted point를 냈는지 검사한다. `complete`는
이 연결과 실제 scene 적용 완료, `geometry_accepted`/`accepted`는 별도의 형상
기준을 뜻한다. 기준은 observed p95 15 mm, predicted/mesh vertex p95 45 mm다.
기존 baseline의 `complete`는 계속 형상 기준까지 요구한다. 기울기 오차 주입은
`--pose-bias-deg 5`로 Pilot의 private hand-eye 추정 회전에만 적용하며, Sim의
카메라/정답/관절 제어 calibration은 바꾸지 않는다.
`--view-budget 12`로 실제 UI 기본 예산도 검사할 수 있다. 안전한 추가 시점이
없으면 더 적은 관측으로 종료할 수 있으며 예산 수를 완전 표면 수집률로 해석하지 않는다.

## 역할별 파일

- `probe_scan_cycle_sim.py`: 실제 fixture와 scene 적용 전후 상태 기록.
- `probe_scan_cycle_pilot.py`: 실제 scan 호출의 피드백/예외 및 Python fatal stack 기록.
- `probe_scan_cycle_ui.py`: operator/Sim session 호출, 두 WebRTC decoder와 결과 판정.
- `scan_cycle_recording.py`: 실제 decoder frame을 wall-clock PTS로 10 fps H.264 기록.
- `research/analysis/evaluate_scan_cycle.py`: 수집 후 Sim-private 정답을 읽는 독립 평가.
- `research/analysis/render_scan_cycle_demo.py`: 원본 녹화 두 개와 단계 기록으로
  1배속 자막 MP4 구성. 촬영되지 않은 geometry/frame은 생성하지 않는다.

## 이 서버 작업의 재현과 보존

아래 driver는 이번 작업의 서버 주소, 설치 UUID/Engine/GPU와 경로에 맞춘
유지 도구다. 범용 설치 명령으로 안내하지 않는다. 기존 설치는
`/home/hckang/ws/newsim`, 별도 검증 설치는
`/home/hckang/ws/elesim-scan-cycle-20261030/install`이다. 정상 installer가
생성한 Compose/Dockerfile 및 release publication/instance 등록을 사용한다.
기존 개발 컨테이너를 재생성하거나 별도 개발 이미지를 만들지 않는다.
자동 image collection을 실행하는 update wrapper도 호출하지 않는다.

로컬 저장소 루트에서:

```bash
python3 -m workbench.research.support.scan_cycle_install prepare
python3 -m workbench.research.support.scan_cycle_run upload
```

`prepare`는 작업 root가 없을 때만 실행한다. 이미 존재하는 이 작업에는
다시 실행하지 않는다. 소스 수정 후에는 먼저 commit하고 `refresh`로 그
commit의 archive를 보낸다. 원격 build가 `preparation.json`에 성공을 기록한
뒤 서버에서 아래 명령을 순서대로 실행한다. `N`은 사용하지 않은 1..24다.

```bash
python3 /home/hckang/ws/elesim-scan-cycle-20261030/probes/scan_cycle_run.py publish
python3 /home/hckang/ws/elesim-scan-cycle-20261030/probes/scan_cycle_run.py launch --attempt N
```

`cycle-NN/ui-output/ui.json`의 `complete=true`와 video 완료를 확인한 뒤:

```bash
python3 /home/hckang/ws/elesim-scan-cycle-20261030/probes/scan_cycle_run.py evaluate --attempt N
python3 /home/hckang/ws/elesim-scan-cycle-20261030/probes/scan_cycle_run.py stop --attempt N
```

`stop`은 이 설치 UUID/system/image를 대조한 세 컨테이너만 중지하고 원래
inventory와 비교한다. 컨테이너·이미지·volume·원본 cache를 삭제하지 않는다.
새 실행은 시작 직전의 전체 container inventory도 따로 대조한다. Probe는 각
실행 폴더에 고정 복사하며 upload가 교체하는 기존 파일은 SHA-256별 history에
보존한다. 이후 도구 변경이 이미 생성한 실행의 probe를 바꾸지 않는다.
실패한 시도도 남긴다. Cache는 새 instance에 복사하며 기존 cache를 비우지
않는다. 같은 fixture의 후속 실행은 mesh 전처리 cache도 재사용할 수 있으므로
첫 삽입과 반복 삽입의 지연을 구분한다.

로컬 수집과 영상 구성:

```bash
python3 -m workbench.research.support.scan_cycle_run collect --attempt N
python3 -m workbench.research.analysis.render_scan_cycle_demo \
  --input workbench/evidence/generated/scan-cycle-20261030/cycle-NN \
  --output workbench/evidence/curated/sim/scan_cycle_demo_DATE.mp4 \
  --cache-note 'Warm repeated mesh'
```

`--cache-note`는 실제 조건을 기입한다. 다른 형상에서 처음 생성된 mesh라면
예를 들어 `Warm kernels / new mesh`로 적으며 같은 mesh의 반복과 구분한다.

원격 SSH 전송에만 `ssh_mss_proxy`의 소켓별 MSS를 적용한다. 호스트 MTU나
전역 SSH 설정을 바꾸지 않는다. DDS/WebRTC는 이 SSH stream을 사용하지 않는다.

## 실패 조사

`probe_scan_cycle_pilot.py --periodic-stacks --stack-interval 1`은 Python
thread에서 `sys._current_frames()`로 참조를 소유한 뒤 stack을 기록한다.
원격 driver에서는 `launch --stack-interval .1`로 10 Hz 수집을 켠다. 기본 probe는
fatal handler만 켜며 주기적 수집은 하지 않는다. 이 수집기는 GIL을 영구 점유한
C 함수의 hang을 진단할 수 없으며 그런 경우 아래 외부 GDB를 사용한다.
이전에 사용한 `faulthandler.dump_traceback_later`는 설치된 CPython 3.10에서
139를 분리 재현했으므로 실제 scan probe에서 사용하지 않는다.
`launch --debug-allocator`는 새 Pilot 하나에만 `PYTHONMALLOC=debug`를 설정한다.
기본 실행은 이 설정을 사용하지 않는다. `research/debug/scan_cycle_native.py`
의 원격 `--attempt N` 명령은 실행 중인 이 작업의 Pilot을 검증한 뒤, generated
tools service에서 host GDB 파일을 읽기 전용으로 붙인다. 해당 Pilot의 private
PID namespace만 공유하며 network/Docker socket/host PID namespace를 받지
않는다. 진단 컨테이너에는 이 목적으로만 ptrace 권한을 준다. attach 중 짧은
정지가 시험 타이밍에 영향을 줄 수 있다. 결과를 일반 무계측 실행과 구분한다.
Debugger를 분리한 뒤 role을 중지하고, debugger 컨테이너/core도 삭제하지 않는다.

`research/debug/pilot_native_repro.py`의 서버 `launch --name NAME --mode
none|native|safe --seconds 120 --interval .001`과 `start --name NAME`은 같은
정상 Pilot 이미지에서 DDS/GPU 없이 실제 상태 직렬화를 비교한다. `native`만
명시적으로 이전 watchdog API를 재현한다. 모든 진단 container/원자료는 보존한다.
GDB 도구의 `--run-dir`로 이 진단 폴더를 지정할 수도 있다. 이 인공 부하의 실행
시간은 앱 성능 지표가 아니며 실제 GPU cycle의 결과와 구분한다.
