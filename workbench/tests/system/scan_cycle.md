# 고정 암 Sim 1-cycle 운영 경로 검사

정상 설치로 만든 Pilot/Sim/UI 이미지 세 개를 분리해 실행한다. 각 probe는
자기 역할만 import할 수 있어야 한다. 실제 Genesis, 관절 피드백, DDS RGB-D,
operator RPC, WebRTC 수신과 `load_scan_result` backend를 사용한다. Native UI
창을 자동 조작하지는 않는다. 현재 완료 범위와 수치는
[`docs/status.md`](../../../docs/status.md)만 소유한다.

`probe_scan_cycle_sim.py`는 고정 암을 배치하고 실제 카메라 앞 0.4 m에 반경
45 mm의 빨간 구를 둔다. 정답은 Sim의 사후 평가 파일에만 남긴다. Pilot에는
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

재구성 기준선은 HSV + 명목 FK + ObservedOnly + 20 mm voxel surface다.
`geometry_complete_claimed=false`, `needs_more_observation=true`를 유지한다.
관측점의 구 표면 오차와 거친 mesh vertex 오차는 별도로 보고한다. 형상 완성,
일반 instance segmentation, GO2 이동, 동적 집기 및 다중 호스트 파일 전송의
수용시험은 아니다. 이 검사는 한 호스트의 사설 internal bridge와 공유 artifact
root를 사용하며 보안/NAT 시험을 대신하지 않는다.

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

원격 SSH 전송에만 `ssh_mss_proxy`의 소켓별 MSS를 적용한다. 호스트 MTU나
전역 SSH 설정을 바꾸지 않는다. DDS/WebRTC는 이 SSH stream을 사용하지 않는다.

## 실패 조사

`probe_scan_cycle_pilot.py --periodic-stacks`를 명시해야 매초 stack을 수집한다.
기본 probe는 fatal handler만 켜며 주기적 stack 수집을 하지 않는다.
`launch --debug-allocator`는 새 Pilot 하나에만 `PYTHONMALLOC=debug`를 설정한다.
기본 실행은 이 설정을 사용하지 않는다. `research/debug/scan_cycle_native.py`
의 원격 `--attempt N` 명령은 실행 중인 이 작업의 Pilot을 검증한 뒤, generated
tools service에서 host GDB 파일을 읽기 전용으로 붙인다. 해당 Pilot의 private
PID namespace만 공유하며 network/Docker socket/host PID namespace를 받지
않는다. 진단 컨테이너에는 이 목적으로만 ptrace 권한을 준다. attach 중 짧은
정지가 시험 타이밍에 영향을 줄 수 있다. 결과를 일반 무계측 실행과 구분한다.
Debugger를 분리한 뒤 role을 중지하고, debugger 컨테이너/core도 삭제하지 않는다.
