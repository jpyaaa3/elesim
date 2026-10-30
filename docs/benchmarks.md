# Workbench 검증과 벤치

소프트웨어 회귀의 실행 목록은 `workbench/tools/quality/check.py`가 소유한다.
무거운 실험은 `workbench/tools/bench/`의 schema-v1 실행기로 관리한다.
계산 함수는 `workbench/research/`에 남고 runtime/release는 workbench를 import하지 않는다.

## 소프트웨어 검사

현재 설치의 생성된 개발 wrapper가 준비된 경우:

```bash
<prefix>/bin/elesim-dev python3 workbench/tools/quality/check.py --group required
<prefix>/bin/elesim-dev python3 workbench/tools/quality/check.py --group extended
```

선택적 GUI도 동일 매트릭스와 workflow 선택을 쓴다. `안정`이라는 별도 합격
목록이나 모든 역할에 대한 일괄 PYTHONPATH는 없다. 각 역할은 자기 source root로
실행된다. 보정 범위를 여러 역할/URDF/RL 사이에서 비교하는 기존 세 검사는
`arm-operating-range` 시스템 검사로 옮겼다. assertion은 유지했다.

```bash
python3 -m workbench.tools.quality.check --selection required --list
python3 -m workbench.tools.quality.test_gui --runner dev \
  --dev-wrapper <현재-prefix>/bin/elesim-dev
```

개발 attachment를 찾거나 새로 만드는 대체 경로는 없다. 준비가 안 된 경우
host에서 실행 가능한 검사만 부분 근거로 기록한다.

## 다섯 벤치 분야

| 분야 | 실행 recipe | 평가 범위 |
| --- | --- | --- |
| 보행/gaze | `walking-core`, `walking-gaze` | production controller의 이동/넘어짐/정지, 별도 DDS gaze trial |
| Sim 성능 | `sim-step` | physics/MPPI/arm hold, 선택적 camera worker, 전체 trial 처리 시간 |
| 시각 보정 | `visual-tilt`, `visual-tilt-replay` | 임의 4DOF 오차와 기준면 방향 보정, FK 원점 보존 |
| 다중 시점 | `multiview-scan`, `multiview-compare` | 연속 self-collision 검사, 관측 통합, 촬영 수와 형상 품질 |
| 형상 완성 | `completion-adapointr`, `completion-replay` | 센서 기반 입력, checkpoint 검증, 형상/속도/원관측 보존 |

```bash
python3 -m workbench.tools.bench list
python3 -m workbench.tools.bench plan sim-step \
  --input config=payload/config/sim/config.yaml --input data=payload/data \
  --param cameras=true --param 'command=[0.8,0,0]'
```

`plan`은 실행하지 않고 입력/설정/각 stage의 argv와 측정 범위를 보여준다.
`--param` 값은 JSON이다. 모르는 이름과 사용하지 않는 옵션, 잘못된 타입은
거부한다. 숨겨진 sample/iteration 변경은 없으며 실제 적용 dt/solver 설정은
probe의 `configuration` 레코드에 저장된다. 진단 개입이 필요한 별도 연구는
해당 debug worker와 명시적인 config를 사용하고 production baseline과 구분한다.

## 로컬 CPU와 설치된 GPU 실행

CPU 해석 벤치의 전체 획득/복원/평가:

```bash
python3 -m workbench.tools.bench run visual-tilt \
  --input config=payload/config/sim/config.yaml --input data=payload/data \
  --output workbench/evidence/generated/<새-run-id>
```

GPU 실행은 대상 호스트의 checkout에서 현재 설치와 인스턴스를 지정한다.
다음 예는 기존 원격 GPU2의 UUID다.

```bash
python3 -m workbench.tools.bench run walking-core \
  --install /home/hckang/ws/newsim --instance test \
  --gpu GPU-a40782b2-881c-e285-2a8e-66cae9162a01 \
  --input config=payload/config/sim/config.yaml --input data=payload/data \
  --output workbench/evidence/generated/<새-run-id>
```

실행기는 ownership의 UUID/Compose project/Docker context/Engine을 확인하고,
선택 인스턴스의 실제 image ID를 고정한다. 생성된 `elesim-compose`로 동일
프로젝트의 `run --rm --no-deps --pull never` 작업만 만든다. 빌드·pull·등록·정상
서비스의 up/down은 실행하지 않는다. 정상 컨테이너의 전후 목록을 기록한다.
진단 자원은 고유 bench ID와 실제 container ID로 확인한 뒤 정리한다.

입력/코드는 읽기 전용, root filesystem은 읽기 전용, network는 `none`이다.
기존 role의 credential/state/device/network 설정을 Compose `!override`/`!reset`으로
제거하고, 병합 결과와 실행 중 실제 mount/network/device를 각각 확인한다.
GPU reservation은 UUID 하나이고 EGL은 컨테이너의 논리 장치 0을 쓴다.
host index 2를 컨테이너의 `CUDA_VISIBLE_DEVICES=2`로 재적용하지 않는다.

2026-10-30 서버 경로에서는 SSH 키 교환과 긴 명령이 패킷 크기에 따라 멈췄다.
호스트 MTU나 전역 SSH 설정을 바꾸지 않는 진단 경로는 checkout에서 다음과 같다.
이 helper는 해당 TCP 소켓의 MSS만 1024로 지정하며 기본 SSH 알고리즘, 인증과
기존 host-key 검증은 그대로 OpenSSH가 수행한다.

```bash
ssh -o 'ProxyCommand=python3 -m workbench.research.support.ssh_mss_proxy %h %p' \
  -o StrictHostKeyChecking=yes hckang@100.74.222.24
```

새 OBJ의 캐시 영향을 비교하는 수동 진단은
`workbench.research.debug.profile_sim_walking --scene-manifest <scene.json>`을 사용한다.
현재 runtime과 동일한 artifact importer로 물리 충돌 및 카메라 복제 씬에 추가하며,
첫 프레임 대기와 물리 OBJ import 시간을 별도로 기록한다. 기존 씬만 준비한 캐시를
복사해 각 새 OBJ의 첫 실행을 분리하고, 동일 OBJ의 반복 실행은 그 복사본을 재사용한다.
이 명령은 새 프로세스의 씬 재구성이며, 같은 프로세스의 `InteractiveScene.rebuild`
시간이나 운영 앱의 hot-swap 완료를 입증하지 않는다.

소비자에게는 필요한 research Python import와 센서/명목 모델만 마운트한다.
전체 checkout/evidence나 producer의 scene/catalog 코드를 넘기지 않는다.
정답 데이터는 producer/evaluator에만 준다. 로컬 CPU 실행은 프로세스 구분이며
filesystem 접근 격리의 증거는 아니다.

실행할 runtime 버전은 **인스턴스가 선택한 이미지**다. 새 코드 검증 때는 push,
대상 checkout 갱신, 정식 update 및 인스턴스의 선택 release를 확인한다.
`update`만으로 기존 pin이 바뀌었다고 가정하지 않는다. manifest에는 Git HEAD,
dirty 상태, 연구 코드 hash, 실제 image ID/label, worker 내부 runtime source hash,
설치된 과학 패키지 버전, GPU UUID와 입력 config/model/checkpoint hash가 남는다.
YAML `extends` 파일도 hash/읽기 전용 mount에 포함하고 실행 후 입력 변경을
재검사한다. Worker는 default/remote로 해석한 Sim 입력 설정도 보존한다.
Probe가 명시적으로 바꾼 실제 설정은 별도의 `configuration` 레코드로 확인한다.

`walking-gaze`는 여러 앱이 필요한 연구이므로 준비된 **현재 생성 dev attachment**
안에서 실행한다. 설치된 단일 역할 image 안에 sibling 앱을 주입하지 않는다.
새 loopback 전용 DDS system/endpoint/topic을 만들어 자체 Sim만 제어한다.
Sim source RGBD와 Pilot broker topic은 서로 다르다. 카메라 작업은 다른 GPU가
보이지 않는 dev/Compose GPU 설정을 요구한다. `use_hardware=true` 설정은
계산 worker 진입 전에 거부한다.

## 시간과 합격의 의미

- `walking-core`/`sim-step` duration은 **simulation seconds**다.
  시작/정착/보행/정지를 포함한 실제 stage wall time을 별도로 기록한다.
- DDS `walking-gaze` duration은 **wall seconds**다. 각 trial은 실제 경과한
  simulation time도 기록하고 그 구간의 이동·넘어짐·기록 끝부분을 확인한다.
- `sim-step`의 짧은 step timer는 command/arm hold/physics/camera dispatch를
  포함한다. 진단 상태 readback/JSON 출력은 그 밖에 있다. `whole_loop_step_ms`는
  이를 포함한 전체 trial wall time/실제 step 수다. 기본 budget은 10 ms다.
- startup은 별도이고 trial의 2 sim초 정착 시간과 이후 시간을 따로 기록한다.
  정착 시간을 첫 보행 MPPI JIT 완료의 증거로 취급하지 않는다.
- 이 physics probe에는 실제 DDS telemetry와 연결된 WebRTC viewer가 없다.
  `--param cameras=true`는 render/dispatch worker를 포함하며 서비스 전체
  성능을 증명하지 않는다. DDS gaze 벤치는 Sim media service가 있지만 viewer는 없다.
- scan은 각 episode의 monotonic elapsed/capture wait를 기록한다.
  sensor timestamp, 복사 시각, ZIP member mtime은 처리 시간으로 사용하지 않는다.

`manifest.json`의 `complete`와 `accepted`는 별개다. 실행이 끝났어도 형상/보행/
성능 기준을 넘지 못하면 실패다. `run`과 `verify`의 종료 코드는 0=합격,
1=실행 완료·기준 미달, 2=실행 미완료/입력·환경·worker 오류다. 전체 함수의
예외를 평가 실패로 포장하지 않는다. 실패 원자료도 수집/보존할 수 있다.

```bash
python3 -m workbench.tools.bench verify <run-dir>
python3 -m workbench.tools.bench report <run-dir>
```

`verify`는 산출물 hash와 저장된 판정을 검증한다. `report`는 명시적인 보고
명령이며 실패 결과도 정상적으로 표시한다. 수학적 평가를 다시 수행하려면
`*-replay`/`multiview-compare` recipe로 새 실행을 만든다. 기존 evaluator의 직접
CLI도 기본은 strict이며 보고만 할 때 `--report-only`를 지정한다.

## 수집과 오래된 증거

대상 호스트에서 archive를 만들면 SHA256이 출력된다.

```bash
python3 -m workbench.tools.bench pack <run-dir> --output <archive.zip>
python3 -m workbench.tools.bench collect <이미-전송한-archive.zip> \
  --sha256 <pack에서-출력한-SHA256> --output <새-수집-dir>
```

저장된 Tailscale SSH endpoint를 통한 직접 수집도 지원한다. 설치의 실행 중인
manager를 명시하고, Connections의 기존 host fingerprint/인증 경로를 재사용한다.
remote command 실행, 업로드, SSH pin 변경이나 새 network service는 없다.

```bash
python3 -m workbench.tools.bench collect /원격/절대경로/archive.zip \
  --sha256 <SHA256> --output <새-수집-dir> \
  --install <로컬-prefix> --manager <현재-manager-container> \
  --topology <로컬-prefix>/connections/test/topology.json --host compute
```

각 archive와 해제한 모든 산출물을 검증한다. 경로 탈출/중복 entry/symlink와
기존 수집 폴더 덮어쓰기를 거부한다. 부분 수집이나 잘못된 archive는 합격으로
승격되지 않는다. 기존 run을 수정해 재사용하지 않고 새 output을 쓴다.

```bash
python3 -m workbench.tools.bench index workbench/evidence/generated \
  --output workbench/evidence/generated/index.json
```

과거 생성물은 Git 제외 원자료다. 색인은 소스 사본의 동일 내용과 용량을
묶어 보여주며, 이전 실행에 manifest가 없으면 `legacy-unverified`로 남긴다.
색인에 있는 declared acceptance는 hash 검증 결과가 아니다. 파일을 삭제하거나
경로를 옮기지 않아 기존 증거 링크/다운로드 경로를 보존한다.

기존 full/resume/ablation/preview shell 및 깨진 preview validation entrypoint 다섯 개는
`walking_suites.json`의 조건과 하나의 suite worker로 합쳤다. resume의 trial 수/시작
번호는 `trials`/`trial_start`로 명시한다. 실제 구현이 없던 standoff/video/preroll/
host-restart 옵션은 제거했다. 중복 수동 demo도 로컬 archive로 옮겼다.
옛 양수 linear arm preset은 현재 joint 범위에서 거부하며 clamp해서 다른 자세의
시험으로 바꾸지 않는다. 안전 회귀나 실제 원자료는 삭제하지 않았다.

현재 사용하는 도구만 [Workbench 목록](../workbench/README.md)에 정리했다.
옛 실행기/그림/실험 CLI 24개는 Git에서 제외된 로컬 ZIP으로 보관하고
[hash 목록](../workbench/archive/index.json)을 남겼다. 회귀에서 쓰는 옛 계산부는
`tests/research/support/`로 옮겨 archive 없이 검사한다. 과거 문서의 보관된
명령은 현재 진입점으로 안내하지 않는다.

## 이번 개편의 검증 경계

CPU 해석 시각 보정은 전체 pipeline을 통과했고 같은 원자료의 재평가 JSON도
정확히 같았다. CPU scan의 producer/consumer/독립 충돌·형상 평가도 끝까지
실행했다. 형상 미달은 실패로 남았다. 기존 GPU scan 비교는 112→39회 촬영과
paired 품질 5/6이라는 같은 결과로 exit 1을 반환한다.

실제 Docker Compose client의 config 병합, 잘못된 GPU/정답 mount 거부,
작업 중단·고아 camera child 정리, 무관한 프로세스 보존, archive round trip을
검사했다. 새 executor의 원격 GPU/Genesis/실제 DDS gaze/manager SSH 수집과
GUI 표시 자체는 별도 live gate다. 생성 dev 환경 준비 미완료인 기존 사유로
canonical required/extended, real-RMW와 isolated release 전체 게이트는 미실행이다.
Host quality 검사에서 기존 세 클래스의 readability 한도 초과는 계속 실패로
표시하며 production 클래스나 기준을 이 개편에서 변경하지 않았다.
