# Live Code Map

EleSim 코드맵은 사람이 관리하는 메서드 목록이 아니라 **현재 worktree에서 매번 생성되는 읽기 전용 그래프**다. 따라서 코드 변경과 문서 스냅샷이 서로 어긋나는 별도 source of truth를 만들지 않는다.

## 실행

선택적 개발 attachment의 영속 `elesim-dev` 환경에서 저장소 루트 기준으로 실행한다.

```bash
elesim-dev python3 misc/tools/code_map/app.py
```

출력된 token 포함 loopback URL을 브라우저에서 연다. 원격 호스트라면 다른 설치 GUI와 마찬가지로 SSH local forwarding을 사용한다. 일반 설치, release image, 사용자 `bin/`에는 이 개발 도구를 포함하지 않는다.

```bash
ssh -N -L 8767:127.0.0.1:8767 user@developer-host
```

고정 포트는 `--port 8767`로 지정한다.

## 분석 범위

분석기는 project module을 import하거나 실행하지 않고 Python 표준 라이브러리 `ast`만 사용한다. Git이 추적하는 Python 파일과 ignore되지 않은 새 Python 파일에서 다음을 추출한다.

- role, package, module, class, function, method, console entrypoint
- import, inheritance, decorator, direct call
- callback, thread, executor, async task 등록
- 함수 인자/반환 포트, 대입·상태 변경, 분기·반복·예외 지점
- ImGui 조작 지점과 UI operator 호출
- test에서 production symbol로 향하는 정적 참조
- DDS/RGB-D, lease/session, WebRTC, Genesis, hardware I/O 의미 표식
- 보수적인 orphan 후보(`orphan_candidate`); 자동 삭제 판정은 아님

노드 ID는 role, repository-relative path, qualified name을 결합한다. 파싱 실패는 이전 결과로 숨기지 않고 자홍색 `unparsed` 노드가 된다. HEAD에서 삭제된 파일은 빨간 tombstone으로 남는다. 추가는 녹색, 수정은 주황색이다.

초기 분석은 저장소 크기에 따라 수 초~수십 초가 걸릴 수 있으며, 동일 worktree 캐시 검사는 약 1초를 목표로 한다. 캐시는 `.elesim/analysis/code-map/snapshot.json`에 저장되며 Git 대상이 아니다. schema version과 전체 입력 digest가 일치할 때만 재사용한다.

## 화면 사용

기본 구조 지도는 700개 노드로 제한된다. `플로우 가족`과 `워크플로`를 고르면 서버가 현재 worktree에서 해당 실행 경로를 다시 계산해 입력·출력·예외 포트를 포함한 bounded graph를 반환한다. role, symbol/path 검색, HEAD 변경 상태, 정적/관측 edge와 표현 깊이를 조합해 범위를 좁힌다.

`테스트 코드 숨기기`는 기본 활성화되어 `test/`, `tests/`, `test_*.py`, `*_test.py`의 노드와 연결선을 구조 지도와 선택한 플로우 양쪽에서 제외한다. 테스트와 production 연결을 조사할 때만 체크를 해제한다.

```text
role → package → module → class → function/method
```

실행 흐름은 역할과 처리 phase별 swimlane으로 표시된다. 플로우를 선택하면 entry에서 시작하는 control/data edge를 우선해 깊이별로 상→하 레이어를 만든다. 노드 안의 이름과 메타데이터는 가로쓰기를 유지한다. 표현 수준은 다음과 같다.

- `개요`: entry와 프로세스·DDS/WebRTC·비동기 경계 중심
- `실행 흐름`: 개요에 분기·병합·루프·예외 처리 지점을 더한 기본 보기
- `전체 근거`: helper, 외부 호출, data/state write를 포함한 원본 bounded slice

기본 보기에서 의미 경계가 없는 helper 연쇄와 같은 경계에서 갈라지는 세부 leaf는 `N개 세부 호출` 노드로 가역적으로 접힌다. 노드를 누르면 해당 묶음만 펼쳐지고 `세부 호출 다시 접기`로 초기화한다. 원본 symbol ID와 source/diff 근거는 버리지 않는다.

루프·분기는 ELK의 cycle breaking/crossing minimization으로 같은 그래프 안에 남으며, 붉은 점선은 source로 되돌아가는 back-edge다. 연결선은 전기 회로도처럼 수직·수평 구간과 90도 코너만 사용한다. 정상 흐름은 레이어 사이 배선 채널을 공유하고, back-edge는 노드 오른쪽 외곽 채널로 우회한다. 호출, data/state write, contract, async/thread, 예외는 서로 다른 색과 선으로 표시한다. 정적 분석은 실제 총실행순서를 알 수 없으므로 노드는 가짜 `#step` 대신 같은 깊이에 같은 `S(stage)`를 표시한다. `upstream` 보기에서도 실제 source→sink 방향은 상→하로 유지된다. 이는 basic-block 단위 실행 증명이나 실제 branch 선택 결과가 아니라, 현재 AST/DDS 계약에서 재구성한 방향성 flow view다.

함수 노드를 선택하면 parameter port, return site, state write, branch/loop/raise 근거와 source/diff를 우측에 표시한다. 노드를 선택하면 파일/행, caller/callee 수, decorator/base/문자열 근거, source 문맥과 HEAD diff를 표시한다. source API는 repository containment와 symlink를 검사하고 UTF-8, 128 KiB, 500줄로 제한한다.

## 플로우 카탈로그

카탈로그는 사람이 관리하는 메서드 목록이 아니다. 현재 소스의 UI widget, protocol operator registry, simulation command, DDS contract와 자율 lifecycle/media/safety 심볼에서 매번 재생성된다.

- `action`: 버튼·체크박스·슬라이더·입력 등 사용자 조작 진입점
- `system`: discovery, authority, simulation, RGB-D, WebRTC, Robot safety 같은 백그라운드 진입점
- `overview`: 기존 상위 흐름을 탐색용 family 카드로 제공

각 entry는 공통 처리 경로를 가리키는 template과 bounded node slice를 가진다. coverage는 정적 경로의 resolution gap 유무이며 runtime 성공률이나 wire-contract 검증 결과가 아니다.

## UI Workflow Explorer

`Mock UI 열기`는 `elesim-ui`를 실행하거나 복제하지 않는 읽기 전용 interaction twin을 연다. 현재 소스에서 발견한 ImGui 컨트롤을 파일과 렌더 함수별로 묶고, 각 컨트롤을 분석기가 생성한 `action` workflow ID에 직접 연결한다. 버튼·체크박스·슬라이더·입력 필드를 조작하면 DDS 메시지나 application 메서드를 호출하지 않고 코드맵의 해당 workflow만 선택한다.

컨트롤 ID는 원본 path, qualified method, source line과 widget metadata에서 유도된다. 동적 f-string/변수 라벨은 source expression과 `dynamic label` 표시를 남기며, branch/loop 내부 컨트롤은 `conditional`로 표시한다. `helpers.py`에서만 발견된 일반화된 위젯은 실제 패널이라고 가장하지 않고 helper surface로 분리한다. `/api/ui-map`은 이 구조를 현재 snapshot에서 매번 생성하므로 별도 수동 Mock UI 명세와의 drift가 생기지 않는다.

## 정적 근거

정적 edge는 점선이며 `exact`, `inferred`, `unresolved` confidence를 가진다. Code Map은 현재 source와 계약에서 얻은 정적 근거만 표시하며, 실행되지 않은 경로를 dead code라고 판단하지 않는다. `PeerEnvelope.trace_context` 필드는 wire compatibility를 위해 유지하지만 Code Map은 외부 trace backend를 조회하지 않는다.

뷰어는 ELK.js 0.9.3을 `web/vendor/`에 고정해 CDN과 별도 프런트엔드 빌드 없이 layered/port layout을 사용한다. 라이선스와 bundle hash는 `web/vendor/NOTICE.md`에 기록한다.

## 보안과 변경 금지

- HTTP server는 loopback 외 주소에 bind하지 않는다.
- 모든 static/API/SSE 요청은 무작위 token을 요구한다.
- POST/PUT/PATCH/DELETE는 405로 거부한다.
- 프로젝트 import, 코드 실행, 파일 저장, Git mutation endpoint는 없다.
- DDS wire schema, runtime role, release artifact를 수정하지 않는다.

## 검증

```bash
elesim-dev python3 -m pytest misc/tools/code_map/tests
elesim-dev python3 misc/tools/quality/check.py --check code-map-tools
```

테스트는 AST 문법 요소, 함수 포트·분기·반복·예외, UI action drift, flow slice/cycle, cache roundtrip, semantic workflow, token/read-only HTTP, path traversal, symlink와 크기 제한을 검증한다.
