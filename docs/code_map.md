# Live Code Map

EleSim 코드맵은 사람이 관리하는 메서드 목록이 아니라 **현재 worktree에서 매번 생성되는 읽기 전용 그래프**다. 따라서 코드 변경과 문서 스냅샷이 서로 어긋나는 별도 source of truth를 만들지 않는다.

## 실행

Developer 설치의 영속 `elesim-dev` 환경에서 저장소 루트 기준으로 실행한다.

```bash
elesim-dev python3 misc/tools/code_map/app.py
```

출력된 token 포함 loopback URL을 브라우저에서 연다. 원격 호스트라면 다른 설치 GUI와 마찬가지로 SSH local forwarding을 사용한다. 일반 설치, release image, 사용자 `bin/`에는 이 개발 도구를 포함하지 않는다.

```bash
ssh -N -L 8767:127.0.0.1:8767 user@developer-host
```

고정 포트는 `--port 8767`로 지정한다. Jaeger 기본 주소는 `http://127.0.0.1:16686`이며 `--jaeger-url`도 **loopback 주소만** 허용한다.

## 분석 범위

분석기는 project module을 import하거나 실행하지 않고 Python 표준 라이브러리 `ast`만 사용한다. Git이 추적하는 Python 파일과 ignore되지 않은 새 Python 파일에서 다음을 추출한다.

- role, package, module, class, function, method, console entrypoint
- import, inheritance, decorator, direct call
- callback, thread, executor, async task 등록
- test에서 production symbol로 향하는 정적 참조
- DDS/RGB-D, lease/session, WebRTC, Genesis, hardware I/O 의미 표식
- 보수적인 orphan 후보(`orphan_candidate`); 자동 삭제 판정은 아님

노드 ID는 role, repository-relative path, qualified name을 결합한다. 파싱 실패는 이전 결과로 숨기지 않고 자홍색 `unparsed` 노드가 된다. HEAD에서 삭제된 파일은 빨간 tombstone으로 남는다. 추가는 녹색, 수정은 주황색이다.

초기 분석은 10초 이내, 동일 worktree 캐시 검사는 약 1초를 목표로 한다. 캐시는 `.elesim/analysis/code-map/snapshot.json`에 저장되며 Git 대상이 아니다. schema version과 전체 입력 digest가 일치할 때만 재사용한다.

## 화면 사용

기본 화면은 300개 노드로 제한된다. role, symbol/path 검색, HEAD 변경 상태, workflow, 정적/관측 edge와 표현 깊이를 조합해 범위를 좁힌다.

```text
role → package → module → class → function/method
```

노드를 선택하면 파일/행, caller/callee 수, decorator/base/문자열 근거, source 문맥과 HEAD diff를 표시한다. source API는 repository containment와 symlink를 검사하고 UTF-8, 128 KiB, 500줄로 제한한다.

## 기본 workflow

1. endpoint descriptor → heartbeat → bounded startup queue
2. UI operator intent → Pilot operator result
3. Pilot target selection → motion lease → command/ack
4. UI simulation command → Sim queue → Genesis apply
5. RGB-D capture/publish → DDS subscriber → perception
6. WebRTC session/signaling → answer/ICE → first frame
7. Robot validation → hardware write → feedback/deadman

workflow coverage는 코드가 해당 용어를 얼마나 발견했는지를 뜻할 뿐 runtime 성공률이나 wire-contract 검증 결과가 아니다.

## 정적 근거와 Jaeger

정적 edge는 점선이며 `exact`, `inferred`, `unresolved` confidence를 가진다. `Jaeger 불러오기`는 loopback Jaeger v3 query API에서 span을 읽어 현재 symbol에 대응되는 관측 edge를 실선으로 겹친다. trace가 없다고 dead code라는 뜻은 아니며 정적 분석 결과를 관측 결과로 덮어쓰지 않는다.

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

테스트는 AST 문법 요소, syntax error, 추가/수정/삭제 drift, cache roundtrip, semantic workflow, token/read-only HTTP, path traversal, symlink와 크기 제한을 검증한다.
