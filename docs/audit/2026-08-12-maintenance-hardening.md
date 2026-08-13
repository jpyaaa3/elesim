# 2026-08-12 maintenance hardening

이 문서는 예외 처리, orphan, 설치 산출물 경계와 중복 축약을 감사한 근거를
기록한다. 단순 문자열 검색만으로 dead code를 판정하지 않았고 Python/JS 참조,
console entry point, 생성 wrapper, package data, curl cache, release tree, wheel과
회귀 테스트를 함께 대조했다.

## 판정 기준

- **잘못된 예외 처리**: 서로 다른 계약의 실패를 같은 오류로 취급하거나,
  복구 불가능한 실패를 숨기거나, 정상 상태를 예외로 거부하는 경로
- **orphan**: 런타임·공개 entry point·생성물·테스트 seam 어디에서도 참조되지
  않고 호환 계약도 없는 코드
- **dummy/source-only**: 회귀·fixture·수동 재현에는 필요하지만 일반 curl 설치,
  생성 context, 역할 wheel 또는 release에는 필요하지 않은 코드와 데이터
- **축약 후보**: 분기 의미, 공개 import와 오류 경계를 유지하면서 한 domain
  helper/type으로 표현할 수 있는 완전 중복

## 수정한 예외 경계

### DDS와 관리 SSH

Docker Desktop sidecar DDS 경로에서 Tailscale peer route는 정상이어도 peer의 SSH
22번 포트는 닫혀 있을 수 있다. 기존 `--tcp-peer` 검사는 이 관리 정책을 DDS
route 실패로 오판해 SROS2 preflight를 중단했다. 이 옵션과 전용 helper를
제거했다. DDS preflight는 interface/address/route를, 관리 채널은 실제 SSH
연결을, application readiness는 endpoint descriptor와 live heartbeat를 각각
검증한다.

### 숨겨지던 런타임 실패

- 필수 real-RMW smoke에서 ROS 2 overlay 부재를 성공한 `SKIP`으로 처리하지 않고
  exit 2로 실패시킨다.
- RGB-D 입력의 shape/type/range 오류만 `dropped` frame으로 처리한다. 오래된
  ROSIDL setter, message 생성, byte serialization과 RMW publish 오류는 role
  lifecycle까지 전파한다. 마지막 입력 drop 사유는 512자로 제한해 기록한다.
- Dynamixel SDK의 정확한 top-level module 부재만 optional 상태로 처리한다. SDK
  내부 import 오류는 전파한다.
- Tailscale SSH의 `auth_none` fallback은 Paramiko authentication exception만
  처리한다. transport/programming 오류는 password compatibility 경로로 숨기지
  않는다.
- Compose `config --services`와 `ps` 명령 실패를 빈/stopped 상태로 바꾸지 않고
  remote command 오류로 보고한다.
- MPC QP solver 실패 때 이전 force를 재사용하지 않고 실패를 전파한다.
- host-native LJI adapter는 명시적 stop method가 없거나 stop이 실패하면
  fail-closed한다. 정상 DDS client에는 존재하지 않는 local loop stop을 호출하지
  않는다. stop 실패 뒤에도 log/perception/worker 정리는 `finally`로 끝낸다.
- runtime start/restart는 부분 기동된 현재 host까지 역순 보상한다. 보상 실패를
  모두 모아 원래 오류를 `.cause`/`__cause__`로 보존한다. session close도 모든
  host에 시도한 뒤 cleanup 오류를 같은 방식으로 보존한다.
- `전체 시작`은 기존 running role이 있으면 `재시작`을 요구한다. 재시작의 최초
  stop이 중간 실패하면 이미 건드린 host의 기존 running role을 복구한다.

## 제거한 orphan

repo 전체 정적 참조, export, 동적 package data와 테스트 seam을 대조한 뒤 다음
고신뢰 private orphan만 제거했다.

- Protocol display conversion helper 1개와 Sim locomotion limit helper 1개
- UI panel의 미사용 control/status/perception helper 6개
- Pilot의 제거된 remote-preview 시작 shim과 정규화 후 도달 불가능한 YOLO alias
- 설치기의 단수 Robot systemd compatibility helper
- connection-manager의 미사용 `addField`와 실제 DOM에서 참조하지 않는 CSS
- setup wizard의 미참조 danger/notice/grid/subsection/segmented/fingerprint CSS

공개 API였던 `resolve_initial_sag_model`은 삭제하지 않았다. optional 파일 부재만
빈 모델로 호환하고 malformed/unreadable 파일은 오류로 남겼다.

## 삭제하지 않은 고위험 후보

다음은 local call이 없거나 예외 탐색이 넓어 보여도 단순 삭제하지 않았다.

- Sim `RuntimePrep._apply_no_clip_pairs`: 구현 호출은 끊겼지만 model/config
  schema가 `no_clip_pairs`를 생성·보존한다. Genesis build 단계에 연결할지 schema
  전체를 폐기할지 실제 collision 검증과 함께 결정해야 한다.
- Pilot의 큰 IK/LJI/waypoint private branch: motion safety 및 외부 subclass
  호환 가능성이 있어 별도 API 결정과 hardware-focused gate가 필요하다.
- host-native LJI capability: production adapter 구현은 아직 없고 테스트 seam만
  있다. 이번 변경은 이를 normal DDS 경로와 격리하고 불완전 adapter를 명시적으로
  거부했으며, 기능 자체의 채택/폐기는 hardware boundary 작업으로 남긴다.
- legacy connection DOM normalization과 security `_switch_only`: 현재 호출은 없지만
  schema/browser migration 및 interrupted rollout 계약을 별도 검토해야 한다.

`except Exception`의 개수 자체를 품질 지표로 삼지 않았다. worker/process/API
경계에서 오류를 구조화하는 catch는 필요하다. 이번 감사에서는 실패의 종류를
잘못 바꾸거나 안전 동작을 생략하는 경로만 수정했다.

## dummy/source-only 격리

- curl bootstrap cache는 전체 Git archive를 보관하지 않는다. setup/protocol/
  ROSIDL/네 역할/config/model/container/developer 입력의 명시 allowlist만 private
  snapshot에 추출한다. 필수 exact file과 각 필수 tree sentinel이 빠지면 cache를
  publish하지 않는다.
- Setup과 protocol Python module, 역할 config는 exact manifest로 검증한다.
  역할 console entrypoint와 Pilot/Sim runtime config를 별도로 요구해 불완전한
  archive가 설치 후에야 깨지는 경로를 차단한다.
- `tests`, `fixtures`, cache/bytecode/egg-info, research 자료와 exact public config
  template 네 개는 curl cache에서 제외한다. Developer의 실제 source checkout은
  이후 별도 full clone이므로 개발 기능은 유지된다.
- generated role/tools context는 프로젝트 최상위 `tests`만 제외한다. Python
  package 내부 `config` 같은 runtime code를 basename으로 재귀 삭제하지 않는다.
- direct install과 release는 네 public example template만 exact 제외한다.
  runtime이 참조하는 `detector.yolo.example.json`은 보존한다.
- infra release의 setup package는 `pyproject.toml`, `requirements.lock`, `src`만
  복사한다. 미참조 `requirements-media.lock`은 source에서도 제거했다.
- wheel verifier는 foreign top-level package, `.data/purelib` 우회, traversal,
  비정규/중복 path, symlink/special file, tests/fixtures/cache/bytecode를 거부한다.
- release verifier는 role public template, exact infra manifest, path type/symlink,
  setup의 모든 console target과 양쪽 web bundle/icon/font의 필수 자산을
  독립적으로 검증한다. Setup production Python module manifest도 exact 비교하며,
  release tree의 모든 directory/file을 `lstat`해 임의 이름을 포함한 symlink와
  special file을 거부한다.
- curl, release build와 독립 release 검증은 `CMakeLists.txt`에 선언된 ROSIDL
  msg/srv/action 전체와 실제 source manifest가 정확히 일치하는지 확인한다.

## 의미를 유지한 축약

- General/Developer wrapper의 byte-identical Compose ownership guard를
  `manager_lifecycle.compose_owner_guard` 하나로 합쳤다.
- Pilot/Robot/Sim에 세 번 복제돼 있던 RGB-D application dataclass를 protocol
  domain type으로 합치고 기존 role import path는 compatibility re-export로
  유지했다. NumPy annotation import를 추가하지 않았으며 root protocol은 기존
  serde 경로에서 이미 NumPy를 사용하므로 새 startup 비용을 만들지 않는다.

## Tailscale uninstall ownership 복구

root-owned sidecar state 때문에 manifest 기반 uninstall이 막히던 경로를 bounded
transaction으로 처리한다. exact install UUID, Compose service/config, immutable
image digest와 sole RW bind를 검증하고, running exact sidecar에서만 fixed script를
실행한다. Host `O_NOFOLLOW` dirfd에 만든 random token을 container mount에서
확인해 현재 host inode와 기존 bind inode가 동일함을 증명한 뒤 PID 1을 멈추고
symlink/special/hardlink를 거부한다. exact tree만 호출 UID/GID와 owner mode로
복구한다. sidecar `rm --force` 전 실패는 `CONT` 또는 exact container start로
복구하며, 성공한 sidecar 제거가 명시적 commit point다. stopped/absent sidecar는
host가 이미 exact tree를 안전하게 순회할 수 있을 때만 진행한다.

## 검증 기록

Docker service를 새로 띄우지 않고 다음을 확인했다.

- installer connection/uninstall/bootstrap 집중: 129 passed
- secure deployment/CLI/configuration/topology/context/installer: 211 passed
- connections 추가 rollback/state 회귀: 29 passed
- setup 전체: 512 passed, 8 host-environment blocked
- protocol 전체: 95 passed + 5 subtests
- release verifier/build helper: 58 passed
- RGB-D compatibility/error boundary: 7 passed
- Dynamixel optional import: 2 passed + 2 subtests
- required smoke wrapper: 1 passed
- 모든 변경 Python `py_compile`, connection JS `node --check`,
  `git diff --check`: passed

임시 pytest 경로를 주입한 host required driver에서는 Protocol 95개와 DDS RGB-D
2개가 통과했다. 호스트에는 SciPy/OpenCV, UI/WebRTC native dependencies,
Dulwich와 요구 버전의 setuptools가
없다. 따라서 Sim/Pilot/Robot/model/UI 일부 테스트는 import 단계에서, setup wheel
실물 build 1건은 host setuptools가 PEP 621 metadata를 읽지 못해
`UNKNOWN-0.0.0`으로 실패했다. Setup 전체의 8건 중 7건은 loopback/Unix socket을
금지한 sandbox, 1건은 Dulwich 부재로 막혔다. 이는 변경된 assertion의 실패가
아니다. Host에서 `quality/check.py --group required`도 실행했지만 pytest와 ROS 2
overlay 부재를 정확히 실패로 보고했다. canonical `elesim-dev` required/extended
gate와 실제 release 재생성은 별도 최종 gate로 남는다.

## 2026-08-14 반복 점검: readiness 원인과 설치 source 고정

최근 Docker Desktop/WSL 로그의 `DDS readiness: ... 실패: __enter__`는 DDS
패킷 자체가 처음부터 사라진 증거가 아니었다. 같은 실행에서 Pilot/Sim의
discovery, target selection, lease grant까지 관찰된 뒤 readiness가 실패했고,
manager가 해당 작업을 rollback하면서 역할 컨테이너가 `137`로 종료됐다. 즉
오류 문자열이 실제 원인(`AttributeError` 계열의 context-manager 호출 실패)을
가리고 있었고, rollback의 종료 코드를 OOM으로 오인하기 쉬운 상태였다.

이번 반복에서 다음 경계를 고정했다.

- `SshHostOperations`는 connector가 반환한 세션을 직접 `__enter__`하지 않고
  `_BorrowedSession`으로 감싼다. connector가 bare session을 반환해도 job의
  `with` 경계를 유지하며, 종료 시에는 `__exit__` 또는 `close()`를 명시적으로
  선택한다. bare-session 회귀 테스트를 추가했다.
- DDS readiness worker는 `Exception`만 job 실패 보고로 수집한다. 취소/프로세스
  인터럽트를 삼키지 않으며, 빈 메시지나 `__enter__`/`__exit__`처럼 모호한
  예외는 구체적인 예외 class를 함께 출력한다.
- InstallState v9에 `source_repository`와 `source_ref`를 기록하고, 설치·GUI·CLI
  모두 같은 값을 update wrapper에 전달한다. 기존 state는 `main`으로 명시적
  migration하며, wrapper는 실행 때 `repository@ref`를 출력한다. 따라서
  `refactoring`에서 고친 manager가 다음 `elesim-update`에서 조용히 GitHub
  `main`을 다시 받아오는 경로를 제거했다. 의도적인 일회성 override는
  `ELESIM_REPOSITORY`/`ELESIM_REF`로만 가능하다.
- 사용하지 않는 `docs/design/installer_network_security_placeholder.md`는
  삭제하고 필요한 운영 경계는 `docs/setup.md`/`docs/deployment.md`로
  통합했다. bootstrap/release의 exact allowlist와 연구 자료 제외 검사는
  유지하므로 source-only 자료가 curl 설치물에 섞이지 않는다.

이 반복 점검은 Docker를 새로 올리지 않고 Python compile, generated update
wrapper `bash -n`, diff whitespace 검사로 확인했다. host에는 pytest와 Docker
daemon 접근 권한이 없어 canonical container gate는 별도 실행이 필요하다.

3차 정합성 점검에서도 setup 모듈 AST와 문서 상대 링크를 다시 순회했다. import
graph 밖에 남은 모듈은
`pyproject.toml` console script가 직접 소유하는 `cli`/`connections`/
`host_proxy`와 manager wrapper가 `python -m`으로 실행하는 bounded
`host_helper`뿐이다. `network`는 `elesim-net` console script가 직접 소유한다.
bootstrap shell syntax, 전체
runtime Python AST/compile, 기록 source를 포함한 generated wrapper syntax도
재검증했다.

## 2026-08-14 반복 점검 2: 예외 경계와 중복 진단 제거

트랜잭션 롤백·보안 generation 복구처럼 `KeyboardInterrupt`까지 cleanup해야 하는
경계의 `BaseException`은 유지하고, 그 외의 무차별 수집은 줄였다.

- SSH fingerprint probe와 host-helper/SSH local-stream drain은 일반 `Exception`만
  수집한다. worker에서 `SystemExit`나 사용자 인터럽트를 일반 명령 실패로
  바꾸지 않는다.
- 복구 루틴의 동작이 전혀 없는 `except BaseException: raise`를 제거했다.
- readiness worker가 SSH/session 호출 자체에서 실패한 경우를 peer heartbeat
  부재와 구분해 `readiness probe error` 및 예외 class를 출력한다. 따라서
  `AttributeError: __enter__`가 DDS timeout으로 위장하지 않는다.
- Sim은 이미 필수 bundle을 검증하고 바로 `_load_joint_layout()`으로 읽는데,
  같은 manifest를 다시 읽어 실패를 `manifest inspect skipped`로 출력하던
  중복 진단 블록을 삭제했다. 필수 manifest 오류는 원래의 검증 예외로 남는다.
- pre-v9 update wrapper는 source ref가 shell text에 고정되어 있으므로 환경변수만
  바꿔서는 복구되지 않는다는 사실을 setup/deployment 문서에 명시하고, 의도한
  bootstrap을 한 번 직접 실행하는 복구 절차를 추가했다. `--purge`가 source
  선택을 바꾸지 않는다는 점도 명시했다.

정적 확인은 변경 모듈 compile, direct context-manager 호출 검색, source-only
allowlist와 generated-wrapper syntax 검사로 수행했다. Docker/pytest가 없는
호스트 제약은 변하지 않아 실제 2-host DDS gate의 증거로 사용하지 않는다.

추가로 Sim 기본 장면의 `sim_target_enable=true` 경로를 확인했다. Genesis의
타깃 구체 생성이 실패해도 `sim target spawn failed`만 출력하고 계속하던
예외는 필수 장면 계약을 깨뜨린 채 Pilot을 기동시키는 숨은 실패였다. 이제
해당 경계에서 `RuntimeError`로 원인을 보존하며, 회귀 테스트가 실패를
삼키지 않는지 확인한다.

bootstrap snapshot 검사는 curl 추출물에서 제외할 public-example 설정을
소스 checkout 검증 시에도 허용하되, 추출 결과에는 계속 복사하지 않는다.
따라서 비-gitignore 예제 파일과 실제 설치 payload의 경계가 서로 다르게
판정되지 않는다.
