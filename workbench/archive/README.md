# 보관한 퇴역 소스

이 디렉터리는 중복 실행기, 옛 결과 경로/샘플 ID 전용 그림 생성기, 중단한
height/mount 보정 실험을 로컬 ZIP으로 보관한다. 현재 쓰는 도구와 명령은
[`../README.md`](../README.md)에 있다.

`retired-tools-20261027.zip`은 **24개 원본**을 담는다. 원본 경로·보관 이유·각
파일 SHA256·archive SHA256은 [`index.json`](index.json)에 있다. 계산 함수가
회귀검사에 필요한 여섯 파일은 별도의 test helper로 유지하고, 코드 정의가
같음을 확인했다. 테스트 파일과 assertion은 제거하지 않았다.

- ZIP과 여기서 풀어 둔 파일은 `.gitignore` 대상이다. 다른 checkout에는 ZIP이
  자동 전송되지 않는다. README와 index만 Git에서 관리한다.
- `worktree/`에는 이번 작업 직전 내용을, `head-0dbe474/`에는 앞선 통합에서
  제거된 다섯 entrypoint의 기록된 Git HEAD 원본을 넣었다.
- 현재 source가 이 디렉터리를 import하거나 실행 경로/PYTHONPATH에 추가하면 안 된다.
- 복구할 때는 index의 hash를 먼저 확인하고 별도 디렉터리에서 원본을 읽는다.
  archive의 옛 명령을 현재 runtime에 바로 적용하지 않는다.
- 정상 source로 다시 사용할 필요가 생기면 현재 catalog/환경/판정 기준에
  맞춰 이관한 뒤 회귀검사를 실행한다. 조용히 원래 경로에 되돌리지 않는다.
- raw 측정 자료·사진·동영상은 옮기지 않았다. 기존 evidence 링크는 유지된다.

이 ZIP은 손대지 않은 원문 보관본이다. 실행 환경과 오래된 외부 의존성까지
묶은 재현 환경은 아니며, 압축을 푸는 것만으로 당시 실험이 실행되지는 않는다.

`retired-visual-servoing-ui-20261101.zip`은 새 Visual Servoing 작업 화면으로
대체한 UI panel 4개와 교체 전 UI regression 파일 3개의 정확한 원문을 보관한다.
역할별 source와 archive member, SHA256은 같은 `index.json`에 기록했다.
Start/cancel·coverage·mock scene/motion authority·파일 선택 회귀는 현재 UI
suite로 이관했다. 이 ZIP은 현재 runtime이나 테스트가 읽지 않는다.

`retired-hug-pipelines-20261110.zip` preserves the exact pre-cleanup Hug workflow sources and instructions. Five obsolete standalone files were removed; shared code and all regression assertions remain. See `../research/experiments/HUG_WORKFLOWS.md` for the dependency boundary. ZIP is local-only and is not pushed by Git.
