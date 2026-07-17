# Configuration

Elesim의 정식 설정 형식은 YAML이며 진입점은 `engine.config.load_app_config()`이다.

## Files

- `configs/config.yaml`: 공통 기본값을 명시한 기준 설정
- `configs/config.pc.yaml`: PC에서 달라지는 값만 가진 프로필
- `configs/config.jetson.yaml`: Jetson에서 달라지는 값만 가진 프로필
- `configs/calibration/`: hand-eye 등 장치 보정값
- `configs/perception/`: detector preset
- `configs/sag/`: arm 처짐 보정 모델

프로필은 다음처럼 한 개의 YAML 부모를 상속한다.

```yaml
schema_version: 1
extends: config.yaml  # configs/ 안에서 같은 폴더의 기준 설정을 상속
simulation:
  runtime:
    use_hardware: true
```

맵은 재귀적으로 병합되고 scalar와 list는 자식 값으로 완전히 교체된다. 상대경로는
그 값을 선언한 설정 파일 위치를 기준으로 해석하며, 순환 상속은 오류로 처리한다.

## Hierarchy

최상위 키는 설정 소유권을 나타낸다.

- `simulation`: 물리, 런타임, 조립 결과, trajectory, simulation camera
- `transport`: 프로세스 간 endpoint
- `robot.arm`: arm hardware, model limit, spawn, URDF, IK
- `robot.go2`: spawn, teleop, hardware bridge, locomotion
- `world`: simulation target과 debug marker
- `vision`: perception provider, detector, publisher, tracker
- `behaviors`: pick과 gaze 동작
- `experiment`: 실험 기록 설정

leaf 이름은 부모 문맥을 전제로 짧게 유지한다. 예를 들어 과거의
`gaze_preview_b_pitch`는 `behaviors.gaze.preview.b_pitch`이다.

## Validation

YAML 로더는 알 수 없는 키, 중복 키, 잘못된 타입, 지원하지 않는
`schema_version`을 즉시 거부한다. Boolean은 YAML 1.2 방식의 `true`와 `false`만
Boolean으로 읽으며 `on`, `off`, `yes`, `no`는 문자열이다.

새 코드에서 구체 포맷 로더를 직접 import하지 말고 다음 진입점만 사용한다.

```python
from engine.config import load_app_config

bundle = load_app_config("configs/config.yaml")
```

## Legacy INI

`results/` 아래 과거 실험 설정을 재현할 수 있도록 `.ini` dispatch는 당분간
남아 있지만 `DeprecationWarning`을 발생시킨다. 새 INI 설정은 만들지 않는다.
명시적인 일회성 변환 도구는 다음과 같다.

```bash
python3 tools/quality/convert_config_ini.py old.ini converted.yaml
```

실험 중 일부 값만 바꾸려면 원본을 문자열 치환하지 말고 YAML overlay를 만든다.

```bash
python3 tools/experiments/config_overlay.py \
  --base configs/config.yaml \
  --output /tmp/elesim-experiment.yaml \
  --set behaviors.gaze.preview.b_pitch=-0.05
```
