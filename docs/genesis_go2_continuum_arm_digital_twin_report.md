# Genesis 기반 GO2 + Continuum Arm Digital Twin 개념 보고서

작성일: 2026-07-09

## 1. 한 줄 요약

이 프로젝트의 digital twin은 실제 GO2 이동 로봇 위에 continuum arm과 손목 카메라가 장착된 시스템을 Genesis 시뮬레이터 안에 재현한 것이다.

목적은 실제 로봇에 바로 올리기 전에 locomotion, gaze, perception, visual servoing, grasp 동작을 하나의 폐루프 환경에서 검증하는 것이다.

## 2. Digital Twin의 핵심 아이디어

여기서 digital twin은 단순한 3D 모델이 아니다.

중요한 점은 실제 로봇과 시뮬레이터가 같은 제어 개념을 공유한다는 것이다.

실제 로봇에서 사용하는 명령은 대략 다음과 같다.

- GO2는 앞으로/옆으로/회전 속도 명령을 받는다.
- arm은 `linear`, `roll`, `seg1`, `seg2` 같은 축약된 제어 입력을 받는다.
- 카메라는 손목에 붙어 물체를 관측한다.

Genesis 안의 twin도 같은 방식으로 움직인다. 그래서 제어 알고리즘 입장에서는 “실제 로봇인지 시뮬레이터인지”가 크게 달라지지 않는다.

## 3. GO2는 어떻게 표현되는가?

GO2는 URDF 기반의 quadruped 모델로 Genesis에 올라간다.

시뮬레이터 안의 GO2는 다음 역할을 한다.

- 이동 베이스 역할
- arm을 싣고 움직이는 플랫폼 역할
- walking/gaze 실험에서 base motion을 만들어내는 역할

GO2는 시뮬레이터가 직접 걷게 할 수도 있고, 실제 로봇에서 들어오는 base pose를 따라가게 할 수도 있다. 이 덕분에 같은 환경에서 “순수 시뮬레이션”과 “실제 로봇 상태 미러링”을 모두 다룰 수 있다.

## 4. Continuum Arm은 어떻게 표현되는가?

continuum arm은 실제로는 연속적으로 휘어지는 구조지만, Genesis 안에서는 여러 개의 짧은 rigid segment를 이어 붙인 관절 체인으로 근사한다.

즉, 실제 arm의 부드러운 곡선을 다음처럼 표현한다.

```text
연속적으로 휘는 arm
  → 여러 개의 짧은 node
  → node 사이의 작은 회전 관절
  → 전체적으로 휘어진 arm 형태
```

하지만 제어기는 모든 작은 관절을 직접 다루지 않는다. 대신 arm 전체를 더 단순한 몇 개의 제어 변수로 본다.

```text
linear, roll, seg1, seg2
```

이 네 값이 들어오면 시뮬레이터는 이를 여러 node joint로 펼쳐서 arm 모양을 만든다.

핵심 아이디어는 이것이다.

> 제어기는 단순한 4축 arm처럼 명령하고, 시뮬레이터는 그 명령을 여러 segment로 나누어 continuum-like shape을 만든다.

## 5. GO2와 Arm은 어떻게 합쳐지는가?

GO2와 arm은 각각 따로 정의된 모델이다. 실행 단계에서 arm base를 GO2 body 위 특정 위치에 고정하여 하나의 로봇처럼 만든다.

개념적으로는 다음과 같다.

```text
GO2 base
  └── fixed mount
        └── continuum arm base
              └── arm body
                    └── gripper + camera
```

이 구조 때문에 GO2가 움직이면 arm과 카메라도 함께 움직인다. 따라서 walking 중 gaze stabilizer나 mobile pick 실험에서 base motion과 arm/camera motion이 함께 반영된다.

## 6. 카메라는 왜 중요한가?

이 시스템은 pick & place를 목표로 하기 때문에 카메라가 매우 중요하다.

Genesis 안에는 손목에 붙은 eye-in-hand 카메라가 있다. 이 카메라는 실제 RealSense가 달린 위치와 방향을 흉내 내도록 arm 끝단에 부착된다.

이 카메라가 렌더링한 RGB-D frame은 perception pipeline으로 들어간다. 따라서 시뮬레이션에서도 실제 로봇처럼 다음 흐름을 테스트할 수 있다.

```text
카메라 영상
  → 물체 인식
  → gaze 제어
  → visual servoing
  → grasp 접근
```

즉, 카메라는 단순 시각화용이 아니라 제어 루프 안에 들어가는 센서다.

## 7. Target은 어떻게 표현되는가?

시뮬레이션에서는 물체를 Genesis 안의 간단한 target object로 둔다.

이 target은 perception이 찾아야 하는 물체 역할을 한다. 로봇은 이 target을 카메라로 보고, gaze를 유지하고, arm을 움직여 가까이 접근한다.

실제 환경의 물체를 완전히 복제하려는 목적보다는, 인식-제어-접근 파이프라인을 검증하기 위한 기준 물체라고 볼 수 있다.

## 8. Host와 Sim의 관계

digital twin은 host와 계속 통신한다.

host는 시뮬레이터에 명령을 보낸다.

- GO2 속도
- arm 목표 자세
- gripper 상태
- target 위치

시뮬레이터는 그 명령을 Genesis 안에서 실행하고 결과를 다시 보낸다.

- 실제 시뮬레이션상의 tip 위치
- 카메라 위치와 방향
- GO2 base 상태
- 카메라 frame

이 구조 덕분에 host는 “명령을 보냈다”에서 끝나지 않고, 시뮬레이터 안에서 실제로 어떤 일이 일어났는지 feedback을 받을 수 있다.

## 9. 왜 이런 구조가 유용한가?

GO2 + arm + camera + object가 모두 연결되어 있기 때문에, 단일 알고리즘만 따로 보는 것이 아니라 전체 pick pipeline을 검증할 수 있다.

예를 들어 다음을 실제 로봇 없이 먼저 확인할 수 있다.

- GO2가 움직일 때 카메라 시야가 어떻게 흔들리는지
- gaze stabilizer가 target을 계속 볼 수 있는지
- target이 화면 안에 들어왔을 때 arm이 visual servoing을 할 수 있는지
- handoff 이후 LJI grasp가 자연스럽게 이어지는지
- 카메라 영상과 실제 arm tip 위치가 서로 일관적인지

이것이 digital twin의 가장 큰 역할이다.

## 10. 한계

이 twin은 실제 로봇을 완벽히 복제한 물리 모델은 아니다.

특히 continuum arm은 실제 연속체가 아니라 여러 rigid segment로 근사되어 있다. 실제 모터의 마찰, backlash, cable/tendon 효과, 카메라 노이즈도 완전히 같지는 않다.

따라서 이 모델의 목적은 “현실과 100% 같은 물리 재현”이 아니라, 실제 로봇에 올리기 전에 제어 구조와 실험 흐름을 검증하는 것이다.

## 11. 결론

Genesis 기반 GO2 + Continuum Arm digital twin은 실제 시스템의 핵심 구조를 시뮬레이션 안에 옮긴 것이다.

핵심은 세 가지다.

1. GO2 위에 arm과 camera가 함께 움직이도록 만든다.
2. 실제 로봇과 같은 형태의 제어 명령을 사용한다.
3. perception부터 grasp까지 전체 폐루프를 시뮬레이션에서 돌릴 수 있게 한다.

이 구조 덕분에 실제 로봇을 움직이기 전에 locomotion, gaze, visual servoing, pick 동작을 더 안전하고 빠르게 검증할 수 있다.
