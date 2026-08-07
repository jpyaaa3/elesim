# IMU Body Pitch 기반 Camera Stabilization 개념 보고서

작성일: 2026-07-09

## 1. 한 줄 요약

IMU Body Pitch 기반 Camera Stabilization은 GO2가 걸을 때 몸체가 앞뒤로 기울어지면서 손목 카메라 시야가 흔들리는 문제를, body pitch 정보를 이용해 미리 보정하는 gaze 안정화 방법이다.

## 2. 왜 필요한가?

GO2가 걷는 동안 몸체는 완전히 수평을 유지하지 않는다. 발이 지면에 닿고 떨어지는 과정에서 body pitch가 계속 변한다.

팔과 카메라는 GO2 몸체 위에 장착되어 있기 때문에, 몸체가 pitch 방향으로 흔들리면 카메라 시야도 같이 흔들린다.

그 결과 카메라 화면에서는 target이 위아래로 움직이는 것처럼 보인다.

```text
GO2 body pitch 변화
  → arm base 자세 변화
  → wrist camera 방향 변화
  → image vertical error 증가
```

특히 pick & place에서는 target을 계속 화면 중앙에 유지해야 하므로, walking 중 pitch 흔들림은 visual servoing과 grasp 안정성에 직접적인 영향을 준다.

## 3. 일반 UV feedback의 한계

가장 단순한 방법은 카메라 화면에서 target이 벗어난 뒤에 다시 중앙으로 맞추는 것이다.

```text
target이 위로 움직임
  → v error 발생
  → 팔을 움직여 보정
```

이 방식은 reactive control이다. 이미 화면이 흔들린 뒤에 따라가는 방식이기 때문에, GO2의 pitch 진동이 빠르거나 반복적이면 target이 계속 위아래로 출렁일 수 있다.

즉, 단순 UV feedback은 “늦게 반응하는” 문제가 있다.

## 4. 핵심 아이디어: Pitch를 미리 본다

IMU 기반 stabilization의 핵심은 카메라 이미지가 흔들리기 전에 몸체 pitch 변화를 먼저 감지하는 것이다.

GO2의 IMU 또는 base state에서 다음 정보를 얻을 수 있다.

- 현재 body pitch
- body pitch rate

pitch rate는 “몸체가 지금 어느 방향으로 얼마나 빠르게 기울고 있는지”를 알려준다.

이 값을 사용하면 곧 target이 이미지에서 위로 움직일지, 아래로 움직일지를 미리 예상할 수 있다.

```text
body pitch rate 관측
  → 곧 생길 vertical image error 예측
  → arm gaze를 미리 보정
```

이것이 pitch preview의 핵심이다.

## 5. Pitch Preview 방식

Pitch Preview는 현재 이미지 오차만 보는 것이 아니라, 가까운 미래에 pitch로 인해 생길 image vertical error를 함께 고려한다.

개념적으로는 다음과 같다.

```text
현재 오차 = camera가 실제로 본 u/v error
예상 오차 = body pitch rate로부터 예측한 v 방향 흔들림

보정해야 할 오차 = 현재 오차 + 예상 오차
```

이렇게 하면 target이 이미 크게 흔들린 뒤에 따라가는 것이 아니라, 흔들림이 생기기 전에 arm을 조금 먼저 움직일 수 있다.

## 6. Arm은 무엇을 움직이나?

카메라 안정화를 위해 arm의 일부 자유도를 사용한다.

현재 시스템에서는 주로 arm segment를 움직여 카메라 방향을 보정한다.

```text
seg1 / seg2 조정
  → wrist camera 방향 변화
  → target이 image center 근처에 유지됨
```

roll도 사용할 수 있지만, GO2 위에 arm이 장착된 구조에서는 roll이 payload 자세와 안정성에 영향을 줄 수 있다. 그래서 실제 운용에서는 roll보다 segment 기반 보정을 우선한다.

## 7. 왜 Pitch가 특히 중요한가?

walking 중 카메라 시야에서 가장 문제가 되는 흔들림은 vertical 방향인 경우가 많다.

GO2 body가 pitch 방향으로 기울면 카메라가 위아래로 흔들리고, 이는 이미지의 `v` 방향 error로 나타난다.

따라서 body pitch를 잘 보정하면 walking 중 target의 상하 흔들림을 크게 줄일 수 있다.

정리하면 다음과 같다.

```text
body pitch disturbance
  → image v disturbance
  → pitch-based preview correction
  → lower vertical tracking error
```

## 8. 전체 동작 흐름

IMU Body Pitch 기반 Camera Stabilization은 다음 순서로 동작한다.

1. 카메라가 target 위치를 관측한다.
2. 현재 `u/v` image error를 계산한다.
3. GO2 body pitch 또는 pitch rate를 읽는다.
4. pitch 변화로 인해 곧 생길 `v` 방향 흔들림을 예측한다.
5. 현재 image error와 preview error를 함께 고려한다.
6. arm segment를 조금 움직여 카메라 방향을 보정한다.
7. 다음 frame에서 다시 관측하고 반복한다.

핵심은 카메라 feedback과 IMU pitch 정보를 함께 쓴다는 점이다.

## 9. UV Feedback과 Pitch Preview의 차이

두 방식의 차이는 다음과 같다.

```text
UV feedback:
  화면이 흔들린 뒤에 보정

Pitch preview:
  몸체 pitch를 보고 화면이 흔들리기 전에 미리 보정
```

UV feedback은 target 위치만 본다. 반면 pitch preview는 target 위치뿐 아니라 로봇 몸체의 움직임 원인도 함께 본다.

그래서 walking처럼 주기적인 base motion이 있는 상황에서는 pitch preview가 더 안정적일 수 있다.

## 10. 이 시스템에서의 역할

이 안정화기는 mobile pick 과정에서 특히 중요하다.

GO2가 target을 향해 걸어가는 동안 카메라는 target을 계속 보고 있어야 한다. 이때 pitch preview는 body motion 때문에 target이 화면 밖으로 밀리거나 vertical error가 커지는 것을 줄여준다.

전체 흐름에서 위치는 다음과 같다.

```text
Perception
  → target UV error 계산
  → GO2 body pitch 관측
  → pitch preview gaze correction
  → walking 중 target 시야 유지
  → handoff 이후 LJI grasp
```

즉, pitch preview는 grasp 자체를 수행하는 알고리즘이라기보다, grasp 단계에 들어가기 전까지 target을 안정적으로 바라보게 해주는 walking gaze 안정화 알고리즘이다.

## 11. 한계

Pitch 기반 stabilization은 body pitch가 camera vertical 흔들림의 주요 원인이라는 가정에 기반한다.

하지만 실제 환경에서는 다른 요인도 존재한다.

- yaw/roll motion
- arm 자체의 지연
- perception delay
- target detection noise
- 모터 응답 지연
- 보행 중 충격과 진동

따라서 pitch preview만으로 모든 흔들림을 제거할 수는 없다. 다만 walking 중 반복적으로 나타나는 vertical 흔들림을 줄이는 데 효과적이다.

## 12. 결론

IMU Body Pitch 기반 Camera Stabilization은 GO2의 몸체 pitch 변화를 이용해 손목 카메라의 vertical 흔들림을 미리 보정하는 방법이다.

핵심은 단순히 카메라 화면을 보고 뒤늦게 따라가는 것이 아니라, IMU/body state를 이용해 곧 생길 image error를 예측하고 선제적으로 arm gaze를 조정하는 것이다.

이 방식은 walking 중 target을 더 안정적으로 시야에 유지하게 해주며, 이후 LJI grasp나 pick 동작이 더 좋은 초기 조건에서 시작되도록 돕는다.
