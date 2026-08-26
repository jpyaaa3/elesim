# RGB-D Edge Broker

## 목적

RGB-D의 raw frame은 카메라가 붙은 source 내부에서만 만들어진다. source가
encoded edge publisher를 사용할 수 있으면 DDS에 올리기 전에 한 번만
압축하고, legacy source만 Pilot relay가 raw를 압축한다. inter-host DDS에는
Pilot이 소유하는 bounded, latest-only encoded stream만 내보낸다. Pilot은 새
runtime role이 아니다.

```text
physical mode
  Robot camera ── local/encoded source DDS ───► Pilot
                                                ├─ decode + perception
                                                └─ relay (legacy raw만 encode) + publish
                                                      ├─► UI
                                                      └─► optional Sim

simulation-only
  Sim/GPU camera ── encoded source DDS ──────────► Pilot
                                                    ├─ decode + perception
                                                    └─ relay/pass-through + publish ──► UI
```

`simulation-only`의 권장 배치는 `Pilot+Sim`을 하나의 Compose unit에 두고,
`UI`를 별도 host/unit에 두는 2-host topology다. 한 PC에서 세 역할을 모두
실행하는 topology도 유효하며, 이 설계는 role을 추가하지 않는다.

## 소유권

| 단계 | 소유자 | 경계 | 데이터 형태 |
| --- | --- | --- | --- |
| camera acquisition | Robot 또는 Sim | source 내부 | raw RGB-D |
| source edge | Robot 또는 Sim | camera process | raw → encoded 1회 |
| local/source handoff | source + Pilot | bounded DDS source topic | bounded latest-only encoded (legacy raw 허용) |
| perception/relay | Pilot | Pilot process | decode + perception; legacy raw만 encode |
| inter-host stream | Pilot | DDS/SROS2 | encoded RGB-D v1 |
| display/consumer | UI, 필요 시 Sim | consumer process | decoded frame |

Pilot은 source가 바뀌어도 broker endpoint와 stream policy의 단일 owner다.
정상 경로에서는 source가 만든 encoded sample을 Pilot이 재압축하지 않고
검증·relay한다. UI와 Sim은 encoder를 갖지 않으며, 각자 필요한 시점에
decode한다. 이 decode 중복은 데이터 경계를 지키기 위한 consumer 책임이며,
source-side encoder를 세 번 복제하는 것과 다르다.

## Wire descriptor

기존 `StreamDescriptor` 필드를 유지한다. encoded RGB-D는 다음 조합으로
식별한다.

```yaml
name: rgbd
transport: dds
media_kind: rgbd
endpoint: /elesim/pilot_main/rgbd/frame
message_type: elesim_interfaces/msg/EncodedRgbdFrame
qos_profile: sensor_data_latest_only
```

Pilot descriptor는 `stream.rgbd`와 `stream.rgbd.broker.v1` capability를
광고한다. `stream.rgbd.broker.v1`가 없는 v6 peer에는 기존 descriptor를
강제로 encoded로 해석시키지 않는다. codec, dimensions, calibration ID,
depth scale, sequence와 payload bound는 frame metadata가 소유한다. 이를
위해 protocol major를 올리거나 ROS service/action을 추가하지 않는다.

현재 raw `RgbdFrame` 구현과의 호환 기간에는 source topic과 broker topic을
설정에 함께 기록할 수 있다. 그러나 inter-host consumer가 raw source topic을
직접 구독하는 새 배포는 금지한다. source raw path는 migration/diagnostic
범위이고, 최종 acceptance 대상은 Pilot broker stream이다.

## Configuration contract

설치 생성 config에는 다음 metadata가 들어간다.

```yaml
rgbd:
  schema_version: 1
  broker_role: pilot
  source_role: auto
  local_handoff: source-dds-to-pilot
  wire:
    format: encoded-rgbd-v1
    capability: stream.rgbd.broker.v1
    topic: /elesim/pilot_main/rgbd/frame
    latest_only: true
```

Sim과 Robot source는 `source-dds-to-pilot` handoff를 사용한다. UI의
`source_role`은 `pilot`이며, UI 설정은 broker topic을 알아야 하지만 source
camera topic을 publish하지 않는다.
설치기의 endpoint derivation이 custom Pilot ID를 반영하므로 YAML에
`pilot-main`을 영구적으로 하드코딩하지 않는다.

## 실패 경계

- local handoff가 늦어지면 Pilot은 bounded latest-only 정책으로 오래된
  frame을 버리고 perception age를 보고한다.
- encode가 실패하면 broker frame을 발행하지 않고 source/consumer가 stale
  frame을 새 frame으로 오인하지 않도록 sequence/boot identity를 유지한다.
- consumer decode 실패는 UI/Sim stream만 끊으며 Pilot perception이나 Robot
  safety를 중지시키지 않는다.
- broker가 재시작하면 새 boot ID와 sequence를 사용한다. consumer는 이전
  broker frame을 재사용하지 않는다.

## 성능 수용 기준

각 frame에 대해 source capture, local handoff, encode, DDS publish, DDS receive,
decode의 timestamp와 sequence gap을 측정한다. 다음을 별도로 기록해야 한다.

- encoded bytes/frame 및 예상 bitrate
- source-to-Pilot frame age
- Pilot encode p50/p95
- DDS publish-to-receive p50/p95
- consumer decode p50/p95
- overwrite/drop/gap count

이 수치가 없으면 Genesis render, GPU→CPU transfer, encoder, DDS transport를
서로 탓하는 진단을 재현할 수 없다.

## 단계적 도입

1. 설치 config와 topology가 `Pilot+Sim` co-location 및 broker ownership을
   표현한다.
2. Robot/Sim source가 raw를 source edge에서 encoded로 바꾸고 bounded latest-only
   topic으로 Pilot에 전달한다. legacy raw는 Pilot relay가 한 번만 encode한다.
3. Pilot이 encoded broker stream을 pass-through/relay publish하고 descriptor
   capability를 광고한다.
4. UI/Sim consumer가 capability를 확인한 뒤 decode한다.
5. raw inter-host RGB-D 구독을 제거하고 frame-age/bandwidth gate를 통과시킨다.

1단계만으로 runtime transport가 바뀌었다고 주장하지 않는다. 각 단계는
해당 role의 smoke와 실제 multi-host SROS2 acceptance를 별도로 통과해야 한다.
