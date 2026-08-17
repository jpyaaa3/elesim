# Distributed Runtime Reference

이 문서는 네트워크 경계만 빠르게 찾기 위한 참조표다. 시스템 전체의 정본은
[`architecture.md`](architecture.md), wire registry는 [`dds_contracts.md`](dds_contracts.md),
실제 배포는 [`deployment.md`](deployment.md)다.

## DDS graph

모든 role은 ROS 2 participant이며 직접 UDP로 통신한다. 중앙 Router, ZMQ,
CURVE, registry, DDS relay는 없다.

```text
/<system_id>/v6/discovery/endpoints
/<system_id>/v6/discovery/heartbeats
/<system_id>/v6/control
/<system_id>/v6/<endpoint-resource-prefix>/rgbd/frame
```

Descriptor는 `PeerRef`, role, capability, stream descriptor와 boot-specific
resource prefix를 광고한다. Heartbeat는 같은 endpoint/boot와 revision을
증명한다. duplicate live boot은 fail closed이며, descriptor+heartbeat gate
전에는 authority를 획득하지 않는다. startup envelope queue는 heartbeat
timeout 동안 최대 512개다.

## Authority

Robot/Sim이 자기 motion lease를 serialize하고, Sim이 UI simulation session을
serialize한다. `select_target`, `renew_target`, `release_target`와
`open/renew/close_simulation_session`은 reliable control carrier다.
`motion_command`는 lease-bound best-effort depth-1이고 local deadman이
독립적으로 만료시킨다. stale boot, sequence, token, epoch은 모두 거부한다.

UI session은 motion lease가 아니며, DDS discovery/SSH/TURN도 authority를 주지
않는다.

## Media surfaces

| 데이터 | 경로 | 규칙 |
| --- | --- | --- |
| physical/sim RGB-D | typed `RgbdFrame` DDS | coherent latest-only, depth 1, 오래된 sample drop |
| observer scene | Sim → UI WebRTC | 별도 track, DTLS/SRTP |
| hand-eye preview | Sim → UI WebRTC | observer와 별도 track |
| offer/answer/ICE signal | `webrtc_signal` DDS | Sim-owned reliable request/reply |

Coturn은 WebRTC ICE media candidate만 relay한다. UI와 Sim이 DDS로 SDP를
주고받지 못하면 TURN이 살아 있어도 video session은 만들 수 없다.

Sim native Genesis Viewer는 전송되지 않는다. UI 화면은 Sim media worker의
observer/hand-eye track이다.

## QoS와 queue

| surface | QoS/규칙 |
| --- | --- |
| descriptor/heartbeat | reliable discovery; heartbeat timeout으로 expiry |
| control/authority/status | reliable bounded carrier |
| motion | best-effort volatile depth 1 |
| RGB-D | best-effort volatile depth 1, latest-only |
| WebRTC pixels | DDS QoS가 아니라 DTLS/SRTP/ICE |

ROS/DDS implementation이 제공하는 liveliness/deadline만으로 안전을 판정하지
않는다. authority TTL, Robot deadman, bounded application queue가 최종
경계다. QoS를 낮춰 렉을 숨기거나 control을 transient-local로 바꾸지 않는다.

## Reachability

지원되는 DDS graph는 common L2 LAN, routed LAN, routed VPN, mutually reachable
global IPv6다. multicast가 cross-router되지 않으면 직접 도달 가능한 static
peer로 discovery를 seed한다.

지원하지 않는 것:

- ordinary IPv4 NAT, CGNAT, symmetric NAT
- SSH local-forward를 DDS locator로 사용하는 것
- HTTP test server/SSH port/TURN port를 DDS address로 저장하는 것
- Coturn/Tailscale SSH로 DDS UDP를 relay한다고 주장하는 것

## Security

`trusted-network`는 owned LAN/routed VPN에서만 plaintext DDS를 허용한다.
공유망은 role-scoped SROS2 enforce(authentication, authorization, encryption)를
사용한다. `ROS_DOMAIN_ID`는 인증 경계가 아니다. SSH local-forward는
loopback-bound setup GUI에만 적용된다.

## Failure checklist

```text
container running
  → interface/address/route valid
  → descriptor + matching heartbeat live
  → Sim scene/media ready
  → lease/session grant
  → WebRTC signaling answer
```

각 단계는 독립적으로 실패할 수 있다. `sim-default` 미발견은 단순히
container state만으로 해결되지 않으며, `elesim-status`, role logs,
`elesim-net namespace-check`, connection-manager preflight를 순서대로 본다.
