# Distributed Runtime

This document describes network behavior. Installation commands are in
[`deployment.md`](deployment.md), and code ownership is in
[`architecture.md`](architecture.md).

## DDS Graph

Every deployment is a ROS 2 participant. There is no application Router,
central registry, ZMQ socket, or CURVE transport. A deployment publishes an
`EndpointDescriptor` and periodic `EndpointHeartbeat` on:

```text
/<system_id>/v5/discovery/endpoints
/<system_id>/v5/discovery/heartbeats
```

The descriptor carries `PeerRef`, role, capabilities, stream descriptors, and
boot-specific `service_prefix`/`topic_prefix`. Descriptor QoS is reliable,
transient-local, keep-last-64. Heartbeats are reliable, volatile, keep-last-64 at
1 Hz; consumers expire a peer after 3.5 seconds. Two live boots claiming one
logical endpoint ID are an error; consumers refuse to acquire authority until
the conflict clears.

All resources live below the canonical `/<system_id>/v5` root. A deployment
builds valid, collision-resistant, boot-specific ROS prefixes from its role,
endpoint ID and boot ID, then advertises the exact values. Consumers must use
those advertised prefixes instead of reconstructing or independently
sanitizing a logical ID such as `sim-default`.

```text
UI -------- reliable control message --------> Controller
Controller -------- best-effort motion ------> Robot or Simulator
Controller <------- reliable reply/state ------ Robot or Simulator

UI -------- reliable control message --------> Simulator
UI <------- status + WebRTC reply ------------ Simulator
```

## Motion Authority

Robot and Simulator each own and serialize access to their own motion lease:

- `select_target` is an idempotent request on the target's reliable control
  topic. It identifies the exact Controller and target boots; the response
  contains a target-issued lease epoch and opaque token.
- `renew_target` is sent before the bounded TTL, and `release_target`
  explicitly relinquishes the lease. The target revokes the lease when renewal
  stops.
- `motion_command` carries both process-instance identities, a monotonic
  sequence, timestamp, and lease token. A target drops a command for a
  mismatched lease, previous process instance, or stale sequence. Ordinary
  motion uses best-effort keep-last-1 delivery; the existing 0.5-second command
  deadman remains local.
- Estop uses the reliable control carrier, bypasses the ordinary active-motion
  lease check, remains source-role checked, and never disables local hardware
  safety.

Controller leases at most one target. A target switch is fail-closed: stop and
release the old target before commanding the new one. DDS discovery alone
never grants motion authority.

## Operator And Simulation Authority

UI sends bounded `operator_intent` requests to Controller and receives
`operator_result` snapshots on the reliable control carrier. Long-running Pick,
Gaze, and IK cancellation is currently expressed through the same versioned
request/reply contract.

Simulator owns one independent UI simulation session:

- `open_simulation_session` and `close_simulation_session` are idempotent
  request/reply messages.
- UI sends `renew_simulation_session` before the bounded TTL; Simulator revokes
  the session after approximately 2 seconds without a valid renewal.
- The session payload carries the granted authority, epoch, token, streams and
  TURN credentials; status/result messages carry state and command results.
- orbit, pan, zoom, pause, resume, step, reset, speed, reset-view and debug
  visibility use bounded reliable `simulation_command` messages.

Acquiring a UI session never grants motion authority. Simulation commands are
placed in a bounded Simulator mailbox and executed on the Genesis main thread;
the ROS executor never mutates Genesis directly.

## Media Plane

| Stream | Sender | Receiver | Transport |
| --- | --- | --- | --- |
| Physical RGBD | Robot | Controller | DDS `rgbd/frame`, latest coherent sample |
| Simulated RGBD | Simulator | Controller | DDS `rgbd/frame`, latest coherent sample |
| Observer scene | Simulator | UI | WebRTC |
| Hand-eye preview | Simulator | UI | separate WebRTC peer |

`RgbdFrame` contains `PeerRef source`, frame sequence, `std_msgs/Header`,
`sensor_msgs/Image` color and depth, `sensor_msgs/CameraInfo`, and
`depth_scale`. Its topic is best-effort, volatile, keep-last-1 with a finite
lifespan so an overloaded subscriber drops old frames instead of accumulating
latency. Raw 640x480 RGB plus depth at 30 Hz is roughly 369 Mbit/s before
DDS/UDP overhead; production profiles must validate RTPS fragmentation,
resource limits, MTU, loss and frame age and may need a separately versioned
compressed RGBD representation. Do not silently put compressed bytes into a
`sensor_msgs/Image` whose encoding claims raw pixels.

## Interface Package

`packages/elesim_interfaces` is the only ROS wire-contract package. The
runtime-wired messages are:

- `PeerRef`, `StreamDescriptor`, `EndpointDescriptor`, and
  `EndpointHeartbeat` for discovery;
- `RgbdFrame` for coherent camera samples;
- `PeerEnvelope` for bounded control, authority, telemetry, status, and WebRTC
  signaling payloads.

Interfaces carry bounded strings/arrays where practical. The interface package
does not hardcode ROS names; deployments advertise their boot-specific
prefixes in `EndpointDescriptor`. Payload schemas inside `PeerEnvelope` are
versioned and validated by `elesim_protocol`.

The package also declares more specific motion, operator, simulation, and TURN
messages/services plus `RunOperatorWorkflow.action`. They are forward contract
work and are generated for compatibility testing, but the current runtime does
not create those services/actions. They must not appear in an SROS2 policy or
network-doctor success criterion until a deployment actually binds them.

The observer stream is the remote-control surface for the Genesis scene. It is
not a screen capture of the native Viewer. UI mouse input produces orbit, pan
and zoom commands, and the toolbar exposes pause/resume, single-step, reset,
speed, reset-view and debug-marker visibility. Clicking the hand-eye preview
swaps it into the main viewport; camera manipulation remains attached only to
the observer stream.

## QoS Contract

| Surface | QoS and application rule |
| --- | --- |
| endpoint descriptor | reliable, transient-local, depth 64 |
| endpoint heartbeat | reliable, volatile, depth 64; 1 Hz; expire after 3.5 s |
| reliable control carrier | reliable, volatile, depth 64; bounded application queues |
| motion command | best-effort, volatile, depth 1; 0.5 s local deadman |
| estop | reliable control carrier; source-role checked |
| telemetry/status/replies | reliable control carrier in the current runtime |
| RGBD | best-effort, volatile, depth 1; finite lifespan |

QoS compatibility is part of the wire contract. Endpoint heartbeats,
lease/session renewal TTLs and deadman timers remain authoritative because DDS
deadlines/liveliness support and reporting differ between RMW implementations.

## WebRTC Signaling And TURN

UI sends a bounded `webrtc_signal` offer on the Simulator's reliable control
topic. It contains the active simulation-session token, stream name, offer type,
and SDP offer (maximum 524288 characters). Simulator validates the session and
returns an answer on the UI's reliable control topic. Observer and hand-eye
negotiate independently, but UI swaps a refreshed pair only after both
replacements succeed. The current non-trickle design needs no ICE-candidate
topic; adding trickle ICE requires an explicit future interface/version change.

WebRTC media always uses DTLS/SRTP. On a flat LAN, ICE can usually connect UI
directly to Simulator. Coturn can relay the encrypted media when direct ICE is
unavailable. In managed mode its REST HMAC secret is mounted into Coturn and
the co-located Simulator. Simulator issues usable, bounded-lifetime credentials
for itself and the active UI session; UI never receives the static secret.
External TURN instead loads a bounded username/credential JSON file in
Simulator only and supplies the usable value to the active UI session. Managed
TURN requires `sros2` because issued credentials and signaling cross DDS.
External mode may use `trusted-network` only under its controlled-LAN/VPN trust
assumption; use SROS2 when the DDS network is shared or observable.

TURN does not carry DDS signaling. If UI and Simulator cannot exchange DDS
traffic, they cannot exchange the SDP needed to establish WebRTC even when both
can reach the TURN server.

## DDS Security Profiles

- `trusted-network`: no DDS encryption. Use only on an owned LAN or routed VPN,
  bind to a chosen interface, and restrict UDP reachability with host/network
  firewalls.
- `sros2`: use DDS Security authentication, access control and encryption in
  enforce mode on untrusted/shared networks. Give each role a distinct enclave
  whose permissions expose only its required DDS topics.

`ROS_DOMAIN_ID` is a discovery partition and collision-avoidance setting, not a
security boundary. WebRTC DTLS/SRTP is unchanged in both profiles. SSH local
forwarding protects only the loopback-bound installation GUI and is not a DDS
or media transport.

## P2P Reachability

DDS data uses direct UDP locators. Supported layouts are a common L2 LAN, a
routed LAN with configured unicast peers, a routed VPN, or mutually reachable
global IPv6. Multicast discovery is not expected to cross routers; static peers
can seed discovery but do not relay user data.

Ordinary IPv4 NAT, CGNAT and symmetric NAT are unsupported. Server-side port
forwarding alone is insufficient because every required DDS participant and
user-data locator must be reachable bidirectionally. DDS has no ICE/TURN-style
NAT traversal. A routed VPN is the supported answer for a laptop behind NAT;
adding a DDS relay would violate this architecture's direct P2P constraint.

## Failure Rules

- Endpoint-heartbeat expiry removes a boot from the local view; lease/session
  renewal TTL expiry independently revokes authority at the owning target.
- Robot safety and deadman logic remain local without Controller or DDS
  availability.
- Sequence numbers are monotonic per process instance; stale commands from an
  old instance are rejected.
- A Simulator reset increments `epoch`. Controller cancels active Pick/Gaze
  work rather than continuing against a replaced world.
- Simulator pause freezes physics while DDS liveness, status and media
  sessions remain established.
- A malformed descriptor or frame is reported as endpoint health; it does
  not terminate another participant's executor.

Software tests must cover target-owned lease/session isolation, restart UUIDs,
stale sequence rejection, command/status delivery, QoS compatibility, RGBD
coherence, WebRTC negotiation, and encoded frames through both real aiortc
peers. Actual DDS discovery on each supported network, SROS2 enforcement,
Genesis GPU rendering, codec timing under load, TURN relay selection, packet
loss, Wi-Fi/VPN reconnect and multi-host latency remain live deployment gates.
