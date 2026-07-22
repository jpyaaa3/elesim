# Distributed Runtime

This document describes network behavior. Installation commands are in
[`deployment.md`](deployment.md), and code ownership is in
[`architecture.md`](architecture.md).

## Control Plane

Every deployment connects to Router through a ZMQ DEALER socket and registers a
protocol-v4 endpoint descriptor. The descriptor contains its exact identity,
role, capabilities and direct-stream advertisements.

```text
UI operator endpoint --operator_intent--> Controller
Controller ----------motion_command-----> Robot or Simulator
Controller <---------telemetry/ack-------- Robot or Simulator

UI simulator endpoint --simulation_command--> Simulator
UI simulator endpoint <--simulation_status--- Simulator
                         ^
                         | discovery, two lease types, forwarding
                       Router
```

The Controller motion lease and UI simulation session are independent:

- Controller is the sole arm/GO2 command owner for one Robot or Simulator.
- UI is the sole observer/pause/reset owner for one Simulator.
- Acquiring a simulation session never grants motion authority.
- Heartbeat expiry, target switch or explicit close revokes the corresponding
  authority and WebRTC signaling session.

Simulation commands are accepted on Router's networking thread, placed in a
bounded Simulator mailbox and executed on the Genesis main thread. Adjacent
orbit, pan and zoom deltas may be coalesced, but lifecycle commands are not.

## Media Plane

| Stream | Sender | Receiver | Transport |
| --- | --- | --- | --- |
| Physical RGBD | Robot | Controller | direct CurveZMQ, latest frame |
| Simulated RGBD | Simulator | Controller | direct CurveZMQ, latest frame |
| Observer scene | Simulator | UI | WebRTC |
| Hand-eye preview | Simulator | UI | separate WebRTC peer |

Router forwards only SDP signaling and session metadata. It never forwards
camera pixels. Captured Simulator frames enter one bounded `FrameHub`; RGBD and
WebRTC consumers read the latest frame instead of invoking Genesis capture
independently.

The observer stream is the remote-control surface for the Genesis scene. It is
not a screen capture of the native Viewer. UI mouse input produces orbit, pan
and zoom commands, and the toolbar exposes pause/resume, single-step, reset,
speed, reset-view and debug-marker visibility. Clicking the hand-eye preview
swaps it into the main viewport; camera manipulation remains attached only to
the observer stream.

## WebRTC And TURN

On a flat LAN, ICE can usually connect UI directly to Simulator. If NAT or a
firewall prevents that path, deploy Coturn and configure Router with its TURN
URL and shared REST HMAC secret. Router mints bounded-lifetime credentials when
it grants a simulation session; neither UI nor Simulator stores the static
secret.

The observer and hand-eye streams use independent peer connections. A failed
stream therefore does not silently masquerade as a connected second view.
Router refreshes both peers' short-lived credentials before expiration. UI
creates replacement peer connections for both streams, sends new offers under
the existing simulation lease, and closes the old receivers only after the
replacement negotiation has started. A local replacement failure leaves the
working receivers and simulation session intact.

## Security Rules

- Plaintext ZMQ is accepted only for loopback defaults.
- Public Router and RGBD endpoints require CurveZMQ by default; RGBD publishers
  additionally allow only Controller's media client key through ZAP.
- Router's authenticator maps a Curve public key to exact `(endpoint_id, role)`
  pairs before registration.
- UI uses one Curve identity for both `ui-main` and the explicitly authorized
  `ui-main-simulator` endpoint.
- WebRTC media is encrypted by DTLS/SRTP; TURN relays encrypted packets.
- Generated private keys and TURN secrets are not committed to Git.

## Failure Rules

- Heartbeat expiry removes an endpoint and revokes its leases/sessions.
- Robot safety and deadman logic remain local without Router availability.
- Sequence numbers are monotonic per endpoint; stale commands are rejected.
- A Simulator reset increments `epoch`. Controller cancels active Pick/Gaze
  work rather than continuing against a replaced world.
- Simulator pause freezes physics while protocol liveness, status and media
  sessions remain established.
- A malformed stream descriptor is reported as endpoint health; it does not
  terminate Controller's routing thread.

Software tests cover routing, lease isolation, command/status forwarding,
authenticated direct RGBD, TURN refresh behavior, and encoded frames through
both real aiortc peers. Actual Genesis GPU rendering, codec timing under load,
TURN relay selection, packet loss and multi-host latency remain live deployment
gates.
