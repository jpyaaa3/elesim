# Distributed Runtime

This document describes network behavior. Installation commands are in
[`deployment.md`](deployment.md), and code ownership is in
[`architecture.md`](architecture.md).

## Control Plane

Every deployment connects to the router through a ZMQ DEALER socket and first
registers a protocol-v3 endpoint descriptor. The descriptor contains role,
capabilities and direct-stream advertisements.

```text
UI --operator_intent--> Controller --motion_command--> Robot or Simulator
UI <--operator_result-- Controller <--telemetry/ack--- Robot or Simulator
                         ^
                         | discovery + lease
                       Router
```

The UI's endpoint selection is an operator intent. The controller requests the
actual lease and remains the sole command owner. A robot and simulator are
interchangeable motion targets at the wire boundary.

## Media Plane

- Robot RGBD: direct latest-frame ZMQ publisher to controller.
- Simulator RGBD: direct latest-frame ZMQ publisher to controller.
- Simulator rendered view: direct WebRTC sender to UI.
- WebRTC signaling: small envelopes forwarded by the router.

Viewer orbit, pan and zoom inputs travel UI -> controller -> router ->
simulator as `camera_input`; the compute machine remains authoritative for the
rendered camera.

## Failure Rules

- Heartbeat expiry removes the endpoint and revokes its leases.
- A target accepts commands only from its active controller lease.
- Sequence numbers are monotonic per endpoint and stale messages are rejected.
- Robot deadman and safety limits operate locally without router availability.
- ZMQ endpoints are intended for a trusted LAN or VPN, not the public Internet.
