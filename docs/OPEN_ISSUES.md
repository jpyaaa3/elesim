# Current Open Issues

Status snapshot: 2026-08-17. This is a live limitation list, not a historical
backlog. The architecture and operator boundaries are in
[`architecture.md`](architecture.md); dated audits remain under
[`audit/`](audit/).

## P0 — must be proven before claiming production readiness

1. **Real multi-host DDS.** Verify descriptor/heartbeat convergence, addressed
   control, lease/session expiry, RGB-D latest-only behavior and stale-boot
   rejection on two to four real hosts.
2. **SROS2 enforce.** Verify the generated role permissions deny unauthorized
   publish/subscribe and that managed rotation revokes the replaced generation.
3. **Physical safety.** Verify Robot/Unitree bridge stop deadlines, arm cleanup,
   deadman behavior and hardware failure handling on the intended Jetson/GO2.

## P1 — required operational gates

- Docker Desktop Tailscale sidecar: enroll two nodes, confirm namespace/address/
  route, static discovery and bidirectional DDS; SSH success alone is not proof.
- Routed LAN/VPN and global IPv6 behavior; explicit diagnostics for unsupported
  IPv4 NAT/CGNAT/symmetric NAT.
- WebRTC observer and hand-eye tracks through direct and Coturn relay ICE, with
  SDP bounds, DTLS/SRTP and renegotiation under peer loss.
- GPU/CPU profiles, NVIDIA reservation/CUDA visibility, NVENC/libx264 fallback,
  X11/WSLg display ownership and actual Genesis Viewer behavior.
- Measure frame age, bandwidth, loss and p95 latency; separate Genesis render,
  camera conversion, transfer, media encode and CPU MPC timing.
- Validate the connection manager's SSH fingerprint pinning, non-default
  OpenSSH port and Tailscale SSH check re-auth on real hosts.

## P2 — planned follow-up

- Runtime wiring for the generated typed ROS services/actions; protocol v6
  `PeerEnvelope` remains the active control/signaling surface today.
- Broader property/stress coverage for perception, camera timing, MPC and
  physical adapter paths.
- Revisit upstream Genesis inertia, neutral-qpos and self-collision warnings
  after model/physics owners provide a measured correction.
- Improve operator diagnostics only when they preserve bounded queues, explicit
  failure and the existing authority/security boundaries.

## Not an issue / do not “fix” by weakening the design

- Coturn does not carry DDS. SSH port forwarding does not supply a DDS locator.
- `ROS_DOMAIN_ID` is not authentication or tenant isolation.
- A running Sim container is not the same as a granted UI simulation session;
  scene/media readiness and exact boot heartbeat are separate gates.
- `elesim-update` does not replace running containers. Apply rebuilt artifacts
  with `elesim-up`; use `elesim-down` first for a deliberate full restart.
- Historical Router/ZMQ notes, pre-v4 topology records and dated audit failures
  are not current runtime defects.
