# Jetson mixed-role rollout — historical record

> Historical implementation note from 2026-08-09. This is not a live runbook or
> acceptance result. Current Robot/Compose ownership and commands are in
> [`deployment.md`](deployment.md), and current host lifecycle is in
> [`setup.md`](setup.md).

## Why the layout exists

A Jetson may own two independent deployment units:

```text
robot-native  → elesim-robot.service + elesim-unitree-bridge.service
runtime       → validated container roles such as Pilot/UI
```

The units have separate prefixes, ownership manifests, security role views and
lifecycle commands. Robot remains native-only and Unitree DDS remains private to
the Jetson–GO2 NIC/domain. Robot and bridge exchange bounded credential-checked
Unix packets; Unitree topics are not part of the inter-host EleSim graph.

## Historical decisions retained as invariants

- A Robot assignment is valid only on a detected Jetson with the native unit.
- A second container installation uses a different prefix and fixed
  `elesim-runtime` project; it never adopts or replaces the Robot unit.
- The bridge receives no EleSim SROS2 Authority or aggregate keystore.
- `elesim-connections` coordinates existing units but does not install systemd,
  create accounts, or silently replace a unit.
- Sim on ARM64 remains a separate image/runtime acceptance gate.

The original execution log and proposal text were intentionally removed from the
current documentation set: dated command transcripts are preserved by Git
history and the audit directory, while repeating them as instructions caused
configuration drift.
