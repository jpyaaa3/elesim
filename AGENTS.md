# Elesim Maintenance Guide

Read `misc/docs/architecture.md` before changing behavior that crosses a process,
protocol, media, configuration, model, or deployment boundary. Read
`misc/docs/setup.md` before changing the installer and `misc/docs/deployment.md`
before changing release or multi-host behavior.

## Runtime Topology

Elesim has five independently deployable applications:

- `router`: endpoint registry, leases, routing, WebRTC signaling, TURN credentials
- `controller`: perception, IK, Pick, Gaze, Vision, and target generation
- `ui`: operator presentation, intent, remote video, and simulation controls
- `simulator`: Genesis, virtual telemetry, RGBD, observer and hand-eye rendering
- `robot`: physical I/O, device feedback, deadman, limits, and local safety

Applications communicate through versioned contracts in `packages/protocol` or
through protocol-advertised media streams. Source sharing in this monorepo is a
development convenience, not a runtime dependency.

## Dependency Rules

- A deployment must not import a sibling deployment.
- UI uses operator and simulation protocol APIs, never controller workflow code.
- Robot must not know about IK, Pick, Genesis, builders, model source, or UI.
- Controller must not import Robot or Simulator implementations.
- Simulator consumes `model/bundles/default`; it rebuilds models only when
  `ELESIM_SIM_DEV_REBUILD=1` is explicitly set for development.
- Share only wire contracts and transport primitives through `packages/protocol`.
- Protocol changes require an explicit schema/version decision and corresponding
  contract and multi-process integration tests.
- Do not add root compatibility launchers, sibling re-export modules, or hidden
  imports that make isolated release installation pass only from the monorepo.

## Configuration And Generated Files

- Source defaults live in each deployment's top-level `config/` directory.
- Installed configuration is generated under the selected installation prefix;
  do not mutate source defaults during installation.
- A top-level deployment `config/` directory is runtime data, but a directory such
  as `src/elesim_simulator/config/` is Python application code. Never filter files
  recursively by basename when preparing build contexts.
- The Controller arm model and Simulator model bundle are immutable runtime inputs.
  Regenerate them with `misc/tooling/model_builder`, not inside a runtime process.
- Installer output, release contexts, and wheels are products that require their
  own isolation checks. A source-tree import test is not a substitute for checking
  the generated context or installed wheel.
- Preserve unrelated local changes, experiment evidence, and generated diagnostic
  output unless the task explicitly owns them.

## Installation And Operations

- The setup wizard must preserve existing host Python, CUDA, ROS, and APT state
  when container mode is selected.
- Generated Compose projects use a prefix-derived namespace and host networking.
  Do not assume `127.0.0.1` refers to another computer.
- `inherit` GPU mode forwards `CUDA_VISIBLE_DEVICES`; `specific` persists one GPU
  index or UUID; `cpu` must disable both container GPU access and the Genesis GPU
  backend.
- The remote Simulator profile is headless but keeps observer and hand-eye render
  streams enabled. A native Genesis Viewer requires an explicit display/X11
  attachment and must not silently become the server default.
- TURN selection currently generates TURN configuration and credentials. Coturn is
  a separate Compose service and must be started/stopped explicitly; documentation
  must not imply that `elesim-up` owns it unless the implementation is changed.
- Label multi-host commands with the machine that owns them. Do not tell users to
  create laptop configuration or destination directories on the compute server.
- `Ctrl+C` on `elesim-logs` stops log following, not the detached services.

## Security

- Non-loopback ZMQ uses CurveZMQ unless an explicit trusted-LAN development
  exception is selected.
- Never commit generated credentials, TURN secrets, private keys, or copied remote
  host configuration. `misc/infra/generated/` is the source-workspace scratch area.
- Distribute only role-required private keys. The complete credential root remains
  on the trusted Router administration host.
- WebRTC uses DTLS/SRTP; Coturn uses short-lived REST credentials issued by Router.
- SSH/`scp` is only a credential transfer mechanism and is not part of Elesim media
  or control transport. Respect non-default SSH ports.
- Revoke temporary X access such as `xhost +si:localuser:root` after stopping a
  Viewer-enabled container.
- Cleanup instructions must name the exact installation prefix. Avoid global Docker
  pruning or broad CUDA/environment changes on shared research machines.

## Code Shape

- Keep ownership visible in directory and class structure. A reader should be able
  to follow a runtime path without opening a large number of one-method files.
- Split classes that accumulate unrelated lifecycle, transport, domain, and UI
  responsibilities; do not split cohesive code merely to reduce line count.
- Prefer existing domain objects and structured parsers over ad hoc dictionaries or
  string rewriting in production code.
- Keep comments focused on invariants, constraints, and non-obvious decisions.

## Verification

For normal changes, run the canonical gate:

```bash
python3 misc/tooling/quality/check.py --group required
```

For structural, installer, protocol, or release changes also run:

```bash
python3 misc/tooling/quality/check.py --group extended
python3 misc/tooling/release/build.py
python3 misc/tooling/release/verify.py dist/releases
```

The detailed per-package matrix is in `misc/docs/architecture.md`. At minimum,
changes must test the owning package. Cross-process changes also require
`misc/integration/smoke_topology.py`. Installer copy/filter changes must assert the
contents of generated contexts and built wheels, including nested Python packages.

Automated tests do not establish real Genesis GPU rendering, TURN relay selection
across an actual NAT, loaded-network WebRTC latency, RealSense behavior, Dynamixel
motion, or GO2 stability. Record those as explicit manual validation results rather
than presenting unit-test success as hardware proof.

## Documentation

- `README.md` is the Korean operator guide and must match commands generated by the
  current installer.
- `misc/docs/architecture.md` owns system boundaries and protocol invariants.
- `misc/docs/setup.md` owns installer internals and network-doctor interpretation.
- `misc/docs/deployment.md` owns release and multi-host deployment detail.
- Update both `misc/docs/OPEN_ISSUES.md` and `misc/docs/OPEN_ISSUES_KR.md` when a
  newly discovered limitation remains unfixed.
