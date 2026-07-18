# Elesim

Elesim controls and simulates a segmented arm mounted on a Unitree GO2. The
repository is a source workspace for five independently deployable programs;
it is not one application installed as a shared source tree.

## Deployments

| Package | Typical machine | Responsibility |
| --- | --- | --- |
| `elesim-ui` | laptop | ImGui operator surface and simulator video |
| `elesim-controller` | laptop | perception, IK, Pick/Gaze and command generation |
| `elesim-router` | laptop or compute PC | registry, discovery, leases and message routing |
| `elesim-robot` | robot Jetson | motor/GO2 I/O, RGBD capture and local safety |
| `elesim-simulator` | compute PC or laptop | Genesis physics, virtual sensors and rendering |

All five exchange protocol-v3 envelopes over ZMQ. Large RGBD streams and
rendered video use advertised direct paths instead of passing through the
router. Only `packages/protocol` is shared between deployment artifacts.

## Repository Layout

```text
packages/protocol/          Versioned wire contract and transport helpers
deployments/ui/             Laptop UI artifact
deployments/controller/     Laptop control-computation artifact
deployments/router/         Network router artifact
deployments/robot/          Jetson artifact and systemd unit
deployments/simulator/      Compute artifact
model/source/               Source robot geometry and blueprint
model/bundles/default/      Prebuilt simulator model bundle
tooling/model_builder/      Offline model compiler
tooling/release/            Isolated release-context builder
integration/                Cross-process topology checks
docs/                       Architecture and operations documents
```

## Development

Run a deployment from source by exposing only its package and the protocol:

```bash
PYTHONPATH=packages/protocol/src:deployments/router/src python3 -m elesim_router.main
PYTHONPATH=packages/protocol/src:deployments/controller/src python3 -m elesim_controller.main
PYTHONPATH=packages/protocol/src:deployments/simulator/src python3 -m elesim_simulator.main
PYTHONPATH=packages/protocol/src:deployments/ui/src python3 -m elesim_ui.main
```

The robot package is normally installed from its release directory on the
Jetson. See [deployment.md](docs/deployment.md).

## Release And Verification

```bash
python3 tooling/release/build.py
PYTHONPATH=packages/protocol/src:deployments/router/src python3 integration/smoke_topology.py
```

Each deployment owns its tests and dependencies. The full verification matrix
is documented in [architecture.md](docs/architecture.md).
