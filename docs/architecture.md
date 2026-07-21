# Elesim Architecture

Elesim is a monorepo of independently built deployments. Source sharing is a
development convenience, not a runtime dependency.

## Runtime Topology

```text
Laptop                                      Robot Jetson
+-------------------+                       +-------------------+
| UI                |-- operator_intent --> |                   |
| Controller        |-- motion_command -----| Router -----------|--> Robot
| perception/IK/pick|<-- telemetry ---------| registry + leases |<-- RGBD
+-------------------+                       +---------+---------+
                                                      |
                                                      | ZMQ
                                            +---------v---------+
                                            | Simulator         |
                                            | Genesis + WebRTC  |
                                            +-------------------+
```

The router may run on the laptop or compute PC. UI never imports controller
workflow code. Controller never imports robot or simulator packages. Robot
does not know about model assets, IK, Pick, Genesis or UI.

## Ownership

| Deployment | Owns | Does not own |
| --- | --- | --- |
| UI | presentation, operator intent, rendered-video receive | IK, workflow, hardware |
| Controller | Vision, Arm model, Look/Aim/Grasp, Gaze, target generation | physical I/O, Genesis |
| Router | endpoint lifecycle, discovery, lease authority, forwarding | domain algorithms |
| Robot | Dynamixel/GO2 drivers, RGBD publishing, deadman, current limits | assets, builders, workflow |
| Simulator | Genesis runtime, model loading, virtual telemetry, video send | operator workflow, hardware |

## Dependency Rule

```text
deployments/* -> elesim_protocol + third-party packages
tooling/model_builder -> model/source + controller model schema
tooling/release -> deployment projects + protocol project
integration -> public process/protocol surfaces
```

A deployment must not import a sibling deployment or a repository-root legacy
module. Communication between deployments is always a protocol message or a
documented media stream.

## Model Lifecycle

`model/source` is builder input. `tooling/model_builder` creates immutable
artifacts under `model/bundles`. The simulator consumes a prebuilt bundle by
default. Runtime rebuilding is a development-only operation enabled explicitly
with `ELESIM_SIM_DEV_REBUILD=1`.

The controller likewise reads `config/arm_model.json` and never constructs an
assembly at runtime. The installed model-builder commands regenerate both
artifacts offline; arm-model intermediate files live in a temporary workspace:

```bash
elesim-build-sim-bundle --assets model/source/assets --output model/bundles/default
elesim-build-arm-model --config deployments/controller/config/config.pc.yaml \
  --assets model/source/assets --output deployments/controller/config/arm_model.json
```

## Protocol Invariants

- Protocol version is exactly v3; old envelopes are rejected.
- Motion targets carry canonical four-element `q`, never hardware `u` values.
- A controller leases at most one robot or simulator endpoint.
- Switching or disconnecting revokes the previous lease.
- Robot and simulator reject stale sequences and mismatched leases.
- Estop bypasses the ordinary active-command path but remains role checked.
- Large media bypasses the router and uses advertised direct endpoints.

## Verification Matrix

The canonical entry point runs this matrix with package-specific import paths:

```bash
python3 tooling/quality/check.py --group required
```

The required gate covers protocol, all five deployments, model/release tooling,
and the five-process topology smoke. The extended gate covers offline tools,
readability budgets, and focused mutation checks:

```bash
python3 tooling/quality/check.py --group extended
```

The equivalent individual commands are:

```bash
PYTHONPATH=packages/protocol/src python3 -m pytest packages/protocol/tests
PYTHONPATH=packages/protocol/src:deployments/router/src python3 -m pytest deployments/router/tests
PYTHONPATH=packages/protocol/src:deployments/robot/src python3 -m pytest deployments/robot/tests
PYTHONPATH=packages/protocol/src:deployments/controller/src python3 -m pytest deployments/controller/tests
PYTHONPATH=packages/protocol/src:deployments/simulator/src python3 -m pytest deployments/simulator/tests
PYTHONPATH=packages/protocol/src:deployments/ui/src python3 -m pytest deployments/ui/tests
PYTHONPATH=tooling/model_builder/src:deployments/controller/src:packages/protocol/src python3 -m pytest tooling/model_builder/tests
PYTHONPATH=packages/protocol/src:deployments/router/src python3 integration/smoke_topology.py
```

Release artifacts have a separate isolation gate. Building release contexts
installs each protocol/application wheel pair into a clean temporary target,
loads deployment configuration, validates the simulator bundle, checks that no
sibling deployment is visible, and invokes the packaged console entry point
with `--help`:

```bash
python3 tooling/release/build.py
python3 tooling/release/verify.py dist/releases
```

## Test Layers

- Contract tests pin protocol, payload, lease, safety, configuration, and role
  boundaries.
- Deterministic property tests exercise UV, LJI, equal-sag, ready-pose and
  reachable FK-to-IK invariants over broad generated inputs.
- Headless workflow tests execute Look -> Aim -> Grasp phase ordering without
  Genesis or camera windows.
- Recorded-log replay turns known field failures into deterministic regression
  reports.
- Focused mutations prove that critical version, lease, stale-command,
  deadman, control-direction, gain, and finite-input guards are observed by the
  tests.
- Live Genesis and hardware-in-loop validation remains a manual gate because
  software-only tests cannot establish physical convergence or camera timing.

Generate a role-specific line-execution report without adding a production
dependency:

```bash
python3 tooling/quality/line_coverage.py controller
```

Use the development container when scientific or graphics dependencies are not
installed on the host.
