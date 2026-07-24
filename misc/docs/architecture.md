# Elesim Architecture

Elesim is a monorepo of independently built release projects. Source sharing is a
development convenience, not a runtime dependency.

## Runtime Topology

```text
Laptop                         Control plane                  Compute PC
+----------------------+       +-------------------+          +--------------------+
| UI operator endpoint |------>|                   |<---------| Simulator endpoint |
| UI simulator endpoint|------>| Router            |          | Genesis main thread|
| Controller           |------>| registry + leases |          +---------+----------+
+----------+-----------+       +---------+---------+                    ||
           ^                             ^                              ||
           |                             |                              ||
           |              Robot Jetson   |                              ||
           |              +--------------+----+                         ||
           +==============| Robot endpoint     |                         ||
           | CurveZMQ RGBD | local I/O + safety|                         ||
           |              +-------------------+                         ||
           |                                                         WebRTC
           +============= CurveZMQ RGBD ================================||
           +================ observer + hand-eye WebRTC ================+
```

The router may run on any mutually reachable host. A public compute server
normally runs Router, Simulator and optional Coturn together, while UI and
Controller remain on the laptop. UI never imports controller workflow code.
Controller never imports robot or simulator packages. Robot does not know
about model assets, IK, Pick, Genesis or UI.

The two UI endpoint identities are intentional. The operator endpoint sends
workflow intent to Controller. The simulator endpoint owns a separate
simulation-operator session and talks to Simulator through Router without
making Controller a video or camera-input relay.

## Ownership

| Release project | Owns | Does not own |
| --- | --- | --- |
| UI | presentation, operator intent, simulator view input, rendered-video receive | IK, workflow, hardware |
| Controller | Vision, Arm model, Look/Aim/Grasp, Gaze, target generation | physical I/O, Genesis |
| Router | endpoint lifecycle, discovery, motion lease, simulation session, signaling, TURN credentials | domain algorithms, media relay |
| Robot | Dynamixel/GO2 drivers, RGBD publishing, deadman, current limits | assets, builders, workflow |
| Simulator | Genesis runtime, model loading, virtual telemetry, observer/hand-eye rendering, simulation commands | operator workflow, hardware |

## Dependency Rule

```text
{router,controller,ui,robot,simulator} -> elesim_protocol + third-party packages
misc/tooling/model_builder -> misc/model/source + controller model schema
misc/tooling/release -> top-level release projects + protocol project
misc/tooling/setup -> packages/protocol public API + deployment artifacts on disk
misc/infra/containers -> setup-generated isolated role image contexts
misc/infra/development -> setup-generated all-in-one coding environment only
misc/integration -> public process/protocol surfaces
```

A release project must not import a sibling project or a repository-root legacy
module. Communication between deployed processes is always a protocol message or a
documented media stream.

The developer container deliberately co-locates all source projects for coding
and tests, but it does not weaken release ownership: no deployment wheel or
general-user role image may import a sibling deployment.

## Model Lifecycle

`misc/model/source` is builder input. `misc/tooling/model_builder` creates immutable
artifacts under `model/bundles`. The simulator consumes a prebuilt bundle by
default. Runtime rebuilding is a development-only operation enabled explicitly
with `ELESIM_SIM_DEV_REBUILD=1`.

The controller likewise reads `config/arm_model.json` and never constructs an
assembly at runtime. The installed model-builder commands regenerate both
artifacts offline; arm-model intermediate files live in a temporary workspace:

```bash
elesim-build-sim-bundle --assets misc/model/source/assets --output model/bundles/default
elesim-build-arm-model --config controller/config/config.pc.yaml \
  --assets misc/model/source/assets --output controller/config/arm_model.json
```

## Protocol Invariants

- Protocol version is exactly v4; old envelopes are rejected.
- Motion targets carry canonical four-element `q`, never hardware `u` values.
- A controller leases at most one robot or simulator endpoint.
- A simulator grants at most one independent UI simulation session.
- Motion leases and simulation sessions are separate authorities: camera or
  pause input cannot grant arm-motion ownership.
- Switching or disconnecting revokes the previous lease.
- Robot and simulator reject stale sequences and mismatched leases.
- Estop bypasses the ordinary active-command path but remains role checked.
- Large media bypasses the router and uses advertised direct endpoints.
- WebRTC signaling uses Router, but observer and hand-eye pixels do not.

## Remote Simulator Semantics

Elesim does not transport the native Genesis desktop window. Simulator owns a
dedicated observer camera whose output is equivalent to the inspectable scene
view needed by an operator. UI receives that observer stream and the robot's
hand-eye preview as two independent WebRTC tracks. Mouse orbit, pan and zoom,
plus pause, resume, single-step, reset, speed, reset-view and debug-marker
commands are versioned protocol messages.

Commands enter a bounded mailbox and are applied only on the Genesis main
thread. Pausing stops physics, not endpoint heartbeats, status delivery or the
WebRTC sessions. Reset increments the simulation epoch; Controller stops an
active Pick/Gaze workflow when it observes a pause edge or epoch change.

## Network Security

- Loopback-only development may use plaintext ZMQ.
- A non-loopback Router or direct RGBD endpoint requires CurveZMQ unless an
  explicit development override is enabled.
- Router authenticates each public key against an exact endpoint ID and role.
- Simulator and Robot advertise a Curve-protected direct RGBD stream and use a
  ZAP allowlist containing only Controller's media client key.
- WebRTC media uses its DTLS/SRTP transport. Coturn is optional for direct LAN
  connectivity and supplies relay candidates for NAT traversal.
- Router issues short-lived TURN REST credentials; the static HMAC secret is
  held only by Router and Coturn.
- Before TURN credentials expire, Router refreshes both peers. UI rebuilds the
  observer and hand-eye peer connections inside the existing simulation
  session and swaps them only after both replacement offers are created.

## Verification Matrix

The canonical entry point runs this matrix with package-specific import paths:

```bash
python3 misc/tooling/quality/check.py --group required
```

The required gate covers protocol, all five release projects, model/release/setup
tooling, the five-process topology smoke, an authenticated CurveZMQ RGBD
roundtrip, and actual encoded observer/hand-eye WebRTC tracks. The extended
gate covers offline tools, readability budgets, and focused mutation checks:

```bash
python3 misc/tooling/quality/check.py --group extended
```

The equivalent individual commands are:

```bash
PYTHONPATH=packages/protocol/src python3 -m pytest packages/protocol/tests
PYTHONPATH=packages/protocol/src:router/src python3 -m pytest router/tests
PYTHONPATH=packages/protocol/src:robot/src python3 -m pytest robot/tests
PYTHONPATH=packages/protocol/src:controller/src python3 -m pytest controller/tests
PYTHONPATH=packages/protocol/src:simulator/src python3 -m pytest simulator/tests
PYTHONPATH=packages/protocol/src:ui/src python3 -m pytest ui/tests
PYTHONPATH=misc/tooling/model_builder/src:controller/src:packages/protocol/src python3 -m pytest misc/tooling/model_builder/tests
PYTHONPATH=packages/protocol/src:misc/tooling/setup/src python3 -m pytest misc/tooling/setup/tests
PYTHONPATH=packages/protocol/src:router/src python3 misc/integration/smoke_topology.py
```

Release artifacts have a separate isolation gate. Building release contexts
installs each protocol/application wheel pair into a clean temporary target,
loads deployment configuration, validates the simulator bundle, checks that no
sibling deployment is visible, and invokes the packaged console entry point
with `--help`:

```bash
python3 misc/tooling/release/build.py
python3 misc/tooling/release/verify.py dist/releases
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
- Transport integration tests use real CurveZMQ/ZAP sockets and real aiortc
  sender/receiver pairs for both named video streams.
- Focused mutations prove that critical version, lease, stale-command,
  deadman, control-direction, gain, and finite-input guards are observed by the
  tests.
- Live Genesis and hardware-in-loop validation remains a manual gate because
  software-only tests cannot establish physical convergence or camera timing.
- Setup-tool tests generate each security/address profile, exercise safe
  bootstrap extraction, connect to real plaintext and CURVE Router processes,
  and validate TCP/STUN probes without importing a sibling deployment.

Generate a role-specific line-execution report without adding a production
dependency:

```bash
python3 misc/tooling/quality/line_coverage.py controller
```

Use the development container when scientific or graphics dependencies are not
installed on the host.
