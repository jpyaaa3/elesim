# Elesim Architecture

Elesim is a monorepo of independently built release projects. Source sharing is a
development convenience, not a runtime dependency.

## Runtime Topology

```text
Laptop                                      Compute PC
+----------------------+    ROS 2 / DDS     +--------------------+
| UI                   |<==================>| Simulator          |
| Controller           |<==================>| Genesis main thread|
+----------+-----------+  UDP peer-to-peer  +---------+----------+
           ^                                          ||
           | ROS 2 / DDS                              || WebRTC
           |                                          ||
           |        Robot Jetson                      ||
           |        +-------------------+             ||
           +=======>| Robot             |             ||
             RGBD   | local I/O + safety|             ||
                    +-------------------+             ||
           <=========== observer + hand-eye WebRTC ====+
```

There is no Elesim Router process and no ZMQ transport. Controller, UI, Robot,
and Simulator are ROS 2 nodes that communicate directly through DDS over UDP.
DDS discovery finds peers; it is not an application registry or an authority.
Each participant must be mutually IP-routable with every participant it needs
to contact.

A public compute server normally runs Simulator and optional Coturn, while UI
and Controller remain on the laptop. This layout works only when the laptop
and server share a LAN, a routed VPN, or another network with bidirectional
reachability. Coturn can relay WebRTC media but cannot relay DDS discovery,
control/RGBD topics, or WebRTC signaling carried over DDS.

UI never imports controller workflow code. Controller never imports robot or
simulator packages. Robot does not know about model assets, IK, Pick, Genesis
or UI. UI's operator relationship with Controller remains separate from its
exclusive simulation session with Simulator.

## Ownership

| Release project | Owns | Does not own |
| --- | --- | --- |
| UI | presentation, operator intent, simulator view input, rendered-video receive | IK, workflow, hardware |
| Controller | Vision, Arm model, Look/Aim/Grasp, Gaze, target generation, one selected target lease | physical I/O, Genesis |
| Robot | Dynamixel/GO2 drivers, RGBD publishing, its motion lease, deadman, current limits | assets, builders, workflow |
| Simulator | Genesis runtime, model loading, virtual telemetry/RGBD, its motion lease and UI session, observer/hand-eye rendering and signaling | operator workflow, hardware |

Robot and Simulator are the only authorities for their own motion leases.
Simulator is the only authority for its UI session. DDS discovery does not
grant either authority, and `ROS_DOMAIN_ID` does not identify or authenticate
an owner.

## Dependency Rule

```text
{controller,ui,robot,simulator} -> elesim_interfaces + third-party packages
misc/tooling/model_builder -> misc/model/source + controller model schema
misc/tooling/release -> top-level release projects + ROS interface project
misc/tooling/setup -> deployment configuration and artifacts on disk
misc/infra/containers -> setup-generated isolated role image contexts
misc/infra/development -> setup-generated all-in-one coding environment only
misc/integration -> public ROS graph and media surfaces
```

A release project must not import a sibling project or a repository-root legacy
module. Communication between deployed processes is always a ROS interface or
a documented media stream.

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

## ROS Interface And Authority Invariants

- Wire contracts live in `packages/elesim_interfaces`; incompatible interface
  changes require an explicit schema/version decision.
- The Router-free wire contract is protocol major 5. A v4/ZMQ endpoint is not
  a compatible peer and must not be silently bridged into an authority path.
- A deployment uses one ROS-safe `system_id` and one unique logical endpoint
  ID. Every boot creates a new boot ID; only the advertised ROS resource
  prefixes, not the logical IDs themselves, must be valid ROS names.
- Every process publishes `EndpointDescriptor` and `EndpointHeartbeat`
  messages containing `PeerRef`, role, capabilities, stream descriptors and
  exact boot-specific topic/service prefixes. Duplicate live boots for one
  endpoint ID fail closed.
- Motion targets carry canonical four-element `q`, never hardware `u` values.
- A controller leases at most one robot or simulator endpoint.
- Robot or Simulator serializes and grants its own lease. A lease contains the
  target and Controller boot identities; a process restart invalidates
  it.
- A simulator grants at most one independent UI simulation session.
- Motion leases and simulation sessions are separate authorities: camera or
  pause input cannot grant arm-motion ownership.
- Switching, explicit release, renewal TTL expiry, or process restart revokes
  the previous lease/session.
- Robot and simulator reject stale sequences and mismatched leases.
- Estop bypasses the ordinary active-command path but remains role checked.
- RGBD is one time-coherent DDS sample. Observer and hand-eye pixels remain
  independent WebRTC streams.
- WebRTC offer/answer signaling is a Simulator-owned reliable DDS
  request/reply exchange on the direct control carrier.
  TURN affects only ICE media candidates; it cannot make the DDS signaling
  path reachable.

## Remote Simulator Semantics

Elesim does not transport the native Genesis desktop window. Simulator owns a
dedicated observer camera whose output is equivalent to the inspectable scene
view needed by an operator. UI receives that observer stream and the robot's
hand-eye preview as two independent WebRTC tracks. Mouse orbit, pan and zoom,
plus pause, resume, single-step, reset, speed, reset-view and debug-marker
commands are versioned DDS messages.

Commands enter a bounded mailbox and are applied only on the Genesis main
thread. Pausing stops physics, not endpoint heartbeats, status delivery or the
WebRTC sessions. Reset increments the simulation epoch; Controller stops an
active Pick/Gaze workflow when it observes a pause edge or epoch change.

## Network Security Profiles

The operator selects exactly one DDS security profile:

- `trusted-network` uses ordinary DDS with no DDS encryption. It is valid only
  on an owned LAN or routed VPN whose interface and firewall restrict
  participation to trusted hosts. Bind DDS to the selected interface and allow
  only the required peers or subnet. `ROS_DOMAIN_ID` reduces accidental graph
  collisions; it is not authentication, authorization, isolation, or
  encryption.
- `sros2` is required on an untrusted LAN, a shared compute network, or any
  network where other tenants can reach DDS. Each deployment receives its own
  keystore enclave and runs DDS Security in enforce mode for authentication,
  access control, and encryption. Permissions must restrict roles to the DDS
  topics they need.

There is no ZMQ, CurveZMQ, CURVE key, ZAP allowlist, or plaintext-ZMQ exception
in the final architecture. WebRTC media always uses DTLS/SRTP independently of
the DDS profile. Coturn relays those encrypted packets when ICE needs a relay.
For managed TURN, the REST HMAC secret is mounted only into Coturn and the
Simulator on the same managed host. Simulator issues short-lived credentials
bound to its active UI session; UI never receives the static secret. A
compromised Simulator can therefore mint TURN credentials, which is an explicit
managed-deployment trust boundary. External TURN uses a bounded JSON credential
file mounted only into Simulator; the active UI receives the usable credential
through the DDS session grant. Controller/UI-only hosts never receive that
file. Under `trusted-network` this inherits the controlled-LAN/VPN trust
assumption; use SROS2 on a shared or observable network.

The local installation GUI remains bound to loopback. Remote administration
uses SSH local forwarding, and its SSH port has no relationship to DDS or TURN.

## Verification Matrix

The canonical entry point runs this matrix with package-specific import paths:

```bash
python3 misc/tooling/quality/check.py --group required
```

The required gate covers ROS interfaces, all four release projects,
model/release/setup tooling, the four-process topology smoke, a real DDS RGBD
roundtrip, target-owned lease/session behavior, and actual encoded
observer/hand-eye WebRTC tracks. The extended
gate covers offline tools, readability budgets, and focused mutation checks:

```bash
python3 misc/tooling/quality/check.py --group extended
```

The equivalent individual commands are:

```bash
colcon test --packages-select elesim_interfaces
python3 -m pytest robot/tests
python3 -m pytest controller/tests
python3 -m pytest simulator/tests
python3 -m pytest ui/tests
PYTHONPATH=misc/tooling/model_builder/src:controller/src python3 -m pytest misc/tooling/model_builder/tests
PYTHONPATH=misc/tooling/setup/src python3 -m pytest misc/tooling/setup/tests
python3 misc/integration/smoke_topology.py
```

Release artifacts have a separate isolation gate. Building release contexts
builds the ROSIDL interface package, installs each transport-neutral
support/application wheel pair into a clean temporary target, loads deployment
configuration, validates the simulator bundle, checks that no sibling
deployment is visible, and invokes the packaged console entry point with
`--help`:

```bash
python3 misc/tooling/release/build.py
python3 misc/tooling/release/verify.py dist/releases
```

## Test Layers

- Contract tests pin ROS interfaces, payload, lease, safety, configuration, and
  role boundaries.
- Deterministic property tests exercise UV, LJI, equal-sag, ready-pose and
  reachable FK-to-IK invariants over broad generated inputs.
- Headless workflow tests execute Look -> Aim -> Grasp phase ordering without
  Genesis or camera windows.
- Recorded-log replay turns known field failures into deterministic regression
  reports.
- Transport integration tests use separate ROS 2 processes over a real DDS
  implementation and real aiortc sender/receiver pairs for both named video
  streams.
- Focused mutations prove that critical version, lease, stale-command,
  deadman, control-direction, gain, and finite-input guards are observed by the
  tests.
- Live Genesis and hardware-in-loop validation remains a manual gate because
  software-only tests cannot establish physical convergence or camera timing.
- Setup-tool tests generate trusted-network and SROS2 profiles, exercise safe
  bootstrap extraction, validate the generated DDS graph configuration, and
  validate DDS/STUN probes without importing a sibling deployment.

The checked-in `PeerEnvelope` carrier is the current protocol-v5 control and
signaling wire contract. The additional typed service/action definitions in
`packages/elesim_interfaces` are generated but are not yet bound by the runtime;
tests and documentation must not advertise those services as active.

Live release gates must cover discovery convergence, duplicate-ID fail-closed
behavior, lease expiry and command deadman timing, SROS2 permissions,
RGBD latency and bandwidth under loss, WebRTC SDP payload limits, routed-VPN
operation, and explicit failure on unsupported NAT-only layouts. Unit tests do
not prove any of those network properties.

Generate a role-specific line-execution report without adding a production
dependency:

```bash
python3 misc/tooling/quality/line_coverage.py controller
```

Use the development container when scientific or graphics dependencies are not
installed on the host.
