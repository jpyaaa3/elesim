# Elesim Architecture

Elesim is a monorepo of independently built release projects. Source sharing is a
development convenience, not a runtime dependency.

## Runtime Topology

```text
Laptop                                      Compute PC
+----------------------+    ROS 2 / DDS     +--------------------+
| UI                   |<==================>| Sim                 |
| Pilot                |<==================>| Genesis main thread |
+----------+-----------+  UDP peer-to-peer  +---------+----------+
           ^                                          ||
           | ROS 2 / DDS                              || WebRTC
           |                                          ||
           |        Robot Jetson                      ||
           |        +-------------+  UDS  +---------+
           +=======>| Robot       |<=====>| Unitree |----> private GO2 DDS/NIC
             RGBD   | I/O + safety|       | bridge  |
                    +-------------+       +---------+
           <=========== observer + hand-eye WebRTC ====+
```

There is no Elesim Router process and no ZMQ transport. Pilot, UI, Robot,
and Sim are ROS 2 nodes that communicate directly through DDS over UDP.
DDS discovery finds peers; it is not an application registry or an authority.
Each participant must be mutually IP-routable with every participant it needs
to contact.

A public compute server normally runs Sim and optional Coturn, while UI
and Pilot remain on the laptop. This layout works only when the laptop
and server share a LAN, a routed VPN, or another network with bidirectional
reachability. Coturn can relay WebRTC media but cannot relay DDS discovery,
control/RGBD topics, or WebRTC signaling carried over DDS.

UI never imports pilot workflow code. Pilot never imports robot or
sim packages. Robot does not know about model assets, IK, Pick, Genesis
or UI. UI's operator relationship with Pilot remains separate from its
exclusive simulation session with Sim.

## Ownership

| Release project | Owns | Does not own |
| --- | --- | --- |
| UI | presentation, operator intent, sim view input, rendered-video receive | IK, workflow, hardware |
| Pilot | Vision, Arm model, Look/Aim/Grasp, Gaze, target generation, one selected target lease | physical I/O, Genesis |
| Robot | Dynamixel/GO2 drivers, RGBD publishing, its motion lease, deadman, current limits | assets, builders, workflow |
| Sim | Genesis runtime, model loading, virtual telemetry/RGBD, its motion lease and UI session, observer/hand-eye rendering and signaling | operator workflow, hardware |

Robot and Sim are the only authorities for their own motion leases.
Sim is the only authority for its UI session. DDS discovery does not
grant either authority, and `ROS_DOMAIN_ID` does not identify or authenticate
an owner.

### Local Unitree boundary

Stock Unitree DDS is not part of the inter-host Elesim graph. On Jetson, the
`elesim-unitree-bridge` daemon is the only process that loads Unitree ROS 2 and
binds CycloneDDS to the private Jetson-to-GO2 NIC/domain. It runs without the
Elesim SROS2 environment. The Robot application remains the only inter-host
participant and communicates with the bridge through bounded Unix
`SOCK_SEQPACKET` messages.

The socket directory is `0750`, the socket is `0660`, and Robot receives only
the bridge group as a supplementary group. Both sides verify `SO_PEERCRED`, a
UUID boot identity and monotonic sequence. Command names and finite parameter
ranges are allowlisted. Disconnect, parse failure, stale/replayed traffic and
keepalive expiry trigger the bridge-side stop; GO2 failure must not prevent the
Robot process from continuing arm safe-hold, torque-off and hardware cleanup.
This daemon is a local hardware adapter, not a fifth deployable application and
not a Router.

## Dependency Rule

```text
{pilot,ui,robot,sim} -> elesim_interfaces + third-party packages
model/builder -> model/source + pilot model schema
misc/tools/release -> top-level release projects + ROS interface project
installer/package -> environment configuration and artifacts on disk
environment/containers -> setup-generated isolated role image contexts
environment/development -> setup-generated all-in-one coding environment only
misc/system_tests -> cross-process DDS/RGBD/WebRTC validation only
```

A release project must not import a sibling project or a repository-root legacy
module. Communication between deployed processes is always a ROS interface or
a documented media stream.

The developer container deliberately co-locates all source projects for coding
and tests, but it does not weaken release ownership: no deployment wheel or
general-user role image may import a sibling deployment.

General installations use one fixed `elesim-runtime` Compose project. Selected
container roles are named `elesim-pilot`, `elesim-ui`, and
`elesim-sim`; Robot stays a native Jetson service. Developer installation
uses the separate fixed `elesim-runtime-dev` project with one persistent
`elesim-dev` container and optional `elesim-jaeger`. It does not also create the
three general-role containers. Managed WebRTC relay adds `elesim-coturn` only
to the Sim host's general project.

## Model Lifecycle

`model/source` is builder input. `model/builder` creates immutable
artifacts under `model/bundles`. The sim consumes a prebuilt bundle by
default. Runtime rebuilding is a development-only operation enabled explicitly
with `ELESIM_SIM_DEV_REBUILD=1`.

The pilot likewise reads `config/arm_model.json` and never constructs an
assembly at runtime. The installed model-builder commands regenerate both
artifacts offline; arm-model intermediate files live in a temporary workspace:

```bash
elesim-build-sim-bundle --assets model/source/assets --output model/bundles/default
elesim-build-arm-model --config pilot/config/config.pc.yaml \
  --assets model/source/assets --output pilot/config/arm_model.json
```

## ROS Interface And Authority Invariants

The complete current `PeerEnvelope` registry, sender/receiver matrix, QoS
classes, and payload-validation rules are maintained in
[`dds_contracts.md`](dds_contracts.md). Treat that registry as the process
boundary: applications exchange envelopes and typed ROSIDL samples, never
implementation methods or sibling deployment imports.

- Wire contracts live in `packages/elesim_interfaces`; incompatible interface
  changes require an explicit schema/version decision.
- The Router-free wire contract is protocol major 6. A v4/ZMQ endpoint is not
  a compatible peer and must not be silently bridged into an authority path.
- A deployment uses one ROS-safe `system_id` and one unique logical endpoint
  ID. Every boot creates a new boot ID; only the advertised ROS resource
  prefixes, not the logical IDs themselves, must be valid ROS names.
- Every process publishes `EndpointDescriptor` and `EndpointHeartbeat`
  messages containing `PeerRef`, role, capabilities, stream descriptors and
  exact boot-specific topic/service prefixes. Duplicate live boots for one
  endpoint ID fail closed.
- Motion targets carry canonical four-element `q`, never hardware `u` values.
- A pilot leases at most one robot or sim endpoint.
- Robot or Sim serializes and grants its own lease. A lease contains the
  target and Pilot boot identities; a process restart invalidates
  it.
- A sim grants at most one independent UI simulation session.
- Motion leases and simulation sessions are separate authorities: camera or
  pause input cannot grant arm-motion ownership.
- Switching, explicit release, renewal TTL expiry, or process restart revokes
  the previous lease/session.
- Robot and sim reject stale sequences and mismatched leases.
- Estop bypasses the ordinary active-command path but remains role checked.
- RGBD is one time-coherent DDS sample. Observer and hand-eye pixels remain
  independent WebRTC streams.
- WebRTC offer/answer signaling is a Sim-owned reliable DDS
  request/reply exchange on the direct control carrier.
  TURN affects only ICE media candidates; it cannot make the DDS signaling
  path reachable.

## Remote Sim Semantics

Elesim does not transport the native Genesis desktop window. Sim owns a
dedicated observer camera whose output is equivalent to the inspectable scene
view needed by an operator. UI receives that observer stream and the robot's
hand-eye preview as two independent WebRTC tracks. Mouse orbit, pan and zoom,
plus pause, resume, single-step, reset, speed, reset-view and debug-marker
commands are versioned DDS messages.

Commands enter a bounded mailbox and are applied only on the Genesis main
thread. Pausing stops physics, not endpoint heartbeats, status delivery or the
WebRTC sessions. Reset increments the simulation epoch; Pilot stops an
active Pick/Gaze workflow when it observes a pause edge or epoch change.
The UI also bounds the combined queued/in-flight command backlog and reports a
full backlog instead of silently accepting input while Sim acknowledgements
are unavailable. Successful simulation results release their transport
bookkeeping, so a lost or slow session cannot grow UI memory without bound.
Sim treats a transient loss of the Pilot or UI peer as a retryable DDS condition:
telemetry and status remain dirty until acknowledged by the transport, and
unsent simulation results return to the bounded mailbox. Reply failures and
heartbeat/receive failures are diagnosed and retried without terminating the
Genesis process's DDS thread.
Robot treats a transient DDS transport failure as a safety event rather than a
process-exit condition: it revokes the local motion lease (running the arm
safe-hold path), keeps the local deadman and hardware monitor ticking, emits a
rate-limited diagnostic, and retries discovery. Cleanup or hardware-stop
failures remain fatal so a failed safety action cannot be hidden.
Each WebRTC video track likewise converts a transient frame-provider or frame
conversion failure into a bounded diagnostic and a black fallback frame; the
aiortc track stays alive so a later camera frame can recover the stream.
The UI simulation session treats a DDS transport reset as a lease loss even
before the peer directory reports an unregistered node: it closes stale media
receivers, discards commands from the old session, and reopens only after a
fresh Sim descriptor is discovered.
The UI operator request pump also marks DDS offline immediately on a heartbeat
reset, so the presentation layer cannot report a stale Pilot connection while
requests are waiting for their normal bounded timeout.

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

The SROS2 policy distinguishes three names that must not be conflated. Direct
control/motion carrier topics use the protocol's collision-resistant hashed
peer key. Enclave paths and configured RGBD topics use the stable endpoint key
shown to the operator. The local active network doctor currently reuses the
first installed role identity rather than receiving a super-user enclave. To
allow that diagnostic on any role host, every role policy has read-only access
to the Robot and Sim RGBD topics; it receives no additional RGBD publish
permission. This is an explicit operational tradeoff: role credentials are not
isolated from RGBD observation.

State schema v8 distinguishes two SROS2 provisioning models. `external` points
at a keystore/enclave supplied and maintained outside Elesim. `managed` records
an Elesim security generation and the local host's role bundle. In managed mode
the operator laptop holds the complete SROS2 Authority. A runtime host receives
the shared public trust material and only the enclaves for roles assigned to
that host; it never receives CA private keys or another host's role keys.

Regardless of provisioning owner, each application sees only the stable
`<prefix>/security/roles/<role>` keystore root. External setup copies common
public material plus that role's enclave into this view; it never mounts the
operator's aggregate keystore into an application container. Managed activation
refreshes only the `public/` and `enclaves/` children while services are stopped,
so the mounted role-root inode remains stable. The aggregate generation tree is
an administrative connection-manager input, not an application mount.

Managed rotation is deliberately system-wide. The connection manager creates a
new generation through the ROS 2 security CLI, validates SHA-256 bundle
manifests, stages every host through authenticated SSH/SFTP, stops affected
roles, atomically activates the generation, restarts and verifies them. Any
partial failure restores the previous generation across the hosts already
touched. The Authority is an administrative asset, not a fifth runtime service
and not a peer-discovery broker.

There is no ZMQ, CurveZMQ, CURVE key, ZAP allowlist, or plaintext-ZMQ exception
in the final architecture. WebRTC media always uses DTLS/SRTP independently of
the DDS profile. Coturn relays those encrypted packets when ICE needs a relay.
For managed TURN, the REST HMAC secret is mounted only into Coturn and the
Sim on the same managed host. Sim issues short-lived credentials
bound to its active UI session; UI never receives the static secret. A
compromised Sim can therefore mint TURN credentials, which is an explicit
managed-deployment trust boundary. External TURN uses a bounded JSON credential
file mounted only into Sim; the active UI receives the usable credential
through the DDS session grant. Pilot/UI-only hosts never receive that
file. Under `trusted-network` this inherits the controlled-LAN/VPN trust
assumption; use SROS2 on a shared or observable network.

The local installation GUI remains bound to loopback. Remote administration
uses SSH local forwarding, and its SSH port has no relationship to DDS or TURN.

## Connection Topology Ownership

`elesim-connections` runs on the operator laptop and persists only non-secret
topology. Schema v2 records an explicit `topology_mode`:

- `full` assigns Pilot, UI, Sim, and Robot exactly once across two to
  four active hosts; Robot remains constrained to a native Jetson host.
- `simulation-only` assigns Pilot, UI, and Sim exactly once across
  one to three container/Compose hosts and contains no Robot/Jetson placeholder.

Both modes mark exactly one host local and allow a host to own multiple roles.
Schema-v1 documents load as `full` and are normalized on save.

Every host has a DDS address and interface used for runtime UDP. A remote host
separately has an SSH hostname, port, user, authentication mode (`openssh` via
agent/key or `tailscale` via Tailscale SSH), and pinned SHA-256 host-key
fingerprint for administration. Tailscale SSH is keyless and uses port 22;
Tailscale ACL `check` rules may require an interactive re-authentication before
the manager can automate commands. The values may happen to name the same
machine, but no DDS locator is derived from SSH and an SSH port such as `2222`
is never a DDS or WebRTC port. Static peers are derived from the active hosts'
DDS addresses only.

## Verification Matrix

The canonical entry point runs this matrix with package-specific import paths:

```bash
python3 misc/tools/quality/check.py --group required
```

The required gate covers ROS interfaces, all four release projects,
model/release/setup tooling, the four-process topology smoke, a real DDS RGBD
roundtrip, target-owned lease/session behavior, and actual encoded
observer/hand-eye WebRTC tracks. The extended
gate covers offline tools, readability budgets, and focused mutation checks:

```bash
python3 misc/tools/quality/check.py --group extended
```

The equivalent individual commands are:

```bash
colcon test --packages-select elesim_interfaces
python3 -m pytest robot/tests
python3 -m pytest pilot/tests
python3 -m pytest sim/tests
python3 -m pytest ui/tests
PYTHONPATH=model/builder/src:pilot/src python3 -m pytest model/builder/tests
PYTHONPATH=installer/package/src python3 -m pytest installer/package/tests
python3 misc/system_tests/smoke_topology.py
```

Release artifacts have a separate isolation gate. Building release contexts
builds the ROSIDL interface package, installs each transport-neutral
support/application wheel pair into a clean temporary target, loads deployment
configuration, validates the sim bundle, checks that no sibling
deployment is visible, and invokes each role's primary packaged console entry
point with `--help`. Robot verification additionally requires both Robot and
Unitree-bridge console-script metadata, the bridge/IPC modules, and exactly the
two systemd units:

```bash
python3 misc/tools/release/build.py
python3 misc/tools/release/verify.py dist/releases
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

The checked-in `PeerEnvelope` carrier is the current protocol-v6 control and
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
python3 misc/tools/quality/line_coverage.py pilot
```

Use the setup-generated environment instead of installing scientific or ROS
dependencies on the host:

```bash
elesim-dev python3 misc/tools/quality/check.py --group required
elesim-dev python3 misc/tools/quality/check.py --group extended
```
