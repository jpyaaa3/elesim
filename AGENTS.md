# Elesim Maintenance Guide

## Current Work Handoff

- Updated: 2026-07-26
- Branch: `refactoring`; all Router-free ROS 2/DDS changes below are currently
  uncommitted.
- Goal: Replace every ZMQ transport and the Router deployment with direct ROS
  2/DDS UDP communication while retaining WebRTC for observer and hand-eye
  video.
- Phase: Implementation and software-only validation are complete. Real
  multi-host networking, SROS2 enforcement, NAT/TURN relay selection, GPU
  rendering, Jetson, and physical hardware behavior remain manual gates.
- Handoff boundary: do **not** restart or broaden the Router/ZMQ-to-DDS
  refactor. There is no remaining software implementation task unless a new
  request explicitly targets one of the manual gates or the not-yet-wired typed
  ROS service/action surface.
- Locked decisions:
  - General mode uses Docker for Simulator, Controller, and UI. Robot is
    native-only and selectable only on detected Jetson/JetPack hosts. There is
    no Router role.
  - Developer mode creates one privileged Ubuntu/WSL amd64 development
    container with the complete coding toolchain; Jaeger is optional.
  - The GUI binds to host loopback only. Remote use goes through SSH forwarding.
  - Installation generates files and Compose contexts but does not build or
    start images.
  - PATH registration uses an idempotent `.bashrc` block; the current parent
    shell still requires `source ~/.bashrc`.
  - Managed Coturn may join the Simulator Compose lifecycle, but it carries
    WebRTC media only and never DDS traffic.
  - External TURN credentials are a bounded JSON file mounted only into
    Simulator. Controller/UI-only installations receive no TURN private file.
  - DDS runtime configuration contains `system_id`, `domain_id`,
    `rmw_implementation`, discovery mode/static peers, bound interface,
    security profile, and optional SROS2 keystore/enclave.
  - `trusted-network` means no DDS encryption and is allowed only on an owned
    LAN or routed VPN restricted by interface and firewall. `sros2` is the
    profile for untrusted/shared networks.
  - `ROS_DOMAIN_ID` is not a security boundary.
  - Robot/Simulator own their motion leases; Simulator separately owns its UI
    simulation session. DDS discovery grants neither.
  - RGBD is a latest-only coherent DDS sample. WebRTC signaling is a
    Simulator-owned reliable DDS request/reply exchange; pixels remain
    DTLS/SRTP WebRTC.
- Implemented:
  - ROS contracts live in `packages/elesim_interfaces`; protocol major 5 uses a
    bounded `PeerEnvelope` DDS carrier for current control/signaling and a typed
    `RgbdFrame` for RGBD. The additional typed services/actions are generated
    but are not runtime-wired.
  - Router and its release were removed. Controller, UI, Simulator, and Robot
    use direct DDS peers; runtime code, configuration, and dependencies contain
    no ZMQ, CurveZMQ, CURVE, or ZAP surface.
  - A boot identity, lease/session epoch plus opaque token, and monotonic
    sequence prevent commands from a previous process or authority from being
    accepted.
  - Robot/Simulator own motion authority; Simulator separately owns the UI
    simulation session. A bounded startup queue retains traffic only until the
    exact source boot descriptor/heartbeat is known, and Controller target
    selection retries at the discovery interval until acknowledged.
  - Robot and Simulator publish coherent latest-only RGBD over DDS. Observer and
    hand-eye pixels remain WebRTC DTLS/SRTP, with signaling carried directly by
    DDS.
  - Robot safety remains local. Lease expiry and command deadman must stop
    motion without relying on DDS discovery callbacks.
  - Unitree ROS 2 integration must share a deliberate executor/context or use
    an explicit bridge when its domain differs from the Elesim domain.
  - Installer state schema v5 generates shared DDS graph settings, role-scoped
    SROS2 inputs, and managed/external TURN inputs. SSH remains setup-only and
    respects non-default ports.
  - Release generation emits exactly four application trees plus shared ROSIDL
    and protocol wheels; no Router tree is emitted.
- Existing installer invariants that remain:
  - `request.py`, `capabilities.py`, `service.py`, and `shell.py` keep one
    validated request boundary, outer-host detection, and atomic PATH
    registration.
  - `gui.py` and `web/` remain token-protected, loopback-published,
    Korean/English, size-limited, path-contained, and cooperatively cancellable.
  - `bootstrap.sh` preserves the invocation directory, selects a free loopback
    port, records outer-host GPU/display facts, and fail-closes rather than
    running a stale cached setup package.
  - Role-scoped SSH transfer still uses an agent or explicitly selected key,
    pins a user-confirmed host fingerprint, and must never copy a complete
    SROS2 authority/keystore to every host.
  - Coturn Compose variables are deliberately written as `$$...`, and its
    `/bin/sh -ec` command remains a one-element list containing the complete
    script. Do not collapse it to a scalar.
  - Specific GPU mode uses one Compose `device_ids` reservation and does not
    reapply the host index through in-container `CUDA_VISIBLE_DEVICES`.
  - The development environment remains one privileged all-project coding
    container with persistent home/venv, WSLg forwarding and optional Jaeger.
  - Release metadata must reject `UNKNOWN` and `0.0.0`; generated contexts must
    contain the complete setup web package and ROSIDL source.
- Verification:
  - Protocol: `82 passed`; setup: `132 passed`; focused Controller startup-race
    tests: `8 passed`.
  - In the `urop` ROS Humble environment, generated ROSIDL built successfully
    and the Router-free four-process CycloneDDS topology smoke passed with
    Controller, Robot, Simulator, and UI in separate OS processes.
  - The required package runs passed in that environment: Robot `69`,
    Controller `349`, Simulator `75`, UI `32`, model/release `28`, DDS RGBD `2`,
    and encoded two-stream WebRTC `1`.
  - The extended gate passed, including all eight focused critical mutations.
    Release tests passed `16`; four versioned release trees and wheels were
    generated with no Router.
  - A host-only standalone verification could not see NumPy after
    `PYTHONNOUSERSITE=1`, but the generated `urop` Developer environment has
    system NumPy 1.26.4 under the same isolation setting. There,
    `release/verify.py dist/releases` verified all four isolated releases.
- Last transport correction (keep this invariant): a real CycloneDDS smoke
  exposed an asymmetric startup-discovery race. A peer must not accept traffic
  until it has the exact source endpoint ID **and boot ID**, but it must also
  not silently lose a valid first request while that descriptor/heartbeat is
  still arriving. `DdsPeerNode` therefore holds at most 512 parsed inbound
  envelopes for one heartbeat timeout and releases them only after the exact
  source identity becomes live. Controller repeats `select_target` once per
  discovery interval until `target_selected`. Do not weaken source validation,
  turn the control QoS transient-local, or replace this with an unbounded queue.
- Current source-of-truth implementation points:
  - `packages/protocol/src/elesim_protocol/dds_transport.py`: DDS peer node,
    direct addressed carrier, discovery, source-boot startup queue, motion and
    simulation authority.
  - `packages/protocol/src/elesim_protocol/rgbd.py`: typed latest-only RGBD
    DDS publisher/subscriber; `packages/elesim_interfaces` owns ROSIDL types.
  - `simulator/src/elesim_simulator/turn.py`: managed/external TURN credential
    handling; WebRTC pixels remain outside DDS.
  - `misc/tooling/setup/src/elesim_setup/`: state schema v5, role-specific DDS
    generation, GUI, network doctor, and external TURN credential validation.
  - `misc/integration/smoke_topology.py`: the canonical four-process real-RMW
    topology smoke; it is not an NAT, GPU, WebRTC-media, or hardware proof.
- Test environment and exact successful commands:
  - The host shell deliberately lacks much of the scientific/ROS test stack;
    do not install it into host Python merely to make a test pass.
  - `urop` is the usable ROS Humble Developer container. Its workspace mount is
    `/home/user/ws` -> `/home/dev/ws`; this repository is
    `/home/dev/ws/elesim` inside it. It has system NumPy 1.26.4, including with
    `PYTHONNOUSERSITE=1`.
  - A temporary ROSIDL overlay was built at
    `/tmp/elesim-rosidl.VfHZXo/install` in `urop`. It is container-local and
    may disappear after recreation; rebuild/source a new overlay before running
    real DDS tests if that path no longer exists.
  - The last passing topology invocation was:

    ```bash
    docker exec urop bash -lc '
      source /opt/ros/humble/setup.bash
      source /tmp/elesim-rosidl.VfHZXo/install/setup.bash
      export PYTHONPATH=/home/dev/ws/elesim/packages/protocol/src:${PYTHONPATH:-}
      cd /home/dev/ws/elesim
      python3 misc/integration/smoke_topology.py
    '
    ```

  - The last passing isolated-release verification invocation was:

    ```bash
    docker exec urop bash -lc '
      source /opt/ros/humble/setup.bash
      source /tmp/elesim-rosidl.VfHZXo/install/setup.bash
      cd /home/dev/ws/elesim
      python3 misc/tooling/release/verify.py dist/releases
    '
    ```

  - `dist/releases/` contains four application trees (`controller`, `ui`,
    `robot`, `simulator`) and a separate `infra` tree. `infra` is not a fifth
    runtime application and must not be mistaken for a Router release.
- Operator-facing installation/run facts:
  - Bootstrap defaults to the local web wizard. Installation only writes the
    prefix/configuration/Compose contexts; it does not build or start images.
    After installation, `source ~/.bashrc` once, then use `elesim-up`,
    `elesim-status`, `elesim-logs`, and `elesim-down` on the machine owning the
    selected role.
  - SSH port forwarding such as `ssh -L 8765:127.0.0.1:8765 -p 2222 ...` is
    only for reaching the loopback-bound installer GUI. Port 2222 has no DDS or
    WebRTC runtime meaning.
  - All participating roles must share compatible DDS `system_id`, domain,
    RMW, discovery mode, interface, and security profile. Static peers seed
    DDS discovery only; neither they nor TURN traverse NAT for DDS.
  - For an owned LAN/routed VPN use `trusted-network` only with an explicit
    interface/firewall boundary. For shared or observable infrastructure use
    role-scoped SROS2 enforce mode. WebRTC remains DTLS/SRTP in both cases.
- Remaining manual validation:
  - Validate discovery and control/RGBD topic interoperability with the pinned RMW
    on one host, L2 LAN, routed LAN/static peers, routed VPN and global IPv6.
  - Confirm ordinary IPv4 NAT, CGNAT and symmetric NAT fail with an actionable
    diagnostic; TURN and static peers must not be presented as DDS NAT
    traversal.
  - Kill Controller/UI/target processes under loss and confirm lease expiry,
    session expiry, stale-sequence rejection and Robot stop deadlines.
  - Measure RGBD bandwidth, fragmentation, loss and p95 frame age;
    prove no subscriber backlog.
  - Validate SROS2 enforce-mode permissions and prove unauthorized
    publish/subscribe attempts are denied.
  - Validate both WebRTC streams, SDP size limits, atomic renegotiation and an
    actual Coturn relay candidate across NAT. This requires two appropriate
    hosts or networks plus ICE stats/Coturn logs; a local namespace simulation
    is not accepted as proof.
  - Launch the curl GUI locally and through a non-default SSH port; SSH
    forwarding remains installation access only.
  - Build/start generated Compose variants with NVIDIA Container Toolkit and
    verify CPU/GPU, WSLg/X11, observer, and hand-eye behavior.
  - Validate native Robot installation and safety behavior on the intended
    Jetson.
- Next commands in the generated Developer environment:
  - `python3 misc/tooling/quality/check.py --group required`
  - `python3 misc/tooling/quality/check.py --group extended`
  - `python3 misc/tooling/release/build.py`
  - `python3 misc/tooling/release/verify.py dist/releases`

Read `misc/docs/architecture.md` before changing behavior that crosses a process,
protocol, media, configuration, model, or deployment boundary. Read
`misc/docs/setup.md` before changing the installer and `misc/docs/deployment.md`
before changing release or multi-host behavior.

## Runtime Topology

Elesim has four independently deployable applications:

- `controller`: perception, IK, Pick, Gaze, Vision, and target generation
- `ui`: operator presentation, intent, remote video, and simulation controls
- `simulator`: Genesis, virtual telemetry, RGBD, observer and hand-eye rendering
- `robot`: physical I/O, device feedback, deadman, limits, and local safety

Applications communicate through ROS 2/DDS contracts in
`packages/elesim_interfaces`; observer and hand-eye pixels use WebRTC. The
current control/signaling contract is the bounded `PeerEnvelope` message.
Additional typed service/action definitions are not runtime-wired yet. There
is no ZMQ or central application Router. Source sharing in this monorepo is a
development convenience, not a runtime dependency.

## Dependency Rules

- A deployment must not import a sibling deployment.
- UI uses operator and simulation ROS interfaces, never controller workflow code.
- Robot must not know about IK, Pick, Genesis, builders, model source, or UI.
- Controller must not import Robot or Simulator implementations.
- Simulator consumes `model/bundles/default`; it rebuilds models only when
  `ELESIM_SIM_DEV_REBUILD=1` is explicitly set for development.
- Share ROS wire contracts through `packages/elesim_interfaces`; payload
  validation and transport primitives may live in `packages/protocol`.
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
- DDS participants must be mutually routable over UDP. Static discovery peers
  seed discovery but do not cross NAT or relay application traffic. Support
  LAN, routed VPN and global IPv6; do not claim ordinary NAT/CGNAT works.
- Bind DDS to the configured interface. All roles in one graph must agree on
  system ID, domain ID, RMW implementation, discovery mode and security profile.
- `inherit` GPU mode forwards `CUDA_VISIBLE_DEVICES`; `specific` persists one GPU
  index or UUID; `cpu` must disable both container GPU access and the Genesis GPU
  backend.
- The remote Simulator profile is headless but keeps observer and hand-eye render
  streams enabled. A native Genesis Viewer requires an explicit display/X11
  attachment and must not silently become the server default.
- Managed TURN selection may include Coturn in the Simulator's generated
  Compose project, so `elesim-up`, `elesim-down`, and `elesim-logs` own its
  lifecycle. External TURN remains independently operated. TURN does not carry
  DDS signaling or data. Managed TURN requires SROS2.
- Label multi-host commands with the machine that owns them. Do not tell users to
  create laptop configuration or destination directories on the compute server.
- `Ctrl+C` on `elesim-logs` stops log following, not the detached services.

## Security

- `trusted-network` has no DDS encryption. Permit it only on a controlled
  LAN/routed VPN, restrict the selected interface and firewall, and describe
  the trust assumption explicitly.
- Use `sros2` with enforce-mode DDS Security authentication, access control and
  encryption on an untrusted LAN or shared compute network. Give every
  deployment a role-scoped enclave.
- `ROS_DOMAIN_ID`, a namespace and an obscure multicast group are not
  authentication or tenant isolation.
- Never commit SROS2 private keys, TURN secrets, generated keystores, or copied
  remote host configuration. `misc/infra/generated/` is the source-workspace
  scratch area.
- WebRTC uses DTLS/SRTP. In managed mode, only Coturn and the co-located
  Simulator hold the static HMAC secret; Simulator issues short-lived
  session-bound credentials and UI receives no static secret. External TURN
  uses a private JSON credential file mounted only into Simulator; UI receives
  the usable value through the active DDS session grant.
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

Automated tests do not establish production-RMW discovery on a LAN/VPN,
SROS2 enforcement, loaded-network RGBD latency, Wi-Fi/VPN reconnect, real
Genesis GPU rendering, TURN relay selection across an actual NAT, WebRTC
latency, RealSense behavior, Dynamixel motion, or GO2 stability. Record those
as explicit manual validation results rather than presenting unit-test success
as hardware proof.

## Documentation

- `README.md` is the Korean operator guide and must match commands generated by the
  current installer.
- `misc/docs/architecture.md` owns system boundaries and ROS interface invariants.
- `misc/docs/setup.md` owns installer internals and network-doctor interpretation.
- `misc/docs/deployment.md` owns release and multi-host deployment detail.
- Update both `misc/docs/OPEN_ISSUES.md` and `misc/docs/OPEN_ISSUES_KR.md` when a
  newly discovered limitation remains unfixed.
