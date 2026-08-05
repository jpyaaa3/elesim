# Elesim Maintenance Guide

## Current Work Handoff

- Updated: 2026-08-05
- Branch: `refactoring`; all Router-free ROS 2/DDS changes below are currently
  uncommitted.
- Goal: Maintain the Router-free ROS 2/DDS architecture while keeping the
  bounded M1 operator-readiness layer and the explicit contract/connection/UI
  follow-up implemented below. Real network, security, GPU and hardware gates
  remain manual; do not confuse the software layer with those acceptance tests.
- Phase: Router-free transport, M2-A preflight, M2-B simulation-only topology,
  and M1 software/operator readiness are implemented and software-validated.
  Real multi-host networking, SROS2 enforcement, NAT/TURN relay selection, GPU
  rendering, Jetson, and physical hardware behavior remain manual gates.
- Handoff boundary: do **not** restart or broaden the Router/ZMQ-to-DDS
  refactor. M2-B is complete: the connection manager now has an explicit
  `full`/`simulation-only` topology mode, schema-v1/v2 compatibility, role-aware
  deployment and security generation, read-only Tailscale hinting, and explicit
  lifecycle status/actions. M1 is also complete. New protocol work must first
  update the DDS contract registry and tests; typed ROS service/action runtime
  wiring remains a separate follow-up.
- Locked decisions:
  - The deployable source trees and runtime role keys are canonically named
    `pilot/`, `sim/`, `ui/`, and `robot/`. Python packages, console commands,
    SROS2 enclaves, Compose service keys, endpoint identifiers and image tags
    use `pilot`/`sim` directly. There is no source-layout alias layer.
  - General mode uses Docker for Pilot, Sim, and UI. Robot is
    native-only and selectable only on detected Jetson/JetPack hosts. There is
    no Router role.
  - Developer mode creates one privileged Ubuntu/WSL amd64 development
    container named `elesim-dev` with the complete coding toolchain; Jaeger is
    the optional separate `elesim-jaeger` service. Do not depend on an external
    development Compose project.
  - General Compose uses the fixed project name `elesim-runtime`, images
    `elesim/<role>:local`, and containers `elesim-pilot`, `elesim-ui`, and
    `elesim-sim` for the selected roles. Pilot and Sim are the actual role
    keys and application names, not aliases. Robot remains native-only.
  - The GUI binds to host loopback only. Remote use goes through SSH forwarding.
  - Installation generates files and Compose contexts but does not build or
    start images.
  - PATH registration uses an idempotent `.bashrc` block; the current parent
    shell still requires `source ~/.bashrc`.
  - General installs default to optional local runtime text archives. Docker
    logs are bounded at 10 MiB x4; `elesim-logs --save` and `elesim-down`
    retain five private snapshots. Developer logging is unchanged.
  - Clean uninstall is host-only and ownership-manifest based. It validates an
    install UUID, exact wrapper/systemd hashes and Docker metadata/labels before
    mutation. Logs and operator Authority are preserved unless explicitly
    purged; external source/credentials/keystores are never owned.
  - The setup GUI has no Docker socket or deletion channel. It may validate a
    manifest and emit exact plan/execute commands, but only the host
    `elesim-uninstall` CLI mutates resources.
  - Managed Coturn may join the Sim Compose lifecycle, but it carries
    WebRTC media only and never DDS traffic.
  - External TURN credentials are a bounded JSON file mounted only into
    Sim. Pilot/UI-only installations receive no TURN private file.
  - DDS runtime configuration contains `system_id`, `domain_id`,
    `rmw_implementation`, discovery mode/static peers, bound interface,
    security profile, and optional SROS2 keystore/enclave.
  - `trusted-network` means no DDS encryption and is allowed only on an owned
    LAN or routed VPN restricted by interface and firewall. `sros2` is the
    profile for untrusted/shared networks.
  - `ROS_DOMAIN_ID` is not a security boundary.
  - Stock Unitree DDS is local/plaintext and confined to a private Jetson-GO2
    NIC/domain. `elesim-unitree-bridge` is the only Unitree participant;
    inter-host `elesim-robot` talks to it over bounded credential-checked Unix
    `SOCK_SEQPACKET` IPC. It is not a fifth application or a Router.
  - Connection topology schema v3 has an explicit `topology_mode`: `full` uses
    all four roles across 2..4 hosts, while `simulation-only` uses only
    Pilot/Sim/UI across 1..3 container/Compose hosts and contains no
    Robot or Jetson placeholder. Schema v1 is loaded as `full` and normalized.
  - Robot/Sim own their motion leases; Sim separately owns its UI
    simulation session. DDS discovery grants neither.
  - RGBD is a latest-only coherent DDS sample. WebRTC signaling is a
    Sim-owned reliable DDS request/reply exchange; pixels remain
    DTLS/SRTP WebRTC.
- Implemented:
  - ROS contracts live in `packages/elesim_interfaces`; protocol major 6 uses a
    bounded `PeerEnvelope` DDS carrier for current control/signaling and a typed
    `RgbdFrame` for RGBD. The additional typed services/actions are generated
    but are not runtime-wired.
  - `packages/protocol/src/elesim_protocol/contracts.py` and
    `docs/dds_contracts.md` enumerate every PeerEnvelope message, sender /
    receiver role, QoS, authority and payload policy. Empty lease renewals and
    acknowledgement/error surfaces are structurally validated against the v6
    wire shapes.
  - Router and its release were removed. Pilot, UI, Sim, and Robot
    use direct DDS peers; runtime code, configuration, and dependencies contain
    no ZMQ, CurveZMQ, CURVE, or ZAP surface.
  - A boot identity, lease/session epoch plus opaque token, and monotonic
    sequence prevent commands from a previous process or authority from being
    accepted.
  - Robot/Sim own motion authority; Sim separately owns the UI
    simulation session. A bounded startup queue retains traffic only until the
    exact source boot descriptor/heartbeat is known, and Pilot target
    selection retries at the discovery interval until acknowledged.
  - Robot and Sim publish coherent latest-only RGBD over DDS. Observer and
    hand-eye pixels remain WebRTC DTLS/SRTP, with signaling carried directly by
    DDS.
  - Robot safety remains local. Lease expiry and command deadman must stop
    motion without relying on DDS discovery callbacks.
  - Robot no longer imports or initializes Unitree ROS 2. A dedicated
    `elesim-unitree-bridge` daemon owns the private Unitree DDS context; Robot
    uses a 64 KiB-bounded JSON Unix packet protocol with `SO_PEERCRED`, boot
    IDs, monotonic sequences, command/parameter allowlists and deadman stop.
    GO2 IPC failure cannot skip arm safe-hold, torque-off or hardware cleanup.
  - Installer state schema v8 distinguishes externally supplied SROS2
    keystores from connection-manager-owned generation/bundle state, while
    retaining managed/external TURN inputs and optional runtime text logging.
    Migrations from v1-v7 disable new log retention. SSH remains setup-only and
    respects non-default ports.
  - `elesim-connections` owns the non-secret multi-host topology. DDS
    address/interface and SSH management host/port are separate fields; one is
    never inferred from the other.
  - The connection manager performs a read-only `tailscale0` address hint (no
    installation, login or ACL mutation) and exposes bounded `check`, `start`,
    `stop` and `restart` host-lifecycle jobs. These report Compose/systemd
    management state only; they do not claim DDS discovery or WebRTC media.
  - `elesim-connections` exposes the two topology modes above. In
    `simulation-only`, the GUI hides the fixed Robot card, allows one to three
    active COM cards, and serializes exactly one Pilot, Sim, and UI.
    Deployment, lifecycle rollback, and SROS2 policy generation use only those
    active roles; no `robot_id` is emitted for simulation-only rollback.
  - `TwoHostPreflight` and the loopback `/api/preflight` endpoint are ephemeral
    and role-neutral. They validate exactly two COM host DDS/SSH endpoints for
    the Jetson-less Tailscale test path, never save a topology, issue keys, or
    deploy roles. A saved `ConnectionTopology` uses explicit `full` or
    `simulation-only` invariants; only the former requires Robot and all four
    roles.
  - In managed SROS2 mode the operator laptop keeps the complete Authority.
    Runtime hosts receive common public material plus only their assigned role
    enclaves. Rotation is an all-host generation transaction with rollback;
    no host receives the CA private keys or unrelated role keys.
  - Release generation emits exactly four application trees plus shared ROSIDL
    and protocol wheels; no Router tree is emitted. The Robot wheel and release
    require both Robot and Unitree-bridge entrypoints plus exactly two systemd
    units.
  - The observer camera uses the pinned Genesis 1.2.0 trackball semantics
    (left orbit, middle pan, right/scroll zoom and ±89° pole clamp). The UI
    renders observer and hand-eye streams in one separate resizable native
    camera window; closing it hides it and the main panel can reopen it.
    Canonical Roll display direction is positive while raw Robot motor polarity
    remains independently configured.
  - Every completed install emits a stdlib-only host uninstaller and an exact
    ownership manifest. General/native use `<prefix>/install-ownership.json`;
    Developer keeps it inside `.elesim/development/` and never owns the source
    checkout.
- Existing installer invariants that remain:
  - `request.py`, `capabilities.py`, `service.py`, and `shell.py` keep one
    validated request boundary, outer-host detection, and atomic PATH
    registration.
  - `gui.py` and `web/` remain token-protected, loopback-published,
    Korean/English, size-limited, path-contained, and cooperatively cancellable.
  - `bootstrap.sh` preserves the invocation directory, selects a free loopback
    port, records outer-host GPU/display facts, and fail-closes rather than
    running a stale cached setup package.
  - Connection-manager SSH transfer uses an agent or explicitly selected key,
    pins a user-confirmed host fingerprint, and must never copy a complete
    SROS2 authority/keystore to every host.
  - Coturn Compose variables are deliberately written as `$$...`, and its
    `/bin/sh -ec` command remains a one-element list containing the complete
    script. Do not collapse it to a scalar.
  - Specific GPU mode uses one Compose `device_ids` reservation and does not
    reapply the host index through in-container `CUDA_VISIBLE_DEVICES`.
  - The development environment remains one persistent privileged all-project
    `elesim-dev` container with persistent home/venv, WSLg forwarding and
    optional separate Jaeger. `elesim-dev` uses Compose `exec`; it must not
    create random `run --rm` development containers.
  - Ownership refresh must fail closed when legacy generated paths exist
    without a manifest. Never auto-adopt them. Managed roots are exact
    Elesim-only subtrees, never the whole Developer checkout, home, or bin
    parent. Do not add prune, wildcard deletion, or upstream-image removal.
  - Runtime log archivers reject direct and ancestor symlinks. Archive failure
    must not prevent `elesim-down` from attempting shutdown, and must still
    produce a nonzero status.
  - Release metadata must reject `UNKNOWN` and `0.0.0`; generated contexts must
    contain the complete setup and connection-manager web packages and ROSIDL
    source.
- Verification:
  - Canonical required gate passed: Protocol `87`, Robot `98`, Pilot
    `350`, Sim `77`, UI `34`, model/release `34`, DDS RGBD `2`, encoded
    two-stream WebRTC `1`, and setup `315`.
  - Generated ROSIDL built successfully and the Router-free four-process
    CycloneDDS topology smoke passed with Pilot, Robot, Sim, and UI
    in separate OS processes.
  - The extended gate passed: quality `17`, analysis `6`, debug `4`, experiment
    `3`, and all eight focused critical mutations.
  - Release tests passed `22`; four versioned release trees and wheels were
    generated and isolated-verified with no Router. Release probes explicitly
    skip the development venv's editable `.pth` files so this isolation result
    is meaningful in the all-project developer environment.
  - Focused logging, uninstall ownership, Unitree IPC and Robot lifecycle
    regressions are included in those gates. Software tests do not prove
    journald/systemd account setup or GO2 stop timing on a real Jetson.
  - Actual ROS 2 Humble SROS2 CLI generation produced and verified role-scoped
    bundles for laptop, compute, and Robot hosts; this proves file generation
    and isolation, not live multi-host enforce-mode authorization.
  - A host-only standalone verification could not see NumPy after
    `PYTHONNOUSERSITE=1`; the generated `elesim-dev` image pins NumPy 1.26.4
    and is now the canonical isolated verification environment.
- Last transport correction (keep this invariant): a real CycloneDDS smoke
  exposed an asymmetric startup-discovery race. A peer must not accept traffic
  until it has the exact source endpoint ID **and boot ID**, but it must also
  not silently lose a valid first request while that descriptor/heartbeat is
  still arriving. `DdsPeerNode` therefore holds at most 512 parsed inbound
  envelopes for one heartbeat timeout and releases them only after the exact
  source identity becomes live. Pilot repeats `select_target` once per
  discovery interval until `target_selected`. Do not weaken source validation,
  turn the control QoS transient-local, or replace this with an unbounded queue.
- Current source-of-truth implementation points:
  - `packages/protocol/src/elesim_protocol/dds_transport.py`: DDS peer node,
    direct addressed carrier, discovery, source-boot startup queue, motion and
    simulation authority.
  - `packages/protocol/src/elesim_protocol/rgbd.py`: typed latest-only RGBD
    DDS publisher/subscriber; `packages/elesim_interfaces` owns ROSIDL types.
  - `sim/src/elesim_sim/turn.py`: managed/external TURN credential
    handling; WebRTC pixels remain outside DDS.
  - `robot/src/elesim_robot/go2/unitree_ipc*.py` and
    `unitree_bridge_daemon.py`: local bounded UDS boundary, peer credentials,
    replay fencing and bridge-side GO2 deadman stop.
  - `installer/package/src/elesim_setup/`: state schema v8, role-specific DDS
    generation, connection topology/GUI, SROS2 Authority generation and
    transactional deployment, network doctor, TURN credential validation, and
    the ephemeral `TwoHostPreflight` contract/API. `connection_manager.py` owns
    topology schema v3 and its `full`/`simulation-only` invariants;
    `security_policy.py` and `secure_deployment.py` filter SROS2/lifecycle
    operations to the active role set.
    Runtime role keys and source trees are the same names (`pilot` and `sim`).
    Old `controller`/`simulator` values are accepted only while reading legacy
    state/topology files; old container names are inspected only for cleanup.
  - `ownership.py` and `uninstall.py`: exact install manifests, host-only
    pre-mutation validation, preserve-by-default cleanup and tombstones.
  - `misc/system_tests/smoke_topology.py`: the canonical four-process real-RMW
    topology smoke; it is not an NAT, GPU, WebRTC-media, or hardware proof.
- Canonical test environment and commands:
  - The host shell deliberately lacks much of the scientific/ROS test stack;
    do not install it into host Python merely to make a test pass.
  - Use the setup-generated persistent `elesim-dev` container. Its entrypoint
    builds a persistent ROSIDL overlay, creates the system-site-packages venv,
    and installs every project editable. Do not add dependencies to host Python
    or reference an external Compose file.
  - Start or enter it with `elesim-up` and `elesim-dev`. The topology invocation
    is:

    ```bash
    elesim-dev python3 misc/system_tests/smoke_topology.py
    ```

  - The isolated-release verification invocation is:

    ```bash
    elesim-dev python3 misc/tools/release/verify.py dist/releases
    ```

  - `dist/releases/` contains four application trees (`pilot`, `ui`,
    `robot`, `sim`) and a separate `infra` tree. `infra` is not a fifth
    runtime application and must not be mistaken for a Router release.
- Operator-facing installation/run facts:
  - Bootstrap defaults to the local web wizard. Installation only writes the
    prefix/configuration/Compose contexts; it does not build or start images.
    After installation, `source ~/.bashrc` once, then use `elesim-up`,
    `elesim-setup status`, `elesim-logs`, `elesim-logs --save`, and
    `elesim-down` on the machine owning the selected role.
  - General installations expose fixed role container names; Developer
    installations expose only `elesim-dev` and optional `elesim-jaeger`.
    Managed TURN adds `elesim-coturn` on the Sim host.
  - Use `elesim-uninstall --plan`, then retype the exact prefix with
    `--confirm-prefix`. Logs and operator Authority remain unless their purge
    flags are supplied. On Robot, remove the exact two installed systemd units
    using the refusal message before rerunning the plan.
  - Native Robot setup generates `elesim-robot.service` plus
    `elesim-unitree-bridge.service`; it prints but never executes account/group,
    ACL, unit registration or service-start commands. Defaults for the private
    Unitree link are `eth0`, domain `1`, and `$HOME/ros2_ws`; bootstrap accepts
    explicit `ELESIM_UNITREE_*` overrides.
  - Use `elesim-connections` on the operator laptop to select `full` (all four
    roles, 2..4 hosts) or `simulation-only` (Pilot/Sim/UI, 1..3
    hosts), validate independent DDS/SSH endpoints, provision or rotate managed
    SROS2 bundles, and deploy them transactionally.
  - Before Jetson is available, use the GUI's `두 호스트 점검`/`Two-host endpoint
    preflight` with exactly two active COM cards. DDS addresses are mutable
    hostname/IP values without ports; `tailscale0` is an interface example.
    SSH uses the remote sshd port (normally 22); an HTTP test on 8080 is not an
    Elesim endpoint and is not saved.
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
  - Kill Pilot/UI/target processes under loss and confirm lease expiry,
    session expiry, stale-sequence rejection and Robot stop deadlines.
  - Measure RGBD bandwidth, fragmentation, loss and p95 frame age;
    prove no subscriber backlog.
  - Validate SROS2 enforce-mode permissions and prove unauthorized
    publish/subscribe attempts are denied.
  - On real 2..4-host topology validate `elesim-connections` SSH fingerprint
    pinning, non-default ports, full managed generation activation, one-host
    failure rollback, and revocation of the replaced generation.
  - Validate both WebRTC streams, SDP size limits, atomic renegotiation and an
    actual Coturn relay candidate across NAT. This requires two appropriate
    hosts or networks plus ICE stats/Coturn logs; a local namespace simulation
    is not accepted as proof.
  - Launch the curl GUI locally and through a non-default SSH port; SSH
    forwarding remains installation access only.
  - Build/start generated Compose variants with NVIDIA Container Toolkit and
    verify CPU/GPU, WSLg/X11, observer, and hand-eye behavior.
  - Validate native Robot installation on the intended Jetson: dedicated
    accounts/group/ACLs, two-unit BindsTo/PartOf lifecycle, private Unitree
    NIC/domain confinement, UDS peer credentials, bridge loss/malformed packet
    stop deadlines, arm cleanup despite IPC failure, and physical safety.
- Next commands in the generated Developer environment:
  - `elesim-dev python3 misc/tools/quality/check.py --group required`
  - `elesim-dev python3 misc/tools/quality/check.py --group extended`
  - `elesim-dev python3 misc/tools/release/build.py`
  - `elesim-dev python3 misc/tools/release/verify.py dist/releases`

Read `docs/architecture.md` before changing behavior that crosses a process,
protocol, media, configuration, model, or deployment boundary. Read
`docs/setup.md` before changing the installer and `docs/deployment.md`
before changing release or multi-host behavior.

## Runtime Topology

Elesim has four independently deployable applications:

- `pilot`: perception, IK, Pick, Gaze, Vision, and target generation
- `ui`: operator presentation, intent, remote video, and simulation controls
- `sim`: Genesis, virtual telemetry, RGBD, observer and hand-eye rendering
- `robot`: physical I/O, device feedback, deadman, limits, and local safety

Applications communicate through ROS 2/DDS contracts in
`packages/elesim_interfaces`; observer and hand-eye pixels use WebRTC. The
current control/signaling contract is the bounded `PeerEnvelope` message.
Additional typed service/action definitions are not runtime-wired yet. There
is no ZMQ or central application Router. Source sharing in this monorepo is a
development convenience, not a runtime dependency. `elesim-unitree-bridge` is
a Robot-host-local hardware adapter over UDS, not an independently deployable
fifth application and not part of inter-host DDS.

## Dependency Rules

- A deployment must not import a sibling deployment.
- UI uses operator and simulation ROS interfaces, never pilot workflow code.
- Robot must not know about IK, Pick, Genesis, builders, model source, or UI.
- Pilot must not import Robot or Sim implementations.
- Sim consumes `model/bundles/default`; it rebuilds models only when
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
  as `src/elesim_sim/config/` is Python application code. Never filter files
  recursively by basename when preparing build contexts.
- The Pilot arm model and Sim model bundle are immutable runtime inputs.
  Regenerate them with `model/builder`, not inside a runtime process.
- Installer output, release contexts, and wheels are products that require their
  own isolation checks. A source-tree import test is not a substitute for checking
  the generated context or installed wheel.
- Preserve unrelated local changes, experiment evidence, and generated diagnostic
  output unless the task explicitly owns them.

## Installation And Operations

- The setup wizard must preserve existing host Python, CUDA, ROS, and APT state
  when container mode is selected.
- General and Developer Compose projects use the fixed names `elesim-runtime`
  and `elesim-runtime-dev` with predictable container/image names and host
  networking. Do not assume `127.0.0.1` refers to another computer.
- DDS participants must be mutually routable over UDP. Static discovery peers
  seed discovery but do not cross NAT or relay application traffic. Support
  LAN, routed VPN and global IPv6; do not claim ordinary NAT/CGNAT works.
- Bind DDS to the configured interface. All roles in one graph must agree on
  system ID, domain ID, RMW implementation, discovery mode and security profile.
- Keep the connection manager's DDS endpoint separate from its optional SSH
  management endpoint. SSH port forwarding never supplies a DDS locator.
- `inherit` GPU mode forwards `CUDA_VISIBLE_DEVICES`; `specific` persists one GPU
  index or UUID; `cpu` must disable both container GPU access and the Genesis GPU
  backend.
- The remote Sim profile is headless but keeps observer and hand-eye render
  streams enabled. A native Genesis Viewer requires an explicit display/X11
  attachment and must not silently become the server default.
- Managed TURN selection may include Coturn in the Sim's generated
  Compose project, so `elesim-up`, `elesim-down`, and `elesim-logs` own its
  lifecycle. External TURN remains independently operated. TURN does not carry
  DDS signaling or data. Managed TURN requires SROS2.
- Label multi-host commands with the machine that owns them. Do not tell users to
  create laptop configuration or destination directories on the compute server.
- `Ctrl+C` on `elesim-logs` stops log following, not the detached services.
- Preserve the ownership manifest and installed `elesim-uninstall` wrapper
  until cleanup completes. Never replace them with raw recursive deletion or
  Docker prune instructions.

## Security

- `trusted-network` has no DDS encryption. Permit it only on a controlled
  LAN/routed VPN, restrict the selected interface and firewall, and describe
  the trust assumption explicitly.
- Use `sros2` with enforce-mode DDS Security authentication, access control and
  encryption on an untrusted LAN or shared compute network. Give every
  deployment a role-scoped enclave.
- State v7 `external` provisioning consumes an operator-supplied local
  keystore/enclave. `managed` provisioning records a connection-manager
  generation and host bundle; do not silently convert between the two.
- Keep the managed SROS2 Authority and CA private keys on the operator laptop.
  A host bundle contains common public material and only that host's assigned
  enclaves. Rotation must stage every host, stop roles, atomically activate one
  generation, restart/verify, and roll all hosts back on partial failure.
- `ROS_DOMAIN_ID`, a namespace and an obscure multicast group are not
  authentication or tenant isolation.
- Never commit SROS2 private keys, TURN secrets, generated keystores, or copied
  remote host configuration. `environment/generated/` is the source-workspace
  scratch area.
- WebRTC uses DTLS/SRTP. In managed mode, only Coturn and the co-located
  Sim hold the static HMAC secret; Sim issues short-lived
  session-bound credentials and UI receives no static secret. External TURN
  uses a private JSON credential file mounted only into Sim; UI receives
  the usable value through the active DDS session grant.
- SSH/`scp` is only a credential transfer mechanism and is not part of Elesim media
  or control transport. Respect non-default SSH ports.
- Unitree DDS remains plaintext only on the private Jetson-GO2 NIC/domain. The
  bridge receives no Elesim enclave or CA material; do not widen SROS2 policy
  for Unitree topics or expose that participant on Tailscale/shared LAN.
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
python3 misc/tools/quality/check.py --group required
```

For structural, installer, protocol, or release changes also run:

```bash
python3 misc/tools/quality/check.py --group extended
python3 misc/tools/release/build.py
python3 misc/tools/release/verify.py dist/releases
```

The detailed per-package matrix is in `docs/architecture.md`. At minimum,
changes must test the owning package. Cross-process changes also require
`misc/system_tests/smoke_topology.py`. Installer copy/filter changes must assert the
contents of generated contexts and built wheels, including nested Python packages.

Automated tests do not establish production-RMW discovery on a LAN/VPN,
SROS2 enforcement, loaded-network RGBD latency, Wi-Fi/VPN reconnect, real
Genesis GPU rendering, TURN relay selection across an actual NAT, WebRTC
latency, RealSense behavior, Dynamixel motion, or GO2 stability. Record those
as explicit manual validation results rather than presenting unit-test success
as hardware proof.

## Documentation

- Repository auxiliary ownership is explicit: `environment/` contains execution
  environment inputs, `installer/` contains bootstrap/setup sources,
  `misc/system_tests/` contains cross-process validation, `model/` owns model source
  and builders, `misc/research/` owns offline work, and `misc/tools/` owns developer/CI
  helpers. No legacy compatibility source tree remains.
- `README.md` is the Korean operator guide and must match commands generated by the
  current installer.
- `docs/architecture.md` owns system boundaries and ROS interface invariants.
- `docs/setup.md` owns installer internals and network-doctor interpretation.
- `docs/deployment.md` owns release and multi-host deployment detail.
- Update both `docs/OPEN_ISSUES.md` and `docs/OPEN_ISSUES_KR.md` when a
  newly discovered limitation remains unfixed.
