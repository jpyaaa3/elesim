# Setup Wizard and Network Doctor

## Scope

`installer/package` owns installation request validation, generated runtime
configuration, role-isolated build contexts, shell wrappers, credential
provisioning, non-secret multi-host topology, transactional SROS2 rollout, and
post-install network diagnosis. It does not own a deployment's domain behavior
and imports no Pilot, UI, Sim, or Robot implementation.

The browser wizard offers two editions:

- **General** installs a selected subset of the four runtime applications.
  Sim, Pilot, and UI use role-isolated Docker images. Robot is a
  native-only, Jetson-detected, exclusive selection. Router is not a role.
- **Developer** prepares one complete Git workspace and one privileged
  Ubuntu/ROS2 development image. It includes all applications, model tooling,
  tests, graphics/scientific dependencies, and optional Jaeger.

The existing terminal `wizard` and non-interactive `install` subcommands remain
available for automation and compatibility. They use the same state and
container generators, but only the browser path exposes the edition and
developer environment as one guided flow. Post-install multi-host topology,
SSH preflight and managed SROS2 rollout belong to the separate
`elesim-connections` browser UI.

## Public Bootstrap

On a clean Ubuntu or WSL host:

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/installer/bootstrap/bootstrap.sh | bash
```

The shell bootstrap:

1. Checks Docker Engine and Compose v2. If Docker is absent on Ubuntu, it asks
   before installing Docker packages; declining leaves the host unchanged.
2. Records host-only facts that would otherwise disappear inside the setup
   container: OS/architecture, Jetson, WSL/WSLg, display availability,
   `nvidia-smi -L`, invocation directory, user, and SSH agent socket.
3. Downloads the standard-library `bootstrap.py` to a temporary file in the
   setup cache and atomically publishes the complete download.
4. Runs it as the calling UID/GID in a disposable `python:3.10-slim`
   container. The container receives the user's home and invocation directory,
   but never the Docker socket.
5. Downloads and safely extracts the requested GitHub source archive, creates a
   cached setup venv, and starts `elesim-setup gui`.
6. Publishes the GUI on host loopback only. Port `8765` is preferred; the
   bootstrap searches the next 99 ports when it is occupied.

The GUI URL carries a random session token once. JavaScript moves it into
`sessionStorage` and removes it from browser history. API requests require the
token header. Static assets are allowlisted, requests are size-limited, and the
server-side path browser cannot leave the mounted home/invocation roots.

Set `ELESIM_NO_OPEN=1` to suppress `xdg-open`, or set
`ELESIM_GUI_PORT=<port>` to choose the first candidate port. Remote operators
must use SSH forwarding rather than exposing the GUI listener:

```bash
ssh -L 8765:127.0.0.1:8765 -p <ssh-port> <user>@<server>
```

The direct Python bootstrap is still supported where Python 3.10 and venv are
already usable:

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/installer/bootstrap/bootstrap.py \
  | python3 -
```

`ELESIM_REPOSITORY`, `ELESIM_REF`, `ELESIM_ARCHIVE_URL`,
`ELESIM_CACHE_DIR`, and `--refresh` control source retrieval. Extraction rejects
absolute paths, parent traversal, links, and device entries.

### Source Cache And Freshness

Source cache v2 keeps each archive URL in its own namespace and stores immutable
snapshots by the resolved Git commit or, for a custom archive without Git
metadata, by the archive SHA-256 digest. The legacy
`sources/<short-url-hash>` cache is left in place but is never an authoritative
input, so an old setup wheel cannot be selected merely because the same branch
URL was used before. GitHub codeload revision identity comes from the commit
recorded in the tar PAX header, as defined by
[Git archive](https://git-scm.com/docs/git-archive/2.46.0.html).

Mutable refs, including branches and tags, are validated on every invocation
with `If-None-Match` or `If-Modified-Since`. An HTTP `304 Not Modified` reuses
the snapshot only after its index and setup package are checked. A changed
response is downloaded and published as a new snapshot. Servers without
validators are downloaded each time and compared by content digest. A full
40-character commit SHA is immutable and may reuse a complete snapshot without
a network request.

Retrieval is fail-closed. A network, HTTP, extraction, archive-contract, or
bootstrap-generation failure preserves any previously completed snapshot for
diagnosis, but does not run it as a stale fallback. The setup entry point is
also checked for the contract-required `gui` command before the GUI starts;
source and release tests keep the complete command list aligned with the
contract.

The shell path passes `ELESIM_ARCHIVE_URL` through a temporary mode-0600 Docker
environment file and removes it on exit, so signed URL query values do not
appear in Docker arguments or cache metadata. It also enables an exact hash
comparison between the freshly downloaded `bootstrap.py` and the archived
copy. Direct stdin or standalone Python execution uses the versioned bootstrap
contract because there is no shell-owned download generation to compare.

Startup output identifies the requested ref, resolved commit or archive digest,
and whether the source was downloaded, validated by HTTP `304`, or read from an
immutable cache. `--refresh` skips conditional request headers and downloads
the complete archive again. When using the piped shell bootstrap, pass the
option to `bash` after `-s --`:

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/refactoring/installer/bootstrap/bootstrap.sh \
  | ELESIM_REF=refactoring bash -s -- --refresh
```

## GUI Request Model

`SetupRequest` is the transport-neutral boundary between the web handler and
installation code. The server overwrites client-provided source repository
fields with trusted bootstrap values, validates selected paths against mounted
roots, and then applies host capability and `InstallState` invariants.

The wizard only generates configuration, contexts, wrappers, and credentials.
It never builds images or starts services. Installation runs in one background
thread; logs are copyable and preserve a user's manual scroll position.
Cancellation is cooperative: it marks the job as cancelling and stops at the
next installer operation/log boundary instead of killing a file write halfway
through.

The default prefix is the directory from which the bootstrap command was
invoked. The default command directory is `<prefix>/bin`.

## General Installation

General mode translates to state schema v8 and one of two backends:

- Sim/Pilot/UI generate a Linux host-network Compose project.
- Robot alone invokes the native role-isolated venv installer on a detected
  Jetson.

The General wizard presents the four roles as independent checkboxes. Select
any non-empty combination of Sim, Pilot, and UI; Robot is available
only on a detected Jetson and must be selected alone. There are no
computer-type presets in the GUI or interactive terminal wizard. For
automation, repeat `--role` (for example `--role sim --role ui`). The
legacy `--profile` option remains hidden for compatibility with older scripts
and is not part of the new selection flow.

The generic container backend deliberately rejects Robot. A physical Jetson
requires a validated JetPack/L4T base, ROS2 Humble, `unitree_ros2`, device
nodes, and local safety permissions that cannot be represented by the generic
amd64 image.

The native installer writes two host-specific units:

- `elesim-robot.service` runs as the invoking host account, joins the
  `elesim-unitree` supplementary group, and owns the Elesim DDS/SROS2
  participant, arm/camera I/O and local safety.
- `elesim-unitree-bridge.service` runs as the dedicated `elesim-unitree`
  account, owns only local/plaintext Unitree DDS on its configured private
  NIC/domain, and is bound to the Robot lifecycle.

The processes exchange bounded JSON packets only through
`/run/elesim-unitree/bridge.sock` (`AF_UNIX` `SOCK_SEQPACKET`, directory
`0750`, socket `0660`). Both verify the configured peer UID through
`SO_PEERCRED`; boot IDs, monotonic sequences, payload allowlists and the Robot
deadman reject stale or malformed traffic. The generated config pins the
actual host account and absolute Unitree workspace. Bootstrap forwards
`ELESIM_UNITREE_ROS2_WS`, falling back to `UNITREE_ROS2_WS` and then
`$HOME/ros2_ws`. Unitree defaults to interface `eth0` and domain `1`; set
`ELESIM_UNITREE_INTERFACE` and `ELESIM_UNITREE_DOMAIN_ID` before bootstrap
when the private GO2 link uses different values. Native installation rejects
an empty inter-host DDS interface or any interface/domain overlap before it
mutates files.

Setup never invokes `sudo`. It prints exact commands to create/reuse the
dedicated account/group, grant the required read/traverse ACLs, register both
units, reload systemd and enable the Robot unit. A managed-SROS2 pending
install is enabled without `--now` and cannot start until provisioning removes
the marker. The bridge wrapper receives no Elesim SROS2 environment; its daemon
clears inherited security variables before binding CycloneDDS to the Unitree
NIC/domain.

Generated layout:

```text
<prefix>/
├── install-state.json
├── install-ownership.json           # UUID and exact deletion boundary
├── maintenance/                     # stdlib-only host uninstaller
├── containers/
│   ├── compose.yaml
│   └── build/
│       ├── <selected-role>/
│       └── tools/
├── roles/<selected-role>/{config,model?}/
├── cache/genesis/
├── security/                        # managed host generations/current
├── authority/                       # operator host only, after provisioning
├── connections/                     # non-secret topology
├── secrets/                         # when generated locally
└── logs/runs/                        # optional bounded text snapshots

<bin-dir>/
├── elesim-build
├── elesim-up
├── elesim-down
├── elesim-logs
├── elesim-setup
├── elesim-net
├── elesim-connections
├── elesim-uninstall
└── elesim-<selected-role>
```

A native Robot installation replaces `containers/` with `roles/robot/`,
`ros/`, and `tools/`, and stores both generated unit files below
`roles/robot/systemd/`. Its command set is `elesim-setup`, `elesim-net`,
`elesim-robot`, `elesim-unitree-bridge`, `elesim-up`, `elesim-logs`,
`elesim-down`, and `elesim-uninstall`. It has no image to build and connection
management belongs on the operator laptop, so native Robot emits neither
`elesim-build` nor `elesim-connections`.

Each application context contains `elesim_interfaces` plus one owned deployment
only.
Source configuration is copied into installed runtime data and never edited.
The Sim receives the immutable model bundle through a read-only mount.
The tools image contains ROS interfaces and setup/doctor, not deployment
implementations.

`network_mode: host` preserves the selected DDS interface/locators and the
meaning of loopback across generated Linux containers. The generated project
name is `elesim-runtime`; images are `elesim/<role>:local`, and selected
long-running containers are `elesim-pilot`, `elesim-ui`, and
`elesim-sim`. Managed TURN adds `elesim-coturn`. The tools service is
transient and is not a user-managed runtime application. A second general
installation on the same host fails its ownership guard instead of inventing
hash-derived names.

## Developer Installation

Developer mode requires Ubuntu/WSL amd64 and generates:

```text
<workspace>/
├── .git/                             # existing or cloned
├── pilot/ ui/ robot/ sim/ ...
├── .elesim/development/
│   ├── compose.yaml
│   ├── install-state.json
│   ├── install-ownership.json
│   ├── maintenance/
│   ├── build/
│   ├── home/
│   └── cache/
└── bin/
    ├── elesim-build
    ├── elesim-up
    ├── elesim-down
    ├── elesim-logs
    ├── elesim-dev
    ├── elesim-connections
    ├── elesim-uninstall
    └── elesim-jaeger-{up,down}       # optional
```

An existing nonempty path is reused only when it is a complete Elesim Git
checkout. The installer never pulls, resets, or deletes it. An existing empty
path is populated through a staging checkout inside that directory so a bind
mount/current working directory is not removed. An unrelated nonempty path is
rejected.

The image input is `environment/development`. It includes ROS2 Humble, Genesis,
Torch, Pinocchio, OpenCV, RealSense, Dynamixel, aiortc, OpenTelemetry, pytest,
build tools, and the pinned GO2 MPC source. Runtime uses host networking, host
IPC, `/dev`, X11, and privileged mode. On a bootstrap-detected WSLg host it
also mounts `/mnt/wslg` and forwards the runtime and Pulse endpoints.

The persistent development home owns `$HOME/.venv`. The entrypoint creates that
venv with system scientific packages visible, installs all Elesim projects
editable into it, prepends it to `PATH`, and then executes the requested
command. This avoids non-root writes to the image's global Python and keeps
console scripts available across restarts.

Optional Jaeger uses a separate Compose profile. The development service gets
OTLP HTTP environment only when Jaeger was selected. `elesim-jaeger-up` starts
the profile; ordinary `elesim-up` does not force observability overhead.

The project and image are fixed as `elesim-runtime-dev` and `elesim/dev:local`.
The only persistent development container is `elesim-dev`; optional tracing
adds `elesim-jaeger`. The `elesim-dev` wrapper starts the persistent container
when necessary and enters it with Compose `exec`, so opening more terminals
does not create randomly named `run --rm` development containers.
`elesim-connections` uses an explicit, removable `elesim-manager` one-shot
container with the Docker socket; it is a management tool, not another
persistent development service. In Developer mode it targets the ordinary
local installation at `~/.local/share/elesim` by default; override
`ELESIM_LOCAL_INSTALL_ROOT` and `ELESIM_LOCAL_BIN_DIR` together when needed.

## GPU Policy

Pilot, Sim, and the developer image share three policies:

- `inherit` forwards an externally assigned `CUDA_VISIBLE_DEVICES` when one is
  set and otherwise leaves application selection unrestricted.
- `specific` uses Docker's `device_ids` reservation to expose exactly one
  index, GPU UUID, or MIG UUID. It normally appears as logical `cuda:0` inside
  the container, so setup does not reapply the host index through
  `CUDA_VISIBLE_DEVICES`.
- `cpu` omits the Compose GPU request, sets an empty
  `CUDA_VISIBLE_DEVICES`, and writes the Sim profile with
  `simulation.runtime.use_gpu: false`. The generated developer image also
  selects CPU PyTorch wheels instead of downloading CUDA wheels.

The installer does not select the GPU with the most free memory because that
observation races with other jobs. On shared systems, leave `inherit` selected
and let the scheduler or launch environment remain authoritative.

NVIDIA modes require a working host driver and NVIDIA Container Toolkit. The
setup container cannot prove that the subsequently built runtime image can
create a GPU context; that remains a post-install check.

## Shell Registration

When selected, setup atomically manages exactly one block in `~/.bashrc`:

```text
# >>> Elesim managed PATH >>>
export PATH=/chosen/bin:"$PATH"
# <<< Elesim managed PATH <<<
```

The first edit preserves `~/.bashrc.elesim.bak`; repeated installs replace the
managed block without duplication. The setup child process cannot modify its
parent shell, so the completion page displays:

```bash
source ~/.bashrc
```

## Runtime Logs And Ownership-Based Uninstall

General installations default to an optional local text archive. Migrations
from state schemas v1 through v7 keep it disabled so an upgrade does not begin
retaining data silently. Every long-running General Compose service uses the
bounded Docker `json-file` driver (`10m`, four files); managed Coturn follows
the same bound. Developer mode is unchanged.

`elesim-logs` with no arguments follows live logs. `elesim-logs --save` writes
one service file per snapshot below `<prefix>/logs/runs/<UTC timestamp>`.
`elesim-down` snapshots before shutdown when archiving is enabled, still runs
the stop operation after an archive failure, and returns nonzero when the
snapshot failed. Native Robot exports both Robot and bridge journald units.
Archives retain the newest five generations, cap each native export at 10 MiB,
and use directory mode `0700` and file mode `0600`. Direct or ancestor symlink
substitution fails closed.

Each completed install writes an ownership manifest and a stdlib-only host
launcher. General/native manifests are `<prefix>/install-ownership.json`;
Developer keeps its manifest under
`<workspace>/.elesim/development/install-ownership.json` so the source checkout
is never treated as generated runtime data. A missing manifest never causes
legacy files to be adopted automatically. The operator must back up and remove
the named legacy generated paths or use that installation's older cleanup
procedure first.

The GUI only validates the manifest and emits exact terminal commands; the
disposable loopback web process receives neither the Docker socket nor a host
deletion channel. The host CLI always plans and revalidates before mutation:

```bash
elesim-uninstall --plan
elesim-uninstall --confirm-prefix /exact/install/prefix
```

The default preserves runtime text logs and the operator SROS2 Authority.
`--purge-logs` and `--purge-authority` are explicit irreversible opt-ins.
External source, TURN credentials and SROS2 keystores are always preserved.
Only exact `elesim/*:local` images and containers whose Compose metadata and
install UUID label match are eligible; there is no prune or wildcard deletion.
For native Robot, an installed or active systemd unit aborts the entire plan
before mutation and prints exact removal commands only when the unit path and
SHA-256 match the generated copy.

## DDS Network And Security

The generated DDS runtime profile contains:

- a ROS-safe `system_id` shared by one Elesim graph;
- `domain_id` and a pinned `rmw_implementation`;
- `multicast` or `static` discovery;
- reachable static peer addresses when multicast is not routed;
- the local interface name used for DDS;
- `trusted-network` or `sros2` security;
- for SROS2, `external` or `managed` provisioning, the role's keystore path and
  enclave, and for managed bundles the active string generation ID.

Static peers seed DDS discovery but do not proxy user traffic or cross NAT.
Every required pair of participants must have bidirectional UDP reachability.
The installer rejects loopback for a multi-host profile and warns that ordinary
IPv4 NAT, CGNAT and symmetric NAT are unsupported. A routed VPN is the
supported remote-laptop topology.

`trusted-network` deliberately enables no DDS encryption. It is allowed only
after the operator confirms that the selected LAN/VPN interface and firewall
limit participation to trusted machines. `ROS_DOMAIN_ID` prevents accidental
graph overlap only; it is not a security control.

`sros2` enables DDS Security in enforce mode. State schema v8 distinguishes:

- `external`: the operator supplies and maintains a local keystore/base
  enclave. Elesim records no managed generation and does not rotate it;
- `managed`: `elesim-connections` keeps the complete Authority on the operator
  laptop, creates a string generation through `ros2 security`, and installs a
  host bundle as the runtime keystore.

For an initial General installation, managed provisioning is intentionally
allowed without a keystore, enclave or generation. Setup still writes the role
configuration, Compose contexts and commands, then atomically creates
`<prefix>/security/provisioning-required`. Generated application start wrappers
fail closed on that marker and direct the operator to `elesim-connections`.
An all-host managed generation activation (or an explicit switch to
`trusted-network`) regenerates configuration and removes the marker in the same
local transaction; rollback restores it. External provisioning continues to
require an existing keystore and enclave during setup. Developer edition does
not accept managed provisioning.

A managed bundle contains common public trust material plus only the enclaves
assigned to that host. Its manifest fixes system, host, generation, enclave
paths and SHA-256 file digests. Authority CA private keys and unrelated role
keys must never enter a host bundle. Security directories and files are
published with private `0700`/`0600` modes.

Setup creates a stable `security/roles/<role>` root for every installed role.
Application containers mount only their own root read-only. For an external
keystore, setup materializes the public files and exact configured enclave into
that root as regular files without modifying the source keystore. Consequently,
changing an external keystore, base enclave, or endpoint ID with `elesim-net
configure` is rejected: reinstall to rebuild the role views explicitly.

The loopback GUI and its SSH forwarding are unchanged. SSH mode may use an
agent/default keys or a selected private key, pins the confirmed host
fingerprint, and does not accept passwords. SSH is a setup transfer channel,
not ROS 2/DDS or WebRTC transport.

## Connection Manager

Run `elesim-connections` on the operator laptop after installing the roles. Its
loopback/token-protected browser UI has two explicit modes:

- `full`: assign Pilot, UI, Sim, and Robot exactly once across two to
  four active hosts. Robot is fixed to a native Jetson/systemd host.
- `simulation-only`: assign Pilot, UI, and Sim exactly once across
  one to three container/Compose hosts. Robot and Jetson are absent by design.

Exactly one host is local; a host may own multiple roles. Schema-v1 topology files
are read as `full` and normalized to schema v3 with an explicit mode.

Each host records two independent paths:

- DDS address and interface: runtime UDP reachability and static-peer
  derivation;
- optional remote SSH host, port, user, agent/identity-file choice, and pinned
  SHA-256 host-key fingerprint: preflight, bundle transfer, lifecycle and logs.

An SSH hostname may equal a DDS address, but neither value is derived from the
other. SSH port `2222` is an administration example only. Topology state is
non-secret: it may retain an identity-file path and host fingerprint, but never
a password, private-key body, SROS2 key, TURN secret, credential, or token.

When the physical Jetson is unavailable, the GUI's **two-host preflight** can
check exactly two active COM cards without saving or deploying a topology. Enter
the current, mutable DDS address (hostname/IP only, no `:port`) and interface
(`tailscale0` on a Tailscale path), then enter the remote host's actual SSH
management host, user, and port. Ordinary SSH over Tailscale uses the sshd port
(normally 22, unless that host was configured differently); the connection
manager does not invent a Tailscale or DDS port. A temporary
`python3 -m http.server 8080` reachability check is outside this document and
must not be entered as a DDS or SSH port. The optional host-key probe is not a
proof of bidirectional DDS, SROS2, RGBD, WebRTC, or NAT traversal. Only `full`
deployment requires the Robot role; `simulation-only` deployment intentionally
starts the three simulation roles without a physical Robot.

If `tailscale0` is present, the connection manager performs a read-only local
`ip -j -4 addr` probe and may prefill the current IPv4 address/interface. This
is only a convenience hint: it never installs Tailscale, logs in, changes ACLs,
or hard-codes an address, and the operator must refresh the value after a
Tailscale reconnect. A routed VPN is recommended for hosts on different
networks; DDS still requires a bidirectional UDP path.

For managed SROS2, provisioning creates role identities and per-host bundles;
deployment first preflights every host and stages the same generation on all of
them. Rotation then captures current state, stops all affected roles, switches
each host's `security/current` link atomically, restarts and runs the network
doctor. A failure restores the previous generation/configuration and restarts
every host already stopped. Partially mixed live generations are not accepted.

## TURN Ownership

State schema v8 keeps TURN endpoint URLs separate from relay ownership and
records an optional Sim-only external credential file:

- `none`: no TURN URL;
- `external`: consume an independently managed relay; a Sim installation
  selects a JSON file containing `username`, `credential`, and optional finite
  `expires_at`;
- `managed`: include Coturn in the generated general-user Compose project.

Managed TURN requires a Sim host, realm, public host, one TURN URL, and a
credential policy. Because credentials and signaling cross DDS, managed mode
requires the `sros2` profile. The Coturn service uses host networking, a pinned
image, and UDP relay range `49160-49200`. Because it is in the same Compose
project, `elesim-up`, `elesim-down`, and `elesim-logs` own its lifecycle. The
standalone release Coturn Compose remains available only for operators who
deliberately choose `external`.

WebRTC remains DTLS/SRTP in both DDS security profiles. Managed Coturn mounts
its REST HMAC secret into Coturn and the co-located Sim only. Sim
issues short-lived credentials bound to its active UI session; UI receives the
issued credential but never the static secret. This makes Sim part of the
managed TURN trust boundary. For external TURN, setup mounts the selected JSON
read-only into the Sim container only; Pilot/UI-only installations
store the URL but do not receive or require the file. Sim sends the
usable username/password to the active UI as part of the DDS session grant.
With `trusted-network`, that exchange inherits the controlled-LAN/VPN trust
assumption; use SROS2 when other users can join or observe the DDS network.

TURN relays WebRTC media only. It cannot make DDS discovery, topics, services,
actions, or SDP signaling reachable through NAT.

Schema-v1/v2 TURN URLs continue to migrate to `external`. Schema v1-v4 states
have no external credential-file field; they remain inspectable, but a
Sim configuration fails closed until the operator selects that file. A schema-v3
Router/ZMQ state migrates to multicast discovery with no inferred static peers:
the old Router address is not enough to prove bidirectional DDS reachability.
An old Curve selection cannot be translated into SROS2 identity/permissions and
therefore migrates fail-closed until the operator selects `trusted-network`
under its stated network assumptions or supplies a valid SROS2
keystore/enclave. Schema-v6 SROS2 state migrates to `external` provisioning;
it is never relabeled as connection-manager-owned material. Only an explicit
managed provisioning/rotation records `security_generation` and
`security_bundle`.

## Non-Interactive Installation

Automation can continue to use:

```bash
PYTHONPATH=installer/package/src \
python3 -m elesim_setup.cli \
  --source-root "$PWD" \
  install \
  --mode container \
  --role sim \
  --dds-system-id elesim \
  --dds-domain-id 42 \
  --dds-rmw-implementation rmw_cyclonedds_cpp \
  --dds-discovery-mode static \
  --dds-static-peer 10.40.0.20 \
  --dds-interface wg0 \
  --gpu-mode inherit \
  --dds-security-profile sros2 \
  --dds-security-provisioning managed \
  --turn-mode external \
  --turn-url 'turn:sim.example.com:3478?transport=udp'
```

`--dry-run` validates and prints the plan without writing runtime files.
With `--dds-security-provisioning external`, credentials must already be
provisioned. `managed` creates a pending, non-runnable General installation;
connection-manager SSH preflight, Authority issuance and all-host rollout still
occur explicitly through `elesim-connections` rather than inside installation.

## Address Reconfiguration

```bash
elesim-net configure
```

Or:

```bash
elesim-net configure --non-interactive \
  --dds-system-id elesim \
  --dds-domain-id 42 \
  --dds-discovery-mode static \
  --dds-static-peer 10.40.0.20 \
  --dds-interface wg0 \
  --dds-security-profile sros2 \
  --dds-keystore /etc/elesim/keystore \
  --dds-enclave /elesim/pilot
```

This rewrites only installed role configuration. Restart roles afterward.
All participants in one graph use the same system/domain IDs and compatible
RMW/QoS settings. `dds.interface` is local to each host. Static peer addresses
name directly reachable participants; `0.0.0.0`, loopback on a multi-host
installation, and an address hidden behind NAT are invalid peers.

The command above is the external-keystore path. `elesim-net configure` does
not replace a managed generation or bundle; change managed topology/security
through `elesim-connections` so every host moves as one transaction.

## Network Doctor

```bash
elesim-net doctor
```

The non-disruptive pass checks:

- DDS participant discovery, endpoint descriptors and heartbeats;
- duplicate endpoint IDs and boot/process-instance changes;
- endpoint, control-carrier, and RGBD topic names and QoS compatibility;
- motion/session authority exchange reachability;
- one passive RGBD subscription without taking a UI session;
- TURN UDP STUN Binding or TCP connectivity;
- Sim advertisement of both DTLS-SRTP WebRTC streams.

The active pass consumes media:

```bash
elesim-net doctor --active --timeout 8
```

It receives one coherent DDS `RgbdFrame`. WebRTC negotiation and decoded-frame
validation remain a live UI test because the doctor does not take the
Sim's exclusive UI session. A successful TURN probe says nothing about
DDS reachability or whether an ICE relay candidate was selected.

Use `--json` for automation. DDS discovery/QoS failure, SROS2 denial for an
authorized role, malformed interfaces, unsupported NAT-only topology, or
missing requested frames are `FAIL`. Optional services that are intentionally
absent are `SKIP` or `WARN`.

## Verification Boundary

Setup tests must cover:

- request and host capability validation;
- state migration and serialization;
- generated role/tools/developer contexts and nested Python packages;
- shell block idempotence;
- trusted-network acknowledgement, SROS2 role-scoped enclave validation and
  external/managed state validation, bundle digest/mode/path containment and
  overwrite protection;
- connection topology validation, separate DDS/SSH endpoints, pinned SSH host
  keys, all-host staging, activation and rollback;
- web asset packaging, token/API boundaries, path containment, and job states;
- bootstrap extraction and shell invocation;
- generated release infrastructure.

Source-tree imports are insufficient. Release verification must inspect the
copied setup package and built wheel, including both `web/` and
`connection_web/` assets and the CJK font.

No automated setup test establishes real SSH-agent behavior against a remote
host, multicast/static-peer behavior on the owning network, SROS2 enforcement
with the production RMW, Wi-Fi/VPN reconnect, WSLg/X11 rendering, NVIDIA
runtime access, Jetson hardware support, Coturn relay selection across an
actual NAT, or RGBD/Genesis frame latency. Record those as explicit manual
validation results.
