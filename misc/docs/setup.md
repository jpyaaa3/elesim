# Setup Wizard and Network Doctor

## Scope

`misc/tooling/setup` owns installation request validation, generated runtime
configuration, role-isolated build contexts, shell wrappers, credential
provisioning, and post-install network diagnosis. It does not own a deployment's
domain behavior and imports no Controller, UI, Simulator, or Robot
implementation.

The browser wizard offers two editions:

- **General** installs a selected subset of the four runtime applications.
  Simulator, Controller, and UI use role-isolated Docker images. Robot is a
  native-only, Jetson-detected, exclusive selection. Router is not a role.
- **Developer** prepares one complete Git workspace and one privileged
  Ubuntu/ROS2 development image. It includes all applications, model tooling,
  tests, graphics/scientific dependencies, and optional Jaeger.

The existing terminal `wizard` and non-interactive `install` subcommands remain
available for automation and compatibility. They use the same state and
container generators, but only the browser path exposes the edition,
role-scoped SSH transfer, and developer environment as one guided flow.

## Public Bootstrap

On a clean Ubuntu or WSL host:

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/misc/setup/bootstrap.sh | bash
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
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/misc/setup/bootstrap.py \
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
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/refactoring/misc/setup/bootstrap.sh \
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

General mode translates to state schema v5 and one of two backends:

- Simulator/Controller/UI generate a Linux host-network Compose project.
- Robot alone invokes the native role-isolated venv installer on a detected
  Jetson.

The generic container backend deliberately rejects Robot. A physical Jetson
requires a validated JetPack/L4T base, ROS2 Humble, `unitree_ros2`, device
nodes, and local safety permissions that cannot be represented by the generic
amd64 image.

Generated layout:

```text
<prefix>/
├── install-state.json
├── containers/
│   ├── compose.yaml
│   └── build/
│       ├── <selected-role>/
│       └── tools/
├── roles/<selected-role>/{config,model?}/
├── cache/genesis/
└── secrets/                         # when generated locally

<bin-dir>/
├── elesim-build
├── elesim-up
├── elesim-down
├── elesim-logs
├── elesim-setup
├── elesim-net
└── elesim-<selected-role>
```

Each application context contains `elesim_interfaces` plus one owned deployment
only.
Source configuration is copied into installed runtime data and never edited.
The Simulator receives the immutable model bundle through a read-only mount.
The tools image contains ROS interfaces and setup/doctor, not deployment
implementations.

`network_mode: host` preserves the selected DDS interface/locators and the
meaning of loopback across generated Linux containers. Prefix-derived Compose
names and image tags prevent separate installs from overwriting one another.

## Developer Installation

Developer mode requires Ubuntu/WSL amd64 and generates:

```text
<workspace>/
├── .git/                             # existing or cloned
├── controller/ ui/ robot/ simulator/ ...
├── .elesim/development/
│   ├── compose.yaml
│   ├── install-state.json
│   ├── build/
│   ├── home/
│   └── cache/
└── bin/
    ├── elesim-build
    ├── elesim-up
    ├── elesim-down
    ├── elesim-logs
    ├── elesim-dev
    └── elesim-jaeger-{up,down}       # optional
```

An existing nonempty path is reused only when it is a complete Elesim Git
checkout. The installer never pulls, resets, or deletes it. An existing empty
path is populated through a staging checkout inside that directory so a bind
mount/current working directory is not removed. An unrelated nonempty path is
rejected.

The image input is `misc/infra/development`. It includes ROS2 Humble, Genesis,
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

## GPU Policy

Controller, Simulator, and the developer image share three policies:

- `inherit` forwards an externally assigned `CUDA_VISIBLE_DEVICES` when one is
  set and otherwise leaves application selection unrestricted.
- `specific` uses Docker's `device_ids` reservation to expose exactly one
  index, GPU UUID, or MIG UUID. It normally appears as logical `cuda:0` inside
  the container, so setup does not reapply the host index through
  `CUDA_VISIBLE_DEVICES`.
- `cpu` omits the Compose GPU request, sets an empty
  `CUDA_VISIBLE_DEVICES`, and writes the Simulator profile with
  `simulation.runtime.use_gpu: false`.

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

## DDS Network And Security

The generated DDS runtime profile contains:

- a ROS-safe `system_id` shared by one Elesim graph;
- `domain_id` and a pinned `rmw_implementation`;
- `multicast` or `static` discovery;
- reachable static peer addresses when multicast is not routed;
- the local interface name used for DDS;
- `trusted-network` or `sros2` security;
- for SROS2, the role's keystore path and enclave.

Static peers seed DDS discovery but do not proxy user traffic or cross NAT.
Every required pair of participants must have bidirectional UDP reachability.
The installer rejects loopback for a multi-host profile and warns that ordinary
IPv4 NAT, CGNAT and symmetric NAT are unsupported. A routed VPN is the
supported remote-laptop topology.

`trusted-network` deliberately enables no DDS encryption. It is allowed only
after the operator confirms that the selected LAN/VPN interface and firewall
limit participation to trusted machines. `ROS_DOMAIN_ID` prevents accidental
graph overlap only; it is not a security control.

`sros2` enables DDS Security in enforce mode. The selected local keystore must
contain the role's enclave, identity and permissions. Setup may generate or
install only the selected role's enclave; SROS2 authority/material creation is
an external administration step. It must not copy a complete certificate
authority or every role's private material to each host.

The loopback GUI and its SSH forwarding are unchanged. SSH mode may use an
agent/default keys or a selected private key, pins the confirmed host
fingerprint, and does not accept passwords. SSH is a setup transfer channel,
not ROS 2/DDS or WebRTC transport.

## TURN Ownership

State schema v5 keeps TURN endpoint URLs separate from relay ownership and
records an optional Simulator-only external credential file:

- `none`: no TURN URL;
- `external`: consume an independently managed relay; a Simulator installation
  selects a JSON file containing `username`, `credential`, and optional finite
  `expires_at`;
- `managed`: include Coturn in the generated general-user Compose project.

Managed TURN requires a Simulator host, realm, public host, one TURN URL, and a
credential policy. Because credentials and signaling cross DDS, managed mode
requires the `sros2` profile. The Coturn service uses host networking, a pinned
image, and UDP relay range `49160-49200`. Because it is in the same Compose
project, `elesim-up`, `elesim-down`, and `elesim-logs` own its lifecycle. The
standalone release Coturn Compose remains available only for operators who
deliberately choose `external`.

WebRTC remains DTLS/SRTP in both DDS security profiles. Managed Coturn mounts
its REST HMAC secret into Coturn and the co-located Simulator only. Simulator
issues short-lived credentials bound to its active UI session; UI receives the
issued credential but never the static secret. This makes Simulator part of the
managed TURN trust boundary. For external TURN, setup mounts the selected JSON
read-only into the Simulator container only; Controller/UI-only installations
store the URL but do not receive or require the file. Simulator sends the
usable username/password to the active UI as part of the DDS session grant.
With `trusted-network`, that exchange inherits the controlled-LAN/VPN trust
assumption; use SROS2 when other users can join or observe the DDS network.

TURN relays WebRTC media only. It cannot make DDS discovery, topics, services,
actions, or SDP signaling reachable through NAT.

Schema-v1/v2 TURN URLs continue to migrate to `external`. Schema v1-v4 states
have no external credential-file field; they remain inspectable, but a
Simulator configuration fails closed until the operator selects that file. A schema-v3
Router/ZMQ state migrates to multicast discovery with no inferred static peers:
the old Router address is not enough to prove bidirectional DDS reachability.
An old Curve selection cannot be translated into SROS2 identity/permissions and
therefore migrates fail-closed until the operator selects `trusted-network`
under its stated network assumptions or supplies a valid SROS2
keystore/enclave.

## Non-Interactive Installation

Automation can continue to use:

```bash
PYTHONPATH=misc/tooling/setup/src \
python3 -m elesim_setup.cli \
  --source-root "$PWD" \
  install \
  --mode container \
  --profile compute \
  --dds-system-id elesim \
  --dds-domain-id 42 \
  --dds-rmw-implementation rmw_cyclonedds_cpp \
  --dds-discovery-mode static \
  --dds-static-peer 10.40.0.20 \
  --dds-interface wg0 \
  --gpu-mode inherit \
  --dds-security-profile trusted-network \
  --turn-mode external \
  --turn-url 'turn:sim.example.com:3478?transport=udp'
```

`--dry-run` validates and prints the plan without writing runtime files.
Non-interactive CLI assumes credentials are already provisioned; GUI-only SSH
transfer is intentionally not hidden inside this legacy surface.

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
  --dds-enclave /elesim/controller
```

This rewrites only installed role configuration. Restart roles afterward.
All participants in one graph use the same system/domain IDs and compatible
RMW/QoS settings. `dds.interface` is local to each host. Static peer addresses
name directly reachable participants; `0.0.0.0`, loopback on a multi-host
installation, and an address hidden behind NAT are invalid peers.

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
- Simulator advertisement of both DTLS-SRTP WebRTC streams.

The active pass consumes media:

```bash
elesim-net doctor --active --timeout 8
```

It receives one coherent DDS `RgbdFrame`. WebRTC negotiation and decoded-frame
validation remain a live UI test because the doctor does not take the
Simulator's exclusive UI session. A successful TURN probe says nothing about
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
  overwrite protection;
- web asset packaging, token/API boundaries, path containment, and job states;
- bootstrap extraction and shell invocation;
- generated release infrastructure.

Source-tree imports are insufficient. Release verification must inspect the
copied setup package and built wheel, including `web/` and its CJK font.

No automated setup test establishes real SSH-agent behavior against a remote
host, multicast/static-peer behavior on the owning network, SROS2 enforcement
with the production RMW, Wi-Fi/VPN reconnect, WSLg/X11 rendering, NVIDIA
runtime access, Jetson hardware support, Coturn relay selection across an
actual NAT, or RGBD/Genesis frame latency. Record those as explicit manual
validation results.
