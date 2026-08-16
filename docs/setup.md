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
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/installer/bootstrap/install.sh | bash
```

The shell bootstrap:

1. Checks Docker Engine and Compose v2. If Docker is absent on Ubuntu, it asks
   before installing Docker packages; declining leaves the host unchanged.
2. Reports host-only facts that would otherwise disappear inside the setup
   container: OS/architecture, Jetson, WSL/WSLg, display availability,
   `nvidia-smi -L`, invocation directory, user, SSH agent socket, the selected
   Docker backend/context/engine identity, and any host `tailscale*`
   interfaces. The Docker facts select and pin `direct-host` versus
   `tailscale-sidecar`; host-interface facts remain hints. Runtime
   namespace-check is authoritative for the actual interface/address/route.
3. Downloads the standard-library `bootstrap.py` to a temporary file in the
   setup cache and atomically publishes the complete download.
4. Runs it as the calling UID/GID in a disposable `python:3.10-slim`
   container. The container receives the user's home and invocation directory,
   but never the Docker socket. Bootstrap also passes the outer account name
   as `ELESIM_HOST_USER`; the numeric UID is not assumed to exist in the
   disposable image's `/etc/passwd`.
5. Downloads and safely extracts the requested GitHub source archive, creates a
   cached setup venv, and starts `elesim-setup gui`.
6. Publishes the GUI on host loopback only. Port `8765` is preferred; the
   bootstrap searches the next 99 ports when it is occupied.

Bootstrap rejects a `DOCKER_HOST` override and a Docker context whose endpoint
is remote `ssh://` or `tcp://`. Generated Compose bind mounts contain local
absolute installation paths, so a remote daemon cannot be adopted as if it
owned those paths. Local Unix-socket and Windows named-pipe contexts remain
supported and are pinned with the observed Engine ID.

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
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/refactoring/installer/bootstrap/install.sh \
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

General mode translates to state schema v9 and one of two container-network
backends:

- Sim/Pilot/UI use the installation-time resolved `direct-host` or
  `tailscale-sidecar` backend.
- Robot invokes the native role-isolated venv installer as its own unit on a
  detected Jetson. A separate general installation may create a Compose unit
  for Pilot/UI on that same host; the two prefixes and lifecycles remain
  independent.

The General wizard presents the four roles as independent checkboxes. Select
any non-empty combination of Sim, Pilot, and UI; Robot is available
only on a detected Jetson and remains a native-only installation, so it must be
selected alone within that installation. A Jetson can therefore have a second
container installation for Pilot/UI. There are no
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
  `elesim-unitree` supplementary group, and owns the EleSim DDS/SROS2
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
the marker. The bridge wrapper receives no EleSim SROS2 environment; its daemon
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
├── elesim-update
├── elesim-logs
├── elesim-status
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
`elesim-down`, `elesim-status`, `elesim-update`, and `elesim-uninstall`. It has no image to build and connection
management belongs on the operator laptop, so native Robot emits neither
`elesim-build` nor `elesim-connections`.

Each application context contains `elesim_interfaces` plus one owned deployment
only.
Source configuration is copied into installed runtime data and never edited.
The Sim receives the immutable model bundle through a read-only mount.
The tools image contains ROS interfaces and setup/doctor, not deployment
implementations.

The installer resolves one runtime-network backend from the selected Docker
daemon and fixes that result in the installed state:

- `direct-host` (**Native host network**) uses `network_mode: host`. Choose this
  for a native Docker Engine whose host namespace already contains the selected
  LAN/VPN interface.
- `tailscale-sidecar` (**Docker Desktop Tailscale sidecar**) creates the
  `tailscale` Compose service and fixed `elesim-tailscale` container inside
  Docker Desktop's Linux VM. Roles, the dedicated `runtime-tools` doctor, and
  active Sim-owned Coturn join it with `network_mode: service:tailscale`, so
  their runtime namespace contains the sidecar's kernel-mode `tailscale0`. The
  ordinary administrative `tools` service remains usable before enrollment.
  The privileged upstream Tailscale image is pinned by version and multiarch
  image-index digest rather than a mutable tag alone.

The automatic decision is made during installation and only the resolved
backend is saved; generated wrappers do not switch Docker contexts on every
start. Docker Desktop does not inherit the WSL distribution's existing
`tailscale0`, so the sidecar is a separate tailnet node with its own address and
persistent node state. It is host network infrastructure, not a fifth EleSim
application, Router, DDS relay, or SSH endpoint.

The transient connection-manager container remains bridged and publishes only
its selected GUI port on host loopback. Its private helper may proxy Tailscale
SSH host-key and deployment connections through host `tailscale nc`; that is an
SSH-management fallback only and never carries DDS UDP. The sidecar path is
different: roles, the dedicated runtime-network doctor, and active Sim-owned
Coturn share the sidecar's actual network namespace.

The lightweight `elesim-net namespace-check` runs in the same namespace as the
role containers before managed security material is issued and immediately
before runtime start. It verifies the selected interface, checks that the
advertised DDS address is assigned to that interface, and, for static
discovery, checks each configured peer route. Failure stops before a new
security generation is left behind. These are structural bind/route checks,
not proof of DDS discovery or application traffic. SSH/Tailscale TCP success
never substitutes for DDS UDP evidence.

#### Docker backend and Tailscale enrollment

The installer does not install, stop, or switch Docker for the host. It uses
the daemon selected by the current `docker` CLI context and reports the fixed
backend during `elesim-net namespace-check`:

```bash
docker info --format 'name={{.Name}} os={{.OperatingSystem}}'
docker context show
ip -br addr | awk '$1 ~ /^tailscale[0-9]+$/ {print}'
```

If the first command reports `docker-desktop` or the context is
`desktop-linux`, the generated container installation uses
`tailscale-sidecar`. Enroll that Docker-side node once:

```bash
elesim-tailscale login
elesim-tailscale status
```

`login` presents a browser/device authorization flow. If the local sidecar
still reports `Running`, an explicit `login` performs Tailscale
re-authentication so a node removed from the admin console cannot be silently
accepted from stale local state. Runtime launch uses an idempotent internal
check and does not open a browser. EleSim does not request
or persist a Tailscale auth/OAuth key or browser credential. The mode-0700
`<prefix>/secrets/tailscale` directory retains only the sidecar's node state so
normal `elesim-down`, `elesim-up`, and `elesim-update` do not require repeated
enrollment. Use the sidecar address
reported by `status` as that host's DDS address. Keep the WSL/host address as
the independent SSH management destination when they differ.

On a native Docker Engine, `direct-host` continues to use the host's existing
interface. The operator may deliberately select a different Docker context
before installation, but EleSim never toggles between native Engine and Docker
Desktop behind other projects' backs.

After installation (the setup wizard intentionally does not build or start
runtime images), run the lightweight check on the machine owning the role:

```bash
elesim-net namespace-check --dds-interface tailscale0
```

For `direct-host`, replace `tailscale0` with the actual host interface when
necessary. For `tailscale-sidecar`, keep `tailscale0` and supply the sidecar's
DDS address through the connection manager. The check enumerates the runtime
container namespace, not merely WSL or the outer host. The installer itself
does not receive a Docker socket and never changes Tailscale ACLs.

For a full lifecycle start, the same private helper accepts only the fixed
EleSim Compose build shape and streams its actual
`docker compose --progress plain build` stdout/stderr back to the manager. A
remote host streams the same output over its already authenticated SSH channel.
The GUI job log and the terminal that launched `elesim-connections` therefore
show real BuildKit lines, not a synthetic heartbeat. Output lines are bounded
and redacted before presentation; the manager still receives neither daemon
socket. After all images are built, the detached Compose lifecycle step uses a
bounded five-minute command timeout. This is separate from the thirty-minute
image build limit, so a slow Docker backend does not make `up --no-build` appear
to be a failed security rollout and trigger an avoidable rollback.
The connection manager performs that launch through the generated
`elesim-up --no-build` wrapper. Its one-shot Viewer option becomes `--view`,
so the normal `DISPLAY` check and temporary `xhost` grant are not bypassed;
for a remote non-interactive SSH launch the host wrapper passes the SSH
management username to `elesim-up --viewer-user` and resolves only bounded
local X11 sockets owned by that account and matching its Xauthority candidates,
failing before Compose when none is usable. When several sessions are present,
it probes their monitor names and prefers a physical connector such as `DP-*`
or `HDMI-*` over an NX, VNC, or other virtual output; an equal-ranked tie is
resolved deterministically (the SSH session's inherited display first, then
socket order). Inherited SSH-forwarded/TCP displays are rejected because the Sim
container receives only the local `/tmp/.X11-unix` socket mount. After
the grant, a one-shot Sim container running with the installed image and UID/GID
must open a hidden X11/GL context before the detached runtime is started. The
normal Sim entrypoint repeats that check before starting DDS, so a display
failure cannot masquerade briefly as DDS readiness. A connection-manager Stop
or failed full Start revokes the same EleSim-owned grant after Sim is stopped;
its bounded, role-specific GPU index/UUID options become
`--cuda-visible-devices` and set `CUDA_VISIBLE_DEVICES` only for that launch.
The connection manager reads each installed Pilot/Sim host policy first;
fixed `specific` and `cpu` policies are shown as disabled controls, while only
`inherit` remains operator-editable.
The generated project name is `elesim-runtime`; images are
`elesim/<role>:local`, and selected
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
    ├── elesim-update
    ├── elesim-logs
    ├── elesim-status
    ├── elesim-dev
    ├── elesim-connections
    └── elesim-uninstall
```

When Jaeger was selected, `elesim-up --jaeger` starts its separate Compose
profile; no `elesim-jaeger-*` wrapper is generated.

An existing nonempty path is reused only when it is a complete EleSim Git
checkout. The installer never pulls, resets, or deletes it. An existing empty
path is populated through a staging checkout inside that directory so a bind
mount/current working directory is not removed. An unrelated nonempty path is
rejected.

## Incremental updates

Every completed install emits a host-side `elesim-update` wrapper. It downloads
the bootstrap from the repository and ref recorded at installation, so source
integrity and bootstrap-generation checks are identical to a fresh install. It
then runs the non-interactive `update` command against the existing state and
ownership manifest.

General state schema v9 persists this source identity as
`source_repository`/`source_ref`; the generated wrapper prints the effective
`repository@ref` before fetching it. An older state without these fields uses
the public `jpyaaa3/elesim@main` default. A wrapper generated before source
pinning has that default baked into its shell text, so setting an environment
variable cannot redirect that old wrapper. Recover it once through the
bootstrap itself, then let the regenerated wrapper take over:

```bash
curl -fsSL https://raw.githubusercontent.com/owner/repo/ref/installer/bootstrap/install.sh \
  | ELESIM_REPOSITORY=owner/repo ELESIM_REF=ref bash -s -- update
```

For a current wrapper, `ELESIM_REPOSITORY=owner/repo ELESIM_REF=ref
elesim-update` is accepted as an explicit, bounded recovery override; the
resulting state records that identity for later updates. `elesim-down --purge`
only removes the installed runtime/manager and does not change the source ref.

For a general container install, refresh preserves the install UUID and mutable
runtime data: topology, managed SROS2 generations and Authority, secrets, logs,
and application caches. It rewrites installer-owned configuration, wrappers,
and build contexts, then builds the selected role images and tools image using
Docker's normal layer cache. It neither runs Compose `down` nor recreates a
running container; `elesim-up` is the explicit activation step.

The generated update and runtime wrappers enforce the installation owner's UID;
run them from that same host account rather than through `sudo`. A root-owned
legacy cache/context is preserved and bypassed with a private fallback, so an
update can proceed without adopting or deleting another user's files.

State v1-v8 and their ownership manifests did not pin a Docker context/Engine
ID. Their first v9 update may adopt the selected daemon only when at least one
exact install-UUID/Compose-labelled container or local image proves that the
old installation belongs to that daemon. A foreign object, ambiguous label, or
an unbuilt legacy install with no Docker artifact fails closed. Select the
original daemon and retry, or use that installation's validated clean uninstall
and reinstall; the updater never invents ownership from an empty daemon.

A v9 Engine-ID mismatch also fails every generated Docker wrapper and
ownership-based uninstall. There is intentionally no automatic rebind after a
Docker Desktop factory reset or daemon replacement. Restore the pinned daemon
long enough to uninstall, or retain the old prefix as evidence and perform a
fresh install in a new empty prefix pending an audited manual cleanup. Do not
edit `install-state.json` or the ownership manifest to bypass the guard.

For a Developer install, the wrapper first rejects staged or unstaged tracked
changes, fetches the installed ref from `origin`, and permits only a
fast-forward merge. Untracked files remain unless Git rejects a path collision.
It then refreshes `.elesim/development` and incrementally builds
`elesim/dev:local`. Native Robot uses the same ownership validation to refresh
its owned venv/configuration/systemd inputs but has no Docker image build.

No edition performs a broad delete, Docker prune, branch switch, security key
rotation, topology inference, or automatic runtime restart.

The image input is `environment/development`. It includes ROS2 Humble, Genesis,
Torch, Pinocchio, OpenCV, RealSense, Dynamixel, aiortc, OpenTelemetry, pytest,
build tools, and the pinned GO2 MPC source. Runtime uses host networking, host
IPC, `/dev`, X11, and privileged mode. On a bootstrap-detected WSLg host it
also mounts `/mnt/wslg` and forwards the runtime and Pulse endpoints.

The persistent development home owns `$HOME/.venv`. The entrypoint creates that
venv with system scientific packages visible, installs all EleSim projects
editable into it, prepends it to `PATH`, and then executes the requested
command. This avoids non-root writes to the image's global Python and keeps
console scripts available across restarts.

Optional Jaeger uses a separate Compose profile. The development service gets
OTLP HTTP environment only when Jaeger was selected. `elesim-up --jaeger`
starts the profile; ordinary `elesim-up` does not force observability overhead.
`elesim-down` includes the profile when Jaeger is installed, so it stops the
Jaeger container together with the development container.

`elesim-status` is a read-only host-local report. It prints the host name and
host/runtime IPs, fixed container state, image, restart/OOM information,
CPU/memory counters, GPU visibility, and (for Sim) the logged Genesis backend,
H.264 encoder (`h264_nvenc` or `libx264`), camera streams and WebRTC transport.
It does not claim that a remote host is online; run it on each host or use
`elesim-connections` for the multi-host lifecycle.

The project and image are fixed as `elesim-runtime-dev` and `elesim/dev:local`.
The only persistent development container is `elesim-dev`; optional tracing
adds `elesim-jaeger`. The `elesim-dev` wrapper starts the persistent container
when necessary and enters it with Compose `exec`, so opening more terminals
does not create randomly named `run --rm` development containers.
`elesim-connections` uses an explicit, removable `elesim-manager` one-shot
container. A short-lived private host helper allows only the generated EleSim
Compose/network operations and optional `tailscale nc`; the manager receives
neither the Docker daemon socket nor the tailscaled local API socket. It is a
management tool, not another persistent development service. In Developer mode it targets the ordinary
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

The general Sim container is launched with the installing user's numeric
UID/GID. Viewer mode grants the same installing user's X11 ACL, rather than a
root ACL. Before returning launch success it also checks that the Sim image can
open an X11/GL context through the mounted Unix socket. This software preflight
does not replace the manual acceptance test that the native Genesis window is
visible and interactive on the intended Ubuntu/WSLg display. Its complete
runtime cache is mounted at `/tmp/elesim-cache` inside
the container and backed by the install's `cache` directory; Genesis uses its
`genesis` child and Quadrants/Numba use sibling children. This keeps runtime
cache writes compatible with a later normal-user `elesim-update` and does not
grant the application container host-root privileges. If an older installation
left that cache root-owned, update preserves it and switches to the managed
`<prefix>/.runtime-cache` subtree instead of deleting data.

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
On container and Developer installations, `elesim-down --purge` performs the
same runtime shutdown and then force-removes only the exact
`elesim-manager` container. It is an explicit interruption of a running
connection-manager job; the default `elesim-down` leaves that management
container alone. Images, caches, topology, security material and logs are not
purged by this option.
When no selected role container exists, `elesim-logs` and `--save` fail with an
actionable `elesim-up` message, while `elesim-down` is a successful no-op and
does not emit a Compose "No resource found" warning.
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

For `tailscale-sidecar`, the ownership manifest also records the exact fixed
container and the install-owned `<prefix>/secrets` root containing the
mode-0700 `<prefix>/secrets/tailscale` node-state directory. Normal down/update
preserves the directory; validated uninstall removes that install-owned root.
Because `tailscaled` may create mode-0700 children as its container user, the
host uninstaller first proves the exact install UUID/Compose service, pinned
image digest and sole read-write state bind. For a running sidecar it reuses the
container's already-established mount namespace. A no-follow host directory
descriptor and one random mount-identity token prove that the container bind
and the currently validated host path are the same directory. It then suspends
`tailscaled`, returns only that bind tree to the invoking host UID/GID, and
removes that exact sidecar as the final container mutation. It never resolves
the host bind again through a new helper mount. If normalization or sidecar
removal fails before that commit point, PID 1 is resumed (or the exact stopped
container is restarted) and the ownership manifest is retained.
The helper restores owner read/write and directory traversal bits after the
ownership change. Symlinks, special files, hard-linked regular files, a
foreign/additional mount, an unavailable image, or an inaccessible tree without
the owned running sidecar fail closed before filesystem removal; no broad host
deletion or Docker prune is used. A stopped/absent sidecar proceeds only when
the host can already traverse and remove the exact tree; otherwise start that
exact sidecar and rerun uninstall. An already absent state directory needs no
repair.
This local cleanup does not revoke the device record from the tailnet control
plane; remove the old node in the Tailscale admin console when decommissioning
it. The upstream `tailscale/tailscale` image is not install-owned and is never
globally pruned.

The GUI only validates the manifest and emits exact terminal commands; the
disposable loopback web process receives neither the Docker socket nor a host
deletion channel. The host CLI always plans and revalidates before mutation:

```bash
elesim-uninstall --plan
elesim-uninstall
```

If a Sim installation owns the generated `elesim-viewer-cleanup` wrapper, the
uninstaller verifies that wrapper's exact manifest path and SHA-256, stops the
exact validated Sim container first, and then invokes the wrapper with no
arguments before changing PATH, removing containers/images, or deleting install
files. Cleanup consumes every exact canonical, fixed-fallback, and bounded
legacy runtime record for that installation, so an update cannot orphan an ACL
created by an older wrapper. A remaining managed Viewer record without that
exact owned wrapper, or a cleanup failure against any saved display, aborts
further removal and preserves the ownership manifest so the ACL can be
recovered with `elesim-update` or the same installation's `elesim-down`. The
Sim container may remain stopped after such a failure; no similarly named
foreign command is run.

The default removes runtime text logs and the operator SROS2 Authority owned by
that installation. `--keep-logs` and `--keep-authority` are explicit retention
options. The completion tombstone lives outside the removed prefix under
`${XDG_STATE_HOME:-~/.local/state}/elesim/uninstall/`.
External source, TURN credentials and SROS2 keystores are always preserved.
Only exact `elesim/*:local` images and containers whose Compose metadata and
install UUID label match are eligible; there is no prune or wildcard deletion.
For native Robot, an installed or active systemd unit aborts the entire plan
before mutation and prints exact removal commands only when the unit path and
SHA-256 match the generated copy.

## DDS Network And Security

The generated DDS runtime profile contains:

- a ROS-safe `system_id` shared by one EleSim graph;
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

`sros2` enables DDS Security in enforce mode. State schema v9 retains the
provisioning distinction introduced in v8:

- `external`: the operator supplies and maintains a local keystore/base
  enclave. EleSim records no managed generation and does not rotate it;
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

Exactly one host is local; a host may own multiple roles. Schema-v1 topology
files are read as `full`; schema-v1-v3 files derive SSH from their historical
shared address and are normalized to schema v4. The ephemeral two-host
preflight contract similarly migrates v1 to v2.

Each host records one advertised DDS IP and interface for runtime UDP
reachability and static-peer derivation. A remote host separately records its
SSH destination, port, user, agent/identity-file choice, and pinned SHA-256 host
key fingerprint. The addresses often match in `direct-host`, but differ when a
Docker Desktop sidecar owns the DDS tailnet identity and SSH terminates at the
WSL/host identity. SSH port `2222` is an administration example only. Topology
state is non-secret: it may retain an identity-file path and host fingerprint,
but never a password, private key body, SROS2 key, TURN secret, Tailscale
auth/OAuth key, credential, or token.

When the physical Jetson is unavailable, select `simulation-only` and save the
active COM topology normally. The GUI's primary maintenance action is now
**Host check**: it checks every saved host's runtime network namespace, install
and SSH management path, and Compose/systemd lifecycle state in one read-only
job. This replaces the old split between the ephemeral two-host preflight and
the saved-topology host-status button. The `/api/preflight` contract remains
available for automation and focused Jetson-less tests, but is not an everyday
GUI action. Enter the current, mutable DDS address (hostname/IP only, no
`:port`) and interface (`tailscale0` is the usual Tailscale path name), then
enter the independent SSH destination, remote user, and port. In sidecar mode
the DDS address comes from `elesim-tailscale status`; it must not be replaced
with the WSL/host SSH address. Ordinary SSH over Tailscale uses the sshd port
(normally 22, unless that host was configured differently); the connection
manager does not invent a Tailscale or DDS port.
A temporary
`python3 -m http.server 8080` reachability check is outside this document and
must not be entered as a DDS or SSH port. Host check is not a proof of
bidirectional DDS, SROS2, RGBD, WebRTC, or NAT traversal. Only `full`
deployment requires the Robot role; `simulation-only` deployment intentionally
starts the three simulation roles without a physical Robot.

If a `tailscale*` interface is present in the selected runtime namespace, the
connection manager performs a read-only address probe and may prefill the
current IPv4 address/interface. This is only a convenience hint: it never
changes ACLs or hard-codes an address, and the operator must refresh the value
after the node identity changes. Sidecar enrollment remains the explicit
`elesim-tailscale login` action; repeating it re-authenticates stale local
state. A routed VPN is recommended for hosts
on different networks; use static discovery and remember that the automated
namespace/route probe is not a bidirectional DDS proof.

The setup wizard intentionally keeps manager-owned DDS/security/SSH fields out
of the normal interaction path. General installs start with a managed SROS2
pending marker; the operator then enters the mutable host addresses, Tailscale
interface, SSH mode/user and host key confirmation in `elesim-connections`.
The manager creates the SROS2 generation and role bundles itself, so an
operator never types an AES value or private key body into setup. Coturn is
owned by the Sim installation rather than represented as a connection-manager
card or topology field; the manager reads its non-secret runtime endpoint from
`elesim-net show` when SROS2 is active and clears it for trusted-network use.

The retired wizard boundary is now part of this document: setup emits only
manager-owned defaults, while `elesim-connections` owns mutable DDS/SSH/TURN
endpoints and managed SROS2 generation. Legacy request fields remain readable
for migration but are not exposed as a second installer configuration path.

For managed SROS2, provisioning creates role identities and per-host bundles;
deployment first preflights every host and stages the same generation on all of
them. Uploaded files are checked against their bundle SHA-256 values. The GUI
labels rotation **Key reissue**. It captures the exact running-role set, stops
only that set, switches each host's `security/current`, resumes existing
containers without building or recreating them, and verifies the generation. A
host that was stopped before provisioning remains stopped. A failure restores
the complete captured configuration, including empty managed-pending fields,
restarts only the roles that were previously running, and removes an inactive
failed generation. A bounded transaction journal remains under the operator
Authority. The internal recovery action converges an interrupted graph to the
Authority-active generation, or to managed-pending when no Authority
generation is active; ordinary GUI use exposes Abort and Host check rather than
a separate advanced recovery panel.

## TURN and ICE Ownership

The Sim application owns WebRTC ICE policy. Direct ICE candidates are always
attempted first; WebRTC remains DTLS/SRTP in both DDS security profiles.

- `trusted-network` uses direct ICE only. The generated Sim configuration has
  no TURN URL or credential source, and the bundled Coturn service is not
  started.
- `sros2` adds the managed Coturn fallback to the Sim Compose project. Sim
  mounts the static REST HMAC secret, issues short-lived credentials bound to
  the active UI session, and sends only the usable credential to UI over the
  authenticated DDS session. UI never receives the static secret.

Coturn is not a role, card, or saved connection-topology field. The
connection manager reads the managed endpoint from the Sim host's
`elesim-net show`, verifies the secret and service, and configures the Sim
runtime transactionally. Switching to trusted-network sends `--clear-turn` and
stops a stale managed relay. Legacy external TURN state remains readable by
the lower-level runtime for migration, but new setup requests do not expose
it.

TURN relays DTLS/SRTP WebRTC media only. It cannot make DDS discovery, topics,
services, actions, or SDP signaling reachable through NAT; the DDS path must be
reachable before Sim can exchange WebRTC offers.

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
  --turn-mode managed
```

`--dry-run` validates and prints the plan without writing runtime files.
With `--dds-security-provisioning external`, credentials must already be
provisioned. `managed` creates a pending, non-runnable General installation;
the connection manager derives the Sim relay endpoint from the saved topology,
then performs SSH preflight, Authority issuance and all-host rollout through
`elesim-connections` rather than inside installation.

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

The full-start action also performs one bounded, read-only application-level
readiness probe after detached launch. Host probes run concurrently under one
60-second deadline; multi-unit hosts share that same deadline. The probe
listens only for transient-local DDS endpoint descriptors and live volatile
heartbeats for every endpoint ID in the
saved topology, including roles co-located on the probing host. A descriptor
without a heartbeat is stale and does not count as readiness; missing
co-located or remote heartbeats fail the start and roll back launched roles.
To make this check strict and repeatable, pass endpoint IDs (not IP addresses
or SSH names):

```bash
elesim-net doctor --json --strict-peers --readiness-only \
  --expect-peer sim-default --timeout 60
```

Before build, stop, or launch, the connection manager also invokes the normal
no-override installed launch guard. It cross-checks each role YAML, CycloneDDS
XML, Compose security/DDS environment, canonical enclave, and role-private key
material. A stale hashed fallback enclave is rejected before runtime mutation.

An endpoint descriptor plus a live heartbeat proves only bounded application-level
DDS discovery/readiness. It does not
prove RGBD frame delivery, WebRTC ICE/DTLS-SRTP, SROS2 authorization, or
physical safety.

## Verification Boundary

Setup tests must cover:

- request and host capability validation;
- state migration and serialization;
- generated role/tools/developer contexts and nested Python packages;
- shell block idempotence;
- trusted-network acknowledgement, SROS2 role-scoped enclave validation and
  external/managed state validation, bundle digest/mode/path containment and
  overwrite protection;
- connection topology v1-v4/preflight v1-v2 migration, independent DDS/SSH
  destinations, pinned SSH host keys, all-host staging, activation and
  rollback;
- automatic-and-fixed `direct-host`/`tailscale-sidecar` backend selection,
  generated Compose namespace relationships, sidecar state ownership, login
  command shape, namespace interface/address/route checks, and absence of
  persisted Tailscale auth/OAuth keys;
- web asset packaging, token/API boundaries, path containment, and job states;
- bootstrap extraction and shell invocation;
- generated release infrastructure.

Source-tree imports are insufficient. Release verification must inspect the
copied setup package and built wheel, including both `web/` and
`connection_web/` assets and the CJK font.

No automated setup test establishes real SSH-agent behavior against a remote
host, multicast/static-peer behavior on the owning network, Docker Desktop
device enrollment or two-host DDS over the sidecar, SROS2 enforcement with the
production RMW, Wi-Fi/VPN reconnect, WSLg/X11 rendering, NVIDIA runtime access,
Jetson hardware support, Coturn relay selection across an actual NAT, or
RGBD/Genesis frame latency. Record those as explicit manual validation results.
