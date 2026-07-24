# Setup Wizard and Network Doctor

## Scope

`misc/tooling/setup` owns installation request validation, generated runtime
configuration, role-isolated build contexts, shell wrappers, credential
provisioning, and post-install network diagnosis. It does not own a deployment's
domain behavior and imports no Router, Controller, UI, Simulator, or Robot
implementation.

The browser wizard offers two editions:

- **General** installs a selected subset of the five runtime applications.
  Router, Simulator, Controller, and UI use role-isolated Docker images. Robot
  is a native-only, Jetson-detected, exclusive selection.
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
3. Downloads the standard-library `bootstrap.py`.
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

General mode translates to state schema v3 and one of two existing backends:

- Router/Simulator/Controller/UI generate a Linux host-network Compose project.
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

Each application context contains protocol plus one owned deployment only.
Source configuration is copied into installed runtime data and never edited.
The Simulator receives the immutable model bundle through a read-only mount.
The tools image contains protocol and setup/doctor, not deployment
implementations.

`network_mode: host` preserves the meaning of loopback and direct advertised
ports across the generated Linux containers. Prefix-derived Compose names and
image tags prevent separate installs from overwriting one another.

## Developer Installation

Developer mode requires Ubuntu/WSL amd64 and generates:

```text
<workspace>/
├── .git/                             # existing or cloned
├── router/ controller/ ui/ ...
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

## Curve Credentials

Non-loopback Curve mode requires one explicit source:

- **existing** validates role-required files under the selected local root;
- **generate** is allowed only on a Router host and invokes
  `misc/infra/bootstrap_security.py`;
- **ssh** downloads only role-required paths from the trusted administration
  host.

SSH mode supports an agent/default keys or one selected private key. It does
not accept or store passwords. The GUI first probes the server key and asks the
operator to confirm its SHA256 fingerprint. The transfer pins that same
fingerprint during authentication, rejects remote symlinks/non-regular files,
limits file count and size, stages downloads, and refuses to overwrite a
different existing credential.

Private keys and `turn.secret` are installed mode `0600`. Controller/UI
installations also receive `doctor-main.key_secret`; the complete credential
root and Router private key remain on the administration host.

## TURN Ownership

State schema v3 separates TURN endpoint URLs from relay ownership:

- `none`: no TURN URL;
- `external`: consume an independently managed relay;
- `managed`: include Coturn in the generated general-user Compose project.

Managed TURN requires Router, Curve security, realm, public host, one TURN URL,
and the generated/shared `turn.secret`. The Coturn service uses host networking,
a pinned image, a read-only secret mount, and UDP relay range `49160-49200`.
Because it is in the same Compose project, `elesim-up`, `elesim-down`, and
`elesim-logs` own its lifecycle. The standalone release Coturn Compose remains
available only for operators who deliberately choose `external`.

Schema-v1/v2 state loads as schema v3; existing TURN URLs migrate to
`external`, preserving previous external ownership semantics.

## Non-Interactive Installation

Automation can continue to use:

```bash
PYTHONPATH=packages/protocol/src:misc/tooling/setup/src \
python3 -m elesim_setup.cli \
  --source-root "$PWD" \
  install \
  --mode container \
  --profile compute \
  --router-host sim.example.com \
  --advertise-host sim.example.com \
  --gpu-mode inherit \
  --security curve \
  --credentials-root /etc/elesim/secrets \
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
  --router-host 192.168.0.10 \
  --advertise-host 192.168.0.30 \
  --security curve \
  --credentials-root /etc/elesim/secrets
```

This rewrites only installed role configuration. Restart roles afterward.
`router_host` is where every endpoint reaches Router; `advertise_host` is where
other machines reach this host's direct RGBD publisher. `0.0.0.0` is bind-only
and is never a valid advertised address.

## Network Doctor

```bash
elesim-net doctor
```

The non-disruptive pass checks:

- Router DNS/TCP reachability;
- an actual protocol-v4 `doctor-main` registration and discovery;
- advertised CurveZMQ RGBD syntax and TCP reachability;
- TURN UDP STUN Binding or TCP connectivity;
- Simulator advertisement of both DTLS-SRTP WebRTC streams.

The active pass consumes media:

```bash
elesim-net doctor --active --timeout 8
```

It receives one RGBD multipart message, opens a simulation session, exchanges
two aiortc offers/answers, and waits for `observer` and `hand_eye_preview`
frames. Stop the normal UI first because Simulator grants one UI simulation
session. A successful ICE connection does not by itself prove that a relay
candidate was selected.

Use `--json` for automation. Authentication failure, malformed protocol,
unreachable configured ports, or missing requested frames are `FAIL`. Optional
services that are intentionally absent are `SKIP` or `WARN`.

## Verification Boundary

Setup tests must cover:

- request and host capability validation;
- state migration and serialization;
- generated role/tools/developer contexts and nested Python packages;
- shell block idempotence;
- role-scoped credential manifests and overwrite protection;
- web asset packaging, token/API boundaries, path containment, and job states;
- bootstrap extraction and shell invocation;
- generated release infrastructure.

Source-tree imports are insufficient. Release verification must inspect the
copied setup package and built wheel, including `web/` and its CJK font.

No automated setup test establishes real SSH-agent behavior against a remote
host, WSLg/X11 rendering, NVIDIA runtime access, Jetson hardware support,
Coturn relay selection across an actual NAT, or Genesis frame latency. Record
those as explicit manual validation results.
