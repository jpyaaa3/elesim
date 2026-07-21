# Setup Wizard and Network Doctor

## Purpose

`elesim-setup` answers three separate questions explicitly:

1. Whether roles run in Docker containers or native role-isolated venvs.
2. Which deployable roles belong on this machine?
3. Which Router, media address, security policy and compute policy should those
   roles use?

Native mode installs every selected role into its own virtual environment.
Container mode generates one isolated image context per role and a Linux
host-network Compose project. Neither mode merges sibling deployments. Container
mode does not alter host Python or APT packages; the outer bootstrap only offers
to install Docker when Docker is absent and the user explicitly agrees.

## Clean Ubuntu Container Bootstrap

Use the shell bootstrap when the machine has no Elesim Python environment:

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/misc/setup/bootstrap.sh | bash
```

The script downloads only `bootstrap.py`, starts it inside a disposable
`python:3.10-slim` container as the current UID/GID, and mounts the user's home
at the same absolute path. The Python bootstrap then safely downloads and
extracts the selected Elesim source archive. The setup container never receives
the Docker socket. It writes build contexts, Compose YAML, generated role config
and wrappers; the host runs Docker only after the wizard exits.

Container mode uses `network_mode: host` intentionally. Loopback configurations
therefore keep the same meaning across Router, Controller, Simulator and UI,
while direct ZMQ/WebRTC ports are not hidden behind a second Docker NAT. This
mode targets Linux hosts. The installation-prefix hash namespaces the Compose
project and image tags, so a second install does not overwrite an existing
project or manually built `elesim-*` image.

Generated commands are:

```text
elesim-build       build selected role images
elesim-up          build and start selected roles
elesim-down        stop and remove role containers
elesim-logs        follow combined logs
elesim-<role>      start one selected role in the foreground
elesim-net         run the network doctor in the tools image
elesim-setup       inspect/reconfigure through the tools image
```

The first Simulator build downloads and exports the Genesis, Torch and
Pinocchio stack and can take several minutes with long quiet intervals. It is
also much larger than Router or UI. Subsequent starts reuse Docker layers unless
the lock files or image template changed.

The generic container backend deliberately rejects the Robot role. A physical
Jetson needs a JetPack/L4T-compatible base, ROS2 Humble, `unitree_ros2`, device
nodes and local safety permissions. Use native Robot installation until a
hardware-specific image is versioned and tested on the target Jetson.

The generated Simulator image currently targets `linux/amd64`. It uses Ubuntu
22.04/ROS2 Humble and the Robotpkg Pinocchio build under `/opt/openrobots`,
matching the validated development environment without mixing the current pip
Pinocchio dependency graph with the project's NumPy 1.x ABI. Controller and
Simulator images install a tested Torch pair explicitly; CPU policy selects the
official CPU-only wheels instead of downloading CUDA libraries.

## Bootstrap Without Git

The public bootstrap is standard-library-only:

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/misc/setup/bootstrap.py | python3 -
```

It performs these steps:

1. Download the selected GitHub source tar archive into
   `~/.cache/elesim/setup/sources`.
2. Reject absolute paths, parent traversal, links and device entries before
   extracting the archive.
3. Create a small bootstrap venv and install `elesim-setup` plus the exact
   protocol package from that archive.
4. Reopen `/dev/tty` and start the same interactive wizard used by a checkout.

`ELESIM_REPOSITORY`, `ELESIM_REF`, `ELESIM_ARCHIVE_URL` and
`ELESIM_CACHE_DIR` override download inputs. `--refresh` replaces the cached
source for that URL.

## Installed Layout

The default user installation is:

```text
~/.local/share/elesim/
├── install-state.json
├── tools/venv/
└── roles/
    ├── router/{venv,config}/
    ├── simulator/{venv,config,model}/
    ├── controller/{venv,config}/
    ├── ui/{venv,config}/
    └── robot/{venv,config,systemd}/

~/.local/bin/
├── elesim-setup
├── elesim-net
└── elesim-<installed-role>
```

Only selected role directories are created. `install-state.json` has mode
`0600` and stores endpoint addresses and credential paths, never CURVE private
key bytes or the TURN static secret.

The generated files are named `installed.yaml`, `runtime.installed.yaml` or,
for Simulator compute selection, `app.installed.yaml`. Checked-in source
defaults are never edited.

## Shared GPU Policy

Controller and Simulator installations have three explicit modes:

- `inherit` is the default. Wrappers do not assign `CUDA_VISIBLE_DEVICES`, so
  Slurm, a shell launcher or a container runtime remains authoritative.
- `specific` exports one index or GPU/MIG UUID selected from `nvidia-smi -L`.
  That physical device normally appears as logical `cuda:0` inside the
  process, so application settings should use `auto` or `0`.
- `cpu` exports an empty `CUDA_VISIBLE_DEVICES` and generates the Simulator
  application profile with `simulation.runtime.use_gpu: false`.

The installer deliberately does not choose the GPU with the most free memory:
that observation races with other laboratory jobs. Use the scheduler for
dynamic allocation and leave the installer in `inherit` mode. Inside Docker,
this policy can only further restrict devices already exposed by Compose; use
Compose `device_ids` or the scheduler to control host-level GPU access.

An inherited installation can change policy for one launch without
reinstalling:

```bash
# Generated Compose installation
CUDA_VISIBLE_DEVICES=1 elesim-up

# Native installation
CUDA_VISIBLE_DEVICES=1 elesim-simulator
CUDA_VISIBLE_DEVICES="" elesim-simulator --cpu
CUDA_VISIBLE_DEVICES="" elesim-controller
```

Container CPU mode is generated explicitly because hiding CUDA alone does not
change the Simulator's Genesis backend setting. Re-run the non-interactive
installer with `--mode container --gpu-mode cpu` when the container stack must
yield all GPUs.

The `--cpu` Simulator override changes the Genesis backend as well as hiding
CUDA. Controller automatically selects CPU when CUDA is hidden.

## Non-Interactive Installation

Automation can invoke the same implementation without prompts:

```bash
PYTHONPATH=packages/protocol/src:misc/tooling/setup/src \
python3 -m elesim_setup.cli \
  --source-root "$PWD" \
  install \
  --profile compute \
  --router-host sim.example.com \
  --advertise-host sim.example.com \
  --gpu-mode inherit \
  --security curve \
  --credentials-root /etc/elesim/secrets \
  --turn-url 'turn:sim.example.com:3478?transport=udp'
```

Use `--dry-run` to validate the source, role selection and security inputs and
print the plan without writing files or invoking pip.

Pin one device with `--gpu-mode specific --gpu-device GPU-...`, or yield every
GPU with `--gpu-mode cpu`. Add `--mode container` to generate the Compose
installation instead of host venvs.

## Address Changes

Run the interactive editor:

```bash
elesim-net configure
```

Or make a scripted change:

```bash
elesim-net configure --non-interactive \
  --router-host 192.168.0.10 \
  --advertise-host 192.168.0.30 \
  --security curve \
  --credentials-root /etc/elesim/secrets
```

This rewrites only installed role configuration. Restart running processes to
load the new values.

`router_host` is the address every endpoint uses to reach Router.
`advertise_host` is the address other machines use to reach this host's direct
RGBD publisher. `0.0.0.0` is generated only as a local bind address; it is
never advertised.

The insecure-LAN mode is a development exception. The wizard does not combine
it with a Router that issues TURN credentials; that topology requires a CURVE
credential root containing the shared TURN static secret.

## Diagnostic Layers

```bash
elesim-net doctor
```

The default, non-disruptive pass checks:

- Router hostname resolution and TCP 5558 reachability.
- A real protocol-v4 endpoint registration as `doctor-main` and discovery.
- Advertised ZMQ RGBD endpoint syntax and TCP reachability.
- TURN URL parsing and either UDP STUN Binding or TCP connectivity.
- Simulator advertisement of both DTLS-SRTP WebRTC streams.

The active pass additionally consumes media:

```bash
elesim-net doctor --active --timeout 8
```

It receives one valid ZMQ RGBD multipart message, opens a protocol simulation
session, exchanges two aiortc offers/answers and waits for frames from both
`observer` and `hand_eye_preview`. Stop the ordinary UI first because a
Simulator accepts one UI simulation session at a time. A successful ICE
connection proves the configured candidates work, but does not by itself
prove that TURN relay rather than a direct candidate was selected.

The wizard installs aiortc/av into its tool venv only on UI or Simulator
hosts. A Robot-only installation retains DNS, TCP, CURVE, discovery, RGBD and
TURN probes but reports that the optional WebRTC dependency is absent if an
active WebRTC test is requested there.

Exit code is zero when there are no `FAIL` results. Missing optional TURN or a
Simulator that was intentionally not started is shown as `SKIP` or `WARN`;
authentication failure, malformed protocol, unreachable configured ports or
missing active frames are `FAIL`.

Use JSON for automation:

```bash
elesim-net doctor --json
```

## CURVE Credentials

New security bundles include `doctor-main` as a UI-role Router identity. It is
not copied to `curve/media-authorized`; an active RGBD check deliberately uses
Controller's media client key, matching production access control.

Run credential generation on one trusted administration host and distribute
only the required private files. If a bundle predates `doctor-main`, generate
a coordinated replacement bundle and redistribute it; do not copy the entire
private tree to every machine merely to satisfy the doctor.

## Boundaries

The setup package imports `elesim_protocol` but no deployable application.
Network tests interact with Router, Simulator and media publishers only through
their public wire contracts. This keeps the installer useful even when all
five releases are deployed on different machines.

Container build contexts are copied, not linked: each contains only protocol,
one selected deployment and its direct lock file. The tools image contains
protocol plus setup/doctor only. Simulator receives the immutable prebuilt model
bundle through a read-only mount and uses the remote/headless application profile
even for a one-machine container stack; the UI observer stream replaces the
native Genesis desktop window.
