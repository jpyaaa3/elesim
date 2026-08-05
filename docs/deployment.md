# Deployment

## Build Release Contexts

From the repository root:

```bash
python3 misc/tools/release/build.py
```

This writes one self-contained build context per role under
`dist/releases/<role>`. Each context contains its application wheel, the
transport-neutral Python support wheel, ROSIDL source at
`interfaces/elesim_interfaces/`, configuration, direct dependency pins and
deployment metadata.
The sim context additionally contains the prebuilt model bundle.
`dist/releases/infra` contains the General/development container inputs, the
standard-library download bootstrap, and the independently installable
setup/doctor/connection-manager package. These are infrastructure, not a fifth
runtime deployment. The source-only `environment/coturn` project remains an
external-relay operator input and is not copied into `dist/releases`.

The build command verifies every generated context by default. It performs a
clean `--no-deps` temporary install, checks wheel ownership, parses the shipped
configuration, validates the sim model bundle, and runs each role's
primary installed entry point with `--help`. For Robot it also requires both
console-script declarations, all bridge/IPC modules, and exactly
`elesim-robot.service` plus `elesim-unitree-bridge.service`. To re-run only this
verification:

```bash
python3 misc/tools/release/verify.py dist/releases
```

`--no-verify` exists only for diagnosing an incomplete build; an artifact made
with that option has not passed the release gate.

## Reproducible Multi-Host Bootstrap

When multiple hosts must install the same source snapshot, use the same full
40-character commit SHA in both the raw bootstrap URL and `ELESIM_REF`. Run the
same command on each owning host, replacing the example SHA with the selected
commit:

```bash
# [each Sim/Pilot/UI host]
elesim_commit=0123456789abcdef0123456789abcdef01234567
curl -fsSL \
  "https://raw.githubusercontent.com/jpyaaa3/elesim/${elesim_commit}/installer/bootstrap/bootstrap.sh" \
  | ELESIM_REF="$elesim_commit" bash
```

A branch or tag is intentionally treated as mutable and checked for freshness
on every invocation; it can therefore move between hosts. A full commit SHA
selects an immutable cached snapshot and makes the source revision printed by
each bootstrap directly comparable. GitHub likewise recommends commit-ID
archives when the extracted contents must remain stable; see
[Downloading source code archives](https://docs.github.com/en/repositories/working-with-files/using-files/downloading-source-code-archives).

## DDS Multi-Host Profiles

All participants in one graph must use the same `system_id`, `domain_id`,
pinned RMW implementation and compatible QoS. Each host separately selects the
LAN/VPN interface on which peers can reach it.

Use multicast discovery on one L2 network. Use explicit, directly reachable
static peer addresses on a routed LAN or VPN where multicast is not forwarded.
Static peers are discovery seeds only; application samples still travel
directly over UDP.

Choose one security profile:

- `trusted-network` uses DDS without encryption. Restrict it to an owned LAN or
  routed VPN, bind the intended interface, and use host/network firewalls to
  exclude untrusted machines.
- `sros2` enables DDS Security authentication, access control and encryption in
  enforce mode. Install only each deployment's enclave on that host. Keep the
  certificate authority and unrelated role private keys off runtime hosts.

For SROS2, choose one provisioning owner. `external` means the operator supplies
and rotates the local keystore/enclave outside Elesim. `managed` means
`elesim-connections` on the operator laptop owns the Authority generation and
deploys a common-public-plus-assigned-enclaves bundle to each host. Never copy
the Authority's `private/` tree to a runtime host.

An initial General installation may select `managed` before any key exists.
That host is deliberately pending: setup writes its runtime/Compose artifacts
and `<install-root>/security/provisioning-required`, while `elesim-up` and role
launchers refuse to start. After every role host has been installed, run
`elesim-connections` on the operator laptop and activate one generation across
all hosts. The configuration transaction removes the marker only after a
runnable managed bundle (or an explicit trusted-network configuration) is in
place; rollback restores the prior marker. External SROS2 installation retains
the existing requirement for a ready local keystore and enclave.

`ROS_DOMAIN_ID` is not authentication, authorization or isolation. Do not use a
domain number as the sole protection on shared compute.

For `trusted-network`, select an interface such as `wg0` in every host's
generated DDS vendor profile and restrict that interface to the VPN peers or
trusted subnet. Do not expose DDS on a public/default interface merely because
the domain ID is uncommon.

For `sros2`, the generated launcher must apply the equivalent of:

```bash
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_KEYSTORE=/etc/elesim/keystore
exec <role-command> --ros-args --enclave <configured-role-enclave>
```

The UI enclave may publish to Pilot operator and Sim
session/signaling control topics and subscribe to their replies/state.
Pilot may publish fenced motion and subscribe to telemetry/RGBD. Robot and
Sim may consume only addressed control/motion traffic and publish only
their own replies, telemetry, and media. Generate permissions from this
least-privilege matrix; a keystore that gives every role wildcard
publish/subscribe rights defeats the profile.

Ordinary IPv4 NAT, CGNAT and symmetric NAT are unsupported by this peer-to-peer
topology. Port forwarding on one compute host alone does not make another
host's DDS locators reachable. Use a routed VPN or mutually reachable global
IPv6. Neither static peers nor TURN change this limit.

### Connection-managed rollout

`elesim-connections` keeps a non-secret topology on the operator laptop. Select
one explicit mode before assigning roles:

- `full`: assign Pilot, Sim, UI, and Robot exactly once across two to
  four hosts; Robot is native/Jetson/systemd-only.
- `simulation-only`: assign Pilot, Sim, and UI exactly once across
  one to three container/Compose hosts; no Robot or Jetson host is created.

Both modes mark exactly one host local. For each host, record a DDS
address/interface independently from its optional SSH management
hostname/port/user/authentication mode/pinned SHA-256 host fingerprint. OpenSSH
uses an agent or a selected identity path; Tailscale SSH is keyless and fixed to
port 22. If the Tailscale ACL uses `action: check`, approve one interactive
Tailscale SSH re-authentication before starting a manager job. Static discovery
peers come from DDS addresses, never SSH values. Schema-v1 topology files are
loaded as `full` and saved in schema v3 with the explicit mode.

While the physical Robot host is unavailable, the connection-manager GUI offers
an ephemeral two-host preflight for exactly two active COM cards. It accepts the
current mutable DDS hostname/IP without a port, the selected interface (for
example `tailscale0`), and the remote SSH management host/user/port. For a
Tailscale SSH endpoint the port is fixed at 22; ordinary OpenSSH uses the
configured sshd port. It does not save a topology, provision keys, or claim that an
SSH host-key probe proves DDS, RGBD, WebRTC, SROS2, or NAT traversal. An HTTP
reachability test such as `python3 -m http.server 8080` is outside the runtime
topology and must not be entered as a DDS/SSH endpoint. Only `full` deployment
requires the Robot role; `simulation-only` deployment intentionally starts the
three simulation roles without a physical Robot.

The same GUI also exposes explicit host-lifecycle actions: `check` is a
read-only per-host Compose/systemd query, while `start`, `stop`, and `restart`
run the existing pinned local/SSH lifecycle commands. Their badges describe
management reachability and process state only; DDS discovery and WebRTC media
are not inferred from a successful SSH command. The UI polls the read-only
status while open and keeps deployment and rotation jobs separate from that poll.

Managed SROS2 rotation creates a complete new generation through the ROS 2
security CLI. Per-host manifests bind the system, host, generation, assigned
enclaves and SHA-256 file digests. Deployment performs all-host preflight and
staging before stopping roles, atomically switches each host's
`<install-root>/security/current`, restarts and verifies the matching generation.
If any phase fails, hosts already touched restore their captured configuration
and their previous role views. Applications do not mount `current` or the
aggregate generation keystore. They mount only
`<install-root>/security/roles/<role>`; activation replaces that stable root's
`public/` and `enclaves/` children while the application is stopped.
Offline hosts therefore stop the rotation before a mixed live graph is created.

Coturn is optional on a flat LAN. The setup GUI's **managed** option places a
pinned Coturn service in the Sim host's generated Compose project. In
that case `elesim-up`, `elesim-down`, and `elesim-logs` manage Sim and
Coturn together. Managed TURN requires the SROS2 profile because ICE
credentials and signaling cross DDS.

The standalone release Compose is for an **external** relay that is deliberately
operated outside the generated Elesim project:

```bash
docker compose \
  --env-file "$HOME/.local/share/elesim/coturn/.env" \
  -f environment/coturn/compose.yaml up -d
```

The four application release contexts intentionally do not copy this standalone
Coturn project. Managed Coturn is generated into the Sim host's runtime
Compose project; an external relay remains independently operated from the
source infrastructure directory above.

Managed Coturn's REST HMAC secret is mounted into Coturn and the co-located
Sim only. Sim issues bounded-lifetime credentials for itself and
the UI, tied to the active simulation session; UI never receives the static
secret. This explicitly trusts the managed Sim to mint TURN credentials.
External TURN uses a separately provisioned JSON file with `username`,
`credential`, and optional `expires_at`. Select it only while installing the
Sim host. Setup validates it as a small regular private file and mounts
it read-only into Sim; Pilot/UI-only hosts keep only the TURN URL.
Sim passes the usable credential to its active UI through the DDS
session grant. TURN relays DTLS/SRTP WebRTC media, not DDS data or signaling.
If that DDS exchange is on a shared network, select SROS2 rather than
`trusted-network`.

## Container Roles

For a clean Ubuntu host, prefer the setup-generated Compose project over manual
role builds:

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/installer/bootstrap/bootstrap.sh | bash
elesim-up
```

The curl command opens a Korean/English loopback-only browser wizard. It
generates files but does not build or start images; `elesim-up` is the first
Docker build. The default prefix is the directory in which the curl command was
started, not a global system directory.

The generated project uses Linux host networking, read-only role configuration
mounts, a read-only Sim model mount, and a profile-gated tools container.
CPU policy omits GPU requests. `inherit` and `specific` request exposed NVIDIA
devices, so the host must have NVIDIA Container Toolkit in addition to its GPU
driver. UI defaults to Mesa software GL inside the X11 container; set
`ELESIM_UI_SOFTWARE_GL=0` only after validating host/container graphics driver
compatibility.

The Compose project name is fixed as `elesim-runtime`. Selected long-running
containers are `elesim-pilot`, `elesim-ui`, and `elesim-sim`, using
images `elesim/<role>:local`; managed TURN adds `elesim-coturn`. Robot is not a
generic container. Because names are fixed, a second general installation on
the same host is rejected rather than assigned an opaque hash-derived name.
The source trees, role keys, package names, application commands and image tags
all use `pilot`/`sim` directly. The old `controller`/`simulator` names are only
accepted at legacy state/topology input boundaries and during cleanup of old
containers.

Build a role by loading the generated wheel names:

```bash
cd dist/releases/sim
set -a
. ./WHEELS.env
set +a
docker build \
  --build-arg PROTOCOL_WHEEL="$PROTOCOL_WHEEL" \
  --build-arg APP_WHEEL="$APP_WHEEL" \
  -t elesim/sim:local .
```

Use the same command in `pilot` or `ui`, changing only the image tag.
The Dockerfile copies `interfaces/elesim_interfaces/` into a temporary colcon
workspace and builds the ROSIDL package; it is not a wheel argument. Runtime DDS
settings are supplied through the role's YAML or CLI arguments. UI containers
also need the host display/GPU mounts appropriate to the workstation.

On shared GPU hosts, prefer scheduler-assigned devices. The installer default
does not overwrite `CUDA_VISIBLE_DEVICES`. If Compose must pin a physical GPU,
choose `specific`; the generated reservation contains one `device_ids` entry
and does not expose the other devices. An in-container selector cannot grant
access to a GPU the container runtime did not expose. CPU-only installs do not
require a GPU reservation.

## Development Container

The GUI's Developer edition creates one complete Git workspace and a privileged
development Compose project under `<workspace>/.elesim/development`. It is not a
fifth runtime deployment and must not be used as a production multi-host
artifact.

An existing checkout is reused without pull/reset; an empty target is cloned at
the selected ref. The image includes all role dependencies and uses a persistent
`$HOME/.venv` for editable workspace installs. `elesim-up` starts the development
container and `elesim-dev` opens a shell. Optional Jaeger is profile-gated and
starts only through `elesim-jaeger-up`.

The project is `elesim-runtime-dev`, its image is `elesim/dev:local`, and its
only persistent coding container is `elesim-dev`; optional tracing adds the
separate `elesim-jaeger`. The `elesim-dev` command uses Compose `exec` into that
container, so repeated shells do not create randomly named temporary
containers. It contains the project-owned ROS/scientific test stack and is the
canonical replacement for external personal development Compose environments.
The connection GUI runs, when requested, in a removable `elesim-manager`
one-shot container. That tool alone receives the Docker socket; it does not
become a fifth persistent development/runtime application.

This mode mounts `/dev`, uses host network/IPC, and is privileged. Use it only
on an owned Ubuntu/WSL amd64 workstation. WSLg mounts are generated only when
the outer bootstrap detected WSLg on the host.

## Per-Host Ownership And Removal

Every installed machine owns and removes only its local role selection. Run
that machine's `elesim-uninstall --plan` locally; neither
`elesim-connections` nor SSH forwarding is a fleet-wide deletion mechanism.
The manifest binds the install UUID to exact Compose metadata, local image
labels, wrapper hashes and—on Robot—both generated systemd unit hashes. A
foreign container, modified wrapper, nested mount or same-name foreign unit
aborts before any mutation.

Text log snapshots and the operator laptop's SROS2 Authority are preserved by
default. Runtime configuration, managed host role keys and generated secrets
are removed. External credentials/keystores and Developer source checkout are
always outside the deletion boundary. Label commands with the host that owns
them and never substitute a remote prefix into a laptop uninstaller command.

## Remote Compute Server

A headless compute host normally runs Sim. Use
`config.remote.yaml`: it disables the native Genesis desktop Viewer while
keeping the observer and hand-eye render cameras enabled.

```bash
elesim-sim \
  --config /opt/elesim/sim/config/config.remote.yaml \
  --runtime-config /etc/elesim/sim.dds.yaml \
  --model-bundle /opt/elesim/sim/model/bundles/default
```

The laptop runs Pilot and UI with the same system/domain/RMW/security
profile. Each host's DDS interface and static peers are host-specific:

```bash
elesim-pilot \
  --config /opt/elesim/pilot/config/default.yaml \
  --runtime-config /etc/elesim/pilot.dds.yaml

elesim-ui --config /etc/elesim/ui.dds.yaml
```

The remote UI receives two WebRTC streams: the operator observer and the
hand-eye preview. It controls orbit, pan, zoom, pause/resume, single-step,
reset, speed and marker visibility through ROS 2/DDS control messages. The native
Genesis Viewer window is not transported.

The selected DDS interface needs bidirectional UDP permitted for the configured
RMW/domain participant and user-data ports. When Coturn is used, permit TCP/UDP
`3478` plus UDP `49160-49200`. A direct WebRTC LAN path may also use
dynamically selected ICE UDP ports. Pin and document the vendor's DDS port
mapping before writing firewall rules; do not open an undocumented broad UDP
range on shared infrastructure.

The setup GUI itself remains on `127.0.0.1`; administer a remote installation
through an SSH local-forward instead of opening the GUI port in the firewall.
SSH's port is unrelated to DDS, RGBD, or TURN ports.

## Robot Jetson

For a connection-manager-owned topology, run the setup wizard on the Jetson and
select **Robot only**. Setup generates `elesim-robot.service` and
`elesim-unitree-bridge.service` and prints exact account/group, ACL, unit
registration and enable commands without running `sudo`. The Robot unit runs as
the invoking account and owns the Elesim DDS/SROS2 participant, hardware and
local safety. The bridge runs as the dedicated `elesim-unitree` account and
owns only stock local/plaintext Unitree DDS.

Bind the bridge to the private Jetson-to-GO2 physical NIC and a domain distinct
from the Elesim graph. Bind Elesim DDS to its LAN/VPN interface. The two
interfaces must differ; never expose Unitree topics on Tailscale or the shared
lab LAN. The only process boundary between them is the credential-checked,
bounded `/run/elesim-unitree/bridge.sock`. A disconnect, malformed packet or
keepalive expiry commands a GO2 stop within the configured deadman/tick bound,
while Robot continues the arm safe-hold/torque-off path independently.

The generated wrappers preserve saved DDS configuration, role-scoped SROS2
keys, provisioning guards and later rotations. The bridge wrapper does not
receive the Elesim SROS2 enclave and Unitree topics are absent from the Elesim
security policy. Set `UNITREE_ROS2_WS` or `ELESIM_UNITREE_ROS2_WS` before
bootstrap if the workspace is not `$HOME/ros2_ws`. Override the private-link
defaults (`eth0`, domain `1`) with `ELESIM_UNITREE_INTERFACE` and
`ELESIM_UNITREE_DOMAIN_ID` before bootstrap when required.

`dist/releases/robot` is a separate standalone/manual artifact. Its fixed
`/opt/elesim-robot` unit and `/etc/elesim/robot.yaml` layout do **not** expose an
`install-state.json`, `elesim-net`, stable role-key views, or the managed
connection-manager lifecycle. Do not register that release tree as a managed
Robot host. If a deliberately standalone, non-managed deployment is required,
its legacy layout is:

```bash
cd /opt/elesim-robot
bash install.sh
```

The script builds the overlay and venv, verifies both service artifacts, and
then prints the remaining administrator prerequisites. It does not create the
`elesim`/`elesim-unitree` accounts or group, copy `/etc/elesim/robot.yaml`,
register either unit, or start systemd. Complete all printed prerequisites and
make the config's IPC users, Unitree workspace, private interface and domain
match the two fixed units before enabling `elesim-robot.service`.

The standalone Jetson artifact contains no UI, pilot workflow, Genesis,
source assets or model builder. Adapting it to a managed topology is future
release-packaging work, not an automatic setup fallback.

## Development Model Rebuild

Normal sim startup uses `model/bundles/default`. Rebuild it only while
editing geometry:

```bash
PYTHONPATH=pilot/src:model/builder/src \
  python3 -m elesim_model_builder.cli
```

Production sim artifacts must use the checked prebuilt bundle.
