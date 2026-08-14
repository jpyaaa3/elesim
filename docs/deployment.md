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
The Docker Desktop `tailscale` Compose service is likewise generated host
network infrastructure. It uses a pinned upstream Tailscale image and does not
create a fifth EleSim release tree or application wheel.

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
  "https://raw.githubusercontent.com/jpyaaa3/elesim/${elesim_commit}/installer/bootstrap/install.sh" \
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
and rotates the local keystore/enclave outside EleSim. `managed` means
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

Both modes mark exactly one host local. For each host, record one advertised
DDS IP/interface and an independent SSH management destination. OpenSSH uses
an agent or a selected identity path; Tailscale SSH is keyless and fixed to
port 22. If the Tailscale ACL uses `action: check`, approve one interactive
Tailscale SSH re-authentication before starting a manager job. Static discovery
peers come only from DDS addresses. Schema-v1 topology files are loaded as
`full`; schema-v1-v3 records derive SSH from their historical shared address
and are saved as schema v4. The ephemeral preflight contract similarly
migrates schema v1 to v2.

While the physical Robot host is unavailable, the connection-manager GUI offers
an ephemeral two-host preflight for exactly two active COM cards. It accepts the
current mutable advertised IP without a port, the selected interface (for
example `tailscale0`), and the independent remote SSH address/user/port. For a
Tailscale SSH endpoint the port is fixed at 22; ordinary OpenSSH uses the
configured sshd port. It does not save a topology, provision keys, or claim
that an SSH host key probe proves DDS, RGBD, WebRTC, SROS2, or NAT traversal. An HTTP
reachability test such as `python3 -m http.server 8080` is outside the runtime
topology and must not be entered as a DDS/SSH endpoint. Only `full` deployment
requires the Robot role; `simulation-only` deployment intentionally starts the
three simulation roles without a physical Robot.

### One computer, multiple deployment units

The topology treats a computer as a host client, not as a single-role box. A
host may carry more than one independent deployment unit. The normal Jetson
mixed layout is a native `robot-native` unit with the mandatory Robot service
and a separate `runtime` Compose unit for validated container roles (currently
Pilot/UI; Sim remains subject to the ARM64 gate).
Those units have independent prefixes, ownership manifests, security role
views, build contexts, and lifecycle commands; stopping or updating one does
not adopt or delete the other. A Robot assignment is valid only when its unit
is native/systemd and the host is marked as a Jetson. A host marked as Jetson
must include that mandatory native Robot unit; its additional Pilot/UI roles
remain a separate validated container unit. The connection manager serializes these units in its existing
topology state and reads older one-unit files as a single `runtime` or
`robot-native` unit.

Container installation resolves and persists one network backend. `direct-host`
(**Native host network**) uses the selected Docker Engine's host namespace.
`tailscale-sidecar` (**Docker Desktop Tailscale sidecar**) runs the fixed
`elesim-tailscale` container and places roles, the dedicated runtime-network
doctor, and active Sim-owned Coturn in the `tailscale` service's namespace. The
ordinary administrative tools service remains usable before enrollment.
Docker Desktop cannot inherit the WSL distribution's existing `tailscale0`;
the kernel-mode sidecar creates its own interface and tailnet IP inside the
Docker VM. This service is host network infrastructure, not an EleSim
application, Router, broker, or DDS relay. Its privileged upstream image is
version-and-digest pinned.

Enroll a generated sidecar with `elesim-tailscale login` and inspect it
with `elesim-tailscale status`. Login uses a browser/device flow; an explicit
repeat of `login` re-authenticates a stale `Running` node. Runtime launch uses
an idempotent internal check and does not open a browser. EleSim stores
no Tailscale auth/OAuth key or browser credential; only the enrolled node state
persists across ordinary down/up/update. Use the sidecar IP as the DDS address
and the WSL/host IP as SSH management address when they differ. Routed
Tailscale graphs use static discovery only.

SSH reachability is not DDS reachability. Before managed security material is
issued, and again before `start` or `restart`, every container host runs the
lightweight `elesim-net namespace-check` through its installed `runtime-tools`
service.
The check requires the configured DDS interface to exist in the same network
namespace used by runtime roles, requires the advertised address to be assigned
to that interface, and, for static discovery, checks each DDS peer route. It
remains a read-only bind/route gate; it does not prove discovery, SROS2
authorization, RGB-D, or WebRTC media. The Tailscale SSH helper still cannot
relay DDS UDP; the sidecar path works by sharing the enrolled namespace rather
than tunneling DDS through SSH.
Management SSH and DDS remain separate paths even when an operator chooses the
same Tailscale address for both. Runtime preflight therefore does not require
the DDS namespace to reach SSH port 22. The manager validates its SSH
connection independently; DDS readiness is decided by interface/address/route
checks followed by live endpoint heartbeats. The generated tools image carries
`iproute2`, and the launch guard rejects stale state/XML/Compose DDS values.

The same GUI also exposes explicit host-lifecycle actions: `check` is a
read-only per-host Compose/systemd query, while `start`, `stop`, and `restart`
run the existing pinned local/SSH lifecycle commands. A user-requested full
start first builds every selected host and only then launches roles with
`--no-build`. Those builds use Compose plain-progress mode and stream the
actual BuildKit stdout/stderr, labelled by host, into both the browser job log
and the terminal that launched `elesim-connections`. This output is not
available through `docker logs` because no role container exists yet, and
`docker events` is not used as a progress substitute. Local streaming crosses
the private allowlisted host helper; remote streaming uses the pinned SSH
session. Neither path gives the manager a Docker or Tailscale daemon socket.
After all images are built, the detached Compose lifecycle step uses a bounded
five-minute command timeout. This is separate from the thirty-minute image
build limit, so a slow Docker backend does not make `up --no-build` appear to
be a failed security rollout and trigger an avoidable rollback.
The final launch uses the installed `elesim-up --no-build` wrapper rather than
bypassing it with a raw Compose invocation. A checked Viewer option therefore
uses the wrapper's real `--view` path, including `DISPLAY` validation and the
temporary bounded `xhost` grant. For a non-interactive connection-manager SSH
launch, the wrapper accepts the invoking user's current display only when it is
a local `:<n>` display backed by an actual `/tmp/.X11-unix/X<n>` socket. It
also probes at most 16 such sockets and the user's normal or GDM Xauthority
paths; SSH-forwarded/TCP displays are not container-reachable and are rejected.
After the host-side `xhost` check, a one-shot Sim container using the installed
image and UID/GID must open a hidden X11/GL context before detached startup.
The normal Sim entrypoint repeats that preflight before DDS is initialized. An
explicit manager Stop and a failed full Start revoke the EleSim-owned grant
after Sim is stopped; security rotation's temporary stop/start deliberately
keeps it because the same Viewer container resumes. The optional GPU number is
passed through the same wrapper as a one-shot `CUDA_VISIBLE_DEVICES` value.
Security deployment and rotation never build or recreate
containers; they resume exactly the role containers that were running before
the switch. Their badges describe
management reachability and process state only; DDS discovery and WebRTC media
are not inferred from a successful SSH command. The UI polls the read-only
status while open and keeps deployment and rotation jobs separate from that poll.

Before lifecycle mutation, the manager runs each installed unit's normal
no-override launch guard. The guard rejects disagreement among role YAML,
CycloneDDS XML, Compose DDS/security environment, canonical SROS2 enclave, and
role-private key material.

After launch, the manager performs one bounded read-only DDS endpoint
descriptor/heartbeat probe for every active endpoint from each host. Host
probes run concurrently and each host's units share one 60-second deadline. A
transient-local descriptor can survive a dead process, so readiness requires a
live volatile heartbeat as well. A missing co-located or remote heartbeat
fails the start and rolls back launched roles; Docker Desktop/WSL namespace
isolation and broken DDS UDP paths are reported as actionable readiness
failures. Use the following command for the same strict application-level
discovery gate:

```bash
elesim-net doctor --strict-peers --readiness-only \
  --expect-peer <endpoint-id> --timeout 60
```

This still does not prove
RGBD, WebRTC, SROS2 authorization, or physical safety.

Generated-Compose assertions, schema migrations, login command shape,
namespace/interface/address checks, and secret-absence scans are automated
software gates. A real Docker Desktop/WSL host and a second tailnet host remain
a manual acceptance gate: confirm sidecar enrollment, static-peer descriptor
discovery, bidirectional control/RGBD, reconnect, and then both WebRTC streams.

Managed SROS2 rotation creates a complete new generation through the ROS 2
security CLI. Per-host manifests bind the system, host, generation, assigned
enclaves and SHA-256 file digests. Deployment performs all-host preflight,
digest-verified staging, and captures the exact running-role set before stopping
roles. It switches each host's
`<install-root>/security/current`, restarts and verifies the matching generation.
If any phase fails, hosts already touched restore their captured configuration
and their previous role views; inactive failed generation directories are
removed. Empty managed-pending fields are restored as empty rather than being
retained from a failed generation. A private transaction journal under the
operator Authority records the last phase, and the explicit recovery action
converges interrupted hosts to the Authority-active generation or managed-pending.
Applications do not mount `current` or the
aggregate generation keystore. They mount only
`<install-root>/security/roles/<role>`; activation replaces that stable root's
`public/` and `enclaves/` children while the application is stopped.
Offline hosts therefore stop the rotation before a mixed live graph is created.

Sim always attempts direct ICE first. A `trusted-network` (plaintext DDS)
installation therefore has no managed Coturn service and passes an empty ICE
server list to aiortc; WebRTC media is still DTLS/SRTP. An `sros2` Sim
installation includes the pinned, Sim-owned Coturn service as the relay
fallback. `elesim-up`, `elesim-down`, and `elesim-logs` own that service with
the Sim container. Pilot/UI-only installs do not receive a Coturn service.
This coupling keeps relay credentials and DDS signaling inside the SROS2 trust
boundary.

The standalone release Compose remains a compatibility tool for an independently
operated relay; the setup wizard does not offer this path for new Sim installs:

```bash
docker compose \
  --env-file "$HOME/.local/share/elesim/coturn/.env" \
  -f environment/coturn/compose.yaml up -d
```

The four application release contexts intentionally do not copy this standalone
Coturn project. Managed Coturn is generated into the Sim host's runtime Compose
project. Existing external states can still be inspected by the lower-level
runtime, but the installer cannot create a new one.

Managed Coturn's REST HMAC secret is mounted into Coturn and the co-located
Sim only. Sim issues bounded-lifetime credentials for itself and
the UI, tied to the active simulation session; UI never receives the static
secret. This explicitly trusts the managed Sim to mint TURN credentials.
Legacy external TURN states may use a separately provisioned JSON file with
`username`, `credential`, and optional `expires_at`; new installer requests do
not accept that file. When such a legacy state is explicitly retained, setup
mounts it read-only into Sim; Pilot/UI-only hosts keep only the TURN URL.
Sim passes the usable credential to its active UI through the DDS
session grant. TURN relays DTLS/SRTP WebRTC media, not DDS data or signaling.
If that DDS exchange is on a shared network, select SROS2 rather than
`trusted-network`.

Coturn is not a fifth role and is not a field in the saved connection topology.
The Sim installation is its owner. During a managed SROS2 transaction the
connection manager reads the non-secret TURN endpoint and secret-file path
from that host's `elesim-net show`, verifies the secret and Compose service,
then invokes `elesim-net configure --turn-mode managed ...`. For
`trusted-network`, it sends `--turn-mode none --clear-turn` and stops any stale
Coturn service. Verification compares the active Sim TURN state and running
service before the job can commit.

## Container Roles

For a clean Ubuntu host, prefer the setup-generated Compose project over manual
role builds:

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/installer/bootstrap/install.sh | bash
elesim-up
```

The curl command opens a Korean/English loopback-only browser wizard. It
generates files but does not build or start images; `elesim-up` is the first
Docker build. The default prefix is the directory in which the curl command was
started, not a global system directory.

An installed `elesim-update` is the incremental refresh boundary. It fetches
the install's recorded repository/ref through the bootstrap, validates the
ownership manifest, regenerates only installer-owned artifacts, and lets Docker
reuse unchanged build layers. Topology, security generations, Authority,
credentials, caches, and logs are preserved. Running containers remain
untouched; a later `elesim-up` explicitly replaces them with the rebuilt image.
This is not a rolling multi-host deployment: update each host independently and
use the connection manager when a protocol or managed-security change requires
coordinated rollout.

The recorded source is visible in `elesim-update` output as `repository@ref`.
For a legacy state that predates source metadata, a wrapper generated before
source pinning has `main` baked into its shell text. Redirect it once through
the intended bootstrap, for example:

```bash
curl -fsSL https://raw.githubusercontent.com/owner/repo/ref/installer/bootstrap/install.sh \
  | ELESIM_REPOSITORY=owner/repo ELESIM_REF=ref bash -s -- update
```

Current wrappers also accept a bounded `ELESIM_REPOSITORY`/`ELESIM_REF`
override for this recovery. `elesim-down --purge` removes runtime resources;
it does not select or repair the source revision.

The first update from an unpinned v1-v8 installation accepts the selected
Docker daemon only if exact install-UUID/Compose labels on at least one prior
container or local image prove ownership there. It refuses foreign/ambiguous
objects and also refuses a never-built legacy installation whose daemon cannot
be proven. Use the original daemon or validated clean uninstall/reinstall; an
empty daemon is not evidence.

After v9, the selected local context and Engine ID are a fail-closed boundary.
`DOCKER_HOST` overrides and remote Docker contexts are unsupported because
generated absolute bind paths are local. A daemon reset/replacement that
changes Engine ID has no automatic rebind: restore the pinned daemon for
validated uninstall, or keep the old prefix untouched and reinstall into a new
empty prefix until an audited manual cleanup is performed.

The generated project uses Linux host networking, read-only role configuration
mounts, a read-only Sim model mount, and a profile-gated tools container. Sim
runs with the installing user's UID/GID and writes Genesis through the
installer-owned cache bind mount; this prevents a runtime start from creating
root-owned cache files that would block the next `elesim-update`.
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
one-shot container. A private host-side helper exposes only allowlisted EleSim
Compose/network commands and optional `tailscale nc`; the manager receives no
Docker or tailscaled daemon socket. It does not become a fifth persistent
development/runtime application. The generated `elesim-down` leaves an active
manager alone by default; `elesim-down --purge` explicitly stops the runtime
and force-removes only that exact manager container.

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

A `tailscale-sidecar` install additionally owns the fixed
`elesim-tailscale` container and exact mode-0700
`<prefix>/secrets/tailscale` node-state directory. Ordinary `elesim-down` and
`elesim-update` preserve that directory so device enrollment is not repeated.
The ownership manifest records the install-owned `<prefix>/secrets` root, which
contains that exact bind directory and any EleSim-owned TURN secret. Validated
uninstall removes that root; it does not prune Docker or remove the upstream
`tailscale/tailscale` image, which may be shared by unrelated projects. Local
state removal does not revoke the device record from the tailnet control plane;
decommission that node in the Tailscale admin console as a separate operator
action.

Text log snapshots and the operator laptop's installation-owned SROS2 Authority
are removed by default; `--keep-logs` and `--keep-authority` retain them.
Runtime configuration, managed host role keys and generated secrets are
removed. External credentials/keystores and Developer source checkout are
always outside the deletion boundary. Label commands with the host that owns
them and never substitute a remote prefix into a laptop uninstaller command.

## Remote Compute Server

A headless compute host normally runs Sim. Use
`config.remote.yaml`: it disables the native Genesis desktop Viewer while
keeping the observer and hand-eye render cameras enabled.

일반 설치에서 실제 그래픽 세션이 있는 Sim 호스트라면 `elesim-up --view`로
이번 실행에 한해 native Viewer를 켤 수 있다. `DISPLAY`와 X11 인증/WSLg가
준비되어 있어야 하며, 설정 파일이나 보안 generation은 변경하지 않는다.
연결관리자가 비대화형 SSH로 실행해 `DISPLAY`를 상속받지 못한 경우에는 Sim
호스트의 실제 X11 Unix socket 중 개인키 확인용 SSH 사용자 소유인 것과 그
사용자의 `.Xauthority` 또는 GDM Xauthority를 제한적으로 확인한다. 이
사용자명은 연결 토폴로지의 SSH 관리 계정에서 자동으로 `--viewer-user`로
전달된다. 같은 사용자의 세션이 여러 개면 `xrandr`의 물리 출력(`DP-*`,
`HDMI-*` 등)을 NX/VNC 같은 가상 출력보다 우선하고, 동률은 SSH 세션의
DISPLAY를 먼저 본 뒤 socket 순서로 결정한다. 접속 가능한 조합이 검증되지
않으면 Sim을 시작하기 전에 실패한다. 호스트 권한 검사 뒤에는 동일 이미지와 UID/GID의
일회성 Sim 컨테이너가 숨겨진 X11/GL context를 실제로 열어야 하며, 본 Sim
entrypoint도 DDS를 초기화하기 전에 같은 검사를 반복한다. 이는 잘못된
DISPLAY가 잠깐 DDS readiness로 보이는 것을 막지만, 최종 Genesis 창이 해당
모니터에서 보이고 조작되는지는 실제 호스트에서 별도로 확인해야 한다.
실행 시 Sim 컨테이너를 실행하는 설치 사용자의 X11 권한을 필요할 때만 임시로
추가하고, `elesim-down`, 연결관리자의 명시적 중지, 또는 전체 시작 롤백 시
EleSim이 기록한 권한만 회수한다. 기존에 있던 권한은 유지한다. Sim은 설치
사용자의 UID/GID로 실행되므로 root ACL을 추가할 필요가 없다.

```bash
DISPLAY=:0 CUDA_VISIBLE_DEVICES=0 elesim-up --view
```

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

For a connection-manager-owned topology, install the mandatory Robot unit on the
Jetson by selecting **Robot only** in the native setup path. Setup generates
`elesim-robot.service` and
`elesim-unitree-bridge.service` and prints exact account/group, ACL, unit
registration and enable commands without running `sudo`. The Robot unit runs as
the invoking account and owns the EleSim DDS/SROS2 participant, hardware and
local safety. The bridge runs as the dedicated `elesim-unitree` account and
owns only stock local/plaintext Unitree DDS.

Bind the bridge to the private Jetson-to-GO2 physical NIC and a domain distinct
from the EleSim graph. Bind EleSim DDS to its LAN/VPN interface. The two
interfaces must differ; never expose Unitree topics on Tailscale or the shared
lab LAN. The only process boundary between them is the credential-checked,
bounded `/run/elesim-unitree/bridge.sock`. A disconnect, malformed packet or
keepalive expiry commands a GO2 stop within the configured deadman/tick bound,
while Robot continues the arm safe-hold/torque-off path independently.

The generated wrappers preserve saved DDS configuration, role-scoped SROS2
keys, provisioning guards and later rotations. The bridge wrapper does not
receive the EleSim SROS2 enclave and Unitree topics are absent from the EleSim
security policy. Set `UNITREE_ROS2_WS` or `ELESIM_UNITREE_ROS2_WS` before
bootstrap if the workspace is not `$HOME/ros2_ws`. Override the private-link
defaults (`eth0`, domain `1`) with `ELESIM_UNITREE_INTERFACE` and
`ELESIM_UNITREE_DOMAIN_ID` before bootstrap when required.

`dist/releases/robot` is a separate standalone/manual artifact. Its fixed
`/opt/elesim-robot` unit and `/etc/elesim/robot.yaml` layout do **not** expose an
`install-state.json`, `elesim-net`, stable role key views, or the managed
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

After the native Robot unit exists, a second general/container installation may
be created on the same Jetson for Pilot and/or UI. Use a different prefix (for
example `/opt/elesim-runtime`) and keep its Compose project separate from the
Robot prefix. In `elesim-connections`, place Robot and the container roles on
the same Jetson card; the saved topology represents them as two deployment
units. The connection manager never installs or replaces either unit
implicitly: each unit must already have its own generated wrappers, ownership
manifest and runtime artifacts. It validates, provisions role-scoped security
material, and coordinates their independent lifecycles. Sim remains unavailable
on the Jetson until its ARM64 image and runtime dependencies have been validated.

## Development Model Rebuild

Normal sim startup uses `model/bundles/default`. Rebuild it only while
editing geometry:

```bash
PYTHONPATH=pilot/src:model/builder/src \
  python3 -m elesim_model_builder.cli
```

Production sim artifacts must use the checked prebuilt bundle.
