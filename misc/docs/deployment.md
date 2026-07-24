# Deployment

## Build Release Contexts

From the repository root:

```bash
python3 misc/tooling/release/build.py
```

This writes one self-contained build context per role under
`dist/releases/<role>`. Each context contains its application wheel, the
transport-neutral Python support wheel, ROSIDL source at
`interfaces/elesim_interfaces/`, configuration, direct dependency pins and
deployment metadata.
The simulator context additionally contains the prebuilt model bundle.
`dist/releases/infra` contains the security bootstrap and optional Coturn
compose files. It also contains the standard-library download bootstrap and
the independently installable setup/doctor package; these are infrastructure,
not a fifth runtime deployment.

The build command verifies every generated context by default. It performs a
clean `--no-deps` temporary install, checks wheel ownership, parses the shipped
configuration, validates the simulator model bundle, and runs the installed
entry point with `--help`. To re-run only this verification:

```bash
python3 misc/tooling/release/verify.py dist/releases
```

`--no-verify` exists only for diagnosing an incomplete build; an artifact made
with that option has not passed the release gate.

## Reproducible Multi-Host Bootstrap

When multiple hosts must install the same source snapshot, use the same full
40-character commit SHA in both the raw bootstrap URL and `ELESIM_REF`. Run the
same command on each owning host, replacing the example SHA with the selected
commit:

```bash
# [each Simulator/Controller/UI host]
elesim_commit=0123456789abcdef0123456789abcdef01234567
curl -fsSL \
  "https://raw.githubusercontent.com/jpyaaa3/elesim/${elesim_commit}/misc/setup/bootstrap.sh" \
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
exec <role-command> --ros-args --enclave /elesim/<role>
```

The UI enclave may publish to Controller operator and Simulator
session/signaling control topics and subscribe to their replies/state.
Controller may publish fenced motion and subscribe to telemetry/RGBD. Robot and
Simulator may consume only addressed control/motion traffic and publish only
their own replies, telemetry, and media. Generate permissions from this
least-privilege matrix; a keystore that gives every role wildcard
publish/subscribe rights defeats the profile.

Ordinary IPv4 NAT, CGNAT and symmetric NAT are unsupported by this peer-to-peer
topology. Port forwarding on the compute server alone does not make the
laptop's DDS locators reachable. Use a routed VPN or mutually reachable global
IPv6. Neither static peers nor TURN change this limit.

Coturn is optional on a flat LAN. The setup GUI's **managed** option places a
pinned Coturn service in the Simulator host's generated Compose project. In
that case `elesim-up`, `elesim-down`, and `elesim-logs` manage Simulator and
Coturn together. Managed TURN requires the SROS2 profile because ICE
credentials and signaling cross DDS.

The standalone release Compose is for an **external** relay that is deliberately
operated outside the generated Elesim project:

```bash
docker compose \
  --env-file dist/releases/infra/coturn/.env \
  -f dist/releases/infra/coturn/compose.yaml up -d
```

Managed Coturn's REST HMAC secret is mounted into Coturn and the co-located
Simulator only. Simulator issues bounded-lifetime credentials for itself and
the UI, tied to the active simulation session; UI never receives the static
secret. This explicitly trusts the managed Simulator to mint TURN credentials.
External TURN uses a separately provisioned JSON file with `username`,
`credential`, and optional `expires_at`. Select it only while installing the
Simulator host. Setup validates it as a small regular private file and mounts
it read-only into Simulator; Controller/UI-only hosts keep only the TURN URL.
Simulator passes the usable credential to its active UI through the DDS
session grant. TURN relays DTLS/SRTP WebRTC media, not DDS data or signaling.
If that DDS exchange is on a shared network, select SROS2 rather than
`trusted-network`.

## Container Roles

For a clean Ubuntu host, prefer the setup-generated Compose project over manual
role builds:

```bash
curl -fsSL https://raw.githubusercontent.com/jpyaaa3/elesim/main/misc/setup/bootstrap.sh | bash
elesim-up
```

The curl command opens a Korean/English loopback-only browser wizard. It
generates files but does not build or start images; `elesim-up` is the first
Docker build. The default prefix is the directory in which the curl command was
started, not a global system directory.

The generated project uses Linux host networking, read-only role configuration
mounts, a read-only Simulator model mount, and a profile-gated tools container.
CPU policy omits GPU requests. `inherit` and `specific` request exposed NVIDIA
devices, so the host must have NVIDIA Container Toolkit in addition to its GPU
driver. UI defaults to Mesa software GL inside the X11 container; set
`ELESIM_UI_SOFTWARE_GL=0` only after validating host/container graphics driver
compatibility.

Build a role by loading the generated wheel names:

```bash
cd dist/releases/simulator
set -a
. ./WHEELS.env
set +a
docker build \
  --build-arg PROTOCOL_WHEEL="$PROTOCOL_WHEEL" \
  --build-arg APP_WHEEL="$APP_WHEEL" \
  -t elesim-simulator .
```

Use the same command in `controller` or `ui`, changing only the image tag.
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

This mode mounts `/dev`, uses host network/IPC, and is privileged. Use it only
on an owned Ubuntu/WSL amd64 workstation. WSLg mounts are generated only when
the outer bootstrap detected WSLg on the host.

## Remote Compute Server

A headless compute host normally runs Simulator. Use
`config.remote.yaml`: it disables the native Genesis desktop Viewer while
keeping the observer and hand-eye render cameras enabled.

```bash
elesim-simulator \
  --config /opt/elesim/simulator/config/config.remote.yaml \
  --runtime-config /etc/elesim/simulator.dds.yaml \
  --model-bundle /opt/elesim/simulator/model/bundles/default
```

The laptop runs Controller and UI with the same system/domain/RMW/security
profile. Each host's DDS interface and static peers are host-specific:

```bash
elesim-controller \
  --config /opt/elesim/controller/config/default.yaml \
  --runtime-config /etc/elesim/controller.dds.yaml

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

Copy only `dist/releases/robot` to the Jetson, then run:

```bash
cd /opt/elesim-robot
bash install.sh
sudo cp systemd/elesim-robot.service /etc/systemd/system/
sudo cp config/default.yaml /etc/elesim/robot.yaml
sudo systemctl daemon-reload
sudo systemctl enable --now elesim-robot
```

The Jetson artifact contains no UI, controller workflow, Genesis, source
assets or model builder.

## Development Model Rebuild

Normal simulator startup uses `model/bundles/default`. Rebuild it only while
editing geometry:

```bash
PYTHONPATH=controller/src:misc/tooling/model_builder/src \
  python3 -m elesim_model_builder.cli
```

Production simulator artifacts must use the checked prebuilt bundle.
