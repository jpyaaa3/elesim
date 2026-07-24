# Deployment

## Build Release Contexts

From the repository root:

```bash
python3 misc/tooling/release/build.py
```

This writes one self-contained build context per role under
`dist/releases/<role>`. Each context contains its application wheel, the exact
protocol wheel, configuration, direct dependency pins and deployment metadata.
The simulator context additionally contains the prebuilt model bundle.
`dist/releases/infra` contains the security bootstrap and optional Coturn
compose files. It also contains the standard-library download bootstrap and
the independently installable setup/doctor package; these are infrastructure,
not a sixth Python deployment.

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
# [each Router/Simulator/Controller/UI host]
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

## Multi-Host Credentials

Plaintext defaults bind to loopback. Before exposing Router or RGBD streams on
a LAN or public server, generate Curve identities and a TURN secret on a
trusted administration machine:

```bash
python3 dist/releases/infra/bootstrap_security.py \
  --output ./elesim-secrets \
  --coturn-env dist/releases/infra/coturn/.env \
  --turn-public-ip 203.0.113.10 \
  --turn-realm sim.example.com
```

When running from source, use `misc/infra/bootstrap_security.py`. Distribute
only the credentials required by each host:

| Host | Required files |
| --- | --- |
| Router | `curve/router/router.key_secret`, `curve/authorized/`, `curve/endpoints.yaml`, `turn.secret` |
| Simulator | `curve/clients/sim-default.key_secret`, Router public key, `curve/media/simulator-media.key_secret`, `curve/media-authorized/` |
| Laptop | `curve/clients/controller-main.key_secret`, `curve/clients/ui-main.key_secret`, `curve/clients/doctor-main.key_secret`, Router public key |
| Robot | `curve/clients/robot-go2.key_secret`, Router public key, `curve/media/robot-media.key_secret`, `curve/media-authorized/` |

Copy `doctor-main.key_secret` to any trusted administration host that runs
`elesim-net doctor`. It authorizes Router registration/discovery only and is
not present in the RGBD media allowlist. The active RGBD probe uses the same
Controller media identity as the production Controller.

Install private files mode `0600` and make the public examples under each
project's `config/` point to their installed locations. Never copy the full
private-key tree to every machine.

The setup GUI can perform this distribution without a manual `scp` sequence.
Generate the bundle while installing Router, then select `receive from Router
host` on the Controller/UI or Robot host. The operator must confirm the probed
SSH host fingerprint. Setup authenticates with the SSH agent/default keys or a
selected key, and copies only the selected roles' manifest. Passwords and the
complete credential root are not accepted by the GUI.

Coturn is optional on a flat LAN. The setup GUI's **managed** option places a
pinned Coturn service in the Router host's generated Compose project. In that
case `elesim-up`, `elesim-down`, and `elesim-logs` manage Router and Coturn
together.

The standalone release Compose is for an **external** relay that is deliberately
operated outside the generated Elesim project:

```bash
docker compose \
  --env-file dist/releases/infra/coturn/.env \
  -f dist/releases/infra/coturn/compose.yaml up -d
```

Router and Coturn must read the same static HMAC secret. UI and Simulator
receive only short-lived credentials minted by Router. Router refreshes them
before expiry; UI and Simulator renegotiate both video peers without replacing
the simulation control lease.

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
cd dist/releases/router
set -a
. ./WHEELS.env
set +a
docker build \
  --build-arg PROTOCOL_WHEEL="$PROTOCOL_WHEEL" \
  --build-arg APP_WHEEL="$APP_WHEEL" \
  -t elesim-router .
```

Use the same command in `controller`, `ui`, or `simulator`, changing only the
image tag. Runtime addresses are supplied through the role's YAML or CLI
arguments. UI containers also need the host display/GPU mounts appropriate to
the workstation.

On shared GPU hosts, prefer scheduler-assigned devices. The installer default
does not overwrite `CUDA_VISIBLE_DEVICES`. If Compose must pin a physical GPU,
choose `specific`; the generated reservation contains one `device_ids` entry
and does not expose the other devices. An in-container selector cannot grant
access to a GPU the container runtime did not expose. CPU-only installs do not
require a GPU reservation.

## Development Container

The GUI's Developer edition creates one complete Git workspace and a privileged
development Compose project under `<workspace>/.elesim/development`. It is not a
sixth runtime deployment and must not be used as a production multi-host
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

A headless compute host normally runs Router and Simulator. Use
`config.remote.yaml`: it disables the native Genesis desktop Viewer while
keeping the observer and hand-eye render cameras enabled.

```bash
elesim-router --config /etc/elesim/router.public.yaml

elesim-simulator \
  --config /opt/elesim/simulator/config/config.remote.yaml \
  --runtime-config /etc/elesim/simulator.public.yaml \
  --model-bundle /opt/elesim/simulator/model/bundles/default
```

The laptop runs Controller and UI against the Router's reachable address:

```bash
elesim-controller \
  --config /opt/elesim/controller/config/default.yaml \
  --runtime-config /etc/elesim/controller.public.yaml

elesim-ui --config /etc/elesim/ui.public.yaml
```

The remote UI receives two WebRTC streams: the operator observer and the
hand-eye preview. It controls orbit, pan, zoom, pause/resume, single-step,
reset, speed and marker visibility through protocol-v4 commands. The native
Genesis Viewer window is not transported.

Required firewall paths are Router TCP `5558`, direct RGBD TCP `5568`, and,
when Coturn is used, TCP/UDP `3478` plus UDP `49160-49200`. A direct WebRTC LAN
path may also use dynamically selected ICE UDP ports.

The setup GUI itself remains on `127.0.0.1`; administer a remote installation
through an SSH local-forward instead of opening the GUI port in the firewall.
SSH's port is unrelated to Router, RGBD, or TURN ports.

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
PYTHONPATH=packages/protocol/src:controller/src:misc/tooling/model_builder/src \
  python3 -m elesim_model_builder.cli
```

Production simulator artifacts must use the checked prebuilt bundle.
