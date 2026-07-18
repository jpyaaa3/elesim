# Deployment

## Build Release Contexts

From the repository root:

```bash
python3 tooling/release/build.py
```

This writes one self-contained build context per role under
`dist/releases/<role>`. Each context contains its application wheel, the exact
protocol wheel, configuration, direct dependency pins and deployment metadata.
The simulator context additionally contains the prebuilt model bundle.

## Container Roles

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
PYTHONPATH=tooling/model_builder/src \
  python3 -m elesim_model_builder.cli
```

Production simulator artifacts must use the checked prebuilt bundle.
