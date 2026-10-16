# cuRobo on Jetson

Pilot has an optional cuRobo `0.7.8+elesim.1` backend for position IK and a Python API /
offline CLI for timed joint trajectories. Robot retains device I/O, leases,
limits and deadman. Genesis remains the simulation/training engine. No DDS
contract changes are required. Importing Pilot or using its native backend
does not import cuRobo, torch or CUDA through this adapter.

## Environment

The tested target is Jetson Orin, JetPack R36.5, CUDA 12.6 and the existing
CUDA-enabled PyTorch. cuRobo V2 has a different API and newer driver requirements;
do not install its current main branch into this backend. The adapter explicitly
requires v0.7.8 (commit `d64c4b005459db10c5dd867d8b30a87d5bda9bdb`).

The setup script applies a source-hash-checked compatibility patch and builds
the `0.7.8+elesim.1` version. The stock Python generator's pair-index capacity
forced this 911-sphere model into a serial kernel that retains stale collision
outputs/gradients. The patch expands the index buffer, widens the parallel
kernel's loop counters, bounds unrolling and rounds blocks to whole warps.
It preserves all collision pairs; it does not disable collision checks. The
adapter rejects the unpatched version or serial fallback. CUDA graphs use fixed
solver shapes; variable-size path validation uses separate buffers.

From the source checkout on Jetson:

```bash
bash workbench/tools/setup_curobo_jetson.sh
```

This creates `$HOME/.cache/elesim/curobo-integration/venv` with system site packages
so it can use JetPack's existing torch. It does not replace torch, alter drivers,
start services, or initialize hardware. Install the Pilot release and its
protocol wheel into that venv before starting a runtime. Existing ROS/DDS setup
is still required for a live Pilot; the offline planner does not initialize DDS.

## Shared model

`payload/data/models/planning/zed-mini` is shipped with Pilot and includes:

- A mesh-independent URDF with exactly four independent axes. The remaining
  eight bend joints mimic their section's master with multiplier 1. Per-node
  angle is preserved, rather than dividing it by five again.
- The same Linear operating bounds `[-0.142, 0]` m, roll and bend limits as the
  generated Pilot IK model. Velocity caps come from Robot's shipped YAML.
- Collision spheres generated from the collision meshes and origins in the
  existing Genesis arm URDF. Filled 30 mm voxels are enclosed by spheres, so
  this is a conservative approximation and can reject narrow clearances.
  Self collision ignores directly connected links, their rigid subassemblies,
  and pairs whose original mesh geometry is analytically separated over the
  entire joint-limit interval. These additional model-level separation bounds
  are recorded in metadata and regression-tested; unsupported rotations retain
  collision checks. This does not certify physical clearance.
- Source hashes, canonical joint order, bounds and generation parameters.

The current artifact is the **arm with plate and open claws**. It includes only
links with collision geometry in the source URDF. It does not include GO2,
unmodelled cables or payload, and does not follow live claw feedback. Rebuild
for a different camera/mechanism and validate the collision representation
before using it for clearance decisions.

The integration also corrects Pilot model generation for fixed/prismatic child
frame origins. Connector anchors had erased the gripper/camera offsets in the
FK context (up to about 4 cm). Active-arm grasp FK is retained; all fixed branch
transforms now match the URDF. Regenerate/copy the new IK model along with the
planning artifact; old cached IK files do not acquire the correction automatically.

Regenerate after changing the source geometry, IK calibration or Robot limits:

```bash
python -m elesim_model_builder.curobo
```

Model-builder's optional `curobo` extra supplies trimesh for generation. Pilot
needs only the generated artifact; it never imports the builder or Genesis.
Copying the artifact to a different computer does not require absolute mesh
paths or model generation there.

## Offline validation and planning

From the source checkout, with Pilot installed or its app root on `PYTHONPATH`:

```bash
python workbench/tests/system/smoke_curobo.py \
  --model payload/data/models/planning/zed-mini \
  --arm-model payload/data/models/arm/default.json \
  --output /tmp/elesim-curobo-smoke.json
```

The probe performs real GPU FK/Pilot FK comparisons, collision checks, a
nonzero joint trajectory, position IK and rejection of an enclosing obstacle.
It does not connect to DDS, enable torque or send movement commands.

The installed `elesim-plan` command (or
`python -m elesim_pilot.robot.arm.curobo`) accepts an explicit world YAML:

```bash
elesim-plan --model data/models/planning/zed-mini \
  --world world.yaml --start -0.08 0 0 0 --goal -0.09 0 0 0 \
  --output trajectory.json
```

All q values use metres/radians in `(linear, roll, theta1, theta2)` order.
`--target x y z` selects position IK followed by joint planning instead of
`--goal`. Output positions have a fixed `dt_s`; preserve timing when consuming
them. Planning failure, nonfinite/out-of-range values, wrong joint names or
excessive velocity fail explicitly. No unchecked interpolation fallback exists.

`world.yaml` uses cuRobo's world schema, with obstacle poses in the arm **plate
frame** and quaternions in wxyz order. `{}` explicitly requests an empty external
world; self collision is still checked. Genesis's live scene is not automatically
transferred to cuRobo. Moving objects, GO2 body geometry and contact-permission
changes must be supplied explicitly; an empty world must not be described as
environment-aware planning. Absolute world mesh paths are recommended.

## Selecting Pilot IK

The existing `robot.arm.ik` section accepts:

```yaml
backend: curobo
curobo_model: ../../data/models/planning/zed-mini
curobo_world: world.yaml
```

Paths are relative to the Pilot config file. Isolated releases rebind the
model path to `../data/models/planning/zed-mini`. The default remains `native`;
there is no automatic activation or service restart on installation.

Pilot converts world targets through the current arm-base transform and checks
returned positions against its existing FK. CPU orientation refinement is
rechecked against the selected backend's collision constraints before returning
success. This option replaces position IK; it does not replace every existing
Home/slider/workflow movement with a planned trajectory. Use `plan_joint` /
`elesim-plan` for full trajectory generation. Static world files require an
explicit update/restart when the environment or base-relative scene changes.

cuRobo does not establish physical calibration or contact dynamics accuracy.
Intentional wrapping contact, changing payload/claw geometry, dynamic scene
updates, automatic workflow trajectory execution and whole-body GO2 planning
remain separate work. New trajectory execution must retain Robot authority,
deadman and current measured-state checks.

## Measured limits

The checked-in Jetson probe evidence is
`workbench/evidence/curated/planning/curobo_jetson_20261016.json`.
It checks 33 configurations, all collision-sphere transforms, grasp position /
rotation, self-collision recovery, a 10 mm trajectory, position IK and obstacle
rejection using the installed Pilot wheel and real CUDA kernels.

For this 911-sphere model the observed initialization was 57.5 s. First/repeat
joint planning took 30.8/10.6 s and first/repeat IK took 56.2/2.10 s. These are
an offline empty-external-world probe, not a real-time performance result.
Keep the native default for existing interactive control. Further sphere-fit,
collision-cost and solver tuning is needed before a live workflow rollout.
