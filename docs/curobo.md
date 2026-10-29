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
forced the original 911-sphere model into a serial kernel that retains stale collision
outputs/gradients. The patch expands the index buffer, widens the parallel
kernel's loop counters, bounds unrolling and rounds blocks to whole warps.
It preserves all collision pairs; it does not disable collision checks. The
adapter rejects the unpatched version or serial fallback. CUDA graphs use fixed
solver shapes; variable-size path validation uses separate buffers.

Normal Jetson GPU Pilot releases now build this pinned backend in the generated
Docker context, including the checked EleSim patch, CUDA 12.6 compiler/development
libraries and cuSolver 11.7.1.2 compatibility DSO. BuildKit checks imports and
linkage without a GPU; a runtime CUDA SVD probe fails early on a cuSolver ABI
mismatch. Live planning/rendering still needs an Orin runtime check.

For a separate offline development venv, from the source checkout on Jetson:


```bash
bash workbench/tools/setup_curobo_jetson.sh "$HOME/elesim/roles/pilot"
```

This creates `$HOME/elesim/roles/pilot/venv` and keeps the cuRobo checkout at
`$HOME/elesim/roles/pilot/curobo`. System site packages let it use JetPack's
existing torch. It does not replace torch, alter drivers, start services, or
initialize hardware. Install the Pilot release and its protocol wheel into that
venv before starting a runtime. Existing ROS/DDS setup is still required for a
live Pilot; the offline planner does not initialize DDS. The optional positional
argument selects another Pilot role root for nonstandard installations.

Live Pilot uses the generated Docker release and registered instance lifecycle.
The offline venv above is a development tool; it does not replace that runtime
installation or its DDS/security setup.

## Shared model

`payload/data/models/planning/zed-mini` is shipped with Pilot and includes:

- A mesh-independent URDF with exactly four independent axes. The remaining
  eight bend joints mimic their section's master with multiplier 1. Per-node
  angle is preserved, rather than dividing it by five again.
- The same Linear operating bounds `[-0.17333333333333334, 0]` m, roll and bend limits as the
  generated Pilot IK model. Velocity caps come from Robot's shipped YAML.
- Collision spheres generated from the collision meshes and origins in the
  existing Genesis arm URDF. Body meshes use filled 30 mm voxels enclosed by
  spheres; the two claws use 15 mm voxels to preserve their open gap. The model
  has 1005 spheres. This is a conservative approximation and can reject narrow
  clearances.
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
The offline result uses schema v2: target plans report `object_center_xyz` and
`goal_xyz`, with the latter at the object center. Schema v1's `approach_xyz`
standoff endpoint is no longer produced.

`world.yaml` uses cuRobo's world schema, with obstacle poses in the arm **plate
frame** and quaternions in wxyz order. `{}` explicitly requests an empty external
world; self collision is still checked. Genesis's live scene is not automatically
transferred to cuRobo. Moving objects, GO2 body geometry and contact-permission
changes must be supplied explicitly; an empty world must not be described as
environment-aware planning. Absolute world mesh paths are recommended.

## UI XYZ planning and execution

Run Pilot on the CUDA-equipped Jetson; UI and Sim can remain on the Mac.
The existing `robot.arm.ik.curobo_model` and `curobo_world` paths configure the
planner even when the ordinary Solve backend remains `native`. An explicit
world YAML is required. Restart Pilot after changing the runtime configuration.
For an arm-only Sim, set Pilot `simulation.runtime.use_go2: false` as well;
otherwise its world coordinates include the GO2 mount transform.

In **Inverse Kinematics**:

Target XYZ is visualized as a green sphere and direction arrow in the Observer
view, including when the selected motion target is Robot. This display-only
UI→Sim command does not move either arm or the red simulation target object.
The arrow shows Target dir; cuRobo position planning does not constrain it.
The normal debug-marker visibility switch also controls this marker.
Hand-eye RGB-D excludes the debug geometry.

1. `Use current XYZ` copies measured grasp FK into Target xyz. Coordinates use
   metres in the same world frame as the Sim, not camera coordinates. For
   collision planning, Target XYZ is the center of a 20 mm target object.
2. Edit Target xyz, then press `Plan XYZ`. Position IK and collision-checked
   joint planning run in a separate, restartable CUDA worker process; this
   button never commands movement. A successful worker keeps its loaded CUDA
   context for the next plan. Cancel terminates a blocked CUDA call, then a
   later plan starts a clean worker and reloads the model.
   The final grasp point, in the empty space between the open claws, reaches
   the object center within 1 mm. The target sphere is included in both IK and
   joint planning, then checked against all robot spheres at every resampled
   waypoint. Both claws remain collision checked; no link is exempted to admit
   the object. The panel shows the object center and planned claw-center XYZ.
   An intersecting start pose, unreachable grasp center or colliding path fails
   explicitly. There is no fallback that stops outside the object. The executor
   leaves the claws open; it does not automatically close them or attach the
   object. Target dir is not constrained by this four-axis position operation.
3. Wait for `ready`, inspect planned XYZ and duration, then press `Execute path`.
   Torque must already be ON and measured claws open. The executor never sends
   Torque ON, gripper or GO2 commands.
4. `Cancel path` discards remaining samples and releases the exact Robot lease
   to request its local safe hold. Torque OFF remains available through the
   normal hardware panel.

The panel shows request acknowledgement/errors, a persistent failure reason,
the current planning stage and elapsed wall time. Stages distinguish CUDA/model
loading, position IK solver initialization, position IK solve,
collision-checked joint planning and validation. CUDA/model and first IK-solver
initialization each have a 15-minute bound; solve, path planning and validation
each have a 3-minute bound. A timed-out stage terminates its worker and reports
which stage timed out. No invented percentage is shown. Without an exact Robot
connection and fresh measured joints, Plan is unavailable and the reason is
displayed before a click.

Execution is currently Robot-only, with identity/v1 IMU correction and a
stationary base. It pins the exact Robot boot and lease, model/world digests,
and prepared start. Plans expire after 120 s and are consumed once. Moving
the start beyond 1 mm / 0.015 rad or changing Target XYZ requires replanning.
At most 4000 samples / 120 s are accepted. The existing adapter validates joint
limits, sampled collisions and velocity caps. Robot's local slew/limits/deadman
continue to apply. Planning velocity uses 100% of Robot's configured arm speed
cap, and the execution path is re-sampled at intervals no longer than 25 ms
with collision and joint-limit validation repeated on every inserted sample.
During execution the waypoint clock waits when measured error reaches 90% of
8 mm / 0.12 rad; exceeding those limits, non-advancing hardware
feedback for 350 ms, a waypoint wait over 3 s, or a Pilot control-loop stall
above 250 ms cancels motion. A delay above 100 ms pauses and rebases the path
clock without dropping a waypoint. The controller never sends a catch-up burst.
The whole-path timeout is
`max(20 s, 4 × nominal duration + 8 s)`.
The final goal is reported complete after three consecutive measured samples
fall within the same 8 mm / 0.12 rad tracking tolerance used during path
execution. The UI shows the live per-axis residual from the final goal. A
tracking error outside that bound still stops the path with the measured and
commanded joint values. This is position tracking, not guaranteed continuous
swept clearance.
There is no catch-up jump or unchecked interpolation fallback.

Other operator changes and background motion are blocked during planning and
execution. Model/world edits require a fresh plan. An empty world is labelled
`20 mm target + self collision; no other world obstacles` in the UI; surrounding objects, GO2 and cables are
not automatically covered. Home, sliders and existing workflows retain their
ordinary behavior outside this explicit operation.

The CPU/real-DDS integration probe uses separate fixture Robot/Pilot/UI
processes and no hardware:

```bash
python workbench/tests/system/smoke_arm_trajectory.py
```

On Jetson, `--cuda-plan-only` instead exercises the real GPU planning worker
with fixture feedback. It creates no DDS participant and never executes motion.
These probes do not certify physical clearance or stop timing.

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

For the original 911-sphere model the observed initialization was 57.5 s. First/repeat
joint planning took 30.8/10.6 s and first/repeat IK took 56.2/2.10 s. These are
an offline empty-external-world probe, not a real-time performance result.
Keep the native default for existing interactive control. Further sphere-fit,
collision-cost and solver tuning is needed before a live workflow rollout.


### Roll Position-mode restoration (2026-10-19)

Roll uses the original Position mode (3), with the original single-turn mapping
and native ABI 2. The temporary Extended-mode requirement, `arm.roll_zero_raw`
configuration, custom readback/goal conversion and ABI 3 were removed together.
Do not reuse the temporary 6144/6126/6140 multi-turn reference as a persistent
zero calibration. Torque-OFF raw readings can accumulate turns even in Position
mode; their presence alone does not justify changing control modes.

After operator Torque-OFF confirmation, mode 4→3 changed raw 6139→2043 while
Homing Offset remained 0 and Torque remained OFF. No encoder zero/EEPROM offset
or motion command was written. The original posture discrepancy remains
unproven; mode restoration is not acceptance of physical pose/FK calibration.
That historical mode correction retained the then-current Linear mapping. The current mapping is the October 28 decision below; Ghost, gripper and speed behavior are retained.


Planning from measured endpoints allows only a small seed projection at the
model boundary (Linear 0.20 mm, other axes 0.001 rad). This handles encoder/servo
endpoint error, e.g. display Linear 213 producing measured -0.142125 m against
-0.142 m. Larger violations fail before CUDA work with the axis and excess.
Actual telemetry and the execution start check remain measured; the model,
IK goals, collision checks and all planned waypoints keep their strict limits.
A saved `Use current XYZ` goal can be reachable even when the new starting
joint sample fails validation; these are separate checks.


### Ghost path preview

After **Plan XYZ** succeeds, the Observer/native Sim view automatically shows a
translucent green arm replaying the planned path (original timing, short endpoint
pause). It also plays when Sim is paused. The physical arm stays still until you
choose Execute. The Ghost is hidden for a new plan, cancel, execution, expiry,
Pilot disconnection or Sim reset. Reopening the Sim session resends the preview.
The visual path uses up to 96 timed samples with interpolation; execution keeps
all original waypoints. Previewing does not change collision geometry, measured
Robot/Sim posture, hand-eye images, or verify a live obstacle map.


Linear calibration update (2026-10-28): **1.5° = 1 mm, max 260°**, or
`q in [-260/1500, 0]` m. This replaces the previous 259°/230 mm mapping and field
199° cap. Configuration, IK and planning artifacts must be deployed together.
The exported oct27 policy/manifest remain byte-identical; the Pilot runner
intersects its output limits with deployment limits without coordinate scaling.

World objects, including target spheres, are converted with
`WorldConfig.get_collision_check_world()` for initial, IK and path queries so
the selected collision checker receives them. Trajectory optimization adds a
2 mm soft self-collision margin per sphere; hard and independent interpolated
path validators retain the original geometry. A plan has up to 12 attempts,
120 seconds, graph fallback from attempt 2 and an external 180-second stage
watchdog. `[curobo-path]` reports attempts, graph/trajopt/finetune time, query
validity and result. The default object target is `(0.2976, 0.1682, 0.2901)` m;
the Wrap tree center `(0.368, 0.16, 0.55)` m is separate. The historical
141-waypoint success used a different target and is not proof of this query.


### Measured feedback during execution

The cuRobo path timing is nominal. Robot execution may pause waypoint progress
when the next sample would place a joint more than 90% of the current hard
tracking limit ahead of measured feedback. Pilot resends the last goal at the
normal cadence to maintain the Robot deadman; it never skips ahead to catch up.
The current hard limits are 8 mm Linear and 0.12 rad per rotary axis. A follow
wait is bounded to three seconds, with an overall execution bound of
`max(20 s, 4 × nominal path duration + 8 s)`. Freshness, route/lease,
scheduling-overrun and final-settle checks remain enforced. Failure records
include measured/commanded q and the violating axis. UI shows axis tracking
errors. Ghost playback shows nominal planning timing and may finish ahead of
feedback-paced physical execution. On 2026-10-19, a 61-waypoint, nominal 1.5 s
path completed on the physical arm at 100% of its configured speed caps; this
confirms that run reached its measured joint goal, but is not a general speed
calibration or clearance guarantee.
