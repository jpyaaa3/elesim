# Reverse cuRobo V2 research experiment

The current path uses cuRobo **v0.9.0**, commit
`42498cf4e7d0d6fbd4ef9fbcec1243c00c888a83`, and its official MorphIt optimizer.
It lives beside the preserved reverse-GA and v1 experiments. Pilot's pinned
0.7.8 runtime backend is not migrated by this repository-only experiment.

## Environment and GPU selection

`setup_v2.sh EXISTING_TORCH_PYTHON NEW_VENV NEW_CUROBO_SOURCE` creates an
independent research environment using the existing Torch packages as read-only
fallback. V2 dependencies take priority inside that venv. The tested server is
`hckang@100.74.222.24`, RTX A6000 GPU2 UUID
`GPU-a40782b2-881c-e285-2a8e-66cae9162a01`, driver 580.65.06, Python 3.11,
Torch 2.12.1+cu130. Export that UUID as `CUDA_VISIBLE_DEVICES` for GPU work.
The official V2 source and resolved dependencies remain separate from GA and v1.

## Fit the model

```bash
CUDA_VISIBLE_DEVICES=GPU-a40782b2-881c-e285-2a8e-66cae9162a01 \
  <V2_VENV>/bin/python -m workbench.research.experiments.hug_reverse_curobo.v2_model \
  --reference-plan <GA_ROOT>/case_01/plan/plan.json --output NEW_MODEL \
  --sphere-budget 2048 --iterations 500
```

The source is the calibrated four-axis arm tree with eight direct mimic joints.
Original collision meshes are restored for fitting. Plate collision is excluded
as requested; the plate root transform and original Genesis geometry remain.
The camera visual mesh is added as collision geometry, matching the existing
Hug task's `materialize_robot` rule. Only the task's fixed end assembly pairs
are exempted from camera self collision; other arm and world checks remain.
OBJ vertices split at normal/UV seams are welded by position before checking
closed topology. This preserves mesh coordinates and extents. Eight actual
non-plate asset regressions check closure, winding, positive volume and unchanged
extents. Earlier open-mesh diagnoses omitted this necessary welding step.

Fitting calls the pinned official MorphIt backend. Its inward surface samples
supply the coverage objective; volume samples are suppressed in the isolated
fitting process. Sphere centers and radii are optimized together. Small-sphere
pruning and learning rates are explicit in metadata. A post-filter removes
outside centers and deep-interior spheres, and shrinks radii to the nearest
surface distance. Quality metrics are sampled evidence, not a proof of exact
containment or complete coverage. Empty links fail generation. The user-supplied
sphere budget defaults to 2048; this is an experiment setting, not a claimed V2
kernel limit. Mechanical and analytically proven collision ignores are preserved;
no random-sampling-based pair removal is added.

`render_model --model NEW_MODEL --plan PLAN_JSON --output IMAGE.png` renders the
actual exported spheres beside original meshes on CPU.

`audit_model --model NEW_MODEL --output AUDIT.json` checks every exported sphere
against original triangles using CPU Trimesh containment and closest-point
distance, independently of the Warp SDF used in fitting.

## Compare three unchanged GA cases

To give the entire path to the native cuRobo planner, use `--planner-mode full`
with the command below, omitting `--release-prefix`. This sends the exact admitted
start and Home to `plan_cspace` with native PRM, B-spline TrajOpt and timing
optimization enabled. No authored waypoint, release prefix or automatic start
offset is permitted. Independent endpoint checks are recorded but do not replace
the start or prevent cuRobo from receiving it. Successful native output may only
be slowed uniformly to meet the task's command-rate limits, then is checked
densely and replayed against the unchanged GA criteria. Failure is retained;
there is no automatic fallback to the authored release policy.

`--planner-mode full --native-profile task` keeps the whole path native while
mapping the experiment's actual speeds into the search URDF: 0.05 m/s linear,
1 rad/s roll and 0.3 rad/s for each per-node bend. It uses native RRT, 64 spline
knots, TrajOpt and TimeOpt, with the motor-scaled graph metric and a stronger
position-bound optimization cost. No motion waypoint or contact-release phase
is supplied. Native interpolation-buffer capacity is selected automatically;
the physical episode remains limited to 20 s.

This profile gives solver bounds at fixed start/Home endpoints at most 50 nm
linear or 0.5 microradian angular numerical slack. Native float32 cubic
interpolation otherwise rejected a feasible trajectory for a 0.12 microradian
roll-bound discrepancy. Command bounds, direction lock and physical acceptance
remain unchanged; substantive excursions are rejected. `--explain-failure`
records the native position/rate/acceleration/jerk/effort checks and the last
internal TimeOpt status without overriding any native failure mask.

The scene currently demands zero sphere overlap with a stationary cylinder.
The original physical task allows correct-side contact within its 5 mm maximum
penetration and lets the object move within its displacement/tilt limits. These
planning and physical contact conditions still differ; a native collision
failure alone does not prove the original GA task is infeasible.

`--native-profile task-contact` adds an explicit proposal approximation for
the original task's permitted contact. Moving nodes and grippers avoid a core
eroded by the task's maximum penetration. Protected housing/wedge/camera links
still avoid the full original cylinder; every link still checks the floor,
and sphere geometry and self-collision checks are unchanged. Both meshes retain
the admitted object's pose. Native collision layers apply this policy to the
independent checker and every planner rollout before warmup. This is not exact
face-aware contact optimization: face/normal/force, corner penetration, and the
free object's dynamics remain governed by the original Genesis acceptance.
No waypoint or release prefix is introduced. A failed physical replay remains
failed regardless of native core clearance.

The following command retains the separately measured mixed release-policy/RRT
experiment. Select `--planner-mode full --attempts 4` for the whole-path trial:

```bash
CUDA_VISIBLE_DEVICES=GPU-a40782b2-881c-e285-2a8e-66cae9162a01 \
OMP_NUM_THREADS=4 PYOPENGL_PLATFORM=egl \
PYTHONPATH=payload/runtime/common/protocol:payload/runtime/docker/sim/app:. \
  <GA_PYTHON> -m workbench.research.experiments.hug_reverse_curobo.v2 batch \
  --ga-root <GA_ROOT> --model NEW_MODEL --v2-python <V2_VENV>/bin/python \
  --gpu GPU-a40782b2-881c-e285-2a8e-66cae9162a01 --output NEW_RUN \
  --planner-mode rrt --release-prefix --attempts 4
```

`v2.py` runs under the unchanged GA/Genesis Python. `v2_worker.py` runs under
V2 Python with an explicit JSON/NPZ boundary. No sibling runtime deployment is
imported by the V2 worker. Each case gets a fresh Genesis process. Source/model
hashes are checked; old plan metadata is never relabeled to bypass a mismatch.

The planner sees a 128-sided circumscribed cylinder **mesh**, measured after
admission and transformed to the actual plate frame, plus the floor. Go2 geometry
is still checked by Genesis only; this is not a complete robot-host scene.
All-link FK is independently compared against the URDF at start and Home before
planning. Targets are known joint configurations; pose IK is unnecessary.
`--planner-mode spline` uses V2 `plan_cspace`, including its B-spline validation.
The current `rrt` experiment calls V2's GPU graph planner directly, with eight
parallel proposals per call. It preserves each successful polyline, applies the
original task's command-rate limits, and independently checks every 10 ms command.
It does not treat a failed spline as valid or claim V2 spline dynamics checks.
Initial CUDA setup, per-query planning, dense command validation, physical replay
and total wall time are recorded separately. A subprocess watchdog bounds a hung
V2 call. Failed planning is never counted as a fast task success.

The RRT search uses necessary bounds derived from the unchanged motor budgets:
an intermediate pose needs at least the endpoint travel through that pose. This
excludes impossible excursions without removing any budget-feasible path.
Search-policy files retain those bounds and motor-scaled distance weights.
Actual total motor travel is checked again before physical evaluation.
The pinned RRT initializes its own metric independently of the robot config.
The current worker applies the requested metric explicitly before warmup and
records its effective value. Its graph resolution is 0.005 in that metric.

If the original contact pose fails the V2 collision test, RRT mode searches a
bounded release offset (at most 20 mm linear, 10 degrees roll, 2 degrees per-node bends).
The exact original start-to-offset prefix remains part of the commands and
must pass the original Genesis contact criteria. The cylinder is never moved
or rebased. V2 densely validates the offset-to-Home suffix. Offset sampling,
selection and prefix timing are saved explicitly.

For a contact release that is more than a small endpoint offset, opt in with
`--release-prefix`. A 625-point grid tests independent progress of each bend
toward straight, and linear/roll progress toward their Home values, clipped to
the budget-derived bounds. Eight diverse collision-free endpoints are retained
even if the original endpoint itself passes collision checking. Each prefix
opens its selected bends first, then retracts/rolls at that opening. It is
physically executed; the complete command path must satisfy every original GA
criterion, including the motor budget. This mode does not demand free-space
cylinder clearance during the prefix. The object is never teleported.

Only float32-scale initial roll jitter may be normalized (maximum 5e-7 rad).
The initial 90-degree direction lock is preserved; post-unlock reversals remain
legal. Bounds, timing, slew and independent dense collision checks run again on
the commands actually sent to Genesis. Physical travel, contact, 50 mm displacement,
10-degree tilt and Home/rest criteria are unchanged. Three fresh nominal admissions
and replays are required for `reverse_path_verified`. No PPO is started and no
forward-grasp, lift or validated reset-pool success is inferred.

CPU checks:

```bash
python3 -m pytest -q workbench/tests/research/experiments/test_hug_curobo_v2.py \
  workbench/tests/research/experiments/test_hug_curobo_v2_meshes.py
```

These are focused research checks. Canonical required/extended/release/DDS gates
have not been run for this change; no generated dev attachment is known here.

`report_v2 --ga-root GA_ROOT --model NEW_MODEL --run NEW_RUN --output REPORT.json`
builds a compact comparison from the saved completions and fitting metadata.
Multiple `--run` arguments preserve failed baselines alongside the final run.
Use `--extra-case RUN/case_01` for a single-case retry; it stays separate from
the complete batch success count.
GA's original wall times include its postprocessing/video; V2 batch times exclude
video and model fitting. Compare these scopes explicitly, and do not interpret a
short failed run as successful-task speedup.
Record observed GPU contention with `--gpu-sharing-note TEXT`; exclusivity is
not inferred from a single GPU snapshot.

`diagnose_v2 --model NEW_MODEL --case NEW_RUN/case_01 --reference
GA_ROOT/case_01/winner/states.npz --output DIAGNOSTIC` independently checks a
known physical reference against both the collision checker and RRT rollout.
Reference commands are diagnostic inputs only and never seed a proposal.

`record --run NEW_RUN/case_02 --candidate-name candidate-0000-lane-0 --output
NEW_VIDEO` records a fresh physical replay, with its own admission and report.

Official references: [pinned V2 source](https://github.com/NVlabs/curobo/tree/42498cf4e7d0d6fbd4ef9fbcec1243c00c888a83),
[sphere fitting](https://nvlabs.github.io/curobo/latest/reference/sphere_fitting.html).

## Recorded comparison, 2026-11-12

After correcting task-speed mapping and float32 endpoint-bound noise, the
`reverse-curobo-v2-task-native-float-20261112/case_02` trial passed native
path/collision/dynamics validation and the unchanged physical task, followed
by three successful fresh admissions/replays. It used `full --native-profile
task`, with no authored motion or start offset. Total case time was 315.25 s
(5 min 15 s), including setup and physical verification; its four native calls
totaled 17.17 s. The accepted command travel was 180/180/534.13 motor degrees
against limits 200/220/600; peak object XY motion was 0.66 mm. Cases 01 and 03
in that profile still failed native collision checks. This is a one-of-three
whole-path result, separate from the earlier three-of-three mixed policy.
Focused server checks including contact-core scene preservation passed 22 tests.

The fit ran 500 iterations and produced 2046 spheres from a 2048 budget
across 16 links in 69.83 s (fitting-worker scope). Radii range from 0.84 to 37.27 mm;
sampled sphere protrusion was zero for every link. The repeated arm-node surface
gap 95th percentile was 2.09 mm. These are sampled geometric metrics.

The full `reverse-curobo-v2-bounded-20261112` batch took 573.15 s (9 min 33 s),
excluding fitting and video. Cases 02 and 03 passed the unchanged physical
criteria and three fresh admissions/replays each. Their case-internal times
were 187.40 s and 261.95 s; case 01 exhausted its planning budget in 110.43 s.
Original GA process wall times were 850.12 s, 1260.19 s and 2670.45 s, with all three
verified (79 min 41 s summed). These scopes differ, and a concurrent `hug_pipeline`
worker was observed on GPU2. This is not an isolated successful-task speedup.

The independently checked GA case 01 reference overlaps the stationary V2
cylinder during 164 samples (0.01–1.64 s), while floor and self-collision checks
remain clear. The physical task permits proper-side contact there.

The later `reverse-curobo-v2-open-release-20261112/case_01` trial passed the same
physical criteria and all three fresh admissions/replays in 254.16 s. It opens
the C-shaped arm first, then retracts/rolls, then uses V2 for the remaining path.
The prefix is authored by the release policy and physically validated; it is
not generated by V2's free-space graph search. Single-case retries stay separate
from the batch success count. Raw failed trials remain preserved.

The final `reverse-curobo-v2-final-20261112` batch used the same release policy,
model and source revision for all three cases. All three passed, each with three
fresh successful admissions/replays. Total wall time was 621.10 s (10 min 21 s),
with case process times of 210.44 s, 212.38 s and 198.26 s. Model generation was
measured separately at 69.83 s. Installation, initial cold compilation and video
are outside the final batch; its compilation caches were already populated.
GPU sharing and the different GA video scope still prevent an isolated speedup
claim. The CPU triangle audit passed all 2046 spheres: zero outside centers and
zero radius excess above 1 micrometre tolerance; minimum inward margin 9.93 um.

See the [curated comparison](../../../evidence/curated/planning/curobo_v2_morphit_20261112.json)
and [actual model figure](../../../evidence/curated/planning/curobo_v2_morphit_20261112.png).

## Native full-path trial, 2026-11-12

`reverse-curobo-v2-full-resumed-20261112` used `--planner-mode full --attempts 4`
on GPU 2 with the same 2046-sphere model and original GA cases. Native PRM,
B-spline trajectory optimization and time optimization were enabled. All cases
used identical source hashes, no authored intermediate points and no automatic
start offset. The earlier user-cancelled full trial was preserved separately.

The batch finished in 406.38 s (6 min 46 s) with zero accepted trajectories out
of three cases. Case process times were 160.81 s, 124.98 s and 120.58 s, including
initialization. Each case made four planning calls, each allowing four internal
attempts. Every call returned `limits_and_dynamics_valid=false`. Case 02 passed
the collision validation mask; cases 01 and 03 did not. Case 03's exact admitted
start was flagged by the independent checker and still passed unchanged to
cuRobo, which also logged start/end collision rejection.

No native candidate passed all validation, so there were no commands to send
to physical evaluation or fresh replay. This is a completed failed planning
experiment, not physical success or a successful-task speedup. The combined
limit/dynamics mask also covers time optimization and finite-buffer checks;
the specific failing component has not been isolated. The focused checks passed
18 tests. Canonical runtime/release gates remain unrun as described above.

The [full-trial comparison](../../../evidence/curated/planning/curobo_v2_full_20261112.json)
keeps this 0/3 native result separate from the earlier 3/3 mixed-policy result.

## Rejected full-planner video diagnostics

The original full trial retained no rejected joint trajectories, so it has no
motion replay video. To inspect such output, run a separate `v2_worker` trial
with the original case query, `--planner-mode full --attempts 1 --capture-rejected`.
This records `rejected-interpolated.npz` and `rejected-native.npz` independently
of the candidate command export. They never enter the physical evaluator or
curriculum just because they were saved.

`preview_rejected --case ORIGINAL_FULL_CASE --planner NEW_DIAGNOSTIC_PLANNER
--output NEW_VIDEO --gpu GPU_UUID` renders a finite rejected trajectory with an
on-screen warning. It uses the actual native joint samples in the unchanged
Genesis scene, with the measured cylinder pose fixed. Physics and servo control
are not stepped, and sample progress is displayed over eight seconds rather
than claiming the rejected native timing. This is a new diagnostic rerun and a
kinematic preview, not the original physical execution or a successful teacher.

The recorded case 02 diagnostic has 394 interpolated native samples and a
10-second, 960x720, 24 fps H.264 video. Its rerun returned false collision and
limit/dynamics masks, whereas the original case 02 trial passed its collision
mask. These are separate runs; the original 0/3 comparison is unchanged. See
the [diagnostic video record](../../../evidence/curated/planning/curobo_v2_full_video_20261112.json).

## Preserved v1 experiment notes

# Reverse cuRobo comparison

Independent candidate generator next to `hug_reverse_ga`; never launches PPO or
modifies an existing run. Uses the shipped patched `0.7.8+elesim.1` Pilot adapter
and planning model. See `docs/curobo.md` for the required environment. Do not
install cuRobo main or change the active training environment.

Run only in an isolated server checkout/environment on an explicitly selected
available GPU, with Pilot, protocol and Sim packages installed:

```bash
CUDA_VISIBLE_DEVICES=<available-GPU-UUID> python3 -m \
  workbench.research.experiments.hug_reverse_curobo.experiment \
  --plan /absolute/path/to/matching/reverse-plan/plan.json \
  --output workbench/evidence/generated/reverse-curobo-new --attempts 8
```

The input is a source/model-matching reverse-GA plan, preserving its exact solver
branch, Home and physical settings. Stale plans are refused, never relabeled.
Four-axis values in this plan are already per-node bends, matching the existing
cuRobo adapter. They must not be divided by five again.

The archived initial comparison used an explicit empty external planning
world: cuRobo checks modeled arm self collision; Genesis checks cylinder contact,
Go2, floor, protected surfaces, displacement, tilt, travel and roll direction.
It is a proposal-and-rejection baseline, not contact-aware optimization. The
planner can repeatedly generate physically invalid paths; no success is promised.
The shipped conservative sphere model may also reject a valid initial posture.
No collision checks or physical acceptance criteria are weakened on failure.

Each proposal is uniformly slowed if needed, preserving the polyline, and padded
with Home up to the existing 20-second budget. Admission, full-state restoration,
leg PD restoration, per-tick acceptance and three fresh nominal replays reuse the
reverse-GA implementation unchanged. One physical lane is used; report this when
comparing against GA's 128-lane batches. Repeated calls are bounded attempts, not
a claim of independent random seeds. Initial planner time, per-query time, physics
time and total wall time are separate. GPU contention invalidates speed claims.
No video is rendered in this baseline; compare against GA compute timing without
video overhead or report the difference. NPZ full-state traces are retained.

Current validation: host CLI/retiming checks only. CUDA planning, geometry parity
for this case, Genesis evaluation and server performance remain unvalidated.

## Bounded release-waypoint exploration

`--strategy via-release --attempts 16` samples deterministic independent axis
progress between the exact start and Home, including four axis-leading seeds.
Each candidate joins two cuRobo trajectories and uses the unchanged physical
acceptance checks. Invalid endpoints stop before repeated planning.
Float32-scale roll jitter is projected to a monotone command only when the
maximum correction is at most 5e-7 radians. Larger reversals are rejected.
Every modified command path is collision-checked again before simulation.
This is a proposal search, not a contact-aware optimizer or verified RL dataset.

## Cylinder world correction

New runs include the admitted cylinder pose transformed from Genesis world to
measured plate coordinates. Sixteen conservative cuboid strips cover the cylinder
cross-section and full height. The exact world is saved in planning-world.json.
This approximation may reject contact/clearance that the real mesh permits.
Floor and Go2 are still checked only by Genesis; this is not a complete world.
The initial contact admission remains unchanged. Initial cylinder collisions are
reported as planning_endpoint_invalid, never bypassed. Contact-permitted release
planning is not implemented yet, and no successful curriculum is claimed.
