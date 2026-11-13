# Solver → reverse teacher → Home PPO → interpolation

This repository-only coordinator owns cylinder experiments, not the runtime RL
playground. Run from the checkout with its prepared scientific Python environment.
Do not install Genesis/Torch into the lightweight host just to run CPU tests.

## Contract

- One arbitrary diameter per immutable study; default mu cutoff 0.4, physical
  friction 0.5. Mapper functions, never stored point-cloud answers, supply cases.
- Select a maximum-covered-area **sampled** C-band q3/q4 triangle after physical
  Home/goal catalog screening. Preserve each selected q/root/feature into GA.
- Reverse GA releases from the exact solver grasp. Its three fresh replays are
  required. Reverse success alone is not forward success.
- An optional native cuRobo teacher uses the same selected grasp and physical
  acceptance. `--reverse-planner auto` tries cuRobo first, then GA after an
  exhausted search, physical rejection or timeout. Execution errors and corrupt
  evidence stop the stage. `ga` remains the default for new plans.
- Feed the verified reverse-teacher states directly into PPO. No forward GA or
  successful forward command replay is required before learning.
- Build at most 32 full-state snapshots by physically replaying the reverse
  prefix; preserve measured velocities and restore leg PD. PPO starts near Wrap
  and extends its overlapping reset frontier toward the reverse Home endpoint.
  At the final frontier, half of resets use original Home and the original
  object placement. PPO must learn any difference from the reverse endpoint.
  **After reset, every action is PPO. There is no scripted Home connection,
  trajectory imitation target, or reversed-velocity assumption.**
- PPO controls all four action channels. q3/q4 hints and bounded post-lift
  command-variation cost are inherited. Linear use is limited to 200 motor
  degrees, and the entire successful five-second hold requires linear 0±3
  motor degrees. Measured lift/roll and loaded contact are still required.
- Three fresh original-Home policy successes and three command replays gate each
  anchor. Completing an update budget is not a success certificate.
- Interpolation blends physical-event-aligned four-axis command paths in q space,
  not neural parameters. Every retimed anchor is replayed first. Vertex tests,
  geometrically valid edge/centroid probes and disjoint validation/test samples
  determine publication. Invalid geometric regions are not assumed filled.
- A `resolve` result is an open-loop candidate for a previously untested point;
  sampled validation cannot certify a continuous region. No extrapolation or
  silent nearest-diameter fallback is provided.

## Commands

```bash
export PYTHONPATH=payload/runtime/common/protocol:payload/runtime/docker/sim/app:.
# Pin the assigned GPU by UUID, not a changing physical index.
export CUDA_VISIBLE_DEVICES=GPU-a40782b2-881c-e285-2a8e-66cae9162a01
export OMP_NUM_THREADS=4 PYOPENGL_PLATFORM=egl
PY=.venv/hug-rl-20261106/bin/python

$PY -m workbench.research.experiments.hug_pipeline plan \
  --mapper /path/to/mapper.html --diameter 100 \
  --output workbench/evidence/generated/hug-pipeline-d100

$PY -m workbench.research.experiments.hug_pipeline run \
  workbench/evidence/generated/hug-pipeline-d100

$PY -m workbench.research.experiments.hug_pipeline status \
  workbench/evidence/generated/hug-pipeline-d100

$PY -m workbench.research.experiments.hug_pipeline resume \
  workbench/evidence/generated/hug-pipeline-d100

$PY -m workbench.research.experiments.hug_pipeline resolve \
  workbench/evidence/generated/hug-pipeline-d100 \
  --x -100 --y 180 --output workbench/evidence/generated/hug-query
```

`plan` does not use the GPU. Default budgets: 128 GA candidates, 100 reverse
generations, 512 PPO environments, 2000 updates, 5 selection
attempts, 30 validation and 100 test samples. Workers run sequentially; the three
policies do not each claim 512 environments concurrently. `--stop-after` supports
screen, reverse, check, train or policy for bounded acceptance tests.
It is a diagnostic stop, not publication readiness.

## Use cuRobo before GA

Create a **new** immutable study using the existing isolated V2 environment and
the fitted, task-matching model:

```bash
$PY -m workbench.research.experiments.hug_pipeline plan \
  --mapper /path/to/mapper.html --diameter 100 --output NEW_STUDY \
  --reverse-planner auto --curobo-model /path/to/model \
  --curobo-python /path/to/v2/bin/python \
  --curobo-profile task --curobo-attempts 4 --curobo-timeout 600

$PY -m workbench.research.experiments.hug_pipeline run NEW_STUDY --stop-after teachers
```

`teachers` checks reverse evidence, prepares the PPO input and physically rebuilds
the full-state reset bank for **all three** anchors, then stops before training.
The check uses the study's `--num-envs`; a small diagnostic study does not prove
the production training batch. `--reverse-planner curobo` disables GA fallback.
Use one pinned GPU UUID in `CUDA_VISIBLE_DEVICES`. The selected profile, attempt
count, whole-attempt time budget, V2 source and model metadata hashes are pinned
in the study. Each attempt uses native whole-path planning with no authored
waypoint or release prefix. A timed-out attempt's own process group is stopped
before fallback starts.

`task` uses strict cylinder clearance. `task-contact` approximates permitted
moving-link contact using an eroded cylinder core; the full cylinder protects
housing/wedge/camera and the floor remains checked. Exact contact face/depth and
free-object dynamics are always checked in Genesis. `task-contact-roll` is an
experimental monotone-roll search subset, not a verified universal solution.

In `auto` mode, explicit `--reuse-reverse` GA inputs are fallback material:
cuRobo is still tried first. Reuse validates the original reports and all saved
replay paths, preserving their JSON/NPZ bytes. Otherwise GA runs its normal
bounded search. A cached fallback is recorded as `search_executed: false`; its
short import time must not be reported as fresh GA search performance.

Each reverse stage saves `reverse-selection.json` with the actual producer,
attempt statuses and wall times. A version-one `hug-reverse-teacher` manifest
in `plan/teacher.json` pins the selected source's artifacts. Native cuRobo
results retain their candidate directory and original provenance rather than
being relabeled as GA generations. Both producers enter the existing PPO
reference validation and physical reset-bank check. No PPO update or learned
task success is implied by teacher acceptance.

The 2026-11-13 GPU2 integration check used the same `task` profile, two proposals
and a 300 s whole-attempt limit for three fixed D100 anchors. Case 02 selected a
fresh native cuRobo path; cases 01 and 03 selected explicitly supplied verified
GA evidence. All three PPO preparations and two-environment, 32-bin physical
reset-bank checks passed. The run stopped at `teachers` with zero PPO updates.
Total time was 620 s; this includes cached GA import rather than fresh GA search.
The native case's reverse stage took 196.4 s and its bank check took 32.9 s.
Host focused checks passed 57 tests and server checks passed 67. See the
[pinned inputs and results](../../../evidence/curated/planning/curobo_teacher_pipeline_20261113.json).

## Restart and evidence

`manifest.json` records stage inputs, output hashes, elapsed times, PID and GPU.
Each attempt runs in a fresh subprocess and directory. Completed matching stages
are reused only after verifying their artifacts. Interrupted learning resumes
from its latest intact model/curriculum pair with the remaining update budget.
An exhausted physical/search/policy gate is preserved as failure, not retried
forever. Reverse failures may exclude that q anchor and reselect within
the configured budget. Execution errors require correcting the environment and
resuming; changed source/model requires a new study, not editing provenance.

The mapper kernel and its collision contract are copied into the study. Existing
studies, NPZ policies, checkpoints and raw videos are never overwritten. Distinct
studies/diameters keep separate roots. The coordinator takes a study lock and
will not steal a still-running worker. It never kills another study, Newton-Gym,
or an unrelated GPU process. Operators must allocate the GPU before starting.

## Linear budget and final pose

Linear motor position is `u = -1500 * linear_m`: Home is 180 degrees
(-120 mm), and the solver/final target is 0 degrees (0 mm). Each physics
tick charges `abs(u_command_next - u_command_previous)` after command limits.
The 200-degree limit includes approach, lift and the entire hold. Exceeding it
latches a fault, suppresses positive rewards and terminates at the control-step
boundary with the existing -30 fault penalty. Reversing direction cannot refund
spent travel. q3/q4 acquire no new cumulative travel limit.

A curriculum reset at `u_reset` starts with `abs(u_reset - 180)` already spent.
Thus nominal Home receives 200 degrees remaining, while a solver-grasp reset at
0 receives 20. This is a minimum-distance debit, not an imposed Home connector
or the reverse teacher's accumulated path length. Infeasible reset bins are
rejected during plan preparation. Physical teacher replay builds the state bank
without PPO budget accounting; after a snapshot is restored, its command is
the starting reference for new travel accounting. Velocities are preserved.

Both measured and commanded linear positions must stay within 3 motor degrees
(2 mm) of zero throughout the five-second hold. Leaving this window resets the
hold timer and removes the verification-time exemption. Lift progress may still
be earned before reaching this endpoint; completed-hold and success rewards
cannot. The remaining fraction `(200 - spent) / 200`, clipped to [0,1], is the
21st actor observation. Training, Home policy evaluation and command replay all
use the same budget/hold environment. Interpolated commands are checked against
the cumulative limit and final target before their physical evaluation.

The reward coefficients remain: q3/q4 potential coefficient 1, first grasp +3,
record lift progress up to +5, record qualified hold up to +10, success +30,
wrong face -1/s with positive rewards blocked, drop -3, fault -30, timeout -5,
time -0.3/s outside qualified hold, action -0.001 times squared normalized
actions, workspace up to -0.2/s, and post-lift command variation up to -1 per
episode. No extra linear-distance reward is added.

New PPO plans are version 3 and budget-aware weights use
`elesim-hug-pipeline-linear-policy` version 1, read by `hug_pipeline.policy`.
Old 20-input policies/checkpoints cannot be resumed as these 21-input policies.
Prepare a new study and reuse the verified reverse evidence; the reverse-GA
implementation and its evidence hashes are unchanged by this PPO constraint.

Per-stage `attempt-*.log` contains the actual worker output. Reverse GA preserves
its ten-generation parade and final videos; final
policy/interpolation evaluation produce caption-free arm-side videos where
configured. Publication and `interpolation.json` distinguish policy success,
command replay success and sampled interpolation readiness.

## Verification boundary

CPU contracts are in `workbench/tests/research/experiments/test_hug_pipeline.py`.
Current live results belong in `docs/status.md`; code availability is not proof
that a fresh diameter has learned three successful policies. The task can stop
with admission failure, reset-bank failure, training failure or interpolation
failure. None is an impossibility proof.

`plan --reuse-reverse /path/to/verified/reverse-plan` optionally pins an existing
release result. Reuse requires identical physical case/root, settings, Home,
model and reverse-GA implementation, plus successful revalidation of the winner
and all three release replays. The original JSON/NPZ provenance is copied intact;
triangle-specific barycentric labels do not define a physical case. A fresh
physical reset-bank check must still pass. No old PPO is required.
Old pipeline plans/checkpoints use a different reset contract: create a new study
and explicitly reuse verified reverse GA evidence; do not resume the old study.

The catalog screen uses one second of goal settling and a final continuous
0.2-second proper Wrap admission, not just a collision-free static frame.

PPO checkpoints at 500-update intervals also trigger a separate, one-environment
nominal-Home recording. This briefly pauses learning; it does not advance the
curriculum or train on the evaluation episode. A failed progress video is evidence,
not a final publication failure or permission to bypass the final three replays.

To keep a previously selected triangle and its exact held-out cases, pass
`--reuse-selection /path/to/previous/selection.json` together with all three
`--reuse-reverse` directories. This pins the selection by digest, rechecks every
case against the mapper, requires safe Home screening and matching reverse
sources, and disables automatic replacement of a failed anchor. The fresh
per-case reverse reset bank still requires admitted contact and full-state
replay checks; a coarse batched goal screen does not replace that validation.
