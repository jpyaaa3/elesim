> **Historical workflow — retired 2026-11-10.** Do not use the commands below for new training.
> Source remains only where current tasks or regression tests depend on it.
> Use `hug_reverse_ga` and `hug_home_rl`; see [workflow status](../HUG_WORKFLOWS.md).
> The exact pre-retirement sources and instructions are preserved in the local archive inventory.

# Independent C-hug GA setup

This research-only runner searches trajectories, never PPO weights. It reuses
HugEnv physics and success criteria without changing the frozen hug_c package.
The current implementation searches **Home-start paths only**, one candidate per
physical lane. It seeds six four-axis knots near the solver goal and evolves
normalized waypoints and segment times with elitism, crossover, mutation and
random immigrants. Commands obey the same bounds and slew rates as PPO.

Prepare (NumPy only, no simulation):

```bash
export PYTHONPATH=payload/runtime/common/protocol:payload/runtime/docker/sim/app:.
python -m workbench.research.experiments.hug_ga.experiment prepare \
  --experiment workbench/evidence/generated/c-hug-concurrent-20261107T1025Z/experiment.json \
  --case c-train-000-d100 \
  --output workbench/evidence/generated/hug-ga-d100 \
  --population 128 --knots 6 --generations 100
```

Explicit simulation launch in the existing pinned Genesis environment:

```bash
CUDA_VISIBLE_DEVICES=GPU-a40782b2-881c-e285-2a8e-66cae9162a01 \
OMP_NUM_THREADS=4 python -m workbench.research.experiments.hug_ga.experiment run \
  --plan workbench/evidence/generated/hug-ga-d100
```

The allocation is 128 simultaneous distinct paths by default; 512 is selectable
at prepare time, but GA throughput/memory has not been GPU-validated. Budget is
100 generations (12,800 candidates), stopping at the first successful generation.
The run refuses reuse of its output. There is no resume yet.

Selection first rejects any forbidden fault or any wrong-face exposure. Physical
success outranks all incomplete progress; success ties prefer shorter duration.
Incomplete safe candidates use hold/lift/grasp progress, then terminal gripper
centre distance. These are provisional search heuristics, not task reward or
certificates. Historical single-touch and q-match do not earn selection credit.
Time reservation at the final knot does not force a physical grasp/lift phase.

Each generation saves genomes, rate-limited macro commands, per-candidate results
and compressed observations/actions/valid masks/object positions/contact roles.
Observations precede actions; object/contact measurements follow actions. Arrays
are time x candidate x feature. Use valid masks to exclude terminated lanes.
Contact roles are macro endpoint samples, not a complete physics-tick contact log.
All records are **search data**, with imitation_eligible=false. A success requires
independent replay and perturbation validation before becoming a demonstration.
No policy NPZ is generated; rollout NPZ is a distinct schema.

Remaining stages of the agreed plan: goal-neighbourhood grasp/lift diagnostic,
object-relative solver reseeding after displacement, physics-tick trajectory
export, independent replay/video verification, perturbation/recovery collection,
and perturbation-based policy validation. See below for the implemented BC/PPO handoff; the remaining stages are not implemented. The initial
q seed is fixed; no q objective continues pulling toward a displaced object's
old target. Run settings and source digest are pinned in plan.json.

Host tests validate path bounds/slew and selection/breeding. Genesis execution is
still untested; prepare is not evidence of physical feasibility or throughput.

## Verified demonstrations, BC, and ordinary PPO

The `handoff` command implements the downstream handoff. It never invokes the
population/evolution runner. All output directories must be new.

```bash
python -m workbench.research.experiments.hug_ga.handoff verify-demo \
  --experiment "$EXPERIMENT" --rollouts "$GENERATION/rollouts.npz" \
  --candidate 0 --output "$OUT/verified-0"

python -m workbench.research.experiments.hug_ga.handoff bc \
  --experiment "$EXPERIMENT" \
  --demonstrations "$OUT/verified-0/demonstration.npz" \
  --output "$OUT/bc" --epochs 100

python -m workbench.research.experiments.hug_ga.handoff ppo \
  --experiment "$EXPERIMENT" --bc "$OUT/bc" \
  --output "$OUT/ppo" --num-envs 512 --iterations 2000
```

Verification reexecutes the selected actions from Home in a fresh single-lane
scene three times. Any failure or wrong-face exposure refuses demonstration
publication. It records new observations rather than relabeling old search
observations. Three repeats at the same nominal condition are not a robustness
proof. The verified schema is a local workflow gate, not a signature/security
boundary. Only trusted local artifacts should be consumed.

BC accepts only verified NPZs with the same train case and exact simulation
context. It trains the existing 20-input/four-output 256/128/64 ELU actor, computes
observation normalization from demonstration data, and exports `actor.pt` plus
normal NumPy policy NPZ. Logged BC losses are training losses, not held-out
validation scores. The actor checkpoint contains no critic or optimizer state.

PPO verifies both artifact hashes and exact actor/context compatibility, creates
fresh critic/optimizer state, and copies actor weights and normalization. Before
any PPO update, it checks NumPy/Torch action agreement and requires three
successful deterministic Home evaluations across its nominal batch. Failure
writes `bc-replay.json` and exits without training. Success permits normal
single-policy PPO using all 512 lanes. This currently uses the existing reward-v4
contract unchanged; it does **not** implement a new sparse-success reward or
claim to solve reward design. `model_initial.pt` preserves the BC starting point;
`model_final.pt` and `policy.npz` are unvalidated fine-tuning outputs, not automatic
replacements for the BC baseline. Use the existing collect/replay commands to
compare them before adoption. Existing arm-side report recording accepts the
exported policy NPZ.

Host: five GA/data-gate tests pass. Pinned server CPU synthetic-data smoke:
BC loss decreases, actor plus normalizer transfer matches exactly, and exported
NumPy actions match Torch within 2.4e-7. No actual GA search, successful physical
demonstration, BC physical validation or PPO fine-tuning has run. Goal-local
search, dynamic solver reseeding, perturbation curricula, and automated final
policy adoption remain follow-ups. Older prepared GA plans must be regenerated
because the GA package source digest includes this handoff module.

## Whole-population parade videos

Every tenth completed generation (10, 20, ..., 100) records the actual batched
candidate evaluation to `generation-0009/parade.mp4`, `generation-0019/parade.mp4`,
etc. Directory numbers are zero-based. All candidates (128 by default) appear
simultaneously in the same world/grid view, each performing its own candidate
path. This is not a montage or a replay of one winning policy. Population changes
are reflected in the displayed count. A fixed elevated arm-side overview fits
all actual Genesis environment offsets, with a long far clip plane.

Output is 1920x1080, 25 fps at current dt=0.01, normal simulated-time speed.
Only raw camera RGB is written: no captions, labels, scores or other overlays.
Physics ticks drive recording, so slow rendering does not change playback speed.
Termination still masks completed candidate credit; recording ends when all lanes
terminate or the episode limit is reached. If search succeeds before generation
10, no tenth-generation movie is produced. Rendering adds wall time on recording
generations; non-recording generations do not call camera.render().

Eight focused host tests cover the existing GA/data gates, tenth-generation
cadence, all-grid framing and raw-frame recording cleanup. Actual 128-lane GPU
physics and caption-free video passed startup smoke on 2026-11-08. The grid uses
1.8 m drawing offsets with eight world-Y slots (16 x 8 in the camera for128);
these visualization offsets do not change physical lane poses. A frustum-fit
camera uses the full landscape frame. Regenerate prepared plans after source
changes; source mismatches are intentionally refused.


## Detached GPU2 run (2026-11-08)

The user authorized the first Home-start D100 anchor search with128 candidates,
six knots and at most100 generations. Current launch metadata is
`workbench/evidence/generated/hug-ga-active.json`; its run root contains the
source SHA inventory, exact launch-source ZIP, smoke videos, and `ga.log`.
The `search/` child holds `plan.json`, `parade-preview.png`, per-generation
artifacts and `status.json`. The process is session-detached. `completion.json`
records success-found, budget-exhausted or an exception without starting BC/PPO.

Progress includes initialization/generation wall time, candidate counts, rejection
reasons and a recent-generation ETA. Until at least one parade is timed, that ETA
does not account reliably for video overhead. A short actual128-lane check is
also available via `experiment smoke --experiment ... --case ... --output NEW`;
it executes only ten macro actions by default and is not a success certificate.

## Single-candidate review video

Record one saved candidate from Home in a fresh single-lane scene:

```bash
python -m workbench.research.analysis.record_ga_candidate \
  --experiment "$RUN/search/experiment.json" \
  --rollouts "$RUN/search/generation-0013/rollouts.npz" \
  --candidate 124 --output "$RUN/candidate-124-review"
```

It records the exact valid saved action prefix with the arm-side report camera,
raw RGB only. Outputs are `candidate.mp4`, per-physics-tick `trajectory.npz` and
`report.json`, including source hash, action count, and replay success/faults.
Exit1 still leaves a failed replay video for review. One successful replay does
not approve imitation data: use the separate three-repeat verify-demo command.
The report helper is outside the frozen GA source digest.


## Current full90-degree lift criterion

New runs use the shared lift90 contract: rotate from nominal wrap roll-90° to
raised roll0° ±5°, with loaded clearance and retention for five continuous
seconds at that roll. GA and PPO use the same HoldTracker and RewardLedger.
GA lift progress is limited by both actual clearance and measured roll progress;
partial roll cannot bank full lift/hold/success. Reports include measured lift
roll error and current posture-complete status. Initial candidates now use the
solver grasp pose followed by a roll0 lift pose; mutation can change every knot
and duration, so this is a seed, not a mandatory motion schedule.

Prepared fresh inputs (no search/training launched):
`workbench/evidence/generated/hug-ga-lift90-20261108/experiment.json` and `search/plan.json`.
Old candidate124 and its video belong to the previous partial-lift contract;
its final measured roll near-55° does not pass this new full-lift condition.
Old experiment/policy versions and context-mismatched GA/BC data are rejected.
Use a new experiment/plan and revalidate demonstrations before PPO handoff.
Earlier reward-v4 descriptions above apply only to the archived first run;
current handoff follows the experiment's reward-v5 contract.

## Optional object XY displacement rejection

To test a compact grasp without changing PPO rewards, prepare a new GA plan:

```bash
python -m workbench.research.experiments.hug_ga.experiment prepare \
  --experiment "$EXPERIMENT" --case c-train-000-d100 --output "$OUT/search" \
  --population 128 --knots 6 --generations 100 --seed 42 \
  --max-object-xy-mm 50
```

Plan schema2 pins this GA-only constraint separately from the physical task
context. New motion contracts (`hug-ga-object-motion-v2`) apply the limit only
while the object is grounded, checked at **every physics tick**. An object is
airborne only when it has no loaded floor contact **and** its oriented cylinder's
lowest point has positive floor clearance. Airborne horizontal motion is exempt;
lifting does not require reaching the final raised roll before this exemption.
Floor contact or nonpositive clearance reactivates the limit after landing.

Default `None` disables rejection. The metric is Euclidean world-XY distance
of the object's centre from its **initial** centre; landing does not reset the
reference. Height, orientation and total distance travelled are excluded.
Exactly50mm is allowed; greater than50mm while grounded disqualifies
irreversibly, including a transient excursion within the same control step.
Lifting or returning inside the radius cannot erase an earlier ground violation.
Completed lanes stop receiving constraint/progress updates. A rejected candidate
stops being evaluated at the next macro boundary and cannot win through other scores.

Reports distinguish peak displacement over the whole episode
(`max_object_xy_displacement_mm`) from the peak while grounded
(`max_ground_object_xy_displacement_mm`), and include the first violating tick
and explicit `object_motion_limit` reason. A large airborne peak alone does not
violate a v2 contract. Rollout metadata preserves the constraint, and three-repeat
demonstration verification reenforces it. BC inputs must have the same constraint;
the BC record keeps that provenance. This does not change the PPO task reward or
impose a new PPO runtime motion boundary.

The single-candidate video helper also accepts `--max-object-xy-mm 50` for a
counterfactual replay of a previous unconstrained winner. It automatically
honours source constraints and refuses a weaker override. Historical v1 contracts
retain their original whole-episode limit, including airborne motion; a numeric
override on such a bounded recording also retains its v1 scope. Existing results
are not relabeled as ground-only results. Prepare a fresh plan for the new source
and v2 contract; no search is launched by preparing it. This experiment retains
the same Home start, full90-degree lift and five-second hold,20s episode,
population, seed and fitness rules.
