# Reverse GA: solver grasp → Home

Repository-only experiment. Given **mapper-frame x, y, diameter in mm**, solve
an exact C-band grasp, validate its free-cylinder physics, and evolve the release
path to the real Home preset. No triangle or stored mapper point cloud is needed.
No PPO starts automatically. This produces curriculum candidates, not a claim
that reversed actions grasp/lift successfully.

## Prepare (CPU, NumPy + Node.js)

From the repository root:

```bash
export PYTHONPATH=payload/runtime/common/protocol:payload/runtime/docker/sim/app:.
python3 -m workbench.research.experiments.hug_reverse_ga.experiment prepare \
  --mapper ../hugmap/mapper.html \
  --x=-113.3127080913172 --y=177.58581371059853 --diameter 100 \
  --mu-max 0.4 --population 128 --generations 100 \
  --output workbench/evidence/generated/reverse-d100
```

Current scope is the existing C-band solver: mapper x≤0, y≥0, roll −90°,
linear0. These are **not world coordinates**. q3/q4 are total segment bends;
command arrays at the physics boundary divide them by five. Static collision and
Home-to-Extend preparation CAD filters are inherited from the solver; they may
exclude solutions outside that envelope and do not certify dynamic reachability.

`--seed-step` (default3°) samples analytic branches to seed exact XY inversion;
it does not quantize the requested XY. Among returned feasible inverse solutions,
minimum mu_min is selected and all alternatives are saved. No solution is recorded
as `solver_no_solution`; this is a search failure, not an impossibility proof.
The geometric cutoff must be below the fixed physical arm friction0.5.
Home comes from the mechanism preset. Example reproduces q3≈−89.5°, q4≈−162.5°,
mu_min≈0.17658. Other x/y/D values use the same command.

## Run (explicit, pinned Genesis environment)

```bash
CUDA_VISIBLE_DEVICES=GPU-a40782b2-881c-e285-2a8e-66cae9162a01 \
OMP_NUM_THREADS=4 PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1 \
.venv/hug-rl-20261106/bin/python \
  -m workbench.research.experiments.hug_reverse_ga.experiment run \
  --plan workbench/evidence/generated/reverse-d100
```

This is an example for the established GPU2 host; no remote execution is implied
by preparation. Source/model changes require a fresh plan. Existing output cannot
be overwritten/resumed. Physical admission or runtime errors stop this attempt.

1. Place a free cylinder at nominal XYD and exact solver q; settle1s. Require
   proper loaded contacts on both groups for the last0.2s, accurate tracking,
   stable floor-supported object and no rejection. Otherwise stop as
   `initial_grasp_invalid`, with zero GA generations. No hidden q preload.
2. Snapshot the complete admitted simulation state. Every batch restores it and
   checks joint positions/velocities and object pose/velocities against admission.
3. Evolve6 intermediate knots plus searchable timings; fixed grasp start and Home
   endpoint. Each lane has20s including an event-triggered1s Home verification.
   Commands are rate-limited every10ms. There is no fixed last-third hold.
4. Select success, then Home completion over budget, then incomplete paths, then
   immediate-stop failures. Crossing a travel cap disqualifies a candidate but
   evaluation continues to Home plus1s rest, the20s timeout, or a hard stop.
   Over-budget completions rank by smallest worst relative group excess, then
   total relative excess. Incomplete paths rank by current Home verification and
   terminal Home error, then budget excess. Budget-disqualified paths can breed
   but cannot trigger success verification or enter a PPO reset pool.
5. Check faults, ZED-side contact, whole-episode50mm XY excursion and10° tilt on
   every physics tick. Grasp loss/opening/sign changes are legal. Home requires
   measured pose/rest, no robot contact, and a stable floor-supported cylinder.
6. A search success triggers three fresh nominal admissions/replays (each across
   the batch). All must pass before `reverse_path_verified`. Failed verification
   ends as `verification_failed`, without exporting a validated reset pool.

See [criteria](../../../../docs/hug-reverse-ga-criteria.md) for thresholds.
Forward `hug_ga` and PPO rewards/termination are unchanged and unused for selection.

## Artifacts and next stage

- `plan.json`, `mapper-kernel.js`: input, exact solutions, selected endpoint,
  source/model hashes, physics, Home and search budget.
- `run/admission/`: contact/tracking/drift report and tick states; initial video.
- `run/generation-NNNN/`: genomes, commands, selection reports and `states.npz`.
- First and every tenth generation: caption-free128-lane `parade.mp4`.
- `run/progress.json`, `completion.json`: atomic status and final outcome.
- `run/verification-N/`: independent admissions/replays and videos.
- `run/curriculum.json`: successful lane and path reference, replay result;
  explicitly does **not** assert forward grasp/lift or a validated reset pool.

NPZ stores10ms samples of full robot joint q/qd, actual arm q/qd, commands,
object pose/linear/angular velocities, proper/wrong-face/floor contacts,
contact force/penetration, violation masks, active masks and timestamps. The
sample at index0 is before motion; index k is after physics tick k. `active[k]`
includes the terminating tick; subsequent lane samples must be ignored. Mask
columns are listed in `provenance.json`. Snapshot restores are process-local;
portable curriculum resets must replay the admitted reverse prefix and validate
its state. Do not restore only q, negate velocities, or rebase object placement.

`active` includes budget-disqualified candidates that are still being evaluated;
it does not certify eligibility. Reports distinguish `completed`, `success`,
`rejected` and `hard_stopped`, and save total travel, per-group excess degrees and
relative excess. `first_violation_tick` remains the first violation even if more
violations occur later. `last_valid_error` is the historical pre-violation error;
selection uses terminal `home_error`, or `last_safe_error` for immediate stops.
Progress includes completion and over-budget completion counts. The policy tag
is `reverse-completion-before-budget-ranking-v2`; fresh source-pinned plans are
required, since previous truncated reports cannot establish full-path travel.

PPO curriculum integration remains a separate task. Near-grasp RL must first
learn grasp-and-lift, then expand resets toward Home. Final validation includes
Home with the **original** object position, even if reverse GA displaced it.

## Verification status

Pure rules and mocked admission/restore tests run on host without Torch/Genesis.
Exact solver preparation has been exercised at the example XYD. The first GPU2 run (2026-11-09) admitted128/128 solver grasps, exercised full-state
restore, rendered the initial video and completed two search generations. A
partial inference-scope reset bug was corrected by using `no_grad` for each tick.
Host tests:23 passed,1 skipped (Torch unavailable); the server Torch CPU regression
passed. Search success and independent successful replays remain unproven. See
[execution evidence](../../../evidence/curated/planning/hug_reverse_ga_gpu2_launch_20261109.json).


The corrected leg-hold run on2026-11-09 (`hug-reverse-ga-legpd-20261109T074421Z`)
passed three independent nominal replays with standing leg PD restored after
snapshot reset. An early successful search now also records
`run/final-population/parade.mp4`, showing the actual diverse population.
`python3 -m workbench.research.analysis.record_reverse_ga --source RUN --output NEW_DIR`
records the verified winner alone and reports measured leg speed/target error.
[Corrected run evidence](../../../evidence/curated/planning/hug_reverse_ga_legpd_result_20261109.json).

Initial roll direction lock: the first nonzero roll command chooses either sign.
Until 180 motor-position degrees have been accumulated in that direction, a
reversal permanently rejects the candidate (initial_roll_reversal). Pauses are
allowed. The lock releases once per trajectory; the total roll budget remains
220 motor-position degrees. 180 motor degrees correspond to 90 physical degrees
with the current mapping.
