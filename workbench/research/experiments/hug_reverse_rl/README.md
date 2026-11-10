> **Historical workflow — retired 2026-11-10.** Do not use the commands below for new training.
> Source remains only where current tasks or regression tests depend on it.
> Use `hug_reverse_ga` and `hug_home_rl`; see [workflow status](../HUG_WORKFLOWS.md).
> The exact pre-retirement sources and instructions are preserved in the local archive inventory.

# PPO reverse curriculum from a verified release path

This research task connects `hug_reverse_ga` output to ordinary four-action PPO.
It starts at a physically admitted solver grasp, learns the existing grasp/lift/
five-second hold task, then moves its reset frontier along the release path toward
Home. The policy controls linear, roll and both segment bends freely; q3/q4 hints
and existing reward-v5 are retained. Release commands are used to rebuild initial
states; they are not actor imitation targets or a scripted forward motion.

The runtime payload remains a general RL library. This cylinder experiment lives
entirely under `workbench/research/experiments/hug_reverse_rl`.

## Commands

From the repository root, using the existing scientific environment on the GPU
host when running physics:

```bash
export PYTHONPATH=payload/runtime/common/protocol:payload/runtime/docker/sim/app:.
PY=.venv/hug-rl-20261106/bin/python
```

CPU-only preparation requires NumPy and access to the verified GA directory:

```bash
"$PY" -m workbench.research.experiments.hug_reverse_rl.experiment prepare \
  --reverse-run workbench/evidence/generated/hug-reverse-ga-legpd-20261109T074421Z \
  --output workbench/evidence/generated/reverse-ppo-d100-plan
```

Preparation validates the selected lane, three independent successful replays,
source/model/physics provenance, tick validity, contact/fault bounds and terminal
Home verification. It packs a portable `reference-path.npz` with full robot q/qd,
commands, cylinder pose/velocities and contacts. Original GA artifacts stay intact.
Default reset spacing is0.25s of the verified path, with the grasp and first
verified Home endpoint included. The redundant last1s Home dwell is excluded.
No physics, actor, optimizer or PPO update is run by `prepare`.

Check the complete reset bank without constructing a learner:

```bash
CUDA_VISIBLE_DEVICES=GPU-a40782b2-881c-e285-2a8e-66cae9162a01 \
OMP_NUM_THREADS=4 PYOPENGL_PLATFORM=egl \
"$PY" -m workbench.research.experiments.hug_reverse_rl.experiment check \
  --plan workbench/evidence/generated/reverse-ppo-d100-plan \
  --output workbench/evidence/generated/reverse-ppo-d100-check --num-envs 2
```

Training is an explicit command, default512 parallel environments:

```bash
CUDA_VISIBLE_DEVICES=GPU-a40782b2-881c-e285-2a8e-66cae9162a01 \
OMP_NUM_THREADS=4 PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1 \
"$PY" -m workbench.research.experiments.hug_reverse_rl.experiment train \
  --plan workbench/evidence/generated/reverse-ppo-d100-plan \
  --output workbench/evidence/generated/reverse-ppo-d100-run \
  --num-envs 512 --iterations 2000
```

`--resume path/to/model_100.pt` resumes weights/optimizer plus the paired
`model_100.pt.curriculum.json` into a **new output directory** under the same plan.
Checkpoint hash, path hash and source hash must match. The curriculum frontier,
outcome windows, cooldown, and reset-sampling RNG are restored. Live episodes
restart; this is not a bitwise continuation of simulator or optimizer RNG state.

Evaluate the final policy at **Home plus the original object placement**:

```bash
CUDA_VISIBLE_DEVICES=GPU-a40782b2-881c-e285-2a8e-66cae9162a01 \
OMP_NUM_THREADS=4 PYOPENGL_PLATFORM=egl \
"$PY" -m workbench.research.experiments.hug_reverse_rl.experiment evaluate \
  --plan workbench/evidence/generated/reverse-ppo-d100-plan \
  --policy workbench/evidence/generated/reverse-ppo-d100-run/policy.npz \
  --output workbench/evidence/generated/reverse-ppo-d100-eval \
  --start nominal-home --repetitions 3 --video
```

Evaluation defaults to one environment; video requires one. `--start fixed
--bin 0` tests the admitted grasp neighborhood; other bin indices test earlier
curriculum starts. Evaluation never advances the curriculum or updates weights.
It requires complete grasp/lift/hold success, with no wrong-face episode contact
or physical fault; failure returns a nonzero exit code.

## Physical state restoration

Every physics invocation rebuilds its own reset bank. It re-admits the free
cylinder at the exact solver grasp, physically replays the recorded reverse
prefix, and captures full Genesis snapshots at selected ticks. Recorded
joint/cylinder positions **and velocities** must match within the explicit plan
tolerances. Each snapshot must restore those states and pass a one-tick physical
safety probe; the initial grasp must retain proper loaded contacts. With two or
more environments, a mixed subset restore must also leave other lanes unchanged.
Any failure blocks training and writes `reset-bank/validation.json`.

Snapshots are process-local. NPZ is portable evidence for rebuilding them, not a
pickle of a Genesis state. Selected environments restore the complete snapshot,
including the recorded velocities; no velocity negation, zeroing, object rebasing
or q-only state reset is used. After restoration the episode clock and reward/
hold history start fresh, and the q potential is initialized from measured q.
The original object reference and geometric target remain fixed.

Genesis resets clear control modes. Every full or partial snapshot restore now
reapplies the standing leg PD target as well as arm control, for all reset bins.
Joint positions and velocities are not teleported. Earlier reverse paths and PPO
plans were recorded with released leg control and must be regenerated; source
hash validation rejects them under this corrected implementation.


## Curriculum progression

- Begin with **only the admitted grasp**. Full lift with measured roll0±5°,
  clearance≥20mm and continuous5s loaded hold is required for a success.
- Default promotion:≥70% clean success over512 completed **frontier** episodes;
  retreat:≤20%. Minimum cooldown80 control steps (five16-step PPO updates).
  These are initial configurable plan defaults, not established optimal values.
- At later stages,80% of path resets use the frontier and20% revisit earlier
  states. Easy-state outcomes and episodes started before the most recent move
  cannot promote the new frontier. Any wrong-face episode contact disqualifies
  the outcome for promotion.
- At the last path stage,50% of resets use Home with the **original nominal
  cylinder**, with the other50% retaining the path mixture. Reverse GA's displaced
  Home endpoint therefore cannot substitute for the original task.
- Training's `nominal_ready` is a progress metric. Explicit policy evaluation at
  nominal Home is still required. Finishing the iteration budget does not assert
  a successful policy or stop early based on that metric.

Settings live in prepared `plan.json` → `curriculum`; they are validated before
use and paired with every checkpoint. Changing them prevents resume from a
checkpoint with another schedule. Reward magnitudes and action semantics are
inherited from the frozen `hug_c` task; old experiments are not rewritten.

## Outputs

`run.json` records the plan and launch arguments. `reset-bank/validation.json`
records physical replay/restore checks. TensorBoard exposes frontier, reverse-path
time, nominal reset fraction and nominal readiness alongside existing task metrics.
Every saved PPO checkpoint has a paired curriculum JSON. `policy.npz` contains the
existing executable20-observation/four-action actor format with the new reset
provenance. `completion.json` distinguishes execution failure from budget end.
Evaluation writes `evaluation.json`, policy rollouts NPZ, and optional caption-free
MP4 videos. It never labels a forward policy successful based solely on reverse GA.

## Historical GPU2 run (completed)

`hug-reverse-ppo-legpd-20261109T144613Z` completed its budget but failed original
Home evaluation. Its recorder/watcher entrypoints have been archived. Keep the
results as boundary-study evidence, not a successful Home policy. Current
recording uses `hug_home_rl.experiment evaluate --video` and
`analysis.export_home_checkpoint`; see the Home task runbook.

## Implementation checks (2026-11-09)

Host checks:48 passed,6 skipped because Torch/tensordict are unavailable. The
existing server scientific environment passed five CPU Torch adapter/model/
checkpoint tests with zero optimizer updates. The real verified100mm path packs
and loads portably. GPU2 physics with two environments validated all21 reset
bins through reverse time4.9s, their contact probes, original Home, and isolated
subset restoration. The bank is rebuilt and checked again at the requested
training parallelism;512-environment training has not been started.

A CPU replay of the GPU-derived reference was deliberately rejected by the
strict quaternion comparison (difference0.00030468 vs tolerance0.0001).
Tolerances were retained; the matching GPU backend passed. Do not assume CPU and
GPU contact trajectories agree within these bounds. Canonical dev/release gates
were not run: the generated development wrapper is unavailable and the host
scientific stack is incomplete. [Recorded evidence](../../../evidence/curated/planning/hug_reverse_ppo_codecheck_20261109.json).
