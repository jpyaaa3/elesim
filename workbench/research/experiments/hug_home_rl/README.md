# Home-prefix PPO and bounded lift smoothness

This external research task makes the two changes following the boundary study:
train the existing policy back to original Home using the verified **forward**
Home-to-handoff prefix, and charge a small, episode-bounded command-variation
cost during lift/hold. The frozen `hug_c`, `hug_reverse_ga` and `hug_reverse_rl`
tasks and their artifact hashes remain usable. No runtime role imports this task.

The actor still has20 observations and four actions: linear, roll, total q3 and
total q4 increments. The geometric hint still targets q3/q4 only. The original
C-grasp,90-degree lift and continuous five-second loaded hold remain necessary.

## Prepare and verify

From the repository root, on the existing scientific GPU host:

```bash
export PYTHONPATH=payload/runtime/common/protocol:payload/runtime/docker/sim/app:.
export CUDA_VISIBLE_DEVICES=GPU-a40782b2-881c-e285-2a8e-66cae9162a01
export OMP_NUM_THREADS=4 PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
HUG_PY=.venv/hug-rl-20261106/bin/python
HUG_PLAN=workbench/evidence/generated/home-ppo-plan

"$HUG_PY" -m workbench.research.experiments.hug_home_rl.experiment prepare \
  --source-run workbench/evidence/generated/hug-reverse-ppo-legpd-20261109T144613Z \
  --verified-bridge workbench/evidence/generated/hug-reverse-home-bridge-verified-20261110 \
  --output "$HUG_PLAN"

"$HUG_PY" -m workbench.research.experiments.hug_home_rl.experiment check \
  --plan "$HUG_PLAN" --output workbench/evidence/generated/home-ppo-check \
  --num-envs 512
```

Preparation is NumPy-only. It checks all three verified Home replays, the initial
checkpoint pairing, the exported initial actor, full-state preservation reports,
the original Home/object start, every recorded command rate and the clean
floor-supported prefix. It copies the complete reference NPZ and initial model/
policy into a portable directory. Subsequent loading requires matching source,
model and artifact digests. An old plan is never relabeled as this task.

The reference time increases0..1.1s. Curriculum bins run in the opposite order:
handoff first, Home last. Default spacing0.05s gives23 snapshots. The bank is
bounded to32 snapshots; `--spacing-s` must respect that bound. The full10ms
record remains available for validation and future narrower-window preparation.

Each physics process replays from nominal Home and captures full Genesis states,
preserving actual joint/object velocities. It checks recorded-state agreement,
all restores and one-tick safety. With multiple environments, resetting a subset
must also preserve untouched lanes through their **next physics tick**, including
their standing leg control. No teacher commands are executed during learned
episodes; the prefix constructs initial states only.

The reset window overlaps the previous two bins, with solved-start retention.
Only outcomes at the newly introduced hard edge and current curriculum epoch
can promote it. Earlier/easier successes cannot do so. At the final stage,
original Home is sampled explicitly with50% probability. The inherited default
evidence gates remain70% promotion/20% retreat over512 frontier episodes, with
an80-control-step cooldown. Configuration and RNG state accompany checkpoints.

## Train, continue and evaluate

Start a short pilot explicitly:

```bash
"$HUG_PY" -m workbench.research.experiments.hug_home_rl.experiment train \
  --plan "$HUG_PLAN" --output workbench/evidence/generated/home-ppo-pilot \
  --num-envs 512 --iterations 200
```

The default training budget is2000 updates when `--iterations` is omitted. A
new run initializes actor, critic and their observation normalization from the
pinned old checkpoint. It checks NumPy/Torch actor equivalence, starts a fresh
optimizer and curriculum, and writes `warm-start.json`. It does not resume the
old GA frontier or optimizer. Every checkpoint is paired with the new plan's
source/reference/initial-model hashes and curriculum state. Reads use
`torch.load(weights_only=True)` and strict model loading.

`train --resume NEW_TASK_CHECKPOINT.pt` restores that checkpoint's weights,
optimizer and curriculum into a **new output directory** under the exact same
plan. Live episodes restart with fresh task and smoothness histories; this is
not a bitwise simulator/RNG continuation. Earlier experiment sidecars are
rejected.

The final actor is an executable `policy.npz`. Validate **without a scripted
approach** from the original Home and object placement:

```bash
"$HUG_PY" -m workbench.research.experiments.hug_home_rl.experiment evaluate \
  --plan "$HUG_PLAN" --policy workbench/evidence/generated/home-ppo-pilot/policy.npz \
  --output workbench/evidence/generated/home-ppo-eval \
  --start nominal-home --repetitions 3 --video
```

`--initial --start fixed --bin 0` evaluates the pinned starting actor at the
new handoff state. `--initial --start nominal-home` records its unassisted Home
baseline; failure is expected before the new approach is learned. Evaluation
never changes weights or advances the curriculum. Video requires one environment
and uses the arm-side camera. Outputs include evaluation JSON, action/command
rollout NPZ, actual smoothness costs and optional caption-free MP4.

Intermediate checkpoints can be exported without a PPO update:

```bash
"$HUG_PY" -m workbench.research.analysis.export_home_checkpoint \
  --plan "$HUG_PLAN" --checkpoint path/to/model_100.pt \
  --output workbench/evidence/generated/home-ppo-report-100
```

Use that report's `policy.npz` with `evaluate --video` from nominal Home and/or
`--start fixed --bin FRONTIER`. `export.json` records the frontier and zero
training updates. `warm-start-policy.npz` in a training directory is only the
actor-equivalence audit probe; the published final/report policies carry full
task/reference metadata.

## Smoothness cost

Let `u` be the **applied** command increment divided by the existing per-control-
step movement cap of each axis. Saturated raw neural outputs are not used.
With deadband0.1, the bounded intensity is

```text
mean(clip((abs(u_current - u_previous) - 0.1) / 1.9, 0, 1)^2)
```

A confirmed proper grasp that becomes floor-free with at least5mm clearance
arms the cost, observed at physics ticks. Its first qualifying control step is
free. Once armed, contact loss or leaving the roll/hold tolerance cannot disable
the cost. Termination/reset ends that history.

At each active control step, charge at most
`budget * control_dt / 5s * intensity`, limited by the remaining episode budget.
Default `budget=1.0`; preparation accepts0..1 through `--smooth-budget`. Thus
total smoothness deduction cannot exceed one point, compared with the existing
30-point success bonus. Constant applied increments and small corrections incur
zero variation cost. Actual zero motion remains free after the stopping transient.
Approach and ground wrapping do not arm this cost.

This is reward shaping only: no command filter, hold script or fixed roll
schedule is introduced. The code change does not by itself establish that a
trained policy will stop oscillating or reach Home. Those are learning/evaluation
results to measure separately.

See [the boundary study](../../../../docs/hug-reverse-ppo-boundary-study.md) for
the original guided result and the reason the forward reference was selected.

## Implementation verification (2026-11-10)

Host focused tests:59 passed,9 skipped because Torch/tensordict are unavailable.
The pinned server environment passed four actual CPU Torch/RSL tests, including
safe actor/normalization transfer, a fresh optimizer and paired resume.

GPU2 validated all23 reset states at both2 and512 environments, including the
next-tick behavior of untouched lanes after subset resets. The initial actor at
the new handoff succeeded3/3 with the new cost; each episode paid0.734705 points
and returned44.2412. Original-Home initial-policy evaluation failed as expected,
with zero smoothness cost before lift. This confirms the baseline and cost
placement, not that a new Home approach or smoother motion has been learned.

The real runner performed one16-environment update and one resumed update;
curriculum steps resumed16→32. Actor/normalization transfer error was0.0. New
checkpoint→NPZ export and arm-side video replay also passed. The full learning
run has not started; GPU2 test processes have exited. Canonical dev/release gates
were not run because the generated development wrapper is unavailable.

[Recorded evidence](../../../evidence/curated/planning/hug_home_ppo_two_changes_codecheck_20261110.json).
