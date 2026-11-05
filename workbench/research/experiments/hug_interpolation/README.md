# Example: learn and interpolate cylinder-hugging trajectories

Train a few representative motions, then test whether nearby placements can use
interpolated trajectories instead of separate RL runs. This example uses the
same robot assets as EleSim and runs directly in Genesis, without starting
Pilot, UI, Robot, DDS, or a physical robot.

**Status:** experimental. Offline tests pass; GPU training and physical replay
have not been validated in this checkout. Start with the physics check below.
No trained checkpoint or success-rate claim is included.

## What is included

- `example_pilot.json`: 30 actual geometric solutions selected from a Hugmap v3
  table, at cylinder diameter 150 mm. Six train anchors, 22 validation cases,
  and two held-out test cases. Source SHA-256 and map coordinates are retained.
- `environment.py`: the external two-action task, reward, reset and contact/hold
  criterion. Band 1 uses roll −90°; band 2 uses roll +90°.
- `experiment.py`: configure, check, train, collect, replay, interpolate, validate.
- `interpolate.py`: local barycentric interpolation of trajectory residuals.
- `configure.py`: URDF/map conversion, task-local robot assets and provenance.

The included data covers **two small triangles**, not both entire bands. It is
static geometry, not evidence of dynamic grasp success. The 82 MB source map,
checkpoints and generated output are deliberately not included. You can run this
example from the small input alone.

## Prerequisites

Use a checkout of this repository and a prepared CUDA training environment.
The pinned stack is Torch 2.12.1, Genesis 1.4.5, tensordict 0.14.2 and
rsl-rl-lib 5.4.0. TensorBoard 2.21.0 is used for training logs.
Installation instructions and the complete protocol are in the
[experiment runbook](../../../../docs/hug-rl-experiment.md).

Run the commands below from the **repository root**, in that environment:

```bash
export PYTHONPATH="$PWD/payload/runtime/common/protocol:$PWD/payload/runtime/docker/sim/app:$PWD${PYTHONPATH:+:$PYTHONPATH}"
hug() { python3 -m workbench.research.experiments.hug_interpolation.experiment "$@"; }
OUT="$PWD/workbench/evidence/generated/hug-example-$(date +%Y%m%d-%H%M%S)"

hug configure \
  --pilot workbench/research/experiments/hug_interpolation/example_pilot.json \
  --output "$OUT/experiment.json"
hug check --experiment "$OUT/experiment.json" --output "$OUT/dependencies.json"
hug check --physics --experiment "$OUT/experiment.json" --output "$OUT/physics-check.json"
```

Continue only if the physics check reports `ready=true`. It checks initial
poses/FK, an ungrasped disturbance control and the PPO/environment interface,
without optimizer updates. Add `--viewer` for a local visual inspection.

## Train

First exercise one case with a short run; 20 iterations are not expected to
produce a successful grasp:

```bash
hug train --experiment "$OUT/experiment.json" \
  --case b1-x-100-y320-d150 --runs "$OUT/smoke-runs" \
  --num-envs 32 --iterations 20
```

Then train the six anchors sequentially, with parallel replicas per anchor:

```bash
hug train --experiment "$OUT/experiment.json" --all \
  --runs "$OUT/runs" --num-envs 256 --iterations 2000
```

These are starting budgets, not a convergence guarantee. Each case gets its own
run directory and checkpoints. The runbook explains memory sizing, resume and
choosing an earlier checkpoint if the final one does not succeed.

## Collect and replay the original motions

```bash
(
  set -e
  for run in "$OUT"/runs/*; do
    [ -d "$run" ] || continue
    cid=$(basename "$run")
    hug collect --experiment "$OUT/experiment.json" --run "$run" \
      --output "$OUT/collected/$cid.json"
    hug replay --experiment "$OUT/experiment.json" \
      --trajectory "$OUT/collected/$cid.json" \
      --output "$OUT/original-replay/$cid.json" \
      --verified-output "$OUT/anchors/$cid.json"
  done
)
```

Commands are recorded at physics ticks and replayed without a policy. Failed
collection/replay returns exit code 1 and preserves the result. Resolve original
replay failures before testing interpolation; do not relabel failed anchors.

## Interpolate and evaluate — no additional RL

```bash
hug interpolate --experiment "$OUT/experiment.json" --anchors "$OUT/anchors" \
  --split validation --output "$OUT/validation-trajectories"
hug validate --experiment "$OUT/experiment.json" \
  --trajectories "$OUT/validation-trajectories" --split validation \
  --output "$OUT/validation-report.json"
```

A failed candidate is a valid experimental result: the report is written and
`validate` returns 1 if any case fails. Once the experimental choices are final,
repeat these two commands with `--split test` and new `test-*` output paths.
Do not use the held-out test cases for anchor selection or training.

The generated artifacts contain the case, model, code and physics context.
Existing result files are not overwritten. Changing those conditions requires a
new experiment; ordinary training continuation uses explicit `--resume`.

## Assumptions and extending the example

The task starts with the arm open away from the object. Linear and roll are
fixed throughout the episode; reaching that initial pose is not trained.
The cylinder is 1.1 m tall and 0.30 kg. The success criterion combines the target
angles, two contact groups, and a one-second hold after a small velocity kick.
It is not a proof of arbitrary force closure, lifting, or hardware safety.
The [runbook](../../../../docs/hug-rl-experiment.md#판정과-모델-가정) specifies all thresholds.

Task-specific entities, observations, rewards and evaluation stay in this
package. Common execution, PPO adaptation, joint mapping and trajectory replay
live in `elesim_sim.rl`. A different shape/task should supply its own environment
and criterion rather than add a cylinder/ellipse/box switch to the payload.

To select new cases from a full v3 map, use `python3 -m
workbench.research.experiments.hug_interpolation.prepare --help`. The full map is
not needed to run the bundled example.
