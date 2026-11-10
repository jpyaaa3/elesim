# Solver → reverse GA → Home PPO → interpolation

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
- Replay reversed commands from original Home/object placement. If needed, search
  intermediate knots and durations with policy-free forward GA. Accept only
  safe motion and 0.3 s stable proper two-segment contact at the selected goal,
  repeated three times. The reference is real forward physics, not reversed qd.
- PPO starts cold at the verified Wrap end and extends its reset frontier back
  through at most 32 full-state snapshots to original Home. Leg PD is restored.
  **After reset, every action is PPO. There is no scripted Home connection.**
- q3/q4 hints, free linear/roll and bounded post-lift command-variation cost are
  inherited. Success still requires measured lift/roll and five-second retention.
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

`plan` does not use the GPU. Default budgets: 128 GA candidates, 100 reverse and
100 forward-repair generations, 512 PPO environments, 2000 updates, 5 selection
attempts, 30 validation and 100 test samples. Workers run sequentially; the three
policies do not each claim 512 environments concurrently. `--stop-after` supports
screen, reverse, forward, check, train or policy for bounded acceptance tests.
It is a diagnostic stop, not publication readiness.

## Restart and evidence

`manifest.json` records stage inputs, output hashes, elapsed times, PID and GPU.
Each attempt runs in a fresh subprocess and directory. Completed matching stages
are reused only after verifying their artifacts. Interrupted learning resumes
from its latest intact model/curriculum pair with the remaining update budget.
An exhausted physical/search/policy gate is preserved as failure, not retried
forever. Reverse/reference failures may exclude that q anchor and reselect within
the configured budget. Execution errors require correcting the environment and
resuming; changed source/model requires a new study, not editing provenance.

The mapper kernel and its collision contract are copied into the study. Existing
studies, NPZ policies, checkpoints and raw videos are never overwritten. Distinct
studies/diameters keep separate roots. The coordinator takes a study lock and
will not steal a still-running worker. It never kills another study, Newton-Gym,
or an unrelated GPU process. Operators must allocate the GPU before starting.

Per-stage `attempt-*.log` contains the actual worker output. Reverse GA preserves
its ten-generation parade and final videos; forward verification and final
policy/interpolation evaluation produce caption-free arm-side videos where
configured. Publication and `interpolation.json` distinguish policy success,
command replay success and sampled interpolation readiness.

## Verification boundary

CPU contracts are in `workbench/tests/research/experiments/test_hug_pipeline.py`.
Current live results belong in `docs/status.md`; code availability is not proof
that a fresh diameter has learned three successful policies. The task can stop
with admission failure, reference failure, training failure or interpolation
failure. None is an impossibility proof.

`plan --reuse-reverse /path/to/verified/reverse-plan` optionally pins an existing
release result. Reuse requires identical physical case/root, settings, Home,
model and reverse-GA implementation, plus successful revalidation of the winner
and all three release replays. The original JSON/NPZ provenance is copied intact;
triangle-specific barycentric labels do not define a physical case. A fresh
forward reference must still pass at the current case. No old PPO is required.

The catalog screen uses one second of goal settling and a final continuous
0.2-second proper Wrap admission, not just a collision-free static frame. During
forward repair, once q is within the declared pose tolerance, actual contact
progress and cylinder position outrank sub-tolerance q noise. The acceptance
criterion remains 0.3-second stable Wrap and three successful fresh replays.

PPO checkpoints at 500-update intervals also trigger a separate, one-environment
nominal-Home recording. This briefly pauses learning; it does not advance the
curriculum or train on the evaluation episode. A failed progress video is evidence,
not a final publication failure or permission to bypass the final three replays.
