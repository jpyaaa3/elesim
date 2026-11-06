# C-hug from Home: free four-DoF motion, lift and q-space interpolation

This is a repository-only task in the EleSim RL playground. Its input comes
from **mapper.html's analytic functions and CAD collision kernel**, never from
its embedded `DATA` point cloud or `map.json`. It does not start Pilot, UI,
Robot, DDS, or physical hardware.

Current software checks are host-only. The revised Genesis environment, new
Home-to-lift motions and trained policies have **not** passed a GPU preflight.
Generating cases does not certify their approach or lift feasibility.

## Experiment contract

- Collect D=100, 125, 150, 175 and 200 mm independently, with C targets
  (`q3<0`, `q4<0`), required μ≤0.4. Each diameter has three training anchors,
  30 validation and 100 test cases: **15 initial policies**, not mixed-D RL.
  q3 and q4 mean **total segment bend**, in degrees in the geometric data.
- Select exactly three valid targets maximizing the rasterized area of
  `S ∩ triangle`, not the triangle area itself. Infeasible interior gaps are
  allowed and rejected when generating validation/test cases.
- Default angular raster: 1° total bend. Search uses seeded multistart boundary
  vertex exchange and full-boundary refinement. It reports the best found;
  **continuous global optimality is not proved**. Fractional edge cells and
  four vertical subsamples estimate the intersection area.
- A candidate q may have several XY solutions. The same minimum-μ,
  collision-free selection as the mapper is used; contact feature/root are
  retained in every case. This is not a proof that all these contacts share
  one dynamically successful strategy; interpolation tests that hypothesis.
- Three train anchors, 30 validation and 100 held-out test targets. The latter
  are evaluated from the function inside the q triangle, not interpolated XY.
- The start is the **real Home preset**, resolved through application config
  and `control_u_to_sim_q`: display controls `(180,180,10,10)` give linear
  **−120 mm**, roll **0°**, q3 **−170°**, q4 **+170°** with current defaults.
  Display calibration offsets are zero in this isolated task.
- Go2 root and legs stay fixed. **PPO controls linear, roll, q3 and q4 from
  Home**, with no forced deployment, roll, approach or lift timeline. Zero
  actions retain the Home command throughout the episode. C is a grasp/hold
  condition; Home's positive q4 remains valid.
- The function goal remains in observations and goal-progress reward as a
  hint. Goal shaping ends after first actual two-role C contact so lifting
  can adapt posture. There is no required final linear or roll position.
- Success requires **lifting**: the oriented cylinder's **lowest point**
  stays ≥20 mm above the floor for **5 continuous seconds**, no floor contact,
  hand-relative slip ≤10 mm, proximal and distal/gripper contacts for ≥80% of
  that window, C posture and no fault. Drops, excess slip or loss of C reset
  the window. The episode limit is 20 s; the lift can begin at any time.
  Five seconds is an experiment default, not a universal external standard.
- Defaults: **512 environments**, 0.01 s physics ticks, four solver substeps,
  policy actions every 0.1 s. Per action, increments are bounded by ±5 mm
  linear, ±0.10 rad roll, ±0.15 rad total bend per segment. Physics-tick
  interpolation only moves toward the command chosen by the policy.
- Gains/force bounds, physical bend/roll/linear limits, node spread, protected
  camera/housing/Go2 contacts, self and floor contacts are monitored. Only
  mechanically adjacent joints and fixed terminal assembly pairs are excluded
  from self-contact rejection; there is no blanket Home baseline exception.
  Positive shaping is disabled after a fault.
- Default physics: contact μ=0.5, floor/object μ=0.01 (pair mixing follows
  Genesis), mass=0.30 kg, cylinder height=1.10 m, Go2 spawn=0.32 m. The tall
  cylinder spans the arm plane from the floor; it is an explicit inherited
  test-object assumption. A shorter object needs a matching support/placement
  scene, not just a renamed diameter. `configure --settings` records overrides.

The simulation boundary uses per-node radians:
`theta1=deg2rad(q3)/5`, `theta2=deg2rad(q4)/5`. Actor observations expose total
bend radians; NPZ command channels explicitly retain the Sim API's per-node
`theta1_rad`/`theta2_rad` convention.

Per policy step, rewards are goal-error progress ×4 before first C contact,
0.05 per contact role, up to 2 for a loaded C lift reaching 20 mm clearance,
−0.01×sum(action²), −2 for a persistent fault, and +30/−5 on success/timeout.
Success is checked separately using the complete hold criterion above.

## Prepare and review without training

Use a checkout and the CUDA environment described in
[the original dependency runbook](../../../../docs/hug-rl-experiment.md).
Torch 2.12.1, Genesis 1.4.5, tensordict 0.14.2 and rsl-rl-lib 5.4.0 are pinned.
Do not install the scientific stack into host Python for an offline check.

From the repository root:

```bash
export PYTHONPATH="$PWD/payload/runtime/common/protocol:$PWD/payload/runtime/docker/sim/app:$PWD${PYTHONPATH:+:$PYTHONPATH}"
OUT="$PWD/workbench/evidence/generated/c-hug-$(date +%Y%m%d-%H%M%S)"
python3 -m workbench.research.experiments.hug_c.collection prepare /path/to/mapper.html \
  --output "$OUT" --diameters-mm 100 125 150 175 200 --mu-max 0.4 --step-deg 1
```

This writes `collection.json` and independent `d100/`, `d125/`, etc., each with
`pilot.json`, `catalog.json` and `triangle.html`. No physics or training starts.
The function-only inputs may fail full-robot Home/goal checks; they are not
approach/lift certificates. To prepare a single diameter instead, use a new
output directory with the existing commands:

```bash
python3 -m workbench.research.experiments.hug_c.prepare /path/to/mapper.html \
  --diameter-mm 150 --mu-max 0.4 --step-deg 1 \
  --output "$OUT/pilot.json" --preview "$OUT/triangle.html"
python3 -m workbench.research.experiments.hug_c.experiment configure \
  --pilot "$OUT/pilot.json" --output "$OUT/experiment.json"
python3 -m workbench.research.experiments.hug_c.experiment check --physics \
  --experiment "$OUT/experiment.json" --output "$OUT/preflight.json"
```

Node.js is needed for preparation and the exact XY selector below. The generated pilot is self-contained;
its recorded source path is provenance, not a runtime file dependency. The
bundled `example_pilot.json` is also function-generated and can replace the
`--pilot` input without the original HTML or Node.

Inspect the preview, selected anchors and preflight. It checks actual Home/FK,
10-tick Home and directly placed geometric goals for all three anchors, and
constructs the PPO interface without optimizer updates. The physical preflight
uses the selected object placements, including Home/object interference. It does **not** certify
an approach or lift. Code 1 means physics failure, code 2 missing dependencies;
exceptions are errors, never successful checks.

If preflight rejects an anchor, inspect and preserve the report. A JSON list of
failed `[q3,q4]` total-degree coordinates can be passed to `prepare --exclude-q`
to regenerate and record the excluded candidates. If Home itself is invalid,
fix the model/scene/mapping rather than substituting another start or merely
excluding more targets. Do not train through a failed preflight.

For a wide solution chart, a single failed endpoint can hide more invalid
placements near the Go2 body (which the mapper excludes). The optional
`catalog_preflight` command screens every function-generated raster entry in
64 parallel scenes at both Home and the directly placed goal. Supply that
report to `prepare --catalog-preflight report.json` to select the triangle only
from static-ready cells; the measured physics settings and model must match.
The three selected anchors still undergo the normal independent preflight.
This is static screening, not an approach/lift certificate. On 2026-11-06 the
GPU2 screen passed 1,572 of 1,592 entries; the reselected chart covered about
88.83% of the static-ready raster area.

An optional zero-action baseline video checks that the command stays at Home;
it does not perform a scripted deployment or lift:

```bash
python3 -m workbench.research.experiments.hug_c.experiment collect \
  --experiment "$OUT/experiment.json" --baseline --case c-train-000-d150 \
  --output "$OUT/baseline.npz" --video "$OUT/baseline.mp4"
```

A baseline failure is a measured result, not automatically a geometry impossibility.

To use the bundled function-generated example directly, skip `prepare` and set
`--pilot workbench/research/experiments/hug_c/example_pilot.json` in `configure`
or `study`. Its 1° selection covers about 88.5% of the sampled C solution area;
this figure does not imply dynamic success or coverage at another diameter.

## Run the three-anchor study

This command **starts actual RL** after its own matching physics preflight:

```bash
python3 -m workbench.research.experiments.hug_c.collection run \
  --collection "$OUT/collection.json" --num-envs 512 \
  --report-every 500 --iterations 2000
```

Diameters run sequentially. Use `--diameters-mm 150` for one diameter. A failure
stops before more anchors/diameters are trained. Completed studies are verified
and skipped; failed/incomplete studies are preserved and require investigation.
For a single prepared pilot, use:

```bash
python3 -m workbench.research.experiments.hug_c.study \
  --pilot "$OUT/pilot.json" --output "$OUT/study" \
  --num-envs 512 --report-every 500 --iterations 2000
```

The next study defaults to **512 environments** per the 2026-11-06 utilization
measurement (30 one-second samples: mean 65.6%, median 69%, maximum 73%; memory
3,175/49,140 MiB at 256 environments). This is a next-run preference, not a
measured 512-environment speedup. The old scripted two-action run was stopped;
its checkpoints/videos are preserved and incompatible with this four-action
contract. No revised learning has started. The machine-readable next-run
preference is `gpu2-next-study.json`. Never stop or restart unrelated GPU jobs,
including Newton-Gym.

A settings JSON can be supplied with `--settings`. Output must be new. Each
anchor starts independently from Home; there is no fourth smoke-training case.
The study evaluates every 500 updates, exporting policy/trajectory NPZ, an
oblique-view MP4 and original replay report. It stops that anchor on a successful
policy collection **and three original timed-command replays**. These are repeated
deterministic reset/replay checks, not a statistical success-rate estimate. The first anchor that
exhausts its budget stops the study; it does not blindly train the rest.
Raw `experiment train` likewise requires `--preflight` with a successful report
matching all three anchors and the exact experiment context.

Successful policies are copied to `study/policies/<case_id>.npz` and verified
command paths to `study/anchors/<case_id>.npz`. All three anchors must each
pass three original replays before interpolation. q3/q4 determine barycentric
weights; **all four free command channels** are blended. Shorter originals
are stretched to the longest duration, with no forced phase schedule and no
shortened original final hold. Network weights and XY endpoints are not
averaged. Every candidate still needs physics replay. No optimizer runs for
validation/test. Validation failure
holds out test data and stops. After nominal validation passes, five validation
cases also replay under eight cardinal/diagonal XY offsets each at ±5 and ±10 mm
(80 checks). The commanded path stays unchanged, so these checks measure tolerance
to object-placement error. A robustness failure holds out test data too; failed
test points are not automatically retrained.

Artifacts include `status.json`, logs, checkpoint reports/videos, independent
policies, collected and verified NPZ trajectories, 30 validation candidates and
reports, and finally 100 test candidates/reports. Geometric coverage and dynamic
success counts are separate. Source/model/code/settings/control-mode/Home
provenance must match. Old two-action or fixed-roll policies are incompatible.

## Select a collected diameter and exact XY

```bash
python3 -m workbench.research.experiments.hug_c.collection resolve \
  --collection "$OUT/collection.json" --mapper /path/to/mapper.html \
  --x-mm -100 --y-mm 300 --diameter-mm 150 --output "$OUT/selection.json"
```

The example XY can be rejected by the function. Coordinates are mapper mm
from housing centre: this C chart has x≤0, y≥0; map x=−world Y, y=world X,
with node0 at map `(0,144.5)` mm. The selector requires an exact collected D,
solves the mapper inverse using nearby function samples as feature/root seeds,
rechecks forward XY, μ, CAD collision and q triangle containment, then returns
three policy/trajectory NPZ paths plus q weights. This is anchor selection,
not motor execution or network-weight interpolation. Completed study context,
original replays, validation, robustness and test reports must all match/pass.

Before training, `--geometry-only` previews q and weights without policies.
That output is not a validated motion. A missing inverse solution means the
local search failed, not a proof of impossibility. The rest of the band outside
the chosen triangle remains unsupported.

To add 110 mm later, append inputs using the same source/settings and start
only its independent study when authorized:

```bash
python3 -m workbench.research.experiments.hug_c.collection prepare /path/to/mapper.html \
  --output "$OUT" --diameters-mm 110 --mu-max 0.4 --step-deg 1
python3 -m workbench.research.experiments.hug_c.collection run \
  --collection "$OUT/collection.json" --diameters-mm 110
```

Existing profiles and NPZ files stay byte for byte unchanged. Adding a diameter
changes data, not the model/code/action/observation contract. There is no silent
nearest-D choice or cross-D interpolation yet. Policy NPZ is for inference,
trajectory NPZ for timed replay, and `.pt` retains same-run training state.

## Host verification

```bash
python3 -m pytest -q workbench/tests/research/experiments/test_hug_c.py \
  workbench/tests/research/experiments/test_hug_c_collection.py
```

These tests cover the application Home mapping, unit conversion, free command
increments, five-second hold reset and success gates, intersection-area
objective, function-only sampling, fixed splits, q interpolation, NPZ policy
contract, mandatory preflight, three-repeat replay gate, three-anchor orchestration,
independent diameter collection, exact XY inverse and preservation of existing NPZs. Synthetic replay
flags used by unit tests are not Genesis or lift success evidence.
