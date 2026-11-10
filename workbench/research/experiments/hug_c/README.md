> **Historical workflow — retired 2026-11-10.** Do not use the commands below for new training.
> Source remains only where current tasks or regression tests depend on it.
> Use `hug_reverse_ga` and `hug_home_rl`; see [workflow status](../HUG_WORKFLOWS.md).
> The exact pre-retirement sources and instructions are preserved in the local archive inventory.

# C-hug from Home: free four-DoF motion, lift and q-space interpolation

This is a repository-only task in the EleSim RL playground. Its input comes
from **mapper.html's analytic functions and CAD collision kernel**, never from
its embedded `DATA` point cloud or `map.json`. It does not start Pilot, UI,
Robot, DDS, or physical hardware.

The previous v2 reward contract reached 2,000 updates on the first 100 mm
anchor and failed to lift. Its videos/checkpoints are preserved. The v3 reward
contract below corrects posture-only reward collection and camera-side contact
credit. Its first study also failed: the chosen placement obstructed ordinary
Home→Extend Arm preparation. New triangles use the preparation-clear domain
and require a physical preparation check before training. Roll, closing and
lifting remain learning/validation questions.

## Experiment contract

- Collect D=100, 125, 150, 175 and 200 mm independently, with C targets
  (`q3<0`, `q4<0`), required μ≤0.4. Each diameter has three training anchors,
  30 validation and 100 test cases: **15 initial policies**, not mixed-D RL.
  q3 and q4 mean **total segment bend**, in degrees in the geometric data.
- Select exactly three valid targets maximizing the rasterized area of
  `S_ready ∩ triangle`, where `S_ready` passes final-pose CAD **and the actual
  mapper Home→Extend Arm envelope filter**. Tie breaks prefer a larger minimum
  preparation margin, then lower worst-anchor μ and better conditioning.
  This is selection contract `preparation-clear-area-v2`. Infeasible interior gaps are
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
- Only the function's **q3/q4** goal appears as a target in observations and
  q shaping. Linear/roll goal coordinates are absent from actor target inputs
  and the q potential. Geometric endpoint metadata stays available for static
  checks; roll completion additionally gates the lift milestones below.
  Final linear remains free. Success requires raised roll0 within5 degrees;
  this is a completion constraint, not a scripted action or roll-target hint.
- Success requires **lifting**: the oriented cylinder's **lowest point**
  stays ≥20 mm above the floor for **5 continuous seconds**, with measured
  **roll0 ±5°** throughout that window and no floor contact,
  hand-relative slip ≤10 mm, proper-face proximal and distal/gripper contacts
  for ≥80% of that window, a confirmed C grasp and no fault. Drops, camera-side
  contact, excess slip, incomplete lift roll or loss of C reset
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
  A fault pays one fixed penalty and terminates at the next policy boundary.
- Default physics: contact μ=0.5, floor/object μ=0.01 (pair mixing follows
  Genesis), mass=0.30 kg, cylinder height=1.10 m, Go2 spawn=0.32 m. The tall
  cylinder spans the arm plane from the floor; it is an explicit inherited
  test-object assumption. A shorter object needs a matching support/placement
  scene, not just a renamed diameter. `configure --settings` records overrides.

The simulation boundary uses per-node radians:
`theta1=deg2rad(q3)/5`, `theta2=deg2rad(q4)/5`. Actor observations expose total
bend radians; NPZ command channels explicitly retain the Sim API's per-node
`theta1_rad`/`theta2_rad` convention.

## Reward contract v5 (90-degree lift)

One policy step lasts 0.1 s. The task records every component separately in
TensorBoard and in collection/replay measurements:

| Component | Default | Eligibility / budget |
| --- | ---: | --- |
| q3/q4 hint | potential range 0–1 | `gamma*Phi(next)-Phi(previous)`, terminal potential zero |
| Proper C grasp | +3 | Two proper contact roles sustained for 0.2 s; once per episode |
| Loaded lift progress | up to +5 | Proper grasp, no floor contact; new best minimum of clearance progress and90° roll progress |
| Continuous hold progress | up to +10 | New best valid continuous hold duration up to 5 s |
| Success | +30 | Full lift/hold criterion, once, then terminate |
| Camera-side contact | −1/s | −0.1 per full policy step; proportional to offending physics ticks |
| Lost airborne grasp | −3/event | Once per drop; re-arm only after another confirmed airborne grasp |
| Forbidden collision / physical limit fault | −30 | Once, then terminate; no timeout penalty added |
| Timeout | −5 | No success at the 20 s limit |
| Requested action magnitude | −0.001 × sum(action²) | Clipped policy requests, including blocked requests, four channels |
| Time | −0.3/s | Exempt only during a qualified loaded hold at final lift roll |

`Phi` depends only on the normalized q3/q4 error; linear and roll do not enter
it. PPO gamma is **0.995**, set by this task without modifying legacy Wrap
defaults. The same gamma is used for q shaping. With terminal `Phi=0`, the
discounted sum of the q shaping terms is exactly `-Phi(initial)`, so an empty
goal posture cannot increase episodic return merely by collecting the hint.
The hint still provides local learning feedback.

Lift/hold budgets are episode records, retained across drops: returning to a
previous height or hold duration does not pay again. Simple contact by itself
no longer pays every step. When **any camera-side contact** occurs within a
policy step, that entire step loses all positive rewards (including success,
q hint and milestone progress), and paid records do not advance. A later proper
contact can recover; a camera-side contact is not an automatic episode fault.

Contact classification uses the contact position and outward normal in the
contacted link's frame. The model-derived camera-side axis is node-local −Z;
gripper/claw axes are transformed through the actual fixed mounts (−X for the
current bundle). It remains consistent under roll and contact A/B swapping.
The engine's normal is B→A in pinned Genesis 1.4.5; the adapter converts it to
robot-outward. A 1 mm centre strip is neutral, the normal consistency tolerance
is 0.15, and loaded contact uses a 0.01 N threshold. Cap/tip normals may be
tangent to the side axis; such contacts are classified by their half-face.
Body/camera collision remains a separate hard fault.

There is no ordinary XY displacement penalty; moving the object while wrapping
remains allowed. Existing one-metre runaway and physical safety limits remain.

To score the declared scenarios and old unsuccessful trajectories without RL:

```bash
python3 -m workbench.research.experiments.hug_c.reward_audit \
  --study /path/to/archived/study --output /new/path/reward-audit.json
```

Synthetic scenario scores exercise the reward rules, not physics or learned
success. The archived q-only rescore omits unknown contact-face penalties and
is therefore an optimistic bound. All weights/thresholds are recorded settings
and can be overridden through `configure --settings` in a fresh experiment.

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
The output also includes `index.html` linking the five q-space reviews. Inputs
record each point's preparation margin, model and preset contract. Old mappers
without the preparation envelope, stale model/presets and old selection
contracts are rejected. Exact held-out and XY inverse queries apply the same
filter; an excluded gap inside the triangle cannot become a valid target.

Before final triangle selection, screen the **entire** preparation-clear catalog
at actual Home and goal poses, then reselect on that measured area:

```bash
python3 -m workbench.research.experiments.hug_c.collection screen \
  --collection "$OUT/collection.json" --output "$OUT/catalog-screen"
SCREENED="${OUT}-screened"
python3 -m workbench.research.experiments.hug_c.collection prepare /path/to/mapper.html \
  --output "$SCREENED" --diameters-mm 100 125 150 175 200 \
  --catalog-preflights "$OUT/catalog-screen" --mu-max 0.4 --step-deg 1
OUT="$SCREENED"
```

The triangle optimizer sees only physically screened cells. Home drift above
1 mm also rejects a catalog entry. Screening does not run an optimizer; old
reports with a different kernel/model/grid/settings are rejected. Keep both
the preliminary inputs and measured reports for comparison.

Run all five physical admission checks without training:

```bash
python3 -m workbench.research.experiments.hug_c.collection check \
  --collection "$OUT/collection.json" --output "$OUT/physics-checks"
```

The function-only inputs may fail actual Home/Extend/goal checks; they are not
closing/lift certificates. To prepare a single diameter instead, use a new
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
10-tick Home and directly placed geometric goals for all three anchors. It also
physically replays the declared Home→Extend Arm primitive at bounded command
rates, followed by a one-second settle. **Any robot/object contact**, a fault,
or object XY drift above 1 mm rejects the anchor; a clean last frame cannot
erase a transient contact. This diagnostic is independent of policy actions.
It constructs the PPO interface without optimizer updates and does **not**
certify subsequent roll, closing or lift. Training requires matching successful
`preparation_tested`/`preparation_ready` and per-anchor records; an old static-only
preflight cannot start training. Code 1 means physics failure, code 2 missing dependencies;
exceptions are errors, never successful checks.

If preflight rejects an anchor, inspect and preserve the report. A JSON list of
failed `[q3,q4]` total-degree coordinates can be passed to `prepare --exclude-q`
to regenerate and record the excluded candidates. If Home itself is invalid,
fix the model/scene/mapping rather than substituting another start or merely
excluding more targets. Do not train through a failed preflight.

For a wide solution chart, a single failed endpoint can hide more invalid
placements and contact-face violations missed by approximate final geometry. The optional
`catalog_preflight` command screens every function-generated raster entry in
64 parallel scenes at both Home and the directly placed goal. Supply that
report to `prepare --catalog-preflight report.json` to select the triangle only
from static-ready cells; the measured physics settings and model must match.
The three selected anchors still undergo the normal independent preflight.
This is static screening, not a replacement for the three physical preparation
checks. Its report must match the new kernel. On 2026-11-06 the **old kernel's**
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
or `study`. Its new 1° selection covers about 74.50% of the preparation-clear,
physically Home/goal-screened C solution area;
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

Use a new collection/study directory for this selection contract. Previous
failed inputs, policies and videos are preserved and are not resumed as the new
triangles. The next study defaults to **512 environments** per the 2026-11-06 utilization
measurement (30 one-second samples: mean 65.6%, median 69%, maximum 73%; memory
3,175/49,140 MiB at 256 environments). This is a next-run preference, not a
measured 512-environment speedup. Both the old scripted two-action run and
the failed v2 reward run are preserved. Their policies cannot resume the v3
reward contract: the actor now has 20 observations (q goals only), four actions
and policy schema v3. New learning must be explicitly started. The machine-readable next-run
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
provenance must match. Old two-action, fixed-roll and reward-v2 policies are incompatible.

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
  workbench/tests/research/experiments/test_hug_c_collection.py \
  workbench/tests/research/experiments/test_hug_c_rewards.py
```

These tests cover the application Home mapping, unit conversion, free command
increments, five-second hold reset and success gates, intersection-area
objective, function-only sampling, fixed splits, q interpolation, NPZ policy
contract, mandatory preflight, three-repeat replay gate, three-anchor orchestration,
independent diameter collection, exact XY inverse and preservation of existing NPZs. Synthetic replay
flags used by unit tests are not Genesis or lift success evidence.

## PPO population with evolutionary exploration (2026-11-07)

`experiment train` now defaults to `--strategy population`. `--strategy single`
retains the original PPO baseline. The population changes search, not the task:
true Home reset, unrestricted four-axis actions, q3/q4-only potential hint,
forbidden-face/structural collision rules, and actual lift/hold success remain.
There is no new reward coefficient and no scripted roll schedule.

Default allocation is four independent actor/critic/optimizer/rollout-buffer
sets, 128 environments per member (`--num-envs 512 --population-size 4`).
The members **control disjoint groups in one 512-environment physics batch**.
Every physics step advances all four policies together. `--num-envs` is the total
number of simultaneously simulated robots; `--population-size` divides that batch. Each generation gives every member 100 PPO
updates. Total training transitions equal the single-policy baseline's budget:
`updates * total_num_envs * 16`. Evaluation transitions are extra and recorded
separately. This design does not claim a wall-clock speed improvement.

After three generations without lexicographic physical progress, at most one
member is replaced. The current champion and one distinct useful approach are
protected; the historical best evaluated checkpoint is also retained. Replacement
cycles through exploration reset, bounded actor-MLP mutation, and an earlier
checkpoint. Archived strategies also supply parents. New branches copy actor and
critic, reset optimizer/rollout state, and gather their own on-policy experience.
No population experience is injected into another PPO buffer.

The selection tuple is success, hold, lift, grasp, proper contact, approach, safe
fraction. It is an explicit task-specific **outer selection criterion**, not a
PPO reward. Approach measures reduction in object-to-gripper-anchor distance;
it is only a stepping-stone heuristic, not a grasp certificate. A behavioural
archive stores safe contact or >=5% safe approach progress. Its four coordinates
are maximum extension and roll/q3/q4 posture at first contact (otherwise closest
approach), normalized to joint ranges and binned into 5 bins each (625 niches).
Each niche keeps its best policy. A novel niche can admit an offspring with lower
fitness than its parent; forbidden-contact performance must not worsen. Novelty
alone cannot qualify an unsafe episode for the archive.

Actor mutation backtracks one fixed noise direction until observed action change
on the parent's visited states is at most 0.12. This bounds local behaviour change,
**not physical safety**. Parent and child receive a separate physics evaluation.
Exploration floors initially range from 0.08 to 0.35 and mutate by 0.8/1.2, bounded
to [0.05, 0.5]. They are saved/restored independently of Torch state_dict.
These are initial research settings, not validated grasp-optimal values.

Run the first anchor only before authorizing the entire collection:

```bash
python -m workbench.research.experiments.hug_c.experiment train \
  --experiment NEW_STUDY/experiment.json --preflight NEW_STUDY/physics-check.json \
  --case c-train-000-d100 --runs NEW_STUDY/runs \
  --strategy population --num-envs 512 --population-size 4 \
  --iterations 500 --generation-updates 100 --stagnation-generations 3
```

The source digest changed: configure a **new** experiment and repeat physics
preflight. Do not rewrite old evidence hashes or resume the old single-policy
run as a population. `--resume --iterations N` adds N updates per member, matching
existing study/report chunk semantics. It requires identical population settings
and the last complete generation; a partial generation is refused rather than
overwritten. Preserve it for diagnosis and use a new output if interrupted there.

Outputs include `population.json`, atomic `population-state.json`, immutable
per-generation evaluated/continuation checkpoints, per-member policy NPZ,
evaluation trajectory NPZ/JSON, archive lineage, rejected offspring evidence,
and `model_final.pt` pointing to the best evaluated policy. Standard export,
collect, replay and study MP4 reporting still use that checkpoint. Evaluation is
currently a fixed nominal Home batch, not independent robustness trials or a
claim about unseen sizes/positions. Small placement-error validation remains a
separate gate. No robot RL has been launched for this implementation.

### Literature and deliberate departures

- [Bai, Cheng & Jin, Evolutionary Reinforcement Learning: A Survey (2023),
  sections on exploration, behaviour characteristics and search operators](https://arxiv.org/pdf/2303.04150):
  motivates separating exploration from reward tuning and documenting descriptor
  bias. An archive is only as useful as the behaviour descriptor; this grasp
  descriptor still needs empirical evaluation.
- [Khadka & Tumer, Evolution-Guided Policy Gradient (2018)](https://papers.nips.cc/paper/7395-evolution-guided-policy-gradient-in-reinforcement-learning.pdf):
  uses an evolutionary population and off-policy experience sharing. We do not
  reproduce ERL's DDPG replay sharing because the learner here remains PPO.
- [Jaderberg et al., Population Based Training (2017)](https://arxiv.org/html/1711.09846v2):
  motivates parameter inheritance and exploration-hyperparameter perturbation.
  Our schedule is synchronous, with one shared physics batch and private PPO buffers, not asynchronous PBT.
- [Nilsson & Cully, PGA-MAP-Elites (2021)](https://www.cmap.polytechnique.fr/~nikolaus.hansen/proceedings/2021/GECCO/proceedings/proceedings_files/p866-nilsson.pdf):
  motivates niche-local competition, rather than rejecting every child weaker
  than its parent globally. This implementation is not its TD3/PG variation loop.
- [Lehman et al., Safe Mutations (2018)](https://arxiv.org/abs/1712.06563):
  motivates checking behavioural disruption. Our bounded-output backtracking is
  not their gradient-sensitivity-scaled SM-G implementation.
- [Marchesini et al., Genetic Soft Updates (ICLR 2021)](https://openreview.net/pdf?id=TGFO0DbD_pk):
  discusses evolutionary policy updates compatible with DRL beyond off-policy
  replay frameworks. We preserve separate lineages rather than implement its
  soft-weight update algorithm; no claim of reproducing Supe-RL is made.

Validation distinguishes software checks (selection, mutation, checkpoints,
NPZ and PPO API compatibility), real Genesis population startup smoke checks,
and actual grasp/lift learning. Startup smoke does not establish task success. Compare equal training **and evaluation** budgets
and report first-contact/grasp/lift discovery, archive coverage, and saturation
stall frequency; aggregate PPO reward alone is insufficient.

## Compact movement and time costs: experiment v4

Experiment schema v4 changes the reward contract to
`bounded-milestones-compact-workspace-time-v4`. The old evolution pilot
`c-hug-evolution-20261107T0518Z` was stopped by user request at 900 completed
updates; its checkpoints, videos and exact source are preserved. A fresh v4
pilot `c-hug-evolution-v4-20261107T0840Z` started at 17:47 KST on 2026-11-07.
Configure a fresh experiment
from the same pilot and repeat preflight before any new training; old experiment
JSONs are rejected, and old population states cannot be resumed under this contract.
Policy observation/action conventions and policy NPZ schema remain v3.

| Cost | Next default | Application |
| --- | --- | --- |
| Time | -0.3 / second, previously -0.1 | Active task time; a currently confirmed, qualified loaded hold exempts verification time |
| Workspace | -0.2 / second maximum | Squared excess footprint distance, saturated at 100 mm beyond the allowance |
| Workspace allowance | 50 mm | Soft margin around the fixed task envelope |
| Action effort | Existing coefficient 0.001 | Requested clipped policy action, including requests blocked at joint bounds |

The episode remains 20 seconds and actual loaded hold verification remains
5 seconds. A touch, rubbing the cylinder, an old grasp event or matching the
empty goal q cannot exempt time. Existing contact/lift/hold credits remain
one-time milestones or episode records, so repeating a contact cannot pay for
waiting. Holding still with zero action has no action-effort cost. A physical
fault still costs 30; even a full 20 seconds of maximal workspace cost, time,
action and timeout does not make immediate collision the cheaper solution in
the tested nonprogress scenario. This is an ordering regression, not a guarantee
against every possible learned exploit.

`workspace.py` forms a fixed convex XY hull containing the collision-box corners
of all ten arm nodes and three gripper links at Home and the nominal goal, plus
the initial cylinder footprint. Measured link positions/quaternions transform
these corners at every physics tick. The largest distance outside that polygon,
after a 50 mm margin, determines the workspace cost:

```text
excess = max(0, max_marker_distance_outside_fixed_hull - 0.05 m)
workspace_fraction = min(excess / 0.10 m, 1)^2
workspace_cost = -0.2 * time_step * workspace_fraction
```

Macro-step reward integrates exposure over its physics ticks. This penalizes
wide horizontal excursions rather than total q travel; in particular the large
necessary Home-to-goal q4 change is not itself a footprint violation. Home and
the nominal goal have zero workspace cost by construction. Vertical travel is
free of this footprint cost. The region stays fixed if the object is pushed;
it is neither a collision boundary nor proof that a successful approach exists
inside it. Necessary detours remain possible at a bounded cost. The 50/100 mm
choices are starting preferences requiring physical evaluation, not geometric
feasibility limits.

Physics measurements report peak workspace excess and cost components. Raw
policy collection charges requested action effort; a command-only trajectory
replay lacks rejected requests and reports its action-effort cost as a lower
bound. Its physical grasp/hold verification remains meaningful. Archived
command-only reward rescoring likewise labels omitted face/workspace costs.
Population reports use efficiency costs only after equal physical/approach
metrics, and the independent reporting selector uses the same field order.

Population schema v3 also fixes variation scheduling: an actual-attempt counter
cycles exploration / weight mutation / rewind and is persisted with state.
It advances even for a rejected candidate. The old `generation % 3` selection
could synchronize with three-generation stagnation and select exploration on
every intervention (observed at generations 3 and 6). Regressions exercise
interventions at 3/6/9/12 and resume the counter correctly.

Offline audit of the recorded 100/800-update commanded trajectories, sampled at
0.1 s, estimates workspace costs of approximately -0.57 / -3.94 respectively.
Both failed 20-second attempts would receive -6 time cost instead of -2.
These are kinematic estimates, not newly replayed physical costs or evidence of
better learning. Generated audit: `workbench/evidence/generated/c-hug-efficiency-v4-20261107/audit.json`.

The v4 startup passed real GPU Home/Extend/goal preflight for all three D100
anchors. `analysis/check_hug_workspace.py` also checked four replicas: Home/goal
excess was zero in all lanes, while direct placement in the extended pose had
405 mm excess and saturated workspace cost in every lane. This teleport check
does not certify a path. A four-member, one-update-each real Genesis/PPO smoke
completed evaluation/checkpoint/NPZ export before the fresh 2000-update first
anchor training began. Detached server reporting owns 100/500/1000/1500/2000
videos and final replay, independent of Codex login; Luna observes and collects.


## Simultaneous population and compatible transfer

Population schema v4 makes synchronous physics the permanent default for
`experiment train`, `study` and collection launches. The default is **four policies
x 128 robots = 512 actual simultaneous robots**, not sequential reuse of 128.
`population_batch.learn_population` obtains four action blocks, calls the full
physical environment once, then sends each block's observations, rewards, dones
and timeout flags only to its own PPO. Normalizers, rollout buffers and optimizers
remain private. Neural optimizer updates are issued serially on the GPU; physical
simulation and multi-policy evaluation advance the full batch together.
The rollout training budget stays unchanged. Evaluation trajectories preserve
each representative lane's own termination time.

For an existing **v3 sequential population with the same v4 reward/task**, use
an explicit transfer into a new run, after fresh configuration and preflight:

```bash
python -m workbench.research.experiments.hug_c.experiment train \
  --experiment NEW/experiment.json --preflight NEW/preflight.json \
  --case c-train-000-d100 --runs NEW/runs \
  --population-from OLD/runs/c-train-000-d100 \
  --num-envs 512 --population-size 4 --iterations REMAINING_UPDATES
```

The transfer refuses any model, task, target, reward, allocation or population
parameter change; only the declared source-code/scheduling transition is allowed.
It copies immutable last-complete-generation checkpoints, including actor, critic,
normalization and optimizer state; exploration floors, staleness, archive parents,
variation-attempt counter and historical champion remain. The adaptive learning
rate is recovered from optimizer param groups. The historical champion is exported
again under the new context with original-checkpoint hashes in `transfer.json`.
Original evidence and partial-generation files are not rewritten. Each policy
continues from the last fully completed generation; the random sampling order can
change with simultaneous stepping. `--iterations` adds updates to inherited counts;
pass `2000 - inherited_updates` to retain the original 2000-update total.
`--resume` remains strict for already-concurrent runs. This is an explicit source
transition, not a silent edit of old experiment hashes.

The per-generation `training_wall_s` is shared by all four members, so do not sum
four copies of it for ETA. Progress reports include `active_members`,
`physical_num_envs` and `generation_updates_done`, updated every 10 PPO updates.
The old sequential schedule is accepted only as a transfer source.

## Arm-side report videos

`analysis.watch_hug_population` records future milestone videos through
`analysis.record_hug_video`. The camera now looks down from the negative-world-Y
side, opposite the cylinder side of these C targets, so the cylinder no longer
sits directly between the viewer and the arm. The fixed view uses position
`(0.85, -1.15, 1.60)` m and look-at `(0.35, 0.05, 0.40)` m, with world Z up.
It also shows the lower cylinder and floor for lift assessment. The viewpoint is
recorded as `diagnostics.report_camera` in the trajectory and report JSON.

To record an exported policy manually from the same viewpoint:

```bash
python -m workbench.research.analysis.record_hug_video \
  --experiment "$OUT/experiment.json" --policy "$POLICY" \
  --output "$OUT/arm-side/trajectory.npz" --video "$OUT/arm-side/policy.mp4"
```

This only collects one deterministic evaluation trajectory; it never trains.
Both outputs must be new paths. A failed grasp still writes its video/trajectory
and exits 1. The report entrypoint lives outside the frozen task-source digest,
so changing presentation does not relabel checkpoints or require restarting an
active population. Direct task CLI videos retain their original viewpoint;
use this report command for arm-side recordings.


## Full90-degree lift contract (2026-11-08)

Current task uses experiment schema5, policy schema4, control mode
`policy-four-dof-home-lift90-v4`, reward contract
`bounded-milestones-lift90-v5`. Canonical C wrapping is roll-90°. The raised
completion pose is roll0° within a default5° tolerance (at most10° configurable).
This is the arm's roll requirement, not an object quaternion requirement.
Every physics tick of the five-second loaded hold must satisfy the measured
roll constraint; leaving it resets the window. Home already has roll0, but
clearance, C grasp, contact coverage and retention remain independently required.
No time-based motion or intermediate roll is imposed; q3/q4 hint remains roll-free.

Lift progress is min(clearance/20mm, roll-lift progress), clipped0..1. Roll-lift
progress is0 at wrap-90° and1 inside the final tolerance. Hold/success credit
and time-cost exemption require the final roll. Reward coefficients did not
change; this updates the physical meaning of completion and progress. GA,
collect/replay, ordinary PPO and demonstration approval share these metrics.

Old partial-lift experiments/policies and their GA/BC artifacts are not silently
accepted under this contract. Configure a fresh experiment from the same pilot,
then prepare fresh GA inputs and repeat preflight before later PPO training.
The original candidate124 result and video stay valid evidence for the older
clearance-only criterion; they do not establish full90-degree lift success.
