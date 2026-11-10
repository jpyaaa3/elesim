# Cylinder GA → reverse-curriculum PPO → interpolation pipeline plan

Status: implementation and GPU verification authorized, 2026-11-10.
The new coordinator is `workbench/research/experiments/hug_pipeline`.
Existing single-case experiments and evidence remain preserved. Implementation
availability is not a claim of successful Home policies or interpolation; live
acceptance is recorded in `docs/status.md`.

## Objective and scope

Given a circle diameter D, automatically select three useful C-grasp anchors,
find release paths with reverse GA, obtain physically valid forward references,
train independent Home-to-wrap/lift policies, export policies and trajectories,
and validate q-space interpolation inside the selected triangle.

The pipeline must work in a fresh directory **without an existing PPO policy**.
It must also resume interrupted stages and reuse matching completed cases.
Adding a diameter must not overwrite previously collected policies.

V1 keeps one diameter per study and one anchor case per PPO policy. D is an
input, not a list hardcoded to100/125/150/175/200. Feasibility still depends on
the actual arm, collision model and friction cutoff. A diameter with no viable
triangle returns that result; arbitrary D does not imply every D is graspable.
Cross-diameter interpolation and new object shapes remain separate extensions.

The policy artifact is an executable actor NPZ. In V1, **interpolation means
interpolating verified four-axis command trajectories collected from those
policies**, using q3/q4 barycentric weights. Neural-network parameters are not
averaged. The interpolated controller is a trajectory replay, not a newly trained
closed-loop policy. This follows the existing Hug interpolation contract.

## Existing pieces and missing interfaces

| Stage | Reusable code | Required connection/change |
| --- | --- | --- |
| Function-derived solutions and triangle | `hug_c.source`, `triangle`, `prepare`, `catalog_preflight` | Stable case IDs; exact selected q/root preservation; bounded re-selection after failed cases |
| Reverse release search | `hug_reverse_ga.search`, `physics`, `criteria` | Pinned selected solver endpoint rather than independently replacing it with minimum-mu XY inverse; per-case worker interface |
| Forward reference | boundary/bridge analysis tools | Automatic original-Home forward validation and bounded repair, with no pretrained-policy prerequisite or fixed tick350/offset |
| Reverse-curriculum PPO | `hug_reverse_rl`, `hug_home_rl`, C-hug MDP | Full Home-to-Wrap reference contract; optional initialization; cold-start lift bootstrap; compatible state-bank ordering |
| Policies and recorded paths | policy exporter, `Trajectory`, replay adapters | Collect complete learned Home episodes with event/state traces and independently replay command paths |
| Interpolation and diameter registry | `hug_c.interpolate`, `collection`, `study` | Shared-vs-case context separation; phase alignment and retimed-anchor replay; updated task/trajectory adapter |
| End-to-end execution | study/collection process patterns | One resumable state machine spanning all stages, explicit budgets, resource scheduling and readiness gates |

Do not solve this by relabeling old plan metadata. Frozen experiments remain
reproducible; new worker adapters and artifacts have an explicit new version.

## Data contracts

A single case record binds D, object XY, public total q3/q4, required mu, the
selected inverse/root/contact branch, nominal wrap roll, the true Home preset,
world object pose, material/physics settings and model/code fingerprints.
The solver endpoint selected for the triangle must survive into GA and PPO.
Choosing another inverse solution for the same XY creates another case; it
cannot silently replace a triangle vertex.

Keep a shared experiment fingerprint for model, physics, observations/actions,
coordinate conventions and reward contract. Keep per-case fingerprints for
geometry, q target, reference path, initialization and checkpoints. Three anchors
necessarily have different case/reference hashes. Interpolation checks shared
compatibility while retaining each original case identity; it must not require
different case hashes to be equal or drop all provenance checks to accept them.
Copy the mapper kernel and its model/preparation contract into the study inputs
and pin their hashes, so resume does not depend on a mutable external HTML file.

Conventions remain: fixed Go2, object mapper XY in mm; mapper x=−world Y and
y=world X in the housing-centered frame. Public q3/q4 are total segment bends.
Per-node conversion occurs only at the command boundary. Home is read from the
mechanism preset. No object-origin rebasing after settling.

Artifacts are separate:

- `case.json`: immutable scenario and selected solution identity.
- `reverse-ga/`: genomes, full physical states, verification and videos.
- `forward-reference.npz`: actual Home-to-Wrap commands, individual joint q/qd,
  cylinder pose/velocities, contacts and event times from real forward physics.
- `ppo/`: paired training/curriculum checkpoints and exported `policy.npz`.
- `trajectory.npz`: collected commands and original replay evidence.
- `aligned-trajectory.npz`: phase-aligned path, time warp and its own replay evidence.
- `interpolation/`: held-out candidates, test results and usable-region mask.
- `manifest.json`: graph state, hashes, attempts, elapsed time and readiness.

## Stage1 — choose three anchors for D

Build the function-derived C solution catalog, apply the chosen static/body/
preparation filters and physical endpoint preflight, then maximize the feasible
q-area inside the triangle: area(S_D ∩ triangle). Use preparation margin, required
friction and triangle conditioning as tie-breaks. Convert the three selected
q3/q4 points to their exact XY for the given D.

Default starting settings can remain1deg q sampling, mu_required≤0.4 and
simulation friction0.5. These are recorded settings, not universal material
constants. Refine the catalog only when the selected resolution is insufficient.
The selected area is a numerical objective on the filtered catalog; do not claim
a continuous global optimum.

The triangle need not lie entirely inside the feasible set. Holes remain masked.
Targets inside a triangle still need a valid solution and branch. If fewer than
three noncollinear anchors exist, report `no_viable_triangle` at this resolution.

If an anchor later fails its GA/reference budget, cache the result and reselect
from remaining candidates within a bounded retry budget. Record computational
failure separately from proven geometry failure. Reuse successful unchanged
anchors instead of repeating their work. Report both geometric coverage and
the eventually validated motion coverage.

## Stage2 — reverse GA for each fixed anchor

Admit the exact selected solver grasp physically, then search grasp→Home with
the existing bounded four-axis genes and real contact/fault checks. Preserve
Go2 standing control after every full/partial snapshot restore.

Home arrival concerns the arm; record the terminal cylinder state separately.
Retain the existing permitted movement during release. Smaller terminal object
drift can rank otherwise successful candidates, but do not replace the movement
constraint with an unrealistic global zero-motion requirement. Three fresh
release replays and full-state records remain necessary.

Export multiple qualified reverse candidates when available. Their approach
families and forward usefulness can differ. Candidate success is not yet a
forward demonstration or a PPO success certificate.

## Stage3 — forward validation and automatic reference repair

This is the essential new stage between reverse GA and RL:

1. Treat reversed joint-command sequences as proposals. Start a **new physical
   simulation** at true Home, original cylinder placement and normal reset
   velocities. Strip redundant dwell and retime within the existing command caps.
2. Execute the proposal forward and measure collisions, camera-side contact,
   joint limits, object motion and the terminal wrap. Never reverse a saved
   state by negating velocities, teleporting through it or assuming contact
   dynamics are reversible.
3. Accept a reference only if it physically reaches the selected wrap with
   stable, proper two-segment loading and the requested pose tolerance. This
   stage ends at Wrap; PPO still learns lift and hold. Object motion during
   contact is allowed by the declared task constraint and is recorded from the
   original nominal reference.
4. If replay fails, repair the proposal with a bounded local search over knot
   poses, intermediate targets and durations, seeded by the reverse result.
   Rank valid wrap completion first, then simpler/shorter movement and smaller
   unnecessary object displacement. This search uses physical wrap criteria,
   **not an already-trained PPO policy as its fitness or prerequisite**.
5. Validate the successful forward reference with three fresh original-Home
   replays and save all full physical states. Exhausting repair attempts yields
   `forward_reference_failed`; it does not silently launch RL with bad data.

The current35-target handoff experiment is useful evidence that such connections
can be found. Its fixed reverse tick350 and offsets are not general pipeline
defaults. Its endpoint-success test depended on an existing policy; replace that
dependency with physical Wrap completion for a fresh case.

A full contact-rich Home-to-Wrap reference may legitimately move the object.
It requires a new reference role/validator. Do not weaken the existing clean
Home-to-handoff validator or feed it a moving-contact path tagged as stationary.

## Stage4 — cold-start lift, then Home-to-Wrap reverse curriculum

Every case can start without any policy artifact. Start PPO at the verified
forward reference's Wrap endpoint, keeping the complete task objective: grasp,
lift with measured roll0±5deg and ≥20mm clearance, and five continuous seconds
of loaded retention. Once this is learnable, expand the initial-state window
backward over the **actual forward physical states** until original Home.

This removes the present circular dependency on a partially trained reverse-PPO
run and a successful policy-assisted bridge. Initialization from a matching
existing case is optional and explicit; a fresh first case cannot require it.

Use overlapping reset windows, hard-edge-only promotion credit, solved-state
retention and smaller expansion steps when advancement fails. Restore all joint
and object velocities, clear task/reward history at episode reset, and keep the
nominal object reference fixed. Use a bounded snapshot cache/selection; do not
allocate every long-path tick at512 environments without a memory plan.

No scripted path is executed after a learned episode starts. q3/q4 hints remain;
linear and roll remain policy-controlled. Keep the approved lift/hold variation
cost with small deadband and an episode budget≤1 point. Do not change all reward
weights to force a stage transition.

Before an anchor is ready, a deterministic deployed policy must complete the
entire task from original Home without guidance, in three fresh nominal replays.
Report stochastic outcomes and configured position-perturbation tests separately.
Measure oscillation/motion quality alongside task success; a small optimization
budget ending is never itself a successful policy result.

## Stage5 — export and collect compatible anchor trajectories

Save each complete actor/normalization as policy NPZ, with its checkpoint and
case hashes. Execute the actor from original Home and record full four-axis
commands plus actual state/contact/event traces. Save policy NPZ separately from
timed trajectory NPZ; loaders reject the other artifact type.

Independently replay every collected command trajectory three times at its own
anchor. Confirm original Home, original object placement and the full lift/hold
criterion. A successful closed-loop policy collection alone does not certify
its open-loop timed commands.

The three motion families must be compatible for interpolation: wrap side,
approach/contact strategy and event ordering. Exact contact node IDs need not
match. If independently learned policies take incompatible routes, try other
successful checkpoints/reference candidates within the budget, or mark the
triangle unsuitable for this interpolation. Same band membership alone is not
a compatibility proof.

## Stage6 — phase-aligned q-space interpolation and validation

Identify physical events such as confirmed wrap, liftoff, final-roll hold start
and success. Align compatible trajectories by these events. Slow phases to a
common duration within command limits; do not arbitrarily speed an anchor up or
shorten its final five-second hold.

Retimed anchors are themselves new candidates: replay them at their original
anchor cases before blending. A time warp can change contact dynamics even if
all joint limits are respected. Preserve both original and aligned references.

For an exact target(x,y,D), solve the permitted q3/q4 branch, compute nonnegative
barycentric weights in the trained q triangle, and blend all four aligned command
channels. Check angle/rate bounds and the target solution mask. Do not average
network weights or silently clamp an invalid command trace and call it verified.

Physics validation without training includes the three aligned vertices, edge
midpoints, centroid and held-out feasible interior points. Reuse the existing
validation/test defaults30/100 as an initial sample budget. Failed targets are
recorded, not used for RL. Maintain separate validation and held-out test sets.

Publication requires the configured evaluation gates. Default full-readiness
behavior retains the existing requirement that designated validation/test cases
pass. If partial publication is explicitly enabled, publish a clearly labeled
usable-region mask and failure counts; do not present the whole triangle as
validated. Boundary refinement and triangle re-selection are bounded operations.

A continuous, previously untested target may get an immediate mathematical
trajectory candidate; contact success is not guaranteed merely by being inside
the triangle. Its verification status remains explicit. There is no v1 fallback
to a different diameter or out-of-triangle extrapolation.

## Coordinator, execution and artifacts

Add one repository-only `hug_pipeline` coordinator with worker adapters, stable
case contracts and `plan/run/resume/status/resolve` commands. Reuse the existing
math, search, scene, PPO and NPZ libraries. The runtime payload remains the
general RL playground; cylinder workflow logic stays in research code.

Persist an atomic state machine, for example:

```text
catalog_ready → anchors_selected
  → per-anchor: reverse_verified → forward_reference_verified
                → lift_bootstrapped → home_policy_verified
                → trajectory_verified → aligned_trajectory_verified
  → interpolation_validated → published
```

Each stage records exact input/output hashes, worker versions, seed, budgets,
attempts, GPU UUID, scoped process IDs, elapsed time and logs. Only matching
completed artifacts can be reused. Resume does not repeat completed training or
confuse a checkpoint file with successful evaluation. Keep classified failures
such as GA budget exhaustion, forward repair failure, lift/Home curriculum
stall and interpolation failure. Never silently alter friction, q branch or
success criteria to make a stage pass.

The three anchor jobs can be scheduled independently. Preserve the measured
defaults of GA population128 and PPO512 environments per worker. GPU concurrency
and total environments are explicit resource limits; benchmark before assuming
three simultaneous512-environment PPO jobs will improve throughput. A single
GPU's validated default is one512-environment PPO worker. Other GPU/hardware
work is outside this coordinator's ownership.

Keep caption-free reverse-GA population videos every10 generations, final
population/winner videos on early success, PPO checkpoint videos, final original-
Home evaluations and representative interpolation videos. Cameras view the arm
side. Status distinguishes GA/reference success, policy success and interpolation
readiness. Estimate remaining time from observed per-stage measurements.

## Implementation order and acceptance

1. Case/shared-context schemas and worker adapters, then selected-solution
   preservation and deterministic triangle selection/re-selection tests.
2. Reverse-GA→forward-reference validation/repair. Prove one fresh100mm anchor
   can produce its reference without any existing PPO or hardcoded case offsets.
3. Full Home-to-Wrap reset bank and cold-start lift→Home PPO. Check selected
   resets, velocities, leg control, smoothness budget and warm-start/resume at
   the intended512-environment scale. Run a short bounded learning pilot.
4. Three-anchor coordinator with interrupt/resume and immutable artifact reuse.
5. Policy collection, command replay, phase alignment and interpolation adapters.
6. One complete fresh-D study through all publication gates; add a second D and
   demonstrate that the first diameter's policy/checkpoint hashes remain unchanged.

CPU tests cover ordering, geometry/branch identity, hashes, state transitions,
budget handling, barycentric boundaries and version rejection. Mock workers
test orchestration failures without treating them as physics evidence. Real
GPU tests must cover each physical boundary and the final Home/interpolation
gates. Do not certify the pipeline from unit tests or one policy-assisted clip.

Completion means a fresh directory and valid D can run all stages, produce three
unguided original-Home policies and independently verified anchor trajectories,
and publish the interpolation evaluation/selection artifacts. A new exact XY
query must return its branch, q-space weights, artifact references and verification
status. Failure must stop with a reproducible reason and resumable state.
