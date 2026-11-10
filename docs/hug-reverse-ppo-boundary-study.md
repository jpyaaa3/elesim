# Reverse-curriculum boundary study — 2026-11-10

This is the single 100mm cylinder case from
`hug-reverse-ppo-legpd-20261109T144613Z`. It does not establish performance on
other positions, diameters or randomized physical conditions. All tests keep the
PPO weights, q3/q4 goal, four action limits, collision rules, 90-degree lift
criterion and five-second loaded hold unchanged. No optimizer updates occurred.

## What the tests found

1. The three frozen checkpoints500/1000/final were each tested from33 states.
   The old reverse prefix was sampled every10ms from3.75 to4.00s, with neighboring
   states, the admitted grasp and nominal Home added. Both500 and1000 succeed
   through3.76s and fail from3.77s onward in this sweep. The final checkpoint
   succeeds at3.75s and fails at3.76s. More training did not expand this boundary.
2. Gaussian exploration alone does not explain the failure. For the final
   checkpoint,3.75s succeeds32/32 with the stored full exploration noise, whereas
   3.77s and4.00s succeed0/32 at each of noise scales0,0.25 and1. The3.76s state
   succeeds0/32 deterministically and10/32 at full noise. These are frozen-policy
   samples with seed42, not training results or randomized physics validation.
3. A bounded command bridge to a pose derived from reverse tick350 connects
   all nine tested reverse-prefix starts to successful wrap/lift/hold for both
   checkpoints500 andfinal. Direct and reversed-command bridges each succeed
   on those nine starts. Nominal Home fails after these unadjusted bridges.
4. The bridge itself reaches nominal Home's handoff safely, with the cylinder
   still at its original placement. The subsequent policy fails. The otherwise
   successful GA Home endpoint has the cylinder displaced by about33mm; an
   arm-pose-only comparison would miss this distinction.
5. A35-candidate search over handoff linear offsets−30..+30mm and roll
   offsets−0.2..+0.2rad finds14 successful candidates for each frozen policy,
   starting from the actual Home and original nominal cylinder placement.

The first-fault records include cylinder/Go2 leg contacts just beyond the older
policy's success boundary. Other cases time out. This does not prove every
failure has the same cause, or that an unsuccessful start is physically
unreachable. The successful bridges directly demonstrate reachability for the
tested cases.

## A verified nominal-Home connection

Selected handoff command, in public total-angle convention:

| Control | Command |
| --- | ---: |
| Linear | −100mm |
| Roll | +5.7296deg |
| q3 total bend | −81.4583deg |
| q4 total bend | +105.6442deg |

The normal bounded command controller moves Home to this target in1.1s, then
hands control to the **unchanged final PPO**. The measured joint pose still has
tracking error and nonzero velocity; the state is not teleported or presented
as a settled pose.

Three freshly rebuilt single-environment replays all complete the original
wrap/lift/hold task in10.6s, with no fault or wrong-face contact. At handoff the
task history, reward ledger and episode clock are cleared to match a curriculum
start. A complete Genesis snapshot preserves joint q/qd and object pose/velocity:
all six state comparison errors are exactly0. This also checks that the result
does not depend on credit or elapsed episode time earned during preparation.

The preparation has no physical fault or wrong-face contact; its maximum
horizontal cylinder drift is4.46e−9m, numerical scale. Its111 full-state frames
span0..1.1s. The joint velocities at handoff remain those actually produced by
the forward approach. The prefix NPZ is recorded evidence for rebuilding a
snapshot bank, not a serialized Genesis state.

A separate96-lane test clears history at the same handoff and uses32 replicas
per noise condition. Results are32/32 at zero noise,32/32 at0.25 of the stored
Gaussian standard deviations, and31/32 at full noise. The endpoint therefore
provides a viable starting distribution for a short curriculum under this
nominal task. This is not a robustness guarantee.

The full Home success in these tests includes a prescribed1.1s entry segment.
**The original PPO alone still fails from Home.** No new Home policy has been
trained by this study.

## Recommended RL change

Use the verified forward entry prefix as a new reset reference. The old PPO
already completes the task from its endpoint; train it to extend that behavior
back to Home. Keep the complete lift/hold goal and q3/q4 potential.

1. Add an explicit forward-prefix reset contract. Validate the nominal Home
   start, complete recorded q/qd/object state, command rates, source hashes and
   clean physical replay. Rebuild snapshots by replaying from original Home,
   rather than by admitting a grasp and replaying the old release. Retain leg
   PD restoration and subset-reset noninterference checks.
2. Store reference time in ascending order0..1.1s; map curriculum difficulty
   in the opposite direction, from the endpoint back to Home. Do not pass this
   data through the old grasp-to-Home validator or silently reverse its schema.
   Its original-object placement and forward joint velocities are material.
3. Sample overlapping intervals of actual prefix snapshots. Start near the
   endpoint; extend the hard edge by small tick increments, reducing the step
   when progression fails. Retain solved starts. Count clean outcomes from the
   newly introduced hard band and current epoch for promotion; easier starts
   must not falsely promote it. The stored10ms resolution supports small
   steps without interpolating physical states. Cache a bounded set of active
   snapshots rather than assuming111 snapshots at512 environments are cheap.
4. Explicitly initialize actor/critic and observation normalization from the
   verified checkpoint. Start a new optimizer and curriculum history, recording
   the original checkpoint hash. This is a new reset experiment, not a resume
   with the old GA curriculum JSON. Preserve original artifacts and provenance.
5. In training, the prefix supplies **initial states only**. All subsequent
   actions come from PPO; there is no scripted handoff segment in a learned
   episode. At the final stage include exact original Home explicitly, rather
   than relying on a continuous sampler to hit an endpoint of measure zero.
6. Begin with a short bounded pilot, evaluating deterministic original-Home
   episodes and newly added starts alongside stochastic training outcomes.
   Save the best nominal-Home checkpoint. Require unassisted Home-to-wrap/lift/
   hold evaluation before calling the learned policy successful.

This changes the curriculum reference and reset sampling first. The test results
do not justify another reward-weight sweep or a claim that noise reduction alone
will fix the old boundary. Whether PPO learns the new entry segment still needs
the proposed pilot; guided evaluation is not evidence of that learning.

## Reproduction and evidence

Maintained entrypoints:

```bash
python -m workbench.research.analysis.probe_reverse_boundary --run RUN --output NEW_OUT
python -m workbench.research.analysis.probe_reverse_boundary --run RUN --output NEW_OUT --noise
python -m workbench.research.analysis.probe_reverse_bridge --run RUN --output NEW_OUT
python -m workbench.research.analysis.probe_reverse_bridge --run RUN --output NEW_OUT \
  --starts -1 --methods direct --policies model_final \
  --target-linear-offsets .02 --target-roll-offsets .1 \
  --fresh-handoff --capture-prefix --video
python -m workbench.research.analysis.probe_reverse_bridge --run RUN --output NEW_OUT \
  --starts -1 --methods direct --policies model_final \
  --target-linear-offsets .02 --target-roll-offsets .1 \
  --fresh-handoff --repeats 32 --noise-scales 0 .25 1
```

Use the pinned server scientific environment and the existing GPU2 UUID, with
the same `PYTHONPATH` as the reverse-PPO runbook. Each command requires a new
output directory. These commands construct no learner and perform no updates.
The offset-grid command and exact protocols are saved beside their raw results.

Raw folders under `workbench/evidence/generated/`:

- `hug-reverse-boundary-20261110`:33 starts ×3 checkpoints.
- `hug-reverse-boundary-noise-20261110`:384 noisy/deterministic replays.
- `hug-reverse-bridge-20261110`:10 starts ×3 entry strategies ×2 checkpoints.
- `hug-reverse-bridge-offsets-20261110`:35 target candidates ×2 checkpoints.
- `hug-reverse-home-bridge-verified-20261110`:three single-environment replays,
  the video and full forward-prefix NPZ records.
- `hug-reverse-home-bridge-noise-20261110`:96 handoff/noise trials.

These total712 policy episodes, in addition to reset-bank admission and safety
probes. GPU2 jobs ended; other GPU work was preserved. Host NumPy bridge unit
checks and actual GPU replay checks cover this report tooling. Canonical
dev/release gates were not run because the generated dev wrapper is unavailable.
The study itself performed no optimizer updates. The subsequent implementation
is in [`hug_home_rl`](../workbench/research/experiments/hug_home_rl/README.md),
with a separate forward reset contract and bounded lift smoothness cost.
Its software/physics checks and any later training are separate evidence from
the frozen-policy results reported here.
