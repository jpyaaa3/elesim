# Reverse GA criteria v1

Status: framework implemented, 2026-11-09; physical acceptance remains unrun.
Implementation and commands: [reverse GA](../workbench/research/experiments/hug_reverse_ga/README.md).
The existing forward GA is unchanged. Preparing a plan does not start a GPU run. New numerical
tolerances below are initial experiment defaults, not measured feasibility claims.

## Purpose and endpoints

Replace manually selected wrap-grasp curriculum waypoints. For each of the three
solver-selected C-band triangle anchors, find a physical path from its solver
grasp to the real Home preset. Keep anchors and object geometry fixed per search.
This is not an optimization of the triangle, grasp target, or lifting trajectory.

Use canonical public coordinates `(linear_m, roll_rad, q3_total_rad, q4_total_rad)`.
The current simulator divides each total bend by five for node commands.
The solver grasp uses the case's exact `goal_command`, including linear and
roll (currently linear0, roll **-90 degrees**, not +90). Home comes from the
mechanism preset, currently linear-120mm, roll0, q3=-170deg, q4=+170deg.
Do not duplicate these endpoints as unrelated hardcoded commands.

GA changes intermediate commands and their durations. Initial grasp and terminal
Home command are fixed endpoints. C posture is required only for initialization;
q3/q4 may change sign while opening and returning to the S-shaped Home posture.

## Initial-state admission, before population evaluation

1. Place the free cylinder at the case's nominal pose, and command the exact
   solver grasp. Do not fix the object, disable gravity/collision, or add a weld.
2. Simulate up to1s of settling at this command with all collision and object
   bounds active. The reference object pose is the nominal pose **before**
   settling; do not hide settling drift by resetting the reference.
3. Require the last0.2s continuously to have proper-face loaded contacts on both
   proximal and distal groups, negative q3/q4, no fault/wrong-face contact,
   floor support, object linear speed<=10mm/s and angular speed<=5deg/s.
   Actual linear error<=2mm, roll error<=2deg, each segment's total-bend
   error<=3deg. Check the existing within-segment joint-spread limit too.
4. Save the settled complete physical state and restore the same state for each
   candidate. Record actual commands, object drift and contacts separately from
   nominal solver values. A load-bearing contact is not evidence of liftability.

If this fails, label the anchor `initial_grasp_invalid`; stop that anchor before
GA. Report missing contact, tracking error, collision, drift or instability.
Do not count it as128 failed trajectories or silently modify the target q.
Any preload/pose correction needs its own explicit recorded preparation rule.
An exactly tangent geometric solution may have no loaded contact in simulation.

## Hard rejection, every physics tick

Keep dt=0.01s, four solver substeps, and control interval0.1s initially.
Check at every physics tick, not only waypoint/control endpoints. Record the
first violating tick, all simultaneous reasons and peak values. Rejection is
permanent even if the candidate later recovers. Freeze scoring after termination.

| Condition | Reverse-GA rule | Provenance |
|---|---|---|
| Command position limits | linear[-173.333,0]mm; roll[-90,+90]deg; each total bend[-180,+180]deg | Existing mechanism bounds |
| Command slew | <=0.5mm linear,0.01rad roll,0.015rad total bend per10ms tick | Existing action rates divided by10 |
| Actual joint bounds | Existing measured fault margins: linear3mm, roll0.03rad, bend0.025rad per node | Existing `HugEnv.tick` |
| Segment deformation | max node angle minus min node angle>0.05rad within either five-node segment rejects | Existing measured spread |
| Forbidden contact | Moving arm–housing/wedge/camera/Go2; non-assembly arm self-contact; object–protected body; moving arm–floor; camera–floor/protected body | Existing `contacts()` pair policy |
| Contact activation | Existing generic loaded-contact threshold>1e-4N; retain adjacent/rigid-assembly exclusions | Existing contact filtering |
| ZED-side object contact | Any loaded wrong-face contact rejects; force>0.01N, neutral strip1mm, normal tolerance0.15 | Existing local-frame face classifier |
| Excess penetration | >5mm for currently monitored non-assembly relevant contacts rejects, even without force activation | Existing penetration rule |
| Invalid physics | Nonfinite robot/object state or active contact data, invalid quaternion/normal, or physics backend error | Reject result; stop run on systemic/batch corruption |
| Object XY excursion | `max_t norm(p_xy(t)-p_xy_nominal)>50mm` rejects | New reverse scope: whole episode, including airborne motion |
| Object tilt | Cylinder axis relative to initial upright axis>10deg rejects; compute from orientation, not centre height | New initial default |
| Extreme escape | Retain existing initial-relative object XYZ distance>1m guard | Existing fault rule |

Equality at the configured limit is allowed. Do not introduce a new measured
velocity cutoff from the command slew limit: physical feedback can lag or
overshoot a rate-limited command. Record measured speed separately.

Permitted: proper/neutral-face arm–object contacts, bounded sliding, opening,
temporary recontact, loss of grasp, either sign of q3/q4, and cylinder–floor
contact. Brief lift within object excursion/tilt bounds is permitted during
withdrawal; final floor support is required. Object-axis spin is unrestricted
for a circular cylinder, but terminal angular speed is still bounded.

The forward GA's airborne50mm exemption is deliberately **not** inherited here:
the reverse task must leave the cylinder near its original location. Neither
loss-of-grasp, the old drop flag, nor failure to lift is a reverse-GA rejection.

## Home success and time budget

Episode budget:20s including verification; initialization settling is separate.
Require **all** the following continuously for1s, starting whenever they become
true. Any interruption resets this timer. Success ends the lane immediately.

- Measured linear within2mm of Home, roll within2deg, total q3 and q4 each
  within3deg; existing node-spread/bound checks also pass.
- Terminal command is Home (not a compensating off-target command).
- Measured arm speeds: linear<=2mm/s, roll and each total bend<=2deg/s.
- No loaded object contact with any robot link, using the generic>1e-4N threshold;
  no arm/object contact penetration>1mm, including unloaded contact entries.
- Cylinder has loaded floor contact, remains within50mm/10deg bounds, and has
  linear speed<=10mm/s and angular speed<=5deg/s. This catches an arm that leaves
  a moving or falling cylinder behind. Finite observation time is not a proof of
  indefinite stability.
- No earlier hard rejection anywhere in the episode.

No dedicated approach/opening/rolling timestamps are imposed. Remove the old
`steps*2/3` waypoint schedule and fixed final-third dwell. All intermediate knot
times and total movement duration are searchable within the20s budget; reaching
Home starts the1s verification window. A clean timeout is an incomplete path,
not a successful curriculum artifact and not a hard physical violation.

## Selection without another weighted reward sum

Use separate ordered classes: verified reverse success > clean incomplete path
> rejected path. Every successful path outranks every failure regardless of time.
PPO rewards and forward grasp/lift/hold progress are not selection terms here.

For measured Home error, define

`E = max(abs(linear-home)/2mm, abs(roll-home)/2deg,
         abs(q3-home)/3deg, abs(q4-home)/3deg)`.

For successful candidates, minimize lexicographically:

1. completion time, including the1s verification;
2. normalized commanded path length
   `sum_ticks sum_j abs(delta_command_j)/(upper_j-lower_j)`;
3. peak object XY excursion, then peak tilt.

For clean incomplete candidates, first prefer a nonzero **current** Home
verification window and maximize its current duration; otherwise minimize
terminal E. Tie-break by terminal object-contact force, then normalized path
length, then peak object excursion. Do not use historical nearest-Home or a
historical verification maximum: a path that approaches Home and leaves again
must not keep that credit. Report E even when a verification window is nonzero.

Rejected paths can fill otherwise-empty breeding slots only after all clean
candidates. Order them by smallest Home E at their last valid tick, then longer
valid prefix. This is an exploration tie-break; no rejected path is exportable.
If a lane fails at its first tick, use the admitted initial-state E. A global
physics error invalidates the generation instead of awarding a candidate rank.

Retain128 candidates and100 generations as the first comparison budget. Keep
raw tenth-generation parades. Label their direction `grasp-to-home` in metadata;
the video itself remains caption-free. Preserve old forward experiments.

## Verification and curriculum export

A search success becomes `reverse_path_verified` only after three independent
nominal replays from the admitted grasp preparation satisfy every criterion.
Keep each replay's initial-state and physics provenance; no start-state forcing
after simulation begins. This establishes nominal repeatability, not robustness
or the ability to execute the path in reverse.

Store a versioned NPZ with full joint positions/velocities (including individual
bend nodes), commanded state, cylinder pose/velocity, timestamps, proper/wrong
face and floor contacts, Home/object metrics and violation masks. Also store
restorable simulator state where supported and source/model/physics hashes.
q-only waypoints are insufficient when the cylinder moves. A sampled reset must
be reproducible either through a validated complete-state restore or forward
replay of the reverse path prefix; restoring q alone is not accepted.

Curriculum uses the recorded sequence from grasp toward Home to move its reset
distribution. It does not negate velocities or treat time-reversed actions as a
successful forward demonstration. Preserve actual velocities in replay-derived
states; any zero-velocity/settled reset variant needs separate physical checking.
Only candidate curriculum states that pass restoration/contact checks can enter
the reset pool. Retain intermediate data at every10ms physics tick; selecting or
simplifying waypoints later must preserve swept-path checks.

The terminal Home may have an object offset within50mm. Track this explicitly:
it is not identical to the nominal Home+original-object task. Final RL validation
must include Home with the original object placement. The endpoint neighborhood
must first produce full grasp-and-lift successes before the curriculum advances.
Reverse-GA success alone does not certify liftability of the solver grasp.

Each triangle anchor is independently admitted, searched and verified. Missing
anchors remain missing; do not silently replace them or claim a complete
three-anchor interpolation experiment. Start-pose progression, PPO training and
cross-anchor interpolation are subsequent tasks, not reverse-GA success criteria.

## Acceptance checks for implementation

- Reject50.001mm excursions even while airborne and after later return; accept50mm.
- Tilt uses oriented cylinder axis; a tipped object with unchanged centre rejects.
- Proper contact loss and q4 sign change toward Home remain legal.
- ZED/forbidden-contact rejection stays permanent after later clean Home arrival.
- Initialization failure cannot enter GA or be hidden by reference rebasing.
- Commanded Home with measured tracking error cannot succeed.
- Home passage, recontact, residual object motion or lost support resets the1s timer.
- A stable partial pose or clean timeout is never exported as a successful path.
- Success outranks low-error failures; historical near-Home events earn no credit.
- Restore/replay checks retain object state and do not invent a nominal placement.
- Three replay failures prevent verified export; no automatic PPO launch follows.

Source references: `hug_ga/search.py`, `hug_ga/constraints.py`,
`hug_c/configure.py`, `hug_c/environment.py`, `hug_c/contacts.py` under
`workbench/research/experiments/`. Physical constants are inherited;20s/1s,
50mm/10deg, endpoint tolerances and rest thresholds above define this v1 proposal.
