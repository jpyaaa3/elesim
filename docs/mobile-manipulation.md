# Mobile observation and C-hug

The experimental workflow targets one stationary upright 100 mm cylinder on a
flat Sim floor. The operator starts at measured Home with virtual torque ON and
open claws. The configured operator Home offsets are accepted for the initial
view; transport checks every arm segment back to the policy's exact canonical
Home before walking or replaying a learned path. It has no whole-building search
or general SLAM claim.

In Visual Servoing, place `cylinder_100mm.obj`; its red physical representation
matches the installed single-instance HSV profile. Selecting it hides the legacy
red calibration sphere so the detector sees one task target. Open Action and use
**Plan mobile task**, then **Start mobile task**. Planning performs preflight;
start authorizes the automatic observation/placement/scan/hug sequence. Stop
workflow cancels the parent and stops its base/arm child. Only a matching Sim
boot and motion lease may execute. No Robot motion is admitted.

## Execution

1. Acquire stationary RGB-D; use a finite arm-search budget if absent. A known
   D100 prior may localize a partial first view for observation planning only.
2. Choose a base pose with at least three feasible camera views. Unknown terrain
   is not assumed free; bounded downward looks can acquire floor coverage.
3. Return through checked arm segments to Home; follow an observed-free base
   path, including rotation sweeps and the transport-arm projection.
   If a direct arm return is blocked, try retracting the linear axis to the
   requested goal before folding; both segments need the same continuous
   collision certificates. Checked history prefixes remain the fallback.
4. Aim and run the existing multi-view scan with the body stationary. A partial
   vertical ROI is allowed. Separate measured and completion points remain saved.
   If a small standing shift reaches the original 2 mm / 0.25-degree capture
   limit, acquisition stops. With at least two already accepted views, the
   mobile task may finish reconstruction from those earlier frames, explicitly
   reporting `base_pose_budget_reached`. No frame beyond the capture limit is
   admitted and larger body jumps remain failures. All final fit, arrival,
   model, tracking and physical-hold checks still apply after reacquisition.
5. Fit the axis/diameter from **observed** points. Completion never certifies
   object diameter or position error. Capture provenance survives model latency.
6. Sample the analytic q triangle, forward-project to XY and reject sampled
   neighbourhoods that leave its supported branch. For each surviving relative
   placement, derive body yaw/position candidates and check local paths.
   An unobserved approach requests bounded floor looks and then reacquires the
   target; it never marks that approach free or repeats a completed scan merely
   because a floor look hid the object. Observation preparation reuses the
   scan's 12,000-pose candidate set, including its cooperative scheduling.
7. Transport Home, move, and reacquire. If the final pose hides the cylinder,
   allow a handoff only when the last observation's age plus motion-drift
   allowance fits the sampled solver neighbourhood. Otherwise observe again.
8. Recheck measured Home and model hash, then let Sim replay the timed command
   sequence in simulation time. Keepalive expiry, a route/scene change or
   control failure stops execution. The parent requires independent contact
   evaluation to label the task successful. The 20 s path length is a deadline:
   a fresh five-second loaded-hold result ends the task early, with an explicit
   cancellation of the remaining generic command samples.

The uncertainty radius is a conservative engineering allowance. The 16 boundary
probes plus centre are a sampled check, not proof of an entire continuous disk.
The map uses observed floor cells, measured non-floor surfaces and a declared
starting stance; it cannot certify hidden obstacles, cables, dynamic surroundings
or hardware foot placement. A footprint contains the Go2 body/leg envelope and
filled arm-sphere projections, preserving gaps rather than using one enclosing
rectangle. Supported tracking uncertainty is 1 mm linear / 0.015 rad per rotary
channel; both certificates and live tracking checks use that same bound.
Selected floor and scan viewpoints reserve one additional tracking displacement
at their endpoints. This leaves room for the realized arrival to pass the next
ordinary collision certificate. The extra reserve uses the existing sphere
motion bounds for self and observed-surface separation; it changes viewpoint
admission, not path-certificate limits or the fixed Home target.
Coarse distance bounds are refined against the requested uncertainty threshold;
ordinary and reserved-endpoint queries use separate cache entries. An inconclusive
ordinary lower bound must not reject an otherwise valid reserved endpoint.
Base route planning additionally checks measured floor around every sampled
footprint, including the starting stance. The padding is the existing base
position tolerance plus `2 R sin(yaw_tolerance / 2)`, where R is the transport
footprint's maximum radius. Missing starting clearance requests a floor view
there before selecting a destination. These extra cells require actual depth
observations; the declared occupied starting stance is not enlarged. Live base
checks still use the measured pose and original footprint. Arrival, freshness,
collision and physical hold limits remain unchanged.
Grasp placement separately reserves 4 mm for solver-XY drift during execution.
The body must stay within 3 mm translation and 0.5 degree rotation, and its actual
full-transform projection must remain inside that reserved 4 mm. These measured
engineering allowances are not statistical guarantees. A narrower supported
region can therefore be rejected before any motion. Target-fit conditioning
saturates at a half-circle of coverage; observing the far side cannot make an
otherwise identical cylinder fit less certain. Identified target points remain
in the collision map even inside the broad robot-body removal box.

`payload/config/pilot/mobile_manipulation.yaml` controls budgets and conservative
error allowances. All input q values are canonical linear metres, roll radians
and per-node bends; displayed q3/q4 are total segment bends. Solver offset and
node count are recorded in the model manifest.

The integrated Poisson mesher requests a 4 mm grid and explicitly allows up to
8 mm when the existing grid/triangle budget is reached. Only budget exceptions
trigger a retry. The published diagnostics retain requested/effective spacing
and all attempts; geometry quality failures are still failures. This inferred
surface is separate from the observed points used for cylinder localization.

## Verification

`workbench/research/experiments/mobile_manipulation/installed_ui.py` exercises the
published UI backend through its installed configuration, using the normal
remote Pilot and Sim services. Run it in the registered UI service's published
image with only the driver and evidence directory mounted. The ordinary UI must
be stopped first because the driver uses that exact UI endpoint and security
identity. No sibling runtime source is mounted for installed acceptance. It
checks both decoded WebRTC streams, target role, measured torque/claws/Home and
the new object revision before submitting the mobile intents. Fixture placement
goes only to Sim. Completion requires the parent's independent loaded-hold
result, rather than an operator acknowledgement. `--video` records both streams.
`--preflight-only` is explicitly labelled as a Home check; development diagnoses
with a patched service use `--evaluation-mode development` and are not installed
acceptance evidence.

Host checks cover state transitions, unknown-space rejection, base settling,
source/capture-time association, bounded correction loops, real analytic
interior-goal selection, operator intents, Sim-time replay/heartbeat, Robot
rejection and continuous loaded hold criteria. These are software evidence.
Waiting for inference holds one checked joint setpoint; measured drift never
silently becomes a new transport posture. Contact evaluation must reach the
final executed physics tick before a successful task can be reported. Stale
success at an earlier tick cannot authorize early completion.

RGB-D acquisition retries only `SimCameraTimeout` (no sample within the receive
window). Transport, decoder and camera lifecycle errors retain their original
reason and fail the workflow. An empty receive window does not establish that
the publisher stopped; compare a fresh source-fenced subscriber with the
workflow's waiting subscriber before changing their lifetime or freshness rules.

Floor observation and lost-target recovery each have a bounded attempt counter
under the existing total deadline. Looking down must not spend the opportunity
to recover the target afterward. Floor planning skips already attempted camera
views and reports missing/blocked footprint cells; predicted visibility never
marks those cells free. Only an actual accepted RGB-D observation updates them.
Before thinning the collision cloud, mapping retains one measured depth point
per observed floor cell. These points pass the same arm/body exclusion and
occupied cells still override free space. Floor looks reuse the existing
12,000-pose FK cache, prioritize missing cells in the proposed goal footprint,
and project them with the received camera calibration. If the goal footprint
is already observed, the search can cover the surrounding approach instead.
Every selected arm segment still requires the ordinary collision certificate.

The isolated physics probe is maintained in:

```
python -m workbench.research.experiments.mobile_manipulation \
  --output workbench/evidence/generated/mobile-physics-RUN
```

It constructs the actual runtime Genesis scene and checks Sim-time playback;
it does not claim a full RGB-D/DDS/mobile acceptance test. `--locomotion raibert`
is an explicit alternative-backend probe when the configured PyMPC environment
is unavailable; its results must not be presented as PyMPC validation.

Live acceptance still requires installed Pilot+Sim with ROS, model assets and the
configured locomotion dependencies: acquire from varied initial placements,
scan from a selected stance, walk into the C-band, then wrap/lift/hold using
interpolated paths. The planner must not receive fixture object pose/mesh/diameter
truth beyond the declared D100 task prior. Test target loss, invisible final
handoff, delayed telemetry, cancellation, model/scene/lease replacement and
unsupported terrain. Record partial outcomes rather than substituting fake
telemetry or considering a completed command path a successful grasp.

Measured runtime probes on 2026-11-15: native research Python cannot initialize
the configured PyMPC backend because Pinocchio is absent. With explicit Raibert,
a measured Home return followed by a 0.2 s replay passed. The encoder metadata,
render-replica joint state and body root agreed in that physics snapshot. The
ungrasped cylinder correctly failed the physical hold evaluation. Starting a
hold at the unsettled ordinary spawn failed tracking; application spawn and
nominal Home are different poses.

The initial learned 20 s anchor path failed tracking at 5 s in the explicit
Raibert probe. The repair replaces runtime pose writes with position actuators:
arm KP/KV 200/20, linear KP/KV 2000/200, force limit 100. The old kinematic mode
remains explicit and cannot run timed hugs. Prebuilt `*.physics.urdf` models
open the actuator limits and include camera collision; `*.physics-open.urdf`
also fix the claws open for a diagnostic comparison with the teacher. The
original planning URDFs retain their original bytes. Both camera replicas and
physics use measured joints. The default integration uses four substeps per
10 ms tick. Arm, floor and feet friction are 0.5, 0.01 and 0.8 respectively;
locomotion setup changes foot friction without overriding the arm material.

Research and runtime share contact-face math, assembly-contact exclusions and
loaded-hold math. Every node keeps its own local protected-face axis. Wrong-face
contact interrupts the hold window; camera/body collision, excessive penetration
or a joint-bound/spread failure remains an episode fault. Sim publishes a hashed
physics profile; Pilot rejects missing/mismatched dynamics or Robot mirroring.
This profile is a compatibility check, not proof of a successful grasp.
It also fences the effective command slew. The Sim bend-rate default is
0.3 rad/s per node, matching the validated paths. The old hardware-derived
0.28777 rad/s fallback accumulated up to 14 degrees of segment-angle lag when
silently applied after phase alignment; a lower-rate configuration now rejects
an incompatible timed path. No rate limiter bypass is added.

The installed normal Sim image provides Pinocchio and PyMPC even though the
native research venv does not. The maintained benchmark can use that image with
read-only source inputs, without starting normal services or building an image:

```
python -m workbench.tools.bench run mobile-hug \
  --install /absolute/current/install --instance SYSTEM --gpu GPU-UUID \
  --input config=/absolute/checkout/payload/config/sim/config.yaml \
  --input data=/absolute/checkout/payload/data \
  --input sim_source=/absolute/checkout/payload/runtime/docker/sim/app \
  --input protocol_source=/absolute/checkout/payload/runtime/common/protocol \
  --param 'base="free"' --param fixed_open_claws=false \
  --output workbench/evidence/generated/mobile-free-RUN
```

Add `--param video=true` for a pure MP4 per anchor, or `--param 'anchors=[0]'`
for one anchor. Genesis recording writes directly to that run's output at
simulation-time cadence. A fixed-base/fixed-open test
is diagnostic only. The first repaired fixed-base anchor passed a five-second
hold. A subsequent free-body PyMPC batch replayed all three paths, but its final
continuous hold windows were only 2.28, 0.73 and 2.88 s: all three failed
acceptance. Those fixtures still used stale world XY after the body settled;
the current probe places the cylinder at solver XY relative to the measured
settled body. The final corrected five-case matrix passed: three active-claw
anchors, fixed-open anchor 0 and the actual planner's selected interior point.
All reached a full five-second hold at 10.11–10.18 simulation seconds. Recorded
body histories satisfied the reserved drift bounds too. Videos are in
`workbench/evidence/generated/mobile-repair-20261115/final-v8/work/probe/run`.
Normal Sim/Pilot services remained stopped. Full-cycle success is not established.

Generate an analytic interior fixture in the prepared developer environment with
`python -m workbench.research.experiments.mobile_manipulation.planning --output FIXTURE.json`.
Run `mobile-hug-case` with the same four source/data inputs as above plus
`--input fixture=/absolute/FIXTURE.json`. `--param compare_claws=true` selects the
five-case comparison. The planning probe's declared floor and target are fixture
truth; its output must never replace measured RGB-D in the live workflow.

The probe returns zero for a completed short replay; with `--anchor 0|1|2`, it
also requires independent physical hold success. A JSON `replay_completed`
value alone never establishes grasp success. Reports go to the explicitly
selected generated-evidence directory. Do not send fixture coordinates back
into Pilot planning.
