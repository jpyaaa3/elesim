# Open Issues

> Path migration note (2026-07-18): references below to `engine/`, root
> launchers, `configs/`, `assets/` and `crafts/` describe the pre-v3 layout.
> Their current homes are the top-level release projects' `src/` trees, project-local `config/`,
> `model/source/assets` and `model/bundles/default`. Historical observations
> are retained so their evidence is not lost.

This file tracks known unresolved or deferred issues so they do not get lost while higher-priority work continues.

## Current Status (2026-08-03)

This section is authoritative for the deployment-based architecture. The long
sections below it are retained as pre-refactor evidence; paths and test counts
there are historical unless repeated here.

### P0. Router-Free ROS 2/DDS Migration Needs Live Proof And Typed-Surface Follow-Up

- Status: open for live validation and typed service/action binding. Router/ZMQ
  removal, the direct DDS carrier, software-only contracts, and the
  four-process CycloneDDS smoke are complete.
- The final topology has no Router, ZMQ, CurveZMQ, CURVE key or ZAP policy.
  Robot/Sim own their motion leases, Sim separately owns its UI
  session, RGBD is DDS, WebRTC signaling is a reliable DDS request/reply
  exchange, and WebRTC pixels remain DTLS/SRTP.
- Control/signaling currently uses the bounded `PeerEnvelope` DDS message.
  Typed service/action definitions are generated but not runtime-wired, so they
  must not be presented as active interfaces.
- Completed software evidence includes generated ROSIDL artifacts, four
  Router-free release trees, protocol/setup and isolated role suites,
  duplicate endpoint fail-closed behavior, restart identity handling,
  target-owned lease/session expiry, stale sequence rejection, coherent RGBD,
  two-stream negotiation, and one four-process real-RMW same-host smoke.
- Required live evidence: one host, L2 multicast, routed static peers, routed
  VPN, global IPv6, production SROS2 enforcement, loss/reorder, process kill,
  Wi-Fi/VPN reconnect, and explicit rejection of NAT-only layouts.

### P0. Live Look-Aim-Grasp Convergence Is Not Proven

- Status: open; highest priority.
- Automated evidence now exists: deterministic UV/LJI/equal-sag/ready/IK
  properties, a headless phase workflow, and recorded failing/healthy grasp log
  replay.
- The recorded failure is correctly diagnosed as an object-world jump,
  measured-motion stalls, a blind handoff near 98 mm, about 20 degrees of look
  error, and final abort.
- Missing evidence: one complete Genesis run and one hardware run proving
  target visibility, bounded camera motion, decreasing `remain`, safe blind
  handoff, plausible contact, and gripper closure.

### P1. Camera And Perception Timing Still Need Live Validation

- Status: open.
- Unit tests prove that Pick stop does not own camera shutdown and that worker
  lifecycle/state transitions are explicit.
- They do not prove RealSense/YOLO/Genesis frame continuity, depth validity, or
  tracker identity while the hand-eye camera moves.
- Required evidence: timestamped frame/drop metrics through Look, Aim, LJI,
  blind handoff, operator stop, reconnect, and target reacquisition.

### P1. Multi-Host And Hardware Deployment Remain Manual Gates

- Status: open.
- Same-host tests do not prove DDS multicast/static-peer discovery, direct
  user-data locators, vendor port mapping, QoS under loss, or SROS2 permissions.
- Actual LAN/routed-VPN/global-IPv6 routing, Jetson USB/serial behavior,
  ROS2/Unitree domain/context coexistence, clock skew, packet loss and process
  restart timing have not been exercised by the live gate.
- Ordinary IPv4 NAT, CGNAT and symmetric NAT are deliberately unsupported.
  TURN relays WebRTC media only; it must never be presented as DDS traversal.

### P1. Remote Genesis Video And Control Need A Live Gate

- Status: open.
- Existing tests prove independent observer/hand-eye media, Sim
  main-thread mailbox behavior, DDS session/signaling contracts, and a
  same-host four-process DDS smoke.
- They do not prove Genesis GPU offscreen capture, aiortc encode/decode latency,
  actual ICE selection, Coturn relay, or responsive orbit/pan/zoom on two real
  machines.
- Required evidence: one direct-LAN run and one TURN-relayed run showing both
  live streams, pause/step/reset semantics, bounded command backlog and clean
  reconnect after either process restarts.

### P1. DDS Security And Remote Setup Need Live Validation

- Status: open; documented operational limitation.
- State schema v8, the non-secret connection topology, separate DDS/SSH
  endpoints, SROS2 Authority generations, per-host role bundles, pinned SSH
  host keys and transactional all-host activation/rollback are implemented and
  software-tested. `external` keystores remain distinct from `managed`
  generations.
- `elesim-connections` must keep the complete Authority on the operator laptop,
  distribute public material plus only each host's assigned enclaves, and fail
  closed before or roll back after a partial deployment. It must never infer a
  DDS locator from an SSH hostname/port.
- `trusted-network` has no DDS encryption and is acceptable only behind a
  deliberate LAN/VPN interface and firewall boundary. `ROS_DOMAIN_ID` is not
  security.
- Managed Coturn deliberately mounts the REST HMAC secret into Coturn and the
  co-located Sim, which issues short-lived session-bound credentials.
  UI must never receive that secret. External TURN can use independently
  provisioned credentials.
- The previous in-process `UnitreeRos2Bridge` security/context conflict is
  resolved in software: a dedicated `elesim-unitree-bridge` daemon owns stock
  local/plaintext Unitree DDS on a distinct private NIC/domain, while Robot
  uses bounded credential-checked Unix IPC and remains the only inter-host
  SROS2 participant. Unitree topics are not added to the Elesim policy.
  Remaining evidence is physical Jetson/GO2 validation of NIC confinement,
  account/ACL setup and stop deadlines under bridge loss or malformed traffic.
- Required live evidence: the browser flow on real hosts, production
  RMW/SROS2 enforce mode including unauthorized publish/subscribe denial, a
  non-default SSH management port and pinned-key failure, full generation
  rotation/rollback, managed/external Coturn lifecycle, and exact cleanup
  commands.

### P1. Fixed Self-Hosted Containers Need Host/GPU Validation

- Status: open for live Docker validation; generators and ownership guards are
  implemented.
- General mode uses fixed `elesim-runtime` role containers and Developer mode
  uses one persistent `elesim-dev` plus optional `elesim-jaeger`; no external
  personal Compose environment is part of the supported test path.
- Required evidence: clean install/build/start/down on Ubuntu and WSL, repeated
  `elesim-dev` shells proving no temporary container proliferation, fixed-name
  collision diagnostics, NVIDIA/CPU variants, GUI forwarding and Jaeger.

### P2. Broad Runtime Fallbacks Need Continued Audit

- Status: open.
- A current scan finds roughly 267 broad exception handlers across runtime projects
  and tooling. Many are deliberate optional-driver/UI fallbacks, but the count
  is too large to assume every one is observable and safe.
- Continue replacing silent fallback with typed expected errors and structured
  endpoint/UI health, especially around cameras, Genesis, ROS2 and telemetry.

### P2. Headless Coverage Is Weak At Physical Adapters

- Status: open by design, not hidden.
- Pure control code is strongly exercised (UV 94%, LJI 91%, workflow 87%, replay
  93%, robot runtime 79%), while UI panels, RealSense/YOLO, Dynamixel transport,
  Unitree bridge and Genesis camera operations remain low.
- See `docs/audit/2026-07-20/coverage.md`. These gaps require integration rigs,
  not assertions over deeper mocks.

### P2. Genesis And Upstream Dynamics Warnings Remain

- Status: open, currently non-fatal.
- GO2 neutral qpos and neutral self-collision filtering still need a live
  contact/dynamics decision. Inertia-frame interpretation also remains pending.
- The `hppfcl` -> `coal` warning originates in the Pinocchio/convex-MPC
  dependency chain; Elesim does not directly import `hppfcl`.

### Closed By The 2026-07-20 Refactor

- Model bundles are self-contained, hashed, validated, and runtime-immutable.
- Robot owns physical I/O, measured canonical `q`, deadman/current/read-failure
  safety, stale-sequence checks and lease enforcement.
- Pilot and Sim use direct protocol-v6 endpoints; copied sibling-role
  implementations were removed and import boundaries are tested.
- Payloads, lifecycle, trace context, reconnect, partial commands, async UI
  state, Pick stop, camera lifecycle and WebRTC signaling have contract tests.
- Core algorithms now have deterministic property, headless and replay tests;
  seven critical mutants are killed by the suite.
- No deployment class may exceed 1000 lines and no function may exceed 900
  lines. Large workflow files remain sectioned by responsibility rather than
  split into one-file-per-method fragments.
- Every generated release is temporarily installed and probed without sibling
  installed releases or source-tree imports.

### Historical: Closed By The 2026-07-21 Remote-Sim Refactor

This records the superseded ZMQ/Router implementation. None of its transport or
credential choices are part of the final ROS 2/DDS architecture.

- Non-loopback Router and RGBD transport are CurveZMQ protected by default;
  Router authorizes exact public-key, endpoint-ID and role tuples.
- UI owns an independent Sim session instead of relaying camera input
  through Pilot.
- Sim publishes separate observer and hand-eye WebRTC views and applies
  orbit, pan, zoom, pause/resume, step, reset, speed and marker commands on the
  Genesis main thread.
- Coturn REST credentials are short-lived and minted by Router; generated
  static secrets and private keys are Git-ignored.
- TURN refresh replaces both UI peer connections inside the existing session;
  failed local replacement preserves the working receivers.

---

## Historical Pre-Refactor Backlog

The remainder documents how the project reached the current state. Treat an
item as current only if it also appears in the authoritative section above.

## General and Potential Codebase Risks (2026-07-01)

These are broader risks found while reviewing the current code shape and recent test results. They are not necessarily current failures, but they are places where "all tests pass" can still leave a real integration problem.

### 1. Headless Tests Do Not Prove Physical Closed-Loop Success

- Status: open.
- Context: Docker pytest currently passes (`299 passed, 1 warning`), but the suite is mostly contracts, synthetic scenarios, and mocked services.
- Risk: Look/Aim/LJI/Grasp can pass headless math and orchestration tests while still failing under Genesis physics, real camera timing, depth jitter, or hardware latency.
- What to add:
  - One repeatable sim smoke run that starts `host.py`, `sim.py`, `ctrl.py`/pick service equivalents, and validates a complete pick log.
  - A recorded-log replay test for `[Visual]`, `[Perception]`, `[Grasp-Ctrl]`, and `[Grasp] blind` sequences.
  - A pass/fail criterion for physical convergence, not only helper outputs.

### 2. Entry-Point And Behavior Modules Are Still Too Large

- Status: open.
- Evidence:
  - `engine/behaviors/pick/actions.py`: about 9341 lines.
  - `sim.py`: about 3055 lines.
  - `host.py`: about 2335 lines.
  - `engine/vision/perception/capture.py`: about 1831 lines.
  - `engine/core/config_loader.py`: about 1465 lines.
- Risk: State ownership and failure paths are hard to audit. Small fixes can unintentionally couple Look, Aim, Grasp, perception, and UI behavior.
- Suggested direction:
  - Split pick actions by phase (`look`, `aim`, `equal_sag`, `grasp_lji`, `blind_finish`) while keeping `ControlService` as the coordinator.
  - Move host transport, command arbitration, trajectory scheduling, and hardware IO into separate modules.
  - Add phase-level state objects so logs and tests can assert transitions explicitly.

### 3. Broad Exception Handling Can Hide Real Faults

- Status: open.
- Evidence: a repository scan found roughly 185 `except Exception` occurrences in `host.py`, `sim.py`, and `engine/`.
- Risk: A failed detector, socket, IK solve, camera update, or GO2 bridge operation may silently degrade into a fallback that looks like normal behavior.
- What to audit:
  - Replace expected optional-dependency failures with typed exceptions.
  - Convert silent `pass` blocks in runtime paths into structured debug/status messages.
  - Make recovery/fallback modes visible in host state so the UI/test GUI can show them.

### 4. Environment Parity Is Weak

- Status: open.
- Evidence:
  - Local `python3 -m pytest` is not available in the current shell, while Docker pytest passes.
  - Local direct imports can fail without dependencies such as `scipy`.
  - GO2 MPC import emits the upstream `hppfcl` -> `coal` deprecation warning.
- Risk: Laptop, Docker, and Jetson can run subtly different dependency stacks; a test result in one environment may not represent another.
- Suggested direction:
  - Document one blessed test runner and one blessed runtime runner.
  - Add an environment smoke command that checks `pytest`, `numpy`, `scipy`, `genesis`, `pinocchio`, `convex_mpc`, camera dependencies, and ROS2/Unitree dependencies when expected.
  - Track the `hppfcl` warning as dependency hygiene, not a pick pipeline defect.

### 5. Config/Profile Drift And Network Topology Need Stronger Ownership

- Status: open.
- Context: `configs/config.yaml` uses localhost endpoints, while `configs/config.pc.yaml` and `configs/config.jetson.yaml` contain explicit PC/Jetson TCP endpoints.
- Risk: Running the right process with the wrong profile can connect to stale hosts, use stale preview endpoints, or make it unclear whether traffic is local loopback, Wi-Fi LAN, or ROS2/Unitree transport.
- Suggested direction:
  - Print a startup profile banner in `host.py`, `sim.py`, and `ctrl.py` showing active config file, bind/connect endpoints, and mode.
  - Add a profile matrix documenting which process runs on laptop vs Jetson.
  - Add a startup handshake that rejects impossible combinations, such as PC profile binding Jetson-only endpoints.

### 6. Generated Runtime Artifacts Need Provenance Checks

- Status: open.
- Context: runtime behavior depends on generated files under `crafts/` such as combined URDFs. Recent linear-zero work specifically depends on regenerated URDF semantics.
- Risk: A stale generated URDF can survive source changes and produce a runtime that no longer matches the code or config.
- Suggested direction:
  - Stamp generated URDFs/manifests with source config hashes or build timestamps.
  - Add a cheap startup check that warns when `crafts/robot.urdf` is older than relevant assets/configs.
  - Keep the linear-zero check in the runtime smoke test, not only in static unit tests.

### 7. Sim/Real Command Semantics Need Boundary Tests

- Status: open.
- Context: previous work separated simulation-only motion behavior, host trajectory scheduling, and real/feedback state paths, but the boundary remains subtle.
- Risk: Sim may appear stable because it follows commanded q, while Real2Sim or real hardware depends on measured q, lag, or source ownership.
- What to verify:
  - For each source (`slider`, `ik`, `lji_step`, `perception`, `sim`), assert whether host applies trajectory smoothing, direct targets, or ignores it.
  - Confirm `sim_q` vs command `q` selection in Grasp/LJI feedback paths.
  - Add a replay test where measured q lags commanded q and verify LJI does not over-integrate.

### 8. Perception And Camera Lifecycle Coupling Is Still Risky

- Status: open.
- Context: Grasp/LJI depends on continuous center/depth updates, and earlier manual runs showed camera/perception stopping or jumping around grasp/stop transitions.
- Risk: A UI stop, pick stop, blind handoff, or recovery path can stop perception when the operator expected only motion to stop.
- Suggested direction:
  - Define explicit lifecycle states for detector, tracker, preview stream, sim-camera relay, and pick phase.
  - Add tests for "stop motion but keep camera alive" across Look, Aim, LJI, blind finish, and user stop.
  - Surface camera/perception state in logs and UI status, not only as side effects.

### 9. Property/Stress Tests Are Missing For Core Algorithms

- Status: open.
- Context: current tests pin known scenarios and invariants, but most algorithms are not tested over broad random or adversarial input sets.
- Risk: IK, LJI, equal-sag, feasible-ready, and UV control can still fail on edge geometry not represented by hand-written fixtures.
- Suggested direction:
  - Add deterministic random seeds for reachable/unreachable IK targets, noisy depth, singular Jacobians, and joint-limit cases.
  - Verify contracts such as "error decreases", "rejects ill-conditioned input", "stays inside command caps", and "reports a reason".
  - Keep exact numeric expected outputs only where the math has a unique answer.

### 10. Dependency Deprecation: `hppfcl` -> `coal`

- Status: open, low priority.
- Context: the full test suite passes but emits a warning in the GO2 MPC import test: `Please update your 'hppfcl' imports to 'coal'`.
- Current interpretation: the repo does not directly import `hppfcl`; the warning appears to come from the `convex_mpc`/Pinocchio dependency chain.
- Suggested direction:
  - Do not treat this as a pick or engine behavior failure.
  - Revisit when updating the Docker/Jetson dependency stack or if the warning turns into an import failure.

## Look-Aim-Grasp Validation Backlog (2026-06-29)

These are not necessarily code defects; they are the next behaviors that must be verified on the real/sim loop before treating the pick pipeline as stable.

### 0. Grasp End-To-End Is Still Unverified

- Status: open, highest priority.
- Context: Look and Aim have been repeatedly tuned, but the full Look -> Aim -> Grasp sequence has not yet been validated after the latest changes.
- What to verify:
  - LJI approach keeps the target visible long enough.
  - `remain` decreases monotonically enough to justify blind handoff.
  - Blind finish does not start while the look axis is still badly misaligned.
  - Gripper closes only after the pre-contact target is physically plausible.
- Evidence needed: one complete run log including `[Pick] look`, `[Aim]`, `[Pick] equal_sag`, `[Grasp-Ctrl]`, and `[Grasp] blind` sections.

### 1. Look Preferred-Vector Selection Needs Live Validation

- Status: open.
- Context: The UI now accepts `Ball dir`, and Look first tries that vector before falling back to rough pre-aim. This is code-tested but not behavior-tested.
- What to verify:
  - `Ball dir` convention is intuitive in real use: `ready/camera -> object`.
  - Preferred vector succeeds for normal target placements without unnecessary pre-aim.
  - Bad preferred vector fails gracefully into pre-aim instead of producing a terrible view pose.
- Evidence needed: logs showing both a preferred-vector success and a fallback case.

### 2. Rough Pre-Aim Fallback May Need Tuning

- Status: open.
- Context: Fallback pre-aim intentionally aims roughly at `u=+0.10, v=0.00` with a wide tolerance. The goal is to avoid a terrible Look seed without over-optimizing.
- What to verify:
  - It does not spend too many steps before Look.
  - It does not accidentally center too well and remove equal-sag signal.
  - It does not swing the arm wider than the failed preferred-vector Look would have.
- Key config: `look_pre_aim_max_steps`, `look_pre_aim_target_uv_u`, `look_pre_aim_tol`, `look_pre_aim_step_scale`.

### 3. Aim Motion Taper Needs Real Camera Validation

- Status: open.
- Context: Aim cap now tapers with remaining UV error, so motions should shrink near the target. Unit tests cover the cap math only.
- What to verify:
  - Large initial error still converges fast enough.
  - Near-target motion no longer overshoots or visibly swings.
  - Divergence/stuck recovery still triggers when needed.
- Evidence needed: `[Visual] aim step` logs showing `delta`, requested roll/seg, and whether equal-sag is accepted.

### 4. Equal-Sag Acceptance Remains A Critical Gate

- Status: open.
- Context: Aim can reach center but still fail with `aim centered but equal sag rejected`.
- What to verify:
  - Rejection reasons are dominated by real geometric failure, not overly strict thresholds.
  - The latest Look/pre-aim behavior produces enough ready-to-centered drift to estimate sag.
  - Accepted equal-sag correction does not move Grasp target laterally away from the object.
- Key config: `sag_drift_max_dir_error_deg`, `sag_drift_max_lateral_m`, `sag_drift_axial_only`.

### 5. LJI Grasp Pilot Needs Stability Audit

- Status: open.
- Context: The LJI path has many recent damping/settling changes. Tests cover sample quality and helper logic, but not the full closed-loop behavior.
- What to verify:
  - `dq_cmd` and `dq_meas` do not alternate signs in a way that shakes the camera.
  - `remain` does not stall around the blind handoff threshold.
  - Depth validity remains stable during approach.
  - Reacquire logic does not fight the main LJI step.
- Evidence needed: `[Grasp-Ctrl]` series with `u_err`, `v_err`, `z_err`, `dq_cmd`, `dq_meas`, `remain`, and any transition messages.

### 6. Blind Handoff Thresholds May Still Be Too Early

- Status: open.
- Context: Previous logs showed blind handoff around `remain ~= 98mm`, followed by IK failures and look error above tolerance.
- What to verify:
  - `blind_micro_start_m` / LJI handoff settings do not enter blind mode while lateral/look error is still large.
  - Blind extend/bisect path can reach close tolerance without repeated IK failure.
  - Post-blind `look_err` is below tolerance before closing.
- Key config: `blind_micro_start_m`, `lij_uv_handoff_m`, `grasp_waypoint_max_dir_error_deg`, `grasp_guided_handoff_m`.

### 7. Linear Zero Definition Needs Generated-URDF Confirmation

- Status: mostly fixed, needs one runtime confirmation.
- Context: Linear q definition was changed so `u_linear=0` maps to URDF/prismatic `q=0`, without a user-facing offset shim.
- What to verify:
  - Rebuilt `crafts/arm.urdf` / `crafts/robot.urdf` use `j_plate_housing` upper limit `0.0`.
  - Runtime starts with the linear joint physically at the intended zero pose.
  - IK and host clamps never command beyond q=0.
- Evidence needed: startup log after rebuild plus a quick `u=0` and `u=max` sanity check.

### 8. Perception Tracking/Depth Robustness Is Still A Separate Risk

- Status: open.
- Context: The pick pipeline now depends heavily on stable center UV and depth during Aim/LJI.
- What to verify:
  - Tracker does not jump between nearby detections or stale boxes.
  - Depth remains valid under arm/camera motion.
  - Mock/sim perception and real perception use compatible world-frame conventions.
- Evidence needed: perception logs around any sudden object-world jump or `depth_valid=false`.

## Inertia Interpretation In Genesis

- Status: unresolved, non-blocking for basic sim startup.
- Context: URDF export writes inertia values in the expected fields (`ixx`, `iyy`, `izz`). Genesis loads the model but exposes `link.inertial_i` with diagonal values reordered like principal moments.
- Observed example: `plate_physics.json` and `crafts/arm.urdf` contain `ixx=0.0750208333`, `iyy=0.300020833`, `izz=0.375`, while Genesis debug output shows `diag=[0.375, 0.3000208437, 0.0750208348]`.
- Current mitigation: `engine/robot/go2/mpc/payload_model.py` now handles multiple `link.inertial_i` shapes and can print raw inertial attributes via `ELISIM_DEBUG_INERTIA=1`.
- Next check: run with `ELISIM_DEBUG_INERTIA=1 python3 sim.py` and inspect `inertial_attrs` for an inertial frame rotation such as `inertial_R`, `inertial_rot`, or `inertial_quat`.
- If Genesis does not expose the inertial frame rotation, read arm inertia directly from URDF/physics JSON for payload compensation instead of using `link.inertial_i`.

## GO2 Neutral Qpos Warning

- Status: unresolved, currently non-fatal.
- Context: Genesis builds the GO2 at neutral `qpos0=0`, but GO2 calf joints have negative-only limits.
- Observed warning: `Neutral robot position (qpos0) exceeds joint limits.`
- Current mitigation: `sim.py` sets GO2 leg joints to a ready pose immediately after `scene.build()`.
- Remaining issue: the warning itself happens during build before the ready pose can be applied.

## GO2 Neutral Self-Collision Filter Warning

- Status: unresolved, likely tied to neutral `qpos0`.
- Context: Genesis filters some geometry pairs causing self-collision in the neutral configuration.
- Observed warning: `Filtered out geometry pairs causing self-collision for the neutral configuration (qpos0)`.
- Current interpretation: this is probably from the invalid neutral GO2 pose, not necessarily from the post-build ready pose.
- Next check: only revisit if actual ready-pose simulation shows unstable contacts or missing expected collisions.

## Update Repo Selective Integration

- Status: mostly integrated as of 2026-06-22.
- Context: `/home/user/dev/ws/humble_ws/update/elesim` contained newer upstream work: PC/Jetson split configs, GO2 hardware bridge, remote perception worker, GO2 mirror mode, and lazy GO2 MPC imports.
- Constraint: do not blindly overwrite current repo because current work added local GO2 assets under `assets/go2/` and `builders/go2_arm_merger.py`.
- Integrated:
  - GO2 local asset layout stays at `assets/go2/go2.urdf` with local DAE meshes.
  - `engine/robot/go2/hardware` now has `pose_source`, `SportModeState`, `LowState`, and 12-DOF leg sync support using `/sportmodestate` and `/lowstate` defaults.
  - `engine.core.protocol.pack_state`, `host.py`, and `sim.py` relay `go2_leg_q`.
  - `sim.py` supports `go2_locomotion.mirror_from_host`; mirror mode follows host base pose and leg q instead of running MPC.
  - Added `configs/config.pc.yaml`, `configs/config.jetson.yaml`, `perception_worker.py`, and GO2 hardware tests from update.
- Verification:
  - `python3 -m py_compile sim.py host.py engine/core/config_loader.py engine/core/protocol.py engine/robot/go2/locomotion/config.py engine/robot/go2/hardware/*.py perception_worker.py`
  - `env PYTHONPATH=. python3 -m unittest discover -s tests/scenarios/go2 -p 'test_10_lowstate.py'`
  - `env PYTHONPATH=. python3 -m unittest discover -s tests/scenarios/go2 -p 'test_11_bridge.py'`
- Remaining check: full PC/Jetson live run still needs real ROS2 Unitree topics and host/sim pair execution.
