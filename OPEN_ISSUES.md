0623 2012 test
# Open Issues

This file tracks known unresolved or deferred issues so they do not get lost while higher-priority work continues.

## Inertia Interpretation In Genesis

- Status: unresolved, non-blocking for basic sim startup.
- Context: URDF export writes inertia values in the expected fields (`ixx`, `iyy`, `izz`). Genesis loads the model but exposes `link.inertial_i` with diagonal values reordered like principal moments.
- Observed example: `plate_physics.json` and `crafts/arm.urdf` contain `ixx=0.0750208333`, `iyy=0.300020833`, `izz=0.375`, while Genesis debug output shows `diag=[0.375, 0.3000208437, 0.0750208348]`.
- Current mitigation: `engine/go2_mpc/payload_model.py` now handles multiple `link.inertial_i` shapes and can print raw inertial attributes via `ELISIM_DEBUG_INERTIA=1`.
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
- Constraint: do not blindly overwrite current repo because current work added local GO2 assets under `assets/go2/` and `builder/go2_arm_merger.py`.
- Integrated:
  - GO2 local asset layout stays at `assets/go2/go2.urdf` with local DAE meshes.
  - `engine/go2_hardware` now has `pose_source`, `SportModeState`, `LowState`, and 12-DOF leg sync support using `/sportmodestate` and `/lowstate` defaults.
  - `engine.protocol.pack_state`, `host.py`, and `sim.py` relay `go2_leg_q`.
  - `sim.py` supports `go2_locomotion.mirror_from_host`; mirror mode follows host base pose and leg q instead of running MPC.
  - Added `config.pc.ini`, `config.jetson.ini`, `perception_worker.py`, and GO2 hardware tests from update.
- Verification:
  - `python3 -m py_compile sim.py host.py engine/config_loader.py engine/protocol.py engine/go2_locomotion/config.py engine/go2_hardware/*.py perception_worker.py`
  - `env PYTHONPATH=. python3 -m unittest discover -s engine/tests -p 'test_lowstate_parser.py'`
  - `env PYTHONPATH=. python3 -m unittest discover -s engine/tests -p 'test_unitree_ros2_bridge.py'`
- Remaining check: full PC/Jetson live run still needs real ROS2 Unitree topics and host/sim pair execution.
