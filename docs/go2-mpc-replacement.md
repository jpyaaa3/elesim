# GO2 locomotion dependency replacement

Decision date: 2026-09-16. Updated: 2026-10-05. The nominal acados adapter is
integrated and `payload/config/sim/config.yaml` currently selects `pympc`.
The latest CPU-MPC/Genesis-GPU flat-ground walking trials passed the bounded
scenarios in status.md; tracking and generalization limits remain. Current evidence and remaining gates are owned
by [status.md](status.md). The old convex backend remains an explicit rollback.

## Decision

Choose [IIT-DLSLab Quadruped-PyMPC](https://github.com/iit-DLSLab/Quadruped-PyMPC),
initially its nominal, gradient-based **acados CPU** controller. Its Python
single-rigid-body MPC implementation is a closer integration fit than adopting
a new C++ control stack. The upstream project offers acados and JAX alternatives;
GPU sampling is now an explicitly approved optional backend, with CPU remaining
the default. GPU contention with Genesis must be measured.
This is an engineering selection, not a claim that it is faster or more stable
in EleSim. See the [upstream README](https://github.com/iit-DLSLab/Quadruped-PyMPC/blob/main/README.md).

Alternatives considered:

- [OCS2](https://github.com/leggedrobotics/ocs2): a general C++ optimal-control
  toolbox, with a ROS 2 branch. Suitable in principle, but a larger integration
  commitment than the selected Python controller.
- [legged_control](https://github.com/qiayuanl/legged_control): its README
  explicitly states that it is no longer supported. Do not migrate to it.

## What must actually be replaced

The remaining old `convex_mpc` backend is not the source default.
`elesim_sim/robot/go2/mpc/controller.py` imports the old package's MPC,
reference trajectory, gait and leg controller, patches model paths/friction,
and overrides private force-bound calculation. `gait_adapter.py` also imports
its gait and robot model. Removing only the pip installation would break default
that rollback option; swapping only the optimizer does not retire the dependency.

The pinned dependency is also installed by `docker/shared/Dockerfile.app`,
`docker/dev/Dockerfile` and `elesim_setup/installer.py`. Retirement must cover
these paths and the import/build regression tests, not just one Dockerfile.

Keep locomotion implementation in Sim. Preserve Pilot's existing high-level
commands, the Genesis/Pinocchio state mapping, local torque limits and stop
behavior. Do not introduce Pilot imports from Sim, change DDS contracts, or
replace the physical Robot's Unitree bridge as part of this work.

## Integration risks and constraints

- This is **not a drop-in dependency**. The upstream
  [controller interface](https://github.com/iit-DLSLab/Quadruped-PyMPC/blob/main/quadruped_pympc/interfaces/srbd_controller_interface.py)
  consumes state/reference/contact sequences and inertia, returning contact
  forces and footholds. It also imports `gym_quadruped` types and global config.
  Establish an explicit Sim adapter; do not pull the demo simulator into runtime
  initialization or silently discard solver failure status.
- Upstream [configuration](https://github.com/iit-DLSLab/Quadruped-PyMPC/blob/main/quadruped_pympc/config.py)
  includes GO2 parameters. They are not EleSim's GO2-plus-arm model. Audit mass,
  inertia, center of mass, leg ordering, quaternion conventions, force frames
  and Jacobian-to-torque signs. Preserve `ArmPayloadCompensator` semantics through
  an explicit parameter mapping; a fixed stock mass is not acceptable.
- Own or adapt gait scheduling, swing trajectories and stance torque conversion
  explicitly. Test their boundaries independently before replacing the current
  `Gait`, `ComTraj`, `LegController` and private MPC subclass.
- The [installation instructions](https://github.com/iit-DLSLab/Quadruped-PyMPC/blob/main/README_install.md)
  require native acados and code-generation tooling. Pin reviewed upstream and
  acados commits plus their dependencies during the prototype; this survey of
  moving `main` is not a reproducible dependency lock. Check Python 3.10, NumPy
  1.26.4, CasADi/Pinocchio and the existing Humble image together. Compatibility
  is not established by upstream demo instructions.
- Keep native libraries in a reusable Sim/dev image layer, before application
  source copies. Prebuild required generators; forbid first-run downloads.
  Give generated solvers private, version/model-keyed cache paths. Do not install
  Conda, modify host Python/APT, or add host shell environment requirements.
  Do not remove CasADi/OSQP build steps until other consumers are audited.
- Preserve upstream [BSD-3-Clause notices](https://github.com/iit-DLSLab/Quadruped-PyMPC/blob/main/LICENSE)
  for any adapted code, and inventory transitive dependency licenses.

## Migration and acceptance

The original dependency-retirement sequence below is historical scope. The
2026-10-05 GPU option retains the CPU/acados implementation and default. Its
current progress and measurements are recorded separately in `status.md`.
The GPU kernel is Sim-owned, uses the same state/force adapter, and adapts the
upstream SRBD equations/cost weights with the BSD notice retained in its module.
It samples body wrench trajectories in parallel and distributes them over the
scheduled support feet. Mass/inertia are dynamic inputs; reset clears the warm
start and random key. This is a distinct optimizer, not a GPU execution toggle
for acados. GPU startup performs local JIT before readiness, without downloads.

1. Record a baseline with the currently pinned backend: standing, forward/sideways
   motion, turning, stopping, arm movement and added payload. Record tracking
   error, falls, force/torque bounds, solver failures and p95 solve time.
2. Build a pinned prototype in Sim/dev only. Test frame/leg mappings, payload
   parameter updates, constraints, invalid outputs and missed solve deadlines
   without requiring a live robot. Define explicit safe behavior on each failure.
3. Run the same Genesis scenarios with identical model, commands and seeds.
   Agree quantitative limits from the baseline before changing the default.
   Check reset, shutdown and authority loss as well as nominal walking.
4. Prove isolated release imports and offline startup, cold/warm Docker builds,
   no-change cache reuse and application-only rebuilds. Measure timings rather
   than assuming acados makes installation faster.
5. Switch the default only after those gates pass. Remove the old package and
   all imports, monkeypatches, private subclassing, installation hooks and stale
   tests. Keep historical instructions in documentation, not runtime shims.

No school-server training environment, checkpoint, live deployment or physical
motion was changed for this decision. Real hardware acceptance remains separate.
