# Elesim Architecture

## Read The System By Behavior

Start from the behavior that is failing, not from an entry-point script.

| Need to change | Start here | Follow into |
| --- | --- | --- |
| Look, Aim, Ready Pose, Grasp | `engine.pick` | `engine/vision/visual_servoing`, `engine/robot/arm` |
| Walking camera stabilization | `engine.gaze` | `engine/robot/go2`, `engine/observability` |
| Detection, tracking, camera relay | `engine.vision` | `engine/vision/perception`, `engine/vision/sim_camera` |
| Arm or GO2 mechanics | `engine.robot` | arm IK/Dynamixel or GO2 hardware/locomotion/MPC |
| Genesis scene and asset spawn | `apps.sim` | `engine.simulation`, `builders` |

## Process Flow

```text
ctrl.py -> apps.control -> ui + engine.pick
host.py -> apps.host -> hardware / GO2 / perception relay
sim.py  -> apps.sim -> builders + Genesis + simulation feedback
perception_worker.py -> apps.perception -> engine.vision.perception -> host
```

The root scripts are compatibility launchers. New process wiring belongs under
`apps/`; reusable robot behavior never belongs in a root script.

## Ownership Rules

- `apps/` owns process lifecycle, CLI parsing, sockets, and dependency wiring.
- `engine/pick` owns the Look-Aim-Ready-Grasp workflow and exposes
  `ControlService`, `ControlClient`, and `PanelState` as its public surface.
- `engine/gaze` owns stabilization policies. It may coordinate with Pick only
  through control ownership and service interfaces.
- `engine/vision` owns observation creation, hand-eye geometry, camera I/O,
  and visual-servo helpers. It does not own motion commands.
- `engine/robot` owns arm and GO2 kinematics, hardware interfaces, and motion
  primitives. It does not own UI or workflow sequencing.
- `engine/config` is the only application configuration import surface.
- `builders/` consumes explicit inputs to produce assets and URDFs; it must not
  import workflow code.
- `ui/` renders and dispatches user intent through public services only.

## Dependency Direction

```text
apps, ui, tools -> engine, builders
engine          -> engine
builders        -> assets and standard libraries
tests           -> public APIs plus deliberate contract internals
```

Do not add a second import path as a compatibility shim. Update repository
callers to the canonical package and remove the old path in the same change.

## Tests And Generated Files

- `tests/scenarios/` follows time-ordered behavior flows.
- `tests/contracts/` protects subsystem invariants during refactors.
- `tests/regressions/` pins concrete old failures.
- `crafts/` is generated runtime output. Do not mix an incidental regeneration
  with an unrelated code change.
- `results/` is experiment evidence. Preserve referenced presets beside the
  result when deleting an obsolete runtime component.
