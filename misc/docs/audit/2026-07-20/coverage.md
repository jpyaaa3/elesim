# Line-Execution Audit, 2026-07-20

This is a standard-library `trace --missing` audit, not branch coverage and not
a claim of physical validation. Reproduce one role with:

```bash
python3 misc/tooling/quality/line_coverage.py controller
```

## Strongly Exercised Control Code

- Controller trajectory: 97%
- Controller workflow coordinator: 87%
- Pick replay diagnostics: 93%
- Camera metrics: 97%
- UV Jacobian: 94%
- Local image Jacobian: 91%
- Grasp trajectory: 100%
- Feasible ready pose: 85%
- Sag drift frame: 86%
- Simulator model bundle validator: 82%
- Robot runtime: 79%
- UI operator session: 81%

The deterministic property tests cover 250 random UV contractions, 250 random
LJI contractions, 200 equal-sag reconstructions, estimator recovery, command
caps, finite-input rejection, and 50 generated reachable FK-to-IK round trips.

## Expected Headless Gaps

Low execution appears mainly in code requiring a window, a live camera, ROS2,
Unitree hardware, Genesis entities, or optional telemetry exporters. Examples
include UI panels, RealSense/YOLO pipelines, Genesis camera mount operations,
the Dynamixel transport, and the Unitree bridge. Raising these percentages with
mock-only assertions would not prove the real integration.

These areas therefore remain explicit live gates:

1. Genesis Look-Aim-Grasp with rendered RGB-D and contact.
2. Jetson Dynamixel current/read-failure/deadman behavior on actual buses.
3. Unitree ROS2 topic and velocity-stop behavior.
4. Camera frame continuity during stop, blind handoff and reconnect.

## Mutation Sensitivity

Seven focused mutants are killed by the current suite: protocol version,
router lease, robot stale sequence, robot GO2 deadman, UV command direction,
LJI gain semantics, and equal-sag finite-input rejection. Run them with:

```bash
python3 misc/tooling/quality/mutation.py
```
