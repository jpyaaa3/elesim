# Pick Workflow

`engine.pick` owns the operator-facing Look, Aim, Ready Pose, and Grasp
workflow. `ControlService` is the public orchestration API; UI, tools, and
apps must not import an implementation mixin directly.

- `actions.py`: shared state, transport, basic controls, and public facade.
- `ready.py`: Look and Ready Pose resolution.
- `aim.py`: UV centering, equal-sag, and visual approach.
- `grasp.py`: guided grasp and Local Image Jacobian control.
- `perception.py`: capture lifecycle, preview, recording, and mock objects.
- `gaze_actions.py`: control-panel commands that delegate to `engine.gaze`.
