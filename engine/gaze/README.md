# Gaze Workflow

`engine.gaze` owns standing and walking camera stabilization. It may request
control through Pick ownership APIs, but it does not own Look-Aim-Grasp
sequencing. Start with `gaze_service.py` for lifecycle and `stabilizer.py` for
the control policy.

Design and experiment references:

- `docs/design/preview_gaze.md`: implemented pitch-preview control contract
- `docs/design/gait_phase_preview.md`: unimplemented gait-phase proposal
- `docs/experiment_framework.md`: run identity, metrics, and comparison rules
