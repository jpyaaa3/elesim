# Walking And Gaze Experiments

## Scope

This framework evaluates camera-space stabilization while GO2 is standing or
walking. It measures locomotion disturbance, target visibility, image error,
and gaze-control effort.

It does not validate Look, Aim, Grasp, LJI completion, or an automatic
gaze-to-grasp handoff. Those workflows may share control infrastructure but are
outside the experiment claim.

## Process Stack

The manual workflow uses three processes:

1. `python host.py --config configs/config.yaml`
2. `ELESIM_WALKING_METRICS=1 ELESIM_RUN_ID=<run_id> python sim.py --config configs/config.yaml`
3. `python tools/experiments/walking_baseline.py --run-id <run_id> ...`

The root scripts remain compatibility launchers; their implementations live in
`apps/`.

## Run Identity And Lifecycle

- `ELESIM_RUN_ID` is read by the sim process at startup.
- Experiment tools validate their `--run-id` against the environment; they do
  not change the identity of an already-running sim.
- One run ID corresponds to one sim process lifetime.
- Repeated trials require a new run ID and sim restart for every trial.
- Run metadata records the Git commit, arm preset, GO2 motion, requested gaze
  mode, actual gaze mode, and preview use/fallback ratios.

Mixing camera and walking logs from different run IDs invalidates timing and
comparison results.

## Gaze Modes

| Mode | Behavior | Status |
| --- | --- | --- |
| `off` | No gaze worker | Implemented |
| `uv` | Image-space feedback | Implemented |
| `uv_ff` | UV feedback plus base angular-velocity feedforward | Implemented |
| `pitch_preview` | One-step solve with pitch-rate lead disturbance | Implemented |
| gait-phase preview | Phase-indexed periodic disturbance template | Proposed only |

Pitch-preview design and safety rules are documented in
`docs/design/preview_gaze.md`. The unimplemented gait-phase follow-up is in
`docs/design/gait_phase_preview.md`.

## Measurements

Sim-side walking logs include body pose and velocity, command source, arm pose,
payload and torque diagnostics, control-rate metadata, gait phase, target frame
location, and fall state.

Camera logs include visibility and target-loss counters, normalized UV error,
tracking confidence, preview provenance, applied gaze increments, and solve
timing. Analysis should compare at least:

- horizontal and vertical RMS image error;
- visible-time ratio and target-loss events;
- body pitch and roll disturbance;
- control effort and saturation;
- preview use and fallback ratios;
- fall state and trial duration.

## Experiment Discipline

Change one condition at a time and hold gait, velocity, arm preset, target,
duration, and initial state constant. Compare multiple trials rather than one
representative run.

An experiment entry point must fail before starting when its requested mode is
disabled or lacks required configuration. Runtime fallback is acceptable only
for transient tick-level failures and must be logged.

Use `tools/analysis/analyze_walking_metrics.py` for summaries and comparison.
Batch runners and historical sign-sweep tools are under `tools/experiments/`.

## Pitch-Trim Sweep

Pitch-trim gains load at sim startup, so every gain case requires a sim
restart. Use `tools/experiments/pitch_trim_sweep.sh` as the operational
checklist and compare each case against an otherwise identical baseline.

## Known Limitations

- Walking CSV output requires a sim process started with matching run identity.
- Camera `sim_time_s` may be absent when a host timestamp is unavailable;
  analysis then merges by wall time.
- Metrics are richest in convex-MPC mode; other locomotion modes may expose
  fewer diagnostics.
- Pitch-preview is one-step and pitch-only, not a general horizon MPC.
- Gait-phase preview is not implemented despite some reserved protocol and log
  fields.
- This framework does not establish grasp or LJI success.
