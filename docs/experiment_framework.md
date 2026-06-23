# Experiment framework

Walking/camera stabilization experiments use a **manual three-process stack**:

1. `python host.py --config config.ini`
2. `ELESIM_WALKING_METRICS=1 ELESIM_RUN_ID=<run_id> python sim.py --config config.ini`
3. `python tools/walking_baseline.py --run-id <run_id> ...`

## Run ID rule

- `ELESIM_RUN_ID` is read **only at sim startup**.
- Baseline/demo scripts **validate** `--run-id` against the environment; they do not export it.
- One `run_id` per sim process. `--repeat N` requires sim restart between trials.

## Pitch-trim sweep

Pitch-trim gains load from `config.ini` at sim startup. Each sweep case needs a sim restart.
See `tools/pitch_trim_sweep.sh`.

## Gaze modes

| Mode | Description |
|------|-------------|
| `off` | No gaze worker |
| `uv` | UV feedback only |
| `uv_ff` | UV + base angular velocity feedforward |
| `preview` | Interface stub only (not connected by default) |

## Known limitations

- Walking CSV requires sim with matching `ELESIM_RUN_ID`.
- Preview MPC is stub-only in v1.
- Camera `sim_time_s` may be empty if host timestamp unavailable; merge uses `wall_time_s`.
- No grasp/LJI validation in this framework.
