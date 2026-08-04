# Pitch-Preview Gaze Control

## Status

Implemented. The runtime mode name is `pitch_preview`.

This controller stabilizes the eye-in-hand camera while GO2 is walking. It is
separate from the Look-Aim-Grasp workflow and does not own grasp or LJI
transitions.

## Control Law

The controller uses the existing 2x3 image Jacobian and solves one regularized
least-squares step per gaze tick:

```text
s  = [u_err, v_err]
du = [delta_roll, delta_s1, delta_s2]

du = -(J^T Q J + R)^-1 J^T Q (s + preview_term)
preview_term = [0, b_pitch * pitch_rate_lead]
pitch_rate_lead = filtered_pitch_rate + tau * pitch_acc_est
```

This is MPC-lite rather than a horizon optimizer. The preview term predicts a
near-future vertical image disturbance, while `R` regularizes actuator motion.
The final command is clipped by both preview-specific and ordinary gaze limits.
The linear axis is not commanded by this controller.

Implementation:

- `pilot/src/elesim_pilot/gaze/preview_lite.py`: pitch-rate filtering and lead estimate
- `pilot/src/elesim_pilot/gaze/preview_mpc.py`: one-step solve
- `pilot/src/elesim_pilot/gaze/gaze_service.py`: lifecycle, fallback, logging, and command dispatch
- `pilot/src/elesim_pilot/pick/gaze_actions.py`: public control-service delegation

## Signal Priority

Pitch rate is resolved in this order:

1. `HostState.go2_base_ang_vel_body[1]` with a valid GO2 base timestamp.
2. Difference consecutive body-pitch samples using the GO2 timestamp.

The estimator uses the GO2 timestamp delta as its primary `dt`. The worker
period is only a fallback for a non-positive or non-finite delta. Estimator
state is reset when a gaze session starts or stops.

## Safety Contract

Experiment entry points must reject `pitch_preview` when
`gaze_preview_enable=false`. A trial must never be labelled as preview while
silently running another mode.

Once a valid preview session has started, an individual tick may fall back to
UV feedback if its host state, timestamp, pitch rate, or solve result is not
usable. That fallback is local to the tick and is recorded with a reason.

The distinction is deliberate:

- Invalid experiment configuration: fail before the run starts.
- Transient runtime input failure: use UV for that tick and record provenance.

## Experiment Provenance

Camera metrics record, among other fields:

- `preview_used`, `preview_fallback`, `preview_fallback_reason`
- `preview_dt_s`, `pitch_rate`, `pitch_rate_lead`, `pitch_acc_est`
- `preview_term_u`, `preview_term_v`
- `du_roll`, `du_s1`, `du_s2`, `preview_solve_time_ms`

Run metadata distinguishes `requested_gaze_mode` from `actual_gaze_mode` and
records preview use and fallback ratios. A requested pitch-preview run is
reported as actual `pitch_preview` only when at least half of eligible ticks
used the preview solve; otherwise it is reported as `uv`.

## Validation

Before comparing this mode with `uv` or `uv_ff`, verify:

1. `preview_used_ratio` is high and fallback reasons are understood.
2. The selected `b_pitch` sign reduces vertical RMS error under the same gait,
   velocity, target, and trial duration.
3. No trial is compared across mismatched run IDs or different sim lifetimes.

The historical positive/negative sign-sweep runner remains under
`research/experiments/run_preview_b_pitch_sign.sh`.

## Limitations

- The disturbance model uses body pitch only.
- It performs one solve per tick and has no multi-step control horizon.
- Performance depends on timestamp quality and the local UV Jacobian.
- It does not model gait phase explicitly. See `gait_phase_preview.md` for the
  unimplemented follow-up design.
