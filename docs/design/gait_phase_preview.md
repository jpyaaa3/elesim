# Gait-Phase Preview Gaze

> Proposal/experimental design. It does not override the current runtime
> contract; see [`../architecture.md`](../architecture.md).

## Status

Design proposal, not implemented in the current tree.

The current code already transports GO2 gait phase and period through portions
of state, protocol, and metrics, and reserves gait-preview metadata fields.
However, there is no gait-phase template builder, template model, or runtime
gait-phase preview branch. Those missing pieces must be implemented and tested
before this mode can be selected or reported as an experiment condition.

## Motivation

Pitch-preview estimates disturbance from instantaneous body pitch rate. A
walking robot also produces repeatable periodic image motion tied to its gait.
The proposed controller learns that periodic disturbance from baseline runs and
previews how it will change over the next short horizon.

Define a phase-indexed image disturbance template:

```text
D(phi) = [D_u(phi), D_v(phi)]
```

The image error already contains the disturbance at the current phase. The
preview term must therefore use the change in disturbance, not its future
absolute value:

```text
D_now = D(phi_now)
D_future = D(phi_future)
phi_future = (phi_now + preview_horizon_s / gait_period_s) mod 1

preview_term = scale * (D_future - D_now)
du = -(J^T Q J + R)^-1 J^T Q (s + preview_term)
```

Using `scale * D_future` would count the current disturbance twice.

## Phase Convention

Template generation and runtime lookup must use the same phase origin. Resolve
phase in this order:

1. Transported `go2_gait_phase`, normalized modulo one.
2. `(sim_time_s mod gait_period_s) / gait_period_s`.
3. A wall-time oscillator anchored at the first run sample, only as a final
   fallback.

Every template must store phase source, phase anchor, gait period, phase
offset, bin count, source run IDs, and nominal velocity. Runtime should reject
or clearly warn about incompatible phase conventions and gait periods.

## Template Construction

The builder should consume both walking and camera CSV files. Camera-only
templates are not acceptable because they cannot reliably recover the MPC gait
phase convention.

For each source run:

1. Validate run metadata, gait, velocity, and run ID.
2. Merge camera samples with walking samples by simulation time when possible,
   otherwise by wall time with an explicit tolerance.
3. Remove startup and shutdown transients.
4. Keep live, visible target samples only.
5. Assign samples to phase bins and aggregate robustly.
6. Fill sparse neighboring bins without hiding large coverage gaps.
7. Save sample counts and diagnostic plots with the template.

Baseline source runs should use gaze `off`; otherwise the template learns the
residual of another controller rather than the raw periodic disturbance.

## Required Components

Suggested ownership under the current hierarchy:

- `pilot/src/elesim_pilot/gaze/gait_phase_preview.py`: load, validate, interpolate, and compute
  preview deltas.
- `misc/research/analysis/build_gait_phase_template.py`: merge runs and build template
  artifacts.
- `pilot/src/elesim_pilot/gaze/gaze_service.py`: an explicit gait-preview runtime branch with
  per-tick UV fallback and provenance.
- `pilot/src/elesim_pilot/config`: gait period, horizon, template path, bins, phase offset, and
  scale configuration.
- `pilot/src/elesim_pilot/observability`: phase source, current/future template values, preview
  delta, and fallback diagnostics.

Do not overload the existing `pitch_preview` label. A gait-phase controller
needs a distinct mode and `preview_type` so experiment results remain
traceable.

## Validation Gates

Unit and contract tests must cover:

- phase wrapping and interpolation;
- sparse-bin handling and template metadata validation;
- identical current/future phase producing approximately zero preview term;
- exact `D_future - D_now` sign and scale;
- camera/walking merge and transient trimming;
- fail-fast behavior for a disabled mode, missing template, or incompatible
  gait period;
- runtime UV fallback with an explicit reason;
- requested versus actual mode in run metadata.

Recommended experiment order:

1. Build one template from repeated `off` runs under a fixed gait and velocity.
2. Inspect phase coverage and periodicity before enabling control.
3. Run several gait-preview trials with near-zero fallback ratio.
4. Compare under identical conditions against `off`, `uv`, `uv_ff`, and
   `pitch_preview`.

The first success criterion is lower vertical RMS error than pitch-preview. A
stronger result is improvement over `uv_ff` without increased target loss or
excessive control effort.
