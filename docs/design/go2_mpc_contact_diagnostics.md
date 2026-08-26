# Go2 MPC contact diagnostics

EleSim keeps the current convex MPC while separating controller, plant, and
contact errors with measurements. Replacing it with an NMPC/WBC stack remains a
later decision, after this evidence shows that Genesis realizes the requested
forces but stance feet still drift.

## Production defaults

- Genesis `1.2.0` uses an explicit Newton rigid solver with 50 iterations.
- Experimental `noslip_iterations` is disabled. It is not a friction model and
  must not hide controller/contact mismatch.
- The simulated surface friction is `0.8`.
- The upstream four-face rectangular MPC constraint uses `0.55`; a circular
  Coulomb projection enforces the physical `0.8` cone before torque generation.
- Normal GRF is capped at `180 N` per foot.
- MPC horizon time is snapped to an integer control stride. At the default
  50 Hz simulation/control rate, its effective timestep is `0.02 s`, not the
  nominal `0.025 s` derived from gait period.
- Genesis leg armature, damping, friction loss, and URDF force ranges are read
  back and verified at startup. Configuration failure stops startup.

Genesis 1.2.0 does not expose the newer elliptic-friction or Signorini options.
Those experiments require an explicit Genesis version upgrade and must not be
implemented by silently ignoring unsupported options.

## Capture

Enable the existing walking metrics gate and run a fixed command profile:

```bash
elesim-dev env ELESIM_WALKING_METRICS=1 ELESIM_RUN_ID=mpc-baseline elesim-sim
```

The Sim metrics directory then contains:

- `mpc-baseline_walking.csv`: base tracking and torque saturation;
- `mpc-baseline_contact.csv`: one row per foot at 10 Hz with stance state,
  foot-tip position/velocity, cumulative slip, raw/projected GRF, actual Genesis
  link contact force, GRF error, friction utilization, and raw/limited/applied
  joint torque;
- `mpc-baseline_meta.json`: configured and effective control/MPC cadence.

Contact diagnostics perform Genesis tensor readback only on their 10 Hz sample
cadence, not on every physics step.

## Interpretation

1. `friction_ratio > 1` in raw GRF identifies an optimizer/contact-cone
   mismatch. Projected desired GRF must remain at or below one.
2. Similar desired and actual normal force but a large tangential GRF error
   identifies contact realization or collision-manifold mismatch.
3. Similar desired/actual GRF with high stance slip identifies foot-point,
   Jacobian, or missing stance-feedback/WBC behavior.
4. Early torque saturation or desired/actual GRF divergence identifies plant
   dynamics, payload, or force-limit mismatch.

Use identical command duration and compare mean velocity error, stance slip
distance, GRF error, body roll/pitch RMS, torque RMS, MPC solve time, and RTF.
Only after model/contact/frame checks pass should an NMPC + WBC port be judged
against this baseline.
