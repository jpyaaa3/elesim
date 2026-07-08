# Body Pitch Preview — Final Result Notes

- Analysis window: full trial duration; visibility from sim eye-camera in-frame projection.
- Body Pitch (IMU pitch-rate lead) is the proposed deployable method for real GO2 hardware.
- Gait Preview is included only as a sim-only ablation that assumes access to gait phase.
- Compared with No Comp., v RMS decreased from 0.374 to 0.135, corresponding to approximately 63.8% reduction.
- Compared with Reactive feedback, Body Pitch reduced v RMS from 0.211 to 0.135, corresponding to approximately 36.0% reduction.
- Body Pitch reduced vertical image-space error versus Reactive and No Comp. baselines in this protocol (standoff 0.85 m, 30 s max duration).

## Summary metrics

- Body Pitch v RMS: 0.135
- Reactive v RMS: 0.211
- No Comp. v RMS: 0.374
- Gait Preview v RMS (ablation): 0.138
- Reactive+FF v RMS: 0.223

## Relative reductions (Body Pitch)

- vs No Comp.: 63.8%
- vs Reactive: 36.0%
- vs Gait Preview: 1.7%
- vs Reactive+FF: 39.3%

## Best single trial

- Best Body Pitch v RMS among logged trials: 0.125
