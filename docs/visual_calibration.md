# Depth-plane visual calibration

Pilot observes fixed planes in the original RGB-D optical frame, compares them
with the camera pose predicted by arm/body measurements, and optionally corrects
the camera's tilt and normal distance. Robot owns actuation and safety. This
observer never changes motor commands, the selected IMU model, or a hand-eye file.

## Operator workflow

1. Start perception with the normal Pilot RGB-D input. Observation defaults to
   six updates per second; it does not require a detected target object.
2. Open **Visual Calibration**. The perception preview outlines eligible planes
   in yellow and labels them with the same IDs as the candidate dropdown.
3. Initially keep the body and arm stationary. Select the actual fixed floor
   plane and press **Confirm selected plane** after it has been observed for
   three consecutive accepted frames. A large tabletop is only a candidate,
   never silently promoted to the floor. The selected reference outline is mint.
4. Inspect tilt error in degrees and signed height error in millimetres. An
   initial observed reference provides relative consistency, not an independently
   measured absolute camera height. A configured floor height is an explicit
   external assumption in the current world coordinate system.
5. Repeat at different stationary arm poses. Keep the reference fixed. Use
   **Export floor observations** to save up to the last 2,000 records as JSONL
   in a private `/tmp/elesim-floor-calibration-*` directory in the Pilot runtime.
   The UI reports the exact path on that host/container. Exports contain no
   camera image, credentials or executable model.
6. Enable **Apply visual correction** only after validating the source timing
   and reference. Application is off by default. The switch changes perception
   world coordinates consumed by Pick/Gaze, so future planned motion can change;
   the observer itself sends no motor commands. It is available only for a valid,
   synchronized observation. Stop/reset, source-boot or camera-calibration changes
   invalidate the old observation/reference as appropriate.

Reference selection/reset and calibration configuration changes require active
IK/Pick/Gaze workflows to stop first. Observation and export can continue while
those workflows run.

Configured `vision.perception.calibration.reference_height_m` permits automatic
matching to that known horizontal plane. With its default `null`, selecting a
reference explicitly is required. An inclined selected plane retains its actual
normal; the observer does not flatten it into world gravity.

## Measurement and timing

Use the original registered depth, its scale and matching intrinsics. PCN points,
UI image rotations, preview overlays and rendered reference geometry are excluded
from the measurement. Invalid/too-near/too-far depth pixels are dropped, at most
4,096 spatially distributed pixels are deprojected, and bounded RANSAC plus
weighted covariance fitting extracts up to four planes. Support area, two-axis
extent, fit residual, plausible orientation/height and temporal association gate
the candidates. Thresholds in `FloorSettings` are initial engineering settings,
not measured camera specifications.

A confirmed plane remains fixed. Its normal and offset are transformed into each
capture's optical frame and compared with the measured normal and offset. The
camera correction uses only minimum tilt and translation normal to the plane;
in-plane translation and rotation about the normal remain unobserved. Nearby
parallel planes make an association ambiguous rather than forcing a match.

Decoded RGB-D retains its existing source ID, boot ID, capture timestamp and
calibration ID. Camera intrinsics and depth scale are included in the local
calibration identity. Robot `measurement_time_s` expresses the arm sample in the
same source wall clock as RGB-D; its original monotonic freshness stamp remains
unchanged. Pilot buffers up to 256 raw measured states and interpolates bracketing
camera poses at capture time, with no extrapolation and no gaps above 150 ms.
GO2-mounted arms require measured body position/orientation with a compatible
sample time; spawn poses are not substituted for absent Robot measurements.

Vendor epoch image timestamps are retained when available. SDK/device-clock
timestamps that cannot be related to the source wall clock currently use host
acquisition time. That fallback does not establish exposure synchronization.
`clock_verified` is therefore false by default for telemetry-derived poses;
raw observations remain available, but their correction cannot be applied.
Verify camera acquisition delay, arm/body sample ages and timing uncertainty at
the required motion speed before changing this setting. An embedded camera pose
is coherent with its exact frame, as in the Sim visual replica; this validates
the geometry path, not the physical arm's calibration.

No capture older than 500 ms, future-dated by more than 30 ms, duplicated or
out of order supplies a correction. Correction reuse is further limited to
200 ms and camera motion within 2 cm / 3 degrees of the measured pose. These
are bounded reuse limits, not an accuracy guarantee.

## Offline fixed-mount fit

On a machine/environment with the Pilot package and its declared scientific
dependencies installed, run:

```bash
python3 -m elesim_pilot.vision.hand_eye_calibration observations.jsonl --output candidate.json
```

The command fits one fixed camera mount and, when necessary, a fixed unknown
reference plane. It reserves every fifth observation for validation, refuses
mixed source epochs/mounts/reference planes, and rejects a rank-deficient fit.
Motion must excite the parameters; repeated copies of one pose are insufficient.
Unverified timing is rejected unless explicitly allowed for exploratory analysis
with `--allow-unverified-clock`. The output is created exclusively and is never
installed automatically. Replacing a physical camera mount requires separate
review of the candidate and its independent validation.

Logs retain paired measured q/body states and available raw IMU/model diagnostic
metadata for later comparisons. Fixed mount fitting assumes the supplied parent
kinematics are trustworthy; pose-dependent arm deformation is not fitted at the
same time. Remaining IMU-model, plane-only and combined evaluations need real
recordings under comparable poses and loads.

## Acceptance boundary

Software tests cover metric projection, robust multi-plane fitting, degenerate
point sets, floor/table ambiguity, identity/reset/staleness fencing, capture-time
pose interpolation, unobservable directions, preview/source isolation, offline
rank checks and held-out fitting. A low residual on the very plane used to make
a correction is not an independent accuracy result. Physical acceptance needs
independent height/angle or reference-point measurements, poses not used for
calibration, false-floor selection and loss rates, and timing/performance evidence
on the intended ZED Mini or D435. Absolute 6-DOF accuracy is not established by
floor consistency alone.
