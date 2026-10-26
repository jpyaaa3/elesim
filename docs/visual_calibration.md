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

## Sensor-only partial-cloud experiment

The next perception stage preserves observed object surface points; it does not
complete an unseen shape. `vision.partial_cloud` accepts a strict schema-1 local
NPZ replay packet containing RGB-D, intrinsics, source identity/time and encoder
readings. Camera-world truth, object poses, asset IDs, silhouettes and meshes are
rejected. This file format adds no DDS message or runtime control command.
Points are deprojected through the original pixels and transformed using the
declared encoder model plus an optional floor correction. Bounded voxel means
combine views without ICP or any other alignment that could hide pose errors.

Three independent research processes implement the baseline:

```bash
python3 -m workbench.research.experiments.vision_scene data --backend genesis --seeds 3
python3 -m workbench.research.experiments.vision_reconstruct data/sensors predictions
python3 -m workbench.research.experiments.vision_evaluate data/truth predictions --output evaluation.json
```

The producer randomizes a box and hidden camera mount error, writing sensor and
private truth directories separately. The consumer receives only its script,
sensor directory and output directory. A declared virtual Cartesian encoder rig
supplies nominal camera extrinsics; no true Sim camera pose or object catalog is
available. The example detector uses image color, not the renderer segmentation
buffer. A known horizontal floor at world Z=0 is an explicit reference assumption.
This test does not exercise real arm FK, exposure synchronization or GO2 motion.

Only the evaluator reads private truth. It measures corresponding visible object
points in fixed world coordinates, without alignment, and their distance to the
true object surface. Correction must reduce p95 point error by at least 70%,
below 12 mm, and keep surface p95 below 12 mm. A separate XY/yaw perturbation
must retain over 25 mm p95 error: a plane cannot resolve those directions.
The analytic CPU backend is a deterministic regression fixture; it is not
evidence of Genesis rendering or physical accuracy. PCN/completion, arbitrary
object segmentation, live workflow wiring and grasp/contact validation remain
separate stages.

The 2026-10-26 acceptance run used fresh `cc030b1` release images on remote GPU 2
(RTX A6000) after updating both installations. Three boxes produced 48 Genesis
RGB-D frames; 36 passed the three-frame stability gate. Observable point p95
fell from 86.46 to 0.99 mm and surface p95 was 0.97 mm. The XY/yaw control stayed
at 70.59 mm. All three processes had networking disabled; actual container mounts
confirmed the consumer received only its script, sensor directory and output.
The existing test services kept their exact container/image IDs and running
state. No diagnostic container remained after the run. These figures use ideal
virtual encoders, a known floor and injected 0.5 mm depth noise; they are not
physical-camera accuracy measurements.

See the [curated record](../workbench/evidence/curated/experiments/vision_partial_cloud_20261026.json)
for image IDs, module/script SHA256, mount audits, thresholds and bundle hashes.
The original bundle, representative sensor frames and `partial-cloud.ply` are
under `workbench/evidence/generated/vision-baseline-20261026/`. To plot a collected
bundle in an environment with NumPy and Matplotlib already installed, use:

```bash
python3 -m workbench.research.analysis.vision_partial_cloud_plot collected baseline.png
```

Sim RGB-D intrinsics now follow the renderer's vertical FOV and integer pixel
centre convention, verified against the actual camera during this run. The
[pinned Genesis camera implementation](https://raw.githubusercontent.com/Genesis-Embodied-AI/genesis-world/v1.4.1/genesis/vis/camera.py)
defines both the vertical focal length and half-pixel raster sampling.
