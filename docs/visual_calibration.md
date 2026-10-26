# Plane-based visual tilt calibration

Pilot observes fixed planes in the original RGB-D optical frame, compares them
with the camera pose predicted by arm/body measurements, and optionally corrects
the camera's tilt. The camera origin remains the FK estimate. Robot owns actuation and safety. This
observer never changes motor commands, the selected IMU model, or a hand-eye file.

## Operator workflow

1. Start perception with the normal Pilot RGB-D input. Observation defaults to
   six updates per second; it does not require a detected target object.
2. Open **Visual Calibration**. The perception preview outlines eligible planes
   in yellow and labels them with the same IDs as the candidate dropdown.
3. Initially keep the body and arm stationary. Select a surface known to be
   horizontal and press **Confirm horizontal reference** after three consecutive
   accepted frames. A floor, tabletop or broad platform supplies the same
   direction constraint regardless of its height. Confirmation asserts world-up;
   it does not preserve an FK-biased initial normal as the reference.
4. Inspect tilt error in degrees. Translation and rotation about the reference
   normal remain unobserved; this is not full four-joint or camera-position
   identification. A lone inclined surface cannot establish horizontal without
   an independent reference. Do not confirm an unknown slope as horizontal.
5. Repeat at different stationary arm poses. Keep the reference fixed. Use
   **Export floor observations** to save up to the last 2,000 records as JSONL
   in a private `/tmp/elesim-floor-calibration-*` directory in the Pilot runtime.
   The UI reports the exact path on that host/container. Exports contain no
   camera image, credentials or executable model.
6. Enable **Apply visual tilt correction** only after validating the source timing
   and reference. Application is off by default. The switch changes perception
   world coordinates consumed by Pick/Gaze, so future planned motion can change;
   the observer itself sends no motor commands. It is available only for a valid,
   synchronized observation. Stop/reset, source-boot or camera-calibration changes
   invalidate the old observation/reference as appropriate.

Reference selection/reset and calibration configuration changes require active
IK/Pick/Gaze workflows to stop first. Observation and export can continue while
those workflows run.

With `vision.perception.calibration.reference_height_m: null` (the default), an
explicit reference confirmation is required. This legacy field remains readable:
a non-null value declares a horizontal reference for automatic direction
matching, but its numeric height does not gate or translate the live correction.
Historical metric-height experiments must explicitly request `correct_distance`.
The lower-level estimator also accepts an independently known inclined reference
normal; it preserves that declared direction rather than flattening it.

## Measurement and timing

Use the original registered depth, its scale and matching intrinsics. PCN points,
UI image rotations, preview overlays and rendered reference geometry are excluded
from the measurement. Invalid/too-near/too-far depth pixels are dropped, at most
4,096 spatially distributed pixels are deprojected, and bounded RANSAC plus
weighted covariance fitting extracts up to four planes. Support area, two-axis
extent, fit residual, plausible orientation/height and temporal association gate
the candidates. Thresholds in `FloorSettings` are initial engineering settings,
not measured camera specifications.

A confirmed reference direction remains fixed. Its normal is transformed into
each capture's optical frame and compared with the measured normal. Correction
uses the minimum tilt rotation and never relocates the camera. All translation
and rotation about the normal remain unobserved. Supported parallel planes at
different levels supply one consistent direction. Competing supported normals
that differ by more than the 3-degree consensus bound reject correction.
Plane offsets remain diagnostic measurements, not a demand to map a surface to
world Z=0. Minimum support, residual, coarse direction and temporal gates remain.

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

## Historical offline fixed-mount fit

On a machine/environment with the Pilot package and its declared scientific
dependencies installed, run:

```bash
python3 -m elesim_pilot.vision.hand_eye_calibration observations.jsonl --output candidate.json
```

This is outside the current imperfect-joint tilt objective. The command fits one fixed camera mount and, when necessary, a fixed unknown
reference plane. It reserves every fifth observation for validation, refuses
mixed source epochs/mounts/reference planes, and rejects a rank-deficient fit.
Motion must excite the parameters; repeated copies of one pose are insufficient.
Unverified timing is rejected unless explicitly allowed for exploratory analysis
with `--allow-unverified-clock`. The output is created exclusively and is never
installed automatically. Replacing a physical camera mount requires separate
review of the candidate and its independent validation.

Current orientation-only exports cannot establish a fixed metric plane and are
rejected by this legacy translation fitter. Use historical metric observations
only when reproducing that experiment; current operation keeps the mount fixed.

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
on the intended ZED Mini or D435. Current primary metrics are remaining tilt in
degrees, supported-pose coverage and cross-view object-normal consistency.
Unchanged camera-origin translation is a required invariant. Absolute 6-DOF
accuracy and individual joint biases are not established by plane direction.

## Imperfect-joint tilt experiment

`vision_arm_scene --joint-errors` independently samples reported four-DOF poses
and synthetic physical joint errors. The rendered arm uses actual q; sensor
packets contain reported q only. The nominal ZED mount is exact and unchanged.
Errors include up to 5 mm Linear, 5 degrees Roll and 1.5 degrees per Bend,
clipped to software limits, plus a bounded pose-dependent deflection fixture.
Trusted horizontal surfaces vary in level and can appear together. No source
truth or scene labels are provided to Pilot.

```bash
python3 -m workbench.research.experiments.vision_arm_scene data --joint-errors --backend genesis \
  --poses 24 --error-seeds 2 --data-root payload/data --config payload/config/sim/config.yaml
python3 -m workbench.research.experiments.vision_reconstruct data/sensors predictions \
  --arm-model payload/data/models/arm/default.json \
  --hand-eye payload/data/calibration/cameras/zed_mini.hand_eye.json
python3 -m workbench.research.experiments.vision_tilt_evaluate data/truth predictions --output evaluation.json
```

The independent evaluator checks true camera tilt and visible object top-face
normals, plus agreement between different views of the same object's normal.
It never performs ICP or adds a translation correction. Position/yaw error can
remain even when the world looks upright; that is reported as an unobserved
direction, not hidden by a height anchor. Static simulation and synthetic noise
do not validate physical load-dependent arm deformation or exposure timing.

## Historical sensor-only partial-cloud experiment

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
python3 -m workbench.research.experiments.vision_reconstruct data/sensors predictions --legacy-height
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

## Historical four-DOF extrinsic-error matrix

`vision_arm_scene.py` extends the Cartesian fixture with the installed arm URDF,
the ZED hand-eye transform and all four joint coordinates: Linear, Roll, Bend 1
and Bend 2. A Latin hypercube spans the declared joint limits without filtering
poses by floor or object visibility. Each pose is crossed with independently
seeded errors and repeated for three RGB-D captures to exercise stabilization.
The bench arm base is explicitly fixed at Z=0.8 m. Genesis poses the actual arm
mesh and checks its node9 transform against independent URDF FK; Pilot uses its
own installed arm model. This remains static geometry, with ideal synchronized
encoders, rather than physical actuation or a moving GO2 acceptance test.

The eight conditions are clean depth, world tilt/height error, local camera-mount
SE(3) error, noisy/incomplete depth, a high plane hiding the floor, a low plane
hiding it, a finite plane with visible floor, and total depth loss. The consumer
receives only sensor packets and the pre-existing nominal arm/hand-eye model.
It cannot read the generated condition labels or true camera/object geometry.
Absent targets and unsupported floors remain in the denominator. The evaluator
counts false plane selections separately from point accuracy; an experiment may
complete while its overall acceptance result is false.

```bash
python3 -m workbench.research.experiments.vision_arm_scene data --backend genesis --poses 24 --error-seeds 2 \
  --data-root payload/data --config payload/config/sim/config.yaml
python3 -m workbench.research.experiments.vision_reconstruct data/sensors predictions \
  --legacy-height \
  --arm-model payload/data/models/arm/default.json \
  --hand-eye payload/data/calibration/cameras/zed_mini.hand_eye.json
python3 -m workbench.research.experiments.vision_arm_evaluate data/truth predictions --output evaluation.json
```

`--report-only` on the evaluator preserves a failed acceptance report with a zero
process exit status; `accepted` in that report remains false. A nearby tabletop
and a camera height error can supply identical plane observations. The configured
height's matching tolerance does not establish the physical identity of a lone
plane. Likewise, local mount error rotates with Roll/Bend and can leave unknown
XY/yaw components after per-frame floor correction. Neither case permits a
universal millimetre accuracy guarantee over arbitrary arm poses.

The GPU 2 run processed all 384 pose/error cases (1,152 frames). Height/tilt point
p95 fell from 113.29 to 2.10 mm, and noisy/incomplete depth from 100.53 to 4.09 mm,
where the object and supported floor were both available. General local mount
SE(3) error retained 56.87 mm p95. Only 32/48 clean cases supplied a supported
floor, and 10/48 also had object points. A low plane hiding the floor was wrongly
accepted in 32/48 cases, worsening point p95 from 1.13 to 106.96 mm. Overall
acceptance failed; these outcomes are not a guarantee over the whole joint space.
The [curated record](../workbench/evidence/curated/experiments/vision_arm_matrix_20261026.json)
and [pose matrix](../workbench/evidence/curated/experiments/vision_arm_matrix_20261026.png)
retain availability, rejected cases and false plane selections.

An exploratory analysis fits one fixed mount per source boot from the blind
consumer's supported-plane observations. Every pose containing object points is
held out of fitting; the fit never reads condition labels, objects or camera
truth. An independent evaluator then measures those object points without
registration. The local-mount condition improved from 56.87 to 2.66 mm p95 over
11 held-out object observations. This supports investigating fixed-mount
calibration under trusted FK, a trusted bench base and a genuine known floor.
It does not establish physical accuracy or an automatic model selector.

```bash
python3 -m workbench.research.analysis.vision_arm_mount_fit predictions/result.json \
  payload/data/calibration/cameras/zed_mini.hand_eye.json --output candidates.json
python3 -m workbench.research.analysis.vision_arm_mount_evaluate candidates.json \
  object-validation --output object-validation.json
```

`object-validation/` is a private evaluator bundle of held-out predicted points,
corresponding ideal depth and true poses. It is not an input to fitting. The
world-tilt/height and noisy conditions did not fit the same fixed-mount model:
their held-out point errors remained above 100 mm. The low-plane case also had
large training plane-distance RMS (87.9 and 102.3 mm), providing a model
consistency diagnostic rather than a demonstrated rejection policy. Candidates
are not applied to the runtime or installed hand-eye files.
