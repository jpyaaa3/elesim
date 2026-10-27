# AdaPoinTr adoption trial

The isolated inference path works on remote GPU 2, but our pipeline using the
PCN-trained checkpoint did not pass the metric-scale box completion criteria.
This is a research trial, separate from the live Pilot workflow, motor commands, IMU model
and plane-calibration application. No production configuration or dependency was
changed. Completion results are not automatically consumed by Pick/Grasp or Sim.

For current runs use the `completion-adapointr`/`completion-replay` recipes in
[`benchmarks.md`](benchmarks.md), plus the maintained input/GPU smoke helpers in
the [Workbench tool list](../workbench/README.md). Old ablation and trial-specific
figure CLIs are kept in the ignored local [archive](../workbench/archive/README.md).

## Pinned assets and environment

- Official source: <https://github.com/yuxumin/PoinTr>, revision
  `4603257ed3db9e7dad349b712e1b2fe0da207015`.
- Source archive SHA256:
  `ab52d68e84bf79e7d7969a826c93672c2ff4d997b714a571b722bcd5f4be668e`.
- The official README's AdaPoinTr PCN checkpoint, SHA256:
  `f58a5650b348ca3d68fc8e01e3e88f57d5a889bbad5521fec7a2760eda664fa1`.
  The file is 389,745,620 bytes and includes training state. The loader uses
  `weights_only=True` and strict parameter loading. We did not train or fine-tune.
- Config: `cfgs/PCN_models/AdaPoinTr.yaml`, 2,048 input points, 16,384 output points,
  graph encoder, 32,494,657 parameters.
- Existing install-owned Sim image supplies the scientific stack for these
  repository-only workers: Torch 2.12.1+cu130, Python 3.10 and Triton. No Sim
  implementation is imported by the inference worker. This is not proof of a
  deployed Pilot dependency set or live perception integration.
- Pure Python `einops==0.8.1` and `easydict==1.13` are unpacked into the trial's
  private dependency directory. Existing image/host packages are untouched.
- Exactly GPU `GPU-a40782b2-881c-e285-2a8e-66cae9162a01` (host index 2, A6000) is
  reserved for model workers. Data preparation, evaluation and plotting have no
  GPU reservation. Every computation container has networking disabled.

The pre-existing image has no nvcc or PointNet++ extension. The isolated backend
loads only AdaPoinTr's upstream modules and supplies the required FPS using
Triton and gather using Torch. FPS follows the published PointNet2_PyTorch
algorithm: initial index zero, near-zero exclusion and 512-lane tie priority.
Eight GPU cases exactly match the independent CPU implementation; gather also
matches. This is not direct binary equivalence testing against a built upstream
CUDA extension. Native Torch supplies DropPath and truncated normal; initialization
is replaced by the strict checkpoint load. Training loss is unavailable.

The loader installs temporary `models`/`utils` import namespaces. Run it only in
its dedicated worker process; importing it into the live Pilot process is not
part of this experiment.

## Sensor-only path

1. Read the existing RGB-D replay and nominal-FK/plane-corrected point records.
   Split red image connected components and retain components with at least 128
   observed points. This fixture segmentation is not a general object detector.
2. Normalize using the observed centroid and maximum observed radius. Default
   normalized radius is .5; radius 1.0 is a sensitivity control. No true object
   center, size or yaw is supplied.
3. Order world Z-up coordinates as PCN `(x,z,y)`, matching the published
   `NormalizeObjectPose` KITTI-to-PCN axis ordering. Object heading is unobserved.
   Deterministically sample/repeat existing points to 2,048; zero geometry is not
   invented for padding.
4. Run the unchanged upstream forward with strict pretrained weights, batch one,
   FP32, TF32 disabled. Invert the declared axes/scale/centroid transforms.
5. Store `observed` and `predicted` separately. Observed coordinates remain exact.
   Only the independent evaluator receives private box geometry. No ICP or truth
   alignment is performed.

The updater publishes runtime payloads and excludes `workbench/`. Remote
experiments therefore fetch only the required research files from the pushed Git
revision and verify their SHA256 values before mounting those files read-only.
Do not expose the source renderer or private truth directory to inference.

## Reproduction entry points

Use the existing install's generated Compose wrapper and selected role image for
the bounded one-off experiment, with the exact mounts/GPU reservation recorded in
the evidence. The following module commands run *inside that scientific worker*,
where the pinned upstream repository, checkpoint and private Python dependencies
have already been staged. They are not host dependency installation commands.

```bash
python3 -m workbench.research.experiments.adapointr_gpu_smoke \
  --vendor /vendor --config /vendor/cfgs/PCN_models/AdaPoinTr.yaml \
  --checkpoint /checkpoint.pth \
  --checkpoint-sha256 f58a5650b348ca3d68fc8e01e3e88f57d5a889bbad5521fec7a2760eda664fa1 \
  --output /output

python3 -m workbench.research.experiments.adapointr_inputs \
  /sensors /recorded-predictions /output/inputs --captures 48

python3 -m workbench.research.experiments.adapointr_infer \
  /inputs /output/predictions \
  --vendor /vendor --config /vendor/cfgs/PCN_models/AdaPoinTr.yaml \
  --checkpoint /checkpoint.pth \
  --checkpoint-sha256 f58a5650b348ca3d68fc8e01e3e88f57d5a889bbad5521fec7a2760eda664fa1 \
  --radius .5 --axis-order world-z-up

python3 -m workbench.research.experiments.adapointr_evaluate \
  /truth /predictions --output /output/evaluation.json
```

The radius 1.0 and `--axis-order model-native` controls use exactly the same
observations. Capture IDs are selected before viewing completion quality. Keep
the separate input/inference/evaluation mounts; a combined evidence bundle also
contains private truth and must not be mounted wholesale into inference.

## Results and limits

The final run used research revision `48fd68f`. Forty-eight requested captures
(24 arm poses, two seeds) produced 20 object observations from ten captures and
eight distinct boxes. Unsupported/no-eligible-object captures remain counted.
This is static 320×240 Genesis RGB-D replay with known nominal mount and the
existing plane correction; it does not exercise real sensor timing or the
combined IMU/plane control path.

| Median per-object metric | Native axes, radius .5 | PCN axes, radius .5 | PCN axes, radius 1.0 |
| --- | ---: | ---: | ---: |
| Surface p95, mm | 31.89 | 27.10 | 28.26 |
| Observed-to-prediction p95, mm | 12.33 | 11.84 | 37.66 |
| F1 at 10 mm | .362 | .432 | .130 |
| Hidden-surface recall at 10 mm | .021 | .065 | .000 |
| Forward wall time, ms | 13.35 | 13.04 | 13.22 |

Observed-only F1 is .595. The median observed surface p95 is 1.06 mm. Metrics
describe the raw learned output; separately preserved observations are not
counted as model predictions to inflate fidelity. They do not evaluate an
observed-plus-predicted fusion or later collision/visibility filtering policy.
The initial geometry criteria were fixed before evaluating this replay: median
surface p95 <=20 mm, observation-to-prediction p95 <=10 mm, precision >=.8,
hidden-surface recall >=.5 and F1 improvement over the observed-only baseline.
All three variants fail geometry. Forward latency passes the initial 100 ms
standalone target. A 13 ms inference result is not a 100 Hz Sim-step guarantee;
Sim/MPPI co-execution and live end-to-end latency remain unmeasured.

Public airplane/sofa demo inference succeeded with identical repeated output,
about 14 ms warm forward time and 171 MiB PyTorch peak allocated memory. The
first forward spent 1.27 seconds including kernel preparation. This excludes
driver/context memory and gives no numeric accuracy result without demo truth.

Only the PCN-trained checkpoint was available: the official AdaPoinTr
ShapeNet-55 link returned an HTML page rather than a checkpoint. The experiment
does not establish whether different weights, training data, rotation handling
or a different partial-frame normalization would solve the quality gap.
Broader checkpoint/domain evaluation or fine-tuning is required before using
generated geometry for grasp planning. No fixed mount or real IMU model was fit.

Host regression: nine new focused tests, 13 passed including related replay
tests; GPU operators: eight FPS cases plus gather. Canonical required/extended,
real-RMW and isolated-release verification were not run because the known
development attachment's initial image preparation remains incomplete. No new
development image or host scientific dependencies were installed.

[Metrics/provenance](../workbench/evidence/curated/experiments/adapointr_20261027.json),
[observed/predicted/truth](../workbench/evidence/curated/experiments/adapointr_objects_20261027.png),
[metric comparison](../workbench/evidence/curated/experiments/adapointr_metrics_20261027.png),
[public demo sanity check](../workbench/evidence/curated/experiments/adapointr_demos_20261027.png).

## Same-sample preprocessing control, 2026-10-27

Research revision `56a717b83cac4c3eb9e652c96c65ed99716867ed` was pushed and
installed through the official remote updater before inference. The same
checkpoint, upstream forward, FP32 backend and sole remote GPU 2 were reused.
Five transforms crossed with five sampling seeds and two public demo clouds
produced **50 inferences**. For each seed, every transform receives the same
physical input point indices and order, checked by inverse-transforming the
model input. All variants keep the original PCN Y-up axes. Outputs are restored
to the original demo coordinates before measurement; no ICP or truth alignment
is used. This controls our affine preprocessing, including any resulting
interaction with the model's internal sampling, rather than changing weights.

| Input transform | Airplane fidelity p95, % diagonal | Sofa fidelity p95, % diagonal |
| --- | ---: | ---: |
| Original demo coordinates | 0.448 | 0.665 |
| Subtract partial centroid only | 0.507 | 5.020 |
| Scale only to centered observed radius .5 | 8.936 | 0.682 |
| Partial centroid + radius .5 (our default) | 6.785 | 0.737 |
| Partial centroid + radius 1.0 | 20.753 | 14.550 |

Fidelity is the distance from each **original observed point** to its nearest
predicted point; p95 is divided by the original observed AABB diagonal. Values
are medians over five seeds. This is not the full-surface metric from the box
trial, and the percentages are not millimeters. The public demos have no complete
ground truth, so neither absolute completion accuracy nor correctness of unseen
geometry can be claimed from these numbers.

The same-seed p95 ratio for our default transform is **15.23×** native for the
airplane and **1.09×** for the sofa. Centering alone gives **7.51×** on the sofa;
scaling alone gives **19.93×** on the airplane. The large effects appear in all
five seeds. Our predeclared strong-degradation criterion was a paired median
ratio above 1.25 and a worse result in at least 80% of seeds. It is met by airplane
scale-only and both center+scale variants, and by sofa center-only and radius
1.0. The sofa default does not meet that criterion: the effects of centering and
scaling cannot be assumed to add independently.

Default-transform output changes against same-seed native outputs are 2.044%
and 1.047% diagonal (symmetric mean nearest-neighbor distance). Native output
variation from resampling alone is 0.259% and 0.410%, measured against seed zero
over the other four seeds. These quantify output sensitivity, not accuracy.
Counts near the FPS zero-exclusion threshold also change, but that interaction
was not separately isolated from coordinate/scale distribution sensitivity.

**Finding:** our partial-centroid/radius preprocessing can itself damage observed
surface fidelity on the public inputs. The earlier box failure measures the
combined checkpoint **and preprocessing** pipeline; attributing it solely to poor
weights or to simulated objects is unsupported. This control does not prove that
preprocessing explains the entire box failure, nor that these two classes
represent general objects. Removing normalization from raw world-meter inputs
would not reproduce the demo's original canonical frame. A replacement must
first define how canonical center, scale and heading are obtained without full
object truth, then pass the existing independent box evaluator. No production
preprocessing or automatic grasp/collision behavior was changed.

The coordinate and metric TDD passed seven new tests, **16 passed** with the
related tests. GPU inference exited zero; normal test Sim/Pilot IDs, images and
stopped state were preserved, and no diagnostic containers remain. Plotting was
CPU-only with no network or GPU reservation. Canonical required/extended,
real-RMW and isolated-release gates remain unrun for the development-attachment
reason stated above.

The historical `adapointr_preprocessing.py` ablation runner and
`adapointr_preprocessing_plot.py` figure command are archived with their original
arguments. The unchanged `prepare_control`/`comparison_metrics` calculations
remain in `workbench/tests/research/support/completion_controls.py`, with all
existing round-trip and fidelity assertions.

[Metrics/provenance](../workbench/evidence/curated/experiments/adapointr_preprocessing_20261027.json),
[fidelity and output sensitivity](../workbench/evidence/curated/experiments/adapointr_preprocessing_metrics_20261027.png),
[same-frame output comparison](../workbench/evidence/curated/experiments/adapointr_preprocessing_clouds_20261027.png).

## AP-PS55 checkpoint/frame replay, 2026-10-27

The proposed original **cropped ShapeNet-55** checkpoint could not be obtained.
Its README URL and the URL recommended by the author in issues
[#88](https://github.com/yuxumin/PoinTr/issues/88) and
[#132](https://github.com/yuxumin/PoinTr/issues/132) resolve to the same expired
share. The author also states in #132 that the best checkpoint was lost. This
trial therefore does **not** validate a recovered original or paper-best S55
checkpoint with the official radius-1 inference path.

An available alternative is the **third-party inference-only AP-PS55** release
from [SGA-GSN](https://huggingface.co/robotic-vt-grasp-project/SGA-GSN/tree/133836b26c3af5806e2594882c2d445f0e0bda24).
The 130,024,020-byte checkpoint has published LFS SHA256
`66e22112aa43e74cd0716012201ea40bda0492e8163897e6e45102e86cb781b7`,
verified after download. Its metadata labels it AP-PS55, epoch 512, with optimizer
state omitted; it records source-checkpoint SHA256
`69c9b32c6b61252f1f9d95edefcf59005de97c2a83caec6a72dc1408d7d69827`.
This is useful provenance from the redistributor, not proof that it is identical
to the unavailable original author's file. The checkpoint is retained only in
the remote research asset directory and is not added to Git or runtime images.

Use the upstream **Projected_ShapeNet55** config: 8,192 output points and grouper
centers `[256,128]`. Cropped ShapeNet55 also has 8,192 output points but uses
`[512,256]` and a different dataset. Merely changing the output count or pairing
the cropped config with this file would not establish the correct model bundle.
Strict loading of AP-PS55 into the Projected config succeeds with no missing or
unexpected parameters. No training or fine-tuning was performed.

The official generic inference script adds unit-radius normalization only for
the `ShapeNet` dataset; `Projected_ShapeNet` training/test transforms do not
recenter/rescale the already prepared dataset coordinates. That does not justify
passing arbitrary world-meter coordinates unchanged. The separate public
SGA-GSN usage subtracts the observed visual/contact centroid while retaining
metric size and world axes. This motivates a **center-only Z-up hypothesis** for
our vision-only input, not a claim to reproduce the full pretrained training
distribution or the tactile pipeline.

Research revision `4664db276a881629f9f9f513cf8774ad3fc16025` was pushed and both
official updaters completed. Required research files were downloaded from that
revision with SHA256 verification. The previous scientific image and sole GPU 2
were retained. The same 48 requested captures, 20 observed objects from ten
captures and eight boxes were used; input IDs, point hashes, sampling seed zero,
inverse world-frame evaluation and acceptance thresholds were fixed. All
observed points remain separate and exact. Inference cannot read private truth;
only the network/GPU-free evaluator and later plots receive it.

The matrix crosses AP-PS55 center-only/radius .5/radius 1.0 with Y-up/Z-up axes,
plus the unchanged PCN radius-.5 Y-up baseline: **140 inferred observations**.
Before seeing its results, center-only Z-up was declared the primary candidate
from the published usage above. All six AP-PS55 treatments remain reported.
The deterministic input sampler is our existing random/repetition policy, not
the upstream `UpSamplePoints` implementation. PCN and PS55 differ in checkpoint,
output density and grouper config, so this is not a weights-only ablation.

| Median per-observation metric | PCN .5, Y-up | AP-PS55 center, Y-up | AP-PS55 center, Z-up (primary) | AP-PS55 .5, Y-up | AP-PS55 .5, Z-up | AP-PS55 1.0, Y-up | AP-PS55 1.0, Z-up |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Predicted surface p95, mm | 27.10 | 13.74 | **13.27** | 13.60 | 13.93 | 39.78 | 38.93 |
| Observation-to-output p95, mm | 11.84 | 4.34 | **4.62** | 2.50 | 2.71 | 16.93 | 12.51 |
| Precision at 10 mm | .823 | .920 | **.922** | .912 | .891 | .405 | .458 |
| F1 at 10 mm | .432 | .841 | **.865** | .802 | .778 | .286 | .309 |
| Hidden-surface recall at 10 mm | .065 | .601 | **.695** | .507 | .473 | .000 | .0004 |
| Warm standalone forward, ms | 13.29 | 13.68 | **13.64** | 13.10 | 13.86 | 13.58 | 14.02 |
| Existing median geometry criteria | FAIL | PASS | **PASS** | PASS | FAIL | FAIL | FAIL |

The observed-only F1 baseline remains **.595** and observed surface p95 **1.06
mm**. Here hidden recall means truth samples more than 10 mm from any observed
point; it is an unobserved-surface proxy, not a ray-visibility classification.
All timings exclude acquisition/segmentation and live Sim/MPPI contention;
the initial remote inventory showed other GPU-2 activity. They are not exclusive
hardware performance or end-to-end latency measurements.

The PCN baseline reproduces the previous geometry values. AP-PS55 already
improves that baseline with the same radius-.5 transform, so improvements cannot
be assigned solely to removing normalization. Center-only improves the primary
candidate's hidden recall and F1 further. Radius 1.0 degrades AP-PS55 sharply:
normalization used by a different checkpoint family is not a general remedy.

Post-trial robustness reruns keep the predeclared primary and vary only sampling
seed to 1 and 2, adding **40 observations**, with no changed acceptance criteria.
All three seed aggregates pass:

| Sampling seed | Surface p95, mm | Observation p95, mm | F1 at 10 mm | Hidden recall at 10 mm |
| --- | ---: | ---: | ---: | ---: |
| 0 | 13.27 | 4.62 | .865 | .695 |
| 1 | 12.71 | 4.75 | .865 | .693 |
| 2 | 12.96 | 4.72 | .866 | .701 |

This is **median acceptance on the existing box replay**, not universal
completion acceptance. At seed zero, all 20 observations improve F1 over their
own observed-only baseline, but only **12/20** meet all five geometry conditions
individually. The worst surface p95 is **24.17 mm**; one box seen twice has hidden
recall around .05. Repeated seeds and repeated views do not create new object
generalization evidence. Unseen shapes, sizes, sensor noise, timing, object
segmentation and live calibration integration remain untested. No production
adapter, grasp collision shape or motion path was enabled.

Seven new config/frame tests and the previous related tests give **23 host
passes**. Inference and evaluation stages all exited zero, normal test Sim/Pilot
IDs/images/stopped states stayed unchanged, and diagnostic containers were
removed. CPU-only figures and the archived output/metadata have verified SHA256
manifests. Canonical required/extended, real-RMW and isolated-release gates remain
unrun for the already documented incomplete development-attachment preparation.

Reproduce inside the existing scientific worker with explicit frame choices:

```bash
python3 -m workbench.research.experiments.adapointr_infer \
  /inputs /output/predictions \
  --vendor /vendor --config /vendor/cfgs/Projected_ShapeNet55_models/AdaPoinTr.yaml \
  --checkpoint /checkpoint.pth \
  --checkpoint-sha256 66e22112aa43e74cd0716012201ea40bda0492e8163897e6e45102e86cb781b7 \
  --profile projected55 --normalization center-only --axis-order model-native --seed 0
```

Here `model-native` means no axis permutation: the input and inverse output
remain in our world Z-up frame. It does not certify the checkpoint's native
dataset orientation. Using `world-z-up` applies the explicit `(x,z,y)` swap and
tests the Y-up alternative.

[Metrics/provenance](../workbench/evidence/curated/experiments/adapointr_profiles_20261027.json),
[all seven conditions](../workbench/evidence/curated/experiments/adapointr_profiles_metrics_20261027.png),
[observed/baseline/primary/truth](../workbench/evidence/curated/experiments/adapointr_profiles_clouds_20261027.png).
