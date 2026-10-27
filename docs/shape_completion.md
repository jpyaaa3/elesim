# AdaPoinTr adoption trial

The isolated inference path works on remote GPU 2, but the original PCN-trained
checkpoint did not pass the metric-scale box completion criteria. This is a
research trial, separate from the live Pilot workflow, motor commands, IMU model
and plane-calibration application. No production configuration or dependency was
changed. Completion results are not automatically consumed by Pick/Grasp or Sim.

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
