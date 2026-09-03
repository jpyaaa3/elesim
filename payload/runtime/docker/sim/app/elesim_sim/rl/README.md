# Wrap-grasp reinforcement learning

Trains a 4-DoF continuum arm on a GO2 quadruped to wrap and lift an upright
cylinder, in Genesis, with PPO from `rsl_rl`.

## What this needs, and what it does not

**No ROS.** The RL stack imports `elesim_sim.config` and `elesim_protocol.messages`,
both of which are plain Python — `elesim_protocol` keeps its ROS imports lazy.
Verified: importing `elesim_sim.rl.train` pulls in no `rclpy`, no
`elesim_interfaces`. A training box needs none of the DDS runtime.

**No Docker.** `elesim-up` and the container images are for the distributed
application. Training runs directly against the Python packages.

## Ubuntu + CUDA setup

```bash
git clone https://github.com/jpyaaa3/elesim.git
cd elesim
git checkout wrap-grasp-rl

conda create -n elesim-rl -y python=3.11
conda activate elesim-rl

# numpy<2 is required: elesim-protocol pins it, and rsl-rl-lib pulls numpy 2
# unless told otherwise.
python -m pip install -r payload/runtime/docker/sim/requirements.lock
python -m pip install rsl-rl-lib tensorboard "numpy<2"
python -m pip install --no-deps -e payload/runtime/common/protocol -e payload/runtime/docker/sim/app
```

Check the backend resolves to CUDA before starting a long run:

```bash
python -c "
import elesim_sim.rl, torch
print('cuda', torch.cuda.is_available(), torch.version.cuda)
print('device', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'cpu')"
```

`runtime.backend: gpu` maps to CUDA on Linux and Metal on macOS, and
`runtime.torch_device: auto` prefers CUDA, then MPS, then CPU. Neither needs
changing between the two.

### Import order

`elesim_sim/rl/__init__.py` imports numpy before anything else. On conda +
pip installs there are several copies of `libomp`, and importing `torch` first
trips OpenMP's duplicate-runtime guard. Entry points import the package before
torch for that reason; keep it that way rather than reaching for
`KMP_DUPLICATE_LIB_OK`.

## Training

```bash
python -m elesim_sim.rl.train \
  --set runtime.n_envs=4096 \
  --set curriculum.stage=1 \
  --set train.save_interval=10 \
  --stamp run1
```

Runs land in `var/rl/sim/<experiment_name>/stage<N>_<stamp>/`. **Reusing a
`--stamp` writes into the same directory**, overwriting checkpoints and stacking
a second tensorboard event file onto the first; give each run its own.

```bash
tensorboard --logdir var/rl/sim/wrap_grasp --port 6006
pgrep -fl elesim_sim.rl.train        # confirm exactly one run is going
```

Resume from a checkpoint:

```bash
python -m elesim_sim.rl.train --resume var/rl/sim/wrap_grasp/stage1_run1/model_200.pt --stamp run2
```

### Environment count

Pick it against the failure mode, not the GPU's memory. The wrap scene's
ceiling is set by the constraint solver, and it is much lower with contact live
than without: a contact-free scene benchmarked fine at 4096 environments on an
M4, while 5324 failed on Jacobian size and contact-heavy rollouts diverged at
1024. Start lower than the memory would allow and raise it once a run survives
past the point where the policy starts pressing on the object.

### Curriculum

One switch; the stage sets the randomisation flags and the success criterion.

| stage | object pose | object radius | approach shaping |
|---|---|---|---|
| 1 | fixed | fixed | on |
| 2 | randomised | fixed | off |
| 3 | randomised | randomised | off |

```bash
python -m elesim_sim.rl.train --set curriculum.stage=2 --resume <stage-1 checkpoint> --stamp s2
```

Success is the lift test's retention check on every stage. The geometric gate
(`success.criterion: geometric`) exists and the spec allows it early, but it
makes a poor objective: it rewards a pose rather than a behaviour — a measured
pose reached 250 deg of wrap with zero object contacts — and at any threshold
at or below 180 deg it is provably not a grasp, because the escape opening is
then at least the object's own diameter.

## Running eval beside a training job

Pin each process to its own GPU with `CUDA_VISIBLE_DEVICES`.  Within a process
the selected device is renumbered to 0, and nothing here hardcodes a device
index, so both torch and Genesis follow it:

```bash
# GPU 0: training
CUDA_VISIBLE_DEVICES=0 python -m elesim_sim.rl.train \
  --set runtime.n_envs=4096 --stamp run1

# GPU 1: evaluation against a checkpoint the training run has written
CUDA_VISIBLE_DEVICES=1 PYOPENGL_PLATFORM=egl python -m elesim_sim.rl.eval \
  --checkpoint var/rl/sim/wrap_grasp/stage1_run1/model_200.pt \
  --set runtime.n_envs=512 \
  --episodes 20 --render 40 \
  --out var/rl/sim/eval/it200.md \
  --video-out var/rl/sim/eval/it200.mp4
```

Evaluate a *checkpoint file*, never the live run: the two processes share
nothing, and checkpoints are complete on write.

### Headless rendering

A training server has no display, so pyrender must go through EGL.
`rl/headless_gl.py` sets `PYOPENGL_PLATFORM=egl` automatically when there is no
`DISPLAY` and the platform is not macOS -- macOS has no EGL and forcing it there
breaks the import.  An existing value is always respected, so set
`PYOPENGL_PLATFORM=osmesa` instead if the driver has no EGL.

Rendering needs a GL-capable driver on the device.  If the eval GPU is in
compute-only mode, run eval with `--render 0` and render separately where a
driver is available.

## Evaluation and export

```bash
python -m elesim_sim.rl.eval --checkpoint <ckpt> --episodes 20 \
  --render 40 --render-every 4 --render-episodes 3
python -m elesim_sim.rl.export_traj --checkpoint <ckpt> --count 20
```

Eval conditions are **offsets** from the configured object centre, and the
support post moves with the object.  Absolute coordinates silently decouple
from `support.center_xy`: a grid of x = 0.24/0.28/0.32 against a support at
x = 0.5 hung the cylinder 0.18 m off its post, so it fell on the first macro
step and all 13824 episodes scored `topple` -- a result about gravity, not the
policy.  A whole column of one failure mode, with `phi_max` equal to
`phi_mean`, means every env terminated on step one; read that as a setup fault
rather than a finding.

Video frames are sampled per physics *substep*, not per macro step.  A macro
step covers `substeps * dt` of simulated time -- 0.4 s at the defaults -- so one
frame per macro step turns a 15-step episode into 15 frames, and the arm's
motion between waypoints is not in the recording at all.  `--render-every N`
sets the substep stride and the output fps defaults to real time for it;
`--render-episodes` keeps recording across resets so an early termination still
leaves something watchable.

`eval` reports success per condition with failures split five ways —
`collision`, `topple`, `retention`, `no_wrap`, `no_reach` — because a single
success rate cannot separate "the arm never arrived" from "it wrapped and then
dropped it", and those call for different fixes.

When collisions dominate, the report adds a second table splitting them by
body -- `support`, `go2`, `self`, `floor` -- because "collision" on its own
cannot say whether the arm is running into the object's stand, the quadruped it
is mounted on, or itself, and those need different fixes.

`export_traj` writes the per-macro-step 4-DoF waypoints for open-loop replay.
Trajectory columns carry commands only; simulator-only quantities go in a
separate diagnostics block. Two things the JSON states explicitly:

- `theta1`/`theta2` are **per-node** angles. A segment's total is `n_seg` times
  the value; reading them as segment totals over-bends by 5x on hardware.
- the beta residual parameters are placeholders, not hardware identification,
  so replay fidelity on the real arm is unverified.

## Measurement tools

```bash
python -m elesim_sim.rl.benchmark                      # T1 step rate
python -m elesim_sim.rl.workspace_probe --placement-search
python -m elesim_sim.rl.render_scene --tag check --place-object-after
python -m elesim_sim.rl.diagnose_nan --checkpoint <ckpt>
```

`render_scene` earns its place: two geometry bugs that survived several rounds
of numerical sweeping were obvious in the first picture. `--place-object-after`
separates "is this pose a wrap" from "can the arm reach it without sweeping the
object away" — conflating them makes a correct pose look wrong.

## Where the mechanism's numbers come from

`rl/app_config.py` reads joint ranges, the u-space mapping, the quadruped's
spawn height and the UI pose presets from `payload/config/sim/config.yaml`. Config
fields left `null` take the application's value.

This is not incidental. Every one of those values was hand-copied at first and
every one was wrong — a bend-axis sign that folded the arm through the trunk, a
command direction that turned the S-shaped Home preset into a C, a roll range of
+/-180 deg against a real +/-90, and a spawn height taken from a stale document
and then "corrected" to a schema default while the app config had the original
right. None of them look wrong in the RL config alone. They only show up against
the mechanism, so the mechanism is the source of truth.

## Known state

- Training reaches the point where the policy presses the arm onto the object
  and then hit `INVALID_FORCE_NAN` six times running. Arm link inertias were
  6.7x to 88x their geometry, which is the most likely cause and has been
  corrected; whether that clears it is **unverified** — the run was stopped at
  iteration 11, before the range where it used to fail.
- Gate G3, the fixed-pose overfit sanity check, has no result yet.
- Geometry-derived inertia assumes uniform density and is a floor for parts
  housing motors or gearing.
