"""PPO training for wrap grasping (rsl_rl).

Asymmetric actor-critic: the actor sees only what a real arm could report, the
critic additionally sees the privileged group.  That split is declared in
`train.runner.obs_groups` and enforced by the environment, which puts simulator
truth in a separate observation group the actor never reads.

Run::

    python -m elesim_sim.rl.train --set runtime.n_envs=1024
    python -m elesim_sim.rl.train --set curriculum.stage=2 --resume <ckpt>
"""

from __future__ import annotations

import argparse
import json
import os
import random
from pathlib import Path
from typing import Optional, Sequence

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
import numpy as np
import torch

from .configs.loader import load_config, to_dict, WrapGraspConfig
from .envs.wrap_env import WrapGraspEnv

_REPO_ROOT = next(root for root in Path(__file__).resolve().parents if (root / "AGENTS.md").is_file())


def set_seed(seed: int, *, deterministic: bool = False) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    if deterministic:
        # Slower, but makes a reported run reproducible rather than
        # approximately reproducible.
        os.environ.setdefault("CUBLAS_WORKSPACE_CONFIG", ":4096:8")
        torch.use_deterministic_algorithms(True, warn_only=True)


def resolve_run_dir(cfg: WrapGraspConfig, *, stamp: str) -> Path:
    root = Path(cfg.train.log_dir)
    if not root.is_absolute():
        root = _REPO_ROOT / root
    name = cfg.train.run_name or f"stage{cfg.curriculum.stage}_{stamp}"
    return root / cfg.train.experiment_name / name


def build_runner(env: WrapGraspEnv, cfg: WrapGraspConfig, log_dir: Path):
    from rsl_rl.runners import OnPolicyRunner

    train_cfg = dict(to_dict(cfg.train.runner))
    train_cfg["num_steps_per_env"] = int(cfg.train.num_steps_per_env)
    train_cfg["save_interval"] = int(cfg.train.save_interval)
    train_cfg["experiment_name"] = str(cfg.train.experiment_name)
    train_cfg.setdefault("logger", "tensorboard")
    train_cfg.setdefault("empirical_normalization", False)
    return OnPolicyRunner(env, train_cfg, log_dir=str(log_dir), device=str(env.device))


def _git_state() -> dict:
    """The repository revision, and whether it had uncommitted changes."""
    import subprocess

    def run(*args: str) -> Optional[str]:
        try:
            out = subprocess.run(
                args, cwd=_REPO_ROOT, capture_output=True, text=True, timeout=10
            )
        except Exception:
            return None
        return out.stdout.strip() if out.returncode == 0 else None

    commit = run("git", "rev-parse", "HEAD")
    status = run("git", "status", "--porcelain")
    return {
        "commit": commit,
        "dirty": bool(status) if status is not None else None,
        "branch": run("git", "rev-parse", "--abbrev-ref", "HEAD"),
    }


def _install_curriculum_sidecar(runner, env: WrapGraspEnv) -> None:
    """Write the curriculum position beside every checkpoint rsl_rl saves.

    rsl_rl owns the checkpoint format, so this goes in a sidecar rather than
    into its dict: a file it does not know about cannot break its loader across
    versions.
    """
    save = runner.save

    def save_with_curriculum(path, *args, **kwargs):
        result = save(path, *args, **kwargs)
        try:
            lo, hi = env.start_pose_range
            Path(path).with_suffix(".curriculum.json").write_text(
                json.dumps({"t_lo": lo, "t_hi": hi}), encoding="utf-8"
            )
        except Exception as exc:  # bookkeeping must never kill a training run
            print(f"[train] could not write the curriculum sidecar: {exc}")
        return result

    runner.save = save_with_curriculum


def _install_best_checkpoint(runner, env: WrapGraspEnv, log_dir: Path) -> None:
    """Keep a copy of the best policy seen at the deployed difficulty.

    Two runs have now peaked and then declined -- one reaching 78.3% success
    from Home at iteration 100 and 0.0% by 700, with every reward term falling
    together -- and periodic checkpoints only preserve a peak by luck.

    "Best" is judged only once the reverse curriculum has reached Home, because
    a rate measured at the top of it is not the task's: with the arm resetting
    already wrapped, a policy scores ~80% for pressing the lift, which would
    beat any honest reading taken later.
    """
    best = {"rate": -1.0}
    # `runner.logger.log`, not `runner.log`: rsl_rl 5.4 logs through a Logger
    # object, and wrapping the runner attribute hooked a method nothing calls.
    logger = runner.logger
    log = logger.log

    def log_and_track(*args, **kwargs):
        result = log(*args, **kwargs)
        try:
            rate, n = env.take_recent_success_rate(min_episodes=200)
            t_lo, _ = env.start_pose_range
            if n >= 200 and t_lo <= 1e-6 and rate > best["rate"]:
                best["rate"] = rate
                runner.save(str(log_dir / "model_best.pt"))
                print(f"[train] best so far: {rate:.1%} over {n} episodes",
                      flush=True)
        except Exception as exc:  # bookkeeping must never kill a training run
            print(f"[train] could not track the best checkpoint: {exc}")
        return result

    logger.log = log_and_track


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None)
    parser.add_argument("--overlay", action="append", default=[])
    parser.add_argument("--set", action="append", default=[], dest="overrides")
    parser.add_argument("--iterations", type=int, default=None)
    parser.add_argument("--resume", default=None, help="checkpoint .pt to resume from")
    parser.add_argument("--stamp", default="run", help="run-directory suffix")
    args = parser.parse_args(argv)

    cfg = load_config(args.config, overlays=args.overlay, overrides=args.overrides)
    set_seed(int(cfg.runtime.seed), deterministic=bool(cfg.runtime.deterministic))

    env = WrapGraspEnv(cfg)
    log_dir = resolve_run_dir(cfg, stamp=args.stamp)
    log_dir.mkdir(parents=True, exist_ok=True)
    meta = env.metadata()
    # The commit the run was trained at.
    #
    # Evaluating a checkpoint with code that differs from the code that trained
    # it silently scores the wrong thing: the observation vector keeps its width
    # while its channels change meaning.  That happened here -- an eval was
    # reading an object "yaw" the training had stopped producing -- so the
    # commit travels with the run and `watch_eval` refuses a mismatch.
    meta["git"] = _git_state()
    # rsl_rl's logger reads env.cfg for its run record.  The environment itself
    # uses that attribute as its typed config throughout, so it is left alone
    # and the readable copy goes to metadata.json instead.
    (log_dir / "metadata.json").write_text(
        json.dumps(meta, indent=2, default=str), encoding="utf-8"
    )
    print(f"[train] run dir       : {log_dir}")
    print(f"[train] envs          : {env.num_envs} on {env.device}")
    print(f"[train] obs           : policy {env.obs_spec.policy}, "
          f"privileged {env.obs_spec.privileged}")
    print(f"[train] curriculum    : stage {cfg.curriculum.stage}, "
          f"success = {cfg.success.criterion}")
    print(f"[train] coverage gate : {meta['coverage_target_deg']:.1f} deg")
    print(f"[train] beta          : {meta['beta']['source']}")

    runner = build_runner(env, cfg, log_dir)
    resume = args.resume or cfg.train.resume
    if resume:
        path = Path(resume).expanduser()
        if not path.is_absolute():
            # Try the working directory before the repo root.  `supervise`
            # resolves its --resume against the cwd and `train` resolved it
            # against the repo root, so the same string meant two different
            # files depending on which one you called: run from sim/,
            # `rl_runs/...` looked for `<repo>/rl_runs/...` and died on a path
            # that reads as though it should exist.
            here = Path.cwd() / path
            path = here if here.is_file() else _REPO_ROOT / path
        print(f"[train] resuming from : {path}")
        # Map to this machine's device.  rsl_rl defaults map_location to None,
        # which restores every tensor to the device recorded in the file, so a
        # checkpoint trained on an Apple GPU fails to load on a CUDA box with
        # "Storage device not recognized: mps" -- and the reverse for a CUDA
        # checkpoint on a machine without one.
        runner.load(str(path), map_location=str(env.device))
        # The reverse curriculum's position lives on the env, not in the
        # checkpoint rsl_rl writes, so a resume would otherwise restart it at
        # the configured `t_range` and hand the policy back the easy start it
        # had already earned its way out of.
        side = path.with_suffix(".curriculum.json")
        if side.is_file() and cfg.start_pose.enable:
            saved = json.loads(side.read_text())
            env.start_pose_range = (saved["t_lo"], saved["t_hi"])
            # Report what the environment is actually set to, not what the file
            # said.  The setter normalises the range into a window of the
            # configured width -- a checkpoint written before the window kept
            # one records (0, 0) at the bottom -- and printing the file's values
            # made a normalised range look like it had been ignored.
            lo, hi = env.start_pose_range
            note = "" if (lo, hi) == (saved["t_lo"], saved["t_hi"]) else (
                f"  (saved {saved['t_lo']:.2f}-{saved['t_hi']:.2f}, "
                f"widened to the configured window)"
            )
            print(f"[train] curriculum at : t = {lo:.2f}-{hi:.2f}{note}")
        elif cfg.start_pose.enable:
            print(f"[train] curriculum at : t = {cfg.start_pose.t_range[0]:.2f}-"
                  f"{cfg.start_pose.t_range[1]:.2f} (nothing saved beside the "
                  f"checkpoint; starting the curriculum over)")

    iterations = int(args.iterations or cfg.train.max_iterations)
    _install_curriculum_sidecar(runner, env)
    _install_best_checkpoint(runner, env, log_dir)
    runner.learn(num_learning_iterations=iterations, init_at_random_ep_len=False)

    stats = env.statistics()
    (log_dir / "final_stats.json").write_text(
        json.dumps(stats, indent=2), encoding="utf-8"
    )
    print("[train] rollout statistics over the whole run:")
    for key, value in stats.items():
        print(f"          {key:<22s} {value:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
