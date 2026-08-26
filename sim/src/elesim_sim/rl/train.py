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

_REPO_ROOT = Path(__file__).resolve().parents[4]


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
        path = Path(resume)
        if not path.is_absolute():
            path = _REPO_ROOT / path
        print(f"[train] resuming from : {path}")
        runner.load(str(path))

    iterations = int(args.iterations or cfg.train.max_iterations)
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
