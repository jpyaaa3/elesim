"""Run training, and pick it up again when the simulator blows up.

Genesis halts the whole batch when its constraint solver produces NaN forces::

    genesis.GenesisException: Invalid constraint forces causing 'nan'.

It is rare, it is not something the policy did -- every metric reads healthy on
the iteration before -- and it scales with how many environments are exposed to
it.  Measured on this task: 2048 envs died at iteration 4 with
`scene.solver_substeps: 1`, and at iteration 207 with 2.  Halving the timestep
again buys more but cannot promise a clean 4000-iteration run, and losing 49
minutes of training to a blow-up that a resume recovers from in seconds is not a
trade worth making.

So: run `train`, and if it exits non-zero, resume from the newest checkpoint and
carry on with the iterations it had left.  The reverse curriculum's position is
restored with it, because `train --resume` reads the sidecar written next to
each checkpoint.

A restart that makes no progress is not retried forever.  If the run comes back
without having written a newer checkpoint than the one it resumed from, the
failure is reproducible rather than a blow-up, and looping on it would burn a
GPU all night to no purpose.

Run::

    python -m elesim_sim.rl.supervise --iterations 4000 --stamp srv_mirror \\
        --set runtime.n_envs=2048 --set curriculum.stage=2

    python -m elesim_sim.rl.supervise --iterations 4000 --stamp srv_next \\
        --resume rl_runs/wrap_grasp/stage2_srv_mirror/model_100.pt
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional, Sequence

_CKPT = re.compile(r"^model_(\d+)\.pt$")


def latest_checkpoint(run_dir: Path) -> tuple[Optional[Path], int]:
    """Newest `model_<n>.pt` in `run_dir`, and its iteration number."""
    best: Optional[Path] = None
    best_n = -1
    if not run_dir.is_dir():
        return None, 0
    for path in run_dir.iterdir():
        m = _CKPT.match(path.name)
        if m and int(m.group(1)) > best_n:
            best, best_n = path, int(m.group(1))
    return best, max(best_n, 0)


def _iteration_of(path: Path) -> int:
    """The iteration a `model_<n>.pt` was written at, or 0."""
    m = _CKPT.match(path.name)
    return int(m.group(1)) if m else 0


def build_command(
    argv: Sequence[str], *, resume: Optional[Path], iterations: int
) -> list[str]:
    """The `train` command line, with `--iterations` and `--resume` replaced."""
    out: list[str] = [sys.executable, "-m", "elesim_sim.rl.train"]
    skip = False
    for i, arg in enumerate(argv):
        if skip:
            skip = False
            continue
        if arg in ("--iterations", "--resume"):
            skip = True
            continue
        if arg.startswith(("--iterations=", "--resume=")):
            continue
        out.append(arg)
    out += ["--iterations", str(iterations)]
    if resume is not None:
        out += ["--resume", str(resume)]
    return out


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--iterations", type=int, required=True,
                        help="total iterations, across restarts")
    parser.add_argument("--stamp", default="run", help="run-directory suffix")
    parser.add_argument("--max-restarts", type=int, default=20)
    parser.add_argument(
        "--resume", default=None,
        help=(
            "checkpoint to start from when the run directory is empty, e.g. "
            "another run's model_100.pt.  Once this run has written a "
            "checkpoint of its own, restarts use that instead."
        ),
    )
    parser.add_argument("--config", default=None)
    parser.add_argument("--overlay", action="append", default=[])
    parser.add_argument("--set", dest="overrides", action="append", default=[])
    args = parser.parse_args(argv)

    from .configs.loader import load_config
    from .train import resolve_run_dir

    cfg = load_config(args.config, overlays=args.overlay, overrides=args.overrides)
    run_dir = resolve_run_dir(cfg, stamp=args.stamp)

    passthrough: list[str] = ["--stamp", args.stamp]
    if args.config:
        passthrough += ["--config", args.config]
    for overlay in args.overlay:
        passthrough += ["--overlay", overlay]
    for override in args.overrides:
        passthrough += ["--set", override]

    seed = Path(args.resume).expanduser() if args.resume else None
    if seed is not None and not seed.is_absolute():
        seed = (Path.cwd() / seed).resolve()
    if seed is not None and not seed.is_file():
        print(f"[supervise] --resume 가 가리키는 파일이 없습니다: {seed}")
        return 2

    total = int(args.iterations)
    restarts = 0
    while True:
        resume, done = latest_checkpoint(run_dir)
        if resume is None and seed is not None:
            # rsl_rl carries on numbering from the checkpoint it loads, so the
            # seed's own iteration counts against the total.
            resume, done = seed, _iteration_of(seed)
        remaining = total - done
        if remaining <= 0:
            print(f"[supervise] {done}/{total} iterations already done")
            return 0
        cmd = build_command(passthrough, resume=resume, iterations=remaining)
        where = (
            f"resuming from {resume}" if resume is not None and resume == seed
            else f"resuming from {resume.name}" if resume else "from scratch"
        )
        print(f"[supervise] {where}, {remaining} iterations to go", flush=True)
        code = subprocess.call(cmd)
        if code == 0:
            print("[supervise] training finished")
            return 0

        _, now = latest_checkpoint(run_dir)
        if restarts >= int(args.max_restarts):
            print(f"[supervise] giving up after {restarts} restarts "
                  f"(exit {code}, at iteration {now})")
            return code
        if now <= done:
            # It died without getting as far as its first checkpoint, so
            # restarting lands in exactly the same place.
            print(f"[supervise] exit {code} with no progress past iteration "
                  f"{done}; not a blow-up, stopping")
            return code
        restarts += 1
        print(f"[supervise] exit {code} at iteration {now}; "
              f"restart {restarts}/{args.max_restarts}", flush=True)
        time.sleep(5)


if __name__ == "__main__":
    raise SystemExit(main())
