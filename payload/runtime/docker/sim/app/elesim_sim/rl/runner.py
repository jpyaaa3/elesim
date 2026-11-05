"""PPO adapter accepting a caller-created environment, never a named task."""
from pathlib import Path
from typing import Mapping
from .execution import TrainingEnvironment


def make_runner(env: TrainingEnvironment, config: Mapping, log_dir: str | Path | None):
    from rsl_rl.runners import OnPolicyRunner
    return OnPolicyRunner(env,dict(config),log_dir=None if log_dir is None else str(log_dir),device=str(env.device))
