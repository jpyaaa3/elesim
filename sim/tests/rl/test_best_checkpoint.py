"""Keeping the best policy seen, rather than whichever happened to land on a
save interval.

Two runs peaked and then declined -- one reaching 78.3% success from Home at
iteration 100 and 0.0% by iteration 700 -- and periodic checkpoints preserved
the peak only by luck.
"""

from __future__ import annotations

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
from pathlib import Path

from elesim_sim.rl.train import _install_best_checkpoint


class _Env:
    """Just the two things the hook reads off the environment."""

    def __init__(self, readings, t_lo=0.0):
        self._readings = list(readings)
        self._t_lo = t_lo

    def take_recent_success_rate(self, min_episodes=1):
        return self._readings.pop(0) if self._readings else (0.0, 0)

    @property
    def start_pose_range(self):
        return (self._t_lo, self._t_lo + 0.15)


class _Logger:
    def log(self, **kwargs):
        return "logged"


class _Runner:
    """rsl_rl 5.4 logs through `runner.logger.log`, not `runner.log`."""

    def __init__(self):
        self.saved = []
        self.logger = _Logger()

    def save(self, path):
        self.saved.append(Path(path).name)


def _run(readings, *, t_lo=0.0, tmp_path=Path("/tmp")):
    runner, env = _Runner(), _Env(readings, t_lo)
    _install_best_checkpoint(runner, env, tmp_path)
    for _ in range(len(readings)):
        runner.logger.log(it=0)
    return runner.saved


def test_it_saves_only_when_the_rate_improves(tmp_path):
    saved = _run([(0.2, 500), (0.5, 500), (0.4, 500), (0.7, 500)],
                 tmp_path=tmp_path)
    assert saved == ["model_best.pt"] * 3      # 0.2, 0.5, 0.7 -- not 0.4


def test_a_decline_after_the_peak_does_not_overwrite_it(tmp_path):
    saved = _run([(0.78, 900), (0.04, 900), (0.0, 900)], tmp_path=tmp_path)
    assert saved == ["model_best.pt"]


def test_nothing_is_saved_before_the_curriculum_reaches_home(tmp_path):
    """A rate measured at the top of the curriculum is not the task's.

    The arm resets already wrapped there, so a policy scores ~80% for pressing
    the lift -- which would beat any honest reading taken later.
    """
    assert _run([(0.9, 900), (0.95, 900)], t_lo=0.85, tmp_path=tmp_path) == []


def test_a_thin_sample_is_not_a_reading(tmp_path):
    assert _run([(1.0, 3)], tmp_path=tmp_path) == []


def test_the_wrapped_log_still_returns_what_it_wrapped(tmp_path):
    runner, env = _Runner(), _Env([(0.5, 500)])
    _install_best_checkpoint(runner, env, tmp_path)
    assert runner.logger.log(it=0) == "logged"


def test_a_failure_in_the_hook_never_kills_training(tmp_path):
    class _Broken(_Env):
        def take_recent_success_rate(self, min_episodes=1):
            raise RuntimeError("boom")

    runner = _Runner()
    _install_best_checkpoint(runner, _Broken([]), tmp_path)
    assert runner.logger.log(it=0) == "logged"
    assert runner.saved == []
