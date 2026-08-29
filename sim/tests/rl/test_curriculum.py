"""Unit tests for the reverse curriculum's movement rule.

A 2048-env run walked the start range from t = 1.0 to 0.60 inside 50 iterations,
landed at a difficulty its policy had never learned, and had no way back: 130
iterations at exactly `success 0.0000, phi 4 deg, topple 0.42`, with every
checkpoint from 200 on scoring the same.  Retreat is what that needed.

The cooldown is a separate guard -- that run would have passed it -- for a
window that fills in about two macro steps at 2048 envs.
"""

from __future__ import annotations

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
import dataclasses

import pytest
import torch

from elesim_sim.rl.configs.loader import load_config


class _Curriculum:
    """The movement rule with the environment stripped away.

    `_note_start_pose_outcome` reads four attributes and the config; binding it
    to a stand-in keeps the test off the simulator, which needs a GPU and 20
    seconds to build a scene.
    """

    def __init__(self, cfg, t_range=(0.85, 1.0)):
        self.cfg = cfg
        self._start_t_lo, self._start_t_hi = t_range
        self._start_window_n = 0
        self._start_window_ok = 0
        self._start_steps_since_move = 0

    def feed(self, *, episodes: int, successes: int, repeats: int = 1) -> None:
        from elesim_sim.rl.envs.wrap_env import WrapGraspEnv

        dones = torch.zeros(max(episodes, 1), dtype=torch.bool)
        dones[:episodes] = True
        ok = torch.zeros_like(dones)
        ok[:successes] = True
        for _ in range(repeats):
            WrapGraspEnv._note_start_pose_outcome(self, dones, ok)

    @property
    def t(self):
        return (round(self._start_t_lo, 3), round(self._start_t_hi, 3))


def _cfg(**kw):
    cfg = load_config()
    start = dataclasses.replace(cfg.start_pose, window=100, **kw)
    return dataclasses.replace(cfg, start_pose=start)


def test_success_walks_the_range_towards_home():
    c = _Curriculum(_cfg(cooldown_updates=0))
    c.feed(episodes=100, successes=100)
    assert c.t == (0.75, 0.9)


def test_failure_walks_it_back_again():
    c = _Curriculum(_cfg(cooldown_updates=0), t_range=(0.35, 0.5))
    c.feed(episodes=100, successes=0)
    assert c.t == (0.45, 0.6)


def test_a_middling_rate_moves_nothing():
    c = _Curriculum(_cfg(cooldown_updates=0), t_range=(0.35, 0.5))
    c.feed(episodes=100, successes=30)      # between retreat_at and advance_at
    assert c.t == (0.35, 0.5)


def test_the_cooldown_bounds_how_fast_it_can_walk():
    """Four windows of perfect success inside one update must not be four steps.

    512 episodes is about two macro steps at 2048 envs, so without a cooldown
    the range can walk several times between two gradient updates.
    """
    cfg = _cfg(cooldown_updates=5)
    per_update = cfg.train.num_steps_per_env
    c = _Curriculum(cfg)
    # Four full windows, but only one update's worth of macro steps.
    c.feed(episodes=100, successes=100, repeats=4)
    assert c.t == (0.85, 1.0)
    # Once five updates have passed, one move -- not the four it banked up.
    c.feed(episodes=0, successes=0, repeats=5 * per_update)
    c.feed(episodes=100, successes=100)
    assert c.t == (0.75, 0.9)


def test_it_stops_at_home_and_at_the_far_end():
    c = _Curriculum(_cfg(cooldown_updates=0), t_range=(0.0, 0.0))
    c.feed(episodes=100, successes=100)
    assert c.t == (0.0, 0.0)
    c = _Curriculum(_cfg(cooldown_updates=0), t_range=(1.0, 1.0))
    c.feed(episodes=100, successes=0)
    assert c.t == (1.0, 1.0)


def test_retreat_can_be_switched_off():
    c = _Curriculum(_cfg(cooldown_updates=0, retreat_at=None), t_range=(0.35, 0.5))
    c.feed(episodes=100, successes=0)
    assert c.t == (0.35, 0.5)


def test_the_window_must_fill_before_anything_moves():
    c = _Curriculum(_cfg(cooldown_updates=0))
    c.feed(episodes=99, successes=99)
    assert c.t == (0.85, 1.0)
