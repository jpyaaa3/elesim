"""What the recorded clip is a clip *of*.

The graded table and the mp4 come from two different rollouts, and only the
graded one used to pin its condition.  The recording cleared the pin, so the
reset fell through to domain randomisation and the clip showed whatever size it
drew -- under a filename naming a size it might not be.  Measured: clips
written for 67 and 100 mm both showed the same cylinder, 24 px wide in both.

These tests pin the pinning.
"""

from __future__ import annotations

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
import torch

from elesim_sim.rl.eval import _record_episode


class _FakeCamera:
    def __init__(self) -> None:
        self.recording = False
        self.frames = 0

    def start_recording(self) -> None:
        self.recording = True

    def render(self) -> None:
        self.frames += 1


class _FakeScene:
    def __init__(self) -> None:
        self.cameras = {"eval": _FakeCamera()}


class _FakeEnv:
    """The slice of WrapGraspEnv that `_record_episode` touches."""

    def __init__(self, *, substeps: int = 4, done_after: int = 2) -> None:
        self.scene = _FakeScene()
        self.substep_monitor = None
        self._eval_override = {"stale": 1.0}
        self.support_moves: list[tuple[float, float]] = []
        self.overrides_at_reset: list[object] = []
        self._substeps = substeps
        self._done_after = done_after
        self._steps = 0

    def move_support_to(self, dx_m: float, dy_m: float) -> None:
        self.support_moves.append((dx_m, dy_m))

    def reset(self):
        # What the object is built from is decided here, so this is the moment
        # the override has to already be in place.
        self.overrides_at_reset.append(
            None if self._eval_override is None else dict(self._eval_override)
        )
        return torch.zeros(1, 4), {}

    def step(self, _actions):
        self._steps += 1
        for i in range(self._substeps):
            if self.substep_monitor is not None:
                self.substep_monitor(self, i)
        done = torch.tensor([self._steps % self._done_after == 0])
        return torch.zeros(1, 4), torch.zeros(1), done, {}


def _policy(obs):
    return torch.zeros(obs.shape[0], 5)


CONDITION = {"dx_m": 0.02, "dy_m": -0.03, "yaw_rad": 0.1, "radius_m": 0.100}


def test_the_condition_is_pinned_before_the_reset_that_builds_the_object():
    env = _FakeEnv()
    _record_episode(
        env, _policy, macro_steps=4, every=1, episodes=1, condition=CONDITION
    )
    assert env.overrides_at_reset == [dict(CONDITION)]


def test_the_pin_carries_every_key_the_reset_reads():
    env = _FakeEnv()
    _record_episode(
        env, _policy, macro_steps=4, every=1, episodes=1, condition=CONDITION
    )
    # `_reset_object` indexes radius_m and yaw_rad rather than .get()-ing them,
    # so a partial condition would raise there instead of here.
    assert set(env.overrides_at_reset[0]) == {"dx_m", "dy_m", "yaw_rad", "radius_m"}


def test_the_support_follows_the_offset_object():
    env = _FakeEnv()
    _record_episode(
        env, _policy, macro_steps=4, every=1, episodes=1, condition=CONDITION
    )
    assert env.support_moves == [(0.02, -0.03)]


def test_the_pin_replaces_a_stale_one_rather_than_being_merged_into_it():
    env = _FakeEnv()
    env._eval_override = {"radius_m": 0.045, "dx_m": 0.9, "dy_m": 0.9, "yaw_rad": 0.0}
    _record_episode(
        env, _policy, macro_steps=4, every=1, episodes=1, condition=CONDITION
    )
    assert env.overrides_at_reset[0]["radius_m"] == 0.100


def test_no_condition_leaves_the_object_randomised():
    # Recording the randomised distribution stays available, but only by
    # asking for it.
    env = _FakeEnv()
    _record_episode(env, _policy, macro_steps=4, every=1, episodes=1)
    assert env.overrides_at_reset == [None]
    assert env.support_moves == []


def test_recording_stops_after_the_requested_episodes():
    env = _FakeEnv(done_after=2)
    _record_episode(
        env, _policy, macro_steps=20, every=1, episodes=3, condition=CONDITION
    )
    assert env._steps == 6


def test_frames_are_taken_every_stride_substeps():
    env = _FakeEnv(substeps=4, done_after=100)
    camera = env.scene.cameras["eval"]
    _record_episode(
        env, _policy, macro_steps=3, every=2, episodes=1, condition=CONDITION
    )
    # 3 macro steps x 4 substeps = 12 substeps, every 2nd captured.
    assert camera.frames == 6
    assert camera.recording


def test_the_previous_substep_monitor_is_restored():
    env = _FakeEnv()
    sentinel = object()
    env.substep_monitor = sentinel
    _record_episode(
        env, _policy, macro_steps=4, every=1, episodes=1, condition=CONDITION
    )
    assert env.substep_monitor is sentinel
