"""Delayed observations may not carry the previous episode into a reset."""
from types import SimpleNamespace

import elesim_sim.rl  # numpy-before-torch ordering
import torch

from elesim_sim.rl.envs.observations import ObservationBuilder


def builder():
    env = SimpleNamespace(
        cfg=SimpleNamespace(observation=SimpleNamespace(actor=SimpleNamespace(delay_steps=(2, 2)))),
        num_envs=2, device="cpu", _generator=torch.Generator().manual_seed(7),
    )
    result = ObservationBuilder(env)
    result.reset_delay(None, 2)
    return result


def test_reset_clears_only_the_finished_environments_delay_history():
    observations = builder()
    observations._apply_obs_delay(torch.tensor([[1.0], [10.0]]))
    observations._apply_obs_delay(torch.tensor([[2.0], [20.0]]))
    observations.reset_delay(torch.tensor([1]), 1)
    actual = observations._apply_obs_delay(torch.tensor([[3.0], [30.0]]))
    torch.testing.assert_close(actual, torch.tensor([[1.0], [30.0]]))


def test_full_reset_has_no_previous_episode_pixels_or_state():
    observations = builder()
    observations._apply_obs_delay(torch.tensor([[1.0], [10.0]]))
    observations._apply_obs_delay(torch.tensor([[2.0], [20.0]]))
    observations.reset_delay(None, 2)
    actual = observations._apply_obs_delay(torch.tensor([[3.0], [30.0]]))
    torch.testing.assert_close(actual, torch.tensor([[3.0], [30.0]]))
