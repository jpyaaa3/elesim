"""A finite sampled force path must survive an infeasible weighted average."""
import numpy as np
import pytest

from elesim_sim.robot.go2 import pympc_jax


@pytest.mark.parametrize("weighted_cost", [np.inf, np.nan])
def test_valid_sample_is_retained_when_mean_seed_and_average_are_infeasible(weighted_cost):
    # Opposing roll corrections can average into a path that crosses the tilt
    # boundary. Each candidate has already passed the same full rollout score.
    candidates = np.arange(5 * 12 * 4 * 3, dtype=float).reshape(5, 12, 4, 3)
    costs = np.array([np.inf, np.inf, 300., 100., np.inf])
    averaged = np.full((12, 4, 3), -10.)
    selected = pympc_jax._select_mppi_update(np, candidates, costs, averaged, weighted_cost, 3)
    np.testing.assert_array_equal(selected, candidates[3])


@pytest.mark.parametrize("costs,weighted_cost,expected", [
    ([10., 12., 1.], 9., "weighted"),
    ([10., 8., 1.], 9., 1),
    ([7., 8., 1.], 9., 0),
    ([np.inf, 8., 1.], np.inf, 1),
    ([np.inf, np.inf, 1.], .5, "weighted"),
])
def test_valid_deterministic_or_weighted_path_keeps_existing_jitter_avoidance(
        costs, weighted_cost, expected):
    candidates = np.arange(3 * 12, dtype=float).reshape(3, 4, 3)
    weighted = np.full((4, 3), 42.)
    selected = pympc_jax._select_mppi_update(np, candidates, np.array(costs),
                                            weighted, weighted_cost, 2)
    np.testing.assert_array_equal(selected, weighted if expected == "weighted" else candidates[expected])


def test_no_finite_sample_does_not_create_a_fabricated_success():
    candidates = np.full((3, 4, 3), np.nan)
    selected = pympc_jax._select_mppi_update(np, candidates, np.full(3, np.inf),
                                            np.zeros((4, 3)), np.inf, 0)
    assert not np.isfinite(selected).any()
