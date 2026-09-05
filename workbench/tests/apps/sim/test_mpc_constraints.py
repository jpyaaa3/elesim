from __future__ import annotations

import math

import numpy as np
import pytest

from elesim_sim.robot.go2.mpc.constraints import (
    MpcSchedule,
    normal_force_variable_indices,
    optimization_mu,
    project_grf,
    project_grf_to_coulomb_cone,
)


def test_rectangular_pyramid_coefficient_is_conservative() -> None:
    assert optimization_mu(0.8) == pytest.approx(0.8 / math.sqrt(2.0))
    assert optimization_mu(0.0) == 0.0


@pytest.mark.parametrize("value", [-1.0, math.inf, -math.inf, math.nan, "bad"])
def test_invalid_physical_mu_is_rejected(value) -> None:
    with pytest.raises(ValueError):
        optimization_mu(value)


def test_grf_projection_clamps_normal_and_radial_tangent() -> None:
    result = project_grf_to_coulomb_cone(
        np.array([4.0, 3.0, 10.0]), physical_mu=0.5, fz_max=6.0
    )
    assert result[2] == pytest.approx(6.0)
    assert np.linalg.norm(result[:2]) == pytest.approx(3.0)
    assert result == pytest.approx([2.4, 1.8, 6.0])


def test_grf_projection_zeroes_tangent_when_normal_is_nonpositive() -> None:
    result = project_grf(np.array([2.0, -3.0, -1.0]), physical_mu=0.8, fz_max=10.0)
    assert result == pytest.approx([0.0, 0.0, 0.0])


def test_grf_projection_preserves_feasible_force_and_returns_copy() -> None:
    source = np.array([1.0, 1.0, 4.0])
    result = project_grf(source, physical_mu=0.8, fz_max=10.0)
    assert result == pytest.approx(source)
    assert result is not source


@pytest.mark.parametrize(
    "grf, physical_mu, fz_max",
    [
        ([1.0, 2.0], 0.8, 5.0),
        ([1.0, 2.0, 3.0], -0.1, 5.0),
        ([1.0, 2.0, 3.0], 0.8, -1.0),
        ([1.0, math.nan, 3.0], 0.8, 5.0),
        ([1.0, 2.0, 3.0], math.inf, 5.0),
    ],
)
def test_invalid_grf_projection_input_is_rejected(grf, physical_mu, fz_max) -> None:
    with pytest.raises(ValueError):
        project_grf(grf, physical_mu=physical_mu, fz_max=fz_max)


def test_schedule_rounds_to_an_integer_stride_and_recomputes_dt() -> None:
    schedule = MpcSchedule.from_cadence(50.0, 0.025)
    assert schedule.solve_stride == 1
    assert schedule.mpc_dt_s == pytest.approx(1.0 / 50.0)
    assert schedule.nominal_mpc_dt_s == pytest.approx(0.025)
    assert schedule.mpc_dt_s == pytest.approx(
        schedule.solve_stride / schedule.effective_control_hz
    )


def test_schedule_uses_requested_integer_stride() -> None:
    assert MpcSchedule.from_cadence(200.0, 0.025).solve_stride == 5


@pytest.mark.parametrize(
    "effective_control_hz, nominal_mpc_dt_s",
    [(0.0, 0.025), (-1.0, 0.025), (math.inf, 0.025), (200.0, 0.0), (200.0, math.nan)],
)
def test_invalid_schedule_input_is_rejected(effective_control_hz, nominal_mpc_dt_s) -> None:
    with pytest.raises(ValueError):
        MpcSchedule.from_cadence(effective_control_hz, nominal_mpc_dt_s)


def test_normal_force_indices_cover_all_legs_at_every_horizon_step() -> None:
    np.testing.assert_array_equal(
        normal_force_variable_indices(2),
        np.array([26, 29, 32, 35, 38, 41, 44, 47]),
    )


@pytest.mark.parametrize("horizon", (0, -1))
def test_normal_force_indices_reject_empty_horizon(horizon: int) -> None:
    with pytest.raises(ValueError, match="horizon"):
        normal_force_variable_indices(horizon)
