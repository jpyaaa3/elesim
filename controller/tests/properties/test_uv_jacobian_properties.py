from __future__ import annotations

import numpy as np
import pytest

from elesim_controller.vision.visual_servoing.uv_jacobian import (
    broyden_update_uv_jacobian,
    solve_uv_control_delta,
)


def _full_row_rank_jacobian(rng: np.random.Generator) -> np.ndarray:
    while True:
        jacobian = rng.normal(size=(2, 3))
        if float(np.linalg.svd(jacobian, compute_uv=False)[-1]) > 0.25:
            return jacobian


def test_random_unsaturated_steps_contract_image_error() -> None:
    rng = np.random.default_rng(20260720)

    for _ in range(250):
        jacobian = _full_row_rank_jacobian(rng)
        error = rng.normal(scale=0.15, size=2)
        delta = solve_uv_control_delta(
            uv_error=error,
            jacobian=jacobian,
            damping=1e-6,
            gain=0.45,
            max_abs_delta=(100.0, 100.0, 100.0),
        )

        next_error = error + jacobian @ delta
        assert np.all(np.isfinite(delta))
        assert float(np.linalg.norm(next_error)) < float(np.linalg.norm(error))


def test_random_commands_never_exceed_per_axis_limits() -> None:
    rng = np.random.default_rng(31)
    limits = np.array([0.3, 0.5, 0.7], dtype=float)

    for _ in range(250):
        delta = solve_uv_control_delta(
            uv_error=rng.normal(size=2),
            jacobian=_full_row_rank_jacobian(rng),
            damping=0.03,
            gain=2.0,
            max_abs_delta=limits,
        )

        assert np.all(np.isfinite(delta))
        assert np.all(np.abs(delta) <= limits + 1e-12)


def test_broyden_exact_sample_matches_observed_change_along_command() -> None:
    rng = np.random.default_rng(77)

    for _ in range(100):
        jacobian = rng.normal(scale=0.1, size=(2, 3))
        control_delta = rng.normal(size=3)
        observed_delta = rng.normal(scale=0.2, size=2)
        updated = broyden_update_uv_jacobian(
            jacobian,
            control_delta=control_delta,
            uv_delta=observed_delta,
            alpha=1.0,
            min_control_norm=0.0,
            max_uv_delta_norm=100.0,
            max_column_norm=100.0,
        )

        np.testing.assert_allclose(updated @ control_delta, observed_delta, atol=1e-10)


@pytest.mark.parametrize(
    ("error", "jacobian"),
    [
        ((float("nan"), 0.0), np.eye(2, 3)),
        ((0.0, 0.0), np.array([[1.0, 0.0, 0.0], [0.0, float("inf"), 0.0]])),
    ],
)
def test_solver_rejects_non_finite_inputs(error: tuple[float, float], jacobian: np.ndarray) -> None:
    with pytest.raises(ValueError, match="finite"):
        solve_uv_control_delta(
            uv_error=error,
            jacobian=jacobian,
            max_abs_delta=(1.0, 1.0, 1.0),
        )
