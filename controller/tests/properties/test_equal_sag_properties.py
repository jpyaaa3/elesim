from __future__ import annotations

import numpy as np

from elesim_controller.vision.visual_servoing.equal_sag_probe import solve_equal_sag_offsets


def _well_conditioned_sensitivity(rng: np.random.Generator) -> np.ndarray:
    basis, _ = np.linalg.qr(rng.normal(size=(3, 2)))
    return basis @ np.diag([0.008, 0.014])


def test_random_reconstructable_drifts_recover_the_generating_offsets() -> None:
    rng = np.random.default_rng(20260720)

    for _ in range(200):
        sensitivity = _well_conditioned_sensitivity(rng)
        offsets = rng.uniform(-8.0, 8.0, size=2)
        drift = sensitivity @ offsets
        estimate = solve_equal_sag_offsets(
            drift_world=drift,
            sensitivity_m_per_deg=sensitivity,
            max_abs_offset_deg=12.0,
            min_drift_m=0.0,
            max_residual_m=1e-9,
        )

        assert estimate.accepted, estimate.reason
        np.testing.assert_allclose(
            [estimate.seg1_equal_offset_deg, estimate.seg2_equal_offset_deg],
            offsets,
            atol=1e-9,
        )
        np.testing.assert_allclose(estimate.reconstructed_drift_world, drift, atol=1e-9)


def test_unmodelled_lateral_component_is_rejected_by_residual_gate() -> None:
    sensitivity = np.array(
        [[0.01, 0.0], [0.0, 0.02], [0.0, 0.0]],
        dtype=float,
    )
    estimate = solve_equal_sag_offsets(
        drift_world=(0.01, -0.02, 0.05),
        sensitivity_m_per_deg=sensitivity,
        min_drift_m=0.0,
        max_residual_m=0.01,
    )

    assert not estimate.accepted
    assert estimate.reason == "residual_too_large"


def test_non_finite_measurement_is_rejected_instead_of_being_accepted_with_nan() -> None:
    estimate = solve_equal_sag_offsets(
        drift_world=(float("nan"), 0.0, 0.0),
        sensitivity_m_per_deg=np.eye(3, 2),
        min_drift_m=0.0,
    )

    assert not estimate.accepted
    assert estimate.reason == "non_finite_input"


def test_non_finite_sensitivity_is_rejected() -> None:
    sensitivity = np.eye(3, 2)
    sensitivity[1, 1] = float("inf")
    estimate = solve_equal_sag_offsets(
        drift_world=(0.01, 0.01, 0.0),
        sensitivity_m_per_deg=sensitivity,
        min_drift_m=0.0,
    )

    assert not estimate.accepted
    assert estimate.reason == "non_finite_input"
