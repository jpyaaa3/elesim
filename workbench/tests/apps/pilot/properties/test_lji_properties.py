from __future__ import annotations

import numpy as np
import pytest

from elesim_pilot.config import PickConfig
from elesim_pilot.pick.actions import ControlService
from elesim_pilot.pick.state import PanelState
from elesim_pilot.vision.visual_servoing.local_image_jacobian import (
    ImageJacobianEstimator3D,
    SampleRejectReason,
    compute_dq_lji,
    joint_saturated,
)


def _full_row_rank_jacobian(rng: np.random.Generator) -> np.ndarray:
    while True:
        jacobian = rng.normal(size=(3, 4))
        singular = np.linalg.svd(jacobian, compute_uv=False)
        if float(singular[-1]) > 0.25:
            return jacobian


def test_random_unsaturated_lji_steps_contract_feature_error() -> None:
    rng = np.random.default_rng(20260720)

    for _ in range(250):
        jacobian = _full_row_rank_jacobian(rng)
        feature_error = rng.normal(scale=0.02, size=3)
        dq, raw = compute_dq_lji(
            j_lji=jacobian,
            s_lji=feature_error,
            damping=1e-6,
            gain_u=0.35,
            gain_v=0.35,
            gain_z=0.35,
            max_dq_linear=100.0,
            max_dq_angle=100.0,
        )

        next_error = feature_error + jacobian @ dq
        assert np.all(np.isfinite(raw))
        assert float(np.linalg.norm(next_error)) < float(np.linalg.norm(feature_error))


def test_random_lji_commands_obey_each_axis_cap() -> None:
    rng = np.random.default_rng(91)
    caps = np.array([0.004, 0.012, 0.006, 0.009], dtype=float)

    for _ in range(250):
        dq, _ = compute_dq_lji(
            j_lji=_full_row_rank_jacobian(rng),
            s_lji=rng.normal(size=3),
            damping=0.05,
            gain_u=0.5,
            gain_v=0.5,
            gain_z=0.5,
            max_dq_linear=caps[0],
            max_dq_angle=caps[1],
            max_dq_theta1=caps[2],
            max_dq_theta2=caps[3],
        )

        assert np.all(np.isfinite(dq))
        assert np.all(np.abs(dq) <= caps + 1e-12)


def test_estimator_recovers_random_known_jacobians_from_measured_motion() -> None:
    rng = np.random.default_rng(1234)

    for _ in range(40):
        expected = _full_row_rank_jacobian(rng)
        estimator = ImageJacobianEstimator3D(window_size=12, min_measured_samples=8)
        for _sample in range(12):
            measured_delta_q = rng.normal(scale=0.01, size=4)
            estimator.push(measured_delta_q, expected @ measured_delta_q)

        measured = estimator.measured_estimate(min_samples=8, condition_max=1e6, min_rank=3)
        assert measured is not None
        actual, rank, condition = measured
        assert rank == 3
        assert np.isfinite(condition)
        np.testing.assert_allclose(actual, expected, atol=1e-9)


def test_zero_measured_motion_is_not_learned_even_when_a_command_was_sent() -> None:
    service = ControlService(PanelState())
    estimator = ImageJacobianEstimator3D(window_size=8)
    service._grasp_lji_estimator_3d = estimator
    service._grasp_lji_pending_sample = {
        "q_before": np.zeros(4),
        "q_after": np.zeros(4),
        "s_before": np.array([0.1, -0.1, 0.2]),
        "s_after": np.array([0.08, -0.08, 0.18]),
        "dq_cmd": np.array([0.005, 0.01, -0.01, 0.01]),
    }

    reason = service._grasp_lji_record_measured_sample(
        pk=PickConfig(lij_sample_min_dq_norm=1e-4),
        settle_ok=True,
        object_lost=False,
    )

    assert reason is SampleRejectReason.JOINT_SATURATED
    assert estimator.sample_count() == 0


def test_estimator_records_measured_delta_not_commanded_delta() -> None:
    service = ControlService(PanelState())
    estimator = ImageJacobianEstimator3D(window_size=8)
    service._grasp_lji_estimator_3d = estimator
    measured_delta = np.array([0.002, 0.004, -0.003, 0.001])
    commanded_delta = np.array([0.006, 0.010, -0.009, 0.004])
    service._grasp_lji_pending_sample = {
        "q_before": np.zeros(4),
        "q_after": measured_delta.copy(),
        "s_before": np.array([0.1, -0.1, 0.2]),
        "s_after": np.array([0.09, -0.08, 0.19]),
        "dq_cmd": commanded_delta,
    }

    reason = service._grasp_lji_record_measured_sample(
        pk=PickConfig(
            lij_sample_min_dq_norm=1e-4,
            lij_sample_meas_cmd_ratio_min=0.0,
            lij_sample_meas_cmd_ratio_max=0.0,
            lij_sample_cmd_meas_cos_min=-1.0,
        ),
        settle_ok=True,
        object_lost=False,
    )

    assert reason is SampleRejectReason.ACCEPTED
    assert estimator.sample_count() == 1
    np.testing.assert_allclose(estimator._samples[0].delta_q, measured_delta)
    assert not np.allclose(estimator._samples[0].delta_q, commanded_delta)


def test_saturation_threshold_honors_the_requested_motion_fraction() -> None:
    assert joint_saturated(
        q_before=(0.0, 0.0, 0.0, 0.0),
        q_cmd=(1.0, 0.0, 0.0, 0.0),
        q_after=(0.075, 0.0, 0.0, 0.0),
        min_motion_frac=0.10,
    )
    assert not joint_saturated(
        q_before=(0.0, 0.0, 0.0, 0.0),
        q_cmd=(1.0, 0.0, 0.0, 0.0),
        q_after=(0.125, 0.0, 0.0, 0.0),
        min_motion_frac=0.10,
    )


def test_lji_solver_rejects_non_finite_inputs() -> None:
    jacobian = np.eye(3, 4)
    jacobian[0, 0] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        compute_dq_lji(
            j_lji=jacobian,
            s_lji=(0.0, 0.0, 0.1),
            damping=0.05,
            gain_u=0.5,
            gain_v=0.5,
            gain_z=0.5,
            max_dq_linear=0.01,
            max_dq_angle=0.02,
        )
