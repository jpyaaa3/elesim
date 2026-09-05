from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from elesim_pilot.robot.arm.iklib.kinematics import _forward_grasp_world
from elesim_pilot.robot.arm.iklib.solver import load_solver_context, solve_ik
from elesim_pilot.vision.visual_servoing.pick_view_pregrasp import (
    generate_view_pregrasp_candidates,
)
from elesim_pilot.vision.visual_servoing.ready_pose import compute_ready_pose_target
from elesim_pilot.vision.visual_servoing.sag_drift_frame import prepare_sag_drift_input


REPO_ROOT = next(parent for parent in Path(__file__).resolve().parents if (parent / "payload").is_dir())
CONFIG_PATH = REPO_ROOT / "payload" / "config" / "pilot" / "config.yaml"


def test_random_ready_targets_preserve_standoff_and_approach_direction() -> None:
    rng = np.random.default_rng(20260720)

    for _ in range(250):
        object_world = rng.uniform(-2.0, 2.0, size=3)
        direction = rng.normal(size=3)
        standoff = float(rng.uniform(0.01, 0.50))
        ready = np.asarray(
            compute_ready_pose_target(object_world, direction, standoff_m=standoff),
            dtype=float,
        )

        object_to_ready = object_world - ready
        expected_direction = direction / np.linalg.norm(direction)
        assert float(np.linalg.norm(object_to_ready)) == pytest.approx(standoff, abs=1e-10)
        np.testing.assert_allclose(object_to_ready / standoff, expected_direction, atol=1e-10)


def test_random_view_candidates_are_finite_and_point_at_the_object() -> None:
    rng = np.random.default_rng(19)

    for _ in range(50):
        object_world = rng.uniform(-1.0, 1.0, size=3)
        offset = rng.normal(size=3)
        offset = -0.2 * offset / np.linalg.norm(offset)
        candidates = generate_view_pregrasp_candidates(
            object_world,
            base_offset_m=offset,
            view_distance_m=0.20,
            lateral_offsets_m=(-0.03, 0.0, 0.03),
            height_offsets_m=(0.0, 0.04),
        )

        assert candidates
        for candidate in candidates:
            position = np.asarray(candidate.pregrasp_world, dtype=float)
            look = np.asarray(candidate.look_dir_world, dtype=float)
            expected = object_world - position
            expected /= np.linalg.norm(expected)
            assert np.all(np.isfinite(position))
            assert float(np.linalg.norm(look)) == pytest.approx(1.0, abs=1e-10)
            np.testing.assert_allclose(look, expected, atol=1e-10)


def test_sag_drift_decomposition_is_orthogonal_and_reconstructs_input() -> None:
    rng = np.random.default_rng(43)

    for _ in range(250):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        drift = rng.normal(scale=0.03, size=3)
        result = prepare_sag_drift_input(
            drift_world=drift,
            axis_world=axis,
            reference_dir=axis,
            max_dir_error_deg=1.0,
            max_lateral_m=1.0,
            min_axial_m=0.0,
            axial_only=True,
        )

        axial = np.asarray(result.sag_input_world, dtype=float)
        lateral = drift - axial
        assert result.usable
        assert float(np.dot(axial, lateral)) == pytest.approx(0.0, abs=1e-10)
        assert float(np.linalg.norm(lateral)) == pytest.approx(result.lateral_m, abs=1e-10)
        np.testing.assert_allclose(axial + lateral, drift, atol=1e-10)


@pytest.mark.parametrize(
    "call",
    [
        lambda: compute_ready_pose_target((float("nan"), 0.0, 0.0), (1.0, 0.0, 0.0), standoff_m=0.2),
        lambda: compute_ready_pose_target((0.0, 0.0, 0.0), (float("inf"), 0.0, 0.0), standoff_m=0.2),
        lambda: compute_ready_pose_target((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), standoff_m=float("nan")),
        lambda: prepare_sag_drift_input(
            drift_world=(float("nan"), 0.0, 0.0),
            axis_world=(1.0, 0.0, 0.0),
            reference_dir=(1.0, 0.0, 0.0),
            max_dir_error_deg=10.0,
            max_lateral_m=0.02,
        ),
    ],
)
def test_geometry_boundaries_reject_non_finite_inputs(call: object) -> None:
    with pytest.raises(ValueError, match="finite"):
        call()


def test_generated_arm_model_round_trips_random_reachable_fk_targets() -> None:
    _bundle, context = load_solver_context(str(CONFIG_PATH))
    rng = np.random.default_rng(20260720)

    for _ in range(50):
        expected_q = np.array(
            [
                rng.uniform(-0.20, -0.03),
                rng.uniform(-0.50, 0.50),
                rng.uniform(-0.25, 0.25),
                rng.uniform(-0.25, 0.25),
            ],
            dtype=float,
        )
        target = _forward_grasp_world(context, expected_q)
        seed = expected_q + rng.normal(scale=(0.005, 0.02, 0.02, 0.02), size=4)
        result = solve_ik(
            target_world=target,
            context=context,
            position_tol_m=5e-4,
            max_iters=160,
            current_seed=seed,
        )

        assert result.success, (result.reason, result.position_error_m)
        assert result.q is not None
        reached = _forward_grasp_world(context, result.q)
        assert float(np.linalg.norm(reached - target)) <= 5e-4


def test_generated_arm_model_rejects_a_clearly_unreachable_target_cleanly() -> None:
    _bundle, context = load_solver_context(str(CONFIG_PATH))
    result = solve_ik(
        target_world=(10.0, 10.0, 10.0),
        context=context,
        position_tol_m=5e-4,
        max_iters=60,
        current_seed=(-0.1, 0.0, 0.0, 0.0),
    )

    assert not result.success
    assert np.isfinite(result.position_error_m)
    assert result.reason == "position tolerance not reached"


def test_runtime_loader_never_rebuilds_a_missing_arm_model(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    missing_model = tmp_path / "not-generated" / "arm_model.json"
    monkeypatch.setenv("ELESIM_ARM_MODEL", str(missing_model))

    with pytest.raises(FileNotFoundError, match="generated arm model not found"):
        load_solver_context(str(CONFIG_PATH))

    assert not missing_model.parent.exists()
