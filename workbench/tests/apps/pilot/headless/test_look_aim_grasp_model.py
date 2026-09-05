from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from elesim_pilot.pick.workflow import PickWorkflowPhase, run_pick_workflow
from elesim_pilot.vision.visual_servoing.feasible_ready_pose import resolve_feasible_ready_pose
from elesim_pilot.vision.visual_servoing.local_image_jacobian import (
    ImageJacobianEstimator3D,
    compute_dq_lji,
)
from elesim_pilot.vision.visual_servoing.uv_jacobian import solve_uv_control_delta


@dataclass(frozen=True)
class _IkResult:
    success: bool
    q: np.ndarray
    position_error_m: float
    direction_angle_rad: float
    reason: str = "headless model"


class _HeadlessPickPlant:
    """Small linear plant used to test phase composition, not robot fidelity."""

    def __init__(self) -> None:
        self.object_world = np.array([0.8, 0.0, 0.2], dtype=float)
        self.preferred_dir = np.array([1.0, 0.0, 0.0], dtype=float)
        self.ready_pose: np.ndarray | None = None
        self.uv = np.array([0.42, -0.31], dtype=float)
        self.uv_jacobian = np.array(
            [[-0.10, 0.015, -0.010], [0.005, -0.08, 0.11]],
            dtype=float,
        )
        self.features = np.array([0.025, -0.020, 0.080], dtype=float)
        self.lji_jacobian = np.array(
            [
                [0.00, -1.20, 0.10, -0.05],
                [0.00, 0.05, -0.90, 1.10],
                [-1.00, 0.02, -0.20, -0.18],
            ],
            dtype=float,
        )
        self.phase_done = False
        self.failed = False
        self.commands: list[np.ndarray] = []

    def look(self) -> None:
        def solve_fn(**kwargs: object) -> _IkResult:
            target = np.asarray(kwargs["target_world"], dtype=float).reshape(3)
            q = np.array([target[0], target[1], target[2], 0.0], dtype=float)
            return _IkResult(True, q, 0.001, math.radians(2.0))

        result = resolve_feasible_ready_pose(
            object_world=self.object_world,
            preferred_dir=self.preferred_dir,
            standoff_m=0.20,
            ik_context={},
            current_seed=np.zeros(4),
            position_tol_m=0.01,
            max_iters=20,
            max_dir_error_deg=10.0,
            skip_search_under_deg=5.0,
            lateral_offsets_m=(-0.05, 0.0, 0.05),
            height_offsets_m=(0.0, 0.05),
            solve_fn=solve_fn,
        )
        self.failed = not result.success
        self.ready_pose = None if result.resolved_target is None else np.asarray(result.resolved_target)
        self.phase_done = result.success

    def aim(self) -> None:
        self.phase_done = False
        for _ in range(30):
            if float(np.linalg.norm(self.uv)) <= 0.005:
                self.phase_done = True
                return
            command = solve_uv_control_delta(
                uv_error=self.uv,
                jacobian=self.uv_jacobian,
                damping=0.01,
                gain=0.45,
                max_abs_delta=(0.8, 0.8, 0.8),
            )
            self.commands.append(command)
            self.uv += self.uv_jacobian @ command
        self.failed = True

    def grasp(self) -> None:
        self.phase_done = False
        estimator = ImageJacobianEstimator3D(window_size=12, min_measured_samples=6)
        for _ in range(50):
            if abs(float(self.features[2])) <= 0.003 and float(np.linalg.norm(self.features[:2])) <= 0.005:
                self.phase_done = True
                measured = estimator.measured_estimate(
                    min_samples=6,
                    condition_max=1e6,
                    min_rank=3,
                )
                assert measured is not None
                np.testing.assert_allclose(measured[0], self.lji_jacobian, atol=1e-9)
                return
            command, _raw = compute_dq_lji(
                j_lji=self.lji_jacobian,
                s_lji=self.features,
                damping=0.01,
                gain_u=0.45,
                gain_v=0.45,
                gain_z=0.45,
                max_dq_linear=0.015,
                max_dq_angle=0.015,
            )
            before = self.features.copy()
            self.features += self.lji_jacobian @ command
            estimator.push(command, self.features - before)
            self.commands.append(command)
        self.failed = True


def test_headless_look_aim_grasp_reaches_precontact_in_order() -> None:
    plant = _HeadlessPickPlant()
    phase_order: list[str] = []

    def begin(phase: PickWorkflowPhase) -> None:
        plant.phase_done = False
        phase_order.append(phase.label)

    result = run_pick_workflow(
        (
            PickWorkflowPhase("look", "look", plant.look),
            PickWorkflowPhase("aim", "acquire", plant.aim),
            PickWorkflowPhase("grasp", "grasp", plant.grasp),
        ),
        timeout_s=2.0,
        begin_phase=begin,
        wait_phase=lambda _label, _timeout: plant.phase_done,
        failed=lambda: plant.failed,
        cancelled=lambda: False,
    )

    assert result.success
    assert phase_order == ["look", "aim", "grasp"]
    assert plant.ready_pose is not None
    np.testing.assert_allclose(plant.object_world - plant.ready_pose, [0.2, 0.0, 0.0])
    assert float(np.linalg.norm(plant.uv)) <= 0.005
    assert abs(float(plant.features[2])) <= 0.003
    assert all(np.all(np.isfinite(command)) for command in plant.commands)
