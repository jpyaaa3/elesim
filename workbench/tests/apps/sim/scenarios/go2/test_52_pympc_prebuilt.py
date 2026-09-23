"""Image-level check: the shipped solver must load and execute without codegen."""

from __future__ import annotations

import numpy as np
import pytest

from elesim_sim.robot.go2.pympc_solver import PyMpcForceSolver, PyMpcInput


def test_prebuilt_acados_solver_executes_one_nominal_step() -> None:
    pytest.importorskip("quadruped_pympc")
    pytest.importorskip("acados_template")
    solver = PyMpcForceSolver()
    feet = np.array(
        [[0.2, 0.1, 0.02], [0.2, -0.1, 0.02],
         [-0.2, 0.1, 0.02], [-0.2, -0.1, 0.02]],
        dtype=float,
    )
    state = PyMpcInput(
        com_position=np.array([0.0, 0.0, 0.3]),
        com_velocity=np.zeros(3),
        rpy=np.zeros(3),
        angular_velocity_body=np.zeros(3),
        feet_world=feet,
        desired_com_position=np.array([0.0, 0.0, 0.3]),
        desired_com_velocity=np.zeros(3),
        desired_rpy=np.zeros(3),
        desired_angular_velocity_body=np.zeros(3),
        footholds_world=feet.copy(),
        contacts=np.ones((4, solver.horizon)),
        mass_kg=15.019,
        inertia_body=np.diag([0.16, 0.47, 0.52]),
    )
    forces = solver.solve(state)
    assert forces.shape == (4, 3)
    assert np.all(np.isfinite(forces))
    assert np.sum(forces[:, 2]) > 0
