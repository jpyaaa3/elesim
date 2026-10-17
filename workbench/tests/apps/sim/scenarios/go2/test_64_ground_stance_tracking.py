"""Keep CPU damping and MPC braking after rejected stance-frame experiments."""
from dataclasses import replace

import numpy as np

from workbench.tests.apps.sim.scenarios.go2.test_58_stance_tracking import controller
from workbench.tests.apps.sim.scenarios.go2.test_59_touchdown_tracking import JACOBIAN
from workbench.tests.apps.sim.scenarios.go2.test_53_pympc_gpu import cpu_math, standing


def test_cpu_retains_the_existing_joint_velocity_damping():
    c, state = controller()
    c._solver.backend = 'acados'
    velocity = np.linalg.solve(JACOBIAN, [.8, -.1, -.1])
    torque = c._torques(state, state.feet_world, np.zeros((4, 3)),
                       np.tile(JACOBIAN, (4, 1, 1)), np.tile(velocity, 4))
    expected = JACOBIAN.T@np.array([0., 0., -50.])-3.6*velocity
    np.testing.assert_allclose(torque.reshape(4, 3)-1.1*velocity,
                               np.tile(expected, (4, 1)), atol=1e-10)


def test_zero_velocity_request_keeps_the_optimizers_body_braking(cpu_math):
    from elesim_sim.robot.go2.pympc_solver import PyMpcForceSolver
    state = replace(standing(), com_velocity=np.array([.5, 0., 0.]),
                    desired_com_velocity=np.zeros(3), support_force_floor_n=10.)
    forces = PyMpcForceSolver(backend='jax_mppi', solver_factory=lambda: cpu_math).solve(state)
    assert np.sum(forces[:, 0]) < -1.
    assert np.all(forces[:, 2] >= 0.)
