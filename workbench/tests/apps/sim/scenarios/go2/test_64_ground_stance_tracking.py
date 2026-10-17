"""Searching and supporting feet must use the same measured ground frame."""
from dataclasses import replace

import numpy as np
import pytest

from elesim_sim.robot.go2.locomotion.types import Go2Command
from workbench.tests.apps.sim.scenarios.go2.test_58_stance_tracking import controller
from workbench.tests.apps.sim.scenarios.go2.test_59_touchdown_tracking import JACOBIAN
from workbench.tests.apps.sim.scenarios.go2.test_53_pympc_gpu import cpu_math, standing


@pytest.mark.parametrize('body_motion', [[-.8, .1, .1], [.5, -.2, -.05], [0., 0., .1]])
@pytest.mark.parametrize('load', [1/6, 1.])
def test_contact_confirmation_does_not_turn_nominal_body_motion_into_an_unloading_torque(body_motion, load):
    c, state = controller()
    c._base_velocity_world = np.asarray(body_motion)
    c._cmd = Go2Command(vx=-.8)
    c._contact_gait.load[:] = load
    foot_error = np.array([0., .02, .03])
    joint_velocity = np.linalg.solve(JACOBIAN, foot_error-body_motion)
    torque = c._torques(state, state.feet_world, np.tile(foot_error, (4, 1)),
                       np.tile(JACOBIAN, (4, 1, 1)), np.tile(joint_velocity, 4))
    net = torque.reshape(4, 3)-1.1*joint_velocity
    expected = JACOBIAN.T@np.array([0., 0., -50.*load])-3.6*np.linalg.solve(JACOBIAN, foot_error)
    np.testing.assert_allclose(net, np.tile(expected, (4, 1)), atol=1e-10)


def test_stop_still_applies_the_mpc_braking_wrench_without_an_extra_motor_bias():
    c, state = controller()
    c._cmd = Go2Command()
    c._settling = True
    state = replace(state, desired_com_velocity=np.zeros(3))
    requested = np.tile([-10., 0., 40.], (4, 1))
    c._solver.solve = lambda _: requested
    velocity = np.linalg.solve(JACOBIAN, [-.3, 0., 0.])
    torque = c._torques(state, state.feet_world, np.zeros((4, 3)),
                       np.tile(JACOBIAN, (4, 1, 1)), np.tile(velocity, 4))
    actual = np.array([-np.linalg.solve(JACOBIAN.T, value-1.1*velocity)
                       for value in torque.reshape(4, 3)])
    np.testing.assert_allclose(actual, requested, atol=1e-10)
    np.testing.assert_array_equal(state.desired_com_velocity, np.zeros(3))


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
