"""A confirmed foot's motor damping must not reverse its support request."""
from dataclasses import replace

import numpy as np
import pytest

from workbench.tests.apps.sim.scenarios.go2.test_58_stance_tracking import controller
from workbench.tests.apps.sim.scenarios.go2.test_59_touchdown_tracking import JACOBIAN
from elesim_sim.robot.go2.pympc_controller import support_damping_floor_torque


@pytest.mark.parametrize('load', [1/6, .5, 1.])
def test_stance_keeps_the_solved_normal_floor_after_actual_motor_damping(load):
    c, state = controller()
    state = replace(state, support_force_floor_n=10.)
    c._base_velocity_world = np.array([0., 0., .1])
    c._contact_gait.load[:] = load
    force = np.tile([0., 0., 10./load], (4, 1))
    c._solver.solve = lambda _: force
    velocity = np.linalg.solve(JACOBIAN, -c._base_velocity_world)
    torque = c._torques(state, state.feet_world, np.zeros((4, 3)),
                       np.tile(JACOBIAN, (4, 1, 1)), np.tile(velocity, 4))
    actual = np.array([np.linalg.solve(JACOBIAN.T, t-1.1*velocity)
                       for t in torque.reshape(4, 3)])
    old_damping = np.linalg.solve(JACOBIAN.T, -3.6*velocity)
    assert old_damping[2] > 10.
    np.testing.assert_allclose(actual[:, 2], -10., atol=1e-10)
    np.testing.assert_allclose(actual[:, :2], np.tile(old_damping[:2], (4, 1)), atol=1e-10)
    np.testing.assert_array_equal(c._forces, force)
    np.testing.assert_array_equal(c._force_requested, force)


@pytest.mark.parametrize('backend,floor', [('acados', 10.), ('jax_mppi', 0.)])
def test_cpu_and_disabled_floor_keep_the_original_damping(backend, floor):
    c, state = controller()
    c._solver.backend = backend
    c._base_velocity_world = np.array([0., 0., .1])
    state = replace(state, support_force_floor_n=floor)
    c._solver.solve = lambda _: np.tile([0., 0., 10.], (4, 1))
    velocity = np.linalg.solve(JACOBIAN, -c._base_velocity_world)
    torque = c._torques(state, state.feet_world, np.zeros((4, 3)),
                       np.tile(JACOBIAN, (4, 1, 1)), np.tile(velocity, 4))
    expected = JACOBIAN.T@np.array([0., 0., -10.])-3.6*velocity
    np.testing.assert_allclose(torque.reshape(4, 3)-1.1*velocity,
                               np.tile(expected, (4, 1)), atol=1e-12)


def test_sufficient_normal_support_keeps_all_existing_error_damping():
    c, state = controller()
    c._base_velocity_world = np.array([0., 0., .1])
    state = replace(state, support_force_floor_n=10.)
    velocity = np.linalg.solve(JACOBIAN, -c._base_velocity_world)
    torque = c._torques(state, state.feet_world, np.zeros((4, 3)),
                       np.tile(JACOBIAN, (4, 1, 1)), np.tile(velocity, 4))
    expected = JACOBIAN.T@np.array([0., 0., -50.])-3.6*velocity
    np.testing.assert_allclose(torque.reshape(4, 3)-1.1*velocity,
                               np.tile(expected, (4, 1)), atol=1e-12)


def test_floor_does_not_add_support_to_searching_feet():
    from workbench.tests.apps.sim.scenarios.go2.test_59_touchdown_tracking import search_controller
    c, state = search_controller()
    state = replace(state, support_force_floor_n=10.)
    torque = c._torques(state, state.feet_world, np.zeros((4, 3)),
                       np.tile(JACOBIAN, (4, 1, 1)), np.zeros(12))
    expected = JACOBIAN.T@np.array([0., 0., -4.5])
    np.testing.assert_allclose(torque.reshape(4, 3), np.tile(expected, (4, 1)), atol=1e-12)
    np.testing.assert_array_equal(c._forces, np.zeros((4, 3)))


def test_bias_compensation_is_bounded_and_has_no_tangential_component():
    correction = support_damping_floor_torque(JACOBIAN, JACOBIAN.T@[4., -3., 1000.],
        np.zeros(3), passive_damping=1.1, minimum_force_n=10.)
    np.testing.assert_allclose(np.linalg.solve(JACOBIAN.T, correction), [0., 0., -60.], atol=1e-10)


def test_rank_deficient_geometry_does_not_invent_a_normal_force():
    correction = support_damping_floor_torque(np.diag([1., 1., 0.]), np.ones(3),
        np.ones(3), passive_damping=1.1, minimum_force_n=10.)
    np.testing.assert_array_equal(correction, np.zeros(3))


def test_final_motor_limits_still_bound_the_floor_compensation():
    c, state = controller()
    c._tau_lim[:] = .5
    c._base_velocity_world = np.array([0., 0., .1])
    state = replace(state, support_force_floor_n=10.)
    velocity = np.linalg.solve(JACOBIAN, -c._base_velocity_world)
    torque = c._torques(state, state.feet_world, np.zeros((4, 3)),
                       np.tile(JACOBIAN, (4, 1, 1)), np.tile(velocity, 4))
    assert np.isfinite(torque).all()
    assert np.max(np.abs(torque)) <= .5


@pytest.mark.parametrize('field,value', [('passive_damping', -1.),
    ('minimum_force_n', float('nan')), ('correction_limit_n', 0.)])
def test_nonfinite_or_invalid_floor_input_is_rejected(field, value):
    options = dict(passive_damping=1.1, minimum_force_n=10., correction_limit_n=60.)
    options[field] = value
    with pytest.raises(ValueError):
        support_damping_floor_torque(np.eye(3), np.zeros(3), np.zeros(3), **options)
