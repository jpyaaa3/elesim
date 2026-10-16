"""Check prescribed ground forces against independent constrained dynamics."""
from __future__ import annotations

import numpy as np
import pytest

from elesim_sim.robot.go2.mpc.contact_inverse_dynamics import contact_inverse_dynamics


def mechanics():
    rng = np.random.default_rng(251016)
    factor = rng.normal(size=(18, 18)) * .1
    mass = factor.T @ factor + np.diag([16.] * 3 + [.5] * 3 + [.03] * 12)
    bias = rng.normal(size=18)
    jacobians = np.zeros((4, 3, 18))
    for i, position in enumerate(((.2, .15, -.3), (.2, -.15, -.3),
                                  (-.2, .15, -.3), (-.2, -.15, -.3))):
        x, y, z = position
        jacobians[i, :, :3] = np.eye(3)
        jacobians[i, :, 3:6] = [[0., z, -y], [-z, 0., x], [y, -x, 0.]]
        jacobians[i, :, 6+3*i:9+3*i] = np.diag([.25, .2, -.15])
    return mass, bias, jacobians


@pytest.mark.parametrize('contacts', [[1, 1, 1, 1], [1, 0, 0, 1],
                                    [0, 1, 0, 0], [0, 0, 0, 0]])
def test_contact_forces_survive_a_forward_dynamics_round_trip(contacts):
    mass, bias, jacobians = mechanics()
    contacts = np.asarray(contacts, dtype=bool)
    forces = np.tile([3., -2., 40.], (4, 1)) * contacts[:, None]
    jdot_velocity = np.arange(12, dtype=float).reshape(4, 3) * .01
    desired_acceleration = -np.ones((4, 3)) * .3
    swing_torque = np.linspace(-2., 2., 12)
    passive_torque = -np.linspace(-1., 1., 12) * 1.1
    result = contact_inverse_dynamics(
        mass, bias, jacobians, jdot_velocity, contacts, forces,
        swing_torque=swing_torque, passive_torque=passive_torque,
        contact_acceleration=desired_acceleration,
    )
    contact_j = jacobians[contacts].reshape(-1, 18)
    # Forward solve has unknown contact forces and uses the returned actuator
    # torques. It does not reuse the inverse solver's elimination matrix.
    count = 3 * np.count_nonzero(contacts)
    forward = np.block([[mass, -contact_j.T],
                        [contact_j, np.zeros((count, count))]])
    active = np.r_[np.zeros(6), result.torque + passive_torque]
    rhs = np.r_[active-bias, (desired_acceleration-jdot_velocity)[contacts].ravel()]
    solution = np.linalg.solve(forward, rhs)
    np.testing.assert_allclose(solution[:18], result.acceleration, atol=1e-10)
    np.testing.assert_allclose(solution[18:].reshape(-1, 3), forces[contacts], atol=1e-10)
    np.testing.assert_array_equal(result.torque.reshape(4, 3)[~contacts],
                                  swing_torque.reshape(4, 3)[~contacts])


def test_static_limit_contains_leg_gravity_and_compensates_only_known_passive_torque():
    mass, _, jacobians = mechanics()
    forces = np.tile([0., 0., 40.], (4, 1))
    external = jacobians.reshape(12, 18).T @ forces.ravel()
    gravity = np.tile([.1, .3, .5], 4)
    bias = np.r_[external[:6], gravity]
    passive = np.linspace(-.2, .2, 12)
    result = contact_inverse_dynamics(
        mass, bias, jacobians, np.zeros((4, 3)), np.ones(4), forces,
        swing_torque=np.zeros(12), passive_torque=passive,
    )
    np.testing.assert_allclose(result.acceleration, 0., atol=1e-11)
    np.testing.assert_allclose(result.torque, gravity-external[6:]-passive, atol=1e-11)


def test_joint_inertia_changes_the_required_torque():
    mass, bias, jacobians = mechanics()
    arguments = (bias, jacobians, np.full((4, 3), .1), np.ones(4),
                 np.tile([4., 0., 35.], (4, 1)))
    original = contact_inverse_dynamics(mass, *arguments, swing_torque=np.zeros(12),
                                        passive_torque=np.zeros(12))
    with_armature = mass.copy()
    with_armature[6:, 6:] += .01*np.eye(12)
    changed = contact_inverse_dynamics(with_armature, *arguments,
                                       swing_torque=np.zeros(12), passive_torque=np.zeros(12))
    assert np.max(np.abs(changed.torque-original.torque)) > .01


@pytest.mark.parametrize('invalid', ['air_force', 'negative_normal', 'nan', 'singular'])
def test_invalid_force_or_underdetermined_mechanics_fail_closed(invalid):
    mass, bias, jacobians = mechanics()
    contacts = np.array([1, 0, 0, 1])
    forces = np.tile([0., 0., 40.], (4, 1)) * contacts[:, None]
    if invalid == 'air_force':
        forces[1, 2] = .1
    elif invalid == 'negative_normal':
        forces[0, 2] = -1.
    elif invalid == 'nan':
        bias[0] = np.nan
    else:
        jacobians[:] = 0.
    with pytest.raises((ValueError, RuntimeError)):
        contact_inverse_dynamics(mass, bias, jacobians, np.zeros((4, 3)), contacts, forces,
                                 swing_torque=np.zeros(12), passive_torque=np.zeros(12))


def test_inputs_are_not_mutated():
    mass, bias, jacobians = mechanics()
    arrays = [mass, bias, jacobians, np.zeros((4, 3)), np.ones(4),
              np.tile([0., 0., 40.], (4, 1)), np.ones(12), np.zeros(12)]
    before = [a.copy() for a in arrays]
    contact_inverse_dynamics(*arrays[:6], swing_torque=arrays[6], passive_torque=arrays[7])
    for value, expected in zip(arrays, before):
        np.testing.assert_array_equal(value, expected)
