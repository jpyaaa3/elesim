"""Use the shipped model and independent Pinocchio algorithms."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from elesim_sim.robot.go2.mpc.pin_contact_dynamics import PinContactDynamics


@pytest.fixture
def robot():
    pin = pytest.importorskip('pinocchio')
    root = next(p for p in Path(__file__).resolve().parents if (p/'payload').is_dir())
    model = pin.buildModelFromUrdf(
        str(root/'payload/data/models/assemblies/zed-mini/assets/go2/go2.urdf'),
        pin.JointModelFreeFlyer())
    frames = [model.getFrameId(f'{leg}_foot') for leg in ('FL', 'FR', 'RL', 'RR')]
    q = pin.neutral(model)
    q[:3] = [.3, -.2, .32]
    q[3:7] = Rotation.from_euler('xyz', [.1, -.2, .3]).as_quat()
    q[7:] = np.tile([.1, .9, -1.8], 4)
    velocity = np.linspace(-.3, .4, model.nv)
    return pin, model, frames, q, velocity


def test_mass_bias_and_armature_match_rnea_without_mutating_controller_model(robot):
    pin, model, frames, q, velocity = robot
    before = np.asarray(model.armature).copy()
    dynamics = PinContactDynamics(model, frames, joint_armature=.01)
    terms = dynamics.terms(q, velocity)
    acceleration = np.linspace(-2., 2., 18)
    inverse = pin.rnea(dynamics.model, dynamics.model.createData(), q, velocity, acceleration)
    np.testing.assert_allclose(terms.mass_matrix @ acceleration + terms.bias, inverse, atol=1e-11)
    np.testing.assert_array_equal(model.armature, before)
    np.testing.assert_array_equal(dynamics.model.armature[6:], np.full(12, .01))


def test_classical_acceleration_bias_and_world_jacobian_match_velocity_derivative(robot):
    pin, model, frames, q, velocity = robot
    dynamics = PinContactDynamics(model, frames, joint_armature=.01)
    terms = dynamics.terms(q, velocity)
    acceleration = np.linspace(-.8, .6, 18)
    epsilon = 1e-6
    observed = []
    for sign in (-1., 1.):
        data = model.createData()
        pose = pin.integrate(model, q, sign*epsilon*velocity)
        pin.forwardKinematics(model, data, pose, velocity+sign*epsilon*acceleration)
        observed.append(np.array([pin.getFrameVelocity(model, data, f,
                                  pin.LOCAL_WORLD_ALIGNED).linear for f in frames]))
    derivative = (observed[1]-observed[0])/(2.*epsilon)
    predicted = np.einsum('fij,j->fi', terms.foot_jacobians, acceleration) + terms.foot_acceleration_bias
    np.testing.assert_allclose(predicted, derivative, atol=2e-9)


def test_rigid_payload_is_rotated_into_base_once_and_can_be_removed(robot):
    pin, model, frames, q, velocity = robot
    dynamics = PinContactDynamics(model, frames, joint_armature=.01)
    rotation = Rotation.from_quat(q[3:7]).as_matrix()
    payload = SimpleNamespace(mass_kg=.5779, com_world=q[:3]+rotation @ [.08, .02, .15],
                              inertia_world=rotation @ np.diag([.004, .005, .006]) @ rotation.T)
    original = model.inertias[1]
    for _ in range(2):
        dynamics.terms(q, velocity, payload=payload)
        expected = original + pin.Inertia(.5779, np.array([.08, .02, .15]), np.diag([.004, .005, .006]))
        np.testing.assert_allclose(dynamics.model.inertias[1].matrix(), expected.matrix(), atol=1e-12)
    dynamics.terms(q, velocity, payload=None)
    np.testing.assert_allclose(dynamics.model.inertias[1].matrix(), original.matrix(), atol=1e-12)
    np.testing.assert_allclose(model.inertias[1].matrix(), original.matrix(), atol=1e-12)


@pytest.mark.parametrize('contacts', [[1, 1, 1, 1], [1, 0, 0, 1], [0, 1, 1, 0]])
def test_real_model_inverse_torque_closes_base_and_contact_equations(robot, contacts):
    from elesim_sim.robot.go2.mpc.contact_inverse_dynamics import contact_inverse_dynamics
    pin, model, frames, q, velocity = robot
    dynamics = PinContactDynamics(model, frames, joint_armature=.01)
    terms = dynamics.terms(q, velocity)
    support = np.asarray(contacts, dtype=bool)
    force = np.tile([2., 1., 40.], (4, 1)) * support[:, None]
    passive = -1.1*velocity[6:]
    swing = -.5*velocity[6:]
    result = contact_inverse_dynamics(
        terms.mass_matrix, terms.bias, terms.foot_jacobians,
        terms.foot_acceleration_bias, support, force, swing_torque=swing,
        passive_torque=passive)
    inverse = pin.rnea(dynamics.model, dynamics.model.createData(), q, velocity, result.acceleration)
    generalized_force = terms.foot_jacobians.reshape(12, 18).T @ force.ravel()
    np.testing.assert_allclose(inverse-generalized_force,
                               np.r_[np.zeros(6), result.torque+passive], atol=1e-10)
    data = model.createData()
    pin.forwardKinematics(model, data, q, velocity, result.acceleration)
    actual = [pin.getFrameClassicalAcceleration(model, data, frames[i],
              pin.LOCAL_WORLD_ALIGNED).linear for i in np.flatnonzero(support)]
    np.testing.assert_allclose(actual, 0., atol=1e-11)
