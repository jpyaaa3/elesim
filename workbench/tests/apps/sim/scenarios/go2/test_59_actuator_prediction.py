"""Predict the first control step through actuator limits and both filters."""
from types import SimpleNamespace

import numpy as np
import pytest

from elesim_sim.robot.go2.mpc.actuator_prediction import build_actuator_prediction
from elesim_sim.robot.go2.mpc.contact_inverse_dynamics import contact_inverse_dynamics


def mechanics():
    rng = np.random.default_rng(161026)
    factor = rng.normal(size=(18, 18))*.1
    mass = factor.T @ factor + np.diag([16.]*3+[.5]*3+[.03]*12)
    jacobians = np.zeros((4, 3, 18))
    for i in range(4):
        jacobians[i, :, :3] = np.eye(3)
        jacobians[i, :, 6+3*i:9+3*i] = .2*np.eye(3)
    return SimpleNamespace(mass_matrix=mass, bias=rng.normal(size=18),
                           foot_jacobians=jacobians, foot_acceleration_bias=rng.normal(size=(4, 3))*.1)


@pytest.mark.parametrize('contacts', [[1, 1, 1, 1], [1, 0, 0, 1], [0, 0, 0, 0]])
@pytest.mark.parametrize('filtering', [False, True])
@pytest.mark.parametrize('clipping', [False, True])
def test_prediction_matches_independent_inverse_limit_filter_and_forward_solve(contacts, filtering, clipping):
    terms = mechanics()
    contacts = np.asarray(contacts, dtype=bool)
    force = np.tile([3., -2., 40.], (4, 1))*contacts[:, None]
    past_force = np.tile([-1., 2., 35.], (4, 1))*contacts[:, None]
    swing = np.linspace(-1., 1., 12)
    passive = -np.linspace(-.4, .6, 12)
    limits = np.full(12, 3. if clipping else 10000.)
    history = np.linspace(-2., 2., 12)
    force_alpha = .7 if filtering else 1.
    torque_alpha = np.where(np.repeat(contacts, 3), .5 if filtering else 1., 1.)
    model = build_actuator_prediction(terms, contacts, swing_torque=swing,
        passive_torque=passive, torque_limits=limits, previous_torque=history,
        torque_alpha=torque_alpha, previous_forces=past_force, force_alpha=force_alpha)
    prediction = model.predict(force)
    requested = force_alpha*force+(1.-force_alpha)*past_force
    inverse = contact_inverse_dynamics(terms.mass_matrix, terms.bias,
        terms.foot_jacobians, terms.foot_acceleration_bias, contacts, requested,
        swing_torque=swing, passive_torque=passive)
    limited = np.clip(inverse.torque, -limits, limits)
    applied = torque_alpha*limited+(1.-torque_alpha)*history
    np.testing.assert_allclose(prediction.requested_torque, inverse.torque, atol=1e-10)
    np.testing.assert_allclose(prediction.applied_torque, applied, atol=1e-10)
    j = terms.foot_jacobians[contacts].reshape(-1, 18)
    system = np.block([[terms.mass_matrix, -j.T], [j, np.zeros((len(j), len(j)))]])
    rhs = np.r_[np.r_[np.zeros(6), applied+passive]-terms.bias,
                -terms.foot_acceleration_bias[contacts].ravel()]
    actual = np.linalg.solve(system, rhs)
    np.testing.assert_allclose(prediction.angular_acceleration, actual[3:6], atol=1e-10)
    np.testing.assert_allclose(prediction.forces[contacts], actual[18:].reshape(-1, 3), atol=1e-10)
    np.testing.assert_array_equal(prediction.forces[~contacts], 0.)


def test_first_step_model_does_not_mutate_its_input_history():
    terms = mechanics()
    history = np.ones(12)
    force = np.tile([0., 0., 40.], (4, 1))
    model = build_actuator_prediction(terms, np.ones(4), swing_torque=np.zeros(12),
        passive_torque=np.zeros(12), torque_limits=np.full(12, 50.),
        previous_torque=history, torque_alpha=np.full(12, .5),
        previous_forces=force, force_alpha=.7)
    before = model.predict(force)
    history[:] = 999.
    force[:] = 999.
    after = model.predict(np.tile([0., 0., 40.], (4, 1)))
    np.testing.assert_array_equal(before.applied_torque, after.applied_torque)


@pytest.mark.parametrize('bad', [-1., np.nan, 1.1])
def test_force_filter_coefficient_is_bounded_and_finite(bad):
    with pytest.raises(ValueError, match='force_alpha'):
        build_actuator_prediction(mechanics(), np.ones(4), swing_torque=np.zeros(12),
            passive_torque=np.zeros(12), torque_limits=np.full(12, 50.),
            previous_torque=np.zeros(12), torque_alpha=np.ones(12),
            previous_forces=np.zeros((4, 3)), force_alpha=bad)
