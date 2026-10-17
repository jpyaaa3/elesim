"""Preserve established support feedback while matching a fresh landing."""
import numpy as np
import pytest

from elesim_sim.robot.go2.locomotion.types import Go2Command
from elesim_sim.robot.go2.pympc_controller import commanded_progress_velocity
from workbench.tests.apps.sim.scenarios.go2.test_58_stance_tracking import controller
from workbench.tests.apps.sim.scenarios.go2.test_59_touchdown_tracking import JACOBIAN


@pytest.mark.parametrize('body_motion', [[-.8, .1, .1], [.5, -.2, -.05], [0., 0., .1]])
@pytest.mark.parametrize('settling', [False, True])
def test_fully_loaded_feet_keep_the_previous_bounded_body_feedback(body_motion, settling):
    c, state = controller()
    c._base_velocity_world = np.array(body_motion)
    c._cmd = Go2Command(vx=-.8)
    c._settling = settling
    velocity = np.linalg.solve(JACOBIAN, -np.array(body_motion))
    actual = c._stance_damping_reference(state, np.tile(JACOBIAN, (4, 1, 1)),
        joint_velocity=np.tile(velocity, 4), foot_velocity=np.zeros((4, 3)))
    progress = np.zeros(3) if settling else commanded_progress_velocity(body_motion, [-.8, 0., 0.])
    expected = 3.6*np.linalg.solve(JACOBIAN, -progress)
    np.testing.assert_allclose(actual.reshape(4, 3), np.tile(expected, (4, 1)), atol=1e-10)


def test_new_landing_does_not_remove_body_feedback_from_the_other_three_feet():
    c, state = controller()
    body_motion = np.array([.3, .1, .1])
    c._base_velocity_world = body_motion
    c._contact_gait.load[0] = 2/3
    velocity = np.linalg.solve(JACOBIAN, -body_motion)
    actual = c._stance_damping_reference(state, np.tile(JACOBIAN, (4, 1, 1)),
        joint_velocity=np.tile(velocity, 4), foot_velocity=np.zeros((4, 3)))
    expected = np.tile(3.6*np.linalg.solve(JACOBIAN, [-.3, 0., 0.]), (4, 1))
    expected[0] = 3.6*velocity
    np.testing.assert_allclose(actual.reshape(4, 3), expected, atol=1e-10)


def test_return_from_landing_still_uses_the_existing_torque_filter():
    c, state = controller()
    c._config.tau_filter_alpha = .25
    c._base_velocity_world = np.array([.3, .1, .1])
    velocity = np.linalg.solve(JACOBIAN, -c._base_velocity_world)
    c._contact_gait.load[:] = 5/6
    previous = c._torques(state, state.feet_world, np.zeros((4, 3)),
        np.tile(JACOBIAN, (4, 1, 1)), np.tile(velocity, 4))
    c._contact_gait.load[:] = 1.
    c._step_i = 1
    result = c._torques(state, state.feet_world, np.zeros((4, 3)),
        np.tile(JACOBIAN, (4, 1, 1)), np.tile(velocity, 4))
    np.testing.assert_allclose(result, .25*c._tau_limited+.75*previous, atol=1e-10)
    assert not np.allclose(result, c._tau_limited)
