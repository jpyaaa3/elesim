"""A moving body must not make joint damping unload a landing foot."""
import numpy as np
import pytest

from elesim_sim.robot.go2.locomotion.types import Go2Command
from workbench.tests.apps.sim.scenarios.go2.test_58_stance_tracking import controller


# GO2 ready-pose geometry: x is controlled by thigh/calf, z by calf.
# The reverse body translation's raw joint drag has an upward component.
JACOBIAN = np.array([[0., -.265, -.1325], [.265, 0., 0.], [.0955, 0., -.1669]])


def search_controller(backend='jax_mppi', *, deep=False, settling=False):
    c, state = controller()
    c._cmd = Go2Command()  # Landing must also complete while braking to stop.
    c._solver.backend = backend
    c._settling = settling
    state.contacts[:] = 0.
    state.feet_world[:] = [0., 0., .023]
    c._last_contacts[:] = 0.
    c._contact_gait.search[:] = True
    c._contact_gait.load[:] = 0.
    if deep:
        c._contact_gait.swing_progress[:] = 1.
        c._touchdowns[:] = [0., 0., -.1]
    else:
        c._contact_gait.swing_progress[:] = .8
    return c, state


@pytest.mark.parametrize('settling', [False, True])
@pytest.mark.parametrize('foot_velocity', [np.zeros(3), np.array([0., .02, .03])])
def test_search_damps_ground_relative_motion_without_unloading_for_body_motion(foot_velocity, settling):
    c, state = search_controller(settling=settling)
    body_velocity = np.array([-.7, 0., 0.])
    joint_velocity = np.linalg.solve(JACOBIAN, foot_velocity-body_velocity)
    torque = c._torques(state, state.feet_world, np.tile(foot_velocity, (4, 1)),
                       np.tile(JACOBIAN, (4, 1, 1)), np.tile(joint_velocity, 4))
    task_force = np.array([0., 0., -4.5])-20.*foot_velocity
    error_velocity = np.linalg.solve(JACOBIAN, foot_velocity)
    expected = JACOBIAN.T@task_force-1.6*error_velocity
    net = torque.reshape(4, 3)-1.1*joint_velocity
    np.testing.assert_allclose(net, np.tile(expected, (4, 1)), atol=1e-12)
    np.testing.assert_array_equal(c._forces, np.zeros((4, 3)))
    np.testing.assert_array_equal(c._force_requested, np.zeros((4, 3)))


def test_stationary_search_foot_respects_the_existing_60_newton_task_limit():
    c, state = search_controller(deep=True)
    joint_velocity = np.linalg.solve(JACOBIAN, [.7, 0., 0.])
    torque = c._torques(state, state.feet_world, np.zeros((4, 3)),
                       np.tile(JACOBIAN, (4, 1, 1)), np.tile(joint_velocity, 4))
    net = torque.reshape(4, 3)-1.1*joint_velocity
    forces = np.array([np.linalg.solve(JACOBIAN.T, value) for value in net])
    np.testing.assert_allclose(forces, np.tile([0., 0., -60.], (4, 1)), atol=1e-10)


def test_cpu_search_retains_its_existing_motor_damping():
    c, state = search_controller('acados')
    velocity = np.linalg.solve(JACOBIAN, [.7, 0., 0.])
    torque = c._torques(state, state.feet_world, np.zeros((4, 3)),
                       np.tile(JACOBIAN, (4, 1, 1)), np.tile(velocity, 4))
    expected = JACOBIAN.T@np.array([0., 0., -4.5])-.5*velocity
    np.testing.assert_allclose(torque.reshape(4, 3), np.tile(expected, (4, 1)), atol=1e-12)


def test_search_keeps_its_original_passive_substep_damping():
    from unittest.mock import Mock
    c, state = search_controller()
    c._entity.set_dofs_damping = Mock()
    c._torques(state, state.feet_world, np.zeros((4, 3)), np.tile(JACOBIAN, (4, 1, 1)), np.zeros(12))
    np.testing.assert_allclose(c._entity.set_dofs_damping.call_args.args[0], np.full(12, 1.1))


def test_swing_probe_does_not_change_ground_relative_search_reference(monkeypatch):
    import elesim_sim.robot.go2.pympc_controller as module
    c, _ = search_controller()
    velocity = np.linalg.solve(JACOBIAN, [.7, 0., 0.])
    expected = c._search_damping_reference(JACOBIAN, velocity, np.zeros(3), np.full(3, 30.))
    monkeypatch.setattr(module, 'swing_damping_reference_torque', lambda *args, **kw: np.zeros(3))
    np.testing.assert_array_equal(c._search_damping_reference(
        JACOBIAN, velocity, np.zeros(3), np.full(3, 30.)), expected)
