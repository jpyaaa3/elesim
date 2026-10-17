"""Retain damping about the swing reference while avoiding motor drag."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from elesim_sim.robot.go2.pympc_controller import (
    PyMpcGenesisController,
    swing_damping_reference_torque,
)


def test_perfect_swing_tracking_has_no_uncommanded_damping_brake():
    jacobian = np.diag([.2, .15, .1])
    joint_velocity = np.array([1., -2., 3.])
    body_motion = np.array([.3, -.2, .1])
    velocity = body_motion+jacobian @ joint_velocity
    reference = swing_damping_reference_torque(jacobian, joint_velocity, velocity, velocity,
                                               damping=1.6, velocity_limits=np.full(3, 30.))
    # Passive .1 + active (.5+1.) retains the former total damping 1.6.
    actual_damping_load = -1.5*joint_velocity + reference - .1*joint_velocity
    np.testing.assert_allclose(actual_damping_load, 0., atol=1e-12)


def test_body_motion_is_removed_from_the_world_foot_velocity_reference():
    jacobian = np.diag([.2, .15, .1])
    body_motion = np.array([.3, -.2, .1])
    actual_joint_velocity = np.array([.5, -.4, .3])
    measured = body_motion+jacobian @ actual_joint_velocity
    desired = np.array([.7, .1, -.2])
    torque = swing_damping_reference_torque(jacobian, actual_joint_velocity, measured, desired,
                                           damping=1.6, velocity_limits=np.full(3, 30.))
    np.testing.assert_allclose(torque, 1.6*np.linalg.solve(jacobian, desired-body_motion))


def test_reference_obeys_urdf_velocity_limits_and_retains_null_space_damping():
    jacobian = np.diag([1., 1., 0.])
    result = swing_damping_reference_torque(jacobian, np.array([0., 0., 8.]),
        np.zeros(3), np.array([100., -100., 100.]), damping=1.6, velocity_limits=np.ones(3))
    np.testing.assert_allclose(result, [1.6, -1.6, 0.])


@pytest.mark.parametrize('bad', [np.array([np.nan, 1., 1.]), np.array([0., 1., 1.])])
def test_invalid_joint_velocity_limit_is_rejected(bad):
    with pytest.raises(ValueError):
        swing_damping_reference_torque(np.eye(3), np.zeros(3), np.zeros(3), np.zeros(3),
                                       damping=1.6, velocity_limits=bad)


def test_only_free_swing_reduces_passive_damping():
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    c._config = SimpleNamespace(joint_damping=.1, leg_kv_damping=1.)
    c._entity = SimpleNamespace(set_dofs_damping=Mock())
    c._leg_dof_idxs = list(range(6, 18))
    c._set_swing_damping(np.array([False, True, False, True]))
    args, kwargs = c._entity.set_dofs_damping.call_args
    np.testing.assert_allclose(args[0], [1.1]*3+[.1]*3+[1.1]*3+[.1]*3)
    assert kwargs['dofs_idx_local'] == list(range(6, 18))


def test_nearly_singular_joint_direction_retains_damping_without_reference_spike():
    torque = swing_damping_reference_torque(
        np.diag([.1, .1, 1e-9]), np.zeros(3), np.zeros(3), [0., 0., 1.],
        damping=1.6, velocity_limits=np.full(3, 30.),
    )
    np.testing.assert_array_equal(torque, np.zeros(3))


def test_velocity_limit_preserves_the_resolved_direction_at_a_singular_pose():
    torque = swing_damping_reference_torque(
        [[1., 1., 0.], [0., 1., 1.], [0., 0., 0.]], np.zeros(3),
        np.zeros(3), [100., 0., 0.], damping=1.6, velocity_limits=[1., 10., 10.],
    )
    np.testing.assert_allclose(np.dot([1., -1., 1.], torque), 0., atol=1e-12)
    np.testing.assert_allclose(torque, 1.6*np.array([1., .5, -.5]), atol=1e-12)


def test_rotating_world_axes_does_not_change_reference_joint_torque():
    from scipy.spatial.transform import Rotation
    rotation = Rotation.from_euler('xyz', [.3, -.2, .4]).as_matrix()
    jacobian = np.diag([.2, .15, .1])
    joint_velocity = np.array([.5, -.4, .3])
    measured, desired = np.array([.6, -.1, .2]), np.array([.8, .2, -.1])
    options = dict(damping=1.6, velocity_limits=np.full(3, 30.))
    plain = swing_damping_reference_torque(jacobian, joint_velocity, measured, desired, **options)
    rotated = swing_damping_reference_torque(rotation @ jacobian, joint_velocity,
        rotation @ measured, rotation @ desired, **options)
    np.testing.assert_allclose(rotated, plain, atol=1e-12)


def test_damping_is_updated_only_when_support_mode_changes_and_restored_after_swing():
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    c._config = SimpleNamespace(joint_damping=.1, leg_kv_damping=1.)
    c._entity = SimpleNamespace(set_dofs_damping=Mock(), set_dofs_kp=Mock(), set_dofs_kv=Mock())
    c._leg_dof_idxs = list(range(6, 18))
    c._set_torque_actuation()
    mask = np.array([False, True, True, False])
    c._set_swing_damping(mask)
    c._set_swing_damping(mask)
    assert c._entity.set_dofs_damping.call_count == 2
    mask[:] = False
    c._set_swing_damping(mask)
    assert c._entity.set_dofs_damping.call_count == 3
    np.testing.assert_allclose(c._entity.set_dofs_damping.call_args.args[0], np.full(12, 1.1))


def torque_controller():
    from workbench.tests.apps.sim.scenarios.go2.test_51_pympc_adapter import (
        _init_torque_controller, sample,
    )
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_torque_controller(c, elapsed_s=.32)
    c._dt = .01
    c._entity.set_dofs_damping = Mock()
    return c, sample()


@pytest.mark.parametrize('velocity_error', [0., .2])
def test_applied_swing_torque_damps_error_and_does_not_brake_reference_motion(velocity_error):
    from elesim_sim.robot.go2.pympc_controller import swing_foot_reference
    c, state = torque_controller()
    state.contacts[:] = 0.
    c._last_contacts[:] = 0.
    c._swing_starts[:] = [0., 0., .02]
    c._touchdowns[:] = [.1, 0., .025]
    c._contact_gait.swing_progress[:] = .4
    position, desired = swing_foot_reference(
        .4, c._swing_starts[0], c._touchdowns[0], duration_s=.16, touchdown_depth_m=.008,
    )
    body_velocity = np.array([.3, -.2, .1])
    joint_velocity = np.tile(desired-body_velocity+velocity_error, 4)
    measured = np.tile(desired+velocity_error, (4, 1))
    torque = c._torques(state, np.tile(position, (4, 1)), measured,
                       np.tile(np.eye(3), (4, 1, 1)), joint_velocity)
    # Include Genesis passive damping in the actual motor load.
    net = torque - .1 * joint_velocity
    np.testing.assert_allclose(net, -(20.+1.6)*velocity_error, atol=1e-12)
    np.testing.assert_allclose(c._entity.set_dofs_damping.call_args.args[0], np.full(12, .1))
    np.testing.assert_array_equal(c._forces, np.zeros((4, 3)))


@pytest.mark.parametrize('searching', [False, True])
def test_stance_and_touchdown_search_keep_their_existing_torque_and_damping(searching):
    results = []
    for backend in ('jax_mppi', 'acados'):
        c, state = torque_controller()
        c._solver.backend = backend
        state.contacts[:] = float(not searching)
        c._contact_gait.search[:] = searching
        c._last_contacts[:] = state.contacts[:, 0]
        c._touchdowns[:] = state.feet_world
        c._contact_gait.swing_progress[:] = 1.
        torque = c._torques(state, state.feet_world, np.zeros((4, 3)),
                           np.tile(np.eye(3), (4, 1, 1)), np.ones(12))
        results.append(torque)
        if backend == 'jax_mppi':
            np.testing.assert_allclose(c._entity.set_dofs_damping.call_args.args[0], np.full(12, 1.1))
    np.testing.assert_array_equal(*results)


def test_cpu_backend_keeps_existing_swing_damping():
    c, state = torque_controller()
    c._solver.backend = 'acados'
    state.contacts[:] = 0.
    c._last_contacts[:] = 0.
    c._swing_starts[:] = state.feet_world
    c._touchdowns[:] = state.feet_world
    c._contact_gait.swing_progress[:] = 1.
    torque = c._torques(state, state.feet_world, np.zeros((4, 3)),
                       np.tile(np.eye(3), (4, 1, 1)), np.ones(12))
    expected = np.full((4, 3), -.5)
    expected[:, 2] -= 8.
    np.testing.assert_allclose(torque.reshape(4, 3), expected)
    c._entity.set_dofs_damping.assert_not_called()
