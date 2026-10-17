"""Retain stance error damping without braking commanded body progress."""
from dataclasses import replace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from elesim_sim.robot.go2.locomotion.types import Go2Command
from elesim_sim.robot.go2.pympc_controller import (
    PyMpcGenesisController,
    commanded_progress_velocity,
    trailing_crouch_reference,
    landing_velocity_reference,
)
from workbench.tests.apps.sim.scenarios.go2.test_51_pympc_adapter import (
    _init_torque_controller, _torque_config, sample,
)


def controller():
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_torque_controller(c, elapsed_s=.32,
        config=_torque_config(aux_kv=2.))
    c._base_velocity_world = np.array([.3, .1, .2])
    c._stance_reference = np.zeros(12)
    c._dt = .01
    state = replace(sample(), rpy=np.zeros(3))
    state.contacts[:] = 1.
    c._last_contacts[:] = 1.
    return c, state


@pytest.mark.parametrize('actual,command,expected', [
    ([.3, .2, -.1], [.8, 0., 0.], [.3, 0., 0.]),
    ([-.3, .2, .1], [-.8, 0., 0.], [-.3, 0., 0.]),
    ([.3, -.2, .1], [0., -.35, 0.], [0., -.2, 0.]),
    ([.6, .1, .1], [.2, 0., 0.], [.2, 0., 0.]),
    ([-.3, .2, .1], [.8, 0., 0.], [0., 0., 0.]),
    ([.3, .2, .1], [0., 0., 0.], [0., 0., 0.]),
])
def test_reference_never_compensates_drift_opposite_motion_or_overspeed(actual, command, expected):
    np.testing.assert_allclose(commanded_progress_velocity(actual, command), expected)


@pytest.mark.parametrize('error', [np.zeros(3), np.array([0., .1, .2])])
def test_actual_stance_motor_load_preserves_error_damping_and_not_progress_drag(error):
    c, state = controller()
    velocity = np.tile(np.array([-.3, 0., 0.])+error, 4)
    torque = c._torques(state, state.feet_world, np.zeros((4, 3)),
                       np.tile(np.eye(3), (4, 1, 1)), velocity)
    net = torque-1.1*velocity  # Include the actual passive Genesis load.
    np.testing.assert_allclose(net.reshape(4, 3),
        np.tile([0., 0., -50.], (4, 1))-3.6*error, atol=1e-12)


@pytest.mark.parametrize('inactive', ['stopped', 'opposite', 'settling', 'cpu', 'turning'])
def test_stop_rotation_and_cpu_keep_the_existing_damping(inactive):
    c, state = controller()
    if inactive == 'stopped':
        c._cmd = Go2Command()
    elif inactive == 'opposite':
        c._base_velocity_world[0] = -.3
    elif inactive == 'settling':
        c._settling = True
    elif inactive == 'cpu':
        c._solver.backend = 'acados'
    elif inactive == 'turning':
        c._cmd = Go2Command(yaw_rate=.8)
    reference = c._stance_damping_reference(state, np.tile(np.eye(3), (4, 1, 1)))
    np.testing.assert_array_equal(reference, np.zeros(12))


def test_only_confirmed_stance_receives_the_ramped_reference():
    c, state = controller()
    state.contacts[1:3] = 0.
    c._contact_gait.search[1] = True
    c._contact_gait.load[:] = [1., 1., 1., .25]
    reference = c._stance_damping_reference(state, np.tile(np.eye(3), (4, 1, 1)))
    np.testing.assert_allclose(reference.reshape(4, 3),
        [[-1.08, 0., 0.], [0., 0., 0.], [0., 0., 0.], [-.27, 0., 0.]])


def test_world_heading_rotation_preserves_joint_reference_torque():
    c, state = controller()
    jacobians = np.tile(np.diag([.2, .15, .1]), (4, 1, 1))
    expected = c._stance_damping_reference(state, jacobians)
    r = Rotation.from_euler('z', .7).as_matrix()
    c._base_velocity_world = r@c._base_velocity_world
    turned = replace(state, rpy=np.array([0., 0., .7]))
    actual = c._stance_damping_reference(turned, r@jacobians)
    np.testing.assert_allclose(actual, expected, atol=1e-12)


def test_reference_still_passes_the_existing_final_actuator_limits():
    c, state = controller()
    c._tau_lim[:] = .5
    torque = c._torques(state, state.feet_world, np.zeros((4, 3)),
                       np.tile(np.eye(3)*.001, (4, 1, 1)), np.zeros(12))
    assert np.isfinite(torque).all()
    assert np.max(np.abs(torque)) <= .5


def test_swing_diagnostic_does_not_change_the_native_stance_reference(monkeypatch):
    import elesim_sim.robot.go2.pympc_controller as module
    c, state = controller()
    jacobians = np.tile(np.eye(3), (4, 1, 1))
    expected = c._stance_damping_reference(state, jacobians)
    assert np.any(expected)
    monkeypatch.setattr(module, 'swing_damping_reference_torque', lambda *args, **kw: np.zeros(3))
    np.testing.assert_array_equal(c._stance_damping_reference(state, jacobians), expected)


@pytest.mark.parametrize('actual,command', [([float('nan'), 0., 0.], [.8, 0., 0.]),
                                         ([.3, 0., 0.], [float('inf'), 0., 0.])])
def test_nonfinite_measured_or_commanded_progress_is_rejected(actual, command):
    with pytest.raises(ValueError):
        commanded_progress_velocity(actual, command)


@pytest.mark.parametrize('vx', [-.8, .8])
def test_body_reference_lowers_the_trailing_hips_by_only_one_centimeter(vx):
    state = replace(sample(), desired_com_position=np.array([0., 0., .3]),
                    desired_rpy=np.zeros(3))
    result = trailing_crouch_reference(state, command_vx=vx, crouch_m=.01)
    leading_x = np.sign(vx)*.1934
    assert result.desired_com_position[2]-leading_x*np.sin(result.desired_rpy[1]) == pytest.approx(.3)
    assert result.desired_com_position[2]+leading_x*np.sin(result.desired_rpy[1]) == pytest.approx(.29)
    assert result.contacts is state.contacts
    np.testing.assert_array_equal(state.desired_rpy, np.zeros(3))


@pytest.mark.parametrize('vx', [0., -.1, .1, .8])
def test_native_reference_matches_the_validated_research_intervention(vx):
    from workbench.research.debug.profile_sim_walking import (
        trailing_crouch_reference as prototype_pose,
        landing_velocity_reference as prototype_velocity,
    )
    c, state = controller()
    c._cmd = Go2Command(vx=vx)
    c._contact_gait.blocked = True
    c._solver.horizon = 12
    expected = prototype_pose(state, command_vx=vx, crouch_m=.01)
    expected = prototype_velocity(expected, blocked=True,
        actual_velocity=c._base_velocity_world, command_world=[vx, 0., 0.], horizon_s=.12)
    actual = c._walking_reference(state)
    np.testing.assert_allclose(actual.desired_com_position, expected.desired_com_position)
    np.testing.assert_allclose(actual.desired_com_velocity, expected.desired_com_velocity)
    np.testing.assert_allclose(actual.desired_rpy, expected.desired_rpy)
    assert actual.contacts is state.contacts


def test_cpu_keeps_its_original_body_and_landing_reference():
    c, state = controller()
    c._solver.backend = 'acados'
    assert c._walking_reference(state) is state


def test_landing_wait_cannot_create_unsupported_acceleration_or_new_contacts():
    _, state = controller()
    changed = landing_velocity_reference(state, blocked=True,
        actual_velocity=[.3, .1, .2], command_world=[.8, 0., 0.], horizon_s=.12)
    np.testing.assert_allclose(changed.desired_com_velocity, [.3, 0., 0.])
    assert changed.contacts is state.contacts
    assert changed.support_loads is state.support_loads
    np.testing.assert_array_equal(changed.desired_angular_velocity_body, state.desired_angular_velocity_body)
