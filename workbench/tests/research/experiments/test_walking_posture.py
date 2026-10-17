"""Keep the leading hips level while testing a bounded trailing crouch."""
from dataclasses import dataclass

import numpy as np
import pytest

from workbench.research.debug.profile_sim_walking import trailing_crouch_reference
from workbench.research.debug.profile_sim_walking import swing_reference_for_axes
from workbench.research.debug.profile_sim_walking import commanded_progress_velocity
from workbench.research.debug.profile_sim_walking import landing_velocity_reference


@dataclass(frozen=True)
class Reference:
    desired_com_position: np.ndarray
    desired_rpy: np.ndarray


@pytest.mark.parametrize('vx', [.8, -.8])
@pytest.mark.parametrize('drop', [.01, .02])
def test_crouch_lowers_only_the_trailing_hip_reference(vx, drop):
    sample = Reference(np.array([1., 2., .3]), np.array([0., 0., .5]))
    changed = trailing_crouch_reference(sample, command_vx=vx, crouch_m=drop)
    height = changed.desired_com_position[2]
    pitch = changed.desired_rpy[1]
    leading_x = np.sign(vx)*.1934
    assert height-leading_x*np.sin(pitch) == pytest.approx(.3)
    assert height+leading_x*np.sin(pitch) == pytest.approx(.3-drop)
    np.testing.assert_array_equal(sample.desired_com_position, [1., 2., .3])
    np.testing.assert_array_equal(sample.desired_rpy, [0., 0., .5])
    np.testing.assert_array_equal(changed.desired_com_position[:2], [1., 2.])
    assert changed.desired_rpy[2] == .5


@pytest.mark.parametrize('vx,drop', [(.8, 0.), (0., .02)])
def test_disabled_or_stopped_reference_is_unchanged(vx, drop):
    sample = Reference(np.array([0., 0., .3]), np.zeros(3))
    assert trailing_crouch_reference(sample, command_vx=vx, crouch_m=drop) is sample


def test_crouch_fades_continuously_with_the_shaped_command():
    sample = Reference(np.array([0., 0., .3]), np.zeros(3))
    a = trailing_crouch_reference(sample, command_vx=.1, crouch_m=.02)
    b = trailing_crouch_reference(sample, command_vx=.2, crouch_m=.02)
    assert .3-a.desired_com_position[2] == pytest.approx(.5*(.3-b.desired_com_position[2]))


@pytest.mark.parametrize('vx,drop', [(float('nan'), .02), (.8, -.01), (.8, .04), (.8, float('inf'))])
def test_invalid_diagnostic_reference_is_rejected(vx, drop):
    with pytest.raises(ValueError):
        trailing_crouch_reference(Reference(np.zeros(3), np.zeros(3)), command_vx=vx, crouch_m=drop)


def test_vertical_intervention_only_cancels_vertical_reference_drag():
    from elesim_sim.robot.go2.pympc_controller import swing_damping_reference_torque
    jacobian = np.array([[.2, .03, .0], [.01, .15, .04], [.0, .04, .12]])
    joint_velocity = np.array([1., -.2, .3])
    base_velocity = np.array([.3, .1, -.05])
    measured = base_velocity+jacobian@joint_velocity
    desired = np.array([.8, -.2, -.4])
    torque = swing_reference_for_axes(swing_damping_reference_torque,
        jacobian, joint_velocity, measured, desired,
        axes='vertical', scale=1., damping=1.6, velocity_limits=np.full(3, 30.))
    np.testing.assert_allclose(jacobian@(torque/1.6), [0., 0., -.35], atol=1e-12)


def test_all_axes_intervention_reproduces_the_installed_reference():
    from elesim_sim.robot.go2.pympc_controller import swing_damping_reference_torque
    args = (np.eye(3), np.ones(3), np.array([2., 3., 4.]), np.array([.8, 0., -.4]))
    options = dict(damping=1.6, velocity_limits=np.full(3, 30.))
    expected = swing_damping_reference_torque(*args, **options)
    actual = swing_reference_for_axes(swing_damping_reference_torque, *args,
                                     axes='all', scale=.5, **options)
    np.testing.assert_allclose(actual, .5*expected)


@pytest.mark.parametrize('actual,command,expected', [
    ([.3, .2, -.1], [.8, 0., 0.], [.3, 0., 0.]),
    ([-.3, .2, .1], [-.8, 0., 0.], [-.3, 0., 0.]),
    ([.3, -.2, .1], [0., -.35, 0.], [0., -.2, 0.]),
    ([.6, .1, .1], [.2, 0., 0.], [.2, 0., 0.]),
    ([-.3, .2, .1], [.8, 0., 0.], [0., 0., 0.]),
    ([.3, .2, .1], [0., 0., 0.], [0., 0., 0.]),
])
def test_stance_reference_only_relaxes_drag_along_existing_commanded_progress(actual, command, expected):
    np.testing.assert_allclose(commanded_progress_velocity(actual, command), expected)


def test_stance_reference_retains_vertical_and_sideways_joint_error_damping():
    from elesim_sim.robot.go2.pympc_controller import swing_damping_reference_torque
    jacobian = np.array([[.2, .03, .0], [.01, .15, .04], [.0, .04, .12]])
    progress = commanded_progress_velocity([.3, .2, -.1], [.8, 0., 0.])
    reference = swing_damping_reference_torque(jacobian, np.zeros(3), progress, np.zeros(3),
        damping=3.6, velocity_limits=np.full(3, 30.))
    qref = np.linalg.solve(jacobian, [-.3, 0., 0.])
    error = np.linalg.solve(jacobian, [0., -.2, .1])
    net = -3.6*(qref+error)+reference
    np.testing.assert_allclose(net, -3.6*error, atol=1e-12)


@dataclass(frozen=True)
class MotionReference:
    com_position: np.ndarray
    desired_com_position: np.ndarray
    desired_com_velocity: np.ndarray
    contacts: np.ndarray


def test_landing_wait_retains_existing_progress_without_accelerating_or_changing_contacts():
    sample = MotionReference(np.array([1., 2., .3]), np.array([1., 2., .28]),
                             np.zeros(3), np.eye(4))
    changed = landing_velocity_reference(sample, blocked=True,
        actual_velocity=[.3, .1, .2], command_world=[.8, 0., 0.], horizon_s=.12)
    np.testing.assert_allclose(changed.desired_com_velocity, [.3, 0., 0.])
    np.testing.assert_allclose(changed.desired_com_position, [1.036, 2., .28])
    assert changed.contacts is sample.contacts
    np.testing.assert_array_equal(sample.desired_com_velocity, np.zeros(3))


def test_landing_intervention_is_inactive_when_the_gait_is_not_blocked():
    sample = MotionReference(np.zeros(3), np.zeros(3), np.ones(3), np.eye(4))
    assert landing_velocity_reference(sample, blocked=False, actual_velocity=[.3, 0., 0.],
        command_world=[.8, 0., 0.], horizon_s=.12) is sample
