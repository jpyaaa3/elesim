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
