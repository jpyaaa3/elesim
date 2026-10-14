from types import SimpleNamespace

import numpy as np
import pytest

from elesim_sim.robot.go2.initial_pose import (
    validate_initial_joint_positions,
    validate_initial_stand_pose,
)
from elesim_sim.robot.go2.locomotion.kinematics import GO2_STAND_Q


def entity():
    joints = {
        name: SimpleNamespace(desc=SimpleNamespace(
            init_qpos=np.array([0.0], dtype=np.float32),
            dofs_limit=np.array([[-3.0, 3.0]], dtype=np.float32),
        )) for name in GO2_STAND_Q
    }
    return SimpleNamespace(is_built=False, get_joint=joints.__getitem__), joints


def test_stand_validation_preserves_urdf_zero_reference():
    robot, joints = entity()
    validate_initial_stand_pose(robot)
    for name, value in GO2_STAND_Q.items():
        assert joints[name].desc.init_qpos.dtype == np.float32
        assert joints[name].desc.init_qpos[0] == 0


def test_ready_pose_validation_never_shifts_joint_limit_coordinates():
    robot, joints = entity()
    validate_initial_joint_positions(robot, {"FL_thigh_joint": 0.9, "FL_calf_joint": -1.8})
    assert joints["FL_thigh_joint"].desc.init_qpos[0] == 0
    assert joints["FL_calf_joint"].desc.init_qpos[0] == 0
    assert joints["FR_thigh_joint"].desc.init_qpos[0] == 0


@pytest.mark.parametrize("invalid", ["limits", "shape", "integer", "nan"])
def test_invalid_joint_leaves_all_descriptions_unchanged(invalid):
    robot, joints = entity()
    last = list(joints)[-1]
    if invalid == "limits":
        joints[last].desc.dofs_limit = np.array([[0.0, 0.1]])
    elif invalid == "shape":
        joints[last].desc.init_qpos = np.zeros(2)
    elif invalid == "integer":
        joints[last].desc.init_qpos = np.zeros(1, dtype=np.int64)
    else:
        joints[last].desc.dofs_limit = np.array([[np.nan, np.nan]])
    before = {name: joint.desc.init_qpos.copy() for name, joint in joints.items()}
    with pytest.raises(ValueError):
        validate_initial_stand_pose(robot)
    for name, joint in joints.items():
        np.testing.assert_array_equal(joint.desc.init_qpos, before[name])


def test_built_scene_is_never_mutated():
    robot, _ = entity()
    robot.is_built = True
    with pytest.raises(RuntimeError, match="before scene.build"):
        validate_initial_stand_pose(robot)


def test_ready_angle_and_joint_limits_share_the_urdf_reference():
    robot, joints = entity()
    calf = joints['FL_calf_joint'].desc
    calf.dofs_limit = np.array([[-2.7227, -.83776]])
    before = calf.dofs_limit.copy()
    validate_initial_stand_pose(robot)
    ready = -1.8
    raw_qpos = ready + calf.init_qpos[0]  # Genesis set_dofs_position contract
    assert calf.dofs_limit[0, 0] <= raw_qpos <= calf.dofs_limit[0, 1]
    np.testing.assert_array_equal(calf.dofs_limit, before)
