from types import SimpleNamespace

import pytest

from elesim_sim.robot.go2.locomotion.kinematics import Go2KinematicsModel


class _Entity:
    def __init__(self, missing: str = "") -> None:
        names = (
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        )
        self._joints = {
            name: SimpleNamespace(dofs_idx_local=[index])
            for index, name in enumerate(names)
            if name != missing
        }

    def get_joint(self, name):
        return self._joints[name]


def test_kinematics_requires_all_twelve_named_leg_dofs() -> None:
    model = Go2KinematicsModel.from_entity(_Entity())
    assert model.all_leg_dof_idx == list(range(12))


def test_kinematics_reports_missing_joint_instead_of_skipping_it() -> None:
    with pytest.raises(ValueError, match="RR_calf_joint"):
        Go2KinematicsModel.from_entity(_Entity(missing="RR_calf_joint"))
