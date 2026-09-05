from types import SimpleNamespace

import numpy as np
import pytest

from elesim_sim.robot.go2.locomotion.kinematics import GO2_READY_Q
from elesim_sim.robot.go2.mpc.payload_model import ArmPayloadCompensator
from elesim_sim.runtime import _set_go2_initial_leg_pose


class _Go2Entity:
    def __init__(self, missing: str = "") -> None:
        self.joints = {
            name: SimpleNamespace(dofs_idx_local=[index])
            for index, name in enumerate(GO2_READY_Q)
            if name != missing
        }
        self.position = None
        self.control = None

    def get_joint(self, name):
        return self.joints[name]

    def set_dofs_position(self, value, *, dofs_idx_local):
        self.position = (np.asarray(value), list(dofs_idx_local))

    def control_dofs_position(self, value, *, dofs_idx_local):
        self.control = (np.asarray(value), list(dofs_idx_local))


def test_initial_pose_requires_every_named_go2_joint() -> None:
    with pytest.raises(ValueError, match="FL_calf_joint"):
        _set_go2_initial_leg_pose(_Go2Entity(missing="FL_calf_joint"))


def test_initial_pose_sets_all_twelve_dofs() -> None:
    entity = _Go2Entity()
    _set_go2_initial_leg_pose(entity)
    assert entity.position is not None and entity.position[1] == list(range(12))
    assert entity.control is not None and entity.control[1] == list(range(12))


def test_explicit_payload_links_must_exist() -> None:
    entity = SimpleNamespace(get_link=lambda name: (_ for _ in ()).throw(KeyError(name)))
    with pytest.raises(ValueError, match="missing_link"):
        ArmPayloadCompensator(entity, link_names={"missing_link"})
