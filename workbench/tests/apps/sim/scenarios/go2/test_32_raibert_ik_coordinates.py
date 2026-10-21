from types import SimpleNamespace

import numpy as np
import pytest

from elesim_sim.robot.go2.locomotion.config import Go2LocomotionConfig
from elesim_sim.robot.go2.locomotion.controller import RaibertTrotController
from elesim_sim.robot.go2.locomotion.kinematics import GO2_STAND_Q
from elesim_sim.robot.go2.locomotion.types import ALL_LEGS


class _Entity:
    def __init__(self, *, floating=True, tuple_result=False, invalid_result=None):
        root_dofs, root_qs = (6, 7) if floating else (0, 0)
        self.n_dofs = root_dofs + 14  # Twelve leg motors plus two arm joints.
        self.n_qs = root_qs + 14
        self.joints = {
            name: SimpleNamespace(
                dofs_idx_local=[root_dofs + index],
                qs_idx_local=[root_qs + index],
            )
            for index, name in enumerate(GO2_STAND_Q)
        }
        self.qpos = np.zeros(self.n_qs)
        if floating:
            self.qpos[3] = 1.0
        self.qpos[root_qs:root_qs + 12] = list(GO2_STAND_Q.values())
        self.qpos[-2:] = [2.4, -2.1]
        self.tuple_result = tuple_result
        self.invalid_result = invalid_result
        self.commands = []

    def get_joint(self, name):
        return self.joints[name]

    def get_link(self, name):
        return SimpleNamespace(
            get_pos=lambda: np.array([0., 0., .4]),
            get_quat=lambda: np.array([1., 0., 0., 0.]),
        )

    def set_dofs_kp(self, *args, **kwargs):
        pass

    def set_dofs_kv(self, *args, **kwargs):
        pass

    def set_dofs_position(self, *args, **kwargs):
        pass

    def control_dofs_position(self, positions, *, dofs_idx_local):
        self.commands.append((np.asarray(positions).copy(), list(dofs_idx_local)))

    def inverse_kinematics(self, **kwargs):
        # Genesis returns the whole configuration, including the four-component
        # root quaternion, even when IK is restricted to three leg DOFs.
        result = self.qpos.copy() if self.invalid_result is None else self.invalid_result
        return (result, np.zeros(6)) if self.tuple_result else result


@pytest.mark.parametrize('floating', [False, True])
@pytest.mark.parametrize('tuple_result', [False, True])
def test_idle_ik_preserves_standing_pose_with_arm_and_floating_root(floating, tuple_result):
    entity = _Entity(floating=floating, tuple_result=tuple_result)
    controller = RaibertTrotController(entity, dt=.01, config=Go2LocomotionConfig())
    np.testing.assert_allclose(
        np.concatenate([controller._leg_q_des[leg] for leg in ALL_LEGS]),
        list(GO2_STAND_Q.values()),
    )

    controller.step()  # No user command: target feet are already at their goals.

    positions, dofs = entity.commands[-1]
    np.testing.assert_allclose(positions, list(GO2_STAND_Q.values()))
    assert dofs == controller._kin.all_leg_dof_idx


@pytest.mark.parametrize('result', [np.array([.1, .8, -1.5]), np.full(21, np.nan)])
def test_invalid_ik_configuration_keeps_known_stand_targets(result):
    entity = _Entity(invalid_result=result)
    controller = RaibertTrotController(entity, dt=.01, config=Go2LocomotionConfig())
    np.testing.assert_allclose(
        np.concatenate([controller._leg_q_des[leg] for leg in ALL_LEGS]),
        list(GO2_STAND_Q.values()),
    )

    controller.step()

    np.testing.assert_allclose(entity.commands[-1][0], list(GO2_STAND_Q.values()))


def test_idle_joint_reference_does_not_follow_load_induced_leg_deflection():
    entity = _Entity()
    controller = RaibertTrotController(entity, dt=.01, config=Go2LocomotionConfig())
    # Feet remain planted as gravity compresses the legs. Re-solving their
    # current world positions would move the PD target with the deflection,
    # removing the restoring position error needed to support the body.
    entity.qpos[7:19] += .15

    controller.step()

    np.testing.assert_allclose(entity.commands[-1][0], list(GO2_STAND_Q.values()))
