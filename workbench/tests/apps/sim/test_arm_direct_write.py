"""Direct arm writes finish with one coherent position/velocity FK update."""
from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from elesim_sim.config import JointLimit, SimParam
from elesim_sim.runtime import SimMover


class _Entity:
    def __init__(self, claws: int, *, contiguous: bool = False) -> None:
        # Deliberate gaps model a merged floating GO2 and prevent a whole-entity
        # zero or a contiguous arm-range assumption from passing unnoticed.
        self.joints = {
            "linear": 18,
            "roll": 20,
            "bend_a": 22,
            "bend_b": 24,
        }
        if claws >= 1:
            self.joints["j_gripper_base_claw_left"] = 26
        if claws >= 2:
            self.joints["j_gripper_base_claw_right"] = 28
        if contiguous:
            self.joints = {name: 18 + i for i, name in enumerate(self.joints)}
        self.q = np.zeros(31)
        self.dq = np.arange(1.0, 32.0)
        self.links_q = self.q.copy()
        self.links_dq = self.dq.copy()
        self.calls: list[tuple] = []
        self.forward_count = 0

    def get_joint(self, name):
        return SimpleNamespace(dofs_idx_local=self.joints[name])

    def get_dofs_position(self, *, dofs_idx_local):
        return self.q[dofs_idx_local].copy()

    def set_dofs_position(self, values, *, dofs_idx_local, zero_velocity=True):
        self.calls.append(("position", tuple(dofs_idx_local), zero_velocity))
        self.q[dofs_idx_local] = values
        if zero_velocity:
            self.dq[:] = 0.0
        # Genesis 1.4.1 position FK refreshes both Cartesian pose and velocity.
        self.links_q = self.q.copy()
        self.links_dq = self.dq.copy()
        self.forward_count += 1

    def set_dofs_velocity(self, values, *, dofs_idx_local, skip_forward=False):
        self.calls.append(("velocity", tuple(dofs_idx_local), skip_forward))
        self.dq[dofs_idx_local] = values
        if not skip_forward:
            self.links_dq = self.dq.copy()
            self.forward_count += 1


@pytest.mark.parametrize("claws", (0, 1, 2))
@pytest.mark.parametrize("instant", (False, True))
@pytest.mark.parametrize("contiguous", (False, True))
def test_arm_and_claws_share_one_final_fk_without_clearing_go2_velocity(claws, instant, contiguous):
    entity = _Entity(claws, contiguous=contiguous)
    mover = SimMover(
        entity,
        SimParam(dt=0.01),
        JointLimit(-90.0, 90.0, 90.0),
        2,
        n_seg=1,
        linear_joint_name="linear",
        roll_joint_name="roll",
        bend_joint_names=["bend_a", "bend_b"],
    )
    entity.calls.clear()
    entity.forward_count = 0
    before_dq = entity.dq.copy()
    mover.set_claw_closed(True)
    command = mover.set_4dof_instant if instant else mover.control_4dof
    command(-0.1, 0.2, 0.3, -0.3)

    indices = list(entity.joints.values())
    assert entity.calls == [
        ("velocity", tuple(indices), True),
        ("position", tuple(indices), False),
    ]
    assert entity.forward_count == 1
    expected_dq = before_dq.copy()
    expected_dq[indices] = 0.0
    np.testing.assert_array_equal(entity.dq, expected_dq)
    np.testing.assert_array_equal(entity.links_dq, expected_dq)
    np.testing.assert_array_equal(entity.links_q, entity.q)
    np.testing.assert_array_equal(entity.q[indices[:4]], mover.get_last_command_full())
    if claws:
        assert entity.q[indices[4]] < 0.0
    if claws == 2:
        assert entity.q[indices[5]] > 0.0
