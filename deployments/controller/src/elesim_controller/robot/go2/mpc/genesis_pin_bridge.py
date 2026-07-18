from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from elesim_controller.simulation.genesis.utils import quat_wxyz_to_xyzw as _quat_wxyz_to_xyzw
from elesim_controller.simulation.genesis.utils import to_numpy_1d as _to_numpy_1d


class GenesisPinBridge:
    """Sync Genesis GO2 entity state into Pinocchio PinGo2Model q/dq vectors."""

    def __init__(self, entity, leg_dof_idxs: list[int], *, payload=None) -> None:
        if len(leg_dof_idxs) != 12:
            raise ValueError(f"expected 12 leg dofs, got {len(leg_dof_idxs)}")
        self._entity = entity
        self._leg_dof_idxs = [int(i) for i in leg_dof_idxs]
        self._payload = payload

    def read_pin_q_dq(self) -> tuple[np.ndarray, np.ndarray]:
        base = self._entity.get_link("base")
        pos = _to_numpy_1d(base.get_pos())[:3]
        quat_xyzw = _quat_wxyz_to_xyzw(_to_numpy_1d(base.get_quat())[:4])
        leg_q = _to_numpy_1d(self._entity.get_dofs_position(dofs_idx_local=self._leg_dof_idxs))
        q = np.concatenate([pos, quat_xyzw, leg_q])

        vel_world = _to_numpy_1d(base.get_vel())[:3]
        ang_world = _to_numpy_1d(base.get_ang())[:3]
        rot = Rot.from_quat(quat_xyzw)
        vel_body = rot.inv().apply(vel_world)
        ang_body = rot.inv().apply(ang_world)
        leg_dq = _to_numpy_1d(self._entity.get_dofs_velocity(dofs_idx_local=self._leg_dof_idxs))
        dq = np.concatenate([vel_body, ang_body, leg_dq])
        return q, dq

    def sync_pin_model(self, pin_model) -> None:
        q, dq = self.read_pin_q_dq()
        pin_model.update_model(q, dq)
        if self._payload is not None:
            self._payload.apply(pin_model)
