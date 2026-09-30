from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from elesim_sim.simulation.genesis.utils import quat_wxyz_to_xyzw as _quat_wxyz_to_xyzw
from elesim_sim.simulation.genesis.utils import to_numpy_1d as _to_numpy_1d


class GenesisPinBridge:
    """Sync Genesis GO2 entity state into Pinocchio PinGo2Model q/dq vectors."""

    def __init__(
        self,
        entity,
        leg_dof_idxs: list[int],
        *,
        payload=None,
    ) -> None:
        if len(leg_dof_idxs) != 12:
            raise ValueError(f"expected 12 leg dofs, got {len(leg_dof_idxs)}")
        self._entity = entity
        self._leg_dof_idxs = [int(i) for i in leg_dof_idxs]
        self._payload = payload
        self._previous_base_pose: tuple[np.ndarray, Rot] | None = None
        self._warned_stale_twist = False
        self.last_q: np.ndarray | None = None
        self.last_dq: np.ndarray | None = None

    def reset(self) -> None:
        """Discard cached kinematics so a reset cannot create a velocity spike."""
        self._previous_base_pose = None
        self._warned_stale_twist = False
        self.last_q = None
        self.last_dq = None
        if self._payload is not None:
            self._payload.reset()

    @staticmethod
    def _prefer_pose_difference(
        readback: np.ndarray,
        estimate: np.ndarray | None,
        *,
        min_motion: float,
    ) -> tuple[np.ndarray, bool]:
        if estimate is None or not np.all(np.isfinite(estimate)):
            return readback, False
        # Genesis link twist readbacks can stay at zero while poses advance.
        # Keep them when they are meaningful; use the independently measured
        # pose delta when a near-zero readback contradicts clear motion.
        if (
            np.all(np.isfinite(readback))
            and np.linalg.norm(readback) < 0.02
            and np.linalg.norm(estimate) > min_motion
        ):
            return estimate, True
        if not np.all(np.isfinite(readback)):
            return estimate, True
        return readback, False

    def read_pin_q_dq(self, *, dt: float | None = None) -> tuple[np.ndarray, np.ndarray]:
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
        elapsed = None if dt is None else float(dt)
        if elapsed is not None and (not np.isfinite(elapsed) or elapsed <= 0.0):
            raise ValueError(f"dt must be finite and positive, got {dt}")
        finite_difference_linear = None
        finite_difference_angular = None
        if self._previous_base_pose is not None and elapsed is not None and elapsed > 0.0:
            previous_pos, previous_rot = self._previous_base_pose
            finite_difference_linear = rot.inv().apply((pos - previous_pos) / elapsed)
            world_rotation_delta = (rot * previous_rot.inv()).as_rotvec() / elapsed
            finite_difference_angular = rot.inv().apply(world_rotation_delta)
        self._previous_base_pose = (pos.copy(), rot)

        vel_body, linear_fallback = self._prefer_pose_difference(
            vel_body, finite_difference_linear, min_motion=0.03
        )
        ang_body, angular_fallback = self._prefer_pose_difference(
            ang_body, finite_difference_angular, min_motion=0.05
        )
        if (linear_fallback or angular_fallback) and not self._warned_stale_twist:
            print(
                "[go2_mpc] Genesis base twist readback is stale/zero; "
                "using pose-difference velocity feedback"
            )
            self._warned_stale_twist = True
        leg_dq = _to_numpy_1d(self._entity.get_dofs_velocity(dofs_idx_local=self._leg_dof_idxs))
        dq = np.concatenate([vel_body, ang_body, leg_dq])
        return q, dq

    def sync_pin_model(self, pin_model, *, dt: float | None = None) -> None:
        q, dq = self.read_pin_q_dq(dt=dt)
        # The camera renderer consumes this already-read controller state.  It
        # must not issue a second GPU->CPU readback merely to pose a frame.
        self.last_q = q.copy()
        self.last_dq = dq.copy()
        pin_model.update_model(q, dq)
        if self._payload is not None:
            self._payload.apply(pin_model, dt=dt)
