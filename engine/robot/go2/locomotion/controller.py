from __future__ import annotations

from typing import Dict, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from engine.robot.go2.locomotion.config import Go2LocomotionConfig
from engine.robot.go2.locomotion.gait import GaitScheduler
from engine.robot.go2.locomotion.kinematics import Go2KinematicsModel, _to_numpy_1d
from engine.robot.go2.locomotion.raibert import RaibertFootPlacement
from engine.robot.go2.locomotion.swing import SwingTrajectory
from engine.robot.go2.locomotion.types import ALL_LEGS, FootTarget, Go2Command, LegId, LegPhase


def _rot_from_wxyz(q_wxyz) -> Rot:
    q = np.asarray(q_wxyz, dtype=float).reshape(4)
    return Rot.from_quat([float(q[1]), float(q[2]), float(q[3]), float(q[0])])


def _body_to_world(pos_body: np.ndarray, base_pos: np.ndarray, base_rot: Rot) -> np.ndarray:
    return np.asarray(base_pos, dtype=float).reshape(3) + base_rot.apply(np.asarray(pos_body, dtype=float).reshape(3))


def _world_to_body(pos_world: np.ndarray, base_pos: np.ndarray, base_rot: Rot) -> np.ndarray:
    return base_rot.inv().apply(np.asarray(pos_world, dtype=float).reshape(3) - np.asarray(base_pos, dtype=float).reshape(3))


def _limit_step(cur: np.ndarray, target: np.ndarray, max_step: float) -> np.ndarray:
    delta = np.asarray(target, dtype=float).reshape(3) - np.asarray(cur, dtype=float).reshape(3)
    step = float(np.linalg.norm(delta))
    if step <= max_step or step <= 1e-12:
        return np.asarray(target, dtype=float).reshape(3)
    return np.asarray(cur, dtype=float).reshape(3) + delta * (max_step / step)


class RaibertTrotController:
    """Stage-1 trot: gait schedule, Raibert placement, swing parabola, Genesis IK, joint PD."""

    def __init__(self, entity, *, dt: float, config: Go2LocomotionConfig) -> None:
        self._entity = entity
        self._dt = float(dt)
        self._config = config
        self._kin = Go2KinematicsModel.from_entity(entity)
        self._gait = GaitScheduler(config)
        self._raibert = RaibertFootPlacement(config)
        self._cmd = Go2Command()
        self._prev_contact: Dict[LegId, LegPhase] = {leg: LegPhase.STANCE for leg in ALL_LEGS}
        self._stance_foot_body: Dict[LegId, np.ndarray] = {}
        self._stance_foot_world: Dict[LegId, np.ndarray] = {}
        self._swing_start_body: Dict[LegId, np.ndarray] = {}
        self._swing_end_body: Dict[LegId, np.ndarray] = {}
        self._leg_q_des: Dict[LegId, np.ndarray] = {}
        self._prev_leg_q_cmd: Optional[np.ndarray] = None
        self._base_dof_idx = self._detect_base_dof_idx()
        self._init_pd_and_pose()

    def _init_pd_and_pose(self) -> None:
        leg_dofs = self._kin.all_leg_dof_idx
        if not leg_dofs:
            return
        kp = np.full(len(leg_dofs), float(self._config.leg_kp), dtype=float)
        kv = np.full(len(leg_dofs), float(self._config.leg_kv), dtype=float)
        self._entity.set_dofs_kp(kp, dofs_idx_local=leg_dofs)
        self._entity.set_dofs_kv(kv, dofs_idx_local=leg_dofs)
        self._entity.set_dofs_position(self._kin.stand_q, dofs_idx_local=leg_dofs)
        self._entity.control_dofs_position(self._kin.stand_q, dofs_idx_local=leg_dofs)

        base_pos, base_rot = self._read_base_pose()
        for leg in ALL_LEGS:
            foot_world = self._snap_foot_to_ground(self._kin.read_foot_pos_world(self._entity, leg))
            self._stance_foot_world[leg] = foot_world.copy()
            self._stance_foot_body[leg] = _world_to_body(foot_world, base_pos, base_rot)
            self._leg_q_des[leg] = self._solve_leg_ik(leg, foot_world, fallback=self._stand_q_for_leg(leg))

    def _hold_idle_stance(self, leg_dofs: list[int]) -> None:
        base_pos, base_rot = self._read_base_pose()
        for leg in ALL_LEGS:
            if leg not in self._stance_foot_world:
                foot_world = self._snap_foot_to_ground(self._kin.read_foot_pos_world(self._entity, leg))
                self._stance_foot_world[leg] = foot_world.copy()
                self._stance_foot_body[leg] = _world_to_body(foot_world, base_pos, base_rot)
        q_des_parts: list[float] = []
        for leg in ALL_LEGS:
            foot_world = self._stance_world_target(leg)
            q_leg = self._solve_leg_ik(leg, foot_world, fallback=self._stand_q_for_leg(leg))
            q_des_parts.extend(q_leg.tolist())
        self._entity.control_dofs_position(np.asarray(q_des_parts, dtype=float), dofs_idx_local=leg_dofs)

    def set_command(self, cmd: Go2Command) -> None:
        self._cmd = cmd

    def _detect_base_dof_idx(self) -> list[int]:
        leg_set = set(self._kin.all_leg_dof_idx)
        n_dofs = int(getattr(self._entity, "n_dofs", 0))
        return [idx for idx in range(n_dofs) if idx not in leg_set]

    def _apply_base_velocity_assist(self, base_rot: Rot) -> None:
        blend = float(self._config.base_vel_blend)
        if blend <= 0.0 or not self._base_dof_idx:
            return
        cmd_world = self._cmd_vel_world(base_rot)
        vel = np.zeros(len(self._base_dof_idx), dtype=float)
        if len(vel) >= 1:
            vel[0] = float(cmd_world[0]) * blend
        if len(vel) >= 2:
            vel[1] = float(cmd_world[1]) * blend
        if len(vel) >= 6:
            vel[5] = float(self._cmd.yaw_rate) * blend
        elif len(vel) >= 3:
            vel[2] = float(self._cmd.yaw_rate) * blend
        self._entity.control_dofs_velocity(vel, dofs_idx_local=self._base_dof_idx)

    def _snap_foot_to_ground(self, foot_world: np.ndarray) -> np.ndarray:
        out = np.asarray(foot_world, dtype=float).reshape(3).copy()
        floor_z = float(self._config.ground_height_m + self._config.foot_radius_m)
        if out[2] < floor_z:
            out[2] = floor_z
        return out

    def _cmd_vel_world(self, base_rot: Rot) -> np.ndarray:
        cmd_body = np.array([float(self._cmd.vx), float(self._cmd.vy), 0.0], dtype=float)
        return base_rot.apply(cmd_body)

    def _stance_world_target(self, leg: LegId) -> np.ndarray:
        return np.asarray(
            self._stance_foot_world.get(
                leg,
                self._snap_foot_to_ground(self._kin.read_foot_pos_world(self._entity, leg)),
            ),
            dtype=float,
        ).reshape(3).copy()

    def step(self) -> None:
        leg_dofs = self._kin.all_leg_dof_idx
        if not leg_dofs:
            return

        if self._cmd.is_idle(self._config.command_idle_threshold):
            self._gait.reset()
            self._prev_leg_q_cmd = None
            self._hold_idle_stance(leg_dofs)
            return

        base_pos, base_rot = self._read_base_pose()
        v_body, _ = self._read_base_velocity(base_rot)
        self._gait.step(self._dt)

        foot_targets_world: Dict[LegId, np.ndarray] = {}
        for leg in ALL_LEGS:
            contact = self._gait.leg_contact(leg)
            prev = self._prev_contact[leg]
            nominal_body = self._kin.nominal_foot_body(leg, body_height_m=self._config.nominal_body_height_m)

            if contact == LegPhase.SWING and prev == LegPhase.STANCE:
                foot_world = self._kin.read_foot_pos_world(self._entity, leg)
                self._swing_start_body[leg] = _world_to_body(foot_world, base_pos, base_rot)
                self._swing_end_body[leg] = self._raibert.compute_foot_target(
                    leg,
                    v_body=v_body,
                    cmd=self._cmd,
                )
            elif contact == LegPhase.STANCE and prev == LegPhase.SWING:
                landed_world = self._snap_foot_to_ground(self._kin.read_foot_pos_world(self._entity, leg))
                self._stance_foot_world[leg] = landed_world.copy()
                self._stance_foot_body[leg] = _world_to_body(landed_world, base_pos, base_rot)

            if contact == LegPhase.STANCE:
                foot_targets_world[leg] = self._stance_world_target(leg)
            else:
                p_start = self._swing_start_body.get(
                    leg,
                    self._stance_foot_body.get(leg, nominal_body),
                )
                p_end = self._swing_end_body.get(
                    leg,
                    self._raibert.compute_foot_target(leg, v_body=v_body, cmd=self._cmd),
                )
                progress = self._gait.swing_progress(leg)
                foot_body = SwingTrajectory.sample(
                    progress,
                    p_start,
                    p_end,
                    self._config.foot_swing_height_m,
                )
                foot_targets_world[leg] = _body_to_world(foot_body, base_pos, base_rot)

            self._prev_contact[leg] = contact

        q_des_parts: list[float] = []
        for leg in ALL_LEGS:
            foot_world = foot_targets_world[leg]
            fallback = self._leg_q_des.get(leg, self._stand_q_for_leg(leg))
            q_leg = self._solve_leg_ik(leg, foot_world, fallback=fallback)
            self._leg_q_des[leg] = q_leg
            q_des_parts.extend(q_leg.tolist())

        q_des = np.asarray(q_des_parts, dtype=float)
        if self._prev_leg_q_cmd is None or self._prev_leg_q_cmd.shape != q_des.shape:
            self._prev_leg_q_cmd = q_des.copy()
        else:
            max_step = float(self._config.leg_max_rate_radps) * self._dt
            q_des = self._prev_leg_q_cmd + np.clip(q_des - self._prev_leg_q_cmd, -max_step, max_step)
            self._prev_leg_q_cmd = q_des.copy()
        self._entity.control_dofs_position(q_des, dofs_idx_local=leg_dofs)
        self._apply_base_velocity_assist(base_rot)

    def foot_targets_body(self) -> Tuple[FootTarget, ...]:
        base_pos, base_rot = self._read_base_pose()
        _ = base_pos, base_rot
        out: list[FootTarget] = []
        for leg in ALL_LEGS:
            if self._gait.leg_contact(leg) == LegPhase.STANCE:
                pos = _world_to_body(
                    self._stance_foot_world.get(
                        leg,
                        self._snap_foot_to_ground(self._kin.read_foot_pos_world(self._entity, leg)),
                    ),
                    base_pos,
                    base_rot,
                )
            else:
                p_start = self._swing_start_body.get(
                    leg,
                    self._stance_foot_body.get(
                        leg,
                        self._kin.nominal_foot_body(leg, body_height_m=self._config.nominal_body_height_m),
                    ),
                )
                p_end = self._swing_end_body.get(
                    leg,
                    self._raibert.compute_foot_target(leg, v_body=np.zeros(3), cmd=self._cmd),
                )
                pos = SwingTrajectory.sample(
                    self._gait.swing_progress(leg),
                    p_start,
                    p_end,
                    self._config.foot_swing_height_m,
                )
            out.append(FootTarget(leg=leg, pos_body=pos))
        return tuple(out)

    def _read_base_pose(self) -> Tuple[np.ndarray, Rot]:
        base = self._entity.get_link("base")
        pos = _to_numpy_1d(base.get_pos())[:3]
        quat = _to_numpy_1d(base.get_quat())[:4]
        return pos, _rot_from_wxyz(quat)

    def _read_base_velocity(self, base_rot: Rot) -> Tuple[np.ndarray, np.ndarray]:
        base = self._entity.get_link("base")
        vel_world = _to_numpy_1d(base.get_vel())[:3]
        ang_world = _to_numpy_1d(base.get_ang())[:3]
        vel_body = base_rot.inv().apply(vel_world)
        ang_body = base_rot.inv().apply(ang_world)
        return vel_body, ang_body

    def _stand_q_for_leg(self, leg: LegId) -> np.ndarray:
        idxs = self._kin.leg_dof_idx.get(leg, [])
        if not idxs:
            return np.zeros(3, dtype=float)
        all_idx = self._kin.all_leg_dof_idx
        values: list[float] = []
        for idx in idxs:
            try:
                pos = all_idx.index(idx)
                values.append(float(self._kin.stand_q[pos]))
            except ValueError:
                values.append(0.0)
        return np.asarray(values, dtype=float)

    def _solve_leg_ik(
        self,
        leg: LegId,
        foot_world: np.ndarray,
        *,
        fallback: np.ndarray,
    ) -> np.ndarray:
        dof_idxs = self._kin.leg_dof_idx.get(leg, [])
        if not dof_idxs:
            return np.asarray(fallback, dtype=float).reshape(-1)

        foot_link = self._kin.foot_link(self._entity, leg)
        try:
            ik_result = self._entity.inverse_kinematics(
                link=foot_link,
                pos=np.asarray(foot_world, dtype=float).reshape(3),
                quat=None,
                local_point=self._kin.foot_local_offset_in_calf(),
                pos_mask=[True, True, True],
                rot_mask=[False, False, False],
                dofs_idx_local=dof_idxs,
                max_solver_iters=40,
            )
            if isinstance(ik_result, tuple):
                ik_result = ik_result[0]
            q_arr = _to_numpy_1d(ik_result)
            if q_arr.size > max(dof_idxs):
                return np.asarray([float(q_arr[idx]) for idx in dof_idxs], dtype=float)
            if q_arr.size == len(dof_idxs):
                return q_arr
        except Exception:
            pass
        return np.asarray(fallback, dtype=float).reshape(-1)
