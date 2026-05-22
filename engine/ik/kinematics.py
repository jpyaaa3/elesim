from __future__ import annotations

import math
from typing import Any, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from engine.joint_defs import JointLimit
from engine.sag_model import segment_errors_from_model


Q4 = np.ndarray
Vec3 = np.ndarray

Q_NEUTRAL = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)
Q_BENT = np.array([0.0, 0.0, -math.radians(36.0), +math.radians(36.0)], dtype=float)


def _pick_manifest_value(mapping: dict[str, Any], *keys: str, default: Any = None) -> Any:
    for key in keys:
        if isinstance(mapping, dict) and key in mapping:
            return mapping[key]
    return default


def _build_q_map(context: dict[str, Any], q4: Sequence[float]) -> dict[str, float]:
    q = np.asarray(q4, dtype=float).reshape(4)
    linear = float(q[0])
    roll = float(q[1])
    theta1 = float(q[2])
    theta2 = float(q[3])
    linear_joint_name = str(context["linear_joint_name"])
    roll_joint_name = str(context["roll_joint_name"])
    bend_joint_names = [str(x) for x in context["bend_joint_names"]]
    n_nodes = len(bend_joint_names)
    n_seg = int(context["n_seg"])
    sag_model = dict(context.get("sag_model", {}) or {})
    theta1_deg = float(np.degrees(theta1))
    theta2_deg = float(np.degrees(theta2))
    seg1_err = np.radians(
        segment_errors_from_model(
            sag_model,
            seg_index=1,
            count=n_seg,
            theta1=theta1_deg,
            theta2=theta2_deg,
        )
    )
    seg2_err = np.radians(
        segment_errors_from_model(
            sag_model,
            seg_index=2,
            count=max(n_nodes - n_seg, 0),
            theta1=theta1_deg,
            theta2=theta2_deg,
        )
    )
    out = {linear_joint_name: linear, roll_joint_name: roll}
    for i, joint_name in enumerate(bend_joint_names):
        if i < n_seg:
            out[joint_name] = float(theta1 + float(seg1_err[i]))
        else:
            out[joint_name] = float(theta2 + float(seg2_err[i - n_seg]))
    return out


def _forward_link_tf(context: dict[str, Any], q4: Sequence[float]) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    q_map = _build_q_map(context, q4)
    part_pose_root = dict(context["part_pose_root"])
    part_rot_root = dict(context["part_rot_root"])
    root = str(context["fk_root_link"])
    spawn_pos = np.asarray(context["spawn_xyz"], dtype=float).reshape(3)
    spawn_euler = np.asarray(context["spawn_euler_deg"], dtype=float).reshape(3)
    R_spawn = Rot.from_euler("xyz", spawn_euler, degrees=True).as_matrix()

    link_tf: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    p_root_local = np.asarray(part_pose_root.get(root, np.zeros(3, dtype=float)), dtype=float).reshape(3)
    R_root_local = np.asarray(part_rot_root.get(root, np.eye(3, dtype=float)), dtype=float).reshape(3, 3)
    link_tf[root] = (spawn_pos + R_spawn @ p_root_local, R_spawn @ R_root_local)

    for meta in list(context["fk_joint_chain"]):
        parent = str(meta["parent"])
        child = str(meta["child"])
        if parent not in link_tf:
            continue
        p_parent, R_parent = link_tf[parent]
        origin_parent = np.asarray(meta["origin_parent"], dtype=float).reshape(3)
        axis_parent = np.asarray(meta["axis_parent"], dtype=float).reshape(3)
        child_rot_parent = np.asarray(meta.get("child_rot_parent", np.eye(3, dtype=float)), dtype=float).reshape(3, 3)
        jtype = str(meta["type"])
        q = float(q_map.get(str(meta["name"]), 0.0))
        if jtype == "prismatic":
            p_child = p_parent + R_parent @ (origin_parent + axis_parent * q)
            R_child = R_parent @ child_rot_parent
        elif jtype == "revolute":
            p_child = p_parent + R_parent @ origin_parent
            R_child = R_parent @ Rot.from_rotvec(axis_parent * q).as_matrix() @ child_rot_parent
        else:
            p_child = p_parent + R_parent @ origin_parent
            R_child = R_parent @ child_rot_parent
        link_tf[child] = (p_child, R_child)
    return link_tf


def _forward_old_tip_world(context: dict[str, Any], q4: Sequence[float]) -> np.ndarray:
    link_tf = _forward_link_tf(context, q4)
    terminal_link = str(context["terminal_link_name"])
    if terminal_link not in link_tf:
        raise RuntimeError(f"terminal link '{terminal_link}' missing from FK result")
    p_link, R_link = link_tf[terminal_link]
    old_tip_local = np.asarray(context["old_tip_local_offset"], dtype=float).reshape(3)
    return np.array(p_link + R_link @ old_tip_local, dtype=float)


def _forward_grasp_world(context: dict[str, Any], q4: Sequence[float]) -> np.ndarray:
    link_tf = _forward_link_tf(context, q4)
    terminal_link = str(context["terminal_link_name"])
    if terminal_link not in link_tf:
        raise RuntimeError(f"terminal link '{terminal_link}' missing from FK result")
    p_link, R_link = link_tf[terminal_link]
    old_tip_local = np.asarray(context["old_tip_local_offset"], dtype=float).reshape(3)
    grasp_offset_local = np.asarray(context["grasp_offset_node_local"], dtype=float).reshape(3)
    return np.array(p_link + R_link @ (old_tip_local + grasp_offset_local), dtype=float)


def _forward_grasp_direction_world(context: dict[str, Any], q4: Sequence[float]) -> np.ndarray:
    link_tf = _forward_link_tf(context, q4)
    tip_world = _forward_grasp_world(context, q4)
    link_name = str(context.get("approach_link_name", "") or context["terminal_link_name"])
    if link_name not in link_tf:
        link_name = str(context["terminal_link_name"])
    p_link, _R_link = link_tf[link_name]
    direction = np.asarray(tip_world, dtype=float).reshape(3) - np.asarray(p_link, dtype=float).reshape(3)
    norm = float(np.linalg.norm(direction))
    if norm <= 1e-9:
        raise RuntimeError("grasp direction is degenerate")
    return np.asarray(direction, dtype=float).reshape(3) / norm


def _damped_pinv(J: np.ndarray, damping: float = 1e-4) -> np.ndarray:
    J = np.asarray(J, dtype=float)
    if J.ndim != 2:
        raise ValueError("J must be a 2-D array")
    if J.size == 0:
        return np.zeros((J.shape[1], J.shape[0]), dtype=float)
    try:
        U, S, Vt = np.linalg.svd(J, full_matrices=False)
    except np.linalg.LinAlgError:
        return np.linalg.pinv(J)
    lam2 = float(max(damping, 1e-12)) ** 2
    Sinv = np.asarray([s / (s * s + lam2) for s in S], dtype=float)
    return Vt.T @ np.diag(Sinv) @ U.T


def _limited_step(
    step: Sequence[float],
    *,
    max_linear_m: float = 0.012,
    max_angle_rad: float = math.radians(4.0),
) -> np.ndarray:
    dq = np.asarray(step, dtype=float).reshape(4).copy()
    scale = 1.0
    if abs(float(dq[0])) > float(max_linear_m) > 0.0:
        scale = min(scale, float(max_linear_m) / abs(float(dq[0])))
    for i in (1, 2, 3):
        if abs(float(dq[i])) > float(max_angle_rad) > 0.0:
            scale = min(scale, float(max_angle_rad) / abs(float(dq[i])))
    return dq * float(max(scale, 0.0))


class _ReachModel:
    def __init__(self, *, context: dict[str, Any], limit: JointLimit) -> None:
        self.context = dict(context)
        self.linear_min = float(context["linear_min_m"])
        self.linear_max = float(context["linear_max_m"])
        self.roll_min = float(limit.roll_min_rad())
        self.roll_max = float(limit.roll_max_rad())
        self.bend_lim = float(limit.bend_lim_rad())

    def clamp_q(self, q: Sequence[float]) -> Q4:
        linear, roll, theta1, theta2 = map(float, np.asarray(q, dtype=float).reshape(4))
        return np.array(
            [
                float(np.clip(linear, self.linear_min, self.linear_max)),
                float(np.clip(roll, self.roll_min, self.roll_max)),
                float(np.clip(theta1, -self.bend_lim, +self.bend_lim)),
                float(np.clip(theta2, -self.bend_lim, +self.bend_lim)),
            ],
            dtype=float,
        )

    def grasp_position(self, q: Sequence[float]) -> Vec3:
        return _forward_grasp_world(self.context, self.clamp_q(q))

    def grasp_direction(self, q: Sequence[float]) -> Vec3:
        return _forward_grasp_direction_world(self.context, self.clamp_q(q))

    def error_vec(self, q: Sequence[float], target_world: Sequence[float]) -> Vec3:
        return self.grasp_position(q) - np.asarray(target_world, dtype=float).reshape(3)

    def direction_error(self, q: Sequence[float], target_dir_world: Sequence[float]) -> float:
        desired_dir = np.asarray(target_dir_world, dtype=float).reshape(3)
        dnorm = float(np.linalg.norm(desired_dir))
        if dnorm <= 1e-9:
            return 0.0
        desired_dir = desired_dir / dnorm
        actual_dir = self.grasp_direction(q)
        return float(1.0 - np.clip(float(np.dot(actual_dir, desired_dir)), -1.0, 1.0))

    def residual_vec(
        self,
        q: Sequence[float],
        *,
        target_world: Sequence[float],
        target_dir_world: Optional[Sequence[float]] = None,
        direction_weight: float = 0.10,
    ) -> np.ndarray:
        pos_err = self.error_vec(q, target_world)
        if target_dir_world is None:
            return np.asarray(pos_err, dtype=float).reshape(3)
        desired_dir = np.asarray(target_dir_world, dtype=float).reshape(3)
        dnorm = float(np.linalg.norm(desired_dir))
        if dnorm <= 1e-9:
            return np.asarray(pos_err, dtype=float).reshape(3)
        desired_dir = desired_dir / dnorm
        dir_err = self.grasp_direction(q) - desired_dir
        return np.concatenate(
            [
                np.asarray(pos_err, dtype=float).reshape(3),
                float(max(direction_weight, 0.0)) * np.asarray(dir_err, dtype=float).reshape(3),
            ],
            axis=0,
        )

    def numerical_jacobian(
        self,
        q: Sequence[float],
        *,
        target_world: Sequence[float],
        target_dir_world: Optional[Sequence[float]] = None,
        direction_weight: float = 0.10,
        eps: float = 1e-4,
    ) -> np.ndarray:
        q0 = self.clamp_q(q)
        eps_vec = np.array([5e-4, 5e-4, max(float(eps), 1e-4), max(float(eps), 1e-4)], dtype=float)
        r0 = self.residual_vec(
            q0,
            target_world=target_world,
            target_dir_world=target_dir_world,
            direction_weight=direction_weight,
        )
        J = np.zeros((r0.shape[0], 4), dtype=float)
        for i in range(4):
            qp = q0.copy()
            qm = q0.copy()
            qp[i] += eps_vec[i]
            qm[i] -= eps_vec[i]
            rp = self.residual_vec(
                qp,
                target_world=target_world,
                target_dir_world=target_dir_world,
                direction_weight=direction_weight,
            )
            rm = self.residual_vec(
                qm,
                target_world=target_world,
                target_dir_world=target_dir_world,
                direction_weight=direction_weight,
            )
            J[:, i] = (rp - rm) / (2.0 * eps_vec[i])
        return J

    def position_jacobian(self, q: Sequence[float], *, eps: float = 1e-4) -> np.ndarray:
        q0 = self.clamp_q(q)
        eps_vec = np.array([5e-4, 5e-4, max(float(eps), 1e-4), max(float(eps), 1e-4)], dtype=float)
        J = np.zeros((3, 4), dtype=float)
        for i in range(4):
            qp = q0.copy()
            qm = q0.copy()
            qp[i] += eps_vec[i]
            qm[i] -= eps_vec[i]
            pp = self.grasp_position(qp)
            pm = self.grasp_position(qm)
            J[:, i] = (pp - pm) / (2.0 * eps_vec[i])
        return J

    def direction_jacobian(self, q: Sequence[float], *, eps: float = 1e-4) -> np.ndarray:
        q0 = self.clamp_q(q)
        eps_vec = np.array([5e-4, 5e-4, max(float(eps), 1e-4), max(float(eps), 1e-4)], dtype=float)
        J = np.zeros((3, 4), dtype=float)
        for i in range(4):
            qp = q0.copy()
            qm = q0.copy()
            qp[i] += eps_vec[i]
            qm[i] -= eps_vec[i]
            dp = self.grasp_direction(qp)
            dm = self.grasp_direction(qm)
            J[:, i] = (dp - dm) / (2.0 * eps_vec[i])
        return J

    def tighten_once(
        self,
        *,
        current_q: Sequence[float],
        actual_tip_world: Sequence[float],
        target_world: Sequence[float],
        target_dir_world: Optional[Sequence[float]] = None,
        direction_weight: float = 0.10,
        damping: float = 1e-2,
        step_scale: float = 1.0,
    ) -> Q4:
        q = self.clamp_q(current_q)
        actual_tip = np.asarray(actual_tip_world, dtype=float).reshape(3)
        pos_err = np.asarray(target_world, dtype=float).reshape(3) - actual_tip
        if target_dir_world is None:
            delta_r = np.asarray(pos_err, dtype=float).reshape(3)
        else:
            desired_dir = np.asarray(target_dir_world, dtype=float).reshape(3)
            dnorm = float(np.linalg.norm(desired_dir))
            if dnorm <= 1e-9:
                delta_r = np.asarray(pos_err, dtype=float).reshape(3)
            else:
                desired_dir = desired_dir / dnorm
                dir_err = desired_dir - self.grasp_direction(q)
                delta_r = np.concatenate(
                    [
                        np.asarray(pos_err, dtype=float).reshape(3),
                        float(max(direction_weight, 0.0)) * np.asarray(dir_err, dtype=float).reshape(3),
                    ],
                    axis=0,
                )
        J = self.numerical_jacobian(
            q,
            target_world=target_world,
            target_dir_world=target_dir_world,
            direction_weight=direction_weight,
        )
        H = J.T @ J + float(max(damping, 1e-9)) * np.eye(4, dtype=float)
        g = J.T @ delta_r
        try:
            dq = np.linalg.solve(H, g)
        except np.linalg.LinAlgError:
            dq = np.linalg.pinv(H) @ g
        return self.clamp_q(q + float(step_scale) * dq)


__all__ = [
    "Q4",
    "Q_BENT",
    "Q_NEUTRAL",
    "Vec3",
    "_ReachModel",
    "_build_q_map",
    "_damped_pinv",
    "_forward_grasp_direction_world",
    "_forward_grasp_world",
    "_forward_link_tf",
    "_forward_old_tip_world",
    "_limited_step",
    "_pick_manifest_value",
]
