from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Any, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation as Rot

import builder.json_builder as assembly_builder
from engine.config_loader import AppConfigBundle, load_app_config_from_ini
from .kinematics import Q4, Q_BENT, Q_NEUTRAL, Vec3, _ReachModel, _pick_manifest_value


@dataclass(frozen=True)
class IkSolveRequest:
    target_world: Vec3
    position_tol_m: float = 1e-4


@dataclass(frozen=True)
class IkSolveResult:
    success: bool
    q: Optional[Q4]
    position_error_m: float
    seed_name: str
    iterations: int
    reason: str = ""


def _load_frame_to_offset(build_dir: str, part: dict[str, Any], *, part_name: str) -> np.ndarray:
    assets = _pick_manifest_value(part, "assets", default={}) or {}
    frame_rel = str(_pick_manifest_value(assets, "frame", default="") or "").strip()
    if not frame_rel:
        raise RuntimeError(f"manifest json is missing frame asset for '{part_name}'")
    frame_path = os.path.join(build_dir, frame_rel)
    with open(frame_path, "r", encoding="utf-8") as f:
        frame_json = json.load(f)
    connectors = _pick_manifest_value(frame_json, "connectors", default={}) or {}
    to_raw = _pick_manifest_value(connectors, "to", default=None)
    if isinstance(to_raw, dict):
        to_raw = _pick_manifest_value(to_raw, "p", default=None)
    if not isinstance(to_raw, (list, tuple)) or len(to_raw) != 3:
        raise RuntimeError(f"frame json is missing valid connectors.to for '{part_name}'")
    return np.array([float(to_raw[0]), float(to_raw[1]), float(to_raw[2])], dtype=float)


def load_solver_context(config_path: str) -> tuple[AppConfigBundle, dict[str, Any]]:
    bundle = load_app_config_from_ini(config_path)
    build_dir = str(bundle.sim_config.build_dir)
    manifest_path = os.path.join(build_dir, str(bundle.sim_config.assy_build_json))
    if bool(bundle.sim_config.rebuild_assembly) or (not os.path.isfile(manifest_path)):
        os.makedirs(build_dir, exist_ok=True)
        assembly_builder.build_default_manifest(
            build_dir,
            use_hardware=bool(bundle.sim_config.use_hardware),
            use_go2=bool(bundle.sim_config.use_go2),
        )
    with open(manifest_path, "r", encoding="utf-8") as f:
        build = json.load(f)

    joints = list(_pick_manifest_value(build, "joints", default=[]))
    parts = list(_pick_manifest_value(build, "parts", default=[]))
    if not joints or not parts:
        raise RuntimeError("manifest json is missing parts or joints")

    part_by_name = {str(_pick_manifest_value(p, "name", default="")).strip(): p for p in parts}
    joint_by_name = {str(_pick_manifest_value(j, "name", default="")).strip(): j for j in joints}

    revolute_names: list[str] = []
    for joint in joints:
        joint_name = str(_pick_manifest_value(joint, "name", default="")).strip()
        joint_type = str(_pick_manifest_value(joint, "type", default="")).strip().lower()
        if joint_name and joint_type == "revolute":
            revolute_names.append(joint_name)
    if len(revolute_names) < 3:
        raise RuntimeError("manifest json does not provide enough rotational joints for IK")

    linear_joint_name = "j_plate_housing"
    if linear_joint_name not in joint_by_name:
        raise RuntimeError("manifest json does not provide linear control joint j_plate_housing")
    roll_joint_name = revolute_names[0]
    bend_joint_names = revolute_names[1:]
    n_nodes = len(bend_joint_names)
    n_seg = int(bundle.spawn_config.n_seg) if bundle.spawn_config.n_seg is not None else max(1, n_nodes // 2)

    part_pose_root: dict[str, np.ndarray] = {}
    part_rot_root: dict[str, np.ndarray] = {}
    for p in parts:
        name = str(_pick_manifest_value(p, "name", default="")).strip()
        pose_root = _pick_manifest_value(p, "pose_root", default={}) or {}
        pr = _pick_manifest_value(pose_root, "p", default=None)
        qr = _pick_manifest_value(pose_root, "q", default=None)
        if not name:
            continue
        if isinstance(pr, (list, tuple)) and len(pr) == 3:
            part_pose_root[name] = np.array([float(pr[0]), float(pr[1]), float(pr[2])], dtype=float)
        if isinstance(qr, (list, tuple)) and len(qr) == 4:
            q_xyzw = np.array([float(qr[0]), float(qr[1]), float(qr[2]), float(qr[3])], dtype=float)
            part_rot_root[name] = Rot.from_quat(q_xyzw).as_matrix()

    parent_of: dict[str, str] = {}
    for j in joints:
        parent = str(_pick_manifest_value(j, "parent", default="")).strip()
        child = str(_pick_manifest_value(j, "child", default="")).strip()
        if parent and child:
            parent_of[child] = parent
    roots = [name for name in part_pose_root.keys() if name not in parent_of]
    if not roots:
        raise RuntimeError("manifest json does not provide a root link")
    fk_root_link = roots[0]

    fk_chain = []
    for meta in joints:
        joint_name = str(_pick_manifest_value(meta, "name", default="")).strip()
        parent = str(_pick_manifest_value(meta, "parent", default="")).strip()
        child = str(_pick_manifest_value(meta, "child", default="")).strip()
        jtype = str(_pick_manifest_value(meta, "type", default="")).strip().lower()
        if not joint_name or not parent or not child:
            continue
        anchor = _pick_manifest_value(meta, "anchor_root", default=[0.0, 0.0, 0.0])
        axis = _pick_manifest_value(meta, "axis_root", default=[1.0, 0.0, 0.0])
        p_parent = part_pose_root.get(parent, np.zeros(3, dtype=float))
        origin_parent = np.array(
            [
                float(anchor[0]) - float(p_parent[0]),
                float(anchor[1]) - float(p_parent[1]),
                float(anchor[2]) - float(p_parent[2]),
            ],
            dtype=float,
        )
        axis_parent = np.array([float(axis[0]), float(axis[1]), float(axis[2])], dtype=float)
        n = float(np.linalg.norm(axis_parent))
        if n > 1e-12:
            axis_parent /= n
        q_parent = part_rot_root.get(parent, np.eye(3, dtype=float))
        q_child = part_rot_root.get(child, np.eye(3, dtype=float))
        child_rot_parent = np.asarray(q_parent, dtype=float).reshape(3, 3).T @ np.asarray(q_child, dtype=float).reshape(3, 3)
        fk_chain.append(
            {
                "name": joint_name,
                "type": jtype,
                "parent": parent,
                "child": child,
                "origin_parent": origin_parent,
                "axis_parent": axis_parent,
                "child_rot_parent": child_rot_parent,
            }
        )

    terminal_link_name = str(_pick_manifest_value(joint_by_name.get(bend_joint_names[-1], {}), "child", default="")).strip()
    if not terminal_link_name:
        raise RuntimeError("manifest json does not provide a terminal bend child link")
    terminal_part = part_by_name.get(terminal_link_name)
    if terminal_part is None:
        raise RuntimeError(f"manifest json is missing terminal part '{terminal_link_name}'")
    old_tip_local_offset = _load_frame_to_offset(build_dir, terminal_part, part_name=terminal_link_name)

    grasp_offset_node_local = np.array([0.0, 0.0, 0.0], dtype=float)
    approach_axis_local = np.array(old_tip_local_offset, dtype=float).reshape(3)
    if "gripper_claw_left" in part_by_name and "gripper_claw_right" in part_by_name and terminal_link_name in part_by_name:
        term_part = part_by_name[terminal_link_name]
        term_pose_root = _pick_manifest_value(term_part, "pose_root", default={}) or {}
        term_p = np.array(_pick_manifest_value(term_pose_root, "p", default=[0.0, 0.0, 0.0]), dtype=float).reshape(3)
        term_q_xyzw = np.array(_pick_manifest_value(term_pose_root, "q", default=[0.0, 0.0, 0.0, 1.0]), dtype=float).reshape(4)
        term_r = Rot.from_quat(term_q_xyzw)
        old_tip_world = term_p + term_r.apply(old_tip_local_offset)

        left_part = part_by_name["gripper_claw_left"]
        right_part = part_by_name["gripper_claw_right"]
        left_pose_root = _pick_manifest_value(left_part, "pose_root", default={}) or {}
        right_pose_root = _pick_manifest_value(right_part, "pose_root", default={}) or {}
        left_p = np.array(_pick_manifest_value(left_pose_root, "p", default=[0.0, 0.0, 0.0]), dtype=float).reshape(3)
        right_p = np.array(_pick_manifest_value(right_pose_root, "p", default=[0.0, 0.0, 0.0]), dtype=float).reshape(3)
        left_q_xyzw = np.array(_pick_manifest_value(left_pose_root, "q", default=[0.0, 0.0, 0.0, 1.0]), dtype=float).reshape(4)
        right_q_xyzw = np.array(_pick_manifest_value(right_pose_root, "q", default=[0.0, 0.0, 0.0, 1.0]), dtype=float).reshape(4)
        left_r = Rot.from_quat(left_q_xyzw)
        right_r = Rot.from_quat(right_q_xyzw)
        left_tip = left_p + left_r.apply(_load_frame_to_offset(build_dir, left_part, part_name="gripper_claw_left"))
        right_tip = right_p + right_r.apply(_load_frame_to_offset(build_dir, right_part, part_name="gripper_claw_right"))
        grasp_mid_world = 0.5 * (left_tip + right_tip)
        grasp_offset_node_local = term_r.inv().apply(grasp_mid_world - old_tip_world)
        base_part = part_by_name.get("gripper_base")
        if base_part is not None:
            base_pose_root = _pick_manifest_value(base_part, "pose_root", default={}) or {}
            base_p = np.array(_pick_manifest_value(base_pose_root, "p", default=[0.0, 0.0, 0.0]), dtype=float).reshape(3)
            approach_axis_local = term_r.inv().apply(base_p - term_p)

    context = {
        "limit": bundle.joint_limit,
        "n_nodes": int(n_nodes),
        "n_seg": int(n_seg),
        "linear_joint_name": linear_joint_name,
        "linear_min_m": float(bundle.mapping_config.linear_q_min_m),
        "linear_max_m": float(bundle.mapping_config.linear_q_max_m),
        "roll_joint_name": roll_joint_name,
        "bend_joint_names": list(bend_joint_names),
        "fk_root_link": fk_root_link,
        "fk_joint_chain": fk_chain,
        "part_pose_root": part_pose_root,
        "part_rot_root": part_rot_root,
        "spawn_xyz": np.array(bundle.spawn_config.spawn_xyz, dtype=float),
        "spawn_euler_deg": np.array(bundle.spawn_config.spawn_euler_deg, dtype=float),
        "terminal_link_name": terminal_link_name,
        "approach_link_name": "gripper_base" if "gripper_base" in part_by_name else terminal_link_name,
        "approach_axis_local": np.array(approach_axis_local, dtype=float),
        "old_tip_local_offset": np.array(old_tip_local_offset, dtype=float),
        "grasp_offset_node_local": np.array(grasp_offset_node_local, dtype=float),
    }
    return bundle, context


def _optimize_position(
    *,
    q0: Sequence[float],
    tol: float,
    model: _ReachModel,
    target_world: Sequence[float],
    max_iters: int,
    damping: float = 1e-2,
    line_search_shrink: float = 0.5,
    line_search_steps: int = 6,
) -> tuple[bool, Q4, float, int]:
    q = model.clamp_q(q0)
    err_vec = model.error_vec(q, target_world)
    err = float(np.linalg.norm(err_vec))
    if err <= tol:
        return True, q.copy(), err, 0

    for iteration in range(1, max(int(max_iters), 1) + 1):
        err_vec = model.error_vec(q, target_world)
        err = float(np.linalg.norm(err_vec))
        if err <= tol:
            return True, q.copy(), err, iteration - 1
        residual = np.asarray(err_vec, dtype=float).reshape(3)
        J = model.position_jacobian(q)
        H = J.T @ J + float(max(damping, 1e-9)) * np.eye(4, dtype=float)
        g = J.T @ residual
        try:
            step = -np.linalg.solve(H, g)
        except np.linalg.LinAlgError:
            step = -np.linalg.pinv(H) @ g
        accepted = False
        for ls_idx in range(max(int(line_search_steps), 1)):
            alpha = float(np.clip(line_search_shrink, 1e-3, 0.999)) ** ls_idx
            q_try = model.clamp_q(q + alpha * step)
            residual_try = np.asarray(model.error_vec(q_try, target_world), dtype=float).reshape(3)
            residual_norm = float(np.linalg.norm(residual))
            residual_try_norm = float(np.linalg.norm(residual_try))
            err_try = float(np.linalg.norm(model.error_vec(q_try, target_world)))
            if residual_try_norm < residual_norm:
                q = q_try
                err = err_try
                accepted = True
                break
        if not accepted:
            break
    return bool(err <= tol), q.copy(), float(err), max(int(max_iters), 1)


def solve_ik(
    *,
    target_world: Sequence[float],
    context: dict[str, Any],
    position_tol_m: float = 1e-4,
    max_iters: int = 120,
    neutral_seed: Optional[Sequence[float]] = None,
    bent_seed: Optional[Sequence[float]] = None,
    current_seed: Optional[Sequence[float]] = None,
) -> IkSolveResult:
    request = IkSolveRequest(
        target_world=np.asarray(target_world, dtype=float).reshape(3),
        position_tol_m=float(position_tol_m),
    )
    model = _ReachModel(context=context, limit=context["limit"])
    tol = float(max(request.position_tol_m, 0.0))
    best_q: Optional[Q4] = None
    best_err = float("inf")
    best_seed = "bent"
    best_iters = int(max_iters)
    seed_specs: list[tuple[str, np.ndarray]] = []
    if current_seed is not None:
        seed_specs.append(("current", np.asarray(current_seed, dtype=float).reshape(4)))
    seed_specs.extend(
        [
            ("neutral", np.asarray(neutral_seed if neutral_seed is not None else Q_NEUTRAL, dtype=float).reshape(4)),
            ("bent", np.asarray(bent_seed if bent_seed is not None else Q_BENT, dtype=float).reshape(4)),
        ]
    )
    seen: set[tuple[float, ...]] = set()
    for seed_name, q_seed in seed_specs:
        key = tuple(np.round(q_seed.astype(float), 9))
        if key in seen:
            continue
        seen.add(key)
        success, q_sol, err, iters = _optimize_position(
            q0=q_seed,
            tol=tol,
            model=model,
            target_world=request.target_world,
            max_iters=max_iters,
        )
        if err < best_err:
            best_q = q_sol
            best_err = err
            best_seed = seed_name
            best_iters = iters
        if success:
            return IkSolveResult(True, q_sol.copy(), err, seed_name, iters, "converged")
    return IkSolveResult(False, None if best_q is None else best_q.copy(), best_err, best_seed, best_iters, "position tolerance not reached")


def tighten_from_actual(
    *,
    current_q: Sequence[float],
    actual_tip_world: Sequence[float],
    target_world: Sequence[float],
    context: dict[str, Any],
    damping: float = 1e-2,
    step_scale: float = 1.0,
) -> np.ndarray:
    model = _ReachModel(context=context, limit=context["limit"])
    q = model.clamp_q(current_q)
    pos_err = np.asarray(target_world, dtype=float).reshape(3) - np.asarray(actual_tip_world, dtype=float).reshape(3)
    J = model.position_jacobian(q)
    H = J.T @ J + float(max(damping, 1e-9)) * np.eye(4, dtype=float)
    g = J.T @ pos_err
    try:
        dq = np.linalg.solve(H, g)
    except np.linalg.LinAlgError:
        dq = np.linalg.pinv(H) @ g
    return model.clamp_q(q + float(step_scale) * dq)


load_ik_context = load_solver_context


__all__ = [
    "IkSolveRequest",
    "IkSolveResult",
    "load_ik_context",
    "load_solver_context",
    "solve_ik",
    "tighten_from_actual",
]
