from __future__ import annotations

import json
import os
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as Rot

import elesim_model_builder.json_builder as assembly_builder
from elesim_controller.config import AppConfigBundle, load_app_config
from elesim_protocol import linear_effective_q_bounds
from elesim_controller.robot.arm.iklib.kinematics import _pick_manifest_value


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


def build_solver_context(
    config_path: str,
    *,
    build_dir: str | os.PathLike[str] | None = None,
) -> tuple[AppConfigBundle, dict[str, Any]]:
    bundle = load_app_config(config_path)
    build_dir = os.fspath(build_dir) if build_dir is not None else str(bundle.sim_config.build_dir)
    if not build_dir:
        raise ValueError("model build directory must be provided explicitly")
    manifest_path = os.path.join(build_dir, str(bundle.sim_config.assy_build_json))
    if bool(bundle.sim_config.rebuild_assembly) or (not os.path.isfile(manifest_path)):
        os.makedirs(build_dir, exist_ok=True)
        assembly_builder.build_default_manifest(
            build_dir,
            use_hardware=bool(bundle.sim_config.use_hardware),
            use_go2=bool(bundle.sim_config.use_go2),
            output_name=str(bundle.sim_config.assy_build_json),
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
    approach_axis_local = np.array([0.0, 0.0, -1.0], dtype=float)
    approach_rot_tip = np.eye(3, dtype=float)
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
            base_q_xyzw = np.array(_pick_manifest_value(base_pose_root, "q", default=[0.0, 0.0, 0.0, 1.0]), dtype=float).reshape(4)
            base_r = Rot.from_quat(base_q_xyzw)
            approach_axis_local = np.array([0.0, 0.0, -1.0], dtype=float)
            approach_rot_tip = (term_r.inv() * base_r).as_matrix()

    linear_min_m, linear_max_m = linear_effective_q_bounds(bundle.mapping_config)
    context = {
        "limit": bundle.joint_limit,
        "n_nodes": int(n_nodes),
        "n_seg": int(n_seg),
        "linear_joint_name": linear_joint_name,
        "linear_min_m": float(linear_min_m),
        "linear_max_m": float(linear_max_m),
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
        "approach_rot_tip": np.array(approach_rot_tip, dtype=float),
        "old_tip_local_offset": np.array(old_tip_local_offset, dtype=float),
        "grasp_offset_node_local": np.array(grasp_offset_node_local, dtype=float),
    }
    return bundle, context


__all__ = ["build_solver_context"]
