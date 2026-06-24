#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import time
from dataclasses import dataclass, field, replace
from typing import Any, Dict, List, Optional, Tuple
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation as Rot
import zmq
import genesis as gs
from genesis.utils import geom as gs_geom

import engine.protocol as proto
import builder.json_builder as assembly_builder
from builder.go2_arm_merger import merge_go2_arm_urdf
from engine.config_loader import (
    Go2LocomotionConfig,
    HardwareConfig,
    IkConfig,
    JointLimit,
    SimConfig,
    SimParam,
    SpawnConfig,
    UrdfExportConfig,
    load_app_config_from_ini,
)
from engine.go2_locomotion import Go2Command
from engine.go2_locomotion.controller import RaibertTrotController
from engine.go2_locomotion.kinematics import GO2_READY_Q, GO2_STAND_Q
from engine.motor import estimate_ideal_sim_rates
from builder.urdf_converter import convert_manifest_file
from engine.sag_model import segment_errors_from_model


def _ensure_genesis_cache_dir() -> None:
    """
    Genesis viewer may try to write a temporary video under ~/.cache/genesis
    even when the app did not explicitly request recording.
    """
    cache_root = os.environ.get("XDG_CACHE_HOME", "").strip()
    if cache_root:
        cache_dir = Path(cache_root).expanduser() / "genesis"
    else:
        cache_dir = Path.home() / ".cache" / "genesis"
    cache_dir.mkdir(parents=True, exist_ok=True)


def _to_numpy_1d(raw) -> np.ndarray:
    if hasattr(raw, "detach"):
        raw = raw.detach()
    if hasattr(raw, "cpu"):
        raw = raw.cpu()
    if hasattr(raw, "numpy"):
        raw = raw.numpy()
    return np.array(raw, dtype=float).reshape(-1)


class PerfLogger:
    _FIELDS = (
        "wall_time_s",
        "samples",
        "fps",
        "loop_avg_ms",
        "loop_max_ms",
        "poll_avg_ms",
        "poll_max_ms",
        "go2_avg_ms",
        "go2_max_ms",
        "markers_avg_ms",
        "markers_max_ms",
        "feedback_avg_ms",
        "feedback_max_ms",
        "physics_avg_ms",
        "physics_max_ms",
        "camera_avg_ms",
        "camera_max_ms",
    )

    def __init__(self, *, enabled: bool, interval_s: float = 2.0, log_path: str = "") -> None:
        self.enabled = bool(enabled)
        self.interval_s = max(0.25, float(interval_s))
        self._last_report_t = time.perf_counter()
        self._count = 0
        self._sum: dict[str, float] = {}
        self._max: dict[str, float] = {}
        self._t0 = self._last_report_t
        self._started_wall = time.time()
        self._log_file = None
        self._writer: Optional[csv.DictWriter] = None
        if self.enabled:
            path = self._resolve_log_path(log_path)
            if path:
                path.parent.mkdir(parents=True, exist_ok=True)
                self._log_file = open(path, "w", newline="", encoding="utf-8")
                self._writer = csv.DictWriter(self._log_file, fieldnames=list(self._FIELDS))
                self._writer.writeheader()
                print(f"[perf] logging to {path}")

    @staticmethod
    def _resolve_log_path(raw: str) -> Optional[Path]:
        value = str(raw or "").strip()
        if not value:
            stamp = time.strftime("sim_perf_%Y%m%d_%H%M%S.csv")
            return Path("logs") / "perf" / stamp
        path = Path(value).expanduser()
        if path.is_dir() or value.endswith(("/", os.sep)):
            stamp = time.strftime("sim_perf_%Y%m%d_%H%M%S.csv")
            path = path / stamp
        return path

    def reset_loop(self) -> None:
        if self.enabled:
            self._t0 = time.perf_counter()

    def section(self, name: str, t0: float) -> None:
        if not self.enabled:
            return
        dt = time.perf_counter() - float(t0)
        self._sum[name] = self._sum.get(name, 0.0) + dt
        self._max[name] = max(self._max.get(name, 0.0), dt)

    def report_if_due(self) -> None:
        if not self.enabled:
            return
        now = time.perf_counter()
        loop_dt = now - self._t0
        self._count += 1
        self._sum["loop"] = self._sum.get("loop", 0.0) + loop_dt
        self._max["loop"] = max(self._max.get("loop", 0.0), loop_dt)
        elapsed = now - self._last_report_t
        if elapsed < self.interval_s:
            return
        count = max(1, self._count)
        fps = count / max(1e-9, elapsed)
        row = {
            "wall_time_s": time.time() - self._started_wall,
            "samples": count,
            "fps": fps,
        }
        parts = [f"fps={fps:.1f}"]
        for name in ("loop", "poll", "go2", "markers", "feedback", "physics", "camera"):
            avg_ms = 1000.0 * self._sum.get(name, 0.0) / count
            max_ms = 1000.0 * self._max.get(name, 0.0)
            row[f"{name}_avg_ms"] = avg_ms
            row[f"{name}_max_ms"] = max_ms
            if name not in self._sum:
                continue
            parts.append(f"{name}={avg_ms:.2f}/{max_ms:.2f}ms")
        print("[perf] " + " ".join(parts))
        if self._writer is not None:
            self._writer.writerow(row)
            if self._log_file is not None:
                self._log_file.flush()
        self._last_report_t = now
        self._count = 0
        self._sum.clear()
        self._max.clear()

    def close(self) -> None:
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None


def _as_single_dof_index(raw_idx) -> int:
    if isinstance(raw_idx, (list, tuple, np.ndarray)):
        arr = np.array(raw_idx).reshape(-1)
        if arr.size <= 0:
            raise ValueError("empty dof index list")
        return int(arr[0])
    return int(raw_idx)


def _rot_from_wxyz(q_wxyz) -> Rot:
    q = np.asarray(q_wxyz, dtype=float).reshape(4)
    return Rot.from_quat([float(q[1]), float(q[2]), float(q[3]), float(q[0])])


def _world_offset(
    pos: Tuple[float, float, float],
    euler_deg: Tuple[float, float, float],
    local_offset: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    world_off = Rot.from_euler("xyz", np.asarray(euler_deg, dtype=float), degrees=True).apply(local_offset)
    return (
        float(pos[0] + world_off[0]),
        float(pos[1] + world_off[1]),
        float(pos[2] + world_off[2]),
    )


class Go2Locomotion:
    """GO2 locomotion adapter (Raibert trot, convex MPC, or host pose mirror)."""

    def __init__(
        self,
        entity,
        *,
        dt: float,
        config: Go2LocomotionConfig,
        arm_entity=None,
        metrics=None,
        command_source: str = "teleop",
    ):
        self._metrics = metrics
        self._arm_q: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
        self._mirror = bool(config.mirror_from_host)
        self._entity = entity
        self._controller = None
        if self._mirror:
            from engine.go2_locomotion.kinematics import Go2KinematicsModel

            self._kin = Go2KinematicsModel.from_entity(entity)
            self._leg_dof_idxs = list(self._kin.all_leg_dof_idx)
            self._last_mirror_pos: Optional[tuple[float, float, float]] = None
            self._last_mirror_rpy: Optional[tuple[float, float, float]] = None
            self._last_mirror_leg_q: Optional[tuple[float, ...]] = None
            self._init_mirror_kinematic()
            return

        mode = str(config.mode).strip().lower()
        if mode == "convex_mpc":
            from engine.go2_mpc.config import Go2MpcConfig
            from engine.go2_mpc.controller import ConvexMpcGenesisController

            mpc_cfg = Go2MpcConfig(
                gait_hz=float(config.gait_hz),
                gait_duty=float(config.gait_duty),
                z_pos_des_m=float(config.z_pos_des_m),
                mpc_steps_per_gait=int(config.mpc_steps_per_gait),
                command_idle_threshold=float(config.command_idle_threshold),
                torque_safety_scale=float(config.torque_safety_scale),
                leg_kv_damping=float(config.mpc_leg_kv_damping),
                stand_kp=float(config.leg_kp),
                stand_kv=float(config.leg_kv),
                ctrl_hz=float(config.mpc_ctrl_hz),
                command_ramp_s=float(config.mpc_command_ramp_s),
                torque_ramp_s=float(config.mpc_torque_ramp_s),
                torque_warmup_s=float(config.mpc_torque_warmup_s),
                ready_pose_s=float(config.mpc_ready_pose_s),
                ready_kp=float(config.mpc_ready_kp),
                ready_kv=float(config.mpc_ready_kv),
                aux_kp=float(config.mpc_aux_kp),
                aux_kv=float(config.mpc_aux_kv),
                tau_filter_alpha=float(config.mpc_tau_filter_alpha),
                force_filter_alpha=float(config.mpc_force_filter_alpha),
                foot_placement_scale=float(config.mpc_foot_placement_scale),
                payload_enable=bool(config.mpc_payload_enable),
                payload_mass_kg=float(config.mpc_payload_mass_kg),
                pitch_trim_gain_x_forward=float(config.mpc_pitch_trim_gain_x_forward),
                pitch_trim_gain_x_backward=float(config.mpc_pitch_trim_gain_x_backward),
                pitch_trim_gain_z=float(config.mpc_pitch_trim_gain_z),
                pitch_trim_z_ref_m=float(config.mpc_pitch_trim_z_ref_m),
                pitch_trim_max_rad=float(config.mpc_pitch_trim_max_rad),
            )
            self._controller = ConvexMpcGenesisController(
                entity,
                dt=float(dt),
                config=mpc_cfg,
                arm_entity=arm_entity,
                metrics=metrics,
                command_source=str(command_source),
            )
        else:
            self._controller = RaibertTrotController(entity, dt=float(dt), config=config)

    @property
    def mirror_mode(self) -> bool:
        return bool(self._mirror)

    def _init_mirror_kinematic(self) -> None:
        """Mirror puppet: no PD actuation; pose is overwritten each frame."""
        n = len(self._leg_dof_idxs)
        self._entity.set_dofs_kp(np.zeros(n, dtype=float), dofs_idx_local=self._leg_dof_idxs)
        self._entity.set_dofs_kv(np.zeros(n, dtype=float), dofs_idx_local=self._leg_dof_idxs)
        self._hold_mirror_stand()

    def _hold_mirror_stand(self) -> None:
        stand_q = np.asarray(self._kin.stand_q, dtype=float)
        self._entity.set_dofs_position(stand_q, dofs_idx_local=self._leg_dof_idxs)

    def apply_mirror_pose(
        self,
        pos: tuple[float, float, float],
        rpy: tuple[float, float, float],
        leg_q: Optional[tuple[float, ...]] = None,
    ) -> None:
        if not self._mirror:
            return
        self._last_mirror_pos = (float(pos[0]), float(pos[1]), float(pos[2]))
        self._last_mirror_rpy = (float(rpy[0]), float(rpy[1]), float(rpy[2]))
        if leg_q is not None and len(leg_q) == 12:
            self._last_mirror_leg_q = tuple(float(v) for v in leg_q)
        self._set_mirror_pose(self._last_mirror_pos, self._last_mirror_rpy, self._last_mirror_leg_q)

    def reapply_last_mirror_pose(self) -> bool:
        if not self._mirror or self._last_mirror_pos is None or self._last_mirror_rpy is None:
            return False
        self._set_mirror_pose(self._last_mirror_pos, self._last_mirror_rpy, self._last_mirror_leg_q)
        return True

    def _set_mirror_pose(
        self,
        pos: tuple[float, float, float],
        rpy: tuple[float, float, float],
        leg_q: Optional[tuple[float, ...]] = None,
    ) -> None:
        rot = Rot.from_euler("xyz", np.asarray(rpy, dtype=float), degrees=False)
        quat_xyzw = rot.as_quat()
        quat_wxyz = np.array(
            [float(quat_xyzw[3]), float(quat_xyzw[0]), float(quat_xyzw[1]), float(quat_xyzw[2])],
            dtype=float,
        )
        self._entity.set_pos(np.asarray(pos, dtype=float).reshape(3))
        self._entity.set_quat(quat_wxyz)
        if leg_q is not None and len(leg_q) == 12:
            self._entity.set_dofs_position(np.asarray(leg_q, dtype=float), dofs_idx_local=self._leg_dof_idxs)
        else:
            self._hold_mirror_stand()
        try:
            self._entity.zero_all_dofs_velocity()
        except Exception:
            pass

    def set_command_source(self, source: str) -> None:
        if self._controller is not None and hasattr(self._controller, "_command_source"):
            self._controller._command_source = str(source)

    def set_arm_q_for_metrics(self, arm_q: tuple[float, float, float, float]) -> None:
        self._arm_q = tuple(float(x) for x in arm_q)

    def record_arm_q_sample(self, arm_q: tuple[float, float, float, float]) -> None:
        self._arm_q = tuple(float(x) for x in arm_q)

    def reset_locomotion(self) -> None:
        if self._controller is not None and hasattr(self._controller, "reset"):
            self._controller.reset()
        self._arm_q = (0.0, 0.0, 0.0, 0.0)
        if self._mirror:
            self._init_mirror_kinematic()

    def set_planar_velocity(self, vx: float, vy: float, wz: float) -> None:
        if self._mirror or self._controller is None:
            return
        self._controller.set_command(Go2Command(vx=float(vx), vy=float(vy), yaw_rate=float(wz)))

    def step(self) -> None:
        if self._mirror:
            return
        if self._controller is not None and hasattr(self._controller, "set_arm_q"):
            self._controller.set_arm_q(self._arm_q)
        if self._controller is not None:
            self._controller.step()


def _make_urdf_morph(
    urdf_path: str,
    pos: Tuple[float, float, float],
    euler: Tuple[float, float, float],
    *,
    fixed: bool,
    requires_jac_and_IK: bool = False,
):
    common = dict(
        file=urdf_path,
        pos=pos,
        euler=euler,
        fixed=bool(fixed),
        prioritize_urdf_material=True,
        merge_fixed_links=False,
        requires_jac_and_IK=bool(requires_jac_and_IK),
    )
    merge_fixed = not bool(requires_jac_and_IK)
    try:
        return gs.morphs.URDF(**common, default_armature=0.0, merge_fixed_links=merge_fixed)
    except TypeError:
        try:
            return gs.morphs.URDF(**common, default_armature=0.0)
        except TypeError:
            return gs.morphs.URDF(file=urdf_path, pos=pos, euler=euler, fixed=bool(fixed))


def _prepare_go2_urdf_with_config_colors(
    source_urdf: str,
    *,
    build_dir: str,
    colors: Dict[str, Tuple[float, float, float, float]],
) -> str:
    go2_colors = {str(k).strip(): v for k, v in (colors or {}).items() if str(k).strip().startswith("go2")}
    if not go2_colors:
        return source_urdf

    def fmt_rgba(rgba: Tuple[float, float, float, float]) -> str:
        vals = [float(x) for x in rgba]
        if len(vals) == 3:
            vals.append(1.0)
        return " ".join(f"{x:.9g}" for x in vals[:4])

    default = go2_colors.get("go2")
    group_color = {
        "base": go2_colors.get("go2_base", default),
        "hip": go2_colors.get("go2_hip", default),
        "thigh": go2_colors.get("go2_thigh", default),
        "calf": go2_colors.get("go2_calf", default),
        "foot": go2_colors.get("go2_foot", default),
    }
    tree = ET.parse(source_urdf)
    root = tree.getroot()
    source_dir = os.path.dirname(os.path.abspath(source_urdf))
    changed = 0
    for mesh in root.findall(".//mesh"):
        filename = str(mesh.attrib.get("filename", "")).strip()
        if filename and not os.path.isabs(filename):
            mesh.attrib["filename"] = os.path.abspath(os.path.join(source_dir, filename))
    for link in root.findall("link"):
        name = str(link.attrib.get("name", ""))
        lname = name.lower()
        rgba = None
        if name == "base":
            rgba = group_color["base"]
        elif "hip" in lname:
            rgba = group_color["hip"]
        elif "thigh" in lname:
            rgba = group_color["thigh"]
        elif "calf" in lname:
            rgba = group_color["calf"]
        elif "foot" in lname:
            rgba = group_color["foot"]
        if rgba is None:
            continue
        for visual in link.findall("visual"):
            material = visual.find("material")
            if material is None:
                material = ET.SubElement(visual, "material", attrib={"name": f"{name}_mat"})
            material.attrib["name"] = f"go2_{name}_mat"
            color = material.find("color")
            if color is None:
                color = ET.SubElement(material, "color")
            color.attrib["rgba"] = fmt_rgba(rgba)
            changed += 1

    if changed <= 0:
        return source_urdf
    os.makedirs(build_dir, exist_ok=True)
    out = os.path.join(build_dir, "go2_colored.urdf")
    tree.write(out, encoding="utf-8", xml_declaration=True)
    print(f"[runtime] GO2 URDF colors applied: {out} visuals={changed}")
    return out


def _set_go2_initial_leg_pose(go2_entity, *, pose_name: str = "ready") -> None:
    """Set GO2 leg joints after Genesis build so calf joints start within limits."""
    pose = GO2_READY_Q if str(pose_name).strip().lower() == "ready" else GO2_STAND_Q
    dof_idxs: list[int] = []
    q_vals: list[float] = []
    for joint_name, q in pose.items():
        try:
            joint = go2_entity.get_joint(str(joint_name))
            raw_idxs = getattr(joint, "dofs_idx_local", None)
        except Exception:
            continue
        if raw_idxs is None:
            continue
        for idx in np.asarray(raw_idxs, dtype=int).reshape(-1):
            dof_idxs.append(int(idx))
            q_vals.append(float(q))
    if not dof_idxs:
        return
    q_arr = np.asarray(q_vals, dtype=float)
    try:
        go2_entity.set_dofs_position(q_arr, dofs_idx_local=dof_idxs)
        go2_entity.control_dofs_position(q_arr, dofs_idx_local=dof_idxs)
        print(f"[runtime] GO2 initial leg pose set: {pose_name} ({len(dof_idxs)} dofs)")
    except Exception as exc:
        print(f"[runtime] GO2 initial leg pose skipped: {exc}")


@dataclass
class JointLayout:
    linear_joint_name: str = "j_plate_housing"
    roll_joint_name: str = "base_roll_x"
    bend_joint_names: List[str] = field(default_factory=list)
    linear_axis_sign: float = 1.0
    roll_axis_sign: float = 1.0
    bend_axis_sign: float = -1.0
    chain_origin_local: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=float))
    tip_link_name: str = ""
    tip_local_offset: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=float))
    old_tip_local_offset: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=float))
    tip_points: List[Tuple[str, np.ndarray]] = field(default_factory=list)
    approach_link_name: str = "gripper_base"
    approach_axis_local: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -1.0], dtype=float))
    approach_rot_tip: np.ndarray = field(default_factory=lambda: np.eye(3, dtype=float))
    control_mode: str = "commanded"
    part_control_mode: Dict[str, str] = field(default_factory=dict)
    part_pose_root: Dict[str, np.ndarray] = field(default_factory=dict)
    part_rot_root: Dict[str, np.ndarray] = field(default_factory=dict)
    fk_root_link: str = "plate"
    fk_joint_chain: List[Dict[str, object]] = field(default_factory=list)
    no_clip_pairs: List[Tuple[str, str]] = field(default_factory=list)


@dataclass
class MarkerSet:
    _ik_target_marker: object = None
    _sim_tip_marker: object = None
    _ik_target_marker_dir: object = None
    _sim_tip_marker_dir: object = None
    _ik_target_marker_pos: Optional[np.ndarray] = None
    _sim_tip_marker_pos: Optional[np.ndarray] = None
    _ik_target_marker_dir_sig: Optional[np.ndarray] = None
    _sim_tip_marker_dir_sig: Optional[np.ndarray] = None
    _dynamic_markers: dict[str, object] = field(default_factory=dict)
    _dynamic_marker_sig: dict[str, np.ndarray] = field(default_factory=dict)

    def draw(self, scene, attr_name: str, pos: np.ndarray, color) -> None:
        pos_arr = np.asarray(pos, dtype=float).reshape(3)
        prev_pos = getattr(self, f"{attr_name}_pos", None)
        marker = getattr(self, attr_name, None)
        if marker is not None and prev_pos is not None and np.array_equal(prev_pos, pos_arr):
            return
        if marker is not None:
            try:
                scene.clear_debug_object(marker)
            except Exception:
                pass
        setattr(self, attr_name, scene.draw_debug_sphere(pos=pos_arr, radius=0.012, color=color))
        setattr(self, f"{attr_name}_pos", pos_arr.copy())

    def draw_direction(self, scene, attr_name: str, pos: np.ndarray, direction: np.ndarray, color) -> None:
        pos_arr = np.asarray(pos, dtype=float).reshape(3)
        dir_arr = np.asarray(direction, dtype=float).reshape(3)
        norm = float(np.linalg.norm(dir_arr))
        if norm <= 1e-9:
            return
        dir_arr = dir_arr / norm
        sig = np.concatenate([pos_arr, dir_arr], axis=0)
        prev_sig = getattr(self, f"{attr_name}_sig", None)
        marker = getattr(self, attr_name, None)
        if marker is not None and prev_sig is not None and np.allclose(prev_sig, sig, atol=1e-9):
            return
        if marker is not None:
            try:
                scene.clear_debug_object(marker)
            except Exception:
                pass
        arrow = scene.draw_debug_arrow(pos=pos_arr, vec=dir_arr * 0.09, radius=0.004, color=color)
        setattr(self, attr_name, arrow)
        setattr(self, f"{attr_name}_sig", sig.copy())

    def draw_dynamic_sphere(self, scene, key: str, pos: np.ndarray, color, radius: float) -> None:
        pos_arr = np.asarray(pos, dtype=float).reshape(3)
        sig = pos_arr.copy()
        marker = self._dynamic_markers.get(key, None)
        prev_sig = self._dynamic_marker_sig.get(key, None)
        if marker is not None and prev_sig is not None and np.allclose(prev_sig, sig, atol=1e-9):
            return
        if marker is not None:
            try:
                scene.clear_debug_object(marker)
            except Exception:
                pass
        self._dynamic_markers[key] = scene.draw_debug_sphere(pos=pos_arr, radius=float(radius), color=color)
        self._dynamic_marker_sig[key] = sig

    def draw_dynamic_arrow(
        self,
        scene,
        key: str,
        pos: np.ndarray,
        direction: np.ndarray,
        color,
        radius: float,
        length: float = 0.09,
    ) -> None:
        pos_arr = np.asarray(pos, dtype=float).reshape(3)
        dir_arr = np.asarray(direction, dtype=float).reshape(3)
        norm = float(np.linalg.norm(dir_arr))
        if norm <= 1e-9:
            return
        dir_arr = dir_arr / norm
        length_f = max(float(length), 0.0)
        sig = np.concatenate([pos_arr, dir_arr, np.array([length_f], dtype=float)], axis=0)
        marker = self._dynamic_markers.get(key, None)
        prev_sig = self._dynamic_marker_sig.get(key, None)
        if marker is not None and prev_sig is not None and np.allclose(prev_sig, sig, atol=1e-9):
            return
        if marker is not None:
            try:
                scene.clear_debug_object(marker)
            except Exception:
                pass
        self._dynamic_markers[key] = scene.draw_debug_arrow(pos=pos_arr, vec=dir_arr * length_f, radius=float(radius), color=color)
        self._dynamic_marker_sig[key] = sig

    def clear_dynamic_missing(self, scene, active_keys: set[str]) -> None:
        stale = [key for key in self._dynamic_markers.keys() if key not in active_keys]
        for key in stale:
            marker = self._dynamic_markers.pop(key, None)
            self._dynamic_marker_sig.pop(key, None)
            if marker is not None:
                try:
                    scene.clear_debug_object(marker)
                except Exception:
                    pass


@dataclass
class SimScene:
    scene: object = None
    mover: Optional["SimMover"] = None
    go2: Optional[Go2Locomotion] = None
    go2_entity: object = None
    walking_metrics: object = None
    eye_camera: object = None
    camera_publisher: object = None
    hand_eye_config_path: str = ""
    n_nodes: int = 0
    n_seg: int = 0
    _last_camera_publish_t: float = 0.0
    _arm_mount_pos_body: Optional[np.ndarray] = None
    _arm_mount_rot_body: Optional[Rot] = None

    def record_arm_go2_mount(self, *, arm_ent, go2_ent) -> None:
        """Store arm root pose relative to GO2 base (for per-step kinematic sync)."""
        from engine.go2_mpc.genesis_pin_bridge import _quat_wxyz_to_xyzw

        base = go2_ent.get_link("base")
        base_pos = self._to_numpy_1d(base.get_pos())[:3]
        base_quat_xyzw = _quat_wxyz_to_xyzw(self._to_numpy_1d(base.get_quat())[:4])
        arm_pos = self._to_numpy_1d(arm_ent.get_pos())[:3]
        arm_quat_xyzw = _quat_wxyz_to_xyzw(self._to_numpy_1d(arm_ent.get_quat())[:4])
        R_b = Rot.from_quat(base_quat_xyzw)
        R_a = Rot.from_quat(arm_quat_xyzw)
        self._arm_mount_pos_body = np.asarray(R_b.inv().apply(arm_pos - base_pos), dtype=float)
        self._arm_mount_rot_body = R_b.inv() * R_a

    def sync_arm_to_go2_base(self) -> None:
        """Keep sim arm entity welded to GO2 base (Genesis weld can drift under MPC)."""
        if (
            self.go2_entity is None
            or self.mover is None
            or self._arm_mount_pos_body is None
            or self._arm_mount_rot_body is None
        ):
            return
        try:
            from engine.go2_mpc.genesis_pin_bridge import _quat_wxyz_to_xyzw

            base = self.go2_entity.get_link("base")
            arm_ent = self.mover.entity
            base_pos = self._to_numpy_1d(base.get_pos())[:3]
            base_quat_xyzw = _quat_wxyz_to_xyzw(self._to_numpy_1d(base.get_quat())[:4])
            R_b = Rot.from_quat(base_quat_xyzw)
            new_pos = base_pos + R_b.apply(self._arm_mount_pos_body)
            new_rot = R_b * self._arm_mount_rot_body
            quat_xyzw = new_rot.as_quat()
            quat_wxyz = np.array(
                [float(quat_xyzw[3]), float(quat_xyzw[0]), float(quat_xyzw[1]), float(quat_xyzw[2])],
                dtype=float,
            )
            arm_ent.set_pos(new_pos)
            arm_ent.set_quat(quat_wxyz)
        except Exception:
            pass

    @staticmethod
    def _to_numpy_1d(raw) -> np.ndarray:
        if hasattr(raw, "detach"):
            raw = raw.detach()
        if hasattr(raw, "cpu"):
            raw = raw.cpu()
        if hasattr(raw, "numpy"):
            raw = raw.numpy()
        return np.asarray(raw, dtype=float).reshape(-1)

    def draw_marker(self, markers: MarkerSet, attr_name: str, pos: np.ndarray, color) -> None:
        if self.scene is None:
            return
        markers.draw(self.scene, attr_name, pos, color)

    def draw_marker_direction(self, markers: MarkerSet, attr_name: str, pos: np.ndarray, direction: np.ndarray, color) -> None:
        if self.scene is None:
            return
        markers.draw_direction(self.scene, attr_name, pos, direction, color)

    def actual_tip_world(self, layout: JointLayout) -> Optional[np.ndarray]:
        if self.mover is None:
            return None
        try:
            if layout.tip_points:
                pts: List[np.ndarray] = []
                for link_name, local_offset in layout.tip_points:
                    link = self.mover.entity.get_link(link_name)
                    p = self._to_numpy_1d(link.get_pos())[:3]
                    q_wxyz = self._to_numpy_1d(link.get_quat())[:4]
                    local = np.asarray(local_offset, dtype=float).reshape(3)
                    tip = gs_geom.transform_by_trans_quat(local, p, q_wxyz)
                    pts.append(np.array(tip, dtype=float))
                if pts:
                    return np.mean(np.stack(pts, axis=0), axis=0)
            if not layout.tip_link_name:
                return None
            link = self.mover.entity.get_link(layout.tip_link_name)
            p = self._to_numpy_1d(link.get_pos())[:3]
            q_wxyz = self._to_numpy_1d(link.get_quat())[:4]
            local = np.asarray(layout.tip_local_offset, dtype=float).reshape(3)
            tip = gs_geom.transform_by_trans_quat(local, p, q_wxyz)
            return np.array(tip, dtype=float)
        except Exception:
            return None

    def actual_tip_direction_world(self, layout: JointLayout) -> Optional[np.ndarray]:
        if self.mover is None:
            return None
        try:
            local_axis = np.asarray(layout.approach_axis_local, dtype=float).reshape(3)
            axis_norm = float(np.linalg.norm(local_axis))
            if axis_norm <= 1e-9 or not layout.tip_link_name:
                return None
            link = self.mover.entity.get_link(str(layout.tip_link_name))
            q_wxyz = self._to_numpy_1d(link.get_quat())[:4]
            R_tip = _rot_from_wxyz(q_wxyz).as_matrix()
            direction = R_tip @ np.asarray(layout.approach_rot_tip, dtype=float).reshape(3, 3) @ (local_axis / axis_norm)
            norm = float(np.linalg.norm(direction))
            if norm <= 1e-9:
                return None
            return (direction / norm).reshape(3)
        except Exception:
            return None

    def desired_tip_pos_from_cmd_target(
        self,
        layout: JointLayout,
        model: SpawnConfig,
        q_target_full: np.ndarray,
    ) -> Optional[np.ndarray]:
        if self.mover is None or not layout.fk_joint_chain:
            return None
        try:
            q_vals = np.asarray(q_target_full, dtype=float).reshape(-1)
            q_map = {name: float(q_vals[i]) for i, name in enumerate(self.mover.dof_names()) if i < q_vals.size}

            spawn_pos = np.array(model.spawn_xyz, dtype=float).reshape(3)
            spawn_euler = np.array(model.spawn_euler_deg, dtype=float).reshape(3)
            R_spawn = Rot.from_euler("xyz", spawn_euler, degrees=True).as_matrix()

            link_tf: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
            root = layout.fk_root_link
            p_root_local = layout.part_pose_root.get(root, np.array([0.0, 0.0, 0.0], dtype=float))
            link_tf[root] = (spawn_pos + R_spawn @ p_root_local, R_spawn.copy())

            for meta in layout.fk_joint_chain:
                parent = str(meta["parent"])
                child = str(meta["child"])
                if parent not in link_tf:
                    continue
                p_parent, R_parent = link_tf[parent]
                origin_parent = np.asarray(meta["origin_parent"], dtype=float).reshape(3)
                axis_parent = np.asarray(meta["axis_parent"], dtype=float).reshape(3)
                child_rot_parent = np.asarray(meta.get("child_rot_parent", np.eye(3)), dtype=float).reshape(3, 3)
                q = float(q_map.get(str(meta["name"]), 0.0))
                if str(meta["type"]) == "prismatic":
                    p_child = p_parent + R_parent @ (origin_parent + axis_parent * q)
                    R_child = R_parent @ child_rot_parent
                elif str(meta["type"]) == "revolute":
                    p_child = p_parent + R_parent @ origin_parent
                    R_child = R_parent @ Rot.from_rotvec(axis_parent * q).as_matrix() @ child_rot_parent
                else:
                    p_child = p_parent + R_parent @ origin_parent
                    R_child = R_parent @ child_rot_parent
                link_tf[child] = (p_child, R_child)

            if layout.tip_points:
                pts: List[np.ndarray] = []
                for link_name, local_offset in layout.tip_points:
                    if link_name not in link_tf:
                        continue
                    p_tip, R_tip = link_tf[link_name]
                    tip_world = p_tip + R_tip @ np.asarray(local_offset, dtype=float).reshape(3)
                    pts.append(np.array(tip_world, dtype=float))
                if pts:
                    return np.mean(np.stack(pts, axis=0), axis=0)
            if not layout.tip_link_name or layout.tip_link_name not in link_tf:
                return None
            p_tip, R_tip = link_tf[layout.tip_link_name]
            tip_world = p_tip + R_tip @ np.asarray(layout.tip_local_offset, dtype=float).reshape(3)
            return np.array(tip_world, dtype=float)
        except Exception:
            return None

    def desired_tip_dir_from_cmd_target(
        self,
        layout: JointLayout,
        model: SpawnConfig,
        q_target_full: np.ndarray,
    ) -> Optional[np.ndarray]:
        if self.mover is None or not layout.fk_joint_chain:
            return None
        try:
            q_vals = np.asarray(q_target_full, dtype=float).reshape(-1)
            q_map = {name: float(q_vals[i]) for i, name in enumerate(self.mover.dof_names()) if i < q_vals.size}

            spawn_pos = np.array(model.spawn_xyz, dtype=float).reshape(3)
            spawn_euler = np.array(model.spawn_euler_deg, dtype=float).reshape(3)
            R_spawn = Rot.from_euler("xyz", spawn_euler, degrees=True).as_matrix()

            link_tf: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
            root = layout.fk_root_link
            p_root_local = layout.part_pose_root.get(root, np.array([0.0, 0.0, 0.0], dtype=float))
            link_tf[root] = (spawn_pos + R_spawn @ p_root_local, R_spawn.copy())

            for meta in layout.fk_joint_chain:
                parent = str(meta["parent"])
                child = str(meta["child"])
                if parent not in link_tf:
                    continue
                p_parent, R_parent = link_tf[parent]
                origin_parent = np.asarray(meta["origin_parent"], dtype=float).reshape(3)
                axis_parent = np.asarray(meta["axis_parent"], dtype=float).reshape(3)
                child_rot_parent = np.asarray(meta.get("child_rot_parent", np.eye(3)), dtype=float).reshape(3, 3)
                q = float(q_map.get(str(meta["name"]), 0.0))
                if str(meta["type"]) == "prismatic":
                    p_child = p_parent + R_parent @ (origin_parent + axis_parent * q)
                    R_child = R_parent @ child_rot_parent
                elif str(meta["type"]) == "revolute":
                    p_child = p_parent + R_parent @ origin_parent
                    R_child = R_parent @ Rot.from_rotvec(axis_parent * q).as_matrix() @ child_rot_parent
                else:
                    p_child = p_parent + R_parent @ origin_parent
                    R_child = R_parent @ child_rot_parent
                link_tf[child] = (p_child, R_child)

            local_axis = np.asarray(layout.approach_axis_local, dtype=float).reshape(3)
            norm_local = float(np.linalg.norm(local_axis))
            if norm_local <= 1e-9 or not layout.tip_link_name or layout.tip_link_name not in link_tf:
                return None
            _p_tip, R_tip = link_tf[layout.tip_link_name]
            direction = R_tip @ np.asarray(layout.approach_rot_tip, dtype=float).reshape(3, 3) @ (local_axis / norm_local)
            norm = float(np.linalg.norm(direction))
            if norm <= 1e-9:
                return None
            return (direction / norm).reshape(3)
        except Exception:
            return None

    def apply_sim_q(self, q_errmodel: proto.SimQ) -> Optional[np.ndarray]:
        if self.mover is None:
            return None
        self.mover.control_4dof(
            float(q_errmodel.linear_m),
            float(q_errmodel.roll_rad),
            float(q_errmodel.theta1_rad),
            float(q_errmodel.theta2_rad),
        )
        return self.mover.target_from_4dof(
            float(q_errmodel.linear_m),
            float(q_errmodel.roll_rad),
            float(q_errmodel.theta1_rad),
            float(q_errmodel.theta2_rad),
        )

    def step(self) -> None:
        if self.go2 is not None:
            self.go2.step()
        if self.scene is not None:
            self.scene.step()

    def reset_environment(self) -> None:
        if self.scene is not None:
            try:
                self.scene.reset()
            except Exception as exc:
                print(f"[sim] scene.reset failed: {exc}")
        if self.go2 is not None:
            self.go2.reset_locomotion()
        if self.mover is not None:
            self.mover.set_4dof_instant(0.0, 0.0, 0.0, 0.0)
            self.mover.set_claw_closed(False)
        for entity in (self.go2_entity, getattr(self.mover, "entity", None) if self.mover is not None else None):
            if entity is None:
                continue
            try:
                entity.zero_all_dofs_velocity()
            except Exception:
                pass
        print("[sim] environment reset")

    def maybe_publish_camera(
        self,
        *,
        arm_q: Optional[tuple[float, float, float, float]],
        max_hz: float,
        force: bool = False,
        rgb_enabled: bool = True,
        depth_enabled: bool = True,
    ) -> None:
        if self.eye_camera is None or self.camera_publisher is None:
            return
        import time

        period = 1.0 / max(1.0, float(max_hz))
        now = time.time()
        if not force and (now - float(self._last_camera_publish_t)) < period:
            return
        try:
            frame = self.eye_camera.capture(
                arm_q=arm_q,
                ts=now,
                rgb_enabled=bool(rgb_enabled),
                depth_enabled=bool(depth_enabled),
            )
            if self.camera_publisher.publish(frame):
                self._last_camera_publish_t = now
        except Exception as exc:
            print(f"[sim_camera] capture/publish failed: {exc}")

    def camera_axes_world(self, *, hand_eye_path: str, parent_link: str = "node9") -> Optional[tuple[np.ndarray, np.ndarray, np.ndarray]]:
        if self.eye_camera is not None:
            axes = self.eye_camera.camera_axes_world()
            if axes is not None:
                origin, look, right = axes
                return (
                    np.asarray(origin, dtype=float).reshape(3),
                    np.asarray(look, dtype=float).reshape(3),
                    np.asarray(right, dtype=float).reshape(3),
                )
        if self.mover is None:
            return None
        try:
            from engine.sim_camera.pose import camera_axes_from_genesis_link

            return camera_axes_from_genesis_link(
                self.mover.entity,
                hand_eye_path=str(hand_eye_path),
                parent_link=str(parent_link),
            )
        except Exception as exc:
            print(f"[sim_camera] camera_axes_world failed: {exc}")
            return None


class RateLimiter:
    def __init__(self, max_rate: np.ndarray):
        self._max_rate = np.array(max_rate, dtype=float)

    def step(self, q_cmd: np.ndarray, q_target: np.ndarray, dt: float) -> np.ndarray:
        dq = q_target - q_cmd
        max_step = self._max_rate * dt
        dq_clamped = np.clip(dq, -max_step, +max_step)
        return q_cmd + dq_clamped


class SimMover:
    def __init__(
        self,
        entity,
        params: SimParam,
        limit: JointLimit,
        n_nodes: int,
        n_seg: Optional[int] = None,
        *,
        linear_joint_name: str = "j_plate_housing",
        roll_joint_name: str = "base_roll_x",
        bend_joint_names: Optional[List[str]] = None,
    ):
        self.entity = entity
        self.p = params
        self.limit = limit
        self.n_nodes = int(n_nodes)
        self.n_seg = int(n_seg) if n_seg is not None else (self.n_nodes // 2)
        self._linear_joint_name = str(linear_joint_name)
        self._roll_joint_name = str(roll_joint_name)
        self._bend_joint_names = (
            [str(x) for x in bend_joint_names]
            if bend_joint_names is not None
            else [f"bend_{i}" for i in range(self.n_nodes)]
        )

        bend_names = [self._linear_joint_name, self._roll_joint_name] + list(self._bend_joint_names)

        pairs: List[Tuple[int, str]] = []
        for name in bend_names:
            j = self.entity.get_joint(name)
            idx = getattr(j, "dofs_idx_local")
            pairs.append((_as_single_dof_index(idx), name))

        pairs.sort(key=lambda t: int(t[0]))
        self.joint_names: List[str] = [n for _, n in pairs]
        self.dofs_idx_local: List[int] = [int(i) for i, _ in pairs]

        self._name2pos: Dict[str, int] = {n: k for k, n in enumerate(self.joint_names)}
        self._linear_pos: Optional[int] = self._name2pos.get(self._linear_joint_name)
        self._bend_pos: List[int] = [self._name2pos[name] for name in self._bend_joint_names]

        self.bend_lim = float(limit.bend_lim_rad())
        max_rate = np.array(
            [float(params.bend_rate), float(params.roll_rate)] + [float(params.bend_rate)] * self.n_nodes,
            dtype=float,
        )
        self._rate = RateLimiter(max_rate=max_rate)

        try:
            raw0 = self.entity.get_dofs_position(dofs_idx_local=self.dofs_idx_local)
            q0 = _to_numpy_1d(raw0)
            if q0.shape[0] != len(self.dofs_idx_local):
                q0 = q0[: len(self.dofs_idx_local)]
            self._q_cmd = q0.copy()
        except Exception:
            self._q_cmd = np.zeros(len(self.dofs_idx_local), dtype=float)

        self._last_q_target: Optional[np.ndarray] = None
        self._last_q_target_cmd: Optional[Tuple[float, float, float, float]] = None
        self._sag_model: dict[str, Any] = {}
        self._claw_left_idx: Optional[int] = None
        self._claw_right_idx: Optional[int] = None
        self._claw_closed: bool = False
        self._claw_left_cmd: float = 0.0
        self._claw_right_cmd: float = 0.0
        self._claw_left_target: float = 0.0
        self._claw_right_target: float = 0.0
        self._claw_rate: float = 0.08
        for joint_name, attr_name in (
            ("j_gripper_base_claw_left", "_claw_left_idx"),
            ("j_gripper_base_claw_right", "_claw_right_idx"),
        ):
            try:
                j = self.entity.get_joint(joint_name)
                setattr(self, attr_name, _as_single_dof_index(getattr(j, "dofs_idx_local")))
            except Exception:
                setattr(self, attr_name, None)
        self._apply_claw_direct(self._claw_left_cmd, self._claw_right_cmd)

    def idx_roll(self) -> Optional[int]:
        return self._name2pos.get(self._roll_joint_name, None)

    def idx_linear(self) -> Optional[int]:
        return self._linear_pos

    def bend_indices(self) -> List[int]:
        return list(self._bend_pos)

    def dof_names(self) -> List[str]:
        return list(self.joint_names)

    def dof_count(self) -> int:
        return int(len(self.joint_names))

    def get_dofs_position(self) -> np.ndarray:
        raw = self.entity.get_dofs_position(dofs_idx_local=self.dofs_idx_local)
        q = _to_numpy_1d(raw)
        if q.shape[0] != len(self.dofs_idx_local):
            q = q[: len(self.dofs_idx_local)]
        return q

    def get_last_target_full(self) -> Optional[np.ndarray]:
        return None if self._last_q_target is None else self._last_q_target.copy()

    def get_last_command_full(self) -> np.ndarray:
        return self._q_cmd.copy()

    def _apply_q_direct(self, q_target: np.ndarray) -> None:
        self.entity.set_dofs_position(q_target, dofs_idx_local=self.dofs_idx_local)

    def set_sag_model(self, sag_model: dict[str, Any]) -> None:
        self._sag_model = dict(sag_model or {})

    def _apply_claw_direct(self, left_value: float, right_value: float) -> None:
        if self._claw_left_idx is not None:
            self.entity.set_dofs_position(np.array([left_value], dtype=float), dofs_idx_local=[self._claw_left_idx])
        if self._claw_right_idx is not None:
            self.entity.set_dofs_position(np.array([right_value], dtype=float), dofs_idx_local=[self._claw_right_idx])

    def _step_claws(self) -> None:
        max_step = float(self._claw_rate) * float(self.p.dt)
        self._claw_left_cmd = float(np.clip(self._claw_left_target, self._claw_left_cmd - max_step, self._claw_left_cmd + max_step))
        self._claw_right_cmd = float(np.clip(self._claw_right_target, self._claw_right_cmd - max_step, self._claw_right_cmd + max_step))
        self._apply_claw_direct(self._claw_left_cmd, self._claw_right_cmd)

    def set_claw_closed(self, closed: bool) -> None:
        self._claw_closed = bool(closed)
        self._claw_left_target = -0.02 if self._claw_closed else 0.0
        self._claw_right_target = 0.02 if self._claw_closed else 0.0

    def target_from_4dof(self, linear_m: float, roll: float, theta1: float, theta2: float) -> np.ndarray:
        linear = float(np.clip(float(linear_m), -0.230, 0.010))
        rl = float(np.clip(float(roll), self.limit.roll_min_rad(), self.limit.roll_max_rad()))
        t1 = float(np.clip(float(theta1), -self.bend_lim, +self.bend_lim))
        t2 = float(np.clip(float(theta2), -self.bend_lim, +self.bend_lim))
        t1_deg = float(np.degrees(t1))
        t2_deg = float(np.degrees(t2))
        seg1_err = np.radians(
            segment_errors_from_model(
                self._sag_model,
                seg_index=1,
                count=self.n_seg,
                theta1=t1_deg,
                theta2=t2_deg,
            )
        )
        seg2_err = np.radians(
            segment_errors_from_model(
                self._sag_model,
                seg_index=2,
                count=max(self.n_nodes - self.n_seg, 0),
                theta1=t1_deg,
                theta2=t2_deg,
            )
        )

        vals: Dict[str, float] = {self._linear_joint_name: linear, self._roll_joint_name: rl}
        for i in range(self.n_nodes):
            base = t1 if i < self.n_seg else t2
            err = float(seg1_err[i]) if i < self.n_seg else float(seg2_err[i - self.n_seg])
            vals[self._bend_joint_names[i]] = float(np.clip(base + err, -self.bend_lim, +self.bend_lim))

        return np.array([vals[n] for n in self.joint_names], dtype=float)

    def control_4dof(self, linear_m: float, roll: float, theta1: float, theta2: float):
        q_target = self.target_from_4dof(linear_m, roll, theta1, theta2)
        self._last_q_target = q_target
        self._last_q_target_cmd = (float(linear_m), float(roll), float(theta1), float(theta2))
        self._q_cmd = self._rate.step(self._q_cmd, q_target, dt=float(self.p.dt))
        self._apply_q_direct(self._q_cmd)
        self._step_claws()

    def set_4dof_instant(self, linear_m: float, roll: float, theta1: float, theta2: float) -> None:
        q_target = self.target_from_4dof(linear_m, roll, theta1, theta2)
        self._last_q_target = q_target
        self._last_q_target_cmd = (float(linear_m), float(roll), float(theta1), float(theta2))
        self._q_cmd = q_target.copy()
        self._apply_q_direct(q_target)
        self._step_claws()


class AssetProcessor:
    """Orchestrate asset prep: ensure manifest json exists, then convert JSON to URDF."""

    def __init__(self, app: "GenesisApp"):
        self.app = app

    def _json_path(self) -> str:
        c = self.app.cfg
        return os.path.join(c.build_dir, c.assy_build_json)

    def _urdf_path(self) -> str:
        c = self.app.cfg
        return os.path.join(c.build_dir, c.urdf_name)

    def _arm_urdf_path(self) -> str:
        c = self.app.cfg
        return os.path.join(c.build_dir, c.arm_urdf_name)

    def prepare_assets(self) -> str:
        t0 = time.time()
        in_json = self._json_path()
        arm_urdf = self._arm_urdf_path()
        robot_urdf = self._urdf_path()
        if self.app.cfg.rebuild_assembly or (not os.path.isfile(in_json)):
            os.makedirs(self.app.cfg.build_dir, exist_ok=True)
            try:
                assembly_builder.build_default_manifest(
                    self.app.cfg.build_dir,
                    use_hardware=bool(self.app.cfg.use_hardware),
                    use_go2=bool(getattr(self.app.cfg, "use_go2", False)),
                )
            except Exception as e:
                raise RuntimeError(f"Auto build failed for {self.app.cfg.assy_build_json}: {e}") from e
            if not os.path.isfile(in_json):
                raise FileNotFoundError(f"manifest json not found after auto-build: {in_json}")

        manifest_go2_found = False
        manifest_parts_count = 0
        try:
            with open(in_json, "r", encoding="utf-8") as f:
                manifest = json.load(f)
            parts = list(manifest.get("parts", []) or [])
            manifest_parts_count = int(len(parts))
            manifest_go2_found = any(str(p.get("name", "")).strip().lower() == "go2" for p in parts if isinstance(p, dict))
        except Exception as exc:
            print(f"[runtime] manifest inspect skipped: {exc}")
        print(
            "[runtime] use_go2=%s manifest_parts=%d go2_part=%s"
            % (
                str(bool(getattr(self.app.cfg, "use_go2", False))).lower(),
                int(manifest_parts_count),
                str(bool(manifest_go2_found)).lower(),
            )
        )

        self._load_joint_layout(in_json)
        self.app._apply_ideal_rates_if_needed()
        convert_manifest_file(in_json, arm_urdf, cfg=self.app.urdf_export_cfg)
        if bool(getattr(self.app.cfg, "use_go2", False)):
            go2_urdf = RuntimePrep._resolve_genesis_go2_urdf()
            go2_urdf = _prepare_go2_urdf_with_config_colors(
                go2_urdf,
                build_dir=self.app.cfg.build_dir,
                colors=self.app.urdf_export_cfg.part_color_rgba_by_name,
            )
            merge_go2_arm_urdf(
                go2_urdf_path=go2_urdf,
                arm_urdf_path=arm_urdf,
                out_urdf_path=robot_urdf,
                mount_xyz=tuple(float(x) for x in self.app.spawn.go2_mount_offset_m),
            )
            print(f"[runtime] combined GO2+arm URDF saved: {robot_urdf}")
        else:
            convert_manifest_file(in_json, robot_urdf, cfg=self.app.urdf_export_cfg)
        print(f"[runtime] use_hardware = {str(bool(self.app.cfg.use_hardware)).lower()}")
        print("[runtime] assets prepared in %.2fs" % (time.time() - t0))
        return arm_urdf

    def _load_joint_layout(self, json_path: str) -> None:
        with open(json_path, "r", encoding="utf-8") as f:
            build = json.load(f)
        def _pick_manifest_value(mapping, *keys, default=None):
            for key in keys:
                if isinstance(mapping, dict) and key in mapping:
                    return mapping[key]
            return default

        joints = list(_pick_manifest_value(build, "joints", default=[]))
        parts = list(_pick_manifest_value(build, "parts", default=[]))
        raw_pairs = list(_pick_manifest_value(build, "no_clip_pairs", default=[]) or [])
        if not joints:
            raise RuntimeError("manifest json is missing joints")
        if not parts:
            raise RuntimeError("manifest json is missing parts")

        linear_joint_name = ""
        revolute_names: List[str] = []
        for j in joints:
            jname = str(_pick_manifest_value(j, "name", default="")).strip()
            jtype = str(_pick_manifest_value(j, "type", default="")).strip().lower()
            if not jname:
                continue
            if jtype == "prismatic" and jname == "j_plate_housing":
                linear_joint_name = jname
            if jtype == "revolute":
                revolute_names.append(jname)

        if not linear_joint_name:
            raise RuntimeError("manifest json does not provide linear control joint j_plate_housing.")
        if len(revolute_names) < 3:
            raise RuntimeError("manifest json does not provide enough control joints (need >=3 revolute).")

        self.app.layout.linear_joint_name = linear_joint_name
        self.app.layout.roll_joint_name = revolute_names[0]
        self.app.layout.bend_joint_names = revolute_names[1:]
        joint_by_name = {str(_pick_manifest_value(j, "name", default="")): j for j in joints}
        first_bend = joint_by_name.get(self.app.layout.bend_joint_names[0]) if self.app.layout.bend_joint_names else None
        if first_bend is None:
            raise RuntimeError("manifest json is missing first bend joint metadata")
        ar = _pick_manifest_value(first_bend, "anchor_root", default=None)
        if not isinstance(ar, (list, tuple)) or len(ar) != 3:
            raise RuntimeError("manifest json is missing valid anchor_root for first bend joint")
        self.app.layout.chain_origin_local = np.array([float(ar[0]), float(ar[1]), float(ar[2])], dtype=float)

        terminal_joint = joint_by_name.get(self.app.layout.bend_joint_names[-1]) if self.app.layout.bend_joint_names else None
        tip_link_name = str(_pick_manifest_value(terminal_joint, "child", default="")) if terminal_joint is not None else ""
        if not tip_link_name:
            raise RuntimeError("manifest json is missing terminal bend child link")
        tip_local_offset = np.array([0.0, 0.0, 0.0], dtype=float)
        tip_points: List[Tuple[str, np.ndarray]] = []
        part_control_mode: Dict[str, str] = {}
        controlled_modes: List[str] = []
        for p in parts:
            name = str(_pick_manifest_value(p, "name", default="")).strip()
            flags = _pick_manifest_value(p, "flags", default={}) or {}
            mode = str(_pick_manifest_value(flags, "control_mode", default=_pick_manifest_value(flags, "ControlMode", default="fixed"))).strip().lower() or "fixed"
            if name:
                part_control_mode[name] = mode
            kind = str(_pick_manifest_value(p, "kind", default="")).strip().lower()
            if kind in ("housing", "wedge", "node", "node_end"):
                controlled_modes.append(mode)
        part_by_name = {str(_pick_manifest_value(p, "name", default="")): p for p in parts}
        def _load_tip_offset(part_name: str) -> np.ndarray:
            part = part_by_name.get(part_name)
            if part is None:
                raise RuntimeError(f"manifest json is missing part entry for tip link '{part_name}'")
            assets = _pick_manifest_value(part, "assets", default={}) or {}
            frame_rel = str(_pick_manifest_value(assets, "frame", default="") or "")
            if not frame_rel:
                raise RuntimeError(f"manifest json is missing frame asset for tip part '{part_name}'")
            frame_abs = os.path.join(self.app.cfg.build_dir, frame_rel)
            with open(frame_abs, "r", encoding="utf-8") as ff:
                frame_json = json.load(ff)
            connectors = _pick_manifest_value(frame_json, "connectors", default={}) or {}
            to_raw = _pick_manifest_value(connectors, "to", default=None)
            if isinstance(to_raw, dict):
                to_raw = _pick_manifest_value(to_raw, "p", default=None)
            if not isinstance(to_raw, (list, tuple)) or len(to_raw) != 3:
                raise RuntimeError(f"frame json is missing valid connectors.to for tip part '{part_name}'")
            return np.array([float(to_raw[0]), float(to_raw[1]), float(to_raw[2])], dtype=float)

        if "gripper_claw_left" in part_by_name and "gripper_claw_right" in part_by_name:
            pose_root_by_name: Dict[str, Tuple[np.ndarray, Rot]] = {}
            for p in parts:
                name = str(_pick_manifest_value(p, "name", default="")).strip()
                pose_root = _pick_manifest_value(p, "pose_root", default={}) or {}
                pr = _pick_manifest_value(pose_root, "p", default=None)
                qr = _pick_manifest_value(pose_root, "q", default=None)
                if not name:
                    continue
                if not (isinstance(pr, (list, tuple)) and len(pr) == 3 and isinstance(qr, (list, tuple)) and len(qr) == 4):
                    continue
                p_root = np.array([float(pr[0]), float(pr[1]), float(pr[2])], dtype=float)
                q_xyzw = np.array([float(qr[0]), float(qr[1]), float(qr[2]), float(qr[3])], dtype=float)
                pose_root_by_name[name] = (p_root, Rot.from_quat(q_xyzw))

            left_local = _load_tip_offset("gripper_claw_left")
            right_local = _load_tip_offset("gripper_claw_right")
            tip_points = [
                ("gripper_claw_left", left_local),
                ("gripper_claw_right", right_local),
            ]
            left_pose = pose_root_by_name.get("gripper_claw_left")
            right_pose = pose_root_by_name.get("gripper_claw_right")
            tip_pose = pose_root_by_name.get(tip_link_name)
            base_pose = pose_root_by_name.get("gripper_base")
            old_tip_local = _load_tip_offset(tip_link_name)
            if left_pose is not None and right_pose is not None and tip_pose is not None:
                left_world = left_pose[0] + left_pose[1].apply(left_local)
                right_world = right_pose[0] + right_pose[1].apply(right_local)
                grasp_mid_world = 0.5 * (left_world + right_world)
                old_tip_world = tip_pose[0] + tip_pose[1].apply(old_tip_local)
                delta_local = tip_pose[1].inv().apply(grasp_mid_world - old_tip_world)
                tip_local_offset = old_tip_local + np.asarray(delta_local, dtype=float).reshape(3)
                if base_pose is not None:
                    self.app.layout.approach_axis_local = np.array([0.0, 0.0, -1.0], dtype=float)
                    self.app.layout.approach_rot_tip = (tip_pose[1].inv() * base_pose[1]).as_matrix()
        else:
            tip_local_offset = _load_tip_offset(tip_link_name)
            old_tip_local = tip_local_offset.copy()
        self.app.layout.tip_link_name = tip_link_name
        self.app.layout.tip_local_offset = tip_local_offset
        self.app.layout.old_tip_local_offset = np.asarray(old_tip_local, dtype=float).reshape(3)
        self.app.layout.tip_points = tip_points
        self.app.layout.part_control_mode = part_control_mode
        if controlled_modes:
            uniq = sorted(set(controlled_modes))
            self.app.layout.control_mode = uniq[0]
            if len(uniq) > 1:
                print(f"[runtime] mixed controlled part modes {uniq}; using chain mode '{self.app.layout.control_mode}'")
        else:
            self.app.layout.control_mode = "commanded"
        no_clip_pairs: List[Tuple[str, str]] = []
        for item in raw_pairs:
            if isinstance(item, (list, tuple)) and len(item) == 2:
                a0 = str(item[0]).strip()
                a1 = str(item[1]).strip()
                if a0 and a1 and a0 != a1:
                    no_clip_pairs.append((a0, a1))
        self.app.layout.no_clip_pairs = no_clip_pairs

        # IK sign convention: convert actual joint axis (+/-X, +/-Y) into scalar signs.
        def _axis_sign(raw_axis, axis_idx: int, *, name: str) -> float:
            a = np.asarray(raw_axis, dtype=float).reshape(-1)
            if a.size <= axis_idx:
                raise RuntimeError(f"manifest json is missing valid axis_root for {name}")
            v = float(a[axis_idx])
            if abs(v) < 1e-9:
                raise RuntimeError(f"manifest json axis_root for {name} has zero component on required axis")
            return -1.0 if v < 0.0 else 1.0

        linear_meta = joint_by_name.get(self.app.layout.linear_joint_name, {}) if self.app.layout.linear_joint_name else {}
        roll_meta = joint_by_name.get(self.app.layout.roll_joint_name, {}) if self.app.layout.roll_joint_name else {}
        bend_meta = joint_by_name.get(self.app.layout.bend_joint_names[0], {}) if self.app.layout.bend_joint_names else {}
        self.app.layout.linear_axis_sign = _axis_sign(_pick_manifest_value(linear_meta, "axis_root", default=None), 0, name=self.app.layout.linear_joint_name)
        self.app.layout.roll_axis_sign = _axis_sign(_pick_manifest_value(roll_meta, "axis_root", default=None), 0, name=self.app.layout.roll_joint_name)
        self.app.layout.bend_axis_sign = _axis_sign(_pick_manifest_value(bend_meta, "axis_root", default=None), 1, name=self.app.layout.bend_joint_names[0])

        part_pose_root: Dict[str, np.ndarray] = {}
        part_rot_root: Dict[str, np.ndarray] = {}
        for p in parts:
            name = str(_pick_manifest_value(p, "name", default=""))
            pose_root = _pick_manifest_value(p, "pose_root", default={}) or {}
            pr = _pick_manifest_value(pose_root, "p", default=[0.0, 0.0, 0.0])
            qr = _pick_manifest_value(pose_root, "q", default=[0.0, 0.0, 0.0, 1.0])
            if isinstance(pr, (list, tuple)) and len(pr) == 3:
                part_pose_root[name] = np.array([float(pr[0]), float(pr[1]), float(pr[2])], dtype=float)
            if isinstance(qr, (list, tuple)) and len(qr) == 4:
                part_rot_root[name] = np.array([float(qr[0]), float(qr[1]), float(qr[2]), float(qr[3])], dtype=float)
        self.app.layout.part_pose_root = part_pose_root
        self.app.layout.part_rot_root = part_rot_root

        parent_of: Dict[str, str] = {}
        for j in joints:
            parent = str(_pick_manifest_value(j, "parent", default=""))
            child = str(_pick_manifest_value(j, "child", default=""))
            if parent and child:
                parent_of[child] = parent
        roots = [name for name in part_pose_root.keys() if name not in parent_of]
        if not roots:
            raise RuntimeError("manifest json does not provide a root link")
        self.app.layout.fk_root_link = roots[0]

        fk_chain = []
        for j in joints:
            jn = str(_pick_manifest_value(j, "name", default="")).strip()
            if not jn:
                continue
            parent = str(_pick_manifest_value(j, "parent", default=""))
            child = str(_pick_manifest_value(j, "child", default=""))
            jtype = str(_pick_manifest_value(j, "type", default="")).strip().lower()
            anchor = _pick_manifest_value(j, "anchor_root", default=[0.0, 0.0, 0.0])
            axis = _pick_manifest_value(j, "axis_root", default=[1.0, 0.0, 0.0])
            p_parent = part_pose_root.get(parent, np.array([0.0, 0.0, 0.0], dtype=float))
            q_parent = part_rot_root.get(parent, np.array([0.0, 0.0, 0.0, 1.0], dtype=float))
            q_child = part_rot_root.get(child, np.array([0.0, 0.0, 0.0, 1.0], dtype=float))
            origin_parent = np.array(
                [float(anchor[0]) - float(p_parent[0]), float(anchor[1]) - float(p_parent[1]), float(anchor[2]) - float(p_parent[2])],
                dtype=float,
            )
            axis_parent = np.array([float(axis[0]), float(axis[1]), float(axis[2])], dtype=float)
            n = float(np.linalg.norm(axis_parent))
            if n > 1e-12:
                axis_parent /= n
            R_parent0 = Rot.from_quat(q_parent)
            R_child0 = Rot.from_quat(q_child)
            R_child_rel = (R_parent0.inv() * R_child0).as_matrix()
            fk_chain.append(
                {
                    "name": jn,
                    "type": jtype,
                    "parent": parent,
                    "child": child,
                    "origin_parent": origin_parent,
                    "axis_parent": axis_parent,
                    "child_rot_parent": R_child_rel,
                }
            )
        self.app.layout.fk_joint_chain = fk_chain


class StateSource:
    """Abstract source of 3-DOF chain state for the SIM runtime."""

    def poll(self) -> None:
        return None

    def estimate_q(self) -> Optional[proto.SimQ]:
        return None

    def ik_target_xyz(self) -> Optional[np.ndarray]:
        return None

    def ik_target_dir(self) -> Optional[np.ndarray]:
        return None

    def sag_model(self) -> dict[str, Any]:
        return {}

    def claw_closed(self) -> bool:
        return False

    def go2_vel(self) -> tuple[float, float, float]:
        return (0.0, 0.0, 0.0)

    def go2_base_pos(self) -> Optional[tuple[float, float, float]]:
        return None

    def go2_base_rpy(self) -> Optional[tuple[float, float, float]]:
        return None

    def go2_leg_q(self) -> Optional[tuple[float, ...]]:
        return None

    def sim_reset_seq(self) -> int:
        return 0

    def debug_markers(self) -> list[dict[str, Any]]:
        return []

    def close(self) -> None:
        return None


class HardwareStateCache(StateSource):
    """
    Direct passthrough cache of the latest host-published state.
    Future IMU/AruCo/camera fusion should implement the same interface.
    """

    def __init__(self) -> None:
        self._last_q: Optional[proto.SimQ] = None
        self._last_ik_target_xyz: Optional[np.ndarray] = None
        self._last_ik_target_dir: Optional[np.ndarray] = None
        self._last_sag_model: dict[str, Any] = {}
        self._last_claw_closed: bool = False
        self._last_go2_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_go2_base_pos: Optional[tuple[float, float, float]] = None
        self._last_go2_base_rpy: Optional[tuple[float, float, float]] = None
        self._last_go2_leg_q: Optional[tuple[float, ...]] = None
        self._last_sim_reset_seq: int = 0
        self._last_debug_markers: list[dict[str, Any]] = []

    def update(
        self,
        q: proto.SimQ,
        ik_target_xyz: Optional[np.ndarray] = None,
        ik_target_dir: Optional[np.ndarray] = None,
        sag_model: Optional[dict[str, Any]] = None,
    ) -> None:
        self._last_q = q
        self._last_ik_target_xyz = None if ik_target_xyz is None else np.array(ik_target_xyz, dtype=float).reshape(3)
        self._last_ik_target_dir = None if ik_target_dir is None else np.array(ik_target_dir, dtype=float).reshape(3)
        if sag_model is not None:
            self._last_sag_model = dict(sag_model)
    def update_claw_closed(self, claw_closed: bool) -> None:
        self._last_claw_closed = bool(claw_closed)

    def update_go2_vel(self, go2_vel: tuple[float, float, float]) -> None:
        self._last_go2_vel = (float(go2_vel[0]), float(go2_vel[1]), float(go2_vel[2]))

    def update_go2_base(
        self,
        go2_base_pos: Optional[tuple[float, float, float]],
        go2_base_rpy: Optional[tuple[float, float, float]],
    ) -> None:
        if go2_base_pos is not None:
            self._last_go2_base_pos = (float(go2_base_pos[0]), float(go2_base_pos[1]), float(go2_base_pos[2]))
        if go2_base_rpy is not None:
            self._last_go2_base_rpy = (float(go2_base_rpy[0]), float(go2_base_rpy[1]), float(go2_base_rpy[2]))

    def update_go2_leg_q(self, go2_leg_q: Optional[tuple[float, ...]]) -> None:
        if go2_leg_q is not None and len(go2_leg_q) == 12:
            self._last_go2_leg_q = tuple(float(v) for v in go2_leg_q)

    def update_sim_reset_seq(self, seq: int) -> None:
        self._last_sim_reset_seq = int(seq)

    def update_ik_target(self, ik_target_xyz: Optional[np.ndarray]) -> None:
        self._last_ik_target_xyz = None if ik_target_xyz is None else np.array(ik_target_xyz, dtype=float).reshape(3)

    def update_ik_target_dir(self, ik_target_dir: Optional[np.ndarray]) -> None:
        self._last_ik_target_dir = None if ik_target_dir is None else np.array(ik_target_dir, dtype=float).reshape(3)

    def update_debug_markers(self, debug_markers: list[dict[str, Any]]) -> None:
        self._last_debug_markers = [dict(marker) for marker in list(debug_markers) if isinstance(marker, dict)]

    def update_sag_model(self, sag_model: Optional[dict[str, Any]]) -> None:
        if sag_model is None:
            return
        self._last_sag_model = dict(sag_model)

    def estimate_q(self) -> Optional[proto.SimQ]:
        return self._last_q

    def ik_target_xyz(self) -> Optional[np.ndarray]:
        return None if self._last_ik_target_xyz is None else self._last_ik_target_xyz.copy()

    def ik_target_dir(self) -> Optional[np.ndarray]:
        return None if self._last_ik_target_dir is None else self._last_ik_target_dir.copy()

    def sag_model(self) -> dict[str, Any]:
        return dict(self._last_sag_model)

    def claw_closed(self) -> bool:
        return bool(self._last_claw_closed)

    def go2_vel(self) -> tuple[float, float, float]:
        return (
            float(self._last_go2_vel[0]),
            float(self._last_go2_vel[1]),
            float(self._last_go2_vel[2]),
        )

    def go2_base_pos(self) -> Optional[tuple[float, float, float]]:
        if self._last_go2_base_pos is None:
            return None
        return (
            float(self._last_go2_base_pos[0]),
            float(self._last_go2_base_pos[1]),
            float(self._last_go2_base_pos[2]),
        )

    def go2_base_rpy(self) -> Optional[tuple[float, float, float]]:
        if self._last_go2_base_rpy is None:
            return None
        return (
            float(self._last_go2_base_rpy[0]),
            float(self._last_go2_base_rpy[1]),
            float(self._last_go2_base_rpy[2]),
        )

    def go2_leg_q(self) -> Optional[tuple[float, ...]]:
        if self._last_go2_leg_q is None:
            return None
        return tuple(float(v) for v in self._last_go2_leg_q)

    def sim_reset_seq(self) -> int:
        return int(self._last_sim_reset_seq)

    def debug_markers(self) -> list[dict[str, Any]]:
        return [dict(marker) for marker in self._last_debug_markers]


class HostStateSubscriber:
    """SIM-side subscriber that consumes host state broadcasts."""

    def __init__(self, endpoint: str) -> None:
        if zmq is None:
            raise RuntimeError("pyzmq is required for sim host subscriber")
        self.endpoint = str(endpoint)
        self.ctx = zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.SUB)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.setsockopt(zmq.SUBSCRIBE, b"")
        self.sock.connect(self.endpoint)
        self.poller = zmq.Poller()
        self.poller.register(self.sock, zmq.POLLIN)
        self.last_q: Optional[proto.SimQ] = None
        self.last_u: Optional[proto.ControlU] = None
        self.last_torque_enabled: bool = False
        self.last_state_ts: float = 0.0
        self.last_ik_target_xyz: Optional[np.ndarray] = None
        self.last_ik_target_dir: Optional[np.ndarray] = None
        self.last_sag_model: dict[str, Any] = {}
        self.last_claw_closed: bool = False
        self.last_go2_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.last_go2_base_pos: Optional[tuple[float, float, float]] = None
        self.last_go2_base_rpy: Optional[tuple[float, float, float]] = None
        self.last_go2_leg_q: Optional[tuple[float, ...]] = None
        self.last_sim_reset_seq: int = 0
        self.last_debug_markers: list[dict[str, Any]] = []

    def close(self) -> None:
        try:
            self.poller.unregister(self.sock)
        except Exception:
            pass
        try:
            self.sock.close(0)
        except Exception:
            pass

    def poll(self) -> None:
        try:
            events = dict(self.poller.poll(timeout=0))
        except zmq.ZMQError:
            return
        if self.sock not in events:
            return
        while True:
            try:
                data = self.sock.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            except zmq.ZMQError:
                return
            try:
                msg = proto.loads_msg(data)
            except Exception:
                continue
            if str(msg.get("t", "")).lower() != "state":
                continue
            self.last_state_ts = float(msg.get("ts", time.time()))
            if "q" in msg:
                try:
                    self.last_q = proto.unpack_q(msg["q"])
                except Exception:
                    pass
            if "u" in msg:
                try:
                    self.last_u = proto.unpack_u(msg["u"])
                except Exception:
                    pass
            if "torque_enabled" in msg:
                self.last_torque_enabled = bool(msg.get("torque_enabled", False))
            target_raw = msg.get("ik_target", None)
            if isinstance(target_raw, (list, tuple)) and len(target_raw) == 3:
                self.last_ik_target_xyz = np.array([float(target_raw[0]), float(target_raw[1]), float(target_raw[2])], dtype=float)
            target_dir_raw = msg.get("ik_target_dir", None)
            if isinstance(target_dir_raw, (list, tuple)) and len(target_dir_raw) == 3:
                self.last_ik_target_dir = np.array(
                    [float(target_dir_raw[0]), float(target_dir_raw[1]), float(target_dir_raw[2])],
                    dtype=float,
                )
            sag_raw = msg.get("sag_model", None)
            if isinstance(sag_raw, dict):
                self.last_sag_model = dict(sag_raw)
            if "claw_closed" in msg:
                self.last_claw_closed = bool(msg.get("claw_closed", False))
            if "go2_vel" in msg:
                try:
                    self.last_go2_vel = proto.unpack_go2_vel(msg.get("go2_vel"))
                except Exception:
                    pass
            pos_raw = msg.get("go2_base_pos", None)
            if isinstance(pos_raw, (list, tuple)) and len(pos_raw) == 3:
                self.last_go2_base_pos = (float(pos_raw[0]), float(pos_raw[1]), float(pos_raw[2]))
            rpy_raw = msg.get("go2_base_rpy", None)
            if isinstance(rpy_raw, (list, tuple)) and len(rpy_raw) == 3:
                self.last_go2_base_rpy = (float(rpy_raw[0]), float(rpy_raw[1]), float(rpy_raw[2]))
            leg_raw = msg.get("go2_leg_q", None)
            if isinstance(leg_raw, (list, tuple)) and len(leg_raw) == 12:
                self.last_go2_leg_q = tuple(float(v) for v in leg_raw)
            if "sim_reset_seq" in msg:
                try:
                    self.last_sim_reset_seq = int(msg.get("sim_reset_seq", 0))
                except (TypeError, ValueError):
                    pass
            debug_markers_raw = msg.get("debug_markers", None)
            if isinstance(debug_markers_raw, list):
                next_markers: list[dict[str, Any]] = []
                for raw in debug_markers_raw:
                    if isinstance(raw, dict):
                        next_markers.append(dict(raw))
                self.last_debug_markers = next_markers


class HostFeedbackPublisher:
    """SIM-side publisher that pushes actual tip feedback to host."""

    def __init__(self, endpoint: str) -> None:
        if zmq is None:
            raise RuntimeError("pyzmq is required for sim feedback publisher")
        self.endpoint = str(endpoint)
        self.ctx = zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.PUSH)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.connect(self.endpoint)

    def send_actual_tip(
        self,
        actual_tip_xyz: Optional[np.ndarray],
        actual_tip_dir: Optional[np.ndarray] = None,
        *,
        camera_origin: Optional[np.ndarray] = None,
        camera_look: Optional[np.ndarray] = None,
        camera_right: Optional[np.ndarray] = None,
    ) -> None:
        if actual_tip_xyz is None and camera_origin is None:
            return
        msg: dict[str, Any] = {
            "t": "sim_state",
            "ts": time.time(),
        }
        if actual_tip_xyz is not None:
            msg["actual_tip"] = [
                float(actual_tip_xyz[0]),
                float(actual_tip_xyz[1]),
                float(actual_tip_xyz[2]),
            ]
        if actual_tip_dir is not None:
            d = np.asarray(actual_tip_dir, dtype=float).reshape(3)
            norm = float(np.linalg.norm(d))
            if norm > 1e-9:
                d = d / norm
                msg["actual_tip_dir"] = [float(d[0]), float(d[1]), float(d[2])]
        if camera_origin is not None:
            o = np.asarray(camera_origin, dtype=float).reshape(3)
            msg["camera_world_origin"] = [float(o[0]), float(o[1]), float(o[2])]
        if camera_look is not None:
            look = np.asarray(camera_look, dtype=float).reshape(3)
            msg["camera_world_look"] = [float(look[0]), float(look[1]), float(look[2])]
        if camera_right is not None:
            right = np.asarray(camera_right, dtype=float).reshape(3)
            msg["camera_world_right"] = [float(right[0]), float(right[1]), float(right[2])]
        try:
            self.sock.send(proto.dumps_msg(msg), flags=zmq.NOBLOCK)
        except Exception:
            pass

    def send_go2_base(self, go2_entity) -> None:
        try:
            from engine.go2_mpc.genesis_pin_bridge import _quat_wxyz_to_xyzw, _to_numpy_1d
            from scipy.spatial.transform import Rotation as Rot

            base = go2_entity.get_link("base")
            pos = _to_numpy_1d(base.get_pos())[:3]
            quat_xyzw = _quat_wxyz_to_xyzw(_to_numpy_1d(base.get_quat())[:4])
            rpy = Rot.from_quat(quat_xyzw).as_euler("xyz", degrees=False)
            vel_world = _to_numpy_1d(base.get_vel())[:3]
            ang_world = _to_numpy_1d(base.get_ang())[:3]
            rot = Rot.from_quat(quat_xyzw)
            vel_body = rot.inv().apply(vel_world)
            ang_body = rot.inv().apply(ang_world)
            now = time.time()
            msg = {
                "t": "sim_state",
                "ts": now,
                "go2_base_pos": [float(pos[0]), float(pos[1]), float(pos[2])],
                "go2_base_rpy": [float(rpy[0]), float(rpy[1]), float(rpy[2])],
                "go2_base_lin_vel_body": [float(vel_body[0]), float(vel_body[1]), float(vel_body[2])],
                "go2_base_ang_vel": [float(ang_body[0]), float(ang_body[1]), float(ang_body[2])],
                "go2_base_timestamp_s": float(now),
            }
            self.sock.send(proto.dumps_msg(msg), flags=zmq.NOBLOCK)
        except Exception:
            pass

    def close(self) -> None:
        try:
            self.sock.close(0)
        except Exception:
            pass


class HostStateSource(StateSource):
    """State source backed by host PUB/SUB updates."""

    def __init__(self, endpoint: str) -> None:
        self._sub = HostStateSubscriber(endpoint)
        self._cache = HardwareStateCache()

    def poll(self) -> None:
        self._sub.poll()
        self._cache.update_ik_target(self._sub.last_ik_target_xyz)
        self._cache.update_ik_target_dir(self._sub.last_ik_target_dir)
        self._cache.update_debug_markers(self._sub.last_debug_markers)
        self._cache.update_sag_model(self._sub.last_sag_model)
        self._cache.update_claw_closed(self._sub.last_claw_closed)
        self._cache.update_go2_vel(self._sub.last_go2_vel)
        self._cache.update_go2_base(self._sub.last_go2_base_pos, self._sub.last_go2_base_rpy)
        self._cache.update_go2_leg_q(self._sub.last_go2_leg_q)
        self._cache.update_sim_reset_seq(self._sub.last_sim_reset_seq)
        if self._sub.last_q is not None:
            self._cache.update(self._sub.last_q, self._sub.last_ik_target_xyz, self._sub.last_ik_target_dir, self._sub.last_sag_model)

    def estimate_q(self) -> Optional[proto.SimQ]:
        return self._cache.estimate_q()

    def ik_target_xyz(self) -> Optional[np.ndarray]:
        return self._cache.ik_target_xyz()

    def ik_target_dir(self) -> Optional[np.ndarray]:
        return self._cache.ik_target_dir()

    def sag_model(self) -> dict[str, Any]:
        return self._cache.sag_model()

    def claw_closed(self) -> bool:
        return self._cache.claw_closed()

    def go2_vel(self) -> tuple[float, float, float]:
        return self._cache.go2_vel()

    def go2_base_pos(self) -> Optional[tuple[float, float, float]]:
        return self._cache.go2_base_pos()

    def go2_base_rpy(self) -> Optional[tuple[float, float, float]]:
        return self._cache.go2_base_rpy()

    def go2_leg_q(self) -> Optional[tuple[float, ...]]:
        return self._cache.go2_leg_q()

    def host_state_age_s(self) -> Optional[float]:
        ts = float(self._sub.last_state_ts)
        if ts <= 0.0:
            return None
        return max(0.0, time.time() - ts)

    def sim_reset_seq(self) -> int:
        return self._cache.sim_reset_seq()

    def debug_markers(self) -> list[dict[str, Any]]:
        return self._cache.debug_markers()

    def close(self) -> None:
        self._sub.close()


class RuntimePrep:
    """Scene wiring and runtime objects."""

    def __init__(self, app: "GenesisApp"):
        self.app = app
    def _detect_n_nodes(self, entity) -> int:
        a = self.app
        if a.layout.bend_joint_names:
            return len(a.layout.bend_joint_names)

        i = 0
        while True:
            try:
                entity.get_joint(f"bend_{i}")
                i += 1
            except Exception:
                break
        if i <= 0:
            raise RuntimeError("No bend_* joints found in loaded URDF")
        return i

    def _apply_no_clip_pairs(self, entity) -> None:
        a = self.app
        pairs = list(a.layout.no_clip_pairs)
        if not pairs:
            return

        methods = []
        for owner in (entity, a.sim_scene.scene):
            if owner is None:
                continue
            for name in (
                "disable_collision_between_links",
                "disable_collision_pair",
                "set_collision_between_links",
                "set_collision_pair",
                "set_pair_collision",
            ):
                fn = getattr(owner, name, None)
                if callable(fn):
                    methods.append((name, fn))

        applied = 0
        for la, lb in pairs:
            la = str(la)
            lb = str(lb)
            link_a = None
            link_b = None
            try:
                link_a = entity.get_link(la)
                link_b = entity.get_link(lb)
            except Exception:
                pass
            done = False
            for mname, fn in methods:
                patterns = []
                if mname.startswith("disable_"):
                    patterns = [(la, lb), (link_a, link_b)]
                else:
                    patterns = [(la, lb, False), (link_a, link_b, False)]
                for args in patterns:
                    if any(x is None for x in args):
                        continue
                    try:
                        fn(*args)
                        applied += 1
                        done = True
                        break
                    except Exception:
                        continue
                if done:
                    break

        if applied > 0:
            print(f"[Collision] no-clip pairs applied: {applied}/{len(pairs)}")
        else:
            print("[Collision] NoClipPairs present, but runtime collision-pair API was not found.")

    def init_genesis(self, urdf_path: str) -> None:
        a = self.app
        use_go2 = bool(getattr(a.cfg, "use_go2", False))
        backend = gs.gpu if a.cfg.use_gpu else gs.cpu
        backend_name = "gpu" if a.cfg.use_gpu else "cpu"
        print(f"[runtime] genesis backend requested: {backend_name}")
        _ensure_genesis_cache_dir()
        try:
            gs.init(backend=backend, logging_level="warning")
        except TypeError:
            gs.init(backend=backend)

        gravity = tuple(float(x) for x in a.params.gravity)
        if use_go2 and gravity == (0.0, 0.0, 0.0):
            gravity = (0.0, 0.0, -9.81)
            print("[runtime] use_go2=true: enabling gravity (0, 0, -9.81)")

        try:
            sim_opts = gs.options.SimOptions(dt=a.params.dt, gravity=gravity, substeps=int(a.params.substeps))
        except TypeError:
            try:
                sim_opts = gs.options.SimOptions(dt=a.params.dt, gravity=gravity)
            except TypeError:
                sim_opts = gs.options.SimOptions(dt=a.params.dt)

        spawn_pos = tuple(float(x) for x in a.spawn.spawn_xyz)
        spawn_euler = tuple(float(x) for x in a.spawn.spawn_euler_deg)
        if use_go2:
            go2_xy = (spawn_pos[0], spawn_pos[1])
            go2_z = float(a.spawn.go2_spawn_height)
            go2_euler = tuple(float(x) for x in a.spawn.go2_spawn_euler_deg)
            go2_pos = (go2_xy[0], go2_xy[1], go2_z)
            mount_off = tuple(float(x) for x in a.spawn.go2_mount_offset_m)
            arm_pos = _world_offset(go2_pos, go2_euler, mount_off)
            arm_euler = spawn_euler
            cam_lookat = (go2_xy[0] + 0.25, go2_xy[1], go2_z + 0.30)
            cam_pos = (go2_xy[0] + 1.10, go2_xy[1] - 1.00, go2_z + 1.10)
        else:
            go2_pos = None
            go2_euler = (0.0, 0.0, 0.0)
            arm_pos = spawn_pos
            arm_euler = spawn_euler
            cam_lookat = (spawn_pos[0] + 0.25, spawn_pos[1], spawn_pos[2])
            cam_pos = (spawn_pos[0] + 1.10, spawn_pos[1] - 1.00, spawn_pos[2] + 1.10)

        a.sim_scene.scene = gs.Scene(
            sim_options=sim_opts,
            viewer_options=gs.options.ViewerOptions(
                camera_pos=cam_pos,
                camera_lookat=cam_lookat,
                camera_fov=35,
                max_FPS=60,
            ),
            show_viewer=bool(a.cfg.enable_viewer),
        )

        if a.cfg.floor:
            floor_ent = a.sim_scene.scene.add_entity(gs.morphs.Plane())
        else:
            floor_ent = None

        go2_entity = None
        if use_go2:
            go2_urdf = self._resolve_genesis_go2_urdf()
            go2_urdf = _prepare_go2_urdf_with_config_colors(
                go2_urdf,
                build_dir=a.cfg.build_dir,
                colors=a.urdf_export_cfg.part_color_rgba_by_name,
            )
            if not go2_urdf:
                raise RuntimeError("use_go2=true but genesis go2.urdf was not found")
            go2_entity = a.sim_scene.scene.add_entity(
                _make_urdf_morph(
                    str(go2_urdf),
                    go2_pos,
                    go2_euler,
                    fixed=False,
                    requires_jac_and_IK=True,
                )
            )
            print(f"[runtime] GO2 spawned at {go2_pos} fixed=false from {go2_urdf}")

        arm_fixed = not use_go2
        ent = a.sim_scene.scene.add_entity(_make_urdf_morph(urdf_path, arm_pos, arm_euler, fixed=arm_fixed))
        if use_go2:
            print(f"[runtime] arm mounted at {arm_pos} fixed=false (weld to GO2 base)")

        if bool(a.spawn.sim_target_enable):
            self._spawn_perception_target()

        eye_camera = None
        if bool(a.cfg.sim_camera_enable) and str(a.cfg.hand_eye_config).strip():
            from engine.sim_camera import Node9EyeInHandCamera

            eye_camera = Node9EyeInHandCamera.create(
                a.sim_scene.scene,
                res=(int(a.cfg.sim_camera_width), int(a.cfg.sim_camera_height)),
                fov_deg=float(a.cfg.sim_camera_fov_deg),
            )

        t_build = time.time()
        a.sim_scene.scene.build()
        if floor_ent is not None:
            try:
                floor_ent.set_friction(0.8)
            except Exception:
                pass
        print("[runtime] scene built in %.2fs" % (time.time() - t_build))

        if use_go2 and go2_entity is not None:
            _set_go2_initial_leg_pose(go2_entity, pose_name="ready")
            go2_mirror = bool(a.go2_locomotion_config.mirror_from_host)
            if not go2_mirror:
                self._weld_arm_to_go2(arm_ent=ent, go2_ent=go2_entity)
            else:
                print("[runtime] GO2 mirror: kinematic base+legs; arm follows via sync")
            a.sim_scene.record_arm_go2_mount(arm_ent=ent, go2_ent=go2_entity)
            from engine.go2_mpc.walking_metrics import WalkingMetricsLogger, WalkingMetricsMeta

            metrics = WalkingMetricsLogger.from_env()
            a.sim_scene.walking_metrics = metrics
            a.sim_scene.go2_entity = go2_entity
            a.sim_scene.go2 = Go2Locomotion(
                go2_entity,
                dt=a.params.dt,
                config=a.go2_locomotion_config,
                arm_entity=ent,
                metrics=metrics,
            )
            if a.go2_locomotion_config.mirror_from_host:
                print("[runtime] GO2 mirror_from_host=true: sim follows host go2_base_* (MPC off)")

        n_nodes = self._detect_n_nodes(ent)
        n_seg = int(a.spawn.n_seg) if a.spawn.n_seg is not None else max(1, n_nodes // 2)

        a.sim_scene.mover = SimMover(
            ent,
            a.params,
            a.limit,
            n_nodes=n_nodes,
            n_seg=n_seg,
            linear_joint_name=a.layout.linear_joint_name,
            roll_joint_name=a.layout.roll_joint_name,
            bend_joint_names=a.layout.bend_joint_names,
        )
        a.sim_scene.n_nodes = n_nodes
        a.sim_scene.n_seg = n_seg

        if eye_camera is not None:
            eye_camera.bind(ent, hand_eye_path=str(a.cfg.hand_eye_config))
            a.sim_scene.eye_camera = eye_camera
            a.sim_scene.hand_eye_config_path = str(a.cfg.hand_eye_config)
            from engine.sim_camera import SimCameraPublisher

            a.sim_scene.camera_publisher = SimCameraPublisher(
                str(a.cfg.sim_camera_port),
                use_jpeg=bool(a.cfg.sim_camera_jpeg),
                jpeg_quality=int(a.cfg.sim_camera_jpeg_quality),
            )

    def _spawn_perception_target(self) -> None:
        a = self.app
        scene = a.sim_scene.scene
        if scene is None:
            return
        pos = tuple(float(x) for x in a.spawn.sim_target_xyz)
        radius = float(a.spawn.sim_target_radius)
        try:
            scene.add_entity(
                gs.morphs.Sphere(
                    radius=max(0.01, radius),
                    pos=pos,
                    fixed=True,
                ),
                surface=gs.surfaces.Rough(color=(0.85, 0.15, 0.15, 1.0)),
            )
            print(f"[runtime] sim perception target sphere at {pos} r={radius:.3f}")
        except Exception as exc:
            print(f"[runtime] sim target spawn failed: {exc}")

    def _weld_arm_to_go2(self, *, arm_ent, go2_ent) -> None:
        a = self.app
        scene = a.sim_scene.scene
        if scene is None:
            return
        try:
            plate = arm_ent.get_link("plate")
            base = go2_ent.get_link("base")
            solver = scene.rigid_solver
            solver.add_weld_constraint(int(plate.idx), int(base.idx))
            print(f"[runtime] GO2 weld: plate(idx={plate.idx}) <-> base(idx={base.idx})")
        except Exception as exc:
            print(f"[runtime] GO2 weld failed: {exc}")

    @staticmethod
    def _resolve_genesis_go2_urdf() -> str:
        local_candidate = os.path.join(
            os.path.dirname(__file__),
            "assets",
            "go2",
            "go2.urdf",
        )
        if os.path.isfile(local_candidate):
            return local_candidate
        return ""



class SimRuntime:
    """Main loop: protocol sync, IK, control, debug markers."""

    def __init__(self, app: "GenesisApp"):
        self.app = app
        self._applied_sim_reset_seq: int = 0
        self._t_mirror_status_log: float = 0.0

    def _maybe_log_mirror_status(self, now: float) -> None:
        a = self.app
        if a.sim_scene.go2 is None or not a.sim_scene.go2.mirror_mode:
            return
        if a.state_source is None:
            return
        if (now - self._t_mirror_status_log) < 2.0:
            return
        self._t_mirror_status_log = now
        age = a.state_source.host_state_age_s() if hasattr(a.state_source, "host_state_age_s") else None
        base_pos = a.state_source.go2_base_pos()
        base_rpy = a.state_source.go2_base_rpy()
        leg_q = a.state_source.go2_leg_q()
        go2_vel = a.state_source.go2_vel()
        endpoint = str(a.cfg.host_sim_port).strip()
        if age is None:
            link_txt = "no host state yet"
        else:
            link_txt = "host state %.2fs ago" % float(age)
        pos_txt = (
            "(%.3f, %.3f, %.3f)" % (base_pos[0], base_pos[1], base_pos[2])
            if base_pos is not None
            else "none"
        )
        rpy_txt = (
            "(%.3f, %.3f, %.3f)" % (base_rpy[0], base_rpy[1], base_rpy[2])
            if base_rpy is not None
            else "none"
        )
        legs_txt = (
            "12 joints FL=(%.2f,%.2f,%.2f)" % (leg_q[0], leg_q[1], leg_q[2])
            if leg_q is not None and len(leg_q) == 12
            else ("none" if leg_q is None else f"{len(leg_q)} joints")
        )
        print(
            "[sim_mirror] %s | endpoint=%s | go2_base_pos=%s rpy=%s | legs=%s | host_go2_vel=(%.2f,%.2f,%.2f)"
            % (
                link_txt,
                endpoint,
                pos_txt,
                rpy_txt,
                legs_txt,
                float(go2_vel[0]),
                float(go2_vel[1]),
                float(go2_vel[2]),
            )
        )

    def _poll_host_and_update_model(self) -> None:
        a = self.app
        if a.state_source is not None:
            a.state_source.poll()
            reset_seq = int(a.state_source.sim_reset_seq())
            if reset_seq > int(self._applied_sim_reset_seq):
                self._applied_sim_reset_seq = reset_seq
                a.sim_scene.reset_environment()
                a.sim_scene.maybe_publish_camera(
                    arm_q=(0.0, 0.0, 0.0, 0.0),
                    max_hz=float(a.cfg.sim_camera_max_hz),
                    force=True,
                    rgb_enabled=bool(a.cfg.sim_camera_rgb),
                    depth_enabled=bool(a.cfg.sim_camera_depth),
                )

    def _cleanup(self) -> None:
        a = self.app
        if a.sim_scene.camera_publisher is not None:
            try:
                a.sim_scene.camera_publisher.close()
            except Exception:
                pass
        if a.state_source is not None:
            a.state_source.close()
        if a.feedback_pub is not None:
            a.feedback_pub.close()

    def run(self) -> None:
        a = self.app
        assert a.sim_scene.scene is not None and a.sim_scene.mover is not None
        perf = PerfLogger(
            enabled=bool(getattr(a.cfg, "perf_log_enable", False)),
            interval_s=float(getattr(a.cfg, "perf_log_interval_s", 2.0)),
            log_path=str(getattr(a.cfg, "perf_log_path", "")),
        )

        try:
            while True:
                perf.reset_loop()
                t_sec = time.perf_counter()
                self._poll_host_and_update_model()
                self._maybe_log_mirror_status(time.time())
                ik_target = a.state_source.ik_target_xyz() if a.state_source is not None else None
                ik_target_dir = a.state_source.ik_target_dir() if a.state_source is not None else None
                sag_model = a.state_source.sag_model() if a.state_source is not None else {}
                a.sim_scene.mover.set_sag_model(sag_model)
                claw_closed = a.state_source.claw_closed() if a.state_source is not None else False
                a.sim_scene.mover.set_claw_closed(claw_closed)
                perf.section("poll", t_sec)
                t_sec = time.perf_counter()
                if a.sim_scene.go2 is not None:
                    go2_mirror = bool(a.sim_scene.go2.mirror_mode)
                    if go2_mirror and a.state_source is not None:
                        base_pos = a.state_source.go2_base_pos()
                        base_rpy = a.state_source.go2_base_rpy()
                        if base_pos is not None and base_rpy is not None:
                            leg_q = a.state_source.go2_leg_q()
                            a.sim_scene.go2.apply_mirror_pose(base_pos, base_rpy, leg_q=leg_q)
                    else:
                        go2_vel = a.state_source.go2_vel() if a.state_source is not None else (0.0, 0.0, 0.0)
                        a.sim_scene.go2.set_planar_velocity(
                            float(go2_vel[0]),
                            float(go2_vel[1]),
                            float(go2_vel[2]),
                        )
                    q_errmodel = a._errmodel_q() if a._has_state_source() else None
                    if q_errmodel is not None:
                        a.sim_scene.go2.set_arm_q_for_metrics(
                            (
                                float(q_errmodel.linear_m),
                                float(q_errmodel.roll_rad),
                                float(q_errmodel.theta1_rad),
                                float(q_errmodel.theta2_rad),
                            )
                        )
                    a.sim_scene.sync_arm_to_go2_base()
                perf.section("go2", t_sec)
                if ik_target is not None and a.spawn.draw_debug_markers:
                    a.sim_scene.draw_marker(a.markers, "_ik_target_marker", ik_target, (1.0, 0.0, 0.0, 0.9))
                    if ik_target_dir is not None:
                        desired_dir = np.asarray(ik_target_dir, dtype=float).reshape(3)
                        dnorm = float(np.linalg.norm(desired_dir))
                        if dnorm > 1e-9:
                            a.sim_scene.draw_marker_direction(
                                a.markers,
                                "_ik_target_marker_dir",
                                ik_target,
                                desired_dir / dnorm,
                                (1.0, 0.4, 0.4, 0.9),
                            )
                q_errmodel = a._errmodel_q() if a._has_state_source() else None
                if q_errmodel is not None:
                    a.sim_scene.apply_sim_q(q_errmodel)

                t_sec = time.perf_counter()
                sim_tip = a.sim_scene.actual_tip_world(a.layout)
                sim_tip_dir = a.sim_scene.actual_tip_direction_world(a.layout)
                cam_origin = cam_look = cam_right = None
                if a.sim_scene.eye_camera is not None and str(a.cfg.hand_eye_config).strip():
                    cam_axes = a.sim_scene.camera_axes_world(hand_eye_path=str(a.cfg.hand_eye_config))
                    if cam_axes is not None:
                        cam_origin, cam_look, cam_right = cam_axes
                if a.feedback_pub is not None:
                    a.feedback_pub.send_actual_tip(
                        sim_tip,
                        sim_tip_dir,
                        camera_origin=cam_origin,
                        camera_look=cam_look,
                        camera_right=cam_right,
                    )
                    if a.sim_scene.go2_entity is not None and (
                        a.sim_scene.go2 is None or not a.sim_scene.go2.mirror_mode
                    ):
                        a.feedback_pub.send_go2_base(a.sim_scene.go2_entity)
                perf.section("feedback", t_sec)
                t_sec = time.perf_counter()
                if a.spawn.draw_debug_markers and sim_tip is not None:
                    a.sim_scene.draw_marker(a.markers, "_sim_tip_marker", sim_tip, (1.0, 1.0, 1.0, 0.95))
                    if sim_tip_dir is not None:
                        a.sim_scene.draw_marker_direction(a.markers, "_sim_tip_marker_dir", sim_tip, sim_tip_dir, (1.0, 1.0, 1.0, 0.98))
                active_dynamic_keys: set[str] = set()
                _HOST_CAMERA_MARKER_NAMES = frozenset({"camera_optical", "camera_look", "camera_right"})
                if a.spawn.draw_debug_markers and cam_origin is not None and cam_look is not None and cam_right is not None:
                    pos_arr = np.asarray(cam_origin, dtype=float).reshape(3)
                    active_dynamic_keys.add("sim_camera:sphere")
                    a.markers.draw_dynamic_sphere(
                        a.sim_scene.scene,
                        "sim_camera:sphere",
                        pos_arr,
                        [0.1, 0.7, 1.0, 0.95],
                        0.010,
                    )
                    active_dynamic_keys.add("sim_camera:look")
                    a.markers.draw_dynamic_arrow(
                        a.sim_scene.scene,
                        "sim_camera:look",
                        pos_arr,
                        np.asarray(cam_look, dtype=float).reshape(3),
                        [0.1, 0.7, 1.0, 0.95],
                        0.004,
                        0.09,
                    )
                    active_dynamic_keys.add("sim_camera:right")
                    a.markers.draw_dynamic_arrow(
                        a.sim_scene.scene,
                        "sim_camera:right",
                        pos_arr,
                        np.asarray(cam_right, dtype=float).reshape(3),
                        [1.0, 0.8, 0.2, 0.95],
                        0.004,
                        0.09,
                    )
                if a.spawn.draw_debug_markers and a.state_source is not None:
                    for marker in a.state_source.debug_markers():
                        if str(marker.get("frame", "world")) != "world":
                            continue
                        pos = marker.get("pos", None)
                        if not isinstance(pos, (list, tuple)) or len(pos) != 3:
                            continue
                        name = str(marker.get("name", "")).strip()
                        if not name or name in _HOST_CAMERA_MARKER_NAMES:
                            continue
                        color_raw = marker.get("color", [0.1, 1.0, 0.1, 0.95])
                        if isinstance(color_raw, (list, tuple)) and len(color_raw) >= 3:
                            rgba = [float(color_raw[0]), float(color_raw[1]), float(color_raw[2]), float(color_raw[3]) if len(color_raw) >= 4 else 0.95]
                        else:
                            rgba = [0.1, 1.0, 0.1, 0.95]
                        radius = float(marker.get("radius", 0.012))
                        pos_arr = np.asarray(pos, dtype=float).reshape(3)
                        sphere_key = f"{name}:sphere"
                        active_dynamic_keys.add(sphere_key)
                        a.markers.draw_dynamic_sphere(a.sim_scene.scene, sphere_key, pos_arr, rgba, radius)
                        direction = marker.get("dir", None)
                        if isinstance(direction, (list, tuple)) and len(direction) == 3:
                            arrow_key = f"{name}:dir"
                            active_dynamic_keys.add(arrow_key)
                            length = float(marker.get("length", 0.09))
                            a.markers.draw_dynamic_arrow(
                                a.sim_scene.scene,
                                arrow_key,
                                pos_arr,
                                np.asarray(direction, dtype=float).reshape(3),
                                rgba,
                                max(0.0025, radius * 0.35),
                                length,
                            )
                a.markers.clear_dynamic_missing(a.sim_scene.scene, active_dynamic_keys)
                perf.section("markers", t_sec)
                t_sec = time.perf_counter()
                a.sim_scene.step()
                perf.section("physics", t_sec)
                if a.sim_scene.go2 is not None and a.sim_scene.go2.mirror_mode:
                    if a.sim_scene.go2.reapply_last_mirror_pose():
                        a.sim_scene.sync_arm_to_go2_base()
                q_cam = a._errmodel_q() if a._has_state_source() else None
                arm_q = None
                if q_cam is not None:
                    arm_q = (
                        float(q_cam.linear_m),
                        float(q_cam.roll_rad),
                        float(q_cam.theta1_rad),
                        float(q_cam.theta2_rad),
                    )
                t_sec = time.perf_counter()
                a.sim_scene.maybe_publish_camera(
                    arm_q=arm_q,
                    max_hz=float(a.cfg.sim_camera_max_hz),
                    rgb_enabled=bool(a.cfg.sim_camera_rgb),
                    depth_enabled=bool(a.cfg.sim_camera_depth),
                )
                perf.section("camera", t_sec)
                perf.report_if_due()
        except KeyboardInterrupt:
            pass
        finally:
            perf.close()
            self._cleanup()


class GenesisApp:
    """Thin orchestrator over asset/runtime/control components."""

    def __init__(
        self,
        params: Optional[SimParam] = None,
        cfg: Optional[SimConfig] = None,
        limit: Optional[JointLimit] = None,
        model: Optional[SpawnConfig] = None,
        *,
        urdf_export_cfg: Optional[UrdfExportConfig] = None,
        ik_cfg: Optional[IkConfig] = None,
        go2_locomotion_config: Optional[Go2LocomotionConfig] = None,
        mapping_cfg: Optional[proto.SimMappingConfig] = None,
        endpoint: Optional[str] = None,
        enable_link: Optional[bool] = None,
        hardware_cfg: Optional[HardwareConfig] = None,
    ):
        self.params = params if params is not None else SimParam()
        self.cfg = cfg if cfg is not None else SimConfig()
        self.limit = limit if limit is not None else JointLimit(
            roll_min_deg=-90.0,
            roll_max_deg=90.0,
            bend_deg=36.0,
        )
        self.spawn = model if model is not None else SpawnConfig()
        self.urdf_export_cfg = urdf_export_cfg if urdf_export_cfg is not None else UrdfExportConfig()
        self.ik_cfg = ik_cfg if ik_cfg is not None else IkConfig()
        self.go2_locomotion_config = (
            go2_locomotion_config if go2_locomotion_config is not None else Go2LocomotionConfig()
        )
        self.hardware_cfg = hardware_cfg if hardware_cfg is not None else HardwareConfig()

        self._proto_cfg = mapping_cfg if mapping_cfg is not None else proto.SimMappingConfig()
        host_state_endpoint = str(self.cfg.host_sim_port).strip()

        self.layout = JointLayout()
        self.markers = MarkerSet()
        self.sim_scene = SimScene()
        self.state_source: Optional[StateSource] = HostStateSource(host_state_endpoint) if host_state_endpoint else None
        feedback_endpoint = str(self.cfg.host_feedback_port).strip()
        self.feedback_pub: Optional[HostFeedbackPublisher] = HostFeedbackPublisher(feedback_endpoint) if feedback_endpoint else None

    def _apply_ideal_rates_if_needed(self) -> None:
        if self.layout.control_mode != "commanded":
            return
        roll_rate = float(self.params.roll_rate)
        bend_rate = float(self.params.bend_rate)
        if np.isfinite(roll_rate) and np.isfinite(bend_rate):
            return
        try:
            est_roll, est_bend = estimate_ideal_sim_rates(self._proto_cfg)
        except Exception:
            return
        self.params = replace(
            self.params,
            roll_rate=roll_rate if np.isfinite(roll_rate) else float(est_roll),
            bend_rate=bend_rate if np.isfinite(bend_rate) else float(est_bend),
        )
        print(
            "[runtime] commanded rates matched to hardware profiles: "
            "roll=%.3f rad/s bend=%.3f rad/s"
            % (float(self.params.roll_rate), float(self.params.bend_rate))
        )

    def _has_state_source(self) -> bool:
        return bool(self.state_source is not None)

    def _errmodel_q(self) -> Optional[proto.SimQ]:
        if self.state_source is None:
            return None
        try:
            return self.state_source.estimate_q()
        except Exception:
            return None

    def run(self) -> None:
        urdf_path = AssetProcessor(self).prepare_assets()
        runtime = RuntimePrep(self)
        runtime.init_genesis(urdf_path)
        SimRuntime(self).run()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(__file__), "config.ini"),
        help="path to ini config file",
    )
    ap.add_argument("--perf-log", action="store_true", help="print periodic sim loop timing")
    ap.add_argument("--perf-interval", type=float, default=None, help="perf log interval in seconds")
    ap.add_argument("--perf-log-file", default=None, help="write perf CSV to this path or directory")
    ap.add_argument("--no-viewer", action="store_true", help="disable Genesis viewer for profiling/headless runs")
    ap.add_argument("--no-sim-camera", action="store_true", help="disable simulated RGB-D camera")
    ap.add_argument("--sim-camera-hz", type=float, default=None, help="override simulated camera publish rate")
    ap.add_argument("--sim-camera-rgb", action=argparse.BooleanOptionalAction, default=None, help="enable/disable simulated camera RGB rendering")
    ap.add_argument("--sim-camera-depth", action=argparse.BooleanOptionalAction, default=None, help="enable/disable simulated camera depth rendering")
    ap.add_argument("--sim-camera-jpeg-quality", type=int, default=None, help="override simulated camera JPEG quality")
    ap.add_argument(
        "--sim-camera-size",
        default=None,
        metavar="WIDTHxHEIGHT",
        help="override simulated camera resolution, e.g. 424x240",
    )
    ap.add_argument("--no-debug-markers", action="store_true", help="disable dynamic debug markers")
    args = ap.parse_args()

    bundle = load_app_config_from_ini(args.config)
    sim_cfg = bundle.sim_config
    spawn_cfg = bundle.spawn_config
    if args.perf_log or args.perf_interval is not None or args.perf_log_file is not None:
        sim_cfg = replace(
            sim_cfg,
            perf_log_enable=True if args.perf_log or args.perf_log_file is not None else bool(sim_cfg.perf_log_enable),
            perf_log_interval_s=(
                float(args.perf_interval)
                if args.perf_interval is not None
                else float(sim_cfg.perf_log_interval_s)
            ),
            perf_log_path=str(args.perf_log_file) if args.perf_log_file is not None else str(sim_cfg.perf_log_path),
        )
    if args.no_viewer:
        sim_cfg = replace(sim_cfg, enable_viewer=False)
    if args.no_sim_camera:
        sim_cfg = replace(sim_cfg, sim_camera_enable=False)
    if args.sim_camera_hz is not None:
        sim_cfg = replace(sim_cfg, sim_camera_max_hz=max(1.0, float(args.sim_camera_hz)))
    if args.sim_camera_rgb is not None:
        sim_cfg = replace(sim_cfg, sim_camera_rgb=bool(args.sim_camera_rgb))
    if args.sim_camera_depth is not None:
        sim_cfg = replace(sim_cfg, sim_camera_depth=bool(args.sim_camera_depth))
    if args.sim_camera_jpeg_quality is not None:
        sim_cfg = replace(sim_cfg, sim_camera_jpeg_quality=max(1, min(100, int(args.sim_camera_jpeg_quality))))
    if args.sim_camera_size:
        try:
            raw_w, raw_h = str(args.sim_camera_size).lower().split("x", 1)
            sim_cfg = replace(sim_cfg, sim_camera_width=max(1, int(raw_w)), sim_camera_height=max(1, int(raw_h)))
        except Exception as exc:
            raise SystemExit(f"invalid --sim-camera-size {args.sim_camera_size!r}; expected WIDTHxHEIGHT") from exc
    perception_mode = str(bundle.perception_config.mode).strip().lower()
    if bool(sim_cfg.sim_camera_auto_disable_unused) and perception_mode not in ("sim", "sim_rendered"):
        if bool(sim_cfg.sim_camera_enable):
            print(f"[sim_camera] disabled: perception.mode={perception_mode!r} does not consume sim camera")
        sim_cfg = replace(sim_cfg, sim_camera_enable=False)
    if args.no_debug_markers:
        spawn_cfg = replace(spawn_cfg, draw_debug_markers=False)
    app = GenesisApp(
        params=bundle.sim_param,
        cfg=sim_cfg,
        hardware_cfg=bundle.hardware_config,
        limit=bundle.joint_limit,
        model=spawn_cfg,
        urdf_export_cfg=bundle.urdf_export_config,
        ik_cfg=bundle.ik_config,
        go2_locomotion_config=bundle.go2_locomotion_config,
        mapping_cfg=bundle.mapping_config,
    )
    app.run()


if __name__ == "__main__":
    main()
