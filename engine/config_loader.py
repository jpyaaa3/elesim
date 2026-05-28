#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import configparser
import os
from dataclasses import dataclass, field
from typing import Optional, Tuple
import engine.protocol as proto
from engine.joint_defs import JointLimit


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_BUILD_DIR = os.path.join(PROJECT_ROOT, "craft")


@dataclass(frozen=True)
class SimParam:
    dt: float = 0.01
    substeps: int = 1
    gravity: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    roll_rate: float = float("inf")
    bend_rate: float = float("inf")

    zmq_hwm: int = 1


@dataclass(frozen=True)
class SimConfig:
    use_gpu: bool = True
    enable_viewer: bool = True
    floor: bool = True
    use_hardware: bool = True
    use_go2: bool = False

    build_dir: str = DEFAULT_BUILD_DIR
    assy_build_json: str = "manifest.json"
    urdf_name: str = "robot.urdf"
    rebuild_assembly: bool = True

    host_ctrl_port: str = "tcp://127.0.0.1:5555"
    host_sim_port: str = "tcp://127.0.0.1:5556"
    host_feedback_port: str = "tcp://127.0.0.1:5557"
    hand_eye_config: str = ""
    show_all_ports: bool = False


@dataclass(frozen=True)
class HardwareConfig:
    command_direction: Tuple[int, int, int, int] = (-1, 1, 1, 1)
    motor_direction: Tuple[int, int, int, int] = (1, 1, 1, 1)
    current_yellow_ma: int = 1800
    current_limit_ma: int = 2500


@dataclass(frozen=True)
class PickFsmConfig:
    enable: bool = False
    relocalize_window: int = 20
    relocalize_timeout_s: float = 1.0
    coarse_offset_m: Tuple[float, float, float] = (0.0, 0.0, 0.15)
    short_approach_m: float = 0.04
    align_step_m: float = 0.005
    align_step_rad: float = 0.0
    align_timeout_s: float = 1.5
    error_threshold_m: float = 0.01
    uncertainty_threshold: float = 0.001
    lift_height_m: float = 0.04
    lift_verify_timeout_s: float = 1.0
    max_attempts: int = 3
    depth_min_m: float = 0.05
    depth_max_m: float = 1.5
    outlier_zscore: float = 2.5
    stage_timeout_s: float = 3.0
    search_stable_frames: int = 3
    anchor_jump_limit_m: float = 0.05
    dropout_soft_limit: int = 3
    dropout_hard_limit: int = 8
    score_pass: float = 1.0
    score_decay_per_s: float = 0.2
    score_reward_observation: float = 0.15
    attempt_hard_fail_only: bool = True


@dataclass(frozen=True)
class UrdfExportConfig:
    robot_name: str = "Robot"
    default_effort: float = 200.0
    default_velocity: float = 3.0
    revolute_effort: Optional[float] = None
    revolute_velocity: Optional[float] = None
    prismatic_effort: Optional[float] = None
    prismatic_velocity: Optional[float] = None
    revolute_damping: float = 0.12
    revolute_friction: float = 0.06
    prismatic_damping: float = 60.0
    prismatic_friction: float = 20.0
    mesh_basename_only: bool = False
    part_color_rgba_by_name: dict[str, Tuple[float, float, float, float]] = field(default_factory=dict)


@dataclass(frozen=True)
class IkConfig:
    tol: float = 1e-4
    max_iters: int = 200
    stall_limit: int = 40

    damping_init: float = 1e-2
    damping_min: float = 1e-6
    damping_max: float = 1e+2
    damping_up: float = 10.0
    damping_down: float = 0.7

    step_scale: float = 1.0
    line_search_steps: int = 4
    line_search_shrink: float = 0.5
    fd_eps: float = 1e-4
    direction_weight: float = 0.1
    prefer_tip_plus_x: bool = True
    direction_tol_deg: float = 1.0
    orientation_tie_eps_m: float = 1e-3


@dataclass(frozen=True)
class SpawnConfig:
    pitch: float = 0.05
    n_seg: Optional[int] = None
    spawn_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    spawn_euler_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    draw_debug_markers: bool = True


@dataclass(frozen=True)
class AppConfigBundle:
    sim_param: SimParam
    sim_config: SimConfig
    hardware_config: HardwareConfig
    joint_limit: JointLimit
    spawn_config: SpawnConfig
    urdf_export_config: UrdfExportConfig
    ik_config: IkConfig
    mapping_config: proto.SimMappingConfig
    pick_fsm_config: PickFsmConfig


def _parse_vec3(text: str, default: Tuple[float, float, float]) -> Tuple[float, float, float]:
    raw = str(text).strip()
    if not raw:
        return default
    parts = [x.strip() for x in raw.split(",")]
    if len(parts) != 3:
        return default
    try:
        return (float(parts[0]), float(parts[1]), float(parts[2]))
    except Exception:
        return default


def _parse_optional_float(text: str, default: Optional[float]) -> Optional[float]:
    raw = str(text).strip()
    if raw == "":
        return default
    if raw.lower() in ("none", "null"):
        return None
    try:
        return float(raw)
    except Exception:
        return default


def _parse_color_rgba(text: str, default: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    raw = str(text).strip()
    if not raw:
        return default
    if raw.startswith("#"):
        h = raw[1:].strip()
        if len(h) == 6:
            try:
                r = int(h[0:2], 16) / 255.0
                g = int(h[2:4], 16) / 255.0
                b = int(h[4:6], 16) / 255.0
                return (r, g, b, 1.0)
            except Exception:
                return default
        if len(h) == 8:
            try:
                r = int(h[0:2], 16) / 255.0
                g = int(h[2:4], 16) / 255.0
                b = int(h[4:6], 16) / 255.0
                a = int(h[6:8], 16) / 255.0
                return (r, g, b, a)
            except Exception:
                return default
        return default
    parts = [x.strip() for x in raw.split(",")]
    if len(parts) != 4:
        return default
    try:
        return (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))
    except Exception:
        return default


def _parse_direction4(text: str, *, key: str) -> Tuple[int, int, int, int]:
    raw = str(text).strip()
    parts = [x.strip() for x in raw.split(",") if x.strip()]
    if len(parts) != 4:
        raise ValueError(f"{key} must contain exactly 4 comma-separated values in order: linear, roll, seg1, seg2")
    values = []
    for part in parts:
        try:
            value = int(part)
        except Exception as exc:
            raise ValueError(f"{key} must contain only integers 1 or -1") from exc
        if value not in (-1, 1):
            raise ValueError(f"{key} must contain only 1 or -1")
        values.append(value)
    return (values[0], values[1], values[2], values[3])


def _default_app_config_bundle() -> AppConfigBundle:
    return AppConfigBundle(
        sim_param=SimParam(),
        sim_config=SimConfig(),
        hardware_config=HardwareConfig(),
        joint_limit=JointLimit(roll_min_deg=-90.0, roll_max_deg=90.0, bend_deg=36.0),
        spawn_config=SpawnConfig(),
        urdf_export_config=UrdfExportConfig(),
        ik_config=IkConfig(),
        mapping_config=proto.SimMappingConfig(),
        pick_fsm_config=PickFsmConfig(),
    )


def _load_sim_param_config(cp: configparser.ConfigParser, defaults: AppConfigBundle) -> SimParam:
    sp0 = defaults.sim_param
    return SimParam(
        dt=cp.getfloat("SimParam", "dt", fallback=sp0.dt),
        substeps=cp.getint("SimParam", "substeps", fallback=sp0.substeps),
        gravity=_parse_vec3(cp.get("SimParam", "gravity", fallback=""), sp0.gravity),
        roll_rate=cp.getfloat("SimParam", "roll_rate", fallback=sp0.roll_rate),
        bend_rate=cp.getfloat("SimParam", "bend_rate", fallback=sp0.bend_rate),
        zmq_hwm=cp.getint("SimParam", "zmq_hwm", fallback=sp0.zmq_hwm),
    )


def _load_sim_config(cp: configparser.ConfigParser, defaults: AppConfigBundle, *, config_dir: str) -> SimConfig:
    sc0 = defaults.sim_config
    build_dir = os.path.abspath(os.path.join(config_dir, "craft"))
    hand_eye_raw = cp.get("runtime", "hand_eye_config", fallback=sc0.hand_eye_config).strip()
    hand_eye_config = (
        os.path.abspath(os.path.join(config_dir, hand_eye_raw))
        if hand_eye_raw and not os.path.isabs(hand_eye_raw)
        else hand_eye_raw
    )
    return SimConfig(
        use_gpu=cp.getboolean("runtime", "use_gpu", fallback=sc0.use_gpu),
        enable_viewer=cp.getboolean("runtime", "enable_viewer", fallback=sc0.enable_viewer),
        floor=cp.getboolean("runtime", "floor", fallback=sc0.floor),
        use_hardware=cp.getboolean("runtime", "use_hardware", fallback=sc0.use_hardware),
        use_go2=cp.getboolean("runtime", "use_go2", fallback=sc0.use_go2),
        build_dir=build_dir,
        assy_build_json=cp.get("runtime", "assy_build_json", fallback=sc0.assy_build_json),
        urdf_name=cp.get("runtime", "urdf_name", fallback=sc0.urdf_name),
        rebuild_assembly=cp.getboolean(
            "model",
            "rebuild_robot",
            fallback=cp.getboolean("model", "rebuild_robot_assets", fallback=sc0.rebuild_assembly),
        ),
        host_ctrl_port=cp.get("runtime", "host_ctrl_port", fallback=sc0.host_ctrl_port),
        host_sim_port=cp.get("runtime", "host_sim_port", fallback=sc0.host_sim_port),
        host_feedback_port=cp.get("runtime", "host_feedback_port", fallback=sc0.host_feedback_port),
        hand_eye_config=hand_eye_config,
        show_all_ports=cp.getboolean("runtime", "show_all_ports", fallback=sc0.show_all_ports),
    )


def _load_hardware_config(cp: configparser.ConfigParser) -> HardwareConfig:
    if cp.has_option("hardware", "dxl_dir_1") or cp.has_option("hardware", "dxl_dir_2") or cp.has_option("hardware", "dxl_dir_3") or cp.has_option("hardware", "dxl_dir_4"):
        raise ValueError("legacy hardware keys dxl_dir_1..4 are no longer supported; use command_direction and motor_direction")
    if not cp.has_option("hardware", "command_direction"):
        raise ValueError("missing required hardware.command_direction")
    if not cp.has_option("hardware", "motor_direction"):
        raise ValueError("missing required hardware.motor_direction")
    return HardwareConfig(
        command_direction=_parse_direction4(cp.get("hardware", "command_direction"), key="hardware.command_direction"),
        motor_direction=_parse_direction4(cp.get("hardware", "motor_direction"), key="hardware.motor_direction"),
        current_yellow_ma=cp.getint("hardware", "current_yellow_ma", fallback=HardwareConfig().current_yellow_ma),
        current_limit_ma=cp.getint("hardware", "current_limit_ma", fallback=HardwareConfig().current_limit_ma),
    )


def _load_joint_limit(cp: configparser.ConfigParser, defaults: AppConfigBundle) -> JointLimit:
    jl0 = defaults.joint_limit
    return JointLimit(
        roll_min_deg=cp.getfloat("model", "roll_min_deg", fallback=jl0.roll_min_deg),
        roll_max_deg=cp.getfloat("model", "roll_max_deg", fallback=jl0.roll_max_deg),
        bend_deg=cp.getfloat("model", "bend_deg", fallback=jl0.bend_deg),
    )


def _load_spawn_config(cp: configparser.ConfigParser, defaults: AppConfigBundle) -> SpawnConfig:
    am0 = defaults.spawn_config
    n_seg_raw = cp.get("app_model", "n_seg", fallback="")
    n_seg = am0.n_seg if n_seg_raw.strip() == "" else int(n_seg_raw)
    return SpawnConfig(
        pitch=cp.getfloat("app_model", "pitch", fallback=am0.pitch),
        n_seg=n_seg,
        spawn_xyz=_parse_vec3(cp.get("spawn", "spawn_position", fallback=""), am0.spawn_xyz),
        spawn_euler_deg=_parse_vec3(cp.get("spawn", "spawn_orientation_deg", fallback=""), am0.spawn_euler_deg),
        draw_debug_markers=cp.getboolean("spawn", "draw_debug_markers", fallback=am0.draw_debug_markers),
    )


def _load_urdf_export_config(cp: configparser.ConfigParser, defaults: AppConfigBundle) -> UrdfExportConfig:
    ue0 = defaults.urdf_export_config
    part_color_rgba_by_name: dict[str, Tuple[float, float, float, float]] = {}
    if cp.has_section("colors"):
        for raw_name, raw_value in cp.items("colors"):
            part_name = str(raw_name).strip()
            if not part_name:
                continue
            part_color_rgba_by_name[part_name] = _parse_color_rgba(str(raw_value), default=(1.0, 1.0, 1.0, 1.0))
    return UrdfExportConfig(
        robot_name=cp.get("urdf_export", "robot_name", fallback=ue0.robot_name),
        default_effort=cp.getfloat("urdf_export", "default_effort", fallback=ue0.default_effort),
        default_velocity=cp.getfloat("urdf_export", "default_velocity", fallback=ue0.default_velocity),
        revolute_effort=_parse_optional_float(cp.get("urdf_export", "revolute_effort", fallback=""), ue0.revolute_effort),
        revolute_velocity=_parse_optional_float(
            cp.get("urdf_export", "revolute_velocity", fallback=""), ue0.revolute_velocity
        ),
        prismatic_effort=_parse_optional_float(cp.get("urdf_export", "prismatic_effort", fallback=""), ue0.prismatic_effort),
        prismatic_velocity=_parse_optional_float(
            cp.get("urdf_export", "prismatic_velocity", fallback=""), ue0.prismatic_velocity
        ),
        revolute_damping=cp.getfloat("urdf_export", "revolute_damping", fallback=ue0.revolute_damping),
        revolute_friction=cp.getfloat("urdf_export", "revolute_friction", fallback=ue0.revolute_friction),
        prismatic_damping=cp.getfloat("urdf_export", "prismatic_damping", fallback=ue0.prismatic_damping),
        prismatic_friction=cp.getfloat("urdf_export", "prismatic_friction", fallback=ue0.prismatic_friction),
        mesh_basename_only=cp.getboolean("urdf_export", "mesh_basename_only", fallback=ue0.mesh_basename_only),
        part_color_rgba_by_name=part_color_rgba_by_name,
    )


def _load_ik_config(cp: configparser.ConfigParser, defaults: AppConfigBundle) -> IkConfig:
    ik0 = defaults.ik_config
    return IkConfig(
        tol=cp.getfloat("ik", "tol", fallback=ik0.tol),
        max_iters=cp.getint("ik", "max_iters", fallback=ik0.max_iters),
        stall_limit=cp.getint("ik", "stall_limit", fallback=ik0.stall_limit),
        damping_init=cp.getfloat("ik", "damping_init", fallback=ik0.damping_init),
        damping_min=cp.getfloat("ik", "damping_min", fallback=ik0.damping_min),
        damping_max=cp.getfloat("ik", "damping_max", fallback=ik0.damping_max),
        damping_up=cp.getfloat("ik", "damping_up", fallback=ik0.damping_up),
        damping_down=cp.getfloat("ik", "damping_down", fallback=ik0.damping_down),
        step_scale=cp.getfloat("ik", "step_scale", fallback=ik0.step_scale),
        line_search_steps=cp.getint("ik", "line_search_steps", fallback=ik0.line_search_steps),
        line_search_shrink=cp.getfloat("ik", "line_search_shrink", fallback=ik0.line_search_shrink),
        fd_eps=cp.getfloat("ik", "fd_eps", fallback=ik0.fd_eps),
        direction_weight=cp.getfloat("ik", "direction_weight", fallback=ik0.direction_weight),
        prefer_tip_plus_x=cp.getboolean("ik", "prefer_tip_plus_x", fallback=ik0.prefer_tip_plus_x),
        direction_tol_deg=cp.getfloat("ik", "direction_tol_deg", fallback=ik0.direction_tol_deg),
        orientation_tie_eps_m=cp.getfloat("ik", "orientation_tie_eps_m", fallback=ik0.orientation_tie_eps_m),
    )


def _load_pick_fsm_config(cp: configparser.ConfigParser, defaults: AppConfigBundle) -> PickFsmConfig:
    p0 = defaults.pick_fsm_config
    return PickFsmConfig(
        enable=cp.getboolean("pick_fsm", "enable", fallback=p0.enable),
        relocalize_window=max(3, cp.getint("pick_fsm", "relocalize_window", fallback=p0.relocalize_window)),
        relocalize_timeout_s=max(0.1, cp.getfloat("pick_fsm", "relocalize_timeout_s", fallback=p0.relocalize_timeout_s)),
        coarse_offset_m=_parse_vec3(cp.get("pick_fsm", "coarse_offset_m", fallback=""), p0.coarse_offset_m),
        short_approach_m=max(0.0, cp.getfloat("pick_fsm", "short_approach_m", fallback=p0.short_approach_m)),
        align_step_m=max(0.0005, cp.getfloat("pick_fsm", "align_step_m", fallback=p0.align_step_m)),
        align_step_rad=cp.getfloat("pick_fsm", "align_step_rad", fallback=p0.align_step_rad),
        align_timeout_s=max(0.1, cp.getfloat("pick_fsm", "align_timeout_s", fallback=p0.align_timeout_s)),
        error_threshold_m=max(0.0001, cp.getfloat("pick_fsm", "error_threshold_m", fallback=p0.error_threshold_m)),
        uncertainty_threshold=max(1e-7, cp.getfloat("pick_fsm", "uncertainty_threshold", fallback=p0.uncertainty_threshold)),
        lift_height_m=max(0.0, cp.getfloat("pick_fsm", "lift_height_m", fallback=p0.lift_height_m)),
        lift_verify_timeout_s=max(0.1, cp.getfloat("pick_fsm", "lift_verify_timeout_s", fallback=p0.lift_verify_timeout_s)),
        max_attempts=max(1, cp.getint("pick_fsm", "max_attempts", fallback=p0.max_attempts)),
        depth_min_m=max(0.0, cp.getfloat("pick_fsm", "depth_min_m", fallback=p0.depth_min_m)),
        depth_max_m=max(0.01, cp.getfloat("pick_fsm", "depth_max_m", fallback=p0.depth_max_m)),
        outlier_zscore=max(0.1, cp.getfloat("pick_fsm", "outlier_zscore", fallback=p0.outlier_zscore)),
        stage_timeout_s=max(0.5, cp.getfloat("pick_fsm", "stage_timeout_s", fallback=p0.stage_timeout_s)),
        search_stable_frames=max(1, cp.getint("pick_fsm", "search_stable_frames", fallback=p0.search_stable_frames)),
        anchor_jump_limit_m=max(1e-4, cp.getfloat("pick_fsm", "anchor_jump_limit_m", fallback=p0.anchor_jump_limit_m)),
        dropout_soft_limit=max(0, cp.getint("pick_fsm", "dropout_soft_limit", fallback=p0.dropout_soft_limit)),
        dropout_hard_limit=max(1, cp.getint("pick_fsm", "dropout_hard_limit", fallback=p0.dropout_hard_limit)),
        score_pass=cp.getfloat("pick_fsm", "score_pass", fallback=p0.score_pass),
        score_decay_per_s=max(0.0, cp.getfloat("pick_fsm", "score_decay_per_s", fallback=p0.score_decay_per_s)),
        score_reward_observation=max(0.0, cp.getfloat("pick_fsm", "score_reward_observation", fallback=p0.score_reward_observation)),
        attempt_hard_fail_only=cp.getboolean("pick_fsm", "attempt_hard_fail_only", fallback=p0.attempt_hard_fail_only),
    )


def _build_mapping_config(joint_limit: JointLimit, hardware_config: HardwareConfig) -> proto.SimMappingConfig:
    return proto.SimMappingConfig(
        linear_q_min_m=-0.230,
        linear_q_max_m=0.010,
        roll_q_min_rad=joint_limit.roll_min_rad(),
        roll_q_max_rad=joint_limit.roll_max_rad(),
        seg1_q_min_rad=-joint_limit.bend_lim_rad(),
        seg1_q_max_rad=+joint_limit.bend_lim_rad(),
        seg2_q_min_rad=-joint_limit.bend_lim_rad(),
        seg2_q_max_rad=+joint_limit.bend_lim_rad(),
        command_direction=hardware_config.command_direction,
    )


def load_app_config_from_ini(path: str) -> AppConfigBundle:
    defaults = _default_app_config_bundle()
    if not path:
        raise FileNotFoundError("config path is empty")
    if not os.path.isfile(path):
        raise FileNotFoundError(f"config file not found: {path}")

    config_dir = os.path.dirname(os.path.abspath(path))

    cp = configparser.ConfigParser()
    cp.optionxform = str
    cp.read(path, encoding="utf-8-sig")
    sim_param_cfg = _load_sim_param_config(cp, defaults)
    sim_config_cfg = _load_sim_config(cp, defaults, config_dir=config_dir)
    hardware_config_cfg = _load_hardware_config(cp)
    joint_limit_cfg = _load_joint_limit(cp, defaults)
    spawn_config_cfg = _load_spawn_config(cp, defaults)
    urdf_export_config_cfg = _load_urdf_export_config(cp, defaults)
    ik_config_cfg = _load_ik_config(cp, defaults)
    pick_fsm_cfg = _load_pick_fsm_config(cp, defaults)
    mapping_config_cfg = _build_mapping_config(joint_limit_cfg, hardware_config_cfg)

    return AppConfigBundle(
        sim_param=sim_param_cfg,
        sim_config=sim_config_cfg,
        hardware_config=hardware_config_cfg,
        joint_limit=joint_limit_cfg,
        spawn_config=spawn_config_cfg,
        urdf_export_config=urdf_export_config_cfg,
        ik_config=ik_config_cfg,
        mapping_config=mapping_config_cfg,
        pick_fsm_config=pick_fsm_cfg,
    )
