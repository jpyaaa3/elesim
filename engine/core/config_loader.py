#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import configparser
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple
import engine.core.protocol as proto
from engine.robot.go2.hardware.config import Go2HardwareConfig
from engine.robot.go2.locomotion.config import Go2LocomotionConfig
from engine.behaviors.gaze.stabilizer import GazeStabilizerConfig
from engine.robot.arm.joint_defs import JointLimit


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_BUILD_DIR = os.path.join(PROJECT_ROOT, "crafts")


@dataclass(frozen=True)
class SimParam:
    dt: float = 0.01
    substeps: int = 1
    realtime: bool = True
    realtime_factor: float = 1.0
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
    arm_urdf_name: str = "arm.urdf"
    rebuild_assembly: bool = True

    host_ctrl_port: str = "tcp://127.0.0.1:5555"
    host_sim_port: str = "tcp://127.0.0.1:5556"
    host_feedback_port: str = "tcp://127.0.0.1:5557"
    hand_eye_config: str = ""
    show_all_ports: bool = False
    traj_enable: bool = True
    traj_duration_s: float = 1.2
    traj_min_s: float = 0.25
    traj_max_s: float = 3.0
    traj_linear_scale_m: float = 0.05
    traj_angular_scale_rad: float = 0.35
    traj_lji_enable: bool = True
    traj_lji_duration_s: float = 0.14
    traj_lji_min_s: float = 0.07
    traj_lji_max_s: float = 0.35
    sim_camera_enable: bool = True
    sim_camera_port: str = "tcp://127.0.0.1:5568"
    sim_camera_jpeg: bool = True
    sim_camera_jpeg_quality: int = 85
    sim_camera_rgb: bool = True
    sim_camera_depth: bool = True
    sim_camera_auto_disable_unused: bool = False
    sim_camera_max_hz: float = 30.0
    sim_camera_width: int = 640
    sim_camera_height: int = 480
    sim_camera_fov_deg: float = 60.0
    sim_side_camera_enable: bool = True
    sim_side_camera_port: str = "tcp://127.0.0.1:5569"
    sim_side_camera_jpeg: bool = True
    sim_side_camera_jpeg_quality: int = 85
    sim_side_camera_max_hz: float = 20.0
    sim_side_camera_record_fps: float = 30.0
    sim_side_camera_width: int = 960
    sim_side_camera_height: int = 540
    sim_side_camera_fov_deg: float = 55.0
    sim_side_camera_pos: Tuple[float, float, float] = (0.45, -1.8, 0.55)
    sim_side_camera_lookat: Tuple[float, float, float] = (0.45, 0.0, 0.25)
    perf_log_enable: bool = False
    perf_log_interval_s: float = 2.0
    perf_log_path: str = ""


@dataclass(frozen=True)
class HardwareConfig:
    command_direction: Tuple[int, int, int, int] = (1, -1, 1, -1)
    motor_direction: Tuple[int, int, int, int] = (1, -1, 1, -1)
    baudrate: int = 57600
    linear_u_limit_deg: float = 250.0
    current_yellow_ma: int = 1800
    current_limit_ma: int = 2500
    host_hw_read_hz: float = 20.0
    host_hw_cmd_hz: float = 30.0
    current_read_hz: float = 20.0
    profile_vel_linear: int = 240
    profile_acc_linear: int = 10
    profile_vel_roll: int = 240
    profile_acc_roll: int = 10
    profile_vel_seg1: int = 60
    profile_acc_seg1: int = 6
    profile_vel_seg2: int = 60
    profile_acc_seg2: int = 6
    profile_vel_claw: int = 80
    profile_acc_claw: int = 5


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
    go2_spawn_height: float = 0.42
    go2_mount_offset_m: Tuple[float, float, float] = (0.0, 0.0, 0.08)
    go2_spawn_euler_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    go2_teleop_vx_mps: float = 0.35
    go2_teleop_vy_mps: float = 0.25
    go2_teleop_wz_radps: float = 0.80
    sim_target_enable: bool = True
    sim_target_xyz: Tuple[float, float, float] = (0.8, 0.0, 0.2)
    sim_target_radius: float = 0.05
    sim_target_color_rgba: Tuple[float, float, float, float] = (0.85, 0.15, 0.15, 1.0)
    sim_target_collision: bool = True
    sim_target_gravity: bool = False


@dataclass(frozen=True)
class PerceptionConfig:
    enabled: bool = True
    detector_config: str = ""
    mode: str = "external"
    detector: str = "external"
    provider: str = "local"
    autostart: bool = False
    preview_bind: str = "tcp://127.0.0.1:5570"
    preview_endpoint: str = "tcp://127.0.0.1:5570"
    preview_jpeg_quality: int = 75
    target_label: str = "sports ball"
    yolo_device: str = ""
    publish_hz: float = 15.0
    show_preview: bool = True
    pipeline: str = "yolo_seg"
    tracker: str = "csrt"
    track_lost_frames: int = 15
    reacquire_on_lost: bool = True
    track_scale_min: float = 0.05
    track_scale_stale_eps: float = 0.002
    track_redetect_stale_frames: int = 20
    track_bbox_shrink_ratio: float = 0.55
    track_init_bbox_padding: float = 1.25
    track_watchdog_min_frames: int = 8
    # CSRT: higher psr_threshold / lower learning rates = less sensitive tracking
    track_csrt_psr_threshold: float = 0.055
    track_csrt_scale_lr: float = 0.0008
    track_csrt_histogram_lr: float = 0.002
    track_csrt_padding: float = 2.0
    track_csrt_scale_step: float = 1.02
    track_bbox_smooth_alpha: float = 0.65
    track_aux_csrt: bool = True
    track_coast_max_frames: int = 12
    track_proximity_scale: float = 0.12
    track_proximity_mask_erode_px: int = 0
    track_redetect_grow_ratio: float = 1.12
    track_redetect_grow_ratio_stale: float = 1.03
    sim_camera_port: str = "tcp://127.0.0.1:5568"
    sim_camera_jpeg: bool = True
    run_local: bool = True

    def resolved_detector_config_path(self) -> Path:
        raw = str(self.detector_config).strip()
        path = Path(raw)
        if path.is_absolute():
            return path
        return Path(PROJECT_ROOT) / path


@dataclass(frozen=True)
class PickConfig:
    enabled: bool = True
    target_scale: float = 0.16
    scale_tol: float = 0.02
    center_tol: float = 0.12
    aim_center_tol: float = 0.08
    center_u_gain: float = 18.0
    center_v_gain: float = 18.0
    center_roll_max: float = 6.0
    center_seg_max: float = 6.0
    center_error_scale_max: float = 3.5
    center_stuck_iters: int = 30
    center_stuck_max_uv: float = 0.14
    target_uv_u: float = 0.5
    target_uv_v: float = 0.0
    quadrant_fill_min: float = 0.80
    ready_pose_standoff_m: float = 0.20
    approach_extend_m: float = 0.09
    approach_extend_step_m: float = 0.01
    grid_cols: int = 2
    grid_rows: int = 2
    target_grid_col: int = 1
    target_grid_row: int = 0
    linear_step_u: float = 2.0
    linear_gain: float = 40.0
    max_iters: int = 80
    require_track_frames: int = 3
    acquire_timeout_s: float = 30.0
    scale_stuck_iters: int = 40
    scale_stuck_ratio: float = 0.5
    approach_min_scale: float = 0.09
    approach_min_steps: int = 50
    approach_loose_center_tol: float = 0.10
    approach_scale_plateau_iters: int = 25
    approach_scale_plateau_eps: float = 0.004
    ready_pose_resolve_dir: bool = True
    ready_pose_max_dir_error_deg: float = 10.0
    # Post-aim corrected ready: best-effort dir acceptance when strict gate misses.
    ready_pose_corrected_max_dir_error_deg: float = 15.0
    ready_pose_skip_search_under_deg: float = 5.0
    ready_pose_lateral_offsets_m: Tuple[float, ...] = (-0.05, 0.0, 0.05)
    ready_pose_height_offsets_m: Tuple[float, ...] = (0.0, 0.05, 0.10)
    ready_pose_look_dot_min: float = 0.85
    ready_pose_align_mode: str = "full"
    ready_pose_align_skip_under_deg: float = 3.0
    ready_pose_align_top_k: int = 3
    auto_grasp_after_aim: bool = False
    grasp_standoff_m: float = 0.05
    grasp_guided_enabled: bool = True
    grasp_waypoint_step_m: float = 0.03
    grasp_guided_handoff_m: float = 0.04
    grasp_blind_uv_only: bool = True
    grasp_object_filter_alpha: float = 0.25
    grasp_approach_filter_alpha: float = 0.20
    grasp_blind_start_m: float = 0.06
    grasp_blind_approach_m: float = 0.02
    grasp_max_waypoints: int = 40
    grasp_waypoint_settle_s: float = 0.40
    grasp_waypoint_settle_timeout_s: float = 4.0
    grasp_waypoint_max_dir_error_deg: float = 12.0
    grasp_waypoint_max_approach_drift_deg: float = 18.0
    grasp_uv_center_tol: float = 0.0
    grasp_online_sag_enabled: bool = True
    grasp_online_sag_max_step_deg: float = 2.0
    grasp_skip_aim_recover_in_mock: bool = True
    sag_drift_max_dir_error_deg: float = 12.0
    sag_drift_max_lateral_m: float = 0.015
    sag_drift_axial_only: bool = True

    # Local image Jacobian grasp approach (LJI path; legacy unchanged when false).
    local_img_jacobian_enabled: bool = True
    lij_window_size: int = 8
    lij_min_samples: int = 4
    lij_damping: float = 0.05
    lij_gain_u: float = 0.35
    lij_gain_v: float = 0.35
    lij_gain_z: float = 0.45
    lij_z_bend_gain: float = 0.2
    lij_seg1_jacobian_scale: float = 0.30
    lij_seg2_jacobian_scale: float = 1.0
    lij_max_dq_theta1: float = 0.004
    lij_stall_steps: int = 0
    lij_stall_remain_eps_m: float = 0.005
    lij_joint_limit_margin_m: float = 0.001
    lij_joint_limit_margin_rad: float = 0.002
    lij_sag_min_lateral_m: float = 0.015
    lij_depth_settled_remain_delta_m: float = 0.005
    lij_max_dq_linear: float = 0.002
    lij_max_dq_angle: float = 0.006
    lij_uv_handoff_m: float = 0.10
    lij_far_linear_cap_m: float = 0.006
    lij_far_z_gain: float = 0.20
    lij_gain_scale_ref_m: float = 0.30
    lij_gain_scale_min: float = 0.12
    lij_settle_dwell_s: float = 0.02
    lij_settle_timeout_s: float = 0.35
    lij_dq_smooth_alpha: float = 0.35
    lij_pipelined_motion: bool = False
    lij_step_period_s: float = 0.10
    lij_condition_max: float = 100.0
    lij_probing_enabled: bool = False
    lij_probing_epsilon_linear: float = 0.001
    lij_probing_epsilon_angle: float = 0.01
    lij_uv_align_tol: float = 0.04
    lij_approach_bias_gain: float = 0.3
    lij_approach_seed_mode: str = "config"
    lij_approach_seed_q_delta: Tuple[float, float, float, float] = (0.0, 0.0, 0.01, 0.01)
    lij_approach_seed_travel_m: float = 0.003
    lij_sample_min_dq_norm: float = 0.0005
    blind_micro_start_m: float = 0.04
    grasp_close_tol_m: float = 0.003
    lij_depth_invalid_frames: int = 3
    lij_depth_valid_ratio_min: float = 0.6
    lij_depth_std_max_m: float = 0.012
    lij_depth_unstable_threshold_m: float = 0.06
    axial_micro_step_m: float = 0.005
    axial_micro_max_total_m: float = 0.025
    axial_micro_remain_max_m: float = 0.06
    axial_micro_remain_margin_m: float = 0.005
    lij_reacquire_max_steps: int = 8
    lij_reacquire_aim_after_steps: int = 4
    lij_reacquire_remain_fail_m: float = 0.08
    lij_reacquire_retrace_gain: float = 1.0
    lij_reacquire_axial_step_m: float = 0.012
    lij_reacquire_v_err_m: float = 0.45

    # Look phase: move to a feasible view pregrasp pose (tip looks at object).
    look_pose_standoff_m: float = 0.20
    look_pose_resolve_dir: bool = True
    look_pose_max_dir_error_deg: float = 10.0
    look_pose_skip_search_under_deg: float = 5.0
    look_pose_lateral_offsets_m: Tuple[float, ...] = (-0.05, 0.0, 0.05)
    look_pose_height_offsets_m: Tuple[float, ...] = (0.0, 0.05, 0.10)
    look_pose_look_dot_min: float = 0.85
    look_pose_align_top_k: int = 3
    look_pre_aim_enabled: bool = True
    look_pre_aim_max_steps: int = 8
    look_pre_aim_target_uv_u: float = 0.10
    look_pre_aim_target_uv_v: float = 0.0
    look_pre_aim_tol: float = 0.12
    look_pre_aim_awful_tol: float = 0.45
    look_pre_aim_step_scale: float = 0.35
    look_post_sag_trim_enabled: bool = True
    look_post_uv_recover_enabled: bool = True
    look_post_uv_max_steps: int = 30
    look_post_uv_center_tol: float = 0.05
    look_post_uv_acquire_s: float = 2.5

    ik_align_mode: str = "lite"
    ik_align_skip_under_deg: float = 10.0
    ik_align_rounds: int = 4

    # Mobile manipulation handoff: walk with gaze until close enough, then LJI grasp.
    mobile_handoff_distance_m: float = 0.30
    mobile_handoff_timeout_slack_m: float = 0.05
    mobile_approach_vx_mps: float = 0.18
    mobile_approach_timeout_s: float = 300.0
    mobile_stop_settle_s: float = 0.40
    mobile_gaze_mode: str = "pitch_preview"


@dataclass(frozen=True)
class ExperimentConfig:
    ownership_enable: bool = False
    preview_fallback_uv_ff: bool = False


@dataclass(frozen=True)
class AppConfigBundle:
    sim_param: SimParam
    sim_config: SimConfig
    hardware_config: HardwareConfig
    joint_limit: JointLimit
    spawn_config: SpawnConfig
    urdf_export_config: UrdfExportConfig
    ik_config: IkConfig
    perception_config: PerceptionConfig
    pick_config: PickConfig
    go2_locomotion_config: Go2LocomotionConfig
    go2_hardware_config: Go2HardwareConfig
    gaze_stabilizer_config: GazeStabilizerConfig
    experiment_config: ExperimentConfig
    mapping_config: proto.SimMappingConfig


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


def _parse_float_list(text: str, default: Tuple[float, ...]) -> Tuple[float, ...]:
    raw = str(text).strip()
    if not raw:
        return default
    parts = [x.strip() for x in raw.split(",") if x.strip()]
    if not parts:
        return default
    try:
        return tuple(float(x) for x in parts)
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


def _load_perception_config(cp: configparser.ConfigParser, defaults: AppConfigBundle) -> PerceptionConfig:
    pc0 = defaults.perception_config
    run_local = cp.getboolean("perception", "run_local", fallback=pc0.run_local)
    provider_default = str(getattr(pc0, "provider", "local") or "local").strip().lower()
    if not cp.has_option("perception", "provider"):
        provider_default = "local" if bool(run_local) else "host"
    provider = cp.get("perception", "provider", fallback=provider_default).strip().lower()
    if provider not in ("local", "host"):
        provider = provider_default if provider_default in ("local", "host") else "local"
    return PerceptionConfig(
        enabled=cp.getboolean("perception", "enabled", fallback=pc0.enabled),
        detector_config=cp.get("perception", "detector_config", fallback=pc0.detector_config),
        mode=cp.get("perception", "mode", fallback=pc0.mode),
        detector=cp.get("perception", "detector", fallback=pc0.detector),
        provider=provider,
        autostart=cp.getboolean("perception", "autostart", fallback=pc0.autostart),
        preview_bind=cp.get("perception", "preview_bind", fallback=pc0.preview_bind),
        preview_endpoint=cp.get("perception", "preview_endpoint", fallback=pc0.preview_endpoint),
        preview_jpeg_quality=cp.getint(
            "perception",
            "preview_jpeg_quality",
            fallback=pc0.preview_jpeg_quality,
        ),
        target_label=cp.get("perception", "target_label", fallback=pc0.target_label),
        yolo_device=cp.get("perception", "yolo_device", fallback=pc0.yolo_device),
        publish_hz=cp.getfloat("perception", "publish_hz", fallback=pc0.publish_hz),
        show_preview=cp.getboolean("perception", "show_preview", fallback=pc0.show_preview),
        pipeline=cp.get("perception", "pipeline", fallback=pc0.pipeline),
        tracker=cp.get("perception", "tracker", fallback=pc0.tracker),
        track_lost_frames=cp.getint("perception", "track_lost_frames", fallback=pc0.track_lost_frames),
        reacquire_on_lost=cp.getboolean("perception", "reacquire_on_lost", fallback=pc0.reacquire_on_lost),
        track_scale_min=cp.getfloat("perception", "track_scale_min", fallback=pc0.track_scale_min),
        track_scale_stale_eps=cp.getfloat(
            "perception", "track_scale_stale_eps", fallback=pc0.track_scale_stale_eps
        ),
        track_redetect_stale_frames=cp.getint(
            "perception", "track_redetect_stale_frames", fallback=pc0.track_redetect_stale_frames
        ),
        track_bbox_shrink_ratio=cp.getfloat(
            "perception", "track_bbox_shrink_ratio", fallback=pc0.track_bbox_shrink_ratio
        ),
        track_init_bbox_padding=cp.getfloat(
            "perception", "track_init_bbox_padding", fallback=pc0.track_init_bbox_padding
        ),
        track_watchdog_min_frames=cp.getint(
            "perception", "track_watchdog_min_frames", fallback=pc0.track_watchdog_min_frames
        ),
        track_csrt_psr_threshold=cp.getfloat(
            "perception", "track_csrt_psr_threshold", fallback=pc0.track_csrt_psr_threshold
        ),
        track_csrt_scale_lr=cp.getfloat(
            "perception", "track_csrt_scale_lr", fallback=pc0.track_csrt_scale_lr
        ),
        track_csrt_histogram_lr=cp.getfloat(
            "perception", "track_csrt_histogram_lr", fallback=pc0.track_csrt_histogram_lr
        ),
        track_csrt_padding=cp.getfloat(
            "perception", "track_csrt_padding", fallback=pc0.track_csrt_padding
        ),
        track_csrt_scale_step=cp.getfloat(
            "perception", "track_csrt_scale_step", fallback=pc0.track_csrt_scale_step
        ),
        track_bbox_smooth_alpha=cp.getfloat(
            "perception", "track_bbox_smooth_alpha", fallback=pc0.track_bbox_smooth_alpha
        ),
        track_aux_csrt=cp.getboolean("perception", "track_aux_csrt", fallback=pc0.track_aux_csrt),
        track_coast_max_frames=cp.getint(
            "perception", "track_coast_max_frames", fallback=pc0.track_coast_max_frames
        ),
        track_proximity_scale=cp.getfloat(
            "perception", "track_proximity_scale", fallback=pc0.track_proximity_scale
        ),
        track_proximity_mask_erode_px=cp.getint(
            "perception",
            "track_proximity_mask_erode_px",
            fallback=pc0.track_proximity_mask_erode_px,
        ),
        track_redetect_grow_ratio=cp.getfloat(
            "perception", "track_redetect_grow_ratio", fallback=pc0.track_redetect_grow_ratio
        ),
        track_redetect_grow_ratio_stale=cp.getfloat(
            "perception",
            "track_redetect_grow_ratio_stale",
            fallback=pc0.track_redetect_grow_ratio_stale,
        ),
        sim_camera_port=cp.get("runtime", "sim_camera_port", fallback=pc0.sim_camera_port),
        sim_camera_jpeg=cp.getboolean("runtime", "sim_camera_jpeg", fallback=pc0.sim_camera_jpeg),
        run_local=bool(run_local),
    )


def _load_pick_config(cp: configparser.ConfigParser, defaults: AppConfigBundle) -> PickConfig:
    from engine.vision.pick.core import grid_cell_center_uv, quadrant_fill_target_scale

    pk0 = defaults.pick_config
    quadrant_fill = cp.getfloat("pick", "quadrant_fill_min", fallback=pk0.quadrant_fill_min)
    scale_default = quadrant_fill_target_scale(quadrant_fill)
    grid_cols = cp.getint("pick", "grid_cols", fallback=pk0.grid_cols)
    grid_rows = cp.getint("pick", "grid_rows", fallback=pk0.grid_rows)
    grid_col = cp.getint("pick", "target_grid_col", fallback=pk0.target_grid_col)
    grid_row = cp.getint("pick", "target_grid_row", fallback=pk0.target_grid_row)
    if cp.has_option("pick", "target_uv_u") and cp.has_option("pick", "target_uv_v"):
        target_u = cp.getfloat("pick", "target_uv_u", fallback=pk0.target_uv_u)
        target_v = cp.getfloat("pick", "target_uv_v", fallback=pk0.target_uv_v)
    else:
        target_u, target_v = grid_cell_center_uv(
            grid_col,
            grid_row,
            grid_cols=grid_cols,
            grid_rows=grid_rows,
        )
    return PickConfig(
        enabled=cp.getboolean("pick", "enabled", fallback=pk0.enabled),
        target_scale=cp.getfloat("pick", "target_scale", fallback=scale_default),
        scale_tol=cp.getfloat("pick", "scale_tol", fallback=pk0.scale_tol),
        center_tol=cp.getfloat("pick", "center_tol", fallback=pk0.center_tol),
        aim_center_tol=cp.getfloat("pick", "aim_center_tol", fallback=pk0.aim_center_tol),
        center_u_gain=cp.getfloat("pick", "center_u_gain", fallback=pk0.center_u_gain),
        center_v_gain=cp.getfloat("pick", "center_v_gain", fallback=pk0.center_v_gain),
        center_roll_max=cp.getfloat("pick", "center_roll_max", fallback=pk0.center_roll_max),
        center_seg_max=cp.getfloat("pick", "center_seg_max", fallback=pk0.center_seg_max),
        center_error_scale_max=cp.getfloat(
            "pick", "center_error_scale_max", fallback=pk0.center_error_scale_max
        ),
        center_stuck_iters=cp.getint(
            "pick", "center_stuck_iters", fallback=pk0.center_stuck_iters
        ),
        center_stuck_max_uv=cp.getfloat(
            "pick", "center_stuck_max_uv", fallback=pk0.center_stuck_max_uv
        ),
        target_uv_u=float(target_u),
        target_uv_v=float(target_v),
        quadrant_fill_min=float(quadrant_fill),
        ready_pose_standoff_m=cp.getfloat(
            "pick", "ready_pose_standoff_m", fallback=pk0.ready_pose_standoff_m
        ),
        approach_extend_m=cp.getfloat("pick", "approach_extend_m", fallback=pk0.approach_extend_m),
        approach_extend_step_m=cp.getfloat(
            "pick", "approach_extend_step_m", fallback=pk0.approach_extend_step_m
        ),
        grid_cols=int(grid_cols),
        grid_rows=int(grid_rows),
        target_grid_col=int(grid_col),
        target_grid_row=int(grid_row),
        linear_step_u=cp.getfloat("pick", "linear_step_u", fallback=pk0.linear_step_u),
        linear_gain=cp.getfloat("pick", "linear_gain", fallback=pk0.linear_gain),
        max_iters=cp.getint("pick", "max_iters", fallback=pk0.max_iters),
        require_track_frames=cp.getint("pick", "require_track_frames", fallback=pk0.require_track_frames),
        acquire_timeout_s=cp.getfloat("pick", "acquire_timeout_s", fallback=pk0.acquire_timeout_s),
        scale_stuck_iters=cp.getint("pick", "scale_stuck_iters", fallback=pk0.scale_stuck_iters),
        scale_stuck_ratio=cp.getfloat("pick", "scale_stuck_ratio", fallback=pk0.scale_stuck_ratio),
        approach_min_scale=cp.getfloat("pick", "approach_min_scale", fallback=pk0.approach_min_scale),
        approach_min_steps=cp.getint("pick", "approach_min_steps", fallback=pk0.approach_min_steps),
        approach_loose_center_tol=cp.getfloat(
            "pick", "approach_loose_center_tol", fallback=pk0.approach_loose_center_tol
        ),
        approach_scale_plateau_iters=cp.getint(
            "pick", "approach_scale_plateau_iters", fallback=pk0.approach_scale_plateau_iters
        ),
        approach_scale_plateau_eps=cp.getfloat(
            "pick", "approach_scale_plateau_eps", fallback=pk0.approach_scale_plateau_eps
        ),
        ready_pose_resolve_dir=cp.getboolean(
            "pick", "ready_pose_resolve_dir", fallback=pk0.ready_pose_resolve_dir
        ),
        ready_pose_max_dir_error_deg=cp.getfloat(
            "pick", "ready_pose_max_dir_error_deg", fallback=pk0.ready_pose_max_dir_error_deg
        ),
        ready_pose_corrected_max_dir_error_deg=cp.getfloat(
            "pick",
            "ready_pose_corrected_max_dir_error_deg",
            fallback=pk0.ready_pose_corrected_max_dir_error_deg,
        ),
        ready_pose_skip_search_under_deg=cp.getfloat(
            "pick", "ready_pose_skip_search_under_deg", fallback=pk0.ready_pose_skip_search_under_deg
        ),
        ready_pose_lateral_offsets_m=_parse_float_list(
            cp.get("pick", "ready_pose_lateral_offsets_m", fallback=""),
            pk0.ready_pose_lateral_offsets_m,
        ),
        ready_pose_height_offsets_m=_parse_float_list(
            cp.get("pick", "ready_pose_height_offsets_m", fallback=""),
            pk0.ready_pose_height_offsets_m,
        ),
        ready_pose_look_dot_min=cp.getfloat(
            "pick", "ready_pose_look_dot_min", fallback=pk0.ready_pose_look_dot_min
        ),
        ready_pose_align_mode=cp.get(
            "pick", "ready_pose_align_mode", fallback=pk0.ready_pose_align_mode
        ).strip().lower(),
        ready_pose_align_skip_under_deg=cp.getfloat(
            "pick", "ready_pose_align_skip_under_deg", fallback=pk0.ready_pose_align_skip_under_deg
        ),
        ready_pose_align_top_k=cp.getint(
            "pick", "ready_pose_align_top_k", fallback=pk0.ready_pose_align_top_k
        ),
        auto_grasp_after_aim=cp.getboolean(
            "pick", "auto_grasp_after_aim", fallback=pk0.auto_grasp_after_aim
        ),
        grasp_standoff_m=cp.getfloat(
            "pick", "grasp_standoff_m", fallback=pk0.grasp_standoff_m
        ),
        grasp_guided_enabled=cp.getboolean(
            "pick", "grasp_guided_enabled", fallback=pk0.grasp_guided_enabled
        ),
        grasp_waypoint_step_m=cp.getfloat(
            "pick", "grasp_waypoint_step_m", fallback=pk0.grasp_waypoint_step_m
        ),
        grasp_guided_handoff_m=cp.getfloat(
            "pick", "grasp_guided_handoff_m", fallback=pk0.grasp_guided_handoff_m
        ),
        grasp_blind_uv_only=cp.getboolean(
            "pick", "grasp_blind_uv_only", fallback=pk0.grasp_blind_uv_only
        ),
        grasp_object_filter_alpha=cp.getfloat(
            "pick", "grasp_object_filter_alpha", fallback=pk0.grasp_object_filter_alpha
        ),
        grasp_approach_filter_alpha=cp.getfloat(
            "pick", "grasp_approach_filter_alpha", fallback=pk0.grasp_approach_filter_alpha
        ),
        grasp_blind_start_m=cp.getfloat(
            "pick", "grasp_blind_start_m", fallback=pk0.grasp_blind_start_m
        ),
        grasp_blind_approach_m=cp.getfloat(
            "pick", "grasp_blind_approach_m", fallback=pk0.grasp_blind_approach_m
        ),
        grasp_max_waypoints=cp.getint(
            "pick", "grasp_max_waypoints", fallback=pk0.grasp_max_waypoints
        ),
        grasp_waypoint_settle_s=cp.getfloat(
            "pick", "grasp_waypoint_settle_s", fallback=pk0.grasp_waypoint_settle_s
        ),
        grasp_waypoint_settle_timeout_s=cp.getfloat(
            "pick",
            "grasp_waypoint_settle_timeout_s",
            fallback=pk0.grasp_waypoint_settle_timeout_s,
        ),
        grasp_waypoint_max_dir_error_deg=cp.getfloat(
            "pick",
            "grasp_waypoint_max_dir_error_deg",
            fallback=pk0.grasp_waypoint_max_dir_error_deg,
        ),
        grasp_waypoint_max_approach_drift_deg=cp.getfloat(
            "pick",
            "grasp_waypoint_max_approach_drift_deg",
            fallback=pk0.grasp_waypoint_max_approach_drift_deg,
        ),
        grasp_uv_center_tol=cp.getfloat(
            "pick", "grasp_uv_center_tol", fallback=pk0.grasp_uv_center_tol
        ),
        grasp_online_sag_enabled=cp.getboolean(
            "pick", "grasp_online_sag_enabled", fallback=pk0.grasp_online_sag_enabled
        ),
        grasp_online_sag_max_step_deg=cp.getfloat(
            "pick", "grasp_online_sag_max_step_deg", fallback=pk0.grasp_online_sag_max_step_deg
        ),
        grasp_skip_aim_recover_in_mock=cp.getboolean(
            "pick",
            "grasp_skip_aim_recover_in_mock",
            fallback=pk0.grasp_skip_aim_recover_in_mock,
        ),
        sag_drift_max_dir_error_deg=cp.getfloat(
            "pick", "sag_drift_max_dir_error_deg", fallback=pk0.sag_drift_max_dir_error_deg
        ),
        sag_drift_max_lateral_m=cp.getfloat(
            "pick", "sag_drift_max_lateral_m", fallback=pk0.sag_drift_max_lateral_m
        ),
        sag_drift_axial_only=cp.getboolean(
            "pick", "sag_drift_axial_only", fallback=pk0.sag_drift_axial_only
        ),
        local_img_jacobian_enabled=cp.getboolean(
            "pick", "local_img_jacobian_enabled", fallback=pk0.local_img_jacobian_enabled
        ),
        lij_window_size=cp.getint("pick", "lij_window_size", fallback=pk0.lij_window_size),
        lij_min_samples=cp.getint("pick", "lij_min_samples", fallback=pk0.lij_min_samples),
        lij_damping=cp.getfloat("pick", "lij_damping", fallback=pk0.lij_damping),
        lij_gain_u=cp.getfloat("pick", "lij_gain_u", fallback=pk0.lij_gain_u),
        lij_gain_v=cp.getfloat("pick", "lij_gain_v", fallback=pk0.lij_gain_v),
        lij_gain_z=cp.getfloat("pick", "lij_gain_z", fallback=pk0.lij_gain_z),
        lij_z_bend_gain=cp.getfloat(
            "pick", "lij_z_bend_gain", fallback=pk0.lij_z_bend_gain
        ),
        lij_seg1_jacobian_scale=cp.getfloat(
            "pick", "lij_seg1_jacobian_scale", fallback=pk0.lij_seg1_jacobian_scale
        ),
        lij_seg2_jacobian_scale=cp.getfloat(
            "pick", "lij_seg2_jacobian_scale", fallback=pk0.lij_seg2_jacobian_scale
        ),
        lij_max_dq_theta1=cp.getfloat(
            "pick", "lij_max_dq_theta1", fallback=pk0.lij_max_dq_theta1
        ),
        lij_stall_steps=cp.getint("pick", "lij_stall_steps", fallback=pk0.lij_stall_steps),
        lij_stall_remain_eps_m=cp.getfloat(
            "pick", "lij_stall_remain_eps_m", fallback=pk0.lij_stall_remain_eps_m
        ),
        lij_joint_limit_margin_m=cp.getfloat(
            "pick", "lij_joint_limit_margin_m", fallback=pk0.lij_joint_limit_margin_m
        ),
        lij_joint_limit_margin_rad=cp.getfloat(
            "pick", "lij_joint_limit_margin_rad", fallback=pk0.lij_joint_limit_margin_rad
        ),
        lij_sag_min_lateral_m=cp.getfloat(
            "pick", "lij_sag_min_lateral_m", fallback=pk0.lij_sag_min_lateral_m
        ),
        lij_depth_settled_remain_delta_m=cp.getfloat(
            "pick",
            "lij_depth_settled_remain_delta_m",
            fallback=pk0.lij_depth_settled_remain_delta_m,
        ),
        lij_max_dq_linear=cp.getfloat(
            "pick", "lij_max_dq_linear", fallback=pk0.lij_max_dq_linear
        ),
        lij_max_dq_angle=cp.getfloat(
            "pick", "lij_max_dq_angle", fallback=pk0.lij_max_dq_angle
        ),
        lij_uv_handoff_m=cp.getfloat(
            "pick", "lij_uv_handoff_m", fallback=pk0.lij_uv_handoff_m
        ),
        lij_far_linear_cap_m=cp.getfloat(
            "pick", "lij_far_linear_cap_m", fallback=pk0.lij_far_linear_cap_m
        ),
        lij_far_z_gain=cp.getfloat(
            "pick", "lij_far_z_gain", fallback=pk0.lij_far_z_gain
        ),
        lij_gain_scale_ref_m=cp.getfloat(
            "pick", "lij_gain_scale_ref_m", fallback=pk0.lij_gain_scale_ref_m
        ),
        lij_gain_scale_min=cp.getfloat(
            "pick", "lij_gain_scale_min", fallback=pk0.lij_gain_scale_min
        ),
        lij_settle_dwell_s=cp.getfloat(
            "pick", "lij_settle_dwell_s", fallback=pk0.lij_settle_dwell_s
        ),
        lij_settle_timeout_s=cp.getfloat(
            "pick", "lij_settle_timeout_s", fallback=pk0.lij_settle_timeout_s
        ),
        lij_dq_smooth_alpha=cp.getfloat(
            "pick", "lij_dq_smooth_alpha", fallback=pk0.lij_dq_smooth_alpha
        ),
        lij_pipelined_motion=cp.getboolean(
            "pick", "lij_pipelined_motion", fallback=pk0.lij_pipelined_motion
        ),
        lij_step_period_s=cp.getfloat(
            "pick", "lij_step_period_s", fallback=pk0.lij_step_period_s
        ),
        lij_condition_max=cp.getfloat(
            "pick", "lij_condition_max", fallback=pk0.lij_condition_max
        ),
        lij_probing_enabled=cp.getboolean(
            "pick", "lij_probing_enabled", fallback=pk0.lij_probing_enabled
        ),
        lij_probing_epsilon_linear=cp.getfloat(
            "pick", "lij_probing_epsilon_linear", fallback=pk0.lij_probing_epsilon_linear
        ),
        lij_probing_epsilon_angle=cp.getfloat(
            "pick", "lij_probing_epsilon_angle", fallback=pk0.lij_probing_epsilon_angle
        ),
        lij_uv_align_tol=cp.getfloat(
            "pick", "lij_uv_align_tol", fallback=pk0.lij_uv_align_tol
        ),
        lij_approach_bias_gain=cp.getfloat(
            "pick", "lij_approach_bias_gain", fallback=pk0.lij_approach_bias_gain
        ),
        lij_approach_seed_mode=cp.get(
            "pick", "lij_approach_seed_mode", fallback=pk0.lij_approach_seed_mode
        ).strip().lower(),
        lij_approach_seed_q_delta=_parse_float_list(
            cp.get("pick", "lij_approach_seed_q_delta", fallback=""),
            pk0.lij_approach_seed_q_delta,
        ),
        lij_approach_seed_travel_m=cp.getfloat(
            "pick", "lij_approach_seed_travel_m", fallback=pk0.lij_approach_seed_travel_m
        ),
        lij_sample_min_dq_norm=cp.getfloat(
            "pick", "lij_sample_min_dq_norm", fallback=pk0.lij_sample_min_dq_norm
        ),
        blind_micro_start_m=cp.getfloat(
            "pick", "blind_micro_start_m", fallback=pk0.blind_micro_start_m
        ),
        grasp_close_tol_m=cp.getfloat(
            "pick", "grasp_close_tol_m", fallback=pk0.grasp_close_tol_m
        ),
        lij_depth_invalid_frames=cp.getint(
            "pick", "lij_depth_invalid_frames", fallback=pk0.lij_depth_invalid_frames
        ),
        lij_depth_valid_ratio_min=cp.getfloat(
            "pick", "lij_depth_valid_ratio_min", fallback=pk0.lij_depth_valid_ratio_min
        ),
        lij_depth_std_max_m=cp.getfloat(
            "pick", "lij_depth_std_max_m", fallback=pk0.lij_depth_std_max_m
        ),
        lij_depth_unstable_threshold_m=cp.getfloat(
            "pick",
            "lij_depth_unstable_threshold_m",
            fallback=pk0.lij_depth_unstable_threshold_m,
        ),
        axial_micro_step_m=cp.getfloat(
            "pick", "axial_micro_step_m", fallback=pk0.axial_micro_step_m
        ),
        axial_micro_max_total_m=cp.getfloat(
            "pick", "axial_micro_max_total_m", fallback=pk0.axial_micro_max_total_m
        ),
        axial_micro_remain_max_m=cp.getfloat(
            "pick", "axial_micro_remain_max_m", fallback=pk0.axial_micro_remain_max_m
        ),
        axial_micro_remain_margin_m=cp.getfloat(
            "pick", "axial_micro_remain_margin_m", fallback=pk0.axial_micro_remain_margin_m
        ),
        lij_reacquire_max_steps=cp.getint(
            "pick", "lij_reacquire_max_steps", fallback=pk0.lij_reacquire_max_steps
        ),
        lij_reacquire_aim_after_steps=cp.getint(
            "pick", "lij_reacquire_aim_after_steps", fallback=pk0.lij_reacquire_aim_after_steps
        ),
        lij_reacquire_remain_fail_m=cp.getfloat(
            "pick", "lij_reacquire_remain_fail_m", fallback=pk0.lij_reacquire_remain_fail_m
        ),
        lij_reacquire_retrace_gain=cp.getfloat(
            "pick", "lij_reacquire_retrace_gain", fallback=pk0.lij_reacquire_retrace_gain
        ),
        lij_reacquire_axial_step_m=cp.getfloat(
            "pick", "lij_reacquire_axial_step_m", fallback=pk0.lij_reacquire_axial_step_m
        ),
        lij_reacquire_v_err_m=cp.getfloat(
            "pick", "lij_reacquire_v_err_m", fallback=pk0.lij_reacquire_v_err_m
        ),
        look_pose_standoff_m=cp.getfloat(
            "pick", "look_pose_standoff_m", fallback=pk0.look_pose_standoff_m
        ),
        look_pose_resolve_dir=cp.getboolean(
            "pick", "look_pose_resolve_dir", fallback=pk0.look_pose_resolve_dir
        ),
        look_pose_max_dir_error_deg=cp.getfloat(
            "pick", "look_pose_max_dir_error_deg", fallback=pk0.look_pose_max_dir_error_deg
        ),
        look_pose_skip_search_under_deg=cp.getfloat(
            "pick", "look_pose_skip_search_under_deg", fallback=pk0.look_pose_skip_search_under_deg
        ),
        look_pose_lateral_offsets_m=_parse_float_list(
            cp.get("pick", "look_pose_lateral_offsets_m", fallback=""),
            pk0.look_pose_lateral_offsets_m,
        ),
        look_pose_height_offsets_m=_parse_float_list(
            cp.get("pick", "look_pose_height_offsets_m", fallback=""),
            pk0.look_pose_height_offsets_m,
        ),
        look_pose_look_dot_min=cp.getfloat(
            "pick", "look_pose_look_dot_min", fallback=pk0.look_pose_look_dot_min
        ),
        look_pose_align_top_k=cp.getint(
            "pick", "look_pose_align_top_k", fallback=pk0.look_pose_align_top_k
        ),
        look_pre_aim_enabled=cp.getboolean(
            "pick", "look_pre_aim_enabled", fallback=pk0.look_pre_aim_enabled
        ),
        look_pre_aim_max_steps=cp.getint(
            "pick", "look_pre_aim_max_steps", fallback=pk0.look_pre_aim_max_steps
        ),
        look_pre_aim_target_uv_u=cp.getfloat(
            "pick", "look_pre_aim_target_uv_u", fallback=pk0.look_pre_aim_target_uv_u
        ),
        look_pre_aim_target_uv_v=cp.getfloat(
            "pick", "look_pre_aim_target_uv_v", fallback=pk0.look_pre_aim_target_uv_v
        ),
        look_pre_aim_tol=cp.getfloat(
            "pick", "look_pre_aim_tol", fallback=pk0.look_pre_aim_tol
        ),
        look_pre_aim_awful_tol=cp.getfloat(
            "pick", "look_pre_aim_awful_tol", fallback=pk0.look_pre_aim_awful_tol
        ),
        look_pre_aim_step_scale=cp.getfloat(
            "pick", "look_pre_aim_step_scale", fallback=pk0.look_pre_aim_step_scale
        ),
        look_post_sag_trim_enabled=cp.getboolean(
            "pick", "look_post_sag_trim_enabled", fallback=pk0.look_post_sag_trim_enabled
        ),
        look_post_uv_recover_enabled=cp.getboolean(
            "pick", "look_post_uv_recover_enabled", fallback=pk0.look_post_uv_recover_enabled
        ),
        look_post_uv_max_steps=cp.getint(
            "pick", "look_post_uv_max_steps", fallback=pk0.look_post_uv_max_steps
        ),
        look_post_uv_center_tol=cp.getfloat(
            "pick", "look_post_uv_center_tol", fallback=pk0.look_post_uv_center_tol
        ),
        look_post_uv_acquire_s=cp.getfloat(
            "pick", "look_post_uv_acquire_s", fallback=pk0.look_post_uv_acquire_s
        ),
        ik_align_mode=cp.get("pick", "ik_align_mode", fallback=pk0.ik_align_mode).strip().lower(),
        ik_align_skip_under_deg=cp.getfloat(
            "pick", "ik_align_skip_under_deg", fallback=pk0.ik_align_skip_under_deg
        ),
        ik_align_rounds=cp.getint("pick", "ik_align_rounds", fallback=pk0.ik_align_rounds),
        mobile_handoff_distance_m=cp.getfloat(
            "pick", "mobile_handoff_distance_m", fallback=pk0.mobile_handoff_distance_m
        ),
        mobile_handoff_timeout_slack_m=cp.getfloat(
            "pick", "mobile_handoff_timeout_slack_m", fallback=pk0.mobile_handoff_timeout_slack_m
        ),
        mobile_approach_vx_mps=cp.getfloat(
            "pick", "mobile_approach_vx_mps", fallback=pk0.mobile_approach_vx_mps
        ),
        mobile_approach_timeout_s=cp.getfloat(
            "pick", "mobile_approach_timeout_s", fallback=pk0.mobile_approach_timeout_s
        ),
        mobile_stop_settle_s=cp.getfloat(
            "pick", "mobile_stop_settle_s", fallback=pk0.mobile_stop_settle_s
        ),
        mobile_gaze_mode=cp.get("pick", "mobile_gaze_mode", fallback=pk0.mobile_gaze_mode).strip().lower(),
    )


def _load_go2_locomotion_config(cp: configparser.ConfigParser, defaults: AppConfigBundle) -> Go2LocomotionConfig:
    gl0 = defaults.go2_locomotion_config
    return Go2LocomotionConfig(
        mirror_from_host=cp.getboolean(
            "go2_locomotion", "mirror_from_host", fallback=gl0.mirror_from_host
        ),
        mode=cp.get("go2_locomotion", "mode", fallback=gl0.mode).strip().lower(),
        stance_time_s=cp.getfloat("go2_locomotion", "stance_time_s", fallback=gl0.stance_time_s),
        swing_time_s=cp.getfloat("go2_locomotion", "swing_time_s", fallback=gl0.swing_time_s),
        raibert_kv=cp.getfloat("go2_locomotion", "raibert_kv", fallback=gl0.raibert_kv),
        nominal_body_height_m=cp.getfloat(
            "go2_locomotion", "nominal_body_height_m", fallback=gl0.nominal_body_height_m
        ),
        foot_swing_height_m=cp.getfloat(
            "go2_locomotion", "foot_swing_height_m", fallback=gl0.foot_swing_height_m
        ),
        leg_kp=cp.getfloat("go2_locomotion", "leg_kp", fallback=gl0.leg_kp),
        leg_kv=cp.getfloat("go2_locomotion", "leg_kv", fallback=gl0.leg_kv),
        command_idle_threshold=cp.getfloat(
            "go2_locomotion", "command_idle_threshold", fallback=gl0.command_idle_threshold
        ),
        ground_height_m=cp.getfloat("go2_locomotion", "ground_height_m", fallback=gl0.ground_height_m),
        foot_radius_m=cp.getfloat("go2_locomotion", "foot_radius_m", fallback=gl0.foot_radius_m),
        leg_max_rate_radps=cp.getfloat(
            "go2_locomotion", "leg_max_rate_radps", fallback=gl0.leg_max_rate_radps
        ),
        foot_max_step_m=cp.getfloat("go2_locomotion", "foot_max_step_m", fallback=gl0.foot_max_step_m),
        base_vel_blend=cp.getfloat("go2_locomotion", "base_vel_blend", fallback=gl0.base_vel_blend),
        gait_hz=cp.getfloat("go2_locomotion", "gait_hz", fallback=gl0.gait_hz),
        gait_duty=cp.getfloat("go2_locomotion", "gait_duty", fallback=gl0.gait_duty),
        z_pos_des_m=cp.getfloat("go2_locomotion", "z_pos_des_m", fallback=gl0.z_pos_des_m),
        mpc_steps_per_gait=cp.getint("go2_locomotion", "mpc_steps_per_gait", fallback=gl0.mpc_steps_per_gait),
        torque_safety_scale=cp.getfloat(
            "go2_locomotion", "torque_safety_scale", fallback=gl0.torque_safety_scale
        ),
        mpc_leg_kv_damping=cp.getfloat(
            "go2_locomotion", "mpc_leg_kv_damping", fallback=gl0.mpc_leg_kv_damping
        ),
        mpc_ctrl_hz=cp.getfloat("go2_locomotion", "mpc_ctrl_hz", fallback=gl0.mpc_ctrl_hz),
        mpc_command_ramp_s=cp.getfloat(
            "go2_locomotion", "mpc_command_ramp_s", fallback=gl0.mpc_command_ramp_s
        ),
        mpc_torque_ramp_s=cp.getfloat(
            "go2_locomotion", "mpc_torque_ramp_s", fallback=gl0.mpc_torque_ramp_s
        ),
        mpc_torque_warmup_s=cp.getfloat(
            "go2_locomotion", "mpc_torque_warmup_s", fallback=gl0.mpc_torque_warmup_s
        ),
        mpc_ready_pose_s=cp.getfloat(
            "go2_locomotion", "mpc_ready_pose_s", fallback=gl0.mpc_ready_pose_s
        ),
        mpc_ready_kp=cp.getfloat("go2_locomotion", "mpc_ready_kp", fallback=gl0.mpc_ready_kp),
        mpc_ready_kv=cp.getfloat("go2_locomotion", "mpc_ready_kv", fallback=gl0.mpc_ready_kv),
        mpc_aux_kp=cp.getfloat("go2_locomotion", "mpc_aux_kp", fallback=gl0.mpc_aux_kp),
        mpc_aux_kv=cp.getfloat("go2_locomotion", "mpc_aux_kv", fallback=gl0.mpc_aux_kv),
        mpc_tau_filter_alpha=cp.getfloat(
            "go2_locomotion", "mpc_tau_filter_alpha", fallback=gl0.mpc_tau_filter_alpha
        ),
        mpc_force_filter_alpha=cp.getfloat(
            "go2_locomotion", "mpc_force_filter_alpha", fallback=gl0.mpc_force_filter_alpha
        ),
        mpc_foot_placement_scale=cp.getfloat(
            "go2_locomotion", "mpc_foot_placement_scale", fallback=gl0.mpc_foot_placement_scale
        ),
        mpc_payload_enable=cp.getboolean(
            "go2_locomotion", "mpc_payload_enable", fallback=gl0.mpc_payload_enable
        ),
        mpc_payload_mass_kg=cp.getfloat(
            "go2_locomotion", "mpc_payload_mass_kg", fallback=gl0.mpc_payload_mass_kg
        ),
        mpc_pitch_trim_gain_x_forward=cp.getfloat(
            "go2_locomotion", "mpc_pitch_trim_gain_x_forward", fallback=gl0.mpc_pitch_trim_gain_x_forward
        ),
        mpc_pitch_trim_gain_x_backward=cp.getfloat(
            "go2_locomotion", "mpc_pitch_trim_gain_x_backward", fallback=gl0.mpc_pitch_trim_gain_x_backward
        ),
        mpc_pitch_trim_gain_z=cp.getfloat(
            "go2_locomotion", "mpc_pitch_trim_gain_z", fallback=gl0.mpc_pitch_trim_gain_z
        ),
        mpc_pitch_trim_z_ref_m=cp.getfloat(
            "go2_locomotion", "mpc_pitch_trim_z_ref_m", fallback=gl0.mpc_pitch_trim_z_ref_m
        ),
        mpc_pitch_trim_max_rad=cp.getfloat(
            "go2_locomotion", "mpc_pitch_trim_max_rad", fallback=gl0.mpc_pitch_trim_max_rad
        ),
    )


def _load_go2_hardware_config(cp: configparser.ConfigParser, defaults: AppConfigBundle) -> Go2HardwareConfig:
    gh0 = defaults.go2_hardware_config
    if not cp.has_section("go2_hardware"):
        return gh0
    return Go2HardwareConfig(
        enabled=cp.getboolean("go2_hardware", "enabled", fallback=gh0.enabled),
        backend=cp.get("go2_hardware", "backend", fallback=gh0.backend).strip().lower(),
        sport_request_topic=cp.get(
            "go2_hardware", "sport_request_topic", fallback=gh0.sport_request_topic
        ).strip(),
        pose_source=cp.get("go2_hardware", "pose_source", fallback=gh0.pose_source).strip().lower(),
        odom_topic=cp.get("go2_hardware", "odom_topic", fallback=gh0.odom_topic).strip(),
        sport_state_topic=cp.get(
            "go2_hardware", "sport_state_topic", fallback=gh0.sport_state_topic
        ).strip(),
        leg_sync=cp.getboolean("go2_hardware", "leg_sync", fallback=gh0.leg_sync),
        lowstate_topic=cp.get("go2_hardware", "lowstate_topic", fallback=gh0.lowstate_topic).strip(),
        cmd_hz=cp.getfloat("go2_hardware", "cmd_hz", fallback=gh0.cmd_hz),
        vel_deadband=cp.getfloat("go2_hardware", "vel_deadband", fallback=gh0.vel_deadband),
        stop_on_zero_vel=cp.getboolean(
            "go2_hardware", "stop_on_zero_vel", fallback=gh0.stop_on_zero_vel
        ),
        vel_feedback_enable=cp.getboolean(
            "go2_hardware", "vel_feedback_enable", fallback=gh0.vel_feedback_enable
        ),
        vel_feedback_kp_vx=cp.getfloat(
            "go2_hardware", "vel_feedback_kp_vx", fallback=gh0.vel_feedback_kp_vx
        ),
        vel_feedback_kp_vy=cp.getfloat(
            "go2_hardware", "vel_feedback_kp_vy", fallback=gh0.vel_feedback_kp_vy
        ),
        vel_feedback_kp_wz=cp.getfloat(
            "go2_hardware", "vel_feedback_kp_wz", fallback=gh0.vel_feedback_kp_wz
        ),
        vel_feedback_max_vx=cp.getfloat(
            "go2_hardware", "vel_feedback_max_vx", fallback=gh0.vel_feedback_max_vx
        ),
        vel_feedback_max_vy=cp.getfloat(
            "go2_hardware", "vel_feedback_max_vy", fallback=gh0.vel_feedback_max_vy
        ),
        vel_feedback_max_wz=cp.getfloat(
            "go2_hardware", "vel_feedback_max_wz", fallback=gh0.vel_feedback_max_wz
        ),
        vel_feedback_max_corr_vx=cp.getfloat(
            "go2_hardware", "vel_feedback_max_corr_vx", fallback=gh0.vel_feedback_max_corr_vx
        ),
        vel_feedback_max_corr_vy=cp.getfloat(
            "go2_hardware", "vel_feedback_max_corr_vy", fallback=gh0.vel_feedback_max_corr_vy
        ),
        vel_feedback_max_corr_wz=cp.getfloat(
            "go2_hardware", "vel_feedback_max_corr_wz", fallback=gh0.vel_feedback_max_corr_wz
        ),
        vel_heading_hold_enable=cp.getboolean(
            "go2_hardware", "vel_heading_hold_enable", fallback=gh0.vel_heading_hold_enable
        ),
        vel_heading_hold_kp=cp.getfloat(
            "go2_hardware", "vel_heading_hold_kp", fallback=gh0.vel_heading_hold_kp
        ),
        vel_heading_hold_ki=cp.getfloat(
            "go2_hardware", "vel_heading_hold_ki", fallback=gh0.vel_heading_hold_ki
        ),
        vel_heading_hold_kd=cp.getfloat(
            "go2_hardware", "vel_heading_hold_kd", fallback=gh0.vel_heading_hold_kd
        ),
        vel_heading_hold_max_wz=cp.getfloat(
            "go2_hardware", "vel_heading_hold_max_wz", fallback=gh0.vel_heading_hold_max_wz
        ),
        vel_heading_hold_integral_max=cp.getfloat(
            "go2_hardware", "vel_heading_hold_integral_max", fallback=gh0.vel_heading_hold_integral_max
        ),
        stand_on_start=cp.get("go2_hardware", "stand_on_start", fallback=gh0.stand_on_start).strip().lower(),
        gait_on_start=cp.get("go2_hardware", "gait_on_start", fallback=gh0.gait_on_start).strip().lower(),
        shutdown_mode=cp.get("go2_hardware", "shutdown_mode", fallback=gh0.shutdown_mode).strip().lower(),
        world_frame_offset_xyz=_parse_vec3(
            cp.get("go2_hardware", "world_frame_offset_xyz", fallback=""),
            gh0.world_frame_offset_xyz,
        ),
        world_frame_yaw_deg=cp.getfloat(
            "go2_hardware", "world_frame_yaw_deg", fallback=gh0.world_frame_yaw_deg
        ),
        ros_workspace=cp.get("go2_hardware", "ros_workspace", fallback=gh0.ros_workspace).strip(),
        obstacles_avoid_request_topic=cp.get(
            "go2_hardware", "obstacles_avoid_request_topic", fallback=gh0.obstacles_avoid_request_topic
        ).strip(),
        obstacles_avoid_api_id=cp.getint(
            "go2_hardware", "obstacles_avoid_api_id", fallback=gh0.obstacles_avoid_api_id
        ),
        status_log_interval_s=cp.getfloat(
            "go2_hardware", "status_log_interval_s", fallback=gh0.status_log_interval_s
        ),
    )


def _load_gaze_stabilizer_config(cp: configparser.ConfigParser, defaults: AppConfigBundle) -> GazeStabilizerConfig:
    g0 = defaults.gaze_stabilizer_config
    if not cp.has_section("gaze_stabilizer"):
        return g0
    return GazeStabilizerConfig(
        enable_feedback=cp.getboolean("gaze_stabilizer", "gaze_enable_feedback", fallback=g0.enable_feedback),
        enable_base_ff=cp.getboolean("gaze_stabilizer", "gaze_enable_base_ff", fallback=g0.enable_base_ff),
        uv_gain=cp.getfloat(
            "gaze_stabilizer",
            "gaze_uv_gain",
            fallback=cp.getfloat("gaze_stabilizer", "gaze_k_uv_u", fallback=g0.uv_gain),
        ),
        base_ff_gain_pitch=cp.getfloat("gaze_stabilizer", "gaze_base_ff_gain_pitch", fallback=g0.base_ff_gain_pitch),
        base_ff_gain_roll=cp.getfloat("gaze_stabilizer", "gaze_base_ff_gain_roll", fallback=g0.base_ff_gain_roll),
        base_ff_gain_yaw=cp.getfloat("gaze_stabilizer", "gaze_base_ff_gain_yaw", fallback=g0.base_ff_gain_yaw),
        max_du_roll=cp.getfloat("gaze_stabilizer", "gaze_max_du_roll", fallback=g0.max_du_roll),
        max_du_s1=cp.getfloat("gaze_stabilizer", "gaze_max_du_s1", fallback=g0.max_du_s1),
        max_du_s2=cp.getfloat("gaze_stabilizer", "gaze_max_du_s2", fallback=g0.max_du_s2),
        hz=cp.getfloat("gaze_stabilizer", "gaze_hz", fallback=g0.hz),
        center_tol=cp.getfloat("gaze_stabilizer", "gaze_center_tol", fallback=g0.center_tol),
        center_u_gain=cp.getfloat("gaze_stabilizer", "gaze_center_u_gain", fallback=g0.center_u_gain),
        center_v_gain=cp.getfloat("gaze_stabilizer", "gaze_center_v_gain", fallback=g0.center_v_gain),
        center_roll_max=cp.getfloat("gaze_stabilizer", "gaze_center_roll_max", fallback=g0.center_roll_max),
        center_seg_max=cp.getfloat("gaze_stabilizer", "gaze_center_seg_max", fallback=g0.center_seg_max),
        step_scale=cp.getfloat("gaze_stabilizer", "gaze_step_scale", fallback=g0.step_scale),
        enable_roll=cp.getboolean("gaze_stabilizer", "gaze_enable_roll", fallback=g0.enable_roll),
        center_u_kd=cp.getfloat("gaze_stabilizer", "gaze_center_u_kd", fallback=g0.center_u_kd),
        center_v_kd=cp.getfloat("gaze_stabilizer", "gaze_center_v_kd", fallback=g0.center_v_kd),
        center_d_seg_max=cp.getfloat("gaze_stabilizer", "gaze_center_d_seg_max", fallback=g0.center_d_seg_max),
        d_filter_alpha=cp.getfloat("gaze_stabilizer", "gaze_d_filter_alpha", fallback=g0.d_filter_alpha),
        max_seg_du_per_tick=cp.getfloat(
            "gaze_stabilizer", "gaze_max_seg_du_per_tick", fallback=g0.max_seg_du_per_tick
        ),
        command_ref_enable=cp.getboolean(
            "gaze_stabilizer", "gaze_command_ref_enable", fallback=g0.command_ref_enable
        ),
        command_ref_max_lead=cp.getfloat(
            "gaze_stabilizer", "gaze_command_ref_max_lead", fallback=g0.command_ref_max_lead
        ),
        cmd_settle_s=cp.getfloat("gaze_stabilizer", "gaze_cmd_settle_s", fallback=g0.cmd_settle_s),
        center_u_seg_s2_scale=cp.getfloat(
            "gaze_stabilizer", "gaze_center_u_seg_s2_scale", fallback=g0.center_u_seg_s2_scale
        ),
        center_u_seg_s1_scale=cp.getfloat(
            "gaze_stabilizer", "gaze_center_u_seg_s1_scale", fallback=g0.center_u_seg_s1_scale
        ),
        fine_err_max=cp.getfloat("gaze_stabilizer", "gaze_fine_err_max", fallback=g0.fine_err_max),
        fine_settle_scale=cp.getfloat("gaze_stabilizer", "gaze_fine_settle_scale", fallback=g0.fine_settle_scale),
        fov_margin=cp.getfloat("gaze_stabilizer", "gaze_fov_margin", fallback=g0.fov_margin),
        clamp_go2_vel_on_large_error=cp.getboolean(
            "gaze_stabilizer",
            "gaze_clamp_go2_vel_on_large_error",
            fallback=g0.clamp_go2_vel_on_large_error,
        ),
        preview_enable=cp.getboolean("gaze_stabilizer", "gaze_preview_enable", fallback=g0.preview_enable),
        preview_tau_s=cp.getfloat("gaze_stabilizer", "gaze_preview_tau_s", fallback=g0.preview_tau_s),
        preview_b_pitch=cp.getfloat("gaze_stabilizer", "gaze_preview_b_pitch", fallback=g0.preview_b_pitch),
        preview_q_u=cp.getfloat("gaze_stabilizer", "gaze_preview_q_u", fallback=g0.preview_q_u),
        preview_q_v=cp.getfloat("gaze_stabilizer", "gaze_preview_q_v", fallback=g0.preview_q_v),
        preview_r_roll=cp.getfloat("gaze_stabilizer", "gaze_preview_r_roll", fallback=g0.preview_r_roll),
        preview_r_s1=cp.getfloat("gaze_stabilizer", "gaze_preview_r_s1", fallback=g0.preview_r_s1),
        preview_r_s2=cp.getfloat("gaze_stabilizer", "gaze_preview_r_s2", fallback=g0.preview_r_s2),
        preview_max_du_roll=cp.getfloat(
            "gaze_stabilizer", "gaze_preview_max_du_roll", fallback=g0.preview_max_du_roll
        ),
        preview_max_du_seg=cp.getfloat(
            "gaze_stabilizer",
            "gaze_preview_max_du_seg",
            fallback=g0.preview_max_du_seg,
        ),
        preview_lowpass_alpha=cp.getfloat(
            "gaze_stabilizer", "gaze_preview_lowpass_alpha", fallback=g0.preview_lowpass_alpha
        ),
        walking_gaze_mode=cp.get("gaze_stabilizer", "gaze_walking_mode", fallback=g0.walking_gaze_mode),
    )


def _load_experiment_config(cp: configparser.ConfigParser, defaults: AppConfigBundle) -> ExperimentConfig:
    e0 = defaults.experiment_config
    if not cp.has_section("experiment"):
        return e0
    return ExperimentConfig(
        ownership_enable=cp.getboolean("experiment", "ownership_enable", fallback=e0.ownership_enable),
        preview_fallback_uv_ff=cp.getboolean(
            "experiment", "preview_fallback_uv_ff", fallback=e0.preview_fallback_uv_ff
        ),
    )


def _default_app_config_bundle() -> AppConfigBundle:
    return AppConfigBundle(
        sim_param=SimParam(),
        sim_config=SimConfig(),
        hardware_config=HardwareConfig(),
        joint_limit=JointLimit(roll_min_deg=-90.0, roll_max_deg=90.0, bend_deg=36.0),
        spawn_config=SpawnConfig(),
        urdf_export_config=UrdfExportConfig(),
        ik_config=IkConfig(),
        perception_config=PerceptionConfig(),
        pick_config=PickConfig(),
        go2_locomotion_config=Go2LocomotionConfig(),
        go2_hardware_config=Go2HardwareConfig(),
        gaze_stabilizer_config=GazeStabilizerConfig(),
        experiment_config=ExperimentConfig(),
        mapping_config=proto.SimMappingConfig(),
    )


def _load_sim_param_config(cp: configparser.ConfigParser, defaults: AppConfigBundle) -> SimParam:
    sp0 = defaults.sim_param
    return SimParam(
        dt=cp.getfloat("SimParam", "dt", fallback=sp0.dt),
        substeps=cp.getint("SimParam", "substeps", fallback=sp0.substeps),
        realtime=cp.getboolean("SimParam", "realtime", fallback=sp0.realtime),
        realtime_factor=cp.getfloat("SimParam", "realtime_factor", fallback=sp0.realtime_factor),
        gravity=_parse_vec3(cp.get("SimParam", "gravity", fallback=""), sp0.gravity),
        roll_rate=cp.getfloat("SimParam", "roll_rate", fallback=sp0.roll_rate),
        bend_rate=cp.getfloat("SimParam", "bend_rate", fallback=sp0.bend_rate),
        zmq_hwm=cp.getint("SimParam", "zmq_hwm", fallback=sp0.zmq_hwm),
    )


def _load_sim_config(cp: configparser.ConfigParser, defaults: AppConfigBundle, *, config_dir: str) -> SimConfig:
    sc0 = defaults.sim_config
    build_dir = os.path.abspath(os.path.join(config_dir, "crafts"))
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
        arm_urdf_name=cp.get("runtime", "arm_urdf_name", fallback=sc0.arm_urdf_name),
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
        traj_enable=cp.getboolean("runtime", "traj_enable", fallback=sc0.traj_enable),
        traj_duration_s=cp.getfloat("runtime", "traj_duration_s", fallback=sc0.traj_duration_s),
        traj_min_s=cp.getfloat("runtime", "traj_min_s", fallback=sc0.traj_min_s),
        traj_max_s=cp.getfloat("runtime", "traj_max_s", fallback=sc0.traj_max_s),
        traj_linear_scale_m=cp.getfloat("runtime", "traj_linear_scale_m", fallback=sc0.traj_linear_scale_m),
        traj_angular_scale_rad=cp.getfloat(
            "runtime", "traj_angular_scale_rad", fallback=sc0.traj_angular_scale_rad
        ),
        traj_lji_enable=cp.getboolean(
            "runtime", "traj_lji_enable", fallback=sc0.traj_lji_enable
        ),
        traj_lji_duration_s=cp.getfloat(
            "runtime", "traj_lji_duration_s", fallback=sc0.traj_lji_duration_s
        ),
        traj_lji_min_s=cp.getfloat(
            "runtime", "traj_lji_min_s", fallback=sc0.traj_lji_min_s
        ),
        traj_lji_max_s=cp.getfloat(
            "runtime", "traj_lji_max_s", fallback=sc0.traj_lji_max_s
        ),
        sim_camera_enable=cp.getboolean("runtime", "sim_camera_enable", fallback=sc0.sim_camera_enable),
        sim_camera_port=cp.get("runtime", "sim_camera_port", fallback=sc0.sim_camera_port),
        sim_camera_jpeg=cp.getboolean("runtime", "sim_camera_jpeg", fallback=sc0.sim_camera_jpeg),
        sim_camera_jpeg_quality=cp.getint("runtime", "sim_camera_jpeg_quality", fallback=sc0.sim_camera_jpeg_quality),
        sim_camera_rgb=cp.getboolean("runtime", "sim_camera_rgb", fallback=sc0.sim_camera_rgb),
        sim_camera_depth=cp.getboolean("runtime", "sim_camera_depth", fallback=sc0.sim_camera_depth),
        sim_camera_auto_disable_unused=cp.getboolean(
            "runtime",
            "sim_camera_auto_disable_unused",
            fallback=sc0.sim_camera_auto_disable_unused,
        ),
        sim_camera_max_hz=cp.getfloat("runtime", "sim_camera_max_hz", fallback=sc0.sim_camera_max_hz),
        sim_camera_width=cp.getint("runtime", "sim_camera_width", fallback=sc0.sim_camera_width),
        sim_camera_height=cp.getint("runtime", "sim_camera_height", fallback=sc0.sim_camera_height),
        sim_camera_fov_deg=cp.getfloat("runtime", "sim_camera_fov_deg", fallback=sc0.sim_camera_fov_deg),
        sim_side_camera_enable=cp.getboolean(
            "runtime",
            "sim_side_camera_enable",
            fallback=sc0.sim_side_camera_enable,
        ),
        sim_side_camera_port=cp.get(
            "runtime",
            "sim_side_camera_port",
            fallback=sc0.sim_side_camera_port,
        ),
        sim_side_camera_jpeg=cp.getboolean(
            "runtime",
            "sim_side_camera_jpeg",
            fallback=sc0.sim_side_camera_jpeg,
        ),
        sim_side_camera_jpeg_quality=cp.getint(
            "runtime",
            "sim_side_camera_jpeg_quality",
            fallback=sc0.sim_side_camera_jpeg_quality,
        ),
        sim_side_camera_max_hz=cp.getfloat(
            "runtime",
            "sim_side_camera_max_hz",
            fallback=sc0.sim_side_camera_max_hz,
        ),
        sim_side_camera_record_fps=cp.getfloat(
            "runtime",
            "sim_side_camera_record_fps",
            fallback=sc0.sim_side_camera_record_fps,
        ),
        sim_side_camera_width=cp.getint(
            "runtime",
            "sim_side_camera_width",
            fallback=sc0.sim_side_camera_width,
        ),
        sim_side_camera_height=cp.getint(
            "runtime",
            "sim_side_camera_height",
            fallback=sc0.sim_side_camera_height,
        ),
        sim_side_camera_fov_deg=cp.getfloat(
            "runtime",
            "sim_side_camera_fov_deg",
            fallback=sc0.sim_side_camera_fov_deg,
        ),
        sim_side_camera_pos=_parse_vec3(
            cp.get("runtime", "sim_side_camera_pos", fallback=""),
            sc0.sim_side_camera_pos,
        ),
        sim_side_camera_lookat=_parse_vec3(
            cp.get("runtime", "sim_side_camera_lookat", fallback=""),
            sc0.sim_side_camera_lookat,
        ),
        perf_log_enable=cp.getboolean("runtime", "perf_log_enable", fallback=sc0.perf_log_enable),
        perf_log_interval_s=cp.getfloat("runtime", "perf_log_interval_s", fallback=sc0.perf_log_interval_s),
        perf_log_path=cp.get("runtime", "perf_log_path", fallback=sc0.perf_log_path),
    )


def _load_hardware_config(cp: configparser.ConfigParser) -> HardwareConfig:
    if cp.has_option("hardware", "dxl_dir_1") or cp.has_option("hardware", "dxl_dir_2") or cp.has_option("hardware", "dxl_dir_3") or cp.has_option("hardware", "dxl_dir_4"):
        raise ValueError("legacy hardware keys dxl_dir_1..4 are no longer supported; use command_direction and motor_direction")
    if not cp.has_option("hardware", "command_direction"):
        raise ValueError("missing required hardware.command_direction")
    if not cp.has_option("hardware", "motor_direction"):
        raise ValueError("missing required hardware.motor_direction")
    hw0 = HardwareConfig()
    return HardwareConfig(
        command_direction=_parse_direction4(cp.get("hardware", "command_direction"), key="hardware.command_direction"),
        motor_direction=_parse_direction4(cp.get("hardware", "motor_direction"), key="hardware.motor_direction"),
        baudrate=cp.getint("hardware", "baudrate", fallback=hw0.baudrate),
        linear_u_limit_deg=cp.getfloat("hardware", "linear_u_limit_deg", fallback=hw0.linear_u_limit_deg),
        current_yellow_ma=cp.getint("hardware", "current_yellow_ma", fallback=hw0.current_yellow_ma),
        current_limit_ma=cp.getint("hardware", "current_limit_ma", fallback=hw0.current_limit_ma),
        host_hw_read_hz=cp.getfloat("hardware", "host_hw_read_hz", fallback=hw0.host_hw_read_hz),
        host_hw_cmd_hz=cp.getfloat("hardware", "host_hw_cmd_hz", fallback=hw0.host_hw_cmd_hz),
        current_read_hz=cp.getfloat("hardware", "current_read_hz", fallback=hw0.current_read_hz),
        profile_vel_linear=cp.getint("hardware", "profile_vel_linear", fallback=hw0.profile_vel_linear),
        profile_acc_linear=cp.getint("hardware", "profile_acc_linear", fallback=hw0.profile_acc_linear),
        profile_vel_roll=cp.getint("hardware", "profile_vel_roll", fallback=hw0.profile_vel_roll),
        profile_acc_roll=cp.getint("hardware", "profile_acc_roll", fallback=hw0.profile_acc_roll),
        profile_vel_seg1=cp.getint("hardware", "profile_vel_seg1", fallback=hw0.profile_vel_seg1),
        profile_acc_seg1=cp.getint("hardware", "profile_acc_seg1", fallback=hw0.profile_acc_seg1),
        profile_vel_seg2=cp.getint("hardware", "profile_vel_seg2", fallback=hw0.profile_vel_seg2),
        profile_acc_seg2=cp.getint("hardware", "profile_acc_seg2", fallback=hw0.profile_acc_seg2),
        profile_vel_claw=cp.getint("hardware", "profile_vel_claw", fallback=hw0.profile_vel_claw),
        profile_acc_claw=cp.getint("hardware", "profile_acc_claw", fallback=hw0.profile_acc_claw),
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
        go2_spawn_height=cp.getfloat("spawn", "go2_spawn_height", fallback=am0.go2_spawn_height),
        go2_mount_offset_m=_parse_vec3(cp.get("spawn", "go2_mount_offset_m", fallback=""), am0.go2_mount_offset_m),
        go2_spawn_euler_deg=_parse_vec3(cp.get("spawn", "go2_spawn_euler_deg", fallback=""), am0.go2_spawn_euler_deg),
        go2_teleop_vx_mps=cp.getfloat("spawn", "go2_teleop_vx_mps", fallback=am0.go2_teleop_vx_mps),
        go2_teleop_vy_mps=cp.getfloat("spawn", "go2_teleop_vy_mps", fallback=am0.go2_teleop_vy_mps),
        go2_teleop_wz_radps=cp.getfloat("spawn", "go2_teleop_wz_radps", fallback=am0.go2_teleop_wz_radps),
        sim_target_enable=cp.getboolean("spawn", "sim_target_enable", fallback=am0.sim_target_enable),
        sim_target_xyz=_parse_vec3(cp.get("spawn", "sim_target_xyz", fallback=""), am0.sim_target_xyz),
        sim_target_radius=cp.getfloat("spawn", "sim_target_radius", fallback=am0.sim_target_radius),
        sim_target_color_rgba=_parse_color_rgba(
            cp.get("spawn", "sim_target_color_rgba", fallback=""),
            am0.sim_target_color_rgba,
        ),
        sim_target_collision=cp.getboolean("spawn", "sim_target_collision", fallback=am0.sim_target_collision),
        sim_target_gravity=cp.getboolean("spawn", "sim_target_gravity", fallback=am0.sim_target_gravity),
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


def _build_mapping_config(joint_limit: JointLimit, hardware_config: HardwareConfig) -> proto.SimMappingConfig:
    return proto.SimMappingConfig(
        linear_u_limit=float(hardware_config.linear_u_limit_deg),
        linear_q_min_m=-0.230,
        linear_q_max_m=0.0,
        roll_q_min_rad=joint_limit.roll_min_rad(),
        roll_q_max_rad=joint_limit.roll_max_rad(),
        seg1_q_min_rad=-joint_limit.bend_lim_rad(),
        seg1_q_max_rad=+joint_limit.bend_lim_rad(),
        seg2_q_min_rad=-joint_limit.bend_lim_rad(),
        seg2_q_max_rad=+joint_limit.bend_lim_rad(),
        command_direction=hardware_config.command_direction,
    )


def _read_config_with_extends(path: str) -> tuple[configparser.ConfigParser, str]:
    root_path = os.path.abspath(path)
    seen: set[str] = set()

    def collect(current: str) -> list[str]:
        current_abs = os.path.abspath(current)
        if current_abs in seen:
            raise ValueError(f"config extends cycle detected at {current_abs}")
        if not os.path.isfile(current_abs):
            raise FileNotFoundError(f"config file not found: {current_abs}")
        seen.add(current_abs)

        probe = configparser.ConfigParser()
        probe.optionxform = str
        probe.read(current_abs, encoding="utf-8-sig")
        parent_raw = probe.get("config", "extends", fallback="").strip()
        paths: list[str] = []
        if parent_raw:
            parent = parent_raw if os.path.isabs(parent_raw) else os.path.join(os.path.dirname(current_abs), parent_raw)
            paths.extend(collect(parent))
        paths.append(current_abs)
        return paths

    paths = collect(root_path)
    cp = configparser.ConfigParser()
    cp.optionxform = str
    cp.read(paths, encoding="utf-8-sig")
    return cp, os.path.dirname(root_path)


def load_app_config_from_ini(path: str) -> AppConfigBundle:
    defaults = _default_app_config_bundle()
    if not path:
        raise FileNotFoundError("config path is empty")
    if not os.path.isfile(path):
        raise FileNotFoundError(f"config file not found: {path}")

    cp, config_dir = _read_config_with_extends(path)
    sim_param_cfg = _load_sim_param_config(cp, defaults)
    sim_config_cfg = _load_sim_config(cp, defaults, config_dir=config_dir)
    hardware_config_cfg = _load_hardware_config(cp)
    joint_limit_cfg = _load_joint_limit(cp, defaults)
    spawn_config_cfg = _load_spawn_config(cp, defaults)
    urdf_export_config_cfg = _load_urdf_export_config(cp, defaults)
    ik_config_cfg = _load_ik_config(cp, defaults)
    perception_config_cfg = _load_perception_config(cp, defaults)
    pick_config_cfg = _load_pick_config(cp, defaults)
    go2_locomotion_config_cfg = _load_go2_locomotion_config(cp, defaults)
    go2_hardware_config_cfg = _load_go2_hardware_config(cp, defaults)
    gaze_stabilizer_config_cfg = _load_gaze_stabilizer_config(cp, defaults)
    experiment_config_cfg = _load_experiment_config(cp, defaults)
    mapping_config_cfg = _build_mapping_config(joint_limit_cfg, hardware_config_cfg)

    return AppConfigBundle(
        sim_param=sim_param_cfg,
        sim_config=sim_config_cfg,
        hardware_config=hardware_config_cfg,
        joint_limit=joint_limit_cfg,
        spawn_config=spawn_config_cfg,
        urdf_export_config=urdf_export_config_cfg,
        ik_config=ik_config_cfg,
        perception_config=perception_config_cfg,
        pick_config=pick_config_cfg,
        go2_locomotion_config=go2_locomotion_config_cfg,
        go2_hardware_config=go2_hardware_config_cfg,
        gaze_stabilizer_config=gaze_stabilizer_config_cfg,
        experiment_config=experiment_config_cfg,
        mapping_config=mapping_config_cfg,
    )
