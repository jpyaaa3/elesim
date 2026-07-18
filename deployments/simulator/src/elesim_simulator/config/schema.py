#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import configparser
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple
import elesim_protocol.messages as proto
from elesim_simulator.robot.go2.hardware.config import Go2HardwareConfig
from elesim_simulator.robot.go2.locomotion.config import Go2LocomotionConfig
from elesim_simulator.gaze.stabilizer import GazeStabilizerConfig
from elesim_simulator.robot.arm.joint_defs import JointLimit


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
    assy_build_json: str = "blueprint.json"
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
    u_offset_linear: float = 0.0
    u_offset_roll: float = 0.0
    u_offset_s1: float = 0.0
    u_offset_s2: float = 0.0
    baudrate: int = 57600
    linear_u_max_deg: float = 250.0
    linear_u_limit_deg: float = 250.0
    current_yellow_ma: int = 1800
    current_limit_ma: int = 2500
    host_hw_read_hz: float = 20.0
    host_hw_cmd_hz: float = 30.0
    current_read_hz: float = 20.0
    arm_servo_thread_enable: bool = False
    arm_servo_thread_hz: float = 120.0
    arm_latency_log_enable: bool = False
    arm_latency_log_interval_s: float = 1.0
    arm_latency_log_path: str = ""
    lji_velocity_mode_enable: bool = False
    lji_velocity_max_deg_s: float = 70.0
    lji_velocity_max_linear_deg_s: float = 30.0
    lji_velocity_max_roll_deg_s: float = 70.0
    lji_velocity_max_s1_deg_s: float = 60.0
    lji_velocity_max_s2_deg_s: float = 60.0
    lji_velocity_min_linear_deg_s: float = 0.0
    lji_velocity_min_roll_deg_s: float = 0.0
    lji_velocity_min_s1_deg_s: float = 0.0
    lji_velocity_min_s2_deg_s: float = 0.0
    lji_velocity_accel_limit_deg_s2: float = 350.0
    lji_velocity_hold_s: float = 0.20
    lji_velocity_hold_max_s: float = 1.20
    lji_velocity_deadman_s: float = 3.00
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
    lij_max_dq_roll: float = 0.006
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
    lij_command_horizon: float = 1.0
    lij_condition_max: float = 100.0
    lij_probing_enabled: bool = False
    lij_probing_epsilon_linear: float = 0.001
    lij_probing_epsilon_angle: float = 0.01
    lij_uv_align_tol: float = 0.04
    lij_uv_priority_err: float = 0.35
    lij_uv_priority_z_scale: float = 0.15
    lij_uv_priority_cap_scale: float = 1.0
    lij_approach_bias_uv_gate: float = 0.25
    lij_measured_v_row_blend: float = 0.0
    lij_measured_v_row_norm_max: float = 120.0
    lij_approach_bias_gain: float = 0.3
    lij_approach_seed_mode: str = "config"
    lij_approach_seed_q_delta: Tuple[float, float, float, float] = (0.0, 0.0, 0.01, 0.01)
    lij_approach_seed_travel_m: float = 0.003
    lij_sample_min_dq_norm: float = 0.0005
    lij_sample_cmd_meas_cos_min: float = -1.0
    lij_sample_meas_cmd_ratio_min: float = 0.0
    lij_sample_meas_cmd_ratio_max: float = 0.0
    lij_bad_motion_reacquire_steps: int = 0
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
    lij_obs_max_camera_z_m: float = 0.0
    lij_obs_camera_jump_m: float = 0.0
    lij_obs_remain_jump_m: float = 0.0

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
