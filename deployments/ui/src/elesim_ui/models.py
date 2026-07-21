from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class HardwareConfig:
    current_yellow_ma: int = 1800
    current_limit_ma: int = 2500


@dataclass(frozen=True)
class PerceptionConfig:
    enabled: bool = True
    detector_config: str = ""
    mode: str = "external"
    detector: str = "external"
    provider: str = "local"
    preview_bind: str = "tcp://127.0.0.1:5570"
    preview_endpoint: str = "tcp://127.0.0.1:5570"
    preview_jpeg_quality: int = 75
    target_label: str = "sports ball"
    yolo_device: str = ""
    publish_hz: float = 15.0
    show_preview: bool = True
    pipeline: str = "yolo_seg"
    tracker: str = "csrt"
    run_local: bool = True


@dataclass(frozen=True)
class PickConfig:
    target_scale: float = 0.16
    scale_tol: float = 0.02
    center_tol: float = 0.12
    target_uv_u: float = 0.5
    target_uv_v: float = 0.0
    ready_pose_standoff_m: float = 0.20
    look_pose_standoff_m: float = 0.20
    mobile_handoff_distance_m: float = 0.30


@dataclass(frozen=True)
class GazeStabilizerConfig:
    enable_feedback: bool = True
    enable_base_ff: bool = False
    uv_gain: float = 1.0
    base_ff_gain_pitch: float = 0.0
    base_ff_gain_roll: float = 0.0
    base_ff_gain_yaw: float = 0.0
    max_du_roll: float = 1.0
    max_du_s1: float = 1.0
    max_du_s2: float = 1.0
    jacobian_damping: float = 0.03
    hz: float = 20.0
    center_tol: float = 0.06
    center_u_gain: float = 18.0
    center_v_gain: float = 18.0
    center_roll_max: float = 8.0
    center_seg_max: float = 8.0
    step_scale: float = 1.0
    enable_roll: bool = False
    walking_gaze_mode: str = "uv_ff"
    preview_enable: bool = False


@dataclass(frozen=True)
class PanelStateDefaults:
    """Offline-safe values rendered before the first controller snapshot."""

    linear: float = 0.0
    roll: float = 0.0
    theta1: float = 0.0
    theta2: float = 0.0
    u_offset_linear: float = 0.0
    u_offset_roll: float = 0.0
    u_offset_s1: float = 0.0
    u_offset_s2: float = 0.0
    offset_revision: int = 0
    paused: bool = False
    claw_closed: bool = False
    torque_lock_bypass: bool = False

    visual_target_scale: float = 0.16
    visual_center_tol: float = 0.12
    visual_target_uv_u: float = 0.5
    visual_target_uv_v: float = 0.0
    visual_scale_tol: float = 0.01
    visual_confidence_min: float = 0.0
    visual_target_label: str = ""
    visual_ready_distance_m: float = 0.20
    visual_look_distance_m: float = 0.20

    perception_running: bool = False
    perception_failed: bool = False
    perception_status_msg: str = ""
    perception_frame_idx: int = 0
    perception_label: str = ""
    perception_confidence: float = 0.0
    perception_camera_xyz: tuple[float, float, float] | None = None
    perception_world_xyz: tuple[float, float, float] | None = None
    perception_tracker_phase: str = "search"
    perception_track_ok_frames: int = 0
    perception_image_scale: float = 0.0
    perception_bbox_wh: tuple[int, int] = (0, 0)
    perception_tracker_backend: str = ""
    perception_last_capture_path: str = ""
    perception_recording: bool = False
    perception_record_with_overlay: bool = False
    perception_last_record_path: str = ""
    perception_center_uv: tuple[float, float] | None = None
    perception_last_update_s: float = 0.0
    perception_hz: float = 0.0

    gaze_running: bool = False
    gaze_mode: str = "idle"
    gaze_status_msg: str = ""
    gaze_u_err: float = 0.0
    gaze_v_err: float = 0.0
    gaze_du_roll: float = 0.0
    gaze_du_s1: float = 0.0
    gaze_du_s2: float = 0.0
    gaze_tick_count: int = 0
    gaze_update_count: int = 0
    gaze_obs_age_s: float = -1.0

    mock_object_x: float = 0.5
    mock_object_y: float = 0.0
    mock_object_z: float = 1.2
    mock_object_dir_x: float = 1.0
    mock_object_dir_y: float = 0.0
    mock_object_dir_z: float = 0.0
    pick_running: bool = False
    pick_failed: bool = False
    pick_phase: str = "idle"
    pick_status_msg: str = ""

    target_x: float = 0.5
    target_y: float = 0.0
    target_z: float = 1.0
    target_vx: float = 1.0
    target_vy: float = 0.0
    target_vz: float = 0.0
    sag_model_path: str = ""
    raw_sag_model: dict[str, Any] | None = None

    ik_running: bool = False
    ik_converged: bool = False
    ik_failed: bool = False
    ik_err_m: float = 0.0
    ik_status_msg: str = ""
    ik_sim_tip_err_m: float = 0.0
    ik_track_roll_err_rad: float = 0.0
    ik_track_theta1_err_rad: float = 0.0
    ik_track_theta2_err_rad: float = 0.0
    ik_track_bend_max_err_rad: float = 0.0
    ik_sol_roll: float = 0.0
    ik_sol_theta1: float = 0.0
    ik_sol_theta2: float = 0.0


def gaze_config_to_dict(config: Any) -> dict[str, Any]:
    if dataclasses.is_dataclass(config):
        return {field.name: getattr(config, field.name) for field in dataclasses.fields(config)}
    return {
        str(key): value
        for key, value in vars(config).items()
        if not str(key).startswith("_")
    }


ControlService = Any
HostState = Any
PanelState = Any
