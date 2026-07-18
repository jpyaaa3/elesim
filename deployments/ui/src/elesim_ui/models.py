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
