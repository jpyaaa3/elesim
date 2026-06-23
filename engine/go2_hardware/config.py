from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class Go2HardwareConfig:
    enabled: bool = False
    backend: str = "unitree_ros2"
    sport_request_topic: str = "/api/sport/request"
    # pose_source: odom (nav_msgs/Odometry) | sportmodestate (unitree_go SportModeState)
    pose_source: str = "odom"
    odom_topic: str = "/odom"
    sport_state_topic: str = "/sportmodestate"
    leg_sync: bool = True
    lowstate_topic: str = "/lowstate"
    cmd_hz: float = 20.0
    vel_deadband: float = 0.02
    stop_on_zero_vel: bool = True
    # Closed-loop Sport Move using pose topic body velocity (/sportmodestate or /odom).
    vel_feedback_enable: bool = True
    vel_feedback_kp_vx: float = 0.8
    vel_feedback_kp_vy: float = 0.8
    vel_feedback_kp_wz: float = 1.0
    vel_feedback_max_vx: float = 0.6
    vel_feedback_max_vy: float = 0.6
    vel_feedback_max_wz: float = 1.0
    stand_on_start: str = "balance"
    # Gait mode applied after stand_on_start: none | static_walk | trot_run | economic_gait
    gait_on_start: str = "static_walk"
    shutdown_mode: str = "damp"
    world_frame_offset_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    world_frame_yaw_deg: float = 0.0
    # Path to unitree_ros2 workspace root (optional; also UNITREE_ROS2_WS env).
    ros_workspace: str = ""
    obstacles_avoid_request_topic: str = "/api/obstacles_avoid/request"
    obstacles_avoid_api_id: int = 1001
    # Periodic ROS2 link diagnostics on Jetson (0 = disabled).
    status_log_interval_s: float = 2.0

    def is_active(self, *, use_go2: bool) -> bool:
        return bool(self.enabled) and bool(use_go2) and str(self.backend).strip().lower() == "unitree_ros2"
