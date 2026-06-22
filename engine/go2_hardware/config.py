from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class Go2HardwareConfig:
    enabled: bool = False
    backend: str = "unitree_ros2"
    sport_request_topic: str = "api/sport/request"
    odom_topic: str = "/odom"
    cmd_hz: float = 20.0
    vel_deadband: float = 0.02
    stop_on_zero_vel: bool = True
    stand_on_start: str = "balance"
    shutdown_mode: str = "damp"
    world_frame_offset_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    world_frame_yaw_deg: float = 0.0

    def is_active(self, *, use_go2: bool) -> bool:
        return bool(self.enabled) and bool(use_go2) and str(self.backend).strip().lower() == "unitree_ros2"
