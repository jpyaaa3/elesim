from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any

import yaml

from elesim_protocol import SimMappingConfig
from elesim_robot.go2.config import Go2HardwareConfig


@dataclass(frozen=True)
class HardwareConfig:
    command_direction: tuple[int, int, int, int] = (1, -1, 1, -1)
    motor_direction: tuple[int, int, int, int] = (1, -1, 1, -1)
    baudrate: int = 1_000_000
    linear_u_max_deg: float = 250.0
    linear_u_limit_deg: float = 250.0
    current_limit_ma: int = 2500
    profile_vel_linear: int = 240
    profile_acc_linear: int = 10
    profile_vel_roll: int = 240
    profile_acc_roll: int = 10
    profile_vel_seg1: int = 150
    profile_acc_seg1: int = 20
    profile_vel_seg2: int = 150
    profile_acc_seg2: int = 20
    profile_vel_claw: int = 80
    profile_acc_claw: int = 5


@dataclass(frozen=True)
class CameraConfig:
    enabled: bool = True
    bind: str = "tcp://0.0.0.0:5568"
    advertise: str = ""
    width: int = 640
    height: int = 480
    fps: int = 30


@dataclass(frozen=True)
class RobotConfig:
    endpoint_id: str
    server_endpoint: str
    device: str
    use_go2: bool
    arm: HardwareConfig
    mapping: SimMappingConfig
    go2: Go2HardwareConfig
    camera: CameraConfig


def _values(cls: type, raw: object) -> dict[str, Any]:
    source = raw if isinstance(raw, dict) else {}
    allowed = {item.name for item in fields(cls)}
    result = {key: value for key, value in source.items() if key in allowed}
    for key, value in tuple(result.items()):
        if key in {"command_direction", "motor_direction", "world_frame_offset_xyz"}:
            result[key] = tuple(value)
    return result


def load_config(path: str | Path) -> RobotConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict):
        raise ValueError("robot config root must be a mapping")
    runtime = raw.get("runtime", {})
    if not isinstance(runtime, dict):
        raise ValueError("runtime config must be a mapping")
    arm = HardwareConfig(**_values(HardwareConfig, raw.get("arm")))
    mapping_values = _values(SimMappingConfig, raw.get("mapping"))
    mapping_values.setdefault("linear_u_max", arm.linear_u_max_deg)
    mapping_values.setdefault("linear_u_limit", arm.linear_u_limit_deg)
    mapping_values.setdefault("command_direction", arm.command_direction)
    return RobotConfig(
        endpoint_id=str(runtime.get("endpoint_id", "robot-go2")),
        server_endpoint=str(runtime.get("server_endpoint", "tcp://127.0.0.1:5558")),
        device=str(runtime.get("device", "")),
        use_go2=bool(runtime.get("use_go2", True)),
        arm=arm,
        mapping=SimMappingConfig(**mapping_values),
        go2=Go2HardwareConfig(**_values(Go2HardwareConfig, raw.get("go2"))),
        camera=CameraConfig(**_values(CameraConfig, raw.get("camera"))),
    )
