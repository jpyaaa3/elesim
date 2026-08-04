"""Strict configuration schema for the physical robot endpoint."""

from __future__ import annotations

import math
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any, Mapping

import yaml

from elesim_protocol import DdsRuntimeSettings, SimMappingConfig
from elesim_robot.go2.config import Go2HardwareConfig


SCHEMA_VERSION = 4


@dataclass(frozen=True)
class HardwareConfig:
    command_direction: tuple[int, int, int, int] = (1, 1, 1, -1)
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
    topic: str = "/elesim/robot_go2/rgbd/frame"
    width: int = 640
    height: int = 480
    fps: int = 30


@dataclass(frozen=True)
class SafetyConfig:
    command_deadman_s: float = 0.5
    monitor_period_s: float = 0.05
    read_failure_limit: int = 3
    telemetry_period_s: float = 0.1
    max_go2_vx_m_s: float = 0.8
    max_go2_vy_m_s: float = 0.5
    max_go2_wz_rad_s: float = 1.5


@dataclass(frozen=True)
class RobotConfig:
    endpoint_id: str
    dds: DdsRuntimeSettings
    device: str
    use_go2: bool
    arm: HardwareConfig
    mapping: SimMappingConfig
    go2: Go2HardwareConfig
    camera: CameraConfig
    safety: SafetyConfig


def _mapping(raw: object, *, context: str) -> dict[str, Any]:
    if raw is None:
        return {}
    if not isinstance(raw, Mapping):
        raise ValueError(f"{context} config must be a mapping")
    return {str(key): value for key, value in raw.items()}


def _values(cls: type, raw: object, *, context: str) -> dict[str, Any]:
    source = _mapping(raw, context=context)
    allowed = {item.name for item in fields(cls)}
    unknown = sorted(set(source) - allowed)
    if unknown:
        raise ValueError(f"unknown {context} config keys: {', '.join(unknown)}")
    result = dict(source)
    tuple_fields = {"command_direction", "motor_direction", "world_frame_offset_xyz"}
    for key in tuple_fields.intersection(result):
        value = result[key]
        if not isinstance(value, (list, tuple)):
            raise ValueError(f"{context}.{key} must be a list")
        result[key] = tuple(value)
    return result


def _finite(value: object, *, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


def _positive(value: object, *, name: str, allow_zero: bool = False) -> float:
    number = _finite(value, name=name)
    invalid = number < 0.0 if allow_zero else number <= 0.0
    if invalid:
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{name} must be {qualifier}")
    return number


def _validate_direction(value: tuple[int, ...], *, name: str) -> None:
    if len(value) != 4 or any(isinstance(item, bool) or int(item) not in {-1, 1} for item in value):
        raise ValueError(f"{name} must contain four values, each -1 or 1")


def _validate_arm(config: HardwareConfig) -> None:
    _validate_direction(config.command_direction, name="arm.command_direction")
    _validate_direction(config.motor_direction, name="arm.motor_direction")
    _positive(config.baudrate, name="arm.baudrate")
    _positive(config.linear_u_max_deg, name="arm.linear_u_max_deg")
    _positive(config.linear_u_limit_deg, name="arm.linear_u_limit_deg")
    if float(config.linear_u_limit_deg) > float(config.linear_u_max_deg):
        raise ValueError("arm.linear_u_limit_deg must not exceed linear_u_max_deg")
    _positive(config.current_limit_ma, name="arm.current_limit_ma")
    for field in fields(config):
        if field.name.startswith("profile_"):
            _positive(getattr(config, field.name), name=f"arm.{field.name}")


def _validate_mapping(config: SimMappingConfig) -> None:
    _validate_direction(tuple(config.command_direction), name="mapping.command_direction")
    bounds = (
        ("linear_u", config.linear_u_min, config.linear_u_max),
        ("roll_u", config.roll_u_min, config.roll_u_max),
        ("seg_u", config.seg_u_min, config.seg_u_max),
        ("linear_q", config.linear_q_min_m, config.linear_q_max_m),
        ("roll_q", config.roll_q_min_rad, config.roll_q_max_rad),
        ("seg1_q", config.seg1_q_min_rad, config.seg1_q_max_rad),
        ("seg2_q", config.seg2_q_min_rad, config.seg2_q_max_rad),
    )
    for name, lower, upper in bounds:
        if _finite(lower, name=f"mapping.{name}_min") >= _finite(upper, name=f"mapping.{name}_max"):
            raise ValueError(f"mapping.{name} minimum must be less than maximum")
    limit = _finite(config.linear_u_limit, name="mapping.linear_u_limit")
    if not float(config.linear_u_min) <= limit <= float(config.linear_u_max):
        raise ValueError("mapping.linear_u_limit must lie inside linear_u bounds")


def _validate_go2(
    config: Go2HardwareConfig,
    *,
    dds: DdsRuntimeSettings,
    active: bool,
    safety: SafetyConfig,
) -> None:
    if str(config.backend).strip().lower() != "unitree_ros2":
        raise ValueError("go2.backend must be 'unitree_ros2'")
    if str(config.pose_source).strip().lower() not in {"odom", "sportmodestate"}:
        raise ValueError("go2.pose_source must be 'odom' or 'sportmodestate'")
    _positive(config.cmd_hz, name="go2.cmd_hz")
    _positive(config.vel_deadband, name="go2.vel_deadband", allow_zero=True)
    if len(tuple(config.world_frame_offset_xyz)) != 3:
        raise ValueError("go2.world_frame_offset_xyz must contain three values")
    for value in config.world_frame_offset_xyz:
        _finite(value, name="go2.world_frame_offset_xyz")
    _finite(config.world_frame_yaw_deg, name="go2.world_frame_yaw_deg")
    if config.ros_domain_id is not None:
        if (
            isinstance(config.ros_domain_id, bool)
            or int(config.ros_domain_id) < 0
            or int(config.ros_domain_id) > 232
        ):
            raise ValueError("go2.ros_domain_id must be in 0..232")
    socket_path = str(config.ipc_socket_path).strip()
    if not socket_path or not Path(socket_path).is_absolute():
        raise ValueError("go2.ipc_socket_path must be an absolute path")
    if len(socket_path.encode("utf-8")) > 100:
        raise ValueError("go2.ipc_socket_path is too long for a Unix socket")
    ipc_users: list[str] = []
    for name in ("ipc_robot_user", "ipc_bridge_user"):
        value = str(getattr(config, name)).strip()
        if (
            not value
            or len(value) > 64
            or any(character.isspace() or character in ":/" for character in value)
        ):
            raise ValueError(f"go2.{name} must be a safe local account name")
        ipc_users.append(value)
    if ipc_users[0] == ipc_users[1]:
        raise ValueError("go2 Robot and bridge IPC users must differ")
    heartbeat = _positive(
        config.ipc_heartbeat_interval_s,
        name="go2.ipc_heartbeat_interval_s",
    )
    if heartbeat >= float(safety.command_deadman_s):
        raise ValueError(
            "go2.ipc_heartbeat_interval_s must be less than "
            "safety.command_deadman_s"
        )
    if not active:
        return
    workspace = str(config.ros_workspace).strip()
    if not workspace or not Path(workspace).is_absolute():
        raise ValueError(
            "go2.ros_workspace must be an absolute path when GO2 is enabled"
        )
    unitree_interface = str(config.network_interface).strip()
    elesim_interface = str(dds.network_interface).strip()
    if not unitree_interface:
        raise ValueError("go2.network_interface is required when GO2 is enabled")
    if (
        len(unitree_interface) > 128
        or any(
            character.isspace() or character == "/"
            for character in unitree_interface
        )
    ):
        raise ValueError("go2.network_interface must be one interface name")
    if not elesim_interface:
        raise ValueError(
            "dds.network_interface is required when GO2 is enabled so the "
            "Unitree and Elesim graphs cannot overlap"
        )
    if unitree_interface == elesim_interface:
        raise ValueError(
            "go2.network_interface must differ from dds.network_interface"
        )
    if config.ros_domain_id is None:
        raise ValueError("go2.ros_domain_id is required when GO2 is enabled")
    if int(config.ros_domain_id) == int(dds.domain_id):
        raise ValueError("go2.ros_domain_id must differ from dds.domain_id")


def _validate_camera(config: CameraConfig) -> None:
    for name in ("width", "height", "fps"):
        _positive(getattr(config, name), name=f"camera.{name}")
    topic = str(config.topic).strip()
    if config.enabled and (not topic or not topic.startswith("/")):
        raise ValueError("camera.topic must be an absolute ROS topic when camera is enabled")


def _validate_safety(config: SafetyConfig) -> None:
    _positive(config.command_deadman_s, name="safety.command_deadman_s")
    _positive(config.monitor_period_s, name="safety.monitor_period_s", allow_zero=True)
    if isinstance(config.read_failure_limit, bool) or int(config.read_failure_limit) < 1:
        raise ValueError("safety.read_failure_limit must be at least 1")
    _positive(config.telemetry_period_s, name="safety.telemetry_period_s")
    _positive(config.max_go2_vx_m_s, name="safety.max_go2_vx_m_s")
    _positive(config.max_go2_vy_m_s, name="safety.max_go2_vy_m_s")
    _positive(config.max_go2_wz_rad_s, name="safety.max_go2_wz_rad_s")


def load_config(path: str | Path) -> RobotConfig:
    source = Path(path)
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    root = _mapping(raw, context="robot")
    allowed_top = {
        "schema_version",
        "runtime",
        "arm",
        "mapping",
        "go2",
        "camera",
        "safety",
        "dds",
    }
    unknown_top = sorted(set(root) - allowed_top)
    if unknown_top:
        raise ValueError(f"unknown robot config keys: {', '.join(unknown_top)}")
    schema_version = root.get("schema_version")
    if isinstance(schema_version, bool) or type(schema_version) is not int or schema_version != SCHEMA_VERSION:
        raise ValueError(f"schema_version must be exactly {SCHEMA_VERSION}")

    runtime = _mapping(root.get("runtime"), context="runtime")
    runtime_allowed = {"endpoint_id", "device", "use_go2"}
    runtime_unknown = sorted(set(runtime) - runtime_allowed)
    if runtime_unknown:
        raise ValueError(f"unknown runtime config keys: {', '.join(runtime_unknown)}")

    arm = HardwareConfig(**_values(HardwareConfig, root.get("arm"), context="arm"))
    mapping_values = _values(SimMappingConfig, root.get("mapping"), context="mapping")
    mapping_values.setdefault("linear_u_max", arm.linear_u_max_deg)
    mapping_values.setdefault("linear_u_limit", arm.linear_u_limit_deg)
    mapping_values.setdefault("command_direction", arm.command_direction)
    mapping = SimMappingConfig(**mapping_values)
    go2 = Go2HardwareConfig(**_values(Go2HardwareConfig, root.get("go2"), context="go2"))
    camera = CameraConfig(**_values(CameraConfig, root.get("camera"), context="camera"))
    safety = SafetyConfig(**_values(SafetyConfig, root.get("safety"), context="safety"))

    endpoint_id = str(runtime.get("endpoint_id", "robot-go2")).strip()
    if not endpoint_id:
        raise ValueError("runtime.endpoint_id must not be empty")
    dds_raw = _mapping(root.get("dds"), context="dds")
    vendor_config = str(dds_raw.get("vendor_config", "")).strip()
    if vendor_config:
        candidate = Path(vendor_config).expanduser()
        dds_raw["vendor_config"] = str(
            candidate
            if candidate.is_absolute()
            else (source.parent / candidate).resolve()
        )
    keystore = str(dds_raw.get("keystore", "")).strip()
    if keystore:
        candidate = Path(keystore).expanduser()
        dds_raw["keystore"] = str(
            candidate
            if candidate.is_absolute()
            else (source.parent / candidate).resolve()
        )
    dds = DdsRuntimeSettings.from_mapping(dds_raw, endpoint_id=endpoint_id)
    use_go2 = runtime.get("use_go2", True)
    if not isinstance(use_go2, bool):
        raise ValueError("runtime.use_go2 must be boolean")

    _validate_arm(arm)
    _validate_mapping(mapping)
    _validate_camera(camera)
    _validate_safety(safety)
    _validate_go2(
        go2,
        dds=dds,
        active=go2.is_active(use_go2=use_go2),
        safety=safety,
    )
    return RobotConfig(
        endpoint_id=endpoint_id,
        dds=dds,
        device=str(runtime.get("device", "")),
        use_go2=use_go2,
        arm=arm,
        mapping=mapping,
        go2=go2,
        camera=camera,
        safety=safety,
    )


__all__ = [
    "CameraConfig",
    "HardwareConfig",
    "RobotConfig",
    "SafetyConfig",
    "load_config",
]
