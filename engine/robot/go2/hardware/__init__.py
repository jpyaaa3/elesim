from __future__ import annotations

__all__ = [
    "Go2HardwareConfig",
    "OdomSample",
    "UnitreeRos2Bridge",
    "create_go2_bridge_if_enabled",
]


def __getattr__(name: str):
    if name == "Go2HardwareConfig":
        from engine.robot.go2.hardware.config import Go2HardwareConfig

        return Go2HardwareConfig
    if name == "OdomSample":
        from engine.robot.go2.hardware.odom_parser import OdomSample

        return OdomSample
    if name in {"UnitreeRos2Bridge", "create_go2_bridge_if_enabled"}:
        from engine.robot.go2.hardware import unitree_ros2_bridge

        return getattr(unitree_ros2_bridge, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
