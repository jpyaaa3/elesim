from __future__ import annotations

__all__ = [
    "Go2HardwareConfig",
    "OdomSample",
    "UnitreeRos2Bridge",
    "create_go2_bridge_if_enabled",
]


def __getattr__(name: str):
    if name == "Go2HardwareConfig":
        from elesim_robot.go2.config import Go2HardwareConfig

        return Go2HardwareConfig
    if name == "OdomSample":
        from elesim_robot.go2.odom_parser import OdomSample

        return OdomSample
    if name in {"UnitreeRos2Bridge", "create_go2_bridge_if_enabled"}:
        from elesim_robot.go2 import unitree_ros2_bridge

        return getattr(unitree_ros2_bridge, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
