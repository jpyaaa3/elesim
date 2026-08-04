from __future__ import annotations


def select_runtime_urdf(*, use_go2: bool, arm_urdf: str, robot_urdf: str) -> str:
    return str(robot_urdf if bool(use_go2) else arm_urdf)
