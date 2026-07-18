from __future__ import annotations

import glob
import os
import sys
from typing import Iterable, List


def _python_version_tag() -> str:
    return f"python{sys.version_info.major}.{sys.version_info.minor}"


def _dist_package_dirs(root: str) -> List[str]:
    root = os.path.abspath(os.path.expanduser(str(root).strip()))
    if not root or not os.path.isdir(root):
        return []
    patterns = [
        os.path.join(root, "local", "lib", _python_version_tag(), "dist-packages"),
        os.path.join(root, "lib", _python_version_tag(), "site-packages"),
        os.path.join(root, "lib", _python_version_tag(), "dist-packages"),
    ]
    out: List[str] = []
    for path in patterns:
        if os.path.isdir(path):
            out.append(path)
    install_glob = os.path.join(root, "install", "*", "local", "lib", _python_version_tag(), "dist-packages")
    for path in sorted(glob.glob(install_glob)):
        if os.path.isdir(path):
            out.append(path)
    return out


def _prepend_sys_path(paths: Iterable[str]) -> List[str]:
    added: List[str] = []
    for raw in paths:
        path = os.path.abspath(os.path.expanduser(str(raw).strip()))
        if not path or not os.path.isdir(path):
            continue
        if path not in sys.path:
            sys.path.insert(0, path)
            added.append(path)
    return added


def _candidate_workspace_roots(config_workspace: str = "") -> List[str]:
    roots: List[str] = []
    for key in ("UNITREE_ROS2_WS", "COLCON_PREFIX_PATH"):
        val = os.environ.get(key, "").strip()
        if val:
            for part in val.split(os.pathsep):
                part = part.strip()
                if part:
                    roots.append(part)
    cfg = str(config_workspace).strip()
    if cfg:
        roots.append(cfg)
    roots.extend(
        [
            "~/unitree_ros2",
            "~/ros2_ws",
            "/home/idim/unitree_ros2",
        ]
    )
    humble = os.environ.get("ROS_DISTRO", "").strip()
    if humble:
        roots.append(f"/opt/ros/{humble}")
    else:
        roots.append("/opt/ros/humble")
    deduped: List[str] = []
    seen = set()
    for root in roots:
        path = os.path.abspath(os.path.expanduser(root))
        if path in seen:
            continue
        seen.add(path)
        deduped.append(path)
    return deduped


def bootstrap_ros_python_path(*, config_workspace: str = "") -> List[str]:
    """Best-effort PYTHONPATH setup for Jetson when ROS setup.bash was not sourced."""
    paths: List[str] = []
    for root in _candidate_workspace_roots(config_workspace):
        paths.extend(_dist_package_dirs(root))
        install_root = os.path.join(root, "install")
        if os.path.isdir(install_root):
            paths.extend(_dist_package_dirs(install_root))
    ament = os.environ.get("AMENT_PREFIX_PATH", "").strip()
    if ament:
        for prefix in ament.split(os.pathsep):
            paths.extend(_dist_package_dirs(prefix.strip()))
    return _prepend_sys_path(paths)


def ros_import_hint(*, config_workspace: str = "") -> str:
    ws = str(config_workspace).strip() or "~/unitree_ros2"
    return (
        "GO2 ROS2 Python packages were not found. On Jetson, source ROS2 + unitree_ros2, then retry:\n"
        "  source /opt/ros/humble/setup.bash\n"
        f"  source {ws}/install/setup.bash\n"
        "  export UNITREE_ROS2_WS=" + ws + "\n"
        "  python robot_agent.py --config configs/config.jetson.yaml\n"
        "Or use: bash scripts/run_robot_jetson.sh\n"
        "Verify: python3 -c \"from unitree_api.msg import Request; print('ok')\""
    )
