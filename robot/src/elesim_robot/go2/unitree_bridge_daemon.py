#!/usr/bin/env python3
"""Dedicated Unitree DDS process; Robot communicates with it only over UDS."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import threading
from pathlib import Path
from xml.sax.saxutils import quoteattr

from elesim_robot.config import load_config
from elesim_robot.go2.unitree_ipc import UnitreeBridgeServer
from elesim_robot.go2.unitree_ros2_bridge import UnitreeRos2Bridge


PROJECT_CONFIG = Path(__file__).resolve().parents[3] / "config/default.yaml"
_WORKSPACE_READY = "ELESIM_UNITREE_WORKSPACE_READY"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="EleSim Unitree local DDS bridge")
    parser.add_argument(
        "--config",
        default=os.environ.get("ELESIM_ROBOT_CONFIG", str(PROJECT_CONFIG)),
    )
    return parser


def _apply_unitree_environment(*, interface: str, domain_id: int) -> None:
    name = str(interface).strip()
    if not name:
        raise ValueError("Unitree network interface is required")
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["ROS_DOMAIN_ID"] = str(int(domain_id))
    os.environ["ROS_LOCALHOST_ONLY"] = "0"
    os.environ["CYCLONEDDS_URI"] = (
        "<CycloneDDS><Domain Id=\"any\"><General><Interfaces>"
        f"<NetworkInterface name={quoteattr(name)} />"
        "</Interfaces></General></Domain></CycloneDDS>"
    )
    for key in (
        "ROS_SECURITY_ENABLE",
        "ROS_SECURITY_STRATEGY",
        "ROS_SECURITY_KEYSTORE",
        "ROS_SECURITY_ENCLAVE_OVERRIDE",
        "ROS_SECURITY_ROOT_DIRECTORY",
        "ELESIM_DDS_STATIC_PEERS",
        "ELESIM_DDS_NETWORK_INTERFACE",
        "FASTRTPS_DEFAULT_PROFILES_FILE",
    ):
        os.environ.pop(key, None)


def _source_workspace_environment(workspace: str) -> Path | None:
    configured = str(workspace).strip()
    if not configured:
        return None
    setup = Path(configured).expanduser().resolve() / "install/setup.bash"
    if not setup.is_file():
        raise RuntimeError(f"Unitree ROS 2 workspace setup is missing: {setup}")
    result = subprocess.run(
        [
            "/bin/bash",
            "-c",
            'set -euo pipefail; source "$1"; env -0',
            "unitree-workspace",
            str(setup),
        ],
        check=False,
        capture_output=True,
        timeout=15.0,
    )
    if result.returncode != 0:
        detail = result.stderr.decode("utf-8", errors="replace").strip()[-1000:]
        raise RuntimeError(f"failed to source Unitree workspace: {detail}")
    for entry in result.stdout.split(b"\0"):
        if not entry or b"=" not in entry:
            continue
        key, value = entry.split(b"=", 1)
        os.environ[os.fsdecode(key)] = os.fsdecode(value)
    return setup


def _ensure_workspace_environment(workspace: str, *, config_path: str) -> None:
    configured = str(workspace).strip()
    if not configured:
        return
    setup = Path(configured).expanduser().resolve() / "install/setup.bash"
    if os.environ.get(_WORKSPACE_READY) == str(setup):
        return
    _source_workspace_environment(configured)
    os.environ[_WORKSPACE_READY] = str(setup)
    os.execve(
        sys.executable,
        [
            sys.executable,
            "-m",
            "elesim_robot.go2.unitree_bridge_daemon",
            "--config",
            str(Path(config_path).expanduser().resolve()),
        ],
        dict(os.environ),
    )


def _run() -> None:
    args = _parser().parse_args()
    config = load_config(args.config)
    stop_event = threading.Event()

    def stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    if not config.go2.is_active(use_go2=config.use_go2):
        while not stop_event.wait(1.0):
            pass
        return

    assert config.go2.ros_domain_id is not None  # Enforced by schema v4.
    _ensure_workspace_environment(config.go2.ros_workspace, config_path=args.config)
    _apply_unitree_environment(
        interface=config.go2.network_interface,
        domain_id=int(config.go2.ros_domain_id),
    )
    backend = UnitreeRos2Bridge(config.go2)
    UnitreeBridgeServer(config.go2, config.safety, backend).serve_forever(stop_event)


def main() -> None:
    _run()


if __name__ == "__main__":
    main()
