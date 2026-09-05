from __future__ import annotations

import tempfile
from pathlib import Path

import pytest
import yaml

from elesim_robot.config import load_config


ROOT = next(path for path in Path(__file__).resolve().parents if (path / "payload").is_dir())


def _load_document(document: dict) -> object:
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "robot.yaml"
        path.write_text(yaml.safe_dump(document), encoding="utf-8")
        return load_config(path)


def test_checked_in_default_config_is_valid() -> None:
    config = load_config(ROOT / "payload/config/robot/default.yaml")
    assert config.dds.security_profile == "trusted-network"
    assert config.camera.topic.startswith("/")
    assert config.safety.command_deadman_s > 0.0
    assert config.go2.network_interface != config.dds.network_interface
    assert config.go2.ros_domain_id != config.dds.domain_id


def test_systemd_accounts_match_the_default_ipc_peer_identities() -> None:
    config = load_config(ROOT / "payload/config/robot/default.yaml")
    robot_unit = (ROOT / "payload/runtime/native/robot/systemd/elesim-robot.service").read_text(
        encoding="utf-8"
    )
    bridge_unit = (
        ROOT / "payload/runtime/native/robot/systemd/elesim-unitree-bridge.service"
    ).read_text(encoding="utf-8")

    assert f"User={config.go2.ipc_robot_user}\n" in robot_unit
    assert f"SupplementaryGroups={config.go2.ipc_bridge_user}\n" in robot_unit
    assert f"User={config.go2.ipc_bridge_user}\n" in bridge_unit
    assert f"Group={config.go2.ipc_bridge_user}\n" in bridge_unit


def test_public_config_uses_sros2_and_static_vpn_discovery() -> None:
    config = load_config(ROOT / "payload/config/robot/public.example.yaml")

    assert config.dds.security_profile == "sros2"
    assert config.dds.discovery_mode == "static"
    assert config.dds.static_peers == ("10.8.0.1",)
    assert config.dds.network_interface == "wg0"
    assert config.camera.topic == "/elesim/robot_go2/rgbd/frame"


def test_unknown_top_level_section_is_rejected() -> None:
    with pytest.raises(ValueError, match="unknown robot config keys"):
        _load_document({"schema_version": 2, "surprise": {}})


def test_unknown_nested_key_is_rejected() -> None:
    with pytest.raises(ValueError, match="unknown arm config keys"):
        _load_document({"schema_version": 4, "arm": {"typo_current_limit": 10}})


@pytest.mark.parametrize("schema", (None, 0, 1, 2, 3, 5, "4"))
def test_schema_version_must_be_exactly_four(schema: object) -> None:
    with pytest.raises(ValueError, match="schema_version"):
        _load_document({"schema_version": schema})


def test_invalid_safety_bounds_are_rejected() -> None:
    with pytest.raises(ValueError, match="command_deadman_s"):
        _load_document(
            {
                "schema_version": 4,
                "safety": {"command_deadman_s": 0.0},
            }
        )


@pytest.mark.parametrize(
    "dds, go2, error",
    (
        (
            {"domain_id": 0, "network_interface": "eth0"},
            {
                "enabled": True,
                "ros_workspace": "/opt/unitree_ros2",
                "ros_domain_id": 1,
                "network_interface": "eth0",
            },
            "network_interface must differ",
        ),
        (
            {"domain_id": 7, "network_interface": "tailscale0"},
            {
                "enabled": True,
                "ros_workspace": "/opt/unitree_ros2",
                "ros_domain_id": 7,
                "network_interface": "eth0",
            },
            "ros_domain_id must differ",
        ),
        (
            {"domain_id": 0, "network_interface": "tailscale0"},
            {
                "enabled": True,
                "ros_workspace": "/opt/unitree_ros2",
                "ros_domain_id": None,
                "network_interface": "eth0",
            },
            "ros_domain_id is required",
        ),
        (
            {"domain_id": 0, "network_interface": "tailscale0"},
            {
                "enabled": True,
                "ros_workspace": "~/ros2_ws",
                "ros_domain_id": 1,
                "network_interface": "eth0",
            },
            "ros_workspace must be an absolute path",
        ),
        (
            {"domain_id": 0, "network_interface": "tailscale0"},
            {
                "enabled": True,
                "ros_workspace": "/opt/unitree_ros2",
                "ros_domain_id": 1,
                "network_interface": "eth0",
                "ipc_robot_user": "same-user",
                "ipc_bridge_user": "same-user",
            },
            "IPC users must differ",
        ),
        (
            {"domain_id": 0, "network_interface": "tailscale0"},
            {
                "enabled": True,
                "ros_workspace": "/opt/unitree_ros2",
                "ros_domain_id": 1,
                "network_interface": "eth 0",
            },
            "must be one interface name",
        ),
    ),
)
def test_active_go2_requires_a_separate_dds_graph(
    dds: dict[str, object],
    go2: dict[str, object],
    error: str,
) -> None:
    with pytest.raises(ValueError, match=error):
        _load_document(
            {
                "schema_version": 4,
                "runtime": {"use_go2": True},
                "dds": dds,
                "go2": go2,
            }
        )


def test_disabled_go2_does_not_require_network_separation() -> None:
    config = _load_document(
        {
            "schema_version": 4,
            "runtime": {"use_go2": False},
            "go2": {"enabled": True},
        }
    )
    assert config.use_go2 is False
