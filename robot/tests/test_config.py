from __future__ import annotations

import tempfile
from pathlib import Path

import pytest
import yaml

from elesim_robot.config import load_config


ROOT = Path(__file__).resolve().parents[2]


def _load_document(document: dict) -> object:
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "robot.yaml"
        path.write_text(yaml.safe_dump(document), encoding="utf-8")
        return load_config(path)


def test_checked_in_default_config_is_valid() -> None:
    config = load_config(ROOT / "robot/config/default.yaml")
    assert config.dds.security_profile == "trusted-network"
    assert config.camera.topic.startswith("/")
    assert config.safety.command_deadman_s > 0.0


def test_public_config_uses_sros2_and_static_vpn_discovery() -> None:
    config = load_config(ROOT / "robot/config/public.example.yaml")

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
        _load_document({"schema_version": 3, "arm": {"typo_current_limit": 10}})


@pytest.mark.parametrize("schema", (None, 0, 1, 2, 4, "3"))
def test_schema_version_must_be_exactly_three(schema: object) -> None:
    with pytest.raises(ValueError, match="schema_version"):
        _load_document({"schema_version": schema})


def test_invalid_safety_bounds_are_rejected() -> None:
    with pytest.raises(ValueError, match="command_deadman_s"):
        _load_document(
            {
                "schema_version": 3,
                "safety": {"command_deadman_s": 0.0},
            }
        )
