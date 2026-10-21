from __future__ import annotations

from pathlib import Path

import yaml

from elesim_ui.config import load_config


REPO_ROOT = next(parent for parent in Path(__file__).resolve().parents if (parent / "payload").is_dir())
CONFIG_DIR = REPO_ROOT / "payload" / "config" / "ui"


def test_default_ui_config_binds_dds_to_loopback_trust_domain() -> None:
    config = load_config(CONFIG_DIR / "default.yaml")

    assert config.endpoint_id == "ui-main"
    assert config.dds.system_id == "elesim"
    assert config.dds.network_interface == "lo"
    assert config.dds.security_profile == "trusted-network"
    assert config.rgbd_topic == "/elesim/pilot_main/rgbd/left_preview"
    assert config.rgbd_right_topic == "/elesim/pilot_main/rgbd/right"
    assert config.show_zed_preview is False


def test_public_ui_config_uses_static_sros2_profile() -> None:
    config = load_config(CONFIG_DIR / "public.example.yaml")

    assert config.dds.domain_id == 42
    assert config.dds.static_peers == ("10.10.0.3",)
    assert config.dds.network_interface == "wg0"
    assert config.dds.security_profile == "sros2"


def test_ui_can_swap_zed_preview_eyes_without_changing_source_topics(tmp_path) -> None:
    raw = yaml.safe_load((CONFIG_DIR / "default.yaml").read_text())
    raw["presentation"]["zed_swap_left_right"] = True
    path = tmp_path / "ui-runtime.yaml"
    path.write_text(yaml.safe_dump(raw))

    config = load_config(path)

    assert config.rgbd_topic == "/elesim/pilot_main/rgbd/right"
    assert config.rgbd_right_topic == "/elesim/pilot_main/rgbd/left_preview"
