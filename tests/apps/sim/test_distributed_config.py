from __future__ import annotations

from pathlib import Path

from elesim_sim.config import load_runtime_role_config


REPO_ROOT = next(parent for parent in Path(__file__).resolve().parents if (parent / "AGENTS.md").is_file())
CONFIG_DIR = REPO_ROOT / "payload" / "config" / "sim"


def test_loopback_runtime_uses_trusted_network_dds_without_turn_secret() -> None:
    config = load_runtime_role_config(CONFIG_DIR / "runtime.yaml")

    assert config.dds.system_id == "elesim"
    assert config.dds.network_interface == "lo"
    assert config.dds.security_profile == "trusted-network"
    assert config.streams["rgbd_topic"] == "/elesim/sim_default/rgbd/frame"
    assert config.turn.static_auth_secret_file is None
    assert config.turn.credential_file is None


def test_public_runtime_uses_static_dds_peer_sros2_and_managed_turn() -> None:
    config = load_runtime_role_config(CONFIG_DIR / "runtime.public.example.yaml")

    assert config.dds.discovery_mode == "static"
    assert config.dds.static_peers == ("10.10.0.2",)
    assert config.dds.security_profile == "sros2"
    assert config.turn.urls == ("turn:sim.example.com:3478?transport=udp",)
    assert config.turn.realm == "sim.example.com"
    assert str(config.turn.static_auth_secret_file).endswith("turn.secret")
    assert config.turn.credential_file is None
