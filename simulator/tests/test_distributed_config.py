from __future__ import annotations

from pathlib import Path

from elesim_simulator.config import load_runtime_role_config


CONFIG_DIR = Path(__file__).parents[1] / "config"


def test_loopback_runtime_has_no_transport_secrets() -> None:
    config = load_runtime_role_config(CONFIG_DIR / "runtime.yaml")

    assert config.server_endpoint == "tcp://127.0.0.1:5558"
    assert config.media_server_secret_file == ""
    assert config.media_client_public_keys_dir == ""


def test_public_runtime_separates_rgbd_bind_and_advertised_address() -> None:
    config = load_runtime_role_config(CONFIG_DIR / "runtime.public.example.yaml")

    assert config.streams["rgbd_bind"] == "tcp://0.0.0.0:5568"
    assert config.streams["rgbd_advertise"] == "tcp://sim.example.com:5568"
    assert config.media_server_secret_file.endswith("simulator-media.key_secret")
    assert config.media_client_public_keys_dir.endswith("media-authorized")
