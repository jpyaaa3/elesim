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
    assert config.safety.router_liveness_s > config.safety.command_deadman_s


def test_public_config_requires_controller_media_allowlist_path() -> None:
    config = load_config(ROOT / "robot/config/public.example.yaml")

    assert config.camera.bind == "tcp://0.0.0.0:5568"
    assert config.security.media_server_secret_file.endswith("robot-media.key_secret")
    assert config.security.media_client_public_keys_dir.endswith("media-authorized")


def test_unknown_top_level_section_is_rejected() -> None:
    with pytest.raises(ValueError, match="unknown robot config keys"):
        _load_document({"schema_version": 2, "surprise": {}})


def test_unknown_nested_key_is_rejected() -> None:
    with pytest.raises(ValueError, match="unknown arm config keys"):
        _load_document({"schema_version": 2, "arm": {"typo_current_limit": 10}})


@pytest.mark.parametrize("schema", (None, 0, 1, 3, "2"))
def test_schema_version_must_be_exactly_two(schema: object) -> None:
    with pytest.raises(ValueError, match="schema_version"):
        _load_document({"schema_version": schema})


def test_invalid_safety_bounds_are_rejected() -> None:
    with pytest.raises(ValueError, match="router_liveness_s"):
        _load_document(
            {
                "schema_version": 2,
                "safety": {"command_deadman_s": 1.0, "router_liveness_s": 0.5},
            }
        )
