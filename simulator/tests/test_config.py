from __future__ import annotations

from pathlib import Path

import pytest

from elesim_simulator.config import load_app_config
from elesim_simulator.config.yaml_schema import ConfigValidationError


CONFIG_DIR = Path(__file__).parents[1] / "config"


@pytest.mark.parametrize(
    "name",
    ("config.yaml", "default.yaml", "config.pc.yaml", "config.jetson.yaml"),
)
def test_simulator_configs_load_with_role_owned_schema(name: str) -> None:
    bundle = load_app_config(str(CONFIG_DIR / name))
    assert bundle.sim_config.sim_camera_width > 0
    assert bundle.sim_config.sim_camera_height > 0
    assert not hasattr(bundle, "pick_config")
    assert not hasattr(bundle, "perception_config")
    assert not hasattr(bundle, "gaze_stabilizer_config")
    assert not hasattr(bundle, "go2_hardware_config")
    assert not hasattr(bundle, "ik_config")
    assert not hasattr(bundle, "hardware_config")


def test_simulator_schema_rejects_controller_workflow_keys(tmp_path: Path) -> None:
    path = tmp_path / "invalid.yaml"
    path.write_text(
        "schema_version: 1\nbehaviors:\n  pick:\n    general:\n      enabled: true\n",
        encoding="utf-8",
    )
    with pytest.raises(ConfigValidationError, match="unknown config key"):
        load_app_config(str(path))


def test_simulator_rejects_ini_configuration(tmp_path: Path) -> None:
    path = tmp_path / "legacy.ini"
    path.write_text("[runtime]\n", encoding="utf-8")
    with pytest.raises(ValueError, match="YAML"):
        load_app_config(str(path))
