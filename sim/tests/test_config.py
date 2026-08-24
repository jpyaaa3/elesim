from __future__ import annotations

from pathlib import Path

import pytest

from elesim_sim.config import load_app_config
from elesim_sim.config.yaml_schema import ConfigValidationError
from elesim_sim.runtime import _select_compute_backend


CONFIG_DIR = Path(__file__).parents[1] / "config"


@pytest.mark.parametrize(
    "name",
    (
        "config.yaml",
    ),
)
def test_sim_configs_load_with_role_owned_schema(name: str) -> None:
    bundle = load_app_config(str(CONFIG_DIR / name))
    assert bundle.sim_config.sim_camera_width > 0
    assert bundle.sim_config.sim_camera_height > 0
    assert bundle.sim_config.sim_observer_camera_width == 320
    assert bundle.sim_config.sim_observer_camera_height == 240
    assert bundle.sim_config.sim_observer_camera_fov_deg == 40.0
    assert bundle.sim_config.sim_observer_camera_pos == (3.5, 0.5, 2.5)
    assert bundle.sim_config.sim_observer_camera_lookat == (0.0, 0.0, 0.5)
    assert bundle.sim_config.perf_log_enable is True
    assert not hasattr(bundle, "pick_config")
    assert not hasattr(bundle, "perception_config")
    assert not hasattr(bundle, "gaze_stabilizer_config")
    assert not hasattr(bundle, "go2_hardware_config")
    assert not hasattr(bundle, "ik_config")
    assert not hasattr(bundle, "hardware_config")


def test_remote_profile_disables_native_viewer_but_keeps_network_cameras() -> None:
    bundle = load_app_config(str(CONFIG_DIR / "config.yaml"), mode="remote")

    assert bundle.sim_config.enable_viewer is False
    assert bundle.sim_config.sim_camera_enable is True
    assert bundle.sim_config.sim_observer_camera_enable is True


def test_cpu_runtime_override_yields_gpu_without_mutating_profile() -> None:
    bundle = load_app_config(str(CONFIG_DIR / "config.yaml"), mode="remote")

    selected = _select_compute_backend(bundle.sim_config, force_cpu=True)

    assert selected.use_gpu is False
    assert bundle.sim_config.use_gpu is True
    assert _select_compute_backend(bundle.sim_config, force_cpu=False) is bundle.sim_config


def test_sim_schema_rejects_controller_workflow_keys(tmp_path: Path) -> None:
    path = tmp_path / "invalid.yaml"
    path.write_text(
        "schema_version: 1\nbehaviors:\n  pick:\n    general:\n      enabled: true\n",
        encoding="utf-8",
    )
    with pytest.raises(ConfigValidationError, match="unknown config key"):
        load_app_config(str(path))


def test_sim_rejects_ini_configuration(tmp_path: Path) -> None:
    path = tmp_path / "legacy.ini"
    path.write_text("[runtime]\n", encoding="utf-8")
    with pytest.raises(ValueError, match="YAML"):
        load_app_config(str(path))
