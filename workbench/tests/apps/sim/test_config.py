from __future__ import annotations

from pathlib import Path

import pytest

from elesim_sim.config import load_app_config
from elesim_sim.config.yaml_schema import ConfigValidationError
from elesim_sim.runtime import _select_compute_backend


REPO_ROOT = next(parent for parent in Path(__file__).resolve().parents if (parent / "payload").is_dir())
CONFIG_DIR = REPO_ROOT / "payload" / "config" / "sim"


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
    assert bundle.sim_config.sim_camera_width == 1280
    assert bundle.sim_config.sim_camera_height == 720
    assert bundle.sim_config.sim_observer_camera_width == 1280
    assert bundle.sim_config.sim_observer_camera_height == 720
    assert bundle.sim_config.sim_observer_camera_fov_deg == 40.0
    assert bundle.sim_config.sim_observer_camera_pos == (3.5, 0.5, 2.5)
    assert bundle.sim_config.sim_observer_camera_lookat == (0.0, 0.0, 0.5)
    assert bundle.sim_config.perf_log_enable is False
    assert bundle.sim_config.genesis_performance_mode is False
    assert bundle.sim_config.camera_execution == "async_process"
    assert bundle.sim_config.camera_first_frame_timeout_s == 30.0
    assert bundle.sim_config.visualizer_max_hz == 30.0
    assert bundle.go2_locomotion_config.mode == "pympc"
    assert bundle.go2_locomotion_config.mpc_solver_backend == "acados"
    assert not hasattr(bundle, "pick_config")
    assert not hasattr(bundle, "perception_config")
    assert not hasattr(bundle, "gaze_stabilizer_config")
    assert not hasattr(bundle, "go2_hardware_config")
    assert not hasattr(bundle, "ik_config")
    assert not hasattr(bundle, "hardware_config")


def test_remote_profile_disables_native_viewer_but_keeps_network_cameras() -> None:
    bundle = load_app_config(str(CONFIG_DIR / "config.yaml"), mode="remote")

    assert bundle.sim_config.enable_viewer is False
    assert bundle.sim_config.telemetry_max_hz == 20.0
    assert bundle.sim_config.sim_camera_enable is True
    assert bundle.sim_config.sim_observer_camera_enable is True
    assert bundle.sim_config.sim_camera_max_hz == 30.0
    assert bundle.sim_config.sim_observer_camera_max_hz == 30.0


def test_gpu_mpc_selection_is_explicit(tmp_path):
    config = tmp_path / "gpu.yaml"
    config.write_text("schema_version: 1\nrobot:\n  go2:\n    locomotion:\n      mpc:\n"
                      "        solver_backend: jax_mppi\n        gpu_samples: 2048\n")
    bundle = load_app_config(str(config))
    assert bundle.go2_locomotion_config.mpc_solver_backend == "jax_mppi"
    assert bundle.go2_locomotion_config.mpc_gpu_samples == 2048


@pytest.mark.parametrize("key,value", [("solver_backend", "auto"), ("gpu_samples", 0),
                                       ("gpu_iterations", 9), ("gpu_seed", -1),
                                       ("gpu_samples", True), ("gpu_iterations", 1.5)])
def test_bad_gpu_mpc_configuration_fails_before_scene_build(tmp_path, key, value):
    import yaml
    config = tmp_path / "gpu.yaml"
    config.write_text(yaml.safe_dump({"schema_version": 1, "robot": {"go2": {
        "locomotion": {"mpc": {key: value}}}}}))
    with pytest.raises(ConfigValidationError, match=key):
        load_app_config(str(config))


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


def test_sim_can_opt_into_static_genesis_arrays(tmp_path: Path) -> None:
    path = tmp_path / "static-arrays.yaml"
    path.write_text(
        "schema_version: 1\nsimulation:\n  runtime:\n"
        "    genesis_performance_mode: true\n",
        encoding="utf-8",
    )
    assert load_app_config(str(path)).sim_config.genesis_performance_mode is True


@pytest.mark.parametrize(
    ("key", "value", "message"),
    (
        ("genesis_performance_mode", "fast", "genesis_performance_mode"),
        ("camera_execution", "threaded", "camera_execution"),
        ("camera_worker_start_timeout_s", 0, "camera_worker_start_timeout_s"),
        ("camera_first_frame_timeout_s", 0, "camera_first_frame_timeout_s"),
        ("visualizer_max_hz", -1, "visualizer_max_hz"),
    ),
)
def test_sim_rejects_invalid_performance_policy(
    tmp_path: Path, key: str, value: object, message: str
) -> None:
    path = tmp_path / "invalid-performance.yaml"
    path.write_text(
        "schema_version: 1\n"
        "simulation:\n"
        "  runtime:\n"
        f"    {key}: {value}\n",
        encoding="utf-8",
    )
    with pytest.raises(ConfigValidationError, match=message):
        load_app_config(str(path))
