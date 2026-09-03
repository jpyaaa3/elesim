from __future__ import annotations

import json
from pathlib import Path

import pytest

from elesim_pilot.config import load_app_config
from elesim_pilot.config.yaml_schema import ConfigValidationError, build_bundle_from_yaml
from elesim_pilot.vision.perception.camera_factory import camera_class
from elesim_pilot.vision.perception.camera_profile import camera_profile
from elesim_pilot.vision.perception_bridge.hand_eye import load_hand_eye_transform
from elesim_pilot.vision.perception.zed_camera import ZedMiniCamera, ZedUnavailableError


REPO_ROOT = next(parent for parent in Path(__file__).resolve().parents if (parent / "AGENTS.md").is_file())
CONFIG_ROOT = REPO_ROOT / "payload" / "config" / "pilot"
MODEL_ROOT = REPO_ROOT / "payload" / "data" / "models" / "assemblies"
DATA_ROOT = REPO_ROOT / "payload" / "data"


def test_camera_profiles_are_explicit_and_select_driver() -> None:
    assert camera_profile("zed_mini").driver == "zed"
    assert camera_profile("d435").driver == "realsense"
    assert camera_class("d435").__name__ == "RealSenseCamera"
    assert camera_class("zed_mini") is ZedMiniCamera


def test_missing_zed_sdk_is_a_hard_profile_error(monkeypatch: pytest.MonkeyPatch) -> None:
    import elesim_pilot.vision.perception.zed_camera as module

    monkeypatch.setattr(module, "sl", None)
    with pytest.raises(ZedUnavailableError, match="optional ZED SDK"):
        ZedMiniCamera()


def test_repository_config_defaults_to_zed_mini() -> None:
    config = load_app_config(str(CONFIG_ROOT / "config.yaml"), mode="pc")
    assert config.perception_config.camera_profile == "zed_mini"
    assert config.sim_config.camera_profile == "zed_mini"
    assert config.sim_config.hand_eye_config.endswith("zed_mini.hand_eye.json")


def test_both_calibrations_load_and_d435_yaml_selects_its_driver_contract() -> None:
    for name in ("zed_mini", "d435"):
        calibration = DATA_ROOT / "calibration/cameras" / f"{name}.hand_eye.json"
        payload = json.loads(calibration.read_text(encoding="utf-8"))
        assert payload["child_frame"]
        transform, _meta = load_hand_eye_transform(calibration)
        assert transform.shape == (4, 4)

    d435 = DATA_ROOT / "calibration/cameras/d435.hand_eye.json"
    bundle = build_bundle_from_yaml(
        {
            "simulation": {
                "assembly": {"build_dir": str(MODEL_ROOT / "d435")},
                "cameras": {
                    "hand_eye": {
                        "profile": "d435",
                        "config": str(d435),
                    }
                },
            },
            "vision": {"perception": {"camera": {"profile": "d435"}}},
        },
        config_dir=str(CONFIG_ROOT),
    )
    assert bundle.sim_config.camera_profile == "d435"
    assert bundle.perception_config.camera_profile == "d435"
    assert camera_class(bundle.perception_config.camera_profile).__name__ == "RealSenseCamera"


def test_camera_profile_and_bundle_mismatch_fails_before_runtime() -> None:
    with pytest.raises(ConfigValidationError, match="requires model bundle"):
        build_bundle_from_yaml(
            {
                "simulation": {
                    "assembly": {"build_dir": str(MODEL_ROOT / "zed-mini")},
                    "cameras": {
                        "hand_eye": {
                            "profile": "d435",
                            "config": str(DATA_ROOT / "calibration/cameras/d435.hand_eye.json"),
                        }
                    },
                },
                "vision": {"perception": {"camera": {"profile": "d435"}}},
            },
            config_dir=str(CONFIG_ROOT),
        )
