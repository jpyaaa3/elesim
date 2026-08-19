from __future__ import annotations

import json
from pathlib import Path

import pytest

from elesim_sim.config import load_app_config
from elesim_sim.config.yaml_schema import ConfigValidationError, build_bundle_from_yaml
from elesim_sim.vision.camera_profile import camera_profile
from elesim_sim.vision.sim_camera.calibration import load_hand_eye_transform


ROOT = Path(__file__).resolve().parents[1]


def test_both_camera_profiles_and_calibrations_load() -> None:
    for name in ("zed_mini", "d435"):
        profile = camera_profile(name)
        calibration = ROOT / "config/calibration" / profile.calibration_filename
        payload = json.loads(calibration.read_text(encoding="utf-8"))
        assert payload["child_frame"]
        transform, _meta = load_hand_eye_transform(calibration)
        assert transform.shape == (4, 4)


def test_repository_sim_config_defaults_to_zed_mini() -> None:
    config = load_app_config(str(ROOT / "config/config.yaml"), mode="pc")
    assert config.sim_config.camera_profile == "zed_mini"
    assert config.sim_config.hand_eye_config.endswith("zed_mini.hand_eye.json")


def test_d435_yaml_selects_the_d435_bundle() -> None:
    d435 = ROOT / "config/calibration/d435.hand_eye.json"
    bundle = build_bundle_from_yaml(
        {
            "simulation": {
                "assembly": {"build_dir": str(ROOT.parent / "model/bundles/d435")},
                "cameras": {
                    "hand_eye": {
                        "profile": "d435",
                        "config": str(d435),
                    }
                },
            }
        },
        config_dir=str(ROOT / "config"),
    )
    assert bundle.sim_config.camera_profile == "d435"


def test_camera_profile_and_bundle_mismatch_fails_before_runtime() -> None:
    with pytest.raises(ConfigValidationError, match="requires model bundle"):
        build_bundle_from_yaml(
            {
                "simulation": {
                    "assembly": {"build_dir": str(ROOT.parent / "model/bundles/default")},
                    "cameras": {
                        "hand_eye": {
                            "profile": "d435",
                            "config": str(ROOT / "config/calibration/d435.hand_eye.json"),
                        }
                    },
                }
            },
            config_dir=str(ROOT / "config"),
        )
