from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

import elesim_model_builder.cli as model_cli
from elesim_model_builder.arm_model import build_arm_model


ROOT = Path(__file__).resolve().parents[3]
BASE_CONFIG = ROOT / "pilot/config/config.yaml"
ASSETS = ROOT / "model/source/assets"


def test_arm_model_build_uses_an_isolated_workspace(tmp_path: Path) -> None:
    configured_build_dir = tmp_path / "configured-build"
    config = tmp_path / "pilot.yaml"
    config.write_text(
        "schema_version: 1\n"
        f"extends: {BASE_CONFIG}\n"
        "simulation:\n"
        "  assembly:\n"
        f"    build_dir: {configured_build_dir}\n"
        "    rebuild_assembly: true\n",
        encoding="utf-8",
    )
    output = tmp_path / "release-config" / "arm_model.json"

    result = build_arm_model(config=config, assets=ASSETS, output=output)

    assert result == output
    assert not configured_build_dir.exists()
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["schema_version"] == 1
    assert payload["context"]["limit"]["__dataclass__"] == "JointLimit"


def test_installed_cli_dispatches_to_the_packaged_builder(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    expected = {
        "config": tmp_path / "pilot.yaml",
        "assets": tmp_path / "assets",
        "output": tmp_path / "arm_model.json",
    }
    received: dict[str, Path] = {}

    def fake_build_arm_model(**kwargs: Path) -> Path:
        received.update(kwargs)
        return kwargs["output"]

    monkeypatch.setattr(model_cli, "build_arm_model", fake_build_arm_model)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "elesim-build-arm-model",
            "--config",
            str(expected["config"]),
            "--assets",
            str(expected["assets"]),
            "--output",
            str(expected["output"]),
        ],
    )

    model_cli.arm_model_main()

    assert received == expected
