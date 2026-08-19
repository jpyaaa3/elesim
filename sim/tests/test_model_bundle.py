from __future__ import annotations

import json
import shutil
import tempfile
from pathlib import Path

import pytest

from elesim_sim.model_bundle import (
    ModelBundleError,
    resolve_camera_profile_bundle,
    resolve_model_bundle,
    validate_model_bundle,
)


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BUNDLE = ROOT / "model/bundles/default"
D435_BUNDLE = ROOT / "model/bundles/d435"


def test_checked_in_default_bundle_is_valid() -> None:
    metadata = validate_model_bundle(DEFAULT_BUNDLE)
    assert metadata["bundle_type"] == "elesim.sim-model"
    assert metadata["entrypoints"]["robot_urdf"] == "robot.urdf"


def test_checked_in_d435_bundle_is_valid() -> None:
    metadata = validate_model_bundle(D435_BUNDLE)
    assert metadata["bundle_type"] == "elesim.sim-model"


def test_explicit_model_bundle_is_resolved_and_validated() -> None:
    assert resolve_model_bundle(DEFAULT_BUNDLE) == DEFAULT_BUNDLE.resolve()


@pytest.mark.parametrize(
    ("profile", "bundle"),
    (("zed_mini", DEFAULT_BUNDLE), ("d435", D435_BUNDLE)),
)
def test_camera_profiles_select_matching_bundles(profile: str, bundle: Path) -> None:
    assert resolve_camera_profile_bundle(profile, bundle) == bundle.resolve()


def test_camera_profile_rejects_a_mismatched_explicit_bundle() -> None:
    with pytest.raises(ModelBundleError, match="requires model bundle"):
        resolve_camera_profile_bundle("d435", DEFAULT_BUNDLE)


def test_environment_model_bundle_takes_precedence(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ELESIM_MODEL_BUNDLE", str(DEFAULT_BUNDLE))

    assert resolve_model_bundle() == DEFAULT_BUNDLE.resolve()


def test_model_bundle_is_discovered_from_release_working_directory(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    with tempfile.TemporaryDirectory() as td:
        release = Path(td) / "sim"
        detached = release / "model/bundles/default"
        shutil.copytree(DEFAULT_BUNDLE, detached)
        work_dir = release / "run"
        work_dir.mkdir(parents=True)
        monkeypatch.delenv("ELESIM_MODEL_BUNDLE", raising=False)
        monkeypatch.chdir(work_dir)

        assert resolve_model_bundle() == detached.resolve()


def test_explicit_missing_model_bundle_fails_without_fallback() -> None:
    with tempfile.TemporaryDirectory() as td:
        missing = Path(td) / "missing"

        with pytest.raises(ModelBundleError, match="not found"):
            resolve_model_bundle(missing)


def test_runtime_rejects_detached_bundle_after_payload_tamper() -> None:
    with tempfile.TemporaryDirectory() as td:
        detached = Path(td) / "bundle"
        shutil.copytree(DEFAULT_BUNDLE, detached)
        metadata = json.loads((detached / "bundle.json").read_text(encoding="utf-8"))
        target = detached / next(iter(metadata["files"]))
        target.write_bytes(target.read_bytes() + b"tampered")

        with pytest.raises(ModelBundleError, match="hash mismatch"):
            validate_model_bundle(detached)


def test_runtime_rejects_reference_that_escapes_bundle() -> None:
    with tempfile.TemporaryDirectory() as td:
        detached = Path(td) / "bundle"
        shutil.copytree(DEFAULT_BUNDLE, detached)
        blueprint_path = detached / "blueprint.json"
        blueprint = json.loads(blueprint_path.read_text(encoding="utf-8"))
        blueprint["parts"][0]["assets"]["mesh"] = "../outside.obj"
        blueprint_path.write_text(json.dumps(blueprint), encoding="utf-8")

        with pytest.raises(ModelBundleError):
            validate_model_bundle(detached)
