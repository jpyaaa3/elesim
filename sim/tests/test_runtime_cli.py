from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest
import elesim_sim.runtime as runtime
from elesim_sim.config import load_app_config


ROOT = Path(__file__).resolve().parents[1]


def test_viewer_flag_overrides_remote_profile(monkeypatch) -> None:
    captured = {}

    class FakeGenesisApp:
        def __init__(self, **kwargs):
            captured.update(kwargs)

        def run(self) -> None:
            return None

    monkeypatch.setattr(runtime, "GenesisApp", FakeGenesisApp)
    runtime.run_runtime(
        config_path=str(ROOT / "config/config.remote.yaml"),
        argv=["--viewer"],
    )

    remote = load_app_config(str(ROOT / "config/config.remote.yaml"))
    assert remote.sim_config.enable_viewer is False
    assert captured["cfg"].enable_viewer is True


def test_no_viewer_flag_keeps_remote_profile_headless(monkeypatch) -> None:
    captured = {}

    class FakeGenesisApp:
        def __init__(self, **kwargs):
            captured.update(kwargs)

        def run(self) -> None:
            return None

    monkeypatch.setattr(runtime, "GenesisApp", FakeGenesisApp)
    runtime.run_runtime(
        config_path=str(ROOT / "config/config.pc.yaml"),
        argv=["--no-viewer"],
    )

    assert captured["cfg"].enable_viewer is False
    assert captured["cfg"].sim_camera_enable is True
    assert captured["cfg"].sim_observer_camera_enable is True


def test_required_sim_target_spawn_failure_is_not_silenced(monkeypatch) -> None:
    class FailingMorphs:
        @staticmethod
        def Sphere(**_kwargs):
            raise ValueError("Genesis rejected target morph")

    fake_genesis = SimpleNamespace(morphs=FailingMorphs())
    monkeypatch.setattr(runtime, "gs", fake_genesis)

    app = SimpleNamespace(
        sim_scene=SimpleNamespace(scene=object()),
        spawn=SimpleNamespace(
            sim_target_xyz=(0.8, 0.0, 0.2),
            sim_target_radius=0.025,
            sim_target_color_rgba=(0.85, 0.15, 0.15, 1.0),
            sim_target_collision=True,
            sim_target_gravity=False,
        ),
    )

    with pytest.raises(RuntimeError, match="sim target spawn failed"):
        runtime.RuntimePrep(app)._spawn_perception_target()
