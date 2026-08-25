from __future__ import annotations

import os
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import elesim_sim.runtime as runtime
from elesim_sim.config import load_app_config
from elesim_sim.main import _configure_gpu_render_environment


ROOT = Path(__file__).resolve().parents[1]


def test_headless_gpu_render_uses_selected_cuda_device(monkeypatch) -> None:
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "2")
    monkeypatch.delenv("CUDA_DEVICE_ORDER", raising=False)
    monkeypatch.delenv("PYOPENGL_PLATFORM", raising=False)
    monkeypatch.delenv("EGL_DEVICE_ID", raising=False)

    _configure_gpu_render_environment(use_gpu=True, viewer=False)

    assert os.environ["CUDA_DEVICE_ORDER"] == "PCI_BUS_ID"
    assert os.environ["PYOPENGL_PLATFORM"] == "egl"
    assert os.environ["EGL_DEVICE_ID"] == "2"


def test_viewer_does_not_force_headless_egl(monkeypatch) -> None:
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "0")
    monkeypatch.delenv("PYOPENGL_PLATFORM", raising=False)
    monkeypatch.delenv("EGL_DEVICE_ID", raising=False)

    _configure_gpu_render_environment(use_gpu=True, viewer=True)

    assert "PYOPENGL_PLATFORM" not in os.environ
    assert "EGL_DEVICE_ID" not in os.environ


def test_macos_gpu_render_leaves_egl_unset(monkeypatch) -> None:
    monkeypatch.setattr(sys, "platform", "darwin")
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "2")
    monkeypatch.delenv("CUDA_DEVICE_ORDER", raising=False)
    monkeypatch.delenv("PYOPENGL_PLATFORM", raising=False)
    monkeypatch.delenv("EGL_DEVICE_ID", raising=False)

    _configure_gpu_render_environment(use_gpu=True, viewer=False)

    assert "PYOPENGL_PLATFORM" not in os.environ
    assert "EGL_DEVICE_ID" not in os.environ
    assert "CUDA_DEVICE_ORDER" not in os.environ


def test_gpu_genesis_init_enables_performance_mode(monkeypatch) -> None:
    captured = {}

    class InitObserved(Exception):
        pass

    def observe_init(**kwargs) -> None:
        captured.update(kwargs)
        raise InitObserved

    monkeypatch.setattr(runtime.gs, "init", observe_init)
    app = SimpleNamespace(cfg=SimpleNamespace(use_go2=False, use_gpu=True))

    with pytest.raises(InitObserved):
        runtime.RuntimePrep(app).init_genesis("")

    assert captured["backend"] is runtime.gs.gpu
    assert captured["logging_level"] == "warning"
    assert captured["performance_mode"] is True


def test_viewer_flag_overrides_remote_profile(monkeypatch) -> None:
    captured = {}

    class FakeGenesisApp:
        def __init__(self, **kwargs):
            captured.update(kwargs)

        def run(self) -> None:
            return None

    monkeypatch.setattr(runtime, "GenesisApp", FakeGenesisApp)
    runtime.run_runtime(
        config_path=str(ROOT / "config/config.yaml"),
        config_mode="remote",
        argv=["--viewer"],
    )

    remote = load_app_config(str(ROOT / "config/config.yaml"), mode="remote")
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
        config_path=str(ROOT / "config/config.yaml"),
        config_mode="remote",
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
