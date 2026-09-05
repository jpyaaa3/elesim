from __future__ import annotations

import os
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import elesim_sim.runtime as runtime
from elesim_sim.config import load_app_config
from elesim_sim.main import _configure_gpu_render_environment


REPO_ROOT = next(parent for parent in Path(__file__).resolve().parents if (parent / "payload").is_dir())
CONFIG_ROOT = REPO_ROOT / "payload" / "config" / "sim"


def test_headless_gpu_render_remaps_single_selected_device_for_egl(monkeypatch) -> None:
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "2")
    monkeypatch.delenv("CUDA_DEVICE_ORDER", raising=False)
    monkeypatch.delenv("PYOPENGL_PLATFORM", raising=False)
    monkeypatch.delenv("EGL_DEVICE_ID", raising=False)

    _configure_gpu_render_environment(use_gpu=True, viewer=False)

    assert os.environ["CUDA_DEVICE_ORDER"] == "PCI_BUS_ID"
    assert os.environ["PYOPENGL_PLATFORM"] == "egl"
    assert os.environ["EGL_DEVICE_ID"] == "0"


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


def test_convex_mpc_physics_morph_skips_genesis_ik_without_merging_links(
    monkeypatch,
) -> None:
    captured = {}

    class Morphs:
        @staticmethod
        def URDF(**kwargs):
            captured.update(kwargs)
            return object()

    monkeypatch.setattr(runtime, "gs", SimpleNamespace(morphs=Morphs()))

    runtime._make_urdf_morph(
        "/model/robot.urdf",
        (0.0, 0.0, 0.32),
        (0.0, 0.0, 0.0),
        fixed=False,
        requires_jac_and_IK=False,
        merge_fixed_links=False,
    )

    assert captured["requires_jac_and_IK"] is False
    assert captured["merge_fixed_links"] is False


def test_only_legacy_raibert_controller_requires_genesis_ik() -> None:
    convex = runtime.Go2LocomotionConfig(mode="convex_mpc")
    raibert = runtime.Go2LocomotionConfig()
    mirror = runtime.Go2LocomotionConfig(mirror_from_host=True)

    assert runtime._requires_genesis_ik(convex) is False
    assert runtime._requires_genesis_ik(raibert) is True
    assert runtime._requires_genesis_ik(mirror) is False


def test_async_runtime_builds_physics_before_starting_visual_worker(monkeypatch) -> None:
    captured = {"order": []}

    class StopAfterInit(Exception):
        pass

    class FakeAssetProcessor:
        def __init__(self, _app) -> None:
            pass

        def prepare_assets(self) -> str:
            return "/model/robot.urdf"

    class FakeRuntimePrep:
        def __init__(self, _app) -> None:
            pass

        def camera_render_spec(self, urdf_path: str):
            assert urdf_path == "/model/robot.urdf"
            return object()

        def init_genesis(self, urdf_path: str, *, attach_scene_cameras: bool) -> None:
            captured["urdf_path"] = urdf_path
            captured["attach_scene_cameras"] = attach_scene_cameras
            captured["order"].append("physics")

    class Scene:
        camera_render_worker = None

        def configure_camera_render_worker(self, *_args, **_kwargs) -> None:
            captured["worker_started"] = True
            captured["order"].append("visual")
            raise StopAfterInit

        def close_frame_dispatchers(self) -> None:
            captured["closed"] = True

        def close_camera_publishers(self) -> None:
            captured["publishers_closed"] = True

    app = SimpleNamespace(
        cfg=SimpleNamespace(
            camera_execution="async_process",
            sim_camera_enable=True,
            hand_eye_config="/config/hand-eye.json",
            sim_observer_camera_enable=True,
            sim_camera_width=640,
            sim_camera_height=480,
            sim_observer_camera_width=640,
            sim_observer_camera_height=480,
            camera_worker_start_timeout_s=180.0,
        ),
        sim_scene=Scene(),
    )
    monkeypatch.setattr(runtime, "AssetProcessor", FakeAssetProcessor)
    monkeypatch.setattr(runtime, "RuntimePrep", FakeRuntimePrep)

    with pytest.raises(StopAfterInit):
        runtime.GenesisApp.run(app)

    assert captured == {
        "worker_started": True,
        "urdf_path": "/model/robot.urdf",
        "attach_scene_cameras": False,
        "order": ["physics", "visual"],
        "closed": True,
        "publishers_closed": True,
    }


def test_viewer_flag_overrides_remote_profile(monkeypatch) -> None:
    captured = {}

    class FakeGenesisApp:
        def __init__(self, **kwargs):
            captured.update(kwargs)

        def run(self) -> None:
            return None

    monkeypatch.setattr(runtime, "GenesisApp", FakeGenesisApp)
    runtime.run_runtime(
        config_path=str(CONFIG_ROOT / "config.yaml"),
        config_mode="remote",
        argv=["--viewer"],
    )

    remote = load_app_config(str(CONFIG_ROOT / "config.yaml"), mode="remote")
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
        config_path=str(CONFIG_ROOT / "config.yaml"),
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
