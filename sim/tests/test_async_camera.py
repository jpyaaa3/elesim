from __future__ import annotations

import multiprocessing as mp
import os
from pathlib import Path
import threading

import numpy as np
import pytest

from elesim_sim.runtime import SimScene
from elesim_sim.vision.sim_camera.async_worker import (
    CameraRenderSpec,
    CameraStateSnapshot,
    CameraRenderWorker,
    SharedRgbdMailbox,
    _apply_snapshot,
)
from elesim_sim.vision.sim_camera.mount import ObserverCamera
from elesim_sim.vision.sim_camera.types import SimCameraIntrinsics


def test_shared_rgbd_mailbox_is_latest_only_and_coherent() -> None:
    mailbox = SharedRgbdMailbox.create(mp.get_context("fork"), width=4, height=3)
    first_color = np.full((3, 4, 3), 7, dtype=np.uint8)
    first_depth = np.full((3, 4), 700, dtype=np.uint16)
    second_color = np.full((3, 4, 3), 9, dtype=np.uint8)
    second_depth = np.full((3, 4), 900, dtype=np.uint16)

    assert mailbox.publish(first_color, first_depth, captured_at=1.0) == 1
    assert mailbox.publish(second_color, second_depth, captured_at=2.0) == 2

    color, depth, sequence, captured_at = mailbox.latest()
    assert sequence == 2
    assert captured_at == 2.0
    np.testing.assert_array_equal(color, second_color)
    np.testing.assert_array_equal(depth, second_depth)


def test_render_snapshot_is_serializable_and_has_epoch_fields() -> None:
    snapshot = CameraStateSnapshot(
        epoch=3,
        sim_step=12,
        sim_time_s=0.24,
        arm_q=(1.0, 2.0, 3.0, 4.0),
        robot_q=(0.0, 1.0),
        robot_q_indices=(0, 1),
        observer_pos=(1.0, 2.0, 3.0),
        observer_lookat=(0.0, 0.0, 0.0),
        target_position=(0.5, 0.0, 0.2),
    )
    assert snapshot.epoch == 3
    assert snapshot.sim_step == 12
    assert snapshot.robot_q_indices == (0, 1)
    assert snapshot.target_position == (0.5, 0.0, 0.2)


def test_render_replica_applies_observer_pose_snapshot() -> None:
    class Camera:
        def __init__(self) -> None:
            self.poses = []

        def set_pose(self, *, pos, lookat, up):
            self.poses.append((tuple(pos), tuple(lookat), tuple(up)))

    camera = Camera()
    observer = ObserverCamera(
        camera=camera,
        intrinsics=SimCameraIntrinsics(100.0, 100.0, 32.0, 24.0, 64, 48),
        pos=(0.0, -2.0, 1.0),
        lookat=(0.0, 0.0, 0.0),
    )
    changed = _apply_snapshot(
        entity=object(),
        mock_entities={},
        target_entity=None,
        observer=observer,
        snapshot=CameraStateSnapshot(
            epoch=0,
            sim_step=1,
            sim_time_s=0.02,
            observer_pos=(1.0, -2.0, 1.0),
            observer_lookat=(1.0, 0.0, 0.0),
        ),
    )

    assert changed is True
    assert observer.pos == (1.0, -2.0, 1.0)
    assert observer.lookat == (1.0, 0.0, 0.0)
    assert camera.poses[-1] == (
        (1.0, -2.0, 1.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0),
    )


def test_render_spec_rejects_missing_urdf() -> None:
    with pytest.raises(ValueError, match="URDF"):
        CameraRenderSpec(
            urdf_path="",
            robot_pos=(0.0, 0.0, 0.0),
            robot_euler_deg=(0.0, 0.0, 0.0),
            requires_jac_and_ik=False,
            use_gpu=False,
            gpu_convert=False,
            dt=0.02,
            gravity=(0.0, 0.0, -9.81),
            substeps=1,
            floor=True,
        )


def test_async_camera_submission_does_not_call_scene_camera() -> None:
    class Camera:
        pos = (1.0, 2.0, 3.0)
        lookat = (0.0, 0.0, 0.0)

        def capture(self, **_kwargs):  # pragma: no cover - must not run
            raise AssertionError("async mode must not capture on the physics thread")

    class Worker:
        ready = True

        def __init__(self) -> None:
            self.submitted = []
            self.epochs = 0
            self.closed = False

        def submit(self, snapshot, streams):
            self.submitted.append((snapshot, streams))
            return True

        def bump_epoch(self):
            self.epochs += 1

        def close(self):
            self.closed = True

    worker = Worker()
    scene = SimScene(eye_camera=Camera(), observer_camera=Camera(), camera_render_worker=worker)
    scene.sim_target_xyz = np.array([0.8, 0.0, 0.2], dtype=float)

    scene.maybe_publish_camera(
        arm_q=(0.0, 0.0, 0.0, 0.0),
        max_hz=30.0,
        sim_time_s=0.0,
        force=True,
        depth_enabled=True,
    )
    scene.maybe_publish_observer_camera(
        max_hz=20.0,
        sim_time_s=0.0,
        force=True,
    )

    assert [entry[1] for entry in worker.submitted] == [
        ("hand_eye_preview",),
        ("observer",),
    ]
    assert all(entry[0].epoch == 0 for entry in worker.submitted)
    assert all(entry[0].target_position == (0.8, 0.0, 0.2) for entry in worker.submitted)
    scene.reset_environment()
    assert worker.epochs == 1
    scene.close_frame_dispatchers()
    assert worker.closed is True


def test_visualizer_refresh_is_rate_limited_when_legacy_step_refresh_is_off(monkeypatch) -> None:
    class Visualizer:
        def __init__(self) -> None:
            self.calls = []

        def update(self, **kwargs):
            self.calls.append(kwargs)

    class Scene:
        visualizer = Visualizer()

        def step(self, **_kwargs):
            return None

    clock = [100.0]
    monkeypatch.setattr("elesim_sim.runtime.time.monotonic", lambda: clock[0])
    genesis_scene = Scene()
    scene = SimScene(
        scene=genesis_scene,
        update_visualizer_on_step=False,
        visualizer_max_hz=10.0,
    )

    scene.step()
    assert genesis_scene.visualizer.calls == [{"force": False, "auto": True}]

    clock[0] = 100.01
    scene.step()
    assert len(genesis_scene.visualizer.calls) == 1

    clock[0] = 100.11
    scene.step()
    assert len(genesis_scene.visualizer.calls) == 2


@pytest.mark.skipif(
    os.environ.get("ELESIM_RUN_ASYNC_CAMERA_INTEGRATION") != "1",
    reason="real Genesis camera worker smoke is opt-in",
)
def test_real_async_camera_worker_delivers_latest_observer_frame() -> None:
    """Exercise the process boundary with a small visual-only arm scene."""

    urdf = Path(__file__).parents[2] / "model" / "bundles" / "default" / "arm.urdf"
    assert urdf.is_file()
    received: list[object] = []
    ready = threading.Event()

    def on_frame(_stream: str, frame: object) -> None:
        received.append(frame)
        ready.set()

    worker = CameraRenderWorker(
        CameraRenderSpec(
            urdf_path=str(urdf),
            robot_pos=(0.0, 0.0, 0.0),
            robot_euler_deg=(0.0, 0.0, 0.0),
            requires_jac_and_ik=False,
            use_gpu=False,
            gpu_convert=False,
            dt=0.02,
            gravity=(0.0, 0.0, 0.0),
            substeps=1,
            floor=False,
            observer_width=64,
            observer_height=48,
        ),
        {"observer": (64, 48)},
        on_frame,
    )
    try:
        worker.start(timeout_s=120.0)
        assert worker.submit(
            CameraStateSnapshot(
                epoch=0,
                sim_step=1,
                sim_time_s=0.02,
                observer_pos=(1.0, 1.0, 1.0),
                observer_lookat=(0.0, 0.0, 0.0),
            ),
            ("observer",),
        )
        assert ready.wait(timeout=60.0), worker.diagnostics()
        frame = received[-1]
        assert frame.color_bgr.shape == (48, 64, 3)
        assert frame.depth_raw.shape == (48, 64)
        assert worker.diagnostics()["completed"]["observer"] >= 1
    finally:
        worker.close()
