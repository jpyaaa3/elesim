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
    _genesis_init_kwargs,
    resolve_single_dof_indices,
)
from elesim_sim.vision.sim_camera.mount import (
    Node9EyeInHandCamera,
    ObserverCamera,
    ObserverViewState,
)
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
        robot_joint_positions=(0.0, 1.0),
        observer_pos=(1.0, 2.0, 3.0),
        observer_lookat=(0.0, 0.0, 0.0),
        target_position=(0.5, 0.0, 0.2),
    )
    assert snapshot.epoch == 3
    assert snapshot.sim_step == 12
    assert snapshot.robot_joint_positions == (0.0, 1.0)
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
        robot_dof_indices=(),
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


def test_render_replica_maps_floating_source_joints_by_name() -> None:
    """A fixed visual replica must not reuse floating-source DOF offsets."""

    class Joint:
        def __init__(self, index: int) -> None:
            self.dofs_idx_local = index

    class Entity:
        # The source's floating base would put these joints at (7, 9), while
        # this fixed visual replica has a compact, independently built layout.
        joints = {"arm_shoulder": Joint(2), "arm_elbow": Joint(5)}

        def __init__(self) -> None:
            self.position_writes = []

        def get_joint(self, name: str) -> Joint:
            return self.joints[name]

        def set_dofs_position(self, values, *, dofs_idx_local) -> None:
            self.position_writes.append(
                (tuple(float(value) for value in values), tuple(dofs_idx_local))
            )

    entity = Entity()
    # The source/floating-base positions would be at (7, 9). Resolve the
    # independently built fixed replica by names instead of reusing them.
    visual_dof_indices = resolve_single_dof_indices(
        entity, ("arm_shoulder", "arm_elbow")
    )
    changed = _apply_snapshot(
        entity=entity,
        robot_dof_indices=visual_dof_indices,
        mock_entities={},
        target_entity=None,
        observer=None,
        snapshot=CameraStateSnapshot(
            epoch=0,
            sim_step=1,
            sim_time_s=0.02,
            robot_joint_positions=(0.25, -0.5),
        ),
    )

    assert changed is False
    assert entity.position_writes == [((0.25, -0.5), (2, 5))]


def test_render_replica_fails_visibly_for_an_unmapped_source_joint() -> None:
    class Joint:
        dofs_idx_local = 1

    class Entity:
        def get_joint(self, name: str) -> Joint:
            if name == "arm_shoulder":
                return Joint()
            raise KeyError(name)

        def set_dofs_position(self, *_args, **_kwargs) -> None:
            raise AssertionError("an incomplete joint map must not be applied")

    with pytest.raises(KeyError, match="arm_elbow"):
        resolve_single_dof_indices(Entity(), ("arm_shoulder", "arm_elbow"))


def test_render_replica_rejects_a_joint_position_count_mismatch() -> None:
    class Entity:
        def set_dofs_position(self, *_args, **_kwargs) -> None:
            raise AssertionError("a mismatched snapshot must not be applied")

    with pytest.raises(RuntimeError, match="size mismatch"):
        _apply_snapshot(
            entity=Entity(),
            robot_dof_indices=(2, 5),
            mock_entities={},
            target_entity=None,
            observer=None,
            snapshot=CameraStateSnapshot(
                epoch=0,
                sim_step=1,
                sim_time_s=0.02,
                robot_joint_positions=(0.25,),
            ),
        )


def test_observer_capture_forces_refresh_after_robot_state_snapshot() -> None:
    """Robot motion must force a fresh observer render even at a fixed pose."""

    class Camera:
        def __init__(self) -> None:
            self.render_calls = []

        def render(self, **kwargs):
            self.render_calls.append(kwargs)
            return np.ones((2, 2, 3), dtype=np.float32), None, None, None

    class Entity:
        def __init__(self) -> None:
            self.position_writes = []

        def set_dofs_position(self, values, *, dofs_idx_local) -> None:
            self.position_writes.append(
                (tuple(float(value) for value in values), tuple(dofs_idx_local))
            )

    camera = Camera()
    observer = ObserverCamera(
        camera=camera,
        intrinsics=SimCameraIntrinsics(100.0, 100.0, 1.0, 1.0, 2, 2),
        pos=(3.5, 0.5, 2.5),
        lookat=(0.0, 0.0, 0.5),
    )
    entity = Entity()
    for step, position in ((1, 0.1), (2, 0.7)):
        _apply_snapshot(
            entity=entity,
            robot_dof_indices=(0,),
            mock_entities={},
            target_entity=None,
            observer=observer,
            snapshot=CameraStateSnapshot(
                epoch=0,
                sim_step=step,
                sim_time_s=0.02 * step,
                robot_joint_positions=(position,),
                observer_pos=observer.pos,
                observer_lookat=observer.lookat,
            ),
        )
        observer.capture(
            rgb_enabled=True,
            depth_enabled=False,
            prefer_gpu=False,
            force_render=True,
        )

    assert entity.position_writes == [((0.1,), (0,)), ((0.7,), (0,))]
    assert [call["force_render"] for call in camera.render_calls] == [True, True]


def test_hand_eye_capture_propagates_force_render() -> None:
    class Camera:
        def __init__(self) -> None:
            self.render_calls = []
            self.events = []

        def move_to_attach(self) -> None:
            self.events.append("move")

        def render(self, **kwargs):
            self.events.append("render")
            self.render_calls.append(kwargs)
            return np.ones((2, 2, 3), dtype=np.float32), None, None, None

    camera = Camera()
    eye = Node9EyeInHandCamera(
        camera=camera,
        intrinsics=SimCameraIntrinsics(100.0, 100.0, 1.0, 1.0, 2, 2),
    )
    eye.capture(
        rgb_enabled=True,
        depth_enabled=False,
        prefer_gpu=False,
        force_render=True,
    )

    assert camera.events[:2] == ["move", "render"]
    assert camera.render_calls == [
        {"rgb": True, "depth": False, "force_render": True}
    ]


def test_visual_worker_avoids_static_performance_compile_on_gpu() -> None:
    fake_genesis = type("Genesis", (), {"gpu": object(), "cpu": object()})()

    kwargs = _genesis_init_kwargs(fake_genesis, use_gpu=True)

    assert kwargs == {"backend": fake_genesis.gpu, "logging_level": "warning"}
    assert "performance_mode" not in kwargs


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
    scene = SimScene(
        hand_eye_enabled=True,
        observer_enabled=True,
        observer_view=ObserverViewState.create(
            res=(64, 48),
            pos=(1.0, 2.0, 3.0),
            lookat=(0.0, 0.0, 0.0),
        ),
        camera_render_worker=worker,
    )
    assert scene.eye_camera is None
    assert scene.observer_camera is None
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


def test_async_hand_eye_dispatch_publishes_rgbd_without_a_physics_camera() -> None:
    class Publisher:
        def __init__(self) -> None:
            self.frames = []

        def publish(self, frame) -> None:
            self.frames.append(frame)

    frame = object()
    publisher = Publisher()
    scene = SimScene(
        hand_eye_enabled=True,
        camera_publisher=publisher,
    )

    scene._on_async_frame("hand_eye_preview", frame)

    assert scene.eye_camera is None
    assert publisher.frames == [frame]


def test_async_observer_commands_update_snapshot_without_a_physics_camera() -> None:
    view = ObserverViewState.create(
        res=(640, 480),
        pos=(0.0, -2.0, 1.0),
        lookat=(0.0, 0.0, 0.0),
    )
    scene = SimScene(observer_enabled=True, observer_view=view)
    original = (view.pos, view.lookat)

    view.apply_operator_command("pan", {"dx": 0.1, "dy": -0.2})
    snapshot = scene._camera_state_snapshot(arm_q=None, sim_time_s=0.0)

    assert scene.observer_camera is None
    assert (view.pos, view.lookat) != original
    assert snapshot.observer_pos == view.pos
    assert snapshot.observer_lookat == view.lookat


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
