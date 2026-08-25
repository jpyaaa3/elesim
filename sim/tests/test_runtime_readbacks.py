from __future__ import annotations

import numpy as np

from elesim_sim.config import JointLimit, SimParam
from elesim_sim.config import SimConfig
from elesim_sim.runtime import GenesisApp, JointLayout, SimMover, SimRuntime, SimScene


class _Joint:
    def __init__(self, index: int) -> None:
        self.dofs_idx_local = index


class _Link:
    def __init__(self, index: int = 0) -> None:
        self.idx_local = index
        self.pos_reads = 0
        self.quat_reads = 0

    def get_pos(self) -> np.ndarray:
        self.pos_reads += 1
        return np.array([1.0, 2.0, 3.0])

    def get_quat(self) -> np.ndarray:
        self.quat_reads += 1
        return np.array([1.0, 0.0, 0.0, 0.0])


class _Entity:
    def __init__(self) -> None:
        names = (
            "linear",
            "roll",
            "bend",
            "j_gripper_base_claw_left",
            "j_gripper_base_claw_right",
        )
        self.joints = {name: _Joint(index) for index, name in enumerate(names)}
        self.link = _Link()
        self.position_writes: list[tuple[np.ndarray, list[int]]] = []
        self.bulk_pos_reads = 0
        self.bulk_quat_reads = 0

    def get_joint(self, name: str) -> _Joint:
        return self.joints[name]

    def get_link(self, _name: str) -> _Link:
        return self.link

    def get_links_pos(self, *, links_idx_local: list[int]) -> np.ndarray:
        self.bulk_pos_reads += 1
        return np.tile(np.array([[1.0, 2.0, 3.0]]), (len(links_idx_local), 1))

    def get_links_quat(self, *, links_idx_local: list[int]) -> np.ndarray:
        self.bulk_quat_reads += 1
        return np.tile(np.array([[1.0, 0.0, 0.0, 0.0]]), (len(links_idx_local), 1))

    def get_dofs_position(self, *, dofs_idx_local: list[int]) -> np.ndarray:
        return np.zeros(len(dofs_idx_local), dtype=float)

    def set_dofs_position(self, values: np.ndarray, *, dofs_idx_local: list[int]) -> None:
        self.position_writes.append((np.asarray(values), list(dofs_idx_local)))


def test_sim_mover_batches_arm_and_claw_position_write() -> None:
    entity = _Entity()
    mover = SimMover(
        entity,
        SimParam(dt=0.02),
        JointLimit(-90.0, 90.0, 90.0),
        1,
        linear_joint_name="linear",
        roll_joint_name="roll",
        bend_joint_names=["bend"],
    )
    entity.position_writes.clear()

    mover.set_claw_closed(True)
    mover.control_4dof(-0.1, 0.2, 0.3, 0.3)

    assert len(entity.position_writes) == 1
    values, indices = entity.position_writes[0]
    assert len(values) == len(indices) == 5


def test_tip_position_and_direction_share_one_link_pose_readback() -> None:
    entity = _Entity()
    scene = SimScene(mover=type("Mover", (), {"entity": entity})())
    layout = JointLayout(
        tip_link_name="tip",
        tip_local_offset=np.zeros(3),
        approach_axis_local=np.array([0.0, 0.0, -1.0]),
    )

    tip, direction = scene.actual_tip_pose_world(layout)

    assert np.array_equal(tip, np.array([1.0, 2.0, 3.0]))
    assert np.array_equal(direction, np.array([0.0, 0.0, -1.0]))
    assert entity.bulk_pos_reads == 1
    assert entity.bulk_quat_reads == 1
    assert entity.link.pos_reads == 0
    assert entity.link.quat_reads == 0


def test_feedback_readback_is_decimated_in_simulation_time() -> None:
    runtime = SimRuntime(GenesisApp(cfg=SimConfig(telemetry_max_hz=20.0)))

    assert runtime._feedback_due(0.00) is True
    assert runtime._feedback_due(0.02) is False
    assert runtime._feedback_due(0.04) is False
    assert runtime._feedback_due(0.06) is True


def test_tip_readback_does_not_bypass_telemetry_cadence() -> None:
    class Scene:
        calls = 0

        def actual_tip_pose_world(self, _layout: object) -> tuple[np.ndarray, np.ndarray]:
            self.calls += 1
            return np.zeros(3), np.ones(3)

    app = GenesisApp(cfg=SimConfig())
    app.sim_scene = Scene()
    app.layout = object()
    runtime = SimRuntime(app)
    runtime._perf = type("Perf", (), {"section": lambda *_args: None})()

    runtime._refresh_tip_feedback_cache(False)
    assert app.sim_scene.calls == 0

    runtime._refresh_tip_feedback_cache(True)
    assert app.sim_scene.calls == 1


def test_camera_pose_readback_does_not_bypass_feedback_cadence() -> None:
    class Scene:
        eye_camera = object()
        calls = 0

        def camera_axes_world(self, *, hand_eye_path: str):
            assert hand_eye_path == "hand-eye.json"
            self.calls += 1
            return (
                np.zeros(3),
                np.array([0.0, 0.0, 0.08]),
                np.array([0.08, 0.0, 0.0]),
            )

    app = GenesisApp(cfg=SimConfig(hand_eye_config="hand-eye.json"))
    app.sim_scene = Scene()
    runtime = SimRuntime(app)

    assert runtime._camera_axes_for_feedback(False) is None
    assert app.sim_scene.calls == 0

    axes = runtime._camera_axes_for_feedback(True)
    assert axes is not None
    assert app.sim_scene.calls == 1


def test_headless_scene_step_skips_redundant_visualizer_sync() -> None:
    class Scene:
        kwargs: dict[str, bool] | None = None

        def step(self, **kwargs: bool) -> None:
            self.kwargs = kwargs

    genesis_scene = Scene()
    scene = SimScene(scene=genesis_scene, update_visualizer_on_step=False)

    scene.step()

    assert genesis_scene.kwargs == {
        "update_visualizer": False,
        "refresh_visualizer": False,
    }
    assert scene.sim_step_count == 1


def test_viewer_scene_step_keeps_visualizer_sync() -> None:
    class Scene:
        kwargs: dict[str, bool] | None = None

        def step(self, **kwargs: bool) -> None:
            self.kwargs = kwargs

    genesis_scene = Scene()
    scene = SimScene(scene=genesis_scene, update_visualizer_on_step=True)

    scene.step()

    assert genesis_scene.kwargs == {
        "update_visualizer": True,
        "refresh_visualizer": True,
    }
