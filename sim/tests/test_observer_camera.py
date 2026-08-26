from __future__ import annotations

import numpy as np
import pytest

from elesim_sim.vision.sim_camera.mount import ObserverCamera
from elesim_sim.vision.sim_camera.types import SimCameraIntrinsics


class Camera:
    def __init__(self) -> None:
        self.poses: list[
            tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]
        ] = []

    def set_pose(self, *, pos, lookat, up) -> None:
        self.poses.append((tuple(pos), tuple(lookat), tuple(up)))

    def render(self, *, rgb, depth, force_render=False):
        self.force_render = bool(force_render)
        return (
            np.zeros((48, 64, 3), dtype=np.uint8) if rgb else None,
            np.zeros((48, 64), dtype=np.float32) if depth else None,
            None,
            None,
        )


def observer() -> ObserverCamera:
    return ObserverCamera(
        camera=Camera(),
        intrinsics=SimCameraIntrinsics(100.0, 100.0, 32.0, 24.0, 64, 48),
        pos=(0.0, -2.0, 1.0),
        lookat=(0.0, 0.0, 0.0),
    )


def test_primary_drag_pans_and_tilts_without_moving_or_rolling_the_camera() -> None:
    value = observer()
    original_eye = np.asarray(value.pos)
    original_target = np.asarray(value.lookat)
    original_radius = np.linalg.norm(np.asarray(value.pos) - original_target)

    value.apply_operator_command("orbit", {"dx": 0.2, "dy": -0.1})

    np.testing.assert_allclose(value.pos, original_eye)
    assert not np.allclose(value.lookat, original_target)
    assert value.lookat[0] > original_target[0]
    assert value.lookat[2] > original_target[2]
    assert np.linalg.norm(np.asarray(value.lookat) - original_eye) == pytest.approx(
        original_radius
    )
    forward = np.asarray(value.lookat) - np.asarray(value.pos)
    right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
    assert right[2] == pytest.approx(0.0)
    assert value.camera.poses[-1][2] == (0.0, 0.0, 1.0)


def test_zoom_changes_only_the_camera_target_distance() -> None:
    value = observer()
    original_target = np.asarray(value.lookat)
    original_radius = np.linalg.norm(np.asarray(value.pos) - original_target)

    value.apply_operator_command("zoom", {"delta": -0.2})

    assert np.linalg.norm(np.asarray(value.pos) - np.asarray(value.lookat)) < original_radius


def test_pan_moves_eye_and_target_together_and_reset_restores_initial_pose() -> None:
    value = observer()
    initial_eye = np.asarray(value.pos)
    initial_target = np.asarray(value.lookat)
    initial_offset = initial_eye - initial_target

    value.apply_operator_command("pan", {"dx": 0.1, "dy": -0.2})

    assert not np.allclose(value.pos, initial_eye)
    np.testing.assert_allclose(np.asarray(value.pos) - np.asarray(value.lookat), initial_offset)

    value.apply_operator_command("reset_view", {})

    np.testing.assert_allclose(value.pos, initial_eye)
    np.testing.assert_allclose(value.lookat, initial_target)


def test_capture_forces_genesis_render_after_pose_update() -> None:
    value = observer()
    value.capture(force_render=True)
    assert value.camera.force_render is True
