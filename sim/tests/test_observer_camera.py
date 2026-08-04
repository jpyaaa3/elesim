from __future__ import annotations

import numpy as np
import pytest

from elesim_sim.vision.sim_camera.mount import ObserverCamera
from elesim_sim.vision.sim_camera.types import SimCameraIntrinsics


class Camera:
    def __init__(self) -> None:
        self.poses: list[tuple[tuple[float, ...], tuple[float, ...]]] = []

    def set_pose(self, *, pos, lookat) -> None:
        self.poses.append((tuple(pos), tuple(lookat)))


def observer() -> ObserverCamera:
    return ObserverCamera(
        camera=Camera(),
        intrinsics=SimCameraIntrinsics(100.0, 100.0, 32.0, 24.0, 64, 48),
        pos=(0.0, -2.0, 1.0),
        lookat=(0.0, 0.0, 0.0),
    )


def test_orbit_preserves_radius_and_zoom_changes_only_radius() -> None:
    value = observer()
    original_target = np.asarray(value.lookat)
    original_radius = np.linalg.norm(np.asarray(value.pos) - original_target)

    value.apply_operator_command("orbit", {"dx": 0.2, "dy": -0.1})

    np.testing.assert_allclose(value.lookat, original_target)
    assert np.linalg.norm(np.asarray(value.pos) - original_target) == pytest.approx(
        original_radius
    )

    value.apply_operator_command("zoom", {"delta": -0.2})

    assert np.linalg.norm(np.asarray(value.pos) - original_target) < original_radius


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
