from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from engine.vision.sim_camera.mount import hand_eye_to_genesis_attach_T, load_hand_eye_offset_T, _OPTICAL_FROM_GENESIS_CAMERA
from engine.vision.sim_camera.pose import _link_world_transform, camera_axes_from_genesis_camera_object

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "host.py").exists())


class _FakeTensor:
    def __init__(self, data):
        self._data = np.asarray(data, dtype=float)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self._data


class _FakeLink:
    def __init__(self, pos, quat_wxyz):
        self._pos = _FakeTensor(pos)
        self._quat = _FakeTensor(quat_wxyz)

    def get_pos(self):
        return self._pos

    def get_quat(self):
        return self._quat


class TestSimCameraMount(unittest.TestCase):
    def test_hand_eye_optical_axes_in_node9(self) -> None:
        """
        ZED Mini mounted rolled 180 deg about the optical axis.

        Look direction is unchanged from the old D435 mount (node9 +X), but the
        roll flips right and down: optical +X -> node9 +Y, optical +Y -> node9
        +Z. Both signs are opposite the D435 values this test used to pin.
        """
        cfg = ROOT / "model_presets" / "visual_servoing" / "hand_eye.camera.json"
        T = load_hand_eye_offset_T(cfg)
        R = T[:3, :3]
        np.testing.assert_allclose(R[:, 2], [1.0, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(R[:, 0], [0.0, 1.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(R[:, 1], [0.0, 0.0, 1.0], atol=1e-6)
        np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-9)

    def test_genesis_camera_object_matches_link_attach(self) -> None:
        cfg = ROOT / "model_presets" / "visual_servoing" / "hand_eye.camera.json"
        link = _FakeLink([0.2, 0.0, 0.5], [1.0, 0.0, 0.0, 0.0])
        T_link = _link_world_transform(link)
        T_attach = hand_eye_to_genesis_attach_T(cfg)
        T_wg = T_link @ T_attach

        class _FakeCam:
            def get_transform(self):
                return T_wg

            def move_to_attach(self):
                return None

        origin, look, right = camera_axes_from_genesis_camera_object(_FakeCam(), axis_len_m=0.1)
        T_opt = T_wg @ _OPTICAL_FROM_GENESIS_CAMERA
        np.testing.assert_allclose(origin, T_opt[:3, 3], atol=1e-6)
        np.testing.assert_allclose(look, T_opt[:3, :3] @ [0.0, 0.0, 0.1], atol=1e-6)
        np.testing.assert_allclose(right, T_opt[:3, :3] @ [0.1, 0.0, 0.0], atol=1e-6)


if __name__ == "__main__":
    unittest.main()
