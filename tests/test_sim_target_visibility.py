"""Tests for sim eye-camera target projection."""

from __future__ import annotations

import unittest

import numpy as np

from engine.experiment.sim_target_visibility import project_world_point


class TestSimTargetVisibility(unittest.TestCase):
    def test_centered_target_in_frame(self) -> None:
        proj = project_world_point(
            object_world=(0.0, 0.0, 2.0),
            camera_origin=(0.0, 0.0, 0.0),
            camera_look=(0.0, 0.0, 1.0),
            camera_right=(1.0, 0.0, 0.0),
            width=640,
            height=480,
            fov_deg=60.0,
        )
        self.assertTrue(proj.in_frame)
        self.assertAlmostEqual(proj.u_norm, 0.0, places=2)
        self.assertAlmostEqual(proj.v_norm, 0.0, places=2)

    def test_behind_camera_not_visible(self) -> None:
        proj = project_world_point(
            object_world=(0.0, 0.0, -1.0),
            camera_origin=(0.0, 0.0, 0.0),
            camera_look=(0.0, 0.0, 1.0),
            camera_right=(1.0, 0.0, 0.0),
            width=640,
            height=480,
            fov_deg=60.0,
        )
        self.assertFalse(proj.in_frame)

    def test_off_frame_lateral(self) -> None:
        proj = project_world_point(
            object_world=(5.0, 0.0, 1.0),
            camera_origin=(0.0, 0.0, 0.0),
            camera_look=(0.0, 0.0, 1.0),
            camera_right=(1.0, 0.0, 0.0),
            width=640,
            height=480,
            fov_deg=60.0,
            margin_px=0.0,
        )
        self.assertFalse(proj.in_frame)
        self.assertGreater(abs(proj.u_norm), 0.5)


if __name__ == "__main__":
    unittest.main()
