"""Tests for pick extend readiness gates."""

from __future__ import annotations

import unittest

from engine.config_loader import PickConfig
from engine.controller.object_pick import pick_ready_for_extend
from engine.controller.perception import VisualObservation


class PickExtendReadyTests(unittest.TestCase):
    def test_aligned_requires_center_and_scale(self) -> None:
        cfg = PickConfig(
            target_scale=0.16,
            scale_tol=0.02,
            center_tol=0.06,
            target_uv_u=0.5,
            target_uv_v=-0.5,
        )
        obs = VisualObservation(
            label="ball",
            confidence=0.9,
            center_uv=(0.51, -0.51),
            scale=0.15,
            timestamp_s=1.0,
        )
        ready, reason = pick_ready_for_extend(obs, cfg=cfg)
        self.assertTrue(ready)
        self.assertEqual(reason, "aligned")

    def test_many_approach_steps_not_enough(self) -> None:
        cfg = PickConfig(target_scale=0.16, scale_tol=0.02, center_tol=0.06)
        obs = VisualObservation(
            label="ball",
            confidence=0.9,
            center_uv=(0.58, -0.54),
            scale=0.129,
            timestamp_s=1.0,
        )
        ready, _ = pick_ready_for_extend(
            obs, cfg=cfg, approach_steps=100, scale_plateau=False
        )
        self.assertFalse(ready)

    def test_scale_plateau_at_target_with_center(self) -> None:
        cfg = PickConfig(
            target_scale=0.16,
            scale_tol=0.02,
            center_tol=0.06,
            target_uv_u=0.5,
            target_uv_v=-0.5,
        )
        obs = VisualObservation(
            label="ball",
            confidence=0.9,
            center_uv=(0.50, -0.50),
            scale=0.145,
            timestamp_s=1.0,
        )
        ready, reason = pick_ready_for_extend(
            obs, cfg=cfg, approach_steps=5, scale_plateau=True
        )
        self.assertTrue(ready)
        self.assertEqual(reason, "scale_plateau")

    def test_scale_ok_without_center_fails(self) -> None:
        cfg = PickConfig(target_scale=0.16, scale_tol=0.02, center_tol=0.06)
        obs = VisualObservation(
            label="ball",
            confidence=0.9,
            center_uv=(0.58, -0.54),
            scale=0.16,
            timestamp_s=1.0,
        )
        ready, _ = pick_ready_for_extend(obs, cfg=cfg)
        self.assertFalse(ready)


if __name__ == "__main__":
    unittest.main()
