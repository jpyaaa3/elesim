"""Tests for pick extend readiness when CSRT scale stalls."""

from __future__ import annotations

import unittest

from engine.config_loader import PickConfig
from engine.controller.object_pick import pick_ready_for_extend
from engine.controller.perception import VisualObservation


class PickExtendReadyTests(unittest.TestCase):
    def test_approach_steps_allow_extend_when_scale_floor_met(self) -> None:
        cfg = PickConfig(
            target_scale=0.16,
            scale_tol=0.02,
            center_tol=0.06,
            target_uv_u=0.5,
            target_uv_v=-0.5,
            approach_min_scale=0.09,
            approach_min_steps=50,
            approach_loose_center_tol=0.10,
        )
        obs = VisualObservation(
            label="ball",
            confidence=0.9,
            center_uv=(0.584, -0.537),
            scale=0.101,
            timestamp_s=1.0,
        )
        ready, reason = pick_ready_for_extend(
            obs, cfg=cfg, approach_steps=77, scale_plateau=False
        )
        self.assertTrue(ready)
        self.assertEqual(reason, "approach_steps")

    def test_scale_plateau_path(self) -> None:
        cfg = PickConfig(approach_min_scale=0.09, approach_loose_center_tol=0.10)
        obs = VisualObservation(
            label="ball",
            confidence=0.9,
            center_uv=(0.51, -0.51),
            scale=0.10,
            timestamp_s=1.0,
        )
        ready, reason = pick_ready_for_extend(
            obs, cfg=cfg, approach_steps=10, scale_plateau=True
        )
        self.assertTrue(ready)
        self.assertEqual(reason, "scale_plateau")


if __name__ == "__main__":
    unittest.main()
