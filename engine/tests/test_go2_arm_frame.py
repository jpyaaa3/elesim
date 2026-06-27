from __future__ import annotations

import unittest

import numpy as np

from engine.coordinates.go2_arm_frame import (
    Go2ArmFrameConfig,
    ik_point_to_sim_frame,
    sim_point_to_ik_frame,
)


class TestGo2ArmFrame(unittest.TestCase):
    def _cfg(self) -> Go2ArmFrameConfig:
        return Go2ArmFrameConfig(
            use_go2=True,
            ik_spawn_xyz=(0.35, 0.0, 0.40),
            ik_spawn_euler_deg=(0.0, 0.0, 0.0),
            go2_spawn_xyz=(0.0, 0.0, 0.32),
            go2_spawn_euler_deg=(0.0, 0.0, 0.0),
            mount_offset_body_m=(0.35, 0.0, 0.08),
            mount_pos_body=(0.35, 0.0, 0.08),
            mount_rot_body_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        )

    def test_identity_when_go2_at_spawn(self) -> None:
        cfg = self._cfg()
        T_ik = cfg.ik_spawn_transform()
        T_arm = cfg.arm_base_transform(cfg.go2_spawn_xyz, (0.0, 0.0, 0.0))
        p_sim = np.array([1.2, 0.0, 0.08], dtype=float)
        p_ik = sim_point_to_ik_frame(p_sim, T_ik_spawn=T_ik, T_arm_current=T_arm)
        back = ik_point_to_sim_frame(p_ik, T_ik_spawn=T_ik, T_arm_current=T_arm)
        self.assertTrue(np.allclose(p_ik, p_sim, atol=1e-6))
        self.assertTrue(np.allclose(back, p_sim, atol=1e-6))

    def test_translation_when_go2_walked(self) -> None:
        cfg = self._cfg()
        T_ik = cfg.ik_spawn_transform()
        go2_pos = (0.5, 0.1, 0.32)
        T_arm = cfg.arm_base_transform(go2_pos, (0.0, 0.0, 0.0))
        p_sim = np.array([1.2, 0.0, 0.08], dtype=float)
        p_ik = sim_point_to_ik_frame(p_sim, T_ik_spawn=T_ik, T_arm_current=T_arm)
        self.assertTrue(np.allclose(p_ik, p_sim - np.array([0.5, 0.1, 0.0]), atol=1e-6))
        back = ik_point_to_sim_frame(p_ik, T_ik_spawn=T_ik, T_arm_current=T_arm)
        self.assertTrue(np.allclose(back, p_sim, atol=1e-6))

    def test_yaw_roundtrip(self) -> None:
        cfg = self._cfg()
        T_ik = cfg.ik_spawn_transform()
        yaw = float(np.pi / 2.0)
        T_arm = cfg.arm_base_transform(cfg.go2_spawn_xyz, (0.0, 0.0, yaw))
        p_sim = np.array([1.2, 0.0, 0.08], dtype=float)
        p_ik = sim_point_to_ik_frame(p_sim, T_ik_spawn=T_ik, T_arm_current=T_arm)
        back = ik_point_to_sim_frame(p_ik, T_ik_spawn=T_ik, T_arm_current=T_arm)
        self.assertFalse(np.allclose(p_ik, p_sim, atol=1e-3))
        self.assertTrue(np.allclose(back, p_sim, atol=1e-6))


if __name__ == "__main__":
    unittest.main()
