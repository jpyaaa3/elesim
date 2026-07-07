from __future__ import annotations

import time
import unittest
from unittest.mock import MagicMock

from engine.core.config_loader import PerceptionConfig, load_app_config_from_ini
from engine.behaviors.pick.actions import ControlService
from engine.behaviors.pick.state import HostState, PanelState
from engine.core.protocol import SimQ


class TestPerceptionRemote(unittest.TestCase):
    def _remote_service(self) -> ControlService:
        state = PanelState()
        client = MagicMock()
        host_state = HostState(
            connected=True,
            tx_seq=1,
            rx_age_s=0.0,
            device="",
            ports=(),
            torque_enabled=False,
            claw_current=0,
            motor_currents_ma={},
            safety_fault="",
            actual_tip_xyz=None,
            actual_tip_dir=None,
            perceived_object_label="ball",
            perceived_object_confidence=0.9,
            perceived_object_camera_xyz=(0.1, 0.2, 0.8),
            perceived_center_uv=(0.05, 0.1),
            perceived_scale=0.2,
            perceived_timestamp_s=time.time(),
            go2_vel=(0.0, 0.0, 0.0),
            reply_ok=True,
            reply_reason="",
            q=SimQ(0.0, 0.0, 0.0, 0.0),
            u=None,
        )
        client.get_state.return_value = host_state
        client.refresh_state.return_value = host_state
        return ControlService(
            state,
            client=client,
            perception_cfg=PerceptionConfig(run_local=False, mode="sim"),
        )

    def test_start_perception_noop_when_remote(self) -> None:
        svc = self._remote_service()
        svc.start_perception_capture()
        self.assertIsNone(svc._perception_capture)
        self.assertIn("Jetson", str(svc.state.perception_status_msg))

    def test_current_visual_observation_host_only(self) -> None:
        svc = self._remote_service()
        host = svc.current_host_state()
        assert host is not None
        svc.state.set_perception_status(
            running=True,
            failed=False,
            msg="local",
            center_uv=(0.99, 0.99),
            image_scale=0.5,
            frame_idx=5,
        )
        obs = svc.current_visual_observation(host)
        self.assertIsNotNone(obs)
        self.assertAlmostEqual(float(obs.center_uv[0]), 0.05, places=4)

    def test_sync_remote_perception_from_host(self) -> None:
        svc = self._remote_service()
        host = svc.current_host_state()
        assert host is not None
        svc._sync_remote_perception_from_host(host)
        self.assertTrue(bool(svc.state.perception_running))
        self.assertEqual(str(svc.state.perception_label), "ball")

    def test_maybe_start_local_skips_remote(self) -> None:
        svc = self._remote_service()
        svc._maybe_start_local_perception()
        self.assertIsNone(svc._perception_capture)


class TestPerceptionWorkerConfig(unittest.TestCase):
    def test_jetson_ini_loads_camera_mode(self) -> None:
        bundle = load_app_config_from_ini("config.jetson.ini")
        pc = bundle.perception_config
        self.assertTrue(pc.run_local)
        self.assertEqual(str(pc.mode).strip().lower(), "camera")
        self.assertFalse(pc.show_preview)

    def test_pc_ini_loads_remote(self) -> None:
        bundle = load_app_config_from_ini("config.pc.ini")
        pc = bundle.perception_config
        self.assertFalse(pc.run_local)


if __name__ == "__main__":
    unittest.main()
