from __future__ import annotations

import math
import sys
import types
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import MagicMock

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "host.py").exists())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

sys.modules.setdefault("zmq", types.ModuleType("zmq"))

from engine.pick import ControlService, PanelState
from engine.config import PerceptionConfig
from engine.pick.state import HostState
from engine.core.protocol import SimQ


def _host(
    *,
    base_pos: tuple[float, float, float] | None = (0.0, 0.0, 0.3),
    sim_base_pos: tuple[float, float, float] | None = None,
    yaw: float = 0.0,
) -> HostState:
    return HostState(
        connected=True,
        tx_seq=0,
        rx_age_s=0.0,
        device="",
        ports=(),
        torque_enabled=False,
        claw_current=0,
        motor_currents_ma={},
        safety_fault="",
        actual_tip_xyz=None,
        actual_tip_dir=None,
        perceived_object_label="",
        perceived_object_confidence=0.0,
        perceived_object_camera_xyz=None,
        perceived_center_uv=None,
        perceived_scale=None,
        perceived_timestamp_s=0.0,
        reply_ok=True,
        reply_reason="",
        q=SimQ(0.0, 0.0, 0.0, 0.0),
        u=None,
        go2_base_pos=base_pos,
        go2_sim_base_pos=sim_base_pos,
        go2_base_rpy=(0.0, 0.0, float(yaw)),
    )


class MobilePickPipelineTests(unittest.TestCase):
    def test_mobile_pick_delegates_to_remote_host(self) -> None:
        client = MagicMock()
        host = replace(
            _host(),
            pick_running=True,
            pick_failed=False,
            pick_phase="acquire",
            pick_status_msg="remote running",
        )
        client.refresh_state.return_value = host
        svc = ControlService(
            PanelState(),
            client=client,
            perception_cfg=PerceptionConfig(run_local=False, provider="host", mode="camera"),
        )

        svc.start_mobile_gaze_lji_pick_e2e()

        client.send_mobile_pick_start.assert_called_once()
        self.assertIsNone(svc._pick_e2e_worker)
        self.assertTrue(svc.state.pick_running)
        self.assertEqual(svc.state.pick_status_msg, "remote running")

    def test_mobile_pick_stop_delegates_to_remote_host(self) -> None:
        client = MagicMock()
        host = replace(
            _host(),
            pick_running=False,
            pick_failed=False,
            pick_phase="idle",
            pick_status_msg="remote stopped",
        )
        client.refresh_state.return_value = host
        svc = ControlService(
            PanelState(),
            client=client,
            perception_cfg=PerceptionConfig(run_local=False, provider="host", mode="camera"),
        )

        svc.stop_pick_e2e()

        client.send_mobile_pick_stop.assert_called_once()
        self.assertFalse(svc.state.pick_running)
        self.assertEqual(svc.state.pick_status_msg, "remote stopped")

    def test_handoff_distance_uses_sim_base_pose(self) -> None:
        svc = ControlService(PanelState(), client=None)
        host = _host(base_pos=(0.0, 0.0, 0.3), sim_base_pos=(0.7, 0.0, 0.3))
        ready, dist = svc._mobile_pick_handoff_ready(
            host,
            (1.0, 0.0, 0.1),
            handoff_distance_m=0.35,
        )
        self.assertTrue(ready)
        self.assertAlmostEqual(float(dist or 0.0), 0.3, places=6)

    def test_timeout_soft_handoff_accepts_near_distance(self) -> None:
        svc = ControlService(PanelState(), client=None)
        host = _host(base_pos=(0.0, 0.0, 0.3))

        ready, dist = svc._mobile_pick_timeout_handoff_ready(
            host,
            (0.712, 0.0, 0.1),
            handoff_distance_m=0.55,
            timeout_slack_m=0.20,
        )
        self.assertTrue(ready)
        self.assertAlmostEqual(float(dist or 0.0), 0.712, places=6)

        ready, _dist = svc._mobile_pick_timeout_handoff_ready(
            host,
            (0.712, 0.0, 0.1),
            handoff_distance_m=0.55,
            timeout_slack_m=0.15,
        )
        self.assertFalse(ready)

    def test_base_velocity_is_body_frame_toward_object(self) -> None:
        svc = ControlService(PanelState(), client=None)
        host = _host(yaw=math.pi / 2.0)
        vx, vy, wz = svc._mobile_pick_base_velocity_toward_object(
            host,
            (0.0, 1.0, 0.1),
            speed_mps=0.2,
        )
        self.assertAlmostEqual(vx, 0.2, places=6)
        self.assertAlmostEqual(vy, 0.0, places=6)
        self.assertAlmostEqual(wz, 0.0, places=6)

    def test_handoff_latch_seeds_lji_without_look(self) -> None:
        svc = ControlService(PanelState(), client=None)
        svc._pick_look_object_world_xyz = (9.0, 9.0, 9.0)
        svc._pick_resolved_ready_dir_world = (0.0, 1.0, 0.0)
        svc._pick_current_tip_world = lambda *, host_state=None: (0.0, 0.0, 0.0)  # type: ignore[method-assign]

        svc._mobile_pick_latch_handoff(
            host_state=_host(),
            object_world=(1.0, 0.0, 0.0),
        )

        self.assertEqual(svc._pick_look_object_world_xyz, None)
        self.assertEqual(svc._pick_centered_object_world_xyz, (1.0, 0.0, 0.0))
        self.assertEqual(svc._pick_frozen_world_xyz, (1.0, 0.0, 0.0))
        self.assertEqual(svc._pick_resolved_ready_dir_world, (1.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
