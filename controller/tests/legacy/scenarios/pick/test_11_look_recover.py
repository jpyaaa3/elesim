from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "AGENTS.md").exists())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_controller.config import PerceptionConfig, PickConfig
from elesim_controller.pick.actions import ControlService
from elesim_controller.pick.state import PanelState


class TestLookPostRecover(unittest.TestCase):
    def test_post_uv_recover_skipped_in_mock(self) -> None:
        svc = ControlService(PanelState())
        svc._pick_cfg = PickConfig(
            look_post_uv_recover_enabled=True,
            grasp_skip_aim_recover_in_mock=True,
        )
        svc._perception_cfg = PerceptionConfig(mode="mock")
        host = MagicMock()
        out = svc._look_post_move_uv_recover(
            pk=svc._pick_cfg,
            host_state=host,
            object_world=(0.3, 0.0, 0.9),
            sag_model={},
        )
        self.assertIs(host, out)

    def test_sag_trim_calls_align(self) -> None:
        svc = ControlService(PanelState())
        svc.client = MagicMock()
        host = MagicMock()
        host.q = MagicMock(linear_m=0.1, roll_rad=0.0, theta1_rad=0.1, theta2_rad=0.0)
        host.reply_ok = True
        with patch.object(
            svc, "_pick_current_tip_world", return_value=(0.2, 0.0, 0.9)
        ), patch.object(
            svc, "_grasp_align_to_approach_dir", return_value=(True, host)
        ) as mock_align, patch.object(
            svc, "_pick_latch_fk_achieved_pose", return_value=True
        ):
            svc._pick_achieved_dir_world = (0.0, 0.0, 1.0)
            out = svc._look_post_sag_trim_to_object(
                object_world=(0.3, 0.0, 0.9),
                sag_model={},
                host_state=host,
            )
        self.assertIs(host, out)
        mock_align.assert_called_once()


if __name__ == "__main__":
    unittest.main()
