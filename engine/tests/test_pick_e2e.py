from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.controller.actions import ControlService
from engine.vision.pick.core import ObjectPickPhase
from engine.controller.state import PanelState


class TestPickE2E(unittest.TestCase):
    def test_rejects_when_busy(self) -> None:
        svc = ControlService(PanelState())
        svc._pick_e2e_worker = type("T", (), {"is_alive": lambda self: True})()
        svc.start_look_aim_grasp_e2e()
        self.assertTrue(svc.state.pick_failed)
        self.assertEqual(svc.state.pick_status_msg, "busy")

    def test_chains_look_aim_grasp(self) -> None:
        svc = ControlService(PanelState(), client=object())
        calls: list[str] = []

        def _look() -> None:
            calls.append("look")
            svc.state.set_pick_status(
                running=False,
                failed=False,
                phase=ObjectPickPhase.DONE.value,
                msg="look done",
            )

        def _aim() -> None:
            calls.append("aim")
            svc.state.set_pick_status(
                running=False,
                failed=False,
                phase=ObjectPickPhase.DONE.value,
                msg="aim done",
            )

        def _grasp() -> None:
            calls.append("grasp")
            svc.state.set_pick_status(
                running=False,
                failed=False,
                phase=ObjectPickPhase.DONE.value,
                msg="grasp done | claw closed",
            )

        with patch.object(svc, "start_look", side_effect=_look), patch.object(
            svc,
            "start_aim",
            side_effect=_aim,
        ), patch.object(svc, "start_grasp", side_effect=_grasp), patch.object(
            svc,
            "start_ready_pose",
        ) as mock_ready, patch.object(
            svc,
            "start_pick_forward",
        ) as mock_pick:
            svc.start_look_aim_grasp_e2e()
            if svc._pick_e2e_worker is not None:
                svc._pick_e2e_worker.join(timeout=2.0)

        self.assertEqual(calls, ["look", "aim", "grasp"])
        mock_ready.assert_not_called()
        mock_pick.assert_not_called()
        self.assertFalse(svc.state.pick_failed)
        self.assertEqual(svc.state.pick_phase, ObjectPickPhase.DONE.value)
        self.assertIn("E2E done", svc.state.pick_status_msg)


if __name__ == "__main__":
    unittest.main()
