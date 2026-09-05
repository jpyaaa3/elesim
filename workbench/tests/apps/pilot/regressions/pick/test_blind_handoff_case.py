from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "payload").is_dir())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from elesim_pilot.pick.actions import ControlService
from elesim_pilot.config import PickConfig


class TestBlindHandoffRegression(unittest.TestCase):
    def test_lji_blind_finish_uses_blind_micro_threshold_not_close_tol(self) -> None:
        pk = PickConfig(blind_micro_start_m=0.10, grasp_close_tol_m=0.003)
        close_only = PickConfig(blind_micro_start_m=0.0, grasp_close_tol_m=0.003)

        self.assertTrue(ControlService._grasp_lji_should_blind_finish(0.098, pk))
        self.assertFalse(ControlService._grasp_lji_should_blind_finish(0.101, pk))
        self.assertFalse(ControlService._grasp_lji_should_blind_finish(0.002, close_only))


if __name__ == "__main__":
    unittest.main()
