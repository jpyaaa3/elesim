from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "AGENTS.md").exists())
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

_SPEC = importlib.util.spec_from_file_location(
    "sag_drift_frame",
    ROOT / "engine" / "vision" / "visual_servoing" / "sag_drift_frame.py",
)
assert _SPEC is not None and _SPEC.loader is not None
_SAG_DRIFT = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _SAG_DRIFT
_SPEC.loader.exec_module(_SAG_DRIFT)
prepare_sag_drift_input = _SAG_DRIFT.prepare_sag_drift_input


class TestEqualSagRejectRegression(unittest.TestCase):
    def test_log_like_lateral_drift_is_rejected_without_axial_only_mode(self) -> None:
        prepared = prepare_sag_drift_input(
            drift_world=(-0.008, -0.016, -0.004),
            axis_world=(0.157, 0.018, -0.123),
            reference_dir=(0.174, -0.005, -0.098),
            max_dir_error_deg=12.0,
            max_lateral_m=0.015,
            axial_only=False,
        )

        self.assertFalse(prepared.usable)
        self.assertEqual(prepared.reason, "lateral_drift_too_large")
        self.assertGreater(float(prepared.lateral_m), 0.015)

    def test_same_log_like_drift_keeps_axial_component_when_axial_only_mode_is_on(self) -> None:
        prepared = prepare_sag_drift_input(
            drift_world=(-0.008, -0.016, -0.004),
            axis_world=(0.157, 0.018, -0.123),
            reference_dir=(0.174, -0.005, -0.098),
            max_dir_error_deg=12.0,
            max_lateral_m=0.015,
            axial_only=True,
        )

        self.assertTrue(prepared.usable)
        self.assertEqual(prepared.reason, "accepted")
        self.assertLess(float(prepared.axial_m), 0.0)
        self.assertGreater(float(prepared.lateral_m), 0.015)


if __name__ == "__main__":
    unittest.main()

