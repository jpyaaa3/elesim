from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
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


class PrepareSagDriftInputTests(unittest.TestCase):
    def test_pure_axial_drift_is_usable(self) -> None:
        prepared = prepare_sag_drift_input(
            drift_world=(0.010, 0.0, 0.0),
            axis_world=(1.0, 0.0, 0.0),
            reference_dir=(1.0, 0.0, 0.0),
            max_dir_error_deg=12.0,
            max_lateral_m=0.015,
        )
        self.assertTrue(prepared.usable)
        self.assertAlmostEqual(prepared.axial_m, 0.010, places=6)
        self.assertAlmostEqual(prepared.lateral_m, 0.0, places=6)
        self.assertAlmostEqual(prepared.sag_input_world[0], 0.010, places=6)

    def test_large_lateral_rejected_when_axial_only_off(self) -> None:
        prepared = prepare_sag_drift_input(
            drift_world=(0.010, 0.020, 0.0),
            axis_world=(1.0, 0.0, 0.0),
            reference_dir=(1.0, 0.0, 0.0),
            max_dir_error_deg=12.0,
            max_lateral_m=0.015,
            axial_only=False,
        )
        self.assertFalse(prepared.usable)
        self.assertEqual(prepared.reason, "lateral_drift_too_large")

    def test_large_lateral_ok_when_axial_only_on(self) -> None:
        """Aim recenter: mostly lateral in FK frame, small axial sag correction still allowed."""
        prepared = prepare_sag_drift_input(
            drift_world=(0.017, 0.030, 0.024),
            axis_world=(0.497, 0.192, -0.846),
            reference_dir=(0.497, 0.192, -0.846),
            max_dir_error_deg=18.0,
            max_lateral_m=0.015,
            axial_only=True,
        )
        self.assertTrue(prepared.usable)
        self.assertEqual(prepared.reason, "accepted")
        self.assertGreater(float(prepared.lateral_m), 0.030)

    def test_dir_error_is_rejected(self) -> None:
        prepared = prepare_sag_drift_input(
            drift_world=(0.010, 0.0, 0.0),
            axis_world=(0.0, 0.0, 1.0),
            reference_dir=(1.0, 0.0, 0.0),
            max_dir_error_deg=12.0,
            max_lateral_m=0.015,
        )
        self.assertFalse(prepared.usable)
        self.assertEqual(prepared.reason, "dir_error_too_large")

    def test_axial_only_false_uses_full_drift(self) -> None:
        prepared = prepare_sag_drift_input(
            drift_world=(0.010, 0.005, 0.0),
            axis_world=(1.0, 0.0, 0.0),
            reference_dir=(1.0, 0.0, 0.0),
            max_dir_error_deg=12.0,
            max_lateral_m=0.020,
            axial_only=False,
        )
        self.assertTrue(prepared.usable)
        self.assertAlmostEqual(prepared.sag_input_world[0], 0.010, places=6)
        self.assertAlmostEqual(prepared.sag_input_world[1], 0.005, places=6)


if __name__ == "__main__":
    unittest.main()
