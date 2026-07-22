from __future__ import annotations

import unittest

from elesim_controller.vision.visual_servoing.sag_drift_frame import prepare_sag_drift_input


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
