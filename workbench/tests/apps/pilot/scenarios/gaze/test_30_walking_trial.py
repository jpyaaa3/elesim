from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from elesim_pilot.pick.state import HostState
from elesim_pilot.experiment.walking_trial import (
    TrialEyeCameraVideoRecorder,
    horizontal_base_object_distance_m,
    host_horizontal_object_distance_m,
    standoff_base_pos,
)
from elesim_protocol import SimQ


def _host_with_base(base_pos: tuple[float, float, float]) -> HostState:
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
        go2_vel=(0.0, 0.0, 0.0),
        reply_ok=True,
        reply_reason="",
        q=SimQ(0.0, 0.0, 0.0, 0.0),
        u=None,
        go2_base_pos=base_pos,
    )


class WalkingTrialGeometryTests(unittest.TestCase):
    def test_horizontal_distance_ignores_z(self) -> None:
        d = horizontal_base_object_distance_m((0.0, 0.0, 0.32), (1.2, 0.0, 0.99))
        self.assertIsNotNone(d)
        self.assertAlmostEqual(float(d), 1.2, places=6)

    def test_horizontal_distance_none_without_base(self) -> None:
        self.assertIsNone(horizontal_base_object_distance_m(None, (1.0, 0.0, 0.0)))

    def test_host_horizontal_distance(self) -> None:
        host = _host_with_base((0.9, 0.0, 0.3))
        d = host_horizontal_object_distance_m(host, (1.2, 0.0, 0.08))
        self.assertAlmostEqual(float(d), 0.3, places=6)

    def test_standoff_stop_condition(self) -> None:
        d = host_horizontal_object_distance_m(_host_with_base((0.95, 0.0, 0.3)), (1.2, 0.0, 0.08))
        self.assertTrue(float(d or 999) <= 0.30)

    def test_standoff_prefers_sim_base_pos(self) -> None:
        host = HostState(
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
            go2_vel=(0.0, 0.0, 0.0),
            reply_ok=True,
            reply_reason="",
            q=SimQ(0.0, 0.0, 0.0, 0.0),
            u=None,
            go2_base_pos=(0.0, 0.0, 0.3),
            go2_sim_base_pos=(0.95, 0.0, 0.3),
        )
        self.assertEqual(standoff_base_pos(host), (0.95, 0.0, 0.3))
        d = host_horizontal_object_distance_m(host, (1.2, 0.0, 0.08))
        self.assertAlmostEqual(float(d or 0.0), 0.25, places=6)


class TrialEyeCameraVideoRecorderTests(unittest.TestCase):
    def test_hold_last_writes_repeated_frames(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "trial.mp4"
            rec = TrialEyeCameraVideoRecorder(
                endpoint="tcp://127.0.0.1:1",
                use_jpeg=True,
                out_path=out,
                fps=20.0,
            )
            frame = MagicMock()
            frame.color_bgr = object()
            with patch.object(rec, "_ensure_subscriber"), patch.object(
                rec, "_write_bgr", side_effect=lambda _img: setattr(rec, "_frame_count", rec._frame_count + 1)
            ) as write_mock:
                rec._subscriber = MagicMock()
                rec._subscriber.recv_latest.side_effect = [frame, None, None]
                self.assertTrue(rec.poll_and_write(timeout_ms=0, hold_last=True))
                self.assertTrue(rec.poll_and_write(timeout_ms=0, hold_last=True))
                self.assertTrue(rec.poll_and_write(timeout_ms=0, hold_last=True))
                self.assertEqual(rec.unique_frame_count, 1)
                self.assertEqual(rec.frame_count, 3)
                self.assertEqual(write_mock.call_count, 3)
            rec.close()

    def test_close_removes_empty_file(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "trial.mp4"
            rec = TrialEyeCameraVideoRecorder(
                endpoint="tcp://127.0.0.1:1",
                use_jpeg=True,
                out_path=out,
            )
            with patch.object(rec, "poll_and_write", return_value=False):
                pass
            self.assertIsNone(rec.close())
            self.assertFalse(out.exists())


if __name__ == "__main__":
    unittest.main()
