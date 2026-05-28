"""Tests for elesim host publish client (no live ZMQ server)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from elesim_bridge.host_client import (
    _HOST_CLIENT_VERSION,
    _recv_reply,
    build_perception_target_payload,
)


class TestHostClientRecv(unittest.TestCase):
    def test_recv_reply_uses_recv_json(self) -> None:
        sock = MagicMock()
        sock.recv_json.return_value = {"t": "ack", "ok": True}
        ack = _recv_reply(sock)
        self.assertTrue(ack["ok"])
        sock.recv_json.assert_called_once()
        sock.recv_multipart.assert_not_called()

    def test_version_marker_present(self) -> None:
        self.assertIn("tracker", _HOST_CLIENT_VERSION)


class TestPerceptionPayload(unittest.TestCase):
    def test_build_payload_with_track_fields(self) -> None:
        payload = build_perception_target_payload(
            object_camera_xyz=[0.01, -0.02, 0.65],
            label="cup",
            track={
                "track_state": "TRACKING_3D",
                "track_confidence": 0.82,
                "bbox_xyxy": [10, 20, 50, 60],
                "center_uv": [30.0, 40.0],
                "mu_camera": [0.01, -0.02, 0.64],
                "sigma_camera": [0.002, 0.002, 0.003],
                "depth_valid_ratio": 0.73,
                "lost_count": 0,
            },
        )
        self.assertEqual(payload["source"], "perception")
        self.assertEqual(payload["track_state"], "TRACKING_3D")
        self.assertAlmostEqual(float(payload["track_confidence"]), 0.82)
        self.assertEqual(len(payload["mu_camera"]), 3)
        self.assertEqual(len(payload["sigma_camera"]), 3)
        self.assertAlmostEqual(float(payload["depth_valid_ratio"]), 0.73)


if __name__ == "__main__":
    unittest.main()
