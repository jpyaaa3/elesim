"""Tests for elesim host publish client (no live ZMQ server)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from elesim_bridge.host_client import _HOST_CLIENT_VERSION, _recv_reply


class TestHostClientRecv(unittest.TestCase):
    def test_recv_reply_uses_recv_json(self) -> None:
        sock = MagicMock()
        sock.recv_json.return_value = {"t": "ack", "ok": True}
        ack = _recv_reply(sock)
        self.assertTrue(ack["ok"])
        sock.recv_json.assert_called_once()
        sock.recv_multipart.assert_not_called()

    def test_version_marker_present(self) -> None:
        self.assertIn("camera-frame", _HOST_CLIENT_VERSION)


if __name__ == "__main__":
    unittest.main()
