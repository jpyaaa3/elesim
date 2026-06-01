from __future__ import annotations

import unittest

from engine.protocol import pack_state


class DebugMarkerProtocolTests(unittest.TestCase):
    def test_debug_marker_length_is_preserved(self) -> None:
        msg = pack_state(
            debug_markers=[
                {
                    "name": "ready_pose_standoff",
                    "frame": "world",
                    "pos": [1.0, 2.0, 3.0],
                    "dir": [0.3, 0.0, 0.0],
                    "length": 0.3,
                }
            ]
        )

        self.assertEqual(msg["debug_markers"][0]["length"], 0.3)


if __name__ == "__main__":
    unittest.main()
