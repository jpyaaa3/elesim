from __future__ import annotations

import io
import unittest
from contextlib import redirect_stderr
from unittest.mock import patch

from workbench.tests.system import smoke_topology


class TopologySmokeContractTests(unittest.TestCase):
    def test_missing_ros_overlay_fails_the_required_topology_gate(self) -> None:
        stderr = io.StringIO()
        with (
            patch.object(smoke_topology, "_runtime_available", return_value=False),
            redirect_stderr(stderr),
        ):
            self.assertEqual(smoke_topology.main(), 2)

        output = stderr.getvalue()
        self.assertIn(
            "ERROR: ROS 2 rclpy and generated elesim_interfaces are required",
            output,
        )
        self.assertNotIn("SKIP", output)


if __name__ == "__main__":
    unittest.main()
