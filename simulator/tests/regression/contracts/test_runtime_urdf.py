from __future__ import annotations

import unittest

from elesim_simulator.core.runtime_urdf import select_runtime_urdf


class RuntimeUrdfSelectionTests(unittest.TestCase):
    def test_runtime_urdf_selection_uses_arm_without_go2(self) -> None:
        self.assertEqual(
            select_runtime_urdf(
                use_go2=False,
                arm_urdf="crafts/arm.urdf",
                robot_urdf="crafts/robot.urdf",
            ),
            "crafts/arm.urdf",
        )

    def test_runtime_urdf_selection_uses_robot_with_go2(self) -> None:
        self.assertEqual(
            select_runtime_urdf(
                use_go2=True,
                arm_urdf="crafts/arm.urdf",
                robot_urdf="crafts/robot.urdf",
            ),
            "crafts/robot.urdf",
        )


if __name__ == "__main__":
    unittest.main()
