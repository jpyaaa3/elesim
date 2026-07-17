from __future__ import annotations

import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from builders.go2_arm_merger import merge_go2_arm_urdf


def _write(path: Path, text: str) -> None:
    path.write_text(text, encoding="utf-8")


class Go2ArmMergerTests(unittest.TestCase):
    def test_merge_adds_base_to_plate_fixed_joint(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            go2 = tmp / "go2.urdf"
            arm = tmp / "arm.urdf"
            out = tmp / "robot.urdf"
            _write(go2, '<robot name="go2"><link name="base"/></robot>')
            _write(arm, '<robot name="arm"><link name="plate"/></robot>')

            result = merge_go2_arm_urdf(
                go2_urdf_path=go2,
                arm_urdf_path=arm,
                out_urdf_path=out,
                mount_xyz=(0.1, 0.2, 0.3),
            )

            self.assertEqual(result, str(out.resolve()))
            root = ET.parse(out).getroot()
            joint = root.find("./joint[@name='j_go2_base_arm_plate']")
            self.assertIsNotNone(joint)
            assert joint is not None
            self.assertEqual(joint.attrib["type"], "fixed")
            self.assertEqual(joint.find("parent").attrib["link"], "base")
            self.assertEqual(joint.find("child").attrib["link"], "plate")
            self.assertEqual(joint.find("origin").attrib["xyz"], "0.1 0.2 0.3")

    def test_merge_rejects_duplicate_link_names(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            go2 = tmp / "go2.urdf"
            arm = tmp / "arm.urdf"
            _write(go2, '<robot name="go2"><link name="base"/></robot>')
            _write(arm, '<robot name="arm"><link name="base"/></robot>')

            with self.assertRaisesRegex(ValueError, "duplicate link"):
                merge_go2_arm_urdf(
                    go2_urdf_path=go2,
                    arm_urdf_path=arm,
                    out_urdf_path=tmp / "robot.urdf",
                    mount_xyz=(0.0, 0.0, 0.0),
                )

    def test_merge_rejects_missing_mount_links(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            go2 = tmp / "go2.urdf"
            arm = tmp / "arm.urdf"
            _write(go2, '<robot name="go2"><link name="base"/></robot>')
            _write(arm, '<robot name="arm"><link name="plate"/></robot>')

            with self.assertRaisesRegex(ValueError, "merge parent link not found"):
                merge_go2_arm_urdf(
                    go2_urdf_path=go2,
                    arm_urdf_path=arm,
                    out_urdf_path=tmp / "robot.urdf",
                    mount_xyz=(0.0, 0.0, 0.0),
                    parent_link="missing",
                )


if __name__ == "__main__":
    unittest.main()
