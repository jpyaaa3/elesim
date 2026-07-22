from __future__ import annotations

import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from elesim_model_builder.go2_arm_merger import merge_go2_arm_urdf


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

    def test_merge_rebases_each_input_mesh_to_output_directory(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            go2 = tmp / "assets/go2/go2.urdf"
            arm = tmp / "arm.urdf"
            out = tmp / "robot.urdf"
            go2.parent.mkdir(parents=True)
            (go2.parent / "dae").mkdir()
            (go2.parent / "dae/base.dae").write_text("mesh", encoding="utf-8")
            (tmp / "assets/arm").mkdir(parents=True)
            (tmp / "assets/arm/plate.obj").write_text("mesh", encoding="utf-8")
            _write(
                go2,
                '<robot name="go2"><link name="base"><visual><geometry>'
                '<mesh filename="dae/base.dae"/></geometry></visual></link></robot>',
            )
            _write(
                arm,
                '<robot name="arm"><link name="plate"><visual><geometry>'
                '<mesh filename="assets/arm/plate.obj"/></geometry></visual></link></robot>',
            )

            merge_go2_arm_urdf(
                go2_urdf_path=go2,
                arm_urdf_path=arm,
                out_urdf_path=out,
                mount_xyz=(0.0, 0.0, 0.0),
            )

            filenames = [mesh.attrib["filename"] for mesh in ET.parse(out).getroot().iter("mesh")]
            self.assertEqual(filenames, ["assets/go2/dae/base.dae", "assets/arm/plate.obj"])


if __name__ == "__main__":
    unittest.main()
