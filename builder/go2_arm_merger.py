from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Sequence


def _fmt_xyz(values: Sequence[float]) -> str:
    vals = [float(v) for v in values]
    if len(vals) != 3:
        raise ValueError(f"expected 3 mount values, got {len(vals)}")
    return "%.9g %.9g %.9g" % (vals[0], vals[1], vals[2])


def _indent(elem: ET.Element, level: int = 0) -> None:
    space = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = space + "  "
        for child in elem:
            _indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = space
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = space


def merge_go2_arm_urdf(
    *,
    go2_urdf_path: str | os.PathLike[str],
    arm_urdf_path: str | os.PathLike[str],
    out_urdf_path: str | os.PathLike[str],
    mount_xyz: Sequence[float],
    parent_link: str = "base",
    child_link: str = "plate",
    joint_name: str = "j_go2_base_arm_plate",
) -> str:
    """Merge a GO2 URDF and arm URDF into one inspection/full-model URDF."""

    go2_path = Path(go2_urdf_path).resolve()
    arm_path = Path(arm_urdf_path).resolve()
    out_path = Path(out_urdf_path).resolve()
    if not go2_path.is_file():
        raise FileNotFoundError(f"GO2 URDF not found: {go2_path}")
    if not arm_path.is_file():
        raise FileNotFoundError(f"arm URDF not found: {arm_path}")

    go2_root = ET.parse(go2_path).getroot()
    arm_root = ET.parse(arm_path).getroot()
    if go2_root.tag != "robot" or arm_root.tag != "robot":
        raise ValueError("both inputs must be URDF robot documents")

    merged = ET.Element("robot", attrib={"name": "go2_arm"})
    for child in list(go2_root):
        merged.append(child)
    for child in list(arm_root):
        merged.append(child)

    joint = ET.SubElement(merged, "joint", attrib={"name": joint_name, "type": "fixed"})
    ET.SubElement(joint, "parent", attrib={"link": parent_link})
    ET.SubElement(joint, "child", attrib={"link": child_link})
    ET.SubElement(joint, "origin", attrib={"xyz": _fmt_xyz(mount_xyz), "rpy": "0 0 0"})

    _indent(merged)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(merged).write(out_path, encoding="utf-8", xml_declaration=True)
    return str(out_path)


__all__ = ["merge_go2_arm_urdf"]
