from __future__ import annotations

import argparse
import os
import xml.etree.ElementTree as ET
from pathlib import Path

from elesim_model_builder import json_builder
from elesim_model_builder.go2_arm_merger import merge_go2_arm_urdf
from elesim_model_builder.urdf_converter import convert_manifest_file


def _rebase_meshes(path: Path, asset_root: Path) -> None:
    tree = ET.parse(path)
    relative_root = os.path.relpath(asset_root, path.parent)
    for mesh in tree.getroot().iter("mesh"):
        filename = str(mesh.get("filename", ""))
        marker = "/assets/"
        if marker in filename:
            suffix = filename.split(marker, 1)[1]
            mesh.set("filename", str(Path(relative_root) / suffix))
        elif filename.startswith("../assets/"):
            mesh.set("filename", str(Path(relative_root) / filename[len("../assets/"):]))
    tree.write(path, encoding="utf-8", xml_declaration=True)


def sim_bundle_main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--assets", default="model/source/assets")
    parser.add_argument("--output", default="model/bundles/default")
    parser.add_argument("--use-go2", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--use-hardware", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--mount", nargs=3, type=float, default=(0.35, 0.0, 0.08))
    args = parser.parse_args()
    assets = Path(args.assets).resolve()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    json_builder.DEFAULT_ASSET_ROOT_DIR = str(assets)
    json_builder.build_default_manifest(
        str(output),
        use_hardware=bool(args.use_hardware),
        use_go2=bool(args.use_go2),
        output_name="blueprint.json",
        asset_root=str(assets),
    )
    arm = output / "arm.urdf"
    convert_manifest_file(str(output / "blueprint.json"), str(arm))
    _rebase_meshes(arm, assets)
    if args.use_go2:
        robot = output / "robot.urdf"
        merge_go2_arm_urdf(
            go2_urdf_path=str(assets / "go2/go2.urdf"),
            arm_urdf_path=str(arm),
            out_urdf_path=str(robot),
            mount_xyz=tuple(args.mount),
        )
        _rebase_meshes(robot, assets)
    print(output)


def arm_model_main() -> None:
    from importlib.util import module_from_spec, spec_from_file_location

    script = Path(__file__).parents[2] / "build_arm_model.py"
    spec = spec_from_file_location("elesim_build_arm_model", script)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {script}")
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    module.main()


if __name__ == "__main__":
    sim_bundle_main()
