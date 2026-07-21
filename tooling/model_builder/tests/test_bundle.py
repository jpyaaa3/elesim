from __future__ import annotations

import json
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from elesim_model_builder.bundle import (
    BundleIntegrityError,
    build_simulator_bundle,
    validate_bundle,
)


ROOT = Path(__file__).resolve().parents[3]
ASSETS = ROOT / "model/source/assets"


def _referenced_paths(bundle: Path) -> list[Path]:
    blueprint = json.loads((bundle / "blueprint.json").read_text(encoding="utf-8"))
    references = [
        Path(value)
        for part in blueprint["parts"]
        for value in part["assets"].values()
    ]
    for urdf_name in ("arm.urdf", "robot.urdf"):
        root = ET.parse(bundle / urdf_name).getroot()
        references.extend(Path(mesh.attrib["filename"]) for mesh in root.iter("mesh"))
    return references


class SimulatorBundleTests(unittest.TestCase):
    def test_bundle_is_self_contained_after_copy(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            built = build_simulator_bundle(
                asset_root=ASSETS,
                output_dir=root / "built",
                use_go2=True,
            )
            detached = root / "detached"
            shutil.copytree(built, detached)

            metadata = validate_bundle(detached)

            self.assertEqual(metadata["schema_version"], 1)
            self.assertEqual(metadata["bundle_type"], "elesim.simulator-model")
            self.assertEqual(
                metadata["entrypoints"],
                {
                    "blueprint": "blueprint.json",
                    "arm_urdf": "arm.urdf",
                    "robot_urdf": "robot.urdf",
                },
            )
            for reference in _referenced_paths(detached):
                self.assertFalse(reference.is_absolute(), reference)
                self.assertNotIn("..", reference.parts, reference)
                self.assertTrue((detached / reference).is_file(), reference)

    def test_bundle_records_every_payload_file(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            bundle = build_simulator_bundle(
                asset_root=ASSETS,
                output_dir=Path(td) / "bundle",
                use_go2=True,
            )
            metadata = validate_bundle(bundle)
            actual = {
                path.relative_to(bundle).as_posix()
                for path in bundle.rglob("*")
                if path.is_file() and path.name != "bundle.json"
            }
            self.assertEqual(set(metadata["files"]), actual)

    def test_tampered_asset_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            bundle = build_simulator_bundle(
                asset_root=ASSETS,
                output_dir=Path(td) / "bundle",
                use_go2=True,
            )
            mesh = bundle / "assets/node/node_mesh.obj"
            mesh.write_text(mesh.read_text(encoding="utf-8") + "\n# tampered\n", encoding="utf-8")

            with self.assertRaisesRegex(BundleIntegrityError, "hash mismatch"):
                validate_bundle(bundle)

    def test_untracked_payload_file_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            bundle = build_simulator_bundle(
                asset_root=ASSETS,
                output_dir=Path(td) / "bundle",
                use_go2=True,
            )
            (bundle / "unexpected.txt").write_text("not in manifest", encoding="utf-8")

            with self.assertRaisesRegex(BundleIntegrityError, "untracked files"):
                validate_bundle(bundle)


if __name__ == "__main__":
    unittest.main()
