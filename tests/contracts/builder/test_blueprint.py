from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from builders.json_builder import build_default_manifest


class AssemblyBlueprintTests(unittest.TestCase):
    def test_default_output_is_blueprint_json(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(build_default_manifest(tmp))
            self.assertEqual(output.name, "blueprint.json")
            self.assertTrue(output.is_file())

    def test_explicit_legacy_output_name_is_preserved(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(build_default_manifest(tmp, output_name="manifest.json"))
            self.assertEqual(output.name, "manifest.json")
            self.assertTrue(output.is_file())


if __name__ == "__main__":
    unittest.main()
