from __future__ import annotations

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from elesim_pilot.config import load_app_config
from elesim_pilot.config.yaml_schema import ConfigValidationError


class TestPerceptionWorkerConfig(unittest.TestCase):
    def test_jetson_ini_loads_camera_mode(self) -> None:
        bundle = load_app_config("payload/config/pilot/config.yaml", mode="jetson")
        pc = bundle.perception_config
        self.assertTrue(pc.run_local)
        self.assertEqual(str(pc.mode).strip().lower(), "camera")
        self.assertFalse(pc.show_preview)

    def test_pc_ini_uses_pilot_owned_perception(self) -> None:
        bundle = load_app_config("payload/config/pilot/config.yaml", mode="pc")
        pc = bundle.perception_config
        self.assertTrue(pc.run_local)
        self.assertEqual(str(pc.provider).strip().lower(), "local")

    def test_stale_remote_provider_is_rejected(self) -> None:
        source = Path("payload/config/pilot/config.yaml")
        with TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / source.name
            text = source.read_text(encoding="utf-8")
            text = text.replace("          provider: local\n          run_local: true", "          provider: host\n          run_local: false", 1)
            path.write_text(text, encoding="utf-8")
            with self.assertRaisesRegex(ConfigValidationError, "Pilot-owned local perception"):
                load_app_config(str(path), mode="pc")


if __name__ == "__main__":
    unittest.main()
