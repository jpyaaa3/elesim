from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from elesim_pilot.config import load_app_config
from elesim_pilot.config.schema import SimConfig
from elesim_pilot.config.yaml_schema import ConfigValidationError


class YamlConfigContractTests(unittest.TestCase):
    def test_schema_defaults_do_not_request_runtime_model_building(self) -> None:
        config = SimConfig()

        self.assertEqual(config.build_dir, "")
        self.assertFalse(config.rebuild_assembly)

    def _write(self, directory: Path, name: str, text: str) -> Path:
        path = directory / name
        path.write_text(text, encoding="utf-8")
        return path

    def test_repository_profiles_load_and_override_base(self) -> None:
        repo = next(parent for parent in Path(__file__).resolve().parents if (parent / "payload").is_dir())
        config = repo / "payload" / "config" / "pilot" / "config.yaml"
        pc = load_app_config(str(config), mode="pc")
        jetson = load_app_config(str(config), mode="jetson")

        self.assertTrue(pc.sim_config.use_hardware)
        self.assertTrue(jetson.sim_config.use_hardware)
        self.assertEqual(pc.sim_param.dt, jetson.sim_param.dt)
        self.assertFalse(hasattr(pc, "go2_hardware_config"))
        self.assertFalse(hasattr(pc, "go2_locomotion_config"))

    def test_unified_profiles_require_a_valid_mode(self) -> None:
        root = next(parent for parent in Path(__file__).resolve().parents if (parent / "payload").is_dir())
        config = root / "payload" / "config" / "pilot" / "config.yaml"
        with self.assertRaisesRegex(ConfigValidationError, "unknown mode"):
            load_app_config(str(config), mode="missing")

    def test_unselected_profile_is_not_schema_parsed(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = self._write(
                Path(tmp),
                "profiles.yaml",
                """schema_version: 1
mode: pc
profiles:
  pc:
    simulation:
      physics:
        dt: 0.02
  jetson:
    this_key_is_only_valid_when_jetson_is_selected: true
""",
            )
            bundle = load_app_config(str(path))
            self.assertAlmostEqual(bundle.sim_param.dt, 0.02)

    def test_extends_deep_merges_mappings_and_replaces_lists(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            self._write(
                directory,
                "base.yaml",
                """schema_version: 1
robot:
  arm:
    hardware:
      baudrate: 57600
      command_direction: [1, -1, 1, -1]
""",
            )
            child = self._write(
                directory,
                "child.yaml",
                """schema_version: 1
extends: base.yaml
robot:
  arm:
    hardware:
      command_direction: [-1, 1, -1, 1]
""",
            )
            bundle = load_app_config(str(child))
            self.assertEqual(bundle.hardware_config.baudrate, 57600)
            self.assertEqual(bundle.hardware_config.command_direction, (-1, 1, -1, 1))

    def test_relative_runtime_paths_resolve_from_config_directory(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            path = self._write(
                directory,
                "config.yaml",
                """schema_version: 1
simulation:
  assembly:
    build_dir: generated
  cameras:
    hand_eye:
      config: presets/camera.json
vision:
  perception:
    detector:
      detector_config: perception/detector.json
""",
            )
            bundle = load_app_config(str(path))
            self.assertEqual(bundle.sim_config.build_dir, str(directory / "generated"))
            self.assertEqual(bundle.sim_config.hand_eye_config, str(directory / "presets/camera.json"))
            self.assertEqual(
                bundle.perception_config.detector_config,
                str(directory / "perception/detector.json"),
            )
            self.assertEqual(
                bundle.perception_config.resolved_detector_config_path(),
                directory / "perception/detector.json",
            )

    def test_inherited_paths_resolve_from_the_file_that_declares_them(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            parent_dir = directory / "parent"
            child_dir = directory / "child"
            parent_dir.mkdir()
            child_dir.mkdir()
            self._write(
                parent_dir,
                "base.yaml",
                """schema_version: 1
simulation:
  assembly:
    build_dir: generated
""",
            )
            child = self._write(
                child_dir,
                "profile.yaml",
                """schema_version: 1
extends: ../parent/base.yaml
""",
            )
            bundle = load_app_config(str(child))
            self.assertEqual(bundle.sim_config.build_dir, str(parent_dir / "generated"))

    def test_yaml_uses_boolean_12_rules(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = self._write(
                Path(tmp),
                "config.yaml",
                """schema_version: 1
behaviors:
  gaze:
    general:
      walking_gaze_mode: off
""",
            )
            bundle = load_app_config(str(path))
            self.assertEqual(bundle.gaze_stabilizer_config.walking_gaze_mode, "off")

    def test_unknown_duplicate_and_invalid_types_fail_fast(self) -> None:
        cases = {
            "unknown.yaml": "schema_version: 1\nsimulation:\n  runtime:\n    typo: true\n",
            "duplicate.yaml": "schema_version: 1\nsimulation: {}\nsimulation: {}\n",
            "type.yaml": "schema_version: 1\nsimulation:\n  physics:\n    dt: fast\n",
        }
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            for name, text in cases.items():
                with self.subTest(name=name):
                    path = self._write(directory, name, text)
                    with self.assertRaises(ConfigValidationError):
                        load_app_config(str(path))

    def test_extends_cycle_fails_fast(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            a = self._write(directory, "a.yaml", "schema_version: 1\nextends: b.yaml\n")
            self._write(directory, "b.yaml", "schema_version: 1\nextends: a.yaml\n")
            with self.assertRaisesRegex(ConfigValidationError, "cycle"):
                load_app_config(str(a))

    def test_ini_configuration_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = self._write(
                Path(tmp),
                "legacy.ini",
                """[SimParam]
dt = 0.03

[hardware]
command_direction = 1, -1, 1, -1
motor_direction = 1, -1, 1, -1
""",
            )
            with self.assertRaisesRegex(ValueError, "YAML"):
                load_app_config(str(path))


if __name__ == "__main__":
    unittest.main()
