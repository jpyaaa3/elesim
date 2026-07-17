from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path

from ui.file_dialog import _applescript_escape, _resolve_initial_dir, browse_open_file_path
from ui.panels.sag import sag_browse_initial_dir


class FileDialogHelpersTests(unittest.TestCase):
    def test_applescript_escape(self) -> None:
        self.assertEqual(_applescript_escape('a"b\\c'), 'a\\"b\\\\c')

    def test_resolve_initial_dir_from_file(self) -> None:
        with tempfile.NamedTemporaryFile(delete=False) as tmp:
            path = tmp.name
        try:
            parent = os.path.dirname(path)
            self.assertEqual(_resolve_initial_dir(path), os.path.realpath(parent))
        finally:
            os.unlink(path)

    def test_path_matches_extensions(self) -> None:
        from ui.file_dialog import _path_matches_extensions

        self.assertTrue(_path_matches_extensions("/tmp/a.json", (".json",)))
        self.assertFalse(_path_matches_extensions("/tmp/a.txt", (".json",)))
        import ui.file_dialog as mod

        calls: list[str] = []

        def fake_tk(**kwargs) -> str | None:
            calls.append(str(kwargs.get("title", "")))
            return "/tmp/example.json"

        original = mod._browse_open_file_tk
        original_platform = mod.sys.platform
        try:
            mod._browse_open_file_tk = fake_tk  # type: ignore[method-assign]
            mod.sys.platform = "linux"
            selected = browse_open_file_path(title="Pick", initial_dir="/tmp", extensions=(".json",))
        finally:
            mod._browse_open_file_tk = original  # type: ignore[method-assign]
            mod.sys.platform = original_platform
        self.assertEqual(selected, "/tmp/example.json")
        self.assertEqual(calls, ["Pick"])

    def test_sag_browser_defaults_to_config_presets(self) -> None:
        root = Path(__file__).resolve().parents[3]
        self.assertEqual(
            Path(sag_browse_initial_dir("")),
            root / "configs" / "sag",
        )

    def test_detector_browser_defaults_to_config_presets(self) -> None:
        import ui.panels.perception as panel

        calls: list[str] = []

        def fake_browse(**kwargs) -> None:
            calls.append(str(kwargs["initial_dir"]))
            return None

        original = panel.browse_open_file_path
        try:
            panel.browse_open_file_path = fake_browse
            self.assertIsNone(panel.browse_detector_config_path(""))
        finally:
            panel.browse_open_file_path = original

        root = Path(__file__).resolve().parents[3]
        self.assertEqual(calls, [str(root / "configs" / "perception")])


if __name__ == "__main__":
    unittest.main()
