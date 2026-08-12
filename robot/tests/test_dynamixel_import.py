from __future__ import annotations

import builtins
import unittest
from unittest.mock import patch

import elesim_robot.arm.dynamixel as dynamixel


class DynamixelImportTests(unittest.TestCase):
    def test_missing_sdk_package_is_the_only_optional_import(self) -> None:
        real_import = builtins.__import__

        def missing_package(name, globals=None, locals=None, fromlist=(), level=0):
            if name == "dynamixel_sdk":
                raise ModuleNotFoundError(
                    "No module named 'dynamixel_sdk'", name="dynamixel_sdk"
                )
            return real_import(name, globals, locals, fromlist, level)

        with patch.object(builtins, "__import__", missing_package):
            self.assertEqual(
                dynamixel._load_dynamixel_sdk(), (None, None, None, None)
            )

    def test_broken_sdk_package_import_is_not_hidden(self) -> None:
        real_import = builtins.__import__
        errors = (
            ModuleNotFoundError("No module named 'serial'", name="serial"),
            ImportError("dynamixel_sdk is missing GroupSyncRead"),
        )
        for error in errors:
            with self.subTest(error=error):
                def broken_package(
                    name, globals=None, locals=None, fromlist=(), level=0
                ):
                    if name == "dynamixel_sdk":
                        raise error
                    return real_import(name, globals, locals, fromlist, level)

                with patch.object(builtins, "__import__", broken_package):
                    with self.assertRaises(type(error)) as captured:
                        dynamixel._load_dynamixel_sdk()
                self.assertIs(captured.exception, error)


if __name__ == "__main__":
    unittest.main()
