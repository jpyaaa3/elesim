from __future__ import annotations

import os
import unittest

from misc.tooling.quality.check import CHECKS, PROTOCOL_SRC, python_path, select_checks


class QualityMatrixTests(unittest.TestCase):
    def test_required_matrix_has_every_deployment_and_topology(self) -> None:
        names = {check.name for check in select_checks(group="required", names=())}
        self.assertEqual(
            names,
            {
                "protocol",
                "router",
                "robot",
                "controller",
                "simulator",
                "ui",
                "model-builder",
                "setup-tools",
                "topology",
                "secure-media",
                "webrtc-media",
            },
        )

    def test_each_deployment_gets_only_its_own_source_root(self) -> None:
        deployment_names = {"router", "robot", "controller", "simulator", "ui"}
        for check in CHECKS:
            if check.name not in deployment_names:
                continue
            own_source = f"{check.name}/src"
            self.assertIn(PROTOCOL_SRC, check.python_paths)
            self.assertIn(own_source, check.python_paths)
            sibling_sources = {
                f"{name}/src" for name in deployment_names if name != check.name
            }
            self.assertTrue(sibling_sources.isdisjoint(check.python_paths))

    def test_explicit_names_preserve_requested_order(self) -> None:
        selected = select_checks(group="required", names=("robot", "protocol"))
        self.assertEqual([check.name for check in selected], ["robot", "protocol"])

    def test_python_path_prepends_matrix_and_deduplicates(self) -> None:
        check = next(check for check in CHECKS if check.name == "router")
        inherited = os.pathsep.join(("/tmp/external", "/tmp/external"))
        entries = python_path(check, inherited).split(os.pathsep)
        self.assertEqual(entries[-1], "/tmp/external")
        self.assertEqual(entries.count("/tmp/external"), 1)

    def test_unknown_check_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "unknown checks"):
            select_checks(group="required", names=("missing",))

    def test_extended_matrix_includes_the_quality_gate_itself(self) -> None:
        names = {check.name for check in select_checks(group="extended", names=())}
        self.assertIn("quality-tools", names)
        self.assertIn("critical-mutations", names)


if __name__ == "__main__":
    unittest.main()
