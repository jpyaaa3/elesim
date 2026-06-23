from __future__ import annotations

import json
import os
import tempfile
import unittest
from unittest import mock

from engine.experiment.run_context import RunContext


class RunContextTests(unittest.TestCase):
    def test_write_meta_schema(self) -> None:
        ctx = RunContext.from_cli(
            run_id="exp_a",
            arm_preset="bent_upward",
            go2_motion="backward",
            gaze_mode="uv",
            notes="test",
        )
        with tempfile.TemporaryDirectory() as td:
            path = ctx.write_meta(td)
            payload = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(payload["run_id"], "exp_a")
            self.assertEqual(payload["arm_preset"], "bent_upward")
            self.assertEqual(payload["go2_motion"], "backward")
            self.assertEqual(payload["gaze_mode"], "uv")

    def test_validate_env_run_id(self) -> None:
        ctx = RunContext(run_id="exp_match")
        with mock.patch.dict(os.environ, {"ELESIM_RUN_ID": "exp_match"}):
            self.assertTrue(ctx.validate_env_run_id())
        with mock.patch.dict(os.environ, {"ELESIM_RUN_ID": "other"}):
            with self.assertRaises(ValueError):
                ctx.validate_env_run_id(strict=True)


if __name__ == "__main__":
    unittest.main()
