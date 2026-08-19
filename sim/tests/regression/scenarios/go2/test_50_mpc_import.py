from __future__ import annotations

import importlib
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


class Go2MpcImportTests(unittest.TestCase):
    def test_genesis_pin_bridge_import_without_convex_mpc(self) -> None:
        sys.modules.pop("convex_mpc", None)
        sys.modules.pop("elesim_sim.robot.go2.mpc.genesis_pin_bridge", None)
        mod = importlib.import_module("elesim_sim.robot.go2.mpc.genesis_pin_bridge")
        self.assertTrue(callable(mod._quat_wxyz_to_xyzw))
        self.assertNotIn("convex_mpc", sys.modules)

    def test_controller_patches_convex_mpc_go2_urdf_path(self) -> None:
        controller = importlib.import_module("elesim_sim.robot.go2.mpc.controller")
        root = Path(__file__).resolve().parents[5]
        expected = root / "model/bundles/default/assets/go2/go2.urdf"
        controller._require_convex_mpc(go2_urdf_path=expected)
        data = importlib.import_module("convex_mpc.go2_robot_data")
        urdf_path = Path(data.URDF_PATH)
        self.assertTrue(urdf_path.is_file())
        self.assertEqual(urdf_path.resolve(), expected.resolve())
        self.assertEqual(urdf_path.name, "go2.urdf")
        self.assertEqual(urdf_path.parent.name, "go2")
        pin_model = data.PinGo2Model()
        self.assertGreaterEqual(pin_model.base_id, 0)

    def test_solver_failure_is_not_hidden_behind_stale_forces(self) -> None:
        controller_module = importlib.import_module("elesim_sim.robot.go2.mpc.controller")
        controller = controller_module.ConvexMpcGenesisController.__new__(
            controller_module.ConvexMpcGenesisController
        )

        class FailingMpc:
            @staticmethod
            def solve_QP(*_args):
                raise RuntimeError("qp failed")

        controller._mpc = FailingMpc()
        controller._pin = object()
        controller._traj = SimpleNamespace(N=1, generate_traj=lambda *_args, **_kwargs: None)
        controller._gait = object()
        controller._sim_time = 0.0
        controller._config = SimpleNamespace(mpc_dt_s=0.02, force_filter_alpha=0.35)
        controller._force_filt = np.full(12, 3.0, dtype=float)
        controller._U_opt = np.full((12, 1), 7.0, dtype=float)
        controller._apply_payload_pitch_trim = lambda _vx: None

        with self.assertRaisesRegex(RuntimeError, "qp failed"):
            controller._solve_mpc(0.0, 0.0, 0.3, 0.0)
        np.testing.assert_array_equal(controller._U_opt, np.full((12, 1), 7.0))


if __name__ == "__main__":
    unittest.main()
