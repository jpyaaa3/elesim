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

    def test_controller_patches_conservative_mpc_friction(self) -> None:
        controller = importlib.import_module("elesim_sim.robot.go2.mpc.controller")
        root = Path(__file__).resolve().parents[5]
        controller._require_convex_mpc(
            go2_urdf_path=root / "model/bundles/default/assets/go2/go2.urdf",
            optimization_friction=0.55,
        )
        centroidal = importlib.import_module("convex_mpc.centroidal_mpc")
        self.assertAlmostEqual(float(centroidal.MU), 0.55)

    def test_qpoases_is_preferred_over_qrqp_when_upstream_solver_is_missing(self) -> None:
        import casadi as ca

        if not ca.has_conic("qpoases"):
            self.skipTest("qpOASES plugin unavailable")
        controller = importlib.import_module("elesim_sim.robot.go2.mpc.controller")
        centroidal = importlib.import_module("convex_mpc.centroidal_mpc")
        root = Path(__file__).resolve().parents[5]
        centroidal.SOLVER_NAME = "missing_solver_for_test"
        controller._require_convex_mpc(
            go2_urdf_path=root / "model/bundles/default/assets/go2/go2.urdf"
        )
        self.assertEqual(centroidal.SOLVER_NAME, "qpoases")

    def test_runtime_force_ranges_define_torque_limits(self) -> None:
        controller = importlib.import_module("elesim_sim.robot.go2.mpc.controller")

        class Entity:
            @staticmethod
            def get_dofs_force_range(*, dofs_idx_local):
                self.assertEqual(dofs_idx_local, list(range(12)))
                upper = np.array([23.7, 23.7, 35.55] * 4)
                return np.vstack((-upper, upper))

        limits = controller._read_torque_limits(Entity(), list(range(12)), safety_scale=0.9)
        np.testing.assert_allclose(limits, np.array([21.33, 21.33, 31.995] * 4))

    def test_physics_parameters_are_applied_and_verified(self) -> None:
        controller_module = importlib.import_module("elesim_sim.robot.go2.mpc.controller")
        controller = controller_module.ConvexMpcGenesisController.__new__(
            controller_module.ConvexMpcGenesisController
        )

        class Entity:
            def __init__(self):
                self.values = {}

            def set_friction(self, value):
                self.values["friction"] = value

            def _set(self, name, value, *, dofs_idx_local):
                self.assertEqual(dofs_idx_local, list(range(12)))
                self.values[name] = np.asarray(value)

            set_dofs_armature = lambda self, value, **kw: self._set("armature", value, **kw)
            set_dofs_damping = lambda self, value, **kw: self._set("damping", value, **kw)
            set_dofs_frictionloss = lambda self, value, **kw: self._set("frictionloss", value, **kw)
            get_dofs_armature = lambda self, **_kw: self.values["armature"]
            get_dofs_damping = lambda self, **_kw: self.values["damping"]
            get_dofs_frictionloss = lambda self, **_kw: self.values["frictionloss"]

        entity = Entity()
        entity.assertEqual = self.assertEqual
        controller._entity = entity
        controller._leg_dof_idxs = list(range(12))
        controller._config = SimpleNamespace(
            physical_friction=0.8,
            joint_armature=0.01,
            joint_damping=0.1,
            joint_frictionloss=0.2,
        )
        controller._apply_go2_physics_params()
        self.assertEqual(entity.values["friction"], 0.8)
        np.testing.assert_allclose(entity.values["armature"], 0.01)

    def test_physics_parameter_mismatch_fails_closed(self) -> None:
        controller_module = importlib.import_module("elesim_sim.robot.go2.mpc.controller")
        controller = controller_module.ConvexMpcGenesisController.__new__(
            controller_module.ConvexMpcGenesisController
        )

        class Entity:
            set_friction = lambda *_args: None
            set_dofs_armature = lambda *_args, **_kwargs: None
            set_dofs_damping = lambda *_args, **_kwargs: None
            set_dofs_frictionloss = lambda *_args, **_kwargs: None
            get_dofs_armature = lambda *_args, **_kwargs: np.zeros(12)
            get_dofs_damping = lambda *_args, **_kwargs: np.full(12, 0.1)
            get_dofs_frictionloss = lambda *_args, **_kwargs: np.full(12, 0.2)

        controller._entity = Entity()
        controller._leg_dof_idxs = list(range(12))
        controller._config = SimpleNamespace(
            physical_friction=0.8,
            joint_armature=0.01,
            joint_damping=0.1,
            joint_frictionloss=0.2,
        )
        with self.assertRaisesRegex(RuntimeError, "armature"):
            controller._apply_go2_physics_params()

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
        controller._config = SimpleNamespace(
            force_filter_alpha=0.35,
            physical_friction=0.8,
            fz_max_n=180.0,
        )
        controller._mpc_dt_s = 0.02
        controller._force_filt = np.full(12, 3.0, dtype=float)
        controller._force_requested = np.zeros(12, dtype=float)
        controller._U_opt = np.full((12, 1), 7.0, dtype=float)
        controller._apply_payload_pitch_trim = lambda _vx: None

        with self.assertRaisesRegex(RuntimeError, "qp failed"):
            controller._solve_mpc(0.0, 0.0, 0.3, 0.0)
        np.testing.assert_array_equal(controller._U_opt, np.full((12, 1), 7.0))

    def test_mpc_normal_force_cap_is_part_of_qp_bounds(self) -> None:
        controller = importlib.import_module("elesim_sim.robot.go2.mpc.controller")

        class BaseMpc:
            def __init__(self, _pin, _trajectory):
                pass

            def _compute_bounds(self, trajectory):
                size = 24 * int(trajectory.N)
                return np.full((size, 1), -np.inf), np.full((size, 1), np.inf)

        mpc = controller._make_bounded_mpc(
            BaseMpc,
            object(),
            SimpleNamespace(N=2),
            fz_max_n=180.0,
        )
        lower, upper = mpc._compute_bounds(SimpleNamespace(N=2))
        expected = np.array([26, 29, 32, 35, 38, 41, 44, 47])
        np.testing.assert_array_equal(upper[expected], np.full((8, 1), 180.0))
        self.assertTrue(np.all(np.isneginf(lower)))


if __name__ == "__main__":
    unittest.main()
