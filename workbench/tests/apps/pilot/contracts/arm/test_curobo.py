import importlib.util
import json
import shutil
import sys
import threading
from types import ModuleType, SimpleNamespace
from pathlib import Path

import numpy as np
import pytest

from elesim_pilot.robot.arm.iklib import solver
from elesim_pilot.robot.arm.iklib.kinematics import _forward_grasp_world
from elesim_pilot.robot.arm import ik
from elesim_pilot.robot.arm.curobo import CuRoboError, CuRoboPlanner

ROOT = Path(__file__).resolve().parents[6]


def test_pilot_import_does_not_require_cuda():
    # cuRobo is optional even on a machine where the package is not installed.
    if importlib.util.find_spec("curobo") is None:
        with pytest.raises(CuRoboError, match="documented Jetson"):
            CuRoboPlanner(ROOT / "payload/data/models/planning/zed-mini", world={})


def test_changed_artifact_is_rejected_before_loading_cuda(tmp_path):
    model = tmp_path / "model"
    shutil.copytree(ROOT / "payload/data/models/planning/zed-mini", model)
    with (model / "arm.urdf").open("a") as handle:
        handle.write("\n<!-- changed calibration -->\n")
    with pytest.raises(CuRoboError, match="regenerate"):
        CuRoboPlanner(model, world={})


def test_selected_backend_converts_moving_base_and_checks_result(monkeypatch):
    context = json.loads((ROOT / "payload/data/models/arm/default.json").read_text())["context"]
    from elesim_pilot.robot.arm.joint_defs import JointLimit
    context["limit"] = JointLimit(**context["limit"]["value"])
    base = np.eye(4)
    base[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    base[:3, 3] = [1.2, -0.8, 0.4]
    context["base_world_T"] = base
    context["curobo_paths"] = ("model", "world")
    expected_q = np.array([-.08, .4, .1, -.2])
    target = _forward_grasp_world(context, expected_q)
    validated = []

    class Planner:
        def solve_position(self, local, *, seed, tolerance_m):
            assert local == pytest.approx(base[:3, :3].T @ (target - base[:3, 3]))
            assert seed == pytest.approx(expected_q)
            return expected_q

        def validate_configuration(self, q):
            validated.append(q)

    monkeypatch.setattr(solver, "_load_curobo", lambda *_: Planner())
    result = solver.solve_ik(target_world=target, context=context, current_seed=expected_q)
    assert result.success
    assert result.q == pytest.approx(expected_q)
    assert len(validated) == 1


def test_orientation_refinement_cannot_bypass_collision_check(monkeypatch):
    def reject(context, q):
        raise CuRoboError("self collision")
    monkeypatch.setattr(solver, "validate_backend_configuration", reject)
    result = ik._checked_result(context={}, success=True, q=np.zeros(4),
                                position_error_m=0., seed_name="test", iterations=0)
    assert not result.success
    assert result.q is None
    assert "self collision" in result.reason


def test_unknown_backend_and_implicit_world_are_rejected(tmp_path, monkeypatch):
    monkeypatch.setenv("ELESIM_ARM_MODEL", str(ROOT / "payload/data/models/arm/default.json"))
    path = tmp_path / "config.yaml"
    base = ROOT / "payload/config/pilot/config.yaml"
    path.write_text(f"schema_version: 1\nextends: {base}\nrobot:\n  arm:\n    ik:\n      backend: typo\n")
    with pytest.raises(ValueError, match="backend"):
        solver.load_solver_context(str(path))
    path.write_text(path.read_text().replace("backend: typo", "backend: curobo"))
    with pytest.raises(ValueError, match="explicit"):
        solver.load_solver_context(str(path))


def test_backend_paths_are_config_relative_and_custom_limits_require_rebuild(tmp_path, monkeypatch):
    monkeypatch.setenv("ELESIM_ARM_MODEL", str(ROOT / "payload/data/models/arm/default.json"))
    path = tmp_path / "config.yaml"
    base = ROOT / "payload/config/pilot/config.yaml"
    model = ROOT / "payload/data/models/planning/zed-mini"
    path.write_text(f"schema_version: 1\nextends: {base}\nrobot:\n  arm:\n    ik:\n      backend: curobo\n      curobo_model: {model}\n      curobo_world: scene.yaml\n")
    _, context = solver.load_solver_context(str(path))
    assert context["curobo_paths"] == (str(model), str(tmp_path / "scene.yaml"))
    path.write_text(path.read_text().replace("    ik:", "    hardware:\n      linear_u_limit_deg: 150.0\n    ik:"))
    with pytest.raises(ValueError, match="rebuild"):
        solver.load_solver_context(str(path))


def test_static_path_checks_do_not_integrate_poses_as_control_actions(monkeypatch):
    torch = pytest.importorskip("torch")
    module = ModuleType("curobo.types.state")

    class JointState:
        @staticmethod
        def from_position(position, joint_names):
            return SimpleNamespace(position=position, velocity=position * 0, joint_names=joint_names)

    module.JointState = JointState
    monkeypatch.setitem(sys.modules, "curobo.types.state", module)
    chunks = []

    class Rollout:
        def __init__(self):
            self.primitive_collision_constraint = SimpleNamespace(
                forward=lambda spheres, **_kwargs: (spheres[..., 1] <= -.13).float())
            self.robot_self_collision_constraint = SimpleNamespace(
                forward=lambda spheres: torch.zeros_like(spheres[..., 1]))

        def rollout_constraint(self, *_args, **_kwargs):
            raise AssertionError("poses must not be integrated from the stored start state")

        def compute_kinematics(self, state):
            assert not bool(state.velocity.any())
            assert state.position.is_contiguous()
            chunks.append(tuple(state.position.shape))
            return SimpleNamespace(robot_spheres=state.position)

    planner = CuRoboPlanner.__new__(CuRoboPlanner)
    planner.bounds = np.array([[-.142, 0], [-1.57, 1.57], [-.62, .62], [-.62, .62]])
    planner._order = [2, 0, 3, 1]
    planner._lock = threading.Lock()
    planner.motion = SimpleNamespace(rollout_fn=Rollout(), kinematics=SimpleNamespace(joint_names=["b1", "linear", "b2", "roll"]))
    planner._validator = Rollout()
    planner.tensor_args = SimpleNamespace(to_device=lambda values: torch.as_tensor(values, dtype=torch.float32))
    path = np.tile([-.01, .2, .1, -.1], (600, 1))
    planner.validate_path(path)
    assert chunks == [(1, 256, 4), (1, 256, 4), (1, 88, 4)]
    path[300, 0] = -.14
    with pytest.raises(CuRoboError, match="collision"):
        planner.validate_path(path)
