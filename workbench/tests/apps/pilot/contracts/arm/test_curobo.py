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
from elesim_pilot.robot.arm.curobo import CuRoboError, CuRoboPlanner, _world_with_target_sphere

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
    path.write_text(path.read_text().replace("backend: typo", "backend: curobo\n      curobo_world: ''"))
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


def test_target_sphere_world_is_per_request_and_keeps_static_world_unchanged():
    world = {"cuboid": {"table": {"dims": [1., 1., .1],
                                   "pose": [0., 0., -.05, 1., 0., 0., 0.]}}}

    target_world = _world_with_target_sphere(world, [.12, -.03, .4])

    assert "sphere" not in world
    assert target_world["cuboid"] == world["cuboid"]
    assert target_world["sphere"]["elesim_target_object"] == {
        "pose": [.12, -.03, .4, 1., 0., 0., 0.], "radius": .01,
    }
    with pytest.raises(CuRoboError, match="finite"):
        _world_with_target_sphere(world, [0., float("nan"), 0.])


def test_target_sphere_clearance_accounts_for_robot_sphere_radius():
    torch = pytest.importorskip("torch")
    planner = CuRoboPlanner.__new__(CuRoboPlanner)
    planner.tensor_args = SimpleNamespace(
        to_device=lambda value: torch.as_tensor(value, dtype=torch.float32))
    robot_spheres = torch.tensor([[[[.025, 0., 0., .01]]]])

    assert planner._validate_target_sphere(robot_spheres, [0., 0., 0.]) == pytest.approx(.005)
    robot_spheres[..., 0] = .02
    with pytest.raises(CuRoboError, match="robot body/gripper enters"):
        planner._validate_target_sphere(robot_spheres, [0., 0., 0.])


def test_object_target_finishes_between_claws_and_keeps_target_collision_enabled():
    planner = CuRoboPlanner.__new__(CuRoboPlanner)
    planner.bounds = np.array([[-1., 1.]] * 4)
    planner.forward = lambda q: {"position": [q[0], 0., 0.]}
    captured = {}

    def solve(target, *, target_sphere_center, **_kwargs):
        captured["ik_target"] = target
        captured["ik_obstacle"] = target_sphere_center
        return np.array([target[0], 0., 0., 0.])

    planner.solve_position = solve
    checked = []

    def validate(q, *, target_sphere_center=None):
        assert target_sphere_center == pytest.approx([.2, 0., 0.])
        checked.append(q[0])

    def plan(start, goal, *, target_sphere_center=None):
        captured["start"] = start
        captured["goal"] = goal
        captured["center"] = target_sphere_center
        return SimpleNamespace(positions=[start, goal])

    planner.validate_configuration = validate
    planner.plan_joint = plan

    goal, trajectory, goal_xyz = planner.plan_to_target([0., 0., 0., 0.], [.2, 0., 0.])

    assert checked == pytest.approx([0., .2])
    assert goal_xyz == pytest.approx([.2, 0., 0.])
    assert goal == pytest.approx([.2, 0., 0., 0.])
    assert trajectory.positions[-1] == pytest.approx(goal)
    assert captured["ik_target"] == pytest.approx([.2, 0., 0.])
    assert captured["ik_obstacle"] == pytest.approx([.2, 0., 0.])
    assert captured["center"] == pytest.approx([.2, 0., 0.])


@pytest.mark.parametrize("bad_ik", [True, False])
def test_object_goal_rejects_standoff_in_ik_or_trajectory_endpoint(bad_ik):
    planner = CuRoboPlanner.__new__(CuRoboPlanner)
    planner.bounds = np.array([[-1., 1.]] * 4)
    planner.forward = lambda q: {"position": [q[0], 0., 0.]}
    planner.validate_configuration = lambda *args, **kwargs: None
    planner.solve_position = lambda *_args, **_kwargs: [.17 if bad_ik else .2, 0., 0., 0.]
    planner.plan_joint = lambda *_args, **_kwargs: SimpleNamespace(positions=[[.17, 0., 0., 0.]])
    with pytest.raises(CuRoboError, match="(does not reach|between the claws)"):
        planner.plan_to_target([0., 0., 0., 0.], [.2, 0., 0.])


@pytest.mark.parametrize("solver_fails", [False, True])
def test_position_ik_includes_target_obstacle_and_restores_world(monkeypatch, solver_fails):
    torch = pytest.importorskip("torch")
    for name, attributes in {
        "curobo.rollout.cost.pose_cost": {"PoseCostMetric": object},
        "curobo.types.math": {"Pose": lambda **kwargs: SimpleNamespace(**kwargs)},
        "curobo.geom.types": {"WorldConfig": SimpleNamespace(from_dict=lambda world: world)},
        "curobo.wrap.reacher.ik_solver": {"IKSolver": object, "IKSolverConfig": object},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)

    worlds = []
    def solve(pose, **kwargs):
        assert pose.position.numpy().reshape(3) == pytest.approx([.2, 0., 0.])
        assert worlds[-1]["sphere"]["elesim_target_object"]["radius"] == .01
        if solver_fails:
            raise CuRoboError("solver failure")
        return SimpleNamespace(success=torch.tensor([True]),
                               solution=torch.tensor([[.2, 0., 0., 0.]]))

    planner = CuRoboPlanner.__new__(CuRoboPlanner)
    planner.bounds = np.array([[-1., 1.]] * 4)
    planner.joint_names = ["linear", "roll", "b1", "b2"]
    planner._ik = SimpleNamespace(joint_names=planner.joint_names,
                                  update_world=worlds.append, solve_single=solve)
    planner._ik_tolerance = .001
    planner._lock = threading.Lock()
    planner.world = {"cuboid": {"table": {"dims": [1, 1, .1]}}}
    planner._world_config = object()
    planner.tensor_args = SimpleNamespace(to_device=lambda value: torch.as_tensor(value, dtype=torch.float32))
    planner.forward = lambda q: {"position": q[:3]}
    kwargs = dict(seed=[0.]*4, target_sphere_center=[.2, 0., 0.])
    if solver_fails:
        with pytest.raises(CuRoboError, match="solver failure"):
            planner.solve_position([.2, 0., 0.], **kwargs)
    else:
        assert planner.solve_position([.2, 0., 0.], **kwargs) == pytest.approx([.2, 0., 0., 0.])
    assert len(worlds) == 2 and worlds[-1] is planner._world_config
    assert worlds[0]["cuboid"] == planner.world["cuboid"]
    assert "sphere" not in planner.world
