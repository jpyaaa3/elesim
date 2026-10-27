"""Optional, local CUDA planning backend. This module never sends motor commands.

All inputs are canonical q=(linear metres, roll, bend1, bend2 radians).
World geometry and Cartesian targets are expressed in the arm's plate frame.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
import importlib.metadata
import hashlib
import json
from pathlib import Path
import threading

import numpy as np
import yaml


class CuRoboError(RuntimeError):
    pass


@dataclass(frozen=True)
class JointTrajectory:
    positions: tuple[tuple[float, float, float, float], ...]
    dt_s: float


class CuRoboPlanner:
    """cuRobo v0.7.8 adapter for EleSim's mechanically coupled four-axis arm.

    Construction and CUDA warmup are explicit; importing Pilot needs no cuRobo.
    Failure never silently falls back to unchecked interpolation.
    """

    def __init__(self, model_dir: str | Path, *, world: dict,
                 interpolation_dt: float = 0.05, use_cuda_graph: bool = True) -> None:
        self.model_dir = Path(model_dir).resolve()
        self.metadata = json.loads((self.model_dir / "metadata.json").read_text())
        if self.metadata.get("schema_version") != 1:
            raise CuRoboError("unsupported cuRobo model schema")
        for name in ("arm.urdf", "robot.yml"):
            if hashlib.sha256((self.model_dir / name).read_bytes()).hexdigest() != self.metadata.get("artifacts", {}).get(name):
                raise CuRoboError(f"planning artifact changed; regenerate the model: {name}")
        self.joint_names = list(self.metadata["joint_names"])
        if len(self.joint_names) != 4 or len(set(self.joint_names)) != 4:
            raise CuRoboError("planning model must have exactly four independent axes")
        self.bounds = np.asarray(self.metadata["q_bounds"], dtype=float)
        if self.bounds.shape != (4, 2) or not np.isfinite(self.bounds).all():
            raise CuRoboError("invalid planning bounds")
        if np.any(self.bounds[:, 0] >= self.bounds[:, 1]):
            raise CuRoboError("planning bounds must have positive spans")
        if not isinstance(world, dict):
            raise CuRoboError("world must be a mapping; use {} for an explicitly empty world")
        if not 0.01 <= float(interpolation_dt) <= 0.1:
            raise ValueError("interpolation_dt must be 0.01..0.1 seconds")
        try:
            version = importlib.metadata.version("nvidia-curobo")
        except importlib.metadata.PackageNotFoundError as exc:
            raise CuRoboError("install the documented Jetson cuRobo v0.7.8 environment") from exc
        if version != "0.7.8+elesim.1":
            raise CuRoboError(f"cuRobo 0.7.8+elesim.1 required (run the Jetson setup script), found {version}")
        import torch
        if not torch.cuda.is_available():
            raise CuRoboError("cuRobo requires an NVIDIA CUDA GPU")
        from curobo.types.base import TensorDeviceType
        from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
        from curobo.rollout.arm_base import ArmBase, ArmBaseConfig

        self.tensor_args = TensorDeviceType()
        raw = yaml.safe_load((self.model_dir / "robot.yml").read_text())["robot_cfg"]
        raw["kinematics"]["urdf_path"] = str(self.model_dir / "arm.urdf")
        raw["kinematics"]["asset_root_path"] = str(self.model_dir)
        self.robot_config = raw
        sphere_count = sum(map(len, raw["kinematics"]["collision_spheres"].values()))
        if not 0 < sphere_count < 1024:
            raise CuRoboError("cuRobo v0.7.8 requires 1..1023 collision spheres")
        self.world = world
        self._ik_tolerance = None
        self.use_cuda_graph = bool(use_cuda_graph)
        self._lock = threading.Lock()
        self.motion = MotionGen(MotionGenConfig.load_from_robot_config(
            raw, world, tensor_args=self.tensor_args, use_cuda_graph=self.use_cuda_graph,
            collision_cache={"obb": 32, "mesh": 16},
            cspace_threshold=0.0001,
            interpolation_dt=float(interpolation_dt),
        ))
        self._order = self._joint_order(self.motion.kinematics.joint_names)
        if not self.motion.kinematics.get_self_collision_config().experimental_kernel:
            raise CuRoboError("parallel self-collision kernel required; reinstall the pinned patched cuRobo")
        # Validation changes batch/horizon shapes. Keep its tensors separate
        # from optimizer/graph-planner buffers captured by CUDA graphs.
        safety = self.motion.rollout_fn
        self._validator = ArmBase(ArmBaseConfig(**{
            field.name: getattr(safety, field.name) for field in fields(ArmBaseConfig)
        }))
        self._ik = None

    def _joint_order(self, names):
        if set(names) != set(self.joint_names) or len(names) != 4:
            raise CuRoboError(f"independent joint mapping mismatch: {names}")
        return [self.joint_names.index(name) for name in names]

    def _q(self, values):
        q = np.asarray(values, dtype=float)
        if q.shape != (4,) or not np.isfinite(q).all():
            raise CuRoboError("q must contain four finite canonical joint values")
        if np.any(q < self.bounds[:, 0] - 1e-7) or np.any(q > self.bounds[:, 1] + 1e-7):
            raise CuRoboError("q exceeds the model's operating limits")
        return q

    def _tensor(self, q, order):
        return self.tensor_args.to_device(np.asarray(q)[order]).reshape(1, 4).contiguous()

    def forward(self, q) -> dict[str, list[float]]:
        """Grasp-frame pose in plate coordinates, suitable for parity checks."""
        q = self._q(q)
        with self._lock:
            state = self._validator.kinematics.get_state(self._tensor(q, self._order))
        return {"position": state.ee_position[0].detach().cpu().tolist(),
                "quaternion_wxyz": state.ee_quaternion[0].detach().cpu().tolist()}

    def validate_configuration(self, q) -> None:
        q = self._q(q)
        self.validate_path(q.reshape(1, 4))

    def validate_path(self, positions) -> None:
        from curobo.types.state import JointState
        positions = np.asarray(positions, dtype=float)
        if positions.ndim != 2 or positions.shape[1] != 4 or not 1 <= len(positions) <= 20000:
            raise CuRoboError("path must have 1..20000 four-axis waypoints")
        if not np.isfinite(positions).all():
            raise CuRoboError("path contains nonfinite joint values")
        if np.any(positions < self.bounds[:, 0] - 1e-7) or np.any(positions > self.bounds[:, 1] + 1e-7):
            raise CuRoboError("path exceeds the model's operating limits")
        with self._lock:
            # v0.7.8 check_start_state temporarily toggles costs and can return
            # early with self-collision disabled. Query constraints directly.
            for index in range(0, len(positions), 256):
                chunk = self.tensor_args.to_device(positions[index:index + 256, self._order]).contiguous()
                # These are poses, not control actions. rollout_constraint
                # integrates from its stored start state and can interpret a
                # static pose as an instantaneous velocity/acceleration jump.
                state = self._validator.compute_kinematics(
                    JointState.from_position(chunk.unsqueeze(0), joint_names=self.motion.kinematics.joint_names))
                # Joint bounds were checked above. Static poses must not be
                # subjected to the optimizer's derivative/null-space costs.
                world_cost = self._validator.primitive_collision_constraint.forward(state.robot_spheres, env_query_idx=None)
                self_cost = self._validator.robot_self_collision_constraint.forward(state.robot_spheres)
                valid = (world_cost.isfinite().all() & self_cost.isfinite().all() &
                         (world_cost == 0).all() & (self_cost == 0).all())
                if not bool(valid.item()):
                    costs = dict(world=float(world_cost.max().item()),
                                 self_collision=float(self_cost.max().item()))
                    raise CuRoboError(f"configuration violates joint or collision constraints: {costs}")

    def plan_joint(self, start, goal) -> JointTrajectory:
        from curobo.types.state import JointState
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig

        left, right = self._q(start), self._q(goal)
        self.validate_configuration(left)
        self.validate_configuration(right)
        names = self.motion.kinematics.joint_names
        start_state = JointState.from_position(self._tensor(left, self._order), joint_names=names)
        goal_state = JointState.from_position(self._tensor(right, self._order), joint_names=names)
        with self._lock:
            try:
                # Both endpoints were independently checked above. The legacy
                # duplicate check can include optimizer state/derivative costs.
                result = self.motion.plan_single_js(start_state, goal_state,
                                                   MotionGenPlanConfig(max_attempts=4, check_start_validity=False))
            finally:
                self.motion.rollout_fn.primitive_collision_constraint.enable_cost()
                self.motion.rollout_fn.robot_self_collision_constraint.enable_cost()
        if not bool(result.success.item()):
            raise CuRoboError(f"cuRobo planning failed: {result.status}")
        plan = result.get_interpolated_plan()
        actual_names = list(plan.joint_names)
        if set(actual_names) != set(self.joint_names):
            raise CuRoboError("trajectory joint names do not match the four-axis model")
        positions = plan.position.detach().cpu().numpy()
        positions = positions[:, [actual_names.index(name) for name in self.joint_names]]
        if not 2 <= len(positions) <= 20000:
            raise CuRoboError("invalid or oversized trajectory")
        self.validate_path(positions)
        if not np.allclose(positions[0], left, atol=1e-4) or not np.allclose(positions[-1], right, atol=1e-3):
            raise CuRoboError("trajectory endpoints do not match the request")
        dt = float(result.interpolation_dt)
        if not np.isfinite(dt) or dt <= 0:
            raise CuRoboError("invalid trajectory timing")
        velocity = np.abs(np.diff(positions, axis=0)) / dt
        if np.any(velocity > np.asarray(self.metadata["max_velocity"]) * 1.05 + 1e-5):
            raise CuRoboError("trajectory exceeds configured Robot velocity limits")
        return JointTrajectory(tuple(tuple(map(float, q)) for q in positions), dt)

    def solve_position(self, target_base, *, seed, tolerance_m: float = 0.001):
        """Position-only IK: the four-axis arm cannot satisfy an arbitrary 6D pose."""
        from curobo.rollout.cost.pose_cost import PoseCostMetric
        from curobo.types.math import Pose
        from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

        target = np.asarray(target_base, dtype=float)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise CuRoboError("target must contain three finite plate-frame coordinates")
        if not np.isfinite(tolerance_m) or not 1e-5 <= tolerance_m <= 0.02:
            raise ValueError("IK tolerance must be 0.00001..0.02 metres")
        q = self._q(seed)
        with self._lock:
            if self._ik is None or self._ik_tolerance != tolerance_m:
                self._ik = IKSolver(IKSolverConfig.load_from_robot_config(
                    self.robot_config, self.world, tensor_args=self.tensor_args,
                    use_cuda_graph=self.use_cuda_graph, num_seeds=32, position_threshold=tolerance_m,
                    collision_cache={"obb": 32, "mesh": 16},
                    high_precision=tolerance_m < 0.001,
                ))
                self._ik.update_pose_cost_metric(PoseCostMetric(
                    reach_partial_pose=True,
                    reach_vec_weight=self.tensor_args.to_device([0, 0, 0, 1, 1, 1]),
                ))
                self._ik_tolerance = tolerance_m
            order = self._joint_order(self._ik.joint_names)
            result = self._ik.solve_single(Pose(
                position=self.tensor_args.to_device(target).reshape(1, 3),
                quaternion=self.tensor_args.to_device([1., 0., 0., 0.]).reshape(1, 4)),
                retract_config=self._tensor(q, order),
                seed_config=self._tensor(q, order).unsqueeze(0))
            if not bool(result.success.reshape(-1)[0].item()):
                raise CuRoboError("cuRobo position IK failed")
            solution = result.solution.reshape(-1, 4)[0].detach().cpu().numpy()
            answer = self._q(solution[np.argsort(order)])
        if np.linalg.norm(np.asarray(self.forward(answer)["position"]) - target) > tolerance_m:
            raise CuRoboError("IK result failed independent position tolerance check")
        return tuple(map(float, answer))


def main() -> None:
    """Offline planning entrypoint. No DDS or hardware modules are initialized."""
    import argparse
    from dataclasses import asdict
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--world", type=Path, required=True,
                        help="cuRobo world YAML in the plate frame; use {} for explicitly empty world")
    parser.add_argument("--start", type=float, nargs=4, required=True)
    parser.add_argument("--goal", type=float, nargs=4)
    parser.add_argument("--target", type=float, nargs=3)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.goal is not None and args.target is not None:
        parser.error("choose --goal or --target")
    planner = CuRoboPlanner(args.model, world=yaml.safe_load(args.world.read_text()))
    goal = args.goal
    if args.target is not None:
        goal = planner.solve_position(args.target, seed=args.start)
    result = {"schema_version": 1, "frame": "plate", "units": ["m", "rad", "rad", "rad"],
              "joint_names": planner.joint_names,
              "model_sha256": hashlib.sha256((planner.model_dir / "metadata.json").read_bytes()).hexdigest(),
              "start_fk": planner.forward(args.start)}
    if goal is not None:
        result["trajectory"] = asdict(planner.plan_joint(args.start, goal))
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(args.output)


if __name__ == "__main__":
    main()
