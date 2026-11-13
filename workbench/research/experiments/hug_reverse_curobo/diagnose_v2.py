"""Validate a saved physical reference against V2; never generate proposals from it."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import yaml

from .v2_geometry import save, sha
from .v2_model import require_v2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--case', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True, help='GA winner states.npz, for validation only')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    require_v2()
    import torch
    from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
    from curobo.motion_planner import MotionPlanner, MotionPlannerCfg

    args.output.mkdir(parents=True, exist_ok=False)
    metadata = json.loads((args.model/'metadata.json').read_text())
    for name, digest in metadata['artifacts'].items():
        if sha(args.model/name) != digest:
            raise ValueError(f'Model artifact mismatch: {name}')
    query = json.loads((args.case/'query.json').read_text())
    robot = yaml.safe_load((args.model/'robot.yml').read_text())
    robot['kinematics']['urdf_path'] = str((args.case/'planner/search.urdf').resolve())
    scene = json.loads((args.case/'planner/scene.json').read_text())
    checker = RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(
        robot_config=robot, scene_model=scene, collision_activation_distance=0.,
        self_collision_activation_distance=0., n_meshes=2, n_cuboids=2))
    canonical = metadata['joint_names']
    names = checker.kinematics.joint_names
    order = [canonical.index(name) for name in names]
    with np.load(args.reference, allow_pickle=False) as data:
        commands = data['command'].reshape(-1, 4).copy()
        times = data['time_s'].copy()
    robot['kinematics']['tool_frames'] = ['elesim_grasp']
    planner = MotionPlanner(MotionPlannerCfg.create(
        robot=robot, scene_model=scene, collision_cache={'mesh': 2, 'primitive': 2},
        graph_planner_algorithm='rrt', enable_trajopt=False, num_trajopt_seeds=8,
        num_ik_seeds=16, num_knots=32, interpolation_dt=.01,
        optimizer_collision_activation_distance=.001,
        always_run_trajopt=False, run_trajopt_on_graph_seed=False))
    resources = planner.graph_planner.resources
    points = np.vstack((query['start'], query['home'], commands))
    valid, graph_valid, world_max, self_max = [], [], [], []
    constraints = {}
    for begin in range(0, len(points), 64):
        tensor = torch.as_tensor(points[begin:begin+64, order], dtype=torch.float32,
                                 device='cuda').contiguous()
        collision_valid = checker.validate(tensor[None]).detach().cpu().numpy().reshape(-1)
        world, self_ = checker.get_scene_self_collision_distance_from_joints(tensor[None])
        world_max.extend(world.detach().cpu().numpy().reshape(len(tensor), -1).max(-1).tolist())
        self_max.extend(self_.detach().cpu().numpy().reshape(len(tensor), -1).max(-1).tolist())
        valid.extend(collision_valid.tolist())
        metrics = resources.feasibility_rollout.compute_metrics_from_action(tensor[:, None])
        graph_valid.extend(metrics.costs_and_constraints.get_feasible(
            include_all_hybrid=False, sum_horizon=True).detach().cpu().numpy().reshape(-1).tolist())
        for collection in ('constraints', 'hybrid_costs_constraints'):
            values = getattr(metrics.costs_and_constraints, collection)
            for name, value in zip(values.names, values.values):
                key = f'{collection}/{name}'
                constraints.setdefault(key, []).extend(
                    value.detach().cpu().numpy().reshape(len(tensor), -1).max(-1).tolist())
    result = dict(reference=str(args.reference), reference_sha256=sha(args.reference),
                  role='Independent validation only; no reference commands enter the proposal generator',
                  endpoint_collision_valid=valid[:2], endpoint_graph_valid=graph_valid[:2],
                  graph_distance_weight=resources.cspace_distance_weight.detach().cpu().tolist(),
                  graph_lower=resources.action_bound_lows.detach().cpu().tolist(),
                  graph_upper=resources.action_bound_highs.detach().cpu().tolist(),
                  reference_samples=len(commands), collision_valid_samples=int(np.count_nonzero(valid[2:])),
                  graph_valid_samples=int(np.count_nonzero(graph_valid[2:])),
                  constraints={key: dict(endpoint_values=value[:2], maximum=max(value),
                                         positive_reference_samples=int(np.count_nonzero(np.asarray(value[2:]) > 0)))
                               for key, value in constraints.items()})
    for label, values in (('scene', world_max), ('self', self_max)):
        indices = np.flatnonzero(np.asarray(values[2:]) > 0)
        result[f'{label}_collision_time_range_s'] = [float(times[indices[0]]), float(times[indices[-1]])] if len(indices) else None
    result['per_obstacle'] = {}
    for kind, obstacles in scene.items():
        for name, obstacle in obstacles.items():
            isolated = RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(
                robot_config=robot, scene_model={kind: {name: obstacle}},
                collision_activation_distance=0., self_collision_activation_distance=0.,
                n_meshes=2, n_cuboids=2))
            collisions = []
            for begin in range(0, len(commands), 64):
                tensor = torch.as_tensor(commands[begin:begin+64, order], dtype=torch.float32,
                                         device='cuda').contiguous()
                world, _ = isolated.get_scene_self_collision_distance_from_joints(tensor[None])
                collisions.extend((world.detach().cpu().numpy().reshape(len(tensor), -1).max(-1) > 0).tolist())
            indices = np.flatnonzero(collisions)
            result['per_obstacle'][name] = dict(collision_samples=len(indices),
                time_range_s=[float(times[indices[0]]), float(times[indices[-1]])] if len(indices) else None)
    save(args.output/'report.json', result)
    np.savez_compressed(args.output/'reference-validation.npz', time_s=times,
                        collision_valid=valid[2:], graph_valid=graph_valid[2:],
                        world_max=world_max[2:], self_max=self_max[2:])
    print(json.dumps(result, indent=2), flush=True)
    planner.destroy()


if __name__ == '__main__':
    main()
