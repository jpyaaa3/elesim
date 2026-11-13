"""V2-only planning process; never imports Genesis or a runtime deployment."""
from __future__ import annotations

import argparse
from collections.abc import Mapping
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from .v2_geometry import (
    budget_search_bounds, command_motor_travel, contact_release_offsets, diverse_release_starts,
    mesh_cylinder_scene, native_endpoint_bounds, normalize_commands, save, sha, small_release_offsets,
    task_velocity_limits, urdf_poses,
)
from .v2_model import require_v2


def contact_layers(scene, kinematics, contract):
    """Keep the full cylinder for protected links, a contact core for moving links.

    This is a proposal approximation. Original face/depth/force criteria are
    enforced by Genesis, including cylinder corners and the actual free object.
    Sphere geometry and self-collision pairs are never changed.
    """
    import torch

    scene.data.set_obstacle_layers('cylinder', 2)
    scene.data.set_obstacle_layers('cylinder_contact_core', 1)
    scene.data.set_obstacle_layers('floor', 3)
    data = kinematics.config.kinematics_config
    if set(contract['moving_links']) & set(contract['protected_links']):
        raise ValueError('Protected links must not receive a contact allowance')
    # Every unspecified link retains the full obstacle and floor checks.
    data.sphere_collision_mask.view(torch.int32).fill_(3)
    for name in contract['moving_links']:
        data.set_link_collision_mask(name, 1)


def plan(args):
    require_v2()
    import torch
    from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
    from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
    from curobo.types import JointState

    started = time.monotonic()
    args.output.mkdir(parents=True, exist_ok=False)
    full = args.planner_mode == 'full'
    task_profile = full and getattr(args, 'native_profile', 'default') != 'default'
    contact_profile = full and getattr(args, 'native_profile', 'default') in ('task-contact', 'task-contact-roll')
    result = dict(status='running', candidates=[], planner_mode=args.planner_mode,
                  start_offset_used=False, authored_waypoints=False if full else bool(args.release_prefix))
    def phase(name):
        result['phase'] = name
        save(args.output/'progress.json', result)
        print(json.dumps(dict(phase=name, elapsed_s=time.monotonic()-started)), flush=True)
    try:
        metadata = json.loads((args.model / 'metadata.json').read_text())
        for name, digest in metadata['artifacts'].items():
            if sha(args.model / name) != digest:
                raise ValueError(f'V2 model artifact mismatch: {name}')
        for name, digest in metadata['source_meshes'].items():
            if sha(name) != digest:
                raise ValueError(f'Original mesh changed after V2 fitting: {name}')
        query = json.loads(args.query.read_text())
        robot = yaml.safe_load((args.model / 'robot.yml').read_text())
        canonical = metadata['joint_names']
        # Keep physically possible paths while excluding poses whose minimum
        # possible command travel already exceeds the original task budget.
        search_lower, search_upper = budget_search_bounds(
            query['start'], query['home'], query['lower'], query['upper'], query['motion_budget'])
        solver_lower, solver_upper = (native_endpoint_bounds(query['start'], query['home'], search_lower, search_upper)
                                      if task_profile else (search_lower, search_upper))
        tree = ET.parse(args.model/'arm.urdf')
        if task_profile:
            nodes = [tree.getroot().find(f"joint[@name='{name}']/limit") for name in canonical]
            velocities = task_velocity_limits([float(node.get('velocity')) for node in nodes], query['rate'])
            for node, velocity in zip(nodes, velocities):
                node.set('velocity', str(float(velocity)))
        for name, lo, hi in zip(canonical, solver_lower, solver_upper):
            limit = tree.getroot().find(f"joint[@name='{name}']/limit")
            limit.set('lower', str(float(lo)))
            limit.set('upper', str(float(hi)))
        search_urdf = args.output/'search.urdf'
        tree.write(search_urdf, encoding='unicode', xml_declaration=True)
        robot['kinematics']['urdf_path'] = str(search_urdf.resolve())
        scale = np.asarray(query['motion_budget']['sim_to_motor_abs_scale'])
        limits = np.asarray(query['motion_budget']['limits_motor_deg'])
        robot['kinematics']['cspace']['cspace_distance_weight'] = (scale/limits[[0, 1, 2, 2]]).tolist()
        save(args.output/'search-policy.json', dict(lower=search_lower.tolist(), upper=search_upper.tolist(),
            native_solver_lower=solver_lower.tolist(), native_solver_upper=solver_upper.tolist(),
            cspace_distance_weight=robot['kinematics']['cspace']['cspace_distance_weight'],
            basis='Necessary minimum endpoint travel through each intermediate pose; original budgets unchanged'))
        contact = query['contact_contract'] if contact_profile else None
        scene = mesh_cylinder_scene(query['geometry'], args.output / 'cylinder.obj',
                                    contact['maximum_penetration_m'] if contact else 0.)
        save(args.output / 'scene.json', scene)
        checker = RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(
            robot_config=robot, scene_model=scene, collision_activation_distance=0.,
            self_collision_activation_distance=0., n_meshes=2, n_cuboids=2))
        if contact_profile:
            contact_layers(checker.scene_model, checker.kinematics, contact)
            save(args.output/'contact-policy.json', dict(
                **contact, full_cylinder_layer=2, moving_contact_core_layer=1, floor_layers=3,
                self_collision_unchanged=True, authored_motion=False,
                geometry='Full cylinder plus radially and axially eroded core at the original pose',
                approximation='Core clearance approximates contact; exact face/depth tests remain in Genesis'))
        names = checker.kinematics.joint_names
        if len(names) != 4 or set(names) != set(canonical):
            raise ValueError(f'Four-axis coupling mismatch: {names}')
        order = [canonical.index(n) for n in names]

        def tensor(q):
            a = np.asarray(q, dtype=np.float32)
            return torch.as_tensor(a[..., order], device='cuda').reshape(1, -1, 4).contiguous()

        # Independent all-link FK checks catch root-frame and mimic conversion errors.
        parity = {}
        endpoints = {}
        for label in ('start', 'home'):
            q = query[label]
            state = checker.get_kinematics(tensor(q))
            expected = urdf_poses(args.model / 'arm.urdf', canonical, q)
            errors = {}
            for link in state.tool_frames:
                pose = state.tool_poses.get_link_pose(link)
                pos = pose.position.detach().cpu().numpy().reshape(-1, 3)[0]
                quat = pose.quaternion.detach().cpu().numpy().reshape(-1, 4)[0]
                rot = Rotation.from_quat(np.roll(quat, -1)).as_matrix()
                errors[link] = dict(position_m=float(np.linalg.norm(pos - expected[link][:3, 3])),
                                    rotation_matrix_max=float(np.max(np.abs(rot - expected[link][:3, :3]))))
            parity[label] = errors
            if max(e['position_m'] for e in errors.values()) > 1e-5 or max(e['rotation_matrix_max'] for e in errors.values()) > 1e-5:
                save(args.output / 'fk-parity.json', parity)
                raise ValueError('V2 FK does not match the independent URDF reference')
            valid = bool(checker.validate(tensor(q)).all().item())
            world, self_ = checker.get_scene_self_collision_distance_from_joints(tensor(q))
            endpoints[label] = dict(valid=valid, scene_max=float(world.max().item()),
                                    self_max=float(self_.max().item()))
        save(args.output / 'fk-parity.json', parity)
        save(args.output / 'endpoints.json', endpoints)
        result['endpoints'] = endpoints
        result['setup_s'] = time.monotonic() - started
        phase('endpoints_checked')
        starts = np.asarray([query['start']])
        if not full and not endpoints['home']['valid']:
            result['status'] = 'planning_endpoint_invalid'
            return result
        offset_used = not full and (args.release_prefix or not endpoints['start']['valid'])
        if offset_used:
            if args.planner_mode != 'rrt':
                result['status'] = 'planning_endpoint_invalid'
                return result
            offsets = (contact_release_offsets(query['start'], query['home'], search_lower, search_upper)
                       if args.release_prefix else small_release_offsets(query['start'], search_lower, search_upper))
            valid = checker.validate(tensor(offsets)).detach().cpu().numpy().reshape(-1)
            starts = (diverse_release_starts(offsets[valid], query['start'])
                      if args.release_prefix else offsets[valid][:8])
            save(args.output / 'start-offsets.json', dict(original=query['start'],
                tested=len(offsets), feasible=int(np.count_nonzero(valid)), selected=starts.tolist(),
                prefix_validation='Original admission and contact-release prefix validated by Genesis',
                policy='executed contact-release prefix' if args.release_prefix else 'small endpoint offset',
                object_pose_changed=False))
            if not len(starts):
                result['status'] = 'no_feasible_start_offset'
                return result
        result['start_offset_used'] = offset_used
        result['authored_waypoints'] = offset_used

        # Planning needs one tool; the independent checker retains every link.
        robot['kinematics']['tool_frames'] = ['elesim_grasp']
        t = time.monotonic()
        phase('planner_initializing')
        if full:
            # Native defaults: PRM, B-spline TrajOpt and timing optimization.
            # Give cuRobo the original endpoints, including any endpoint the
            # independent checker dislikes, and retain its own failure result.
            extra = (dict(graph_planner_algorithm='rrt', num_knots=64,
                          rrt_planner_config={'cspace_resolution': .005},
                          num_trajopt_seeds=8, num_ik_seeds=16,
                          optimizer_collision_activation_distance=.001) if task_profile else {})
            cfg = MotionPlannerCfg.create(
                robot=robot, scene_model=scene, collision_cache={'mesh': 2, 'primitive': 2},
                interpolation_dt=.01, random_seed=int(query['seed']), **extra)
            if task_profile:
                # Increase native position-bound cost, without clipping poses
                # or changing any acceptance constraint or motion phase.
                for rollout in cfg.trajopt_solver_config.optimizer_rollout_configs:
                    if rollout.cost_cfg is not None and rollout.cost_cfg.cspace_cfg is not None:
                        rollout.cost_cfg.cspace_cfg.weight[0] = 1e6
            save(args.output/'planning-policy.json', dict(
                original_start=query['start'], goal=query['home'],
                authored_waypoints=[], automatic_start_offset=False,
                graph_planner_algorithm=cfg.graph_planner_algorithm,
                enable_trajopt=cfg.enable_trajopt,
                always_run_trajopt=cfg.always_run_trajopt,
                run_trajopt_on_graph_seed=cfg.run_trajopt_on_graph_seed,
                native_profile=args.native_profile,
                task_velocity_limits=velocities.tolist() if task_profile else None,
                num_knots=cfg.num_knots,
                interpolation_capacity=cfg.trajopt_solver_config.interpolation_buffer_size,
                position_bound_cost_weight=1e6 if task_profile else 'native_default',
                endpoint_numerical_slack_only=task_profile,
                original_command_bounds_preserved=True,
                command_retiming='uniform slowdown only'))
        else:
            cfg = MotionPlannerCfg.create(
                robot=robot, scene_model=scene, collision_cache={'mesh': 2, 'primitive': 2},
                graph_planner_algorithm='rrt', num_trajopt_seeds=8, num_ik_seeds=16,
                rrt_planner_config={'cspace_resolution': .005} if args.planner_mode == 'rrt' else None,
                interpolation_dt=.01, optimizer_collision_activation_distance=.001,
                random_seed=int(query['seed']), num_knots=32,
                always_run_trajopt=False, run_trajopt_on_graph_seed=False,
                enable_trajopt=args.planner_mode == 'spline')
        planner = MotionPlanner(cfg)
        if contact_profile:
            rollouts = (planner.trajopt_solver.get_all_rollout_instances()
                        + planner.ik_solver.get_all_rollout_instances()
                        + planner.graph_planner.resources.get_all_rollout_instances()
                        + list(planner.trajopt_solver.additional_metrics_rollouts.values()))
            kinematics = [planner.kinematics]
            for rollout in rollouts:
                kinematics.extend((rollout.transition_model.robot_model,
                                   rollout.metrics_transition_model.robot_model))
            for kin in kinematics:
                contact_layers(planner.scene_collision_checker, kin, contact)
        if args.native_profile == 'task-contact-roll':
            # Search a monotone-roll subset of the original task. Keep model
            # speeds symmetric for native TimeOpt; constrain signed velocity
            # in TrajOpt costs and its dense acceptance rollout instead.
            from curobo._src.cost.cost_cspace_type import CSpaceCostType
            changed = 0
            for rollout in rollouts:
                for cost in rollout.get_cost_component_by_name('cspace'):
                    if cost.config.cost_type != CSpaceCostType.STATE:
                        continue
                    bounds = cost.config.joint_limits.clone()
                    index = bounds.joint_names.index(canonical[1])
                    bounds.velocity[0, index] = -5e-6
                    cost.config.joint_limits = bounds
                    cost.config.activation_distance[1] = 0.
                    cost.config.weight[1] = 1e6
                    cost._weight[1] = 1e6
                    changed += 1
            if not changed:
                raise RuntimeError('No native state costs received the Roll constraint')
            save(args.output/'roll-policy.json', dict(
                signed_velocity_lower_rad_s=-5e-6, state_costs_updated=changed,
                search_subset='Monotone Roll throughout; original task allows reversals after unlock',
                command_direction_acceptance='Original GA check; only existing float32 jitter tolerance',
                native_timeopt_enabled=True, authored_waypoints=[]))
        if args.planner_mode == 'rrt' or task_profile:
            resources = planner.graph_planner.resources
            weights = torch.as_tensor(
                np.asarray(robot['kinematics']['cspace']['cspace_distance_weight'])[order],
                dtype=torch.float32, device='cuda')
            # The pinned V2 RRT resources initialize their own metric to ones,
            # independently of the robot's cspace weight. Apply the task metric
            # before warmup/connector allocation. Its weighted span fits inside
            # the already allocated direct-edge steering capacity.
            span = resources.action_bound_highs-resources.action_bound_lows
            if (span*weights).abs().max() > (span*resources.cspace_distance_weight).abs().max()+1e-6:
                raise ValueError('Requested graph metric exceeds allocated direct-edge capacity')
            resources.cspace_distance_weight.copy_(weights)
            policy_path = args.output/'search-policy.json'
            policy = json.loads(policy_path.read_text())
            policy.update(effective_graph_distance_weight=resources.cspace_distance_weight.detach().cpu().tolist(),
                          graph_cspace_resolution=planner.graph_planner.config.cspace_resolution)
            save(policy_path, policy)
        def js(q):
            values = np.asarray(q, dtype=np.float32)[order]
            return JointState.from_position(torch.as_tensor(values, device='cuda').reshape(1, 4), joint_names=names)

        # Warm exactly the joint-space pipeline used by this experiment.
        # The generic warmup additionally compiles pose IK, which is not used.
        if args.planner_mode != 'rrt':
            planner.plan_cspace(js(query['home']), js(query['home']), max_attempts=1,
                                enable_graph_attempt=0, finetune_attempts=0)
        else:
            planner.graph_planner.warmup(num_warmup_iterations=1, max_batch_size=8)
        torch.cuda.synchronize()
        result['planner_initialization_s'] = time.monotonic() - t
        phase('planning')

        for attempt in range(args.attempts):
            folder = args.output / f'candidate-{attempt:04d}'
            folder.mkdir()
            t = time.monotonic()
            if args.planner_mode == 'rrt':
                lane_starts = np.asarray([starts[(attempt*8+lane) % len(starts)] for lane in range(8)])
                graph = planner.graph_planner.find_path(
                    torch.as_tensor(lane_starts[:, order], dtype=torch.float32, device='cuda').contiguous(),
                    js(query['home']).position.expand(8, -1).contiguous(),
                    query_mask=torch.ones(8, dtype=torch.bool, device='cuda'), group_count=1,
                    interpolate_waypoints=False)
                torch.cuda.synchronize()
                entry = dict(attempt=attempt, planning_s=time.monotonic()-t,
                             trajectory_source='V2 RRT polyline; task-rate timing; no spline smoothing',
                             graph_success=bool(graph.success.any().item()), lanes=[])
                paths = []
                for lane in range(8):
                    if not bool(graph.success[lane].item()):
                        continue
                    raw = graph.plan_waypoints[lane].detach().cpu().numpy()
                    graph_names = list(graph.joint_names)
                    raw = raw[:, [graph_names.index(n) for n in canonical]]
                    prefix_duration = 0.
                    if offset_used:
                        prefix = [query['start']]
                        if args.release_prefix:
                            # Open first, then retract/roll at the selected
                            # opening. Contact is checked throughout in Genesis.
                            prefix.append([*query['start'][:2], *lane_starts[lane, 2:]])
                        raw = np.vstack((prefix, raw))
                        delta = np.diff(raw[:len(prefix)+1], axis=0)
                        prefix_duration = float(np.maximum(.01, np.max(np.abs(delta)/query['rate'], axis=1)*.01).sum())
                    np.savez_compressed(folder/f'raw-lane-{lane}.npz', positions=raw)
                    try:
                        commands, timing = normalize_commands(
                            raw, .01, query['start'], query['home'], query['lower'], query['upper'], query['rate'])
                        travel = command_motor_travel(commands, query['motion_budget'])
                        if np.any(travel > np.asarray(query['motion_budget']['limits_motor_deg'])+1e-6):
                            raise ValueError(f'Command motor travel exceeds task budget: {travel.tolist()}')
                        suffix_index = int(np.ceil(prefix_duration/.01-1e-8))
                        if not bool(checker.validate(tensor(commands[suffix_index:])).all().item()):
                            raise ValueError('RRT commands fail independent dense collision validation')
                        np.savez_compressed(folder/f'commands-lane-{lane}.npz', commands=commands[None])
                        paths.append(lane)
                        entry['lanes'].append(dict(lane=lane, status='candidate', timing=timing,
                            offset=(lane_starts[lane]-query['start']).tolist(),
                            contact_prefix_s=prefix_duration,
                            prefix_policy='open bends, then retract/roll' if args.release_prefix else 'small offset'))
                    except ValueError as exc:
                        entry['lanes'].append(dict(lane=lane, status='command_rejected', error=str(exc)))
                entry['status'] = 'candidate' if paths else 'planner_failed'
                result['candidates'].append(entry)
                save(folder/'planning.json', entry)
                save(args.output/'progress.json', result)
                print(json.dumps(entry), flush=True)
                continue
            solved = planner.plan_cspace(js(query['home']), js(query['start']), max_attempts=4,
                                         enable_graph_attempt=1, finetune_attempts=3 if full else 1)
            torch.cuda.synchronize()
            entry = dict(attempt=attempt, planning_s=time.monotonic() - t,
                         internal_attempts=planner.last_planning_attempts,
                         original_start_used=True, authored_waypoints=[],
                         graph_success=planner.last_graph_success,
                         graph_time_s=planner.last_graph_time,
                         graph_telemetry=[{k: dict(v) if isinstance(v, Mapping) else v
                                           for k, v in vars(r).items()}
                                          for r in planner.last_graph_call_telemetry])
            if solved is not None:
                entry['feasible'] = None if solved.feasible is None else solved.feasible.detach().cpu().tolist()
                entry['cspace_error'] = None if solved.cspace_error is None else solved.cspace_error.detach().cpu().tolist()
                entry['validation'] = {name: value.detach().cpu().tolist()
                                       for name, value in solved.debug_info.get('validation', {}).items()}
            if getattr(args, 'explain_failure', False):
                from .native_failure import explain
                entry['native_failure_detail'] = explain(planner, solved)
                save(folder/'native-failure.json', entry['native_failure_detail'])
            if solved is None or not bool(solved.success.any().item()):
                entry['status'] = 'planner_failed'
                if solved is not None and getattr(args, 'capture_rejected', False):
                    entry['diagnostic_only_trajectories'] = []
                    for label, trajectory in (('interpolated', solved.interpolated_trajectory),
                                              ('native', solved.js_solution)):
                        if trajectory is None or trajectory.position is None:
                            continue
                        positions = trajectory.position.detach().cpu().numpy()
                        positions = positions.reshape(-1, positions.shape[-2], positions.shape[-1])[0]
                        if label == 'interpolated' and solved.interpolated_last_tstep is not None:
                            count = int(solved.interpolated_last_tstep.detach().cpu().reshape(-1)[0])
                            positions = positions[:max(1, min(count, len(positions)))]
                        trajectory_names = list(trajectory.joint_names)
                        positions = positions[:, [trajectory_names.index(n) for n in canonical]]
                        raw_dt = (np.asarray(np.nan) if trajectory.dt is None else
                                  trajectory.dt.detach().cpu().numpy().reshape(-1)[0])
                        name = f'rejected-{label}.npz'
                        np.savez_compressed(folder/name, positions=positions, dt_s=raw_dt,
                                            diagnostic_only=True, native_success=False)
                        entry['diagnostic_only_trajectories'].append(dict(
                            file=name, finite=bool(np.isfinite(positions).all()), samples=len(positions)))
            else:
                trajectory = solved.get_interpolated_plan()
                p = trajectory.position.detach().cpu().numpy().reshape(-1, 4)
                trajectory_names = list(trajectory.joint_names)
                p = p[:, [trajectory_names.index(n) for n in canonical]]
                source_dt = float(trajectory.dt.detach().cpu().reshape(-1)[0])
                np.savez_compressed(folder / 'raw-trajectory.npz', positions=p, dt_s=source_dt)
                try:
                    commands, timing = normalize_commands(
                        p, source_dt, query['start'], query['home'], query['lower'], query['upper'], query['rate'],
                        retiming='uniform' if full else 'segment')
                    travel = command_motor_travel(commands, query['motion_budget'])
                    if np.any(travel > np.asarray(query['motion_budget']['limits_motor_deg'])+1e-6):
                        raise ValueError(f'Command motor travel exceeds task budget: {travel.tolist()}')
                    if not bool(checker.validate(tensor(commands)).all().item()):
                        raise ValueError('Normalized commands fail independent dense collision validation')
                    np.savez_compressed(folder / 'commands.npz', commands=commands[None])
                    entry.update(status='candidate', command_timing=timing,
                                 trajectory_source=str(solved.trajectory_source))
                except ValueError as exc:
                    entry.update(status='command_rejected', error=str(exc))
            result['candidates'].append(entry)
            save(folder / 'planning.json', entry)
            save(args.output / 'progress.json', result)
            print(json.dumps(entry), flush=True)
        result['status'] = 'candidates_generated' if any(x['status'] == 'candidate' for x in result['candidates']) else 'budget_exhausted'
        planner.destroy()
        return result
    except BaseException as exc:
        result.update(status='execution_error', error=repr(exc))
        raise
    finally:
        result['elapsed_s'] = time.monotonic() - started
        save(args.output / 'completion.json', result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--query', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--attempts', type=int, default=4)
    parser.add_argument('--planner-mode', choices=('full', 'spline', 'rrt'), default='spline')
    parser.add_argument('--release-prefix', action='store_true', help='Search executed contact release before the V2 suffix')
    parser.add_argument('--capture-rejected', action='store_true',
                        help='Save rejected native outputs for diagnostic preview only; never create commands from them')
    parser.add_argument('--native-profile', choices=('default', 'task', 'task-contact', 'task-contact-roll'), default='default')
    parser.add_argument('--explain-failure', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.attempts <= 16:
        parser.error('attempts must be 1..16')
    if args.release_prefix and args.planner_mode != 'rrt':
        parser.error('--release-prefix requires --planner-mode rrt')
    if args.native_profile != 'default' and args.planner_mode != 'full':
        parser.error('Non-default native profiles require --planner-mode full')
    result = plan(args)
    return 0 if result['status'] == 'candidates_generated' else 1


if __name__ == '__main__':
    raise SystemExit(main())
