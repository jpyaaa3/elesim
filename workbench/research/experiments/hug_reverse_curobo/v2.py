"""Run isolated V2 proposals against the unchanged reverse-GA physics contract."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np

from .v2_geometry import save, sha


def run_case(plan_path, output, args):
    from workbench.research.experiments.hug_c.configure import asset_digest
    from workbench.research.experiments.hug_c.environment import HugEnv
    from workbench.research.experiments.hug_c.experiment import check_versions
    from workbench.research.experiments.hug_c.metrics import local_link_index
    from workbench.research.experiments.hug_reverse_ga.prepare import digest
    from workbench.research.experiments.hug_reverse_ga.physics import admit, arrays, evaluate, sample
    from workbench.research.experiments.hug_reverse_ga.motion_budget import require_feasible

    plan = json.loads(plan_path.read_text())
    if (plan.get('schema') != 'hug-reverse-ga-plan' or plan.get('version') != 1
            or plan['source_sha256'] != digest()
            or plan['simulation_context']['model_sha256'] != asset_digest()):
        raise ValueError('Reverse-GA source/model mismatch; do not relabel an old plan')
    check_versions()
    if os.environ.get('CUDA_VISIBLE_DEVICES') != args.gpu:
        raise ValueError('CUDA_VISIBLE_DEVICES must exactly match the selected GPU UUID')
    case = plan['cases'][0]
    require_feasible(case['goal_command'], plan['home'])
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    result = dict(status='running', forward_success=False,
                  lift_success=False, reset_pool_validated=False, automatic_ppo_launch=False)
    try:
        save(output / 'provenance.json', dict(plan=plan, plan_sha256=sha(plan_path),
             model=str(args.model.resolve()), model_sha256=sha(args.model / 'metadata.json'),
             gpu_uuid=args.gpu, planner='cuRobo V2 v0.9.0',
             planner_mode=args.planner_mode,
             release_prefix=args.release_prefix,
             authored_waypoints_allowed=args.planner_mode != 'full',
             automatic_start_offset_allowed=args.planner_mode != 'full',
             native_profile=args.native_profile,
             source_sha256={p.name: sha(p) for p in Path(__file__).parent.glob('v2*.py')},
             physics_python=sys.executable, planner_python=str(args.v2_python),
             assigned_roles='research only; no runtime deployment',
             planner_world='Measured stationary cylinder mesh and floor in plate coordinates; Go2 checked in Genesis'))
        env = HugEnv(plan, case, output / 'assets', n_envs=1, backend='gpu', auto_reset=False)
        if not admit(env, output / 'admission'):
            result['status'] = 'initial_grasp_invalid'
            return result
        state, baseline = env.scene.scene.get_state(), sample(env)
        lower, upper, rate = arrays(env)
        idx = local_link_index(env.scene.robot, 'plate')
        def cpu(t):
            return t.detach().cpu().numpy()
        geometry = dict(diameter_mm=case['diameter_mm'], height_m=env.s['height_m'],
                        plate_position=cpu(env.scene.robot.get_links_pos())[0, idx].tolist(),
                        plate_quaternion_wxyz=cpu(env.scene.robot.get_links_quat())[0, idx].tolist(),
                        object_position=baseline['object_pos'][0].tolist(),
                        object_quaternion_wxyz=baseline['object_quat'][0].tolist())
        query = dict(start=case['goal_command'], home=plan['home'], lower=lower.tolist(),
                     upper=upper.tolist(), rate=rate.tolist(), seed=plan['seed'], geometry=geometry,
                     motion_budget=plan['motion_budget'])
        if args.native_profile in ('task-contact', 'task-contact-roll'):
            from workbench.research.experiments.hug_c.contacts import camera_side_axes
            query['contact_contract'] = dict(maximum_penetration_m=env.s['max_penetration_m'],
                moving_links=sorted(camera_side_axes()), protected_links=['housing', 'wedge', 'camera'],
                face_validation='Unchanged Genesis contact position, normal and force classification')
        save(output / 'query.json', query)
        command = [str(args.v2_python), '-m', 'workbench.research.experiments.hug_reverse_curobo.v2_worker',
                   '--model', str(args.model.resolve()), '--query', str((output / 'query.json').resolve()),
                   '--output', str((output / 'planner').resolve()), '--attempts', str(args.attempts)]
        command.extend(['--planner-mode', args.planner_mode])
        if args.release_prefix:
            command.append('--release-prefix')
        command.extend(['--native-profile', args.native_profile])
        if args.explain_failure:
            command.append('--explain-failure')
            command.append('--capture-rejected')
        planner_started = time.monotonic()
        # A V2 CUDA call cannot hang the entire batch indefinitely.
        with (output / 'planner.log').open('w') as log:
            process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT,
                                     timeout=args.planner_timeout)
        result['planner_process_s'] = time.monotonic() - planner_started
        result['planner_returncode'] = process.returncode
        completion = output / 'planner/completion.json'
        if not completion.exists():
            result['status'] = 'planner_execution_error'
            return result
        planner_result = json.loads(completion.read_text())
        result['planner_result'] = planner_result
        candidates = sorted((output / 'planner').glob('candidate-*/commands*.npz'))
        if not candidates:
            result['status'] = planner_result['status']
            return result
        result['physics_candidates'] = []
        for source in candidates:
            candidate_name = source.parent.name + source.stem.removeprefix('commands')
            directory = output / candidate_name
            directory.mkdir()
            shutil.copyfile(source, directory / 'commands.npz')
            with np.load(source, allow_pickle=False) as data:
                paths = data['commands'].copy()
            t = time.monotonic()
            reports = evaluate(env, paths, directory, state, baseline)
            candidate = dict(candidate=candidate_name, physics_s=time.monotonic() - t, report=reports[0])
            result['physics_candidates'].append(candidate)
            print(json.dumps(candidate), flush=True)
            save(output / 'progress.json', result)
            if not reports[0]['success']:
                continue
            replays = []
            for repeat in range(3):
                replay = directory / f'verification-{repeat+1}'
                replay.mkdir()
                if not admit(env, replay / 'admission'):
                    replays.append(False)
                    continue
                replay_state, replay_baseline = env.scene.scene.get_state(), sample(env)
                checked = evaluate(env, paths, replay, replay_state, replay_baseline)
                replays.append(bool(checked[0]['success']))
            candidate['replays'] = replays
            if all(replays):
                result.update(status='reverse_path_verified', winner=candidate_name, replays=replays)
                save(output / 'curriculum.json', dict(status='reverse_path_verified',
                    states=f'{candidate_name}/states.npz', lane=0, replays=replays,
                    forward_success=False, lift_success=False, reset_pool_validated=False,
                    reset='Replay admitted reverse prefix preserving full object state and velocities'))
                return result
        result['status'] = 'physics_budget_exhausted'
        return result
    except subprocess.TimeoutExpired:
        result['status'] = 'planner_timeout'
        return result
    except BaseException as exc:
        result.update(status='execution_error', error=repr(exc))
        raise
    finally:
        result['elapsed_s'] = time.monotonic() - started
        save(output / 'completion.json', result)


def batch(args):
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    rows = []
    for case in ('case_01', 'case_02', 'case_03'):
        t = time.monotonic()
        command = [sys.executable, '-m', 'workbench.research.experiments.hug_reverse_curobo.v2', 'case', '--plan',
                   str(args.ga_root / case / 'plan/plan.json'), '--output', str(args.output / case),
                   '--model', str(args.model), '--v2-python', str(args.v2_python),
                   '--gpu', args.gpu, '--attempts', str(args.attempts),
                   '--planner-timeout', str(args.planner_timeout)]
        command.extend(['--planner-mode', args.planner_mode])
        if args.release_prefix:
            command.append('--release-prefix')
        command.extend(['--native-profile', args.native_profile])
        if args.explain_failure:
            command.append('--explain-failure')
        # Spawn a fresh Genesis process for each case; never reuse global scene initialization.
        with (args.output / f'{case}.log').open('w') as log:
            p = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
        row = dict(case=case, returncode=p.returncode, elapsed_s=time.monotonic() - t)
        completed = args.output / case / 'completion.json'
        if completed.exists():
            row['result'] = json.loads(completed.read_text())
        rows.append(row)
        save(args.output / 'summary.json', dict(cases=rows, elapsed_s=time.monotonic() - started))
        print(json.dumps(row), flush=True)
    return 0 if all(row.get('result', {}).get('status') == 'reverse_path_verified' for row in rows) else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    for name in ('case', 'batch'):
        p = sub.add_parser(name)
        p.add_argument('--output', type=Path, required=True)
        p.add_argument('--model', type=Path, required=True)
        p.add_argument('--v2-python', type=Path, required=True)
        p.add_argument('--gpu', required=True)
        p.add_argument('--attempts', type=int, default=4)
        p.add_argument('--planner-timeout', type=int, default=900)
        p.add_argument('--planner-mode', choices=('full', 'spline', 'rrt'), default='spline')
        p.add_argument('--release-prefix', action='store_true')
        p.add_argument('--native-profile', choices=('default', 'task', 'task-contact', 'task-contact-roll'), default='default')
        p.add_argument('--explain-failure', action='store_true')
        if name == 'case':
            p.add_argument('--plan', type=Path, required=True)
        else:
            p.add_argument('--ga-root', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.attempts <= 16 or not 30 <= args.planner_timeout <= 3600:
        parser.error('Invalid attempt/time budget')
    if args.release_prefix and args.planner_mode != 'rrt':
        parser.error('--release-prefix requires --planner-mode rrt')
    if args.native_profile != 'default' and args.planner_mode != 'full':
        parser.error('Non-default native profiles require --planner-mode full')
    if args.command == 'batch':
        return batch(args)
    result = run_case(args.plan, args.output, args)
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result['status'] == 'reverse_path_verified' else 1


if __name__ == '__main__':
    raise SystemExit(main())
