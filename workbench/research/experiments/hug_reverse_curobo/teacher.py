"""Read verified native V2 evidence as reverse-state PPO teaching material.

The original cuRobo provenance and candidate names stay intact. No GA generation
or search result is fabricated to satisfy the legacy GA directory layout.
"""
import json
from pathlib import Path
import re
import shutil

import numpy as np


def prepare(run, output, *, spacing_s=.25, seed=42):
    from workbench.research.experiments.hug_reverse_ga.prepare import digest as physics_digest, save
    from workbench.research.experiments.hug_c.configure import asset_digest, validate_settings, load_mechanism
    from workbench.research.experiments.hug_reverse_rl.data import FIELDS, digest, sha, require_verified, validate_path
    from workbench.research.experiments.hug_reverse_rl.schedule import DEFAULTS, validate_options

    run, output = Path(run), Path(output)
    if output.exists():
        raise FileExistsError(output)
    if not np.isfinite(spacing_s) or spacing_s <= 0 or type(seed) is not int or seed < 0:
        raise ValueError('Invalid spacing or seed')
    read = lambda name: json.loads((run/name).read_text())
    provenance = read('provenance.json')
    task = provenance['plan']
    if (task.get('schema') != 'hug-reverse-ga-plan' or task.get('version') != 1
            or task['source_sha256'] != physics_digest()
            or task['simulation_context']['model_sha256'] != asset_digest()):
        raise ValueError('Reverse physics source/model mismatch')
    validate_settings(task['settings'])
    if task['home'] != list(load_mechanism().presets['home']):
        raise ValueError('Home preset changed')
    if (provenance.get('planner') != 'cuRobo V2 v0.9.0'
            or provenance.get('planner_mode') != 'full'
            or provenance.get('authored_waypoints_allowed') is not False
            or provenance.get('automatic_start_offset_allowed') is not False
            or provenance.get('release_prefix') is not False):
        raise ValueError('Require native whole-path cuRobo evidence')
    completion, curriculum = read('completion.json'), read('curriculum.json')
    winner = completion.get('winner', '')
    if not isinstance(winner, str) or not re.fullmatch(r'candidate-\d{4}', winner):
        raise ValueError('Invalid cuRobo winner')
    planner = completion['planner_result']
    attempt = int(winner.removeprefix('candidate-'))
    native = next((c for c in planner['candidates'] if c['attempt'] == attempt), {})
    masks = native.get('validation', {})
    if (planner.get('planner_mode') != 'full' or planner.get('start_offset_used') is not False
            or planner.get('authored_waypoints') is not False or native.get('status') != 'candidate'
            or native.get('original_start_used') is not True or native.get('authored_waypoints') != []
            or set(masks) != {'path_valid', 'collision_valid', 'limits_and_dynamics_valid'}
            or any(np.asarray(value).dtype != np.bool_ or not np.asarray(value).size
                   or not np.asarray(value).all() for value in masks.values())):
        raise ValueError('Native path/collision/dynamics validation must pass')
    if curriculum.get('states') != winner+'/states.npz' or curriculum.get('lane') != 0:
        raise ValueError('Winner path mismatch')
    report_file = winner+'/report.json'
    reports = read(report_file)['candidates']
    if len(reports) != 1 or reports[0]['candidate'] != 0:
        raise ValueError('Expected one physical candidate lane')
    replay_files = [f'{winner}/verification-{i}/report.json' for i in (1, 2, 3)]
    replay_reports = [read(name)['candidates'] for name in replay_files]
    require_verified(completion, curriculum, reports[0], replay_reports)

    sources = ['provenance.json', 'query.json', 'completion.json', 'curriculum.json', report_file]

    def extract(folder, report, admission):
        states_file, admission_file = folder+'/states.npz', admission+'/states.npz'
        sources.extend((states_file, admission_file))
        with np.load(run/states_file, allow_pickle=False) as raw:
            meta = json.loads(str(raw['metadata_json'].item()))
            if (meta['case'] != task['cases'][0] or meta['simulation_context'] != task['simulation_context']
                    or meta.get('direction') != 'grasp-to-home'):
                raise ValueError('Reverse states provenance mismatch')
            active = raw['active'][:, 0].astype(bool)
            n = round(report['duration_s']/task['settings']['dt'])+1
            if n > len(active) or not active[:n].all() or active[n:].any():
                raise ValueError('Invalid winner validity mask')
            path = {k: raw[k][:n].copy() if k == 'time_s' else raw[k][:n, 0].copy() for k in FIELDS}
        with np.load(run/admission_file, allow_pickle=False) as initial:
            for k in ('touch', 'floor', 'fault', 'wrong'):
                path[k][0] = initial[k][-1, 0]
        validate_path(path, task)
        return path

    path = extract(winner, reports[0], 'admission')
    for i, rows in enumerate(replay_reports, 1):
        if len(rows) != 1 or rows[0]['candidate'] != 0:
            raise ValueError('Expected one replay lane')
        extract(f'{winner}/verification-{i}', rows[0], f'{winner}/verification-{i}/admission')
    sources.extend(replay_files)
    end = validate_path(path, task)
    stride = max(1, round(spacing_s/task['settings']['dt']))
    options = dict(DEFAULTS, seed=seed)
    validate_options(options)
    output.mkdir(parents=True)
    shutil.copyfile(run/'provenance.json', output/'teacher-provenance.json')
    np.savez_compressed(output/'reference-path.npz', **path)
    result = dict(schema='hug-reverse-ppo-plan', version=1, source_sha256=digest(), reverse_plan=task,
        settings=task['settings'], case=task['cases'][0], home=task['home'],
        pool_ticks=sorted(set([0, end, *range(0, end+1, stride)])), curriculum=options,
        reference_producer='curobo-v2', reference_source=str(run.resolve()),
        teacher_provenance_sha256=sha(output/'teacher-provenance.json'),
        restore='validated full Genesis snapshots from admitted reverse prefix replay',
        state_tolerances=dict(joint_q=1e-4, joint_qd=.003, object_pos=1e-4, object_quat=1e-4,
                              object_vel=.003, object_ang=.003),
        source_files={p: sha(run/p) for p in sources},
        simulation_context=dict(task['simulation_context'], code_sha256=digest(),
            control_mode='reverse-curriculum-four-dof-v1', reward_contract='bounded-milestones-lift90-v5',
            source_reverse_sha256=task['source_sha256'], reset_contract='full-state-prefix-bank-v1'),
        physical_reset_pool_validated=False, reference_path_sha256=sha(output/'reference-path.npz'))
    save(output/'plan.json', result)
    return result
