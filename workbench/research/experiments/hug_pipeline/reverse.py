"""Bounded reverse teachers with native cuRobo proposals and optional GA fallback."""
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time

from .contracts import canonical, files_hashes, save, sha, verify_files


def curobo_sources():
    folder=Path(__file__).parent.parent/'hug_reverse_curobo'
    return {name:sha(folder/name) for name in
            ('v2.py','v2_worker.py','v2_geometry.py','v2_model.py','native_failure.py','teacher.py')}


def configuration(args):
    mode = getattr(args, 'reverse_planner', 'ga')
    if mode == 'ga':
        return dict(mode='ga')
    if args.backend != 'gpu':
        raise ValueError('cuRobo requires the GPU physics backend')
    model, python = getattr(args, 'curobo_model', None), getattr(args, 'curobo_python', None)
    if model is None or python is None:
        raise ValueError('cuRobo requires --curobo-model and --curobo-python')
    model, python = Path(model).resolve(), Path(os.path.abspath(python))
    if not python.is_file():
        raise ValueError('cuRobo Python is missing')
    attempts, timeout = args.curobo_attempts, args.curobo_timeout
    if not 1 <= attempts <= 16 or not 30 <= timeout <= 3600:
        raise ValueError('Invalid cuRobo attempt/time budget')
    return dict(mode=mode, model=str(model), python=str(python),
                model_metadata_sha256=sha(model/'metadata.json'),
                source_sha256=curobo_sources(),
                native_profile=args.curobo_profile, attempts=attempts, timeout_s=timeout)


def _process(command, log, timeout):
    """A timeout must release this attempt's entire GPU process group."""
    with Path(log).open('w') as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            return process.wait(timeout=timeout), False
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
            # The direct parent can exit before its CUDA child. Kill the owned
            # group even when wait() already returned; never target other jobs.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()
            return process.returncode, True


def prepare_reference(reference, output, **kwargs):
    """Version-one teacher selection; legacy GA roots retain their exact reader."""
    reference = Path(reference)
    marker = reference/'teacher.json'
    if not marker.exists():
        from workbench.research.experiments.hug_reverse_rl.data import prepare
        return prepare(reference, output, **kwargs)
    record = json.loads(marker.read_text())
    if (record.get('schema') != 'hug-reverse-teacher' or record.get('version') != 1
            or (record.get('producer'),record.get('run')) not in
               (('curobo-v2','curobo'),('ga','fallback-ga'))):
        raise ValueError('Unknown reverse teacher contract')
    source = reference/record['run']
    verify_files(source, record['artifacts'])
    task = json.loads((reference/'plan.json').read_text())
    if record.get('task_sha256') != canonical(task):
        raise ValueError('Reverse teacher task changed')
    if record['producer']=='ga':
        from .reuse import compatible
        from workbench.research.experiments.hug_reverse_rl.data import prepare
        old=json.loads((source/'plan.json').read_text())
        study=dict(settings=task['settings'],home=task['home'],shared_context=task['simulation_context'])
        if not compatible(study,task['cases'][0],old):
            raise ValueError('GA fallback physical case changed')
        return prepare(source,output,**kwargs)
    original = json.loads((source/'provenance.json').read_text())['plan']
    if task != original:
        raise ValueError('Reverse teacher task changed')
    from workbench.research.experiments.hug_reverse_curobo.teacher import prepare
    return prepare(source, output, **kwargs)


def run(study, case, output, root):
    from .planning import reverse_plan
    from .reuse import reuse_reverse
    config = study.get('reverse_planner', dict(mode='ga'))
    if config['mode']=='ga':
        reused = reuse_reverse(study, case, output)
        if reused is not None:
            return dict(reused,reverse_producer='ga')
    output = Path(output)
    directory = output/'plan'
    task = reverse_plan(study, case, directory, Path(root)/'mapper.html')
    attempts = []
    started = time.monotonic()

    def finish(result, producer):
        selected = dict(schema='hug-reverse-selection', version=1, mode=config['mode'],
                        producer=producer, attempts=attempts, elapsed_s=time.monotonic()-started,
                        status=result['status'], accepted=result['status'] == 'reverse_path_verified')
        save(output/'reverse-selection.json', selected)
        return dict(result, accepted=selected['accepted'], reverse_producer=producer,
                    reverse_attempts=attempts, reverse_elapsed_s=selected['elapsed_s'])

    if config['mode'] in ('curobo', 'auto'):
        if config['source_sha256']!=curobo_sources():
            raise ValueError('Pinned cuRobo source changed; prepare a new study')
        if sha(Path(config['model'])/'metadata.json') != config['model_metadata_sha256']:
            raise ValueError('Pinned cuRobo model changed')
        gpu = os.environ.get('CUDA_VISIBLE_DEVICES', '')
        if not gpu.startswith('GPU-') or ',' in gpu:
            raise ValueError('cuRobo requires exactly one GPU UUID in CUDA_VISIBLE_DEVICES')
        command = [sys.executable, '-m', 'workbench.research.experiments.hug_reverse_curobo.v2',
            'case', '--plan', str(directory/'plan.json'), '--output', str(directory/'curobo'),
            '--model', config['model'], '--v2-python', config['python'], '--gpu', gpu,
            '--planner-mode', 'full', '--native-profile', config['native_profile'],
            '--attempts', str(config['attempts']),
            '--planner-timeout', str(min(300, config['timeout_s']))]
        t = time.monotonic()
        code, timed_out = _process(command, output/'curobo.log', config['timeout_s'])
        file = directory/'curobo/completion.json'
        result = (dict(status='wall_timeout') if timed_out else
                  json.loads(file.read_text()) if file.exists() else dict(status='execution_error'))
        attempts.append(dict(producer='curobo-v2', native_profile=config['native_profile'],
                             elapsed_s=time.monotonic()-t, status=result['status'], returncode=code))
        save(output/'reverse-progress.json', dict(attempts=attempts))
        if result['status'] == 'reverse_path_verified':
            if code != 0:
                raise RuntimeError('Successful cuRobo evidence has a failed process')
            # Fully validate the retained physical winner and every fresh replay
            # before selecting it. PPO will separately rebuild its reset bank.
            from workbench.research.experiments.hug_reverse_curobo.teacher import prepare
            prepare(directory/'curobo', output/'teacher-validation', seed=study['seed'])
            save(directory/'teacher.json', dict(schema='hug-reverse-teacher', version=1,
                 producer='curobo-v2', run='curobo', task_sha256=canonical(task),
                 artifacts=files_hashes(directory/'curobo')))
            return finish(result, 'curobo-v2')
        if result['status'] not in ('budget_exhausted', 'physics_budget_exhausted',
                                    'planner_timeout', 'wall_timeout', 'initial_grasp_invalid'):
            raise RuntimeError('cuRobo execution failed: '+str(result))
        if config['mode'] == 'curobo':
            return finish(result, None)
        # Explicit existing reverse inputs become fallback material in auto
        # mode. Keep both the failed proposal and the original GA bytes.
        t=time.monotonic()
        cache=output/'fallback-reuse'
        reused=reuse_reverse(study,case,cache)
        if reused is not None:
            source=directory/'fallback-ga'
            shutil.move(str(cache/'plan'),source)
            save(directory/'teacher.json',dict(schema='hug-reverse-teacher',version=1,
                producer='ga',run='fallback-ga',task_sha256=canonical(task),artifacts=files_hashes(source)))
            attempts.append(dict(producer='ga',elapsed_s=time.monotonic()-t,status=reused['status'],
                                 reused_from=reused['reused_from'],search_executed=False))
            return finish(reused,'ga')

    from workbench.research.experiments.hug_reverse_ga.experiment import search
    run_dir = directory/'run'
    run_dir.mkdir()
    t = time.monotonic()
    result = search(task, run_dir, study['backend'])
    save(run_dir/'completion.json', result)
    attempts.append(dict(producer='ga', elapsed_s=time.monotonic()-t, status=result['status']))
    return finish(result, 'ga')
