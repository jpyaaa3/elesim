"""Server-owned population reports that survive the operator/Codex session.

Read-only with respect to training: never changes policies, rewards or processes.
Only simulation collect/replay children and new report artifacts are created.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[3]
QUALITY = ('success', 'hold', 'lift', 'grasp', 'contact', 'approach', 'efficiency', 'safe')


def now():
    return datetime.now(timezone.utc).isoformat()


def save(path, value):
    temporary = path.with_suffix('.pending.json')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False))
    temporary.replace(path)


def training_alive(pid, experiment, proc=Path('/proc')):
    """Reject exited/zombie/reused PIDs rather than just kill(pid, 0)."""
    try:
        directory = proc/str(pid)
        fields = (directory/'stat').read_text().rsplit(')', 1)[1].split()
        command = (directory/'cmdline').read_bytes().split(b'\0')
        return (fields[0] != 'Z' and b'train' in command
                and b'workbench.research.experiments.hug_c.experiment' in command
                and os.fsencode(experiment) in command)
    except (OSError, IndexError):
        return False


def ready_generations(run):
    config = json.loads((run/'population.json').read_text())
    generations = []
    for path in sorted(run.glob('generation-*/summary.json')):
        summary = json.loads(path.read_text())
        updates = summary['training_env_steps']//(config['num_envs']*16)
        generations.append((updates, path.parent, summary))
    return generations


def milestone_policy(run, milestone):
    for updates, directory, summary in ready_generations(run):
        if updates >= milestone:
            winner = max(range(len(summary['reports'])),
                         key=lambda i: tuple(summary['reports'][i].get(k, 0.) for k in QUALITY))
            return directory/f'member-{winner:02d}'/'policy.npz', updates
    return None


def capture(root, output, policy, gpu, *, replay=False, execute=subprocess.run):
    """A failed grasp (exit 1) is a report; CLI faults/timeouts are errors."""
    output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(policy, output/'policy.npz')
    env = dict(os.environ, CUDA_VISIBLE_DEVICES=gpu, OMP_NUM_THREADS='2',
               PYOPENGL_PLATFORM='egl', PYTHONPATH='payload/runtime/common/protocol:payload/runtime/docker/sim/app:.')
    prefix = [sys.executable, '-u', '-m', 'workbench.research.experiments.hug_c.experiment']
    command = [sys.executable, '-u', '-m', 'workbench.research.analysis.record_hug_video',
                      '--experiment', str(root/'experiment.json'),
                      '--policy', str(output/'policy.npz'), '--output', str(output/'trajectory.npz'),
                      '--video', str(output/'policy.mp4')]
    record = dict(started_utc=now(), source_policy=str(policy), command=command)
    save(output/'report.json', record)
    with (output/'collect.log').open('xb') as log:
        result = execute(command, cwd=REPO, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=1200)
    record['collect_returncode'] = result.returncode
    record['artifacts_complete'] = all((output/name).is_file() and (output/name).stat().st_size > 0
                                      for name in ('policy.mp4', 'trajectory.npz'))
    if result.returncode not in (0, 1) or not record['artifacts_complete']:
        record.update(status='error', finished_utc=now())
        save(output/'report.json', record)
        return record
    from elesim_sim.rl.trajectory import Trajectory
    diagnostics = Trajectory.load(output/'trajectory.npz').diagnostics
    record['collection'] = diagnostics['collection']
    record['report_camera'] = diagnostics.get('report_camera')
    if replay:
        command = prefix+['replay', '--experiment', str(root/'experiment.json'),
                          '--trajectory', str(output/'trajectory.npz'), '--output', str(output/'replay.json'),
                          '--verified-output', str(output/'verified.npz')]
        with (output/'replay.log').open('xb') as log:
            result = execute(command, cwd=REPO, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=1200)
        record['replay_returncode'] = result.returncode
        record['replay_report_created'] = (output/'replay.json').is_file()
        if result.returncode not in (0, 1) or not record['replay_report_created']:
            record.update(status='error', finished_utc=now())
            save(output/'report.json', record)
            return record
    record.update(status='complete', finished_utc=now())
    save(output/'report.json', record)
    return record


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--case', required=True)
    parser.add_argument('--milestones', type=int, nargs='+', default=[500,1000,1500,2000])
    parser.add_argument('--poll-seconds', type=float, default=60.)
    parser.add_argument('--max-hours', type=float, default=8.)
    parser.add_argument('--once', action='store_true', help='Read status and report only milestones already available')
    args = parser.parse_args(argv)
    if args.poll_seconds <= 0 or args.max_hours <= 0 or any(n <= 0 for n in args.milestones):
        parser.error('Intervals and milestones must be positive')
    root = args.root.resolve()
    launch = json.loads((root/'launch.json').read_text())
    if Path(args.case).name != args.case or args.case in ('.', '..'):
        parser.error('Case must be a single path component')
    run = root/'runs'/args.case
    output = root/'server-reports'
    output.mkdir(exist_ok=True)
    import fcntl
    with (output/'watch.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        status_path = output/'watch-status.json'
        status = json.loads(status_path.read_text()) if status_path.exists() else dict(tasks={})
        status.update(pid=os.getpid(), started_utc=now(), train_pid=launch['train_pid'])
        deadline = time.monotonic()+args.max_hours*3600
        while True:
            status.update(updated_utc=now(), training_alive=training_alive(launch['train_pid'], root/'experiment.json'))
            progress = run/'population-progress.json'
            status['training_progress'] = json.loads(progress.read_text()) if progress.exists() else None
            save(status_path, status)
            tasks = []
            for milestone in sorted(set(args.milestones)):
                candidate = milestone_policy(run, milestone)
                if candidate:
                    tasks.append((f'updates-{milestone:05d}', candidate[0], False))
            if not status['training_alive']:
                state_path = run/'population-state.json'
                state = json.loads(state_path.read_text()) if state_path.exists() else {}
                checkpoint = state.get('champion_checkpoint')
                if checkpoint:
                    tasks.append(('final', (run/checkpoint).with_name('policy.npz'), True))
            for name, policy, final_replay in tasks:
                if status['tasks'].get(name, {}).get('status') in ('complete', 'error'):
                    continue
                # Restart never overwrites an interrupted report attempt.
                attempt = output/name
                suffix = 1
                while attempt.exists():
                    attempt = output/(name+f'-attempt-{suffix:02d}'); suffix += 1
                status['tasks'][name] = dict(status='running', directory=str(attempt), started_utc=now())
                save(status_path, status)
                try:
                    result = capture(root, attempt, policy, launch['gpu_uuid'], replay=final_replay)
                    status['tasks'][name].update(result)
                except Exception as error:
                    status['tasks'][name].update(status='error', error=repr(error), finished_utc=now())
                save(status_path, status)
            if args.once or not status['training_alive'] or time.monotonic() >= deadline:
                status.update(phase='finished' if not status['training_alive'] else 'idle' if args.once else 'deadline',
                              finished_utc=now())
                save(status_path, status)
                return 0
            time.sleep(args.poll_seconds)


if __name__ == '__main__':
    raise SystemExit(main())
