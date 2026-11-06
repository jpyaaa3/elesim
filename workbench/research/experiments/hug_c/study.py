"""Sequential anchor RL with checkpoint videos and policy-free validation.

Run this module in a detached session. Every subprocess has its own log, and
status.json is atomically replaced at stage boundaries. No hardware is used.
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

from .configure import load_experiment, write_json, DEFAULT_NUM_ENVS
from elesim_sim.rl.trajectory import Trajectory


class Study:
    def __init__(self, args):
        self.args = args
        self.out = args.output.resolve()
        self.out.mkdir(parents=True, exist_ok=True)
        if (self.out/'status.json').exists() or (self.out/'experiment.json').exists():
            raise ValueError('Choose a new study output directory')
        self.exp_path = self.out/'experiment.json'
        self.status = dict(schema='elesim-c-hug-study', schema_version=1, pid=os.getpid(),
                           started_utc=self.now(), phase='starting', command_count=0,
                           num_envs=args.num_envs, iterations_per_anchor=args.iterations,
                           report_every=args.report_every, anchors={}, commands=[])
        self.publish()

    @staticmethod
    def now():
        return datetime.now(timezone.utc).isoformat()

    def publish(self, **updates):
        self.status.update(updates, updated_utc=self.now())
        temp = self.out/'status.json.tmp'
        temp.write_text(json.dumps(self.status, indent=2, allow_nan=False)+'\n')
        temp.replace(self.out/'status.json')

    def command(self, stage, *arguments, allow_failure=False):
        number = self.status['command_count']
        directory = self.out/'logs'; directory.mkdir(exist_ok=True)
        log = directory/f'{number:03d}-{stage}.log'
        argv = [sys.executable, '-u', '-m', __package__+'.experiment', *map(str, arguments)]
        record = dict(stage=stage, argv=argv, log=str(log), started_utc=self.now())
        self.status['commands'].append(record)
        self.publish(phase=stage, active_log=str(log), command_count=number+1)
        print(f'[{self.now()}] {stage}: {log}', flush=True)
        started = time.monotonic()
        with log.open('xb') as stream:
            result = subprocess.run(argv, stdout=stream, stderr=subprocess.STDOUT)
        record.update(returncode=result.returncode, elapsed_s=time.monotonic()-started,
                      finished_utc=self.now())
        self.publish()
        if result.returncode and (not allow_failure or result.returncode != 1):
            raise RuntimeError(f'{stage} exited {result.returncode}; see {log}')
        return result.returncode

    def evaluate(self, run, directory):
        directory.mkdir(parents=True, exist_ok=False)
        self.command('export', 'export', '--experiment', self.exp_path,
                     '--run', run, '--output', directory/'policy.npz')
        collected = self.command('collect-video', 'collect', '--experiment', self.exp_path,
                                '--policy', directory/'policy.npz', '--output', directory/'trajectory.npz',
                                '--video', directory/'policy.mp4', allow_failure=True)
        replayed = self.command('original-replay', 'replay', '--experiment', self.exp_path,
                               '--trajectory', directory/'trajectory.npz', '--output', directory/'replay.json',
                               '--verified-output', directory/'verified.npz', allow_failure=True)
        trajectory = Trajectory.load(directory/'verified.npz')
        result = dict(collection=trajectory.diagnostics['collection'],
                      replay=trajectory.diagnostics['replay_evaluation'],
                      success=collected == 0 and replayed == 0,
                      directory=str(directory), video=str(directory/'policy.mp4'))
        write_json(directory/'report.json', result)
        return result

    def run(self):
        args = self.args
        config=['configure','--pilot',args.pilot,'--output',self.exp_path]
        if args.settings:config+=['--settings',args.settings]
        self.command('configure',*config)
        exp = load_experiment(self.exp_path)
        self.command('dependencies', 'check', '--experiment', self.exp_path,
                     '--output', self.out/'dependencies.json')
        self.command('physics-check', 'check', '--physics', '--experiment', self.exp_path,
                     '--output', self.out/'physics-check.json')
        anchors = [case for case in exp['cases'] if case['split'] == 'train']
        # Each of exactly three anchors starts independently from the true Home.
        successful = self.out/'anchors'; successful.mkdir()
        policies = self.out/'policies'; policies.mkdir()
        for case in anchors:
            cid = case['case_id']; completed = 0; best = None; reports = []
            self.publish(case_id=cid, completed_iterations=0)
            while completed < args.iterations:
                amount = min(args.report_every, args.iterations-completed)
                command = ['train', '--experiment', self.exp_path, '--case', cid,
                           '--runs', self.out/'runs', '--num-envs', args.num_envs, '--iterations', amount,
                           '--preflight',self.out/'physics-check.json']
                if completed:
                    command.append('--resume')
                self.command('anchor-train', *command)
                completed += amount
                report = self.evaluate(self.out/'runs'/cid, self.out/'reports'/cid/f'updates-{completed:05d}')
                report['completed_iterations'] = completed
                reports.append(report)
                if report['success']:
                    best = report
                self.publish(completed_iterations=completed)
                self.status['anchors'][cid] = dict(reports=reports, best=best)
                self.publish()
                if best is not None:break
            if best is not None:
                shutil.copyfile(Path(best['directory'])/'verified.npz', successful/(cid+'.npz'))
                shutil.copyfile(Path(best['directory'])/'policy.npz', policies/(cid+'.npz'))
            else:
                self.publish(phase='finished-anchor-failures',failed_anchors=[cid],validation='not run: three successful replays required',finished_utc=self.now())
                return 1
        self.command('interpolate-validation', 'interpolate', '--experiment', self.exp_path,
                     '--anchors', successful, '--split', 'validation', '--output', self.out/'validation-trajectories')
        validation = self.command('validate', 'validate', '--experiment', self.exp_path,
                                  '--trajectories', self.out/'validation-trajectories', '--split', 'validation',
                                  '--output', self.out/'validation-report.json', allow_failure=True)
        if validation:
            self.publish(phase='finished-validation-failures', test='held out', finished_utc=self.now())
            return 1
        robustness=self.command('position-robustness','validate','--experiment',self.exp_path,
            '--trajectories',self.out/'validation-trajectories','--split','validation',
            '--offsets-mm',5,10,'--limit',5,'--output',self.out/'position-robustness.json',allow_failure=True)
        if robustness:
            self.publish(phase='finished-position-robustness-failures',test='held out',finished_utc=self.now())
            return 1
        # No parameter changes or further RL are made using validation/test.
        self.command('interpolate-test', 'interpolate', '--experiment', self.exp_path,
                     '--anchors', successful, '--split', 'test', '--output', self.out/'test-trajectories')
        tested = self.command('test', 'validate', '--experiment', self.exp_path,
                              '--trajectories', self.out/'test-trajectories', '--split', 'test',
                              '--output', self.out/'test-report.json', allow_failure=True)
        self.publish(phase='finished' if tested == 0 else 'finished-test-failures', finished_utc=self.now())
        return tested


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--pilot',type=Path,required=True)
    parser.add_argument('--settings',type=Path)
    parser.add_argument('--num-envs', type=int, default=DEFAULT_NUM_ENVS)
    parser.add_argument('--iterations', type=int, default=2000)
    parser.add_argument('--report-every', type=int, default=500)
    args = parser.parse_args(argv)
    if min(args.num_envs, args.iterations, args.report_every) < 1:
        parser.error('envs, iterations and report interval must be positive')
    study = Study(args)
    try:
        return study.run()
    except Exception as exc:
        study.publish(phase='error', error=repr(exc), finished_utc=study.now())
        raise


if __name__ == '__main__':
    raise SystemExit(main())
