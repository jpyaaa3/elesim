"""One isolated physical stage per process; Genesis GPU state never leaks stages."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from .contracts import load_study,save,canonical


def run(job):
    root=Path(job['root']);study=load_study(root);out=Path(job['output']);kind=job['kind']
    if kind=='screen':
        from .screen import screen
        screen(root,study,out)
        return dict(accepted=True,criterion='Home checked and sustained Wrap admission screened')
    case=job['case'];out.mkdir(parents=True,exist_ok=False)
    if kind=='reverse':
        from .planning import reverse_plan
        from workbench.research.experiments.hug_reverse_ga.experiment import search
        from .reuse import reuse_reverse
        reused=reuse_reverse(study,case,out)
        if reused is not None:return reused
        plan_dir=out/'plan';p=reverse_plan(study,case,plan_dir,root/'mapper.html');run_dir=plan_dir/'run';run_dir.mkdir()
        result=search(p,run_dir,study['backend']);save(run_dir/'completion.json',result)
        return dict(result,accepted=result['status']=='reverse_path_verified')
    if kind=='forward':
        from .reference import search
        result=search(study,case,Path(job['reverse'])/'plan',out,study_root=root)
        return dict(result,accepted=result['status']=='forward_reference_verified')
    if kind=='ppo-plan':
        from .learning import prepare
        prepare(study,case,Path(job['reference']),out/'plan')
        return dict(accepted=True,initial_policy_required=False)
    if kind=='check':
        from .learning import load,create_env
        p,path=load(Path(job['plan'])/'plan')
        env=create_env(p,path,out,study['budgets']['num_envs'],study['backend'])
        return dict(accepted=True,**env.bank.report)
    if kind=='train':
        from .learning import train
        train(Path(job['plan'])/'plan',out/'training',num_envs=study['budgets']['num_envs'],
              iterations=job.get('iterations',study['budgets']['iterations']),backend=study['backend'],
              resume=Path(job['resume']) if job.get('resume') else None,study_root=root)
        return dict(accepted=True,home_verified=False,training_budget_finished=True)
    if kind in ('policy','progress','replay','aligned','interpolation'):
        from .evaluation import evaluate
        if kind in ('policy','progress'):
            return evaluate(study,case,out,policy_path=Path(job['policy']),repetitions=1 if kind=='progress' else 3)
        with np.load(job['trajectory'],allow_pickle=False) as data:commands=data['commands'].copy()
        return evaluate(study,case,out,commands=commands,repetitions=3,video=kind!='interpolation')
    raise ValueError('Unknown worker '+kind)


def main():
    p=argparse.ArgumentParser();p.add_argument('job',type=Path);a=p.parse_args();job=json.loads(a.job.read_text())
    try:
        result=run(job);save(Path(job['result']),result)
        return 0 if result.get('accepted') is True else 1
    except BaseException as exc:
        save(Path(job['result']),dict(accepted=False,status='execution_error',error=repr(exc)));raise

if __name__=='__main__':raise SystemExit(main())
