"""Cold-start PPO plans and paired checkpoints; no scripted learned episodes."""
from pathlib import Path
import json
import time
import subprocess
import sys
import re
import numpy as np
from .contracts import save,sha,digest,canonical
from workbench.research.experiments.hug_reverse_rl.data import validate_path


def prepare(study,case,reference,output):
    from workbench.research.experiments.hug_home_rl.schedule import DEFAULTS as SCHEDULE
    from workbench.research.experiments.hug_home_rl.smoothness import DEFAULTS as SMOOTH
    from .reverse import prepare_reference as prepare_reverse
    # Both producers retain independent physical replay/provenance gates.
    base=prepare_reverse(reference,output,seed=study['seed'])
    expected=dict(case,start_command=case['goal_command'])
    if base['case']!=expected or base['settings']!=study['settings'] or base['home']!=study['home']:
        raise ValueError('Reverse evidence does not match selected case/physics/Home')
    with np.load(output/'reference-path.npz',allow_pickle=False) as raw:
        path={k:raw[k].copy() for k in raw.files}
    end=validate_path(path,base['reverse_plan'])
    plan=dict(base,schema='hug-pipeline-ppo',version=3,source_sha256=digest(),case=case,
        simulation_context=dict(study['shared_context'],case_sha256=canonical(case)),
        shared_context=study['shared_context'],curriculum=dict(SCHEDULE,seed=study['seed']),smoothness=dict(SMOOTH),
        initial_checkpoint_sha256=None,initialization='random actor/critic; no previous policy required',
        reference_direction='grasp-to-home',scripted_episode_actions=False)
    plan['pool_ticks']=sorted(set(np.rint(np.linspace(0,end,min(32,end+1))).astype(int).tolist()))
    from .linear import validate_resets
    validate_resets(path['command'][plan['pool_ticks'],0])
    save(output/'plan.json',plan)
    return plan


def load(directory):
    from workbench.research.experiments.hug_c.configure import validate_settings,load_mechanism
    from workbench.research.experiments.hug_home_rl.schedule import validate as schedule_validate
    from workbench.research.experiments.hug_home_rl.smoothness import validate as smooth_validate
    from .contracts import shared
    directory=Path(directory);p=json.loads((directory/'plan.json').read_text())
    if (p.get('schema')!='hug-pipeline-ppo' or p.get('version')!=3 or p['source_sha256']!=digest()
        or p['shared_context']!=shared(p['settings']) or p['simulation_context']!=dict(p['shared_context'],case_sha256=canonical(p['case']))
        or p['reference_path_sha256']!=sha(directory/'reference-path.npz')
        or p['home']!=list(load_mechanism().presets['home'])):raise ValueError('PPO plan/model/source mismatch')
    validate_settings(p['settings']);schedule_validate(p['curriculum']);smooth_validate(p['smoothness'])
    if (p.get('reference_producer')=='curobo-v2'
        and sha(directory/'teacher-provenance.json')!=p['teacher_provenance_sha256']):
        raise ValueError('Teacher provenance changed')
    with np.load(directory/'reference-path.npz',allow_pickle=False) as d:path={k:d[k].copy() for k in d.files}
    ga=p['reverse_plan']
    if (p.get('reference_direction')!='grasp-to-home' or p.get('scripted_episode_actions') is not False
        or ga['settings']!=p['settings'] or ga['home']!=p['home']
        or ga['cases']!=[dict(p['case'],start_command=p['case']['goal_command'])]):
        raise ValueError('Reverse evidence/case mismatch')
    end=validate_path(path,ga);ticks=p['pool_ticks']
    if (not 2<=len(ticks)<=32 or any(type(i) is not int for i in ticks) or ticks!=sorted(set(ticks))
        or ticks[0]!=0 or ticks[-1]!=end):raise ValueError('Invalid reverse bank ordering')
    from .linear import validate_resets
    validate_resets(path['command'][ticks,0])
    return p,path


def create_env(plan,path,output,num_envs,backend):
    # The bank replays contact-rich references and preserves all velocities.
    # Reverse snapshots seed resets; learned episodes use only PPO actions.
    from .environment import ReverseHomeEnv
    from workbench.research.experiments.hug_c.experiment import check_versions
    check_versions()
    return ReverseHomeEnv(plan,path,output,num_envs=num_envs,backend=backend)


def train(plan_dir,output,*,num_envs,iterations,backend,resume=None,study_root=None):
    from workbench.research.experiments.hug_c.experiment import runner
    from .policy import export_policy
    from workbench.research.experiments.wrap_grasp.train import set_seed
    from workbench.research.experiments.hug_home_rl.experiment import bind_checkpoint,resume as restore
    plan,path=load(plan_dir);output.mkdir(parents=True,exist_ok=False);started=time.monotonic()
    set_seed(plan['curriculum']['seed']);env=create_env(plan,path,output,num_envs,backend)
    learner=runner(env,output);bind_checkpoint(learner,env,plan)
    if resume:restore(learner,env,plan,resume)
    if study_root is not None:
        paired_save=learner.save
        def report_checkpoint(file,*args,**kwargs):
            result=paired_save(file,*args,**kwargs)
            match=re.fullmatch(r'model_(\d+)\.pt',Path(file).name)
            if match and int(match[1])>0 and int(match[1])%500==0:
                directory=output/('progress-'+match[1]);directory.mkdir()
                policy=directory/'policy.npz'
                export_policy(learner.alg.get_policy(),policy,dict(env.metadata(),shared_context=plan['shared_context'],case_sha256=canonical(plan['case'])))
                job=dict(root=str(study_root),kind='progress',case=plan['case'],output=str(directory/'evaluation'),policy=str(policy),result=str(directory/'result.json'))
                save(directory/'job.json',job)
                with (directory/'evaluation.log').open('w') as log:
                    completed=subprocess.run([sys.executable,'-u','-m','workbench.research.experiments.hug_pipeline.worker',str(directory/'job.json')],stdout=log,stderr=subprocess.STDOUT)
                save(directory/'recording.json',dict(returncode=completed.returncode,checkpoint_sha256=sha(file),training_updates=0))
            return result
        learner.save=report_checkpoint
    save(output/'run.json',dict(initialization='resume' if resume else 'cold',num_envs=num_envs,iterations=iterations,
        resume=str(resume) if resume else None,scripted_episode_actions=False))
    learner.save(str(output/'model_initial.pt'))
    if iterations>0:learner.learn(num_learning_iterations=iterations,init_at_random_ep_len=False)
    learner.save(str(output/'model_final.pt'))
    export_policy(learner.alg.get_policy(),output/'policy.npz',dict(env.metadata(),
        shared_context=plan['shared_context'],case_sha256=canonical(plan['case']),checkpoint_sha256=sha(output/'model_final.pt')))
    save(output/'completion.json',dict(status='training_budget_finished',elapsed_s=time.monotonic()-started,
        curriculum=env.schedule.state_dict(),home_verified=False))
