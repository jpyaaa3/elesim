"""Producer evidence and fallback gates; fixtures are not physical evidence."""
import json
from pathlib import Path

import numpy as np
import pytest

from workbench.research.experiments.hug_pipeline import contracts, reverse, planning, reuse
from workbench.research.experiments.hug_reverse_curobo import teacher
from workbench.research.experiments.hug_reverse_ga.prepare import digest
from workbench.research.experiments.hug_c.configure import asset_digest
from workbench.tests.research.experiments.test_hug_reverse_rl import path_fixture


def evidence(tmp_path):
    path, task=path_fixture()
    task.update(schema='hug-reverse-ga-plan',version=1,source_sha256=digest(),
                simulation_context=dict(model_sha256=asset_digest()))
    root=tmp_path/'native';root.mkdir()
    provenance=dict(plan=task,planner='cuRobo V2 v0.9.0',planner_mode='full',
                    authored_waypoints_allowed=False,automatic_start_offset_allowed=False,release_prefix=False)
    contracts.save(root/'provenance.json',provenance)
    contracts.save(root/'query.json',{})
    winner='candidate-0001'
    native=dict(attempt=1,status='candidate',original_start_used=True,authored_waypoints=[],
                validation={k:[[True]] for k in ('path_valid','collision_valid','limits_and_dynamics_valid')})
    completion=dict(status='reverse_path_verified',replays=[True]*3,winner=winner,
        planner_result=dict(planner_mode='full',start_offset_used=False,authored_waypoints=False,candidates=[native]))
    contracts.save(root/'completion.json',completion)
    contracts.save(root/'curriculum.json',dict(status='reverse_path_verified',replays=[True]*3,
                                             states=winner+'/states.npz',lane=0))
    report=dict(candidate=0,success=True,rejected=False,reasons=[],duration_s=6.)
    metadata=dict(case=task['cases'][0],simulation_context=task['simulation_context'],direction='grasp-to-home')
    archive={k:v if k=='time_s' else v[:,None] for k,v in path.items()}
    archive.update(active=np.ones((len(path['time_s']),1),bool),metadata_json=np.asarray(json.dumps(metadata)))
    for folder,admission in ((winner,'admission'),*[(f'{winner}/verification-{i}',f'{winner}/verification-{i}/admission') for i in (1,2,3)]):
        (root/folder).mkdir(parents=True,exist_ok=True);(root/admission).mkdir(parents=True,exist_ok=True)
        contracts.save(root/folder/'report.json',dict(candidates=[report]))
        np.savez_compressed(root/folder/'states.npz',**archive)
        np.savez_compressed(root/admission/'states.npz',**{k:path[k][:1,None] for k in ('touch','floor','fault','wrong')})
    return root,task


def test_native_teacher_preserves_provenance_and_velocities_without_ga_generation(tmp_path):
    root,task=evidence(tmp_path)
    result=teacher.prepare(root,tmp_path/'ppo')
    assert result['reference_producer']=='curobo-v2' and result['reverse_plan']==task
    assert (tmp_path/'ppo/teacher-provenance.json').read_bytes()==(root/'provenance.json').read_bytes()
    assert result['physical_reset_pool_validated'] is False
    assert not list(root.glob('generation-*'))
    assert 'candidate-0001/verification-3/states.npz' in result['source_files']


@pytest.mark.parametrize('damage',['native_collision','prefix','replay_failed','unsafe_replay','winner_escape'])
def test_rejected_or_modified_native_evidence_never_enters_ppo(tmp_path,damage):
    root,_=evidence(tmp_path)
    completion=json.loads((root/'completion.json').read_text())
    if damage=='native_collision':completion['planner_result']['candidates'][0]['validation']['collision_valid']=[[False]]
    if damage=='prefix':completion['planner_result']['authored_waypoints']=True
    if damage=='replay_failed':completion['replays'][1]=False
    if damage=='winner_escape':completion['winner']='../candidate-0001'
    if damage=='unsafe_replay':
        f=root/'candidate-0001/verification-2/states.npz'
        with np.load(f,allow_pickle=False) as raw:data={k:raw[k].copy() for k in raw.files}
        data['wrong'][100,0]=True;np.savez_compressed(f,**data)
    contracts.save(root/'completion.json',completion)
    with pytest.raises(ValueError):teacher.prepare(root,tmp_path/'ppo')
    assert not (tmp_path/'ppo').exists()


def backend_fixture(tmp_path,monkeypatch,status,mode='auto'):
    out=tmp_path/'stage';out.mkdir();model=tmp_path/'model';model.mkdir()
    contracts.save(model/'metadata.json',{})
    config=dict(mode=mode,model=str(model),model_metadata_sha256=contracts.sha(model/'metadata.json'),
                source_sha256=reverse.curobo_sources(),python='/existing/v2/python',native_profile='task',attempts=2,timeout_s=300)
    study=dict(reverse_planner=config,backend='gpu',seed=42)
    calls=[]
    def plan(*args):
        directory=args[2];directory.mkdir();contracts.save(directory/'plan.json',{'task':True});return {'task':True}
    monkeypatch.setattr(planning,'reverse_plan',plan)
    monkeypatch.setattr(reuse,'reuse_reverse',lambda *args:None)
    monkeypatch.setenv('CUDA_VISIBLE_DEVICES','GPU-test')
    def process(command,log,timeout):
        assert command[command.index('--planner-mode')+1]=='full'
        assert command[command.index('--native-profile')+1]=='task'
        calls.append('curobo')
        directory=Path(command[command.index('--output')+1]);directory.mkdir()
        contracts.save(directory/'completion.json',dict(status=status))
        return (0 if status=='reverse_path_verified' else 1),False
    monkeypatch.setattr(reverse,'_process',process)
    from workbench.research.experiments.hug_reverse_ga import experiment
    def ga(*args):calls.append('ga');return dict(status='reverse_path_verified',replays=[True]*3)
    monkeypatch.setattr(experiment,'search',ga)
    return study,out,calls


def test_exhausted_curobo_uses_ga_and_retains_failed_proposal(tmp_path,monkeypatch):
    study,out,calls=backend_fixture(tmp_path,monkeypatch,'budget_exhausted')
    result=reverse.run(study,{},out,tmp_path)
    assert calls==['curobo','ga'] and result['accepted'] and result['reverse_producer']=='ga'
    assert (out/'plan/curobo/completion.json').exists()
    assert [c['producer'] for c in result['reverse_attempts']]==['curobo-v2','ga']


def test_curobo_only_failure_does_not_launch_ga(tmp_path,monkeypatch):
    study,out,calls=backend_fixture(tmp_path,monkeypatch,'budget_exhausted',mode='curobo')
    assert not reverse.run(study,{},out,tmp_path)['accepted']
    assert calls==['curobo']


def test_infrastructure_error_does_not_silently_fallback(tmp_path,monkeypatch):
    study,out,calls=backend_fixture(tmp_path,monkeypatch,'execution_error')
    with pytest.raises(RuntimeError,match='execution failed'):reverse.run(study,{},out,tmp_path)
    assert calls==['curobo']


def test_successful_curobo_is_selected_without_launching_ga(tmp_path,monkeypatch):
    study,out,calls=backend_fixture(tmp_path,monkeypatch,'reverse_path_verified')
    monkeypatch.setattr(teacher,'prepare',lambda *args,**kwargs:None)
    result=reverse.run(study,{},out,tmp_path)
    assert calls==['curobo'] and result['reverse_producer']=='curobo-v2'
    assert json.loads((out/'plan/teacher.json').read_text())['producer']=='curobo-v2'


def test_failed_teacher_validation_does_not_launch_ga(tmp_path,monkeypatch):
    study,out,calls=backend_fixture(tmp_path,monkeypatch,'reverse_path_verified')
    def reject(*args,**kwargs):raise ValueError('invalid evidence')
    monkeypatch.setattr(teacher,'prepare',reject)
    with pytest.raises(ValueError,match='invalid evidence'):reverse.run(study,{},out,tmp_path)
    assert calls==['curobo'] and not (out/'plan/teacher.json').exists()


def test_auto_uses_explicit_ga_evidence_only_after_curobo_failure(tmp_path,monkeypatch):
    study,out,calls=backend_fixture(tmp_path,monkeypatch,'budget_exhausted')
    original=b'{"original":"GA provenance"}'
    def cached(study,case,directory):
        assert directory==out/'fallback-reuse'
        (directory/'plan').mkdir(parents=True)
        (directory/'plan/plan.json').write_bytes(original)
        return dict(accepted=True,status='reverse_path_verified',replays=[True]*3,reused_from='explicit-ga-input')
    monkeypatch.setattr(reuse,'reuse_reverse',cached)
    result=reverse.run(study,{},out,tmp_path)
    assert calls==['curobo'] and result['reverse_producer']=='ga'
    assert result['reverse_attempts'][-1]['search_executed'] is False
    assert (out/'plan/fallback-ga/plan.json').read_bytes()==original
    assert json.loads((out/'plan/teacher.json').read_text())['producer']=='ga'


def test_teachers_stop_checks_all_three_anchors_before_learning(tmp_path,monkeypatch):
    from workbench.research.experiments.hug_pipeline import coordinator
    plan=dict(budgets=dict(max_reselections=1))
    monkeypatch.setattr(coordinator,'load_study',lambda root:plan)
    contracts.save(tmp_path/'plan.json',{})
    cases=[dict(case_id='anchor-'+str(i)) for i in range(3)]
    monkeypatch.setattr(planning,'choose',lambda *args:({},cases))
    runner=coordinator.Coordinator(tmp_path)
    visited=[]
    def stage(key,kind,**kwargs):
        visited.append(kind)
        return tmp_path/key
    runner.stage=stage
    runner.run('teachers')
    assert visited==['screen',*(['reverse','ppo-plan','check']*3)]
    assert not (tmp_path/'publication.json').exists()
