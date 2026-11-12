"""Curriculum evidence gates, portable path input and checkpoint provenance."""
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from workbench.research.experiments.hug_reverse_rl.schedule import Curriculum,DEFAULTS
from workbench.research.experiments.hug_reverse_rl.data import require_verified,validate_path,prepare,load_plan,sha
from workbench.research.experiments.hug_reverse_rl.bank import compare
from workbench.research.experiments.hug_reverse_rl.experiment import bind_checkpoint,resume
from workbench.research.experiments.hug_c.configure import DEFAULTS as SETTINGS,load_mechanism
from workbench.research.experiments.hug_reverse_ga.criteria import REASONS


def path_fixture():
    home=np.asarray(load_mechanism().presets['home']);goal=np.array([0.,-np.pi/2,-.31,-.56])
    command=np.vstack((np.linspace(goal,home,501),np.tile(home,(100,1))));n=len(command)
    path=dict(actual=command.copy(),actual_vel=np.zeros((n,4)),joint_q=np.zeros((n,12)),joint_qd=np.zeros((n,12)),
        object_pos=np.tile([.376,.113,.55],(n,1)),object_quat=np.tile([1.,0,0,0],(n,1)),
        object_vel=np.zeros((n,3)),object_ang=np.zeros((n,3)),touch=np.zeros((n,2)),floor=np.ones(n,bool),
        fault=np.zeros(n,bool),wrong=np.zeros(n,bool),contact_force=np.zeros(n),contact_penetration=np.zeros(n),
        violation_mask=np.zeros((n,len(REASONS)),bool),command=command,time_s=np.arange(n)*.01)
    path['touch'][0]=1
    plan=dict(settings=dict(SETTINGS),cases=[dict(goal_command=goal.tolist(),home_command=home.tolist(),object_xyz_m=[.376,.113,.55])],home=home.tolist())
    return path,plan


def verified():
    return (dict(status='reverse_path_verified',replays=[True]*3),
        dict(status='reverse_path_verified',replays=[True]*3),
        dict(success=True,rejected=False,reasons=[]),[[dict(success=True,rejected=False)] for _ in range(3)])


@pytest.mark.parametrize('change',['search_failed','replay_failed','not_three','bad_candidate','rejected_replay'])
def test_only_verified_reverse_paths_enter_prepare(change):
    completion,curr,report,replays=verified()
    if change=='search_failed':completion['status']='budget_exhausted'
    if change=='replay_failed':curr['replays'][1]=False
    if change=='not_three':replays.pop()
    if change=='bad_candidate':report['rejected']=True
    if change=='rejected_replay':replays[1][0]['success']=False
    with pytest.raises(ValueError):require_verified(completion,curr,report,replays)


def test_valid_path_and_exclude_redundant_home_dwell():
    path,plan=path_fixture();assert validate_path(path,plan)==501


def test_over_budget_completion_cannot_become_curriculum():
    completion,curr,report,replays=verified()
    report.update(completed=True,success=False,rejected=True,reasons=['bend_travel'],
                  evaluation_status='completed_over_budget')
    with pytest.raises(ValueError):require_verified(completion,curr,report,replays)


@pytest.mark.parametrize('change',['fault','wrong','xy','tilt','nan','slew','command','missing_joint','no_contact','not_home','moving_object','clock'])
def test_invalid_reset_path_rejected(change):
    path,plan=path_fixture()
    if change in ('fault','wrong'):path[change][100]=True
    if change=='xy':path['object_pos'][100,0]+=.051
    if change=='tilt':path['object_quat'][100]=[np.cos(.2),np.sin(.2),0,0]
    if change=='nan':path['joint_q'][100,1]=np.nan
    if change=='slew':path['command'][100,0]-=.01
    if change=='command':path['command'][100,0]=.1
    if change=='missing_joint':path['joint_q']=path['joint_q'][:,:4]
    if change=='no_contact':path['touch'][0]=0
    if change=='not_home':path['actual'][-1,0]-=.01
    if change=='moving_object':path['object_vel'][-1,0]=.02
    if change=='clock':path['time_s'][10]+=.001
    with pytest.raises((ValueError,RuntimeError)):validate_path(path,plan)


def test_frontier_starts_exactly_at_grasp_and_needs_full_success():
    c=Curriculum(4,dict(window=4,cooldown_steps=1))
    bins,epochs=c.sample(100);assert not bins.any()
    c.observe([0]*4,[0]*4,[False]*4);assert c.frontier==0
    c.observe([0]*4,[0]*4,[True]*4);assert c.frontier==1
    # Prior-grasp episodes and easy retention successes do not promote frontier1.
    c.observe([0,1],[0,0],[True,True]);assert c.frontier==1
    c.observe([0]*4,[1]*4,[True]*4);assert c.frontier==1
    c.observe([1]*4,[1]*4,[True]*4);assert c.frontier==2
    c.observe([2]*4,[2]*4,[False]*4);assert c.frontier==1


def test_cooldown_blocks_one_batch_from_racing_through_curriculum():
    c=Curriculum(4,dict(window=2,cooldown_steps=3))
    for _ in range(2):c.observe([0]*2,[0]*2,[True]*2)
    assert c.frontier==0
    c.observe([],[],[]);assert c.frontier==1
    c.observe([1]*2,[1]*2,[True]*2);assert c.frontier==1


def test_reset_mixture_keeps_old_states_and_original_home():
    c=Curriculum(4);c.frontier=2
    b,_=c.sample(1000);assert set(b)=={0,1,2} and -1 not in b
    c.frontier=3;b,_=c.sample(1000);assert set(b)=={-1,0,1,2,3}
    c=Curriculum(2,dict(window=2,cooldown_steps=1));c.frontier=1
    c.observe([-1,-1],[0,0],[True,True]);assert c.nominal_ready
    c.observe([-1,-1],[0,0],[False,False]);assert not c.nominal_ready


def test_checkpoint_restores_frontier_history_and_random_sampling():
    c=Curriculum(4,dict(window=2,cooldown_steps=1));c.observe([0,0],[0,0],[True,True]);c.sample(6)
    state=json.loads(json.dumps(c.state_dict()));d=Curriculum(4,c.options);d.load_state_dict(state)
    assert np.array_equal(c.sample(100)[0],d.sample(100)[0])
    bad=dict(state,frontier=7)
    with pytest.raises(ValueError):d.load_state_dict(bad)


def test_snapshot_comparison_includes_velocities_and_preserves_other_lanes():
    a=dict(object_pos=np.arange(12).reshape(4,3)/100,object_vel=np.zeros((4,3)))
    compare(a,a,dict(object_pos=1e-6,object_vel=1e-6),[0,2])
    compare(a,{k:v[[1,3]] for k,v in a.items()},dict(object_pos=1e-6,object_vel=1e-6),[1,3])
    wrong=dict(a,object_vel=np.ones((4,3)))
    with pytest.raises(RuntimeError):compare(wrong,a,dict(object_vel=1e-6))


def test_checkpoint_is_paired_with_ppo_and_rejects_modified_weights(tmp_path):
    learner=SimpleNamespace(save=lambda path:Path(path).write_bytes(b'weights'),load=lambda path:None,current_learning_iteration=9)
    env=SimpleNamespace(schedule=Curriculum(4),reset=lambda:None)
    plan=dict(source_sha256='source',reference_path_sha256='path')
    bind_checkpoint(learner,env,plan);checkpoint=tmp_path/'model.pt';learner.save(str(checkpoint))
    resume(learner,env,plan,checkpoint);assert learner.current_learning_iteration==10
    checkpoint.write_bytes(b'other')
    with pytest.raises(ValueError):resume(learner,env,plan,checkpoint)


def test_prepare_packs_portable_lane_without_gpu(tmp_path):
    from workbench.research.experiments.hug_reverse_ga.prepare import digest
    from workbench.research.experiments.hug_c.configure import asset_digest
    path,ga=path_fixture();source=tmp_path/'ga';source.mkdir();output=tmp_path/'ppo'
    ga.update(schema='hug-reverse-ga-plan',version=1,source_sha256=digest(),simulation_context=dict(model_sha256=asset_digest()))
    ga['cases'][0].update(case_id='test',diameter_mm=100)
    def save(name,data):
        p=source/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(data))
    save('plan.json',ga);completion,curr,report,replays=verified()
    report.update(candidate=0,duration_s=6.);curr.update(generation=0,candidate=0,lane=0,states='generation-0000/states.npz')
    save('run/completion.json',completion);save('run/curriculum.json',curr);save('run/generation-0000/report.json',dict(candidates=[report]))
    for i,rows in enumerate(replays,1):save(f'run/verification-{i}/report.json',dict(candidates=rows))
    np.savez_compressed(source/'run/generation-0000/states.npz',**{k:v if k=='time_s' else v[:,None] for k,v in path.items()},
        active=np.ones((len(path['time_s']),1),bool),metadata_json=np.asarray(json.dumps(dict(case=ga['cases'][0],simulation_context=ga['simulation_context'],direction='grasp-to-home'))))
    (source/'run/admission').mkdir();np.savez_compressed(source/'run/admission/states.npz',**{k:path[k][:1,None] for k in ('touch','floor','fault','wrong')})
    plan=prepare(source,output);packed,loaded=load_plan(output)
    assert plan==packed and plan['pool_ticks'][0]==0 and plan['pool_ticks'][-1]==501
    assert not plan['physical_reset_pool_validated'] and loaded['joint_q'].shape==(601,12)
    # A portable plan no longer needs access to the original GA source directory.
    assert sha(output/'reference-path.npz')==plan['reference_path_sha256']
