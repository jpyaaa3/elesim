"""CPU contracts for bounded lift cost, overlapping starts and forward data."""
import json
import numpy as np
import pytest
from workbench.research.experiments.hug_home_rl.smoothness import Smoothness,validate
from workbench.research.experiments.hug_home_rl.schedule import WindowCurriculum
from workbench.research.experiments.hug_home_rl.data import validate_prefix
from workbench.research.experiments.hug_c.configure import DEFAULTS,load_mechanism


def cost(n=1):
    c=Smoothness(n,np.ones(4),.1);c.reset(np.arange(n),np.zeros((n,4)));return c


def test_no_cost_before_lift_and_stationary_hold_is_free():
    c=cost();q=np.zeros((1,4))
    for i in range(20):
        q[:]=i%2
        assert c.step(q,np.array([False]),np.array([True]))[0]==0
    c.step(q,np.array([True]),np.array([True]))
    for _ in range(100):assert c.step(q,np.array([False]),np.array([True]))[0]==0


def test_oscillation_cost_is_bounded_and_contact_loss_cannot_disable_it():
    c=cost();c.step(np.zeros((1,4)),np.array([True]),np.array([True]))
    total=0.
    for i in range(1000):
        penalty=c.step(np.full((1,4),i%2),np.array([False]),np.array([True]))
        total-=penalty[0]
    assert 0<total<=1.00000001 and c.armed[0]
    assert c.spent[0]==pytest.approx(1)
    assert c.step(np.zeros((1,4)),np.array([False]),np.array([True]))[0]==0


def test_small_corrections_are_free_and_dead_episodes_are_not_charged():
    c=cost();c.step(np.zeros((1,4)),np.array([True]),np.array([True]))
    assert c.step(np.full((1,4),.05),np.array([False]),np.array([True]))[0]==0
    assert c.step(np.ones((1,4)),np.array([False]),np.array([False]))[0]==0


def test_cost_uses_applied_motion_not_raw_saturated_actions_and_reset_is_selective():
    c=cost(2);c.step(np.zeros((2,4)),np.ones(2,bool),np.ones(2,bool))
    c.step(np.ones((2,4)),np.zeros(2,bool),np.ones(2,bool));before=c.spent.copy()
    c.reset([0],np.zeros((1,4)))
    assert c.spent[0]==0 and not c.armed[0]
    assert c.spent[1]==before[1] and c.armed[1]
    # An unchanged clipped waypoint cannot accrue reversal cost indefinitely.
    for _ in range(100):c.step(np.ones((2,4)),np.zeros(2,bool),np.ones(2,bool))
    assert c.spent[1]-before[1]<.02


@pytest.mark.parametrize('options',[{'budget':-1},{'budget':1.1},{'budget':float('nan')},{'deadband':2},{'reference_duration_s':0}])
def test_invalid_smoothness_rejected(options):
    with pytest.raises(ValueError):Smoothness(1,np.ones(4),.1,options)


def test_overlapping_window_retains_hard_and_solved_starts_but_only_hard_promotes():
    c=WindowCurriculum(10,dict(window=4,cooldown_steps=1));c.frontier=4
    b,_=c.sample(10000)
    assert set(b)==set(range(5))
    assert .35<np.mean(b==4)<.45
    assert np.mean((b==2)|(b==3))>.4
    c.observe([2,3,2,3],[0]*4,[True]*4);assert c.frontier==4
    c.observe([4]*4,[0]*4,[True]*4);assert c.frontier==5
    c.observe([5]*4,[0]*4,[True]*4);assert c.frontier==5


def test_window_checkpoint_restores_rng_and_final_home_is_explicit():
    c=WindowCurriculum(4);c.frontier=3;c.sample(12)
    state=json.loads(json.dumps(c.state_dict()));d=WindowCurriculum(4);d.load_state_dict(state)
    a,_=c.sample(1000);b,_=d.sample(1000)
    assert np.array_equal(a,b) and -.1<np.mean(a==-1)-.5<.1
    with pytest.raises(ValueError):WindowCurriculum(4,dict(overlap_bins=2)).load_state_dict(state)


def fixture():
    home=np.asarray(load_mechanism().presets['home']);n=111
    cmd=np.tile(home,(n,1));cmd[:,2]=np.linspace(home[2],home[2]+.2,n)
    pos=np.tile([.376,.113,.55],(n,1))
    path=dict(actual=cmd.copy(),actual_vel=np.zeros((n,4)),joint_q=np.zeros((n,24)),joint_qd=np.zeros((n,24)),
        object_pos=pos,object_quat=np.tile([1.,0,0,0],(n,1)),object_vel=np.zeros((n,3)),object_ang=np.zeros((n,3)),
        touch=np.zeros((n,2)),floor=np.ones(n,bool),fault=np.zeros(n,bool),wrong=np.zeros(n,bool),
        contact_force=np.zeros(n),contact_penetration=np.zeros(n),command=cmd,time_s=np.arange(n)*.01)
    base=dict(home=home.tolist(),settings=dict(DEFAULTS),case=dict(object_xyz_m=pos[0].tolist()),simulation_context={'test':True})
    meta=dict(phase='nominal-Home-to-handoff',training_updates=0,task_history_reset_at_handoff=True,simulation_context=base['simulation_context'])
    return path,meta,base


def test_forward_prefix_starts_at_home_and_has_no_grasp_admission_requirement():
    path,meta,base=fixture();assert validate_prefix(path,meta,base)==110


@pytest.mark.parametrize('change',['clock','home','object','velocity','fault','wrong','slew','drift','missing','nan','old_direction','history'])
def test_bad_forward_prefix_is_rejected(change):
    path,meta,base=fixture()
    if change=='clock':path['time_s'][2]+=.001
    if change=='home':path['command'][0,0]=0
    if change=='object':path['object_pos'][0,0]+=.01
    if change=='velocity':path['object_vel'][0,2]=.2
    if change in ('fault','wrong'):path[change][3]=True
    if change=='slew':path['command'][2,0]+=.01
    if change=='drift':path['object_pos'][100,0]+=.003
    if change=='missing':path.pop('joint_qd')
    if change=='nan':path['actual_vel'][2,0]=np.nan
    if change=='old_direction':meta['phase']='grasp-to-home'
    if change=='history':meta['task_history_reset_at_handoff']=False
    with pytest.raises((ValueError,RuntimeError)):validate_prefix(path,meta,base)


def prepare_fixture(tmp_path,monkeypatch):
    from workbench.research.experiments.hug_home_rl.data import prepare,sha
    from workbench.research.experiments.hug_c.configure import asset_digest,OBSERVATIONS,ACTIONS,POLICY_VERSION
    from workbench.research.experiments.hug_reverse_rl.data import digest
    path,meta,base=fixture()
    base.update(source_sha256=digest(),reference_path_sha256='old-reference',state_tolerances=dict(joint_q=1e-4,joint_qd=.003,object_pos=1e-4,object_quat=1e-4,object_vel=.003,object_ang=.003))
    base['simulation_context']['model_sha256']=asset_digest()
    base['simulation_context']['code_sha256']=base['source_sha256']
    base['case'].update(goal_command=[0.,-np.pi/2,-.31,-.56],diameter_mm=100)
    source=tmp_path/'source';bridge=tmp_path/'bridge';out=tmp_path/'out'
    (source/'training').mkdir(parents=True);(source/'reports/model_final').mkdir(parents=True)
    model=source/'training/model_final.pt';model.write_bytes(b'CPU preparation never deserializes checkpoint data')
    side=dict(schema='hug-reverse-ppo-checkpoint',version=1,source_sha256=base['source_sha256'],reference_path_sha256=base['reference_path_sha256'],checkpoint_sha256=sha(model))
    (source/'training/model_final.pt.curriculum.json').write_text(json.dumps(side))
    policy=source/'reports/model_final/policy.npz'
    metadata=dict(base['case'],simulation_context=base['simulation_context'],checkpoint_sha256=sha(model),reverse_reference_sha256=base['reference_path_sha256'])
    np.savez_compressed(policy,schema=np.asarray('elesim-c-hug-policy'),schema_version=np.asarray(POLICY_VERSION),observation_names=np.asarray(OBSERVATIONS),action_names=np.asarray(ACTIONS),metadata_json=np.asarray(json.dumps(metadata)),norm_mean=np.zeros(20),norm_std=np.ones(20),norm_eps=np.asarray(1e-8),n_layers=np.asarray(1),w0=np.zeros((4,20)),b0=np.zeros(4))
    records=[]
    for i in (1,2,3):
        folder=bridge/f'replay-{i}';folder.mkdir(parents=True)
        report=dict(policy_sha256=sha(policy),fresh_handoff=True,physical_preservation_errors={k:0 for k in base['state_tolerances']},results=[dict(start_tick=-1,success=True,fault=False,wrong_face=False)])
        (folder/'model_final.json').write_text(json.dumps(report))
        np.savez_compressed(folder/'model_final-prefix.npz',**{k:v if k=='time_s' else v[:,None] for k,v in path.items()},metadata_json=np.asarray(json.dumps(meta)))
        records.append(dict(returncode=0,result=report))
    (bridge/'completion.json').write_text(json.dumps(dict(status='verified',results=records)))
    monkeypatch.setattr('workbench.research.experiments.hug_reverse_rl.data.load_plan',lambda p:(base,None))
    plan=prepare(source,bridge,out)
    return out,plan,source,bridge


def test_prepared_plan_is_portable_and_orders_endpoint_to_original_home(tmp_path,monkeypatch):
    import shutil
    from workbench.research.experiments.hug_home_rl.data import load_plan
    out,plan,source,bridge=prepare_fixture(tmp_path,monkeypatch)
    shutil.rmtree(source);shutil.rmtree(bridge)
    loaded,path=load_plan(out)
    assert loaded==plan and plan['pool_ticks']==list(range(110,-1,-5))
    assert path['joint_q'].shape==(111,24) and plan['smoothness']['budget']==1


@pytest.mark.parametrize('change',['model','policy','ordering','fault_resigned','budget'])
def test_plan_tampering_or_resigned_unsafe_prefix_rejected(tmp_path,monkeypatch,change):
    from workbench.research.experiments.hug_home_rl.data import load_plan,sha
    out,plan,_,_=prepare_fixture(tmp_path,monkeypatch)
    if change=='model':(out/'initial-model.pt').write_bytes(b'changed')
    if change=='policy':(out/'initial-policy.npz').write_bytes(b'changed')
    if change=='ordering':plan['pool_ticks'].reverse()
    if change=='budget':plan['smoothness']['budget']=2
    if change=='fault_resigned':
        with np.load(out/'reference-path.npz',allow_pickle=False) as d:path={k:d[k].copy() for k in d.files}
        path['fault'][3]=True;np.savez_compressed(out/'reference-path.npz',**path)
        plan['reference_path_sha256']=sha(out/'reference-path.npz')
    (out/'plan.json').write_text(json.dumps(plan))
    with pytest.raises((ValueError,RuntimeError)):load_plan(out)
