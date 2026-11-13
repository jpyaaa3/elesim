"""Pipeline contracts: cold start, exact anchors, phase alignment and safe resume."""
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from workbench.research.experiments.hug_pipeline import contracts,reference,learning,evaluation,coordinator
from workbench.research.experiments.hug_c.configure import DEFAULTS,load_mechanism


def fixture():
    home=np.asarray(load_mechanism().presets['home']);goal=np.array([0,-np.pi/2,-.3,-.5]);n=1601
    cmd=np.array([np.linspace(home[j],goal[j],n-100) for j in range(4)]).T
    cmd=np.vstack((cmd,np.tile(goal,(100,1))))
    path=dict(actual=cmd.copy(),actual_vel=np.zeros((n,4)),joint_q=np.zeros((n,24)),joint_qd=np.zeros((n,24)),
        object_pos=np.tile([.4,.1,.55],(n,1)),object_quat=np.tile([1.,0,0,0],(n,1)),
        object_vel=np.zeros((n,3)),object_ang=np.zeros((n,3)),touch=np.ones((n,2)),floor=np.ones(n,bool),
        fault=np.zeros(n,bool),wrong=np.zeros(n,bool),contact_force=np.ones(n),contact_penetration=np.zeros(n),
        command=cmd,time_s=np.arange(n)*.01)
    case=dict(case_id='test',goal_command=goal.tolist(),object_xyz_m=[.4,.1,.55],diameter_mm=100)
    plan=dict(home=home.tolist(),settings=dict(DEFAULTS),case=case)
    return plan,path


def test_forward_contract_allows_measured_object_motion_but_requires_wrap():
    p,path=fixture();path['object_pos'][100:,0]+=.02
    assert reference.validate(path,p)==1600
    path['touch'][-1,1]=0
    with pytest.raises(ValueError,match='Wrap'):reference.validate(path,p)


@pytest.mark.parametrize('damage',['home','velocity','clock','wrong','drift','slew','nan'])
def test_forward_rejects_invalid_reset_or_physics(damage):
    p,path=fixture()
    if damage=='home':path['command'][0,0]+=.01
    if damage=='velocity':path['joint_qd'][0,0]=1
    if damage=='clock':path['time_s'][1]+=.005
    if damage=='wrong':path['wrong'][500]=True
    if damage=='drift':path['object_pos'][500,0]+=.051
    if damage=='slew':path['command'][500,0]+=.02
    if damage=='nan':path['object_vel'][500,0]=np.nan
    with pytest.raises((ValueError,RuntimeError)):reference.validate(path,p)


def test_cold_plan_has_no_forward_or_policy_prerequisite(tmp_path,monkeypatch):
    from workbench.tests.research.experiments.test_hug_reverse_rl import path_fixture
    from workbench.research.experiments.hug_reverse_rl import data
    from workbench.research.experiments.hug_home_rl.schedule import DEFAULTS as SCHEDULE
    path,ga=path_fixture();case=dict(ga['cases'][0],case_id='test')
    ga['cases']=[dict(case,start_command=case['goal_command'])]
    study=dict(settings=ga['settings'],home=ga['home'],shared_context=contracts.shared(ga['settings']),seed=42)
    def verified_input(source,output,**kwargs):
        assert source==tmp_path/'reverse'
        output.mkdir();np.savez_compressed(output/'reference-path.npz',**path)
        return dict(reverse_plan=ga,case=ga['cases'][0],settings=ga['settings'],home=ga['home'],
            reference_path_sha256=contracts.sha(output/'reference-path.npz'),state_tolerances={})
    monkeypatch.setattr(data,'prepare',verified_input)
    out=tmp_path/'plan';prepared=learning.prepare(study,case,tmp_path/'reverse',out)
    loaded,_=learning.load(out)
    assert prepared==loaded and loaded['initial_checkpoint_sha256'] is None
    assert len(loaded['pool_ticks'])<=32 and loaded['pool_ticks'][0]==0
    assert loaded['pool_ticks'][-1]==501
    assert loaded['curriculum']['overlap_bins']==SCHEDULE['overlap_bins']
    assert loaded['scripted_episode_actions'] is False
    loaded['pool_ticks'].reverse();contracts.save(out/'plan.json',loaded)
    with pytest.raises(ValueError,match='ordering'):learning.load(out)


def test_exact_anchor_plan_does_not_reinvert_xy(tmp_path,monkeypatch):
    from workbench.research.experiments.hug_pipeline.planning import reverse_plan
    from workbench.research.experiments.hug_c import source
    row=dict(q_deg=[-90.,-160.],xy_mm=[-100.,180.],feature='test',root=0,mu_min=.2)
    home=list(load_mechanism().presets['home']);case=contracts.make_case(row,100,home)
    study=dict(home=home,settings=dict(DEFAULTS),shared_context=contracts.shared(DEFAULTS),mu_max=.4,
        budgets=dict(population=128,generations=20),knots=6,seed=42)
    monkeypatch.setattr(source,'read_kernel',lambda *a,**kw:('kernel',{}))
    monkeypatch.setattr(source,'evaluate',lambda *a,**kw:[row])
    monkeypatch.setattr(source,'inverse',lambda *a,**kw:pytest.fail('Do not choose another inverse root'))
    result=reverse_plan(study,case,tmp_path/'out',tmp_path/'mapper')
    assert result['cases'][0]['q_deg']==row['q_deg']
    assert result['cases'][0]['goal_command']==case['goal_command']


def test_alignment_stretches_each_phase_and_barycentric_vertex_is_exact():
    p,_=fixture();home=np.asarray(p['home']);paths=[]
    for lengths in ([20,30,40,500],[25,20,50,500],[15,35,30,500]):
        events=np.r_[0,np.cumsum(lengths)];c=np.tile(home,(events[-1]+1,1));c[:,0]=np.linspace(home[0],0,len(c))
        paths.append(dict(commands=c,events=events))
    aligned,events=evaluation.align(paths,p)
    assert np.array_equal(np.diff(events),[25,35,50,500])
    command,w=evaluation.blend(aligned,[[-100,-170],[-70,-170],[-100,-140]],[-70,-170],p)
    assert np.array_equal(command,aligned[1]) and np.array_equal(w,[0,1,0])
    with pytest.raises(ValueError,match='Extrapolation'):
        evaluation.blend(aligned,[[-100,-170],[-70,-170],[-100,-140]],[-60,-170],p)


def test_same_case_identity_is_independent_of_split_and_mu_rounding():
    row=dict(q_deg=[-90.,-160.],xy_mm=[-100.,180.],feature='test',root=0,mu_min=.2)
    home=list(load_mechanism().presets['home']);a=contracts.make_case(row,100,home)
    b=contracts.make_case(dict(row,split='test',mu_min=.200000001),100,home)
    assert a['case_id']==b['case_id']
    assert a['case_id']!=contracts.make_case(row,125,home)['case_id']


def test_artifact_hash_prevents_silent_reuse(tmp_path):
    (tmp_path/'policy.npz').write_bytes(b'first');h=contracts.files_hashes(tmp_path)
    contracts.verify_files(tmp_path,h);(tmp_path/'policy.npz').write_bytes(b'changed')
    with pytest.raises(ValueError,match='changed'):contracts.verify_files(tmp_path,h)


def test_latest_checkpoint_only_uses_verified_pair(tmp_path):
    for i in (100,200):
        p=tmp_path/f'model_{i}.pt';p.write_bytes(str(i).encode())
        contracts.save(Path(str(p)+'.curriculum.json'),dict(checkpoint_sha256=contracts.sha(p),curriculum={'steps':i*16}))
    assert coordinator.Coordinator.latest_checkpoint(tmp_path).name=='model_200.pt'
    (tmp_path/'model_200.pt').write_bytes(b'incomplete')
    assert coordinator.Coordinator.latest_checkpoint(tmp_path).name=='model_100.pt'


def test_reverse_seed_endpoints_stay_within_limits():
    p,path=fixture();lo=np.array([-.173,-np.pi/2,-np.pi/5,-np.pi/5]);hi=np.array([0,np.pi/2,np.pi/5,np.pi/5])
    seeds=reference.seeds(path['command'],128,6,lo,hi,np.random.default_rng(0))
    assert seeds.shape==(128,7,5) and np.min(seeds)>=0 and np.max(seeds)<=1


def coordinator_fixture(tmp_path,monkeypatch):
    contracts.save(tmp_path/'plan.json',{'test':True})
    monkeypatch.setattr(coordinator,'load_study',lambda root:{'budgets':{'iterations':20}})
    return coordinator.Coordinator(tmp_path)


def test_completed_stage_reuses_only_unchanged_inputs_and_outputs(tmp_path,monkeypatch):
    c=coordinator_fixture(tmp_path,monkeypatch);out=tmp_path/'output';out.mkdir();(out/'x.json').write_text('{}')
    payload=dict(root=str(tmp_path),kind='test',case={'id':1})
    c.state['stages']['unit']=dict(status='complete',input_sha256=contracts.canonical(payload),output=str(out),artifacts=contracts.files_hashes(out))
    assert c.stage('unit','test',case={'id':1})==out
    with pytest.raises(ValueError,match='inputs'):c.stage('unit','test',case={'id':2})
    (out/'x.json').write_text('{"modified":true}')
    with pytest.raises(ValueError,match='changed'):c.stage('unit','test',case={'id':1})


def test_failed_physics_stage_cannot_be_marked_ready_on_resume(tmp_path,monkeypatch):
    c=coordinator_fixture(tmp_path,monkeypatch)
    payload=dict(root=str(tmp_path),kind='forward',case={})
    c.state['stages']['f']=dict(status='failed',input_sha256=contracts.canonical(payload),result={'accepted':False,'status':'forward_reference_failed'})
    with pytest.raises(coordinator.StageFailed):c.stage('f','forward',case={})


def test_reference_does_not_accept_partial_joint_or_contact_states():
    p,path=fixture();path['joint_q']=path['joint_q'][:,:2];path['joint_qd']=path['joint_qd'][:,:2]
    with pytest.raises(ValueError,match='joint'):reference.validate(path,p)


def test_alignment_rejects_event_order_incompatible_with_interpolation():
    p,path=fixture();bad=dict(commands=path['command'],events=np.array([0,100,90,500,1600]))
    with pytest.raises(ValueError,match='ordering'):evaluation.align([bad]*3,p)


def test_reselection_keeps_unchanged_anchor_case_not_triangle_weights():
    row=dict(q_deg=[-90.,-160.],xy_mm=[-100.,180.],feature='test',root=0,mu_min=.2)
    home=list(load_mechanism().presets['home'])
    a=contracts.make_case(dict(row,q_barycentric=[1,0,0]),100,home)
    b=contracts.make_case(dict(row,q_barycentric=[0,1,0]),100,home)
    assert a==b and 'q_barycentric' not in a


def test_coordinator_runs_and_reuses_verified_worker_outputs(tmp_path,monkeypatch):
    c=coordinator_fixture(tmp_path,monkeypatch);calls=[]
    class Process:
        pid=999999999
        def __init__(self,args,**kwargs):
            job=json.loads(Path(args[-1]).read_text());calls.append(job)
            output=Path(job['output']);output.mkdir()
            contracts.save(output/'report.json',{'measured':True})
            contracts.save(Path(job['result']),{'accepted':True})
        def wait(self,**kwargs):return 0
    monkeypatch.setattr(coordinator.subprocess,'Popen',Process)
    first=c.stage('unit','fake',case={'id':1});assert first.is_dir()
    assert c.stage('unit','fake',case={'id':1})==first and len(calls)==1
    reloaded=coordinator.Coordinator(tmp_path)
    assert reloaded.stage('unit','fake',case={'id':1})==first and len(calls)==1
    assert reloaded.state['stages']['unit']['status']=='complete'


def test_interrupted_training_resumes_with_remaining_budget(tmp_path,monkeypatch):
    c=coordinator_fixture(tmp_path,monkeypatch);prior=tmp_path/'prior';prior.mkdir()
    checkpoint=prior/'model_4.pt';checkpoint.write_bytes(b'checkpoint')
    contracts.save(Path(str(checkpoint)+'.curriculum.json'),dict(checkpoint_sha256=contracts.sha(checkpoint),curriculum={'steps':80}))
    payload=dict(root=str(tmp_path),kind='train',case={})
    c.state['stages']['t']=dict(status='interrupted',input_sha256=contracts.canonical(payload),attempt=1,output=str(prior))
    class Process:
        pid=999999999
        def __init__(self,args,**kwargs):
            job=json.loads(Path(args[-1]).read_text());assert job['iterations']==15 and job['resume']==str(checkpoint)
            Path(job['output']).mkdir();contracts.save(Path(job['result']),{'accepted':True,'home_verified':False})
        def wait(self,**kwargs):return 0
    monkeypatch.setattr(coordinator.subprocess,'Popen',Process)
    c.stage('t','train',case={})
    assert c.state['stages']['t']['result']['home_verified'] is False


@pytest.mark.parametrize('policy_passes',[True,False])
def test_full_stage_graph_publishes_only_after_home_and_interpolation_gates(tmp_path,monkeypatch,policy_passes):
    """Mock orchestration only: this is deliberately not physics evidence."""
    from workbench.research.experiments.hug_pipeline import planning
    from workbench.research.experiments.hug_c import source
    p,_=fixture();p.update(diameter_mm=100,mu_max=.4,budgets={'max_reselections':2},shared_context={})
    monkeypatch.setattr(coordinator,'load_study',lambda root:p)
    contracts.save(tmp_path/'plan.json',{'fixture':True})
    c=coordinator.Coordinator(tmp_path)
    rows=[dict(q_deg=q,xy_mm=[-100.,180.],feature='test',root=0,mu_min=.2) for q in ([-100,-170],[-70,-170],[-100,-140],[-90,-160])]
    cases=[contracts.make_case(dict(r,split='train' if i<3 else 'test'),100,p['home']) for i,r in enumerate(rows)]
    monkeypatch.setattr(planning,'choose',lambda *args:({},cases))
    monkeypatch.setattr(source,'read_kernel',lambda *args,**kw:('',{}))
    monkeypatch.setattr(source,'evaluate',lambda *args,**kw:[None]*4)
    visited=[]
    def stage(key,kind,**kwargs):
        visited.append(kind);out=tmp_path/'fake'/key;out.mkdir(parents=True,exist_ok=True)
        if kind=='train':
            (out/'training').mkdir();(out/'training/policy.npz').write_bytes(b'not a physics policy; mock orchestration')
        if kind=='policy' and not policy_passes:raise coordinator.StageFailed(key,{'accepted':False})
        if kind in ('policy','replay'):
            commands=np.tile(p['home'],(601,1));commands[:,0]=np.linspace(p['home'][0],0,601)
            np.savez_compressed(out/'rollout-1.npz',commands=commands,events=[0,25,50,100,600])
        return out
    c.stage=stage
    if policy_passes:
        c.run();assert c.state['status']=='published'
        publication=json.loads((tmp_path/'publication.json').read_text())
        assert publication['ready'] and len(publication['anchors'])==3
        assert visited.count('reverse')==visited.count('train')==3
        assert 'forward' not in visited
        assert visited.count('policy')==visited.count('replay')==visited.count('aligned')==3
        assert visited[-1]=='interpolation'
    else:
        with pytest.raises(coordinator.StageFailed):c.run()
        assert 'aligned' not in visited and 'interpolation' not in visited
        assert not (tmp_path/'publication.json').exists()


def test_forward_selection_does_not_reward_subtolerance_q_noise_over_contact():
    base=dict(success=False,rejected=False,best_wrap_ticks=0,near_goal_contacts=0,pose_error=.001,final_xy_m=.02,peak_xy_m=.02)
    touching=dict(base,pose_error=.9,near_goal_contacts=1)
    assert reference.rank(touching)>reference.rank(base)
    wrapped=dict(base,success=True,best_wrap_ticks=30,near_goal_contacts=2)
    assert reference.rank(wrapped)>reference.rank(touching)
    assert reference.rank(dict(base,pose_error=.99,final_xy_m=.001))>reference.rank(base)


def test_reverse_reuse_requires_same_physical_root_not_just_same_xy():
    from workbench.research.experiments.hug_pipeline.reuse import compatible
    from workbench.research.experiments.hug_reverse_ga.prepare import digest
    p,_=fixture();p['shared_context']=contracts.shared(p['settings'])
    row=dict(q_deg=[-90.,-160.],xy_mm=[-100.,180.],feature='test',root=0,mu_min=.2)
    case=contracts.make_case(row,100,p['home'])
    old=dict(schema='hug-reverse-ga-plan',version=1,source_sha256=digest(),settings=p['settings'],home=p['home'],simulation_context=p['shared_context'],cases=[dict(case,q_barycentric=[1,0,0])])
    assert compatible(p,case,old)
    old['cases'][0]['root']=1;assert not compatible(p,case,old)
    old['cases'][0]=case;old['source_sha256']='outdated';assert not compatible(p,case,old)


def test_contact_based_pause_never_introduces_command_jump():
    path=np.column_stack([np.linspace(0,.1,101)]*4)
    paused=reference.pause_proposal(path,50,30)
    assert paused.shape==path.shape
    assert np.allclose(paused[:50],path[:50])
    assert np.allclose(paused[50:80],path[50])
    assert np.max(abs(np.diff(paused,axis=0)))<=np.max(abs(np.diff(path,axis=0)))+1e-12


def test_seed_commands_are_rate_limited_from_true_home():
    home=np.array([-.12,0,-.59,.59]);goal=np.array([0,-1.57,-.3,-.5]);rate=np.array([.0005,.01,.003,.003])
    path=reference.slew_proposal(np.tile(goal,(500,1)),home,goal,rate,600)
    assert np.array_equal(path[0],home)
    assert np.all(abs(np.diff(path,axis=0))<=rate+1e-12)
    assert np.allclose(path[-1],goal)


def test_forward_reference_rejects_cumulative_roll_detour():
    p,path=fixture()
    # Within angle bounds and per-tick slew, same endpoints, excessive round trip.
    path['command'][:401,1]=np.linspace(0,.3,401)
    path['command'][401:801,1]=np.linspace(.3,-.8,400)
    path['command'][801:1201,1]=np.linspace(-.8,-.5,400)
    path['command'][1201:1501,1]=np.linspace(-.5,-np.pi/2,300)
    with pytest.raises(ValueError,match='motion budget'):
        reference.validate(path,p)


def test_ppo_worker_consumes_reverse_plan_without_forward_stage(tmp_path,monkeypatch):
    from workbench.research.experiments.hug_pipeline import worker
    study={'settings':{}};case={'case_id':'anchor'};calls=[]
    monkeypatch.setattr(worker,'load_study',lambda root:study)
    monkeypatch.setattr(learning,'prepare',lambda *args:calls.append(args))
    out=tmp_path/'output';reverse=tmp_path/'reverse'
    result=worker.run(dict(root=str(tmp_path),kind='ppo-plan',case=case,
                           output=str(out),reverse=str(reverse)))
    assert result['accepted'] and calls==[(study,case,reverse/'plan',out/'plan')]


def test_reverse_environment_retains_ppo_actions_and_uses_reverse_bank():
    # Source inspection also runs on the lightweight host without Torch/Genesis.
    import ast
    source=Path(learning.__file__).with_name('environment.py').read_text()
    tree=ast.parse(source);cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ReverseHomeEnv')
    assert cls.bases[0].id=='HomeEnv'
    assert not any(isinstance(n,ast.FunctionDef) and n.name in ('step','reset') for n in cls.body)
    assignments={n.targets[0].id:n.value.id for n in cls.body if isinstance(n,ast.Assign)}
    assert assignments['reset_bank_class']=='LinearResetBank'
    assert assignments['core_env_class']=='LinearHugEnv'


def test_fixed_selection_preserves_original_anchors_and_requires_home(tmp_path,monkeypatch):
    from workbench.research.experiments.hug_pipeline import planning,reuse
    from workbench.research.experiments.hug_c import source
    home=list(load_mechanism().presets['home'])
    qs=[[-90.,-160.],[-60.,-160.],[-60.,-130.],[-70.,-150.],[-75.,-155.]]
    rows=[dict(q_deg=q,xy_mm=[-100.,180.],feature='side',root=1,mu_min=.2) for q in qs]
    cases=[contracts.make_case(dict(r,split='train' if i<3 else 'validation' if i==3 else 'test'),100,home) for i,r in enumerate(rows)]
    selection=dict(pilot={'original':True},cases=cases);contracts.save(tmp_path/'fixed-selection.json',selection)
    contracts.save(tmp_path/'plan.json',{})
    study=dict(fixed_selection_sha256=contracts.sha(tmp_path/'fixed-selection.json'),validation=1,test=1,
               diameter_mm=100,mu_max=.4,home=home,reverse_inputs=[dict(directory=str(tmp_path))])
    screen=tmp_path/'screen.json'
    contracts.save(screen,dict(results=[dict(q_deg=q,home_passed=True,wrap_admitted=False) for q in qs[:3]]))
    monkeypatch.setattr(source,'read_kernel',lambda *a,**k:('kernel',{}))
    monkeypatch.setattr(source,'evaluate',lambda *a,**k:rows)
    monkeypatch.setattr(reuse,'compatible',lambda *a:True)
    assert planning.fixed_selection(tmp_path,study,[],screen)==(selection['pilot'],cases)
    with pytest.raises(ValueError,match='no automatic reselection'):planning.fixed_selection(tmp_path,study,[qs[0]],screen)
    contracts.save(screen,dict(results=[]))
    with pytest.raises(ValueError,match='Home screen failed'):planning.fixed_selection(tmp_path,study,[],screen)
