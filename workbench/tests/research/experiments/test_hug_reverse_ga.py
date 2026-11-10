"""Reverse selection and physics-tick gates, without Genesis/GPU."""
import numpy as np
import pytest
from workbench.research.experiments.hug_reverse_ga.criteria import (
    Tracker,admission_good,home_good,tilt,error,violations)
from workbench.research.experiments.hug_reverse_ga.search import seed_population,decode,quality,breed

HOME=np.array([-.12,0,-.59,.59]);GOAL=np.array([0,-np.pi/2,-.31,-.56])
LO=np.array([-.173333,-np.pi/2,-np.pi/5,-np.pi/5]);HI=np.array([0,np.pi/2,np.pi/5,np.pi/5])
RATE=np.array([.0005,.01,.003,.003]);ORIGIN=np.array([.3,.1,.55])


def sample(actual=HOME):
    return dict(actual=np.array([actual],float),actual_vel=np.zeros((1,4)),
        object_pos=np.array([ORIGIN]),object_quat=np.array([[1.,0,0,0]]),
        object_vel=np.zeros((1,3)),object_ang=np.zeros((1,3)),floor=np.array([True]),
        touch=np.ones((1,2)),fault=np.array([False]),wrong=np.array([False]),
        contact_force=np.zeros(1),contact_penetration=np.zeros(1))


def update(t,s,command=HOME,previous=HOME):
    return t.update(s,np.array([command]),np.array([previous]),HOME,LO,HI,RATE,ORIGIN)


def test_exact_home_requires_full_continuous_second():
    t=Tracker(1,[0.],.01);s=sample()
    for _ in range(99):update(t,s)
    assert not t.success[0]
    update(t,s);assert t.success[0] and not t.active[0]


@pytest.mark.parametrize('kind',['contact','penetration','tracking','speed','object_speed','angular','support','command'])
def test_home_window_resets(kind):
    t=Tracker(1,[0.],.01);s=sample()
    for _ in range(50):update(t,s)
    command=HOME.copy()
    if kind=='contact':s['contact_force'][0]=.001
    if kind=='penetration':s['contact_penetration'][0]=.002
    if kind=='tracking':s['actual'][0,2]+=.02
    if kind=='speed':s['actual_vel'][0,2]=.01
    if kind=='object_speed':s['object_vel'][0,0]=.02
    if kind=='angular':s['object_ang'][0,0]=.2
    if kind=='support':s['floor'][0]=False
    if kind=='command':command[0]+=.0001
    update(t,s,command)
    assert t.window[0]==0 and not t.success[0]


def test_rejection_permanent_and_freezes_scores():
    t=Tracker(1,[8.],.01);s=sample();s['wrong'][0]=True
    update(t,s);assert t.rejected[0] and t.first_tick[0]==1
    before=t.reports()
    for _ in range(101):update(t,sample())
    assert t.reports()==before and not t.success[0]


def test_airborne_xy_limit_not_exempt_or_rebased():
    s=sample();s['floor'][0]=False;s['object_pos'][0,0]+=.050001
    mask,_,_=violations(s,np.array([HOME]),np.array([HOME]),LO,HI,RATE,ORIGIN)
    assert mask[0,2]
    s['object_pos'][0]=ORIGIN+[0,0,.2]
    mask,_,_=violations(s,np.array([HOME]),np.array([HOME]),LO,HI,RATE,ORIGIN)
    assert not mask[0,2]


def test_tilt_uses_orientation():
    s=sample();s['object_quat'][0]=[np.cos(np.deg2rad(6)),np.sin(np.deg2rad(6)),0,0]
    mask,_,angle=violations(s,np.array([HOME]),np.array([HOME]),LO,HI,RATE,ORIGIN)
    assert mask[0,3] and np.isclose(angle[0],np.deg2rad(12))
    with pytest.raises(RuntimeError):tilt([[0,0,0,0]])


def test_grasp_loss_and_home_s_shape_legal():
    s=sample();s['touch'][:]=0
    assert home_good(s,np.array([HOME]),HOME)[0]
    assert not admission_good(s,GOAL)[0]
    s=sample(GOAL);assert admission_good(s,GOAL)[0]
    s['touch'][0,1]=0;assert not admission_good(s,GOAL)[0]


def test_total_bend_tolerance():
    s=sample();s['actual'][0,2]+=np.deg2rad(3.1)/5
    assert error(s['actual'],HOME)[0]>1


def test_tick_bookkeeping_remains_mutable_after_return(monkeypatch):
    torch=pytest.importorskip('torch')
    from types import SimpleNamespace
    from workbench.research.experiments.hug_reverse_ga import physics
    env=SimpleNamespace(device='cpu')
    def simulate(command):
        env.bookkeeping=command.sum(dim=1)>0
    env.tick=simulate
    monkeypatch.setattr(physics,'sample',lambda e:None)
    physics.tick(env,np.ones((1,4)))
    env.bookkeeping[:]=False  # Same in-place operation used by HugEnv.reset().
    assert not env.bookkeeping.any()


def test_search_fixed_endpoints_slew_and_searchable_duration():
    rng=np.random.default_rng(1);p=seed_population(8,3,GOAL,HOME,LO,HI,rng)
    p[:, -1,4]=0
    paths=decode(p,GOAL,HOME,LO,HI,RATE,2000)
    assert np.allclose(paths[:,0],GOAL) and np.allclose(paths[:,-1],HOME)
    assert np.all(abs(np.diff(paths,axis=1))<=RATE+1e-12)
    assert np.all(paths>=LO-1e-12) and np.all(paths<=HI+1e-12)
    slow=p.copy();slow[:,-1,4]=1
    assert not np.allclose(decode(slow,GOAL,HOME,LO,HI,RATE,2000)[:,1000],paths[:,1000])
    p[0,0,0]=np.nan
    with pytest.raises(ValueError):decode(p,GOAL,HOME,LO,HI,RATE,20)


def test_selection_success_over_clean_over_rejected():
    t=Tracker(1,[9.],.01);r=t.reports()[0]
    success=dict(r,success=True,duration_s=20.,home_error=1.)
    clean=dict(r,home_error=0.)
    rejected=dict(r,rejected=True,last_valid_error=0.)
    assert quality(success)>quality(clean)>quality(rejected)
    assert quality(dict(clean,home_window_s=.5))>quality(clean)
    p=np.random.default_rng(0).random((8,4,5))
    reports=[rejected]*7+[success]
    children=breed(p,reports,np.random.default_rng(4))
    assert np.array_equal(children[0],p[7]) and children.shape==p.shape


@pytest.mark.parametrize('drift,expected',[(.05,False),(.050001,True)])
def test_xy_boundary(drift,expected):
    s=sample();origin=np.array([0.,0.,.55]);s['object_pos'][0]=[drift,0,.55]
    mask,_,_=violations(s,np.array([HOME]),np.array([HOME]),LO,HI,RATE,origin)
    assert bool(mask[0,2])==expected


def test_admission_checks_entire_settle_not_only_last_window(tmp_path,monkeypatch):
    from types import SimpleNamespace
    from workbench.research.experiments.hug_reverse_ga import physics
    initial=sample(GOAL);calls=[]
    env=SimpleNamespace(num_envs=1,s={'dt':.01},case={'goal_command':GOAL.tolist(),'object_xyz_m':ORIGIN.tolist()},
        experiment={'simulation_context':{}},reset=lambda:None)
    monkeypatch.setattr(physics,'sample',lambda e:initial)
    monkeypatch.setattr(physics,'arrays',lambda e:(LO,HI,RATE))
    def tick(e,c):
        s=sample(GOAL);calls.append(1)
        if len(calls)==1:s['object_pos'][0,0]+=.051
        return s
    monkeypatch.setattr(physics,'tick',tick)
    assert not physics.admit(env,tmp_path/'admission')
    import json
    result=json.loads((tmp_path/'admission/report.json').read_text())
    assert result['status']=='initial_grasp_invalid' and result['violation_any'][0][2]
    assert len(calls)==100


def test_admission_missing_loaded_contact(tmp_path,monkeypatch):
    from types import SimpleNamespace
    from workbench.research.experiments.hug_reverse_ga import physics
    s=sample(GOAL);s['touch'][:]=0
    env=SimpleNamespace(num_envs=1,s={'dt':.01},case={'goal_command':GOAL.tolist(),'object_xyz_m':ORIGIN.tolist()},
        experiment={'simulation_context':{}},reset=lambda:None)
    monkeypatch.setattr(physics,'sample',lambda e:s)
    monkeypatch.setattr(physics,'arrays',lambda e:(LO,HI,RATE))
    monkeypatch.setattr(physics,'tick',lambda e,c:s)
    assert not physics.admit(env,tmp_path/'admission')


def test_evaluate_restores_full_snapshot_and_timeout_not_success(tmp_path,monkeypatch):
    from types import SimpleNamespace
    from workbench.research.experiments.hug_reverse_ga import physics
    calls=[];snapshot=object();s=sample(GOAL)
    env=SimpleNamespace(num_envs=1,s={'dt':.01},case={'home_command':HOME.tolist(),'object_xyz_m':ORIGIN.tolist()},
        experiment={'simulation_context':{}},reset=lambda:calls.append('reset'),
        scene=SimpleNamespace(go2_leg_dofs=[],scene=SimpleNamespace(reset=lambda state:calls.append(state))),
        mapper=SimpleNamespace(waypoint=np.zeros((1,4))),start=GOAL)
    monkeypatch.setattr(physics,'sample',lambda e:s)
    monkeypatch.setattr(physics,'arrays',lambda e:(LO,HI,RATE))
    monkeypatch.setattr(physics,'tick',lambda e,c:s)
    reports=physics.evaluate(env,np.tile(GOAL,(1,6,1)),tmp_path,snapshot)
    assert calls==['reset',snapshot]
    assert not reports[0]['success'] and not reports[0]['rejected']
    data=np.load(tmp_path/'states.npz');assert len(data['time_s'])==6


@pytest.mark.parametrize('admitted,expected',[(False,'initial_grasp_invalid'),(True,'verification_failed')])
def test_orchestration_admission_and_replay_fail_closed(tmp_path,monkeypatch,admitted,expected):
    from contextlib import nullcontext
    from types import SimpleNamespace
    from workbench.research.experiments.hug_reverse_ga import experiment,physics
    from workbench.research.experiments.hug_ga import parade
    case={'goal_command':GOAL.tolist(),'home_command':HOME.tolist()}
    env=SimpleNamespace(s={'dt':.01},scene=SimpleNamespace(scene=SimpleNamespace(get_state=lambda:object())))
    monkeypatch.setattr(parade,'create_env',lambda *a,**k:env)
    recorded=[]
    def record(env,path):
        recorded.append(path)
        return nullcontext()
    monkeypatch.setattr(parade,'record',record)
    monkeypatch.setattr(physics,'admit',lambda *a:admitted)
    monkeypatch.setattr(physics,'sample',lambda e:sample(GOAL))
    monkeypatch.setattr(physics,'arrays',lambda e:(LO,HI,RATE))
    calls=[]
    def evaluate(*args):
        calls.append(1)
        r=Tracker(1,[0.],.01).reports()[0]
        return [dict(r,success=len(calls)==1)]*2
    monkeypatch.setattr(physics,'evaluate',evaluate)
    plan=dict(cases=[case],population=2,home=HOME.tolist(),knots=1,seed=42,generations=1,parade_every=10)
    result=experiment.search(plan,tmp_path,'cpu')
    assert result['status']==expected
    assert len(calls)==(5 if admitted else 0)
    if admitted:assert tmp_path/'final-population/parade.mp4' in recorded
    if admitted:
        import json
        data=json.loads((tmp_path/'curriculum.json').read_text())
        assert data['status']=='verification_failed' and not data['reset_pool_validated']


def test_leg_hold_reissued_for_selected_lanes_without_pose_teleport(monkeypatch):
    import sys
    from types import SimpleNamespace
    from workbench.research.experiments.hug_reverse_ga.physics import restore_leg_hold
    calls=[]
    class Tensor:
        def __init__(self,data):self.data=np.array(data)
        def repeat(self,n):return Tensor(np.tile(self.data,n))
        def expand(self,n,width):return np.broadcast_to(self.data,(n,width))
    monkeypatch.setitem(sys.modules,'torch',SimpleNamespace(tensor=lambda x,**k:Tensor(x),float32='float32'))
    robot=SimpleNamespace(control_dofs_position=lambda pose,**kw:calls.append((pose.copy(),kw)))
    env=SimpleNamespace(num_envs=4,device='cpu',scene=SimpleNamespace(robot=robot,go2_leg_dofs=list(range(12)),
        cfg=SimpleNamespace(scene=SimpleNamespace(go2=SimpleNamespace(leg_pose_rad=[0.,.8,-1.5])))))
    restore_leg_hold(env);restore_leg_hold(env,[1,3])
    assert calls[0][0].shape==(4,12) and calls[1][0].shape==(2,12)
    assert calls[1][1]['envs_idx']==[1,3]
    assert np.array_equal(calls[0][0][0],np.tile([0.,.8,-1.5],4))
