"""Reward ordering and real-model contact-frame regressions; no physics claims."""
import math
import numpy as np
import pytest
from workbench.research.experiments.hug_c.configure import DEFAULTS,OBSERVATIONS
from workbench.research.experiments.hug_c.contacts import camera_side_axes,classify_faces,object_contact_faces
from workbench.research.experiments.hug_c.control import pose_error
from workbench.research.experiments.hug_c.rewards import RewardLedger,q_potential


HOME=np.array([-.12,0.,math.radians(-34),math.radians(34)])
GOAL=np.array([0.,0.,math.radians(-.1),math.radians(-35.7)])


def flags(n=1,**changes):
    values=dict(confirmed_grasp=np.zeros(n,bool),lift_fraction=np.zeros(n),hold_fraction=np.zeros(n),
                wrong_fraction=np.zeros(n),fault=np.zeros(n,bool),success=np.zeros(n,bool),
                timeout=np.zeros(n,bool),drops=np.zeros(n))
    values.update({k:np.full(n,v) for k,v in changes.items()})
    return values


def ledger(n=1):
    r=RewardLedger(n,DEFAULTS);r.reset(np.arange(n),np.tile(HOME,(n,1)),GOAL);return r


def test_q_hint_ignores_linear_and_roll_in_both_actual_and_goal():
    a=np.tile(HOME,(2,1));a[1,:2]=[0.,math.pi/2]
    g=GOAL.copy();other=g.copy();other[:2]=[-.15,.8]
    np.testing.assert_array_equal(q_potential(a,g,DEFAULTS),q_potential(a,other,DEFAULTS))
    assert q_potential(a,g,DEFAULTS)[0]==q_potential(a,g,DEFAULTS)[1]
    assert pose_error(a,g,DEFAULTS)[0]==pose_error(a,g,DEFAULTS)[1]
    assert 'goal_linear_m' not in OBSERVATIONS and 'goal_roll_rad' not in OBSERVATIONS


def test_camera_side_axes_come_from_actual_mount_and_allow_tip_caps():
    axes=camera_side_axes()
    np.testing.assert_allclose(axes['node0'],[0,0,-1],atol=1e-7)
    for name in ('gripper_base','gripper_claw_left','gripper_claw_right'):
        np.testing.assert_allclose(axes[name],[-1,0,0],atol=1e-7)
    points=np.array([[.05,0,.035],[.05,0,-.035],[0,0,0],[0,0,.035]])
    normals=np.array([[1,0,0],[1,0,0],[1,0,0],[0,0,-1]])
    good,bad=classify_faces(points,normals,np.zeros_like(points),np.tile([1,0,0,0],(4,1)),np.array([0,0,-1]))
    np.testing.assert_array_equal(good,[True,False,False,False])
    np.testing.assert_array_equal(bad,[False,True,False,False])


@pytest.mark.parametrize('roll',[-math.pi/2,0.,math.pi/2])
@pytest.mark.parametrize('swap',[False,True])
def test_contact_sides_are_invariant_to_roll_translation_and_pair_order(roll,swap):
    # Global IDs differ from entity-local indices; padding has a spurious force.
    q=np.array([math.cos(roll/2),math.sin(roll/2),0,0])
    R=np.array([[1,0,0],[0,math.cos(roll),-math.sin(roll)],[0,math.sin(roll),math.cos(roll)]])
    pos=np.array([[[3.,4.,5.],[3.2,4.,5.]]]);quats=np.tile(q,(1,2,1))
    points=np.array([[0,0,.035],[-.025,0,0],[0,0,-.035]])@R.T+pos[0,[0,1,0]]
    outward=np.array([[0,0,1],[-1,0,0],[0,0,-1]])@R.T
    a=np.array([[5,8,-1]]);b=np.array([[9,9,-1]])
    if swap:a,b=b,a;outward=-outward
    raw=dict(link_a=a,link_b=b,position=points[None],normal=-outward[None],
             force_a=np.ones((1,3,3)),valid_mask=np.array([[True,True,False]]))
    roles=np.zeros(10,int);roles[5]=1;roles[8]=2
    objects=np.zeros(10,bool);objects[9]=True
    local=np.zeros(10,int);local[8]=1
    axes=np.zeros((10,3));axes[5]=[0,0,-1];axes[8]=[-1,0,0]
    touch,bad=object_contact_faces(raw,pos,quats,local,axes,roles,objects,DEFAULTS)
    np.testing.assert_array_equal(touch,[[True,False]])
    np.testing.assert_array_equal(bad,[True])


def test_wrong_face_blocks_every_positive_term_and_does_not_bank_progress():
    r=ledger()
    reward,done,terms=r.step(np.array([GOAL]),GOAL,np.zeros((1,4)),**flags(
        confirmed_grasp=True,lift_fraction=1.,hold_fraction=1.,wrong_fraction=.1,success=True))
    assert not done[0] and reward[0]<0
    assert all(v[0]<=0 for v in terms.values())
    assert not r.grasp_paid[0] and r.best_lift[0]==0 and r.best_hold[0]==0
    _,_,terms=r.step(np.array([GOAL]),GOAL,np.zeros((1,4)),**flags(confirmed_grasp=True,lift_fraction=1.))
    assert terms['grasp'][0]==3 and terms['lift'][0]==5


def test_grasp_lift_and_hold_milestones_cannot_be_farmed_after_a_reset():
    r=ledger();action=np.zeros((1,4))
    for _ in range(4):
        r.step(np.array([GOAL]),GOAL,action,**flags(confirmed_grasp=True,lift_fraction=.8,hold_fraction=.7))
        r.step(np.array([GOAL]),GOAL,action,**flags(drops=1.))
    assert r.totals['grasp'][0]==3
    assert r.totals['lift'][0]==pytest.approx(4)
    assert r.totals['hold'][0]==pytest.approx(7)
    assert r.totals['drop'][0]==-12
    r.step(np.array([GOAL]),GOAL,action,**flags(confirmed_grasp=True,lift_fraction=1.,hold_fraction=1.,success=True))
    assert r.totals['lift'][0]==pytest.approx(5) and r.totals['hold'][0]==pytest.approx(10)
    assert r.totals['success'][0]==30


def test_failed_q_only_motion_cannot_collect_extra_discounted_q_return():
    scores=[]
    for move in (False,True):
        r=ledger();score=0
        for i in range(200):
            q=HOME+(GOAL-HOME)*min(1,(i+1)/70) if move else HOME
            _,_,terms=r.step(q[None],GOAL,np.zeros((1,4)),**flags(timeout=i==199))
            score+=DEFAULTS['reward_gamma']**i*terms['q_hint'][0]
        scores.append(score)
    np.testing.assert_allclose(scores,[-q_potential(HOME,GOAL,DEFAULTS)]*2,atol=2e-6)


def test_fault_terminates_with_one_fixed_penalty_and_no_later_payments():
    r=ledger()
    reward,done,terms=r.step(GOAL[None],GOAL,np.zeros((1,4)),**flags(
        fault=True,confirmed_grasp=True,lift_fraction=1.,hold_fraction=1.,success=True))
    assert done[0] and terms['fault'][0]==-30 and all(v[0]<=0 for v in terms.values())
    reward,done,terms=r.step(GOAL[None],GOAL,np.zeros((1,4)),**flags(fault=True))
    assert reward[0]==0 and not done[0] and r.totals['fault'][0]==-30


def test_reset_is_scoped_and_costs_use_clipped_actions():
    r=ledger(2);q=np.tile(GOAL,(2,1))
    r.step(q,GOAL,np.full((2,4),100.),**flags(2,confirmed_grasp=True,lift_fraction=1.))
    np.testing.assert_allclose(r.totals['action'],[-.004,-.004])
    r.reset(np.array([0]),HOME[None],GOAL)
    assert not r.grasp_paid[0] and r.grasp_paid[1]
    assert r.best_lift[0]==0 and r.best_lift[1]==1


def test_scripted_reward_ordering_for_idle_push_grasp_drop_and_success():
    scores={}
    for name in ('idle','wrong_push','proper_grasp','lift_drop','success'):
        r=ledger();total=0
        for i in range(200):
            q=HOME if name=='idle' else HOME+(GOAL-HOME)*min(1,(i+1)/70)
            proper=name in ('proper_grasp','lift_drop','success') and i>=50
            lifted=name in ('lift_drop','success') and i>=60
            hold=max(0,min(1,(i-59)/50)) if lifted else 0
            drop=name=='lift_drop' and i==80
            if name=='lift_drop' and i>=80:proper=False;lifted=False;hold=0
            passed=name=='success' and hold>=1
            reward,done,_=r.step(q[None],GOAL,np.zeros((1,4)),**flags(
                confirmed_grasp=proper,lift_fraction=float(lifted),hold_fraction=hold,
                wrong_fraction=float(name=='wrong_push' and 20<=i<80),drops=int(drop),
                success=passed,timeout=i==199))
            total+=DEFAULTS['reward_gamma']**i*reward[0]
            if done[0]:break
        scores[name]=total
    assert scores['success']>scores['lift_drop']>scores['proper_grasp']>scores['idle']>scores['wrong_push'],scores


def test_audit_success_beats_slow_success_and_repeated_drops_cannot_farm():
    from workbench.research.experiments.hug_c.reward_audit import scenario_scores
    results={r['scenario']:r for r in scenario_scores()}
    assert results['success']['discounted_total']>results['slow_success']['discounted_total']
    assert results['empty_q_target']['discounted_total']<=results['idle']['discounted_total']
    repeated=results['repeat_lift_drop']['components']
    assert repeated['grasp']==3 and repeated['lift']<=5.00001 and repeated['hold']<=10.00001
    assert repeated['drop']< -3


def test_wrong_face_resets_hold_and_never_confirms_an_airborne_grasp():
    from workbench.research.experiments.hug_c.metrics import HoldTracker
    h=HoldTracker(2,DEFAULTS);q=np.tile(GOAL,(2,1));clear=np.full(2,.03)
    floor=np.zeros(2,bool);fault=np.zeros(2,bool);touch=np.ones((2,2));rel=np.zeros((2,3))
    invalid=np.array([False,True])
    for _ in range(500):out=h.update(clear,floor,touch,q,fault,rel,invalid)
    np.testing.assert_array_equal(out,[True,False])
    invalid[:]=True;h.update(clear,floor,touch,q,fault,rel,invalid)
    assert h.count[0]==0 and h.interrupted[0]


def test_runner_uses_task_discount_without_changing_the_legacy_task(monkeypatch):
    from types import SimpleNamespace
    from workbench.research.experiments.hug_c.experiment import runner
    from workbench.research.experiments.wrap_grasp.configs.loader import load_config,to_dict
    import elesim_sim.rl.runner as shared
    before=to_dict(load_config().train.runner)['algorithm']['gamma']
    monkeypatch.setattr(shared,'make_runner',lambda env,config,path:config)
    config=runner(SimpleNamespace(experiment={'settings':DEFAULTS},device='cpu'),None)
    assert config['algorithm']['gamma']==.995
    assert to_dict(load_config().train.runner)['algorithm']['gamma']==before


def test_time_cost_stronger_but_only_qualified_loaded_hold_exempts_it():
    r=ledger(3);q=np.tile(GOAL,(3,1))
    f=flags(3);f['confirmed_grasp'][:]=[False,True,True];f['hold_fraction'][:]=[0,0,.2]
    _,_,terms=r.step(q,GOAL,np.zeros((3,4)),**f)
    np.testing.assert_allclose(terms['time'],[-.03,-.03,0])
    f['wrong_fraction'][2]=.1
    _,_,terms=r.step(q,GOAL,np.zeros((3,4)),**f)
    assert terms['time'][2]==pytest.approx(-.03)


def test_workspace_cost_cannot_be_farmed_or_outweigh_immediate_fault():
    r=ledger();q=HOME[None];zero=np.zeros((1,4))
    for i in range(DEFAULTS['episode_steps']):
        r.step(q,GOAL,zero,workspace_fraction=np.ones(1),**flags(timeout=i==199))
    assert r.totals['workspace'][0]==pytest.approx(-4.)
    assert r.totals['time'][0]==pytest.approx(-6.,abs=2e-5)
    total=float(sum(v[0] for v in r.totals.values()))
    assert total > -DEFAULTS['penalty_fault']
    r.step(q,GOAL,zero,workspace_fraction=np.ones(1),**flags())
    assert r.totals['workspace'][0]==pytest.approx(-4.)


def test_blocked_requested_actions_are_charged_but_zero_hold_commands_are_free():
    torch=pytest.importorskip('torch')
    from types import SimpleNamespace
    from workbench.research.experiments.hug_c.environment import HugEnv
    env=object.__new__(HugEnv);env.s=DEFAULTS;env.device='cpu'
    command=torch.tensor([[0.,math.pi/2,math.pi/5,math.pi/5]]).repeat(2,1)
    env.mapper=SimpleNamespace(waypoint=command)
    env.goal=torch.tensor(GOAL,dtype=torch.float32)
    env.actual=lambda:command
    env.episode_length_buf=torch.zeros(2,dtype=torch.long);env.max_episode_length=200
    env.grasp_ticks=torch.zeros(2,dtype=torch.long);env.grasp_tick=torch.full((2,),-1)
    env.success=torch.zeros(2,dtype=torch.bool);env.fault=torch.zeros_like(env.success)
    env.last_floor=torch.ones_like(env.success)
    env.macro_wrong=torch.zeros(2);env.macro_drops=torch.zeros(2)
    env.workspace_exposure=torch.zeros(2)
    env.hold=SimpleNamespace(count=torch.zeros(2,dtype=torch.long),required_ticks=500,
                             fraction=lambda:torch.ones(2))
    env.object_clearance=lambda:torch.zeros(2)
    env.ledger=RewardLedger(2,DEFAULTS,torch,'cpu')
    env.ledger.reset(torch.arange(2),command,env.goal)
    _,_,terms=env.finish_action(command.clone(),torch.tensor([[1.,1,1,1],[0.,0,0,0]]))
    np.testing.assert_allclose(terms['action'].numpy(),[-.004,0.],atol=1e-7)
    assert env.effort_basis=='requested-policy-action'
    _,_,terms=env.finish_action(command.clone())
    np.testing.assert_allclose(terms['action'].numpy(),[0.,0.],atol=1e-7)
    assert env.effort_basis=='applied-command lower bound'
