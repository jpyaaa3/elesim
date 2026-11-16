"""A local route must certify its edges, remain bounded, and honor cancellation."""
import numpy as np
import pytest
from elesim_pilot.robot.arm.joint_path import connect_joint_path


def test_certified_history_shortening_removes_loops_without_crossing_an_obstacle():
    from elesim_pilot.robot.arm.joint_path import shorten_joint_path
    start,goal,_,_,certify,checked=problem()
    left=np.array([-.8,.8,0.,0.]);right=np.array([.8,.8,0.,0.])
    path=np.array([start,left,right,left,right,goal]);original=path.copy()
    assert all(certify(a,b)['accepted'] for a,b in zip(path,path[1:]))
    result,report=shorten_joint_path(path,certify)
    assert len(result)<len(path) and report['edge_checks']>0
    np.testing.assert_array_equal(result[[0,-1]],[start,goal])
    np.testing.assert_array_equal(path,original)
    assert all(certify(a,b)['accepted'] for a,b in zip(result,result[1:]))


def test_history_shortening_budget_retains_the_certified_remainder():
    from elesim_pilot.robot.arm.joint_path import shorten_joint_path
    path=np.array([[i,0.,0.,0.] for i in range(6)]);checked=[]
    def certify(a,b):
        checked.append((a.copy(),b.copy()));return dict(accepted=False)
    result,report=shorten_joint_path(path,certify,max_checks=1)
    np.testing.assert_array_equal(result,path)
    assert len(checked)==report['edge_checks']==1 and report['budget_exhausted']


def test_history_shortening_cancellation_never_calls_the_checker():
    from elesim_pilot.robot.arm.joint_path import shorten_joint_path
    checked=[]
    with pytest.raises(RuntimeError,match='cancelled'):
        shorten_joint_path(np.zeros((3,4)),lambda *a:checked.append(a),cancelled=lambda:True)
    assert not checked


def test_history_shortening_deadline_retains_the_original_safe_path():
    from elesim_pilot.robot.arm.joint_path import shorten_joint_path
    ticks=iter([0.,.01,.04,.05]);checked=[];path=np.zeros((5,4))
    def certify(*args):checked.append(args);return dict(accepted=False)
    result,report=shorten_joint_path(path,certify,max_seconds=.03,clock=lambda:next(ticks))
    np.testing.assert_array_equal(result,path)
    assert len(checked)==1 and report['budget_exhausted']


@pytest.mark.parametrize('invalid',['path','seconds','checks'])
def test_invalid_history_shortening_fails_before_certification(invalid):
    from elesim_pilot.robot.arm.joint_path import shorten_joint_path
    path=np.zeros((3,4));kwargs={};checked=[]
    if invalid=='path':path[0,0]=np.nan
    if invalid=='seconds':kwargs['max_seconds']=np.inf
    if invalid=='checks':kwargs['max_checks']=True
    with pytest.raises(ValueError):shorten_joint_path(path,lambda *a:checked.append(a),**kwargs)
    assert not checked


def problem():
    start=np.array([-.8,0.,0.,0.]);goal=-start
    samples=np.array([[x,y,0.,0.] for x in np.linspace(-.9,.9,15) for y in [-.7,.7]])
    bounds=np.array([[-1.,1.]]*4);checked=[]
    def certify(a,b):
        checked.append((a.copy(),b.copy()))
        delta=b[:2]-a[:2];length=delta@delta
        t=np.clip(-(a[:2]@delta)/length,0.,1.) if length else 0.
        return dict(accepted=np.linalg.norm(a[:2]+t*delta)>.35)
    return start,goal,samples,bounds,certify,checked


def test_curved_route_uses_only_continuously_certified_edges():
    start,goal,samples,bounds,certify,checked=problem()
    assert not certify(start,goal)['accepted']
    path,report=connect_joint_path(start,[goal],bounds,certify,samples=samples,max_seconds=2.)
    assert path is not None and report['status']=='found' and report['goal_index']==0
    np.testing.assert_array_equal(path[0],start);np.testing.assert_array_equal(path[-1],goal)
    assert np.all(path>=bounds[:,0]) and np.all(path<=bounds[:,1])
    assert all(any(np.array_equal(a,x) and np.array_equal(b,y) for x,y in checked)
        for a,b in zip(path,path[1:]))
    assert all(certify(a,b)['accepted'] for a,b in zip(path,path[1:]))


def test_multi_goal_forest_does_not_accept_an_invalid_goal_or_seed():
    start,goal,samples,bounds,certify,checked=problem()
    path,report=connect_joint_path(start,[np.zeros(4),goal],bounds,certify,
        samples=samples,seeds=[np.zeros(4)],max_seconds=2.)
    assert path is not None and report['goal_index']==1
    np.testing.assert_array_equal(path[-1],goal)
    assert all(np.linalg.norm(q[:2])>.35 for q in path)


def test_certified_seed_connects_a_blocked_direct_route_within_three_checks():
    start,goal,samples,bounds,certify,checked=problem()
    via=np.array([0.,.8,0.,0.])
    path,report=connect_joint_path(start,[goal],bounds,certify,samples=samples,
        seeds=[via],max_checks=3)
    assert path is not None and report['status']=='found'
    np.testing.assert_array_equal(path,[start,via,goal])
    assert report['edge_checks']==len(checked)==3
    assert all(any(np.array_equal(a,x) and np.array_equal(b,y) for x,y in checked)
        for a,b in zip(path,path[1:]))


def test_exhausted_budget_never_returns_an_uncertified_partial_route():
    start,goal,samples,bounds,certify,checked=problem()
    path,report=connect_joint_path(start,[goal],bounds,certify,samples=samples,max_checks=1)
    assert path is None and report['edge_checks']==len(checked)==1


def test_already_at_goal_keeps_an_executable_zero_motion_segment():
    start,_,samples,bounds,certify,_=problem()
    path,report=connect_joint_path(start,[start],bounds,certify,samples=samples)
    np.testing.assert_array_equal(path,[start,start])
    assert report['status']=='found'


def test_cancellation_does_not_call_the_collision_checker():
    start,goal,samples,bounds,certify,checked=problem()
    with pytest.raises(RuntimeError,match='cancelled'):
        connect_joint_path(start,[goal],bounds,certify,samples=samples,cancelled=lambda:True)
    assert not checked


@pytest.mark.parametrize('invalid',['start','goal','sample','bounds','budget'])
def test_invalid_or_unbounded_queries_fail_before_search(invalid):
    start,goal,samples,bounds,certify,checked=problem();kwargs={}
    if invalid=='start':start[0]=np.nan
    if invalid=='goal':goal[0]=2.
    if invalid=='sample':samples[0,0]=2.
    if invalid=='bounds':bounds[0]=[1.,-1.]
    if invalid=='budget':kwargs['max_seconds']=float('inf')
    with pytest.raises(ValueError):
        connect_joint_path(start,[goal],bounds,certify,samples=samples,**kwargs)
    assert not checked
