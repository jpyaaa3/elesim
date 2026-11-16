"""A continuous camera goal must fit its frustum and the original clearances."""
import numpy as np
import pytest

from elesim_pilot.robot.arm.view_refinement import refine_view,frustum_clearances


def problem():
    start=np.array([-.5,0.,0.,0.]);bounds=np.array([[-.5,.5],[-.1,.1],[-.1,.1],[-.1,.1]])
    points=np.array([[.4,-.04,1.],[.4,.04,1.]])
    intrinsics=np.array([500.,500.,320.,240.,640.,480.]);calls=[]
    def camera(q):
        pose=np.eye(4);pose[:3,3]=q[:3];return pose
    def margins(q):
        calls.append(q.copy());return np.array([q[0]+.25])
    return start,bounds,points,intrinsics,camera,margins,calls


def test_refinement_finds_a_view_inside_both_frustum_and_clearance_constraints():
    start,bounds,points,intr,camera,margins,calls=problem()
    assert np.min(frustum_clearances(camera(start),points,intr))<0
    q,report=refine_view(start,points,bounds,camera,margins,intr,
        motion_rates=np.ones(4),max_seconds=2.)
    assert q is not None and report['status']=='found'
    assert np.all(q>=bounds[:,0]) and np.all(q<=bounds[:,1])
    assert np.all(margins(q)>0)
    assert np.min(frustum_clearances(camera(q),points,intr))>=-1e-8
    assert calls and report['evaluations']<=1024


@pytest.mark.parametrize('clearance',[-1.,0.])
def test_infeasible_or_zero_clearance_never_produces_a_goal(clearance):
    start,bounds,points,intr,camera,_,calls=problem()
    q,report=refine_view(start,points,bounds,camera,lambda q:np.array([clearance]),intr,
        motion_rates=np.ones(4),max_seconds=2.,max_evaluations=64)
    assert q is None and report['status']!='found'


def test_exhausted_budget_never_returns_an_unchecked_partial_goal():
    start,bounds,points,intr,camera,margins,calls=problem()
    q,report=refine_view(start,points,bounds,camera,margins,intr,
        motion_rates=np.ones(4),max_evaluations=1)
    assert q is None and report['evaluations']==len(calls)==1


def test_cancelled_refinement_does_not_read_camera_or_collision_models():
    start,bounds,points,intr,camera,margins,calls=problem()
    with pytest.raises(RuntimeError,match='cancelled'):
        refine_view(start,points,bounds,camera,margins,intr,
            motion_rates=np.ones(4),cancelled=lambda:True)
    assert not calls


@pytest.mark.parametrize('invalid',['points','bounds','intrinsics','rates','budget'])
def test_invalid_refinement_inputs_fail_before_evaluating_geometry(invalid):
    start,bounds,points,intr,camera,margins,calls=problem();rates=np.ones(4);kwargs={}
    if invalid=='points':points[0,0]=np.nan
    if invalid=='bounds':bounds[0]=[1.,-1.]
    if invalid=='intrinsics':intr[0]=0.
    if invalid=='rates':rates[0]=0.
    if invalid=='budget':kwargs['max_seconds']=float('inf')
    with pytest.raises(ValueError):
        refine_view(start,points,bounds,camera,margins,intr,motion_rates=rates,**kwargs)
    assert not calls
