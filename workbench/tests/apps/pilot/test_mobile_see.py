"""Coupled SEE placement, swept motion and measured-control regressions."""
import threading
import time
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from elesim_pilot.mobile_manipulation.base_planner import body_frame,FloorObservationRequired
from elesim_pilot.mobile_manipulation.contracts import BaseGoal,Settings
from elesim_pilot.mobile_manipulation.local_map import LocalMap,rectangular_footprint
from elesim_pilot.mobile_manipulation.ownership import Ownership
from elesim_pilot.mobile_manipulation.scan_motion import ScanMotion
from elesim_pilot.mobile_manipulation.see_planner import MobileSeePlanner,ViewMotion
from elesim_pilot.reconstruction.scan_collision import ScanCollision


def ports():
    bounds=np.array([[-.005,.005],[-.6,.6],[0.,0.],[0.,0.]])
    def links(q):
        pose=np.eye(4);pose[:3,3]=[q[0],0.,.15]
        return {'tip':pose}
    arm=NS(names=['tip'],local={'tip':np.zeros((1,3))},radii={'tip':np.array([.01])},
        bounds=bounds,uncertainty=np.array([.001,.015,.015,.015]),
        joints=[dict(index=0,multiplier=1.,kind='prismatic',t=np.eye(4))],ancestry={'tip':[0]},
        link_poses=links,source_hashes={'arm_model.json':'test-model'},
        configuration_free=lambda *a,**kw:True,certify_segment=lambda *a,**kw:dict(accepted=True))
    collision=ScanCollision(arm,np.eye(4),reserve_tracking=True)
    grid=LocalMap(.05)
    grid.free.update((x,y) for x in range(-30,31) for y in range(-30,31))
    snapshot=dict(joint=dict(q=np.zeros(4),sim_time=1.,mono=1.,stamp=100.),pose=np.zeros(3),
        base=dict(pos=np.zeros(3),rpy=np.zeros(3),velocity=np.zeros(3),angular=np.zeros(3),locomotion_stage='stand'))
    p=NS(arm=arm,collision=collision,grid=grid,s=Settings(),arm_in_body=np.eye(4),
        model=NS(meta={'home':np.zeros(4)}),feedback=NS(snapshot=lambda _:snapshot),
        route=lambda:('sim','boot','lease'),_measured_body=lambda snap:body_frame(snap['pose'],snap['base']['pos'][2],snap['base']['rpy'][:2]),
        footprint=rectangular_footprint([[-.38,.38],[-.25,.25]],spacing=.025),
        service=NS(_go2_arm_mount=NS(arm_base_world_transform=lambda pos,rpy:body_frame(np.r_[pos[:2],rpy[2]],pos[2],rpy[:2]))))
    return p,snapshot


class Camera:
    model_digest='test-model'
    lower=np.array([-.005,-.6,0.,0.])
    upper=np.array([.005,.6,0.,0.])
    def __init__(self):self.context={'base_world_T':np.eye(4)}
    def camera_pose(self,q):
        pose=np.eye(4);pose[:3,3]=[q[0],0.,0.]
        pose[:3,:3]=Rotation.from_euler('y',q[1]).as_matrix()
        return self.context['base_world_T']@pose


def planner(p):
    def unexpected(*a,**kw):raise AssertionError('fully observed route requires no floor recovery')
    mobility=NS(attempted=[],observe_floor=unexpected)
    frontier=np.array([0.,0.,.4]);desired=np.array([-.14,0.,0.])
    direction=frontier-desired;direction/=np.linalg.norm(direction)
    backend=NS(propose=lambda *a,**kw:np.array([np.r_[desired,direction,frontier]]))
    result=MobileSeePlanner(Camera(),p,mobility,backend=backend)
    points=np.array([[x,y,.4] for x in [-.02,.02] for y in [-.02,.02]])
    result.set_observations([(points,np.eye(4))],NS(fx=100.,fy=100.,cx=60.,cy=30.,width=120,height=60))
    return result,points


def test_frontier_can_require_base_and_arm_motion_from_the_same_objective():
    p,snapshot=ports();view,points=planner(p)
    choice=view.choose(points,np.zeros(4),[np.eye(4)],[])
    assert choice is not None,view.last_search
    assert np.linalg.norm(choice['base_pose'][:2])>.04
    assert abs(choice['q'][1])>.05
    assert choice['visible_frontiers']==1 and choice['policy']=='see'
    assert choice['motion'].certificate['environment_scope']=='observed_swept_floor'
    assert len(choice['motion'].base.path)>1
    np.testing.assert_array_equal(snapshot['pose'],np.zeros(3))


def test_coupled_ik_supports_all_arm_axes_held_at_their_operating_bounds():
    p,_=ports();view,points=planner(p)
    view.operating_bounds[:]=0.;view.q[:]=0.
    view.local_poses[:]=np.eye(4);view.poses[:]=np.eye(4)
    choice=view.choose(points,np.zeros(4),[np.eye(4)],[])
    assert choice is not None,view.last_search
    np.testing.assert_array_equal(choice['q'],np.zeros(4))
    assert np.linalg.norm(choice['base_pose'][:2])>.04


def test_unknown_floor_requests_measurement_and_blocked_floor_rejects_motion():
    p,snapshot=ports();view,_=planner(p)
    goal=np.array([.1,0.,0.])
    p.grid.free.clear()
    with pytest.raises(FloorObservationRequired) as request:
        view.motion(goal,np.zeros(4),snapshot,cancelled=lambda:False)
    assert request.value.footprint is not None and request.value.margin_m>0
    p.grid.blocked.add(p.grid.cell([0.,0.]))
    assert view.motion(goal,np.zeros(4),snapshot,cancelled=lambda:False) is None


def test_swept_arm_envelope_includes_shapes_between_the_endpoints():
    p,snapshot=ports();view,_=planner(p)
    p.arm.bounds[0]=[-1.,1.]
    def links(q):
        pose=np.eye(4);pose[:3,3]=[.7+q[0],.55,.3]
        return {'tip':pose}
    p.arm.link_poses=links
    snapshot['joint']['q'][0]=-.2
    p.grid.free.remove(p.grid.cell([.7,.55]))
    for linear in [-.2,.2]:
        assert p.grid.footprint_free([0.,0.,0.],[[.7+linear,.55]])
    with pytest.raises(FloorObservationRequired):
        view.motion(np.array([.05,0.,0.]),np.array([.2,0.,0.,0.]),snapshot,cancelled=lambda:False)


def test_mobile_sweep_uses_the_shipped_multi_link_arm_geometry():
    from elesim_pilot.robot.arm.self_collision import ArmSelfCollision
    root=Path(__file__).resolve().parents[4]/'payload/data'
    p,snapshot=ports()
    p.arm=ArmSelfCollision(root/'models/planning/zed-mini',joint_uncertainty=[.001,.015,.015,.015])
    p.collision=ScanCollision(p.arm,np.eye(4),reserve_tracking=True)
    home=np.array(json.loads((root/'policies/hug-d100/manifest.json').read_text())['home'])
    p.model.meta['home']=home.copy();snapshot['joint']['q']=np.array([-.12,0.,-.5,.5])
    view=MobileSeePlanner.__new__(MobileSeePlanner)
    view.ports=p;view.collision_checker=p.collision;view.return_q=home.copy()
    target=snapshot['joint']['q']+np.array([0.,.02,.005,-.005])
    plan=view.motion(np.array([.03,0.,0.]),target,snapshot,cancelled=lambda:False)
    assert plan is not None
    assert plan.certificate['swept_joint_samples']>=2 and plan.reach_m>0
    assert p.grid.segment_free(plan.base.path[0],plan.base.path[-1],plan.footprint)


def test_first_capture_accepts_the_existing_home_with_its_normal_certificate():
    from elesim_pilot.robot.arm.self_collision import ArmSelfCollision
    root=Path(__file__).resolve().parents[4]/'payload/data'
    p,snapshot=ports();p.clock=lambda:1.
    p.arm=ArmSelfCollision(root/'models/planning/zed-mini',joint_uncertainty=[.001,.015,.015,.015])
    p.collision=ScanCollision(p.arm,np.eye(4),reserve_tracking=True)
    home=np.array(json.loads((root/'policies/hug-d100/manifest.json').read_text())['home'])
    snapshot['joint']['q']=home.copy()
    assert p.arm.configuration_free(home)
    assert not p.collision.configuration_free(home)
    motion=ScanMotion(p,dict(grant=None,ownership=None));plans=[]
    motion.planner=NS(update_base=lambda snapshot:None)
    def wait(plan,cancelled):
        plans.append(plan);return dict(wall=100.)
    motion._wait=wait
    assert motion.settle(home,cancelled=lambda:False)==100.
    assert len(plans)==1 and plans[0].certificate['accepted']
    np.testing.assert_array_equal(plans[0].start_q,home)
    np.testing.assert_array_equal(plans[0].base.arm_q,home)


def test_mobile_observation_snapshot_uses_each_new_measured_base():
    p,snapshot=ports();view,_=planner(p)
    before=view.model.camera_pose(np.zeros(4))
    snapshot['base']['pos'][:2]=[.12,-.08]
    snapshot['base']['rpy'][2]=.3
    snapshot['pose'][:]=[.12,-.08,.3]
    view.update_base(snapshot)
    after=view.model.camera_pose(np.zeros(4))
    np.testing.assert_allclose(after,body_frame(snapshot['pose'],0.))
    assert not np.allclose(before,after)
    np.testing.assert_allclose(view.poses,after[None]@view.local_poses)


def test_scene_mapping_uses_captured_body_pose_after_feedback_history_expires():
    p,_=ports();p.clock=lambda:1.;mapped=[]
    p.arm_in_body=np.eye(4);p.arm_in_body[:3,3]=[.2,0.,.3]
    captured=body_frame([.6,-.2,.3],.32,[.02,-.03])
    measurement=NS(base_world_transform=captured@p.arm_in_body)
    def stale(*args):raise AssertionError('capture body already came from source-fenced feedback')
    p.feedback.associate=stale
    p._map_scene=lambda m,pose,base,body:mapped.append(body)
    ScanMotion(p,dict(grant=None,ownership=None)).observe_scene(measurement,np.eye(4))
    np.testing.assert_allclose(mapped[0]['pos'],[.6,-.2,.32])
    np.testing.assert_allclose(mapped[0]['rpy'],[.02,-.03,.3])


def test_floor_recovery_replans_with_new_joints_and_retains_frontiers():
    p,snapshot=ports();view,points=planner(p)
    calls=[];frontiers=[]
    request=FloorObservationRequired(np.zeros(3),footprint=p.footprint)
    def choose(proposals,observed,snap,accepted,*,cancelled):
        frontiers.append(proposals)
        if not calls:return None,request
        assert snap['joint']['q'][1]==.1
        return {'q':snap['joint']['q']},None
    def recover(missing,*,cancelled):
        assert missing is request
        calls.append(missing);snapshot['joint']['q'][1]=.1
    view._choose_mobile=choose;view.mobility.observe_floor=recover
    result=view.choose(points,np.zeros(4),[np.eye(4)],[])
    assert result['q'][1]==.1 and len(frontiers)==2
    assert frontiers[0] is frontiers[1]


def test_mobile_motion_sends_both_actuator_groups_only_from_control_ticks():
    p,snapshot=ports();now=[1.];sent=[]
    p.clock=lambda:now[0];p._summary={};p.stopped=False
    ownership=Ownership();ownership.acquire(p.route());grant=ownership.grant('scan',{'arm','base'})
    def send(g,o,**values):
        o.require(g,p.route(),'arm');o.require(g,p.route(),'base')
        sent.append((threading.get_ident(),values))
    p._send=send
    motion=ScanMotion(p,dict(grant=grant,ownership=ownership))
    q=np.array([0.,.12,0.,0.]);base_pose=np.array([.08,0.,0.])
    goal=BaseGoal(base_pose,np.array([np.zeros(3),base_pose]),q,dict(kind='observation'))
    plan=ViewMotion(goal,np.zeros(4),p.footprint,dict(accepted=True),np.eye(4),3.,.6)
    result=[];failure=[]
    def wait():
        try:result.append(motion._wait(plan,lambda:False))
        except BaseException as exc:failure.append(exc)
    worker=threading.Thread(target=wait);worker.start()
    deadline=time.monotonic()+2.
    while motion.request is None and time.monotonic()<deadline:time.sleep(.001)
    assert motion.request is not None
    tick_thread=threading.get_ident()
    for _ in range(220):
        now[0]+=.05
        snapshot['joint'].update(mono=now[0],sim_time=now[0],stamp=100.+now[0])
        motion.tick()
        if sent:
            snapshot['joint']['q']=np.array(sent[-1][1]['q'])
            velocity=np.array(sent[-1][1]['go2_vel'])
            snapshot['pose'][:2]+=velocity[:2]*.05
            snapshot['base']['pos'][:2]=snapshot['pose'][:2]
            snapshot['base']['velocity']=velocity.copy()
        if result or failure:break
        time.sleep(.0002)
    if not result and not failure:motion.cancel('test timed out')
    worker.join(2.)
    assert not failure,failure
    assert result and not worker.is_alive()
    assert {thread for thread,_ in sent}=={tick_thread}
    assert any(row['go2_vel'][0]>0 and row['q'][1]>0 for _,row in sent)
    assert sent[-1][1]['go2_vel']==[0.,0.,0.]


def test_cancel_wakes_pending_mobile_motion_without_future_commands():
    p,snapshot=ports();p.clock=lambda:1.;p._summary={};p.stopped=False
    sent=[];p._send=lambda *a,**kw:sent.append(kw)
    motion=ScanMotion(p,dict(grant=None,ownership=None))
    goal=BaseGoal(np.zeros(3),np.zeros((1,3)),np.zeros(4),{})
    plan=ViewMotion(goal,np.zeros(4),p.footprint,dict(accepted=True),np.eye(4),0.)
    failure=[]
    def wait():
        try:motion._wait(plan,lambda:False)
        except InterruptedError as exc:failure.append(str(exc))
    worker=threading.Thread(target=wait);worker.start()
    deadline=time.monotonic()+2.
    while motion.request is None and time.monotonic()<deadline:time.sleep(.001)
    motion.cancel('operator cancelled');worker.join(2.)
    assert failure==['operator cancelled'] and not worker.is_alive()
    assert sent==[dict(go2_vel=[0.,0.,0.])]
    with pytest.raises(InterruptedError,match='operator cancelled'):motion.tick()
    assert len(sent)==1


def test_floor_recovery_cannot_bypass_the_configured_arm_limits():
    p,snapshot=ports();p.clock=lambda:1.
    motion=ScanMotion(p,dict(grant=None,ownership=None))
    goal=BaseGoal(np.zeros(3),np.zeros((1,3)),np.array([-.1,0.,0.,0.]),{})
    plan=ViewMotion(goal,np.zeros(4),p.footprint,dict(accepted=True),np.eye(4),0.)
    with pytest.raises(ValueError,match='operating bounds'):
        motion._wait(plan,lambda:False)
    assert motion.request is None


def test_active_scan_and_grasp_mapping_share_one_floor_observation_budget():
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    p=SimPorts.__new__(SimPorts);p.s=Settings(search_limit=2);p.floor_attempts=0;p._summary={}
    assert p.reserve_floor_view()==1
    assert p.reserve_floor_view()==2
    with pytest.raises(RuntimeError,match='floor observation budget exhausted'):
        p.reserve_floor_view()
    assert p.floor_attempts==2 and p._summary['mapping_attempts']==2


def test_normal_mobile_scan_entry_attaches_coupled_motion_to_live_scan():
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    stub,_=ports();p=SimPorts.__new__(SimPorts);p.__dict__.update(vars(stub))
    p.generation=0;p.clock=lambda:1.;p.target_id='target';captured=[]
    def start(target,max_views,**options):
        captured.append((target,max_views,options));return dict(job_id='job')
    p.scan=NS(start=start)
    op=p.begin('scan',None,None)
    assert op['scan_job_id']=='job' and len(captured)==1
    target,max_views,options=captured[0]
    assert target=='target' and max_views==12
    assert options['view_policy']=='see' and isinstance(options['mobility'],ScanMotion)
    assert options['mobility'].op is op and options['on_observation'] is not None


def test_see_parent_grants_the_same_scan_both_motion_channels():
    from elesim_pilot.mobile_manipulation.coordinator import Coordinator
    from .test_mobile_manipulation import Ports
    p=Ports();original=p.begin;grants=[]
    def begin(phase,grant,ownership,**kwargs):
        if phase=='scan':
            ownership.require(grant,p.route(),'arm')
            ownership.require(grant,p.route(),'base')
            grants.append(grant)
        return original(phase,grant,ownership,**kwargs)
    p.begin=begin
    coordinator=Coordinator(p,clock=lambda:0.)
    coordinator.start(coordinator.plan()['plan_id']);coordinator.tick()
    assert coordinator.phase=='scan' and len(grants)==1
    assert p.calls==['acquire','scan']


@pytest.mark.parametrize('drift,allowed',[(.00001,True),(.003,False)])
def test_fixed_base_view_does_not_turn_pose_jitter_into_a_walking_request(drift,allowed):
    p,snapshot=ports();p.clock=lambda:1.
    motion=ScanMotion(p,dict(grant=None,ownership=None))
    view,_=planner(p);motion.planner=view
    initial=view.motion(snapshot['pose'].copy(),np.zeros(4),snapshot,cancelled=lambda:False)
    choice=dict(base_pose=snapshot['pose'].copy(),q=np.zeros(4),motion=initial)
    p.grid.free.clear()  # A fixed-base arm view does not authorize any walk.
    snapshot['pose'][0]=drift;snapshot['base']['pos'][0]=drift
    recorded=[]
    def finish(plan,cancelled):
        recorded.append(plan)
        return dict(q=plan.base.arm_q.copy())
    motion._wait=finish
    if not allowed:
        with pytest.raises(RuntimeError,match='arm base moved'):
            motion.relocate(choice,cancelled=lambda:False)
        assert not recorded
        return
    motion.relocate(choice,cancelled=lambda:False)
    assert len(recorded[0].base.path)==1
    np.testing.assert_array_equal(recorded[0].base.pose,snapshot['pose'])


def test_coupled_view_keeps_its_planned_base_goal_during_recertification():
    p,snapshot=ports();p.clock=lambda:1.
    motion=ScanMotion(p,dict(grant=None,ownership=None));view,_=planner(p);motion.planner=view
    target=np.array([.03,0.,0.]);q=np.zeros(4)
    initial=view.motion(target,q,snapshot,cancelled=lambda:False)
    choice=dict(base_pose=target.copy(),q=q,motion=initial)
    snapshot['pose'][0]=.00001;snapshot['base']['pos'][0]=.00001
    recorded=[]
    def finish(plan,cancelled):
        recorded.append(plan)
        return dict(q=plan.base.arm_q.copy())
    motion._wait=finish
    motion.relocate(choice,cancelled=lambda:False)
    assert len(recorded[0].base.path)>1
    np.testing.assert_array_equal(recorded[0].base.pose,target)


def test_see_uses_intermediate_coupled_pose_when_frontier_endpoint_is_out_of_frame(monkeypatch):
    p,snapshot=ports();view,points=planner(p)
    # The endpoint sought by SEE is around x=-.14; the object fits only in
    # this narrower field of view, reached on the way to that endpoint.
    monkeypatch.setattr(view,'_contains_extent',lambda observed,pose:-.11<pose[0,3]<-.05)
    monkeypatch.setattr(view,'_information_view',lambda *a,**kw:None)
    choice=view.choose(points,snapshot['joint']['q'],[np.eye(4)],[])
    assert choice is not None
    assert choice['policy']=='see' and choice['visible_frontiers']>0
    assert -.11<choice['pose'][0,3]<-.05
    assert len(choice['motion'].base.path)>1
    assert choice['motion'].certificate['environment_scope']=='observed_swept_floor'
    assert p.grid.segment_free(choice['motion'].base.path[0],choice['motion'].base.path[-1],
        choice['motion'].footprint)


def test_certification_budget_keeps_stationary_option_among_many_higher_ranked_moves():
    from elesim_pilot.mobile_manipulation.see_planner import _certification_candidates
    def candidate(score,x):return (score,np.array([x,0.,0.]),None,None,None,None)
    candidates=[candidate(100-i,.1+i*.001) for i in range(20)]+[candidate(1.,0.)]
    selected=_certification_candidates(candidates,np.zeros(3))
    assert len(selected)==16
    assert any(np.allclose(item[1],0.) for item in selected)
    assert any(not np.allclose(item[1],0.) for item in selected)
