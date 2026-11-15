"""Mobile manipulation geometry, fault fences and complete parent transitions."""
from types import SimpleNamespace as NS
from pathlib import Path
import numpy as np
import pytest
from elesim_pilot.mobile_manipulation.contracts import Settings,TargetEstimate,BaseGoal
from elesim_pilot.mobile_manipulation.feedback import Feedback
from elesim_pilot.mobile_manipulation.target_track import TargetTrack
from elesim_pilot.mobile_manipulation.local_map import LocalMap,rectangular_footprint
from elesim_pilot.mobile_manipulation.base_executor import BaseExecutor
from elesim_pilot.mobile_manipulation.base_planner import solver_xy,node_frame
from elesim_pilot.mobile_manipulation.coordinator import Coordinator
from elesim_pilot.mobile_manipulation.ownership import Ownership


def cylinder(center=(.4,.1),r=.05):
    return np.array([[center[0]+r*np.cos(a),center[1]+r*np.sin(a),z]
        for z in np.linspace(.45,.65,12) for a in np.linspace(-np.pi/2,np.pi/2,60)])


def test_operator_calibrated_home_is_admitted_without_redefining_the_policy_start():
    from elesim_protocol import SimMappingConfig
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    ports=SimPorts.__new__(SimPorts)
    home=np.array([-.12,0.,-np.deg2rad(170)/5,np.deg2rad(170)/5])
    ports.model=NS(meta=dict(home=home.copy()))
    ports.service=NS(control_mapping=lambda:SimMappingConfig(command_direction=(1,1,1,-1)))
    ports.state=NS(offset_values=lambda:(0.,-9.,0.,0.,1))
    ports._require_home_start(home)
    calibrated=home.copy();calibrated[1]=-np.deg2rad(4.5)
    ports._require_home_start(calibrated)
    with pytest.raises(RuntimeError,match='Home'):
        ports._require_home_start(np.zeros(4))
    np.testing.assert_array_equal(ports.model.meta['home'],home)


@pytest.mark.parametrize('cause', ['empty', 'invalid_sample', 'not_started'])
def test_mobile_capture_retries_only_a_missing_sample(cause):
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    from elesim_pilot.vision.perception.sim_rendered_camera import SimCameraTimeout
    from elesim_protocol import DdsTransportError
    failures = dict(empty=SimCameraTimeout('no DDS RGB-D sample received'),
        invalid_sample=DdsTransportError('invalid encoded RGB-D sample: damaged metadata'),
        not_started=RuntimeError('SimRenderedCamera not started'))
    ports=SimPorts.__new__(SimPorts);ports._initialize=lambda:None
    now=[0.];attempts=[]
    def capture(**kwargs):
        attempts.append(1);now[0]+=.1
        raise failures[cause]
    ports.camera=NS(capture=capture);ports.clock=lambda:now[0]
    ports.stopped=False;ports._summary={}
    if cause=='empty':
        with pytest.raises(RuntimeError,match='no fresh stationary RGB-D capture'):
            ports._capture()
        assert len(attempts)>=49
        assert ports.summary()['capture_rejections']==dict(no_frame=len(attempts))
    else:
        with pytest.raises(type(failures[cause])) as caught:
            ports._capture()
        assert caught.value is failures[cause]
        assert len(attempts)==1 and 'capture_rejections' not in ports.summary()


def test_partial_observation_retains_axis_without_claiming_full_height():
    t=TargetTrack();e=t.update(cylinder(),stamp=1.,source=('sim','boot','calib'))
    assert np.allclose(e.at_height(.55),[.4,.1,.55],atol=1e-6)
    assert e.diameter==pytest.approx(.1,abs=1e-6)
    with pytest.raises(RuntimeError,match='identity'):
        t.update(cylinder((.8,.1)),stamp=2.,source=e.source)
    with pytest.raises(RuntimeError,match='replayed'):
        t.update(cylinder(),stamp=1.,source=e.source)


def test_more_than_half_circle_coverage_cannot_worsen_position_allowance():
    estimates=[]
    for arc in (np.pi,2*np.pi):
        cloud=np.array([[.4+.05*np.cos(a),.1+.05*np.sin(a),z]
            for z in np.linspace(.45,.65,5) for a in np.linspace(-arc/2,arc/2,100)])
        estimates.append(TargetTrack().update(cloud,stamp=1.,source=('sim','boot','calib')))
    assert estimates[1].error_m<=estimates[0].error_m+1e-10
    assert estimates[1].error_m==pytest.approx(.003)


def test_base_move_reexpresses_target_instead_of_moving_world_object():
    e=TargetTrack().update(cylinder(),stamp=1.,source=('sim','boot','calib'))
    node=np.eye(4);node[2,3]=.55
    assert np.allclose(solver_xy(e,node),[-100,544.5])
    moved=node_frame([.1,0,np.pi/2],node,0)
    assert np.allclose(solver_xy(e,moved),[300,244.5])
    assert np.allclose(e.origin[:2],[.4,.1])


def test_split_feedback_and_capture_time_matching():
    now=[1.];f=Feedback(lambda:now[0]);source=('sim','boot')
    f.accept(dict(q=[0,0,0,0],q_source='simulated',measurement_time_s=100.,measurement_monotonic_s=1.),source)
    f.accept(dict(go2_base_pos=[0,0,.32],go2_base_rpy=[0,0,0],go2_base_timestamp_s=100.),source)
    assert f.snapshot(source)['joint']['stamp']==100.
    assert f.associate(NS(ts=100.),source)['pos'][2]==.32
    with pytest.raises(RuntimeError):f.associate(NS(ts=102.),source)
    now[0]=2.
    with pytest.raises(RuntimeError,match='stale'):f.snapshot(source)
    f.accept(dict(go2_base_pos=[0,0,.32],go2_base_rpy=[0,0,0],go2_base_timestamp_s=102.),('sim','newboot'))
    with pytest.raises(RuntimeError):f.snapshot(('sim','newboot'))


def test_split_half_frame_uses_the_last_coherent_pair_without_extending_freshness():
    now=[1.];feedback=Feedback(lambda:now[0]);source=('sim','boot')
    def arm(stamp):
        feedback.accept(dict(q=[stamp,0,0,0],q_source='simulated',
            measurement_time_s=stamp,measurement_monotonic_s=stamp),source)
    def body(stamp):
        feedback.accept(dict(go2_base_pos=[0,0,.3],go2_base_rpy=[0,0,0],
            go2_base_timestamp_s=stamp),source)
    arm(100.);body(100.)
    now[0]=1.3;arm(100.3)
    assert feedback.snapshot(source)['joint']['stamp']==100.
    now[0]=1.6;arm(100.6)
    with pytest.raises(RuntimeError,match='stale'):
        feedback.snapshot(source)
    body(100.6)
    assert feedback.snapshot(source)['joint']['stamp']==100.6


def test_unknown_space_and_swept_body_cannot_be_walked_through():
    m=LocalMap(.1);foot=rectangular_footprint([[-.1,.1],[-.1,.1]],spacing=.05,padding=0)
    with pytest.raises(ValueError):m.path([0,0,0],[1,0,0],foot)
    m.observe(np.array([[x,y,0.] for x in np.arange(-.5,1.6,.025) for y in np.arange(-.5,.6,.025)]))
    assert len(m.path([0,0,0],[1,0,0],foot))==2
    m.observe(np.array([[.5,y,.2] for y in np.arange(-.5,.6,.025)]))
    with pytest.raises(ValueError):m.path([0,0,0],[1,0,0],foot)


def test_footprint_diagnostics_distinguish_unknown_floor_from_obstacles():
    grid=LocalMap(.1);grid.free.add((0,0));grid.blocked.add((2,0))
    footprint=np.array([[.01,0],[.11,0],[.21,0]])
    status=grid.footprint_status([0,0,0],footprint)
    assert status['total_cells']==3 and status['unknown_cells']==status['blocked_cells']==1
    np.testing.assert_allclose(status['unknown_xy_m'],[[.15,.05]])
    np.testing.assert_allclose(status['blocked_xy_m'],[[.25,.05]])
    assert grid.free=={(0,0)} and not grid.footprint_free([0,0,0],footprint)


def test_floor_looks_choose_a_new_camera_view_after_an_unproductive_capture(monkeypatch,tmp_path):
    from scipy.stats import qmc
    from elesim_pilot.mobile_manipulation import runtime
    monkeypatch.setattr(qmc,'Halton',lambda *a,**k:NS(random=lambda n:np.array([[x,0,0,0] for x in [0,.05,.1]])))
    class CameraModel:
        def camera_pose(self,q):
            pose=np.diag([1.,-1.,-1.,1.]);pose[:3,3]=[.45+q[0],0,.5];return pose
    monkeypatch.setattr(runtime,'ArmCameraModel',lambda *a,**k:CameraModel())
    ports=runtime.SimPorts.__new__(runtime.SimPorts)
    ports.arm=NS(bounds=np.array([[0.,1.]]*4));ports.scan=NS(model_dir=tmp_path/'assembly',hand_eye=tmp_path/'eye.json')
    ports.service=NS(_ik_context_for_host=lambda h:dict(base_world_T=np.eye(4)))
    ports.connection=NS(state_sink=NS(get_state=lambda:None));ports.route=lambda:('sim','boot','lease')
    ports.feedback=NS(snapshot=lambda *a:dict(joint=dict(q=np.zeros(4)),pose=np.zeros(3)))
    ports.grid=LocalMap(.1);ports.mapping_views=[];ports._arm_prepare=lambda qs:[np.zeros(4),*qs]
    first=ports._map_search_prepare(1)[-1]
    second=ports._map_search_prepare(2)[-1]
    assert len(ports.mapping_views)==2 and abs(first[0]-second[0])>=.03
    assert not ports.grid.free and not ports.grid.blocked


def test_base_executor_requires_measured_settle_and_stops_on_blockage():
    now=[0.];s=Settings();e=BaseExecutor(s,lambda:now[0])
    goal=BaseGoal(np.array([.1,0,0]),np.array([[0,0,0],[.1,0,0]]),np.zeros(4),{})
    e.start(goal,[0,0,0],1)
    v,done=e.step([0,0,0],0,0,lambda *a:True);assert v[0]>0 and not done
    now[0]=1.;v,done=e.step([.1,0,0],.1,0,lambda *a:True);assert not done
    e.step([.1,0,0],0,0,lambda *a:True);now[0]=1.6
    assert e.step([.1,0,0],0,0,lambda *a:True)[1]
    e.start(goal,[0,0,0],1)
    with pytest.raises(RuntimeError):e.step([0,0,0],0,0,lambda *a:False)


def test_reached_observation_goal_holds_without_demanding_a_corrective_grid_sweep():
    now=[0.];executor=BaseExecutor(Settings(),lambda:now[0])
    goal=BaseGoal(np.zeros(3),np.array([[0.,0.,0.]]),np.zeros(4),dict(kind='observation'))
    executor.start(goal,[.0003,0,0],1)
    def blocked(*args):raise AssertionError('no movement requested')
    velocity,done=executor.step([.0003,0,0],0,0,blocked)
    assert not velocity.any() and not done
    now[0]=.6
    velocity,done=executor.step([.0003,0,0],0,0,blocked)
    assert not velocity.any() and done


class Ports:
    def __init__(self):self.calls=[];self.identity=('sim','boot','lease');self.scene=1;self.stops=[]
    def route(self):return self.identity
    def context(self):return self.scene
    def preflight(self,target):pass
    def guard(self):pass
    def summary(self):return {}
    def stop(self,reason):self.stops.append(reason)
    def begin(self,phase,grant,ownership,**args):
        self.calls.append(phase);ownership.require(grant,self.identity,'arm');return phase,args
    def poll(self,op):
        phase,args=op
        if phase in {'acquire','reacquire','post_scan_acquire'}:return dict(visible=True)
        if phase in {'plan_grasp','plan_observation'}:return dict(goal='goal')
        if phase=='transport':return dict(purpose=args['purpose'])
        if phase=='scan':return dict(quality_ok=True)
        if phase=='check_grasp':return dict(accepted=True)
        if phase=='verify':return dict(verified=False,reason='no independent hold observation')
        return {}


def test_parent_cycle_uses_observation_and_grasp_base_goals_and_is_truthful():
    p=Ports();c=Coordinator(p,clock=lambda:0.)
    c.start(c.plan()['plan_id'])
    for _ in range(20):c.tick()
    assert c.phase=='unverified' and not c.active
    assert p.calls==['acquire','plan_observation','transport','move_observation','aim_observation','scan','post_scan_acquire',
        'plan_grasp','transport','move_grasp','reacquire','check_grasp','hug','verify']
    assert c.ownership.session is None


def test_search_after_reconstruction_resumes_post_scan_acquisition_instead_of_restarting_scan():
    ports=Ports();poll=ports.poll;missed=[False]
    def miss_once(op):
        if op[0]=='post_scan_acquire' and not missed[0]:
            missed[0]=True;return dict(visible=False)
        return poll(op)
    ports.poll=miss_once;coordinator=Coordinator(ports,clock=lambda:0.)
    coordinator.start(coordinator.plan()['plan_id'])
    for _ in range(24):coordinator.tick()
    assert ports.calls.count('scan')==1
    index=ports.calls.index('search')
    assert ports.calls[index+1]=='post_scan_acquire'
    assert 'plan_grasp' in ports.calls and coordinator.phase=='unverified'


def test_unobserved_grasp_path_collects_floor_and_resumes_without_repeating_scan():
    ports=Ports();poll=ports.poll;missing=[True]
    def ask_for_floor(op):
        if op[0]=='plan_grasp' and missing[0]:
            missing[0]=False;return dict(needs_mapping=True)
        return poll(op)
    ports.poll=ask_for_floor;coordinator=Coordinator(ports,clock=lambda:0.)
    coordinator.start(coordinator.plan()['plan_id'])
    for _ in range(24):coordinator.tick()
    index=ports.calls.index('map_search')
    assert ports.calls[index+1]=='post_scan_acquire'
    assert ports.calls.count('scan')==1 and ports.calls.count('plan_grasp')==2
    assert coordinator.phase=='unverified'


def test_floor_observation_budget_does_not_consume_target_recovery_budget():
    ports=Ports();poll=ports.poll;maps=[0];hidden=[False]
    def acquire(op):
        phase,_=op
        if phase=='plan_grasp' and maps[0]<2:
            maps[0]+=1;return dict(needs_mapping=True)
        if phase=='post_scan_acquire' and maps[0]==2 and not hidden[0]:
            hidden[0]=True;return dict(visible=False)
        return poll(op)
    ports.poll=acquire
    coordinator=Coordinator(ports,settings=Settings(search_limit=2),clock=lambda:0.)
    coordinator.start(coordinator.plan()['plan_id'])
    for _ in range(35):coordinator.tick()
    assert coordinator.phase=='unverified'
    assert ports.calls.count('map_search')==2 and ports.calls.count('search')==1
    assert coordinator.status()['mapping_attempts']==2 and coordinator.searches==1


def test_missing_floor_keeps_its_own_bounded_failure():
    ports=Ports();poll=ports.poll
    ports.poll=lambda op:dict(needs_mapping=True) if op[0]=='plan_grasp' else poll(op)
    coordinator=Coordinator(ports,settings=Settings(search_limit=2),clock=lambda:0.)
    coordinator.start(coordinator.plan()['plan_id'])
    for _ in range(30):coordinator.tick()
    assert coordinator.phase=='failed' and coordinator.reason=='no_observed_free_grasp_path'
    assert ports.calls.count('map_search')==2 and coordinator.searches==0


def test_grasp_geometry_can_request_floor_observation_without_marking_unknown_cells_free():
    from elesim_pilot.mobile_manipulation.base_planner import BasePlanner,FloorObservationRequired
    rows=[dict(feature='side',root=1,xy_mm=[-100,300],q_deg=[-50,-150])]
    model=NS(meta={'home':[0,0,0,0]},sample_triangle=lambda n:rows,
        solve_many=lambda points:[dict(feature='side',root=1,xy_mm=p) for p in points])
    grid=LocalMap();footprint=np.array([[0.,0.]])
    planner=BasePlanner(model,grid,footprint,np.eye(4),.3,Settings())
    target=TargetTrack().update(cylinder((.7,.15)),stamp=1.,source=('sim','boot','calib'))
    with pytest.raises(FloorObservationRequired) as caught:planner.grasp(target,np.zeros(3))
    assert np.isfinite(caught.value.pose).all()
    assert not grid.free and not grid.blocked


def test_parent_route_change_and_cancel_cannot_continue_into_hug():
    p=Ports();c=Coordinator(p,clock=lambda:0.);c.start(c.plan()['plan_id']);p.identity=('sim','other','lease')
    c.tick();assert c.phase=='failed' and p.stops
    assert 'hug' not in p.calls
    p=Ports();c=Coordinator(p,clock=lambda:0.);c.start(c.plan()['plan_id'])
    c.cancel();c.tick();assert c.phase=='cancelled' and len(p.calls)==1


def test_motion_grant_revocation_fences_old_children():
    o=Ownership();r=('sim','boot','lease');o.acquire(r);g=o.grant('scan',{'arm'})
    o.release();o.acquire(r)
    with pytest.raises(RuntimeError):o.require(g,r,'arm')


def test_parent_stop_delivery_failure_still_fences_the_old_task():
    p=Ports();c=Coordinator(p,clock=lambda:0.);c.start(c.plan()['plan_id'])
    p.stop=lambda reason:(_ for _ in ()).throw(RuntimeError('route disappeared'))
    c.cancel();assert c.phase=='cancelled' and c.ownership.session is None
    assert 'stop delivery' in c.reason


def test_real_model_selects_an_interior_goal_with_observed_free_floor():
    import shutil
    if not shutil.which('node'):pytest.skip('Node solver unavailable')
    from elesim_pilot.reconstruction.hug import HugModel
    from elesim_pilot.mobile_manipulation.base_planner import BasePlanner
    model=HugModel(Path(__file__).resolve().parents[4]/'payload/data/policies/hug-d100')
    grid=LocalMap(.05)
    grid.observe(np.array([[x,y,0] for x in np.arange(-.4,1.4,.03) for y in np.arange(-.7,.8,.03)]))
    # Known target occupancy belongs to the local obstacle map.
    grid.observe(np.array([[.7+.05*np.cos(a),.15+.05*np.sin(a),.55] for a in np.linspace(0,2*np.pi,80)]))
    foot=rectangular_footprint([[-.1,.1],[-.08,.08]],padding=.01)
    node=np.eye(4);node[2,3]=.55
    p=BasePlanner(model,grid,foot,node,0.,Settings())
    e=TargetTrack().update(cylinder((.7,.15)),stamp=1.,source=('sim','boot','calib'))
    goal=p.grasp(e,np.array([0.,0.,0.]))
    assert goal.report['kind']=='grasp'
    assert min(goal.report['weights'])>0
    assert p.robust_solution(e,goal.pose,goal.report['error_allowance_m']) is not None
    assert grid.segment_free(goal.path[-2],goal.path[-1],foot)


def test_uncertain_final_handoff_replans_and_is_bounded():
    class Uncertain(Ports):
        def poll(self,op):
            if op[0]=='check_grasp':return dict(accepted=False)
            return super().poll(op)
    p=Uncertain();c=Coordinator(p,clock=lambda:0.);c.start(c.plan()['plan_id'])
    for _ in range(80):c.tick()
    assert c.phase=='failed' and c.reason=='grasp_localization_uncertain'
    assert 'hug' not in p.calls


def test_mapping_budget_does_not_allow_blind_walking():
    class Unknown(Ports):
        def poll(self,op):
            if op[0]=='plan_observation':return dict(needs_mapping=True)
            return super().poll(op)
    p=Unknown();c=Coordinator(p,clock=lambda:0.);c.start(c.plan()['plan_id'])
    for _ in range(80):c.tick()
    assert c.phase=='failed' and c.reason=='no_observed_free_base_path'
    assert 'move_observation' not in p.calls


def test_two_same_named_child_stages_cannot_reuse_a_finished_grant():
    o=Ownership();route=('sim','boot','lease');o.acquire(route)
    old=o.grant('plan_grasp',{'arm'});o.finish(old)
    current=o.grant('plan_grasp',{'arm'})
    assert old!=current
    with pytest.raises(RuntimeError):o.require(old,route,'arm')


def test_grasp_arrival_charges_yaw_error_at_object_distance():
    now=[0.];settings=Settings();executor=BaseExecutor(settings,lambda:now[0])
    goal=BaseGoal(np.zeros(3),np.array([[0.,0.,0.]]),np.zeros(4),dict(kind='grasp',target_reach_m=.5))
    executor.start(goal,[0,0,0],1)
    # Ten milliradians is inside the generic yaw tolerance but shifts a target
    # 0.5 m away by 5 mm, larger than this task's 3 mm arrival allowance.
    _,done=executor.step([0,0,.01],0,0,lambda *a:True);assert not done
    now[0]=1.;_,done=executor.step([0,0,.01],0,0,lambda *a:True);assert not done


def test_combined_motion_requires_both_child_channels_before_sending():
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    ports=SimPorts.__new__(SimPorts);route=('sim','boot','lease');sent=[]
    ports.route=lambda:route
    ports.connection=NS(send_workflow_motion=lambda *args:sent.append(args))
    owner=Ownership();owner.acquire(route);grant=owner.grant('scan',{'arm'})
    with pytest.raises(RuntimeError,match='motion grant'):
        ports._send(grant,owner,q=[0,0,0,0],go2_vel=[.05,0,0])
    assert not sent


def test_completed_commands_with_stale_success_evidence_remain_unverified():
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    ports=SimPorts.__new__(SimPorts);ports.clock=lambda:0.;ports.generation=0
    ports.hug_execution_id='path';ports.hug_final_tick=2000
    ports._latest={'manipulation_evaluation':dict(id='path',tick=1999,success=True)}
    owner=Ownership();owner.acquire(('sim','boot','lease'));grant=owner.grant('verify',{'arm'})
    op=ports.begin('verify',grant,owner)
    assert not op['result']['verified'] and not op['result']['evaluated_failure']
    ports._latest['manipulation_evaluation'].update(tick=2000)
    assert ports.begin('verify',grant,owner)['result']['verified']


@pytest.mark.parametrize('stale',[False,True])
@pytest.mark.parametrize('pitch',[0.,.006,.008])
def test_early_hold_completion_requires_fresh_evidence_and_reserved_drift(stale,pitch):
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    ports=SimPorts.__new__(SimPorts);ports.clock=lambda:0.;ports.last_send=0.;ports._summary={}
    ports.feedback=NS(snapshot=lambda *a:dict(pose=np.zeros(3)))
    from scipy.spatial.transform import Rotation
    current=np.eye(4);current[:3,:3]=Rotation.from_euler('y',pitch).as_matrix()
    current[0,3]=.0029 if pitch==.008 else .0015
    ports.route=lambda:('sim','boot','lease');ports._measured_body=lambda s:current
    ports.s=Settings();ports.node_in_body=np.eye(4);ports.node_in_body[2,3]=.3;ports.planner=NS(offset=144.5)
    target=NS(origin=np.array([.5,.1,.5]),axis=np.array([0.,0.,1.]))
    op=dict(phase='hug',started=0.,grant=None,ownership=None,body_frame=np.eye(4),timed_id='path',
        target=target,solver_xy=np.array([-100.,644.5]))
    ports.operation=op;ports.hug_evaluation_tick=2000
    ports._latest=dict(timed_arm=dict(id='path',state='executing',tick=1015),
        manipulation_evaluation=dict(id='path',tick=1014 if stale else 1015,success=True,hold_s=5.))
    sent=[];ports._send=lambda *a,**kw:sent.append(kw)
    if pitch==.008:
        with pytest.raises(RuntimeError,match='reserved solver-coordinate'):ports.poll(op)
        assert not sent
        return
    result=ports.poll(op)
    if stale:
        assert result is None and not sent and ports.hug_evaluation_tick==2000
    else:
        assert result=={} and ports.hug_evaluation_tick==1015
        assert sent==[dict(timed_arm=dict(id='path',operation='cancel'))]


def test_identified_target_survives_the_broad_robot_body_mask(tmp_path,monkeypatch):
    from elesim_pilot.mobile_manipulation import runtime
    from elesim_pilot.vision.partial_cloud import VoxelCloud
    ports=runtime.SimPorts.__new__(runtime.SimPorts)
    cloud=np.array([[.34+.05*np.cos(a),.1+.05*np.sin(a),.5] for a in np.linspace(0,2*np.pi,80)])
    sensor=NS(depth_raw=np.ones((8,10)),encoder_q=np.zeros(4),ts=10.)
    ports._capture=lambda:(NS(sensor=sensor),np.eye(4),np.eye(4))
    path=tmp_path/'models.yaml';path.write_text('models:\n  segmenter: {}\n')
    ports.scan=NS(models_path=path);ports.scan_source=('sim','boot','calib')
    segmenter=NS(implementation=NS(segment=lambda *a:[NS(mask=np.ones((8,10),bool))]))
    monkeypatch.setattr(runtime,'load_model',lambda *a:segmenter)
    monkeypatch.setattr(runtime,'reconstruct_view',lambda *a,**kw:NS(points_before=cloud.copy()))
    ports.collision=NS(base=np.eye(4),cloud=VoxelCloud(voxel_m=.01,max_voxels=1000),
        _centers=lambda q:np.array([[9.,9.,9.]]),_radii=np.array([.01]),tree=None)
    ports.feedback=NS(snapshot=lambda *a:dict(base=dict(pos=np.array([0.,0.,.3]),rpy=np.zeros(3)),
        pose=np.zeros(3),joint=dict(received=10.,stamp=10.)))
    ports.route=lambda:('sim','boot','lease');ports.clock=lambda:10.;ports.generation=0;ports.stopped=False
    ports.s=Settings();ports.grid=LocalMap();ports.track=TargetTrack();ports.target_id='target';ports._summary={}
    result=ports._observe()
    assert result['visible'] and ports.collision.tree is not None
    assert all(ports.grid.cell(p) in ports.grid.blocked for p in cloud)


def test_planning_holds_one_checked_pose_instead_of_following_sensor_drift():
    from concurrent.futures import Future
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    ports=SimPorts.__new__(SimPorts);now=[1.];q=np.array([-.12,0.,-.59,.59]);actual=q.copy()
    ports.clock=lambda:now[0];ports.last_send=0.;ports.route=lambda:('sim','boot','lease')
    ports.feedback=NS(snapshot=lambda *args:dict(joint=dict(q=actual.copy()),pose=np.zeros(3)))
    sent=[];ports._send=lambda *args,**kw:sent.append(kw['q'])
    op=dict(phase='plan_observation',started=0.,grant=None,ownership=None,future=Future(),hold_q=q.copy())
    ports.operation=op
    assert ports.poll(op) is None
    actual[2]+=.005;now[0]+=.1
    assert ports.poll(op) is None
    np.testing.assert_array_equal(sent,[q,q])
    actual[2]+=.02;now[0]+=.1
    with pytest.raises(RuntimeError,match='checked posture'):ports.poll(op)


def test_target_search_rechecks_return_to_last_visible_pose_after_a_floor_look():
    from concurrent.futures import Future
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    ports=SimPorts.__new__(SimPorts);home=np.array([-.12,0.,-.59,.59]);floor=home+[.04,-.7,.3,-.1]
    ports.clock=lambda:0.;ports.generation=0;ports._initialize=lambda:None
    ports.feedback=NS(snapshot=lambda *a:dict(joint=dict(q=floor.copy())))
    ports.route=lambda:('sim','boot','lease');ports.history=[home.copy(),floor.copy()]
    ports.last_target_q=home.copy();checked=[]
    def certify(waypoints):
        checked.extend(waypoints);return [floor.copy()]+waypoints
    def submit(function,*args):
        future=Future();future.set_result(function(*args));return future
    ports._arm_prepare=certify;ports.pool=NS(submit=submit)
    op=ports.begin('search',None,None,index=3)
    assert op['future'].done()
    np.testing.assert_array_equal(checked,[home])


@pytest.mark.parametrize('blocked',['none','direct','all'])
def test_return_uses_shortcut_only_when_its_whole_segment_is_certified(blocked):
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    ports=SimPorts.__new__(SimPorts)
    start=np.array([0.,1.,0.,0.]);via=np.array([0.,1.,1.,0.]);goal=np.zeros(4)
    ports.feedback=NS(snapshot=lambda *a:dict(joint=dict(q=start.copy())))
    ports.route=lambda:('sim','boot','lease');ports._summary={};ports.history=[goal.copy(),via.copy(),start.copy()]
    ports.connection=NS(state_sink=NS(get_state=lambda:None))
    ports.service=NS(_ik_context_for_host=lambda h:dict(base_world_T=np.eye(4)))
    checked=[]
    def certify(a,b):
        checked.append((a.copy(),b.copy()))
        direct=np.array_equal(a,start) and np.array_equal(b,goal)
        return dict(accepted=blocked=='none' or (blocked=='direct' and not direct))
    ports.collision=NS(base=None,certify_segment=certify)
    if blocked=='all':
        with pytest.raises(RuntimeError,match='path blocked'):ports._return_prepare(goal)
    else:
        points=ports._return_prepare(goal)
        if blocked=='none':np.testing.assert_array_equal(points,[start,goal])
        else:assert any(np.array_equal(point,via) for point in points)
        np.testing.assert_array_equal(points[-1],goal)
    assert checked


def test_actual_body_tilt_changes_final_solver_coordinates():
    from elesim_pilot.mobile_manipulation.base_planner import BasePlanner,body_frame
    model=NS(meta={'home':[0,0,0,0]},solve_many=lambda points:[dict(feature='side',root=1,xy_mm=p) for p in points])
    node=np.eye(4);node[:3,3]=[.3,0,.1]
    planner=BasePlanner(model,None,None,node,.32,Settings())
    target=TargetTrack().update(cylinder((.7,.15)),stamp=1.,source=('sim','boot','calib'))
    flat=planner.robust_solution(target,[0,0,0],.003)
    measured=body_frame([0,0,0],.30,[.02,.04])
    tilted=planner.robust_solution(target,[0,0,0],.003,measured)
    np.testing.assert_allclose(tilted['xy_mm'],solver_xy(target,measured@node))
    assert not np.allclose(flat['xy_mm'],tilted['xy_mm'])
