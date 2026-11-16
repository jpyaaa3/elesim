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


def test_scan_readiness_reuses_the_final_d100_quality_without_refreshing_the_track():
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    ports=SimPorts.__new__(SimPorts);ports.target_id='target';ports.clock=lambda:1.
    ports.scan_source=('sim','boot','calib');previous=object();ports.track=NS(estimate=previous)
    assert ports._scan_observation_ready(cylinder())
    assert not ports._scan_observation_ready(cylinder(r=.055))
    assert not ports._scan_observation_ready(np.zeros((3,3)))
    assert ports.track.estimate is previous


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


@pytest.mark.parametrize('resolution',[.04,.01])
@pytest.mark.parametrize('margin',[0.,.022])
def test_footprint_raster_matches_the_exact_union_of_tracking_rectangles(resolution,margin):
    from elesim_pilot.mobile_manipulation.contracts import planar
    grid=LocalMap(resolution);foot=np.random.default_rng(37).uniform(-.3,.3,(53,2))
    pose=np.array([-.007,.013,.41]);t=planar(pose);points=foot@t[:2,:2].T+t[:2,3]
    low=np.floor((points-margin)/resolution).astype(int)
    high=np.floor((points+margin)/resolution).astype(int);expected=set()
    for a,b in zip(low,high):
        expected.update((x,y) for x in range(a[0],b[0]+1) for y in range(a[1],b[1]+1))
    assert grid._footprint_cells(pose,foot,margin_m=margin)==sorted(expected)


def test_fine_grid_separates_a_measured_obstacle_from_a_clear_transport_boundary():
    # A 40 mm cell merges these physically separate regions; the margin and
    # obstacle stay identical when measured floor is rasterized at 10 mm.
    goal=np.array([.219,.01,0.]);obstacle=np.array([[.6505,.15,.2]])
    statuses=[]
    for resolution in (.04,.01):
        grid=LocalMap(resolution);foot=rectangular_footprint([[-.38,.38],[-.25,.25]],
            spacing=resolution/2,padding=.02)
        cells=grid._footprint_cells(goal,foot,margin_m=.022)
        grid.observe(np.array([[*(np.array(c)+.5)*resolution,0.] for c in cells]))
        grid.observe(obstacle)
        statuses.append(grid.footprint_status(goal,foot,margin_m=.022))
    assert statuses[0]['blocked_cells']>0
    assert statuses[1]['blocked_cells']==statuses[1]['unknown_cells']==0


def test_local_map_translation_keeps_the_same_measured_cells_routes_and_diagnostics():
    shift=np.array([.137,-.083]);base=LocalMap(.01);moved=LocalMap(.01,origin_xy=shift)
    points=np.array([[x+.002,y+.003,0.] for x in np.arange(-.1,.31,.005) for y in np.arange(-.1,.11,.005)])
    base.observe(points);moved.observe(points+np.r_[shift,0.])
    foot=np.array([[.002,.003],[.012,.013]]);start=np.array([0.,0.,.1]);goal=np.array([.2,0.,.1])
    assert base.free==moved.free and base.blocked==moved.blocked
    np.testing.assert_allclose(moved.path(start+np.r_[shift,0.],goal+np.r_[shift,0.],foot),
        base.path(start,goal,foot)+np.r_[shift,0.])
    base.blocked.add((0,0));moved.blocked.add((0,0))
    a=base.footprint_status(start,foot);b=moved.footprint_status(start+np.r_[shift,0.],foot)
    np.testing.assert_allclose(np.array(b['blocked_xy_m'])-shift,a['blocked_xy_m'])
    assert a['blocked_cells']==b['blocked_cells']


def test_fine_local_window_covers_both_ends_of_the_requested_transport_corridor():
    grid=LocalMap(.01);start=np.zeros(3);goal=np.array([.44,.57,-1.31])
    grid.recenter((start[:2]+goal[:2])/2)
    foot=rectangular_footprint([[-.38,.38],[-.25,.25]],spacing=.005,padding=.02)
    for pose in (start,goal):
        cells=grid._footprint_cells(pose,foot,margin_m=.022)
        assert all(grid._inside(c) for c in cells)
        assert not grid.footprint_free(pose,foot,margin_m=.022)
    assert not grid.free and not grid.blocked and grid.max_cells==40000


@pytest.mark.parametrize('all_blocked',[False,True])
def test_grasp_floor_requests_never_select_an_observed_obstacle(all_blocked):
    from elesim_pilot.mobile_manipulation.base_planner import BasePlanner,FloorObservationRequired
    row=dict(xy_mm=[0.,400.],feature='side',root=1)
    model=NS(meta={'home':np.zeros(4)},sample_triangle=lambda n:[row],
        solve_many=lambda points:[row for _ in points])
    def status(pose,*a,**kw):
        blocked=all_blocked or abs(angle_for_test(pose[2]))<.4
        return dict(blocked_cells=int(blocked),unknown_cells=int(not blocked))
    def angle_for_test(value):return (value+np.pi)%(2*np.pi)-np.pi
    grid=NS(footprint_free=lambda pose,*a,**kw:np.linalg.norm(pose[:2])<1e-9,
        footprint_status=status)
    planner=BasePlanner(model,grid,np.array([[0.,0.]]),np.eye(4),.3,Settings())
    target=TargetTrack().update(cylinder((.7,.15)),stamp=1.,source=('sim','boot','calib'))
    if all_blocked:
        with pytest.raises(ValueError,match='no reachable C-band'):
            planner.grasp(target,np.zeros(3))
    else:
        with pytest.raises(FloorObservationRequired) as caught:
            planner.grasp(target,np.zeros(3))
        assert status(caught.value.pose)['blocked_cells']==0
        assert status(caught.value.pose)['unknown_cells']>0


@pytest.mark.parametrize('operation',['grasp','observation'])
def test_observed_obstacle_at_the_start_cannot_be_cleared_by_floor_views(operation):
    from elesim_pilot.mobile_manipulation.base_planner import BasePlanner
    grid=LocalMap();grid.blocked.add((0,0))
    planner=BasePlanner(NS(meta={}),grid,np.array([[0.,0.]]),np.eye(4),.3,Settings())
    target=TargetTrack().update(cylinder(),stamp=1.,source=('sim','boot','calib'))
    with pytest.raises(ValueError,match='observed obstacle'):
        if operation=='grasp':planner.grasp(target,np.zeros(3))
        else:planner.observation(target,np.zeros(3),lambda *a:None)


def test_path_margin_requires_measured_room_for_small_tracking_errors():
    grid=LocalMap(.04)
    foot=rectangular_footprint([[-.38,.38],[-.25,.25]],padding=.02)
    start=np.array([-.006,.001,.0003]);goal=start+np.array([.22,.01,0.])
    delta=goal-start
    for t in np.linspace(0,1,30):grid.free.update(grid._footprint_cells(start+t*delta,foot))
    assert grid.segment_free(start,goal,foot)
    before=grid.free.copy()
    # Nominal samples must not authorize an unseen adjacent cell. Margin is
    # required by planning, while measured-motion checks retain the true pose.
    assert not grid.segment_free(start,goal,foot,margin_m=.025)
    assert grid.free==before
    with pytest.raises(ValueError):grid.path(start,goal,foot,margin_m=.025)
    needed=set()
    for t in np.linspace(0,1,30):needed.update(grid._footprint_cells(start+t*delta,foot,margin_m=.025))
    grid.observe(np.array([[*(np.array(c)+.5)*grid.resolution,0.] for c in needed]))
    assert grid.segment_free(start,goal,foot,margin_m=.025)
    for dx,dy,yaw in ((-.003,0.,-.002),(.008,-.006,.02),(-.008,.006,-.02)):
        assert grid.segment_free(start+[dx,dy,yaw],goal+[dx,dy,yaw],foot)
    outside=next(iter(needed-before))
    grid.observe(np.array([[*(np.array(outside)+.5)*grid.resolution,.2]]))
    assert grid.segment_free(start,goal,foot)
    assert not grid.segment_free(start,goal,foot,margin_m=.025)


def test_grasp_maps_the_starting_clearance_before_selecting_a_destination():
    from elesim_pilot.mobile_manipulation.base_planner import BasePlanner,FloorObservationRequired
    grid=LocalMap();foot=rectangular_footprint([[-.38,.38],[-.25,.25]],padding=.02)
    current=np.array([-.006,.001,.0003])
    grid.free.update(grid._footprint_cells(current,foot))
    class Model:
        meta={'home':[0.,0.,0.,0.]}
        def sample_triangle(self,n):raise AssertionError('map missing start clearance first')
    planner=BasePlanner(Model(),grid,foot,np.eye(4),.3,Settings())
    target=TargetTrack().update(cylinder((.7,.15)),stamp=1.,source=('sim','boot','calib'))
    with pytest.raises(FloorObservationRequired) as caught:planner.grasp(target,current)
    np.testing.assert_array_equal(caught.value.pose,current)


def test_floor_request_targets_missing_tracking_room_without_creating_free_cells():
    from elesim_pilot.mobile_manipulation.base_planner import FloorObservationRequired
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    ports=SimPorts.__new__(SimPorts);ports.grid=LocalMap();ports._summary={}
    ports.footprint=np.array([[.1,.1]]);ports.planner=NS(clearance_m=.025)
    ports.grid.free.add((2,2));before=ports.grid.free.copy()
    assert ports._mapping_required(FloorObservationRequired([0.,0.,0.]))==dict(needs_mapping=True)
    assert ports.mapping_margin_m==.025
    assert ports.summary()['map_goal']['unknown_cells']>0
    assert ports.grid.free==before


@pytest.mark.parametrize('invalid_first',[False,True])
def test_floor_looks_choose_a_new_camera_view_after_an_unproductive_capture(monkeypatch,tmp_path,invalid_first):
    from scipy.stats import qmc
    from elesim_pilot.mobile_manipulation import runtime
    monkeypatch.setattr(qmc,'Halton',lambda *a,**k:NS(random=lambda n:np.array([[x,0,0,0] for x in [0,.05,.1]])))
    class CameraModel:
        def camera_pose(self,q):
            pose=np.diag([1.,-1.,-1.,1.]);pose[:3,3]=[.45+q[0],0,.5];return pose
    monkeypatch.setattr(runtime,'ArmCameraModel',lambda *a,**k:CameraModel())
    ports=runtime.SimPorts.__new__(runtime.SimPorts)
    ports.arm=NS(bounds=np.array([[0.,1.]]*4),uncertainty=np.array([.001,.015,.015,.015]));ports.scan=NS(model_dir=tmp_path/'assembly',hand_eye=tmp_path/'eye.json')
    ports.service=NS(_ik_context_for_host=lambda h:dict(base_world_T=np.eye(4)))
    ports.connection=NS(state_sink=NS(get_state=lambda:None));ports.route=lambda:('sim','boot','lease')
    ports.feedback=NS(snapshot=lambda *a:dict(joint=dict(q=np.zeros(4)),pose=np.zeros(3)))
    ports.grid=LocalMap(.1);ports.mapping_views=[];ports._summary={};ports._arm_prepare=lambda qs:[np.zeros(4),*qs]
    ports.collision=NS(base=None,configuration_free=lambda q:not invalid_first or q[0]>0)
    first=ports._map_search_prepare(1)[-1]
    if invalid_first:
        assert first[0]>0 and ports._summary['floor_search']['arrival_rejections']>0
    second=ports._map_search_prepare(2)[-1]
    assert len(ports.mapping_views)==2 and abs(first[0]-second[0])>=.03
    assert not ports.grid.free and not ports.grid.blocked


def test_floor_search_reuses_cached_views_and_aims_at_missing_goal_cells(monkeypatch,tmp_path):
    from scipy.stats import qmc
    from elesim_pilot.mobile_manipulation import runtime
    monkeypatch.setattr(qmc,'Halton',lambda *a,**k:NS(random=lambda n:np.zeros((1,4))))
    class CameraModel:
        def camera_pose(self,q):
            pose=np.diag([1.,-1.,-1.,1.]);pose[:3,3]=[.45+q[0],q[1],.5];return pose
    model=CameraModel();monkeypatch.setattr(runtime,'ArmCameraModel',lambda *a,**k:model)
    ports=runtime.SimPorts.__new__(runtime.SimPorts)
    ports.arm=NS(bounds=np.array([[0.,1.]]*4),uncertainty=np.array([.001,.015,.015,.015]));ports.scan=NS(model_dir=tmp_path/'assembly',hand_eye=tmp_path/'eye.json')
    ports.service=NS(_ik_context_for_host=lambda h:dict(base_world_T=np.eye(4)))
    ports.connection=NS(state_sink=NS(get_state=lambda:None));ports.route=lambda:('sim','boot','lease')
    ports.feedback=NS(snapshot=lambda *a:dict(joint=dict(q=np.zeros(4)),pose=np.zeros(3)))
    ports.camera_q=np.array([[0.,0,0,0],[.2,.15,0,0]])
    ports.camera_cache=np.array([model.camera_pose(q) for q in ports.camera_q])
    ports.mapping_goal_pose=np.zeros(3);ports.footprint=np.array([[.65,.15]])
    ports.mapping_intrinsics=NS(fx=500.,fy=500.,cx=50.,cy=50.,width=100,height=100)
    ports.collision=NS(base=None,configuration_free=lambda q:True)
    ports.grid=LocalMap(.1);ports.mapping_views=[];ports._summary={}
    ports._arm_prepare=lambda qs:[np.zeros(4),*qs]
    result=ports._map_search_prepare(1)
    np.testing.assert_allclose(result[-1],ports.camera_q[1])
    assert not ports.grid.free and not ports.grid.blocked


@pytest.mark.parametrize('route',['direct','later_direct','later_simple','refined','history','blocked','proposal_rejected'])
def test_floor_search_checks_direct_candidates_before_certified_history_detours(monkeypatch,tmp_path,route):
    from scipy.stats import qmc
    from elesim_pilot.mobile_manipulation import runtime
    monkeypatch.setattr(qmc,'Halton',lambda *a,**k:NS(random=lambda n:np.array([[0.,0,0,0],[.1,0,0,0]])))
    class CameraModel:
        def camera_pose(self,q):
            pose=np.diag([1.,-1.,-1.,1.]);pose[:3,3]=[.45+q[0],0.,.5];return pose
    monkeypatch.setattr(runtime,'ArmCameraModel',lambda *a,**k:CameraModel())
    ports=runtime.SimPorts.__new__(runtime.SimPorts)
    current=np.array([0.,.3,0.,0.]);home=np.array([0.,.15,0.,0.])
    ports.arm=NS(bounds=np.array([[0.,1.]]*4),uncertainty=np.array([.001,.015,.015,.015]));ports.scan=NS(model_dir=tmp_path/'assembly',hand_eye=tmp_path/'eye.json')
    ports.service=NS(_ik_context_for_host=lambda h:dict(base_world_T=np.eye(4)))
    ports.connection=NS(state_sink=NS(get_state=lambda:None));ports.route=lambda:('sim','boot','lease')
    ports.feedback=NS(snapshot=lambda *a:dict(joint=dict(q=current.copy()),pose=np.zeros(3)))
    ports.grid=LocalMap(.1);ports.mapping_views=[];ports._summary={};ports.history=[home.copy(),current.copy()]
    ports.generation=0;ports.stopped=False
    checked=[]
    def certify(a,b):
        checked.append((a.copy(),b.copy()))
        direct=np.array_equal(a,current) and b[1]==0.
        clear_direct=direct and ((route=='direct' and b[0]==0.) or (route=='later_direct' and b[0]>.05))
        clear_history=(np.array_equal(a,b) or (np.array_equal(a,current) and np.array_equal(b,home))
            or (np.array_equal(a,home) and b[1]==0.))
        return dict(accepted=clear_direct or (route=='history' and clear_history),reason='observed_surface_collision_on_path')
    ports.collision=NS(base=None,configuration_free=lambda q:route!='refined',certify_segment=certify)
    refined_calls=[]
    if route=='refined':
        def refine(current,probes,model,seeds):
            refined_calls.append(len(probes));goal=np.array([.15,0.,0.,0.])
            ports.mapping_views.append(model.camera_pose(goal))
            return [current.copy(),goal]
        ports._refine_floor_prepare=refine
    simple_calls=[]
    if route=='later_simple':
        def returning(goal,*,use_history=True):
            simple_calls.append((goal.copy(),use_history))
            if use_history or goal[0]<.05:raise runtime.ArmPathBlocked('long history needed')
            ports._summary['arm_return']=dict(strategy='retract_transform_extend')
            return [current.copy(),home.copy(),goal.copy()]
        ports._return_prepare=returning
    if route in {'blocked','proposal_rejected'}:
        if route=='proposal_rejected':
            from elesim_pilot.robot.arm import joint_path
            monkeypatch.setattr(joint_path,'connect_joint_path',lambda start,goals,*a,**kw:
                (np.array([start,home,goals[0]]),dict(goal_index=0)))
        with pytest.raises(RuntimeError,match='path'):ports._map_search_prepare(1)
        assert not ports.mapping_views
    else:
        path=ports._map_search_prepare(1)
        if route=='history':
            assert any(np.array_equal(q,home) for q in path)
        elif route=='later_simple':
            assert len(path)==3 and path[-1][0]==pytest.approx(.1)
            assert len(simple_calls)==2 and not any(use_history for q,use_history in simple_calls)
        elif route=='refined':
            assert refined_calls and path[-1][0]==pytest.approx(.15)
            assert not checked
        else:
            assert len(path)==2
            assert path[-1][0]==pytest.approx(.1 if route=='later_direct' else 0.)
            assert not any(np.array_equal(b,home) for a,b in checked)
        assert len(ports.mapping_views)==1
    assert not ports.grid.free and not ports.grid.blocked


@pytest.mark.parametrize('case',['accepted','measured_jitter','blocked_path','lost_reserve','moved_body'])
def test_refined_floor_goal_still_requires_a_live_path_reserve_and_view(monkeypatch,tmp_path,case):
    from elesim_pilot.mobile_manipulation import runtime
    from elesim_pilot.robot.arm import view_refinement
    p=runtime.SimPorts.__new__(runtime.SimPorts);current=np.zeros(4);goal=np.ones(4)*.05
    p.arm=NS(bounds=np.array([[-.2,.2]]*4),uncertainty=np.array([.001,.015,.015,.015]))
    reserved=[]
    def margins(q,**kwargs):
        reserved.append(kwargs);return np.array([.1])
    p.collision=NS(base=np.eye(4),configuration_margins=margins,
        configuration_free=lambda q:case!='lost_reserve')
    p.mapping_intrinsics=NS(fx=500.,fy=500.,cx=320.,cy=240.,width=640,height=480)
    p.mapping_views=[];p.generation=0;p.stopped=False;p._summary={'floor_search':{}}
    p.scan=NS(model_dir=tmp_path/'assembly',hand_eye=tmp_path/'eye.json')
    def camera_model(*a,base_world_transform,**kw):
        pose=np.diag([1.,-1.,-1.,1.]);pose[:3,3]=[.45,0.,.5]
        return NS(camera_pose=lambda q:base_world_transform@pose)
    monkeypatch.setattr(runtime,'ArmCameraModel',camera_model)
    def refine(*args,**kwargs):
        args[4](goal)
        return goal.copy(),dict(status='found')
    monkeypatch.setattr(view_refinement,'refine_view',refine)
    checked=[]
    def prepare(waypoints):
        checked.extend(waypoints)
        if case=='blocked_path':raise runtime.ArmPathBlocked('blocked live segment')
        if case=='moved_body':p.collision.base[0,3]=1.
        if case=='measured_jitter':p.collision.base[0,3]=.001
        return [current.copy(),*waypoints]
    p._arm_prepare=prepare
    model=camera_model(base_world_transform=np.eye(4))
    path=p._refine_floor_prepare(current,np.array([[.45,-.01,0.],[.45,.01,0.]]),model,[])
    assert checked
    assert any(x.get('base_position_error_m',0)>0 and x.get('base_angle_error_rad',0)>0 for x in reserved)
    if case in {'accepted','measured_jitter'}:
        np.testing.assert_array_equal(path[-1],goal)
        assert len(p.mapping_views)==1
        assert p._summary['floor_search']['path_strategy']=='refined_direct'
    else:
        assert path is None and not p.mapping_views


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


@pytest.mark.parametrize('acquisition',['acquire','post_scan_acquire'])
def test_floor_views_continue_before_one_final_fresh_target_recovery(acquisition):
    ports=Ports();poll=ports.poll;views=[0];recovered=[False]
    planning='plan_observation' if acquisition=='acquire' else 'plan_grasp'
    def collect(op):
        phase,_=op
        if phase==planning and views[0]==0:return dict(needs_mapping=True)
        if phase=='map_search':views[0]+=1
        if phase==acquisition and 0<views[0]<3:
            return dict(visible=False,needs_mapping=True)
        if phase==acquisition and views[0]==3 and not recovered[0]:
            recovered[0]=True;return dict(visible=False)
        return poll(op)
    ports.poll=collect;c=Coordinator(ports,settings=Settings(search_limit=3),clock=lambda:0.)
    c.start(c.plan()['plan_id'])
    for _ in range(40):c.tick()
    assert c.phase=='unverified' and views[0]==3
    assert c.mapping_attempts==3 and c.searches==1
    assert ports.calls.count('scan')==1
    last_view=max(i for i,phase in enumerate(ports.calls) if phase=='map_search')
    assert ports.calls.index('search')>last_view
    assert ports.calls[last_view+1:last_view+4]==[acquisition,'search',acquisition]


@pytest.mark.parametrize('acquisition,reason',[('acquire','no_observed_free_base_path'),
    ('post_scan_acquire','no_observed_free_grasp_path')])
def test_continuing_floor_views_keeps_the_same_budget_and_cannot_start_blind_motion(acquisition,reason):
    ports=Ports();poll=ports.poll;started=[False]
    planning='plan_observation' if acquisition=='acquire' else 'plan_grasp'
    def missing(op):
        phase,_=op
        if phase==planning:started[0]=True;return dict(needs_mapping=True)
        if phase==acquisition and started[0]:return dict(visible=False,needs_mapping=True)
        return poll(op)
    ports.poll=missing;c=Coordinator(ports,settings=Settings(search_limit=2),clock=lambda:0.)
    c.start(c.plan()['plan_id'])
    for _ in range(30):c.tick()
    assert c.phase=='failed' and c.reason==reason
    assert ports.calls.count('map_search')==2 and c.searches==0
    assert ('move_observation' if acquisition=='acquire' else 'move_grasp') not in ports.calls


@pytest.mark.parametrize('status',['unknown','free','blocked'])
def test_missing_target_can_continue_only_a_real_unblocked_pending_floor_request(status):
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    p=SimPorts.__new__(SimPorts);p.grid=LocalMap(.01);p.footprint=np.array([[.005,.005]])
    p.mapping_goal_pose=np.zeros(3);p.mapping_margin_m=0.;p._summary={}
    p.track=NS(estimate=NS(stamp=1.))
    if status=='free':p.grid.observe(np.array([[.005,.005,0.]]))
    if status=='blocked':p.grid.observe(np.array([[.005,.005,.5]]))
    result=p._missing_target(allow_mapping=True)
    assert not result['visible'] and bool(result.get('needs_mapping'))==(status=='unknown')
    assert p.track.estimate.stamp==1.
    assert p._missing_target(allow_mapping=False)==dict(visible=False)


@pytest.mark.parametrize('phase',['plan_observation','plan_grasp'])
def test_fresh_base_planning_discards_the_previous_floor_request(phase):
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    p=SimPorts.__new__(SimPorts);p.mapping_goal_pose=np.ones(3);p.mapping_goal_xy=np.ones(2)
    p.generation=0;p.clock=lambda:0.;p.route=lambda:('sim','boot','lease')
    p.feedback=NS(snapshot=lambda *a:dict(joint=dict(q=np.zeros(4))))
    p.pool=NS(submit=lambda *a:object())
    p.begin(phase,None,None)
    assert not hasattr(p,'mapping_goal_pose') and not hasattr(p,'mapping_goal_xy')


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
    # Mounted arm node is in front of the body, so the cylinder need not sit
    # against the body's corner to land in the supported C-band triangle.
    node=np.eye(4);node[:3,3]=[.2,0.,.55]
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


def test_floor_witnesses_keep_one_measured_point_per_cell_with_a_hard_budget():
    from elesim_pilot.mobile_manipulation.local_map import floor_cell_points
    sensor=NS(depth_raw=np.ones((4,4)),depth_scale=1.,
        intrinsics=NS(fx=10.,fy=10.,cx=1.5,cy=1.5))
    pose=np.diag([1.,-1.,-1.,1.]);pose[2,3]=1.
    points=floor_cell_points(sensor,pose,resolution=.05,max_cells=16)
    assert len(points)==len({tuple(c) for c in np.floor(points[:,:2]/.05).astype(int)})==16
    assert np.all(points[:,2]==0.)
    with pytest.raises(RuntimeError,match='map budget'):
        floor_cell_points(sensor,pose,resolution=.05,max_cells=15)
    local=floor_cell_points(sensor,pose,resolution=.05,max_cells=1,
        cell_window=(np.array([1,1]),np.array([2,2])))
    np.testing.assert_allclose(local,[[.05,.05,0.]])


def test_floor_witnesses_follow_the_translated_map_origin():
    from elesim_pilot.mobile_manipulation.local_map import floor_cell_points
    sensor=NS(depth_raw=np.ones((4,4)),depth_scale=1.,
        intrinsics=NS(fx=13.,fy=13.,cx=1.5,cy=1.5))
    pose=np.diag([1.,-1.,-1.,1.]);pose[:3,3]=[.013,.017,1.]
    shift=np.array([.137,-.083]);moved=pose.copy();moved[:2,3]+=shift
    a=floor_cell_points(sensor,pose,resolution=.01,max_cells=100)
    b=floor_cell_points(sensor,moved,resolution=.01,max_cells=100,origin_xy=shift)
    np.testing.assert_allclose(b-np.r_[shift,0.],a,atol=1e-15)


def test_local_map_retains_only_measured_cells_in_its_capacity_derived_window():
    grid=LocalMap(resolution=1.,max_cells=16);grid.recenter([0.,0.])
    points=np.array([[x+.5,y+.5,0.] for x in range(-10,10) for y in range(-10,10)])
    grid.observe(points)
    assert len(grid.free)==16 and not grid.blocked
    assert grid.footprint_free([0.,0.,0.],[[.5,.5]])
    grid.observe([[.5,.5,.2]])
    assert (0,0) in grid.blocked and (0,0) not in grid.free
    grid.recenter([4.,0.])
    assert not grid.free and not grid.blocked
    assert not grid.footprint_free([0.,0.,0.],[[.5,.5]])
    grid.observe(points)
    assert len(grid.free)==16 and (4,0) in grid.free and (0,0) not in grid.free
    assert grid.footprint_status([0.,0.,0.],[[.5,.5]])['unknown_cells']==1


@pytest.mark.parametrize('empty',[False,True])
def test_floor_witnesses_preserve_the_first_valid_pixel_and_cell_order(empty):
    import math
    from elesim_pilot.mobile_manipulation.local_map import floor_cell_points
    depth=(1000+np.arange(96).reshape(8,12)%15).astype(np.uint16)
    depth[0,:3]=[0,1200,65535]
    if empty:depth.fill(0)
    intr=NS(fx=30.,fy=30.,cx=5.5,cy=3.5)
    sensor=NS(depth_raw=depth,depth_scale=.001,intrinsics=intr)
    pose=np.diag([1.,-1.,-1.,1.]);pose[2,3]=1.
    expected={}
    for v,u in np.ndindex(depth.shape):
        z=float(depth[v,u])*.001
        if not .1<=z<=5 or abs(1-z)>=.025:continue
        point=np.array([(u-intr.cx)/intr.fx*z,-(v-intr.cy)/intr.fy*z,1-z])
        cell=tuple(math.floor(value/.08) for value in point[:2])
        expected.setdefault(cell,point)
    points=floor_cell_points(sensor,pose,resolution=.08,max_cells=96)
    np.testing.assert_array_equal(points,np.array([expected[c] for c in sorted(expected)]).reshape(-1,3))


@pytest.mark.parametrize('depth_mm,exclusion,expected_free',[
    (1000,None,True),(800,None,False),(0,None,False),
    (1000,'body',False),(1000,'arm',False),(1000,'occupied',False),
])
def test_floor_pixels_survive_cloud_thinning_without_inventing_free_space(
        tmp_path,monkeypatch,depth_mm,exclusion,expected_free):
    from elesim_pilot.mobile_manipulation import runtime
    from elesim_pilot.vision.partial_cloud import SensorObservation,VoxelCloud
    from elesim_pilot.vision.perception.depth_pose import CameraIntrinsics
    depth=np.full((480,640),2000,np.uint16)
    sampled=set(np.linspace(0,depth.size-1,6000,dtype=int))
    pixel=next(i for i in range(100*640+400,100*640+420) if i not in sampled)
    v,u=divmod(pixel,640);depth[v,u]=depth_mm
    pose=np.diag([1.,-1.,-1.,1.]);pose[2,3]=1.
    floor=np.array([u/500.,-v/500.,0.])
    sensor=SensorObservation(np.zeros((480,640,3),np.uint8),depth,.001,
        CameraIntrinsics(500.,500.,0.,0.,640,480),1,10.,'sim','boot','calib',(0.,)*4)
    ports=runtime.SimPorts.__new__(runtime.SimPorts)
    ports._capture=lambda:(NS(sensor=sensor),pose,np.eye(4))
    path=tmp_path/'models.yaml';path.write_text('models:\n  segmenter: {}\n')
    ports.scan=NS(models_path=path)
    monkeypatch.setattr(runtime,'load_model',lambda *a:NS(implementation=NS(segment=lambda *a:[])))
    center=floor if exclusion=='arm' else np.array([9.,9.,9.])
    ports.collision=NS(base=np.eye(4),cloud=VoxelCloud(voxel_m=.01,max_voxels=10000),
        _centers=lambda q:np.array([center]),_radii=np.array([.02]),tree=None)
    body=np.r_[floor[:2],.3] if exclusion=='body' else np.array([0.,0.,.3])
    ports.feedback=NS(snapshot=lambda *a:dict(base=dict(pos=body,rpy=np.zeros(3))))
    ports.route=lambda:('sim','boot','lease');ports.generation=0;ports.stopped=False
    ports.grid=LocalMap();ports.track=NS(estimate=None);ports.target_id='target';ports._summary={}
    if exclusion=='occupied':ports.grid.blocked.add(ports.grid.cell(floor))
    assert ports._observe()==dict(visible=False)
    assert (ports.grid.cell(floor) in ports.grid.free)==expected_free


def test_identified_target_survives_the_broad_robot_body_mask(tmp_path,monkeypatch):
    from elesim_pilot.mobile_manipulation import runtime
    from elesim_pilot.vision.partial_cloud import VoxelCloud
    ports=runtime.SimPorts.__new__(runtime.SimPorts)
    cloud=np.array([[.34+.05*np.cos(a),.1+.05*np.sin(a),.5] for a in np.linspace(0,2*np.pi,80)])
    sensor=NS(depth_raw=np.ones((8,10)),depth_scale=1.,
        intrinsics=NS(fx=1.,fy=1.,cx=0.,cy=0.),encoder_q=np.zeros(4),ts=10.)
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


def test_completed_observation_preserves_every_certified_waypoint_for_return():
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    ports=SimPorts.__new__(SimPorts)
    path=np.array([[0.,0.,0.,0.],[0.,.2,0.,0.],[0.,.2,.3,0.]])
    ports.clock=lambda:1.;ports.route=lambda:('sim','boot','lease')
    ports.feedback=NS(snapshot=lambda *a:dict(joint=dict(q=path[-1].copy(),sim_time=1.)))
    ports.last_command=path[-1].copy();ports.last_send=1.;ports.history=[path[0].copy()]
    op=dict(phase='map_search',started=0.,grant=None,ownership=None,path=path,index=2,
        approach=NS(step=lambda q,dt:path[-1].copy()),stable=0.,last_at=1.,args={})
    ports.operation=op
    assert ports.poll(op)==dict(purpose='')
    np.testing.assert_array_equal(ports.history,path)


def test_target_search_rechecks_return_to_last_visible_pose_after_a_floor_look():
    from concurrent.futures import Future
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    ports=SimPorts.__new__(SimPorts);home=np.array([-.12,0.,-.59,.59]);floor=home+[.04,-.7,.3,-.1]
    ports.clock=lambda:0.;ports.generation=0;ports._initialize=lambda:None;ports._summary={}
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
    ports.arm=NS(bounds=np.array([[0.,1.]]*4),uncertainty=np.array([.001,.015,.015,.015]))
    start=np.array([0.,1.,0.,0.]);via=np.array([0.,1.,1.,0.]);goal=np.zeros(4)
    ports.feedback=NS(snapshot=lambda *a:dict(joint=dict(q=start.copy())))
    unused_home=np.array([0.,0.,2.,0.])
    ports.route=lambda:('sim','boot','lease');ports._summary={};ports.history=[goal.copy(),unused_home,via.copy(),start.copy()]
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
        else:
            assert any(np.array_equal(point,via) for point in points)
            assert not any(np.array_equal(point,unused_home) for point in points)
        np.testing.assert_array_equal(points[-1],goal)
    assert checked


@pytest.mark.parametrize('blocked',['none','retract','fold'])
def test_return_can_retract_before_folding_but_certifies_both_segments(blocked):
    from elesim_pilot.mobile_manipulation.runtime import SimPorts,ArmPathBlocked
    ports=SimPorts.__new__(SimPorts)
    ports.arm=NS(bounds=np.array([[-.121,0.],[-1.,1.],[-1.,1.],[-1.,1.]]),uncertainty=np.array([.001,.015,.015,.015]))
    start=np.array([-.074,-.067,-.117,.603]);goal=np.array([-.12,0.,-.593,.593])
    ports.feedback=NS(snapshot=lambda *a:dict(joint=dict(q=start.copy())))
    ports.route=lambda:('sim','boot','lease');ports._summary={};ports.history=[]
    ports.connection=NS(state_sink=NS(get_state=lambda:None))
    ports.service=NS(_ik_context_for_host=lambda h:dict(base_world_T=np.eye(4)))
    checked=[]
    def certify(a,b):
        checked.append((a.copy(),b.copy()))
        retract=(np.allclose(a[1:],b[1:]) and b[0]==goal[0])
        fold=(a[0]==goal[0] and np.array_equal(b,goal))
        accepted=(retract and blocked!='retract') or (fold and blocked!='fold')
        return dict(accepted=accepted,reason='observed_surface_collision_on_path')
    ports.collision=NS(base=None,certify_segment=certify)
    if blocked=='none':
        path=ports._return_prepare(goal)
        np.testing.assert_array_equal(path[0],start)
        np.testing.assert_array_equal(path[1],np.r_[goal[0],start[1:]])
        np.testing.assert_array_equal(path[2],goal)
        assert ports.summary()['arm_return']['strategy']=='retract_first'
    else:
        with pytest.raises(ArmPathBlocked):ports._return_prepare(goal)
    assert checked


def test_return_folds_while_fully_retracted_then_extends_to_the_unchanged_goal():
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    ports=SimPorts.__new__(SimPorts);start=np.array([-.05,.5,0.,0.]);goal=start.copy();goal[1]=-.5
    ports.feedback=NS(snapshot=lambda *a:dict(joint=dict(q=start.copy())))
    ports.arm=NS(bounds=np.array([[-.2,0.],[-1.,1.],[-1.,1.],[-1.,1.]]),uncertainty=np.array([.001,.015,.015,.015]))
    ports.route=lambda:('sim','boot','lease');ports._summary={};ports.history=[]
    ports.connection=NS(state_sink=NS(get_state=lambda:None))
    ports.service=NS(_ik_context_for_host=lambda h:dict(base_world_T=np.eye(4)))
    checked=[]
    def certify(a,b):
        checked.append((a.copy(),b.copy()))
        # A continuous joint-space barrier across roll=0 blocks a direct
        # turn at the requested linear position; it ends below -140 mm.
        if a[1]*b[1]<=0:
            fraction=-a[1]/(b[1]-a[1]) if b[1]!=a[1] else 0.
            return dict(accepted=a[0]+fraction*(b[0]-a[0])<-.14)
        return dict(accepted=True)
    ports.collision=NS(base=None,certify_segment=certify)
    path=np.asarray(ports._return_prepare(goal))
    np.testing.assert_array_equal(path[[0,-1]],[start,goal])
    assert len(path)==4 and np.all(path[1:3,0]==-.199)
    assert ports.summary()['arm_return']['strategy']=='retract_transform_extend'
    assert all(any(np.array_equal(a,x) and np.array_equal(b,y) for x,y in checked)
        for a,b in zip(path,path[1:]))


@pytest.mark.parametrize('live_shortcut_lost',[False,True])
def test_history_return_shortens_and_recertifies_instead_of_replaying_every_bend(live_shortcut_lost):
    from elesim_pilot.mobile_manipulation.runtime import SimPorts
    ports=SimPorts.__new__(SimPorts)
    points=np.array([[0.,0.,0.,0.],[0.,.5,0.,0.],[.2,.5,0.,0.],
        [.4,.5,0.,0.],[.4,0.,0.,0.]])
    start,a,b,c,goal=points
    ports.arm=NS(bounds=np.array([[0.,1.]]*4),uncertainty=np.array([.001,.015,.015,.015]))
    ports.feedback=NS(snapshot=lambda *args:dict(joint=dict(q=start.copy())))
    ports.route=lambda:('sim','boot','lease');ports._summary={};ports.history=list(points[:-1][::-1])
    ports.connection=NS(state_sink=NS(get_state=lambda:None))
    ports.service=NS(_ik_context_for_host=lambda h:dict(base_world_T=np.eye(4)))
    allowed=[(start,a),(a,b),(b,c),(c,goal),(start,b)];checked=[]
    def certify(x,y):
        checked.append((x.copy(),y.copy()))
        if live_shortcut_lost and np.array_equal(x,start) and np.array_equal(y,b):
            if sum(np.array_equal(u,start) and np.array_equal(v,b) for u,v in checked)>1:
                return dict(accepted=False)
        ok=np.array_equal(x,y) or any(np.array_equal(x,u) and np.array_equal(y,v) for u,v in allowed)
        return dict(accepted=ok)
    ports.collision=NS(base=None,certify_segment=certify)
    result=ports._return_prepare(goal)
    np.testing.assert_array_equal(result,[start,start,a,b,c,goal] if live_shortcut_lost else [start,b,c,goal])
    # One check authorizes the new edge; a second checks it at fresh feedback.
    assert sum(np.array_equal(x,start) and np.array_equal(y,b) for x,y in checked)>=2
    report=ports.summary()['arm_return']
    assert report['segments']==(5 if live_shortcut_lost else 3)
    assert report['shortening']['original_segments']==5


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
