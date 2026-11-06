"""Host-only C-hug regressions; synthetic replay flags are not physics evidence."""
import copy
import json
import math
from pathlib import Path
import numpy as np
import pytest
from elesim_sim.rl.trajectory import Trajectory
from workbench.research.experiments.hug_c import configure as cfg
from workbench.research.experiments.hug_c.prepare import prepare
from workbench.research.experiments.hug_c.triangle import RasterArea,select,weights,contains
from workbench.research.experiments.hug_c.metrics import (
    cylinder_clearance, HoldTracker, local_link_index, position_offsets_mm)
from workbench.research.experiments.hug_c.interpolate import interpolate


def pilot(tmp_path):
    vertices=[[-10,-100],[-30,-100],[-10,-120]]
    cases=[dict(case_id=f'a{i}',split='train',q_deg=q,xy_mm=[-100-i*5,300+i*5],mu_min=.2,
                diameter_mm=150,feature='side',root=-1) for i,q in enumerate(vertices)]
    cases.append(dict(cases[0],case_id='v',split='validation',q_deg=[-14,-106],xy_mm=[-110,310]))
    cases.append(dict(cases[0],case_id='t',split='test',q_deg=[-16,-108],xy_mm=[-115,315]))
    data=dict(schema='elesim-c-hug-pilot',schema_version=1,source=dict(kind='mapper-function',point_data_used=False),
              selection={'mu_max':.4},cases=cases,triangle={'branch':'test-chart','anchor_ids':['a0','a1','a2']})
    p=tmp_path/'pilot.json';p.write_text(json.dumps(data));return p


def test_real_home_and_total_bend_conversion(tmp_path):
    exp=cfg.configure(pilot(tmp_path));home=exp['home']
    assert home['ui_controls']==dict(u_linear=180.,u_roll=180.,u_s1=10.,u_s2=10.)
    np.testing.assert_allclose(home['command'],[-.12,0,np.deg2rad(-34),np.deg2rad(34)],atol=1e-10)
    np.testing.assert_allclose(home['total_deg'],[-170,170],atol=1e-10)
    c=exp['cases'][0]
    np.testing.assert_allclose(c['goal_command'][2:],np.deg2rad(np.array(c['q_deg'])/5))
    assert c['start_command'][3]>0  # Home may not be rejected by a target-only sign constraint.
    dest=tmp_path/'experiment.json';cfg.write_json(dest,exp)
    assert cfg.load_experiment(dest)==exp
    exp['cases'][0]['goal_command'][2]+=.01;dest.write_text(json.dumps(exp))
    with pytest.raises(ValueError,match='changed'):cfg.load_experiment(dest)


def test_zero_actions_do_not_force_any_deployment_or_lift(tmp_path):
    from workbench.research.experiments.hug_c.control import advance_command
    exp=cfg.configure(pilot(tmp_path));c=exp['cases'][0];s=exp['settings']
    command=np.array([c['start_command']])
    for _ in range(s['episode_steps']):command=advance_command(command,np.zeros((1,4)),s)
    np.testing.assert_array_equal(command[0],c['start_command'])
    actions=np.array([[1,0,0,0]])
    moved=advance_command(command,actions,s)
    assert moved[0,0]>command[0,0]
    np.testing.assert_array_equal(moved[0,1:],command[0,1:])
    actions=np.array([[0,-1,0,1]])
    moved=advance_command(command,actions,s)
    assert moved[0,1]<command[0,1]
    assert moved[0,3]>command[0,3]
    np.testing.assert_array_equal(moved[0,[0,2]],command[0,[0,2]])
    cfg.validate_command(c,s,0,c['start_command'])
    bad=command[0].copy();bad[1]=-.1
    with pytest.raises(ValueError,match='Home'):cfg.validate_command(c,s,0,bad)
    bad=command[0].copy();bad[0]=-1
    with pytest.raises(ValueError,match='bounds'):cfg.validate_command(c,s,1,bad)



def test_cylinder_bottom_and_five_second_hold_require_actual_lifting():
    q=np.array([[1.,0,0,0],[math.sqrt(.5),math.sqrt(.5),0,0]])
    clear=cylinder_clearance(np.array([[0,0,.55],[0,0,.55]]),q,.075,1.1)
    np.testing.assert_allclose(clear,[0,.475],atol=1e-8)
    s=cfg.DEFAULTS;h=HoldTracker(4,s)
    actual=np.tile([-.05,-.7,-.2,-.4],(4,1))  # final linear and roll are free, not fixed 0.
    actual[3,3]=.2
    clearance=np.array([.03,0.,.03,.03]);floor=np.array([False,True,False,False]);fault=np.array([False,False,True,False])
    touch=np.ones((4,2));relative=np.zeros((4,3))
    for _ in range(100):result=h.update(clearance,floor,touch,actual,fault,relative)
    assert not result.any()  # One second cannot pass.
    for _ in range(399):result=h.update(clearance,floor,touch,actual,fault,relative)
    assert not result.any()
    result=h.update(clearance,floor,touch,actual,fault,relative)
    np.testing.assert_array_equal(result,[True,False,False,False])
    clearance[0]=0
    assert not h.update(clearance,floor,touch,actual,fault,relative).any()
    assert h.count[0]==0  # A drop resets the continuous window.



def test_gripper_relative_pose_uses_local_not_contact_link_index():
    from types import SimpleNamespace
    robot=SimpleNamespace(links=[SimpleNamespace(name='gripper_base',idx=25,idx_local=24)])
    assert local_link_index(robot,'gripper_base')==24
    with pytest.raises(ValueError):local_link_index(robot,'unknown')


def test_position_error_sweep_includes_cardinal_and_diagonal_offsets_once():
    offsets=position_offsets_mm([0,5,10,5])
    assert len(offsets)==17
    assert offsets[0]==[0,0]
    assert [5,0] in offsets and [-10,10] in offsets
    assert len(position_offsets_mm([5,10]))==16
    with pytest.raises(ValueError):position_offsets_mm([-1])


def test_fractional_area_counts_intersection_and_allows_interior_hole():
    p=np.array([(x,y) for x in np.arange(-3.5,0,1) for y in np.arange(-3.5,0,1)])
    triangle=np.array([[-4,-4],[0,-4],[-4,0]])
    full=RasterArea(p,1)
    assert full.covered(triangle)==pytest.approx(8)
    hole=p[~np.all(p==[-3.5,-3.5],axis=1)]
    assert RasterArea(hole,1).covered(triangle)==pytest.approx(7)
    rows=[dict(q_deg=q.tolist(),mu_min=.2) for q in hole]
    ids,report=select(rows,1)
    assert len(set(ids))==3 and 0<report['coverage']<=1
    assert report['covered_area_deg2']<=report['triangle_area_deg2']+1e-8
    assert report['method']=='exhaustive-raster'
    assert not report['continuous_global_optimum_proven']


def test_q_interpolation_blends_all_four_free_commands(tmp_path):
    exp=cfg.configure(pilot(tmp_path));s=exp['settings']
    anchors=[fake_trace(c,exp['simulation_context'],duration=12+i) for i,c in enumerate(exp['cases'][:3])]
    target=dict(exp['cases'][3],simulation_context=exp['simulation_context'],duration_s=14.,
                control_mode=cfg.CONTROL_MODE,xy_mm=[-400,0])
    result=interpolate(anchors,target)
    np.testing.assert_allclose(result.metadata['weights'],[.5,.2,.3])
    np.testing.assert_allclose(result.commands[0],target['start_command'])
    expected=sum(w*a.sample(result.times/14*a.times[-1]) for w,a in zip([.5,.2,.3],anchors))
    np.testing.assert_allclose(result.commands,expected)
    assert result.commands[-1,0]!=0  # No forced final deployment.
    assert result.commands[-1,1]!=0  # No forced final lift-roll target.
    assert not result.diagnostics['physics_validated']
    target['q_deg']=[-100,-100]
    with pytest.raises(ValueError,match='extrapolation'):interpolate(anchors,target)
    anchors[0].diagnostics['replay_success']=False
    with pytest.raises(ValueError,match='Replay'):interpolate(anchors,dict(target,q_deg=[-14,-106]))



def test_prepare_executes_functions_not_embedded_point_data(tmp_path):
    # Deliberately synthetic geometry: verifies routing/splits, not physical grasp quality.
    html='''const DATA=(()=>{throw Error("must not read stored points")})();
const rot=()=>{};
function model(q3,q4){if(q3<-25||q4<-25)return {candidates:[]};return {candidates:[{x:q3,y:200-q4,mu:.2,kind:'side',branch:-1}]};}
let current=[];
const CONTACT_OVERLAP_TOL_MM=1;
function collisionGeometry(){return [];}
function selfCollision(){return false;}
function objectCollision(){return false;}
function muLimit(){}'''
    path=tmp_path/'mapper.html';path.write_text(html)
    result=prepare(path,step=10,validation=3,test=4)
    assert not result['source']['point_data_used']
    assert len(result['cases'])==10
    assert [c['split'] for c in result['cases']].count('train')==3
    assert len({c['case_id'] for c in result['cases']})==10
    assert all(c['q_deg'][0]*c['q_deg'][1]>0 and c['mu_min']<=.4 for c in result['cases'])
    assert all(contains(np.array(result['selection']['vertices_total_deg']),np.array([c['q_deg']]))[0] for c in result['cases'])
    # A physical screen limits the selectable raster cells and held-out chart.
    ready=[[-25.,-25.],[-15.,-25.],[-25.,-15.]]
    screening=dict(schema='elesim-c-hug-catalog-preflight',source=result['source'],
                   diameter_mm=150.,step_deg=10.,mu_max=.4,
                   simulation_context=dict(model_sha256=cfg.asset_digest(),settings=cfg.DEFAULTS),
                   results=[dict(q_deg=q,static_ready=True) for q in ready])
    report=tmp_path/'screening.json';report.write_text(json.dumps(screening))
    screened=prepare(path,step=10,validation=3,test=4,catalog_preflight=report)
    assert screened['selection']['feasible_samples']==3
    assert screened['selection']['static_catalog_screened']
    assert {tuple(c['q_deg']) for c in screened['cases'][:3]}==set(map(tuple,ready))
    generated=tmp_path/'screened.json';generated.write_text(json.dumps(screened))
    cfg.configure(generated)
    with pytest.raises(ValueError,match='screened catalog'):
        cfg.configure(generated,{'mass_kg':.5})
    screening['diameter_mm']=100;report.write_text(json.dumps(screening))
    with pytest.raises(ValueError,match='Catalog preflight'):
        prepare(path,step=10,validation=3,test=4,catalog_preflight=report)


@pytest.mark.parametrize('change',[{'unknown':1},{'episode_steps':.5},{'hold_duration_s':0},{'friction':.3},{'height_m':.1},
                                   {'linear_min_m':-.1},{'bend_per_node_rad':.3},{'roll_min_rad':0.}])
def test_invalid_contract_is_rejected(tmp_path,change):
    with pytest.raises(ValueError):cfg.configure(pilot(tmp_path),change)


def test_numpy_policy_uses_new_observation_contract(tmp_path):
    from workbench.research.experiments.hug_c.policy import NumpyPolicy
    n=len(cfg.OBSERVATIONS);p=tmp_path/'policy.npz'
    arrays=dict(schema=np.asarray('elesim-c-hug-policy'),schema_version=np.asarray(2),action_names=np.asarray(cfg.ACTIONS),
                observation_names=np.asarray(cfg.OBSERVATIONS),metadata_json=np.asarray('{}'),
                norm_mean=np.zeros(n,np.float32),norm_std=np.ones(n,np.float32),norm_eps=np.asarray(1e-8),
                n_layers=np.asarray(1),w0=np.ones((4,n),np.float32),b0=np.zeros(4,np.float32))
    np.savez_compressed(p,**arrays);policy=NumpyPolicy(p)
    np.testing.assert_allclose(policy(np.ones((3,n),np.float32)),n*np.ones((3,4)),atol=1e-5)
    with pytest.raises(ValueError):policy(np.ones((1,7)))
    arrays['schema']=np.asarray('elesim-hug-policy');np.savez_compressed(p,**arrays)
    with pytest.raises(ValueError):NumpyPolicy(p)


def test_offline_cli_creates_only_q_interpolation_candidates(tmp_path):
    from workbench.research.experiments.hug_c.experiment import main
    exp_path=tmp_path/'experiment.json'
    assert main(['configure','--pilot',str(pilot(tmp_path)),'--output',str(exp_path)])==0
    exp=cfg.load_experiment(exp_path);anchors=tmp_path/'anchors'
    for c in exp['cases'][:3]:fake_trace(c,exp['simulation_context']).save(anchors/(c['case_id']+'.npz'))
    out=tmp_path/'candidates'
    assert main(['interpolate','--experiment',str(exp_path),'--anchors',str(anchors),'--output',str(out),'--split','validation'])==0
    traces=[Trajectory.load(p) for p in out.glob('*.npz')]
    assert len(traces)==1 and traces[0].metadata['case_id']=='v'
    assert not traces[0].diagnostics['physics_validated']



@pytest.mark.parametrize('passes',[True,False])
def test_study_trains_only_three_anchors_and_stops_on_failure(tmp_path,passes):
    from types import SimpleNamespace
    from workbench.research.experiments.hug_c.study import Study
    calls=[]
    class StubStudy(Study):
        # These are orchestration fixtures, not simulations or trained models.
        def command(self,stage,*arguments,allow_failure=False):
            calls.append((stage,arguments))
            if stage=='configure':cfg.write_json(self.exp_path,cfg.configure(self.args.pilot))
            return 0
        def evaluate(self,run,directory):
            directory.mkdir(parents=True)
            if passes:
                (directory/'verified.npz').write_bytes(b'unit-test copy fixture')
                (directory/'policy.npz').write_bytes(b'unit-test policy copy fixture')
            return dict(success=passes,directory=str(directory))
    args=SimpleNamespace(output=tmp_path/'study',pilot=pilot(tmp_path),settings=None,num_envs=256,iterations=2000,report_every=500)
    study=StubStudy(args);result=study.run()
    trains=[a for stage,a in calls if stage=='anchor-train']
    if passes:
        assert result==0 and len(trains)==3
        assert {a[a.index('--case')+1] for a in trains}=={'a0','a1','a2'}
    else:
        assert result==1 and len(trains)==4
        assert all(a[a.index('--case')+1]=='a0' for a in trains)
        assert not any(stage=='interpolate-validation' for stage,_ in calls)


def test_training_requires_real_matching_preflight_before_gpu_import(tmp_path):
    from workbench.research.experiments.hug_c.experiment import main
    exp=cfg.configure(pilot(tmp_path));path=tmp_path/'experiment.json';cfg.write_json(path,exp)
    report=tmp_path/'preflight.json';report.write_text(json.dumps(dict(ready=False,physics_tested=False)))
    with pytest.raises(ValueError,match='preflight'):
        main(['train','--experiment',str(path),'--preflight',str(report),'--all','--runs',str(tmp_path/'runs')])
    assert not (tmp_path/'runs').exists()


def test_interpolation_requires_three_original_replays(tmp_path):
    exp=cfg.configure(pilot(tmp_path))
    traces=[fake_trace(c,exp['simulation_context']) for c in exp['cases'][:3]]
    for a in traces:a.diagnostics['replay_repetitions']=1
    target=dict(exp['cases'][3],simulation_context=exp['simulation_context'],duration_s=12.,control_mode=cfg.CONTROL_MODE)
    with pytest.raises(ValueError,match='Replay'):interpolate(traces,target)



def fake_trace(c,ctx,duration=12.):
    # Synthetic replay flags exercise interpolation/routing, not physics success.
    s=ctx['settings'];t=np.arange(round(duration/s['dt'])+1)*s['dt']
    start=np.array(c['start_command']);end=np.array([-.04,-.5,*c['goal_command'][2:]])
    alpha=np.clip(t/(duration-5),0,1)
    commands=start+(end-start)*alpha[:,None]
    metadata=dict(c,simulation_context=ctx,duration_s=duration,control_mode=cfg.CONTROL_MODE)
    return Trajectory(t,commands,cfg.CHANNELS,metadata,dict(replay_success=True,physics_validated=True,replay_repetitions=3))


def test_hold_window_restarts_after_slip_and_requires_loaded_contact():
    s=cfg.DEFAULTS;h=HoldTracker(2,s)
    q=np.tile([0.,0.,-.2,-.4],(2,1));touch=np.array([[1.,1.],[0.,0.]])
    c=np.full(2,.03);floor=np.zeros(2,bool);fault=np.zeros(2,bool);relative=np.zeros((2,3))
    for _ in range(400):assert not h.update(c,floor,touch,q,fault,relative).any()
    relative[0,0]=.02
    h.update(c,floor,touch,q,fault,relative)
    assert h.count[0]==1
    for _ in range(499):result=h.update(c,floor,touch,q,fault,relative)
    np.testing.assert_array_equal(result,[True,False])


def test_function_goal_remains_a_hint_without_a_command_schedule(tmp_path):
    from workbench.research.experiments.hug_c.control import pose_error,advance_command
    exp=cfg.configure(pilot(tmp_path));s=exp['settings'];c=exp['cases'][0]
    actual=np.array([c['start_command']]);goal=np.array(c['goal_command'])
    nearer=actual+.1*(goal-actual)
    assert pose_error(nearer,goal,s)[0]<pose_error(actual,goal,s)[0]
    assert 'goal_q3_total_rad' in cfg.OBSERVATIONS and 'goal_q4_total_rad' in cfg.OBSERVATIONS
    assert cfg.DEFAULT_NUM_ENVS==512 and cfg.ACTIONS==('linear','roll','q3_total','q4_total')


def test_hold_contact_coverage_and_floor_contact_are_independent_gates():
    h=HoldTracker(3,cfg.DEFAULTS)
    q=np.tile([-.04,-.6,-.2,-.4],(3,1));clear=np.full(3,.03)
    floor=np.array([False,False,True]);fault=np.zeros(3,bool);relative=np.zeros((3,3))
    for i in range(500):
        touch=np.array([[i<400,i<400],[i<399,i<399],[True,True]])
        result=h.update(clear,floor,touch,q,fault,relative)
    np.testing.assert_array_equal(result,[True,False,False])


def test_old_scripted_schema_and_context_action_changes_are_rejected(tmp_path):
    exp=cfg.configure(pilot(tmp_path));p=tmp_path/'experiment.json'
    exp['schema_version']=1;p.write_text(json.dumps(exp))
    with pytest.raises(ValueError,match='fresh four-DoF'):cfg.load_experiment(p)
    exp['schema_version']=2;exp['simulation_context']['action_names']=['q3','q4']
    p.write_text(json.dumps(exp))
    with pytest.raises(ValueError,match='changed'):cfg.load_experiment(p)
