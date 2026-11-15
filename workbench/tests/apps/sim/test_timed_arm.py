"""Simulation-time arm replay, heartbeat and lease revocation tests."""
import pytest
from elesim_protocol import SimMappingConfig,SimQ
from elesim_protocol.timed_arm import validate
from elesim_sim.control_state import SimulationStateSource


def rig():
    now=[0.];s=SimulationStateSource(SimMappingConfig(),clock=lambda:now[0]);home=SimQ(-.12,0.,-.59,.59)
    s.set_physics_profile(dict(arm_control_mode='position'))
    s.seed_estimate_q(home);s.apply_command(dict(command='torque_on'))
    commands=[[home.linear_m,home.roll_rad,home.theta1_rad,home.theta2_rad],[-.1199,0.,-.59,.59],[-.1198,0.,-.59,.59]]
    start=dict(command='target',timed_arm=dict(id='test',operation='start',dt_s=.01,q=commands))
    return s,home,start,now


def test_uses_simulation_time_not_network_poll_rate():
    s,home,start,now=rig();s.apply_command(start)
    s.advance_sim_time(10.,home,.01);assert s.estimate_q()==home
    now[0]=.1;s.advance_sim_time(10.01,home,.01);assert s.estimate_q().linear_m==pytest.approx(-.1199)
    now[0]=.2;s.advance_sim_time(10.02,s.estimate_q(),.01)
    assert s.timed_arm_status()['state']=='completed'
    assert s.estimate_q().linear_m==pytest.approx(-.1198)


@pytest.mark.parametrize('failure',['heartbeat','lease','torque','dt','start'])
def test_interruptions_hold_and_never_resume_old_path(failure):
    s,home,start,now=rig();s.apply_command(start)
    if failure=='heartbeat':now[0]=1.
    elif failure=='lease':s.revoke_control()
    elif failure=='torque':s.apply_command(dict(command='torque_off'))
    elif failure=='start':home=SimQ(-.1,0.,0.,0.)
    s.advance_sim_time(10.,home,.02 if failure=='dt' else .01)
    assert s.timed_arm_status()['state']=='cancelled'
    s.advance_sim_time(20.,home,.01)
    assert s.timed_arm_status()['state']=='cancelled'


def test_target_cannot_overwrite_running_timed_path_or_forge_heartbeat():
    s,home,start,now=rig();s.apply_command(start)
    with pytest.raises(ValueError):s.apply_command(dict(command='target',q=[0,0,0,0]))
    with pytest.raises(ValueError):s.apply_command(dict(command='target',timed_arm=dict(id='wrong',operation='keepalive')))
    s.apply_command(dict(command='target',timed_arm=dict(id='test',operation='cancel')))
    assert s.timed_arm_status()['state']=='cancelled'


@pytest.mark.parametrize('failure', ['clock_jump', 'invalid_joint', 'invalid_clock'])
def test_invalid_physics_state_cannot_skip_to_a_finished_path(failure):
    s,home,start,_=rig();s.apply_command(start);s.advance_sim_time(1.,home,.01)
    assert s.manipulation_evaluation_due()
    stamp=1.02 if failure=='clock_jump' else float('nan') if failure=='invalid_clock' else 1.01
    measured=SimQ(home.linear_m,0.,float('nan'),home.theta2_rad) if failure=='invalid_joint' else home
    s.advance_sim_time(stamp,measured,.01)
    assert s.timed_arm_status()['state']=='cancelled'
    assert s.timed_arm_status()['tick']==0


def test_evaluation_counts_each_physics_tick_once_and_resets_for_a_new_execution():
    import numpy as np
    from types import SimpleNamespace as NS
    # No-contact fixture: hold time can accrue, but grasp success must not.
    names=['node0','node5','node9','camera','gripper_base']
    links=[NS(name=name,idx=i,idx_local=i) for i,name in enumerate(names)]
    positions=np.zeros((5,3));positions[3,2]=.05
    quats=np.tile([1.,0.,0.,0.],(5,1))
    robot=NS(links=links,get_links_pos=lambda:positions,get_links_quat=lambda:quats)
    contacts=dict(valid_mask=np.zeros(0,bool),link_a=np.zeros(0,int),link_b=np.zeros(0,int),
        normal=np.zeros((0,3)),position=np.zeros((0,3)),force_a=np.zeros((0,3)))
    joint_names=['j_plate_housing','j_housing_wedge','j_wedge_node0']+[f'j_node{i}_node{i+1}' for i in range(9)]
    robot.get_contacts=lambda **kw:contacts
    robot.get_joint=lambda name:NS(dofs_idx_local=joint_names.index(name))
    robot.get_dofs_position=lambda **kw:np.array([0.,0.]+[-.3]*5+[-.5]*5)
    obj=NS(links=[NS(idx=20)],get_contacts=lambda **kw:contacts,
        get_pos=lambda:np.array([0.,0.,.6]),get_quat=lambda:np.array([1.,0.,0.,0.]))
    s,home,start,_=rig();s.apply_command(start);s.advance_sim_time(1.,home,.01)
    actual=[0.,0.,-.3,-.5]
    # End-state linear command is zero for the physical hold criterion.
    s.seed_estimate_q(SimQ(*actual))
    for _ in range(600):s.sample_manipulation_evaluation(robot,obj,actual,.01)
    report=s.manipulation_evaluation_status()
    # This is DDS telemetry and persisted evidence, not a NumPy-only result.
    import json
    json.dumps(report)
    assert report['tick']==0 and report['hold_s']==pytest.approx(.01)
    assert not report['success']
    assert not s.manipulation_evaluation_due()
    s.apply_command(dict(command='target',timed_arm=dict(id='test',operation='cancel')))
    s.seed_estimate_q(home);s.apply_command(start)
    assert s.manipulation_evaluation_status()=={}
    s.advance_sim_time(2.,home,.01);s.seed_estimate_q(SimQ(*actual))
    s.sample_manipulation_evaluation(robot,obj,actual,.01)
    assert s.manipulation_evaluation_status()['hold_s']==pytest.approx(.01)


def test_structural_contract_bounded_and_finite():
    _,_,start,_=rig();raw=start['timed_arm'];validate(raw)
    raw['q'][0][0]=float('nan')
    with pytest.raises(ValueError):validate(raw)
    with pytest.raises(ValueError):validate(dict(id='a',operation='start',dt_s=.01,q=[[0,0,0,0]]*2402))


def test_timed_path_refuses_nonphysical_sim_and_profile_change_cancels():
    s,_,start,_=rig();s.set_physics_profile(dict(arm_control_mode='kinematic'))
    with pytest.raises(ValueError,match='physical position'):s.apply_command(start)
    s.set_physics_profile(dict(arm_control_mode='position'));s.apply_command(start)
    s.set_physics_profile(dict(arm_control_mode='position',substeps=2))
    assert s.timed_arm_status()['reason']=='physics_profile_changed'


def test_path_refuses_a_lower_actuator_rate_instead_of_distorting_its_geometry():
    s,_,start,_=rig()
    s.set_physics_profile(dict(arm_control_mode='position',command_slew_per_s=[.05,1.,.28,.28]))
    start['timed_arm']['q'][1][2]+=.0029
    with pytest.raises(ValueError,match='actuator slew'):s.apply_command(start)


def test_physical_success_requires_full_loaded_hold_and_restarts_for_slip():
    import numpy as np
    from elesim_sim.simulation.hold_evaluation import HoldEvaluation
    h=HoldEvaluation();h.reset('path')
    args=dict(dt=.01,clearance=.03,floor=False,touch=[True,True],proper=True,
        actual=[0.,0.,-.3,-.5],command=[0.,0.,-.3,-.5],relative=np.zeros(3))
    for _ in range(450):result=h.update(**args)
    assert not result['success']
    args['relative']=np.array([.011,0,0])
    h.update(**args)
    for _ in range(498):result=h.update(**args)
    assert not result['success']
    result=h.update(**args);assert result['success']
    result=h.update(**dict(args,floor=True));assert not result['success']
    for _ in range(500):result=h.update(**args)
    assert result['success']
    result=h.update(**dict(args,forbidden=True));assert not result['success']


def test_wrong_face_restarts_hold_but_physical_collision_remains_a_fault():
    import numpy as np
    from elesim_sim.simulation.hold_evaluation import HoldEvaluation
    h=HoldEvaluation();h.reset('contact-parity')
    args=dict(dt=.01,clearance=.03,floor=False,touch=[True,True],proper=True,
        actual=[0.,0.,-.3,-.5],command=[0.,0.,-.3,-.5],relative=np.zeros(3))
    h.update(**dict(args,forbidden=True))
    for _ in range(500):result=h.update(**args)
    assert result['success'] and not result['physical_fault']
    h.update(**dict(args,physical_fault=True))
    for _ in range(600):result=h.update(**args)
    assert not result['success'] and result['physical_fault']
    h.reset('next-execution')
    for _ in range(500):result=h.update(**args)
    assert result['success'] and not result['physical_fault']
