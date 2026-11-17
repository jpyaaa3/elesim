"""Walk normally, finish leg leveling, then admit the measured arrival."""
from types import SimpleNamespace as NS
import numpy as np
import pytest

from elesim_pilot.mobile_manipulation.base_executor import BaseExecutor
from elesim_pilot.mobile_manipulation.contracts import BaseGoal,Settings
from elesim_pilot.mobile_manipulation.feedback import Feedback
from elesim_pilot.mobile_manipulation.runtime import SimPorts


def executor():
    now=[0.]
    e=BaseExecutor(Settings(),lambda:now[0])
    goal=BaseGoal(np.zeros(3),np.zeros((1,3)),np.zeros(4),dict(kind='grasp',target_reach_m=.5))
    e.start(goal,[.2,0,0],1)
    return e,now


def step(e,pose,*,ready=False,speed=0.,free=True):
    return e.step(pose,speed,0.,lambda *a:free,posture_ready=ready)


@pytest.mark.parametrize('fault',['','MPC fault'])
def test_normal_walking_tilt_is_not_a_task_failure_but_controller_fault_is(fault):
    p=SimPorts.__new__(SimPorts)
    p.connection=NS(state_sink=NS(get_state=lambda:NS(connected=True,torque_enabled=True,
        claw_closed=False,safety_fault='',reply_ok=True)))
    p.simulation_status=lambda:NS(paused=False)
    p.route=lambda:('sim','boot','lease');p.s=Settings();p.model=None
    base=dict(rpy=np.array([.12,-.10,0.]))
    p.feedback=NS(snapshot=lambda *a:dict(base=base),bases=[base])
    p._latest={'go2_controller_fault':fault}
    if fault:
        with pytest.raises(RuntimeError,match='MPC fault'):p.guard()
    else:p.guard()


def test_stop_waits_for_leveling_without_reissuing_walk_during_body_shift():
    e,now=executor()
    v,done=step(e,[.006,0,0],speed=.08)
    assert not v.any() and not done and e.stage=='leveling'
    # Measured pose moves while the legs level the body. No new base motion
    # may compete with that position-control transition.
    now[0]=2.
    v,done=step(e,[.02,0,0],ready=False,free=False)
    assert not v.any() and not done and e.stage=='leveling'
    now[0]=3.;step(e,[.002,0,0],ready=True)
    now[0]=3.6;v,done=step(e,[.002,0,0],ready=True)
    assert not v.any() and done and e.stage=='completed'


@pytest.mark.parametrize('pose',[[.006,0,0],[0,0,.01]])
def test_after_leveling_actual_grasp_error_still_requires_a_checked_correction(pose):
    e,now=executor()
    step(e,pose);step(e,pose,ready=True)
    now[0]=.6;v,done=step(e,pose,ready=True)
    assert not done and not v.any() and e.stage=='approach'
    with pytest.raises(RuntimeError,match='no longer observed free'):
        step(e,pose,ready=True,free=False)
    v,done=step(e,pose,ready=True)
    assert v.any() and not done
    # A correction is not success until another stop and leveling/settling.
    step(e,[.001,0,0]);now[0]=1.;step(e,[.001,0,0],ready=True)
    now[0]=1.6
    assert step(e,[.001,0,0],ready=True)[1]


def test_stationary_leveling_has_a_deadline_and_does_not_pass_on_velocity_alone():
    e,now=executor();step(e,[0,0,0])
    now[0]=20.
    assert not step(e,[0,0,0],ready=False)[1]
    now[0]=91.
    with pytest.raises(RuntimeError,match='timeout'):step(e,[0,0,0],ready=False)


def test_settling_requires_a_stable_measured_position():
    e,now=executor();step(e,[0,0,0]);step(e,[0,0,0],ready=True)
    now[0]=.4;step(e,[.0025,0,0],ready=True)
    now[0]=.6
    assert not step(e,[.0025,0,0],ready=True)[1]
    now[0]=1.
    assert step(e,[.0025,0,0],ready=True)[1]


@pytest.mark.parametrize('stage,tilt,ready',[
    ('walk',0.,False),('to_stand',0.,False),('unknown',0.,False),
    ('stand',.12,False),('stand',.01,True),
])
def test_mobile_passes_actual_stand_and_level_readiness_to_arrival(stage,tilt,ready):
    p=SimPorts.__new__(SimPorts);p.s=Settings();p.clock=lambda:0.;p._summary={}
    p.route=lambda:('sim','boot','lease');home=np.array([-.12,0.,-.59,.59])
    p.model=NS(meta=dict(home=home))
    p.feedback=NS(snapshot=lambda *a:dict(pose=np.zeros(3),joint=dict(q=home),
        base=dict(velocity=np.zeros(3),angular=np.zeros(3),rpy=np.array([tilt,0.,0.]),locomotion_stage=stage)))
    checked=[]
    def sample(*args,**kwargs):checked.append(kwargs['posture_ready']);return np.zeros(3),False
    p.operation=dict(phase='move_grasp',started=0.,grant=None,ownership=None,
        executor=NS(step=sample,stage='leveling'))
    p._send=lambda *a,**kw:None
    p.poll(p.operation)
    assert checked==[ready]


def test_missing_stage_cannot_reuse_a_preceding_standing_claim():
    feedback=Feedback(lambda:0.)
    payload=dict(q=[0.,0.,0.,0.],q_source='simulated',measurement_time_s=1.,measurement_monotonic_s=1.,
        go2_base_pos=[0.,0.,.3],go2_base_rpy=[0.,0.,0.],go2_base_timestamp_s=1.,go2_locomotion_stage='stand')
    feedback.accept(payload,('sim','boot'))
    assert feedback.snapshot(('sim','boot'))['base']['locomotion_stage']=='stand'
    payload.update(measurement_time_s=2.,measurement_monotonic_s=2.,go2_base_timestamp_s=2.)
    payload.pop('go2_locomotion_stage')
    feedback.accept(payload,('sim','boot'))
    assert feedback.snapshot(('sim','boot'))['base']['locomotion_stage']=='unknown'
