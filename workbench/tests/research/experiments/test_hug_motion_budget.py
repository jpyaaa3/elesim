"""Motor-command total variation gates; no reward or net-displacement cost."""
import numpy as np
import pytest
from elesim_protocol.messages import SimQ,sim_q_to_motor_deg
from workbench.research.experiments.hug_reverse_ga.motion_budget import (
    SCALE,LIMITS_DEG,increment,grouped,exceeded,require_feasible)
from workbench.research.experiments.hug_reverse_ga.criteria import Tracker


def test_matches_existing_motor_mapping():
    a=np.array([-.12,0,-.59,.59]);b=np.array([-.05,-.7,-.3,-.5])
    motor=lambda q:np.array(list(vars(sim_q_to_motor_deg(SimQ(*q))).values()))
    np.testing.assert_allclose(increment(a,b),abs(motor(a)-motor(b)))
    assert increment([0,0,0,0],[0,-np.pi/2,0,0])[1]==pytest.approx(180)


def test_inclusive_group_limits_and_no_total_limit():
    np.testing.assert_array_equal(LIMITS_DEG,[200,220,600])
    assert not exceeded([200,220,300,300]).any()
    assert exceeded([200.001,220,300,300]).tolist()==[True,False,False]
    assert exceeded([200,220.001,300,300]).tolist()==[False,True,False]
    assert exceeded([200,220,300,300.001]).tolist()==[False,False,True]
    assert not exceeded([200,220,0,600]).any()


def test_previous_near_home_candidate_passes_new_limits_at_old_rejection_tick():
    # Its 500.19-degree bend total exceeded the previous 500-degree limit.
    travel=[182.293628788,180.977378234,169.838961671,330.355308179]
    assert grouped(travel)[2]==pytest.approx(500.19426985)
    assert not exceeded(travel).any()


@pytest.mark.parametrize('bends,minimum',[
    ((-91.5,-157.5),406.),((-64.5,-166.5),442.),((-10.5,-169.5),499.),
])
def test_three_existing_case_endpoints_fit_budget(bends,minimum):
    home=np.array([-.12,0,*np.deg2rad([-34,34])])
    goal=np.array([0,-np.pi/2,*np.deg2rad(np.asarray(bends)/5)])
    require_feasible(home,goal)
    np.testing.assert_allclose(grouped(increment(home,goal)),[180,180,minimum])


def test_round_trip_counts_and_bends_do_not_cancel():
    path=np.array([[0,0,0,0],[0,0,100,-150],[0,0,0,0]])/SCALE
    travel=increment(path[1:],path[:-1]).sum(0)
    np.testing.assert_allclose(grouped(travel),[0,0,500])


def test_feasibility_before_genesis_import():
    require_feasible([-.12,0,-np.deg2rad(34),np.deg2rad(34)],
                     [0,-np.pi/2,-np.deg2rad(2.1),-np.deg2rad(33.9)])
    with pytest.raises(ValueError,match='roll: minimum 360.000 > limit 220.000'):
        require_feasible([0,-np.pi/2,0,0],[0,np.pi/2,0,0])


def test_budget_disqualifies_but_keeps_evaluating_until_a_hard_stop():
    home=np.zeros(4);t=Tracker(1,[0.],.01)
    s=dict(actual=np.zeros((1,4)),actual_vel=np.zeros((1,4)),
           object_pos=np.zeros((1,3)),object_quat=np.array([[1.,0,0,0]]),
           object_vel=np.zeros((1,3)),object_ang=np.zeros((1,3)),floor=np.ones(1,bool),
           fault=np.zeros(1,bool),wrong=np.zeros(1,bool),contact_force=np.ones(1),
           contact_penetration=np.zeros(1))
    prev=np.zeros((1,4));lo=np.full(4,-10.);hi=np.full(4,10.);rate=np.ones(4)
    for k in range(3):
        cmd=prev.copy();cmd[0,1]+=80/SCALE[1]
        t.update(s,cmd,prev,home,lo,hi,rate,[0,0,0]);prev=cmd
    assert t.reports()[0]['reasons']==['roll_travel']
    assert not t.success[0] and t.active[0] and t.rejected[0]
    assert t.reports()[0]['first_violation_tick']==3
    cmd=prev.copy();cmd[0,1]+=80/SCALE[1]
    t.update(s,cmd,prev,home,lo,hi,rate,[0,0,0]);prev=cmd
    r=t.reports()[0]
    assert r['motion_group_motor_deg'][1]==pytest.approx(320)
    assert r['motion_excess_motor_deg']==pytest.approx([0,100,0])
    assert r['motion_excess_max_ratio']==pytest.approx(100/220)
    assert r['first_violation_tick']==3 and not r['hard_stopped']
    s['fault'][0]=True
    t.update(s,prev,prev,home,lo,hi,rate,[0,0,0])
    assert not t.active[0] and t.hard_stopped[0]
    assert t.reports()[0]['reasons']==['physical_fault','roll_travel']
    assert t.reports()[0]['first_hard_violation_tick']==5
    before=t.reports()
    t.update(s,prev+1,prev,home,lo,hi,rate,[0,0,0])
    assert t.reports()==before


@pytest.mark.parametrize("sign",[1,-1])
@pytest.mark.parametrize("steps,rejected", [([80,0,-1],True),([90,90,-10,10],False),([90,89.9,-.1],True)])
def test_initial_roll_direction_lock(sign,steps,rejected):
    home=np.zeros(4);t=Tracker(1,[0.],.01)
    s=dict(actual=np.zeros((1,4)),actual_vel=np.zeros((1,4)),
           object_pos=np.zeros((1,3)),object_quat=np.array([[1.,0,0,0]]),
           object_vel=np.zeros((1,3)),object_ang=np.zeros((1,3)),floor=np.ones(1,bool),
           fault=np.zeros(1,bool),wrong=np.zeros(1,bool),contact_force=np.ones(1),
           contact_penetration=np.zeros(1))
    prev=np.zeros((1,4));lo=np.full(4,-10.);hi=np.full(4,10.);rate=np.ones(4)
    for step in steps:
        cmd=prev.copy();cmd[0,1]+=sign*step/SCALE[1]
        t.update(s,cmd,prev,home,lo,hi,rate,[0,0,0]);prev=cmd
    assert bool(t.rejected[0]) == rejected
    assert t.reports()[0]["reasons"] == (["initial_roll_reversal"] if rejected else [])
