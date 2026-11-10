"""Shared GA/RL physical lift contract; NumPy fixtures are not GPU evidence."""
import math
import numpy as np
import pytest

from workbench.research.experiments.hug_c import configure as cfg
from workbench.research.experiments.hug_c.metrics import (
    HoldTracker, lift_posture_complete, lift_posture_fraction, lift_roll_error)
from workbench.research.experiments.hug_c.rewards import RewardLedger
from workbench.research.experiments.hug_ga.search import seed_population


def test_measured_roll_must_complete_lift_before_five_second_window():
    settings=cfg.DEFAULTS;tracker=HoldTracker(3,settings)
    # Original winner's final roll, nominal wrap, and fully raised pose.
    actual=np.array([[0.,-.96509099,-.27,-.58],
                     [0.,-math.pi/2,-.27,-.58], [0.,0.,-.27,-.58]])
    arguments=(np.full(3,.049),np.zeros(3,bool),np.ones((3,2)),actual,
               np.zeros(3,bool),np.zeros((3,3)))
    for _ in range(499):assert not tracker.update(*arguments).any()
    np.testing.assert_array_equal(tracker.update(*arguments),[False,False,True])
    assert tracker.count.tolist()==[0,0,500]
    # Leaving the tolerance resets the entire window, even with continuing contact.
    actual[2,1]=settings['lift_roll_tolerance_rad']+.001
    assert not tracker.update(*arguments).any()
    assert tracker.count[2]==0 and tracker.interrupted[2]
    actual[2,1]=0
    tracker.update(*arguments)
    assert tracker.count[2]==1


def test_roll_boundary_and_progress_are_symmetric_and_height_is_still_required():
    tolerance=cfg.DEFAULTS['lift_roll_tolerance_rad']
    q=np.tile([0.,0.,-.2,-.4],(7,1))
    q[:,1]=[-math.pi/2,-tolerance-.001,-tolerance,0,tolerance,tolerance+.001,math.pi/2]
    np.testing.assert_array_equal(lift_posture_complete(q,cfg.DEFAULTS),[False,False,True,True,True,False,False])
    progress=lift_posture_fraction(q,cfg.DEFAULTS)
    np.testing.assert_allclose(progress[[0,2,3,4,6]],[0,1,1,1,0])
    np.testing.assert_allclose(lift_roll_error(q,cfg.DEFAULTS),abs(q[:,1]))
    tracker=HoldTracker(7,cfg.DEFAULTS)
    for _ in range(501):
        result=tracker.update(np.zeros(7),np.zeros(7,bool),np.ones((7,2)),q,
                              np.zeros(7,bool),np.zeros((7,3)))
    assert not result.any()


def test_partial_lift_cannot_collect_full_lift_hold_success_or_time_exemption():
    q=np.array([[0.,-.96509099,-.27,-.58],[0.,0.,-.27,-.58]])
    ledger=RewardLedger(2,cfg.DEFAULTS);ledger.reset(np.arange(2),q,q[0])
    _,done,terms=ledger.step(q,q[0],np.zeros((2,4)),confirmed_grasp=np.ones(2,bool),
        lift_fraction=np.ones(2),hold_fraction=np.ones(2),wrong_fraction=np.zeros(2),
        fault=np.zeros(2,bool),success=np.ones(2,bool),timeout=np.zeros(2,bool),drops=np.zeros(2))
    assert 0<terms['lift'][0]<5 and terms['lift'][1]==5
    np.testing.assert_array_equal(terms['hold'],[0,10])
    np.testing.assert_array_equal(terms['success'],[0,30])
    np.testing.assert_array_equal(done,[False,True])
    assert terms['time'][0]<0 and terms['time'][1]==0


def test_ga_seed_uses_grasp_q_then_raised_roll_without_constraining_search():
    from workbench.research.experiments.hug_c.control import command_bounds
    lo,hi=command_bounds(cfg.DEFAULTS)
    home=np.array([-.12,0.,-.59,.59]);grasp=np.array([0.,-math.pi/2,-.31,-.57])
    raised=grasp.copy();raised[1]=0
    genes=seed_population(16,6,home,grasp,lo,hi,np.random.default_rng(42),lift_goal=raised)
    knots=lo+genes[0,:,:4]*(hi-lo)
    np.testing.assert_allclose(knots[2],grasp)
    np.testing.assert_allclose(knots[-1],raised)
    assert not np.allclose(genes[0],genes[1])


@pytest.mark.parametrize('change',[{'lift_roll_target_rad':-.5},
    {'lift_roll_tolerance_rad':0},{'lift_roll_tolerance_rad':math.radians(90)}])
def test_settings_cannot_silently_relax_full_lift_to_old_partial_lift(change):
    with pytest.raises(ValueError):cfg.validate_settings(dict(cfg.DEFAULTS,**change))


def test_old_partial_lift_experiments_and_policies_are_rejected(tmp_path):
    import json
    from workbench.research.experiments.hug_c.policy import NumpyPolicy
    path=tmp_path/'old.json'
    path.write_text(json.dumps(dict(schema='elesim-c-hug-experiment',schema_version=4,
                                  control_mode='policy-four-dof-home-lift-v3')))
    with pytest.raises(ValueError,match='fresh'):cfg.load_experiment(path)
    policy=tmp_path/'old.npz'
    np.savez_compressed(policy,schema=np.asarray('elesim-c-hug-policy'),schema_version=np.asarray(3))
    with pytest.raises(ValueError,match='policy'):NumpyPolicy(policy)
