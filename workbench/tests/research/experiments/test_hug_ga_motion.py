"""Ground-only motion bounds, airborne exemption and legacy provenance."""
import numpy as np
import pytest

from workbench.research.experiments.hug_ga.constraints import (
    MotionTracker,motion_contract,read_motion_contract)
from workbench.research.experiments.hug_ga.search import quality


def sample(tracker,positions,ticks,*,floor=None,clearance=None):
    n=len(positions)
    tracker.observe(np.asarray(positions),np.asarray(ticks),
        floor_contact=np.ones(n,bool) if floor is None else np.asarray(floor,bool),
        clearance=np.zeros(n) if clearance is None else np.asarray(clearance))


def test_transient_excursion_stays_disqualified_after_returning():
    tracker=MotionTracker([0.,0.,.55],2,motion_contract(50))
    sample(tracker,[[.049,0.,.55],[0.,0.,2.]],[1,1])
    assert not tracker.violated.any()
    sample(tracker,[[.051,0.,.55],[0.,0.,3.]],[2,2])
    sample(tracker,[[0.,0.,.55],[0.,0.,4.]],[3,3])
    np.testing.assert_array_equal(tracker.violated,[True,False])
    np.testing.assert_allclose(tracker.peak_mm,[51,0],atol=1e-5)
    np.testing.assert_array_equal(tracker.first_tick,[2,-1])


def test_displacement_is_from_start_not_path_length_and_masks_finished_lanes():
    tracker=MotionTracker([1.,2.,.55],2,motion_contract(50))
    for tick,x in enumerate((.02,-.02,.02,-.02)):
        sample(tracker,[[1+x,2.,.55],[1.,2.,.55]],[tick,tick])
    assert not tracker.violated.any()  # Travel exceeds50mm but displacement does not.
    tracker.active[0]=False
    sample(tracker,[[2.,2.,.55],[1.,2.1,.55]],[5,5])
    np.testing.assert_array_equal(tracker.violated,[False,True])
    np.testing.assert_allclose(tracker.peak_mm,[20,100],atol=1e-3)


def test_boundary_and_disabled_bound():
    tracker=MotionTracker([0.,0.,0.],1,motion_contract(50))
    sample(tracker,[[.05,0.,0.]],[1])
    assert not tracker.violated[0]
    sample(tracker,[[.0501,0.,0.]],[2])
    assert tracker.violated[0]
    tracker=MotionTracker([0.,0.,0.],1,motion_contract())
    sample(tracker,[[2.,0.,0.]],[1])
    assert not tracker.violated[0] and tracker.peak_mm[0]==2000
    assert read_motion_contract({})==motion_contract()


@pytest.mark.parametrize('limit',[0,-1,float('nan'),float('inf'),True])
def test_invalid_limits_rejected(limit):
    with pytest.raises(ValueError):motion_contract(limit)


def test_wrong_metric_cannot_silently_pass_as_verified_constraint():
    contract=motion_contract(50);contract['metric']='xyz-distance'
    with pytest.raises(ValueError):read_motion_contract(dict(constraints=contract))


def test_violation_cannot_be_rescued_by_success_or_progress():
    report=dict(fault=False,wrong_face_any=False,success=False,hold_fraction=0.,
                lift_fraction=0.,grasp_fraction=0.,terminal_distance_m=1.,duration_s=20.)
    illegal=dict(report,success=True,hold_fraction=1.,lift_fraction=1.,grasp_fraction=1.,
                 terminal_distance_m=0.,object_motion_violation=True)
    assert quality(illegal)<quality(report)


def test_airborne_translation_exempt_but_separate_peaks_preserved():
    tracker=MotionTracker([0.,0.,.55],1,motion_contract(50))
    sample(tracker,[[.03,0.,.55]],[1])
    sample(tracker,[[.2,0.,.8]],[2],floor=[False],clearance=[.1])
    sample(tracker,[[.16,0.,.8]],[3],floor=[False],clearance=[.1])
    assert not tracker.violated[0] and tracker.first_tick[0]==-1
    np.testing.assert_allclose(tracker.peak_mm,[200],atol=1e-4)
    np.testing.assert_allclose(tracker.peak_ground_mm,[30],atol=1e-4)
    # Exemption does not use an inflated airborne peak when checking a landing.
    sample(tracker,[[.04,0.,.55]],[4])
    assert not tracker.violated[0]
    np.testing.assert_allclose(tracker.peak_ground_mm,[40],atol=1e-4)


def test_landing_reenables_bound_and_lifting_does_not_clear_past_violation():
    tracker=MotionTracker([0.,0.,.55],2,motion_contract(50))
    sample(tracker,[[.051,0.,.55],[.04,0.,.55]],[1,1])
    sample(tracker,[[.2,0.,.8],[.2,0.,.8]],[2,2],floor=[False,False],clearance=[.1,.1])
    np.testing.assert_array_equal(tracker.violated,[True,False])
    sample(tracker,[[.2,0.,.8],[.08,0.,.55]],[3,3],floor=[False,True],clearance=[.1,0])
    np.testing.assert_array_equal(tracker.violated,[True,True])
    np.testing.assert_array_equal(tracker.first_tick,[1,3])


def test_airborne_requires_no_contact_and_a_positive_lowest_point():
    tracker=MotionTracker([0.,0.,.55],3,motion_contract(50))
    sample(tracker,[[.1,0.,.55]]*3,[1]*3,floor=[True,False,False],clearance=[.1,0,.1])
    np.testing.assert_array_equal(tracker.violated,[True,True,False])


def test_ground_constraint_refuses_missing_contact_evidence():
    tracker=MotionTracker([0.,0.,.55],1,motion_contract(50))
    with pytest.raises(ValueError,match='floor contact'):
        tracker.observe(np.array([[.1,0.,.55]]),np.array([1]))


def test_legacy_whole_episode_constraint_is_not_silently_relaxed():
    old=dict(schema='hug-ga-object-motion-v1',metric='world-xy-center-displacement',
        reference='initial-object-center',max_object_xy_mm=50.,sampling='physics-tick',
        comparison='strictly-greater-than')
    assert read_motion_contract(dict(constraints=old))==old
    tracker=MotionTracker([0.,0.,.55],1,old)
    sample(tracker,[[.2,0.,.8]],[1],floor=[False],clearance=[.1])
    assert tracker.violated[0]
    assert tracker.peak_ground_mm[0]==0
