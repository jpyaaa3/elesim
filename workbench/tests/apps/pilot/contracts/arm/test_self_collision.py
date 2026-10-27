"""CPU collision-model parity and continuous joint-segment rejection."""
from pathlib import Path

import numpy as np
import pytest

from elesim_pilot.robot.arm.self_collision import ArmSelfCollision


MODEL = Path(__file__).resolve().parents[6]/'payload/data/models/planning/zed-mini'


def test_straight_arm_is_free_and_folded_arm_is_rejected():
    guard = ArmSelfCollision(MODEL)
    assert guard.configuration_free([0., 0., 0., 0.])
    assert not guard.configuration_free([-.1, 0., .628, .628])


def test_invalid_joint_data_cannot_be_certified():
    guard = ArmSelfCollision(MODEL)
    for q in ([0,0,0], [0,0,float('nan'),0], [0,0,2,0]):
        with pytest.raises(ValueError):guard.configuration_free(q)


def test_repeated_free_pose_has_a_zero_motion_certificate():
    guard = ArmSelfCollision(MODEL)
    result = guard.certify_segment(np.zeros(4),np.zeros(4))
    assert result['accepted'] and result['checked_configurations'] >= 1
    assert result['minimum_margin_m'] > 0


def test_safe_endpoints_cannot_hide_a_colliding_midpoint():
    guard=ArmSelfCollision(MODEL)
    a=np.array([-.070349122,.894044078,.417569274,-.042680587])
    b=np.array([-.091113243,-1.136831976,.231333191,-.627647143])
    assert guard.configuration_free(a) and guard.configuration_free(b)
    assert not guard.configuration_free((a+b)*.5)
    result=guard.certify_segment(a,b)
    assert not result['accepted'] and result['reason']=='self_collision_on_path'


def test_certificate_cannot_succeed_when_subdivision_budget_is_exhausted():
    guard=ArmSelfCollision(MODEL)
    result=guard.certify_segment(np.zeros(4),[-.132063,.382158,-.33934,.558898],max_checks=2)
    assert not result['accepted'] and result['reason']=='collision_certificate_budget'


def test_planning_fk_matches_the_independent_actual_urdf_for_all_collision_links():
    from workbench.research.experiments.vision_arm_scene import ArmUrdf
    guard=ArmSelfCollision(MODEL)
    data=MODEL.parents[2]
    reference=ArmUrdf(data/'models/assemblies/zed-mini/arm.urdf',data/'calibration/cameras/zed_mini.hand_eye.json',base_height_m=0)
    for q in ([0,0,0,0],[-.13,.38,-.34,.56],[-.1,-.4,.15,.2]):
        actual=reference.link_poses(q);planned=guard.link_poses(q)
        for name in guard.names:np.testing.assert_allclose(planned[name],actual[name],atol=1e-10)


def test_artifact_edit_is_rejected_before_use(tmp_path):
    import shutil
    for name in ('arm.urdf','robot.yml','metadata.json'):shutil.copyfile(MODEL/name,tmp_path/name)
    with (tmp_path/'robot.yml').open('a') as stream:stream.write('\n# changed\n')
    with pytest.raises(ValueError,match='digest mismatch'):ArmSelfCollision(tmp_path)


def test_certified_uncertainty_box_contains_perturbed_dense_actual_motion():
    bounds=np.array([.004,.06,.018,.018])
    robust=ArmSelfCollision(MODEL,joint_uncertainty=bounds)
    exact=ArmSelfCollision(MODEL)
    a=np.zeros(4);b=np.array([-.132063,.382158,-.33934,.558898])
    assert robust.certify_segment(a,b)['accepted']
    rng=np.random.default_rng(593)
    for fraction in np.linspace(0,1,25):
        for _ in range(3):
            actual=np.clip(a*(1-fraction)+b*fraction+rng.uniform(-bounds,bounds),exact.bounds[:,0],exact.bounds[:,1])
            assert exact.configuration_free(actual)
