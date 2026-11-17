"""A slow arrival correction moves the body while retaining planted feet."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from elesim_sim.robot.go2.pympc_controller import planted_body_joint_target
from elesim_sim.robot.go2.foot_workspace import foot_position_body
from elesim_sim.robot.go2.locomotion.kinematics import GO2_STAND_Q,GO2_LEG_JOINTS
from elesim_sim.robot.go2.locomotion.types import Go2Command
from workbench.tests.apps.sim.scenarios.go2.test_67_braking_latch import walking


def feet():
    q=np.array([GO2_STAND_Q[name] for name in GO2_LEG_JOINTS]).reshape(4,3)
    return np.array([foot_position_body(i,angles) for i,angles in enumerate(q)])


@pytest.mark.parametrize('shift,yaw',[([.025,0.,0.],0.),([0.,-.025,0.],0.),
    ([.015,-.015,0.],.04),([0.,0.,0.],-.05)])
def test_joint_ik_places_the_body_without_moving_anchored_feet(shift,yaw):
    before=feet()
    q=planted_body_joint_target(before,shift,yaw,np.full((4,3),-np.pi),np.full((4,3),np.pi))
    after=np.array([foot_position_body(i,a) for i,a in enumerate(q.reshape(4,3))])
    np.testing.assert_allclose(after@Rotation.from_euler('z',yaw).as_matrix().T+shift,before,atol=1e-10)


def test_workspace_violation_cannot_produce_a_joint_reference():
    with pytest.raises(ValueError,match='workspace'):
        planted_body_joint_target(feet(),[1.,0.,0.],0.,np.full((4,3),-np.pi),np.full((4,3),np.pi))


def test_small_intent_finishes_landing_then_uses_leg_control_without_restarting_gait():
    c,calls=walking();c._command_shaper.set_target(Go2Command(vx=.02))
    c._kin.stand_q=np.zeros(12)
    c._contact_gait.confirmed[:]=True;c._contact_gait.load[:]=1.;c._contact_gait.blocked=False
    trimmed=[];c._trim_standing_body=lambda:trimmed.append(True) or True
    for _ in range(100):c._step()
    assert c.locomotion_stage=='stand' and trimmed
    assert calls.count('begin_stand')==1


def test_trim_integrates_once_in_a_fixed_anchor_frame_and_stops_at_workspace_bound():
    c,_=walking();c._in_place_pose=lambda nominal:np.array([GO2_STAND_Q[n] for n in GO2_LEG_JOINTS])
    c._foot_lower=np.full((4,3),-np.pi);c._foot_upper=np.full((4,3),np.pi)
    c._kin.stand_q=np.zeros(12);c._cmd=Go2Command(vx=.02);c._dt=.1
    for _ in range(10):assert c._trim_standing_body()
    np.testing.assert_allclose(c._stand_trim_displacement,[.02,0.,0.])
    q=c._standing_q.copy();anchor=c._stand_trim_anchor.copy()
    c._cmd=Go2Command()
    assert c._trim_standing_body()
    np.testing.assert_allclose(c._standing_q,q)
    np.testing.assert_array_equal(c._stand_trim_anchor,anchor)
    c._cmd=Go2Command(vx=.02)
    for _ in range(100):
        if not c._trim_standing_body():break
    else:raise AssertionError('unbounded planted-body shift')
    assert np.linalg.norm(c._stand_trim_displacement[:2])<=.05
