"""Actuator, model and contact invariants without importing Genesis."""
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from elesim_sim.robot.arm.actuation import PositionActuator,PositionGains
from elesim_sim.simulation.materials import apply_link_friction,apply_foot_friction
from elesim_sim.simulation.contact_metrics import camera_side_axes_from_frames,classify_faces,arm_contact_faults
from elesim_sim.model_bundle import physical_model_path


def test_position_actuator_does_not_write_poses_or_clear_velocity():
    calls=[]
    entity=NS(set_dofs_kp=lambda q,**kw:calls.append(('kp',q,kw)),
        set_dofs_kv=lambda q,**kw:calls.append(('kv',q,kw)),
        set_dofs_force_range=lambda lo,hi,**kw:calls.append(('limits',lo,hi)),
        control_dofs_position=lambda q,**kw:calls.append(('target',q,kw)))
    actuator=PositionActuator(entity,[6,7,12],6)
    np.testing.assert_equal(calls[0][1],[2000,200,200])
    np.testing.assert_equal(calls[1][1],[200,20,20])
    actuator.command([-.1,.2,-.3]);assert calls[-1][0]=='target'
    assert calls[-1][2]['dofs_idx_local']==[6,7,12]
    with pytest.raises(ValueError):actuator.command([0.,float('nan'),0.])


def test_bent_proximal_link_keeps_its_own_protected_face():
    frames={name:np.eye(4) for name in ('node0','node9','camera','gripper_base')}
    terminal=Rotation.from_euler('y',130,degrees=True).as_matrix()
    for name in ('node9','camera','gripper_base'):frames[name][:3,:3]=terminal
    frames['camera'][:3,3]=terminal@np.array([0.,0.,.05])
    axes=camera_side_axes_from_frames(frames)
    good,bad=classify_faces(np.array([0.,0.,-.04]),np.array([0.,0.,-1.]),np.zeros(3),
        np.array([1.,0.,0.,0.]),axes['node0'])
    assert good and not bad
    # The former node9-world-axis rule calls this very same contact forbidden.
    good,bad=classify_faces(np.array([0.,0.,-.04]),np.array([0.,0.,-1.]),np.zeros(3),
        np.array([1.,0.,0.,0.]),terminal@np.array([0.,0.,1.]))
    assert bad and not good


def test_friction_keeps_feet_and_arm_separate():
    result={}
    links=[NS(name=name,set_friction=lambda mu,n=name:result.update({n:mu}))
        for name in ('node0','FR_foot','gripper_base')]
    apply_link_friction(NS(links=links))
    assert result=={'node0':.5,'FR_foot':.8,'gripper_base':.5}
    apply_foot_friction(NS(links=links),.9)
    assert result=={'node0':.5,'FR_foot':.9,'gripper_base':.5}


def test_physical_model_opens_limits_and_keeps_legacy_bytes():
    import xml.etree.ElementTree as ET
    root=Path(__file__).resolve().parents[4]/'payload/data/models/assemblies/zed-mini'
    physical=physical_model_path(root)
    tree=ET.parse(physical).getroot()
    for name in ('j_housing_wedge','j_wedge_node0'):
        limit=tree.find("joint[@name='%s']/limit"%name)
        assert float(limit.get('lower'))<0<float(limit.get('upper'))
    assert tree.find("link[@name='camera']/collision") is not None
    old=ET.parse(root/'robot.urdf').getroot()
    assert old.find("joint[@name='j_housing_wedge']/limit").get('lower')=='0'


@pytest.mark.parametrize('a,b,expected',[(1,4,False),(1,2,True),(1,3,True),(1,0,True),(3,0,False),(4,3,True)])
def test_contact_faults_allow_only_admissible_contact_pairs(a,b,expected):
    roles=np.array([0,1,2,3,0]);objects=np.array([False,False,False,False,True])
    raw=dict(link_a=np.array([a]),link_b=np.array([b]),valid_mask=np.array([True]),
        force_a=np.array([[1.,0.,0.]]),normal=np.array([[1.,0.,0.]]),position=np.zeros((1,3)))
    assert bool(arm_contact_faults(raw,roles,objects,[])) is expected
    if a==1 and b==2:assert not arm_contact_faults(raw,roles,objects,[(1,2)])
