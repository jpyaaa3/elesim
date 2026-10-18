"""Shared, current-tick Genesis state must retain the public getter values."""
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from elesim_sim.robot.go2.mpc.genesis_pin_bridge import GenesisPinBridge
from elesim_sim.robot.go2.mpc.payload_model import ArmPayloadCompensator


class Entity:
    def __init__(self):
        self.n_links, self.n_dofs = 19, 30
        self.calls = []
        rng = np.random.default_rng(123)
        self.values = dict(pos=rng.normal(size=(19, 3)),
            quat=Rotation.random(19,random_state=rng).as_quat()[:,[3,0,1,2]],
            vel=rng.normal(size=(19,3)),ang=rng.normal(size=(19,3)),
            position=rng.normal(size=30),velocity=rng.normal(size=30),
            contact_force=rng.normal(size=(19,3)))
        self.links=[]
        for i in range(19):
            link=SimpleNamespace(idx_local=i,name='base' if i==5 else 'arm'+str(i),
                desc=SimpleNamespace(mass=.1+.01*i,inertial_pos=rng.normal(size=3)*.02,
                    inertial_quat=np.array([1.,0.,0.,0.]),inertia=np.diag([.001,.002,.003])))
            for field in ('pos','quat','vel','ang'):
                setattr(link,'get_'+field,lambda f=field,j=i:self.values[f][j].copy())
            self.links.append(link)

    def get_link(self,name):
        return next(link for link in self.links if link.name==name)

    def read(self,name,indices=None):
        self.calls.append((name,indices))
        source=self.values[name]
        value=source.clone() if hasattr(source,'clone') else source.copy()
        return value if indices is None else value[indices]

    def get_links_pos(self,links_idx_local=None):return self.read('pos',links_idx_local)
    def get_links_quat(self,links_idx_local=None):return self.read('quat',links_idx_local)
    def get_links_vel(self,links_idx_local=None):return self.read('vel',links_idx_local)
    def get_links_ang(self,links_idx_local=None):return self.read('ang',links_idx_local)
    def get_dofs_position(self,dofs_idx_local=None):return self.read('position',dofs_idx_local)
    def get_dofs_velocity(self,dofs_idx_local=None):return self.read('velocity',dofs_idx_local)
    def get_links_net_contact_force(self):return self.read('contact_force')


def reader(entity):
    from elesim_sim.simulation.genesis.control_state import EntityControlStateReader
    return EntityControlStateReader(entity)


def test_shared_state_reads_seven_contiguous_fields_once_and_owns_current_tick():
    entity=Entity()
    capture=reader(entity)
    snapshot=capture.read()
    expected={name:value.copy() for name,value in entity.values.items()}
    assert entity.calls==[(name,None) for name in expected]
    for name,value in expected.items():
        np.testing.assert_array_equal(getattr(snapshot,name),value)
    entity.values['pos'] += 1.
    current=capture.read()
    np.testing.assert_array_equal(snapshot.pos,expected['pos'])
    np.testing.assert_array_equal(current.pos,entity.values['pos'])


def test_tensor_fields_use_one_host_transfer_and_match_numpy_fields():
    torch=pytest.importorskip('torch')
    entity=Entity()
    expected=reader(entity).read()
    entity.values={name:torch.tensor(value,dtype=torch.float64) for name,value in entity.values.items()}
    original_cpu=torch.Tensor.cpu
    calls=[]
    def copy_to_host(tensor,*args,**kwargs):
        calls.append(tuple(tensor.shape))
        return original_cpu(tensor,*args,**kwargs)
    with patch.object(torch.Tensor,'cpu',copy_to_host):
        actual=reader(entity).read()
    assert len(calls)==1
    for name in entity.values:
        np.testing.assert_array_equal(getattr(actual,name),getattr(expected,name))


def test_bridge_and_full_payload_share_a_snapshot_with_no_further_gpu_queries():
    entity=Entity()
    dofs=[14,7,19,3,21,9,12,8,16,11,20,6]
    payload=ArmPayloadCompensator(entity,link_names={link.name for link in entity.links if link.idx_local!=5})
    reference_payload=payload.measure()
    reference_q,reference_dq=GenesisPinBridge(entity,dofs).read_pin_q_dq(dt=.01)
    snapshot=reader(entity).read()
    entity.calls.clear()
    actual_q,actual_dq=GenesisPinBridge(entity,dofs).read_pin_q_dq(dt=.01,snapshot=snapshot)
    payload.reset()
    actual_payload=payload.measure(snapshot=snapshot)
    assert entity.calls==[]
    np.testing.assert_array_equal(actual_q,reference_q)
    np.testing.assert_array_equal(actual_dq,reference_dq)
    for field in ('mass_kg','com_world','vel_world','inertia_world'):
        np.testing.assert_array_equal(getattr(actual_payload,field),getattr(reference_payload,field))


@pytest.mark.parametrize('name,shape',[('pos',(18,3)),('quat',(19,3)),('velocity',(2,30)),('contact_force',(19,2))])
def test_bad_public_field_shape_is_rejected(name,shape):
    entity=Entity();entity.values[name]=np.zeros(shape)
    with pytest.raises(ValueError,match=name):reader(entity).read()


def test_snapshot_bridge_retains_live_and_pose_difference_twist_and_reset():
    entity=Entity()
    entity.values['pos'][5]=[0.,0.,.32]
    entity.values['quat'][5]=[1.,0.,0.,0.]
    entity.values['vel'][5]=0.
    entity.values['ang'][5]=0.
    capture=reader(entity)
    scalar=GenesisPinBridge(entity,list(range(12)))
    shared=GenesisPinBridge(entity,list(range(12)))
    for shift,yaw in ((0.,0.),(.002,.01),(.006,.02)):
        entity.values['pos'][5,0]=shift
        entity.values['quat'][5]=Rotation.from_euler('z',yaw).as_quat()[[3,0,1,2]]
        expected=scalar.read_pin_q_dq(dt=.02)
        actual=shared.read_pin_q_dq(dt=.02,snapshot=capture.read())
        for first,second in zip(expected,actual):np.testing.assert_array_equal(second,first)
    shared.reset();entity.values['pos'][5,0]=5.
    _,velocity=shared.read_pin_q_dq(dt=.02,snapshot=capture.read())
    np.testing.assert_array_equal(velocity[:6],0.)
    entity.values['vel'][5]=[.2,.3,.1]
    entity.values['ang'][5]=[.1,-.1,.2]
    scalar.reset();shared.reset()
    expected=scalar.read_pin_q_dq(dt=.02)
    actual=shared.read_pin_q_dq(dt=.02,snapshot=capture.read())
    np.testing.assert_array_equal(actual[1],expected[1])


def test_payload_motion_reset_and_separate_entity_keep_live_public_values():
    entity=Entity()
    for name in ('vel','ang'):entity.values[name].fill(0.)
    capture=reader(entity)
    names={link.name for link in entity.links if link.idx_local!=5}
    shared=ArmPayloadCompensator(entity,link_names=names)
    separate=ArmPayloadCompensator(entity,link_names=names)
    for shift in (0.,.002,.004):
        entity.values['pos'][:,0]+=shift
        actual=shared.measure(dt=.02,snapshot=capture.read())
        expected=separate.measure(dt=.02)
        for field in ('com_world','vel_world','inertia_world'):
            np.testing.assert_array_equal(getattr(actual,field),getattr(expected,field))
    for payload in (shared,separate):payload.reset()
    entity.values['pos'][:,0]+=1.
    unrelated=reader(Entity()).read()
    actual=shared.measure(dt=.02,snapshot=unrelated)
    expected=separate.measure(dt=.02)
    np.testing.assert_array_equal(actual.vel_world,expected.vel_world)
    np.testing.assert_array_equal(actual.com_world,expected.com_world)


def test_foreign_snapshot_cannot_supply_body_or_contact_feedback():
    from elesim_sim.robot.go2.pympc_controller import PyMpcGenesisController
    entity=Entity();other=reader(Entity()).read()
    with pytest.raises(ValueError,match='another entity'):
        GenesisPinBridge(entity,list(range(12))).read_pin_q_dq(snapshot=other)
    control=PyMpcGenesisController.__new__(PyMpcGenesisController)
    control._entity=entity
    with pytest.raises(ValueError,match='another entity'):
        control._normal_contact_forces(snapshot=other)


def test_contact_feedback_reuses_snapshot_and_preserves_the_foot_order():
    from elesim_sim.robot.go2.pympc_controller import PyMpcGenesisController
    entity=Entity()
    indices=[8,3,14,10]
    for name,index in zip(('FL','FR','RL','RR'),indices):entity.links[index].name=name+'_foot'
    control=PyMpcGenesisController.__new__(PyMpcGenesisController)
    control._entity=entity
    expected=control._normal_contact_forces()
    snapshot=reader(entity).read()
    entity.calls.clear()
    actual=control._normal_contact_forces(snapshot=snapshot)
    assert entity.calls==[]
    np.testing.assert_array_equal(actual,expected)
    actual.fill(0.)
    np.testing.assert_array_equal(snapshot.contact_force[indices,2],expected)


def test_small_entities_keep_the_existing_getter_path():
    from elesim_sim.simulation.genesis.control_state import EntityControlStateReader
    assert EntityControlStateReader.for_entity(SimpleNamespace()) is None
