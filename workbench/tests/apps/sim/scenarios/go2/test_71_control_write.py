"""Static DOF selection must preserve every command and unrelated actuator."""
from types import SimpleNamespace

import numpy as np
import pytest

from elesim_sim.robot.go2.locomotion.command import Go2CommandShaper
from elesim_sim.robot.go2.locomotion.types import Go2Command
from elesim_sim.robot.go2.pympc_controller import PyMpcGenesisController


@pytest.mark.parametrize('indices,compact', [
    (list(range(6,18)),True), ([4],True), ([],False),
    ([4,6,7],False), ([5,4,3],False), ([4,4,5],False),
    ([-3,-2,-1],False), ([11,3,9,2,14,7,8,6,5,4,10,12],False),
])
def test_static_selection_preserves_order_and_only_compacts_contiguous_dofs(indices,compact):
    controller=object.__new__(PyMpcGenesisController)
    controller._leg_dof_idxs=indices.copy()
    selected=controller._leg_control_dofs
    assert isinstance(selected,range)==compact
    assert list(selected)==indices
    assert controller._leg_control_dofs is selected
    assert controller._leg_dof_idxs==indices


class Actuators:
    def __init__(self):
        self.mode=np.full(32,2)
        self.force=np.arange(32,dtype=float)
        self.position=np.linspace(-1.,1.,32)
        self.velocity=np.linspace(1.,2.,32)
        self.writes=[]
        self.raw_writes=[]

    def control_dofs_force(self,value,*,dofs_idx_local):
        self.raw_writes.append(value)
        self.writes.append((np.asarray(value).copy(),dofs_idx_local))
        self.mode[list(dofs_idx_local)]=1
        self.force[list(dofs_idx_local)]=value


def walking_controller(indices):
    c=object.__new__(PyMpcGenesisController)
    c._entity=Actuators()
    c._leg_dof_idxs=indices
    c._dt=.01;c._sim_time=1.;c._faulted=False;c._pose_stage='walk'
    c._command_shaper=Go2CommandShaper()
    c._command_shaper.set_target(Go2Command(vx=.8))
    c._solver=SimpleNamespace(backend='jax_mppi')
    c._config=SimpleNamespace(command_idle_threshold=.05)
    c._torque_mode_active=True;c._step_i=0
    c._sample=lambda:(None,None,None,None,None)
    c._torques=lambda *_args:np.linspace(-3.,3.,12)
    c._record_contact_metrics=lambda _sample:None
    return c


@pytest.mark.parametrize('indices',[list(range(6,18)),[11,3,9,2,14,7,8,6,5,4,10,12]])
def test_every_physics_tick_writes_same_torque_without_touching_arm_or_modes(indices):
    c=walking_controller(indices)
    original={name:getattr(c._entity,name).copy() for name in ('mode','force','position','velocity')}
    for _ in range(2):c._step()
    assert len(c._entity.writes)==2
    for value,selected in c._entity.writes:
        assert list(selected)==indices
        assert isinstance(selected,range)==(indices==list(range(6,18)))
        np.testing.assert_array_equal(value,np.linspace(-3.,3.,12))
    unrelated=[i for i in range(32) if i not in indices]
    np.testing.assert_array_equal(c._entity.mode[indices],np.ones(12))
    np.testing.assert_array_equal(c._entity.force[indices],np.linspace(-3.,3.,12))
    for name in ('mode','force'):
        np.testing.assert_array_equal(getattr(c._entity,name)[unrelated],original[name][unrelated])
    for name in ('position','velocity'):
        np.testing.assert_array_equal(getattr(c._entity,name),original[name])
    assert c._step_i==2


def test_force_input_is_a_cpu_tensor_with_the_original_precision_and_current_values():
    torch=pytest.importorskip('torch')
    c=walking_controller(list(range(6,18)))
    values=np.linspace(-3.,3.,12,dtype=np.float64)
    c._torques=lambda *_args:values.copy()
    c._step()
    first=c._entity.raw_writes[0]
    assert isinstance(first,torch.Tensor)
    assert first.device.type=='cpu' and first.dtype==torch.float64
    np.testing.assert_array_equal(first.numpy(),values)
    values+=.125
    c._step()
    np.testing.assert_array_equal(c._entity.raw_writes[1].numpy(),values)
    np.testing.assert_array_equal(first.numpy(),values-.125)
