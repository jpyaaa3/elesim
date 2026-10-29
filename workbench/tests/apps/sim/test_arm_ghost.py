from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from elesim_protocol import ProtocolError, SimulationCommandRequest
from elesim_sim.simulation.arm_ghost import ArmGhost, sample_preview
from elesim_sim.simulation.operator_control import SimulationOperatorCommand, SimulationOperatorController

ROOT = Path(__file__).resolve().parents[4]
URDF = ROOT/'payload/data/models/assemblies/zed-mini/arm.urdf'


def preview():
    return dict(plan_id='plan', times_s=[0.,1.], positions=[[-.1,0,0,0],[-.11,.1,.2,-.2]],base=np.eye(4).tolist())


def request(value):
    return SimulationCommandRequest.from_payload(dict(schema_version=1,request_id='r',session_id='s',
        command='set_arm_ghost',arguments={'preview':value}))


@pytest.mark.parametrize('mutation',[
    lambda p:p.update(times_s=[0.,0.]), lambda p:p.update(times_s=[0.,121.]),
    lambda p:p.update(positions=[[0]*4]*97), lambda p:p['positions'][0].__setitem__(0,float('nan')),
    lambda p:p['base'][0].__setitem__(0,2), lambda p:p['base'][0].__setitem__(0,-1),
    lambda p:p.update(torque_enabled=True), lambda p:p.update(plan_id=''),
])
def test_preview_is_bounded_finite_and_has_no_motion_surface(mutation):
    value=preview();mutation(value)
    with pytest.raises(ProtocolError):request(value)


def test_playback_uses_timestamps_pause_and_expiry():
    value=request(preview()).arguments['preview']
    np.testing.assert_allclose(sample_preview(value,.5),[-.105,.05,.1,-.1])
    assert sample_preview(value,1.5)==tuple(value['positions'][-1])
    assert sample_preview(value,1.75)==tuple(value['positions'][0])
    assert sample_preview(value,121) is None
    assert sample_preview(None,0) is None


def test_mesh_ghost_has_no_physics_calls_and_clears_bounded_handles():
    draws=[];clears=[]
    scene=NS(draw_debug_mesh=lambda mesh, **kw:draws.append((mesh,kw)) or len(draws),
             clear_debug_object=clears.append)
    ghost=ArmGhost(URDF)
    ghost.update(scene,[-.1,0,0,0],np.eye(4))
    count=len(draws);assert count>10
    assert np.all(draws[0][0].visual.face_colors[:,3]==85)
    ghost.update(scene,[-.1,0,0,0],np.eye(4));assert len(draws)==count
    ghost.update(scene,[-.12,.2,.1,-.1],np.eye(4));assert len(clears)==count
    ghost.update(scene,None,None)
    assert not ghost.handles and len(clears)==2*count


def test_operator_preview_does_not_reset_or_step_simulation():
    effects=[]
    controller=SimulationOperatorController(reset_environment=lambda:effects.append('reset'),
                                            observer_command=lambda *args:effects.append('camera'))
    r=request(preview())
    ok,_=controller.apply(SimulationOperatorCommand(ui_id='u',session_id='s',request_ids=('r',),
                                                command=r.command,arguments=r.arguments))
    assert ok and controller.arm_ghost['plan_id']=='plan' and not effects
    r=request(None)
    controller.apply(SimulationOperatorCommand(ui_id='u',session_id='s',request_ids=('r',),
                                              command=r.command,arguments=r.arguments))
    assert controller.arm_ghost is None
