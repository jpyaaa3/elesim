"""q3/q4 weights blend all four free command paths after original replay."""
import numpy as np
from elesim_sim.rl.trajectory import Trajectory
from .configure import CHANNELS,CONTROL_MODE,episode_duration,validate_command
from .triangle import weights
from .control import action_rates


def interpolate(anchors,target,*,samples=None):
    if len(anchors)!=3:
        raise ValueError('Exactly three q anchors required')
    ctx=target.get('simulation_context')
    if not isinstance(ctx,dict) or ctx.get('control_mode')!=CONTROL_MODE or target.get('control_mode')!=CONTROL_MODE:
        raise ValueError('Missing free-four-DoF context')
    s=ctx['settings']
    duration=max(float(a.times[-1]) for a in anchors)
    if not np.isclose(target['duration_s'],duration) or not 0<duration<=episode_duration(s):
        raise ValueError('Target must use the longest original path duration within episode limit')
    for a in anchors:
        a.validate()
        if (a.channels!=CHANNELS or a.diagnostics.get('replay_success') is not True
            or a.diagnostics.get('physics_validated') is not True or a.diagnostics.get('replay_repetitions',0)<3):
            raise ValueError('Replay all original anchors successfully first')
        for k in ('simulation_context','diameter_mm','roll_deg','branch','start_command','control_mode'):
            if a.metadata.get(k)!=target.get(k):
                raise ValueError(f'Anchor mismatch: {k}')
        if a.metadata.get('split')!='train' or not np.isclose(a.metadata['duration_s'],a.times[-1]):
            raise ValueError('Invalid original anchor')
        if not np.allclose(a.commands[0],target['start_command'],atol=1e-7):
            raise ValueError('Anchor does not start at Home')
    w=weights([a.metadata['q_deg'] for a in anchors],np.array(target['q_deg']))
    if np.any(w < -1e-9):
        raise ValueError('q extrapolation outside triangle is forbidden')
    w=np.maximum(w,0);w/=w.sum()
    if samples is None:samples=round(duration/s['dt'])+1
    if isinstance(samples,bool) or int(samples)!=samples or samples<2:
        raise ValueError('At least two trajectory samples required')
    times=np.linspace(0,duration,int(samples));commands=np.zeros((len(times),4))
    # Stretch faster originals to the longest duration: never speed up their
    # commands or shorten their successful final hold window. No fixed roll,
    # deployment or lift timing is imposed on any candidate.
    for weight,a in zip(w,anchors):
        commands+=weight*a.sample(times/duration*a.times[-1])
    for i,value in enumerate(commands):
        validate_command(target,s,i,value)
    if np.any(np.abs(np.diff(commands,axis=0))/(times[1]-times[0]) >
              action_rates(s)/s['action_ticks']/s['dt']+1e-5):
        raise ValueError('Interpolation exceeds physical command rate')
    return Trajectory(times,commands,CHANNELS,dict(target,
        interpolation='q3-q4-time-normalized-four-dof',weights=w.tolist(),
        anchor_ids=[a.metadata['case_id'] for a in anchors]),
        dict(replay_success=None,physics_validated=False)).validate()
