"""Bounded source-fenced histories. Image association never uses future/latest q."""
from collections import deque
from threading import RLock
import numpy as np
from .contracts import vector


class Feedback:
    def __init__(self,clock):
        self.lock=RLock();self.clock=clock;self.source=None;self.joints=deque(maxlen=256);self.bases=deque(maxlen=256)
    def accept(self,payload,source):
        with self.lock:return self._accept(payload,source)
    def _accept(self,payload,source):
        if self.source!=source:
            self.source=source;self.joints.clear();self.bases.clear()
        now=self.clock()
        if 'q' in payload:
            if payload.get('q_source')!='simulated':raise ValueError('Sim measured feedback required')
            q=vector(payload['q'],4);stamp=float(payload['measurement_time_s']);mono=float(payload['measurement_monotonic_s'])
            if not np.isfinite([stamp,mono]).all():raise ValueError('invalid joint timestamp')
            if not self.joints or mono>self.joints[-1]['mono']:
                sim_time=payload.get('sim_time_s')
                if sim_time is not None and (isinstance(sim_time,bool) or not np.isfinite(sim_time) or sim_time<0):
                    raise ValueError('invalid simulation time')
                self.joints.append(dict(q=q,stamp=stamp,mono=mono,received=now,sim_time=sim_time))
        if 'go2_base_pos' in payload:
            pos=vector(payload['go2_base_pos'],3);rpy=vector(payload['go2_base_rpy'],3)
            velocity=vector(payload.get('go2_base_lin_vel_body',[0,0,0]),3)
            angular=vector(payload.get('go2_base_ang_vel',[0,0,0]),3)
            stamp=float(payload['go2_base_timestamp_s'])
            if not np.isfinite(stamp):raise ValueError('invalid base timestamp')
            if not self.bases or stamp>self.bases[-1]['stamp']:
                self.bases.append(dict(pos=pos,rpy=rpy,velocity=velocity,angular=angular,stamp=stamp,received=now))
    def snapshot(self,source,max_age=.5):
        with self.lock:return self._snapshot(source,max_age)
    def _snapshot(self,source,max_age=.5):
        if source!=self.source or not self.joints or not self.bases:raise RuntimeError('missing source-matched arm/base feedback')
        ji,bi=len(self.joints)-1,len(self.bases)-1
        j,b=self.joints[ji],self.bases[bi]
        # Sim emits arm and body fields separately. A new half-frame must not
        # invalidate a still-fresh coherent pair while its other half arrives.
        # Walk the bounded histories backwards without widening either gate.
        while abs(j['stamp']-b['stamp'])>.2:
            if j['stamp']>b['stamp']:ji-=1
            else:bi-=1
            if min(ji,bi)<0:raise RuntimeError('arm/base samples are not coherent')
            j,b=self.joints[ji],self.bases[bi]
        arm_age=self.clock()-j['received'];base_age=self.clock()-b['received']
        if max(arm_age,base_age)>max_age:
            raise RuntimeError(f'stale arm/base feedback (arm {arm_age:.3f}s, base {base_age:.3f}s)')
        return dict(joint=j,base=b,pose=np.r_[b['pos'][:2],b['rpy'][2]])
    def associate(self,frame,source,max_delta=.05):
        with self.lock:return self._associate(frame,source,max_delta)
    def _associate(self,frame,source,max_delta=.05):
        if source!=self.source:raise RuntimeError('image source changed')
        matches=[b for b in self.bases if abs(b['stamp']-frame.ts)<=max_delta]
        if not matches:raise RuntimeError('no capture-time base measurement')
        b=min(matches,key=lambda b:abs(b['stamp']-frame.ts))
        if np.linalg.norm(b['velocity'])>.01 or np.linalg.norm(b['angular'])>.05:
            raise RuntimeError('stationary capture required')
        return b
