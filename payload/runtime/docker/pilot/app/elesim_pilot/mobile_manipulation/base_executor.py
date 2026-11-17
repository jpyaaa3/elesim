"""Feedback-based holonomic base control; no open-loop timed walk fallback."""
import numpy as np
from .contracts import angle


class BaseExecutor:
    def __init__(self,settings,clock):self.s=settings;self.clock=clock;self.active=False
    def start(self,goal,current_pose,map_revision):
        if self.active:raise RuntimeError('base motion already active')
        self.goal=goal;self.index=1 if len(goal.path)>1 else 0
        self.started=self.clock();self.last_progress=self.started;self.best=float('inf')
        self.settled=None;self.map_revision=map_revision;self.active=True
        self.stage='approach';self.precise=False;self.settle_pose=None
    def _close(self,delta,*,precise):
        close=np.linalg.norm(delta[:2])<self.s.base_position_tolerance_m and abs(delta[2])<self.s.base_yaw_tolerance_rad
        if precise and self.goal.report.get('kind')=='grasp':
            combined=np.linalg.norm(delta[:2])+self.goal.report['target_reach_m']*abs(delta[2])
            close=close and combined<=self.s.arrival_error_m
        return close
    def step(self,pose,linear_speed,angular_speed,segment_free,*,posture_ready):
        if not self.active:return np.zeros(3),True
        now=self.clock();pose=np.asarray(pose,float)
        if now-self.started>self.s.base_timeout_s:raise RuntimeError('base approach timeout')
        next_pose=self.goal.path[self.index]
        delta=next_pose-pose;delta[2]=angle(delta[2])
        close=self._close(delta,precise=self.precise and self.index==len(self.goal.path)-1)
        if self.stage=='leveling':
            # Zero velocity lets Sim finish landing, then level through leg
            # position control. Do not restart walking during that transition.
            if posture_ready and linear_speed<.01 and angular_speed<.05:
                change=pose-self.settle_pose if self.settle_pose is not None else np.zeros(3)
                if (self.settle_pose is None or np.linalg.norm(change[:2])>.002
                        or abs(angle(change[2]))>np.deg2rad(.25)):
                    self.settled=now;self.settle_pose=pose.copy()
                if now-self.settled>=self.s.settle_s:
                    # Leveling may shift the body. Check the settled pose;
                    # only then permit a new, swept-checked position correction.
                    if self._close(delta,precise=True):
                        self.active=False;self.stage='completed';return np.zeros(3),True
                    self.stage='approach';self.precise=True
                    self.best=float('inf');self.last_progress=now
                    self.settled=None;self.settle_pose=None
            else:self.settled=None;self.settle_pose=None
            return np.zeros(3),False
        if close:
            if self.index<len(self.goal.path)-1:
                self.index+=1;self.best=float('inf');self.last_progress=now;return np.zeros(3),False
            self.stage='leveling';self.settled=None;self.settle_pose=None
            return np.zeros(3),False
        # A goal already reached needs zero velocity and measured settling,
        # not a submillimetre corrective move into a different grid cell.
        # Every nonzero command still requires a checked swept footprint.
        if not segment_free(pose,next_pose):raise RuntimeError('base path no longer observed free')
        self.settled=None
        distance=np.linalg.norm(delta[:2])+.2*abs(delta[2])
        if distance<self.best-.0002:self.best=distance;self.last_progress=now
        if now-self.last_progress>10.:raise RuntimeError('base made no measured progress')
        c,s=np.cos(pose[2]),np.sin(pose[2]);body=np.array([[c,s],[-s,c]])@delta[:2]
        v=body*.8;norm=np.linalg.norm(v)
        if norm>self.s.base_speed_mps:v*=self.s.base_speed_mps/norm
        return np.r_[v,np.clip(delta[2]*.8,-self.s.yaw_speed_radps,self.s.yaw_speed_radps)],False
    def cancel(self):self.active=False;self.stage='cancelled';return np.zeros(3)
