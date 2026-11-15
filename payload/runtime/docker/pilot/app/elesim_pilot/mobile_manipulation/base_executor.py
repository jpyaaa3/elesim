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
    def step(self,pose,linear_speed,angular_speed,segment_free):
        if not self.active:return np.zeros(3),True
        now=self.clock();pose=np.asarray(pose,float)
        if now-self.started>self.s.base_timeout_s:raise RuntimeError('base approach timeout')
        next_pose=self.goal.path[self.index]
        if not segment_free(pose,next_pose):raise RuntimeError('base path no longer observed free')
        delta=next_pose-pose;delta[2]=angle(delta[2])
        close=np.linalg.norm(delta[:2])<self.s.base_position_tolerance_m and abs(delta[2])<self.s.base_yaw_tolerance_rad
        if self.goal.report.get('kind')=='grasp' and self.index==len(self.goal.path)-1:
            combined=np.linalg.norm(delta[:2])+self.goal.report['target_reach_m']*abs(delta[2])
            close=close and combined<=self.s.arrival_error_m
        if close:
            if self.index<len(self.goal.path)-1:
                self.index+=1;self.best=float('inf');self.last_progress=now;return np.zeros(3),False
            if linear_speed<.01 and angular_speed<.05:
                if self.settled is None:self.settled=now
                if now-self.settled>=self.s.settle_s:self.active=False;return np.zeros(3),True
            else:self.settled=None
            return np.zeros(3),False
        self.settled=None
        distance=np.linalg.norm(delta[:2])+.2*abs(delta[2])
        if distance<self.best-.0002:self.best=distance;self.last_progress=now
        if now-self.last_progress>10.:raise RuntimeError('base made no measured progress')
        c,s=np.cos(pose[2]),np.sin(pose[2]);body=np.array([[c,s],[-s,c]])@delta[:2]
        v=body*.8;norm=np.linalg.norm(v)
        if norm>self.s.base_speed_mps:v*=self.s.base_speed_mps/norm
        return np.r_[v,np.clip(delta[2]*.8,-self.s.yaw_speed_radps,self.s.yaw_speed_radps)],False
    def cancel(self):self.active=False;return np.zeros(3)
