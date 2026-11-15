"""Slow measured-feedback progress along an already certified joint segment."""
import numpy as np


class JointApproach:
    rates=np.array([.005,.04,.04,.04])
    tolerance=np.array([.001,.015,.015,.015])

    def __init__(self,start,goal,*,tolerance=None):
        self.start=np.array(start,dtype=float,copy=True)
        self.goal=np.array(goal,dtype=float,copy=True)
        if (self.start.shape!=(4,) or self.goal.shape!=(4,)
                or not np.isfinite([self.start,self.goal]).all()):
            raise ValueError('invalid arm approach endpoints')
        self.tolerance=np.array(self.tolerance if tolerance is None else tolerance,float,copy=True)
        if self.tolerance.shape!=(4,) or not np.isfinite(self.tolerance).all() or np.any(self.tolerance<=0):
            raise ValueError('invalid arm approach tolerance')
        self.delta=self.goal-self.start
        self.duration=max(.1,float(np.max(abs(self.delta)/self.rates)))
        self.fraction=0.;self.command=self.start.copy()

    def step(self,actual,dt):
        actual=np.asarray(actual,float)
        if actual.shape!=(4,) or not np.isfinite(actual).all() or not np.isfinite(dt) or not 0<=dt<=.1:
            raise ValueError('invalid arm approach feedback/time')
        if np.any(abs(actual-self.command)>self.tolerance+1e-12):
            raise RuntimeError('arm did not follow approach')
        fraction=min(1.,self.fraction+dt/self.duration)
        active=abs(self.delta)>1e-12
        if np.any(active):
            # Keep headroom inside the existing certificate's tracking tube.
            # A dropped/delayed command slows progress; it cannot accumulate
            # into a growing lead over the measured arm.
            limit=actual+np.sign(self.delta)*(.8*self.tolerance)
            cap=float(np.min((limit-self.start)[active]/self.delta[active]))
            fraction=max(self.fraction,min(fraction,cap))
        self.fraction=fraction
        self.command=self.start+self.delta*fraction
        return self.command.copy()
