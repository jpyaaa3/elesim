"""Evidence-driven reset frontier with retention, retreat and checkpoint state."""
from copy import deepcopy
import numpy as np

DEFAULTS=dict(advance_at=.7,retreat_at=.2,window=512,cooldown_steps=80,
              retention_fraction=.2,nominal_fraction=.5,seed=42)


def validate_options(o):
    if set(o)!=set(DEFAULTS):raise ValueError('Unknown or missing curriculum option')
    if not 0<=o['retreat_at']<o['advance_at']<=1:raise ValueError('Invalid promotion thresholds')
    if not 0<=o['retention_fraction']<1 or not 0<o['nominal_fraction']<1:raise ValueError('Invalid reset mixture')
    for k in ('window','cooldown_steps','seed'):
        if type(o[k]) is not int or o[k]<(0 if k=='seed' else 1):raise ValueError('Invalid '+k)


class Curriculum:
    def __init__(self,stages,options=None):
        if type(stages) is not int or stages<2:raise ValueError('Require grasp and Home path stages')
        self.options=dict(DEFAULTS,**(options or {}));validate_options(self.options)
        self.stages=stages;self.frontier=0;self.epoch=0;self.steps=0;self.last_move=0
        self.outcomes=[];self.nominal_outcomes=[];self.nominal_ready=False
        self.rng=np.random.default_rng(self.options['seed'])

    @property
    def nominal_enabled(self):return self.frontier==self.stages-1

    def sample(self,count):
        # Negative1 explicitly means Home plus the original nominal object.
        bins=np.full(count,self.frontier,int)
        if self.frontier:
            retention=self.rng.random(count)<self.options['retention_fraction']
            bins[retention]=self.rng.integers(0,self.frontier,size=int(retention.sum()))
        if self.nominal_enabled:
            nominal=self.rng.random(count)<self.options['nominal_fraction'];bins[nominal]=-1
        return bins,np.full(count,self.epoch,int)

    def observe(self,bins,epochs,success):
        self.steps+=1
        bins=np.asarray(bins);epochs=np.asarray(epochs);success=np.asarray(success,bool)
        # An episode started before the last promotion cannot move the new frontier.
        current=epochs==self.epoch
        for b,ok in zip(bins[current],success[current]):
            if b==self.frontier:self.outcomes.append(bool(ok))
            if b==-1 and self.nominal_enabled:self.nominal_outcomes.append(bool(ok))
        w=self.options['window'];self.outcomes=self.outcomes[-w:];self.nominal_outcomes=self.nominal_outcomes[-w:]
        if len(self.nominal_outcomes)>=w:self.nominal_ready=float(np.mean(self.nominal_outcomes))>=self.options['advance_at']
        if len(self.outcomes)<w or self.steps-self.last_move<self.options['cooldown_steps']:return
        rate=float(np.mean(self.outcomes));target=self.frontier
        if rate>=self.options['advance_at']:target=min(self.stages-1,target+1)
        elif rate<=self.options['retreat_at']:target=max(0,target-1)
        self.outcomes=[]
        if target!=self.frontier:
            self.frontier=target;self.epoch+=1;self.last_move=self.steps
            self.nominal_outcomes=[];self.nominal_ready=False

    def state_dict(self):
        return deepcopy(dict(stages=self.stages,options=self.options,frontier=self.frontier,epoch=self.epoch,
            steps=self.steps,last_move=self.last_move,outcomes=self.outcomes,nominal_outcomes=self.nominal_outcomes,
            nominal_ready=self.nominal_ready,rng=self.rng.bit_generator.state))

    def load_state_dict(self,s):
        if (s['stages']!=self.stages or s['options']!=self.options or not 0<=s['frontier']<self.stages
            or any(type(s[k]) is not int or s[k]<0 for k in ('frontier','epoch','steps','last_move'))
            or s['last_move']>s['steps'] or any(len(s[k])>self.options['window'] or any(type(x) is not bool for x in s[k]) for k in ('outcomes','nominal_outcomes'))):
            raise ValueError('Invalid curriculum checkpoint')
        for k in ('frontier','epoch','steps','last_move','outcomes','nominal_outcomes','nominal_ready'):setattr(self,k,deepcopy(s[k]))
        self.rng.bit_generator.state=deepcopy(s['rng'])
