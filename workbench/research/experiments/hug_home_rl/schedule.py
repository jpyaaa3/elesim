"""Overlapping reset windows; only the new hard edge can promote progress."""
import numpy as np
from workbench.research.experiments.hug_reverse_rl.schedule import Curriculum,DEFAULTS as BASE,validate_options

DEFAULTS=dict(BASE,overlap_bins=3,hard_fraction=.5)


def validate(options):
    if set(options)!=set(DEFAULTS):raise ValueError('Unknown/missing window option')
    validate_options({k:options[k] for k in BASE})
    if type(options['overlap_bins']) is not int or options['overlap_bins']<2 or not 0<options['hard_fraction']<=1:
        raise ValueError('Invalid overlap window/hard fraction')


class WindowCurriculum(Curriculum):
    def __init__(self,stages,options=None):
        options=dict(DEFAULTS,**(options or {}));validate(options)
        super().__init__(stages,{k:options[k] for k in BASE});self.options=options

    def sample(self,count):
        bins=np.full(count,self.frontier,int)
        if self.frontier:
            retained=self.rng.random(count)<self.options['retention_fraction']
            bins[retained]=self.rng.integers(0,self.frontier,size=int(retained.sum()))
            overlap=~retained&(self.rng.random(count)>self.options['hard_fraction'])
            low=max(0,self.frontier-self.options['overlap_bins']+1)
            bins[overlap]=self.rng.integers(low,self.frontier,size=int(overlap.sum()))
        if self.nominal_enabled:
            nominal=self.rng.random(count)<self.options['nominal_fraction'];bins[nominal]=-1
        return bins,np.full(count,self.epoch,int)
