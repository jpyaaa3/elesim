"""A physical admission check, never a policy command schedule."""
import math
import numpy as np

from .control import action_rates


def commands(settings, contract):
    start=np.asarray(contract['home_command'],float)
    end=np.asarray(contract['extend_command'],float)
    actions=max(1,math.ceil(float(np.max(abs(end-start)/action_rates(settings)))))
    ticks=actions*settings['action_ticks']
    return np.vstack((np.tile(start,(11,1)),np.linspace(start,end,ticks+1)[1:],
                      np.tile(end,(round(1/settings['dt']),1))))


class PreparationTracker:
    """Keep transient contacts/faults; a clean final frame cannot erase them."""
    def __init__(self, origin):
        self.origin=np.asarray(origin,float);self.contact_ticks=0
        self.first_contact_s=None;self.fault=False;self.max_displacement_mm=0.

    def update(self,time_s,position,contact,fault):
        self.contact_ticks+=bool(contact);self.fault|=bool(fault)
        if contact and self.first_contact_s is None:self.first_contact_s=time_s
        distance=float(np.linalg.norm((np.asarray(position)-self.origin)[:2])*1000)
        self.max_displacement_mm=max(self.max_displacement_mm,distance)

    def report(self,completed):
        return dict(passed=bool(completed and not self.fault and self.contact_ticks==0 and self.max_displacement_mm<=1.),
                    completed=bool(completed),contact_ticks=int(self.contact_ticks),first_contact_s=self.first_contact_s,
                    fault=self.fault,max_object_displacement_mm=self.max_displacement_mm)


def check(env):
    """Replay the declared preparation once in the selected object's scene."""
    import torch
    contract=env.experiment['source']['preparation']
    trajectory=commands(env.s,contract)
    env.reset();tracker=PreparationTracker(env.case['object_xyz_m'])
    for i,command in enumerate(trajectory[1:],1):
        env.tick(env.start.new_tensor(command).reshape(1,4))
        raw=env.scene.robot.get_contacts(exclude_self_contact=False,is_padded=True)
        active=raw['valid_mask']&(raw['force_a'].norm(dim=-1)>1e-4)
        contact=active&(torch.isin(raw['link_a'].long(),env.obj)|torch.isin(raw['link_b'].long(),env.obj))
        tracker.update(i*env.s['dt'],env.scene.object.get_pos()[0].detach().cpu().numpy(),
                       bool(contact.any()),bool(env.fault[0]))
        if tracker.fault:break
    report=tracker.report(i==len(trajectory)-1)
    report.update(tested=True,duration_s=i*env.s['dt'],criterion='Home→Extend Arm: no robot-object contact, no fault, <=1 mm object XY drift',
                  policy_schedule_forced=False)
    return report
