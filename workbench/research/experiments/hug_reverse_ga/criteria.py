"""Task completion, permanent disqualification and immediate-stop rules."""
import numpy as np
from .motion_budget import increment,exceeded,grouped,contract,excess,LIMITS_DEG

SCALE = np.array([1.,1.,5.,5.])
POSE_TOL = np.array([.002,*np.deg2rad([2.,3.,3.])])
SPEED_TOL = np.array([.002,*np.deg2rad([2.,2.,2.])])
REASONS = ('physical_fault','wrong_face','xy_limit','tilt_limit','command_bounds','command_slew','initial_roll_reversal','linear_travel','roll_travel','bend_travel')
EVALUATION_POLICY = 'reverse-completion-before-budget-ranking-v2'


def tilt(quat):
    q=np.asarray(quat,float)
    norm=np.linalg.norm(q,axis=-1)
    if not np.isfinite(q).all() or np.any(abs(norm-1)>.01):
        raise RuntimeError('Invalid cylinder quaternion')
    q=q/norm[...,None]
    return np.arccos(np.clip(1-2*(q[...,1]**2+q[...,2]**2),-1,1))


def error(actual, target):
    return np.max(abs((actual-target)*SCALE)/POSE_TOL,axis=-1)


def object_stable(s):
    return (s['floor'] & (np.linalg.norm(s['object_vel'],axis=-1)<=.01)
            & (np.linalg.norm(s['object_ang'],axis=-1)<=np.deg2rad(5)))


def violations(s, command, previous, lower, upper, rate, origin, travel=None):
    xy=np.linalg.norm(s['object_pos'][:,:2]-np.asarray(origin)[:2],axis=-1)
    angle=tilt(s['object_quat'])
    mask=np.stack((s['fault'],s['wrong'],xy>.05,angle>np.deg2rad(10),
        np.any((command<lower-1e-7)|(command>upper+1e-7),axis=-1),
        np.any(abs(command-previous)>rate+1e-7,axis=-1)),axis=-1)
    budget=np.zeros((*mask.shape[:-1],3),bool) if travel is None else exceeded(travel)
    return np.concatenate((mask,np.zeros((*mask.shape[:-1],1),bool),budget),axis=-1),xy,angle


def admission_good(s, goal):
    return (np.all(s['touch']>0,axis=-1) & np.all(s['actual'][:,2:]<0,axis=-1)
            & (error(s['actual'],goal)<=1) & object_stable(s))


def home_good(s, command, home):
    return ((error(s['actual'],home)<=1) & np.all(abs(s['actual_vel']*SCALE)<=SPEED_TOL,axis=-1)
            & np.all(abs(command-home)<=1e-7,axis=-1) & (s['contact_force']<=1e-4)
            & (s['contact_penetration']<=.001) & object_stable(s))


class Tracker:
    def __init__(self,n,initial_error,dt):
        self.dt=dt;self.active=np.ones(n,bool);self.rejected=np.zeros(n,bool);self.success=np.zeros(n,bool)
        self.hard_stopped=np.zeros(n,bool);self.completed=np.zeros(n,bool)
        self.window=np.zeros(n,int);self.ticks=np.zeros(n,int);self.valid_ticks=np.zeros(n,int)
        self.first_tick=np.full(n,-1,int);self.reasons=np.zeros((n,len(REASONS)),bool)
        self.first_hard_tick=np.full(n,-1,int)
        self.last_valid_error=np.asarray(initial_error).copy();self.home_error=self.last_valid_error.copy()
        self.last_safe_error=self.last_valid_error.copy()
        self.travel=np.zeros((n,4))
        self.roll_direction=np.zeros(n)
        self.roll_initial_deg=np.zeros(n)
        self.roll_unlocked=np.zeros(n,bool)
        self.path=np.zeros(n);self.xy=np.zeros(n);self.angle=np.zeros(n);self.force=np.zeros(n)

    def update(self,s,command,previous,home,lower,upper,rate,origin):
        active=self.active.copy()
        delta=np.asarray(command)[:,1]-np.asarray(previous)[:,1]
        moving=active&(abs(delta)>1e-10)
        first=moving&(self.roll_direction==0)
        self.roll_direction[first]=np.sign(delta[first])
        reversal=moving&~self.roll_unlocked&(np.sign(delta)!=self.roll_direction)
        step=increment(command,previous)
        self.roll_initial_deg+=np.where(moving&~self.roll_unlocked&~reversal,step[:,1],0)
        self.roll_unlocked|=self.roll_initial_deg>=180.-1e-6
        self.travel+=np.where(active[:,None],step,0)
        mask,xy,angle=violations(s,command,previous,lower,upper,rate,origin,self.travel)
        mask[:,-4]=reversal
        bad=mask.any(-1)&active
        # Budget violations disqualify, but do not censor the rest of the path.
        hard_bad=mask[:,:-3].any(-1)&active
        self.ticks+=active
        first_bad=bad&(self.first_tick<0)
        self.first_tick[first_bad]=self.ticks[first_bad]
        self.first_hard_tick[hard_bad]=self.ticks[hard_bad]
        self.reasons[active]|=mask[active]
        self.rejected|=bad
        self.hard_stopped|=hard_bad
        self.home_error[active]=error(s['actual'],home)[active]
        valid=active&~bad;self.last_valid_error[valid]=self.home_error[valid];self.valid_ticks+=valid
        safe=active&~hard_bad;self.last_safe_error[safe]=self.home_error[safe]
        self.path+=np.where(active,np.sum(abs(command-previous)/(upper-lower),axis=-1),0)
        self.xy=np.maximum(self.xy,np.where(active,xy,0));self.angle=np.maximum(self.angle,np.where(active,angle,0))
        self.force[active]=s['contact_force'][active]
        good=home_good(s,command,home)&safe
        self.window[active]=np.where(good[active],self.window[active]+1,0)
        # Completion still requires the exact Home command and continuous rest.
        self.completed|=safe&(self.window>=round(1/self.dt))
        self.success=self.completed&~self.rejected
        self.active&=~(self.completed|self.hard_stopped)
        return active

    def reports(self):
        over=excess(self.travel);ratios=over/LIMITS_DEG
        return [dict(candidate=i,success=bool(self.success[i]),rejected=bool(self.rejected[i]),
            completed=bool(self.completed[i]),hard_stopped=bool(self.hard_stopped[i]),
            evaluation_policy=EVALUATION_POLICY,
            evaluation_status=('success' if self.success[i] else 'hard_stop' if self.hard_stopped[i]
                else 'completed_over_budget' if self.completed[i]
                else 'timeout_over_budget' if self.rejected[i] else 'timeout'),
            first_violation_tick=int(self.first_tick[i]),reasons=[r for r,b in zip(REASONS,self.reasons[i]) if b],
            first_hard_violation_tick=int(self.first_hard_tick[i]),last_safe_error=float(self.last_safe_error[i]),
            last_valid_error=float(self.last_valid_error[i]),valid_ticks=int(self.valid_ticks[i]),
            duration_s=float(self.ticks[i]*self.dt),home_window_s=float(self.window[i]*self.dt),
            motion_motor_deg=self.travel[i].tolist(),motion_group_motor_deg=grouped(self.travel[i]).tolist(),motion_budget=contract(),
            motion_excess_motor_deg=over[i].tolist(),motion_excess_ratio=ratios[i].tolist(),
            motion_excess_max_ratio=float(ratios[i].max()),motion_excess_sum_ratio=float(ratios[i].sum()),
            home_error=float(self.home_error[i]),path_length=float(self.path[i]),peak_xy_m=float(self.xy[i]),
            peak_tilt_rad=float(self.angle[i]),object_contact_force_n=float(self.force[i])) for i in range(len(self.active))]
