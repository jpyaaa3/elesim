"""Pure NumPy endpoint and permanent-rejection rules; angles use total bends."""
import numpy as np

SCALE = np.array([1.,1.,5.,5.])
POSE_TOL = np.array([.002,*np.deg2rad([2.,3.,3.])])
SPEED_TOL = np.array([.002,*np.deg2rad([2.,2.,2.])])
REASONS = ('physical_fault','wrong_face','xy_limit','tilt_limit','command_bounds','command_slew')


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


def violations(s, command, previous, lower, upper, rate, origin):
    xy=np.linalg.norm(s['object_pos'][:,:2]-np.asarray(origin)[:2],axis=-1)
    angle=tilt(s['object_quat'])
    mask=np.stack((s['fault'],s['wrong'],xy>.05,angle>np.deg2rad(10),
        np.any((command<lower-1e-7)|(command>upper+1e-7),axis=-1),
        np.any(abs(command-previous)>rate+1e-7,axis=-1)),axis=-1)
    return mask,xy,angle


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
        self.window=np.zeros(n,int);self.ticks=np.zeros(n,int);self.valid_ticks=np.zeros(n,int)
        self.first_tick=np.full(n,-1,int);self.reasons=np.zeros((n,len(REASONS)),bool)
        self.last_valid_error=np.asarray(initial_error).copy();self.home_error=self.last_valid_error.copy()
        self.path=np.zeros(n);self.xy=np.zeros(n);self.angle=np.zeros(n);self.force=np.zeros(n)

    def update(self,s,command,previous,home,lower,upper,rate,origin):
        active=self.active.copy();mask,xy,angle=violations(s,command,previous,lower,upper,rate,origin)
        bad=mask.any(-1)&active
        self.ticks+=active;self.first_tick[bad]=self.ticks[bad];self.reasons[bad]=mask[bad]
        self.rejected|=bad
        self.home_error[active]=error(s['actual'],home)[active]
        valid=active&~bad;self.last_valid_error[valid]=self.home_error[valid];self.valid_ticks+=valid
        self.path+=np.where(active,np.sum(abs(command-previous)/(upper-lower),axis=-1),0)
        self.xy=np.maximum(self.xy,np.where(active,xy,0));self.angle=np.maximum(self.angle,np.where(active,angle,0))
        self.force[active]=s['contact_force'][active]
        good=home_good(s,command,home)&valid
        self.window[active]=np.where(good[active],self.window[active]+1,0)
        self.success|=valid&(self.window>=round(1/self.dt))
        self.active&=~(self.success|self.rejected)
        return active

    def reports(self):
        return [dict(candidate=i,success=bool(self.success[i]),rejected=bool(self.rejected[i]),
            first_violation_tick=int(self.first_tick[i]),reasons=[r for r,b in zip(REASONS,self.reasons[i]) if b],
            last_valid_error=float(self.last_valid_error[i]),valid_ticks=int(self.valid_ticks[i]),
            duration_s=float(self.ticks[i]*self.dt),home_window_s=float(self.window[i]*self.dt),
            home_error=float(self.home_error[i]),path_length=float(self.path[i]),peak_xy_m=float(self.xy[i]),
            peak_tilt_rad=float(self.angle[i]),object_contact_force_n=float(self.force[i])) for i in range(len(self.active))]
