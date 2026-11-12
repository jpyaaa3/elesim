"""GA acceptance travel limits, in canonical motor-position degrees.

No net-displacement subtraction, reward term, or post-lift PPO restriction.
Direction reversals at the hardware boundary preserve absolute increments.
Reverse GA continues evaluation after excess to measure task completion.
"""
import numpy as np
from elesim_protocol.messages import SimMappingConfig

GROUPS = ('linear', 'roll', 'seg1_plus_seg2')
LIMITS_DEG = np.array([200., 220., 600.])
_mapping = SimMappingConfig()
SCALE = np.array([
    (_mapping.linear_u_max-_mapping.linear_u_min)/(_mapping.linear_q_max_m-_mapping.linear_q_min_m),
    (_mapping.roll_u_max-_mapping.roll_u_min)/(_mapping.roll_q_max_rad-_mapping.roll_q_min_rad),
    (_mapping.seg_u_max-_mapping.seg_u_min)/(_mapping.seg1_q_max_rad-_mapping.seg1_q_min_rad),
    (_mapping.seg_u_max-_mapping.seg_u_min)/(_mapping.seg2_q_max_rad-_mapping.seg2_q_min_rad),
])


def increment(command, previous):
    # Linear sign and constant offsets disappear under absolute differences.
    return np.abs(np.asarray(command)-np.asarray(previous))*SCALE


def grouped(travel):
    a=np.asarray(travel)
    return np.stack((a[...,0],a[...,1],a[...,2]+a[...,3]),axis=-1)


def exceeded(travel):
    return grouped(travel)>LIMITS_DEG+1e-6


def excess(travel):
    """Per-group excess in motor degrees; respect the admission tolerance."""
    return np.where(exceeded(travel),np.maximum(grouped(travel)-LIMITS_DEG,0.),0.)


def require_feasible(start, target):
    minimum=grouped(increment(target,start))
    if np.any(minimum>LIMITS_DEG+1e-6):
        details=', '.join(f'{name}: minimum {value:.3f} > limit {limit:.3f} motor deg'
                         for name,value,limit in zip(GROUPS,minimum,LIMITS_DEG)
                         if value>limit+1e-6)
        raise ValueError('GA motion budget infeasible: '+details)


def contract():
    return dict(basis='sum(abs(delta motor-position command)); no net subtraction',
                groups=list(GROUPS),limits_motor_deg=LIMITS_DEG.tolist(),
                initial_roll_one_direction_motor_deg=180.,
                sim_to_motor_abs_scale=SCALE.tolist())
