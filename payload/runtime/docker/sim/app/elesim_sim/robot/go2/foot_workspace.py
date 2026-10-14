"""GO2 leg workspace checks in the URDF base frame (metres/radians)."""
from __future__ import annotations
import numpy as np
from .locomotion.kinematics import HIP_OFFSET_BODY
from .locomotion.types import ALL_LEGS

_LENGTH = .213
_LATERAL = .0955


def foot_position_body(leg, angles):
    hip, thigh, calf = np.asarray(angles, dtype=float)
    side = 1. if leg in (0, 2) else -1.
    x = -_LENGTH * (np.sin(thigh) + np.sin(thigh + calf))
    z = -_LENGTH * (np.cos(thigh) + np.cos(thigh + calf))
    y = side * _LATERAL
    return HIP_OFFSET_BODY[ALL_LEGS[leg]] + np.array([
        x, np.cos(hip)*y - np.sin(hip)*z, np.sin(hip)*y + np.cos(hip)*z])


def foot_angles_body(leg, position):
    """Lower-leg, bent-knee IK branch; None means no geometric solution."""
    x, y, z = np.asarray(position, dtype=float) - HIP_OFFSET_BODY[ALL_LEGS[leg]]
    lateral = _LATERAL if leg in (0, 2) else -_LATERAL
    radial2 = y*y + z*z - lateral*lateral
    if not np.isfinite([x, y, z]).all() or radial2 <= 0:
        return None
    down = -np.sqrt(radial2)
    cosine = (x*x + down*down - 2*_LENGTH**2) / (2*_LENGTH**2)
    if abs(cosine) > 1 + 1e-10:
        return None
    calf = -np.arccos(np.clip(cosine, -1., 1.))
    hip = (np.arctan2(z, y) - np.arctan2(down, lateral) + np.pi) % (2*np.pi) - np.pi
    thigh = np.arctan2(-x, -down) - np.arctan2(np.sin(calf), 1 + np.cos(calf))
    return np.array([hip, thigh, calf])


def reachable(leg, target_world, base_position, base_rotation, lower, upper):
    q = foot_angles_body(leg, np.asarray(base_rotation).T @ (target_world-base_position))
    return q is not None and bool(np.all(q >= lower) and np.all(q <= upper))


def project_ground_target(leg, target_world, base_position, base_rotation, lower, upper):
    """Return target and feasibility, preserving the requested ground height."""
    target = np.asarray(target_world, dtype=float).copy()
    base_position = np.asarray(base_position, dtype=float)
    base_rotation = np.asarray(base_rotation, dtype=float)
    if reachable(leg, target, base_position, base_rotation, lower, upper):
        return target, True
    # Pull an overextended target toward the nominal footprint, on the same
    # horizontal surface. Never manufacture a successful airborne landing.
    neutral = foot_position_body(leg, [.1 if leg in (0, 2) else -.1, .9, -1.8])
    anchor = base_position + base_rotation @ neutral
    anchor[2] = target[2]
    previous = 0.
    for fraction in np.linspace(1/24, 1., 24):
        point = target + fraction * (anchor-target)
        if reachable(leg, point, base_position, base_rotation, lower, upper):
            low, high = previous, fraction
            for _ in range(16):
                middle = (low+high)/2
                if reachable(leg, target+middle*(anchor-target), base_position, base_rotation, lower, upper):
                    high = middle
                else:
                    low = middle
            return target+high*(anchor-target), True
        previous = fraction
    # This bounded search is not a complete workspace solver. The body may
    # be too high/tilted, or another ground location may require a new step.
    return target, False
