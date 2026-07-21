"""Virtual arm rates matching the configured Dynamixel profile convention."""

from __future__ import annotations

import math

import elesim_protocol.messages as proto


PROFILE_VELOCITY_UNIT_DEG_S = 0.229 * 6.0
ROLL_PROFILE_VELOCITY = 240
BEND_PROFILE_VELOCITY = 60


def estimate_ideal_sim_rates(mapping: proto.SimMappingConfig) -> tuple[float, float]:
    roll_u_per_s = ROLL_PROFILE_VELOCITY * PROFILE_VELOCITY_UNIT_DEG_S
    bend_u_per_s = BEND_PROFILE_VELOCITY * PROFILE_VELOCITY_UNIT_DEG_S
    roll_rad_per_u = (mapping.roll_q_max_rad - mapping.roll_q_min_rad) / max(
        1e-9, mapping.roll_u_max - mapping.roll_u_min
    )
    bend1_rad_per_u = (mapping.seg1_q_max_rad - mapping.seg1_q_min_rad) / max(
        1e-9, mapping.seg_u_max - mapping.seg_u_min
    )
    bend2_rad_per_u = (mapping.seg2_q_max_rad - mapping.seg2_q_min_rad) / max(
        1e-9, mapping.seg_u_max - mapping.seg_u_min
    )
    roll_rate = abs(roll_u_per_s * roll_rad_per_u)
    bend_rate = min(abs(bend_u_per_s * bend1_rad_per_u), abs(bend_u_per_s * bend2_rad_per_u))
    if not math.isfinite(roll_rate) or not math.isfinite(bend_rate):
        raise ValueError("arm mapping produced non-finite rates")
    return float(roll_rate), float(bend_rate)
