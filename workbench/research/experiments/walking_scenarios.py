"""Scenario presets used only by the offline walking experiments."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ArmPosePreset(str, Enum):
    NEUTRAL = "neutral"
    FORWARD_EXTENDED = "forward_extended"
    BENT_UPWARD = "bent_upward"
    BENT_SIDE = "bent_side"


@dataclass(frozen=True)
class ArmPoseValues:
    linear_m: float
    roll_rad: float
    theta1_rad: float
    theta2_rad: float


_ARM_PRESETS: dict[ArmPosePreset, ArmPoseValues] = {
    ArmPosePreset.NEUTRAL: ArmPoseValues(0.0, 0.0, 0.0, 0.0),
    ArmPosePreset.FORWARD_EXTENDED: ArmPoseValues(0.55, 0.0, 0.45, 0.45),
    ArmPosePreset.BENT_UPWARD: ArmPoseValues(0.15, 0.0, -0.55, -0.55),
    ArmPosePreset.BENT_SIDE: ArmPoseValues(0.20, 0.65, -0.35, -0.35),
}


def get_arm_pose(preset: ArmPosePreset | str) -> ArmPoseValues:
    key = preset if isinstance(preset, ArmPosePreset) else ArmPosePreset(str(preset).strip().lower())
    return _ARM_PRESETS[key]


def arm_pose_as_q(preset: ArmPosePreset | str) -> tuple[float, float, float, float]:
    pose = get_arm_pose(preset)
    return pose.linear_m, pose.roll_rad, pose.theta1_rad, pose.theta2_rad


def validate_arm_pose(preset, mapping) -> None:
    """Reject historical out-of-range presets rather than silently clamping them."""
    import math
    q = arm_pose_as_q(preset)
    bounds = ((mapping.linear_q_min_m, mapping.linear_q_max_m),
              (mapping.roll_q_min_rad, mapping.roll_q_max_rad),
              (mapping.seg1_q_min_rad, mapping.seg1_q_max_rad),
              (mapping.seg2_q_min_rad, mapping.seg2_q_max_rad))
    if any(not math.isfinite(value) or not lower <= value <= upper
           for value, (lower, upper) in zip(q, bounds)):
        raise ValueError(f"historical preset {preset!r} exceeds the configured joint limits: {q}")


BASELINE_SCENARIOS: tuple[tuple[ArmPosePreset, str, tuple[float, float, float], str], ...] = (
    (ArmPosePreset.NEUTRAL, "forward", (0.35, 0.0, 0.0), "none"),
    (ArmPosePreset.NEUTRAL, "backward", (-0.35, 0.0, 0.0), "none"),
    (ArmPosePreset.BENT_UPWARD, "backward", (-0.35, 0.0, 0.0), "none"),
    (ArmPosePreset.BENT_UPWARD, "forward", (0.35, 0.0, 0.0), "none"),
    (ArmPosePreset.BENT_SIDE, "turn", (0.0, 0.0, 0.5), "left"),
)
