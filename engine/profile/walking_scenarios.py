from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Tuple


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


# Display-space defaults; tune against crafts URDF joint limits.
_ARM_PRESETS: dict[ArmPosePreset, ArmPoseValues] = {
    ArmPosePreset.NEUTRAL: ArmPoseValues(0.0, 0.0, 0.0, 0.0),
    ArmPosePreset.FORWARD_EXTENDED: ArmPoseValues(0.55, 0.0, 0.45, 0.45),
    ArmPosePreset.BENT_UPWARD: ArmPoseValues(0.15, 0.0, -0.55, -0.55),
    ArmPosePreset.BENT_SIDE: ArmPoseValues(0.20, 0.65, -0.35, -0.35),
}


def get_arm_pose(preset: ArmPosePreset | str) -> ArmPoseValues:
    key = ArmPosePreset(str(preset).strip().lower())
    return _ARM_PRESETS[key]


def arm_pose_as_q(preset: ArmPosePreset | str) -> Tuple[float, float, float, float]:
    p = get_arm_pose(preset)
    return (p.linear_m, p.roll_rad, p.theta1_rad, p.theta2_rad)


BASELINE_SCENARIOS: tuple[tuple[ArmPosePreset, str, tuple[float, float, float], str], ...] = (
    (ArmPosePreset.NEUTRAL, "forward", (0.35, 0.0, 0.0), "none"),
    (ArmPosePreset.NEUTRAL, "backward", (-0.35, 0.0, 0.0), "none"),
    (ArmPosePreset.BENT_UPWARD, "backward", (-0.35, 0.0, 0.0), "none"),
    (ArmPosePreset.BENT_UPWARD, "forward", (0.35, 0.0, 0.0), "none"),
    (ArmPosePreset.BENT_SIDE, "turn", (0.0, 0.0, 0.5), "left"),
)
