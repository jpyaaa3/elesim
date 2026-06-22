from __future__ import annotations

from typing import Any, Sequence, Tuple

from engine.go2_locomotion.kinematics import GO2_LEG_JOINTS

# Unitree LowState motor_state index per Genesis leg joint name.
_UNITREE_MOTOR_BY_JOINT: dict[str, int] = {
    "FL_hip_joint": 3,
    "FL_thigh_joint": 4,
    "FL_calf_joint": 5,
    "FR_hip_joint": 0,
    "FR_thigh_joint": 1,
    "FR_calf_joint": 2,
    "RL_hip_joint": 9,
    "RL_thigh_joint": 10,
    "RL_calf_joint": 11,
    "RR_hip_joint": 6,
    "RR_thigh_joint": 7,
    "RR_calf_joint": 8,
}


def lowstate_leg_q_genesis_order(msg: Any) -> Tuple[float, ...]:
    """Map unitree_go LowState motor_state[0:12] -> Genesis leg joint order."""
    motors = getattr(msg, "motor_state", None)
    if motors is None or len(motors) < 12:
        raise ValueError("lowstate motor_state missing or too short")
    out: list[float] = []
    for joint_name in GO2_LEG_JOINTS:
        idx = _UNITREE_MOTOR_BY_JOINT[str(joint_name)]
        motor = motors[int(idx)]
        out.append(float(getattr(motor, "q", 0.0)))
    if len(out) != 12:
        raise ValueError(f"expected 12 leg joints, got {len(out)}")
    return tuple(out)


def as_leg_q(raw: Sequence[float]) -> Tuple[float, ...]:
    vals = [float(v) for v in raw]
    if len(vals) != 12:
        raise ValueError(f"expected 12 leg joint values, got {len(vals)}")
    return tuple(vals)
