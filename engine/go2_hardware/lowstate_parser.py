from __future__ import annotations

from dataclasses import dataclass
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


@dataclass(frozen=True)
class LowStateMotorSample:
    q: Tuple[float, ...]
    dq: Tuple[float, ...] | None = None
    torque_nm: Tuple[float, ...] | None = None


_DQ_FIELD_CANDIDATES: tuple[str, ...] = ("dq", "d_q", "vel", "velocity")
_TORQUE_FIELD_CANDIDATES: tuple[str, ...] = ("tau_est", "tau", "torque")


def _mapped_motors(msg: Any) -> list[Any]:
    motors = getattr(msg, "motor_state", None)
    if motors is None or len(motors) < 12:
        raise ValueError("lowstate motor_state missing or too short")
    return [motors[int(_UNITREE_MOTOR_BY_JOINT[str(joint_name)])] for joint_name in GO2_LEG_JOINTS]


def _read_required_series(motors: Sequence[Any], field: str) -> Tuple[float, ...]:
    out: list[float] = []
    for motor in motors:
        out.append(float(getattr(motor, field, 0.0)))
    if len(out) != 12:
        raise ValueError(f"expected 12 leg joint values, got {len(out)}")
    return tuple(out)


def _read_optional_series(motors: Sequence[Any], fields: Sequence[str]) -> Tuple[float, ...] | None:
    for field in fields:
        vals: list[float] = []
        ok = True
        for motor in motors:
            if not hasattr(motor, field):
                ok = False
                break
            try:
                vals.append(float(getattr(motor, field)))
            except (TypeError, ValueError):
                ok = False
                break
        if ok and len(vals) == 12:
            return tuple(vals)
    return None


def lowstate_motor_sample_genesis_order(msg: Any) -> LowStateMotorSample:
    """Map unitree_go LowState motor_state[0:12] -> Genesis leg joint order."""
    motors = _mapped_motors(msg)
    q = _read_required_series(motors, "q")
    dq = _read_optional_series(motors, _DQ_FIELD_CANDIDATES)
    torque_nm = _read_optional_series(motors, _TORQUE_FIELD_CANDIDATES)
    return LowStateMotorSample(q=q, dq=dq, torque_nm=torque_nm)


def lowstate_leg_q_genesis_order(msg: Any) -> Tuple[float, ...]:
    """Map unitree_go LowState motor_state[0:12] -> Genesis leg joint order."""
    return lowstate_motor_sample_genesis_order(msg).q


def as_leg_q(raw: Sequence[float]) -> Tuple[float, ...]:
    vals = [float(v) for v in raw]
    if len(vals) != 12:
        raise ValueError(f"expected 12 leg joint values, got {len(vals)}")
    return tuple(vals)
