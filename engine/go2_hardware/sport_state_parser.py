from __future__ import annotations

from typing import Any, Sequence, Tuple

from engine.go2_hardware.odom_parser import OdomSample, world_to_elesim


def _as_xyz(raw: Sequence[float]) -> tuple[float, float, float]:
    arr = list(raw)
    if len(arr) < 3:
        raise ValueError(f"expected 3 values, got {len(arr)}")
    return (float(arr[0]), float(arr[1]), float(arr[2]))


def sportmodestate_to_sample(
    msg: Any,
    *,
    offset_xyz: Sequence[float],
    yaw_deg: float,
) -> OdomSample:
    """Parse unitree_go SportModeState (/sportmodestate) into host base state."""
    try:
        stamp = msg.stamp
        ts = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except Exception:
        ts = 0.0

    pos, rpy = world_to_elesim(
        _as_xyz(msg.position),
        _as_xyz(msg.imu_state.rpy),
        offset_xyz,
        yaw_deg,
    )
    vel_raw = getattr(msg, "velocity", None)
    if vel_raw is None:
        lin_body = (0.0, 0.0, 0.0)
    else:
        lin_body = _as_xyz(vel_raw)

    gyro_raw = getattr(getattr(msg, "imu_state", None), "gyroscope", None)
    if gyro_raw is not None:
        ang_body = _as_xyz(gyro_raw)
    else:
        ang_body = (0.0, 0.0, float(getattr(msg, "yaw_speed", 0.0)))

    return OdomSample(
        pos=pos,
        rpy=rpy,
        lin_vel_body=lin_body,
        ang_vel_body=ang_body,
        timestamp_s=float(ts),
    )
