from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as Rot


@dataclass(frozen=True)
class OdomSample:
    pos: Tuple[float, float, float]
    rpy: Tuple[float, float, float]
    lin_vel_body: Tuple[float, float, float]
    ang_vel_body: Tuple[float, float, float]
    timestamp_s: float
    # 12 leg joint angles (rad) in Genesis URDF order when available from /lf/lowstate.
    leg_q: Optional[Tuple[float, ...]] = None


def parse_odom_pose(
    position: Sequence[float],
    orientation_quat_xyzw: Sequence[float],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    pos = np.asarray(position, dtype=float).reshape(3)
    quat = np.asarray(orientation_quat_xyzw, dtype=float).reshape(4)
    rpy = Rot.from_quat(quat).as_euler("xyz", degrees=False)
    return (
        (float(pos[0]), float(pos[1]), float(pos[2])),
        (float(rpy[0]), float(rpy[1]), float(rpy[2])),
    )


def world_to_elesim(
    pos: Sequence[float],
    rpy: Sequence[float],
    offset_xyz: Sequence[float],
    yaw_deg: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    p = np.asarray(pos, dtype=float).reshape(3)
    offset = np.asarray(offset_xyz, dtype=float).reshape(3)
    yaw_rad = float(np.deg2rad(float(yaw_deg)))
    rot_yaw = Rot.from_euler("z", yaw_rad)
    p_out = rot_yaw.apply(p) + offset
    rpy_arr = np.asarray(rpy, dtype=float).reshape(3)
    rpy_out = (rot_yaw * Rot.from_euler("xyz", rpy_arr)).as_euler("xyz", degrees=False)
    return (
        (float(p_out[0]), float(p_out[1]), float(p_out[2])),
        (float(rpy_out[0]), float(rpy_out[1]), float(rpy_out[2])),
    )


def odom_msg_to_sample(
    *,
    position: Sequence[float],
    orientation_quat_xyzw: Sequence[float],
    lin_vel_world: Sequence[float],
    ang_vel_world: Sequence[float],
    timestamp_s: float,
    offset_xyz: Sequence[float],
    yaw_deg: float,
) -> OdomSample:
    pos, rpy = parse_odom_pose(position, orientation_quat_xyzw)
    pos, rpy = world_to_elesim(pos, rpy, offset_xyz, yaw_deg)
    quat = Rot.from_euler("xyz", np.asarray(rpy, dtype=float)).as_quat()
    rot = Rot.from_quat(quat)
    lin_world = np.asarray(lin_vel_world, dtype=float).reshape(3)
    ang_world = np.asarray(ang_vel_world, dtype=float).reshape(3)
    lin_body = rot.inv().apply(lin_world)
    ang_body = rot.inv().apply(ang_world)
    return OdomSample(
        pos=pos,
        rpy=rpy,
        lin_vel_body=(float(lin_body[0]), float(lin_body[1]), float(lin_body[2])),
        ang_vel_body=(float(ang_body[0]), float(ang_body[1]), float(ang_body[2])),
        timestamp_s=float(timestamp_s),
    )
