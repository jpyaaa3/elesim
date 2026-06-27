#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class ControlU:
    u_linear: float
    u_roll: float
    u_s1: float
    u_s2: float


@dataclass(frozen=True)
class SimQ:
    linear_m: float
    roll_rad: float
    theta1_rad: float
    theta2_rad: float


@dataclass(frozen=True)
class SimMappingConfig:
    linear_u_min: float = 0.0
    linear_u_max: float = 360.0
    # Hardware travel cap in motor [u] units; mapping scale stays linear_u_max.
    linear_u_limit: float = 250.0
    roll_u_min: float = 0.0
    roll_u_max: float = 360.0
    seg_u_min: float = 0.0
    seg_u_max: float = 360.0

    linear_q_min_m: float = -0.230
    linear_q_max_m: float = 0.010
    roll_q_min_rad: float = -math.pi / 2.0
    roll_q_max_rad: float = +math.pi / 2.0
    seg1_q_min_rad: float = -math.radians(36.0)
    seg1_q_max_rad: float = +math.radians(36.0)
    seg2_q_min_rad: float = -math.radians(36.0)
    seg2_q_max_rad: float = +math.radians(36.0)

    command_direction: tuple[int, int, int, int] = (-1, 1, 1, 1)


def _clamp(x: float, lo: float, hi: float) -> float:
    return float(min(max(float(x), float(lo)), float(hi)))


def linear_motor_u_limit(cfg: SimMappingConfig) -> float:
    return float(min(float(cfg.linear_u_max), float(cfg.linear_u_limit)))


def clamp_linear_motor_u(u_linear: float, cfg: SimMappingConfig) -> float:
    return _clamp(u_linear, float(cfg.linear_u_min), linear_motor_u_limit(cfg))


def _apply_axis_direction(u_value: float, direction: int, u_min: float, u_max: float) -> float:
    if int(direction) < 0:
        return float(u_min) + float(u_max) - float(u_value)
    return float(u_value)


def _map_axis_to_u(q_value: float, q_min: float, q_max: float, u_min: float, u_max: float) -> float:
    q_lo = min(float(q_min), float(q_max))
    q_hi = max(float(q_min), float(q_max))
    q_value = _clamp(q_value, q_lo, q_hi)
    if abs(float(q_max) - float(q_min)) < 1e-12:
        return float(u_min)
    ratio = (float(q_value) - float(q_min)) / (float(q_max) - float(q_min))
    return _clamp(float(u_min) + ratio * (float(u_max) - float(u_min)), u_min, u_max)


def _map_u_to_axis(u_value: float, u_min: float, u_max: float, q_min: float, q_max: float) -> float:
    u_value = _clamp(u_value, u_min, u_max)
    if abs(float(u_max) - float(u_min)) < 1e-12:
        return float(q_min)
    ratio = (float(u_value) - float(u_min)) / (float(u_max) - float(u_min))
    q_value = float(q_min) + ratio * (float(q_max) - float(q_min))
    return _clamp(q_value, min(float(q_min), float(q_max)), max(float(q_min), float(q_max)))


def sim_q_to_motor_deg(q: SimQ, cfg: SimMappingConfig = SimMappingConfig()) -> ControlU:
    return ControlU(
        u_linear=clamp_linear_motor_u(
            _map_axis_to_u(q.linear_m, cfg.linear_q_min_m, cfg.linear_q_max_m, cfg.linear_u_min, cfg.linear_u_max),
            cfg,
        ),
        u_roll=_map_axis_to_u(q.roll_rad, cfg.roll_q_min_rad, cfg.roll_q_max_rad, cfg.roll_u_min, cfg.roll_u_max),
        u_s1=_map_axis_to_u(q.theta1_rad, cfg.seg1_q_min_rad, cfg.seg1_q_max_rad, cfg.seg_u_min, cfg.seg_u_max),
        u_s2=_map_axis_to_u(q.theta2_rad, cfg.seg2_q_min_rad, cfg.seg2_q_max_rad, cfg.seg_u_min, cfg.seg_u_max),
    )


def motor_deg_to_sim_q(u: ControlU, cfg: SimMappingConfig = SimMappingConfig()) -> SimQ:
    return SimQ(
        linear_m=_map_u_to_axis(u.u_linear, cfg.linear_u_min, cfg.linear_u_max, cfg.linear_q_min_m, cfg.linear_q_max_m),
        roll_rad=_map_u_to_axis(u.u_roll, cfg.roll_u_min, cfg.roll_u_max, cfg.roll_q_min_rad, cfg.roll_q_max_rad),
        theta1_rad=_map_u_to_axis(u.u_s1, cfg.seg_u_min, cfg.seg_u_max, cfg.seg1_q_min_rad, cfg.seg1_q_max_rad),
        theta2_rad=_map_u_to_axis(u.u_s2, cfg.seg_u_min, cfg.seg_u_max, cfg.seg2_q_min_rad, cfg.seg2_q_max_rad),
    )


def control_u_to_sim_q(u: ControlU, cfg: SimMappingConfig = SimMappingConfig()) -> SimQ:
    dirs = tuple(int(v) for v in cfg.command_direction)
    motor_u = ControlU(
        u_linear=_clamp(
            _apply_axis_direction(
                clamp_linear_motor_u(u.u_linear, cfg),
                dirs[0],
                cfg.linear_u_min,
                cfg.linear_u_max,
            ),
            cfg.linear_u_min,
            cfg.linear_u_max,
        ),
        u_roll=_clamp(_apply_axis_direction(u.u_roll, dirs[1], cfg.roll_u_min, cfg.roll_u_max), cfg.roll_u_min, cfg.roll_u_max),
        u_s1=_clamp(_apply_axis_direction(u.u_s1, dirs[2], cfg.seg_u_min, cfg.seg_u_max), cfg.seg_u_min, cfg.seg_u_max),
        u_s2=_clamp(_apply_axis_direction(u.u_s2, dirs[3], cfg.seg_u_min, cfg.seg_u_max), cfg.seg_u_min, cfg.seg_u_max),
    )
    return motor_deg_to_sim_q(motor_u, cfg)


def sim_q_to_control_u(q: SimQ, cfg: SimMappingConfig = SimMappingConfig()) -> ControlU:
    dirs = tuple(int(v) for v in cfg.command_direction)
    motor_u = sim_q_to_motor_deg(q, cfg)
    return ControlU(
        u_linear=_apply_axis_direction(motor_u.u_linear, dirs[0], cfg.linear_u_min, cfg.linear_u_max),
        u_roll=_apply_axis_direction(motor_u.u_roll, dirs[1], cfg.roll_u_min, cfg.roll_u_max),
        u_s1=_apply_axis_direction(motor_u.u_s1, dirs[2], cfg.seg_u_min, cfg.seg_u_max),
        u_s2=_apply_axis_direction(motor_u.u_s2, dirs[3], cfg.seg_u_min, cfg.seg_u_max),
    )


def now_s() -> float:
    return time.time()


def dumps_msg(msg: Dict[str, Any]) -> bytes:
    return json.dumps(msg, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def loads_msg(buf: bytes) -> Dict[str, Any]:
    return json.loads(buf.decode("utf-8"))


def pack_state(
    *,
    u: Optional[ControlU] = None,
    q: Optional[SimQ] = None,
    ts: Optional[float] = None,
    torque_enabled: Optional[bool] = None,
    ik_target_xyz: Optional[tuple[float, float, float]] = None,
    ik_target_dir: Optional[tuple[float, float, float]] = None,
    actual_tip_xyz: Optional[tuple[float, float, float]] = None,
    actual_tip_dir: Optional[tuple[float, float, float]] = None,
    sag_model: Optional[dict[str, Any]] = None,
    claw_closed: Optional[bool] = None,
    claw_current: Optional[int] = None,
    motor_currents_ma: Optional[dict[str, int]] = None,
    safety_fault: Optional[str] = None,
    perceived_object_label: Optional[str] = None,
    perceived_object_confidence: Optional[float] = None,
    perceived_object_camera: Optional[tuple[float, float, float]] = None,
    perceived_center_uv: Optional[tuple[float, float]] = None,
    perceived_scale: Optional[float] = None,
    perceived_timestamp_s: Optional[float] = None,
    debug_markers: Optional[list[dict[str, Any]]] = None,
    go2_vel: Optional[tuple[float, float, float]] = None,
    go2_base_rpy: Optional[tuple[float, float, float]] = None,
    go2_base_pos: Optional[tuple[float, float, float]] = None,
    go2_base_lin_vel_body: Optional[tuple[float, float, float]] = None,
    go2_base_ang_vel: Optional[tuple[float, float, float]] = None,
    go2_base_timestamp_s: Optional[float] = None,
    go2_leg_q: Optional[tuple[float, ...]] = None,
    go2_leg_dq: Optional[tuple[float, ...]] = None,
    go2_leg_torque_nm: Optional[tuple[float, ...]] = None,
    go2_sport_pose: Optional[str] = None,
    go2_sport_pose_seq: Optional[int] = None,
    go2_obstacles_avoid_enabled: Optional[bool] = None,
    go2_obstacles_avoid_seq: Optional[int] = None,
    sim_reset_seq: Optional[int] = None,
    sim_time_s: Optional[float] = None,
    sim_wall_elapsed_s: Optional[float] = None,
    sim_realtime_factor: Optional[float] = None,
    sim_step_count: Optional[int] = None,
) -> Dict[str, Any]:
    ts = now_s() if ts is None else float(ts)
    out: Dict[str, Any] = {"t": "state", "ts": ts}
    if u is not None:
        out["u"] = {"linear": u.u_linear, "roll": u.u_roll, "s1": u.u_s1, "s2": u.u_s2}
    if q is not None:
        out["q"] = {
            "linear_m": q.linear_m,
            "roll_rad": q.roll_rad,
            "theta1_rad": q.theta1_rad,
            "theta2_rad": q.theta2_rad,
        }
    if torque_enabled is not None:
        out["torque_enabled"] = bool(torque_enabled)
    if ik_target_xyz is not None:
        out["ik_target"] = [float(ik_target_xyz[0]), float(ik_target_xyz[1]), float(ik_target_xyz[2])]
    if ik_target_dir is not None:
        out["ik_target_dir"] = [float(ik_target_dir[0]), float(ik_target_dir[1]), float(ik_target_dir[2])]
    if actual_tip_xyz is not None:
        out["actual_tip"] = [float(actual_tip_xyz[0]), float(actual_tip_xyz[1]), float(actual_tip_xyz[2])]
    if actual_tip_dir is not None:
        out["actual_tip_dir"] = [float(actual_tip_dir[0]), float(actual_tip_dir[1]), float(actual_tip_dir[2])]
    if sag_model is not None:
        out["sag_model"] = dict(sag_model)
    if claw_closed is not None:
        out["claw_closed"] = bool(claw_closed)
    if claw_current is not None:
        out["claw_current"] = int(claw_current)
    if motor_currents_ma is not None:
        out["motor_currents_ma"] = {str(k): int(v) for k, v in motor_currents_ma.items()}
    if safety_fault is not None:
        out["safety_fault"] = str(safety_fault)
    if perceived_object_label is not None:
        out["perceived_object_label"] = str(perceived_object_label)
    if perceived_object_confidence is not None:
        out["perceived_object_confidence"] = float(perceived_object_confidence)
    if perceived_object_camera is not None:
        out["perceived_object_camera"] = [
            float(perceived_object_camera[0]),
            float(perceived_object_camera[1]),
            float(perceived_object_camera[2]),
        ]
    if perceived_center_uv is not None:
        out["perceived_center_uv"] = [float(perceived_center_uv[0]), float(perceived_center_uv[1])]
    if perceived_scale is not None:
        out["perceived_scale"] = float(perceived_scale)
    if perceived_timestamp_s is not None:
        out["perceived_timestamp_s"] = float(perceived_timestamp_s)
    if go2_vel is not None:
        out["go2_vel"] = [float(go2_vel[0]), float(go2_vel[1]), float(go2_vel[2])]
    if go2_base_rpy is not None:
        out["go2_base_rpy"] = [float(go2_base_rpy[0]), float(go2_base_rpy[1]), float(go2_base_rpy[2])]
    if go2_base_pos is not None:
        out["go2_base_pos"] = [float(go2_base_pos[0]), float(go2_base_pos[1]), float(go2_base_pos[2])]
    if go2_base_lin_vel_body is not None:
        out["go2_base_lin_vel_body"] = [
            float(go2_base_lin_vel_body[0]),
            float(go2_base_lin_vel_body[1]),
            float(go2_base_lin_vel_body[2]),
        ]
    if go2_base_ang_vel is not None:
        out["go2_base_ang_vel"] = [
            float(go2_base_ang_vel[0]),
            float(go2_base_ang_vel[1]),
            float(go2_base_ang_vel[2]),
        ]
    if go2_base_timestamp_s is not None:
        out["go2_base_timestamp_s"] = float(go2_base_timestamp_s)
    if go2_leg_q is not None and len(go2_leg_q) == 12:
        out["go2_leg_q"] = [float(v) for v in go2_leg_q]
    if go2_leg_dq is not None and len(go2_leg_dq) == 12:
        out["go2_leg_dq"] = [float(v) for v in go2_leg_dq]
    if go2_leg_torque_nm is not None and len(go2_leg_torque_nm) == 12:
        out["go2_leg_torque_nm"] = [float(v) for v in go2_leg_torque_nm]
    if go2_sport_pose is not None:
        out["go2_sport_pose"] = str(go2_sport_pose).strip().lower()
    if go2_sport_pose_seq is not None:
        out["go2_sport_pose_seq"] = int(go2_sport_pose_seq)
    if go2_obstacles_avoid_enabled is not None:
        out["go2_obstacles_avoid_enabled"] = bool(go2_obstacles_avoid_enabled)
    if go2_obstacles_avoid_seq is not None:
        out["go2_obstacles_avoid_seq"] = int(go2_obstacles_avoid_seq)
    if sim_reset_seq is not None:
        out["sim_reset_seq"] = int(sim_reset_seq)
    if sim_time_s is not None:
        out["sim_time_s"] = float(sim_time_s)
    if sim_wall_elapsed_s is not None:
        out["sim_wall_elapsed_s"] = float(sim_wall_elapsed_s)
    if sim_realtime_factor is not None:
        out["sim_realtime_factor"] = float(sim_realtime_factor)
    if sim_step_count is not None:
        out["sim_step_count"] = int(sim_step_count)
    if debug_markers is not None:
        packed_markers: list[dict[str, Any]] = []
        for raw in list(debug_markers):
            if not isinstance(raw, dict):
                continue
            name = str(raw.get("name", "")).strip()
            frame = str(raw.get("frame", "world")).strip() or "world"
            pos = raw.get("pos", None)
            if not name or not isinstance(pos, (list, tuple)) or len(pos) != 3:
                continue
            marker: dict[str, Any] = {
                "name": name,
                "frame": frame,
                "pos": [float(pos[0]), float(pos[1]), float(pos[2])],
            }
            direction = raw.get("dir", None)
            if isinstance(direction, (list, tuple)) and len(direction) == 3:
                marker["dir"] = [float(direction[0]), float(direction[1]), float(direction[2])]
            color = raw.get("color", None)
            if isinstance(color, (list, tuple)) and len(color) in (3, 4):
                marker["color"] = [float(v) for v in color]
            if "radius" in raw:
                marker["radius"] = float(raw.get("radius", 0.0))
            if "length" in raw:
                marker["length"] = float(raw.get("length", 0.0))
            if "ttl_ms" in raw:
                marker["ttl_ms"] = int(raw.get("ttl_ms", 0))
            packed_markers.append(marker)
        out["debug_markers"] = packed_markers
    return out


def pack_target_q(q: SimQ, *, source: str, seq: int, ts: Optional[float] = None) -> Dict[str, Any]:
    ts = now_s() if ts is None else float(ts)
    return {
        "t": "target",
        "ts": ts,
        "seq": int(seq),
        "source": str(source),
        "q": {
            "linear_m": q.linear_m,
            "roll_rad": q.roll_rad,
            "theta1_rad": q.theta1_rad,
            "theta2_rad": q.theta2_rad,
        },
    }


def unpack_u(d: Dict[str, Any]) -> ControlU:
    return ControlU(
        u_linear=float(d.get("linear", 0.0)),
        u_roll=float(d.get("roll", 0.0)),
        u_s1=float(d.get("s1", 0.0)),
        u_s2=float(d.get("s2", 0.0)),
    )


def unpack_q(d: Dict[str, Any]) -> SimQ:
    return SimQ(
        linear_m=float(d.get("linear_m", 0.0)),
        roll_rad=float(d.get("roll_rad", 0.0)),
        theta1_rad=float(d.get("theta1_rad", 0.0)),
        theta2_rad=float(d.get("theta2_rad", 0.0)),
    )


def unpack_go2_vel(raw: Any) -> tuple[float, float, float]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        raise ValueError("go2_vel must be [vx, vy, wz]")
    return (float(raw[0]), float(raw[1]), float(raw[2]))


def unpack_go2_sport_pose(raw: Any) -> str:
    pose = str(raw).strip().lower()
    if not pose:
        raise ValueError("go2_sport_pose must be a non-empty string")
    return pose


def unpack_go2_obstacles_avoid_enable(raw: Any) -> bool:
    if isinstance(raw, bool):
        return bool(raw)
    if isinstance(raw, (int, float)):
        return bool(int(raw))
    text = str(raw).strip().lower()
    if text in ("1", "true", "yes", "on", "enable", "enabled"):
        return True
    if text in ("0", "false", "no", "off", "disable", "disabled"):
        return False
    raise ValueError("go2_obstacles_avoid_enable must be a boolean")


def unpack_vec3(raw: Any, *, name: str) -> tuple[float, float, float]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        raise ValueError(f"{name} must be [x, y, z]")
    return (float(raw[0]), float(raw[1]), float(raw[2]))
