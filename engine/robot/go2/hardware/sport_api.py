from __future__ import annotations

import json
from typing import Sequence

API_DAMP = 1001
API_BALANCE_STAND = 1002
API_STOP_MOVE = 1003
API_STAND_UP = 1004
API_STAND_DOWN = 1005
API_RECOVERY_STAND = 1006
API_MOVE = 1008
API_STATIC_WALK = 1061
API_TROT_RUN = 1062
API_ECONOMIC_GAIT = 1063

GO2_SPORT_POSES: dict[str, int] = {
    "balance_stand": API_BALANCE_STAND,
    "stand_up": API_STAND_UP,
    "stand_down": API_STAND_DOWN,
    "recovery_stand": API_RECOVERY_STAND,
    "static_walk": API_STATIC_WALK,
    "trot_run": API_TROT_RUN,
    "economic_gait": API_ECONOMIC_GAIT,
}


def build_move_parameter(vx: float, vy: float, wz: float) -> str:
    return json.dumps({"x": float(vx), "y": float(vy), "z": float(wz)}, separators=(",", ":"))


def velocity_below_deadband(
    vx: float,
    vy: float,
    wz: float,
    deadband: float,
) -> bool:
    d = max(float(deadband), 0.0)
    return abs(float(vx)) <= d and abs(float(vy)) <= d and abs(float(wz)) <= d


def normalize_stand_on_start(value: str) -> str:
    raw = str(value).strip().lower()
    if raw in ("none", "balance", "stand_up"):
        return raw
    return "balance"


def normalize_shutdown_mode(value: str) -> str:
    raw = str(value).strip().lower()
    if raw in ("none", "damp", "stop"):
        return raw
    return "damp"


def stand_api_id(stand_on_start: str) -> int | None:
    mode = normalize_stand_on_start(stand_on_start)
    if mode == "balance":
        return API_BALANCE_STAND
    if mode == "stand_up":
        return API_STAND_UP
    return None


def shutdown_api_id(shutdown_mode: str) -> int | None:
    mode = normalize_shutdown_mode(shutdown_mode)
    if mode == "damp":
        return API_DAMP
    if mode == "stop":
        return API_STOP_MOVE
    return None


def normalize_gait_on_start(value: str) -> str:
    raw = str(value).strip().lower().replace("-", "_")
    aliases = {
        "static": "static_walk",
        "staticwalk": "static_walk",
        "trot": "trot_run",
        "trotrun": "trot_run",
        "economic": "economic_gait",
    }
    return aliases.get(raw, raw)


def gait_on_start_api_id(gait_on_start: str) -> int | None:
    mode = normalize_gait_on_start(gait_on_start)
    if mode in ("none", ""):
        return None
    return sport_pose_api_id(mode)


def clamp_velocity(vx: float, vy: float, wz: float) -> tuple[float, float, float]:
    return float(vx), float(vy), float(wz)


def normalize_go2_sport_pose(value: str) -> str:
    raw = str(value).strip().lower().replace("-", "_")
    aliases = {
        "balance": "balance_stand",
        "stand": "stand_up",
        "sit": "stand_down",
        "lie_down": "stand_down",
        "prone": "stand_down",
        "stand_up_from_fall": "recovery_stand",
        "get_up": "recovery_stand",
    }
    return aliases.get(raw, raw)


def sport_pose_api_id(pose: str) -> int | None:
    key = normalize_go2_sport_pose(pose)
    return GO2_SPORT_POSES.get(key)


def fill_unitree_request(
    msg: object,
    *,
    api_id: int,
    parameter: str = "",
    identity_id: int = 0,
    noreply: bool = True,
) -> None:
    """Populate unitree_api/msg/Request fields required by GO2 Sport service."""
    header = getattr(msg, "header", None)
    if header is None:
        raise ValueError("unitree Request missing header")
    identity = getattr(header, "identity", None)
    if identity is None:
        raise ValueError("unitree Request missing header.identity")
    identity.id = int(identity_id)
    identity.api_id = int(api_id)
    lease = getattr(header, "lease", None)
    if lease is not None:
        lease.id = 0
    policy = getattr(header, "policy", None)
    if policy is not None:
        policy.priority = 0
        policy.noreply = bool(noreply)
    msg.parameter = str(parameter)
    if hasattr(msg, "binary"):
        msg.binary = []
