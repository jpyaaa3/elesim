from __future__ import annotations

import json
from typing import Sequence

API_DAMP = 1001
API_BALANCE_STAND = 1002
API_STOP_MOVE = 1003
API_STAND_UP = 1004
API_MOVE = 1008


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


def clamp_velocity(vx: float, vy: float, wz: float) -> tuple[float, float, float]:
    return float(vx), float(vy), float(wz)
