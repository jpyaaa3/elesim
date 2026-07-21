"""Thread-safe normalized orbit/pan/zoom commands for the rendered camera."""

from __future__ import annotations

import math
import threading
from typing import Iterable

import numpy as np


_LOCK = threading.Lock()
_PENDING: list[tuple[str, tuple[float, ...]]] = []


def enqueue(command: str, values: Iterable[float] = ()) -> None:
    name = str(command).strip().lower()
    if name not in {"orbit", "pan", "zoom", "reset"}:
        raise ValueError(f"unsupported camera command: {name}")
    with _LOCK:
        _PENDING.append((name, tuple(float(value) for value in values)))


def consume_pose(
    pos: tuple[float, float, float],
    lookat: tuple[float, float, float],
    *,
    reset_pos: tuple[float, float, float],
    reset_lookat: tuple[float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    with _LOCK:
        commands = list(_PENDING)
        _PENDING.clear()
    eye = np.asarray(pos, dtype=float)
    target = np.asarray(lookat, dtype=float)
    for command, values in commands:
        if command == "reset":
            eye = np.asarray(reset_pos, dtype=float)
            target = np.asarray(reset_lookat, dtype=float)
            continue
        offset = eye - target
        radius = max(float(np.linalg.norm(offset)), 0.05)
        if command == "orbit" and len(values) >= 2:
            yaw = math.atan2(float(offset[1]), float(offset[0])) - values[0] * math.pi
            pitch = math.asin(float(np.clip(offset[2] / radius, -0.98, 0.98))) + values[1] * math.pi * 0.5
            pitch = float(np.clip(pitch, -1.45, 1.45))
            offset = radius * np.array(
                [math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch)]
            )
            eye = target + offset
        elif command == "zoom" and values:
            radius = float(np.clip(radius * math.exp(values[0] * 1.5), 0.08, 20.0))
            eye = target + offset / max(float(np.linalg.norm(offset)), 1e-9) * radius
        elif command == "pan" and len(values) >= 2:
            forward = target - eye
            forward /= max(float(np.linalg.norm(forward)), 1e-9)
            right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
            right /= max(float(np.linalg.norm(right)), 1e-9)
            up = np.cross(right, forward)
            shift = (-values[0] * right + values[1] * up) * radius
            eye += shift
            target += shift
    return tuple(float(value) for value in eye), tuple(float(value) for value in target)

