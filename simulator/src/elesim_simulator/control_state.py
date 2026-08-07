"""Thread-safe command state consumed by the Genesis runtime."""

from __future__ import annotations

import math
import threading
import time
from numbers import Real
from typing import Any, Mapping, Optional

import numpy as np

from elesim_protocol import MotionCommandRequest, SimMappingConfig, SimQ, default_start_sim_q


def _vector(raw: object, length: int, *, name: str) -> tuple[float, ...]:
    if not isinstance(raw, (list, tuple)) or len(raw) != length:
        raise ValueError(f"{name} must contain {length} values")
    values: list[float] = []
    for raw_value in raw:
        if isinstance(raw_value, bool) or not isinstance(raw_value, Real):
            raise ValueError(f"{name} must contain finite numbers")
        value = float(raw_value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must contain finite numbers")
        values.append(value)
    return tuple(values)


class SimulationStateSource:
    """Latest-value command mailbox shared by protocol and simulation threads."""

    def __init__(self, mapping: SimMappingConfig, *, clock=time.monotonic) -> None:
        self.mapping = mapping
        self.clock = clock
        self._lock = threading.RLock()
        self._q = default_start_sim_q(mapping)
        self._ik_target: Optional[np.ndarray] = None
        self._ik_direction: Optional[np.ndarray] = None
        self._sag_model: dict[str, Any] = {}
        self._claw_closed = False
        self._go2_velocity = (0.0, 0.0, 0.0)
        self._go2_base_pos: Optional[tuple[float, float, float]] = None
        self._go2_base_rpy: Optional[tuple[float, float, float]] = None
        self._go2_leg_q: Optional[tuple[float, ...]] = None
        self._go2_sport_pose = ""
        self._go2_sport_pose_seq = 0
        self._obstacles_avoid_enabled = False
        self._obstacles_avoid_seq = 0
        self._sim_target: Optional[np.ndarray] = None
        self._planned_move_target: Optional[np.ndarray] = None
        self._planned_move_target_hold_dir = False
        self._planned_move_preview_waypoints: list[tuple[float, float, float, float]] = []
        self._planned_move_preview_seq = 0
        self._sim_reset_seq = 0
        self._debug_markers: list[dict[str, Any]] = []
        self._last_update_at: Optional[float] = None
        self._torque_enabled = False

    def poll(self) -> None:
        return None

    def close(self) -> None:
        return None

    def apply_command(self, payload: Mapping[str, Any]) -> str:
        command = str(payload.get("command", "")).strip()
        if command == "target":
            self.apply_target(payload)
            return "target"
        if command == "sim_reset":
            with self._lock:
                self._sim_reset_seq += 1
                self._last_update_at = self.clock()
            return "sim_reset"
        if command == "torque_on":
            with self._lock:
                self._torque_enabled = True
            return "torque_on"
        if command in {"torque_off", "estop"}:
            with self._lock:
                self._torque_enabled = False
                self._go2_velocity = (0.0, 0.0, 0.0)
            return command
        raise ValueError("unsupported_command")

    def apply_target(self, payload: Mapping[str, Any]) -> None:
        parsed = MotionCommandRequest.from_payload(payload)
        body = parsed.raw
        with self._lock:
            if parsed.q is not None:
                self._q = SimQ(*parsed.q)
            if parsed.go2_velocity is not None:
                self._go2_velocity = tuple(parsed.go2_velocity)
            if "target" in body:
                self._ik_target = np.asarray(_vector(body["target"], 3, name="target"), dtype=float)
            if "target_dir" in body:
                self._ik_direction = np.asarray(
                    _vector(body["target_dir"], 3, name="target_dir"),
                    dtype=float,
                )
            if "sag_model" in body:
                if not isinstance(body["sag_model"], Mapping):
                    raise ValueError("sag_model must be an object")
                self._sag_model = dict(body["sag_model"])
            if "claw_closed" in body:
                if not isinstance(body["claw_closed"], bool):
                    raise ValueError("claw_closed must be boolean")
                self._claw_closed = body["claw_closed"]
            if "sim_target" in body:
                self._sim_target = np.asarray(
                    _vector(body["sim_target"], 3, name="sim_target"),
                    dtype=float,
                )
            if "planned_move_target" in body:
                self._planned_move_target = np.asarray(
                    _vector(body["planned_move_target"], 3, name="planned_move_target"),
                    dtype=float,
                )
            if "planned_move_target_hold_dir" in body:
                if not isinstance(body["planned_move_target_hold_dir"], bool):
                    raise ValueError("planned_move_target_hold_dir must be boolean")
                self._planned_move_target_hold_dir = body["planned_move_target_hold_dir"]
            if "planned_move_preview_waypoints" in body:
                raw_waypoints = body["planned_move_preview_waypoints"]
                if not isinstance(raw_waypoints, list):
                    raise ValueError("planned_move_preview_waypoints must be a list")
                self._planned_move_preview_waypoints = [
                    _vector(wp, 4, name="planned_move_preview_waypoint") for wp in raw_waypoints
                ]
                self._planned_move_preview_seq += 1
            if "debug_markers" in body:
                if not isinstance(body["debug_markers"], list):
                    raise ValueError("debug_markers must be a list")
                self._debug_markers = [
                    dict(marker) for marker in body["debug_markers"] if isinstance(marker, Mapping)
                ]
            if "go2_sport_pose" in body:
                pose = str(body["go2_sport_pose"]).strip().lower()
                if not pose:
                    raise ValueError("go2_sport_pose must not be empty")
                self._go2_sport_pose = pose
                self._go2_sport_pose_seq += 1
            if "go2_obstacles_avoid_enable" in body:
                enabled = body["go2_obstacles_avoid_enable"]
                if not isinstance(enabled, bool):
                    raise ValueError("go2_obstacles_avoid_enable must be boolean")
                self._obstacles_avoid_enabled = enabled
                self._obstacles_avoid_seq += 1
            self._last_update_at = self.clock()

    def revoke_control(self) -> None:
        with self._lock:
            self._go2_velocity = (0.0, 0.0, 0.0)

    def estimate_q(self) -> SimQ:
        with self._lock:
            return self._q

    def seed_estimate_q(self, q: SimQ) -> None:
        with self._lock:
            self._q = q

    def ik_target_xyz(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._ik_target is None else self._ik_target.copy()

    def ik_target_dir(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._ik_direction is None else self._ik_direction.copy()

    def sag_model(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._sag_model)

    def claw_closed(self) -> bool:
        with self._lock:
            return bool(self._claw_closed)

    def go2_vel(self) -> tuple[float, float, float]:
        with self._lock:
            return tuple(self._go2_velocity)

    def update_go2_mirror(
        self,
        *,
        base_pos: Optional[tuple[float, float, float]] = None,
        base_rpy: Optional[tuple[float, float, float]] = None,
        leg_q: Optional[tuple[float, ...]] = None,
    ) -> None:
        with self._lock:
            if base_pos is not None:
                self._go2_base_pos = tuple(_vector(base_pos, 3, name="go2 base pos"))
            if base_rpy is not None:
                self._go2_base_rpy = tuple(_vector(base_rpy, 3, name="go2 base rpy"))
            if leg_q is not None:
                self._go2_leg_q = tuple(_vector(leg_q, 12, name="go2 leg q"))

    def go2_base_pos(self) -> Optional[tuple[float, float, float]]:
        with self._lock:
            return self._go2_base_pos

    def go2_base_rpy(self) -> Optional[tuple[float, float, float]]:
        with self._lock:
            return self._go2_base_rpy

    def go2_leg_q(self) -> Optional[tuple[float, ...]]:
        with self._lock:
            return self._go2_leg_q

    def go2_sport_pose(self) -> str:
        with self._lock:
            return self._go2_sport_pose

    def go2_sport_pose_seq(self) -> int:
        with self._lock:
            return self._go2_sport_pose_seq

    def go2_obstacles_avoid_enabled(self) -> bool:
        with self._lock:
            return self._obstacles_avoid_enabled

    def go2_obstacles_avoid_seq(self) -> int:
        with self._lock:
            return self._obstacles_avoid_seq

    def sim_reset_seq(self) -> int:
        with self._lock:
            return self._sim_reset_seq

    def sim_target_xyz(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._sim_target is None else self._sim_target.copy()

    def planned_move_target_xyz(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._planned_move_target is None else self._planned_move_target.copy()

    def planned_move_target_hold_dir(self) -> bool:
        with self._lock:
            return bool(self._planned_move_target_hold_dir)

    def planned_move_preview_waypoints(self) -> list[tuple[float, float, float, float]]:
        with self._lock:
            return list(self._planned_move_preview_waypoints)

    def planned_move_preview_seq(self) -> int:
        with self._lock:
            return self._planned_move_preview_seq

    def debug_markers(self) -> list[dict[str, Any]]:
        with self._lock:
            return [dict(marker) for marker in self._debug_markers]

    def host_state_age_s(self) -> Optional[float]:
        with self._lock:
            if self._last_update_at is None:
                return None
            return max(0.0, float(self.clock() - self._last_update_at))


__all__ = ["SimulationStateSource"]
