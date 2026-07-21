"""Thread-safe view of telemetry received from the selected endpoint."""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable, Mapping, Optional

from elesim_protocol import ControlU, SimMappingConfig, SimQ, sim_q_to_control_u, unpack_q, unpack_u

from elesim_controller.pick.state import HostState


def _tuple(raw: object, length: int) -> Optional[tuple[float, ...]]:
    if not isinstance(raw, (list, tuple)) or len(raw) != length:
        return None
    try:
        values = tuple(float(value) for value in raw)
    except (TypeError, ValueError):
        return None
    return values if all(math.isfinite(value) for value in values) else None


def _number(raw: object, default: float = 0.0) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return float(default)
    return value if math.isfinite(value) else float(default)


def _integer(raw: object, default: int = 0) -> int:
    try:
        return int(raw)
    except (TypeError, ValueError):
        return int(default)


def _number_dict(raw: object, caster: Callable[[object], Any]) -> dict[str, Any]:
    if not isinstance(raw, Mapping):
        return {}
    result: dict[str, Any] = {}
    for key, value in raw.items():
        try:
            result[str(key)] = caster(value)
        except (TypeError, ValueError):
            continue
    return result


def _sim_q(raw: object) -> SimQ:
    values = _tuple(raw, 4)
    if values is not None:
        return SimQ(values[0], values[1], values[2], values[3])
    if isinstance(raw, Mapping):
        return unpack_q(dict(raw))
    raise ValueError("q must be a canonical four-vector")


class RemoteState:
    """Accumulates typed telemetry without exposing wire dictionaries to Pick."""

    def __init__(
        self,
        mapping: SimMappingConfig,
        *,
        stale_after_s: float = 2.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.mapping = mapping
        self.stale_after_s = max(0.1, float(stale_after_s))
        self.clock = clock
        self._lock = threading.RLock()
        self._router_connected = False
        self._target_id = ""
        self._last_rx_at: Optional[float] = None
        self._data: dict[str, Any] = {}
        self._q: Optional[SimQ] = None
        self._u: Optional[ControlU] = None
        self._sim_q: Optional[SimQ] = None
        self._sim_u: Optional[ControlU] = None
        self._reply_ok = True
        self._reply_reason = ""

    def router_connected(self, connected: bool) -> None:
        with self._lock:
            self._router_connected = bool(connected)

    def target_changed(self, target_id: str) -> None:
        target = str(target_id)
        with self._lock:
            if target == self._target_id:
                return
            self._target_id = target
            self._last_rx_at = None
            self._data.clear()
            self._q = None
            self._u = None
            self._sim_q = None
            self._sim_u = None

    def accept_ack(self, payload: Mapping[str, Any]) -> None:
        with self._lock:
            self._last_rx_at = self.clock()
            self._reply_ok = bool(payload.get("ok", True))
            self._reply_reason = str(payload.get("reason", ""))
            for key in (
                "device",
                "ports",
                "torque_enabled",
                "claw_current",
                "motor_currents_ma",
                "motor_positions_raw",
                "motor_positions_deg",
                "safety_fault",
            ):
                if key in payload:
                    self._data[key] = payload[key]

    def accept_error(self, reason: str) -> None:
        with self._lock:
            self._reply_ok = False
            self._reply_reason = str(reason)

    def accept_telemetry(self, payload: Mapping[str, Any]) -> None:
        body = dict(payload)
        with self._lock:
            self._last_rx_at = self.clock()
            self._data.update(body)
            if "q" in body:
                try:
                    self._q = _sim_q(body["q"])
                    self._u = sim_q_to_control_u(self._q, self.mapping)
                except (TypeError, ValueError):
                    self._reply_ok = False
                    self._reply_reason = "telemetry q decode failed"
            if "u" in body:
                try:
                    self._u = unpack_u(body["u"])
                except (TypeError, ValueError):
                    self._reply_ok = False
                    self._reply_reason = "telemetry u decode failed"
            if "sim_q" in body:
                try:
                    self._sim_q = _sim_q(body["sim_q"])
                    self._sim_u = sim_q_to_control_u(self._sim_q, self.mapping)
                except (TypeError, ValueError):
                    self._reply_ok = False
                    self._reply_reason = "telemetry sim_q decode failed"
            if not self._reply_reason:
                self._reply_ok = True

    @property
    def object_world_xyz(self) -> Optional[tuple[float, float, float]]:
        with self._lock:
            value = _tuple(self._data.get("object_world"), 3)
            return None if value is None else (value[0], value[1], value[2])

    def rx_age_s(self) -> float:
        with self._lock:
            if self._last_rx_at is None:
                return float("inf")
            return max(0.0, float(self.clock() - self._last_rx_at))

    def snapshot(self, *, tx_seq: int) -> HostState:
        with self._lock:
            data = dict(self._data)
            has_rx = self._last_rx_at is not None
            age = -1.0 if not has_rx else max(0.0, float(self.clock() - self._last_rx_at))
            connected = bool(
                self._router_connected
                and self._target_id
                and has_rx
                and age <= self.stale_after_s
            )

            perceived_camera = _tuple(data.get("perceived_object_camera"), 3)
            perceived_uv = _tuple(data.get("perceived_center_uv"), 2)
            actual_tip = _tuple(data.get("actual_tip"), 3)
            actual_tip_dir = _tuple(data.get("actual_tip_dir"), 3)
            base_rpy = _tuple(data.get("go2_base_rpy"), 3)
            base_pos = _tuple(data.get("go2_base_pos"), 3)
            sim_base_pos = _tuple(data.get("go2_sim_base_pos"), 3)
            base_velocity = _tuple(data.get("go2_base_lin_vel_body"), 3)
            angular_velocity = _tuple(data.get("go2_base_ang_vel"), 3)
            go2_velocity = _tuple(data.get("go2_vel"), 3) or (0.0, 0.0, 0.0)
            leg_q = _tuple(data.get("go2_leg_q"), 12)
            leg_dq = _tuple(data.get("go2_leg_dq"), 12)
            leg_torque = _tuple(data.get("go2_leg_torque_nm"), 12)

            return HostState(
                connected=connected,
                tx_seq=int(tx_seq),
                rx_age_s=age,
                device=str(data.get("device", "")),
                ports=tuple(str(value) for value in data.get("ports", ()) if str(value)),
                torque_enabled=bool(data.get("torque_enabled", False)),
                claw_current=_integer(data.get("claw_current")),
                motor_currents_ma=_number_dict(data.get("motor_currents_ma"), int),
                motor_positions_raw=_number_dict(data.get("motor_positions_raw"), int),
                motor_positions_deg=_number_dict(data.get("motor_positions_deg"), float),
                safety_fault=str(data.get("safety_fault", "")),
                actual_tip_xyz=None if actual_tip is None else tuple(actual_tip),
                actual_tip_dir=None if actual_tip_dir is None else tuple(actual_tip_dir),
                perceived_object_label=str(data.get("perceived_object_label", "")),
                perceived_object_confidence=_number(data.get("perceived_object_confidence")),
                perceived_object_camera_xyz=None if perceived_camera is None else tuple(perceived_camera),
                perceived_center_uv=None if perceived_uv is None else tuple(perceived_uv),
                perceived_scale=(
                    None
                    if data.get("perceived_scale") is None
                    else _number(data.get("perceived_scale"))
                ),
                perceived_timestamp_s=_number(data.get("perceived_timestamp_s")),
                perception_running=bool(data.get("perception_running", False)),
                perception_failed=bool(data.get("perception_failed", False)),
                perception_status=str(data.get("perception_status", "")),
                perception_source=str(data.get("perception_source", "")),
                perception_preview_endpoint=str(data.get("perception_preview_endpoint", "")),
                perception_recording=bool(data.get("perception_recording", False)),
                perception_record_with_overlay=bool(data.get("perception_record_with_overlay", False)),
                perception_last_record_path=str(data.get("perception_last_record_path", "")),
                perception_last_capture_path=str(data.get("perception_last_capture_path", "")),
                perception_hz=max(0.0, _number(data.get("perception_hz"))),
                gaze_running=bool(data.get("gaze_running", False)),
                gaze_mode=str(data.get("gaze_mode", "idle") or "idle"),
                gaze_status_msg=str(data.get("gaze_status_msg", "")),
                gaze_u_err=_number(data.get("gaze_u_err")),
                gaze_v_err=_number(data.get("gaze_v_err")),
                gaze_du_roll=_number(data.get("gaze_du_roll")),
                gaze_du_s1=_number(data.get("gaze_du_s1")),
                gaze_du_s2=_number(data.get("gaze_du_s2")),
                gaze_tick_count=_integer(data.get("gaze_tick_count")),
                gaze_update_count=_integer(data.get("gaze_update_count")),
                gaze_obs_age_s=_number(data.get("gaze_obs_age_s"), -1.0),
                gaze_config=dict(data.get("gaze_config", {})) if isinstance(data.get("gaze_config"), Mapping) else {},
                pick_running=bool(data.get("pick_running", False)),
                pick_failed=bool(data.get("pick_failed", False)),
                pick_phase=str(data.get("pick_phase", "idle") or "idle"),
                pick_status_msg=str(data.get("pick_status_msg", "")),
                go2_vel=(go2_velocity[0], go2_velocity[1], go2_velocity[2]),
                go2_base_rpy=None if base_rpy is None else tuple(base_rpy),
                go2_base_pos=None if base_pos is None else tuple(base_pos),
                go2_sim_base_pos=None if sim_base_pos is None else tuple(sim_base_pos),
                go2_base_lin_vel_body=None if base_velocity is None else tuple(base_velocity),
                go2_base_ang_vel=None if angular_velocity is None else tuple(angular_velocity),
                go2_base_timestamp_s=_number(data.get("go2_base_timestamp_s")),
                go2_gait_phase=(None if data.get("go2_gait_phase") is None else _number(data.get("go2_gait_phase"))),
                go2_gait_period_s=(None if data.get("go2_gait_period_s") is None else _number(data.get("go2_gait_period_s"))),
                host_state_age_s=age,
                go2_leg_q=leg_q,
                go2_leg_dq=leg_dq,
                go2_leg_torque_nm=leg_torque,
                go2_sport_pose=str(data.get("go2_sport_pose", "")),
                go2_sport_pose_seq=_integer(data.get("go2_sport_pose_seq")),
                go2_obstacles_avoid_enabled=bool(data.get("go2_obstacles_avoid_enabled", False)),
                go2_obstacles_avoid_seq=_integer(data.get("go2_obstacles_avoid_seq")),
                sim_time_s=_number(data.get("sim_time_s")),
                sim_wall_elapsed_s=_number(data.get("sim_wall_elapsed_s")),
                sim_realtime_factor=_number(data.get("sim_realtime_factor")),
                sim_step_count=_integer(data.get("sim_step_count")),
                reply_ok=bool(self._reply_ok),
                reply_reason=str(self._reply_reason),
                q=self._q,
                u=self._u,
                sim_q=self._sim_q,
                sim_u=self._sim_u,
            )


__all__ = ["RemoteState"]
