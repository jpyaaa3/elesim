"""Command port used by controller workflows.

``ControllerConnection`` owns DDS transport and attaches its thread-safe
submission method here.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Mapping, Optional

from elesim_protocol import ControlU, SimMappingConfig, SimQ, control_u_to_sim_q, sim_q_to_control_u

from elesim_controller.remote_state import RemoteState
from .state import HostState


CommandSender = Callable[..., None]


class ControlClient:
    """Stable command API consumed by Pick, Gaze and operator services."""

    def __init__(
        self,
        *,
        cfg: Optional[SimMappingConfig] = None,
        remote_state: Optional[RemoteState] = None,
    ) -> None:
        self.cfg = cfg or SimMappingConfig()
        self._state = remote_state or RemoteState(self.cfg)
        self._sender: Optional[CommandSender] = None
        self._lock = threading.RLock()
        self.tx_seq = 0

    def attach_sender(self, sender: CommandSender) -> None:
        with self._lock:
            self._sender = sender

    def close(self) -> None:
        self._state.peer_connected(False)

    @property
    def is_connected(self) -> bool:
        return bool(self.get_state().connected)

    @property
    def last_object_world_xyz(self) -> Optional[tuple[float, float, float]]:
        return self._state.object_world_xyz

    def peer_connected(self, connected: bool) -> None:
        self._state.peer_connected(connected)

    def target_changed(self, target_id: str) -> None:
        self._state.target_changed(target_id)

    def accept_telemetry(self, payload: Mapping[str, Any]) -> None:
        self._state.accept_telemetry(payload)

    def accept_ack(self, payload: Mapping[str, Any]) -> None:
        self._state.accept_ack(payload)

    def accept_error(self, reason: str) -> None:
        self._state.accept_error(reason)

    def rx_age_s(self) -> float:
        return self._state.rx_age_s()

    def get_state(self) -> HostState:
        with self._lock:
            sequence = self.tx_seq
        return self._state.snapshot(tx_seq=sequence)

    def poll(self) -> None:
        """Compatibility with workflow polling; transport updates asynchronously."""

    def refresh_state(self) -> HostState:
        return self.get_state()

    def _send(self, message: Mapping[str, Any], *, force: bool = False) -> None:
        with self._lock:
            self.tx_seq += 1
            sender = self._sender
        if sender is None:
            self._state.accept_error("controller transport is not started")
            return
        sender(dict(message), force=bool(force))

    def estop(self) -> None:
        self._send({"t": "estop"}, force=True)

    def send_sim_reset(self) -> None:
        self._send({"t": "sim_reset"}, force=True)

    def torque_on(self, *, resume: bool = False) -> None:
        self._send({"t": "torque_on", "resume": bool(resume)}, force=True)

    def torque_off(self) -> None:
        self._send({"t": "torque_off"}, force=True)

    def request_ports(self) -> None:
        self._send({"t": "ports"}, force=True)

    def set_device(self, device: str) -> None:
        self._send({"t": "set_device", "device": str(device)}, force=True)

    def disconnect_device(self) -> None:
        self._send({"t": "disconnect_device"}, force=True)

    def _workflow_is_local(self, operation: str) -> None:
        raise RuntimeError(f"{operation} belongs to the controller deployment, not the target endpoint")

    def send_perception_start(self, *, config: Optional[Any] = None) -> None:
        self._workflow_is_local("perception_start")

    def send_perception_stop(self) -> None:
        self._workflow_is_local("perception_stop")

    def send_perception_refresh(self) -> None:
        self._workflow_is_local("perception_refresh")

    def send_perception_capture(self, *, include_overlay: bool = True) -> None:
        self._workflow_is_local("perception_capture")

    def send_perception_record_start(self, *, include_overlay: bool = False, fps: float = 0.0) -> None:
        self._workflow_is_local("perception_record_start")

    def send_perception_record_stop(self) -> None:
        self._workflow_is_local("perception_record_stop")

    def send_gaze_start_standing(self, *, run_id: str = "") -> None:
        self._workflow_is_local("gaze_start_standing")

    def send_gaze_start_walking(self, *, run_id: str = "", gaze_mode: str = "") -> None:
        self._workflow_is_local("gaze_start_walking")

    def send_gaze_stop(self) -> None:
        self._workflow_is_local("gaze_stop")

    def stop_lji_velocity_control(self, *, reason: str = "client_stop") -> None:
        self._workflow_is_local("lji_velocity_stop")

    def send_gaze_config_update(self, config: dict[str, Any]) -> None:
        self._workflow_is_local("gaze_config_update")

    def send_mobile_pick_start(self) -> None:
        self._workflow_is_local("mobile_pick_start")

    def send_lji_grasp_start(self) -> None:
        self._workflow_is_local("lji_grasp_start")

    def send_pick_stop(self) -> None:
        self._workflow_is_local("pick_stop")

    def send_mobile_pick_stop(self) -> None:
        self._workflow_is_local("mobile_pick_stop")

    def send_perception_observation(
        self,
        *,
        object_camera_xyz: tuple[float, float, float],
        label: str = "",
        confidence: float = 0.0,
        image_center_uv: tuple[float, float],
        image_scale: float,
        depth_valid: bool = True,
        object_world: Optional[tuple[float, float, float]] = None,
        camera_world_origin: Optional[tuple[float, float, float]] = None,
        camera_world_look: Optional[tuple[float, float, float]] = None,
        camera_world_right: Optional[tuple[float, float, float]] = None,
        wait_ack_s: float = 0.0,
    ) -> Optional[tuple[float, float, float]]:
        payload: dict[str, Any] = {
            "t": "target",
            "source": "perception",
            "object_camera": [float(value) for value in object_camera_xyz],
            "object_label": str(label),
            "object_confidence": float(confidence),
            "image_center_uv": [float(value) for value in image_center_uv],
            "image_scale": float(image_scale),
            "depth_valid": bool(depth_valid),
        }
        for key, value in (
            ("object_world", object_world),
            ("camera_world_origin", camera_world_origin),
            ("camera_world_look", camera_world_look),
            ("camera_world_right", camera_world_right),
        ):
            if value is not None:
                payload[key] = [float(item) for item in value]
        self._send(payload)
        return object_world if object_world is not None else self.last_object_world_xyz

    def send_claw_command(self, *, claw_closed: bool, source: str = "target") -> None:
        self._send(
            {"t": "target", "source": str(source), "claw_closed": bool(claw_closed)},
            force=True,
        )

    def send_go2_velocity(self, *, vx: float, vy: float, wz: float, source: str = "target") -> None:
        self._send(
            {
                "t": "target",
                "source": str(source),
                "go2_vel": [float(vx), float(vy), float(wz)],
            }
        )

    def send_go2_sport_pose(self, *, pose: str, source: str = "target") -> None:
        self._send(
            {"t": "target", "source": str(source), "go2_sport_pose": str(pose).strip().lower()},
            force=True,
        )

    def send_go2_obstacles_avoid(self, *, enabled: bool, source: str = "target") -> None:
        self._send(
            {"t": "target", "source": str(source), "go2_obstacles_avoid_enable": bool(enabled)},
            force=True,
        )

    def send_sim_target_xyz(self, *, xyz: tuple[float, float, float], source: str = "target") -> None:
        self._send(
            {"t": "target", "source": str(source), "sim_target": [float(value) for value in xyz]},
            force=True,
        )

    def send_debug_markers(self, markers: list[dict[str, Any]], *, source: str = "target") -> None:
        self._send(
            {"t": "target", "source": str(source), "debug_markers": [dict(value) for value in markers]},
            force=True,
        )

    def send_target_meta(
        self,
        *,
        target_xyz: tuple[float, float, float],
        target_dir: tuple[float, float, float],
        source: str = "target",
    ) -> None:
        self._send(
            {
                "t": "target",
                "source": str(source),
                "target": [float(value) for value in target_xyz],
                "target_dir": [float(value) for value in target_dir],
            },
            force=True,
        )

    def send_ready_pose_meta(
        self,
        *,
        target_dir: tuple[float, float, float],
        standoff_m: float,
        source: str = "target",
    ) -> None:
        self._send(
            {
                "t": "target",
                "source": str(source),
                "ready_pose_dir": [float(value) for value in target_dir],
                "ready_pose_standoff_m": float(standoff_m),
            },
            force=True,
        )

    def send_sag_model_meta(self, sag_model: dict[str, Any], *, source: str = "target") -> None:
        self._send(
            {"t": "target", "source": str(source), "sag_model": dict(sag_model)},
            force=True,
        )

    def maybe_send_target_q(self, q: SimQ, *, source: str = "sim", force: bool = False) -> None:
        self._send_target_q(q, source=source, force=force)

    def _send_target_q(
        self,
        q: SimQ,
        *,
        source: str,
        target_xyz: Optional[tuple[float, float, float]] = None,
        target_dir: Optional[tuple[float, float, float]] = None,
        sag_model: Optional[dict[str, Any]] = None,
        claw_closed: Optional[bool] = None,
        force: bool = False,
    ) -> None:
        payload: dict[str, Any] = {
            "t": "target",
            "source": str(source),
            "q": [float(q.linear_m), float(q.roll_rad), float(q.theta1_rad), float(q.theta2_rad)],
        }
        if target_xyz is not None:
            payload["target"] = [float(value) for value in target_xyz]
        if target_dir is not None:
            payload["target_dir"] = [float(value) for value in target_dir]
        if sag_model is not None:
            payload["sag_model"] = dict(sag_model)
        if claw_closed is not None:
            payload["claw_closed"] = bool(claw_closed)
        self._send(payload, force=force)

    def send_target_q(self, q: SimQ, *, source: str = "ui", force: bool = False) -> None:
        self._send_target_q(q, source=source, force=force)

    def send_target_values(
        self,
        *,
        linear_m: float,
        roll_rad: float,
        theta1_rad: float,
        theta2_rad: float,
        source: str = "ui",
        target_xyz: Optional[tuple[float, float, float]] = None,
        target_dir: Optional[tuple[float, float, float]] = None,
        sag_model: Optional[dict[str, Any]] = None,
        claw_closed: Optional[bool] = None,
        force: bool = False,
    ) -> None:
        self._send_target_q(
            SimQ(float(linear_m), float(roll_rad), float(theta1_rad), float(theta2_rad)),
            source=source,
            target_xyz=target_xyz,
            target_dir=target_dir,
            sag_model=sag_model,
            claw_closed=claw_closed,
            force=force,
        )

    def q_to_control_u(
        self,
        *,
        linear_m: float,
        roll_rad: float,
        theta1_rad: float,
        theta2_rad: float,
    ) -> ControlU:
        return sim_q_to_control_u(
            SimQ(float(linear_m), float(roll_rad), float(theta1_rad), float(theta2_rad)),
            self.cfg,
        )

    def control_u_to_q(
        self,
        *,
        u_linear: float,
        u_roll: float,
        u_s1: float,
        u_s2: float,
    ) -> SimQ:
        return control_u_to_sim_q(
            ControlU(float(u_linear), float(u_roll), float(u_s1), float(u_s2)),
            self.cfg,
        )


__all__ = ["ControlClient"]
