from __future__ import annotations

import os
import threading
import time
from typing import Any, Optional

import zmq

from engine.core.protocol import (
    ControlU,
    SimMappingConfig,
    SimQ,
    control_u_to_sim_q,
    sim_q_to_control_u,
    unpack_q,
    unpack_u,
)
from engine.observability.tracing import message_span
from .state import HostState


class ControlClient:
    """Controller-side host client."""

    def __init__(
        self,
        endpoint: str = "tcp://127.0.0.1:5555",
        *,
        send_hz: float = 30.0,
        cfg: Optional[SimMappingConfig] = None,
    ) -> None:
        if zmq is None:
            raise RuntimeError("pyzmq is required for ControlClient")
        self.endpoint = str(endpoint)
        self.cfg = cfg or SimMappingConfig()
        self.send_hz = float(send_hz)
        self._send_period = (1.0 / self.send_hz) if self.send_hz > 0 else 0.0

        self.ctx = zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.DEALER)
        self.sock.linger = 0
        self.sock.setsockopt(zmq.IDENTITY, f"gensim-{os.getpid()}-{int(time.time()*1000)}".encode("utf-8"))
        self.sock.connect(self.endpoint)

        self.poller = zmq.Poller()
        self.poller.register(self.sock, zmq.POLLIN)
        self._io_lock = threading.Lock()

        self.is_connected = True
        self.tx_seq = 0
        self._t_last_tx = 0.0
        self._hello_period_s = 0.5
        self._t_last_hello = 0.0

        self.last_ack_ts = 0.0
        self.last_state_ts = 0.0
        self._t_last_rx_wall = 0.0
        self.last_q: SimQ | None = None
        self.last_u: ControlU | None = None
        self.last_sim_q: SimQ | None = None
        self.last_sim_u: ControlU | None = None
        self.last_ports: list[str] = []
        self.last_device: str = ""
        self.torque_enabled: bool = False
        self.last_claw_current: int = 0
        self.last_motor_currents_ma: dict[str, int] = {}
        self.last_motor_positions_raw: dict[str, int] = {}
        self.last_motor_positions_deg: dict[str, float] = {}
        self.last_safety_fault: str = ""
        self.last_actual_tip_xyz: Optional[tuple[float, float, float]] = None
        self.last_actual_tip_dir: Optional[tuple[float, float, float]] = None
        self.last_perceived_object_label: str = ""
        self.last_perceived_object_confidence: float = 0.0
        self.last_perceived_object_camera_xyz: Optional[tuple[float, float, float]] = None
        self.last_perceived_center_uv: Optional[tuple[float, float]] = None
        self.last_perceived_scale: Optional[float] = None
        self.last_perceived_timestamp_s: float = 0.0
        self.last_perception_running: bool = False
        self.last_perception_failed: bool = False
        self.last_perception_status: str = ""
        self.last_perception_source: str = ""
        self.last_perception_preview_endpoint: str = ""
        self.last_perception_recording: bool = False
        self.last_perception_record_with_overlay: bool = False
        self.last_perception_last_record_path: str = ""
        self.last_perception_last_capture_path: str = ""
        self.last_perception_hz: float = 0.0
        self.last_gaze_running: bool = False
        self.last_gaze_mode: str = "idle"
        self.last_gaze_status_msg: str = ""
        self.last_gaze_u_err: float = 0.0
        self.last_gaze_v_err: float = 0.0
        self.last_gaze_du_roll: float = 0.0
        self.last_gaze_du_s1: float = 0.0
        self.last_gaze_du_s2: float = 0.0
        self.last_gaze_tick_count: int = 0
        self.last_gaze_update_count: int = 0
        self.last_gaze_obs_age_s: float = -1.0
        self.last_gaze_config: dict[str, Any] = {}
        self.last_pick_running: bool = False
        self.last_pick_failed: bool = False
        self.last_pick_phase: str = "idle"
        self.last_pick_status_msg: str = ""
        self.last_object_world_xyz: Optional[tuple[float, float, float]] = None
        self.last_go2_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.last_go2_base_rpy: Optional[tuple[float, float, float]] = None
        self.last_go2_base_pos: Optional[tuple[float, float, float]] = None
        self.last_go2_base_lin_vel_body: Optional[tuple[float, float, float]] = None
        self.last_go2_base_ang_vel: Optional[tuple[float, float, float]] = None
        self.last_go2_base_timestamp_s: float = 0.0
        self.last_go2_leg_q: Optional[tuple[float, ...]] = None
        self.last_go2_leg_dq: Optional[tuple[float, ...]] = None
        self.last_go2_leg_torque_nm: Optional[tuple[float, ...]] = None
        self.last_go2_sport_pose: str = ""
        self.last_go2_sport_pose_seq: int = 0
        self.last_go2_obstacles_avoid_enabled: bool = False
        self.last_go2_obstacles_avoid_seq: int = 0
        self.last_sim_target_xyz: Optional[tuple[float, float, float]] = None
        self.last_sim_time_s: float = 0.0
        self.last_sim_wall_elapsed_s: float = 0.0
        self.last_sim_realtime_factor: float = 0.0
        self.last_sim_step_count: int = 0
        self.last_reply_ok: bool = True
        self.last_reply_reason: str = ""

        self._send_hello(force=True)

    def close(self) -> None:
        try:
            self.poller.unregister(self.sock)
        except (KeyError, AttributeError):
            pass
        try:
            self.sock.close(0)
        except AttributeError:
            pass
        self.is_connected = False

    def rx_age_s(self) -> float:
        if self._t_last_rx_wall <= 0.0:
            return float("inf")
        return float(time.time() - self._t_last_rx_wall)

    def get_state(self) -> HostState:
        return HostState(
            connected=bool(self.is_connected),
            tx_seq=int(self.tx_seq),
            rx_age_s=float(self.rx_age_s()),
            device=str(self.last_device),
            ports=tuple(str(x) for x in self.last_ports),
            torque_enabled=bool(self.torque_enabled),
            claw_current=int(self.last_claw_current),
            motor_currents_ma=dict(self.last_motor_currents_ma),
            motor_positions_raw=dict(self.last_motor_positions_raw),
            motor_positions_deg=dict(self.last_motor_positions_deg),
            safety_fault=str(self.last_safety_fault),
            actual_tip_xyz=self.last_actual_tip_xyz,
            actual_tip_dir=self.last_actual_tip_dir,
            perceived_object_label=str(self.last_perceived_object_label),
            perceived_object_confidence=float(self.last_perceived_object_confidence),
            perceived_object_camera_xyz=self.last_perceived_object_camera_xyz,
            perceived_center_uv=self.last_perceived_center_uv,
            perceived_scale=self.last_perceived_scale,
            perceived_timestamp_s=float(self.last_perceived_timestamp_s),
            perception_running=bool(self.last_perception_running),
            perception_failed=bool(self.last_perception_failed),
            perception_status=str(self.last_perception_status),
            perception_source=str(self.last_perception_source),
            perception_preview_endpoint=str(self.last_perception_preview_endpoint),
            perception_recording=bool(self.last_perception_recording),
            perception_record_with_overlay=bool(self.last_perception_record_with_overlay),
            perception_last_record_path=str(self.last_perception_last_record_path),
            perception_last_capture_path=str(self.last_perception_last_capture_path),
            perception_hz=float(self.last_perception_hz),
            gaze_running=bool(self.last_gaze_running),
            gaze_mode=str(self.last_gaze_mode),
            gaze_status_msg=str(self.last_gaze_status_msg),
            gaze_u_err=float(self.last_gaze_u_err),
            gaze_v_err=float(self.last_gaze_v_err),
            gaze_du_roll=float(self.last_gaze_du_roll),
            gaze_du_s1=float(self.last_gaze_du_s1),
            gaze_du_s2=float(self.last_gaze_du_s2),
            gaze_tick_count=int(self.last_gaze_tick_count),
            gaze_update_count=int(self.last_gaze_update_count),
            gaze_obs_age_s=float(self.last_gaze_obs_age_s),
            gaze_config=dict(self.last_gaze_config),
            pick_running=bool(self.last_pick_running),
            pick_failed=bool(self.last_pick_failed),
            pick_phase=str(self.last_pick_phase),
            pick_status_msg=str(self.last_pick_status_msg),
            go2_vel=(
                float(self.last_go2_vel[0]),
                float(self.last_go2_vel[1]),
                float(self.last_go2_vel[2]),
            ),
            go2_base_rpy=self.last_go2_base_rpy,
            go2_base_pos=self.last_go2_base_pos,
            go2_base_lin_vel_body=self.last_go2_base_lin_vel_body,
            go2_base_ang_vel=self.last_go2_base_ang_vel,
            go2_base_timestamp_s=float(self.last_go2_base_timestamp_s),
            host_state_age_s=float(self.rx_age_s()),
            go2_leg_q=self.last_go2_leg_q,
            go2_leg_dq=self.last_go2_leg_dq,
            go2_leg_torque_nm=self.last_go2_leg_torque_nm,
            go2_sport_pose=str(self.last_go2_sport_pose),
            go2_sport_pose_seq=int(self.last_go2_sport_pose_seq),
            go2_obstacles_avoid_enabled=bool(self.last_go2_obstacles_avoid_enabled),
            go2_obstacles_avoid_seq=int(self.last_go2_obstacles_avoid_seq),
            sim_time_s=float(self.last_sim_time_s),
            sim_wall_elapsed_s=float(self.last_sim_wall_elapsed_s),
            sim_realtime_factor=float(self.last_sim_realtime_factor),
            sim_step_count=int(self.last_sim_step_count),
            reply_ok=bool(self.last_reply_ok),
            reply_reason=str(self.last_reply_reason),
            q=self.last_q,
            u=self.last_u,
            sim_q=self.last_sim_q,
            sim_u=self.last_sim_u,
        )

    def _update_sim_clock_fields(self, msg: dict[str, Any]) -> None:
        if "sim_time_s" in msg:
            try:
                self.last_sim_time_s = float(msg.get("sim_time_s", 0.0))
            except (TypeError, ValueError):
                pass
        if "sim_wall_elapsed_s" in msg:
            try:
                self.last_sim_wall_elapsed_s = float(msg.get("sim_wall_elapsed_s", 0.0))
            except (TypeError, ValueError):
                pass
        if "sim_realtime_factor" in msg:
            try:
                self.last_sim_realtime_factor = float(msg.get("sim_realtime_factor", 0.0))
            except (TypeError, ValueError):
                pass
        if "sim_step_count" in msg:
            try:
                self.last_sim_step_count = int(msg.get("sim_step_count", 0))
            except (TypeError, ValueError):
                pass
        sim_target_raw = msg.get("sim_target", None)
        if isinstance(sim_target_raw, (list, tuple)) and len(sim_target_raw) == 3:
            try:
                self.last_sim_target_xyz = (
                    float(sim_target_raw[0]),
                    float(sim_target_raw[1]),
                    float(sim_target_raw[2]),
                )
            except (TypeError, ValueError):
                pass

    def _update_perception_fields(self, msg: dict[str, Any]) -> None:
        if "perceived_object_label" in msg:
            self.last_perceived_object_label = str(msg.get("perceived_object_label", ""))
        if "perceived_object_confidence" in msg:
            try:
                self.last_perceived_object_confidence = float(msg.get("perceived_object_confidence", 0.0))
            except (TypeError, ValueError):
                self.last_perceived_object_confidence = 0.0
        object_camera_raw = msg.get("perceived_object_camera", None)
        if isinstance(object_camera_raw, (list, tuple)) and len(object_camera_raw) == 3:
            self.last_perceived_object_camera_xyz = (
                float(object_camera_raw[0]),
                float(object_camera_raw[1]),
                float(object_camera_raw[2]),
            )
        center_uv_raw = msg.get("perceived_center_uv", None)
        if isinstance(center_uv_raw, (list, tuple)) and len(center_uv_raw) == 2:
            self.last_perceived_center_uv = (float(center_uv_raw[0]), float(center_uv_raw[1]))
        if "perceived_scale" in msg:
            try:
                self.last_perceived_scale = float(msg.get("perceived_scale", 0.0))
            except (TypeError, ValueError):
                self.last_perceived_scale = None
        if "perceived_timestamp_s" in msg:
            try:
                self.last_perceived_timestamp_s = float(msg.get("perceived_timestamp_s", 0.0))
            except (TypeError, ValueError):
                self.last_perceived_timestamp_s = 0.0
        if "perception_running" in msg:
            self.last_perception_running = bool(msg.get("perception_running", False))
        if "perception_failed" in msg:
            self.last_perception_failed = bool(msg.get("perception_failed", False))
        if "perception_status" in msg:
            self.last_perception_status = str(msg.get("perception_status", ""))
        if "perception_source" in msg:
            self.last_perception_source = str(msg.get("perception_source", ""))
        if "perception_preview_endpoint" in msg:
            self.last_perception_preview_endpoint = str(msg.get("perception_preview_endpoint", ""))
        if "perception_recording" in msg:
            self.last_perception_recording = bool(msg.get("perception_recording", False))
        if "perception_record_with_overlay" in msg:
            self.last_perception_record_with_overlay = bool(msg.get("perception_record_with_overlay", False))
        if "perception_last_record_path" in msg:
            self.last_perception_last_record_path = str(msg.get("perception_last_record_path", ""))
        if "perception_last_capture_path" in msg:
            self.last_perception_last_capture_path = str(msg.get("perception_last_capture_path", ""))
        if "perception_hz" in msg:
            try:
                self.last_perception_hz = max(0.0, float(msg.get("perception_hz", 0.0)))
            except (TypeError, ValueError):
                self.last_perception_hz = 0.0
        object_world_raw = msg.get("object_world", None)
        if isinstance(object_world_raw, (list, tuple)) and len(object_world_raw) == 3:
            try:
                self.last_object_world_xyz = (
                    float(object_world_raw[0]),
                    float(object_world_raw[1]),
                    float(object_world_raw[2]),
                )
            except (TypeError, ValueError):
                pass

    def _update_gaze_fields(self, msg: dict[str, Any]) -> None:
        if "gaze_running" in msg:
            self.last_gaze_running = bool(msg.get("gaze_running", False))
        if "gaze_mode" in msg:
            self.last_gaze_mode = str(msg.get("gaze_mode", "idle"))
        if "gaze_status_msg" in msg:
            self.last_gaze_status_msg = str(msg.get("gaze_status_msg", ""))
        for key, attr, default in (
            ("gaze_u_err", "last_gaze_u_err", 0.0),
            ("gaze_v_err", "last_gaze_v_err", 0.0),
            ("gaze_du_roll", "last_gaze_du_roll", 0.0),
            ("gaze_du_s1", "last_gaze_du_s1", 0.0),
            ("gaze_du_s2", "last_gaze_du_s2", 0.0),
            ("gaze_obs_age_s", "last_gaze_obs_age_s", -1.0),
        ):
            if key in msg:
                try:
                    setattr(self, attr, float(msg.get(key, default)))
                except (TypeError, ValueError):
                    setattr(self, attr, float(default))
        if "gaze_update_count" in msg:
            try:
                self.last_gaze_update_count = int(msg.get("gaze_update_count", 0))
            except (TypeError, ValueError):
                self.last_gaze_update_count = 0
        if "gaze_tick_count" in msg:
            try:
                self.last_gaze_tick_count = int(msg.get("gaze_tick_count", 0))
            except (TypeError, ValueError):
                self.last_gaze_tick_count = 0
        if "gaze_config" in msg and isinstance(msg.get("gaze_config"), dict):
            self.last_gaze_config = dict(msg.get("gaze_config", {}))

    def _update_pick_fields(self, msg: dict[str, Any]) -> None:
        if "pick_running" in msg:
            self.last_pick_running = bool(msg.get("pick_running", False))
        if "pick_failed" in msg:
            self.last_pick_failed = bool(msg.get("pick_failed", False))
        if "pick_phase" in msg:
            self.last_pick_phase = str(msg.get("pick_phase", "idle") or "idle")
        if "pick_status_msg" in msg:
            self.last_pick_status_msg = str(msg.get("pick_status_msg", "") or "")

    def _tuple12(self, raw: Any) -> Optional[tuple[float, ...]]:
        if not isinstance(raw, (list, tuple)) or len(raw) != 12:
            return None
        try:
            return tuple(float(v) for v in raw)
        except (TypeError, ValueError):
            return None

    def _update_go2_motor_fields(self, msg: dict[str, Any]) -> None:
        q = self._tuple12(msg.get("go2_leg_q", None))
        if q is not None:
            self.last_go2_leg_q = q
        dq = self._tuple12(msg.get("go2_leg_dq", None))
        if dq is not None:
            self.last_go2_leg_dq = dq
        torque_nm = self._tuple12(msg.get("go2_leg_torque_nm", None))
        if torque_nm is not None:
            self.last_go2_leg_torque_nm = torque_nm

    def _send(self, msg: dict) -> None:
        outgoing = dict(msg)
        with self._io_lock:
            try:
                with message_span(
                    "control.transport.send",
                    outgoing,
                    endpoint=self.endpoint,
                    direction="send",
                ):
                    self.sock.send_json(outgoing, flags=zmq.NOBLOCK)
            except zmq.ZMQError as exc:
                self.is_connected = False
                self.last_reply_ok = False
                self.last_reply_reason = f"transport send failed: {exc}"

    def _send_hello(self, *, force: bool = False) -> None:
        now = time.time()
        if (
            not bool(force)
            and self._hello_period_s > 0.0
            and (now - self._t_last_hello) < self._hello_period_s
        ):
            return
        self._t_last_hello = now
        self._send({"t": "hello", "ts": now})

    def poll(self) -> None:
        with self._io_lock:
            self._poll_unlocked()

    def _poll_unlocked(self) -> None:
        try:
            events = dict(self.poller.poll(timeout=0))
        except zmq.ZMQError as exc:
            self.is_connected = False
            self.last_reply_ok = False
            self.last_reply_reason = f"transport poll failed: {exc}"
            return
        if self.sock not in events:
            return
        try:
            msg = self.sock.recv_json(flags=zmq.NOBLOCK)
        except ValueError as exc:
            self.last_reply_ok = False
            self.last_reply_reason = f"transport recv decode failed: {exc}"
            return
        except zmq.ZMQError as exc:
            self.is_connected = False
            self.last_reply_ok = False
            self.last_reply_reason = f"transport recv failed: {exc}"
            return

        with message_span(
            "control.transport.receive",
            msg,
            endpoint=self.endpoint,
            direction="receive",
        ):
            pass

        self._t_last_rx_wall = time.time()
        t = str(msg.get("t", "")).lower()
        if t == "ack":
            self.last_ack_ts = float(msg.get("ts", time.time()))
            self.last_reply_ok = bool(msg.get("ok", True))
            self.last_reply_reason = str(msg.get("reason", ""))
            if "ports" in msg and isinstance(msg.get("ports"), list):
                self.last_ports = [str(v) for v in msg.get("ports", [])]
            if "device" in msg:
                new_device = str(msg.get("device", ""))
                if new_device != self.last_device:
                    self.last_q = None
                    self.last_u = None
                    self.last_sim_q = None
                    self.last_sim_u = None
                    self.last_state_ts = 0.0
                self.last_device = new_device
            if "torque_enabled" in msg:
                self.torque_enabled = bool(msg.get("torque_enabled", False))
            if "claw_current" in msg:
                self.last_claw_current = int(msg.get("claw_current", 0))
            if "motor_currents_ma" in msg and isinstance(msg.get("motor_currents_ma"), dict):
                self.last_motor_currents_ma = {str(k): int(v) for k, v in dict(msg.get("motor_currents_ma", {})).items()}
            if "motor_positions_raw" in msg and isinstance(msg.get("motor_positions_raw"), dict):
                self.last_motor_positions_raw = {
                    str(k): int(v) for k, v in dict(msg.get("motor_positions_raw", {})).items()
                }
            if "motor_positions_deg" in msg and isinstance(msg.get("motor_positions_deg"), dict):
                self.last_motor_positions_deg = {
                    str(k): float(v) for k, v in dict(msg.get("motor_positions_deg", {})).items()
                }
            if "safety_fault" in msg:
                self.last_safety_fault = str(msg.get("safety_fault", ""))
            self._update_go2_motor_fields(msg)
            self._update_sim_clock_fields(msg)
            self._update_perception_fields(msg)
            self._update_gaze_fields(msg)
            self._update_pick_fields(msg)
            object_world_raw = msg.get("object_world", None)
            if isinstance(object_world_raw, (list, tuple)) and len(object_world_raw) == 3:
                self.last_object_world_xyz = (
                    float(object_world_raw[0]),
                    float(object_world_raw[1]),
                    float(object_world_raw[2]),
                )
            actual_tip_raw = msg.get("actual_tip", None)
            if isinstance(actual_tip_raw, (list, tuple)) and len(actual_tip_raw) == 3:
                self.last_actual_tip_xyz = (
                    float(actual_tip_raw[0]),
                    float(actual_tip_raw[1]),
                    float(actual_tip_raw[2]),
                )
            actual_tip_dir_raw = msg.get("actual_tip_dir", None)
            if isinstance(actual_tip_dir_raw, (list, tuple)) and len(actual_tip_dir_raw) == 3:
                self.last_actual_tip_dir = (
                    float(actual_tip_dir_raw[0]),
                    float(actual_tip_dir_raw[1]),
                    float(actual_tip_dir_raw[2]),
                )
            self.is_connected = True
            return

        if t == "state":
            self.last_state_ts = float(msg.get("ts", time.time()))
            if "q" in msg:
                try:
                    self.last_q = unpack_q(msg["q"])
                except (TypeError, ValueError) as exc:
                    self.last_reply_ok = False
                    self.last_reply_reason = f"state q decode failed: {exc}"
            if "u" in msg:
                try:
                    self.last_u = unpack_u(msg["u"])
                except (TypeError, ValueError) as exc:
                    self.last_reply_ok = False
                    self.last_reply_reason = f"state u decode failed: {exc}"
            if "sim_q" in msg:
                try:
                    self.last_sim_q = unpack_q(msg["sim_q"])
                    self.last_sim_u = sim_q_to_control_u(self.last_sim_q, self.cfg)
                except (TypeError, ValueError) as exc:
                    self.last_reply_ok = False
                    self.last_reply_reason = f"state sim_q decode failed: {exc}"
            if "torque_enabled" in msg:
                self.torque_enabled = bool(msg.get("torque_enabled", False))
            if "claw_current" in msg:
                self.last_claw_current = int(msg.get("claw_current", 0))
            if "motor_currents_ma" in msg and isinstance(msg.get("motor_currents_ma"), dict):
                self.last_motor_currents_ma = {str(k): int(v) for k, v in dict(msg.get("motor_currents_ma", {})).items()}
            if "motor_positions_raw" in msg and isinstance(msg.get("motor_positions_raw"), dict):
                self.last_motor_positions_raw = {
                    str(k): int(v) for k, v in dict(msg.get("motor_positions_raw", {})).items()
                }
            if "motor_positions_deg" in msg and isinstance(msg.get("motor_positions_deg"), dict):
                self.last_motor_positions_deg = {
                    str(k): float(v) for k, v in dict(msg.get("motor_positions_deg", {})).items()
                }
            if "safety_fault" in msg:
                self.last_safety_fault = str(msg.get("safety_fault", ""))
            self._update_go2_motor_fields(msg)
            if "go2_vel" in msg:
                try:
                    raw_go2_vel = msg.get("go2_vel", [0.0, 0.0, 0.0])
                    self.last_go2_vel = (
                        float(raw_go2_vel[0]),
                        float(raw_go2_vel[1]),
                        float(raw_go2_vel[2]),
                    )
                except (TypeError, ValueError, IndexError):
                    self.last_go2_vel = (0.0, 0.0, 0.0)
            rpy_raw = msg.get("go2_base_rpy", None)
            if isinstance(rpy_raw, (list, tuple)) and len(rpy_raw) == 3:
                self.last_go2_base_rpy = (float(rpy_raw[0]), float(rpy_raw[1]), float(rpy_raw[2]))
            pos_raw = msg.get("go2_base_pos", None)
            if isinstance(pos_raw, (list, tuple)) and len(pos_raw) == 3:
                self.last_go2_base_pos = (float(pos_raw[0]), float(pos_raw[1]), float(pos_raw[2]))
            lin_raw = msg.get("go2_base_lin_vel_body", None)
            if isinstance(lin_raw, (list, tuple)) and len(lin_raw) == 3:
                self.last_go2_base_lin_vel_body = (float(lin_raw[0]), float(lin_raw[1]), float(lin_raw[2]))
            ang_raw = msg.get("go2_base_ang_vel", None)
            if isinstance(ang_raw, (list, tuple)) and len(ang_raw) == 3:
                self.last_go2_base_ang_vel = (float(ang_raw[0]), float(ang_raw[1]), float(ang_raw[2]))
            if "go2_base_timestamp_s" in msg:
                try:
                    self.last_go2_base_timestamp_s = float(msg.get("go2_base_timestamp_s", 0.0))
                except (TypeError, ValueError):
                    pass
            if "go2_sport_pose" in msg:
                self.last_go2_sport_pose = str(msg.get("go2_sport_pose", "")).strip().lower()
            if "go2_sport_pose_seq" in msg:
                try:
                    self.last_go2_sport_pose_seq = int(msg.get("go2_sport_pose_seq", 0))
                except (TypeError, ValueError):
                    pass
            if "go2_obstacles_avoid_enabled" in msg:
                self.last_go2_obstacles_avoid_enabled = bool(msg.get("go2_obstacles_avoid_enabled", False))
            if "go2_obstacles_avoid_seq" in msg:
                try:
                    self.last_go2_obstacles_avoid_seq = int(msg.get("go2_obstacles_avoid_seq", 0))
                except (TypeError, ValueError):
                    pass
            self._update_sim_clock_fields(msg)
            self._update_perception_fields(msg)
            self._update_gaze_fields(msg)
            self._update_pick_fields(msg)
            actual_tip_raw = msg.get("actual_tip", None)
            if isinstance(actual_tip_raw, (list, tuple)) and len(actual_tip_raw) == 3:
                self.last_actual_tip_xyz = (
                    float(actual_tip_raw[0]),
                    float(actual_tip_raw[1]),
                    float(actual_tip_raw[2]),
                )
            actual_tip_dir_raw = msg.get("actual_tip_dir", None)
            if isinstance(actual_tip_dir_raw, (list, tuple)) and len(actual_tip_dir_raw) == 3:
                self.last_actual_tip_dir = (
                    float(actual_tip_dir_raw[0]),
                    float(actual_tip_dir_raw[1]),
                    float(actual_tip_dir_raw[2]),
                )
            self.is_connected = True
            if self.last_reply_reason == "":
                self.last_reply_ok = True

    def refresh_state(self) -> HostState:
        self.poll()
        if self._t_last_rx_wall <= 0.0 or self.rx_age_s() > 1.0:
            self._send_hello()
        return self.get_state()

    def estop(self) -> None:
        self._send({"t": "estop", "ts": time.time()})

    def send_sim_reset(self) -> None:
        self._send({"t": "sim_reset", "ts": time.time()})

    def torque_on(self, *, resume: bool = False) -> None:
        self._send(
            {
                "t": "torque_on",
                "ts": time.time(),
                "resume": bool(resume),
            }
        )

    def torque_off(self) -> None:
        self._send({"t": "torque_off", "ts": time.time()})

    def request_ports(self) -> None:
        self._send({"t": "ports", "ts": time.time()})

    def set_device(self, device: str) -> None:
        self._send({"t": "set_device", "ts": time.time(), "device": str(device)})

    def disconnect_device(self) -> None:
        self._send({"t": "disconnect_device", "ts": time.time()})

    def send_perception_start(self, *, config: Optional[Any] = None) -> None:
        now = time.time()
        payload: dict[str, Any] = {"t": "perception_start", "ts": now}
        cfg = config
        if cfg is not None:
            payload["config"] = {
                "detector_config": str(getattr(cfg, "detector_config", "")),
                "mode": str(getattr(cfg, "mode", "")),
                "detector": str(getattr(cfg, "detector", "")),
                "target_label": str(getattr(cfg, "target_label", "")),
                "yolo_device": str(getattr(cfg, "yolo_device", "")),
                "publish_hz": float(getattr(cfg, "publish_hz", 0.0)),
                "pipeline": str(getattr(cfg, "pipeline", "")),
                "tracker": str(getattr(cfg, "tracker", "")),
            }
        self._send(payload)

    def send_perception_stop(self) -> None:
        self._send({"t": "perception_stop", "ts": time.time()})

    def send_perception_refresh(self) -> None:
        self._send({"t": "perception_refresh", "ts": time.time()})

    def send_perception_capture(self, *, include_overlay: bool = True) -> None:
        self._send(
            {
                "t": "perception_capture",
                "ts": time.time(),
                "include_overlay": bool(include_overlay),
            }
        )

    def send_perception_record_start(self, *, include_overlay: bool = False, fps: float = 0.0) -> None:
        self._send(
            {
                "t": "perception_record_start",
                "ts": time.time(),
                "include_overlay": bool(include_overlay),
                "fps": float(fps),
            }
        )

    def send_perception_record_stop(self) -> None:
        self._send({"t": "perception_record_stop", "ts": time.time()})

    def send_gaze_start_standing(self, *, run_id: str = "") -> None:
        now = time.time()
        self.tx_seq += 1
        self._send(
            {
                "t": "gaze_start_standing",
                "ts": now,
                "seq": self.tx_seq,
                "run_id": str(run_id),
            }
        )

    def send_gaze_start_walking(self, *, run_id: str = "", gaze_mode: str = "") -> None:
        now = time.time()
        self.tx_seq += 1
        self._send(
            {
                "t": "gaze_start_walking",
                "ts": now,
                "seq": self.tx_seq,
                "run_id": str(run_id),
                "gaze_mode": str(gaze_mode),
            }
        )

    def send_gaze_stop(self) -> None:
        now = time.time()
        self.tx_seq += 1
        self._send({"t": "gaze_stop", "ts": now, "seq": self.tx_seq})

    def stop_lji_velocity_control(self, *, reason: str = "client_stop") -> None:
        now = time.time()
        self.tx_seq += 1
        self._send(
            {
                "t": "lji_velocity_stop",
                "ts": now,
                "seq": self.tx_seq,
                "reason": str(reason),
            }
        )

    def send_gaze_config_update(self, config: dict[str, Any]) -> None:
        now = time.time()
        self.tx_seq += 1
        self._send(
            {
                "t": "gaze_config_update",
                "ts": now,
                "seq": self.tx_seq,
                "config": dict(config),
            }
        )

    def send_mobile_pick_start(self) -> None:
        now = time.time()
        self.tx_seq += 1
        self._send({"t": "mobile_pick_start", "ts": now, "seq": self.tx_seq})

    def send_lji_grasp_start(self) -> None:
        now = time.time()
        self.tx_seq += 1
        self._send({"t": "lji_grasp_start", "ts": now, "seq": self.tx_seq})

    def send_pick_stop(self) -> None:
        now = time.time()
        self.tx_seq += 1
        self._send({"t": "pick_stop", "ts": now, "seq": self.tx_seq})

    def send_mobile_pick_stop(self) -> None:
        self.send_pick_stop()

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
        now = time.time()
        with self._io_lock:
            self.tx_seq += 1
            self.last_perceived_center_uv = (float(image_center_uv[0]), float(image_center_uv[1]))
            self.last_perceived_scale = float(image_scale)
            self.last_perceived_timestamp_s = float(now)
            self.last_perceived_object_label = str(label)
            self.last_perceived_object_confidence = float(confidence)
            self.last_perceived_object_camera_xyz = (
                float(object_camera_xyz[0]),
                float(object_camera_xyz[1]),
                float(object_camera_xyz[2]),
            )
            if bool(depth_valid) and object_world is None:
                self.last_object_world_xyz = None
            try:
                payload: dict[str, Any] = {
                    "t": "target",
                    "ts": now,
                    "seq": self.tx_seq,
                    "source": "perception",
                    "object_camera": [
                        float(object_camera_xyz[0]),
                        float(object_camera_xyz[1]),
                        float(object_camera_xyz[2]),
                    ],
                    "object_label": str(label),
                    "object_confidence": float(confidence),
                    "image_center_uv": [float(image_center_uv[0]), float(image_center_uv[1])],
                    "image_scale": float(image_scale),
                    "depth_valid": bool(depth_valid),
                }
                if object_world is not None:
                    payload["object_world"] = [
                        float(object_world[0]),
                        float(object_world[1]),
                        float(object_world[2]),
                    ]
                if camera_world_origin is not None:
                    payload["camera_world_origin"] = [float(x) for x in camera_world_origin]
                if camera_world_look is not None:
                    payload["camera_world_look"] = [float(x) for x in camera_world_look]
                if camera_world_right is not None:
                    payload["camera_world_right"] = [float(x) for x in camera_world_right]
                self.sock.send_json(payload, flags=zmq.NOBLOCK)
            except zmq.ZMQError as exc:
                self.is_connected = False
                self.last_reply_ok = False
                self.last_reply_reason = f"transport send failed: {exc}"
                return None
            if not bool(depth_valid):
                return self.last_object_world_xyz
            wait_s = max(float(wait_ack_s), 0.0)
            if wait_s <= 0.0:
                return self.last_object_world_xyz
            deadline = time.time() + wait_s
            while time.time() < deadline:
                self._poll_unlocked()
                if self.last_object_world_xyz is not None:
                    return self.last_object_world_xyz
                time.sleep(0.005)
            return self.last_object_world_xyz

    def send_claw_command(self, *, claw_closed: bool, source: str = "target") -> None:
        now = time.time()
        self.tx_seq += 1
        self._send({"t": "target", "ts": now, "seq": self.tx_seq, "source": str(source), "claw_closed": bool(claw_closed)})

    def send_go2_velocity(self, *, vx: float, vy: float, wz: float, source: str = "target") -> None:
        now = time.time()
        self.tx_seq += 1
        self._send(
            {
                "t": "target",
                "ts": now,
                "seq": self.tx_seq,
                "source": str(source),
                "go2_vel": [float(vx), float(vy), float(wz)],
            }
        )

    def send_go2_sport_pose(self, *, pose: str, source: str = "target") -> None:
        now = time.time()
        self.tx_seq += 1
        self._send(
            {
                "t": "target",
                "ts": now,
                "seq": self.tx_seq,
                "source": str(source),
                "go2_sport_pose": str(pose).strip().lower(),
            }
        )

    def send_go2_obstacles_avoid(self, *, enabled: bool, source: str = "target") -> None:
        now = time.time()
        self.tx_seq += 1
        self._send(
            {
                "t": "target",
                "ts": now,
                "seq": self.tx_seq,
                "source": str(source),
                "go2_obstacles_avoid_enable": bool(enabled),
            }
        )

    def send_sim_target_xyz(self, *, xyz: tuple[float, float, float], source: str = "target") -> None:
        now = time.time()
        self.tx_seq += 1
        self._send(
            {
                "t": "target",
                "ts": now,
                "seq": self.tx_seq,
                "source": str(source),
                "sim_target": [float(xyz[0]), float(xyz[1]), float(xyz[2])],
            }
        )

    def send_partial_control_u(self, partial_u: dict[str, float], *, source: str = "slider") -> None:
        now = time.time()
        self.tx_seq += 1
        self._send({"t": "target", "ts": now, "seq": self.tx_seq, "source": str(source), "u": {str(k): float(v) for k, v in partial_u.items()}})

    def send_debug_markers(self, markers: list[dict[str, Any]], *, source: str = "target") -> None:
        now = time.time()
        self.tx_seq += 1
        self._send(
            {
                "t": "target",
                "ts": now,
                "seq": self.tx_seq,
                "source": str(source),
                "debug_markers": [dict(marker) for marker in markers],
            }
        )

    def send_target_meta(
        self,
        *,
        target_xyz: tuple[float, float, float],
        target_dir: tuple[float, float, float],
        source: str = "target",
    ) -> None:
        now = time.time()
        self.tx_seq += 1
        self._send(
            {
                "t": "target",
                "ts": now,
                "seq": self.tx_seq,
                "source": str(source),
                "target": [float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])],
                "target_dir": [float(target_dir[0]), float(target_dir[1]), float(target_dir[2])],
            }
        )

    def send_ready_pose_meta(
        self,
        *,
        target_dir: tuple[float, float, float],
        standoff_m: float,
        source: str = "target",
    ) -> None:
        now = time.time()
        self.tx_seq += 1
        self._send(
            {
                "t": "target",
                "ts": now,
                "seq": self.tx_seq,
                "source": str(source),
                "ready_pose_dir": [float(target_dir[0]), float(target_dir[1]), float(target_dir[2])],
                "ready_pose_standoff_m": float(standoff_m),
            }
        )

    def send_sag_model_meta(self, sag_model: dict[str, Any], *, source: str = "target") -> None:
        now = time.time()
        self.tx_seq += 1
        self._send(
            {
                "t": "target",
                "ts": now,
                "seq": self.tx_seq,
                "source": str(source),
                "sag_model": dict(sag_model),
            }
        )

    def maybe_send_target_q(self, q: SimQ, *, source: str = "sim", force: bool = False) -> None:
        self._maybe_send_target_q(q, source=source, target_xyz=None, target_dir=None, sag_model=None, claw_closed=None, force=force)

    def _maybe_send_target_q(
        self,
        q: SimQ,
        *,
        source: str,
        target_xyz: Optional[tuple[float, float, float]],
        target_dir: Optional[tuple[float, float, float]],
        sag_model: Optional[dict[str, Any]],
        claw_closed: Optional[bool],
        force: bool = False,
    ) -> None:
        now = time.time()
        if (not force) and self._send_period > 0 and (now - self._t_last_tx) < self._send_period:
            return
        self._t_last_tx = now
        self.tx_seq += 1
        msg = {
            "t": "target",
            "ts": now,
            "seq": self.tx_seq,
            "source": str(source),
            "q": {
                "linear_m": float(q.linear_m),
                "roll_rad": float(q.roll_rad),
                "theta1_rad": float(q.theta1_rad),
                "theta2_rad": float(q.theta2_rad),
            },
        }
        if target_xyz is not None:
            msg["target"] = [float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])]
        if target_dir is not None:
            msg["target_dir"] = [float(target_dir[0]), float(target_dir[1]), float(target_dir[2])]
        if sag_model is not None:
            msg["sag_model"] = dict(sag_model)
        if claw_closed is not None:
            msg["claw_closed"] = bool(claw_closed)
        self._send(msg)

    def send_target_q(self, q: SimQ, *, source: str = "ui", force: bool = False) -> None:
        self.maybe_send_target_q(q, source=source, force=force)

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
        sag_model: Optional[dict[str, str]] = None,
        claw_closed: Optional[bool] = None,
        force: bool = False,
    ) -> None:
        self._maybe_send_target_q(
            SimQ(
                linear_m=float(linear_m),
                roll_rad=float(roll_rad),
                theta1_rad=float(theta1_rad),
                theta2_rad=float(theta2_rad),
            ),
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
            SimQ(
                linear_m=float(linear_m),
                roll_rad=float(roll_rad),
                theta1_rad=float(theta1_rad),
                theta2_rad=float(theta2_rad),
            ),
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
            ControlU(
                u_linear=float(u_linear),
                u_roll=float(u_roll),
                u_s1=float(u_s1),
                u_s2=float(u_s2),
            ),
            self.cfg,
        )
