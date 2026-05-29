from __future__ import annotations

import os
import time
from typing import Any, Optional

import zmq

from engine.protocol import (
    ControlU,
    SimMappingConfig,
    SimQ,
    control_u_to_sim_q,
    sim_q_to_control_u,
    unpack_q,
    unpack_u,
)
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

        self.is_connected = True
        self.tx_seq = 0
        self._t_last_tx = 0.0

        self.last_ack_ts = 0.0
        self.last_state_ts = 0.0
        self._t_last_rx_wall = 0.0
        self.last_q: SimQ | None = None
        self.last_u: ControlU | None = None
        self.last_ports: list[str] = []
        self.last_device: str = ""
        self.torque_enabled: bool = False
        self.last_claw_current: int = 0
        self.last_motor_currents_ma: dict[str, int] = {}
        self.last_safety_fault: str = ""
        self.last_actual_tip_xyz: Optional[tuple[float, float, float]] = None
        self.last_actual_tip_dir: Optional[tuple[float, float, float]] = None
        self.last_perceived_object_label: str = ""
        self.last_perceived_object_confidence: float = 0.0
        self.last_perceived_object_camera_xyz: Optional[tuple[float, float, float]] = None
        self.last_perceived_center_uv: Optional[tuple[float, float]] = None
        self.last_perceived_scale: Optional[float] = None
        self.last_perceived_timestamp_s: float = 0.0
        self.last_object_world_xyz: Optional[tuple[float, float, float]] = None
        self.last_reply_ok: bool = True
        self.last_reply_reason: str = ""

        self._send({"t": "hello", "ts": time.time()})

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
            safety_fault=str(self.last_safety_fault),
            actual_tip_xyz=self.last_actual_tip_xyz,
            actual_tip_dir=self.last_actual_tip_dir,
            perceived_object_label=str(self.last_perceived_object_label),
            perceived_object_confidence=float(self.last_perceived_object_confidence),
            perceived_object_camera_xyz=self.last_perceived_object_camera_xyz,
            perceived_center_uv=self.last_perceived_center_uv,
            perceived_scale=self.last_perceived_scale,
            perceived_timestamp_s=float(self.last_perceived_timestamp_s),
            reply_ok=bool(self.last_reply_ok),
            reply_reason=str(self.last_reply_reason),
            q=self.last_q,
            u=self.last_u,
        )

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

    def _send(self, msg: dict) -> None:
        try:
            self.sock.send_json(msg, flags=zmq.NOBLOCK)
        except zmq.ZMQError as exc:
            self.is_connected = False
            self.last_reply_ok = False
            self.last_reply_reason = f"transport send failed: {exc}"

    def poll(self) -> None:
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
                    self.last_state_ts = 0.0
                self.last_device = new_device
            if "torque_enabled" in msg:
                self.torque_enabled = bool(msg.get("torque_enabled", False))
            if "claw_current" in msg:
                self.last_claw_current = int(msg.get("claw_current", 0))
            if "motor_currents_ma" in msg and isinstance(msg.get("motor_currents_ma"), dict):
                self.last_motor_currents_ma = {str(k): int(v) for k, v in dict(msg.get("motor_currents_ma", {})).items()}
            if "safety_fault" in msg:
                self.last_safety_fault = str(msg.get("safety_fault", ""))
            self._update_perception_fields(msg)
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
            if "torque_enabled" in msg:
                self.torque_enabled = bool(msg.get("torque_enabled", False))
            if "claw_current" in msg:
                self.last_claw_current = int(msg.get("claw_current", 0))
            if "motor_currents_ma" in msg and isinstance(msg.get("motor_currents_ma"), dict):
                self.last_motor_currents_ma = {str(k): int(v) for k, v in dict(msg.get("motor_currents_ma", {})).items()}
            if "safety_fault" in msg:
                self.last_safety_fault = str(msg.get("safety_fault", ""))
            self._update_perception_fields(msg)
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
        return self.get_state()

    def estop(self) -> None:
        self._send({"t": "estop", "ts": time.time()})

    def torque_on(self) -> None:
        self._send({"t": "torque_on", "ts": time.time()})

    def torque_off(self) -> None:
        self._send({"t": "torque_off", "ts": time.time()})

    def request_ports(self) -> None:
        self._send({"t": "ports", "ts": time.time()})

    def set_device(self, device: str) -> None:
        self._send({"t": "set_device", "ts": time.time(), "device": str(device)})

    def disconnect_device(self) -> None:
        self._send({"t": "disconnect_device", "ts": time.time()})

    def send_perception_observation(
        self,
        *,
        object_camera_xyz: tuple[float, float, float],
        label: str = "",
        confidence: float = 0.0,
        image_center_uv: tuple[float, float],
        image_scale: float,
        wait_ack_s: float = 0.25,
    ) -> Optional[tuple[float, float, float]]:
        now = time.time()
        self.tx_seq += 1
        self.last_object_world_xyz = None
        self._send(
            {
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
            }
        )
        deadline = time.time() + max(float(wait_ack_s), 0.0)
        while time.time() < deadline:
            self.poll()
            if self.last_object_world_xyz is not None:
                return self.last_object_world_xyz
            time.sleep(0.01)
        return self.last_object_world_xyz

    def send_claw_command(self, *, claw_closed: bool, source: str = "target") -> None:
        now = time.time()
        self.tx_seq += 1
        self._send({"t": "target", "ts": now, "seq": self.tx_seq, "source": str(source), "claw_closed": bool(claw_closed)})

    def send_partial_control_u(self, partial_u: dict[str, float], *, source: str = "slider") -> None:
        now = time.time()
        self.tx_seq += 1
        self._send({"t": "target", "ts": now, "seq": self.tx_seq, "source": str(source), "u": {str(k): float(v) for k, v in partial_u.items()}})

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
