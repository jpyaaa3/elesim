#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import math
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Optional, Tuple

import numpy as np
import glfw
import imgui
from imgui.integrations.glfw import GlfwRenderer
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
from engine import ik as ik_pipeline
from engine.config_loader import IkConfig
from engine.sag_model import load_sag_model_json

DEFAULT_SAG_MODEL_PATH = os.path.join(os.path.dirname(__file__), "assets", "sag_model.json")


def _resolve_sag_model_path(path: str) -> str:
    raw = str(path or "").strip()
    if not raw:
        return DEFAULT_SAG_MODEL_PATH
    if os.path.isabs(raw):
        return raw
    return os.path.abspath(raw)


def _load_sag_model_or_empty(path: str) -> dict[str, Any]:
    model = load_sag_model_json(_resolve_sag_model_path(path))
    return dict(model) if isinstance(model, dict) else {}


def _resolve_initial_sag_model() -> dict[str, Any]:
    try:
        model = _load_sag_model_or_empty(DEFAULT_SAG_MODEL_PATH)
        if isinstance(model, dict) and model:
            return model
    except Exception:
        pass
    return {}


@dataclass(frozen=True)
class HostState:
    connected: bool
    tx_seq: int
    rx_age_s: float
    device: str
    ports: tuple[str, ...]
    torque_enabled: bool
    claw_current: int
    actual_tip_xyz: Optional[tuple[float, float, float]]
    reply_ok: bool
    reply_reason: str
    q: Optional[SimQ]
    u: Optional[ControlU]


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
        self.last_actual_tip_xyz: Optional[tuple[float, float, float]] = None
        self.last_reply_ok: bool = True
        self.last_reply_reason: str = ""

        self._send({"t": "hello", "ts": time.time()})

    def close(self) -> None:
        try:
            self.poller.unregister(self.sock)
        except KeyError:
            pass
        except AttributeError:
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
            actual_tip_xyz=self.last_actual_tip_xyz,
            reply_ok=bool(self.last_reply_ok),
            reply_reason=str(self.last_reply_reason),
            q=self.last_q,
            u=self.last_u,
        )

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
            actual_tip_raw = msg.get("actual_tip", None)
            if isinstance(actual_tip_raw, (list, tuple)) and len(actual_tip_raw) == 3:
                self.last_actual_tip_xyz = (
                    float(actual_tip_raw[0]),
                    float(actual_tip_raw[1]),
                    float(actual_tip_raw[2]),
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
            actual_tip_raw = msg.get("actual_tip", None)
            if isinstance(actual_tip_raw, (list, tuple)) and len(actual_tip_raw) == 3:
                self.last_actual_tip_xyz = (
                    float(actual_tip_raw[0]),
                    float(actual_tip_raw[1]),
                    float(actual_tip_raw[2]),
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

    def send_claw_command(self, *, claw_closed: bool, source: str = "target") -> None:
        now = time.time()
        self.tx_seq += 1
        self._send(
            {
                "t": "target",
                "ts": now,
                "seq": self.tx_seq,
                "source": str(source),
                "claw_closed": bool(claw_closed),
            }
        )

    def send_partial_control_u(self, partial_u: dict[str, float], *, source: str = "slider") -> None:
        now = time.time()
        self.tx_seq += 1
        self._send(
            {
                "t": "target",
                "ts": now,
                "seq": self.tx_seq,
                "source": str(source),
                "u": {str(k): float(v) for k, v in partial_u.items()},
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
                "target_dir": [
                    float(target_dir[0]),
                    float(target_dir[1]),
                    float(target_dir[2]),
                ],
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
            msg["target_dir"] = [
                float(target_dir[0]),
                float(target_dir[1]),
                float(target_dir[2]),
            ]
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


@dataclass
class PanelState:
    linear: float = 0.0
    roll: float = 0.0
    theta1: float = 0.0
    theta2: float = 0.0
    paused: bool = False
    claw_closed: bool = False

    # IK target (world coordinates)
    target_x: float = 0.50
    target_y: float = 0.00
    target_z: float = 1.00
    target_vx: float = 1.0
    target_vy: float = 0.0
    target_vz: float = 0.0
    sag_model_path: str = DEFAULT_SAG_MODEL_PATH
    raw_sag_model: Optional[dict[str, Any]] = None

    # IK runtime status (read-only for UI thread)
    ik_running: bool = False
    ik_converged: bool = False
    ik_failed: bool = False
    ik_err_m: float = 0.0
    # Debug: physics tracking + sim tip error (to diagnose 'fake converge')
    ik_sim_tip_err_m: float = 0.0
    ik_track_roll_err_rad: float = 0.0
    ik_track_theta1_err_rad: float = 0.0
    ik_track_theta2_err_rad: float = 0.0
    ik_track_bend_max_err_rad: float = 0.0

    # IK best/current solution (for UI display)
    ik_sol_roll: float = 0.0
    ik_sol_theta1: float = 0.0
    ik_sol_theta2: float = 0.0

    _lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)

    def snapshot(self) -> Tuple[float, float, float, float, bool, Tuple[float, float, float], Tuple[float, float, float], dict[str, Any]]:
        with self._lock:
            sag_model = dict(self.raw_sag_model) if isinstance(self.raw_sag_model, dict) else {}
            return (
                self.linear,
                self.roll,
                self.theta1,
                self.theta2,
                self.paused,
                (self.target_x, self.target_y, self.target_z),
                (self.target_vx, self.target_vy, self.target_vz),
                sag_model,
            )

    def set_all(self, linear: float, roll: float, theta1: float, theta2: float, paused: bool) -> None:
        with self._lock:
            self.linear = float(linear)
            self.roll = float(roll)
            self.theta1 = float(theta1)
            self.theta2 = float(theta2)
            self.paused = bool(paused)

    def set_q(self, linear: float, roll: float, theta1: float, theta2: float) -> None:
        with self._lock:
            self.linear = float(linear)
            self.roll = float(roll)
            self.theta1 = float(theta1)
            self.theta2 = float(theta2)

    def reset_q(self) -> None:
        self.set_q(0.0, 0.0, 0.0, 0.0)

    def set_target(self, x: float, y: float, z: float) -> None:
        with self._lock:
            self.target_x = float(x)
            self.target_y = float(y)
            self.target_z = float(z)

    def set_target_dir(self, vx: float, vy: float, vz: float) -> None:
        with self._lock:
            self.target_vx = float(vx)
            self.target_vy = float(vy)
            self.target_vz = float(vz)

    def set_sag_model(self, model_path: str, sag_model: dict[str, Any]) -> None:
        with self._lock:
            self.sag_model_path = str(model_path)
            self.raw_sag_model = dict(sag_model)

    def set_paused(self, paused: bool) -> None:
        with self._lock:
            self.paused = bool(paused)

    def toggle_claw_closed(self) -> None:
        with self._lock:
            self.claw_closed = not bool(self.claw_closed)

    def set_claw_closed(self, closed: bool) -> None:
        with self._lock:
            self.claw_closed = bool(closed)

    def set_ik_status(self, running: bool, converged: bool, failed: bool, err_m: float) -> None:
        with self._lock:
            self.ik_running = bool(running)
            self.ik_converged = bool(converged)
            self.ik_failed = bool(failed)
            self.ik_err_m = float(err_m)

    def clear_ik_status(self) -> None:
        self.set_ik_status(running=False, converged=False, failed=False, err_m=0.0)

    def set_ik_solution(self, roll: float, theta1: float, theta2: float) -> None:
        with self._lock:
            self.ik_sol_roll = float(roll)
            self.ik_sol_theta1 = float(theta1)
            self.ik_sol_theta2 = float(theta2)

    def set_ik_debug(
        self,
        *,
        sim_tip_err_m: float,
        roll_err_rad: float,
        theta1_err_rad: float,
        theta2_err_rad: float,
        bend_max_err_rad: float,
    ) -> None:
        """UI display helpers: sim-thread writes debug error metrics for 'fake converge' diagnosis."""
        with self._lock:
            self.ik_sim_tip_err_m = float(sim_tip_err_m)
            self.ik_track_roll_err_rad = float(roll_err_rad)
            self.ik_track_theta1_err_rad = float(theta1_err_rad)
            self.ik_track_theta2_err_rad = float(theta2_err_rad)
            self.ik_track_bend_max_err_rad = float(bend_max_err_rad)


class ControlService:
    """Controller-side actions: IK solve, target send, host commands."""

    def __init__(
        self,
        state: PanelState,
        client: Optional[ControlClient] = None,
        mapping_cfg: Optional[SimMappingConfig] = None,
        ik_cfg: Optional[IkConfig] = None,
        ik_context: Optional[dict[str, Any]] = None,
        config_path: Optional[str] = None,
    ) -> None:
        self.state = state
        self.client = client
        self._mapping_cfg = mapping_cfg or SimMappingConfig()
        self._ik_cfg = ik_cfg or IkConfig()
        self._ik_context = dict(ik_context or {})
        self._config_path = None if config_path is None else str(config_path)
        self._ik_worker: Optional[threading.Thread] = None

    def refresh_ik_context(self) -> None:
        if not self._config_path:
            return
        try:
            _, ik_context = ik_pipeline.load_solver_context(self._config_path)
            self._ik_context = dict(ik_context or {})
        except Exception as exc:
            print(f"[UI] IK context reload failed: {exc}")

    def refresh_host_state(self) -> Optional[HostState]:
        if self.client is None:
            return None
        host_state = self.client.refresh_state()
        if host_state.q is not None:
            self.state.set_q(
                float(host_state.q.linear_m),
                float(host_state.q.roll_rad),
                float(host_state.q.theta1_rad),
                float(host_state.q.theta2_rad),
            )
        return host_state

    def has_client(self) -> bool:
        return self.client is not None

    def current_host_state(self) -> Optional[HostState]:
        if self.client is None:
            return None
        return self.client.get_state()

    def current_control_u(self) -> ControlU:
        if self.client is not None:
            return self.client.q_to_control_u(
                linear_m=float(self.state.linear),
                roll_rad=float(self.state.roll),
                theta1_rad=float(self.state.theta1),
                theta2_rad=float(self.state.theta2),
            )
        return sim_q_to_control_u(
            SimQ(
                linear_m=float(self.state.linear),
                roll_rad=float(self.state.roll),
                theta1_rad=float(self.state.theta1),
                theta2_rad=float(self.state.theta2),
            ),
            self._mapping_cfg,
        )

    def control_mapping(self) -> SimMappingConfig:
        return self.client.cfg if self.client is not None else self._mapping_cfg

    def apply_control_u(self, *, u_linear: float, u_roll: float, u_s1: float, u_s2: float) -> None:
        if self.client is not None:
            q_new = self.client.control_u_to_q(
                u_linear=float(u_linear),
                u_roll=float(u_roll),
                u_s1=float(u_s1),
                u_s2=float(u_s2),
            )
        else:
            q_new = control_u_to_sim_q(
                ControlU(
                    u_linear=float(u_linear),
                    u_roll=float(u_roll),
                    u_s1=float(u_s1),
                    u_s2=float(u_s2),
                ),
                self._mapping_cfg,
            )
        self.state.set_q(
            float(q_new.linear_m),
            float(q_new.roll_rad),
            float(q_new.theta1_rad),
            float(q_new.theta2_rad),
        )

    def apply_partial_control_u(self, partial_u: dict[str, float]) -> None:
        current_u = self.current_control_u()
        merged = {
            "linear": float(current_u.u_linear),
            "roll": float(current_u.u_roll),
            "s1": float(current_u.u_s1),
            "s2": float(current_u.u_s2),
        }
        for key, value in partial_u.items():
            merged[str(key).strip().lower()] = float(value)
        self.apply_control_u(
            u_linear=float(merged["linear"]),
            u_roll=float(merged["roll"]),
            u_s1=float(merged["s1"]),
            u_s2=float(merged["s2"]),
        )
        if self.client is not None:
            self.client.send_partial_control_u({str(k).strip().lower(): float(v) for k, v in partial_u.items()}, source="slider")

    def send_current_target(self, *, source: str) -> None:
        if self.client is not None and ((not self.state.paused) or (source == "target")):
            self.client.send_target_values(
                linear_m=float(self.state.linear),
                roll_rad=float(self.state.roll),
                theta1_rad=float(self.state.theta1),
                theta2_rad=float(self.state.theta2),
                source=source,
                target_xyz=(float(self.state.target_x), float(self.state.target_y), float(self.state.target_z)),
                target_dir=(
                    float(self.state.target_vx),
                    float(self.state.target_vy),
                    float(self.state.target_vz),
                ),
                sag_model=(dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {}),
                claw_closed=bool(self.state.claw_closed),
                force=bool(source == "target"),
            )

    def send_current_target_meta(self, *, source: str = "target") -> None:
        if self.client is not None:
            self.client.send_target_meta(
                target_xyz=(float(self.state.target_x), float(self.state.target_y), float(self.state.target_z)),
                target_dir=(
                    float(self.state.target_vx),
                    float(self.state.target_vy),
                    float(self.state.target_vz),
                ),
                source=source,
            )

    def load_sag_model(self, model_path: str) -> tuple[str, dict[str, Any]]:
        resolved_path = _resolve_sag_model_path(model_path)
        model = _load_sag_model_or_empty(resolved_path)
        self.state.set_sag_model(resolved_path, model)
        return resolved_path, model

    def send_claw_command(self, *, closed: bool) -> None:
        if self.client is not None:
            self.client.send_claw_command(claw_closed=bool(closed), source="target")

    def _start_position_solve(self, target: np.ndarray) -> None:
        if self.state.ik_running:
            return
        self.refresh_ik_context()
        ctx = dict(self._ik_context)
        ctx["sag_model"] = dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {}
        required = (
            "limit",
            "fk_joint_chain",
            "terminal_link_name",
            "old_tip_local_offset",
            "grasp_offset_node_local",
        )
        if any(k not in ctx for k in required):
            print("[UI] IK solve rejected | missing ik_context fields")
            self.state.set_ik_status(running=False, converged=False, failed=True, err_m=float("inf"))
            return

        self.state.set_ik_status(running=True, converged=False, failed=False, err_m=float("inf"))

        def _worker() -> None:
            try:
                current_seed = np.array(
                    [
                        float(self.state.linear),
                        float(self.state.roll),
                        float(self.state.theta1),
                        float(self.state.theta2),
                    ],
                    dtype=float,
                )
                result = ik_pipeline.solve_then_tweak(
                    target_world=target,
                    target_dir_world=np.array([self.state.target_vx, self.state.target_vy, self.state.target_vz], dtype=float),
                    context=ctx,
                    position_tol_m=float(self._ik_cfg.tol),
                    max_iters=max(int(self._ik_cfg.max_iters), 1),
                    current_seed=current_seed,
                )
                if result.success and result.q is not None:
                    q = np.asarray(result.q, dtype=float).reshape(4)
                    refined_pos_err = float(result.position_error_m)
                    self.state.set_q(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
                    self.state.set_ik_solution(float(q[1]), float(q[2]), float(q[3]))
                    self.state.set_ik_status(running=False, converged=True, failed=False, err_m=refined_pos_err)
                    self.send_current_target(source="ik")
                else:
                    print(
                        "[UI] IK solve failed | target=(%.3f, %.3f, %.3f) | err=%s"
                        % (float(target[0]), float(target[1]), float(target[2]), float(result.position_error_m))
                    )
                    self.state.set_ik_status(
                        running=False,
                        converged=False,
                        failed=True,
                        err_m=float(result.position_error_m),
                    )
            finally:
                self._ik_worker = None

        self._ik_worker = threading.Thread(target=_worker, daemon=True)
        self._ik_worker.start()

    def start_ik_solve(self) -> None:
        target = np.array([self.state.target_x, self.state.target_y, self.state.target_z], dtype=float)
        self._start_position_solve(target)

    def start_tweak(self) -> None:
        if self.state.ik_running:
            return
        self.refresh_ik_context()
        ctx = dict(self._ik_context)
        ctx["sag_model"] = dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {}
        required = (
            "limit",
            "fk_joint_chain",
            "terminal_link_name",
            "approach_axis_local",
        )
        if any(k not in ctx for k in required):
            print("[UI] Tweak rejected | missing ik_context fields")
            self.state.set_ik_status(running=False, converged=False, failed=True, err_m=float("inf"))
            return

        direction = np.array([self.state.target_vx, self.state.target_vy, self.state.target_vz], dtype=float)
        dnorm = float(np.linalg.norm(direction))
        if dnorm <= 1e-9:
            self.state.set_ik_status(running=False, converged=False, failed=True, err_m=float("inf"))
            return
        target_dir = direction / dnorm
        self.state.set_ik_status(running=True, converged=False, failed=False, err_m=float("inf"))

        def _worker() -> None:
            try:
                current_q = np.array(
                    [
                        float(self.state.linear),
                        float(self.state.roll),
                        float(self.state.theta1),
                        float(self.state.theta2),
                    ],
                    dtype=float,
                )
                hold_target = None
                if self.client is not None:
                    host_state = self.client.refresh_state()
                    if host_state is not None and host_state.actual_tip_xyz is not None:
                        hold_target = np.array(host_state.actual_tip_xyz, dtype=float).reshape(3)
                if hold_target is None:
                    hold_target = None
                tweak_result = ik_pipeline.tweak_only(
                    current_q=current_q,
                    hold_target_world=hold_target,
                    target_dir_world=target_dir,
                    context=ctx,
                    position_hold_tol_m=5e-3,
                    rounds=10,
                )
                q = np.asarray(tweak_result.q, dtype=float).reshape(4)
                self.state.set_q(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
                self.state.set_ik_solution(float(q[1]), float(q[2]), float(q[3]))
                self.state.set_ik_status(
                    running=False,
                    converged=bool(tweak_result.converged),
                    failed=not bool(tweak_result.converged),
                    err_m=float(tweak_result.position_error_m),
                )
                self.send_current_target(source="ik")
            except Exception as exc:
                print(f"[UI] Tweak failed: {exc}")
                self.state.set_ik_status(running=False, converged=False, failed=True, err_m=float("inf"))
            finally:
                self._ik_worker = None

        self._ik_worker = threading.Thread(target=_worker, daemon=True)
        self._ik_worker.start()

    def request_ports(self) -> None:
        if self.client is not None:
            self.client.request_ports()

    def set_device(self, device: str) -> None:
        if self.client is not None:
            self.client.set_device(device)

    def disconnect_device(self) -> None:
        if self.client is not None:
            self.client.disconnect_device()

    def torque_on(self) -> None:
        if self.client is not None:
            self.client.torque_on()

    def torque_off(self) -> None:
        if self.client is not None:
            self.client.torque_off()

    def close(self) -> None:
        if self.client is not None:
            self.client.close()


class ControlPanel:
    """External ImGui window that draws and edits PanelState."""

    def __init__(
        self,
        state: PanelState,
        service: ControlService,
        *,
        use_hardware: bool = False,
    ):
        self.state = state
        self.service = service
        self._use_hardware = bool(use_hardware)
        self._stop = False
        self._hw_header_init_open = False
        self._ctrl_header_init_open = False
        self._ik_header_init_open = False
        self._sag_header_init_open = False
        self._ctrl_window_init = False
        self._port_input = ""
        self._host_state: Optional[HostState] = None
        self._sag_model_path_draft = str(self.state.sag_model_path)
        self._sag_status_text = ""
        self._sag_status_ok = True

    def stop(self) -> None:
        self._stop = True

    def _begin_disabled_ui(self, disabled: bool) -> str | None:
        if not disabled:
            return None
        begin_disabled = getattr(imgui, "begin_disabled", None)
        if callable(begin_disabled):
            begin_disabled()
            return "begin_disabled"
        item_disabled = getattr(imgui, "ITEM_DISABLED", None)
        push_item_flag = getattr(imgui, "push_item_flag", None)
        push_style_var = getattr(imgui, "push_style_var", None)
        style_alpha = getattr(imgui, "STYLE_ALPHA", None)
        if item_disabled is not None and callable(push_item_flag):
            push_item_flag(item_disabled, True)
            if style_alpha is not None and callable(push_style_var):
                push_style_var(style_alpha, imgui.get_style().alpha * 0.5)
                return "push_item_flag+alpha"
            return "push_item_flag"
        return None

    def _end_disabled_ui(self, token: str | None) -> None:
        if token is None:
            return
        if token == "begin_disabled":
            end_disabled = getattr(imgui, "end_disabled", None)
            if callable(end_disabled):
                end_disabled()
            return
        if token == "push_item_flag+alpha":
            pop_style_var = getattr(imgui, "pop_style_var", None)
            if callable(pop_style_var):
                pop_style_var()
        pop_item_flag = getattr(imgui, "pop_item_flag", None)
        if callable(pop_item_flag):
            pop_item_flag()

    def _draw_ik_panel(self) -> None:
        # --- IK Target (world xyz) ---
        if not self._ik_header_init_open:
            cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
            imgui.set_next_item_open(True, cond)
            self._ik_header_init_open = True

        if imgui.collapsing_header("Inverse Kinematics", visible=True)[0]:
            _ret = imgui.input_float3(
                "target [m]",
                self.state.target_x, self.state.target_y, self.state.target_z,
                format="%.4f",
            )
            if isinstance(_ret, tuple) and len(_ret) == 2:
                changed, (x, y, z) = _ret
            else:
                changed, x, y, z = _ret
            if changed:
                self.state.set_target(float(x), float(y), float(z))
                self.service.send_current_target_meta(source="target")

            _dir_ret = imgui.input_float3(
                "target dir",
                self.state.target_vx, self.state.target_vy, self.state.target_vz,
                format="%.3f",
            )
            if isinstance(_dir_ret, tuple) and len(_dir_ret) == 2:
                changed_dir, (vx, vy, vz) = _dir_ret
            else:
                changed_dir, vx, vy, vz = _dir_ret
            if changed_dir:
                self.state.set_target_dir(float(vx), float(vy), float(vz))
                self.service.send_current_target_meta(source="target")

            if imgui.button("Solve IK"):
                self.service.start_ik_solve()
            imgui.same_line()
            if imgui.button("Tweak"):
                self.service.start_tweak()
            imgui.same_line()
            if imgui.button("Stop IK"):
                self.state.clear_ik_status()

            # Status line
            status = "idle"
            if self.state.ik_running:
                status = "running"
            if self.state.ik_converged:
                status = "converged"
            if self.state.ik_failed:
                status = "failed"
            imgui.text(f"IK status: {status} | err: {self.state.ik_err_m*1000:.2f} mm")

    def _draw_hardware_panel(self) -> None:
        if (not self._use_hardware) or (not self.service.has_client()):
            return
        if not self._hw_header_init_open:
            cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
            imgui.set_next_item_open(True, cond)
            self._hw_header_init_open = True
        if imgui.collapsing_header("Hardware", visible=True)[0]:
            state = self._host_state if self._host_state is not None else self.service.current_host_state()
            if state is None:
                imgui.text("Host: OFF")
                return
            imgui.text(f"Host: {'OK' if state.connected else 'OFF'}")
            imgui.text(f"tx_seq={state.tx_seq} rx_age={state.rx_age_s:.2f}s")
            current_device = str(state.device or "").strip()
            if current_device:
                imgui.text(f"Current Port: {current_device}")
                if not self._port_input:
                    self._port_input = current_device
            changed_port, new_port = imgui.input_text("Port", self._port_input, 256)
            if changed_port:
                self._port_input = str(new_port)
            if imgui.button("Search Ports"):
                self.service.request_ports()
            ports = list(state.ports)
            if ports:
                imgui.text("Detected Ports:")
                imgui.same_line()
                for idx, port in enumerate(ports):
                    if imgui.small_button(f"{port}##port_{idx}"):
                        self._port_input = str(port)
                    if (idx + 1) < len(ports):
                        imgui.same_line()
            if imgui.button("Apply Port"):
                self.service.set_device(self._port_input.strip())
            imgui.same_line()
            if imgui.button("Disconnect Port"):
                self.service.disconnect_device()
                self._port_input = ""
            reply_reason = str(state.reply_reason or "").strip()
            if reply_reason:
                if bool(state.reply_ok):
                    if reply_reason == "ports":
                        if not ports:
                            imgui.text("No serial ports found")
                    else:
                        imgui.text(f"Host: {reply_reason}")
                else:
                    imgui.text_colored(f"Host: {reply_reason}", 1.0, 0.35, 0.35)
            if imgui.button("Torque On"):
                self.service.torque_on()
            imgui.same_line()
            if imgui.button("Torque Off"):
                self.service.torque_off()

    def _draw_3dof_panel(self) -> None:
        if not self._ctrl_header_init_open:
            cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
            imgui.set_next_item_open(True, cond)
            self._ctrl_header_init_open = True
        if not imgui.collapsing_header("4-DOF Controls", visible=True)[0]:
            return

        link_state = self._host_state if self._host_state is not None else None
        sliders_locked = bool(self._use_hardware and ((not self.service.has_client()) or link_state is None or not bool(link_state.torque_enabled)))
        disable_token = self._begin_disabled_ui(sliders_locked)

        u_now = self.service.current_control_u()
        cfg = self.service.control_mapping()

        changed_linear, u_linear = imgui.slider_float(
            "linear [u]", float(u_now.u_linear),
            float(cfg.linear_u_min), float(cfg.linear_u_max),
            format="%.1f"
        )
        changed_rdeg, u_roll = imgui.slider_float(
            "roll [u]", float(u_now.u_roll),
            float(cfg.roll_u_min), float(cfg.roll_u_max),
            format="%.1f"
        )
        changed_s1, u_s1 = imgui.slider_float(
            "seg1 [u]", float(u_now.u_s1),
            float(cfg.seg_u_min), float(cfg.seg_u_max),
            format="%.1f"
        )
        changed_s2, u_s2 = imgui.slider_float(
            "seg2 [u]", float(u_now.u_s2),
            float(cfg.seg_u_min), float(cfg.seg_u_max),
            format="%.1f"
        )
        self._end_disabled_ui(disable_token)

        changed_any = bool((not sliders_locked) and (changed_linear or changed_rdeg or changed_s1 or changed_s2))
        if self.state.ik_running and changed_any:
            self.state.clear_ik_status()
        if changed_any:
            partial_u: dict[str, float] = {}
            if changed_linear:
                partial_u["linear"] = float(u_linear)
            if changed_rdeg:
                partial_u["roll"] = float(u_roll)
            if changed_s1:
                partial_u["s1"] = float(u_s1)
            if changed_s2:
                partial_u["s2"] = float(u_s2)
            self.service.apply_partial_control_u(partial_u)
        if sliders_locked:
            imgui.text("Sliders locked until Torque On")

        tip_xyz = link_state.actual_tip_xyz if link_state is not None else None
        if tip_xyz is None:
            imgui.text("Tip xyz [m]: unavailable")
        else:
            imgui.text(
                "Tip xyz [m]: (%.3f, %.3f, %.3f)"
                % (float(tip_xyz[0]), float(tip_xyz[1]), float(tip_xyz[2]))
            )

        if imgui.button("Open Gripper"):
            self.state.set_claw_closed(False)
            self.service.send_claw_command(closed=False)
        imgui.same_line()
        if imgui.button("Close Gripper"):
            self.state.set_claw_closed(True)
            self.service.send_claw_command(closed=True)
        if imgui.button("Reset"):
            self.state.clear_ik_status()
            self.state.reset_q()
            self.service.send_current_target(source="slider")
        _, paused = imgui.checkbox("Lock", self.state.paused)
        self.state.set_paused(bool(paused))

    def _draw_sag_panel(self) -> None:
        if not self._sag_header_init_open:
            cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
            imgui.set_next_item_open(True, cond)
            self._sag_header_init_open = True
        if imgui.collapsing_header("Sag Model", visible=True)[0]:
            changed, sag_path = imgui.input_text("sag model path", self._sag_model_path_draft, 512)
            if changed:
                self._sag_model_path_draft = str(sag_path)
            if imgui.button("Load Model"):
                try:
                    resolved_path, model = self.service.load_sag_model(self._sag_model_path_draft)
                    self._sag_model_path_draft = str(resolved_path)
                    self.service.send_current_target(source="target")
                    raw_type = str(model.get("model_type", "") or "").strip()
                    if raw_type:
                        model_type = raw_type
                    elif any(k in model for k in ("c1_family", "c1_params", "a1", "b1_coeffs", "c2_family", "c2_params", "a2", "b2_coeffs")):
                        model_type = "refined"
                    elif any(k in model for k in ("seg1_distribution", "seg1_amplitude", "seg2_distribution", "seg2_amplitude")):
                        model_type = "legacy"
                    else:
                        model_type = "unknown"
                    self._sag_status_text = f"loaded: {resolved_path} ({model_type})"
                    self._sag_status_ok = True
                except Exception as exc:
                    self._sag_status_text = f"load failed: {exc}"
                    self._sag_status_ok = False

            imgui.text(f"Applied model: {self.state.sag_model_path}")
            if self._sag_status_text:
                if self._sag_status_ok:
                    imgui.text(self._sag_status_text)
                else:
                    imgui.text_colored(self._sag_status_text, 1.0, 0.35, 0.35)


    def _draw_controls_window(self) -> None:
        if not self._ctrl_window_init:
            cond = getattr(imgui, "ONCE", getattr(imgui, "FIRST_USE_EVER", 1))
            io = imgui.get_io()
            imgui.set_next_window_position(0.0, 0.0, cond)
            imgui.set_next_window_size(float(io.display_size.x), float(io.display_size.y), cond)
            self._ctrl_window_init = True
        imgui.begin("###arm_control_window", True)
        self._draw_hardware_panel()
        if self._use_hardware and self.service.has_client():
            imgui.separator()
        self._draw_3dof_panel()
        imgui.separator()
        self._draw_ik_panel()
        imgui.separator()
        self._draw_sag_panel()
        imgui.end()
    def run(self) -> None:
        if not glfw.init():
            raise SystemExit("glfw.init() failed.")

        glfw.window_hint(glfw.RESIZABLE, True)
        win_w = 800
        win_h = 600
        monitor = glfw.get_primary_monitor()
        if monitor is not None:
            mode = glfw.get_video_mode(monitor)
            if mode is not None:
                width = int(getattr(mode.size, "width", 0) or 0)
                height = int(getattr(mode.size, "height", 0) or 0)
                if width > 0 and height > 0:
                    win_w = max(640, width // 3)
                    win_h = max(600, height // 2)
        window = glfw.create_window(win_w, win_h, "Arm Control", None, None)
        if not window:
            glfw.terminate()
            raise SystemExit("Failed to create GLFW window.")

        glfw.make_context_current(window)

        imgui.create_context()
        impl = GlfwRenderer(window)


        try:
            while not glfw.window_should_close(window) and not self._stop:
                self._host_state = self.service.refresh_host_state()
                glfw.poll_events()
                impl.process_inputs()

                imgui.new_frame()
                self._draw_controls_window()
                imgui.render()

                impl.render(imgui.get_draw_data())
                glfw.swap_buffers(window)
                time.sleep(0.01)
        finally:
            impl.shutdown()
            glfw.terminate()

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(__file__), "config.ini"),
        help="path to ini config file",
    )
    args = ap.parse_args()

    bundle, ik_context = ik_pipeline.load_solver_context(args.config)
    link = ControlClient(str(bundle.sim_config.host_ctrl_port), cfg=bundle.mapping_config)
    initial_sag = _resolve_initial_sag_model()
    state = PanelState(
        sag_model_path=DEFAULT_SAG_MODEL_PATH,
        raw_sag_model=(dict(initial_sag) if dict(initial_sag) else None),
    )
    service = ControlService(
        state,
        client=link,
        mapping_cfg=bundle.mapping_config,
        ik_cfg=bundle.ik_config,
        ik_context=ik_context,
        config_path=args.config,
    )
    gui = ControlPanel(
        state,
        service,
        use_hardware=bool(bundle.sim_config.use_hardware),
    )
    try:
        service.refresh_host_state()
        service.send_current_target_meta(source="target")
        gui.run()
    finally:
        service.close()


if __name__ == "__main__":
    main()
