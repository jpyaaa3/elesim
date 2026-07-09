#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import csv
import os
import signal
import threading
import time
from dataclasses import replace
from pathlib import Path
from typing import Any, Dict, Optional, Set

import numpy as np
import zmq

from engine.core.config_loader import HardwareConfig, PerceptionConfig, PickConfig, load_app_config_from_ini
from engine.behaviors.gaze.stabilizer import (
    GazeStabilizerConfig,
    gaze_config_to_dict,
    patch_gaze_config,
)
from engine.behaviors.pick.state import HostState
from engine.robot.go2.hardware import UnitreeRos2Bridge, create_go2_bridge_if_enabled
from engine.robot.go2.hardware.odom_parser import OdomSample
from engine.robot.go2.hardware.sport_api import normalize_go2_sport_pose, sport_pose_api_id
from engine.observability.pick_timing import enabled as pick_profile_enabled
from engine.robot.arm.iklib.solver import load_solver_context
from engine.robot.arm.dynamixel import load_hardware, tick_to_deg_0_360
from engine.core.trajectory import QuinticTimingConfig, QuinticTrajectoryRunner
from engine.vision.visual_servoing.ready_pose import compute_ready_pose_target
import engine.core.protocol as proto
from engine.vision.perception_bridge.hand_eye import camera_axes_world, camera_point_to_world, load_hand_eye_transform
from engine.vision.perception.capture import (
    PerceptionCapture,
    PerceptionSnapshot,
    default_perception_capture_dir,
)
from engine.vision.perception.preview_stream import PreviewFramePublisher
from engine.vision.sim_camera.pose import camera_point_to_world_from_axes

from serial.tools import list_ports as serial_list_ports


def _read_cmdline(pid: int) -> list[str]:
    try:
        raw = open(f"/proc/{int(pid)}/cmdline", "rb").read()
    except Exception:
        return []
    return [part.decode("utf-8", errors="replace") for part in raw.split(b"\0") if part]


def _find_host_processes() -> list[int]:
    current = os.getpid()
    found: list[int] = []
    proc_root = "/proc"
    try:
        entries = os.listdir(proc_root)
    except Exception:
        return found
    for entry in entries:
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == current:
            continue
        cmd = _read_cmdline(pid)
        if not cmd:
            continue
        if any(os.path.basename(arg) == "host.py" for arg in cmd):
            found.append(pid)
    return sorted(found)


def _terminate_host_processes(*, timeout_s: float = 2.0, force: bool = True) -> None:
    pids = _find_host_processes()
    if not pids:
        return
    print(f"[host] terminating existing host.py process(es): {', '.join(str(p) for p in pids)}")
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.time() + max(0.1, float(timeout_s))
    while time.time() < deadline:
        alive = [pid for pid in pids if os.path.isdir(f"/proc/{pid}")]
        if not alive:
            return
        time.sleep(0.05)
    if force:
        for pid in pids:
            if os.path.isdir(f"/proc/{pid}"):
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


def _local_client_endpoint(bind_addr: str) -> str:
    """Return a loopback endpoint for a TCP bind address."""
    addr = str(bind_addr).strip()
    if not addr.startswith("tcp://"):
        return addr
    rest = addr[len("tcp://") :]
    if ":" not in rest:
        return addr
    _host, port = rest.rsplit(":", 1)
    return f"tcp://127.0.0.1:{port}"


class _DirectEmbeddedControlClient:
    """In-process client used by embedded robot control to avoid loopback ZMQ latency."""

    def __init__(self, host: "ControlHost", cfg: proto.SimMappingConfig) -> None:
        self._host = host
        self.cfg = cfg
        self.host_native_control = True
        self.tx_seq = 0
        self.is_connected = True
        self.last_reply_ok = True
        self.last_reply_reason = ""
        self.last_object_world_xyz: Optional[tuple[float, float, float]] = None
        self._t_last_tx = 0.0
        self._send_period = 0.0

    def close(self) -> None:
        self.is_connected = False

    def rx_age_s(self) -> float:
        return 0.0

    def get_state(self) -> HostState:
        state = self._host._host_state_snapshot(
            tx_seq=int(self.tx_seq),
            reply_ok=bool(self.last_reply_ok),
            reply_reason=str(self.last_reply_reason),
        )
        self.last_object_world_xyz = self._host.last_perceived_object_world_xyz
        return state

    def refresh_state(self) -> HostState:
        return self.get_state()

    def has_hardware(self) -> bool:
        return bool(self._host._has_hw())

    def refresh_lji_state(self) -> HostState:
        if self._host._has_hw():
            try:
                self._host._read_hw_state(read_currents=False)
            except Exception as exc:
                self.last_reply_ok = False
                self.last_reply_reason = f"lji_hw_read_failed:{exc}"
        return self.get_state()

    def apply_lji_q_direct(
        self,
        q: proto.SimQ,
        *,
        sag_model: Optional[dict[str, Any]] = None,
        target_xyz: Optional[tuple[float, float, float]] = None,
        target_dir: Optional[tuple[float, float, float]] = None,
        claw_closed: Optional[bool] = None,
        source: str = "lji_step",
    ) -> HostState:
        self.tx_seq += 1
        host = self._host
        if target_xyz is not None:
            host.last_ik_target_xyz = tuple(float(v) for v in target_xyz)
        if target_dir is not None:
            host.last_ik_target_dir = tuple(float(v) for v in target_dir)
        if isinstance(sag_model, dict):
            host.last_sag_model = dict(sag_model)
        if claw_closed is not None:
            host.last_claw_closed = bool(claw_closed)

        host._clear_arm_servo_target()
        host._pending_target_u = None
        host._pending_target_axes = set()
        host._target_u_state = proto.sim_q_to_control_u(q, host.cfg)
        host._pending_target_q = q
        host._pending_target_seq = int(self.tx_seq)
        host._last_target_apply_error = ""

        if host._safety_fault:
            host._pending_target_q = None
            host._cancel_trajectory()
            ok = False
            reason = str(host._safety_fault)
        else:
            host._schedule_target_motion(q, source=source)
            if host._has_hw() and host._use_trajectory_for_source(source):
                ok = True
                reason = "host_native_lji_trajectory"
            elif host._has_hw():
                ok, complete = host._apply_sim_q_target(q)
                if ok:
                    host._target_u_state = proto.sim_q_to_control_u(q, host.cfg)
                    if bool(complete):
                        host._pending_target_q = None
                reason = host._last_target_apply_error or "host_native_lji_direct"
            else:
                host.last_q = q
                host.last_u = proto.sim_q_to_control_u(q, host.cfg)
                host.last_state_ts = time.time()
                host._pending_target_q = None
                host._cancel_trajectory()
                ok = True
                reason = "host_native_lji_sim"

        self.last_reply_ok = bool(ok)
        self.last_reply_reason = str(reason)
        self.last_object_world_xyz = host.last_perceived_object_world_xyz
        host._state_broadcast_requested.set()
        return self.refresh_lji_state()

    def q_to_control_u(
        self,
        *,
        linear_m: float,
        roll_rad: float,
        theta1_rad: float,
        theta2_rad: float,
    ) -> proto.ControlU:
        return proto.sim_q_to_control_u(
            proto.SimQ(
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
    ) -> proto.SimQ:
        return proto.control_u_to_sim_q(
            proto.ControlU(
                u_linear=float(u_linear),
                u_roll=float(u_roll),
                u_s1=float(u_s1),
                u_s2=float(u_s2),
            ),
            self.cfg,
        )

    def send_partial_control_u(self, partial_u: dict[str, float], *, source: str = "gaze") -> None:
        now = time.time()
        self.tx_seq += 1
        ok, reason = self._host._submit_direct_partial_control_u(
            partial_u,
            source=str(source),
            seq=int(self.tx_seq),
            client_ts_s=float(now),
        )
        self.last_reply_ok = bool(ok)
        self.last_reply_reason = str(reason)

    def _submit_target_payload(
        self,
        payload: dict[str, Any],
        *,
        force: bool = False,
    ) -> None:
        now = time.time()
        if (not bool(force)) and self._send_period > 0.0 and (now - self._t_last_tx) < self._send_period:
            return
        self._t_last_tx = now
        self.tx_seq += 1
        msg = {"t": "target", "ts": float(now), "seq": int(self.tx_seq)}
        msg.update(dict(payload))
        ok, reason = self._host._handle_direct_target_msg(msg)
        self.last_reply_ok = bool(ok)
        self.last_reply_reason = str(reason)
        self.last_object_world_xyz = self._host.last_perceived_object_world_xyz

    def maybe_send_target_q(self, q: proto.SimQ, *, source: str = "sim", force: bool = False) -> None:
        self._maybe_send_target_q(
            q,
            source=source,
            target_xyz=None,
            target_dir=None,
            sag_model=None,
            claw_closed=None,
            force=force,
        )

    def _maybe_send_target_q(
        self,
        q: proto.SimQ,
        *,
        source: str,
        target_xyz: Optional[tuple[float, float, float]],
        target_dir: Optional[tuple[float, float, float]],
        sag_model: Optional[dict[str, Any]],
        claw_closed: Optional[bool],
        force: bool = False,
    ) -> None:
        payload: dict[str, Any] = {
            "source": str(source),
            "q": {
                "linear_m": float(q.linear_m),
                "roll_rad": float(q.roll_rad),
                "theta1_rad": float(q.theta1_rad),
                "theta2_rad": float(q.theta2_rad),
            },
        }
        if target_xyz is not None:
            payload["target"] = [float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])]
        if target_dir is not None:
            payload["target_dir"] = [float(target_dir[0]), float(target_dir[1]), float(target_dir[2])]
        if sag_model is not None:
            payload["sag_model"] = dict(sag_model)
        if claw_closed is not None:
            payload["claw_closed"] = bool(claw_closed)
        self._submit_target_payload(payload, force=force)

    def send_target_q(self, q: proto.SimQ, *, source: str = "ui", force: bool = False) -> None:
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
        sag_model: Optional[dict[str, Any]] = None,
        claw_closed: Optional[bool] = None,
        force: bool = False,
    ) -> None:
        self._maybe_send_target_q(
            proto.SimQ(
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

    def send_target_meta(
        self,
        *,
        target_xyz: tuple[float, float, float],
        target_dir: tuple[float, float, float],
        source: str = "target",
    ) -> None:
        self._submit_target_payload(
            {
                "source": str(source),
                "target": [float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])],
                "target_dir": [float(target_dir[0]), float(target_dir[1]), float(target_dir[2])],
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
        self._submit_target_payload(
            {
                "source": str(source),
                "ready_pose_dir": [float(target_dir[0]), float(target_dir[1]), float(target_dir[2])],
                "ready_pose_standoff_m": float(standoff_m),
            },
            force=True,
        )

    def send_sag_model_meta(self, sag_model: dict[str, Any], *, source: str = "target") -> None:
        self._submit_target_payload(
            {"source": str(source), "sag_model": dict(sag_model)},
            force=True,
        )

    def send_claw_command(self, *, claw_closed: bool, source: str = "target") -> None:
        self._submit_target_payload(
            {"source": str(source), "claw_closed": bool(claw_closed)},
            force=True,
        )

    def send_go2_velocity(self, *, vx: float, vy: float, wz: float, source: str = "target") -> None:
        self._submit_target_payload(
            {"source": str(source), "go2_vel": [float(vx), float(vy), float(wz)]},
            force=True,
        )

    def send_go2_sport_pose(self, *, pose: str, source: str = "target") -> None:
        self._submit_target_payload(
            {"source": str(source), "go2_sport_pose": str(pose).strip().lower()},
            force=True,
        )

    def send_go2_obstacles_avoid(self, *, enabled: bool, source: str = "target") -> None:
        self._submit_target_payload(
            {"source": str(source), "go2_obstacles_avoid_enable": bool(enabled)},
            force=True,
        )

    def send_sim_target_xyz(self, *, xyz: tuple[float, float, float], source: str = "target") -> None:
        self._submit_target_payload(
            {"source": str(source), "sim_target": [float(xyz[0]), float(xyz[1]), float(xyz[2])]},
            force=True,
        )

    def send_debug_markers(self, markers: list[dict[str, Any]], *, source: str = "target") -> None:
        self._submit_target_payload(
            {"source": str(source), "debug_markers": [dict(marker) for marker in markers]},
            force=True,
        )

    def send_perception_start(self, *, config: Optional[Any] = None) -> None:
        del config
        try:
            ok = self._host.start_perception_worker(config=self._host._remote_perception_config())
            reason = "perception_start" if ok else "perception_start_failed"
        except Exception as exc:
            ok = False
            reason = f"perception_start_failed:{exc}"
        self.last_reply_ok = bool(ok)
        self.last_reply_reason = str(reason)
        self._host._state_broadcast_requested.set()

    def send_perception_stop(self) -> None:
        ok = self._host.stop_perception_worker()
        self.last_reply_ok = bool(ok)
        self.last_reply_reason = "perception_stop" if ok else "perception_stop_pending"
        self._host._state_broadcast_requested.set()

    def send_perception_refresh(self) -> None:
        ok = self._host.refresh_perception_worker()
        self.last_reply_ok = bool(ok)
        self.last_reply_reason = "perception_refresh" if ok else "perception_not_running"
        self._host._state_broadcast_requested.set()


class ControlHost:
    """ROUTER-side host that receives controller requests and drives hardware."""

    def __init__(
        self,
        *,
        bind_addr: str,
        sim_pub_addr: str,
        sim_feedback_addr: str,
        hw: Any,
        direction_by_id: Dict[int, int],
        device: str,
        hardware_cfg: Optional[HardwareConfig],
        config_path: str = "",
        ik_config: Optional[Any] = None,
        ik_context: Optional[dict[str, Any]] = None,
        hand_eye_transform: Optional[Any] = None,
        hand_eye_parent_frame: str = "node9",
        pick_config: Optional[PickConfig] = None,
        perception_config: Optional[PerceptionConfig] = None,
        gaze_config: Optional[Any] = None,
        ownership_enable: bool = False,
        show_all_ports: bool = False,
        cfg: proto.SimMappingConfig = proto.SimMappingConfig(),
        trajectory_cfg: Optional[QuinticTimingConfig] = None,
        trajectory_lji_cfg: Optional[QuinticTimingConfig] = None,
        traj_lji_enable: bool = True,
        state_hz: float = 10.0,
        hw_read_hz: float = 20.0,
        hw_cmd_hz: float = 30.0,
        go2_bridge: Optional[UnitreeRos2Bridge] = None,
    ) -> None:
        if zmq is None:
            raise SystemExit("pyzmq is required. Install: pip install pyzmq")
        self.cfg = cfg
        self.hw = hw
        self.direction_by_id = direction_by_id
        self.device = str(device)
        self.hardware_cfg = hardware_cfg
        self.config_path = str(config_path)
        self.ik_config = ik_config
        self.ik_context = dict(ik_context or {})
        self.hand_eye_transform = None if hand_eye_transform is None else np.asarray(hand_eye_transform, dtype=float).reshape(4, 4)
        self.hand_eye_parent_frame = str(hand_eye_parent_frame)
        self.pick_config = pick_config or PickConfig()
        self.perception_config = perception_config or PerceptionConfig()
        self.gaze_config = gaze_config or GazeStabilizerConfig()
        self.ownership_enable = bool(ownership_enable)
        self.show_all_ports = bool(show_all_ports)
        self._go2_bridge = go2_bridge
        self._embedded_ctrl_endpoint = _local_client_endpoint(bind_addr)
        self._embedded_control_client: Optional[Any] = None
        self._embedded_control_service: Optional[Any] = None

        self.ctx = zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.ROUTER)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.bind(bind_addr)
        self.sim_pub = self.ctx.socket(zmq.PUB)
        self.sim_pub.setsockopt(zmq.LINGER, 0)
        self.sim_pub.bind(str(sim_pub_addr))
        self.sim_feedback = self.ctx.socket(zmq.PULL)
        self.sim_feedback.setsockopt(zmq.LINGER, 0)
        self.sim_feedback.bind(str(sim_feedback_addr))

        self.poller = zmq.Poller()
        self.poller.register(self.sock, zmq.POLLIN)
        self.poller.register(self.sim_feedback, zmq.POLLIN)

        self.clients: Set[bytes] = set()
        self.last_u: Optional[proto.ControlU] = None
        self.last_q: Optional[proto.SimQ] = None
        self.last_state_ts: float = 0.0
        self._last_target_log_t: float = 0.0
        self._last_target_log_key: str = ""
        self.last_sim_u: Optional[proto.ControlU] = None
        self.last_sim_q: Optional[proto.SimQ] = None
        self.last_sim_state_ts: float = 0.0
        self.torque_enabled: bool = False
        self.last_ik_target_xyz: Optional[tuple[float, float, float]] = None
        self.last_ik_target_dir: Optional[tuple[float, float, float]] = None
        self.last_ready_pose_dir: tuple[float, float, float] = (1.0, 0.0, 0.0)
        self.last_ready_pose_standoff_m: float = float(self.pick_config.ready_pose_standoff_m)
        self.last_actual_tip_xyz: Optional[tuple[float, float, float]] = None
        self.last_actual_tip_dir: Optional[tuple[float, float, float]] = None
        self.last_perceived_object_label: str = ""
        self.last_perceived_object_confidence: float = 0.0
        self.last_perceived_object_camera_xyz: Optional[tuple[float, float, float]] = None
        self.last_perceived_object_world_xyz: Optional[tuple[float, float, float]] = None
        self.last_perceived_center_uv: Optional[tuple[float, float]] = None
        self.last_perceived_scale: Optional[float] = None
        self.last_perceived_timestamp_s: float = 0.0
        self.perception_hz: float = 0.0
        self._perception_rate_last_t: float = 0.0
        self._perception_rate_last_frame_idx: int = -1
        self.perception_running: bool = False
        self.perception_failed: bool = False
        self.perception_status: str = "stopped"
        self.perception_source: str = "host"
        self.perception_last_capture_path: str = ""
        self.perception_last_record_path: str = ""
        self.perception_record_with_overlay: bool = False
        self._perception_capture: Optional[PerceptionCapture] = None
        self._perception_lock = threading.RLock()
        self._perception_log_interval_s = 1.0
        self._last_perception_log_t = 0.0
        self._last_perception_log_key = ""
        self._preview_publisher: Optional[PreviewFramePublisher] = None
        preview_bind = str(getattr(self.perception_config, "preview_bind", "")).strip()
        if preview_bind:
            self._preview_publisher = PreviewFramePublisher(
                preview_bind,
                jpeg_quality=int(getattr(self.perception_config, "preview_jpeg_quality", 75)),
            )
            self._preview_publisher.start()
        self.last_sag_model: dict[str, Any] = {}
        self.last_claw_closed: bool = False
        self.last_go2_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_go2_vel_ts: float = 0.0
        self._go2_vel_deadman_s: float = 0.35
        self.last_go2_base_rpy: Optional[tuple[float, float, float]] = None
        self.last_go2_base_pos: Optional[tuple[float, float, float]] = None
        self.last_go2_base_lin_vel_body: Optional[tuple[float, float, float]] = None
        self.last_go2_base_ang_vel: Optional[tuple[float, float, float]] = None
        self.last_go2_leg_q: Optional[tuple[float, ...]] = None
        self.last_go2_leg_dq: Optional[tuple[float, ...]] = None
        self.last_go2_leg_torque_nm: Optional[tuple[float, ...]] = None
        self.last_go2_base_timestamp_s: float = 0.0
        self.last_go2_sport_pose: str = ""
        self.last_go2_sport_pose_seq: int = 0
        self._go2_motion_armed: bool = False
        self.last_go2_obstacles_avoid_enabled: bool = False
        self.last_go2_obstacles_avoid_seq: int = 0
        self._sim_camera_origin: Optional[tuple[float, float, float]] = None
        self._sim_camera_look: Optional[tuple[float, float, float]] = None
        self._sim_camera_right: Optional[tuple[float, float, float]] = None
        self._sim_camera_ts: float = 0.0
        self.last_sim_target_xyz: Optional[tuple[float, float, float]] = None
        self._sim_reset_seq: int = 0
        self.last_sim_time_s: float = 0.0
        self.last_sim_wall_elapsed_s: float = 0.0
        self.last_sim_realtime_factor: float = 0.0
        self.last_sim_step_count: int = 0
        self._last_hw_pos_by_id: Dict[int, int] = {}
        self._last_claw_current: int = 0
        self._claw_close_stalled: bool = False

        self._state_period = 1.0 / max(0.1, float(state_hz))
        self._read_period = 1.0 / max(0.1, float(hw_read_hz))
        self._cmd_period = 1.0 / max(0.1, float(hw_cmd_hz))
        current_read_hz = float(getattr(self.hardware_cfg, "current_read_hz", hw_read_hz) or hw_read_hz)
        self._current_read_period = 1.0 / max(0.1, current_read_hz)
        self._t_read = 0.0
        self._t_current_read = 0.0
        self._t_state = 0.0
        self._t_cmd = 0.0
        self._state_broadcast_requested = threading.Event()
        self._event_state_min_period = 1.0 / 30.0
        self._t_event_state = 0.0

        self._pending_target_q: Optional[proto.SimQ] = None
        self._pending_target_u: Optional[proto.ControlU] = None
        self._pending_target_axes: Set[str] = set()
        self._pending_target_seq: int = -1
        self._target_u_state: Optional[proto.ControlU] = None

        self._ids = getattr(hw, "ids", [])
        self._hw_lock = threading.RLock()
        self._stop_event = threading.Event()
        self._arm_servo_thread_enable = bool(
            getattr(hardware_cfg, "arm_servo_thread_enable", False) if hardware_cfg is not None else False
        )
        self._arm_servo_period = 1.0 / max(
            1.0,
            float(getattr(hardware_cfg, "arm_servo_thread_hz", 120.0) if hardware_cfg is not None else 120.0),
        )
        self._arm_servo_cond = threading.Condition()
        self._arm_servo_stop = threading.Event()
        self._arm_servo_thread: Optional[threading.Thread] = None
        self._arm_servo_target_u: Optional[proto.ControlU] = None
        self._arm_servo_target_axes: Set[str] = set()
        self._arm_servo_target_seq: int = -1
        self._arm_servo_target_source: str = ""
        self._arm_servo_applied_seq: int = -1
        self._arm_servo_submit_count: int = 0
        self._arm_servo_apply_count: int = 0
        self._arm_servo_last_error: str = ""
        self._arm_servo_start_skip_logged: bool = False
        self._arm_servo_target_meta: dict[str, Any] = {}
        self._arm_latency_log_enable = (
            os.environ.get("ELESIM_ARM_LATENCY", "").strip().lower() in ("1", "true", "yes", "on")
            or bool(getattr(hardware_cfg, "arm_latency_log_enable", False) if hardware_cfg is not None else False)
        )
        self._arm_latency_log_interval_s = max(
            0.1,
            float(
                getattr(hardware_cfg, "arm_latency_log_interval_s", 1.0)
                if hardware_cfg is not None
                else 1.0
            ),
        )
        self._arm_latency_last_log_t: float = 0.0
        self._arm_latency_counts: dict[str, int] = {}
        self._arm_latency_sums: dict[str, float] = {}
        self._arm_latency_max: dict[str, float] = {}
        self._arm_latency_last_follow: dict[str, Any] = {}
        self._arm_latency_csv_path: str = ""
        self._arm_latency_csv_file: Optional[Any] = None
        self._arm_latency_csv_writer: Optional[csv.DictWriter] = None
        self._init_arm_latency_csv()
        self._claw_open_deg = 340.0
        self._claw_close_deg = 230.0
        self._claw_stop_current = -200
        self._current_yellow_ma = int(getattr(hardware_cfg, "current_yellow_ma", 1800) if hardware_cfg is not None else 1800)
        self._current_limit_ma = int(getattr(hardware_cfg, "current_limit_ma", 2500) if hardware_cfg is not None else 2500)
        self._last_motor_current_by_id: Dict[int, int] = {}
        self._safety_fault: str = ""
        self._yellow_zone_ids: Set[int] = set()
        self._red_torque_off_ids: Set[int] = set()
        self._debug_markers_by_name: Dict[str, dict[str, Any]] = {}
        self._last_target_apply_error: str = ""
        self._trajectory = QuinticTrajectoryRunner(trajectory_cfg or QuinticTimingConfig())
        self._trajectory_lji = QuinticTrajectoryRunner(
            trajectory_lji_cfg or trajectory_cfg or QuinticTimingConfig()
        )
        self._traj_lji_enable = bool(traj_lji_enable)
        self._traj_step_log_every = 10
        self._traj_step_count = 0
        self._traj_last_apply_ok: Optional[bool] = None
        self._traj_profile_start_s: Optional[float] = None
        self._start_arm_servo_thread()
        if not self._has_hw():
            self._set_virtual_neutral_state()
        if bool(getattr(self.perception_config, "autostart", False)):
            self.start_perception_worker()

    def _set_virtual_neutral_state(self) -> None:
        neutral_q = proto.default_start_sim_q(self.cfg)
        self.last_q = neutral_q
        self.last_u = proto.DEFAULT_START_CONTROL_U
        self._target_u_state = self.last_u
        self.last_state_ts = time.time()
        self.last_sim_u = None
        self.last_sim_q = None
        self.last_sim_state_ts = 0.0
        self._debug_markers_by_name: Dict[str, dict[str, Any]] = {}
        self._last_target_apply_error = ""
        self._trajectory.cancel()

    def _clear_go2_vel(self) -> None:
        self.last_go2_vel = (0.0, 0.0, 0.0)
        self._last_go2_vel_ts = 0.0

    def _effective_go2_vel(self, now: Optional[float] = None) -> tuple[float, float, float]:
        if self._last_go2_vel_ts <= 0.0:
            return (0.0, 0.0, 0.0)
        now_s = proto.now_s() if now is None else float(now)
        if (now_s - self._last_go2_vel_ts) <= self._go2_vel_deadman_s:
            return self.last_go2_vel
        if any(abs(v) > 1e-9 for v in self.last_go2_vel):
            print("[host] go2 velocity deadman: stopping stale command")
        self._clear_go2_vel()
        if self._go2_bridge is not None:
            self._go2_bridge.set_velocity(0.0, 0.0, 0.0)
        return (0.0, 0.0, 0.0)

    @staticmethod
    def _go2_vel_nonzero(vel: tuple[float, float, float]) -> bool:
        return any(abs(float(v)) > 1e-6 for v in vel)

    def _go2_requires_user_stand(self) -> bool:
        return self._go2_bridge is not None

    def _go2_motion_lock_reason(self, vel: tuple[float, float, float]) -> str:
        if (
            self._go2_requires_user_stand()
            and self._go2_vel_nonzero(vel)
            and not bool(self._go2_motion_armed)
        ):
            return "go2 motion locked: press Stand first"
        return ""

    def _authorize_go2_sport_pose(self, pose: str) -> tuple[bool, str]:
        normalized = normalize_go2_sport_pose(pose)
        if normalized in ("stand_up", "stand_down"):
            return True, ""
        if self._go2_requires_user_stand() and not bool(self._go2_motion_armed):
            return False, "go2 posture locked: press Stand first"
        return True, ""

    def _note_go2_sport_pose(self, pose: str) -> None:
        normalized = normalize_go2_sport_pose(pose)
        if normalized == "stand_up":
            self._go2_motion_armed = True
        elif normalized == "stand_down":
            self._go2_motion_armed = False

    def _reset_simulation_state(self) -> None:
        """Virtual/sim mode: reset host-side command state and bump sim reset counter."""
        self._sim_reset_seq += 1
        self._set_virtual_neutral_state()
        self._pending_target_q = None
        self._pending_target_u = None
        self._pending_target_axes = set()
        self._pending_target_seq = -1
        self._clear_go2_vel()
        self.last_go2_base_rpy = None
        self.last_go2_base_pos = None
        self.last_go2_base_lin_vel_body = None
        self.last_go2_base_ang_vel = None
        self.last_go2_leg_q = None
        self.last_go2_leg_dq = None
        self.last_go2_leg_torque_nm = None
        self.last_go2_base_timestamp_s = 0.0
        self.last_go2_sport_pose = ""
        self.last_go2_obstacles_avoid_enabled = False
        self.last_claw_closed = False
        self.last_perceived_object_label = ""
        self.last_perceived_object_confidence = 0.0
        self.last_perceived_object_camera_xyz = None
        self.last_perceived_center_uv = None
        self.last_perceived_scale = None
        self.last_perceived_timestamp_s = 0.0
        self.perception_hz = 0.0
        self._perception_rate_last_t = 0.0
        self._perception_rate_last_frame_idx = -1
        self.last_actual_tip_xyz = None
        self.last_actual_tip_dir = None
        self.last_sim_u = None
        self.last_sim_q = None
        self.last_sim_state_ts = 0.0
        self.last_sim_time_s = 0.0
        self.last_sim_wall_elapsed_s = 0.0
        self.last_sim_realtime_factor = 0.0
        self.last_sim_step_count = 0
        print(f"[host] sim reset | seq={int(self._sim_reset_seq)}")

    def _cancel_trajectory(self) -> None:
        self._trajectory.cancel()
        self._trajectory_lji.cancel()
        self._traj_step_count = 0
        self._traj_last_apply_ok = None
        self._traj_profile_start_s = None

    def _use_trajectory_for_source(self, source: str) -> bool:
        src = str(source).strip().lower()
        if src in {"lji", "lji_step"}:
            return bool(self._traj_lji_enable and self._trajectory_lji.cfg.enable)
        if not bool(self._trajectory.cfg.enable):
            return False
        # Visual servo / aim: immediate partial u; IK: long quintic.
        return src == "ik"

    @staticmethod
    def _joint_delta_max(q_start: proto.SimQ, q_goal: proto.SimQ) -> float:
        return float(
            max(
                abs(float(q_goal.linear_m) - float(q_start.linear_m)),
                abs(float(q_goal.roll_rad) - float(q_start.roll_rad)),
                abs(float(q_goal.theta1_rad) - float(q_start.theta1_rad)),
                abs(float(q_goal.theta2_rad) - float(q_start.theta2_rad)),
            )
        )

    def _schedule_target_motion(self, q: proto.SimQ, *, source: str) -> None:
        if not self._use_trajectory_for_source(source):
            self._cancel_trajectory()
            return
        if self._has_hw() and self.last_q is None:
            try:
                self._read_hw_state()
            except Exception as exc:
                print(f"[host] hw state read before trajectory failed: {exc}")
        self._start_trajectory(q, source=source)

    def _active_trajectory_runner(self, source: str) -> QuinticTrajectoryRunner:
        if str(source).strip().lower() in {"lji", "lji_step"}:
            return self._trajectory_lji
        return self._trajectory

    def _start_trajectory(self, q_goal: proto.SimQ, *, source: str = "ik") -> None:
        src = str(source).strip().lower()
        runner = self._active_trajectory_runner(source)
        now = time.time()
        if runner.active:
            q_start = runner.step(now_s=now).q_cmd
        else:
            q_start = self.last_q
        if q_start is None:
            q_start = q_goal
        skip_delta = 0.0015 if src in {"lji", "lji_step"} else 0.015
        if self._joint_delta_max(q_start, q_goal) < float(skip_delta):
            self._cancel_trajectory()
            print(
                "[host] trajectory skipped | small delta source=%s q_goal=(%.4f, %.4f, %.4f, %.4f)"
                % (
                    str(source),
                    float(q_goal.linear_m),
                    float(q_goal.roll_rad),
                    float(q_goal.theta1_rad),
                    float(q_goal.theta2_rad),
                )
            )
            if pick_profile_enabled():
                print("[Profile] traj skip | source=%s dt=0.0ms" % str(source))
            return
        runner.start(q_start=q_start, q_goal=q_goal, now_s=now)
        if src not in {"lji", "lji_step"}:
            self._trajectory_lji.cancel()
        else:
            self._trajectory.cancel()
        self._traj_step_count = 0
        self._traj_last_apply_ok = None
        if pick_profile_enabled():
            self._traj_profile_start_s = time.time()
            print("[Profile] traj start | source=%s" % str(source))
        print(
            "[host] trajectory start | q_start=(%.4f, %.4f, %.4f, %.4f) -> q_goal=(%.4f, %.4f, %.4f, %.4f)"
            % (
                float(q_start.linear_m),
                float(q_start.roll_rad),
                float(q_start.theta1_rad),
                float(q_start.theta2_rad),
                float(q_goal.linear_m),
                float(q_goal.roll_rad),
                float(q_goal.theta1_rad),
                float(q_goal.theta2_rad),
            )
        )

    def _has_hw(self) -> bool:
        return self.hw is not None

    def _list_ports(self) -> list[str]:
        if serial_list_ports is None:
            return []
        try:
            ports = [str(p.device) for p in serial_list_ports.comports()]
            if self.show_all_ports:
                return ports
            filtered: list[str] = []
            for dev in ports:
                base = os.path.basename(str(dev))
                if base.startswith("ttyUSB") or base.startswith("ttyACM"):
                    filtered.append(str(dev))
            return filtered
        except Exception:
            return []

    def set_device(self, device: str) -> None:
        new_device = str(device).strip()
        if not new_device:
            raise ValueError("empty device")
        start_servo = False
        old_hw_to_close = None
        with self._hw_lock:
            if new_device == str(self.device).strip() and self.hw is not None:
                start_servo = True
            else:
                start_servo = False
            if start_servo:
                pass
            else:
                self._clear_arm_servo_target()
                old_hw = self.hw
                old_direction = dict(self.direction_by_id)
                old_ids = list(self._ids)
                old_device = str(self.device)
                self._pending_target_q = None
                self._pending_target_u = None
                self._pending_target_axes = set()
                self._pending_target_seq = -1
                self._cancel_trajectory()
                self._target_u_state = None
                self.last_u = None
                self.last_q = None
                self.last_state_ts = 0.0
                self.last_sim_u = None
                self.last_sim_q = None
                self.last_sim_state_ts = 0.0
                self.torque_enabled = False
                self.last_ik_target_xyz = None
                self.last_ik_target_dir = None
                self.last_actual_tip_xyz = None
                self.last_actual_tip_dir = None
                self.last_perceived_object_label = ""
                self.last_perceived_object_confidence = 0.0
                self.last_perceived_object_camera_xyz = None
                self.last_perceived_center_uv = None
                self.last_perceived_scale = None
                self.last_perceived_timestamp_s = 0.0
                self.perception_hz = 0.0
                self._perception_rate_last_t = 0.0
                self._perception_rate_last_frame_idx = -1
                self.last_sag_model = {}
                self.last_claw_closed = False
                self._clear_go2_vel()
                self._last_hw_pos_by_id = {}
                self._last_claw_current = 0
                self._claw_close_stalled = False
                self._last_motor_current_by_id = {}
                self._safety_fault = ""
                self._yellow_zone_ids = set()
                self._red_torque_off_ids = set()
                if old_hw is not None:
                    old_hw_to_close = old_hw
                    self.hw = None
                    self.direction_by_id = {}
                    self._ids = []
                    self.device = ""
        if start_servo:
            self._start_arm_servo_thread()
            return
        if old_hw_to_close is not None:
            self._stop_arm_servo_thread()
            try:
                old_hw_to_close.close()
            except Exception:
                pass
        with self._hw_lock:
            try:
                new_hw, new_direction = load_hardware(new_device, hardware_cfg=self.hardware_cfg)
                new_hw.open()
            except Exception as exc:
                if old_hw is not None:
                    try:
                        old_hw.open()
                    except Exception:
                        pass
                self.hw = old_hw
                self.direction_by_id = old_direction
                self._ids = old_ids
                self.device = old_device
                raise RuntimeError(f"failed to open device {new_device}: {exc}") from exc
            self.hw = new_hw
            self.direction_by_id = new_direction
            self._ids = list(getattr(new_hw, "ids", []))
            self.device = new_device
            self._arm_servo_start_skip_logged = False
            start_servo = True
        if start_servo:
            self._start_arm_servo_thread()

    def clear_device(self) -> None:
        self._stop_arm_servo_thread()
        with self._hw_lock:
            old_hw = self.hw
            self._pending_target_q = None
            self._pending_target_u = None
            self._pending_target_axes = set()
            self._pending_target_seq = -1
            self._cancel_trajectory()
            self._target_u_state = None
            self.last_u = None
            self.last_q = None
            self.last_state_ts = 0.0
            self.last_sim_u = None
            self.last_sim_q = None
            self.last_sim_state_ts = 0.0
            self.torque_enabled = False
            self.last_ik_target_xyz = None
            self.last_ik_target_dir = None
            self.last_actual_tip_xyz = None
            self.last_actual_tip_dir = None
            self.last_perceived_object_label = ""
            self.last_perceived_object_confidence = 0.0
            self.last_perceived_object_camera_xyz = None
            self.last_perceived_center_uv = None
            self.last_perceived_scale = None
            self.last_perceived_timestamp_s = 0.0
            self.perception_hz = 0.0
            self._perception_rate_last_t = 0.0
            self._perception_rate_last_frame_idx = -1
            self.last_sag_model = {}
            self.last_claw_closed = False
            self._clear_go2_vel()
            self._last_hw_pos_by_id = {}
            self._last_claw_current = 0
            self._claw_close_stalled = False
            self._last_motor_current_by_id = {}
            self._safety_fault = ""
            self._yellow_zone_ids = set()
            self._red_torque_off_ids = set()
            self.hw = None
            self.direction_by_id = {}
            self._ids = []
            self.device = ""
            self._set_virtual_neutral_state()
            if old_hw is not None:
                try:
                    old_hw.close()
                except Exception:
                    pass

    def _is_allowed_source(self, source: str) -> bool:
        return str(source) in (
            "slider",
            "ik",
            "sim",
            "target",
            "perception",
            "gaze",
            "lji",
            "lji_step",
            "servo",
            "experiment",
        )

    def _active_debug_markers(self) -> list[dict[str, Any]]:
        now = time.time()
        expired = [name for name, marker in self._debug_markers_by_name.items() if float(marker.get("_expiry_wall", 0.0)) < now]
        for name in expired:
            self._debug_markers_by_name.pop(name, None)
        out: list[dict[str, Any]] = []
        for marker in self._debug_markers_by_name.values():
            clean = {k: v for k, v in marker.items() if not str(k).startswith("_")}
            out.append(clean)
        return out

    def _set_debug_marker(
        self,
        *,
        name: str,
        pos: Any,
        frame: str = "world",
        direction: Optional[Any] = None,
        color: Optional[list[float]] = None,
        radius: Optional[float] = None,
        length: Optional[float] = None,
        ttl_ms: int = 250,
    ) -> None:
        marker: dict[str, Any] = {
            "name": str(name),
            "frame": str(frame),
            "pos": [float(pos[0]), float(pos[1]), float(pos[2])],
            "ttl_ms": int(ttl_ms),
            "_expiry_wall": time.time() + max(int(ttl_ms), 1) / 1000.0,
        }
        if direction is not None:
            marker["dir"] = [float(direction[0]), float(direction[1]), float(direction[2])]
        if color is not None:
            marker["color"] = [float(v) for v in color]
        if radius is not None:
            marker["radius"] = float(radius)
        if length is not None:
            marker["length"] = float(length)
        self._debug_markers_by_name[str(name)] = marker

    def _ready_pose_direction(self) -> tuple[float, float, float]:
        direction = np.asarray(self.last_ready_pose_dir, dtype=float).reshape(3)
        if float(np.linalg.norm(direction)) <= 1e-9 and self.last_ik_target_dir is not None:
            direction = np.asarray(self.last_ik_target_dir, dtype=float).reshape(3)
        if float(np.linalg.norm(direction)) <= 1e-9:
            direction = np.array([1.0, 0.0, 0.0], dtype=float)
        return (float(direction[0]), float(direction[1]), float(direction[2]))

    def _set_grasp_target_markers(
        self,
        object_world: Any,
        *,
        standoff_m: float,
        ttl_ms: int,
        corrected: bool = False,
    ) -> None:
        obj = np.asarray(object_world, dtype=float).reshape(3)
        direction = self._ready_pose_direction()
        try:
            target = compute_ready_pose_target(
                (float(obj[0]), float(obj[1]), float(obj[2])),
                direction,
                standoff_m=float(max(standoff_m, 0.0)),
            )
        except ValueError:
            return
        target_arr = np.asarray(target, dtype=float).reshape(3)
        standoff_vec = target_arr - obj
        actual_offset_m = float(np.linalg.norm(standoff_vec))
        color = [1.0, 0.75, 0.12, 0.95] if bool(corrected) else [0.35, 0.85, 1.0, 0.95]
        line_color = [1.0, 0.55, 0.05, 0.65] if bool(corrected) else [0.35, 0.85, 1.0, 0.60]
        self._set_debug_marker(
            name="grasp_target",
            pos=target,
            direction=direction,
            color=color,
            radius=0.014,
            ttl_ms=int(ttl_ms),
        )
        self._set_debug_marker(
            name="grasp_standoff",
            pos=(float(obj[0]), float(obj[1]), float(obj[2])),
            direction=(float(standoff_vec[0]), float(standoff_vec[1]), float(standoff_vec[2])),
            color=line_color,
            radius=0.006,
            length=actual_offset_m,
            ttl_ms=int(ttl_ms),
        )

    def _set_ready_pose_markers(self, object_world: Any, *, ttl_ms: int) -> None:
        obj = np.asarray(object_world, dtype=float).reshape(3)
        direction = self._ready_pose_direction()
        try:
            target = compute_ready_pose_target(
                (float(obj[0]), float(obj[1]), float(obj[2])),
                direction,
                standoff_m=float(self.last_ready_pose_standoff_m),
            )
        except ValueError:
            return
        target_arr = np.asarray(target, dtype=float).reshape(3)
        standoff_vec = target_arr - obj
        actual_offset_m = float(np.linalg.norm(standoff_vec))
        self._set_debug_marker(
            name="ready_pose",
            pos=target,
            direction=direction,
            color=[0.72, 1.0, 0.28, 0.95],
            radius=0.014,
            ttl_ms=int(ttl_ms),
        )
        self._set_debug_marker(
            name="ready_pose_standoff",
            pos=(float(obj[0]), float(obj[1]), float(obj[2])),
            direction=(float(standoff_vec[0]), float(standoff_vec[1]), float(standoff_vec[2])),
            color=[0.72, 1.0, 0.28, 0.60],
            radius=0.006,
            length=actual_offset_m,
            ttl_ms=int(ttl_ms),
        )

    def _set_perception_debug_markers(
        self,
        *,
        object_world: tuple[float, float, float],
        object_label: str,
        object_camera_xyz: tuple[float, float, float],
        world_tag: str,
        camera_world: Optional[tuple[float, float, float]] = None,
        camera_look: Optional[tuple[float, float, float]] = None,
        camera_right: Optional[tuple[float, float, float]] = None,
        ttl_ms: int = 3000,
    ) -> None:
        p_cam = np.asarray(object_camera_xyz, dtype=float).reshape(3)
        p_w = np.asarray(object_world, dtype=float).reshape(3)
        label_txt = str(object_label).strip()
        log_key = f"{label_txt}:{world_tag}"
        now = time.time()
        if (
            log_key != self._last_perception_log_key
            or (now - float(self._last_perception_log_t)) >= float(self._perception_log_interval_s)
        ):
            self._last_perception_log_key = log_key
            self._last_perception_log_t = now
            print(
                f"[Perception] label={label_txt or '-'} "
                f"camera=[{p_cam[0]:+.4f}, {p_cam[1]:+.4f}, {p_cam[2]:+.4f}] m "
                f"world=[{p_w[0]:+.4f}, {p_w[1]:+.4f}, {p_w[2]:+.4f}] m ({world_tag})"
            )
        label_suffix = f":{label_txt}" if label_txt else ""
        self._set_debug_marker(
            name=f"perceived_object{label_suffix}",
            pos=object_world,
            color=[0.1, 0.95, 0.2, 0.95],
            radius=0.012,
            ttl_ms=int(ttl_ms),
        )
        if camera_world is not None and camera_look is not None and camera_right is not None:
            self._set_debug_marker(
                name="camera_optical",
                pos=camera_world,
                color=[0.1, 0.7, 1.0, 0.95],
                radius=0.010,
                ttl_ms=int(ttl_ms),
            )
            self._set_debug_marker(
                name="camera_look",
                pos=camera_world,
                direction=camera_look,
                color=[0.1, 0.7, 1.0, 0.95],
                radius=0.004,
                ttl_ms=int(ttl_ms),
            )
            self._set_debug_marker(
                name="camera_right",
                pos=camera_world,
                direction=camera_right,
                color=[1.0, 0.8, 0.2, 0.95],
                radius=0.004,
                ttl_ms=int(ttl_ms),
            )

    def _sim_camera_pose_fresh(self, *, max_age_s: float = 0.35) -> bool:
        if self._sim_camera_origin is None:
            return False
        if float(self._sim_camera_ts) <= 0.0:
            return False
        return (time.time() - float(self._sim_camera_ts)) <= float(max(max_age_s, 0.05))

    def _update_perception_markers(
        self, object_camera_xyz: tuple[float, float, float], *, object_label: str = ""
    ) -> tuple[bool, str, Optional[np.ndarray]]:
        if self.hand_eye_transform is None:
            return False, "perception disabled: missing hand-eye", None
        p_cam = np.asarray(object_camera_xyz, dtype=float).reshape(3)
        if self._sim_camera_pose_fresh():
            origin = np.asarray(self._sim_camera_origin, dtype=float).reshape(3)
            look = np.asarray(self._sim_camera_look, dtype=float).reshape(3)
            right = np.asarray(self._sim_camera_right, dtype=float).reshape(3)
            try:
                p_w = camera_point_to_world_from_axes(origin, look, right, p_cam)
                object_world = (float(p_w[0]), float(p_w[1]), float(p_w[2]))
                self.last_perceived_object_world_xyz = object_world
                self._set_perception_debug_markers(
                    object_world=object_world,
                    object_label=object_label,
                    object_camera_xyz=tuple(float(v) for v in p_cam),
                    world_tag="sim feedback",
                    camera_world=(float(origin[0]), float(origin[1]), float(origin[2])),
                    camera_look=(float(look[0]), float(look[1]), float(look[2])),
                    camera_right=(float(right[0]), float(right[1]), float(right[2])),
                )
                return True, "perception markers updated (sim feedback)", np.asarray(object_world, dtype=float)
            except Exception:
                pass
        if not self.ik_context or self.last_q is None:
            return False, "perception rejected: no robot q available yet", None
        q4 = np.array(
            [
                float(self.last_q.linear_m),
                float(self.last_q.roll_rad),
                float(self.last_q.theta1_rad),
                float(self.last_q.theta2_rad),
            ],
            dtype=float,
        )
        try:
            object_world = camera_point_to_world(
                self.ik_context,
                q4,
                self.hand_eye_transform,
                np.asarray(object_camera_xyz, dtype=float).reshape(3),
                parent_frame=self.hand_eye_parent_frame,
            )
            camera_world, camera_look, camera_right = camera_axes_world(
                self.ik_context,
                q4,
                self.hand_eye_transform,
                parent_frame=self.hand_eye_parent_frame,
            )
        except Exception as exc:
            return False, f"perception transform failed: {exc}", None
        p_cam = np.asarray(object_camera_xyz, dtype=float).reshape(3)
        p_w = np.asarray(object_world, dtype=float).reshape(3)
        self.last_perceived_object_world_xyz = (
            float(p_w[0]),
            float(p_w[1]),
            float(p_w[2]),
        )
        label_txt = str(object_label).strip()
        self._set_perception_debug_markers(
            object_world=(float(p_w[0]), float(p_w[1]), float(p_w[2])),
            object_label=object_label,
            object_camera_xyz=(float(p_cam[0]), float(p_cam[1]), float(p_cam[2])),
            world_tag="fk",
            camera_world=(float(camera_world[0]), float(camera_world[1]), float(camera_world[2])),
            camera_look=(float(camera_look[0]), float(camera_look[1]), float(camera_look[2])),
            camera_right=(float(camera_right[0]), float(camera_right[1]), float(camera_right[2])),
        )
        return True, "perception markers updated", p_w

    def _apply_go2_base_from_odom(self, sample: OdomSample) -> None:
        self.last_go2_base_pos = tuple(float(v) for v in sample.pos)
        self.last_go2_base_rpy = tuple(float(v) for v in sample.rpy)
        self.last_go2_base_lin_vel_body = tuple(float(v) for v in sample.lin_vel_body)
        self.last_go2_base_ang_vel = tuple(float(v) for v in sample.ang_vel_body)
        if sample.leg_q is not None and len(sample.leg_q) == 12:
            self.last_go2_leg_q = tuple(float(v) for v in sample.leg_q)
        if sample.leg_dq is not None and len(sample.leg_dq) == 12:
            self.last_go2_leg_dq = tuple(float(v) for v in sample.leg_dq)
        if sample.leg_torque_nm is not None and len(sample.leg_torque_nm) == 12:
            self.last_go2_leg_torque_nm = tuple(float(v) for v in sample.leg_torque_nm)
        self.last_go2_base_timestamp_s = float(sample.timestamp_s)

    def _broadcast_state_now(self) -> None:
        now = proto.now_s()
        perception_payload = self._perception_state_payload()
        gaze_payload = self._gaze_state_payload()
        pick_payload = self._pick_state_payload()
        self._broadcast(
            proto.pack_state(
                u=self.last_u,
                q=self.last_q,
                sim_q=self.last_sim_q,
                ts=self.last_state_ts or now,
                torque_enabled=self.torque_enabled,
                ik_target_xyz=self.last_ik_target_xyz,
                ik_target_dir=self.last_ik_target_dir,
                actual_tip_xyz=self.last_actual_tip_xyz,
                actual_tip_dir=self.last_actual_tip_dir,
                perceived_object_label=(self.last_perceived_object_label or None),
                perceived_object_confidence=self.last_perceived_object_confidence,
                perceived_object_camera=self.last_perceived_object_camera_xyz,
                perceived_center_uv=self.last_perceived_center_uv,
                perceived_scale=self.last_perceived_scale,
                perceived_timestamp_s=(self.last_perceived_timestamp_s or None),
                perception_running=bool(self.perception_running),
                perception_failed=bool(self.perception_failed),
                perception_status=str(self.perception_status),
                perception_source=str(self.perception_source),
                perception_preview_endpoint=str(getattr(self.perception_config, "preview_bind", "")),
                perception_recording=bool(perception_payload.get("perception_recording", False)),
                perception_record_with_overlay=bool(perception_payload.get("perception_record_with_overlay", False)),
                perception_last_record_path=str(perception_payload.get("perception_last_record_path", "")),
                perception_last_capture_path=str(perception_payload.get("perception_last_capture_path", "")),
                perception_hz=float(perception_payload.get("perception_hz", 0.0)),
                gaze_running=bool(gaze_payload.get("gaze_running", False)),
                gaze_mode=str(gaze_payload.get("gaze_mode", "idle")),
                gaze_status_msg=str(gaze_payload.get("gaze_status_msg", "")),
                gaze_u_err=float(gaze_payload.get("gaze_u_err", 0.0)),
                gaze_v_err=float(gaze_payload.get("gaze_v_err", 0.0)),
                gaze_du_roll=float(gaze_payload.get("gaze_du_roll", 0.0)),
                gaze_du_s1=float(gaze_payload.get("gaze_du_s1", 0.0)),
                gaze_du_s2=float(gaze_payload.get("gaze_du_s2", 0.0)),
                gaze_obs_age_s=float(gaze_payload.get("gaze_obs_age_s", -1.0)),
                gaze_tick_count=int(gaze_payload.get("gaze_tick_count", 0)),
                gaze_update_count=int(gaze_payload.get("gaze_update_count", 0)),
                gaze_config=dict(gaze_payload.get("gaze_config", {})),
                pick_running=bool(pick_payload.get("pick_running", False)),
                pick_failed=bool(pick_payload.get("pick_failed", False)),
                pick_phase=str(pick_payload.get("pick_phase", "idle")),
                pick_status_msg=str(pick_payload.get("pick_status_msg", "")),
                sag_model=self.last_sag_model,
                claw_closed=self.last_claw_closed,
                go2_vel=self._effective_go2_vel(now),
                go2_base_rpy=self.last_go2_base_rpy,
                go2_base_pos=self.last_go2_base_pos,
                go2_base_lin_vel_body=self.last_go2_base_lin_vel_body,
                go2_base_ang_vel=self.last_go2_base_ang_vel,
                go2_base_timestamp_s=(self.last_go2_base_timestamp_s or None),
                go2_leg_q=self.last_go2_leg_q,
                go2_leg_dq=self.last_go2_leg_dq,
                go2_leg_torque_nm=self.last_go2_leg_torque_nm,
                go2_sport_pose=(self.last_go2_sport_pose or None),
                go2_sport_pose_seq=int(self.last_go2_sport_pose_seq),
                go2_obstacles_avoid_enabled=bool(self.last_go2_obstacles_avoid_enabled),
                go2_obstacles_avoid_seq=int(self.last_go2_obstacles_avoid_seq),
                sim_target_xyz=self.last_sim_target_xyz,
                sim_reset_seq=int(self._sim_reset_seq),
                sim_time_s=self.last_sim_time_s,
                sim_wall_elapsed_s=self.last_sim_wall_elapsed_s,
                sim_realtime_factor=self.last_sim_realtime_factor,
                sim_step_count=self.last_sim_step_count,
                claw_current=self._last_claw_current,
                motor_currents_ma={self._motor_name_by_id(int(k)): int(v) for k, v in self._last_motor_current_by_id.items()},
                safety_fault=(self._safety_fault or None),
                debug_markers=self._active_debug_markers(),
            )
        )

    def _host_state_snapshot(
        self,
        *,
        tx_seq: int = 0,
        reply_ok: bool = True,
        reply_reason: str = "",
    ) -> HostState:
        now = proto.now_s()
        perception_payload = self._perception_state_payload()
        gaze_payload = self._gaze_state_payload()
        pick_payload = self._pick_state_payload()
        return HostState(
            connected=True,
            tx_seq=int(tx_seq),
            rx_age_s=0.0,
            device=str(self.device),
            ports=(),
            torque_enabled=bool(self.torque_enabled),
            claw_current=int(self._last_claw_current),
            motor_currents_ma={
                self._motor_name_by_id(int(k)): int(v)
                for k, v in self._last_motor_current_by_id.items()
            },
            safety_fault=str(self._safety_fault or ""),
            actual_tip_xyz=self.last_actual_tip_xyz,
            actual_tip_dir=self.last_actual_tip_dir,
            perceived_object_label=str(self.last_perceived_object_label or ""),
            perceived_object_confidence=float(self.last_perceived_object_confidence),
            perceived_object_camera_xyz=self.last_perceived_object_camera_xyz,
            perceived_center_uv=self.last_perceived_center_uv,
            perceived_scale=self.last_perceived_scale,
            perceived_timestamp_s=float(self.last_perceived_timestamp_s or 0.0),
            reply_ok=bool(reply_ok),
            reply_reason=str(reply_reason),
            q=self.last_q,
            u=self.last_u,
            sim_q=self.last_sim_q,
            sim_u=self.last_sim_u,
            go2_vel=self._effective_go2_vel(now),
            go2_base_rpy=self.last_go2_base_rpy,
            go2_base_pos=self.last_go2_base_pos,
            go2_base_lin_vel_body=self.last_go2_base_lin_vel_body,
            go2_base_ang_vel=self.last_go2_base_ang_vel,
            go2_base_timestamp_s=float(self.last_go2_base_timestamp_s or 0.0),
            host_state_age_s=0.0,
            go2_leg_q=self.last_go2_leg_q,
            go2_leg_dq=self.last_go2_leg_dq,
            go2_leg_torque_nm=self.last_go2_leg_torque_nm,
            go2_sport_pose=str(self.last_go2_sport_pose or ""),
            go2_sport_pose_seq=int(self.last_go2_sport_pose_seq),
            go2_obstacles_avoid_enabled=bool(self.last_go2_obstacles_avoid_enabled),
            go2_obstacles_avoid_seq=int(self.last_go2_obstacles_avoid_seq),
            sim_time_s=float(self.last_sim_time_s),
            sim_wall_elapsed_s=float(self.last_sim_wall_elapsed_s),
            sim_realtime_factor=float(self.last_sim_realtime_factor),
            sim_step_count=int(self.last_sim_step_count),
            perception_running=bool(self.perception_running),
            perception_failed=bool(self.perception_failed),
            perception_status=str(self.perception_status),
            perception_source=str(self.perception_source),
            perception_preview_endpoint=str(getattr(self.perception_config, "preview_bind", "")),
            perception_recording=bool(perception_payload.get("perception_recording", False)),
            perception_record_with_overlay=bool(
                perception_payload.get("perception_record_with_overlay", False)
            ),
            perception_last_record_path=str(perception_payload.get("perception_last_record_path", "")),
            perception_last_capture_path=str(perception_payload.get("perception_last_capture_path", "")),
            perception_hz=float(perception_payload.get("perception_hz", 0.0)),
            gaze_running=bool(gaze_payload.get("gaze_running", False)),
            gaze_mode=str(gaze_payload.get("gaze_mode", "idle")),
            gaze_status_msg=str(gaze_payload.get("gaze_status_msg", "")),
            gaze_u_err=float(gaze_payload.get("gaze_u_err", 0.0)),
            gaze_v_err=float(gaze_payload.get("gaze_v_err", 0.0)),
            gaze_du_roll=float(gaze_payload.get("gaze_du_roll", 0.0)),
            gaze_du_s1=float(gaze_payload.get("gaze_du_s1", 0.0)),
            gaze_du_s2=float(gaze_payload.get("gaze_du_s2", 0.0)),
            gaze_obs_age_s=float(gaze_payload.get("gaze_obs_age_s", -1.0)),
            gaze_tick_count=int(gaze_payload.get("gaze_tick_count", 0)),
            gaze_update_count=int(gaze_payload.get("gaze_update_count", 0)),
            gaze_config=dict(gaze_payload.get("gaze_config", {})),
            pick_running=bool(pick_payload.get("pick_running", False)),
            pick_failed=bool(pick_payload.get("pick_failed", False)),
            pick_phase=str(pick_payload.get("pick_phase", "idle")),
            pick_status_msg=str(pick_payload.get("pick_status_msg", "")),
        )

    def _gaze_state_payload(self) -> Dict[str, Any]:
        cfg = self._effective_gaze_config()
        service = self._embedded_control_service
        if service is None:
            return {
                "gaze_running": False,
                "gaze_mode": "idle",
                "gaze_status_msg": "",
                "gaze_u_err": 0.0,
                "gaze_v_err": 0.0,
                "gaze_du_roll": 0.0,
                "gaze_du_s1": 0.0,
                "gaze_du_s2": 0.0,
                "gaze_obs_age_s": -1.0,
                "gaze_tick_count": 0,
                "gaze_update_count": 0,
                "gaze_config": gaze_config_to_dict(cfg),
            }
        st = service.state
        with st._lock:
            return {
                "gaze_running": bool(st.gaze_running),
                "gaze_mode": str(st.gaze_mode),
                "gaze_status_msg": str(st.gaze_status_msg),
                "gaze_u_err": float(st.gaze_u_err),
                "gaze_v_err": float(st.gaze_v_err),
                "gaze_du_roll": float(st.gaze_du_roll),
                "gaze_du_s1": float(st.gaze_du_s1),
                "gaze_du_s2": float(st.gaze_du_s2),
                "gaze_obs_age_s": float(st.gaze_obs_age_s),
                "gaze_tick_count": int(st.gaze_tick_count),
                "gaze_update_count": int(st.gaze_update_count),
                "gaze_config": gaze_config_to_dict(cfg),
            }

    def _pick_state_payload(self) -> Dict[str, Any]:
        service = self._embedded_control_service
        if service is None:
            return {
                "pick_running": False,
                "pick_failed": False,
                "pick_phase": "idle",
                "pick_status_msg": "",
            }
        st = service.state
        with st._lock:
            running = bool(st.pick_running)
            failed = bool(st.pick_failed)
            phase = str(st.pick_phase or "idle")
            msg = str(st.pick_status_msg or "")
        try:
            running = bool(running or service.pick_e2e_running())
        except Exception:
            pass
        return {
            "pick_running": bool(running),
            "pick_failed": bool(failed),
            "pick_phase": str(phase),
            "pick_status_msg": str(msg),
        }

    def _effective_gaze_config(self) -> GazeStabilizerConfig:
        service = self._embedded_control_service
        if service is not None:
            try:
                return service.gaze_config
            except Exception:
                pass
        if isinstance(self.gaze_config, GazeStabilizerConfig):
            return self.gaze_config
        return GazeStabilizerConfig()

    def _new_on_device_panel_state(self):
        from engine.behaviors.pick import PanelState

        state = PanelState()
        hw = self.hardware_cfg
        if hw is not None:
            state.set_u_offsets(
                linear=float(getattr(hw, "u_offset_linear", 0.0)),
                roll=float(getattr(hw, "u_offset_roll", 0.0)),
                s1=float(getattr(hw, "u_offset_s1", 0.0)),
                s2=float(getattr(hw, "u_offset_s2", 0.0)),
            )
        pk = self.pick_config
        pc = self.perception_config
        state.visual_target_label = str(getattr(pc, "target_label", "")).strip()
        state.visual_target_scale = float(getattr(pk, "target_scale", state.visual_target_scale))
        state.visual_center_tol = float(getattr(pk, "center_tol", state.visual_center_tol))
        state.visual_target_uv_u = float(getattr(pk, "target_uv_u", state.visual_target_uv_u))
        state.visual_target_uv_v = float(getattr(pk, "target_uv_v", state.visual_target_uv_v))
        state.visual_scale_tol = float(getattr(pk, "scale_tol", state.visual_scale_tol))
        state.visual_ready_distance_m = float(
            getattr(pk, "ready_pose_standoff_m", state.visual_ready_distance_m)
        )
        state.visual_look_distance_m = float(
            getattr(pk, "look_pose_standoff_m", state.visual_look_distance_m)
        )
        if self.last_q is not None:
            state.set_q(
                float(self.last_q.linear_m),
                float(self.last_q.roll_rad),
                float(self.last_q.theta1_rad),
                float(self.last_q.theta2_rad),
            )
        return state

    def _ensure_on_device_control_service(self):
        if self._embedded_control_service is not None:
            client = self._embedded_control_client
            service_hw = bool(getattr(self._embedded_control_service, "_use_hardware", False))
            if (
                service_hw == bool(self._has_hw())
                and bool(getattr(client, "host_native_control", False))
            ):
                return self._embedded_control_service
            self._close_on_device_control_service()
        from engine.behaviors.pick import ControlService

        perception_cfg = replace(
            self.perception_config,
            run_local=False,
            provider="host",
            show_preview=False,
        )
        client = _DirectEmbeddedControlClient(self, self.cfg)
        service = ControlService(
            self._new_on_device_panel_state(),
            client=client,
            mapping_cfg=self.cfg,
            ik_cfg=self.ik_config,
            ik_context=self.ik_context,
            config_path=self.config_path or None,
            perception_cfg=perception_cfg,
            pick_cfg=self.pick_config,
            gaze_cfg=self.gaze_config,
            ownership_enable=bool(self.ownership_enable),
            hand_eye_transform=self.hand_eye_transform,
            hand_eye_parent_frame=self.hand_eye_parent_frame,
            use_hardware=self._has_hw(),
            remote_gaze_delegate=False,
        )
        self._embedded_control_client = client
        self._embedded_control_service = service
        print("[host] on-device robot control service ready | host-native LJI direct path")
        self._broadcast_state_now()
        return service

    def _stop_on_device_gaze(self) -> None:
        service = self._embedded_control_service
        if service is not None:
            try:
                service.stop_gaze_stabilizer()
            except Exception as exc:
                print(f"[host] on-device gaze stop failed: {exc}")

    def _close_on_device_control_service(self) -> None:
        self._stop_on_device_gaze()
        client = self._embedded_control_client
        if client is not None:
            try:
                client.close()
            except Exception:
                pass
        self._embedded_control_client = None
        self._embedded_control_service = None

    def _update_external_debug_markers(self, raw_markers: list[dict[str, Any]]) -> tuple[bool, str]:
        updated = 0
        for raw in list(raw_markers):
            if not isinstance(raw, dict):
                continue
            name = str(raw.get("name", "")).strip()
            frame = str(raw.get("frame", "world")).strip() or "world"
            pos = raw.get("pos", None)
            if frame != "world" or not name or not isinstance(pos, (list, tuple)) or len(pos) != 3:
                continue
            direction = raw.get("dir", None)
            if not (isinstance(direction, (list, tuple)) and len(direction) == 3):
                direction = None
            color = raw.get("color", None)
            if not (isinstance(color, (list, tuple)) and len(color) in (3, 4)):
                color = None
            radius = raw.get("radius", None)
            length = raw.get("length", None)
            ttl_ms = int(raw.get("ttl_ms", 250))
            self._set_debug_marker(
                name=name,
                pos=pos,
                frame="world",
                direction=direction,
                color=list(color) if color is not None else None,
                radius=float(radius) if radius is not None else None,
                length=float(length) if length is not None else None,
                ttl_ms=ttl_ms,
            )
            updated += 1
        return (updated > 0), (f"debug markers updated: {updated}" if updated > 0 else "no valid world debug markers")

    def _reply(self, ident: bytes, msg: Dict[str, Any]) -> None:
        try:
            self.sock.send_multipart([ident, proto.dumps_msg(msg)], flags=0)
        except Exception:
            pass

    def _broadcast(self, msg: Dict[str, Any]) -> None:
        data = proto.dumps_msg(msg)
        dead: Set[bytes] = set()
        for ident in list(self.clients):
            try:
                self.sock.send_multipart([ident, data], flags=zmq.NOBLOCK)
            except Exception:
                dead.add(ident)
        self.clients.difference_update(dead)
        try:
            self.sim_pub.send(data, flags=zmq.NOBLOCK)
        except Exception:
            pass

    def _arm_latency_count(self, name: str, n: int = 1) -> None:
        if not bool(self._arm_latency_log_enable):
            return
        key = str(name)
        self._arm_latency_counts[key] = int(self._arm_latency_counts.get(key, 0)) + int(n)

    def _arm_latency_sample_ms(self, name: str, value_ms: float) -> None:
        if not bool(self._arm_latency_log_enable):
            return
        value = float(value_ms)
        if not np.isfinite(value):
            return
        key = str(name)
        count_key = f"{key}.n"
        self._arm_latency_counts[count_key] = int(self._arm_latency_counts.get(count_key, 0)) + 1
        self._arm_latency_sums[key] = float(self._arm_latency_sums.get(key, 0.0)) + value
        self._arm_latency_max[key] = max(float(self._arm_latency_max.get(key, value)), value)

    def _init_arm_latency_csv(self) -> None:
        if not bool(self._arm_latency_log_enable):
            return
        raw_path = str(
            os.environ.get(
                "ELESIM_ARM_LATENCY_LOG",
                getattr(self.hardware_cfg, "arm_latency_log_path", "") if self.hardware_cfg is not None else "",
            )
            or ""
        ).strip()
        if not raw_path:
            raw_path = "engine/logs/arm_latency"
        path = Path(raw_path).expanduser()
        if not path.is_absolute():
            path = Path.cwd() / path
        if path.suffix.lower() != ".csv":
            stamp = time.strftime("%Y%m%d_%H%M%S")
            path = path / f"{stamp}_arm_latency.csv"
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            file_exists = path.exists() and path.stat().st_size > 0
            self._arm_latency_csv_file = open(path, "a", newline="", encoding="utf-8")
            fields = [
                "ts",
                "recv",
                "submit",
                "submit_reject",
                "apply",
                "replace",
                "client_to_host_avg_ms",
                "client_to_host_max_ms",
                "client_to_write_avg_ms",
                "client_to_write_max_ms",
                "obs_age_avg_ms",
                "obs_age_max_ms",
                "recv_to_write_avg_ms",
                "recv_to_write_max_ms",
                "sync_write_avg_ms",
                "sync_write_max_ms",
                "follow50_avg_ms",
                "follow50_max_ms",
                "follow90_avg_ms",
                "follow90_max_ms",
                "err_u",
                "source",
                "seq",
                "last_error",
            ]
            self._arm_latency_csv_writer = csv.DictWriter(self._arm_latency_csv_file, fieldnames=fields)
            if not file_exists:
                self._arm_latency_csv_writer.writeheader()
                self._arm_latency_csv_file.flush()
            self._arm_latency_csv_path = str(path)
            print(f"[arm_latency] csv={self._arm_latency_csv_path}")
        except Exception as exc:
            self._arm_latency_csv_path = ""
            self._arm_latency_csv_file = None
            self._arm_latency_csv_writer = None
            print(f"[arm_latency] csv open failed: {exc}")

    def _arm_latency_avg_max_values(self, name: str) -> tuple[Optional[float], Optional[float]]:
        n = int(self._arm_latency_counts.get(f"{name}.n", 0))
        if n <= 0:
            return None, None
        avg = float(self._arm_latency_sums.get(name, 0.0)) / float(n)
        mx = float(self._arm_latency_max.get(name, 0.0))
        return avg, mx

    def _arm_latency_write_csv_row(self, row: dict[str, Any]) -> None:
        writer = self._arm_latency_csv_writer
        fh = self._arm_latency_csv_file
        if writer is None or fh is None:
            return
        try:
            writer.writerow(row)
            fh.flush()
        except Exception as exc:
            print(f"[arm_latency] csv write failed: {exc}")

    def _control_u_value(self, u: proto.ControlU, axis: str) -> float:
        key = str(axis).strip().lower()
        if key == "linear":
            return float(u.u_linear)
        if key == "roll":
            return float(u.u_roll)
        if key == "s1":
            return float(u.u_s1)
        if key == "s2":
            return float(u.u_s2)
        return 0.0

    def _control_u_err_max(self, target_u: proto.ControlU, actual_u: proto.ControlU, axes: Set[str]) -> float:
        vals = [
            abs(self._control_u_value(target_u, axis) - self._control_u_value(actual_u, axis))
            for axis in set(axes)
            if str(axis).strip().lower() in ("linear", "roll", "s1", "s2")
        ]
        return float(max(vals) if vals else 0.0)

    def _arm_latency_maybe_log(self, *, now_s: Optional[float] = None) -> None:
        if not bool(self._arm_latency_log_enable):
            return
        now = time.time() if now_s is None else float(now_s)
        if (now - float(self._arm_latency_last_log_t)) < float(self._arm_latency_log_interval_s):
            return
        self._arm_latency_last_log_t = now

        def _avg_max(name: str) -> str:
            avg, mx = self._arm_latency_avg_max_values(name)
            if avg is None or mx is None:
                return "-"
            return f"{avg:.1f}/{mx:.1f}"

        follow = dict(self._arm_latency_last_follow or {})
        err_now = follow.get("last_err_u", None)
        err_s = "-" if err_now is None else f"{float(err_now):.2f}u"
        names = [
            "client_to_host_ms",
            "client_to_write_ms",
            "obs_age_ms",
            "recv_to_write_ms",
            "sync_write_ms",
            "follow50_ms",
            "follow90_ms",
        ]
        row: dict[str, Any] = {
            "ts": float(now),
            "recv": int(self._arm_latency_counts.get("recv", 0)),
            "submit": int(self._arm_latency_counts.get("submit", 0)),
            "submit_reject": int(self._arm_latency_counts.get("submit_reject", 0)),
            "apply": int(self._arm_latency_counts.get("apply", 0)),
            "replace": int(self._arm_latency_counts.get("replace", 0)),
            "err_u": "" if err_now is None else float(err_now),
            "source": str(follow.get("source", self._arm_servo_target_source) or ""),
            "seq": int(follow.get("seq", self._arm_servo_applied_seq)),
            "last_error": str(self._arm_servo_last_error or ""),
        }
        for name in names:
            avg, mx = self._arm_latency_avg_max_values(name)
            prefix = name.removesuffix("_ms")
            row[f"{prefix}_avg_ms"] = "" if avg is None else float(avg)
            row[f"{prefix}_max_ms"] = "" if mx is None else float(mx)
        print(
            "[arm_latency] "
            f"recv={int(self._arm_latency_counts.get('recv', 0))} "
            f"submit={int(self._arm_latency_counts.get('submit', 0))} "
            f"reject={int(self._arm_latency_counts.get('submit_reject', 0))} "
            f"apply={int(self._arm_latency_counts.get('apply', 0))} "
            f"replace={int(self._arm_latency_counts.get('replace', 0))} "
            f"client->host(avg/max)={_avg_max('client_to_host_ms')}ms "
            f"client->write(avg/max)={_avg_max('client_to_write_ms')}ms "
            f"obs_age(avg/max)={_avg_max('obs_age_ms')}ms "
            f"recv->write(avg/max)={_avg_max('recv_to_write_ms')}ms "
            f"sync_write(avg/max)={_avg_max('sync_write_ms')}ms "
            f"follow50(avg/max)={_avg_max('follow50_ms')}ms "
            f"follow90(avg/max)={_avg_max('follow90_ms')}ms "
            f"err={err_s} "
            f"source={str(follow.get('source', self._arm_servo_target_source) or '-')} "
            f"seq={int(follow.get('seq', self._arm_servo_applied_seq))} "
            f"last_error={self._arm_servo_last_error or '-'}"
        )
        self._arm_latency_write_csv_row(row)
        self._arm_latency_counts.clear()
        self._arm_latency_sums.clear()
        self._arm_latency_max.clear()

    def _arm_latency_observe_follow(self, now_s: Optional[float] = None) -> None:
        if not bool(self._arm_latency_log_enable) or self.last_u is None:
            return
        follow = self._arm_latency_last_follow
        if not follow:
            self._arm_latency_maybe_log(now_s=now_s)
            return
        target_u = follow.get("target_u")
        axes_raw = follow.get("axes", set())
        if not isinstance(target_u, proto.ControlU):
            self._arm_latency_maybe_log(now_s=now_s)
            return
        axes = set(axes_raw) if isinstance(axes_raw, (set, list, tuple)) else set()
        now = time.time() if now_s is None else float(now_s)
        err = self._control_u_err_max(target_u, self.last_u, axes)
        follow["last_err_u"] = float(err)
        initial = float(max(float(follow.get("initial_err_u", 0.0)), 1e-6))
        apply_s = float(follow.get("apply_s", now))
        if not bool(follow.get("hit50", False)) and err <= initial * 0.5:
            follow["hit50"] = True
            self._arm_latency_sample_ms("follow50_ms", (now - apply_s) * 1000.0)
        if not bool(follow.get("hit90", False)) and err <= max(initial * 0.1, 0.75):
            follow["hit90"] = True
            self._arm_latency_sample_ms("follow90_ms", (now - apply_s) * 1000.0)
        self._arm_latency_maybe_log(now_s=now)

    def _read_hw_state(self, *, read_currents: bool = True) -> None:
        if not self._has_hw():
            return
        try:
            with self._hw_lock:
                ticks_by_id = self.hw.get_present_positions()
        except Exception:
            return
        currents_by_id: Dict[int, int] = dict(self._last_motor_current_by_id)
        if bool(read_currents):
            for dxl_id in self._ids:
                try:
                    with self._hw_lock:
                        currents_by_id[int(dxl_id)] = int(self.hw.get_present_current(int(dxl_id)))
                except Exception:
                    continue
            self._last_motor_current_by_id = dict(currents_by_id)
            self._last_claw_current = int(currents_by_id.get(int(self.hw.cfg.id_claw), self._last_claw_current))
            self._check_current_limit()
        self._last_hw_pos_by_id = dict(ticks_by_id)
        if not self._ids or len(self._ids) < 4:
            return
        motor_deg_vals = []
        for dxl_id in self._ids[:4]:
            tick = int(ticks_by_id.get(dxl_id, 0))
            direction = int(self.direction_by_id.get(dxl_id, +1))
            motor_deg_vals.append(tick_to_deg_0_360(tick, direction))
        motor_deg = proto.ControlU(
            u_linear=motor_deg_vals[0],
            u_roll=motor_deg_vals[1],
            u_s1=motor_deg_vals[2],
            u_s2=motor_deg_vals[3],
        )
        self.last_q = proto.motor_deg_to_sim_q(motor_deg, self.cfg)
        self.last_u = proto.sim_q_to_control_u(self.last_q, self.cfg)
        if self._target_u_state is None:
            self._target_u_state = self.last_u
        self.last_state_ts = time.time()
        self._arm_latency_observe_follow(now_s=float(self.last_state_ts))

    def _motor_name_by_id(self, dxl_id: int) -> str:
        if not self._has_hw():
            return f"id_{int(dxl_id)}"
        cfg = self.hw.cfg
        mapping = {
            int(cfg.id_linear): "linear",
            int(cfg.id_roll): "roll",
            int(cfg.id_seg1): "seg1",
            int(cfg.id_seg2): "seg2",
            int(cfg.id_claw): "claw",
        }
        return mapping.get(int(dxl_id), f"id_{int(dxl_id)}")

    def _trip_safety_fault(self, reason: str, *, red_dxl_id: Optional[int] = None) -> None:
        self._safety_fault = str(reason)
        print(f"[host] RED zone trip: {self._safety_fault}")
        self._clear_arm_servo_target()
        self._pending_target_q = None
        self._pending_target_u = None
        self._pending_target_axes = set()
        self._pending_target_seq = -1
        self._cancel_trajectory()
        try:
            if self._has_hw():
                with self._hw_lock:
                    if red_dxl_id is None:
                        self.hw.torque_off_all()
                        self.torque_enabled = False
                        print("[host] RED zone action: torque off all motors (unknown id)")
                    else:
                        red_id = int(red_dxl_id)
                        self.hw.torque_off_id(red_id)
                        self._red_torque_off_ids.add(red_id)
                        print(
                            f"[host] RED zone action: torque off {self._motor_name_by_id(red_id)} motor id={red_id}"
                        )
        except Exception as exc:
            print(f"[host] RED zone action failed: {exc}")

    def _check_current_limit(self) -> None:
        yellow = abs(int(self._current_yellow_ma))
        limit = abs(int(self._current_limit_ma))
        if limit <= 0 or self._safety_fault:
            return
        next_yellow_ids: Set[int] = set()
        for dxl_id, current_ma in list(self._last_motor_current_by_id.items()):
            current_abs = abs(int(current_ma))
            if yellow > 0 and current_abs > yellow:
                next_yellow_ids.add(int(dxl_id))
                if int(dxl_id) not in self._yellow_zone_ids:
                    print(
                        f"[host] YELLOW zone: {self._motor_name_by_id(int(dxl_id))} current={int(current_ma)} mA "
                        f"(yellow={yellow}, red={limit})"
                    )
            if abs(int(current_ma)) > limit:
                self._trip_safety_fault(
                    f"overcurrent {self._motor_name_by_id(int(dxl_id))}: {int(current_ma)} mA exceeds {limit} mA",
                    red_dxl_id=int(dxl_id),
                )
                return
        for dxl_id in (self._yellow_zone_ids - next_yellow_ids):
            current_ma = int(self._last_motor_current_by_id.get(int(dxl_id), 0))
            print(f"[host] YELLOW zone cleared: {self._motor_name_by_id(int(dxl_id))} current={current_ma} mA")
        self._yellow_zone_ids = next_yellow_ids

    def _yellow_scale_for_id(self, dxl_id: int) -> float:
        yellow = abs(int(self._current_yellow_ma))
        red = abs(int(self._current_limit_ma))
        if red <= 0 or yellow <= 0 or red <= yellow:
            return 1.0
        current_ma = abs(int(self._last_motor_current_by_id.get(int(dxl_id), 0)))
        if current_ma <= yellow:
            return 1.0
        if current_ma >= red:
            return 0.0
        frac = float(red - current_ma) / float(red - yellow)
        return float(max(min(frac, 1.0), 0.0))

    def _limit_target_q(self, q: proto.SimQ) -> tuple[proto.SimQ, bool]:
        if (not self._has_hw()) or self.last_q is None:
            return q, True
        ids = (
            int(self.hw.cfg.id_linear),
            int(self.hw.cfg.id_roll),
            int(self.hw.cfg.id_seg1),
            int(self.hw.cfg.id_seg2),
        )
        scales = [self._yellow_scale_for_id(dxl_id) for dxl_id in ids]
        current = self.last_q
        current_vals = np.array(
            [
                float(current.linear_m),
                float(current.roll_rad),
                float(current.theta1_rad),
                float(current.theta2_rad),
            ],
            dtype=float,
        )
        target_vals = np.array(
            [
                float(q.linear_m),
                float(q.roll_rad),
                float(q.theta1_rad),
                float(q.theta2_rad),
            ],
            dtype=float,
        )
        linear_min_m, linear_max_m = proto.linear_effective_q_bounds(self.cfg)
        target_vals[0] = float(np.clip(target_vals[0], linear_min_m, linear_max_m))
        limited_vals = current_vals.copy()
        complete = True
        for i, scale in enumerate(scales):
            limited_vals[i] = current_vals[i] + float(scale) * (target_vals[i] - current_vals[i])
            if abs(float(limited_vals[i] - target_vals[i])) > 1e-9:
                complete = False
        return (
            proto.SimQ(
                linear_m=float(limited_vals[0]),
                roll_rad=float(limited_vals[1]),
                theta1_rad=float(limited_vals[2]),
                theta2_rad=float(limited_vals[3]),
            ),
            bool(complete),
        )

    def _update_claw_hw(self) -> None:
        if (not self._has_hw()) or self._safety_fault:
            return
        claw_id = int(self.hw.cfg.id_claw)
        tick = self._last_hw_pos_by_id.get(claw_id, None)
        if tick is None:
            return
        claw_deg = tick_to_deg_0_360(int(tick), int(self.hw.direction.get(claw_id, +1)))
        if self.last_claw_closed:
            if int(self._last_claw_current) <= int(self._claw_stop_current):
                self._claw_close_stalled = True
                target_deg = float(claw_deg)
            else:
                self._claw_close_stalled = False
                target_deg = float(self._claw_close_deg)
        else:
            self._claw_close_stalled = False
            target_deg = float(self._claw_open_deg)
        try:
            with self._hw_lock:
                self.hw.command_claw_deg(target_deg)
        except Exception:
            return

    def _apply_sim_q_target(self, q: proto.SimQ) -> tuple[bool, bool]:
        if self._safety_fault:
            return False, False
        if not self._has_hw():
            return False, False
        q_limited, complete = self._limit_target_q(q)
        motor_deg = proto.sim_q_to_motor_deg(q_limited, self.cfg)
        try:
            with self._hw_lock:
                self.hw.command_4dof_deg(motor_deg.u_linear, motor_deg.u_roll, motor_deg.u_s1, motor_deg.u_s2)
            self._last_target_apply_error = ""
            return True, complete
        except Exception as exc:
            self._last_target_apply_error = f"hw_apply_failed: {exc}"
            print(f"[host] hw apply failed: {exc}")
            return False, False

    def _merge_partial_target_u(self, partial_u: Dict[str, float]) -> Optional[proto.ControlU]:
        base = self._target_u_state if self._target_u_state is not None else self.last_u
        if base is None:
            base = proto.DEFAULT_START_CONTROL_U
        values = {
            "linear": float(base.u_linear),
            "roll": float(base.u_roll),
            "s1": float(base.u_s1),
            "s2": float(base.u_s2),
        }
        changed_axes: Set[str] = set()
        for key, raw in partial_u.items():
            k = str(key).strip().lower()
            if k not in values:
                continue
            values[k] = float(raw)
            changed_axes.add(k)
        if not changed_axes:
            return None
        self._pending_target_axes = changed_axes
        merged = proto.ControlU(
            u_linear=float(values["linear"]),
            u_roll=float(values["roll"]),
            u_s1=float(values["s1"]),
            u_s2=float(values["s2"]),
        )
        self._target_u_state = merged
        self._pending_target_u = merged
        return merged

    def _apply_partial_u_target(self, u: proto.ControlU, axes: Set[str]) -> bool:
        if not axes:
            return True
        self._target_u_state = u
        if self._safety_fault:
            return False
        if not self._has_hw():
            self.last_u = u
            self.last_q = proto.control_u_to_sim_q(u, self.cfg)
            self.last_state_ts = time.time()
            return True
        q = proto.control_u_to_sim_q(u, self.cfg)
        q_limited, complete = self._limit_target_q(q)
        motor_deg = proto.sim_q_to_motor_deg(q_limited, self.cfg)
        goals_deg: Dict[int, float] = {}
        if "linear" in axes:
            goals_deg[self.hw.cfg.id_linear] = float(motor_deg.u_linear)
        if "roll" in axes:
            goals_deg[self.hw.cfg.id_roll] = float(motor_deg.u_roll)
        if "s1" in axes:
            goals_deg[self.hw.cfg.id_seg1] = float(motor_deg.u_s1)
        if "s2" in axes:
            goals_deg[self.hw.cfg.id_seg2] = float(motor_deg.u_s2)
        try:
            with self._hw_lock:
                self.hw.command_partial_deg(goals_deg)
            self._last_target_apply_error = ""
            return bool(complete)
        except Exception as exc:
            self._last_target_apply_error = f"hw_apply_failed: {exc}"
            print(f"[host] partial hw apply failed: {exc}")
            return False

    def _submit_direct_partial_control_u(
        self,
        partial_u: dict[str, float],
        *,
        source: str,
        seq: int,
        client_ts_s: float,
    ) -> tuple[bool, str]:
        target_recv_s = float(proto.now_s())
        source_s = str(source)
        if not self._is_allowed_source(source_s):
            return False, "source_reject"
        try:
            raw_u = {str(k).strip().lower(): float(v) for k, v in dict(partial_u).items()}
        except (TypeError, ValueError):
            return False, "bad_partial_u"
        u_keys = set(raw_u.keys())
        if not u_keys or not u_keys.issubset({"linear", "roll", "s1", "s2"}):
            return False, "bad_partial_u"
        if bool(self._arm_latency_log_enable):
            self._arm_latency_count("recv")
            if float(client_ts_s) > 0.0:
                self._arm_latency_sample_ms(
                    "client_to_host_ms",
                    (target_recv_s - float(client_ts_s)) * 1000.0,
                )
        merged_u = self._merge_partial_target_u(raw_u)
        if merged_u is None:
            return False, "empty_partial_u"
        partial_target_axes = set(self._pending_target_axes)
        q = proto.control_u_to_sim_q(merged_u, self.cfg)
        self._last_target_apply_error = ""
        log_key = "%s:true:%.4f:%.4f:%.4f:%.4f" % (
            source_s,
            float(q.linear_m),
            float(q.roll_rad),
            float(q.theta1_rad),
            float(q.theta2_rad),
        )
        now_log = time.time()
        if (
            log_key != self._last_target_log_key
            or (now_log - float(self._last_target_log_t)) >= 1.0
        ):
            self._last_target_log_key = log_key
            self._last_target_log_t = now_log
            print(
                "[host] direct target | seq=%d source=%s partial_u=true q=(%.4f, %.4f, %.4f, %.4f)"
                % (
                    int(seq),
                    source_s,
                    float(q.linear_m),
                    float(q.roll_rad),
                    float(q.theta1_rad),
                    float(q.theta2_rad),
                )
            )
        self._cancel_trajectory()
        submitted = self._submit_arm_servo_partial_target(
            merged_u,
            partial_target_axes,
            seq=int(seq),
            source=source_s,
            host_recv_s=float(target_recv_s),
            client_ts_s=float(client_ts_s),
            perceived_ts_s=float(self.last_perceived_timestamp_s),
        )
        self._pending_target_q = None
        self._pending_target_u = None
        self._pending_target_axes = set()
        self._pending_target_seq = -1
        if bool(submitted):
            reason = "direct_partial_submitted"
        else:
            reason = self._arm_servo_last_error or "direct_partial_submit_failed"
        if not self._has_hw():
            self.last_q = q
            self.last_u = merged_u
            self.last_state_ts = time.time()
        self._state_broadcast_requested.set()
        return bool(submitted), reason

    def _handle_direct_target_msg(self, msg: dict[str, Any]) -> tuple[bool, str]:
        source = str(msg.get("source", "target"))
        if not self._is_allowed_source(source):
            return False, "source_reject"
        if self._safety_fault:
            return False, str(self._safety_fault)
        raw_sim_target = msg.get("sim_target", None)
        if raw_sim_target is not None:
            if not (isinstance(raw_sim_target, (list, tuple)) and len(raw_sim_target) == 3):
                return False, "bad_sim_target"
            try:
                self.last_sim_target_xyz = (
                    float(raw_sim_target[0]),
                    float(raw_sim_target[1]),
                    float(raw_sim_target[2]),
                )
            except (TypeError, ValueError):
                return False, "bad_sim_target"
            self._state_broadcast_requested.set()
            return True, "sim_target"
        raw_debug_markers = msg.get("debug_markers", None)
        if isinstance(raw_debug_markers, list):
            ok, reason = self._update_external_debug_markers(raw_debug_markers)
            self._state_broadcast_requested.set()
            return bool(ok), str(reason)
        if "u" in msg and isinstance(msg.get("u"), dict):
            client_ts_s = 0.0
            try:
                client_ts_s = float(msg.get("ts", 0.0) or 0.0)
            except (TypeError, ValueError):
                client_ts_s = 0.0
            return self._submit_direct_partial_control_u(
                dict(msg.get("u", {})),
                source=source,
                seq=int(msg.get("seq", -1)),
                client_ts_s=float(client_ts_s),
            )
        target_raw = msg.get("target", None)
        if isinstance(target_raw, (list, tuple)) and len(target_raw) == 3:
            self.last_ik_target_xyz = (
                float(target_raw[0]),
                float(target_raw[1]),
                float(target_raw[2]),
            )
        target_dir_raw = msg.get("target_dir", None)
        if isinstance(target_dir_raw, (list, tuple)) and len(target_dir_raw) == 3:
            self.last_ik_target_dir = (
                float(target_dir_raw[0]),
                float(target_dir_raw[1]),
                float(target_dir_raw[2]),
            )
        ready_pose_dir_raw = msg.get("ready_pose_dir", None)
        if isinstance(ready_pose_dir_raw, (list, tuple)) and len(ready_pose_dir_raw) == 3:
            self.last_ready_pose_dir = (
                float(ready_pose_dir_raw[0]),
                float(ready_pose_dir_raw[1]),
                float(ready_pose_dir_raw[2]),
            )
        ready_pose_standoff_raw = msg.get("ready_pose_standoff_m", None)
        if ready_pose_standoff_raw is not None:
            try:
                self.last_ready_pose_standoff_m = max(0.0, float(ready_pose_standoff_raw))
            except (TypeError, ValueError):
                pass
        if ready_pose_dir_raw is not None or ready_pose_standoff_raw is not None:
            if self.last_perceived_object_world_xyz is not None:
                standoff = float(self.last_ready_pose_standoff_m)
                if standoff <= float(self.pick_config.grasp_standoff_m) + 1e-6:
                    self._set_grasp_target_markers(
                        self.last_perceived_object_world_xyz,
                        standoff_m=float(self.pick_config.grasp_standoff_m),
                        ttl_ms=30000,
                    )
                else:
                    self._set_ready_pose_markers(
                        self.last_perceived_object_world_xyz,
                        ttl_ms=30000,
                    )
        sag_raw = msg.get("sag_model", None)
        if isinstance(sag_raw, dict):
            self.last_sag_model = dict(sag_raw)
        if "claw_closed" in msg:
            self.last_claw_closed = bool(msg.get("claw_closed", False))
        if "go2_vel" in msg:
            try:
                next_vel = proto.unpack_go2_vel(msg.get("go2_vel"))
            except Exception:
                return False, "bad_go2_vel"
            lock_reason = self._go2_motion_lock_reason(next_vel)
            if lock_reason:
                self._clear_go2_vel()
                if self._go2_bridge is not None:
                    self._go2_bridge.set_velocity(0.0, 0.0, 0.0)
                return False, lock_reason
            self.last_go2_vel = next_vel
            self._last_go2_vel_ts = proto.now_s()
            if self._go2_bridge is not None:
                vx, vy, wz = self.last_go2_vel
                self._go2_bridge.set_velocity(vx, vy, wz)
        if "go2_sport_pose" in msg:
            try:
                pose = normalize_go2_sport_pose(proto.unpack_go2_sport_pose(msg.get("go2_sport_pose")))
            except Exception:
                return False, "bad_go2_sport_pose"
            if sport_pose_api_id(pose) is None:
                return False, f"unknown GO2 sport pose: {pose}"
            ok_pose, pose_reason = self._authorize_go2_sport_pose(pose)
            if not ok_pose:
                return False, pose_reason
            if self._go2_bridge is not None:
                try:
                    self._go2_bridge.call_sport_pose(pose)
                except ValueError as exc:
                    return False, str(exc)
            self._clear_go2_vel()
            self._note_go2_sport_pose(pose)
            self.last_go2_sport_pose = str(pose)
            self.last_go2_sport_pose_seq += 1
        if "go2_obstacles_avoid_enable" in msg:
            try:
                enabled = proto.unpack_go2_obstacles_avoid_enable(msg.get("go2_obstacles_avoid_enable"))
            except Exception:
                return False, "bad_go2_obstacles_avoid_enable"
            if self._go2_bridge is not None:
                self._go2_bridge.set_obstacles_avoid(enabled)
            self.last_go2_obstacles_avoid_enabled = bool(enabled)
            self.last_go2_obstacles_avoid_seq += 1

        q: Optional[proto.SimQ] = None
        if "q" in msg:
            q = proto.unpack_q(msg["q"])
        elif "u" in msg and isinstance(msg.get("u"), dict):
            q = proto.control_u_to_sim_q(proto.unpack_u(msg["u"]), self.cfg)
        if q is None:
            if (
                target_raw is None
                and target_dir_raw is None
                and ready_pose_dir_raw is None
                and ready_pose_standoff_raw is None
                and sag_raw is None
                and "claw_closed" not in msg
                and "go2_vel" not in msg
                and "go2_sport_pose" not in msg
                and "go2_obstacles_avoid_enable" not in msg
            ):
                return False, "bad_target"
            self._state_broadcast_requested.set()
            return True, ""

        seq = int(msg.get("seq", -1))
        self._clear_arm_servo_target()
        self._pending_target_u = None
        self._pending_target_axes = set()
        self._target_u_state = proto.sim_q_to_control_u(q, self.cfg)
        self._pending_target_q = q
        self._pending_target_seq = int(seq)
        self._last_target_apply_error = ""
        self._schedule_target_motion(q, source=source)
        if (
            str(source).strip().lower() in {"lji", "lji_step"}
            and self._has_hw()
            and not self._use_trajectory_for_source(source)
        ):
            applied_hw, complete = self._apply_sim_q_target(q)
            if applied_hw:
                self._target_u_state = proto.sim_q_to_control_u(q, self.cfg)
                if bool(complete):
                    self._pending_target_q = None
            self._state_broadcast_requested.set()
            return bool(applied_hw), self._last_target_apply_error or "direct_lji_step"
        if not self._has_hw():
            self.last_q = q
            self.last_u = proto.sim_q_to_control_u(q, self.cfg)
            self.last_state_ts = time.time()
            self._pending_target_q = None
            self._cancel_trajectory()
        self._state_broadcast_requested.set()
        return True, self._last_target_apply_error or ""

    def _start_arm_servo_thread(self) -> None:
        if not bool(self._arm_servo_thread_enable) or not self._has_hw():
            reason = "disabled" if not bool(self._arm_servo_thread_enable) else "no_hw"
            self._arm_servo_last_error = f"arm_servo_not_started:{reason}"
            if bool(self._arm_latency_log_enable) and not bool(self._arm_servo_start_skip_logged):
                print(
                    "[host] low-latency arm servo thread skipped | "
                    f"reason={reason} enable={bool(self._arm_servo_thread_enable)} hw={self._has_hw()} "
                    f"device={self.device or '-'}"
                )
                self._arm_servo_start_skip_logged = True
            return
        if self._arm_servo_thread is not None and self._arm_servo_thread.is_alive():
            return
        self._arm_servo_stop.clear()
        self._arm_servo_thread = threading.Thread(
            target=self._arm_servo_loop,
            name="arm-servo",
            daemon=True,
        )
        self._arm_servo_thread.start()
        print(
            "[host] low-latency arm servo thread started | hz=%.1f"
            % float(1.0 / max(float(self._arm_servo_period), 1e-6))
        )
        self._arm_servo_last_error = ""

    def _stop_arm_servo_thread(self) -> None:
        self._arm_servo_stop.set()
        with self._arm_servo_cond:
            self._arm_servo_target_u = None
            self._arm_servo_target_axes = set()
            self._arm_servo_target_seq = -1
            self._arm_servo_cond.notify_all()
        worker = self._arm_servo_thread
        if worker is not None and worker.is_alive() and threading.current_thread() is not worker:
            worker.join(timeout=1.0)
        self._arm_servo_thread = None

    def _clear_arm_servo_target(self) -> None:
        with self._arm_servo_cond:
            self._arm_servo_target_u = None
            self._arm_servo_target_axes = set()
            self._arm_servo_target_seq = -1
            self._arm_servo_target_source = ""
            self._arm_servo_target_meta = {}
            self._arm_servo_cond.notify_all()

    def _submit_arm_servo_partial_target(
        self,
        u: Optional[proto.ControlU],
        axes: Set[str],
        *,
        seq: int,
        source: str,
        host_recv_s: float,
        client_ts_s: float,
        perceived_ts_s: float,
    ) -> bool:
        if (
            bool(self._arm_servo_thread_enable)
            and self._has_hw()
            and (self._arm_servo_thread is None or not self._arm_servo_thread.is_alive())
        ):
            self._start_arm_servo_thread()
        reject_reason = ""
        if not bool(self._arm_servo_thread_enable):
            reject_reason = "disabled"
        elif not self._has_hw():
            reject_reason = "no_hw"
        elif self._arm_servo_thread is None:
            reject_reason = "thread_none"
        elif not self._arm_servo_thread.is_alive():
            reject_reason = "thread_dead"
        elif u is None:
            reject_reason = "no_target_u"
        elif not axes:
            reject_reason = "no_axes"
        if reject_reason:
            self._arm_servo_last_error = f"arm_servo_submit_reject:{reject_reason}"
            self._arm_latency_count("submit_reject")
            return False
        with self._arm_servo_cond:
            if self._arm_servo_target_u is not None and self._arm_servo_target_seq != self._arm_servo_applied_seq:
                self._arm_latency_count("replace")
            self._arm_servo_target_u = u
            self._arm_servo_target_axes = set(axes)
            self._arm_servo_target_seq = int(seq)
            self._arm_servo_target_source = str(source)
            self._arm_servo_target_meta = {
                "host_recv_s": float(host_recv_s),
                "client_ts_s": float(client_ts_s),
                "perceived_ts_s": float(perceived_ts_s),
                "source": str(source),
            }
            self._arm_servo_submit_count += 1
            self._arm_latency_count("submit")
            self._arm_servo_cond.notify()
        return True

    def _arm_servo_loop(self) -> None:
        last_seq = -1
        next_apply_t = 0.0
        while not self._stop_event.is_set() and not self._arm_servo_stop.is_set():
            with self._arm_servo_cond:
                if self._arm_servo_target_u is None or self._arm_servo_target_seq == last_seq:
                    self._arm_servo_cond.wait(timeout=float(self._arm_servo_period))
                target_u = self._arm_servo_target_u
                axes = set(self._arm_servo_target_axes)
                seq = int(self._arm_servo_target_seq)
                meta = dict(self._arm_servo_target_meta)
            if target_u is None or not axes or seq == last_seq:
                continue
            now = time.time()
            delay = float(next_apply_t - now)
            if delay > 0.0:
                time.sleep(min(delay, float(self._arm_servo_period)))
            with self._arm_servo_cond:
                target_u = self._arm_servo_target_u
                axes = set(self._arm_servo_target_axes)
                seq = int(self._arm_servo_target_seq)
                meta = dict(self._arm_servo_target_meta)
            if target_u is None or not axes or seq == last_seq:
                continue
            if self._safety_fault:
                with self._arm_servo_cond:
                    self._arm_servo_last_error = str(self._safety_fault)
                    self._arm_servo_applied_seq = int(seq)
                last_seq = int(seq)
                continue
            actual_before = self.last_u
            apply_start = time.time()
            ok = self._apply_partial_u_target(target_u, axes)
            apply_end = time.time()
            self._arm_latency_count("apply")
            host_recv_s = float(meta.get("host_recv_s", 0.0))
            client_ts_s = float(meta.get("client_ts_s", 0.0))
            perceived_ts_s = float(meta.get("perceived_ts_s", 0.0))
            if host_recv_s > 0.0:
                self._arm_latency_sample_ms("recv_to_write_ms", (apply_start - host_recv_s) * 1000.0)
            if client_ts_s > 0.0:
                self._arm_latency_sample_ms("client_to_write_ms", (apply_end - client_ts_s) * 1000.0)
            if perceived_ts_s > 0.0:
                self._arm_latency_sample_ms("obs_age_ms", (apply_start - perceived_ts_s) * 1000.0)
            self._arm_latency_sample_ms("sync_write_ms", (apply_end - apply_start) * 1000.0)
            with self._arm_servo_cond:
                self._arm_servo_applied_seq = int(seq)
                if bool(ok):
                    self._arm_servo_apply_count += 1
                    self._arm_servo_last_error = ""
                else:
                    self._arm_servo_last_error = self._last_target_apply_error or "arm_servo_apply_failed"
            if bool(ok):
                if isinstance(actual_before, proto.ControlU):
                    initial_err = self._control_u_err_max(target_u, actual_before, axes)
                else:
                    initial_err = 0.0
                self._arm_latency_last_follow = {
                    "seq": int(seq),
                    "source": str(meta.get("source", self._arm_servo_target_source)),
                    "target_u": target_u,
                    "axes": set(axes),
                    "apply_s": float(apply_end),
                    "initial_err_u": float(initial_err),
                    "last_err_u": float(initial_err),
                    "hit50": False,
                    "hit90": False,
                }
                self._state_broadcast_requested.set()
                self._arm_latency_maybe_log(now_s=float(apply_end))
            last_seq = int(seq)
            next_apply_t = time.time() + float(self._arm_servo_period)

    def torque_on(self, *, configure_modes: bool = True, set_profiles: bool = True, go_mid: bool = False) -> None:
        if not self._has_hw():
            raise RuntimeError("no device selected")
        with self._hw_lock:
            if self.torque_enabled and not self._safety_fault and not self._red_torque_off_ids:
                return
            if configure_modes:
                self.hw.set_operating_modes()
            if set_profiles:
                self.hw.set_profiles()
            self.hw.torque_on_all()
            self.torque_enabled = True
            self._safety_fault = ""
            self._red_torque_off_ids = set()
            if go_mid:
                self.hw.go_mid_pose()

    def torque_off(self) -> None:
        if not self._has_hw():
            raise RuntimeError("no device selected")
        self._clear_arm_servo_target()
        with self._hw_lock:
            self._pending_target_q = None
            self._pending_target_seq = -1
            self._cancel_trajectory()
            self.hw.torque_off_all()
            self.torque_enabled = False
            self._red_torque_off_ids = set()

    def _perception_state_payload(self) -> Dict[str, Any]:
        with self._perception_lock:
            cap = self._perception_capture
            recording = bool(cap.is_recording()) if cap is not None else False
            active_record_path = cap.recording_path() if recording and cap is not None else ""
            last_record_path = active_record_path or str(self.perception_last_record_path)
            last_capture_path = str(self.perception_last_capture_path)
            record_with_overlay = bool(self.perception_record_with_overlay)
        payload: Dict[str, Any] = {
            "perception_running": bool(self.perception_running),
            "perception_failed": bool(self.perception_failed),
            "perception_status": str(self.perception_status),
            "perception_source": str(self.perception_source),
            "perception_preview_endpoint": str(getattr(self.perception_config, "preview_bind", "")),
            "perception_recording": bool(recording),
            "perception_record_with_overlay": bool(record_with_overlay),
            "perception_hz": float(self.perception_hz),
        }
        if last_record_path:
            payload["perception_last_record_path"] = str(last_record_path)
        if last_capture_path:
            payload["perception_last_capture_path"] = str(last_capture_path)
        if self.last_perceived_center_uv is not None:
            payload["perceived_center_uv"] = [
                float(self.last_perceived_center_uv[0]),
                float(self.last_perceived_center_uv[1]),
            ]
        if self.last_perceived_scale is not None:
            payload["perceived_scale"] = float(self.last_perceived_scale)
        if float(self.last_perceived_timestamp_s) > 0.0:
            payload["perceived_timestamp_s"] = float(self.last_perceived_timestamp_s)
        payload["perceived_object_label"] = str(self.last_perceived_object_label)
        payload["perceived_object_confidence"] = float(self.last_perceived_object_confidence)
        if self.last_perceived_object_camera_xyz is not None:
            p_cam = self.last_perceived_object_camera_xyz
            payload["perceived_object_camera"] = [float(p_cam[0]), float(p_cam[1]), float(p_cam[2])]
        if self.last_perceived_object_world_xyz is not None:
            p_w = self.last_perceived_object_world_xyz
            payload["object_world"] = [float(p_w[0]), float(p_w[1]), float(p_w[2])]
        return payload

    def _remote_perception_config(self, raw: Any = None) -> PerceptionConfig:
        cfg = self.perception_config
        if not isinstance(raw, dict):
            return replace(cfg, mode="camera", run_local=True, provider="local", show_preview=False)
        updates: Dict[str, Any] = {
            "mode": "camera",
            "run_local": True,
            "provider": "local",
            "show_preview": False,
        }
        for key in (
            "detector_config",
            "detector",
            "target_label",
            "yolo_device",
            "pipeline",
            "tracker",
        ):
            value = raw.get(key, None)
            if value is not None and str(value).strip():
                updates[key] = str(value).strip()
        if raw.get("publish_hz", None) is not None:
            try:
                hz = float(raw.get("publish_hz"))
                if hz > 0.0:
                    updates["publish_hz"] = hz
            except (TypeError, ValueError):
                pass
        return replace(cfg, **updates)

    def _on_perception_snapshot(self, snap: PerceptionSnapshot) -> None:
        with self._perception_lock:
            self.perception_running = bool(snap.running)
            self.perception_failed = bool(snap.failed)
            self.perception_status = str(snap.status_msg)
            frame_idx = int(snap.frame_idx)
            now = time.monotonic()
            if not bool(snap.running) or bool(snap.failed):
                self.perception_hz = 0.0
                self._perception_rate_last_t = 0.0
                self._perception_rate_last_frame_idx = -1
            elif self._perception_rate_last_frame_idx < 0 or frame_idx <= self._perception_rate_last_frame_idx:
                self.perception_hz = 0.0
                self._perception_rate_last_t = now
                self._perception_rate_last_frame_idx = frame_idx
            else:
                dt = max(1e-6, now - float(self._perception_rate_last_t))
                frames = max(1, frame_idx - int(self._perception_rate_last_frame_idx))
                inst_hz = float(frames) / dt
                prev = float(self.perception_hz)
                self.perception_hz = inst_hz if prev <= 0.0 else (0.75 * prev + 0.25 * inst_hz)
                self._perception_rate_last_t = now
                self._perception_rate_last_frame_idx = frame_idx

    def capture_perception_worker_frame(self, *, include_overlay: bool = True) -> tuple[bool, str, str]:
        with self._perception_lock:
            cap = self._perception_capture
        if cap is None or not cap.is_running():
            return False, "", "perception_not_running"
        path = cap.save_cached_frames(
            default_perception_capture_dir(),
            extra_meta={
                "mode": str(getattr(self.perception_config, "mode", "camera")),
                "source": "host",
                "include_overlay_requested": bool(include_overlay),
            },
        )
        if path is None:
            with self._perception_lock:
                self.perception_failed = True
                self.perception_status = "capture failed: no cached frame"
            return False, "", "perception_no_cached_frame"
        path_s = str(path.resolve())
        with self._perception_lock:
            self.perception_last_capture_path = path_s
            self.perception_failed = False
            self.perception_status = f"snapshot saved: {path_s}"
        print(f"[perception] snapshot saved: {path_s}")
        return True, path_s, "perception_capture"

    def start_perception_worker_recording(
        self,
        *,
        include_overlay: bool = False,
        fps: Optional[float] = None,
    ) -> tuple[bool, str, str]:
        with self._perception_lock:
            cap = self._perception_capture
        if cap is None or not cap.is_running():
            return False, "", "perception_not_running"
        record_fps = float(fps) if fps is not None and float(fps) > 0.0 else float(getattr(self.perception_config, "publish_hz", 20.0))
        ok, path_s = cap.start_recording(
            default_perception_capture_dir(),
            fps=record_fps,
            include_overlay=bool(include_overlay),
        )
        if not ok:
            return False, str(path_s), "perception_recording_active"
        with self._perception_lock:
            self.perception_last_record_path = str(path_s)
            self.perception_record_with_overlay = bool(include_overlay)
            self.perception_failed = False
            self.perception_status = f"recording started: {path_s}"
        print(
            "[perception] recording started on host (%s): %s"
            % ("overlay" if include_overlay else "raw", str(path_s))
        )
        return True, str(path_s), "perception_record_start"

    def stop_perception_worker_recording(self) -> tuple[bool, str, int, str]:
        with self._perception_lock:
            cap = self._perception_capture
        if cap is None:
            return False, "", 0, "perception_not_running"
        ok, path_s, frame_count = cap.stop_recording()
        if not ok:
            return False, str(path_s), int(frame_count), "perception_recording_inactive"
        with self._perception_lock:
            self.perception_last_record_path = str(path_s)
            self.perception_failed = False
            self.perception_status = f"recording saved ({int(frame_count)}f): {path_s}"
        print(f"[perception] recording saved on host ({int(frame_count)}f): {path_s}")
        return True, str(path_s), int(frame_count), "perception_record_stop"

    def _publish_preview_frame(self, image_bgr: Any, *, meta: Optional[dict[str, Any]] = None) -> None:
        publisher = self._preview_publisher
        if publisher is None:
            return
        was_empty = int(getattr(publisher, "published", 0)) <= 0
        try:
            publisher.publish(image_bgr, meta=meta)
            if was_empty:
                try:
                    h, w = image_bgr.shape[:2]
                    print(f"[preview_stream] first frame queued {int(w)}x{int(h)} meta={dict(meta or {})}")
                except Exception:
                    print("[preview_stream] first frame queued")
        except Exception as exc:
            print(f"[preview_stream] publish failed: {exc}")

    def _publish_perception_observation_from_worker(
        self,
        *,
        object_camera_xyz: tuple[float, float, float],
        label: str,
        confidence: float,
        image_center_uv: tuple[float, float],
        image_scale: float,
        depth_valid: bool = True,
        object_world: Optional[tuple[float, float, float]] = None,
        camera_world_origin: Optional[tuple[float, float, float]] = None,
        camera_world_look: Optional[tuple[float, float, float]] = None,
        camera_world_right: Optional[tuple[float, float, float]] = None,
    ) -> Optional[tuple[float, float, float]]:
        with self._perception_lock:
            self.last_perceived_center_uv = (
                float(image_center_uv[0]),
                float(image_center_uv[1]),
            )
            self.last_perceived_scale = float(image_scale)
            self.last_perceived_object_confidence = float(confidence)
            self.last_perceived_object_label = str(label)
            self.last_perceived_timestamp_s = float(proto.now_s())
            self.last_perceived_object_camera_xyz = (
                float(object_camera_xyz[0]),
                float(object_camera_xyz[1]),
                float(object_camera_xyz[2]),
            )

            result_world: Optional[tuple[float, float, float]] = None
            if object_world is not None:
                p_w = np.asarray(object_world, dtype=float).reshape(3)
                result_world = (float(p_w[0]), float(p_w[1]), float(p_w[2]))
                self.last_perceived_object_world_xyz = result_world
                has_frame_cam_pose = (
                    camera_world_origin is not None
                    and camera_world_look is not None
                    and camera_world_right is not None
                )
                self._set_perception_debug_markers(
                    object_world=result_world,
                    object_label=str(label),
                    object_camera_xyz=self.last_perceived_object_camera_xyz,
                    world_tag="sim_frame_pose" if has_frame_cam_pose else "worker_world",
                    camera_world=camera_world_origin,
                    camera_look=camera_world_look,
                    camera_right=camera_world_right,
                    ttl_ms=3000,
                )
            elif bool(depth_valid):
                ok, _reason, p_w = self._update_perception_markers(
                    self.last_perceived_object_camera_xyz,
                    object_label=str(label),
                )
                if bool(ok) and p_w is not None:
                    arr = np.asarray(p_w, dtype=float).reshape(3)
                    result_world = (float(arr[0]), float(arr[1]), float(arr[2]))
            else:
                result_world = self.last_perceived_object_world_xyz
            self._state_broadcast_requested.set()
            return result_world

    def start_perception_worker(self, *, config: Optional[PerceptionConfig] = None) -> bool:
        with self._perception_lock:
            old = self._perception_capture
            if old is not None and old.is_running():
                self.perception_running = True
                self.perception_failed = False
                self.perception_status = "already running"
                return True
            if old is not None:
                self._perception_capture = None
            cfg = config or self._remote_perception_config()
            self.perception_config = cfg
            self.perception_running = True
            self.perception_failed = False
            self.perception_status = "starting"
            self.perception_source = "host"
            self.perception_hz = 0.0
            self._perception_rate_last_t = 0.0
            self._perception_rate_last_frame_idx = -1
            cap = PerceptionCapture(
                cfg,
                publish_fn=self._publish_perception_observation_from_worker,
                on_snapshot=self._on_perception_snapshot,
                preview_publish_fn=self._publish_preview_frame,
            )
            self._perception_capture = cap
            cap.start()
            return True

    def stop_perception_worker(self, *, timeout_s: float = 5.0) -> bool:
        with self._perception_lock:
            cap = self._perception_capture
        if cap is None:
            with self._perception_lock:
                self.perception_running = False
                self.perception_failed = False
                self.perception_status = "stopped"
                self.perception_hz = 0.0
                self._perception_rate_last_t = 0.0
                self._perception_rate_last_frame_idx = -1
            return True
        active_record_path = cap.recording_path() if cap.is_recording() else ""
        stopped = cap.stop(timeout_s=float(timeout_s))
        with self._perception_lock:
            if active_record_path:
                self.perception_last_record_path = str(active_record_path)
            if stopped:
                self._perception_capture = None
                self.perception_running = False
                self.perception_failed = False
                self.perception_status = "stopped"
                self.perception_hz = 0.0
                self._perception_rate_last_t = 0.0
                self._perception_rate_last_frame_idx = -1
            else:
                self.perception_running = False
                self.perception_failed = True
                self.perception_status = "stop pending"
                self.perception_hz = 0.0
        return bool(stopped)

    def refresh_perception_worker(self) -> bool:
        with self._perception_lock:
            cap = self._perception_capture
        if cap is None or not cap.is_running():
            with self._perception_lock:
                self.perception_running = False
                self.perception_failed = True
                self.perception_status = "not running"
            return False
        ok = cap.request_refresh()
        with self._perception_lock:
            self.perception_running = True
            self.perception_failed = not bool(ok)
            self.perception_status = "refresh requested" if ok else "refresh rejected"
        return bool(ok)

    def close(self) -> None:
        self._stop_event.set()
        try:
            self._stop_arm_servo_thread()
        except Exception:
            pass
        try:
            self._close_on_device_control_service()
        except Exception:
            pass
        try:
            self.stop_perception_worker(timeout_s=2.0)
        except Exception:
            pass
        if self._preview_publisher is not None:
            try:
                self._preview_publisher.close()
            except Exception:
                pass
        if self._go2_bridge is not None:
            try:
                self._go2_bridge.stop()
            except Exception:
                pass
        if self._arm_latency_csv_file is not None:
            try:
                self._arm_latency_csv_file.close()
            except Exception:
                pass
            self._arm_latency_csv_file = None
            self._arm_latency_csv_writer = None
        try:
            self.poller.unregister(self.sock)
        except Exception:
            pass
        try:
            self.sock.close(0)
        except Exception:
            pass
        try:
            self.sim_pub.close(0)
        except Exception:
            pass
        try:
            self.sim_feedback.close(0)
        except Exception:
            pass

    def _handle_sim_feedback(self, msg: Dict[str, Any]) -> None:
        if str(msg.get("t", "")).lower() != "sim_state":
            return
        sim_q_raw = msg.get("sim_q", msg.get("q", None))
        if sim_q_raw is not None:
            try:
                self.last_sim_q = proto.unpack_q(sim_q_raw)
                self.last_sim_u = proto.sim_q_to_control_u(self.last_sim_q, self.cfg)
                self.last_sim_state_ts = float(msg.get("ts", proto.now_s()))
            except (TypeError, ValueError):
                pass
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
        rpy_raw = msg.get("go2_base_rpy", None)
        if self._go2_bridge is None and isinstance(rpy_raw, (list, tuple)) and len(rpy_raw) == 3:
            self.last_go2_base_rpy = (float(rpy_raw[0]), float(rpy_raw[1]), float(rpy_raw[2]))
        pos_raw = msg.get("go2_base_pos", None)
        if self._go2_bridge is None and isinstance(pos_raw, (list, tuple)) and len(pos_raw) == 3:
            self.last_go2_base_pos = (float(pos_raw[0]), float(pos_raw[1]), float(pos_raw[2]))
        lin_raw = msg.get("go2_base_lin_vel_body", None)
        if self._go2_bridge is None and isinstance(lin_raw, (list, tuple)) and len(lin_raw) == 3:
            self.last_go2_base_lin_vel_body = (float(lin_raw[0]), float(lin_raw[1]), float(lin_raw[2]))
        ang_raw = msg.get("go2_base_ang_vel", None)
        if self._go2_bridge is None and isinstance(ang_raw, (list, tuple)) and len(ang_raw) == 3:
            self.last_go2_base_ang_vel = (float(ang_raw[0]), float(ang_raw[1]), float(ang_raw[2]))
        if self._go2_bridge is None and "go2_base_timestamp_s" in msg:
            try:
                self.last_go2_base_timestamp_s = float(msg.get("go2_base_timestamp_s", 0.0))
            except (TypeError, ValueError):
                pass
        if self._go2_bridge is None:
            leg_q_raw = msg.get("go2_leg_q", None)
            if isinstance(leg_q_raw, (list, tuple)) and len(leg_q_raw) == 12:
                self.last_go2_leg_q = tuple(float(v) for v in leg_q_raw)
            leg_dq_raw = msg.get("go2_leg_dq", None)
            if isinstance(leg_dq_raw, (list, tuple)) and len(leg_dq_raw) == 12:
                self.last_go2_leg_dq = tuple(float(v) for v in leg_dq_raw)
            leg_torque_raw = msg.get("go2_leg_torque_nm", None)
            if isinstance(leg_torque_raw, (list, tuple)) and len(leg_torque_raw) == 12:
                self.last_go2_leg_torque_nm = tuple(float(v) for v in leg_torque_raw)
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
        cam_origin_raw = msg.get("camera_world_origin", None)
        if isinstance(cam_origin_raw, (list, tuple)) and len(cam_origin_raw) == 3:
            self._sim_camera_origin = (float(cam_origin_raw[0]), float(cam_origin_raw[1]), float(cam_origin_raw[2]))
            self._sim_camera_ts = float(msg.get("ts", proto.now_s()))
        cam_look_raw = msg.get("camera_world_look", None)
        if isinstance(cam_look_raw, (list, tuple)) and len(cam_look_raw) == 3:
            self._sim_camera_look = (float(cam_look_raw[0]), float(cam_look_raw[1]), float(cam_look_raw[2]))
        cam_right_raw = msg.get("camera_world_right", None)
        if isinstance(cam_right_raw, (list, tuple)) and len(cam_right_raw) == 3:
            self._sim_camera_right = (float(cam_right_raw[0]), float(cam_right_raw[1]), float(cam_right_raw[2]))

    def _handle_msg(self, ident: bytes, msg: Dict[str, Any]) -> None:
        self.clients.add(ident)
        t = str(msg.get("t", "")).lower()
        if t in ("hello", "hi"):
            self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": True, "device": self.device, "torque_enabled": self.torque_enabled})
            return
        if t == "estop":
            ok = True
            try:
                self.torque_off()
            except Exception:
                ok = False
            self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": ok, "device": self.device, "torque_enabled": self.torque_enabled})
            return
        if t == "torque_on":
            ok = True
            try:
                resume = bool(msg.get("resume", False))
                self.torque_on(configure_modes=not resume, set_profiles=not resume)
            except Exception:
                ok = False
            self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": ok, "device": self.device, "torque_enabled": self.torque_enabled})
            return
        if t == "torque_off":
            ok = True
            try:
                self.torque_off()
            except Exception:
                ok = False
            self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": ok, "device": self.device, "torque_enabled": self.torque_enabled})
            return
        if t == "sim_reset":
            ok = True
            reason = "sim_reset"
            try:
                if self._has_hw():
                    ok = False
                    reason = "sim_reset_hw_unsupported"
                else:
                    self._reset_simulation_state()
                    self._broadcast_state_now()
            except Exception as exc:
                ok = False
                reason = f"sim_reset_failed:{exc}"
            self._reply(
                ident,
                {
                    "t": "ack",
                    "ts": proto.now_s(),
                    "ok": bool(ok),
                    "reason": str(reason),
                    "device": self.device,
                    "torque_enabled": self.torque_enabled,
                    "sim_reset_seq": int(self._sim_reset_seq),
                },
            )
            return
        if t == "ports":
            ports = self._list_ports()
            ports_text = ", ".join(ports) if ports else "None"
            print(f"[host] ports searched: {ports_text}")
            self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": True, "device": self.device, "ports": ports, "reason": "ports", "torque_enabled": self.torque_enabled})
            return
        if t == "set_device":
            device = str(msg.get("device", "")).strip()
            ok = True
            reason = f"device set to {device}" if device else "device unchanged"
            try:
                self.set_device(device)
            except Exception as exc:
                ok = False
                reason = str(exc)
            self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": ok, "device": self.device, "ports": self._list_ports(), "reason": reason, "torque_enabled": self.torque_enabled})
            return
        if t == "disconnect_device":
            ok = True
            reason = "device disconnected"
            try:
                self.clear_device()
            except Exception as exc:
                ok = False
                reason = str(exc)
            self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": ok, "device": self.device, "ports": self._list_ports(), "reason": reason, "torque_enabled": self.torque_enabled})
            return
        if t == "gaze_config_update":
            ok = True
            reason = "gaze_config_updated"
            try:
                patch = msg.get("config", {})
                self.gaze_config = patch_gaze_config(self._effective_gaze_config(), patch)
                service = self._embedded_control_service
                if service is not None:
                    service.update_gaze_stabilizer_config(self.gaze_config, send_remote=False)
                print(
                    "[gaze] config updated | "
                    "uv_gain=%.2f center=(%.1f,%.1f) cap=%.1f tick=%.1f settle=%.3f mode=%s"
                    % (
                        float(self.gaze_config.uv_gain),
                        float(self.gaze_config.center_u_gain),
                        float(self.gaze_config.center_v_gain),
                        float(self.gaze_config.center_seg_max),
                        float(self.gaze_config.max_seg_du_per_tick),
                        float(self.gaze_config.cmd_settle_s),
                        str(self.gaze_config.walking_gaze_mode),
                    )
                )
            except Exception as exc:
                ok = False
                reason = f"gaze_config_update_failed:{exc}"
            self._reply(
                ident,
                {
                    "t": "ack",
                    "ts": proto.now_s(),
                    "ok": bool(ok),
                    "reason": str(reason),
                    "device": self.device,
                    "torque_enabled": self.torque_enabled,
                }
                | self._gaze_state_payload(),
            )
            self._broadcast_state_now()
            return
        if t == "gaze_start_standing":
            ok = True
            reason = "on_device_gaze_standing"
            try:
                service = self._ensure_on_device_control_service()
                service.start_gaze_stabilizer_standing(run_id=str(msg.get("run_id", "")))
                ok = bool(service.state.gaze_running)
                reason = str(service.state.gaze_status_msg or reason)
            except Exception as exc:
                ok = False
                reason = f"on_device_gaze_start_failed:{exc}"
            self._reply(
                ident,
                {
                    "t": "ack",
                    "ts": proto.now_s(),
                    "ok": bool(ok),
                    "reason": str(reason),
                    "device": self.device,
                    "torque_enabled": self.torque_enabled,
                }
                | self._gaze_state_payload(),
            )
            self._broadcast_state_now()
            return
        if t == "gaze_start_walking":
            ok = True
            mode = str(msg.get("gaze_mode", "")).strip()
            reason = "on_device_gaze_walking"
            try:
                if self._go2_requires_user_stand() and not bool(self._go2_motion_armed):
                    ok = False
                    reason = "go2 walking gaze locked: press Stand first"
                else:
                    service = self._ensure_on_device_control_service()
                    service.start_gaze_stabilizer_walking(
                        run_id=str(msg.get("run_id", "")),
                        gaze_mode=mode or None,
                    )
                    ok = bool(service.state.gaze_running)
                    reason = str(service.state.gaze_status_msg or reason)
            except Exception as exc:
                ok = False
                reason = f"on_device_gaze_start_failed:{exc}"
            self._reply(
                ident,
                {
                    "t": "ack",
                    "ts": proto.now_s(),
                    "ok": bool(ok),
                    "reason": str(reason),
                    "device": self.device,
                    "torque_enabled": self.torque_enabled,
                }
                | self._gaze_state_payload(),
            )
            self._broadcast_state_now()
            return
        if t == "gaze_stop":
            ok = True
            reason = "on_device_gaze_stop"
            try:
                self._stop_on_device_gaze()
            except Exception as exc:
                ok = False
                reason = f"on_device_gaze_stop_failed:{exc}"
            self._reply(
                ident,
                {
                    "t": "ack",
                    "ts": proto.now_s(),
                    "ok": bool(ok),
                    "reason": str(reason),
                    "device": self.device,
                    "torque_enabled": self.torque_enabled,
                }
                | self._gaze_state_payload(),
            )
            self._broadcast_state_now()
            return
        if t == "mobile_pick_start":
            ok = True
            reason = "on_device_mobile_pick_start"
            try:
                if self._go2_requires_user_stand() and not bool(self._go2_motion_armed):
                    ok = False
                    reason = "go2 mobile pick locked: press Stand first"
                else:
                    service = self._ensure_on_device_control_service()
                    service.start_mobile_gaze_lji_pick_e2e()
            except Exception as exc:
                ok = False
                reason = f"on_device_mobile_pick_start_failed:{exc}"
            ack = {
                "t": "ack",
                "ts": proto.now_s(),
                "ok": bool(ok),
                "reason": str(reason),
                "device": self.device,
                "torque_enabled": self.torque_enabled,
            }
            ack.update(self._pick_state_payload())
            ack.update(self._gaze_state_payload())
            ack.update(self._perception_state_payload())
            self._reply(ident, ack)
            self._broadcast_state_now()
            return
        if t == "lji_grasp_start":
            ok = True
            reason = "on_device_lji_grasp_start"
            try:
                service = self._ensure_on_device_control_service()
                service.start_lji_grasp_only()
                ok = not bool(service.state.pick_failed)
                reason = str(service.state.pick_status_msg or reason)
            except Exception as exc:
                ok = False
                reason = f"on_device_lji_grasp_start_failed:{exc}"
            ack = {
                "t": "ack",
                "ts": proto.now_s(),
                "ok": bool(ok),
                "reason": str(reason),
                "device": self.device,
                "torque_enabled": self.torque_enabled,
            }
            ack.update(self._pick_state_payload())
            ack.update(self._gaze_state_payload())
            ack.update(self._perception_state_payload())
            self._reply(ident, ack)
            self._broadcast_state_now()
            return
        if t == "mobile_pick_stop" or t == "pick_stop":
            ok = True
            reason = "on_device_pick_stop"
            try:
                service = self._embedded_control_service
                if service is not None:
                    service.stop_pick_e2e()
                else:
                    reason = "on_device_pick_not_running"
            except Exception as exc:
                ok = False
                reason = f"on_device_pick_stop_failed:{exc}"
            ack = {
                "t": "ack",
                "ts": proto.now_s(),
                "ok": bool(ok),
                "reason": str(reason),
                "device": self.device,
                "torque_enabled": self.torque_enabled,
            }
            ack.update(self._pick_state_payload())
            ack.update(self._gaze_state_payload())
            ack.update(self._perception_state_payload())
            self._reply(ident, ack)
            self._broadcast_state_now()
            return
        if t == "perception_start":
            ok = True
            reason = "perception_start"
            try:
                cfg = self._remote_perception_config(msg.get("config", None))
                print(
                    "[perception] start requested | mode=%s detector=%s provider=%s preview=%s yolo_device=%s"
                    % (
                        str(getattr(cfg, "mode", "")),
                        str(getattr(cfg, "detector", "")),
                        str(getattr(cfg, "provider", "")),
                        str(getattr(cfg, "preview_bind", "")),
                        str(getattr(cfg, "yolo_device", "")),
                    )
                )
                ok = self.start_perception_worker(config=cfg)
            except Exception as exc:
                ok = False
                reason = f"perception_start_failed:{exc}"
                print(f"[perception] start failed: {exc}")
                with self._perception_lock:
                    self.perception_running = False
                    self.perception_failed = True
                    self.perception_status = str(reason)
            else:
                print(f"[perception] start ack | ok={bool(ok)} reason={reason}")
            ack = {
                "t": "ack",
                "ts": proto.now_s(),
                "ok": bool(ok),
                "reason": str(reason),
                "device": self.device,
                "torque_enabled": self.torque_enabled,
            }
            ack.update(self._perception_state_payload())
            self._reply(ident, ack)
            self._broadcast_state_now()
            return
        if t == "perception_stop":
            ok = True
            reason = "perception_stop"
            try:
                ok = self.stop_perception_worker()
                if not ok:
                    reason = "perception_stop_pending"
            except Exception as exc:
                ok = False
                reason = f"perception_stop_failed:{exc}"
            ack = {
                "t": "ack",
                "ts": proto.now_s(),
                "ok": bool(ok),
                "reason": str(reason),
                "device": self.device,
                "torque_enabled": self.torque_enabled,
            }
            ack.update(self._perception_state_payload())
            self._reply(ident, ack)
            self._broadcast_state_now()
            return
        if t == "perception_refresh":
            ok = self.refresh_perception_worker()
            ack = {
                "t": "ack",
                "ts": proto.now_s(),
                "ok": bool(ok),
                "reason": "perception_refresh" if ok else "perception_not_running",
                "device": self.device,
                "torque_enabled": self.torque_enabled,
            }
            ack.update(self._perception_state_payload())
            self._reply(ident, ack)
            self._broadcast_state_now()
            return
        if t == "perception_capture":
            ok, path_s, reason = self.capture_perception_worker_frame(
                include_overlay=bool(msg.get("include_overlay", True))
            )
            ack = {
                "t": "ack",
                "ts": proto.now_s(),
                "ok": bool(ok),
                "reason": str(reason),
                "device": self.device,
                "torque_enabled": self.torque_enabled,
                "perception_last_capture_path": str(path_s),
            }
            ack.update(self._perception_state_payload())
            self._reply(ident, ack)
            self._broadcast_state_now()
            return
        if t == "perception_record_start":
            fps: Optional[float] = None
            try:
                fps_raw = float(msg.get("fps", 0.0))
                if fps_raw > 0.0:
                    fps = fps_raw
            except (TypeError, ValueError):
                fps = None
            ok, path_s, reason = self.start_perception_worker_recording(
                include_overlay=bool(msg.get("include_overlay", False)),
                fps=fps,
            )
            ack = {
                "t": "ack",
                "ts": proto.now_s(),
                "ok": bool(ok),
                "reason": str(reason),
                "device": self.device,
                "torque_enabled": self.torque_enabled,
                "perception_last_record_path": str(path_s),
            }
            ack.update(self._perception_state_payload())
            self._reply(ident, ack)
            self._broadcast_state_now()
            return
        if t == "perception_record_stop":
            ok, path_s, frame_count, reason = self.stop_perception_worker_recording()
            ack = {
                "t": "ack",
                "ts": proto.now_s(),
                "ok": bool(ok),
                "reason": str(reason),
                "device": self.device,
                "torque_enabled": self.torque_enabled,
                "perception_last_record_path": str(path_s),
                "perception_record_frame_count": int(frame_count),
            }
            ack.update(self._perception_state_payload())
            self._reply(ident, ack)
            self._broadcast_state_now()
            return
        if t == "target":
            target_recv_s = float(proto.now_s())
            client_ts_s = 0.0
            try:
                client_ts_s = float(msg.get("ts", 0.0) or 0.0)
            except (TypeError, ValueError):
                client_ts_s = 0.0
            source = str(msg.get("source", "sim"))
            if not self._is_allowed_source(source):
                self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": False, "reason": "source_reject", "device": self.device, "torque_enabled": self.torque_enabled})
                return
            raw_sim_target = msg.get("sim_target", None)
            if raw_sim_target is not None:
                if not (isinstance(raw_sim_target, (list, tuple)) and len(raw_sim_target) == 3):
                    self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": False, "reason": "bad_sim_target", "device": self.device, "torque_enabled": self.torque_enabled})
                    return
                try:
                    self.last_sim_target_xyz = (
                        float(raw_sim_target[0]),
                        float(raw_sim_target[1]),
                        float(raw_sim_target[2]),
                    )
                except (TypeError, ValueError):
                    self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": False, "reason": "bad_sim_target", "device": self.device, "torque_enabled": self.torque_enabled})
                    return
                self._reply(
                    ident,
                    {
                        "t": "ack",
                        "ts": proto.now_s(),
                        "ok": True,
                        "reason": "sim_target",
                        "device": self.device,
                        "torque_enabled": self.torque_enabled,
                        "sim_target": [float(v) for v in self.last_sim_target_xyz],
                    },
                )
                self._broadcast_state_now()
                return
            if self._safety_fault:
                self._reply(
                    ident,
                    {
                        "t": "ack",
                        "ts": proto.now_s(),
                        "ok": False,
                        "reason": self._safety_fault,
                        "device": self.device,
                        "torque_enabled": self.torque_enabled,
                    },
                )
                return
            raw_debug_markers = msg.get("debug_markers", None)
            if isinstance(raw_debug_markers, list):
                ok, reason = self._update_external_debug_markers(raw_debug_markers)
                self._reply(
                    ident,
                    {
                        "t": "ack",
                        "ts": proto.now_s(),
                        "ok": bool(ok),
                        "reason": str(reason),
                        "device": self.device,
                        "torque_enabled": self.torque_enabled,
                    },
                )
                return
            if source == "perception":
                center_uv_raw = msg.get("image_center_uv", None)
                if not (isinstance(center_uv_raw, (list, tuple)) and len(center_uv_raw) == 2):
                    self._reply(
                        ident,
                        {
                            "t": "ack",
                            "ts": proto.now_s(),
                            "ok": False,
                            "reason": "perception missing image_center_uv",
                            "device": self.device,
                            "torque_enabled": self.torque_enabled,
                        },
                    )
                    return
                self.last_perceived_center_uv = (
                    float(center_uv_raw[0]),
                    float(center_uv_raw[1]),
                )
                scale_raw = msg.get("image_scale", None)
                if scale_raw is not None:
                    try:
                        self.last_perceived_scale = float(scale_raw)
                    except (TypeError, ValueError):
                        self.last_perceived_scale = None
                confidence_raw = msg.get("object_confidence", None)
                if confidence_raw is not None:
                    try:
                        self.last_perceived_object_confidence = float(confidence_raw)
                    except (TypeError, ValueError):
                        self.last_perceived_object_confidence = 0.0
                self.last_perceived_object_label = str(msg.get("object_label", ""))
                self.last_perceived_timestamp_s = float(proto.now_s())
                depth_valid = bool(msg.get("depth_valid", True))
                object_camera_raw = msg.get("object_camera", None)
                if isinstance(object_camera_raw, (list, tuple)) and len(object_camera_raw) == 3:
                    self.last_perceived_object_camera_xyz = (
                        float(object_camera_raw[0]),
                        float(object_camera_raw[1]),
                        float(object_camera_raw[2]),
                    )
                object_world = None
                ok = True
                reason = ""
                object_world_raw = msg.get("object_world", None)
                cam_origin_raw = msg.get("camera_world_origin", None)
                cam_look_raw = msg.get("camera_world_look", None)
                cam_right_raw = msg.get("camera_world_right", None)
                has_frame_cam_pose = (
                    isinstance(cam_origin_raw, (list, tuple))
                    and len(cam_origin_raw) == 3
                    and isinstance(cam_look_raw, (list, tuple))
                    and len(cam_look_raw) == 3
                    and isinstance(cam_right_raw, (list, tuple))
                    and len(cam_right_raw) == 3
                )
                if isinstance(object_world_raw, (list, tuple)) and len(object_world_raw) == 3:
                    try:
                        p_w = np.asarray(
                            [float(object_world_raw[0]), float(object_world_raw[1]), float(object_world_raw[2])],
                            dtype=float,
                        ).reshape(3)
                    except (TypeError, ValueError):
                        p_w = None
                    if p_w is not None:
                        object_world = (float(p_w[0]), float(p_w[1]), float(p_w[2]))
                        self.last_perceived_object_world_xyz = object_world
                        world_tag = "sim_frame_pose" if has_frame_cam_pose else "mock_world"
                        cam_world = cam_look = cam_right = None
                        if has_frame_cam_pose:
                            cam_world = (float(cam_origin_raw[0]), float(cam_origin_raw[1]), float(cam_origin_raw[2]))
                            cam_look = (float(cam_look_raw[0]), float(cam_look_raw[1]), float(cam_look_raw[2]))
                            cam_right = (float(cam_right_raw[0]), float(cam_right_raw[1]), float(cam_right_raw[2]))
                        p_cam = self.last_perceived_object_camera_xyz or (0.0, 0.0, 0.0)
                        self._set_perception_debug_markers(
                            object_world=object_world,
                            object_label=str(self.last_perceived_object_label),
                            object_camera_xyz=tuple(float(v) for v in p_cam),
                            world_tag=world_tag,
                            camera_world=cam_world,
                            camera_look=cam_look,
                            camera_right=cam_right,
                            ttl_ms=30000 if world_tag == "mock_world" else 3000,
                        )
                        ok, reason = True, f"perception {world_tag}"
                elif depth_valid and self.last_perceived_object_camera_xyz is not None:
                    ok, reason, object_world = self._update_perception_markers(
                        self.last_perceived_object_camera_xyz,
                        object_label=self.last_perceived_object_label,
                    )
                else:
                    ok, reason = True, ""
                    object_world = self.last_perceived_object_world_xyz
                    if object_world is not None:
                        label_suffix = (
                            f":{self.last_perceived_object_label}"
                            if str(self.last_perceived_object_label).strip()
                            else ""
                        )
                        self._set_debug_marker(
                            name=f"perceived_object{label_suffix}",
                            pos=object_world,
                            color=[0.1, 0.95, 0.2, 0.95],
                            radius=0.012,
                            ttl_ms=30000,
                        )
                ack: Dict[str, Any] = {
                    "t": "ack",
                    "ts": proto.now_s(),
                    "ok": bool(ok),
                    "reason": str(reason),
                    "device": self.device,
                    "torque_enabled": self.torque_enabled,
                    "perceived_center_uv": [
                        float(self.last_perceived_center_uv[0]),
                        float(self.last_perceived_center_uv[1]),
                    ],
                    "perceived_scale": float(self.last_perceived_scale or 0.0),
                    "perceived_timestamp_s": float(self.last_perceived_timestamp_s),
                    "perceived_object_label": str(self.last_perceived_object_label),
                    "perceived_object_confidence": float(self.last_perceived_object_confidence),
                }
                if self.last_perceived_object_camera_xyz is not None:
                    p_cam = self.last_perceived_object_camera_xyz
                    ack["perceived_object_camera"] = [float(p_cam[0]), float(p_cam[1]), float(p_cam[2])]
                if object_world is not None:
                    p_w = np.asarray(object_world, dtype=float).reshape(3)
                    ack["object_world"] = [float(p_w[0]), float(p_w[1]), float(p_w[2])]
                self._reply(ident, ack)
                self._broadcast_state_now()
                return
            seq = int(msg.get("seq", -1))
            q: Optional[proto.SimQ] = None
            partial_u_mode = False
            partial_target_u: Optional[proto.ControlU] = None
            partial_target_axes: Set[str] = set()
            if "u" in msg and isinstance(msg.get("u"), dict):
                raw_u = dict(msg["u"])
                u_keys = {str(k).strip().lower() for k in raw_u.keys()}
                if u_keys.issubset({"linear", "roll", "s1", "s2"}) and u_keys:
                    partial_u_mode = True
                    if bool(self._arm_latency_log_enable):
                        self._arm_latency_count("recv")
                        if client_ts_s > 0.0:
                            self._arm_latency_sample_ms(
                                "client_to_host_ms",
                                (target_recv_s - client_ts_s) * 1000.0,
                            )
                    merged_u = self._merge_partial_target_u({str(k): float(v) for k, v in raw_u.items()})
                    if merged_u is not None:
                        partial_target_u = merged_u
                        partial_target_axes = set(self._pending_target_axes)
                        q = proto.control_u_to_sim_q(merged_u, self.cfg)
                else:
                    q = proto.control_u_to_sim_q(proto.unpack_u(msg["u"]), self.cfg)
            elif "q" in msg:
                q = proto.unpack_q(msg["q"])
            target_raw = msg.get("target", None)
            if isinstance(target_raw, (list, tuple)) and len(target_raw) == 3:
                self.last_ik_target_xyz = (float(target_raw[0]), float(target_raw[1]), float(target_raw[2]))
            target_dir_raw = msg.get("target_dir", None)
            if isinstance(target_dir_raw, (list, tuple)) and len(target_dir_raw) == 3:
                self.last_ik_target_dir = (
                    float(target_dir_raw[0]),
                    float(target_dir_raw[1]),
                    float(target_dir_raw[2]),
                )
            ready_pose_dir_raw = msg.get("ready_pose_dir", None)
            if isinstance(ready_pose_dir_raw, (list, tuple)) and len(ready_pose_dir_raw) == 3:
                self.last_ready_pose_dir = (
                    float(ready_pose_dir_raw[0]),
                    float(ready_pose_dir_raw[1]),
                    float(ready_pose_dir_raw[2]),
                )
            ready_pose_standoff_raw = msg.get("ready_pose_standoff_m", None)
            if ready_pose_standoff_raw is not None:
                try:
                    self.last_ready_pose_standoff_m = max(0.0, float(ready_pose_standoff_raw))
                except (TypeError, ValueError):
                    pass
            if ready_pose_dir_raw is not None or ready_pose_standoff_raw is not None:
                if self.last_perceived_object_world_xyz is not None:
                    standoff = float(self.last_ready_pose_standoff_m)
                    if standoff <= float(self.pick_config.grasp_standoff_m) + 1e-6:
                        self._set_grasp_target_markers(
                            self.last_perceived_object_world_xyz,
                            standoff_m=float(self.pick_config.grasp_standoff_m),
                            ttl_ms=30000,
                        )
                    else:
                        self._set_ready_pose_markers(
                            self.last_perceived_object_world_xyz,
                            ttl_ms=30000,
                        )
            sag_raw = msg.get("sag_model", None)
            if isinstance(sag_raw, dict):
                self.last_sag_model = dict(sag_raw)
            if "claw_closed" in msg:
                self.last_claw_closed = bool(msg.get("claw_closed", False))
            if "go2_vel" in msg:
                try:
                    next_vel = proto.unpack_go2_vel(msg.get("go2_vel"))
                except Exception:
                    self._reply(
                        ident,
                        {
                            "t": "ack",
                            "ts": proto.now_s(),
                            "ok": False,
                            "reason": "bad_go2_vel",
                            "device": self.device,
                            "torque_enabled": self.torque_enabled,
                        },
                    )
                    return
                lock_reason = self._go2_motion_lock_reason(next_vel)
                if lock_reason:
                    self._clear_go2_vel()
                    if self._go2_bridge is not None:
                        self._go2_bridge.set_velocity(0.0, 0.0, 0.0)
                    self._reply(
                        ident,
                        {
                            "t": "ack",
                            "ts": proto.now_s(),
                            "ok": False,
                            "reason": lock_reason,
                            "device": self.device,
                            "torque_enabled": self.torque_enabled,
                        },
                    )
                    self._broadcast_state_now()
                    return
                self.last_go2_vel = next_vel
                self._last_go2_vel_ts = proto.now_s()
                if self._go2_bridge is not None:
                    vx, vy, wz = self.last_go2_vel
                    self._go2_bridge.set_velocity(vx, vy, wz)
            if "go2_sport_pose" in msg:
                try:
                    pose = normalize_go2_sport_pose(proto.unpack_go2_sport_pose(msg.get("go2_sport_pose")))
                except Exception:
                    self._reply(
                        ident,
                        {
                            "t": "ack",
                            "ts": proto.now_s(),
                            "ok": False,
                            "reason": "bad_go2_sport_pose",
                            "device": self.device,
                            "torque_enabled": self.torque_enabled,
                        },
                    )
                    return
                if sport_pose_api_id(pose) is None:
                    self._reply(
                        ident,
                        {
                            "t": "ack",
                            "ts": proto.now_s(),
                            "ok": False,
                            "reason": f"unknown GO2 sport pose: {pose}",
                            "device": self.device,
                            "torque_enabled": self.torque_enabled,
                        },
                    )
                    return
                ok_pose, pose_reason = self._authorize_go2_sport_pose(pose)
                if not ok_pose:
                    self._reply(
                        ident,
                        {
                            "t": "ack",
                            "ts": proto.now_s(),
                            "ok": False,
                            "reason": pose_reason,
                            "device": self.device,
                            "torque_enabled": self.torque_enabled,
                        },
                    )
                    return
                if self._go2_bridge is not None:
                    try:
                        self._go2_bridge.call_sport_pose(pose)
                    except ValueError as exc:
                        self._reply(
                            ident,
                            {
                                "t": "ack",
                                "ts": proto.now_s(),
                                "ok": False,
                                "reason": str(exc),
                                "device": self.device,
                                "torque_enabled": self.torque_enabled,
                            },
                        )
                        return
                self._clear_go2_vel()
                self._note_go2_sport_pose(pose)
                self.last_go2_sport_pose = str(pose)
                self.last_go2_sport_pose_seq += 1
            if "go2_obstacles_avoid_enable" in msg:
                try:
                    enabled = proto.unpack_go2_obstacles_avoid_enable(msg.get("go2_obstacles_avoid_enable"))
                except Exception:
                    self._reply(
                        ident,
                        {
                            "t": "ack",
                            "ts": proto.now_s(),
                            "ok": False,
                            "reason": "bad_go2_obstacles_avoid_enable",
                            "device": self.device,
                            "torque_enabled": self.torque_enabled,
                        },
                    )
                    return
                if self._go2_bridge is not None:
                    self._go2_bridge.set_obstacles_avoid(enabled)
                self.last_go2_obstacles_avoid_enabled = bool(enabled)
                self.last_go2_obstacles_avoid_seq += 1
            if q is None:
                if (
                    target_raw is None
                    and target_dir_raw is None
                    and ready_pose_dir_raw is None
                    and ready_pose_standoff_raw is None
                    and sag_raw is None
                    and "claw_closed" not in msg
                    and "go2_vel" not in msg
                    and "go2_sport_pose" not in msg
                    and "go2_obstacles_avoid_enable" not in msg
                ):
                    self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": False, "reason": "bad_target", "device": self.device, "torque_enabled": self.torque_enabled})
                    return
                self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": True, "seq": seq, "device": self.device, "torque_enabled": self.torque_enabled})
                if "go2_vel" in msg or "claw_closed" in msg or "go2_sport_pose" in msg or "go2_obstacles_avoid_enable" in msg:
                    self._broadcast_state_now()
                return
            self._pending_target_q = q
            self._pending_target_seq = seq
            self._last_target_apply_error = ""
            log_key = "%s:%s:%.4f:%.4f:%.4f:%.4f" % (
                str(source),
                str(bool(partial_u_mode)).lower(),
                float(q.linear_m),
                float(q.roll_rad),
                float(q.theta1_rad),
                float(q.theta2_rad),
            )
            now_log = time.time()
            log_target = True
            if bool(partial_u_mode) and str(source).strip().lower() in ("slider", "gaze", "servo"):
                log_target = (
                    log_key != self._last_target_log_key
                    or (now_log - float(self._last_target_log_t)) >= 1.0
                )
            if log_target:
                self._last_target_log_key = log_key
                self._last_target_log_t = now_log
                print(
                    "[host] target received | seq=%d source=%s partial_u=%s q=(%.4f, %.4f, %.4f, %.4f)"
                    % (
                        int(seq),
                        str(source),
                        str(bool(partial_u_mode)).lower(),
                        float(q.linear_m),
                        float(q.roll_rad),
                        float(q.theta1_rad),
                        float(q.theta2_rad),
                    )
                )
            if not partial_u_mode:
                self._clear_arm_servo_target()
                self._pending_target_u = None
                self._pending_target_axes = set()
                self._target_u_state = proto.sim_q_to_control_u(q, self.cfg)
                self._schedule_target_motion(q, source=source)
                if (
                    str(source).strip().lower() in {"lji", "lji_step"}
                    and self._has_hw()
                    and not self._use_trajectory_for_source(source)
                ):
                    applied_hw, complete = self._apply_sim_q_target(q)
                    if applied_hw:
                        self._target_u_state = proto.sim_q_to_control_u(q, self.cfg)
                        if bool(complete):
                            self._pending_target_q = None
                    else:
                        self._reply(
                            ident,
                            {
                                "t": "ack",
                                "ts": proto.now_s(),
                                "ok": False,
                                "reason": self._last_target_apply_error or "direct_lji_step_failed",
                                "device": self.device,
                                "torque_enabled": self.torque_enabled,
                            },
                        )
                        self._state_broadcast_requested.set()
                        return
            else:
                self._cancel_trajectory()
                if self._submit_arm_servo_partial_target(
                    partial_target_u,
                    partial_target_axes,
                    seq=int(seq),
                    source=str(source),
                    host_recv_s=float(target_recv_s),
                    client_ts_s=float(client_ts_s),
                    perceived_ts_s=float(self.last_perceived_timestamp_s),
                ):
                    self._pending_target_q = None
                    self._pending_target_u = None
                    self._pending_target_axes = set()
                    self._pending_target_seq = -1
            if not self._has_hw():
                self.last_q = q
                self.last_u = proto.sim_q_to_control_u(q, self.cfg)
                self.last_state_ts = time.time()
                if not partial_u_mode:
                    self._pending_target_q = None
                    self._cancel_trajectory()
            self._reply(
                ident,
                {
                    "t": "ack",
                    "ts": proto.now_s(),
                    "ok": True,
                    "seq": seq,
                    "device": self.device,
                    "torque_enabled": self.torque_enabled,
                    "reason": (self._last_target_apply_error or ""),
                },
            )
            return
        self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": False, "reason": "unknown_type", "device": self.device, "torque_enabled": self.torque_enabled})

    def loop_forever(self) -> None:
        while not self._stop_event.is_set():
            now = time.time()
            try:
                events = dict(self.poller.poll(timeout=10))
            except KeyboardInterrupt:
                break
            if self.sock in events and events[self.sock] & zmq.POLLIN:
                while True:
                    try:
                        ident, data = self.sock.recv_multipart(flags=zmq.NOBLOCK)
                    except Exception:
                        break
                    try:
                        msg = proto.loads_msg(data)
                    except Exception:
                        self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": False, "reason": "json", "torque_enabled": self.torque_enabled})
                        continue
                    self._handle_msg(ident, msg)
            if self.sim_feedback in events and events[self.sim_feedback] & zmq.POLLIN:
                while True:
                    try:
                        data = self.sim_feedback.recv(flags=zmq.NOBLOCK)
                    except Exception:
                        break
                    try:
                        msg = proto.loads_msg(data)
                    except Exception:
                        continue
                    self._handle_sim_feedback(msg)
            if (now - self._t_read) >= self._read_period:
                self._t_read = now
                read_currents = (now - self._t_current_read) >= self._current_read_period
                if read_currents:
                    self._t_current_read = now
                self._read_hw_state(read_currents=read_currents)
                self._update_claw_hw()
            if self._pending_target_q is not None and (now - self._t_cmd) >= self._cmd_period:
                self._t_cmd = now
                applied = False
                if self._pending_target_u is not None and self._pending_target_axes:
                    applied = self._apply_partial_u_target(self._pending_target_u, set(self._pending_target_axes))
                    if applied:
                        self._pending_target_u = None
                        self._pending_target_axes = set()
                        self._pending_target_q = None
                        self._cancel_trajectory()
                        self._state_broadcast_requested.set()
                else:
                    if self._trajectory_lji.active:
                        step = self._trajectory_lji.step(now_s=now)
                    elif self._trajectory.active:
                        step = self._trajectory.step(now_s=now)
                    else:
                        step = None
                    if step is not None:
                        q_cmd = step.q_cmd
                        self._traj_step_count += 1
                    else:
                        q_cmd = self._pending_target_q
                    applied_hw, complete = self._apply_sim_q_target(q_cmd)
                    if step is not None and (
                        self._traj_step_count == 1
                        or (self._traj_step_count % max(int(self._traj_step_log_every), 1) == 0)
                        or bool(step.done)
                    ):
                        print(
                            "[host] trajectory step | idx=%d done=%s q_cmd=(%.4f, %.4f, %.4f, %.4f)"
                            % (
                                int(self._traj_step_count),
                                str(bool(step.done)).lower(),
                                float(q_cmd.linear_m),
                                float(q_cmd.roll_rad),
                                float(q_cmd.theta1_rad),
                                float(q_cmd.theta2_rad),
                            )
                        )
                    if self._traj_last_apply_ok is None or bool(applied_hw) != bool(self._traj_last_apply_ok):
                        if applied_hw:
                            print("[host] target apply success")
                        else:
                            print(f"[host] target apply failed | {self._last_target_apply_error or 'unknown'}")
                        self._traj_last_apply_ok = bool(applied_hw)
                    if applied_hw:
                        self._target_u_state = proto.sim_q_to_control_u(q_cmd, self.cfg)
                        if step is not None:
                            if bool(step.done):
                                print("[host] trajectory complete")
                                if pick_profile_enabled() and self._traj_profile_start_s is not None:
                                    dt_ms = (time.time() - float(self._traj_profile_start_s)) * 1000.0
                                    print("[Profile] traj complete | dt=%.1fms" % float(dt_ms))
                                    self._traj_profile_start_s = None
                                self._pending_target_q = None
                        elif complete:
                            self._pending_target_q = None
                        self._state_broadcast_requested.set()
                    else:
                        if self._last_target_apply_error:
                            self._broadcast(
                                {
                                    "t": "ack",
                                    "ts": proto.now_s(),
                                    "ok": False,
                                    "reason": self._last_target_apply_error,
                                    "device": self.device,
                                    "torque_enabled": self.torque_enabled,
                                }
                            )
            now_event = time.time()
            if (
                self._state_broadcast_requested.is_set()
                and (now_event - self._t_event_state) >= self._event_state_min_period
            ):
                self._state_broadcast_requested.clear()
                self._t_event_state = now_event
                self._broadcast_state_now()
            if (now - self._t_state) >= self._state_period:
                self._t_state = now
                if self._go2_bridge is not None:
                    self._go2_bridge.tick_cmd(now)
                    sample = self._go2_bridge.latest_state()
                    if sample is not None:
                        self._apply_go2_base_from_odom(sample)
                    self._go2_bridge.maybe_log_status(now)
                self._broadcast_state_now()


def run_host(
    *,
    config_path: str,
    bind_addr: str,
    device: str,
) -> None:
    bundle = load_app_config_from_ini(str(config_path))
    hw_cfg: HardwareConfig | None = bundle.hardware_config
    ik_context: dict[str, Any] = {}
    hand_eye_transform = None
    hand_eye_parent_frame = "node9"
    try:
        _ik_bundle, ik_context = load_solver_context(str(config_path))
    except Exception as exc:
        print(f"[host] IK context unavailable for perception markers: {exc}")
        ik_context = {}
    hand_eye_path = str(bundle.sim_config.hand_eye_config).strip()
    if hand_eye_path:
        try:
            hand_eye_transform, hand_eye_meta = load_hand_eye_transform(hand_eye_path)
            hand_eye_parent_frame = str(hand_eye_meta.get("parent_frame", "node9"))
        except Exception as exc:
            print(f"[host] hand-eye config unavailable: {exc}")
            hand_eye_transform = None
    hw = None
    direction: Dict[int, int] = {}
    device = str(device).strip()
    if device:
        hw, direction = load_hardware(device, hardware_cfg=hw_cfg)
    go2_bridge: Optional[UnitreeRos2Bridge] = None
    try:
        go2_bridge = create_go2_bridge_if_enabled(
            bundle.go2_hardware_config,
            use_go2=bool(bundle.sim_config.use_go2),
        )
        if go2_bridge is not None:
            go2_bridge.start()
    except Exception as exc:
        print(f"[host] go2 bridge unavailable: {exc}")
        go2_bridge = None
    try:
        if hw is not None:
            hw.open()
        server = ControlHost(
            bind_addr=str(bind_addr),
            sim_pub_addr=str(bundle.sim_config.host_sim_port),
            sim_feedback_addr=str(bundle.sim_config.host_feedback_port),
            hw=hw,
            direction_by_id=direction,
            device=device,
            hardware_cfg=hw_cfg,
            config_path=str(config_path),
            ik_config=bundle.ik_config,
            ik_context=ik_context,
            hand_eye_transform=hand_eye_transform,
            hand_eye_parent_frame=hand_eye_parent_frame,
            pick_config=bundle.pick_config,
            perception_config=bundle.perception_config,
            gaze_config=bundle.gaze_stabilizer_config,
            ownership_enable=bool(getattr(bundle.experiment_config, "ownership_enable", False)),
            show_all_ports=bool(bundle.sim_config.show_all_ports),
            cfg=bundle.mapping_config,
            trajectory_cfg=QuinticTimingConfig(
                enable=bool(bundle.sim_config.traj_enable),
                duration_s=float(bundle.sim_config.traj_duration_s),
                min_duration_s=float(bundle.sim_config.traj_min_s),
                max_duration_s=float(bundle.sim_config.traj_max_s),
                linear_scale_m=float(bundle.sim_config.traj_linear_scale_m),
                angular_scale_rad=float(bundle.sim_config.traj_angular_scale_rad),
            ),
            trajectory_lji_cfg=QuinticTimingConfig(
                enable=bool(bundle.sim_config.traj_lji_enable),
                duration_s=float(bundle.sim_config.traj_lji_duration_s),
                min_duration_s=float(bundle.sim_config.traj_lji_min_s),
                max_duration_s=float(bundle.sim_config.traj_lji_max_s),
                linear_scale_m=float(bundle.sim_config.traj_linear_scale_m),
                angular_scale_rad=float(bundle.sim_config.traj_angular_scale_rad),
            ),
            traj_lji_enable=bool(bundle.sim_config.traj_lji_enable),
            hw_read_hz=float(getattr(bundle.hardware_config, "host_hw_read_hz", 20.0)),
            hw_cmd_hz=float(getattr(bundle.hardware_config, "host_hw_cmd_hz", 30.0)),
            go2_bridge=go2_bridge,
        )
        print(f"[host] comm with ctrl by {bind_addr}")
        print(f"[host] comm with sim by {bundle.sim_config.host_sim_port}")
        try:
            server.loop_forever()
        finally:
            server.close()
    finally:
        try:
            if hw is not None:
                hw.close()
        except Exception:
            pass


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(__file__), "config.ini"),
        help="path to ini config file",
    )
    ap.add_argument("--replace", action="store_true", help="terminate an existing host.py process before binding ports")
    args = ap.parse_args()
    config_path = str(args.config)
    if args.replace:
        _terminate_host_processes()
    bundle = load_app_config_from_ini(config_path)

    run_host(
        config_path=config_path,
        bind_addr=str(bundle.sim_config.host_ctrl_port),
        device="",
    )


if __name__ == "__main__":
    main()
