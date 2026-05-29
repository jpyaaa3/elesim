#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import os
import threading
import time
from typing import Any, Dict, Optional, Set

import numpy as np
import zmq

from engine.config_loader import load_app_config_from_ini
from engine.config_loader import HardwareConfig
from engine.iklib.solver import load_solver_context
from engine.motor import load_hardware, tick_to_deg_0_360
import engine.protocol as proto
from addons.perception_bridge.hand_eye import camera_axes_world, camera_point_to_world, load_hand_eye_transform

from serial.tools import list_ports as serial_list_ports


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
        ik_context: Optional[dict[str, Any]] = None,
        hand_eye_transform: Optional[Any] = None,
        hand_eye_parent_frame: str = "node9",
        show_all_ports: bool = False,
        cfg: proto.SimMappingConfig = proto.SimMappingConfig(),
        state_hz: float = 10.0,
        hw_read_hz: float = 20.0,
        hw_cmd_hz: float = 30.0,
    ) -> None:
        if zmq is None:
            raise SystemExit("pyzmq is required. Install: pip install pyzmq")
        self.cfg = cfg
        self.hw = hw
        self.direction_by_id = direction_by_id
        self.device = str(device)
        self.hardware_cfg = hardware_cfg
        self.ik_context = dict(ik_context or {})
        self.hand_eye_transform = None if hand_eye_transform is None else np.asarray(hand_eye_transform, dtype=float).reshape(4, 4)
        self.hand_eye_parent_frame = str(hand_eye_parent_frame)
        self.show_all_ports = bool(show_all_ports)

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
        self.torque_enabled: bool = False
        self.last_ik_target_xyz: Optional[tuple[float, float, float]] = None
        self.last_ik_target_dir: Optional[tuple[float, float, float]] = None
        self.last_actual_tip_xyz: Optional[tuple[float, float, float]] = None
        self.last_actual_tip_dir: Optional[tuple[float, float, float]] = None
        self.last_perceived_object_label: str = ""
        self.last_perceived_object_confidence: float = 0.0
        self.last_perceived_object_camera_xyz: Optional[tuple[float, float, float]] = None
        self.last_perceived_center_uv: Optional[tuple[float, float]] = None
        self.last_perceived_scale: Optional[float] = None
        self.last_perceived_timestamp_s: float = 0.0
        self.last_sag_model: dict[str, Any] = {}
        self.last_claw_closed: bool = False
        self._last_hw_pos_by_id: Dict[int, int] = {}
        self._last_claw_current: int = 0
        self._claw_close_stalled: bool = False

        self._state_period = 1.0 / max(0.1, float(state_hz))
        self._read_period = 1.0 / max(0.1, float(hw_read_hz))
        self._cmd_period = 1.0 / max(0.1, float(hw_cmd_hz))
        self._t_read = 0.0
        self._t_state = 0.0
        self._t_cmd = 0.0

        self._pending_target_q: Optional[proto.SimQ] = None
        self._pending_target_u: Optional[proto.ControlU] = None
        self._pending_target_axes: Set[str] = set()
        self._pending_target_seq: int = -1
        self._target_u_state: Optional[proto.ControlU] = None

        self._ids = getattr(hw, "ids", [])
        self._hw_lock = threading.RLock()
        self._stop_event = threading.Event()
        self._claw_open_deg = 340.0
        self._claw_close_deg = 230.0
        self._claw_stop_current = -200
        self._current_yellow_ma = int(getattr(hardware_cfg, "current_yellow_ma", 1800) if hardware_cfg is not None else 1800)
        self._current_limit_ma = int(getattr(hardware_cfg, "current_limit_ma", 2500) if hardware_cfg is not None else 2500)
        self._last_motor_current_by_id: Dict[int, int] = {}
        self._safety_fault: str = ""
        self._yellow_zone_ids: Set[int] = set()
        if not self._has_hw():
            self._set_virtual_neutral_state()

    def _set_virtual_neutral_state(self) -> None:
        neutral_q = proto.SimQ(
            linear_m=0.0,
            roll_rad=0.0,
            theta1_rad=0.0,
            theta2_rad=0.0,
        )
        self.last_q = neutral_q
        self.last_u = proto.sim_q_to_control_u(neutral_q, self.cfg)
        self._target_u_state = self.last_u
        self.last_state_ts = time.time()
        self._debug_markers_by_name: Dict[str, dict[str, Any]] = {}

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
        with self._hw_lock:
            if new_device == str(self.device).strip() and self.hw is not None:
                return
            old_hw = self.hw
            old_direction = dict(self.direction_by_id)
            old_ids = list(self._ids)
            old_device = str(self.device)
            self._pending_target_q = None
            self._pending_target_u = None
            self._pending_target_axes = set()
            self._pending_target_seq = -1
            self._target_u_state = None
            self.last_u = None
            self.last_q = None
            self.last_state_ts = 0.0
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
            self.last_sag_model = {}
            self.last_claw_closed = False
            self._last_hw_pos_by_id = {}
            self._last_claw_current = 0
            self._claw_close_stalled = False
            self._last_motor_current_by_id = {}
            self._safety_fault = ""
            self._yellow_zone_ids = set()
            if old_hw is not None:
                try:
                    old_hw.close()
                except Exception:
                    pass
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

    def clear_device(self) -> None:
        with self._hw_lock:
            old_hw = self.hw
            self._pending_target_q = None
            self._pending_target_u = None
            self._pending_target_axes = set()
            self._pending_target_seq = -1
            self._target_u_state = None
            self.last_u = None
            self.last_q = None
            self.last_state_ts = 0.0
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
            self.last_sag_model = {}
            self.last_claw_closed = False
            self._last_hw_pos_by_id = {}
            self._last_claw_current = 0
            self._claw_close_stalled = False
            self._last_motor_current_by_id = {}
            self._safety_fault = ""
            self._yellow_zone_ids = set()
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
        return str(source) in ("slider", "ik", "sim", "target", "perception")

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
        self._debug_markers_by_name[str(name)] = marker

    def _update_perception_markers(
        self, object_camera_xyz: tuple[float, float, float], *, object_label: str = ""
    ) -> tuple[bool, str, Optional[np.ndarray]]:
        if not self.ik_context or self.hand_eye_transform is None:
            return False, "perception disabled: missing hand-eye or IK context", None
        if self.last_q is None:
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
        label_txt = str(object_label).strip()
        print(
            f"[Perception] label={label_txt or '-'} "
            f"camera=[{p_cam[0]:+.4f}, {p_cam[1]:+.4f}, {p_cam[2]:+.4f}] m "
            f"world=[{p_w[0]:+.4f}, {p_w[1]:+.4f}, {p_w[2]:+.4f}] m"
        )
        label_suffix = f":{label_txt}" if label_txt else ""
        marker_ttl_ms = 3000
        self._set_debug_marker(
            name=f"perceived_object{label_suffix}",
            pos=object_world,
            color=[0.1, 0.95, 0.2, 0.95],
            radius=0.012,
            ttl_ms=marker_ttl_ms,
        )
        self._set_debug_marker(
            name="camera_optical",
            pos=camera_world,
            color=[0.1, 0.7, 1.0, 0.95],
            radius=0.010,
            ttl_ms=marker_ttl_ms,
        )
        self._set_debug_marker(
            name="camera_look",
            pos=camera_world,
            direction=camera_look,
            color=[0.1, 0.7, 1.0, 0.95],
            radius=0.004,
            ttl_ms=marker_ttl_ms,
        )
        self._set_debug_marker(
            name="camera_right",
            pos=camera_world,
            direction=camera_right,
            color=[1.0, 0.8, 0.2, 0.95],
            radius=0.004,
            ttl_ms=marker_ttl_ms,
        )
        return True, "perception markers updated", p_w

    def _broadcast_state_now(self) -> None:
        now = proto.now_s()
        self._broadcast(
            proto.pack_state(
                u=self.last_u,
                q=self.last_q,
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
                sag_model=self.last_sag_model,
                claw_closed=self.last_claw_closed,
                claw_current=self._last_claw_current,
                motor_currents_ma={self._motor_name_by_id(int(k)): int(v) for k, v in self._last_motor_current_by_id.items()},
                safety_fault=(self._safety_fault or None),
                debug_markers=self._active_debug_markers(),
            )
        )

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
            ttl_ms = int(raw.get("ttl_ms", 250))
            self._set_debug_marker(
                name=name,
                pos=pos,
                frame="world",
                direction=direction,
                color=list(color) if color is not None else None,
                radius=float(radius) if radius is not None else None,
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

    def _read_hw_state(self) -> None:
        if not self._has_hw():
            return
        try:
            with self._hw_lock:
                ticks_by_id = self.hw.get_present_positions()
        except Exception:
            return
        currents_by_id: Dict[int, int] = dict(self._last_motor_current_by_id)
        for dxl_id in self._ids:
            try:
                with self._hw_lock:
                    currents_by_id[int(dxl_id)] = int(self.hw.get_present_current(int(dxl_id)))
            except Exception:
                continue
        self._last_hw_pos_by_id = dict(ticks_by_id)
        self._last_motor_current_by_id = dict(currents_by_id)
        self._last_claw_current = int(currents_by_id.get(int(self.hw.cfg.id_claw), self._last_claw_current))
        self._check_current_limit()
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

    def _trip_safety_fault(self, reason: str) -> None:
        self._safety_fault = str(reason)
        print(f"[host] RED zone trip: {self._safety_fault}")
        self._pending_target_q = None
        self._pending_target_u = None
        self._pending_target_axes = set()
        self._pending_target_seq = -1
        try:
            if self._has_hw():
                with self._hw_lock:
                    self.hw.torque_off_all()
        except Exception:
            pass
        self.torque_enabled = False

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
                    f"overcurrent {self._motor_name_by_id(int(dxl_id))}: {int(current_ma)} mA exceeds {limit} mA"
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
            return True, complete
        except Exception:
            return False, False

    def _merge_partial_target_u(self, partial_u: Dict[str, float]) -> Optional[proto.ControlU]:
        base = self._target_u_state if self._target_u_state is not None else self.last_u
        if base is None:
            base = proto.ControlU(u_linear=0.0, u_roll=0.0, u_s1=180.0, u_s2=180.0)
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
            return bool(complete)
        except Exception:
            return False

    def torque_on(self, *, configure_modes: bool = True, set_profiles: bool = True, go_mid: bool = False) -> None:
        if not self._has_hw():
            raise RuntimeError("no device selected")
        with self._hw_lock:
            if self.torque_enabled:
                return
            if configure_modes:
                self.hw.set_operating_modes()
            if set_profiles:
                self.hw.set_profiles()
            self.hw.torque_on_all()
            self.torque_enabled = True
            self._safety_fault = ""
            if go_mid:
                self.hw.go_mid_pose()

    def torque_off(self) -> None:
        if not self._has_hw():
            raise RuntimeError("no device selected")
        with self._hw_lock:
            self._pending_target_q = None
            self._pending_target_seq = -1
            self.hw.torque_off_all()
            self.torque_enabled = False

    def close(self) -> None:
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
        if "q" in msg:
            try:
                self.last_q = proto.unpack_q(msg["q"])
                self.last_u = proto.sim_q_to_control_u(self.last_q, self.cfg)
                self.last_state_ts = float(msg.get("ts", proto.now_s()))
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
                self.torque_on()
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
        if t == "target":
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
            source = str(msg.get("source", "sim"))
            if not self._is_allowed_source(source):
                self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": False, "reason": "source_reject", "device": self.device, "torque_enabled": self.torque_enabled})
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
                if depth_valid and self.last_perceived_object_camera_xyz is not None:
                    ok, reason, object_world = self._update_perception_markers(
                        self.last_perceived_object_camera_xyz,
                        object_label=self.last_perceived_object_label,
                    )
                else:
                    ok, reason = True, "perception image (depth invalid, uv/scale only)"
                ack: Dict[str, Any] = {
                    "t": "ack",
                    "ts": proto.now_s(),
                    "ok": bool(ok),
                    "reason": str(reason),
                    "device": self.device,
                    "torque_enabled": self.torque_enabled,
                }
                if object_world is not None:
                    p_w = np.asarray(object_world, dtype=float).reshape(3)
                    ack["object_world"] = [float(p_w[0]), float(p_w[1]), float(p_w[2])]
                self._reply(ident, ack)
                self._broadcast_state_now()
                return
            seq = int(msg.get("seq", -1))
            q: Optional[proto.SimQ] = None
            partial_u_mode = False
            if "u" in msg and isinstance(msg.get("u"), dict):
                raw_u = dict(msg["u"])
                u_keys = {str(k).strip().lower() for k in raw_u.keys()}
                if u_keys.issubset({"linear", "roll", "s1", "s2"}) and u_keys:
                    partial_u_mode = True
                    merged_u = self._merge_partial_target_u({str(k): float(v) for k, v in raw_u.items()})
                    if merged_u is not None:
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
            sag_raw = msg.get("sag_model", None)
            if isinstance(sag_raw, dict):
                self.last_sag_model = dict(sag_raw)
            if "claw_closed" in msg:
                self.last_claw_closed = bool(msg.get("claw_closed", False))
            if q is None:
                if target_raw is None and target_dir_raw is None and sag_raw is None and "claw_closed" not in msg:
                    self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": False, "reason": "bad_target", "device": self.device, "torque_enabled": self.torque_enabled})
                    return
                self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": True, "seq": seq, "device": self.device, "torque_enabled": self.torque_enabled})
                return
            self._pending_target_q = q
            self._pending_target_seq = seq
            if not partial_u_mode:
                self._pending_target_u = None
                self._pending_target_axes = set()
                self._target_u_state = proto.sim_q_to_control_u(q, self.cfg)
            if not self._has_hw():
                self.last_q = q
                self.last_u = proto.sim_q_to_control_u(q, self.cfg)
                self.last_state_ts = time.time()
            self._reply(ident, {"t": "ack", "ts": proto.now_s(), "ok": True, "seq": seq, "device": self.device, "torque_enabled": self.torque_enabled})
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
                self._read_hw_state()
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
                else:
                    applied_hw, complete = self._apply_sim_q_target(self._pending_target_q)
                    if applied_hw:
                        self._target_u_state = proto.sim_q_to_control_u(self._pending_target_q, self.cfg)
                        if complete:
                            self._pending_target_q = None
            if (now - self._t_state) >= self._state_period:
                self._t_state = now
                self._broadcast(
                    proto.pack_state(
                        u=self.last_u,
                        q=self.last_q,
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
                        sag_model=self.last_sag_model,
                        claw_closed=self.last_claw_closed,
                        claw_current=self._last_claw_current,
                        motor_currents_ma={self._motor_name_by_id(int(k)): int(v) for k, v in self._last_motor_current_by_id.items()},
                        safety_fault=(self._safety_fault or None),
                        debug_markers=self._active_debug_markers(),
                    )
                )


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
            ik_context=ik_context,
            hand_eye_transform=hand_eye_transform,
            hand_eye_parent_frame=hand_eye_parent_frame,
            show_all_ports=bool(bundle.sim_config.show_all_ports),
            cfg=bundle.mapping_config,
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
    config_path = os.path.join(os.path.dirname(__file__), "config.ini")
    bundle = load_app_config_from_ini(config_path)

    run_host(
        config_path=config_path,
        bind_addr=str(bundle.sim_config.host_ctrl_port),
        device="",
    )


if __name__ == "__main__":
    main()
