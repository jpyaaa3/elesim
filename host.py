#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import os
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional, Sequence, Set

import numpy as np
import zmq

from engine import ik as ik_pipeline
from engine.config_loader import load_app_config_from_ini
from engine.config_loader import HardwareConfig
from engine.config_loader import PickFsmConfig
from engine.iklib.solver import load_solver_context
from engine.motor import load_hardware, tick_to_deg_0_360
from engine.pick_view_pregrasp import (
    ViewPregraspCandidate,
    ViewCandidateMetrics,
    ViewPregraspLimits,
    camera_visibility_fail_reasons,
    camera_visibility_ok,
    evaluate_view_candidate,
    format_view_candidate_log,
    generate_view_pregrasp_candidates,
    pick_best_strict_candidate,
    view_candidate_passes,
    view_candidate_passes_strict,
)
from engine.pick_visual_servo import (
    LOOK_JACOBIAN_AXIS_NAMES,
    JacobianLookGains,
    LookAlignLimits,
    LookGains,
    Q4Delta,
    advance_allowed,
    apply_q_delta,
    apply_q_delta_to_tuple,
    camera_xy_error,
    compute_advance_delta_q,
    compute_backoff_delta_q,
    compute_jacobian_look_delta_q,
    compute_look_delta_q,
    error_vector_2d,
    estimate_jacobian_column,
    jacobian_column_usable,
    look_align_ok,
    should_send_look_command,
    stack_jacobian,
)
import engine.protocol as proto
from addons.perception_bridge.hand_eye import (
    camera_axes_world,
    camera_point_to_world,
    load_hand_eye_transform,
    world_point_to_camera,
)

from serial.tools import list_ports as serial_list_ports


class PickStage(str, Enum):
    TARGET_LOCK = "TARGET_LOCK"
    VIEW_ALIGN = "VIEW_ALIGN"
    STOP_AND_CHECK = "STOP_AND_CHECK"
    LOOK_ALIGN = "LOOK_ALIGN"
    ADVANCE_SMALL = "ADVANCE_SMALL"
    COMMIT_GATE = "COMMIT_GATE"
    SHORT_APPROACH = "SHORT_APPROACH"
    CLOSE_GRIPPER = "CLOSE_GRIPPER"
    LIFT_AND_VERIFY = "LIFT_AND_VERIFY"


_PICK_STAGE_ALIASES: dict[str, PickStage] = {
    "SEARCH": PickStage.TARGET_LOCK,
    "TARGET_LOCK": PickStage.TARGET_LOCK,
    "COARSE_WORLD_PREGRASP": PickStage.VIEW_ALIGN,
    "STOP_AND_RELOCALIZE": PickStage.STOP_AND_CHECK,
    "CAMERA_SERVO_ALIGN": PickStage.LOOK_ALIGN,
    "CONFIDENCE_GATE": PickStage.COMMIT_GATE,
}


def resolve_pick_stage(stage_raw: str) -> PickStage:
    key = str(stage_raw).strip().upper()
    if key in _PICK_STAGE_ALIASES:
        return _PICK_STAGE_ALIASES[key]
    return PickStage(key)


@dataclass
class PickContext:
    stage: PickStage = PickStage.TARGET_LOCK
    stage_enter_ts: float = 0.0
    attempt: int = 0
    object_camera_samples: deque[tuple[float, float, float]] = field(default_factory=deque)
    object_world_latest: Optional[tuple[float, float, float]] = None
    object_camera_mu: Optional[tuple[float, float, float]] = None
    object_camera_cov: Optional[np.ndarray] = None
    object_world_mu: Optional[tuple[float, float, float]] = None
    desired_camera_object: tuple[float, float, float] = (0.0, 0.0, 0.1)
    pregrasp_world: Optional[tuple[float, float, float]] = None
    short_approach_world: Optional[tuple[float, float, float]] = None
    lift_start_z: Optional[float] = None
    stage_error_m: float = float("inf")
    stage_uncertainty: float = float("inf")
    last_perception_ts: float = 0.0
    anchor_world_xyz: Optional[tuple[float, float, float]] = None
    anchor_world_ts: float = 0.0
    anchor_confidence: float = 0.0
    consecutive_detection_count: int = 0
    dropout_count: int = 0
    last_valid_camera_mu: Optional[tuple[float, float, float]] = None
    last_valid_cov: Optional[np.ndarray] = None
    score: float = 0.0
    align_last_cmd_ts: float = 0.0
    align_prev_error_m: float = float("inf")
    align_no_improve_count: int = 0
    coarse_last_cmd_ts: float = 0.0
    coarse_target_q: Optional[tuple[float, float, float, float]] = None
    coarse_view_planned: bool = False
    coarse_candidate_tag: str = ""
    coarse_view_score: float = float("-inf")
    coarse_predicted_camera: Optional[tuple[float, float, float]] = None
    coarse_failed_tags: Set[str] = field(default_factory=set)
    manual_mode: bool = False
    track_state: str = ""
    track_confidence: float = 0.0
    depth_valid_ratio: float = 0.0
    track_lost_count: int = 0
    perception_mu_camera: Optional[tuple[float, float, float]] = None
    perception_sigma_camera: Optional[tuple[float, float, float]] = None
    last_object_camera: Optional[tuple[float, float, float]] = None
    advance_count: int = 0
    look_jacobian: Optional[np.ndarray] = None
    look_cal_phase: str = ""
    look_cal_substep: str = ""
    look_cal_axis_idx: int = 0
    look_cal_axis_order: list[int] = field(default_factory=list)
    look_cal_phase_ts: float = 0.0
    look_cal_columns: list[np.ndarray] = field(default_factory=list)
    look_cal_baseline_e: Optional[np.ndarray] = None
    look_cal_q_anchor: Optional[tuple[float, float, float, float]] = None


def filtered_camera_stats(
    samples: list[tuple[float, float, float]] | np.ndarray, *, outlier_zscore: float
) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    arr = np.asarray(samples, dtype=float)
    if arr.ndim != 2 or arr.shape[0] < 3 or arr.shape[1] != 3:
        return None, None
    mean = np.mean(arr, axis=0)
    std = np.std(arr, axis=0) + 1e-9
    zscore = np.abs((arr - mean) / std)
    keep = np.all(zscore <= float(outlier_zscore), axis=1)
    filtered = arr[keep]
    if filtered.shape[0] < 3:
        filtered = arr
    mu = np.mean(filtered, axis=0)
    cov = np.cov(filtered.T)
    if cov.shape != (3, 3):
        cov = np.diag(np.array([1e-4, 1e-4, 1e-4], dtype=float))
    return mu, cov


def should_pass_confidence_gate(*, error_m: float, uncertainty: float, error_threshold_m: float, uncertainty_threshold: float) -> bool:
    return float(error_m) <= float(error_threshold_m) and float(uncertainty) <= float(uncertainty_threshold)


def should_stage_timeout(*, stage_elapsed_s: float, timeout_s: float) -> bool:
    return float(stage_elapsed_s) > float(timeout_s)


def should_hard_fail(*, dropout_count: int, dropout_hard_limit: int, stage_elapsed_s: float, timeout_s: float, error_m: float) -> bool:
    return (
        int(dropout_count) > int(dropout_hard_limit)
        and float(stage_elapsed_s) > float(timeout_s)
        and float(error_m) > 1e-6
    )


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
        pick_fsm_cfg: Optional[PickFsmConfig] = None,
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
        self.pick_fsm_cfg = pick_fsm_cfg or PickFsmConfig()

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
        self._pick = PickContext()
        self._pick.stage_enter_ts = time.time()
        _pcfg = self.pick_fsm_cfg
        self._pick.desired_camera_object = (
            float(_pcfg.desired_camera_xy_m[0]),
            float(_pcfg.desired_camera_xy_m[1]),
            float(_pcfg.desired_camera_z_m),
        )
        self._pick_enabled = bool(self.pick_fsm_cfg.enable)
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
        self._set_debug_marker(
            name=f"perceived_object{label_suffix}",
            pos=object_world,
            color=[0.1, 0.95, 0.2, 0.95],
            radius=0.012,
            ttl_ms=250,
        )
        self._set_debug_marker(
            name="camera_optical",
            pos=camera_world,
            color=[0.1, 0.7, 1.0, 0.95],
            radius=0.010,
            ttl_ms=250,
        )
        self._set_debug_marker(
            name="camera_look",
            pos=camera_world,
            direction=camera_look,
            color=[0.1, 0.7, 1.0, 0.95],
            radius=0.004,
            ttl_ms=250,
        )
        self._set_debug_marker(
            name="camera_right",
            pos=camera_world,
            direction=camera_right,
            color=[1.0, 0.8, 0.2, 0.95],
            radius=0.004,
            ttl_ms=250,
        )
        return True, "perception markers updated", p_w

    def _pick_set_stage(self, stage: PickStage, now: float) -> None:
        prev = self._pick.stage
        self._pick.stage = stage
        self._pick.stage_enter_ts = float(now)
        if stage == PickStage.VIEW_ALIGN and prev != stage:
            self._pick.coarse_last_cmd_ts = 0.0
            self._pick.coarse_target_q = None
            self._pick.coarse_view_planned = False
            self._pick.coarse_candidate_tag = ""
            self._pick.coarse_view_score = float("-inf")
            self._pick.coarse_predicted_camera = None
            self._pick.coarse_failed_tags.clear()
        if stage == PickStage.LOOK_ALIGN and prev != stage:
            self._pick.align_last_cmd_ts = 0.0
            self._pick.align_prev_error_m = float("inf")
            self._pick.align_no_improve_count = 0
            self._pick.look_jacobian = None
            self._pick.look_cal_columns = []
            self._pick.look_cal_baseline_e = None
            self._pick.look_cal_q_anchor = None
            self._pick.look_cal_axis_idx = 0
            self._pick_sync_desired_camera_from_live()
            cfg = self.pick_fsm_cfg
            if bool(cfg.look_jacobian_include_roll):
                self._pick.look_cal_axis_order = [0, 1, 2]
            else:
                self._pick.look_cal_axis_order = [1, 2]
            skip_cal = self._pick_look_align_ok()
            if str(cfg.look_servo_mode).strip().lower() == "jacobian" and not skip_cal:
                self._pick.look_cal_phase = "running"
                self._pick.look_cal_substep = "baseline_wait"
                self._pick.look_cal_phase_ts = float(now)
            else:
                self._pick.look_cal_phase = "done" if skip_cal else ""
                self._pick.look_cal_substep = ""
                if skip_cal:
                    print("[pick] LOOK_ALIGN: xy already within threshold, skip Jacobian cal", flush=True)

    def _pick_can_auto_advance(self) -> bool:
        """Uncommanded pick flow (e.g. TARGET_LOCK search). Manual mode stays on TARGET_LOCK."""
        return not bool(self._pick.manual_mode)

    def _pick_can_complete_stage(self) -> bool:
        """Forward chain when stage goals are met (manual stage buttons still advance)."""
        return True

    def _pick_reset_to_search(self, now: float, *, increment_attempt: bool = False) -> None:
        if increment_attempt:
            self._pick.attempt += 1
        self._pick.stage_error_m = float("inf")
        self._pick.stage_uncertainty = float("inf")
        self._pick.pregrasp_world = None
        self._pick.short_approach_world = None
        self._pick.object_camera_mu = None
        self._pick.object_camera_cov = None
        self._pick.object_world_mu = None
        self._pick.lift_start_z = None
        self._pick.dropout_count = 0
        self._pick.consecutive_detection_count = 0
        self._pick.align_last_cmd_ts = 0.0
        self._pick.align_prev_error_m = float("inf")
        self._pick.align_no_improve_count = 0
        self._pick.coarse_last_cmd_ts = 0.0
        self._pick.coarse_target_q = None
        self._pick.coarse_view_planned = False
        self._pick.coarse_candidate_tag = ""
        self._pick.coarse_view_score = float("-inf")
        self._pick.coarse_predicted_camera = None
        self._pick.coarse_failed_tags.clear()
        self._pick.last_object_camera = None
        self._pick.advance_count = 0
        self._pick.look_jacobian = None
        self._pick.look_cal_phase = ""
        self._pick.look_cal_substep = ""
        self._pick.look_cal_columns = []
        self._pick.look_cal_baseline_e = None
        self._pick.look_cal_q_anchor = None
        self._pick_set_stage(PickStage.TARGET_LOCK, now)

    def _pick_soft_fail(self) -> None:
        self._pick.dropout_count += 1
        self._pick.score = float(self._pick.score) - 0.05

    def _pick_hard_fail(self, now: float) -> None:
        if bool(self.pick_fsm_cfg.attempt_hard_fail_only):
            self._pick_reset_to_search(now, increment_attempt=True)
        else:
            self._pick_reset_to_search(now, increment_attempt=True)

    def _pick_decay_score(self, dt_s: float) -> None:
        self._pick.score = float(self._pick.score) - float(self.pick_fsm_cfg.score_decay_per_s) * max(float(dt_s), 0.0)
        self._pick.score = float(max(min(self._pick.score, 10.0), -10.0))

    def _pick_try_update_anchor(self, world_xyz: Optional[tuple[float, float, float]]) -> bool:
        if world_xyz is None:
            self._pick.consecutive_detection_count = 0
            return False
        p = np.asarray(world_xyz, dtype=float).reshape(3)
        prev = self._pick.anchor_world_xyz
        jump_ok = True
        if prev is not None:
            d = float(np.linalg.norm(p - np.asarray(prev, dtype=float).reshape(3)))
            jump_ok = d <= float(self.pick_fsm_cfg.anchor_jump_limit_m)
        if jump_ok:
            self._pick.consecutive_detection_count += 1
            self._pick.anchor_world_xyz = (float(p[0]), float(p[1]), float(p[2]))
            self._pick.anchor_world_ts = time.time()
            self._pick.anchor_confidence = float(min(1.0, self._pick.anchor_confidence + 0.25))
            return True
        self._pick.consecutive_detection_count = 0
        self._pick.anchor_confidence = float(max(0.0, self._pick.anchor_confidence - 0.15))
        return False

    def _pick_camera_point_to_world(
        self, object_camera_xyz: Sequence[float]
    ) -> Optional[tuple[float, float, float]]:
        p_w = self._camera_to_world_point(np.asarray(object_camera_xyz, dtype=float).reshape(3))
        if p_w is None:
            return None
        return (float(p_w[0]), float(p_w[1]), float(p_w[2]))

    def _pick_resolve_anchor_world(self) -> Optional[tuple[float, float, float]]:
        if self._pick.anchor_world_xyz is not None:
            return self._pick.anchor_world_xyz
        if self._pick.object_world_latest is not None:
            return self._pick.object_world_latest
        if self._pick.object_world_mu is not None:
            return self._pick.object_world_mu
        mu = self._pick_effective_camera_mu()
        if mu is not None:
            world = self._pick_camera_point_to_world(mu)
            if world is not None:
                return world
        if self._pick.last_object_camera is not None:
            world = self._pick_camera_point_to_world(self._pick.last_object_camera)
            if world is not None:
                return world
        if self._pick.object_camera_samples:
            world = self._pick_camera_point_to_world(self._pick.object_camera_samples[-1])
            if world is not None:
                return world
        return None

    def _pick_explain_missing_anchor(self) -> str:
        if (time.time() - float(self._pick.last_perception_ts)) > 1.5:
            return "coarse_no_anchor: no recent perception packets (check host endpoint / Start Perception)"
        if self.last_q is None:
            return "coarse_no_anchor: robot q unavailable (wait for sim/ctrl feedback)"
        if not self.ik_context or self.hand_eye_transform is None:
            return "coarse_no_anchor: hand-eye or IK context missing (check hand_eye_config)"
        if self._pick.last_object_camera is None and not self._pick.object_camera_samples:
            return "coarse_no_anchor: perception connected but no object_camera yet"
        state = str(self._pick.track_state or "").strip().upper()
        if state in ("LOST", "SEARCH", ""):
            return f"coarse_no_anchor: tracker not locked yet (state={state or 'unknown'})"
        return (
            "coarse_no_anchor: cannot transform object to world "
            f"(track={state}, conf={self._pick.track_confidence:.2f})"
        )

    def _pick_bootstrap_anchor_from_perception(self, now: float) -> bool:
        resolved = self._pick_resolve_anchor_world()
        if resolved is None:
            return False
        self._pick.anchor_world_xyz = resolved
        self._pick.anchor_world_ts = float(now)
        self._pick.anchor_confidence = max(float(self._pick.anchor_confidence), 0.5)
        if self._pick.object_world_latest is None:
            self._pick.object_world_latest = resolved
        return True

    def _pick_parse_vec3(self, raw: Any) -> Optional[tuple[float, float, float]]:
        if not isinstance(raw, (list, tuple)) or len(raw) != 3:
            return None
        try:
            return (float(raw[0]), float(raw[1]), float(raw[2]))
        except (TypeError, ValueError):
            return None

    def _pick_tracker_measurement_ok(self) -> bool:
        state = str(self._pick.track_state or "").strip().upper()
        if state in ("LOST", "SEARCH"):
            return False
        if state == "" and self._pick_live_object_camera() is None:
            return False
        if float(self._pick.track_confidence) < float(self.pick_fsm_cfg.track_confidence_min):
            return False
        if float(self._pick.depth_valid_ratio) < float(self.pick_fsm_cfg.depth_valid_ratio_min):
            return False
        return True

    def _pick_effective_camera_mu(self) -> Optional[tuple[float, float, float]]:
        if bool(self.pick_fsm_cfg.use_perception_mu) and self._pick.perception_mu_camera is not None:
            return self._pick.perception_mu_camera
        if self._pick.object_camera_mu is not None:
            return self._pick.object_camera_mu
        return None

    def _pick_apply_perception_mu_sigma(
        self,
        mu_camera: tuple[float, float, float],
        sigma_camera: tuple[float, float, float],
    ) -> None:
        sig = np.asarray(sigma_camera, dtype=float).reshape(3)
        sig = np.maximum(sig, 1e-6)
        cov = np.diag(sig * sig)
        self._pick.perception_mu_camera = tuple(float(v) for v in mu_camera)
        self._pick.perception_sigma_camera = tuple(float(v) for v in sigma_camera)
        self._pick.object_camera_mu = self._pick.perception_mu_camera
        self._pick.object_camera_cov = cov
        self._pick.last_valid_camera_mu = self._pick.object_camera_mu
        self._pick.last_valid_cov = cov
        self._pick.stage_uncertainty = float(np.trace(cov))

    def _pick_record_perception_packet(
        self,
        msg: Dict[str, Any],
        object_camera_xyz: tuple[float, float, float],
        object_world_xyz: Optional[tuple[float, float, float]],
    ) -> None:
        self._pick.last_object_camera = (
            float(object_camera_xyz[0]),
            float(object_camera_xyz[1]),
            float(object_camera_xyz[2]),
        )
        self._pick.track_state = str(msg.get("track_state", self._pick.track_state or "")).strip()
        if "track_confidence" in msg:
            try:
                self._pick.track_confidence = float(msg.get("track_confidence"))
            except (TypeError, ValueError):
                pass
        if "depth_valid_ratio" in msg:
            try:
                self._pick.depth_valid_ratio = float(msg.get("depth_valid_ratio"))
            except (TypeError, ValueError):
                pass
        if "lost_count" in msg:
            try:
                self._pick.track_lost_count = int(msg.get("lost_count", 0))
            except (TypeError, ValueError):
                pass
        self._pick.last_perception_ts = time.time()

        z = float(object_camera_xyz[2])
        depth_ok = float(self.pick_fsm_cfg.depth_min_m) <= z <= float(self.pick_fsm_cfg.depth_max_m)
        if not depth_ok:
            return

        tracker_ok = self._pick_tracker_measurement_ok()
        mu_raw = self._pick_parse_vec3(msg.get("mu_camera", None))
        sigma_raw = self._pick_parse_vec3(msg.get("sigma_camera", None))
        if tracker_ok and mu_raw is not None and sigma_raw is not None:
            self._pick_apply_perception_mu_sigma(mu_raw, sigma_raw)
            self._pick.dropout_count = 0
            self._pick.score = float(self._pick.score) + float(self.pick_fsm_cfg.score_reward_observation)
        else:
            state_up = str(self._pick.track_state or "").strip().upper()
            if state_up in ("LOST", "SEARCH"):
                self._pick_soft_fail()

        window = max(3, int(self.pick_fsm_cfg.relocalize_window))
        self._pick.object_camera_samples.append(
            (float(object_camera_xyz[0]), float(object_camera_xyz[1]), float(object_camera_xyz[2]))
        )
        while len(self._pick.object_camera_samples) > window:
            self._pick.object_camera_samples.popleft()

        if object_world_xyz is not None:
            self._pick.object_world_latest = (
                float(object_world_xyz[0]),
                float(object_world_xyz[1]),
                float(object_world_xyz[2]),
            )
            if tracker_ok:
                if not (mu_raw and sigma_raw):
                    self._pick.dropout_count = 0
                    self._pick.score = float(self._pick.score) + float(self.pick_fsm_cfg.score_reward_observation)
        elif self._pick.last_object_camera is not None:
            world_est = self._pick_camera_point_to_world(self._pick.last_object_camera)
            if world_est is not None:
                self._pick.object_world_latest = world_est

    def _pick_record_perception_sample(
        self, object_camera_xyz: tuple[float, float, float], object_world_xyz: Optional[tuple[float, float, float]]
    ) -> None:
        self._pick_record_perception_packet({}, object_camera_xyz, object_world_xyz)

    def _manual_hold_stage(self, stage: PickStage) -> bool:
        """Manual mode blocks auto stage transitions via _pick_can_auto_advance(), not control ticks."""
        return bool(self._pick.manual_mode) and self._pick.stage == stage

    def _estimate_object_camera_stats(self) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        return filtered_camera_stats(list(self._pick.object_camera_samples), outlier_zscore=float(self.pick_fsm_cfg.outlier_zscore))

    def _camera_to_world_point(self, object_camera_xyz: np.ndarray) -> Optional[np.ndarray]:
        if not self.ik_context or self.hand_eye_transform is None or self.last_q is None:
            return None
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
            p_w = camera_point_to_world(
                self.ik_context,
                q4,
                self.hand_eye_transform,
                np.asarray(object_camera_xyz, dtype=float).reshape(3),
                parent_frame=self.hand_eye_parent_frame,
            )
        except Exception:
            return None
        return np.asarray(p_w, dtype=float).reshape(3)

    def _pick_cmd_world_target(self, world_xyz: tuple[float, float, float], *, seq: int) -> None:
        if self.last_q is None:
            return
        self._pending_target_q = proto.SimQ(
            linear_m=float(world_xyz[0]),
            roll_rad=float(self.last_q.roll_rad),
            theta1_rad=float(self.last_q.theta1_rad),
            theta2_rad=float(self.last_q.theta2_rad),
        )
        self._pending_target_seq = int(seq)
        self.last_ik_target_xyz = (float(world_xyz[0]), float(world_xyz[1]), float(world_xyz[2]))

    def _pick_view_limits(self) -> ViewPregraspLimits:
        cfg = self.pick_fsm_cfg
        return ViewPregraspLimits(
            z_min_m=float(cfg.view_camera_z_min_m),
            z_max_m=float(cfg.view_camera_z_max_m),
            x_abs_max_m=float(cfg.view_camera_x_abs_max_m),
            y_abs_max_m=float(cfg.view_camera_y_abs_max_m),
            z_target_m=float(cfg.view_camera_z_target_m),
        )

    def _pick_current_seed_q(self) -> np.ndarray:
        if self.last_q is not None:
            return np.array(
                [
                    float(self.last_q.linear_m),
                    float(self.last_q.roll_rad),
                    float(self.last_q.theta1_rad),
                    float(self.last_q.theta2_rad),
                ],
                dtype=float,
            )
        return np.array([0.0, 0.0, 0.0, 0.0], dtype=float)

    def _pick_predict_object_camera_at_q(
        self,
        q4: Sequence[float],
        object_world: Sequence[float],
    ) -> Optional[np.ndarray]:
        if not self.ik_context or self.hand_eye_transform is None:
            return None
        try:
            return world_point_to_camera(
                self.ik_context,
                q4,
                self.hand_eye_transform,
                np.asarray(object_world, dtype=float).reshape(3),
                parent_frame=self.hand_eye_parent_frame,
            )
        except Exception:
            return None

    def _pick_solve_pregrasp_ik(
        self,
        pregrasp_world: tuple[float, float, float],
        look_dir_world: Optional[tuple[float, float, float]],
        current_seed: np.ndarray,
        *,
        position_only: bool = False,
    ):
        pos_tol = max(0.005, float(self.pick_fsm_cfg.error_threshold_m))
        target_world = np.asarray(pregrasp_world, dtype=float)
        if position_only:
            return ik_pipeline.solve_then_align(
                target_world=target_world,
                target_dir_world=None,
                context=self.ik_context,
                position_tol_m=pos_tol,
                max_iters=80,
                current_seed=current_seed,
            )
        look_arr = None if look_dir_world is None else np.asarray(look_dir_world, dtype=float).reshape(3)
        ik_res = ik_pipeline.solve_then_align(
            target_world=target_world,
            target_dir_world=look_arr,
            context=self.ik_context,
            position_tol_m=pos_tol,
            max_iters=80,
            current_seed=current_seed,
        )
        if (not ik_res.success) or ik_res.q is None:
            ik_res = ik_pipeline.solve_then_align(
                target_world=target_world,
                target_dir_world=None,
                context=self.ik_context,
                position_tol_m=pos_tol,
                max_iters=80,
                current_seed=current_seed,
            )
        return ik_res

    def _pick_make_current_pose_candidate(
        self,
        object_world: np.ndarray,
    ) -> Optional[ViewPregraspCandidate]:
        if self.last_actual_tip_xyz is None:
            return None
        pre = np.asarray(self.last_actual_tip_xyz, dtype=float).reshape(3)
        look = object_world - pre
        look_n = float(np.linalg.norm(look))
        if look_n <= 1e-9:
            look_dir: tuple[float, float, float] = (1.0, 0.0, 0.0)
        else:
            look_dir = (float(look[0] / look_n), float(look[1] / look_n), float(look[2] / look_n))
        return ViewPregraspCandidate(
            pregrasp_world=(float(pre[0]), float(pre[1]), float(pre[2])),
            look_dir_world=look_dir,
            tag="current_pose",
        )

    def _pick_try_plan_current_pose(
        self,
        obj: np.ndarray,
        *,
        limits: ViewPregraspLimits,
        desired_xy: tuple[float, float],
        live_p: Optional[tuple[float, float, float]],
        look_dot_min: float,
        accept_live_current: bool,
        log: bool,
    ) -> Optional[tuple[ViewPregraspCandidate, np.ndarray, ViewCandidateMetrics]]:
        if "current_pose" in self._pick.coarse_failed_tags:
            return None
        current_cand = self._pick_make_current_pose_candidate(obj)
        if current_cand is None:
            return None
        q = np.asarray(self._pick_current_seed_q(), dtype=float).reshape(4)
        metrics = evaluate_view_candidate(
            q,
            obj,
            ik_context=self.ik_context,
            hand_eye_transform=self.hand_eye_transform,
            parent_frame=self.hand_eye_parent_frame,
            limits=limits,
            desired_xy=desired_xy,
        )
        if metrics is None:
            return None
        if log:
            print(
                f"[view_candidate] {format_view_candidate_log(current_cand, q, metrics, obj, limits=limits)}",
                flush=True,
            )
        if not view_candidate_passes_strict(
            metrics,
            limits=limits,
            look_dot_min=look_dot_min,
            tag="current_pose",
            live_p_camera=live_p,
            accept_current_if_live_visible=accept_live_current,
        ):
            return None
        return current_cand, q, metrics

    def _pick_plan_view_pregrasp(
        self,
        current_seed: np.ndarray,
    ) -> Optional[tuple[Any, np.ndarray, float, Optional[np.ndarray]]]:
        if self._pick.anchor_world_xyz is None or not self.ik_context:
            return None
        if self.hand_eye_transform is None:
            return None
        obj = np.asarray(self._pick.anchor_world_xyz, dtype=float).reshape(3)
        cfg = self.pick_fsm_cfg
        limits = self._pick_view_limits()
        desired_xy = self._pick_desired_camera_xy_for_view_plan()
        live_p = self._pick_live_object_camera()
        look_dot_min = float(cfg.view_look_dot_min)
        log_all = bool(cfg.view_log_all_candidates)
        accept_live_current = bool(cfg.view_accept_current_if_live_visible)

        if live_p is not None and bool(cfg.view_use_live_desired_xy):
            print(
                f"[view_align] plan desired_xy=({desired_xy[0]:+.4f},{desired_xy[1]:+.4f}) "
                f"live_camera=({live_p[0]:+.4f},{live_p[1]:+.4f},{live_p[2]:+.4f})",
                flush=True,
            )

        fast = self._pick_try_plan_current_pose(
            obj,
            limits=limits,
            desired_xy=desired_xy,
            live_p=live_p,
            look_dot_min=look_dot_min,
            accept_live_current=accept_live_current,
            log=log_all,
        )
        if fast is not None:
            cand, q, metrics = fast
            print(
                f"[view_align] selected tag={cand.tag} (fast path) look_dot={metrics.look_dot:.3f} "
                f"visible_pred={metrics.visible_pred} pred_camera={metrics.p_camera} score={metrics.score:.4f}",
                flush=True,
            )
            p_cam = np.asarray(metrics.p_camera, dtype=float).reshape(3)
            return cand, q, float(metrics.score), p_cam

        grid_candidates = generate_view_pregrasp_candidates(
            obj,
            base_offset_m=cfg.coarse_offset_m,
            view_distances_m=cfg.view_distances_m,
            lateral_offsets_m=cfg.view_lateral_offsets_m,
            height_offsets_m=cfg.view_height_offsets_m,
        )
        strict_rows: list[tuple[ViewPregraspCandidate, np.ndarray, ViewCandidateMetrics]] = []
        total_ik_ok = 0
        reject_fov: dict[str, int] = {}
        reject_look_dot = 0
        for cand in grid_candidates:
            if cand.tag in self._pick.coarse_failed_tags:
                continue
            ik_res = self._pick_solve_pregrasp_ik(
                cand.pregrasp_world,
                None,
                current_seed,
                position_only=True,
            )
            if (not ik_res.success) or ik_res.q is None:
                continue
            total_ik_ok += 1
            q = np.asarray(ik_res.q, dtype=float).reshape(4)
            metrics = evaluate_view_candidate(
                q,
                obj,
                ik_context=self.ik_context,
                hand_eye_transform=self.hand_eye_transform,
                parent_frame=self.hand_eye_parent_frame,
                limits=limits,
                desired_xy=desired_xy,
            )
            if metrics is None:
                continue
            if log_all:
                print(
                    f"[view_candidate] {format_view_candidate_log(cand, q, metrics, obj, limits=limits)}",
                    flush=True,
                )
            passed_fk = view_candidate_passes(metrics, limits=limits, look_dot_min=look_dot_min)
            passed = view_candidate_passes_strict(
                metrics,
                limits=limits,
                look_dot_min=look_dot_min,
                tag=str(cand.tag),
                live_p_camera=live_p,
                accept_current_if_live_visible=accept_live_current,
            )
            if passed:
                strict_rows.append((cand, q, metrics))
            else:
                for reason in camera_visibility_fail_reasons(metrics.p_camera, limits):
                    key = reason.split("(", 1)[0]
                    reject_fov[key] = int(reject_fov.get(key, 0)) + 1
                if metrics.visible_pred and float(metrics.look_dot) < look_dot_min:
                    reject_look_dot += 1

        if not strict_rows:
            print(
                f"[view_align] no strict candidate (ik_ok={total_ik_ok} total_tags={len(candidates)} "
                f"look_dot_min={look_dot_min:.2f} fov_reject={reject_fov} look_dot_reject={reject_look_dot})",
                flush=True,
            )
            return None

        best = pick_best_strict_candidate(strict_rows)
        if best is None:
            return None
        cand, q, metrics = best
        live_note = ""
        if (
            str(cand.tag) == "current_pose"
            and live_p is not None
            and accept_live_current
            and camera_visibility_ok(live_p, limits)
            and not view_candidate_passes(metrics, limits=limits, look_dot_min=look_dot_min)
        ):
            live_note = " hold=live_visible"
        print(
            f"[view_align] selected tag={cand.tag}{live_note} look_dot={metrics.look_dot:.3f} "
            f"visible_pred={metrics.visible_pred} pred_camera={metrics.p_camera} "
            f"score={metrics.score:.4f} ({len(strict_rows)} strict / {total_ik_ok} ik_ok)",
            flush=True,
        )
        p_cam = np.asarray(metrics.p_camera, dtype=float).reshape(3)
        return cand, q, float(metrics.score), p_cam

    def _pick_plan_coarse_offset_fallback(
        self,
        current_seed: np.ndarray,
    ) -> Optional[tuple[Any, np.ndarray, float, Optional[np.ndarray]]]:
        if self._pick.anchor_world_xyz is None or not self.ik_context or self.hand_eye_transform is None:
            return None
        obj = np.asarray(self._pick.anchor_world_xyz, dtype=float).reshape(3)
        pre = obj + np.asarray(self.pick_fsm_cfg.coarse_offset_m, dtype=float).reshape(3)
        look = obj - pre
        look_n = float(np.linalg.norm(look))
        if look_n <= 1e-9:
            look_dir = (1.0, 0.0, 0.0)
        else:
            look_dir = (float(look[0] / look_n), float(look[1] / look_n), float(look[2] / look_n))
        cand = ViewPregraspCandidate(
            pregrasp_world=(float(pre[0]), float(pre[1]), float(pre[2])),
            look_dir_world=look_dir,
            tag="coarse_offset_fallback",
        )
        ik_res = self._pick_solve_pregrasp_ik(cand.pregrasp_world, None, current_seed, position_only=True)
        if (not ik_res.success) or ik_res.q is None:
            return None
        q = np.asarray(ik_res.q, dtype=float).reshape(4)
        limits = self._pick_view_limits()
        metrics = evaluate_view_candidate(
            q,
            obj,
            ik_context=self.ik_context,
            hand_eye_transform=self.hand_eye_transform,
            parent_frame=self.hand_eye_parent_frame,
            limits=limits,
            desired_xy=self._pick_desired_camera_xy(),
        )
        if metrics is None or not view_candidate_passes(
            metrics, limits=limits, look_dot_min=float(self.pick_fsm_cfg.view_look_dot_min)
        ):
            return None
        p_cam = np.asarray(metrics.p_camera, dtype=float).reshape(3)
        return cand, q, float(metrics.score), p_cam

    def _pick_coarse_visibility_ok(self) -> bool:
        limits = self._pick_view_limits()
        mu = self._pick_effective_camera_mu()
        if mu is not None and self._pick_tracker_measurement_ok():
            return camera_visibility_ok(mu, limits)
        if self._pick.anchor_world_xyz is None:
            return False
        q4 = self._pick.coarse_target_q
        if q4 is None and self.last_q is not None:
            q4 = (
                float(self.last_q.linear_m),
                float(self.last_q.roll_rad),
                float(self.last_q.theta1_rad),
                float(self.last_q.theta2_rad),
            )
        if q4 is None:
            return False
        p_cam = self._pick_predict_object_camera_at_q(q4, self._pick.anchor_world_xyz)
        if p_cam is None:
            return False
        return camera_visibility_ok(p_cam, limits)

    def _pick_look_limits(self) -> LookAlignLimits:
        cfg = self.pick_fsm_cfg
        return LookAlignLimits(
            xy_threshold_m=float(cfg.look_xy_threshold_m),
            xy_deadband_m=float(cfg.look_xy_deadband_m),
        )

    def _pick_look_gains(self) -> LookGains:
        cfg = self.pick_fsm_cfg
        return LookGains(
            theta1_per_error_x=float(cfg.look_gain_theta1),
            theta2_per_error_y=float(cfg.look_gain_theta2),
            max_step_rad=float(cfg.look_max_step_rad),
        )

    def _pick_jacobian_gains(self) -> JacobianLookGains:
        cfg = self.pick_fsm_cfg
        max_step = float(cfg.look_max_step_rad)
        return JacobianLookGains(
            gain=float(cfg.look_jacobian_gain),
            damping=float(cfg.look_jacobian_damping),
            max_step_roll_rad=max_step,
            max_step_theta_rad=max_step,
            column_norm_min=float(cfg.look_jacobian_column_norm_min),
        )

    def _pick_use_jacobian_servo(self) -> bool:
        if str(self.pick_fsm_cfg.look_servo_mode).strip().lower() != "jacobian":
            return False
        if str(self._pick.look_cal_phase) == "running":
            return False
        if self._pick.look_jacobian is None:
            return False
        return str(self._pick.look_cal_phase) in ("", "done")

    def _pick_current_q_tuple(self) -> Optional[tuple[float, float, float, float]]:
        if self.last_q is None:
            return None
        return (
            float(self.last_q.linear_m),
            float(self.last_q.roll_rad),
            float(self.last_q.theta1_rad),
            float(self.last_q.theta2_rad),
        )

    def _pick_apply_q_tuple(self, q: tuple[float, float, float, float], now: float) -> None:
        self._pending_target_q = proto.SimQ(
            linear_m=float(q[0]),
            roll_rad=float(q[1]),
            theta1_rad=float(q[2]),
            theta2_rad=float(q[3]),
        )
        self._pending_target_seq = int(max(self._pending_target_seq, 0) + 1)
        self._pick_flush_pending_motion_target(now)

    def _pick_measure_camera_error_vector(self) -> Optional[np.ndarray]:
        mu_tuple = self._pick_effective_camera_mu()
        if mu_tuple is None:
            mu_arr, _cov = self._estimate_object_camera_stats()
            if mu_arr is None:
                return None
            mu_tuple = (float(mu_arr[0]), float(mu_arr[1]), float(mu_arr[2]))
        ex, ey, _norm = camera_xy_error(mu_tuple, self._pick_desired_camera_xy())
        return error_vector_2d(ex, ey)

    def _pick_jacobian_axis_eps(self, axis: int) -> float:
        cfg = self.pick_fsm_cfg
        if int(axis) == 0:
            return float(cfg.look_jacobian_eps_roll_rad)
        return float(cfg.look_jacobian_eps_theta_rad)

    def _pick_jacobian_perturb_delta(self, axis: int, eps: float) -> Q4Delta:
        ax = int(axis)
        if ax == 0:
            return Q4Delta(roll_rad=float(eps))
        if ax == 1:
            return Q4Delta(theta1_rad=float(eps))
        if ax == 2:
            return Q4Delta(theta2_rad=float(eps))
        return Q4Delta()

    def _pick_jacobian_calibrate_failed(self, reason: str) -> None:
        self._pick.look_cal_phase = "failed"
        self._pick.look_jacobian = None
        self._pick.look_cal_substep = ""
        print(f"[look_jacobian] calibration failed: {reason} (heuristic fallback)", flush=True)

    def _pick_jacobian_calibrate_tick(self, now: float) -> bool:
        """Run one calibration step. Returns True while calibration still in progress."""
        if str(self._pick.look_cal_phase) != "running":
            return False
        cfg = self.pick_fsm_cfg
        stage_elapsed = float(now - float(self._pick.stage_enter_ts))
        if stage_elapsed > float(cfg.look_jacobian_cal_timeout_s):
            self._pick_jacobian_calibrate_failed("timeout")
            return False
        if not self._pick_tracker_measurement_ok() or self._pick_effective_camera_mu() is None:
            if (now - float(self._pick.look_cal_phase_ts)) > float(cfg.look_jacobian_cal_timeout_s) * 0.5:
                self._pick_jacobian_calibrate_failed("no_track")
            return True
        settle_s = float(cfg.look_jacobian_settle_s)
        settled = (now - float(self._pick.look_cal_phase_ts)) >= settle_s
        axis_order = list(self._pick.look_cal_axis_order or [1, 2])
        if self._pick.look_cal_axis_idx >= len(axis_order):
            self._pick_jacobian_calibrate_failed("axis_index")
            return False
        axis = int(axis_order[self._pick.look_cal_axis_idx])
        axis_name = LOOK_JACOBIAN_AXIS_NAMES[axis] if 0 <= axis < 3 else str(axis)
        sub = str(self._pick.look_cal_substep or "baseline_wait")

        if sub == "baseline_wait":
            if not settled:
                return True
            e0 = self._pick_measure_camera_error_vector()
            if e0 is None:
                return True
            q_now = self._pick_current_q_tuple()
            if q_now is None:
                self._pick_jacobian_calibrate_failed("no_q")
                return False
            self._pick.look_cal_baseline_e = np.asarray(e0, dtype=float).reshape(2)
            self._pick.look_cal_q_anchor = q_now
            eps = self._pick_jacobian_axis_eps(axis)
            delta = self._pick_jacobian_perturb_delta(axis, eps)
            q_pert = apply_q_delta_to_tuple(q_now, delta)
            self._pick_apply_q_tuple(q_pert, now)
            self._pick.look_cal_substep = "measure_wait"
            self._pick.look_cal_phase_ts = float(now)
            print(
                f"[look_jacobian] perturb axis={axis_name} eps={eps:+.4f} "
                f"e0=[{e0[0]:+.4f},{e0[1]:+.4f}]",
                flush=True,
            )
            return True

        if sub == "measure_wait":
            if not settled:
                return True
            e1 = self._pick_measure_camera_error_vector()
            e0 = self._pick.look_cal_baseline_e
            q_anchor = self._pick.look_cal_q_anchor
            if e1 is None or e0 is None or q_anchor is None:
                self._pick_jacobian_calibrate_failed("measure")
                return False
            eps = self._pick_jacobian_axis_eps(axis)
            col = estimate_jacobian_column(e0, e1, eps)
            norm_min = float(cfg.look_jacobian_column_norm_min)
            de_norm = float(np.linalg.norm(np.asarray(e1) - np.asarray(e0)))
            self._pick_apply_q_tuple(q_anchor, now)
            if not jacobian_column_usable(col, norm_min=norm_min):
                print(
                    f"[look_jacobian] skip singular column {axis_name} de_norm={de_norm:.5f} "
                    f"col_norm={float(np.linalg.norm(col)):.6f}",
                    flush=True,
                )
                self._pick.look_cal_substep = "restore_wait"
                self._pick.look_cal_phase_ts = float(now)
                return True
            self._pick.look_cal_columns.append(np.asarray(col, dtype=float).reshape(2))
            print(
                f"[look_jacobian] column {axis_name} de_norm={de_norm:.5f} "
                f"col=[{col[0]:+.5f},{col[1]:+.5f}]",
                flush=True,
            )
            self._pick.look_cal_substep = "restore_wait"
            self._pick.look_cal_phase_ts = float(now)
            return True

        if sub == "restore_wait":
            if not settled:
                return True
            self._pick.look_cal_axis_idx += 1
            if self._pick.look_cal_axis_idx >= len(axis_order):
                if not self._pick.look_cal_columns:
                    self._pick_jacobian_calibrate_failed("no_usable_columns")
                    return False
                try:
                    j_mat = stack_jacobian(self._pick.look_cal_columns)
                except Exception as exc:
                    self._pick_jacobian_calibrate_failed(str(exc))
                    return False
                self._pick.look_jacobian = j_mat
                self._pick.look_cal_phase = "done"
                self._pick.look_cal_substep = ""
                print(f"[look_jacobian] J=\n{j_mat}", flush=True)
                return False
            self._pick.look_cal_substep = "baseline_wait"
            self._pick.look_cal_phase_ts = float(now)
            self._pick.look_cal_baseline_e = None
            self._pick.look_cal_q_anchor = None
            return True

        self._pick.look_cal_substep = "baseline_wait"
        self._pick.look_cal_phase_ts = float(now)
        return True

    def _pick_sync_desired_camera_from_live(self) -> bool:
        """Align LOOK/ADVANCE error target to current perception (not config 0,0)."""
        live = self._pick_live_object_camera()
        if live is None:
            return False
        dz = float(self._pick.desired_camera_object[2])
        self._pick.desired_camera_object = (float(live[0]), float(live[1]), dz)
        print(
            f"[pick] desired_camera_xy synced to live ({live[0]:+.4f},{live[1]:+.4f})",
            flush=True,
        )
        return True

    def _pick_desired_camera_xy(self) -> tuple[float, float]:
        d = self._pick.desired_camera_object
        return (float(d[0]), float(d[1]))

    def _pick_live_object_camera(self) -> Optional[tuple[float, float, float]]:
        if self._pick.last_object_camera is not None:
            return self._pick.last_object_camera
        mu = self._pick_effective_camera_mu()
        if mu is None:
            return None
        return (float(mu[0]), float(mu[1]), float(mu[2]))

    def _pick_desired_camera_xy_for_view_plan(self) -> tuple[float, float]:
        cfg = self.pick_fsm_cfg
        if bool(cfg.view_use_live_desired_xy):
            live = self._pick_live_object_camera()
            if live is not None:
                return (float(live[0]), float(live[1]))
        return self._pick_desired_camera_xy()

    def _pick_camera_xy_state(self) -> Optional[tuple[float, float, float, float]]:
        mu_tuple = self._pick_effective_camera_mu()
        if mu_tuple is None:
            return None
        desired_xy = self._pick_desired_camera_xy()
        ex, ey, norm_xy = camera_xy_error(mu_tuple, desired_xy)
        mu_z = float(mu_tuple[2])
        return ex, ey, norm_xy, mu_z

    def _pick_look_align_ok(self) -> bool:
        state = self._pick_camera_xy_state()
        if state is None:
            return False
        ex, ey, _, _ = state
        return look_align_ok(ex, ey, self._pick_look_limits())

    def _pick_tracker_lost(self) -> bool:
        state = str(self._pick.track_state or "").strip().upper()
        if state == "LOST":
            return True
        if self._pick.stage == PickStage.TARGET_LOCK:
            return False
        if state == "SEARCH":
            return True
        if state == "":
            return self._pick_live_object_camera() is None
        return False

    def _pick_handle_tracker_lost(self, now: float) -> bool:
        if not self._pick_tracker_lost():
            return False
        self._pending_target_q = None
        self._pending_target_u = None
        self._pending_target_axes = set()
        if self.last_q is not None and float(self.pick_fsm_cfg.advance_backoff_m) > 0.0:
            delta = compute_backoff_delta_q(float(self.pick_fsm_cfg.advance_backoff_m))
            lin, roll, t1, t2 = apply_q_delta(
                float(self.last_q.linear_m),
                float(self.last_q.roll_rad),
                float(self.last_q.theta1_rad),
                float(self.last_q.theta2_rad),
                delta,
            )
            self._pending_target_q = proto.SimQ(linear_m=lin, roll_rad=roll, theta1_rad=t1, theta2_rad=t2)
            self._pending_target_seq = int(max(self._pending_target_seq, 0) + 1)
            self._pick_flush_pending_motion_target(now)
            print(
                f"[pick] track lost ({self._pick.track_state}): backoff linear -{self.pick_fsm_cfg.advance_backoff_m:.3f}m",
                flush=True,
            )
        self._pick.advance_count = 0
        self._pick_set_stage(PickStage.TARGET_LOCK, now)
        return True

    def _pick_apply_q_delta_motion(self, delta, now: float) -> None:
        if self.last_q is None:
            return
        lin, roll, t1, t2 = apply_q_delta(
            float(self.last_q.linear_m),
            float(self.last_q.roll_rad),
            float(self.last_q.theta1_rad),
            float(self.last_q.theta2_rad),
            delta,
        )
        self._pending_target_q = proto.SimQ(linear_m=lin, roll_rad=roll, theta1_rad=t1, theta2_rad=t2)
        self._pending_target_seq = int(max(self._pending_target_seq, 0) + 1)
        self._pick_flush_pending_motion_target(now)

    def _pick_visual_servo_look_step(self, now: float) -> bool:
        if self.last_q is None:
            return False
        xy_state = self._pick_camera_xy_state()
        if xy_state is None:
            return False
        ex, ey, norm_xy, _z = xy_state
        self._pick.stage_error_m = float(norm_xy)
        limits = self._pick_look_limits()
        if not should_send_look_command(ex, ey, limits):
            return False
        if (now - float(self._pick.align_last_cmd_ts)) < float(self.pick_fsm_cfg.look_cmd_period_s):
            return False
        if float(norm_xy) > float(self._pick.align_prev_error_m) + 1e-4:
            self._pick.align_no_improve_count += 1
        else:
            self._pick.align_no_improve_count = 0
        self._pick.align_prev_error_m = float(norm_xy)
        cfg = self.pick_fsm_cfg
        if self._pick_use_jacobian_servo():
            e = error_vector_2d(ex, ey)
            j_mat = self._pick.look_jacobian
            assert j_mat is not None
            delta = compute_jacobian_look_delta_q(
                e,
                j_mat,
                self._pick_jacobian_gains(),
                limits=limits,
                include_roll=bool(cfg.look_jacobian_include_roll),
            )
            mode_tag = "jacobian"
        else:
            delta = compute_look_delta_q(ex, ey, self._pick_look_gains(), limits=limits)
            mode_tag = "heuristic"
        if (
            abs(float(delta.linear_m)) < 1e-9
            and abs(float(delta.roll_rad)) < 1e-9
            and abs(float(delta.theta1_rad)) < 1e-9
            and abs(float(delta.theta2_rad)) < 1e-9
        ):
            return False
        self._pick_apply_q_delta_motion(delta, now)
        self._pick.align_last_cmd_ts = float(now)
        print(
            f"[pick] LOOK_ALIGN ({mode_tag}) ex={ex:+.4f} ey={ey:+.4f} norm={norm_xy:.4f} "
            f"droll={delta.roll_rad:+.4f} dt1={delta.theta1_rad:+.4f} dt2={delta.theta2_rad:+.4f}",
            flush=True,
        )
        return True

    def _pick_execute_advance_small(self, now: float) -> bool:
        xy_state = self._pick_camera_xy_state()
        if xy_state is None or self.last_q is None:
            return False
        ex, ey, _, _ = xy_state
        if not advance_allowed(ex, ey, self._pick_look_limits()):
            print(f"[pick] ADVANCE_SMALL blocked: xy not aligned ex={ex:+.4f} ey={ey:+.4f}", flush=True)
            return False
        if int(self._pick.advance_count) >= int(self.pick_fsm_cfg.max_advance_steps):
            print(
                f"[pick] ADVANCE_SMALL blocked: max_advance_steps reached "
                f"(count={self._pick.advance_count}/{self.pick_fsm_cfg.max_advance_steps})",
                flush=True,
            )
            return False
        step_m = float(self.pick_fsm_cfg.advance_step_m)
        delta = compute_advance_delta_q(step_m)
        self._pick_apply_q_delta_motion(delta, now)
        self._pick.advance_count += 1
        print(f"[pick] ADVANCE_SMALL +{step_m:.4f}m (count={self._pick.advance_count})", flush=True)
        return True

    def _pick_ready_for_commit(self) -> bool:
        xy_state = self._pick_camera_xy_state()
        if xy_state is None:
            return False
        ex, ey, _, mu_z = xy_state
        if not look_align_ok(ex, ey, self._pick_look_limits()):
            return False
        if float(mu_z) > float(self.pick_fsm_cfg.commit_z_max_m):
            return False
        return True

    def _pick_apply_coarse_target_q(self, q: np.ndarray, now: float) -> None:
        self._pending_target_q = proto.SimQ(
            linear_m=float(q[0]),
            roll_rad=float(q[1]),
            theta1_rad=float(q[2]),
            theta2_rad=float(q[3]),
        )
        self._pick.coarse_target_q = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        self._pending_target_seq = int(max(self._pending_target_seq, 0) + 1)
        self._pick.coarse_last_cmd_ts = float(now)
        self._t_cmd = 0.0

    def _pick_flush_pending_motion_target(self, now: float) -> None:
        if self._pending_target_q is None:
            return
        if self._pending_target_u is not None and self._pending_target_axes:
            if self._apply_partial_u_target(self._pending_target_u, set(self._pending_target_axes)):
                self._pending_target_u = None
                self._pending_target_axes = set()
                self._pending_target_q = None
            return
        applied_hw, complete = self._apply_sim_q_target(self._pending_target_q)
        if applied_hw:
            self._target_u_state = proto.sim_q_to_control_u(self._pending_target_q, self.cfg)
            if complete:
                self._pending_target_q = None
        elif not self._has_hw():
            q_limited, complete = self._limit_target_q(self._pending_target_q)
            self.last_q = q_limited
            self.last_u = proto.sim_q_to_control_u(q_limited, self.cfg)
            self.last_state_ts = time.time()
            if complete:
                self._pending_target_q = None
        self._t_cmd = float(now)

    def _pick_advance_after_view_align(self, now: float) -> None:
        """When already visible at current_pose, skip STOP_AND_CHECK and start LOOK_ALIGN."""
        if self._pick.stage != PickStage.VIEW_ALIGN:
            return
        if str(self._pick.coarse_candidate_tag or "") != "current_pose":
            return
        if not self._pick_coarse_visibility_ok():
            return
        if not self._pick_can_complete_stage():
            return
        print("[pick] VIEW_ALIGN current_pose visible -> LOOK_ALIGN", flush=True)
        self._pick_set_stage(PickStage.LOOK_ALIGN, now)

    def _pick_execute_coarse_pregrasp(self, now: float, *, force: bool = False) -> bool:
        if self._pick.anchor_world_xyz is None:
            if not self._pick_bootstrap_anchor_from_perception(now):
                print(f"[pick] coarse skipped: {self._pick_explain_missing_anchor()}", flush=True)
                return False
        if not self.ik_context:
            print("[pick] coarse skipped: IK context unavailable", flush=True)
            return False

        current_seed = self._pick_current_seed_q()
        need_plan = bool(force or (not self._pick.coarse_view_planned) or self._pick.coarse_target_q is None)
        if need_plan:
            plan = self._pick_plan_view_pregrasp(current_seed)
            if plan is None:
                plan = self._pick_plan_coarse_offset_fallback(current_seed)
            if plan is None:
                print(
                    "[pick] view_align_no_strict_candidate: no IK solution passes FOV+look_dot",
                    flush=True,
                )
                return False
            cand, q, score, p_cam = plan
            self._pick.pregrasp_world = cand.pregrasp_world
            self._pick.coarse_candidate_tag = str(cand.tag)
            self._pick.coarse_view_score = float(score)
            if p_cam is not None:
                pc = np.asarray(p_cam, dtype=float).reshape(3)
                self._pick.coarse_predicted_camera = (float(pc[0]), float(pc[1]), float(pc[2]))
            else:
                self._pick.coarse_predicted_camera = None
            self._pick.coarse_view_planned = True
            self.last_ik_target_xyz = self._pick.pregrasp_world
            self.last_ik_target_dir = cand.look_dir_world
            self._set_debug_marker(
                name="pregrasp_target",
                pos=self._pick.pregrasp_world,
                color=[1.0, 0.3, 0.2, 0.95],
                radius=0.01,
                ttl_ms=300,
            )
            if (
                self.hand_eye_transform is not None
                and self.ik_context
                and self._pick.anchor_world_xyz is not None
            ):
                view_metrics = evaluate_view_candidate(
                    q,
                    self._pick.anchor_world_xyz,
                    ik_context=self.ik_context,
                    hand_eye_transform=self.hand_eye_transform,
                    parent_frame=self.hand_eye_parent_frame,
                    limits=self._pick_view_limits(),
                    desired_xy=self._pick_desired_camera_xy(),
                )
                if view_metrics is not None:
                    self._set_debug_marker(
                        name="view_align_camera",
                        pos=view_metrics.camera_world,
                        color=[0.2, 0.5, 1.0, 0.95],
                        radius=0.008,
                        ttl_ms=300,
                    )
                    self._set_debug_marker(
                        name="view_align_camera_look",
                        pos=view_metrics.camera_world,
                        direction=np.asarray(view_metrics.camera_look, dtype=float).reshape(3),
                        color=[0.3, 0.7, 1.0, 0.95],
                        radius=0.004,
                        ttl_ms=300,
                    )
            pred_txt = self._pick.coarse_predicted_camera if self._pick.coarse_predicted_camera is not None else "n/a"
            print(
                f"[pick] coarse view-pregrasp tag={cand.tag} score={score:.3f} "
                f"pred_camera={pred_txt} pregrasp={self._pick.pregrasp_world}",
                flush=True,
            )
            self._pick_apply_coarse_target_q(q, now)
            self._pick_flush_pending_motion_target(now)
            self._pick_advance_after_view_align(now)
            return True

        if (not force) and (now - float(self._pick.coarse_last_cmd_ts)) < 0.20:
            return bool(self._pick.coarse_target_q is not None)

        if self._pick.coarse_target_q is not None:
            q = np.asarray(self._pick.coarse_target_q, dtype=float).reshape(4)
            self._pick_apply_coarse_target_q(q, now)
            self._pick_flush_pending_motion_target(now)
            return True
        return False

    def _tick_pick_fsm(self, now: float) -> None:
        if not self._pick_enabled or self._safety_fault:
            return
        stage_elapsed = float(now - self._pick.stage_enter_ts)
        self._pick_decay_score(self._state_period)
        if should_stage_timeout(stage_elapsed_s=stage_elapsed, timeout_s=float(self.pick_fsm_cfg.stage_timeout_s)) and self._pick.stage not in (
            PickStage.TARGET_LOCK,
            PickStage.STOP_AND_CHECK,
            PickStage.LOOK_ALIGN,
            PickStage.LIFT_AND_VERIFY,
        ):
            if self._pick_can_auto_advance():
                self._pick_reset_to_search(now, increment_attempt=True)
            return
        if self._pick.attempt >= int(self.pick_fsm_cfg.max_attempts):
            self._pick_enabled = False
            return
        if self._pick.stage == PickStage.TARGET_LOCK:
            if self._pick.manual_mode:
                return
            if self._pick_handle_tracker_lost(now):
                return
            perception_fresh = (now - float(self._pick.last_perception_ts)) <= 1.0
            tracker_ready = (
                str(self._pick.track_state or "").strip().upper() == "TRACKING_3D"
                and float(self._pick.track_confidence) >= float(self.pick_fsm_cfg.search_track_conf_min)
            )
            _mu_s, cov_s = self._estimate_object_camera_stats()
            stable_cov = bool(
                cov_s is not None and float(np.trace(cov_s)) <= float(self.pick_fsm_cfg.uncertainty_threshold)
            )
            if tracker_ready and self._pick.perception_mu_camera is not None:
                stable_cov = bool(
                    float(self._pick.stage_uncertainty) <= float(self.pick_fsm_cfg.uncertainty_threshold)
                )
            stable = self._pick_try_update_anchor(
                self._pick.object_world_latest if (perception_fresh and stable_cov and (tracker_ready or _mu_s is not None)) else None
            )
            if not stable:
                self._pick_soft_fail()
                return
            if self._pick.consecutive_detection_count >= int(self.pick_fsm_cfg.search_stable_frames):
                self._pick_set_stage(PickStage.VIEW_ALIGN, now)
            return
        if self._pick.stage == PickStage.VIEW_ALIGN:
            if self._pick.anchor_world_xyz is None:
                if not self._pick_bootstrap_anchor_from_perception(now):
                    print(f"[pick] VIEW_ALIGN: {self._pick_explain_missing_anchor()}, returning to TARGET_LOCK", flush=True)
                    self._pick_reset_to_search(now)
                    return
            self._pick_execute_coarse_pregrasp(now, force=False)
            pre = np.asarray(self._pick.pregrasp_world, dtype=float) if self._pick.pregrasp_world is not None else None
            if pre is None:
                return
            # Transition only after actual approach to pregrasp (or timeout), not immediately.
            reach_tol = max(0.02, float(self.pick_fsm_cfg.error_threshold_m) * 2.0)
            reached = str(self._pick.coarse_candidate_tag or "") == "current_pose"
            if not reached and self.last_actual_tip_xyz is not None:
                dist = float(np.linalg.norm(np.asarray(self.last_actual_tip_xyz, dtype=float).reshape(3) - pre))
                self._pick.stage_error_m = dist
                reached = dist <= reach_tol
            elif not reached and self._pick.coarse_target_q is not None and self.last_q is not None:
                q_now = np.array(
                    [
                        float(self.last_q.linear_m),
                        float(self.last_q.roll_rad),
                        float(self.last_q.theta1_rad),
                        float(self.last_q.theta2_rad),
                    ],
                    dtype=float,
                )
                q_goal = np.asarray(self._pick.coarse_target_q, dtype=float).reshape(4)
                reached = float(np.linalg.norm(q_now - q_goal)) <= 0.02
            if reached:
                if self._pick_coarse_visibility_ok():
                    if self._pick_can_complete_stage():
                        if str(self._pick.coarse_candidate_tag or "") == "current_pose":
                            print("[pick] VIEW_ALIGN current_pose visible -> LOOK_ALIGN", flush=True)
                            self._pick_set_stage(PickStage.LOOK_ALIGN, now)
                        else:
                            print("[pick] VIEW_ALIGN complete -> STOP_AND_CHECK", flush=True)
                            self._pick_set_stage(PickStage.STOP_AND_CHECK, now)
                    return
                failed_tag = str(self._pick.coarse_candidate_tag or "").strip()
                if failed_tag:
                    self._pick.coarse_failed_tags.add(failed_tag)
                print(
                    f"[pick] VIEW_ALIGN reached but visibility failed tag={failed_tag or 'unknown'} "
                    f"pred={self._pick.coarse_predicted_camera} track={self._pick.track_state}",
                    flush=True,
                )
                self._pick.coarse_view_planned = False
                self._pick.coarse_target_q = None
                self._pick_execute_coarse_pregrasp(now, force=True)
                if stage_elapsed > float(self.pick_fsm_cfg.stage_timeout_s) and self._pick_can_auto_advance():
                    self._pick_hard_fail(now)
                return
            if self._pick_handle_tracker_lost(now):
                return
            if stage_elapsed > float(self.pick_fsm_cfg.stage_timeout_s):
                if self._pick_can_auto_advance():
                    self._pick_hard_fail(now)
            return
        if self._pick.stage == PickStage.STOP_AND_CHECK:
            if self._pick_handle_tracker_lost(now):
                return
            if not self._pick_tracker_measurement_ok():
                self._pick_soft_fail()
            else:
                mu_eff = self._pick_effective_camera_mu()
                cov = self._pick.object_camera_cov
                if mu_eff is None or cov is None:
                    mu, cov_est = self._estimate_object_camera_stats()
                    if mu is not None and cov_est is not None:
                        self._pick.object_camera_mu = (float(mu[0]), float(mu[1]), float(mu[2]))
                        self._pick.object_camera_cov = cov_est
                        self._pick.stage_uncertainty = float(np.trace(cov_est))
                        mu_eff = self._pick.object_camera_mu
                        cov = cov_est
                if mu_eff is not None and cov is not None:
                    uncertainty = float(np.trace(cov))
                    if uncertainty <= float(self.pick_fsm_cfg.uncertainty_threshold):
                        self._pick.last_valid_camera_mu = mu_eff
                        self._pick.last_valid_cov = cov
                        mu_arr = np.asarray(mu_eff, dtype=float).reshape(3)
                        mu_world = self._camera_to_world_point(mu_arr)
                        if mu_world is not None:
                            self._pick.object_world_mu = (float(mu_world[0]), float(mu_world[1]), float(mu_world[2]))
                            self._pick_try_update_anchor(self._pick.object_world_mu)
                            self._set_debug_marker(
                                name="object_mu_world",
                                pos=self._pick.object_world_mu,
                                color=[0.2, 0.95, 0.8, 0.95],
                                radius=0.01,
                                ttl_ms=300,
                            )
                            if self._pick_can_complete_stage():
                                if int(self._pick.advance_count) >= int(
                                    self.pick_fsm_cfg.max_advance_steps
                                ):
                                    if self._pick_ready_for_commit():
                                        print(
                                            "[pick] STOP_AND_CHECK: max_advance_steps -> COMMIT_GATE",
                                            flush=True,
                                        )
                                        self._pick_set_stage(PickStage.COMMIT_GATE, now)
                                    elif self._pick_can_auto_advance():
                                        self._pick_hard_fail(now)
                                    else:
                                        print(
                                            f"[pick] STOP_AND_CHECK: max_advance_steps "
                                            f"({self._pick.advance_count}/"
                                            f"{self.pick_fsm_cfg.max_advance_steps}); "
                                            "manual: goto ADVANCE_SMALL to continue",
                                            flush=True,
                                        )
                                else:
                                    print("[pick] STOP_AND_CHECK complete -> LOOK_ALIGN", flush=True)
                                    self._pick_set_stage(PickStage.LOOK_ALIGN, now)
                            return
            if stage_elapsed > float(self.pick_fsm_cfg.relocalize_timeout_s):
                if self._pick_can_auto_advance():
                    self._pick_hard_fail(now)
            return
        if self._pick.stage == PickStage.LOOK_ALIGN:
            if self._pick_handle_tracker_lost(now):
                return
            if not self._pick_tracker_measurement_ok():
                self._pick_soft_fail()
            elif self._pick_effective_camera_mu() is not None and self._pick.object_camera_cov is not None:
                self._pick.dropout_count = 0
            else:
                mu_new, cov_new = self._estimate_object_camera_stats()
                if mu_new is not None and cov_new is not None:
                    self._pick.object_camera_mu = (float(mu_new[0]), float(mu_new[1]), float(mu_new[2]))
                    self._pick.object_camera_cov = cov_new
                    self._pick.last_valid_camera_mu = self._pick.object_camera_mu
                    self._pick.last_valid_cov = cov_new
                    self._pick.stage_uncertainty = float(np.trace(cov_new))
                    self._pick.dropout_count = 0
            if self._pick.object_camera_mu is None and self._pick.last_valid_camera_mu is not None:
                self._pick.object_camera_mu = self._pick.last_valid_camera_mu
                self._pick.object_camera_cov = self._pick.last_valid_cov
                self._pick_soft_fail()
            if self._pick_effective_camera_mu() is None:
                self._pick_hard_fail(now)
                return
            xy_state = self._pick_camera_xy_state()
            if xy_state is not None:
                _ex, _ey, norm_xy, _mu_z = xy_state
                self._pick.stage_error_m = float(norm_xy)
            if self._pick.pregrasp_world is not None:
                self.last_ik_target_xyz = (
                    float(self._pick.pregrasp_world[0]),
                    float(self._pick.pregrasp_world[1]),
                    float(self._pick.pregrasp_world[2]),
                )
            if self._pick_ready_for_commit():
                if self._pick_can_complete_stage():
                    self._pick_set_stage(PickStage.COMMIT_GATE, now)
                return
            if self._pick_look_align_ok():
                if int(self._pick.advance_count) >= int(self.pick_fsm_cfg.max_advance_steps):
                    if self._pick_ready_for_commit() and self._pick_can_complete_stage():
                        self._pick_set_stage(PickStage.COMMIT_GATE, now)
                    elif self._pick_can_auto_advance():
                        self._pick_hard_fail(now)
                    return
                if self._pick_can_complete_stage():
                    self._pick_set_stage(PickStage.ADVANCE_SMALL, now)
                return
            if int(self._pick.align_no_improve_count) >= int(self.pick_fsm_cfg.align_no_improve_limit):
                if self._pick_can_complete_stage():
                    self._pick_set_stage(PickStage.STOP_AND_CHECK, now)
                self._pick.align_no_improve_count = 0
                return
            if stage_elapsed > float(self.pick_fsm_cfg.align_timeout_s):
                if should_hard_fail(
                    dropout_count=int(self._pick.dropout_count),
                    dropout_hard_limit=int(self.pick_fsm_cfg.dropout_hard_limit),
                    stage_elapsed_s=stage_elapsed,
                    timeout_s=float(self.pick_fsm_cfg.align_timeout_s),
                    error_m=float(self._pick.stage_error_m),
                ):
                    if self._pick_can_auto_advance():
                        self._pick_hard_fail(now)
                else:
                    self._pick_soft_fail()
                return
            if str(self._pick.look_cal_phase) == "running":
                self._pick_jacobian_calibrate_tick(now)
                return
            self._pick_visual_servo_look_step(now)
            return
        if self._pick.stage == PickStage.ADVANCE_SMALL:
            if self._pick_handle_tracker_lost(now):
                return
            if self._pick_execute_advance_small(now):
                if self._pick_can_complete_stage():
                    self._pick_set_stage(PickStage.STOP_AND_CHECK, now)
            else:
                at_max = int(self._pick.advance_count) >= int(self.pick_fsm_cfg.max_advance_steps)
                if at_max:
                    if self._pick_ready_for_commit() and self._pick_can_complete_stage():
                        self._pick_set_stage(PickStage.COMMIT_GATE, now)
                    elif not self._pick.manual_mode and self._pick_can_complete_stage():
                        self._pick_set_stage(PickStage.STOP_AND_CHECK, now)
                elif self._pick_can_complete_stage():
                    self._pick_set_stage(PickStage.LOOK_ALIGN, now)
            return
        if self._pick.stage == PickStage.COMMIT_GATE:
            if self._pick_handle_tracker_lost(now):
                return
            pass_gate = (
                self._pick_look_align_ok()
                and self._pick_ready_for_commit()
                and should_pass_confidence_gate(
                    error_m=float(self._pick.stage_error_m),
                    uncertainty=float(self._pick.stage_uncertainty),
                    error_threshold_m=float(self.pick_fsm_cfg.look_xy_threshold_m),
                    uncertainty_threshold=float(self.pick_fsm_cfg.uncertainty_threshold),
                )
                and float(self._pick.score) >= float(self.pick_fsm_cfg.score_pass)
                and self._pick_tracker_measurement_ok()
                and float(self._pick.track_confidence) >= float(self.pick_fsm_cfg.track_confidence_min)
                and float(self._pick.depth_valid_ratio) >= float(self.pick_fsm_cfg.depth_valid_ratio_min)
            )
            if pass_gate:
                if self._pick_can_complete_stage():
                    self._pick_set_stage(PickStage.SHORT_APPROACH, now)
            else:
                if int(self._pick.dropout_count) <= int(self.pick_fsm_cfg.dropout_soft_limit):
                    if self._pick_can_complete_stage():
                        self._pick_set_stage(PickStage.LOOK_ALIGN, now)
                    self._pick_soft_fail()
                else:
                    self._pick_hard_fail(now)
            return
        if self._pick.stage == PickStage.SHORT_APPROACH:
            if self._pick.anchor_world_xyz is None:
                self._pick_hard_fail(now)
                return
            obj = np.asarray(self._pick.anchor_world_xyz, dtype=float)
            target = obj + np.array([0.0, 0.0, max(0.0, float(self.pick_fsm_cfg.short_approach_m))], dtype=float)
            self._pick.short_approach_world = (float(target[0]), float(target[1]), float(target[2]))
            self.last_ik_target_xyz = self._pick.short_approach_world
            self._set_debug_marker(
                name="short_approach_target",
                pos=self._pick.short_approach_world,
                color=[0.95, 0.7, 0.1, 0.95],
                radius=0.010,
                ttl_ms=300,
            )
            if self.last_q is None:
                if stage_elapsed > float(self.pick_fsm_cfg.stage_timeout_s):
                    if self._pick_can_auto_advance():
                        self._pick_hard_fail(now)
                return
            reach_tol = max(0.01, float(self.pick_fsm_cfg.error_threshold_m) * 2.0)
            if self.last_actual_tip_xyz is not None:
                dist = float(
                    np.linalg.norm(np.asarray(self.last_actual_tip_xyz, dtype=float).reshape(3) - target)
                )
                self._pick.stage_error_m = dist
                if dist <= reach_tol:
                    if self._pick_can_complete_stage():
                        self._pick_set_stage(PickStage.CLOSE_GRIPPER, now)
                    return
            dz = float(np.clip(float(self.pick_fsm_cfg.short_approach_m) * 0.25, 0.0, float(self.pick_fsm_cfg.align_step_m)))
            self._pending_target_q = proto.SimQ(
                linear_m=float(self.last_q.linear_m + dz),
                roll_rad=float(self.last_q.roll_rad),
                theta1_rad=float(self.last_q.theta1_rad),
                theta2_rad=float(self.last_q.theta2_rad),
            )
            self._pending_target_seq = int(max(self._pending_target_seq, 0) + 1)
            if stage_elapsed > float(self.pick_fsm_cfg.stage_timeout_s):
                if self._pick_can_auto_advance():
                    self._pick_hard_fail(now)
            return
        if self._pick.stage == PickStage.CLOSE_GRIPPER:
            self.last_claw_closed = True
            if self._pick_can_complete_stage():
                self._pick_set_stage(PickStage.LIFT_AND_VERIFY, now)
            return
        if self._pick.stage == PickStage.LIFT_AND_VERIFY:
            if self.last_actual_tip_xyz is None:
                if stage_elapsed > float(self.pick_fsm_cfg.lift_verify_timeout_s):
                    if self._pick_can_auto_advance():
                        self._pick_hard_fail(now)
                return
            cur_z = float(self.last_actual_tip_xyz[2])
            if self._pick.lift_start_z is None:
                self._pick.lift_start_z = cur_z
                if self.last_q is not None:
                    self._pending_target_q = proto.SimQ(
                        linear_m=float(self.last_q.linear_m - float(self.pick_fsm_cfg.lift_height_m)),
                        roll_rad=float(self.last_q.roll_rad),
                        theta1_rad=float(self.last_q.theta1_rad),
                        theta2_rad=float(self.last_q.theta2_rad),
                    )
                    self._pending_target_seq = int(max(self._pending_target_seq, 0) + 1)
            lift_ok = (cur_z - float(self._pick.lift_start_z)) >= float(self.pick_fsm_cfg.lift_height_m) * 0.5
            grip_ok = bool(self.last_claw_closed) and bool(self._claw_close_stalled)
            if lift_ok or grip_ok:
                self._pick.attempt = 0
                self._pick_reset_to_search(now, increment_attempt=False)
                return
            if stage_elapsed > float(self.pick_fsm_cfg.lift_verify_timeout_s):
                self.last_claw_closed = False
                if self._pick_can_auto_advance():
                    self._pick_hard_fail(now)
            return

    def _handle_pick_command(self, ident: bytes, msg: Dict[str, Any]) -> None:
        cmd = str(msg.get("cmd", "")).strip().lower()
        now = time.time()
        if cmd == "start":
            self._pick_enabled = True
            self._pick.manual_mode = False
            self._pick_reset_to_search(now, increment_attempt=False)
            self._reply(
                ident,
                {"t": "ack", "ts": proto.now_s(), "ok": True, "reason": "pick_started", "device": self.device, "torque_enabled": self.torque_enabled},
            )
            return
        if cmd == "stop":
            self._pick_enabled = False
            self._pick.manual_mode = False
            self._pick_reset_to_search(now, increment_attempt=False)
            self._reply(
                ident,
                {"t": "ack", "ts": proto.now_s(), "ok": True, "reason": "pick_stopped", "device": self.device, "torque_enabled": self.torque_enabled},
            )
            return
        if cmd == "reset":
            self._pick.attempt = 0
            self._pick_enabled = bool(self.pick_fsm_cfg.enable)
            self._pick.manual_mode = False
            self._pick_reset_to_search(now, increment_attempt=False)
            self._reply(
                ident,
                {"t": "ack", "ts": proto.now_s(), "ok": True, "reason": "pick_reset", "device": self.device, "torque_enabled": self.torque_enabled},
            )
            return
        if cmd == "goto_stage":
            stage_raw = str(msg.get("stage", "")).strip().upper()
            try:
                target_stage = resolve_pick_stage(stage_raw)
            except Exception:
                self._reply(
                    ident,
                    {"t": "ack", "ts": proto.now_s(), "ok": False, "reason": "bad_stage", "device": self.device, "torque_enabled": self.torque_enabled},
                )
                return
            # Manual stage forcing should also arm FSM.
            self._pick_enabled = True
            self._pick.manual_mode = True
            # If user jumps to coarse without explicit anchor, fallback to latest perceived world point.
            if target_stage == PickStage.VIEW_ALIGN and self._pick.anchor_world_xyz is None:
                if not self._pick_bootstrap_anchor_from_perception(now):
                    reason = self._pick_explain_missing_anchor()
                    self._reply(
                        ident,
                        {
                            "t": "ack",
                            "ts": proto.now_s(),
                            "ok": False,
                            "reason": reason,
                            "device": self.device,
                            "torque_enabled": self.torque_enabled,
                        },
                    )
                    print(f"[pick] goto COARSE rejected: {reason}", flush=True)
                    return
            prev_count = int(self._pick.advance_count)
            if target_stage == PickStage.ADVANCE_SMALL:
                self._pick.advance_count = 0
                if prev_count > 0:
                    print(
                        f"[pick] manual ADVANCE_SMALL: advance_count reset ({prev_count} -> 0)",
                        flush=True,
                    )
            elif (
                target_stage == PickStage.LOOK_ALIGN
                and prev_count >= int(self.pick_fsm_cfg.max_advance_steps)
            ):
                self._pick.advance_count = 0
                print(
                    f"[pick] manual LOOK_ALIGN: advance_count reset ({prev_count} -> 0)",
                    flush=True,
                )
            self._pick_set_stage(target_stage, now)
            print(f"[pick] goto stage {target_stage.value} (manual)", flush=True)
            coarse_ok = True
            if target_stage == PickStage.VIEW_ALIGN:
                coarse_ok = bool(self._pick_execute_coarse_pregrasp(now, force=True))
                if coarse_ok:
                    self._pick_advance_after_view_align(now)
            if target_stage == PickStage.LOOK_ALIGN:
                self._pick.align_last_cmd_ts = 0.0
                self._pick.align_prev_error_m = float("inf")
                self._pick.align_no_improve_count = 0
            if target_stage == PickStage.VIEW_ALIGN and not coarse_ok:
                self._pick_set_stage(PickStage.TARGET_LOCK, now)
                self._reply(
                    ident,
                    {
                        "t": "ack",
                        "ts": proto.now_s(),
                        "ok": False,
                        "reason": "view_align_no_strict_candidate: no FOV+look_dot pregrasp (check anchor/hand-eye/IK)",
                        "device": self.device,
                        "torque_enabled": self.torque_enabled,
                    },
                )
                print("[pick] goto VIEW_ALIGN rejected: view_align_no_strict_candidate", flush=True)
                return
            self._reply(
                ident,
                {
                    "t": "ack",
                    "ts": proto.now_s(),
                    "ok": True,
                    "reason": f"stage_forced:{target_stage.value}",
                    "device": self.device,
                    "torque_enabled": self.torque_enabled,
                },
            )
            return
        if cmd == "set_mode":
            mode_raw = str(msg.get("mode", "")).strip().lower()
            if mode_raw not in ("auto", "manual"):
                self._reply(
                    ident,
                    {"t": "ack", "ts": proto.now_s(), "ok": False, "reason": "bad_mode", "device": self.device, "torque_enabled": self.torque_enabled},
                )
                return
            self._pick.manual_mode = bool(mode_raw == "manual")
            self._reply(
                ident,
                {
                    "t": "ack",
                    "ts": proto.now_s(),
                    "ok": True,
                    "reason": f"mode_set:{mode_raw}",
                    "device": self.device,
                    "torque_enabled": self.torque_enabled,
                },
            )
            return
        self._reply(
            ident,
            {"t": "ack", "ts": proto.now_s(), "ok": False, "reason": "bad_pick_cmd", "device": self.device, "torque_enabled": self.torque_enabled},
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
        if t == "pick_cmd":
            self._handle_pick_command(ident, msg)
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
                if (
                    bool(self.pick_fsm_cfg.ignore_perception_in_short_approach)
                    and self._pick_enabled
                    and self._pick.stage == PickStage.SHORT_APPROACH
                ):
                    self._reply(
                        ident,
                        {
                            "t": "ack",
                            "ts": proto.now_s(),
                            "ok": True,
                            "reason": "perception_ignored_short_approach",
                            "device": self.device,
                            "torque_enabled": self.torque_enabled,
                        },
                    )
                    return
                object_camera_raw = msg.get("object_camera", None)
                if isinstance(object_camera_raw, (list, tuple)) and len(object_camera_raw) == 3:
                    object_camera_xyz = (
                        float(object_camera_raw[0]),
                        float(object_camera_raw[1]),
                        float(object_camera_raw[2]),
                    )
                    ok, reason, object_world = self._update_perception_markers(
                        object_camera_xyz,
                        object_label=str(msg.get("object_label", "")),
                    )
                    object_world_tuple: Optional[tuple[float, float, float]] = None
                    if object_world is not None:
                        p_w = np.asarray(object_world, dtype=float).reshape(3)
                        object_world_tuple = (float(p_w[0]), float(p_w[1]), float(p_w[2]))
                    if self._pick_enabled:
                        self._pick_record_perception_packet(msg, object_camera_xyz, object_world_tuple)
                    else:
                        self._pick_record_perception_sample(object_camera_xyz, object_world_tuple)
                    ack: Dict[str, Any] = {
                        "t": "ack",
                        "ts": proto.now_s(),
                        "ok": bool(ok),
                        "reason": str(reason),
                        "device": self.device,
                        "torque_enabled": self.torque_enabled,
                    }
                    if object_world_tuple is not None:
                        ack["object_world"] = [float(object_world_tuple[0]), float(object_world_tuple[1]), float(object_world_tuple[2])]
                    self._reply(ident, ack)
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
            self._tick_pick_fsm(now)
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
                    elif not self._has_hw():
                        q_limited, complete = self._limit_target_q(self._pending_target_q)
                        self.last_q = q_limited
                        self.last_u = proto.sim_q_to_control_u(q_limited, self.cfg)
                        self.last_state_ts = time.time()
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
                        sag_model=self.last_sag_model,
                        claw_closed=self.last_claw_closed,
                        claw_current=self._last_claw_current,
                        motor_currents_ma={self._motor_name_by_id(int(k)): int(v) for k, v in self._last_motor_current_by_id.items()},
                        safety_fault=(self._safety_fault or None),
                        debug_markers=self._active_debug_markers(),
                        pick_stage=self._pick.stage.value,
                        pick_error_m=(
                            None if (not np.isfinite(float(self._pick.stage_error_m))) else float(self._pick.stage_error_m)
                        ),
                        pick_uncertainty=(
                            None
                            if (not np.isfinite(float(self._pick.stage_uncertainty)))
                            else float(self._pick.stage_uncertainty)
                        ),
                        pick_attempt=int(self._pick.attempt),
                        pick_anchor_age_s=(
                            None
                            if float(self._pick.anchor_world_ts) <= 0.0
                            else float(max(0.0, now - float(self._pick.anchor_world_ts)))
                        ),
                        pick_anchor_confidence=float(self._pick.anchor_confidence),
                        pick_dropout_count=int(self._pick.dropout_count),
                        pick_score=float(self._pick.score),
                        pick_track_state=str(self._pick.track_state or ""),
                        pick_track_confidence=float(self._pick.track_confidence),
                        pick_depth_valid_ratio=float(self._pick.depth_valid_ratio),
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
            pick_fsm_cfg=bundle.pick_fsm_config,
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
