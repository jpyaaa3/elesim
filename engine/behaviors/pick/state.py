from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Any, Optional, Tuple

from engine.core.protocol import ControlU, SimQ


@dataclass(frozen=True)
class HostState:
    connected: bool
    tx_seq: int
    rx_age_s: float
    device: str
    ports: tuple[str, ...]
    torque_enabled: bool
    claw_current: int
    motor_currents_ma: dict[str, int]
    safety_fault: str
    actual_tip_xyz: Optional[tuple[float, float, float]]
    actual_tip_dir: Optional[tuple[float, float, float]]
    perceived_object_label: str
    perceived_object_confidence: float
    perceived_object_camera_xyz: Optional[tuple[float, float, float]]
    perceived_center_uv: Optional[tuple[float, float]]
    perceived_scale: Optional[float]
    perceived_timestamp_s: float
    reply_ok: bool
    reply_reason: str
    q: Optional[SimQ]
    u: Optional[ControlU]
    sim_q: Optional[SimQ] = None
    sim_u: Optional[ControlU] = None
    go2_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
    go2_base_rpy: Optional[tuple[float, float, float]] = None
    go2_base_pos: Optional[tuple[float, float, float]] = None
    go2_sim_base_pos: Optional[tuple[float, float, float]] = None
    go2_base_lin_vel_body: Optional[tuple[float, float, float]] = None
    go2_base_ang_vel: Optional[tuple[float, float, float]] = None
    go2_base_timestamp_s: float = 0.0
    go2_gait_phase: Optional[float] = None
    go2_gait_period_s: Optional[float] = None
    host_state_age_s: float = -1.0
    go2_leg_q: Optional[tuple[float, ...]] = None
    go2_leg_dq: Optional[tuple[float, ...]] = None
    go2_leg_torque_nm: Optional[tuple[float, ...]] = None
    go2_sport_pose: str = ""
    go2_sport_pose_seq: int = 0
    go2_obstacles_avoid_enabled: bool = False
    go2_obstacles_avoid_seq: int = 0
    sim_time_s: float = 0.0
    sim_wall_elapsed_s: float = 0.0
    sim_realtime_factor: float = 0.0
    sim_step_count: int = 0
    perception_running: bool = False
    perception_failed: bool = False
    perception_status: str = ""
    perception_source: str = ""
    perception_preview_endpoint: str = ""
    perception_recording: bool = False
    perception_record_with_overlay: bool = False
    perception_last_record_path: str = ""
    perception_last_capture_path: str = ""
    perception_hz: float = 0.0
    gaze_running: bool = False
    gaze_mode: str = "idle"
    gaze_status_msg: str = ""
    gaze_u_err: float = 0.0
    gaze_v_err: float = 0.0
    gaze_du_roll: float = 0.0
    gaze_du_s1: float = 0.0
    gaze_du_s2: float = 0.0
    gaze_tick_count: int = 0
    gaze_update_count: int = 0
    gaze_obs_age_s: float = -1.0
    gaze_config: dict[str, Any] = field(default_factory=dict)
    pick_running: bool = False
    pick_failed: bool = False
    pick_phase: str = "idle"
    pick_status_msg: str = ""


@dataclass
class PanelState:
    linear: float = 0.0
    roll: float = 0.0
    theta1: float = 0.0
    theta2: float = 0.0
    u_offset_linear: float = 0.0
    u_offset_roll: float = 0.0
    u_offset_s1: float = 0.0
    u_offset_s2: float = 0.0
    offset_revision: int = 0
    paused: bool = False
    claw_closed: bool = False
    torque_lock_bypass: bool = False
    visual_target_scale: float = 0.16
    visual_center_tol: float = 0.12
    visual_target_uv_u: float = 0.5
    visual_target_uv_v: float = 0.0
    visual_scale_tol: float = 0.01
    visual_confidence_min: float = 0.0
    visual_target_label: str = ""
    visual_ready_distance_m: float = 0.20
    visual_look_distance_m: float = 0.30

    perception_running: bool = False
    perception_failed: bool = False
    perception_status_msg: str = ""
    perception_frame_idx: int = 0
    perception_label: str = ""
    perception_confidence: float = 0.0
    perception_camera_xyz: Optional[tuple[float, float, float]] = None
    perception_world_xyz: Optional[tuple[float, float, float]] = None
    perception_tracker_phase: str = "search"
    perception_track_ok_frames: int = 0
    perception_image_scale: float = 0.0
    perception_bbox_wh: tuple[int, int] = (0, 0)
    perception_tracker_backend: str = ""
    perception_last_capture_path: str = ""
    perception_recording: bool = False
    perception_record_with_overlay: bool = False
    perception_last_record_path: str = ""
    perception_center_uv: Optional[tuple[float, float]] = None
    perception_last_update_s: float = 0.0
    perception_hz: float = 0.0

    gaze_running: bool = False
    gaze_mode: str = "idle"
    gaze_status_msg: str = ""
    gaze_u_err: float = 0.0
    gaze_v_err: float = 0.0
    gaze_du_roll: float = 0.0
    gaze_du_s1: float = 0.0
    gaze_du_s2: float = 0.0
    gaze_tick_count: int = 0
    gaze_update_count: int = 0
    gaze_obs_age_s: float = -1.0

    mock_object_x: float = 0.5
    mock_object_y: float = 0.0
    mock_object_z: float = 1.2
    mock_object_dir_x: float = 1.0
    mock_object_dir_y: float = 0.0
    mock_object_dir_z: float = 0.0

    pick_running: bool = False
    pick_failed: bool = False
    pick_phase: str = "idle"
    pick_status_msg: str = ""

    target_x: float = 0.50
    target_y: float = 0.00
    target_z: float = 1.00
    target_vx: float = 1.0
    target_vy: float = 0.0
    target_vz: float = 0.0
    sag_model_path: str = ""
    raw_sag_model: Optional[dict[str, Any]] = None

    ik_running: bool = False
    ik_converged: bool = False
    ik_failed: bool = False
    ik_err_m: float = 0.0
    ik_status_msg: str = ""
    ik_sim_tip_err_m: float = 0.0
    ik_track_roll_err_rad: float = 0.0
    ik_track_theta1_err_rad: float = 0.0
    ik_track_theta2_err_rad: float = 0.0
    ik_track_bend_max_err_rad: float = 0.0

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

    def offset_values(self) -> Tuple[float, float, float, float, int]:
        with self._lock:
            return (
                float(self.u_offset_linear),
                float(self.u_offset_roll),
                float(self.u_offset_s1),
                float(self.u_offset_s2),
                int(self.offset_revision),
            )

    def set_u_offset(self, axis: str, value: float) -> None:
        key = str(axis).strip().lower()
        with self._lock:
            if key == "linear":
                self.u_offset_linear = float(value)
            elif key == "roll":
                self.u_offset_roll = float(value)
            elif key == "s1":
                self.u_offset_s1 = float(value)
            elif key == "s2":
                self.u_offset_s2 = float(value)
            else:
                raise ValueError(f"unknown offset axis: {axis}")
            self.offset_revision += 1

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

    def set_torque_lock_bypass(self, enabled: bool) -> None:
        with self._lock:
            self.torque_lock_bypass = bool(enabled)

    def set_ik_status(self, running: bool, converged: bool, failed: bool, err_m: float, msg: str = "") -> None:
        with self._lock:
            self.ik_running = bool(running)
            self.ik_converged = bool(converged)
            self.ik_failed = bool(failed)
            self.ik_err_m = float(err_m)
            self.ik_status_msg = str(msg)

    def clear_ik_status(self) -> None:
        self.set_ik_status(running=False, converged=False, failed=False, err_m=0.0, msg="")

    def set_perception_status(
        self,
        *,
        running: bool,
        failed: bool,
        msg: str,
        frame_idx: int = 0,
        label: str = "",
        confidence: float = 0.0,
        camera_xyz: Optional[tuple[float, float, float]] = None,
        world_xyz: Optional[tuple[float, float, float]] = None,
        tracker_phase: str = "",
        track_ok_frames: int = 0,
        image_scale: float = -1.0,
        bbox_wh: Optional[tuple[int, int]] = None,
        tracker_backend: str = "",
        center_uv: Optional[tuple[float, float]] = None,
        perception_hz: Optional[float] = None,
    ) -> None:
        with self._lock:
            self.perception_running = bool(running)
            self.perception_failed = bool(failed)
            self.perception_status_msg = str(msg)
            self.perception_frame_idx = int(frame_idx)
            self.perception_label = str(label)
            self.perception_confidence = float(confidence)
            self.perception_camera_xyz = None if camera_xyz is None else tuple(camera_xyz)
            if world_xyz is not None:
                self.perception_world_xyz = tuple(world_xyz)
            elif not self.pick_running:
                self.perception_world_xyz = None
            if str(tracker_phase).strip():
                self.perception_tracker_phase = str(tracker_phase)
            self.perception_track_ok_frames = int(track_ok_frames)
            if float(image_scale) >= 0.0:
                self.perception_image_scale = float(image_scale)
            if bbox_wh is not None:
                self.perception_bbox_wh = (int(bbox_wh[0]), int(bbox_wh[1]))
            if str(tracker_backend).strip():
                self.perception_tracker_backend = str(tracker_backend)
            if center_uv is not None:
                self.perception_center_uv = (float(center_uv[0]), float(center_uv[1]))
            if perception_hz is not None:
                self.perception_hz = max(0.0, float(perception_hz))
            elif not bool(running):
                self.perception_hz = 0.0
            if bool(running) and int(frame_idx) > 0:
                self.perception_last_update_s = float(time.time())

    def set_gaze_status(
        self,
        *,
        running: bool,
        mode: str = "",
        msg: str = "",
        u_err: Optional[float] = None,
        v_err: Optional[float] = None,
        du_roll: Optional[float] = None,
        du_s1: Optional[float] = None,
        du_s2: Optional[float] = None,
        obs_age_s: Optional[float] = None,
        tick_count: Optional[int] = None,
        update_count: Optional[int] = None,
    ) -> None:
        with self._lock:
            self.gaze_running = bool(running)
            if str(mode).strip():
                self.gaze_mode = str(mode)
            if msg is not None:
                self.gaze_status_msg = str(msg)
            if u_err is not None:
                self.gaze_u_err = float(u_err)
            if v_err is not None:
                self.gaze_v_err = float(v_err)
            if du_roll is not None:
                self.gaze_du_roll = float(du_roll)
            if du_s1 is not None:
                self.gaze_du_s1 = float(du_s1)
            if du_s2 is not None:
                self.gaze_du_s2 = float(du_s2)
            if obs_age_s is not None:
                self.gaze_obs_age_s = float(obs_age_s)
            if tick_count is not None:
                self.gaze_tick_count = int(tick_count)
            if update_count is not None:
                self.gaze_update_count = int(update_count)

    def set_perception_last_capture(self, path: str) -> None:
        with self._lock:
            self.perception_last_capture_path = str(path)

    def set_perception_recording(self, recording: bool, path: str = "") -> None:
        with self._lock:
            self.perception_recording = bool(recording)
            if str(path).strip():
                self.perception_last_record_path = str(path)

    def set_perception_record_overlay(self, enabled: bool) -> None:
        with self._lock:
            self.perception_record_with_overlay = bool(enabled)

    def clear_perception_status(self) -> None:
        self.set_perception_status(running=False, failed=False, msg="")

    def mock_object_world_xyz(self) -> tuple[float, float, float]:
        with self._lock:
            return (float(self.mock_object_x), float(self.mock_object_y), float(self.mock_object_z))

    def mock_object_preferred_dir(self) -> tuple[float, float, float]:
        with self._lock:
            return (
                float(self.mock_object_dir_x),
                float(self.mock_object_dir_y),
                float(self.mock_object_dir_z),
            )

    def set_mock_object_world_xyz(self, x: float, y: float, z: float) -> None:
        with self._lock:
            self.mock_object_x = float(x)
            self.mock_object_y = float(y)
            self.mock_object_z = float(z)

    def set_mock_object_preferred_dir(self, x: float, y: float, z: float) -> None:
        with self._lock:
            self.mock_object_dir_x = float(x)
            self.mock_object_dir_y = float(y)
            self.mock_object_dir_z = float(z)

    def set_pick_status(
        self,
        *,
        running: bool,
        failed: bool,
        phase: str,
        msg: str = "",
    ) -> None:
        with self._lock:
            self.pick_running = bool(running)
            self.pick_failed = bool(failed)
            self.pick_phase = str(phase)
            self.pick_status_msg = str(msg)

    def clear_pick_status(self) -> None:
        self.set_pick_status(running=False, failed=False, phase="idle", msg="")

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
        with self._lock:
            self.ik_sim_tip_err_m = float(sim_tip_err_m)
            self.ik_track_roll_err_rad = float(roll_err_rad)
            self.ik_track_theta1_err_rad = float(theta1_err_rad)
            self.ik_track_theta2_err_rad = float(theta2_err_rad)
            self.ik_track_bend_max_err_rad = float(bend_max_err_rad)
