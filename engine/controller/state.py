from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any, Optional, Tuple

from engine.protocol import ControlU, SimQ


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
    calibration_running: bool = False
    calibration_status_msg: str = ""
    visual_running: bool = False
    visual_failed: bool = False
    visual_status_msg: str = ""
    visual_target_scale: float = 0.16
    visual_center_tol: float = 0.10
    visual_target_uv_u: float = 0.5
    visual_target_uv_v: float = -0.5
    visual_scale_tol: float = 0.01
    visual_confidence_min: float = 0.0
    visual_target_label: str = ""

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

    def set_calibration_status(self, *, running: bool, msg: str) -> None:
        with self._lock:
            self.calibration_running = bool(running)
            self.calibration_status_msg = str(msg)

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

    def set_ik_status(self, running: bool, converged: bool, failed: bool, err_m: float, msg: str = "") -> None:
        with self._lock:
            self.ik_running = bool(running)
            self.ik_converged = bool(converged)
            self.ik_failed = bool(failed)
            self.ik_err_m = float(err_m)
            self.ik_status_msg = str(msg)

    def clear_ik_status(self) -> None:
        self.set_ik_status(running=False, converged=False, failed=False, err_m=0.0, msg="")

    def set_visual_status(self, *, running: bool, failed: bool, msg: str) -> None:
        with self._lock:
            self.visual_running = bool(running)
            self.visual_failed = bool(failed)
            self.visual_status_msg = str(msg)

    def clear_visual_status(self) -> None:
        self.set_visual_status(running=False, failed=False, msg="")

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

    def clear_perception_status(self) -> None:
        self.set_perception_status(running=False, failed=False, msg="")

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
