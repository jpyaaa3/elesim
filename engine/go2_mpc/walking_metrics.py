from __future__ import annotations

import csv
import json
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Optional

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from engine.go2_mpc.control_rate import ControlRateInfo
from engine.go2_mpc.genesis_pin_bridge import _quat_wxyz_to_xyzw, _to_numpy_1d


WALKING_CSV_FIELDS = [
    "time_s",
    "go2_cmd_vx",
    "go2_cmd_vy",
    "go2_cmd_wz",
    "command_source",
    "base_pos_x",
    "base_pos_y",
    "base_pos_z",
    "base_roll",
    "base_pitch",
    "base_yaw",
    "base_lin_vel_x",
    "base_lin_vel_y",
    "base_lin_vel_z",
    "base_ang_vel_x",
    "base_ang_vel_y",
    "base_ang_vel_z",
    "arm_linear_m",
    "arm_roll_rad",
    "arm_theta1_rad",
    "arm_theta2_rad",
    "payload_com_body_x",
    "payload_com_body_y",
    "payload_com_body_z",
    "tau_norm",
    "tau_max_abs",
    "tau_saturation_count",
    "tau_saturation_ratio",
    "torque_update_flag",
    "torque_hold_flag",
    "sim_step_count",
    "fall_flag",
]

CAMERA_CSV_FIELDS = [
    "time_s",
    "target_visible",
    "u_err",
    "v_err",
    "bbox_scale",
    "tracking_confidence",
    "target_lost_count",
    "time_since_last_seen",
]


@dataclass
class WalkingMetricsMeta:
    run_id: str
    arm_preset: str = "neutral"
    gaze_enabled: bool = False
    turn_direction: str = "none"
    command_source: str = "teleop"
    pitch_trim_config: dict[str, float] = field(default_factory=dict)
    control_rate: dict[str, float] = field(default_factory=dict)
    extra: dict[str, Any] = field(default_factory=dict)


@dataclass
class WalkingMetricsCounters:
    sim_step_count: int = 0
    torque_update_count: int = 0
    torque_hold_count: int = 0
    tau_saturation_total: int = 0


class WalkingMetricsLogger:
    """Sim-side walking CSV logger (Method B split files)."""

    def __init__(
        self,
        *,
        run_id: str,
        log_dir: str | Path = "logs/walking_baseline",
        meta: Optional[WalkingMetricsMeta] = None,
    ) -> None:
        self.run_id = str(run_id)
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.meta = meta or WalkingMetricsMeta(run_id=self.run_id)
        self.counters = WalkingMetricsCounters()
        self._walking_path = self.log_dir / f"{self.run_id}_walking.csv"
        self._meta_path = self.log_dir / f"{self.run_id}_meta.json"
        self._walking_file = open(self._walking_path, "w", newline="", encoding="utf-8")
        self._walking_writer = csv.DictWriter(self._walking_file, fieldnames=WALKING_CSV_FIELDS)
        self._walking_writer.writeheader()
        self._tau_lim: Optional[np.ndarray] = None
        self._started_at = time.time()
        self._write_meta()

    @classmethod
    def from_env(cls, *, run_id: Optional[str] = None, meta: Optional[WalkingMetricsMeta] = None) -> Optional[WalkingMetricsLogger]:
        if os.environ.get("ELESIM_WALKING_METRICS", "").strip().lower() not in ("1", "true", "yes", "on"):
            return None
        rid = run_id or time.strftime("run_%Y%m%d_%H%M%S")
        return cls(run_id=rid, meta=meta)

    def set_tau_limits(self, tau_lim: np.ndarray) -> None:
        self._tau_lim = np.asarray(tau_lim, dtype=float).reshape(-1)

    def _write_meta(self) -> None:
        payload = asdict(self.meta)
        payload["control_rate"] = dict(self.meta.control_rate)
        payload["pitch_trim_config"] = dict(self.meta.pitch_trim_config)
        with open(self._meta_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)

    def close(self) -> None:
        if not self._walking_file.closed:
            self.counters_snapshot = asdict(self.counters)
            self.meta.extra["counters"] = self.counters_snapshot
            self._write_meta()
            self._walking_file.close()

    def record_torque_step(self, *, recomputed: bool, hold: bool) -> None:
        self.counters.sim_step_count += 1
        if recomputed:
            self.counters.torque_update_count += 1
        if hold:
            self.counters.torque_hold_count += 1

    def sample_go2(
        self,
        *,
        go2_entity,
        go2_cmd: tuple[float, float, float],
        command_source: str,
        arm_q: Optional[tuple[float, float, float, float]] = None,
        payload_com_body: Optional[np.ndarray] = None,
        tau: Optional[np.ndarray] = None,
        torque_update_flag: bool = False,
        torque_hold_flag: bool = False,
        fall_flag: bool = False,
        time_s: Optional[float] = None,
    ) -> None:
        base = go2_entity.get_link("base")
        pos = _to_numpy_1d(base.get_pos())[:3]
        quat_xyzw = _quat_wxyz_to_xyzw(_to_numpy_1d(base.get_quat())[:4])
        rpy = Rot.from_quat(quat_xyzw).as_euler("xyz", degrees=False)
        vel_world = _to_numpy_1d(base.get_vel())[:3]
        ang_world = _to_numpy_1d(base.get_ang())[:3]
        rot = Rot.from_quat(quat_xyzw)
        vel_body = rot.inv().apply(vel_world)
        ang_body = rot.inv().apply(ang_world)

        tau_arr = np.zeros(12, dtype=float) if tau is None else np.asarray(tau, dtype=float).reshape(-1)
        tau_norm = float(np.linalg.norm(tau_arr))
        tau_max_abs = float(np.max(np.abs(tau_arr))) if tau_arr.size else 0.0
        sat_count = 0
        sat_ratio = 0.0
        if self._tau_lim is not None and self._tau_lim.size == tau_arr.size:
            sat = np.abs(tau_arr) >= (0.98 * np.abs(self._tau_lim))
            sat_count = int(np.count_nonzero(sat))
            sat_ratio = float(sat_count / max(1, tau_arr.size))
            self.counters.tau_saturation_total += sat_count

        pc = payload_com_body if payload_com_body is not None else np.zeros(3, dtype=float)
        aq = arm_q if arm_q is not None else (0.0, 0.0, 0.0, 0.0)

        row = {
            "time_s": float(time_s if time_s is not None else time.time() - self._started_at),
            "go2_cmd_vx": float(go2_cmd[0]),
            "go2_cmd_vy": float(go2_cmd[1]),
            "go2_cmd_wz": float(go2_cmd[2]),
            "command_source": str(command_source),
            "base_pos_x": float(pos[0]),
            "base_pos_y": float(pos[1]),
            "base_pos_z": float(pos[2]),
            "base_roll": float(rpy[0]),
            "base_pitch": float(rpy[1]),
            "base_yaw": float(rpy[2]),
            "base_lin_vel_x": float(vel_body[0]),
            "base_lin_vel_y": float(vel_body[1]),
            "base_lin_vel_z": float(vel_body[2]),
            "base_ang_vel_x": float(ang_body[0]),
            "base_ang_vel_y": float(ang_body[1]),
            "base_ang_vel_z": float(ang_body[2]),
            "arm_linear_m": float(aq[0]),
            "arm_roll_rad": float(aq[1]),
            "arm_theta1_rad": float(aq[2]),
            "arm_theta2_rad": float(aq[3]),
            "payload_com_body_x": float(pc[0]),
            "payload_com_body_y": float(pc[1]),
            "payload_com_body_z": float(pc[2]),
            "tau_norm": tau_norm,
            "tau_max_abs": tau_max_abs,
            "tau_saturation_count": sat_count,
            "tau_saturation_ratio": sat_ratio,
            "torque_update_flag": int(bool(torque_update_flag)),
            "torque_hold_flag": int(bool(torque_hold_flag)),
            "sim_step_count": int(self.counters.sim_step_count),
            "fall_flag": int(bool(fall_flag)),
        }
        self._walking_writer.writerow(row)


class CameraMetricsLogger:
    """Ctrl/perception-side camera CSV logger (Method B)."""

    def __init__(
        self,
        *,
        run_id: str,
        log_dir: str | Path = "logs/walking_baseline",
    ) -> None:
        self.run_id = str(run_id)
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._path = self.log_dir / f"{self.run_id}_camera.csv"
        self._file = open(self._path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=CAMERA_CSV_FIELDS)
        self._writer.writeheader()
        self._target_lost_count = 0
        self._last_visible_time: Optional[float] = None
        self._started_at = time.time()

    @classmethod
    def from_env(cls, *, run_id: str) -> Optional[CameraMetricsLogger]:
        if os.environ.get("ELESIM_WALKING_METRICS", "").strip().lower() not in ("1", "true", "yes", "on"):
            return None
        return cls(run_id=run_id)

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()

    def sample(
        self,
        *,
        target_visible: bool,
        u_err: float,
        v_err: float,
        bbox_scale: float = 0.0,
        tracking_confidence: float = 0.0,
        time_s: Optional[float] = None,
    ) -> None:
        now = float(time_s if time_s is not None else time.time() - self._started_at)
        if target_visible:
            self._last_visible_time = now
        else:
            self._target_lost_count += 1
        since = 0.0 if self._last_visible_time is None else max(0.0, now - self._last_visible_time)
        self._writer.writerow(
            {
                "time_s": now,
                "target_visible": int(bool(target_visible)),
                "u_err": float(u_err),
                "v_err": float(v_err),
                "bbox_scale": float(bbox_scale),
                "tracking_confidence": float(tracking_confidence),
                "target_lost_count": int(self._target_lost_count),
                "time_since_last_seen": float(since),
            }
        )


def detect_fall(base_pos_z: float, base_pitch: float, *, z_min: float = 0.12, pitch_max_rad: float = 0.85) -> bool:
    return float(base_pos_z) < float(z_min) or abs(float(base_pitch)) > float(pitch_max_rad)


def control_rate_meta(info: ControlRateInfo) -> dict[str, float]:
    return {
        "sim_hz_est": float(info.sim_hz),
        "ctrl_hz_config": float(info.ctrl_hz_config),
        "ctrl_hz_effective": float(info.ctrl_hz_effective),
        "ctrl_decim": float(info.ctrl_decim),
    }
