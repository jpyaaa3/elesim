"""Pilot-side camera and gaze diagnostics."""

from __future__ import annotations

import csv
import os
import time
from pathlib import Path
from typing import Optional


CAMERA_CSV_FIELDS = [
    "wall_time_s",
    "time_s",
    "sim_time_s",
    "host_go2_base_timestamp_s",
    "host_state_age_s",
    "target_visible",
    "u_err",
    "v_err",
    "bbox_scale",
    "tracking_confidence",
    "target_lost_frame_count",
    "target_lost_event_count",
    "target_lost_count",
    "time_since_last_seen",
    "preview_used",
    "preview_fallback",
    "preview_fallback_reason",
    "gait_phase",
    "gait_phase_future",
    "gait_template_u_now",
    "gait_template_v_now",
    "gait_template_u_future",
    "gait_template_v_future",
    "preview_term_u",
    "preview_term_v",
    "preview_dt_s",
    "pitch_rate",
    "pitch_rate_lead",
    "pitch_acc_est",
    "b_pitch",
    "du_roll",
    "du_s1",
    "du_s2",
    "preview_solve_time_ms",
]


def env_run_id(explicit: Optional[str] = None) -> str:
    run_id = str(explicit or "").strip()
    if run_id:
        return run_id
    configured = os.environ.get("ELESIM_RUN_ID", "").strip()
    return configured or time.strftime("run_%Y%m%d_%H%M%S")


class CameraMetricsLogger:
    def __init__(self, *, run_id: str, log_dir: str | Path = "logs/walking_baseline") -> None:
        self.run_id = str(run_id)
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._file = open(self.log_dir / f"{self.run_id}_camera.csv", "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=CAMERA_CSV_FIELDS)
        self._writer.writeheader()
        self._target_lost_frame_count = 0
        self._target_lost_event_count = 0
        self._was_visible = True
        self._last_visible_time: Optional[float] = None
        self._started_at = time.time()

    @classmethod
    def from_env(cls, *, run_id: str) -> Optional["CameraMetricsLogger"]:
        enabled = os.environ.get("ELESIM_WALKING_METRICS", "").strip().lower()
        return cls(run_id=env_run_id(run_id)) if enabled in {"1", "true", "yes", "on"} else None

    def close(self) -> None:
        if not self._file.closed:
            self._file.close()

    def sample(
        self,
        *,
        target_visible: bool,
        u_err: Optional[float] = None,
        v_err: Optional[float] = None,
        bbox_scale: float = 0.0,
        tracking_confidence: float = 0.0,
        time_s: Optional[float] = None,
        wall_time_s: Optional[float] = None,
        sim_time_s: Optional[float] = None,
        host_go2_base_timestamp_s: Optional[float] = None,
        host_state_age_s: Optional[float] = None,
        preview_used: Optional[int] = None,
        preview_fallback: Optional[int] = None,
        preview_fallback_reason: str = "",
        preview_dt_s: Optional[float] = None,
        pitch_rate: Optional[float] = None,
        pitch_rate_lead: Optional[float] = None,
        pitch_acc_est: Optional[float] = None,
        b_pitch: Optional[float] = None,
        du_roll: Optional[float] = None,
        du_s1: Optional[float] = None,
        du_s2: Optional[float] = None,
        preview_solve_time_ms: Optional[float] = None,
        gait_phase: Optional[float] = None,
        gait_phase_future: Optional[float] = None,
        gait_template_u_now: Optional[float] = None,
        gait_template_v_now: Optional[float] = None,
        gait_template_u_future: Optional[float] = None,
        gait_template_v_future: Optional[float] = None,
        preview_term_u: Optional[float] = None,
        preview_term_v: Optional[float] = None,
    ) -> None:
        wall_time = float(
            wall_time_s
            if wall_time_s is not None
            else (time_s if time_s is not None else time.time() - self._started_at)
        )
        visible = bool(target_visible)
        if visible:
            self._last_visible_time = wall_time
        else:
            self._target_lost_frame_count += 1
            if self._was_visible:
                self._target_lost_event_count += 1
        self._was_visible = visible
        since_seen = 0.0 if self._last_visible_time is None else max(0.0, wall_time - self._last_visible_time)

        optional_float = lambda value: "" if value is None else float(value)
        self._writer.writerow(
            {
                "wall_time_s": wall_time,
                "time_s": wall_time,
                "sim_time_s": optional_float(sim_time_s),
                "host_go2_base_timestamp_s": optional_float(host_go2_base_timestamp_s),
                "host_state_age_s": optional_float(host_state_age_s),
                "target_visible": int(visible),
                "u_err": optional_float(u_err),
                "v_err": optional_float(v_err),
                "bbox_scale": float(bbox_scale),
                "tracking_confidence": float(tracking_confidence),
                "target_lost_frame_count": self._target_lost_frame_count,
                "target_lost_event_count": self._target_lost_event_count,
                "target_lost_count": self._target_lost_frame_count,
                "time_since_last_seen": since_seen,
                "preview_used": "" if preview_used is None else int(preview_used),
                "preview_fallback": "" if preview_fallback is None else int(preview_fallback),
                "preview_fallback_reason": str(preview_fallback_reason or ""),
                "gait_phase": optional_float(gait_phase),
                "gait_phase_future": optional_float(gait_phase_future),
                "gait_template_u_now": optional_float(gait_template_u_now),
                "gait_template_v_now": optional_float(gait_template_v_now),
                "gait_template_u_future": optional_float(gait_template_u_future),
                "gait_template_v_future": optional_float(gait_template_v_future),
                "preview_term_u": optional_float(preview_term_u),
                "preview_term_v": optional_float(preview_term_v),
                "preview_dt_s": optional_float(preview_dt_s),
                "pitch_rate": optional_float(pitch_rate),
                "pitch_rate_lead": optional_float(pitch_rate_lead),
                "pitch_acc_est": optional_float(pitch_acc_est),
                "b_pitch": optional_float(b_pitch),
                "du_roll": optional_float(du_roll),
                "du_s1": optional_float(du_s1),
                "du_s2": optional_float(du_s2),
                "preview_solve_time_ms": optional_float(preview_solve_time_ms),
            }
        )
