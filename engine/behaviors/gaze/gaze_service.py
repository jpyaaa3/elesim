from __future__ import annotations

import os
import threading
import time
from dataclasses import replace
from typing import TYPE_CHECKING, Optional

import numpy as np

from engine.behaviors.pick.control_ownership import (
    ControlOwner,
    ControlOwnership,
    ControlOwnershipError,
    ControlState,
)
from engine.behaviors.gaze.stabilizer import GazeStabilizer, GazeStabilizerConfig
from engine.behaviors.gaze.gait_phase_preview import (
    GaitPhasePreviewModel,
    resolve_gait_period_s,
    resolve_gait_phase,
)
from engine.behaviors.gaze.preview_lite import PitchLeadEstimator, resolve_pitch_rate
from engine.behaviors.gaze.preview_mpc import solve_preview_du
from engine.observability.walking_metrics import CameraMetricsLogger, _env_run_id
from engine.vision.visual_servoing.uv_jacobian import default_uv_jacobian

if TYPE_CHECKING:
    from engine.behaviors.pick.actions import ControlService


class GazeControlService:
    """UV-only / walking gaze stabilizer worker with control ownership."""

    def __init__(
        self,
        parent: ControlService,
        config: GazeStabilizerConfig,
        *,
        ownership: Optional[ControlOwnership] = None,
        ownership_enable: bool = False,
    ) -> None:
        self._parent = parent
        self._config = config
        self._stabilizer = GazeStabilizer(config)
        self._ownership_enable = bool(ownership_enable)
        self._ownership = ownership or ControlOwnership()
        self._worker: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._mode = "idle"
        self._gaze_mode = "off"
        self._camera_logger: Optional[CameraMetricsLogger] = None
        self._run_id = ""
        self._update_count = 0
        self._gaze_ticks = 0
        self._preview_used_ticks = 0
        self._preview_fallback_ticks = 0
        self._gait_model: Optional[GaitPhasePreviewModel] = None
        self._gait_period_s = 0.0
        self._gait_wall_t0_s = 0.0

    def _target_visible_live(self, u_err: Optional[float], v_err: Optional[float], scale: float) -> bool:
        del scale
        return u_err is not None and v_err is not None

    def preview_tick_stats(self) -> dict[str, float]:
        ticks = max(0, int(self._gaze_ticks))
        denom = float(max(1, ticks))
        return {
            "gaze_ticks": float(ticks),
            "preview_used_ticks": float(self._preview_used_ticks),
            "preview_fallback_ticks": float(self._preview_fallback_ticks),
            "preview_used_ratio": float(self._preview_used_ticks) / denom,
            "preview_fallback_ratio": float(self._preview_fallback_ticks) / denom,
        }

    @property
    def ownership(self) -> ControlOwnership:
        return self._ownership

    @property
    def is_running(self) -> bool:
        return self._worker is not None and self._worker.is_alive()

    def stop(self) -> None:
        self._stop.set()
        if self._worker is not None:
            self._worker.join(timeout=2.0)
        self._worker = None
        owner = self._ownership.owner
        if owner in (ControlOwner.GAZE_TRACK, ControlOwner.WALK_APPROACH):
            self._ownership.release(owner)
        if self._camera_logger is not None:
            self._camera_logger.close()
            self._camera_logger = None
        self._parent.reset_gaze_derivative_state()
        self._parent.state.set_gaze_status(running=False, mode="idle", msg="stopped")
        self._gaze_mode = "off"

    def start_standing_uv_only(self, *, run_id: str = "") -> None:
        cfg = GazeStabilizerConfig(
            enable_feedback=True,
            enable_base_ff=False,
            uv_gain=self._config.uv_gain,
            max_du_roll=self._config.max_du_roll,
            max_du_s1=self._config.max_du_s1,
            max_du_s2=self._config.max_du_s2,
            hz=float(self._config.hz),
            jacobian_damping=self._config.jacobian_damping,
        )
        self._start(mode="standing", owner=ControlOwner.GAZE_TRACK, config=cfg, run_id=run_id, gaze_mode="uv")

    def start_walking_gaze(self, *, run_id: str = "", gaze_mode: str = "uv_ff") -> None:
        mode = str(gaze_mode).strip().lower()
        if mode == "off":
            raise ValueError("gaze_mode=off should not start worker")
        if mode == "uv":
            cfg = replace(self._config, enable_feedback=True, enable_base_ff=False)
        elif mode == "uv_ff":
            cfg = replace(self._config, enable_feedback=True, enable_base_ff=True)
        elif mode == "preview":
            if not bool(self._config.gait_preview_enable):
                raise ValueError("gaze_mode=preview requires gaze_gait_preview_enable=true")
            cfg = replace(self._config, enable_feedback=True, enable_base_ff=False)
        elif mode == "pitch_preview":
            if not bool(self._config.preview_enable):
                raise ValueError("gaze_mode=pitch_preview requires gaze_preview_enable=true")
            cfg = replace(self._config, enable_feedback=True, enable_base_ff=False)
        else:
            cfg = self._config
        self._start(
            mode="walking",
            owner=ControlOwner.WALK_APPROACH,
            config=cfg,
            run_id=run_id,
            gaze_mode=mode,
        )

    def start_stop_and_grasp_demo(self) -> None:
        if self._worker is not None:
            return

        def _worker() -> None:
            try:
                self.start_walking_gaze()
                time.sleep(3.0)
                self._parent.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
                time.sleep(0.5)
                self.stop()
                if hasattr(self._parent, "start_look_aim_grasp_e2e"):
                    self._parent.start_look_aim_grasp_e2e()
            except Exception as exc:
                print(f"[gaze] stop-and-grasp demo failed: {exc}")

        self._worker = threading.Thread(target=_worker, name="gaze-demo4", daemon=True)
        self._worker.start()

    def _resolve_run_id(self, run_id: str) -> str:
        return _env_run_id(run_id)

    def _start(
        self,
        *,
        mode: str,
        owner: ControlOwner,
        config: GazeStabilizerConfig,
        run_id: str,
        gaze_mode: str,
    ) -> None:
        if self.is_running:
            return
        if not self._ownership.can_start(owner):
            raise ControlOwnershipError(f"cannot start gaze: owner={self._ownership.owner.value}")
        state = ControlState.GAZE_TRACK if owner == ControlOwner.GAZE_TRACK else ControlState.WALK_APPROACH
        self._ownership.acquire(owner, state=state)
        self._stop.clear()
        self._mode = mode
        self._gaze_mode = gaze_mode
        self._stabilizer = GazeStabilizer(config)
        self._run_id = self._resolve_run_id(run_id)
        unified = os.environ.get("ELESIM_WALKING_UNIFIED_CAMERA", "").strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        )
        self._camera_logger = None if unified else CameraMetricsLogger.from_env(run_id=self._run_id)
        self._update_count = 0
        self._gaze_ticks = 0
        self._preview_used_ticks = 0
        self._preview_fallback_ticks = 0
        self._gait_model = None
        self._gait_period_s = 0.0
        self._gait_wall_t0_s = 0.0
        self._pitch_lead = PitchLeadEstimator()
        self._prev_pitch_rad: Optional[float] = None
        if gaze_mode == "pitch_preview":
            if not bool(config.preview_enable):
                raise ValueError("gaze_mode=pitch_preview requires gaze_preview_enable=true")
        elif gaze_mode == "preview":
            tmpl_path = str(config.gait_template_path or "").strip()
            if not tmpl_path:
                raise ValueError("gaze_mode=preview requires gaze_gait_template_path")
            self._gait_model = GaitPhasePreviewModel.load(tmpl_path)
            meta_period = float(self._gait_model.template.gait_period_s)
            cfg_period = resolve_gait_period_s(
                gait_period_s=float(config.gait_period_s),
                gait_hz=2.5,
            )
            if meta_period > 0.0:
                self._gait_period_s = meta_period
            elif cfg_period > 0.0:
                self._gait_period_s = cfg_period
            else:
                raise ValueError("invalid gait_period_s for preview")
            if cfg_period > 0.0 and meta_period > 0.0 and abs(cfg_period - meta_period) > 0.05:
                print(
                    f"[gaze] warning: config gait_period_s={cfg_period:.3f} "
                    f"differs from template {meta_period:.3f}"
                )
        self._parent.reset_gaze_derivative_state()
        self._parent.state.set_gaze_status(
            running=True,
            mode=str(mode),
            msg="started",
            u_err=0.0,
            v_err=0.0,
            du_roll=0.0,
            du_s1=0.0,
            du_s2=0.0,
            obs_age_s=-1.0,
            update_count=0,
        )
        self._worker = threading.Thread(target=self._worker_loop, name=f"gaze-{mode}", daemon=True)
        self._worker.start()

    def _worker_period_s(self) -> float:
        return 1.0 / max(1.0, float(self._stabilizer.config.hz))

    def _extract_obs(self, host):
        return self._parent.current_visual_observation(host)

    def _host_timing(self, host) -> tuple[Optional[float], Optional[float]]:
        if host is None:
            return None, None
        sim_ts = float(host.go2_base_timestamp_s) if float(host.go2_base_timestamp_s) > 0.0 else None
        age = float(host.rx_age_s) if host.connected else None
        return sim_ts, age

    def _log_camera_sample(
        self,
        *,
        host,
        target_visible: bool,
        u_err: Optional[float],
        v_err: Optional[float],
        bbox_scale: float = 0.0,
        tracking_confidence: float = 0.0,
        preview_used: Optional[int] = None,
        preview_fallback: Optional[int] = None,
        preview_fallback_reason: str = "",
        preview_dt_s: Optional[float] = None,
        pitch_rate: Optional[float] = None,
        pitch_rate_lead: Optional[float] = None,
        pitch_acc_est: Optional[float] = None,
        b_pitch: Optional[float] = None,
        preview_term_u: Optional[float] = None,
        preview_term_v: Optional[float] = None,
        gait_phase: Optional[float] = None,
        gait_phase_future: Optional[float] = None,
        gait_template_u_now: Optional[float] = None,
        gait_template_v_now: Optional[float] = None,
        gait_template_u_future: Optional[float] = None,
        gait_template_v_future: Optional[float] = None,
        du_roll: Optional[float] = None,
        du_s1: Optional[float] = None,
        du_s2: Optional[float] = None,
        preview_solve_time_ms: Optional[float] = None,
    ) -> None:
        if self._camera_logger is None:
            return
        sim_ts, age = self._host_timing(host)
        self._camera_logger.sample(
            target_visible=target_visible,
            u_err=u_err,
            v_err=v_err,
            bbox_scale=bbox_scale,
            tracking_confidence=tracking_confidence,
            wall_time_s=time.time(),
            sim_time_s=sim_ts,
            host_go2_base_timestamp_s=sim_ts,
            host_state_age_s=age,
            preview_used=preview_used,
            preview_fallback=preview_fallback,
            preview_fallback_reason=preview_fallback_reason,
            preview_dt_s=preview_dt_s,
            pitch_rate=pitch_rate,
            pitch_rate_lead=pitch_rate_lead,
            pitch_acc_est=pitch_acc_est,
            b_pitch=b_pitch,
            preview_term_v=preview_term_v,
            gait_phase=gait_phase,
            gait_phase_future=gait_phase_future,
            gait_template_u_now=gait_template_u_now,
            gait_template_v_now=gait_template_v_now,
            gait_template_u_future=gait_template_u_future,
            gait_template_v_future=gait_template_v_future,
            preview_term_u=preview_term_u,
            du_roll=du_roll,
            du_s1=du_s1,
            du_s2=du_s2,
            preview_solve_time_ms=preview_solve_time_ms,
        )

    def _go2_is_moving(self, host) -> bool:
        if host is None:
            return False
        vx, vy, wz = (float(host.go2_vel[0]), float(host.go2_vel[1]), float(host.go2_vel[2]))
        return abs(vx) > 0.04 or abs(vy) > 0.04 or abs(wz) > 0.04

    def _walking_base_ff_du(self, host) -> np.ndarray:
        moving = self._go2_is_moving(host)
        cfg = self._stabilizer.config
        if host is None or (not cfg.enable_base_ff and not moving):
            return np.zeros(3, dtype=float)
        base_ang = host.go2_base_ang_vel
        if base_ang is None:
            return np.zeros(3, dtype=float)
        ff_cfg = cfg
        if moving and not cfg.enable_base_ff:
            ff_cfg = replace(self._config, enable_base_ff=True)
        return GazeStabilizer(ff_cfg).compute_display_u_delta(
            uv_error=np.zeros(2, dtype=float),
            jacobian=np.eye(2, 3, dtype=float),
            base_ang_vel_body=np.asarray(base_ang, dtype=float),
        )

    def _clip_preview_du(self, du: np.ndarray) -> np.ndarray:
        cfg = self._stabilizer.config
        out = np.asarray(du, dtype=float).reshape(3)
        roll_lim = float(min(cfg.preview_max_du_roll, cfg.max_du_roll))
        seg_lim = float(min(cfg.preview_max_du_seg, cfg.max_du_s1, cfg.max_du_s2))
        out[0] = float(np.clip(out[0], -roll_lim, roll_lim))
        out[1] = float(np.clip(out[1], -seg_lim, seg_lim))
        out[2] = float(np.clip(out[2], -seg_lim, seg_lim))
        return out

    def _apply_preview_solve(
        self,
        *,
        host,
        obs,
        u_err: float,
        v_err: float,
        period: float,
        preview_term: np.ndarray,
        diag: dict[str, float],
    ) -> tuple[bool, str, dict[str, float]]:
        cfg = self._stabilizer.config
        jacobian = default_uv_jacobian(
            center_u_gain=float(cfg.center_u_gain),
            center_v_gain=float(cfg.center_v_gain),
        )
        result = solve_preview_du(
            jacobian=jacobian,
            s=np.array([u_err, v_err], dtype=float),
            preview_term=np.asarray(preview_term, dtype=float).reshape(2),
            q_u=float(cfg.preview_q_u),
            q_v=float(cfg.preview_q_v),
            r_roll=float(cfg.preview_r_roll),
            r_s1=float(cfg.preview_r_s1),
            r_s2=float(cfg.preview_r_s2),
        )
        diag["preview_solve_time_ms"] = float(result.solve_time_ms)
        diag["preview_term_u"] = float(preview_term.reshape(2)[0])
        diag["preview_term_v"] = float(preview_term.reshape(2)[1])
        if not result.ok:
            return False, str(result.reason or "solve_fail"), diag

        du = self._clip_preview_du(result.du)
        mode, current_u, next_u, _, _ = self._parent.apply_gaze_preview_correction(
            obs,
            du=du,
            dt_s=period,
        )
        roll_du = float(next_u.u_roll - current_u.u_roll)
        s1_du = float(next_u.u_s1 - current_u.u_s1)
        s2_du = float(next_u.u_s2 - current_u.u_s2)
        diag["du_roll"] = roll_du
        diag["du_s1"] = s1_du
        diag["du_s2"] = s2_du
        if mode == "none":
            return False, "no_motion", diag
        self._update_count += 1
        self._parent.state.set_gaze_status(
            running=True,
            mode=str(self._mode),
            msg="tracking (preview_mpc)",
            u_err=float(u_err),
            v_err=float(v_err),
            du_roll=roll_du,
            du_s1=s1_du,
            du_s2=s2_du,
            obs_age_s=float(obs.age_s),
            update_count=int(self._update_count),
        )
        return True, "", diag

    def _try_pitch_preview_step(
        self,
        host,
        obs,
        u_err: float,
        v_err: float,
        period: float,
    ) -> tuple[bool, str, dict[str, float]]:
        cfg = self._stabilizer.config
        diag: dict[str, float] = {}
        if host is None:
            return False, "missing_host", diag
        ts = float(getattr(host, "go2_base_timestamp_s", 0.0) or 0.0)
        if ts <= 0.0:
            return False, "missing_ts", diag
        pitch_rate, pitch_rad = resolve_pitch_rate(
            host,
            prev_pitch_rad=self._prev_pitch_rad,
            prev_go2_base_timestamp_s=self._pitch_lead.prev_go2_base_timestamp_s,
            worker_period_s=period,
        )
        if pitch_rad is not None:
            self._prev_pitch_rad = float(pitch_rad)
        if pitch_rate is None:
            return False, "missing_pitch_rate", diag
        lead = self._pitch_lead.update(
            pitch_rate=float(pitch_rate),
            go2_base_timestamp_s=ts,
            worker_period_s=period,
            tau_s=float(cfg.preview_tau_s),
            lowpass_alpha=float(cfg.preview_lowpass_alpha),
        )
        if not lead.ok:
            return False, str(lead.reason or "pitch_lead_fail"), diag
        diag.update(
            {
                "preview_dt_s": float(lead.preview_dt_s),
                "pitch_rate": float(lead.pitch_rate_filtered),
                "pitch_rate_lead": float(lead.pitch_rate_lead),
                "pitch_acc_est": float(lead.pitch_acc_est),
                "b_pitch": float(cfg.preview_b_pitch),
            }
        )
        preview_term = np.array([0.0, float(cfg.preview_b_pitch) * float(lead.pitch_rate_lead)], dtype=float)
        return self._apply_preview_solve(
            host=host,
            obs=obs,
            u_err=u_err,
            v_err=v_err,
            period=period,
            preview_term=preview_term,
            diag=diag,
        )

    def _try_gait_preview_step(
        self,
        host,
        obs,
        u_err: float,
        v_err: float,
        period: float,
    ) -> tuple[bool, str, dict[str, float]]:
        cfg = self._stabilizer.config
        diag: dict[str, float] = {}
        if host is None:
            return False, "missing_host", diag
        if self._gait_model is None:
            return False, "template_lookup_fail", diag
        ts = float(getattr(host, "go2_base_timestamp_s", 0.0) or 0.0)
        if ts <= 0.0:
            return False, "missing_ts", diag

        wall_t = time.time()
        if self._gait_wall_t0_s <= 0.0:
            self._gait_wall_t0_s = wall_t

        host_phase = getattr(host, "go2_gait_phase", None)
        period_s = float(self._gait_period_s)
        if host.go2_gait_period_s is not None and float(host.go2_gait_period_s) > 0.0:
            period_s = float(host.go2_gait_period_s)

        phase_now, _src = resolve_gait_phase(
            host_gait_phase=float(host_phase) if host_phase is not None else None,
            sim_time_s=ts,
            wall_time_s=wall_t,
            wall_t0_s=self._gait_wall_t0_s,
            gait_period_s=period_s,
            phase_offset=float(cfg.gait_phase_offset),
        )
        if phase_now is None:
            return False, "missing_gait_phase", diag

        delta = self._gait_model.preview_delta(
            phase_now,
            scale=float(cfg.gait_preview_scale),
            horizon_s=float(cfg.gait_preview_horizon_s),
            period_s=period_s,
        )
        diag.update(
            {
                "gait_phase": float(delta.phase_now),
                "gait_phase_future": float(delta.phase_future),
                "gait_template_u_now": float(delta.d_now[0]),
                "gait_template_v_now": float(delta.d_now[1]),
                "gait_template_u_future": float(delta.d_future[0]),
                "gait_template_v_future": float(delta.d_future[1]),
                "preview_term_u": float(delta.preview_term[0]),
                "preview_term_v": float(delta.preview_term[1]),
            }
        )
        if not delta.ok:
            return False, str(delta.reason or "template_lookup_fail"), diag

        return self._apply_preview_solve(
            host=host,
            obs=obs,
            u_err=u_err,
            v_err=v_err,
            period=period,
            preview_term=np.asarray(delta.preview_term, dtype=float).reshape(2),
            diag=diag,
        )

    def _worker_loop(self) -> None:
        period = self._worker_period_s()
        owner = self._ownership.owner
        try:
            while not self._stop.is_set():
                if self._ownership_enable:
                    try:
                        self._ownership.heartbeat(owner)
                    except ControlOwnershipError:
                        break
                host = self._parent.refresh_host_state() if self._parent.client is not None else None
                obs = self._extract_obs(host)
                if obs is None or obs.center_uv is None:
                    age_s = -1.0
                    reason = "no host connection"
                    if host is not None:
                        if float(host.perceived_timestamp_s) > 0.0:
                            age_s = max(0.0, time.time() - float(host.perceived_timestamp_s))
                        if host.perceived_center_uv is None:
                            reason = "waiting for host UV (Start Perception?)"
                        elif host.perceived_scale is None:
                            reason = "host missing scale (detection incomplete?)"
                        elif age_s > float(self._parent._visual_obs_stale_s):
                            reason = f"host UV stale ({age_s:.1f}s)"
                        else:
                            reason = "host UV filtered (label/confidence mismatch?)"
                    self._log_camera_sample(host=host, target_visible=False, u_err=None, v_err=None)
                    self._parent.state.set_gaze_status(
                        running=True,
                        mode=str(self._mode),
                        msg=reason,
                        obs_age_s=age_s,
                    )
                    self._parent.reset_gaze_derivative_state()
                    time.sleep(period)
                    continue

                u_err = float(obs.center_uv[0]) - float(self._parent.state.visual_target_uv_u)
                v_err = float(obs.center_uv[1]) - float(self._parent.state.visual_target_uv_v)
                self._gaze_ticks += 1

                preview_diag: dict[str, float] = {}
                used_preview = False
                fallback_reason = ""
                if self._gaze_mode in ("preview", "pitch_preview"):
                    if self._gaze_mode == "pitch_preview":
                        used_preview, fallback_reason, preview_diag = self._try_pitch_preview_step(
                            host, obs, u_err, v_err, period
                        )
                    else:
                        used_preview, fallback_reason, preview_diag = self._try_gait_preview_step(
                            host, obs, u_err, v_err, period
                        )
                    if used_preview:
                        self._preview_used_ticks += 1
                        self._log_camera_sample(
                            host=host,
                            target_visible=self._target_visible_live(u_err, v_err, float(obs.scale or 0.0)),
                            u_err=u_err,
                            v_err=v_err,
                            bbox_scale=float(obs.scale or 0.0),
                            tracking_confidence=float(obs.confidence or 0.0),
                            preview_used=1,
                            preview_fallback=0,
                            preview_fallback_reason="",
                            gait_phase=preview_diag.get("gait_phase"),
                            gait_phase_future=preview_diag.get("gait_phase_future"),
                            gait_template_u_now=preview_diag.get("gait_template_u_now"),
                            gait_template_v_now=preview_diag.get("gait_template_v_now"),
                            gait_template_u_future=preview_diag.get("gait_template_u_future"),
                            gait_template_v_future=preview_diag.get("gait_template_v_future"),
                            preview_term_u=preview_diag.get("preview_term_u"),
                            preview_term_v=preview_diag.get("preview_term_v"),
                            preview_dt_s=preview_diag.get("preview_dt_s"),
                            pitch_rate=preview_diag.get("pitch_rate"),
                            pitch_rate_lead=preview_diag.get("pitch_rate_lead"),
                            pitch_acc_est=preview_diag.get("pitch_acc_est"),
                            b_pitch=preview_diag.get("b_pitch"),
                            du_roll=preview_diag.get("du_roll"),
                            du_s1=preview_diag.get("du_s1"),
                            du_s2=preview_diag.get("du_s2"),
                            preview_solve_time_ms=preview_diag.get("preview_solve_time_ms"),
                        )
                        time.sleep(period)
                        continue
                    self._preview_fallback_ticks += 1

                ff_du = (
                    self._walking_base_ff_du(host)
                    if self._mode == "walking" and self._gaze_mode not in ("preview", "pitch_preview")
                    else None
                )
                mode, current_u, next_u, _, _ = self._parent.apply_gaze_uv_correction(
                    obs,
                    extra_du=ff_du if ff_du is not None and np.any(np.abs(ff_du) > 1e-9) else None,
                    dt_s=period,
                )
                roll_du = float(next_u.u_roll - current_u.u_roll)
                s1_du = float(next_u.u_s1 - current_u.u_s1)
                s2_du = float(next_u.u_s2 - current_u.u_s2)
                status_msg = "centered" if mode == "none" else "tracking"
                if mode == "settling":
                    status_msg = "settling (plant lag)"
                elif mode == "pd_damp":
                    status_msg = "damping"
                elif mode == "gain_u_seg":
                    status_msg = "tracking (u→seg)"
                elif mode == "gaze_stabilizer":
                    status_msg = "tracking (gaze_stabilizer)"
                if mode not in ("none", "settling"):
                    self._update_count += 1
                elif self._go2_is_moving(host):
                    status_msg = "tracking (go2 motion)"
                if self._gaze_mode in ("preview", "pitch_preview") and not used_preview:
                    status_msg = f"preview fallback ({fallback_reason})"
                self._log_camera_sample(
                    host=host,
                    target_visible=self._target_visible_live(u_err, v_err, float(obs.scale or 0.0)),
                    u_err=u_err,
                    v_err=v_err,
                    bbox_scale=float(obs.scale or 0.0),
                    tracking_confidence=float(obs.confidence or 0.0),
                    preview_used=0,
                    preview_fallback=1 if self._gaze_mode in ("preview", "pitch_preview") else 0,
                    preview_fallback_reason=fallback_reason if self._gaze_mode in ("preview", "pitch_preview") else "",
                    gait_phase=preview_diag.get("gait_phase"),
                    gait_phase_future=preview_diag.get("gait_phase_future"),
                    gait_template_u_now=preview_diag.get("gait_template_u_now"),
                    gait_template_v_now=preview_diag.get("gait_template_v_now"),
                    gait_template_u_future=preview_diag.get("gait_template_u_future"),
                    gait_template_v_future=preview_diag.get("gait_template_v_future"),
                    preview_term_u=preview_diag.get("preview_term_u"),
                    preview_term_v=preview_diag.get("preview_term_v"),
                    preview_solve_time_ms=preview_diag.get("preview_solve_time_ms"),
                )
                self._parent.state.set_gaze_status(
                    running=True,
                    mode=str(self._mode),
                    msg=status_msg,
                    u_err=float(u_err),
                    v_err=float(v_err),
                    du_roll=roll_du,
                    du_s1=s1_du,
                    du_s2=s2_du,
                    obs_age_s=float(obs.age_s),
                    update_count=int(self._update_count),
                )
                time.sleep(period)
        finally:
            if owner in (ControlOwner.GAZE_TRACK, ControlOwner.WALK_APPROACH):
                self._ownership.release(owner)
