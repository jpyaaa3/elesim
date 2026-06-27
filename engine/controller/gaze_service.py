from __future__ import annotations

import threading
import time
from dataclasses import replace
from typing import TYPE_CHECKING, Optional

import numpy as np

from engine.controller.control_ownership import ControlOwner, ControlOwnership, ControlOwnershipError
from engine.gaze_stabilizer.controller import GazeStabilizer, GazeStabilizerConfig
from engine.profile.walking_metrics import CameraMetricsLogger
from engine.protocol import ControlU

if TYPE_CHECKING:
    from engine.controller.actions import ControlService


class GazeControlService:
    """UV-only / walking gaze stabilizer worker with control ownership."""

    def __init__(self, parent: ControlService, config: GazeStabilizerConfig) -> None:
        self._parent = parent
        self._config = config
        self._stabilizer = GazeStabilizer(config)
        self._ownership = ControlOwnership()
        self._worker: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._mode = "idle"
        self._camera_logger: Optional[CameraMetricsLogger] = None
        self._run_id = ""

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

    def start_standing_uv_only(self, *, run_id: str = "") -> None:
        cfg = GazeStabilizerConfig(
            enable_feedback=True,
            enable_base_ff=False,
            uv_gain=self._config.uv_gain,
            max_du_roll=self._config.max_du_roll,
            max_du_s1=self._config.max_du_s1,
            max_du_s2=self._config.max_du_s2,
            hz=float(self._config.hz),
        )
        self._start(mode="standing", owner=ControlOwner.GAZE_TRACK, config=cfg, run_id=run_id)

    def start_walking_gaze(self, *, run_id: str = "") -> None:
        self._start(mode="walking", owner=ControlOwner.WALK_APPROACH, config=self._config, run_id=run_id)

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

    def _start(self, *, mode: str, owner: ControlOwner, config: GazeStabilizerConfig, run_id: str) -> None:
        if self.is_running:
            return
        if not self._ownership.can_start(owner):
            raise ControlOwnershipError(f"cannot start gaze: owner={self._ownership.owner.value}")
        self._ownership.acquire(owner)
        self._stop.clear()
        self._mode = mode
        self._stabilizer = GazeStabilizer(config)
        self._run_id = run_id or time.strftime("gaze_%Y%m%d_%H%M%S")
        self._camera_logger = CameraMetricsLogger.from_env(run_id=self._run_id)
        self._update_count = 0
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

    def _worker_loop(self) -> None:
        period = self._worker_period_s()
        owner = self._ownership.owner
        try:
            while not self._stop.is_set():
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
                if self._camera_logger is not None:
                    self._camera_logger.sample(
                        target_visible=True,
                        u_err=u_err,
                        v_err=v_err,
                        bbox_scale=float(obs.scale or 0.0),
                        tracking_confidence=float(obs.confidence or 0.0),
                    )

                ff_du = self._walking_base_ff_du(host)
                mode, current_u, next_u, _, _ = self._parent.apply_gaze_uv_correction(
                    obs,
                    extra_du=ff_du if np.any(np.abs(ff_du) > 1e-9) else None,
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
                if mode != "none" and mode != "settling":
                    self._update_count += 1
                elif self._go2_is_moving(host):
                    status_msg = "tracking (go2 motion)"
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
