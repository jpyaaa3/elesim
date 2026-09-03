"""Gaze-control commands exposed through the Pick control service."""
from __future__ import annotations
from ._deps import *  # noqa: F401,F403

class GazeActions:
    def _gaze_u_error_via_seg(
        self,
        u_err: float,
        *,
        center_tol: float,
        step_scale: float,
    ) -> tuple[float, float]:
        """Horizontal UV → s1/s2 when gaze roll is disabled (GO2 mount)."""
        g = self._gaze_cfg
        if bool(g.enable_roll) or abs(float(u_err)) <= float(center_tol):
            return 0.0, 0.0
        pk = self._gaze_center_pick_config()
        cap = float(g.center_seg_max) * float(max(step_scale, 0.05))
        u_gain = float(pk.center_u_gain) * float(max(g.uv_gain, 0.05)) * float(max(step_scale, 0.05))
        s2_u = float(
            np.clip(
                u_gain * float(u_err) * float(g.center_u_seg_s2_scale),
                -cap,
                cap,
            )
        )
        s1_u = float(
            np.clip(
                -u_gain * float(u_err) * float(g.center_u_seg_s1_scale),
                -cap,
                cap,
            )
        )
        return s1_u, s2_u

    def apply_gaze_uv_correction(
        self,
        obs: VisualObservation,
        *,
        extra_du: Optional[np.ndarray] = None,
        dt_s: Optional[float] = None,
    ) -> tuple[str, ControlU, ControlU, float, float]:
        """Apply one gaze UV step: P centering + optional D damping on seg axes."""
        tu = float(self.state.visual_target_uv_u)
        tv = float(self.state.visual_target_uv_v)
        u_err = float(obs.center_uv[0]) - tu
        v_err = float(obs.center_uv[1]) - tv
        current_u = self._gaze_control_current_u()
        g = self._gaze_cfg
        period = float(dt_s) if dt_s is not None else (1.0 / max(1.0, float(g.hz)))
        next_u, mode, _, _ = self._apply_pick_center_step(
            obs,
            current_u,
            cfg=self._gaze_center_pick_config(),
            coupled_axes=True,
            fallback_gains=True,
            step_scale=float(g.step_scale),
        )
        if not bool(g.enable_roll):
            next_u = ControlU(
                u_linear=float(current_u.u_linear),
                u_roll=float(current_u.u_roll),
                u_s1=float(next_u.u_s1),
                u_s2=float(next_u.u_s2),
            )
            if mode != "none" and next_u == current_u:
                mode = "none"
        pk_gaze = self._gaze_center_pick_config()
        s1_u, s2_u = self._gaze_u_error_via_seg(
            u_err,
            center_tol=float(pk_gaze.center_tol),
            step_scale=float(g.step_scale),
        )
        if abs(s1_u) > 1e-9 or abs(s2_u) > 1e-9:
            next_u = self._clamp_display_u(
                ControlU(
                    u_linear=float(current_u.u_linear),
                    u_roll=float(next_u.u_roll),
                    u_s1=float(next_u.u_s1 + s1_u),
                    u_s2=float(next_u.u_s2 + s2_u),
                )
            )
            if mode == "none" and next_u != current_u:
                mode = "gain_u_seg"
        s1_d, s2_d = self._gaze_derivative_seg_du(u_err, v_err, dt_s=period)
        if abs(s1_d) > 1e-9 or abs(s2_d) > 1e-9:
            next_u = self._clamp_display_u(
                ControlU(
                    u_linear=float(current_u.u_linear),
                    u_roll=float(next_u.u_roll),
                    u_s1=float(next_u.u_s1 + s1_d),
                    u_s2=float(next_u.u_s2 + s2_d),
                )
            )
            if mode == "none" and next_u != current_u:
                mode = "pd_damp"
        if extra_du is not None:
            du = np.asarray(extra_du, dtype=float).reshape(3)
            roll_du = float(du[0]) if bool(g.enable_roll) else 0.0
            next_u = self._clamp_display_u(
                ControlU(
                    u_linear=float(current_u.u_linear),
                    u_roll=float(next_u.u_roll + roll_du),
                    u_s1=float(next_u.u_s1 + float(du[1])),
                    u_s2=float(next_u.u_s2 + float(du[2])),
                )
            )
            if not bool(g.enable_roll):
                next_u = ControlU(
                    u_linear=float(current_u.u_linear),
                    u_roll=float(current_u.u_roll),
                    u_s1=float(next_u.u_s1),
                    u_s2=float(next_u.u_s2),
                )
        seg_cap = float(g.max_seg_du_per_tick)
        if seg_cap > 0.0:
            ds1 = float(np.clip(float(next_u.u_s1 - current_u.u_s1), -seg_cap, seg_cap))
            ds2 = float(np.clip(float(next_u.u_s2 - current_u.u_s2), -seg_cap, seg_cap))
            next_u = ControlU(
                u_linear=float(current_u.u_linear),
                u_roll=float(next_u.u_roll),
                u_s1=float(current_u.u_s1 + ds1),
                u_s2=float(current_u.u_s2 + ds2),
            )
            if mode != "none" and next_u == current_u:
                mode = "none"
        settle_s = float(g.cmd_settle_s)
        err_mag = max(abs(float(u_err)), abs(float(v_err)))
        if err_mag <= float(g.fine_err_max) and float(g.fine_settle_scale) > 0.0:
            settle_s *= float(g.fine_settle_scale)
        if (
            settle_s > 0.0
            and mode != "none"
            and next_u != current_u
            and self._gaze_last_sent_du_mag > 0.15
            and (time.time() - float(self._gaze_last_cmd_wall_s)) < settle_s
        ):
            return "settling", current_u, current_u, u_err, v_err
        if mode != "none" and next_u != current_u:
            partial: dict[str, float] = {
                "s1": float(next_u.u_s1),
                "s2": float(next_u.u_s2),
            }
            if bool(g.enable_roll):
                partial["roll"] = float(next_u.u_roll)
            self.apply_partial_control_u(partial, source="gaze")
            self._set_gaze_command_ref(next_u)
            self._gaze_last_cmd_wall_s = float(time.time())
            self._gaze_last_sent_du_mag = abs(float(next_u.u_s1 - current_u.u_s1)) + abs(
                float(next_u.u_s2 - current_u.u_s2)
            )
        return mode, current_u, next_u, u_err, v_err

    def apply_gaze_preview_correction(
        self,
        obs: VisualObservation,
        *,
        du: np.ndarray,
        dt_s: Optional[float] = None,
    ) -> tuple[str, ControlU, ControlU, float, float]:
        """Apply one preview MPC-lite step (Jacobian solve); linear axis fixed."""
        tu = float(self.state.visual_target_uv_u)
        tv = float(self.state.visual_target_uv_v)
        u_err = float(obs.center_uv[0]) - tu
        v_err = float(obs.center_uv[1]) - tv
        current_u = self._gaze_control_current_u()
        g = self._gaze_cfg
        du_v = np.asarray(du, dtype=float).reshape(3)
        roll_du = float(du_v[0]) if bool(g.enable_roll) else 0.0
        next_u = self._clamp_display_u(
            ControlU(
                u_linear=float(current_u.u_linear),
                u_roll=float(current_u.u_roll + roll_du),
                u_s1=float(current_u.u_s1 + float(du_v[1])),
                u_s2=float(current_u.u_s2 + float(du_v[2])),
            )
        )
        if not bool(g.enable_roll):
            next_u = ControlU(
                u_linear=float(current_u.u_linear),
                u_roll=float(current_u.u_roll),
                u_s1=float(next_u.u_s1),
                u_s2=float(next_u.u_s2),
            )
        seg_cap = float(g.max_seg_du_per_tick)
        if seg_cap > 0.0:
            ds1 = float(np.clip(float(next_u.u_s1 - current_u.u_s1), -seg_cap, seg_cap))
            ds2 = float(np.clip(float(next_u.u_s2 - current_u.u_s2), -seg_cap, seg_cap))
            next_u = ControlU(
                u_linear=float(current_u.u_linear),
                u_roll=float(next_u.u_roll),
                u_s1=float(current_u.u_s1 + ds1),
                u_s2=float(current_u.u_s2 + ds2),
            )
        mode = "preview_mpc" if next_u != current_u else "none"
        settle_s = float(g.cmd_settle_s)
        err_mag = max(abs(float(u_err)), abs(float(v_err)))
        if err_mag <= float(g.fine_err_max) and float(g.fine_settle_scale) > 0.0:
            settle_s *= float(g.fine_settle_scale)
        if (
            settle_s > 0.0
            and mode != "none"
            and next_u != current_u
            and self._gaze_last_sent_du_mag > 0.15
            and (time.time() - float(self._gaze_last_cmd_wall_s)) < settle_s
        ):
            return "settling", current_u, current_u, u_err, v_err
        if mode != "none" and next_u != current_u:
            partial: dict[str, float] = {
                "s1": float(next_u.u_s1),
                "s2": float(next_u.u_s2),
            }
            if bool(g.enable_roll):
                partial["roll"] = float(next_u.u_roll)
            self.apply_partial_control_u(partial, source="gaze")
            self._set_gaze_command_ref(next_u)
            self._gaze_last_cmd_wall_s = float(time.time())
            self._gaze_last_sent_du_mag = abs(float(next_u.u_s1 - current_u.u_s1)) + abs(
                float(next_u.u_s2 - current_u.u_s2)
            )
        return mode, current_u, next_u, u_err, v_err

    def close(self) -> None:
        self.stop_gaze_stabilizer()
        self.stop_object_pick()
        self.stop_perception_capture()
        if self.client is not None:
            self.client.close()

    def start_gaze_stabilizer_standing(self, *, run_id: str = "") -> None:
        if self._delegate_gaze_to_host():
            if hasattr(self.client, "send_gaze_start_standing"):
                self.client.send_gaze_start_standing(run_id=run_id)
                self.state.set_gaze_status(running=True, mode="standing/on-device", msg="start requested")
                print("[gaze] on-device standing start requested")
            else:
                self.state.set_gaze_status(running=False, mode="idle", msg="remote host lacks gaze_start_standing")
            return
        if self._visual_busy() and not self._gaze_busy():
            self.state.set_gaze_status(running=False, mode="idle", msg="rejected: visual pipeline busy")
            print("[gaze] rejected: visual pipeline busy")
            return
        try:
            self._gaze_service.start_standing_uv_only(run_id=run_id)
            self.state.set_gaze_status(running=True, mode="standing", msg="started")
        except Exception as exc:
            self.state.set_gaze_status(running=False, mode="idle", msg=f"start failed: {exc}")
            print(f"[gaze] start standing failed: {exc}")

    def start_gaze_stabilizer_walking(self, *, run_id: str = "", gaze_mode: str | None = None) -> None:
        from elesim_pilot.gaze.stabilizer import resolve_walking_gaze_mode

        mode = resolve_walking_gaze_mode(self._gaze_cfg, gaze_mode)
        if self._delegate_gaze_to_host():
            if hasattr(self.client, "send_gaze_start_walking"):
                self.client.send_gaze_start_walking(run_id=run_id, gaze_mode=mode)
                self.state.set_gaze_status(running=True, mode=f"walking/{mode}/on-device", msg="start requested")
                print(f"[gaze] on-device walking start requested | mode={mode}")
            else:
                self.state.set_gaze_status(running=False, mode="idle", msg="remote host lacks gaze_start_walking")
            return
        if self._visual_busy() and not self._gaze_busy():
            self.state.set_gaze_status(running=False, mode="idle", msg="rejected: visual pipeline busy")
            print("[gaze] rejected: visual pipeline busy")
            return
        try:
            self._gaze_service.start_walking_gaze(run_id=run_id, gaze_mode=mode)
            self.state.set_gaze_status(running=True, mode=f"walking/{mode}", msg="started")
        except Exception as exc:
            self.state.set_gaze_status(running=False, mode="idle", msg=f"start failed: {exc}")
            print(f"[gaze] start walking failed: {exc}")

    def stop_gaze_stabilizer(self) -> None:
        if self._delegate_gaze_to_host():
            if hasattr(self.client, "send_gaze_stop"):
                self.client.send_gaze_stop()
                self.state.set_gaze_status(running=False, mode="idle", msg="on-device stop requested")
                print("[gaze] on-device stop requested")
                return
        self._gaze_service.stop()

    def start_demo4_stop_and_grasp(self) -> None:
        if self._pick_busy() or self._gaze_busy():
            print("[demo4] rejected: pipeline busy")
            return
        self._gaze_service.start_stop_and_grasp_demo()

