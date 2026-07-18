"""Aim, equal-sag, and visual-centering workflow methods."""
from __future__ import annotations
from ._deps import *  # noqa: F401,F403
from elesim_simulator.observability.tracing import traced_thread_target

class AimActions:
    def _pick_reach_model(
        self,
        sag_model: Optional[dict[str, Any]] = None,
        host_state: Optional[HostState] = None,
    ):
        from elesim_simulator.robot.arm.iklib.kinematics import _ReachModel

        self.refresh_ik_context()
        limit = self._ik_context.get("limit")
        if limit is None:
            raise RuntimeError("ik context missing joint limit")
        ctx = self._ik_context_for_host(
            host_state if host_state is not None else self.current_host_state(),
            sag_model=sag_model,
        )
        return _ReachModel(context=ctx, limit=limit)

    def _pick_hold_align_display_u(
        self,
        obs: VisualObservation,
        *,
        center_tol: float,
    ) -> bool:
        """Re-apply roll/seg so gripper stays on target_uv after a Cartesian step."""
        current_u = self.current_control_u()
        next_u, mode, _, _ = self._apply_pick_center_step(obs, current_u)
        if next_u == current_u or mode == "none":
            return False
        self._send_display_control_u_and_wait(next_u, timeout_s=1.0, source="slider")
        return True

    def _pick_ee_axis_world(
        self,
        model: Any,
        q: np.ndarray,
        *,
        axis_local: tuple[float, float, float] = (1.0, 0.0, 0.0),
    ) -> np.ndarray:
        """Unit vector in world frame for a body-fixed axis (default EE local +X)."""
        from elesim_simulator.robot.arm.iklib.kinematics import _forward_link_tf

        context = model.context
        q4 = model.clamp_q(q)
        link_tf = _forward_link_tf(context, q4)
        term = str(context["terminal_link_name"])
        if term not in link_tf:
            raise RuntimeError(f"terminal link missing from FK: {term}")
        _p_link, R_link = link_tf[term]
        approach_rot_tip = np.asarray(
            context.get("approach_rot_tip", np.eye(3)), dtype=float
        ).reshape(3, 3)
        local = np.asarray(axis_local, dtype=float).reshape(3)
        local_norm = float(np.linalg.norm(local))
        if local_norm <= 1e-9:
            local = np.array([1.0, 0.0, 0.0], dtype=float)
        else:
            local = local / local_norm
        direction = R_link @ approach_rot_tip @ local
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-9:
            return np.asarray(model.grasp_direction(q4), dtype=float).reshape(3)
        return direction / norm

    def _pick_extend_cartesian(
        self,
        distance_m: float,
        host_state: Optional[HostState] = None,
    ) -> float:
        """Advance grasp point ``distance_m`` along EE local -Z via ``elesim_simulator.robot.arm.ik.solve_then_align``."""
        delta = float(max(0.0, distance_m))
        if delta <= 1e-6:
            return 0.0
        try:
            sag_model = self._pick_final_sag_model()
            model = self._pick_reach_model(sag_model=sag_model, host_state=host_state)
        except Exception as exc:
            print(f"[Pick] extend | IK model unavailable: {exc}")
            return 0.0

        q0 = self._q_array_from_state(host_state)
        tip0 = np.asarray(model.grasp_position(q0), dtype=float).reshape(3)
        axis_w = self._pick_ee_axis_world(model, q0, axis_local=(0.0, 0.0, -1.0))
        target = tip0 + axis_w * delta
        target_mode = "cartesian"
        corrected = self._pick_corrected_object_world_xyz
        estimate = self._pick_equal_sag_estimate
        if corrected is not None and estimate is not None and bool(estimate.accepted):
            corrected_target = np.asarray(corrected, dtype=float).reshape(3)
            to_corrected = corrected_target - tip0
            axial_m = float(np.dot(to_corrected, axis_w))
            lateral_m = float(np.linalg.norm(to_corrected - axis_w * axial_m))
            max_axial_m = max(0.18, float(delta) * 3.0)
            max_lateral_m = max(0.045, float(delta) * 0.8)
            if 0.002 <= axial_m <= max_axial_m and lateral_m <= max_lateral_m:
                target = corrected_target
                target_mode = "equal_sag_corrected_object"
            else:
                print(
                    "[Pick] equal_sag target fallback | axial=%.1fmm lateral=%.1fmm "
                    "limits=(%.1f, %.1f)mm"
                    % (
                        axial_m * 1000.0,
                        lateral_m * 1000.0,
                        max_axial_m * 1000.0,
                        max_lateral_m * 1000.0,
                    )
                )
        dir_hold = np.asarray(model.grasp_direction(q0), dtype=float).reshape(3)

        self.refresh_ik_context()
        ctx = self._ik_context_for_host(host_state, sag_model=sag_model)
        required = (
            "limit",
            "fk_joint_chain",
            "terminal_link_name",
            "old_tip_local_offset",
            "grasp_offset_node_local",
        )
        if any(k not in ctx for k in required):
            print("[Pick] extend | missing ik_context fields")
            return 0.0

        self.state.set_ik_status(
            running=True,
            converged=False,
            failed=False,
            err_m=float("inf"),
            msg="pick extend IK",
        )
        result = ik_pipeline.solve_then_align(
            target_world=target,
            target_dir_world=dir_hold,
            context=ctx,
            position_tol_m=max(float(self._ik_cfg.tol), 1e-4),
            max_iters=max(int(self._ik_cfg.max_iters), 1),
            current_seed=q0,
            **self._ik_align_kwargs(force_full=True),
        )
        if not result.success or result.q is None:
            self.state.set_ik_status(
                running=False,
                converged=False,
                failed=True,
                err_m=float(result.position_error_m),
                msg=str(result.reason),
            )
            print(
                "[Pick] extend | IK failed | reason=%s err=%.4fm"
                % (str(result.reason), float(result.position_error_m))
            )
            return 0.0

        q1 = np.asarray(result.q, dtype=float).reshape(4)
        tip1 = np.asarray(model.grasp_position(q1), dtype=float).reshape(3)
        travel = float(np.dot(tip1 - tip0, axis_w))
        if travel < 1e-6:
            self.state.set_ik_status(
                running=False,
                converged=False,
                failed=True,
                err_m=float(result.position_error_m),
                msg="no motion along local -Z",
            )
            print("[Pick] extend | no motion along local -Z")
            return 0.0

        align_msg = "pick extend | %s %.0fmm" % (str(target_mode), delta * 1000.0)
        if result.align_attempted:
            align_msg = "%s | dir %.1f -> %.1f deg" % (
                align_msg,
                float(np.degrees(result.initial_direction_angle_rad)),
                float(np.degrees(result.direction_angle_rad)),
            )
        self._apply_ik_solution_to_host(
            q1,
            ik_target=target,
            ik_target_dir=dir_hold,
            err_m=float(result.position_error_m),
            status_msg=align_msg,
            timeout_s=3.0,
        )
        self._pick_extend_progress_m = float(
            self._pick_extend_progress_m + max(0.0, travel)
        )
        print(
            "[Pick] extend | solve_then_align | mode=%s dist=%.0fmm travel=%.0fmm prog=%.0f/%.0fmm "
            "| target=(%.3f, %.3f, %.3f)"
            % (
                str(target_mode),
                delta * 1000.0,
                travel * 1000.0,
                float(self._pick_extend_progress_m) * 1000.0,
                float(self._pick_config_effective().approach_extend_m) * 1000.0,
                float(target[0]),
                float(target[1]),
                float(target[2]),
            )
        )
        return max(0.0, travel)

    def _wait_for_track_lock(self, *, timeout_s: float, require_frames: int) -> bool:
        deadline = time.time() + max(float(timeout_s), 0.1)
        next_search_wall = time.time() + 0.6
        while time.time() < deadline:
            if self._pick_stop_event.is_set():
                return False
            if self._track_locked(require_frames=int(require_frames)):
                self._capture_pick_reacquire_offset()
                self._reset_pick_search_state()
                return True
            if time.time() >= next_search_wall:
                moved = self._pick_apply_lost_follow_step(reason="acquire")
                if not moved:
                    moved = self._pick_apply_fov_search_step(reason="acquire")
                next_search_wall = time.time() + 0.6
                if not moved:
                    return False
            time.sleep(0.05)
        return False

    def _pick_center_lost(
        self,
        obs: VisualObservation,
        *,
        center_tol: float,
        ratio: Optional[float] = None,
    ) -> bool:
        u_d, v_d, _, _ = self._visual_uv_errors(obs)
        r = float(self._pick_center_reenter_ratio if ratio is None else ratio)
        tol = float(center_tol) * r
        return abs(u_d) > tol or abs(v_d) > tol

    def _apply_pick_center_step(
        self,
        obs: VisualObservation,
        current_u: ControlU,
        *,
        cfg: Optional[PickConfig] = None,
        fallback_gains: bool = False,
        coupled_axes: bool = False,
        step_scale: Optional[float] = None,
    ) -> tuple[ControlU, str, float, float]:
        cfg = self._pick_config_effective() if cfg is None else cfg
        center_tol = float(cfg.center_tol)
        tu, tv = float(cfg.target_uv_u), float(cfg.target_uv_v)
        u = float(obs.center_uv[0])
        v = float(obs.center_uv[1])
        u_delta = u - tu
        v_delta = v - tv
        u_in_tol = abs(u_delta) <= center_tol
        v_in_tol = abs(v_delta) <= center_tol
        v_only = False
        if coupled_axes:
            both_ok = u_in_tol and v_in_tol
            if both_ok:
                u_over = v_over = False
            elif u_in_tol and not v_in_tol:
                # u done — freeze roll; finish v with seg only (roll motion was fighting v).
                u_over = False
                v_over = True
                v_only = True
            elif v_in_tol and not u_in_tol:
                u_over = True
                v_over = False
            else:
                u_over = abs(u_delta) > 1e-9
                v_over = abs(v_delta) > 1e-9
        else:
            u_over = abs(u_delta) > center_tol
            v_over = abs(v_delta) > center_tol
        if coupled_axes:
            step_scale = float(
                max(
                    min(
                        float(step_scale if step_scale is not None else self._pick_aim_step_scale),
                        1.0,
                    ),
                    0.05,
                )
            )
        else:
            step_scale = 1.0
        if not u_over and not v_over:
            return current_u, "none", 0.0, 0.0

        err_mag = max(abs(float(u_delta)), abs(float(v_delta)))
        if coupled_axes:
            taper_ref = float(max(self._pick_aim_taper_ref_uv, center_tol, 1e-6))
            taper_min = float(np.clip(float(self._pick_aim_taper_min), 0.05, 1.0))
            taper = float(np.clip(err_mag / taper_ref, taper_min, 1.0))
            step_scale *= taper
        seg_cap = float(cfg.center_seg_max) * step_scale
        roll_cap = float(cfg.center_roll_max) * step_scale
        if coupled_axes and not v_only:
            self._update_pick_uv_jacobian(current_u=current_u, obs=obs)
        use_gain_fallback = bool(v_only) or bool(fallback_gains) or (
            err_mag > float(self._pick_aim_gain_fallback_uv)
        )
        if use_gain_fallback:
            roll_du = 0.0
            if u_over:
                roll_du += float(
                    np.clip(
                        float(cfg.center_u_gain) * float(u_delta) * step_scale,
                        -roll_cap,
                        roll_cap,
                    )
                )
            if coupled_axes and v_over and not v_only:
                roll_du += float(
                    np.clip(
                        float(cfg.center_v_gain) * float(v_delta) * step_scale * 0.5,
                        -roll_cap,
                        roll_cap,
                    )
                )
                roll_du = float(np.clip(roll_du, -roll_cap, roll_cap))
            v_gain = float(cfg.center_v_gain) * (
                float(self._pick_aim_v_only_gain_scale) if v_only else step_scale
            )
            s1_du = (
                self._center_seg_du(
                    target_v=tv,
                    obs_v=v,
                    cap=seg_cap,
                    gain=v_gain,
                )
                if v_over
                else 0.0
            )
            if v_only and v_over:
                min_step = float(self._pick_aim_v_min_seg_step) * float(step_scale)
                if abs(float(s1_du)) < min_step and abs(float(v_delta)) > 1e-9:
                    s1_du = float(
                        np.copysign(
                            min_step,
                            float(s1_du) if abs(float(s1_du)) > 1e-9 else -float(v_delta),
                        )
                    )
            s2_gain_scale = 1.0 if (coupled_axes or v_only) else 0.5
            s2_du = (
                float(
                    np.clip(
                        -float(cfg.center_v_gain) * float(v_delta) * step_scale * s2_gain_scale,
                        -seg_cap,
                        seg_cap,
                    )
                )
                if v_over and not v_only
                else 0.0
            )
            mode = "gain_v_only" if v_only else "gain_fallback"
        else:
            if not coupled_axes:
                self._update_pick_uv_jacobian(current_u=current_u, obs=obs)
            uv_error = np.array(
                [
                    float(u_delta) if bool(u_over) else 0.0,
                    float(v_delta) if bool(v_over) else 0.0,
                ],
                dtype=float,
            )
            du3 = solve_uv_control_delta(
                uv_error=uv_error,
                jacobian=self._pick_uv_jacobian,
                damping=0.03,
                gain=1.0,
                max_abs_delta=(roll_cap, seg_cap, seg_cap),
            )
            roll_du = float(du3[0])
            s1_du = float(du3[1])
            s2_du = float(du3[2])
            if u_over and v_over:
                mode = "uv_jacobian"
            elif u_over:
                mode = "uv_jacobian_u"
            else:
                mode = "uv_jacobian_v"

        next_u = self._clamp_display_u(
            ControlU(
                u_linear=float(current_u.u_linear),
                u_roll=float(current_u.u_roll + roll_du),
                u_s1=float(current_u.u_s1 + s1_du),
                u_s2=float(current_u.u_s2 + s2_du),
            )
        )
        if next_u == current_u:
            return current_u, "none", roll_du, max(abs(s1_du), abs(s2_du))
        return next_u, mode, roll_du, max(abs(s1_du), abs(s2_du))

    def _apply_pick_approach_step(
        self, obs: VisualObservation, current_u: ControlU
    ) -> tuple[ControlU, str, float, float, float]:
        cfg = self._pick_config_effective()
        # Hold gripper–object UV while advancing (do not only push dlinear).
        aligned_u, mode, roll_du, seg_du = self._apply_pick_center_step(obs, current_u)
        conv = evaluate_pick_convergence(obs, cfg=cfg)
        scale_err = float(cfg.target_scale) - float(obs.scale)
        linear_du = 0.0
        if scale_err > float(cfg.scale_tol):
            forward_gain = 1.0 if conv.center_ok else 0.35
            if bool(self._pick_scale_stuck_burst) or bool(self._pick_approach_scale_plateau):
                forward_gain = 1.0
            elif float(obs.scale) >= float(cfg.approach_min_scale):
                forward_gain = max(forward_gain, 0.9)
            linear_cap = float(cfg.linear_step_u) * float(self._pick_approach_linear_step_scale)
            if bool(self._pick_approach_scale_plateau):
                linear_cap *= 1.5
            # Display u_linear→0 is forward (see protocol linear mapping + command_direction).
            linear_du = -float(
                forward_gain
                * np.clip(
                    float(cfg.linear_gain) * scale_err,
                    0.0,
                    linear_cap,
                )
            )
        next_u = self._clamp_display_u(
            ControlU(
                u_linear=float(aligned_u.u_linear + linear_du),
                u_roll=float(aligned_u.u_roll),
                u_s1=float(aligned_u.u_s1),
                u_s2=float(aligned_u.u_s2),
            )
        )
        return next_u, str(mode), float(roll_du), float(seg_du), float(linear_du)

    def stop_object_pick(self) -> None:
        self._pick_stop_event.set()
        self._pick_center_phase = "u"
        self._reset_pick_last_seen_uv()
        self._reset_pick_uv_jacobian()
        self._pick_approach_latched = False
        self._pick_extend_done = False
        self._pick_extend_latched = False
        self._pick_extend_progress_m = 0.0
        self._pick_extend_stall = 0
        self._pick_frozen_world_xyz = None
        self._pick_clamp_streak = 0
        self._pick_scale_stuck_iters = 0
        self._pick_scale_stuck_burst = False
        self._pick_center_stuck_iters = 0
        self._pick_approach_steps = 0
        self._pick_approach_plateau_iters = 0
        self._pick_approach_last_scale = None
        self._pick_approach_scale_plateau = False
        self._reset_pick_search_state()
        self._reset_pick_drift_accounting()
        self._reset_pick_equal_sag_state()
        self.state.set_pick_status(running=False, failed=False, phase=ObjectPickPhase.IDLE.value, msg="stopped")

    def stop_aim(self) -> None:
        self.stop_object_pick()

    def start_aim(self) -> None:
        if self._pick_busy() or self._visual_busy():
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="busy",
            )
            return
        if self.client is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="no host client",
            )
            return
        if self._pick_look_ready_pose_world_xyz is None or self._pick_look_object_world_xyz is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="run Look first",
            )
            return

        self._reset_pick_last_seen_uv()
        self._reset_pick_uv_jacobian()
        self._reset_grasp_guided_state()
        host_state = self.client.refresh_state()
        obs = self.current_visual_observation(host_state)
        if obs is not None:
            self._record_pick_last_seen_uv(obs)
        cfg = self._pick_config_effective()
        self._pick_stop_event.clear()
        self._pick_center_phase = "u"
        self._pick_approach_latched = False
        self._pick_extend_done = False
        self._pick_extend_latched = False
        self._pick_extend_progress_m = 0.0
        self._pick_extend_stall = 0
        self._pick_clamp_streak = 0
        self._pick_center_stuck_iters = 0
        self._pick_approach_steps = 0
        self._reset_pick_aim_progress()
        self._reset_pick_search_state()
        self._reset_pick_drift_accounting()
        self._reset_pick_equal_sag_result_state()
        look_object = tuple(float(v) for v in self._pick_look_object_world_xyz)
        self._send_look_object_anchor_markers()
        initial_ready = self._compute_pick_ready_pose(look_object)
        if initial_ready is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="aim: cannot compute initial ready pose from look object",
            )
            return
        self._pick_initial_object_world_xyz = look_object
        self._pick_initial_ready_pose_world_xyz = tuple(float(v) for v in initial_ready)
        self._pick_frozen_world_xyz = look_object
        if not str(self.state.visual_target_label).strip():
            self.state.visual_target_label = str(self._perception_cfg.target_label).strip()
        self.state.set_pick_status(
            running=True,
            failed=False,
            phase=ObjectPickPhase.ACQUIRE.value,
            msg="aim acquiring target",
        )

        if self._perception_capture is None or not self._perception_capture.is_running():
            self._maybe_start_local_perception()

        def _worker() -> None:
            try:
                pk = self._pick_config_effective()
                aim_pk = self._pick_config_for_aim()
                print(
                    "[Aim] start | max_iters=%d target_uv=(%+.3f,%+.3f) "
                    "aim_center_tol=%.3f pick_center_tol=%.3f step_scale=%.2f settle=%.2fs"
                    % (
                        int(pk.max_iters),
                        float(pk.target_uv_u),
                        float(pk.target_uv_v),
                        float(aim_pk.center_tol),
                        float(pk.center_tol),
                        float(self._pick_aim_step_scale),
                        float(self._pick_aim_settle_s),
                    )
                )
                if not self._wait_for_track_lock(
                    timeout_s=float(pk.acquire_timeout_s),
                    require_frames=int(pk.require_track_frames),
                ):
                    print("[Aim] acquire | track lock timeout")
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg="aim acquire timeout",
                    )
                    return
                print("[Aim] acquire | track locked")

                stale_count = 0
                max_iters = int(pk.max_iters)
                for it in range(max_iters):
                    step_idx = it + 1
                    if self._pick_stop_event.is_set():
                        print(f"[Aim] step {step_idx}/{max_iters} | stopped")
                        self.state.set_pick_status(
                            running=False,
                            failed=False,
                            phase=ObjectPickPhase.IDLE.value,
                            msg="stopped",
                        )
                        return

                    host_state = self.client.refresh_state() if self.client is not None else None
                    obs = self.current_visual_observation(host_state)
                    if obs is None:
                        stale_count += 1
                        if stale_count >= 2 and self._pick_apply_lost_follow_step(
                            reason="aim_observation_lost"
                        ):
                            stale_count = 0
                            time.sleep(0.05)
                            continue
                        if stale_count >= 3:
                            if self._pick_apply_fov_search_step(reason="aim_observation_lost"):
                                stale_count = 0
                                time.sleep(0.05)
                                continue
                            self.state.set_pick_status(
                                running=False,
                                failed=True,
                                phase=ObjectPickPhase.FAILED.value,
                                msg="aim observation lost | fov search exhausted",
                            )
                            return
                        time.sleep(0.05)
                        continue
                    stale_count = 0
                    self._record_pick_last_seen_uv(obs)
                    self._capture_pick_reacquire_offset()
                    self._reset_pick_search_state()

                    conv = evaluate_pick_convergence(obs, cfg=aim_pk)
                    u_d, v_d, tu, tv = self._visual_uv_errors(obs)
                    err_mag = max(abs(float(u_d)), abs(float(v_d)))
                    if self._aim_error_diverged(err_mag):
                        self._pick_aim_diverge_count += 1
                        rollback_u = self._pick_aim_last_command_u
                        prev_err = float(self._pick_aim_last_command_err or 0.0)
                        reduced = self._reduce_aim_step_scale()
                        self._reset_pick_uv_jacobian()
                        self._pick_aim_stuck_iters = 0
                        self._pick_aim_best_uv_err = None
                        self._pick_aim_last_command_u = None
                        self._pick_aim_last_command_err = None
                        print(
                            "[Aim] diverging | rollback=%s err %.3f -> %.3f "
                            "step_scale=%.2f count=%d"
                            % (
                                "yes" if rollback_u is not None else "no",
                                float(prev_err),
                                float(err_mag),
                                float(self._pick_aim_runtime_step_scale),
                                int(self._pick_aim_diverge_count),
                            )
                        )
                        if rollback_u is not None:
                            self.state.set_pick_status(
                                running=True,
                                failed=False,
                                phase=ObjectPickPhase.CENTER.value,
                                msg=(
                                    "aim damping | uv=(%+.3f,%+.3f) step_scale=%.2f"
                                    % (
                                        float(u_d),
                                        float(v_d),
                                        float(self._pick_aim_runtime_step_scale),
                                    )
                                ),
                            )
                            self._send_display_control_u_and_wait(
                                rollback_u,
                                timeout_s=float(self._pick_aim_command_timeout_s),
                                source="slider",
                            )
                            time.sleep(float(self._pick_aim_settle_s))
                            continue
                        if not reduced and self._pick_aim_diverge_count >= 3:
                            self.state.set_pick_status(
                                running=False,
                                failed=True,
                                phase=ObjectPickPhase.FAILED.value,
                                msg=(
                                    "aim diverging | delta=(%+.3f,%+.3f)"
                                    % (float(u_d), float(v_d))
                                ),
                            )
                            return
                    eps = float(self._pick_aim_progress_eps)
                    if (
                        self._pick_aim_best_uv_err is None
                        or err_mag < float(self._pick_aim_best_uv_err) - eps
                    ):
                        self._pick_aim_best_uv_err = float(err_mag)
                        self._pick_aim_stuck_iters = 0
                        self._pick_aim_runtime_step_scale = min(
                            float(self._pick_aim_step_scale),
                            float(self._pick_aim_runtime_step_scale) * 1.15,
                        )
                    else:
                        self._pick_aim_stuck_iters += 1

                    stuck_lim = max(1, int(pk.center_stuck_iters))
                    if self._pick_aim_stuck_iters >= stuck_lim:
                        recovered = False
                        if self._pick_aim_jacobian_resets < int(self._pick_aim_jacobian_reset_max):
                            self._pick_aim_jacobian_resets += 1
                            self._reset_pick_uv_jacobian()
                            self._pick_aim_stuck_iters = 0
                            self._pick_aim_best_uv_err = None
                            print(
                                "[Aim] center_stuck | reset uv jacobian (%d/%d) | delta=(%+.3f,%+.3f)"
                                % (
                                    int(self._pick_aim_jacobian_resets),
                                    int(self._pick_aim_jacobian_reset_max),
                                    float(u_d),
                                    float(v_d),
                                )
                            )
                            recovered = True
                        elif self._reduce_aim_step_scale():
                            self._reset_pick_uv_jacobian()
                            self._pick_aim_stuck_iters = 0
                            self._pick_aim_best_uv_err = None
                            print(
                                "[Aim] center_stuck | damp step_scale=%.2f | delta=(%+.3f,%+.3f)"
                                % (
                                    float(self._pick_aim_runtime_step_scale),
                                    float(u_d),
                                    float(v_d),
                                )
                            )
                            recovered = True
                        if recovered:
                            time.sleep(0.05)
                            continue
                        self.state.set_pick_status(
                            running=False,
                            failed=True,
                            phase=ObjectPickPhase.FAILED.value,
                            msg=(
                                "aim center stuck | uv not improving | delta=(%+.3f,%+.3f)"
                                % (float(u_d), float(v_d))
                            ),
                        )
                        print(
                            "[Aim] center_stuck | give up | delta=(%+.3f,%+.3f) steps=%d"
                            % (float(u_d), float(v_d), int(step_idx))
                        )
                        return

                    if conv.center_ok:
                        self._pick_try_estimate_equal_sag(host_state)
                        estimate = self._pick_equal_sag_estimate
                        if estimate is not None and bool(estimate.accepted):
                            drift_mm = 0.0
                            if self._pick_ready_pose_drift_world is not None:
                                drift_mm = float(
                                    np.linalg.norm(
                                        np.asarray(self._pick_ready_pose_drift_world, dtype=float)
                                    )
                                    * 1000.0
                                )
                            aim_msg = (
                                "aim done | drift=%.1fmm seg1=%+.2fdeg seg2=%+.2fdeg"
                                % (
                                    drift_mm,
                                    float(estimate.seg1_equal_offset_deg),
                                    float(estimate.seg2_equal_offset_deg),
                                )
                            )
                            self.state.set_pick_status(
                                running=False,
                                failed=False,
                                phase=ObjectPickPhase.DONE.value,
                                msg=aim_msg,
                            )
                        else:
                            reason = "no estimate" if estimate is None else str(estimate.reason)
                            self.state.set_pick_status(
                                running=False,
                                failed=True,
                                phase=ObjectPickPhase.FAILED.value,
                                msg=f"aim centered but equal sag rejected | {reason}",
                            )
                        return

                    current_u = self.current_control_u()
                    next_u, center_mode, roll_req, seg_req = self._apply_pick_center_step(
                        obs,
                        current_u,
                        cfg=aim_pk,
                        fallback_gains=(err_mag > float(self._pick_aim_gain_fallback_uv)),
                        coupled_axes=True,
                        step_scale=float(self._pick_aim_runtime_step_scale),
                    )
                    du_roll = float(next_u.u_roll - current_u.u_roll)
                    du_s1 = float(next_u.u_s1 - current_u.u_s1)
                    du_s2 = float(next_u.u_s2 - current_u.u_s2)
                    snap = self.perception_snapshot()
                    bbox_wh = snap.bbox_wh if snap is not None else self.state.perception_bbox_wh
                    self._log_visual_step(
                        "aim",
                        step_idx,
                        max_iters,
                        phase=ObjectPickPhase.CENTER.value,
                        uv=f"({conv.u_err:+.3f},{conv.v_err:+.3f})",
                        target=f"({tu:+.3f},{tv:+.3f})",
                        delta=f"({u_d:+.3f},{v_d:+.3f})",
                        scale=f"{conv.scale:.3f}",
                        droll=f"{du_roll:+.2f}",
                        ds1=f"{du_s1:+.2f}",
                        ds2=f"{du_s2:+.2f}",
                        req_roll=f"{roll_req:+.2f}",
                        req_seg=f"{seg_req:+.2f}",
                        mode=center_mode,
                        j_updates=int(self._pick_uv_jacobian_update_count),
                        tracker=str(self.state.perception_tracker_phase),
                        bbox=f"{int(bbox_wh[0])}x{int(bbox_wh[1])}",
                    )

                    if next_u == current_u:
                        self._pick_clamp_streak += 1
                        if self._pick_clamp_streak >= int(self._pick_clamp_stall_limit):
                            self.state.set_pick_status(
                                running=False,
                                failed=True,
                                phase=ObjectPickPhase.FAILED.value,
                                msg=(
                                    "aim stalled (no motion) | delta=(%+.3f,%+.3f)"
                                    % (float(u_d), float(v_d))
                                ),
                            )
                            return
                        time.sleep(0.05)
                        continue

                    self._pick_clamp_streak = 0
                    self._pick_center_steps_total += 1
                    self.state.set_pick_status(
                        running=True,
                        failed=False,
                        phase=ObjectPickPhase.CENTER.value,
                        msg=(
                            "aim center | uv=(%+.3f,%+.3f) target=(%+.3f,%+.3f)"
                            % (float(conv.u_err), float(conv.v_err), float(tu), float(tv))
                        ),
                    )
                    self._send_display_control_u_and_wait(
                        next_u,
                        timeout_s=float(self._pick_aim_command_timeout_s),
                        source="slider",
                    )
                    self._pick_aim_last_command_u = current_u
                    self._pick_aim_last_command_err = float(err_mag)
                    time.sleep(float(self._pick_aim_settle_s))

                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg="aim iteration limit",
                )
            finally:
                self._pick_worker = None

        self._pick_worker = threading.Thread(
            target=traced_thread_target("pick.aim", _worker),
            name="aim",
            daemon=True,
        )
        self._pick_worker.start()

    def start_equal_sag_tweak(self) -> None:
        """Deprecated alias: corrected ready + direction align is unified in start_ready_pose()."""
        corrected_ready = self._pick_corrected_ready_pose()
        if corrected_ready is None or not isinstance(self._pick_equal_sag_model, dict):
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="run Aim first; no corrected ready pose",
            )
            return
        estimate = self._pick_equal_sag_estimate
        if estimate is None or not bool(estimate.accepted):
            reason = "no accepted equal sag estimate" if estimate is None else str(estimate.reason)
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg=f"tweak rejected | {reason}",
            )
            return
        self.start_ready_pose()

    def start_pick_forward(self, *, distance_m: float = 0.05) -> None:
        if self.state.ik_running or self._visual_busy():
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="busy",
            )
            return
        if self.client is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="no host client",
            )
            return
        step_m = float(max(distance_m, 0.0))
        if step_m <= 1e-9:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="invalid pick distance",
            )
            return
        host_state = self.client.refresh_state()
        tip = None if host_state is None else host_state.actual_tip_xyz
        direction = None if host_state is None else host_state.actual_tip_dir
        if tip is None or direction is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="missing actual_tip feedback; run sim/host feedback first",
            )
            return
        tip_world = np.asarray(tip, dtype=float).reshape(3)
        dir_world = np.asarray(direction, dtype=float).reshape(3)
        dnorm = float(np.linalg.norm(dir_world))
        if dnorm <= 1e-9:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="tcp direction is zero",
            )
            return
        dir_world = dir_world / dnorm
        target_tip = tip_world + dir_world * step_m

        self.refresh_ik_context()
        if isinstance(self._pick_equal_sag_model, dict) and self._pick_equal_sag_model:
            sag_model_override = dict(self._pick_equal_sag_model)
        else:
            sag_model_override = (
                dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {}
            )
        ctx = self._ik_context_for_host(host_state, sag_model=sag_model_override)
        required = ("limit", "fk_joint_chain", "terminal_link_name", "old_tip_local_offset", "grasp_offset_node_local")
        if any(k not in ctx for k in required):
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="missing IK context",
            )
            return

        current_seed = np.array(
            [
                float(self.state.linear),
                float(self.state.roll),
                float(self.state.theta1),
                float(self.state.theta2),
            ],
            dtype=float,
        )
        # IK minimizes grasp-point error; the orange marker is the visual TCP (actual_tip).
        try:
            grasp0 = ik_kin._forward_grasp_world(ctx, current_seed)
            target_world = target_tip + (np.asarray(grasp0, dtype=float).reshape(3) - tip_world)
        except Exception:
            target_world = target_tip.copy()

        self.state.set_target(float(target_tip[0]), float(target_tip[1]), float(target_tip[2]))
        self.state.set_target_dir(float(dir_world[0]), float(dir_world[1]), float(dir_world[2]))
        self.state.set_pick_status(
            running=True,
            failed=False,
            phase=ObjectPickPhase.EXTEND.value,
            msg="pick forward solving",
        )
        self.state.set_ik_status(
            running=True,
            converged=False,
            failed=False,
            err_m=float("inf"),
            msg="pick forward solving",
        )

        def _worker() -> None:
            try:
                result = ik_pipeline.solve_then_align(
                    target_world=target_world,
                    target_dir_world=dir_world,
                    context=ctx,
                    position_tol_m=float(self._ik_cfg.tol),
                    max_iters=max(int(self._ik_cfg.max_iters), 1),
                    current_seed=current_seed,
                )
                if result.success and result.q is not None:
                    q = np.asarray(result.q, dtype=float).reshape(4)
                    self._apply_ik_solution_to_host(
                        q,
                        ik_target=target_tip,
                        ik_target_dir=dir_world,
                        err_m=float(result.position_error_m),
                        status_msg="pick +%.0fmm | %s" % (float(step_m) * 1000.0, str(result.reason)),
                        timeout_s=3.0,
                        sag_model_override=sag_model_override,
                    )
                    self.state.set_pick_status(
                        running=False,
                        failed=False,
                        phase=ObjectPickPhase.DONE.value,
                        msg="pick done | moved %.0fmm along tcp" % (float(step_m) * 1000.0),
                    )
                else:
                    self.state.set_ik_status(
                        running=False,
                        converged=False,
                        failed=True,
                        err_m=float(result.position_error_m),
                        msg=str(result.reason),
                    )
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg="pick IK failed | " + str(result.reason),
                    )
            finally:
                self._ik_worker = None

        self._ik_worker = threading.Thread(
            target=traced_thread_target("pick.forward", _worker),
            name="pick-forward",
            daemon=True,
        )
        self._ik_worker.start()

    def start_object_pick(self) -> None:
        self.start_pick_forward(distance_m=0.05)
        return
        if self._pick_busy() or self._visual_busy():
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="busy",
            )
            return
        if self.client is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="no host client",
            )
            return

        cfg = self._pick_config_effective()
        self._pick_stop_event.clear()
        self._pick_center_phase = "u"
        self._pick_approach_latched = False
        self._pick_extend_done = False
        self._pick_extend_latched = False
        self._pick_extend_progress_m = 0.0
        self._pick_extend_stall = 0
        self._pick_clamp_streak = 0
        self._pick_scale_stuck_iters = 0
        self._pick_scale_stuck_burst = False
        self._pick_center_stuck_iters = 0
        self._pick_approach_steps = 0
        self._pick_approach_plateau_iters = 0
        self._pick_approach_last_scale = None
        self._pick_approach_scale_plateau = False
        self._pick_extend_ready_logged = False
        self._reset_pick_search_state()
        self._reset_pick_drift_accounting()
        self._reset_pick_equal_sag_state()
        self._latch_pick_frozen_world()
        self._pick_latch_initial_ready_pose()
        if not str(self.state.visual_target_label).strip():
            self.state.visual_target_label = str(self._perception_cfg.target_label).strip()
        self.state.set_pick_status(
            running=True,
            failed=False,
            phase=ObjectPickPhase.ACQUIRE.value,
            msg="acquiring target",
        )

        if self._perception_capture is None or not self._perception_capture.is_running():
            self._maybe_start_local_perception()

        def _worker() -> None:
            try:
                pk = self._pick_config_effective()
                print(
                    "[Pick] start | max_iters=%d grid=%dx%d cell=(%d,%d) target_uv=(%+.3f,%+.3f) "
                    "target_scale=%.3f (quadrant %.0f%%) extend=%.0fmm center_tol=%.3f"
                    % (
                        int(pk.max_iters),
                        int(pk.grid_cols),
                        int(pk.grid_rows),
                        int(pk.target_grid_col),
                        int(pk.target_grid_row),
                        float(pk.target_uv_u),
                        float(pk.target_uv_v),
                        float(pk.target_scale),
                        float(pk.quadrant_fill_min) * 100.0,
                        float(pk.approach_extend_m) * 1000.0,
                        float(pk.center_tol),
                    )
                )
                if not self._wait_for_track_lock(
                    timeout_s=float(pk.acquire_timeout_s),
                    require_frames=int(pk.require_track_frames),
                ):
                    print("[Pick] acquire | track lock timeout")
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg="track acquire timeout",
                    )
                    return
                print("[Pick] acquire | track locked")

                stale_count = 0
                max_iters = int(pk.max_iters)
                for it in range(max_iters):
                    current_u = self.current_control_u()
                    step_idx = it + 1
                    if self._pick_stop_event.is_set():
                        print(f"[Pick] step {step_idx}/{max_iters} | stopped")
                        self.state.set_pick_status(
                            running=False,
                            failed=False,
                            phase=ObjectPickPhase.IDLE.value,
                            msg="stopped",
                        )
                        return

                    host_state = self.client.refresh_state() if self.client is not None else None
                    obs = self.current_visual_observation(host_state)
                    if obs is None:
                        stale_count += 1
                        print(f"[Pick] step {step_idx}/{max_iters} | stale obs ({stale_count}/3)")
                        if stale_count >= 3:
                            if self._pick_apply_fov_search_step(reason="observation_lost"):
                                stale_count = 0
                                time.sleep(0.05)
                                continue
                            self.state.set_pick_status(
                                running=False,
                                failed=True,
                                phase=ObjectPickPhase.FAILED.value,
                                msg="observation lost | fov search exhausted",
                            )
                            return
                        time.sleep(0.05)
                        continue
                    stale_count = 0
                    self._capture_pick_reacquire_offset()
                    self._reset_pick_search_state()

                    self._pick_latch_initial_ready_pose()
                    conv = evaluate_pick_convergence(obs, cfg=pk)
                    u_d, v_d, _, _ = self._visual_uv_errors(obs)
                    if conv.center_ok:
                        self._pick_try_estimate_equal_sag(host_state)
                        self._pick_approach_latched = True
                        self._pick_center_stuck_iters = 0
                    center_tol = float(pk.center_tol)
                    use_approach = bool(
                        self._pick_approach_latched
                        and not self._pick_center_lost(
                            obs,
                            center_tol=center_tol,
                            ratio=float(self._pick_approach_lost_ratio),
                        )
                    )
                    if not use_approach and self._pick_center_lost(obs, center_tol=center_tol):
                        self._pick_approach_latched = False
                    # Recovered alignment but still too small in image → approach again.
                    if (
                        not use_approach
                        and conv.center_ok
                        and not conv.scale_ok
                    ):
                        self._pick_approach_latched = True
                        use_approach = True

                    if (
                        not use_approach
                        and conv.scale_ok
                        and not conv.center_ok
                        and max(abs(float(u_d)), abs(float(v_d)))
                        <= float(pk.center_stuck_max_uv)
                    ):
                        self._pick_center_stuck_iters += 1
                        center_stuck_lim = max(1, int(pk.center_stuck_iters))
                        if self._pick_center_stuck_iters >= center_stuck_lim:
                            self._pick_approach_latched = True
                            use_approach = True
                            print(
                                "[Pick] center_stuck | forcing approach | scale=%.3f "
                                "delta=(%+.3f,%+.3f) tol=%.3f"
                                % (
                                    float(conv.scale),
                                    float(u_d),
                                    float(v_d),
                                    float(pk.center_tol),
                                )
                            )
                    elif not conv.scale_ok or conv.center_ok:
                        self._pick_center_stuck_iters = 0

                    scale_stuck_thresh = float(pk.target_scale) * float(pk.scale_stuck_ratio)
                    stuck_lim = max(1, int(pk.scale_stuck_iters))
                    if conv.scale_ok:
                        self._pick_scale_stuck_iters = 0
                        self._pick_scale_stuck_burst = False
                    elif float(conv.scale) < scale_stuck_thresh:
                        self._pick_scale_stuck_iters += 1
                        if (
                            not self._pick_scale_stuck_burst
                            and self._pick_scale_stuck_iters >= stuck_lim
                        ):
                            self._pick_approach_latched = True
                            use_approach = True
                            self._pick_scale_stuck_burst = True
                            print(
                                "[Pick] scale_stuck | forcing approach | scale=%.3f "
                                "target=%.3f tracker=%s"
                                % (
                                    float(conv.scale),
                                    float(pk.target_scale),
                                    str(self.state.perception_tracker_phase),
                                )
                            )
                        elif (
                            self._pick_scale_stuck_burst
                            and self._pick_scale_stuck_iters >= stuck_lim * 2
                        ):
                            snap = self.perception_snapshot()
                            bbox_wh = snap.bbox_wh if snap is not None else (0, 0)
                            self.state.set_pick_status(
                                running=False,
                                failed=True,
                                phase=ObjectPickPhase.FAILED.value,
                                msg=(
                                    "scale stuck at %.3f (check tracker bbox %dx%d) | "
                                    "target_scale=%.3f"
                                )
                                % (
                                    float(conv.scale),
                                    int(bbox_wh[0]),
                                    int(bbox_wh[1]),
                                    float(pk.target_scale),
                                ),
                            )
                            return

                    if not bool(self._pick_extend_latched):
                        if use_approach:
                            plateau_eps = float(pk.approach_scale_plateau_eps)
                            if self._pick_approach_last_scale is not None and abs(
                                float(conv.scale) - float(self._pick_approach_last_scale)
                            ) < plateau_eps:
                                self._pick_approach_plateau_iters += 1
                            else:
                                self._pick_approach_plateau_iters = 0
                            self._pick_approach_last_scale = float(conv.scale)
                            plateau_need = max(1, int(pk.approach_scale_plateau_iters))
                            self._pick_approach_scale_plateau = (
                                int(self._pick_approach_plateau_iters) >= plateau_need
                            )
                        else:
                            self._pick_approach_plateau_iters = 0
                            self._pick_approach_scale_plateau = False

                        extend_ready, extend_reason = pick_ready_for_extend(
                            obs,
                            cfg=pk,
                            approach_steps=int(self._pick_approach_steps),
                            scale_plateau=bool(self._pick_approach_scale_plateau),
                        )
                        if extend_ready:
                            self._pick_extend_latched = True
                            if not bool(self._pick_extend_ready_logged):
                                du, dv = pick_uv_deltas(obs, cfg=pk)
                                print(
                                    "[Pick] extend ready (%s) | scale=%.3f steps=%d "
                                    "plateau=%s | delta_u=%+.3f delta_v=%+.3f"
                                    % (
                                        str(extend_reason),
                                        float(conv.scale),
                                        int(self._pick_approach_steps),
                                        str(self._pick_approach_scale_plateau),
                                        float(du),
                                        float(dv),
                                    )
                                )
                                self._pick_extend_ready_logged = True

                    ext_target_m = float(pk.approach_extend_m)
                    if bool(self._pick_extend_latched):
                        if float(self._pick_extend_progress_m) < ext_target_m - 1e-3:
                            remain_m = ext_target_m - float(self._pick_extend_progress_m)
                            self.state.set_pick_status(
                                running=True,
                                failed=False,
                                phase=ObjectPickPhase.EXTEND.value,
                                msg="extend %.0f/%.0f mm | uv=(%.3f, %.3f) scale=%.3f"
                                % (
                                    float(self._pick_extend_progress_m) * 1000.0,
                                    ext_target_m * 1000.0,
                                    conv.u_err,
                                    conv.v_err,
                                    conv.scale,
                                ),
                            )
                            traveled_m = self._pick_extend_cartesian(
                                remain_m, host_state
                            )
                            print(
                                "[Pick] step %d/%d | extend | cart=%.1fmm prog=%.0f/%.0fmm "
                                "| uv=(%.3f, %.3f) scale=%.3f"
                                % (
                                    step_idx,
                                    max_iters,
                                    traveled_m * 1000.0,
                                    float(self._pick_extend_progress_m) * 1000.0,
                                    ext_target_m * 1000.0,
                                    conv.u_err,
                                    conv.v_err,
                                    conv.scale,
                                )
                            )
                            if traveled_m < remain_m * 0.25:
                                self._pick_extend_stall += 1
                                if self._pick_extend_stall >= 2:
                                    self.state.set_pick_status(
                                        running=False,
                                        failed=True,
                                        phase=ObjectPickPhase.FAILED.value,
                                        msg=(
                                            "extend stalled | prog=%.0fmm target=%.0fmm"
                                        )
                                        % (
                                            float(self._pick_extend_progress_m) * 1000.0,
                                            ext_target_m * 1000.0,
                                        ),
                                    )
                                    return
                            else:
                                self._pick_extend_stall = 0
                            time.sleep(0.05)
                            continue
                        self._pick_extend_done = True
                        print(
                            "[Pick] step %d/%d | done | uv=(%.3f, %.3f) scale=%.3f"
                            % (step_idx, max_iters, conv.u_err, conv.v_err, conv.scale)
                        )
                        self.state.set_pick_status(
                            running=False,
                            failed=False,
                            phase=ObjectPickPhase.DONE.value,
                            msg="pick done | extend %.0fmm | uv=(%.3f, %.3f) scale=%.3f"
                            % (
                                float(self._pick_extend_progress_m) * 1000.0,
                                conv.u_err,
                                conv.v_err,
                                conv.scale,
                            ),
                        )
                        return

                    center_mode = ""
                    roll_req = 0.0
                    seg_req = 0.0
                    linear_req = 0.0
                    if use_approach:
                        phase = ObjectPickPhase.APPROACH
                        next_u, center_mode, roll_req, seg_req, linear_req = (
                            self._apply_pick_approach_step(obs, current_u)
                        )
                    else:
                        phase = ObjectPickPhase.CENTER
                        next_u, center_mode, roll_req, seg_req = self._apply_pick_center_step(
                            obs, current_u
                        )

                    du_linear = float(next_u.u_linear - current_u.u_linear)
                    du_roll = float(next_u.u_roll - current_u.u_roll)
                    du_seg = float(next_u.u_s1 - current_u.u_s1)
                    u_d, v_d, tu, tv = self._visual_uv_errors(obs)
                    snap = self.perception_snapshot()
                    bbox_wh = snap.bbox_wh if snap is not None else self.state.perception_bbox_wh
                    pick_fields: dict[str, object] = dict(
                        phase=phase.value,
                        uv=f"({conv.u_err:+.3f},{conv.v_err:+.3f})",
                        target=f"({tu:+.3f},{tv:+.3f})",
                        delta=f"({u_d:+.3f},{v_d:+.3f})",
                        scale=f"{conv.scale:.3f}",
                        center_ok=str(conv.center_ok),
                        scale_ok=str(conv.scale_ok),
                        dlinear=f"{du_linear:+.2f}",
                        droll=f"{du_roll:+.2f}",
                        dseg=f"{du_seg:+.2f}",
                        req_roll=f"{roll_req:+.2f}",
                        req_seg=f"{seg_req:+.2f}",
                        req_linear=f"{linear_req:+.2f}",
                        plateau=str(self._pick_approach_scale_plateau),
                        approach_n=int(self._pick_approach_steps),
                        tracker=str(self.state.perception_tracker_phase),
                        bbox=f"{int(bbox_wh[0])}x{int(bbox_wh[1])}",
                    )
                    if center_mode:
                        pick_fields["mode"] = center_mode
                    self._log_visual_step(
                        "pick",
                        step_idx,
                        max_iters,
                        **pick_fields,
                    )

                    if next_u == current_u:
                        v_align_tol = float(pk.center_tol) * float(self._pick_approach_v_hold_ratio)
                        if phase == ObjectPickPhase.CENTER and (
                            abs(u_d) <= float(pk.center_tol)
                            and abs(v_d) > v_align_tol
                        ):
                            self._pick_clamp_streak += 1
                            print(
                                "[Pick] step %d/%d | no actuator change | mode=%s "
                                "req_roll=%+.2f req_seg=%+.2f u_s1=%.1f streak=%d"
                                % (
                                    step_idx,
                                    max_iters,
                                    center_mode or "?",
                                    float(roll_req),
                                    float(seg_req),
                                    float(current_u.u_s1),
                                    int(self._pick_clamp_streak),
                                )
                            )
                            if self._pick_clamp_streak >= int(self._pick_clamp_stall_limit):
                                self.state.set_pick_status(
                                    running=False,
                                    failed=True,
                                    phase=ObjectPickPhase.FAILED.value,
                                    msg=(
                                        "v align stalled (no motion) | delta_v=%+.3f "
                                        "target_v=%+.3f req_seg=%+.1f"
                                    )
                                    % (float(v_d), float(tv), float(seg_req)),
                                )
                                return
                        else:
                            print(f"[Pick] step {step_idx}/{max_iters} | command clamped")
                        if phase == ObjectPickPhase.APPROACH and not conv.scale_ok:
                            self.state.set_pick_status(
                                running=False,
                                failed=True,
                                phase=ObjectPickPhase.FAILED.value,
                                msg="approach linear limit | u_linear=%.1f scale=%.3f target=%.3f"
                                % (float(current_u.u_linear), conv.scale, float(pk.target_scale)),
                            )
                            return
                        extend_ready_clamp, _ = pick_ready_for_extend(
                            obs,
                            cfg=pk,
                            approach_steps=int(self._pick_approach_steps),
                            scale_plateau=bool(self._pick_approach_scale_plateau),
                        )
                        if extend_ready_clamp:
                            self.state.set_pick_status(
                                running=False,
                                failed=False,
                                phase=ObjectPickPhase.DONE.value,
                                msg="pick ready | uv=(%.3f, %.3f) scale=%.3f (grasp manual)"
                                % (conv.u_err, conv.v_err, conv.scale),
                            )
                            return
                        time.sleep(0.05)
                        continue

                    self._pick_clamp_streak = 0
                    if use_approach:
                        self._pick_approach_steps += 1
                    elif phase == ObjectPickPhase.CENTER:
                        self._pick_center_steps_total += 1
                    self.state.set_pick_status(
                        running=True,
                        failed=False,
                        phase=phase.value,
                        msg="%s | uv=(%.3f, %.3f) scale=%.3f"
                        % (phase.value, conv.u_err, conv.v_err, conv.scale),
                    )
                    self._send_display_control_u_and_wait(next_u, timeout_s=1.0, source="slider")
                    current_u = next_u
                    time.sleep(0.05)

                print(f"[Pick] iteration limit ({max_iters} steps)")
                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg="iteration limit",
                )
            except Exception as exc:
                print(f"[Pick] failed: {exc}")
                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg=str(exc),
                )
            finally:
                self._pick_frozen_world_xyz = None
                self._pick_worker = None

        self._pick_worker = threading.Thread(
            target=traced_thread_target("pick.object", _worker),
            name="object-pick",
            daemon=True,
        )
        self._pick_worker.start()
