"""Guided grasp and local-image-Jacobian workflow methods."""
from __future__ import annotations
from ._deps import *  # noqa: F401,F403
from elesim_pilot.observability.tracing import traced_thread_target

def _service_type():
    from .actions import ControlService
    return ControlService

class GraspGeometryActions:
    @staticmethod
    def _compute_grasp_nominal_endpoint(
        object_world: tuple[float, float, float],
        approach_dir: tuple[float, float, float] | np.ndarray,
        *,
        standoff_m: float,
    ) -> tuple[float, float, float]:
        return compute_ready_pose_target(
            object_world,
            tuple(float(v) for v in approach_dir),
            standoff_m=float(standoff_m),
        )

    @staticmethod
    def _grasp_axial_distance(
        tip_world: tuple[float, float, float] | np.ndarray,
        nominal_world: tuple[float, float, float] | np.ndarray,
        approach_dir: tuple[float, float, float] | np.ndarray,
    ) -> float:
        tip = np.asarray(tip_world, dtype=float).reshape(3)
        nominal = np.asarray(nominal_world, dtype=float).reshape(3)
        direction = _service_type()._unit_vec3(approach_dir)
        return float(np.dot(nominal - tip, direction))

    @staticmethod
    def _grasp_object_standoff_m(
        tip_world: tuple[float, float, float] | np.ndarray,
        object_world: tuple[float, float, float] | np.ndarray,
    ) -> float:
        tip = np.asarray(tip_world, dtype=float).reshape(3)
        obj = np.asarray(object_world, dtype=float).reshape(3)
        return float(np.linalg.norm(obj - tip))

    @staticmethod
    def _grasp_approach_remaining_m(
        tip_world: tuple[float, float, float] | np.ndarray,
        object_world: tuple[float, float, float] | np.ndarray,
        grasp_standoff_m: float,
    ) -> float:
        return _service_type()._grasp_object_standoff_m(
            tip_world,
            object_world,
        ) - float(max(grasp_standoff_m, 0.0))

    @staticmethod
    def _grasp_look_at_dir(
        tip_world: tuple[float, float, float] | np.ndarray,
        object_world: tuple[float, float, float] | np.ndarray,
    ) -> np.ndarray:
        tip = np.asarray(tip_world, dtype=float).reshape(3)
        obj = np.asarray(object_world, dtype=float).reshape(3)
        vec = obj - tip
        norm = float(np.linalg.norm(vec))
        if norm <= 1e-9:
            raise ValueError("degenerate look-at")
        return vec / norm

    @staticmethod
    def _grasp_precontact_from_tip(
        tip_world: tuple[float, float, float] | np.ndarray,
        object_world: tuple[float, float, float] | np.ndarray,
        grasp_standoff_m: float,
    ) -> tuple[float, float, float]:
        tip = np.asarray(tip_world, dtype=float).reshape(3)
        look = _service_type()._grasp_look_at_dir(tip, object_world)
        standoff = float(max(grasp_standoff_m, 0.0))
        pre = np.asarray(object_world, dtype=float).reshape(3) - look * standoff
        return (float(pre[0]), float(pre[1]), float(pre[2]))

    def _pick_grasp_trajectory_start_position(
        self,
    ) -> Optional[tuple[float, float, float]]:
        """Geometric grasp-path anchor: latched Look view pose (not Aim/centered ready)."""
        if self._pick_look_ready_pose_world_xyz is not None:
            return self._pick_look_ready_pose_world_xyz
        if self._pick_resolved_ready_pose_world_xyz is not None:
            return self._pick_resolved_ready_pose_world_xyz
        return None

    def _pick_grasp_trajectory_end_position(
        self,
        object_world: tuple[float, float, float],
        approach_dir: tuple[float, float, float] | np.ndarray,
        *,
        standoff_m: float,
    ) -> tuple[float, float, float]:
        """Grasp path terminus: pre-contact nominal at grasp_standoff (not object center)."""
        return self._compute_grasp_nominal_endpoint(
            object_world,
            approach_dir,
            standoff_m=float(standoff_m),
        )

    @staticmethod
    def _grasp_waypoint_behind_tip(
        waypoint: GraspWaypoint,
        tip_world: tuple[float, float, float],
        nominal_world: tuple[float, float, float],
        approach_dir: tuple[float, float, float] | np.ndarray,
    ) -> bool:
        """True when the waypoint lies toward Look, past the current tip on the approach axis."""
        wp_dist = _service_type()._grasp_axial_distance(
            waypoint.position_world,
            nominal_world,
            approach_dir,
        )
        tip_dist = _service_type()._grasp_axial_distance(
            tip_world,
            nominal_world,
            approach_dir,
        )
        return wp_dist > tip_dist + 1e-4


class GraspLjiSolverActions(GraspGeometryActions):
    """LJI setup, feature construction, constrained solve, and command shaping."""

    def _reset_grasp_guided_state(self) -> None:
        self._grasp_waypoint_idx = 0
        self._grasp_online_sag_model = None
        self._grasp_nominal_dir = None
        self._grasp_trajectory_nominal_pose = None
        self._grasp_executed_waypoints = []
        self._grasp_traj_start = None
        self._grasp_look_anchor = None
        self._grasp_handoff_look_dir = None
        self._grasp_object_world_filtered = None
        self._grasp_approach_dir_filtered = None
        self._grasp_uv_only_mode = False
        self._grasp_approach_mode = GraspApproachMode.LOCAL_IMG_JACOBIAN
        self._grasp_lji_estimator_3d = None
        self._grasp_lji_servo_3d = None
        self._grasp_lji_frozen_sag_model = None
        self._grasp_depth_history.clear()
        self._grasp_lji_object_lost_count = 0
        self._grasp_lji_last_reliable_object_world = None
        self._grasp_lji_last_reliable_approach_dir = None
        self._grasp_lji_last_reliable_depth = None
        self._grasp_lji_last_reliable_camera_xyz = None
        self._grasp_lji_last_good_q = None
        self._grasp_lji_pending_sample = None
        self._grasp_lji_last_dq_cmd = None
        self._grasp_lji_command_q = None
        self._grasp_lji_reacquire_anchor_dq = None
        self._grasp_lji_reacquire_steps = 0
        self._grasp_lji_reacquire_aim_tried = False
        self._grasp_lji_reacquire_prev_remain = None
        self._grasp_lji_v_err_hist = []
        self._grasp_lji_last_transition = "-"
        self._grasp_lji_sat_streak = 0
        self._grasp_lji_bad_motion_streak = 0
        self._grasp_lji_force_reacquire_reason = ""
        self._grasp_lji_remain_hist: list[float] = []

    def _grasp_lji_sag_model(self) -> dict[str, Any]:
        """Fixed equal-sag from grasp start; LJI does not run online sag updates."""
        frozen = self._grasp_lji_frozen_sag_model
        if isinstance(frozen, dict) and frozen:
            return dict(frozen)
        return self._pick_grasp_sag_model()

    def _grasp_init_lji_controller(self, pk: PickConfig) -> None:
        seed_j = default_j_lji_seed(
            center_u_gain=float(pk.center_u_gain),
            center_v_gain=float(pk.center_v_gain),
            z_bend_gain=float(pk.lij_z_bend_gain),
            command_direction=tuple(int(v) for v in self.control_mapping().command_direction),
            seg1_jacobian_scale=float(pk.lij_seg1_jacobian_scale),
            seg2_jacobian_scale=float(pk.lij_seg2_jacobian_scale),
        )
        # LJI uses aim-time equal-sag frozen at grasp start; no per-waypoint online sag.
        frozen_sag = self._pick_grasp_sag_model()
        self._grasp_lji_frozen_sag_model = (
            dict(frozen_sag) if isinstance(frozen_sag, dict) and frozen_sag else {}
        )
        self._grasp_lji_estimator_3d = ImageJacobianEstimator3D(
            window_size=int(pk.lij_window_size),
            seed_j=seed_j,
            min_measured_samples=int(pk.lij_min_samples),
            condition_max=float(pk.lij_condition_max),
            min_rank=3,
        )
        gains = LocalImageJacobianServoGains(
            damping=float(pk.lij_damping),
            gain_u=float(pk.lij_gain_u),
            gain_v=float(pk.lij_gain_v),
            gain_z=float(pk.lij_gain_z),
            max_dq_linear=float(pk.lij_max_dq_linear),
            max_dq_angle=float(pk.lij_max_dq_angle),
        )
        self._grasp_lji_servo_3d = LocalImageJacobianServo3D(
            estimator=self._grasp_lji_estimator_3d,
            gains=gains,
            min_samples=int(pk.lij_min_samples),
            condition_max=float(pk.lij_condition_max),
            min_rank=3,
            command_direction=tuple(int(v) for v in self.control_mapping().command_direction),
            measured_v_row_blend=float(pk.lij_measured_v_row_blend),
            measured_v_row_norm_max=float(pk.lij_measured_v_row_norm_max),
        )
        self._grasp_approach_mode = GraspApproachMode.LOCAL_IMG_JACOBIAN
        self._grasp_depth_history.clear()
        self._grasp_lji_object_lost_count = 0
        self._grasp_lji_pending_sample = None
        self._grasp_lji_last_dq_cmd = None
        self._grasp_lji_command_q = None
        self._grasp_lji_reacquire_anchor_dq = None
        self._grasp_lji_reacquire_steps = 0
        self._grasp_lji_reacquire_aim_tried = False
        self._grasp_lji_reacquire_prev_remain = None
        self._grasp_lji_v_err_hist = []
        self._grasp_lji_last_transition = "-"
        self._grasp_lji_sat_streak = 0
        self._grasp_lji_bad_motion_streak = 0
        self._grasp_lji_force_reacquire_reason = ""

    @staticmethod
    def _grasp_lji_q_delta4(raw: Sequence[float]) -> tuple[float, float, float, float]:
        vals = [float(v) for v in raw]
        while len(vals) < 4:
            vals.append(0.0)
        return (vals[0], vals[1], vals[2], vals[3])

    def _grasp_lji_build_features_3d(
        self,
        obs: Optional[VisualObservation],
        *,
        remain_m: float,
    ) -> Optional[np.ndarray]:
        if obs is None:
            return None
        # Observation error (obs - target), same convention as solve_uv_control_delta.
        u_d, v_d, _, _ = self._visual_uv_errors(obs)
        return np.array([float(u_d), float(v_d), float(remain_m)], dtype=float)

    def _grasp_lji_camera_outlier_reason(
        self,
        raw_camera_xyz: Any,
        *,
        pk: PickConfig,
    ) -> str:
        if raw_camera_xyz is None:
            return ""
        try:
            p_cam = np.asarray(raw_camera_xyz, dtype=float).reshape(3)
        except (TypeError, ValueError):
            return "camera_invalid"
        if not bool(np.all(np.isfinite(p_cam))):
            return "camera_invalid"
        z_m = float(p_cam[2])
        if z_m <= 0.0:
            return "camera_invalid"
        max_z = float(max(getattr(pk, "lij_obs_max_camera_z_m", 0.0), 0.0))
        if max_z > 1e-6 and z_m > max_z:
            return "camera_z_outlier"
        last = self._grasp_lji_last_reliable_camera_xyz
        jump_m = float(max(getattr(pk, "lij_obs_camera_jump_m", 0.0), 0.0))
        if last is not None and jump_m > 1e-6:
            prev = np.asarray(last, dtype=float).reshape(3)
            if float(np.linalg.norm(p_cam - prev)) > jump_m:
                return "camera_jump"
        return ""

    def _grasp_lji_remain_outlier_reason(
        self,
        remain_m: float,
        *,
        pk: PickConfig,
    ) -> str:
        jump_m = float(max(getattr(pk, "lij_obs_remain_jump_m", 0.0), 0.0))
        if jump_m <= 1e-6:
            return ""
        last = self._grasp_lji_last_reliable_depth
        if last is None:
            return ""
        if abs(float(remain_m) - float(last)) > jump_m:
            return "remain_jump"
        return ""

    def _grasp_lji_observation_outlier(
        self,
        obs: Optional[VisualObservation],
        host_state: Optional[HostState],
        *,
        pk: PickConfig,
    ) -> str:
        if obs is None or host_state is None:
            return ""
        raw = getattr(host_state, "perceived_object_camera_xyz", None)
        return self._grasp_lji_camera_outlier_reason(raw, pk=pk)

    @staticmethod
    def _grasp_lji_gain_scale(
        remain_m: float,
        pk: PickConfig,
        *,
        close_tol_m: float,
    ) -> float:
        """1.0 when far; ramps down toward close_tol to damp oscillation near contact."""
        ref = float(max(pk.lij_gain_scale_ref_m, close_tol_m + 0.01))
        floor = float(np.clip(pk.lij_gain_scale_min, 0.05, 1.0))
        if float(remain_m) >= 0.9 * ref:
            return 1.0
        if float(remain_m) <= 0.2 * ref:
            return floor
        span = max(ref - float(close_tol_m), 1e-4)
        t = (float(remain_m) - float(close_tol_m)) / span
        return float(np.clip(t, floor, 1.0))

    def _grasp_lji_step_limits(
        self,
        remain_m: float,
        pk: PickConfig,
        *,
        close_tol_m: float,
    ) -> tuple[float, float, float, float, float]:
        """Per-step dq caps and gain scale; far range allows larger linear."""
        scale = self._grasp_lji_gain_scale(remain_m, pk, close_tol_m=close_tol_m)
        handoff = float(max(pk.lij_uv_handoff_m, 0.01))
        if float(remain_m) > handoff:
            max_lin = float(
                max(pk.lij_max_dq_linear, min(pk.lij_far_linear_cap_m, pk.lij_far_z_gain * remain_m))
            )
        else:
            max_lin = float(pk.lij_max_dq_linear)
        max_lin *= scale
        max_ang = float(pk.lij_max_dq_roll) * scale
        max_t1 = float(pk.lij_max_dq_theta1) * scale
        max_t2 = float(pk.lij_max_dq_angle) * scale
        return max_lin, max_ang, max_t1, max_t2, scale

    def _grasp_lji_fk_z_row(
        self,
        q: np.ndarray,
        approach_dir: np.ndarray,
        *,
        sag_model: Optional[dict[str, Any]] = None,
    ) -> np.ndarray:
        model = self._pick_reach_model(sag_model=sag_model)
        j_pos = model.position_jacobian(q)
        return z_jacobian_row_from_position_jacobian(j_pos, approach_dir)

    def _grasp_lji_blocked_axes_for_limits(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        *,
        pk: PickConfig,
    ) -> tuple[np.ndarray, tuple[str, ...]]:
        flags = self._grasp_lji_joint_limit_flags(
            q,
            margin_m=float(pk.lij_joint_limit_margin_m),
            margin_rad=float(pk.lij_joint_limit_margin_rad),
            cfg=self._mapping_cfg,
        )
        d = np.asarray(dq, dtype=float).reshape(4)
        blocked = np.zeros(4, dtype=bool)
        names: list[str] = []

        def _mark(idx: int, name: str) -> None:
            blocked[int(idx)] = True
            names.append(str(name))

        if flags["linear_max"] and float(d[0]) > 0.0:
            _mark(0, "linear_max")
        if flags["linear_min"] and float(d[0]) < 0.0:
            _mark(0, "linear_min")
        if flags["roll_max"] and float(d[1]) > 0.0:
            _mark(1, "roll_max")
        if flags["roll_min"] and float(d[1]) < 0.0:
            _mark(1, "roll_min")
        if flags["theta1_max"] and float(d[2]) > 0.0:
            _mark(2, "theta1_max")
        if flags["theta1_min"] and float(d[2]) < 0.0:
            _mark(2, "theta1_min")
        if flags["theta2_max"] and float(d[3]) > 0.0:
            _mark(3, "theta2_max")
        if flags["theta2_min"] and float(d[3]) < 0.0:
            _mark(3, "theta2_min")
        return blocked, tuple(names)

    @staticmethod
    def _grasp_lji_solve_with_axis_mask(
        j: np.ndarray,
        s_lji: np.ndarray,
        *,
        free_mask: np.ndarray,
        damping: float,
        gain_u: float,
        gain_v: float,
        gain_z: float,
        max_lin: float,
        max_ang: float,
        max_t1: float,
        max_t2: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        free = np.asarray(free_mask, dtype=bool).reshape(4)
        if int(np.count_nonzero(free)) <= 0:
            return np.zeros(4, dtype=float), np.zeros(4, dtype=float)
        jj = np.asarray(j, dtype=float).reshape(3, 4)[:, free]
        s = np.asarray(s_lji, dtype=float).reshape(3)
        j_stack = np.vstack(
            [
                float(gain_z) * jj[2:3, :],
                float(gain_u) * jj[0:1, :],
                float(gain_v) * jj[1:2, :],
            ]
        )
        s_stack = np.array([float(s[2]), float(s[0]), float(s[1])], dtype=float)
        lam = float(max(damping, 1e-9))
        jj_t = j_stack @ j_stack.T
        pinv = j_stack.T @ np.linalg.inv(jj_t + (lam * lam) * np.eye(3, dtype=float))
        dq_free_raw = (-pinv @ s_stack.reshape(3, 1)).reshape(-1)
        dq_raw = np.zeros(4, dtype=float)
        dq_raw[free] = dq_free_raw
        dq = clip_dq(
            dq_raw,
            max_dq_linear=max_lin,
            max_dq_angle=max_ang,
            max_dq_theta1=max_t1,
            max_dq_theta2=max_t2,
        )
        return dq, dq_raw

    @staticmethod
    def _grasp_lji_weighted_system(
        j: np.ndarray,
        s_lji: np.ndarray,
        *,
        gain_u: float,
        gain_v: float,
        gain_z: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        jj = np.asarray(j, dtype=float).reshape(3, 4)
        s = np.asarray(s_lji, dtype=float).reshape(3)
        gains = np.array([float(gain_z), float(gain_u), float(gain_v)], dtype=float)
        ordered_j = np.vstack(
            [
                jj[2:3, :],
                jj[0:1, :],
                jj[1:2, :],
            ]
        )
        ordered_s = np.array([float(s[2]), float(s[0]), float(s[1])], dtype=float)
        active = gains > 1e-12
        a = ordered_j[active, :]
        b = gains[active] * ordered_s[active]
        return a, b

    @staticmethod
    def _grasp_lji_box_objective(
        a: np.ndarray,
        b: np.ndarray,
        dq: np.ndarray,
        *,
        damping: float,
    ) -> float:
        x = np.asarray(dq, dtype=float).reshape(4)
        matrix = np.asarray(a, dtype=float)
        target = np.asarray(b, dtype=float).reshape(-1)
        if matrix.ndim != 2 or matrix.shape[1] != 4 or matrix.shape[0] != target.size:
            raise ValueError(f"incompatible LJI objective shapes: A={matrix.shape}, b={target.shape}")
        r = matrix @ x + target
        lam = float(max(damping, 1e-9))
        return float(np.dot(r, r) + lam * lam * np.dot(x, x))

    @staticmethod
    def _grasp_lji_solve_box_constrained(
        j: np.ndarray,
        s_lji: np.ndarray,
        *,
        lower: np.ndarray,
        upper: np.ndarray,
        damping: float,
        gain_u: float,
        gain_v: float,
        gain_z: float,
        initial: Optional[np.ndarray] = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        a, b = _service_type()._grasp_lji_weighted_system(
            j,
            s_lji,
            gain_u=gain_u,
            gain_v=gain_v,
            gain_z=gain_z,
        )
        lo = np.asarray(lower, dtype=float).reshape(4)
        hi = np.asarray(upper, dtype=float).reshape(4)
        if initial is None:
            x = np.zeros(4, dtype=float)
        else:
            x = np.asarray(initial, dtype=float).reshape(4)
        x = np.minimum(np.maximum(x, lo), hi)
        lam = float(max(damping, 1e-9))
        h = a.T @ a + (lam * lam) * np.eye(4, dtype=float)
        g = a.T @ b
        try:
            lip = float(np.linalg.eigvalsh(h).max())
        except Exception:
            lip = float(np.linalg.norm(h, ord=2))
        step = 1.0 / max(lip, 1e-9)
        for _ in range(80):
            x_next = np.minimum(np.maximum(x - step * (h @ x + g), lo), hi)
            if float(np.linalg.norm(x_next - x)) <= 1e-10:
                x = x_next
                break
            x = x_next
        return x.copy(), a, b

    def _grasp_lji_compute_step_dq(
        self,
        servo: LocalImageJacobianServo3D,
        s_lji: np.ndarray,
        *,
        q: np.ndarray,
        approach_dir: np.ndarray,
        sag_model: Optional[dict[str, Any]],
        remain_m: float,
        pk: PickConfig,
        close_tol_m: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, int, float, bool, str]:
        max_lin, max_ang, max_t1, max_t2, scale = self._grasp_lji_step_limits(
            remain_m, pk, close_tol_m=close_tol_m
        )
        uv_err_mag = float(np.linalg.norm(np.asarray(s_lji, dtype=float).reshape(3)[:2]))
        uv_priority_err = float(max(pk.lij_uv_priority_err, 0.0))
        uv_priority = uv_priority_err > 1e-9 and uv_err_mag >= uv_priority_err
        if uv_priority:
            cap_scale = float(np.clip(pk.lij_uv_priority_cap_scale, 0.1, 2.0))
            linear_scale = float(np.clip(pk.lij_uv_priority_z_scale, 0.0, 1.0))
            max_lin *= linear_scale
            max_ang = max(float(max_ang), float(pk.lij_max_dq_roll) * cap_scale)
            max_t1 = max(float(max_t1), float(pk.lij_max_dq_theta1) * cap_scale)
            max_t2 = max(float(max_t2), float(pk.lij_max_dq_angle) * cap_scale)
        gain_z_scale = scale
        if uv_priority:
            gain_z_scale *= float(np.clip(pk.lij_uv_priority_z_scale, 0.0, 1.0))
        z_row = self._grasp_lji_fk_z_row(
            q,
            approach_dir,
            sag_model=sag_model,
        )
        dq, dq_raw, j, rank, cond, avail = servo.compute_dq(
            s_lji,
            z_row=z_row,
            max_dq_linear=max_lin,
            max_dq_angle=max_ang,
            max_dq_theta1=max_t1,
            max_dq_theta2=max_t2,
            gain_u=float(pk.lij_gain_u) * scale,
            gain_v=float(pk.lij_gain_v) * scale,
            gain_z=float(pk.lij_gain_z) * gain_z_scale,
        )
        controller = "local_img_jacobian"
        limit_flags = self._grasp_lji_joint_limit_flags(
            q,
            margin_m=float(pk.lij_joint_limit_margin_m),
            margin_rad=float(pk.lij_joint_limit_margin_rad),
            cfg=self._mapping_cfg,
        )
        active_limit_names = tuple(name for name, active in limit_flags.items() if bool(active))
        if active_limit_names and bool(self._host_native_lji_runtime()):
            lower = np.array([-max_lin, -max_ang, -max_t1, -max_t2], dtype=float)
            upper = np.array([+max_lin, +max_ang, +max_t1, +max_t2], dtype=float)
            if bool(limit_flags.get("linear_min", False)):
                lower[0] = max(float(lower[0]), 0.0)
            if bool(limit_flags.get("linear_max", False)):
                upper[0] = min(float(upper[0]), 0.0)
            if bool(limit_flags.get("roll_min", False)):
                lower[1] = max(float(lower[1]), 0.0)
            if bool(limit_flags.get("roll_max", False)):
                upper[1] = min(float(upper[1]), 0.0)
            if bool(limit_flags.get("theta1_min", False)):
                lower[2] = max(float(lower[2]), 0.0)
            if bool(limit_flags.get("theta1_max", False)):
                upper[2] = min(float(upper[2]), 0.0)
            if bool(limit_flags.get("theta2_min", False)):
                lower[3] = max(float(lower[3]), 0.0)
            if bool(limit_flags.get("theta2_max", False)):
                upper[3] = min(float(upper[3]), 0.0)
            dq_guarded = self._grasp_lji_guard_dq_at_limits(q, dq, pk=pk)
            dq_box, a_box, b_box = self._grasp_lji_solve_box_constrained(
                j,
                np.asarray(s_lji, dtype=float).reshape(3),
                lower=lower,
                upper=upper,
                damping=float(pk.lij_damping),
                gain_u=float(pk.lij_gain_u) * scale,
                gain_v=float(pk.lij_gain_v) * scale,
                gain_z=float(pk.lij_gain_z) * gain_z_scale,
                initial=dq_guarded,
            )
            obj_guarded = self._grasp_lji_box_objective(
                a_box,
                b_box,
                dq_guarded,
                damping=float(pk.lij_damping),
            )
            obj_box = self._grasp_lji_box_objective(
                a_box,
                b_box,
                dq_box,
                damping=float(pk.lij_damping),
            )
            if obj_box < obj_guarded - 1e-10:
                dq = dq_box
                dq_raw = dq_box.copy()
                controller = "local_img_jacobian_bounded_limits"
                if int(self._grasp_waypoint_idx) <= 3 or int(self._grasp_waypoint_idx) % 10 == 0:
                    print(
                        "[Grasp] LJI bounded-limit solve | active=%s bounds=[%+.4f,%+.4f,%+.4f,%+.4f]/[%+.4f,%+.4f,%+.4f,%+.4f]"
                        % (
                            ",".join(active_limit_names),
                            float(lower[0]),
                            float(lower[1]),
                            float(lower[2]),
                            float(lower[3]),
                            float(upper[0]),
                            float(upper[1]),
                            float(upper[2]),
                            float(upper[3]),
                        )
                    )
        bias_gain = float(max(0.0, pk.lij_approach_bias_gain)) * scale
        bias_gate = float(max(pk.lij_approach_bias_uv_gate, 0.0))
        if (
            bias_gain > 1e-9
            and float(remain_m) > float(close_tol_m) + 1e-4
            and (bias_gate <= 1e-9 or uv_err_mag <= bias_gate)
        ):
            seed = np.asarray(self._grasp_lji_q_delta4(pk.lij_approach_seed_q_delta), dtype=float)
            if float(np.linalg.norm(seed)) > 1e-9:
                j_uv = np.asarray(j[0:2, :], dtype=float).reshape(2, 4)
                n_uv = null_space_projector_mn(j_uv, damping=float(pk.lij_damping))
                dq_bias = bias_gain * (n_uv @ seed.reshape(4))
                # Only keep the posture/approach bias if FK predicts less axial remain.
                z_effect = float(np.dot(np.asarray(z_row, dtype=float).reshape(4), dq_bias))
                if z_effect < -1e-9:
                    dq = clip_dq(
                        np.asarray(dq, dtype=float).reshape(4) + dq_bias,
                        max_dq_linear=max_lin,
                        max_dq_angle=max_ang,
                        max_dq_theta1=max_t1,
                        max_dq_theta2=max_t2,
                    )
                    dq_raw = np.asarray(dq_raw, dtype=float).reshape(4) + dq_bias
        return dq, dq_raw, j, int(rank), float(cond), bool(avail), controller

    @staticmethod
    def _grasp_lji_joint_limit_flags(
        q: np.ndarray,
        *,
        margin_m: float,
        margin_rad: float,
        cfg: SimMappingConfig,
    ) -> dict[str, bool]:
        arr = np.asarray(q, dtype=float).reshape(4)
        m_lin = float(max(margin_m, 0.0))
        m_ang = float(max(margin_rad, 0.0))
        linear_min_m, linear_max_m = linear_effective_q_bounds(cfg)
        return {
            "linear_max": float(arr[0]) >= float(linear_max_m) - m_lin,
            "linear_min": float(arr[0]) <= float(linear_min_m) + m_lin,
            "roll_max": float(arr[1]) >= float(cfg.roll_q_max_rad) - m_ang,
            "roll_min": float(arr[1]) <= float(cfg.roll_q_min_rad) + m_ang,
            "theta1_max": float(arr[2]) >= float(cfg.seg1_q_max_rad) - m_ang,
            "theta1_min": float(arr[2]) <= float(cfg.seg1_q_min_rad) + m_ang,
            "theta2_max": float(arr[3]) >= float(cfg.seg2_q_max_rad) - m_ang,
            "theta2_min": float(arr[3]) <= float(cfg.seg2_q_min_rad) + m_ang,
        }

    def _grasp_lji_guard_dq_at_limits(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        *,
        pk: PickConfig,
    ) -> np.ndarray:
        """Zero dq components that would drive further into a joint limit."""
        out = np.asarray(dq, dtype=float).reshape(4).copy()
        flags = self._grasp_lji_joint_limit_flags(
            q,
            margin_m=float(pk.lij_joint_limit_margin_m),
            margin_rad=float(pk.lij_joint_limit_margin_rad),
            cfg=self._mapping_cfg,
        )
        if flags["linear_max"] and float(out[0]) > 0.0:
            out[0] = 0.0
        if flags["linear_min"] and float(out[0]) < 0.0:
            out[0] = 0.0
        if flags["roll_max"] and float(out[1]) > 0.0:
            out[1] = 0.0
        if flags["roll_min"] and float(out[1]) < 0.0:
            out[1] = 0.0
        if flags["theta1_max"] and float(out[2]) > 0.0:
            out[2] = 0.0
        if flags["theta1_min"] and float(out[2]) < 0.0:
            out[2] = 0.0
        if flags["theta2_max"] and float(out[3]) > 0.0:
            out[3] = 0.0
        if flags["theta2_min"] and float(out[3]) < 0.0:
            out[3] = 0.0
        return out

    def _grasp_lji_update_stall_watch(
        self,
        *,
        pk: PickConfig,
        remain_m: float,
        sample_reason: SampleRejectReason,
        q: np.ndarray,
        dq_meas: Optional[np.ndarray] = None,
    ) -> Optional[str]:
        """Abort only when hard joint limits block motion with no remain progress."""
        window = int(pk.lij_stall_steps)
        if window <= 0:
            return None
        flags = self._grasp_lji_joint_limit_flags(
            q,
            margin_m=float(pk.lij_joint_limit_margin_m),
            margin_rad=float(pk.lij_joint_limit_margin_rad),
            cfg=self._mapping_cfg,
        )
        at_hard_limit = any(bool(flags[k]) for k in flags)
        if not at_hard_limit:
            self._grasp_lji_sat_streak = 0
            return None
        if dq_meas is not None and bool(flags["linear_max"]):
            meas = np.asarray(dq_meas, dtype=float).reshape(4)
            if float(meas[0]) < -0.0003:
                self._grasp_lji_sat_streak = max(0, int(self._grasp_lji_sat_streak) - 2)
        if sample_reason == SampleRejectReason.JOINT_SATURATED:
            self._grasp_lji_sat_streak += 1
        else:
            self._grasp_lji_sat_streak = 0
        self._grasp_lji_remain_hist.append(float(remain_m))
        win = max(2, window)
        if len(self._grasp_lji_remain_hist) > win:
            self._grasp_lji_remain_hist = self._grasp_lji_remain_hist[-win:]
        if self._grasp_lji_sat_streak < win:
            return None
        remain_span = max(self._grasp_lji_remain_hist) - min(self._grasp_lji_remain_hist)
        if float(remain_span) > float(pk.lij_stall_remain_eps_m):
            return None
        blocked = [k for k in ("linear_max", "linear_min", "theta2_max", "theta2_min", "theta1_max", "theta1_min") if flags[k]]
        lim_txt = ",".join(blocked)
        return (
            "grasp lji | stall at remain=%.0fmm (%s saturated, no progress)"
            % (float(remain_m) * 1000.0, lim_txt)
        )

    def _grasp_lji_update_bad_motion_watch(
        self,
        *,
        pk: PickConfig,
        sample_reason: SampleRejectReason,
    ) -> Optional[str]:
        threshold = int(max(getattr(pk, "lij_bad_motion_reacquire_steps", 0), 0))
        if threshold <= 0:
            self._grasp_lji_bad_motion_streak = 0
            return None
        bad_reasons = {
            SampleRejectReason.JOINT_SATURATED,
            SampleRejectReason.MOTION_MISMATCH,
        }
        if sample_reason in bad_reasons:
            self._grasp_lji_bad_motion_streak += 1
        elif sample_reason == SampleRejectReason.ACCEPTED:
            self._grasp_lji_bad_motion_streak = 0
        else:
            self._grasp_lji_bad_motion_streak = max(
                0,
                int(self._grasp_lji_bad_motion_streak) - 1,
            )
        if int(self._grasp_lji_bad_motion_streak) < threshold:
            return None
        self._grasp_lji_bad_motion_streak = 0
        return "bad_motion"


class GraspLjiSafetyActions(GraspLjiSolverActions):
    """Depth gates, reliable-state latching, retract, and command smoothing."""

    def _grasp_lji_depth_snapshot(
        self,
        *,
        remain_m: float,
        tip_world: Optional[tuple[float, float, float]] = None,
        object_world: Optional[tuple[float, float, float]] = None,
        approach_dir: Optional[np.ndarray] = None,
    ) -> tuple[bool, float]:
        snap = self.perception_snapshot()
        depth_valid = bool(snap is not None and snap.depth_valid)
        z_axial = float(remain_m)
        if snap is not None and snap.p_camera is not None and depth_valid:
            z_axial = float(snap.p_camera[2])
        elif (
            tip_world is not None
            and object_world is not None
            and approach_dir is not None
        ):
            z_axial = self._grasp_axial_distance(
                tip_world,
                object_world,
                approach_dir,
            )
        self._grasp_depth_history.append(
            (bool(depth_valid), float(z_axial), float(remain_m))
        )
        return depth_valid, float(z_axial)

    def _grasp_lji_eval_depth_stability(
        self,
        pk: PickConfig,
        *,
        remain_m: float,
    ) -> tuple[bool, str]:
        hist = list(self._grasp_depth_history)
        if len(hist) < 2:
            return True, "insufficient_history"
        invalid_streak = 0
        for depth_valid, _, _ in reversed(hist):
            if not bool(depth_valid):
                invalid_streak += 1
            else:
                break
        if invalid_streak >= int(pk.lij_depth_invalid_frames):
            return False, "invalid_streak"
        valid_ratio = float(sum(1 for dv, _, _ in hist if dv)) / float(len(hist))
        if valid_ratio < float(pk.lij_depth_valid_ratio_min):
            return False, "valid_ratio"
        settled_delta = float(max(pk.lij_depth_settled_remain_delta_m, 1e-4))
        settled_z: list[float] = []
        prev_remain: Optional[float] = None
        for depth_valid, camera_z, hist_remain in hist:
            if not bool(depth_valid):
                prev_remain = None
                continue
            if prev_remain is not None:
                if abs(float(hist_remain) - float(prev_remain)) > settled_delta:
                    prev_remain = float(hist_remain)
                    continue
            settled_z.append(float(camera_z))
            prev_remain = float(hist_remain)
        if len(settled_z) >= 2:
            z_std = float(np.std(np.asarray(settled_z, dtype=float)))
            if z_std > float(pk.lij_depth_std_max_m):
                return False, "z_std"
        if float(remain_m) <= float(pk.lij_depth_unstable_threshold_m):
            return False, "close_range"
        return True, "ok"

    @staticmethod
    def _grasp_lji_should_blind_finish(remain_m: float, pk: PickConfig) -> bool:
        threshold = float(max(pk.blind_micro_start_m, 0.0))
        return float(remain_m) <= threshold + 1e-6

    def _grasp_lji_visual_tracking_lost(
        self,
        s_lji: Optional[np.ndarray],
        *,
        pk: PickConfig,
    ) -> bool:
        """True when |v| diverges even though the tracker may still return obs."""
        if s_lji is None:
            return False
        v_abs = abs(float(s_lji[1]))
        hard = float(max(pk.lij_reacquire_v_err_m, 0.15))
        if v_abs >= hard * 1.2:
            return True
        hist = list(self._grasp_lji_v_err_hist)
        if len(hist) >= 4 and v_abs >= hard:
            if v_abs > float(hist[-4]) + 0.10:
                return True
        if v_abs >= hard and int(self._grasp_lji_sat_streak) >= 3:
            return True
        return False

    def _grasp_lji_should_reacquire(
        self,
        *,
        object_lost: bool,
        remain_m: float,
        close_tol_m: float,
        pk: PickConfig,
    ) -> bool:
        if not bool(object_lost):
            return False
        if float(remain_m) <= float(close_tol_m) + 1e-4:
            return False
        return int(self._grasp_lji_reacquire_steps) < int(pk.lij_reacquire_max_steps)

    def _grasp_lji_begin_reacquire(
        self,
        *,
        prev_mode: GraspApproachMode,
        remain_m: float,
    ) -> None:
        if prev_mode == GraspApproachMode.REACQUIRE:
            return
        self._grasp_lji_reacquire_anchor_dq = None
        self._grasp_lji_reacquire_aim_tried = False
        self._grasp_lji_reacquire_prev_remain = float(remain_m)
        self._pick_lost_follow_count = 0
        self._grasp_lji_bad_motion_streak = 0
        est = self._grasp_lji_estimator_3d
        if est is not None:
            est.clear()

    def _grasp_lji_end_reacquire(self) -> None:
        self._grasp_lji_reacquire_anchor_dq = None
        self._grasp_lji_reacquire_steps = 0
        self._grasp_lji_reacquire_aim_tried = False
        self._grasp_lji_reacquire_prev_remain = None
        self._grasp_lji_v_err_hist = []
        self._grasp_lji_bad_motion_streak = 0

    def _grasp_lji_retract_dq_to_last_good_q(
        self,
        *,
        q_before: np.ndarray,
        pk: PickConfig,
    ) -> Optional[np.ndarray]:
        q_good = self._grasp_lji_last_good_q
        if q_good is None:
            return None
        dq = np.asarray(q_good, dtype=float).reshape(4) - np.asarray(
            q_before, dtype=float
        ).reshape(4)
        if float(np.linalg.norm(dq)) <= 1e-7:
            return None
        cap = max(
            float(pk.lij_reacquire_axial_step_m) * 3.0,
            float(pk.lij_max_dq_linear) * 2.0,
            float(pk.lij_max_dq_angle) * 2.0,
        )
        norm = float(np.linalg.norm(dq))
        if norm > cap:
            dq = dq * (cap / norm)
        return self._grasp_lji_guard_dq_at_limits(
            np.asarray(q_before, dtype=float).reshape(4),
            dq,
            pk=pk,
        )

    def _grasp_lji_compute_axial_retract_dq(
        self,
        *,
        pk: PickConfig,
        approach_dir: np.ndarray,
        object_world: tuple[float, float, float],
        sag_model: dict[str, Any],
        host_state: Optional[HostState],
        q_before: np.ndarray,
    ) -> Optional[np.ndarray]:
        """Retract along approach axis (increases remain), not joint-space -dq."""
        step_m = abs(float(pk.lij_reacquire_axial_step_m))
        if step_m <= 1e-6:
            return None
        ok, q_target = self._grasp_solve_axial_ik_q(
            distance_m=-step_m,
            approach_dir=approach_dir,
            object_world=object_world,
            sag_model=dict(sag_model),
            host_state=host_state,
            label="lji reacquire axial retract",
        )
        if not ok or q_target is None:
            return None
        dq = np.asarray(q_target, dtype=float).reshape(4) - np.asarray(
            q_before, dtype=float
        ).reshape(4)
        if float(np.linalg.norm(dq)) <= 1e-7:
            return None
        return self._grasp_lji_guard_dq_at_limits(
            np.asarray(q_before, dtype=float).reshape(4),
            dq,
            pk=pk,
        )

    def _grasp_lji_latch_reliable_state(
        self,
        *,
        object_world: tuple[float, float, float],
        approach_dir: np.ndarray,
        remain_m: float,
        host_state: Optional[HostState],
    ) -> None:
        self._grasp_lji_last_reliable_object_world = tuple(float(v) for v in object_world)
        dir_u = self._unit_vec3(approach_dir)
        self._grasp_lji_last_reliable_approach_dir = (
            float(dir_u[0]),
            float(dir_u[1]),
            float(dir_u[2]),
        )
        self._grasp_lji_last_reliable_depth = float(remain_m)
        raw_cam = getattr(host_state, "perceived_object_camera_xyz", None)
        if raw_cam is not None:
            try:
                p_cam = np.asarray(raw_cam, dtype=float).reshape(3)
                if bool(np.all(np.isfinite(p_cam))):
                    self._grasp_lji_last_reliable_camera_xyz = (
                        float(p_cam[0]),
                        float(p_cam[1]),
                        float(p_cam[2]),
                    )
            except (TypeError, ValueError):
                pass
        q = self._q_array_from_state(host_state)
        self._grasp_lji_last_good_q = q.copy()

    def _grasp_solve_axial_ik_q(
        self,
        *,
        distance_m: float,
        approach_dir: np.ndarray,
        object_world: tuple[float, float, float],
        sag_model: dict[str, Any],
        host_state: Optional[HostState],
        label: str,
    ) -> tuple[bool, Optional[np.ndarray]]:
        """Solve axial IK only (no host apply); returns q solution or None."""
        delta = float(distance_m)
        if abs(delta) <= 1e-6:
            return True, self._q_array_from_state(host_state)
        try:
            model = self._pick_reach_model(sag_model=sag_model)
        except Exception:
            return False, None
        q0 = self._q_array_from_state(host_state)
        tip0 = np.asarray(model.grasp_position(q0), dtype=float).reshape(3)
        axis_w = self._unit_vec3(approach_dir)
        target = tip0 + axis_w * delta
        try:
            dir_hold = self._grasp_look_at_dir(tip0, object_world)
        except ValueError:
            dir_hold = axis_w
        self.refresh_ik_context()
        ctx = self._ik_context_for_host(host_state, sag_model=sag_model)
        result = ik_pipeline.solve_then_align(
            target_world=target,
            target_dir_world=dir_hold,
            context=ctx,
            position_tol_m=self._grasp_step_position_tol_m(),
            max_iters=max(int(self._ik_cfg.max_iters), 1),
            current_seed=q0,
            **self._grasp_align_ik_kwargs(),
        )
        if not result.success or result.q is None:
            return False, None
        return True, np.asarray(result.q, dtype=float).reshape(4)

    def _grasp_lji_approach_seed_from_ik(
        self,
        *,
        pk: PickConfig,
        approach_dir: np.ndarray,
        object_world: tuple[float, float, float],
        sag_model: dict[str, Any],
        host_state: Optional[HostState],
    ) -> np.ndarray:
        q0 = self._q_array_from_state(host_state)
        ok, q_ik = self._grasp_solve_axial_ik_q(
            distance_m=float(pk.lij_approach_seed_travel_m),
            approach_dir=approach_dir,
            object_world=object_world,
            sag_model=dict(sag_model),
            host_state=host_state,
            label="lji approach seed ik",
        )
        if ok and q_ik is not None:
            return np.asarray(q_ik, dtype=float).reshape(4) - q0
        return np.zeros(4, dtype=float)

    def _grasp_lji_approach_seed(
        self,
        *,
        pk: PickConfig,
        approach_dir: np.ndarray,
        object_world: tuple[float, float, float],
        sag_model: dict[str, Any],
        host_state: Optional[HostState],
    ) -> np.ndarray:
        mode = str(pk.lij_approach_seed_mode).strip().lower()
        if mode == "axial_ik":
            return self._grasp_lji_approach_seed_from_ik(
                pk=pk,
                approach_dir=approach_dir,
                object_world=object_world,
                sag_model=sag_model,
                host_state=host_state,
            )
        return np.asarray(self._grasp_lji_q_delta4(pk.lij_approach_seed_q_delta), dtype=float)

    def _grasp_lji_smooth_dq(self, dq: np.ndarray, *, pk: PickConfig) -> np.ndarray:
        alpha = float(max(0.0, min(0.95, pk.lij_dq_smooth_alpha)))
        prev = self._grasp_lji_last_dq_cmd
        raw = np.asarray(dq, dtype=float).reshape(4)
        if alpha <= 1e-6 or prev is None:
            return raw.copy()
        blended = alpha * np.asarray(prev, dtype=float).reshape(4) + (1.0 - alpha) * raw
        return blended.reshape(4)

    @staticmethod
    def _grasp_lji_command_horizon(pk: PickConfig) -> float:
        raw = float(getattr(pk, "lij_command_horizon", 1.0))
        if not math.isfinite(raw):
            return 1.0
        return float(np.clip(raw, 1.0, 8.0))

    def _grasp_lji_apply_command_horizon(
        self,
        dq: np.ndarray,
        *,
        q_before: np.ndarray,
        pk: PickConfig,
    ) -> np.ndarray:
        horizon = self._grasp_lji_command_horizon(pk)
        dq_arr = np.asarray(dq, dtype=float).reshape(4)
        if horizon <= 1.0001:
            return dq_arr.copy()
        q0 = np.asarray(q_before, dtype=float).reshape(4)
        q_target = self._clamp_q(q0 + horizon * dq_arr)
        return np.asarray(q_target, dtype=float).reshape(4) - q0


class GraspLjiRuntimeActions(GraspLjiSafetyActions):
    """Measured-motion sampling, trace recording, reacquire, and blind handoff."""

    def _grasp_lji_wait_motion_fraction(
        self,
        *,
        q_before: np.ndarray,
        dq_cmd: np.ndarray,
        timeout_s: float,
        min_frac: float = 0.30,
    ) -> Optional[HostState]:
        """Pipelined LJI: poll until measured q moved a fraction of commanded dq."""
        if self.client is None:
            time.sleep(min(float(timeout_s), 0.05))
            return None
        qb = np.asarray(q_before, dtype=float).reshape(4)
        dq = np.asarray(dq_cmd, dtype=float).reshape(4)
        cmd_norm = float(np.linalg.norm(dq))
        if cmd_norm <= 1e-6:
            return self._refresh_lji_state()
        deadline = time.time() + float(max(timeout_s, 0.04))
        poll_s = 0.015 if not bool(self._use_hardware) else 0.03
        frac = float(np.clip(min_frac, 0.02, 0.90))
        last_state: Optional[HostState] = None
        while time.time() < deadline:
            time.sleep(poll_s)
            last_state = self._refresh_lji_state()
            if last_state is None:
                continue
            q_now = self._q_array_for_motion_feedback(last_state)
            meas = q_now - qb
            if float(np.linalg.norm(meas)) >= frac * cmd_norm:
                return last_state
            for i in range(4):
                cmd_i = float(dq[i])
                if abs(cmd_i) > 0.0008 and abs(float(meas[i])) >= frac * abs(cmd_i):
                    return last_state
        return last_state

    def _grasp_apply_q_delta(
        self,
        dq: np.ndarray,
        *,
        host_state: Optional[HostState],
        sag_model: dict[str, Any],
        timeout_s: float = 2.0,
        wait_settle: bool = True,
        step_period_s: float = 0.0,
        linear_tol_m: Optional[float] = None,
        angle_tol_rad: Optional[float] = None,
        motion_wait_frac: float = 0.15,
    ) -> tuple[np.ndarray, Optional[HostState]]:
        q0 = self._grasp_lji_command_base_q(host_state)
        dq_arr = np.asarray(dq, dtype=float).reshape(4)
        q_cmd = self._clamp_q(q0 + dq_arr)
        self.state.set_q(
            float(q_cmd[0]),
            float(q_cmd[1]),
            float(q_cmd[2]),
            float(q_cmd[3]),
        )
        motion_source = "lji_step"
        if self._host_native_lji_runtime():
            apply_direct = getattr(self.client, "apply_lji_q_direct")
            velocity_dt_s = float(max(step_period_s, 0.02))
            qdot_arr = np.asarray(q_cmd - q0, dtype=float).reshape(4) / velocity_dt_s
            host_after = apply_direct(
                SimQ(
                    linear_m=float(q_cmd[0]),
                    roll_rad=float(q_cmd[1]),
                    theta1_rad=float(q_cmd[2]),
                    theta2_rad=float(q_cmd[3]),
                ),
                qdot=SimQ(
                    linear_m=float(qdot_arr[0]),
                    roll_rad=float(qdot_arr[1]),
                    theta1_rad=float(qdot_arr[2]),
                    theta2_rad=float(qdot_arr[3]),
                ),
                qdot_ref=SimQ(
                    linear_m=float(q0[0]),
                    roll_rad=float(q0[1]),
                    theta1_rad=float(q0[2]),
                    theta2_rad=float(q0[3]),
                ),
                velocity_dt_s=velocity_dt_s,
                sag_model=dict(sag_model),
                target_xyz=(
                    float(self.state.target_x),
                    float(self.state.target_y),
                    float(self.state.target_z),
                ),
                target_dir=(
                    float(self.state.target_vx),
                    float(self.state.target_vy),
                    float(self.state.target_vz),
                ),
                claw_closed=bool(self.state.claw_closed),
                source=motion_source,
            )
            if host_after is None or bool(host_after.reply_ok):
                if (
                    host_after is not None
                    and str(getattr(host_after, "reply_reason", "")) == "host_native_lji_velocity"
                ):
                    self._grasp_lji_command_q = None
                else:
                    self._grasp_lji_command_q = q_cmd.copy()
            if float(motion_wait_frac) > 1e-6:
                wait_s = float(max(step_period_s, 0.04))
                host_after = self._grasp_lji_wait_motion_fraction(
                    q_before=q0,
                    dq_cmd=dq_arr,
                    timeout_s=wait_s,
                    min_frac=float(motion_wait_frac),
                ) or host_after
        elif bool(wait_settle):
            host_after = self._send_state_q_and_wait(
                timeout_s=float(timeout_s),
                source=motion_source,
                force=True,
                sag_model_override=dict(sag_model),
                linear_tol_m=linear_tol_m,
                angle_tol_rad=angle_tol_rad,
            )
        else:
            self.send_current_target(
                source=motion_source,
                force=True,
                sag_model_override=dict(sag_model),
            )
            wait_s = float(max(step_period_s, 0.06))
            host_after = self._grasp_lji_wait_motion_fraction(
                q_before=q0,
                dq_cmd=dq_arr,
                timeout_s=wait_s,
            )
            if host_after is None and self.client is not None:
                host_after = self.client.refresh_state()
        if host_after is not None and (not bool(host_after.reply_ok)):
            reason = str(host_after.reply_reason).strip() or "unknown"
            print(
                "[Grasp] lji apply failed | reason=%s q_cmd=%s"
                % (
                    reason,
                    "[%.4f,%.4f,%.4f,%.4f]"
                    % tuple(float(v) for v in np.asarray(q_cmd).reshape(4)),
                )
            )
        return q_cmd, host_after

    @staticmethod
    def _grasp_lji_motion_mismatch_reason(
        *,
        dq_cmd: np.ndarray,
        delta_q: np.ndarray,
        pk: PickConfig,
    ) -> str:
        cmd = np.asarray(dq_cmd, dtype=float).reshape(4)[1:4]
        meas = np.asarray(delta_q, dtype=float).reshape(4)[1:4]
        cmd_norm = float(np.linalg.norm(cmd))
        meas_norm = float(np.linalg.norm(meas))
        if cmd_norm <= max(float(pk.lij_sample_min_dq_norm), 1e-7):
            return ""
        if meas_norm <= 1e-9:
            return ""
        ratio = meas_norm / max(cmd_norm, 1e-9)
        ratio_min = float(max(getattr(pk, "lij_sample_meas_cmd_ratio_min", 0.0), 0.0))
        ratio_max = float(max(getattr(pk, "lij_sample_meas_cmd_ratio_max", 0.0), 0.0))
        if ratio_min > 1e-6 and ratio < ratio_min:
            return "motion_mismatch"
        if ratio_max > 1e-6 and ratio > ratio_max:
            return "motion_mismatch"
        cos_min = float(getattr(pk, "lij_sample_cmd_meas_cos_min", -1.0))
        if cos_min > -1.0:
            cos = float(np.dot(cmd, meas) / max(cmd_norm * meas_norm, 1e-9))
            if cos < cos_min:
                return "motion_mismatch"
        return ""

    def _grasp_lji_record_measured_sample(
        self,
        *,
        pk: PickConfig,
        settle_ok: bool,
        object_lost: bool,
        pipelined: bool = False,
    ) -> SampleRejectReason:
        pending = self._grasp_lji_pending_sample
        est = self._grasp_lji_estimator_3d
        if pending is None or est is None:
            return SampleRejectReason.DQ_TOO_SMALL
        q_before = np.asarray(pending["q_before"], dtype=float).reshape(4)
        s_before = np.asarray(pending["s_before"], dtype=float).reshape(3)
        if "q_after" not in pending:
            self._grasp_lji_pending_sample = None
            return SampleRejectReason.SETTLE_TIMEOUT
        q_after = np.asarray(pending["q_after"], dtype=float).reshape(4)
        s_after = np.asarray(
            pending.get("s_after", pending["s_before"]),
            dtype=float,
        ).reshape(3)
        dq_cmd = np.asarray(pending["dq_cmd"], dtype=float).reshape(4)
        delta_q = q_after - q_before
        delta_s = s_after - s_before
        saturated = joint_saturated(q_before, dq_cmd, q_after)
        ok, reason = check_sample_quality(
            delta_q=delta_q,
            min_dq_norm=float(pk.lij_sample_min_dq_norm),
            object_lost=bool(object_lost),
            settle_ok=bool(settle_ok),
            joint_saturated=bool(saturated),
        )
        if ok:
            mismatch_reason = self._grasp_lji_motion_mismatch_reason(
                dq_cmd=dq_cmd,
                delta_q=delta_q,
                pk=pk,
            )
            if mismatch_reason:
                est.clear()
                self._grasp_lji_last_dq_cmd = None
                self._grasp_lji_command_q = None
                self._stop_lji_velocity_control("lji_motion_mismatch")
                self._grasp_lji_pending_sample = None
                return SampleRejectReason.MOTION_MISMATCH
            est.push(delta_q, delta_s)
        self._grasp_lji_pending_sample = None
        return reason

    @staticmethod
    def _grasp_lji_log_float(value: Any) -> float:
        try:
            out = float(value)
        except Exception:
            return float("nan")
        return out if math.isfinite(out) else float("nan")

    @staticmethod
    def _grasp_lji_log_vec4(value: Optional[np.ndarray]) -> tuple[float, float, float, float]:
        if value is None:
            nan = float("nan")
            return nan, nan, nan, nan
        try:
            arr = np.asarray(value, dtype=float).reshape(4)
        except Exception:
            nan = float("nan")
            return nan, nan, nan, nan
        return tuple(float(v) for v in arr)

    def _grasp_lji_log_enabled(self) -> bool:
        raw = os.environ.get("ELESIM_LJI_LOG", "1").strip().lower()
        return raw not in {"0", "false", "no", "off"}

    def _grasp_lji_log_start(self) -> None:
        if not self._grasp_lji_log_enabled():
            return
        self._grasp_lji_log_close()
        raw_path = os.environ.get("ELESIM_LJI_LOG_PATH", "").strip()
        base = Path(raw_path) if raw_path else Path("logs/lji_grasp")
        if not base.is_absolute():
            base = Path.cwd() / base
        try:
            base.mkdir(parents=True, exist_ok=True)
            path = base / ("%s_lji_grasp.csv" % time.strftime("%Y%m%d_%H%M%S"))
            fh = open(path, "a", newline="", encoding="utf-8")
            fields = [
                "unix_s",
                "t_rel_s",
                "seq",
                "step_idx",
                "mode",
                "controller",
                "transition",
                "object_lost",
                "step_elapsed_s",
                "remain_before_m",
                "remain_before_mm",
                "remain_m",
                "remain_mm",
                "remain_delta_mm",
                "close_tol_m",
                "u_err",
                "v_err",
                "z_err",
                "depth_valid",
                "depth_valid_ratio",
                "j_rank",
                "j_cond",
                "j_available",
                "dq_cmd_0",
                "dq_cmd_1",
                "dq_cmd_2",
                "dq_cmd_3",
                "dq_meas_0",
                "dq_meas_1",
                "dq_meas_2",
                "dq_meas_3",
                "q_cmd_0",
                "q_cmd_1",
                "q_cmd_2",
                "q_cmd_3",
                "sample_reason",
                "ik_status",
                "note",
            ]
            writer = csv.DictWriter(fh, fieldnames=fields)
            writer.writeheader()
            fh.flush()
            self._grasp_lji_log_file = fh
            self._grasp_lji_log_writer = writer
            self._grasp_lji_log_path = str(path)
            self._grasp_lji_log_start_t = time.time()
            self._grasp_lji_log_seq = 0
            print(f"[Grasp-Ctrl] csv={self._grasp_lji_log_path}")
        except Exception as exc:
            self._grasp_lji_log_file = None
            self._grasp_lji_log_writer = None
            self._grasp_lji_log_path = ""
            print(f"[Grasp-Ctrl] csv open failed: {exc}")

    def _grasp_lji_log_close(self) -> None:
        fh = self._grasp_lji_log_file
        if fh is not None:
            try:
                fh.flush()
                fh.close()
            except Exception:
                pass
        self._grasp_lji_log_file = None
        self._grasp_lji_log_writer = None

    def _grasp_lji_log_write_control_step(
        self,
        *,
        step_idx: int,
        mode: GraspApproachMode,
        s_lji: Optional[np.ndarray],
        depth_valid: bool,
        depth_valid_ratio: float,
        j_rank: int,
        j_cond: float,
        j_available: bool,
        dq_cmd: np.ndarray,
        dq_meas: Optional[np.ndarray],
        q_cmd: np.ndarray,
        controller: str,
        transition: str,
        object_lost: int,
        remain_m: float,
        close_tol_m: float,
        remain_before_m: Optional[float] = None,
        step_elapsed_s: Optional[float] = None,
        ik_status: str,
        sample_reason: str,
        note: str = "",
    ) -> None:
        writer = self._grasp_lji_log_writer
        fh = self._grasp_lji_log_file
        if writer is None or fh is None:
            return
        u_err = float(s_lji[0]) if s_lji is not None else float("nan")
        v_err = float(s_lji[1]) if s_lji is not None else float("nan")
        z_err = float(s_lji[2]) if s_lji is not None else float("nan")
        dq0, dq1, dq2, dq3 = self._grasp_lji_log_vec4(dq_cmd)
        dm0, dm1, dm2, dm3 = self._grasp_lji_log_vec4(dq_meas)
        q0, q1, q2, q3 = self._grasp_lji_log_vec4(q_cmd)
        now = time.time()
        self._grasp_lji_log_seq += 1
        remain_after = self._grasp_lji_log_float(remain_m)
        remain_before = (
            self._grasp_lji_log_float(remain_before_m)
            if remain_before_m is not None
            else float("nan")
        )
        remain_delta_mm = (
            (remain_after - remain_before) * 1000.0
            if math.isfinite(remain_after) and math.isfinite(remain_before)
            else float("nan")
        )
        row = {
            "unix_s": now,
            "t_rel_s": now - float(self._grasp_lji_log_start_t or now),
            "seq": int(self._grasp_lji_log_seq),
            "step_idx": int(step_idx),
            "mode": str(mode.value),
            "controller": str(controller),
            "transition": str(transition),
            "object_lost": int(object_lost),
            "step_elapsed_s": self._grasp_lji_log_float(step_elapsed_s),
            "remain_before_m": remain_before,
            "remain_before_mm": remain_before * 1000.0 if math.isfinite(remain_before) else float("nan"),
            "remain_m": remain_after,
            "remain_mm": remain_after * 1000.0 if math.isfinite(remain_after) else float("nan"),
            "remain_delta_mm": remain_delta_mm,
            "close_tol_m": self._grasp_lji_log_float(close_tol_m),
            "u_err": self._grasp_lji_log_float(u_err),
            "v_err": self._grasp_lji_log_float(v_err),
            "z_err": self._grasp_lji_log_float(z_err),
            "depth_valid": int(bool(depth_valid)),
            "depth_valid_ratio": self._grasp_lji_log_float(depth_valid_ratio),
            "j_rank": int(j_rank),
            "j_cond": self._grasp_lji_log_float(j_cond),
            "j_available": int(bool(j_available)),
            "dq_cmd_0": dq0,
            "dq_cmd_1": dq1,
            "dq_cmd_2": dq2,
            "dq_cmd_3": dq3,
            "dq_meas_0": dm0,
            "dq_meas_1": dm1,
            "dq_meas_2": dm2,
            "dq_meas_3": dm3,
            "q_cmd_0": q0,
            "q_cmd_1": q1,
            "q_cmd_2": q2,
            "q_cmd_3": q3,
            "sample_reason": str(sample_reason),
            "ik_status": str(ik_status),
            "note": str(note),
        }
        try:
            writer.writerow(row)
            fh.flush()
        except Exception as exc:
            print(f"[Grasp-Ctrl] csv write failed: {exc}")

    def _grasp_lji_log_event(
        self,
        *,
        step_idx: int,
        mode: GraspApproachMode,
        note: str,
        remain_m: Optional[float] = None,
        close_tol_m: Optional[float] = None,
        object_lost: int = 0,
        transition: str = "-",
        step_elapsed_s: Optional[float] = None,
    ) -> None:
        nan4 = np.full(4, float("nan"), dtype=float)
        remain = float(remain_m) if remain_m is not None else float("nan")
        self._grasp_lji_log_write_control_step(
            step_idx=int(step_idx),
            mode=mode,
            s_lji=None,
            depth_valid=False,
            depth_valid_ratio=float("nan"),
            j_rank=0,
            j_cond=float("nan"),
            j_available=False,
            dq_cmd=nan4,
            dq_meas=None,
            q_cmd=nan4,
            controller="event",
            transition=str(transition),
            object_lost=int(object_lost),
            remain_m=remain,
            close_tol_m=float(close_tol_m) if close_tol_m is not None else float("nan"),
            remain_before_m=remain,
            step_elapsed_s=step_elapsed_s,
            ik_status="-",
            sample_reason="event",
            note=str(note),
        )

    def _grasp_lji_log_control_step(
        self,
        *,
        step_idx: int = 0,
        mode: GraspApproachMode,
        s_lji: Optional[np.ndarray],
        depth_valid: bool,
        depth_valid_ratio: float,
        j_rank: int,
        j_cond: float,
        j_available: bool,
        dq_cmd: np.ndarray,
        dq_meas: Optional[np.ndarray],
        q_cmd: np.ndarray,
        controller: str,
        transition: str,
        object_lost: int,
        remain_m: float,
        close_tol_m: float,
        remain_before_m: Optional[float] = None,
        step_elapsed_s: Optional[float] = None,
        ik_status: str,
        sample_reason: str,
        note: str = "",
    ) -> None:
        u_err = float(s_lji[0]) if s_lji is not None else float("nan")
        v_err = float(s_lji[1]) if s_lji is not None else float("nan")
        z_err = float(s_lji[2]) if s_lji is not None else float("nan")
        meas_txt = (
            "[%.4f,%.4f,%.4f,%.4f]" % tuple(float(v) for v in dq_meas.reshape(4))
            if dq_meas is not None
            else "n/a"
        )
        print(
            "[Grasp-Ctrl] mode=%s | u_err=%+.4f v_err=%+.4f z_err=%+.4f | "
            "depth_valid=%s depth_valid_ratio=%.2f | J3d_rank=%d (need>=3) "
            "J3d_cond=%.1f J3d_available=%s | dq_cmd=%s dq_meas=%s q_cmd=%s | "
            "sample=%s | controller=%s | transition=%s | object_lost=%d | "
            "remain=%.1fmm close_tol=%.1fmm | ik=%s"
            % (
                str(mode.value),
                u_err,
                v_err,
                z_err,
                str(bool(depth_valid)).lower(),
                float(depth_valid_ratio),
                int(j_rank),
                float(j_cond),
                str(bool(j_available)).lower(),
                "[%.4f,%.4f,%.4f,%.4f]"
                % tuple(float(v) for v in np.asarray(dq_cmd).reshape(4)),
                meas_txt,
                "[%.4f,%.4f,%.4f,%.4f]"
                % tuple(float(v) for v in np.asarray(q_cmd).reshape(4)),
                str(sample_reason),
                str(controller),
                str(transition),
                int(object_lost),
                float(remain_m) * 1000.0,
                float(close_tol_m) * 1000.0,
                str(ik_status),
            )
        )
        self._grasp_lji_log_write_control_step(
            step_idx=int(step_idx),
            mode=mode,
            s_lji=s_lji,
            depth_valid=bool(depth_valid),
            depth_valid_ratio=float(depth_valid_ratio),
            j_rank=int(j_rank),
            j_cond=float(j_cond),
            j_available=bool(j_available),
            dq_cmd=np.asarray(dq_cmd, dtype=float).reshape(4),
            dq_meas=dq_meas,
            q_cmd=np.asarray(q_cmd, dtype=float).reshape(4),
            controller=controller,
            transition=transition,
            object_lost=int(object_lost),
            remain_m=float(remain_m),
            remain_before_m=remain_before_m,
            step_elapsed_s=step_elapsed_s,
            close_tol_m=float(close_tol_m),
            ik_status=ik_status,
            sample_reason=sample_reason,
            note=note,
        )

    def _grasp_lji_blind_finish_if_needed(
        self,
        *,
        object_world: tuple[float, float, float],
        approach_dir: np.ndarray,
        nominal_world: tuple[float, float, float],
        host_state: Optional[HostState],
        sag_model: dict[str, Any],
        standoff_m: float,
        close_tol_m: float,
    ) -> tuple[bool, Optional[np.ndarray], Optional[HostState]]:
        """One-shot latched blind axial extend when remain is above close_tol."""
        tip = self._pick_current_tip_world(host_state=host_state)
        if tip is None:
            return False, None, host_state
        use_obj = self._grasp_lji_last_reliable_object_world or tuple(
            float(v) for v in object_world
        )
        use_dir = self._grasp_lji_last_reliable_approach_dir
        if use_dir is not None:
            dir_u = self._unit_vec3(use_dir)
        else:
            dir_u = self._unit_vec3(approach_dir)
        nominal = self._pick_grasp_trajectory_end_position(
            use_obj,
            dir_u,
            standoff_m=float(standoff_m),
        )
        remain = self._grasp_axial_distance(tip, nominal, dir_u)
        if float(remain) <= float(close_tol_m) + 1e-4:
            try:
                q_now = self._q_array_from_state(host_state)
            except Exception:
                q_now = None
            return True, q_now, host_state
        pk = self._pick_config_effective()
        if bool(pk.grasp_blind_uv_only):
            self._grasp_uv_only_mode = True
            print("[Grasp] LJI blind one-shot extend | uv-only perception kept")
        elif self._perception_capture is not None and self._perception_capture.is_running():
            self.stop_perception_capture(stop_recording=not bool(self.state.perception_recording))
            print("[Grasp] perception stopped | LJI blind one-shot extend | recording kept=%s" % str(bool(self.state.perception_recording)).lower())
        look_v = self._grasp_look_at_dir(tip, use_obj)
        handoff_look = (float(look_v[0]), float(look_v[1]), float(look_v[2]))
        self._grasp_handoff_look_dir = handoff_look
        print(
            "[Grasp] LJI blind finish | remain=%.1fmm > close_tol %.1fmm"
            % (float(remain) * 1000.0, float(close_tol_m) * 1000.0)
        )
        jac_ok, q_jac, host_state, _target_jac = self._grasp_lji_blind_finish_with_jacobian(
            object_world=use_obj,
            approach_dir=dir_u,
            nominal_world=tuple(float(v) for v in nominal),
            host_state=host_state,
            sag_model=dict(sag_model),
            standoff_m=float(standoff_m),
            close_tol_m=float(close_tol_m),
            initial_remain_m=float(remain),
        )
        if jac_ok:
            return True, q_jac, host_state
        print("[Grasp] LJI blind finish | learned axial step unavailable; fallback IK")
        blind_ok, q_cmd, host_state, _target = self._grasp_blind_final_approach(
            object_world=use_obj,
            look_dir=handoff_look,
            sag_model=dict(sag_model),
            host_state=host_state,
            grasp_standoff_m=float(standoff_m),
            approach_dir=dir_u,
            nominal_world=tuple(float(v) for v in nominal),
        )
        return bool(blind_ok), q_cmd, host_state

    def _grasp_lji_learned_z_row(
        self,
        *,
        pk: PickConfig,
    ) -> Optional[np.ndarray]:
        est = self._grasp_lji_estimator_3d
        if est is None:
            return None
        min_samples = max(1, min(int(pk.lij_min_samples), 2))
        measured = est.measured_estimate(
            min_samples=min_samples,
            condition_max=float("inf"),
            min_rank=1,
        )
        if measured is None:
            return None
        j_meas, _rank, _cond = measured
        row = np.asarray(j_meas, dtype=float).reshape(3, 4)[2, :].copy()
        if not np.all(np.isfinite(row)):
            return None
        if float(np.linalg.norm(row)) <= 1e-9:
            return None
        return row

    def _grasp_lji_blind_finish_with_jacobian(
        self,
        *,
        object_world: tuple[float, float, float],
        approach_dir: np.ndarray,
        nominal_world: tuple[float, float, float],
        host_state: Optional[HostState],
        sag_model: dict[str, Any],
        standoff_m: float,
        close_tol_m: float,
        initial_remain_m: float,
    ) -> tuple[bool, Optional[np.ndarray], Optional[HostState], tuple[float, float, float]]:
        """Final blind axial motion from the learned LJI z row; IK is fallback."""
        servo = self._grasp_lji_servo_3d
        if servo is None or self.client is None:
            return False, None, host_state, tuple(float(v) for v in nominal_world)
        pk = self._pick_config_effective()
        z_row = self._grasp_lji_learned_z_row(pk=pk)
        if z_row is None:
            return False, None, host_state, tuple(float(v) for v in nominal_world)

        obj_tuple = tuple(float(v) for v in object_world)
        nominal_arr = np.asarray(nominal_world, dtype=float).reshape(3)
        axis = self._unit_vec3(approach_dir)
        target_world = tuple(float(v) for v in nominal_arr)
        q_cmd: Optional[np.ndarray] = None
        max_steps = int(
            np.clip(
                math.ceil(
                    max(float(initial_remain_m), float(pk.blind_micro_start_m), 0.01)
                    / max(float(pk.lij_far_linear_cap_m), float(pk.lij_max_dq_linear), 1e-3)
                )
                + 2,
                2,
                16,
            )
        )
        step_period_s = float(max(pk.lij_step_period_s, 0.04))
        for step_idx in range(max_steps):
            tip = self._pick_current_tip_world(host_state=host_state)
            if tip is None:
                return False, q_cmd, host_state, target_world
            remain = self._grasp_axial_distance(tip, nominal_arr, axis)
            if float(remain) <= float(close_tol_m) + 1e-4:
                try:
                    q_now = self._q_array_from_state(host_state)
                except Exception:
                    q_now = q_cmd
                target_world = self._grasp_precontact_from_tip(
                    tip,
                    obj_tuple,
                    float(standoff_m),
                )
                print(
                    "[Grasp] LJI blind finish | learned axial done step=%d remain=%.1fmm"
                    % (int(step_idx), float(remain) * 1000.0)
                )
                return True, q_now, host_state, target_world

            q_before = self._grasp_lji_command_base_q(host_state)
            z_err = float(max(0.0, float(remain) - float(close_tol_m)))
            max_lin, max_ang, max_t1, max_t2, scale = self._grasp_lji_step_limits(
                float(remain),
                pk,
                close_tol_m=float(close_tol_m),
            )
            s_axial = np.array([0.0, 0.0, z_err], dtype=float)
            dq_cmd, _dq_raw, _j, rank, cond, _avail = servo.compute_dq(
                s_axial,
                z_row=z_row,
                max_dq_linear=max_lin,
                max_dq_angle=max_ang,
                max_dq_theta1=max_t1,
                max_dq_theta2=max_t2,
                gain_u=0.0,
                gain_v=0.0,
                gain_z=float(pk.lij_gain_z) * float(scale),
            )
            dq_cmd = self._grasp_lji_guard_dq_at_limits(
                q_before,
                dq_cmd,
                pk=pk,
            )
            if float(np.linalg.norm(dq_cmd)) <= 1e-7:
                return False, q_cmd, host_state, target_world
            dq_apply = self._grasp_lji_apply_command_horizon(
                dq_cmd,
                q_before=q_before,
                pk=pk,
            )
            z_pred = float(np.dot(z_row.reshape(4), np.asarray(dq_apply, dtype=float).reshape(4)))
            if z_pred >= -1e-7:
                print(
                    "[Grasp] LJI blind finish | learned z row predicts no approach dz=%.5f"
                    % float(z_pred)
                )
                return False, q_cmd, host_state, target_world
            q_cmd, host_state = self._grasp_apply_q_delta(
                dq_apply,
                host_state=host_state,
                sag_model=dict(sag_model),
                timeout_s=max(0.12, step_period_s * 2.0),
                wait_settle=False,
                step_period_s=step_period_s,
                motion_wait_frac=0.0 if self._host_native_lji_runtime() else 0.25,
            )
            print(
                "[Grasp] LJI blind finish | learned axial step=%d/%d remain=%.1fmm dz_pred=%.1fmm rank=%d cond=%.1f"
                % (
                    int(step_idx + 1),
                    int(max_steps),
                    float(remain) * 1000.0,
                    float(-z_pred) * 1000.0,
                    int(rank),
                    float(cond),
                )
            )

        tip_final = self._pick_current_tip_world(host_state=host_state)
        if tip_final is None:
            return False, q_cmd, host_state, target_world
        remain_final = self._grasp_axial_distance(tip_final, nominal_arr, axis)
        if float(remain_final) <= max(float(close_tol_m) * 3.0, 0.012) + 1e-4:
            target_world = self._grasp_precontact_from_tip(
                tip_final,
                obj_tuple,
                float(standoff_m),
            )
            print(
                "[Grasp] LJI blind finish | learned axial accepted remain=%.1fmm"
                % (float(remain_final) * 1000.0)
            )
            return True, q_cmd, host_state, target_world
        return False, q_cmd, host_state, target_world

    def _grasp_lji_try_reacquire(
        self,
        *,
        grasp_cfg: PickConfig,
        host_state: Optional[HostState],
        pk: PickConfig,
    ) -> tuple[bool, Optional[VisualObservation], Optional[HostState]]:
        if not self._grasp_visual_recover_supported():
            return False, self.current_visual_observation(host_state), host_state
        centered_ok, obs, host_state = self._grasp_aim_recover_after_move(
            cfg=grasp_cfg,
            host_state=host_state,
            label="lji reacquire",
        )
        return bool(centered_ok), obs, host_state

    def _grasp_complete_precontact_and_close(
        self,
        *,
        live_object: tuple[float, float, float],
        nominal_live: tuple[float, float, float],
        dir_u: np.ndarray,
        q_cmd: np.ndarray,
        host_state: Optional[HostState],
        sag_model: dict[str, Any],
        waypoint_count: int,
        claw_label: str = "grasp pre-contact",
    ) -> bool:
        object_tuple = tuple(float(v) for v in live_object)
        target_arr = np.asarray(nominal_live, dtype=float).reshape(3)
        actual_offset_m = float(
            np.linalg.norm(np.asarray(object_tuple, dtype=float).reshape(3) - target_arr)
        )
        self.send_grasp_meta(source="target")
        self._send_grasp_target_markers(
            object_world=object_tuple,
            target=target_arr,
            direction=dir_u,
            actual_offset_m=actual_offset_m,
            corrected=bool(self._pick_grasp_uses_equal_sag()),
        )
        closed_ok, claw_suffix = self._close_gripper_after_grasp_arrival(
            host_state=host_state,
            q_cmd=q_cmd,
            target_world=target_arr,
            sag_model=dict(sag_model),
            label=str(claw_label),
            nominal_world=tuple(float(v) for v in nominal_live),
            approach_dir=dir_u,
        )
        if not bool(closed_ok):
            return False
        done_msg = "grasp done | waypoints=%d | %s" % (
            int(waypoint_count),
            str(claw_suffix),
        )
        self.state.set_pick_status(
            running=False,
            failed=False,
            phase=ObjectPickPhase.DONE.value,
            msg=done_msg,
        )
        self.state.set_ik_status(
            running=False,
            converged=True,
            failed=False,
            err_m=0.0,
            msg=done_msg,
        )
        print("[Grasp] %s" % done_msg)
        return True


class GraspTrackingActions(GraspLjiRuntimeActions):
    """Object filtering, visual recovery, and sag correction during approach."""

    def _grasp_init_filtered_tracking(
        self,
        object_world: tuple[float, float, float],
        approach_dir: tuple[float, float, float] | np.ndarray,
    ) -> None:
        obj = tuple(float(v) for v in object_world)
        dir_u = self._unit_vec3(approach_dir)
        self._grasp_object_world_filtered = obj
        self._grasp_approach_dir_filtered = (
            float(dir_u[0]),
            float(dir_u[1]),
            float(dir_u[2]),
        )

    def _grasp_filtered_object_world(self) -> Optional[tuple[float, float, float]]:
        if self._grasp_object_world_filtered is not None:
            return tuple(float(v) for v in self._grasp_object_world_filtered)
        return self._pick_grasp_object_world()

    def _grasp_filtered_approach_dir(self) -> Optional[tuple[float, float, float]]:
        if self._grasp_approach_dir_filtered is not None:
            return tuple(float(v) for v in self._grasp_approach_dir_filtered)
        return self._grasp_aim_latched_direction()

    def _grasp_update_filtered_tracking(
        self,
        *,
        tip_world: Optional[tuple[float, float, float]],
        pk: PickConfig,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """EMA-update object_world and approach_dir during guided grasp."""
        obj_f = self._grasp_object_world_filtered
        dir_f = self._grasp_approach_dir_filtered
        if obj_f is None or dir_f is None:
            seed_obj = self._pick_grasp_object_world()
            seed_dir = self._grasp_aim_latched_direction(object_world=seed_obj)
            if seed_obj is None or seed_dir is None:
                raise RuntimeError("grasp filtered tracking missing seed")
            self._grasp_init_filtered_tracking(seed_obj, seed_dir)
            obj_f = self._grasp_object_world_filtered
            dir_f = self._grasp_approach_dir_filtered
        assert obj_f is not None and dir_f is not None

        alpha_obj = float(np.clip(pk.grasp_object_filter_alpha, 0.0, 1.0))
        alpha_dir = float(np.clip(pk.grasp_approach_filter_alpha, 0.0, 1.0))

        if (not bool(self._grasp_uv_only_mode)) and alpha_obj > 1e-6:
            snap = self.perception_snapshot()
            if (
                snap is not None
                and bool(snap.depth_valid)
                and snap.p_world is not None
                and not self._grasp_lji_camera_outlier_reason(
                    getattr(snap, "p_camera", None),
                    pk=pk,
                )
            ):
                live = tuple(float(v) for v in snap.p_world)
                obj_f = tuple(
                    (1.0 - alpha_obj) * float(obj_f[i]) + alpha_obj * live[i]
                    for i in range(3)
                )
                self._grasp_object_world_filtered = obj_f

        if tip_world is not None and alpha_dir > 1e-6:
            try:
                chord = self._grasp_look_at_dir(tip_world, obj_f)
                dir_arr = np.asarray(dir_f, dtype=float).reshape(3)
                blended = (1.0 - alpha_dir) * dir_arr + alpha_dir * chord
                dir_u = self._unit_vec3(blended)
                dir_f = (float(dir_u[0]), float(dir_u[1]), float(dir_u[2]))
                self._grasp_approach_dir_filtered = dir_f
            except ValueError:
                pass

        return obj_f, dir_f

    def _grasp_aim_latched_direction(
        self,
        object_world: Optional[tuple[float, float, float]] = None,
    ) -> Optional[tuple[float, float, float]]:
        """Approach axis from Aim/Look latch (not tip→object chord at Grasp click)."""
        if self._pick_resolved_ready_dir_world is not None:
            return self._pick_resolved_ready_dir_world
        if self._pick_look_dir_world is not None:
            return self._pick_look_dir_world
        return self._pick_ready_direction(
            object_world=object_world,
            prefer_current_tip=False,
        )

    def _live_camera_feedback_enabled(self) -> bool:
        """True when real camera perception can drive post-move UV loops."""
        pk = self._pick_config_effective()
        if bool(pk.grasp_skip_aim_recover_in_mock):
            mode = str(self._perception_cfg.mode).strip().lower()
            if mode == "mock" or not bool(self._use_hardware):
                return False
        return True

    def _grasp_visual_recover_supported(self) -> bool:
        """True when live perception can close the post-IK UV aim loop."""
        if not self._live_camera_feedback_enabled():
            return False
        cap = self._perception_capture
        if cap is None or not cap.is_running():
            return False
        return True

    def _look_post_sag_trim_to_object(
        self,
        *,
        object_world: tuple[float, float, float],
        sag_model: dict[str, Any],
        host_state: Optional[HostState],
    ) -> Optional[HostState]:
        """Re-align grasp axis toward object after Look move (compensate sag pointing error)."""
        if host_state is None or host_state.q is None:
            return host_state
        tip = self._pick_current_tip_world(host_state=host_state)
        if tip is None:
            return host_state
        try:
            look_dir = self._grasp_look_at_dir(tip, object_world)
        except ValueError:
            print("[Look] sag-trim | degenerate tip-object geometry")
            return host_state
        ok, host_state = self._grasp_align_to_approach_dir(
            approach_dir=look_dir,
            sag_model=sag_model,
            host_state=host_state,
            label="look | sag-trim look-at",
            apply_timeout_s=6.0,
        )
        if not ok:
            print("[Look] sag-trim | align IK failed (continue)")
            return host_state
        self._pick_latch_fk_achieved_pose(host_state=host_state, sag_model=sag_model)
        if self._pick_achieved_dir_world is not None:
            d = tuple(float(v) for v in self._pick_achieved_dir_world)
            self._pick_look_dir_world = d
            self._pick_resolved_ready_dir_world = d
        return host_state

    def _look_post_move_uv_recover(
        self,
        *,
        pk: PickConfig,
        host_state: Optional[HostState],
        object_world: tuple[float, float, float],
        sag_model: dict[str, Any],
    ) -> Optional[HostState]:
        """Center object in image after Look IK (roll/seg) when sag shifted the view."""
        if not self._live_camera_feedback_enabled():
            print("[Look] post-move uv recover | skipped (mock/sim)")
            return host_state
        if self._perception_capture is None or not self._perception_capture.is_running():
            self._maybe_start_local_perception()

        acquire_s = float(max(pk.look_post_uv_acquire_s, 0.5))
        max_steps = max(1, int(pk.look_post_uv_max_steps))
        tol = float(max(pk.look_post_uv_center_tol, 0.01))
        recover_cfg = replace(self._pick_config_effective(), center_tol=tol)

        deadline = time.time() + acquire_s
        obs: Optional[VisualObservation] = None
        while time.time() < deadline:
            if self.client is not None:
                host_state = self.client.refresh_state()
            obs = self.current_visual_observation(host_state)
            if obs is not None:
                break
            time.sleep(0.05)

        if obs is None:
            print("[Look] post-move uv recover | no observation within %.1fs" % acquire_s)
            return host_state

        centered_ok, obs, stall = self._grasp_uv_center_until_tol(
            obs,
            cfg=recover_cfg,
            max_total_steps=int(max_steps),
        )
        if obs is not None:
            u_d, v_d, _, _ = self._visual_uv_errors(obs)
            print(
                "[Look] post-move uv recover | centered=%s tol=%.3f steps<=%d uv=(%+.3f,%+.3f)%s"
                % (
                    str(bool(centered_ok)).lower(),
                    float(tol),
                    int(max_steps),
                    float(u_d),
                    float(v_d),
                    (" | stall=%s" % str(stall)) if stall else "",
                )
            )

        live = self._pick_latest_object_world()
        if live is not None:
            live_tuple = tuple(float(v) for v in live)
            self._pick_look_object_world_xyz = live_tuple
            self._pick_frozen_world_xyz = live_tuple
            self._pick_initial_object_world_xyz = live_tuple

        if self.client is not None:
            host_state = self.client.refresh_state()
        self._pick_latch_fk_achieved_pose(host_state=host_state, sag_model=sag_model)
        if self._pick_achieved_dir_world is not None:
            d = tuple(float(v) for v in self._pick_achieved_dir_world)
            self._pick_look_dir_world = d
            self._pick_resolved_ready_dir_world = d
        return host_state

    @staticmethod
    def _grasp_motion_apply_timeout_s(pk: PickConfig) -> float:
        """Host apply timeout for one grasp IK step (motion + partial dwell budget)."""
        motion = float(max(pk.grasp_waypoint_settle_timeout_s, 0.0))
        dwell = float(max(pk.grasp_waypoint_settle_s, 0.0))
        return max(motion + 0.5 * dwell, 6.0)

    def _grasp_lji_refresh_after_step(
        self,
        *,
        q_cmd: np.ndarray,
        host_state: Optional[HostState],
        label: str,
        dwell_s: float,
        settle_timeout_s: float,
        linear_tol_m: float,
        angle_tol_rad: float,
    ) -> Optional[HostState]:
        """LJI continuous motion: skip blocking settle unless dwell/timeout configured."""
        if float(settle_timeout_s) <= 1e-6 and float(dwell_s) <= 1e-6:
            if self.client is not None:
                return self.client.refresh_state()
            return host_state
        state = self._grasp_wait_waypoint_settle(
            q_cmd=q_cmd,
            host_state=host_state,
            label=label,
            settle_s=float(dwell_s),
            settle_timeout_s=float(settle_timeout_s),
            linear_tol_m=float(linear_tol_m),
            angle_tol_rad=float(angle_tol_rad),
        )
        if state is not None:
            return state
        if self.client is not None:
            return self.client.refresh_state()
        return host_state

    def _grasp_wait_waypoint_settle(
        self,
        *,
        q_cmd: np.ndarray,
        host_state: Optional[HostState],
        label: str,
        settle_s: float,
        settle_timeout_s: float,
        linear_tol_m: float = 2e-3,
        angle_tol_rad: float = math.radians(2.0),
    ) -> Optional[HostState]:
        """Wait for commanded q to settle, then dwell before the next waypoint."""
        dwell = float(max(settle_s, 0.0))
        motion_budget = float(max(settle_timeout_s, 0.0))
        if dwell <= 1e-6 and motion_budget <= 1e-6:
            return host_state

        total_budget = motion_budget + dwell
        deadline = time.time() + max(total_budget, 0.05)
        settled = False
        poll_s = 0.35
        while time.time() < deadline:
            remaining = deadline - time.time()
            if remaining <= 1e-3:
                break
            settled_state, ok = self._wait_until_q_settled(
                q_cmd,
                timeout_s=min(poll_s, remaining),
                linear_tol_m=float(linear_tol_m),
                angle_tol_rad=float(angle_tol_rad),
            )
            if settled_state is not None:
                host_state = settled_state
            if ok:
                settled = True
                break

        if settled and dwell > 1e-6:
            dwell_remaining = max(0.0, deadline - time.time())
            if dwell_remaining > 1e-3:
                time.sleep(dwell_remaining)
                if self.client is not None:
                    host_state = self.client.refresh_state()

        if not settled:
            print(
                "[Grasp] %s | settle | q_ok=false | budget=%.2fs (blocking next wp)"
                % (str(label), total_budget)
            )
            return None

        print(
            "[Grasp] %s | settle | q_ok=true dwell=%.2fs"
            % (str(label), dwell)
        )
        return host_state

    def _grasp_clip_sag_update(
        self,
        base_sag: dict[str, Any],
        current: Optional[dict[str, Any]],
        estimate: EqualSagEstimate,
        *,
        max_step_deg: float,
    ) -> dict[str, Any]:
        if not bool(estimate.accepted):
            if isinstance(current, dict) and current:
                return dict(current)
            return dict(base_sag)
        prev = dict(current) if isinstance(current, dict) and current else dict(base_sag)
        s1_prev = float(prev.get("seg1_equal_offset_deg", 0.0))
        s2_prev = float(prev.get("seg2_equal_offset_deg", 0.0))
        max_step = float(max(0.0, max_step_deg))
        s1 = s1_prev + float(
            np.clip(float(estimate.seg1_equal_offset_deg) - s1_prev, -max_step, max_step)
        )
        s2 = s2_prev + float(
            np.clip(float(estimate.seg2_equal_offset_deg) - s2_prev, -max_step, max_step)
        )
        return apply_equal_sag_offsets(
            base_sag,
            seg1_equal_offset_deg=float(s1),
            seg2_equal_offset_deg=float(s2),
        )

    def _grasp_update_online_sag_bias(
        self,
        *,
        host_state: Optional[HostState],
        object_world: tuple[float, float, float],
        approach_dir: np.ndarray,
        pk: PickConfig,
        label: str = "",
        min_lateral_m: float = 0.0,
    ) -> tuple[float, float]:
        if not bool(pk.grasp_online_sag_enabled):
            return 0.0, 0.0
        if host_state is None or host_state.q is None:
            return 0.0, 0.0
        base_sag = (
            dict(self.state.raw_sag_model)
            if isinstance(self.state.raw_sag_model, dict)
            else {}
        )
        sag_model = self._pick_grasp_sag_model()
        try:
            model = self._pick_reach_model(sag_model=sag_model)
            q0 = self._q_array_from_state(host_state)
            tip = np.asarray(model.grasp_position(q0), dtype=float).reshape(3)
            fk_dir = np.asarray(model.grasp_direction(q0), dtype=float).reshape(3)
            fk_norm = float(np.linalg.norm(fk_dir))
            if fk_norm <= 1e-9:
                return 0.0, 0.0
            fk_dir = fk_dir / fk_norm
        except Exception:
            return 0.0, 0.0
        obj = np.asarray(object_world, dtype=float).reshape(3)
        axis = self._unit_vec3(approach_dir)
        look_ref = self._grasp_look_at_dir(tip, object_world)
        standoff_axial = float(np.dot(obj - tip, axis))
        if standoff_axial <= 1e-4:
            return 0.0, 0.0
        try:
            desired = np.asarray(
                compute_ready_pose_target(
                    tuple(float(v) for v in obj),
                    tuple(float(v) for v in axis),
                    standoff_m=float(standoff_axial),
                ),
                dtype=float,
            ).reshape(3)
        except ValueError:
            return 0.0, 0.0
        drift = desired - tip
        prepared = prepare_sag_drift_input(
            drift_world=drift,
            axis_world=axis,
            reference_dir=look_ref,
            max_dir_error_deg=float(max(pk.grasp_waypoint_max_approach_drift_deg, 5.0)),
            max_lateral_m=float(pk.sag_drift_max_lateral_m),
            min_axial_m=0.0005,
            axial_only=False,
        )
        if not bool(prepared.usable):
            if label:
                print(
                    "[Grasp] %s | sag skipped | reason=%s axial=%.1fmm lateral=%.1fmm dir_err=%.1fdeg"
                    % (
                        str(label),
                        str(prepared.reason),
                        float(prepared.axial_m) * 1000.0,
                        float(prepared.lateral_m) * 1000.0,
                        float(prepared.dir_error_deg),
                    )
                )
            return 0.0, 0.0
        if float(prepared.lateral_m) < float(min_lateral_m):
            return 0.0, 0.0
        self.refresh_ik_context()
        ctx = self._ik_context_for_host(host_state)
        try:
            estimate = estimate_equal_sag_from_ready_pose_drift(
                context=ctx,
                q4=q0,
                ready_pose_drift_world=prepared.sag_input_world,
                sag_model=base_sag,
            )
        except Exception:
            return 0.0, 0.0
        prev = (
            dict(self._grasp_online_sag_model)
            if isinstance(self._grasp_online_sag_model, dict)
            else dict(base_sag)
        )
        s1_prev = float(prev.get("seg1_equal_offset_deg", 0.0))
        s2_prev = float(prev.get("seg2_equal_offset_deg", 0.0))
        updated = self._grasp_clip_sag_update(
            base_sag,
            self._grasp_online_sag_model,
            estimate,
            max_step_deg=float(pk.grasp_online_sag_max_step_deg),
        )
        self._grasp_online_sag_model = dict(updated)
        d1 = float(updated.get("seg1_equal_offset_deg", 0.0)) - s1_prev
        d2 = float(updated.get("seg2_equal_offset_deg", 0.0)) - s2_prev
        if label and (abs(d1) > 1e-4 or abs(d2) > 1e-4):
            print(
                "[Grasp] %s | sag | d_seg1=%+.2fdeg d_seg2=%+.2fdeg axial=%.1fmm lateral=%.1fmm dir_err=%.1fdeg"
                % (
                    str(label),
                    float(d1),
                    float(d2),
                    float(prepared.axial_m) * 1000.0,
                    float(prepared.lateral_m) * 1000.0,
                    float(prepared.dir_error_deg),
                )
            )
        return float(d1), float(d2)

    def _grasp_uv_center_until_tol(
        self,
        obs: VisualObservation,
        *,
        cfg: PickConfig,
        max_total_steps: int = 48,
    ) -> tuple[bool, Optional[VisualObservation], str]:
        current_obs = obs
        center_tol = float(cfg.center_tol)
        stall = ""
        best_err = float("inf")
        stuck_iters = 0
        stuck_limit = max(6, int(cfg.center_stuck_iters))
        total_steps = max(1, int(max_total_steps))

        for _ in range(total_steps):
            conv = evaluate_pick_convergence(current_obs, cfg=cfg)
            if bool(conv.center_ok):
                return True, current_obs, stall
            err_mag = max(abs(float(conv.u_err)), abs(float(conv.v_err)))
            if err_mag < best_err - float(self._pick_aim_progress_eps):
                best_err = float(err_mag)
                stuck_iters = 0
            else:
                stuck_iters += 1
            if stuck_iters >= stuck_limit:
                stall = "stuck"
                break
            use_fallback = err_mag > max(
                center_tol * 2.0,
                float(self._pick_aim_gain_fallback_uv) * 0.25,
            )
            current_u = self.current_control_u()
            next_u, _, _, _ = self._apply_pick_center_step(
                current_obs,
                current_u,
                cfg=cfg,
                coupled_axes=True,
                fallback_gains=bool(use_fallback),
            )
            if next_u == current_u:
                stall = "clamp"
                break
            self._send_display_control_u_and_wait(next_u, timeout_s=1.0, source="slider")
            time.sleep(float(self._pick_aim_settle_s))
            host_state = self.client.refresh_state() if self.client is not None else None
            refreshed = self.current_visual_observation(host_state)
            if refreshed is None:
                stall = "no_obs"
                break
            current_obs = refreshed

        conv = evaluate_pick_convergence(current_obs, cfg=cfg)
        return bool(conv.center_ok), current_obs, stall

    def _grasp_aim_recover_after_move(
        self,
        *,
        cfg: PickConfig,
        host_state: Optional[HostState],
        max_total_steps: int = 12,
        label: str = "",
    ) -> tuple[bool, Optional[VisualObservation], Optional[HostState]]:
        """Re-center UV after IK (same tol as Aim; runs until centered or step cap)."""
        if self.client is not None:
            host_state = self.client.refresh_state()
        obs = self.current_visual_observation(host_state)
        if obs is None:
            return False, None, host_state
        u0, v0, _, _ = self._visual_uv_errors(obs)
        err0 = max(abs(float(u0)), abs(float(v0)))
        tol = float(max(cfg.center_tol, 1e-3))
        steps = max(
            int(max_total_steps),
            int(np.ceil(err0 / tol * 4.0)) + 6,
        )
        steps = min(steps, 60)
        centered_ok, obs, stall = self._grasp_uv_center_until_tol(
            obs,
            cfg=cfg,
            max_total_steps=int(steps),
        )
        if (not bool(centered_ok)) and obs is not None and stall in ("stuck", "clamp", ""):
            u_d, v_d, _, _ = self._visual_uv_errors(obs)
            err1 = max(abs(float(u_d)), abs(float(v_d)))
            if err1 > tol + 1e-4:
                self._reset_pick_uv_jacobian()
                retry_steps = min(60, int(steps) + int(np.ceil(err1 / tol * 3.0)))
                centered_ok, obs, stall = self._grasp_uv_center_until_tol(
                    obs,
                    cfg=cfg,
                    max_total_steps=int(retry_steps),
                )
        if label and obs is not None:
            u_d, v_d, _, _ = self._visual_uv_errors(obs)
            extra = (" | stall=%s" % str(stall)) if stall else ""
            print(
                "[Grasp] %s | aim recover | centered=%s tol=%.3f steps<=%d uv=(%+.3f,%+.3f)%s"
                % (
                    str(label),
                    str(bool(centered_ok)).lower(),
                    float(tol),
                    int(steps),
                    float(u_d),
                    float(v_d),
                    extra,
                )
            )
        return bool(centered_ok), obs, host_state


class GraspWaypointActions(GraspTrackingActions):
    """Waypoint IK, direction alignment, Cartesian advance, and final approach."""

    def _grasp_ik_to_waypoint(
        self,
        *,
        waypoint: GraspWaypoint,
        sag_model: dict[str, Any],
        host_state: Optional[HostState],
        label: str = "grasp waypoint",
        seed_override: Optional[np.ndarray] = None,
    ) -> tuple[bool, Optional[np.ndarray], Optional[HostState], float]:
        """IK to an absolute planned grasp waypoint pose (position + direction)."""
        if self.client is None or host_state is None or host_state.q is None:
            return False, None, host_state, float("inf")
        target = np.asarray(waypoint.position_world, dtype=float).reshape(3)
        target_dir = np.asarray(waypoint.direction_world, dtype=float).reshape(3)
        try:
            if seed_override is not None:
                q0 = np.asarray(seed_override, dtype=float).reshape(4)
            else:
                q0 = self._q_array_from_state(host_state)
        except Exception as exc:
            print("[Grasp] %s | seed failed: %s" % (str(label), str(exc)))
            return False, None, host_state, float("inf")

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
            print("[Grasp] %s | missing ik_context fields" % str(label))
            return False, None, host_state, float("inf")

        object_world = self._pick_grasp_object_world()
        ik_kwargs = self._grasp_align_ik_kwargs()
        ik_call: dict[str, Any] = {
            "target_world": target,
            "target_dir_world": target_dir,
            "context": ctx,
            "position_tol_m": max(float(self._ik_cfg.tol), 1e-4),
            "max_iters": max(int(self._ik_cfg.max_iters), 1),
            "current_seed": q0,
            **ik_kwargs,
        }
        if object_world is not None:
            ik_call["object_world"] = tuple(float(v) for v in object_world)

        self.state.set_ik_status(
            running=True,
            converged=False,
            failed=False,
            err_m=float("inf"),
            msg=str(label),
        )
        result = ik_pipeline.solve_then_align(
            target_world=ik_call["target_world"],
            target_dir_world=ik_call["target_dir_world"],
            context=ik_call["context"],
            position_tol_m=float(ik_call["position_tol_m"]),
            max_iters=int(ik_call["max_iters"]),
            current_seed=ik_call["current_seed"],
            tweak_position_hold_tol_m=float(ik_call["tweak_position_hold_tol_m"]),
            tweak_rounds=int(ik_call["tweak_rounds"]),
            align_mode=ik_call["align_mode"],
            align_skip_under_deg=ik_call["align_skip_under_deg"],
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
                "[Grasp] %s | IK failed | reason=%s err=%.4fm"
                % (str(label), str(result.reason), float(result.position_error_m))
            )
            return False, None, host_state, float(result.position_error_m)

        q1 = np.asarray(result.q, dtype=float).reshape(4)
        err_m = float(result.position_error_m)
        pk = self._pick_config_effective()
        max_drift_rad = math.radians(
            float(max(pk.grasp_waypoint_max_approach_drift_deg, 0.0))
        )
        ik_target_dir = target_dir
        if object_world is not None:
            try:
                model = self._pick_reach_model(sag_model=sag_model)
                tip1 = np.asarray(model.grasp_position(q1), dtype=float).reshape(3)
                fk_dir = np.asarray(model.grasp_direction(q1), dtype=float).reshape(3)
                look_vec = (
                    np.asarray(object_world, dtype=float).reshape(3) - tip1
                )
                look_len = float(np.linalg.norm(look_vec))
                if look_len > 1e-9:
                    look_u = look_vec / look_len
                    ik_target_dir = look_u
                    fk_norm = float(np.linalg.norm(fk_dir))
                    if fk_norm > 1e-9:
                        drift = float(
                            np.arccos(
                                float(
                                    np.clip(
                                        float(np.dot(fk_dir / fk_norm, look_u)),
                                        -1.0,
                                        1.0,
                                    )
                                )
                            )
                        )
                        if drift > max_drift_rad + 1e-9:
                            print(
                                "[Grasp] %s | FK look-at drift %.1f deg > tol %.1f deg"
                                % (
                                    str(label),
                                    float(np.degrees(drift)),
                                    float(pk.grasp_waypoint_max_approach_drift_deg),
                                )
                            )
                            return False, None, host_state, err_m
            except Exception:
                pass
        align_msg = "%s | err=%.1fmm standoff=%.0fmm" % (
            str(label),
            err_m * 1000.0,
            float(waypoint.standoff_m) * 1000.0,
        )
        if result.align_attempted:
            align_msg = "%s | look-at %.1f -> %.1f deg" % (
                align_msg,
                float(np.degrees(result.initial_direction_angle_rad)),
                float(np.degrees(result.direction_angle_rad)),
            )
        host_state = self._apply_ik_solution_to_host(
            q1,
            ik_target=target,
            ik_target_dir=ik_target_dir,
            err_m=err_m,
            status_msg=align_msg,
            timeout_s=3.0,
            sag_model_override=dict(sag_model),
        )
        if host_state is not None and (not bool(host_state.reply_ok)):
            return False, q1, host_state, err_m

        reached, _, host_state = self._wait_until_grasp_target_reached(
            target_world=target,
            q_cmd=q1,
            sag_model=sag_model,
            timeout_s=5.0,
            position_tol_m=max(float(self._ik_cfg.tol), 0.012),
        )
        if not bool(reached):
            print("[Grasp] %s | settle timeout (continue)" % str(label))
        print(
            "[Grasp] %s | ik | target=(%.3f, %.3f, %.3f) err=%.1fmm"
            % (
                str(label),
                float(target[0]),
                float(target[1]),
                float(target[2]),
                err_m * 1000.0,
            )
        )
        return True, q1, host_state, err_m

    def _grasp_align_to_approach_dir(
        self,
        *,
        approach_dir: np.ndarray,
        sag_model: dict[str, Any],
        host_state: Optional[HostState],
        label: str = "grasp align",
        apply_timeout_s: float = 6.0,
    ) -> tuple[bool, Optional[HostState]]:
        """Align grasp axis toward ``approach_dir`` without advancing the grasp point."""
        if self.client is None or host_state is None or host_state.q is None:
            return False, host_state
        try:
            model = self._pick_reach_model(sag_model=sag_model, host_state=host_state)
            q0 = self._q_array_from_state(host_state)
            tip0 = np.asarray(model.grasp_position(q0), dtype=float).reshape(3)
            target_dir = self._unit_vec3(approach_dir)
        except Exception as exc:
            print(f"[Grasp] {label} | align precompute failed: {exc}")
            return False, host_state

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
            print(f"[Grasp] {label} | missing ik_context fields")
            return False, host_state

        self.state.set_ik_status(
            running=True,
            converged=False,
            failed=False,
            err_m=float("inf"),
            msg=str(label),
        )
        result = ik_pipeline.solve_then_align(
            target_world=tip0,
            target_dir_world=target_dir,
            context=ctx,
            position_tol_m=max(float(self._ik_cfg.tol), 1e-4),
            max_iters=max(int(self._ik_cfg.max_iters), 1),
            current_seed=q0,
            **self._grasp_align_ik_kwargs(),
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
                "[Grasp] %s | align IK failed | reason=%s err=%.4fm"
                % (str(label), str(result.reason), float(result.position_error_m))
            )
            return False, host_state

        q1 = np.asarray(result.q, dtype=float).reshape(4)
        align_msg = str(label)
        if result.align_attempted:
            align_msg = "%s | dir %.1f -> %.1f deg" % (
                str(label),
                float(np.degrees(result.initial_direction_angle_rad)),
                float(np.degrees(result.direction_angle_rad)),
            )
        host_state = self._apply_ik_solution_to_host(
            q1,
            ik_target=tip0,
            ik_target_dir=np.asarray(target_dir, dtype=float).reshape(3),
            err_m=float(result.position_error_m),
            status_msg=align_msg,
            timeout_s=float(apply_timeout_s),
            sag_model_override=dict(sag_model),
        )
        return True, host_state

    def _grasp_cartesian_advance_along_dir(
        self,
        distance_m: float,
        approach_dir: np.ndarray,
        *,
        object_world: Optional[tuple[float, float, float]] = None,
        look_dir_hold: Optional[np.ndarray] = None,
        sag_model: dict[str, Any],
        host_state: Optional[HostState] = None,
        label: str = "grasp waypoint",
        apply_timeout_s: float = 6.0,
    ) -> tuple[bool, float, Optional[np.ndarray], Optional[HostState]]:
        delta = float(max(0.0, distance_m))
        if delta <= 1e-6:
            return True, 0.0, None, host_state
        try:
            model = self._pick_reach_model(sag_model=sag_model)
        except Exception as exc:
            print("[Grasp] %s | reach model failed: %s" % (str(label), str(exc)))
            return False, 0.0, None, host_state

        q0 = self._q_array_from_state(host_state)
        tip0 = np.asarray(model.grasp_position(q0), dtype=float).reshape(3)
        axis_w = self._unit_vec3(approach_dir)
        target = tip0 + axis_w * delta
        if look_dir_hold is not None:
            dir_hold = self._unit_vec3(look_dir_hold)
        elif object_world is not None:
            try:
                dir_hold = self._grasp_look_at_dir(tip0, object_world)
            except ValueError:
                dir_hold = axis_w
        else:
            dir_hold = axis_w

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
            print("[Grasp] %s | missing ik_context fields" % str(label))
            return False, 0.0, None, host_state

        self.state.set_ik_status(
            running=True,
            converged=False,
            failed=False,
            err_m=float("inf"),
            msg=str(label),
        )
        pos_tol = self._grasp_step_position_tol_m()
        result = ik_pipeline.solve_then_align(
            target_world=target,
            target_dir_world=dir_hold,
            context=ctx,
            position_tol_m=pos_tol,
            max_iters=max(int(self._ik_cfg.max_iters), 1),
            current_seed=q0,
            **self._grasp_align_ik_kwargs(),
        )
        accept_best = False
        if (not result.success) and result.q is not None:
            err_m = float(result.position_error_m)
            q_try = np.asarray(result.q, dtype=float).reshape(4)
            tip_try = np.asarray(model.grasp_position(q_try), dtype=float).reshape(3)
            travel_try = float(np.dot(tip_try - tip0, axis_w))
            accept_best = (
                str(result.reason) == "position tolerance not reached"
                and err_m <= max(pos_tol * 2.0, 0.006)
                and travel_try >= 0.001
            )
            if accept_best:
                print(
                    "[Grasp] %s | IK best-effort | err=%.1fmm travel=%.1fmm"
                    % (str(label), err_m * 1000.0, travel_try * 1000.0)
                )
        if (not result.success or result.q is None) and not accept_best:
            self.state.set_ik_status(
                running=False,
                converged=False,
                failed=True,
                err_m=float(result.position_error_m),
                msg=str(result.reason),
            )
            print(
                "[Grasp] %s | IK failed | reason=%s err=%.4fm"
                % (str(label), str(result.reason), float(result.position_error_m))
            )
            return False, 0.0, None, host_state

        q1 = np.asarray(result.q, dtype=float).reshape(4)
        tip1 = np.asarray(model.grasp_position(q1), dtype=float).reshape(3)
        travel = float(np.dot(tip1 - tip0, axis_w))
        if travel < 1e-6:
            self.state.set_ik_status(
                running=False,
                converged=False,
                failed=True,
                err_m=float(result.position_error_m),
                msg="no motion along approach axis",
            )
            print("[Grasp] %s | no motion along approach axis" % str(label))
            return False, 0.0, None, host_state

        align_msg = "%s | %.0fmm" % (str(label), delta * 1000.0)
        if result.align_attempted:
            align_msg = "%s | dir %.1f -> %.1f deg" % (
                align_msg,
                float(np.degrees(result.initial_direction_angle_rad)),
                float(np.degrees(result.direction_angle_rad)),
            )
        ik_target_dir = dir_hold
        if look_dir_hold is None and object_world is not None:
            try:
                ik_target_dir = self._grasp_look_at_dir(tip1, object_world)
            except ValueError:
                ik_target_dir = dir_hold
        host_state = self._apply_ik_solution_to_host(
            q1,
            ik_target=target,
            ik_target_dir=ik_target_dir,
            err_m=float(result.position_error_m),
            status_msg=align_msg,
            timeout_s=float(apply_timeout_s),
            sag_model_override=dict(sag_model),
        )
        if host_state is not None and (not bool(host_state.reply_ok)):
            return False, 0.0, q1, host_state
        ref_dir = look_dir_hold if look_dir_hold is not None else None
        if ref_dir is not None or object_world is not None:
            if ref_dir is not None:
                look_err = self._grasp_fk_dir_error_deg(model, q1, ref_dir)
            else:
                look_err = self._grasp_fk_look_at_error_deg(model, q1, object_world)
            pk = self._pick_config_effective()
            print(
                "[Grasp] %s | travel=%.1fmm look_err=%.1fdeg (tol %.1fdeg)"
                % (
                    str(label),
                    float(travel) * 1000.0,
                    float(look_err),
                    float(pk.grasp_waypoint_max_approach_drift_deg),
                )
            )
        return True, max(0.0, travel), q1, host_state

    def _grasp_advance_waypoint_ik(
        self,
        *,
        tip_world: tuple[float, float, float],
        nominal_world: tuple[float, float, float],
        object_world: tuple[float, float, float],
        approach_dir: np.ndarray,
        step_m: float,
        guided_handoff_m: float,
        sag_model: dict[str, Any],
        host_state: Optional[HostState],
        waypoint_idx: int,
        apply_timeout_s: float = 6.0,
    ) -> tuple[bool, Optional[np.ndarray], Optional[HostState]]:
        dist = self._grasp_axial_distance(tip_world, nominal_world, approach_dir)
        margin = max(0.0, float(dist) - float(guided_handoff_m))
        travel_m = min(float(step_m), float(margin))
        if travel_m <= 1e-6:
            return True, None, host_state
        label = "grasp waypoint %d" % int(waypoint_idx)
        ok = False
        travel_actual = 0.0
        q_cmd = None
        travel_try = float(travel_m)
        for _bisect in range(6):
            if travel_try < 1e-4:
                break
            ok, travel_actual, q_cmd, host_state = self._grasp_cartesian_advance_along_dir(
                travel_try,
                approach_dir,
                object_world=object_world,
                sag_model=sag_model,
                host_state=host_state,
                label=label if _bisect == 0 else "%s | bisect" % label,
                apply_timeout_s=float(apply_timeout_s),
            )
            if ok and q_cmd is not None and float(travel_actual) > 1e-6:
                break
            travel_try *= 0.5
        if not ok or q_cmd is None:
            return False, None, host_state
        pk = self._pick_config_effective()
        try:
            model = self._pick_reach_model(sag_model=sag_model)
            look_err = self._grasp_fk_look_at_error_deg(model, q_cmd, object_world)
            if look_err > float(pk.grasp_waypoint_max_approach_drift_deg):
                look_dir = self._grasp_look_at_dir(
                    model.grasp_position(q_cmd),
                    object_world,
                )
                align_ok, host_state = self._grasp_align_to_approach_dir(
                    approach_dir=look_dir,
                    sag_model=sag_model,
                    host_state=host_state,
                    label="%s | look-at" % str(label),
                    apply_timeout_s=float(apply_timeout_s),
                )
                if align_ok and host_state is not None and host_state.q is not None:
                    q_cmd = self._q_array_from_state(host_state)
        except Exception:
            pass
        return True, q_cmd, host_state

    def _grasp_blind_final_approach(
        self,
        *,
        object_world: tuple[float, float, float],
        look_dir: tuple[float, float, float],
        sag_model: dict[str, Any],
        host_state: Optional[HostState],
        grasp_standoff_m: float = 0.0,
        approach_dir: np.ndarray,
        nominal_world: tuple[float, float, float],
    ) -> tuple[bool, Optional[np.ndarray], Optional[HostState], tuple[float, float, float]]:
        """One-shot blind extend: latched tip→object look + axial move to nominal."""
        obj_tuple = tuple(float(v) for v in object_world)
        standoff_target = float(max(grasp_standoff_m, 0.0))
        reach_tol_m = max(float(self._ik_cfg.tol), 0.003)
        pk = self._pick_config_effective()
        dir_tol_deg = float(max(pk.grasp_waypoint_max_approach_drift_deg, 1.0))
        axis = self._unit_vec3(approach_dir)
        look_u = self._unit_vec3(look_dir)
        look_hold = np.asarray(look_u, dtype=float).reshape(3)
        nominal_arr = np.asarray(nominal_world, dtype=float).reshape(3)
        q_cmd: Optional[np.ndarray] = None
        target_world = obj_tuple

        tip = self._pick_current_tip_world(host_state=host_state)
        if tip is None:
            return False, q_cmd, host_state, target_world
        remain0 = self._grasp_axial_distance(tip, nominal_arr, axis)
        if remain0 <= reach_tol_m + 1e-4:
            try:
                q_cmd = self._q_array_from_state(host_state)
            except Exception:
                q_cmd = None
            target_world = self._grasp_precontact_from_tip(
                tip,
                obj_tuple,
                standoff_target,
            )
            print(
                "[Grasp] blind done | already at nominal | axial_remain=%.1fmm"
                % (float(remain0) * 1000.0)
            )
            return True, q_cmd, host_state, target_world

        try:
            model = self._pick_reach_model(sag_model=sag_model)
            q_seed = self._q_array_from_state(host_state)
            look_err0 = self._grasp_fk_dir_error_deg(model, q_seed, look_hold)
        except Exception:
            look_err0 = float("inf")

        print(
            "[Grasp] blind extend | axial_remain=%.1fmm look_err=%.1fdeg (latched look)"
            % (float(remain0) * 1000.0, float(look_err0))
        )

        if look_err0 > dir_tol_deg + 0.5:
            align_ok, host_state = self._grasp_align_to_approach_dir(
                approach_dir=look_hold,
                sag_model=sag_model,
                host_state=host_state,
                label="grasp blind | pre-align",
            )
            if not align_ok:
                print("[Grasp] blind | pre-align failed (continue extend)")

        travel_hi = max(0.0, float(remain0) - float(reach_tol_m))
        ok = False
        q_step: Optional[np.ndarray] = None
        travel_actual = 0.0
        travel = float(travel_hi)
        tip_arr = np.asarray(tip, dtype=float).reshape(3)
        for _bisect in range(6):
            if travel < 1e-4:
                break
            ok, travel_actual, q_step, host_state = self._grasp_cartesian_advance_along_dir(
                travel,
                axis,
                object_world=obj_tuple,
                look_dir_hold=look_hold,
                sag_model=sag_model,
                host_state=host_state,
                label="grasp blind" if _bisect == 0 else "grasp blind | bisect",
            )
            if ok and q_step is not None and float(travel_actual) > 1e-6:
                break
            travel *= 0.5
        if not ok or q_step is None:
            print(
                "[Grasp] blind extend | IK failed | axial_remain=%.1fmm"
                % (float(remain0) * 1000.0)
            )
            return False, q_cmd, host_state, target_world

        q_cmd = q_step
        target_pos = tip_arr + axis * float(max(0.0, travel_actual))
        reached, _, host_state = self._wait_until_grasp_target_reached(
            target_world=target_pos,
            q_cmd=q_cmd,
            sag_model=sag_model,
            timeout_s=10.0,
            position_tol_m=max(reach_tol_m, 0.012),
        )
        if not bool(reached):
            print("[Grasp] blind extend | settle timeout (continue)")

        try:
            model = self._pick_reach_model(sag_model=sag_model)
            look_err1 = self._grasp_fk_dir_error_deg(model, q_cmd, look_hold)
            if look_err1 > dir_tol_deg + 0.5:
                print(
                    "[Grasp] blind | post look_err=%.1fdeg > tol %.1fdeg | re-align"
                    % (float(look_err1), float(dir_tol_deg))
                )
                align_ok, host_state = self._grasp_align_to_approach_dir(
                    approach_dir=look_hold,
                    sag_model=sag_model,
                    host_state=host_state,
                    label="grasp blind | post-align",
                )
                if align_ok and host_state is not None and host_state.q is not None:
                    q_cmd = self._q_array_from_state(host_state)
                    look_err1 = self._grasp_fk_dir_error_deg(model, q_cmd, look_hold)
            print(
                "[Grasp] blind extend | travel=%.1fmm look_err=%.1fdeg"
                % (float(travel_actual) * 1000.0, float(look_err1))
            )
        except Exception:
            pass

        tip_final = self._pick_current_tip_world(host_state=host_state)
        if tip_final is None:
            return False, q_cmd, host_state, target_world
        remain_final = self._grasp_axial_distance(tip_final, nominal_arr, axis)
        if remain_final > max(reach_tol_m * 3.0, 0.012) + 1e-4:
            print(
                "[Grasp] blind abort | axial_remain=%.1fmm > tol %.1fmm"
                % (float(remain_final) * 1000.0, float(reach_tol_m) * 1000.0)
            )
            return False, q_cmd, host_state, target_world

        target_world = self._grasp_precontact_from_tip(
            tip_final,
            obj_tuple,
            standoff_target,
        )
        print(
            "[Grasp] blind done | axial_remain=%.1fmm (pre-contact)"
            % (float(remain_final) * 1000.0)
        )
        return True, q_cmd, host_state, target_world


class GraspGuidedActions(GraspWaypointActions):
    """Public guided-grasp setup and the legacy guided worker."""

    def _start_grasp_guided_approach(self, *, internal: bool = False) -> bool:
        """Start online UV→sag→axial-IK loop toward pre-contact (no offline plan)."""
        if not internal and (self.state.ik_running or self._visual_busy()):
            self._set_pick_failure("busy")
            return False
        if self.client is None:
            self._set_pick_failure("no host client")
            return False
        object_world = self._pick_grasp_object_world()
        if object_world is None:
            self._set_pick_failure("grasp missing object (run Look or enable perception)")
            return False
        dir_tuple = self._grasp_aim_latched_direction(object_world=object_world)
        if dir_tuple is None:
            self._set_pick_failure("cannot infer grasp approach direction")
            return False

        pk = self._pick_config_effective()
        dir_u = self._unit_vec3(dir_tuple)
        dir3 = (float(dir_u[0]), float(dir_u[1]), float(dir_u[2]))
        standoff_m = float(max(pk.grasp_standoff_m, 0.0))
        live_object = tuple(float(v) for v in object_world)
        self._grasp_init_filtered_tracking(live_object, dir_u)
        look_anchor = self._pick_grasp_trajectory_start_position()
        nominal_world = self._pick_grasp_trajectory_end_position(
            live_object,
            dir_u,
            standoff_m=standoff_m,
        )

        self._grasp_nominal_dir = dir3
        self._grasp_trajectory_nominal_pose = tuple(float(v) for v in nominal_world)
        self._grasp_executed_waypoints = []
        self._grasp_look_anchor = (
            tuple(float(v) for v in look_anchor) if look_anchor is not None else None
        )

        host_state = self.client.refresh_state()
        tip = self._pick_current_tip_world(host_state=host_state)
        if tip is None:
            self._set_pick_failure("grasp | tip FK unavailable")
            return False
        self._grasp_traj_start = tip

        prev_worker = self._ik_worker
        if prev_worker is not None and prev_worker.is_alive():
            print("[Grasp] stopping previous guided worker")
            self._pick_stop_event.set()
            prev_worker.join(timeout=2.0)
        self._pick_stop_event.clear()

        self._grasp_waypoint_idx = 0
        base_sag = self._pick_grasp_sag_model()
        self._grasp_online_sag_model = dict(base_sag) if base_sag else None
        if bool(pk.local_img_jacobian_enabled):
            self._grasp_init_lji_controller(pk)

        self.state.set_pick_status(
            running=True,
            failed=False,
            phase=ObjectPickPhase.GRASP_APPROACH.value,
            msg="grasp starting",
        )
        self.state.set_ik_status(
            running=True,
            converged=False,
            failed=False,
            err_m=float("inf"),
            msg="grasp",
        )

        def _worker() -> None:
            self._run_grasp_guided_approach_worker(
                object_world=live_object,
                approach_dir=dir_u,
                nominal_world=tuple(float(v) for v in nominal_world),
            )

        self._ik_worker = threading.Thread(
            target=traced_thread_target("pick.grasp", _worker),
            name="grasp-guided",
            daemon=True,
        )
        self._ik_worker.start()
        return True

    def _run_grasp_guided_approach_worker(
        self,
        *,
        object_world: tuple[float, float, float],
        approach_dir: np.ndarray,
        nominal_world: tuple[float, float, float],
    ) -> None:
        """Guided grasp worker: LJI path or legacy axial-IK waypoint loop."""
        pk = self._pick_config_effective()
        if bool(pk.local_img_jacobian_enabled):
            self._run_grasp_lji_approach_worker(
                object_world=object_world,
                approach_dir=approach_dir,
                nominal_world=nominal_world,
            )
            return
        self._run_grasp_guided_legacy_approach_worker(
            object_world=object_world,
            approach_dir=approach_dir,
            nominal_world=nominal_world,
        )

    def _run_grasp_guided_legacy_approach_worker(
        self,
        *,
        object_world: tuple[float, float, float],
        approach_dir: np.ndarray,
        nominal_world: tuple[float, float, float],
    ) -> None:
        """Legacy loop: UV center → sag → axial IK while remain > guided handoff."""
        pk = self._pick_config_effective()
        grasp_cfg = self._pick_config_for_grasp()
        step_m = float(max(pk.grasp_waypoint_step_m, 0.005))
        guided_handoff_m = float(max(pk.grasp_guided_handoff_m, 0.0))
        max_waypoints = max(1, int(pk.grasp_max_waypoints))
        waypoint_settle_s = float(max(pk.grasp_waypoint_settle_s, 0.0))
        waypoint_settle_timeout_s = float(max(pk.grasp_waypoint_settle_timeout_s, 0.0))
        motion_apply_timeout_s = self._grasp_motion_apply_timeout_s(pk)
        standoff_m = float(max(pk.grasp_standoff_m, 0.0))
        reach_tol_m = max(float(self._ik_cfg.tol), 0.005)
        success = False
        traj_start = self._grasp_traj_start
        look_anchor = self._grasp_look_anchor
        if traj_start is None:
            self._set_pick_failure("grasp | start position missing")
            return
        try:
            if self._perception_capture is None or not self._perception_capture.is_running():
                self._maybe_start_local_perception()

            host_state = self.client.refresh_state() if self.client is not None else None
            q_cmd: Optional[np.ndarray] = None
            sag_model = self._pick_grasp_sag_model()
            live_object, dir_tuple_seed = self._grasp_update_filtered_tracking(
                tip_world=self._pick_current_tip_world(host_state=host_state),
                pk=pk,
            )
            dir_u = self._unit_vec3(dir_tuple_seed)
            dir_tuple = (float(dir_u[0]), float(dir_u[1]), float(dir_u[2]))
            print(
                "[Grasp] guided start | handoff=%.0fmm standoff=%.0fmm "
                "step=%.0fmm settle=%.2fs motion_tol=%.2fs uv_center_tol=%.3f "
                "blind_uv_only=%s obj_alpha=%.2f dir_alpha=%.2f"
                % (
                    guided_handoff_m * 1000.0,
                    standoff_m * 1000.0,
                    step_m * 1000.0,
                    waypoint_settle_s,
                    waypoint_settle_timeout_s,
                    float(grasp_cfg.center_tol),
                    str(bool(pk.grasp_blind_uv_only)).lower(),
                    float(pk.grasp_object_filter_alpha),
                    float(pk.grasp_approach_filter_alpha),
                )
            )

            wp_idx = 0
            while wp_idx < max_waypoints:
                step_t0 = time.time()
                if self._pick_stop_event.is_set():
                    self.state.set_pick_status(
                        running=False,
                        failed=False,
                        phase=ObjectPickPhase.IDLE.value,
                        msg="grasp stopped",
                    )
                    return

                tip = self._pick_current_tip_world(host_state=host_state)
                if tip is None:
                    self._set_pick_failure("grasp guided | tip FK unavailable")
                    return

                try:
                    live_object, dir_tuple_live = self._grasp_update_filtered_tracking(
                        tip_world=tip,
                        pk=pk,
                    )
                except RuntimeError:
                    live_object = self._grasp_filtered_object_world() or object_world
                    dir_live = self._grasp_filtered_approach_dir()
                    if dir_live is None:
                        self._set_pick_failure("grasp guided | filtered tracking unavailable")
                        return
                    dir_tuple_live = dir_live
                dir_u = self._unit_vec3(dir_tuple_live)
                dir_tuple = (float(dir_u[0]), float(dir_u[1]), float(dir_u[2]))
                nominal_live = self._pick_grasp_trajectory_end_position(
                    live_object,
                    dir_u,
                    standoff_m=standoff_m,
                )
                remain = self._grasp_axial_distance(tip, nominal_live, dir_u)
                if remain <= guided_handoff_m + 1e-4:
                    print(
                        "[Grasp] guided handoff | remain=%.1fmm <= %.0fmm (blind extend)"
                        % (float(remain) * 1000.0, guided_handoff_m * 1000.0)
                    )
                    break
                if remain <= reach_tol_m + 1e-4:
                    print(
                        "[Grasp] nominal reached | remain=%.1fmm"
                        % (float(remain) * 1000.0)
                    )
                    break

                wp_idx += 1
                self._grasp_waypoint_idx = int(wp_idx)
                wp_label = "wp %d" % int(wp_idx)

                if q_cmd is not None:
                    host_state = self._grasp_wait_waypoint_settle(
                        q_cmd=q_cmd,
                        host_state=host_state,
                        label="%s | motion gate" % str(wp_label),
                        settle_s=0.0,
                        settle_timeout_s=waypoint_settle_timeout_s,
                    )
                    if host_state is None:
                        self.state.set_pick_status(
                            running=False,
                            failed=True,
                            phase=ObjectPickPhase.FAILED.value,
                            msg="grasp | %s motion not settled before next wp"
                            % str(wp_label),
                        )
                        return

                centered_ok = False
                obs: Optional[VisualObservation] = None
                if self._grasp_visual_recover_supported():
                    centered_ok, obs, host_state = self._grasp_aim_recover_after_move(
                        cfg=grasp_cfg,
                        host_state=host_state,
                        label=wp_label,
                    )
                    if obs is None:
                        print(
                            "[Grasp] %s | aim recover | no observation (continue)"
                            % str(wp_label)
                        )
                else:
                    print(
                        "[Grasp] %s | aim recover skipped | mock/sim (sag+IK only)"
                        % str(wp_label)
                    )
                    if self.client is not None:
                        host_state = self.client.refresh_state()
                    obs = self.current_visual_observation(host_state)

                if self.client is not None:
                    host_state = self.client.refresh_state()
                self._grasp_update_online_sag_bias(
                    host_state=host_state,
                    object_world=live_object,
                    approach_dir=dir_u,
                    pk=pk,
                    label=wp_label,
                )
                sag_model = self._pick_grasp_sag_model()

                ok, q_cmd, host_state = self._grasp_advance_waypoint_ik(
                    tip_world=tip,
                    nominal_world=nominal_live,
                    object_world=tuple(float(v) for v in live_object),
                    approach_dir=dir_u,
                    step_m=step_m,
                    guided_handoff_m=guided_handoff_m,
                    sag_model=dict(sag_model),
                    host_state=host_state,
                    waypoint_idx=wp_idx,
                    apply_timeout_s=motion_apply_timeout_s,
                )
                if not ok:
                    self._set_pick_failure("grasp | %s axial IK failed" % str(wp_label))
                    return

                tip_after = self._pick_current_tip_world(host_state=host_state)
                if tip_after is not None:
                    executed_wp = GraspWaypoint(
                        position_world=tip_after,
                        direction_world=dir_tuple,
                        standoff_m=self._grasp_object_standoff_m(tip_after, live_object),
                        q_seed=(
                            tuple(float(v) for v in q_cmd.reshape(4))
                            if q_cmd is not None
                            else None
                        ),
                    )
                    self._grasp_executed_waypoints.append(executed_wp)

                self._send_grasp_trajectory_markers(
                    start_position=traj_start,
                    end_position=tuple(float(v) for v in nominal_live),
                    object_world=tuple(float(v) for v in live_object),
                    waypoints=list(self._grasp_executed_waypoints),
                    highlight_idx=int(len(self._grasp_executed_waypoints) - 1),
                    look_anchor_position=look_anchor,
                )

                if q_cmd is not None:
                    host_state = self._grasp_wait_waypoint_settle(
                        q_cmd=q_cmd,
                        host_state=host_state,
                        label=wp_label,
                        settle_s=waypoint_settle_s,
                        settle_timeout_s=waypoint_settle_timeout_s,
                    )
                    if host_state is None:
                        self._set_pick_failure("grasp | %s motion settle timeout" % str(wp_label))
                        return

                tip = self._pick_current_tip_world(host_state=host_state)
                if tip is None:
                    self._set_pick_failure("grasp guided | tip FK unavailable after settle")
                    return
                remain = self._grasp_axial_distance(tip, nominal_live, dir_u)
                if obs is not None:
                    u_d, v_d, _, _ = self._visual_uv_errors(obs)
                    uv_txt = "(%+.3f,%+.3f)" % (float(u_d), float(v_d))
                else:
                    uv_txt = "n/a"
                print(
                    "[Grasp] %s | remain=%.1fmm centered=%s uv=%s"
                    % (
                        str(wp_label),
                        float(remain) * 1000.0,
                        str(bool(centered_ok)).lower(),
                        str(uv_txt),
                    )
                )
                self.state.set_pick_status(
                    running=True,
                    failed=False,
                    phase=ObjectPickPhase.GRASP_APPROACH.value,
                    msg="grasp %s | remain=%.0fmm"
                    % (str(wp_label), float(remain) * 1000.0),
                )

            if wp_idx >= max_waypoints:
                tip_cap = self._pick_current_tip_world(host_state=host_state)
                remain_cap = (
                    self._grasp_axial_distance(tip_cap, nominal_world, dir_u)
                    if tip_cap is not None
                    else float("inf")
                )
                if remain_cap > guided_handoff_m + 1e-4:
                    print(
                        "[Grasp] waypoint cap %d | remain=%.1fmm (continue blind)"
                        % (int(max_waypoints), float(remain_cap) * 1000.0)
                    )

            tip_handoff_pre = self._pick_current_tip_world(host_state=host_state)
            try:
                live_object, dir_tuple_live = self._grasp_update_filtered_tracking(
                    tip_world=tip_handoff_pre,
                    pk=pk,
                )
            except RuntimeError:
                live_object = self._grasp_filtered_object_world() or object_world
                dir_live = self._grasp_filtered_approach_dir()
                if dir_live is None:
                    self._set_pick_failure("grasp | handoff filtered tracking unavailable")
                    return
                dir_tuple_live = dir_live
            dir_u = self._unit_vec3(dir_tuple_live)
            dir_tuple = (float(dir_u[0]), float(dir_u[1]), float(dir_u[2]))
            host_state = self.client.refresh_state() if self.client is not None else None
            nominal_live = self._pick_grasp_trajectory_end_position(
                live_object,
                dir_u,
                standoff_m=standoff_m,
            )
            tip_handoff = self._pick_current_tip_world(host_state=host_state)
            handoff_look: Optional[tuple[float, float, float]] = None
            if tip_handoff is not None:
                try:
                    look_v = self._grasp_look_at_dir(tip_handoff, live_object)
                    handoff_look = (
                        float(look_v[0]),
                        float(look_v[1]),
                        float(look_v[2]),
                    )
                    self._grasp_handoff_look_dir = handoff_look
                    print(
                        "[Grasp] handoff look latch | dir=(%.3f,%.3f,%.3f) obj=(%.3f,%.3f,%.3f)"
                        % (
                            handoff_look[0],
                            handoff_look[1],
                            handoff_look[2],
                            float(live_object[0]),
                            float(live_object[1]),
                            float(live_object[2]),
                        )
                    )
                except ValueError:
                    print("[Grasp] handoff look latch | failed (degenerate geometry)")

            if bool(pk.grasp_blind_uv_only):
                self._grasp_uv_only_mode = True
                print(
                    "[Grasp] handoff | uv-only perception (depth frozen, mask center active)"
                )
                if self._grasp_visual_recover_supported():
                    _, _, host_state = self._grasp_aim_recover_after_move(
                        cfg=grasp_cfg,
                        host_state=host_state,
                        label="handoff | uv center",
                    )
            else:
                self.stop_perception_capture(stop_recording=not bool(self.state.perception_recording))
                print("[Grasp] perception stopped | blind one-shot extend | recording kept=%s" % str(bool(self.state.perception_recording)).lower())

            if handoff_look is not None and host_state is not None:
                _, host_state = self._grasp_align_to_approach_dir(
                    approach_dir=np.asarray(handoff_look, dtype=float).reshape(3),
                    sag_model=dict(sag_model),
                    host_state=host_state,
                    label="grasp handoff | look-at align",
                )

            tip_after_loop = self._pick_current_tip_world(host_state=host_state)
            axial_remain = (
                self._grasp_axial_distance(tip_after_loop, nominal_live, dir_u)
                if tip_after_loop is not None
                else float("inf")
            )
            if axial_remain > reach_tol_m + 1e-4:
                if handoff_look is None:
                    self._set_pick_failure("grasp | blind extend missing handoff look direction")
                    return
                print(
                    "[Grasp] blind extend | axial_remain=%.1fmm > tol %.1fmm"
                    % (float(axial_remain) * 1000.0, float(reach_tol_m) * 1000.0)
                )
                blind_ok, q_cmd, host_state, _ = self._grasp_blind_final_approach(
                    object_world=tuple(float(v) for v in live_object),
                    look_dir=handoff_look,
                    sag_model=dict(sag_model),
                    host_state=host_state,
                    grasp_standoff_m=standoff_m,
                    approach_dir=dir_u,
                    nominal_world=tuple(float(v) for v in nominal_live),
                )
                if not blind_ok or q_cmd is None:
                    self._set_pick_failure("grasp | blind extend failed")
                    return

            if q_cmd is None:
                self._set_pick_failure("grasp | no commanded q")
                return

            object_tuple = tuple(float(v) for v in live_object)
            target_arr = np.asarray(nominal_live, dtype=float).reshape(3)
            actual_offset_m = float(
                np.linalg.norm(np.asarray(object_tuple, dtype=float).reshape(3) - target_arr)
            )
            self.send_grasp_meta(source="target")
            self._send_grasp_target_markers(
                object_world=object_tuple,
                target=target_arr,
                direction=dir_u,
                actual_offset_m=actual_offset_m,
                corrected=bool(self._pick_grasp_uses_equal_sag()),
            )

            closed_ok, claw_suffix = self._close_gripper_after_grasp_arrival(
                host_state=host_state,
                q_cmd=q_cmd,
                target_world=target_arr,
                sag_model=dict(sag_model),
                label="grasp pre-contact",
                nominal_world=tuple(float(v) for v in nominal_live),
                approach_dir=dir_u,
            )
            if not bool(closed_ok):
                return

            done_msg = "grasp done | waypoints=%d | %s" % (
                int(self._grasp_waypoint_idx),
                str(claw_suffix),
            )
            self.state.set_pick_status(
                running=False,
                failed=False,
                phase=ObjectPickPhase.DONE.value,
                msg=done_msg,
            )
            self.state.set_ik_status(
                running=False,
                converged=True,
                failed=False,
                err_m=0.0,
                msg=done_msg,
            )
            success = True
            print("[Grasp] %s" % done_msg)
        finally:
            self._grasp_uv_only_mode = False
            cancelled = bool(self._pick_stop_event.is_set() or self._pick_e2e_cancel.is_set())
            if (
                not cancelled
                and self._perception_capture is not None
                and self._perception_capture.is_running()
            ):
                self.stop_perception_capture(stop_recording=not bool(self.state.perception_recording))
            if not success and not self.state.pick_failed and not cancelled:
                self._set_pick_failure("grasp failed")
            self._ik_worker = None


class GraspActions(GraspGuidedActions):
    """LJI approach loop exposed to the pilot service."""

    def _run_grasp_lji_approach_worker(
        self,
        *,
        object_world: tuple[float, float, float],
        approach_dir: np.ndarray,
        nominal_world: tuple[float, float, float],
    ) -> None:
        """LJI loop until remain <= blind_micro_start_m; then one-shot blind axial."""
        pk = self._pick_config_effective()
        max_waypoints = max(1, int(pk.grasp_max_waypoints))
        standoff_m = float(max(pk.grasp_standoff_m, 0.0))
        close_tol_m = float(max(pk.grasp_close_tol_m, float(self._ik_cfg.tol), 0.003))
        lji_settle_dwell_s = float(max(pk.lij_settle_dwell_s, 0.0))
        lji_motion_settle_timeout_s = float(max(pk.lij_settle_timeout_s, 0.0))
        lji_settle_angle_tol = max(
            7.5e-4,
            min(0.0015, float(pk.lij_max_dq_angle) * 0.25),
        )
        lji_settle_linear_tol = max(
            2.0e-4,
            min(6.0e-4, float(pk.lij_max_dq_linear) * 0.25),
        )
        lji_apply_timeout_s = max(float(lji_motion_settle_timeout_s), 0.05)
        lji_host_native = self._host_native_lji_runtime()
        lji_pipelined = bool(pk.lij_pipelined_motion or self._use_hardware or lji_host_native)
        lji_step_period_s = float(max(pk.lij_step_period_s, 0.0))
        lji_obs_wait_s = (
            min(0.08, max(lji_step_period_s, 0.04))
            if bool(lji_pipelined)
            else 0.25
        )
        success = False
        traj_start = self._grasp_traj_start
        look_anchor = self._grasp_look_anchor
        servo = self._grasp_lji_servo_3d
        if traj_start is None or servo is None:
            self._set_pick_failure("grasp lji | init missing")
            return
        try:
            if self._perception_capture is None or not self._perception_capture.is_running():
                self._maybe_start_local_perception()

            host_state = self._refresh_lji_state()
            self._grasp_lji_command_q = self._q_array_from_state(host_state).copy()
            q_cmd: Optional[np.ndarray] = None
            sag_model = self._grasp_lji_sag_model()
            mode = GraspApproachMode.LOCAL_IMG_JACOBIAN
            self._grasp_approach_mode = mode
            prev_mode = mode
            print(
                "[Grasp] LJI3D start | close_tol=%.1fmm blind_at_remain=%.0fmm "
                "gain_z=%.2f z_bend=%.2f settle_tol=(%.4fm,%.4frad) "
                "pipelined=%s runtime=%s"
                % (
                    close_tol_m * 1000.0,
                    float(pk.blind_micro_start_m) * 1000.0,
                    float(pk.lij_gain_z),
                    float(pk.lij_z_bend_gain),
                    float(lji_settle_linear_tol),
                    float(lji_settle_angle_tol),
                    str(bool(lji_pipelined)).lower(),
                    "host_native" if bool(lji_host_native) else "client",
                )
            )
            if bool(self._use_hardware) and not bool(pk.lij_pipelined_motion):
                print("[Grasp] LJI3D hardware mode | forcing pipelined motion steps")
            self._grasp_lji_log_start()
            self._grasp_lji_log_event(
                step_idx=0,
                mode=mode,
                note="start",
                close_tol_m=close_tol_m,
            )

            wp_idx = 0
            while wp_idx < max_waypoints:
                step_t0 = time.time()
                if self._pick_stop_event.is_set():
                    self._grasp_lji_log_event(
                        step_idx=int(wp_idx),
                        mode=mode,
                        note="stop_event",
                        close_tol_m=close_tol_m,
                    )
                    self.state.set_pick_status(
                        running=False,
                        failed=False,
                        phase=ObjectPickPhase.IDLE.value,
                        msg="grasp stopped",
                    )
                    return

                tip = self._pick_current_tip_world(host_state=host_state)
                if tip is None:
                    self._grasp_lji_log_event(
                        step_idx=int(wp_idx),
                        mode=mode,
                        note="tip_fk_unavailable",
                        close_tol_m=close_tol_m,
                        step_elapsed_s=time.time() - float(step_t0),
                    )
                    self._set_pick_failure("grasp lji | tip FK unavailable")
                    return

                try:
                    live_object, dir_tuple_live = self._grasp_update_filtered_tracking(
                        tip_world=tip,
                        pk=pk,
                    )
                except RuntimeError:
                    live_object = self._grasp_filtered_object_world() or object_world
                    dir_live = self._grasp_filtered_approach_dir()
                    if dir_live is None:
                        self._grasp_lji_log_event(
                            step_idx=int(wp_idx),
                            mode=mode,
                            note="filtered_tracking_unavailable",
                            close_tol_m=close_tol_m,
                            step_elapsed_s=time.time() - float(step_t0),
                        )
                        self._set_pick_failure("grasp lji | filtered tracking unavailable")
                        return
                    dir_tuple_live = dir_live
                dir_u = self._unit_vec3(dir_tuple_live)
                nominal_live = self._pick_grasp_trajectory_end_position(
                    live_object,
                    dir_u,
                    standoff_m=standoff_m,
                )
                remain = self._grasp_axial_distance(tip, nominal_live, dir_u)
                remain_outlier = self._grasp_lji_remain_outlier_reason(
                    float(remain),
                    pk=pk,
                )
                if remain_outlier and self._grasp_lji_last_reliable_object_world is not None:
                    live_object = tuple(float(v) for v in self._grasp_lji_last_reliable_object_world)
                    if self._grasp_lji_last_reliable_approach_dir is not None:
                        dir_u = self._unit_vec3(self._grasp_lji_last_reliable_approach_dir)
                    nominal_live = self._pick_grasp_trajectory_end_position(
                        live_object,
                        dir_u,
                        standoff_m=standoff_m,
                    )
                    remain = self._grasp_axial_distance(tip, nominal_live, dir_u)
                    self._grasp_object_world_filtered = live_object
                    self._grasp_approach_dir_filtered = (
                        float(dir_u[0]),
                        float(dir_u[1]),
                        float(dir_u[2]),
                    )
                depth_valid, _ = self._grasp_lji_depth_snapshot(
                    remain_m=float(remain),
                    tip_world=tip,
                    object_world=tuple(float(v) for v in live_object),
                    approach_dir=dir_u,
                )
                hist = list(self._grasp_depth_history)
                depth_valid_ratio = (
                    float(sum(1 for dv, _, _ in hist if dv)) / float(len(hist))
                    if hist
                    else 0.0
                )
                depth_stable, depth_reason = self._grasp_lji_eval_depth_stability(
                    pk,
                    remain_m=float(remain),
                )
                depth_reliable = bool(depth_valid and depth_stable)

                if float(remain) <= close_tol_m + 1e-4:
                    self._grasp_lji_log_event(
                        step_idx=int(wp_idx),
                        mode=mode,
                        note="precontact_break",
                        remain_m=float(remain),
                        close_tol_m=close_tol_m,
                        step_elapsed_s=time.time() - float(step_t0),
                    )
                    print(
                        "[Grasp] LJI | precontact | remain=%.1fmm <= close_tol %.1fmm"
                        % (float(remain) * 1000.0, close_tol_m * 1000.0)
                    )
                    break

                obs, host_state = self._grasp_lji_wait_visual_observation(
                    host_state,
                    wait_s=lji_obs_wait_s,
                )
                object_lost = bool(obs is None or remain_outlier)
                lost_reason = str(remain_outlier) if remain_outlier else ("no_observation" if object_lost else "")
                obs_outlier = "" if remain_outlier else self._grasp_lji_observation_outlier(obs, host_state, pk=pk)
                force_reacquire_reason = str(self._grasp_lji_force_reacquire_reason or "")
                if force_reacquire_reason:
                    object_lost = True
                    lost_reason = force_reacquire_reason
                    obs = None
                    self._grasp_lji_force_reacquire_reason = ""
                    est_f = self._grasp_lji_estimator_3d
                    if est_f is not None:
                        est_f.clear()
                if obs_outlier:
                    object_lost = True
                    lost_reason = str(obs_outlier)
                    obs = None
                    est_o = self._grasp_lji_estimator_3d
                    if est_o is not None:
                        est_o.clear()
                elif remain_outlier:
                    obs = None
                    est_o = self._grasp_lji_estimator_3d
                    if est_o is not None:
                        est_o.clear()
                s_lji_now = self._grasp_lji_build_features_3d(obs, remain_m=float(remain))
                if s_lji_now is not None:
                    self._grasp_lji_v_err_hist.append(abs(float(s_lji_now[1])))
                    if len(self._grasp_lji_v_err_hist) > 8:
                        self._grasp_lji_v_err_hist = self._grasp_lji_v_err_hist[-8:]
                visual_lost = self._grasp_lji_visual_tracking_lost(s_lji_now, pk=pk)
                if visual_lost and not object_lost:
                    est_v = self._grasp_lji_estimator_3d
                    if est_v is not None:
                        est_v.clear()
                if object_lost:
                    self._grasp_lji_object_lost_count += 1
                else:
                    self._grasp_lji_object_lost_count = 0
                    self._record_pick_last_seen_uv(obs)
                    if int(self._grasp_lji_reacquire_steps) > 0:
                        self._grasp_lji_end_reacquire()

                if (
                    object_lost
                    and int(self._grasp_lji_reacquire_steps)
                    >= int(pk.lij_reacquire_max_steps)
                    and not self._grasp_lji_should_blind_finish(float(remain), pk)
                ):
                    self._grasp_lji_log_event(
                        step_idx=int(wp_idx),
                        mode=mode,
                        note="tracking_lost_after_reacquire",
                        remain_m=float(remain),
                        close_tol_m=close_tol_m,
                        object_lost=int(self._grasp_lji_object_lost_count),
                        step_elapsed_s=time.time() - float(step_t0),
                    )
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg=(
                            "grasp lji | tracking lost after reacquire "
                            "(remain=%.0fmm)"
                            % (float(remain) * 1000.0)
                        ),
                    )
                    return

                transition = "-"
                if self._grasp_lji_should_blind_finish(float(remain), pk):
                    self._grasp_lji_log_event(
                        step_idx=int(wp_idx),
                        mode=mode,
                        note="blind_finish_break",
                        remain_m=float(remain),
                        close_tol_m=close_tol_m,
                        object_lost=int(self._grasp_lji_object_lost_count),
                        step_elapsed_s=time.time() - float(step_t0),
                    )
                    print(
                        "[Grasp] LJI | blind finish | remain=%.1fmm <= %.1fmm"
                        % (
                            float(remain) * 1000.0,
                            float(pk.blind_micro_start_m) * 1000.0,
                        )
                    )
                    break
                if self._grasp_lji_should_reacquire(
                    object_lost=bool(object_lost),
                    remain_m=float(remain),
                    close_tol_m=close_tol_m,
                    pk=pk,
                ):
                    if mode != GraspApproachMode.REACQUIRE:
                        transition = "%s|reacquire" % (str(lost_reason or "object_lost"))
                    self._grasp_lji_begin_reacquire(
                        prev_mode=mode,
                        remain_m=float(remain),
                    )
                    mode = GraspApproachMode.REACQUIRE
                else:
                    mode = GraspApproachMode.LOCAL_IMG_JACOBIAN
                    if not object_lost:
                        self._grasp_lji_latch_reliable_state(
                            object_world=tuple(float(v) for v in live_object),
                            approach_dir=dir_u,
                            remain_m=float(remain),
                            host_state=host_state,
                        )

                if mode != prev_mode and transition != "-":
                    self._grasp_lji_last_transition = transition
                prev_mode = mode
                self._grasp_approach_mode = mode

                wp_idx += 1
                self._grasp_waypoint_idx = int(wp_idx)
                wp_label = "lji %d" % int(wp_idx)
                controller_tag = "local_img_jacobian"
                ik_status = "-"
                dq_cmd_arr = np.zeros(4, dtype=float)
                dq_apply_arr = np.zeros(4, dtype=float)
                j_rank = 0
                j_cond = float("inf")
                j_available = False
                sample_reason = SampleRejectReason.DQ_TOO_SMALL
                settle_ok = True
                s_lji: Optional[np.ndarray] = None
                remain_after = float(remain)
                dq_meas: Optional[np.ndarray] = None

                if mode == GraspApproachMode.REACQUIRE:
                    controller_tag = "reacquire"
                    q_before = self._q_array_from_state(host_state)
                    q_cmd = None
                    moved = False
                    self._grasp_lji_reacquire_steps += 1
                    step = int(self._grasp_lji_reacquire_steps)
                    if step == 1:
                        cap = self._perception_capture
                        if cap is not None:
                            cap.request_refresh()
                    aim_at = max(1, int(pk.lij_reacquire_aim_after_steps))
                    recovered = False
                    if step >= aim_at and not bool(self._grasp_lji_reacquire_aim_tried):
                        self._grasp_lji_reacquire_aim_tried = True
                        _ok_r, _obs_r, host_state = self._grasp_lji_try_reacquire(
                            grasp_cfg=pk,
                            host_state=host_state,
                            pk=pk,
                        )
                        if _ok_r and _obs_r is not None:
                            self._grasp_lji_object_lost_count = 0
                            self._grasp_lji_end_reacquire()
                            self._record_pick_last_seen_uv(_obs_r)
                            recovered = True
                            moved = True
                    if not recovered:
                        # Reacquire must stay low-latency: avoid solving a fresh IK
                        # waypoint while visual tracking is already missing.
                        dq_back = self._grasp_lji_retract_dq_to_last_good_q(
                            q_before=q_before,
                            pk=pk,
                        )
                        if dq_back is not None:
                            dq_cmd_arr = np.asarray(dq_back, dtype=float).reshape(4)
                            dq_apply_arr = dq_cmd_arr.copy()
                            q_cmd, host_state = self._grasp_apply_q_delta(
                                dq_apply_arr,
                                host_state=host_state,
                                sag_model=dict(sag_model),
                                timeout_s=lji_apply_timeout_s,
                                wait_settle=not lji_pipelined,
                                step_period_s=lji_step_period_s,
                                linear_tol_m=lji_settle_linear_tol,
                                angle_tol_rad=lji_settle_angle_tol,
                            )
                            moved = True
                        if self._pick_apply_lost_follow_step(
                            reason="grasp_lji_fov",
                            allow_refresh=False,
                        ):
                            host_state = self._refresh_lji_state() or host_state
                            moved = True
                    if moved and q_cmd is not None:
                        host_state = self._grasp_lji_refresh_after_step(
                            q_cmd=q_cmd,
                            host_state=host_state,
                            label=wp_label,
                            dwell_s=lji_settle_dwell_s,
                            settle_timeout_s=lji_motion_settle_timeout_s,
                            linear_tol_m=lji_settle_linear_tol,
                            angle_tol_rad=lji_settle_angle_tol,
                        )
                        settle_ok = host_state is not None
                    obs_after, host_state = self._grasp_lji_wait_visual_observation(
                        host_state,
                        wait_s=lji_obs_wait_s,
                    )
                    obs_after_outlier = self._grasp_lji_observation_outlier(obs_after, host_state, pk=pk)
                    s_after_reacquire = self._grasp_lji_build_features_3d(
                        obs_after, remain_m=float(remain_after)
                    )
                    if (
                        obs_after is not None
                        and not obs_after_outlier
                        and not self._grasp_lji_visual_tracking_lost(s_after_reacquire, pk=pk)
                    ):
                        self._grasp_lji_object_lost_count = 0
                        self._grasp_lji_end_reacquire()
                        self._record_pick_last_seen_uv(obs_after)
                    elif obs_after_outlier:
                        obs_after = None
                        s_after_reacquire = None
                    tip_after = self._pick_current_tip_world(host_state=host_state)
                    if tip_after is not None:
                        remain_after = float(
                            self._grasp_axial_distance(tip_after, nominal_live, dir_u)
                        )
                    self._grasp_lji_reacquire_prev_remain = float(remain_after)
                    s_lji = s_after_reacquire
                    if moved and q_cmd is not None:
                        dq_meas = np.asarray(q_cmd, dtype=float) - np.asarray(
                            q_before, dtype=float
                        )
                    self._grasp_lji_log_control_step(
                        step_idx=int(wp_idx),
                        mode=mode,
                        s_lji=s_lji,
                        depth_valid=depth_valid,
                        depth_valid_ratio=depth_valid_ratio,
                        j_rank=0,
                        j_cond=float("inf"),
                        j_available=False,
                        dq_cmd=dq_apply_arr,
                        dq_meas=dq_meas,
                        q_cmd=q_cmd if q_cmd is not None else q_before,
                        controller=controller_tag,
                        transition=transition,
                        object_lost=int(self._grasp_lji_object_lost_count),
                        remain_m=float(remain_after),
                        remain_before_m=float(remain),
                        step_elapsed_s=time.time() - float(step_t0),
                        close_tol_m=close_tol_m,
                        ik_status="-",
                        sample_reason="n/a",
                        note="reacquire",
                    )
                    self.state.set_pick_status(
                        running=True,
                        failed=False,
                        phase=ObjectPickPhase.GRASP_APPROACH.value,
                        msg="grasp %s | remain=%.0fmm mode=%s reacquire=%d"
                        % (
                            str(wp_label),
                            float(remain_after) * 1000.0,
                            str(mode.value),
                            int(self._grasp_lji_reacquire_steps),
                        ),
                    )
                    continue

                s_lji = self._grasp_lji_build_features_3d(obs, remain_m=float(remain))
                if s_lji is None:
                    if bool(pk.lij_probing_enabled):
                        controller_tag = "probing"
                        eps_l = float(pk.lij_probing_epsilon_linear)
                        eps_a = float(pk.lij_probing_epsilon_angle)
                        probe = np.array([eps_l, eps_a, eps_a, eps_a], dtype=float)
                        q_measured_before = self._q_array_from_state(host_state)
                        q_before = self._grasp_lji_command_base_q(host_state)
                        dq_apply_arr = probe.copy()
                        if float(pk.lij_dq_smooth_alpha) > 1e-6:
                            probe = self._grasp_lji_smooth_dq(probe, pk=pk)
                            dq_apply_arr = probe.copy()
                        self._grasp_lji_pending_sample = {
                            "q_before": q_measured_before.copy(),
                            "s_before": np.zeros(3, dtype=float),
                            "dq_cmd": dq_apply_arr.copy(),
                        }
                        q_cmd, host_state = self._grasp_apply_q_delta(
                            dq_apply_arr,
                            host_state=host_state,
                            sag_model=dict(sag_model),
                            timeout_s=lji_apply_timeout_s,
                            wait_settle=not lji_pipelined,
                            step_period_s=lji_step_period_s,
                            linear_tol_m=lji_settle_linear_tol,
                            angle_tol_rad=lji_settle_angle_tol,
                            motion_wait_frac=0.0 if bool(lji_host_native) else 0.15,
                        )
                        if float(pk.lij_dq_smooth_alpha) > 1e-6:
                            self._grasp_lji_last_dq_cmd = probe.copy()
                    else:
                        self._grasp_lji_log_control_step(
                            step_idx=int(wp_idx),
                            mode=mode,
                            s_lji=None,
                            depth_valid=depth_valid,
                            depth_valid_ratio=depth_valid_ratio,
                            j_rank=0,
                            j_cond=float("inf"),
                            j_available=False,
                            dq_cmd=np.zeros(4, dtype=float),
                            dq_meas=None,
                            q_cmd=self._q_array_from_state(host_state),
                            controller="no_observation",
                            transition=transition,
                            object_lost=int(self._grasp_lji_object_lost_count),
                            remain_m=float(remain),
                            remain_before_m=float(remain),
                            step_elapsed_s=time.time() - float(step_t0),
                            close_tol_m=close_tol_m,
                            ik_status="-",
                            sample_reason="no_observation",
                            note="s_lji_none",
                        )
                        if lji_step_period_s > 1e-6:
                            time.sleep(min(float(lji_step_period_s), 0.10))
                        continue
                else:
                    q_before = self._grasp_lji_command_base_q(host_state)
                    (
                        dq_cmd_arr,
                        _dq_raw,
                        _j,
                        j_rank,
                        j_cond,
                        j_available,
                        controller_tag,
                    ) = self._grasp_lji_compute_step_dq(
                        servo,
                        np.asarray(s_lji, dtype=float).reshape(3),
                        q=q_before,
                        approach_dir=dir_u,
                        sag_model=dict(sag_model),
                        remain_m=float(remain),
                        pk=pk,
                        close_tol_m=close_tol_m,
                    )
                    if not j_available and bool(pk.lij_probing_enabled):
                        controller_tag = "probing"
                        eps_l = float(pk.lij_probing_epsilon_linear)
                        eps_a = float(pk.lij_probing_epsilon_angle)
                        dq_cmd_arr = np.array([eps_l, eps_a, eps_a, eps_a], dtype=float)
                    if not j_available:
                        est = self._grasp_lji_estimator_3d
                        n_samp = int(est.sample_count()) if est is not None else 0
                        print(
                            "[Grasp] %s | J3d seed/fk | samples=%d need>=%d rank=%d cond=%.1f"
                            % (
                                str(wp_label),
                                n_samp,
                                int(pk.lij_min_samples),
                                int(j_rank),
                                float(j_cond),
                            )
                        )
                    q_measured_before = self._q_array_from_state(host_state)
                    dq_cmd_arr = self._grasp_lji_guard_dq_at_limits(
                        q_before,
                        dq_cmd_arr,
                        pk=pk,
                    )
                    if float(pk.lij_dq_smooth_alpha) > 1e-6:
                        dq_cmd_arr = self._grasp_lji_smooth_dq(dq_cmd_arr, pk=pk)
                    dq_apply_arr = self._grasp_lji_apply_command_horizon(
                        dq_cmd_arr,
                        q_before=q_before,
                        pk=pk,
                    )
                    command_horizon = self._grasp_lji_command_horizon(pk)
                    self._grasp_lji_pending_sample = {
                        "q_before": q_measured_before.copy(),
                        "s_before": s_lji.copy(),
                        "dq_cmd": dq_apply_arr.copy(),
                    }
                    q_cmd, host_state = self._grasp_apply_q_delta(
                        dq_apply_arr,
                        host_state=host_state,
                        sag_model=dict(sag_model),
                        timeout_s=lji_apply_timeout_s,
                        wait_settle=not lji_pipelined,
                        step_period_s=lji_step_period_s,
                        linear_tol_m=lji_settle_linear_tol,
                        angle_tol_rad=lji_settle_angle_tol,
                        motion_wait_frac=(
                            0.0
                            if bool(lji_host_native)
                            else max(0.03, 0.15 / float(command_horizon))
                        ),
                    )
                    if float(pk.lij_dq_smooth_alpha) > 1e-6:
                        self._grasp_lji_last_dq_cmd = np.asarray(
                            dq_cmd_arr, dtype=float
                        ).reshape(4).copy()

                if q_cmd is not None and not lji_pipelined:
                    host_state = self._grasp_lji_refresh_after_step(
                        q_cmd=q_cmd,
                        host_state=host_state,
                        label=wp_label,
                        dwell_s=lji_settle_dwell_s,
                        settle_timeout_s=lji_motion_settle_timeout_s,
                        linear_tol_m=lji_settle_linear_tol,
                        angle_tol_rad=lji_settle_angle_tol,
                    )
                    settle_ok = host_state is not None
                elif q_cmd is not None:
                    settle_ok = True

                if bool(lji_host_native) and lji_step_period_s > 1e-6:
                    tick_remaining_s = float(lji_step_period_s) - (
                        time.time() - float(step_t0)
                    )
                    if tick_remaining_s > 1e-4:
                        time.sleep(min(float(tick_remaining_s), 0.08))

                obs_after, host_state = self._grasp_lji_wait_visual_observation(
                    host_state,
                    wait_s=lji_obs_wait_s,
                )
                tip_after = self._pick_current_tip_world(host_state=host_state)
                remain_after = (
                    self._grasp_axial_distance(
                        tip_after,
                        nominal_live,
                        dir_u,
                    )
                    if tip_after is not None
                    else float(remain)
                )
                s_after = self._grasp_lji_build_features_3d(
                    obs_after, remain_m=float(remain_after)
                )
                dq_meas = None
                pending = self._grasp_lji_pending_sample
                if pending is not None and q_cmd is not None:
                    pending["q_after"] = self._q_array_from_state(host_state).copy()
                    pending["s_after"] = (
                        s_after.copy() if s_after is not None else pending["s_before"].copy()
                    )
                    q_before_m = np.asarray(pending["q_before"], dtype=float)
                    dq_meas = np.asarray(pending["q_after"], dtype=float) - q_before_m
                    sample_reason = self._grasp_lji_record_measured_sample(
                        pk=pk,
                        settle_ok=bool(settle_ok),
                        object_lost=bool(obs_after is None),
                        pipelined=bool(lji_pipelined),
                    )
                    q_after = self._q_array_from_state(host_state)
                    stall_msg = self._grasp_lji_update_stall_watch(
                        pk=pk,
                        remain_m=float(remain_after),
                        sample_reason=sample_reason,
                        q=q_after,
                        dq_meas=dq_meas,
                    )
                    if sample_reason == SampleRejectReason.JOINT_SATURATED:
                        est = self._grasp_lji_estimator_3d
                        if est is not None and self._grasp_lji_sat_streak >= 3:
                            est.clear()
                    bad_motion_reason = self._grasp_lji_update_bad_motion_watch(
                        pk=pk,
                        sample_reason=sample_reason,
                    )
                    if bad_motion_reason:
                        est_b = self._grasp_lji_estimator_3d
                        if est_b is not None:
                            est_b.clear()
                        self._grasp_lji_last_dq_cmd = None
                        self._grasp_lji_command_q = None
                        self._grasp_lji_force_reacquire_reason = bad_motion_reason
                        self._stop_lji_velocity_control(
                            "lji_%s" % bad_motion_reason
                        )
                        transition = "%s|reacquire" % bad_motion_reason
                        print(
                            "[Grasp] LJI bad motion | reason=%s sample=%s -> reacquire"
                            % (bad_motion_reason, str(sample_reason.value))
                        )
                    if stall_msg is not None:
                        print("[Grasp] %s" % stall_msg)
                        self._set_pick_failure(stall_msg)
                        return

                self._grasp_lji_log_control_step(
                    step_idx=int(wp_idx),
                    mode=mode,
                    s_lji=s_lji,
                    depth_valid=depth_valid,
                    depth_valid_ratio=depth_valid_ratio,
                    j_rank=int(j_rank),
                    j_cond=float(j_cond),
                    j_available=bool(j_available),
                    dq_cmd=dq_apply_arr,
                    dq_meas=dq_meas,
                    q_cmd=q_cmd if q_cmd is not None else self._q_array_from_state(host_state),
                    controller=controller_tag,
                    transition=transition,
                    object_lost=int(self._grasp_lji_object_lost_count),
                    remain_m=float(remain_after),
                    remain_before_m=float(remain),
                    step_elapsed_s=time.time() - float(step_t0),
                    close_tol_m=close_tol_m,
                    ik_status=ik_status,
                    sample_reason=str(sample_reason.value),
                    note="lji_step",
                )
                self.state.set_pick_status(
                    running=True,
                    failed=False,
                    phase=ObjectPickPhase.GRASP_APPROACH.value,
                    msg="grasp %s | remain=%.0fmm mode=%s"
                    % (str(wp_label), float(remain) * 1000.0, str(mode.value)),
                )

            if q_cmd is None:
                host_state = self._refresh_lji_state()
                q_cmd = self._q_array_from_state(host_state)

            tip_pre_blind = self._pick_current_tip_world(host_state=host_state)
            use_obj_blind = self._grasp_lji_last_reliable_object_world or tuple(
                float(v) for v in object_world
            )
            use_dir_blind = self._grasp_lji_last_reliable_approach_dir
            dir_blind = (
                self._unit_vec3(use_dir_blind)
                if use_dir_blind is not None
                else self._unit_vec3(approach_dir)
            )
            nominal_blind = self._pick_grasp_trajectory_end_position(
                use_obj_blind,
                dir_blind,
                standoff_m=standoff_m,
            )
            remain_pre_blind = (
                self._grasp_axial_distance(tip_pre_blind, nominal_blind, dir_blind)
                if tip_pre_blind is not None
                else float("inf")
            )
            if float(remain_pre_blind) <= float(close_tol_m) + 1e-4:
                pass
            elif self._grasp_lji_should_blind_finish(float(remain_pre_blind), pk):
                blind_ok, q_blind, host_state = self._grasp_lji_blind_finish_if_needed(
                    object_world=object_world,
                    approach_dir=approach_dir,
                    nominal_world=nominal_world,
                    host_state=host_state,
                    sag_model=dict(sag_model),
                    standoff_m=standoff_m,
                    close_tol_m=close_tol_m,
                )
                if not blind_ok or q_blind is None:
                    self._set_pick_failure("grasp lji | blind finish failed")
                    return
                q_cmd = q_blind
            else:
                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg=(
                        "grasp lji | remain=%.0fmm > blind_at %.0fmm"
                        % (
                            float(remain_pre_blind) * 1000.0,
                            float(pk.blind_micro_start_m) * 1000.0,
                        )
                    ),
                )
                return

            tip_final = self._pick_current_tip_world(host_state=host_state)
            try:
                live_object, dir_tuple_live = self._grasp_update_filtered_tracking(
                    tip_world=tip_final,
                    pk=pk,
                )
            except RuntimeError:
                live_object = self._grasp_filtered_object_world() or object_world
                dir_live = self._grasp_filtered_approach_dir()
                if dir_live is None:
                    self._set_pick_failure("grasp lji | final tracking unavailable")
                    return
                dir_tuple_live = dir_live
            dir_u = self._unit_vec3(dir_tuple_live)
            nominal_live = self._pick_grasp_trajectory_end_position(
                live_object,
                dir_u,
                standoff_m=standoff_m,
            )
            success = self._grasp_complete_precontact_and_close(
                live_object=tuple(float(v) for v in live_object),
                nominal_live=tuple(float(v) for v in nominal_live),
                dir_u=dir_u,
                q_cmd=q_cmd,
                host_state=host_state,
                sag_model=dict(sag_model),
                waypoint_count=int(self._grasp_waypoint_idx),
                claw_label="grasp lji pre-contact",
            )
        finally:
            try:
                self._stop_lji_velocity_control("grasp_lji_done")
            finally:
                self._grasp_lji_log_close()
                self._grasp_uv_only_mode = False
                cancelled = bool(
                    self._pick_stop_event.is_set()
                    or self._pick_e2e_cancel.is_set()
                )
                if (
                    not cancelled
                    and self._perception_capture is not None
                    and self._perception_capture.is_running()
                ):
                    self.stop_perception_capture(
                        stop_recording=not bool(self.state.perception_recording)
                    )
                if not success and not self.state.pick_failed and not cancelled:
                    self._set_pick_failure("grasp lji failed")
                self._ik_worker = None
