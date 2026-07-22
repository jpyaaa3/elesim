"""Look and ready-pose workflow methods for ControlService."""
from __future__ import annotations
from ._deps import *  # noqa: F401,F403
from elesim_controller.observability.tracing import traced_thread_target

class ReadyGeometryActions:
    def _pick_frozen_world(self) -> Optional[tuple[float, float, float]]:
        frozen = self._pick_frozen_world_xyz
        if frozen is not None:
            return tuple(frozen)
        if self.state.perception_world_xyz is not None:
            return tuple(self.state.perception_world_xyz)
        if self.client is not None and self.client.last_object_world_xyz is not None:
            return tuple(self.client.last_object_world_xyz)
        return None

    def _pick_latest_object_world(self) -> Optional[tuple[float, float, float]]:
        if self.client is not None and self.client.last_object_world_xyz is not None:
            return tuple(self.client.last_object_world_xyz)
        if self.state.perception_world_xyz is not None:
            return tuple(self.state.perception_world_xyz)
        return None

    def _pick_grasp_object_world(self) -> Optional[tuple[float, float, float]]:
        """Target object for Grasp: Aim-centered > Look-latched > live perception."""
        for candidate in (
            self._pick_centered_object_world_xyz,
            self._pick_look_object_world_xyz,
            self._pick_latest_object_world(),
            self._pick_frozen_world(),
            self._pick_initial_object_world_xyz,
        ):
            if candidate is not None:
                return tuple(float(v) for v in candidate)
        return None

    def _pick_grasp_sag_model(self) -> dict[str, Any]:
        if isinstance(self._grasp_online_sag_model, dict) and self._grasp_online_sag_model:
            return dict(self._grasp_online_sag_model)
        if isinstance(self._pick_equal_sag_model, dict) and self._pick_equal_sag_model:
            return dict(self._pick_equal_sag_model)
        if isinstance(self.state.raw_sag_model, dict):
            return dict(self.state.raw_sag_model)
        return {}

    def _pick_grasp_uses_equal_sag(self) -> bool:
        if isinstance(self._grasp_online_sag_model, dict) and self._grasp_online_sag_model:
            return True
        return isinstance(self._pick_equal_sag_model, dict) and bool(self._pick_equal_sag_model)

    def _pick_current_tip_world(
        self, *, host_state: Optional[HostState] = None
    ) -> Optional[tuple[float, float, float]]:
        try:
            model = self._pick_reach_model(self._pick_grasp_sag_model())
            if host_state is None and self.client is not None:
                host_state = self.client.refresh_state()
            q0 = self._q_array_from_state(host_state)
            tip = np.asarray(model.grasp_position(q0), dtype=float).reshape(3)
            return (float(tip[0]), float(tip[1]), float(tip[2]))
        except Exception:
            return None

    def _pick_auto_preferred_dir(
        self,
        object_world: tuple[float, float, float],
        *,
        tip_world: Optional[tuple[float, float, float]] = None,
    ) -> Optional[tuple[float, float, float]]:
        if self._pick_resolved_ready_dir_world is not None:
            return self._pick_resolved_ready_dir_world
        if self._pick_look_dir_world is not None:
            return self._pick_look_dir_world
        tip = tip_world or self._pick_look_tip_world_xyz
        if tip is None:
            tip = self._pick_current_tip_world()
        if tip is None:
            return None
        look_vec = np.asarray(object_world, dtype=float).reshape(3) - np.asarray(tip, dtype=float).reshape(3)
        look_len = float(np.linalg.norm(look_vec))
        if look_len <= 1e-6:
            return None
        unit = look_vec / look_len
        return (float(unit[0]), float(unit[1]), float(unit[2]))

    def _pick_ready_direction(
        self,
        *,
        object_world: Optional[tuple[float, float, float]] = None,
        tip_world: Optional[tuple[float, float, float]] = None,
        prefer_current_tip: bool = False,
    ) -> Optional[tuple[float, float, float]]:
        if prefer_current_tip:
            tip = tip_world or self._pick_current_tip_world()
            obj = (
                object_world
                or self._pick_centered_object_world_xyz
                or self._pick_latest_object_world()
                or self._pick_frozen_world()
                or self._pick_look_object_world_xyz
                or self._pick_initial_object_world_xyz
            )
            if tip is not None and obj is not None:
                look_vec = (
                    np.asarray(obj, dtype=float).reshape(3)
                    - np.asarray(tip, dtype=float).reshape(3)
                )
                look_len = float(np.linalg.norm(look_vec))
                if look_len > 1e-6:
                    unit = look_vec / look_len
                    return (float(unit[0]), float(unit[1]), float(unit[2]))
        if self._pick_resolved_ready_dir_world is not None:
            return self._pick_resolved_ready_dir_world
        if self._pick_look_dir_world is not None:
            return self._pick_look_dir_world
        obj = (
            object_world
            or self._pick_latest_object_world()
            or self._pick_frozen_world()
            or self._pick_look_object_world_xyz
            or self._pick_initial_object_world_xyz
        )
        if obj is None:
            return None
        return self._pick_auto_preferred_dir(obj, tip_world=tip_world)

    def _reset_pick_resolved_ready_state(self) -> None:
        self._pick_resolved_ready_dir_world = None
        self._pick_resolved_ready_pose_world_xyz = None

    def _compute_pick_ready_pose(
        self,
        object_world: tuple[float, float, float],
        *,
        tip_world: Optional[tuple[float, float, float]] = None,
        direction: Optional[tuple[float, float, float]] = None,
    ) -> Optional[tuple[float, float, float]]:
        dir_tuple = direction
        if dir_tuple is None:
            dir_tuple = self._pick_ready_direction(
                object_world=object_world,
                tip_world=tip_world,
            )
        if dir_tuple is None:
            return None
        try:
            return compute_ready_pose_target(
                tuple(float(v) for v in object_world),
                dir_tuple,
                standoff_m=float(self._pick_config_effective().ready_pose_standoff_m),
            )
        except ValueError:
            return None

    def _reset_pick_equal_sag_state(self) -> None:
        self._pick_initial_object_world_xyz = None
        self._pick_initial_ready_pose_world_xyz = None
        self._reset_pick_resolved_ready_state()
        self._reset_pick_look_state()
        self._reset_pick_equal_sag_result_state()

    def _reset_pick_look_state(self) -> None:
        self._pick_look_object_world_xyz = None
        self._pick_look_ready_pose_world_xyz = None
        self._pick_look_tip_world_xyz = None
        self._pick_look_dir_world = None
        self._pick_achieved_tip_world_xyz = None
        self._pick_achieved_dir_world = None

    def _pick_latch_fk_achieved_pose(
        self,
        *,
        host_state: Optional[HostState],
        sag_model: Optional[dict[str, Any]] = None,
    ) -> bool:
        """Record FK tip pose/direction after a move (actual, not IK target)."""
        if host_state is None or host_state.q is None:
            return False
        try:
            model = self._pick_reach_model(sag_model=sag_model)
            q0 = self._q_array_from_state(host_state)
            tip = np.asarray(model.grasp_position(q0), dtype=float).reshape(3)
            direc = np.asarray(model.grasp_direction(q0), dtype=float).reshape(3)
            if float(np.linalg.norm(direc)) <= 1e-9:
                return False
            direc = direc / float(np.linalg.norm(direc))
            self._pick_achieved_tip_world_xyz = (
                float(tip[0]),
                float(tip[1]),
                float(tip[2]),
            )
            self._pick_achieved_dir_world = (
                float(direc[0]),
                float(direc[1]),
                float(direc[2]),
            )
            return True
        except Exception:
            return False

    def _pick_fk_grasp_axis(
        self,
        *,
        host_state: Optional[HostState],
        sag_model: Optional[dict[str, Any]] = None,
    ) -> Optional[np.ndarray]:
        if host_state is None or host_state.q is None:
            return None
        try:
            model = self._pick_reach_model(sag_model=sag_model)
            q0 = self._q_array_from_state(host_state)
            direc = np.asarray(model.grasp_direction(q0), dtype=float).reshape(3)
            norm = float(np.linalg.norm(direc))
            if norm <= 1e-9:
                return None
            return direc / norm
        except Exception:
            return None

    def _reset_pick_equal_sag_result_state(self) -> None:
        self._pick_centered_object_world_xyz = None
        self._pick_centered_ready_pose_world_xyz = None
        self._pick_ready_pose_drift_world = None
        self._pick_corrected_object_world_xyz = None
        self._pick_equal_sag_estimate = None
        self._pick_equal_sag_model = None
        self._pick_equal_sag_attempted = False

    def _pick_corrected_ready_pose(self) -> Optional[tuple[float, float, float]]:
        """Pre-grasp target after Aim: current perception + equal-sag FK (not look-time object)."""
        if self._pick_centered_ready_pose_world_xyz is not None:
            return tuple(float(v) for v in self._pick_centered_ready_pose_world_xyz)
        centered_object = self._pick_centered_object_world_xyz
        if centered_object is None:
            return None
        return self._compute_pick_ready_pose(tuple(float(v) for v in centered_object))

    def _pick_latch_initial_ready_pose(self) -> bool:
        if self._pick_initial_ready_pose_world_xyz is not None:
            return True
        object_world = self._pick_latest_object_world() or self._pick_frozen_world()
        if object_world is None:
            return False
        ready_pose = self._compute_pick_ready_pose(
            object_world,
            tip_world=self._pick_current_tip_world(),
        )
        if ready_pose is None:
            return False
        self._pick_initial_object_world_xyz = tuple(float(v) for v in object_world)
        self._pick_initial_ready_pose_world_xyz = tuple(float(v) for v in ready_pose)
        if self._pick_frozen_world_xyz is None:
            self._pick_frozen_world_xyz = tuple(float(v) for v in object_world)
        print(
            "[Pick] equal_sag latch | initial_object=(%.3f, %.3f, %.3f) "
            "initial_ready=(%.3f, %.3f, %.3f)"
            % (
                float(object_world[0]),
                float(object_world[1]),
                float(object_world[2]),
                float(ready_pose[0]),
                float(ready_pose[1]),
                float(ready_pose[2]),
            )
        )
        return True

    def _send_look_object_anchor_markers(self) -> None:
        """Show Look-latched object world position in sim/host during Aim."""
        if self.client is None:
            return
        look_object = self._pick_look_object_world_xyz
        if look_object is None:
            return
        markers: list[dict[str, Any]] = [
            {
                "name": "look_object_anchor",
                "frame": "world",
                "pos": [float(v) for v in look_object],
                "color": [0.95, 0.20, 0.85, 0.95],
                "radius": 0.015,
                "ttl_ms": 600000,
            }
        ]
        look_dir = self._pick_look_dir_world
        if look_dir is not None:
            markers.append(
                {
                    "name": "look_object_anchor_dir",
                    "frame": "world",
                    "pos": [float(v) for v in look_object],
                    "dir": [float(v) for v in look_dir],
                    "color": [0.95, 0.20, 0.85, 0.55],
                    "radius": 0.005,
                    "length": 0.08,
                    "ttl_ms": 600000,
                }
            )
        self.client.send_debug_markers(markers, source="target")

    def _send_equal_sag_markers(self) -> None:
        if self.client is None:
            return
        corrected_object = self._pick_corrected_object_world_xyz
        centered_ready = self._pick_centered_ready_pose_world_xyz
        drift = self._pick_ready_pose_drift_world
        if corrected_object is None:
            return
        corrected_ready = self._compute_pick_ready_pose(corrected_object)
        markers: list[dict[str, Any]] = [
            {
                "name": "equal_sag_corrected_object",
                "frame": "world",
                "pos": [float(v) for v in corrected_object],
                "color": [1.0, 0.55, 0.05, 0.92],
                "radius": 0.012,
                "ttl_ms": 30000,
            }
        ]
        if corrected_ready is not None:
            direction = self._pick_ready_direction(object_world=corrected_object)
            if direction is not None:
                markers.append(
                    {
                        "name": "equal_sag_corrected_ready",
                        "frame": "world",
                        "pos": [float(v) for v in corrected_ready],
                        "dir": [float(v) for v in direction],
                        "color": [1.0, 0.75, 0.12, 0.95],
                        "radius": 0.011,
                        "ttl_ms": 30000,
                    }
                )
        if centered_ready is not None and drift is not None:
            drift_len = float(np.linalg.norm(np.asarray(drift, dtype=float).reshape(3)))
            markers.append(
                {
                    "name": "equal_sag_ready_drift",
                    "frame": "world",
                    "pos": [float(v) for v in centered_ready],
                    "dir": [float(v) for v in drift],
                    "color": [1.0, 0.42, 0.08, 0.70],
                    "radius": 0.005,
                    "length": drift_len,
                    "ttl_ms": 30000,
                }
            )
        self.client.send_debug_markers(markers, source="target")

    def _pick_ready_pose_drift_vectors(
        self,
        *,
        initial_object: tuple[float, float, float],
        centered_object: tuple[float, float, float],
        centered_direction: Optional[tuple[float, float, float]] = None,
    ) -> Optional[tuple[tuple[float, float, float], tuple[float, float, float], np.ndarray]]:
        """Ready-pose drift using pick standoff on both sides (not look view pose)."""
        initial_ready = self._compute_pick_ready_pose(initial_object)
        centered_ready = self._compute_pick_ready_pose(
            centered_object,
            direction=centered_direction,
        )
        if initial_ready is None or centered_ready is None:
            return None
        drift = (
            np.asarray(initial_ready, dtype=float).reshape(3)
            - np.asarray(centered_ready, dtype=float).reshape(3)
        )
        return (
            tuple(float(v) for v in initial_ready),
            tuple(float(v) for v in centered_ready),
            drift,
        )

    def _pick_try_estimate_equal_sag(self, host_state: Optional[HostState]) -> None:
        if bool(self._pick_equal_sag_attempted):
            return
        if not self._pick_latch_initial_ready_pose():
            return
        initial_object = self._pick_initial_object_world_xyz
        centered_object = self._pick_latest_object_world()
        if initial_object is None or centered_object is None:
            return
        pk = self._pick_config_effective()
        self.refresh_ik_context()
        base_sag = dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {}
        ctx = self._ik_context_for_host(host_state, sag_model=base_sag)
        q4 = self._q_array_from_state(host_state)
        fk_axis = self._pick_fk_grasp_axis(host_state=host_state, sag_model=base_sag)
        centered_dir: Optional[tuple[float, float, float]] = None
        if fk_axis is not None:
            centered_dir = (float(fk_axis[0]), float(fk_axis[1]), float(fk_axis[2]))
        drift_pack = self._pick_ready_pose_drift_vectors(
            initial_object=tuple(float(v) for v in initial_object),
            centered_object=tuple(float(v) for v in centered_object),
            centered_direction=centered_dir,
        )
        if drift_pack is None:
            return
        initial_ready, centered_ready, drift = drift_pack
        self._pick_equal_sag_attempted = True
        self._pick_initial_ready_pose_world_xyz = tuple(float(v) for v in initial_ready)
        self._pick_centered_object_world_xyz = tuple(float(v) for v in centered_object)
        self._pick_centered_ready_pose_world_xyz = tuple(float(v) for v in centered_ready)
        self._pick_ready_pose_drift_world = tuple(float(v) for v in drift)
        self._pick_corrected_object_world_xyz = tuple(float(v) for v in centered_object)

        reference = self._pick_ready_direction()
        prepared: Optional[SagDriftComponents] = None
        if fk_axis is not None and reference is not None:
            prepared = prepare_sag_drift_input(
                drift_world=drift,
                axis_world=fk_axis,
                reference_dir=reference,
                max_dir_error_deg=float(pk.sag_drift_max_dir_error_deg),
                max_lateral_m=float(pk.sag_drift_max_lateral_m),
                axial_only=bool(pk.sag_drift_axial_only),
            )
        if prepared is None or not bool(prepared.usable):
            reason = "missing_fk_axis" if prepared is None else str(prepared.reason)
            estimate = EqualSagEstimate(
                accepted=False,
                seg1_equal_offset_deg=0.0,
                seg2_equal_offset_deg=0.0,
                drift_world=tuple(float(v) for v in drift),
                reconstructed_drift_world=(0.0, 0.0, 0.0),
                residual_m=float(np.linalg.norm(drift)),
                condition=float("inf"),
                reason=str(reason),
            )
            axial_mm = (
                float(prepared.axial_m) * 1000.0 if prepared is not None else float("nan")
            )
            lateral_mm = (
                float(prepared.lateral_m) * 1000.0 if prepared is not None else float("nan")
            )
            dir_err = (
                float(prepared.dir_error_deg) if prepared is not None else float("nan")
            )
            print(
                "[Pick] equal_sag skipped | reason=%s axial=%.1fmm lateral=%.1fmm "
                "dir_err=%.1fdeg (max_dir=%.1fdeg max_lat=%.0fmm axial_only=%s)"
                % (
                    str(reason),
                    axial_mm,
                    lateral_mm,
                    dir_err,
                    float(pk.sag_drift_max_dir_error_deg),
                    float(pk.sag_drift_max_lateral_m) * 1000.0,
                    str(bool(pk.sag_drift_axial_only)).lower(),
                )
            )
        else:
            sag_input = prepared.sag_input_world
            try:
                estimate = estimate_equal_sag_from_ready_pose_drift(
                    context=ctx,
                    q4=q4,
                    ready_pose_drift_world=sag_input,
                    sag_model=base_sag,
                )
            except Exception as exc:
                estimate = EqualSagEstimate(
                    accepted=False,
                    seg1_equal_offset_deg=0.0,
                    seg2_equal_offset_deg=0.0,
                    drift_world=tuple(float(v) for v in sag_input),
                    reconstructed_drift_world=(0.0, 0.0, 0.0),
                    residual_m=float(np.linalg.norm(sag_input)),
                    condition=float("inf"),
                    reason=f"estimate_failed: {exc}",
                )
            if (not bool(estimate.accepted)) and str(estimate.reason) == "drift_too_small":
                estimate = EqualSagEstimate(
                    accepted=True,
                    seg1_equal_offset_deg=0.0,
                    seg2_equal_offset_deg=0.0,
                    drift_world=tuple(float(v) for v in sag_input),
                    reconstructed_drift_world=(0.0, 0.0, 0.0),
                    residual_m=float(np.linalg.norm(sag_input)),
                    condition=0.0,
                    reason="drift_too_small_zero_correction",
                )
            print(
                "[Pick] equal_sag drift frame | axial=%.1fmm lateral=%.1fmm dir_err=%.1fdeg"
                % (
                    float(prepared.axial_m) * 1000.0,
                    float(prepared.lateral_m) * 1000.0,
                    float(prepared.dir_error_deg),
                )
            )
        self._pick_equal_sag_estimate = estimate
        drift_mm = float(np.linalg.norm(drift) * 1000.0)
        if bool(estimate.accepted):
            self._pick_equal_sag_model = apply_equal_sag_offsets(
                base_sag,
                seg1_equal_offset_deg=float(estimate.seg1_equal_offset_deg),
                seg2_equal_offset_deg=float(estimate.seg2_equal_offset_deg),
            )
            self._send_equal_sag_markers()
        print(
            "[Pick] equal_sag %s | total_drift=%.1fmm seg1=%+.3fdeg seg2=%+.3fdeg "
            "residual=%.1fmm cond=%.1f search_steps=%d center_steps=%d approach_steps=%d "
            "reacquire_u=(roll=%+.1f, seg=%+.1f) reason=%s"
            % (
                "accepted" if bool(estimate.accepted) else "rejected",
                drift_mm,
                float(estimate.seg1_equal_offset_deg),
                float(estimate.seg2_equal_offset_deg),
                float(estimate.residual_m) * 1000.0,
                float(estimate.condition),
                int(self._pick_fov_search_steps_total),
                int(self._pick_center_steps_total),
                int(self._pick_approach_steps),
                float(self._pick_fov_reacquire_roll_u),
                float(self._pick_fov_reacquire_seg_u),
                str(estimate.reason),
            )
        )
        print(
            "[Pick] equal_sag drift detail | initial_object=(%.3f, %.3f, %.3f) "
            "centered_object=(%.3f, %.3f, %.3f) initial_ready=(%.3f, %.3f, %.3f) "
            "centered_ready=(%.3f, %.3f, %.3f) drift=(%+.3f, %+.3f, %+.3f) "
            "pick_target_object=(%.3f, %.3f, %.3f) pick_target_ready=(%.3f, %.3f, %.3f)"
            % (
                float(initial_object[0]),
                float(initial_object[1]),
                float(initial_object[2]),
                float(centered_object[0]),
                float(centered_object[1]),
                float(centered_object[2]),
                float(initial_ready[0]),
                float(initial_ready[1]),
                float(initial_ready[2]),
                float(centered_ready[0]),
                float(centered_ready[1]),
                float(centered_ready[2]),
                float(drift[0]),
                float(drift[1]),
                float(drift[2]),
                float(centered_object[0]),
                float(centered_object[1]),
                float(centered_object[2]),
                float(centered_ready[0]),
                float(centered_ready[1]),
                float(centered_ready[2]),
            )
        )

    def _pick_final_sag_model(self) -> dict[str, Any]:
        if isinstance(self._pick_equal_sag_model, dict):
            return dict(self._pick_equal_sag_model)
        return dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {}

    @staticmethod
    def _unit_vec3(v: Any, *, fallback: tuple[float, float, float] = (1.0, 0.0, 0.0)) -> np.ndarray:
        arr = np.asarray(v, dtype=float).reshape(3)
        norm = float(np.linalg.norm(arr))
        if norm <= 1e-9:
            return np.asarray(fallback, dtype=float).reshape(3)
        return arr / norm

    def _send_grasp_trajectory_markers(
        self,
        *,
        start_position: tuple[float, float, float],
        end_position: tuple[float, float, float],
        object_world: tuple[float, float, float],
        waypoints: list[GraspWaypoint],
        highlight_idx: int = -1,
        look_anchor_position: tuple[float, float, float] | None = None,
    ) -> None:
        if self.client is None:
            return
        markers = build_grasp_trajectory_markers(
            start_position=start_position,
            end_position=end_position,
            object_world=object_world,
            waypoints=waypoints,
            highlight_idx=int(highlight_idx),
            look_anchor_position=look_anchor_position,
        )
        self.client.send_debug_markers(markers, source="target")

    def _send_grasp_target_markers(
        self,
        *,
        object_world: tuple[float, float, float],
        target: np.ndarray,
        direction: np.ndarray,
        actual_offset_m: float,
        corrected: bool,
    ) -> None:
        if self.client is None:
            return
        obj = np.asarray(object_world, dtype=float).reshape(3)
        tgt = np.asarray(target, dtype=float).reshape(3)
        d = self._unit_vec3(direction)
        standoff_vec = tgt - obj
        color = [1.0, 0.75, 0.12, 0.95] if bool(corrected) else [0.35, 0.85, 1.0, 0.95]
        line_color = [1.0, 0.55, 0.05, 0.65] if bool(corrected) else [0.35, 0.85, 1.0, 0.60]
        self.client.send_debug_markers(
            [
                {
                    "name": "grasp_target",
                    "frame": "world",
                    "pos": [float(v) for v in tgt],
                    "dir": [float(v) for v in d],
                    "color": color,
                    "radius": 0.014,
                    "ttl_ms": 30000,
                },
                {
                    "name": "grasp_standoff",
                    "frame": "world",
                    "pos": [float(v) for v in obj],
                    "dir": [float(v) for v in standoff_vec],
                    "color": line_color,
                    "radius": 0.006,
                    "length": float(actual_offset_m),
                    "ttl_ms": 30000,
                },
            ],
            source="target",
        )

    def _send_ready_pose_markers(
        self,
        *,
        object_world: tuple[float, float, float],
        target: np.ndarray,
        direction: np.ndarray,
        actual_offset_m: float,
        corrected: bool,
    ) -> None:
        if self.client is None:
            return
        obj = np.asarray(object_world, dtype=float).reshape(3)
        tgt = np.asarray(target, dtype=float).reshape(3)
        ready_to_object = obj - tgt
        color = [1.0, 0.75, 0.12, 0.95] if bool(corrected) else [0.72, 1.0, 0.28, 0.95]
        line_color = [1.0, 0.55, 0.05, 0.65] if bool(corrected) else [0.72, 1.0, 0.28, 0.60]
        self.client.send_debug_markers(
            [
                {
                    "name": "ready_pose",
                    "frame": "world",
                    "pos": [float(v) for v in tgt],
                    "color": color,
                    "radius": 0.005,
                    "ttl_ms": 30000,
                },
                {
                    "name": "ready_pose_dir",
                    "frame": "world",
                    "pos": [float(v) for v in tgt],
                    "dir": [float(v) for v in ready_to_object],
                    "color": line_color,
                    "radius": 0.004,
                    "length": float(actual_offset_m),
                    "ttl_ms": 30000,
                },
            ],
            source="target",
        )


class ReadySolveActions(ReadyGeometryActions):
    """Feasible ready-pose resolution, IK dispatch, and grasp entry."""

    def _start_ready_pose_resolve_and_solve(
        self,
        *,
        object_world: tuple[float, float, float],
        preferred_dir: np.ndarray,
        sag_model: dict[str, Any],
        label: str,
        corrected: bool,
        resolve_dir: bool,
        target_world: Optional[tuple[float, float, float]] = None,
        max_dir_error_deg: Optional[float] = None,
        accept_best_effort_dir_error_deg: Optional[float] = None,
        pick_phase: str = ObjectPickPhase.READY.value,
        profile_phase: str = "ready",
        close_gripper_after: bool = False,
    ) -> None:
        self.refresh_ik_context()
        ctx = self._ik_context_for_host(None, sag_model=sag_model)
        required = (
            "limit",
            "fk_joint_chain",
            "terminal_link_name",
            "old_tip_local_offset",
            "grasp_offset_node_local",
        )
        if any(k not in ctx for k in required):
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="missing IK context",
            )
            self.state.set_ik_status(
                running=False,
                converged=False,
                failed=True,
                err_m=float("inf"),
                msg="missing IK context",
            )
            return

        pk = self._pick_config_effective()
        preferred_arr = self._unit_vec3(preferred_dir)
        self.state.set_pick_status(
            running=True,
            failed=False,
            phase=str(pick_phase),
            msg=f"{label} resolving feasible dir" if bool(resolve_dir) else f"{label} solving",
        )
        self.state.set_ik_status(
            running=True,
            converged=False,
            failed=False,
            err_m=float("inf"),
            msg=f"{label} resolving feasible dir" if bool(resolve_dir) else f"{label} solving",
        )

        def _worker() -> None:
            timing, t0 = self._begin_pick_profile(str(profile_phase))
            host_times: dict[str, float] = {}
            success = False
            try:
                host_state = self.client.refresh_state() if self.client is not None else None
                ctx_live = self._ik_context_for_host(host_state, sag_model=sag_model)
                current_seed = self._q_array_from_state(host_state)
                target_arr: np.ndarray
                direction_arr: np.ndarray
                q: Optional[np.ndarray] = None
                align_msg = ""
                position_error_m = float("inf")
                direction_error_deg = float("inf")
                resolved_meta = ""
                ready_align = self._ready_ik_align_kwargs()

                dir_tol_deg = (
                    float(max_dir_error_deg)
                    if max_dir_error_deg is not None
                    else float(pk.ready_pose_max_dir_error_deg)
                )
                if bool(resolve_dir):
                    resolved = resolve_feasible_ready_pose(
                        object_world=object_world,
                        preferred_dir=preferred_arr,
                        standoff_m=float(pk.ready_pose_standoff_m),
                        ik_context=ctx_live,
                        current_seed=current_seed,
                        position_tol_m=float(self._ik_cfg.tol),
                        max_iters=max(int(self._ik_cfg.max_iters), 1),
                        tweak_rounds=int(ready_align["tweak_rounds"]),
                        max_dir_error_deg=dir_tol_deg,
                        skip_search_under_deg=float(pk.ready_pose_skip_search_under_deg),
                        lateral_offsets_m=tuple(pk.ready_pose_lateral_offsets_m),
                        height_offsets_m=tuple(pk.ready_pose_height_offsets_m),
                        look_dot_min=float(pk.ready_pose_look_dot_min),
                        hand_eye_transform=self._hand_eye_transform,
                        hand_eye_parent_frame=self._hand_eye_parent_frame,
                        align_top_k=int(pk.ready_pose_align_top_k),
                        align_mode=str(ready_align["align_mode"]),
                        align_skip_under_deg=float(ready_align["align_skip_under_deg"]),
                        timing=timing,
                        accept_best_effort_dir_error_deg=accept_best_effort_dir_error_deg,
                    )
                    if not resolved.success or resolved.q is None:
                        fail_msg = (
                            "no feasible ready dir | best_dir_err=%.1fdeg evaluated=%d"
                            % (
                                float(resolved.best_rejected_dir_err_deg),
                                int(resolved.evaluated_count),
                            )
                        )
                        self.state.set_ik_status(
                            running=False,
                            converged=False,
                            failed=True,
                            err_m=float("inf"),
                            msg=fail_msg,
                        )
                        self.state.set_pick_status(
                            running=False,
                            failed=True,
                            phase=ObjectPickPhase.FAILED.value,
                            msg=fail_msg,
                        )
                        print("[Pick] %s failed | %s" % (str(label), fail_msg))
                        return

                    target_arr = np.asarray(resolved.resolved_target, dtype=float).reshape(3)
                    direction_arr = np.asarray(resolved.resolved_dir, dtype=float).reshape(3)
                    q = np.asarray(resolved.q, dtype=float).reshape(4)
                    position_error_m = float(resolved.position_error_m)
                    direction_error_deg = float(math.degrees(resolved.direction_angle_rad))
                    align_msg = (
                        "%s | tag=%s dir_err=%.1fdeg delta=%.1fdeg"
                        % (
                            str(resolved.reason),
                            str(resolved.candidate_tag),
                            float(math.degrees(resolved.direction_angle_rad)),
                            float(resolved.user_dir_delta_deg),
                        )
                    )
                    resolved_meta = (
                        "requested_dir=(%.3f, %.3f, %.3f) resolved_dir=(%.3f, %.3f, %.3f)"
                        % (
                            float(resolved.requested_dir[0]),
                            float(resolved.requested_dir[1]),
                            float(resolved.requested_dir[2]),
                            float(resolved.resolved_dir[0]),
                            float(resolved.resolved_dir[1]),
                            float(resolved.resolved_dir[2]),
                        )
                    )
                    self._pick_resolved_ready_dir_world = tuple(float(v) for v in direction_arr)
                    self._pick_resolved_ready_pose_world_xyz = tuple(float(v) for v in target_arr)
                else:
                    if target_world is not None:
                        target_arr = np.asarray(target_world, dtype=float).reshape(3)
                    else:
                        try:
                            target_tuple = compute_ready_pose_target(
                                object_world,
                                tuple(float(v) for v in preferred_arr),
                                standoff_m=float(pk.ready_pose_standoff_m),
                            )
                        except ValueError as exc:
                            self.state.set_pick_status(
                                running=False,
                                failed=True,
                                phase=ObjectPickPhase.FAILED.value,
                                msg=str(exc),
                            )
                            self.state.set_ik_status(
                                running=False,
                                converged=False,
                                failed=True,
                                err_m=float("inf"),
                                msg=str(exc),
                            )
                            return
                        target_arr = np.asarray(target_tuple, dtype=float).reshape(3)
                    direction_arr = preferred_arr
                    if timing is not None:
                        timing.ik_calls += 1
                        with timing.span("resolve_single"):
                            result = ik_pipeline.solve_then_align(
                                target_world=target_arr,
                                target_dir_world=direction_arr,
                                context=ctx_live,
                                position_tol_m=float(self._ik_cfg.tol),
                                max_iters=max(int(self._ik_cfg.max_iters), 1),
                                current_seed=current_seed,
                                timing=timing,
                                **ready_align,
                            )
                        timing.resolve_reason = "single_solve"
                        timing.candidates_evaluated = 1
                    else:
                        result = ik_pipeline.solve_then_align(
                            target_world=target_arr,
                            target_dir_world=direction_arr,
                            context=ctx_live,
                            position_tol_m=float(self._ik_cfg.tol),
                            max_iters=max(int(self._ik_cfg.max_iters), 1),
                            current_seed=current_seed,
                            **ready_align,
                        )
                    if (not result.success) or result.q is None:
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
                            msg=f"{label} IK failed | {result.reason}",
                        )
                        return
                    q = np.asarray(result.q, dtype=float).reshape(4)
                    position_error_m = float(result.position_error_m)
                    direction_error_deg = float(math.degrees(result.direction_angle_rad))
                    align_msg = str(result.reason)
                    if result.align_attempted:
                        align_msg = "%s | dir %.1f -> %.1f deg" % (
                            str(result.reason),
                            float(np.degrees(result.initial_direction_angle_rad)),
                            float(np.degrees(result.direction_angle_rad)),
                        )

                assert q is not None
                object_tuple = tuple(float(v) for v in object_world)
                actual_offset_m = float(
                    np.linalg.norm(np.asarray(object_tuple, dtype=float).reshape(3) - target_arr)
                )
                self.state.set_target(float(target_arr[0]), float(target_arr[1]), float(target_arr[2]))
                self.state.set_target_dir(
                    float(direction_arr[0]),
                    float(direction_arr[1]),
                    float(direction_arr[2]),
                )
                if str(profile_phase) == "grasp":
                    self.send_grasp_meta(source="target")
                    self._send_grasp_target_markers(
                        object_world=object_tuple,
                        target=target_arr,
                        direction=direction_arr,
                        actual_offset_m=actual_offset_m,
                        corrected=bool(corrected),
                    )
                elif str(profile_phase) == "ready":
                    self.send_ready_pose_meta(source="target")
                    self._send_ready_pose_markers(
                        object_world=object_tuple,
                        target=target_arr,
                        direction=direction_arr,
                        actual_offset_m=actual_offset_m,
                        corrected=bool(corrected),
                    )
                apply_timeout_s = 8.0 if bool(close_gripper_after) else 3.0
                host_state = self._apply_ik_solution_to_host(
                    q,
                    ik_target=target_arr,
                    ik_target_dir=direction_arr,
                    err_m=float(position_error_m),
                    status_msg=f"{label} | {align_msg}",
                    timeout_s=float(apply_timeout_s),
                    sag_model_override=dict(sag_model),
                    host_times=host_times,
                )
                if bool(corrected):
                    self._send_equal_sag_markers()
                claw_suffix = ""
                if bool(close_gripper_after):
                    closed_ok, claw_suffix = self._close_gripper_after_grasp_arrival(
                        host_state=host_state,
                        q_cmd=q,
                        target_world=target_arr,
                        sag_model=dict(sag_model),
                        label=str(label),
                    )
                    if not bool(closed_ok):
                        return
                done_msg = "%s done | err=%.1fmm dir_err=%.1fdeg align=%s" % (
                    str(label),
                    float(position_error_m) * 1000.0,
                    float(direction_error_deg),
                    str(ready_align["align_mode"]),
                )
                if claw_suffix:
                    done_msg = "%s | %s" % (done_msg, claw_suffix)
                if resolved_meta:
                    done_msg = "%s | %s" % (done_msg, resolved_meta)
                self.state.set_pick_status(
                    running=False,
                    failed=False,
                    phase=ObjectPickPhase.DONE.value,
                    msg=done_msg,
                )
                success = True
                print(
                    "[Pick] %s done | target=(%.3f, %.3f, %.3f) dir=(%.3f, %.3f, %.3f) "
                    "err=%.1fmm dir_err=%.1fdeg align=%s corrected=%s %s"
                    % (
                        str(label),
                        float(target_arr[0]),
                        float(target_arr[1]),
                        float(target_arr[2]),
                        float(direction_arr[0]),
                        float(direction_arr[1]),
                        float(direction_arr[2]),
                        float(position_error_m) * 1000.0,
                        float(direction_error_deg),
                        str(ready_align["align_mode"]),
                        str(bool(corrected)).lower(),
                        resolved_meta,
                    )
                )
            finally:
                self._finish_pick_profile(
                    phase=str(profile_phase),
                    timing=timing,
                    t0=t0,
                    host_times=host_times,
                    success=success,
                )
                self._ik_worker = None

        self._ik_worker = threading.Thread(
            target=traced_thread_target(f"pick.{profile_phase}", _worker),
            name=str(profile_phase),
            daemon=True,
        )
        self._ik_worker.start()

    def _start_grasp_to_object(self, *, internal: bool = False) -> bool:
        """IK move to pre-grasp point grasp_standoff_m before centered object along approach dir."""
        if not internal and (self.state.ik_running or self._visual_busy()):
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="busy",
            )
            return False
        if self.client is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="no host client",
            )
            return False
        object_world = self._pick_grasp_object_world()
        if object_world is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="grasp missing object (run Look or enable perception)",
            )
            return False
        sag_model = self._pick_grasp_sag_model()
        use_equal_sag = self._pick_grasp_uses_equal_sag()
        object_tuple = tuple(float(v) for v in object_world)
        dir_tuple = self._pick_ready_direction(
            object_world=object_tuple,
            prefer_current_tip=True,
        )
        if dir_tuple is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="cannot infer grasp approach direction",
            )
            return False
        pk = self._pick_config_effective()
        if bool(pk.local_img_jacobian_enabled) and self._use_hardware and not self._host_native_lji_runtime():
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="hardware grasp requires host-native LJI runtime",
            )
            print("[Grasp] blocked hardware LJI grasp outside host-native runtime")
            return False
        if bool(pk.grasp_guided_enabled):
            return self._start_grasp_guided_approach(internal=internal)
        direction = np.asarray(dir_tuple, dtype=float).reshape(3)
        standoff_m = float(max(pk.grasp_standoff_m, 0.0))
        try:
            grasp_target = compute_ready_pose_target(
                object_tuple,
                tuple(float(v) for v in direction),
                standoff_m=standoff_m,
            )
        except ValueError:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="cannot compute grasp target from approach direction",
            )
            return False
        self._start_ready_pose_resolve_and_solve(
            object_world=object_tuple,
            preferred_dir=direction,
            sag_model=sag_model,
            label="grasp pre-contact",
            corrected=bool(use_equal_sag),
            resolve_dir=False,
            target_world=grasp_target,
            pick_phase=ObjectPickPhase.GRASP.value,
            profile_phase="grasp",
            close_gripper_after=True,
        )
        return True

    def start_grasp(self) -> None:
        """Guided online loop toward pre-contact, or legacy one-shot pre-contact IK."""
        if self.state.ik_running or self._visual_busy():
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="busy",
            )
            return
        self._pick_stop_event.clear()
        self._start_grasp_to_object()

    def _wait_grasp_ik_done(self, *, timeout_s: float, label: str = "grasp") -> bool:
        deadline = time.time() + float(max(timeout_s, 1.0))
        while time.time() < deadline:
            if self._pick_stop_event.is_set():
                print("[Pick] %s | stopped" % str(label))
                return False
            if self.state.pick_failed and self._ik_worker is None:
                print(
                    "[Pick] %s | failed | %s"
                    % (str(label), str(self.state.pick_status_msg))
                )
                return False
            if self._ik_worker is None and not bool(self.state.pick_running):
                if str(self.state.pick_phase) == ObjectPickPhase.DONE.value:
                    return True
                if str(self.state.pick_phase) == ObjectPickPhase.FAILED.value:
                    return False
            time.sleep(0.05)
        print("[Pick] %s | timeout after %.1fs" % (str(label), float(timeout_s)))
        return False


class LookActions(ReadySolveActions):
    """Preferred-direction selection, rough pre-aim, and Look execution."""

    def _pick_user_preferred_dir(self) -> Optional[tuple[float, float, float]]:
        try:
            raw = np.asarray(self.state.mock_object_preferred_dir(), dtype=float).reshape(3)
        except Exception:
            return None
        if not np.all(np.isfinite(raw)):
            return None
        norm = float(np.linalg.norm(raw))
        if norm <= 1e-6:
            return None
        unit = raw / norm
        return (float(unit[0]), float(unit[1]), float(unit[2]))

    def _pick_look_seed_dir(
        self,
        object_world: tuple[float, float, float],
        *,
        tip_world: Optional[tuple[float, float, float]] = None,
    ) -> Optional[tuple[float, float, float]]:
        user_dir = self._pick_user_preferred_dir()
        if user_dir is not None:
            return user_dir
        return self._pick_auto_preferred_dir(object_world, tip_world=tip_world)

    def _solve_look_pose_candidate(
        self,
        *,
        object_tuple: tuple[float, float, float],
        preferred_world_arr: np.ndarray,
        standoff_m: float,
        ctx_live: dict[str, Any],
        q_seed: np.ndarray,
        pk: PickConfig,
        timing: Optional[PickTimingCollector],
    ) -> tuple[
        Optional[np.ndarray],
        Optional[np.ndarray],
        Optional[np.ndarray],
        float,
        str,
        Optional[str],
    ]:
        resolve_dir = bool(pk.look_pose_resolve_dir)
        preferred_world_arr = np.asarray(preferred_world_arr, dtype=float).reshape(3)
        if bool(resolve_dir):
            resolved = resolve_feasible_ready_pose(
                object_world=object_tuple,
                preferred_dir=preferred_world_arr,
                standoff_m=float(standoff_m),
                ik_context=ctx_live,
                current_seed=q_seed,
                position_tol_m=float(self._ik_cfg.tol),
                max_iters=max(int(self._ik_cfg.max_iters), 1),
                tweak_rounds=int(pk.ik_align_rounds),
                max_dir_error_deg=float(pk.look_pose_max_dir_error_deg),
                skip_search_under_deg=float(pk.look_pose_skip_search_under_deg),
                lateral_offsets_m=tuple(pk.look_pose_lateral_offsets_m),
                height_offsets_m=tuple(pk.look_pose_height_offsets_m),
                look_dot_min=float(pk.look_pose_look_dot_min),
                hand_eye_transform=self._hand_eye_transform,
                hand_eye_parent_frame=self._hand_eye_parent_frame,
                align_top_k=int(pk.look_pose_align_top_k),
                align_skip_under_deg=float(pk.ik_align_skip_under_deg),
                timing=timing,
            )
            if (
                not resolved.success
                or resolved.q is None
                or resolved.resolved_dir is None
                or resolved.resolved_target is None
            ):
                fail_msg = (
                    "no feasible view pose | best_dir_err=%.1fdeg evaluated=%d"
                    % (
                        float(resolved.best_rejected_dir_err_deg),
                        int(resolved.evaluated_count),
                    )
                )
                return None, None, None, float("inf"), "", fail_msg

            q = np.asarray(resolved.q, dtype=float).reshape(4)
            target_arr = np.asarray(resolved.resolved_target, dtype=float).reshape(3)
            look_dir_used = np.asarray(resolved.resolved_dir, dtype=float).reshape(3)
            align_msg = (
                "%s | tag=%s dir_err=%.1fdeg delta=%.1fdeg"
                % (
                    str(resolved.reason),
                    str(resolved.candidate_tag),
                    float(np.degrees(resolved.direction_angle_rad)),
                    float(resolved.user_dir_delta_deg),
                )
            )
            return (
                q,
                target_arr,
                look_dir_used,
                float(resolved.position_error_m),
                align_msg,
                None,
            )

        try:
            target_tuple = compute_ready_pose_target(
                object_tuple,
                tuple(float(v) for v in preferred_world_arr),
                standoff_m=float(standoff_m),
            )
        except ValueError as exc:
            return None, None, None, float("inf"), "", str(exc)
        target_arr = np.asarray(target_tuple, dtype=float).reshape(3)
        look_dir_used = preferred_world_arr
        if timing is not None:
            timing.ik_calls += 1
            with timing.span("resolve_single"):
                result = ik_pipeline.solve_then_align(
                    target_world=target_arr,
                    target_dir_world=look_dir_used,
                    context=ctx_live,
                    position_tol_m=float(self._ik_cfg.tol),
                    max_iters=max(int(self._ik_cfg.max_iters), 1),
                    current_seed=q_seed,
                    timing=timing,
                    **self._ik_align_kwargs(force_full=True),
                )
            timing.resolve_reason = "single_solve"
            timing.candidates_evaluated = 1
        else:
            result = ik_pipeline.solve_then_align(
                target_world=target_arr,
                target_dir_world=look_dir_used,
                context=ctx_live,
                position_tol_m=float(self._ik_cfg.tol),
                max_iters=max(int(self._ik_cfg.max_iters), 1),
                current_seed=q_seed,
                **self._ik_align_kwargs(force_full=True),
            )
        if not result.success or result.q is None:
            return (
                None,
                None,
                None,
                float(result.position_error_m),
                "",
                "look IK failed | " + str(result.reason),
            )
        align_msg = str(result.reason)
        if result.align_attempted:
            align_msg = "%s | dir %.1f -> %.1f deg" % (
                str(result.reason),
                float(np.degrees(result.initial_direction_angle_rad)),
                float(np.degrees(result.direction_angle_rad)),
            )
        return (
            np.asarray(result.q, dtype=float).reshape(4),
            target_arr,
            look_dir_used,
            float(result.position_error_m),
            align_msg,
            None,
        )

    def _look_pre_aim_rough(
        self,
        *,
        pk: PickConfig,
        host_state: Optional[HostState],
    ) -> tuple[bool, Optional[HostState], str]:
        if not bool(pk.look_pre_aim_enabled):
            return False, host_state, "disabled"
        if self.client is None:
            return False, host_state, "no host client"
        if self._perception_run_local:
            self._maybe_start_local_perception()
        lock_timeout = min(max(float(pk.acquire_timeout_s), 0.5), 2.5)
        if not self._wait_for_track_lock(
            timeout_s=float(lock_timeout),
            require_frames=max(1, min(int(pk.require_track_frames), 2)),
        ):
            return False, host_state, "track lock timeout"

        aim_cfg = replace(
            pk,
            target_uv_u=float(pk.look_pre_aim_target_uv_u),
            target_uv_v=float(pk.look_pre_aim_target_uv_v),
            center_tol=float(max(pk.look_pre_aim_tol, 0.01)),
            center_roll_max=min(float(pk.center_roll_max), 2.0),
            center_seg_max=min(float(pk.center_seg_max), 2.0),
        )
        max_steps = max(1, int(pk.look_pre_aim_max_steps))
        awful_tol = float(max(pk.look_pre_aim_awful_tol, aim_cfg.center_tol))
        step_scale = float(np.clip(float(pk.look_pre_aim_step_scale), 0.05, 1.0))
        last_obs: Optional[VisualObservation] = None
        for step_idx in range(max_steps):
            if self._pick_stop_event.is_set():
                return False, host_state, "stopped"
            host_state = self.client.refresh_state()
            obs = self.current_visual_observation(host_state)
            if obs is None:
                time.sleep(0.05)
                continue
            last_obs = obs
            u = float(obs.center_uv[0])
            v = float(obs.center_uv[1])
            du = u - float(aim_cfg.target_uv_u)
            dv = v - float(aim_cfg.target_uv_v)
            if abs(du) <= float(aim_cfg.center_tol) and abs(dv) <= float(aim_cfg.center_tol):
                return True, host_state, "offset reached"

            current_u = self.current_control_u()
            next_u, mode, _, _ = self._apply_pick_center_step(
                obs,
                current_u,
                cfg=aim_cfg,
                fallback_gains=False,
                coupled_axes=True,
                step_scale=step_scale,
            )
            if next_u == current_u:
                return abs(u) <= awful_tol and abs(v) <= awful_tol, host_state, "clamped"
            self.apply_control_u(
                u_linear=float(next_u.u_linear),
                u_roll=float(next_u.u_roll),
                u_s1=float(next_u.u_s1),
                u_s2=float(next_u.u_s2),
                apply_offset=True,
            )
            self.send_current_target(source="look_pre_aim")
            print(
                "[Look] pre-aim step %d/%d | uv=(%+.3f,%+.3f) target=(%+.3f,%+.3f) mode=%s"
                % (
                    int(step_idx + 1),
                    int(max_steps),
                    float(u),
                    float(v),
                    float(aim_cfg.target_uv_u),
                    float(aim_cfg.target_uv_v),
                    str(mode),
                )
            )
            time.sleep(0.10)

        if last_obs is None:
            return False, host_state, "no observation"
        u = float(last_obs.center_uv[0])
        v = float(last_obs.center_uv[1])
        if abs(u) <= awful_tol and abs(v) <= awful_tol:
            return True, host_state, "visible enough"
        return False, host_state, "awful view uv=(%+.3f,%+.3f)" % (u, v)

    def start_look(self) -> None:
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
        self._reset_pick_last_seen_uv()
        self._reset_pick_uv_jacobian()
        self._pick_stop_event.clear()
        self._reset_pick_search_state()
        self._reset_pick_drift_accounting()
        self._reset_pick_equal_sag_state()
        self._reset_grasp_guided_state()
        self._pick_frozen_world_xyz = None
        host_state = self.client.refresh_state()
        obs = self.current_visual_observation(host_state)
        if obs is not None:
            self._record_pick_last_seen_uv(obs)
        object_world = self._pick_latest_object_world() or self._pick_frozen_world()
        if object_world is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="no object world coordinate",
            )
            return

        base_sag = dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {}
        try:
            model = self._pick_reach_model(base_sag)
            q0 = self._q_array_from_state(host_state)
            if (
                self._go2_arm_mount is not None
                and host_state is not None
                and host_state.actual_tip_xyz is not None
            ):
                tip = np.asarray(host_state.actual_tip_xyz, dtype=float).reshape(3)
            else:
                tip = np.asarray(model.grasp_position(q0), dtype=float).reshape(3)
        except Exception as exc:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg=f"reach model failed: {exc}",
            )
            return

        object_arr = np.asarray(object_world, dtype=float).reshape(3)
        object_sim_tuple = tuple(float(v) for v in object_arr)
        object_tuple = object_sim_tuple
        tip_tuple = tuple(float(v) for v in tip)
        auto_dir = self._pick_look_seed_dir(object_sim_tuple, tip_world=tip_tuple)
        if auto_dir is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="cannot infer look seed direction",
            )
            return
        preferred_arr = np.asarray(auto_dir, dtype=float).reshape(3)
        self.refresh_ik_context()
        ctx = self._ik_context_for_host(host_state, sag_model=base_sag)
        required = (
            "limit",
            "fk_joint_chain",
            "terminal_link_name",
            "old_tip_local_offset",
            "grasp_offset_node_local",
        )
        if any(k not in ctx for k in required):
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="missing IK context",
            )
            return

        self.state.set_target(float(tip[0]), float(tip[1]), float(tip[2]))
        self.state.set_target_dir(
            float(preferred_arr[0]),
            float(preferred_arr[1]),
            float(preferred_arr[2]),
        )
        self.state.set_pick_status(
            running=True,
            failed=False,
            phase=ObjectPickPhase.LOOK.value,
            msg="look resolving feasible view pose",
        )
        self.state.set_ik_status(
            running=True,
            converged=False,
            failed=False,
            err_m=float("inf"),
            msg="look solving",
        )

        def _worker() -> None:
            timing, t0 = self._begin_pick_profile("look")
            host_times: dict[str, float] = {}
            success = False
            try:
                host_now = self.client.refresh_state() if self.client is not None else host_state
                ctx_live = self._ik_context_for_host(host_now, sag_model=base_sag)
                object_tuple = object_sim_tuple
                preferred_world_arr = np.asarray(preferred_arr, dtype=float).reshape(3)
                pk = self._pick_config_effective()
                q, target_arr, look_dir_used, err_m, align_msg, fail_msg = (
                    self._solve_look_pose_candidate(
                        object_tuple=object_tuple,
                        preferred_world_arr=preferred_world_arr,
                        standoff_m=float(pk.look_pose_standoff_m),
                        ctx_live=ctx_live,
                        q_seed=q0,
                        pk=pk,
                        timing=timing,
                    )
                )
                standoff_used = float(pk.look_pose_standoff_m)
                if fail_msg is not None:
                    print("[Look] preferred dir failed | %s" % str(fail_msg))
                    pre_ok, host_now, pre_reason = self._look_pre_aim_rough(
                        pk=pk,
                        host_state=host_now,
                    )
                    if pre_ok:
                        host_now = self.client.refresh_state() if self.client is not None else host_now
                        ctx_live = self._ik_context_for_host(host_now, sag_model=base_sag)
                        latest_object = self._pick_latest_object_world() or object_tuple
                        latest_obj_arr = np.asarray(latest_object, dtype=float).reshape(3)
                        tip_after = self._pick_current_tip_world(host_state=host_now)
                        if tip_after is not None:
                            tip_after_arr = np.asarray(tip_after, dtype=float).reshape(3)
                            look_vec = latest_obj_arr - tip_after_arr
                            dist = float(np.linalg.norm(look_vec))
                            if dist > 1e-6:
                                fallback_dir = look_vec / dist
                                standoff_used = float(np.clip(dist, 0.12, 0.30))
                                object_tuple = tuple(float(v) for v in latest_obj_arr)
                                q_seed = self._q_array_from_state(host_now)
                                (
                                    q,
                                    target_arr,
                                    look_dir_used,
                                    err_m,
                                    align_msg,
                                    fallback_fail,
                                ) = self._solve_look_pose_candidate(
                                    object_tuple=object_tuple,
                                    preferred_world_arr=fallback_dir,
                                    standoff_m=float(standoff_used),
                                    ctx_live=ctx_live,
                                    q_seed=q_seed,
                                    pk=pk,
                                    timing=timing,
                                )
                                if fallback_fail is None:
                                    align_msg = "pre-aim fallback | " + str(align_msg)
                                    fail_msg = None
                                else:
                                    fail_msg = (
                                        "%s | pre-aim=%s | fallback=%s"
                                        % (str(fail_msg), str(pre_reason), str(fallback_fail))
                                    )
                            else:
                                fail_msg = "%s | pre-aim=%s | fallback degenerate geometry" % (
                                    str(fail_msg),
                                    str(pre_reason),
                                )
                        else:
                            fail_msg = "%s | pre-aim=%s | no fallback tip" % (
                                str(fail_msg),
                                str(pre_reason),
                            )
                    else:
                        fail_msg = "%s | pre-aim=%s" % (str(fail_msg), str(pre_reason))

                if fail_msg is not None:
                    self.state.set_ik_status(
                        running=False,
                        converged=False,
                        failed=True,
                        err_m=float(err_m),
                        msg=str(fail_msg),
                    )
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg=str(fail_msg),
                    )
                    return

                if q is None or target_arr is None or look_dir_used is None:
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg="look: missing IK solution",
                    )
                    return

                look_dir_world = np.asarray(look_dir_used, dtype=float).reshape(3)
                target_world = np.asarray(target_arr, dtype=float).reshape(3)
                look_tuple = tuple(float(v) for v in look_dir_world)
                view_tuple = tuple(float(v) for v in target_world)
                self.state.set_target(float(target_world[0]), float(target_world[1]), float(target_world[2]))
                self.state.set_target_dir(
                    float(look_dir_world[0]),
                    float(look_dir_world[1]),
                    float(look_dir_world[2]),
                )
                self._pick_resolved_ready_dir_world = look_tuple
                self._pick_resolved_ready_pose_world_xyz = view_tuple
                self._apply_ik_solution_to_host(
                    q,
                    ik_target=target_world,
                    ik_target_dir=look_dir_world,
                    err_m=float(err_m),
                    status_msg="look | " + align_msg,
                    timeout_s=3.0,
                    sag_model_override=dict(base_sag),
                    host_times=host_times,
                )
                host_after = self.client.refresh_state() if self.client is not None else None
                self._pick_latch_fk_achieved_pose(
                    host_state=host_after,
                    sag_model=dict(base_sag),
                )
                if bool(pk.look_post_sag_trim_enabled):
                    host_after = self._look_post_sag_trim_to_object(
                        object_world=object_sim_tuple,
                        sag_model=dict(base_sag),
                        host_state=host_after,
                    )
                if bool(pk.look_post_uv_recover_enabled):
                    host_after = self._look_post_move_uv_recover(
                        pk=pk,
                        host_state=host_after,
                        object_world=object_sim_tuple,
                        sag_model=dict(base_sag),
                    )
                latch_object_arr = np.asarray(object_sim_tuple, dtype=float).reshape(3)
                if self._pick_look_object_world_xyz is not None:
                    latch_object_arr = np.asarray(
                        self._pick_look_object_world_xyz, dtype=float
                    ).reshape(3)
                look_latch = look_tuple
                if self._pick_look_dir_world is not None:
                    look_latch = tuple(float(v) for v in self._pick_look_dir_world)
                elif self._pick_achieved_dir_world is not None:
                    look_latch = tuple(float(v) for v in self._pick_achieved_dir_world)
                if self._pick_achieved_tip_world_xyz is not None:
                    tip_tuple = tuple(float(v) for v in self._pick_achieved_tip_world_xyz)
                dir_err_deg = float("nan")
                if self._pick_achieved_dir_world is not None:
                    dot = float(
                        np.clip(
                            float(
                                np.dot(
                                    np.asarray(self._pick_achieved_dir_world, dtype=float).reshape(3),
                                    np.asarray(look_tuple, dtype=float).reshape(3),
                                )
                            ),
                            -1.0,
                            1.0,
                        )
                    )
                    dir_err_deg = float(math.degrees(math.acos(dot)))
                latch_object = (
                    float(latch_object_arr[0]),
                    float(latch_object_arr[1]),
                    float(latch_object_arr[2]),
                )
                self._pick_look_object_world_xyz = latch_object
                self._pick_look_ready_pose_world_xyz = view_tuple
                self._pick_look_tip_world_xyz = tip_tuple
                self._pick_look_dir_world = look_latch
                self._pick_resolved_ready_dir_world = look_latch
                self._pick_initial_object_world_xyz = latch_object
                self._pick_initial_ready_pose_world_xyz = view_tuple
                self._pick_frozen_world_xyz = latch_object
                self.state.set_pick_status(
                    running=False,
                    failed=False,
                    phase=ObjectPickPhase.DONE.value,
                    msg=(
                        "look done | view_pose=(%.3f, %.3f, %.3f) standoff=%.0fmm"
                        % (
                            float(target_arr[0]),
                            float(target_arr[1]),
                            float(target_arr[2]),
                            float(standoff_used) * 1000.0,
                        )
                    ),
                )
                success = True
                print(
                    "[Pick] look done | object=(%.3f, %.3f, %.3f) view_pose=(%.3f, %.3f, %.3f) "
                    "look_dir=(%.3f, %.3f, %.3f) standoff=%.0fmm dir_err=%.1fdeg"
                    % (
                        float(latch_object_arr[0]),
                        float(latch_object_arr[1]),
                        float(latch_object_arr[2]),
                        float(target_arr[0]),
                        float(target_arr[1]),
                        float(target_arr[2]),
                        float(look_latch[0]),
                        float(look_latch[1]),
                        float(look_latch[2]),
                        float(standoff_used) * 1000.0,
                        float(dir_err_deg),
                    )
                )
            finally:
                self._finish_pick_profile(
                    phase="look",
                    timing=timing,
                    t0=t0,
                    host_times=host_times,
                    success=success,
                )
                self._ik_worker = None

        self._ik_worker = threading.Thread(
            target=traced_thread_target("pick.look", _worker),
            name="look",
            daemon=True,
        )
        self._ik_worker.start()


class ReadyActions(LookActions):
    """Public ready-pose action exposed to the controller service."""

    def start_ready_pose(self) -> None:
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
        if self._pick_look_ready_pose_world_xyz is None or self._pick_look_dir_world is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="run Look first",
            )
            return
        self._pick_stop_event.clear()
        corrected_ready = self._pick_corrected_ready_pose()
        use_corrected = corrected_ready is not None and isinstance(self._pick_equal_sag_model, dict)
        if use_corrected:
            object_world = self._pick_centered_object_world_xyz
            if object_world is None:
                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg="corrected ready missing centered object",
                )
                return
            sag_model = dict(self._pick_equal_sag_model)
            label = "corrected pre-grasp"
            target_world = tuple(float(v) for v in corrected_ready)
            dir_tuple = self._pick_ready_direction(
                object_world=tuple(float(v) for v in object_world),
                prefer_current_tip=True,
            )
        else:
            object_world = self._pick_look_object_world_xyz
            if object_world is None:
                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg="run Look first",
                )
                return
            sag_model = dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {}
            label = "pre-grasp"
            dir_tuple = self._pick_look_dir_world
            target_world = self._compute_pick_ready_pose(
                tuple(float(v) for v in object_world),
                direction=dir_tuple,
            )
            if target_world is None:
                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg="cannot compute pre-grasp target",
                )
                return

        if dir_tuple is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="cannot infer pre-grasp direction",
            )
            return

        object_tuple = tuple(float(v) for v in object_world)
        direction = np.asarray(dir_tuple, dtype=float).reshape(3)
        pk = self._pick_config_effective()
        accept_best_effort = (
            float(pk.ready_pose_corrected_max_dir_error_deg)
            if bool(use_corrected)
            else None
        )
        self._start_ready_pose_resolve_and_solve(
            object_world=object_tuple,
            preferred_dir=direction,
            sag_model=sag_model,
            label=label,
            corrected=bool(use_corrected),
            resolve_dir=bool(pk.ready_pose_resolve_dir),
            target_world=target_world,
            max_dir_error_deg=float(pk.ready_pose_max_dir_error_deg),
            accept_best_effort_dir_error_deg=accept_best_effort,
        )
