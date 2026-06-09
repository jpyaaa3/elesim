from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import replace
from time import perf_counter
from typing import Any, Optional

import numpy as np

from engine import ik as ik_pipeline
from engine.iklib import kinematics as ik_kin
from engine.config_loader import IkConfig, PerceptionConfig, PickConfig
from engine.protocol import ControlU, SimMappingConfig, SimQ, control_u_to_sim_q, sim_q_to_control_u
from engine.sag_model import load_sag_model_json
from engine.visual_servoing.equal_sag_probe import (
    EqualSagEstimate,
    SagDriftComponents,
    apply_equal_sag_offsets,
    estimate_equal_sag_from_ready_pose_drift,
    prepare_sag_drift_input,
)
from engine.profile.pick_timing import (
    GraspPlanProfile,
    GraspPlanStats,
    PickPhaseProfile,
    PickTimingCollector,
    enabled as pick_profile_enabled,
    fk_call_count,
    format_grasp_plan_report,
    format_report,
    install_fk_counter,
    reset_fk_count,
    uninstall_fk_counter,
)
from engine.visual_servoing.feasible_ready_pose import resolve_feasible_ready_pose
from engine.visual_servoing.grasp_trajectory import (
    GraspWaypoint,
    build_grasp_trajectory_markers,
    plan_grasp_approach_trajectory,
    plan_grasp_feasible_next_waypoint,
    plan_grasp_feasible_trajectory,
    trajectory_path_length_m,
)
from engine.visual_servoing.uv_jacobian import (
    broyden_update_uv_jacobian,
    default_uv_jacobian,
    solve_uv_control_delta,
)

from .client import ControlClient
from .perception import VisualObservation, extract_visual_observation
from .object_pick import (
    ObjectPickPhase,
    compute_ready_pose_target,
    evaluate_pick_convergence,
    pick_ready_for_extend,
    pick_uv_deltas,
)
from .perception_capture import PerceptionCapture, PerceptionSnapshot, TrackerPhase
from .state import HostState, PanelState


DEFAULT_SAG_MODEL_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "assets", "sag_model.json")


def resolve_sag_model_path(path: str) -> str:
    raw = str(path or "").strip()
    if not raw:
        return DEFAULT_SAG_MODEL_PATH
    if os.path.isabs(raw):
        return raw
    return os.path.abspath(raw)


def load_sag_model_or_empty(path: str) -> dict[str, Any]:
    model = load_sag_model_json(resolve_sag_model_path(path))
    return dict(model) if isinstance(model, dict) else {}


def resolve_initial_sag_model() -> dict[str, Any]:
    try:
        model = load_sag_model_or_empty(DEFAULT_SAG_MODEL_PATH)
        if isinstance(model, dict) and model:
            return model
    except Exception:
        pass
    return {}


class ControlService:
    """Controller-side actions: IK solve, target send, host commands."""

    def __init__(
        self,
        state: PanelState,
        client: Optional[ControlClient] = None,
        mapping_cfg: Optional[SimMappingConfig] = None,
        ik_cfg: Optional[IkConfig] = None,
        ik_context: Optional[dict[str, Any]] = None,
        config_path: Optional[str] = None,
        perception_cfg: Optional[PerceptionConfig] = None,
        pick_cfg: Optional[PickConfig] = None,
        hand_eye_transform: Optional[np.ndarray] = None,
        hand_eye_parent_frame: str = "node9",
        use_hardware: bool = True,
    ) -> None:
        self.state = state
        self.client = client
        self._use_hardware = bool(use_hardware)
        self._mapping_cfg = mapping_cfg or SimMappingConfig()
        self._ik_cfg = ik_cfg or IkConfig()
        self._ik_context = dict(ik_context or {})
        self._config_path = None if config_path is None else str(config_path)
        self._perception_cfg = perception_cfg or PerceptionConfig()
        self._pick_cfg = pick_cfg or PickConfig()
        self._hand_eye_transform = (
            None
            if hand_eye_transform is None
            else np.asarray(hand_eye_transform, dtype=float).reshape(4, 4).copy()
        )
        self._hand_eye_parent_frame = str(hand_eye_parent_frame)
        self._perception_capture: Optional[PerceptionCapture] = None
        self._last_pick_profile: Optional[PickPhaseProfile] = None
        self._last_grasp_plan_profile: Optional[GraspPlanProfile] = None
        self._pick_worker: Optional[threading.Thread] = None
        self._pick_e2e_worker: Optional[threading.Thread] = None
        self._pick_e2e_cancel = threading.Event()
        self._pick_e2e_phase_timeout_s = 300.0
        self._pick_stop_event = threading.Event()
        self._pick_center_phase = "u"
        self._pick_approach_v_hold_ratio = 0.5
        self._pick_approach_seg_u_max = 0.35
        self._pick_approach_latched = False
        self._pick_extend_done = False
        self._pick_extend_latched = False
        self._pick_extend_progress_m = 0.0
        self._pick_extend_stall = 0
        self._pick_frozen_world_xyz: Optional[tuple[float, float, float]] = None
        self._pick_center_reenter_ratio = 1.5
        self._pick_approach_lost_ratio = 2.5
        self._pick_approach_linear_step_scale = 3.0
        self._pick_clamp_streak = 0
        self._pick_clamp_stall_limit = 20
        self._pick_scale_stuck_iters = 0
        self._pick_scale_stuck_burst = False
        self._pick_center_stuck_iters = 0
        self._pick_approach_steps = 0
        self._pick_approach_plateau_iters = 0
        self._pick_approach_last_scale: Optional[float] = None
        self._pick_approach_scale_plateau = False
        self._pick_initial_object_world_xyz: Optional[tuple[float, float, float]] = None
        self._pick_initial_ready_pose_world_xyz: Optional[tuple[float, float, float]] = None
        self._pick_centered_object_world_xyz: Optional[tuple[float, float, float]] = None
        self._pick_centered_ready_pose_world_xyz: Optional[tuple[float, float, float]] = None
        self._pick_ready_pose_drift_world: Optional[tuple[float, float, float]] = None
        self._pick_corrected_object_world_xyz: Optional[tuple[float, float, float]] = None
        self._pick_look_object_world_xyz: Optional[tuple[float, float, float]] = None
        self._pick_look_ready_pose_world_xyz: Optional[tuple[float, float, float]] = None
        self._pick_look_tip_world_xyz: Optional[tuple[float, float, float]] = None
        self._pick_look_dir_world: Optional[tuple[float, float, float]] = None
        self._pick_achieved_tip_world_xyz: Optional[tuple[float, float, float]] = None
        self._pick_achieved_dir_world: Optional[tuple[float, float, float]] = None
        self._pick_resolved_ready_dir_world: Optional[tuple[float, float, float]] = None
        self._pick_resolved_ready_pose_world_xyz: Optional[tuple[float, float, float]] = None
        self._pick_equal_sag_estimate: Optional[EqualSagEstimate] = None
        self._pick_equal_sag_model: Optional[dict[str, Any]] = None
        self._pick_equal_sag_attempted = False
        self._grasp_waypoint_idx = 0
        self._grasp_online_sag_model: Optional[dict[str, Any]] = None
        self._grasp_nominal_dir: Optional[tuple[float, float, float]] = None
        self._grasp_trajectory_nominal_pose: Optional[tuple[float, float, float]] = None
        self._grasp_planned_waypoints: list[GraspWaypoint] = []
        self._grasp_plan_ready = False
        self._grasp_plan_traj_start: Optional[tuple[float, float, float]] = None
        self._grasp_plan_object_world: Optional[tuple[float, float, float]] = None
        self._grasp_plan_look_anchor: Optional[tuple[float, float, float]] = None
        self._pick_search_origin_u: Optional[ControlU] = None
        self._pick_search_step_index = 0
        self._pick_search_max_steps = 48
        self._pick_aim_step_scale = 1.0
        self._pick_aim_command_timeout_s = 1.0
        self._pick_aim_settle_s = 0.08
        self._pick_aim_gain_fallback_uv = 0.35
        self._pick_aim_v_min_seg_step = 2.5
        self._pick_aim_v_only_gain_scale = 2.5
        self._pick_aim_progress_eps = 0.015
        self._pick_aim_stuck_iters = 0
        self._pick_aim_best_uv_err: Optional[float] = None
        self._pick_aim_jacobian_resets = 0
        self._pick_aim_jacobian_reset_max = 2
        self._pick_search_roll_step_u = 3.0
        self._pick_search_seg_step_u = 3.0
        self._pick_search_roll_max_u = 36.0
        self._pick_search_seg_max_u = 30.0
        self._pick_prev_seen_uv: Optional[tuple[float, float]] = None
        self._pick_prev_seen_uv_wall = 0.0
        self._pick_last_seen_uv: Optional[tuple[float, float]] = None
        self._pick_last_seen_uv_wall = 0.0
        self._pick_last_seen_uv_delta: Optional[tuple[float, float]] = None
        self._pick_lost_follow_count = 0
        self._pick_lost_follow_max_steps = 18
        self._pick_lost_follow_uv_timeout_s = 4.0
        self._pick_lost_follow_roll_step_u = 1.5
        self._pick_lost_follow_seg_step_u = 1.2
        self._pick_lost_fallback_seg1_step_u = 0.8
        self._pick_uv_jacobian = default_uv_jacobian(
            center_u_gain=float(self._pick_cfg.center_u_gain),
            center_v_gain=float(self._pick_cfg.center_v_gain),
        )
        self._pick_uv_jacobian_last_u3: Optional[np.ndarray] = None
        self._pick_uv_jacobian_last_uv: Optional[np.ndarray] = None
        self._pick_uv_jacobian_update_count = 0
        self._pick_fov_search_steps_total = 0
        self._pick_center_steps_total = 0
        self._pick_fov_reacquire_roll_u = 0.0
        self._pick_fov_reacquire_seg_u = 0.0
        self._ik_worker: Optional[threading.Thread] = None
        self._calibration_current_threshold_ma = 1400
        self._calibration_current_delta_ma = 350
        self._calibration_current_min_threshold_ma = 650
        self._calibration_current_min_rise_ma = 200
        self._calibration_abort_current_ma = 2000
        self._calibration_step_u = 1.0
        self._calibration_poll_s = 0.22
        self._calibration_ema_alpha = 0.35
        self._calibration_release_consecutive = 3
        self._calibration_baseline_samples = 4
        self._calibration_feedback_reads = 6
        # Host motor_currents_ma keys use seg1/seg2; control-u axes use s1/s2.
        self._calibration_current_keys = {
            "s1": ("s1", "seg1"),
            "s2": ("s2", "seg2"),
        }
        self._visual_obs_stale_s = 0.75

    def _wait_until_q_settled(
        self,
        target_q: np.ndarray,
        *,
        timeout_s: float = 1.0,
        linear_tol_m: float = 2e-3,
        angle_tol_rad: float = math.radians(2.0),
        consecutive: int = 3,
    ) -> Optional[HostState]:
        if self.client is None:
            time.sleep(0.15)
            return None
        deadline = time.time() + float(max(timeout_s, 0.05))
        target = np.asarray(target_q, dtype=float).reshape(4)
        sim_mode = not bool(self._use_hardware)
        poll_s = 0.02 if sim_mode else 0.05
        required_stable = 2 if sim_mode else max(int(consecutive), 1)
        stable_count = 0
        last_state: Optional[HostState] = None

        def _is_settled(q_now: np.ndarray) -> bool:
            return (
                abs(float(q_now[0] - target[0])) <= float(linear_tol_m)
                and abs(float(q_now[1] - target[1])) <= float(angle_tol_rad)
                and abs(float(q_now[2] - target[2])) <= float(angle_tol_rad)
                and abs(float(q_now[3] - target[3])) <= float(angle_tol_rad)
            )

        if sim_mode:
            state = self.client.refresh_state()
            last_state = state
            if state is not None and state.q is not None:
                q_now = np.array(
                    [
                        float(state.q.linear_m),
                        float(state.q.roll_rad),
                        float(state.q.theta1_rad),
                        float(state.q.theta2_rad),
                    ],
                    dtype=float,
                )
                if _is_settled(q_now):
                    return state

        while time.time() < deadline:
            time.sleep(float(poll_s))
            state = self.client.refresh_state()
            last_state = state
            if state is None or state.q is None:
                continue
            q_now = np.array(
                [
                    float(state.q.linear_m),
                    float(state.q.roll_rad),
                    float(state.q.theta1_rad),
                    float(state.q.theta2_rad),
                ],
                dtype=float,
            )
            if _is_settled(q_now):
                stable_count += 1
                if stable_count >= required_stable:
                    return state
            else:
                stable_count = 0
        return last_state

    @staticmethod
    def _q_near_commanded(
        q_now: np.ndarray,
        target_q: np.ndarray,
        *,
        linear_tol_m: float = 2e-3,
        angle_tol_rad: float = math.radians(2.0),
    ) -> bool:
        now = np.asarray(q_now, dtype=float).reshape(4)
        target = np.asarray(target_q, dtype=float).reshape(4)
        return (
            abs(float(now[0] - target[0])) <= float(linear_tol_m)
            and abs(float(now[1] - target[1])) <= float(angle_tol_rad)
            and abs(float(now[2] - target[2])) <= float(angle_tol_rad)
            and abs(float(now[3] - target[3])) <= float(angle_tol_rad)
        )

    def _grasp_position_from_host_state(
        self,
        host_state: Optional[HostState],
        *,
        sag_model: Optional[dict[str, Any]] = None,
    ) -> Optional[np.ndarray]:
        if host_state is None or host_state.q is None:
            return None
        try:
            model = self._pick_reach_model(sag_model=sag_model)
            q = np.array(
                [
                    float(host_state.q.linear_m),
                    float(host_state.q.roll_rad),
                    float(host_state.q.theta1_rad),
                    float(host_state.q.theta2_rad),
                ],
                dtype=float,
            )
            return np.asarray(model.grasp_position(q), dtype=float).reshape(3)
        except Exception:
            return None

    def _wait_until_grasp_target_reached(
        self,
        *,
        target_world: np.ndarray,
        q_cmd: np.ndarray,
        sag_model: dict[str, Any],
        timeout_s: float = 10.0,
        position_tol_m: Optional[float] = None,
    ) -> tuple[bool, float, Optional[HostState]]:
        """Wait until commanded q is applied and FK grasp point reaches pre-contact target."""
        if self.client is None:
            return False, float("inf"), None
        target = np.asarray(target_world, dtype=float).reshape(3)
        q_target = np.asarray(q_cmd, dtype=float).reshape(4)
        tol_m = float(position_tol_m if position_tol_m is not None else self._ik_cfg.tol)
        tol_m = max(tol_m, 0.005)
        sim_mode = not bool(self._use_hardware)
        poll_s = 0.02 if sim_mode else 0.05
        required_stable = 2 if sim_mode else 3
        stable = 0
        last_err_m = float("inf")
        last_state: Optional[HostState] = None
        deadline = time.time() + float(max(timeout_s, 0.5))

        while time.time() < deadline:
            time.sleep(float(poll_s))
            state = self.client.refresh_state()
            last_state = state
            if state is None or state.q is None or (not bool(state.reply_ok)):
                stable = 0
                continue
            q_now = np.array(
                [
                    float(state.q.linear_m),
                    float(state.q.roll_rad),
                    float(state.q.theta1_rad),
                    float(state.q.theta2_rad),
                ],
                dtype=float,
            )
            if not self._q_near_commanded(q_now, q_target):
                stable = 0
                continue
            grasp_pos = self._grasp_position_from_host_state(state, sag_model=sag_model)
            if grasp_pos is None:
                stable = 0
                continue
            last_err_m = float(np.linalg.norm(grasp_pos - target))
            if last_err_m <= tol_m:
                stable += 1
                if stable >= required_stable:
                    return True, last_err_m, state
            else:
                stable = 0
        return False, last_err_m, last_state

    def _close_gripper_after_grasp_arrival(
        self,
        *,
        host_state: Optional[HostState],
        q_cmd: np.ndarray,
        target_world: np.ndarray,
        sag_model: dict[str, Any],
        label: str,
        arrival_timeout_s: float = 10.0,
    ) -> tuple[bool, str]:
        if host_state is None or (not bool(host_state.reply_ok)):
            reason = (
                str(host_state.reply_reason).strip()
                if host_state is not None
                else "no host state"
            ) or "host apply failed"
            fail_msg = f"{label} | cannot close gripper: {reason}"
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg=fail_msg,
            )
            self.state.set_ik_status(
                running=False,
                converged=False,
                failed=True,
                err_m=float("inf"),
                msg=fail_msg,
            )
            print("[Pick] %s" % fail_msg)
            return False, fail_msg

        close_tol_m = max(float(self._ik_cfg.tol), 0.003)
        reached, arrival_err_m, settle_state = self._wait_until_grasp_target_reached(
            target_world=target_world,
            q_cmd=q_cmd,
            sag_model=sag_model,
            timeout_s=float(arrival_timeout_s),
            position_tol_m=close_tol_m,
        )
        if not bool(reached):
            fail_msg = (
                "%s | pre-contact not reached (err=%.1fmm); gripper kept open"
                % (str(label), float(arrival_err_m) * 1000.0)
            )
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg=fail_msg,
            )
            self.state.set_ik_status(
                running=False,
                converged=False,
                failed=True,
                err_m=float(arrival_err_m),
                msg=fail_msg,
            )
            print("[Pick] %s" % fail_msg)
            return False, fail_msg

        if settle_state is not None:
            grasp_pos = self._grasp_position_from_host_state(
                settle_state,
                sag_model=sag_model,
            )
            if grasp_pos is not None:
                arrival_err_m = float(
                    np.linalg.norm(
                        np.asarray(grasp_pos, dtype=float).reshape(3)
                        - np.asarray(target_world, dtype=float).reshape(3)
                    )
                )
                if arrival_err_m > close_tol_m + 1e-4:
                    fail_msg = (
                        "%s | pre-contact verify failed (err=%.1fmm); gripper kept open"
                        % (str(label), float(arrival_err_m) * 1000.0)
                    )
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg=fail_msg,
                    )
                    self.state.set_ik_status(
                        running=False,
                        converged=False,
                        failed=True,
                        err_m=float(arrival_err_m),
                        msg=fail_msg,
                    )
                    print("[Pick] %s" % fail_msg)
                    return False, fail_msg

        self.state.set_claw_closed(True)
        self.send_claw_command(closed=True)
        done_suffix = "arrival=%.1fmm | claw closed" % (float(arrival_err_m) * 1000.0)
        print(
            "[Pick] %s | pre-contact reached | err=%.1fmm | claw closed"
            % (str(label), float(arrival_err_m) * 1000.0)
        )
        return True, done_suffix

    def _ik_align_kwargs(self, *, force_full: bool = False) -> dict[str, Any]:
        pk = self._pick_config_effective()
        mode = "full" if bool(force_full) else str(pk.ik_align_mode).strip().lower()
        if mode not in ("full", "lite"):
            mode = "lite"
        return {
            "align_mode": mode,
            "align_skip_under_deg": float(pk.ik_align_skip_under_deg),
            "tweak_rounds": max(int(pk.ik_align_rounds), 1),
        }

    def _grasp_look_at_ik_kwargs(self) -> dict[str, Any]:
        """Look-at-object grasp IK: position GN + tweak (no full align seed bank)."""
        pk = self._pick_config_effective()
        drift_tol_deg = float(max(pk.grasp_waypoint_max_approach_drift_deg, 0.1))
        return {
            "align_skip_under_deg": float(pk.ik_align_skip_under_deg),
            "tweak_rounds": max(int(pk.ik_align_rounds), 1),
            "direction_tol_deg": drift_tol_deg,
            "tweak_position_hold_tol_m": max(float(self._ik_cfg.tol), 1e-3),
        }

    def _ready_ik_align_kwargs(self) -> dict[str, Any]:
        pk = self._pick_config_effective()
        mode = str(pk.ready_pose_align_mode).strip().lower()
        if mode not in ("full", "lite"):
            mode = "full"
        return {
            "align_mode": mode,
            "align_skip_under_deg": float(pk.ready_pose_align_skip_under_deg),
            "tweak_rounds": max(int(pk.ik_align_rounds), 1),
        }

    def refresh_ik_context(self) -> None:
        if not self._config_path:
            return
        try:
            _, ik_context = ik_pipeline.load_solver_context(self._config_path)
            self._ik_context = dict(ik_context or {})
        except Exception as exc:
            print(f"[UI] IK context reload failed: {exc}")

    def refresh_host_state(self) -> Optional[HostState]:
        if self.client is None:
            return None
        host_state = self.client.refresh_state()
        if host_state.q is not None:
            self.state.set_q(
                float(host_state.q.linear_m),
                float(host_state.q.roll_rad),
                float(host_state.q.theta1_rad),
                float(host_state.q.theta2_rad),
            )
        return host_state

    def has_client(self) -> bool:
        return self.client is not None

    def current_host_state(self) -> Optional[HostState]:
        if self.client is None:
            return None
        return self.client.get_state()

    def current_visual_observation(self, host_state: Optional[HostState] = None) -> Optional[VisualObservation]:
        state = host_state if host_state is not None else self.current_host_state()
        return extract_visual_observation(
            state,
            target_label=str(self.state.visual_target_label),
            stale_timeout_s=float(self._visual_obs_stale_s),
            min_confidence=float(self.state.visual_confidence_min),
        )

    def _visual_target_uv(self) -> tuple[float, float]:
        return (float(self.state.visual_target_uv_u), float(self.state.visual_target_uv_v))

    def _visual_uv_errors(self, obs: VisualObservation) -> tuple[float, float, float, float]:
        tu, tv = self._visual_target_uv()
        u = float(obs.center_uv[0])
        v = float(obs.center_uv[1])
        return u - tu, v - tv, tu, tv

    def _uv_control_errors(self, obs: VisualObservation) -> tuple[float, float]:
        du, dv, _, _ = self._visual_uv_errors(obs)
        return -du, -dv

    def _center_seg_du(
        self,
        *,
        target_v: float,
        obs_v: float,
        cap: float,
        gain: float,
    ) -> float:
        """seg drives image v toward target_v (+seg lowers normalized v on this robot)."""
        g = float(gain)
        v_delta = float(obs_v) - float(target_v)
        return float(np.clip(g * v_delta, -float(cap), float(cap)))

    def _visual_uv_centered(self, obs: VisualObservation, *, center_tol: Optional[float] = None) -> bool:
        tol = float(self.state.visual_center_tol if center_tol is None else center_tol)
        du, dv, _, _ = self._visual_uv_errors(obs)
        return abs(du) <= tol and abs(dv) <= tol

    @staticmethod
    def _control_u3(display_u: ControlU) -> np.ndarray:
        return np.array(
            [
                float(display_u.u_roll),
                float(display_u.u_s1),
                float(display_u.u_s2),
            ],
            dtype=float,
        )

    def _reset_pick_uv_jacobian(self) -> None:
        cfg = self._pick_config_effective()
        self._pick_uv_jacobian = default_uv_jacobian(
            center_u_gain=float(cfg.center_u_gain),
            center_v_gain=float(cfg.center_v_gain),
        )
        self._pick_uv_jacobian_last_u3 = None
        self._pick_uv_jacobian_last_uv = None
        self._pick_uv_jacobian_update_count = 0

    def _reset_pick_aim_progress(self) -> None:
        self._pick_aim_stuck_iters = 0
        self._pick_aim_best_uv_err = None
        self._pick_aim_jacobian_resets = 0

    def _update_pick_uv_jacobian(
        self,
        *,
        current_u: ControlU,
        obs: VisualObservation,
    ) -> None:
        u3 = self._control_u3(current_u)
        uv = np.asarray(obs.center_uv, dtype=float).reshape(2)
        if (
            self._pick_uv_jacobian_last_u3 is not None
            and self._pick_uv_jacobian_last_uv is not None
        ):
            control_delta = u3 - self._pick_uv_jacobian_last_u3
            uv_delta = uv - self._pick_uv_jacobian_last_uv
            before = np.asarray(self._pick_uv_jacobian, dtype=float).copy()
            self._pick_uv_jacobian = broyden_update_uv_jacobian(
                before,
                control_delta=control_delta,
                uv_delta=uv_delta,
                alpha=0.35,
                min_control_norm=0.35,
            )
            if not np.allclose(before, self._pick_uv_jacobian):
                self._pick_uv_jacobian_update_count += 1
        self._pick_uv_jacobian_last_u3 = u3
        self._pick_uv_jacobian_last_uv = uv

    def _visual_busy(self) -> bool:
        return self._ik_worker is not None or self._pick_worker is not None

    def _pick_busy(self) -> bool:
        return self._pick_worker is not None

    def pick_e2e_running(self) -> bool:
        worker = self._pick_e2e_worker
        return worker is not None and worker.is_alive()

    def _wait_pick_phase_done(self, *, timeout_s: float, label: str) -> bool:
        deadline = time.time() + float(max(timeout_s, 1.0))
        while time.time() < deadline:
            if self._pick_e2e_cancel.is_set():
                print("[E2E] %s | cancelled" % str(label))
                return False
            if self.state.pick_failed:
                print(
                    "[E2E] %s | failed | %s"
                    % (str(label), str(self.state.pick_status_msg))
                )
                return False
            if (
                str(self.state.pick_phase) == ObjectPickPhase.DONE.value
                and not bool(self.state.pick_running)
                and self._ik_worker is None
                and self._pick_worker is None
            ):
                return True
            time.sleep(0.05)
        print("[E2E] %s | timeout after %.1fs" % (str(label), float(timeout_s)))
        return False

    def stop_pick_e2e(self) -> None:
        self._pick_e2e_cancel.set()
        self.stop_object_pick()

    def start_look_aim_grasp_e2e(self) -> None:
        """Run Look -> Aim -> Grasp (pre-contact IK + close gripper)."""
        if self.pick_e2e_running() or self._pick_busy() or self._visual_busy():
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

        self._pick_e2e_cancel.clear()
        timeout_s = float(self._pick_e2e_phase_timeout_s)

        def _worker() -> None:
            try:
                print("[E2E] start | Look -> Aim -> Grasp")
                self.state.set_pick_status(
                    running=True,
                    failed=False,
                    phase=ObjectPickPhase.LOOK.value,
                    msg="E2E: Look",
                )

                self.start_look()
                if self.state.pick_failed:
                    return
                if not self._wait_pick_phase_done(timeout_s=timeout_s, label="look"):
                    if not self.state.pick_failed:
                        self.state.set_pick_status(
                            running=False,
                            failed=True,
                            phase=ObjectPickPhase.FAILED.value,
                            msg="E2E: look timeout",
                        )
                    return

                self.state.set_pick_status(
                    running=True,
                    failed=False,
                    phase=ObjectPickPhase.ACQUIRE.value,
                    msg="E2E: Aim",
                )
                self.start_aim()
                if self.state.pick_failed:
                    return
                if not self._wait_pick_phase_done(timeout_s=timeout_s, label="aim"):
                    if not self.state.pick_failed:
                        self.state.set_pick_status(
                            running=False,
                            failed=True,
                            phase=ObjectPickPhase.FAILED.value,
                            msg="E2E: aim timeout",
                        )
                    return

                self.state.set_pick_status(
                    running=True,
                    failed=False,
                    phase=ObjectPickPhase.GRASP.value,
                    msg="E2E: Grasp",
                )
                self.start_grasp()
                if self.state.pick_failed:
                    return
                if not self._wait_pick_phase_done(timeout_s=timeout_s, label="grasp"):
                    if not self.state.pick_failed:
                        self.state.set_pick_status(
                            running=False,
                            failed=True,
                            phase=ObjectPickPhase.FAILED.value,
                            msg="E2E: grasp timeout",
                        )
                    return

                if str(self.state.pick_phase) != ObjectPickPhase.DONE.value:
                    return

                self.state.set_pick_status(
                    running=False,
                    failed=False,
                    phase=ObjectPickPhase.DONE.value,
                    msg="E2E done | Look -> Aim -> Grasp",
                )
                print("[E2E] done | Look -> Aim -> Grasp")
            finally:
                self._pick_e2e_worker = None

        self._pick_e2e_worker = threading.Thread(
            target=_worker,
            name="pick-e2e",
            daemon=True,
        )
        self._pick_e2e_worker.start()

    def start_look_aim_e2e(self) -> None:
        """Run Look -> Aim only (no grasp)."""
        if self.pick_e2e_running() or self._pick_busy() or self._visual_busy():
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

        self._pick_e2e_cancel.clear()
        timeout_s = float(self._pick_e2e_phase_timeout_s)

        def _worker() -> None:
            try:
                print("[E2E] start | Look -> Aim")
                self.state.set_pick_status(
                    running=True,
                    failed=False,
                    phase=ObjectPickPhase.LOOK.value,
                    msg="E2E: Look",
                )

                self.start_look()
                if self.state.pick_failed:
                    return
                if not self._wait_pick_phase_done(timeout_s=timeout_s, label="look"):
                    if not self.state.pick_failed:
                        self.state.set_pick_status(
                            running=False,
                            failed=True,
                            phase=ObjectPickPhase.FAILED.value,
                            msg="E2E: look timeout",
                        )
                    return

                self.state.set_pick_status(
                    running=True,
                    failed=False,
                    phase=ObjectPickPhase.ACQUIRE.value,
                    msg="E2E: Aim",
                )
                self.start_aim()
                if self.state.pick_failed:
                    return
                if not self._wait_pick_phase_done(timeout_s=timeout_s, label="aim"):
                    if not self.state.pick_failed:
                        self.state.set_pick_status(
                            running=False,
                            failed=True,
                            phase=ObjectPickPhase.FAILED.value,
                            msg="E2E: aim timeout",
                        )
                    return

                if str(self.state.pick_phase) != ObjectPickPhase.DONE.value:
                    return

                self.state.set_pick_status(
                    running=False,
                    failed=False,
                    phase=ObjectPickPhase.DONE.value,
                    msg="E2E done | Look -> Aim",
                )
                print("[E2E] done | Look -> Aim")
            finally:
                self._pick_e2e_worker = None

        self._pick_e2e_worker = threading.Thread(
            target=_worker,
            name="pick-e2e",
            daemon=True,
        )
        self._pick_e2e_worker.start()

    def start_look_ready_pick_e2e(self, *, pick_distance_m: float = 0.15) -> None:
        """Deprecated: use start_look_aim_grasp_e2e()."""
        _ = float(pick_distance_m)
        self.start_look_aim_grasp_e2e()

    def _reset_pick_search_state(self) -> None:
        self._pick_search_origin_u = None
        self._pick_search_step_index = 0
        self._pick_lost_follow_count = 0

    def _reset_pick_drift_accounting(self) -> None:
        self._pick_fov_search_steps_total = 0
        self._pick_center_steps_total = 0
        self._pick_fov_reacquire_roll_u = 0.0
        self._pick_fov_reacquire_seg_u = 0.0

    def _capture_pick_reacquire_offset(self) -> None:
        if self._pick_search_origin_u is None or self._pick_search_step_index <= 0:
            return
        current = self.current_control_u()
        origin = self._pick_search_origin_u
        self._pick_fov_reacquire_roll_u = float(current.u_roll - origin.u_roll)
        self._pick_fov_reacquire_seg_u = float(current.u_s1 - origin.u_s1)

    def _reset_pick_last_seen_uv(self) -> None:
        self._pick_prev_seen_uv = None
        self._pick_prev_seen_uv_wall = 0.0
        self._pick_last_seen_uv = None
        self._pick_last_seen_uv_wall = 0.0
        self._pick_last_seen_uv_delta = None

    def _record_pick_last_seen_uv(self, obs: VisualObservation) -> None:
        now = time.time()
        prev = self._pick_last_seen_uv
        prev_wall = float(self._pick_last_seen_uv_wall)
        current = (
            float(obs.center_uv[0]),
            float(obs.center_uv[1]),
        )
        self._pick_prev_seen_uv = prev
        self._pick_prev_seen_uv_wall = prev_wall
        if prev is not None and now >= prev_wall:
            self._pick_last_seen_uv_delta = (
                float(current[0] - prev[0]),
                float(current[1] - prev[1]),
            )
        else:
            self._pick_last_seen_uv_delta = None
        self._pick_last_seen_uv = current
        self._pick_last_seen_uv_wall = now

    def _pick_lost_exit_uv_delta(
        self,
        *,
        target_u: float,
        target_v: float,
        velocity_tol: float,
        position_tol: float,
    ) -> tuple[float, float, str]:
        delta = self._pick_last_seen_uv_delta
        if delta is not None:
            du_vel = float(delta[0])
            dv_vel = float(delta[1])
            if abs(du_vel) > float(velocity_tol) or abs(dv_vel) > float(velocity_tol):
                return du_vel, dv_vel, "uv_velocity"
        last_uv = self._pick_last_seen_uv
        if last_uv is None:
            return 0.0, 0.0, "none"
        du_pos = float(last_uv[0]) - float(target_u)
        dv_pos = float(last_uv[1]) - float(target_v)
        if abs(du_pos) <= float(position_tol) and abs(dv_pos) <= float(position_tol):
            return 0.0, 0.0, "none"
        return (
            du_pos,
            dv_pos,
            "last_uv_position",
        )

    @staticmethod
    def _pick_search_offset_for_index(
        index: int,
        *,
        roll_step: float,
        seg_step: float,
        roll_max: float,
        seg_max: float,
    ) -> tuple[float, float]:
        slot = int(index) % 8
        level = int(index) // 8 + 1
        roll_amp = min(float(level) * float(roll_step), float(roll_max))
        seg_amp = min(float(level) * float(seg_step), float(seg_max))
        pattern = (
            (+roll_amp, 0.0),
            (-roll_amp, 0.0),
            (0.0, +seg_amp),
            (0.0, -seg_amp),
            (+roll_amp, +seg_amp),
            (+roll_amp, -seg_amp),
            (-roll_amp, +seg_amp),
            (-roll_amp, -seg_amp),
        )
        return pattern[slot]

    def _track_locked(self, *, require_frames: int) -> bool:
        cap = self._perception_capture
        return (
            cap is not None
            and cap.tracker_phase() == TrackerPhase.TRACK.value
            and cap.track_ok_frames() >= int(require_frames)
        )

    def _pick_apply_lost_follow_step(self, *, reason: str) -> bool:
        if self._pick_lost_follow_count >= int(self._pick_lost_follow_max_steps):
            return False
        cap = self._perception_capture
        if cap is not None:
            cap.request_refresh()
        if self._pick_search_origin_u is None:
            self._pick_search_origin_u = self.current_control_u()

        cfg = self._pick_config_effective()
        current = self.current_control_u()
        tu = float(cfg.target_uv_u)
        tv = float(cfg.target_uv_v)
        roll_du = 0.0
        s1_du = 0.0
        s2_du = 0.0
        mode = "seg1_fallback"
        exit_du = 0.0
        exit_dv = 0.0
        exit_mode = "none"
        last_uv = self._pick_last_seen_uv
        last_age_s = time.time() - float(self._pick_last_seen_uv_wall)
        if (
            last_uv is not None
            and 0.0 <= last_age_s <= float(self._pick_lost_follow_uv_timeout_s)
        ):
            velocity_tol = max(float(cfg.center_tol) * 0.2, 0.01)
            position_tol = max(float(cfg.center_tol) * 0.5, 0.03)
            exit_du, exit_dv, exit_mode = self._pick_lost_exit_uv_delta(
                target_u=tu,
                target_v=tv,
                velocity_tol=velocity_tol,
                position_tol=position_tol,
            )
            if abs(exit_du) > velocity_tol or abs(exit_dv) > velocity_tol:
                du3 = solve_uv_control_delta(
                    uv_error=(float(exit_du), float(exit_dv)),
                    jacobian=self._pick_uv_jacobian,
                    damping=0.03,
                    gain=1.0,
                    max_abs_delta=(
                        float(self._pick_lost_follow_roll_step_u),
                        float(self._pick_lost_follow_seg_step_u),
                        float(self._pick_lost_follow_seg_step_u),
                    ),
                )
                roll_du = float(du3[0])
                s1_du = float(du3[1])
                s2_du = float(du3[2])
                mode = exit_mode

        if mode == "seg1_fallback":
            # On this arm, decreasing seg1 display-u raises the camera/head.
            s1_du = -float(self._pick_lost_fallback_seg1_step_u)

        next_u = self._clamp_display_u(
            ControlU(
                u_linear=float(current.u_linear),
                u_roll=float(current.u_roll + roll_du),
                u_s1=float(current.u_s1 + s1_du),
                u_s2=float(current.u_s2 + s2_du),
            )
        )
        if next_u == current:
            return False

        self._pick_lost_follow_count += 1
        self._pick_search_step_index += 1
        origin = self._pick_search_origin_u
        if origin is not None:
            self._pick_fov_reacquire_roll_u = float(next_u.u_roll - origin.u_roll)
            self._pick_fov_reacquire_seg_u = float(next_u.u_s1 - origin.u_s1)
        self._pick_fov_search_steps_total += 1
        self.state.set_pick_status(
            running=True,
            failed=False,
            phase=ObjectPickPhase.ACQUIRE.value,
            msg=(
                "lost follow %d/%d | %s | mode=%s droll=%+.1f ds1=%+.1f ds2=%+.1f"
                % (
                    int(self._pick_lost_follow_count),
                    int(self._pick_lost_follow_max_steps),
                    str(reason),
                    str(mode),
                    float(next_u.u_roll - current.u_roll),
                    float(next_u.u_s1 - current.u_s1),
                    float(next_u.u_s2 - current.u_s2),
                )
            ),
        )
        print(
            "[Pick] lost_follow step %d/%d | %s | mode=%s last_uv=%s age=%.2fs "
            "exit_uv=(%+.3f,%+.3f) exit_mode=%s | u=(%.1f, %.1f, %.1f, %.1f)"
            % (
                int(self._pick_lost_follow_count),
                int(self._pick_lost_follow_max_steps),
                str(reason),
                str(mode),
                "none"
                if last_uv is None
                else "(%+.3f,%+.3f)" % (float(last_uv[0]), float(last_uv[1])),
                float(last_age_s),
                float(exit_du),
                float(exit_dv),
                str(exit_mode),
                float(next_u.u_linear),
                float(next_u.u_roll),
                float(next_u.u_s1),
                float(next_u.u_s2),
            )
        )
        self._send_display_control_u_and_wait(
            next_u,
            timeout_s=float(self._pick_aim_command_timeout_s),
            source="slider",
        )
        time.sleep(float(self._pick_aim_settle_s))
        if cap is not None:
            cap.request_refresh()
        return True

    def _pick_apply_fov_search_step(self, *, reason: str) -> bool:
        cap = self._perception_capture
        if cap is not None:
            cap.request_refresh()
        if self._pick_search_step_index >= int(self._pick_search_max_steps):
            return False
        if self._pick_search_origin_u is None:
            self._pick_search_origin_u = self.current_control_u()
        origin = self._pick_search_origin_u
        roll_off, seg_off = self._pick_search_offset_for_index(
            int(self._pick_search_step_index),
            roll_step=float(self._pick_search_roll_step_u),
            seg_step=float(self._pick_search_seg_step_u),
            roll_max=float(self._pick_search_roll_max_u),
            seg_max=float(self._pick_search_seg_max_u),
        )
        self._pick_search_step_index += 1
        next_u = self._clamp_display_u(
            ControlU(
                u_linear=float(origin.u_linear),
                u_roll=float(origin.u_roll + roll_off),
                u_s1=float(origin.u_s1 + seg_off),
                u_s2=float(origin.u_s2 + seg_off),
            )
        )
        self.state.set_pick_status(
            running=True,
            failed=False,
            phase=ObjectPickPhase.ACQUIRE.value,
            msg=(
                "fov search %d/%d | %s | droll=%+.1f dseg=%+.1f"
                % (
                    int(self._pick_search_step_index),
                    int(self._pick_search_max_steps),
                    str(reason),
                    float(next_u.u_roll - origin.u_roll),
                    float(next_u.u_s1 - origin.u_s1),
                )
            ),
        )
        print(
            "[Pick] fov_search step %d/%d | %s | u=(%.1f, %.1f, %.1f, %.1f)"
            % (
                int(self._pick_search_step_index),
                int(self._pick_search_max_steps),
                str(reason),
                float(next_u.u_linear),
                float(next_u.u_roll),
                float(next_u.u_s1),
                float(next_u.u_s2),
            )
        )
        self._send_display_control_u_and_wait(
            next_u,
            timeout_s=float(self._pick_aim_command_timeout_s),
            source="slider",
        )
        time.sleep(float(self._pick_aim_settle_s))
        self._pick_fov_search_steps_total += 1
        self._pick_fov_reacquire_roll_u = float(next_u.u_roll - origin.u_roll)
        self._pick_fov_reacquire_seg_u = float(next_u.u_s1 - origin.u_s1)
        if cap is not None:
            cap.request_refresh()
        return True

    def _q_array_from_state(self, host_state: Optional[HostState] = None) -> np.ndarray:
        src = host_state if host_state is not None else self.current_host_state()
        if src is not None and src.q is not None:
            return np.array(
                [
                    float(src.q.linear_m),
                    float(src.q.roll_rad),
                    float(src.q.theta1_rad),
                    float(src.q.theta2_rad),
                ],
                dtype=float,
            )
        return np.array(
            [
                float(self.state.linear),
                float(self.state.roll),
                float(self.state.theta1),
                float(self.state.theta2),
            ],
            dtype=float,
        )

    def _clamp_q(self, q: np.ndarray) -> np.ndarray:
        arr = np.asarray(q, dtype=float).reshape(4).copy()
        cfg = self._mapping_cfg
        arr[0] = float(np.clip(arr[0], cfg.linear_q_min_m, cfg.linear_q_max_m))
        arr[1] = float(np.clip(arr[1], cfg.roll_q_min_rad, cfg.roll_q_max_rad))
        arr[2] = float(np.clip(arr[2], cfg.seg1_q_min_rad, cfg.seg1_q_max_rad))
        arr[3] = float(np.clip(arr[3], cfg.seg2_q_min_rad, cfg.seg2_q_max_rad))
        return arr

    def _begin_pick_profile(self, phase: str) -> tuple[Optional[PickTimingCollector], float]:
        if not pick_profile_enabled():
            return None, 0.0
        install_fk_counter()
        reset_fk_count()
        return PickTimingCollector(), perf_counter()

    def _finish_pick_profile(
        self,
        *,
        phase: str,
        timing: Optional[PickTimingCollector],
        t0: float,
        host_times: dict[str, float],
        success: bool,
    ) -> None:
        if timing is None or t0 <= 0.0:
            return
        profile = timing.to_profile(
            phase=str(phase),
            t_total_s=float(perf_counter() - t0),
            t_host_apply_s=float(host_times.get("host_apply_s", 0.0)),
            t_settle_s=float(host_times.get("settle_s", 0.0)),
            success=bool(success),
        )
        self._last_pick_profile = profile
        print(format_report(profile))
        uninstall_fk_counter()

    def _begin_grasp_plan_profile(
        self,
    ) -> tuple[Optional[PickTimingCollector], GraspPlanStats, float]:
        if not pick_profile_enabled():
            return None, GraspPlanStats(), 0.0
        install_fk_counter()
        reset_fk_count()
        return PickTimingCollector(), GraspPlanStats(), perf_counter()

    def _finish_grasp_plan_profile(
        self,
        *,
        timing: Optional[PickTimingCollector],
        stats: GraspPlanStats,
        t0: float,
        t_geom_s: float,
        t_kinematic_s: float,
        waypoints: int,
        geom_waypoints: int,
        ik_success: int,
        success: bool,
    ) -> None:
        if timing is None or t0 <= 0.0:
            return
        profile = GraspPlanProfile(
            t_total_s=float(perf_counter() - t0),
            t_geom_s=float(t_geom_s),
            t_kinematic_s=float(t_kinematic_s),
            t_solve_position_s=timing.get("solve_position"),
            t_align_s=timing.get("align_direction"),
            ik_calls=int(timing.ik_calls),
            ik_success=int(ik_success),
            fk_calls=fk_call_count(),
            waypoints=int(waypoints),
            geom_waypoints=int(geom_waypoints),
            stats=stats,
            success=bool(success),
        )
        self._last_grasp_plan_profile = profile
        print(format_grasp_plan_report(profile))
        uninstall_fk_counter()

    def _send_state_q_and_wait(
        self,
        *,
        timeout_s: float = 1.0,
        source: str = "ik",
        force: bool = False,
        sag_model_override: Optional[dict[str, Any]] = None,
        host_times: Optional[dict[str, float]] = None,
    ) -> Optional[HostState]:
        q_cmd = np.array(
            [
                float(self.state.linear),
                float(self.state.roll),
                float(self.state.theta1),
                float(self.state.theta2),
            ],
            dtype=float,
        )
        t_send = perf_counter()
        self.send_current_target(
            source=source,
            force=force,
            sag_model_override=sag_model_override,
        )
        t_after_send = perf_counter()
        host_state = self._wait_until_q_settled(q_cmd, timeout_s=float(timeout_s))
        if host_times is not None:
            host_times["host_apply_s"] = float(t_after_send - t_send)
            host_times["settle_s"] = float(perf_counter() - t_after_send)
        if host_state is not None and (not bool(host_state.reply_ok)):
            reason = str(host_state.reply_reason).strip() or "unknown host apply failure"
            self.state.set_ik_status(
                running=False,
                converged=False,
                failed=True,
                err_m=float(self.state.ik_err_m),
                msg=f"IK converged but HW apply failed: {reason}",
            )
        return host_state

    def _send_display_control_u_and_wait(self, display_u: ControlU, *, timeout_s: float = 1.0, source: str = "ik") -> Optional[HostState]:
        self.apply_control_u(
            u_linear=float(display_u.u_linear),
            u_roll=float(display_u.u_roll),
            u_s1=float(display_u.u_s1),
            u_s2=float(display_u.u_s2),
            apply_offset=True,
        )
        return self._send_state_q_and_wait(timeout_s=float(timeout_s), source=source)

    def _clamp_display_u(self, display_u: ControlU) -> ControlU:
        cfg = self.control_mapping()
        return ControlU(
            u_linear=float(np.clip(display_u.u_linear, cfg.linear_u_min, cfg.linear_u_max)),
            u_roll=float(np.clip(display_u.u_roll, cfg.roll_u_min, cfg.roll_u_max)),
            u_s1=float(np.clip(display_u.u_s1, cfg.seg_u_min, cfg.seg_u_max)),
            u_s2=float(np.clip(display_u.u_s2, cfg.seg_u_min, cfg.seg_u_max)),
        )

    def _command_q_and_wait(
        self,
        q: np.ndarray,
        *,
        timeout_s: float = 1.0,
        source: str = "slider",
        force: bool = False,
        sag_model_override: Optional[dict[str, Any]] = None,
    ) -> Optional[HostState]:
        q_cmd = self._clamp_q(q)
        self.state.set_q(float(q_cmd[0]), float(q_cmd[1]), float(q_cmd[2]), float(q_cmd[3]))
        return self._send_state_q_and_wait(
            timeout_s=float(timeout_s),
            source=source,
            force=force,
            sag_model_override=sag_model_override,
        )

    def _apply_ik_solution_to_host(
        self,
        q: np.ndarray,
        *,
        ik_target: np.ndarray,
        ik_target_dir: np.ndarray,
        err_m: float,
        status_msg: str,
        timeout_s: float = 2.0,
        sag_model_override: Optional[dict[str, Any]] = None,
        host_times: Optional[dict[str, float]] = None,
    ) -> Optional[HostState]:
        """Same path as UI Solve IK: update panel target + q, then send to sim/host."""
        q_cmd = self._clamp_q(q)
        target = np.asarray(ik_target, dtype=float).reshape(3)
        direction = np.asarray(ik_target_dir, dtype=float).reshape(3)
        dnorm = float(np.linalg.norm(direction))
        if dnorm > 1e-9:
            direction = direction / dnorm
        self.state.set_target(float(target[0]), float(target[1]), float(target[2]))
        self.state.set_target_dir(float(direction[0]), float(direction[1]), float(direction[2]))
        self.state.set_q(float(q_cmd[0]), float(q_cmd[1]), float(q_cmd[2]), float(q_cmd[3]))
        self.state.set_ik_solution(float(q_cmd[1]), float(q_cmd[2]), float(q_cmd[3]))
        self.state.set_ik_status(
            running=False,
            converged=True,
            failed=False,
            err_m=float(err_m),
            msg=str(status_msg),
        )
        host_state = self._send_state_q_and_wait(
            timeout_s=float(timeout_s),
            source="ik",
            force=True,
            sag_model_override=sag_model_override,
            host_times=host_times,
        )
        if host_state is not None and (not bool(host_state.reply_ok)):
            reason = str(host_state.reply_reason).strip() or "unknown host apply failure"
            self.state.set_ik_status(
                running=False,
                converged=False,
                failed=True,
                err_m=float(err_m),
                msg=f"IK converged but HW apply failed: {reason}",
            )
        return host_state

    @staticmethod
    def _log_visual_step(tag: str, step_idx: int, step_max: int, **fields: object) -> None:
        parts = [f"[Visual] {tag} step {int(step_idx)}/{int(step_max)}"]
        for key, value in fields.items():
            parts.append(f"{key}={value}")
        print(" | ".join(parts))

    def _offsets(self) -> dict[str, float]:
        linear, roll, s1, s2, _rev = self.state.offset_values()
        return {
            "linear": float(linear),
            "roll": float(roll),
            "s1": float(s1),
            "s2": float(s2),
        }

    def _display_to_actual_u(self, display_u: ControlU, *, apply_offset: bool = True) -> ControlU:
        offsets = self._offsets() if bool(apply_offset) else {"linear": 0.0, "roll": 0.0, "s1": 0.0, "s2": 0.0}
        return ControlU(
            u_linear=float(display_u.u_linear + offsets["linear"]),
            u_roll=float(display_u.u_roll + offsets["roll"]),
            u_s1=float(display_u.u_s1 + offsets["s1"]),
            u_s2=float(display_u.u_s2 + offsets["s2"]),
        )

    def _actual_to_display_u(self, actual_u: ControlU) -> ControlU:
        offsets = self._offsets()
        return ControlU(
            u_linear=float(actual_u.u_linear - offsets["linear"]),
            u_roll=float(actual_u.u_roll - offsets["roll"]),
            u_s1=float(actual_u.u_s1 - offsets["s1"]),
            u_s2=float(actual_u.u_s2 - offsets["s2"]),
        )

    def current_control_u(self) -> ControlU:
        actual_u: ControlU
        if self.client is not None:
            actual_u = self.client.q_to_control_u(
                linear_m=float(self.state.linear),
                roll_rad=float(self.state.roll),
                theta1_rad=float(self.state.theta1),
                theta2_rad=float(self.state.theta2),
            )
        else:
            actual_u = sim_q_to_control_u(
                SimQ(
                    linear_m=float(self.state.linear),
                    roll_rad=float(self.state.roll),
                    theta1_rad=float(self.state.theta1),
                    theta2_rad=float(self.state.theta2),
                ),
                self._mapping_cfg,
            )
        display_u = self._actual_to_display_u(actual_u)
        cfg = self.control_mapping()
        return ControlU(
            u_linear=float(min(max(display_u.u_linear, cfg.linear_u_min), cfg.linear_u_max)),
            u_roll=float(min(max(display_u.u_roll, cfg.roll_u_min), cfg.roll_u_max)),
            u_s1=float(min(max(display_u.u_s1, cfg.seg_u_min), cfg.seg_u_max)),
            u_s2=float(min(max(display_u.u_s2, cfg.seg_u_min), cfg.seg_u_max)),
        )

    def control_mapping(self) -> SimMappingConfig:
        return self.client.cfg if self.client is not None else self._mapping_cfg

    def current_offsets(self) -> dict[str, float]:
        return self._offsets()

    def apply_control_u(self, *, u_linear: float, u_roll: float, u_s1: float, u_s2: float, apply_offset: bool = True) -> None:
        actual_u = self._display_to_actual_u(
            ControlU(
                u_linear=float(u_linear),
                u_roll=float(u_roll),
                u_s1=float(u_s1),
                u_s2=float(u_s2),
            ),
            apply_offset=bool(apply_offset),
        )
        if self.client is not None:
            q_new = self.client.control_u_to_q(
                u_linear=float(actual_u.u_linear),
                u_roll=float(actual_u.u_roll),
                u_s1=float(actual_u.u_s1),
                u_s2=float(actual_u.u_s2),
            )
        else:
            q_new = control_u_to_sim_q(
                actual_u,
                self._mapping_cfg,
            )
        self.state.set_q(
            float(q_new.linear_m),
            float(q_new.roll_rad),
            float(q_new.theta1_rad),
            float(q_new.theta2_rad),
        )

    def apply_partial_control_u(self, partial_u: dict[str, float]) -> None:
        current_u = self.current_control_u()
        merged = {
            "linear": float(current_u.u_linear),
            "roll": float(current_u.u_roll),
            "s1": float(current_u.u_s1),
            "s2": float(current_u.u_s2),
        }
        for key, value in partial_u.items():
            merged[str(key).strip().lower()] = float(value)
        self.apply_control_u(
            u_linear=float(merged["linear"]),
            u_roll=float(merged["roll"]),
            u_s1=float(merged["s1"]),
            u_s2=float(merged["s2"]),
        )
        if self.client is not None:
            offsets = self._offsets()
            adjusted = {
                str(k).strip().lower(): float(v) + float(offsets[str(k).strip().lower()])
                for k, v in partial_u.items()
            }
            self.client.send_partial_control_u(adjusted, source="slider")

    def set_display_offset(self, axis: str, value: float) -> None:
        self.state.set_u_offset(axis, float(value))

    def home_controls(self) -> None:
        self.state.clear_ik_status()
        self.apply_control_u(u_linear=180.0, u_roll=180.0, u_s1=10.0, u_s2=10.0, apply_offset=True)
        self.send_current_target(source="slider")

    def extend_arm_controls(self) -> None:
        self.state.clear_ik_status()
        self.apply_control_u(u_linear=15.0, u_roll=180.0, u_s1=180.0, u_s2=180.0, apply_offset=False)
        self.send_current_target(source="slider")

    def send_current_target(
        self,
        *,
        source: str,
        force: bool = False,
        sag_model_override: Optional[dict[str, Any]] = None,
    ) -> None:
        if self.client is not None and (
            force or (not self.state.paused) or (source == "target")
        ):
            self.client.send_target_values(
                linear_m=float(self.state.linear),
                roll_rad=float(self.state.roll),
                theta1_rad=float(self.state.theta1),
                theta2_rad=float(self.state.theta2),
                source=source,
                target_xyz=(float(self.state.target_x), float(self.state.target_y), float(self.state.target_z)),
                target_dir=(float(self.state.target_vx), float(self.state.target_vy), float(self.state.target_vz)),
                sag_model=(
                    dict(sag_model_override)
                    if isinstance(sag_model_override, dict)
                    else (
                        dict(self.state.raw_sag_model)
                        if isinstance(self.state.raw_sag_model, dict)
                        else {}
                    )
                ),
                claw_closed=bool(self.state.claw_closed),
                force=force or bool(source == "target"),
            )

    def send_current_target_meta(self, *, source: str = "target") -> None:
        if self.client is not None:
            self.client.send_target_meta(
                target_xyz=(float(self.state.target_x), float(self.state.target_y), float(self.state.target_z)),
                target_dir=(float(self.state.target_vx), float(self.state.target_vy), float(self.state.target_vz)),
                source=source,
            )

    def send_ready_pose_meta(self, *, source: str = "target") -> None:
        if self.client is None:
            return
        dir_tuple = self._pick_ready_direction()
        if dir_tuple is None:
            with self.state._lock:
                dir_tuple = (
                    float(self.state.target_vx),
                    float(self.state.target_vy),
                    float(self.state.target_vz),
                )
            if float(np.linalg.norm(dir_tuple)) <= 1e-9:
                return
        self.client.send_ready_pose_meta(
            target_dir=dir_tuple,
            standoff_m=float(self.state.visual_ready_distance_m),
            source=source,
        )

    def send_grasp_meta(self, *, source: str = "target") -> None:
        if self.client is None:
            return
        dir_tuple = self._pick_ready_direction(prefer_current_tip=True)
        if dir_tuple is None:
            return
        pk = self._pick_config_effective()
        self.client.send_ready_pose_meta(
            target_dir=dir_tuple,
            standoff_m=float(pk.grasp_standoff_m),
            source=source,
        )

    def send_sag_model_meta(self, *, source: str = "target") -> None:
        if self.client is not None:
            self.client.send_sag_model_meta(
                dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {},
                source=source,
            )

    def load_sag_model(self, model_path: str) -> tuple[str, dict[str, Any]]:
        resolved_path = resolve_sag_model_path(model_path)
        model = load_sag_model_or_empty(resolved_path)
        self.state.set_sag_model(resolved_path, model)
        return resolved_path, model

    def send_claw_command(self, *, closed: bool) -> None:
        if self.client is not None:
            self.client.send_claw_command(claw_closed=bool(closed), source="target")

    def _start_position_solve(self, target: np.ndarray) -> None:
        if self.state.ik_running or self._visual_busy():
            return
        if self._pick_busy():
            return
        self.refresh_ik_context()
        ctx = dict(self._ik_context)
        ctx["sag_model"] = dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {}
        required = ("limit", "fk_joint_chain", "terminal_link_name", "old_tip_local_offset", "grasp_offset_node_local")
        if any(k not in ctx for k in required):
            print("[UI] IK solve rejected | missing ik_context fields")
            self.state.set_ik_status(running=False, converged=False, failed=True, err_m=float("inf"), msg="missing IK context")
            return

        self.state.set_ik_status(running=True, converged=False, failed=False, err_m=float("inf"), msg="solving")

        def _worker() -> None:
            try:
                current_seed = np.array([float(self.state.linear), float(self.state.roll), float(self.state.theta1), float(self.state.theta2)], dtype=float)
                result = ik_pipeline.solve_then_align(
                    target_world=target,
                    target_dir_world=np.array([self.state.target_vx, self.state.target_vy, self.state.target_vz], dtype=float),
                    context=ctx,
                    position_tol_m=float(self._ik_cfg.tol),
                    max_iters=max(int(self._ik_cfg.max_iters), 1),
                    current_seed=current_seed,
                )
                if result.success and result.q is not None:
                    q = np.asarray(result.q, dtype=float).reshape(4)
                    refined_pos_err = float(result.position_error_m)
                    self.state.set_q(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
                    self.state.set_ik_solution(float(q[1]), float(q[2]), float(q[3]))
                    align_msg = str(result.reason)
                    if result.align_attempted:
                        align_msg = "%s | dir %.1f -> %.1f deg" % (
                            str(result.reason),
                            float(np.degrees(result.initial_direction_angle_rad)),
                            float(np.degrees(result.direction_angle_rad)),
                        )
                    self.state.set_ik_status(running=False, converged=True, failed=False, err_m=refined_pos_err, msg=align_msg)
                    if result.align_attempted:
                        print(
                            "[UI] Solve IK align | kept=%s | improved=%s | dir_deg %.2f -> %.2f"
                            % (
                                str(bool(result.align_position_kept)).lower(),
                                str(bool(result.align_direction_improved)).lower(),
                                float(np.degrees(result.initial_direction_angle_rad)),
                                float(np.degrees(result.direction_angle_rad)),
                            )
                        )
                    host_state = self._send_state_q_and_wait(timeout_s=2.0, source="ik", force=True)
                    if host_state is not None and (not bool(host_state.reply_ok)):
                        reason = str(host_state.reply_reason).strip() or "unknown host apply failure"
                        self.state.set_ik_status(
                            running=False,
                            converged=False,
                            failed=True,
                            err_m=refined_pos_err,
                            msg=f"IK converged but HW apply failed: {reason}",
                        )
                else:
                    print("[UI] IK solve failed | target=(%.3f, %.3f, %.3f) | err=%s" % (float(target[0]), float(target[1]), float(target[2]), float(result.position_error_m)))
                    self.state.set_ik_status(running=False, converged=False, failed=True, err_m=float(result.position_error_m), msg=str(result.reason))
            finally:
                self._ik_worker = None

        self._ik_worker = threading.Thread(target=_worker, daemon=True)
        self._ik_worker.start()

    def _calibration_current_for_axis(self, host_state: HostState, axis: str) -> Optional[int]:
        for key in self._calibration_current_keys.get(str(axis), (str(axis),)):
            value = host_state.motor_currents_ma.get(key)
            if value is not None:
                return abs(int(value))
        return None

    def _refresh_calibration_feedback(self, axis: str) -> tuple[Optional[HostState], Optional[int], float]:
        if self.client is None:
            return None, None, 0.0
        time.sleep(self._calibration_poll_s)
        host_state: Optional[HostState] = None
        current_val: Optional[int] = None
        peak_ma = 0.0
        for _ in range(int(self._calibration_feedback_reads)):
            host_state = self.client.refresh_state()
            if host_state is not None:
                sample = self._calibration_current_for_axis(host_state, axis)
                if sample is not None:
                    current_val = int(sample)
                    peak_ma = max(peak_ma, float(current_val))
            time.sleep(0.03)
        return host_state, current_val, peak_ma

    def _calibration_contact_threshold_ma(self, baseline_ma: float) -> float:
        relative = float(baseline_ma) + float(self._calibration_current_delta_ma)
        return float(max(float(self._calibration_current_min_threshold_ma), relative))

    def _calibration_is_contact(
        self,
        *,
        baseline_ma: float,
        threshold_ma: float,
        peak_ma: float,
        ema_ma: Optional[float],
    ) -> bool:
        reading = float(max(peak_ma, float(ema_ma if ema_ma is not None else 0.0)))
        if reading >= float(threshold_ma):
            return True
        return (reading - float(baseline_ma)) >= float(self._calibration_current_min_rise_ma)

    def _calibration_measure_baseline(self, axis: str, display_u: float) -> float:
        self.apply_partial_control_u({axis: float(display_u)})
        samples: list[float] = []
        for _ in range(int(self._calibration_baseline_samples)):
            _host_state, current_val, peak_ma = self._refresh_calibration_feedback(axis)
            if current_val is not None:
                samples.append(float(max(current_val, peak_ma)))
        if not samples:
            raise RuntimeError(f"missing {axis} current feedback")
        return float(sum(samples) / len(samples))

    def _calibration_update_ema(self, ema: Optional[float], current_val: float) -> float:
        if ema is None:
            return float(current_val)
        alpha = float(self._calibration_ema_alpha)
        return float(alpha * float(current_val) + (1.0 - alpha) * float(ema))

    def _calibration_axis_command_direction(self, axis: str) -> int:
        cfg = self.control_mapping()
        dirs = tuple(int(v) for v in cfg.command_direction)
        index = {"s1": 2, "s2": 3}.get(str(axis).strip().lower())
        if index is None:
            raise ValueError(f"unknown calibration axis: {axis}")
        return int(dirs[index])

    def _calibration_probe_display_direction(self, axis: str) -> int:
        # UI/display u step that reduces commanded seg value (respects command_direction).
        return -1 if self._calibration_axis_command_direction(axis) > 0 else +1

    def _calibration_probe_axis(
        self,
        axis: str,
        *,
        start_u: float,
        lo: float,
        hi: float,
        baseline_ma: float,
        threshold_ma: float,
    ) -> tuple[float, float, int]:
        step = float(self._calibration_step_u)
        direction = int(self._calibration_probe_display_direction(axis))
        display_u = float(start_u)
        ema: Optional[float] = None
        peak_seen_ma = 0.0
        baseline_ma = float(baseline_ma)
        self.apply_partial_control_u({axis: display_u})
        _host_state, current_val, peak_ma = self._refresh_calibration_feedback(axis)
        if _host_state is None or current_val is None:
            raise RuntimeError(f"missing {axis} current feedback")
        peak_seen_ma = max(peak_seen_ma, float(peak_ma))
        ema = self._calibration_update_ema(ema, float(peak_ma))
        while True:
            next_u = float(display_u) + float(direction) * step
            if direction < 0 and next_u < float(lo) - 1e-9:
                break
            if direction > 0 and next_u > float(hi) + 1e-9:
                break
            display_u = float(max(lo, min(hi, next_u)))
            self.apply_partial_control_u({axis: display_u})
            _host_state, current_val, peak_ma = self._refresh_calibration_feedback(axis)
            if _host_state is None or current_val is None:
                raise RuntimeError(f"missing {axis} current feedback")
            peak_seen_ma = max(peak_seen_ma, float(peak_ma))
            ema = self._calibration_update_ema(ema, float(peak_ma))
            if float(max(peak_ma, ema)) >= float(self._calibration_abort_current_ma):
                raise RuntimeError(f"{axis} current too high during calibration")
            if self._calibration_is_contact(
                baseline_ma=baseline_ma,
                threshold_ma=threshold_ma,
                peak_ma=peak_ma,
                ema_ma=ema,
            ):
                return float(display_u), float(ema), direction
        raise RuntimeError(
            f"no current rise on {axis} "
            f"(peak={peak_seen_ma:.0f}mA baseline={baseline_ma:.0f} thr={threshold_ma:.0f} end_u={display_u:.1f})"
        )

    def _calibration_release_axis(
        self,
        axis: str,
        *,
        contact_u: float,
        lo: float,
        hi: float,
        baseline_ma: float,
        threshold_ma: float,
        probe_direction: int,
        ema: float,
    ) -> float:
        step = float(self._calibration_step_u)
        release_dir = -int(probe_direction)
        display_u = float(contact_u)
        clear_count = 0
        release_display = float(contact_u)
        while True:
            next_u = float(display_u) + float(release_dir) * step
            if release_dir < 0 and next_u < float(lo) - 1e-9:
                break
            if release_dir > 0 and next_u > float(hi) + 1e-9:
                break
            display_u = float(max(lo, min(hi, next_u)))
            self.apply_partial_control_u({axis: display_u})
            _host_state, current_val, peak_ma = self._refresh_calibration_feedback(axis)
            if _host_state is None or current_val is None:
                raise RuntimeError(f"missing {axis} current feedback")
            ema = self._calibration_update_ema(ema, float(peak_ma))
            if float(max(peak_ma, ema)) >= float(self._calibration_abort_current_ma):
                raise RuntimeError(f"{axis} current too high during release")
            if not self._calibration_is_contact(
                baseline_ma=float(baseline_ma),
                threshold_ma=threshold_ma,
                peak_ma=peak_ma,
                ema_ma=ema,
            ):
                clear_count += 1
                if clear_count >= int(self._calibration_release_consecutive):
                    release_display = float(display_u)
                    break
            else:
                clear_count = 0
        if clear_count < int(self._calibration_release_consecutive):
            raise RuntimeError(f"release point not found on {axis}")
        return float(release_display)

    def start_calibration(self) -> None:
        if self._visual_busy():
            self.state.set_calibration_status(running=False, msg="busy")
            return
        if self.client is None:
            self.state.set_calibration_status(running=False, msg="no feedback client")
            return
        host_state = self.client.refresh_state()
        if host_state is None or not host_state.connected:
            self.state.set_calibration_status(running=False, msg="host offline")
            return
        if not bool(host_state.torque_enabled):
            self.state.set_calibration_status(running=False, msg="torque off")
            return
        if any(self._calibration_current_for_axis(host_state, axis) is None for axis in ("s1", "s2")):
            self.state.set_calibration_status(running=False, msg="missing motor currents (s1/s2)")
            return
        self.state.set_calibration_status(running=True, msg="calibrating")

        def _worker() -> None:
            try:
                cfg = self.control_mapping()
                host_u = host_state.u
                if host_u is not None:
                    display_u = self._actual_to_display_u(host_u)
                    display_vals = {"s1": float(display_u.u_s1), "s2": float(display_u.u_s2)}
                else:
                    current_u = self.current_control_u()
                    display_vals = {"s1": float(current_u.u_s1), "s2": float(current_u.u_s2)}
                hi = float(cfg.seg_u_max)
                lo = float(cfg.seg_u_min)
                for axis in ("s1", "s2"):
                    start_u = float(display_vals[axis])
                    self.state.set_calibration_status(running=True, msg=f"baseline {axis}")
                    baseline_ma = self._calibration_measure_baseline(axis, start_u)
                    threshold_ma = self._calibration_contact_threshold_ma(baseline_ma)
                    self.state.set_calibration_status(
                        running=True,
                        msg=f"probing {axis} (base={baseline_ma:.0f}mA thr={threshold_ma:.0f}mA)",
                    )
                    contact_u, ema, probe_dir = self._calibration_probe_axis(
                        axis,
                        start_u=start_u,
                        lo=lo,
                        hi=hi,
                        baseline_ma=baseline_ma,
                        threshold_ma=threshold_ma,
                    )
                    self.state.set_calibration_status(running=True, msg=f"releasing {axis}")
                    release_display = self._calibration_release_axis(
                        axis,
                        contact_u=contact_u,
                        lo=lo,
                        hi=hi,
                        baseline_ma=baseline_ma,
                        threshold_ma=threshold_ma,
                        probe_direction=probe_dir,
                        ema=ema,
                    )
                    self.state.set_u_offset(axis, float(release_display))
                    display_vals[axis] = 0.0
                    self.apply_partial_control_u({axis: 0.0})
                    self.state.set_calibration_status(running=True, msg=f"{axis} offset set to {release_display:.1f}")
                self.state.set_calibration_status(running=False, msg="calibration completed")
            except Exception as exc:
                self.state.set_calibration_status(running=False, msg=f"calibration failed: {exc}")
            finally:
                self._ik_worker = None

        self._ik_worker = threading.Thread(target=_worker, daemon=True)
        self._ik_worker.start()

    def start_ik_solve(self) -> None:
        target = np.array([self.state.target_x, self.state.target_y, self.state.target_z], dtype=float)
        self._start_position_solve(target)

    def request_ports(self) -> None:
        if self.client is not None:
            self.client.request_ports()

    def set_device(self, device: str) -> None:
        if self.client is not None:
            self.client.set_device(device)

    def disconnect_device(self) -> None:
        if self.client is not None:
            self.client.disconnect_device()

    def torque_on(self, *, resume: bool = False) -> None:
        if self.client is not None:
            self.client.torque_on(resume=bool(resume))

    def torque_off(self) -> None:
        if self.client is not None:
            self.client.torque_off()

    def perception_snapshot(self) -> Optional[PerceptionSnapshot]:
        cap = self._perception_capture
        return None if cap is None else cap.snapshot()

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
            base_sag = (
                dict(self.state.raw_sag_model)
                if isinstance(self.state.raw_sag_model, dict)
                else {}
            )
            model = self._pick_reach_model(base_sag)
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
        ctx = dict(self._ik_context)
        base_sag = dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {}
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
        d = self._unit_vec3(direction)
        standoff_vec = tgt - obj
        color = [1.0, 0.75, 0.12, 0.95] if bool(corrected) else [0.72, 1.0, 0.28, 0.95]
        line_color = [1.0, 0.55, 0.05, 0.65] if bool(corrected) else [0.72, 1.0, 0.28, 0.60]
        self.client.send_debug_markers(
            [
                {
                    "name": "ready_pose",
                    "frame": "world",
                    "pos": [float(v) for v in tgt],
                    "dir": [float(v) for v in d],
                    "color": color,
                    "radius": 0.014,
                    "ttl_ms": 30000,
                },
                {
                    "name": "ready_pose_standoff",
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
        ctx = dict(self._ik_context)
        ctx["sag_model"] = dict(sag_model)
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
                        ik_context=ctx,
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
                                context=ctx,
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
                            context=ctx,
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

        self._ik_worker = threading.Thread(target=_worker, name=str(profile_phase), daemon=True)
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
        if bool(pk.grasp_guided_enabled):
            if not self._run_grasp_trajectory_plan():
                return False
            return self._start_grasp_guided_execute(internal=internal)
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
        """Guided: plan trajectory then execute. Legacy: one-shot pre-contact IK."""
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

    def grasp_trajectory_planned(self) -> bool:
        """True when a kinematic grasp plan is stored and ready to execute."""
        return bool(self._grasp_plan_ready and self._grasp_planned_waypoints)

    def start_grasp_plan(self) -> None:
        """Compute and visualize the kinematic grasp trajectory (no robot motion)."""
        pk = self._pick_config_effective()
        if not bool(pk.grasp_guided_enabled):
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="grasp plan requires grasp_guided_enabled",
            )
            return
        if self.state.ik_running or self._visual_busy():
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="busy",
            )
            return
        self._pick_stop_event.clear()
        self._reset_grasp_guided_state()
        self._run_grasp_trajectory_plan()

    def start_grasp_execute(self) -> None:
        """Execute a previously planned grasp trajectory."""
        pk = self._pick_config_effective()
        if not bool(pk.grasp_guided_enabled):
            self.start_grasp()
            return
        if self.state.ik_running or self._visual_busy():
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="busy",
            )
            return
        if not self.grasp_trajectory_planned():
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="grasp execute | no plan (run Grasp Plan first)",
            )
            return
        self._pick_stop_event.clear()
        self._start_grasp_guided_execute()

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
        object_tuple = tuple(float(v) for v in object_arr)
        tip_tuple = tuple(float(v) for v in tip)
        auto_dir = self._pick_auto_preferred_dir(object_tuple, tip_world=tip_tuple)
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
        ctx = dict(self._ik_context)
        ctx["sag_model"] = dict(base_sag)
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
                pk = self._pick_config_effective()
                resolve_dir = bool(pk.look_pose_resolve_dir)
                q: Optional[np.ndarray] = None
                target_arr: Optional[np.ndarray] = None
                look_dir_used: Optional[np.ndarray] = None
                err_m = float("inf")
                align_msg = ""

                if bool(resolve_dir):
                    resolved = resolve_feasible_ready_pose(
                        object_world=object_tuple,
                        preferred_dir=preferred_arr,
                        standoff_m=float(pk.look_pose_standoff_m),
                        ik_context=ctx,
                        current_seed=q0,
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
                        return

                    q = np.asarray(resolved.q, dtype=float).reshape(4)
                    target_arr = np.asarray(resolved.resolved_target, dtype=float).reshape(3)
                    look_dir_used = np.asarray(resolved.resolved_dir, dtype=float).reshape(3)
                    err_m = float(resolved.position_error_m)
                    align_msg = (
                        "%s | tag=%s dir_err=%.1fdeg delta=%.1fdeg"
                        % (
                            str(resolved.reason),
                            str(resolved.candidate_tag),
                            float(np.degrees(resolved.direction_angle_rad)),
                            float(resolved.user_dir_delta_deg),
                        )
                    )
                else:
                    try:
                        target_tuple = compute_ready_pose_target(
                            object_tuple,
                            tuple(float(v) for v in preferred_arr),
                            standoff_m=float(pk.look_pose_standoff_m),
                        )
                    except ValueError as exc:
                        self.state.set_pick_status(
                            running=False,
                            failed=True,
                            phase=ObjectPickPhase.FAILED.value,
                            msg=str(exc),
                        )
                        return
                    target_arr = np.asarray(target_tuple, dtype=float).reshape(3)
                    look_dir_used = preferred_arr
                    if timing is not None:
                        timing.ik_calls += 1
                        with timing.span("resolve_single"):
                            result = ik_pipeline.solve_then_align(
                                target_world=target_arr,
                                target_dir_world=look_dir_used,
                                context=ctx,
                                position_tol_m=float(self._ik_cfg.tol),
                                max_iters=max(int(self._ik_cfg.max_iters), 1),
                                current_seed=q0,
                                timing=timing,
                                **self._ik_align_kwargs(force_full=True),
                            )
                        timing.resolve_reason = "single_solve"
                        timing.candidates_evaluated = 1
                    else:
                        result = ik_pipeline.solve_then_align(
                            target_world=target_arr,
                            target_dir_world=look_dir_used,
                            context=ctx,
                            position_tol_m=float(self._ik_cfg.tol),
                            max_iters=max(int(self._ik_cfg.max_iters), 1),
                            current_seed=q0,
                            **self._ik_align_kwargs(force_full=True),
                        )
                    if result.success and result.q is not None:
                        q = np.asarray(result.q, dtype=float).reshape(4)
                        err_m = float(result.position_error_m)
                        align_msg = str(result.reason)
                        if result.align_attempted:
                            align_msg = "%s | dir %.1f -> %.1f deg" % (
                                str(result.reason),
                                float(np.degrees(result.initial_direction_angle_rad)),
                                float(np.degrees(result.direction_angle_rad)),
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
                            msg="look IK failed | " + str(result.reason),
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

                look_tuple = tuple(float(v) for v in look_dir_used)
                view_tuple = tuple(float(v) for v in target_arr)
                self.state.set_target(float(target_arr[0]), float(target_arr[1]), float(target_arr[2]))
                self.state.set_target_dir(
                    float(look_dir_used[0]),
                    float(look_dir_used[1]),
                    float(look_dir_used[2]),
                )
                self._pick_resolved_ready_dir_world = look_tuple
                self._pick_resolved_ready_pose_world_xyz = view_tuple
                self._apply_ik_solution_to_host(
                    q,
                    ik_target=target_arr,
                    ik_target_dir=look_dir_used,
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
                self._pick_look_object_world_xyz = object_tuple
                self._pick_look_ready_pose_world_xyz = view_tuple
                self._pick_look_tip_world_xyz = tip_tuple
                self._pick_look_dir_world = look_tuple
                self._pick_initial_object_world_xyz = object_tuple
                self._pick_initial_ready_pose_world_xyz = view_tuple
                self._pick_frozen_world_xyz = object_tuple
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
                            float(pk.look_pose_standoff_m) * 1000.0,
                        )
                    ),
                )
                success = True
                print(
                    "[Pick] look done | object=(%.3f, %.3f, %.3f) view_pose=(%.3f, %.3f, %.3f) "
                    "look_dir=(%.3f, %.3f, %.3f) standoff=%.0fmm dir_err=%.1fdeg"
                    % (
                        float(object_arr[0]),
                        float(object_arr[1]),
                        float(object_arr[2]),
                        float(target_arr[0]),
                        float(target_arr[1]),
                        float(target_arr[2]),
                        float(look_dir_used[0]),
                        float(look_dir_used[1]),
                        float(look_dir_used[2]),
                        float(pk.look_pose_standoff_m) * 1000.0,
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

        self._ik_worker = threading.Thread(target=_worker, name="look", daemon=True)
        self._ik_worker.start()

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

    def _latch_pick_frozen_world(self) -> None:
        self._pick_frozen_world_xyz = self._pick_frozen_world()

    def _on_perception_snapshot(self, snap: PerceptionSnapshot) -> None:
        world_xyz = snap.p_world
        if bool(self.state.pick_running):
            world_xyz = self._pick_frozen_world()
        self.state.set_perception_status(
            running=bool(snap.running),
            failed=bool(snap.failed),
            msg=str(snap.status_msg),
            frame_idx=int(snap.frame_idx),
            label=str(snap.label),
            confidence=float(snap.confidence),
            camera_xyz=snap.p_camera,
            world_xyz=world_xyz,
            tracker_phase=str(snap.tracker_phase),
            track_ok_frames=int(snap.track_ok_frames),
            image_scale=float(snap.image_scale),
            bbox_wh=tuple(snap.bbox_wh),
            tracker_backend=str(snap.tracker_backend),
        )

    def _pick_config_effective(self) -> PickConfig:
        """Panel/runtime overrides on top of loaded ``config.ini`` pick settings."""
        pk = self._pick_cfg
        return replace(
            pk,
            target_scale=float(self.state.visual_target_scale),
            scale_tol=float(self.state.visual_scale_tol),
            center_tol=float(self.state.visual_center_tol),
            target_uv_u=float(self.state.visual_target_uv_u),
            target_uv_v=float(self.state.visual_target_uv_v),
            ready_pose_standoff_m=float(self.state.visual_ready_distance_m),
            look_pose_standoff_m=float(self.state.visual_look_distance_m),
        )

    def _pick_config_for_aim(self) -> PickConfig:
        """Stricter UV tolerance for Aim finish (centroid near target crosshair)."""
        pk = self._pick_config_effective()
        aim_tol = float(max(0.01, float(self._pick_cfg.aim_center_tol)))
        return replace(pk, center_tol=aim_tol)

    def _pick_config_for_grasp(self) -> PickConfig:
        """UV tolerance for guided grasp waypoints (looser than Aim finish)."""
        pk = self._pick_config_effective()
        raw_tol = float(pk.grasp_uv_center_tol)
        if raw_tol <= 0.0:
            # 0 = auto: between aim and pick center — post-IK roll/seg recovery needs slack.
            tol = float(
                max(
                    0.04,
                    min(float(pk.approach_loose_center_tol) * 0.6, float(pk.center_tol) * 0.5),
                )
            )
        else:
            tol = float(max(0.01, raw_tol))
        return replace(pk, center_tol=tol)

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
        direction = ControlService._unit_vec3(approach_dir)
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
        return ControlService._grasp_object_standoff_m(
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
        look = ControlService._grasp_look_at_dir(tip, object_world)
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
        wp_dist = ControlService._grasp_axial_distance(
            waypoint.position_world,
            nominal_world,
            approach_dir,
        )
        tip_dist = ControlService._grasp_axial_distance(
            tip_world,
            nominal_world,
            approach_dir,
        )
        return wp_dist > tip_dist + 1e-4

    def _reset_grasp_guided_state(self) -> None:
        self._grasp_waypoint_idx = 0
        self._grasp_online_sag_model = None
        self._grasp_nominal_dir = None
        self._grasp_trajectory_nominal_pose = None
        self._grasp_planned_waypoints = []
        self._grasp_plan_ready = False
        self._grasp_plan_traj_start = None
        self._grasp_plan_object_world = None
        self._grasp_plan_look_anchor = None

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

    def _grasp_visual_recover_supported(self) -> bool:
        """True when live perception can close the post-IK UV aim loop."""
        pk = self._pick_config_effective()
        if bool(pk.grasp_skip_aim_recover_in_mock):
            mode = str(self._perception_cfg.mode).strip().lower()
            if mode == "mock" or not bool(self._use_hardware):
                return False
        cap = self._perception_capture
        if cap is None or not cap.is_running():
            return False
        return True

    def _grasp_feasible_plan_callbacks(
        self,
        *,
        sag_model: dict[str, Any],
        timing: Optional[PickTimingCollector] = None,
    ) -> tuple[Any, Any]:
        """Return (ik_fn, fk_fn) for grasp trajectory feasibility filtering."""
        self.refresh_ik_context()
        ctx = dict(self._ik_context)
        ctx["sag_model"] = dict(sag_model)
        ik_kwargs = {
            "context": ctx,
            "position_tol_m": max(float(self._ik_cfg.tol), 1e-4),
            "max_iters": max(int(self._ik_cfg.max_iters), 1),
            **self._grasp_look_at_ik_kwargs(),
        }
        model = self._pick_reach_model(sag_model)
        ik_success = {"n": 0}

        def ik_fn(**kwargs: Any) -> Any:
            merged = dict(ik_kwargs)
            merged.update(kwargs)
            if timing is not None:
                timing.ik_calls += 1
                merged["timing"] = timing
            result = ik_pipeline.solve_then_look_at_tweak(**merged)
            if timing is not None and bool(getattr(result, "success", False)):
                ik_success["n"] += 1
            return result

        def fk_fn(q: np.ndarray) -> Any:
            q4 = np.asarray(q, dtype=float).reshape(4)
            pos = np.asarray(model.grasp_position(q4), dtype=float).reshape(3)
            direc = np.asarray(model.grasp_direction(q4), dtype=float).reshape(3)
            return type(
                "FkTip",
                (),
                {
                    "position_world": (
                        float(pos[0]),
                        float(pos[1]),
                        float(pos[2]),
                    ),
                    "direction_world": (
                        float(direc[0]),
                        float(direc[1]),
                        float(direc[2]),
                    ),
                },
            )()

        ik_fn.ik_success_counter = ik_success  # type: ignore[attr-defined]
        return ik_fn, fk_fn

    def _grasp_wait_waypoint_settle(
        self,
        *,
        q_cmd: np.ndarray,
        host_state: Optional[HostState],
        label: str,
        settle_s: float,
        settle_timeout_s: float,
    ) -> Optional[HostState]:
        """Wait for commanded q to settle, then dwell before the next waypoint."""
        dwell = float(max(settle_s, 0.0))
        timeout = float(max(settle_timeout_s, 0.0))
        if dwell <= 1e-6 and timeout <= 1e-6:
            return host_state

        settled = False
        if timeout > 1e-6:
            settled_state = self._wait_until_q_settled(
                q_cmd,
                timeout_s=timeout,
            )
            if settled_state is not None:
                host_state = settled_state
                settled = True

        if dwell > 1e-6:
            time.sleep(dwell)
            if self.client is not None:
                host_state = self.client.refresh_state()

        print(
            "[Grasp] %s | settle | q_ok=%s dwell=%.2fs"
            % (str(label), str(bool(settled)).lower(), dwell)
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
        look_ref = self._grasp_look_at_dir(tip, object_world)
        axial_to_object = float(np.dot(obj - tip, fk_dir))
        if axial_to_object <= 1e-4:
            return 0.0, 0.0
        try:
            desired = np.asarray(
                compute_ready_pose_target(
                    tuple(float(v) for v in obj),
                    tuple(float(v) for v in fk_dir),
                    standoff_m=float(axial_to_object),
                ),
                dtype=float,
            ).reshape(3)
        except ValueError:
            return 0.0, 0.0
        drift = desired - tip
        prepared = prepare_sag_drift_input(
            drift_world=drift,
            axis_world=fk_dir,
            reference_dir=look_ref,
            max_dir_error_deg=float(pk.sag_drift_max_dir_error_deg),
            max_lateral_m=float(pk.sag_drift_max_lateral_m),
            axial_only=bool(pk.sag_drift_axial_only),
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
        self.refresh_ik_context()
        ctx = dict(self._ik_context)
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
        max_micro_steps: int = 5,
    ) -> tuple[bool, Optional[VisualObservation], str]:
        current_obs = obs
        center_tol = float(cfg.center_tol)
        stall = ""
        for _ in range(max(1, int(max_micro_steps))):
            conv = evaluate_pick_convergence(current_obs, cfg=cfg)
            if bool(conv.center_ok):
                return True, current_obs, stall
            err_mag = max(abs(float(conv.u_err)), abs(float(conv.v_err)))
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
        max_micro_steps: int = 10,
        label: str = "",
    ) -> tuple[bool, Optional[VisualObservation], Optional[HostState]]:
        """Aim recover: re-center UV after a planned waypoint IK (sag miss)."""
        if self.client is not None:
            host_state = self.client.refresh_state()
        obs = self.current_visual_observation(host_state)
        if obs is None:
            return False, None, host_state
        u0, v0, _, _ = self._visual_uv_errors(obs)
        err0 = max(abs(float(u0)), abs(float(v0)))
        tol = float(max(cfg.center_tol, 1e-3))
        steps = max(
            int(max_micro_steps),
            int(np.ceil(err0 / tol * 3.0)) + 3,
        )
        steps = min(steps, 45)
        centered_ok, obs, stall = self._grasp_uv_center_until_tol(
            obs,
            cfg=cfg,
            max_micro_steps=int(steps),
        )
        if label and obs is not None:
            u_d, v_d, _, _ = self._visual_uv_errors(obs)
            extra = (" | stall=%s" % str(stall)) if stall else ""
            print(
                "[Grasp] %s | aim recover | centered=%s steps=%d uv=(%+.3f,%+.3f)%s"
                % (
                    str(label),
                    str(bool(centered_ok)).lower(),
                    int(steps),
                    float(u_d),
                    float(v_d),
                    extra,
                )
            )
        return bool(centered_ok), obs, host_state

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
        ctx = dict(self._ik_context)
        ctx["sag_model"] = dict(sag_model)
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

        object_world = self._grasp_plan_object_world or self._pick_grasp_object_world()
        ik_kwargs = self._grasp_look_at_ik_kwargs()
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
        result = ik_pipeline.solve_then_look_at_tweak(**ik_call)
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
    ) -> tuple[bool, Optional[HostState]]:
        """Align grasp axis toward ``approach_dir`` without advancing the grasp point."""
        if self.client is None or host_state is None or host_state.q is None:
            return False, host_state
        try:
            model = self._pick_reach_model(sag_model=sag_model)
            q0 = self._q_array_from_state(host_state)
            tip0 = np.asarray(model.grasp_position(q0), dtype=float).reshape(3)
            target_dir = self._unit_vec3(approach_dir)
        except Exception as exc:
            print(f"[Grasp] {label} | align precompute failed: {exc}")
            return False, host_state

        self.refresh_ik_context()
        ctx = dict(self._ik_context)
        ctx["sag_model"] = dict(sag_model)
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
                "[Grasp] %s | align IK failed | reason=%s err=%.4fm"
                % (str(label), str(result.reason), float(result.position_error_m))
            )
            return False, host_state

        q1 = np.asarray(result.q, dtype=float).reshape(4)
        host_state = self._apply_ik_solution_to_host(
            q1,
            ik_target=tip0,
            ik_target_dir=np.asarray(target_dir, dtype=float).reshape(3),
            err_m=float(result.position_error_m),
            status_msg=f"{label} | align",
            timeout_s=3.0,
            sag_model_override=dict(sag_model),
        )
        return True, host_state

    def _grasp_cartesian_advance_along_dir(
        self,
        distance_m: float,
        approach_dir: np.ndarray,
        *,
        sag_model: dict[str, Any],
        host_state: Optional[HostState] = None,
        label: str = "grasp waypoint",
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
        dir_hold = np.asarray(model.grasp_direction(q0), dtype=float).reshape(3)

        self.refresh_ik_context()
        ctx = dict(self._ik_context)
        ctx["sag_model"] = dict(sag_model)
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
        host_state = self._apply_ik_solution_to_host(
            q1,
            ik_target=target,
            ik_target_dir=dir_hold,
            err_m=float(result.position_error_m),
            status_msg=align_msg,
            timeout_s=3.0,
            sag_model_override=dict(sag_model),
        )
        if host_state is not None and (not bool(host_state.reply_ok)):
            return False, 0.0, q1, host_state
        return True, max(0.0, travel), q1, host_state

    def _grasp_advance_waypoint_ik(
        self,
        *,
        tip_world: tuple[float, float, float],
        nominal_world: tuple[float, float, float],
        approach_dir: np.ndarray,
        step_m: float,
        blind_start_m: float,
        sag_model: dict[str, Any],
        host_state: Optional[HostState],
        waypoint_idx: int,
    ) -> tuple[bool, Optional[np.ndarray], Optional[HostState]]:
        dist = self._grasp_axial_distance(tip_world, nominal_world, approach_dir)
        margin = max(0.0, float(dist) - float(blind_start_m))
        travel_m = min(float(step_m), float(margin))
        if travel_m <= 1e-6:
            return True, None, host_state
        label = "grasp waypoint %d" % int(waypoint_idx)
        ok, travel_actual, q_cmd, host_state = self._grasp_cartesian_advance_along_dir(
            travel_m,
            approach_dir,
            sag_model=sag_model,
            host_state=host_state,
            label=label,
        )
        if not ok or q_cmd is None:
            return False, None, host_state
        travel_for_settle = float(max(0.0, travel_actual))
        reached, _, host_state = self._wait_until_grasp_target_reached(
            target_world=np.asarray(
                np.asarray(tip_world, dtype=float).reshape(3)
                + self._unit_vec3(approach_dir) * travel_for_settle,
                dtype=float,
            ),
            q_cmd=q_cmd,
            sag_model=sag_model,
            timeout_s=5.0,
            position_tol_m=max(float(self._ik_cfg.tol), 0.012),
        )
        if not bool(reached):
            # waypoint 단계에서는 IK 전진 자체가 성공했으면 다음 waypoint로 진행한다.
            # (FK settle gate는 travel_m(계획)과 travel_actual(실측) 불일치에 민감할 수 있음)
            print("[Grasp] %s | settle timeout (continue)" % str(label))
            return True, q_cmd, host_state
        return True, q_cmd, host_state

    def _grasp_blind_final_approach(
        self,
        *,
        object_world: tuple[float, float, float],
        blind_approach_m: float,
        sag_model: dict[str, Any],
        host_state: Optional[HostState],
        grasp_standoff_m: float = 0.0,
        blind_start_m: float = 0.06,
    ) -> tuple[bool, Optional[np.ndarray], Optional[HostState], tuple[float, float, float]]:
        """Blind look-at steps until pre-contact standoff is within reach tolerance."""
        obj_tuple = tuple(float(v) for v in object_world)
        standoff_target = float(max(grasp_standoff_m, 0.0))
        step_m = float(max(blind_approach_m, 0.005))
        reach_tol_m = max(float(self._ik_cfg.tol), 0.003)
        max_steps = max(
            1,
            int(math.ceil(float(max(blind_start_m, step_m)) / step_m)) + 2,
        )
        q_cmd: Optional[np.ndarray] = None
        target_world = obj_tuple

        for blind_idx in range(max_steps):
            tip = self._pick_current_tip_world(host_state=host_state)
            if tip is None:
                return False, q_cmd, host_state, target_world
            remaining = self._grasp_approach_remaining_m(
                tip,
                obj_tuple,
                standoff_target,
            )
            if remaining <= reach_tol_m + 1e-4:
                if q_cmd is None:
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
                    "[Grasp] blind done | step=%d remain=%.1fmm (pre-contact)"
                    % (int(blind_idx), float(remaining) * 1000.0)
                )
                return True, q_cmd, host_state, target_world

            travel_hi = min(step_m, max(0.0, remaining - reach_tol_m))
            if travel_hi < 1e-6:
                break
            look_dir = self._grasp_look_at_dir(tip, obj_tuple)
            wp_label = "grasp blind %d" % int(blind_idx + 1)
            ok = False
            q_step: Optional[np.ndarray] = None
            target_pos = np.asarray(tip, dtype=float).reshape(3)
            travel = float(travel_hi)
            for _bisect in range(5):
                if travel < 1e-4:
                    break
                target_pos = np.asarray(tip, dtype=float).reshape(3) + look_dir * travel
                look_target = self._grasp_look_at_dir(target_pos, obj_tuple)
                waypoint = GraspWaypoint(
                    position_world=(
                        float(target_pos[0]),
                        float(target_pos[1]),
                        float(target_pos[2]),
                    ),
                    direction_world=(
                        float(look_target[0]),
                        float(look_target[1]),
                        float(look_target[2]),
                    ),
                    standoff_m=float(
                        self._grasp_object_standoff_m(target_pos, obj_tuple)
                    ),
                )
                ok, q_step, host_state, _ = self._grasp_ik_to_waypoint(
                    waypoint=waypoint,
                    sag_model=sag_model,
                    host_state=host_state,
                    label=wp_label if _bisect == 0 else "%s | bisect" % wp_label,
                )
                if ok and q_step is not None:
                    break
                travel *= 0.5
            if not ok or q_step is None:
                print(
                    "[Grasp] %s | IK failed | remain=%.1fmm"
                    % (wp_label, float(remaining) * 1000.0)
                )
                return False, q_cmd, host_state, target_world
            q_cmd = q_step
            reached, _, host_state = self._wait_until_grasp_target_reached(
                target_world=target_pos,
                q_cmd=q_cmd,
                sag_model=sag_model,
                timeout_s=8.0,
                position_tol_m=reach_tol_m,
            )
            if not bool(reached):
                print(
                    "[Grasp] %s | settle failed | remain=%.1fmm"
                    % (wp_label, float(remaining) * 1000.0)
                )
                return False, q_cmd, host_state, target_world
            tip_after = self._pick_current_tip_world(host_state=host_state)
            if tip_after is not None:
                remain_after = self._grasp_approach_remaining_m(
                    tip_after,
                    obj_tuple,
                    standoff_target,
                )
                print(
                    "[Grasp] %s | travel=%.0fmm remain=%.1fmm→%.1fmm"
                    % (
                        wp_label,
                        float(travel) * 1000.0,
                        float(remaining) * 1000.0,
                        float(remain_after) * 1000.0,
                    )
                )

        tip = self._pick_current_tip_world(host_state=host_state)
        if tip is None:
            return False, q_cmd, host_state, target_world
        remaining = self._grasp_approach_remaining_m(
            tip,
            obj_tuple,
            standoff_target,
        )
        if remaining > reach_tol_m + 1e-4:
            print(
                "[Grasp] blind abort | pre-contact not reached | remain=%.1fmm tol=%.1fmm"
                % (float(remaining) * 1000.0, float(reach_tol_m) * 1000.0)
            )
            return False, q_cmd, host_state, target_world
        target_world = self._grasp_precontact_from_tip(
            tip,
            obj_tuple,
            standoff_target,
        )
        return True, q_cmd, host_state, target_world

    def _run_grasp_trajectory_plan(self) -> bool:
        """Build kinematic grasp waypoints from current FK tip; store plan for execute."""
        self._grasp_planned_waypoints = []
        self._grasp_plan_ready = False
        self._grasp_plan_traj_start = None
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
                msg="grasp plan | missing object (run Look or enable perception)",
            )
            return False
        dir_tuple = self._grasp_aim_latched_direction(object_world=object_world)
        if dir_tuple is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="grasp plan | cannot infer approach direction",
            )
            return False

        pk = self._pick_config_effective()
        step_m = float(max(pk.grasp_waypoint_step_m, 0.005))
        blind_start_m = float(max(pk.grasp_blind_start_m, 0.0))
        max_waypoints = max(1, int(pk.grasp_max_waypoints))
        feasible_dir_tol_deg = float(max(pk.grasp_waypoint_max_dir_error_deg, 0.0))
        feasible_drift_tol_deg = float(max(pk.grasp_waypoint_max_approach_drift_deg, 0.0))
        standoff_m = float(max(pk.grasp_standoff_m, 0.0))
        dir_u = self._unit_vec3(dir_tuple)
        dir3 = (float(dir_u[0]), float(dir_u[1]), float(dir_u[2]))
        live_object = self._pick_grasp_object_world() or object_world
        look_anchor = self._pick_grasp_trajectory_start_position()
        nominal_world = self._pick_grasp_trajectory_end_position(
            live_object,
            dir_u,
            standoff_m=standoff_m,
        )

        self._grasp_nominal_dir = dir3
        self._grasp_trajectory_nominal_pose = tuple(float(v) for v in nominal_world)
        self._grasp_plan_object_world = tuple(float(v) for v in live_object)
        self._grasp_plan_look_anchor = (
            tuple(float(v) for v in look_anchor) if look_anchor is not None else None
        )

        sag_model = self._pick_grasp_sag_model()
        plan_timing, plan_stats, plan_t0 = self._begin_grasp_plan_profile()
        ik_fn, fk_fn = self._grasp_feasible_plan_callbacks(
            sag_model=sag_model,
            timing=plan_timing,
        )
        host_state = self.client.refresh_state()
        try:
            q_seed = self._q_array_from_state(host_state)
        except Exception:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="grasp plan | seed q unavailable",
            )
            return False

        fk_seed = fk_fn(q_seed)
        traj_start = tuple(float(v) for v in fk_seed.position_world)
        self._grasp_plan_traj_start = traj_start
        t_geom_s = 0.0
        if plan_timing is not None:
            with plan_timing.span("geom_plan"):
                geom_plan = plan_grasp_approach_trajectory(
                    start_position=traj_start,
                    end_position=nominal_world,
                    start_direction=dir3,
                    end_direction=dir3,
                    object_world=live_object,
                    step_m=step_m,
                    blind_start_m=blind_start_m,
                    grasp_standoff_m=standoff_m,
                    max_waypoints=max_waypoints,
                )
            t_geom_s = plan_timing.get("geom_plan")
        else:
            geom_plan = plan_grasp_approach_trajectory(
                start_position=traj_start,
                end_position=nominal_world,
                start_direction=dir3,
                end_direction=dir3,
                object_world=live_object,
                step_m=step_m,
                blind_start_m=blind_start_m,
                grasp_standoff_m=standoff_m,
                max_waypoints=max_waypoints,
            )
        t_kinematic_s = 0.0
        if plan_timing is not None:
            with plan_timing.span("kinematic_plan"):
                self._grasp_planned_waypoints = plan_grasp_feasible_trajectory(
                    start_position=traj_start,
                    end_position=nominal_world,
                    start_direction=dir3,
                    end_direction=dir3,
                    object_world=live_object,
                    q_seed=q_seed,
                    step_m=step_m,
                    blind_start_m=blind_start_m,
                    ik_fn=ik_fn,
                    fk_fn=fk_fn,
                    grasp_standoff_m=standoff_m,
                    max_waypoints=max_waypoints,
                    max_dir_error_deg=feasible_dir_tol_deg,
                    max_approach_drift_deg=feasible_drift_tol_deg,
                    stats=plan_stats,
                )
            t_kinematic_s = plan_timing.get("kinematic_plan")
        else:
            self._grasp_planned_waypoints = plan_grasp_feasible_trajectory(
                start_position=traj_start,
                end_position=nominal_world,
                start_direction=dir3,
                end_direction=dir3,
                object_world=live_object,
                q_seed=q_seed,
                step_m=step_m,
                blind_start_m=blind_start_m,
                ik_fn=ik_fn,
                fk_fn=fk_fn,
                grasp_standoff_m=standoff_m,
                max_waypoints=max_waypoints,
                max_dir_error_deg=feasible_dir_tol_deg,
                max_approach_drift_deg=feasible_drift_tol_deg,
            )
        plan_n = len(self._grasp_planned_waypoints)
        geom_n = len(geom_plan)
        path_len = trajectory_path_length_m(
            self._grasp_planned_waypoints,
            start_position=traj_start,
        )
        start_standoff = self._grasp_object_standoff_m(traj_start, live_object)
        start_remain = self._grasp_approach_remaining_m(
            traj_start,
            live_object,
            standoff_m,
        )
        print(
            "[Grasp] plan | kinematic feasible=%d geom_ref=%d path=%.0fmm "
            "standoff %.0f→%.0fmm remain=%.0fmm tip=(%.3f,%.3f,%.3f)"
            % (
                int(plan_n),
                int(geom_n),
                float(path_len) * 1000.0,
                start_standoff * 1000.0,
                standoff_m * 1000.0,
                start_remain * 1000.0,
                float(traj_start[0]),
                float(traj_start[1]),
                float(traj_start[2]),
            )
        )
        ik_ok_n = int(getattr(ik_fn, "ik_success_counter", {}).get("n", 0))
        self._finish_grasp_plan_profile(
            timing=plan_timing,
            stats=plan_stats,
            t0=plan_t0,
            t_geom_s=t_geom_s,
            t_kinematic_s=t_kinematic_s,
            waypoints=int(plan_n),
            geom_waypoints=int(geom_n),
            ik_success=ik_ok_n,
            success=bool(plan_n > 0),
        )
        if plan_n <= 0:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="grasp plan | no feasible kinematic waypoints",
            )
            return False

        self._grasp_plan_ready = True
        self._send_grasp_trajectory_markers(
            start_position=traj_start,
            end_position=tuple(float(v) for v in nominal_world),
            object_world=tuple(float(v) for v in live_object),
            waypoints=list(self._grasp_planned_waypoints),
            look_anchor_position=self._grasp_plan_look_anchor,
        )
        plan_msg = "grasp plan | %d waypoints | path=%.0fmm" % (
            int(plan_n),
            float(path_len) * 1000.0,
        )
        self.state.set_pick_status(
            running=False,
            failed=False,
            phase=ObjectPickPhase.GRASP_PLAN.value,
            msg=plan_msg,
        )
        print("[Grasp] %s" % plan_msg)
        return True

    def _start_grasp_guided_execute(self, *, internal: bool = False) -> bool:
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
        if not self.grasp_trajectory_planned():
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="grasp execute | no plan (run Grasp Plan first)",
            )
            return False
        if self._grasp_nominal_dir is None or self._grasp_plan_object_world is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="grasp execute | plan context missing",
            )
            return False

        prev_worker = self._ik_worker
        if prev_worker is not None and prev_worker.is_alive():
            print("[Grasp] stopping previous guided worker")
            self._pick_stop_event.set()
            prev_worker.join(timeout=2.0)
        self._pick_stop_event.clear()

        self._grasp_waypoint_idx = 0
        base_sag = self._pick_grasp_sag_model()
        self._grasp_online_sag_model = dict(base_sag) if base_sag else None

        self.state.set_pick_status(
            running=True,
            failed=False,
            phase=ObjectPickPhase.GRASP_APPROACH.value,
            msg="grasp execute starting",
        )
        self.state.set_ik_status(
            running=True,
            converged=False,
            failed=False,
            err_m=float("inf"),
            msg="grasp execute",
        )

        object_world = self._grasp_plan_object_world
        approach_dir = np.asarray(self._grasp_nominal_dir, dtype=float).reshape(3)

        def _worker() -> None:
            self._run_grasp_guided_execute_worker(
                object_world=object_world,
                approach_dir=approach_dir,
            )

        self._ik_worker = threading.Thread(
            target=_worker,
            name="grasp-guided-exec",
            daemon=True,
        )
        self._ik_worker.start()
        return True

    def _run_grasp_guided_execute_worker(
        self,
        *,
        object_world: tuple[float, float, float],
        approach_dir: np.ndarray,
    ) -> None:
        pk = self._pick_config_effective()
        grasp_cfg = self._pick_config_for_grasp()
        step_m = float(max(pk.grasp_waypoint_step_m, 0.005))
        blind_start_m = float(max(pk.grasp_blind_start_m, 0.0))
        blind_approach_m = float(max(pk.grasp_blind_approach_m, 0.0))
        max_waypoints = max(1, int(pk.grasp_max_waypoints))
        waypoint_settle_s = float(max(pk.grasp_waypoint_settle_s, 0.0))
        waypoint_settle_timeout_s = float(max(pk.grasp_waypoint_settle_timeout_s, 0.0))
        feasible_dir_tol_deg = float(max(pk.grasp_waypoint_max_dir_error_deg, 0.0))
        feasible_drift_tol_deg = float(max(pk.grasp_waypoint_max_approach_drift_deg, 0.0))
        standoff_m = float(max(pk.grasp_standoff_m, 0.0))
        dir_u = self._unit_vec3(approach_dir)
        dir_tuple = (float(dir_u[0]), float(dir_u[1]), float(dir_u[2]))
        success = False
        traj_start = self._grasp_plan_traj_start
        look_anchor = self._grasp_plan_look_anchor
        if traj_start is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="grasp execute | plan start missing",
            )
            return
        try:
            if self._perception_capture is None or not self._perception_capture.is_running():
                self.start_perception_capture()

            print(
                "[Grasp] execute start | step=%.0fmm blind_start=%.0fmm standoff=%.0fmm "
                "max_wp=%d planned=%d settle=%.2fs"
                % (
                    step_m * 1000.0,
                    blind_start_m * 1000.0,
                    standoff_m * 1000.0,
                    int(max_waypoints),
                    int(len(self._grasp_planned_waypoints)),
                    waypoint_settle_s,
                )
            )

            host_state = self.client.refresh_state() if self.client is not None else None
            try:
                q_seed = self._q_array_from_state(host_state)
            except Exception:
                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg="grasp execute | seed q unavailable",
                )
                return

            feasible_queue_idx = 0
            for wp in range(max_waypoints):
                if self._pick_stop_event.is_set():
                    self.state.set_pick_status(
                        running=False,
                        failed=False,
                        phase=ObjectPickPhase.IDLE.value,
                        msg="grasp stopped",
                    )
                    return

                self._grasp_waypoint_idx = int(wp + 1)
                live_object = self._pick_grasp_object_world() or object_world
                nominal_world = self._pick_grasp_trajectory_end_position(
                    live_object,
                    dir_u,
                    standoff_m=standoff_m,
                )
                tip = self._pick_current_tip_world(host_state=host_state)
                if tip is None:
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg="grasp guided | tip FK unavailable",
                    )
                    return

                remain = self._grasp_approach_remaining_m(
                    tip,
                    live_object,
                    standoff_m,
                )
                if remain <= blind_start_m + 1e-4:
                    print(
                        "[Grasp] wp %d | blind threshold | remain=%.1fmm"
                        % (int(wp + 1), float(remain) * 1000.0)
                    )
                    break

                sag_model = self._pick_grasp_sag_model()

                planned_wp: GraspWaypoint | None = None
                if feasible_queue_idx < len(self._grasp_planned_waypoints):
                    planned_wp = self._grasp_planned_waypoints[feasible_queue_idx]
                else:
                    ik_fn, fk_fn = self._grasp_feasible_plan_callbacks(
                        sag_model=sag_model
                    )
                    try:
                        q_now = self._q_array_from_state(host_state)
                    except Exception:
                        q_now = q_seed
                    planned_wp = plan_grasp_feasible_next_waypoint(
                        start_position=tip,
                        end_position=nominal_world,
                        start_direction=dir_tuple,
                        end_direction=dir_tuple,
                        object_world=live_object,
                        q_seed=q_now,
                        step_m=step_m,
                        blind_start_m=blind_start_m,
                        ik_fn=ik_fn,
                        fk_fn=fk_fn,
                        grasp_standoff_m=standoff_m,
                        max_dir_error_deg=feasible_dir_tol_deg,
                        max_approach_drift_deg=feasible_drift_tol_deg,
                    )
                if planned_wp is None:
                    print(
                        "[Grasp] wp %d | no feasible waypoint | remain=%.1fmm"
                        % (int(wp + 1), float(remain) * 1000.0)
                    )
                    break

                wp_label = "wp %d/%d" % (int(wp + 1), int(max_waypoints))
                self._send_grasp_trajectory_markers(
                    start_position=traj_start,
                    end_position=tuple(float(v) for v in nominal_world),
                    object_world=tuple(float(v) for v in live_object),
                    waypoints=list(self._grasp_planned_waypoints),
                    highlight_idx=int(wp),
                    look_anchor_position=look_anchor,
                )
                ok, q_cmd, host_state, _ = self._grasp_ik_to_waypoint(
                    waypoint=planned_wp,
                    sag_model=sag_model,
                    host_state=host_state,
                    label="grasp %s" % wp_label,
                    seed_override=(
                        np.asarray(planned_wp.q_seed, dtype=float)
                        if planned_wp.q_seed is not None
                        else None
                    ),
                )
                if (not ok or q_cmd is None) and planned_wp.q_seed is not None:
                    ok, q_cmd, host_state, _ = self._grasp_ik_to_waypoint(
                        waypoint=planned_wp,
                        sag_model=sag_model,
                        host_state=host_state,
                        label="grasp %s | planned seed" % wp_label,
                        seed_override=np.asarray(planned_wp.q_seed, dtype=float),
                    )
                if not ok or q_cmd is None:
                    ik_fn, fk_fn = self._grasp_feasible_plan_callbacks(
                        sag_model=sag_model
                    )
                    try:
                        q_now = self._q_array_from_state(host_state)
                    except Exception:
                        q_now = q_seed
                    replanned = plan_grasp_feasible_next_waypoint(
                        start_position=tip,
                        end_position=nominal_world,
                        start_direction=dir_tuple,
                        end_direction=dir_tuple,
                        object_world=live_object,
                        q_seed=q_now,
                        step_m=step_m,
                        blind_start_m=blind_start_m,
                        ik_fn=ik_fn,
                        fk_fn=fk_fn,
                        grasp_standoff_m=standoff_m,
                        max_dir_error_deg=feasible_dir_tol_deg,
                        max_approach_drift_deg=feasible_drift_tol_deg,
                    )
                    if replanned is not None:
                        ok, q_cmd, host_state, _ = self._grasp_ik_to_waypoint(
                            waypoint=replanned,
                            sag_model=sag_model,
                            host_state=host_state,
                            label="grasp %s | replan" % wp_label,
                            seed_override=(
                                np.asarray(replanned.q_seed, dtype=float)
                                if replanned.q_seed is not None
                                else None
                            ),
                        )
                        if ok and q_cmd is not None:
                            planned_wp = replanned
                if not ok or q_cmd is None:
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg="grasp guided | waypoint IK failed",
                    )
                    return
                feasible_queue_idx += 1

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
                            "[Grasp] %s | aim recover | no observation (continue sag)"
                            % str(wp_label)
                        )
                else:
                    print(
                        "[Grasp] %s | aim recover skipped | mock/sim (IK+sag only)"
                        % str(wp_label)
                    )
                    if self.client is not None:
                        host_state = self.client.refresh_state()
                    obs = self.current_visual_observation(host_state)

                host_state = self.client.refresh_state() if self.client is not None else None
                self._grasp_update_online_sag_bias(
                    host_state=host_state,
                    object_world=live_object,
                    approach_dir=dir_u,
                    pk=pk,
                    label=wp_label,
                )

                host_state = self._grasp_wait_waypoint_settle(
                    q_cmd=q_cmd,
                    host_state=host_state,
                    label=wp_label,
                    settle_s=waypoint_settle_s,
                    settle_timeout_s=waypoint_settle_timeout_s,
                )

                tip = self._pick_current_tip_world(host_state=host_state)
                if tip is None:
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg="grasp guided | tip FK unavailable after aim",
                    )
                    return
                remain = self._grasp_approach_remaining_m(
                    tip,
                    live_object,
                    standoff_m,
                )
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
                    msg="grasp waypoint %d | remain=%.0fmm"
                    % (int(wp + 1), float(remain) * 1000.0),
                )

            self.stop_perception_capture()
            print("[Grasp] perception stopped | blind approach")

            live_object = self._pick_grasp_object_world() or object_world
            nominal_world = self._pick_grasp_trajectory_end_position(
                live_object,
                dir_u,
                standoff_m=standoff_m,
            )
            sag_model = self._pick_grasp_sag_model()
            host_state = self.client.refresh_state() if self.client is not None else None

            self.state.set_pick_status(
                running=True,
                failed=False,
                phase=ObjectPickPhase.GRASP.value,
                msg="grasp blind approach",
            )
            blind_ok, q_cmd, host_state, target_world = self._grasp_blind_final_approach(
                object_world=tuple(float(v) for v in live_object),
                blind_approach_m=blind_approach_m,
                sag_model=sag_model,
                host_state=host_state,
                grasp_standoff_m=standoff_m,
                blind_start_m=blind_start_m,
            )
            if not blind_ok or q_cmd is None:
                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg="grasp blind approach failed",
                )
                return

            object_tuple = tuple(float(v) for v in live_object)
            target_arr = np.asarray(target_world, dtype=float).reshape(3)
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
            )
            if not bool(closed_ok):
                return

            done_msg = "grasp done | guided waypoints=%d | %s" % (
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
            self._grasp_plan_ready = False
        finally:
            if not success and not self.state.pick_failed:
                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg="grasp execute failed",
                )
            self._ik_worker = None

    def _pick_reach_model(self, sag_model: Optional[dict[str, Any]] = None):
        from engine.iklib.kinematics import _ReachModel

        self.refresh_ik_context()
        limit = self._ik_context.get("limit")
        if limit is None:
            raise RuntimeError("ik context missing joint limit")
        ctx = dict(self._ik_context)
        if sag_model is not None:
            ctx["sag_model"] = dict(sag_model)
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
        from engine.iklib.kinematics import _forward_link_tf

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
        """Advance grasp point ``distance_m`` along EE local -Z via ``engine.ik.solve_then_align``."""
        delta = float(max(0.0, distance_m))
        if delta <= 1e-6:
            return 0.0
        try:
            sag_model = self._pick_final_sag_model()
            model = self._pick_reach_model(sag_model=sag_model)
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
        ctx = dict(self._ik_context)
        ctx["sag_model"] = dict(sag_model)
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
        step_scale = (
            1.0
            if v_only
            else float(max(min(float(self._pick_aim_step_scale), 1.0), 0.05))
            if coupled_axes
            else 1.0
        )
        seg_cap = float(cfg.center_seg_max) * step_scale
        roll_cap = float(cfg.center_roll_max) * step_scale
        if not u_over and not v_over:
            return current_u, "none", 0.0, 0.0

        err_mag = max(abs(float(u_delta)), abs(float(v_delta)))
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
                min_step = float(self._pick_aim_v_min_seg_step)
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
            self.start_perception_capture()

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
                    eps = float(self._pick_aim_progress_eps)
                    if (
                        self._pick_aim_best_uv_err is None
                        or err_mag < float(self._pick_aim_best_uv_err) - eps
                    ):
                        self._pick_aim_best_uv_err = float(err_mag)
                        self._pick_aim_stuck_iters = 0
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
                        elif self._pick_apply_fov_search_step(reason="aim_center_stuck"):
                            self._pick_aim_stuck_iters = 0
                            self._pick_aim_best_uv_err = None
                            print(
                                "[Aim] center_stuck | fov_search | delta=(%+.3f,%+.3f)"
                                % (float(u_d), float(v_d))
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
                    time.sleep(float(self._pick_aim_settle_s))

                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg="aim iteration limit",
                )
            finally:
                self._pick_worker = None

        self._pick_worker = threading.Thread(target=_worker, daemon=True)
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
        ctx = dict(self._ik_context)
        if isinstance(self._pick_equal_sag_model, dict) and self._pick_equal_sag_model:
            ctx["sag_model"] = dict(self._pick_equal_sag_model)
            sag_model_override = dict(self._pick_equal_sag_model)
        else:
            sag_model_override = (
                dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {}
            )
            ctx["sag_model"] = dict(sag_model_override)
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
            target_ik = target_tip + (np.asarray(grasp0, dtype=float).reshape(3) - tip_world)
        except Exception:
            target_ik = target_tip.copy()

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
                    target_world=target_ik,
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

        self._ik_worker = threading.Thread(target=_worker, daemon=True)
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
            self.start_perception_capture()

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

        self._pick_worker = threading.Thread(target=_worker, name="object-pick", daemon=True)
        self._pick_worker.start()

    def _publish_perception_to_host(
        self,
        *,
        object_camera_xyz: tuple[float, float, float],
        label: str,
        confidence: float,
        image_center_uv: tuple[float, float],
        image_scale: float,
        depth_valid: bool = True,
        object_world: Optional[tuple[float, float, float]] = None,
    ) -> Optional[tuple[float, float, float]]:
        if self.client is None:
            return None
        freeze_world = bool(self.state.pick_running)
        publish_depth = bool(depth_valid) and (
            not freeze_world or not bool(self._pick_equal_sag_attempted)
        )
        p_world = self.client.send_perception_observation(
            object_camera_xyz=object_camera_xyz,
            label=label,
            confidence=confidence,
            image_center_uv=image_center_uv,
            image_scale=image_scale,
            depth_valid=publish_depth,
            object_world=object_world,
        )
        if freeze_world:
            frozen = self._pick_frozen_world()
            return frozen if frozen is not None else p_world
        if p_world is not None:
            self._pick_frozen_world_xyz = tuple(p_world)
        return p_world

    def start_perception_capture(self, *, config: Optional[PerceptionConfig] = None) -> None:
        if self._perception_capture is not None and self._perception_capture.is_running():
            self.state.set_perception_status(running=True, failed=False, msg="already running")
            return
        if self.client is None:
            self.state.set_perception_status(running=False, failed=True, msg="no host client")
            return
        cfg = config or self._perception_cfg
        self._perception_cfg = cfg
        self.state.visual_target_label = str(cfg.target_label).strip()
        self._perception_capture = PerceptionCapture(
            cfg,
            publish_fn=self._publish_perception_to_host,
            on_snapshot=self._on_perception_snapshot,
            target_uv_fn=lambda: (
                float(self.state.visual_target_uv_u),
                float(self.state.visual_target_uv_v),
            ),
            mock_world_xyz_fn=self._mock_world_xyz_from_state,
        )
        self.state.set_perception_status(running=True, failed=False, msg="starting")
        self._perception_capture.start()

    def stop_perception_capture(self) -> None:
        cap = self._perception_capture
        if cap is not None:
            stopped = cap.stop()
            if not stopped:
                self.state.set_perception_status(running=True, failed=False, msg="stopping")
                return
        self._perception_capture = None
        self.state.set_perception_status(running=False, failed=False, msg="stopped")

    def refresh_perception_capture(self) -> None:
        cap = self._perception_capture
        if cap is None or not cap.is_running():
            self.state.set_perception_status(running=False, failed=True, msg="perception is not running")
            return
        if cap.request_refresh():
            self.state.set_perception_status(running=True, failed=False, msg="refresh requested (YOLO)")
        else:
            self.state.set_perception_status(running=False, failed=True, msg="refresh rejected")

    def update_perception_config(self, config: PerceptionConfig) -> None:
        self._perception_cfg = config
        self.state.visual_target_label = str(config.target_label).strip()

    def _mock_world_xyz_from_state(self) -> Optional[tuple[float, float, float]]:
        if str(self._perception_cfg.mode).strip().lower() != "mock":
            return None
        return self.state.mock_object_world_xyz()

    def set_mock_object_world(self, x: float, y: float, z: float) -> None:
        self.state.set_mock_object_world_xyz(float(x), float(y), float(z))

    def publish_mock_object_world(self) -> bool:
        """Push current mock object world XYZ to host (updates sim marker)."""
        if self.client is None:
            return False
        world_xyz = self.state.mock_object_world_xyz()
        camera_xyz = self.state.perception_camera_xyz
        if camera_xyz is None:
            camera_xyz = (0.0, 0.0, 0.65)
        label = str(self.state.perception_label).strip() or str(self.state.visual_target_label).strip() or "mock_object"
        confidence = float(self.state.perception_confidence)
        if confidence <= 0.0:
            confidence = 1.0
        image_scale = float(self.state.perception_image_scale)
        if image_scale <= 0.0:
            image_scale = float(self._pick_config_effective().target_scale)
        p_world = self._publish_perception_to_host(
            object_camera_xyz=tuple(float(v) for v in camera_xyz),
            label=label,
            confidence=confidence,
            image_center_uv=(0.0, 0.0),
            image_scale=image_scale,
            depth_valid=True,
            object_world=world_xyz,
        )
        ack_xyz = p_world if p_world is not None else world_xyz
        self.state.set_perception_status(
            running=bool(self.state.perception_running),
            failed=False,
            msg="mock object moved",
            world_xyz=ack_xyz,
            label=label,
            confidence=confidence,
            camera_xyz=camera_xyz,
        )
        return p_world is not None

    def close(self) -> None:
        self.stop_object_pick()
        self.stop_perception_capture()
        if self.client is not None:
            self.client.close()
