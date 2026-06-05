from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import replace
from typing import Any, Optional

import numpy as np

from engine import ik as ik_pipeline
from engine.iklib import kinematics as ik_kin
from engine.config_loader import IkConfig, PerceptionConfig, PickConfig
from engine.protocol import ControlU, SimMappingConfig, SimQ, control_u_to_sim_q, sim_q_to_control_u
from engine.sag_model import load_sag_model_json
from engine.visual_servoing.equal_sag_probe import (
    EqualSagEstimate,
    apply_equal_sag_offsets,
    estimate_equal_sag_from_ready_pose_drift,
)
from engine.visual_servoing.feasible_ready_pose import resolve_feasible_ready_pose
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
    ) -> None:
        self.state = state
        self.client = client
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
        self._pick_worker: Optional[threading.Thread] = None
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
        self._pick_resolved_ready_dir_world: Optional[tuple[float, float, float]] = None
        self._pick_resolved_ready_pose_world_xyz: Optional[tuple[float, float, float]] = None
        self._pick_equal_sag_estimate: Optional[EqualSagEstimate] = None
        self._pick_equal_sag_model: Optional[dict[str, Any]] = None
        self._pick_equal_sag_attempted = False
        self._pick_search_origin_u: Optional[ControlU] = None
        self._pick_search_step_index = 0
        self._pick_search_max_steps = 48
        self._pick_aim_step_scale = 0.35
        self._pick_aim_command_timeout_s = 1.2
        self._pick_aim_settle_s = 0.25
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
        stable_count = 0
        last_state: Optional[HostState] = None
        while time.time() < deadline:
            time.sleep(0.05)
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
            if (
                abs(float(q_now[0] - target[0])) <= float(linear_tol_m)
                and abs(float(q_now[1] - target[1])) <= float(angle_tol_rad)
                and abs(float(q_now[2] - target[2])) <= float(angle_tol_rad)
                and abs(float(q_now[3] - target[3])) <= float(angle_tol_rad)
            ):
                stable_count += 1
                if stable_count >= max(int(consecutive), 1):
                    return state
            else:
                stable_count = 0
        return last_state

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

    def _send_state_q_and_wait(
        self,
        *,
        timeout_s: float = 1.0,
        source: str = "ik",
        force: bool = False,
        sag_model_override: Optional[dict[str, Any]] = None,
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
        self.send_current_target(
            source=source,
            force=force,
            sag_model_override=sag_model_override,
        )
        host_state = self._wait_until_q_settled(q_cmd, timeout_s=float(timeout_s))
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
        if self.client is not None:
            self.client.send_ready_pose_meta(
                target_dir=(
                    float(self.state.visual_target_dir_x),
                    float(self.state.visual_target_dir_y),
                    float(self.state.visual_target_dir_z),
                ),
                standoff_m=float(self.state.visual_ready_distance_m),
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

    def _pick_requested_ready_direction(self) -> tuple[float, float, float]:
        return (
            float(self.state.visual_target_dir_x),
            float(self.state.visual_target_dir_y),
            float(self.state.visual_target_dir_z),
        )

    def _pick_ready_direction(self) -> tuple[float, float, float]:
        if self._pick_resolved_ready_dir_world is not None:
            return self._pick_resolved_ready_dir_world
        return self._pick_requested_ready_direction()

    def _reset_pick_resolved_ready_state(self) -> None:
        self._pick_resolved_ready_dir_world = None
        self._pick_resolved_ready_pose_world_xyz = None

    def _compute_pick_ready_pose(
        self, object_world: tuple[float, float, float]
    ) -> Optional[tuple[float, float, float]]:
        try:
            return compute_ready_pose_target(
                tuple(float(v) for v in object_world),
                self._pick_ready_direction(),
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

    def _reset_pick_equal_sag_result_state(self) -> None:
        self._pick_centered_object_world_xyz = None
        self._pick_centered_ready_pose_world_xyz = None
        self._pick_ready_pose_drift_world = None
        self._pick_corrected_object_world_xyz = None
        self._pick_equal_sag_estimate = None
        self._pick_equal_sag_model = None
        self._pick_equal_sag_attempted = False

    def _pick_corrected_ready_pose(self) -> Optional[tuple[float, float, float]]:
        corrected_object = self._pick_corrected_object_world_xyz
        if corrected_object is None:
            return None
        return self._compute_pick_ready_pose(tuple(float(v) for v in corrected_object))

    def _pick_latch_initial_ready_pose(self) -> bool:
        if self._pick_initial_ready_pose_world_xyz is not None:
            return True
        object_world = self._pick_latest_object_world() or self._pick_frozen_world()
        if object_world is None:
            return False
        ready_pose = self._compute_pick_ready_pose(object_world)
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
            direction = self._pick_ready_direction()
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

    def _pick_try_estimate_equal_sag(self, host_state: Optional[HostState]) -> None:
        if bool(self._pick_equal_sag_attempted):
            return
        if not self._pick_latch_initial_ready_pose():
            return
        centered_object = self._pick_latest_object_world()
        if centered_object is None:
            return
        centered_ready = self._compute_pick_ready_pose(centered_object)
        if centered_ready is None or self._pick_initial_ready_pose_world_xyz is None:
            return
        self._pick_equal_sag_attempted = True
        self._pick_centered_object_world_xyz = tuple(float(v) for v in centered_object)
        self._pick_centered_ready_pose_world_xyz = tuple(float(v) for v in centered_ready)
        drift = (
            np.asarray(self._pick_initial_ready_pose_world_xyz, dtype=float).reshape(3)
            - np.asarray(centered_ready, dtype=float).reshape(3)
        )
        self._pick_ready_pose_drift_world = tuple(float(v) for v in drift)
        corrected_object = np.asarray(centered_object, dtype=float).reshape(3) + drift
        self._pick_corrected_object_world_xyz = tuple(float(v) for v in corrected_object)

        self.refresh_ik_context()
        ctx = dict(self._ik_context)
        base_sag = dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {}
        q4 = self._q_array_from_state(host_state)
        try:
            estimate = estimate_equal_sag_from_ready_pose_drift(
                context=ctx,
                q4=q4,
                ready_pose_drift_world=drift,
                sag_model=base_sag,
            )
        except Exception as exc:
            estimate = EqualSagEstimate(
                accepted=False,
                seg1_equal_offset_deg=0.0,
                seg2_equal_offset_deg=0.0,
                drift_world=tuple(float(v) for v in drift),
                reconstructed_drift_world=(0.0, 0.0, 0.0),
                residual_m=float(np.linalg.norm(drift)),
                condition=float("inf"),
                reason=f"estimate_failed: {exc}",
            )
        if (not bool(estimate.accepted)) and str(estimate.reason) == "drift_too_small":
            estimate = EqualSagEstimate(
                accepted=True,
                seg1_equal_offset_deg=0.0,
                seg2_equal_offset_deg=0.0,
                drift_world=tuple(float(v) for v in drift),
                reconstructed_drift_world=(0.0, 0.0, 0.0),
                residual_m=float(np.linalg.norm(drift)),
                condition=0.0,
                reason="drift_too_small_zero_correction",
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
            phase=ObjectPickPhase.READY.value,
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
            try:
                host_state = self.client.refresh_state() if self.client is not None else None
                current_seed = self._q_array_from_state(host_state)
                target_arr: np.ndarray
                direction_arr: np.ndarray
                q: Optional[np.ndarray] = None
                align_msg = ""
                position_error_m = float("inf")
                resolved_meta = ""

                if bool(resolve_dir):
                    resolved = resolve_feasible_ready_pose(
                        object_world=object_world,
                        preferred_dir=preferred_arr,
                        standoff_m=float(pk.ready_pose_standoff_m),
                        ik_context=ctx,
                        current_seed=current_seed,
                        position_tol_m=float(self._ik_cfg.tol),
                        max_iters=max(int(self._ik_cfg.max_iters), 1),
                        max_dir_error_deg=float(pk.ready_pose_max_dir_error_deg),
                        skip_search_under_deg=float(pk.ready_pose_skip_search_under_deg),
                        lateral_offsets_m=tuple(pk.ready_pose_lateral_offsets_m),
                        height_offsets_m=tuple(pk.ready_pose_height_offsets_m),
                        look_dot_min=float(pk.ready_pose_look_dot_min),
                        hand_eye_transform=self._hand_eye_transform,
                        hand_eye_parent_frame=self._hand_eye_parent_frame,
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
                    result = ik_pipeline.solve_then_align(
                        target_world=target_arr,
                        target_dir_world=direction_arr,
                        context=ctx,
                        position_tol_m=float(self._ik_cfg.tol),
                        max_iters=max(int(self._ik_cfg.max_iters), 1),
                        current_seed=current_seed,
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
                self.send_ready_pose_meta(source="target")
                self._send_ready_pose_markers(
                    object_world=object_tuple,
                    target=target_arr,
                    direction=direction_arr,
                    actual_offset_m=actual_offset_m,
                    corrected=bool(corrected),
                )
                self._apply_ik_solution_to_host(
                    q,
                    ik_target=target_arr,
                    ik_target_dir=direction_arr,
                    err_m=float(position_error_m),
                    status_msg=f"{label} | {align_msg}",
                    timeout_s=3.0,
                    sag_model_override=dict(sag_model),
                )
                if bool(corrected):
                    self._send_equal_sag_markers()
                done_msg = "%s done | err=%.1fmm" % (str(label), float(position_error_m) * 1000.0)
                if resolved_meta:
                    done_msg = "%s | %s" % (done_msg, resolved_meta)
                self.state.set_pick_status(
                    running=False,
                    failed=False,
                    phase=ObjectPickPhase.DONE.value,
                    msg=done_msg,
                )
                print(
                    "[Pick] %s done | target=(%.3f, %.3f, %.3f) dir=(%.3f, %.3f, %.3f) "
                    "err=%.1fmm corrected=%s %s"
                    % (
                        str(label),
                        float(target_arr[0]),
                        float(target_arr[1]),
                        float(target_arr[2]),
                        float(direction_arr[0]),
                        float(direction_arr[1]),
                        float(direction_arr[2]),
                        float(position_error_m) * 1000.0,
                        str(bool(corrected)).lower(),
                        resolved_meta,
                    )
                )
            finally:
                self._ik_worker = None

        self._ik_worker = threading.Thread(target=_worker, name="ready-pose", daemon=True)
        self._ik_worker.start()

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
        ready_pose = self._compute_pick_ready_pose(tuple(float(v) for v in object_world))
        if ready_pose is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="invalid ready target dir",
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
        look_vec = object_arr - tip
        look_len = float(np.linalg.norm(look_vec))
        if look_len <= 1e-6:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="object too close to current tip for Look",
            )
            return

        look_dir = look_vec / look_len
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
        self.state.set_target_dir(float(look_dir[0]), float(look_dir[1]), float(look_dir[2]))
        self.state.set_pick_status(
            running=True,
            failed=False,
            phase=ObjectPickPhase.LOOK.value,
            msg="look solving",
        )
        self.state.set_ik_status(
            running=True,
            converged=False,
            failed=False,
            err_m=float("inf"),
            msg="look solving",
        )

        def _worker() -> None:
            try:
                result = ik_pipeline.solve_then_align(
                    target_world=tip,
                    target_dir_world=look_dir,
                    context=ctx,
                    position_tol_m=float(self._ik_cfg.tol),
                    max_iters=max(int(self._ik_cfg.max_iters), 1),
                    current_seed=q0,
                )
                if result.success and result.q is not None:
                    q = np.asarray(result.q, dtype=float).reshape(4)
                    align_msg = str(result.reason)
                    if result.align_attempted:
                        align_msg = "%s | dir %.1f -> %.1f deg" % (
                            str(result.reason),
                            float(np.degrees(result.initial_direction_angle_rad)),
                            float(np.degrees(result.direction_angle_rad)),
                        )
                    self._apply_ik_solution_to_host(
                        q,
                        ik_target=tip,
                        ik_target_dir=look_dir,
                        err_m=float(result.position_error_m),
                        status_msg="look | " + align_msg,
                        timeout_s=2.0,
                        sag_model_override=dict(base_sag),
                    )
                    object_tuple = tuple(float(v) for v in object_arr)
                    ready_tuple = tuple(float(v) for v in np.asarray(ready_pose, dtype=float).reshape(3))
                    tip_tuple = tuple(float(v) for v in tip)
                    look_tuple = tuple(float(v) for v in look_dir)
                    self._pick_look_object_world_xyz = object_tuple
                    self._pick_look_ready_pose_world_xyz = ready_tuple
                    self._pick_look_tip_world_xyz = tip_tuple
                    self._pick_look_dir_world = look_tuple
                    self._pick_initial_object_world_xyz = object_tuple
                    self._pick_initial_ready_pose_world_xyz = ready_tuple
                    self._pick_frozen_world_xyz = object_tuple
                    ready_arr = np.asarray(ready_tuple, dtype=float).reshape(3)
                    ready_dir = np.asarray(self._pick_ready_direction(), dtype=float).reshape(3)
                    offset_m = float(np.linalg.norm(object_arr - ready_arr))
                    self._send_ready_pose_markers(
                        object_world=object_tuple,
                        target=ready_arr,
                        direction=ready_dir,
                        actual_offset_m=offset_m,
                        corrected=False,
                    )
                    self.send_ready_pose_meta(source="target")
                    self.state.set_pick_status(
                        running=False,
                        failed=False,
                        phase=ObjectPickPhase.DONE.value,
                        msg=(
                            "look done | baseline ready=(%.3f, %.3f, %.3f)"
                            % (float(ready_arr[0]), float(ready_arr[1]), float(ready_arr[2]))
                        ),
                    )
                    print(
                        "[Pick] look done | object=(%.3f, %.3f, %.3f) tip=(%.3f, %.3f, %.3f) "
                        "look_dir=(%.3f, %.3f, %.3f) baseline_ready=(%.3f, %.3f, %.3f)"
                        % (
                            float(object_arr[0]),
                            float(object_arr[1]),
                            float(object_arr[2]),
                            float(tip[0]),
                            float(tip[1]),
                            float(tip[2]),
                            float(look_dir[0]),
                            float(look_dir[1]),
                            float(look_dir[2]),
                            float(ready_arr[0]),
                            float(ready_arr[1]),
                            float(ready_arr[2]),
                        )
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
            finally:
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
        self._pick_stop_event.clear()
        direction = np.asarray(self._pick_requested_ready_direction(), dtype=float).reshape(3)
        corrected_ready = self._pick_corrected_ready_pose()
        use_corrected = corrected_ready is not None and isinstance(self._pick_equal_sag_model, dict)
        pk = self._pick_config_effective()
        resolve_dir = bool(pk.ready_pose_resolve_dir)
        if use_corrected:
            object_world = self._pick_corrected_object_world_xyz
            if object_world is None:
                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg="corrected ready missing corrected object",
                )
                return
            sag_model = dict(self._pick_equal_sag_model)
            label = "corrected ready pose"
        else:
            object_world = (
                self._pick_latest_object_world()
                or self._pick_frozen_world()
                or self._pick_look_object_world_xyz
                or self._pick_initial_object_world_xyz
            )
            if object_world is None:
                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg="no object world coordinate",
                )
                return
            sag_model = dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {}
            label = "uncorrected ready pose"

        object_tuple = tuple(float(v) for v in object_world)
        if not bool(resolve_dir):
            try:
                target_tuple = compute_ready_pose_target(
                    object_tuple,
                    tuple(float(v) for v in direction),
                    standoff_m=float(pk.ready_pose_standoff_m),
                )
            except ValueError as exc:
                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg=str(exc),
                )
                return
            target = np.asarray(target_tuple, dtype=float).reshape(3)
            actual_offset_m = float(
                np.linalg.norm(np.asarray(object_tuple, dtype=float).reshape(3) - target)
            )
            self.state.set_target(float(target[0]), float(target[1]), float(target[2]))
            self.state.set_target_dir(float(direction[0]), float(direction[1]), float(direction[2]))
            self.send_ready_pose_meta(source="target")
            self._send_ready_pose_markers(
                object_world=object_tuple,
                target=target,
                direction=direction,
                actual_offset_m=actual_offset_m,
                corrected=bool(use_corrected),
            )
            self.state.set_pick_status(
                running=False,
                failed=False,
                phase=ObjectPickPhase.IDLE.value,
                msg=(
                    "%s | object=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f) "
                    "standoff=%.0fmm actual=%.0fmm"
                    % (
                        str(label),
                        float(object_tuple[0]),
                        float(object_tuple[1]),
                        float(object_tuple[2]),
                        float(target[0]),
                        float(target[1]),
                        float(target[2]),
                        float(pk.ready_pose_standoff_m) * 1000.0,
                        float(actual_offset_m) * 1000.0,
                    )
                ),
            )
            print(
                "[Pick] %s | object=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f) "
                "dir=(%.3f, %.3f, %.3f) standoff=%.0fmm actual=%.0fmm"
                % (
                    str(label),
                    float(object_tuple[0]),
                    float(object_tuple[1]),
                    float(object_tuple[2]),
                    float(target[0]),
                    float(target[1]),
                    float(target[2]),
                    float(direction[0]),
                    float(direction[1]),
                    float(direction[2]),
                    float(pk.ready_pose_standoff_m) * 1000.0,
                    float(actual_offset_m) * 1000.0,
                )
            )

        self._start_ready_pose_resolve_and_solve(
            object_world=object_tuple,
            preferred_dir=direction,
            sag_model=sag_model,
            label=label,
            corrected=bool(use_corrected),
            resolve_dir=bool(resolve_dir),
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
        pk = self._pick_cfg
        return PickConfig(
            enabled=bool(pk.enabled),
            target_scale=float(self.state.visual_target_scale),
            scale_tol=float(self.state.visual_scale_tol),
            center_tol=float(self.state.visual_center_tol),
            center_u_gain=float(pk.center_u_gain),
            center_v_gain=float(pk.center_v_gain),
            center_roll_max=float(pk.center_roll_max),
            center_seg_max=float(pk.center_seg_max),
            center_error_scale_max=float(pk.center_error_scale_max),
            center_stuck_iters=int(pk.center_stuck_iters),
            center_stuck_max_uv=float(pk.center_stuck_max_uv),
            target_uv_u=float(self.state.visual_target_uv_u),
            target_uv_v=float(self.state.visual_target_uv_v),
            quadrant_fill_min=float(pk.quadrant_fill_min),
            ready_pose_standoff_m=float(self.state.visual_ready_distance_m),
            approach_extend_m=float(pk.approach_extend_m),
            approach_extend_step_m=float(pk.approach_extend_step_m),
            grid_cols=int(pk.grid_cols),
            grid_rows=int(pk.grid_rows),
            target_grid_col=int(pk.target_grid_col),
            target_grid_row=int(pk.target_grid_row),
            linear_step_u=float(pk.linear_step_u),
            linear_gain=float(pk.linear_gain),
            max_iters=int(pk.max_iters),
            require_track_frames=int(pk.require_track_frames),
            acquire_timeout_s=float(pk.acquire_timeout_s),
            scale_stuck_iters=int(pk.scale_stuck_iters),
            scale_stuck_ratio=float(pk.scale_stuck_ratio),
            approach_min_scale=float(pk.approach_min_scale),
            approach_min_steps=int(pk.approach_min_steps),
            approach_loose_center_tol=float(pk.approach_loose_center_tol),
            approach_scale_plateau_iters=int(pk.approach_scale_plateau_iters),
            approach_scale_plateau_eps=float(pk.approach_scale_plateau_eps),
            aim_center_tol=float(pk.aim_center_tol),
        )

    def _pick_config_for_aim(self) -> PickConfig:
        """Stricter UV tolerance for Aim finish (centroid near target crosshair)."""
        pk = self._pick_config_effective()
        aim_tol = float(max(0.01, float(self._pick_cfg.aim_center_tol)))
        return replace(pk, center_tol=aim_tol)

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
    ) -> tuple[ControlU, str, float, float]:
        cfg = self._pick_config_effective() if cfg is None else cfg
        center_tol = float(cfg.center_tol)
        tu, tv = float(cfg.target_uv_u), float(cfg.target_uv_v)
        u = float(obs.center_uv[0])
        v = float(obs.center_uv[1])
        u_delta = u - tu
        v_delta = v - tv
        # Same UV band as evaluate_pick_convergence (avoid center_ok False while still correcting).
        u_over = abs(u_delta) > float(center_tol)
        v_over = abs(v_delta) > float(center_tol)
        step_scale = float(max(min(float(self._pick_aim_step_scale), 1.0), 0.05))
        seg_cap = float(cfg.center_seg_max) * step_scale
        roll_cap = float(cfg.center_roll_max) * step_scale
        if not u_over and not v_over:
            return current_u, "none", 0.0, 0.0

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
        if u_over and v_over:
            mode = "uv_jacobian"
        elif u_over:
            mode = "uv_jacobian_u"
        else:
            mode = "uv_jacobian_v"
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
        self._reset_pick_search_state()
        self._reset_pick_drift_accounting()
        self._reset_pick_equal_sag_result_state()
        self._pick_initial_object_world_xyz = tuple(self._pick_look_object_world_xyz)
        self._pick_initial_ready_pose_world_xyz = tuple(self._pick_look_ready_pose_world_xyz)
        self._pick_frozen_world_xyz = tuple(self._pick_look_object_world_xyz)
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
                            self.state.set_pick_status(
                                running=False,
                                failed=False,
                                phase=ObjectPickPhase.DONE.value,
                                msg=(
                                    "aim done | corrected ready ready | drift=%.1fmm "
                                    "seg1=%+.2fdeg seg2=%+.2fdeg"
                                )
                                % (
                                    drift_mm,
                                    float(estimate.seg1_equal_offset_deg),
                                    float(estimate.seg2_equal_offset_deg),
                                ),
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
                        obs, current_u, cfg=aim_pk
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
        if self.state.ik_running or self._visual_busy():
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="busy",
            )
            return
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

        self.refresh_ik_context()
        ctx = dict(self._ik_context)
        ctx["sag_model"] = dict(self._pick_equal_sag_model)
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

        target = np.asarray(corrected_ready, dtype=float).reshape(3)
        direction = np.asarray(self._pick_ready_direction(), dtype=float).reshape(3)
        dnorm = float(np.linalg.norm(direction))
        if dnorm > 1e-9:
            direction = direction / dnorm
        self.state.set_target(float(target[0]), float(target[1]), float(target[2]))
        self.state.set_target_dir(float(direction[0]), float(direction[1]), float(direction[2]))
        self.state.set_pick_status(
            running=True,
            failed=False,
            phase=ObjectPickPhase.TWEAK.value,
            msg="tweak solving corrected ready",
        )
        self.state.set_ik_status(
            running=True,
            converged=False,
            failed=False,
            err_m=float("inf"),
            msg="tweak solving corrected ready",
        )

        def _worker() -> None:
            try:
                current_seed = np.array(
                    [
                        float(self.state.linear),
                        float(self.state.roll),
                        float(self.state.theta1),
                        float(self.state.theta2),
                    ],
                    dtype=float,
                )
                result = ik_pipeline.solve_then_align(
                    target_world=target,
                    target_dir_world=direction,
                    context=ctx,
                    position_tol_m=float(self._ik_cfg.tol),
                    max_iters=max(int(self._ik_cfg.max_iters), 1),
                    current_seed=current_seed,
                )
                if result.success and result.q is not None:
                    q = np.asarray(result.q, dtype=float).reshape(4)
                    align_msg = str(result.reason)
                    if result.align_attempted:
                        align_msg = "%s | dir %.1f -> %.1f deg" % (
                            str(result.reason),
                            float(np.degrees(result.initial_direction_angle_rad)),
                            float(np.degrees(result.direction_angle_rad)),
                        )
                    self._apply_ik_solution_to_host(
                        q,
                        ik_target=target,
                        ik_target_dir=direction,
                        err_m=float(result.position_error_m),
                        status_msg="tweak corrected ready | " + align_msg,
                        timeout_s=3.0,
                        sag_model_override=dict(self._pick_equal_sag_model),
                    )
                    self._send_equal_sag_markers()
                    self.state.set_pick_status(
                        running=False,
                        failed=False,
                        phase=ObjectPickPhase.DONE.value,
                        msg=(
                            "tweak done | corrected ready | err=%.1fmm"
                            % (float(result.position_error_m) * 1000.0)
                        ),
                    )
                    print(
                        "[Aim] tweak done | target=(%.3f, %.3f, %.3f) err=%.1fmm "
                        "seg1=%+.3fdeg seg2=%+.3fdeg"
                        % (
                            float(target[0]),
                            float(target[1]),
                            float(target[2]),
                            float(result.position_error_m) * 1000.0,
                            float(estimate.seg1_equal_offset_deg),
                            float(estimate.seg2_equal_offset_deg),
                        )
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
                        msg="tweak IK failed | " + str(result.reason),
                    )
            finally:
                self._ik_worker = None

        self._ik_worker = threading.Thread(target=_worker, daemon=True)
        self._ik_worker.start()

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

    def close(self) -> None:
        self.stop_object_pick()
        self.stop_perception_capture()
        if self.client is not None:
            self.client.close()
