from __future__ import annotations

import math
import os
import threading
import time
from typing import Any, Optional

import numpy as np

from addons.perception_bridge.hand_eye import camera_axes_world, load_hand_eye_transform
from engine import ik as ik_pipeline
from engine.config_loader import IkConfig, PerceptionConfig, PickConfig, load_app_config_from_ini
from engine.protocol import ControlU, SimMappingConfig, SimQ, control_u_to_sim_q, sim_q_to_control_u
from engine.sag_model import load_sag_model_json

from .client import ControlClient
from .perception import VisualObservation, extract_visual_observation
from .object_pick import ObjectPickPhase, evaluate_pick_convergence
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
    ) -> None:
        self.state = state
        self.client = client
        self._mapping_cfg = mapping_cfg or SimMappingConfig()
        self._ik_cfg = ik_cfg or IkConfig()
        self._ik_context = dict(ik_context or {})
        self._config_path = None if config_path is None else str(config_path)
        self._perception_cfg = perception_cfg or PerceptionConfig()
        self._pick_cfg = pick_cfg or PickConfig()
        self._perception_capture: Optional[PerceptionCapture] = None
        self._pick_worker: Optional[threading.Thread] = None
        self._pick_stop_event = threading.Event()
        self._hand_eye_transform = None
        self._hand_eye_parent_frame = "node9"
        self._ik_worker: Optional[threading.Thread] = None
        self._visual_worker: Optional[threading.Thread] = None
        self._visual_stop_event = threading.Event()
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
        self._visual_outer_iters = 12
        self._visual_center_outer_iters = 40
        self._center_min_progress_uv = 0.012
        self._center_stall_steps = 3
        self._center_u_enter_v_ratio = 0.5
        self._center_seg_step_max = 1.0
        self._center_seg_coupling_u = 0.05
        self._visual_u_deadband = 0.05
        self._visual_v_deadband = 0.05
        self._visual_scale_deadband = 0.01
        self._visual_auto_roll_u_max = 6.0
        self._visual_auto_seg_u_max = 5.0
        self._visual_auto_linear_u_max = 6.0
        self._visual_u_gain = 14.0
        self._visual_v_gain = 12.0
        self._visual_scale_gain = 60.0
        self._visual_center_roll_u_max = 3.0
        self._visual_center_seg_u_max = 3.0
        self._visual_center_u_gain = 8.0
        self._visual_center_v_gain = 8.0
        self._center_roll_u_max = 4.0
        self._center_seg_u_max = 4.0
        self._center_u_gain = 10.0
        self._center_v_gain = 10.0
        self._center_roll_rad_max = math.radians(6.0)
        self._center_tilt_rad_max = math.radians(6.0)
        self._center_tilt_step_scale = 0.5
        self._manual_camera_angle_step = math.radians(2.5)
        self._manual_camera_linear_m_max = 0.002
        self._manual_camera_roll_rad_max = math.radians(4.0)
        self._manual_camera_seg_rad_max = math.radians(3.0)
        self._manual_camera_angle_tol_rad = math.radians(0.75)
        self._manual_camera_iters = 8
        if self._config_path:
            try:
                bundle = load_app_config_from_ini(self._config_path)
                hand_eye_path = str(bundle.sim_config.hand_eye_config).strip()
                if hand_eye_path:
                    self._hand_eye_transform, hand_eye_meta = load_hand_eye_transform(hand_eye_path)
                    self._hand_eye_parent_frame = str(hand_eye_meta.get("parent_frame", "node9"))
            except Exception:
                self._hand_eye_transform = None

    @staticmethod
    def _normalize_dir(vec: np.ndarray) -> Optional[np.ndarray]:
        arr = np.asarray(vec, dtype=float).reshape(3)
        norm = float(np.linalg.norm(arr))
        if norm <= 1e-9:
            return None
        return arr / norm

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

    def _visual_uv_centered(self, obs: VisualObservation, *, center_tol: Optional[float] = None) -> bool:
        tol = float(self.state.visual_center_tol if center_tol is None else center_tol)
        du, dv, _, _ = self._visual_uv_errors(obs)
        return abs(du) <= tol and abs(dv) <= tol

    def _visual_busy(self) -> bool:
        return self._ik_worker is not None or self._visual_worker is not None or self._pick_worker is not None

    def _pick_busy(self) -> bool:
        return self._pick_worker is not None

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

    def _send_state_q_and_wait(self, *, timeout_s: float = 1.0, source: str = "ik") -> Optional[HostState]:
        q_cmd = np.array(
            [
                float(self.state.linear),
                float(self.state.roll),
                float(self.state.theta1),
                float(self.state.theta2),
            ],
            dtype=float,
        )
        self.send_current_target(source=source)
        return self._wait_until_q_settled(q_cmd, timeout_s=float(timeout_s))

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

    def _command_q_and_wait(self, q: np.ndarray, *, timeout_s: float = 1.0) -> Optional[HostState]:
        q_cmd = self._clamp_q(q)
        self.state.set_q(float(q_cmd[0]), float(q_cmd[1]), float(q_cmd[2]), float(q_cmd[3]))
        return self._send_state_q_and_wait(timeout_s=float(timeout_s), source="slider")

    def _camera_axes_from_q(self, q: np.ndarray) -> Optional[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]:
        if self._hand_eye_transform is None:
            return None
        try:
            origin, look_vec, right_vec = camera_axes_world(
                self._ik_context,
                np.asarray(q, dtype=float).reshape(4),
                self._hand_eye_transform,
                parent_frame=self._hand_eye_parent_frame,
                axis_len_m=0.08,
            )
        except Exception:
            return None
        look = self._normalize_dir(look_vec)
        right = self._normalize_dir(right_vec)
        if look is None or right is None:
            return None
        up = self._normalize_dir(np.cross(look, right))
        if up is None:
            return None
        return np.asarray(origin, dtype=float).reshape(3), look, right, up

    def _camera_look_jacobian(self, q: np.ndarray) -> Optional[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]:
        axes = self._camera_axes_from_q(q)
        if axes is None:
            return None
        _origin, look, right, up = axes
        J = np.zeros((3, 4), dtype=float)
        eps = np.array([0.002, math.radians(2.0), math.radians(2.0), math.radians(2.0)], dtype=float)
        q_base = np.asarray(q, dtype=float).reshape(4)
        for idx in range(4):
            q_try = q_base.copy()
            q_try[idx] += eps[idx]
            q_try = self._clamp_q(q_try)
            applied = float(q_try[idx] - q_base[idx])
            if abs(applied) <= 1e-9:
                continue
            axes_try = self._camera_axes_from_q(q_try)
            if axes_try is None:
                continue
            _origin_try, look_try, _right_try, _up_try = axes_try
            J[:, idx] = (look_try - look) / applied
        return J, look, right, up

    @staticmethod
    def _vector_angle_rad(a: np.ndarray, b: np.ndarray) -> float:
        va = np.asarray(a, dtype=float).reshape(3)
        vb = np.asarray(b, dtype=float).reshape(3)
        na = float(np.linalg.norm(va))
        nb = float(np.linalg.norm(vb))
        if na <= 1e-12 or nb <= 1e-12:
            return float("inf")
        cosang = float(np.clip(np.dot(va, vb) / (na * nb), -1.0, 1.0))
        return float(math.acos(cosang))

    def _apply_manual_camera_rotation(
        self,
        *,
        right_angle_rad: float = 0.0,
        up_angle_rad: float = 0.0,
        status_prefix: str,
        stop_running_visual: bool = True,
        update_visual_status: bool = True,
    ) -> None:
        if stop_running_visual and self._visual_worker is not None:
            self.stop_visual_servo()
        q_now = self._q_array_from_state(self.current_host_state())
        jac_data = self._camera_look_jacobian(q_now)
        if jac_data is None:
            self.state.set_visual_status(running=False, failed=False, msg=f"{status_prefix} | camera model unavailable")
            return
        J, look, right, up = jac_data
        desired = self._normalize_dir(
            look
            + float(right_angle_rad) * right
            + float(up_angle_rad) * up
        )
        if desired is None:
            self.state.set_visual_status(running=False, failed=False, msg=f"{status_prefix} | invalid target")
            return
        state: Optional[HostState] = None
        final_err_deg = float("inf")
        q_cmd = np.asarray(q_now, dtype=float).reshape(4).copy()
        for _ in range(int(self._manual_camera_iters)):
            jac_data_now = self._camera_look_jacobian(q_cmd)
            if jac_data_now is None:
                self.state.set_visual_status(running=False, failed=False, msg=f"{status_prefix} | camera model unavailable")
                return
            J_now, look_now, _right_now, _up_now = jac_data_now
            final_err_deg = float(np.degrees(self._vector_angle_rad(look_now, desired)))
            if final_err_deg <= float(np.degrees(self._manual_camera_angle_tol_rad)):
                break
            dlook = np.asarray(desired - look_now, dtype=float).reshape(3)
            dq = np.linalg.pinv(J_now) @ dlook
            dq = np.asarray(dq, dtype=float).reshape(4)
            dq[0] = float(np.clip(dq[0], -self._manual_camera_linear_m_max, self._manual_camera_linear_m_max))
            dq[1] = float(np.clip(dq[1], -self._manual_camera_roll_rad_max, self._manual_camera_roll_rad_max))
            dq[2] = float(np.clip(dq[2], -self._manual_camera_seg_rad_max, self._manual_camera_seg_rad_max))
            dq[3] = float(np.clip(dq[3], -self._manual_camera_seg_rad_max, self._manual_camera_seg_rad_max))
            q_try = self._clamp_q(q_cmd + dq)
            if np.allclose(q_try, q_cmd, atol=1e-9, rtol=0.0):
                break
            state = self._command_q_and_wait(q_try, timeout_s=1.0)
            if state is not None and state.q is not None:
                q_cmd = np.array(
                    [
                        float(state.q.linear_m),
                        float(state.q.roll_rad),
                        float(state.q.theta1_rad),
                        float(state.q.theta2_rad),
                    ],
                    dtype=float,
                )
            else:
                q_cmd = q_try
        if not update_visual_status:
            return
        obs = self.current_visual_observation(state)
        if obs is None:
            self.state.set_visual_status(
                running=False,
                failed=False,
                msg=f"{status_prefix} | moved | residual {final_err_deg:.2f} deg",
            )
            return
        self.state.set_visual_status(
            running=False,
            failed=False,
            msg="%s | uv=(%.3f, %.3f) scale=%.3f | residual %.2f deg"
            % (status_prefix, float(obs.center_uv[0]), float(obs.center_uv[1]), float(obs.scale), float(final_err_deg)),
        )

    def _visual_error_vec(self, obs: VisualObservation) -> np.ndarray:
        u_err, v_err = self._uv_control_errors(obs)
        target_scale = float(self.state.visual_target_scale)
        return np.array(
            [
                float(u_err),
                float(v_err),
                float(target_scale - obs.scale),
            ],
            dtype=float,
        )

    def _visual_cost(self, obs: VisualObservation) -> float:
        err = self._visual_error_vec(obs)
        return float(4.0 * err[0] ** 2 + 4.0 * err[1] ** 2 + err[2] ** 2)

    @staticmethod
    def _log_visual_step(tag: str, step_idx: int, step_max: int, **fields: object) -> None:
        parts = [f"[Visual] {tag} step {int(step_idx)}/{int(step_max)}"]
        for key, value in fields.items():
            parts.append(f"{key}={value}")
        print(" | ".join(parts))

    def _visual_candidate_delta(
        self,
        obs: VisualObservation,
        *,
        center_only: bool = False,
    ) -> tuple[float, float, float]:
        u_err, v_err, scale_err = [float(v) for v in self._visual_error_vec(obs)]
        roll_du = 0.0
        seg_du = 0.0
        linear_du = 0.0
        if center_only:
            u_gain = float(self._visual_center_u_gain)
            v_gain = float(self._visual_center_v_gain)
            roll_max = float(self._visual_center_roll_u_max)
            seg_max = float(self._visual_center_seg_u_max)
        else:
            u_gain = float(self._visual_u_gain)
            v_gain = float(self._visual_v_gain)
            roll_max = float(self._visual_auto_roll_u_max)
            seg_max = float(self._visual_auto_seg_u_max)
        if abs(u_err) > float(self._visual_u_deadband):
            roll_du = float(np.clip(u_gain * u_err, -roll_max, roll_max))
        if abs(v_err) > float(self._visual_v_deadband):
            seg_du = float(np.clip(v_gain * v_err, -seg_max, seg_max))
        if abs(scale_err) > float(self._visual_scale_deadband):
            # Display u_linear=0 is forward; decrease u to enlarge object scale.
            linear_du = float(
                np.clip(
                    -self._visual_scale_gain * scale_err,
                    -self._visual_auto_linear_u_max,
                    self._visual_auto_linear_u_max,
                )
            )
        return linear_du, roll_du, seg_du

    def _apply_visual_candidate(
        self,
        base_u: ControlU,
        *,
        linear_du: float = 0.0,
        roll_du: float = 0.0,
        seg_du: float = 0.0,
        source: str = "ik",
    ) -> tuple[ControlU, Optional[HostState], Optional[VisualObservation]]:
        candidate_u = self._clamp_display_u(
            ControlU(
                u_linear=float(base_u.u_linear + linear_du),
                u_roll=float(base_u.u_roll + roll_du),
                u_s1=float(base_u.u_s1 + seg_du),
                u_s2=float(base_u.u_s2 + seg_du),
            )
        )
        state = self._send_display_control_u_and_wait(candidate_u, timeout_s=1.0, source=source)
        return candidate_u, state, self.current_visual_observation(state)

    def _revert_visual_candidate(self, base_u: ControlU) -> Optional[HostState]:
        return self._send_display_control_u_and_wait(base_u, timeout_s=0.8, source="ik")

    def _best_visual_candidate_for_axis(
        self,
        base_u: ControlU,
        current_obs: VisualObservation,
        *,
        linear_du: float = 0.0,
        roll_du: float = 0.0,
        seg_du: float = 0.0,
    ) -> tuple[ControlU, Optional[VisualObservation]]:
        current_cost = self._visual_cost(current_obs)
        best_u = base_u
        best_obs = current_obs
        best_cost = current_cost
        deltas = [
            (linear_du, roll_du, seg_du),
            (-linear_du, -roll_du, -seg_du),
        ]
        tried = set()
        for cand_linear, cand_roll, cand_seg in deltas:
            key = (round(cand_linear, 6), round(cand_roll, 6), round(cand_seg, 6))
            if key in tried:
                continue
            tried.add(key)
            if abs(cand_linear) <= 1e-9 and abs(cand_roll) <= 1e-9 and abs(cand_seg) <= 1e-9:
                continue
            cand_u, _state, cand_obs = self._apply_visual_candidate(
                base_u,
                linear_du=float(cand_linear),
                roll_du=float(cand_roll),
                seg_du=float(cand_seg),
                source="ik",
            )
            if cand_obs is None:
                self._revert_visual_candidate(base_u)
                continue
            cand_cost = self._visual_cost(cand_obs)
            if cand_cost + 1e-6 < best_cost:
                best_u = cand_u
                best_obs = cand_obs
                best_cost = cand_cost
                if cand_u != base_u:
                    return best_u, best_obs
            self._revert_visual_candidate(base_u)
        return best_u, best_obs

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
        self.apply_control_u(u_linear=15.0, u_roll=180.0, u_s1=180.0, u_s2=180.0, apply_offset=False)
        self.send_current_target(source="slider")

    def send_current_target(self, *, source: str) -> None:
        if self.client is not None and ((not self.state.paused) or (source == "target")):
            self.client.send_target_values(
                linear_m=float(self.state.linear),
                roll_rad=float(self.state.roll),
                theta1_rad=float(self.state.theta1),
                theta2_rad=float(self.state.theta2),
                source=source,
                target_xyz=(float(self.state.target_x), float(self.state.target_y), float(self.state.target_z)),
                target_dir=(float(self.state.target_vx), float(self.state.target_vy), float(self.state.target_vz)),
                sag_model=(dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {}),
                claw_closed=bool(self.state.claw_closed),
                force=bool(source == "target"),
            )

    def send_current_target_meta(self, *, source: str = "target") -> None:
        if self.client is not None:
            self.client.send_target_meta(
                target_xyz=(float(self.state.target_x), float(self.state.target_y), float(self.state.target_z)),
                target_dir=(float(self.state.target_vx), float(self.state.target_vy), float(self.state.target_vz)),
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
                    self.send_current_target(source="ik")
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

    def start_tweak(self) -> None:
        if self.state.ik_running or self._visual_busy():
            return
        self.refresh_ik_context()
        ctx = dict(self._ik_context)
        ctx["sag_model"] = dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {}
        required = ("limit", "fk_joint_chain", "terminal_link_name", "approach_axis_local")
        if any(k not in ctx for k in required):
            print("[UI] Tweak rejected | missing ik_context fields")
            self.state.set_ik_status(running=False, converged=False, failed=True, err_m=float("inf"), msg="missing IK context")
            return

        direction = np.array([self.state.target_vx, self.state.target_vy, self.state.target_vz], dtype=float)
        dnorm = float(np.linalg.norm(direction))
        if dnorm <= 1e-9:
            self.state.set_ik_status(running=False, converged=False, failed=True, err_m=float("inf"), msg="invalid target direction")
            return
        target_dir = direction / dnorm
        self.state.set_ik_status(running=True, converged=False, failed=False, err_m=float("inf"), msg="tweaking")

        def _worker() -> None:
            try:
                q_cmd = np.array([float(self.state.linear), float(self.state.roll), float(self.state.theta1), float(self.state.theta2)], dtype=float)
                if self.client is None:
                    self.state.set_ik_status(running=False, converged=False, failed=True, err_m=float("inf"), msg="no feedback client")
                    return

                host_state = self.client.refresh_state()
                if host_state is None or host_state.actual_tip_xyz is None or host_state.actual_tip_dir is None:
                    self.state.set_ik_status(running=False, converged=False, failed=True, err_m=float("inf"), msg="missing actual tip feedback")
                    return
                if host_state.q is not None:
                    q_cmd = np.array(
                        [float(host_state.q.linear_m), float(host_state.q.roll_rad), float(host_state.q.theta1_rad), float(host_state.q.theta2_rad)],
                        dtype=float,
                    )

                hold_target = np.array(host_state.actual_tip_xyz, dtype=float).reshape(3)
                actual_pos = np.array(host_state.actual_tip_xyz, dtype=float).reshape(3)
                actual_dir = self._normalize_dir(np.array(host_state.actual_tip_dir, dtype=float).reshape(3))
                if actual_dir is None:
                    self.state.set_ik_status(running=False, converged=False, failed=True, err_m=float("inf"), msg="invalid actual direction")
                    return

                session = ik_pipeline.begin_tweak_session(
                    current_q=q_cmd,
                    hold_target_world=hold_target,
                    target_dir_world=target_dir,
                    initial_step_scale=1.0,
                )
                pos_tol = 5e-3
                dir_tol_deg = 5.0
                last_pos_err = float("inf")
                last_dir_ang = float("inf")

                for _iter in range(10):
                    feedback = ik_pipeline.evaluate_tweak_feedback(
                        session=session,
                        actual_tip_world=actual_pos,
                        actual_dir_world=actual_dir,
                        position_tol_m=pos_tol,
                        direction_tol_deg=dir_tol_deg,
                    )
                    session = feedback.state
                    last_pos_err = float(feedback.position_error_m)
                    last_dir_ang = float(feedback.direction_angle_rad)
                    if feedback.converged:
                        q_cmd = np.asarray(session.q, dtype=float).reshape(4).copy()
                        self.state.set_q(float(q_cmd[0]), float(q_cmd[1]), float(q_cmd[2]), float(q_cmd[3]))
                        self.state.set_ik_solution(float(q_cmd[1]), float(q_cmd[2]), float(q_cmd[3]))
                        self.state.set_ik_status(
                            running=False,
                            converged=True,
                            failed=False,
                            err_m=float(feedback.position_error_m),
                            msg="tweak converged | dir %.1f deg | steps %d" % (float(np.degrees(feedback.direction_angle_rad)), int(session.accepted_steps)),
                        )
                        self.send_current_target(source="ik")
                        return

                    step = ik_pipeline.compute_tweak_session_step(
                        session=session,
                        context=ctx,
                        actual_tip_world=actual_pos,
                        actual_dir_world=actual_dir,
                    )
                    if not bool(step.accepted):
                        session = ik_pipeline.reject_tweak_step(session=session, step=step)
                        self.state.set_ik_status(running=False, converged=False, failed=True, err_m=float(feedback.position_error_m), msg=str(session.reason))
                        return

                    prev_session = session
                    prev_q = q_cmd.copy()
                    q_cmd = np.asarray(step.q, dtype=float).reshape(4).copy()
                    session = ik_pipeline.accept_tweak_step(session=session, step=step)
                    self.state.set_q(float(q_cmd[0]), float(q_cmd[1]), float(q_cmd[2]), float(q_cmd[3]))
                    self.send_current_target(source="ik")
                    post_state = self._wait_until_q_settled(q_cmd, timeout_s=1.0)
                    if post_state is None or post_state.actual_tip_xyz is None or post_state.actual_tip_dir is None:
                        self.state.set_ik_status(running=False, converged=False, failed=True, err_m=float(feedback.position_error_m), msg="lost actual feedback")
                        return
                    new_pos = np.array(post_state.actual_tip_xyz, dtype=float).reshape(3)
                    new_dir = self._normalize_dir(np.array(post_state.actual_tip_dir, dtype=float).reshape(3))
                    if new_dir is None:
                        self.state.set_ik_status(running=False, converged=False, failed=True, err_m=float(feedback.position_error_m), msg="invalid actual direction")
                        return
                    feedback_new = ik_pipeline.evaluate_tweak_feedback(
                        session=session,
                        actual_tip_world=new_pos,
                        actual_dir_world=new_dir,
                        position_tol_m=pos_tol,
                        direction_tol_deg=dir_tol_deg,
                    )
                    if feedback_new.cost <= feedback.cost + 1e-9:
                        actual_pos = new_pos
                        actual_dir = new_dir
                        session = feedback_new.state
                        continue

                    session = ik_pipeline.reject_tweak_step(session=prev_session, step=step)
                    q_cmd = prev_q.copy()
                    self.state.set_q(float(q_cmd[0]), float(q_cmd[1]), float(q_cmd[2]), float(q_cmd[3]))
                    self.send_current_target(source="ik")
                    self._wait_until_q_settled(q_cmd, timeout_s=0.8)
                    if float(session.step_scale) <= 0.051:
                        self.state.set_ik_status(running=False, converged=False, failed=True, err_m=float(feedback_new.position_error_m), msg=str(session.reason))
                        return

                self.state.set_q(float(q_cmd[0]), float(q_cmd[1]), float(q_cmd[2]), float(q_cmd[3]))
                self.state.set_ik_solution(float(q_cmd[1]), float(q_cmd[2]), float(q_cmd[3]))
                self.state.set_ik_status(
                    running=False,
                    converged=False,
                    failed=True,
                    err_m=float(last_pos_err),
                    msg="iteration limit | dir %.1f deg | steps %d" % (float(np.degrees(last_dir_ang)), int(session.accepted_steps)),
                )
            except Exception as exc:
                print(f"[UI] Tweak failed: {exc}")
                self.state.set_ik_status(running=False, converged=False, failed=True, err_m=float("inf"), msg=str(exc))
            finally:
                self._ik_worker = None

        self._ik_worker = threading.Thread(target=_worker, daemon=True)
        self._ik_worker.start()

    def nudge_visual_pan(self, direction: int) -> None:
        self.apply_visual_pan_angle(direction=int(direction), angle_deg=float(np.degrees(self._manual_camera_angle_step)))

    def nudge_visual_tilt(self, direction: int) -> None:
        self.apply_visual_tilt_angle(direction=int(direction), angle_deg=float(np.degrees(self._manual_camera_angle_step)))

    def apply_visual_pan_angle(self, *, direction: int, angle_deg: float) -> None:
        angle_rad = math.radians(max(0.0, float(angle_deg)))
        self._apply_manual_camera_rotation(
            right_angle_rad=float(direction) * angle_rad,
            up_angle_rad=0.0,
            status_prefix="manual pan",
        )

    def apply_visual_tilt_angle(self, *, direction: int, angle_deg: float) -> None:
        angle_rad = math.radians(max(0.0, float(angle_deg)))
        self._apply_manual_camera_rotation(
            right_angle_rad=0.0,
            up_angle_rad=float(direction) * angle_rad,
            status_prefix="manual tilt",
        )

    def stop_visual_servo(self) -> None:
        self._visual_stop_event.set()
        self.state.set_visual_status(running=False, failed=False, msg="stopped")

    def _start_visual_controller(self, *, center_only: bool) -> None:
        if self._visual_busy():
            self.state.set_visual_status(running=False, failed=True, msg="busy")
            return
        if self.client is None:
            self.state.set_visual_status(running=False, failed=True, msg="no feedback client")
            return
        host_state = self.client.refresh_state()
        obs = self.current_visual_observation(host_state)
        if obs is None:
            self.state.set_visual_status(running=False, failed=True, msg="no valid observation")
            return
        self._visual_stop_event.clear()
        self.state.set_visual_status(
            running=True,
            failed=False,
            msg=("centering object" if center_only else "visual servo running"),
        )

        def _worker() -> None:
            try:
                if center_only:
                    max_steps = int(self._visual_center_outer_iters)
                    center_tol = float(self.state.visual_center_tol)
                    target_u, target_v = self._visual_target_uv()
                    min_progress_uv = float(self._center_min_progress_uv)
                    stall_limit = int(self._center_stall_steps)
                    print(
                        "[Visual] center start | max_steps=%d tol=%.3f target_uv=(%+.3f,%+.3f) | uv=(%.3f, %.3f) scale=%.3f"
                        % (
                            max_steps,
                            center_tol,
                            target_u,
                            target_v,
                            float(obs.center_uv[0]),
                            float(obs.center_uv[1]),
                            float(obs.scale),
                        )
                    )
                    current_host = host_state
                    current_obs = obs
                    current_u = self.current_control_u()
                    stall_count = 0
                    force_seg_only = False
                    last_mode = ""
                    center_phase = "u"
                    u_enter_v = center_tol * float(self._center_u_enter_v_ratio)

                    for step_idx in range(1, max_steps + 1):
                        if self._visual_stop_event.is_set():
                            print(f"[Visual] center step {step_idx}/{max_steps} | stopped")
                            self.state.set_visual_status(running=False, failed=False, msg="stopped")
                            return
                        if current_obs is None:
                            print(f"[Visual] center step {step_idx}/{max_steps} | observation lost")
                            self.state.set_visual_status(running=False, failed=True, msg="observation lost")
                            return

                        u0 = float(current_obs.center_uv[0])
                        v0 = float(current_obs.center_uv[1])
                        u_delta0 = u0 - target_u
                        v_delta0 = v0 - target_v
                        if self._visual_uv_centered(current_obs, center_tol=center_tol):
                            print(
                                "[Visual] center step %d/%d | done | uv=(%.3f, %.3f) target_uv=(%+.3f,%+.3f) scale=%.3f"
                                % (step_idx, max_steps, u0, v0, target_u, target_v, float(current_obs.scale))
                            )
                            self.state.set_visual_status(
                                running=False,
                                failed=False,
                                msg="centered | uv=(%.3f, %.3f) target=(%+.3f,%+.3f) scale=%.3f"
                                % (u0, v0, target_u, target_v, float(current_obs.scale)),
                            )
                            return

                        snapshot_p_cam = None if current_host is None else current_host.perceived_object_camera_xyz
                        p_cam_ok = (
                            snapshot_p_cam is not None and float(snapshot_p_cam[2]) > 1e-6
                        )
                        v_needs = abs(v_delta0) > center_tol
                        if center_phase == "v" and abs(u_delta0) > center_tol:
                            center_phase = "u"
                        if center_phase == "u" and abs(u_delta0) <= u_enter_v and v_needs:
                            center_phase = "v"
                        use_snapshot_roll = (
                            center_phase == "u"
                            and abs(u_delta0) > u_enter_v
                            and p_cam_ok
                            and not force_seg_only
                            and abs(target_u) < 0.05
                        )
                        step_mode = ""
                        prev_ts = float(current_obs.timestamp_s)

                        if use_snapshot_roll:
                            q_now = self._q_array_from_state(current_host)
                            roll_raw = float(
                                np.clip(
                                    math.atan2(float(snapshot_p_cam[0]), float(snapshot_p_cam[2])),
                                    -self._center_roll_rad_max,
                                    self._center_roll_rad_max,
                                )
                            )
                            roll_scale = float(
                                np.clip(abs(u_delta0) / max(center_tol, 1e-6), 0.25, 1.0)
                            )
                            roll_delta = roll_raw * roll_scale
                            q_try = np.asarray(q_now, dtype=float).copy()
                            q_try[1] += roll_delta
                            q_try = self._clamp_q(q_try)
                            if np.allclose(q_try, q_now, atol=1e-9, rtol=0.0):
                                use_snapshot_roll = False
                            else:
                                step_mode = "snapshot_q_roll"
                                self._log_visual_step(
                                    "center",
                                    step_idx,
                                    max_steps,
                                    mode=step_mode,
                                    uv=f"({u0:+.3f},{v0:+.3f})",
                                    droll_deg=f"{float(np.degrees(roll_delta)):+.2f}",
                                    roll_scale=f"{roll_scale:.2f}",
                                )
                                self._command_q_and_wait(q_try, timeout_s=1.0)
                                self.state.set_visual_status(
                                    running=True,
                                    failed=False,
                                    msg="center | snapshot roll %.2f deg | uv=(%.3f, %.3f)"
                                    % (float(np.degrees(roll_delta)), u0, v0),
                                )

                        if not use_snapshot_roll:
                            if center_phase == "v" and (v_needs or force_seg_only):
                                uv_force_axis: Optional[str] = "v"
                            elif abs(u_delta0) > center_tol and not p_cam_ok:
                                uv_force_axis = "u"
                            else:
                                uv_force_axis = None
                            next_u, step_mode = self._apply_center_uv_step(
                                current_obs,
                                current_u,
                                center_tol=center_tol,
                                force_axis=uv_force_axis,
                                seg_u_max=float(self._center_seg_step_max),
                                u_active_tol=u_enter_v,
                                target_uv=(target_u, target_v),
                            )
                            if step_mode == "none":
                                print(
                                    "[Visual] center step %d/%d | command clamped | uv=(%.3f, %.3f)"
                                    % (step_idx, max_steps, u0, v0)
                                )
                                self.state.set_visual_status(
                                    running=False,
                                    failed=False,
                                    msg="command clamped | uv=(%.3f, %.3f)" % (u0, v0),
                                )
                                return
                            droll = float(next_u.u_roll - current_u.u_roll)
                            dseg = float(next_u.u_s1 - current_u.u_s1)
                            self._log_visual_step(
                                "center",
                                step_idx,
                                max_steps,
                                mode=step_mode,
                                uv=f"({u0:+.3f},{v0:+.3f})",
                                droll=f"{droll:+.2f}",
                                dseg=f"{dseg:+.2f}",
                            )
                            self._send_display_control_u_and_wait(next_u, timeout_s=1.0, source="ik")
                            current_u = next_u
                            self.state.set_visual_status(
                                running=True,
                                failed=False,
                                msg="center | %s | uv=(%.3f, %.3f)" % (step_mode, u0, v0),
                            )

                        next_host, next_obs = self._wait_center_observation(prev_ts=prev_ts)
                        if self._visual_stop_event.is_set():
                            print(
                                f"[Visual] center step {step_idx}/{max_steps} | stopped during wait"
                            )
                            self.state.set_visual_status(running=False, failed=False, msg="stopped")
                            return
                        if next_obs is None:
                            self._log_visual_step(
                                "center",
                                step_idx,
                                max_steps,
                                mode="wait_timeout",
                                uv=f"({u0:+.3f},{v0:+.3f})",
                                last=step_mode,
                            )
                            self.state.set_visual_status(
                                running=False,
                                failed=False,
                                msg="awaiting new detection timed out",
                            )
                            return

                        nu = float(next_obs.center_uv[0])
                        nv = float(next_obs.center_uv[1])
                        nu_delta = nu - target_u
                        if step_mode == "uv_seg" and abs(nu_delta) > center_tol and abs(nu_delta) > abs(u_delta0) + float(
                            self._center_seg_coupling_u
                        ):
                            print(
                                "[Visual] center step %d/%d | seg coupling | u %.3f -> %.3f, back to u phase"
                                % (step_idx, max_steps, u0, nu)
                            )
                            center_phase = "u"
                            force_seg_only = False
                            stall_count = 0
                        progress = max(abs(nu - u0), abs(nv - v0))
                        if progress < min_progress_uv:
                            stall_count += 1
                            self._log_visual_step(
                                "center",
                                step_idx,
                                max_steps,
                                mode="stall",
                                uv=f"({nu:+.3f},{nv:+.3f})",
                                progress=f"{progress:.4f}",
                                stall=f"{stall_count}/{stall_limit}",
                                last=step_mode,
                            )
                            if stall_count >= stall_limit:
                                stall_count = 0
                                if step_mode == "snapshot_q_roll":
                                    force_seg_only = True
                                elif step_mode == "uv_seg":
                                    force_seg_only = False
                                else:
                                    force_seg_only = True
                                print(
                                    "[Visual] center step %d/%d | stall recovery | force_seg_only=%s"
                                    % (step_idx, max_steps, force_seg_only)
                                )
                        else:
                            stall_count = 0
                            force_seg_only = False

                        self._log_visual_step(
                            "center",
                            step_idx,
                            max_steps,
                            mode="wait_ok",
                            uv=f"({nu:+.3f},{nv:+.3f})",
                            progress=f"{progress:.4f}",
                            last=step_mode,
                            phase=center_phase,
                        )
                        last_mode = step_mode
                        current_host = next_host
                        current_obs = next_obs

                    print(f"[Visual] center | iteration limit ({max_steps} steps) | last={last_mode}")
                    self.state.set_visual_status(running=False, failed=True, msg="iteration limit")

                max_steps = int(self._visual_outer_iters)
                print(
                    "[Visual] servo start | max_steps=%d | uv=(%.3f, %.3f) scale=%.3f target_scale=%.3f"
                    % (
                        max_steps,
                        float(obs.center_uv[0]),
                        float(obs.center_uv[1]),
                        float(obs.scale),
                        float(self.state.visual_target_scale),
                    )
                )
                current_obs = obs
                current_u = self.current_control_u()
                stale_count = 0
                for step_idx in range(1, max_steps + 1):
                    if self._visual_stop_event.is_set():
                        print(f"[Visual] servo step {step_idx}/{max_steps} | stopped")
                        self.state.set_visual_status(running=False, failed=False, msg="stopped")
                        return
                    if current_obs is None:
                        print(f"[Visual] servo step {step_idx}/{max_steps} | observation lost")
                        self.state.set_visual_status(running=False, failed=True, msg="observation lost")
                        return
                    center_ok = self._visual_uv_centered(current_obs)
                    scale_ok = True if center_only else (
                        float(current_obs.scale) >= float(self.state.visual_target_scale) - float(self.state.visual_scale_tol)
                    )
                    if center_ok and scale_ok:
                        print(
                            "[Visual] servo step %d/%d | done | uv=(%.3f, %.3f) scale=%.3f"
                            % (
                                step_idx,
                                max_steps,
                                float(current_obs.center_uv[0]),
                                float(current_obs.center_uv[1]),
                                float(current_obs.scale),
                            )
                        )
                        self.state.set_visual_status(
                            running=False,
                            failed=False,
                            msg=("%s | uv=(%.3f, %.3f) scale=%.3f"
                                 % ("centered" if center_only else "converged",
                                    float(current_obs.center_uv[0]),
                                    float(current_obs.center_uv[1]),
                                    float(current_obs.scale))),
                        )
                        return

                    linear_du, roll_du, seg_du = self._visual_candidate_delta(current_obs, center_only=center_only)
                    if center_only:
                        linear_du = 0.0
                        u_delta, v_delta, _, _ = self._visual_uv_errors(current_obs)
                        center_tol = float(self.state.visual_center_tol)
                        if abs(u_delta) > center_tol:
                            seg_du = 0.0
                        else:
                            roll_du = 0.0
                    if abs(linear_du) <= 1e-9 and abs(roll_du) <= 1e-9 and abs(seg_du) <= 1e-9:
                        self._log_visual_step(
                            "servo",
                            step_idx,
                            max_steps,
                            mode="deadband",
                            uv=f"({float(current_obs.center_uv[0]):+.3f},{float(current_obs.center_uv[1]):+.3f})",
                            scale=f"{float(current_obs.scale):.3f}",
                        )
                        self.state.set_visual_status(
                            running=False,
                            failed=False,
                            msg="within deadband | uv=(%.3f, %.3f) scale=%.3f"
                            % (float(current_obs.center_uv[0]), float(current_obs.center_uv[1]), float(current_obs.scale)),
                        )
                        return

                    self._log_visual_step(
                        "servo",
                        step_idx,
                        max_steps,
                        uv=f"({float(current_obs.center_uv[0]):+.3f},{float(current_obs.center_uv[1]):+.3f})",
                        scale=f"{float(current_obs.scale):.3f}",
                        dlinear=f"{linear_du:+.2f}",
                        droll=f"{roll_du:+.2f}",
                        dseg=f"{seg_du:+.2f}",
                    )
                    next_u = self._clamp_display_u(
                        ControlU(
                            u_linear=float(current_u.u_linear + linear_du),
                            u_roll=float(current_u.u_roll + roll_du),
                            u_s1=float(current_u.u_s1 + seg_du),
                            u_s2=float(current_u.u_s2 + seg_du),
                        )
                    )
                    if next_u == current_u:
                        self.state.set_visual_status(
                            running=False,
                            failed=True,
                            msg="command clamped | uv=(%.3f, %.3f) scale=%.3f"
                            % (float(current_obs.center_uv[0]), float(current_obs.center_uv[1]), float(current_obs.scale)),
                        )
                        return
                    host_now = self._send_display_control_u_and_wait(next_u, timeout_s=1.0, source="ik")
                    current_u = next_u
                    latest_obs = self.current_visual_observation(host_now)
                    if latest_obs is None:
                        stale_count += 1
                        if stale_count >= 2:
                            self.state.set_visual_status(running=False, failed=True, msg="stale observation")
                            return
                    else:
                        stale_count = 0
                        current_obs = latest_obs
                    self.state.set_visual_status(
                        running=True,
                        failed=False,
                        msg="%s | uv=(%.3f, %.3f) scale=%.3f conf=%.2f"
                        % (
                            ("centering u" if center_only and abs(self._visual_uv_errors(current_obs)[0]) > float(self.state.visual_center_tol) else
                             "centering v" if center_only else
                             "tracking"),
                            float(current_obs.center_uv[0]),
                            float(current_obs.center_uv[1]),
                            float(current_obs.scale),
                            float(current_obs.confidence),
                        ),
                    )
                print(f"[Visual] servo | iteration limit ({max_steps} steps)")
                self.state.set_visual_status(running=False, failed=True, msg="iteration limit")
            except Exception as exc:
                print(f"[Visual] failed: {exc}")
                self.state.set_visual_status(running=False, failed=True, msg=f"visual servo failed: {exc}")
            finally:
                self._visual_worker = None

        self._visual_worker = threading.Thread(target=_worker, daemon=True)
        self._visual_worker.start()

    def start_visual_servo(self) -> None:
        self._start_visual_controller(center_only=False)

    def start_visual_centering(self) -> None:
        self._start_visual_controller(center_only=True)

    def request_ports(self) -> None:
        if self.client is not None:
            self.client.request_ports()

    def set_device(self, device: str) -> None:
        if self.client is not None:
            self.client.set_device(device)

    def disconnect_device(self) -> None:
        if self.client is not None:
            self.client.disconnect_device()

    def torque_on(self) -> None:
        if self.client is not None:
            self.client.torque_on()

    def torque_off(self) -> None:
        if self.client is not None:
            self.client.torque_off()

    def perception_snapshot(self) -> Optional[PerceptionSnapshot]:
        cap = self._perception_capture
        return None if cap is None else cap.snapshot()

    def _on_perception_snapshot(self, snap: PerceptionSnapshot) -> None:
        world_xyz = snap.p_world
        if bool(self.state.pick_running) and world_xyz is None:
            world_xyz = self.state.perception_world_xyz
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
        )

    def _pick_config_effective(self) -> PickConfig:
        pk = self._pick_cfg
        return PickConfig(
            enabled=bool(pk.enabled),
            target_scale=float(self.state.visual_target_scale),
            scale_tol=float(self.state.visual_scale_tol),
            center_tol=float(self.state.visual_center_tol),
            target_uv_u=float(self.state.visual_target_uv_u),
            target_uv_v=float(self.state.visual_target_uv_v),
            linear_step_u=float(pk.linear_step_u),
            linear_gain=float(pk.linear_gain),
            max_iters=int(pk.max_iters),
            require_track_frames=int(pk.require_track_frames),
            acquire_timeout_s=float(pk.acquire_timeout_s),
        )

    def _wait_for_track_lock(self, *, timeout_s: float, require_frames: int) -> bool:
        deadline = time.time() + max(float(timeout_s), 0.1)
        while time.time() < deadline:
            if self._pick_stop_event.is_set():
                return False
            cap = self._perception_capture
            if cap is not None and cap.track_ok_frames() >= int(require_frames):
                if cap.tracker_phase() == TrackerPhase.TRACK.value:
                    return True
            time.sleep(0.05)
        return False

    def _apply_center_uv_step(
        self,
        obs: VisualObservation,
        current_u: ControlU,
        *,
        center_tol: float,
        force_axis: Optional[str] = None,
        seg_u_max: Optional[float] = None,
        u_active_tol: Optional[float] = None,
        target_uv: Optional[tuple[float, float]] = None,
    ) -> tuple[ControlU, str]:
        roll_du = 0.0
        seg_du = 0.0
        if target_uv is None:
            tu, tv = self._visual_target_uv()
        else:
            tu, tv = float(target_uv[0]), float(target_uv[1])
        u = float(obs.center_uv[0])
        v = float(obs.center_uv[1])
        u_delta = u - tu
        v_delta = v - tv
        u_err, v_err = -u_delta, -v_delta
        mode = "none"
        u_tol = float(u_active_tol if u_active_tol is not None else center_tol)
        u_over = abs(u_delta) > u_tol
        v_over = abs(v_delta) > float(center_tol)
        seg_cap = float(self._visual_center_seg_u_max if seg_u_max is None else seg_u_max)
        if force_axis == "u" or (force_axis is None and u_over):
            roll_du = float(
                np.clip(
                    self._visual_center_u_gain * u_err,
                    -self._visual_center_roll_u_max,
                    self._visual_center_roll_u_max,
                )
            )
            mode = "uv_roll"
        elif force_axis == "v" or (force_axis is None and v_over):
            seg_du = float(
                np.clip(
                    self._visual_center_v_gain * v_err,
                    -seg_cap,
                    seg_cap,
                )
            )
            mode = "uv_seg"
        next_u = self._clamp_display_u(
            ControlU(
                u_linear=float(current_u.u_linear),
                u_roll=float(current_u.u_roll + roll_du),
                u_s1=float(current_u.u_s1 + seg_du),
                u_s2=float(current_u.u_s2 + seg_du),
            )
        )
        if next_u == current_u:
            mode = "none"
        return next_u, mode

    def _apply_pick_center_step(self, obs: VisualObservation, current_u: ControlU) -> ControlU:
        cfg = self._pick_config_effective()
        center_tol = float(cfg.center_tol)
        tu, tv = float(cfg.target_uv_u), float(cfg.target_uv_v)
        next_u, _mode = self._apply_center_uv_step(
            obs,
            current_u,
            center_tol=center_tol,
            u_active_tol=center_tol * float(self._center_u_enter_v_ratio),
            seg_u_max=float(self._center_seg_step_max),
            target_uv=(tu, tv),
        )
        return next_u

    def _wait_center_observation(
        self,
        *,
        prev_ts: float,
        timeout_s: float = 2.0,
    ) -> tuple[Optional[HostState], Optional[VisualObservation]]:
        deadline = time.time() + float(timeout_s)
        while time.time() < deadline:
            if self._visual_stop_event.is_set():
                return None, None
            time.sleep(0.05)
            polled_host = self.client.refresh_state() if self.client is not None else None
            polled_obs = self.current_visual_observation(polled_host)
            if polled_obs is None:
                continue
            if float(polled_obs.timestamp_s) <= float(prev_ts) + 1e-6:
                continue
            return polled_host, polled_obs
        return None, None

    def _apply_pick_approach_step(self, obs: VisualObservation, current_u: ControlU) -> ControlU:
        cfg = self._pick_config_effective()
        scale_err = float(cfg.target_scale) - float(obs.scale)
        linear_du = 0.0
        if scale_err > float(cfg.scale_tol):
            # Display u_linear→0 is forward (see protocol linear mapping + command_direction).
            linear_du = -float(
                np.clip(
                    float(cfg.linear_gain) * scale_err,
                    0.0,
                    float(cfg.linear_step_u),
                )
            )
        return self._clamp_display_u(
            ControlU(
                u_linear=float(current_u.u_linear + linear_du),
                u_roll=float(current_u.u_roll),
                u_s1=float(current_u.u_s1),
                u_s2=float(current_u.u_s2),
            )
        )

    def stop_object_pick(self) -> None:
        self._pick_stop_event.set()
        self.state.set_pick_status(running=False, failed=False, phase=ObjectPickPhase.IDLE.value, msg="stopped")

    def start_object_pick(self) -> None:
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
                    "[Pick] start | max_iters=%d target_scale=%.3f center_tol=%.3f target_uv=(%+.3f,%+.3f)"
                    % (
                        int(pk.max_iters),
                        float(pk.target_scale),
                        float(pk.center_tol),
                        float(pk.target_uv_u),
                        float(pk.target_uv_v),
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

                current_u = self.current_control_u()
                stale_count = 0
                max_iters = int(pk.max_iters)
                for it in range(max_iters):
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
                            self.state.set_pick_status(
                                running=False,
                                failed=True,
                                phase=ObjectPickPhase.FAILED.value,
                                msg="observation lost",
                            )
                            return
                        time.sleep(0.05)
                        continue
                    stale_count = 0

                    conv = evaluate_pick_convergence(obs, cfg=pk)
                    if conv.center_ok and conv.scale_ok:
                        print(
                            "[Pick] step %d/%d | done | uv=(%.3f, %.3f) scale=%.3f"
                            % (step_idx, max_iters, conv.u_err, conv.v_err, conv.scale)
                        )
                        self.state.set_pick_status(
                            running=False,
                            failed=False,
                            phase=ObjectPickPhase.DONE.value,
                            msg="pick ready | uv=(%.3f, %.3f) scale=%.3f (grasp manual)"
                            % (conv.u_err, conv.v_err, conv.scale),
                        )
                        return

                    if not conv.center_ok:
                        phase = ObjectPickPhase.CENTER
                        next_u = self._apply_pick_center_step(obs, current_u)
                    else:
                        phase = ObjectPickPhase.APPROACH
                        next_u = self._apply_pick_approach_step(obs, current_u)

                    du_linear = float(next_u.u_linear - current_u.u_linear)
                    du_roll = float(next_u.u_roll - current_u.u_roll)
                    du_seg = float(next_u.u_s1 - current_u.u_s1)
                    self._log_visual_step(
                        "pick",
                        step_idx,
                        max_iters,
                        phase=phase.value,
                        uv=f"({conv.u_err:+.3f},{conv.v_err:+.3f})",
                        scale=f"{conv.scale:.3f}",
                        center_ok=str(conv.center_ok),
                        scale_ok=str(conv.scale_ok),
                        dlinear=f"{du_linear:+.2f}",
                        droll=f"{du_roll:+.2f}",
                        dseg=f"{du_seg:+.2f}",
                    )

                    if next_u == current_u:
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
                        if conv.center_ok and conv.scale_ok:
                            self.state.set_pick_status(
                                running=False,
                                failed=False,
                                phase=ObjectPickPhase.DONE.value,
                                msg="pick ready | uv=(%.3f, %.3f) scale=%.3f (grasp manual)"
                                % (conv.u_err, conv.v_err, conv.scale),
                            )
                            return
                        self.state.set_pick_status(
                            running=False,
                            failed=True,
                            phase=ObjectPickPhase.FAILED.value,
                            msg="command clamped | uv=(%.3f, %.3f) scale=%.3f"
                            % (conv.u_err, conv.v_err, conv.scale),
                        )
                        return

                    self.state.set_pick_status(
                        running=True,
                        failed=False,
                        phase=phase.value,
                        msg="%s | uv=(%.3f, %.3f) scale=%.3f"
                        % (phase.value, conv.u_err, conv.v_err, conv.scale),
                    )
                    self._send_display_control_u_and_wait(next_u, timeout_s=1.0, source="ik")
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
        publish_depth = bool(depth_valid) and not freeze_world
        p_world = self.client.send_perception_observation(
            object_camera_xyz=object_camera_xyz,
            label=label,
            confidence=confidence,
            image_center_uv=image_center_uv,
            image_scale=image_scale,
            depth_valid=publish_depth,
        )
        if freeze_world:
            frozen = self.state.perception_world_xyz
            return frozen if frozen is not None else p_world
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
        )
        self.state.set_perception_status(running=True, failed=False, msg="starting")
        self._perception_capture.start()

    def stop_perception_capture(self) -> None:
        cap = self._perception_capture
        if cap is not None:
            cap.stop()
        self.state.set_perception_status(running=False, failed=False, msg="stopped")

    def update_perception_config(self, config: PerceptionConfig) -> None:
        self._perception_cfg = config
        self.state.visual_target_label = str(config.target_label).strip()

    def close(self) -> None:
        self._visual_stop_event.set()
        self.stop_object_pick()
        self.stop_perception_capture()
        if self.client is not None:
            self.client.close()
