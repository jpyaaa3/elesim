from __future__ import annotations

import math
import os
import threading
import time
from typing import Any, Optional

import numpy as np

from engine import ik as ik_pipeline
from engine.config_loader import IkConfig, PerceptionConfig, PickConfig
from engine.protocol import ControlU, SimMappingConfig, SimQ, control_u_to_sim_q, sim_q_to_control_u
from engine.sag_model import load_sag_model_json

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

    def _visual_busy(self) -> bool:
        return self._ik_worker is not None or self._pick_worker is not None

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

    def _send_state_q_and_wait(
        self,
        *,
        timeout_s: float = 1.0,
        source: str = "ik",
        force: bool = False,
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
        self.send_current_target(source=source, force=force)
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

    def _command_q_and_wait(
        self,
        q: np.ndarray,
        *,
        timeout_s: float = 1.0,
        source: str = "slider",
        force: bool = False,
    ) -> Optional[HostState]:
        q_cmd = self._clamp_q(q)
        self.state.set_q(float(q_cmd[0]), float(q_cmd[1]), float(q_cmd[2]), float(q_cmd[3]))
        return self._send_state_q_and_wait(
            timeout_s=float(timeout_s), source=source, force=force
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
        return self._send_state_q_and_wait(
            timeout_s=float(timeout_s), source="ik", force=True
        )

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
        self.apply_control_u(u_linear=15.0, u_roll=180.0, u_s1=10.0, u_s2=10.0, apply_offset=False)
        self.send_current_target(source="slider")

    def extend_arm_controls(self) -> None:
        self.state.clear_ik_status()
        self.apply_control_u(u_linear=15.0, u_roll=180.0, u_s1=180.0, u_s2=180.0, apply_offset=False)
        self.send_current_target(source="slider")

    def send_current_target(self, *, source: str, force: bool = False) -> None:
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
                sag_model=(dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {}),
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

    def start_ready_pose(self) -> None:
        if self.state.ik_running or self._visual_busy():
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="busy",
            )
            return
        if self.client is not None:
            self.client.refresh_state()
        object_world = self._pick_frozen_world()
        if object_world is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="no object world coordinate",
            )
            return

        direction = (
            float(self.state.visual_target_dir_x),
            float(self.state.visual_target_dir_y),
            float(self.state.visual_target_dir_z),
        )
        pk = self._pick_config_effective()
        try:
            target = compute_ready_pose_target(
                tuple(float(v) for v in object_world),
                direction,
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

        self.state.set_target(float(target[0]), float(target[1]), float(target[2]))
        self.state.set_target_dir(float(direction[0]), float(direction[1]), float(direction[2]))
        if self.client is not None:
            self.client.send_debug_markers(
                [
                    {
                        "name": "ready_pose",
                        "frame": "world",
                        "pos": [float(target[0]), float(target[1]), float(target[2])],
                        "dir": [float(direction[0]), float(direction[1]), float(direction[2])],
                        "color": [0.72, 1.0, 0.28, 0.95],
                        "radius": 0.014,
                        "ttl_ms": 30000,
                    }
                ],
                source="target",
            )
        self.state.set_pick_status(
            running=False,
            failed=False,
            phase=ObjectPickPhase.IDLE.value,
            msg=(
                "ready pose | object=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f) standoff=%.0fmm"
                % (
                    float(object_world[0]),
                    float(object_world[1]),
                    float(object_world[2]),
                    float(target[0]),
                    float(target[1]),
                    float(target[2]),
                    float(pk.ready_pose_standoff_m) * 1000.0,
                )
            ),
        )
        print(
            "[Pick] ready pose | object=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f) "
            "dir=(%.3f, %.3f, %.3f) standoff=%.0fmm"
            % (
                float(object_world[0]),
                float(object_world[1]),
                float(object_world[2]),
                float(target[0]),
                float(target[1]),
                float(target[2]),
                float(direction[0]),
                float(direction[1]),
                float(direction[2]),
                float(pk.ready_pose_standoff_m) * 1000.0,
            )
        )
        self._start_position_solve(np.asarray(target, dtype=float))

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
            ready_pose_standoff_m=float(pk.ready_pose_standoff_m),
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
        )

    def _pick_reach_model(self):
        from engine.iklib.kinematics import _ReachModel

        self.refresh_ik_context()
        limit = self._ik_context.get("limit")
        if limit is None:
            raise RuntimeError("ik context missing joint limit")
        return _ReachModel(context=dict(self._ik_context), limit=limit)

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
        self._send_display_control_u_and_wait(next_u, timeout_s=1.0, source="ik")
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
            model = self._pick_reach_model()
        except Exception as exc:
            print(f"[Pick] extend | IK model unavailable: {exc}")
            return 0.0

        q0 = self._q_array_from_state(host_state)
        tip0 = np.asarray(model.grasp_position(q0), dtype=float).reshape(3)
        axis_w = self._pick_ee_axis_world(model, q0, axis_local=(0.0, 0.0, -1.0))
        target = tip0 + axis_w * delta
        dir_hold = np.asarray(model.grasp_direction(q0), dtype=float).reshape(3)

        self.refresh_ik_context()
        ctx = dict(self._ik_context)
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

        align_msg = "pick extend | local-Z %.0fmm" % (delta * 1000.0)
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
            "[Pick] extend | solve_then_align | dist=%.0fmm travel=%.0fmm prog=%.0f/%.0fmm "
            "| target=(%.3f, %.3f, %.3f)"
            % (
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
        while time.time() < deadline:
            if self._pick_stop_event.is_set():
                return False
            cap = self._perception_capture
            if cap is not None and cap.track_ok_frames() >= int(require_frames):
                if cap.tracker_phase() == TrackerPhase.TRACK.value:
                    return True
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
        self, obs: VisualObservation, current_u: ControlU
    ) -> tuple[ControlU, str, float, float]:
        cfg = self._pick_config_effective()
        center_tol = float(cfg.center_tol)
        tu, tv = float(cfg.target_uv_u), float(cfg.target_uv_v)
        u = float(obs.center_uv[0])
        v = float(obs.center_uv[1])
        u_delta = u - tu
        v_delta = v - tv
        # Same UV band as evaluate_pick_convergence (avoid center_ok False while still correcting).
        u_over = abs(u_delta) > float(center_tol)
        v_over = abs(v_delta) > float(center_tol)
        seg_cap = float(cfg.center_seg_max)
        roll_cap = float(cfg.center_roll_max)
        u_gain = float(cfg.center_u_gain)
        v_gain = float(cfg.center_v_gain)
        err_scale_max = float(cfg.center_error_scale_max)
        if not u_over and not v_over:
            return current_u, "none", 0.0, 0.0

        roll_du = 0.0
        seg_du = 0.0
        if u_over:
            u_err = float(tu - u)
            roll_scale = float(
                np.clip(
                    abs(u_delta) / max(float(center_tol), 1e-6), 0.5, err_scale_max
                )
            )
            roll_du = float(
                np.clip(u_gain * u_err * roll_scale, -roll_cap, roll_cap)
            )
        if v_over:
            seg_du = float(
                self._center_seg_du(
                    target_v=tv, obs_v=v, cap=seg_cap, gain=v_gain
                )
            )

        next_u = self._clamp_display_u(
            ControlU(
                u_linear=float(current_u.u_linear),
                u_roll=float(current_u.u_roll + roll_du),
                u_s1=float(current_u.u_s1 + seg_du),
                u_s2=float(current_u.u_s2 + seg_du),
            )
        )
        if next_u == current_u:
            return current_u, "none", roll_du, seg_du
        if u_over and v_over:
            mode = "pick_uv"
        elif u_over:
            mode = "uv_roll"
        else:
            mode = "uv_seg"
        return next_u, mode, roll_du, seg_du

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
        self._latch_pick_frozen_world()
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
                    u_d, v_d, _, _ = self._visual_uv_errors(obs)
                    if conv.center_ok:
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
