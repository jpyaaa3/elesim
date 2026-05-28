from __future__ import annotations

import math
import os
import threading
import time
from typing import Any, Optional

import numpy as np

from engine import ik as ik_pipeline
from engine.config_loader import IkConfig
from engine.protocol import ControlU, SimMappingConfig, SimQ, control_u_to_sim_q, sim_q_to_control_u
from engine.sag_model import load_sag_model_json

from .client import ControlClient
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
    ) -> None:
        self.state = state
        self.client = client
        self._mapping_cfg = mapping_cfg or SimMappingConfig()
        self._ik_cfg = ik_cfg or IkConfig()
        self._ik_context = dict(ik_context or {})
        self._config_path = None if config_path is None else str(config_path)
        self._ik_worker: Optional[threading.Thread] = None
        self._calibration_current_threshold_ma = 1400
        self._calibration_current_delta_ma = 500
        self._calibration_abort_current_ma = 2000
        self._calibration_step_u = 2.0
        self._calibration_poll_s = 0.15
        self._calibration_ema_alpha = 0.25
        self._calibration_release_consecutive = 3
        self._calibration_baseline_samples = 3
        # Host motor_currents_ma keys use seg1/seg2; control-u axes use s1/s2.
        self._calibration_current_keys = {
            "s1": ("s1", "seg1"),
            "s2": ("s2", "seg2"),
        }

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
        if self.state.ik_running or self._ik_worker is not None:
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

    def _refresh_calibration_feedback(self, axis: str) -> tuple[Optional[HostState], Optional[int]]:
        if self.client is None:
            return None, None
        time.sleep(self._calibration_poll_s)
        host_state: Optional[HostState] = None
        current_val: Optional[int] = None
        for _ in range(3):
            host_state = self.client.refresh_state()
            if host_state is not None:
                current_val = self._calibration_current_for_axis(host_state, axis)
            time.sleep(0.02)
        return host_state, current_val

    def _calibration_contact_threshold_ma(self, baseline_ma: float) -> float:
        relative = float(baseline_ma) + float(self._calibration_current_delta_ma)
        absolute = float(self._calibration_current_threshold_ma)
        return float(max(relative, absolute * 0.75))

    def _calibration_measure_baseline(self, axis: str, display_u: float) -> float:
        self.apply_partial_control_u({axis: float(display_u)})
        samples: list[float] = []
        for _ in range(int(self._calibration_baseline_samples)):
            _host_state, current_val = self._refresh_calibration_feedback(axis)
            if current_val is not None:
                samples.append(float(current_val))
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
        threshold_ma: float,
    ) -> tuple[float, float, int]:
        step = float(self._calibration_step_u)
        direction = int(self._calibration_probe_display_direction(axis))
        display_u = float(start_u)
        ema: Optional[float] = None
        self.apply_partial_control_u({axis: display_u})
        _host_state, current_val = self._refresh_calibration_feedback(axis)
        if _host_state is None or current_val is None:
            raise RuntimeError(f"missing {axis} current feedback")
        ema = self._calibration_update_ema(ema, float(current_val))
        while True:
            next_u = float(display_u) + float(direction) * step
            if direction < 0 and next_u < float(lo) - 1e-9:
                break
            if direction > 0 and next_u > float(hi) + 1e-9:
                break
            display_u = float(max(lo, min(hi, next_u)))
            self.apply_partial_control_u({axis: display_u})
            _host_state, current_val = self._refresh_calibration_feedback(axis)
            if _host_state is None or current_val is None:
                raise RuntimeError(f"missing {axis} current feedback")
            ema = self._calibration_update_ema(ema, float(current_val))
            if float(ema) >= float(self._calibration_abort_current_ma):
                raise RuntimeError(f"{axis} current too high during calibration")
            if float(ema) >= float(threshold_ma):
                return float(display_u), float(ema), direction
        raise RuntimeError(f"no current rise on {axis}")

    def _calibration_release_axis(
        self,
        axis: str,
        *,
        contact_u: float,
        lo: float,
        hi: float,
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
            _host_state, current_val = self._refresh_calibration_feedback(axis)
            if _host_state is None or current_val is None:
                raise RuntimeError(f"missing {axis} current feedback")
            ema = self._calibration_update_ema(ema, float(current_val))
            if float(ema) >= float(self._calibration_abort_current_ma):
                raise RuntimeError(f"{axis} current too high during release")
            if float(ema) < float(threshold_ma):
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
        if self._ik_worker is not None:
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
                        threshold_ma=threshold_ma,
                    )
                    self.state.set_calibration_status(running=True, msg=f"releasing {axis}")
                    release_display = self._calibration_release_axis(
                        axis,
                        contact_u=contact_u,
                        lo=lo,
                        hi=hi,
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
        if self.state.ik_running or self._ik_worker is not None:
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

    def close(self) -> None:
        if self.client is not None:
            self.client.close()
