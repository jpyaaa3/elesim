from __future__ import annotations

import math
import os
import threading
import time
from collections import deque
from dataclasses import replace
from pathlib import Path
from time import perf_counter
from typing import Any, Optional, Sequence

import numpy as np

from engine.robot.arm import ik as ik_pipeline
from engine.robot.arm.mounts.go2_mount import Go2ArmMount
from engine.robot.arm.iklib import kinematics as ik_kin
from engine.core.config_loader import IkConfig, PerceptionConfig, PickConfig, SimConfig, load_app_config_from_ini
from engine.behaviors.gaze.stabilizer import GazeStabilizerConfig, patch_gaze_config
from engine.behaviors.gaze.gaze_service import GazeControlService
from engine.core.protocol import (
    ControlU,
    DEFAULT_START_CONTROL_U,
    SimMappingConfig,
    SimQ,
    control_u_to_sim_q,
    default_start_sim_q,
    linear_effective_q_bounds,
    linear_motor_u_limit,
    sim_q_to_control_u,
)
from engine.experiment.walking_trial import host_horizontal_object_distance_m, standoff_base_pos
from engine.robot.arm.sag_model import load_sag_model_json
from engine.vision.visual_servoing.equal_sag_probe import (
    EqualSagEstimate,
    SagDriftComponents,
    apply_equal_sag_offsets,
    estimate_equal_sag_from_ready_pose_drift,
    prepare_sag_drift_input,
)
from engine.observability.pick_timing import (
    PickPhaseProfile,
    PickTimingCollector,
    enabled as pick_profile_enabled,
    fk_call_count,
    format_report,
    install_fk_counter,
    reset_fk_count,
    uninstall_fk_counter,
)
from engine.vision.visual_servoing.feasible_ready_pose import resolve_feasible_ready_pose
from engine.vision.visual_servoing.grasp_trajectory import (
    GraspWaypoint,
    build_grasp_trajectory_markers,
)
from engine.vision.visual_servoing.local_image_jacobian import (
    GraspApproachMode,
    ImageJacobianEstimator3D,
    LocalImageJacobianServo3D,
    LocalImageJacobianServoGains,
    SampleRejectReason,
    check_sample_quality,
    default_j_lji_seed,
    joint_saturated,
    z_jacobian_row_from_position_jacobian,
)
from engine.vision.visual_servoing.uv_jacobian import (
    broyden_update_uv_jacobian,
    default_uv_jacobian,
    solve_uv_control_delta,
)

from .client import ControlClient
from engine.vision.perception.observation import VisualObservation, extract_local_perception_observation, extract_visual_observation
from engine.vision.pick.core import (
    ObjectPickPhase,
    compute_ready_pose_target,
    evaluate_pick_convergence,
    pick_ready_for_extend,
    pick_uv_deltas,
)
from engine.vision.perception.capture import (
    PerceptionCapture,
    PerceptionSnapshot,
    TrackerPhase,
    default_perception_capture_dir,
    save_perception_frame_bundle,
    _ensure_pick_place_path,
)
from .state import HostState, PanelState


DEFAULT_SAG_MODEL_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
    "model_presets",
    "sag",
    "sag_model.json",
)


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
        gaze_cfg: Optional[GazeStabilizerConfig] = None,
        ownership_enable: bool = False,
        hand_eye_transform: Optional[np.ndarray] = None,
        hand_eye_parent_frame: str = "node9",
        go2_arm_mount: Optional[Go2ArmMount] = None,
        use_hardware: bool = True,
        remote_gaze_delegate: bool = True,
    ) -> None:
        self.state = state
        self.client = client
        self._use_hardware = bool(use_hardware)
        self._mapping_cfg = mapping_cfg or SimMappingConfig()
        self._ik_cfg = ik_cfg or IkConfig()
        self._ik_context = dict(ik_context or {})
        self._config_path = None if config_path is None else str(config_path)
        self._perception_cfg = perception_cfg or PerceptionConfig()
        self._perception_run_local = self._perception_config_runs_locally(self._perception_cfg)
        self._remote_gaze_delegate = bool(remote_gaze_delegate)
        self._pick_cfg = pick_cfg or PickConfig()
        self._hand_eye_transform = (
            None
            if hand_eye_transform is None
            else np.asarray(hand_eye_transform, dtype=float).reshape(4, 4).copy()
        )
        self._hand_eye_parent_frame = str(hand_eye_parent_frame)
        self._go2_arm_mount = go2_arm_mount
        self._perception_capture: Optional[PerceptionCapture] = None
        self._perception_capture_epoch: int = 0
        self._perception_rate_last_t: float = 0.0
        self._perception_rate_last_frame_idx: int = -1
        self._perception_hz: float = 0.0
        self._side_camera_recorder: Optional[Any] = None
        self._side_camera_record_path: Optional[Path] = None
        self._remote_preview_stop = threading.Event()
        self._remote_preview_thread: Optional[threading.Thread] = None
        self._last_pick_profile: Optional[PickPhaseProfile] = None
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
        self._grasp_executed_waypoints: list[GraspWaypoint] = []
        self._grasp_traj_start: Optional[tuple[float, float, float]] = None
        self._grasp_look_anchor: Optional[tuple[float, float, float]] = None
        self._grasp_handoff_look_dir: Optional[tuple[float, float, float]] = None
        self._grasp_object_world_filtered: Optional[tuple[float, float, float]] = None
        self._grasp_approach_dir_filtered: Optional[tuple[float, float, float]] = None
        self._grasp_uv_only_mode: bool = False
        self._grasp_approach_mode = GraspApproachMode.LOCAL_IMG_JACOBIAN
        self._grasp_lji_estimator_3d: Optional[ImageJacobianEstimator3D] = None
        self._grasp_lji_servo_3d: Optional[LocalImageJacobianServo3D] = None
        self._grasp_lji_frozen_sag_model: Optional[dict[str, Any]] = None
        self._grasp_depth_history: deque[tuple[bool, float, float]] = deque(maxlen=32)
        self._grasp_lji_object_lost_count = 0
        self._grasp_lji_last_reliable_object_world: Optional[tuple[float, float, float]] = None
        self._grasp_lji_last_reliable_approach_dir: Optional[tuple[float, float, float]] = None
        self._grasp_lji_last_reliable_depth: Optional[float] = None
        self._grasp_lji_last_good_q: Optional[np.ndarray] = None
        self._grasp_lji_pending_sample: Optional[dict[str, Any]] = None
        self._grasp_lji_last_dq_cmd: Optional[np.ndarray] = None
        self._grasp_lji_reacquire_anchor_dq: Optional[np.ndarray] = None
        self._grasp_lji_reacquire_steps = 0
        self._grasp_lji_reacquire_aim_tried = False
        self._grasp_lji_reacquire_prev_remain: Optional[float] = None
        self._grasp_lji_v_err_hist: list[float] = []
        self._grasp_lji_last_transition: str = "-"
        self._grasp_lji_sat_streak = 0
        self._grasp_lji_remain_hist: list[float] = []
        self._pick_search_origin_u: Optional[ControlU] = None
        self._pick_search_step_index = 0
        self._pick_search_max_steps = 48
        self._pick_aim_step_scale = 0.45
        self._pick_aim_command_timeout_s = 1.0
        self._pick_aim_settle_s = 0.08
        self._pick_aim_gain_fallback_uv = 0.35
        self._pick_aim_taper_ref_uv = 0.35
        self._pick_aim_taper_min = 0.20
        self._pick_aim_v_min_seg_step = 0.8
        self._pick_aim_v_only_gain_scale = 1.25
        self._pick_aim_progress_eps = 0.015
        self._pick_aim_stuck_iters = 0
        self._pick_aim_best_uv_err: Optional[float] = None
        self._pick_aim_jacobian_resets = 0
        self._pick_aim_jacobian_reset_max = 2
        self._pick_aim_runtime_step_scale = float(self._pick_aim_step_scale)
        self._pick_aim_step_scale_min = 0.12
        self._pick_aim_diverge_ratio = 1.20
        self._pick_aim_diverge_abs = 0.04
        self._pick_aim_diverge_count = 0
        self._pick_aim_last_command_u: Optional[ControlU] = None
        self._pick_aim_last_command_err: Optional[float] = None
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
        self._visual_obs_stale_s = 0.75
        self._gaze_cfg = gaze_cfg or GazeStabilizerConfig()
        self._gaze_service = GazeControlService(
            self,
            self._gaze_cfg,
            ownership_enable=bool(ownership_enable),
        )
        self._gaze_prev_uv_err: Optional[tuple[float, float]] = None
        self._gaze_dv_err_rate_filt: float = 0.0
        self._gaze_last_cmd_wall_s: float = 0.0
        self._gaze_last_sent_du_mag: float = 0.0

    @property
    def gaze_config(self) -> GazeStabilizerConfig:
        return self._gaze_cfg

    def update_gaze_stabilizer_config(
        self,
        patch: dict[str, Any] | GazeStabilizerConfig,
        *,
        send_remote: bool = True,
    ) -> GazeStabilizerConfig:
        if isinstance(patch, GazeStabilizerConfig):
            next_cfg = patch
            outbound_patch = {}
        else:
            outbound_patch = dict(patch)
            next_cfg = patch_gaze_config(self._gaze_cfg, outbound_patch)
        self._gaze_cfg = next_cfg
        self._gaze_service.update_config(next_cfg)
        if (
            bool(send_remote)
            and outbound_patch
            and self._delegate_gaze_to_host()
            and hasattr(self.client, "send_gaze_config_update")
        ):
            self.client.send_gaze_config_update(outbound_patch)
        return next_cfg

    def _gaze_busy(self) -> bool:
        return self._gaze_service.is_running

    def _delegate_gaze_to_host(self) -> bool:
        return (
            bool(self._remote_gaze_delegate)
            and not bool(self._perception_run_local)
            and self.client is not None
        )

    def _wait_until_q_settled(
        self,
        target_q: np.ndarray,
        *,
        timeout_s: float = 1.0,
        linear_tol_m: float = 2e-3,
        angle_tol_rad: float = math.radians(2.0),
        consecutive: int = 3,
    ) -> tuple[Optional[HostState], bool]:
        """Poll host q until it matches ``target_q``. Returns (state, settled_ok)."""
        if self.client is None:
            time.sleep(0.15)
            return None, False
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
            if state is not None:
                q_now = self._q_array_for_motion_feedback(state)
                if _is_settled(q_now):
                    return state, True

        while time.time() < deadline:
            time.sleep(float(poll_s))
            state = self.client.refresh_state()
            last_state = state
            if state is None:
                continue
            q_now = self._q_array_for_motion_feedback(state)
            if _is_settled(q_now):
                stable_count += 1
                if stable_count >= required_stable:
                    return state, True
            else:
                stable_count = 0
        return last_state, False

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
            model = self._pick_reach_model(sag_model=sag_model, host_state=host_state)
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
        nominal_world: Optional[tuple[float, float, float]] = None,
        approach_dir: Optional[np.ndarray] = None,
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
        axial_gate_m = max(close_tol_m * 3.0, 0.008)
        reached = False
        axial_arrival = False
        arrival_err_m = float("inf")
        settle_state: Optional[HostState] = host_state
        if nominal_world is not None and approach_dir is not None:
            grasp_pos = self._grasp_position_from_host_state(
                host_state,
                sag_model=sag_model,
            )
            if grasp_pos is not None:
                axial_remain = self._grasp_axial_distance(
                    tuple(float(v) for v in grasp_pos),
                    nominal_world,
                    approach_dir,
                )
                arrival_err_m = float(axial_remain)
                if float(axial_remain) <= axial_gate_m + 1e-4:
                    reached = True
                    axial_arrival = True
                    print(
                        "[Pick] %s | axial pre-contact ok | remain=%.1fmm (gate %.1fmm)"
                        % (
                            str(label),
                            float(axial_remain) * 1000.0,
                            float(axial_gate_m) * 1000.0,
                        )
                    )
        if not bool(reached):
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

        if settle_state is not None and not bool(axial_arrival):
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

    def _grasp_step_position_tol_m(self) -> float:
        """Looser position tol for continuum axial steps (default ik tol is 0.1 mm)."""
        return float(max(float(self._ik_cfg.tol), 0.003))

    def _grasp_align_ik_kwargs(self) -> dict[str, Any]:
        """Grasp IK: full direction align toward look-at (position along latched axis)."""
        pk = self._pick_config_effective()
        skip_deg = float(min(pk.grasp_waypoint_max_dir_error_deg, pk.ik_align_skip_under_deg))
        return {
            "align_mode": "full",
            "align_skip_under_deg": max(skip_deg, 1.0),
            "tweak_rounds": max(int(pk.ik_align_rounds), 2),
            "tweak_position_hold_tol_m": max(float(self._ik_cfg.tol), 1e-3),
        }

    def _grasp_fk_dir_error_deg(
        self,
        model: Any,
        q: np.ndarray,
        reference_dir: tuple[float, float, float] | np.ndarray,
    ) -> float:
        fk_dir = np.asarray(model.grasp_direction(q), dtype=float).reshape(3)
        ref = self._unit_vec3(reference_dir)
        fk_norm = float(np.linalg.norm(fk_dir))
        if fk_norm <= 1e-9:
            return float("inf")
        dot = float(np.clip(float(np.dot(fk_dir / fk_norm, ref)), -1.0, 1.0))
        return float(np.degrees(np.arccos(dot)))

    def _grasp_fk_look_at_error_deg(
        self,
        model: Any,
        q: np.ndarray,
        object_world: tuple[float, float, float] | np.ndarray,
    ) -> float:
        tip = np.asarray(model.grasp_position(q), dtype=float).reshape(3)
        look = self._grasp_look_at_dir(tip, object_world)
        return self._grasp_fk_dir_error_deg(model, q, look)

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

    def _current_arm_base_world_T(self, host_state: Optional[HostState] = None) -> Optional[np.ndarray]:
        mount = self._go2_arm_mount
        if mount is None:
            return None
        go2_pos, go2_rpy = mount.default_go2_pose()
        if host_state is not None:
            if host_state.go2_base_pos is not None:
                go2_pos = host_state.go2_base_pos
            if host_state.go2_base_rpy is not None:
                go2_rpy = host_state.go2_base_rpy
        return mount.arm_base_world_transform(go2_pos, go2_rpy)

    def _with_current_arm_base(
        self,
        context: dict[str, Any],
        host_state: Optional[HostState] = None,
    ) -> dict[str, Any]:
        ctx = dict(context)
        base_world_T = self._current_arm_base_world_T(host_state)
        if base_world_T is None:
            return ctx
        return ik_kin.with_base_world_transform(ctx, base_world_T)

    def _ik_context_for_host(
        self,
        host_state: Optional[HostState] = None,
        *,
        sag_model: Optional[dict[str, Any]] = None,
    ) -> dict[str, Any]:
        ctx = dict(self._ik_context)
        if sag_model is not None:
            ctx["sag_model"] = dict(sag_model)
        return self._with_current_arm_base(ctx, host_state)

    @staticmethod
    def _perception_config_runs_locally(config: PerceptionConfig) -> bool:
        provider = str(getattr(config, "provider", "")).strip().lower()
        if provider == "host":
            return False
        if not bool(getattr(config, "run_local", True)):
            return False
        return True

    def perception_run_local(self) -> bool:
        return bool(self._perception_run_local)

    def _maybe_start_local_perception(self) -> None:
        if not self._perception_run_local:
            return
        if self._perception_capture is None or not self._perception_capture.is_running():
            self.start_perception_capture()

    def _sync_remote_perception_from_host(self, host_state: HostState) -> None:
        if self._perception_run_local:
            return
        record_path = str(getattr(host_state, "perception_last_record_path", "") or "")
        self.state.set_perception_recording(
            bool(getattr(host_state, "perception_recording", False)),
            record_path,
        )
        capture_path = str(getattr(host_state, "perception_last_capture_path", "") or "")
        if capture_path:
            self.state.set_perception_last_capture(capture_path)
        if bool(getattr(host_state, "perception_recording", False)):
            self.state.set_perception_record_overlay(
                bool(getattr(host_state, "perception_record_with_overlay", False))
            )
        stale_s = float(self._visual_obs_stale_s)
        now = time.time()
        ts = float(host_state.perceived_timestamp_s)
        center_uv = host_state.perceived_center_uv
        scale = host_state.perceived_scale
        perception_hz = float(getattr(host_state, "perception_hz", 0.0))
        fresh = (
            center_uv is not None
            and scale is not None
            and ts > 0.0
            and (now - ts) <= stale_s
        )
        if fresh:
            self.state.set_perception_status(
                running=True,
                failed=bool(getattr(host_state, "perception_failed", False)),
                msg=str(getattr(host_state, "perception_status", "") or "remote Jetson worker"),
                frame_idx=1,
                label=str(host_state.perceived_object_label),
                confidence=float(host_state.perceived_object_confidence),
                camera_xyz=host_state.perceived_object_camera_xyz,
                image_scale=float(scale),
                center_uv=(float(center_uv[0]), float(center_uv[1])),
                perception_hz=perception_hz,
            )
            with self.state._lock:
                self.state.perception_last_update_s = float(ts)
            return
        worker_running = bool(getattr(host_state, "perception_running", False))
        worker_failed = bool(getattr(host_state, "perception_failed", False))
        worker_msg = str(getattr(host_state, "perception_status", "") or "").strip()
        if worker_running or worker_failed or worker_msg:
            self.state.set_perception_status(
                running=worker_running,
                failed=worker_failed,
                msg=worker_msg or "remote: waiting for detection",
                perception_hz=perception_hz,
            )
            return
        self.state.set_perception_status(
            running=False,
            failed=False,
            msg="remote: stopped",
            perception_hz=0.0,
        )

    def _sync_remote_gaze_from_host(self, host_state: HostState) -> None:
        if not self._delegate_gaze_to_host():
            return
        self.state.set_gaze_status(
            running=bool(getattr(host_state, "gaze_running", False)),
            mode=str(getattr(host_state, "gaze_mode", "") or "idle"),
            msg=str(getattr(host_state, "gaze_status_msg", "") or ""),
            u_err=float(getattr(host_state, "gaze_u_err", 0.0)),
            v_err=float(getattr(host_state, "gaze_v_err", 0.0)),
            du_roll=float(getattr(host_state, "gaze_du_roll", 0.0)),
            du_s1=float(getattr(host_state, "gaze_du_s1", 0.0)),
            du_s2=float(getattr(host_state, "gaze_du_s2", 0.0)),
            obs_age_s=float(getattr(host_state, "gaze_obs_age_s", -1.0)),
            tick_count=int(getattr(host_state, "gaze_tick_count", 0)),
            update_count=int(getattr(host_state, "gaze_update_count", 0)),
        )

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
        self._sync_remote_perception_from_host(host_state)
        self._sync_remote_gaze_from_host(host_state)
        return host_state

    def has_client(self) -> bool:
        return self.client is not None

    def current_host_state(self) -> Optional[HostState]:
        if self.client is None or not hasattr(self.client, "get_state"):
            return None
        return self.client.get_state()

    def current_visual_observation(self, host_state: Optional[HostState] = None) -> Optional[VisualObservation]:
        target_label = str(self.state.visual_target_label)
        stale_s = float(self._visual_obs_stale_s)
        min_conf = float(self.state.visual_confidence_min)
        state = host_state if host_state is not None else self.current_host_state()
        obs = extract_visual_observation(
            state,
            target_label=target_label,
            stale_timeout_s=stale_s,
            min_confidence=min_conf,
        )
        if obs is not None:
            return obs
        if not self._perception_run_local:
            return None
        st = self.state
        return extract_local_perception_observation(
            running=bool(st.perception_running),
            center_uv=st.perception_center_uv,
            image_scale=float(st.perception_image_scale),
            last_update_s=float(st.perception_last_update_s),
            label=str(st.perception_label),
            confidence=float(st.perception_confidence),
            target_label=target_label,
            stale_timeout_s=stale_s,
            min_confidence=min_conf,
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
        self._pick_aim_runtime_step_scale = float(self._pick_aim_step_scale)
        self._pick_aim_diverge_count = 0
        self._pick_aim_last_command_u = None
        self._pick_aim_last_command_err = None

    def _aim_error_diverged(self, err_mag: float) -> bool:
        prev = self._pick_aim_last_command_err
        if prev is None:
            return False
        threshold = max(
            float(prev) * float(self._pick_aim_diverge_ratio),
            float(prev) + float(self._pick_aim_diverge_abs),
        )
        return float(err_mag) > float(threshold)

    def _reduce_aim_step_scale(self) -> bool:
        old = float(self._pick_aim_runtime_step_scale)
        new = max(float(self._pick_aim_step_scale_min), old * 0.5)
        self._pick_aim_runtime_step_scale = new
        return new < old - 1e-9

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
        return self._ik_worker is not None or self._pick_worker is not None or self._gaze_busy()

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
        self.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
        self.stop_gaze_stabilizer()
        self.stop_object_pick()

    def _mobile_pick_object_world(self) -> Optional[tuple[float, float, float]]:
        obj = self._pick_latest_object_world() or self._pick_frozen_world()
        if obj is None:
            return None
        return tuple(float(v) for v in obj)

    def _mobile_pick_handoff_distance_m(
        self,
        host_state: Optional[HostState],
        object_world: Optional[tuple[float, float, float]],
    ) -> Optional[float]:
        if host_state is None or object_world is None:
            return None
        try:
            return host_horizontal_object_distance_m(host_state, object_world)
        except Exception:
            return None

    def _mobile_pick_handoff_ready(
        self,
        host_state: Optional[HostState],
        object_world: Optional[tuple[float, float, float]],
        *,
        handoff_distance_m: float,
    ) -> tuple[bool, Optional[float]]:
        dist = self._mobile_pick_handoff_distance_m(host_state, object_world)
        if dist is None:
            return False, None
        return bool(float(dist) <= float(handoff_distance_m)), float(dist)

    def _mobile_pick_timeout_handoff_ready(
        self,
        host_state: Optional[HostState],
        object_world: Optional[tuple[float, float, float]],
        *,
        handoff_distance_m: float,
        timeout_slack_m: float,
    ) -> tuple[bool, Optional[float]]:
        dist = self._mobile_pick_handoff_distance_m(host_state, object_world)
        if dist is None:
            return False, None
        soft_limit_m = float(handoff_distance_m) + max(float(timeout_slack_m), 0.0)
        return bool(float(dist) <= soft_limit_m), float(dist)

    def _mobile_pick_base_velocity_toward_object(
        self,
        host_state: Optional[HostState],
        object_world: Optional[tuple[float, float, float]],
        *,
        speed_mps: float,
    ) -> tuple[float, float, float]:
        speed = float(max(speed_mps, 0.0))
        if speed <= 1e-6:
            return 0.0, 0.0, 0.0
        if host_state is None or object_world is None:
            return speed, 0.0, 0.0
        base_pos = standoff_base_pos(host_state)
        if base_pos is None:
            return speed, 0.0, 0.0
        dx = float(object_world[0]) - float(base_pos[0])
        dy = float(object_world[1]) - float(base_pos[1])
        dist = float(math.hypot(dx, dy))
        if dist <= 1e-6:
            return 0.0, 0.0, 0.0
        yaw = 0.0
        if host_state.go2_base_rpy is not None:
            try:
                yaw = float(host_state.go2_base_rpy[2])
            except (TypeError, ValueError, IndexError):
                yaw = 0.0
        c = math.cos(yaw)
        s = math.sin(yaw)
        x_body = c * dx + s * dy
        y_body = -s * dx + c * dy
        scale = speed / max(dist, 1e-6)
        return float(x_body * scale), float(y_body * scale), 0.0

    def _mobile_pick_wait_for_object(
        self,
        *,
        timeout_s: float,
    ) -> tuple[Optional[HostState], Optional[tuple[float, float, float]]]:
        deadline = time.time() + float(max(timeout_s, 0.1))
        host_state: Optional[HostState] = None
        while time.time() < deadline:
            if self._pick_e2e_cancel.is_set() or self._pick_stop_event.is_set():
                return host_state, None
            host_state = self.client.refresh_state() if self.client is not None else None
            obs = self.current_visual_observation(host_state)
            obj = self._mobile_pick_object_world()
            if obs is not None and obj is not None:
                return host_state, obj
            time.sleep(0.05)
        return host_state, self._mobile_pick_object_world()

    def _mobile_pick_latch_handoff(
        self,
        *,
        host_state: Optional[HostState],
        object_world: tuple[float, float, float],
    ) -> None:
        obj = tuple(float(v) for v in object_world)
        self._reset_pick_last_seen_uv()
        self._reset_pick_uv_jacobian()
        self._reset_pick_search_state()
        self._reset_pick_drift_accounting()
        self._reset_pick_resolved_ready_state()
        self._reset_pick_equal_sag_result_state()
        self._reset_pick_look_state()
        self._reset_grasp_guided_state()
        self._pick_frozen_world_xyz = obj
        self._pick_initial_object_world_xyz = obj
        self._pick_centered_object_world_xyz = obj

        tip = self._pick_current_tip_world(host_state=host_state)
        direction = self._pick_ready_direction(
            object_world=obj,
            tip_world=tip,
            prefer_current_tip=True,
        )
        if direction is not None:
            self._pick_resolved_ready_dir_world = tuple(float(v) for v in direction)
        if tip is not None:
            tip_tuple = tuple(float(v) for v in tip)
            self._pick_resolved_ready_pose_world_xyz = tip_tuple
            self._pick_look_tip_world_xyz = tip_tuple

    def start_mobile_gaze_lji_pick_e2e(self) -> None:
        """Walk with gaze until handoff distance, then run LJI grasp."""
        if self.pick_e2e_running() or self._pick_busy() or self.state.ik_running or self._ik_worker is not None:
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
        self._pick_stop_event.clear()
        timeout_s = float(self._pick_e2e_phase_timeout_s)

        def _worker() -> None:
            success = False
            try:
                pk = self._pick_config_effective()
                handoff_m = float(max(pk.mobile_handoff_distance_m, pk.grasp_standoff_m))
                handoff_timeout_slack_m = float(max(pk.mobile_handoff_timeout_slack_m, 0.0))
                soft_handoff_m = handoff_m + handoff_timeout_slack_m
                approach_v = float(max(pk.mobile_approach_vx_mps, 0.0))
                approach_timeout_s = float(max(pk.mobile_approach_timeout_s, 0.1))
                settle_s = float(max(pk.mobile_stop_settle_s, 0.0))
                gaze_mode = str(pk.mobile_gaze_mode or "uv_ff").strip().lower()
                print(
                    "[MobilePick] start | gaze=%s handoff=%.0fmm soft=%.0fmm approach_v=%.2fm/s"
                    % (gaze_mode, handoff_m * 1000.0, soft_handoff_m * 1000.0, approach_v)
                )
                self.state.set_pick_status(
                    running=True,
                    failed=False,
                    phase=ObjectPickPhase.ACQUIRE.value,
                    msg="mobile pick: acquiring target",
                )

                perception_running = bool(self.state.perception_running)
                if not self._perception_run_local and self.client is not None:
                    host_preview = self.client.refresh_state()
                    self._sync_remote_perception_from_host(host_preview)
                    perception_running = bool(getattr(host_preview, "perception_running", False))
                if self._perception_run_local:
                    perception_running = (
                        self._perception_capture is not None
                        and self._perception_capture.is_running()
                    )
                if not perception_running:
                    self.start_perception_capture(config=self._perception_cfg)

                host_state, object_world = self._mobile_pick_wait_for_object(
                    timeout_s=min(approach_timeout_s, 5.0),
                )
                if object_world is None:
                    obs = self.current_visual_observation(host_state)
                    detail = "uv=%s world=%s" % (
                        "ok" if obs is not None else "missing",
                        "ok" if self._mobile_pick_object_world() is not None else "missing",
                    )
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg=f"mobile pick: no target observation | {detail}",
                    )
                    return

                self.start_gaze_stabilizer_walking(gaze_mode=gaze_mode)
                deadline = time.time() + approach_timeout_s
                last_dist: Optional[float] = None
                handoff_reason = "threshold"
                while time.time() < deadline:
                    if self._pick_e2e_cancel.is_set() or self._pick_stop_event.is_set():
                        self.state.set_pick_status(
                            running=False,
                            failed=False,
                            phase=ObjectPickPhase.IDLE.value,
                            msg="mobile pick stopped",
                        )
                        return
                    host_state = self.client.refresh_state() if self.client is not None else host_state
                    object_world = self._mobile_pick_object_world() or object_world
                    ready, dist = self._mobile_pick_handoff_ready(
                        host_state,
                        object_world,
                        handoff_distance_m=handoff_m,
                    )
                    if dist is not None:
                        last_dist = float(dist)
                    if ready:
                        break
                    vx, vy, wz = self._mobile_pick_base_velocity_toward_object(
                        host_state,
                        object_world,
                        speed_mps=approach_v,
                    )
                    self.send_go2_velocity(vx=vx, vy=vy, wz=wz)
                    dist_txt = "n/a" if last_dist is None else "%.0fmm" % (last_dist * 1000.0)
                    self.state.set_pick_status(
                        running=True,
                        failed=False,
                        phase=ObjectPickPhase.NAVIGATE.value,
                        msg="mobile gaze approach | dist=%s handoff=%.0fmm soft=%.0fmm"
                        % (dist_txt, handoff_m * 1000.0, soft_handoff_m * 1000.0),
                    )
                    time.sleep(0.10)
                else:
                    self.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
                    self.stop_gaze_stabilizer()
                    host_state = self.client.refresh_state() if self.client is not None else host_state
                    object_world = self._mobile_pick_object_world() or object_world
                    soft_ready, dist = self._mobile_pick_timeout_handoff_ready(
                        host_state,
                        object_world,
                        handoff_distance_m=handoff_m,
                        timeout_slack_m=handoff_timeout_slack_m,
                    )
                    if dist is not None:
                        last_dist = float(dist)
                    dist_txt = "n/a" if last_dist is None else "%.0fmm" % (last_dist * 1000.0)
                    if soft_ready:
                        handoff_reason = "timeout-soft"
                    else:
                        self.state.set_pick_status(
                            running=False,
                            failed=True,
                            phase=ObjectPickPhase.FAILED.value,
                            msg="mobile pick: handoff timeout | dist=%s handoff=%.0fmm soft=%.0fmm"
                            % (dist_txt, handoff_m * 1000.0, soft_handoff_m * 1000.0),
                        )
                        return

                    print(
                        "[MobilePick] timeout soft handoff | dist=%s <= %.0fmm"
                        % (dist_txt, soft_handoff_m * 1000.0)
                    )
                    self.state.set_pick_status(
                        running=True,
                        failed=False,
                        phase=ObjectPickPhase.HANDOFF.value,
                        msg="mobile soft handoff -> LJI | dist=%s handoff=%.0fmm soft=%.0fmm"
                        % (dist_txt, handoff_m * 1000.0, soft_handoff_m * 1000.0),
                    )

                self.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
                self.stop_gaze_stabilizer()
                if settle_s > 1e-6:
                    time.sleep(settle_s)
                host_state = self.client.refresh_state() if self.client is not None else host_state
                object_world = self._mobile_pick_object_world() or object_world
                self._mobile_pick_latch_handoff(
                    host_state=host_state,
                    object_world=object_world,
                )
                dist_txt = "n/a" if last_dist is None else "%.0fmm" % (last_dist * 1000.0)
                self.state.set_pick_status(
                    running=True,
                    failed=False,
                    phase=ObjectPickPhase.HANDOFF.value,
                    msg="mobile %s handoff -> LJI | dist=%s"
                    % ("soft" if handoff_reason == "timeout-soft" else "threshold", dist_txt),
                )
                print("[MobilePick] %s handoff | dist=%s -> LJI grasp" % (handoff_reason, dist_txt))

                self.start_grasp()
                if self.state.pick_failed:
                    return
                if not self._wait_pick_phase_done(timeout_s=timeout_s, label="mobile_lji_grasp"):
                    if not self.state.pick_failed:
                        self.state.set_pick_status(
                            running=False,
                            failed=True,
                            phase=ObjectPickPhase.FAILED.value,
                            msg="mobile pick: LJI grasp timeout",
                        )
                    return
                if str(self.state.pick_phase) != ObjectPickPhase.DONE.value:
                    return
                self.state.set_pick_status(
                    running=False,
                    failed=False,
                    phase=ObjectPickPhase.DONE.value,
                    msg="mobile pick done | gaze -> LJI grasp",
                )
                success = True
                print("[MobilePick] done | gaze -> LJI grasp")
            except Exception as exc:
                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg=f"mobile pick failed: {exc}",
                )
                print(f"[MobilePick] failed: {exc}")
            finally:
                self.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
                self.stop_gaze_stabilizer()
                cancelled = bool(self._pick_e2e_cancel.is_set() or self._pick_stop_event.is_set())
                if not success and not self.state.pick_failed and not cancelled:
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg="mobile pick failed",
                    )
                self._pick_e2e_worker = None

        self._pick_e2e_worker = threading.Thread(
            target=_worker,
            name="mobile-pick-e2e",
            daemon=True,
        )
        self._pick_e2e_worker.start()

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

    def _pick_apply_lost_follow_step(self, *, reason: str, allow_refresh: bool = True) -> bool:
        if self._pick_lost_follow_count >= int(self._pick_lost_follow_max_steps):
            return False
        cap = self._perception_capture
        if bool(allow_refresh) and cap is not None:
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

    def _q_array_for_motion_feedback(
        self,
        host_state: Optional[HostState] = None,
    ) -> np.ndarray:
        """Actual q for settle/measurement; sim-only publishes it as sim_q."""
        src = host_state if host_state is not None else self.current_host_state()
        if src is not None and src.sim_q is not None:
            return np.array(
                [
                    float(src.sim_q.linear_m),
                    float(src.sim_q.roll_rad),
                    float(src.sim_q.theta1_rad),
                    float(src.sim_q.theta2_rad),
                ],
                dtype=float,
            )
        return self._q_array_from_state(src)

    def _clamp_q(self, q: np.ndarray) -> np.ndarray:
        arr = np.asarray(q, dtype=float).reshape(4).copy()
        cfg = self._mapping_cfg
        linear_min_m, linear_max_m = linear_effective_q_bounds(cfg)
        arr[0] = float(np.clip(arr[0], linear_min_m, linear_max_m))
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
        host_state, _settled = self._wait_until_q_settled(q_cmd, timeout_s=float(timeout_s))
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
            u_linear=float(np.clip(display_u.u_linear, cfg.linear_u_min, linear_motor_u_limit(cfg))),
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
            u_linear=float(min(max(display_u.u_linear, cfg.linear_u_min), linear_motor_u_limit(cfg))),
            u_roll=float(min(max(display_u.u_roll, cfg.roll_u_min), cfg.roll_u_max)),
            u_s1=float(min(max(display_u.u_s1, cfg.seg_u_min), cfg.seg_u_max)),
            u_s2=float(min(max(display_u.u_s2, cfg.seg_u_min), cfg.seg_u_max)),
        )

    def control_mapping(self) -> SimMappingConfig:
        if self.client is not None:
            cfg = getattr(self.client, "cfg", None)
            if isinstance(cfg, SimMappingConfig):
                return cfg
        return self._mapping_cfg

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

    def send_go2_velocity(self, *, vx: float, vy: float, wz: float) -> None:
        if self.client is not None:
            self.client.send_go2_velocity(vx=float(vx), vy=float(vy), wz=float(wz), source="target")

    def send_go2_sport_pose(self, *, pose: str) -> None:
        if self.client is not None:
            self.client.send_go2_sport_pose(pose=str(pose), source="target")

    def send_go2_obstacles_avoid(self, *, enabled: bool) -> None:
        if self.client is not None:
            self.client.send_go2_obstacles_avoid(enabled=bool(enabled), source="target")

    def send_sim_target_xyz(self, x: float, y: float, z: float) -> None:
        self.state.set_mock_object_world_xyz(float(x), float(y), float(z))
        if self.client is not None and hasattr(self.client, "send_sim_target_xyz"):
            self.client.send_sim_target_xyz(
                xyz=(float(x), float(y), float(z)),
                source="target",
            )

    def _start_position_solve(self, target: np.ndarray) -> None:
        if self.state.ik_running or self._visual_busy():
            return
        if self._pick_busy():
            return
        self.refresh_ik_context()
        ctx = self._ik_context_for_host(
            self.current_host_state(),
            sag_model=dict(self.state.raw_sag_model) if isinstance(self.state.raw_sag_model, dict) else {},
        )
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

    def _retire_perception_capture(self, cap: PerceptionCapture, *, stop_recording: bool = True) -> None:
        if self._perception_capture is not cap:
            return
        if bool(stop_recording):
            self._stop_side_camera_recording()
        self._perception_capture = None
        self._perception_capture_epoch += 1
        self._perception_rate_last_t = 0.0
        self._perception_rate_last_frame_idx = -1
        self._perception_hz = 0.0

    def _on_perception_snapshot(
        self,
        snap: PerceptionSnapshot,
        *,
        capture_epoch: int = 0,
    ) -> None:
        if int(capture_epoch) > 0 and int(capture_epoch) != int(self._perception_capture_epoch):
            return
        frame_idx = int(snap.frame_idx)
        now = time.monotonic()
        if not bool(snap.running) or bool(snap.failed):
            self._perception_hz = 0.0
            self._perception_rate_last_t = 0.0
            self._perception_rate_last_frame_idx = -1
        elif self._perception_rate_last_frame_idx < 0 or frame_idx <= self._perception_rate_last_frame_idx:
            self._perception_hz = 0.0
            self._perception_rate_last_t = now
            self._perception_rate_last_frame_idx = frame_idx
        else:
            dt = max(1e-6, now - float(self._perception_rate_last_t))
            frames = max(1, frame_idx - int(self._perception_rate_last_frame_idx))
            inst_hz = float(frames) / dt
            prev = float(self._perception_hz)
            self._perception_hz = inst_hz if prev <= 0.0 else (0.75 * prev + 0.25 * inst_hz)
            self._perception_rate_last_t = now
            self._perception_rate_last_frame_idx = frame_idx
        world_xyz = snap.p_world
        if bool(self.state.pick_running):
            world_xyz = self._pick_frozen_world()
        self.state.set_perception_status(
            running=bool(snap.running),
            failed=bool(snap.failed),
            msg=str(snap.status_msg),
            frame_idx=frame_idx,
            label=str(snap.label),
            confidence=float(snap.confidence),
            camera_xyz=snap.p_camera,
            world_xyz=world_xyz,
            tracker_phase=str(snap.tracker_phase),
            track_ok_frames=int(snap.track_ok_frames),
            image_scale=float(snap.image_scale),
            bbox_wh=tuple(snap.bbox_wh),
            tracker_backend=str(snap.tracker_backend),
            center_uv=snap.center_uv,
            perception_hz=float(self._perception_hz),
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
        return replace(
            pk,
            center_tol=aim_tol,
            center_roll_max=min(float(pk.center_roll_max), 3.0),
            center_seg_max=min(float(pk.center_seg_max), 3.0),
        )

    def _gaze_center_pick_config(self) -> PickConfig:
        """Gaze UV centering: dedicated gains/deadband from [gaze_stabilizer]."""
        pk = self._pick_config_effective()
        g = self._gaze_cfg
        gain = float(max(g.uv_gain, 0.05))
        return replace(
            pk,
            center_tol=float(g.center_tol),
            center_u_gain=float(g.center_u_gain) * gain,
            center_v_gain=float(g.center_v_gain) * gain,
            center_roll_max=float(g.center_roll_max),
            center_seg_max=float(g.center_seg_max),
        )

    def reset_gaze_derivative_state(self) -> None:
        self._gaze_prev_uv_err = None
        self._gaze_dv_err_rate_filt = 0.0
        self._gaze_last_cmd_wall_s = 0.0
        self._gaze_last_sent_du_mag = 0.0

    def _gaze_derivative_seg_du(
        self,
        u_err: float,
        v_err: float,
        *,
        dt_s: float,
    ) -> tuple[float, float]:
        """PD derivative term on normalized UV error (s1/s2 only, matches P coupling)."""
        g = self._gaze_cfg
        kd_v = float(g.center_v_kd)
        if kd_v <= 0.0:
            self._gaze_prev_uv_err = (float(u_err), float(v_err))
            return 0.0, 0.0
        if self._gaze_prev_uv_err is None:
            self._gaze_prev_uv_err = (float(u_err), float(v_err))
            return 0.0, 0.0
        dt = max(float(dt_s), 1e-3)
        _prev_u, prev_v = self._gaze_prev_uv_err
        raw_dv = (float(v_err) - float(prev_v)) / dt
        alpha = float(np.clip(float(g.d_filter_alpha), 0.0, 1.0))
        self._gaze_dv_err_rate_filt = alpha * raw_dv + (1.0 - alpha) * float(self._gaze_dv_err_rate_filt)
        self._gaze_prev_uv_err = (float(u_err), float(v_err))
        cap = float(g.center_d_seg_max) * float(max(g.step_scale, 0.05))
        s1_d = float(np.clip(kd_v * self._gaze_dv_err_rate_filt, -cap, cap))
        s2_d = float(np.clip(-0.5 * kd_v * self._gaze_dv_err_rate_filt, -cap, cap))
        return s1_d, s2_d

    def _pick_config_for_grasp(self) -> PickConfig:
        """UV tolerance for guided grasp (defaults to aim_center_tol when unset)."""
        pk = self._pick_config_effective()
        raw_tol = float(pk.grasp_uv_center_tol)
        if raw_tol <= 0.0:
            tol = float(max(0.01, float(pk.aim_center_tol)))
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
        self._grasp_lji_last_good_q = None
        self._grasp_lji_pending_sample = None
        self._grasp_lji_last_dq_cmd = None
        self._grasp_lji_reacquire_anchor_dq = None
        self._grasp_lji_reacquire_steps = 0
        self._grasp_lji_reacquire_aim_tried = False
        self._grasp_lji_reacquire_prev_remain = None
        self._grasp_lji_v_err_hist = []
        self._grasp_lji_last_transition = "-"
        self._grasp_lji_sat_streak = 0
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
        )
        self._grasp_approach_mode = GraspApproachMode.LOCAL_IMG_JACOBIAN
        self._grasp_depth_history.clear()
        self._grasp_lji_object_lost_count = 0
        self._grasp_lji_pending_sample = None
        self._grasp_lji_last_dq_cmd = None
        self._grasp_lji_reacquire_anchor_dq = None
        self._grasp_lji_reacquire_steps = 0
        self._grasp_lji_reacquire_aim_tried = False
        self._grasp_lji_reacquire_prev_remain = None
        self._grasp_lji_v_err_hist = []
        self._grasp_lji_last_transition = "-"

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
        max_ang = float(pk.lij_max_dq_angle) * scale
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
            gain_z=float(pk.lij_gain_z) * scale,
        )
        return dq, dq_raw, j, int(rank), float(cond), bool(avail), "local_img_jacobian"

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
        est = self._grasp_lji_estimator_3d
        if est is not None:
            est.clear()

    def _grasp_lji_end_reacquire(self) -> None:
        self._grasp_lji_reacquire_anchor_dq = None
        self._grasp_lji_reacquire_steps = 0
        self._grasp_lji_reacquire_aim_tried = False
        self._grasp_lji_reacquire_prev_remain = None
        self._grasp_lji_v_err_hist = []

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
            return self.client.refresh_state()
        deadline = time.time() + float(max(timeout_s, 0.04))
        poll_s = 0.015 if not bool(self._use_hardware) else 0.03
        frac = float(np.clip(min_frac, 0.10, 0.90))
        last_state: Optional[HostState] = None
        while time.time() < deadline:
            time.sleep(poll_s)
            last_state = self.client.refresh_state()
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
    ) -> tuple[np.ndarray, Optional[HostState]]:
        q0 = self._q_array_from_state(host_state)
        dq_arr = np.asarray(dq, dtype=float).reshape(4)
        q_cmd = self._clamp_q(q0 + dq_arr)
        self.state.set_q(
            float(q_cmd[0]),
            float(q_cmd[1]),
            float(q_cmd[2]),
            float(q_cmd[3]),
        )
        motion_source = "lji" if bool(wait_settle) else "lji_step"
        if bool(wait_settle):
            host_after = self._send_state_q_and_wait(
                timeout_s=float(timeout_s),
                source=motion_source,
                force=True,
                sag_model_override=dict(sag_model),
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
        if bool(pipelined):
            self._grasp_lji_pending_sample = None
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
            est.push(delta_q, delta_s)
        self._grasp_lji_pending_sample = None
        return reason

    def _grasp_lji_log_control_step(
        self,
        *,
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
        ik_status: str,
        sample_reason: str,
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

    def _start_grasp_guided_approach(self, *, internal: bool = False) -> bool:
        """Start online UV→sag→axial-IK loop toward pre-contact (no offline plan)."""
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
        dir_tuple = self._grasp_aim_latched_direction(object_world=object_world)
        if dir_tuple is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="cannot infer grasp approach direction",
            )
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
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="grasp | tip FK unavailable",
            )
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
            target=_worker,
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
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="grasp | start position missing",
            )
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
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg="grasp guided | tip FK unavailable",
                    )
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
                        self.state.set_pick_status(
                            running=False,
                            failed=True,
                            phase=ObjectPickPhase.FAILED.value,
                            msg="grasp guided | filtered tracking unavailable",
                        )
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
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg="grasp | %s axial IK failed" % str(wp_label),
                    )
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
                        self.state.set_pick_status(
                            running=False,
                            failed=True,
                            phase=ObjectPickPhase.FAILED.value,
                            msg="grasp | %s motion settle timeout" % str(wp_label),
                        )
                        return

                tip = self._pick_current_tip_world(host_state=host_state)
                if tip is None:
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg="grasp guided | tip FK unavailable after settle",
                    )
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
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg="grasp | handoff filtered tracking unavailable",
                    )
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
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg="grasp | blind extend missing handoff look direction",
                    )
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
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg="grasp | blind extend failed",
                    )
                    return

            if q_cmd is None:
                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg="grasp | no commanded q",
                )
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
                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg="grasp failed",
                )
            self._ik_worker = None

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
        waypoint_settle_timeout_s = float(max(pk.grasp_waypoint_settle_timeout_s, 0.0))
        motion_apply_timeout_s = self._grasp_motion_apply_timeout_s(pk)
        standoff_m = float(max(pk.grasp_standoff_m, 0.0))
        close_tol_m = float(max(pk.grasp_close_tol_m, float(self._ik_cfg.tol), 0.003))
        lji_settle_dwell_s = float(max(pk.lij_settle_dwell_s, 0.0))
        lji_motion_settle_timeout_s = float(max(pk.lij_settle_timeout_s, 0.0))
        lji_settle_angle_tol = max(0.006, float(pk.lij_max_dq_angle) * 0.55)
        lji_settle_linear_tol = max(5e-4, float(pk.lij_max_dq_linear) * 0.45)
        lji_pipelined = bool(pk.lij_pipelined_motion)
        lji_step_period_s = float(max(pk.lij_step_period_s, 0.0))
        success = False
        traj_start = self._grasp_traj_start
        look_anchor = self._grasp_look_anchor
        servo = self._grasp_lji_servo_3d
        if traj_start is None or servo is None:
            self.state.set_pick_status(
                running=False,
                failed=True,
                phase=ObjectPickPhase.FAILED.value,
                msg="grasp lji | init missing",
            )
            return
        try:
            if self._perception_capture is None or not self._perception_capture.is_running():
                self._maybe_start_local_perception()

            host_state = self.client.refresh_state() if self.client is not None else None
            q_cmd: Optional[np.ndarray] = None
            sag_model = self._grasp_lji_sag_model()
            mode = GraspApproachMode.LOCAL_IMG_JACOBIAN
            self._grasp_approach_mode = mode
            prev_mode = mode
            print(
                "[Grasp] LJI3D start | close_tol=%.1fmm blind_at_remain=%.0fmm "
                "gain_z=%.2f z_bend=%.2f"
                % (
                    close_tol_m * 1000.0,
                    float(pk.blind_micro_start_m) * 1000.0,
                    float(pk.lij_gain_z),
                    float(pk.lij_z_bend_gain),
                )
            )

            wp_idx = 0
            while wp_idx < max_waypoints:
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
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg="grasp lji | tip FK unavailable",
                    )
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
                        self.state.set_pick_status(
                            running=False,
                            failed=True,
                            phase=ObjectPickPhase.FAILED.value,
                            msg="grasp lji | filtered tracking unavailable",
                        )
                        return
                    dir_tuple_live = dir_live
                dir_u = self._unit_vec3(dir_tuple_live)
                nominal_live = self._pick_grasp_trajectory_end_position(
                    live_object,
                    dir_u,
                    standoff_m=standoff_m,
                )
                remain = self._grasp_axial_distance(tip, nominal_live, dir_u)
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
                    print(
                        "[Grasp] LJI | precontact | remain=%.1fmm <= close_tol %.1fmm"
                        % (float(remain) * 1000.0, close_tol_m * 1000.0)
                    )
                    break

                obs = self.current_visual_observation(host_state)
                object_lost = obs is None
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
                        transition = "object_lost|reacquire"
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
                        dq_back = self._grasp_lji_compute_axial_retract_dq(
                            pk=pk,
                            approach_dir=dir_u,
                            object_world=tuple(float(v) for v in live_object),
                            sag_model=dict(sag_model),
                            host_state=host_state,
                            q_before=q_before,
                        )
                        if dq_back is None:
                            dq_back = self._grasp_lji_retract_dq_to_last_good_q(
                                q_before=q_before,
                                pk=pk,
                            )
                        if dq_back is not None:
                            dq_cmd_arr = np.asarray(dq_back, dtype=float).reshape(4)
                            q_cmd, host_state = self._grasp_apply_q_delta(
                                dq_cmd_arr,
                                host_state=host_state,
                                sag_model=dict(sag_model),
                                timeout_s=motion_apply_timeout_s,
                                wait_settle=not lji_pipelined,
                                step_period_s=lji_step_period_s,
                            )
                            moved = True
                        if self._pick_apply_lost_follow_step(
                            reason="grasp_lji_fov",
                            allow_refresh=False,
                        ):
                            host_state = (
                                self.client.refresh_state()
                                if self.client is not None
                                else host_state
                            )
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
                    obs_after = self.current_visual_observation(host_state)
                    if obs_after is not None and not self._grasp_lji_visual_tracking_lost(
                        self._grasp_lji_build_features_3d(
                            obs_after, remain_m=float(remain_after)
                        ),
                        pk=pk,
                    ):
                        self._grasp_lji_object_lost_count = 0
                        self._grasp_lji_end_reacquire()
                        self._record_pick_last_seen_uv(obs_after)
                    tip_after = self._pick_current_tip_world(host_state=host_state)
                    if tip_after is not None:
                        remain_after = float(
                            self._grasp_axial_distance(tip_after, nominal_live, dir_u)
                        )
                    self._grasp_lji_reacquire_prev_remain = float(remain_after)
                    s_lji = self._grasp_lji_build_features_3d(
                        obs_after, remain_m=float(remain_after)
                    )
                    if moved and q_cmd is not None:
                        dq_meas = np.asarray(q_cmd, dtype=float) - np.asarray(
                            q_before, dtype=float
                        )
                    self._grasp_lji_log_control_step(
                        mode=mode,
                        s_lji=s_lji,
                        depth_valid=depth_valid,
                        depth_valid_ratio=depth_valid_ratio,
                        j_rank=0,
                        j_cond=float("inf"),
                        j_available=False,
                        dq_cmd=dq_cmd_arr,
                        dq_meas=dq_meas,
                        q_cmd=q_cmd if q_cmd is not None else q_before,
                        controller=controller_tag,
                        transition=transition,
                        object_lost=int(self._grasp_lji_object_lost_count),
                        remain_m=float(remain_after),
                        close_tol_m=close_tol_m,
                        ik_status="-",
                        sample_reason="n/a",
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
                        q_before = self._q_array_from_state(host_state)
                        self._grasp_lji_pending_sample = {
                            "q_before": q_before.copy(),
                            "s_before": np.zeros(3, dtype=float),
                            "dq_cmd": probe.copy(),
                        }
                        if float(pk.lij_dq_smooth_alpha) > 1e-6:
                            probe = self._grasp_lji_smooth_dq(probe, pk=pk)
                        q_cmd, host_state = self._grasp_apply_q_delta(
                            probe,
                            host_state=host_state,
                            sag_model=dict(sag_model),
                            timeout_s=motion_apply_timeout_s,
                            wait_settle=not lji_pipelined,
                            step_period_s=lji_step_period_s,
                        )
                        if float(pk.lij_dq_smooth_alpha) > 1e-6:
                            self._grasp_lji_last_dq_cmd = probe.copy()
                    else:
                        continue
                else:
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
                        q=self._q_array_from_state(host_state),
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
                    q_before = self._q_array_from_state(host_state)
                    dq_cmd_arr = self._grasp_lji_guard_dq_at_limits(
                        q_before,
                        dq_cmd_arr,
                        pk=pk,
                    )
                    if float(pk.lij_dq_smooth_alpha) > 1e-6:
                        dq_cmd_arr = self._grasp_lji_smooth_dq(dq_cmd_arr, pk=pk)
                    self._grasp_lji_pending_sample = {
                        "q_before": q_before.copy(),
                        "s_before": s_lji.copy(),
                        "dq_cmd": dq_cmd_arr.copy(),
                    }
                    q_cmd, host_state = self._grasp_apply_q_delta(
                        dq_cmd_arr,
                        host_state=host_state,
                        sag_model=dict(sag_model),
                        timeout_s=motion_apply_timeout_s,
                        wait_settle=not lji_pipelined,
                        step_period_s=lji_step_period_s,
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

                obs_after = self.current_visual_observation(host_state)
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
                    if stall_msg is not None:
                        print("[Grasp] %s" % stall_msg)
                        self.state.set_pick_status(
                            running=False,
                            failed=True,
                            phase=ObjectPickPhase.FAILED.value,
                            msg=stall_msg,
                        )
                        return

                self._grasp_lji_log_control_step(
                    mode=mode,
                    s_lji=s_lji,
                    depth_valid=depth_valid,
                    depth_valid_ratio=depth_valid_ratio,
                    j_rank=int(j_rank),
                    j_cond=float(j_cond),
                    j_available=bool(j_available),
                    dq_cmd=dq_cmd_arr,
                    dq_meas=dq_meas,
                    q_cmd=q_cmd if q_cmd is not None else self._q_array_from_state(host_state),
                    controller=controller_tag,
                    transition=transition,
                    object_lost=int(self._grasp_lji_object_lost_count),
                    remain_m=float(remain_after),
                    close_tol_m=close_tol_m,
                    ik_status=ik_status,
                    sample_reason=str(sample_reason.value),
                )
                self.state.set_pick_status(
                    running=True,
                    failed=False,
                    phase=ObjectPickPhase.GRASP_APPROACH.value,
                    msg="grasp %s | remain=%.0fmm mode=%s"
                    % (str(wp_label), float(remain) * 1000.0, str(mode.value)),
                )

            if q_cmd is None:
                host_state = self.client.refresh_state() if self.client is not None else None
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
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg="grasp lji | blind finish failed",
                    )
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
                    self.state.set_pick_status(
                        running=False,
                        failed=True,
                        phase=ObjectPickPhase.FAILED.value,
                        msg="grasp lji | final tracking unavailable",
                    )
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
            self._grasp_uv_only_mode = False
            cancelled = bool(self._pick_stop_event.is_set() or self._pick_e2e_cancel.is_set())
            if (
                not cancelled
                and self._perception_capture is not None
                and self._perception_capture.is_running()
            ):
                self.stop_perception_capture(stop_recording=not bool(self.state.perception_recording))
            if not success and not self.state.pick_failed and not cancelled:
                self.state.set_pick_status(
                    running=False,
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg="grasp lji failed",
                )
            self._ik_worker = None

    def _pick_reach_model(
        self,
        sag_model: Optional[dict[str, Any]] = None,
        host_state: Optional[HostState] = None,
    ):
        from engine.robot.arm.iklib.kinematics import _ReachModel

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
        from engine.robot.arm.iklib.kinematics import _forward_link_tf

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
        """Advance grasp point ``distance_m`` along EE local -Z via ``engine.robot.arm.ik.solve_then_align``."""
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
        camera_world_origin: Optional[tuple[float, float, float]] = None,
        camera_world_look: Optional[tuple[float, float, float]] = None,
        camera_world_right: Optional[tuple[float, float, float]] = None,
    ) -> Optional[tuple[float, float, float]]:
        if self.client is None:
            return None
        freeze_world = bool(self.state.pick_running)
        if bool(self._grasp_uv_only_mode):
            publish_depth = False
        else:
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
            camera_world_origin=camera_world_origin,
            camera_world_look=camera_world_look,
            camera_world_right=camera_world_right,
        )
        if freeze_world:
            frozen = self._pick_frozen_world()
            return frozen if frozen is not None else p_world
        if p_world is not None:
            self._pick_frozen_world_xyz = tuple(p_world)
        return p_world

    def _remote_preview_endpoint(self) -> str:
        endpoint = str(getattr(self._perception_cfg, "preview_endpoint", "")).strip()
        if endpoint:
            return endpoint
        host = self.current_host_state()
        if host is not None:
            endpoint = str(getattr(host, "perception_preview_endpoint", "")).strip()
        return endpoint

    def _start_remote_preview(self) -> None:
        if not bool(getattr(self._perception_cfg, "show_preview", True)):
            print("[perception] remote preview disabled by config")
            return
        if self._remote_preview_thread is not None and self._remote_preview_thread.is_alive():
            print("[perception] remote preview already running")
            return
        endpoint = self._remote_preview_endpoint()
        if not endpoint:
            self.state.set_perception_status(
                running=bool(self.state.perception_running),
                failed=True,
                msg="remote preview endpoint missing",
            )
            return
        print(f"[perception] remote preview connecting: {endpoint}")
        self._remote_preview_stop.clear()

        def _worker() -> None:
            try:
                from engine.vision.perception.preview import close_preview, show_preview
                from engine.vision.perception.preview_stream import PreviewFrameSubscriber
            except Exception as exc:
                self.state.set_perception_status(
                    running=bool(self.state.perception_running),
                    failed=True,
                    msg=f"remote preview import failed: {exc}",
                )
                return
            sub = PreviewFrameSubscriber(endpoint)
            got_first = False
            last_wait_log_s = 0.0
            try:
                while not self._remote_preview_stop.is_set():
                    frame = sub.recv_latest(timeout_ms=250)
                    if frame is None:
                        now = time.time()
                        if now - last_wait_log_s >= 3.0:
                            print(f"[perception] remote preview waiting for frames: {endpoint}")
                            last_wait_log_s = now
                        continue
                    if not got_first:
                        got_first = True
                        print(
                            "[perception] remote preview first frame: %dx%d"
                            % (int(frame.image_bgr.shape[1]), int(frame.image_bgr.shape[0]))
                        )
                    key = show_preview("elesim_remote_perception", frame.image_bgr)
                    if key in (ord("q"), 27):
                        break
            finally:
                try:
                    sub.close()
                except Exception:
                    pass
                try:
                    close_preview("elesim_remote_perception")
                except Exception:
                    pass

        self._remote_preview_thread = threading.Thread(
            target=_worker,
            name="remote-preview",
            daemon=True,
        )
        self._remote_preview_thread.start()

    def _stop_remote_preview(self) -> None:
        self._remote_preview_stop.set()
        thread = self._remote_preview_thread
        if thread is not None:
            thread.join(timeout=1.0)
        self._remote_preview_thread = None

    def start_perception_capture(self, *, config: Optional[PerceptionConfig] = None) -> None:
        if config is not None:
            self.update_perception_config(config)
        if not self._perception_run_local:
            if self.client is not None and hasattr(self.client, "send_perception_start"):
                self.client.send_perception_start(config=self._perception_cfg)
                self._start_remote_preview()
            self.state.set_perception_status(
                running=True,
                failed=False,
                msg="remote: starting Jetson perception",
            )
            return
        old = self._perception_capture
        if old is not None:
            if old.is_running():
                if not old.stop(timeout_s=10.0):
                    self._retire_perception_capture(old)
                    self.state.set_perception_status(
                        running=False,
                        failed=True,
                        msg="prior capture did not stop; retrying start",
                    )
                else:
                    self._retire_perception_capture(old)
            else:
                self._retire_perception_capture(old)
        cfg = config or self._perception_cfg
        self._perception_cfg = cfg
        self.state.visual_target_label = str(cfg.target_label).strip()
        epoch = int(self._perception_capture_epoch) + 1
        self._perception_capture_epoch = epoch
        cap = PerceptionCapture(
            cfg,
            publish_fn=self._publish_perception_to_host,
            on_snapshot=lambda snap, e=epoch: self._on_perception_snapshot(
                snap,
                capture_epoch=e,
            ),
            target_uv_fn=lambda: (
                float(self.state.visual_target_uv_u),
                float(self.state.visual_target_uv_v),
            ),
            mock_world_xyz_fn=self._mock_world_xyz_from_state,
        )
        self._perception_capture = cap
        self.state.set_perception_status(running=True, failed=False, msg="starting")
        cap.start()

    def stop_perception_capture(self, *, stop_recording: bool = True) -> None:
        if not self._perception_run_local:
            self._stop_remote_preview()
            if bool(stop_recording):
                self._stop_side_camera_recording()
            if self.client is not None and hasattr(self.client, "send_perception_stop"):
                self.client.send_perception_stop()
            if bool(stop_recording):
                self.state.set_perception_recording(False)
            self.state.set_perception_status(
                running=False,
                failed=False,
                msg="remote: stopping Jetson perception",
            )
            return
        cap = self._perception_capture
        if cap is None:
            if bool(stop_recording):
                self._stop_side_camera_recording()
                self.state.set_perception_recording(False)
            self.state.set_perception_status(running=False, failed=False, msg="stopped")
            return
        stopped = cap.stop(stop_recording=bool(stop_recording))
        if not stopped:
            if bool(stop_recording):
                self._retire_perception_capture(cap, stop_recording=True)
                self.state.set_perception_recording(False)
            self.state.set_perception_status(running=False, failed=True, msg="stop pending")
            return
        if bool(stop_recording):
            self._retire_perception_capture(cap, stop_recording=True)
            self.state.set_perception_recording(False)
        self.state.set_perception_status(
            running=False,
            failed=False,
            msg="stopped" if bool(stop_recording) else "stopped (recording kept)",
        )

    def refresh_perception_capture(self) -> None:
        if not self._perception_run_local:
            if self.client is not None and hasattr(self.client, "send_perception_refresh"):
                self.client.send_perception_refresh()
            self.state.set_perception_status(
                running=bool(self.state.perception_running),
                failed=False,
                msg="remote: refresh requested",
            )
            return
        cap = self._perception_capture
        if cap is None or not cap.is_running():
            self.state.set_perception_status(running=False, failed=True, msg="perception is not running")
            return
        if cap.request_refresh():
            self.state.set_perception_status(running=True, failed=False, msg="refresh requested (YOLO)")
        else:
            self.state.set_perception_status(running=False, failed=True, msg="refresh rejected")

    def _side_camera_config(self) -> Optional[SimConfig]:
        cfg_path = self._config_path or str(Path(__file__).resolve().parents[3] / "config.ini")
        try:
            cfg = load_app_config_from_ini(str(cfg_path)).sim_config
        except Exception as exc:
            print(f"[perception] side camera config load failed: {exc}")
            return None
        endpoint = str(getattr(cfg, "sim_side_camera_port", "")).strip()
        if not bool(getattr(cfg, "sim_side_camera_enable", False)) or not endpoint:
            return None
        return cfg

    @staticmethod
    def _side_record_path_for(record_path: str | Path) -> Path:
        p = Path(record_path)
        stem = p.stem
        if stem.endswith("_record"):
            stem = stem[: -len("_record")]
        return p.with_name(f"{stem}_side.mp4")

    @staticmethod
    def _side_snapshot_stem_for(capture_path: str | Path) -> str:
        stem = Path(capture_path).stem
        for suffix in ("_depth_vis", "_color", "_overlay", "_depth", "_meta"):
            if stem.endswith(suffix):
                stem = stem[: -len(suffix)]
                break
        return f"{stem}_side"

    def _start_side_camera_recording(self, record_path: str | Path) -> Optional[Path]:
        if self._side_camera_recorder is not None:
            self._stop_side_camera_recording()
        cfg = self._side_camera_config()
        if cfg is None:
            return None
        try:
            from engine.vision.sim_camera.recording import SimCameraVideoRecorder
        except Exception as exc:
            print(f"[perception] side recorder import failed: {exc}")
            return None
        out_path = self._side_record_path_for(record_path)
        rec = SimCameraVideoRecorder(
            str(cfg.sim_side_camera_port),
            out_path=out_path,
            fps=float(getattr(cfg, "sim_side_camera_record_fps", 30.0)),
            use_jpeg=bool(cfg.sim_side_camera_jpeg),
        )
        if not rec.start():
            print(f"[perception] side recording skipped: {rec.last_error}")
            return None
        self._side_camera_recorder = rec
        self._side_camera_record_path = out_path
        print(f"[perception] side recording started: {out_path.resolve()}")
        return out_path

    def _stop_side_camera_recording(self) -> Optional[tuple[bool, str, int, int, str]]:
        rec = self._side_camera_recorder
        if rec is None:
            return None
        self._side_camera_recorder = None
        self._side_camera_record_path = None
        ok, path_s, frame_count, unique_count, err = rec.stop()
        if ok:
            print(
                "[perception] side recording saved (%df/%du): %s"
                % (int(frame_count), int(unique_count), path_s)
            )
        else:
            print(f"[perception] side recording stop failed: {err or path_s}")
        return bool(ok), str(path_s), int(frame_count), int(unique_count), str(err or "")

    def _capture_side_camera_snapshot(self, paired_path: str | Path) -> Optional[Path]:
        cfg = self._side_camera_config()
        if cfg is None:
            return None
        try:
            from engine.vision.sim_camera.recording import (
                capture_sim_camera_snapshot,
                save_sim_camera_snapshot,
            )
        except Exception as exc:
            print(f"[perception] side snapshot import failed: {exc}")
            return None
        try:
            frame = capture_sim_camera_snapshot(
                str(cfg.sim_side_camera_port),
                use_jpeg=bool(cfg.sim_side_camera_jpeg),
                timeout_s=1.5,
            )
            if frame is None:
                print("[perception] side snapshot skipped: no side camera frame")
                return None
            paired = Path(paired_path)
            stem = self._side_snapshot_stem_for(paired)
            side_path = save_sim_camera_snapshot(
                frame=frame,
                out_dir=paired.parent,
                stem=stem,
                meta={
                    "paired_capture": str(paired.resolve()),
                    "endpoint": str(cfg.sim_side_camera_port),
                },
            )
            print(f"[perception] side snapshot saved {side_path.resolve()}")
            return side_path
        except Exception as exc:
            print(f"[perception] side snapshot failed: {exc}")
            return None

    def capture_perception_frame(self) -> bool:
        """Save latest perception frame (or one-shot sim grab) under logs/perception_capture/."""
        if not self._perception_run_local:
            if self.client is None or not hasattr(self.client, "send_perception_capture"):
                self.state.set_perception_status(
                    running=bool(self.state.perception_running),
                    failed=True,
                    msg="remote: snapshot unsupported by host client",
                )
                return False
            self.client.send_perception_capture(
                include_overlay=bool(self.state.perception_record_with_overlay)
            )
            self.state.set_perception_status(
                running=bool(self.state.perception_running),
                failed=False,
                msg="remote: snapshot requested on Jetson",
            )
            return True
        out_dir = default_perception_capture_dir()
        cap = self._perception_capture
        path: Optional[Path] = None
        if cap is not None and cap.has_cached_frame():
            path = cap.save_cached_frames(
                out_dir,
                extra_meta={"mode": str(self._perception_cfg.mode)},
            )
        if path is None:
            path = self._capture_sim_perception_frame_once(out_dir)
        if path is None:
            msg = "capture failed: no frame (start perception or check sim camera)"
            self.state.set_perception_status(
                running=bool(self.state.perception_running),
                failed=True,
                msg=msg,
            )
            print(f"[perception] {msg}")
            return False
        path_s = str(path.resolve())
        side_path = self._capture_side_camera_snapshot(path)
        side_s = "" if side_path is None else str(side_path.resolve())
        self.state.set_perception_last_capture(path_s)
        msg = f"saved {path_s}" if not side_s else f"saved {path_s} + side {side_s}"
        self.state.set_perception_status(
            running=bool(self.state.perception_running),
            failed=False,
            msg=msg,
        )
        print(f"[perception] {msg}")
        return True

    def start_perception_recording(self) -> bool:
        """Start recording local perception frames to MP4 under logs/perception_capture/."""
        if not self._perception_run_local:
            if self.client is None or not hasattr(self.client, "send_perception_record_start"):
                self.state.set_perception_status(
                    running=bool(self.state.perception_running),
                    failed=True,
                    msg="remote: recording unsupported by host client",
                )
                return False
            use_overlay = bool(self.state.perception_record_with_overlay)
            self.client.send_perception_record_start(
                include_overlay=use_overlay,
                fps=float(self._perception_cfg.publish_hz),
            )
            self.state.set_perception_recording(True, "Jetson host")
            overlay_tag = "overlay" if use_overlay else "raw"
            self.state.set_perception_status(
                running=bool(self.state.perception_running),
                failed=False,
                msg=f"remote: recording start requested on Jetson ({overlay_tag})",
            )
            return True
        cap = self._perception_capture
        if cap is None or not cap.is_running():
            self.state.set_perception_status(running=False, failed=True, msg="perception is not running")
            return False
        use_overlay = bool(self.state.perception_record_with_overlay)
        ok, path_s = cap.start_recording(
            default_perception_capture_dir(),
            fps=float(self._perception_cfg.publish_hz),
            include_overlay=use_overlay,
        )
        if not ok:
            self.state.set_perception_status(running=True, failed=True, msg="recording already active")
            return False
        self.state.set_perception_recording(True, path_s)
        overlay_tag = "overlay" if use_overlay else "raw"
        side_path = self._start_side_camera_recording(path_s)
        side_msg = "" if side_path is None else f" + side {side_path.resolve()}"
        self.state.set_perception_status(
            running=True,
            failed=False,
            msg=f"recording started ({overlay_tag}): {path_s}{side_msg}",
        )
        print(f"[perception] recording started ({overlay_tag}): {path_s}{side_msg}")
        return True

    def stop_perception_recording(self) -> bool:
        if not self._perception_run_local:
            if self.client is None or not hasattr(self.client, "send_perception_record_stop"):
                self.state.set_perception_status(
                    running=bool(self.state.perception_running),
                    failed=True,
                    msg="remote: recording unsupported by host client",
                )
                return False
            self.client.send_perception_record_stop()
            self.state.set_perception_recording(False)
            self.state.set_perception_status(
                running=bool(self.state.perception_running),
                failed=False,
                msg="remote: recording stop requested on Jetson",
            )
            return True
        cap = self._perception_capture
        if cap is None:
            self.state.set_perception_recording(False)
            self.state.set_perception_status(running=False, failed=True, msg="perception is not running")
            return False
        ok, path_s, frame_count = cap.stop_recording()
        if not ok:
            self._stop_side_camera_recording()
            self.state.set_perception_status(
                running=bool(self.state.perception_running),
                failed=True,
                msg="recording is not active",
            )
            return False
        side_result = self._stop_side_camera_recording()
        side_msg = ""
        if side_result is not None:
            side_ok, side_path_s, side_frames, side_unique, side_err = side_result
            if side_ok:
                side_msg = " | side %df/%du: %s" % (side_frames, side_unique, side_path_s)
            else:
                side_msg = f" | side failed: {side_err or side_path_s}"
        self.state.set_perception_recording(False, path_s)
        self.state.set_perception_status(
            running=bool(self.state.perception_running),
            failed=False,
            msg=f"recording saved ({frame_count}f): {path_s}{side_msg}",
        )
        print(f"[perception] recording saved ({frame_count}f): {path_s}{side_msg}")
        return True

    def toggle_perception_recording(self) -> bool:
        if bool(self.state.perception_recording):
            return self.stop_perception_recording()
        return self.start_perception_recording()

    def _capture_sim_perception_frame_once(self, out_dir: Path) -> Optional[Path]:
        if str(self._perception_cfg.mode).strip().lower() != "sim":
            return None
        _ensure_pick_place_path()
        try:
            from engine.vision.perception.sim_rendered_camera import SimRenderedCamera
        except Exception as exc:
            print(f"[perception] sim camera import failed: {exc}")
            return None
        cfg = self._perception_cfg
        try:
            with SimRenderedCamera(
                endpoint=str(cfg.sim_camera_port),
                use_jpeg=bool(cfg.sim_camera_jpeg),
            ) as cam:
                frame = cam.capture(retries=60)
            return save_perception_frame_bundle(
                out_dir=out_dir,
                color_bgr=frame.color_bgr,
                depth_raw=frame.depth_raw,
                meta={
                    "mode": "sim",
                    "one_shot": True,
                    "depth_scale": float(frame.depth_scale),
                },
            )
        except Exception as exc:
            print(f"[perception] one-shot sim capture failed: {exc}")
            return None

    def update_perception_config(self, config: PerceptionConfig) -> None:
        self._perception_cfg = config
        self._perception_run_local = self._perception_config_runs_locally(config)
        self.state.visual_target_label = str(config.target_label).strip()

    def _mock_world_xyz_from_state(self) -> Optional[tuple[float, float, float]]:
        if str(self._perception_cfg.mode).strip().lower() != "mock":
            return None
        return self.state.mock_object_world_xyz()

    def set_mock_object_world(self, x: float, y: float, z: float) -> None:
        self.state.set_mock_object_world_xyz(float(x), float(y), float(z))

    def mock_object_preferred_dir(self) -> tuple[float, float, float]:
        return self.state.mock_object_preferred_dir()

    def set_mock_object_preferred_dir(self, x: float, y: float, z: float) -> None:
        self.state.set_mock_object_preferred_dir(float(x), float(y), float(z))

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
        current_u = self.current_control_u()
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
            self.apply_partial_control_u(partial)
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
        current_u = self.current_control_u()
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
            self.apply_partial_control_u(partial)
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
        from engine.behaviors.gaze.stabilizer import resolve_walking_gaze_mode

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

    def reset_simulation(self) -> None:
        """Reset sim GO2+arm pose, stop workers, and zero teleop commands."""
        self.stop_gaze_stabilizer()
        self.stop_object_pick()
        self.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
        start_q = default_start_sim_q(self._mapping_cfg)
        self.state.set_q(
            float(start_q.linear_m),
            float(start_q.roll_rad),
            float(start_q.theta1_rad),
            float(start_q.theta2_rad),
        )
        self.state.clear_ik_status()
        self.state.set_pick_status(running=False, failed=False, phase=ObjectPickPhase.IDLE.value, msg="")
        if self.client is not None:
            self.client.send_sim_reset()
            self.send_current_target(source="sim", force=True)
        print("[ctrl] simulation reset requested")
