from __future__ import annotations

import csv
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

from elesim_pilot.robot.arm import ik as ik_pipeline
from elesim_pilot.robot.arm.mounts.go2_mount import Go2ArmMount
from elesim_pilot.robot.arm.iklib import kinematics as ik_kin
from elesim_pilot.config import IkConfig, PerceptionConfig, PickConfig, SimConfig, load_app_config
from elesim_pilot.gaze.stabilizer import GazeStabilizerConfig, patch_gaze_config
from elesim_pilot.gaze.gaze_service import GazeControlService
from elesim_pilot.observability.tracing import traced_thread_target
from elesim_protocol import (
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
from elesim_pilot.experiment.walking_trial import host_horizontal_object_distance_m, standoff_base_pos
from elesim_pilot.robot.arm.sag_model import load_sag_model_json
from elesim_pilot.vision.visual_servoing.equal_sag_probe import (
    EqualSagEstimate,
    SagDriftComponents,
    apply_equal_sag_offsets,
    estimate_equal_sag_from_ready_pose_drift,
    prepare_sag_drift_input,
)
from elesim_pilot.observability.pick_timing import (
    PickPhaseProfile,
    PickTimingCollector,
    enabled as pick_profile_enabled,
    fk_call_count,
    format_report,
    install_fk_counter,
    reset_fk_count,
    uninstall_fk_counter,
)
from elesim_pilot.vision.visual_servoing.feasible_ready_pose import resolve_feasible_ready_pose
from elesim_pilot.vision.visual_servoing.grasp_trajectory import (
    GraspWaypoint,
    build_grasp_trajectory_markers,
)
from elesim_pilot.vision.visual_servoing.local_image_jacobian import (
    GraspApproachMode,
    ImageJacobianEstimator3D,
    LocalImageJacobianServo3D,
    LocalImageJacobianServoGains,
    SampleRejectReason,
    check_sample_quality,
    clip_dq,
    default_j_lji_seed,
    joint_saturated,
    null_space_projector_mn,
    z_jacobian_row_from_position_jacobian,
)
from elesim_pilot.vision.visual_servoing.uv_jacobian import (
    broyden_update_uv_jacobian,
    default_uv_jacobian,
    solve_uv_control_delta,
)

from .client import ControlClient
from elesim_pilot.vision.perception.observation import VisualObservation, extract_local_perception_observation, extract_visual_observation
from elesim_pilot.vision.pick.core import (
    ObjectPickPhase,
    compute_ready_pose_target,
    evaluate_pick_convergence,
    pick_ready_for_extend,
    pick_uv_deltas,
)
from elesim_pilot.vision.perception.capture import (
    PerceptionCapture,
    PerceptionSnapshot,
    TrackerPhase,
    default_perception_capture_dir,
    save_perception_frame_bundle,
    _ensure_pick_place_path,
)
from .state import HostState, PanelState

from .aim import AimActions
from .gaze_actions import GazeActions
from .grasp import GraspActions
from .perception import PerceptionActions
from .ready import ReadyActions
from .wrap import WrapActions
from .workflow import PickWorkflowPhase, run_pick_workflow


def _default_sag_model_path() -> str:
    for root in Path(__file__).resolve().parents:
        candidate = root / "payload/data/calibration/arm/sag_model.json"
        if candidate.is_file():
            return str(candidate)
    return "/opt/elesim/data/calibration/arm/sag_model.json"


DEFAULT_SAG_MODEL_PATH = _default_sag_model_path()


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
    """Compatibility API: return no initial model when the optional file is absent.

    Malformed or unreadable model files are real configuration failures and are
    deliberately not hidden behind the historical empty-model fallback.
    """

    try:
        return load_sag_model_or_empty(DEFAULT_SAG_MODEL_PATH)
    except FileNotFoundError:
        return {}


class _ControlServiceCore(
    ReadyActions, GraspActions, AimActions, PerceptionActions, GazeActions,
    WrapActions,
):
    """Construction, shared worker state, settling, and final gripper close."""

    def __init__(
        self,
        state: PanelState,
        client: Optional[ControlClient] = None,
        mapping_cfg: Optional[SimMappingConfig] = None,
        ik_cfg: Optional[IkConfig] = None,
        ik_context: Optional[dict[str, Any]] = None,
        config_path: Optional[str] = None,
        config_mode: Optional[str] = None,
        perception_cfg: Optional[PerceptionConfig] = None,
        pick_cfg: Optional[PickConfig] = None,
        gaze_cfg: Optional[GazeStabilizerConfig] = None,
        ownership_enable: bool = False,
        hand_eye_transform: Optional[np.ndarray] = None,
        hand_eye_parent_frame: str = "node9",
        go2_arm_mount: Optional[Go2ArmMount] = None,
        use_hardware: bool = True,
    ) -> None:
        self.state = state
        self.client = client
        self._use_hardware = bool(use_hardware)
        self._mapping_cfg = mapping_cfg or SimMappingConfig()
        self._ik_cfg = ik_cfg or IkConfig()
        self._ik_context = dict(ik_context or {})
        self._config_path = None if config_path is None else str(config_path)
        self._config_mode = None if config_mode is None else str(config_mode)
        self._perception_cfg = perception_cfg or PerceptionConfig()
        self._perception_run_local = self._perception_config_runs_locally(self._perception_cfg)
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
        self._observer_camera_recorder: Optional[Any] = None
        self._observer_camera_record_path: Optional[Path] = None
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
        self._grasp_lji_last_reliable_camera_xyz: Optional[tuple[float, float, float]] = None
        self._grasp_lji_last_good_q: Optional[np.ndarray] = None
        self._grasp_lji_pending_sample: Optional[dict[str, Any]] = None
        self._grasp_lji_last_dq_cmd: Optional[np.ndarray] = None
        self._grasp_lji_command_q: Optional[np.ndarray] = None
        self._grasp_lji_reacquire_anchor_dq: Optional[np.ndarray] = None
        self._grasp_lji_reacquire_steps = 0
        self._grasp_lji_reacquire_aim_tried = False
        self._grasp_lji_reacquire_prev_remain: Optional[float] = None
        self._grasp_lji_v_err_hist: list[float] = []
        self._grasp_lji_last_transition: str = "-"
        self._grasp_lji_sat_streak = 0
        self._grasp_lji_bad_motion_streak = 0
        self._grasp_lji_force_reacquire_reason = ""
        self._grasp_lji_remain_hist: list[float] = []
        self._grasp_lji_log_file: Optional[Any] = None
        self._grasp_lji_log_writer: Optional[csv.DictWriter] = None
        self._grasp_lji_log_path: str = ""
        self._grasp_lji_log_start_t: float = 0.0
        self._grasp_lji_log_seq: int = 0
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
        self._gaze_command_ref_u: Optional[ControlU] = None

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
        """Workflow computation is always owned by this deployment."""
        return False

    def _delegate_pick_to_host(self) -> bool:
        """Robot and sim endpoints never execute Pick workflows."""
        return False

    def _set_pick_failure(self, message: str) -> None:
        self.state.set_pick_status(
            running=False,
            failed=True,
            phase=ObjectPickPhase.FAILED.value,
            msg=message,
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
            self._set_pick_failure(fail_msg)
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
            self._set_pick_failure(fail_msg)
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
                    self._set_pick_failure(fail_msg)
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


class _PilotContextActions(_ControlServiceCore):
    """IK context, remote-state synchronization, visual state, and stop lifecycle."""

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
            _, ik_context = ik_pipeline.load_solver_context(
                self._config_path,
                mode=self._config_mode,
            )
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

    def _sync_remote_pick_from_host(self, host_state: HostState) -> None:
        if not self._delegate_pick_to_host():
            return
        self.state.set_pick_status(
            running=bool(getattr(host_state, "pick_running", False)),
            failed=bool(getattr(host_state, "pick_failed", False)),
            phase=str(
                getattr(host_state, "pick_phase", ObjectPickPhase.IDLE.value)
                or ObjectPickPhase.IDLE.value
            ),
            msg=str(getattr(host_state, "pick_status_msg", "") or ""),
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
        self._sync_remote_pick_from_host(host_state)
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

    def _grasp_lji_wait_visual_observation(
        self,
        host_state: Optional[HostState],
        *,
        wait_s: float = 0.25,
    ) -> tuple[Optional[VisualObservation], Optional[HostState]]:
        """LJI is sensitive to transient stale observations; poll briefly before reacquire."""
        obs = self.current_visual_observation(host_state)
        if obs is not None:
            return obs, host_state
        cap = self._perception_capture
        if cap is not None:
            cap.request_refresh()
        deadline = time.time() + float(max(wait_s, 0.0))
        while time.time() < deadline and not self._pick_stop_event.is_set():
            time.sleep(0.02)
            if self.client is not None:
                try:
                    host_state = self.client.refresh_state()
                except Exception:
                    pass
            obs = self.current_visual_observation(host_state)
            if obs is not None:
                return obs, host_state
        return None, host_state

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

    def _retire_finished_visual_workers(self) -> None:
        if self._ik_worker is not None and not self._ik_worker.is_alive():
            self._ik_worker = None
        if self._pick_worker is not None and not self._pick_worker.is_alive():
            self._pick_worker = None
        if self._pick_e2e_worker is not None and not self._pick_e2e_worker.is_alive():
            self._pick_e2e_worker = None
        if self._ik_worker is None and self._pick_worker is None and bool(self.state.ik_running):
            self.state.clear_ik_status()

    def _join_visual_worker_briefly(self, attr: str, *, timeout_s: float = 0.35) -> bool:
        worker = getattr(self, attr, None)
        if worker is None:
            return False
        if not worker.is_alive():
            setattr(self, attr, None)
            return False
        if threading.current_thread() is not worker:
            worker.join(timeout=float(max(timeout_s, 0.0)))
        if not worker.is_alive():
            setattr(self, attr, None)
            return False
        return True

    def pick_e2e_running(self) -> bool:
        self._retire_finished_visual_workers()
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
        if self._delegate_pick_to_host() and (
            hasattr(self.client, "send_pick_stop")
            or hasattr(self.client, "send_mobile_pick_stop")
        ):
            self._pick_e2e_cancel.set()
            try:
                if hasattr(self.client, "send_mobile_pick_stop"):
                    self.client.send_mobile_pick_stop()
                else:
                    self.client.send_pick_stop()
                self.state.set_pick_status(
                    running=False,
                    failed=False,
                    phase=ObjectPickPhase.IDLE.value,
                    msg="on-device pick stop requested",
                )
                host_state = self.client.refresh_state()
                self._sync_remote_pick_from_host(host_state)
                self._sync_remote_gaze_from_host(host_state)
                self._sync_remote_perception_from_host(host_state)
                print("[Pick] on-device stop requested")
            except Exception as exc:
                self._set_pick_failure(f"on-device pick stop failed: {exc}")
                print(f"[Pick] on-device stop failed: {exc}")
            return
        self._pick_e2e_cancel.set()
        self.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
        self.stop_gaze_stabilizer()
        self.stop_object_pick()


class _MobilePickWorkflowActions(_PilotContextActions):
    """Mobile gaze-to-handoff workflow and standalone LJI grasp launch."""

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
        if self._delegate_pick_to_host() and hasattr(self.client, "send_mobile_pick_start"):
            if (
                self.state.pick_running
                or self.pick_e2e_running()
                or self._pick_busy()
                or self.state.ik_running
                or self._ik_worker is not None
            ):
                self.state.set_pick_status(
                    running=bool(self.state.pick_running),
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg="busy",
                )
                return
            self._pick_e2e_cancel.clear()
            self._pick_stop_event.clear()
            try:
                self.client.send_mobile_pick_start()
                self.state.set_pick_status(
                    running=True,
                    failed=False,
                    phase=ObjectPickPhase.ACQUIRE.value,
                    msg="on-device mobile pick start requested",
                )
                host_state = self.client.refresh_state()
                self._sync_remote_pick_from_host(host_state)
                self._sync_remote_gaze_from_host(host_state)
                self._sync_remote_perception_from_host(host_state)
                print("[MobilePick] on-device start requested")
            except Exception as exc:
                self._set_pick_failure(f"on-device mobile pick start failed: {exc}")
                print(f"[MobilePick] on-device start failed: {exc}")
            return

        if self.pick_e2e_running() or self._pick_busy() or self.state.ik_running or self._ik_worker is not None:
            self._set_pick_failure("busy")
            return
        if self.client is None:
            self._set_pick_failure("no host client")
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
                    self._set_pick_failure(f"mobile pick: no target observation | {detail}")
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
                        self._set_pick_failure("mobile pick: LJI grasp timeout")
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
                self._set_pick_failure(f"mobile pick failed: {exc}")
                print(f"[MobilePick] failed: {exc}")
            finally:
                self.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
                self.stop_gaze_stabilizer()
                cancelled = bool(self._pick_e2e_cancel.is_set() or self._pick_stop_event.is_set())
                if not success and not self.state.pick_failed and not cancelled:
                    self._set_pick_failure("mobile pick failed")
                self._pick_e2e_worker = None

        self._pick_e2e_worker = threading.Thread(
            target=traced_thread_target("pick.e2e.mobile_gaze_lji", _worker),
            name="mobile-pick-e2e",
            daemon=True,
        )
        self._pick_e2e_worker.start()

    def start_lji_grasp_only(self) -> None:
        """Run arm-only LJI grasp without starting mobile gaze or locomotion."""
        if self._delegate_pick_to_host() and hasattr(self.client, "send_lji_grasp_start"):
            if (
                self.state.pick_running
                or self.pick_e2e_running()
                or self._pick_busy()
                or self.state.ik_running
                or self._ik_worker is not None
            ):
                self.state.set_pick_status(
                    running=bool(self.state.pick_running),
                    failed=True,
                    phase=ObjectPickPhase.FAILED.value,
                    msg="busy",
                )
                return
            self._pick_e2e_cancel.clear()
            self._pick_stop_event.clear()
            try:
                self.client.send_lji_grasp_start()
                self.state.set_pick_status(
                    running=True,
                    failed=False,
                    phase=ObjectPickPhase.GRASP.value,
                    msg="on-device LJI grasp start requested",
                )
                host_state = self.client.refresh_state()
                self._sync_remote_pick_from_host(host_state)
                self._sync_remote_gaze_from_host(host_state)
                self._sync_remote_perception_from_host(host_state)
                print("[Pick] on-device LJI grasp start requested")
            except Exception as exc:
                self._set_pick_failure(f"on-device LJI grasp start failed: {exc}")
                print(f"[Pick] on-device LJI grasp start failed: {exc}")
            return

        if self._use_hardware and not self._host_native_lji_runtime():
            self._set_pick_failure("LJI grasp must run on-device host-native in hardware mode")
            print("[Pick] blocked hardware LJI grasp outside host-native runtime")
            return

        if self.client is None:
            self._set_pick_failure("no host client")
            return
        self.send_go2_velocity(vx=0.0, vy=0.0, wz=0.0)
        self.stop_gaze_stabilizer()
        self._pick_e2e_cancel.set()
        self._pick_stop_event.set()
        self._retire_finished_visual_workers()
        busy_reasons: list[str] = []
        if self._join_visual_worker_briefly("_pick_e2e_worker"):
            busy_reasons.append("mobile pipeline stopping")
        if self._join_visual_worker_briefly("_pick_worker"):
            busy_reasons.append("pick worker stopping")
        if self._join_visual_worker_briefly("_ik_worker"):
            busy_reasons.append("ik worker running")
        self._retire_finished_visual_workers()
        if bool(self.state.ik_running):
            busy_reasons.append("ik state running")
        if busy_reasons:
            self._set_pick_failure("arm busy: " + ", ".join(busy_reasons))
            return
        self._pick_e2e_cancel.clear()
        self._pick_stop_event.clear()
        self._reset_pick_last_seen_uv()
        self._reset_pick_uv_jacobian()
        self._reset_pick_search_state()
        self._reset_pick_drift_accounting()
        self._reset_pick_equal_sag_result_state()
        self._reset_grasp_guided_state()
        self._start_grasp_to_object(internal=True)


class _VisualSearchActions(_MobilePickWorkflowActions):
    """Sequential Pick workflows, target reacquisition, and FOV search."""

    def _start_pick_workflow(
        self,
        *,
        phases: Sequence[PickWorkflowPhase],
        trace_name: str,
        description: str,
    ) -> None:
        if self.pick_e2e_running() or self._pick_busy() or self._visual_busy():
            self._set_pick_failure("busy")
            return
        if self.client is None:
            self._set_pick_failure("no host client")
            return

        self._pick_e2e_cancel.clear()
        timeout_s = float(self._pick_e2e_phase_timeout_s)

        def _begin_phase(phase: PickWorkflowPhase) -> None:
            self.state.set_pick_status(
                running=True,
                failed=False,
                phase=str(phase.state_phase),
                msg=f"E2E: {phase.label.title()}",
            )

        def _worker() -> None:
            try:
                print(f"[E2E] start | {description}")
                result = run_pick_workflow(
                    phases,
                    timeout_s=timeout_s,
                    begin_phase=_begin_phase,
                    wait_phase=lambda label, timeout: self._wait_pick_phase_done(
                        timeout_s=timeout,
                        label=label,
                    ),
                    failed=lambda: bool(self.state.pick_failed),
                    cancelled=self._pick_e2e_cancel.is_set,
                )
                if result.success:
                    self.state.set_pick_status(
                        running=False,
                        failed=False,
                        phase=ObjectPickPhase.DONE.value,
                        msg=f"E2E done | {description}",
                    )
                    print(f"[E2E] done | {description}")
                elif result.reason in {"timeout", "exception"} and not self.state.pick_failed:
                    detail = f": {result.detail}" if result.detail else ""
                    self._set_pick_failure(f"E2E: {result.phase} {result.reason}{detail}")
            finally:
                self._pick_e2e_worker = None

        self._pick_e2e_worker = threading.Thread(
            target=traced_thread_target(trace_name, _worker),
            name="pick-e2e",
            daemon=True,
        )
        self._pick_e2e_worker.start()

    def start_look_aim_grasp_e2e(self) -> None:
        """Run Look -> Aim -> Grasp (pre-contact IK + close gripper)."""
        self._start_pick_workflow(
            phases=(
                PickWorkflowPhase("look", ObjectPickPhase.LOOK.value, self.start_look),
                PickWorkflowPhase("aim", ObjectPickPhase.ACQUIRE.value, self.start_aim),
                PickWorkflowPhase("grasp", ObjectPickPhase.GRASP.value, self.start_grasp),
            ),
            trace_name="pick.e2e.look_aim_grasp",
            description="Look -> Aim -> Grasp",
        )

    def start_look_aim_e2e(self) -> None:
        """Run Look -> Aim only (no grasp)."""
        self._start_pick_workflow(
            phases=(
                PickWorkflowPhase("look", ObjectPickPhase.LOOK.value, self.start_look),
                PickWorkflowPhase("aim", ObjectPickPhase.ACQUIRE.value, self.start_aim),
            ),
            trace_name="pick.e2e.look_aim",
            description="Look -> Aim",
        )

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


class _MotionFeedbackActions(_VisualSearchActions):
    """Measured q handling, command waits, mapping, and IK result application."""

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

    def _host_native_lji_runtime(self) -> bool:
        hw_check = getattr(self.client, "has_hardware", None) if self.client is not None else None
        if callable(hw_check):
            hardware_value = hw_check()
            client_has_hw = isinstance(hardware_value, (bool, np.bool_)) and bool(hardware_value)
        else:
            client_has_hw = True
        return bool(
            self._use_hardware
            and self.client is not None
            and client_has_hw
            and getattr(self.client, "host_native_control", False) is True
            and hasattr(self.client, "apply_lji_q_direct")
        )

    def _grasp_lji_command_base_q(self, host_state: Optional[HostState]) -> np.ndarray:
        if self._host_native_lji_runtime() and self._grasp_lji_command_q is not None:
            return np.asarray(self._grasp_lji_command_q, dtype=float).reshape(4).copy()
        return self._q_array_from_state(host_state)

    def _refresh_lji_state(self) -> Optional[HostState]:
        if self.client is None:
            return None
        refresh = (
            getattr(self.client, "refresh_lji_state", None)
            if self._host_native_lji_runtime()
            else None
        )
        state = refresh() if callable(refresh) else self.client.refresh_state()
        return state if isinstance(state, HostState) else None

    def _stop_lji_velocity_control(self, reason: str) -> None:
        """Stop only the explicit host-native test/adapter contract.

        The normal DDS ``ControlClient`` owns no on-device LJI loop.  Calling
        its compatibility method and swallowing the resulting RuntimeError hid
        both that boundary and real stop failures from an actual adapter.
        """

        if not self._host_native_lji_runtime():
            return
        stop = getattr(self.client, "stop_lji_velocity_control", None)
        if not callable(stop):
            raise RuntimeError(
                "host-native LJI adapter must implement stop_lji_velocity_control"
            )
        stop(reason=str(reason))

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
        linear_tol_m: Optional[float] = None,
        angle_tol_rad: Optional[float] = None,
        consecutive: Optional[int] = None,
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
        wait_kwargs: dict[str, Any] = {}
        if linear_tol_m is not None:
            wait_kwargs["linear_tol_m"] = float(linear_tol_m)
        if angle_tol_rad is not None:
            wait_kwargs["angle_tol_rad"] = float(angle_tol_rad)
        if consecutive is not None:
            wait_kwargs["consecutive"] = int(consecutive)
        host_state, _settled = self._wait_until_q_settled(
            q_cmd,
            timeout_s=float(timeout_s),
            **wait_kwargs,
        )
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


class ControlService(_MotionFeedbackActions):
    """Operator-facing pilot commands and runtime configuration helpers."""

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

    def apply_partial_control_u(self, partial_u: dict[str, float], *, source: str = "slider") -> None:
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
            self.client.send_target_values(
                linear_m=float(self.state.linear),
                roll_rad=float(self.state.roll),
                theta1_rad=float(self.state.theta1),
                theta2_rad=float(self.state.theta2),
                source=str(source),
            )

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
            force or (not self.state.controls_locked) or (source == "target")
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

    def _latch_pick_frozen_world(self) -> None:
        self._pick_frozen_world_xyz = self._pick_frozen_world()

    def _retire_perception_capture(self, cap: PerceptionCapture, *, stop_recording: bool = True) -> None:
        if self._perception_capture is not cap:
            return
        if bool(stop_recording):
            self._stop_observer_camera_recording()
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
        """Panel/runtime overrides on top of the loaded deployment pick settings."""
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
        self._gaze_command_ref_u = None

    def _gaze_control_current_u(self) -> ControlU:
        actual_u = self.current_control_u()
        g = self._gaze_cfg
        if not bool(getattr(g, "command_ref_enable", False)):
            self._gaze_command_ref_u = actual_u
            return actual_u

        ref_u = self._gaze_command_ref_u
        if ref_u is None:
            self._gaze_command_ref_u = actual_u
            return actual_u

        lead = float(max(0.0, float(getattr(g, "command_ref_max_lead", 0.0))))
        if lead <= 0.0:
            self._gaze_command_ref_u = actual_u
            return actual_u

        roll_ref = float(ref_u.u_roll) if bool(g.enable_roll) else float(actual_u.u_roll)
        bounded = self._clamp_display_u(
            ControlU(
                u_linear=float(actual_u.u_linear),
                u_roll=float(np.clip(roll_ref, float(actual_u.u_roll - lead), float(actual_u.u_roll + lead))),
                u_s1=float(np.clip(ref_u.u_s1, float(actual_u.u_s1 - lead), float(actual_u.u_s1 + lead))),
                u_s2=float(np.clip(ref_u.u_s2, float(actual_u.u_s2 - lead), float(actual_u.u_s2 + lead))),
            )
        )
        self._gaze_command_ref_u = bounded
        return bounded

    def _set_gaze_command_ref(self, display_u: ControlU) -> None:
        if bool(getattr(self._gaze_cfg, "command_ref_enable", False)):
            self._gaze_command_ref_u = self._clamp_display_u(display_u)
        else:
            self._gaze_command_ref_u = None

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
