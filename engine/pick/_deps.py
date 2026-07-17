"""Shared implementation dependencies for Pick workflow mixins.

Workflow mixins intentionally share this import surface so that splitting the
large service does not create a second, slightly different set of robotics
conventions.
"""

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

from engine.config import IkConfig, PerceptionConfig, PickConfig, SimConfig, load_app_config
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
from engine.gaze.gaze_service import GazeControlService
from engine.gaze.stabilizer import GazeStabilizerConfig, patch_gaze_config
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
from engine.robot.arm import ik as ik_pipeline
from engine.robot.arm.iklib import kinematics as ik_kin
from engine.robot.arm.mounts.go2_mount import Go2ArmMount
from engine.robot.arm.sag_model import load_sag_model_json
from engine.vision.perception.capture import (
    PerceptionCapture,
    PerceptionSnapshot,
    TrackerPhase,
    _ensure_pick_place_path,
    default_perception_capture_dir,
    save_perception_frame_bundle,
)
from engine.vision.perception.observation import (
    VisualObservation,
    extract_local_perception_observation,
    extract_visual_observation,
)
from engine.vision.pick.core import (
    ObjectPickPhase,
    compute_ready_pose_target,
    evaluate_pick_convergence,
    pick_ready_for_extend,
    pick_uv_deltas,
)
from engine.vision.visual_servoing.equal_sag_probe import (
    EqualSagEstimate,
    SagDriftComponents,
    apply_equal_sag_offsets,
    estimate_equal_sag_from_ready_pose_drift,
    prepare_sag_drift_input,
)
from engine.vision.visual_servoing.feasible_ready_pose import resolve_feasible_ready_pose
from engine.vision.visual_servoing.grasp_trajectory import GraspWaypoint, build_grasp_trajectory_markers
from engine.vision.visual_servoing.local_image_jacobian import (
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
from engine.vision.visual_servoing.uv_jacobian import (
    broyden_update_uv_jacobian,
    default_uv_jacobian,
    solve_uv_control_delta,
)

from .client import ControlClient
from .state import HostState, PanelState
