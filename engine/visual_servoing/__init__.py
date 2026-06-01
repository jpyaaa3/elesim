"""Reusable visual-servo and view-planning helpers.

This package keeps math helpers and candidate-generation utilities that are
useful across pick experiments, without declaring every past runtime strategy
as part of the current official control flow.

Current official runtime path:
- engine.controller.perception_capture
- engine.controller.object_pick

This package is intentionally lower-level than that runtime path.
"""

from .pick_view_pregrasp import (
    ViewCandidateMetrics,
    ViewPregraspCandidate,
    ViewPregraspLimits,
    camera_visibility_fail_reasons,
    camera_visibility_ok,
    camera_visibility_score,
    camera_visibility_score_soft,
    evaluate_view_candidate,
    format_view_candidate_log,
    generate_view_pregrasp_candidates,
    pick_best_strict_candidate,
    pick_best_visible_candidate,
    view_candidate_passes,
    view_candidate_passes_strict,
    view_candidate_score,
)
from .equal_sag_probe import (
    EqualSagEstimate,
    apply_equal_sag_offsets,
    estimate_equal_sag_from_ready_pose_drift,
    solve_equal_sag_offsets,
)
from .pick_visual_servo import (
    LOOK_JACOBIAN_AXIS_NAMES,
    JacobianLookGains,
    LookAlignLimits,
    LookGains,
    Q4Delta,
    advance_allowed,
    apply_q_delta,
    apply_q_delta_to_tuple,
    camera_xy_error,
    compute_advance_delta_q,
    compute_backoff_delta_q,
    compute_jacobian_look_delta_q,
    compute_look_delta_q,
    damped_pseudoinverse,
    error_vector_2d,
    estimate_jacobian_column,
    jacobian_column_usable,
    look_align_ok,
    q4_tuple_to_delta,
    should_send_look_command,
    stack_jacobian,
)
from .ready_pose import compute_ready_pose_target

__all__ = [
    "LOOK_JACOBIAN_AXIS_NAMES",
    "EqualSagEstimate",
    "JacobianLookGains",
    "LookAlignLimits",
    "LookGains",
    "Q4Delta",
    "ViewCandidateMetrics",
    "ViewPregraspCandidate",
    "ViewPregraspLimits",
    "advance_allowed",
    "apply_equal_sag_offsets",
    "apply_q_delta",
    "apply_q_delta_to_tuple",
    "camera_visibility_fail_reasons",
    "camera_visibility_ok",
    "camera_visibility_score",
    "camera_visibility_score_soft",
    "camera_xy_error",
    "compute_advance_delta_q",
    "compute_backoff_delta_q",
    "compute_jacobian_look_delta_q",
    "compute_look_delta_q",
    "compute_ready_pose_target",
    "damped_pseudoinverse",
    "error_vector_2d",
    "estimate_jacobian_column",
    "estimate_equal_sag_from_ready_pose_drift",
    "evaluate_view_candidate",
    "format_view_candidate_log",
    "generate_view_pregrasp_candidates",
    "jacobian_column_usable",
    "look_align_ok",
    "pick_best_strict_candidate",
    "pick_best_visible_candidate",
    "q4_tuple_to_delta",
    "should_send_look_command",
    "solve_equal_sag_offsets",
    "stack_jacobian",
    "view_candidate_passes",
    "view_candidate_passes_strict",
    "view_candidate_score",
]
