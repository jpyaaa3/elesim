"""Reusable visual-servo and view-planning helpers.

This package keeps math helpers and candidate-generation utilities that are
useful across pick experiments, without declaring every past runtime strategy
as part of the current official control flow.

Current official runtime path:
- elesim_pilot.vision.perception.capture
- elesim_pilot.vision.pick.core

This package is intentionally lower-level than that runtime path.
"""

from __future__ import annotations

_EXPORTS = {
    "LOOK_JACOBIAN_AXIS_NAMES": "pick_visual_servo",
    "EqualSagEstimate": "equal_sag_probe",
    "JacobianLookGains": "pick_visual_servo",
    "LookAlignLimits": "pick_visual_servo",
    "LookGains": "pick_visual_servo",
    "Q4Delta": "pick_visual_servo",
    "UV_CONTROL_AXIS_NAMES": "uv_jacobian",
    "ViewCandidateMetrics": "pick_view_pregrasp",
    "ViewPregraspCandidate": "pick_view_pregrasp",
    "ViewPregraspLimits": "pick_view_pregrasp",
    "advance_allowed": "pick_visual_servo",
    "apply_equal_sag_offsets": "equal_sag_probe",
    "apply_q_delta": "pick_visual_servo",
    "apply_q_delta_to_tuple": "pick_visual_servo",
    "broyden_update_uv_jacobian": "uv_jacobian",
    "camera_visibility_fail_reasons": "pick_view_pregrasp",
    "camera_visibility_ok": "pick_view_pregrasp",
    "camera_visibility_score": "pick_view_pregrasp",
    "camera_visibility_score_soft": "pick_view_pregrasp",
    "camera_xy_error": "pick_visual_servo",
    "compute_advance_delta_q": "pick_visual_servo",
    "compute_backoff_delta_q": "pick_visual_servo",
    "compute_jacobian_look_delta_q": "pick_visual_servo",
    "compute_look_delta_q": "pick_visual_servo",
    "compute_ready_pose_target": "ready_pose",
    "damped_pseudoinverse": "pick_visual_servo",
    "default_uv_jacobian": "uv_jacobian",
    "error_vector_2d": "pick_visual_servo",
    "estimate_equal_sag_from_ready_pose_drift": "equal_sag_probe",
    "estimate_jacobian_column": "pick_visual_servo",
    "evaluate_view_candidate": "pick_view_pregrasp",
    "FeasibleLookPoseResult": "feasible_look_pose",
    "FeasibleReadyPoseResult": "feasible_ready_pose",
    "format_view_candidate_log": "pick_view_pregrasp",
    "generate_view_pregrasp_candidates": "pick_view_pregrasp",
    "jacobian_column_usable": "pick_visual_servo",
    "look_align_ok": "pick_visual_servo",
    "pick_best_strict_candidate": "pick_view_pregrasp",
    "pick_best_visible_candidate": "pick_view_pregrasp",
    "q4_tuple_to_delta": "pick_visual_servo",
    "resolve_feasible_look_pose": "feasible_look_pose",
    "resolve_feasible_ready_pose": "feasible_ready_pose",
    "should_send_look_command": "pick_visual_servo",
    "solve_equal_sag_offsets": "equal_sag_probe",
    "solve_uv_control_delta": "uv_jacobian",
    "stack_jacobian": "pick_visual_servo",
    "view_candidate_passes": "pick_view_pregrasp",
    "view_candidate_passes_strict": "pick_view_pregrasp",
    "view_candidate_score": "pick_view_pregrasp",
}

__all__ = sorted(_EXPORTS)


def __getattr__(name: str):
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    import importlib

    module = importlib.import_module(f"{__name__}.{module_name}")
    return getattr(module, name)
