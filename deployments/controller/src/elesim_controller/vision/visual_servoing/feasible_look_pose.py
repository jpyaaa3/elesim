"""Deprecated: tip-fixed look direction search.

Look now uses resolve_feasible_ready_pose (view pregrasp move) from actions.start_look.
This module is kept for reference/tests only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Optional, Sequence

import numpy as np

from .pick_view_pregrasp import (
    ViewPregraspCandidate,
    ViewPregraspLimits,
    evaluate_view_candidate,
    generate_view_pregrasp_candidates,
)
from .ready_pose import compute_ready_pose_target


def _default_solve_then_align(**kwargs: Any) -> Any:
    from elesim_controller.robot.arm import ik as ik_pipeline

    return ik_pipeline.solve_then_align(**kwargs)


def _normalize(vec: Sequence[float]) -> Optional[np.ndarray]:
    v = np.asarray(vec, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    if n <= 1e-9:
        return None
    return v / n


def _direction_angle_rad(actual_dir_unit: np.ndarray, desired_dir_unit: np.ndarray) -> float:
    dot = float(np.clip(np.dot(actual_dir_unit, desired_dir_unit), -1.0, 1.0))
    return float(math.acos(dot))


@dataclass(frozen=True)
class FeasibleLookPoseResult:
    success: bool
    resolved_dir: Optional[tuple[float, float, float]] = None
    q: Optional[np.ndarray] = None
    direction_angle_rad: float = float("inf")
    candidate_tag: str = ""
    requested_dir: tuple[float, float, float] = (0.0, 0.0, 0.0)
    user_dir_delta_deg: float = float("inf")
    evaluated_count: int = 0
    best_rejected_dir_err_deg: float = float("inf")
    position_error_m: float = float("inf")
    camera_score: float = float("-inf")
    reason: str = ""


@dataclass(frozen=True)
class _RankKey:
    # Primary: match candidate desired direction (solve_then_align direction_angle_rad)
    direction_angle_rad: float
    # Secondary: camera score (higher better)
    neg_camera_score: float
    # Tertiary: how close candidate direction is to original desired direction
    neg_user_pref: float
    position_error_m: float

    def __lt__(self, other: "_RankKey") -> bool:
        return (
            self.direction_angle_rad,
            self.neg_camera_score,
            self.neg_user_pref,
            self.position_error_m,
        ) < (
            other.direction_angle_rad,
            other.neg_camera_score,
            other.neg_user_pref,
            other.position_error_m,
        )


def _user_preferred_candidate(
    *,
    object_world: Sequence[float],
    preferred_dir_unit: np.ndarray,
    standoff_m: float,
) -> Optional[ViewPregraspCandidate]:
    try:
        pregrasp = compute_ready_pose_target(
            object_world,
            preferred_dir_unit,
            standoff_m=float(standoff_m),
        )
    except ValueError:
        return None
    obj = np.asarray(object_world, dtype=float).reshape(3)
    look = _normalize(obj - np.asarray(pregrasp, dtype=float).reshape(3))
    if look is None:
        return None
    return ViewPregraspCandidate(
        pregrasp_world=(float(pregrasp[0]), float(pregrasp[1]), float(pregrasp[2])),
        look_dir_world=(float(look[0]), float(look[1]), float(look[2])),
        tag="user_preferred",
    )


def _build_candidates(
    *,
    object_world: Sequence[float],
    preferred_dir_unit: np.ndarray,
    standoff_m: float,
    lateral_offsets_m: Sequence[float],
    height_offsets_m: Sequence[float],
) -> list[ViewPregraspCandidate]:
    base_offset = -preferred_dir_unit * float(max(standoff_m, 0.0))
    grid = generate_view_pregrasp_candidates(
        object_world,
        base_offset_m=base_offset,
        view_distance_m=float(standoff_m),
        lateral_offsets_m=lateral_offsets_m,
        height_offsets_m=height_offsets_m,
    )
    user_cand = _user_preferred_candidate(
        object_world=object_world,
        preferred_dir_unit=preferred_dir_unit,
        standoff_m=standoff_m,
    )
    out: list[ViewPregraspCandidate] = []
    seen: set[str] = set()
    if user_cand is not None:
        out.append(user_cand)
        seen.add(user_cand.tag)
    for cand in grid:
        if cand.tag in seen:
            continue
        seen.add(cand.tag)
        out.append(cand)
    return out


def resolve_feasible_look_pose(
    *,
    tip_world: Sequence[float],
    object_world: Sequence[float],
    desired_look_dir: Sequence[float],
    standoff_m: float,
    ik_context: dict[str, Any],
    current_seed: Sequence[float],
    position_tol_m: float,
    max_iters: int,
    max_dir_error_deg: float = 10.0,
    skip_search_under_deg: float = 5.0,
    lateral_offsets_m: Sequence[float] = (-0.05, 0.0, 0.05),
    height_offsets_m: Sequence[float] = (0.0, 0.05, 0.10),
    look_dot_min: float = 0.85,
    view_limits: Optional[ViewPregraspLimits] = None,
    hand_eye_transform: Optional[np.ndarray] = None,
    hand_eye_parent_frame: str = "node9",
    solve_fn: Optional[Callable[..., Any]] = None,
) -> FeasibleLookPoseResult:
    """Return best (q, dir) pair that keeps tip position feasible."""
    solve = solve_fn or _default_solve_then_align

    desired_unit = _normalize(desired_look_dir)
    if desired_unit is None:
        return FeasibleLookPoseResult(
            success=False,
            requested_dir=(0.0, 0.0, 0.0),
            reason="invalid desired look dir",
        )

    requested_dir = (float(desired_unit[0]), float(desired_unit[1]), float(desired_unit[2]))
    limits = view_limits or ViewPregraspLimits()
    max_dir_error_rad = math.radians(float(max(max_dir_error_deg, 0.0)))
    skip_search_under_rad = math.radians(float(max(skip_search_under_deg, 0.0)))

    candidates = _build_candidates(
        object_world=object_world,
        preferred_dir_unit=desired_unit,
        standoff_m=float(standoff_m),
        lateral_offsets_m=lateral_offsets_m,
        height_offsets_m=height_offsets_m,
    )
    if not candidates:
        return FeasibleLookPoseResult(
            success=False,
            requested_dir=requested_dir,
            reason="no candidates generated",
        )

    tip_arr = np.asarray(tip_world, dtype=float).reshape(3)
    he_transform = (
        None
        if hand_eye_transform is None
        else np.asarray(hand_eye_transform, dtype=float).reshape(4, 4).copy()
    )

    evaluated_count = 0
    best: Optional[tuple[_RankKey, ViewPregraspCandidate, Any, float, float]] = None
    # (rank_key, cand, ik_result, camera_score, user_pref)
    best_rejected_dir_err_rad = float("inf")

    def _eval_candidate(cand: ViewPregraspCandidate) -> Optional[tuple[_RankKey, ViewPregraspCandidate, Any, float, float]]:
        nonlocal evaluated_count, best_rejected_dir_err_rad
        evaluated_count += 1
        target_dir = np.asarray(cand.look_dir_world, dtype=float).reshape(3)

        ik_result = solve(
            target_world=tip_arr,
            target_dir_world=target_dir,
            context=ik_context,
            position_tol_m=float(position_tol_m),
            max_iters=max(int(max_iters), 1),
            current_seed=current_seed,
        )

        if bool(ik_result.success) and getattr(ik_result, "q", None) is not None:
            dir_err = float(ik_result.direction_angle_rad)
            best_rejected_dir_err_rad = min(best_rejected_dir_err_rad, dir_err)

            if dir_err > max_dir_error_rad:
                return None

            # Preference: candidate direction should be close to original desired direction.
            cand_unit = _normalize(target_dir)
            user_pref = float(np.dot(cand_unit, desired_unit)) if cand_unit is not None else -1.0

            camera_score = 0.0
            if he_transform is not None:
                metrics = evaluate_view_candidate(
                    ik_result.q,
                    object_world,
                    ik_context=ik_context,
                    hand_eye_transform=he_transform,
                    parent_frame=str(hand_eye_parent_frame),
                    limits=limits,
                )
                if metrics is None:
                    return None
                if float(metrics.look_dot) < float(look_dot_min):
                    return None
                camera_score = float(metrics.score)

            rank = _RankKey(
                direction_angle_rad=dir_err,
                neg_camera_score=-camera_score,
                neg_user_pref=-user_pref,
                position_error_m=float(ik_result.position_error_m),
            )
            return rank, cand, ik_result, camera_score, user_pref
        return None

    # Fast path: evaluate user_preferred first
    user_cand: Optional[ViewPregraspCandidate] = next(
        (c for c in candidates if c.tag == "user_preferred"), None
    )
    if user_cand is not None:
        row = _eval_candidate(user_cand)
        if row is not None:
            rank, cand, ik_result, _camera_score, _user_pref = row
            if float(rank.direction_angle_rad) <= skip_search_under_rad:
                resolved_dir = (float(cand.look_dir_world[0]), float(cand.look_dir_world[1]), float(cand.look_dir_world[2]))
                look_unit = _normalize(resolved_dir)
                delta_deg = (
                    float(_direction_angle_rad(look_unit, desired_unit) * 180.0 / math.pi)
                    if look_unit is not None
                    else float("inf")
                )
                return FeasibleLookPoseResult(
                    success=True,
                    resolved_dir=resolved_dir,
                    q=np.asarray(ik_result.q, dtype=float).reshape(4).copy(),
                    direction_angle_rad=float(ik_result.direction_angle_rad),
                    candidate_tag=str(cand.tag),
                    requested_dir=requested_dir,
                    user_dir_delta_deg=delta_deg,
                    evaluated_count=evaluated_count,
                    best_rejected_dir_err_deg=float(math.degrees(best_rejected_dir_err_rad)),
                    position_error_m=float(ik_result.position_error_m),
                    camera_score=0.0,
                    reason="fast_path",
                )
            best = (rank, cand, ik_result, 0.0, float(0.0))  # camera_score/user_pref overwritten below

    for cand in candidates:
        if user_cand is not None and cand.tag == user_cand.tag:
            continue
        row = _eval_candidate(cand)
        if row is None:
            continue
        rank, cand_best, ik_best, camera_score, user_pref = row
        if best is None:
            best = (rank, cand_best, ik_best, camera_score, user_pref)
        else:
            best_rank = best[0]
            if rank < best_rank:
                best = (rank, cand_best, ik_best, camera_score, user_pref)

    if best is None:
        return FeasibleLookPoseResult(
            success=False,
            requested_dir=requested_dir,
            evaluated_count=evaluated_count,
            best_rejected_dir_err_deg=float(math.degrees(best_rejected_dir_err_rad)),
            reason="no feasible look dir",
        )

    rank, cand, ik_result, camera_score, _user_pref = best
    resolved_dir = (float(cand.look_dir_world[0]), float(cand.look_dir_world[1]), float(cand.look_dir_world[2]))
    look_unit = _normalize(resolved_dir)
    delta_deg = (
        float(_direction_angle_rad(look_unit, desired_unit) * 180.0 / math.pi)
        if look_unit is not None
        else float("inf")
    )
    return FeasibleLookPoseResult(
        success=True,
        resolved_dir=resolved_dir,
        q=np.asarray(ik_result.q, dtype=float).reshape(4).copy(),
        direction_angle_rad=float(ik_result.direction_angle_rad),
        candidate_tag=str(cand.tag),
        requested_dir=requested_dir,
        user_dir_delta_deg=delta_deg,
        evaluated_count=evaluated_count,
        best_rejected_dir_err_deg=float(math.degrees(best_rejected_dir_err_rad)),
        position_error_m=float(ik_result.position_error_m),
        camera_score=float(camera_score),
        reason="grid_search",
    )


__all__ = [
    "FeasibleLookPoseResult",
    "resolve_feasible_look_pose",
]

