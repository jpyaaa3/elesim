"""Search for a robot-feasible ready pose direction via view-pregrasp candidates."""

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
    from engine import ik as ik_pipeline

    return ik_pipeline.solve_then_align(**kwargs)


def _normalize(vec: Sequence[float]) -> Optional[np.ndarray]:
    v = np.asarray(vec, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    if n <= 1e-9:
        return None
    return v / n


@dataclass(frozen=True)
class FeasibleReadyPoseResult:
    success: bool
    resolved_target: Optional[tuple[float, float, float]] = None
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
    direction_angle_rad: float
    neg_camera_score: float
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


@dataclass(frozen=True)
class _EvaluatedCandidate:
    candidate: ViewPregraspCandidate
    ik_result: Any
    camera_score: float
    user_pref: float
    rank: _RankKey


def _seed_preferred_candidate(
    object_world: Sequence[float],
    preferred_dir: Sequence[float],
    *,
    standoff_m: float,
) -> Optional[ViewPregraspCandidate]:
    preferred_unit = _normalize(preferred_dir)
    if preferred_unit is None:
        return None
    try:
        ready = compute_ready_pose_target(
            object_world,
            preferred_unit,
            standoff_m=float(standoff_m),
        )
    except ValueError:
        return None
    obj = np.asarray(object_world, dtype=float).reshape(3)
    look = _normalize(obj - np.asarray(ready, dtype=float).reshape(3))
    if look is None:
        return None
    return ViewPregraspCandidate(
        pregrasp_world=(float(ready[0]), float(ready[1]), float(ready[2])),
        look_dir_world=(float(look[0]), float(look[1]), float(look[2])),
        tag="seed_preferred",
    )


def _build_candidates(
    object_world: Sequence[float],
    preferred_dir: Sequence[float],
    *,
    standoff_m: float,
    lateral_offsets_m: Sequence[float],
    height_offsets_m: Sequence[float],
) -> list[ViewPregraspCandidate]:
    preferred_unit = _normalize(preferred_dir)
    if preferred_unit is None:
        return []

    base_offset = -preferred_unit * float(max(standoff_m, 0.0))
    grid = generate_view_pregrasp_candidates(
        object_world,
        base_offset_m=base_offset,
        view_distance_m=float(standoff_m),
        lateral_offsets_m=lateral_offsets_m,
        height_offsets_m=height_offsets_m,
    )

    seen_tags: set[str] = set()
    out: list[ViewPregraspCandidate] = []
    seed_cand = _seed_preferred_candidate(
        object_world,
        preferred_unit,
        standoff_m=float(standoff_m),
    )
    if seed_cand is not None:
        out.append(seed_cand)
        seen_tags.add(seed_cand.tag)

    for cand in grid:
        if cand.tag in seen_tags:
            continue
        seen_tags.add(cand.tag)
        out.append(cand)
    return out


def _solve_candidate(
    cand: ViewPregraspCandidate,
    *,
    ik_context: dict[str, Any],
    current_seed: Sequence[float],
    position_tol_m: float,
    max_iters: int,
    tweak_position_hold_tol_m: float,
    tweak_rounds: int,
    solve_fn: Callable[..., Any],
) -> Any:
    target = np.asarray(cand.pregrasp_world, dtype=float).reshape(3)
    direction = np.asarray(cand.look_dir_world, dtype=float).reshape(3)
    return solve_fn(
        target_world=target,
        target_dir_world=direction,
        context=ik_context,
        position_tol_m=float(position_tol_m),
        max_iters=max(int(max_iters), 1),
        current_seed=current_seed,
        tweak_position_hold_tol_m=float(tweak_position_hold_tol_m),
        tweak_rounds=int(tweak_rounds),
    )


def _evaluate_candidate(
    cand: ViewPregraspCandidate,
    ik_result: Any,
    *,
    object_world: Sequence[float],
    preferred_unit: np.ndarray,
    ik_context: dict[str, Any],
    max_dir_error_rad: float,
    look_dot_min: float,
    hand_eye_transform: Optional[np.ndarray],
    hand_eye_parent_frame: str,
    view_limits: ViewPregraspLimits,
) -> Optional[_EvaluatedCandidate]:
    if (not ik_result.success) or ik_result.q is None:
        return None

    direction = np.asarray(cand.look_dir_world, dtype=float).reshape(3)

    dir_err = float(ik_result.direction_angle_rad)
    if dir_err > float(max_dir_error_rad):
        return None

    camera_score = 0.0
    if hand_eye_transform is not None:
        metrics = evaluate_view_candidate(
            ik_result.q,
            object_world,
            ik_context=ik_context,
            hand_eye_transform=hand_eye_transform,
            parent_frame=str(hand_eye_parent_frame),
            limits=view_limits,
        )
        if metrics is None:
            return None
        if float(metrics.look_dot) < float(look_dot_min):
            return None
        camera_score = float(metrics.score)

    look_unit = _normalize(direction)
    user_pref = float(np.dot(look_unit, preferred_unit)) if look_unit is not None else -1.0
    rank = _RankKey(
        direction_angle_rad=dir_err,
        neg_camera_score=-camera_score,
        neg_user_pref=-user_pref,
        position_error_m=float(ik_result.position_error_m),
    )
    return _EvaluatedCandidate(
        candidate=cand,
        ik_result=ik_result,
        camera_score=camera_score,
        user_pref=user_pref,
        rank=rank,
    )


def resolve_feasible_ready_pose(
    *,
    object_world: Sequence[float],
    preferred_dir: Sequence[float],
    standoff_m: float,
    ik_context: dict[str, Any],
    current_seed: Sequence[float],
    position_tol_m: float,
    max_iters: int,
    tweak_position_hold_tol_m: float = 1.5e-2,
    tweak_rounds: int = 10,
    max_dir_error_deg: float = 10.0,
    skip_search_under_deg: float = 5.0,
    lateral_offsets_m: Sequence[float] = (-0.05, 0.0, 0.05),
    height_offsets_m: Sequence[float] = (0.0, 0.05, 0.10),
    look_dot_min: float = 0.85,
    view_limits: Optional[ViewPregraspLimits] = None,
    hand_eye_transform: Optional[np.ndarray] = None,
    hand_eye_parent_frame: str = "node9",
    solve_fn: Optional[Callable[..., Any]] = None,
) -> FeasibleReadyPoseResult:
    """Find a ready pose where IK can achieve both position and approach direction."""
    solve = solve_fn or _default_solve_then_align
    preferred_unit = _normalize(preferred_dir)
    if preferred_unit is None:
        return FeasibleReadyPoseResult(
            success=False,
            requested_dir=(0.0, 0.0, 0.0),
            reason="invalid preferred direction",
        )

    requested_dir = (
        float(preferred_unit[0]),
        float(preferred_unit[1]),
        float(preferred_unit[2]),
    )
    limits = view_limits or ViewPregraspLimits()
    max_dir_error_rad = math.radians(float(max(max_dir_error_deg, 0.0)))
    skip_search_under_rad = math.radians(float(max(skip_search_under_deg, 0.0)))

    candidates = _build_candidates(
        object_world,
        preferred_unit,
        standoff_m=float(standoff_m),
        lateral_offsets_m=lateral_offsets_m,
        height_offsets_m=height_offsets_m,
    )
    if not candidates:
        return FeasibleReadyPoseResult(
            success=False,
            requested_dir=requested_dir,
            reason="no candidates generated",
        )

    he_transform = None
    if hand_eye_transform is not None:
        he_transform = np.asarray(hand_eye_transform, dtype=float).reshape(4, 4)

    best_pass: Optional[_EvaluatedCandidate] = None
    best_rejected_dir_err_rad = float("inf")
    evaluated_count = 0

    def _eval_one(cand: ViewPregraspCandidate) -> Optional[_EvaluatedCandidate]:
        nonlocal evaluated_count, best_rejected_dir_err_rad
        evaluated_count += 1
        ik_result = _solve_candidate(
            cand,
            ik_context=ik_context,
            current_seed=current_seed,
            position_tol_m=position_tol_m,
            max_iters=max_iters,
            tweak_position_hold_tol_m=tweak_position_hold_tol_m,
            tweak_rounds=tweak_rounds,
            solve_fn=solve,
        )
        if ik_result.success and ik_result.q is not None:
            dir_err = float(ik_result.direction_angle_rad)
            if dir_err < best_rejected_dir_err_rad:
                best_rejected_dir_err_rad = dir_err
        return _evaluate_candidate(
            cand,
            ik_result,
            object_world=object_world,
            preferred_unit=preferred_unit,
            ik_context=ik_context,
            max_dir_error_rad=max_dir_error_rad,
            look_dot_min=look_dot_min,
            hand_eye_transform=he_transform,
            hand_eye_parent_frame=hand_eye_parent_frame,
            view_limits=limits,
        )

    seed_cand = candidates[0] if candidates[0].tag == "seed_preferred" else None
    seed_row: Optional[_EvaluatedCandidate] = None
    if seed_cand is not None:
        seed_row = _eval_one(seed_cand)
        if seed_row is not None and float(seed_row.ik_result.direction_angle_rad) <= skip_search_under_rad:
            resolved_dir = tuple(float(v) for v in seed_cand.look_dir_world)
            resolved_target = tuple(float(v) for v in seed_cand.pregrasp_world)
            look_unit = _normalize(resolved_dir)
            delta_deg = (
                float(math.degrees(math.acos(float(np.clip(np.dot(look_unit, preferred_unit), -1.0, 1.0)))))
                if look_unit is not None
                else float("inf")
            )
            return FeasibleReadyPoseResult(
                success=True,
                resolved_target=resolved_target,
                resolved_dir=resolved_dir,
                q=np.asarray(seed_row.ik_result.q, dtype=float).reshape(4).copy(),
                direction_angle_rad=float(seed_row.ik_result.direction_angle_rad),
                candidate_tag=str(seed_cand.tag),
                requested_dir=requested_dir,
                user_dir_delta_deg=delta_deg,
                evaluated_count=evaluated_count,
                best_rejected_dir_err_deg=float(math.degrees(best_rejected_dir_err_rad)),
                position_error_m=float(seed_row.ik_result.position_error_m),
                camera_score=float(seed_row.camera_score),
                reason="fast_path",
            )
        if seed_row is not None:
            best_pass = seed_row

    start_idx = 1 if seed_cand is not None else 0
    for cand in candidates[start_idx:]:
        row = _eval_one(cand)
        if row is None:
            continue
        if best_pass is None or row.rank < best_pass.rank:
            best_pass = row

    if best_pass is None:
        return FeasibleReadyPoseResult(
            success=False,
            requested_dir=requested_dir,
            evaluated_count=evaluated_count,
            best_rejected_dir_err_deg=float(math.degrees(best_rejected_dir_err_rad)),
            reason="no feasible ready dir",
        )

    cand = best_pass.candidate
    resolved_dir = tuple(float(v) for v in cand.look_dir_world)
    resolved_target = tuple(float(v) for v in cand.pregrasp_world)
    look_unit = _normalize(resolved_dir)
    delta_deg = (
        float(math.degrees(math.acos(float(np.clip(np.dot(look_unit, preferred_unit), -1.0, 1.0)))))
        if look_unit is not None
        else float("inf")
    )
    return FeasibleReadyPoseResult(
        success=True,
        resolved_target=resolved_target,
        resolved_dir=resolved_dir,
        q=np.asarray(best_pass.ik_result.q, dtype=float).reshape(4).copy(),
        direction_angle_rad=float(best_pass.ik_result.direction_angle_rad),
        candidate_tag=str(cand.tag),
        requested_dir=requested_dir,
        user_dir_delta_deg=delta_deg,
        evaluated_count=evaluated_count,
        best_rejected_dir_err_deg=float(math.degrees(best_rejected_dir_err_rad)),
        position_error_m=float(best_pass.ik_result.position_error_m),
        camera_score=float(best_pass.camera_score),
        reason="grid_search",
    )


__all__ = [
    "FeasibleReadyPoseResult",
    "resolve_feasible_ready_pose",
]
