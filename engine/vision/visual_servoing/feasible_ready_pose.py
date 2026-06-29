"""Search for a robot-feasible ready pose direction via view-pregrasp candidates."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Callable, Literal, Optional, Sequence

import numpy as np

if TYPE_CHECKING:
    from engine.observability.pick_timing import PickTimingCollector

from .pick_view_pregrasp import (
    ViewPregraspCandidate,
    ViewPregraspLimits,
    evaluate_view_candidate,
    generate_view_pregrasp_candidates,
)
from .ready_pose import compute_ready_pose_target

AlignMode = Literal["full", "lite"]


def _default_solve_then_align(**kwargs: Any) -> Any:
    from engine.robot.arm import ik as ik_pipeline

    return ik_pipeline.solve_then_align(**kwargs)


def _default_solve_position_only(**kwargs: Any) -> Any:
    from engine.robot.arm import ik as ik_pipeline

    return ik_pipeline.solve_position_only(**kwargs)


_default_solve_position_only.__position_only__ = True  # type: ignore[attr-defined]


def _make_align_solve_fn(
    *,
    align_mode: AlignMode,
    align_skip_under_deg: Optional[float],
    tweak_rounds: int,
) -> Callable[..., Any]:
    def solve_fn(**kwargs: Any) -> Any:
        from engine.robot.arm import ik as ik_pipeline

        kwargs.setdefault("align_mode", align_mode)
        kwargs.setdefault("align_skip_under_deg", align_skip_under_deg)
        kwargs.setdefault("tweak_rounds", int(tweak_rounds))
        return ik_pipeline.solve_then_align(**kwargs)

    return solve_fn


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
    timing: Optional["PickTimingCollector"] = None,
    align_mode: Optional[AlignMode] = None,
    align_skip_under_deg: Optional[float] = None,
) -> Any:
    target = np.asarray(cand.pregrasp_world, dtype=float).reshape(3)
    direction = np.asarray(cand.look_dir_world, dtype=float).reshape(3)
    if timing is not None:
        timing.ik_calls += 1
    kwargs: dict[str, Any] = dict(
        target_world=target,
        target_dir_world=direction,
        context=ik_context,
        position_tol_m=float(position_tol_m),
        max_iters=max(int(max_iters), 1),
        current_seed=current_seed,
    )
    position_only = bool(getattr(solve_fn, "__position_only__", False))
    if not position_only:
        kwargs["tweak_position_hold_tol_m"] = float(tweak_position_hold_tol_m)
        kwargs["tweak_rounds"] = int(tweak_rounds)
        if align_mode is not None:
            kwargs["align_mode"] = align_mode
        if align_skip_under_deg is not None:
            kwargs["align_skip_under_deg"] = float(align_skip_under_deg)
    if timing is not None:
        kwargs["timing"] = timing
    return solve_fn(**kwargs)


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
    timing: Optional["PickTimingCollector"] = None,
    enforce_dir_gate: bool = True,
    enforce_camera_gate: bool = True,
) -> Optional[_EvaluatedCandidate]:
    if (not ik_result.success) or ik_result.q is None:
        return None

    direction = np.asarray(cand.look_dir_world, dtype=float).reshape(3)

    dir_err = float(ik_result.direction_angle_rad)
    if enforce_dir_gate and dir_err > float(max_dir_error_rad):
        return None

    camera_score = 0.0
    if hand_eye_transform is not None:
        if timing is not None:
            with timing.span("view_eval"):
                metrics = evaluate_view_candidate(
                    ik_result.q,
                    object_world,
                    ik_context=ik_context,
                    hand_eye_transform=hand_eye_transform,
                    parent_frame=str(hand_eye_parent_frame),
                    limits=view_limits,
                )
        else:
            metrics = evaluate_view_candidate(
                ik_result.q,
                object_world,
                ik_context=ik_context,
                hand_eye_transform=hand_eye_transform,
                parent_frame=str(hand_eye_parent_frame),
                limits=view_limits,
            )
        if metrics is None:
            if enforce_camera_gate:
                return None
        elif enforce_camera_gate and float(metrics.look_dot) < float(look_dot_min):
            return None
        elif metrics is not None:
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


def _result_from_best(
    best_pass: _EvaluatedCandidate,
    *,
    requested_dir: tuple[float, float, float],
    preferred_unit: np.ndarray,
    evaluated_count: int,
    best_rejected_dir_err_rad: float,
    resolve_reason: str,
    timing: Optional["PickTimingCollector"],
) -> FeasibleReadyPoseResult:
    cand = best_pass.candidate
    resolved_dir = tuple(float(v) for v in cand.look_dir_world)
    resolved_target = tuple(float(v) for v in cand.pregrasp_world)
    look_unit = _normalize(resolved_dir)
    delta_deg = (
        float(math.degrees(math.acos(float(np.clip(np.dot(look_unit, preferred_unit), -1.0, 1.0)))))
        if look_unit is not None
        else float("inf")
    )
    if timing is not None:
        timing.candidates_evaluated = int(evaluated_count)
        timing.resolve_reason = str(resolve_reason)
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
        reason=str(resolve_reason),
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
    align_top_k: int = 0,
    align_mode: AlignMode = "full",
    align_skip_under_deg: Optional[float] = None,
    timing: Optional["PickTimingCollector"] = None,
    accept_best_effort_dir_error_deg: Optional[float] = None,
) -> FeasibleReadyPoseResult:
    """Find a ready pose where IK can achieve both position and approach direction."""
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
    use_two_phase = int(align_top_k) > 0 and solve_fn is None
    if solve_fn is None:
        if use_two_phase:
            screen_solve = _default_solve_position_only
            full_solve = _make_align_solve_fn(
                align_mode="full",
                align_skip_under_deg=align_skip_under_deg,
                tweak_rounds=int(tweak_rounds),
            )
            seed_solve = full_solve
            grid_solve = screen_solve
        else:
            solve = _make_align_solve_fn(
                align_mode=align_mode,
                align_skip_under_deg=align_skip_under_deg,
                tweak_rounds=int(tweak_rounds),
            )
            seed_solve = solve
            grid_solve = solve
    else:
        seed_solve = solve_fn
        grid_solve = solve_fn

    if timing is not None:
        with timing.span("candidate_build"):
            candidates = _build_candidates(
                object_world,
                preferred_unit,
                standoff_m=float(standoff_m),
                lateral_offsets_m=lateral_offsets_m,
                height_offsets_m=height_offsets_m,
            )
    else:
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
    best_near_miss: Optional[_EvaluatedCandidate] = None
    best_rejected_dir_err_rad = float("inf")
    evaluated_count = 0
    effort_limit_rad = math.radians(
        float(
            max(
                accept_best_effort_dir_error_deg
                if accept_best_effort_dir_error_deg is not None
                else max(float(max_dir_error_deg) * 1.5, 15.0),
                float(max_dir_error_deg),
            )
        )
    )

    def _track_near_miss(row: Optional[_EvaluatedCandidate]) -> None:
        nonlocal best_near_miss
        if row is None:
            return
        if best_near_miss is None or row.rank < best_near_miss.rank:
            best_near_miss = row

    def _track_dir_err(ik_result: Any) -> None:
        nonlocal best_rejected_dir_err_rad
        if ik_result.success and ik_result.q is not None:
            dir_err = float(ik_result.direction_angle_rad)
            if dir_err < best_rejected_dir_err_rad:
                best_rejected_dir_err_rad = dir_err

    def _eval_one(
        cand: ViewPregraspCandidate,
        *,
        solve: Callable[..., Any],
        seed: Sequence[float],
    ) -> Optional[_EvaluatedCandidate]:
        nonlocal evaluated_count
        evaluated_count += 1
        ik_result = _solve_candidate(
            cand,
            ik_context=ik_context,
            current_seed=seed,
            position_tol_m=position_tol_m,
            max_iters=max_iters,
            tweak_position_hold_tol_m=tweak_position_hold_tol_m,
            tweak_rounds=tweak_rounds,
            solve_fn=solve,
            timing=timing,
        )
        _track_dir_err(ik_result)
        row = _evaluate_candidate(
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
            timing=timing,
            enforce_dir_gate=True,
            enforce_camera_gate=True,
        )
        if row is not None:
            return row
        relaxed = _evaluate_candidate(
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
            timing=timing,
            enforce_dir_gate=False,
            enforce_camera_gate=False,
        )
        _track_near_miss(relaxed)
        return None

    def _run_search() -> FeasibleReadyPoseResult:
        nonlocal best_pass
        seed_cand = candidates[0] if candidates[0].tag == "seed_preferred" else None
        seed_row: Optional[_EvaluatedCandidate] = None
        if seed_cand is not None:
            seed_row = _eval_one(seed_cand, solve=seed_solve, seed=current_seed)
            if seed_row is not None and float(seed_row.ik_result.direction_angle_rad) <= skip_search_under_rad:
                resolved_dir = tuple(float(v) for v in seed_cand.look_dir_world)
                resolved_target = tuple(float(v) for v in seed_cand.pregrasp_world)
                look_unit = _normalize(resolved_dir)
                delta_deg = (
                    float(
                        math.degrees(
                            math.acos(float(np.clip(np.dot(look_unit, preferred_unit), -1.0, 1.0)))
                        )
                    )
                    if look_unit is not None
                    else float("inf")
                )
                if timing is not None:
                    timing.candidates_evaluated = int(evaluated_count)
                    timing.resolve_reason = "fast_path"
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
        grid_cands = candidates[start_idx:]

        if use_two_phase and grid_cands:
            screened: list[_EvaluatedCandidate] = []
            for cand in grid_cands:
                row = _eval_one(cand, solve=grid_solve, seed=current_seed)
                if row is None:
                    continue
                screened.append(row)
                if best_pass is None or row.rank < best_pass.rank:
                    best_pass = row

            screened.sort(key=lambda row: row.rank)
            top_k = max(int(align_top_k), 1)
            finalists = screened[:top_k]
            if timing is not None:
                with timing.span("align_top_k"):
                    for row in finalists:
                        seed_q = np.asarray(row.ik_result.q, dtype=float).reshape(4)
                        full_row = _eval_one(row.candidate, solve=full_solve, seed=seed_q)
                        if full_row is None:
                            continue
                        if best_pass is None or full_row.rank < best_pass.rank:
                            best_pass = full_row
            else:
                for row in finalists:
                    seed_q = np.asarray(row.ik_result.q, dtype=float).reshape(4)
                    full_row = _eval_one(row.candidate, solve=full_solve, seed=seed_q)
                    if full_row is None:
                        continue
                    if best_pass is None or full_row.rank < best_pass.rank:
                        best_pass = full_row
        else:
            for cand in grid_cands:
                row = _eval_one(cand, solve=grid_solve, seed=current_seed)
                if row is None:
                    continue
                if best_pass is None or row.rank < best_pass.rank:
                    best_pass = row

        if best_pass is None:
            if (
                best_near_miss is not None
                and float(best_near_miss.ik_result.direction_angle_rad) <= float(effort_limit_rad)
            ):
                if timing is not None:
                    timing.candidates_evaluated = int(evaluated_count)
                    timing.resolve_reason = "best_effort"
                return _result_from_best(
                    best_near_miss,
                    requested_dir=requested_dir,
                    preferred_unit=preferred_unit,
                    evaluated_count=evaluated_count,
                    best_rejected_dir_err_rad=best_rejected_dir_err_rad,
                    resolve_reason="best_effort",
                    timing=timing,
                )
            if timing is not None:
                timing.candidates_evaluated = int(evaluated_count)
                timing.resolve_reason = "no feasible ready dir"
            return FeasibleReadyPoseResult(
                success=False,
                requested_dir=requested_dir,
                evaluated_count=evaluated_count,
                best_rejected_dir_err_deg=float(math.degrees(best_rejected_dir_err_rad)),
                reason="no feasible ready dir",
            )

        resolve_reason = "grid_search_2phase" if use_two_phase else "grid_search"
        return _result_from_best(
            best_pass,
            requested_dir=requested_dir,
            preferred_unit=preferred_unit,
            evaluated_count=evaluated_count,
            best_rejected_dir_err_rad=best_rejected_dir_err_rad,
            resolve_reason=resolve_reason,
            timing=timing,
        )

    if timing is not None:
        with timing.span("resolve_grid"):
            return _run_search()
    return _run_search()


__all__ = [
    "AlignMode",
    "FeasibleReadyPoseResult",
    "resolve_feasible_ready_pose",
]
