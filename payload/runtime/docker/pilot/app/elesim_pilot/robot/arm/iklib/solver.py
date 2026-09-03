from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Any, Optional, Sequence

import numpy as np

from elesim_pilot.config import AppConfigBundle, load_app_config
from ..joint_defs import JointLimit
from .kinematics import Q4, Q_BENT, Q_NEUTRAL, Vec3, _ReachModel


@dataclass(frozen=True)
class IkSolveRequest:
    target_world: Vec3
    position_tol_m: float = 1e-4


@dataclass(frozen=True)
class IkSolveResult:
    success: bool
    q: Optional[Q4]
    position_error_m: float
    seed_name: str
    iterations: int
    reason: str = ""


def load_solver_context(
    config_path: str,
    *,
    mode: str | None = None,
) -> tuple[AppConfigBundle, dict[str, Any]]:
    bundle = load_app_config(config_path, mode=mode)
    model_path = os.environ.get("ELESIM_ARM_MODEL", "").strip()
    if not model_path:
        config_dir = os.path.dirname(os.path.abspath(config_path))
        candidates = (
            os.path.abspath(os.path.join(config_dir, "../../data/models/arm/default.json")),
            "/opt/elesim/data/models/arm/default.json",
            os.path.join(config_dir, "arm_model.json"),
        )
        model_path = next((path for path in candidates if os.path.isfile(path)), candidates[0])
    if os.path.isfile(model_path):
        with open(model_path, "r", encoding="utf-8") as model_file:
            model = json.load(model_file)
        if int(model.get("schema_version", 0)) != 1:
            raise RuntimeError(f"unsupported arm model schema: {model_path}")
        context = dict(model.get("context", {}))
        limit_raw = context.get("limit", {})
        if isinstance(limit_raw, dict) and limit_raw.get("__dataclass__") == "JointLimit":
            context["limit"] = JointLimit(**dict(limit_raw.get("value", {})))
        if not isinstance(context.get("limit"), JointLimit):
            raise RuntimeError(f"arm model is missing JointLimit: {model_path}")
        return bundle, context

    raise FileNotFoundError(
        f"generated arm model not found: {model_path}; run elesim-build-arm-model"
    )


def _optimize_position(
    *,
    q0: Sequence[float],
    tol: float,
    model: _ReachModel,
    target_world: Sequence[float],
    max_iters: int,
    damping: float = 1e-2,
    line_search_shrink: float = 0.5,
    line_search_steps: int = 6,
) -> tuple[bool, Q4, float, int]:
    q = model.clamp_q(q0)
    err_vec = model.error_vec(q, target_world)
    err = float(np.linalg.norm(err_vec))
    if err <= tol:
        return True, q.copy(), err, 0

    for iteration in range(1, max(int(max_iters), 1) + 1):
        err_vec = model.error_vec(q, target_world)
        err = float(np.linalg.norm(err_vec))
        if err <= tol:
            return True, q.copy(), err, iteration - 1
        residual = np.asarray(err_vec, dtype=float).reshape(3)
        J = model.position_jacobian(q)
        H = J.T @ J + float(max(damping, 1e-9)) * np.eye(4, dtype=float)
        g = J.T @ residual
        try:
            step = -np.linalg.solve(H, g)
        except np.linalg.LinAlgError:
            step = -np.linalg.pinv(H) @ g
        accepted = False
        for ls_idx in range(max(int(line_search_steps), 1)):
            alpha = float(np.clip(line_search_shrink, 1e-3, 0.999)) ** ls_idx
            q_try = model.clamp_q(q + alpha * step)
            residual_try = np.asarray(model.error_vec(q_try, target_world), dtype=float).reshape(3)
            residual_norm = float(np.linalg.norm(residual))
            residual_try_norm = float(np.linalg.norm(residual_try))
            err_try = float(np.linalg.norm(model.error_vec(q_try, target_world)))
            if residual_try_norm < residual_norm:
                q = q_try
                err = err_try
                accepted = True
                break
        if not accepted:
            break
    return bool(err <= tol), q.copy(), float(err), max(int(max_iters), 1)


def solve_ik(
    *,
    target_world: Sequence[float],
    context: dict[str, Any],
    position_tol_m: float = 1e-4,
    max_iters: int = 120,
    neutral_seed: Optional[Sequence[float]] = None,
    bent_seed: Optional[Sequence[float]] = None,
    current_seed: Optional[Sequence[float]] = None,
) -> IkSolveResult:
    request = IkSolveRequest(
        target_world=np.asarray(target_world, dtype=float).reshape(3),
        position_tol_m=float(position_tol_m),
    )
    model = _ReachModel(context=context, limit=context["limit"])
    tol = float(max(request.position_tol_m, 0.0))
    best_q: Optional[Q4] = None
    best_err = float("inf")
    best_seed = "bent"
    best_iters = int(max_iters)
    seed_specs: list[tuple[str, np.ndarray]] = []
    if current_seed is not None:
        seed_specs.append(("current", np.asarray(current_seed, dtype=float).reshape(4)))
    seed_specs.extend(
        [
            ("neutral", np.asarray(neutral_seed if neutral_seed is not None else Q_NEUTRAL, dtype=float).reshape(4)),
            ("bent", np.asarray(bent_seed if bent_seed is not None else Q_BENT, dtype=float).reshape(4)),
        ]
    )
    seen: set[tuple[float, ...]] = set()
    for seed_name, q_seed in seed_specs:
        key = tuple(np.round(q_seed.astype(float), 9))
        if key in seen:
            continue
        seen.add(key)
        success, q_sol, err, iters = _optimize_position(
            q0=q_seed,
            tol=tol,
            model=model,
            target_world=request.target_world,
            max_iters=max_iters,
        )
        if err < best_err:
            best_q = q_sol
            best_err = err
            best_seed = seed_name
            best_iters = iters
        if success:
            return IkSolveResult(True, q_sol.copy(), err, seed_name, iters, "converged")
    return IkSolveResult(False, None if best_q is None else best_q.copy(), best_err, best_seed, best_iters, "position tolerance not reached")


def tighten_from_actual(
    *,
    current_q: Sequence[float],
    actual_tip_world: Sequence[float],
    target_world: Sequence[float],
    context: dict[str, Any],
    damping: float = 1e-2,
    step_scale: float = 1.0,
) -> np.ndarray:
    model = _ReachModel(context=context, limit=context["limit"])
    q = model.clamp_q(current_q)
    pos_err = np.asarray(target_world, dtype=float).reshape(3) - np.asarray(actual_tip_world, dtype=float).reshape(3)
    J = model.position_jacobian(q)
    H = J.T @ J + float(max(damping, 1e-9)) * np.eye(4, dtype=float)
    g = J.T @ pos_err
    try:
        dq = np.linalg.solve(H, g)
    except np.linalg.LinAlgError:
        dq = np.linalg.pinv(H) @ g
    return model.clamp_q(q + float(step_scale) * dq)


load_ik_context = load_solver_context


__all__ = [
    "IkSolveRequest",
    "IkSolveResult",
    "load_ik_context",
    "load_solver_context",
    "solve_ik",
    "tighten_from_actual",
]
