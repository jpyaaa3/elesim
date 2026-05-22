#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import ast
import json
import math
import os
from typing import Any, Callable

import numpy as np


_ALLOWED_FUNCS: dict[str, Callable[..., float]] = {
    "abs": abs,
    "min": min,
    "max": max,
    "round": round,
}

_ALLOWED_CONSTS: dict[str, float] = {
    "pi": math.pi,
    "e": math.e,
}

_ALLOWED_BINOPS = (ast.Add, ast.Sub, ast.Mult, ast.Div, ast.FloorDiv, ast.Mod, ast.Pow, ast.BitXor)
_ALLOWED_UNARYOPS = (ast.UAdd, ast.USub)


def _eval_node(node: ast.AST, env: dict[str, float]) -> float:
    if isinstance(node, ast.Expression):
        return _eval_node(node.body, env)
    if isinstance(node, ast.Constant):
        if isinstance(node.value, (int, float)):
            return float(node.value)
        raise ValueError("only numeric constants are allowed")
    if isinstance(node, ast.Name):
        name = str(node.id)
        if name in env:
            return float(env[name])
        if name in _ALLOWED_CONSTS:
            return float(_ALLOWED_CONSTS[name])
        raise ValueError(f"unknown symbol '{name}'")
    if isinstance(node, ast.BinOp) and isinstance(node.op, _ALLOWED_BINOPS):
        lhs = _eval_node(node.left, env)
        rhs = _eval_node(node.right, env)
        if isinstance(node.op, ast.Add):
            return lhs + rhs
        if isinstance(node.op, ast.Sub):
            return lhs - rhs
        if isinstance(node.op, ast.Mult):
            return lhs * rhs
        if isinstance(node.op, ast.Div):
            return lhs / rhs
        if isinstance(node.op, ast.FloorDiv):
            return lhs // rhs
        if isinstance(node.op, ast.Mod):
            return lhs % rhs
        if isinstance(node.op, ast.Pow):
            return lhs ** rhs
        if isinstance(node.op, ast.BitXor):
            return lhs ** rhs
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, _ALLOWED_UNARYOPS):
        val = _eval_node(node.operand, env)
        return +val if isinstance(node.op, ast.UAdd) else -val
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
        fn_name = str(node.func.id)
        fn = _ALLOWED_FUNCS.get(fn_name)
        if fn is None:
            raise ValueError(f"function '{fn_name}' is not allowed")
        if node.keywords:
            raise ValueError("keyword arguments are not allowed")
        args = [_eval_node(arg, env) for arg in node.args]
        return float(fn(*args))
    raise ValueError(f"unsupported expression node: {type(node).__name__}")


def eval_sag_expr_deg(
    expr: str,
    *,
    i: int,
    n: int,
    theta1: float = 0.0,
    theta2: float = 0.0,
) -> float:
    text = str(expr or "").strip()
    if not text:
        return 0.0
    tree = ast.parse(text, mode="eval")
    env = {
        "i": float(i),
        "n": float(n),
        "theta1": float(theta1),
        "theta2": float(theta2),
    }
    return float(_eval_node(tree, env))


def parse_distribution_deg(text: str, count: int) -> np.ndarray:
    raw = str(text or "").strip()
    m = max(int(count), 0)
    if m <= 0:
        return np.zeros((0,), dtype=float)
    if not raw:
        return np.zeros((m,), dtype=float)
    parts = [p.strip() for p in raw.split(",")]
    if len(parts) != m:
        raise ValueError(f"distribution must have exactly {m} comma-separated values")
    vals = [float(eval_sag_expr_deg(p, i=0, n=m, theta1=0.0, theta2=0.0)) for p in parts]
    return np.array(vals, dtype=float)


def eval_amplitude_deg(expr: str, *, theta1: float = 0.0, theta2: float = 0.0) -> float:
    return eval_sag_expr_deg(expr, i=0, n=1, theta1=theta1, theta2=theta2)


def segment_errors_deg(
    distribution_text: str,
    amplitude_expr: str,
    count: int,
    *,
    theta1: float = 0.0,
    theta2: float = 0.0,
) -> np.ndarray:
    dist = parse_distribution_deg(distribution_text, count)
    amp = float(eval_amplitude_deg(amplitude_expr, theta1=theta1, theta2=theta2))
    return amp * dist


def load_sag_model_json(path: str) -> dict[str, Any]:
    raw_path = str(path or "").strip()
    if not raw_path:
        return {}
    if not os.path.isfile(raw_path):
        raise FileNotFoundError(f"sag model file not found: {raw_path}")
    with open(raw_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError("sag model file must contain a JSON object")
    return dict(data)


def load_sag_model_file(path: str) -> dict[str, str]:
    data = load_sag_model_json(path)
    return {
        "seg1_distribution": str(data.get("seg1_distribution", "") or ""),
        "seg1_amplitude": str(data.get("seg1_amplitude", "") or ""),
        "seg2_distribution": str(data.get("seg2_distribution", "") or ""),
        "seg2_amplitude": str(data.get("seg2_amplitude", "") or ""),
    }


def _eval_c_family_scalar(theta_deg: float, family: str, params: list[float] | tuple[float, ...] | np.ndarray) -> float:
    p = np.asarray(params, dtype=float).reshape(-1)
    x = float(theta_deg)
    if family == "const":
        return float(p[0]) if p.size >= 1 else 1.0
    if family == "quad_zero":
        return float(p[0]) * (1.0 - (x / 36.0) ** 2) if p.size >= 1 else 0.0
    if family == "quad_offset":
        if p.size < 2:
            return 1.0
        return float(p[0]) * (1.0 - (x / 36.0) ** 2) + float(p[1])
    raise ValueError(f"unknown sag C family: {family}")


def segment_errors_from_model(
    sag_model: dict[str, Any] | None,
    *,
    seg_index: int,
    count: int,
    theta1: float = 0.0,
    theta2: float = 0.0,
) -> np.ndarray:
    model = dict(sag_model or {})
    n = max(int(count), 0)
    if n <= 0:
        return np.zeros((0,), dtype=float)
    mode = str(model.get("model_type", "") or "").strip().lower()
    if mode == "func_finder_refined_v1":
        seg_tag = "1" if int(seg_index) == 1 else "2"
        c_family = str(model.get(f"c{seg_tag}_family", "const") or "const")
        c_params = np.asarray(model.get(f"c{seg_tag}_params", [1.0]), dtype=float).reshape(-1)
        a = np.asarray(model.get(f"a{seg_tag}", []), dtype=float).reshape(-1)
        b = np.asarray(model.get(f"b{seg_tag}_coeffs", []), dtype=float)
        if a.shape[0] != n:
            raise ValueError(f"refined sag model a{seg_tag} length mismatch: expected {n}, got {a.shape[0]}")
        if b.shape != (n, 3):
            raise ValueError(f"refined sag model b{seg_tag}_coeffs must have shape ({n}, 3)")
        c_theta = float(theta1) if int(seg_index) == 1 else float(theta2)
        c = _eval_c_family_scalar(c_theta, c_family, c_params)
        features = np.asarray([float(theta1), float(theta2), float(theta1) * float(theta2)], dtype=float)
        return float(c) * (a + b @ features)
    if any(k in model for k in ("seg1_distribution", "seg1_amplitude", "seg2_distribution", "seg2_amplitude")):
        if int(seg_index) == 1:
            return segment_errors_deg(
                str(model.get("seg1_distribution", "") or ""),
                str(model.get("seg1_amplitude", "") or ""),
                n,
                theta1=theta1,
                theta2=theta2,
            )
        return segment_errors_deg(
            str(model.get("seg2_distribution", "") or ""),
            str(model.get("seg2_amplitude", "") or ""),
            n,
            theta1=theta1,
            theta2=theta2,
        )
    return np.zeros((n,), dtype=float)
