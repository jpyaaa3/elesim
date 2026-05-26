#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import csv
import datetime as dt
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Optional

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.config_loader import load_app_config_from_ini
from engine.protocol import ControlU, control_u_to_sim_q


DEFAULT_CONFIG_PATH = str(ROOT / "config.ini")
FIRST_SEGMENT_LENGTH_MM = 19.0
SEGMENT_LENGTH_MM = 50.0
N_SEG_PER_ARM = 5
KST = dt.timezone(dt.timedelta(hours=9))


@dataclass(frozen=True)
class Sample:
    number: int
    roll_raw_deg: float
    theta1_raw_deg: float
    theta2_raw_deg: float
    theta1_bend_deg: float
    theta2_bend_deg: float
    first_vector_xy: tuple[float, float]
    measured_angles_deg: np.ndarray


@dataclass(frozen=True)
class RoughGuessResult:
    a1: np.ndarray
    a2: np.ndarray
    b1_coeffs: np.ndarray
    b2_coeffs: np.ndarray
    c1_samples: np.ndarray
    c2_samples: np.ndarray
    score: float


@dataclass(frozen=True)
class RefinedResult:
    c1_family: str
    c2_family: str
    c1_params: np.ndarray
    c2_params: np.ndarray
    a1: np.ndarray
    a2: np.ndarray
    b1_coeffs: np.ndarray
    b2_coeffs: np.ndarray
    score: float


def _refined_to_sag_model_json(refined: RefinedResult) -> dict[str, Any]:
    return {
        "c1_family": str(refined.c1_family),
        "c2_family": str(refined.c2_family),
        "c1_params": np.asarray(refined.c1_params, dtype=float).reshape(-1).tolist(),
        "c2_params": np.asarray(refined.c2_params, dtype=float).reshape(-1).tolist(),
        "a1": np.asarray(refined.a1, dtype=float).reshape(-1).tolist(),
        "a2": np.asarray(refined.a2, dtype=float).reshape(-1).tolist(),
        "b1_coeffs": np.asarray(refined.b1_coeffs, dtype=float).tolist(),
        "b2_coeffs": np.asarray(refined.b2_coeffs, dtype=float).tolist(),
    }


def _export_refined_sag_model(refined: RefinedResult, out_path: str) -> str:
    out_file = Path(out_path)
    out_file.parent.mkdir(parents=True, exist_ok=True)
    payload = _refined_to_sag_model_json(refined)
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    out_file.write_text(text + "\n", encoding="utf-8")
    return str(out_file)


def _normalize_vector(vx: float, vy: float, *, length: float) -> tuple[float, float]:
    norm = math.hypot(vx, vy)
    if norm < 1e-9:
        raise ValueError("first segment vector is degenerate")
    scale = float(length) / norm
    return (vx * scale, vy * scale)


def _rotate_deg(vx: float, vy: float, angle_deg: float) -> tuple[float, float]:
    rad = math.radians(angle_deg)
    c = math.cos(rad)
    s = math.sin(rad)
    return (c * vx - s * vy, s * vx + c * vy)


def _reconstruct_chain_from_angles(
    angle_deg_list: list[float] | np.ndarray,
    first_vector_xy: tuple[float, float],
    *,
    first_segment_length_mm: float = FIRST_SEGMENT_LENGTH_MM,
    segment_length_mm: float = SEGMENT_LENGTH_MM,
) -> np.ndarray:
    vx, vy = _normalize_vector(first_vector_xy[0], first_vector_xy[1], length=first_segment_length_mm)
    pts: list[tuple[float, float]] = [(0.0, 0.0)]
    x, y = 0.0, 0.0
    x += vx
    y += vy
    pts.append((x, y))
    cur_vx, cur_vy = vx, vy
    for angle_deg in np.asarray(angle_deg_list, dtype=float):
        cur_vx, cur_vy = _rotate_deg(cur_vx, cur_vy, float(angle_deg))
        cur_vx, cur_vy = _normalize_vector(cur_vx, cur_vy, length=segment_length_mm)
        x += cur_vx
        y += cur_vy
        pts.append((x, y))
    return np.asarray(pts, dtype=float)


def _load_csv_rows(csv_path: str) -> list[dict[str, Any]]:
    with open(csv_path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _parse_float(row: dict[str, Any], key: str) -> float:
    try:
        return float(str(row[key]).strip())
    except Exception as exc:
        raise ValueError(f"invalid float field '{key}'") from exc


def _read_measured_angles_deg(row: dict[str, Any]) -> np.ndarray:
    vals: list[float] = []
    i = 0
    while True:
        key = f"angle_deg_{i}"
        if key not in row:
            break
        vals.append(_parse_float(row, key))
        i += 1
    if len(vals) != 10:
        raise ValueError(f"expected 10 measured angles, got {len(vals)}")
    return np.asarray(vals, dtype=float)


def _read_first_vector(row: dict[str, Any]) -> tuple[float, float]:
    return (_parse_float(row, "2_x"), _parse_float(row, "2_y"))


def _map_raw_to_bend_deg(roll_raw_deg: float, theta1_raw_deg: float, theta2_raw_deg: float, *, config_path: str) -> tuple[float, float]:
    cfg = load_app_config_from_ini(str(config_path))
    q = control_u_to_sim_q(
        ControlU(
            u_roll=float(roll_raw_deg),
            u_s1=float(theta1_raw_deg),
            u_s2=float(theta2_raw_deg),
        ),
        cfg.mapping_config,
    )
    return (math.degrees(float(q.theta1_rad)), math.degrees(float(q.theta2_rad)))


def load_samples(csv_path: str, *, config_path: str) -> list[Sample]:
    rows = _load_csv_rows(csv_path)
    samples: list[Sample] = []
    for row in rows:
        roll_raw = _parse_float(row, "roll_deg")
        theta1_raw = _parse_float(row, "theta1_deg")
        theta2_raw = _parse_float(row, "theta2_deg")
        theta1_bend, theta2_bend = _map_raw_to_bend_deg(
            roll_raw,
            theta1_raw,
            theta2_raw,
            config_path=config_path,
        )
        samples.append(
            Sample(
                number=int(_parse_float(row, "number")),
                roll_raw_deg=roll_raw,
                theta1_raw_deg=theta1_raw,
                theta2_raw_deg=theta2_raw,
                theta1_bend_deg=theta1_bend,
                theta2_bend_deg=theta2_bend,
                first_vector_xy=_read_first_vector(row),
                measured_angles_deg=_read_measured_angles_deg(row),
            )
        )
    return samples


def _c_const(theta_deg: np.ndarray, p: np.ndarray) -> np.ndarray:
    x = np.asarray(theta_deg, dtype=float)
    return np.full_like(x, float(p[0]), dtype=float)


def _c_quad_zero(theta_deg: np.ndarray, p: np.ndarray) -> np.ndarray:
    x = np.asarray(theta_deg, dtype=float)
    return float(p[0]) * (1.0 - (x / 36.0) ** 2)


def _c_quad_offset(theta_deg: np.ndarray, p: np.ndarray) -> np.ndarray:
    x = np.asarray(theta_deg, dtype=float)
    return float(p[0]) * (1.0 - (x / 36.0) ** 2) + float(p[1])


C_FAMILIES: dict[str, Callable[[np.ndarray, np.ndarray], np.ndarray]] = {
    "const": _c_const,
    "quad_zero": _c_quad_zero,
    "quad_offset": _c_quad_offset,
}


def _format_c_expr(var_name: str, family: str, params: np.ndarray) -> str:
    p = np.asarray(params, dtype=float).reshape(-1)
    if family == "const":
        return f"{p[0]:.4f}"
    if family == "quad_zero":
        return f"{p[0]:.4f}\\left(1-\\left(\\frac{{{var_name}}}{{36}}\\right)^2\\right)"
    if family == "quad_offset":
        return f"{p[0]:.4f}\\left(1-\\left(\\frac{{{var_name}}}{{36}}\\right)^2\\right)+{p[1]:.4f}"
    return f"{family}{np.round(p, 4).tolist()}"


def _format_b_coeff_math_lines(name: str, coeffs: np.ndarray) -> list[str]:
    coeffs = np.asarray(coeffs, dtype=float)
    lines: list[str] = []
    for idx, row in enumerate(coeffs):
        lines.append(
            rf"${name}_{{{idx}}}(\theta_1,\theta_2)="
            rf"{row[0]:.3f}\theta_1+{row[1]:.3f}\theta_2+{row[2]:.3f}\theta_1\theta_2$"
        )
    return lines


def _pack_math_lines(lines: list[str], *, per_line: int = 2) -> list[str]:
    packed: list[str] = []
    for start in range(0, len(lines), per_line):
        chunk = lines[start : start + per_line]
        if len(chunk) == 1:
            packed.append(chunk[0])
            continue
        merged = r"$\qquad$".join(s.strip("$") for s in chunk)
        packed.append("$" + merged + "$")
    return packed


def _format_b_coeff_lines(name: str, coeffs: np.ndarray) -> str:
    coeffs = np.asarray(coeffs, dtype=float)
    parts: list[str] = []
    for idx, row in enumerate(coeffs):
        parts.append(
            f"{name}[{idx}] = {row[0]:.3f}*theta1 + {row[1]:.3f}*theta2 + {row[2]:.3f}*theta1*theta2"
        )
    return " | ".join(parts)


def _segment_error_matrices(samples: list[Sample]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    theta1 = np.asarray([s.theta1_bend_deg for s in samples], dtype=float)
    theta2 = np.asarray([s.theta2_bend_deg for s in samples], dtype=float)
    seg1_y = np.asarray([s.measured_angles_deg[:N_SEG_PER_ARM] for s in samples], dtype=float)
    seg2_y = np.asarray([s.measured_angles_deg[N_SEG_PER_ARM:] for s in samples], dtype=float)
    err1 = seg1_y - theta1[:, None]
    err2 = seg2_y - theta2[:, None]
    return theta1, theta2, err1, err2


def _momentum_feature_matrix(theta1: np.ndarray, theta2: np.ndarray) -> np.ndarray:
    t1 = np.asarray(theta1, dtype=float).reshape(-1)
    t2 = np.asarray(theta2, dtype=float).reshape(-1)
    return np.column_stack([t1, t2, t1 * t2])


def _fit_ab_for_segment(theta1: np.ndarray, theta2: np.ndarray, err_mat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    x = _momentum_feature_matrix(theta1, theta2)
    design = np.column_stack([np.ones((len(x),), dtype=float), x])
    coeffs, *_ = np.linalg.lstsq(design, np.asarray(err_mat, dtype=float), rcond=None)
    a = np.asarray(coeffs[0], dtype=float)
    b = np.asarray(coeffs[1:].T, dtype=float)
    return a, b


def _predict_segment_from_ab(
    theta1: np.ndarray | float,
    theta2: np.ndarray | float,
    a: np.ndarray,
    b_coeffs: np.ndarray,
) -> np.ndarray:
    t1 = np.asarray(theta1, dtype=float)
    t2 = np.asarray(theta2, dtype=float)
    x = np.stack([t1, t2, t1 * t2], axis=-1)
    return np.asarray(a, dtype=float) + np.tensordot(x, np.asarray(b_coeffs, dtype=float), axes=([-1], [1]))


def _estimate_projection_scales(err_mat: np.ndarray, pred_mat: np.ndarray) -> np.ndarray:
    scales: list[float] = []
    for row_err, row_pred in zip(np.asarray(err_mat, dtype=float), np.asarray(pred_mat, dtype=float)):
        denom = float(np.dot(row_pred, row_pred))
        if denom < 1e-9:
            scales.append(1.0)
        else:
            scales.append(float(np.dot(row_err, row_pred)) / denom)
    return np.asarray(scales, dtype=float)


def _rough_guess(samples: list[Sample]) -> RoughGuessResult:
    theta1, theta2, err1, err2 = _segment_error_matrices(samples)
    a1, b1_coeffs = _fit_ab_for_segment(theta1, theta2, err1)
    a2, b2_coeffs = _fit_ab_for_segment(theta1, theta2, err2)
    pred1 = _predict_segment_from_ab(theta1, theta2, a1, b1_coeffs)
    pred2 = _predict_segment_from_ab(theta1, theta2, a2, b2_coeffs)
    c1_samples = _estimate_projection_scales(err1, pred1)
    c2_samples = _estimate_projection_scales(err2, pred2)
    recon1 = c1_samples[:, None] * pred1
    recon2 = c2_samples[:, None] * pred2
    score = float(np.sum((err1 - recon1) ** 2) + np.sum((err2 - recon2) ** 2))
    return RoughGuessResult(
        a1=np.asarray(a1, dtype=float),
        a2=np.asarray(a2, dtype=float),
        b1_coeffs=np.asarray(b1_coeffs, dtype=float),
        b2_coeffs=np.asarray(b2_coeffs, dtype=float),
        c1_samples=np.asarray(c1_samples, dtype=float),
        c2_samples=np.asarray(c2_samples, dtype=float),
        score=score,
    )


def _fit_c_family(theta: np.ndarray, c_samples: np.ndarray, family: str) -> np.ndarray:
    x = np.asarray(theta, dtype=float)
    y = np.asarray(c_samples, dtype=float)
    if family == "const":
        return np.asarray([float(np.mean(y))], dtype=float)
    if family == "quad_zero":
        basis = 1.0 - (x / 36.0) ** 2
        denom = float(np.dot(basis, basis))
        if denom < 1e-9:
            return np.asarray([1.0], dtype=float)
        a = float(np.dot(basis, y) / denom)
        return np.asarray([a], dtype=float)
    if family == "quad_offset":
        basis = 1.0 - (x / 36.0) ** 2
        mat = np.column_stack([basis, np.ones_like(basis)])
        coeffs, *_ = np.linalg.lstsq(mat, y, rcond=None)
        return np.asarray(coeffs, dtype=float)
    raise ValueError(f"unknown C family: {family}")


def _eval_c_family(theta: np.ndarray, family: str, params: np.ndarray) -> np.ndarray:
    return C_FAMILIES[family](np.asarray(theta, dtype=float), np.asarray(params, dtype=float))


def _refine_from_rough(samples: list[Sample], rough: RoughGuessResult) -> RefinedResult:
    theta1, theta2, err1, err2 = _segment_error_matrices(samples)
    best: Optional[RefinedResult] = None
    for fam1 in C_FAMILIES.keys():
        p1 = _fit_c_family(theta1, rough.c1_samples, fam1)
        c1 = _eval_c_family(theta1, fam1, p1)
        if np.any(np.abs(c1) < 1e-6):
            continue
        scaled1 = err1 / c1[:, None]
        for fam2 in C_FAMILIES.keys():
            p2 = _fit_c_family(theta2, rough.c2_samples, fam2)
            c2 = _eval_c_family(theta2, fam2, p2)
            if np.any(np.abs(c2) < 1e-6):
                continue
            scaled2 = err2 / c2[:, None]
            a1, b1_coeffs = _fit_ab_for_segment(theta1, theta2, scaled1)
            a2, b2_coeffs = _fit_ab_for_segment(theta1, theta2, scaled2)
            recon1 = _predict_segment_from_ab(theta1, theta2, a1, b1_coeffs)
            recon2 = _predict_segment_from_ab(theta1, theta2, a2, b2_coeffs)
            score = float(np.sum((scaled1 - recon1) ** 2) + np.sum((scaled2 - recon2) ** 2))
            cand = RefinedResult(
                c1_family=fam1,
                c2_family=fam2,
                c1_params=np.asarray(p1, dtype=float),
                c2_params=np.asarray(p2, dtype=float),
                a1=np.asarray(a1, dtype=float),
                a2=np.asarray(a2, dtype=float),
                b1_coeffs=np.asarray(b1_coeffs, dtype=float),
                b2_coeffs=np.asarray(b2_coeffs, dtype=float),
                score=score,
            )
            if best is None or cand.score < best.score:
                best = cand
    if best is None:
        raise RuntimeError("failed to refine C families")
    return best


def _fit_poly2(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    x = np.asarray(x, dtype=float).reshape(-1)
    y = np.asarray(y, dtype=float).reshape(-1)
    if len(x) < 3:
        raise ValueError("need at least 3 samples for quadratic fit")
    return np.polyfit(x, y, deg=2)


def _poly2_eval(coeffs: np.ndarray, x: np.ndarray) -> np.ndarray:
    return np.polyval(np.asarray(coeffs, dtype=float), np.asarray(x, dtype=float))


def _nodewise_fit_summary(samples: list[Sample]) -> dict[str, list[np.ndarray]]:
    seg1_x = np.asarray([s.theta1_bend_deg for s in samples], dtype=float)
    seg2_x = np.asarray([s.theta2_bend_deg for s in samples], dtype=float)
    seg1_y = np.asarray([s.measured_angles_deg[:N_SEG_PER_ARM] for s in samples], dtype=float)
    seg2_y = np.asarray([s.measured_angles_deg[N_SEG_PER_ARM:] for s in samples], dtype=float)
    seg1_err = seg1_y - seg1_x[:, None]
    seg2_err = seg2_y - seg2_x[:, None]
    return {
        "seg1_actual": [_fit_poly2(seg1_x, seg1_y[:, i]) for i in range(N_SEG_PER_ARM)],
        "seg2_actual": [_fit_poly2(seg2_x, seg2_y[:, i]) for i in range(N_SEG_PER_ARM)],
        "seg1_err": [_fit_poly2(seg1_x, seg1_err[:, i]) for i in range(N_SEG_PER_ARM)],
        "seg2_err": [_fit_poly2(seg2_x, seg2_err[:, i]) for i in range(N_SEG_PER_ARM)],
    }


def _nodewise_analysis(samples: list[Sample]) -> dict[str, list[np.ndarray]]:
    if len(samples) < 3:
        raise ValueError("need at least 3 samples for nodewise analysis")
    return _nodewise_fit_summary(samples)


def _predict_angles_from_refined(sample: Sample, refined: RefinedResult) -> np.ndarray:
    c1 = float(_eval_c_family(np.asarray([sample.theta1_bend_deg], dtype=float), refined.c1_family, refined.c1_params)[0])
    c2 = float(_eval_c_family(np.asarray([sample.theta2_bend_deg], dtype=float), refined.c2_family, refined.c2_params)[0])
    err1 = c1 * _predict_segment_from_ab(sample.theta1_bend_deg, sample.theta2_bend_deg, refined.a1, refined.b1_coeffs)
    err2 = c2 * _predict_segment_from_ab(sample.theta1_bend_deg, sample.theta2_bend_deg, refined.a2, refined.b2_coeffs)
    return np.concatenate(
        [
            np.asarray(sample.theta1_bend_deg + err1, dtype=float),
            np.asarray(sample.theta2_bend_deg + err2, dtype=float),
        ],
        axis=0,
    )


def _predict_angles_from_rough(sample_index: int, sample: Sample, rough: RoughGuessResult) -> np.ndarray:
    c1 = float(rough.c1_samples[sample_index])
    c2 = float(rough.c2_samples[sample_index])
    err1 = c1 * _predict_segment_from_ab(sample.theta1_bend_deg, sample.theta2_bend_deg, rough.a1, rough.b1_coeffs)
    err2 = c2 * _predict_segment_from_ab(sample.theta1_bend_deg, sample.theta2_bend_deg, rough.a2, rough.b2_coeffs)
    return np.concatenate(
        [
            np.asarray(sample.theta1_bend_deg + err1, dtype=float),
            np.asarray(sample.theta2_bend_deg + err2, dtype=float),
        ],
        axis=0,
    )


def _plot_results(samples: list[Sample], refined: RefinedResult, out_path: str) -> str:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise RuntimeError("matplotlib is required for func_finder plotting") from exc

    n = len(samples)
    cols = 2
    rows = max(1, math.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(8 * cols, 5 * rows))
    axes_arr = np.atleast_1d(axes).reshape(-1)

    for ax, sample in zip(axes_arr, samples):
        pred_angles = _predict_angles_from_refined(sample, refined)
        pred_pts = _reconstruct_chain_from_angles(pred_angles, sample.first_vector_xy)
        meas_pts = _reconstruct_chain_from_angles(sample.measured_angles_deg, sample.first_vector_xy)
        ax.plot(meas_pts[:, 0], meas_pts[:, 1], "o-", label="measured", linewidth=2)
        ax.plot(pred_pts[:, 0], pred_pts[:, 1], "s--", label="predicted", linewidth=2)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.3)
        ax.set_title(
            f"row {sample.number} | seg1={sample.theta1_bend_deg:.1f} deg | seg2={sample.theta2_bend_deg:.1f} deg"
        )
        ax.set_xlabel("x [mm]")
        ax.set_ylabel("y [mm]")

    for ax in axes_arr[n:]:
        ax.axis("off")
    handles, labels = axes_arr[0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc="upper right")
    c1_expr = _format_c_expr("theta1", refined.c1_family, refined.c1_params)
    c2_expr = _format_c_expr("theta2", refined.c2_family, refined.c2_params)
    fig.text(0.02, 0.985, r"$\mathrm{sag}_{\mathrm{seg1}}(i)=C_1(\theta_1)\left[A_1(i)+B_1(i,\theta_1,\theta_2)\right]$", ha="left", va="top", fontsize=12)
    fig.text(0.52, 0.985, r"$\mathrm{sag}_{\mathrm{seg2}}(i)=C_2(\theta_2)\left[A_2(i)+B_2(i,\theta_1,\theta_2)\right]$", ha="left", va="top", fontsize=12)
    fig.text(0.02, 0.955, rf"$C_1(\theta_1)={c1_expr}$", ha="left", va="top", fontsize=11)
    fig.text(0.52, 0.955, rf"$C_2(\theta_2)={c2_expr}$", ha="left", va="top", fontsize=11)
    fig.text(0.02, 0.928, rf"$A_1={np.round(refined.a1, 3).tolist()}$", ha="left", va="top", fontsize=10)
    fig.text(0.52, 0.928, rf"$A_2={np.round(refined.a2, 3).tolist()}$", ha="left", va="top", fontsize=10)
    y0 = 0.900
    for line in _format_b_coeff_math_lines("B1", refined.b1_coeffs):
        fig.text(0.02, y0, line, ha="left", va="top", fontsize=9)
        y0 -= 0.024
    y1 = 0.900
    for line in _format_b_coeff_math_lines("B2", refined.b2_coeffs):
        fig.text(0.52, y1, line, ha="left", va="top", fontsize=9)
        y1 -= 0.024
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.74))
    out_file = Path(out_path)
    out_file.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_file, dpi=180, bbox_inches="tight")
    plt.show()
    plt.close(fig)
    return str(out_file)


def _plot_rough_results(samples: list[Sample], rough: RoughGuessResult, out_path: str) -> str:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise RuntimeError("matplotlib is required for func_finder plotting") from exc

    n = len(samples)
    cols = 2
    rows = max(1, math.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(8 * cols, 5 * rows))
    axes_arr = np.atleast_1d(axes).reshape(-1)

    for idx, (ax, sample) in enumerate(zip(axes_arr, samples)):
        pred_angles = _predict_angles_from_rough(idx, sample, rough)
        pred_pts = _reconstruct_chain_from_angles(pred_angles, sample.first_vector_xy)
        meas_pts = _reconstruct_chain_from_angles(sample.measured_angles_deg, sample.first_vector_xy)
        ax.plot(meas_pts[:, 0], meas_pts[:, 1], "o-", label="measured", linewidth=2)
        ax.plot(pred_pts[:, 0], pred_pts[:, 1], "s--", label="rough", linewidth=2)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.3)
        ax.set_title(
            f"row {sample.number} | seg1={sample.theta1_bend_deg:.1f} deg | seg2={sample.theta2_bend_deg:.1f} deg"
        )
        ax.set_xlabel("x [mm]")
        ax.set_ylabel("y [mm]")

    for ax in axes_arr[n:]:
        ax.axis("off")
    handles, labels = axes_arr[0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc="upper right")
    fig.text(0.02, 0.985, r"$\mathrm{rough}_{\mathrm{seg1}}(i)=C_1[r]\left[A_1(i)+B_1(i,\theta_1,\theta_2)\right]$", ha="left", va="top", fontsize=12)
    fig.text(0.52, 0.985, r"$\mathrm{rough}_{\mathrm{seg2}}(i)=C_2[r]\left[A_2(i)+B_2(i,\theta_1,\theta_2)\right]$", ha="left", va="top", fontsize=12)
    fig.text(0.02, 0.955, rf"$A_1={np.round(rough.a1, 3).tolist()}$", ha="left", va="top", fontsize=10)
    fig.text(0.52, 0.955, rf"$A_2={np.round(rough.a2, 3).tolist()}$", ha="left", va="top", fontsize=10)
    fig.text(0.02, 0.928, rf"$\mathrm{{score}}={rough.score:.4f}$", ha="left", va="top", fontsize=10)
    y0 = 0.900
    for line in _format_b_coeff_math_lines("B1", rough.b1_coeffs):
        fig.text(0.02, y0, line, ha="left", va="top", fontsize=9)
        y0 -= 0.024
    y1 = 0.900
    for line in _format_b_coeff_math_lines("B2", rough.b2_coeffs):
        fig.text(0.52, y1, line, ha="left", va="top", fontsize=9)
        y1 -= 0.024
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.74))
    out_file = Path(out_path)
    out_file.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_file, dpi=180, bbox_inches="tight")
    plt.show()
    plt.close(fig)
    return str(out_file)


class FuncFinderGui:
    def __init__(self) -> None:
        import tkinter as tk

        self._tk = tk
        self.root = tk.Tk()
        self.root.title("func_finder")

        self.csv_path_var = tk.StringVar()
        self.config_path_var = tk.StringVar(value=DEFAULT_CONFIG_PATH)
        self.status_var = tk.StringVar(value="ready")

        self._samples: list[Sample] = []
        self._rough_guess_result: Optional[RoughGuessResult] = None
        self._refined_result: Optional[RefinedResult] = None
        self._build()

    def _build(self) -> None:
        tk = self._tk
        pad = {"padx": 6, "pady": 4}

        tk.Label(self.root, text="csv").grid(row=0, column=0, sticky="w", **pad)
        tk.Entry(self.root, textvariable=self.csv_path_var, width=60).grid(row=0, column=1, sticky="we", **pad)
        tk.Button(self.root, text="Browse", command=self._browse_csv).grid(row=0, column=2, **pad)

        tk.Label(self.root, text="config").grid(row=1, column=0, sticky="w", **pad)
        tk.Entry(self.root, textvariable=self.config_path_var, width=60).grid(row=1, column=1, sticky="we", **pad)
        tk.Button(self.root, text="Config", command=self._browse_config).grid(row=1, column=2, **pad)

        tk.Button(self.root, text="Load CSV", command=self._load_csv).grid(row=2, column=0, sticky="we", **pad)
        tk.Button(self.root, text="Rough Guess", command=self._plot_nodewise).grid(row=2, column=1, sticky="we", **pad)
        tk.Button(self.root, text="Refine", command=self._fit_all).grid(row=2, column=2, sticky="we", **pad)
        tk.Button(self.root, text="Visualize", command=self._plot_best).grid(row=3, column=0, columnspan=3, sticky="we", **pad)

        tk.Label(self.root, textvariable=self.status_var, anchor="w", justify="left").grid(
            row=4, column=0, columnspan=3, sticky="we", **pad
        )

        self.root.grid_columnconfigure(1, weight=1)

    def _browse_csv(self) -> None:
        from tkinter import filedialog

        path = filedialog.askopenfilename(
            title="Select CSV",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if path:
            self.csv_path_var.set(path)

    def _browse_config(self) -> None:
        from tkinter import filedialog

        path = filedialog.askopenfilename(
            title="Select config.ini",
            filetypes=[("INI files", "*.ini"), ("All files", "*.*")],
        )
        if path:
            self.config_path_var.set(path)

    def _load_csv(self) -> None:
        from tkinter import messagebox

        csv_path = self.csv_path_var.get().strip()
        if not csv_path:
            messagebox.showerror("func_finder", "csv path is empty")
            return
        try:
            self._samples = load_samples(csv_path, config_path=self.config_path_var.get().strip())
            self._rough_guess_result = None
            self._refined_result = None
            self.status_var.set(f"loaded samples: {len(self._samples)}")
        except Exception as exc:
            messagebox.showerror("func_finder", str(exc))

    def _fit_all(self) -> None:
        from tkinter import filedialog, messagebox

        if not self._samples:
            messagebox.showerror("func_finder", "no samples loaded")
            return
        try:
            if self._rough_guess_result is None:
                self._rough_guess_result = _rough_guess(self._samples)
            best = _refine_from_rough(self._samples, self._rough_guess_result)
            out_dir = ROOT / "assets"
            out_dir.mkdir(parents=True, exist_ok=True)
            stamp = dt.datetime.now(KST).strftime("%Y%m%d_%H%M%S")
            out_path = filedialog.asksaveasfilename(
                title="Export sag model JSON",
                initialdir=str(out_dir),
                initialfile=f"func_finder_refined_{stamp}.json",
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            )
            if not out_path:
                return
            self._refined_result = best
            saved_path = _export_refined_sag_model(best, out_path)
            self.status_var.set(
                f"refined best C1={best.c1_family}{np.round(best.c1_params, 4).tolist()} | "
                f"C2={best.c2_family}{np.round(best.c2_params, 4).tolist()}\n"
                f"A1={np.round(best.a1, 3).tolist()}\n"
                f"A2={np.round(best.a2, 3).tolist()}\n"
                f"B1 coeffs={np.round(best.b1_coeffs, 3).tolist()}\n"
                f"B2 coeffs={np.round(best.b2_coeffs, 3).tolist()}\n"
                f"score={best.score:.4f}\n"
                f"exported sag model: {saved_path}"
            )
        except Exception as exc:
            messagebox.showerror("func_finder", str(exc))

    def _plot_nodewise(self) -> None:
        from tkinter import messagebox

        if not self._samples:
            messagebox.showerror("func_finder", "no samples loaded")
            return
        try:
            summary = _nodewise_analysis(self._samples)
            rough = _rough_guess(self._samples)
            self._rough_guess_result = rough
            self._refined_result = None
            seg1_err = [np.round(c, 4).tolist() for c in summary["seg1_err"]]
            seg2_err = [np.round(c, 4).tolist() for c in summary["seg2_err"]]
            self.status_var.set(
                f"rough guess complete\n"
                f"A1={np.round(rough.a1, 3).tolist()}\n"
                f"A2={np.round(rough.a2, 3).tolist()}\n"
                f"B1 coeffs={np.round(rough.b1_coeffs, 3).tolist()}\n"
                f"B2 coeffs={np.round(rough.b2_coeffs, 3).tolist()}\n"
                f"C1 samples={np.round(rough.c1_samples, 3).tolist()}\n"
                f"C2 samples={np.round(rough.c2_samples, 3).tolist()}\n"
                f"rough score={rough.score:.4f}\n"
                f"seg1 err poly2 [a,b,c] by node: {seg1_err}\n"
                f"seg2 err poly2 [a,b,c] by node: {seg2_err}"
            )
        except Exception as exc:
            messagebox.showerror("func_finder", str(exc))

    def _plot_best(self) -> None:
        from tkinter import filedialog, messagebox

        if self._refined_result is None and self._rough_guess_result is None:
            messagebox.showerror("func_finder", "no rough/refined result")
            return
        try:
            out_dir = ROOT / "addons" / "func_finder" / "output"
            out_dir.mkdir(parents=True, exist_ok=True)
            stamp = dt.datetime.now(KST).strftime("%Y%m%d_%H%M%S")
            default_name = (
                f"func_finder_fit_{stamp}.png"
                if self._refined_result is not None
                else f"func_finder_rough_{stamp}.png"
            )
            out_path = filedialog.asksaveasfilename(
                title="Save visualization",
                initialdir=str(out_dir),
                initialfile=default_name,
                defaultextension=".png",
                filetypes=[("PNG files", "*.png"), ("All files", "*.*")],
            )
            if not out_path:
                return
            if self._refined_result is not None:
                out_path = _plot_results(self._samples, self._refined_result, out_path)
            else:
                out_path = _plot_rough_results(self._samples, self._rough_guess_result, out_path)
            self.status_var.set(f"{self.status_var.get()}\nplot saved: {out_path}")
        except Exception as exc:
            messagebox.showerror("func_finder", str(exc))

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    FuncFinderGui().run()


if __name__ == "__main__":
    main()
