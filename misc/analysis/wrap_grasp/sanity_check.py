#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Analytic circular-arc sanity check for the wrap-grasp sweep.

Compares the numeric coarse-to-fine sweep (wrap_sweep.py) against closed-form
circular-arc-approximation formulas, using ONLY values already extracted into
geometry_report.json (h pitch, per-node bend limit, node capsule radius).

Formulas (as specified for this check -- a single-segment, constant-curvature
approximation; the numeric sweep uses BOTH segments plus self-collision, which
this does not model, so agreement is expected only to within an order of
magnitude, not exactly):
    R(alpha)      = h / (2*sin(alpha/2))               curvature radius for per-node angle alpha
    inner envelope(alpha) = R(alpha) * cos(alpha/2)    concave-side envelope radius
    min wrappable radius ~= R(alpha_max)*cos(alpha_max/2) - d_inner
    alpha(r)      = 2*asin(h / (2*(r + d_inner)))      per-node angle needed for object radius r

Run with:
    python3 misc/analysis/wrap_grasp/sanity_check.py --geometry-report geometry_report.json
"""

from __future__ import annotations

import argparse
import json
import math
from typing import Any, Optional


def analytic_curves(h_m: float, alpha_max_deg: float, d_inner_m: float, radii_mm: list[float]) -> dict[str, Any]:
    alpha_max_rad = math.radians(alpha_max_deg)
    r_min_curvature_m = h_m / (2.0 * math.sin(alpha_max_rad / 2.0))
    inner_envelope_at_alpha_max_m = r_min_curvature_m * math.cos(alpha_max_rad / 2.0)
    min_wrappable_radius_m = inner_envelope_at_alpha_max_m - d_inner_m

    alpha_required_deg = []
    for r_mm in radii_mm:
        r_m = r_mm / 1000.0
        arg = h_m / (2.0 * (r_m + d_inner_m))
        if abs(arg) > 1.0:
            alpha_required_deg.append(None)  # no real solution: h alone (at alpha->0) already exceeds 2(r+d_inner)
        else:
            alpha_required_deg.append(math.degrees(2.0 * math.asin(arg)))

    return {
        "inputs": {"h_m": h_m, "alpha_max_deg": alpha_max_deg, "d_inner_m": d_inner_m},
        "R_min_curvature_m": r_min_curvature_m,
        "inner_envelope_at_alpha_max_m": inner_envelope_at_alpha_max_m,
        "min_wrappable_radius_m": min_wrappable_radius_m,
        "min_wrappable_diameter_mm": min_wrappable_radius_m * 2000.0,
        "alpha_required_deg_by_radius_mm": dict(zip(radii_mm, alpha_required_deg)),
    }


def load_geometry(path: str) -> tuple[float, float, float]:
    with open(path, "r", encoding="utf-8") as f:
        report = json.load(f)
    a = report["A_arc_geometry"]
    h_m = a["h_pitch_m"]["mean_m"]
    alpha_max_deg = a["alpha_max_deg_per_node"]
    b = report["B_collision_geometry"]["link_capsules_backbone"]
    node0 = b.get("node0")
    if node0 is None or node0.get("value") is None and "radius_m" not in node0:
        raise RuntimeError("node0 capsule radius not found in geometry_report.json -- run extract_geometry.py first")
    d_inner_m = node0["radius_m"]
    return h_m, alpha_max_deg, d_inner_m


def compare_to_sweep(analytic: dict[str, Any], strategy_map_path: Optional[str]) -> dict[str, Any]:
    if strategy_map_path is None:
        return {"note": "no --strategy-map given; skipping numeric comparison"}
    with open(strategy_map_path, "r", encoding="utf-8") as f:
        strategy = json.load(f)
    numeric_caged_range = strategy.get("caged_diameter_range_mm")
    analytic_min_mm = analytic["min_wrappable_diameter_mm"]
    return {
        "analytic_min_wrappable_diameter_mm": analytic_min_mm,
        "numeric_caged_diameter_range_mm": numeric_caged_range,
        "ratio_numeric_min_over_analytic": (
            (numeric_caged_range[0] / analytic_min_mm) if numeric_caged_range and analytic_min_mm else None
        ),
        "interpretation": (
            "The analytic formula models ONE segment forming a constant-curvature arc in "
            "isolation, with the object centered exactly at that arc's own center of curvature. "
            "The numeric sweep instead grid-searches the real two-segment geometry with the "
            "actual base assembly (plate/housing/gripper) present, gated by real self-collision "
            "checks and a stricter 'no escape gate' (G<2r) requirement rather than pure tangency. "
            "Agreement within roughly the same order of magnitude is the expected sanity-check "
            "outcome; an exact match is not, and a large (>~3x) or inverted gap should be treated "
            "as a signal to re-check the numeric pipeline, not as proof the analytic model is right."
        ),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--geometry-report", default="geometry_report.json")
    ap.add_argument("--strategy-map", default=None, help="optional strategy_map.json to compare against")
    ap.add_argument("--radii-mm", type=float, nargs="+", default=[10, 20, 27.5, 37.5, 62.5, 87.5, 125.0])
    ap.add_argument("--out", default="sanity_check.json")
    args = ap.parse_args()

    h_m, alpha_max_deg, d_inner_m = load_geometry(args.geometry_report)
    analytic = analytic_curves(h_m, alpha_max_deg, d_inner_m, args.radii_mm)
    comparison = compare_to_sweep(analytic, args.strategy_map)

    result = {"analytic": analytic, "comparison_to_numeric_sweep": comparison}
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)

    print(f"R_min (curvature radius at alpha_max) = {analytic['R_min_curvature_m']*1000:.2f} mm")
    print(f"inner envelope at alpha_max            = {analytic['inner_envelope_at_alpha_max_m']*1000:.2f} mm")
    print(f"analytic min wrappable diameter         = {analytic['min_wrappable_diameter_mm']:.2f} mm")
    if "numeric_caged_diameter_range_mm" in comparison:
        print(f"numeric caged diameter range             = {comparison['numeric_caged_diameter_range_mm']} mm")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
