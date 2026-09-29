#!/usr/bin/env python3
"""Summarize Sim GO2 contact/slip metrics into Markdown and JSON reports."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path
from typing import Any


LEGS = ("FL", "FR", "RL", "RR")
RUN_ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")


def _number(value: Any) -> float | None:
    if value is None or str(value).strip() == "":
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _stats(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "mean": None, "p95": None, "max": None}
    return {
        "count": len(values),
        "mean": sum(values) / len(values),
        "p95": _percentile(values, 0.95),
        "max": max(values),
    }


def _stance(value: Any) -> bool:
    return str(value or "").strip().lower() in {"1", "true", "yes", "on"}


def summarize_contact(
    rows: list[dict[str, str]],
    *,
    slip_threshold_mps: float = 0.02,
    high_slip_threshold_mps: float = 0.05,
) -> dict[str, Any]:
    """Summarize stance slip, Coulomb utilization, GRF error and leg coverage."""

    if slip_threshold_mps < 0 or high_slip_threshold_mps < slip_threshold_mps:
        raise ValueError("slip thresholds must satisfy 0 <= slip <= high-slip")
    stance_rows = [row for row in rows if _stance(row.get("stance"))]
    slip_values = [
        number
        for row in stance_rows
        if (number := _number(row.get("slip_speed_mps"))) is not None
    ]
    friction_values = [
        number
        for row in stance_rows
        if (number := _number(row.get("friction_ratio"))) is not None
    ]
    grf_errors = [
        number
        for row in stance_rows
        if (number := _number(row.get("grf_error_norm"))) is not None
    ]
    times = [
        number
        for row in rows
        if (number := _number(row.get("sim_time_s"))) is not None
    ]

    per_leg: dict[str, Any] = {}
    for leg in LEGS:
        leg_stance = [row for row in stance_rows if row.get("leg", "").upper() == leg]
        leg_slip = [
            number
            for row in leg_stance
            if (number := _number(row.get("slip_speed_mps"))) is not None
        ]
        leg_friction = [
            number
            for row in leg_stance
            if (number := _number(row.get("friction_ratio"))) is not None
        ]
        leg_grf = [
            number
            for row in leg_stance
            if (number := _number(row.get("grf_error_norm"))) is not None
        ]
        leg_distance = [
            number
            for row in rows
            if row.get("leg", "").upper() == leg
            and (number := _number(row.get("slip_distance_m"))) is not None
        ]
        per_leg[leg] = {
            "stance_samples": len(leg_stance),
            "slip_speed_mps": _stats(leg_slip),
            "slip_distance_m": max(leg_distance) if leg_distance else None,
            "slip_over_threshold_ratio": (
                sum(value > slip_threshold_mps for value in leg_slip) / len(leg_slip)
                if leg_slip else None
            ),
            "high_slip_ratio": (
                sum(value > high_slip_threshold_mps for value in leg_slip) / len(leg_slip)
                if leg_slip else None
            ),
            "friction_ratio": _stats(leg_friction),
            "friction_over_one_ratio": (
                sum(value > 1.0 for value in leg_friction) / len(leg_friction)
                if leg_friction else None
            ),
            "grf_error_norm": _stats(leg_grf),
        }

    slip_over = sum(value > slip_threshold_mps for value in slip_values)
    high_slip = sum(value > high_slip_threshold_mps for value in slip_values)
    friction_over = sum(value > 1.0 for value in friction_values)
    return {
        "row_count": len(rows),
        "stance_sample_count": len(stance_rows),
        "duration_s": (max(times) - min(times)) if times else None,
        "slip_threshold_mps": slip_threshold_mps,
        "high_slip_threshold_mps": high_slip_threshold_mps,
        "stance_slip_speed_mps": _stats(slip_values),
        "stance_slip_over_threshold_ratio": slip_over / len(slip_values) if slip_values else None,
        "stance_high_slip_ratio": high_slip / len(slip_values) if slip_values else None,
        "stance_friction_ratio": _stats(friction_values),
        "stance_friction_over_one_ratio": friction_over / len(friction_values) if friction_values else None,
        "stance_grf_error_norm": _stats(grf_errors),
        "per_leg": per_leg,
    }


def _fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "—"
    return f"{float(value):.{digits}f}"


def render_markdown(run_id: str, summary: dict[str, Any]) -> str:
    """Render a concise operator-readable interpretation and leg table."""

    slip = summary["stance_slip_speed_mps"]
    friction = summary["stance_friction_ratio"]
    grf = summary["stance_grf_error_norm"]
    rows = [
        f"# MPC contact/slip report: `{run_id}`",
        "",
        f"- Duration: {_fmt(summary['duration_s'])} s",
        f"- Contact samples: {summary['row_count']} total, {summary['stance_sample_count']} stance",
        f"- Stance slip speed: mean {_fmt(slip['mean'])}, p95 {_fmt(slip['p95'])}, max {_fmt(slip['max'])} m/s",
        f"- Stance samples above {summary['slip_threshold_mps']:.3f} m/s: {_fmt(100 * summary['stance_slip_over_threshold_ratio'], 1) if summary['stance_slip_over_threshold_ratio'] is not None else '—'}%",
        f"- Stance samples above {summary['high_slip_threshold_mps']:.3f} m/s: {_fmt(100 * summary['stance_high_slip_ratio'], 1) if summary['stance_high_slip_ratio'] is not None else '—'}%",
        f"- Friction utilization (desired tangential force / physical Coulomb limit): mean {_fmt(friction['mean'])}, p95 {_fmt(friction['p95'])}, max {_fmt(friction['max'])}",
        f"- Stance samples with friction ratio > 1: {_fmt(100 * summary['stance_friction_over_one_ratio'], 1) if summary['stance_friction_over_one_ratio'] is not None else '—'}%",
        f"- Desired vs actual GRF error norm: mean {_fmt(grf['mean'])}, p95 {_fmt(grf['p95'])}, max {_fmt(grf['max'])} N",
        "",
        "| Leg | Stance samples | Slip mean / p95 / max (m/s) | Max cumulative slip (m) | Friction p95 / max | GRF error p95 (N) |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for leg in LEGS:
        item = summary["per_leg"][leg]
        leg_slip = item["slip_speed_mps"]
        leg_friction = item["friction_ratio"]
        leg_grf = item["grf_error_norm"]
        rows.append(
            f"| {leg} | {item['stance_samples']} | "
            f"{_fmt(leg_slip['mean'])} / {_fmt(leg_slip['p95'])} / {_fmt(leg_slip['max'])} | "
            f"{_fmt(item['slip_distance_m'])} | "
            f"{_fmt(leg_friction['p95'])} / {_fmt(leg_friction['max'])} | "
            f"{_fmt(leg_grf['p95'])} |"
        )
    rows.extend(
        [
            "",
            "## Reading the signals",
            "",
            "- Stance is the commanded gait phase, not verified ground contact. Check measured normal force before interpreting slip.",
            "- Foot velocity is an average between diagnostic positions; the first sample is unavailable. Phase transitions can affect this estimate.",
            "- Friction ratio above 1 means the MPC desired tangential force exceeds the configured physical Coulomb limit.",
            "- High GRF error with low measured slip points toward contact-force realization or collision-manifold mismatch.",
            "- High stance slip with friction ratio below 1 and small GRF error points toward foot-point/Jacobian or stance-feedback error.",
            "- Compare runs with the same gait, command, duration, payload pose, and simulator settings. This report is diagnostic evidence, not a pass/fail verdict.",
            "",
        ]
    )
    return "\n".join(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_id", help="ELESIM_RUN_ID used for the Sim run")
    parser.add_argument("--log-dir", default="logs/walking_baseline")
    parser.add_argument("--slip-threshold-mps", type=float, default=0.02)
    parser.add_argument("--high-slip-threshold-mps", type=float, default=0.05)
    args = parser.parse_args()
    if not RUN_ID_PATTERN.fullmatch(args.run_id):
        parser.error("run_id must contain only letters, digits, '.', '_' or '-' (max 128 chars)")
    log_dir = Path(args.log_dir)
    contact_path = log_dir / f"{args.run_id}_contact.csv"
    if not contact_path.is_file():
        raise SystemExit(f"contact metrics not found: {contact_path}")
    with contact_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit(f"contact metrics are empty: {contact_path}")
    summary = summarize_contact(
        rows,
        slip_threshold_mps=args.slip_threshold_mps,
        high_slip_threshold_mps=args.high_slip_threshold_mps,
    )
    summary["run_id"] = args.run_id
    summary["source"] = str(contact_path)
    json_path = log_dir / f"{args.run_id}_contact_report.json"
    markdown_path = log_dir / f"{args.run_id}_contact_report.md"
    json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    markdown_path.write_text(render_markdown(args.run_id, summary), encoding="utf-8")
    print(f"Markdown report: {markdown_path}")
    print(f"JSON report:     {json_path}")


if __name__ == "__main__":
    main()
