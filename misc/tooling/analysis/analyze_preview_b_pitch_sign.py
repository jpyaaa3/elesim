#!/usr/bin/env python3
"""Compare preview b_pitch sign sweep runs (+0.05 vs -0.05)."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from misc.tooling.analysis.analyze_walking_metrics import summarize_run


def _load_meta(log_dir: Path, run_id: str) -> dict[str, Any]:
    path = log_dir / f"{run_id}_meta.json"
    if not path.is_file():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def _mean_preview_term_v(log_dir: Path, run_id: str) -> float | None:
    cam_path = log_dir / f"{run_id}_camera.csv"
    if not cam_path.is_file():
        return None
    lines = cam_path.read_text(encoding="utf-8").strip().splitlines()
    if len(lines) < 2:
        return None
    header = lines[0].split(",")
    if "preview_term_v" not in header:
        return None
    idx = header.index("preview_term_v")
    vals: list[float] = []
    for row in lines[1:]:
        cols = row.split(",")
        if idx >= len(cols):
            continue
        raw = cols[idx].strip()
        if not raw:
            continue
        try:
            vals.append(float(raw))
        except ValueError:
            continue
    if not vals:
        return None
    return float(sum(vals) / len(vals))


def _collect_runs(log_dir: Path, prefix: str) -> list[str]:
    pat = re.compile(rf"^{re.escape(prefix)}_.+_\d{{3}}_summary\.json$")
    return sorted(p.stem.replace("_summary", "") for p in log_dir.glob(f"{prefix}_*_summary.json") if pat.match(p.name))


def _aggregate_group(log_dir: Path, run_ids: list[str]) -> dict[str, Any]:
    if not run_ids:
        return {"run_ids": [], "count": 0}
    v_rms: list[float] = []
    used: list[float] = []
    fallback: list[float] = []
    term_v: list[float] = []
    for rid in run_ids:
        summary = summarize_run(rid, log_dir, write_plots=False)
        meta = _load_meta(log_dir, rid)
        v_rms.append(float(summary.get("v_rms", 0.0)))
        used.append(float(meta.get("preview_used_ratio", 0.0)))
        fallback.append(float(meta.get("preview_fallback_ratio", 0.0)))
        tv = _mean_preview_term_v(log_dir, rid)
        if tv is not None:
            term_v.append(tv)
    n = len(run_ids)
    return {
        "run_ids": run_ids,
        "count": n,
        "v_rms_mean": float(sum(v_rms) / n),
        "preview_used_ratio_mean": float(sum(used) / n),
        "preview_fallback_ratio_mean": float(sum(fallback) / n),
        "preview_term_v_mean": float(sum(term_v) / len(term_v)) if term_v else None,
    }


def compare_sign_sweep(log_dir: Path) -> dict[str, Any]:
    pos = _aggregate_group(log_dir, _collect_runs(log_dir, "exp_gaze_preview_bp05"))
    neg = _aggregate_group(log_dir, _collect_runs(log_dir, "exp_gaze_preview_bn05"))
    recommendation = ""
    if pos["count"] and neg["count"]:
        if pos["v_rms_mean"] <= neg["v_rms_mean"]:
            recommendation = "prefer gaze_preview_b_pitch=+0.05 (lower v_rms_mean)"
        else:
            recommendation = "prefer gaze_preview_b_pitch=-0.05 (lower v_rms_mean)"
    return {
        "log_dir": str(log_dir),
        "b_pitch_positive": pos,
        "b_pitch_negative": neg,
        "recommendation": recommendation,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="Analyze preview b_pitch sign sweep")
    ap.add_argument("--log-dir", default="logs/walking_baseline")
    ap.add_argument("--out", default=None, help="output json path (default: log_dir/preview_b_pitch_sign_compare.json)")
    args = ap.parse_args()
    log_dir = Path(args.log_dir)
    result = compare_sign_sweep(log_dir)
    out = Path(args.out) if args.out else log_dir / "preview_b_pitch_sign_compare.json"
    out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    if not result["b_pitch_positive"]["count"] or not result["b_pitch_negative"]["count"]:
        print(
            "\nNo complete sweep yet. Run misc/tooling/experiments/run_preview_b_pitch_sign.sh after enabling gaze_preview_enable=true.",
            file=sys.stderr,
        )


if __name__ == "__main__":
    main()
