#!/usr/bin/env python3
"""Merge walking/camera CSV logs and emit summary statistics."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any, Optional


def _read_csv(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _float_or_none(raw: Any) -> Optional[float]:
    if raw in ("", None):
        return None
    try:
        return float(raw)
    except (TypeError, ValueError):
        return None


def _time_column(row: dict[str, Any]) -> float:
    for key in ("wall_time_s", "time_s"):
        v = _float_or_none(row.get(key))
        if v is not None:
            return v
    return 0.0


def _rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return float(math.sqrt(sum(v * v for v in values) / len(values)))


def _corr(a: list[float], b: list[float]) -> float:
    n = min(len(a), len(b))
    if n < 3:
        return 0.0
    ma = sum(a[:n]) / n
    mb = sum(b[:n]) / n
    num = sum((a[i] - ma) * (b[i] - mb) for i in range(n))
    da = math.sqrt(sum((a[i] - ma) ** 2 for i in range(n)))
    db = math.sqrt(sum((b[i] - mb) ** 2 for i in range(n)))
    if da < 1e-12 or db < 1e-12:
        return 0.0
    return float(num / (da * db))


def _nearest_merge(walking: list[dict], camera: list[dict], *, max_dt: float = 0.05) -> list[dict]:
    if not walking or not camera:
        return []
    cam_times = [_time_column(r) for r in camera]
    out: list[dict] = []
    for w in walking:
        tw = _time_column(w)
        j = min(range(len(cam_times)), key=lambda i: abs(cam_times[i] - tw))
        if abs(cam_times[j] - tw) > max_dt:
            continue
        row = dict(w)
        row.update(camera[j])
        out.append(row)
    return out


def _lost_counts(camera: list[dict]) -> tuple[int, int]:
    frame = 0
    event = 0
    if camera:
        last = camera[-1]
        frame = int(float(last.get("target_lost_frame_count") or last.get("target_lost_count") or 0))
        event = int(float(last.get("target_lost_event_count") or 0))
    return frame, event


def summarize_run(
    run_id: str,
    log_dir: Path,
    *,
    write_merged: bool = False,
    write_plots: bool = True,
) -> dict[str, Any]:
    walking = _read_csv(log_dir / f"{run_id}_walking.csv")
    camera = _read_csv(log_dir / f"{run_id}_camera.csv")
    meta_path = log_dir / f"{run_id}_meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.is_file() else {}

    pitch = [math.degrees(float(r["base_pitch"])) for r in walking if r.get("base_pitch")]
    roll = [math.degrees(float(r["base_roll"])) for r in walking if r.get("base_roll")]
    u_err = [_float_or_none(r.get("u_err")) for r in camera]
    v_err = [_float_or_none(r.get("v_err")) for r in camera]
    u_vals = [v for v in u_err if v is not None]
    v_vals = [v for v in v_err if v is not None]
    visible = [int(float(r.get("target_visible") or 0)) for r in camera]
    vis_ratio = float(sum(visible) / max(1, len(visible))) if camera else 0.0
    lost_frame, lost_event = _lost_counts(camera)

    merged = _nearest_merge(walking, camera)
    if write_merged and merged:
        merged_path = log_dir / f"{run_id}_merged.csv"
        keys = sorted({k for row in merged for k in row.keys()})
        with open(merged_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(merged)

    pitch_m = [float(r["base_pitch"]) for r in merged if r.get("base_pitch") and _float_or_none(r.get("v_err")) is not None]
    roll_m = [float(r["base_roll"]) for r in merged if r.get("base_roll") and _float_or_none(r.get("u_err")) is not None]
    v_m = [_float_or_none(r.get("v_err")) for r in merged]
    u_m = [_float_or_none(r.get("u_err")) for r in merged]
    v_clean = [v for v in v_m if v is not None]
    u_clean = [v for v in u_m if v is not None]

    tau_max = max((float(r.get("tau_max_abs") or 0.0) for r in walking), default=0.0)
    sat_ratios = [float(r.get("tau_saturation_ratio") or 0.0) for r in walking]
    fall = any(int(float(r.get("fall_flag") or 0)) for r in walking)

    summary = {
        "run_id": run_id,
        "meta": meta,
        "pitch_rms": _rms(pitch),
        "roll_rms": _rms(roll),
        "pitch_rms_deg": _rms(pitch),
        "roll_rms_deg": _rms(roll),
        "max_abs_pitch": max((abs(float(r["base_pitch"])) for r in walking), default=0.0),
        "max_abs_roll": max((abs(float(r["base_roll"])) for r in walking), default=0.0),
        "tau_max_abs": tau_max,
        "tau_saturation_ratio": float(sum(sat_ratios) / max(1, len(sat_ratios))),
        "fall_detected": bool(fall),
        "visible_time_ratio": vis_ratio,
        "target_lost_frame_count": lost_frame,
        "target_lost_event_count": lost_event,
        "target_lost_count": lost_frame,
        "u_rms": _rms(u_vals),
        "v_rms": _rms(v_vals),
        "u_err_rms": _rms(u_vals),
        "v_err_rms": _rms(v_vals),
        "max_abs_u": max((abs(v) for v in u_vals), default=0.0),
        "max_abs_v": max((abs(v) for v in v_vals), default=0.0),
        "corr_pitch_v": _corr(pitch_m, v_clean),
        "corr_roll_u": _corr(roll_m, u_clean),
        "walking_rows": len(walking),
        "camera_rows": len(camera),
        "merged_rows": len(merged),
    }
    out_path = log_dir / f"{run_id}_summary.json"
    out_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    if write_plots:
        _maybe_plot(run_id, log_dir, walking, camera)
    return summary


def evaluate_pitch_trim(before: dict[str, Any], after: dict[str, Any]) -> dict[str, Any]:
    pitch_before = float(before.get("pitch_rms_deg", before.get("pitch_rms", 0.0)))
    pitch_after = float(after.get("pitch_rms_deg", after.get("pitch_rms", 0.0)))
    reduction = 0.0 if pitch_before < 1e-9 else (pitch_before - pitch_after) / pitch_before
    u_reg = float(after.get("u_rms", 0.0)) <= 1.2 * float(before.get("u_rms", 0.0) or 1e-9)
    v_reg = float(after.get("v_rms", 0.0)) <= 1.2 * float(before.get("v_rms", 0.0) or 1e-9)
    vis_ok = float(after.get("visible_time_ratio", 1.0)) >= float(before.get("visible_time_ratio", 0.0)) - 0.05
    lost_ok = int(after.get("target_lost_event_count", 0)) <= int(before.get("target_lost_event_count", 0)) + 1
    return {
        "pitch_rms_reduction_frac": float(reduction),
        "pitch_trim_pass_30pct": bool(reduction >= 0.30),
        "no_fall_regression": not bool(after.get("fall_detected")) or not bool(before.get("fall_detected")),
        "uv_rms_within_20pct": bool(u_reg and v_reg),
        "visibility_ok": bool(vis_ok),
        "lost_events_ok": bool(lost_ok),
        "overall_pass": bool(reduction >= 0.30 and vis_ok and lost_ok and not after.get("fall_detected")),
    }


def _maybe_plot(run_id: str, log_dir: Path, walking: list[dict], camera: list[dict]) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return
    plot_dir = log_dir / f"{run_id}_plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    if walking:
        t = [_time_column(r) for r in walking]
        pitch = [math.degrees(float(r["base_pitch"])) for r in walking]
        roll = [math.degrees(float(r["base_roll"])) for r in walking]
        plt.figure(figsize=(8, 3))
        plt.plot(t, pitch, label="pitch")
        plt.plot(t, roll, label="roll")
        plt.ylabel("deg")
        plt.xlabel("time [s]")
        plt.legend()
        plt.title(f"{run_id} base attitude")
        plt.tight_layout()
        plt.savefig(plot_dir / "pitch_roll.png")
        plt.close()
    if camera:
        t = [_time_column(r) for r in camera]
        u = [_float_or_none(r.get("u_err")) or 0.0 for r in camera]
        v = [_float_or_none(r.get("v_err")) or 0.0 for r in camera]
        vis = [int(float(r.get("target_visible") or 0)) for r in camera]
        plt.figure(figsize=(8, 3))
        plt.plot(t, u, label="u_err")
        plt.plot(t, v, label="v_err")
        plt.legend()
        plt.xlabel("time [s]")
        plt.title(f"{run_id} UV error")
        plt.tight_layout()
        plt.savefig(plot_dir / "uv_err.png")
        plt.close()
        plt.figure(figsize=(8, 1.5))
        plt.plot(t, vis)
        plt.ylim(-0.1, 1.1)
        plt.ylabel("visible")
        plt.xlabel("time [s]")
        plt.tight_layout()
        plt.savefig(plot_dir / "visibility.png")
        plt.close()


def compare_runs(run_ids: list[str], log_dir: Path) -> dict[str, Any]:
    summaries = {rid: summarize_run(rid, log_dir, write_plots=False) for rid in run_ids}
    table = []
    for rid, s in summaries.items():
        table.append(
            {
                "run_id": rid,
                "pitch_rms_deg": s["pitch_rms_deg"],
                "u_rms": s["u_rms"],
                "v_rms": s["v_rms"],
                "visible_time_ratio": s["visible_time_ratio"],
                "target_lost_event_count": s["target_lost_event_count"],
            }
        )
    return {"runs": summaries, "table": table}


def _find_latest_run_id(log_dir: Path) -> Optional[str]:
    metas = sorted(log_dir.glob("*_meta.json"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not metas:
        return None
    return metas[0].name.replace("_meta.json", "")


def main() -> None:
    ap = argparse.ArgumentParser(description="Analyze walking baseline CSV logs")
    ap.add_argument("run_id", nargs="?", default="")
    ap.add_argument("--log-dir", default="logs/walking_baseline")
    ap.add_argument("--latest", action="store_true")
    ap.add_argument("--compare", nargs="+", default=None, help="run ids to compare")
    ap.add_argument("--merged", action="store_true")
    ap.add_argument("--evaluate-trim", nargs=2, metavar=("BEFORE", "AFTER"))
    args = ap.parse_args()
    log_dir = Path(args.log_dir)

    if args.evaluate_trim:
        before = summarize_run(args.evaluate_trim[0], log_dir, write_plots=False)
        after = summarize_run(args.evaluate_trim[1], log_dir, write_plots=False)
        result = evaluate_pitch_trim(before, after)
        print(json.dumps(result, indent=2))
        return

    if args.compare:
        result = compare_runs(list(args.compare), log_dir)
        out = log_dir / "compare_summary.json"
        out.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, indent=2))
        return

    run_id = args.run_id
    if args.latest or not run_id:
        run_id = _find_latest_run_id(log_dir) or ""
    if not run_id:
        raise SystemExit("no run_id found")
    result = summarize_run(run_id, log_dir, write_merged=bool(args.merged))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
