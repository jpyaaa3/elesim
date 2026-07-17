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


def effective_visibility_flags(
    camera: list[dict[str, Any]],
    *,
    frozen_samples: int = 6,
    min_scale: float = 0.0,
    max_center_abs: float = 1.05,
) -> list[int]:
    """Reject logger 'visible' samples that are tracker ghosts (frozen UV / off-frame)."""
    flags: list[int] = []
    streak = 0
    prev_key: Optional[tuple[float, float, float]] = None
    frozen_active = False
    for row in camera:
        raw = str(row.get("target_visible") or "").strip().lower() in ("1", "true", "yes")
        scale = float(_float_or_none(row.get("bbox_scale")) or 0.0)
        u = _float_or_none(row.get("u_err"))
        v = _float_or_none(row.get("v_err"))
        key: Optional[tuple[float, float, float]] = None
        if u is not None and v is not None:
            key = (round(float(u), 5), round(float(v), 5), round(float(scale), 8))

        if key is not None and key == prev_key:
            streak += 1
        else:
            streak = 1
            prev_key = key
            frozen_active = False
        if streak >= int(max(2, frozen_samples)):
            frozen_active = True

        ok = bool(
            raw
            and u is not None
            and v is not None
            and scale >= float(min_scale)
            and abs(float(u)) <= float(max_center_abs)
            and abs(float(v)) <= float(max_center_abs)
            and not frozen_active
        )
        flags.append(int(ok))
    return flags


def _visibility_lost_counts(flags: list[int]) -> tuple[int, int]:
    """Return (invisible_frame_count, visible_to_invisible_events) from effective flags."""
    if not flags:
        return 0, 0
    lost_frames = sum(1 for f in flags if not int(f))
    events = sum(
        1 for i in range(1, len(flags)) if int(flags[i - 1]) and not int(flags[i])
    )
    return int(lost_frames), int(events)


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


def _mean(values: list[float]) -> float:
    if not values:
        return 0.0
    return float(sum(values) / len(values))


def _std(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    m = _mean(values)
    return float(math.sqrt(sum((v - m) ** 2 for v in values) / len(values)))


def _detrend_rms(values: list[float]) -> float:
    """RMS of fluctuations around the per-run mean (ignores steady UV bias)."""
    if not values:
        return 0.0
    m = _mean(values)
    return _rms([v - m for v in values])


def _camera_uv_errors(
    camera: list[dict],
    *,
    dedupe: bool = True,
    visibility_flags: Optional[list[int]] = None,
) -> tuple[list[float], list[float]]:
    u_vals: list[float] = []
    v_vals: list[float] = []
    prev: Optional[tuple[Any, ...]] = None
    for i, row in enumerate(camera):
        if visibility_flags is not None:
            if i >= len(visibility_flags) or not int(visibility_flags[i]):
                continue
        elif str(row.get("target_visible") or "").strip().lower() not in ("1", "true", "yes"):
            continue
        u = _float_or_none(row.get("u_err"))
        v = _float_or_none(row.get("v_err"))
        if u is None or v is None:
            continue
        key = (u, v, row.get("sim_time_s"))
        if dedupe and key == prev:
            continue
        prev = key
        u_vals.append(float(u))
        v_vals.append(float(v))
    return u_vals, v_vals


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


def _trim_logs_by_window(
    walking: list[dict],
    camera: list[dict],
    *,
    analysis_window_s: Optional[float],
) -> tuple[list[dict], list[dict], Optional[float]]:
    """Keep rows within [t0, t0 + window] using wall_time_s (first sample = t0)."""
    if analysis_window_s is None or float(analysis_window_s) <= 0.0:
        return walking, camera, None
    times = [_float_or_none(r.get("wall_time_s")) for r in walking + camera]
    times = [t for t in times if t is not None]
    if not times:
        return walking, camera, float(analysis_window_s)
    t0 = min(times)
    t_end = t0 + float(analysis_window_s)

    def _keep(row: dict) -> bool:
        t = _float_or_none(row.get("wall_time_s"))
        return t is not None and t0 <= t <= t_end

    return [r for r in walking if _keep(r)], [r for r in camera if _keep(r)], float(analysis_window_s)


def _sim_visibility_flags(walking: list[dict[str, Any]]) -> Optional[list[int]]:
    if not walking or "sim_target_in_frame" not in walking[0]:
        return None
    out: list[int] = []
    for row in walking:
        raw = row.get("sim_target_in_frame")
        if raw in ("", None):
            out.append(0)
        else:
            out.append(int(float(raw)))
    return out


def _sim_visibility_for_camera(
    camera: list[dict[str, Any]],
    walking: list[dict[str, Any]],
    *,
    max_dt: float = 0.08,
) -> Optional[list[int]]:
    walk_flags = _sim_visibility_flags(walking)
    if walk_flags is None:
        return None
    walk_times = [_time_column(r) for r in walking]
    if not walk_times:
        return None
    out: list[int] = []
    for row in camera:
        ts = _time_column(row)
        j = min(range(len(walk_times)), key=lambda i: abs(float(walk_times[i]) - ts))
        if abs(float(walk_times[j]) - ts) > float(max_dt):
            out.append(0)
        else:
            out.append(int(walk_flags[j]))
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
    analysis_window_s: Optional[float] = None,
) -> dict[str, Any]:
    walking = _read_csv(log_dir / f"{run_id}_walking.csv")
    camera = _read_csv(log_dir / f"{run_id}_camera.csv")
    walking, camera, win_s = _trim_logs_by_window(walking, camera, analysis_window_s=analysis_window_s)
    meta_path = log_dir / f"{run_id}_meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.is_file() else {}

    pitch = [math.degrees(float(r["base_pitch"])) for r in walking if r.get("base_pitch")]
    roll = [math.degrees(float(r["base_roll"])) for r in walking if r.get("base_roll")]
    u_err = [_float_or_none(r.get("u_err")) for r in camera]
    v_err = [_float_or_none(r.get("v_err")) for r in camera]
    sim_walk_flags = _sim_visibility_flags(walking)
    sim_cam_flags = _sim_visibility_for_camera(camera, walking)
    perception_flags = effective_visibility_flags(camera)

    if sim_walk_flags is not None:
        vis_flags = sim_walk_flags
        vis_ratio = float(sum(sim_walk_flags) / max(1, len(sim_walk_flags)))
        lost_frame, lost_event = _visibility_lost_counts(sim_walk_flags)
        uv_mask = sim_cam_flags if sim_cam_flags is not None else None
    else:
        vis_flags = perception_flags
        vis_ratio = float(sum(perception_flags) / max(1, len(perception_flags)))
        lost_frame, lost_event = _visibility_lost_counts(perception_flags)
        uv_mask = perception_flags

    u_dedup, v_dedup = _camera_uv_errors(camera, dedupe=True, visibility_flags=uv_mask)
    visible_raw = [int(float(r.get("target_visible") or 0)) for r in camera]
    vis_ratio_raw = float(sum(visible_raw) / max(1, len(visible_raw))) if camera else 0.0
    vis_ratio_perception = float(sum(perception_flags) / max(1, len(perception_flags))) if perception_flags else 0.0
    lost_frame_raw, lost_event_raw = _lost_counts(camera)

    camera_eff = [r for i, r in enumerate(camera) if uv_mask is not None and i < len(uv_mask) and uv_mask[i]]
    merged = _nearest_merge(walking, camera_eff)
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
        "visible_time_ratio_raw": vis_ratio_raw,
        "visible_time_ratio_perception": vis_ratio_perception,
        "visibility_source": "sim_camera" if sim_walk_flags is not None else "perception_heuristic",
        "target_lost_frame_count": lost_frame,
        "target_lost_event_count": lost_event,
        "target_lost_count": lost_frame,
        "target_lost_frame_count_raw": lost_frame_raw,
        "target_lost_event_count_raw": lost_event_raw,
        "u_rms": _rms(u_dedup),
        "v_rms": _rms(v_dedup),
        "u_err_rms": _rms(u_dedup),
        "v_err_rms": _rms(v_dedup),
        "u_mean": _mean(u_dedup),
        "v_mean": _mean(v_dedup),
        "u_std": _std(u_dedup),
        "v_std": _std(v_dedup),
        "u_detrend_rms": _detrend_rms(u_dedup),
        "v_detrend_rms": _detrend_rms(v_dedup),
        "camera_samples_deduped": len(v_dedup),
        "analysis_window_s": win_s,
        "max_abs_u": max((abs(v) for v in u_dedup), default=0.0),
        "max_abs_v": max((abs(v) for v in v_dedup), default=0.0),
        "corr_pitch_v": _corr(pitch_m, v_clean),
        "corr_roll_u": _corr(roll_m, u_clean),
        "walking_rows": len(walking),
        "camera_rows": len(camera),
        "merged_rows": len(merged),
    }
    out_path = log_dir / f"{run_id}_summary.json"
    out_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    if write_plots:
        plot_flags = sim_cam_flags if sim_cam_flags is not None else perception_flags
        _maybe_plot(run_id, log_dir, walking, camera, visibility_flags=plot_flags)
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


def _maybe_plot(
    run_id: str,
    log_dir: Path,
    walking: list[dict],
    camera: list[dict],
    *,
    visibility_flags: Optional[list[int]] = None,
) -> None:
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
        vis = (
            list(visibility_flags)
            if visibility_flags is not None
            else [int(float(r.get("target_visible") or 0)) for r in camera]
        )
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
        plt.ylabel("visible (sim in-frame)")
        plt.xlabel("time [s]")
        plt.tight_layout()
        plt.savefig(plot_dir / "visibility.png")
        plt.close()


def compare_runs(
    run_ids: list[str],
    log_dir: Path,
    *,
    analysis_window_s: Optional[float] = None,
) -> dict[str, Any]:
    summaries = {
        rid: summarize_run(rid, log_dir, write_plots=False, analysis_window_s=analysis_window_s)
        for rid in run_ids
    }
    table = []
    for rid, s in summaries.items():
        table.append(
            {
                "run_id": rid,
                "pitch_rms_deg": s["pitch_rms_deg"],
                "u_rms": s["u_rms"],
                "v_rms": s["v_rms"],
                "v_std": s.get("v_std"),
                "v_detrend_rms": s.get("v_detrend_rms"),
                "visible_time_ratio": s["visible_time_ratio"],
                "visible_time_ratio_raw": s.get("visible_time_ratio_raw"),
                "target_lost_event_count": s["target_lost_event_count"],
                "analysis_window_s": s.get("analysis_window_s"),
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
    ap.add_argument("--analysis-window-s", type=float, default=None,
                    help="Only analyze first N seconds from first wall_time sample (e.g. 10)")
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
        result = compare_runs(
            list(args.compare),
            log_dir,
            analysis_window_s=args.analysis_window_s,
        )
        out = log_dir / "compare_summary.json"
        out.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, indent=2))
        return

    run_id = args.run_id
    if args.latest or not run_id:
        run_id = _find_latest_run_id(log_dir) or ""
    if not run_id:
        raise SystemExit("no run_id found")
    result = summarize_run(
        run_id,
        log_dir,
        write_merged=bool(args.merged),
        analysis_window_s=args.analysis_window_s,
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
