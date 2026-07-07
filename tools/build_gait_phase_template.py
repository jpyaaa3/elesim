#!/usr/bin/env python3
"""Build gait-phase UV disturbance template from off-mode walking logs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.config_loader import load_app_config_from_ini
from engine.gaze_stabilizer.gait_phase_preview import (
    PHASE_SOURCE_GO2,
    PHASE_SOURCE_SIM,
    PHASE_SOURCE_WALL,
    fill_empty_bins,
    resolve_gait_period_s,
    resolve_gait_phase,
)


def _float_or_none(raw: str) -> Optional[float]:
    s = str(raw).strip()
    if not s:
        return None
    try:
        v = float(s)
        return v if math.isfinite(v) else None
    except ValueError:
        return None


def _load_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        return [], []
    lines = text.splitlines()
    header = lines[0].split(",")
    rows = []
    for line in lines[1:]:
        cols = line.split(",")
        rows.append(dict(zip(header, cols)))
    return header, rows


def _nearest_walking_row(walking_rows: list[dict[str, str]], t_key: str, t_val: float) -> Optional[dict[str, str]]:
    best = None
    best_dt = float("inf")
    for row in walking_rows:
        t = _float_or_none(row.get(t_key, ""))
        if t is None:
            continue
        dt = abs(float(t) - float(t_val))
        if dt < best_dt:
            best_dt = dt
            best = row
    return best


def _merge_camera_walking(
    cam_rows: list[dict[str, str]],
    walking_rows: list[dict[str, str]],
) -> list[dict[str, str]]:
    if not walking_rows:
        return [dict(r) for r in cam_rows]
    merged: list[dict[str, str]] = []
    for cam in cam_rows:
        row = dict(cam)
        sim_t = _float_or_none(cam.get("sim_time_s", "")) or _float_or_none(
            cam.get("host_go2_base_timestamp_s", "")
        )
        wall_t = _float_or_none(cam.get("wall_time_s", "")) or _float_or_none(cam.get("time_s", ""))
        walk = None
        if sim_t is not None:
            walk = _nearest_walking_row(walking_rows, "sim_time_s", sim_t)
        if walk is None and wall_t is not None:
            walk = _nearest_walking_row(walking_rows, "wall_time_s", wall_t)
        if walk is not None:
            for k, v in walk.items():
                if k not in row or not str(row.get(k, "")).strip():
                    row[k] = v
                if k in ("go2_gait_phase", "go2_gait_period_s", "go2_cmd_vx", "sim_time_s"):
                    row[k] = v
        merged.append(row)
    return merged


def _row_phase(
    row: dict[str, str],
    *,
    run_t0: float,
    gait_period_s: float,
    phase_offset: float,
) -> tuple[Optional[float], str]:
    host_phase = _float_or_none(row.get("go2_gait_phase", ""))
    sim_t = _float_or_none(row.get("sim_time_s", "")) or _float_or_none(
        row.get("host_go2_base_timestamp_s", "")
    )
    wall_t = _float_or_none(row.get("wall_time_s", "")) or _float_or_none(row.get("time_s", ""))
    phase, src = resolve_gait_phase(
        host_gait_phase=host_phase,
        sim_time_s=float(sim_t or 0.0),
        wall_time_s=float(wall_t or 0.0),
        wall_t0_s=float(run_t0),
        gait_period_s=float(gait_period_s),
        phase_offset=float(phase_offset),
    )
    return phase, src


def _collect_run_rows(
    run_id: str,
    log_dir: Path,
    *,
    gait_period_s: float,
    phase_offset: float,
    trim_start_s: float,
    trim_end_s: float,
    vx_nominal: float,
    vx_tol: float,
) -> tuple[list[dict[str, Any]], str]:
    cam_path = log_dir / f"{run_id}_camera.csv"
    if not cam_path.is_file():
        raise FileNotFoundError(f"missing camera csv: {cam_path}")
    _, cam_rows = _load_csv(cam_path)
    walk_path = log_dir / f"{run_id}_walking.csv"
    walk_rows: list[dict[str, str]] = []
    if walk_path.is_file():
        _, walk_rows = _load_csv(walk_path)
    merged = _merge_camera_walking(cam_rows, walk_rows)

    wall_times = [
        _float_or_none(r.get("wall_time_s", "")) or _float_or_none(r.get("time_s", "")) for r in merged
    ]
    wall_times = [t for t in wall_times if t is not None]
    if not wall_times:
        return [], PHASE_SOURCE_WALL
    run_t0 = float(min(wall_times))
    run_t1 = float(max(wall_times))
    phase_source = PHASE_SOURCE_WALL

    out: list[dict[str, Any]] = []
    for row in merged:
        wall_t = _float_or_none(row.get("wall_time_s", "")) or _float_or_none(row.get("time_s", ""))
        if wall_t is None:
            continue
        rel = float(wall_t) - run_t0
        if rel < float(trim_start_s):
            continue
        if (run_t1 - run_t0) - rel < float(trim_end_s):
            continue
        if str(row.get("target_visible", "1")).strip() not in ("1", "true", "True"):
            continue
        u_err = _float_or_none(row.get("u_err", ""))
        v_err = _float_or_none(row.get("v_err", ""))
        if u_err is None or v_err is None:
            continue
        vx = _float_or_none(row.get("go2_cmd_vx", ""))
        if vx is not None and abs(float(vx) - float(vx_nominal)) > float(vx_tol):
            continue
        phase, src = _row_phase(
            row,
            run_t0=run_t0,
            gait_period_s=gait_period_s,
            phase_offset=phase_offset,
        )
        if phase is None:
            continue
        if src == PHASE_SOURCE_GO2:
            phase_source = PHASE_SOURCE_GO2
        elif src == PHASE_SOURCE_SIM and phase_source != PHASE_SOURCE_GO2:
            phase_source = PHASE_SOURCE_SIM
        out.append({"phase": float(phase), "u_err": float(u_err), "v_err": float(v_err)})
    return out, phase_source


def build_template(
    *,
    runs: list[str],
    log_dir: Path,
    gait_period_s: float,
    phase_offset: float,
    num_bins: int,
    trim_start_s: float,
    trim_end_s: float,
    vx_nominal: float,
    vx_tol: float,
) -> dict[str, Any]:
    all_rows: list[dict[str, Any]] = []
    phase_source = PHASE_SOURCE_WALL
    for run_id in runs:
        rows, src = _collect_run_rows(
            run_id,
            log_dir,
            gait_period_s=gait_period_s,
            phase_offset=phase_offset,
            trim_start_s=trim_start_s,
            trim_end_s=trim_end_s,
            vx_nominal=vx_nominal,
            vx_tol=vx_tol,
        )
        all_rows.extend(rows)
        if src == PHASE_SOURCE_GO2:
            phase_source = PHASE_SOURCE_GO2
        elif src == PHASE_SOURCE_SIM and phase_source != PHASE_SOURCE_GO2:
            phase_source = PHASE_SOURCE_SIM
    if not all_rows:
        raise SystemExit("no samples after filtering")

    u_mean = float(np.mean([r["u_err"] for r in all_rows]))
    v_mean = float(np.mean([r["v_err"] for r in all_rows]))

    n = int(num_bins)
    u_bins = [[] for _ in range(n)]
    v_bins = [[] for _ in range(n)]
    for row in all_rows:
        b = int(float(row["phase"]) * n) % n
        u_bins[b].append(float(row["u_err"]) - u_mean)
        v_bins[b].append(float(row["v_err"]) - v_mean)

    counts = np.array([len(b) for b in u_bins], dtype=float)
    u_template = np.array([float(np.mean(b)) if b else 0.0 for b in u_bins], dtype=float)
    v_template = np.array([float(np.mean(b)) if b else 0.0 for b in v_bins], dtype=float)
    u_std = np.array([float(np.std(b)) if len(b) > 1 else 0.0 for b in u_bins], dtype=float)
    v_std = np.array([float(np.std(b)) if len(b) > 1 else 0.0 for b in v_bins], dtype=float)

    u_template = fill_empty_bins(u_template, counts)
    v_template = fill_empty_bins(v_template, counts)

    phase_anchor = "sim_time_zero" if phase_source == PHASE_SOURCE_SIM else "run_first_sample"
    if phase_source == PHASE_SOURCE_GO2:
        phase_anchor = "sim_time_zero"

    return {
        "metadata": {
            "phase_source": phase_source,
            "phase_anchor": phase_anchor,
            "gait_period_s": float(gait_period_s),
            "num_bins": int(n),
            "phase_offset": float(phase_offset),
            "trim_start_s": float(trim_start_s),
            "trim_end_s": float(trim_end_s),
            "runs": list(runs),
            "vx_nominal": float(vx_nominal),
            "sample_total": int(len(all_rows)),
        },
        "u_template": [float(x) for x in u_template],
        "v_template": [float(x) for x in v_template],
        "sample_count": [int(c) for c in counts],
        "u_std": [float(x) for x in u_std],
        "v_std": [float(x) for x in v_std],
    }


def _maybe_plot(payload: dict[str, Any], out_path: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return
    meta = payload["metadata"]
    n = int(meta["num_bins"])
    phase = np.linspace(0.0, 1.0, n, endpoint=False) + 0.5 / n
    stem = out_path.with_suffix("")
    counts = np.asarray(payload["sample_count"], dtype=float)

    plt.figure(figsize=(8, 3))
    plt.plot(phase, payload["u_template"], marker="o", ms=3)
    plt.xlabel("gait phase")
    plt.ylabel("u_template")
    plt.title("phase vs u_template")
    plt.tight_layout()
    plt.savefig(f"{stem}_phase_u.png")
    plt.close()

    plt.figure(figsize=(8, 3))
    plt.plot(phase, payload["v_template"], marker="o", ms=3)
    plt.xlabel("gait phase")
    plt.ylabel("v_template")
    plt.title("phase vs v_template")
    plt.tight_layout()
    plt.savefig(f"{stem}_phase_v.png")
    plt.close()

    plt.figure(figsize=(8, 2))
    plt.bar(phase, counts, width=0.9 / n)
    plt.xlabel("gait phase")
    plt.ylabel("sample_count")
    plt.tight_layout()
    plt.savefig(f"{stem}_sample_count.png")
    plt.close()


def main() -> None:
    ap = argparse.ArgumentParser(description="Build gait-phase UV template from off logs")
    ap.add_argument("--config", default="config.ini")
    ap.add_argument("--log-dir", default="logs/walking_baseline")
    ap.add_argument("--runs", nargs="+", required=True)
    ap.add_argument("--gait-period", type=float, default=None)
    ap.add_argument("--num-bins", type=int, default=32)
    ap.add_argument("--phase-offset", type=float, default=0.0)
    ap.add_argument("--trim-start-s", type=float, default=2.0)
    ap.add_argument("--trim-end-s", type=float, default=2.0)
    ap.add_argument("--vx-nominal", type=float, default=0.35)
    ap.add_argument("--vx-tol", type=float, default=0.05)
    ap.add_argument("--output", required=True)
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    bundle = load_app_config_from_ini(args.config)
    gait_period = resolve_gait_period_s(
        gait_period_s=float(args.gait_period or bundle.gaze_stabilizer_config.gait_period_s),
        gait_hz=float(bundle.go2_locomotion_config.gait_hz),
    )
    if gait_period <= 0.0:
        raise SystemExit("invalid gait_period_s; set --gait-period or gait_hz in config")

    runs = list(args.runs)
    log_dir = Path(args.log_dir)
    payload = build_template(
        runs=runs,
        log_dir=log_dir,
        gait_period_s=gait_period,
        phase_offset=float(args.phase_offset),
        num_bins=int(args.num_bins),
        trim_start_s=float(args.trim_start_s),
        trim_end_s=float(args.trim_end_s),
        vx_nominal=float(args.vx_nominal),
        vx_tol=float(args.vx_tol),
    )
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    if not args.no_plots:
        _maybe_plot(payload, out)
    print(f"[build_gait_phase_template] wrote {out} samples={payload['metadata']['sample_total']}")


if __name__ == "__main__":
    main()
