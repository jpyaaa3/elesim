#!/usr/bin/env python3
"""Merge walking/camera CSV logs and emit summary statistics."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any


def _read_csv(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return float(math.sqrt(sum(v * v for v in values) / len(values)))


def _nearest_merge(walking: list[dict], camera: list[dict], *, max_dt: float = 0.05) -> list[dict]:
    if not walking or not camera:
        return []
    cam_times = [float(r["time_s"]) for r in camera]
    out: list[dict] = []
    for w in walking:
        tw = float(w["time_s"])
        j = min(range(len(cam_times)), key=lambda i: abs(cam_times[i] - tw))
        if abs(cam_times[j] - tw) > max_dt:
            continue
        row = dict(w)
        row.update(camera[j])
        out.append(row)
    return out


def summarize_run(run_id: str, log_dir: Path) -> dict[str, Any]:
    walking = _read_csv(log_dir / f"{run_id}_walking.csv")
    camera = _read_csv(log_dir / f"{run_id}_camera.csv")
    meta_path = log_dir / f"{run_id}_meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.is_file() else {}

    pitch = [math.degrees(float(r["base_pitch"])) for r in walking if r.get("base_pitch")]
    u_err = [float(r["u_err"]) for r in camera if r.get("u_err")]
    v_err = [float(r["v_err"]) for r in camera if r.get("v_err")]
    lost = int(camera[-1]["target_lost_count"]) if camera else 0

    summary = {
        "run_id": run_id,
        "meta": meta,
        "pitch_rms_deg": _rms(pitch),
        "u_err_rms": _rms(u_err),
        "v_err_rms": _rms(v_err),
        "target_lost_count": lost,
        "walking_rows": len(walking),
        "camera_rows": len(camera),
    }
    out_path = log_dir / f"{run_id}_summary.json"
    out_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    _maybe_plot(run_id, log_dir, walking, camera)
    return summary


def _maybe_plot(run_id: str, log_dir: Path, walking: list[dict], camera: list[dict]) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return
    plot_dir = log_dir / f"{run_id}_plots"
    plot_dir.mkdir(parents=True, exist_ok=True)
    if walking:
        t = [float(r["time_s"]) for r in walking]
        pitch = [math.degrees(float(r["base_pitch"])) for r in walking]
        plt.figure(figsize=(8, 3))
        plt.plot(t, pitch)
        plt.ylabel("pitch [deg]")
        plt.xlabel("time [s]")
        plt.title(f"{run_id} base pitch")
        plt.tight_layout()
        plt.savefig(plot_dir / "pitch.png")
        plt.close()
    if camera:
        t = [float(r["time_s"]) for r in camera]
        u = [float(r["u_err"]) for r in camera]
        v = [float(r["v_err"]) for r in camera]
        plt.figure(figsize=(8, 3))
        plt.plot(t, u, label="u_err")
        plt.plot(t, v, label="v_err")
        plt.legend()
        plt.xlabel("time [s]")
        plt.title(f"{run_id} UV error")
        plt.tight_layout()
        plt.savefig(plot_dir / "uv_err.png")
        plt.close()


def compare_runs(run_a: str, run_b: str, log_dir: Path) -> dict[str, Any]:
    sa = summarize_run(run_a, log_dir)
    sb = summarize_run(run_b, log_dir)
    return {"a": sa, "b": sb, "delta": {
        "pitch_rms_deg": sb["pitch_rms_deg"] - sa["pitch_rms_deg"],
        "u_err_rms": sb["u_err_rms"] - sa["u_err_rms"],
        "v_err_rms": sb["v_err_rms"] - sa["v_err_rms"],
        "target_lost_count": sb["target_lost_count"] - sa["target_lost_count"],
    }}


def main() -> None:
    ap = argparse.ArgumentParser(description="Analyze walking baseline CSV logs")
    ap.add_argument("run_id")
    ap.add_argument("--log-dir", default="logs/walking_baseline")
    ap.add_argument("--compare", default="", help="second run_id for ON/OFF comparison")
    args = ap.parse_args()
    log_dir = Path(args.log_dir)
    if args.compare:
        result = compare_runs(args.run_id, args.compare, log_dir)
    else:
        result = summarize_run(args.run_id, log_dir)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
