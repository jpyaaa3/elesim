#!/usr/bin/env python3
"""Offline + optional live sweep for gait-phase preview tuning."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Optional

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.gaze_stabilizer.gait_phase_preview import GaitPhasePreviewModel, resolve_gait_phase


def _float_or_none(raw: Any) -> Optional[float]:
    if raw in ("", None):
        return None
    try:
        v = float(raw)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _read_csv(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    with path.open(encoding="utf-8", newline="") as fh:
        return list(csv.DictReader(fh))


def _time_column(row: dict[str, Any]) -> float:
    for key in ("wall_time_s", "time_s", "sim_time_s"):
        t = _float_or_none(row.get(key))
        if t is not None:
            return float(t)
    return 0.0


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


def _rms(vals: Iterable[float]) -> float:
    xs = [float(v) for v in vals]
    if not xs:
        return float("inf")
    return math.sqrt(sum(v * v for v in xs) / len(xs))


def _discover_run_ids(log_dir: Path, prefix: str, *, notes_filter: str = "") -> list[str]:
    pat = re.compile(rf"^{re.escape(prefix)}(\d{{3}})_meta\.json$")
    ids: list[str] = []
    for path in sorted(log_dir.glob(f"{prefix}*_meta.json")):
        m = pat.match(path.name)
        if not m:
            continue
        rid = path.name.replace("_meta.json", "")
        if notes_filter:
            meta_path = log_dir / path.name
            try:
                meta = json.loads(meta_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                continue
            if notes_filter not in str(meta.get("notes") or ""):
                continue
        ids.append(rid)
    return ids


def load_merged_frames(log_dir: Path, run_id: str) -> list[dict[str, Any]]:
    walking = _read_csv(log_dir / f"{run_id}_walking.csv")
    camera = _read_csv(log_dir / f"{run_id}_camera.csv")
    merged = _nearest_merge(walking, camera)
    out: list[dict[str, Any]] = []
    for row in merged:
        v_err = _float_or_none(row.get("v_err"))
        if v_err is None:
            continue
        vis = row.get("sim_target_in_frame")
        if vis not in ("", None) and int(float(vis)) == 0:
            continue
        phase = _float_or_none(row.get("go2_gait_phase"))
        period = _float_or_none(row.get("go2_gait_period_s"))
        sim_t = _float_or_none(row.get("sim_time_s"))
        if phase is None and sim_t is not None and period and period > 0.0:
            phase = (float(sim_t) % float(period)) / float(period)
        if phase is None:
            continue
        out.append(
            {
                "v_err": float(v_err),
                "u_err": float(_float_or_none(row.get("u_err")) or 0.0),
                "phase": float(phase) % 1.0,
                "period_s": float(period or 0.0),
                "sim_time_s": float(sim_t or 0.0),
            }
        )
    return out


@dataclass(frozen=True)
class SweepParams:
    phase_offset: float
    horizon_s: float
    scale: float

    def tag(self) -> str:
        off = int(round(self.phase_offset * 100.0))
        hz = int(round(self.horizon_s * 1000.0))
        sc = int(round(self.scale * 100.0))
        sign = "m" if off < 0 else "p"
        return f"off{sign}{abs(off):03d}_hz{hz:03d}_sc{sc:03d}"


@dataclass(frozen=True)
class SweepScore:
    params: SweepParams
    proxy_v_rms: float
    proxy_gain: float
    align_corr: float
    n_frames: int

    def as_dict(self) -> dict[str, Any]:
        return {
            "phase_offset": self.params.phase_offset,
            "horizon_s": self.params.horizon_s,
            "scale": self.params.scale,
            "tag": self.params.tag(),
            "proxy_v_rms": self.proxy_v_rms,
            "proxy_gain": self.proxy_gain,
            "align_corr": self.align_corr,
            "n_frames": self.n_frames,
        }


def _preview_terms(
    model: GaitPhasePreviewModel,
    frames: list[dict[str, Any]],
    params: SweepParams,
    *,
    period_s: float,
) -> tuple[list[float], list[float]]:
    terms_v: list[float] = []
    terms_u: list[float] = []
    for fr in frames:
        phase_now, _ = resolve_gait_phase(
            host_gait_phase=float(fr["phase"]),
            sim_time_s=float(fr.get("sim_time_s") or 0.0),
            wall_time_s=0.0,
            wall_t0_s=0.0,
            gait_period_s=float(period_s),
            phase_offset=float(params.phase_offset),
        )
        if phase_now is None:
            continue
        delta = model.preview_delta(
            phase_now,
            scale=float(params.scale),
            horizon_s=float(params.horizon_s),
            period_s=float(period_s),
        )
        if not delta.ok:
            continue
        terms_u.append(float(delta.preview_term[0]))
        terms_v.append(float(delta.preview_term[1]))
    return terms_u, terms_v


def score_params(
    model: GaitPhasePreviewModel,
    frames: list[dict[str, Any]],
    params: SweepParams,
    *,
    period_s: float,
) -> Optional[SweepScore]:
    if not frames:
        return None
    v_errs = [float(fr["v_err"]) for fr in frames]
    terms_u: list[float] = []
    terms_v: list[float] = []
    aligned_v: list[float] = []
    for fr in frames:
        phase_now, _ = resolve_gait_phase(
            host_gait_phase=float(fr["phase"]),
            sim_time_s=float(fr.get("sim_time_s") or 0.0),
            wall_time_s=0.0,
            wall_t0_s=0.0,
            gait_period_s=float(period_s),
            phase_offset=float(params.phase_offset),
        )
        if phase_now is None:
            continue
        delta = model.preview_delta(
            phase_now,
            scale=float(params.scale),
            horizon_s=float(params.horizon_s),
            period_s=float(period_s),
        )
        if not delta.ok:
            continue
        terms_u.append(float(delta.preview_term[0]))
        terms_v.append(float(delta.preview_term[1]))
        aligned_v.append(float(fr["v_err"]))

    n = min(len(aligned_v), len(terms_v))
    if n < 20:
        return None
    aligned_v = aligned_v[:n]
    terms_v = terms_v[:n]

    denom = sum(t * t for t in terms_v)
    if denom < 1e-12:
        return None
    gain = -sum(v * t for v, t in zip(aligned_v, terms_v)) / denom
    residual = [v + gain * t for v, t in zip(aligned_v, terms_v)]
    proxy = _rms(residual)

    mv = statistics.mean(aligned_v)
    mt = statistics.mean(terms_v)
    num = sum((aligned_v[i] - mv) * (terms_v[i] - mt) for i in range(n))
    dv = math.sqrt(sum((aligned_v[i] - mv) ** 2 for i in range(n)))
    dt = math.sqrt(sum((terms_v[i] - mt) ** 2 for i in range(n)))
    corr = float(num / (dv * dt)) if dv > 1e-12 and dt > 1e-12 else 0.0

    return SweepScore(
        params=params,
        proxy_v_rms=float(proxy),
        proxy_gain=float(gain),
        align_corr=float(corr),
        n_frames=int(n),
    )


def _parse_grid(raw: str) -> list[float]:
    vals = []
    for part in str(raw).split(","):
        part = part.strip()
        if not part:
            continue
        vals.append(float(part))
    return vals


def run_offline_sweep(
    *,
    log_dir: Path,
    template_path: Path,
    run_ids: list[str],
    phase_offsets: list[float],
    horizons: list[float],
    scales: list[float],
) -> list[SweepScore]:
    model = GaitPhasePreviewModel.load(template_path)
    period_s = float(model.template.gait_period_s)
    if period_s <= 0.0:
        raise SystemExit(f"invalid template gait_period_s in {template_path}")

    frames: list[dict[str, Any]] = []
    for rid in run_ids:
        frames.extend(load_merged_frames(log_dir, rid))
    if not frames:
        raise SystemExit("no merged frames for sweep")

    scored: list[SweepScore] = []
    for off in phase_offsets:
        for hz in horizons:
            for sc in scales:
                row = score_params(
                    model,
                    frames,
                    SweepParams(phase_offset=float(off), horizon_s=float(hz), scale=float(sc)),
                    period_s=period_s,
                )
                if row is not None:
                    scored.append(row)
    scored.sort(key=lambda r: (r.proxy_v_rms, -abs(r.align_corr)))
    return scored


def _patch_config(
    src: Path,
    dst: Path,
    *,
    phase_offset: float,
    horizon_s: float,
    scale: float,
    gait_period_s: float,
) -> None:
    text = src.read_text(encoding="utf-8")
    repl = {
        r"^gaze_gait_phase_offset\s*=\s*[-+0-9.]+": f"gaze_gait_phase_offset = {phase_offset}",
        r"^gaze_gait_preview_horizon_s\s*=\s*[-+0-9.]+": f"gaze_gait_preview_horizon_s = {horizon_s}",
        r"^gaze_gait_preview_scale\s*=\s*[-+0-9.]+": f"gaze_gait_preview_scale = {scale}",
        r"^gaze_gait_period_s\s*=\s*[-+0-9.]+": f"gaze_gait_period_s = {gait_period_s}",
        r"^gaze_gait_preview_enable\s*=\s*\w+": "gaze_gait_preview_enable = true",
        r"^gaze_preview_enable\s*=\s*\w+": "gaze_preview_enable = true",
    }
    for pat, val in repl.items():
        text = re.sub(pat, val, text, count=1, flags=re.M)
    # Keep runtime asset paths valid when config is not loaded from repo root.
    root = src.resolve().parent
    text = re.sub(
        r"^hand_eye_config\s*=\s*(.+)$",
        lambda m: f"hand_eye_config = {(root / m.group(1).strip()).resolve()}",
        text,
        count=1,
        flags=re.M,
    )
    text = re.sub(
        r"^gait_gait_template_path\s*=\s*(.+)$",
        lambda m: f"gait_gait_template_path = {(root / m.group(1).strip()).resolve()}",
        text,
        count=1,
        flags=re.M,
    )
    text = re.sub(
        r"^gaze_gait_template_path\s*=\s*(.+)$",
        lambda m: f"gaze_gait_template_path = {(root / m.group(1).strip()).resolve()}",
        text,
        count=1,
        flags=re.M,
    )
    dst.write_text(text, encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser(description="Sweep gait-phase preview params")
    ap.add_argument("--log-dir", default="logs/walking_baseline")
    ap.add_argument("--template", default="logs/gait_templates/neutral_forward_vx035_template.json")
    ap.add_argument("--run-prefix", default="neutral_forward_gait_preview_")
    ap.add_argument("--notes-filter", default="v8")
    ap.add_argument("--phase-offsets", default="-0.20,-0.15,-0.10,-0.05,0.0,0.05,0.10,0.15,0.20")
    ap.add_argument("--horizons", default="0.05,0.06,0.08,0.10,0.12,0.14")
    ap.add_argument("--scales", default="0.6,0.8,1.0,1.2,1.4,1.6")
    ap.add_argument("--top-k", type=int, default=10)
    ap.add_argument("--out", default="results/gait_preview_tune/sweep_offline.json")
    ap.add_argument("--write-top-configs", action="store_true")
    ap.add_argument("--source-config", default="config.ini")
    ap.add_argument("--config-out-dir", default="results/gait_preview_tune/configs")
    args = ap.parse_args()

    log_dir = Path(args.log_dir)
    run_ids = _discover_run_ids(log_dir, str(args.run_prefix), notes_filter=str(args.notes_filter))
    if not run_ids:
        raise SystemExit(f"no runs found for prefix={args.run_prefix!r} notes_filter={args.notes_filter!r}")

    scored = run_offline_sweep(
        log_dir=log_dir,
        template_path=Path(args.template),
        run_ids=run_ids,
        phase_offsets=_parse_grid(args.phase_offsets),
        horizons=_parse_grid(args.horizons),
        scales=_parse_grid(args.scales),
    )
    if not scored:
        raise SystemExit("sweep produced no scores")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "run_ids": run_ids,
        "n_combos_scored": len(scored),
        "baseline_default": next(
            (
                s.as_dict()
                for s in scored
                if abs(s.params.phase_offset) < 1e-9
                and abs(s.params.horizon_s - 0.08) < 1e-9
                and abs(s.params.scale - 1.0) < 1e-9
            ),
            None,
        ),
        "top": [s.as_dict() for s in scored[: max(1, int(args.top_k))]],
    }
    out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    print(f"[sweep] runs={len(run_ids)} frames scored across {len(scored)} combos")
    print(f"[sweep] wrote {out_path}")
    print("[sweep] top candidates (lower proxy_v_rms is better):")
    for i, row in enumerate(scored[: max(1, int(args.top_k))], start=1):
        p = row.params
        print(
            f"  {i:02d}. off={p.phase_offset:+.2f} hz={p.horizon_s:.3f} sc={p.scale:.2f} "
            f"proxy_v_rms={row.proxy_v_rms:.4f} corr={row.align_corr:+.3f} gain={row.proxy_gain:+.3f}"
        )

    if args.write_top_configs:
        model = GaitPhasePreviewModel.load(Path(args.template))
        period_s = float(model.template.gait_period_s)
        src = Path(args.source_config)
        out_dir = Path(args.config_out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        for row in scored[: max(1, int(args.top_k))]:
            p = row.params
            dst = out_dir / f"gait_tune_{p.tag()}.ini"
            _patch_config(
                src,
                dst,
                phase_offset=p.phase_offset,
                horizon_s=p.horizon_s,
                scale=p.scale,
                gait_period_s=period_s,
            )
            print(f"[sweep] config {dst}")


if __name__ == "__main__":
    main()
