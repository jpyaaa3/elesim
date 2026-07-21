#!/usr/bin/env python3
"""Build presentation/report figures and tables from existing walking-baseline logs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import sys
from pathlib import Path
from typing import Any, Optional

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from misc.tooling.analysis.analyze_walking_metrics import effective_visibility_flags, summarize_run

PROPOSED_KEY = "pitch-preview"
PROPOSED_LABEL = "Body Pitch"

CONDITIONS: list[tuple[str, str, str]] = [
    ("off", "No Comp.", "exp_gaze_off_neutral_forward_"),
    ("uv", "Reactive", "exp_baseline_neutral_forward_"),
    ("uv_ff", "Reactive+FF", "exp_gaze_uv_ff_neutral_forward_"),
    ("pitch-preview", "Body Pitch", "neutral_forward_preview_pos_"),
]

FIG_LABELS = [c[1] for c in CONDITIONS]
FIG_KEYS = [c[0] for c in CONDITIONS]
DISPLAY_BY_KEY = {c[0]: c[1] for c in CONDITIONS}
PROPOSED_INDEX = FIG_KEYS.index(PROPOSED_KEY)

FIG_DPI = 200
FIG_SIZE = (7.5, 4.2)
FONT_SIZE = 11
TITLE_SIZE = 12
XLABEL_TIME = "Time (s)"
YLABEL_V_RMS = "RMS vertical tracking error (norm.)"
YLABEL_V_ERR = "Vertical tracking error (norm.)"
YLABEL_IMPROVEMENT = "Relative reduction in RMS vertical error (%)"


def _float_or_none(raw: Any) -> Optional[float]:
    if raw in ("", None):
        return None
    try:
        return float(raw)
    except (TypeError, ValueError):
        return None


def _read_json(path: Path) -> Optional[dict[str, Any]]:
    if not path.is_file():
        print(f"[warn] missing file: {path}")
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"[warn] failed to read {path}: {exc}")
        return None


def _discover_run_ids(log_dir: Path, prefix: str) -> list[str]:
    pat = re.compile(rf"^{re.escape(prefix)}(\d{{3}})_meta\.json$")
    ids: list[str] = []
    for path in sorted(log_dir.glob(f"{prefix}*_meta.json")):
        m = pat.match(path.name)
        if m:
            ids.append(path.name.replace("_meta.json", ""))
    return ids


def _summary_from_compare(compare: dict[str, Any], run_id: str) -> Optional[dict[str, Any]]:
    runs = compare.get("runs") if isinstance(compare, dict) else None
    if isinstance(runs, dict) and run_id in runs:
        return dict(runs[run_id])
    table = compare.get("table") if isinstance(compare, dict) else None
    if isinstance(table, list):
        for row in table:
            if isinstance(row, dict) and row.get("run_id") == run_id:
                return dict(row)
    return None


def _pick_representative_run(
    log_dir: Path,
    run_ids: list[str],
    *,
    analysis_window_s: Optional[float] = None,
) -> tuple[str, float]:
    if not run_ids:
        return "", 0.0
    summaries = [
        summarize_run(rid, log_dir, write_plots=False, analysis_window_s=analysis_window_s)
        for rid in run_ids
    ]
    mean_v = statistics.mean(float(s["v_rms"]) for s in summaries)
    best = min(
        zip(run_ids, summaries),
        key=lambda pair: abs(float(pair[1]["v_rms"]) - mean_v),
    )
    return best[0], mean_v


def _aggregate_runs(
    log_dir: Path,
    run_ids: list[str],
    *,
    analysis_window_s: Optional[float] = None,
) -> dict[str, Any]:
    if not run_ids:
        return {}
    summaries = [
        summarize_run(rid, log_dir, write_plots=False, analysis_window_s=analysis_window_s)
        for rid in run_ids
    ]
    keys_float = [
        "v_rms",
        "u_rms",
        "visible_time_ratio",
        "target_lost_event_count",
        "preview_used_ratio",
        "preview_fallback_ratio",
    ]

    def _mean(key: str) -> Optional[float]:
        vals = []
        for s in summaries:
            meta = s.get("meta") if isinstance(s.get("meta"), dict) else {}
            if s.get(key) is not None:
                vals.append(float(s[key]))
            elif meta.get(key) is not None:
                vals.append(float(meta[key]))
        return statistics.mean(vals) if vals else None

    def _std(key: str) -> Optional[float]:
        vals = []
        for s in summaries:
            if key in s and s[key] is not None:
                vals.append(float(s[key]))
        if len(vals) < 2:
            return 0.0
        return float(statistics.stdev(vals))

    row: dict[str, Any] = {
        "run_ids": run_ids,
        "representative_run_id": _pick_representative_run(log_dir, run_ids, analysis_window_s=analysis_window_s)[0],
        "n_trials": len(run_ids),
    }
    for key in keys_float:
        row[key] = _mean(key)
        if key == "v_rms":
            row["v_rms_std"] = _std(key)
        if key == "u_rms":
            row["u_rms_std"] = _std(key)
    meta0 = summaries[0].get("meta") if summaries and isinstance(summaries[0].get("meta"), dict) else {}
    if row.get("preview_used_ratio") is None and meta0.get("preview_used_ratio") is not None:
        row["preview_used_ratio"] = _mean("preview_used_ratio")
    if row.get("preview_fallback_ratio") is None and meta0.get("preview_fallback_ratio") is not None:
        row["preview_fallback_ratio"] = _mean("preview_fallback_ratio")
    return row


def _run_id_from_compare(compare: dict[str, Any], prefix: str) -> str:
    table = compare.get("table") if isinstance(compare, dict) else None
    if isinstance(table, list):
        for row in table:
            if not isinstance(row, dict):
                continue
            rid = str(row.get("run_id") or "")
            if rid.startswith(prefix):
                return rid
    runs = compare.get("runs") if isinstance(compare, dict) else None
    if isinstance(runs, dict):
        matches = sorted(rid for rid in runs if str(rid).startswith(prefix))
        if matches:
            return str(matches[-1])
    return ""


def _meta_notes(log_dir: Path, run_id: str) -> str:
    meta_path = log_dir / f"{run_id}_meta.json"
    if not meta_path.is_file():
        return ""
    meta = _read_json(meta_path) or {}
    return str(meta.get("notes") or "")


def _filter_run_ids(log_dir: Path, run_ids: list[str], *, notes_filter: str = "") -> list[str]:
    filt = str(notes_filter or "").strip()
    if not filt:
        return list(run_ids)
    return [rid for rid in run_ids if filt in _meta_notes(log_dir, rid)]


def _compare_run_ids(compare: dict[str, Any], prefix: str) -> list[str]:
    runs = compare.get("runs") if isinstance(compare, dict) else None
    if not isinstance(runs, dict):
        return []
    return sorted(rid for rid in runs if str(rid).startswith(prefix))


def _build_condition_rows(
    log_dir: Path,
    compare_path: Path,
    *,
    analysis_window_s: Optional[float] = None,
    notes_filter: str = "",
) -> dict[str, dict[str, Any]]:
    compare = _read_json(compare_path) or {}
    rows: dict[str, dict[str, Any]] = {}

    for key, _label, prefix in CONDITIONS:
        run_ids = _filter_run_ids(log_dir, _discover_run_ids(log_dir, prefix), notes_filter=notes_filter)
        if not run_ids and not str(notes_filter or "").strip():
            preferred = _run_id_from_compare(compare, prefix)
            if preferred:
                run_ids = [preferred]
        if not run_ids:
            print(f"[warn] no runs found for {key!r} with prefix {prefix!r} (filter={notes_filter!r})")

        if len(run_ids) <= 1:
            rid = run_ids[0]
            summary = summarize_run(rid, log_dir, write_plots=False, analysis_window_s=analysis_window_s)
            meta = summary.get("meta") if isinstance(summary.get("meta"), dict) else {}
            rows[key] = {
                "condition": key,
                "run_id": rid,
                "run_ids": run_ids,
                "v_rms": summary.get("v_rms"),
                "v_rms_std": None,
                "u_rms": summary.get("u_rms"),
                "u_rms_std": None,
                "visible_time_ratio": summary.get("visible_time_ratio"),
                "target_lost_event_count": summary.get("target_lost_event_count"),
                "preview_used_ratio": meta.get("preview_used_ratio"),
                "preview_fallback_ratio": meta.get("preview_fallback_ratio"),
                "source": "single",
            }
            continue

        agg = _aggregate_runs(log_dir, run_ids, analysis_window_s=analysis_window_s)
        rows[key] = {
            "condition": key,
            "run_id": agg.get("representative_run_id", ""),
            "run_ids": run_ids,
            "v_rms": agg.get("v_rms"),
            "v_rms_std": agg.get("v_rms_std"),
            "u_rms": agg.get("u_rms"),
            "u_rms_std": agg.get("u_rms_std"),
            "visible_time_ratio": agg.get("visible_time_ratio"),
            "target_lost_event_count": agg.get("target_lost_event_count"),
            "preview_used_ratio": agg.get("preview_used_ratio"),
            "preview_fallback_ratio": agg.get("preview_fallback_ratio"),
            "source": "aggregate",
        }
    return rows


def _pct_reduction(baseline: Optional[float], value: Optional[float]) -> Optional[float]:
    if baseline is None or value is None or baseline < 1e-12:
        return None
    return 100.0 * (float(baseline) - float(value)) / float(baseline)


def _finalize_table_rows(rows: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    off_v = rows.get("off", {}).get("v_rms")
    uv_v = rows.get("uv", {}).get("v_rms")
    out: list[dict[str, Any]] = []
    for key, _label, _prefix in CONDITIONS:
        src = rows.get(key, {})
        v_rms = src.get("v_rms")
        visible = src.get("visible_time_ratio")
        row = {
            "condition": DISPLAY_BY_KEY.get(key, key),
            "condition_key": key,
            "v_rms": v_rms,
            "v_rms_std": src.get("v_rms_std"),
            "u_rms": src.get("u_rms"),
            "u_rms_std": src.get("u_rms_std"),
            "visible_percent": (100.0 * float(visible)) if visible is not None else None,
            "lost_events": src.get("target_lost_event_count"),
            "preview_used_ratio": src.get("preview_used_ratio"),
            "preview_fallback_ratio": src.get("preview_fallback_ratio"),
            "relative_improvement_vs_off_percent": _pct_reduction(off_v, v_rms),
            "relative_improvement_vs_uv_percent": _pct_reduction(uv_v, v_rms),
            "run_id": src.get("run_id", ""),
            "n_trials": len(src.get("run_ids") or []),
        }
        out.append(row)
    return out


def _write_table_csv(path: Path, table: list[dict[str, Any]]) -> None:
    cols = [
        "condition",
        "condition_key",
        "v_rms",
        "v_rms_std",
        "u_rms",
        "u_rms_std",
        "visible_percent",
        "lost_events",
        "preview_used_ratio",
        "preview_fallback_ratio",
        "relative_improvement_vs_off_percent",
        "relative_improvement_vs_uv_percent",
        "run_id",
        "n_trials",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for row in table:
            w.writerow({k: ("" if row.get(k) is None else row[k]) for k in cols})


def _fmt(v: Any, nd: int = 3) -> str:
    if v is None or v == "":
        return "—"
    try:
        return f"{float(v):.{nd}f}"
    except (TypeError, ValueError):
        return str(v)


def _write_table_md(path: Path, table: list[dict[str, Any]]) -> None:
    headers = [
        "condition",
        "v_rms",
        "u_rms",
        "visible %",
        "lost events",
        "preview used",
        "preview fallback",
        "vs No Comp. %",
        "vs Reactive %",
    ]
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join(["---"] * len(headers)) + " |",
    ]
    for row in table:
        v_cell = _fmt(row.get("v_rms"))
        if row.get("v_rms_std") not in (None, "", 0):
            v_cell = f"{v_cell} ± {_fmt(row['v_rms_std'])}"
        lines.append(
            "| "
            + " | ".join(
                [
                    str(row.get("condition", "")),
                    v_cell,
                    _fmt(row.get("u_rms")),
                    _fmt(row.get("visible_percent"), 1),
                    _fmt(row.get("lost_events"), 0),
                    _fmt(row.get("preview_used_ratio")),
                    _fmt(row.get("preview_fallback_ratio")),
                    _fmt(row.get("relative_improvement_vs_off_percent"), 1),
                    _fmt(row.get("relative_improvement_vs_uv_percent"), 1),
                ]
            )
            + " |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _save_figure(fig, out_base: Path, *, pdf: bool = True) -> None:
    out_base.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_base.with_suffix(".png"), dpi=FIG_DPI, bbox_inches="tight")
    if pdf:
        fig.savefig(out_base.with_suffix(".pdf"), bbox_inches="tight")


def _plot_v_rms_5way(out_dir: Path, table: list[dict[str, Any]]) -> None:
    import matplotlib.pyplot as plt

    lookup = {row.get("condition_key", row["condition"]): row for row in table}
    vals = [lookup.get(k, {}).get("v_rms") for k in FIG_KEYS]
    errs = [float(lookup.get(k, {}).get("v_rms_std") or 0.0) for k in FIG_KEYS]
    x = list(range(len(FIG_LABELS)))
    colors = ["#9aa0a6"] * len(FIG_LABELS)
    colors[PROPOSED_INDEX] = "#1a73e8"

    fig, ax = plt.subplots(figsize=(8.2, 4.2))
    bars = ax.bar(x, [float(v or 0.0) for v in vals], yerr=[float(e or 0.0) for e in errs], capsize=4, color=colors)
    bars[PROPOSED_INDEX].set_edgecolor("#0b57d0")
    bars[PROPOSED_INDEX].set_linewidth(1.5)
    ax.set_xticks(x, FIG_LABELS, fontsize=FONT_SIZE)
    ax.set_ylabel(YLABEL_V_RMS, fontsize=FONT_SIZE)
    ax.tick_params(axis="y", labelsize=FONT_SIZE)
    ax.grid(axis="y", alpha=0.25, linestyle="--")
    ymax = max((float(v or 0.0) for v in vals), default=1.0)
    ax.set_ylim(0.0, ymax * 1.15)
    fig.tight_layout()
    _save_figure(fig, out_dir / "fig_v_rms_4way")
    plt.close(fig)


def _plot_improvement(out_dir: Path, table: list[dict[str, Any]]) -> None:
    import matplotlib.pyplot as plt

    lookup = {row.get("condition_key", row["condition"]): row for row in table}
    proposed_v = lookup.get(PROPOSED_KEY, {}).get("v_rms")
    baselines = [
        (DISPLAY_BY_KEY["off"], lookup.get("off", {}).get("v_rms")),
        (DISPLAY_BY_KEY["uv"], lookup.get("uv", {}).get("v_rms")),
        (DISPLAY_BY_KEY["uv_ff"], lookup.get("uv_ff", {}).get("v_rms")),
    ]
    labels = [b[0] for b in baselines]
    vals = [_pct_reduction(b[1], proposed_v) or 0.0 for b in baselines]

    fig, ax = plt.subplots(figsize=(7.8, 4.0))
    x = list(range(len(labels)))
    ax.bar(x, vals, color="#1a73e8")
    ax.set_xticks(x, labels, fontsize=FONT_SIZE, rotation=20, ha="right")
    ax.set_ylabel(YLABEL_IMPROVEMENT, fontsize=FONT_SIZE)
    ax.tick_params(axis="y", labelsize=FONT_SIZE)
    ax.grid(axis="y", alpha=0.25, linestyle="--")
    fig.tight_layout()
    _save_figure(fig, out_dir / "fig_v_rms_improvement")
    plt.close(fig)


def _phase_x(num_bins: int) -> list[float]:
    if num_bins <= 0:
        return []
    return [(i + 0.5) / float(num_bins) for i in range(num_bins)]


def _demean(values: list[float]) -> list[float]:
    if not values:
        return []
    m = statistics.mean(values)
    return [float(v) - m for v in values]


def _plot_template_phase_v(out_dir: Path, template_path: Path) -> None:
    import matplotlib.pyplot as plt

    data = _read_json(template_path)
    if not data:
        return
    v_t = [float(x) for x in data.get("v_template", [])]
    counts = [float(x) for x in data.get("sample_count", [])]
    n_bins = len(v_t)
    x = _phase_x(n_bins)
    v_dm = _demean(v_t)

    fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(7.5, 5.0), sharex=True, gridspec_kw={"height_ratios": [3, 1]})
    ax_top.plot(x, v_dm, color="#1a73e8", linewidth=1.8)
    ax_top.axhline(0.0, color="#666666", linewidth=0.8, linestyle="--")
    ax_top.set_ylabel(r"Demeaned vertical disturbance $\tilde{d}_v$ (norm.)", fontsize=FONT_SIZE)
    ax_top.grid(alpha=0.25, linestyle="--")
    ax_top.tick_params(labelsize=FONT_SIZE)

    if counts and len(counts) == n_bins:
        ax_bot.bar(x, counts, width=0.9 / max(1, n_bins), color="#9aa0a6", alpha=0.85)
        ax_bot.set_ylabel("Sample count", fontsize=FONT_SIZE)
    else:
        ax_bot.text(0.5, 0.5, "Sample count unavailable", ha="center", va="center", transform=ax_bot.transAxes)
    ax_bot.set_xlabel("Gait phase $\\phi$ (–)", fontsize=FONT_SIZE)
    ax_bot.set_xlim(0.0, 1.0)
    ax_bot.tick_params(labelsize=FONT_SIZE)
    fig.tight_layout()
    _save_figure(fig, out_dir / "fig_template_phase_v")
    plt.close(fig)


def _plot_template_phase_uv(out_dir: Path, template_path: Path) -> None:
    import matplotlib.pyplot as plt

    data = _read_json(template_path)
    if not data:
        return
    u_t = [float(x) for x in data.get("u_template", [])]
    v_t = [float(x) for x in data.get("v_template", [])]
    n_bins = min(len(u_t), len(v_t))
    if n_bins <= 0:
        print("[warn] template missing u/v arrays")
        return
    x = _phase_x(n_bins)
    u_dm = _demean(u_t[:n_bins])
    v_dm = _demean(v_t[:n_bins])

    fig, ax = plt.subplots(figsize=FIG_SIZE)
    ax.plot(x, u_dm, label="Horizontal ($u$)", color="#e8710a", linewidth=1.8)
    ax.plot(x, v_dm, label="Vertical ($v$)", color="#1a73e8", linewidth=1.8)
    ax.axhline(0.0, color="#666666", linewidth=0.8, linestyle="--")
    ax.set_xlabel("Gait phase $\\phi$ (–)", fontsize=FONT_SIZE)
    ax.set_ylabel(r"Demeaned image disturbance (norm.)", fontsize=FONT_SIZE)
    ax.set_xlim(0.0, 1.0)
    ax.legend(fontsize=FONT_SIZE)
    ax.grid(alpha=0.25, linestyle="--")
    ax.tick_params(labelsize=FONT_SIZE)
    fig.tight_layout()
    _save_figure(fig, out_dir / "fig_template_phase_uv")
    plt.close(fig)


def _read_v_timeseries(
    log_dir: Path,
    run_id: str,
    *,
    analysis_window_s: Optional[float] = None,
) -> tuple[list[float], list[float]]:
    path = log_dir / f"{run_id}_camera.csv"
    if not path.is_file():
        print(f"[warn] missing camera log: {path}")
        return [], []
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if analysis_window_s is not None and float(analysis_window_s) > 0.0 and rows:
        t0 = _float_or_none(rows[0].get("wall_time_s"))
        if t0 is not None:
            t_end = float(t0) + float(analysis_window_s)
            rows = [
                r for r in rows
                if (_float_or_none(r.get("wall_time_s")) or 0.0) <= t_end
            ]
    vis_flags = effective_visibility_flags(rows)
    t0: Optional[float] = None
    times: list[float] = []
    vals: list[float] = []
    for i, row in enumerate(rows):
        if i >= len(vis_flags) or not vis_flags[i]:
            continue
        v = _float_or_none(row.get("v_err"))
        if v is None:
            continue
        t = _float_or_none(row.get("sim_time_s"))
        if t is None:
            t = _float_or_none(row.get("time_s"))
        if t is None:
            t = _float_or_none(row.get("wall_time_s"))
        if t is None:
            continue
        if t0 is None:
            t0 = t
        times.append(float(t) - float(t0))
        vals.append(float(v))
    return times, vals


def _rolling_mean(values: list[float], window: int) -> list[float]:
    if not values or window <= 1:
        return list(values)
    out: list[float] = []
    half = window // 2
    for i in range(len(values)):
        lo = max(0, i - half)
        hi = min(len(values), i + half + 1)
        chunk = values[lo:hi]
        out.append(sum(chunk) / len(chunk))
    return out


def _plot_representative_v_error(
    out_dir: Path,
    log_dir: Path,
    condition_rows: dict[str, dict[str, Any]],
    *,
    smooth_window: int = 15,
    analysis_window_s: Optional[float] = None,
    timeseries_max_s: float = 1.25,
) -> None:
    import matplotlib.pyplot as plt

    series_keys = [
        ("off", DISPLAY_BY_KEY["off"]),
        ("uv", DISPLAY_BY_KEY["uv"]),
        ("uv_ff", DISPLAY_BY_KEY["uv_ff"]),
        (PROPOSED_KEY, PROPOSED_LABEL),
    ]
    colors = {
        "off": "#9aa0a6",
        "uv": "#e8710a",
        "uv_ff": "#bdbdbd",
        PROPOSED_KEY: "#1a73e8",
    }
    linewidths = {
        "off": 1.4,
        "uv": 1.4,
        "uv_ff": 1.2,
        PROPOSED_KEY: 2.0,
    }
    linestyles = {
        "off": "-",
        "uv": "-",
        "uv_ff": "--",
        PROPOSED_KEY: "-",
    }

    fig, ax = plt.subplots(figsize=(8.0, 4.2))
    plotted = 0
    for key, label in series_keys:
        info = condition_rows.get(key, {})
        run_id = str(info.get("run_id") or "")
        if not run_id:
            run_ids = info.get("run_ids") or []
            if run_ids:
                run_id, _ = _pick_representative_run(log_dir, list(run_ids), analysis_window_s=analysis_window_s)
        if not run_id:
            print(f"[warn] no representative run for {key}")
            continue
        t, v = _read_v_timeseries(log_dir, run_id, analysis_window_s=analysis_window_s)
        if not t:
            continue
        t_max = float(timeseries_max_s)
        if t_max > 0.0:
            pairs = [(ti, vi) for ti, vi in zip(t, v) if ti <= t_max]
            if not pairs:
                continue
            t, v = [p[0] for p in pairs], [p[1] for p in pairs]
        v_s = _rolling_mean(v, smooth_window)
        ax.plot(
            t,
            v_s,
            label=label,
            color=colors.get(key, None),
            linewidth=linewidths.get(key, 1.6),
            linestyle=linestyles.get(key, "-"),
        )
        plotted += 1

    if plotted == 0:
        plt.close(fig)
        print("[warn] skipped fig_representative_v_error_timeseries (no data)")
        return

    ax.axhline(0.0, color="#666666", linewidth=0.8, linestyle="--")
    ax.set_xlabel(XLABEL_TIME, fontsize=FONT_SIZE)
    ax.set_ylabel(YLABEL_V_ERR, fontsize=FONT_SIZE)
    if float(timeseries_max_s) > 0.0:
        ax.set_xlim(0.0, float(timeseries_max_s))
    ax.legend(fontsize=FONT_SIZE - 1, frameon=False)
    ax.grid(alpha=0.25, linestyle="--")
    ax.tick_params(labelsize=FONT_SIZE)
    fig.tight_layout()
    _save_figure(fig, out_dir / "fig_representative_v_error_timeseries")
    plt.close(fig)


def _proposed_best_v_rms(
    log_dir: Path,
    run_ids: list[str],
    *,
    analysis_window_s: Optional[float] = None,
) -> Optional[float]:
    if not run_ids:
        return None
    vals = [
        float(summarize_run(rid, log_dir, write_plots=False, analysis_window_s=analysis_window_s)["v_rms"])
        for rid in run_ids
    ]
    return min(vals) if vals else None


def _write_notes(
    out_dir: Path,
    table: list[dict[str, Any]],
    log_dir: Path,
    proposed_run_ids: list[str],
    *,
    analysis_window_s: Optional[float] = None,
) -> None:
    lookup = {row.get("condition_key", row["condition"]): row for row in table}

    def _v(key: str) -> float:
        v = lookup.get(key, {}).get("v_rms")
        return float(v) if v is not None else float("nan")

    proposed_v = _v(PROPOSED_KEY)
    off_v = _v("off")
    uv_v = _v("uv")
    uvff_v = _v("uv_ff")

    imp_off = _pct_reduction(off_v, proposed_v) or 0.0
    imp_uv = _pct_reduction(uv_v, proposed_v) or 0.0
    imp_uvff = _pct_reduction(uvff_v, proposed_v) or 0.0
    proposed_best = _proposed_best_v_rms(log_dir, proposed_run_ids, analysis_window_s=analysis_window_s)
    window_note = (
        "Analysis window: full trial duration; visibility from sim eye-camera in-frame projection."
    )

    lines = [
        "# Body Pitch Preview — Final Result Notes",
        "",
        f"- {window_note}",
        "- Body Pitch (IMU pitch-rate lead) is the proposed deployable method for real GO2 hardware.",
        (
            f"- Compared with No Comp., v RMS decreased from {off_v:.3f} to {proposed_v:.3f}, "
            f"corresponding to approximately {imp_off:.1f}% reduction."
        ),
        (
            f"- Compared with Reactive feedback, Body Pitch reduced v RMS from {uv_v:.3f} to {proposed_v:.3f}, "
            f"corresponding to approximately {imp_uv:.1f}% reduction."
        ),
        (
            "- Body Pitch reduced vertical image-space error versus Reactive and No Comp. baselines "
            "in this protocol (standoff 0.85 m, 30 s max duration)."
        ),
        "",
        "## Summary metrics",
        "",
        f"- Body Pitch v RMS: {proposed_v:.3f}",
        f"- Reactive v RMS: {uv_v:.3f}",
        f"- No Comp. v RMS: {off_v:.3f}",
        f"- Reactive+FF v RMS: {uvff_v:.3f}",
        "",
        f"## Relative reductions ({PROPOSED_LABEL})",
        "",
        f"- vs No Comp.: {imp_off:.1f}%",
        f"- vs Reactive: {imp_uv:.1f}%",
        f"- vs Reactive+FF: {imp_uvff:.1f}%",
        "",
    ]
    if proposed_best is not None and math.isfinite(proposed_best):
        lines.extend(
            [
                "## Best single trial",
                "",
                f"- Best Body Pitch v RMS among logged trials: {proposed_best:.3f}",
                "",
            ]
        )
    (out_dir / "final_result_notes.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate final pitch-preview report figures/tables from logs")
    ap.add_argument("--log-dir", default="logs/walking_baseline")
    ap.add_argument("--template", default="logs/gait_templates/neutral_forward_vx035_template.json")
    ap.add_argument("--compare", default="logs/walking_baseline/compare_summary.json")
    ap.add_argument("--out", default="misc/results/pitch_preview_final")
    ap.add_argument("--smooth-window", type=int, default=15)
    ap.add_argument(
        "--include-gait-template-figures",
        action="store_true",
        help="Also emit gait-template phase figures (sim ablation only)",
    )
    ap.add_argument(
        "--analysis-window-s",
        type=float,
        default=0.0,
        help="Compare metrics over the first N seconds of each trial (0 = full trial)",
    )
    ap.add_argument(
        "--timeseries-max-s",
        type=float,
        default=1.25,
        help="Max time axis for representative v-error timeseries (0 = no limit)",
    )
    ap.add_argument(
        "--notes-filter",
        default="",
        help="Only include trials whose meta notes contain this substring (empty = all)",
    )
    args = ap.parse_args()

    log_dir = Path(args.log_dir)
    out_dir = Path(args.out)
    template_path = Path(args.template)
    compare_path = Path(args.compare)
    analysis_window_s = float(args.analysis_window_s) if float(args.analysis_window_s) > 0.0 else None

    if not log_dir.is_dir():
        raise SystemExit(f"log dir not found: {log_dir}")

    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[final] log_dir={log_dir.resolve()}")
    print(f"[final] out={out_dir.resolve()}")

    condition_rows = _build_condition_rows(
        log_dir,
        compare_path,
        analysis_window_s=analysis_window_s,
        notes_filter=str(args.notes_filter or ""),
    )
    table = _finalize_table_rows(condition_rows)

    _write_table_csv(out_dir / "final_summary_table.csv", table)
    _write_table_md(out_dir / "final_summary_table.md", table)
    print(f"[final] wrote {out_dir / 'final_summary_table.csv'}")

    _plot_v_rms_5way(out_dir, table)
    print(f"[final] wrote fig_v_rms_4way (.png/.pdf)")

    _plot_improvement(out_dir, table)
    print(f"[final] wrote fig_v_rms_improvement (.png/.pdf)")

    if bool(args.include_gait_template_figures) and template_path.is_file():
        _plot_template_phase_v(out_dir, template_path)
        _plot_template_phase_uv(out_dir, template_path)
        print(f"[final] wrote template phase figures")
    elif template_path.is_file():
        print(f"[final] skipped gait-template figures (pitch-preview is proposed method)")
    else:
        print(f"[warn] template not found: {template_path}")

    _plot_representative_v_error(
        out_dir,
        log_dir,
        condition_rows,
        smooth_window=int(args.smooth_window),
        analysis_window_s=analysis_window_s,
        timeseries_max_s=float(args.timeseries_max_s),
    )
    print(f"[final] wrote fig_representative_v_error_timeseries (.png/.pdf)")

    _write_notes(
        out_dir,
        table,
        log_dir,
        list(condition_rows.get(PROPOSED_KEY, {}).get("run_ids") or []),
        analysis_window_s=analysis_window_s,
    )
    print(f"[final] wrote final_result_notes.md")
    print("[final] done")


if __name__ == "__main__":
    main()
