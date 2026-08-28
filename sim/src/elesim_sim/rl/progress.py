"""Has this training run stopped improving?

Three tests, all read off a train log:

* the reverse curriculum has walked the start point back to Home,
* the success rate has stopped moving,
* the wrap angle has stopped moving.

A run that passes all three has nothing left to gain from more iterations.
What remains is a configuration or task change: measured limits like the coil
not reaching a 80 mm cylinder, or the position jitter not covering +/-60 mm, do
not yield to a longer run.

The comparison is between two adjacent windows rather than against a peak, so a
noisy plateau reads as flat instead of as decline.  Windows are in iterations,
and an iteration means the same amount of policy movement regardless of how many
environments produced it -- PPO takes the same number of gradient steps either
way, and the adaptive KL schedule caps how far the policy can move per step.
So the thresholds do not need rescaling for env count.

Run::

    python -m elesim_sim.rl.progress ~/elesim/logs/srv_v4.log
    python -m elesim_sim.rl.progress sim/rl_runs/eval/stage2_srv_v4/curve.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
from pathlib import Path
from typing import Optional, Sequence

#: Change over the comparison window counted as flat.
SUCCESS_EPS = 0.005          # fraction, so half a point
PHI_EPS = 0.05               # radians, about 3 degrees


def series_from_log(text: str, key: str) -> list[float]:
    """Every value logged under `key`, in order."""
    return [
        float(m.group(1))
        for m in re.finditer(rf"{re.escape(key)}:\s+([-\d.]+)", text)
    ]


def series_from_csv(path: Path) -> dict[str, list[float]]:
    """The same three series out of a `watch_eval` curve.

    The curve is per-checkpoint rather than per-iteration, so its windows are
    coarser; the thresholds are the same because they are about the size of the
    change, not how many samples produced it.
    """
    rows: list[dict[str, str]] = []
    with path.open(newline="", encoding="utf-8") as fh:
        rows = [r for r in csv.DictReader(fh) if (r.get("iteration") or "").isdigit()]
    rows.sort(key=lambda r: int(r["iteration"]))
    return {
        "iteration": [float(r["iteration"]) for r in rows],
        "success": [float(r["success_rate"]) for r in rows],
        "phi": [math.radians(float(r["phi_mean_deg"])) for r in rows],
        "curriculum": [],
    }


def read(path: Path) -> dict[str, list[float]]:
    if path.suffix.lower() == ".csv":
        return series_from_csv(path)
    text = path.read_text(errors="replace")
    return {
        "iteration": [
            float(i + 1) for i in range(len(re.findall(r"Iteration time:", text)))
        ],
        "success": series_from_log(text, "term/success"),
        "phi": series_from_log(text, "wrap/phi_rad"),
        "curriculum": series_from_log(text, "curriculum/start_t_hi"),
    }


def verdict(data: dict[str, list[float]], window: int) -> tuple[str, list[str]]:
    """A recommendation, and the lines explaining it."""
    lines: list[str] = []
    success, phi, curr = data["success"], data["phi"], data["curriculum"]
    n = min(len(success), len(phi))
    lines.append(f"기록된 지점 {n}")

    curriculum_done = True
    if curr:
        t = curr[-1]
        curriculum_done = t <= 1e-6
        lines.append(
            f"  커리큘럼 t_hi   {t:.2f}   "
            + ("끝남 (Home 에서 시작 중)" if curriculum_done
               else "아직 물러나는 중 — 계속 돌려야 함")
        )
    else:
        lines.append("  커리큘럼 t_hi   기록 없음 (평가 곡선에는 없는 값)")

    if n < 2 * window:
        lines.append(f"  판정에는 {2 * window} 지점이 필요합니다 (지금 {n})")
        return "계속 — 아직 판정할 만큼 모이지 않았습니다", lines

    def mean(v: list[float], a: int, b: int) -> float:
        return statistics.mean(v[a:b])

    s_prev, s_now = mean(success, n - 2 * window, n - window), mean(success, n - window, n)
    p_prev, p_now = mean(phi, n - 2 * window, n - window), mean(phi, n - window, n)
    s_flat = abs(s_now - s_prev) < SUCCESS_EPS
    p_flat = abs(p_now - p_prev) < PHI_EPS
    lines.append(
        f"  성공률   {s_prev:.4f} -> {s_now:.4f}   변화 {(s_now - s_prev) * 100:+.2f}%p"
        f"   {'평평' if s_flat else '아직 움직임'}"
    )
    lines.append(
        f"  Φ        {p_prev:.3f} -> {p_now:.3f} rad   변화 {p_now - p_prev:+.3f}"
        f"   {'평평' if p_flat else '아직 움직임'}"
    )

    if not curriculum_done:
        return "계속 — 커리큘럼이 아직 Home 까지 물러나지 않았습니다", lines
    if s_flat and p_flat:
        return "멈춰도 됩니다 — 더 돌려도 얻을 것이 없습니다", lines
    return "계속 — 아직 개선 중입니다", lines


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", help="train log, or a watch_eval curve.csv")
    parser.add_argument(
        "--window", type=int, default=None,
        help="iterations per comparison window (default: 500 for a log, 3 for a curve)",
    )
    args = parser.parse_args(argv)

    path = Path(args.path).expanduser()
    if not path.is_file():
        raise SystemExit(f"파일이 없습니다: {path}")
    data = read(path)
    window = args.window or (3 if path.suffix.lower() == ".csv" else 500)
    call, lines = verdict(data, window)
    for line in lines:
        print(line)
    print()
    print(f"  => {call}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
