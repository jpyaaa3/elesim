"""Plot independent evaluation evidence; this process may read private truth."""
from __future__ import annotations

import argparse
from itertools import product
import json
from pathlib import Path

import numpy as np


def plot_evidence(directory: Path, output: Path):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    result = json.loads((directory / "evaluation/evaluation.json").read_text())
    with np.load(directory / "produced/data/sensors/view-0007.npz", allow_pickle=False) as packet:
        rgb = packet["color_bgr"][..., ::-1]
    with np.load(directory / "consumer/predictions/view-0007.npz", allow_pickle=False) as packet:
        before, after = packet["points_before"], packet["points_after"]
    with np.load(directory / "produced/data/truth/view-0007.npz", allow_pickle=False) as truth:
        signs = np.array(list(product((-0.5, 0.5), repeat=3)))
        corners = signs * truth["object_size"] @ truth["object_rotation"].T + truth["object_center"]

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5), layout="constrained")
    axes[0].imshow(rgb)
    axes[0].set_title("RGB input (one view)")
    axes[0].axis("off")
    axes[1].scatter(before[:, 0]*1000, before[:, 2]*1000, s=2, alpha=0.5,
                    color="#dd674f", label="Before")
    axes[1].scatter(after[:, 0]*1000, after[:, 2]*1000, s=2, alpha=0.5,
                    color="#159a86", label="After")
    for i in range(8):
        for j in range(i+1, 8):
            if np.count_nonzero(signs[i] != signs[j]) == 1:
                axes[1].plot(corners[[i, j], 0]*1000, corners[[i, j], 2]*1000,
                             color="#343a40", linewidth=1,
                             label="True box" if (i, j) == (0, 1) else None)
    axes[1].axhline(0, color="#aaaaaa", linestyle="--", linewidth=1)
    axes[1].set(xlabel="World X (mm)", ylabel="World Z (mm)",
                title="Observed surface in fixed coordinates", aspect="equal")
    axes[1].legend(fontsize=8)
    labels = ["Tilt + height", "XY + yaw"]
    names = ["observable", "unobservable"]
    x = np.arange(2)
    for offset, key, color, label in ((-.18, "before_p95_m", "#dd674f", "Before"),
                                       (.18, "after_p95_m", "#159a86", "After")):
        values = [result["metrics"][name][key]*1000 for name in names]
        bars = axes[2].bar(x+offset, values, width=.36, color=color, label=label)
        axes[2].bar_label(bars, labels=[f"{v:.1f}" for v in values], padding=3, fontsize=9)
    axes[2].set(xticks=x, xticklabels=labels, ylabel="Point error, p95 (mm)",
                title="Independent object-point error", ylim=(0, 105))
    axes[2].legend(fontsize=8)
    fig.suptitle("Sensor-only reconstruction | Genesis, 3 boxes, 48 views (36 stable)", fontsize=14)
    fig.supxlabel("Known floor, ideal virtual encoders, 0.5 mm synthetic depth noise; no ICP. Physical accuracy is untested.",
                  fontsize=9)
    fig.savefig(output, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    plot_evidence(args.directory, args.output)


if __name__ == "__main__":
    main()
